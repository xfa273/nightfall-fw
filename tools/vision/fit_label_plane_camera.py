#!/usr/bin/env python3
"""Fit camera/plane geometry from stationary, known micromouse poses.

The ordinary marker homography maps every image point onto the marker reference
plane.  Elevated blue/red tracking labels therefore appear displaced.  This
tool uses stationary placements whose true blue-centre position and heading are
known to recover the camera centre and height needed by
``label_plane_geometry.py``.

The fit is deliberately separate from turn data.  Four non-collinear poses are
used for fitting and at least one held-out pose is required before the emitted
artifact can declare itself safety-qualified.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np

import board_layout
import board_metric_geometry
import camera_capture_fingerprint
import label_plane_geometry


MANIFEST_SCHEMA = "nightfall_label_plane_known_pose_manifest_v3"
CALIBRATION_SCHEMA = "nightfall_label_plane_camera_fit_v1"
SAFETY_MINIMUM_FIT_SPAN_MM = 400.0
SAFETY_MINIMUM_FIT_HULL_AREA_MM2 = 120000.0
SAFETY_MINIMUM_HOLD_DURATION_S = 1.5
SAFETY_MINIMUM_DISTINCT_POSITION_MM = 90.0
SAFETY_MAXIMUM_STABILITY_P95_MM = 0.75
SAFETY_MAXIMUM_FIT_P95_MM = 1.0
SAFETY_MAXIMUM_VALIDATION_P95_MM = 1.0
SAFETY_MAXIMUM_VALIDATION_ERROR_MM = 1.5
SAFETY_MINIMUM_CAMERA_HEIGHT_MM = 100.0
SAFETY_MAXIMUM_CAMERA_HEIGHT_MM = 3000.0


@dataclass(frozen=True)
class Placement:
    placement_id: str
    role: str
    apparent_blue_xy_mm: tuple[float, float]
    apparent_red_xy_mm: tuple[float, float]
    true_blue_xy_mm: tuple[float, float]
    true_red_xy_mm: tuple[float, float]
    source: dict[str, Any]
    stability_p95_mm: float
    stationary_duration_s: float


def _validate_capture_setups(
    captures: Sequence[
        camera_capture_fingerprint.CameraCaptureFingerprint
    ],
) -> None:
    if not captures:
        raise ValueError("at least one placement capture_session is required")
    if any(not capture.safety_qualified for capture in captures):
        raise ValueError(
            "every placement capture_session must be safety-qualified"
        )
    camera_setup_digests = {
        capture.camera_setup_sha256 for capture in captures
    }
    if None in camera_setup_digests or len(camera_setup_digests) != 1:
        raise ValueError(
            "all stationary placements must use the same fixed camera setup"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit full-board blue/red label-height correction from stationary "
            "known-pose trajectory CSVs."
        )
    )
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-fit-span-mm", type=float, default=400.0)
    parser.add_argument("--minimum-fit-hull-area-mm2", type=float, default=150000.0)
    parser.add_argument("--minimum-hold-duration-s", type=float, default=1.5)
    parser.add_argument("--maximum-stability-p95-mm", type=float, default=0.75)
    parser.add_argument("--maximum-fit-p95-mm", type=float, default=1.0)
    parser.add_argument("--maximum-validation-p95-mm", type=float, default=1.0)
    parser.add_argument("--maximum-validation-error-mm", type=float, default=1.5)
    parser.add_argument("--minimum-camera-height-mm", type=float, default=100.0)
    parser.add_argument("--maximum-camera-height-mm", type=float, default=3000.0)
    parser.add_argument("--huber-delta-mm", type=float, default=0.5)
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{field} must be a finite number")
    return result


def _pair(value: Any, field: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{field} must be a two-element array")
    return (_finite(value[0], f"{field}[0]"), _finite(value[1], f"{field}[1]"))


def _reference_plane_provenance(value: Any) -> dict[str, str]:
    if not isinstance(value, dict):
        raise ValueError("reference_plane_provenance must be an object")
    result: dict[str, str] = {}
    for field in ("surface", "measurement", "confirmed_date", "remeasure_if"):
        item = value.get(field)
        if not isinstance(item, str) or not item.strip():
            raise ValueError(
                f"reference_plane_provenance.{field} must be non-empty"
            )
        result[field] = item
    if value.get("height_reference") != "maze floor":
        raise ValueError(
            "reference_plane_provenance.height_reference must be maze floor"
        )
    result["height_reference"] = "maze floor"
    return result


def _resolve(base: Path, value: Any, field: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a non-empty path")
    path = Path(value)
    if not path.is_absolute():
        path = base / path
    path = path.resolve()
    if not path.is_file():
        raise ValueError(f"{field} does not exist: {path}")
    return path


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid {label} JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError(f"{label} root must be an object")
    return raw


def _verify_source_board_calibration(
    path: Path,
    *,
    expected_board_sha256: str,
    expected_canonical_size: int,
) -> None:
    raw = _load_json(path, "source board calibration")
    if raw.get("schema") != "nightfall_markerless_board_calibration_v3":
        raise ValueError("source board calibration schema is invalid")
    try:
        digest = str(raw["measured_layout"]["sha256"]).lower()
        canonical_size = int(raw["canonical"]["size_px"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("source board calibration bindings are invalid") from exc
    if digest != expected_board_sha256.lower():
        raise ValueError("source board calibration uses a different layout")
    if canonical_size != expected_canonical_size:
        raise ValueError("source board calibration uses a different canonical size")


def _convex_hull(
    points: Sequence[tuple[float, float]],
) -> list[tuple[float, float]]:
    unique = sorted(set(points))
    if len(unique) < 3:
        return unique

    def cross(
        origin: tuple[float, float],
        first: tuple[float, float],
        second: tuple[float, float],
    ) -> float:
        return (first[0] - origin[0]) * (second[1] - origin[1]) - (
            first[1] - origin[1]
        ) * (second[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _polygon_area(points: Sequence[tuple[float, float]]) -> float:
    if len(points) < 3:
        return 0.0
    return 0.5 * abs(
        sum(
            first[0] * second[1] - second[0] * first[1]
            for first, second in zip(points, (*points[1:], points[0]))
        )
    )


def _inside_convex_hull(
    point: tuple[float, float],
    hull: Sequence[tuple[float, float]],
) -> bool:
    if len(hull) < 3:
        return False
    signs: list[float] = []
    for first, second in zip(hull, (*hull[1:], hull[0])):
        signs.append(
            (second[0] - first[0]) * (point[1] - first[1])
            - (second[1] - first[1]) * (point[0] - first[0])
        )
    return all(value >= -1e-9 for value in signs) or all(
        value <= 1e-9 for value in signs
    )


def _canonical_to_board(
    xy_px: np.ndarray,
    layout: board_layout.BoardLayout,
) -> np.ndarray:
    bounds = layout.raw["canvas_bounds_mm"]
    x_min = float(bounds["x_min"])
    y_max = float(bounds["y_max"])
    return np.column_stack(
        [
            x_min + xy_px[:, 0] / layout.pixels_per_mm,
            y_max - xy_px[:, 1] / layout.pixels_per_mm,
        ]
    )


def _trajectory_label_centres(
    path: Path,
    layout: board_layout.BoardLayout,
    metric_geometry: Optional[board_metric_geometry.BoardMetricGeometry] = None,
    start_s: Optional[float] = None,
    end_s: Optional[float] = None,
) -> tuple[np.ndarray, np.ndarray, float, int, float]:
    blue_px: list[tuple[float, float]] = []
    red_px: list[tuple[float, float]] = []
    selected_times: list[float] = []
    with path.open(newline="", encoding="ascii") as stream:
        reader = csv.DictReader(stream)
        required = {
            "label_x_px",
            "label_y_px",
            "front_label_x_px",
            "front_label_y_px",
            "pose_valid",
            "heading_valid",
        }
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f"{path} is missing raw blue/red label coordinates")
        for row in reader:
            try:
                time_s = float(row.get("video_pts_s", row.get("time_s", "nan")))
            except (TypeError, ValueError):
                continue
            if not math.isfinite(time_s):
                continue
            if start_s is not None and time_s < start_s:
                continue
            if end_s is not None and time_s > end_s:
                continue
            try:
                pose_valid = float(row["pose_valid"])
                heading_valid = float(row["heading_valid"])
            except (TypeError, ValueError):
                continue
            if (
                not math.isfinite(pose_valid)
                or not math.isfinite(heading_valid)
                or pose_valid <= 0.5
                or heading_valid <= 0.5
            ):
                continue
            try:
                blue = (float(row["label_x_px"]), float(row["label_y_px"]))
                red = (
                    float(row["front_label_x_px"]),
                    float(row["front_label_y_px"]),
                )
            except (TypeError, ValueError):
                continue
            if all(math.isfinite(value) for value in (*blue, *red)):
                blue_px.append(blue)
                red_px.append(red)
                selected_times.append(time_s)
    if len(blue_px) < 100:
        raise ValueError(f"{path} has fewer than 100 simultaneous label samples")
    if metric_geometry is None:
        blue_mm = _canonical_to_board(np.asarray(blue_px), layout)
        red_mm = _canonical_to_board(np.asarray(red_px), layout)
    else:
        blue_mm = metric_geometry.map_points(np.asarray(blue_px, dtype=float))
        red_mm = metric_geometry.map_points(np.asarray(red_px, dtype=float))
    blue_median = np.median(blue_mm, axis=0)
    red_median = np.median(red_mm, axis=0)
    blue_distance = np.linalg.norm(blue_mm - blue_median, axis=1)
    red_distance = np.linalg.norm(red_mm - red_median, axis=1)
    stability = float(
        max(np.percentile(blue_distance, 95), np.percentile(red_distance, 95))
    )
    duration = float(max(selected_times) - min(selected_times))
    return blue_median, red_median, stability, len(blue_px), duration


def _tracking_values(raw: dict[str, Any]) -> tuple[float, float, float]:
    if raw.get("schema") != "nightfall_machine_footprint_v1":
        raise ValueError("tracking geometry must use nightfall_machine_footprint_v1")
    try:
        labels = raw["tracking_labels"]
        blue_height = _finite(
            labels["blue_centre"]["surface_height_mm"],
            "tracking_labels.blue_centre.surface_height_mm",
        )
        red_height = _finite(
            labels["red_front"]["surface_height_mm"],
            "tracking_labels.red_front.surface_height_mm",
        )
        distance = _finite(
            labels["red_front"]["horizontal_distance_from_blue_centre_mm"],
            "tracking_labels.red_front.horizontal_distance_from_blue_centre_mm",
        )
    except (KeyError, TypeError) as exc:
        raise ValueError("tracking-label geometry fields are invalid") from exc
    if blue_height < 0.0 or red_height < 0.0 or distance <= 0.0:
        raise ValueError("tracking-label heights must be non-negative and distance positive")
    return blue_height, red_height, distance


def load_placements(
    manifest_path: Path,
) -> tuple[
    dict[str, Any],
    Path,
    Path,
    board_layout.BoardLayout,
    float,
    float,
    float,
    float,
    list[Placement],
    camera_capture_fingerprint.CameraCaptureFingerprint,
]:
    manifest = _load_json(manifest_path, "known-pose manifest")
    if manifest.get("schema") != MANIFEST_SCHEMA:
        raise ValueError(f"manifest schema must be {MANIFEST_SCHEMA}")
    base = manifest_path.resolve().parent
    reference_provenance = _reference_plane_provenance(
        manifest.get("reference_plane_provenance")
    )
    board_path = _resolve(base, manifest.get("board_layout"), "board_layout")
    tracking_path = _resolve(
        base, manifest.get("tracking_geometry"), "tracking_geometry"
    )
    canonical_size = int(_finite(manifest.get("canonical_size_px", 900), "canonical_size_px"))
    layout = board_layout.load(board_path, canonical_size)
    board_digest = sha256_file(board_path)
    metric_path: Optional[Path] = None
    metric_geometry: Optional[board_metric_geometry.BoardMetricGeometry] = None
    if manifest.get("board_metric_geometry") is not None:
        metric_path = _resolve(
            base,
            manifest.get("board_metric_geometry"),
            "board_metric_geometry",
        )
        metric_geometry = board_metric_geometry.load_geometry(
            metric_path,
            expected_board_layout_sha256=board_digest,
            expected_canonical_size_px=canonical_size,
            require_safety_qualified=True,
        )
    if metric_geometry is None:
        raise ValueError(
            "board_metric_geometry is required before label-plane calibration"
        )
    tracking = _load_json(tracking_path, "tracking geometry")
    blue_height, red_height, front_distance = _tracking_values(tracking)
    reference_height = _finite(
        manifest.get("reference_plane_absolute_height_mm"),
        "reference_plane_absolute_height_mm",
    )
    if reference_height < 0.0:
        raise ValueError("reference-plane height must be non-negative")
    if metric_geometry is not None and not math.isclose(
        reference_height,
        metric_geometry.reference_plane_absolute_height_mm,
        rel_tol=0.0,
        abs_tol=1e-6,
    ):
        raise ValueError(
            "known-pose reference-plane height does not match "
            "board_metric_geometry"
        )
    items = manifest.get("placements")
    if not isinstance(items, list) or not items:
        raise ValueError("placements must be a non-empty array")
    placements: list[Placement] = []
    captures: list[
        camera_capture_fingerprint.CameraCaptureFingerprint
    ] = []
    ids: set[str] = set()
    for index, item in enumerate(items):
        field = f"placements[{index}]"
        if not isinstance(item, dict):
            raise ValueError(f"{field} must be an object")
        placement_id = item.get("id")
        if not isinstance(placement_id, str) or not placement_id:
            raise ValueError(f"{field}.id must be a non-empty string")
        if placement_id in ids:
            raise ValueError(f"duplicate placement id: {placement_id}")
        ids.add(placement_id)
        role = item.get("role")
        if role not in ("fit", "validation"):
            raise ValueError(f"{field}.role must be fit or validation")
        capture = camera_capture_fingerprint.load_capture_session(
            base,
            item.get("capture_session"),
        )
        if not capture.safety_qualified:
            raise ValueError(
                f"{field}.capture_session is legacy/unverified and cannot "
                "fit safety-qualified geometry"
            )
        captures.append(capture)
        true_blue = _pair(item.get("true_blue_center_mm"), f"{field}.true_blue_center_mm")
        heading_deg = _finite(item.get("true_heading_deg"), f"{field}.true_heading_deg")
        start_s = (
            None
            if item.get("start_s") is None
            else _finite(item.get("start_s"), f"{field}.start_s")
        )
        end_s = (
            None
            if item.get("end_s") is None
            else _finite(item.get("end_s"), f"{field}.end_s")
        )
        if start_s is not None and end_s is not None and end_s <= start_s:
            raise ValueError(f"{field}.end_s must be greater than start_s")
        heading = math.radians(heading_deg)
        true_red = (
            true_blue[0] + front_distance * math.cos(heading),
            true_blue[1] + front_distance * math.sin(heading),
        )
        trajectory = _resolve(base, item.get("trajectory_csv"), f"{field}.trajectory_csv")
        if trajectory.resolve() != capture.artifact("trajectory_csv").path:
            raise ValueError(
                f"{field}.trajectory_csv is not the capture_session trajectory"
            )
        source_calibration_value = item.get("source_calibration_json")
        source_calibration = (
            trajectory.with_name("calibration.json")
            if source_calibration_value is None
            else _resolve(
                base,
                source_calibration_value,
                f"{field}.source_calibration_json",
            )
        )
        if not source_calibration.is_file():
            raise ValueError(
                f"{field}.source_calibration_json does not exist: "
                f"{source_calibration}"
            )
        if (
            source_calibration.resolve()
            != capture.artifact("source_board_calibration").path
        ):
            raise ValueError(
                f"{field}.source_calibration_json is not the capture_session "
                "board calibration"
            )
        _verify_source_board_calibration(
            source_calibration,
            expected_board_sha256=board_digest,
            expected_canonical_size=canonical_size,
        )
        blue, red, stability, sample_count, duration = _trajectory_label_centres(
            trajectory, layout, metric_geometry, start_s, end_s
        )
        placements.append(
            Placement(
                placement_id=placement_id,
                role=role,
                apparent_blue_xy_mm=(float(blue[0]), float(blue[1])),
                apparent_red_xy_mm=(float(red[0]), float(red[1])),
                true_blue_xy_mm=true_blue,
                true_red_xy_mm=true_red,
                source={
                    "trajectory_csv": str(trajectory),
                    "sha256": sha256_file(trajectory),
                    "samples": sample_count,
                    "true_heading_deg": heading_deg,
                    "start_s": start_s,
                    "end_s": end_s,
                    "stationary_duration_s": duration,
                    "source_calibration_json": str(source_calibration.resolve()),
                    "source_calibration_sha256": sha256_file(source_calibration),
                },
                stability_p95_mm=stability,
                stationary_duration_s=duration,
            )
        )
    _validate_capture_setups(captures)
    for first_index, first in enumerate(placements):
        for second in placements[first_index + 1 :]:
            if first.source["trajectory_csv"] != second.source["trajectory_csv"]:
                continue
            first_start = (
                -math.inf
                if first.source["start_s"] is None
                else float(first.source["start_s"])
            )
            first_end = (
                math.inf
                if first.source["end_s"] is None
                else float(first.source["end_s"])
            )
            second_start = (
                -math.inf
                if second.source["start_s"] is None
                else float(second.source["start_s"])
            )
            second_end = (
                math.inf
                if second.source["end_s"] is None
                else float(second.source["end_s"])
            )
            if max(first_start, second_start) <= min(first_end, second_end):
                raise ValueError(
                    "placement intervals overlap in the same trajectory: "
                    f"{first.placement_id} and {second.placement_id}"
                )
    return (
        manifest,
        board_path,
        tracking_path,
        layout,
        reference_height,
        blue_height,
        red_height,
        front_distance,
        placements,
        captures,
    )


def _fit_rows(
    placements: Sequence[Placement],
    reference_height_mm: float,
    blue_height_mm: float,
    red_height_mm: float,
) -> tuple[np.ndarray, np.ndarray, list[tuple[int, str]]]:
    rows: list[list[float]] = []
    values: list[float] = []
    labels: list[tuple[int, str]] = []
    for placement_index, placement in enumerate(placements):
        for label, apparent, physical, absolute_height in (
            (
                "blue",
                placement.apparent_blue_xy_mm,
                placement.true_blue_xy_mm,
                blue_height_mm,
            ),
            (
                "red",
                placement.apparent_red_xy_mm,
                placement.true_red_xy_mm,
                red_height_mm,
            ),
        ):
            relative_height = absolute_height - reference_height_mm
            qx, qy = apparent
            px, py = physical
            rows.append([relative_height * qx, -relative_height, 0.0])
            values.append(qx - px)
            rows.append([relative_height * qy, 0.0, -relative_height])
            values.append(qy - py)
            labels.extend(((placement_index, label), (placement_index, label)))
    return np.asarray(rows), np.asarray(values), labels


def fit_camera(
    placements: Sequence[Placement],
    reference_height_mm: float,
    blue_height_mm: float,
    red_height_mm: float,
    huber_delta_mm: float,
) -> tuple[label_plane_geometry.LabelPlaneGeometry, float]:
    fit = [item for item in placements if item.role == "fit"]
    if len(fit) < 4:
        raise ValueError("at least four fit placements are required")
    design, values, labels = _fit_rows(
        fit, reference_height_mm, blue_height_mm, red_height_mm
    )
    if np.linalg.matrix_rank(design) < 3:
        raise ValueError("known-pose calibration geometry is rank deficient")
    weights = np.ones(len(values), dtype=float)
    parameters = np.zeros(3, dtype=float)
    for _ in range(30):
        root_weight = np.sqrt(weights)
        candidate, _, _, _ = np.linalg.lstsq(
            design * root_weight[:, None], values * root_weight, rcond=None
        )
        residual = design @ candidate - values
        point_norm: dict[tuple[int, str], float] = {}
        for row_index in range(0, len(residual), 2):
            point_norm[labels[row_index]] = float(
                math.hypot(residual[row_index], residual[row_index + 1])
            )
        updated = np.asarray(
            [
                min(1.0, huber_delta_mm / max(point_norm[label], 1e-12))
                for label in labels
            ]
        )
        parameters = candidate
        if float(np.max(np.abs(updated - weights))) < 1e-9:
            break
        weights = updated
    inverse_height = float(parameters[0])
    if inverse_height <= 0.0:
        raise ValueError("fit produced a non-positive camera height")
    relative_camera_height = 1.0 / inverse_height
    camera_x = float(parameters[1] / inverse_height)
    camera_y = float(parameters[2] / inverse_height)
    camera_height = reference_height_mm + relative_camera_height
    condition = float(np.linalg.cond(design))
    return (
        label_plane_geometry.LabelPlaneGeometry(
            camera_center_x_mm=camera_x,
            camera_center_y_mm=camera_y,
            camera_height_mm=camera_height,
            reference_plane_height_mm=reference_height_mm,
            blue_label_height_mm=blue_height_mm,
            red_label_height_mm=red_height_mm,
            front_label_distance_mm=0.0,
            board_layout_sha256="0" * 64,
            tracking_calibration_sha256="0" * 64,
            safety_qualified=False,
        ),
        condition,
    )


def _errors(
    placements: Sequence[Placement],
    geometry: label_plane_geometry.LabelPlaneGeometry,
) -> list[float]:
    result: list[float] = []
    for placement in placements:
        blue = geometry.correct_blue(placement.apparent_blue_xy_mm)
        red = geometry.correct_red(placement.apparent_red_xy_mm)
        result.extend(
            (
                math.dist(blue, placement.true_blue_xy_mm),
                math.dist(red, placement.true_red_xy_mm),
            )
        )
    return result


def _stats(values: Sequence[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "median": None, "p95": None, "max": None}
    array = np.asarray(values, dtype=float)
    return {
        "count": int(len(array)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def main() -> int:
    args = parse_args()
    try:
        for name in (
            "minimum_fit_span_mm",
            "minimum_fit_hull_area_mm2",
            "minimum_hold_duration_s",
            "maximum_stability_p95_mm",
            "maximum_fit_p95_mm",
            "maximum_validation_p95_mm",
            "maximum_validation_error_mm",
            "minimum_camera_height_mm",
            "maximum_camera_height_mm",
            "huber_delta_mm",
        ):
            value = _finite(getattr(args, name), name)
            if value <= 0.0:
                raise ValueError(f"--{name.replace('_', '-')} must be positive")
        if args.maximum_camera_height_mm <= args.minimum_camera_height_mm:
            raise ValueError("maximum camera height must exceed minimum")
        hard_limits = (
            (
                args.minimum_fit_span_mm
                >= SAFETY_MINIMUM_FIT_SPAN_MM,
                "--minimum-fit-span-mm cannot weaken the 400 mm safety floor",
            ),
            (
                args.minimum_fit_hull_area_mm2
                >= SAFETY_MINIMUM_FIT_HULL_AREA_MM2,
                "--minimum-fit-hull-area-mm2 cannot weaken the safety floor",
            ),
            (
                args.minimum_hold_duration_s
                >= SAFETY_MINIMUM_HOLD_DURATION_S,
                "--minimum-hold-duration-s cannot weaken the safety floor",
            ),
            (
                args.maximum_stability_p95_mm
                <= SAFETY_MAXIMUM_STABILITY_P95_MM,
                "--maximum-stability-p95-mm cannot weaken the safety ceiling",
            ),
            (
                args.maximum_fit_p95_mm <= SAFETY_MAXIMUM_FIT_P95_MM,
                "--maximum-fit-p95-mm cannot weaken the safety ceiling",
            ),
            (
                args.maximum_validation_p95_mm
                <= SAFETY_MAXIMUM_VALIDATION_P95_MM,
                "--maximum-validation-p95-mm cannot weaken the safety ceiling",
            ),
            (
                args.maximum_validation_error_mm
                <= SAFETY_MAXIMUM_VALIDATION_ERROR_MM,
                "--maximum-validation-error-mm cannot weaken the safety ceiling",
            ),
            (
                args.minimum_camera_height_mm
                >= SAFETY_MINIMUM_CAMERA_HEIGHT_MM,
                "--minimum-camera-height-mm cannot weaken the safety floor",
            ),
            (
                args.maximum_camera_height_mm
                <= SAFETY_MAXIMUM_CAMERA_HEIGHT_MM,
                "--maximum-camera-height-mm cannot weaken the safety ceiling",
            ),
        )
        for valid, message in hard_limits:
            if not valid:
                raise ValueError(message)
        (
            manifest,
            board_path,
            tracking_path,
            _layout,
            reference_height,
            blue_height,
            red_height,
            front_distance,
            placements,
            captures,
        ) = load_placements(args.manifest.resolve())
        reference_provenance = _reference_plane_provenance(
            manifest.get("reference_plane_provenance")
        )
        metric_path = (
            _resolve(
                args.manifest.resolve().parent,
                manifest.get("board_metric_geometry"),
                "board_metric_geometry",
            )
            if manifest.get("board_metric_geometry") is not None
            else None
        )
        metric_geometry = (
            board_metric_geometry.load_geometry(
                metric_path,
                expected_board_layout_sha256=sha256_file(board_path),
                expected_canonical_size_px=int(
                    _finite(manifest.get("canonical_size_px", 900), "canonical_size_px")
                ),
                require_safety_qualified=True,
            )
            if metric_path is not None
            else None
        )
        fit = [item for item in placements if item.role == "fit"]
        validation = [item for item in placements if item.role == "validation"]
        if not validation:
            raise ValueError("at least one held-out validation placement is required")
        x_span = float(np.ptp([item.true_blue_xy_mm[0] for item in fit]))
        y_span = float(np.ptp([item.true_blue_xy_mm[1] for item in fit]))
        fit_points = [item.true_blue_xy_mm for item in fit]
        fit_hull = _convex_hull(fit_points)
        fit_hull_area = _polygon_area(fit_hull)
        fitted, condition = fit_camera(
            placements,
            reference_height,
            blue_height,
            red_height,
            args.huber_delta_mm,
        )
        board_digest = sha256_file(board_path)
        tracking_digest = sha256_file(tracking_path)
        geometry = label_plane_geometry.LabelPlaneGeometry(
            camera_center_x_mm=fitted.camera_center_x_mm,
            camera_center_y_mm=fitted.camera_center_y_mm,
            camera_height_mm=fitted.camera_height_mm,
            reference_plane_height_mm=reference_height,
            blue_label_height_mm=blue_height,
            red_label_height_mm=red_height,
            front_label_distance_mm=front_distance,
            board_layout_sha256=board_digest,
            tracking_calibration_sha256=tracking_digest,
            safety_qualified=False,
        )
        fit_stats = _stats(_errors(fit, geometry))
        validation_stats = _stats(_errors(validation, geometry))
        maximum_stability = max(item.stability_p95_mm for item in placements)
        minimum_hold_duration = min(
            item.stationary_duration_s for item in placements
        )
        failures: list[str] = []
        if x_span < args.minimum_fit_span_mm or y_span < args.minimum_fit_span_mm:
            failures.append("fit placements do not span the required board area")
        if fit_hull_area < args.minimum_fit_hull_area_mm2:
            failures.append("fit placement convex-hull area is too small")
        for first_index, first in enumerate(fit_points):
            for second in fit_points[first_index + 1 :]:
                if math.dist(first, second) < SAFETY_MINIMUM_DISTINCT_POSITION_MM:
                    failures.append("fit placements are not spatially distinct")
                    break
        for item in validation:
            if not _inside_convex_hull(item.true_blue_xy_mm, fit_hull):
                failures.append("held-out placement lies outside the fit hull")
            if min(math.dist(item.true_blue_xy_mm, point) for point in fit_points) < (
                SAFETY_MINIMUM_DISTINCT_POSITION_MM
            ):
                failures.append("held-out placement duplicates a fit position")
        if minimum_hold_duration < args.minimum_hold_duration_s:
            failures.append("stationary hold duration is shorter than required")
        if maximum_stability > args.maximum_stability_p95_mm:
            failures.append("stationary label stability exceeds the limit")
        if float(fit_stats["p95"]) > args.maximum_fit_p95_mm:
            failures.append("fit residual p95 exceeds the limit")
        if float(validation_stats["p95"]) > args.maximum_validation_p95_mm:
            failures.append("held-out residual p95 exceeds the limit")
        if float(validation_stats["max"]) > args.maximum_validation_error_mm:
            failures.append("held-out maximum residual exceeds the limit")
        if not (
            args.minimum_camera_height_mm
            <= geometry.camera_height_mm - reference_height
            <= args.maximum_camera_height_mm
        ):
            failures.append("camera height is outside the configured physical range")
        if not math.isfinite(condition) or condition > 1.0e8:
            failures.append("fit design is ill-conditioned")
        qualified = not failures
        artifact = {
            "schema": label_plane_geometry.SCHEMA,
            "coordinate_system": label_plane_geometry.COORDINATE_SYSTEM,
            "camera_center_board_mm": {
                "x": geometry.camera_center_x_mm,
                "y": geometry.camera_center_y_mm,
                "height": geometry.camera_height_mm,
            },
            "reference_plane_absolute_height_mm": reference_height,
            "label_absolute_heights_mm": {
                "blue_center": blue_height,
                "red_front": red_height,
            },
            "front_label_distance_mm": front_distance,
            "bindings": {
                "board_layout_sha256": board_digest,
                "board_metric_geometry": (
                    {
                        "path": str(metric_path),
                        "sha256": sha256_file(metric_path),
                        "reference_plane_absolute_height_mm": (
                            metric_geometry.reference_plane_absolute_height_mm
                        ),
                    }
                    if metric_path is not None and metric_geometry is not None
                    else None
                ),
                "tracking_calibration_sha256": tracking_digest,
                # The first fingerprint is the canonical setup binding used
                # by existing target-trajectory consumers.  Every placement
                # fingerprint is also retained for reproducibility, and the
                # loader above requires their camera_setup_sha256 values to
                # match exactly.
                "capture_session": captures[0].to_json(),
                "capture_sessions": [
                    capture.to_json() for capture in captures
                ],
            },
            "qualification": {
                "safety_qualified": qualified,
                "capture_session_safety_qualified": (
                    all(capture.safety_qualified for capture in captures)
                ),
                "failures": failures,
            },
            "calibration": {
                "schema": CALIBRATION_SCHEMA,
                "safety_qualified": qualified,
                "failures": failures,
                "manifest": {
                    "path": str(args.manifest.resolve()),
                    "sha256": sha256_file(args.manifest.resolve()),
                    "declared_schema": manifest.get("schema"),
                },
                "reference_plane_provenance": reference_provenance,
                "fit_placement_count": len(fit),
                "validation_placement_count": len(validation),
                "fit_span_mm": {"x": x_span, "y": y_span},
                "fit_hull_area_mm2": fit_hull_area,
                "design_condition": condition,
                "stationary_stability_p95_mm_max": maximum_stability,
                "stationary_hold_duration_s_min": minimum_hold_duration,
                "fit_error_mm": fit_stats,
                "held_out_error_mm": validation_stats,
                "limits": {
                    "minimum_fit_span_mm": args.minimum_fit_span_mm,
                    "minimum_fit_hull_area_mm2": args.minimum_fit_hull_area_mm2,
                    "minimum_hold_duration_s": args.minimum_hold_duration_s,
                    "maximum_stability_p95_mm": args.maximum_stability_p95_mm,
                    "maximum_fit_p95_mm": args.maximum_fit_p95_mm,
                    "maximum_validation_p95_mm": args.maximum_validation_p95_mm,
                    "maximum_validation_error_mm": args.maximum_validation_error_mm,
                },
                "placements": [
                    {
                        "id": item.placement_id,
                        "role": item.role,
                        "apparent_blue_xy_mm": list(item.apparent_blue_xy_mm),
                        "apparent_red_xy_mm": list(item.apparent_red_xy_mm),
                        "true_blue_xy_mm": list(item.true_blue_xy_mm),
                        "true_red_xy_mm": list(item.true_red_xy_mm),
                        "stability_p95_mm": item.stability_p95_mm,
                        "stationary_duration_s": item.stationary_duration_s,
                        "source": item.source,
                    }
                    for item in placements
                ],
            },
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(artifact, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(
            "[LABEL-PLANE-CAL] qualified={} camera=({:.2f},{:.2f},{:.2f})mm "
            "fit_p95={:.3f}mm validation_p95={:.3f}mm validation_max={:.3f}mm".format(
                int(qualified),
                geometry.camera_center_x_mm,
                geometry.camera_center_y_mm,
                geometry.camera_height_mm,
                float(fit_stats["p95"]),
                float(validation_stats["p95"]),
                float(validation_stats["max"]),
            )
        )
        print(f"[LABEL-PLANE-CAL] output={args.output}")
        return 0 if qualified else 1
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        print(f"[LABEL-PLANE-CAL][ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
