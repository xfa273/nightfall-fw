#!/usr/bin/env python3
"""Apply a qualified blue/red label-plane correction to a trajectory CSV."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Sequence

import numpy as np

import aruco_trajectory as aruco
import board_layout
import board_metric_geometry
import camera_capture_fingerprint
import label_plane_geometry


SIDECAR_SCHEMA = "nightfall_height_corrected_trajectory_v1"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Reproject apparent reference-plane blue/red label coordinates "
            "onto their measured physical planes."
        )
    )
    parser.add_argument("trajectory_csv", type=Path)
    parser.add_argument("--board-layout", type=Path, required=True)
    parser.add_argument(
        "--board-metric-geometry",
        type=Path,
        default=None,
        help=(
            "qualified dense board map used by the source trajectory; "
            "required when the label-plane calibration binds one"
        ),
    )
    parser.add_argument("--tracking-geometry", type=Path, required=True)
    parser.add_argument("--label-plane-geometry", type=Path, required=True)
    parser.add_argument(
        "--capture-session-manifest",
        type=Path,
        required=True,
        help=(
            "JSON object containing capture_session, or a direct "
            "nightfall_camera_capture_session_v1 object"
        ),
    )
    parser.add_argument("--canonical-size", type=int, default=900)
    parser.add_argument("--smooth-window", type=int, default=9)
    parser.add_argument(
        "--source-calibration-json",
        type=Path,
        default=None,
        help=(
            "markerless calibration.json for the input CSV (default: sibling "
            "calibration.json)"
        ),
    )
    parser.add_argument(
        "--maximum-baseline-error-p95-mm",
        type=float,
        default=1.5,
    )
    parser.add_argument(
        "--confirm-unchanged-camera-board-setup",
        action="store_true",
        help=(
            "explicitly confirm that camera position, board markers, crop, and "
            "physical lens match the stationary calibration"
        ),
    )
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--sidecar", type=Path, default=None)
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_tracking_digest(path: Path) -> tuple[str, dict[str, Any]]:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid tracking geometry JSON: {exc}") from exc
    if not isinstance(raw, dict) or raw.get("schema") != "nightfall_machine_footprint_v1":
        raise ValueError(
            "tracking geometry must use nightfall_machine_footprint_v1"
        )
    try:
        labels = raw["tracking_labels"]
        blue = float(labels["blue_centre"]["surface_height_mm"])
        red = float(labels["red_front"]["surface_height_mm"])
        distance = float(
            labels["red_front"]["horizontal_distance_from_blue_centre_mm"]
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("tracking geometry has invalid tracking-label fields") from exc
    if not all(math.isfinite(value) for value in (blue, red, distance)):
        raise ValueError("tracking-label geometry values must be finite")
    return sha256_file(path), raw


def _load_source_calibration(
    path: Path,
    *,
    expected_board_sha256: str,
    expected_canonical_size: int,
) -> dict[str, Any]:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid source calibration JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError("source calibration root must be an object")
    if raw.get("schema") != "nightfall_markerless_board_calibration_v3":
        raise ValueError("source calibration schema is invalid")
    try:
        measured = raw["measured_layout"]
        measured_digest = str(measured["sha256"]).lower()
        canonical_size = int(raw["canonical"]["size_px"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("source calibration bindings are invalid") from exc
    if measured_digest != expected_board_sha256.lower():
        raise ValueError("source trajectory used a different board layout")
    if canonical_size != expected_canonical_size:
        raise ValueError("source trajectory used a different canonical size")
    return raw


def _load_target_capture(
    manifest_path: Path,
) -> camera_capture_fingerprint.CameraCaptureFingerprint:
    try:
        raw = json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid capture-session manifest JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError("capture-session manifest root must be an object")
    block = raw.get("capture_session")
    if block is None and raw.get("schema") == (
        camera_capture_fingerprint.CAPTURE_SESSION_SCHEMA
    ):
        block = raw
    result = camera_capture_fingerprint.load_capture_session(
        manifest_path.resolve().parent, block
    )
    if not result.safety_qualified:
        raise ValueError(
            "target capture_session is legacy/unverified and cannot produce "
            "an absolute-scene trajectory"
        )
    return result


def _load_calibration_capture(
    geometry_path: Path,
) -> camera_capture_fingerprint.CameraCaptureFingerprint:
    try:
        raw = json.loads(geometry_path.read_text(encoding="utf-8"))
        block = raw["bindings"]["capture_session"]
    except (json.JSONDecodeError, KeyError, TypeError) as exc:
        raise ValueError(
            "label-plane geometry lacks a bound calibration capture_session"
        ) from exc
    return camera_capture_fingerprint.revalidate_capture_fingerprint(block)


def _column(rows: Sequence[dict[str, str]], name: str) -> np.ndarray:
    values = []
    for row in rows:
        try:
            value = float(row.get(name, ""))
        except (TypeError, ValueError):
            value = float("nan")
        values.append(value)
    return np.asarray(values, dtype=float)


def _canonical_to_board(
    x_px: np.ndarray,
    y_px: np.ndarray,
    layout: board_layout.BoardLayout,
) -> tuple[np.ndarray, np.ndarray]:
    bounds = layout.raw["canvas_bounds_mm"]
    return (
        float(bounds["x_min"]) + x_px / layout.pixels_per_mm,
        float(bounds["y_max"]) - y_px / layout.pixels_per_mm,
    )


def _board_to_canonical(
    x_mm: np.ndarray,
    y_mm: np.ndarray,
    layout: board_layout.BoardLayout,
) -> tuple[np.ndarray, np.ndarray]:
    bounds = layout.raw["canvas_bounds_mm"]
    return (
        (x_mm - float(bounds["x_min"])) * layout.pixels_per_mm,
        (float(bounds["y_max"]) - y_mm) * layout.pixels_per_mm,
    )


def _interpolate(values: np.ndarray, valid: np.ndarray, field: str) -> np.ndarray:
    good = np.flatnonzero(valid & np.isfinite(values))
    if len(good) < 2:
        raise ValueError(f"fewer than two valid {field} samples")
    return np.interp(np.arange(len(values)), good, values[good])


def _unwrap_sparse_degrees(values: np.ndarray, valid: np.ndarray) -> np.ndarray:
    good = np.flatnonzero(valid & np.isfinite(values))
    if len(good) < 2:
        raise ValueError("fewer than two valid corrected heading samples")
    unwrapped_good = np.degrees(np.unwrap(np.radians(values[good])))
    return np.interp(np.arange(len(values)), good, unwrapped_good)


def correct_rows(
    rows: list[dict[str, str]],
    layout: board_layout.BoardLayout,
    geometry: label_plane_geometry.LabelPlaneGeometry,
    smooth_window: int,
    metric_geometry: board_metric_geometry.BoardMetricGeometry | None = None,
) -> tuple[list[dict[str, str]], dict[str, Any]]:
    blue_px_x = _column(rows, "label_x_px")
    blue_px_y = _column(rows, "label_y_px")
    red_px_x = _column(rows, "front_label_x_px")
    red_px_y = _column(rows, "front_label_y_px")
    time_s = _column(rows, "video_pts_s")
    if not np.all(np.isfinite(time_s)):
        time_s = _column(rows, "time_s")
    if len(time_s) < 2 or not np.all(np.isfinite(time_s)):
        raise ValueError("trajectory has invalid video/time timestamps")
    if not np.all(np.diff(time_s) > 0.0):
        raise ValueError("trajectory timestamps must be strictly increasing")
    if metric_geometry is None:
        apparent_blue_x, apparent_blue_y = _canonical_to_board(
            blue_px_x, blue_px_y, layout
        )
        apparent_red_x, apparent_red_y = _canonical_to_board(
            red_px_x, red_px_y, layout
        )
    else:
        apparent_blue_x = np.full(len(rows), np.nan, dtype=float)
        apparent_blue_y = np.full(len(rows), np.nan, dtype=float)
        apparent_red_x = np.full(len(rows), np.nan, dtype=float)
        apparent_red_y = np.full(len(rows), np.nan, dtype=float)
        blue_finite = np.isfinite(blue_px_x) & np.isfinite(blue_px_y)
        red_finite = np.isfinite(red_px_x) & np.isfinite(red_px_y)
        if np.any(blue_finite):
            apparent_blue = metric_geometry.map_points(
                np.column_stack([blue_px_x[blue_finite], blue_px_y[blue_finite]])
            )
            apparent_blue_x[blue_finite], apparent_blue_y[blue_finite] = (
                apparent_blue.T
            )
        if np.any(red_finite):
            apparent_red = metric_geometry.map_points(
                np.column_stack([red_px_x[red_finite], red_px_y[red_finite]])
            )
            apparent_red_x[red_finite], apparent_red_y[red_finite] = apparent_red.T
    source_pose_valid = _column(rows, "pose_valid")
    source_heading_valid = _column(rows, "heading_valid")
    if not np.any(np.isfinite(source_pose_valid)):
        raise ValueError("trajectory has no finite pose_valid samples")
    if not np.any(np.isfinite(source_heading_valid)):
        raise ValueError("trajectory has no finite heading_valid samples")
    pose_valid = (
        (source_pose_valid > 0.5)
        & np.isfinite(apparent_blue_x)
        & np.isfinite(apparent_blue_y)
        & np.isfinite(blue_px_x)
        & np.isfinite(blue_px_y)
    )
    heading_valid = (
        pose_valid
        & (source_heading_valid > 0.5)
        & np.isfinite(apparent_red_x)
        & np.isfinite(apparent_red_y)
        & np.isfinite(red_px_x)
        & np.isfinite(red_px_y)
    )
    corrected_blue_x = np.full(len(rows), np.nan, dtype=float)
    corrected_blue_y = np.full(len(rows), np.nan, dtype=float)
    corrected_red_x = np.full(len(rows), np.nan, dtype=float)
    corrected_red_y = np.full(len(rows), np.nan, dtype=float)
    corrected_yaw = np.full(len(rows), np.nan, dtype=float)
    corrected_baseline = np.full(len(rows), np.nan, dtype=float)
    for index in np.flatnonzero(pose_valid):
        blue = geometry.correct_blue(
            (apparent_blue_x[index], apparent_blue_y[index])
        )
        corrected_blue_x[index], corrected_blue_y[index] = blue
    for index in np.flatnonzero(heading_valid):
        pair = geometry.correct_pair(
            (apparent_blue_x[index], apparent_blue_y[index]),
            (apparent_red_x[index], apparent_red_y[index]),
        )
        corrected_red_x[index], corrected_red_y[index] = pair.red_xy_mm
        corrected_yaw[index] = pair.yaw_deg
        corrected_baseline[index] = pair.baseline_mm
    blue_x_raw = _interpolate(corrected_blue_x, pose_valid, "blue x")
    blue_y_raw = _interpolate(corrected_blue_y, pose_valid, "blue y")
    yaw_raw = _unwrap_sparse_degrees(corrected_yaw, heading_valid)
    blue_x = aruco.savitzky_golay(blue_x_raw, smooth_window)
    blue_y = aruco.savitzky_golay(blue_y_raw, smooth_window)
    yaw = aruco.savitzky_golay(yaw_raw, smooth_window)
    vx = np.gradient(blue_x, time_s)
    vy = np.gradient(blue_y, time_s)
    speed = np.hypot(vx, vy)
    canonical_x_raw, canonical_y_raw = _board_to_canonical(
        blue_x_raw, blue_y_raw, layout
    )
    canonical_x, canonical_y = _board_to_canonical(blue_x, blue_y, layout)
    grid = layout.raw["grid"]
    origin_x, origin_y = (float(value) for value in grid["origin_mm"])
    pitch = float(grid["pitch_mm"])
    x_cell_raw = (blue_x_raw - origin_x) / pitch
    y_cell_raw = (blue_y_raw - origin_y) / pitch
    x_cell = (blue_x - origin_x) / pitch
    y_cell = (blue_y - origin_y) / pitch
    naive_yaw = np.full(len(rows), np.nan, dtype=float)
    heading_indexes = np.flatnonzero(heading_valid)
    naive_yaw[heading_indexes] = np.degrees(
        np.unwrap(
            np.arctan2(
                apparent_red_y[heading_indexes]
                - apparent_blue_y[heading_indexes],
                apparent_red_x[heading_indexes]
                - apparent_blue_x[heading_indexes],
            )
        )
    )
    apparent_position_error = np.hypot(
        corrected_blue_x - apparent_blue_x,
        corrected_blue_y - apparent_blue_y,
    )
    yaw_delta = np.abs(
        (corrected_yaw - naive_yaw + 180.0) % 360.0 - 180.0
    )

    def fmt(value: float) -> str:
        return "" if not math.isfinite(float(value)) else f"{float(value):.6f}"

    result: list[dict[str, str]] = []
    for index, original in enumerate(rows):
        row = dict(original)
        for name in (
            "x_mm",
            "y_mm",
            "yaw_deg_raw_unwrapped",
            "yaw_deg_unwrapped",
            "yaw_deg",
        ):
            row[f"apparent_floor_{name}"] = original.get(name, "")
        values = {
            "canonical_x_px_raw": canonical_x_raw[index],
            "canonical_y_px_raw": canonical_y_raw[index],
            "canonical_x_px": canonical_x[index],
            "canonical_y_px": canonical_y[index],
            "x_cell_raw": x_cell_raw[index],
            "y_cell_raw": y_cell_raw[index],
            "x_cell": x_cell[index],
            "y_cell": y_cell[index],
            "x_mm": blue_x[index],
            "y_mm": blue_y[index],
            "yaw_deg_raw_unwrapped": yaw_raw[index],
            "yaw_deg_unwrapped": yaw[index],
            "yaw_deg": yaw[index] % 360.0,
            "vx_cell_s": vx[index] / pitch,
            "vy_cell_s": vy[index] / pitch,
            "speed_cell_s": speed[index] / pitch,
            "vx_mm_s": vx[index],
            "vy_mm_s": vy[index],
            "speed_mm_s": speed[index],
            "corrected_red_x_mm": corrected_red_x[index],
            "corrected_red_y_mm": corrected_red_y[index],
            "corrected_front_label_distance_mm": corrected_baseline[index],
        }
        row.update({name: fmt(value) for name, value in values.items()})
        row["pose_valid"] = str(int(pose_valid[index]))
        row["heading_valid"] = str(int(heading_valid[index]))
        row["position_source"] = "blue_label_height_corrected"
        row["heading_source"] = "front_label_height_corrected"
        result.append(row)

    def stats(values: np.ndarray) -> dict[str, float]:
        finite = values[np.isfinite(values)]
        return {
            "median": float(np.median(finite)),
            "p95": float(np.percentile(finite, 95)),
            "max": float(np.max(finite)),
        }

    metadata = {
        "sample_count": len(rows),
        "pose_valid_count": int(np.count_nonzero(pose_valid)),
        "heading_valid_count": int(np.count_nonzero(heading_valid)),
        "blue_position_correction_mm": stats(apparent_position_error),
        "heading_correction_abs_deg": stats(yaw_delta),
        "corrected_front_label_distance_mm": stats(corrected_baseline),
        "corrected_front_label_error_abs_mm": stats(
            np.abs(corrected_baseline - geometry.front_label_distance_mm)
        ),
    }
    return result, metadata


def write_csv(path: Path, rows: Sequence[dict[str, str]]) -> None:
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for name in row:
            if name not in seen:
                seen.add(name)
                fieldnames.append(name)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="ascii") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    try:
        for path in (
            args.trajectory_csv,
            args.board_layout,
            args.tracking_geometry,
            args.label_plane_geometry,
            args.capture_session_manifest,
        ):
            if not path.is_file():
                raise ValueError(f"input does not exist: {path}")
        if (
            args.board_metric_geometry is not None
            and not args.board_metric_geometry.is_file()
        ):
            raise ValueError(
                f"input does not exist: {args.board_metric_geometry}"
            )
        if args.canonical_size < 400:
            raise ValueError("--canonical-size must be at least 400")
        if args.smooth_window < 5 or args.smooth_window % 2 == 0:
            raise ValueError("--smooth-window must be an odd integer >= 5")
        if (
            not math.isfinite(args.maximum_baseline_error_p95_mm)
            or args.maximum_baseline_error_p95_mm <= 0.0
            or args.maximum_baseline_error_p95_mm > 1.5
        ):
            raise ValueError(
                "--maximum-baseline-error-p95-mm must be in (0, 1.5]"
            )
        layout = board_layout.load(args.board_layout, args.canonical_size)
        board_digest = sha256_file(args.board_layout)
        dense_metric = (
            board_metric_geometry.load_geometry(
                args.board_metric_geometry,
                expected_board_layout_sha256=board_digest,
                expected_canonical_size_px=args.canonical_size,
                require_safety_qualified=True,
            )
            if args.board_metric_geometry is not None
            else None
        )
        source_calibration_path = (
            args.source_calibration_json
            if args.source_calibration_json is not None
            else args.trajectory_csv.with_name("calibration.json")
        )
        if not source_calibration_path.is_file():
            raise ValueError(
                "source markerless calibration does not exist: "
                f"{source_calibration_path}"
            )
        _load_source_calibration(
            source_calibration_path,
            expected_board_sha256=board_digest,
            expected_canonical_size=args.canonical_size,
        )
        target_capture = _load_target_capture(args.capture_session_manifest)
        if (
            target_capture.artifact("trajectory_csv").path
            != args.trajectory_csv.resolve()
        ):
            raise ValueError(
                "target capture_session does not bind the input trajectory"
            )
        if (
            target_capture.artifact("source_board_calibration").path
            != source_calibration_path.resolve()
        ):
            raise ValueError(
                "target capture_session does not bind the source calibration"
            )
        calibration_capture = _load_calibration_capture(
            args.label_plane_geometry
        )
        if (
            target_capture.camera_setup_sha256
            != calibration_capture.camera_setup_sha256
        ):
            raise ValueError(
                "target camera setup does not match label-plane calibration"
            )
        tracking_digest, tracking = _load_tracking_digest(args.tracking_geometry)
        geometry = label_plane_geometry.load_geometry(
            args.label_plane_geometry,
            expected_board_layout_sha256=board_digest,
            expected_tracking_calibration_sha256=tracking_digest,
        )
        try:
            geometry_raw = json.loads(
                args.label_plane_geometry.read_text(encoding="utf-8")
            )
            metric_binding = geometry_raw["bindings"].get(
                "board_metric_geometry"
            )
        except (json.JSONDecodeError, KeyError, TypeError) as exc:
            raise ValueError("label-plane geometry has invalid bindings") from exc
        if metric_binding is None:
            if dense_metric is not None:
                raise ValueError(
                    "label-plane calibration did not use board_metric_geometry"
                )
        else:
            if dense_metric is None:
                raise ValueError(
                    "label-plane calibration requires --board-metric-geometry"
                )
            if not isinstance(metric_binding, dict) or metric_binding.get(
                "sha256"
            ) != sha256_file(args.board_metric_geometry):
                raise ValueError("board_metric_geometry binding mismatch")
        labels = tracking["tracking_labels"]
        expected = (
            float(labels["blue_centre"]["surface_height_mm"]),
            float(labels["red_front"]["surface_height_mm"]),
            float(
                labels["red_front"]["horizontal_distance_from_blue_centre_mm"]
            ),
        )
        actual = (
            geometry.blue_label_height_mm,
            geometry.red_label_height_mm,
            geometry.front_label_distance_mm,
        )
        if any(
            not math.isclose(left, right, abs_tol=1e-9, rel_tol=0.0)
            for left, right in zip(expected, actual)
        ):
            raise ValueError("label-plane geometry does not match tracking geometry")
        with args.trajectory_csv.open(newline="", encoding="ascii") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None:
                raise ValueError("trajectory CSV has no header")
            required = {
                "label_x_px",
                "label_y_px",
                "front_label_x_px",
                "front_label_y_px",
                "pose_valid",
                "heading_valid",
            }
            if not required.issubset(reader.fieldnames):
                missing = sorted(required.difference(reader.fieldnames))
                raise ValueError(
                    "trajectory CSV is missing required columns: "
                    + ", ".join(missing)
                )
            rows = list(reader)
        corrected, correction = correct_rows(
            rows, layout, geometry, args.smooth_window, dense_metric
        )
        if (
            correction["corrected_front_label_error_abs_mm"]["p95"]
            > args.maximum_baseline_error_p95_mm
        ):
            raise ValueError(
                "corrected front-label baseline error p95 exceeds the limit"
            )
        output = (
            args.output
            if args.output is not None
            else args.trajectory_csv.with_name(
                args.trajectory_csv.stem + "_height_corrected.csv"
            )
        )
        sidecar = (
            args.sidecar
            if args.sidecar is not None
            else output.with_suffix(".calibration.json")
        )
        source_paths = {
            path.resolve()
            for path in (
                args.trajectory_csv,
                args.board_layout,
                args.tracking_geometry,
                args.label_plane_geometry,
                args.capture_session_manifest,
                source_calibration_path,
            )
        }
        if args.board_metric_geometry is not None:
            source_paths.add(args.board_metric_geometry.resolve())
        if output.resolve() in source_paths or sidecar.resolve() in source_paths:
            raise ValueError("refusing to overwrite a calibration input")
        if output.resolve() == sidecar.resolve():
            raise ValueError("output CSV and sidecar must be different paths")
        write_csv(output, corrected)
        report = {
            "schema": SIDECAR_SCHEMA,
            "input": {
                "trajectory_csv": str(args.trajectory_csv.resolve()),
                "sha256": sha256_file(args.trajectory_csv),
            },
            "output": {
                "trajectory_csv": str(output.resolve()),
                "sha256": sha256_file(output),
            },
            "bindings": {
                "board_layout": str(args.board_layout.resolve()),
                "board_layout_sha256": board_digest,
                "board_metric_geometry": (
                    str(args.board_metric_geometry.resolve())
                    if args.board_metric_geometry is not None
                    else None
                ),
                "board_metric_geometry_sha256": (
                    sha256_file(args.board_metric_geometry)
                    if args.board_metric_geometry is not None
                    else None
                ),
                "tracking_geometry": str(args.tracking_geometry.resolve()),
                "tracking_geometry_sha256": tracking_digest,
                "label_plane_geometry": str(args.label_plane_geometry.resolve()),
                "label_plane_geometry_sha256": sha256_file(
                    args.label_plane_geometry
                ),
                "source_board_calibration": str(
                    source_calibration_path.resolve()
                ),
                "source_board_calibration_sha256": sha256_file(
                    source_calibration_path
                ),
            },
            "capture_session": target_capture.to_json(),
            "qualification": {
                "height_correction_applied": True,
                "source_geometry_safety_qualified": geometry.safety_qualified,
                "source_board_layout_verified": True,
                "target_capture_session_safety_qualified": True,
                "camera_setup_matches_calibration": True,
                "operator_confirmed_unchanged_camera_board_setup": (
                    args.confirm_unchanged_camera_board_setup
                ),
                "absolute_scene_eligible": (
                    args.confirm_unchanged_camera_board_setup
                ),
            },
            "correction": correction,
        }
        sidecar.parent.mkdir(parents=True, exist_ok=True)
        sidecar.write_text(
            json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(
            "[LABEL-PLANE] samples={} blue_shift_p95={:.3f}mm "
            "yaw_shift_p95={:.3f}deg output={}".format(
                correction["sample_count"],
                correction["blue_position_correction_mm"]["p95"],
                correction["heading_correction_abs_deg"]["p95"],
                output,
            )
        )
        print(f"[LABEL-PLANE] sidecar={sidecar}")
        return 0
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        print(f"[LABEL-PLANE][ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
