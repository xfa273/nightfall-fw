#!/usr/bin/env python3
"""Verify provenance for a height-corrected markerless trajectory."""

from __future__ import annotations

import hashlib
import hmac
import json
import math
from pathlib import Path
from typing import Any

import label_plane_geometry


SCHEMA = "nightfall_height_corrected_trajectory_v1"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _bound_file(item: Any, label: str) -> tuple[Path, str]:
    if not isinstance(item, dict):
        raise ValueError(f"{label} must be an object")
    path_value = item.get(label)
    digest = item.get(f"{label}_sha256")
    if not isinstance(path_value, str) or not isinstance(digest, str):
        raise ValueError(f"{label} path/digest is missing")
    path = Path(path_value).resolve()
    if not path.is_file():
        raise ValueError(f"bound {label} does not exist: {path}")
    actual = sha256_file(path)
    if not hmac.compare_digest(actual, digest.lower()):
        raise ValueError(f"bound {label} SHA-256 does not match")
    return path, actual


def _load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid bound {label} JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError(f"bound {label} root must be an object")
    return raw


def _validate_board_layout(path: Path) -> None:
    raw = _load_object(path, "board layout")
    if raw.get("schema") != "nightfall_vision_board_layout_v1":
        raise ValueError("bound board layout schema is invalid")
    if raw.get("coordinate_system") != "x_right_y_forward_mm":
        raise ValueError("bound board layout coordinate system is invalid")
    try:
        bounds = raw["canvas_bounds_mm"]
        grid = raw["grid"]
        origin = grid["origin_mm"]
        if not isinstance(origin, list) or len(origin) != 2:
            raise ValueError("grid origin must contain two coordinates")
        cells = grid["cells"]
        if isinstance(cells, bool) or not isinstance(cells, int):
            raise ValueError("grid cell count must be an integer")
        numeric = (
            float(bounds["x_min"]),
            float(bounds["x_max"]),
            float(bounds["y_min"]),
            float(bounds["y_max"]),
            float(origin[0]),
            float(origin[1]),
            float(grid["pitch_mm"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("bound board layout geometry is invalid") from exc
    if not all(math.isfinite(value) for value in numeric):
        raise ValueError("bound board layout geometry must be finite")
    x_min, x_max, y_min, y_max, origin_x, origin_y, pitch = numeric
    if x_max <= x_min or y_max <= y_min:
        raise ValueError("bound board layout bounds are inverted")
    x_span = x_max - x_min
    y_span = y_max - y_min
    if abs(x_span - y_span) > max(x_span, y_span) * 1e-6:
        raise ValueError("bound board layout bounds must be square")
    if pitch <= 0.0 or cells < 2:
        raise ValueError("bound board grid dimensions must be positive")
    grid_right = origin_x + cells * pitch
    grid_top = origin_y + cells * pitch
    if not (
        x_min <= origin_x < grid_right <= x_max
        and y_min <= origin_y < grid_top <= y_max
    ):
        raise ValueError("bound board grid must fit inside the canvas")
    try:
        markers = raw["markers"]
        if not isinstance(markers, dict):
            raise ValueError("bound board markers must be an object")
        marker_ids = set(markers)
        if not {"4", "5", "6"}.issubset(marker_ids):
            raise ValueError("bound board layout must contain markers 4/5/6")
        if not marker_ids.issubset({"4", "5", "6", "7"}):
            raise ValueError("bound board layout has an unsupported marker ID")
        centres: dict[str, tuple[float, float]] = {}
        for marker_id, marker in markers.items():
            centre = marker["center_mm"]
            if not isinstance(centre, list) or len(centre) != 2:
                raise ValueError(f"bound board marker {marker_id} centre is invalid")
            values = (
                float(centre[0]),
                float(centre[1]),
                float(marker["side_mm"]),
                float(marker["rotation_deg"]),
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"bound board marker {marker_id} is invalid")
            if values[2] <= 0.0:
                raise ValueError(f"bound board marker {marker_id} side is invalid")
            centres[marker_id] = (values[0], values[1])
            half = values[2] / 2.0
            angle = math.radians(values[3])
            cosine = math.cos(angle)
            sine = math.sin(angle)
            for offset_x, offset_y in (
                (-half, half),
                (half, half),
                (half, -half),
                (-half, -half),
            ):
                corner_x = values[0] + cosine * offset_x - sine * offset_y
                corner_y = values[1] + sine * offset_x + cosine * offset_y
                if not (
                    x_min <= corner_x <= x_max
                    and y_min <= corner_y <= y_max
                ):
                    raise ValueError(
                        f"bound board marker {marker_id} extends outside canvas"
                    )
        first = centres["5"]
        second = centres["4"]
        third = centres["6"]
        twice_area = abs(
            (second[0] - first[0]) * (third[1] - first[1])
            - (second[1] - first[1]) * (third[0] - first[0])
        )
        if twice_area <= 1e-6:
            raise ValueError("bound board marker centres are degenerate")
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise ValueError("bound board marker geometry is invalid") from exc


def verify_height_corrected_trajectory(
    trajectory_csv: Path,
    sidecar_path: Path,
) -> dict[str, Any]:
    """Return a verified sidecar or fail closed on any binding mismatch."""

    trajectory = trajectory_csv.resolve()
    sidecar = sidecar_path.resolve()
    if not trajectory.is_file():
        raise ValueError(f"trajectory does not exist: {trajectory}")
    if not sidecar.is_file():
        raise ValueError(f"height-correction sidecar does not exist: {sidecar}")
    try:
        raw = json.loads(sidecar.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid height-correction sidecar JSON: {exc}") from exc
    if not isinstance(raw, dict) or raw.get("schema") != SCHEMA:
        raise ValueError(f"height-correction sidecar schema must be {SCHEMA}")
    try:
        output = raw["output"]
        qualification = raw["qualification"]
        bindings = raw["bindings"]
        if not isinstance(output, dict):
            raise TypeError("output must be an object")
        if not isinstance(qualification, dict):
            raise TypeError("qualification must be an object")
        if not isinstance(bindings, dict):
            raise TypeError("bindings must be an object")
        output_path = Path(output["trajectory_csv"]).resolve()
        output_digest = str(output["sha256"]).lower()
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("height-correction sidecar fields are invalid") from exc
    if output_path != trajectory:
        raise ValueError("sidecar output path does not match trajectory")
    if not hmac.compare_digest(sha256_file(trajectory), output_digest):
        raise ValueError("height-corrected trajectory SHA-256 does not match")
    for field in (
        "height_correction_applied",
        "source_geometry_safety_qualified",
        "source_board_layout_verified",
        "operator_confirmed_unchanged_camera_board_setup",
        "absolute_scene_eligible",
    ):
        if qualification.get(field) is not True:
            raise ValueError(f"qualification.{field} must be explicitly true")
    bound: dict[str, tuple[Path, str]] = {}
    for label in (
        "board_layout",
        "tracking_geometry",
        "label_plane_geometry",
        "source_board_calibration",
    ):
        bound[label] = _bound_file(bindings, label)
    board_path, board_digest = bound["board_layout"]
    tracking_path, tracking_digest = bound["tracking_geometry"]
    geometry_path, _geometry_digest = bound["label_plane_geometry"]
    source_calibration_path, _source_calibration_digest = bound[
        "source_board_calibration"
    ]
    # Validate the bound files as a coherent calibration set, rather than only
    # trusting the booleans copied into the trajectory sidecar.
    _validate_board_layout(board_path)
    tracking = _load_object(tracking_path, "tracking geometry")
    if tracking.get("schema") != "nightfall_machine_footprint_v1":
        raise ValueError("bound tracking geometry schema is invalid")
    try:
        reference = tracking["reference_point"]
        if (
            reference.get("name") != "blue-label centre"
            or reference.get("coincident_with_machine_centre") is not True
            or reference.get("coincident_with_turn_centre") is not True
        ):
            raise ValueError(
                "tracking reference must be the coincident blue/machine/turn centre"
            )
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError("bound tracking reference point is invalid") from exc
    try:
        labels = tracking["tracking_labels"]
        if labels.get("height_reference") != "maze floor":
            raise ValueError("tracking-label height reference must be maze floor")
        blue = float(labels["blue_centre"]["surface_height_mm"])
        red = float(labels["red_front"]["surface_height_mm"])
        distance = float(
            labels["red_front"]["horizontal_distance_from_blue_centre_mm"]
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("bound tracking-label geometry is invalid") from exc
    if not all(math.isfinite(value) for value in (blue, red, distance)):
        raise ValueError("bound tracking-label geometry must be finite")
    geometry = label_plane_geometry.load_geometry(
        geometry_path,
        expected_board_layout_sha256=board_digest,
        expected_tracking_calibration_sha256=tracking_digest,
    )
    expected = (blue, red, distance)
    actual = (
        geometry.blue_label_height_mm,
        geometry.red_label_height_mm,
        geometry.front_label_distance_mm,
    )
    if any(
        not math.isclose(left, right, abs_tol=1e-9, rel_tol=0.0)
        for left, right in zip(expected, actual)
    ):
        raise ValueError(
            "bound label-plane geometry does not match tracking-label geometry"
        )
    source_calibration = _load_object(
        source_calibration_path, "source board calibration"
    )
    if (
        source_calibration.get("schema")
        != "nightfall_markerless_board_calibration_v3"
    ):
        raise ValueError("bound source board calibration schema is invalid")
    try:
        source_board_digest = str(
            source_calibration["measured_layout"]["sha256"]
        ).lower()
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("bound source board calibration is invalid") from exc
    if not hmac.compare_digest(source_board_digest, board_digest):
        raise ValueError(
            "source trajectory board layout does not match correction layout"
        )
    return raw
