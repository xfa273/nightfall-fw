#!/usr/bin/env python3
"""Load a measured fixed-marker layout for maze-plane rectification."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

import aruco_trajectory as aruco


SCHEMA = "nightfall_vision_board_layout_v1"
REQUIRED_MARKERS = (5, 4, 6)


@dataclass
class BoardLayout:
    path: Path
    raw: dict[str, Any]
    target_corners_px: dict[int, np.ndarray]
    grid: aruco.GridCalibration
    grid_pitch_mm: float
    pixels_per_mm: float


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{field} must be a finite number")
    return number


def _pair(value: Any, field: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{field} must be a two-element array")
    return (
        _finite_number(value[0], f"{field}[0]"),
        _finite_number(value[1], f"{field}[1]"),
    )


def _marker_corners_mm(
    center: tuple[float, float],
    side_mm: float,
    rotation_deg: float,
) -> np.ndarray:
    half = side_mm / 2.0
    offsets = np.asarray(
        [
            [-half, half],
            [half, half],
            [half, -half],
            [-half, -half],
        ],
        dtype=float,
    )
    angle = math.radians(rotation_deg)
    rotation = np.asarray(
        [
            [math.cos(angle), -math.sin(angle)],
            [math.sin(angle), math.cos(angle)],
        ],
        dtype=float,
    )
    return offsets @ rotation.T + np.asarray(center, dtype=float)


def _reject_duplicate_keys(
    pairs: list[tuple[str, Any]],
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_nonstandard_constant(value: str) -> Any:
    raise ValueError(f"non-standard JSON numeric constant: {value}")


def load(path: Path, canonical_size: int) -> BoardLayout:
    if canonical_size < 2:
        raise ValueError("canonical_size must be at least 2")
    try:
        raw = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_nonstandard_constant,
        )
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid board-layout JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError("board layout root must be an object")
    if raw.get("schema") != SCHEMA:
        raise ValueError(f"board layout schema must be {SCHEMA}")
    if raw.get("coordinate_system") != "x_right_y_forward_mm":
        raise ValueError("board layout coordinate_system must be x_right_y_forward_mm")

    bounds = raw.get("canvas_bounds_mm")
    if not isinstance(bounds, dict):
        raise ValueError("canvas_bounds_mm must be an object")
    x_min = _finite_number(bounds.get("x_min"), "canvas_bounds_mm.x_min")
    x_max = _finite_number(bounds.get("x_max"), "canvas_bounds_mm.x_max")
    y_min = _finite_number(bounds.get("y_min"), "canvas_bounds_mm.y_min")
    y_max = _finite_number(bounds.get("y_max"), "canvas_bounds_mm.y_max")
    x_span = x_max - x_min
    y_span = y_max - y_min
    if x_span <= 0 or y_span <= 0:
        raise ValueError("canvas bounds must have positive width and height")
    if abs(x_span - y_span) > max(x_span, y_span) * 1e-6:
        raise ValueError(
            "canvas bounds must be square so x/y metric scale is identical"
        )
    pixels_per_mm = (canonical_size - 1.0) / x_span

    markers = raw.get("markers")
    if not isinstance(markers, dict):
        raise ValueError("markers must be an object")
    target_corners: dict[int, np.ndarray] = {}
    for marker_id_text, item in markers.items():
        try:
            marker_id = int(marker_id_text)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid marker ID in layout: {marker_id_text}") from exc
        if marker_id not in aruco.FIXED_ORDER:
            raise ValueError(
                f"layout marker {marker_id} is not one of {aruco.FIXED_ORDER}"
            )
        if marker_id in target_corners:
            raise ValueError(f"duplicate marker ID in layout: {marker_id}")
        if not isinstance(item, dict):
            raise ValueError(f"marker {marker_id} must be an object")
        center = _pair(item.get("center_mm"), f"marker {marker_id}.center_mm")
        side_mm = _finite_number(
            item.get("side_mm"),
            f"marker {marker_id}.side_mm",
        )
        rotation_deg = _finite_number(
            item.get("rotation_deg", 0.0),
            f"marker {marker_id}.rotation_deg",
        )
        if side_mm <= 0:
            raise ValueError(f"marker {marker_id}.side_mm must be positive")
        corners_mm = _marker_corners_mm(center, side_mm, rotation_deg)
        if (
            np.min(corners_mm[:, 0]) < x_min
            or np.max(corners_mm[:, 0]) > x_max
            or np.min(corners_mm[:, 1]) < y_min
            or np.max(corners_mm[:, 1]) > y_max
        ):
            raise ValueError(f"marker {marker_id} extends outside canvas_bounds_mm")
        corners_px = np.column_stack(
            [
                (corners_mm[:, 0] - x_min) * pixels_per_mm,
                (y_max - corners_mm[:, 1]) * pixels_per_mm,
            ]
        )
        target_corners[marker_id] = corners_px.astype(np.float32)
    missing = [
        marker_id for marker_id in REQUIRED_MARKERS if marker_id not in target_corners
    ]
    if missing:
        raise ValueError(f"board layout is missing marker(s): {missing}")
    required_centers = np.asarray(
        [np.mean(target_corners[marker_id], axis=0) for marker_id in REQUIRED_MARKERS],
        dtype=np.float32,
    )
    vector_a = required_centers[1] - required_centers[0]
    vector_b = required_centers[2] - required_centers[0]
    triangle_area = (
        abs(float(vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0])) / 2.0
    )
    if triangle_area < 100.0:
        raise ValueError("required marker centers must span a non-degenerate triangle")

    grid_data = raw.get("grid")
    if not isinstance(grid_data, dict):
        raise ValueError("grid must be an object")
    origin_x_mm, origin_y_mm = _pair(
        grid_data.get("origin_mm"),
        "grid.origin_mm",
    )
    pitch_mm = _finite_number(grid_data.get("pitch_mm"), "grid.pitch_mm")
    cells_value = grid_data.get("cells")
    if isinstance(cells_value, bool) or not isinstance(cells_value, int):
        raise ValueError("grid.cells must be an integer")
    cells = cells_value
    if pitch_mm <= 0:
        raise ValueError("grid.pitch_mm must be positive")
    if cells < 2:
        raise ValueError("grid.cells must be an integer >= 2")
    grid_right_mm = origin_x_mm + cells * pitch_mm
    grid_top_mm = origin_y_mm + cells * pitch_mm
    if not (
        x_min <= origin_x_mm < grid_right_mm <= x_max
        and y_min <= origin_y_mm < grid_top_mm <= y_max
    ):
        raise ValueError("grid must fit inside canvas_bounds_mm")

    x_origin_px = (origin_x_mm - x_min) * pixels_per_mm
    y_origin_px = (y_max - grid_top_mm) * pixels_per_mm
    pitch_px = pitch_mm * pixels_per_mm
    lines = np.arange(cells + 1, dtype=float)
    grid = aruco.GridCalibration(
        x_lines_px=x_origin_px + pitch_px * lines,
        y_lines_px=y_origin_px + pitch_px * lines,
        x_origin_px=x_origin_px,
        y_origin_px=y_origin_px,
        x_pitch_px=pitch_px,
        y_pitch_px=pitch_px,
        cells=cells,
        x_peak_contrast=float("nan"),
        y_peak_contrast=float("nan"),
    )
    return BoardLayout(
        path=path,
        raw=raw,
        target_corners_px=target_corners,
        grid=grid,
        grid_pitch_mm=pitch_mm,
        pixels_per_mm=pixels_per_mm,
    )
