#!/usr/bin/env python3
"""Correct elevated tracking labels after reference-plane rectification.

A board homography maps an image ray to its intersection ``Q`` with the marker
reference plane. When the tracked label is actually on a parallel plane at a
different absolute height, ``Q`` is not the physical label position ``P``.
This module performs that ray/plane correction in board coordinates and keeps
the camera/layout provenance needed to use it safely.

Board axes are ``+x`` right, ``+y`` forward and ``+z`` up.  Yaw is measured
counter-clockwise from ``+x``, so a vector along ``+y`` has yaw ``+90 deg``.
The historical public identifier ``apparent_floor`` is retained for API/CSV
compatibility; in the current rig it means the apparent point on the 2 mm
ArUco reference plane, not an intersection with the maze floor.
"""

from __future__ import annotations

import hmac
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence


SCHEMA = "nightfall_label_plane_geometry_v1"
COORDINATE_SYSTEM = "board_x_right_y_forward_z_up_mm"
_SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")


@dataclass(frozen=True)
class CorrectedLabelPair:
    """Physical blue/red label pose in board coordinates."""

    blue_xy_mm: tuple[float, float]
    red_xy_mm: tuple[float, float]
    baseline_mm: float
    yaw_deg: float


@dataclass(frozen=True)
class LabelPlaneGeometry:
    """Camera and label-plane geometry bound to its source calibrations."""

    camera_center_x_mm: float
    camera_center_y_mm: float
    camera_height_mm: float
    reference_plane_height_mm: float
    blue_label_height_mm: float
    red_label_height_mm: float
    front_label_distance_mm: float
    board_layout_sha256: str
    tracking_calibration_sha256: str
    safety_qualified: bool

    @property
    def camera_center_xy_mm(self) -> tuple[float, float]:
        return (self.camera_center_x_mm, self.camera_center_y_mm)

    def correct_blue(
        self, apparent_floor_xy_mm: Sequence[float]
    ) -> tuple[float, float]:
        return correct_apparent_floor_point(
            apparent_floor_xy_mm,
            camera_center_xy_mm=self.camera_center_xy_mm,
            camera_height_mm=self.camera_height_mm,
            reference_plane_height_mm=self.reference_plane_height_mm,
            label_height_mm=self.blue_label_height_mm,
        )

    def correct_red(
        self, apparent_floor_xy_mm: Sequence[float]
    ) -> tuple[float, float]:
        return correct_apparent_floor_point(
            apparent_floor_xy_mm,
            camera_center_xy_mm=self.camera_center_xy_mm,
            camera_height_mm=self.camera_height_mm,
            reference_plane_height_mm=self.reference_plane_height_mm,
            label_height_mm=self.red_label_height_mm,
        )

    def correct_pair(
        self,
        apparent_blue_xy_mm: Sequence[float],
        apparent_red_xy_mm: Sequence[float],
    ) -> CorrectedLabelPair:
        return correct_label_pair(
            apparent_blue_xy_mm,
            apparent_red_xy_mm,
            self,
        )


def _finite_float(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{field} must be a finite number")
    return result


def _xy(value: Sequence[float], field: str) -> tuple[float, float]:
    if isinstance(value, (str, bytes)):
        raise ValueError(f"{field} must contain exactly two finite coordinates")
    try:
        if len(value) != 2:
            raise ValueError(
                f"{field} must contain exactly two finite coordinates"
            )
    except TypeError as exc:
        raise ValueError(
            f"{field} must contain exactly two finite coordinates"
        ) from exc
    return (
        _finite_float(value[0], f"{field}[0]"),
        _finite_float(value[1], f"{field}[1]"),
    )


def _sha256(value: Any, field: str) -> str:
    if not isinstance(value, str) or _SHA256_RE.fullmatch(value) is None:
        raise ValueError(f"{field} must be a 64-digit SHA-256 hex string")
    return value.lower()


def _validate_heights(
    camera_height_mm: float,
    reference_plane_height_mm: float,
    label_height_mm: float,
    field: str,
) -> None:
    if reference_plane_height_mm < 0.0:
        raise ValueError("reference plane absolute height must be non-negative")
    if camera_height_mm <= reference_plane_height_mm:
        raise ValueError("camera height must be above the reference plane")
    if label_height_mm < 0.0:
        raise ValueError(f"{field} must be at or above the maze floor")
    if label_height_mm >= camera_height_mm:
        raise ValueError(f"{field} must be below the camera height")


def correct_apparent_floor_point(
    apparent_floor_xy_mm: Sequence[float],
    *,
    camera_center_xy_mm: Sequence[float],
    camera_height_mm: float,
    reference_plane_height_mm: float,
    label_height_mm: float,
) -> tuple[float, float]:
    """Recover a label's physical position from apparent reference-plane Q.

    ``camera_height_mm``, ``reference_plane_height_mm`` and
    ``label_height_mm`` are absolute heights in the same board frame.  The
    reference-plane homography's apparent point is scaled about the camera
    centre by ``(H - h) / (H - h_ref)`` to intersect the actual label plane.
    """

    qx, qy = _xy(apparent_floor_xy_mm, "apparent_floor_xy_mm")
    cx, cy = _xy(camera_center_xy_mm, "camera_center_xy_mm")
    camera_height = _finite_float(camera_height_mm, "camera_height_mm")
    reference_height = _finite_float(
        reference_plane_height_mm,
        "reference_plane_height_mm",
    )
    label_height = _finite_float(label_height_mm, "label_height_mm")
    _validate_heights(
        camera_height,
        reference_height,
        label_height,
        "label_height_mm",
    )
    scale = (camera_height - label_height) / (
        camera_height - reference_height
    )
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError("label-plane correction scale must be positive")
    return (
        cx + scale * (qx - cx),
        cy + scale * (qy - cy),
    )


def correct_label_pair(
    apparent_blue_xy_mm: Sequence[float],
    apparent_red_xy_mm: Sequence[float],
    geometry: LabelPlaneGeometry,
) -> CorrectedLabelPair:
    """Correct both label planes and return the directed board-frame pose."""

    blue = geometry.correct_blue(apparent_blue_xy_mm)
    red = geometry.correct_red(apparent_red_xy_mm)
    dx = red[0] - blue[0]
    dy = red[1] - blue[1]
    baseline = math.hypot(dx, dy)
    if baseline <= 0.0:
        raise ValueError("corrected blue/red label baseline must be positive")
    return CorrectedLabelPair(
        blue_xy_mm=blue,
        red_xy_mm=red,
        baseline_mm=baseline,
        yaw_deg=math.degrees(math.atan2(dy, dx)),
    )


def load_geometry(
    path: Path,
    *,
    expected_board_layout_sha256: str,
    expected_tracking_calibration_sha256: str,
) -> LabelPlaneGeometry:
    """Load a provenance-bound label-plane geometry JSON artifact."""

    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid label-plane geometry JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError("label-plane geometry root must be an object")
    if raw.get("schema") != SCHEMA:
        raise ValueError(f"label-plane geometry schema must be {SCHEMA}")
    if raw.get("coordinate_system") != COORDINATE_SYSTEM:
        raise ValueError(
            f"label-plane coordinate_system must be {COORDINATE_SYSTEM}"
        )

    try:
        camera = raw["camera_center_board_mm"]
        labels = raw["label_absolute_heights_mm"]
        bindings = raw["bindings"]
        qualification = raw["qualification"]
        calibration = raw["calibration"]
        camera_x = _finite_float(camera["x"], "camera_center_board_mm.x")
        camera_y = _finite_float(camera["y"], "camera_center_board_mm.y")
        camera_height = _finite_float(
            camera["height"], "camera_center_board_mm.height"
        )
        reference_height = _finite_float(
            raw["reference_plane_absolute_height_mm"],
            "reference_plane_absolute_height_mm",
        )
        blue_height = _finite_float(
            labels["blue_center"], "label_absolute_heights_mm.blue_center"
        )
        red_height = _finite_float(
            labels["red_front"], "label_absolute_heights_mm.red_front"
        )
        front_distance = _finite_float(
            raw["front_label_distance_mm"], "front_label_distance_mm"
        )
        board_digest = _sha256(
            bindings["board_layout_sha256"],
            "bindings.board_layout_sha256",
        )
        tracking_digest = _sha256(
            bindings["tracking_calibration_sha256"],
            "bindings.tracking_calibration_sha256",
        )
        safety_qualified = qualification["safety_qualified"]
    except (KeyError, TypeError) as exc:
        raise ValueError("label-plane geometry fields are invalid") from exc

    if front_distance <= 0.0:
        raise ValueError("front_label_distance_mm must be positive")
    if safety_qualified is not True:
        raise ValueError(
            "qualification.safety_qualified must be explicitly true"
        )
    if not isinstance(calibration, dict):
        raise ValueError("label-plane calibration evidence must be an object")
    try:
        if calibration.get("schema") != "nightfall_label_plane_camera_fit_v1":
            raise ValueError("calibration schema is invalid")
        if calibration.get("safety_qualified") is not True:
            raise ValueError("calibration must be explicitly safety-qualified")
        if calibration.get("failures") != [] or qualification.get("failures") != []:
            raise ValueError("qualified calibration must have no recorded failures")
        fit_count = int(calibration["fit_placement_count"])
        validation_count = int(calibration["validation_placement_count"])
        fit_span_x = _finite_float(
            calibration["fit_span_mm"]["x"], "calibration.fit_span_mm.x"
        )
        fit_span_y = _finite_float(
            calibration["fit_span_mm"]["y"], "calibration.fit_span_mm.y"
        )
        hull_area = _finite_float(
            calibration["fit_hull_area_mm2"],
            "calibration.fit_hull_area_mm2",
        )
        hold_duration = _finite_float(
            calibration["stationary_hold_duration_s_min"],
            "calibration.stationary_hold_duration_s_min",
        )
        stability = _finite_float(
            calibration["stationary_stability_p95_mm_max"],
            "calibration.stationary_stability_p95_mm_max",
        )
        fit_p95 = _finite_float(
            calibration["fit_error_mm"]["p95"],
            "calibration.fit_error_mm.p95",
        )
        validation_p95 = _finite_float(
            calibration["held_out_error_mm"]["p95"],
            "calibration.held_out_error_mm.p95",
        )
        validation_max = _finite_float(
            calibration["held_out_error_mm"]["max"],
            "calibration.held_out_error_mm.max",
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"label-plane calibration evidence is invalid: {exc}") from exc
    if fit_count < 4 or validation_count < 1:
        raise ValueError("label-plane calibration lacks fit/validation placements")
    if fit_span_x < 400.0 or fit_span_y < 400.0 or hull_area < 120000.0:
        raise ValueError("label-plane calibration does not span the board")
    if hold_duration < 1.5 or stability > 0.75:
        raise ValueError("label-plane stationary calibration quality is insufficient")
    if fit_p95 > 1.0 or validation_p95 > 1.0 or validation_max > 1.5:
        raise ValueError("label-plane fit/held-out residual exceeds safety limits")
    _validate_heights(
        camera_height,
        reference_height,
        blue_height,
        "blue label absolute height",
    )
    _validate_heights(
        camera_height,
        reference_height,
        red_height,
        "red label absolute height",
    )
    relative_camera_height = camera_height - reference_height
    if not 100.0 <= relative_camera_height <= 3000.0:
        raise ValueError(
            "camera height above the reference plane must be within "
            "100..3000 mm"
        )

    expected_board = _sha256(
        expected_board_layout_sha256,
        "expected_board_layout_sha256",
    )
    expected_tracking = _sha256(
        expected_tracking_calibration_sha256,
        "expected_tracking_calibration_sha256",
    )
    if not hmac.compare_digest(board_digest, expected_board):
        raise ValueError("board-layout SHA-256 does not match geometry binding")
    if not hmac.compare_digest(tracking_digest, expected_tracking):
        raise ValueError(
            "tracking-calibration SHA-256 does not match geometry binding"
        )

    return LabelPlaneGeometry(
        camera_center_x_mm=camera_x,
        camera_center_y_mm=camera_y,
        camera_height_mm=camera_height,
        reference_plane_height_mm=reference_height,
        blue_label_height_mm=blue_height,
        red_label_height_mm=red_height,
        front_label_distance_mm=front_distance,
        board_layout_sha256=board_digest,
        tracking_calibration_sha256=tracking_digest,
        safety_qualified=True,
    )
