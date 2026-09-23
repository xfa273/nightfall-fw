#!/usr/bin/env python3
"""Dense reference-plane metric correction for the fixed maze camera.

The four outer ArUco markers remove frame-to-frame projective motion, but they
cannot by themselves prove that an interior canonical pixel has the requested
metric coordinate.  This module represents a smooth map from those canonical
pixels to measured board coordinates.  It is fitted from a distributed set of
known points on one physical reference plane (normally flush marks on the maze
floor), with independent held-out points used as a fail-closed qualification
gate.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np


SCHEMA = "nightfall_board_metric_geometry_v1"
COORDINATE_SYSTEM = "canonical_px_to_x_right_y_forward_mm"
MINIMUM_POINTS = 32
MINIMUM_VALIDATION_POINTS = 8
MINIMUM_HULL_AREA_MM2 = 300_000.0
MAXIMUM_FIT_P95_MM = 0.75
MAXIMUM_FIT_ERROR_MM = 1.5
MAXIMUM_VALIDATION_P95_MM = 1.0
MAXIMUM_VALIDATION_ERROR_MM = 1.5
MAXIMUM_CONDITION_NUMBER = 1.0e5
REFERENCE_PROVENANCE_FIELDS = (
    "surface",
    "height_reference",
    "measurement",
    "confirmed_date",
    "remeasure_if",
)


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


def _positive_integer(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{field} must be a positive integer")
    return value


def _digest(value: Any, field: str) -> str:
    if not isinstance(value, str) or len(value) != 64:
        raise ValueError(f"{field} must be a SHA-256 digest")
    try:
        int(value, 16)
    except ValueError as exc:
        raise ValueError(f"{field} must be a SHA-256 digest") from exc
    return value.lower()


def reference_plane_provenance(value: Any) -> dict[str, str]:
    if not isinstance(value, dict):
        raise ValueError("reference_plane_provenance must be an object")
    result: dict[str, str] = {}
    for field in REFERENCE_PROVENANCE_FIELDS:
        item = value.get(field)
        if not isinstance(item, str) or not item.strip():
            raise ValueError(
                f"reference_plane_provenance.{field} must be a non-empty string"
            )
        result[field] = item.strip()
    if result["height_reference"] != "maze floor":
        raise ValueError(
            "reference_plane_provenance.height_reference must be maze floor"
        )
    return result


def polynomial_terms(degree: int) -> tuple[tuple[int, int], ...]:
    if degree not in (1, 2, 3):
        raise ValueError("polynomial degree must be 1, 2, or 3")
    return tuple(
        (x_power, total - x_power)
        for total in range(degree + 1)
        for x_power in range(total + 1)
    )


def design_matrix(
    points_xy: np.ndarray,
    center_px: tuple[float, float],
    scale_px: tuple[float, float],
    terms: Sequence[tuple[int, int]],
) -> np.ndarray:
    points = np.asarray(points_xy, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
        raise ValueError("points must be a finite Nx2 array")
    sx, sy = scale_px
    if not (math.isfinite(sx) and math.isfinite(sy) and sx > 0.0 and sy > 0.0):
        raise ValueError("normalization scale must be finite and positive")
    x = (points[:, 0] - center_px[0]) / sx
    y = (points[:, 1] - center_px[1]) / sy
    return np.column_stack(
        [np.power(x, x_power) * np.power(y, y_power) for x_power, y_power in terms]
    )


def convex_hull_area(points_xy: np.ndarray) -> float:
    points = sorted(set(map(tuple, np.asarray(points_xy, dtype=float).tolist())))
    if len(points) < 3:
        return 0.0

    def cross(origin: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (
            a[1] - origin[1]
        ) * (b[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    return 0.5 * abs(
        sum(
            first[0] * second[1] - second[0] * first[1]
            for first, second in zip(hull, hull[1:] + hull[:1])
        )
    )


def error_stats(errors_mm: np.ndarray) -> dict[str, float]:
    values = np.asarray(errors_mm, dtype=float)
    if values.ndim != 1 or not len(values) or not np.all(np.isfinite(values)):
        raise ValueError("errors must be a non-empty finite vector")
    return {
        "median_mm": float(np.median(values)),
        "p95_mm": float(np.percentile(values, 95)),
        "max_mm": float(np.max(values)),
    }


@dataclass(frozen=True)
class BoardMetricGeometry:
    canonical_size_px: int
    reference_plane_absolute_height_mm: float
    reference_plane_provenance: dict[str, str]
    board_layout_sha256: str
    source_manifest_sha256: str
    degree: int
    terms: tuple[tuple[int, int], ...]
    center_px: tuple[float, float]
    scale_px: tuple[float, float]
    x_coefficients_mm: tuple[float, ...]
    y_coefficients_mm: tuple[float, ...]
    calibration_hull_mm2: float
    fit_stats: dict[str, float]
    validation_stats: dict[str, float]
    qualification: dict[str, Any]
    reference_points: tuple[dict[str, Any], ...]

    @property
    def safety_qualified(self) -> bool:
        return self.qualification.get("safety_qualified") is True

    def map_points(self, canonical_xy_px: np.ndarray) -> np.ndarray:
        matrix = design_matrix(
            canonical_xy_px,
            self.center_px,
            self.scale_px,
            self.terms,
        )
        mapped = np.column_stack(
            [
                np.sum(
                    matrix * np.asarray(self.x_coefficients_mm, dtype=float),
                    axis=1,
                ),
                np.sum(
                    matrix * np.asarray(self.y_coefficients_mm, dtype=float),
                    axis=1,
                ),
            ]
        )
        if not np.all(np.isfinite(mapped)):
            raise ValueError("board metric map produced non-finite coordinates")
        return mapped

    def map_point(self, canonical_xy_px: Sequence[float]) -> tuple[float, float]:
        mapped = self.map_points(np.asarray([canonical_xy_px], dtype=float))[0]
        return float(mapped[0]), float(mapped[1])


def fit_geometry(
    canonical_xy_px: np.ndarray,
    board_xy_mm: np.ndarray,
    validation_mask: np.ndarray,
    *,
    canonical_size_px: int,
    reference_plane_absolute_height_mm: float,
    reference_plane_provenance_value: Any,
    board_layout_sha256: str,
    source_manifest_sha256: str,
    degree: int = 3,
) -> BoardMetricGeometry:
    observed = np.asarray(canonical_xy_px, dtype=float)
    expected = np.asarray(board_xy_mm, dtype=float)
    validation = np.asarray(validation_mask, dtype=bool)
    if observed.shape != expected.shape or observed.ndim != 2 or observed.shape[1] != 2:
        raise ValueError("canonical and board points must be matching Nx2 arrays")
    if len(observed) < MINIMUM_POINTS:
        raise ValueError(f"at least {MINIMUM_POINTS} distributed points are required")
    if validation.shape != (len(observed),):
        raise ValueError("validation_mask must have one entry per point")
    if np.count_nonzero(validation) < MINIMUM_VALIDATION_POINTS:
        raise ValueError(
            f"at least {MINIMUM_VALIDATION_POINTS} held-out points are required"
        )
    train = ~validation
    terms = polynomial_terms(degree)
    if np.count_nonzero(train) < max(MINIMUM_POINTS - MINIMUM_VALIDATION_POINTS, len(terms) * 2):
        raise ValueError("too few fit points for the requested polynomial")
    if not np.all(np.isfinite(observed)) or not np.all(np.isfinite(expected)):
        raise ValueError("calibration points must be finite")
    size = _positive_integer(canonical_size_px, "canonical_size_px")
    height = _finite(
        reference_plane_absolute_height_mm,
        "reference_plane_absolute_height_mm",
    )
    if height < 0.0:
        raise ValueError("reference plane height must be non-negative")
    provenance = reference_plane_provenance(reference_plane_provenance_value)
    layout_digest = _digest(board_layout_sha256, "board_layout_sha256")
    manifest_digest = _digest(source_manifest_sha256, "source_manifest_sha256")
    center = ((size - 1.0) / 2.0, (size - 1.0) / 2.0)
    scale = (max(center[0], 1.0), max(center[1], 1.0))
    matrix = design_matrix(observed, center, scale, terms)
    condition = float(np.linalg.cond(matrix[train]))
    x_coefficients, _, rank_x, _ = np.linalg.lstsq(
        matrix[train], expected[train, 0], rcond=None
    )
    y_coefficients, _, rank_y, _ = np.linalg.lstsq(
        matrix[train], expected[train, 1], rcond=None
    )
    if rank_x != len(terms) or rank_y != len(terms):
        raise ValueError("calibration design matrix is rank deficient")
    predicted = np.column_stack(
        [matrix @ x_coefficients, matrix @ y_coefficients]
    )
    errors = np.linalg.norm(predicted - expected, axis=1)
    fit_stats = error_stats(errors[train])
    validation_stats = error_stats(errors[validation])
    hull_area = convex_hull_area(expected)
    failures: list[str] = []
    if condition > MAXIMUM_CONDITION_NUMBER:
        failures.append("fit design is ill-conditioned")
    if hull_area < MINIMUM_HULL_AREA_MM2:
        failures.append("known points do not span enough of the maze")
    if fit_stats["p95_mm"] > MAXIMUM_FIT_P95_MM:
        failures.append("fit p95 error exceeds 0.75 mm")
    if fit_stats["max_mm"] > MAXIMUM_FIT_ERROR_MM:
        failures.append("fit maximum error exceeds 1.5 mm")
    if validation_stats["p95_mm"] > MAXIMUM_VALIDATION_P95_MM:
        failures.append("held-out p95 error exceeds 1.0 mm")
    if validation_stats["max_mm"] > MAXIMUM_VALIDATION_ERROR_MM:
        failures.append("held-out maximum error exceeds 1.5 mm")
    return BoardMetricGeometry(
        canonical_size_px=size,
        reference_plane_absolute_height_mm=height,
        reference_plane_provenance=provenance,
        board_layout_sha256=layout_digest,
        source_manifest_sha256=manifest_digest,
        degree=degree,
        terms=terms,
        center_px=center,
        scale_px=scale,
        x_coefficients_mm=tuple(map(float, x_coefficients)),
        y_coefficients_mm=tuple(map(float, y_coefficients)),
        calibration_hull_mm2=hull_area,
        fit_stats=fit_stats,
        validation_stats=validation_stats,
        qualification={
            "safety_qualified": not failures,
            "failures": failures,
            "condition_number": condition,
            "limits": {
                "minimum_points": MINIMUM_POINTS,
                "minimum_validation_points": MINIMUM_VALIDATION_POINTS,
                "minimum_hull_area_mm2": MINIMUM_HULL_AREA_MM2,
                "maximum_fit_p95_mm": MAXIMUM_FIT_P95_MM,
                "maximum_fit_error_mm": MAXIMUM_FIT_ERROR_MM,
                "maximum_validation_p95_mm": MAXIMUM_VALIDATION_P95_MM,
                "maximum_validation_error_mm": MAXIMUM_VALIDATION_ERROR_MM,
                "maximum_condition_number": MAXIMUM_CONDITION_NUMBER,
            },
        },
        reference_points=tuple(
            point_records(observed, expected, validation, predicted)
        ),
    )


def to_json(geometry: BoardMetricGeometry) -> dict[str, Any]:
    return {
        "schema": SCHEMA,
        "coordinate_system": COORDINATE_SYSTEM,
        "canonical_size_px": geometry.canonical_size_px,
        "reference_plane_absolute_height_mm": (
            geometry.reference_plane_absolute_height_mm
        ),
        "reference_plane_provenance": geometry.reference_plane_provenance,
        "bindings": {
            "board_layout_sha256": geometry.board_layout_sha256,
            "source_manifest_sha256": geometry.source_manifest_sha256,
        },
        "model": {
            "kind": "total_degree_polynomial",
            "degree": geometry.degree,
            "terms": [list(item) for item in geometry.terms],
            "normalization": {
                "center_px": list(geometry.center_px),
                "scale_px": list(geometry.scale_px),
            },
            "x_coefficients_mm": list(geometry.x_coefficients_mm),
            "y_coefficients_mm": list(geometry.y_coefficients_mm),
        },
        "evidence": {
            "calibration_hull_mm2": geometry.calibration_hull_mm2,
            "fit": geometry.fit_stats,
            "held_out": geometry.validation_stats,
            "points": list(geometry.reference_points),
        },
        "qualification": geometry.qualification,
    }


def _load_pair(value: Any, field: str) -> tuple[float, float]:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{field} must be a two-element array")
    return _finite(value[0], f"{field}[0]"), _finite(value[1], f"{field}[1]")


def load_geometry(
    path: Path,
    *,
    expected_board_layout_sha256: str | None = None,
    expected_canonical_size_px: int | None = None,
    require_safety_qualified: bool = True,
) -> BoardMetricGeometry:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid board metric geometry JSON: {exc}") from exc
    if not isinstance(raw, dict) or raw.get("schema") != SCHEMA:
        raise ValueError(f"board metric geometry schema must be {SCHEMA}")
    if raw.get("coordinate_system") != COORDINATE_SYSTEM:
        raise ValueError(f"coordinate_system must be {COORDINATE_SYSTEM}")
    size = _positive_integer(raw.get("canonical_size_px"), "canonical_size_px")
    height = _finite(
        raw.get("reference_plane_absolute_height_mm"),
        "reference_plane_absolute_height_mm",
    )
    if height < 0.0:
        raise ValueError("reference plane height must be non-negative")
    provenance = reference_plane_provenance(raw.get("reference_plane_provenance"))
    bindings = raw.get("bindings")
    if not isinstance(bindings, dict):
        raise ValueError("bindings must be an object")
    layout_digest = _digest(
        bindings.get("board_layout_sha256"), "bindings.board_layout_sha256"
    )
    source_digest = _digest(
        bindings.get("source_manifest_sha256"), "bindings.source_manifest_sha256"
    )
    if expected_board_layout_sha256 is not None and layout_digest != _digest(
        expected_board_layout_sha256, "expected_board_layout_sha256"
    ):
        raise ValueError("board metric geometry board-layout SHA mismatch")
    if expected_canonical_size_px is not None and size != expected_canonical_size_px:
        raise ValueError("board metric geometry canonical size mismatch")
    model = raw.get("model")
    if not isinstance(model, dict) or model.get("kind") != "total_degree_polynomial":
        raise ValueError("model must be total_degree_polynomial")
    degree = _positive_integer(model.get("degree"), "model.degree")
    expected_terms = polynomial_terms(degree)
    terms_raw = model.get("terms")
    if terms_raw != [list(item) for item in expected_terms]:
        raise ValueError("model.terms do not match the declared degree")
    normalization = model.get("normalization")
    if not isinstance(normalization, dict):
        raise ValueError("model.normalization must be an object")
    center = _load_pair(normalization.get("center_px"), "model.normalization.center_px")
    scale = _load_pair(normalization.get("scale_px"), "model.normalization.scale_px")
    if scale[0] <= 0.0 or scale[1] <= 0.0:
        raise ValueError("normalization scale must be positive")
    x_coefficients = model.get("x_coefficients_mm")
    y_coefficients = model.get("y_coefficients_mm")
    if not isinstance(x_coefficients, list) or not isinstance(y_coefficients, list):
        raise ValueError("model coefficients must be arrays")
    if len(x_coefficients) != len(expected_terms) or len(y_coefficients) != len(expected_terms):
        raise ValueError("model coefficient count does not match degree")
    x_coefficients_tuple = tuple(
        _finite(value, f"model.x_coefficients_mm[{index}]")
        for index, value in enumerate(x_coefficients)
    )
    y_coefficients_tuple = tuple(
        _finite(value, f"model.y_coefficients_mm[{index}]")
        for index, value in enumerate(y_coefficients)
    )
    evidence = raw.get("evidence")
    if not isinstance(evidence, dict):
        raise ValueError("evidence must be an object")
    hull = _finite(evidence.get("calibration_hull_mm2"), "evidence.calibration_hull_mm2")
    fit_stats = evidence.get("fit")
    validation_stats = evidence.get("held_out")
    if not isinstance(fit_stats, dict) or not isinstance(validation_stats, dict):
        raise ValueError("evidence fit and held_out must be objects")
    for name, stats in (("fit", fit_stats), ("held_out", validation_stats)):
        for key in ("median_mm", "p95_mm", "max_mm"):
            _finite(stats.get(key), f"evidence.{name}.{key}")
    qualification = raw.get("qualification")
    if not isinstance(qualification, dict):
        raise ValueError("qualification must be an object")
    if require_safety_qualified and qualification.get("safety_qualified") is not True:
        raise ValueError("board metric geometry is not safety-qualified")
    points_raw = evidence.get("points")
    if not isinstance(points_raw, list) or len(points_raw) < MINIMUM_POINTS:
        raise ValueError(
            f"evidence.points must contain at least {MINIMUM_POINTS} records"
        )
    reference_points: list[dict[str, Any]] = []
    held_out_count = 0
    point_ids: set[str] = set()
    for index, item in enumerate(points_raw):
        if not isinstance(item, dict):
            raise ValueError(f"evidence.points[{index}] must be an object")
        canonical = _load_pair(
            item.get("canonical_xy_px"),
            f"evidence.points[{index}].canonical_xy_px",
        )
        board = _load_pair(
            item.get("board_xy_mm"),
            f"evidence.points[{index}].board_xy_mm",
        )
        role = item.get("role")
        if role not in ("fit", "held_out"):
            raise ValueError(
                f"evidence.points[{index}].role must be fit or held_out"
            )
        held_out_count += role == "held_out"
        point_id = item.get("point_id", f"point-{index}")
        if not isinstance(point_id, str) or not point_id:
            raise ValueError(f"evidence.points[{index}].point_id is invalid")
        if point_id in point_ids:
            raise ValueError(f"duplicate evidence point ID: {point_id}")
        point_ids.add(point_id)
        reference_points.append(
            {
                "point_id": point_id,
                "canonical_xy_px": list(canonical),
                "board_xy_mm": list(board),
                "role": role,
            }
        )
    if held_out_count < MINIMUM_VALIDATION_POINTS:
        raise ValueError(
            f"evidence.points must contain at least {MINIMUM_VALIDATION_POINTS} held-out records"
        )
    if len(reference_points) - held_out_count < (
        MINIMUM_POINTS - MINIMUM_VALIDATION_POINTS
    ):
        raise ValueError("evidence.points has too few fit records")
    geometry = BoardMetricGeometry(
        canonical_size_px=size,
        reference_plane_absolute_height_mm=height,
        reference_plane_provenance=provenance,
        board_layout_sha256=layout_digest,
        source_manifest_sha256=source_digest,
        degree=degree,
        terms=expected_terms,
        center_px=center,
        scale_px=scale,
        x_coefficients_mm=x_coefficients_tuple,
        y_coefficients_mm=y_coefficients_tuple,
        calibration_hull_mm2=hull,
        fit_stats={key: float(value) for key, value in fit_stats.items()},
        validation_stats={key: float(value) for key, value in validation_stats.items()},
        qualification=qualification,
        reference_points=tuple(reference_points),
    )
    canonical_points = np.asarray(
        [item["canonical_xy_px"] for item in reference_points], dtype=float
    )
    expected_points = np.asarray(
        [item["board_xy_mm"] for item in reference_points], dtype=float
    )
    validation_mask = np.asarray(
        [item["role"] == "held_out" for item in reference_points], dtype=bool
    )
    if len(np.unique(expected_points, axis=0)) != len(expected_points):
        raise ValueError("evidence.points contains duplicate board coordinates")
    mapped_points = geometry.map_points(canonical_points)
    measured_errors = np.linalg.norm(mapped_points - expected_points, axis=1)
    measured_fit = error_stats(measured_errors[~validation_mask])
    measured_validation = error_stats(measured_errors[validation_mask])
    measured_hull = convex_hull_area(expected_points)
    measured_condition = float(
        np.linalg.cond(
            design_matrix(
                canonical_points,
                geometry.center_px,
                geometry.scale_px,
                geometry.terms,
            )[~validation_mask]
        )
    )
    for stored_name, stored, measured in (
        ("fit", geometry.fit_stats, measured_fit),
        ("held_out", geometry.validation_stats, measured_validation),
    ):
        for key in ("median_mm", "p95_mm", "max_mm"):
            if not math.isclose(
                float(stored[key]), measured[key], rel_tol=0.0, abs_tol=1e-6
            ):
                raise ValueError(f"evidence.{stored_name}.{key} is inconsistent")
    if not math.isclose(hull, measured_hull, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError("evidence.calibration_hull_mm2 is inconsistent")
    limits = qualification.get("limits")
    expected_limits = {
        "minimum_points": MINIMUM_POINTS,
        "minimum_validation_points": MINIMUM_VALIDATION_POINTS,
        "minimum_hull_area_mm2": MINIMUM_HULL_AREA_MM2,
        "maximum_fit_p95_mm": MAXIMUM_FIT_P95_MM,
        "maximum_fit_error_mm": MAXIMUM_FIT_ERROR_MM,
        "maximum_validation_p95_mm": MAXIMUM_VALIDATION_P95_MM,
        "maximum_validation_error_mm": MAXIMUM_VALIDATION_ERROR_MM,
        "maximum_condition_number": MAXIMUM_CONDITION_NUMBER,
    }
    if limits != expected_limits:
        raise ValueError("qualification.limits do not match the hard policy")
    stored_condition = _finite(
        qualification.get("condition_number"),
        "qualification.condition_number",
    )
    if not math.isclose(
        stored_condition, measured_condition, rel_tol=1e-9, abs_tol=1e-9
    ):
        raise ValueError("qualification.condition_number is inconsistent")
    measured_qualified = (
        measured_condition <= MAXIMUM_CONDITION_NUMBER
        and measured_hull >= MINIMUM_HULL_AREA_MM2
        and measured_fit["p95_mm"] <= MAXIMUM_FIT_P95_MM
        and measured_fit["max_mm"] <= MAXIMUM_FIT_ERROR_MM
        and measured_validation["p95_mm"] <= MAXIMUM_VALIDATION_P95_MM
        and measured_validation["max_mm"] <= MAXIMUM_VALIDATION_ERROR_MM
    )
    if geometry.safety_qualified != measured_qualified:
        raise ValueError("qualification.safety_qualified is inconsistent")
    if geometry.safety_qualified and qualification.get("failures") != []:
        raise ValueError("qualified geometry must have an empty failure list")
    return geometry


def point_records(
    canonical_xy_px: np.ndarray,
    board_xy_mm: np.ndarray,
    validation_mask: Iterable[bool],
    mapped_xy_mm: np.ndarray,
) -> list[dict[str, Any]]:
    return [
        {
            "canonical_xy_px": list(map(float, observed)),
            "board_xy_mm": list(map(float, expected)),
            "role": "held_out" if bool(validation) else "fit",
            "mapped_xy_mm": list(map(float, mapped)),
            "error_mm": float(np.linalg.norm(mapped - expected)),
        }
        for observed, expected, validation, mapped in zip(
            canonical_xy_px, board_xy_mm, validation_mask, mapped_xy_mm
        )
    ]
