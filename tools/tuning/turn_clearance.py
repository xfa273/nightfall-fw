#!/usr/bin/env python3
"""Clearance-aware Nightfall turn design and video validation.

The firmware turn model remains the source of the nominal centre trajectory.
This module adds the missing physical layer: a full rectangular mouse
envelope, explicit wall panels and posts, a quantitative uncertainty budget,
and the same clearance metric for simulated and measured trajectories.

Coordinate convention follows ``turn_tune.py``:

* x is vehicle-right
* y is vehicle-forward
* positive heading is a left turn

The built-in scenes are local worst-case fixtures for one primitive.  A JSON
scene can be supplied when validating a particular physical maze layout.
Nothing in this tool writes firmware parameters.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import importlib.util
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence


EPS = 1.0e-9
DEFAULT_CELL_PITCH_MM = 90.0
DEFAULT_POST_SIZE_MM = 6.0
DEFAULT_WALL_THICKNESS_MM = 6.0
DEFAULT_ROBOT_FRONT_MM = 35.0
DEFAULT_ROBOT_REAR_MM = 35.0
DEFAULT_ROBOT_LEFT_MM = 19.5
DEFAULT_ROBOT_RIGHT_MM = 19.5
DEFAULT_FOOTPRINT_ARTIFACT = "tools/tuning/data/mini_r2_0_footprint.json"
MAX_OFFSET_PAIR_EVALUATIONS_PER_ALPHA = 4096


_TURN_TUNE_MODULE: Any = None


@dataclass(frozen=True)
class PoseSample:
    time_s: float
    phase: str
    x_mm: float
    y_mm: float
    theta_deg: float


@dataclass(frozen=True)
class Footprint:
    """Completed-machine extents from the tracked machine/turn centre.

    The defaults are the user-confirmed mini_r2_0 measurements recorded in
    ``tools/tuning/data/mini_r2_0_footprint.json``.
    """

    front_mm: float = DEFAULT_ROBOT_FRONT_MM
    rear_mm: float = DEFAULT_ROBOT_REAR_MM
    left_mm: float = DEFAULT_ROBOT_LEFT_MM
    right_mm: float = DEFAULT_ROBOT_RIGHT_MM

    @property
    def corner_radius_mm(self) -> float:
        return max(
            math.hypot(self.front_mm, self.left_mm),
            math.hypot(self.front_mm, self.right_mm),
            math.hypot(self.rear_mm, self.left_mm),
            math.hypot(self.rear_mm, self.right_mm),
        )

    def polygon(self, pose: PoseSample) -> tuple[tuple[float, float], ...]:
        heading = math.radians(pose.theta_deg)
        right = (math.cos(heading), math.sin(heading))
        forward = (-math.sin(heading), math.cos(heading))

        def point(right_mm: float, forward_mm: float) -> tuple[float, float]:
            return (
                pose.x_mm + right_mm * right[0] + forward_mm * forward[0],
                pose.y_mm + right_mm * right[1] + forward_mm * forward[1],
            )

        return (
            point(self.right_mm, self.front_mm),
            point(-self.left_mm, self.front_mm),
            point(-self.left_mm, -self.rear_mm),
            point(self.right_mm, -self.rear_mm),
        )


def footprint_basis(footprint: Footprint) -> dict[str, Any]:
    """Describe whether a report uses the measured mini_r2_0 default."""

    uses_measured_default = footprint == Footprint()
    return {
        "reference_point": "blue-label centre = machine/turn centre",
        "uses_measured_default": uses_measured_default,
        "measurement_artifact": (
            DEFAULT_FOOTPRINT_ARTIFACT if uses_measured_default else None
        ),
        "provenance": (
            "user-confirmed completed-machine measurement (2026-08-11)"
            if uses_measured_default
            else "command-line override; provenance not encoded by this report"
        ),
    }


@dataclass(frozen=True)
class ObstacleRect:
    obstacle_id: str
    kind: str
    min_x_mm: float
    min_y_mm: float
    max_x_mm: float
    max_y_mm: float

    def polygon(self) -> tuple[tuple[float, float], ...]:
        return (
            (self.min_x_mm, self.min_y_mm),
            (self.max_x_mm, self.min_y_mm),
            (self.max_x_mm, self.max_y_mm),
            (self.min_x_mm, self.max_y_mm),
        )


@dataclass(frozen=True)
class TurnTarget:
    x_right_mm: float
    y_forward_mm: float
    theta_deg: float


@dataclass(frozen=True)
class TurnScene:
    name: str
    start_x_mm: float
    start_y_mm: float
    start_heading_deg: float
    target: TurnTarget
    target_world_x_mm: float
    target_world_y_mm: float
    target_world_heading_deg: float
    obstacles: tuple[ObstacleRect, ...]
    notes: str


@dataclass(frozen=True)
class ClearanceBudget:
    mechanical_mm: float = 0.5
    position_or_model_mm: float = 3.0
    heading_deg: float = 0.5
    interpolation_mm: float = 0.125

    def components(self, footprint: Footprint) -> dict[str, float]:
        return {
            "mechanical_mm": self.mechanical_mm,
            "position_or_model_mm": self.position_or_model_mm,
            "heading_mm": (
                math.radians(self.heading_deg) * footprint.corner_radius_mm
            ),
            "interpolation_mm": self.interpolation_mm,
        }

    def total_mm(self, footprint: Footprint) -> float:
        return sum(self.components(footprint).values())


@dataclass(frozen=True)
class PairDistance:
    signed_distance_mm: float
    robot_point: tuple[float, float]
    obstacle_point: tuple[float, float]
    robot_feature: str


@dataclass(frozen=True)
class ClearanceResult:
    raw_min_clearance_mm: float
    uncertainty_mm: float
    effective_min_clearance_mm: float
    required_margin_mm: float
    margin_passed: bool
    physical_collision: bool
    first_margin_violation_time_s: Optional[float]
    first_margin_violation_theta_deg: Optional[float]
    first_collision_time_s: Optional[float]
    first_collision_theta_deg: Optional[float]
    first_collision_obstacle_id: Optional[str]
    first_collision_robot_feature: Optional[str]
    worst_time_s: float
    worst_phase: str
    worst_x_mm: float
    worst_y_mm: float
    worst_theta_deg: float
    obstacle_id: str
    obstacle_kind: str
    robot_feature: str
    robot_closest_x_mm: float
    robot_closest_y_mm: float
    obstacle_closest_x_mm: float
    obstacle_closest_y_mm: float
    evaluated_pose_count: int


@dataclass(frozen=True)
class SearchCandidate:
    alpha_deg_s2: float
    dist_in_mm: float
    dist_out_mm: float
    endpoint_error_mm: float
    heading_error_deg: float
    parameter_change_norm: float
    duration_ms: int
    clearance: ClearanceResult
    clearance_model_slip_angle_coefficient: float
    model_scope_qualified: bool
    model_scope_violations: tuple[str, ...]
    final_x_mm: float
    final_y_mm: float
    final_theta_deg: float


def _load_turn_tune_module() -> Any:
    global _TURN_TUNE_MODULE
    if _TURN_TUNE_MODULE is not None:
        return _TURN_TUNE_MODULE
    path = Path(__file__).with_name("turn_tune.py")
    spec = importlib.util.spec_from_file_location("nightfall_turn_tune_clearance", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    _TURN_TUNE_MODULE = module
    return _TURN_TUNE_MODULE


def _canonical_geometry(
    code: int, cell_pitch_mm: float = DEFAULT_CELL_PITCH_MM
) -> tuple[float, float, float, bool]:
    half = 0.5 * cell_pitch_mm
    diagonal_half = half / math.sqrt(2.0)
    mapping: dict[int, tuple[float, float, float]] = {
        300: (half, half, 90.0),
        400: (half, half, 90.0),
        501: (2.0 * half, 2.0 * half, 90.0),
        601: (2.0 * half, 2.0 * half, 90.0),
        502: (0.0, 2.0 * half, 180.0),
        602: (0.0, 2.0 * half, 180.0),
        701: (2.0 * half, half, 45.0),
        702: (2.0 * half, half, 45.0),
        703: (3.0 * diagonal_half, diagonal_half, 45.0),
        704: (3.0 * diagonal_half, diagonal_half, 45.0),
        801: (math.sqrt(2.0) * half, math.sqrt(2.0) * half, 90.0),
        802: (math.sqrt(2.0) * half, math.sqrt(2.0) * half, 90.0),
        901: (half, 2.0 * half, 135.0),
        902: (half, 2.0 * half, 135.0),
        903: (diagonal_half, 3.0 * diagonal_half, 135.0),
        904: (diagonal_half, 3.0 * diagonal_half, 135.0),
    }
    if code not in mapping:
        raise ValueError(f"no canonical turn geometry for code {code}")
    forward_mm, lateral_abs_mm, angle_abs_deg = mapping[code]
    right = code in (300, 501, 502, 701, 703, 801, 901, 903)
    return forward_mm, lateral_abs_mm, angle_abs_deg, right


def canonical_target(
    code: int, cell_pitch_mm: float = DEFAULT_CELL_PITCH_MM
) -> TurnTarget:
    forward_mm, lateral_abs_mm, angle_abs_deg, right = _canonical_geometry(
        code, cell_pitch_mm
    )
    return TurnTarget(
        x_right_mm=lateral_abs_mm if right else -lateral_abs_mm,
        y_forward_mm=forward_mm,
        theta_deg=-angle_abs_deg if right else angle_abs_deg,
    )


def _transform_local(
    start_x_mm: float,
    start_y_mm: float,
    start_heading_deg: float,
    local_x_mm: float,
    local_y_mm: float,
) -> tuple[float, float]:
    heading = math.radians(start_heading_deg)
    right = (math.cos(heading), math.sin(heading))
    forward = (-math.sin(heading), math.cos(heading))
    return (
        start_x_mm + right[0] * local_x_mm + forward[0] * local_y_mm,
        start_y_mm + right[1] * local_x_mm + forward[1] * local_y_mm,
    )


def _post(obstacle_id: str, x_mm: float, y_mm: float, size_mm: float) -> ObstacleRect:
    half = 0.5 * size_mm
    return ObstacleRect(
        obstacle_id,
        "post",
        x_mm - half,
        y_mm - half,
        x_mm + half,
        y_mm + half,
    )


def _vertical_panel(
    obstacle_id: str,
    x_mm: float,
    y0_mm: float,
    y1_mm: float,
    thickness_mm: float,
) -> ObstacleRect:
    half = 0.5 * thickness_mm
    return ObstacleRect(
        obstacle_id,
        "wall",
        x_mm - half,
        min(y0_mm, y1_mm),
        x_mm + half,
        max(y0_mm, y1_mm),
    )


def _horizontal_panel(
    obstacle_id: str,
    y_mm: float,
    x0_mm: float,
    x1_mm: float,
    thickness_mm: float,
) -> ObstacleRect:
    half = 0.5 * thickness_mm
    return ObstacleRect(
        obstacle_id,
        "wall",
        min(x0_mm, x1_mm),
        y_mm - half,
        max(x0_mm, x1_mm),
        y_mm + half,
    )


def canonical_scene(
    code: int,
    *,
    cell_pitch_mm: float = DEFAULT_CELL_PITCH_MM,
    post_size_mm: float = DEFAULT_POST_SIZE_MM,
    wall_thickness_mm: float = DEFAULT_WALL_THICKNESS_MM,
) -> TurnScene:
    """Build a local conservative wall/post fixture for one turn primitive."""
    target = canonical_target(code, cell_pitch_mm)
    _, _, _, right = _canonical_geometry(code, cell_pitch_mm)
    side = 1.0 if right else -1.0
    out_turn = code in (703, 704, 801, 802, 903, 904)
    start_x = 0.0
    start_y = cell_pitch_mm if out_turn else 0.5 * cell_pitch_mm
    if code in (300, 400):
        # Small-90 connects perpendicular wall-opening midpoints.  Starting it
        # at the large-turn cell-centre parity would put its canonical endpoint
        # on a maze post.
        start_y = 0.0
    start_heading = (-45.0 if right else 45.0) if out_turn else 0.0
    target_world_x, target_world_y = _transform_local(
        start_x,
        start_y,
        start_heading,
        target.x_right_mm,
        target.y_forward_mm,
    )

    obstacles: list[ObstacleRect] = []
    pitch = cell_pitch_mm
    post_x = (-1.5 * pitch, -0.5 * pitch, 0.5 * pitch, 1.5 * pitch)
    post_y = (-pitch, 0.0, pitch, 2.0 * pitch, 3.0 * pitch)
    for x_index, x_mm in enumerate(post_x):
        for y_index, y_mm in enumerate(post_y):
            obstacles.append(
                _post(f"post-x{x_index}-y{y_index}", x_mm, y_mm, post_size_mm)
            )

    panel_lo = 0.5 * post_size_mm
    panel_hi = pitch - 0.5 * post_size_mm
    if code in (501, 601, 502, 602, 701, 702, 901, 902):
        obstacles.append(
            _vertical_panel(
                "approach-inner-wall",
                side * 0.5 * pitch,
                panel_lo,
                panel_hi,
                wall_thickness_mm,
            )
        )
        obstacles.append(
            _vertical_panel(
                "approach-outer-wall",
                -side * 0.5 * pitch,
                panel_lo,
                panel_hi,
                wall_thickness_mm,
            )
        )
    if code in (501, 601, 703, 704):
        obstacles.append(
            _horizontal_panel(
                "exit-inner-wall",
                pitch,
                side * (0.5 * pitch + 0.5 * post_size_mm),
                side * (1.5 * pitch - 0.5 * post_size_mm),
                wall_thickness_mm,
            )
        )
    if code in (903, 904):
        obstacles.append(
            _vertical_panel(
                "exit-inner-wall",
                side * 0.5 * pitch,
                panel_lo,
                panel_hi,
                wall_thickness_mm,
            )
        )

    if code in (300, 400):
        # Union of every locally legal panel except the cardinal approach
        # opening and the perpendicular exit opening.  Minimum clearance to
        # this union equals the worst clearance over the legal wall patterns.
        post_half = 0.5 * post_size_mm
        for y_index, y_mm in enumerate(post_y):
            for x_index in range(len(post_x) - 1):
                required_approach_opening = (
                    abs(y_mm) <= EPS
                    and abs(post_x[x_index] + 0.5 * pitch) <= EPS
                    and abs(post_x[x_index + 1] - 0.5 * pitch) <= EPS
                )
                if not required_approach_opening:
                    obstacles.append(
                        _horizontal_panel(
                            f"small-h-x{x_index}-y{y_index}",
                            y_mm,
                            post_x[x_index] + post_half,
                            post_x[x_index + 1] - post_half,
                            wall_thickness_mm,
                        )
                    )
        exit_x = side * 0.5 * pitch
        for x_index, x_mm in enumerate(post_x):
            for y_index in range(len(post_y) - 1):
                required_exit_opening = (
                    abs(x_mm - exit_x) <= EPS
                    and abs(post_y[y_index]) <= EPS
                    and abs(post_y[y_index + 1] - pitch) <= EPS
                )
                if not required_exit_opening:
                    obstacles.append(
                        _vertical_panel(
                            f"small-v-x{x_index}-y{y_index}",
                            x_mm,
                            post_y[y_index] + post_half,
                            post_y[y_index + 1] - post_half,
                            wall_thickness_mm,
                        )
                    )

    notes = (
        "All nearby 6 mm posts are explicit. Wall panels represent a "
        "conservative legal inner/outer local fixture; validate the final "
        "candidate against the actual test-maze scene and video."
    )
    if code in (300, 400):
        notes = (
            "Small-90 uses the union of every local panel legal with its two "
            "required openings. Runtime wall-end placement still requires a "
            "measured --scene-json and video for final acceptance."
        )
    return TurnScene(
        name=f"canonical-code{code}",
        start_x_mm=start_x,
        start_y_mm=start_y,
        start_heading_deg=start_heading,
        target=target,
        target_world_x_mm=target_world_x,
        target_world_y_mm=target_world_y,
        target_world_heading_deg=start_heading + target.theta_deg,
        obstacles=tuple(obstacles),
        notes=notes,
    )


def load_scene(path: Path) -> TurnScene:
    raw = json.loads(path.read_text(encoding="utf-8"))
    start = raw["start_pose"]
    target = raw["target_pose"]
    start_x = float(start["x_mm"])
    start_y = float(start["y_mm"])
    start_heading = float(start["theta_deg"])
    target_world_x = float(target["x_mm"])
    target_world_y = float(target["y_mm"])
    target_world_heading = float(target["theta_deg"])
    if not all(
        math.isfinite(value)
        for value in (
            start_x,
            start_y,
            start_heading,
            target_world_x,
            target_world_y,
            target_world_heading,
        )
    ):
        raise ValueError("scene start/target poses must be finite")
    local = raw.get("target_local")
    if local is None:
        dx = target_world_x - start_x
        dy = target_world_y - start_y
        heading = math.radians(start_heading)
        local = {
            "x_right_mm": math.cos(heading) * dx + math.sin(heading) * dy,
            "y_forward_mm": -math.sin(heading) * dx + math.cos(heading) * dy,
            "theta_deg": _angle_error_deg(target_world_heading, start_heading),
        }
    local_x = float(local["x_right_mm"])
    local_y = float(local["y_forward_mm"])
    local_heading = float(local["theta_deg"])
    if not all(math.isfinite(value) for value in (local_x, local_y, local_heading)):
        raise ValueError("scene target_local must be finite")
    expected_world_x, expected_world_y = _transform_local(
        start_x, start_y, start_heading, local_x, local_y
    )
    expected_world_heading = start_heading + local_heading
    if (
        math.hypot(
            target_world_x - expected_world_x,
            target_world_y - expected_world_y,
        )
        > 1.0e-6
        or abs(_angle_error_deg(target_world_heading, expected_world_heading))
        > 1.0e-6
    ):
        raise ValueError("scene target_local and target_pose are inconsistent")
    obstacles_list = []
    obstacle_ids = set()
    for item in raw["obstacles"]:
        obstacle_id = item.get("id", item.get("obstacle_id"))
        if obstacle_id is None:
            raise ValueError("scene obstacle is missing id")
        obstacle_id = str(obstacle_id)
        if not obstacle_id or obstacle_id in obstacle_ids:
            raise ValueError("scene obstacle ids must be non-empty and unique")
        obstacle_ids.add(obstacle_id)
        kind = str(item["kind"])
        if kind not in ("post", "wall"):
            raise ValueError(f"scene obstacle {obstacle_id} has invalid kind {kind}")
        bounds = tuple(
            float(item[name])
            for name in ("min_x_mm", "min_y_mm", "max_x_mm", "max_y_mm")
        )
        if not all(math.isfinite(value) for value in bounds):
            raise ValueError(f"scene obstacle {obstacle_id} bounds must be finite")
        if bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
            raise ValueError(f"scene obstacle {obstacle_id} bounds are inverted")
        obstacles_list.append(
            ObstacleRect(
                obstacle_id,
                kind,
                *bounds,
            )
        )
    obstacles = tuple(obstacles_list)
    return TurnScene(
        name=str(raw.get("name", path.stem)),
        start_x_mm=start_x,
        start_y_mm=start_y,
        start_heading_deg=start_heading,
        target=TurnTarget(
            local_x,
            local_y,
            local_heading,
        ),
        target_world_x_mm=target_world_x,
        target_world_y_mm=target_world_y,
        target_world_heading_deg=target_world_heading,
        obstacles=obstacles,
        notes=str(raw.get("notes", "user-supplied scene")),
    )


def scene_to_dict(scene: TurnScene) -> dict[str, Any]:
    return {
        "name": scene.name,
        "start_pose": {
            "x_mm": scene.start_x_mm,
            "y_mm": scene.start_y_mm,
            "theta_deg": scene.start_heading_deg,
        },
        "target_local": asdict(scene.target),
        "target_pose": {
            "x_mm": scene.target_world_x_mm,
            "y_mm": scene.target_world_y_mm,
            "theta_deg": scene.target_world_heading_deg,
        },
        "obstacles": [
            {
                "id": item.obstacle_id,
                "kind": item.kind,
                "min_x_mm": item.min_x_mm,
                "min_y_mm": item.min_y_mm,
                "max_x_mm": item.max_x_mm,
                "max_y_mm": item.max_y_mm,
            }
            for item in scene.obstacles
        ],
        "notes": scene.notes,
    }


def _axes(polygon: Sequence[tuple[float, float]]) -> Iterable[tuple[float, float]]:
    for index, point in enumerate(polygon):
        other = polygon[(index + 1) % len(polygon)]
        dx = other[0] - point[0]
        dy = other[1] - point[1]
        length = math.hypot(dx, dy)
        if length > EPS:
            yield (-dy / length, dx / length)


def _project(
    polygon: Sequence[tuple[float, float]], axis: tuple[float, float]
) -> tuple[float, float]:
    values = [point[0] * axis[0] + point[1] * axis[1] for point in polygon]
    return min(values), max(values)


def _closest_point_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[tuple[float, float], float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator <= EPS:
        closest = start
    else:
        amount = (
            (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
        ) / denominator
        amount = min(1.0, max(0.0, amount))
        closest = (start[0] + amount * dx, start[1] + amount * dy)
    return closest, math.hypot(point[0] - closest[0], point[1] - closest[1])


def _feature_name(
    point: tuple[float, float], polygon: Sequence[tuple[float, float]]
) -> str:
    corners = ("front-right", "front-left", "rear-left", "rear-right")
    corner_distances = [math.dist(point, corner) for corner in polygon]
    closest_corner = min(range(4), key=corner_distances.__getitem__)
    if corner_distances[closest_corner] <= 1.0e-6:
        return corners[closest_corner]
    edges = ("front-edge", "left-edge", "rear-edge", "right-edge")
    edge_distances = []
    for index in range(4):
        _, distance = _closest_point_segment(
            point, polygon[index], polygon[(index + 1) % 4]
        )
        edge_distances.append(distance)
    return edges[min(range(4), key=edge_distances.__getitem__)]


def polygon_signed_distance(
    robot: Sequence[tuple[float, float]],
    obstacle: Sequence[tuple[float, float]],
) -> PairDistance:
    separated = False
    minimum_penetration = math.inf
    for axis in (*tuple(_axes(robot)), *tuple(_axes(obstacle))):
        robot_min, robot_max = _project(robot, axis)
        obstacle_min, obstacle_max = _project(obstacle, axis)
        if robot_max < obstacle_min - EPS or obstacle_max < robot_min - EPS:
            separated = True
        else:
            translation = min(
                robot_max - obstacle_min,
                obstacle_max - robot_min,
            )
            minimum_penetration = min(minimum_penetration, translation)

    best_distance = math.inf
    best_robot = robot[0]
    best_obstacle = obstacle[0]
    for robot_point in robot:
        for index, obstacle_start in enumerate(obstacle):
            obstacle_end = obstacle[(index + 1) % len(obstacle)]
            closest, distance = _closest_point_segment(
                robot_point, obstacle_start, obstacle_end
            )
            if distance < best_distance:
                best_distance = distance
                best_robot = robot_point
                best_obstacle = closest
    for obstacle_point in obstacle:
        for index, robot_start in enumerate(robot):
            robot_end = robot[(index + 1) % len(robot)]
            closest, distance = _closest_point_segment(
                obstacle_point, robot_start, robot_end
            )
            if distance < best_distance:
                best_distance = distance
                best_robot = closest
                best_obstacle = obstacle_point

    if separated:
        signed = best_distance
    else:
        signed = -max(0.0, minimum_penetration)
    return PairDistance(
        signed,
        best_robot,
        best_obstacle,
        _feature_name(best_robot, robot),
    )


def _interpolate_pose(a: PoseSample, b: PoseSample, fraction: float) -> PoseSample:
    return PoseSample(
        time_s=a.time_s + (b.time_s - a.time_s) * fraction,
        phase=b.phase if fraction >= 0.5 else a.phase,
        x_mm=a.x_mm + (b.x_mm - a.x_mm) * fraction,
        y_mm=a.y_mm + (b.y_mm - a.y_mm) * fraction,
        theta_deg=a.theta_deg + (b.theta_deg - a.theta_deg) * fraction,
    )


def densify_poses(
    samples: Sequence[PoseSample],
    footprint: Footprint,
    max_corner_step_mm: float,
) -> list[PoseSample]:
    if not samples:
        raise ValueError("trajectory has no pose samples")
    if max_corner_step_mm <= 0.0:
        raise ValueError("max corner step must be positive")
    result = [samples[0]]
    for start, end in zip(samples, samples[1:]):
        translation = math.hypot(end.x_mm - start.x_mm, end.y_mm - start.y_mm)
        rotation = (
            footprint.corner_radius_mm
            * abs(math.radians(end.theta_deg - start.theta_deg))
        )
        intervals = max(1, int(math.ceil((translation + rotation) / max_corner_step_mm)))
        for index in range(1, intervals + 1):
            result.append(_interpolate_pose(start, end, index / intervals))
    return result


def _aabb_lower_bound(
    min_x: float,
    min_y: float,
    max_x: float,
    max_y: float,
    obstacle: ObstacleRect,
) -> float:
    dx = max(obstacle.min_x_mm - max_x, min_x - obstacle.max_x_mm, 0.0)
    dy = max(obstacle.min_y_mm - max_y, min_y - obstacle.max_y_mm, 0.0)
    return math.hypot(dx, dy)


def evaluate_clearance(
    samples: Sequence[PoseSample],
    scene: TurnScene,
    footprint: Footprint,
    budget: ClearanceBudget,
    required_margin_mm: float,
    max_corner_step_mm: float = 0.5,
) -> ClearanceResult:
    if not scene.obstacles:
        raise ValueError("scene has no obstacles")
    dense = densify_poses(samples, footprint, max_corner_step_mm)
    best_distance = math.inf
    best_pair: Optional[PairDistance] = None
    best_pose: Optional[PoseSample] = None
    best_obstacle: Optional[ObstacleRect] = None
    uncertainty = budget.total_mm(footprint)
    first_margin_violation: Optional[tuple[PoseSample, ObstacleRect, PairDistance]] = None
    first_collision: Optional[tuple[PoseSample, ObstacleRect, PairDistance]] = None
    radius = footprint.corner_radius_mm
    swept_min_x = min(pose.x_mm for pose in dense) - radius
    swept_max_x = max(pose.x_mm for pose in dense) + radius
    swept_min_y = min(pose.y_mm for pose in dense) - radius
    swept_max_y = max(pose.y_mm for pose in dense) + radius
    obstacles = sorted(
        (
            (
                _aabb_lower_bound(
                    swept_min_x,
                    swept_min_y,
                    swept_max_x,
                    swept_max_y,
                    obstacle,
                ),
                index,
                obstacle,
                obstacle.polygon(),
            )
            for index, obstacle in enumerate(scene.obstacles)
        ),
        key=lambda item: (item[0], item[1]),
    )
    for pose in dense:
        robot = footprint.polygon(pose)
        robot_min_x = min(point[0] for point in robot)
        robot_max_x = max(point[0] for point in robot)
        robot_min_y = min(point[1] for point in robot)
        robot_max_y = max(point[1] for point in robot)
        for _, _, obstacle, obstacle_polygon in obstacles:
            lower_bound = _aabb_lower_bound(
                robot_min_x,
                robot_min_y,
                robot_max_x,
                robot_max_y,
                obstacle,
            )
            # Once overlap has been found, every AABB-separated obstacle has
            # positive distance and therefore cannot improve a negative best.
            if lower_bound > max(best_distance, 0.0):
                continue
            pair = polygon_signed_distance(robot, obstacle_polygon)
            if (
                first_margin_violation is None
                and pair.signed_distance_mm - uncertainty < required_margin_mm
            ):
                first_margin_violation = (pose, obstacle, pair)
            if first_collision is None and pair.signed_distance_mm <= 0.0:
                first_collision = (pose, obstacle, pair)
            if pair.signed_distance_mm < best_distance:
                best_distance = pair.signed_distance_mm
                best_pair = pair
                best_pose = pose
                best_obstacle = obstacle
    assert best_pair is not None and best_pose is not None and best_obstacle is not None
    effective = best_distance - uncertainty
    first_margin_pose = (
        None if first_margin_violation is None else first_margin_violation[0]
    )
    first_collision_pose = None if first_collision is None else first_collision[0]
    return ClearanceResult(
        raw_min_clearance_mm=best_distance,
        uncertainty_mm=uncertainty,
        effective_min_clearance_mm=effective,
        required_margin_mm=required_margin_mm,
        margin_passed=effective + EPS >= required_margin_mm,
        physical_collision=best_distance <= 0.0,
        first_margin_violation_time_s=(
            None if first_margin_pose is None else first_margin_pose.time_s
        ),
        first_margin_violation_theta_deg=(
            None if first_margin_pose is None else first_margin_pose.theta_deg
        ),
        first_collision_time_s=(
            None if first_collision_pose is None else first_collision_pose.time_s
        ),
        first_collision_theta_deg=(
            None if first_collision_pose is None else first_collision_pose.theta_deg
        ),
        first_collision_obstacle_id=(
            None if first_collision is None else first_collision[1].obstacle_id
        ),
        first_collision_robot_feature=(
            None if first_collision is None else first_collision[2].robot_feature
        ),
        worst_time_s=best_pose.time_s,
        worst_phase=best_pose.phase,
        worst_x_mm=best_pose.x_mm,
        worst_y_mm=best_pose.y_mm,
        worst_theta_deg=best_pose.theta_deg,
        obstacle_id=best_obstacle.obstacle_id,
        obstacle_kind=best_obstacle.kind,
        robot_feature=best_pair.robot_feature,
        robot_closest_x_mm=best_pair.robot_point[0],
        robot_closest_y_mm=best_pair.robot_point[1],
        obstacle_closest_x_mm=best_pair.obstacle_point[0],
        obstacle_closest_y_mm=best_pair.obstacle_point[1],
        evaluated_pose_count=len(dense),
    )


def simulation_world_samples(sim: Any, scene: TurnScene) -> list[PoseSample]:
    local_samples = [
        PoseSample(0.0, "start", 0.0, 0.0, 0.0),
        *[
            PoseSample(
                sample.t_ms / 1000.0,
                sample.phase,
                sample.x_mm,
                sample.y_mm,
                sample.theta_deg,
            )
            for sample in sim.samples
        ],
    ]
    result = []
    for sample in local_samples:
        x_mm, y_mm = _transform_local(
            scene.start_x_mm,
            scene.start_y_mm,
            scene.start_heading_deg,
            sample.x_mm,
            sample.y_mm,
        )
        result.append(
            PoseSample(
                sample.time_s,
                sample.phase,
                x_mm,
                y_mm,
                scene.start_heading_deg + sample.theta_deg,
            )
        )
    return result


def _turn_cli(
    args: argparse.Namespace,
    overrides: Optional[dict[str, float]] = None,
    slip_angle_coefficient: Optional[float] = None,
) -> list[str]:
    values = overrides or {}
    cli = [
        "simulate",
        "--runner",
        "shortest",
        "--mode",
        str(args.mode),
        "--code",
        str(args.code),
        "--angle-policy",
        args.angle_policy,
        "--json",
    ]
    if args.shortest_params is not None:
        cli.extend(["--shortest-params", str(args.shortest_params)])
    for name in ("velocity", "alpha", "dist_in", "dist_out", "angle"):
        value = values.get(name, getattr(args, name, None))
        if value is not None:
            cli.extend(["--" + name.replace("_", "-"), str(value)])
    slip = (
        args.slip_angle_coefficient
        if slip_angle_coefficient is None
        else slip_angle_coefficient
    )
    cli.extend(["--slip-angle-coefficient", str(slip)])
    return cli


def resolve_simulation(
    args: argparse.Namespace,
    overrides: Optional[dict[str, float]] = None,
    slip_angle_coefficient: Optional[float] = None,
) -> tuple[Any, Any, Any]:
    turn_tune = _load_turn_tune_module()
    parsed = turn_tune.build_arg_parser().parse_args(
        _turn_cli(args, overrides, slip_angle_coefficient)
    )
    turn = turn_tune.resolve_turn(parsed)
    constants = turn_tune.load_constants(parsed)
    sim = turn_tune.simulate_turn(
        turn,
        constants,
        args.entry_speed,
        args.out_speed,
        parsed.slip_angle_coefficient,
    )
    return turn_tune, turn, sim


def _slip_model_scope(
    args: argparse.Namespace,
    turn: Any,
    slip_angle_coefficient: float,
) -> dict[str, Any]:
    """Return whether a trajectory model is inside a bound calibration scope."""
    nominal_zero_slip = abs(slip_angle_coefficient) <= EPS
    model_path = getattr(args, "slip_model_json", None)
    if (
        nominal_zero_slip
        and getattr(args, "command", None) == "video"
        and model_path is None
    ):
        return {
            "qualified": True,
            "kind": "nominal-video-extraction",
            "coefficient_s2_m": 0.0,
            "artifact": None,
            "violations": [],
        }

    if model_path is None:
        return {
            "qualified": False,
            "kind": (
                "nominal-zero-slip-unvalidated"
                if nominal_zero_slip
                else "unbound-empirical"
            ),
            "coefficient_s2_m": slip_angle_coefficient,
            "artifact": None,
            "violations": (
                [
                    "nominal zero-slip trajectory has no swept-path validation artifact"
                ]
                if nominal_zero_slip
                else [
                    "non-zero slip coefficient has no --slip-model-json calibration scope"
                ]
            ),
        }
    model = json.loads(model_path.read_text(encoding="utf-8"))
    if model.get("schema") != "nightfall_turn_empirical_sideslip_model_v1":
        raise ValueError(f"{model_path}: unsupported slip-model schema")
    try:
        calibrated = model["turn"]
        calibrated_coefficient = float(model["model"]["coefficient_s2_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{model_path}: invalid slip-model scope") from exc
    qualification = model.get("qualification")
    artifact_safety_qualified = (
        isinstance(qualification, dict)
        and qualification.get("safety_qualified") is True
    )

    side = "right" if turn.signed_angle_deg < 0.0 else "left"
    checks = (
        (
            artifact_safety_qualified,
            "calibration artifact is not safety-qualified for swept-clearance recommendations",
        ),
        (
            abs(slip_angle_coefficient - calibrated_coefficient) <= 1.0e-9,
            "coefficient differs from the bound calibration",
        ),
        (args.mode == int(calibrated["mode"]), "mode is outside calibration"),
        (args.code == int(calibrated["code"]), "turn code is outside calibration"),
        (side == calibrated["side"], "turn side is outside calibration"),
        (
            abs(turn.velocity_mm_s - float(calibrated["velocity_mm_s"])) <= 1.0e-6,
            "velocity is outside calibration",
        ),
        (
            abs(turn.alpha_deg_s2 - float(calibrated["alpha_deg_s2"])) <= 1.0e-6,
            "angular acceleration is outside calibration",
        ),
        (
            abs(abs(turn.signed_angle_deg) - float(calibrated["effective_angle_deg"]))
            <= 1.0e-6,
            "effective angle is outside calibration",
        ),
        (
            not calibrated.get("positive_entry_regime_only", False)
            or turn.dist_in_mm > EPS,
            "entry offset is outside the positive-entry calibration regime",
        ),
    )
    violations = [message for passed, message in checks if not passed]
    return {
        "qualified": not violations,
        "kind": "bound-nominal" if nominal_zero_slip else "bound-empirical",
        "coefficient_s2_m": slip_angle_coefficient,
        "artifact": str(model_path),
        "calibrated_turn": calibrated,
        "qualification": qualification,
        "violations": violations,
    }


def _combined_model_scope(
    args: argparse.Namespace,
    turn: Any,
    coefficients: Iterable[float],
) -> dict[str, Any]:
    scopes = [_slip_model_scope(args, turn, value) for value in coefficients]
    violations = [
        message
        for scope in scopes
        for message in scope["violations"]
    ]
    return {
        "qualified": all(scope["qualified"] for scope in scopes),
        "models": scopes,
        "violations": sorted(set(violations)),
    }


def _apply_model_scope(report: dict[str, Any], scope: dict[str, Any]) -> None:
    report["model"]["scope"] = scope
    report["acceptance"]["model_scope_qualified"] = scope["qualified"]
    report["acceptance"]["passed"] = (
        report["acceptance"]["passed"] and scope["qualified"]
    )
    report["acceptance"]["rule"] = (
        "clearance, endpoint, heading, and model-scope gates must all pass"
    )


def _footprint_from_args(args: argparse.Namespace) -> Footprint:
    footprint = Footprint(
        front_mm=args.robot_front_mm,
        rear_mm=args.robot_rear_mm,
        left_mm=args.robot_left_mm,
        right_mm=args.robot_right_mm,
    )
    if min(asdict(footprint).values()) <= 0.0:
        raise ValueError("all robot extents must be positive")
    return footprint


def _budget_from_args(args: argparse.Namespace) -> ClearanceBudget:
    values = (
        args.mechanical_uncertainty_mm,
        args.position_uncertainty_mm,
        args.heading_uncertainty_deg,
        0.5 * args.max_corner_step_mm,
    )
    if min(values) < 0.0:
        raise ValueError("uncertainty components must be non-negative")
    return ClearanceBudget(*values)


def _scene_from_args(args: argparse.Namespace) -> TurnScene:
    if args.scene_json is not None:
        return load_scene(args.scene_json)
    return canonical_scene(
        args.code,
        cell_pitch_mm=args.cell_pitch_mm,
        post_size_mm=args.post_size_mm,
        wall_thickness_mm=args.wall_thickness_mm,
    )


def _sim_report(
    sim: Any,
    scene: TurnScene,
    footprint: Footprint,
    budget: ClearanceBudget,
    clearance: ClearanceResult,
    maximum_endpoint_error_mm: float,
    maximum_heading_error_deg: float,
) -> dict[str, Any]:
    endpoint = {
        "x_right_mm": sim.final_pose.x_mm,
        "y_forward_mm": sim.final_pose.y_mm,
        "theta_deg": sim.final_pose.theta_deg,
    }
    endpoint_distance_error = math.hypot(
        endpoint["x_right_mm"] - scene.target.x_right_mm,
        endpoint["y_forward_mm"] - scene.target.y_forward_mm,
    )
    heading_error = endpoint["theta_deg"] - scene.target.theta_deg
    endpoint_passed = endpoint_distance_error <= maximum_endpoint_error_mm
    heading_passed = abs(heading_error) <= maximum_heading_error_deg
    return {
        "schema": "nightfall_turn_clearance_report_v1",
        "source": "firmware-nominal-simulation",
        "turn": asdict(sim.turn),
        "model": {
            "slip_angle_coefficient": sim.slip_angle_coefficient,
            "slip_angle_formula": (
                "velocity_heading_lag_deg = omega_deg_s * speed_m_s * K"
            ),
        },
        "duration_ms": sim.duration_ms,
        "endpoint": endpoint,
        "endpoint_error": {
            "distance_mm": endpoint_distance_error,
            "heading_deg": heading_error,
        },
        "scene": scene_to_dict(scene),
        "footprint": asdict(footprint),
        "footprint_basis": footprint_basis(footprint),
        "uncertainty": {
            **budget.components(footprint),
            "total_mm": budget.total_mm(footprint),
        },
        "clearance": asdict(clearance),
        "acceptance": {
            "passed": clearance.margin_passed and endpoint_passed and heading_passed,
            "clearance_passed": clearance.margin_passed,
            "endpoint_passed": endpoint_passed,
            "heading_passed": heading_passed,
            "maximum_endpoint_error_mm": maximum_endpoint_error_mm,
            "maximum_heading_error_deg": maximum_heading_error_deg,
            "rule": "clearance, endpoint, and heading gates must all pass",
        },
        "safety": (
            "Host-side candidate only. Defaults use the user-confirmed 70 x "
            "39 mm completed-machine envelope centred on the coincident blue "
            "label/machine/turn reference; physical qualification still "
            "requires a calibrated absolute scene and repeated video."
        ),
    }


def _print_clearance(label: str, report: dict[str, Any]) -> None:
    endpoint = report["endpoint"]
    error = report["endpoint_error"]
    clearance = report["clearance"]
    print(
        f"[TURN-CLEARANCE] {label} endpoint "
        f"x={endpoint['x_right_mm']:.3f}mm y={endpoint['y_forward_mm']:.3f}mm "
        f"theta={endpoint['theta_deg']:.3f}deg error={error['distance_mm']:.3f}mm"
    )
    print(
        "[TURN-CLEARANCE] raw={:.3f}mm uncertainty={:.3f}mm "
        "effective={:.3f}mm required={:.3f}mm pass={} collision={}".format(
            clearance["raw_min_clearance_mm"],
            clearance["uncertainty_mm"],
            clearance["effective_min_clearance_mm"],
            clearance["required_margin_mm"],
            int(clearance["margin_passed"]),
            int(clearance["physical_collision"]),
        )
    )
    model_scope = report.get("model", {}).get("scope")
    if model_scope is not None:
        print(
            "[TURN-CLEARANCE] model_scope={} violations={}".format(
                int(model_scope["qualified"]),
                "; ".join(model_scope["violations"]) or "none",
            )
        )
    print(
        "[TURN-CLEARANCE] closest t={:.3f}s phase={} obstacle={}({}) "
        "feature={} pose=({:.3f},{:.3f},{:.3f}deg)".format(
            clearance["worst_time_s"],
            clearance["worst_phase"],
            clearance["obstacle_id"],
            clearance["obstacle_kind"],
            clearance["robot_feature"],
            clearance["worst_x_mm"],
            clearance["worst_y_mm"],
            clearance["worst_theta_deg"],
        )
    )
    if clearance["first_collision_time_s"] is not None:
        print(
            "[TURN-CLEARANCE] first_collision t={:.3f}s theta={:.3f}deg "
            "obstacle={} feature={}".format(
                clearance["first_collision_time_s"],
                clearance["first_collision_theta_deg"],
                clearance["first_collision_obstacle_id"],
                clearance["first_collision_robot_feature"],
            )
        )


def _write_json(path: Optional[Path], report: dict[str, Any], print_json: bool) -> None:
    encoded = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(encoded, encoding="utf-8")
    if print_json:
        print(encoded, end="")


def render_plot(
    path: Path,
    scene: TurnScene,
    footprint: Footprint,
    trajectories: Sequence[tuple[str, Sequence[PoseSample], ClearanceResult, str]],
) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon, Rectangle
    except ModuleNotFoundError as exc:
        raise RuntimeError("matplotlib is required for --plot") from exc

    figure, axis = plt.subplots(figsize=(8.0, 8.0))
    for obstacle in scene.obstacles:
        color = "#ef233c" if obstacle.kind == "post" else "#333333"
        axis.add_patch(
            Rectangle(
                (obstacle.min_x_mm, obstacle.min_y_mm),
                obstacle.max_x_mm - obstacle.min_x_mm,
                obstacle.max_y_mm - obstacle.min_y_mm,
                facecolor=color,
                edgecolor=color,
                alpha=0.85,
                zorder=2,
            )
        )

    for label, samples, clearance, color in trajectories:
        axis.plot(
            [sample.x_mm for sample in samples],
            [sample.y_mm for sample in samples],
            color=color,
            linewidth=2.0,
            label=label,
            zorder=4,
        )
        stride = max(1, len(samples) // 22)
        outline_indexes = set(range(0, len(samples), stride))
        worst_index = min(
            range(len(samples)),
            key=lambda index: abs(samples[index].time_s - clearance.worst_time_s),
        )
        outline_indexes.add(worst_index)
        outline_indexes.add(len(samples) - 1)
        for index in sorted(outline_indexes):
            polygon = footprint.polygon(samples[index])
            worst = index == worst_index
            axis.add_patch(
                Polygon(
                    polygon,
                    closed=True,
                    fill=False,
                    edgecolor="#ff8c00" if worst else color,
                    linewidth=2.2 if worst else 0.7,
                    alpha=1.0 if worst else 0.22,
                    zorder=5,
                )
            )
        axis.plot(
            [clearance.robot_closest_x_mm, clearance.obstacle_closest_x_mm],
            [clearance.robot_closest_y_mm, clearance.obstacle_closest_y_mm],
            color="#ff8c00",
            linewidth=2.0,
            zorder=6,
        )

    axis.scatter(
        [scene.start_x_mm],
        [scene.start_y_mm],
        marker="o",
        color="#2b6cb0",
        label="primitive start",
        zorder=7,
    )
    axis.scatter(
        [scene.target_world_x_mm],
        [scene.target_world_y_mm],
        marker="x",
        s=80,
        color="#2f855a",
        label="canonical endpoint",
        zorder=7,
    )
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, linestyle=":", alpha=0.35)
    axis.set_xlabel("world x / vehicle-right at primitive start [mm]")
    axis.set_ylabel("world y [mm]")
    axis.set_title(scene.name + "\nfull-body swept clearance")
    axis.legend(loc="best")
    figure.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=180)
    plt.close(figure)


def command_evaluate(args: argparse.Namespace) -> int:
    _, turn, sim = resolve_simulation(args)
    scene = _scene_from_args(args)
    footprint = _footprint_from_args(args)
    budget = _budget_from_args(args)
    samples = simulation_world_samples(sim, scene)
    clearance = evaluate_clearance(
        samples,
        scene,
        footprint,
        budget,
        args.required_margin_mm,
        args.max_corner_step_mm,
    )
    report = _sim_report(
        sim,
        scene,
        footprint,
        budget,
        clearance,
        args.maximum_endpoint_error_mm,
        args.maximum_heading_error_deg,
    )
    _apply_model_scope(
        report,
        _combined_model_scope(args, turn, [args.slip_angle_coefficient]),
    )
    if not args.json:
        _print_clearance("simulated", report)
    _write_json(args.report_json, report, args.json)
    if args.plot is not None:
        render_plot(args.plot, scene, footprint, [("simulated", samples, clearance, "#2563eb")])
    return 0 if report["acceptance"]["passed"] else 1


def _solve_offsets_for_core(
    current_turn: Any,
    scene: TurnScene,
    core: Any,
) -> tuple[float, float]:
    """Solve the continuous in/out offsets that close the canonical endpoint."""

    theta = math.radians(core.final_pose.theta_deg)
    out_dx = -math.sin(theta)
    out_dy = math.cos(theta)
    if abs(out_dx) > 1.0e-6:
        dist_out = (scene.target.x_right_mm - core.final_pose.x_mm) / out_dx
        dist_in = (
            scene.target.y_forward_mm - core.final_pose.y_mm - dist_out * out_dy
        )
    else:
        # At 180 degrees the entry and exit straights are collinear and cannot
        # correct lateral closure.  Alpha supplies x closure; project the
        # current offsets onto the remaining y-closure constraint so the
        # proposed pair changes as little as possible.
        wanted = scene.target.y_forward_mm - core.final_pose.y_mm
        current_projection = (
            current_turn.dist_in_mm + out_dy * current_turn.dist_out_mm
        )
        correction = (current_projection - wanted) / (1.0 + out_dy * out_dy)
        dist_in = current_turn.dist_in_mm - correction
        dist_out = current_turn.dist_out_mm - out_dy * correction
    return dist_in, dist_out


def _candidate_for_offsets(
    args: argparse.Namespace,
    current_turn: Any,
    scene: TurnScene,
    footprint: Footprint,
    budget: ClearanceBudget,
    alpha_deg_s2: float,
    dist_in: float,
    dist_out: float,
) -> Optional[tuple[SearchCandidate, Any, list[PoseSample]]]:
    if not (
        args.minimum_offset_mm <= dist_in <= args.maximum_offset_mm
        and args.minimum_offset_mm <= dist_out <= args.maximum_offset_mm
    ):
        return None
    _, _, sim = resolve_simulation(
        args,
        {"alpha": alpha_deg_s2, "dist_in": dist_in, "dist_out": dist_out},
    )
    endpoint_error = math.hypot(
        sim.final_pose.x_mm - scene.target.x_right_mm,
        sim.final_pose.y_mm - scene.target.y_forward_mm,
    )
    heading_error = sim.final_pose.theta_deg - scene.target.theta_deg
    if endpoint_error > args.maximum_endpoint_error_mm:
        return None
    if abs(heading_error) > args.maximum_heading_error_deg:
        return None
    samples = simulation_world_samples(sim, scene)
    clearance = evaluate_clearance(
        samples,
        scene,
        footprint,
        budget,
        args.required_margin_mm,
        args.max_corner_step_mm,
    )
    worst_slip = args.slip_angle_coefficient
    worst_samples = samples
    for alternate_slip in args.robust_slip_angle_coefficient:
        if abs(alternate_slip - args.slip_angle_coefficient) <= EPS:
            continue
        _, _, alternate_sim = resolve_simulation(
            args,
            {"alpha": alpha_deg_s2, "dist_in": dist_in, "dist_out": dist_out},
            alternate_slip,
        )
        alternate_samples = simulation_world_samples(alternate_sim, scene)
        alternate_clearance = evaluate_clearance(
            alternate_samples,
            scene,
            footprint,
            budget,
            args.required_margin_mm,
            args.max_corner_step_mm,
        )
        if (
            alternate_clearance.effective_min_clearance_mm
            < clearance.effective_min_clearance_mm
        ):
            clearance = alternate_clearance
            worst_slip = alternate_slip
            worst_samples = alternate_samples
    model_scope = _combined_model_scope(
        args,
        sim.turn,
        [args.slip_angle_coefficient, *args.robust_slip_angle_coefficient],
    )
    change = (
        abs(alpha_deg_s2 - current_turn.alpha_deg_s2)
        / max(current_turn.alpha_deg_s2, 1.0)
        + abs(dist_in - current_turn.dist_in_mm) / 45.0
        + abs(dist_out - current_turn.dist_out_mm) / 45.0
    )
    return (
        SearchCandidate(
            alpha_deg_s2,
            dist_in,
            dist_out,
            endpoint_error,
            heading_error,
            change,
            sim.duration_ms,
            clearance,
            worst_slip,
            model_scope["qualified"],
            tuple(model_scope["violations"]),
            sim.final_pose.x_mm,
            sim.final_pose.y_mm,
            sim.final_pose.theta_deg,
        ),
        sim,
        worst_samples,
    )


def _candidate_for_alpha(
    args: argparse.Namespace,
    current_turn: Any,
    scene: TurnScene,
    footprint: Footprint,
    budget: ClearanceBudget,
    alpha_deg_s2: float,
) -> Optional[tuple[SearchCandidate, Any, list[PoseSample]]]:
    """Return the legacy nearest-closure candidate for one alpha.

    Search uses :func:`_candidates_for_alpha` so that it also examines all
    practical quantized offsets inside the configured endpoint tolerance.  The
    singular helper remains useful to callers that explicitly want the closest
    canonical closure.
    """

    _, _, core = resolve_simulation(
        args,
        {"alpha": alpha_deg_s2, "dist_in": 0.0, "dist_out": 0.0},
    )
    dist_in, dist_out = _solve_offsets_for_core(current_turn, scene, core)
    if args.offset_quantum_mm > 0.0:
        dist_in = round(dist_in / args.offset_quantum_mm) * args.offset_quantum_mm
        dist_out = round(dist_out / args.offset_quantum_mm) * args.offset_quantum_mm
    return _candidate_for_offsets(
        args,
        current_turn,
        scene,
        footprint,
        budget,
        alpha_deg_s2,
        dist_in,
        dist_out,
    )


def _quantized_offset_indices(
    minimum_mm: float,
    maximum_mm: float,
    quantum_mm: float,
) -> range:
    first = math.ceil((minimum_mm - EPS) / quantum_mm)
    last = math.floor((maximum_mm + EPS) / quantum_mm)
    return range(first, last + 1)


def _offset_pairs_for_alpha(
    args: argparse.Namespace,
    scene: TurnScene,
    core: Any,
) -> list[tuple[float, float]]:
    """Enumerate a bounded superset of endpoint-feasible quantized offsets.

    Straight phases stop on the first 1 ms tick at or beyond their requested
    distance.  The continuous endpoint is therefore padded by two maximum
    straight steps, then every returned pair is checked with the actual
    discrete simulator by ``_candidate_for_offsets``.
    """

    quantum = args.offset_quantum_mm
    if quantum <= 0.0:
        return []

    theta = math.radians(core.final_pose.theta_deg)
    out_dx = -math.sin(theta)
    out_dy = math.cos(theta)
    maximum_speed_mm_s = max(
        abs(core.entry_speed_mm_s),
        abs(core.out_speed_mm_s),
        abs(core.turn.velocity_mm_s),
    )
    turn_tune = _load_turn_tune_module()
    discrete_endpoint_pad_mm = (
        2.0 * maximum_speed_mm_s * float(turn_tune.DT_S)
    )
    continuous_limit_mm = (
        args.maximum_endpoint_error_mm + discrete_endpoint_pad_mm
    )

    if abs(out_dx) > 1.0e-6:
        nominal_out = (
            scene.target.x_right_mm - core.final_pose.x_mm
        ) / out_dx
        out_radius = continuous_limit_mm / abs(out_dx)
        out_min = max(args.minimum_offset_mm, nominal_out - out_radius)
        out_max = min(args.maximum_offset_mm, nominal_out + out_radius)
        out_indices = _quantized_offset_indices(out_min, out_max, quantum)
    else:
        lateral_error = abs(core.final_pose.x_mm - scene.target.x_right_mm)
        if lateral_error > continuous_limit_mm + EPS:
            return []
        out_indices = _quantized_offset_indices(
            args.minimum_offset_mm,
            args.maximum_offset_mm,
            quantum,
        )
        if len(out_indices) > MAX_OFFSET_PAIR_EVALUATIONS_PER_ALPHA:
            raise ValueError(
                "offset grid exceeds per-alpha evaluation limit; increase "
                "--offset-quantum-mm or narrow the offset bounds"
            )

    pairs: list[tuple[float, float]] = []
    for out_index in out_indices:
        dist_out = out_index * quantum
        nominal_in = (
            scene.target.y_forward_mm
            - core.final_pose.y_mm
            - dist_out * out_dy
        )
        in_min = max(
            args.minimum_offset_mm,
            nominal_in - continuous_limit_mm,
        )
        in_max = min(
            args.maximum_offset_mm,
            nominal_in + continuous_limit_mm,
        )
        for in_index in _quantized_offset_indices(in_min, in_max, quantum):
            dist_in = in_index * quantum
            predicted_x = core.final_pose.x_mm + dist_out * out_dx
            predicted_y = (
                core.final_pose.y_mm + dist_in + dist_out * out_dy
            )
            if (
                math.hypot(
                    predicted_x - scene.target.x_right_mm,
                    predicted_y - scene.target.y_forward_mm,
                )
                > continuous_limit_mm + EPS
            ):
                continue
            pairs.append((dist_in, dist_out))
            if len(pairs) > MAX_OFFSET_PAIR_EVALUATIONS_PER_ALPHA:
                raise ValueError(
                    "offset grid exceeds per-alpha evaluation limit; increase "
                    "--offset-quantum-mm or narrow the offset bounds"
                )
    return sorted(set(pairs))


def _candidates_for_alpha(
    args: argparse.Namespace,
    current_turn: Any,
    scene: TurnScene,
    footprint: Footprint,
    budget: ClearanceBudget,
    alpha_deg_s2: float,
) -> list[tuple[SearchCandidate, Any, list[PoseSample]]]:
    """Return every practical offset pair that passes endpoint/heading gates."""

    if args.offset_quantum_mm <= 0.0:
        candidate = _candidate_for_alpha(
            args, current_turn, scene, footprint, budget, alpha_deg_s2
        )
        return [] if candidate is None else [candidate]

    _, _, core = resolve_simulation(
        args,
        {"alpha": alpha_deg_s2, "dist_in": 0.0, "dist_out": 0.0},
    )
    candidates = []
    for dist_in, dist_out in _offset_pairs_for_alpha(args, scene, core):
        candidate = _candidate_for_offsets(
            args,
            current_turn,
            scene,
            footprint,
            budget,
            alpha_deg_s2,
            dist_in,
            dist_out,
        )
        if candidate is not None:
            candidates.append(candidate)
    return candidates


def command_search(args: argparse.Namespace) -> int:
    _, current_turn, current_sim = resolve_simulation(args)
    scene = _scene_from_args(args)
    footprint = _footprint_from_args(args)
    budget = _budget_from_args(args)
    alpha_min = args.alpha_min
    alpha_max = args.alpha_max
    if alpha_min is None:
        alpha_min = max(100.0, current_turn.alpha_deg_s2 * 0.5)
    if alpha_max is None:
        alpha_max = current_turn.alpha_deg_s2 * 1.5
    if alpha_max < alpha_min or args.alpha_step <= 0.0:
        raise ValueError("invalid alpha search range")

    current_samples = simulation_world_samples(current_sim, scene)
    current_clearance = evaluate_clearance(
        current_samples,
        scene,
        footprint,
        budget,
        args.required_margin_mm,
        args.max_corner_step_mm,
    )
    current_report = _sim_report(
        current_sim,
        scene,
        footprint,
        budget,
        current_clearance,
        args.maximum_endpoint_error_mm,
        args.maximum_heading_error_deg,
    )
    current_scope = _combined_model_scope(
        args,
        current_turn,
        [args.slip_angle_coefficient, *args.robust_slip_angle_coefficient],
    )
    _apply_model_scope(current_report, current_scope)

    candidates: list[tuple[SearchCandidate, Any, list[PoseSample]]] = []
    alpha = alpha_min
    while alpha <= alpha_max + 0.5 * args.alpha_step:
        candidates.extend(
            _candidates_for_alpha(
                args, current_turn, scene, footprint, budget, alpha
            )
        )
        alpha += args.alpha_step
    if not candidates:
        raise ValueError("no candidate satisfies endpoint and offset bounds")
    margin_passing = [item for item in candidates if item[0].clearance.margin_passed]
    safe = [item for item in margin_passing if item[0].model_scope_qualified]
    if safe:
        safe.sort(
            key=lambda item: (
                -item[0].clearance.effective_min_clearance_mm,
                item[0].endpoint_error_mm,
                item[0].parameter_change_norm,
            )
        )
        recommended = safe[0]
        recommendation_reason = (
            "largest worst-model clearance among margin-passing candidates"
        )
    else:
        candidates.sort(
            key=lambda item: (
                -item[0].clearance.effective_min_clearance_mm,
                item[0].endpoint_error_mm,
                item[0].parameter_change_norm,
            )
        )
        recommended = candidates[0]
        recommendation_reason = (
            "clearance passes only outside the bound empirical-model scope"
            if margin_passing
            else "largest clearance found; required margin not reached"
        )

    ranked = sorted(
        candidates,
        key=lambda item: (
            not item[0].clearance.margin_passed,
            -item[0].clearance.effective_min_clearance_mm,
            item[0].endpoint_error_mm,
            item[0].parameter_change_norm,
        ),
    )
    report = {
        "schema": "nightfall_turn_clearance_search_v1",
        "source": "firmware-nominal-simulation",
        "scene": scene_to_dict(scene),
        "footprint": asdict(footprint),
        "footprint_basis": footprint_basis(footprint),
        "uncertainty": {
            **budget.components(footprint),
            "total_mm": budget.total_mm(footprint),
        },
        "required_margin_mm": args.required_margin_mm,
        "current": current_report,
        "search": {
            "alpha_min_deg_s2": alpha_min,
            "alpha_max_deg_s2": alpha_max,
            "alpha_step_deg_s2": args.alpha_step,
            "minimum_offset_mm": args.minimum_offset_mm,
            "maximum_offset_mm": args.maximum_offset_mm,
            "offset_quantum_mm": args.offset_quantum_mm,
            "maximum_endpoint_error_mm": args.maximum_endpoint_error_mm,
            "maximum_heading_error_deg": args.maximum_heading_error_deg,
            "endpoint_model_slip_angle_coefficient": args.slip_angle_coefficient,
            "robust_clearance_slip_angle_coefficients": sorted(
                set(
                    [args.slip_angle_coefficient]
                    + args.robust_slip_angle_coefficient
                )
            ),
            "evaluated_candidates": len(candidates),
            "margin_passing_candidates": len(margin_passing),
            "safety_qualified_candidates": len(safe),
        },
        "recommendation_reason": recommendation_reason,
        "safe_recommendation_available": bool(safe),
        "recommended": asdict(recommended[0]) if safe else None,
        "best_diagnostic": None if safe else asdict(recommended[0]),
        "shortlist": [asdict(item[0]) for item in ranked[: args.top]],
        "candidate_only": True,
        "next_steps": [
            "validate the measured footprint against an absolute board-coordinate scene and repeated video",
            "after changing mode2 params, regenerate/check the PC motion table with tools/route_precompute/generate.py and tools/route_precompute/run_tests.sh",
            "do not run the candidate until the generated table and firmware builds pass",
        ],
    }
    if not args.json:
        _print_clearance("current", current_report)
        selected = asdict(recommended[0])
        selected_clearance = selected["clearance"]
        print(
            "[TURN-CLEARANCE] search candidates={} passing={} reason={}".format(
                len(candidates), len(safe), recommendation_reason
            )
        )
        print(
            "[TURN-CLEARANCE] {} alpha={:.1f}deg/s2 in={:.3f}mm "
            "out={:.3f}mm effective={:.3f}mm clearance_pass={} scope_pass={}".format(
                "recommended" if safe else "best_diagnostic_do_not_apply",
                selected["alpha_deg_s2"],
                selected["dist_in_mm"],
                selected["dist_out_mm"],
                selected_clearance["effective_min_clearance_mm"],
                int(selected_clearance["margin_passed"]),
                int(selected["model_scope_qualified"]),
            )
        )
        if safe:
            fields = current_turn.source_fields
            print("[TURN-CLEARANCE] candidate_assignments")
            print(f"  .{fields['alpha']} = {selected['alpha_deg_s2']:.3f}f,")
            print(f"  .{fields['dist_in']} = {selected['dist_in_mm']:.3f}f,")
            print(f"  .{fields['dist_out']} = {selected['dist_out_mm']:.3f}f,")
        else:
            print("[TURN-CLEARANCE] safe_candidate=0 do_not_apply=1")
        print("[TURN-CLEARANCE] candidate_only=1 source_files_edited=0")
        print(
            "[TURN-CLEARANCE] after_param_change=run "
            "tools/route_precompute/generate.py && "
            "tools/route_precompute/run_tests.sh"
        )
    _write_json(args.report_json, report, args.json)
    if args.plot is not None:
        render_plot(
            args.plot,
            scene,
            footprint,
            [
                ("current", current_samples, current_clearance, "#dc2626"),
                (
                    "recommended" if safe else "best diagnostic (fails)",
                    recommended[2],
                    recommended[0].clearance,
                    "#16a34a",
                ),
            ],
        )
    return 0 if safe else 1


def _unwrap_degrees(values: Sequence[float]) -> list[float]:
    if not values:
        return []
    result = [values[0]]
    for value in values[1:]:
        candidate = value
        while candidate - result[-1] > 180.0:
            candidate -= 360.0
        while candidate - result[-1] < -180.0:
            candidate += 360.0
        result.append(candidate)
    return result


def _moving_average(values: Sequence[float], radius: int = 2) -> list[float]:
    result = []
    for index in range(len(values)):
        lo = max(0, index - radius)
        hi = min(len(values), index + radius + 1)
        result.append(sum(values[lo:hi]) / (hi - lo))
    return result


def _gradient(values: Sequence[float], times: Sequence[float]) -> list[float]:
    result = [0.0] * len(values)
    for index in range(1, len(values) - 1):
        dt = times[index + 1] - times[index - 1]
        if dt > 0.0:
            result[index] = (values[index + 1] - values[index - 1]) / dt
    if len(values) > 1:
        result[0] = result[1]
        result[-1] = result[-2]
    return result


def _median(values: Sequence[float]) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("cannot take median of an empty sequence")
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def _angle_error_deg(actual_deg: float, expected_deg: float) -> float:
    return (actual_deg - expected_deg + 180.0) % 360.0 - 180.0


def _interpolate_value(
    times: Sequence[float], values: Sequence[float], query_time_s: float
) -> float:
    if query_time_s <= times[0]:
        return values[0]
    if query_time_s >= times[-1]:
        return values[-1]
    upper = bisect.bisect_right(times, query_time_s)
    lower = upper - 1
    span = times[upper] - times[lower]
    if span <= 0.0:
        return values[lower]
    fraction = (query_time_s - times[lower]) / span
    return values[lower] + fraction * (values[upper] - values[lower])


def _reference_core_progress(
    reference_sim: Any, sign: float
) -> tuple[list[float], list[float]]:
    core = [sample for sample in reference_sim.samples if sample.phase == "turn_core"]
    if len(core) < 3:
        raise ValueError("reference simulation has no usable turn core")
    previous = [
        sample.t_ms
        for sample in reference_sim.samples
        if sample.t_ms < core[0].t_ms
    ]
    origin_ms = previous[-1] if previous else core[0].t_ms - 1
    times = [0.0]
    progress = [0.0]
    for sample in core:
        times.append((sample.t_ms - origin_ms) / 1000.0)
        progress.append(max(0.0, sign * sample.theta_deg))
    return times, progress


def _align_reference_core_start(
    times: Sequence[float],
    measured_omega_deg_s: Sequence[float],
    sign: float,
    active_start: int,
    active_end: int,
    omega_threshold_deg_s: float,
    reference_sim: Any,
) -> tuple[float, float, float, float]:
    """Align nominal angular-profile time without discarding its slow onset."""
    model_times, model_progress = _reference_core_progress(reference_sim, sign)
    spacings = [
        times[index + 1] - times[index]
        for index in range(len(times) - 1)
        if times[index + 1] > times[index]
    ]
    median_spacing_s = _median(spacings)

    model_omega = _gradient(model_progress, model_times)
    threshold_index = next(
        (
            index
            for index, value in enumerate(model_omega)
            if value >= omega_threshold_deg_s
        ),
        None,
    )
    if threshold_index is None:
        raise ValueError(
            "reference turn never reaches the requested angular-rate threshold"
        )
    initial = times[active_start] - model_times[threshold_index]
    search_span_s = max(0.040, 4.0 * median_spacing_s)
    search_step_s = min(0.0005, max(0.0001, median_spacing_s / 12.0))
    candidate_count = int(math.ceil(2.0 * search_span_s / search_step_s)) + 1
    observed = [max(0.0, sign * value) for value in measured_omega_deg_s]
    fit_lo = max(0, active_start - max(4, int(0.040 / median_spacing_s)))
    fit_hi = min(
        len(times),
        active_end + max(5, int(0.060 / median_spacing_s)) + 1,
    )
    best: Optional[tuple[float, float, float, float]] = None
    for candidate_index in range(candidate_count):
        candidate = initial - search_span_s + candidate_index * search_step_s
        predicted_progress = [
            _interpolate_value(
                model_times,
                model_progress,
                max(0.0, time_s - candidate),
            )
            for time_s in times
        ]
        predicted_omega = _gradient(
            _moving_average(predicted_progress), times
        )
        pairs = [
            (observed[index], max(0.0, predicted_omega[index]))
            for index in range(fit_lo, fit_hi)
        ]
        denominator = sum(predicted * predicted for _, predicted in pairs)
        scale = (
            sum(actual * predicted for actual, predicted in pairs) / denominator
            if denominator > EPS
            else 1.0
        )
        scale = min(1.5, max(0.5, scale))
        squared_error = sum(
            (actual - scale * predicted) ** 2
            for actual, predicted in pairs
        ) / max(1, len(pairs))
        predicted_active = next(
            (
                index
                for index in range(fit_lo, fit_hi)
                if predicted_omega[index] >= omega_threshold_deg_s
            ),
            fit_hi,
        )
        onset_error = abs(predicted_active - active_start)
        score = squared_error + onset_error * omega_threshold_deg_s**2
        item = (score, candidate, math.sqrt(squared_error), scale)
        if best is None or item[0] < best[0]:
            best = item
    assert best is not None
    return best[1], best[2], best[3], median_spacing_s


def extract_video_turn(
    path: Path,
    scene: TurnScene,
    turn: Any,
    reference_sim: Any,
    turn_index: int = 1,
    omega_threshold_deg_s: float = 80.0,
    maximum_angle_shortfall_deg: float = 3.0,
    minimum_preroll_s: float = 0.040,
    minimum_postroll_s: float = 0.020,
    maximum_pose_gap_s: float = 0.050,
    registration_mode: str = "normalized",
    absolute_yaw_offset_deg: float = -90.0,
) -> tuple[list[PoseSample], dict[str, Any]]:
    with path.open("r", encoding="utf-8", errors="ignore") as stream:
        rows = [row for row in csv.DictReader(line for line in stream if not line.lstrip().startswith("#"))]
    if len(rows) < 10:
        raise ValueError(f"{path}: fewer than ten trajectory rows")
    required = ("x_mm", "y_mm")
    if any(name not in rows[0] for name in required):
        raise ValueError(f"{path}: x_mm/y_mm columns are required")
    yaw_name = next(
        (name for name in ("yaw_deg_unwrapped", "yaw_deg", "heading_deg_unwrapped") if name in rows[0]),
        None,
    )
    if yaw_name is None:
        raise ValueError(f"{path}: a measured yaw column is required")
    time_name = "video_pts_s" if "video_pts_s" in rows[0] else "time_s"
    valid_rows = []
    for row in rows:
        try:
            values = (
                float(row[time_name]),
                float(row["x_mm"]),
                float(row["y_mm"]),
                float(row[yaw_name]),
            )
        except (KeyError, ValueError):
            continue
        pose_valid = float(row.get("pose_valid", row.get("tracking_valid", "1")) or "0")
        heading_valid = float(row.get("heading_valid", "1") or "0")
        if all(math.isfinite(value) for value in values) and pose_valid > 0.5 and heading_valid > 0.5:
            valid_rows.append(values)
    if len(valid_rows) < 10:
        raise ValueError(f"{path}: fewer than ten valid measured poses")

    times = [row[0] for row in valid_rows]
    x_values = [row[1] for row in valid_rows]
    y_values = [row[2] for row in valid_rows]
    yaw = _moving_average(_unwrap_degrees([row[3] for row in valid_rows]))
    if any(times[index + 1] <= times[index] for index in range(len(times) - 1)):
        raise ValueError(f"{path}: valid pose timestamps are not strictly increasing")
    omega = _gradient(yaw, times)
    sign = -1.0 if turn.signed_angle_deg < 0.0 else 1.0
    active = [sign * value >= omega_threshold_deg_s for value in omega]
    regions: list[tuple[int, int]] = []
    index = 0
    maximum_gap_s = 0.060
    while index < len(active):
        if not active[index]:
            index += 1
            continue
        start = index
        end = index
        last_active = index
        index += 1
        while index < len(active):
            if active[index]:
                end = index
                last_active = index
            elif times[index] - times[last_active] > maximum_gap_s:
                break
            index += 1
        yaw_change = sign * (yaw[end] - yaw[start])
        if yaw_change >= 0.60 * abs(turn.signed_angle_deg):
            regions.append((start, end))
    if turn_index <= 0 or turn_index > len(regions):
        raise ValueError(
            f"{path}: found {len(regions)} matching turn(s), cannot select {turn_index}"
        )
    active_start, active_end = regions[turn_index - 1]
    core_start_time, alignment_rmse, alignment_scale, median_spacing_s = (
        _align_reference_core_start(
            times,
            omega,
            sign,
            active_start,
            active_end,
            omega_threshold_deg_s,
            reference_sim,
        )
    )
    if core_start_time - times[0] < minimum_preroll_s:
        raise ValueError(
            f"{path}: only {core_start_time - times[0]:.3f}s of valid pre-roll; "
            f"need {minimum_preroll_s:.3f}s"
        )
    baseline_end = core_start_time - max(0.012, 1.5 * median_spacing_s)
    baseline_start = baseline_end - max(0.050, 6.0 * median_spacing_s)
    baseline_values = [
        value
        for time_s, value in zip(times, yaw)
        if baseline_start <= time_s <= baseline_end
    ]
    if len(baseline_values) < 3:
        raise ValueError(f"{path}: too few valid pre-turn heading samples")
    start_yaw = _median(baseline_values)
    progress = [sign * (value - start_yaw) for value in yaw]
    target_angle = abs(turn.signed_angle_deg)
    core_end = active_end
    completion_omega = max(20.0, 0.25 * omega_threshold_deg_s)
    while core_end + 1 < len(progress) and not (
        progress[core_end] >= target_angle - 1.0
        and sign * omega[core_end] <= completion_omega
    ):
        core_end += 1
    if progress[core_end] < target_angle - maximum_angle_shortfall_deg:
        raise ValueError(
            f"{path}: turn is incomplete ({progress[core_end]:.2f}/{target_angle:.2f}deg)"
        )

    heading = math.radians(start_yaw)
    forward_board = (math.cos(heading), math.sin(heading))
    core_x = _interpolate_value(times, x_values, core_start_time)
    core_y = _interpolate_value(times, y_values, core_start_time)
    origin_x = core_x - turn.dist_in_mm * forward_board[0]
    origin_y = core_y - turn.dist_in_mm * forward_board[1]

    finish = core_end
    traveled_after = 0.0
    while finish + 1 < len(valid_rows) and traveled_after < turn.dist_out_mm:
        traveled_after += math.hypot(
            x_values[finish + 1] - x_values[finish],
            y_values[finish + 1] - y_values[finish],
        )
        finish += 1
    if traveled_after + 0.5 < turn.dist_out_mm:
        raise ValueError(
            f"{path}: only {traveled_after:.2f}/{turn.dist_out_mm:.2f}mm of "
            "valid out-offset trajectory"
        )
    if times[-1] - times[finish] < minimum_postroll_s:
        raise ValueError(
            f"{path}: only {times[-1] - times[finish]:.3f}s of valid post-roll; "
            f"need {minimum_postroll_s:.3f}s"
        )
    selected_start = max(0, bisect.bisect_left(times, core_start_time) - 2)
    selected_end = min(len(times) - 1, finish + 2)
    maximum_gap = max(
        times[index + 1] - times[index]
        for index in range(selected_start, selected_end)
    )
    if maximum_gap > maximum_pose_gap_s:
        raise ValueError(
            f"{path}: {maximum_gap:.3f}s valid-pose gap exceeds "
            f"{maximum_pose_gap_s:.3f}s"
        )

    entry_duration_s = turn.dist_in_mm / max(turn.velocity_mm_s, EPS)
    if registration_mode == "normalized":
        samples = [
            PoseSample(
                0.0,
                "video-primitive-start",
                scene.start_x_mm,
                scene.start_y_mm,
                scene.start_heading_deg,
            )
        ]
        core_world_x, core_world_y = _transform_local(
            scene.start_x_mm,
            scene.start_y_mm,
            scene.start_heading_deg,
            0.0,
            turn.dist_in_mm,
        )
        samples.append(
            PoseSample(
                entry_duration_s,
                "video-turn-core-start",
                core_world_x,
                core_world_y,
                scene.start_heading_deg,
            )
        )
        first_measured = bisect.bisect_right(times, core_start_time)
        for index in range(first_measured, finish + 1):
            dx = x_values[index] - origin_x
            dy = y_values[index] - origin_y
            local_x = math.sin(heading) * dx - math.cos(heading) * dy
            local_y = math.cos(heading) * dx + math.sin(heading) * dy
            world_x, world_y = _transform_local(
                scene.start_x_mm,
                scene.start_y_mm,
                scene.start_heading_deg,
                local_x,
                local_y,
            )
            samples.append(
                PoseSample(
                    entry_duration_s + times[index] - core_start_time,
                    "video-turn-core" if index <= core_end else "video-out-offset",
                    world_x,
                    world_y,
                    scene.start_heading_deg + sign * progress[index],
                )
            )
    elif registration_mode == "absolute":
        trajectory_start_time = max(times[0], core_start_time - entry_duration_s)
        samples = [
            PoseSample(
                0.0,
                "video-in-offset",
                _interpolate_value(times, x_values, trajectory_start_time),
                _interpolate_value(times, y_values, trajectory_start_time),
                _interpolate_value(times, yaw, trajectory_start_time)
                + absolute_yaw_offset_deg,
            )
        ]
        first_measured = bisect.bisect_right(times, trajectory_start_time)
        for index in range(first_measured, finish + 1):
            phase = "video-in-offset"
            if times[index] >= core_start_time:
                phase = (
                    "video-turn-core"
                    if index <= core_end
                    else "video-out-offset"
                )
            samples.append(
                PoseSample(
                    times[index] - trajectory_start_time,
                    phase,
                    x_values[index],
                    y_values[index],
                    yaw[index] + absolute_yaw_offset_deg,
                )
            )
    else:
        raise ValueError(f"unknown video registration mode: {registration_mode}")
    metadata = {
        "path": str(path.resolve()),
        "turn_index": turn_index,
        "valid_rows": len(valid_rows),
        "active_region_count": len(regions),
        "core_start_time_s": core_start_time,
        "core_end_time_s": times[core_end],
        "finish_time_s": times[finish],
        "start_board_yaw_deg": start_yaw,
        "registration_mode": registration_mode,
        "absolute_yaw_offset_deg": (
            absolute_yaw_offset_deg if registration_mode == "absolute" else None
        ),
        "estimated_primitive_origin_board_x_mm": (
            origin_x if registration_mode == "normalized" else None
        ),
        "estimated_primitive_origin_board_y_mm": (
            origin_y if registration_mode == "normalized" else None
        ),
        "angle_progress_deg": progress[core_end],
        "reference_alignment_omega_rmse_deg_s": alignment_rmse,
        "reference_alignment_omega_scale": alignment_scale,
        "median_valid_pose_spacing_s": median_spacing_s,
        "maximum_selected_pose_gap_s": maximum_gap,
        "preroll_s": core_start_time - times[0],
        "postroll_s": times[-1] - times[finish],
        "method": (
            "the nominal angular-rate profile aligns the slow turn onset; "
            + (
                "measured positions/headings are normalized to the configured "
                "primitive origin"
                if registration_mode == "normalized"
                else "measured positions/headings remain in the scene's absolute "
                "board coordinate frame"
            )
        ),
    }
    return samples, metadata


def command_video(args: argparse.Namespace) -> int:
    _, turn, sim = resolve_simulation(args)
    model_scope = _combined_model_scope(
        args, turn, [args.slip_angle_coefficient]
    )
    scene = _scene_from_args(args)
    footprint = _footprint_from_args(args)
    budget = _budget_from_args(args)
    trials: list[dict[str, Any]] = []
    measured_paths: list[tuple[str, list[PoseSample], ClearanceResult, str]] = []
    heading = math.radians(scene.start_heading_deg)
    if not args.json:
        print(
            "[TURN-CLEARANCE] registration={} absolute_clearance_gate={}".format(
                args.registration_mode,
                int(args.registration_mode == "absolute"),
            )
        )
        print(
            "[TURN-CLEARANCE] extraction_model_scope={} violations={}".format(
                int(model_scope["qualified"]),
                "; ".join(model_scope["violations"]) or "none",
            )
        )
    for trial_index, trajectory_csv in enumerate(args.trajectory_csv, start=1):
        samples, extraction = extract_video_turn(
            trajectory_csv,
            scene,
            turn,
            sim,
            args.turn_index,
            args.omega_threshold_deg_s,
            args.maximum_angle_shortfall_deg,
            args.minimum_preroll_s,
            args.minimum_postroll_s,
            args.maximum_pose_gap_s,
            args.registration_mode,
            args.absolute_yaw_offset_deg,
        )
        clearance = evaluate_clearance(
            samples,
            scene,
            footprint,
            budget,
            args.required_margin_mm,
            args.max_corner_step_mm,
        )
        dx = samples[-1].x_mm - scene.start_x_mm
        dy = samples[-1].y_mm - scene.start_y_mm
        endpoint_local_x = math.cos(heading) * dx + math.sin(heading) * dy
        endpoint_local_y = -math.sin(heading) * dx + math.cos(heading) * dy
        endpoint_distance_error = math.hypot(
            endpoint_local_x - scene.target.x_right_mm,
            endpoint_local_y - scene.target.y_forward_mm,
        )
        heading_error = _angle_error_deg(
            samples[-1].theta_deg,
            scene.target_world_heading_deg,
        )
        endpoint_passed = (
            endpoint_distance_error <= args.maximum_endpoint_error_mm
        )
        heading_passed = abs(heading_error) <= args.maximum_heading_error_deg
        trial_report = {
            "source": "video-measured-trajectory",
            "turn": asdict(turn),
            "extraction": extraction,
            "endpoint": {
                "x_right_mm": endpoint_local_x,
                "y_forward_mm": endpoint_local_y,
                "theta_deg": _angle_error_deg(
                    samples[-1].theta_deg, scene.start_heading_deg
                ),
            },
            "endpoint_error": {
                "distance_mm": endpoint_distance_error,
                "heading_deg": heading_error,
            },
            "clearance": asdict(clearance),
            "acceptance": {
                "passed": (
                    clearance.margin_passed
                    and endpoint_passed
                    and heading_passed
                    and model_scope["qualified"]
                ),
                "clearance_passed": clearance.margin_passed,
                "endpoint_passed": endpoint_passed,
                "heading_passed": heading_passed,
                "model_scope_qualified": model_scope["qualified"],
                "maximum_endpoint_error_mm": args.maximum_endpoint_error_mm,
                "maximum_heading_error_deg": args.maximum_heading_error_deg,
                "gate_kind": (
                    "absolute-clearance"
                    if args.registration_mode == "absolute"
                    else "normalized-shape"
                ),
                "absolute_clearance_qualified": (
                    args.registration_mode == "absolute"
                    and clearance.margin_passed
                    and endpoint_passed
                    and heading_passed
                    and model_scope["qualified"]
                ),
                "rule": (
                    "clearance, endpoint, heading, and extraction-model scope "
                    "gates must all pass"
                ),
            },
        }
        trials.append(trial_report)
        measured_paths.append(
            (f"video trial {trial_index}", samples, clearance, "#dc2626")
        )
        if not args.json:
            _print_clearance(f"video[{trial_index}]", trial_report)

    worst_index = min(
        range(len(trials)),
        key=lambda index: trials[index]["clearance"]["effective_min_clearance_mm"],
    )
    all_passed = all(trial["acceptance"]["passed"] for trial in trials)
    report = {
        "schema": "nightfall_turn_clearance_video_batch_v1",
        "source": "video-measured-trajectories",
        "registration_mode": args.registration_mode,
        "turn": asdict(turn),
        "extraction_model_scope": model_scope,
        "scene": scene_to_dict(scene),
        "footprint": asdict(footprint),
        "footprint_basis": footprint_basis(footprint),
        "uncertainty": {
            **budget.components(footprint),
            "total_mm": budget.total_mm(footprint),
        },
        "trials": trials,
        "aggregate": {
            "count": len(trials),
            "passing_count": sum(
                int(trial["acceptance"]["passed"]) for trial in trials
            ),
            "worst_trial_index": worst_index + 1,
            "worst_effective_min_clearance_mm": trials[worst_index]["clearance"][
                "effective_min_clearance_mm"
            ],
        },
        "acceptance": {
            "passed": all_passed,
            "gate_kind": (
                "absolute-clearance"
                if args.registration_mode == "absolute"
                else "normalized-shape"
            ),
            "absolute_clearance_qualified": (
                args.registration_mode == "absolute" and all_passed
            ),
            "maximum_endpoint_error_mm": args.maximum_endpoint_error_mm,
            "maximum_heading_error_deg": args.maximum_heading_error_deg,
            "rule": (
                "every repeated trial must pass clearance, endpoint, heading, "
                "trajectory-completeness, and extraction-model scope gates"
            ),
        },
        "safety": (
            (
                "Absolute scene registration preserves trial placement, but "
                "clearance still depends on camera/label-height calibration; "
                "the defaults already use the user-confirmed measured body "
                "extents."
                if args.registration_mode == "absolute"
                else "Normalized registration evaluates turn shape/repeatability "
                "but removes absolute trial placement; it is not an absolute "
                "body-to-maze clearance gate."
            )
        ),
    }
    if not args.json:
        print(
            "[TURN-CLEARANCE] video_batch trials={} passing={} worst={} "
            "effective={:.3f}mm pass={}".format(
                len(trials),
                report["aggregate"]["passing_count"],
                worst_index + 1,
                report["aggregate"]["worst_effective_min_clearance_mm"],
                int(all_passed),
            )
        )
    _write_json(args.report_json, report, args.json)
    if args.plot is not None:
        sim_samples = simulation_world_samples(sim, scene)
        sim_clearance = evaluate_clearance(
            sim_samples,
            scene,
            footprint,
            budget,
            args.required_margin_mm,
            args.max_corner_step_mm,
        )
        render_plot(
            args.plot,
            scene,
            footprint,
            [
                ("nominal simulation", sim_samples, sim_clearance, "#2563eb"),
                *measured_paths,
            ],
        )
    return 0 if all_passed else 1


def _add_turn_selection(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--mode", type=int, default=2)
    parser.add_argument("--code", type=int, default=901)
    parser.add_argument("--shortest-params", type=Path, default=None)
    parser.add_argument("--angle-policy", choices=("runtime", "configured"), default="runtime")
    parser.add_argument("--velocity", type=float, default=None)
    parser.add_argument("--alpha", type=float, default=None)
    parser.add_argument("--angle", type=float, default=None)
    parser.add_argument("--dist-in", type=float, default=None)
    parser.add_argument("--dist-out", type=float, default=None)
    parser.add_argument("--entry-speed", type=float, default=None)
    parser.add_argument("--out-speed", type=float, default=None)
    parser.add_argument(
        "--slip-angle-coefficient",
        type=float,
        default=0.0,
        help="empirical understeer K [s^2/m]; zero is the nominal body-heading model",
    )
    parser.add_argument(
        "--slip-model-json",
        type=Path,
        default=None,
        help=(
            "calibration artifact that binds a non-zero K to its validated "
            "mode/code/side/speed/alpha/entry regime"
        ),
    )


def _add_clearance_args(
    parser: argparse.ArgumentParser,
    *,
    endpoint_error_default_mm: float,
    heading_error_default_deg: float,
) -> None:
    parser.add_argument("--scene-json", type=Path, default=None)
    parser.add_argument("--cell-pitch-mm", type=float, default=DEFAULT_CELL_PITCH_MM)
    parser.add_argument("--post-size-mm", type=float, default=DEFAULT_POST_SIZE_MM)
    parser.add_argument("--wall-thickness-mm", type=float, default=DEFAULT_WALL_THICKNESS_MM)
    parser.add_argument(
        "--robot-front-mm",
        type=float,
        default=DEFAULT_ROBOT_FRONT_MM,
        help="front extent from turn centre (measured mini_r2_0 default: 35 mm)",
    )
    parser.add_argument(
        "--robot-rear-mm",
        type=float,
        default=DEFAULT_ROBOT_REAR_MM,
        help="rear extent from turn centre (measured mini_r2_0 default: 35 mm)",
    )
    parser.add_argument(
        "--robot-left-mm",
        type=float,
        default=DEFAULT_ROBOT_LEFT_MM,
        help="left extent from turn centre (measured mini_r2_0 default: 19.5 mm)",
    )
    parser.add_argument(
        "--robot-right-mm",
        type=float,
        default=DEFAULT_ROBOT_RIGHT_MM,
        help="right extent from turn centre (measured mini_r2_0 default: 19.5 mm)",
    )
    parser.add_argument("--required-margin-mm", type=float, default=3.0)
    parser.add_argument(
        "--maximum-endpoint-error-mm",
        type=float,
        default=endpoint_error_default_mm,
    )
    parser.add_argument(
        "--maximum-heading-error-deg",
        type=float,
        default=heading_error_default_deg,
    )
    parser.add_argument("--mechanical-uncertainty-mm", type=float, default=0.5)
    parser.add_argument(
        "--position-uncertainty-mm",
        type=float,
        default=3.0,
        help="simulator model residual or video pose uncertainty",
    )
    parser.add_argument("--heading-uncertainty-deg", type=float, default=0.5)
    parser.add_argument("--max-corner-step-mm", type=float, default=0.5)
    parser.add_argument("--plot", type=Path, default=None)
    parser.add_argument("--report-json", type=Path, default=None)
    parser.add_argument("--json", action="store_true")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Design and validate turn parameters using full-body wall/post clearance"
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    evaluate = subparsers.add_parser("evaluate", help="evaluate the current or overridden turn")
    _add_turn_selection(evaluate)
    _add_clearance_args(
        evaluate, endpoint_error_default_mm=3.0, heading_error_default_deg=1.0
    )
    evaluate.set_defaults(func=command_evaluate)

    search = subparsers.add_parser(
        "search", help="search alpha and solve in/out offsets for canonical closure"
    )
    _add_turn_selection(search)
    _add_clearance_args(
        search, endpoint_error_default_mm=1.0, heading_error_default_deg=0.25
    )
    search.add_argument("--alpha-min", type=float, default=None)
    search.add_argument("--alpha-max", type=float, default=None)
    search.add_argument("--alpha-step", type=float, default=250.0)
    search.add_argument("--minimum-offset-mm", type=float, default=0.0)
    search.add_argument("--maximum-offset-mm", type=float, default=60.0)
    search.add_argument(
        "--offset-quantum-mm",
        type=float,
        default=0.5,
        help="round proposed in/out offsets to a practical setting increment",
    )
    search.add_argument("--top", type=int, default=8)
    search.add_argument(
        "--robust-slip-angle-coefficient",
        type=float,
        action="append",
        default=[],
        help=(
            "also evaluate candidate clearance with this K [s^2/m]; repeat "
            "the option to require the worst model to pass"
        ),
    )
    search.set_defaults(func=command_search)

    video = subparsers.add_parser(
        "video", help="apply the same footprint clearance gate to a trajectory CSV"
    )
    video.add_argument("trajectory_csv", type=Path, nargs="+")
    _add_turn_selection(video)
    _add_clearance_args(
        video, endpoint_error_default_mm=5.0, heading_error_default_deg=2.0
    )
    video.add_argument("--turn-index", type=int, default=1)
    video.add_argument("--omega-threshold-deg-s", type=float, default=80.0)
    video.add_argument("--maximum-angle-shortfall-deg", type=float, default=3.0)
    video.add_argument("--minimum-preroll-s", type=float, default=0.040)
    video.add_argument("--minimum-postroll-s", type=float, default=0.020)
    video.add_argument("--maximum-pose-gap-s", type=float, default=0.050)
    video.add_argument(
        "--registration-mode",
        choices=("normalized", "absolute"),
        default="normalized",
        help=(
            "normalized removes trial placement for shape tuning; absolute "
            "keeps board coordinates and requires --scene-json in that frame"
        ),
    )
    video.add_argument(
        "--absolute-yaw-offset-deg",
        type=float,
        default=-90.0,
        help=(
            "add to vision CSV yaw in absolute mode; -90 converts yaw=0 along "
            "+board-x to the scene convention theta=0 along +world-y"
        ),
    )
    video.set_defaults(func=command_video)
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    positive = (
        "cell_pitch_mm",
        "post_size_mm",
        "wall_thickness_mm",
        "max_corner_step_mm",
        "maximum_endpoint_error_mm",
        "maximum_heading_error_deg",
    )
    for name in positive:
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"--{name.replace('_', '-')} must be positive")
    if args.post_size_mm >= args.cell_pitch_mm:
        raise ValueError("post size must be smaller than cell pitch")
    if args.wall_thickness_mm >= args.cell_pitch_mm:
        raise ValueError("wall thickness must be smaller than cell pitch")
    nonnegative = (
        "required_margin_mm",
        "mechanical_uncertainty_mm",
        "position_uncertainty_mm",
        "heading_uncertainty_deg",
        "slip_angle_coefficient",
        "maximum_angle_shortfall_deg",
        "minimum_preroll_s",
        "minimum_postroll_s",
    )
    for name in nonnegative:
        if hasattr(args, name):
            value = getattr(args, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"--{name.replace('_', '-')} must be non-negative")
    for name in ("omega_threshold_deg_s", "maximum_pose_gap_s"):
        if hasattr(args, name):
            value = getattr(args, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"--{name.replace('_', '-')} must be positive")
    if hasattr(args, "absolute_yaw_offset_deg") and not math.isfinite(
        args.absolute_yaw_offset_deg
    ):
        raise ValueError("--absolute-yaw-offset-deg must be finite")
    if getattr(args, "top", 1) <= 0:
        raise ValueError("--top must be positive")
    if getattr(args, "offset_quantum_mm", 0.0) < 0.0:
        raise ValueError("--offset-quantum-mm must be non-negative")
    for value in getattr(args, "robust_slip_angle_coefficient", []):
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(
                "--robust-slip-angle-coefficient must be finite and non-negative"
            )
    if getattr(args, "turn_index", 1) <= 0:
        raise ValueError("--turn-index must be positive")
    if (
        getattr(args, "registration_mode", "normalized") == "absolute"
        and getattr(args, "scene_json", None) is None
    ):
        raise ValueError("--registration-mode absolute requires --scene-json")


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    try:
        _validate_args(args)
        return args.func(args)
    except (FileNotFoundError, KeyError, ValueError, RuntimeError) as exc:
        print(f"[TURN-CLEARANCE][ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
