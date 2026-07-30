#!/usr/bin/env python3
"""Measure video turn trajectories and propose bounded model corrections.

This tool deliberately never edits firmware parameters.  It converts one or
more board-coordinate trajectories into the local coordinate system used by
``turn_tune.py``:

* x: vehicle right [mm]
* y: vehicle forward [mm]
* theta: left turn positive [deg]

With ``--propose-fit`` it applies a conservative first-order correction:

    next_sim_target = current_sim + gain * (current_sim - observed)

and asks the existing kinematic fitter for a candidate parameter set.  The
candidate is an experiment for the next secured trial, not an automatic
firmware write.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np


@dataclass
class Pose:
    x_right_mm: float
    y_forward_mm: float
    theta_deg: float


@dataclass
class Trial:
    path: str
    sample_count: int
    source_start_s: float
    source_end_s: float
    motion_start_s: float
    motion_end_s: float
    duration_s: float
    start_board_x_mm: float
    start_board_y_mm: float
    start_board_yaw_deg: float
    end_board_x_mm: float
    end_board_y_mm: float
    end_board_yaw_deg: float
    endpoint: Pose
    path_length_mm: float
    peak_speed_mm_s: float
    yaw_change_abs_deg: float
    tracking_valid_fraction: float
    heading_valid_fraction: float
    start_pose_speed_median_mm_s: float
    end_pose_speed_median_mm_s: float
    start_pose_window_s: float
    end_pose_window_s: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Measure markerless video trajectories in the Nightfall turn "
            "coordinate frame and optionally propose the next bounded fit."
        )
    )
    parser.add_argument(
        "trajectory_csv",
        type=Path,
        nargs="+",
        help="one or more trajectory CSV files from repeated identical trials",
    )
    parser.add_argument(
        "--cell-size-mm",
        type=float,
        default=None,
        help="convert x_cell/y_cell when metric columns are absent",
    )
    parser.add_argument(
        "--start-s",
        type=float,
        default=None,
        help="explicit analysis start in video PTS seconds",
    )
    parser.add_argument(
        "--end-s",
        type=float,
        default=None,
        help="explicit analysis end in video PTS seconds",
    )
    parser.add_argument(
        "--motion-threshold-mm-s",
        type=float,
        default=20.0,
        help="speed threshold used for automatic trial boundaries",
    )
    parser.add_argument(
        "--pose-window-ms",
        type=float,
        default=150.0,
        help="stationary window used for start/end pose medians",
    )
    parser.add_argument(
        "--maximum-pose-window-speed-mm-s",
        type=float,
        default=10.0,
        help="maximum median speed in each start/end pose window",
    )
    parser.add_argument(
        "--minimum-pose-window-coverage",
        type=float,
        default=0.80,
        help="minimum fraction of --pose-window-ms present on each side",
    )
    parser.add_argument(
        "--anchor-right-mm",
        type=float,
        default=0.0,
        help="tracked point offset from control reference, vehicle-right axis",
    )
    parser.add_argument(
        "--anchor-forward-mm",
        type=float,
        default=0.0,
        help="tracked point offset from control reference, vehicle-forward axis",
    )
    parser.add_argument(
        "--minimum-valid-fraction",
        type=float,
        default=0.99,
        help="reject a trial below this finite-pose fraction",
    )
    parser.add_argument(
        "--minimum-heading-valid-fraction",
        type=float,
        default=0.99,
        help="reject a trial below this measured-heading fraction",
    )
    parser.add_argument(
        "--maximum-endpoint-std-mm",
        type=float,
        default=2.0,
        help="fit gate for repeated-trial endpoint standard deviation",
    )
    parser.add_argument(
        "--minimum-fit-trials",
        type=int,
        default=5,
        help="minimum identical repeats required before proposing parameters",
    )
    parser.add_argument(
        "--maximum-yaw-std-deg",
        type=float,
        default=1.0,
        help="fit gate for repeated-trial yaw standard deviation",
    )
    parser.add_argument(
        "--report-json",
        type=Path,
        default=None,
        help="write the complete machine-readable report",
    )
    parser.add_argument(
        "--propose-fit",
        action="store_true",
        help="run turn_tune.py's fitter against a conservative correction target",
    )
    parser.add_argument(
        "--feedback-gain",
        type=float,
        default=0.5,
        help="fraction of measured error requested from the model fit",
    )
    parser.add_argument(
        "--maximum-turn-yaw-error-deg",
        type=float,
        default=30.0,
        help="reject a fit if measured turn yaw differs this much",
    )
    parser.add_argument(
        "--maximum-angle-step-deg",
        type=float,
        default=5.0,
        help="maximum proposed one-iteration angle parameter change",
    )
    parser.add_argument(
        "--maximum-offset-step-mm",
        type=float,
        default=5.0,
        help="maximum proposed one-iteration in/out offset change",
    )
    parser.add_argument(
        "--maximum-velocity-step-mm-s",
        type=float,
        default=250.0,
        help="maximum proposed one-iteration turn velocity change",
    )
    parser.add_argument(
        "--maximum-alpha-step-deg-s2",
        type=float,
        default=5000.0,
        help="maximum proposed one-iteration turn alpha change",
    )
    add_turn_selection_args(parser)
    return parser.parse_args()


def add_turn_selection_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--runner",
        choices=("shortest", "search"),
        default="shortest",
    )
    parser.add_argument("--mode", type=int, default=2)
    parser.add_argument("--code", type=int, default=501)
    parser.add_argument("--search-index", type=int, default=0)
    parser.add_argument("--side", choices=("right", "left"), default="right")
    parser.add_argument("--entry-speed", type=float, default=None)
    parser.add_argument("--out-speed", type=float, default=None)
    parser.add_argument(
        "--vary",
        default="dist_in,dist_out,angle",
        help="parameters passed to turn_tune fit",
    )


def _load_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    lines: list[str] = []
    with path.open("r", encoding="utf-8", errors="ignore") as stream:
        for raw in stream:
            if raw.lstrip().startswith("#") or not raw.strip():
                continue
            lines.append(raw)
    if not lines:
        raise ValueError(f"no CSV rows in {path}")
    return list(csv.DictReader(lines))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _column(
    rows: Sequence[dict[str, str]],
    names: Sequence[str],
    *,
    required: bool = True,
) -> Optional[np.ndarray]:
    for name in names:
        if name not in rows[0]:
            continue
        values: list[float] = []
        for row in rows:
            try:
                values.append(float(row.get(name, "nan")))
            except ValueError:
                values.append(float("nan"))
        return np.asarray(values, dtype=float)
    if required:
        raise ValueError(f"missing required column; tried {', '.join(names)}")
    return None


def _unwrap_degrees(values: np.ndarray) -> np.ndarray:
    return np.degrees(np.unwrap(np.radians(values)))


def _gradient(values: np.ndarray, time_s: np.ndarray) -> np.ndarray:
    if len(values) < 2:
        return np.zeros_like(values)
    safe_time = time_s.copy()
    for index in range(1, len(safe_time)):
        if safe_time[index] <= safe_time[index - 1]:
            safe_time[index] = safe_time[index - 1] + 1e-6
    return np.gradient(values, safe_time)


def _longest_true_run(mask: np.ndarray) -> tuple[int, int]:
    best_start = 0
    best_end = -1
    current_start: Optional[int] = None
    for index, value in enumerate(mask):
        if value and current_start is None:
            current_start = index
        if current_start is not None and (not value or index == len(mask) - 1):
            end = index if value and index == len(mask) - 1 else index - 1
            if end - current_start > best_end - best_start:
                best_start, best_end = current_start, end
            current_start = None
    return best_start, best_end


def _median(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    if not len(finite):
        raise ValueError("pose window contains no finite values")
    return float(np.median(finite))


def _pose_window(
    time_s: np.ndarray,
    boundary_s: float,
    duration_s: float,
    before: bool,
) -> np.ndarray:
    if before:
        mask = (time_s >= boundary_s - duration_s) & (time_s <= boundary_s)
    else:
        mask = (time_s >= boundary_s) & (time_s <= boundary_s + duration_s)
    indexes = np.flatnonzero(mask)
    if len(indexes) >= 2:
        return indexes
    nearest = int(np.argmin(np.abs(time_s - boundary_s)))
    count = min(5, len(time_s))
    if before:
        return np.arange(max(0, nearest - count + 1), nearest + 1)
    return np.arange(nearest, min(len(time_s), nearest + count))


def _tracked_to_reference(
    x_mm: np.ndarray,
    y_mm: np.ndarray,
    yaw_deg: np.ndarray,
    anchor_right_mm: float,
    anchor_forward_mm: float,
) -> tuple[np.ndarray, np.ndarray]:
    yaw_rad = np.radians(yaw_deg)
    right_x = np.sin(yaw_rad)
    right_y = -np.cos(yaw_rad)
    forward_x = np.cos(yaw_rad)
    forward_y = np.sin(yaw_rad)
    reference_x = x_mm - anchor_right_mm * right_x - anchor_forward_mm * forward_x
    reference_y = y_mm - anchor_right_mm * right_y - anchor_forward_mm * forward_y
    return reference_x, reference_y


def analyze_trial(path: Path, args: argparse.Namespace) -> Trial:
    rows = _load_csv_rows(path)
    time_s = _column(rows, ("video_pts_s", "time_s"))
    assert time_s is not None
    x_mm = _column(rows, ("x_mm",), required=False)
    y_mm = _column(rows, ("y_mm",), required=False)
    if x_mm is None or y_mm is None:
        if args.cell_size_mm is None:
            raise ValueError(f"{path}: x_mm/y_mm absent; provide --cell-size-mm")
        x_cell = _column(rows, ("x_cell",))
        y_cell = _column(rows, ("y_cell",))
        assert x_cell is not None and y_cell is not None
        x_mm = x_cell * args.cell_size_mm
        y_mm = y_cell * args.cell_size_mm
    yaw = _column(
        rows,
        ("yaw_deg_unwrapped", "yaw_deg", "heading_deg_unwrapped"),
    )
    assert yaw is not None
    yaw = _unwrap_degrees(yaw)

    valid_column = _column(
        rows,
        ("pose_valid", "tracking_valid"),
        required=False,
    )
    heading_valid_column = _column(
        rows,
        ("heading_valid",),
        required=False,
    )
    finite_position = np.isfinite(time_s) & np.isfinite(x_mm) & np.isfinite(y_mm)
    valid = finite_position
    if valid_column is not None:
        valid &= valid_column > 0.5
    heading_valid = valid & np.isfinite(yaw)
    if heading_valid_column is not None:
        heading_valid &= heading_valid_column > 0.5
    if np.count_nonzero(valid) < 2:
        raise ValueError(f"{path}: fewer than two valid poses")
    if np.count_nonzero(heading_valid) < 2:
        raise ValueError(f"{path}: fewer than two valid headings")
    if not np.all(valid):
        good_position = np.flatnonzero(valid)
        indexes = np.arange(len(rows))
        x_mm = np.interp(indexes, good_position, x_mm[good_position])
        y_mm = np.interp(indexes, good_position, y_mm[good_position])
    if not np.all(heading_valid):
        good_heading = np.flatnonzero(heading_valid)
        indexes = np.arange(len(rows))
        yaw = np.interp(indexes, good_heading, yaw[good_heading])

    x_mm, y_mm = _tracked_to_reference(
        x_mm,
        y_mm,
        yaw,
        args.anchor_right_mm,
        args.anchor_forward_mm,
    )
    speed = _column(rows, ("speed_mm_s",), required=False)
    if speed is None:
        speed = np.hypot(_gradient(x_mm, time_s), _gradient(y_mm, time_s))

    source_mask = np.ones(len(rows), dtype=bool)
    if args.start_s is not None:
        source_mask &= time_s >= args.start_s
    if args.end_s is not None:
        source_mask &= time_s <= args.end_s
    source_indexes = np.flatnonzero(source_mask)
    if len(source_indexes) < 5:
        raise ValueError(f"{path}: selected interval has fewer than five rows")
    lo, hi = int(source_indexes[0]), int(source_indexes[-1])

    if args.start_s is not None and args.end_s is not None:
        motion_start_index, motion_end_index = lo, hi
    else:
        active = speed[lo : hi + 1] >= args.motion_threshold_mm_s
        active_start, active_end = _longest_true_run(active)
        if active_end < active_start:
            raise ValueError(
                f"{path}: no motion above {args.motion_threshold_mm_s} mm/s"
            )
        motion_start_index = lo + active_start
        motion_end_index = lo + active_end

    window_s = args.pose_window_ms / 1000.0
    start_indexes = _pose_window(
        time_s,
        float(time_s[motion_start_index]),
        window_s,
        before=True,
    )
    end_indexes = _pose_window(
        time_s,
        float(time_s[motion_end_index]),
        window_s,
        before=False,
    )
    start_pose_window_s = float(time_s[motion_start_index] - time_s[start_indexes[0]])
    end_pose_window_s = float(time_s[end_indexes[-1]] - time_s[motion_end_index])
    minimum_window_s = window_s * args.minimum_pose_window_coverage
    for label, value in (
        ("start", start_pose_window_s),
        ("end", end_pose_window_s),
    ):
        if value + 1e-9 < minimum_window_s:
            raise ValueError(
                f"{path}: {label} pose window spans {value:.3f}s; "
                f"requires at least {minimum_window_s:.3f}s"
            )
    evaluation = slice(int(start_indexes[0]), int(end_indexes[-1]) + 1)
    valid_fraction = float(np.mean(valid[evaluation]))
    if valid_fraction < args.minimum_valid_fraction:
        raise ValueError(
            f"{path}: selected-interval valid fraction "
            f"{valid_fraction:.3f} is below "
            f"{args.minimum_valid_fraction:.3f}"
        )
    heading_valid_fraction = float(np.mean(heading_valid[evaluation]))
    if heading_valid_fraction < args.minimum_heading_valid_fraction:
        raise ValueError(
            f"{path}: selected-interval heading-valid fraction "
            f"{heading_valid_fraction:.3f} is below "
            f"{args.minimum_heading_valid_fraction:.3f}"
        )
    start_pose_speed = _median(np.abs(speed[start_indexes]))
    end_pose_speed = _median(np.abs(speed[end_indexes]))
    for label, value in (
        ("start", start_pose_speed),
        ("end", end_pose_speed),
    ):
        if value > args.maximum_pose_window_speed_mm_s:
            raise ValueError(
                f"{path}: {label} pose-window median speed "
                f"{value:.3f} mm/s exceeds "
                f"{args.maximum_pose_window_speed_mm_s:.3f} mm/s"
            )
    start_x = _median(x_mm[start_indexes])
    start_y = _median(y_mm[start_indexes])
    start_yaw = _median(yaw[start_indexes])
    end_x = _median(x_mm[end_indexes])
    end_y = _median(y_mm[end_indexes])
    end_yaw = _median(yaw[end_indexes])

    dx = end_x - start_x
    dy = end_y - start_y
    heading = math.radians(start_yaw)
    x_right = math.sin(heading) * dx - math.cos(heading) * dy
    y_forward = math.cos(heading) * dx + math.sin(heading) * dy
    theta = end_yaw - start_yaw

    path_slice = slice(motion_start_index, motion_end_index + 1)
    path_x = x_mm[path_slice]
    path_y = y_mm[path_slice]
    path_length = float(np.sum(np.hypot(np.diff(path_x), np.diff(path_y))))
    peak_speed = float(np.nanmax(speed[path_slice]))
    return Trial(
        path=str(path.resolve()),
        sample_count=int(motion_end_index - motion_start_index + 1),
        source_start_s=float(time_s[lo]),
        source_end_s=float(time_s[hi]),
        motion_start_s=float(time_s[motion_start_index]),
        motion_end_s=float(time_s[motion_end_index]),
        duration_s=float(time_s[motion_end_index] - time_s[motion_start_index]),
        start_board_x_mm=start_x,
        start_board_y_mm=start_y,
        start_board_yaw_deg=start_yaw,
        end_board_x_mm=end_x,
        end_board_y_mm=end_y,
        end_board_yaw_deg=end_yaw,
        endpoint=Pose(x_right, y_forward, theta),
        path_length_mm=path_length,
        peak_speed_mm_s=peak_speed,
        yaw_change_abs_deg=abs(theta),
        tracking_valid_fraction=valid_fraction,
        heading_valid_fraction=heading_valid_fraction,
        start_pose_speed_median_mm_s=start_pose_speed,
        end_pose_speed_median_mm_s=end_pose_speed,
        start_pose_window_s=start_pose_window_s,
        end_pose_window_s=end_pose_window_s,
    )


def _summary(values: Sequence[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "std": float(np.std(array, ddof=1)) if len(array) > 1 else 0.0,
        "min": float(np.min(array)),
        "max": float(np.max(array)),
    }


def summarize_trials(trials: Sequence[Trial]) -> dict[str, Any]:
    x = [trial.endpoint.x_right_mm for trial in trials]
    y = [trial.endpoint.y_forward_mm for trial in trials]
    theta = [trial.endpoint.theta_deg for trial in trials]
    return {
        "count": len(trials),
        "endpoint": {
            "x_right_mm": _summary(x),
            "y_forward_mm": _summary(y),
            "theta_deg": _summary(theta),
        },
        "duration_s": _summary([trial.duration_s for trial in trials]),
        "path_length_mm": _summary([trial.path_length_mm for trial in trials]),
        "peak_speed_mm_s": _summary([trial.peak_speed_mm_s for trial in trials]),
        "tracking_valid_fraction": _summary(
            [trial.tracking_valid_fraction for trial in trials]
        ),
        "heading_valid_fraction": _summary(
            [trial.heading_valid_fraction for trial in trials]
        ),
    }


def _load_turn_tune_module() -> Any:
    path = Path(__file__).with_name("turn_tune.py")
    spec = importlib.util.spec_from_file_location("nightfall_turn_tune", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _selection_cli(args: argparse.Namespace) -> list[str]:
    result = ["--runner", args.runner]
    if args.runner == "shortest":
        result.extend(["--mode", str(args.mode), "--code", str(args.code)])
    else:
        result.extend(
            [
                "--search-index",
                str(args.search_index),
                "--side",
                args.side,
            ]
        )
    if args.entry_speed is not None:
        result.extend(["--entry-speed", str(args.entry_speed)])
    if args.out_speed is not None:
        result.extend(["--out-speed", str(args.out_speed)])
    return result


def _validate_candidate_deltas(
    parameter_deltas: dict[str, float],
    varied: set[str],
    delta_limits: dict[str, float],
) -> None:
    for name in sorted(varied):
        if name not in parameter_deltas:
            continue
        if abs(parameter_deltas[name]) > delta_limits[name]:
            raise ValueError(
                f"proposed {name} step {parameter_deltas[name]:.3f} "
                f"exceeds one-iteration limit "
                f"{delta_limits[name]:.3f}"
            )


def propose_fit(
    args: argparse.Namespace,
    aggregate: dict[str, Any],
) -> dict[str, Any]:
    if aggregate["count"] < args.minimum_fit_trials:
        raise ValueError(
            f"{aggregate['count']} trial(s) supplied; --propose-fit requires "
            f"at least {args.minimum_fit_trials}"
        )
    endpoint = aggregate["endpoint"]
    std_xy = max(
        endpoint["x_right_mm"]["std"],
        endpoint["y_forward_mm"]["std"],
    )
    std_yaw = endpoint["theta_deg"]["std"]
    if std_xy > args.maximum_endpoint_std_mm:
        raise ValueError(
            f"endpoint std {std_xy:.3f} mm exceeds fit gate "
            f"{args.maximum_endpoint_std_mm:.3f} mm"
        )
    if std_yaw > args.maximum_yaw_std_deg:
        raise ValueError(
            f"yaw std {std_yaw:.3f} deg exceeds fit gate "
            f"{args.maximum_yaw_std_deg:.3f} deg"
        )

    turn_tune = _load_turn_tune_module()
    parser = turn_tune.build_arg_parser()
    selection = _selection_cli(args)
    sim_args = parser.parse_args(["simulate", *selection, "--json"])
    constants = turn_tune.load_constants(sim_args)
    current_turn = turn_tune.resolve_turn(sim_args)
    current_sim = turn_tune.simulate_turn(
        current_turn,
        constants,
        sim_args.entry_speed,
        sim_args.out_speed,
    )
    current = turn_tune.pose_to_dict(current_sim.final_pose)
    observed = {
        "x_mm": endpoint["x_right_mm"]["median"],
        "y_mm": endpoint["y_forward_mm"]["median"],
        "theta_deg": endpoint["theta_deg"]["median"],
    }
    expected_yaw = current["theta_deg"]
    observed_yaw = observed["theta_deg"]
    if expected_yaw * observed_yaw <= 0.0:
        raise ValueError(
            "observed turn yaw has the wrong sign: expected "
            f"{expected_yaw:.3f} deg, observed {observed_yaw:.3f} deg"
        )
    yaw_error = expected_yaw - observed_yaw
    if abs(yaw_error) > args.maximum_turn_yaw_error_deg:
        raise ValueError(
            "observed turn yaw differs from the selected turn by "
            f"{yaw_error:.3f} deg, above "
            f"{args.maximum_turn_yaw_error_deg:.3f} deg"
        )
    correction = {
        "dx_mm": current["x_mm"] - observed["x_mm"],
        "dy_mm": current["y_mm"] - observed["y_mm"],
        "dtheta_deg": yaw_error,
    }
    target = {
        "x_mm": current["x_mm"] + args.feedback_gain * correction["dx_mm"],
        "y_mm": current["y_mm"] + args.feedback_gain * correction["dy_mm"],
        "theta_deg": current["theta_deg"]
        + args.feedback_gain * correction["dtheta_deg"],
    }
    fit_cli = [
        "fit",
        *selection,
        "--target-x",
        str(target["x_mm"]),
        "--target-y",
        str(target["y_mm"]),
        "--target-theta",
        str(target["theta_deg"]),
        "--vary",
        args.vary,
        "--json",
    ]
    fit_args = parser.parse_args(fit_cli)
    fit_turn = turn_tune.resolve_turn(fit_args)
    initial, fitted, values = turn_tune.fit_turn(
        fit_args,
        fit_turn,
        turn_tune.load_constants(fit_args),
    )
    parameter_deltas = {
        "velocity": (fitted.turn.velocity_mm_s - current_turn.velocity_mm_s),
        "alpha": fitted.turn.alpha_deg_s2 - current_turn.alpha_deg_s2,
        "dist_in": fitted.turn.dist_in_mm - current_turn.dist_in_mm,
        "dist_out": fitted.turn.dist_out_mm - current_turn.dist_out_mm,
        "angle": (
            abs(fitted.turn.signed_angle_deg) - abs(current_turn.signed_angle_deg)
        ),
    }
    delta_limits = {
        "velocity": args.maximum_velocity_step_mm_s,
        "alpha": args.maximum_alpha_step_deg_s2,
        "dist_in": args.maximum_offset_step_mm,
        "dist_out": args.maximum_offset_step_mm,
        "angle": args.maximum_angle_step_deg,
    }
    varied = {
        "angle" if name.strip() == "angle_abs" else name.strip()
        for name in args.vary.split(",")
    }
    _validate_candidate_deltas(
        parameter_deltas,
        varied,
        delta_limits,
    )
    assignments: dict[str, float] = {}
    for logical_name, field_name in fitted.turn.source_fields.items():
        if logical_name not in varied:
            continue
        if logical_name == "angle":
            assignments[field_name] = abs(fitted.turn.signed_angle_deg)
        elif logical_name == "velocity":
            assignments[field_name] = fitted.turn.velocity_mm_s
        elif logical_name == "alpha":
            assignments[field_name] = fitted.turn.alpha_deg_s2
        elif logical_name == "dist_in":
            assignments[field_name] = fitted.turn.dist_in_mm
        elif logical_name == "dist_out":
            assignments[field_name] = fitted.turn.dist_out_mm
    return {
        "method": "bounded first-order model correction",
        "feedback_gain": args.feedback_gain,
        "fit_gate": {
            "passed": True,
            "endpoint_std_mm": std_xy,
            "yaw_std_deg": std_yaw,
            "expected_turn_yaw_deg": expected_yaw,
            "observed_turn_yaw_deg": observed_yaw,
            "turn_yaw_error_deg": yaw_error,
            "maximum_turn_yaw_error_deg": (args.maximum_turn_yaw_error_deg),
        },
        "current_simulated_endpoint": current,
        "observed_median_endpoint": observed,
        "desired_minus_observed": correction,
        "next_simulation_target": target,
        "vary": args.vary.split(","),
        "initial": turn_tune.sim_to_dict(initial),
        "fitted": turn_tune.sim_to_dict(fitted),
        "suggested_values": values,
        "suggested_assignments": assignments,
        "parameter_deltas": parameter_deltas,
        "parameter_delta_limits": delta_limits,
        "safety": (
            "candidate only; no source file was edited and the next run "
            "requires the normal lifted/secured HIL procedure"
        ),
    }


def print_report(report: dict[str, Any]) -> None:
    aggregate = report["aggregate"]
    endpoint = aggregate["endpoint"]
    print(
        "[VIDEO-TURN] trials={} valid_fraction_min={:.3f} "
        "heading_valid_fraction_min={:.3f}".format(
            aggregate["count"],
            aggregate["tracking_valid_fraction"]["min"],
            aggregate["heading_valid_fraction"]["min"],
        )
    )
    print(
        "[VIDEO-TURN] endpoint_median "
        "x_right={:.3f}mm y_forward={:.3f}mm theta={:.3f}deg".format(
            endpoint["x_right_mm"]["median"],
            endpoint["y_forward_mm"]["median"],
            endpoint["theta_deg"]["median"],
        )
    )
    print(
        "[VIDEO-TURN] repeatability_std x={:.3f}mm y={:.3f}mm theta={:.3f}deg".format(
            endpoint["x_right_mm"]["std"],
            endpoint["y_forward_mm"]["std"],
            endpoint["theta_deg"]["std"],
        )
    )
    if "proposal" in report:
        proposal = report["proposal"]
        correction = proposal["desired_minus_observed"]
        print(
            "[VIDEO-TURN] measured_error "
            "dx={:.3f}mm dy={:.3f}mm dtheta={:.3f}deg".format(
                correction["dx_mm"],
                correction["dy_mm"],
                correction["dtheta_deg"],
            )
        )
        print("[VIDEO-TURN] candidate_assignments")
        for name, value in proposal["suggested_assignments"].items():
            print(f"  .{name} = {value:.3f}f,")
        print("[VIDEO-TURN] candidate_only=1 source_files_edited=0")


def validate_args(args: argparse.Namespace) -> None:
    float_arguments = (
        "cell_size_mm",
        "start_s",
        "end_s",
        "motion_threshold_mm_s",
        "pose_window_ms",
        "maximum_pose_window_speed_mm_s",
        "minimum_pose_window_coverage",
        "anchor_right_mm",
        "anchor_forward_mm",
        "minimum_valid_fraction",
        "minimum_heading_valid_fraction",
        "maximum_endpoint_std_mm",
        "maximum_yaw_std_deg",
        "feedback_gain",
        "maximum_turn_yaw_error_deg",
        "maximum_angle_step_deg",
        "maximum_offset_step_mm",
        "maximum_velocity_step_mm_s",
        "maximum_alpha_step_deg_s2",
        "entry_speed",
        "out_speed",
    )
    for name in float_arguments:
        value = getattr(args, name)
        if value is not None and not math.isfinite(value):
            raise ValueError(f"--{name.replace('_', '-')} must be finite")
    if args.cell_size_mm is not None and args.cell_size_mm <= 0.0:
        raise ValueError("--cell-size-mm must be positive")
    if args.start_s is not None and args.end_s is not None:
        if args.end_s <= args.start_s:
            raise ValueError("--end-s must be greater than --start-s")
    if args.pose_window_ms <= 0.0:
        raise ValueError("--pose-window-ms must be positive")
    if args.motion_threshold_mm_s <= 0.0:
        raise ValueError("--motion-threshold-mm-s must be positive")
    if args.maximum_pose_window_speed_mm_s < 0.0:
        raise ValueError("--maximum-pose-window-speed-mm-s must be non-negative")
    if not 0.0 < args.minimum_pose_window_coverage <= 1.0:
        raise ValueError("--minimum-pose-window-coverage must be in (0, 1]")
    if not 0.0 < args.minimum_valid_fraction <= 1.0:
        raise ValueError("--minimum-valid-fraction must be in (0, 1]")
    if not 0.0 < args.minimum_heading_valid_fraction <= 1.0:
        raise ValueError("--minimum-heading-valid-fraction must be in (0, 1]")
    if not 0.0 < args.feedback_gain <= 1.0:
        raise ValueError("--feedback-gain must be in (0, 1]")
    if args.minimum_fit_trials < 1:
        raise ValueError("--minimum-fit-trials must be positive")
    if args.maximum_endpoint_std_mm < 0.0:
        raise ValueError("--maximum-endpoint-std-mm must be non-negative")
    if args.maximum_yaw_std_deg < 0.0:
        raise ValueError("--maximum-yaw-std-deg must be non-negative")
    for name in (
        "maximum_turn_yaw_error_deg",
        "maximum_angle_step_deg",
        "maximum_offset_step_mm",
        "maximum_velocity_step_mm_s",
        "maximum_alpha_step_deg_s2",
    ):
        if getattr(args, name) <= 0.0:
            raise ValueError(f"--{name.replace('_', '-')} must be positive")

    resolved = [path.resolve() for path in args.trajectory_csv]
    if len(set(resolved)) != len(resolved):
        raise ValueError("the same trajectory path was supplied more than once")
    if len(resolved) > 1:
        hashes: dict[str, Path] = {}
        for path in resolved:
            if not path.is_file():
                raise FileNotFoundError(path)
            digest = _sha256(path)
            if digest in hashes:
                raise ValueError(
                    f"duplicate trajectory content: {hashes[digest]} and {path}"
                )
            hashes[digest] = path


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        trials = [analyze_trial(path, args) for path in args.trajectory_csv]
        aggregate = summarize_trials(trials)
        report: dict[str, Any] = {
            "schema": "nightfall_video_turn_report_v1",
            "coordinate_frame": {
                "x_right_mm": "vehicle right is positive",
                "y_forward_mm": "vehicle forward is positive",
                "theta_deg": "left turn is positive",
            },
            "analysis": {
                "motion_threshold_mm_s": args.motion_threshold_mm_s,
                "pose_window_ms": args.pose_window_ms,
                "maximum_pose_window_speed_mm_s": (args.maximum_pose_window_speed_mm_s),
                "minimum_pose_window_coverage": (args.minimum_pose_window_coverage),
                "anchor_right_mm": args.anchor_right_mm,
                "anchor_forward_mm": args.anchor_forward_mm,
                "minimum_valid_fraction": args.minimum_valid_fraction,
                "minimum_heading_valid_fraction": (args.minimum_heading_valid_fraction),
                "turn_selection": {
                    "runner": args.runner,
                    "mode": args.mode,
                    "code": args.code,
                    "search_index": args.search_index,
                    "side": args.side,
                    "entry_speed_mm_s": args.entry_speed,
                    "out_speed_mm_s": args.out_speed,
                },
            },
            "trials": [
                {
                    **asdict(trial),
                    "endpoint": asdict(trial.endpoint),
                }
                for trial in trials
            ],
            "aggregate": aggregate,
        }
        if args.propose_fit:
            report["proposal"] = propose_fit(args, aggregate)
        if args.report_json is not None:
            args.report_json.parent.mkdir(parents=True, exist_ok=True)
            args.report_json.write_text(
                json.dumps(
                    report,
                    indent=2,
                    sort_keys=True,
                    allow_nan=False,
                )
                + "\n",
                encoding="utf-8",
            )
        print_report(report)
        return 0
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(f"[VIDEO-TURN][ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
