#!/usr/bin/env python3
"""F413 turn trajectory simulator, trace replay, and parameter fitter.

Coordinate convention:
  x_mm: right side of the mouse is positive
  y_mm: forward is positive
  theta_deg: left turn is positive, right turn is negative
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Optional


DT_S = 0.001
FLAG_MOTOR_FWD = 0x0004
FLAG_MOTOR_REV = 0x0010
DEFAULT_OMEGA_THRESHOLD_DPS = 10.0


@dataclass(frozen=True)
class Constants:
    rounding_scale: float
    omega_cap_deg_s: float


@dataclass(frozen=True)
class SmoothProfile:
    omega_peak_deg_s: float
    t_acc_s: float
    t_cruise_s: float
    t_total_s: float
    ticks: int


@dataclass(frozen=True)
class TurnSpec:
    runner: str
    label: str
    signed_angle_deg: float
    alpha_deg_s2: float
    velocity_mm_s: float
    dist_in_mm: float
    dist_out_mm: float
    source_fields: dict[str, str]


@dataclass(frozen=True)
class Pose:
    x_mm: float = 0.0
    y_mm: float = 0.0
    theta_rad: float = 0.0

    @property
    def theta_deg(self) -> float:
        return math.degrees(self.theta_rad)


@dataclass(frozen=True)
class PathSample:
    t_ms: int
    phase: str
    x_mm: float
    y_mm: float
    theta_deg: float
    velocity_mm_s: float
    omega_deg_s: float


@dataclass(frozen=True)
class SimResult:
    turn: TurnSpec
    profile: SmoothProfile
    entry_speed_mm_s: float
    out_speed_mm_s: float
    final_speed_mm_s: float
    final_pose: Pose
    duration_ms: int
    samples: list[PathSample]


@dataclass(frozen=True)
class TraceRecord:
    timestamp_ms: int
    flags: int
    target_velocity_mm_s: float
    real_velocity_mm_s: float
    target_omega_deg_s: float
    real_omega_deg_s: float
    target_distance_mm: Optional[float]
    distance_mm: Optional[float]
    target_angle_deg: Optional[float]
    angle_deg: Optional[float]


@dataclass(frozen=True)
class ReplayResult:
    path: Path
    phase: str
    records: int
    duration_ms: int
    median_period_ms: float
    target_pose: Pose
    real_pose: Pose
    target_distance_delta_mm: Optional[float]
    real_distance_delta_mm: Optional[float]
    target_angle_delta_deg: Optional[float]
    real_angle_delta_deg: Optional[float]
    omega_active_ms: int


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _strip_c_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//.*", "", text)


def _clean_number(value: str) -> Optional[float]:
    value = value.strip()
    value = value.strip("()")
    value = re.sub(r"([0-9.])([fFuUlL]+)\b", r"\1", value)
    try:
        return float(value)
    except ValueError:
        return None


def _parse_define_float(path: Path, name: str, default: float) -> float:
    if not path.is_file():
        return default
    text = _strip_c_comments(path.read_text(encoding="utf-8", errors="ignore"))
    match = re.search(rf"^\s*#\s*define\s+{re.escape(name)}\s+(.+?)\s*$", text, re.M)
    if not match:
        return default
    parsed = _clean_number(match.group(1))
    return default if parsed is None else parsed


def load_constants(args: argparse.Namespace) -> Constants:
    root = repo_root()
    params_h = Path(args.params_h) if args.params_h else root / "params/f413_preorder/params.h"
    path_h = (
        Path(args.f413_path_h)
        if args.f413_path_h
        else root / "platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc/f413_path_run.h"
    )
    return Constants(
        rounding_scale=_parse_define_float(params_h, "TURN_OMEGA_PROFILE_ROUNDING_SCALE", 1.2),
        omega_cap_deg_s=_parse_define_float(path_h, "NIGHTFALL_F413_PATH_OMEGA_CAP", 2200.0),
    )


def _find_matching_brace(text: str, open_index: int) -> int:
    depth = 0
    for i in range(open_index, len(text)):
        ch = text[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return i
    raise ValueError("missing matching brace")


def _parse_fields(block: str) -> dict[str, float]:
    fields: dict[str, float] = {}
    for match in re.finditer(r"\.([A-Za-z_][A-Za-z0-9_]*)\s*=\s*([^,}\n]+)", block):
        parsed = _clean_number(match.group(2))
        if parsed is not None:
            fields[match.group(1)] = parsed
    return fields


def _top_level_child_blocks(block: str) -> list[str]:
    children: list[str] = []
    depth = 0
    start: Optional[int] = None
    for i, ch in enumerate(block):
        if ch == "{":
            if depth == 0:
                start = i
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0 and start is not None:
                children.append(block[start : i + 1])
                start = None
    return children


def parse_shortest_modes(path: Path) -> dict[int, dict[str, float]]:
    text = _strip_c_comments(path.read_text(encoding="utf-8", errors="ignore"))
    modes: dict[int, dict[str, float]] = {}
    for match in re.finditer(r"const\s+ShortestRunModeParams_t\s+shortestRunModeParams(\d+)\s*=", text):
        mode = int(match.group(1))
        open_index = text.find("{", match.end())
        if open_index < 0:
            continue
        close_index = _find_matching_brace(text, open_index)
        modes[mode] = _parse_fields(text[open_index : close_index + 1])
    return modes


def parse_search_params(path: Path) -> list[dict[str, float]]:
    text = _strip_c_comments(path.read_text(encoding="utf-8", errors="ignore"))
    match = re.search(r"const\s+SearchRunParams_t\s+searchRunParams\s*\[[^\]]*\]\s*=", text)
    if not match:
        return []
    open_index = text.find("{", match.end())
    if open_index < 0:
        return []
    close_index = _find_matching_brace(text, open_index)
    outer = text[open_index + 1 : close_index]
    return [_parse_fields(child) for child in _top_level_child_blocks(outer)]


def _field(params: dict[str, float], name: str, fallback: float) -> float:
    value = params.get(name, fallback)
    return fallback if value <= 0.0 else value


def shortest_turn_from_code(code: int, params: dict[str, float]) -> TurnSpec:
    mapping: dict[int, tuple[str, bool, str, str, str, str, str]] = {
        300: ("small_r90", True, "velocity_turn90", "alpha_turn90", "angle_turn_90", "dist_offset_in", "dist_offset_out"),
        400: ("small_l90", False, "velocity_turn90", "alpha_turn90", "angle_turn_90", "dist_offset_in", "dist_offset_out"),
        501: ("large_r90", True, "velocity_l_turn_90", "alpha_l_turn_90", "angle_l_turn_90", "dist_l_turn_in_90", "dist_l_turn_out_90"),
        601: ("large_l90", False, "velocity_l_turn_90", "alpha_l_turn_90", "angle_l_turn_90", "dist_l_turn_in_90", "dist_l_turn_out_90"),
        502: ("large_r180", True, "velocity_l_turn_180", "alpha_l_turn_180", "angle_l_turn_180", "dist_l_turn_in_180", "dist_l_turn_out_180"),
        602: ("large_l180", False, "velocity_l_turn_180", "alpha_l_turn_180", "angle_l_turn_180", "dist_l_turn_in_180", "dist_l_turn_out_180"),
        701: ("r45in", True, "velocity_turn45in", "alpha_turn45in", "angle_turn45in", "dist_turn45in_in", "dist_turn45in_out"),
        702: ("l45in", False, "velocity_turn45in", "alpha_turn45in", "angle_turn45in", "dist_turn45in_in", "dist_turn45in_out"),
        703: ("r45out", True, "velocity_turn45out", "alpha_turn45out", "angle_turn45out", "dist_turn45out_in", "dist_turn45out_out"),
        704: ("l45out", False, "velocity_turn45out", "alpha_turn45out", "angle_turn45out", "dist_turn45out_in", "dist_turn45out_out"),
        801: ("v90r", True, "velocity_turnV90", "alpha_turnV90", "angle_turnV90", "dist_turnV90_in", "dist_turnV90_out"),
        802: ("v90l", False, "velocity_turnV90", "alpha_turnV90", "angle_turnV90", "dist_turnV90_in", "dist_turnV90_out"),
        901: ("r135in", True, "velocity_turn135in", "alpha_turn135in", "angle_turn135in", "dist_turn135in_in", "dist_turn135in_out"),
        902: ("l135in", False, "velocity_turn135in", "alpha_turn135in", "angle_turn135in", "dist_turn135in_in", "dist_turn135in_out"),
        903: ("r135out", True, "velocity_turn135out", "alpha_turn135out", "angle_turn135out", "dist_turn135out_in", "dist_turn135out_out"),
        904: ("l135out", False, "velocity_turn135out", "alpha_turn135out", "angle_turn135out", "dist_turn135out_in", "dist_turn135out_out"),
    }
    if code not in mapping:
        raise ValueError(f"unsupported shortest path turn code: {code}")
    label, right, v_field, alpha_field, angle_field, in_field, out_field = mapping[code]
    angle_abs = _field(params, angle_field, 90.0)
    signed_angle = -abs(angle_abs) if right else abs(angle_abs)
    return TurnSpec(
        runner="shortest",
        label=f"code{code}:{label}",
        signed_angle_deg=signed_angle,
        alpha_deg_s2=_field(params, alpha_field, 10000.0),
        velocity_mm_s=_field(params, v_field, 200.0),
        dist_in_mm=params.get(in_field, 0.0),
        dist_out_mm=params.get(out_field, 0.0),
        source_fields={"velocity": v_field, "alpha": alpha_field, "angle": angle_field, "dist_in": in_field, "dist_out": out_field},
    )


def search_turn_from_params(index: int, side: str, params: dict[str, float]) -> TurnSpec:
    right = side == "right"
    angle_abs = _field(params, "angle_turn_90", 90.0)
    return TurnSpec(
        runner="search",
        label=f"search[{index}]:{'r90' if right else 'l90'}",
        signed_angle_deg=-abs(angle_abs) if right else abs(angle_abs),
        alpha_deg_s2=_field(params, "alpha_turn90", 10000.0),
        velocity_mm_s=_field(params, "velocity_turn90", 200.0),
        dist_in_mm=params.get("dist_offset_in", 0.0),
        dist_out_mm=params.get("dist_offset_out", 0.0),
        source_fields={"velocity": "velocity_turn90", "alpha": "alpha_turn90", "angle": "angle_turn_90", "dist_in": "dist_offset_in", "dist_out": "dist_offset_out"},
    )


def apply_overrides(turn: TurnSpec, args: argparse.Namespace) -> TurnSpec:
    return TurnSpec(
        runner=turn.runner,
        label=turn.label,
        signed_angle_deg=args.angle if args.angle is not None else turn.signed_angle_deg,
        alpha_deg_s2=args.alpha if args.alpha is not None else turn.alpha_deg_s2,
        velocity_mm_s=args.velocity if args.velocity is not None else turn.velocity_mm_s,
        dist_in_mm=args.dist_in if args.dist_in is not None else turn.dist_in_mm,
        dist_out_mm=args.dist_out if args.dist_out is not None else turn.dist_out_mm,
        source_fields=turn.source_fields,
    )


def resolve_turn(args: argparse.Namespace) -> TurnSpec:
    root = repo_root()
    if args.runner == "manual":
        missing = [name for name in ("angle", "alpha", "velocity", "dist_in", "dist_out") if getattr(args, name) is None]
        if missing:
            raise ValueError(f"manual runner requires: {', '.join('--' + name.replace('_', '-') for name in missing)}")
        turn = TurnSpec(
            runner="manual",
            label="manual",
            signed_angle_deg=args.angle,
            alpha_deg_s2=args.alpha,
            velocity_mm_s=args.velocity,
            dist_in_mm=args.dist_in,
            dist_out_mm=args.dist_out,
            source_fields={},
        )
        return turn

    if args.runner == "shortest":
        path = Path(args.shortest_params) if args.shortest_params else root / "params/f413_preorder/shortest_run_params_split.c"
        modes = parse_shortest_modes(path)
        if args.mode not in modes:
            raise ValueError(f"shortest mode {args.mode} not found in {path}")
        return apply_overrides(shortest_turn_from_code(args.code, modes[args.mode]), args)

    if args.runner == "search":
        path = Path(args.search_params) if args.search_params else root / "params/f413_preorder/search_run_params_split.c"
        entries = parse_search_params(path)
        if args.search_index < 0 or args.search_index >= len(entries):
            raise ValueError(f"search index {args.search_index} not found in {path}")
        return apply_overrides(search_turn_from_params(args.search_index, args.side, entries[args.search_index]), args)

    raise ValueError(f"unknown runner: {args.runner}")


def build_profile(turn: TurnSpec, constants: Constants) -> SmoothProfile:
    angle_abs = abs(turn.signed_angle_deg)
    if angle_abs <= 0.0 or turn.alpha_deg_s2 <= 0.0:
        return SmoothProfile(0.0, 0.0, 0.0, 0.0, 0)

    omega_peak = math.sqrt((2.0 * turn.alpha_deg_s2 * angle_abs) / 3.0)
    if constants.omega_cap_deg_s > 0.0:
        omega_peak = min(abs(omega_peak), constants.omega_cap_deg_s)
    if omega_peak <= 0.0:
        return SmoothProfile(0.0, 0.0, 0.0, 0.0, 0)

    rounding_scale = max(constants.rounding_scale, 0.1)
    t_acc = (omega_peak / turn.alpha_deg_s2) * rounding_scale
    t_cruise = (angle_abs / omega_peak) - t_acc
    if t_cruise < 0.0:
        t_cruise = 0.0
        omega_peak = angle_abs / t_acc
        if constants.omega_cap_deg_s > 0.0:
            omega_peak = min(abs(omega_peak), constants.omega_cap_deg_s)
        t_cruise = max(0.0, (angle_abs / omega_peak) - t_acc)
    t_total = (2.0 * t_acc) + t_cruise
    return SmoothProfile(
        omega_peak_deg_s=omega_peak,
        t_acc_s=t_acc,
        t_cruise_s=t_cruise,
        t_total_s=t_total,
        ticks=int(math.ceil(t_total / DT_S - 1e-12)),
    )


def sample_omega_deg_s(profile: SmoothProfile, signed_peak_deg_s: float, t_s: float) -> float:
    if profile.t_total_s <= 0.0 or profile.t_acc_s <= 0.0 or signed_peak_deg_s == 0.0 or t_s <= 0.0:
        return 0.0
    omega_peak_abs = abs(signed_peak_deg_s)
    if t_s < profile.t_acc_s:
        omega_abs = 0.5 * omega_peak_abs * (1.0 - math.cos(math.pi * (t_s / profile.t_acc_s)))
    elif t_s < profile.t_acc_s + profile.t_cruise_s:
        omega_abs = omega_peak_abs
    elif t_s < profile.t_total_s:
        td = t_s - (profile.t_acc_s + profile.t_cruise_s)
        omega_abs = 0.5 * omega_peak_abs * (1.0 + math.cos(math.pi * (td / profile.t_acc_s)))
    else:
        return 0.0
    return -omega_abs if signed_peak_deg_s < 0.0 else omega_abs


def advance_pose(pose: Pose, velocity_mm_s: float, omega_deg_s: float, dt_s: float) -> Pose:
    theta = pose.theta_rad
    omega_rad_s = math.radians(omega_deg_s)
    if abs(omega_rad_s) < 1e-12:
        dx = -velocity_mm_s * math.sin(theta) * dt_s
        dy = velocity_mm_s * math.cos(theta) * dt_s
        dtheta = 0.0
    else:
        dtheta = omega_rad_s * dt_s
        theta_next = theta + dtheta
        dx = (velocity_mm_s / omega_rad_s) * (math.cos(theta_next) - math.cos(theta))
        dy = (velocity_mm_s / omega_rad_s) * (math.sin(theta_next) - math.sin(theta))
    return Pose(pose.x_mm + dx, pose.y_mm + dy, theta + dtheta)


def _sample(time_s: float, phase: str, pose: Pose, velocity: float, omega: float) -> PathSample:
    return PathSample(
        t_ms=int(round(time_s * 1000.0)),
        phase=phase,
        x_mm=pose.x_mm,
        y_mm=pose.y_mm,
        theta_deg=pose.theta_deg,
        velocity_mm_s=velocity,
        omega_deg_s=omega,
    )


def _simulate_straight(
    pose: Pose,
    time_s: float,
    phase: str,
    distance_mm: float,
    start_velocity_mm_s: float,
    target_velocity_mm_s: float,
    samples: list[PathSample],
) -> tuple[Pose, float, float]:
    if distance_mm <= 0.0:
        return pose, target_velocity_mm_s, time_s

    velocity = start_velocity_mm_s
    acceleration = (target_velocity_mm_s * target_velocity_mm_s - start_velocity_mm_s * start_velocity_mm_s) / (2.0 * distance_mm)
    traveled = 0.0
    guard = 0
    while traveled < distance_mm:
        guard += 1
        if guard > 1_000_000:
            raise RuntimeError("straight simulation did not converge")
        velocity += acceleration * DT_S
        if acceleration > 0.0 and velocity > target_velocity_mm_s:
            velocity = target_velocity_mm_s
        elif acceleration < 0.0 and velocity < target_velocity_mm_s:
            velocity = target_velocity_mm_s
        pose = advance_pose(pose, velocity, 0.0, DT_S)
        traveled += max(0.0, velocity * DT_S)
        time_s += DT_S
        samples.append(_sample(time_s, phase, pose, velocity, 0.0))
        if velocity <= 0.0 and target_velocity_mm_s <= 0.0 and acceleration <= 0.0:
            break
    return pose, target_velocity_mm_s, time_s


def simulate_turn(
    turn: TurnSpec,
    constants: Constants,
    entry_speed_mm_s: Optional[float] = None,
    out_speed_mm_s: Optional[float] = None,
) -> SimResult:
    entry_speed = turn.velocity_mm_s if entry_speed_mm_s is None else entry_speed_mm_s
    if out_speed_mm_s is None:
        out_speed = entry_speed if turn.runner == "search" else turn.velocity_mm_s
    else:
        out_speed = out_speed_mm_s

    profile = build_profile(turn, constants)
    samples: list[PathSample] = []
    pose = Pose()
    time_s = 0.0

    pose, speed, time_s = _simulate_straight(
        pose,
        time_s,
        "in_offset",
        turn.dist_in_mm,
        entry_speed,
        turn.velocity_mm_s,
        samples,
    )

    signed_peak = -profile.omega_peak_deg_s if turn.signed_angle_deg < 0.0 else profile.omega_peak_deg_s
    profile_elapsed = 0.0
    loop_elapsed = 0.0
    while loop_elapsed < profile.t_total_s - 1e-12:
        omega = sample_omega_deg_s(profile, signed_peak, profile_elapsed)
        pose = advance_pose(pose, turn.velocity_mm_s, omega, DT_S)
        profile_elapsed += DT_S
        loop_elapsed += DT_S
        time_s += DT_S
        samples.append(_sample(time_s, "turn_core", pose, turn.velocity_mm_s, omega))

    pose, speed, time_s = _simulate_straight(
        pose,
        time_s,
        "out_offset",
        turn.dist_out_mm,
        turn.velocity_mm_s,
        out_speed,
        samples,
    )

    return SimResult(
        turn=turn,
        profile=profile,
        entry_speed_mm_s=entry_speed,
        out_speed_mm_s=out_speed,
        final_speed_mm_s=speed,
        final_pose=pose,
        duration_ms=int(round(time_s * 1000.0)),
        samples=samples,
    )


def _parse_int(value: str) -> int:
    return int(value.strip(), 0)


def _parse_float(value: str) -> float:
    return float(value.strip())


def _row_float(row: dict[str, str], names: Iterable[str]) -> Optional[float]:
    for name in names:
        value = row.get(name)
        if value is None or value == "":
            continue
        try:
            return _parse_float(value)
        except ValueError:
            continue
    return None


def _row_int(row: dict[str, str], names: Iterable[str], default: int = 0) -> int:
    for name in names:
        value = row.get(name)
        if value is None or value == "":
            continue
        try:
            return _parse_int(value)
        except ValueError:
            continue
    return default


def resolve_csv_path(path_str: str) -> Path:
    path = Path(path_str).expanduser()
    if path.is_file():
        return path
    if path.is_dir():
        candidates = sorted(path.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
        if candidates:
            return candidates[0]
    raise FileNotFoundError(f"CSV path not found: {path}")


def load_trace_csv(path: Path) -> tuple[dict[str, str], list[TraceRecord]]:
    meta: dict[str, str] = {}
    columns: list[str] = []
    records: list[TraceRecord] = []
    with path.open("r", encoding="ascii", errors="ignore", newline="") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                key, sep, value = line.partition("=")
                if sep:
                    meta[key] = value
                if line.startswith("#mm_columns="):
                    columns = [c.strip() for c in value.split(",") if c.strip()]
                continue
            parts = [p.strip() for p in line.split(",")]
            if not columns and any(part == "timestamp_ms" for part in parts):
                columns = parts
                continue
            if not columns or len(parts) != len(columns):
                continue
            row = dict(zip(columns, parts))
            try:
                target_omega = _row_float(row, ("target_omega_mdps",))
                real_omega = _row_float(row, ("real_omega_mdps", "omega_z_mdps", "gyro_z_raw_mdps"))
                target_angle_mdeg = _row_float(row, ("target_angle_mdeg",))
                angle_mdeg = _row_float(row, ("angle_mdeg",))
                records.append(
                    TraceRecord(
                        timestamp_ms=_row_int(row, ("timestamp_ms",)),
                        flags=_row_int(row, ("flags",)),
                        target_velocity_mm_s=_row_float(row, ("target_velocity_mm_s",)) or 0.0,
                        real_velocity_mm_s=_row_float(row, ("real_velocity_mm_s", "velocity_mm_s")) or 0.0,
                        target_omega_deg_s=(target_omega or 0.0) / 1000.0,
                        real_omega_deg_s=(real_omega or 0.0) / 1000.0,
                        target_distance_mm=_row_float(row, ("target_distance_mm",)),
                        distance_mm=_row_float(row, ("distance_mm",)),
                        target_angle_deg=(target_angle_mdeg / 1000.0 if target_angle_mdeg is not None else None),
                        angle_deg=(angle_mdeg / 1000.0 if angle_mdeg is not None else None),
                    )
                )
            except ValueError:
                continue
    return meta, records


def select_records(
    records: list[TraceRecord],
    phase: str,
    start_ms: Optional[int],
    end_ms: Optional[int],
    omega_threshold_dps: float,
) -> tuple[str, list[TraceRecord]]:
    if not records:
        return phase, []

    selected_phase = phase
    selected = records
    if phase == "auto":
        if any(r.flags & FLAG_MOTOR_REV for r in records):
            selected_phase = "motor-rev"
        else:
            selected_phase = "all"

    if selected_phase == "motor-rev":
        selected = [r for r in records if r.flags & FLAG_MOTOR_REV]
    elif selected_phase == "motor-forward":
        selected = [r for r in records if r.flags & FLAG_MOTOR_FWD]
    elif selected_phase == "omega":
        indexes = [
            i
            for i, r in enumerate(records)
            if abs(r.target_omega_deg_s) >= omega_threshold_dps or abs(r.real_omega_deg_s) >= omega_threshold_dps
        ]
        selected = records[min(indexes) : max(indexes) + 1] if indexes else []
    elif selected_phase == "all":
        selected = records
    else:
        raise ValueError(f"unknown phase: {phase}")

    if selected and (start_ms is not None or end_ms is not None):
        origin = selected[0].timestamp_ms
        selected = [
            r
            for r in selected
            if (start_ms is None or r.timestamp_ms - origin >= start_ms)
            and (end_ms is None or r.timestamp_ms - origin <= end_ms)
        ]
    return selected_phase, selected


def _delta_optional(values: list[Optional[float]]) -> Optional[float]:
    available = [v for v in values if v is not None]
    if len(available) < 2:
        return None
    return available[-1] - available[0]


def integrate_records(records: list[TraceRecord], target: bool, integration: str) -> Pose:
    pose = Pose()
    if len(records) < 2:
        return pose
    for prev, cur in zip(records, records[1:]):
        dt_s = max(0.0, (cur.timestamp_ms - prev.timestamp_ms) / 1000.0)
        if target:
            v_prev, w_prev = prev.target_velocity_mm_s, prev.target_omega_deg_s
            v_cur, w_cur = cur.target_velocity_mm_s, cur.target_omega_deg_s
        else:
            v_prev, w_prev = prev.real_velocity_mm_s, prev.real_omega_deg_s
            v_cur, w_cur = cur.real_velocity_mm_s, cur.real_omega_deg_s
        if integration == "trapezoid":
            v = 0.5 * (v_prev + v_cur)
            w = 0.5 * (w_prev + w_cur)
        else:
            v = v_prev
            w = w_prev
        pose = advance_pose(pose, v, w, dt_s)
    return pose


def replay_trace(
    csv_path: Path,
    phase: str,
    start_ms: Optional[int],
    end_ms: Optional[int],
    omega_threshold_dps: float,
    integration: str,
) -> ReplayResult:
    _, records = load_trace_csv(csv_path)
    selected_phase, selected = select_records(records, phase, start_ms, end_ms, omega_threshold_dps)
    if not selected:
        raise ValueError("no records selected")
    periods = [selected[i].timestamp_ms - selected[i - 1].timestamp_ms for i in range(1, len(selected))]
    duration_ms = selected[-1].timestamp_ms - selected[0].timestamp_ms if len(selected) >= 2 else 0
    omega_active_ms = sum(
        max(0, selected[i].timestamp_ms - selected[i - 1].timestamp_ms)
        for i in range(1, len(selected))
        if abs(selected[i - 1].target_omega_deg_s) >= omega_threshold_dps
        or abs(selected[i - 1].real_omega_deg_s) >= omega_threshold_dps
    )
    return ReplayResult(
        path=csv_path,
        phase=selected_phase,
        records=len(selected),
        duration_ms=duration_ms,
        median_period_ms=float(statistics.median(periods)) if periods else 0.0,
        target_pose=integrate_records(selected, target=True, integration=integration),
        real_pose=integrate_records(selected, target=False, integration=integration),
        target_distance_delta_mm=_delta_optional([r.target_distance_mm for r in selected]),
        real_distance_delta_mm=_delta_optional([r.distance_mm for r in selected]),
        target_angle_delta_deg=_delta_optional([r.target_angle_deg for r in selected]),
        real_angle_delta_deg=_delta_optional([r.angle_deg for r in selected]),
        omega_active_ms=omega_active_ms,
    )


def angle_error_deg(actual: float, expected: float) -> float:
    err = actual - expected
    while err > 180.0:
        err -= 360.0
    while err < -180.0:
        err += 360.0
    return err


def pose_error(actual: Pose, expected: Pose) -> dict[str, float]:
    return {
        "dx_mm": actual.x_mm - expected.x_mm,
        "dy_mm": actual.y_mm - expected.y_mm,
        "dtheta_deg": angle_error_deg(actual.theta_deg, expected.theta_deg),
    }


def pose_to_dict(pose: Pose) -> dict[str, float]:
    return {"x_mm": pose.x_mm, "y_mm": pose.y_mm, "theta_deg": pose.theta_deg}


def profile_to_dict(profile: SmoothProfile) -> dict[str, float | int]:
    return {
        "omega_peak_deg_s": profile.omega_peak_deg_s,
        "t_acc_s": profile.t_acc_s,
        "t_cruise_s": profile.t_cruise_s,
        "t_total_s": profile.t_total_s,
        "ticks": profile.ticks,
    }


def sim_to_dict(result: SimResult, include_samples: bool = False) -> dict[str, object]:
    payload: dict[str, object] = {
        "turn": asdict(result.turn),
        "profile": profile_to_dict(result.profile),
        "entry_speed_mm_s": result.entry_speed_mm_s,
        "out_speed_mm_s": result.out_speed_mm_s,
        "final_speed_mm_s": result.final_speed_mm_s,
        "final_pose": pose_to_dict(result.final_pose),
        "duration_ms": result.duration_ms,
    }
    if include_samples:
        payload["samples"] = [asdict(sample) for sample in result.samples]
    return payload


def replay_to_dict(result: ReplayResult) -> dict[str, object]:
    return {
        "path": str(result.path),
        "phase": result.phase,
        "records": result.records,
        "duration_ms": result.duration_ms,
        "median_period_ms": result.median_period_ms,
        "target_pose": pose_to_dict(result.target_pose),
        "real_pose": pose_to_dict(result.real_pose),
        "real_minus_target": pose_error(result.real_pose, result.target_pose),
        "target_distance_delta_mm": result.target_distance_delta_mm,
        "real_distance_delta_mm": result.real_distance_delta_mm,
        "target_angle_delta_deg": result.target_angle_delta_deg,
        "real_angle_delta_deg": result.real_angle_delta_deg,
        "omega_active_ms": result.omega_active_ms,
    }


def print_sim_summary(result: SimResult) -> None:
    turn = result.turn
    profile = result.profile
    print(f"[TURN-SIM] source={turn.runner} label={turn.label}")
    print(
        "[TURN-SIM] params "
        f"v={turn.velocity_mm_s:.3f}mm/s alpha={turn.alpha_deg_s2:.3f}deg/s^2 "
        f"angle={turn.signed_angle_deg:.3f}deg in={turn.dist_in_mm:.3f}mm out={turn.dist_out_mm:.3f}mm"
    )
    print(
        "[TURN-SIM] profile "
        f"omega_peak={profile.omega_peak_deg_s:.3f}deg/s "
        f"t_acc={profile.t_acc_s * 1000.0:.1f}ms "
        f"t_cruise={profile.t_cruise_s * 1000.0:.1f}ms "
        f"t_total={profile.t_total_s * 1000.0:.1f}ms ticks={profile.ticks}"
    )
    print(
        "[TURN-SIM] speeds "
        f"entry={result.entry_speed_mm_s:.3f}mm/s out={result.out_speed_mm_s:.3f}mm/s "
        f"final={result.final_speed_mm_s:.3f}mm/s"
    )
    print(
        "[TURN-SIM] final_pose "
        f"x={result.final_pose.x_mm:.3f}mm y={result.final_pose.y_mm:.3f}mm "
        f"theta={result.final_pose.theta_deg:.3f}deg duration={result.duration_ms}ms"
    )


def print_replay_summary(result: ReplayResult, sim: Optional[SimResult] = None) -> None:
    print(f"[TURN-REPLAY] file={result.path}")
    print(
        "[TURN-REPLAY] selected "
        f"phase={result.phase} records={result.records} duration={result.duration_ms}ms "
        f"median_period={result.median_period_ms:.3f}ms omega_active={result.omega_active_ms}ms"
    )
    print(
        "[TURN-REPLAY] target_pose "
        f"x={result.target_pose.x_mm:.3f}mm y={result.target_pose.y_mm:.3f}mm "
        f"theta={result.target_pose.theta_deg:.3f}deg"
    )
    print(
        "[TURN-REPLAY] real_pose "
        f"x={result.real_pose.x_mm:.3f}mm y={result.real_pose.y_mm:.3f}mm "
        f"theta={result.real_pose.theta_deg:.3f}deg"
    )
    err = pose_error(result.real_pose, result.target_pose)
    print(
        "[TURN-REPLAY] real_minus_target "
        f"dx={err['dx_mm']:.3f}mm dy={err['dy_mm']:.3f}mm dtheta={err['dtheta_deg']:.3f}deg"
    )
    if result.target_distance_delta_mm is not None or result.real_distance_delta_mm is not None:
        print(
            "[TURN-REPLAY] logged_delta "
            f"target_distance={_fmt_optional(result.target_distance_delta_mm, 'mm')} "
            f"real_distance={_fmt_optional(result.real_distance_delta_mm, 'mm')} "
            f"target_angle={_fmt_optional(result.target_angle_delta_deg, 'deg')} "
            f"real_angle={_fmt_optional(result.real_angle_delta_deg, 'deg')}"
        )
    if sim is not None:
        err_target = pose_error(result.target_pose, sim.final_pose)
        err_real = pose_error(result.real_pose, sim.final_pose)
        print(
            "[TURN-REPLAY] target_minus_sim "
            f"dx={err_target['dx_mm']:.3f}mm dy={err_target['dy_mm']:.3f}mm "
            f"dtheta={err_target['dtheta_deg']:.3f}deg"
        )
        print(
            "[TURN-REPLAY] real_minus_sim "
            f"dx={err_real['dx_mm']:.3f}mm dy={err_real['dy_mm']:.3f}mm "
            f"dtheta={err_real['dtheta_deg']:.3f}deg"
        )


def _fmt_optional(value: Optional[float], unit: str) -> str:
    return "n/a" if value is None else f"{value:.3f}{unit}"


def export_samples_csv(path: Path, samples: list[PathSample]) -> None:
    with path.open("w", encoding="ascii", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=("t_ms", "phase", "x_mm", "y_mm", "theta_deg", "velocity_mm_s", "omega_deg_s"),
        )
        writer.writeheader()
        for sample in samples:
            writer.writerow(asdict(sample))


def _turn_with_values(base: TurnSpec, values: dict[str, float]) -> TurnSpec:
    sign = -1.0 if base.signed_angle_deg < 0.0 else 1.0
    return TurnSpec(
        runner=base.runner,
        label=base.label,
        signed_angle_deg=sign * values.get("angle_abs", abs(base.signed_angle_deg)),
        alpha_deg_s2=values.get("alpha", base.alpha_deg_s2),
        velocity_mm_s=values.get("velocity", base.velocity_mm_s),
        dist_in_mm=values.get("dist_in", base.dist_in_mm),
        dist_out_mm=values.get("dist_out", base.dist_out_mm),
        source_fields=base.source_fields,
    )


def fit_turn(args: argparse.Namespace, turn: TurnSpec, constants: Constants) -> tuple[SimResult, SimResult, dict[str, float]]:
    target_pose = Pose(args.target_x, args.target_y, math.radians(args.target_theta))
    initial_values = {
        "velocity": turn.velocity_mm_s,
        "alpha": turn.alpha_deg_s2,
        "dist_in": turn.dist_in_mm,
        "dist_out": turn.dist_out_mm,
        "angle_abs": abs(turn.signed_angle_deg),
    }
    values = dict(initial_values)
    vary = set(args.vary.split(","))
    if "angle" in vary:
        vary.remove("angle")
        vary.add("angle_abs")
    allowed = {"velocity", "alpha", "dist_in", "dist_out", "angle_abs"}
    unknown = vary - allowed
    if unknown:
        raise ValueError(f"unknown fit variables: {', '.join(sorted(unknown))}")

    steps = {
        "velocity": args.step_velocity,
        "alpha": args.step_alpha,
        "dist_in": args.step_dist,
        "dist_out": args.step_dist,
        "angle_abs": args.step_angle,
    }
    bounds = {
        "velocity": (1.0, args.max_velocity),
        "alpha": (1.0, args.max_alpha),
        "dist_in": (args.min_offset, args.max_offset),
        "dist_out": (args.min_offset, args.max_offset),
        "angle_abs": (1.0, 360.0),
    }

    def clamp(name: str, value: float) -> float:
        low, high = bounds[name]
        return min(high, max(low, value))

    def cost_for(candidate_values: dict[str, float]) -> float:
        candidate = _turn_with_values(turn, candidate_values)
        sim = simulate_turn(candidate, constants, args.entry_speed, args.out_speed)
        err = pose_error(sim.final_pose, target_pose)
        pose_cost = (
            err["dx_mm"] * err["dx_mm"]
            + err["dy_mm"] * err["dy_mm"]
            + (args.theta_weight_mm_per_deg * err["dtheta_deg"]) ** 2
        )
        reg = 0.0
        if args.regularization > 0.0:
            scales = {"velocity": 100.0, "alpha": 1000.0, "dist_in": 10.0, "dist_out": 10.0, "angle_abs": 5.0}
            for name in vary:
                delta = (candidate_values[name] - initial_values[name]) / scales[name]
                reg += delta * delta
        return pose_cost + (args.regularization * reg)

    best_cost = cost_for(values)
    for _ in range(args.max_iters):
        improved = False
        for name in sorted(vary):
            for direction in (1.0, -1.0):
                candidate = dict(values)
                candidate[name] = clamp(name, candidate[name] + direction * steps[name])
                candidate_cost = cost_for(candidate)
                if candidate_cost + 1e-9 < best_cost:
                    values = candidate
                    best_cost = candidate_cost
                    improved = True
        if not improved:
            for name in vary:
                steps[name] *= 0.5
            if all(steps[name] <= args.min_step for name in vary):
                break

    initial_sim = simulate_turn(turn, constants, args.entry_speed, args.out_speed)
    fitted_sim = simulate_turn(_turn_with_values(turn, values), constants, args.entry_speed, args.out_speed)
    return initial_sim, fitted_sim, values


def print_fit_summary(initial: SimResult, fitted: SimResult, target_pose: Pose, values: dict[str, float]) -> None:
    initial_err = pose_error(initial.final_pose, target_pose)
    fitted_err = pose_error(fitted.final_pose, target_pose)
    print("[TURN-FIT] initial")
    print_sim_summary(initial)
    print(
        "[TURN-FIT] initial_error "
        f"dx={initial_err['dx_mm']:.3f}mm dy={initial_err['dy_mm']:.3f}mm dtheta={initial_err['dtheta_deg']:.3f}deg"
    )
    print("[TURN-FIT] fitted")
    print_sim_summary(fitted)
    print(
        "[TURN-FIT] fitted_error "
        f"dx={fitted_err['dx_mm']:.3f}mm dy={fitted_err['dy_mm']:.3f}mm dtheta={fitted_err['dtheta_deg']:.3f}deg"
    )
    if fitted.turn.source_fields:
        print("[TURN-FIT] suggested_assignments")
        fields = fitted.turn.source_fields
        if "velocity" in fields:
            print(f"  .{fields['velocity']} = {fitted.turn.velocity_mm_s:.3f}f,")
        if "alpha" in fields:
            print(f"  .{fields['alpha']} = {fitted.turn.alpha_deg_s2:.3f}f,")
        if "angle" in fields:
            print(f"  .{fields['angle']} = {abs(fitted.turn.signed_angle_deg):.3f}f,")
        if "dist_in" in fields:
            print(f"  .{fields['dist_in']} = {fitted.turn.dist_in_mm:.3f}f,")
        if "dist_out" in fields:
            print(f"  .{fields['dist_out']} = {fitted.turn.dist_out_mm:.3f}f,")


def add_common_paths(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--params-h", default=None, help="params.h path for TURN_OMEGA_PROFILE_ROUNDING_SCALE")
    parser.add_argument("--f413-path-h", default=None, help="f413_path_run.h path for NIGHTFALL_F413_PATH_OMEGA_CAP")


def add_turn_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--runner", choices=("shortest", "search", "manual"), default="shortest")
    parser.add_argument("--shortest-params", default=None, help="shortest_run_params_split.c path")
    parser.add_argument("--search-params", default=None, help="search_run_params_split.c path")
    parser.add_argument("--mode", type=int, default=2, help="shortest mode number")
    parser.add_argument("--code", type=int, default=501, help="shortest path turn code")
    parser.add_argument("--search-index", type=int, default=0, help="searchRunParams index")
    parser.add_argument("--side", choices=("right", "left"), default="right", help="search turn side")
    parser.add_argument("--velocity", type=float, default=None, help="override turn velocity [mm/s]")
    parser.add_argument("--alpha", type=float, default=None, help="override angular acceleration [deg/s^2]")
    parser.add_argument("--angle", type=float, default=None, help="override signed turn angle [deg]")
    parser.add_argument("--dist-in", type=float, default=None, help="override front/in offset [mm]")
    parser.add_argument("--dist-out", type=float, default=None, help="override rear/out offset [mm]")
    parser.add_argument("--entry-speed", type=float, default=None, help="speed before in-offset [mm/s]")
    parser.add_argument("--out-speed", type=float, default=None, help="speed after out-offset [mm/s]")


def add_json_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--json", action="store_true", help="print machine-readable JSON")


def command_simulate(args: argparse.Namespace) -> int:
    constants = load_constants(args)
    turn = resolve_turn(args)
    sim = simulate_turn(turn, constants, args.entry_speed, args.out_speed)
    if args.export_csv:
        export_samples_csv(Path(args.export_csv), sim.samples)
    if args.json:
        print(json.dumps(sim_to_dict(sim, include_samples=args.include_samples), indent=2, sort_keys=True))
    else:
        print_sim_summary(sim)
        if args.export_csv:
            print(f"[TURN-SIM] exported={args.export_csv}")
    return 0


def command_replay(args: argparse.Namespace) -> int:
    csv_path = resolve_csv_path(args.csv_path)
    replay = replay_trace(csv_path, args.phase, args.start_ms, args.end_ms, args.omega_threshold, args.integration)
    sim: Optional[SimResult] = None
    if args.compare_sim:
        constants = load_constants(args)
        turn = resolve_turn(args)
        entry_speed = args.entry_speed
        if entry_speed is None:
            _, records = load_trace_csv(csv_path)
            _, selected = select_records(records, args.phase, args.start_ms, args.end_ms, args.omega_threshold)
            if selected:
                entry_speed = selected[0].target_velocity_mm_s
        sim = simulate_turn(turn, constants, entry_speed, args.out_speed)
    if args.json:
        payload = replay_to_dict(replay)
        if sim is not None:
            payload["simulation"] = sim_to_dict(sim)
            payload["target_minus_sim"] = pose_error(replay.target_pose, sim.final_pose)
            payload["real_minus_sim"] = pose_error(replay.real_pose, sim.final_pose)
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print_replay_summary(replay, sim)
    return 0


def command_fit(args: argparse.Namespace) -> int:
    constants = load_constants(args)
    turn = resolve_turn(args)
    initial, fitted, values = fit_turn(args, turn, constants)
    target_pose = Pose(args.target_x, args.target_y, math.radians(args.target_theta))
    if args.json:
        print(
            json.dumps(
                {
                    "initial": sim_to_dict(initial),
                    "fitted": sim_to_dict(fitted),
                    "target_pose": pose_to_dict(target_pose),
                    "initial_error": pose_error(initial.final_pose, target_pose),
                    "fitted_error": pose_error(fitted.final_pose, target_pose),
                    "values": values,
                },
                indent=2,
                sort_keys=True,
            )
        )
    else:
        print_fit_summary(initial, fitted, target_pose, values)
    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="F413 turn trajectory tuning tool")
    subparsers = parser.add_subparsers(dest="command", required=True)

    simulate = subparsers.add_parser("simulate", help="simulate one turn from params or manual values")
    add_common_paths(simulate)
    add_turn_args(simulate)
    add_json_arg(simulate)
    simulate.add_argument("--export-csv", default=None, help="write simulated trajectory samples to CSV")
    simulate.add_argument("--include-samples", action="store_true", help="include samples in JSON output")
    simulate.set_defaults(func=command_simulate)

    replay = subparsers.add_parser("replay", help="integrate a trace CSV and compare target/real pose")
    add_common_paths(replay)
    add_turn_args(replay)
    add_json_arg(replay)
    replay.add_argument("csv_path", help="trace CSV path or directory")
    replay.add_argument("--phase", choices=("auto", "all", "motor-rev", "motor-forward", "omega"), default="auto")
    replay.add_argument("--start-ms", type=int, default=None, help="relative start trim after selected phase start")
    replay.add_argument("--end-ms", type=int, default=None, help="relative end trim after selected phase start")
    replay.add_argument("--omega-threshold", type=float, default=DEFAULT_OMEGA_THRESHOLD_DPS, help="omega active threshold [deg/s]")
    replay.add_argument("--integration", choices=("previous", "trapezoid"), default="previous")
    replay.add_argument("--compare-sim", action="store_true", help="also simulate selected turn and print pose differences")
    replay.set_defaults(func=command_replay)

    fit = subparsers.add_parser("fit", help="fit turn parameters to a target final pose")
    add_common_paths(fit)
    add_turn_args(fit)
    add_json_arg(fit)
    fit.add_argument("--target-x", type=float, required=True, help="target final x [mm]")
    fit.add_argument("--target-y", type=float, required=True, help="target final y [mm]")
    fit.add_argument("--target-theta", type=float, required=True, help="target final theta [deg]")
    fit.add_argument("--vary", default="velocity,alpha,dist_in,dist_out,angle", help="comma-separated variables: velocity,alpha,dist_in,dist_out,angle")
    fit.add_argument("--theta-weight-mm-per-deg", type=float, default=2.0)
    fit.add_argument("--regularization", type=float, default=0.1)
    fit.add_argument("--max-iters", type=int, default=240)
    fit.add_argument("--step-velocity", type=float, default=50.0)
    fit.add_argument("--step-alpha", type=float, default=500.0)
    fit.add_argument("--step-dist", type=float, default=2.0)
    fit.add_argument("--step-angle", type=float, default=1.0)
    fit.add_argument("--min-step", type=float, default=0.01)
    fit.add_argument("--min-offset", type=float, default=-30.0)
    fit.add_argument("--max-offset", type=float, default=120.0)
    fit.add_argument("--max-velocity", type=float, default=5000.0)
    fit.add_argument("--max-alpha", type=float, default=200000.0)
    fit.set_defaults(func=command_fit)
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    try:
        return args.func(args)
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(f"[TURN-TUNE][ERROR] {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
