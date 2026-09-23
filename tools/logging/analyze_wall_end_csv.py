#!/usr/bin/env python3
"""Summarize F413 wall-end events and turn offset geometry from a trace CSV."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


FLAG_MOTOR_REV = 0x0010
WALL_END_RIGHT = 0x0040
WALL_END_LEFT = 0x0080
WALL_TRACE_DERIV_V2 = 0x0400


@dataclass
class TraceData:
    path: Path
    meta: dict[str, str]
    rows: list[dict[str, str]]


@dataclass
class TurnMetric:
    start_index: int
    end_index: int
    angle_deg: float
    forward_mm: float
    lateral_mm: float
    duration_ms: int
    capture_complete: bool
    boundary_gap_ms: int


def _select_path(value: str) -> Path:
    path = Path(value)
    if path.is_dir():
        candidates = list(path.glob("trace_bin_*.csv"))
        if not candidates:
            candidates = [
                item
                for item in path.glob("*.csv")
                if not item.name.endswith(".plotjuggler.csv")
            ]
        candidates.sort(key=lambda item: item.stat().st_mtime)
        if not candidates:
            raise ValueError(f"no CSV files in {path}")
        return candidates[-1]
    if not path.is_file():
        raise ValueError(f"CSV file not found: {path}")
    return path


def _load_trace(path: Path) -> TraceData:
    meta: dict[str, str] = {}
    columns: list[str] = []
    rows: list[dict[str, str]] = []

    with path.open("r", encoding="ascii", errors="ignore", newline="") as stream:
        for raw in stream:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                payload = line[1:]
                if "=" in payload:
                    key, value = payload.split("=", 1)
                    meta[key.strip()] = value.strip()
                if line.startswith("#mm_columns="):
                    columns = next(csv.reader([line.removeprefix("#mm_columns=")]))
                    columns = [column.strip() for column in columns]
                continue
            if not columns:
                continue
            values = next(csv.reader([line]))
            if len(values) == len(columns):
                rows.append(dict(zip(columns, values)))

    if not rows:
        raise ValueError(f"no trace records in {path}")
    return TraceData(path=path, meta=meta, rows=rows)


def _number(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def _integer(row: dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(float(row.get(key, default)))
    except (TypeError, ValueError):
        return default


def _reserved_layout(trace: TraceData) -> dict[str, str]:
    names = [
        name.strip()
        for name in trace.meta.get("wall_trace_reserved_i32", "").split(",")
        if name.strip()
    ]
    return {
        name: f"reserved_i32_{index}"
        for index, name in enumerate(names[:4])
    }


def _turn_segments(
    rows: list[dict[str, str]],
) -> list[tuple[int, int, bool, int]]:
    segments: list[tuple[int, int, bool, int]] = []
    start: int | None = None

    for index, row in enumerate(rows):
        active = bool(_integer(row, "flags") & FLAG_MOTOR_REV)
        if active and start is None:
            start = index
        elif not active and start is not None:
            end = index - 1
            boundary_gap_ms = (
                _integer(rows[index], "timestamp_ms")
                - _integer(rows[end], "timestamp_ms")
            )
            segments.append((start, end, boundary_gap_ms <= 10, boundary_gap_ms))
            start = None
    if start is not None:
        segments.append((start, len(rows) - 1, False, 0))
    return segments


def _integrate_turn(
    rows: list[dict[str, str]],
    start: int,
    end: int,
    capture_complete: bool,
    boundary_gap_ms: int,
) -> TurnMetric:
    x_mm = 0.0
    y_mm = 0.0
    heading_rad = 0.0
    first = rows[start]
    previous_time_ms = _integer(first, "timestamp_ms")
    previous_velocity = _number(first, "real_velocity_mm_s")
    previous_omega = _number(first, "real_omega_mdps") * 0.001

    for row in rows[start + 1 : end + 1]:
        timestamp_ms = _integer(row, "timestamp_ms")
        dt_ms = timestamp_ms - previous_time_ms
        if dt_ms < 0:
            dt_ms = 0
        if dt_ms > 5:
            dt_ms = 5
        dt_s = dt_ms * 0.001
        velocity = _number(row, "real_velocity_mm_s")
        omega = _number(row, "real_omega_mdps") * 0.001
        mean_velocity = 0.5 * (previous_velocity + velocity)
        mean_omega = 0.5 * (previous_omega + omega)
        mean_heading = heading_rad + math.radians(mean_omega * dt_s * 0.5)

        x_mm += mean_velocity * dt_s * math.cos(mean_heading)
        y_mm += mean_velocity * dt_s * math.sin(mean_heading)
        heading_rad += math.radians(mean_omega * dt_s)
        previous_time_ms = timestamp_ms
        previous_velocity = velocity
        previous_omega = omega

    return TurnMetric(
        start_index=start,
        end_index=end,
        angle_deg=math.degrees(heading_rad),
        forward_mm=x_mm,
        lateral_mm=abs(y_mm),
        duration_ms=(
            _integer(rows[end], "timestamp_ms")
            - _integer(rows[start], "timestamp_ms")
        ),
        capture_complete=capture_complete,
        boundary_gap_ms=boundary_gap_ms,
    )


def _wall_end_events(
    trace: TraceData, turn_starts: list[int]
) -> list[tuple[int, int, int | None]]:
    events: list[tuple[int, int, int | None]] = []
    previous = 0

    for index, row in enumerate(trace.rows):
        detected = _integer(row, "reserved_u16_0") & (
            WALL_END_RIGHT | WALL_END_LEFT
        )
        rising = detected & ~previous
        if rising:
            next_turn = next((start for start in turn_starts if start > index), None)
            events.append((index, rising, next_turn))
        previous = detected
    return events


def _print_events(trace: TraceData, turn_starts: list[int]) -> None:
    layout = _reserved_layout(trace)
    events = _wall_end_events(trace, turn_starts)

    print(f"[WALL-END] events={len(events)}")
    for number, (index, sides, next_turn) in enumerate(events, 1):
        row = trace.rows[index]
        side = (
            "RL"
            if sides == (WALL_END_RIGHT | WALL_END_LEFT)
            else ("R" if sides & WALL_END_RIGHT else "L")
        )
        event_distance = _number(row, "distance_mm")
        deriv_r = _integer(row, layout.get("deriv_r", ""))
        deriv_l = _integer(row, layout.get("deriv_l", ""))
        latched_r = _integer(row, layout.get("detected_deriv_r", ""))
        latched_l = _integer(row, layout.get("detected_deriv_l", ""))
        next_text = "next_turn=n/a"
        if next_turn is not None:
            next_row = trace.rows[next_turn]
            next_distance = _number(next_row, "distance_mm")
            next_text = (
                f"next_turn_d={next_distance:.1f}mm "
                f"event_to_turn={next_distance - event_distance:.1f}mm"
            )
        print(
            f"  #{number:02d} side={side} seq={_integer(row, 'seq')} "
            f"t={_integer(row, 'timestamp_ms')}ms d={event_distance:.1f}mm "
            f"sensor_r/l={_integer(row, 'adc_r')}/{_integer(row, 'adc_l')} "
            f"deriv_r/l={deriv_r}/{deriv_l} "
            f"trigger_r/l={latched_r}/{latched_l} {next_text}"
        )


def _print_turns(
    trace: TraceData,
    target_angle_deg: float,
    angle_tolerance_deg: float,
    target_axis_mm: float,
    current_dist_in_mm: float | None,
    current_dist_out_mm: float | None,
    current_alpha_deg_s2: float | None,
) -> None:
    selected: list[TurnMetric] = []
    is_uturn = abs(target_angle_deg - 180.0) <= angle_tolerance_deg

    print("[TURN-OFFSET] completed turn phases:")
    for number, (
        start,
        end,
        capture_complete,
        boundary_gap_ms,
    ) in enumerate(_turn_segments(trace.rows), 1):
        metric = _integrate_turn(
            trace.rows,
            start,
            end,
            capture_complete,
            boundary_gap_ms,
        )
        complete = metric.capture_complete and (
            abs(abs(metric.angle_deg) - target_angle_deg)
            <= angle_tolerance_deg
        )
        if complete:
            status = "selected"
        elif not metric.capture_complete:
            status = f"ignored-gap({metric.boundary_gap_ms}ms)"
        else:
            status = "ignored-angle"
        print(
            f"  #{number:02d} seq={_integer(trace.rows[start], 'seq')}"
            f"->{_integer(trace.rows[end], 'seq')} "
            f"duration={metric.duration_ms}ms angle={metric.angle_deg:.2f}deg "
            f"arc_forward={metric.forward_mm:.2f}mm "
            f"arc_lateral={metric.lateral_mm:.2f}mm {status}"
        )
        if complete:
            selected.append(metric)

    if not selected:
        print("  recommendation=n/a (no complete target-angle turns)")
        return

    mean_forward = sum(item.forward_mm for item in selected) / len(selected)
    mean_lateral = sum(item.lateral_mm for item in selected) / len(selected)
    print(
        f"  mean_arc={mean_forward:.2f}/{mean_lateral:.2f}mm "
        f"target_axis={target_axis_mm:.2f}mm"
    )

    if is_uturn:
        lateral_error = mean_lateral - target_axis_mm
        print(f"  lateral_error={lateral_error:+.2f}mm")
        if current_dist_in_mm is not None:
            recommended_out = max(0.0, current_dist_in_mm + mean_forward)
            print(
                f"  recommended_dist_out={recommended_out:.2f}mm "
                f"for dist_in={current_dist_in_mm:.2f}mm"
            )
        if (
            current_alpha_deg_s2 is not None
            and current_alpha_deg_s2 > 0.0
            and target_axis_mm > 0.0
        ):
            recommended_alpha = (
                current_alpha_deg_s2
                * (mean_lateral / target_axis_mm) ** 2
            )
            print(
                f"  recommended_alpha={recommended_alpha:.0f}deg/s^2 "
                f"from current_alpha={current_alpha_deg_s2:.0f}deg/s^2"
            )
        if current_dist_in_mm is not None and current_dist_out_mm is not None:
            current_forward = (
                current_dist_in_mm + mean_forward - current_dist_out_mm
            )
            print(
                f"  current_total_forward={current_forward:+.2f}mm "
                f"current_lateral={mean_lateral:.2f}mm "
                f"using dist_in/out={current_dist_in_mm:.2f}/"
                f"{current_dist_out_mm:.2f}mm"
            )
        return

    recommended_in = max(0.0, target_axis_mm - mean_forward)
    recommended_out = max(0.0, target_axis_mm - mean_lateral)
    print(
        f"  recommended_dist_in={recommended_in:.2f}mm "
        f"recommended_dist_out={recommended_out:.2f}mm"
    )
    if current_dist_in_mm is not None and current_dist_out_mm is not None:
        print(
            f"  current_total={mean_forward + current_dist_in_mm:.2f}/"
            f"{mean_lateral + current_dist_out_mm:.2f}mm "
            f"using dist_in/out={current_dist_in_mm:.2f}/"
            f"{current_dist_out_mm:.2f}mm"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Analyze F413 wall-end detection positions and smooth-turn "
            "geometry"
        )
    )
    parser.add_argument(
        "csv_path",
        help="trace CSV path or directory (latest CSV is selected)",
    )
    parser.add_argument("--target-angle", type=float, default=90.0)
    parser.add_argument("--angle-tolerance", type=float, default=5.0)
    parser.add_argument("--target-axis-mm", type=float, default=90.0)
    parser.add_argument("--dist-in", type=float)
    parser.add_argument("--dist-out", type=float)
    parser.add_argument("--alpha", type=float)
    args = parser.parse_args()

    try:
        path = _select_path(args.csv_path)
        trace = _load_trace(path)
    except ValueError as error:
        parser.error(str(error))

    has_deriv_v2 = any(
        _integer(row, "reserved_u16_0") & WALL_TRACE_DERIV_V2
        for row in trace.rows
    )
    layout = trace.meta.get("wall_trace_reserved_i32", "")
    if not layout and has_deriv_v2:
        layout = "deriv_r,deriv_l,detected_deriv_r,detected_deriv_l"
        trace.meta["wall_trace_reserved_i32"] = layout
    print(f"[WALL-END] file={trace.path}")
    print(
        f"[WALL-END] trace_version="
        f"{trace.meta.get('wall_trace_observe', '2' if has_deriv_v2 else 'unknown')} "
        f"layout={layout or 'unknown'}"
    )
    if layout != "deriv_r,deriv_l,detected_deriv_r,detected_deriv_l":
        print(
            "[WALL-END] warning=derivative telemetry is unavailable; "
            "capture with wall_trace_observe=2"
        )

    turn_starts = [start for start, _, _, _ in _turn_segments(trace.rows)]
    _print_events(trace, turn_starts)
    _print_turns(
        trace,
        target_angle_deg=abs(args.target_angle),
        angle_tolerance_deg=args.angle_tolerance,
        target_axis_mm=args.target_axis_mm,
        current_dist_in_mm=args.dist_in,
        current_dist_out_mm=args.dist_out,
        current_alpha_deg_s2=args.alpha,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
