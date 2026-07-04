#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path


SEARCH_EVENT_MARKER = 0x5345
EVENT_NAMES = {
    0xE0: "session_start",
    0xE1: "phase",
    0xE2: "decision",
    0xE3: "motion_end",
    0xE4: "session_end",
    0xE5: "route_fail",
}
TARGET_NAMES = {
    0: "goal",
    1: "full",
    2: "start",
}
REL_NAMES = {
    0: "forward",
    1: "right",
    2: "back",
    3: "left",
}
DIR_NAMES = {
    0: "N",
    1: "E",
    2: "S",
    3: "W",
}
MOTION_NAMES = {
    1: "entry_section",
    2: "forward_section",
    3: "smooth_turn_r90",
    4: "back_turn_180",
    5: "smooth_turn_l90",
    6: "final_stop",
}
PHASE_STATUS_NAMES = {
    1: "start",
    2: "reached",
    3: "full_complete",
    4: "full_unreachable",
}
ROUTE_FAIL_REASON_NAMES = {
    1: "max_actions",
    2: "no_current_step",
    3: "no_next_rel",
}


def _is_number(text: str) -> bool:
    try:
        float(text)
    except ValueError:
        return False
    return True


def _read_trace_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    columns: list[str] | None = None
    rows: list[dict[str, str]] = []

    with path.open("r", encoding="ascii", errors="ignore", newline="") as f:
        reader = csv.reader(f)
        for parts in reader:
            if not parts:
                continue
            first = parts[0].strip()
            if first.startswith("#mm_columns="):
                columns = [p.strip() for p in ",".join(parts)[len("#mm_columns=") :].split(",")]
                continue
            if first.startswith("#"):
                continue
            parts = [p.strip() for p in parts]
            if columns is None:
                if parts and not _is_number(parts[0]):
                    columns = parts
                continue
            if len(parts) != len(columns):
                continue
            if not all(_is_number(p) for p in parts):
                continue
            rows.append(dict(zip(columns, parts)))

    if columns is None:
        raise ValueError(f"CSV header not found: {path}")
    return columns, rows


def _i(row: dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(float(row.get(key, "")))
    except (TypeError, ValueError):
        return default


def _f(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return default


def _flag_text(flags: int) -> str:
    names: list[str] = []
    if flags & 0x01:
        names.append("known")
    if flags & 0x02:
        names.append("acceled_in")
    if flags & 0x04:
        names.append("acceled_out")
    if flags & 0x08:
        names.append("next_turn90")
    return "|".join(names) if names else "-"


def _event_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    return [
        row
        for row in rows
        if _i(row, "event_marker") == SEARCH_EVENT_MARKER and _i(row, "event_type") in EVENT_NAMES
    ]


def _detail(row: dict[str, str]) -> str:
    event_type = _i(row, "event_type")
    if event_type == 0xE1:
        return f"phase_status={PHASE_STATUS_NAMES.get(_i(row, 'event_smap_step'), _i(row, 'event_smap_step'))}"
    if event_type == 0xE2:
        return (
            f"smap={_i(row, 'event_smap_step')} wall=0x{_i(row, 'event_wall_info'):04X} "
            f"cell=0x{_i(row, 'event_map_cell'):04X} next_after={REL_NAMES.get(_i(row, 'event_next_after_forward'), '-')}"
        )
    if event_type == 0xE3:
        motion = MOTION_NAMES.get(_i(row, "event_motion_kind"), str(_i(row, "event_motion_kind")))
        return (
            f"{motion} status={_i(row, 'event_motion_status')} "
            f"dt={_i(row, 'event_motion_duration_ms')}ms arg0={_i(row, 'event_arg0_x1000') / 1000.0:.3f} "
            f"arg1={_i(row, 'event_arg1_x1000') / 1000.0:.3f}"
        )
    if event_type == 0xE4:
        return f"completed={_i(row, 'event_completed')} route_failed={_i(row, 'event_route_failed')}"
    if event_type == 0xE5:
        reason = ROUTE_FAIL_REASON_NAMES.get(_i(row, "event_route_reason"), _i(row, "event_route_reason"))
        return (
            f"reason={reason} smap={_i(row, 'event_smap_step')} wall=0x{_i(row, 'event_wall_info'):04X} "
            f"cell=0x{_i(row, 'event_map_cell'):04X}"
        )
    return f"param={_i(row, 'event_param_index')}"


def print_summary(path: Path, limit: int) -> None:
    _columns, rows = _read_trace_csv(path)
    events = _event_rows(rows)
    if limit > 0:
        events = events[-limit:]

    print("seq,time_ms,event,action,pos,dir,next,target,flags,target_v,real_v,target_angle,angle,detail")
    if not events:
        return
    first_ts = _i(events[0], "timestamp_ms")
    for row in events:
        time_ms = _i(row, "timestamp_ms") - first_ts
        event_type = _i(row, "event_type")
        target = TARGET_NAMES.get(_i(row, "event_target"), str(_i(row, "event_target")))
        next_rel = REL_NAMES.get(_i(row, "event_next_rel"), str(_i(row, "event_next_rel")))
        direction = DIR_NAMES.get(_i(row, "event_dir"), str(_i(row, "event_dir")))
        pos = f"({_i(row, 'event_x')},{_i(row, 'event_y')})"
        print(
            f"{_i(row, 'seq')},{time_ms},{EVENT_NAMES[event_type]},{_i(row, 'event_action')},"
            f"{pos},{direction},{next_rel},{target},{_flag_text(_i(row, 'event_flags'))},"
            f"{_f(row, 'target_velocity_mm_s'):.0f},{_f(row, 'real_velocity_mm_s'):.0f},"
            f"{_f(row, 'target_angle_mdeg') / 1000.0:.3f},{_f(row, 'angle_mdeg') / 1000.0:.3f},"
            f"{_detail(row)}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize F413 search event trace CSV")
    parser.add_argument("csv", type=Path)
    parser.add_argument("--limit", type=int, default=0, help="show only the latest N event rows")
    args = parser.parse_args()

    print_summary(args.csv.expanduser(), args.limit)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
