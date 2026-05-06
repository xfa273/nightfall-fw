#!/usr/bin/env python3
import argparse
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

FLAG_SWITCH = 0x0001
FLAG_IDLE = 0x0002
FLAG_MOTOR_FWD = 0x0004
FLAG_MOTOR_COAST = 0x0008
FLAG_MOTOR_REV = 0x0010
FLAG_SMOKE = 0x0020
FLAG_SEARCH_SAFE = 0x0040
FLAG_SHORTEST_SAFE = 0x0080
FLAG_ABORT_SWITCH = 0x0100
FLAG_ABORT_WALL_FAULT = 0x0200
FLAG_ABORT_ENCODER_FAULT = 0x0400
FLAG_ABORT_IMU_FAULT = 0x0800
FLAG_AUTO = 0x8000

DEFAULT_COLUMNS = [
    "timestamp_ms",
    "seq",
    "encoder_l",
    "encoder_r",
    "motor_out_l",
    "motor_out_r",
    "omega_z_mdps",
    "flags",
]

ABORT_FLAG_ORDER = [
    (FLAG_ABORT_SWITCH, "abort_switch"),
    (FLAG_ABORT_WALL_FAULT, "abort_wall_fault"),
    (FLAG_ABORT_ENCODER_FAULT, "abort_encoder_fault"),
    (FLAG_ABORT_IMU_FAULT, "abort_imu_fault"),
]

PHASE_ORDER = [
    (FLAG_MOTOR_FWD, "motor_forward"),
    (FLAG_MOTOR_REV, "motor_reverse"),
    (FLAG_MOTOR_COAST, "motor_coast"),
    (FLAG_IDLE, "idle"),
    (FLAG_SMOKE, "smoke"),
]


@dataclass
class TraceRecord:
    timestamp_ms: int
    seq: int
    encoder_l: int
    encoder_r: int
    motor_out_l: int
    motor_out_r: int
    omega_z_mdps: int
    flags: int


@dataclass
class PhaseSegment:
    phase: str
    start_index: int
    end_index: int


def _looks_like_trace_row(line: str) -> bool:
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 8:
        return False
    if not parts[0].isdigit():
        return False
    try:
        for p in parts[1:]:
            int(p)
    except ValueError:
        return False
    return True


def _has_trace_rows(path: Path) -> bool:
    try:
        with path.open("r", encoding="ascii", errors="ignore") as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("#"):
                    continue
                if _looks_like_trace_row(line):
                    return True
    except OSError:
        return False
    return False


def _csv_matches_expect(path: Path, expect: str) -> bool:
    if expect == "none":
        return _has_trace_rows(path)

    columns = DEFAULT_COLUMNS[:]
    flags_index = 7
    has_idle = False
    has_fwd = False
    has_coast = False
    has_rev = False
    has_search_safe = False
    has_shortest_safe = False

    try:
        with path.open("r", encoding="ascii", errors="ignore") as f:
            for raw in f:
                line = raw.strip()
                if not line:
                    continue

                if line.startswith("#mm_columns="):
                    cols = [c.strip() for c in line[len("#mm_columns=") :].split(",") if c.strip()]
                    if cols:
                        columns = cols
                        try:
                            flags_index = columns.index("flags")
                        except ValueError:
                            flags_index = -1
                    continue

                if line.startswith("#"):
                    continue

                parts = [p.strip() for p in line.split(",")]
                if len(parts) != len(columns):
                    continue
                if flags_index < 0 or flags_index >= len(parts):
                    continue

                try:
                    flags = int(parts[flags_index])
                except ValueError:
                    continue

                has_idle = has_idle or bool(flags & FLAG_IDLE)
                has_fwd = has_fwd or bool(flags & FLAG_MOTOR_FWD)
                has_coast = has_coast or bool(flags & FLAG_MOTOR_COAST)
                has_rev = has_rev or bool(flags & FLAG_MOTOR_REV)
                has_search_safe = has_search_safe or bool(flags & FLAG_SEARCH_SAFE)
                has_shortest_safe = has_shortest_safe or bool(flags & FLAG_SHORTEST_SAFE)
    except OSError:
        return False

    if expect == "x":
        return has_idle
    if expect == "y":
        return has_fwd and has_coast and has_rev
    if expect == "z":
        return has_search_safe and has_fwd and has_coast
    return has_shortest_safe and has_fwd and has_coast and has_rev


def _resolve_csv_path(path_str: str, expect: str) -> Path:
    path = Path(path_str).expanduser()
    if path.is_file():
        return path

    if path.is_dir():
        candidates = sorted(path.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
        if not candidates:
            raise FileNotFoundError(f"CSV file not found in directory: {path}")

        if expect != "none":
            for c in candidates:
                if _csv_matches_expect(c, expect):
                    return c

        for c in candidates:
            if _has_trace_rows(c):
                return c
        return candidates[0]

    raise FileNotFoundError(f"CSV path not found: {path}")


def _parse_int(s: str) -> int:
    return int(s.strip())


def _phase_from_flags(flags: int) -> str:
    for bit, name in PHASE_ORDER:
        if flags & bit:
            return name
    return "none"


def _load_trace_csv(path: Path) -> tuple[dict[str, str], list[str], list[TraceRecord]]:
    fw_meta: dict[str, str] = {}
    columns = DEFAULT_COLUMNS[:]
    records: list[TraceRecord] = []

    with path.open("r", encoding="ascii", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue

            if line.startswith("#fw_"):
                k, _, v = line.partition("=")
                fw_meta[k] = v
                continue

            if line.startswith("#mm_columns="):
                cols = [c.strip() for c in line[len("#mm_columns=") :].split(",") if c.strip()]
                if cols:
                    columns = cols
                continue

            parts = [p.strip() for p in line.split(",")]
            if len(parts) != len(columns):
                continue

            row = dict(zip(columns, parts))
            try:
                record = TraceRecord(
                    timestamp_ms=_parse_int(row["timestamp_ms"]),
                    seq=_parse_int(row["seq"]),
                    encoder_l=_parse_int(row["encoder_l"]),
                    encoder_r=_parse_int(row["encoder_r"]),
                    motor_out_l=_parse_int(row["motor_out_l"]),
                    motor_out_r=_parse_int(row["motor_out_r"]),
                    omega_z_mdps=_parse_int(row["omega_z_mdps"]),
                    flags=_parse_int(row["flags"]),
                )
            except (KeyError, ValueError):
                continue

            records.append(record)

    return fw_meta, columns, records


def _build_segments(records: list[TraceRecord]) -> list[PhaseSegment]:
    if not records:
        return []

    segments: list[PhaseSegment] = []
    current_phase = _phase_from_flags(records[0].flags)
    start = 0

    for i in range(1, len(records)):
        phase = _phase_from_flags(records[i].flags)
        if phase != current_phase:
            segments.append(PhaseSegment(phase=current_phase, start_index=start, end_index=i - 1))
            current_phase = phase
            start = i

    segments.append(PhaseSegment(phase=current_phase, start_index=start, end_index=len(records) - 1))
    return segments


def _phase_counts(records: Iterable[TraceRecord]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for rec in records:
        phase = _phase_from_flags(rec.flags)
        counts[phase] = counts.get(phase, 0) + 1
    return counts


def _abort_flag_counts(records: Iterable[TraceRecord]) -> dict[str, int]:
    counts: dict[str, int] = {}
    for rec in records:
        for bit, name in ABORT_FLAG_ORDER:
            if rec.flags & bit:
                counts[name] = counts.get(name, 0) + 1
    return counts


def _signed_delta_16(start: int, end: int) -> int:
    raw = (end - start) & 0xFFFF
    if raw >= 0x8000:
        raw -= 0x10000
    return raw


def _median_period_ms(records: list[TraceRecord]) -> float:
    if len(records) < 2:
        return 0.0
    diffs = [records[i].timestamp_ms - records[i - 1].timestamp_ms for i in range(1, len(records))]
    return float(statistics.median(diffs))


def _print_summary(csv_path: Path, fw_meta: dict[str, str], records: list[TraceRecord], segments: list[PhaseSegment]) -> None:
    print(f"[TRACE-ANALYZE] file={csv_path}")
    if fw_meta:
        target = fw_meta.get("#fw_target", "")
        machine = fw_meta.get("#fw_machine_unit", "")
        if target:
            print(f"[TRACE-ANALYZE] fw_target={target}")
        if machine:
            print(f"[TRACE-ANALYZE] machine_unit={machine}")

    print(f"[TRACE-ANALYZE] records={len(records)} median_period_ms={_median_period_ms(records):.1f}")

    counts = _phase_counts(records)
    for k in sorted(counts.keys()):
        print(f"[TRACE-ANALYZE] phase_count[{k}]={counts[k]}")

    abort_counts = _abort_flag_counts(records)
    for _, name in ABORT_FLAG_ORDER:
        if abort_counts.get(name, 0) > 0:
            print(f"[TRACE-ANALYZE] abort_count[{name}]={abort_counts[name]}")

    print("[TRACE-ANALYZE] segments:")
    for idx, seg in enumerate(segments):
        start = records[seg.start_index]
        end = records[seg.end_index]
        d_enc_l = _signed_delta_16(start.encoder_l, end.encoder_l)
        d_enc_r = _signed_delta_16(start.encoder_r, end.encoder_r)
        max_motor = max(
            max(abs(r.motor_out_l), abs(r.motor_out_r))
            for r in records[seg.start_index : seg.end_index + 1]
        )
        print(
            "  "
            f"#{idx:02d} phase={seg.phase:<13} count={seg.end_index - seg.start_index + 1:<3d} "
            f"seq={start.seq}->{end.seq} ts={start.timestamp_ms}->{end.timestamp_ms} "
            f"d_enc=({d_enc_l},{d_enc_r}) max_motor={max_motor}"
        )


def _validate_expectation(expect: str, counts: dict[str, int], records: list[TraceRecord]) -> int:
    if expect == "none":
        return 0

    if expect == "x":
        if counts.get("idle", 0) <= 0:
            print("[TRACE-ANALYZE][ERROR] expected idle phase records for x session", flush=True)
            return 1
        return 0

    has_search_safe = any((rec.flags & FLAG_SEARCH_SAFE) != 0 for rec in records)
    has_shortest_safe = any((rec.flags & FLAG_SHORTEST_SAFE) != 0 for rec in records)
    if expect == "z" and not has_search_safe:
        print("[TRACE-ANALYZE][ERROR] expected search-safe flag(bit6) records for z session", flush=True)
        return 1
    if expect == "j" and not has_shortest_safe:
        print("[TRACE-ANALYZE][ERROR] expected shortest-safe flag(bit7) records for j session", flush=True)
        return 1

    required = ["motor_forward", "motor_coast", "motor_reverse"]
    if expect == "z":
        required = ["motor_forward", "motor_coast"]
    missing = [p for p in required if counts.get(p, 0) <= 0]
    if missing:
        print(f"[TRACE-ANALYZE][ERROR] expected {expect} phases missing: {', '.join(missing)}", flush=True)
        return 1
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Analyze Nightfall trace CSV by phase flags")
    ap.add_argument("csv_path", help="CSV file path or directory (latest *.csv is selected)")
    ap.add_argument(
        "--expect",
        choices=["none", "x", "y", "z", "j"],
        default="none",
        help="Validate expected phase set for session type",
    )
    args = ap.parse_args()

    csv_path = _resolve_csv_path(args.csv_path, args.expect)
    fw_meta, _, records = _load_trace_csv(csv_path)
    if not records:
        print("[TRACE-ANALYZE][ERROR] no trace records found")
        return 2

    segments = _build_segments(records)
    _print_summary(csv_path, fw_meta, records, segments)
    return _validate_expectation(args.expect, _phase_counts(records), records)


if __name__ == "__main__":
    raise SystemExit(main())
