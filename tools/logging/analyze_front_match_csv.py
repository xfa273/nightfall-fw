#!/usr/bin/env python3
"""Summarize F413 mode1 case0 sub4 front-wall matching traces."""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
from pathlib import Path


FRONT_MATCH_MARKER = 0xF400
FRONT_MATCH_MARKER_MASK = 0xFF00
PHASE_NAMES = {
    0: "align-yaw",
    1: "settle-yaw",
    2: "align-position",
    3: "settle-final",
    4: "hold",
    5: "backoff-too-close",
    0xFE: "paused-wall-lost",
    0xFF: "paused-too-close",
}


def _int(row: dict[str, str], field: str, default: int = 0) -> int:
    try:
        return int(float(row.get(field, "")))
    except (TypeError, ValueError):
        return default


def _select_csv(path: Path) -> Path:
    if path.is_file():
        return path
    candidates = [
        item
        for item in path.glob("*.csv")
        if not item.name.endswith(".plotjuggler.csv")
    ]
    if not candidates:
        raise FileNotFoundError(f"no CSV files found under {path}")
    return max(candidates, key=lambda item: item.stat().st_mtime)


def _read_samples(path: Path) -> list[dict[str, float | int]]:
    samples: list[dict[str, float | int]] = []
    columns: list[str] | None = None
    data_lines: list[str] = []
    with path.open(newline="", encoding="utf-8-sig") as handle:
        for line in handle:
            if line.startswith("#mm_columns="):
                columns = next(csv.reader([line.removeprefix("#mm_columns=").strip()]))
            elif line.strip() and not line.startswith("#"):
                data_lines.append(line)

    if columns is None:
        reader: object = csv.DictReader(data_lines)
    else:
        rows = csv.reader(data_lines)
        reader = (
            dict(zip(columns, values))
            for values in rows
            if values != columns and len(values) >= len(columns)
        )

    for row in reader:
        if not isinstance(row, dict):
            continue
        marker_phase = _int(row, "reserved_u16_0")
        if (
            _int(row, "op_mode", -1) != 1
            or _int(row, "op_case", -1) != 0
            or _int(row, "op_sub", -1) != 4
            or (marker_phase & FRONT_MATCH_MARKER_MASK) != FRONT_MATCH_MARKER
        ):
            continue
        samples.append(
            {
                "timestamp_ms": _int(row, "timestamp_ms"),
                "phase": marker_phase & 0xFF,
                "state_elapsed_ms": _int(row, "reserved_u16_1"),
                "fr_mm": _int(row, "reserved_i32_0") / 1000.0,
                "fl_mm": _int(row, "reserved_i32_1") / 1000.0,
                "position_error_mm": _int(row, "reserved_i32_2") / 1000.0,
                "yaw_error_mm": _int(row, "reserved_i32_3") / 1000.0,
                "target_velocity_mm_s": _int(row, "target_velocity_mm_s"),
                "real_velocity_mm_s": _int(row, "real_velocity_mm_s"),
                "target_omega_dps": _int(row, "target_omega_mdps") / 1000.0,
                "real_omega_dps": _int(row, "real_omega_mdps") / 1000.0,
                "motor_out_l": _int(row, "motor_out_l"),
                "motor_out_r": _int(row, "motor_out_r"),
                "adc_fr": _int(row, "adc_fr"),
                "adc_fl": _int(row, "adc_fl"),
            }
        )
    return samples


def _segments(samples: list[dict[str, float | int]]) -> list[tuple[int, int]]:
    segments: list[tuple[int, int]] = []
    start = 0
    for index in range(1, len(samples)):
        if samples[index]["phase"] != samples[index - 1]["phase"]:
            segments.append((start, index - 1))
            start = index
    segments.append((start, len(samples) - 1))
    return segments


def _fmt_sample(sample: dict[str, float | int]) -> str:
    return (
        f"FR={sample['fr_mm']:.2f} FL={sample['fl_mm']:.2f} "
        f"pos={sample['position_error_mm']:+.2f} yaw={sample['yaw_error_mm']:+.2f}"
    )


def analyze(path: Path) -> int:
    csv_path = _select_csv(path)
    samples = _read_samples(csv_path)
    if not samples:
        print(f"ERROR: no mode1 case0 sub4 front-match telemetry in {csv_path}", file=sys.stderr)
        return 2

    first_time = int(samples[0]["timestamp_ms"])
    periods = [
        int(samples[index]["timestamp_ms"]) - int(samples[index - 1]["timestamp_ms"])
        for index in range(1, len(samples))
        if int(samples[index]["timestamp_ms"]) > int(samples[index - 1]["timestamp_ms"])
    ]
    duration_ms = int(samples[-1]["timestamp_ms"]) - first_time
    period_text = f"{statistics.median(periods):.1f}" if periods else "n/a"

    print(f"front-match trace: {csv_path}")
    print(
        f"records={len(samples)} duration={duration_ms / 1000.0:.3f}s "
        f"median_period={period_text}ms"
    )
    print(f"initial: {_fmt_sample(samples[0])}")
    print(f"final:   {_fmt_sample(samples[-1])}")
    print("segments:")

    segments = _segments(samples)
    for start, end in segments:
        chunk = samples[start : end + 1]
        phase = int(chunk[0]["phase"])
        start_ms = int(chunk[0]["timestamp_ms"]) - first_time
        end_ms = int(chunk[-1]["timestamp_ms"]) - first_time
        max_v = max(abs(float(item["target_velocity_mm_s"])) for item in chunk)
        max_w = max(abs(float(item["target_omega_dps"])) for item in chunk)
        max_real_v = max(abs(float(item["real_velocity_mm_s"])) for item in chunk)
        max_real_w = max(abs(float(item["real_omega_dps"])) for item in chunk)
        print(
            f"  {start_ms / 1000.0:7.3f}-{end_ms / 1000.0:7.3f}s "
            f"{PHASE_NAMES.get(phase, f'phase-{phase}'):>20} n={len(chunk):4d} "
            f"{_fmt_sample(chunk[0])} -> {_fmt_sample(chunk[-1])} "
            f"cmd|max|=({max_v:.0f}mm/s,{max_w:.1f}deg/s) "
            f"real|max|=({max_real_v:.0f}mm/s,{max_real_w:.1f}deg/s)"
        )

    transitions = [
        (int(samples[index - 1]["phase"]), int(samples[index]["phase"]))
        for index in range(1, len(samples))
        if samples[index]["phase"] != samples[index - 1]["phase"]
    ]
    hold_entries = int(samples[0]["phase"] == 4) + sum(
        next_phase == 4 for _, next_phase in transitions
    )
    reacquires = sum(prev_phase == 4 and next_phase in (0, 2) for prev_phase, next_phase in transitions)
    backoff_entries = int(samples[0]["phase"] == 5) + sum(
        next_phase == 5 for _, next_phase in transitions
    )
    pause_entries = int(samples[0]["phase"] in (0xFE, 0xFF)) + sum(
        next_phase in (0xFE, 0xFF) for _, next_phase in transitions
    )
    hold_samples = [sample for sample in samples if sample["phase"] == 4]
    hold_commanded = sum(
        abs(float(sample["target_velocity_mm_s"])) > 1.0
        or abs(float(sample["target_omega_dps"])) > 1.0
        for sample in hold_samples
    )
    print(
        f"events: hold_entries={hold_entries} reacquires={reacquires} "
        f"backoff_entries={backoff_entries} "
        f"pause_entries={pause_entries} hold_commanded_samples={hold_commanded}"
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "path",
        nargs="?",
        type=Path,
        default=Path(__file__).resolve().parent / "logs",
        help="trace CSV or directory containing trace CSV files",
    )
    args = parser.parse_args()
    try:
        return analyze(args.path)
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
