#!/usr/bin/env python3
import argparse
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

FLAG_MOTOR_REV = 0x0010
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


@dataclass
class TurnRecord:
    timestamp_ms: int
    motor_out_l: int
    motor_out_r: int
    omega_z_mdps: int
    flags: int


def _parse_int(value: str) -> int:
    return int(value.strip())


def _resolve_csv_path(path_str: str) -> Path:
    path = Path(path_str).expanduser()
    if path.is_file():
        return path
    if path.is_dir():
        candidates = sorted(path.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
        if not candidates:
            raise FileNotFoundError(f"CSV file not found in directory: {path}")
        return candidates[0]
    raise FileNotFoundError(f"CSV path not found: {path}")


def _load_csv(path: Path) -> tuple[dict[str, str], list[TurnRecord]]:
    meta: dict[str, str] = {}
    columns = DEFAULT_COLUMNS[:]
    records: list[TurnRecord] = []

    with path.open("r", encoding="ascii", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                key, sep, value = line.partition("=")
                if sep:
                    meta[key] = value
                if line.startswith("#mm_columns="):
                    parsed = [c.strip() for c in value.split(",") if c.strip()]
                    if parsed:
                        columns = parsed
                continue

            parts = [p.strip() for p in line.split(",")]
            if len(parts) != len(columns):
                continue
            row = dict(zip(columns, parts))
            try:
                records.append(
                    TurnRecord(
                        timestamp_ms=_parse_int(row["timestamp_ms"]),
                        motor_out_l=_parse_int(row["motor_out_l"]),
                        motor_out_r=_parse_int(row["motor_out_r"]),
                        omega_z_mdps=_parse_int(row["omega_z_mdps"]),
                        flags=_parse_int(row["flags"]),
                    )
                )
            except (KeyError, ValueError):
                continue
    return meta, records


def _select_turn_records(records: list[TurnRecord], use_all: bool) -> list[TurnRecord]:
    if use_all:
        return records
    selected = [r for r in records if r.flags & FLAG_MOTOR_REV]
    return selected if selected else records


def _median_period_ms(records: list[TurnRecord]) -> float:
    if len(records) < 2:
        return 0.0
    return float(statistics.median(records[i].timestamp_ms - records[i - 1].timestamp_ms for i in range(1, len(records))))


def _integrated_angle_series(records: list[TurnRecord]) -> list[float]:
    if not records:
        return []
    angles = [0.0]
    angle = 0.0
    for i in range(1, len(records)):
        dt_s = max(0.0, (records[i].timestamp_ms - records[i - 1].timestamp_ms) / 1000.0)
        omega_prev = records[i - 1].omega_z_mdps / 1000.0
        omega_now = records[i].omega_z_mdps / 1000.0
        angle += 0.5 * (omega_prev + omega_now) * dt_s
        angles.append(angle)
    return angles


def _settle_time_ms(records: list[TurnRecord], angles: list[float], target: float, tolerance: float) -> Optional[int]:
    if not records or not angles:
        return None
    for i, angle in enumerate(angles):
        if all(abs(a - target) <= tolerance for a in angles[i:]):
            return records[i].timestamp_ms - records[0].timestamp_ms
    return None


def _infer_target(meta: dict[str, str]) -> Optional[float]:
    test_id = meta.get("#last_test_id", "").strip()
    if test_id == "3":
        return -90.0
    if test_id == "4":
        return 90.0
    return None


def _fmt_optional(value: object, unit: str = "") -> str:
    if value is None:
        return "n/a"
    if isinstance(value, int):
        return f"{value}{unit}"
    if math.isfinite(value):
        return f"{value:.2f}{unit}"
    return "n/a"


def _print_summary(path: Path, meta: dict[str, str], records: list[TurnRecord], target: Optional[float], tolerance: float) -> int:
    if not records:
        print("[TURN-ANALYZE][ERROR] no records found")
        return 2

    angles = _integrated_angle_series(records)
    duration_ms = records[-1].timestamp_ms - records[0].timestamp_ms if len(records) >= 2 else 0
    final_angle = angles[-1] if angles else 0.0
    max_omega = max(abs(r.omega_z_mdps) / 1000.0 for r in records)
    max_motor = max(max(abs(r.motor_out_l), abs(r.motor_out_r)) for r in records)
    mean_motor = statistics.fmean(max(abs(r.motor_out_l), abs(r.motor_out_r)) for r in records)
    peak_positive = max(angles) if angles else 0.0
    peak_negative = min(angles) if angles else 0.0

    print(f"[TURN-ANALYZE] file={path}")
    for key in ("#fw_target", "#fw_machine_unit", "#last_test_id", "#last_test_status", "#last_test_angle_deg"):
        if key in meta:
            print(f"[TURN-ANALYZE] {key[1:]}={meta[key]}")
    print(f"[TURN-ANALYZE] records={len(records)} median_period_ms={_median_period_ms(records):.1f}")
    print(f"[TURN-ANALYZE] duration_ms={duration_ms}")
    print(f"[TURN-ANALYZE] final_angle_est_deg={final_angle:.2f}")
    print(f"[TURN-ANALYZE] peak_angle_pos_deg={peak_positive:.2f}")
    print(f"[TURN-ANALYZE] peak_angle_neg_deg={peak_negative:.2f}")
    print(f"[TURN-ANALYZE] max_omega_abs_dps={max_omega:.2f}")
    print(f"[TURN-ANALYZE] max_motor_abs={max_motor}")
    print(f"[TURN-ANALYZE] mean_motor_abs={mean_motor:.1f}")

    if target is not None:
        final_error = final_angle - target
        if target >= 0.0:
            overshoot = max(0.0, peak_positive - target)
        else:
            overshoot = max(0.0, target - peak_negative)
        settle_ms = _settle_time_ms(records, angles, target, tolerance)
        print(f"[TURN-ANALYZE] target_angle_deg={target:.2f}")
        print(f"[TURN-ANALYZE] final_error_deg={final_error:.2f}")
        print(f"[TURN-ANALYZE] overshoot_deg={overshoot:.2f}")
        print(f"[TURN-ANALYZE] settle_time_ms={_fmt_optional(settle_ms)}")
        print(f"[TURN-ANALYZE] tolerance_deg={tolerance:.2f}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze F413 turn trace CSV for tuning metrics")
    parser.add_argument("csv_path", help="CSV file path or directory (latest *.csv is selected)")
    parser.add_argument("--target-angle", type=float, default=None, help="target angle in degrees")
    parser.add_argument("--tolerance", type=float, default=5.0, help="settling tolerance in degrees")
    parser.add_argument("--all-records", action="store_true", help="use all records instead of motor_reverse phase")
    args = parser.parse_args()

    csv_path = _resolve_csv_path(args.csv_path)
    meta, records = _load_csv(csv_path)
    selected = _select_turn_records(records, args.all_records)
    target = args.target_angle if args.target_angle is not None else _infer_target(meta)
    return _print_summary(csv_path, meta, selected, target, args.tolerance)


if __name__ == "__main__":
    raise SystemExit(main())
