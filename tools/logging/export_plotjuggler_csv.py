#!/usr/bin/env python3
import argparse
import csv
import sys
from pathlib import Path
from typing import Optional


FLAG_IDLE = 0x0002
FLAG_MOTOR_FWD = 0x0004
FLAG_MOTOR_COAST = 0x0008
FLAG_MOTOR_REV = 0x0010
FLAG_ABORT_SWITCH = 0x0100
FLAG_ABORT_WALL_FAULT = 0x0200
FLAG_ABORT_ENCODER_FAULT = 0x0400
FLAG_ABORT_IMU_FAULT = 0x0800
FLAG_ANGLE_TARGET = 0x2000
FLAG_AUTO = 0x8000


def _is_number(text: str) -> bool:
    try:
        float(text)
    except ValueError:
        return False
    return True


def _has_csv_rows(path: Path) -> bool:
    try:
        with path.open("r", encoding="ascii", errors="ignore", newline="") as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("#"):
                    continue
                parts = [p.strip() for p in line.split(",")]
                if len(parts) >= 2 and _is_number(parts[0]) and _is_number(parts[1]):
                    return True
    except OSError:
        return False
    return False


def _select_input(path: Path) -> Path:
    if path.is_file():
        return path
    if not path.is_dir():
        raise FileNotFoundError(path)
    candidates = [p for p in path.glob("*.csv") if not p.name.endswith(".plotjuggler.csv") and _has_csv_rows(p)]
    if not candidates:
        raise FileNotFoundError(f"CSV not found in {path}")
    return max(candidates, key=lambda p: p.stat().st_mtime)


def _parse_meta_line(line: str, meta: dict[str, str]) -> Optional[list[str]]:
    if line.startswith("#mm_columns="):
        return [c.strip() for c in line[len("#mm_columns=") :].split(",")]
    if line.startswith("#") and "=" in line:
        key, value = line[1:].split("=", 1)
        meta[key.strip()] = value.strip()
    return None


def _load_nightfall_csv(path: Path) -> tuple[list[str], list[dict[str, str]], dict[str, str]]:
    columns: Optional[list[str]] = None
    rows: list[dict[str, str]] = []
    meta: dict[str, str] = {}

    with path.open("r", encoding="ascii", errors="ignore", newline="") as f:
        reader = csv.reader(f)
        for raw_parts in reader:
            if not raw_parts:
                continue
            first = raw_parts[0].strip()
            line = ",".join(raw_parts).strip()
            if not line:
                continue
            if first.startswith("#"):
                parsed_columns = _parse_meta_line(line, meta)
                if parsed_columns:
                    columns = parsed_columns
                continue
            parts = [p.strip() for p in raw_parts]
            if columns is None:
                if parts and not _is_number(parts[0]):
                    columns = parts
                    continue
                raise ValueError(f"CSV header not found: {path}")
            if parts == columns:
                continue
            if len(parts) != len(columns):
                continue
            if not all(_is_number(p) for p in parts):
                continue
            rows.append(dict(zip(columns, parts)))

    if columns is None:
        raise ValueError(f"CSV header not found: {path}")
    if not rows:
        raise ValueError(f"CSV data rows not found: {path}")
    return columns, rows, meta


def _num(row: dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key)
    if value is None or value == "":
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _int(row: dict[str, str], key: str, default: int = 0) -> int:
    return int(_num(row, key, float(default)))


def _fmt(value: float) -> str:
    if value == int(value):
        return str(int(value))
    return f"{value:.9g}"


def _infer_tune_axis(rows: list[dict[str, str]], meta: dict[str, str]) -> Optional[str]:
    axis = meta.get("tune_axis")
    if axis:
        return axis
    for row in rows:
        if "reserved_i32_0" not in row or "reserved_i32_1" not in row:
            continue
        if _int(row, "reserved_i32_0") == 0:
            continue
        axis_code = _int(row, "reserved_i32_1", -1)
        if axis_code == 0:
            return "velocity"
        if axis_code == 1:
            return "omega"
        if axis_code == 2:
            return "distance"
        if axis_code == 3:
            return "angle"
    return None


def _reserved_i32_layout(meta: dict[str, str]) -> list[str]:
    layout = meta.get("tune_reserved_i32", "")
    if not layout:
        layout = meta.get("wall_trace_reserved_i32", "")
    return [x.strip() for x in layout.split(",") if x.strip()]


def _target_distance_mm(row: dict[str, str], meta: dict[str, str]) -> Optional[float]:
    if "target_distance_mm" in row:
        return _num(row, "target_distance_mm")
    layout = _reserved_i32_layout(meta)
    if len(layout) >= 3 and layout[2] == "target_distance_x1000":
        return _num(row, "reserved_i32_2") / 1000.0
    return None


def _build_output(columns: list[str], rows: list[dict[str, str]], meta: dict[str, str], with_derived: bool) -> tuple[list[str], list[list[str]]]:
    first_ts = _num(rows[0], "timestamp_ms")
    base_columns = ["time", "time_ms"] + columns
    base_column_set = set(base_columns)
    derived_columns: list[str] = []
    tune_axis = _infer_tune_axis(rows, meta)

    if with_derived:
        if "target_distance_mm" not in base_column_set:
            derived_columns.append("target_distance_mm")
        derived_columns.extend([
            c for c in [
                "distance_error_mm",
                "angle_deg",
                "target_angle_deg",
                "target_omega_dps",
                "real_omega_dps",
                "gyro_z_raw_dps",
                "velocity_error_mm_s",
                "accel_velocity_error_mm_s",
                "omega_error_dps",
                "motor_out_avg",
                "motor_out_diff",
                "flag_idle",
                "flag_motor_forward",
                "flag_motor_coast",
                "flag_motor_reverse",
                "flag_abort_switch",
                "flag_abort_wall_fault",
                "flag_abort_encoder_fault",
                "flag_abort_imu_fault",
                "flag_angle_target",
                "flag_auto",
            ] if c not in base_column_set
        ])
        if tune_axis is not None:
            derived_columns.extend(["tune_ref", "tune_error"])

    out_columns = base_columns + derived_columns
    out_rows: list[list[str]] = []
    target_distance_integrated = 0.0
    prev_timestamp_ms = first_ts

    for row in rows:
        timestamp_ms = _num(row, "timestamp_ms")
        time_ms = timestamp_ms - first_ts
        values = [_fmt(time_ms / 1000.0), _fmt(time_ms)]
        values.extend(row.get(c, "") for c in columns)

        if with_derived:
            flags = _int(row, "flags")
            target_distance = _target_distance_mm(row, meta)
            tune_ref = _num(row, "reserved_i32_0") / 1000.0
            if target_distance is None and tune_axis == "velocity":
                target_distance_integrated += tune_ref * max(0.0, timestamp_ms - prev_timestamp_ms) * 0.001
                target_distance = target_distance_integrated
            elif target_distance is None and tune_axis == "distance":
                target_distance = tune_ref
            target_omega_dps = _num(row, "target_omega_mdps") / 1000.0
            real_omega_dps = _num(row, "real_omega_mdps") / 1000.0
            gyro_z_raw_dps = _num(row, "gyro_z_raw_mdps") / 1000.0
            motor_l = _num(row, "motor_out_l")
            motor_r = _num(row, "motor_out_r")
            derived_values = [
                ("target_distance_mm", _fmt(target_distance if target_distance is not None else 0.0)),
                ("distance_error_mm", _fmt((target_distance - _num(row, "distance_mm")) if target_distance is not None else 0.0)),
                ("angle_deg", _fmt(_num(row, "angle_mdeg") / 1000.0)),
                ("target_angle_deg", _fmt(_num(row, "target_angle_mdeg") / 1000.0)),
                ("target_omega_dps", _fmt(target_omega_dps)),
                ("real_omega_dps", _fmt(real_omega_dps)),
                ("gyro_z_raw_dps", _fmt(gyro_z_raw_dps)),
                ("velocity_error_mm_s", _fmt(_num(row, "target_velocity_mm_s") - _num(row, "real_velocity_mm_s"))),
                ("accel_velocity_error_mm_s", _fmt(_num(row, "target_velocity_mm_s") - _num(row, "accel_velocity_mm_s"))),
                ("omega_error_dps", _fmt(target_omega_dps - real_omega_dps)),
                ("motor_out_avg", _fmt((motor_l + motor_r) * 0.5)),
                ("motor_out_diff", _fmt(motor_l - motor_r)),
                ("flag_idle", "1" if flags & FLAG_IDLE else "0"),
                ("flag_motor_forward", "1" if flags & FLAG_MOTOR_FWD else "0"),
                ("flag_motor_coast", "1" if flags & FLAG_MOTOR_COAST else "0"),
                ("flag_motor_reverse", "1" if flags & FLAG_MOTOR_REV else "0"),
                ("flag_abort_switch", "1" if flags & FLAG_ABORT_SWITCH else "0"),
                ("flag_abort_wall_fault", "1" if flags & FLAG_ABORT_WALL_FAULT else "0"),
                ("flag_abort_encoder_fault", "1" if flags & FLAG_ABORT_ENCODER_FAULT else "0"),
                ("flag_abort_imu_fault", "1" if flags & FLAG_ABORT_IMU_FAULT else "0"),
                ("flag_angle_target", "1" if flags & FLAG_ANGLE_TARGET else "0"),
                ("flag_auto", "1" if flags & FLAG_AUTO else "0"),
            ]
            values.extend(v for c, v in derived_values if c not in base_column_set)
            if tune_axis is not None:
                if tune_axis == "distance":
                    tune_actual = _num(row, "distance_mm")
                elif tune_axis == "velocity":
                    tune_actual = _num(row, "real_velocity_mm_s")
                elif tune_axis == "angle":
                    tune_actual = _num(row, "angle_mdeg") / 1000.0
                elif tune_axis == "omega":
                    tune_actual = _num(row, "real_omega_mdps") / 1000.0
                else:
                    tune_actual = 0.0
                values.extend([_fmt(tune_ref), _fmt(tune_ref - tune_actual)])

        out_rows.append(values)
        prev_timestamp_ms = timestamp_ms

    return out_columns, out_rows


def _default_output_path(input_path: Path) -> Path:
    return input_path.with_name(f"{input_path.stem}.plotjuggler.csv")


def main() -> int:
    ap = argparse.ArgumentParser(description="Export Nightfall trace CSV to a PlotJuggler-friendly CSV")
    ap.add_argument("input", help="Nightfall CSV file or a directory containing CSV files")
    ap.add_argument("-o", "--output", default="", help="Output CSV path")
    ap.add_argument("--no-derived", action="store_true", help="Do not add derived columns")
    args = ap.parse_args()

    try:
        input_path = _select_input(Path(args.input).expanduser())
        output_path = Path(args.output).expanduser() if args.output else _default_output_path(input_path)
        columns, rows, meta = _load_nightfall_csv(input_path)
        out_columns, out_rows = _build_output(columns, rows, meta, not args.no_derived)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="ascii", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(out_columns)
            writer.writerows(out_rows)
    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        return 1

    print(f"[PlotJuggler] input : {input_path}")
    print(f"[PlotJuggler] output: {output_path}")
    print(f"[PlotJuggler] rows  : {len(out_rows)}")
    print("[PlotJuggler] import: File -> Load data -> CSV, select 'time' as the time axis")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
