#!/usr/bin/env python3

import argparse
import csv
import os
from typing import Dict, Any, List, Tuple


TURN_BANK_COUNT = 5
MODES = [2, 3, 4, 5, 6, 7]


FLOAT_FIELDS = [
    "velocity_turn90",
    "alpha_turn90",
    "acceleration_turn",
    "dist_offset_in",
    "dist_offset_out",
    "val_offset_in",
    "angle_turn_90",
    "dist_wall_end",
    "velocity_l_turn_90",
    "alpha_l_turn_90",
    "angle_l_turn_90",
    "dist_l_turn_in_90",
    "dist_l_turn_out_90",
    "velocity_l_turn_180",
    "alpha_l_turn_180",
    "angle_l_turn_180",
    "dist_l_turn_in_180",
    "dist_l_turn_out_180",
    "velocity_turn45in",
    "alpha_turn45in",
    "angle_turn45in",
    "dist_turn45in_in",
    "dist_turn45in_out",
    "velocity_turn45out",
    "alpha_turn45out",
    "angle_turn45out",
    "dist_turn45out_in",
    "dist_turn45out_out",
    "velocity_turnV90",
    "alpha_turnV90",
    "angle_turnV90",
    "dist_turnV90_in",
    "dist_turnV90_out",
    "velocity_turn135in",
    "alpha_turn135in",
    "angle_turn135in",
    "dist_turn135in_in",
    "dist_turn135in_out",
    "velocity_turn135out",
    "alpha_turn135out",
    "angle_turn135out",
    "dist_turn135out_in",
    "dist_turn135out_out",
    "accel_switch_velocity",
]

UINT16_FIELDS = [
    "fan_power",
    "wall_end_thr_r_high",
    "wall_end_thr_r_low",
    "wall_end_thr_l_high",
    "wall_end_thr_l_low",
]

INT_FIELDS = [
    "makepath_type_case3",
    "makepath_type_case47",
]

REQUIRED_FIELDS = [
    "mode",
    "bank",
    *FLOAT_FIELDS,
    *UINT16_FIELDS,
    *INT_FIELDS,
]


def _fmt_float(v: float) -> str:
    # keep a stable and compact representation, always with 'f'
    s = ("%.10g" % v)
    if ("e" not in s) and ("." not in s):
        s += ".0"
    return f"{s}f"


def _parse_row(row: Dict[str, str]) -> Tuple[int, int, Dict[str, Any]]:
    try:
        mode = int(row["mode"], 0)
        bank = int(row["bank"], 0)
    except Exception as e:
        raise ValueError(f"Invalid mode/bank: mode={row.get('mode')} bank={row.get('bank')}") from e

    if mode not in MODES:
        raise ValueError(f"Unsupported mode {mode} (supported: {MODES})")
    if not (0 <= bank < TURN_BANK_COUNT):
        raise ValueError(f"Invalid bank {bank} (expected 0..{TURN_BANK_COUNT-1})")

    params: Dict[str, Any] = {}

    for k in FLOAT_FIELDS:
        try:
            params[k] = float(row[k])
        except Exception as e:
            raise ValueError(f"Invalid float field {k}: {row.get(k)}") from e

    for k in UINT16_FIELDS:
        try:
            v = int(row[k], 0)
        except Exception as e:
            raise ValueError(f"Invalid uint16 field {k}: {row.get(k)}") from e
        if not (0 <= v <= 0xFFFF):
            raise ValueError(f"uint16 out of range {k}: {v}")
        params[k] = v

    for k in INT_FIELDS:
        try:
            params[k] = int(row[k], 0)
        except Exception as e:
            raise ValueError(f"Invalid int field {k}: {row.get(k)}") from e

    return mode, bank, params


def read_csv(path: str) -> Dict[int, Dict[int, Dict[str, Any]]]:
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise RuntimeError("CSV header is missing")

        missing = [k for k in REQUIRED_FIELDS if k not in reader.fieldnames]
        if missing:
            raise RuntimeError(f"CSV missing required columns: {missing}")

        data: Dict[int, Dict[int, Dict[str, Any]]] = {}
        for idx, row in enumerate(reader, start=2):
            if row.get("mode", "").strip() == "":
                continue
            try:
                mode, bank, params = _parse_row(row)
            except Exception as e:
                raise RuntimeError(f"{path}:{idx}: {e}")

            data.setdefault(mode, {})
            if bank in data[mode]:
                raise RuntimeError(f"{path}:{idx}: duplicate entry for mode={mode} bank={bank}")
            data[mode][bank] = params

    return data


def write_c(out_path: str, data: Dict[int, Dict[int, Dict[str, Any]]]) -> None:
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    lines: List[str] = []
    lines.append('#include "shortest_run_params.h"')
    lines.append("")

    for mode in MODES:
        banks = data.get(mode, {})
        if 0 not in banks:
            raise RuntimeError(f"mode {mode} must define bank 0")

        def key_of(p: Dict[str, Any]) -> Tuple[Any, ...]:
            return (
                *(p[k] for k in FLOAT_FIELDS),
                p["fan_power"],
                *(p[k] for k in INT_FIELDS),
                p["wall_end_thr_r_high"],
                p["wall_end_thr_r_low"],
                p["wall_end_thr_l_high"],
                p["wall_end_thr_l_low"],
            )

        emitted: Dict[Tuple[Any, ...], str] = {}
        bank_struct_name: Dict[int, str] = {}

        for bank in range(TURN_BANK_COUNT):
            if bank not in banks:
                continue
            p = banks[bank]
            k = key_of(p)
            if k in emitted:
                bank_struct_name[bank] = emitted[k]
                continue

            struct_name = f"k_turn_mode{mode}_bank{bank}"
            emitted[k] = struct_name
            bank_struct_name[bank] = struct_name

            lines.append(f"static const ShortestRunModeParams_t {struct_name} = {{")
            for fk in FLOAT_FIELDS:
                lines.append(f"    .{fk} = {_fmt_float(p[fk])},")
            lines.append(f"    .fan_power = {p['fan_power']},")
            for ik in INT_FIELDS:
                lines.append(f"    .{ik} = {p[ik]},")
            for uk in ("wall_end_thr_r_high", "wall_end_thr_r_low", "wall_end_thr_l_high", "wall_end_thr_l_low"):
                lines.append(f"    .{uk} = {p[uk]},")
            lines.append("};")
            lines.append("")

        # pointer array (fallback: bank0)
        bank0_name = bank_struct_name.get(0)
        if bank0_name is None:
            raise RuntimeError(f"mode {mode} must define bank 0")

        lines.append(f"const ShortestRunModeParams_t* const shortestTurnBankMode{mode}[SHORTEST_TURN_BANK_COUNT] = {{")
        for bank in range(TURN_BANK_COUNT):
            ref_name = bank_struct_name.get(bank, bank0_name)
            lines.append(f"    &{ref_name},")
        lines.append("};")
        lines.append("")

    with open(out_path, "w", newline="\n") as f:
        f.write("\n".join(lines))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    data = read_csv(args.csv)
    write_c(args.out, data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
