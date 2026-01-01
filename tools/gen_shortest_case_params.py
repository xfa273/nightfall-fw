#!/usr/bin/env python3

import argparse
import csv
import os
from typing import Any, Dict, List, Tuple


MODES = [2, 3, 4, 5, 6, 7]
CASE_COUNT_BY_MODE = {
    2: 9,
    3: 9,
    4: 9,
    5: 9,
    6: 9,
    7: 5,
}


FLOAT_FIELDS = [
    "acceleration_straight",
    "acceleration_straight_dash",
    "velocity_straight",
    "acceleration_d_straight",
    "acceleration_d_straight_dash",
    "velocity_d_straight",
    "kp_wall",
    "kp_diagonal",
]

INT_FIELDS = [
    "solver_profile",
]

REQUIRED_FIELDS = [
    "mode",
    "case",
    *FLOAT_FIELDS,
    *INT_FIELDS,
]

SOLVER_PROFILE_MAP = {
    "0": 0,
    "1": 1,
    "2": 2,
    "SOLVER_PROFILE_STANDARD": 0,
    "SOLVER_PROFILE_STRAIGHT_STRONG": 1,
    "SOLVER_PROFILE_STRAIGHT_WEAK": 2,
}


def _fmt_float(v: float) -> str:
    s = ("%.10g" % v)
    if ("e" not in s) and ("." not in s):
        s += ".0"
    return f"{s}f"


def _parse_solver_profile(raw: str) -> int:
    k = raw.strip()
    if k in SOLVER_PROFILE_MAP:
        return int(SOLVER_PROFILE_MAP[k])

    try:
        return int(k, 0)
    except Exception as e:
        raise ValueError(f"Invalid solver_profile: {raw}") from e


def _parse_row(row: Dict[str, str]) -> Tuple[int, int, Dict[str, Any]]:
    try:
        mode = int(row["mode"], 0)
        case = int(row["case"], 0)
    except Exception as e:
        raise ValueError(f"Invalid mode/case: mode={row.get('mode')} case={row.get('case')}") from e

    if mode not in MODES:
        raise ValueError(f"Unsupported mode {mode} (supported: {MODES})")

    max_case = CASE_COUNT_BY_MODE[mode]
    if not (1 <= case <= max_case):
        raise ValueError(f"Invalid case {case} for mode {mode} (expected 1..{max_case})")

    params: Dict[str, Any] = {}

    for k in FLOAT_FIELDS:
        try:
            params[k] = float(row[k])
        except Exception as e:
            raise ValueError(f"Invalid float field {k}: {row.get(k)}") from e

    params["solver_profile"] = _parse_solver_profile(row["solver_profile"])

    return mode, case, params


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
                mode, case, params = _parse_row(row)
            except Exception as e:
                raise RuntimeError(f"{path}:{idx}: {e}")

            data.setdefault(mode, {})
            if case in data[mode]:
                raise RuntimeError(f"{path}:{idx}: duplicate entry for mode={mode} case={case}")
            data[mode][case] = params

    return data


def write_c(out_path: str, data: Dict[int, Dict[int, Dict[str, Any]]]) -> None:
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    lines: List[str] = []
    lines.append('#include "shortest_run_params.h"')
    lines.append("")

    for mode in MODES:
        cases = data.get(mode)
        if cases is None:
            raise RuntimeError(f"mode {mode} is missing")

        count = CASE_COUNT_BY_MODE[mode]
        for c in range(1, count + 1):
            if c not in cases:
                raise RuntimeError(f"mode {mode} missing case {c}")

        lines.append(f"const ShortestRunCaseParams_t shortestRunCaseParamsMode{mode}[{count}] = {{")

        for c in range(1, count + 1):
            p = cases[c]
            lines.append("    {")
            for fk in FLOAT_FIELDS:
                lines.append(f"        .{fk} = {_fmt_float(p[fk])},")
            lines.append(f"        .solver_profile = {int(p['solver_profile'])},")
            lines.append("    },")

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
