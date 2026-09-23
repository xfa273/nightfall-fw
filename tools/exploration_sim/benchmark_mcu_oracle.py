"""Compare the MCU kernel against its source model on catalog or replay snapshots.

Pure host execution. No serial, debug probe, firmware changes, or motor control.
"""
from __future__ import annotations

import argparse
import ctypes
import json
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    __package__ = "tools.exploration_sim"

from .mcu_oracle import McuOracle
from .oracle import TimeOracle
from .mazes import MazeFormatError, catalog, load_maze


def only_required(walls, required):
    height, width = len(walls), len(walls[0])
    result = [[15] * width for _ in range(height)]
    for x, y, direction in required:
        dx, dy = ((0, 1), (1, 0))[direction]
        if walls[y][x] & (1 << direction):
            raise AssertionError("MCU dependency includes a closed edge")
        result[y][x] &= 15 ^ (1 << direction)
        result[y + dy][x + dx] &= 15 ^ (1 << ((direction + 2) % 4))
    return result


def compare(mcu, generic, identifier, projection, walls, goals, budget):
    actual = mcu.solve(walls, goals, budget=budget)
    expected = generic.solve(walls, goals, details=False)
    parity = (actual["status"] == ("exact" if expected["status"] == "ok" else expected["status"]) and
              (actual["status"] != "exact" or actual["goal_entry_us"] ==
               round(expected["goal_entry_s"] * 1_000_000)))
    sufficient = None
    if actual["status"] == "exact":
        reconstructed = generic.solve(only_required(walls, actual["required_edges"]), goals, details=False)
        sufficient = (reconstructed["status"] == "ok" and actual["goal_entry_us"] ==
                      round(reconstructed["goal_entry_s"] * 1_000_000))
    compact = {k: v for k, v in actual.items() if k not in ("required_masks", "required_edges")}
    return {"id": identifier, "projection": projection, "width": len(walls[0]),
            "height": len(walls), "mcu": compact, "generic": expected,
            "parity": parity, "requirements_sufficient": sufficient}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", action="store_true")
    parser.add_argument("--cases", type=Path, action="append", default=[])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--budget", type=int, default=4096)
    args = parser.parse_args(argv)
    if not args.catalog and not args.cases:
        parser.error("Choose --catalog or --cases")
    mcu, generic = McuOracle(), TimeOracle("mini_r2_0", mode=2, case=8)
    fingerprint = bytes((ctypes.c_char * 65).in_dll(mcu.lib, "nf_mcu_slalom_inputs_sha256")).split(b"\0", 1)[0].decode()
    report = {"schema": "nightfall_mcu_oracle_parity_v1", "profile": "f413-mode2-case8",
              "table_inputs_sha256": fingerprint, "budget": args.budget,
              "rows": [], "skipped": []}
    cases = []
    if args.catalog:
        for entry in catalog(download=False):
            try:
                maze = load_maze(entry["id"], download=False)
                cases.append((entry["id"], "full_truth", maze.walls, maze.goals))
            except MazeFormatError as error:
                report["skipped"].append({"id": entry["id"], "reason": str(error)})
    for filename in args.cases:
        for case in json.loads(filename.read_text())["cases"]:
            for projection in ("optimistic", "conservative"):
                cases.append((case["id"], projection, case[projection], case["goals"]))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for identifier, projection, walls, goals in cases:
        row = compare(mcu, generic, identifier, projection, walls, goals, args.budget)
        report["rows"].append(row)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        print(identifier, projection, row["mcu"]["status"],
              f'{row["mcu"]["elapsed_s"] * 1000:.3f}ms',
              "PASS" if row["parity"] and row["requirements_sufficient"] is not False else "FAIL", flush=True)
    rows = report["rows"]
    report["passed"] = all(r["parity"] and r["requirements_sufficient"] is not False for r in rows)
    report["summary"] = {"cases": len(rows), "skipped": len(report["skipped"]),
                         "exact": sum(r["mcu"]["status"] == "exact" for r in rows),
                         "max_host_elapsed_s": max(r["mcu"]["elapsed_s"] for r in rows),
                         "max_workspace": max(r["mcu"]["workspace_used"] for r in rows),
                         "max_heap_groups": max(r["mcu"]["heap_peak"] for r in rows)}
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["summary"]))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
