#!/usr/bin/env python3
"""Validate saved motor-off MCU mailbox results against the same C on the host.

Read-only: this script never opens a probe or serial port. Both JSON files are
required so a hardware result is checked against its actual fixture corpus.
"""
from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tools.exploration_sim.mcu_oracle import McuOracle

STATUS = {"pending": 0, "exact": 1, "no-path": 2, "no-feasible-terminal": 3,
          "invalid": 4, "capacity": 5, "overflow": 6}


def checksum(values):
    result = 2166136261
    for value in values:
        result = ((result ^ value) * 16777619) & 0xFFFFFFFF
    return result


def validate(corpus, mailbox, *, case_indices=None, oracle=None):
    oracle = oracle or McuOracle()
    errors = []
    header = mailbox.get("header", {})
    for key, expected in (("magic", 0x4E464542), ("version", 1), ("status", 2),
                          ("cfsr", 0), ("hfsr", 0), ("fault_pc", 0)):
        if header.get(key) != expected:
            errors.append(f"Mailbox {key}: expected {expected}, got {header.get(key)}")
    if header.get("row_count") != len(mailbox.get("rows", [])):
        errors.append("Mailbox row count disagrees with parsed rows")
    if not isinstance(header.get("cpu_hz"), int) or header["cpu_hz"] <= 0:
        errors.append("Missing positive DWT CPU frequency")
    for key in ("motors_allowed", "flash_write", "fram_access"):
        if mailbox.get(key) is not False:
            errors.append(f"Expected {key}=false for this SRAM benchmark")
    if (mailbox.get("flash_unchanged") is not True or
            not mailbox.get("flash_before_sha256") or
            mailbox.get("flash_before_sha256") != mailbox.get("flash_after_sha256")):
        errors.append("Full-Flash before/after SHA256 equality is missing")
    indexed = {}
    for row in mailbox.get("rows", []):
        identifier = row.get("case_id")
        if identifier in indexed:
            errors.append(f"Duplicate row case_id={identifier}")
        indexed[identifier] = row
    indices = list(range(len(corpus["cases"]))) if case_indices is None else list(case_indices)
    rows, expected_ids = [], set()
    for index in indices:
        if not 0 <= index < len(corpus["cases"]):
            errors.append(f"Fixture index {index} is outside corpus")
            continue
        case = corpus["cases"][index]
        for projection in range(2):
            case_id = (index << 1) | projection
            projection_name = "conservative" if projection else "optimistic"
            wall_map = [[walls | ((15 ^ known) if projection else 0)
                         for walls, known in zip(wall_row, known_row)]
                        for wall_row, known_row in zip(case["walls"], case["known"])]
            if wall_map != case[projection_name]:
                errors.append(f"{case['id']} {projection_name}: saved projection is inconsistent")
            expected = oracle.solve(wall_map, case["goals"])
            expected_status = STATUS[expected["status"]]
            init_status = 1 if [0, 0] in [list(goal) for goal in case["goals"]] else 0
            init_fields = {"status": init_status, "slices": 1,
                           "checksum": checksum(value for row in wall_map for value in row)}
            solve_fields = {"status": expected_status, "goal_entry_us": expected["goal_entry_us"],
                            "stop_us": expected["stop_us"], "expanded": expected["expanded_states"],
                            "slices": expected["work_units"],
                            "required_edges": len(expected["required_edges"]),
                            "checksum": checksum(expected["required_masks"]) if expected["requirements_complete"] else 0}
            counter_fields = {"status": expected_status, "slices": expected["work_units"],
                              "expanded": expected["expanded_states"],
                              "required_edges": expected["relaxed_edges"],
                              "checksum": expected["heap_peak"]}
            mismatches = []
            device_workspace = []
            for suffix, fields in ((0x10000, init_fields), (0, solve_fields), (0x20000, counter_fields)):
                identifier = case_id | suffix
                expected_ids.add(identifier)
                actual = indexed.get(identifier)
                if actual is None:
                    mismatches.append(f"Missing row {identifier}")
                    continue
                for name, value in fields.items():
                    if actual.get(name) != value:
                        mismatches.append(f"row {identifier} {name}: expected {value}, got {actual.get(name)}")
                workspace = actual.get("workspace_bytes", 0)
                device_workspace.append(workspace)
                if not 0 < workspace <= 200 * 1024:
                    mismatches.append(f"row {identifier} invalid workspace {workspace}")
            if len(set(device_workspace)) > 1:
                mismatches.append("Init/solve/counter workspace sizes disagree")
            actual_solve = indexed.get(case_id, {})
            rows.append({"id": case["id"], "projection": projection_name,
                         "case_id": case_id, "passed": not mismatches,
                         "errors": mismatches, "status": expected["status"],
                         "goal_entry_us": expected["goal_entry_us"], "stop_us": expected["stop_us"],
                         "required_edges": solve_fields["required_edges"],
                         "required_checksum": solve_fields["checksum"],
                         "work_units": expected["work_units"],
                         "host_workspace_bytes": expected["workspace_used"],
                         "device_workspace_bytes": device_workspace[0] if device_workspace else None,
                         "device_elapsed_ms": actual_solve.get("elapsed_ms"),
                         "device_max_slice_ms": actual_solve.get("max_slice_ms")})
            errors.extend(f"{case['id']} {projection_name}: {error}" for error in mismatches)
    extra_ids = set(indexed) - expected_ids
    if extra_ids:
        errors.append(f"Unexpected mailbox rows: {sorted(extra_ids)}")
    fingerprint = bytes((ctypes.c_char * 65).in_dll(oracle.lib, "nf_mcu_slalom_inputs_sha256")).split(b"\0", 1)[0].decode()
    return {"schema": "nightfall_mcu_planner_validation_v1", "passed": not errors,
            "errors": errors, "rows": rows, "host_table_inputs_sha256": fingerprint,
            "note": "Host/device pointer ABI workspace differences are expected; all algorithm counters and dependency bytes must match."}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=Path, required=True)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--case-index", type=int, action="append")
    args = parser.parse_args(argv)
    report = validate(json.loads(args.cases.read_text()), json.loads(args.result.read_text()),
                      case_indices=args.case_index)
    report.update(cases_sha256=hashlib.sha256(args.cases.read_bytes()).hexdigest(),
                  result_sha256=hashlib.sha256(args.result.read_bytes()).hexdigest(),
                  cases_path=str(args.cases.resolve()), result_path=str(args.result.resolve()))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(f"{'PASS' if report['passed'] else 'FAIL'}: {len(report['rows'])} projections, {len(report['errors'])} errors")
    for error in report["errors"]:
        print(error)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
