"""Export partial observations from saved replays for host / motor-off MCU timing.

All live cases reconstruct only event changes, never the final map. A separate
full_truth case is explicitly evaluation-only and must not drive exploration.
Coordinates and wall bits match the simulator: x east, y north, NESW=1/2/4/8.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import statistics
import sys
import time

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    __package__ = "tools.exploration_sim"

from .engine import Knowledge


def _firmware_nibble(value):
    return sum(bool(value & (1 << direction)) << (3 - direction) for direction in range(4))


def firmware_map(known, walls):
    """F405/F413 map bytes: low walls, high walls-or-unknown, N/E/S/W=8/4/2/1.

    A known edge has equal high and low bits; unknown is high=1 / low=0.
    Firmware declares uint16_t map[][] but each value occupies only eight bits.
    """
    return [[_firmware_nibble(walls[y][x]) |
             (_firmware_nibble(walls[y][x] | (15 ^ known[y][x])) << 4)
             for x in range(len(known[0]))] for y in range(len(known))]


def stage_indices(events, goals):
    goal_set = {tuple(goal) for goal in goals}
    first_goal = next((i for i, e in enumerate(events) if (e["x"], e["y"]) in goal_set), None)
    certificate = next((i for i, e in enumerate(events) if e.get("certified")), None)
    uncertain = [i for i, e in enumerate(events)
                 if i >= (first_goal or 0) and not e.get("certified")]
    selected = {"initial": 0}
    if first_goal is not None:
        selected["first_goal"] = first_goal
    if uncertain:
        selected["uncertain_mid"] = uncertain[len(uncertain) // 2]
    if certificate is not None:
        selected["certificate"] = certificate
    return selected


def _snapshot(knowledge, maze, event, stage, algorithm, result, truth_only=False):
    known = [row[:] for row in knowledge.known]
    walls = [row[:] for row in knowledge.walls]
    goal_mask = [[int((x, y) in {tuple(g) for g in maze["goals"]})
                  for x in range(maze["width"])] for y in range(maze["height"])]
    digest = hashlib.sha256(bytes(v for matrix in (known, walls) for row in matrix for v in row)).hexdigest()
    profile = result["profile"]
    return {
        "id": f'{maze["id"]}_{algorithm}_{stage}', "stage": stage,
        "algorithm": algorithm, "evaluation_only": truth_only,
        "width": maze["width"], "height": maze["height"],
        "x": event["x"], "y": event["y"], "heading": event["heading"],
        "step": event["step"], "sim_elapsed_s": event["t"],
        "sim_incoming_motion_s": event.get("dt"),
        "known": known, "walls": walls,
        "visited": [[int(v) for v in row] for row in knowledge.visited],
        "goal_mask": goal_mask, "goals": maze["goals"],
        "optimistic": knowledge.map_for(True), "conservative": knowledge.map_for(False),
        "firmware_map_u8": firmware_map(known, walls),
        "known_edges": knowledge.count,
        "visited_cells": sum(sum(row) for row in knowledge.visited),
        "map_sha256": digest,
        "replay_bounds": {key: event.get(key) for key in ("lower_s", "upper_s", "certified")},
        "nominal_cell_transit_ms": 1000 * profile["cell_mm"] / profile["search_speed_mm_s"],
        "source": {"maze_id": maze["id"], "maze_source_sha256": maze.get("source_sha256"),
                   "maze_source_revision": maze.get("source_revision"),
                   "fw_git_sha": result.get("metadata", {}).get("fw_git_sha"),
                   "machine": profile["machine"], "options": result.get("options", {})},
    }


def export_cases(result, algorithm="relevant", include_truth=True):
    """Rebuild and validate observation history, then sample named stages."""
    maze = result["maze"]
    width, height = maze["width"], maze["height"]
    if not (1 <= width <= 32 and 1 <= height <= 32):
        raise ValueError("MCU corpus supports 1..32 cells in each dimension")
    run = next((run for run in result["runs"] if run["algorithm"] == algorithm), None)
    if run is None or not run.get("events"):
        raise ValueError(f"Replay has no nonempty {algorithm} run")
    selected = stage_indices(run["events"], maze["goals"])
    knowledge = Knowledge(width, height)
    cases = []
    for index, event in enumerate(run["events"]):
        if event["step"] != index or not (0 <= event["x"] < width and 0 <= event["y"] < height):
            raise ValueError("Replay step / pose is inconsistent")
        for x, y, direction, wall in event.get("changes", []):
            if not (0 <= x < width and 0 <= y < height and 0 <= direction < 4 and wall in (0, 1)):
                raise ValueError("Observation outside the maze or invalid edge")
            if bool(maze["walls"][y][x] & (1 << direction)) != bool(wall):
                raise ValueError("Observation disagrees with evaluation truth")
            knowledge.set_edge(x, y, direction, bool(wall))
        knowledge.visited[event["y"]][event["x"]] = True
        if "known_edges" in event and event["known_edges"] != knowledge.count:
            raise ValueError("Replay known-edge counter disagrees with event history")
        for stage, selected_index in selected.items():
            if index == selected_index:
                cases.append(_snapshot(knowledge, maze, event, stage, algorithm, result))
    if include_truth:
        truth = Knowledge(width, height)
        for y in range(height):
            for x in range(width):
                for direction in range(4):
                    truth.set_edge(x, y, direction, bool(maze["walls"][y][x] & (1 << direction)))
        # Full-map knowledge does not imply the robot actually visited every cell.
        truth.visited = [row[:] for row in knowledge.visited]
        cases.append(_snapshot(truth, maze, run["events"][-1], "full_truth", algorithm, result, True))
    return {"schema": "nightfall_mcu_benchmark_cases_v1", "cases": cases,
            "profile": result["profile"], "options": result.get("options", {}),
            "missing_stages": [name for name in ("first_goal", "uncertain_mid", "certificate") if name not in selected],
            "notes": ["Live cases contain only observations available at that replay step.",
                      "full_truth is evaluation-only and must never be selected by a running exploration policy.",
                      "nominal_cell_transit_ms is a scale reference, not a safe foreground deadline.",
                      "Replay L/U may be retained bounds; a fresh solve can tighten them."]}


def header_text(corpus):
    """Emit standalone const input fixtures; does not add firmware build inputs."""
    lines = ["/* Generated by mcu_benchmark_cases.py. Motor-off benchmark input only. */",
             "#ifndef NIGHTFALL_MCU_BENCHMARK_CASES_H", "#define NIGHTFALL_MCU_BENCHMARK_CASES_H",
             "#include <stdint.h>", "#include <stddef.h>",
             "typedef struct {", "  const char *id;",
             "  uint8_t width, height, x, y, heading, evaluation_only;",
             "  uint16_t step, known_edges;",
             "  const uint8_t *known, *walls, *visited, *goals, *firmware_map;",
             "} nf_mcu_benchmark_case_t;", "",
             f"#if defined(NF_BENCH_CASE_INDEX) && (NF_BENCH_CASE_INDEX < 0 || NF_BENCH_CASE_INDEX >= {len(corpus['cases'])})",
             '#error "NF_BENCH_CASE_INDEX is outside the generated fixture corpus"',
             "#endif", ""]
    fields = {"known": "known", "walls": "walls", "visited": "visited",
              "goals": "goal_mask", "firmware_map": "firmware_map_u8"}
    for index, case in enumerate(corpus["cases"]):
        lines.append(f"#if !defined(NF_BENCH_CASE_INDEX) || NF_BENCH_CASE_INDEX == {index}")
        for cname, key in fields.items():
            values = [v for row in case[key] for v in row]
            lines.append(f"static const uint8_t nf_bench_{index}_{cname}[{len(values)}] = {{")
            for start in range(0, len(values), 32):
                lines.append("  " + ",".join(str(v) for v in values[start:start + 32]) + ",")
            lines.append("};")
        lines.append("#endif")
    lines.extend(["", "static const nf_mcu_benchmark_case_t nf_mcu_benchmark_cases[] = {"])
    for index, case in enumerate(corpus["cases"]):
        identifier = json.dumps(case["id"], ensure_ascii=True)
        numbers = ", ".join(str(int(case[k])) for k in ("width", "height", "x", "y", "heading", "evaluation_only", "step", "known_edges"))
        pointers = ", ".join(f"nf_bench_{index}_{key}" for key in fields)
        lines.append(f"#if !defined(NF_BENCH_CASE_INDEX) || NF_BENCH_CASE_INDEX == {index}")
        lines.append(f"  {{ {identifier}, {numbers}, {pointers} }},")
        lines.append("#endif")
    lines.extend(["};", "#define NF_MCU_BENCHMARK_CASE_COUNT (sizeof(nf_mcu_benchmark_cases) / sizeof(nf_mcu_benchmark_cases[0]))", "#endif", ""])
    return "\n".join(lines)


def host_benchmark(corpus, repeats=3):
    """Measure uncached host oracle calls; do not extrapolate to Cortex-M4."""
    from .oracle import TimeOracle
    rows = []
    for case in corpus["cases"]:
        source, options = case["source"], case["source"]["options"]
        oracle = TimeOracle(source["machine"], options.get("shortest_mode", 2), options.get("shortest_case"))
        for map_name in ("optimistic", "conservative"):
            samples = []
            statuses = []
            for _ in range(repeats):
                # Bypass the Python result memoizer. Internal graph-build caches
                # may warm; that is reported, not mistaken for MCU timing.
                oracle._cache.clear()
                cpu_start, wall_start = time.thread_time_ns(), time.perf_counter_ns()
                solution = oracle.solve(case[map_name], case["goals"])
                cpu_ms = (time.thread_time_ns() - cpu_start) / 1e6
                wall_ms = (time.perf_counter_ns() - wall_start) / 1e6
                samples.append({"cpu_ms": cpu_ms, "wall_ms": wall_ms})
                statuses.append(solution.get("status"))
            def metrics(field):
                values = sorted(sample[field] for sample in samples)
                return {"min": min(values), "median": statistics.median(values),
                        "p95_nearest_rank": values[max(0, math.ceil(0.95 * len(values)) - 1)], "max": max(values)}
            rows.append({"case_id": case["id"], "projection": map_name,
                         "map_sha256": case["map_sha256"], "evaluation_only": case["evaluation_only"],
                         "statuses": statuses, "samples": samples,
                         "cpu_ms": metrics("cpu_ms"), "wall_ms": metrics("wall_ms")})
    return {"environment": {"platform": platform.platform(), "machine": platform.machine(),
                             "python": platform.python_version(), "repeats": repeats},
            "scope": "Uncached Python+ctypes+JSON+host C planner. Excludes shared-library build. Internal planner caches may warm. This is not STM32 latency or WCET.",
            "rows": rows}


def mcu_expected(corpus):
    """Expected counters/checksums for planner_bench.c, using the same C kernel."""
    from .mcu_oracle import McuOracle
    oracle = McuOracle()
    status_ids = {"pending": 0, "exact": 1, "no-path": 2, "no-feasible-terminal": 3,
                  "invalid": 4, "capacity": 5, "overflow": 6}
    def checksum(values):
        value = 2166136261
        for byte in values:
            value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
        return value
    rows = []
    for index, case in enumerate(corpus["cases"]):
        for projection, name in enumerate(("optimistic", "conservative")):
            solved = oracle.solve(case[name], case["goals"], budget=4096)
            row = {"case_id": index * 2 + projection, "fixture": case["id"],
                   "projection": name, "evaluation_only": case["evaluation_only"],
                   "status_name": solved["status"], "status": status_ids[solved["status"]],
                   "goal_entry_us": solved["goal_entry_us"], "stop_us": solved["stop_us"],
                   "workspace_bytes": solved["workspace_used"], "expanded": solved["expanded_states"],
                   "work_units": solved["work_units"], "relaxed_edges": solved["relaxed_edges"],
                   "heap_peak": solved["heap_peak"],
                   "required_edges": len(solved["required_edges"]) if solved["requirements_complete"] else 0,
                   "checksum": checksum(solved["required_masks"]) if solved["requirements_complete"] else 0,
                   "input_checksum": checksum(v for row in case[name] for v in row),
                   "host_elapsed_s": solved["elapsed_s"], "map_sha256": case["map_sha256"]}
            rows.append(row)
    root = Path(__file__).resolve().parents[2]
    source_names = ("mcu_slalom_time_planner.c", "mcu_slalom_time_planner.h", "mcu_slalom_tables.c", "mcu_slalom_tables.h")
    return {"schema": "nightfall_mcu_bench_expected_v1", "rows": rows,
            "model": "fixed F413 mode2 case8 even when input maze originated in classic competition",
            "notes": ["Host elapsed time is not Cortex-M4 timing.",
                      "Expected counters use the current C kernel; regenerate after changing queue/planner code.",
                      "MCU timing rows: bit16=begin; bit17=counters; base=(fixtureindex<<1)|projection.",
                      "MCU solver row slices counts step(1) calls, while counters row slices is actual work_units."],
            "source_sha256": {name: hashlib.sha256((root / "common/route" / name).read_bytes()).hexdigest() for name in source_names}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    parser.add_argument("--algorithm", choices=("baseline", "relevant"), default="relevant")
    parser.add_argument("--json", type=Path)
    parser.add_argument("--header", type=Path)
    parser.add_argument("--without-truth", action="store_true")
    parser.add_argument("--host-benchmark", type=Path, help="Measure uncached host oracle calls and write JSON (can take minutes)")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--mcu-expected", type=Path, help="Write same-C-kernel expected rows for the SRAM benchmark")
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    corpus = export_cases(json.loads(args.replay.read_text(encoding="utf-8")), args.algorithm, not args.without_truth)
    corpus["replay_sha256"] = hashlib.sha256(args.replay.read_bytes()).hexdigest()
    for destination, payload in ((args.json, json.dumps(corpus, indent=2)),
                                  (args.header, header_text(corpus))):
        if destination:
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(payload + "\n", encoding="utf-8")
    if args.mcu_expected:
        expected = mcu_expected(corpus)
        args.mcu_expected.parent.mkdir(parents=True, exist_ok=True)
        args.mcu_expected.write_text(json.dumps(expected, indent=2) + "\n", encoding="utf-8")
    if args.host_benchmark:
        measured = host_benchmark(corpus, args.repeats)
        args.host_benchmark.parent.mkdir(parents=True, exist_ok=True)
        args.host_benchmark.write_text(json.dumps(measured, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"cases": [{key: case[key] for key in ("id", "stage", "step", "known_edges", "evaluation_only")} for case in corpus["cases"]], "missing_stages": corpus["missing_stages"]}, indent=2))


if __name__ == "__main__":
    main()
