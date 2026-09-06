"""End-to-end host replay of the actual cooperative C exploration policy.

Uses the fixed F413 mode2 case8 compact oracle, including on a 16x16 classic
competition layout. This is a host algorithm/scheduling experiment, not a
classic-machine oracle or a hardware timing measurement.
"""
from __future__ import annotations

import argparse
from collections import Counter
import ctypes as ct
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    __package__ = "tools.exploration_sim"

from .baseline import next_baseline
from .engine import Knowledge, DX, DY, apply_timing, simulate
from .mazes import load_maze
from .mcu_oracle import McuOracle
from .profiles import get_profile

ROOT = Path(__file__).resolve().parents[2]
STATUSES = ("move", "fallback", "complete", "invalid", "pending", "proposal")
REASONS = ("none", "bad_snapshot", "bad_config", "oracle_pending", "oracle_failure",
           "stale_oracle", "bad_dependencies", "navigation_budget", "cost_range",
           "no_target", "unobserved_step", "certified", "brake_straight", "unsafe_acceleration", "surrogate",
           "progress_fallback")


class Config(ct.Structure):
    _fields_ = [(name, ct.c_uint32) for name in
                ("straight_us", "known_straight_us", "turn90_us", "uturn_us",
                 "navigation_edge_budget", "oracle_edge_budget")] + [
                ("unknown_cost_per_mille", ct.c_uint16), ("information_gain_per_mille", ct.c_uint16),
                ("pending_surrogate", ct.c_bool)]


class Decision(ct.Structure):
    _fields_ = [(name, ct.c_int) for name in ("status", "reason", "phase", "oracle_status")] + [
        ("generation", ct.c_uint32), ("epoch", ct.c_uint32), ("navigation_edges", ct.c_uint32),
        ("lower_us", ct.c_uint64), ("direction", ct.c_uint8), ("target_x", ct.c_uint8),
        ("target_y", ct.c_uint8), ("has_target", ct.c_bool), ("known_straight", ct.c_bool),
        ("next_is_turn90", ct.c_bool), ("certified", ct.c_bool)]

    def to_dict(self):
        result = {name: getattr(self, name) for name, _ in self._fields_}
        result.update(status=STATUSES[self.status], reason=REASONS[self.reason])
        return result


class Stats(ct.Structure):
    _fields_ = [(name, ct.c_uint64) for name in
                ("policy_jobs", "policy_work_units", "oracle_started", "oracle_callbacks",
                 "oracle_work_units", "oracle_completed", "oracle_aborted", "navigation_edges",
                 "policy_workspace_bytes", "oracle_workspace_bytes")]


def library():
    sources = [ROOT / name for name in (
        "tools/solver_host/mcu_exploration_bridge.c", "common/exploration/exploration_policy.c",
        "common/route/mcu_slalom_time_planner.c", "common/route/mcu_slalom_tables.c")]
    inputs = [*sources, *(source.with_suffix(".h") for source in sources[1:])]
    digest = hashlib.sha256()
    for source in inputs:
        digest.update(source.read_bytes())
    digest.update(os.environ.get("CC", "cc").encode())
    output = ROOT / "build/solver_host" / f"mcu_policy_{digest.hexdigest()[:16]}.so"
    if not output.exists():
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix(f".{os.getpid()}.tmp.so")
        subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-O3", "-shared", "-fPIC",
                        "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-Wno-misleading-indentation",
                        *(str(source) for source in sources), "-o", str(temporary)], check=True)
        temporary.replace(output)
    lib = ct.CDLL(str(output))
    lib.nf_exploration_default_config.restype = Config
    lib.nf_host_exploration_create.argtypes = [ct.c_uint8, ct.c_uint8, ct.POINTER(Config), ct.c_bool]
    lib.nf_host_exploration_create.restype = ct.c_void_p
    lib.nf_host_exploration_free.argtypes = [ct.c_void_p]
    snapshot_args = [ct.c_void_p, *([ct.POINTER(ct.c_uint8)] * 4), ct.c_uint32, ct.c_uint32,
                     ct.c_uint8, ct.c_uint8, ct.c_uint8]
    lib.nf_host_exploration_begin.argtypes = [*snapshot_args, ct.c_bool]
    lib.nf_host_exploration_apply.argtypes = [*snapshot_args, ct.POINTER(Decision)]
    lib.nf_host_exploration_note_goal.argtypes = snapshot_args
    lib.nf_host_exploration_note_goal.restype = ct.c_bool
    lib.nf_host_exploration_guard.argtypes = [*snapshot_args, ct.c_bool, ct.POINTER(Decision)]
    lib.nf_host_exploration_progress.argtypes = [*snapshot_args, ct.c_bool, ct.POINTER(Decision)]
    for name in ("step", "oracle_step"):
        getattr(lib, f"nf_host_exploration_{name}").argtypes = [ct.c_void_p, ct.c_uint32]
    lib.nf_host_exploration_result.argtypes = [ct.c_void_p, ct.POINTER(Decision)]
    lib.nf_host_exploration_stats.argtypes = [ct.c_void_p, ct.POINTER(Stats)]
    return lib


class McuPolicy:
    """Caller-owned immutable copies are held in the C bridge until next begin."""
    def __init__(self, width, height, profile, *, drain=True, oracle_budget=256, pending_surrogate=False):
        self.lib = library()
        config = self.lib.nf_exploration_default_config()
        config.straight_us = round(profile["cell_mm"] / profile["search_speed_mm_s"] * 1e6)
        config.known_straight_us = round(profile["cell_mm"] / profile["known_speed_mm_s"] * 1e6)
        config.turn90_us = round(profile["turn90_s"] * 1e6)
        config.uturn_us = round((profile["uturn_s"] + profile["cell_mm"] / profile["search_speed_mm_s"]) * 1e6)
        config.oracle_edge_budget = oracle_budget
        config.pending_surrogate = pending_surrogate
        self.handle = self.lib.nf_host_exploration_create(width, height, ct.byref(config), drain)
        if not self.handle:
            raise MemoryError("Cannot allocate C policy/oracle workspace")
        self.width, self.height = width, height
        self.generation = 0
        self._observations = None
        self.config = {name: getattr(config, name) for name, _ in config._fields_}
        source_paths = [ROOT / name for name in ("common/exploration/exploration_policy.c",
                                                "common/route/mcu_slalom_time_planner.c",
                                                "common/route/mcu_slalom_tables.c")]
        self.source_sha256 = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                              for path in [*source_paths, *(path.with_suffix(".h") for path in source_paths)]}

    def close(self):
        if self.handle:
            self.lib.nf_host_exploration_free(self.handle)
            self.handle = None

    def _args(self, knowledge, goals, pos, heading, epoch=1):
        if (knowledge.width, knowledge.height) != (self.width, self.height):
            raise ValueError("Snapshot dimensions changed")
        data = ct.c_uint8 * (self.width * self.height)
        goal_set = set(map(tuple, goals))
        goal_mask = [int((x, y) in goal_set) for y in range(self.height) for x in range(self.width)]
        arrays = [data(*(int(v) for row in matrix for v in row)) for matrix in
                  (knowledge.known, knowledge.walls, knowledge.visited)] + [data(*goal_mask)]
        observation = b"".join(bytes(array) for array in arrays[:3])
        if observation != self._observations:
            self.generation += 1
            self._observations = observation
        return [self.handle, *arrays, self.generation, epoch, *pos, heading]

    def begin(self, knowledge, goals, pos, heading, *, predictive=False, epoch=1):
        return self.lib.nf_host_exploration_begin(*self._args(knowledge, goals, pos, heading, epoch), predictive)

    def step(self, budget=4096):
        return self.lib.nf_host_exploration_step(self.handle, budget)

    def oracle_step(self, budget):
        return self.lib.nf_host_exploration_oracle_step(self.handle, budget)

    def result(self):
        decision = Decision()
        self.lib.nf_host_exploration_result(self.handle, ct.byref(decision))
        return decision

    def apply(self, knowledge, goals, pos, heading, decision, *, epoch=1):
        self.lib.nf_host_exploration_apply(*self._args(knowledge, goals, pos, heading, epoch), ct.byref(decision))
        return decision

    def note_goal(self, knowledge, goals, pos, heading):
        return self.lib.nf_host_exploration_note_goal(*self._args(knowledge, goals, pos, heading))

    def guard(self, knowledge, goals, pos, heading, decision, accelerated):
        self.lib.nf_host_exploration_guard(*self._args(knowledge, goals, pos, heading), accelerated, ct.byref(decision))
        return decision

    def progress_guard(self, knowledge, goals, pos, heading, decision, rejected_or_missed):
        self.lib.nf_host_exploration_progress(*self._args(knowledge, goals, pos, heading), rejected_or_missed, ct.byref(decision))
        return decision

    def decide(self, knowledge, goals, pos, heading):
        status = self.begin(knowledge, goals, pos, heading)
        while status == 4:
            status = self.step()
        return self.result()

    def stats(self):
        stats = Stats()
        self.lib.nf_host_exploration_stats(self.handle, ct.byref(stats))
        return {name: getattr(stats, name) for name, _ in stats._fields_}


class EngineOracle:
    """Use the identical fixed compact model for the Python comparison policies."""
    def __init__(self):
        self.oracle = McuOracle()
        self.calls = self.work_units = 0

    def solve(self, walls, goals):
        solved = self.oracle.solve(walls, goals)
        self.calls += 1
        self.work_units += solved["work_units"]
        if solved["status"] not in ("exact", "no-path", "no-feasible-terminal"):
            raise RuntimeError(f"Compact route oracle failed: {solved['status']}")
        return {**solved, "status": "ok" if solved["status"] == "exact" else solved["status"],
                "goal_entry_s": solved["goal_entry_us"] / 1e6 if solved["status"] == "exact" else None}


def _fallback(knowledge, maze, pos, heading, reached_goal):
    result = next_baseline(knowledge.known, knowledge.walls, knowledge.visited,
                           maze.goals, pos, heading, "full" if reached_goal else "goal")
    decision = Decision()
    decision.status = 0 if result["direction"] is not None else 1
    decision.direction = result["direction"] if result["direction"] is not None else 255
    decision.known_straight = result["known_straight"]
    decision.next_is_turn90 = result["next_is_turn90"]
    return decision, result


def _travel_allowance(profile, turn):
    # A deliberately local conservative scheduler window: no future path or
    # ground-truth data is used to estimate how much foreground work fits.
    if turn in (1, 3):
        return profile["turn90_s"] * 1000
    if turn == 2:
        return (profile["uturn_s"] + profile["cell_mm"] / profile["search_speed_mm_s"]) * 1000
    return profile["cell_mm"] / profile["known_speed_mm_s"] * 1000


def _predict(policy, knowledge, maze, pos, heading, milliseconds, policy_per_ms, oracle_per_ms):
    """Independent policy/oracle work ceilings, not a Cortex-M time model."""
    status = policy.begin(knowledge, maze.goals, pos, heading, predictive=True)
    initial = policy.stats()
    policy_limit = max(0, int(milliseconds * policy_per_ms))
    oracle_limit = max(0, int(milliseconds * oracle_per_ms))
    while True:
        stats = policy.stats()
        remaining_policy = policy_limit - (stats["policy_work_units"] - initial["policy_work_units"])
        remaining_oracle = oracle_limit - (stats["oracle_work_units"] - initial["oracle_work_units"])
        if status == 4:
            # One policy unit may call oracle_step(1); reserve one oracle unit.
            if remaining_policy <= 0 or remaining_oracle <= 0:
                break
            status = policy.step(min(remaining_policy, 256))
        else:
            decision = policy.result()
            if ((decision.status == 1 and decision.reason == 3) or decision.reason == 14) and remaining_oracle > 0:
                oracle_status = policy.oracle_step(min(remaining_oracle, 4096))
                if oracle_status != 0 and remaining_policy > 0:
                    status = policy.begin(knowledge, maze.goals, pos, heading, predictive=True)
                elif oracle_status == 0:
                    continue
                else:
                    break
            else:
                break
    # An unfinished navigation job does not starve an already-started oracle.
    # The firmware adapter can schedule these workspaces independently too.
    remaining_oracle = oracle_limit - (policy.stats()["oracle_work_units"] - initial["oracle_work_units"])
    if remaining_oracle > 0:
        policy.oracle_step(remaining_oracle)
    final = policy.stats()
    return policy.result(), {
        "window_ms": milliseconds, "policy_budget": policy_limit, "oracle_budget": oracle_limit,
        "policy_used": final["policy_work_units"] - initial["policy_work_units"],
        "oracle_used": final["oracle_work_units"] - initial["oracle_work_units"],
        "ready": status != 4,
    }


def simulate_c(maze, profile, *, max_steps=12000, predictive=False,
               policy_per_ms=256, oracle_per_ms=4096, pending_surrogate=False, progress=None):
    if tuple(maze.start) != (0, 0):
        raise ValueError("Current C replay adapter supports firmware start (0,0), north")
    policy = McuPolicy(maze.width, maze.height, profile, drain=not predictive,
                       oracle_budget=1 if predictive else 4096, pending_surrogate=pending_surrogate)
    knowledge = Knowledge(maze.width, maze.height)
    pos, heading = tuple(maze.start), 0
    reached_goal = False
    goal_step = certificate_step = None
    incoming = dict(turn=0, known_straight=False)
    events, predicted = [], None
    lower = None
    reason = "step_limit"
    counters = dict(fallback_steps=0, accepted_predictions=0, rejected_predictions=0,
                    pending_arrivals=0, acceleration_brake_steps=0, progress_recovery_steps=0)
    started = time.perf_counter()
    try:
        for step in range(max_steps + 1):
            changes = knowledge.observe(maze.walls, pos, heading)
            if pos in maze.goals:
                if not reached_goal:
                    goal_step = step
                reached_goal = True
                policy.note_goal(knowledge, maze.goals, pos, heading)
            prediction_schedule = None
            if predictive:
                if predicted is not None:
                    decision, prediction_schedule = predicted
                    if decision.status == 4:
                        counters["pending_arrivals"] += 1
                    else:
                        decision = policy.apply(knowledge, maze.goals, pos, heading, decision)
                else:
                    decision = Decision(status=4, direction=255)
            else:
                decision = policy.decide(knowledge, maze.goals, pos, heading)
                if decision.status in (0, 2):
                    decision = policy.apply(knowledge, maze.goals, pos, heading, decision)
            ungated_decision = decision.to_dict()
            rejected_or_missed = predicted is not None and decision.status not in (0, 2)
            decision = policy.progress_guard(knowledge, maze.goals, pos, heading, decision, rejected_or_missed)
            if decision.reason == 15:
                counters["progress_recovery_steps"] += 1
            if predictive and predicted is not None:
                if decision.status in (0, 2):
                    counters["accepted_predictions"] += 1
                elif ungated_decision["status"] != "pending":
                    counters["rejected_predictions"] += 1
            raw_decision = decision.to_dict()
            if decision.lower_us:
                lower = decision.lower_us / 1e6
            if decision.status == 2 and decision.certified and reached_goal:
                certificate_step = step
                reason = "route_certified"
            fallback = None
            if decision.status not in (0, 2):
                counters["fallback_steps"] += 1
                decision, fallback = _fallback(knowledge, maze, pos, heading, reached_goal)
            # Mirror the portable guard: a previous accelerated straight
            # promises one more observed-open forward cell before any turn.
            decision = policy.guard(knowledge, maze.goals, pos, heading, decision,
                                     bool(incoming["known_straight"]))
            if decision.reason == 12:
                counters["acceleration_brake_steps"] += 1
                certificate_step = None
                reason = "step_limit"
            if decision.status == 3:
                raise RuntimeError(f"Acceleration/decision guard failed at step {step}: {decision.to_dict()}")
            certified = decision.status == 2 and decision.certified and reached_goal
            event = dict(step=step, x=pos[0], y=pos[1], heading=heading, changes=changes,
                         known_edges=knowledge.count, visited_cells=sum(sum(row) for row in knowledge.visited),
                         lower_s=lower, upper_s=lower if certified else None, certified=certified,
                         phase="done" if certified else "verify" if reached_goal else "goal",
                         target=[decision.target_x, decision.target_y] if decision.has_target else None,
                         dt=0.0, t=0.0, decision=raw_decision, applied_decision=decision.to_dict(),
                         ungated_decision=ungated_decision,
                         schedule=prediction_schedule, **incoming)
            events.append(event)
            if progress and (step % 50 == 0 or certified):
                progress({"algorithm": "c_predictive" if predictive else "c_drain", "step": step,
                          "known_edges": knowledge.count, "certified": certified})
            if certified or step == max_steps:
                break
            if fallback and fallback["phase"] == "done":
                reason = "full_map_without_certificate"
                break
            direction = decision.direction
            if direction >= 4 or not knowledge.open(*pos, direction):
                raise RuntimeError(f"C replay attempted an unobserved/blocked step at {step}: {raw_decision}")
            if maze.walls[pos[1]][pos[0]] & (1 << direction):
                raise RuntimeError("C replay collision with truth")
            next_pos = (pos[0] + DX[direction], pos[1] + DY[direction])
            incoming = dict(turn=(direction - heading) % 4, known_straight=bool(decision.known_straight))
            if predictive:
                predicted = _predict(policy, knowledge, maze, next_pos, direction,
                                     _travel_allowance(profile, incoming["turn"]), policy_per_ms, oracle_per_ms)
            pos, heading = next_pos, direction
        duration = apply_timing(events, profile)
        stats = policy.stats()
        fallback_reasons = dict(Counter(event["decision"]["reason"] for event in events
                                        if event["decision"]["status"] not in ("move", "complete")))
        surrogate_steps = sum(event["applied_decision"]["reason"] == "surrogate" for event in events)
        # Independent fresh solves verify that a returned certificate also
        # holds on the fully observed-open projection and the real maze.
        audit = None
        if events[-1]["certified"]:
            audit_oracle = McuOracle()
            optimistic = audit_oracle.solve(knowledge.map_for(True), maze.goals)
            conservative = audit_oracle.solve(knowledge.map_for(False), maze.goals)
            truth = audit_oracle.solve(maze.walls, maze.goals)
            costs = [result["goal_entry_us"] for result in (optimistic, conservative, truth)]
            if any(result["status"] != "exact" for result in (optimistic, conservative, truth)) or len(set(costs)) != 1:
                raise RuntimeError(f"False C certificate: costs={costs}")
            if costs[0] != round(lower * 1e6):
                raise RuntimeError("Cached certificate cost differs from independent fresh solve")
            audit = dict(optimistic_us=costs[0], conservative_us=costs[1], truth_us=costs[2],
                         independent_solves=3, passed=True)
        return {"algorithm": "c_predictive" if predictive else "c_drain", "events": events,
                "summary": dict(duration_s=duration, steps=len(events) - 1,
                    distance_m=(len(events) - 1) * profile["cell_mm"] / 1000,
                    turns90=sum(event["turn"] in (1, 3) for event in events[1:]),
                    uturns=sum(event["turn"] == 2 for event in events[1:]),
                    empty_transit_steps=sum(not event["changes"] for event in events[1:]),
                    known_edges=knowledge.count, visited_cells=events[-1]["visited_cells"],
                    first_goal_s=events[goal_step]["t"] if goal_step is not None else None,
                    certificate_s=events[certificate_step]["t"] if certificate_step is not None else None,
                    home_s=None, certified=events[-1]["certified"], lower_s=lower,
                    upper_s=lower if events[-1]["certified"] else None, reason=reason,
                    completed=events[-1]["certified"], host_elapsed_s=time.perf_counter() - started,
                    fallback_reasons=fallback_reasons, surrogate_steps=surrogate_steps, **counters, **stats),
                "certificate_audit": audit, "policy_config": policy.config,
                "scheduling": dict(predictive=predictive, pending_surrogate=pending_surrogate,
                                   policy_units_per_ms=policy_per_ms, oracle_units_per_ms=oracle_per_ms,
                                   model="Independent work ceilings per nominal travel ms; not measured MCU timing"),
                "source_sha256": policy.source_sha256,
                "final_known": knowledge.known, "final_walls": knowledge.walls,
                "optimistic_route": [], "known_route": []}
    finally:
        policy.close()


def compare(maze, profile, *, max_steps=12000, include_predictive=False,
            policy_per_ms=256, oracle_per_ms=4096, pending_surrogate=False, progress=None):
    runs = []
    for algorithm in ("baseline", "relevant"):
        oracle = EngineOracle()
        run = simulate(maze, profile, oracle, algorithm, max_steps=max_steps, progress=progress)
        run["summary"].update(oracle_started=oracle.calls, oracle_work_units=oracle.work_units)
        runs.append(run)
    runs.append(simulate_c(maze, profile, max_steps=max_steps, progress=progress))
    if include_predictive:
        runs.append(simulate_c(maze, profile, max_steps=max_steps, predictive=True,
                              policy_per_ms=policy_per_ms, oracle_per_ms=oracle_per_ms,
                              pending_surrogate=pending_surrogate, progress=progress))
    reference = [(event["x"], event["y"], event["heading"]) for event in runs[1]["events"]]
    for run in runs:
        route = [(event["x"], event["y"], event["heading"]) for event in run["events"]]
        divergence = next((i for i, (left, right) in enumerate(zip(reference, route)) if left != right), None)
        if divergence is None and len(reference) != len(route):
            divergence = min(len(reference), len(route))
        run["summary"].update(first_divergence_from_python_relevant=divergence,
                              route_sha256=hashlib.sha256(json.dumps(route).encode()).hexdigest())
    return {"schema": "nightfall_mcu_policy_replay_v1", "maze": maze.to_dict(), "profile": profile,
            "options": dict(max_steps=max_steps, shortest_mode=2, shortest_case=8, epsilon=0,
                            return_home=False, policy_units_per_ms=policy_per_ms,
                            oracle_units_per_ms=oracle_per_ms, pending_surrogate=pending_surrogate), "runs": runs,
            "metadata": {"oracle": "actual compact C, fixed F413 mode2 case8 for every policy and maze size",
                "notes": ["Classic competition data is a 16x16 layout only; timing uses the selected exploration profile and F413 shortest model.",
                          "C probe navigation is quantized to 5ms; ties/dependencies may differ from Python, so route equality is not required.",
                          "C guard can add one braking straight after an accelerated arrival.",
                          "Movement times are the simulator kinematic estimate; host CPU elapsed is not STM32 latency.",
                          "Predictive ceilings are independent work units per nominal travel ms, not measured CPU throughput.",
                          "Predictive mode uses real C begin/step/apply and separate oracle stepping, with legacy fallback on missed arrival or newly blocked proposed edges.",
                          "Oracle counters exclude the independent final certificate audit.",
                          "No firmware, UART, motors or NVM are touched."]}}


def benchmark_grid(maze_ids, profile, *, policy_per_ms=64, oracle_rates=(20, 50),
                   max_steps=6000, trace_directory=None, progress=None):
    """Compact repeatable report, with full event traces optionally in build/."""
    report = {"schema": "nightfall_mcu_policy_budget_grid_v1", "profile": profile,
              "policy_units_per_ms": policy_per_ms, "oracle_rates": list(oracle_rates),
              "max_steps": max_steps, "experiments": [], "mazes": {},
              "model": "All policies use fixed F413 mode2 case8 compact C oracle; nominal exploration movement timing only",
              "limitations": ["Independent work ceilings are scenarios, not validated interrupt/foreground CPU budgets.",
                              "No sensor/copy/initialization/IRQ CPU overhead or motion-HIL uncertainty is modeled.",
                              "Surrogate navigation never certifies; fresh exact optimistic/known/truth solves audit each completion.",
                              "No-certificate outcomes remain incomplete; no stationary waiting time is fabricated."]}
    if trace_directory is not None:
        trace_directory = Path(trace_directory)
        trace_directory.mkdir(parents=True, exist_ok=True)
    for maze_id in maze_ids:
        maze = load_maze(maze_id)
        report["mazes"][maze_id] = {key: value for key, value in maze.to_dict().items() if key != "walls"}
        baseline = simulate(maze, profile, EngineOracle(), "baseline", max_steps=max_steps)
        for rate in oracle_rates:
            for hybrid in (False, True):
                run = simulate_c(maze, profile, predictive=True, max_steps=max_steps,
                                 policy_per_ms=policy_per_ms, oracle_per_ms=rate,
                                 pending_surrogate=hybrid)
                row = {"maze_id": maze_id, "policy_units_per_ms": policy_per_ms,
                       "oracle_units_per_ms": rate, "pending_surrogate": hybrid,
                       "baseline": baseline["summary"], "summary": run["summary"],
                       "certificate_audit": run["certificate_audit"], "policy_config": run["policy_config"],
                       "source_sha256": run["source_sha256"]}
                if trace_directory is not None:
                    destination = trace_directory / f"{maze_id}-policy{policy_per_ms:g}-oracle{rate:g}-hybrid{int(hybrid)}.json"
                    destination.write_text(json.dumps({"maze": maze.to_dict(), "profile": profile, "run": run}, indent=2) + "\n")
                    row["artifact"] = str(destination)
                report["experiments"].append(row)
                if progress:
                    progress({"maze_id": maze_id, "oracle_units_per_ms": rate, "pending_surrogate": hybrid,
                              "steps": run["summary"]["steps"], "duration_s": run["summary"]["duration_s"],
                              "certified": run["summary"]["certified"]})
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("maze_id", nargs="?")
    parser.add_argument("--grid", nargs="+", metavar="MAZE", help="Compare surrogate off/on over several mazes")
    parser.add_argument("--oracle-rates", nargs="+", type=float, default=[20, 50])
    parser.add_argument("--trace-directory", type=Path, help="Full per-grid event JSON output directory (use build/)")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--machine", default="mini_r2_0", choices=("mini_r2_0", "classic_r1_0"))
    parser.add_argument("--max-steps", type=int, default=12000)
    parser.add_argument("--predictive", action="store_true")
    parser.add_argument("--pending-surrogate", action=argparse.BooleanOptionalAction, default=False,
                        help="Guide navigation with a cheap C cardinal route while the exact oracle runs")
    parser.add_argument("--policy-units-per-ms", type=float, default=64)
    parser.add_argument("--oracle-units-per-ms", type=float, default=20)
    args = parser.parse_args()
    if args.max_steps < 1 or args.policy_units_per_ms < 0 or args.oracle_units_per_ms < 0 or any(rate < 0 for rate in args.oracle_rates):
        parser.error("Step limit must be positive and scheduler rates nonnegative")
    if bool(args.maze_id) == bool(args.grid):
        parser.error("Provide one maze_id or --grid MAZE ..., exclusively")
    def progress(update):
        if "step" not in update or update["step"] % 100 == 0 or update.get("certified"):
            print(json.dumps(update), flush=True)
    if args.grid:
        result = benchmark_grid(args.grid, get_profile(args.machine), max_steps=args.max_steps,
                                policy_per_ms=args.policy_units_per_ms, oracle_rates=args.oracle_rates,
                                trace_directory=args.trace_directory, progress=progress)
    else:
        result = compare(load_maze(args.maze_id), get_profile(args.machine), max_steps=args.max_steps,
                         include_predictive=args.predictive, policy_per_ms=args.policy_units_per_ms,
                         oracle_per_ms=args.oracle_units_per_ms, pending_surrogate=args.pending_surrogate, progress=progress)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    if "runs" in result:
        print(json.dumps([{ "algorithm": run["algorithm"], **run["summary"]} for run in result["runs"]], indent=2))


if __name__ == "__main__":
    main()
