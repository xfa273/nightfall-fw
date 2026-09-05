"""Exact host time-planner adapter. Coordinates are cell centres, y north.

Input walls use ``1 << direction`` for N/E/S/W directions 0/1/2/3.
The compiled generic planners use a different bit layout; C translates it.
Imported planner source is pinned in this checkout, so no sibling worktree,
network, numpy, or runtime git object is required.
"""
from __future__ import annotations

import ctypes
import hashlib
import json
from pathlib import Path
import os
import platform
import subprocess
import threading

ROOT = Path(__file__).resolve().parents[2]
PLANNER_SOURCE_COMMIT = "4fb45ed"
_BUILD_LOCK = threading.Lock()


def _library(machine: str) -> ctypes.CDLL:
    classic = machine in ("classic_r1_0", "classic_v2", "classic")
    profile = "classic_r1_0" if classic else "f413_preorder"
    names = [
        "tools/solver_host/exploration_oracle.c",
        "tools/solver_host/slalom_time_plan_host.c",
        "tools/solver_host/slalom_profile_baseline.c",
        "tools/solver_host/maze_ascii.c",
        "common/route/motion_time.c",
        "common/route/orthogonal_time_planner.c",
        "common/route/route_clearance.c",
        "common/route/slalom_plan_legacy_codec.c",
        "common/route/legacy_path_codec.c",
        f"params/{profile}/shortest_run_params_split.c",
    ]
    # slalom_time_planner.c is included in the adapter for private geometry replay.
    tracked = [Path(__file__), *(ROOT / name for name in names)]
    tracked.extend((ROOT / "common/route").glob("*.h"))
    tracked.extend((ROOT / "tools/solver_host").glob("*.h"))
    tracked.extend([ROOT / "common/route/slalom_time_planner.c",
                    ROOT / "tools/solver_host/slalom_trace_cache.inc",
                    ROOT / f"params/{profile}/params.h",
                    ROOT / "platform/stm32f405/Core/Inc/shortest_run_params.h",
                    ROOT / "platform/stm32f405/Core/Inc/solver_params.h"])
    digest = hashlib.sha256()
    for file in tracked:
        digest.update(file.read_bytes())
    digest.update(platform.platform().encode())
    digest.update(os.environ.get("CC", "cc").encode())
    build = ROOT / "build/solver_host"
    output = build / f"exploration_oracle_{profile}_{digest.hexdigest()[:16]}.so"
    with _BUILD_LOCK:
        if not output.exists():
            build.mkdir(parents=True, exist_ok=True)
            # Add an exact replay memoizer only to the host build copy. Keep
            # imported common/route source byte-identical for future merges.
            source = (ROOT / "common/route/slalom_time_planner.c").read_text()
            marker = "static double nf_slalom_connector_command_unit("
            if source.count(marker) != 1:
                raise RuntimeError("Planner source changed: review host replay-cache insertion")
            helper = (ROOT / "tools/solver_host/slalom_trace_cache.inc").read_text()
            generated = output.with_suffix(".generated.c")
            generated.write_text(source.replace(marker, helper + "\n" + marker))
            temporary = output.with_suffix(f".{os.getpid()}.tmp.so")
            command = [os.environ.get("CC", "cc"), "-std=c11", "-O3", "-fPIC", "-shared",
                       "-Wall", "-Wextra", "-Wpedantic", "-Wno-strict-prototypes",
                       f"-DNF_ORACLE_CLASSIC={int(classic)}",
                       f'-DNF_ORACLE_SLALOM_SOURCE="{generated}"']
            for directory in ("common/route", "tools/solver_host", "tools/solver_host/include",
                              "platform/stm32f405/Core/Inc", f"params/{profile}"):
                command.extend(["-I", str(ROOT / directory)])
            command.extend(str(ROOT / name) for name in names)
            command.extend(["-lm", "-o", str(temporary)])
            result = subprocess.run(command, capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError(f"Host time-planner build failed:\n{result.stderr}")
            temporary.replace(output)
    lib = ctypes.CDLL(str(output))
    lib.nf_exploration_oracle.argtypes = [ctypes.c_uint, ctypes.c_uint,
        ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_uint, ctypes.c_uint, ctypes.c_uint,
        ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint,
        ctypes.c_char_p, ctypes.c_size_t]
    lib.nf_exploration_oracle.restype = ctypes.c_int
    return lib


class TimeOracle:
    """Goal-entry minimum over the imported planner's finite motion graph.

    ``stop_s`` is the selected route's feasible stopping-tail time; it is not
    the optimized objective. The graph, provisional diagonal geometry, and
    commanded kinematics do not constitute a physical optimality guarantee.
    Exploration speed is configured independently in the simulator.
    """
    def __init__(self, machine="mini_r2_0", mode=2, case=None, planner=None):
        classic = machine in ("classic_r1_0", "classic_v2", "classic")
        self.machine = machine
        self.mode = int(mode)
        self.case = int(case if case is not None else (1 if classic else 8))
        self.planner = planner or ("orthogonal" if classic else "slalom")
        if machine not in ("mini_r2_0", "f413_preorder", "half", "classic_r1_0", "classic_v2", "classic"):
            raise ValueError(f"Unknown machine: {machine}")
        if self.planner not in ("slalom", "orthogonal") or (classic and self.planner == "slalom"):
            raise ValueError("Classic has an orthogonal time model; its diagonal geometry is not calibrated")
        if self.mode not in range(2, 6) or self.case not in range(1, 10):
            raise ValueError("Shortest mode must be 2..5 and case 1..9")
        self.lib = _library(machine)
        self._lock = threading.Lock()
        self._cache = {}

    def solve(self, walls, goals, details=True, start=(0, 0), heading=0):
        """Return JSON-ready costs and a sufficient set of required open edges.

        ``required_edges`` contains canonical N or E ``[x, y, direction]``
        edges. Mini requirements include topology guards, sampled centre-line
        crossings, and the complete stopping tail. ``route_points`` uses
        fractional cell-centre coordinates (0,0 is the start centre).
        """
        height = len(walls)
        width = len(walls[0]) if height else 0
        if not 1 <= width <= 32 or not 1 <= height <= 32 or any(len(row) != width for row in walls):
            raise ValueError("Expected a rectangular 1..32 cell maze")
        flat = [int(value) for row in walls for value in row]
        if any(value < 0 or value > 15 for value in flat):
            raise ValueError("Wall masks must use four N/E/S/W bits")
        goal_data = [0] * (width * height)
        for x, y in goals:
            if not 0 <= x < width or not 0 <= y < height:
                raise ValueError("Goal outside maze")
            goal_data[y * width + x] = 1
        if not any(goal_data):
            raise ValueError("At least one goal is required")
        sx, sy = start
        if not 0 <= sx < width or not 0 <= sy < height or heading not in range(4):
            raise ValueError("Start pose outside maze or invalid heading")
        key = (width, height, bytes(flat), bytes(goal_data), bool(details), tuple(start), heading)
        with self._lock:
            if key in self._cache:
                return json.loads(self._cache[key])
            buffer = ctypes.create_string_buffer(2_000_000)
            byte_array = ctypes.c_uint8 * (width * height)
            status = self.lib.nf_exploration_oracle(width, height,
                byte_array(*flat), byte_array(*goal_data), sx, sy, heading, int(self.planner == "slalom"),
                self.mode, self.case, int(details), buffer, len(buffer))
            if status:
                raise RuntimeError("Time-planner result allocation or output capacity failed")
            result = json.loads(buffer.value)
            result.update({"machine": self.machine, "mode": self.mode, "case": self.case,
                           "objective": "goal_entry_s", "source_commit": PLANNER_SOURCE_COMMIT})
            result.setdefault("required_edges", [])
            result.setdefault("route_points", [])
            self._cache[key] = json.dumps(result)
            if len(self._cache) > 4096:
                self._cache.pop(next(iter(self._cache)))
            return result
