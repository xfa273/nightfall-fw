"""Host harness for the allocation-free MCU oracle (same C as firmware).

The firmware model is fixed to F413 mode2 case8 and integer microseconds.
This adapter exists for parity/regression benchmarks, not simulator defaults.
"""
from __future__ import annotations

import ctypes
import hashlib
import os
from pathlib import Path
import subprocess
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
_BUILD_LOCK = threading.Lock()


class Result(ctypes.Structure):
    _fields_ = [("status", ctypes.c_int),
                ("goal_entry_us", ctypes.c_uint32), ("stop_us", ctypes.c_uint32),
                ("expanded_states", ctypes.c_uint32), ("relaxed_edges", ctypes.c_uint32),
                ("work_units", ctypes.c_uint32), ("heap_peak", ctypes.c_uint16),
                ("action_count", ctypes.c_uint16), ("goal_x", ctypes.c_uint8),
                ("goal_y", ctypes.c_uint8), ("requirements_complete", ctypes.c_bool),
                ("workspace_used", ctypes.c_size_t)]


def library():
    sources = [ROOT / "common/route" / name for name in
               ("mcu_slalom_time_planner.c", "mcu_slalom_tables.c")]
    digest = hashlib.sha256()
    for source in [*sources, *(p.with_suffix(".h") for p in sources)]:
        digest.update(source.read_bytes())
    digest.update(os.environ.get("CC", "cc").encode())
    build = ROOT / "build/solver_host"
    output = build / f"mcu_oracle_{digest.hexdigest()[:16]}.so"
    with _BUILD_LOCK:
        if not output.exists():
            build.mkdir(parents=True, exist_ok=True)
            temporary = output.with_suffix(f".{os.getpid()}.tmp.so")
            subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-O3", "-shared", "-fPIC",
                            "-Wall", "-Wextra", "-Wpedantic", "-Werror",
                            *(str(p) for p in sources), "-o", str(temporary)], check=True)
            temporary.replace(output)
    lib = ctypes.CDLL(str(output))
    lib.nf_mcu_slalom_workspace_bytes.restype = ctypes.c_size_t
    lib.nf_mcu_slalom_workspace_bytes_for.argtypes = [ctypes.c_uint8, ctypes.c_uint8]
    lib.nf_mcu_slalom_workspace_bytes_for.restype = ctypes.c_size_t
    byte_pointer = ctypes.POINTER(ctypes.c_uint8)
    lib.nf_mcu_slalom_begin.argtypes = [ctypes.c_void_p, ctypes.c_size_t,
        ctypes.c_uint8, ctypes.c_uint8, byte_pointer, byte_pointer,
        ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, ctypes.POINTER(ctypes.c_void_p)]
    lib.nf_mcu_slalom_begin.restype = ctypes.c_int
    lib.nf_mcu_slalom_step.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.nf_mcu_slalom_step.restype = ctypes.c_int
    lib.nf_mcu_slalom_result.argtypes = [ctypes.c_void_p, ctypes.POINTER(Result),
                                       byte_pointer, ctypes.c_size_t]
    lib.nf_mcu_slalom_result.restype = ctypes.c_int
    lib.nf_mcu_slalom_status_name.argtypes = [ctypes.c_int]
    lib.nf_mcu_slalom_status_name.restype = ctypes.c_char_p
    return lib


class McuOracle:
    def __init__(self):
        self.lib = library()

    def solve(self, walls, goals, start=(0, 0), heading=0, budget=4096,
              max_work=200_000_000):
        width, height = len(walls[0]), len(walls)
        if not 2 <= width <= 32 or not 2 <= height <= 32 or any(len(r) != width for r in walls):
            raise ValueError("Expected rectangular2..32 maze")
        if budget <= 0:
            raise ValueError("Budget must be positive")
        cells = width * height
        size = self.lib.nf_mcu_slalom_workspace_bytes_for(width, height)
        storage = ctypes.create_string_buffer(size + 24)
        address = (ctypes.addressof(storage) + 7) & ~7
        ctypes.memset(address + size, 0xA5, 16)
        data = ctypes.c_uint8 * cells
        goal_data = [0] * cells
        for x, y in goals:
            goal_data[y * width + x] = 1
        context = ctypes.c_void_p()
        started = time.perf_counter()
        status = self.lib.nf_mcu_slalom_begin(address, size, width, height,
            data(*(v for row in walls for v in row)), data(*goal_data),
            start[0], start[1], heading, ctypes.byref(context))
        begin_s = time.perf_counter() - started
        result = Result()
        required = data()
        calls = 0
        max_call = 0.0
        while status == 0:
            if calls * budget >= max_work:
                break
            before = time.perf_counter()
            status = self.lib.nf_mcu_slalom_step(context, budget)
            max_call = max(max_call, time.perf_counter() - before)
            calls += 1
        if context:
            self.lib.nf_mcu_slalom_result(context, ctypes.byref(result), required, cells)
        if ctypes.string_at(address + size, 16) != bytes([0xA5]) * 16:
            raise RuntimeError("MCU solver wrote beyond caller workspace")
        out = {name: getattr(result, name) for name, _ in Result._fields_}
        out["status"] = self.lib.nf_mcu_slalom_status_name(status).decode()
        out.update(elapsed_s=time.perf_counter() - started, begin_s=begin_s, max_call_s=max_call,
                   calls=calls, required_masks=list(required),
                   required_edges=[[x, y, d] for y in range(height) for x in range(width)
                                   for d in range(2) if required[y * width + x] & (1 << d)])
        return out
