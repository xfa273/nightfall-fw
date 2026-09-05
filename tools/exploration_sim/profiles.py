"""Read exploration defaults from the checked-out mode 1 case 1 parameters.

This is a kinematic comparison model, not a prediction of closed-loop hardware
timing. In particular, F413 spot turns stop on measured angle and may perform
sensor-dependent alignment; their duration must be calibrated from a real run.
"""

from __future__ import annotations

import math
from pathlib import Path
import re


REPO_ROOT = Path(__file__).resolve().parents[2]
_MACHINES = {
    "mini_r2_0": ("f413_preorder", "half", "mini_r2 · mode 1 case 1"),
    "classic_r1_0": ("classic_r1_0", "classic", "classic_r1 · mode 1 case 1"),
}
_NUMBER = r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)[fFuU]*"


def _read_number(text: str, pattern: str, name: str) -> float:
    match = re.search(pattern + _NUMBER, text, flags=re.MULTILINE)
    if match is None:
        raise ValueError(f"Cannot read numeric firmware parameter {name}")
    return float(match.group(1))


def _macro(text: str, name: str) -> float:
    return _read_number(text, rf"^\s*#define\s+{re.escape(name)}\s+", name)


def _standard_params(text: str) -> dict[str, float]:
    # Select the first top-level structure rather than relying on translated
    # comments or matching a field from the low-speed entry by accident.
    match = re.search(r"searchRunParams\s*\[\s*2\s*\]\s*=\s*\{\s*(?://[^\n]*\n\s*)*\{([^}]+)\}", text)
    if match is None:
        raise ValueError("Cannot locate searchRunParams[0]")
    block = match.group(1)
    fields = (
        "acceleration_straight", "acceleration_straight_dash",
        "velocity_turn90", "alpha_turn90", "dist_offset_in",
        "dist_offset_out", "angle_turn_90", "wall_align_enable",
    )
    return {
        name: _read_number(block, rf"\.{name}\s*=\s*", name)
        for name in fields
    }


def smooth_turn_duration(angle_deg: float, alpha_deg_s2: float,
                         rounding: float = 1.0, omega_cap: float = 0.0) -> float:
    """Firmware cosine-blended angular profile, excluding in/out straights."""
    if not all(math.isfinite(v) for v in (angle_deg, alpha_deg_s2, rounding, omega_cap)):
        raise ValueError("Turn parameters must be finite")
    if angle_deg <= 0 or alpha_deg_s2 <= 0:
        raise ValueError("Angle and angular acceleration must be positive")
    omega = math.sqrt(2.0 * alpha_deg_s2 * angle_deg / 3.0)
    if omega_cap > 0:
        omega = min(omega, omega_cap)
    t_acc = omega / alpha_deg_s2 * max(0.1, rounding)
    t_cruise = angle_deg / omega - t_acc
    if t_cruise < 0:
        t_cruise = 0.0
        omega = angle_deg / t_acc
        if omega_cap > 0:
            omega = min(omega, omega_cap)
        t_cruise = max(0.0, angle_deg / omega - t_acc)
    return 2.0 * t_acc + t_cruise


def get_profile(machine: str, repo_root: Path | str | None = None) -> dict:
    """Return JSON-ready defaults, source references and timing qualifications.

    Primary velocities use the nominal slalom/half-cell launch speed. Firmware
    entry speed and its first known-straight boost are also reported separately.
    ``uturn_s`` is rotation and dwell only; add braking/launch in the simulator.
    """
    aliases = {"mini_r2": "mini_r2_0", "half": "mini_r2_0",
               "classic_r1": "classic_r1_0", "classic": "classic_r1_0"}
    machine = aliases.get(machine, machine)
    if machine not in _MACHINES:
        raise ValueError(f"Unknown machine: {machine}")
    param_dir, family, label = _MACHINES[machine]
    root = Path(repo_root) if repo_root is not None else REPO_ROOT
    params_path = f"params/{param_dir}/params.h"
    run_path = f"params/{param_dir}/search_run_params_split.c"
    macros = (root / params_path).read_text(encoding="utf-8")
    run = _standard_params((root / run_path).read_text(encoding="utf-8"))
    half_cell = _macro(macros, "DIST_HALF_SEC")
    first_section = _macro(macros, "DIST_FIRST_SEC")
    rounding = _macro(macros, "TURN_OMEGA_PROFILE_ROUNDING_SCALE")
    rotate_alpha = _macro(macros, "ALPHA_ROTATE_90")
    cell = 2.0 * half_cell
    speed = run["velocity_turn90"]
    acceleration = run["acceleration_straight"]
    dash_accel = run["acceleration_straight_dash"]
    if dash_accel <= 0:
        dash_accel = acceleration
    entry_speed = math.sqrt(2 * acceleration * (first_section + half_cell))
    # F413 has an explicit angular cap; no cap exists in the F405 profile.
    omega_cap = 2200.0 if family == "half" else 0.0
    if family == "half":
        path_header = root / "platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc/f413_path_run.h"
        cap_match = re.search(r"#define\s+NIGHTFALL_F413_PATH_OMEGA_CAP\s+\(\s*" + _NUMBER,
                              path_header.read_text(encoding="utf-8"))
        if cap_match is None:
            raise ValueError("Cannot read F413 path angular velocity cap")
        omega_cap = float(cap_match.group(1))
    angular_turn_s = smooth_turn_duration(run["angle_turn_90"], run["alpha_turn90"], rounding, omega_cap)
    turn90_s = angular_turn_s + (run["dist_offset_in"] + run["dist_offset_out"]) / speed
    nominal_rotate_angle = 2 * _macro(macros, "ANGLE_ROTATE_90_R")
    # F405 driveR uses three angular thirds. This reference is also a useful
    # explicit starting estimate for F413, whose actual PID turn is unmodeled.
    rotate_reference_s = 2.5 * math.sqrt(2 * nominal_rotate_angle / (3 * rotate_alpha))
    dwell_s = 0.4 if family == "half" else 0.2
    return {
        "machine": machine, "family": family, "label": label,
        "cell_mm": cell,
        "search_speed_mm_s": speed,
        "known_speed_mm_s": math.sqrt(speed * speed + 2 * dash_accel * cell),
        "acceleration_mm_s2": acceleration,
        "dash_acceleration_mm_s2": dash_accel,
        "turn_alpha_deg_s2": run["alpha_turn90"],
        "turn_rounding": rounding,
        "turn_entry_mm": run["dist_offset_in"],
        "turn_exit_mm": run["dist_offset_out"],
        "turn_omega_cap_deg_s": omega_cap,
        "turn90_s": turn90_s,
        "turn90_angular_s": angular_turn_s,
        "uturn_s": rotate_reference_s + dwell_s,
        "uturn_dwell_s": dwell_s,
        "sensor_delay_s": 0.0,
        "entry_distance_mm": first_section + half_cell,
        "entry_speed_mm_s": entry_speed,
        "entry_known_speed_mm_s": math.sqrt(entry_speed ** 2 + 2 * dash_accel * cell),
        "source_refs": [run_path, params_path,
                        "platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_search_step.c"
                        if family == "half" else "platform/stm32f405/Core/Src/drive.c"],
        "notes": [
            "Defaults are extracted from searchRunParams[0] (mode 1 case 1).",
            "Nominal cruise uses slalom speed; firmware entry cruise is higher because initial acceleration covers first-section plus half-cell.",
            "Known-section maximum models the firmware's single-cell acceleration followed by cruise; it is not an independent firmware limit.",
            "Slalom time includes the cosine angular profile and entry/exit distances at nominal speed.",
            "U-turn time excludes braking/launch, wall alignment, sensor noise, processing, NVM writes and operator delays.",
            "F413 U-turn is measured-angle PID; its editable duration is an estimate using the spot-turn angular reference, not a hardware timing prediction."
            if family == "half" else
            "F405 U-turn estimate uses driveR angular thirds and one 200 ms wait; front/side alignment can add time.",
        ],
    }


def get_profiles(repo_root: Path | str | None = None) -> dict[str, dict]:
    return {machine: get_profile(machine, repo_root) for machine in _MACHINES}
