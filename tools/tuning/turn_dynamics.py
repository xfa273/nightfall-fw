"""Empirical dynamics on the existing F413 1 kHz turn reference.

All lengths are millimetres, time is seconds, and public angles are degrees.
The coordinate frame is shared with ``turn_tune``: +x right, +y forward,
and positive yaw turns left.  This is a reduced response model, not a motor,
traction, wall sensor, or closed-loop firmware simulation.

The response laws are ``tau_v * v' + v = velocity_gain * v_ref`` and
``tau_w * omega' + omega = yaw_gain * omega_ref(t - yaw_delay_s)``.
Heading integrates omega; the travel direction follows heading through
``lateral_tau_s * course' + course = heading``.  The latter represents an
effective sideslip/trajectory lag, and is not a measured tire friction law.
Yaw lag and course lag can be hard to distinguish from positions alone.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, fields
from functools import lru_cache
from typing import Any, Mapping, Sequence

import numpy as np

try:
    from . import turn_tune
except ImportError:  # Direct execution/import with tools/tuning on sys.path.
    import turn_tune


@dataclass(frozen=True)
class DynamicsModel:
    velocity_gain: float = 1.0
    velocity_tau_s: float = 0.0
    yaw_gain: float = 1.0
    yaw_tau_s: float = 0.0
    yaw_delay_s: float = 0.0
    lateral_tau_s: float = 0.0

    def __post_init__(self) -> None:
        for field in fields(self):
            value = getattr(self, field.name)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{field.name} must be finite")
            low, high = (0.05, 3.0) if field.name.endswith("gain") else (0.0, 1.0)
            if not low <= value <= high:
                raise ValueError(f"{field.name} must be in [{low}, {high}]")

    @classmethod
    def from_dict(cls, values: Mapping[str, Any]) -> DynamicsModel:
        unknown = set(values) - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError(f"unknown dynamics fields: {', '.join(sorted(unknown))}")
        return cls(**values)

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class PerformanceLimits:
    """Optional independently established limits; none are assumed by default."""

    max_velocity_mm_s: float | None = None
    max_omega_deg_s: float | None = None
    max_alpha_deg_s2: float | None = None
    max_lateral_accel_mm_s2: float | None = None

    def __post_init__(self) -> None:
        for field in fields(self):
            value = getattr(self, field.name)
            if value is not None and (not math.isfinite(value) or value <= 0.0):
                raise ValueError(f"{field.name} must be finite and positive")


@dataclass(frozen=True)
class Prediction:
    time_s: np.ndarray
    x_mm: np.ndarray
    y_mm: np.ndarray
    theta_deg: np.ndarray
    course_deg: np.ndarray
    velocity_mm_s: np.ndarray
    omega_deg_s: np.ndarray
    velocity_ref_mm_s: np.ndarray
    omega_ref_deg_s: np.ndarray
    phase: np.ndarray
    command_duration_s: float
    model: DynamicsModel
    metrics: dict[str, Any]

    def to_dict(self, include_samples: bool = True) -> dict[str, Any]:
        result: dict[str, Any] = {
            "model": self.model.to_dict(),
            "command_duration_s": self.command_duration_s,
            "metrics": self.metrics,
        }
        if include_samples:
            result["samples"] = [
                {
                    "time_s": float(self.time_s[index]),
                    "x_mm": float(self.x_mm[index]),
                    "y_mm": float(self.y_mm[index]),
                    "theta_deg": float(self.theta_deg[index]),
                    "course_deg": float(self.course_deg[index]),
                    "velocity_mm_s": float(self.velocity_mm_s[index]),
                    "omega_deg_s": float(self.omega_deg_s[index]),
                    "velocity_ref_mm_s": float(self.velocity_ref_mm_s[index]),
                    "omega_ref_deg_s": float(self.omega_ref_deg_s[index]),
                    "phase": str(self.phase[index]),
                }
                for index in range(len(self.time_s))
            ]
        return result


def _validate_turn(turn: turn_tune.TurnSpec, constants: turn_tune.Constants,
                   entry_speed: float, out_speed: float) -> None:
    for name in ("signed_angle_deg", "alpha_deg_s2", "velocity_mm_s", "dist_in_mm", "dist_out_mm"):
        if not math.isfinite(getattr(turn, name)):
            raise ValueError(f"{name} must be finite")
    if not 0.0 < abs(turn.signed_angle_deg) <= 360.0:
        raise ValueError("absolute turn angle must be in (0, 360] degrees")
    if turn.alpha_deg_s2 <= 0.0 or turn.velocity_mm_s <= 0.0:
        raise ValueError("turn alpha and velocity must be positive")
    if turn.dist_in_mm < 0.0 or turn.dist_out_mm < 0.0:
        raise ValueError("turn offsets must be nonnegative")
    if not math.isfinite(entry_speed) or not math.isfinite(out_speed) or min(entry_speed, out_speed) < 0.0:
        raise ValueError("entry and exit speed must be finite and nonnegative")
    if not math.isfinite(constants.rounding_scale) or not math.isfinite(constants.omega_cap_deg_s):
        raise ValueError("profile constants must be finite")
    profile = turn_tune.build_profile(turn, constants)
    estimate = profile.t_total_s
    estimate += 2.0 * turn.dist_in_mm / (entry_speed + turn.velocity_mm_s)
    estimate += 2.0 * turn.dist_out_mm / (out_speed + turn.velocity_mm_s)
    if estimate > 30.0:
        raise ValueError("isolated-turn reference exceeds 30 seconds")


@lru_cache(maxsize=128)
def _reference(runner: str, angle: float, alpha: float, velocity: float,
               dist_in: float, dist_out: float, constants: turn_tune.Constants,
               entry_speed: float, out_speed: float) -> tuple[tuple[float, ...], tuple[float, ...], tuple[str, ...]]:
    turn = turn_tune.TurnSpec(runner, "reference", angle, alpha, velocity, dist_in, dist_out, {})
    ideal = turn_tune.simulate_turn(turn, constants, entry_speed, out_speed)
    return (
        tuple(sample.velocity_mm_s for sample in ideal.samples),
        tuple(sample.omega_deg_s for sample in ideal.samples),
        tuple(sample.phase for sample in ideal.samples),
    )


def _response(previous: float, target: float, tau: float, dt: float) -> tuple[float, float]:
    """Exact endpoint and interval mean for a held first-order input."""
    if tau <= 0.0:
        return target, target
    decay = math.exp(-dt / tau)
    mean_factor = -math.expm1(-dt / tau) * tau / dt
    return target + (previous - target) * decay, target + (previous - target) * mean_factor


def predict(
    turn: turn_tune.TurnSpec,
    constants: turn_tune.Constants,
    sample_times_s: Sequence[float] | np.ndarray | None = None,
    *,
    model: DynamicsModel = DynamicsModel(),
    entry_speed_mm_s: float | None = None,
    out_speed_mm_s: float | None = None,
    settle_time_s: float = 0.15,
    limits: PerformanceLimits | None = None,
) -> Prediction:
    """Predict an isolated turn, then continue at its exit speed while settling.

    t=0 is the start of the in-offset segment, not the first visible video
    frame. Initial velocity is the steady response to entry speed, and initial
    yaw rate/heading/course are zero. Zero gains/lags corrections reproduce
    the ideal trajectory (unit gains, all time constants zero).

    Requested sample times must be ordered and inside the simulated interval;
    no clipping, arbitrary alignment, or endpoint extrapolation is performed.
    Returned metrics always describe the full internal 1 kHz prediction.
    """
    entry_speed = turn.velocity_mm_s if entry_speed_mm_s is None else float(entry_speed_mm_s)
    exit_speed = (entry_speed if turn.runner == "search" else turn.velocity_mm_s) if out_speed_mm_s is None else float(out_speed_mm_s)
    _validate_turn(turn, constants, entry_speed, exit_speed)
    if not math.isfinite(settle_time_s) or not 0.0 <= settle_time_s <= 10.0:
        raise ValueError("settle_time_s must be finite and in [0, 10]")
    velocity_commands, omega_commands, phase_commands = _reference(
        turn.runner, turn.signed_angle_deg, turn.alpha_deg_s2, turn.velocity_mm_s,
        turn.dist_in_mm, turn.dist_out_mm, constants, entry_speed, exit_speed,
    )
    command_count = len(velocity_commands)
    dt = turn_tune.DT_S
    settle_count = int(math.ceil(settle_time_s / dt - 1e-12))
    count = command_count + settle_count
    time_s = np.arange(count + 1, dtype=float) * dt
    command_duration_s = command_count * dt
    velocity_ref = np.asarray(velocity_commands + (exit_speed,) * settle_count, dtype=float)
    omega_ref = np.asarray(omega_commands + (0.0,) * settle_count, dtype=float)
    phase = np.asarray(("start",) + phase_commands + ("settle",) * settle_count)
    if model.yaw_delay_s:
        delayed_time = time_s[:-1] - model.yaw_delay_s
        omega_input = np.interp(delayed_time, time_s[:-1], omega_ref, left=0.0, right=0.0)
    else:
        omega_input = omega_ref

    x = np.zeros(count + 1)
    y = np.zeros(count + 1)
    heading = np.zeros(count + 1)
    course = np.zeros(count + 1)
    velocity = np.zeros(count + 1)
    omega = np.zeros(count + 1)
    velocity[0] = entry_speed * model.velocity_gain
    course_rates = np.zeros(count)
    mean_velocities = np.zeros(count)
    for index in range(count):
        velocity[index + 1], mean_velocity = _response(
            float(velocity[index]), float(velocity_ref[index]) * model.velocity_gain, model.velocity_tau_s, dt,
        )
        omega[index + 1], mean_omega = _response(
            float(omega[index]), float(omega_input[index]) * model.yaw_gain, model.yaw_tau_s, dt,
        )
        heading[index + 1] = heading[index] + math.radians(mean_omega) * dt
        if model.lateral_tau_s <= 0.0:
            course[index + 1] = heading[index + 1]
        else:
            tau = model.lateral_tau_s
            heading_rate = math.radians(mean_omega)
            course[index + 1] = (
                heading[index + 1] - heading_rate * tau
                + (course[index] - heading[index] + heading_rate * tau) * math.exp(-dt / tau)
            )
        course_rate = (course[index + 1] - course[index]) / dt
        pose = turn_tune.advance_pose(
            turn_tune.Pose(float(x[index]), float(y[index]), float(course[index])),
            mean_velocity, math.degrees(float(course_rate)), dt,
        )
        x[index + 1], y[index + 1] = pose.x_mm, pose.y_mm
        course_rates[index] = course_rate
        mean_velocities[index] = mean_velocity

    metrics: dict[str, Any] = {
        "duration_s": float(time_s[-1]),
        "command_duration_s": command_duration_s,
        "endpoint": {"x_mm": float(x[-1]), "y_mm": float(y[-1]), "theta_deg": math.degrees(float(heading[-1]))},
        "max_velocity_mm_s": float(np.max(np.abs(velocity))),
        "max_omega_deg_s": float(np.max(np.abs(omega))),
        "max_alpha_deg_s2": float(np.max(np.abs(np.diff(omega) / dt))),
        "max_lateral_accel_mm_s2": float(np.max(np.abs(mean_velocities * course_rates))),
        "max_reference_lateral_accel_mm_s2": float(np.max(np.abs(velocity_ref * np.radians(omega_ref)))),
        "max_slip_deg": float(np.max(np.abs(np.degrees(heading - course)))),
    }
    ratios: dict[str, float] = {}
    if limits is not None:
        for field in fields(limits):
            limit = getattr(limits, field.name)
            if limit is not None:
                ratios[field.name] = metrics[field.name] / limit
    metrics["limit_ratios"] = ratios
    metrics["limits_checked"] = bool(ratios)
    metrics["limits_passed"] = all(ratio <= 1.0 for ratio in ratios.values()) if ratios else None
    metrics["exceeded_limits"] = [name for name, ratio in ratios.items() if ratio > 1.0]

    arrays = [x, y, np.degrees(heading), np.degrees(course), velocity, omega,
              np.concatenate(([entry_speed], velocity_ref)), np.concatenate(([0.0], omega_ref))]
    if sample_times_s is not None:
        requested = np.asarray(sample_times_s, dtype=float)
        if requested.ndim != 1 or requested.size == 0 or not np.all(np.isfinite(requested)):
            raise ValueError("sample times must be a nonempty finite one-dimensional array")
        if np.any(np.diff(requested) < 0.0):
            raise ValueError("sample times must be ordered")
        if requested[0] < 0.0 or requested[-1] > time_s[-1]:
            raise ValueError(f"sample times must be inside [0, {time_s[-1]:.6f}] seconds")
        arrays = [np.interp(requested, time_s, array) for array in arrays]
        phase = phase[np.searchsorted(time_s, requested, side="right").clip(1, len(time_s)) - 1]
        time_s = requested.copy()
    return Prediction(time_s, *arrays, phase, command_duration_s, model, metrics)
