"""Host checks including a parity oracle compiled from the firmware source."""

from __future__ import annotations

import math
import shutil
import subprocess
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

import turn_tune
from turn_dynamics import DynamicsModel, PerformanceLimits, predict


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src"
CONSTANTS = turn_tune.Constants(1.2, 2200.0)


def example_turn(angle: float = -90.0) -> turn_tune.TurnSpec:
    return turn_tune.TurnSpec("shortest", "example", angle, 4700.0, 500.0, 5.0, 15.0, {})


def extract_function(path: Path, declaration: str) -> str:
    source = path.read_text(encoding="utf-8")
    start = source.index(declaration)
    open_brace = source.index("{", start)
    depth = 1
    end = open_brace + 1
    while depth:
        if source[end] == "{":
            depth += 1
        elif source[end] == "}":
            depth -= 1
        end += 1
    return source[start:end]


class DynamicsTests(unittest.TestCase):
    def test_identity_matches_existing_reference_including_accelerating_offsets(self) -> None:
        for angle in (-45.0, 89.0, -133.3, 179.0):
            with self.subTest(angle=angle):
                turn = example_turn(angle)
                ideal = turn_tune.simulate_turn(turn, CONSTANTS, 300.0, 700.0)
                actual = predict(turn, CONSTANTS, entry_speed_mm_s=300.0, out_speed_mm_s=700.0, settle_time_s=0.0)
                np.testing.assert_allclose(actual.x_mm[1:], [sample.x_mm for sample in ideal.samples], atol=1e-9)
                np.testing.assert_allclose(actual.y_mm[1:], [sample.y_mm for sample in ideal.samples], atol=1e-9)
                np.testing.assert_allclose(actual.theta_deg[1:], [sample.theta_deg for sample in ideal.samples], atol=1e-9)

    def test_dynamic_mirror_and_yaw_area_after_settling(self) -> None:
        model = DynamicsModel(velocity_gain=0.95, velocity_tau_s=0.02, yaw_gain=1.03,
                              yaw_tau_s=0.015, yaw_delay_s=0.004, lateral_tau_s=0.012)
        right = predict(example_turn(-90.0), CONSTANTS, model=model, settle_time_s=0.5)
        left = predict(example_turn(90.0), CONSTANTS, model=model, settle_time_s=0.5)
        np.testing.assert_allclose(right.x_mm, -left.x_mm, atol=1e-10)
        np.testing.assert_allclose(right.y_mm, left.y_mm, atol=1e-10)
        np.testing.assert_allclose(right.theta_deg, -left.theta_deg, atol=1e-10)
        self.assertAlmostEqual(right.theta_deg[-1], -92.7, places=3)
        self.assertGreater(right.metrics["max_slip_deg"], 0.0)
        self.assertLess(abs(right.theta_deg[-1] - right.course_deg[-1]), 1e-5)

    def test_lateral_lag_changes_course_without_changing_body_yaw(self) -> None:
        ideal = predict(example_turn(), CONSTANTS, settle_time_s=0.0)
        lagged = predict(example_turn(), CONSTANTS, model=DynamicsModel(lateral_tau_s=0.03), settle_time_s=0.0)
        np.testing.assert_allclose(ideal.theta_deg, lagged.theta_deg)
        self.assertGreater(lagged.y_mm[-1], ideal.y_mm[-1])
        self.assertLess(lagged.x_mm[-1], ideal.x_mm[-1])

    def test_arbitrary_sample_times_use_complete_internal_metrics(self) -> None:
        full = predict(example_turn(), CONSTANTS)
        times = np.asarray([0.0, 0.0105, 0.1357, full.time_s[-1]])
        sampled = predict(example_turn(), CONSTANTS, times)
        np.testing.assert_allclose(sampled.x_mm, np.interp(times, full.time_s, full.x_mm))
        self.assertEqual(sampled.metrics, full.metrics)
        for bad in ([-0.001], [full.time_s[-1] + 0.001], [0.1, 0.0], [math.nan], []):
            with self.subTest(times=bad), self.assertRaises(ValueError):
                predict(example_turn(), CONSTANTS, bad)

    def test_invalid_physical_values_are_rejected(self) -> None:
        for field, value in (("velocity_gain", 0.0), ("yaw_tau_s", -0.1), ("yaw_gain", math.nan)):
            with self.subTest(field=field), self.assertRaises(ValueError):
                DynamicsModel(**{field: value})
        with self.assertRaises(ValueError):
            DynamicsModel.from_dict({"unrecognized": 1.0})
        for field, value in (("velocity_mm_s", 0.0), ("dist_in_mm", -1.0), ("alpha_deg_s2", math.inf), ("signed_angle_deg", 0.0)):
            with self.subTest(field=field), self.assertRaises(ValueError):
                predict(replace(example_turn(), **{field: value}), CONSTANTS)

    def test_limits_are_explicit_and_do_not_claim_unknown_margin(self) -> None:
        unknown = predict(example_turn(), CONSTANTS, settle_time_s=0.0)
        self.assertIsNone(unknown.metrics["limits_passed"])
        checked = predict(example_turn(), CONSTANTS, settle_time_s=0.0,
                          limits=PerformanceLimits(max_velocity_mm_s=400.0, max_lateral_accel_mm_s2=100000.0))
        self.assertFalse(checked.metrics["limits_passed"])
        self.assertEqual(checked.metrics["exceeded_limits"], ["max_velocity_mm_s"])
        self.assertEqual(checked.metrics["limit_ratios"]["max_velocity_mm_s"], 1.25)


class FirmwareSourceParityTests(unittest.TestCase):
    """Compile actual pure firmware functions; no HAL, firmware edits or hardware."""

    @classmethod
    def setUpClass(cls) -> None:
        compiler = shutil.which("cc")
        if compiler is None:
            raise unittest.SkipTest("C compiler unavailable for firmware profile parity")
        cls.directory = tempfile.TemporaryDirectory(prefix="nightfall-profile-parity-")
        directory = Path(cls.directory.name)
        source = directory / "parity.c"
        cls.binary = directory / "parity"
        prelude = """
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
static float test_rounding;
static float test_cap;
#define TURN_OMEGA_PROFILE_ROUNDING_SCALE test_rounding
typedef struct { float omega_peak_deg_s, t_acc_s, t_cruise_s, t_total_s; } f413_path_run_smooth_turn_t;
typedef f413_path_run_smooth_turn_t f413_search_step_smooth_turn_t;
static volatile float s_omega_profile_peak, s_omega_profile_t_acc;
static volatile float s_omega_profile_t_cruise, s_omega_profile_t_total;
"""
        functions = "\n".join((
            extract_function(SRC / "f413_path_run.c", "static f413_path_run_smooth_turn_t f413_path_run_build_smooth_turn("),
            extract_function(SRC / "f413_search_step.c", "static f413_search_step_smooth_turn_t f413_search_step_build_smooth_turn("),
            extract_function(SRC / "f413_control.c", "static float f413_ctrl_sample_omega_profile("),
        ))
        main = """
int main(int argc, char **argv) {
  if (argc != 8) return 2;
  float angle = atof(argv[1]), alpha = atof(argv[2]);
  test_cap = atof(argv[3]); test_rounding = atof(argv[4]);
  f413_path_run_smooth_turn_t p = atoi(argv[7])
      ? f413_search_step_build_smooth_turn(angle, alpha)
      : f413_path_run_build_smooth_turn(angle, alpha, test_cap);
  s_omega_profile_peak = (angle < 0 ? -1 : 1) * p.omega_peak_deg_s;
  s_omega_profile_t_acc = p.t_acc_s;
  s_omega_profile_t_cruise = p.t_cruise_s;
  s_omega_profile_t_total = p.t_total_s;
  float t = atof(argv[5]);
  if (atoi(argv[6])) t *= p.t_total_s;
  printf("%.9g %.9g %.9g %.9g %.9g\\n", p.omega_peak_deg_s, p.t_acc_s,
         p.t_cruise_s, p.t_total_s, f413_ctrl_sample_omega_profile(t));
  return 0;
}
"""
        source.write_text(prelude + functions + main, encoding="utf-8")
        subprocess.run([compiler, "-std=c99", "-O0", str(source), "-lm", "-o", str(cls.binary)],
                       check=True, capture_output=True, text=True)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.directory.cleanup()

    def test_profiles_and_samples_match_both_firmware_runners(self) -> None:
        # Includes omega cap, triangular fallback, minimum rounding and signs.
        cases = [(-90.0, 4700.0, 2200.0, 1.2), (45.0, 7700.0, 2200.0, 1.2),
                 (-180.0, 200000.0, 2200.0, 1.2), (135.0, 10000.0, 0.0, 2.0),
                 (90.0, 10000.0, 2200.0, 0.01)]
        for angle, alpha, cap, rounding in cases:
            for fraction in (0.0, 0.1, 0.49, 0.65, 0.95, 1.0, 1.1):
                for search in (0, 1):
                    # Search has no omega cap; shortest uses its mode parameter.
                    profile = turn_tune.build_profile(
                        replace(example_turn(angle), alpha_deg_s2=alpha),
                        turn_tune.Constants(rounding, 0.0 if search else cap))
                    with self.subTest(angle=angle, alpha=alpha, rounding=rounding, fraction=fraction, search=search):
                        args = [angle, alpha, cap, rounding, fraction, 1, search]
                        values = np.fromstring(subprocess.check_output([str(self.binary), *map(str, args)], text=True), sep=" ")
                        peak = math.copysign(profile.omega_peak_deg_s, angle)
                        expected = [profile.omega_peak_deg_s, profile.t_acc_s, profile.t_cruise_s,
                                    profile.t_total_s, turn_tune.sample_omega_deg_s(profile, peak, profile.t_total_s * fraction)]
                        np.testing.assert_allclose(values, expected, rtol=8e-6, atol=3e-4)


if __name__ == "__main__":
    unittest.main()
