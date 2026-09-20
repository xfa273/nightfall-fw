"""Identification tests use held-out parameter conditions, not self-fit scores."""
import json
from dataclasses import asdict
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import turn_simulator as sim
from turn_dynamics import DynamicsModel, predict
from turn_tune import Constants, TurnSpec


def synthetic_run(alpha, index, model):
    spec = TurnSpec("manual", "synthetic-r90", -90, alpha, 300, 0, 0, {})
    constants = Constants(1.2, 2200)
    prediction = predict(spec, constants, model=model, settle_time_s=0.08)
    take = np.arange(0, len(prediction.time_s), 5)
    return SimpleNamespace(id=f"a{alpha}-repeat{index}", group_id=f"alpha{alpha}", spec=spec,
                           constants=constants, time_s=prediction.time_s[take],
                           x_mm=prediction.x_mm[take], y_mm=prediction.y_mm[take],
                           theta_deg=prediction.theta_deg[take].tolist(), diagnostic_only=True,
                           provenance={"machine": "synthetic", "original_params": {"dist_in_mm": 5, "dist_out_mm": 8}},
                           warnings=[])


class SimulatorTests(unittest.TestCase):
    def test_identification_predicts_unseen_alpha(self):
        truth = DynamicsModel(velocity_gain=0.97, lateral_tau_s=0.016)
        train = [synthetic_run(a, 0, truth) for a in (7000, 11000)]
        held = [synthetic_run(9000, 0, truth)]
        fitted, details = sim.fit_model(train, ("velocity_gain", "lateral_tau_s"))
        self.assertTrue(details["converged"])
        self.assertAlmostEqual(fitted.velocity_gain, truth.velocity_gain, places=3)
        self.assertAlmostEqual(fitted.lateral_tau_s, truth.lateral_tau_s, places=3)
        self.assertLess(sim.evaluate_runs(held, fitted)["rmse_mm"], 0.05)
        self.assertGreater(sim.evaluate_runs(held, DynamicsModel())["rmse_mm"], 1)

    def test_cross_validation_never_splits_repeats(self):
        truth = DynamicsModel(velocity_gain=0.98)
        dataset = SimpleNamespace(runs=[synthetic_run(a, i, truth) for a in (7000, 9000, 11000) for i in (0, 1)],
                                  warnings=[], exclusions=[])
        artifact = sim.calibrate(dataset, vary=("velocity_gain",))
        self.assertEqual(len(artifact["validation_folds"]), 3)
        for fold in artifact["validation_folds"]:
            self.assertNotIn(fold["held_out_group"], fold["training_groups"])
            self.assertEqual(len(fold["predicted"]["runs"]), 2)
        self.assertTrue(artifact["metrics"]["held_out_improves_ideal"])
        self.assertFalse(artifact["qualification"]["safety_qualified"])

    def test_tuner_improves_target_without_changing_speed_or_angle(self):
        p = dict(velocity=300, alpha=10000, angle=-90, dist_in=3, dist_out=3,
                 target_x=45, target_y=45, target_theta=-90)
        result = sim.tune(p)
        self.assertLess(result["metrics"]["endpoint_error_mm"], 0.5)
        self.assertLess(result["metrics"]["endpoint_error_mm"], result["optimization"]["initial_error_mm"])
        self.assertEqual(result["parameters"]["velocity"], p["velocity"])
        self.assertEqual(result["parameters"]["angle"], p["angle"])
        self.assertFalse(result["metrics"]["safe_recommendation_available"])

    def test_invalid_and_out_of_scope_parameters(self):
        p = dict(velocity=300, alpha=10000, angle=-90, dist_in=8, dist_out=8)
        for change in ({"velocity": float("nan")}, {"alpha": 0}, {"dist_in": -1}, {"angle": 0}):
            with self.assertRaises(ValueError):
                sim.simulation(p | change)
        dataset = SimpleNamespace(runs=[synthetic_run(10000, 0, DynamicsModel())], warnings=[], exclusions=[])
        artifact = sim.calibrate(dataset, vary=("velocity_gain",))
        response = sim.simulation(p | {"velocity": 500}, artifact)
        self.assertFalse(response["metrics"]["within_observed_bounds"])
        self.assertTrue(any("velocity" in w for w in response["warnings"]))

    def test_cli_output_is_strict_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "prediction.json"
            result = subprocess.run([sys.executable, str(Path(sim.__file__)), "simulate", "--output", str(path)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            output = json.loads(path.read_text())
            self.assertGreater(len(output["predicted"]), 100)
            self.assertFalse(output["metrics"]["safe_recommendation_available"])

    def test_xy_only_cannot_identify_yaw_response(self):
        run = synthetic_run(9000, 0, DynamicsModel())
        run.theta_deg = None
        with self.assertRaisesRegex(ValueError, "trusted body-heading"):
            sim.fit_model([run], ("yaw_tau_s", "lateral_tau_s"))

    def test_tuner_preserves_zero_entry_and_fixed_coverage(self):
        parameters = dict(velocity=300, alpha=9000, angle=-90, dist_in=0, dist_out=8,
                          target_x=45, target_y=45, target_theta=-90)
        artifact = {"coefficients": {}, "constants": asdict(Constants(1.2, 2200)),
                    "coverage": {k: [parameters[k]]*2 for k in sim.PARAM_FIELDS},
                    "regime": "zero-entry", "heading_identified": False}
        response = sim.tune(parameters, artifact)
        self.assertEqual(response["parameters"]["dist_in"], 0)
        self.assertTrue(response["metrics"]["within_observed_bounds"])
        self.assertEqual(response["parameters"]["dist_out"], 8)

    def test_explicit_limits_and_exact_export_values(self):
        p = dict(velocity=500, alpha=9750, angle=-135, dist_in=4.0001, dist_out=17.5,
                 limit_lateral_accel=100)
        response = sim.simulation(p)
        self.assertFalse(response["metrics"]["limits_passed"])
        assignments = response["assignments"]
        exported = float(assignments.split("dist_in = ")[1].split(";")[0])
        self.assertEqual(exported, p["dist_in"])

    def test_truncated_turn_is_not_training_evidence(self):
        run = synthetic_run(9000, 0, DynamicsModel())
        run.time_s = run.time_s[:8]
        dataset = SimpleNamespace(runs=[run], warnings=[], exclusions=[])
        with self.assertRaisesRegex(ValueError, "No usable"):
            sim.select_runs(dataset)


if __name__ == "__main__":
    unittest.main()
