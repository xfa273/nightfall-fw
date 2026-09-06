from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from .app import Application, comparison_csv
from .engine import Knowledge, simulate
from .mazes import Maze
from .mcu_policy import EngineOracle, simulate_c
from .mcu_policy_ui import export_ui_replay
from .profiles import get_profile


class McuUIReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        profile = get_profile("mini_r2_0")
        maze = Maze(8, 8, Knowledge(8, 8).walls, (0, 0), [(3, 3)], {"id": "test_open"})
        cls.payload = {"maze": maze.to_dict(), "profile": profile,
                       "run": simulate_c(maze, profile, predictive=True, max_steps=512,
                                         policy_per_ms=64, oracle_per_ms=20)}
        cls.baseline = simulate(maze, profile, EngineOracle(), "baseline", max_steps=512)

    def test_ui_alias_preserves_every_timeline_field_and_actual_implementation(self):
        result = export_ui_replay(self.payload, self.baseline, source_max_steps=512)
        self.assertEqual([run["algorithm"] for run in result["runs"]], ["baseline", "relevant"])
        actual = result["runs"][1]
        self.assertEqual(actual["source_algorithm"], "c_predictive")
        self.assertEqual(self.payload["run"]["algorithm"], "c_predictive")
        self.assertEqual(actual["events"], self.payload["run"]["events"])
        self.assertEqual(actual["summary"], self.payload["run"]["summary"])
        self.assertIn("C 実装", actual["label"])
        self.assertEqual(result["metadata"]["replay_kind"], "mcu_finite_budget")
        self.assertEqual(result["options"]["policy_units_per_ms"], 64)
        self.assertEqual(result["options"]["oracle_units_per_ms"], 20)
        self.assertEqual(result["options"]["max_steps"], 512)
        self.assertFalse(result["options"]["pending_surrogate"])
        self.assertTrue(result["metadata"]["certificate_audit"]["passed"])
        self.assertTrue(any("固定版" in note for note in result["metadata"]["notes_ja"]))
        # JSON round-trip must preserve booleans/null bounds and numeric times.
        self.assertEqual(json.loads(json.dumps(result, allow_nan=False)), result)

    def test_inconsistent_time_is_rejected(self):
        bad = deepcopy(self.payload)
        bad["run"]["events"][1]["dt"] += 0.1
        with self.assertRaisesRegex(ValueError, "time/duration"):
            export_ui_replay(bad, self.baseline)

    def test_actual_application_loads_export_and_can_export_csv(self):
        result = export_ui_replay(self.payload, self.baseline, source_max_steps=512)
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            # Supply a local catalog fixture; this loader test needs no network.
            (directory / "16MM2014CX.maze").write_text(
                "+---+---+\n|     G |\n+   +   +\n| S     |\n+---+---+\n")
            replay = directory / "replay.json"
            replay.write_text(json.dumps(result, allow_nan=False))
            application = Application(maze_dir=directory, replay=replay)
            self.assertEqual(application.replay, result)
            self.assertEqual(application.replay["metadata"]["schema"], "nightfall_exploration_v1")
            self.assertIn("c_predictive", comparison_csv(application.replay))

    def test_future_map_and_certificate_cannot_be_backfilled(self):
        bad = deepcopy(self.payload)
        bad["run"]["events"][0]["known_edges"] += 1
        with self.assertRaisesRegex(ValueError, "known-edge"):
            export_ui_replay(bad, self.baseline)
        bad = deepcopy(self.payload)
        bad["run"]["summary"]["certificate_s"] = 0.0
        with self.assertRaisesRegex(ValueError, "certificate timestamp"):
            export_ui_replay(bad, self.baseline)


if __name__ == "__main__":
    unittest.main()
