import unittest

from .engine import Knowledge, simulate
from .mazes import Maze, load_maze
from .mcu_policy import EngineOracle, McuPolicy, _predict, simulate_c
from .profiles import get_profile


def open_maze(size=8, goals=((3, 3),)):
    knowledge = Knowledge(size, size)
    return Maze(size, size, knowledge.walls, (0, 0), list(goals), {"id": "synthetic_open"})


class McuPolicyReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = get_profile("mini_r2_0")

    def test_drain_replay_matches_python_route_and_certifies_observed_map(self):
        maze = open_maze()
        python = simulate(maze, self.profile, EngineOracle(), max_steps=512)
        actual = simulate_c(maze, self.profile, max_steps=512)
        route = lambda run: [(event["x"], event["y"], event["heading"]) for event in run["events"]]
        self.assertEqual(route(actual), route(python))
        self.assertTrue(actual["summary"]["certified"])
        self.assertTrue(actual["certificate_audit"]["passed"])
        self.assertEqual(actual["summary"]["fallback_steps"], 0)
        self.assertGreater(actual["summary"]["oracle_work_units"], 0)
        self.assertGreater(actual["summary"]["policy_work_units"], 0)

    def test_prediction_is_rejected_when_arrival_observes_a_wall(self):
        maze = open_maze(goals=((0, 3),))
        maze.walls[1][0] |= 1
        maze.walls[2][0] |= 4
        knowledge = Knowledge(8, 8)
        knowledge.observe(maze.walls, (0, 0), 0)
        policy = McuPolicy(8, 8, self.profile)
        try:
            status = policy.begin(knowledge, maze.goals, (0, 1), 0, predictive=True)
            while status == 4:
                status = policy.step(128)
            proposed = policy.result()
            self.assertEqual(proposed.status, 5)
            self.assertEqual(proposed.direction, 0)
            knowledge.observe(maze.walls, (0, 1), 0)
            applied = policy.apply(knowledge, maze.goals, (0, 1), 0, proposed)
            self.assertEqual(applied.status, 1)
            self.assertEqual(applied.reason, 10)
            self.assertEqual(applied.direction, 255)
            self.assertFalse(applied.certified)
        finally:
            policy.close()

    def test_arrival_rejects_changed_epoch(self):
        maze = open_maze()
        knowledge = Knowledge(8, 8)
        knowledge.observe(maze.walls, (0, 0), 0)
        policy = McuPolicy(8, 8, self.profile)
        try:
            decision = policy.decide(knowledge, maze.goals, (0, 0), 0)
            self.assertEqual(decision.status, 0)
            decision = policy.apply(knowledge, maze.goals, (0, 0), 0, decision, epoch=2)
            self.assertEqual(decision.status, 1)
            self.assertEqual(decision.reason, 5)
        finally:
            policy.close()

    def test_pending_exact_oracle_never_certifies(self):
        maze = open_maze()
        knowledge = Knowledge(8, 8)
        knowledge.observe(maze.walls, (3, 3), 0)
        policy = McuPolicy(8, 8, self.profile, drain=False, oracle_budget=1)
        try:
            policy.note_goal(knowledge, maze.goals, (3, 3), 0)
            decision = policy.decide(knowledge, maze.goals, (3, 3), 0)
            self.assertEqual(decision.status, 1)
            self.assertEqual(decision.reason, 3)
            self.assertFalse(decision.certified)
            self.assertEqual(policy.stats()["oracle_work_units"], 1)
        finally:
            policy.close()

    def test_predictive_replay_respects_work_limits_and_certificate(self):
        run = simulate_c(open_maze(), self.profile, predictive=True, max_steps=512)
        self.assertTrue(run["summary"]["certified"])
        self.assertTrue(run["certificate_audit"]["passed"])
        for event in run["events"]:
            schedule = event["schedule"]
            if schedule:
                self.assertLessEqual(schedule["policy_used"], schedule["policy_budget"])
                self.assertLessEqual(schedule["oracle_used"], schedule["oracle_budget"])

    def test_zero_scheduler_work_falls_back_without_false_certificate(self):
        maze = open_maze()
        baseline = simulate(maze, self.profile, EngineOracle(), "baseline", max_steps=512)
        run = simulate_c(maze, self.profile, predictive=True, max_steps=512,
                         policy_per_ms=0, oracle_per_ms=0)
        self.assertFalse(run["summary"]["certified"])
        self.assertEqual(run["summary"]["policy_work_units"], 0)
        self.assertEqual(run["summary"]["oracle_started"], 0)
        self.assertEqual(run["summary"]["steps"], baseline["summary"]["steps"])
        self.assertEqual(run["summary"]["reason"], "full_map_without_certificate")

    def test_surrogate_keeps_exact_oracle_running_and_never_grants_certificate(self):
        maze = open_maze()
        knowledge = Knowledge(8, 8)
        knowledge.observe(maze.walls, (0, 0), 0)
        knowledge.observe(maze.walls, (3, 3), 0)
        policy = McuPolicy(8, 8, self.profile, drain=False, oracle_budget=1, pending_surrogate=True)
        try:
            policy.note_goal(knowledge, maze.goals, (3, 3), 0)
            decision, schedule = _predict(policy, knowledge, maze, (3, 3), 0, 1, 10000, 200)
            self.assertEqual(decision.reason, 14)
            self.assertFalse(decision.certified)
            self.assertEqual(decision.lower_us, 0)
            self.assertEqual(schedule["oracle_used"], 200)
            self.assertGreater(policy.stats()["oracle_work_units"], policy.stats()["oracle_callbacks"])
        finally:
            policy.close()

    def test_full_size_workspace_and_first_observed_action(self):
        maze = open_maze(32, ((16, 16),))
        knowledge = Knowledge(32, 32)
        knowledge.observe(maze.walls, (0, 0), 0)
        policy = McuPolicy(32, 32, self.profile)
        try:
            decision = policy.decide(knowledge, maze.goals, (0, 0), 0)
            self.assertEqual(decision.status, 0)
            self.assertTrue(knowledge.open(0, 0, decision.direction))
            self.assertLessEqual(policy.stats()["oracle_workspace_bytes"], 224 * 1024)
            self.assertLessEqual(policy.stats()["policy_workspace_bytes"], 31 * 1024)
        finally:
            policy.close()

    def test_regression_short_windows_do_not_mix_policies_into_a_loop(self):
        try:
            maze = load_maze("32MM2023HX", download=False)
        except FileNotFoundError:
            self.skipTest("Pinned competition dataset is not cached; this test does not download")
        # Before the shared progress guard, this scenario changed no facts
        # after step1086 and continued oscillating through the6000-step cap.
        run = simulate_c(maze, self.profile, predictive=True, max_steps=1800,
                         policy_per_ms=64, oracle_per_ms=50, pending_surrogate=True)
        self.assertTrue(run["summary"]["certified"])
        self.assertTrue(run["certificate_audit"]["passed"])
        self.assertGreater(run["summary"]["progress_recovery_steps"], 0)
        self.assertLess(run["summary"]["steps"], 1800)


if __name__ == "__main__":
    unittest.main()
