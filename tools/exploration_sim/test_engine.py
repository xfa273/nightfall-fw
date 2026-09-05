"""Small graph tests independent of the compiled competition route solver."""
from collections import deque
import copy
import unittest

from tools.exploration_sim.engine import Knowledge, apply_timing, linear_time, simulate
from tools.exploration_sim.app import configure
from tools.exploration_sim.mazes import Maze
from tools.exploration_sim.profiles import get_profile


DX = (0, 1, 0, -1)
DY = (1, 0, -1, 0)


def make_grid(width=4, height=4, goals=None):
    walls = [[0] * width for _ in range(height)]
    for y in range(height):
        for x in range(width):
            for d in range(4):
                if not (0 <= x + DX[d] < width and 0 <= y + DY[d] < height):
                    walls[y][x] |= 1 << d
    return Maze(width, height, walls, (0, 0), goals or [(width - 1, height - 1)])


def add_wall(maze, x, y, d):
    maze.walls[y][x] |= 1 << d
    nx, ny = x + DX[d], y + DY[d]
    if 0 <= nx < maze.width and 0 <= ny < maze.height:
        maze.walls[ny][nx] |= 1 << ((d + 2) % 4)


class GraphOracle:
    """Unit-cost shortest route plus its complete required-open edge set.

    This deliberately has no reference to ground truth or robot knowledge;
    the only graph it receives is the map supplied by the engine.
    """
    def __init__(self):
        self.calls = []

    def solve(self, walls, goals, details=True):
        self.calls.append(copy.deepcopy(walls))
        width, height = len(walls[0]), len(walls)
        goal_set = set(map(tuple, goals))
        queue, parent = deque([(0, 0)]), {(0, 0): None}
        end = None
        while queue:
            x, y = queue.popleft()
            if (x, y) in goal_set:
                end = (x, y)
                break
            for d in range(4):
                nx, ny = x + DX[d], y + DY[d]
                if walls[y][x] & (1 << d):
                    continue
                if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in parent:
                    parent[nx, ny] = (x, y)
                    queue.append((nx, ny))
        if end is None:
            return {"status": "no_path", "required_edges": [], "route_points": []}
        route = [end]
        while parent[route[-1]] is not None:
            route.append(parent[route[-1]])
        route.reverse()
        required = []
        for (x, y), (nx, ny) in zip(route, route[1:]):
            if ny != y:
                required.append([x, min(y, ny), 0])
            else:
                required.append([min(x, nx), y, 1])
        return {"status": "ok", "goal_entry_s": float(len(route) - 1),
                "required_edges": required,
                "route_points": [list(pos) for pos in route]}


class InfeasibleOracle:
    def solve(self, walls, goals, details=True):
        return {"status": "no_path", "required_edges": [], "route_points": []}


class KnowledgeTests(unittest.TestCase):
    def test_front_side_observation_is_symmetric_and_does_not_see_through(self):
        maze = make_grid()
        add_wall(maze, 1, 1, 1)
        knowledge = Knowledge(4, 4)
        changes = knowledge.observe(maze.walls, (1, 1), 0)
        self.assertEqual(len(changes), 3)
        self.assertEqual(knowledge.known[1][1], 1 | 2 | 8)
        self.assertFalse(knowledge.known[1][1] & 4)  # Rear was not sensed.
        self.assertTrue(knowledge.walls[1][2] & 8)
        self.assertTrue(knowledge.open(1, 1, 0))
        self.assertTrue(knowledge.open(1, 2, 2))
        self.assertFalse(knowledge.known[2][1] & 1)  # No seeing past adjacent edge.
        self.assertEqual(knowledge.observe(maze.walls, (1, 1), 0), [])

    def test_unknown_edges_are_open_only_in_lower_bound_map(self):
        knowledge = Knowledge(4, 4)
        self.assertEqual(knowledge.map_for(True)[1][1], 0)
        self.assertEqual(knowledge.map_for(False)[1][1], 15)
        knowledge.set_edge(1, 1, 0, False)
        self.assertFalse(knowledge.map_for(False)[1][1] & 1)
        self.assertFalse(knowledge.map_for(False)[2][1] & 4)


class TimingTests(unittest.TestCase):
    def test_triangular_and_trapezoidal_reference_integrals(self):
        self.assertAlmostEqual(linear_time(100, 0, 0, 1000, 100), 2.0)
        self.assertAlmostEqual(linear_time(1000, 0, 0, 100, 100), 11.0)
        self.assertAlmostEqual(linear_time(90, 300, 300, 300, 1000), 0.3)

    def test_straight_run_carries_velocity_and_stops_at_end(self):
        profile = get_profile("mini_r2_0")
        events = [{"turn": 0, "known_straight": False} for _ in range(5)]
        duration = apply_timing(events, profile)
        # 4*90mm at 300mm/s plus 0.15s acceleration and 0.15s braking loss.
        self.assertAlmostEqual(duration, 1.5)
        self.assertAlmostEqual(sum(event.get("dt", 0) for event in events), duration)
        self.assertTrue(all(a["t"] < b["t"] for a, b in zip(events, events[1:])))

    def test_back_turn_has_braking_and_restart_translation(self):
        profile = get_profile("mini_r2_0")
        events = [{"turn": 0, "known_straight": False},
                  {"turn": 0, "known_straight": False},
                  {"turn": 2, "known_straight": False},
                  {"turn": 0, "known_straight": False}]
        apply_timing(events, profile)
        # Nominal 300mm/s stops in exactly half of a 90mm cell at 1000mm/s².
        # The backward action includes 0.3s braking + 0.3s launch, beyond spin.
        self.assertAlmostEqual(events[2]["dt"], profile["uturn_s"] + 0.6)


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.maze = make_grid()
        self.maze.metadata["family"] = "half"

    def test_rejects_speed_that_cannot_stop_within_half_cell(self):
        with self.assertRaisesRegex(ValueError, "半区画"):
            configure({"profile": {"acceleration_mm_s2": 100}}, self.maze)

    def test_rejects_nonfinite_boolean_and_unknown_profile_inputs(self):
        for value in (float("nan"), float("inf"), True, "not-a-number"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                configure({"profile": {"search_speed_mm_s": value}}, self.maze)
        with self.assertRaisesRegex(ValueError, "Unknown editable"):
            configure({"profile": {"cell_mm": 180}}, self.maze)

    def test_rejects_incompatible_machine_and_fractional_shortest_case(self):
        with self.assertRaises(ValueError):
            configure({"machine": "classic_r1_0"}, self.maze)
        with self.assertRaisesRegex(ValueError, "integers"):
            configure({"shortest_case": 1.5}, self.maze)
        with self.assertRaises(ValueError):
            configure({"return_home": "false"}, self.maze)

    def test_turn_time_recalculates_unless_explicitly_measured(self):
        original, _ = configure({}, self.maze)
        altered, _ = configure({"profile": {"turn_alpha_deg_s2": 5000}}, self.maze)
        measured, _ = configure({"profile": {"turn_alpha_deg_s2": 5000, "turn90_s": 0.42}}, self.maze)
        self.assertGreater(altered["turn90_s"], original["turn90_s"])
        self.assertEqual(measured["turn90_s"], 0.42)


class SimulationTests(unittest.TestCase):
    def setUp(self):
        self.profile = get_profile("mini_r2_0")

    def assert_legal_replay(self, maze, result):
        for before, after in zip(result["events"], result["events"][1:]):
            d = after["heading"]
            self.assertEqual((after["x"], after["y"]),
                             (before["x"] + DX[d], before["y"] + DY[d]))
            self.assertFalse(maze.walls[before["y"]][before["x"]] & (1 << d))
            self.assertGreater(after["t"], before["t"])
        for event in result["events"]:
            if event["lower_s"] is not None and event["upper_s"] is not None:
                self.assertLessEqual(event["lower_s"], event["upper_s"] + 1e-9)

    def test_hidden_truth_cannot_change_decisions_or_oracle_inputs(self):
        original = make_grid()
        changed = copy.deepcopy(original)
        add_wall(changed, 3, 2, 0)  # Beyond all observations made by step 1.
        for algorithm in ("baseline", "relevant"):
            first_oracle, second_oracle = GraphOracle(), GraphOracle()
            first = simulate(original, self.profile, first_oracle, algorithm, max_steps=1)
            second = simulate(changed, self.profile, second_oracle, algorithm, max_steps=1)
            self.assertEqual(first["events"], second["events"], algorithm)
            self.assertEqual(first_oracle.calls, second_oracle.calls, algorithm)

    def test_both_policies_avoid_barriers_and_return_home(self):
        maze = make_grid()
        add_wall(maze, 0, 0, 1)
        add_wall(maze, 1, 1, 0)
        add_wall(maze, 1, 2, 1)
        add_wall(maze, 2, 2, 0)
        for algorithm in ("baseline", "relevant"):
            result = simulate(maze, self.profile, GraphOracle(), algorithm,
                              return_home=True, max_steps=150)
            self.assert_legal_replay(maze, result)
            self.assertTrue(result["summary"]["completed"], result["summary"])
            self.assertTrue(result["summary"]["certified"])
            self.assertIsNotNone(result["summary"]["home_s"])
            self.assertEqual((result["events"][-1]["x"], result["events"][-1]["y"]), maze.start)

    def test_new_wall_on_optimistic_route_invalidates_lower_bound(self):
        maze = make_grid(3, 3, [(0, 2)])
        add_wall(maze, 0, 1, 0)
        oracle = GraphOracle()
        result = simulate(maze, self.profile, oracle, "relevant", max_steps=80)
        self.assertEqual(result["events"][0]["lower_s"], 2.0)
        self.assertEqual(result["summary"]["lower_s"], 4.0)
        self.assertEqual(result["summary"]["upper_s"], 4.0)
        self.assertTrue(result["summary"]["certified"])
        self.assertGreaterEqual(len(oracle.calls), 2)
        self.assert_legal_replay(maze, result)

    def test_relevant_policy_can_certify_without_visiting_whole_maze(self):
        maze = make_grid(4, 4)
        relevant = simulate(maze, self.profile, GraphOracle(), "relevant", max_steps=100)
        baseline = simulate(maze, self.profile, GraphOracle(), "baseline", max_steps=100)
        self.assertEqual(relevant["summary"]["reason"], "route_certified")
        self.assertTrue(relevant["summary"]["certified"])
        self.assertLess(relevant["summary"]["visited_cells"], 16)
        self.assertEqual(baseline["summary"]["visited_cells"], 16)
        self.assertLess(relevant["summary"]["steps"], baseline["summary"]["steps"])
        self.assertEqual(relevant["summary"]["lower_s"], baseline["summary"]["lower_s"])

    def test_first_goal_path_is_shared_by_both_policies(self):
        maze = make_grid(4, 4)
        add_wall(maze, 0, 1, 0)
        add_wall(maze, 2, 1, 1)
        runs = [simulate(maze, self.profile, GraphOracle(), algorithm, max_steps=100)
                for algorithm in ("baseline", "relevant")]
        paths = []
        for result in runs:
            path = []
            for event in result["events"]:
                path.append((event["x"], event["y"], event["heading"]))
                if (event["x"], event["y"]) in maze.goals:
                    break
            paths.append(path)
        self.assertEqual(paths[0], paths[1])

    def test_return_home_step_limit_cannot_report_complete(self):
        maze = make_grid(1, 3)
        for algorithm in ("baseline", "relevant"):
            result = simulate(maze, self.profile, GraphOracle(), algorithm,
                              return_home=True, max_steps=3)
            self.assertFalse(result["summary"]["completed"], result["summary"])
            self.assertEqual(result["summary"]["reason"], "step_limit")
            self.assertIsNone(result["summary"]["home_s"])

    def test_infeasible_shortest_model_finishes_discovery_without_certificate(self):
        maze = make_grid(1, 3)
        result = simulate(maze, self.profile, InfeasibleOracle(), "relevant", max_steps=20)
        self.assertTrue(result["summary"]["completed"])
        self.assertEqual(result["summary"]["reason"], "no_feasible_route")
        self.assertFalse(result["summary"]["certified"])
        self.assertEqual(result["summary"]["visited_cells"], 3)

    def test_infeasible_shortest_model_still_honors_requested_return(self):
        maze = make_grid(1, 3)
        result = simulate(maze, self.profile, InfeasibleOracle(), "relevant",
                          return_home=True, max_steps=20)
        self.assertTrue(result["summary"]["completed"])
        self.assertFalse(result["summary"]["certified"])
        self.assertIsNotNone(result["summary"]["home_s"])
        self.assertEqual((result["events"][-1]["x"], result["events"][-1]["y"]), maze.start)

    def test_full_exploration_terminates_with_enclosed_unvisited_cell(self):
        maze = make_grid(3, 3, [(0, 2)])
        for d in range(4):
            add_wall(maze, 1, 1, d)
        result = simulate(maze, self.profile, GraphOracle(), "baseline", max_steps=100)
        self.assertEqual(result["summary"]["reason"], "all_reachable_visited")
        self.assertEqual(result["summary"]["visited_cells"], 8)
        self.assert_legal_replay(maze, result)


if __name__ == "__main__":
    unittest.main()
