"""Meaningful host oracle checks: exact memoization and edge certificates."""
from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
import ctypes
import json
import unittest

from tools.exploration_sim.oracle import TimeOracle


def open_maze(n):
    walls = [[0] * n for _ in range(n)]
    for x in range(n):
        walls[0][x] |= 4
        walls[n - 1][x] |= 1
    for y in range(n):
        walls[y][0] |= 8
        walls[y][n - 1] |= 2
    set_edge(walls, 0, 0, 1, True)
    return walls


def set_edge(walls, x, y, direction, wall):
    dx, dy = ((0, 1), (1, 0), (0, -1), (-1, 0))[direction]
    for xx, yy, d in ((x, y, direction), (x + dx, y + dy, (direction + 2) % 4)):
        if wall:
            walls[yy][xx] |= 1 << d
        else:
            walls[yy][xx] &= 15 ^ (1 << d)


def raw_solve(oracle, walls, goals, disable_memo=False):
    n = len(walls)
    data = ctypes.c_uint8 * (n * n)
    target = [0] * (n * n)
    for x, y in goals:
        target[y * n + x] = 1
    output = ctypes.create_string_buffer(2_000_000)
    result = oracle.lib.nf_exploration_oracle(n, n,
        data(*(x for row in walls for x in row)), data(*target), 0, 0, 0,
        int(oracle.planner == "slalom"), oracle.mode, oracle.case,
        1 | (2 if disable_memo else 0), output, len(output))
    assert result == 0
    return json.loads(output.value)


class OracleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mini = TimeOracle("mini_r2_0")
        cls.classic = TimeOracle("classic_r1_0")

    def test_replay_memoization_preserves_entire_route_and_required_edges(self):
        for n, goal in ((4, (2, 2)), (6, (4, 3)), (6, (2, 4))):
            walls = open_maze(n)
            if n == 6:
                set_edge(walls, 1, 2, 0, True)
                set_edge(walls, 2, 2, 1, True)
            cached = raw_solve(self.mini, walls, [goal])
            original = raw_solve(self.mini, walls, [goal], disable_memo=True)
            self.assertEqual(cached, original)
            self.assertEqual(cached["status"], "ok")

    def test_required_open_set_is_sufficient_for_equal_objective(self):
        for oracle in (self.mini, self.classic):
            for n, goal in ((4, (2, 2)), (6, (4, 3))):
                walls = open_maze(n)
                optimistic = oracle.solve(walls, [goal])
                known_only = [[15] * n for _ in range(n)]
                for x, y, d in optimistic["required_edges"]:
                    self.assertFalse(walls[y][x] & (1 << d))
                    set_edge(known_only, x, y, d, False)
                certified = oracle.solve(known_only, [goal])
                self.assertEqual(certified["status"], "ok")
                self.assertEqual(certified["goal_entry_s"], optimistic["goal_entry_s"])
                self.assertGreaterEqual(certified["stop_s"], certified["goal_entry_s"])

    def test_each_reported_dependency_blocks_selected_optimal_route(self):
        # Optimal costs may remain equal if a different route ties, so the
        # stronger requirement assertion lives in the sufficiency test above.
        walls = open_maze(4)
        result = self.mini.solve(walls, [(2, 2)])
        for x, y, d in result["required_edges"]:
            blocked = deepcopy(walls)
            set_edge(blocked, x, y, d, True)
            alternate = self.mini.solve(blocked, [(2, 2)], details=False)
            if alternate["status"] == "ok":
                self.assertGreaterEqual(alternate["goal_entry_s"], result["goal_entry_s"])

    def test_independent_instances_are_thread_safe(self):
        # ctypes releases the GIL while C runs. Each C trace cache must remain
        # scoped to that thread's maze/config, even across separate instances.
        requests = []
        for index in range(8):
            maze = open_maze(6)
            set_edge(maze, 1 + index % 3, 1 + index % 2, index % 2, True)
            requests.append(("mini_r2_0" if index % 3 else "classic_r1_0", maze))

        def run(request):
            machine, walls = request
            return TimeOracle(machine).solve(walls, [(4, 3)])

        expected = [run(request) for request in requests]
        with ThreadPoolExecutor(max_workers=4) as pool:
            observed = list(pool.map(run, requests))
        self.assertEqual(observed, expected)

    def test_nonzero_start_and_heading_are_respected(self):
        result = self.classic.solve(open_maze(6), [(4, 3)], start=(1, 3), heading=1)
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["route_points"][0], [1.0, 3.0])

    def test_reject_unsupported_profile_and_inconsistent_shared_wall(self):
        with self.assertRaises(ValueError):
            TimeOracle("classic_r1_0", planner="slalom")
        with self.assertRaises(ValueError):
            self.mini.solve([[16]], [(0, 0)])
        bad = open_maze(4)
        bad[0][0] |= 1
        self.assertEqual(self.mini.solve(bad, [(2, 2)])["status"], "invalid-maze")
        unavailable = TimeOracle("mini_r2_0", case=1)
        self.assertEqual(unavailable.solve(open_maze(4), [(2, 2)])["status"], "invalid_config")


if __name__ == "__main__":
    unittest.main()
