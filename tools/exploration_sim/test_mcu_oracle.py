"""MCU exactness, dependency sufficiency, resumability and workspace safety."""
import ctypes
import random
import unittest

from tools.exploration_sim.mcu_oracle import McuOracle, Result
from tools.exploration_sim.oracle import TimeOracle
from tools.exploration_sim.test_oracle import open_maze, set_edge


class McuOracleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mcu = McuOracle()
        cls.generic = TimeOracle()

    def assert_parity(self, walls, goals, **kwargs):
        actual = self.mcu.solve(walls, goals, **kwargs)
        expected = self.generic.solve(walls, goals, **{k: v for k, v in kwargs.items()
                                                     if k in ("start", "heading")})
        self.assertEqual(actual["status"], "exact" if expected["status"] == "ok" else expected["status"])
        if actual["status"] == "exact":
            self.assertTrue(actual["requirements_complete"])
            self.assertEqual(actual["goal_entry_us"], round(expected["goal_entry_s"] * 1_000_000))
            self.assertGreaterEqual(actual["stop_us"], actual["goal_entry_us"])
            height, width = len(walls), len(walls[0])
            only_required = [[15] * width for _ in range(height)]
            for y in range(height):
                for x in range(width):
                    self.assertEqual(actual["required_masks"][y * width + x] & walls[y][x], 0)
            for x, y, direction in actual["required_edges"]:
                set_edge(only_required, x, y, direction, False)
            certified = self.generic.solve(only_required, goals, **{k: v for k, v in kwargs.items()
                                                                   if k in ("start", "heading")})
            self.assertEqual(certified["status"], "ok")
            self.assertEqual(round(certified["goal_entry_s"] * 1_000_000), actual["goal_entry_us"])
        else:
            self.assertFalse(actual["requirements_complete"])
            self.assertFalse(any(actual["required_masks"]))
        return actual

    def test_full_32_graph_and_compact_workspace(self):
        result = self.assert_parity(open_maze(32), [(15, 15)])
        self.assertLessEqual(result["workspace_used"], 200 * 1024)
        self.assertGreater(result["expanded_states"], 4000)

    def test_obstructed_mazes_and_required_edge_certificates(self):
        for seed in range(16):
            n = 4 + seed % 5
            walls = open_maze(n)
            rng = random.Random(seed)
            for y in range(n):
                for x in range(n):
                    for d in range(2):
                        if (d == 0 and y + 1 < n or d == 1 and x + 1 < n) and rng.random() < .16:
                            set_edge(walls, x, y, d, True)
            with self.subTest(seed=seed):
                self.assert_parity(walls, [(n - 2, n - 2)])

    def test_budget_changes_preserve_result_and_requirements(self):
        walls = open_maze(6)
        set_edge(walls, 1, 2, 0, True)
        results = [self.mcu.solve(walls, [(4, 3)], budget=b) for b in (1, 7, 256, 4096)]
        keys = ("status", "goal_entry_us", "stop_us", "required_masks", "work_units",
                "expanded_states", "action_count")
        self.assertEqual([tuple(r[k] for k in keys) for r in results],
                         [tuple(results[0][k] for k in keys)] * len(results))

    def test_nonzero_start_headings_and_goal_set(self):
        for heading in range(4):
            with self.subTest(heading=heading):
                self.assert_parity(open_maze(8), [(5, 5), (5, 6)], start=(3, 3), heading=heading)
        self.assert_parity(open_maze(4), [(0, 0)])

    def test_rectangular_graph(self):
        walls = [[0] * 7 for _ in range(4)]
        for y in range(4):
            for x in range(7):
                walls[y][x] = (8 if x == 0 else 0) | (2 if x == 6 else 0) | (4 if y == 0 else 0) | (1 if y == 3 else 0)
        set_edge(walls, 0, 0, 1, True)
        self.assert_parity(walls, [(5, 2)])

    def test_pending_is_not_a_certificate_and_begin_snapshots_map(self):
        lib = self.mcu.lib
        size = lib.nf_mcu_slalom_workspace_bytes_for(4, 4)
        storage = ctypes.create_string_buffer(size + 8)
        address = (ctypes.addressof(storage) + 7) & ~7
        data = ctypes.c_uint8 * 16
        walls = data(*(v for row in open_maze(4) for v in row))
        goals = data(); goals[10] = 1
        context = ctypes.c_void_p()
        self.assertEqual(lib.nf_mcu_slalom_begin(address, size - 1, 4, 4, walls, goals,
                                               0, 0, 0, ctypes.byref(context)), 5)
        self.assertFalse(context.value)
        self.assertEqual(lib.nf_mcu_slalom_begin(address, size, 4, 4, walls, goals,
                                               0, 0, 0, ctypes.byref(context)), 0)
        before, after = Result(), Result()
        required = data(*([15] * 16))
        lib.nf_mcu_slalom_result(context, ctypes.byref(before), required, 16)
        self.assertFalse(before.requirements_complete)
        self.assertFalse(any(required))
        self.assertEqual(lib.nf_mcu_slalom_step(context, 0), 0)
        lib.nf_mcu_slalom_result(context, ctypes.byref(after), None, 0)
        self.assertEqual(before.work_units, after.work_units)
        for i in range(16): walls[i] = 15; goals[i] = 0
        while lib.nf_mcu_slalom_step(context, 17) == 0:
            lib.nf_mcu_slalom_result(context, ctypes.byref(after), required, 16)
            self.assertFalse(after.requirements_complete)
            self.assertFalse(any(required))
        lib.nf_mcu_slalom_result(context, ctypes.byref(after), required, 16)
        self.assertTrue(after.requirements_complete)
        self.assertEqual(after.goal_entry_us, 1_061_918)
        # A reused successful result must lose its certificate even if the
        # caller accidentally supplies too little dependency output capacity.
        self.assertEqual(lib.nf_mcu_slalom_result(context, ctypes.byref(after), required, 15), 5)
        self.assertEqual(after.status, 5)
        self.assertFalse(after.requirements_complete)
        self.assertEqual(after.goal_entry_us, 0)
        self.assertFalse(any(required[:15]))
        lib.nf_mcu_slalom_result(context, ctypes.byref(after), required, 16)
        self.assertTrue(after.requirements_complete)
        self.assertEqual(lib.nf_mcu_slalom_result(None, ctypes.byref(after), None, 0), 4)
        self.assertEqual(after.status, 4)
        self.assertFalse(after.requirements_complete)


if __name__ == "__main__":
    unittest.main()
