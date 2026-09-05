import math
import unittest

from tools.exploration_sim.baseline import next_baseline
from tools.exploration_sim.profiles import get_profile, smooth_turn_duration


def open_grid(n=4):
    known = [[15] * n for _ in range(n)]
    walls = [[0] * n for _ in range(n)]
    for x in range(n):
        walls[0][x] |= 4
        walls[-1][x] |= 1
    for y in range(n):
        walls[y][0] |= 8
        walls[y][-1] |= 2
    return known, walls


class BaselineTests(unittest.TestCase):
    def test_relative_tie_break_and_observed_wall_detour(self):
        known, walls = open_grid()
        visited = [[True] * 4 for _ in range(4)]
        # Forward north and right east both decrease Manhattan distance.
        result = next_baseline(known, walls, visited, [(3, 3)], (1, 1), 0)
        self.assertEqual(result["direction"], 0)
        self.assertTrue(result["known_straight"])
        walls[1][1] |= 1
        walls[2][1] |= 4
        self.assertEqual(next_baseline(known, walls, visited, [(3, 3)], (1, 1), 0)["direction"], 1)

    def test_full_phase_targets_unvisited_not_unknown_cells(self):
        known, walls = open_grid()
        visited = [[True] * 4 for _ in range(4)]
        visited[2][3] = False  # Its walls are entirely known, but not visited.
        result = next_baseline(known, walls, visited, [(2, 2)], (2, 2), 0)
        self.assertEqual((result["phase"], result["direction"]), ("full", 1))
        visited[2][3] = True
        self.assertEqual(next_baseline(known, walls, visited, [(2, 2)], (2, 2), 0, "full")["phase"], "done")

    def test_finish_when_unvisited_cells_are_unreachable_without_home_return(self):
        known, walls = open_grid()
        visited = [[True] * 4 for _ in range(4)]
        visited[3][3] = False
        walls[3][3] = 15
        walls[2][3] |= 1
        walls[3][2] |= 2
        result = next_baseline(known, walls, visited, [(1, 1)], (2, 2), 0, "full")
        self.assertEqual(result["phase"], "done")
        self.assertIsNone(result["direction"])

    def test_cannot_accelerate_into_adjacent_target_even_if_visited(self):
        known, walls = open_grid()
        visited = [[True] * 4 for _ in range(4)]
        result = next_baseline(known, walls, visited, [(1, 2)], (1, 1), 0)
        self.assertEqual(result["direction"], 0)
        self.assertFalse(result["known_straight"])

    def test_true_unknown_wall_cannot_influence_decision(self):
        known, walls = open_grid()
        visited = [[True] * 4 for _ in range(4)]
        known[1][1] &= ~1
        known[2][1] &= ~4
        before = next_baseline(known, walls, visited, [(1, 3)], (1, 1), 0)
        walls[1][1] |= 1
        walls[2][1] |= 4
        after = next_baseline(known, walls, visited, [(1, 3)], (1, 1), 0)
        self.assertEqual(before, after)

    def test_full_route_visits_all_reachable_cells(self):
        known, walls = open_grid()
        visited = [[False] * 4 for _ in range(4)]
        pos, heading, phase = (0, 0), 0, "goal"
        for _ in range(100):
            x, y = pos
            visited[y][x] = True
            result = next_baseline(known, walls, visited, [(3, 3)], pos, heading, phase)
            phase = result["phase"]
            if phase == "done":
                break
            heading = result["direction"]
            pos = (x + (0, 1, 0, -1)[heading], y + (1, 0, -1, 0)[heading])
        self.assertEqual(phase, "done")
        self.assertTrue(all(all(row) for row in visited))


class ProfileTests(unittest.TestCase):
    def test_live_mode1_case1_sources(self):
        mini, classic = get_profile("mini_r2_0"), get_profile("classic_r1_0")
        self.assertEqual((mini["cell_mm"], mini["search_speed_mm_s"], mini["acceleration_mm_s2"]), (90, 300, 1000))
        self.assertEqual((classic["cell_mm"], classic["search_speed_mm_s"], classic["dash_acceleration_mm_s2"]), (180, 1000, 4000))
        self.assertAlmostEqual(mini["known_speed_mm_s"], math.sqrt(270000))
        self.assertAlmostEqual(classic["known_speed_mm_s"], math.sqrt(2440000))
        self.assertGreater(mini["entry_speed_mm_s"], mini["search_speed_mm_s"])
        self.assertGreater(classic["entry_speed_mm_s"], classic["search_speed_mm_s"])

    def test_cosine_turn_time_and_caps(self):
        t = smooth_turn_duration(90, 10000, 1.2)
        self.assertAlmostEqual(t, 2.7 * math.sqrt(60 / 10000))
        self.assertGreater(smooth_turn_duration(90, 10000, 1.2, 100), t)
        with self.assertRaises(ValueError):
            smooth_turn_duration(90, 0)


if __name__ == "__main__":
    unittest.main()
