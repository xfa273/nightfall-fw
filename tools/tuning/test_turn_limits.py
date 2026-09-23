import argparse
import unittest
from pathlib import Path
import turn_tune

ROOT = Path(__file__).resolve().parents[2]

class TurnLimitsTest(unittest.TestCase):
    def constants(self, profile, mode, **overrides):
        args = dict(params_h=str(ROOT / 'params' / profile / 'params.h'),
                    shortest_params=str(ROOT / 'params' / profile / 'shortest_run_params_split.c'),
                    f413_path_h=None, runner='shortest', mode=mode, omega_max=None)
        args.update(overrides)
        return turn_tune.load_constants(argparse.Namespace(**args))

    def test_limits_are_mode_parameters(self):
        for mode in range(2, 8):
            self.assertEqual(self.constants('f413_preorder', mode).omega_cap_deg_s, 0)
            self.assertEqual(self.constants('mini_r3_0', mode).omega_cap_deg_s,
                             3000 if mode in (3, 4, 5, 6, 7) else 0)

    def test_explicit_simulator_override_and_search(self):
        self.assertEqual(self.constants('mini_r3_0', 6, omega_max=0).omega_cap_deg_s, 0)
        self.assertEqual(self.constants('mini_r3_0', 6, omega_max=3000).omega_cap_deg_s, 3000)
        self.assertEqual(self.constants('mini_r3_0', 6, runner='search').omega_cap_deg_s, 0)

    def test_invalid_limit_rejected(self):
        for value in (-1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                self.constants('mini_r3_0', 6, omega_max=value)

if __name__ == '__main__':
    unittest.main()
