#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import unittest
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
VISION_ROOT = REPO_ROOT / "tools/vision"
sys.path.insert(0, str(VISION_ROOT))

import desk_green_pair_probe as probe  # noqa: E402


def arguments() -> argparse.Namespace:
    return argparse.Namespace(
        minimum_pair_distance_px=85.0,
        maximum_pair_distance_px=210.0,
        maximum_tracking_distance_px=70.0,
        initial_x_px=None,
        initial_y_px=None,
    )


class DeskGreenPairProbeTest(unittest.TestCase):
    def test_initial_selection_prefers_large_rigid_pair(self):
        candidates = [
            probe.GreenComponent(5300, np.asarray([1210.0, 748.0])),
            probe.GreenComponent(3300, np.asarray([1050.0, 763.0])),
            probe.GreenComponent(3100, np.asarray([1740.0, 852.0])),
            probe.GreenComponent(1200, np.asarray([1589.0, 1027.0])),
        ]
        selected = probe.choose_pair(
            candidates,
            prediction_xy=None,
            expected_separation=155.0,
            args=arguments(),
        )
        self.assertIsNotNone(selected)
        assert selected is not None
        first, second, center, separation = selected
        self.assertEqual(first.area + second.area, 8600)
        np.testing.assert_allclose(center, [1130.0, 755.5])
        self.assertAlmostEqual(separation, 160.7016, places=3)

    def test_prediction_rejects_larger_distant_distractor(self):
        candidates = [
            probe.GreenComponent(4500, np.asarray([560.0, 735.0])),
            probe.GreenComponent(4400, np.asarray([705.0, 728.0])),
            probe.GreenComponent(7000, np.asarray([1500.0, 600.0])),
            probe.GreenComponent(7000, np.asarray([1650.0, 600.0])),
        ]
        selected = probe.choose_pair(
            candidates,
            prediction_xy=np.asarray([632.0, 731.0]),
            expected_separation=150.0,
            args=arguments(),
        )
        self.assertIsNotNone(selected)
        assert selected is not None
        _, _, center, _ = selected
        np.testing.assert_allclose(center, [632.5, 731.5])


if __name__ == "__main__":
    unittest.main()
