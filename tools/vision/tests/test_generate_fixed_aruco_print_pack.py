#!/usr/bin/env python3
"""Tests for the fixed ArUco print-pack generator."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

import generate_fixed_aruco_print_pack as print_pack  # noqa: E402


class FixedArucoPrintPackTest(unittest.TestCase):
    def test_marker_modules_have_black_one_module_border(self) -> None:
        for marker_id, _role in print_pack.MARKER_LAYOUT:
            modules = print_pack.marker_modules(marker_id)
            self.assertEqual(len(modules), print_pack.MODULES_PER_SIDE)
            self.assertTrue(all(value == 1 for value in modules[0]))
            self.assertTrue(all(value == 1 for value in modules[-1]))
            self.assertTrue(all(row[0] == 1 for row in modules))
            self.assertTrue(all(row[-1] == 1 for row in modules))

    def test_default_sizes_fit_a4_and_have_quiet_zone(self) -> None:
        self.assertEqual(
            print_pack.validate_sizes(print_pack.DEFAULT_SIZES_MM),
            (60.0, 40.0),
        )
        self.assertEqual(print_pack.quiet_zone_mm(60.0), 10.0)
        self.assertEqual(print_pack.quiet_zone_mm(40.0), 10.0)
        with self.assertRaisesRegex(ValueError, "do not fit"):
            print_pack.validate_sizes((80.0,))

    def test_svg_has_physical_dimensions_and_expected_cells(self) -> None:
        svg = print_pack.make_svg(5, 60.0)
        self.assertIn('width="80mm"', svg)
        self.assertIn('height="80mm"', svg)
        self.assertIn("DICT_4X4_50 ID 5", svg)
        expected_black_cells = sum(
            sum(row) for row in print_pack.marker_modules(5)
        )
        self.assertEqual(svg.count("<rect"), expected_black_cells + 1)


if __name__ == "__main__":
    unittest.main()
