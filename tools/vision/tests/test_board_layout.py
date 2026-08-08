#!/usr/bin/env python3

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
VISION_ROOT = REPO_ROOT / "tools/vision"
sys.path.insert(0, str(VISION_ROOT))

import board_layout  # noqa: E402
import aruco_trajectory as aruco  # noqa: E402


class BoardLayoutTest(unittest.TestCase):
    def test_writer_fps_uses_nominal_hfr_rate_without_rounding_ntsc(self):
        self.assertEqual(aruco.writer_fps(239.914), 240.0)
        self.assertEqual(aruco.writer_fps(119.88), 120.0)
        self.assertEqual(aruco.writer_fps(29.97), 29.97)

    def test_example_has_metric_grid_and_aruco_corner_order(self):
        layout = board_layout.load(
            VISION_ROOT / "board_layout_4x4_example.json",
            900,
        )
        self.assertEqual(layout.grid.cells, 8)
        self.assertAlmostEqual(layout.grid_pitch_mm, 90.0)
        self.assertAlmostEqual(
            layout.grid.x_pitch_px,
            layout.grid.y_pitch_px,
        )
        self.assertAlmostEqual(
            layout.grid.x_origin_px,
            layout.grid.y_origin_px,
        )
        self.assertEqual(sorted(layout.target_corners_px), [4, 5, 6, 7])

        marker_5 = layout.target_corners_px[5]
        self.assertLess(marker_5[0, 0], marker_5[1, 0])
        self.assertAlmostEqual(marker_5[0, 1], marker_5[1, 1])
        self.assertLess(marker_5[1, 1], marker_5[2, 1])
        side_lengths = np.linalg.norm(
            np.roll(marker_5, -1, axis=0) - marker_5,
            axis=1,
        )
        np.testing.assert_allclose(side_lengths, side_lengths[0], rtol=1e-6)

    def test_measured_targets_drive_fixed_marker_homography(self):
        layout = board_layout.load(
            VISION_ROOT / "board_layout_4x4_example.json",
            900,
        )
        observations = []
        for _ in range(5):
            observations.append(
                {
                    marker_id: corners * 0.8 + np.asarray([100.0, 40.0])
                    for marker_id, corners in layout.target_corners_px.items()
                }
            )
        calibrations, targets = aruco.build_calibrations(
            observations,
            900,
            50.0,
            layout.target_corners_px,
        )
        self.assertEqual(len(calibrations), 5)
        self.assertTrue(all(not item.used_previous_homography for item in calibrations))
        self.assertTrue(all(item.inlier_corner_count == 4 for item in calibrations))
        self.assertTrue(all(item.reprojection_rmse_px < 1e-3 for item in calibrations))
        np.testing.assert_allclose(targets[5], layout.target_corners_px[5])

    def test_measured_layout_accepts_any_three_visible_markers(self):
        layout = board_layout.load(
            VISION_ROOT / "board_layout_4x4_example.json",
            900,
        )
        visible = {
            marker_id: corners * 0.8 + np.asarray([100.0, 40.0])
            for marker_id, corners in layout.target_corners_px.items()
            if marker_id != 5
        }
        calibrations, _ = aruco.build_calibrations(
            [visible for _ in range(5)],
            900,
            50.0,
            layout.target_corners_px,
        )
        self.assertTrue(all(not item.used_previous_homography for item in calibrations))
        self.assertTrue(all(item.inlier_marker_count == 3 for item in calibrations))

    def test_fixed_camera_can_allow_bounded_marker_occlusion(self):
        layout = board_layout.load(
            VISION_ROOT / "board_layout_4x4_example.json",
            900,
        )
        visible = {
            marker_id: corners * 0.8 + np.asarray([100.0, 40.0])
            for marker_id, corners in layout.target_corners_px.items()
        }
        observations = [visible, visible] + [{} for _ in range(8)] + [visible]

        with self.assertRaisesRegex(RuntimeError, "more than 5 consecutive"):
            aruco.build_calibrations(
                observations,
                900,
                50.0,
                layout.target_corners_px,
            )

        calibrations, _ = aruco.build_calibrations(
            observations,
            900,
            50.0,
            layout.target_corners_px,
            maximum_consecutive_fallback_frames=8,
        )
        self.assertEqual(
            sum(item.used_previous_homography for item in calibrations),
            8,
        )

    def test_measured_layout_prefers_four_marker_centers(self):
        layout = board_layout.load(
            VISION_ROOT / "board_layout_4x4_example.json",
            900,
        )
        scale_by_id = {5: 0.65, 7: 1.20, 4: 1.45, 6: 0.85}
        visible = {}
        for marker_id, corners in layout.target_corners_px.items():
            center = np.mean(corners, axis=0)
            visible[marker_id] = center + (corners - center) * scale_by_id[marker_id]

        calibrations, _ = aruco.build_calibrations(
            [visible for _ in range(5)],
            900,
            50.0,
            layout.target_corners_px,
        )

        self.assertTrue(all(not item.used_previous_homography for item in calibrations))
        self.assertTrue(all(item.inlier_corner_count == 4 for item in calibrations))
        source_centers = np.float32(
            [np.mean(visible[marker_id], axis=0) for marker_id in aruco.FIXED_ORDER]
        )
        expected_centers = np.float32(
            [
                np.mean(layout.target_corners_px[marker_id], axis=0)
                for marker_id in aruco.FIXED_ORDER
            ]
        )
        projected = aruco.cv2.perspectiveTransform(
            source_centers[np.newaxis],
            calibrations[0].homography,
        )[0]
        np.testing.assert_allclose(projected, expected_centers, atol=1e-3)

    def test_duplicate_json_keys_are_rejected(self):
        source = (VISION_ROOT / "board_layout_4x4_example.json").read_text(
            encoding="utf-8"
        )
        source = source.replace(
            '  "grid": {',
            '  "grid": {},\n  "grid": {',
            1,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.json"
            path.write_text(source, encoding="utf-8")
            with self.assertRaisesRegex(
                ValueError,
                "duplicate JSON key: grid",
            ):
                board_layout.load(path, 900)

    def test_nonstandard_nan_constant_is_rejected_even_if_unused(self):
        source = (VISION_ROOT / "board_layout_4x4_example.json").read_text(
            encoding="utf-8"
        )
        source = source.replace(
            '  "schema":',
            '  "unused_extension": NaN,\n  "schema":',
            1,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nan.json"
            path.write_text(source, encoding="utf-8")
            with self.assertRaisesRegex(
                ValueError,
                "non-standard JSON numeric constant: NaN",
            ):
                board_layout.load(path, 900)


if __name__ == "__main__":
    unittest.main()
