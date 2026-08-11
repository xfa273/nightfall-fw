#!/usr/bin/env python3

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import sys


VISION_ROOT = Path(__file__).parents[1]
REPO_ROOT = VISION_ROOT.parents[1]
sys.path.insert(0, str(VISION_ROOT))

import trajectory_calibration  # noqa: E402


class TrajectoryCalibrationBindingTest(unittest.TestCase):
    def _fixture(self, root: Path) -> tuple[Path, Path]:
        trajectory = root / "trajectory_height_corrected.csv"
        trajectory.write_text("time_s,x_mm,y_mm\n0,1,2\n", encoding="ascii")
        board = root / "board.json"
        board.write_bytes(
            (VISION_ROOT / "data/board_layout_8x8_60mm.json").read_bytes()
        )
        tracking = root / "tracking.json"
        tracking.write_bytes(
            (
                REPO_ROOT
                / "tools/tuning/data/mini_r2_0_footprint.json"
            ).read_bytes()
        )
        geometry = root / "geometry.json"
        geometry.write_text(
            json.dumps(
                {
                    "schema": "nightfall_label_plane_geometry_v1",
                    "coordinate_system": "board_x_right_y_forward_z_up_mm",
                    "camera_center_board_mm": {
                        "x": 360.0,
                        "y": 360.0,
                        "height": 700.0,
                    },
                    "reference_plane_absolute_height_mm": 5.0,
                    "label_absolute_heights_mm": {
                        "blue_center": 10.0,
                        "red_front": 2.0,
                    },
                    "front_label_distance_mm": 24.0,
                    "bindings": {
                        "board_layout_sha256": (
                            trajectory_calibration.sha256_file(board)
                        ),
                        "tracking_calibration_sha256": (
                            trajectory_calibration.sha256_file(tracking)
                        ),
                    },
                    "qualification": {
                        "safety_qualified": True,
                        "failures": [],
                    },
                    "calibration": {
                        "schema": "nightfall_label_plane_camera_fit_v1",
                        "safety_qualified": True,
                        "failures": [],
                        "fit_placement_count": 4,
                        "validation_placement_count": 1,
                        "fit_span_mm": {"x": 630.0, "y": 630.0},
                        "fit_hull_area_mm2": 396900.0,
                        "stationary_hold_duration_s_min": 2.0,
                        "stationary_stability_p95_mm_max": 0.2,
                        "fit_error_mm": {"p95": 0.2},
                        "held_out_error_mm": {"p95": 0.3, "max": 0.4},
                    },
                }
            )
            + "\n",
            encoding="utf-8",
        )
        source_calibration = root / "calibration.json"
        source_calibration.write_text(
            json.dumps(
                {
                    "schema": "nightfall_markerless_board_calibration_v3",
                    "measured_layout": {
                        "sha256": trajectory_calibration.sha256_file(board),
                    },
                    "canonical": {"size_px": 900},
                }
            )
            + "\n",
            encoding="utf-8",
        )
        sidecar = root / "trajectory_height_corrected.calibration.json"
        sidecar.write_text(
            json.dumps(
                {
                    "schema": trajectory_calibration.SCHEMA,
                    "output": {
                        "trajectory_csv": str(trajectory.resolve()),
                        "sha256": trajectory_calibration.sha256_file(trajectory),
                    },
                    "bindings": {
                        "board_layout": str(board.resolve()),
                        "board_layout_sha256": trajectory_calibration.sha256_file(
                            board
                        ),
                        "tracking_geometry": str(tracking.resolve()),
                        "tracking_geometry_sha256": (
                            trajectory_calibration.sha256_file(tracking)
                        ),
                        "label_plane_geometry": str(geometry.resolve()),
                        "label_plane_geometry_sha256": (
                            trajectory_calibration.sha256_file(geometry)
                        ),
                        "source_board_calibration": str(
                            source_calibration.resolve()
                        ),
                        "source_board_calibration_sha256": (
                            trajectory_calibration.sha256_file(
                                source_calibration
                            )
                        ),
                    },
                    "qualification": {
                        "height_correction_applied": True,
                        "source_geometry_safety_qualified": True,
                        "source_board_layout_verified": True,
                        "operator_confirmed_unchanged_camera_board_setup": True,
                        "absolute_scene_eligible": True,
                    },
                }
            )
            + "\n",
            encoding="utf-8",
        )
        return trajectory, sidecar

    def test_verified_sidecar_binds_trajectory_and_calibrations(self):
        with tempfile.TemporaryDirectory() as directory:
            trajectory, sidecar = self._fixture(Path(directory))
            result = trajectory_calibration.verify_height_corrected_trajectory(
                trajectory, sidecar
            )
            self.assertTrue(result["qualification"]["absolute_scene_eligible"])

    def test_tampered_trajectory_and_unqualified_geometry_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trajectory, sidecar = self._fixture(root)
            trajectory.write_text(
                "time_s,x_mm,y_mm\n0,999,2\n", encoding="ascii"
            )
            with self.assertRaisesRegex(ValueError, "trajectory SHA-256"):
                trajectory_calibration.verify_height_corrected_trajectory(
                    trajectory, sidecar
                )

            trajectory, sidecar = self._fixture(root)
            geometry = root / "geometry.json"
            raw_geometry = json.loads(geometry.read_text(encoding="utf-8"))
            raw_geometry["qualification"]["safety_qualified"] = False
            geometry.write_text(json.dumps(raw_geometry) + "\n", encoding="utf-8")
            raw = json.loads(sidecar.read_text(encoding="utf-8"))
            raw["bindings"]["label_plane_geometry_sha256"] = (
                trajectory_calibration.sha256_file(geometry)
            )
            sidecar.write_text(json.dumps(raw) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "explicitly true"):
                trajectory_calibration.verify_height_corrected_trajectory(
                    trajectory, sidecar
                )

    def test_non_object_qualification_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            trajectory, sidecar = self._fixture(Path(directory))
            raw = json.loads(sidecar.read_text(encoding="utf-8"))
            raw["qualification"] = []
            sidecar.write_text(json.dumps(raw) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "sidecar fields"):
                trajectory_calibration.verify_height_corrected_trajectory(
                    trajectory, sidecar
                )

    def test_board_validator_matches_optional_id7_and_rejects_bad_bounds(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = VISION_ROOT / "data/board_layout_8x8_60mm.json"
            raw = json.loads(source.read_text(encoding="utf-8"))
            raw["markers"].pop("7")
            three_marker = root / "three-marker.json"
            three_marker.write_text(json.dumps(raw) + "\n", encoding="utf-8")
            trajectory_calibration._validate_board_layout(three_marker)

            raw["canvas_bounds_mm"]["x_max"] -= 1.0
            non_square = root / "non-square.json"
            non_square.write_text(json.dumps(raw) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "square"):
                trajectory_calibration._validate_board_layout(non_square)


if __name__ == "__main__":
    unittest.main()
