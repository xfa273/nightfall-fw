#!/usr/bin/env python3

from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
VISION_ROOT = REPO_ROOT / "tools/vision"
sys.path.insert(0, str(VISION_ROOT))


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


import board_layout  # noqa: E402
import label_plane_geometry  # noqa: E402
import trajectory_calibration  # noqa: E402


APPLY = _load(
    "test_apply_label_plane_geometry_module",
    VISION_ROOT / "apply_label_plane_geometry.py",
)


class _FakeArtifact:
    def __init__(self, path: Path):
        self.path = path.resolve()


class _FakeCaptureFingerprint:
    safety_qualified = True
    camera_setup_sha256 = "12" * 32

    def __init__(self, trajectory: Path, calibration: Path):
        self._artifacts = {
            "trajectory_csv": _FakeArtifact(trajectory),
            "source_board_calibration": _FakeArtifact(calibration),
        }

    def artifact(self, name: str):
        return self._artifacts[name]

    def to_json(self):
        return {
            "schema": "nightfall_camera_capture_fingerprint_v1",
            "safety_qualified": True,
            "fingerprint_sha256": "34" * 32,
            "camera_setup_sha256": self.camera_setup_sha256,
            "artifacts": {},
        }


def _apparent(
    physical_xy: tuple[float, float],
    height_mm: float,
    camera: tuple[float, float, float],
    reference_height_mm: float,
) -> tuple[float, float]:
    cx, cy, camera_height = camera
    scale = (camera_height - reference_height_mm) / (
        camera_height - height_mm
    )
    return (
        cx + scale * (physical_xy[0] - cx),
        cy + scale * (physical_xy[1] - cy),
    )


class ApplyLabelPlaneGeometryTest(unittest.TestCase):
    def test_cli_output_sidecar_passes_production_verifier(self):
        board_path = (
            REPO_ROOT / "tools/vision/data/board_layout_8x8_60mm.json"
        )
        tracking_path = (
            REPO_ROOT / "tools/tuning/data/mini_r2_0_footprint.json"
        )
        layout = board_layout.load(board_path, 900)
        board_digest = APPLY.sha256_file(board_path)
        tracking_digest = APPLY.sha256_file(tracking_path)
        camera = (250.0, 740.0, 620.0)
        reference_height = 2.0
        bounds = layout.raw["canvas_bounds_mm"]

        def canonical(point: tuple[float, float]) -> tuple[float, float]:
            return (
                (point[0] - float(bounds["x_min"])) * layout.pixels_per_mm,
                (float(bounds["y_max"]) - point[1]) * layout.pixels_per_mm,
            )

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_csv = root / "trajectory.csv"
            geometry_path = root / "label_plane_geometry.json"
            source_calibration = root / "calibration.json"
            capture_manifest = root / "capture-session.json"
            output_csv = root / "trajectory_height_corrected.csv"
            sidecar_path = root / "trajectory_height_corrected.calibration.json"

            rows = []
            for index in range(21):
                blue = (180.0 + index, 270.0 + 0.5 * index)
                red = (blue[0] + 24.0, blue[1])
                apparent_blue = _apparent(
                    blue, 10.0, camera, reference_height
                )
                apparent_red = _apparent(
                    red, 2.0, camera, reference_height
                )
                blue_px = canonical(apparent_blue)
                red_px = canonical(apparent_red)
                rows.append(
                    {
                        "frame": str(index),
                        "video_pts_s": f"{index / 240.0:.9f}",
                        "label_x_px": f"{blue_px[0]:.12f}",
                        "label_y_px": f"{blue_px[1]:.12f}",
                        "front_label_x_px": f"{red_px[0]:.12f}",
                        "front_label_y_px": f"{red_px[1]:.12f}",
                        "x_mm": f"{apparent_blue[0]:.12f}",
                        "y_mm": f"{apparent_blue[1]:.12f}",
                        "yaw_deg_raw_unwrapped": "0",
                        "yaw_deg_unwrapped": "0",
                        "yaw_deg": "0",
                        "pose_valid": "1",
                        "heading_valid": "1",
                    }
                )
            with input_csv.open("w", newline="", encoding="ascii") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)

            geometry_path.write_text(
                json.dumps(
                    {
                        "schema": "nightfall_label_plane_geometry_v1",
                        "coordinate_system": (
                            "board_x_right_y_forward_z_up_mm"
                        ),
                        "camera_center_board_mm": {
                            "x": camera[0],
                            "y": camera[1],
                            "height": camera[2],
                        },
                        "reference_plane_absolute_height_mm": reference_height,
                        "label_absolute_heights_mm": {
                            "blue_center": 10.0,
                            "red_front": 2.0,
                        },
                        "front_label_distance_mm": 24.0,
                        "bindings": {
                            "board_layout_sha256": board_digest,
                            "tracking_calibration_sha256": tracking_digest,
                            "capture_session": {"placeholder": True},
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
                    },
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )
            source_calibration.write_text(
                json.dumps(
                    {
                        "schema": "nightfall_markerless_board_calibration_v3",
                        "measured_layout": {"sha256": board_digest},
                        "canonical": {"size_px": 900},
                    },
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )
            capture_manifest.write_text("{}\n", encoding="utf-8")
            capture = _FakeCaptureFingerprint(input_csv, source_calibration)

            argv = [
                "apply_label_plane_geometry.py",
                str(input_csv),
                "--board-layout",
                str(board_path),
                "--tracking-geometry",
                str(tracking_path),
                "--label-plane-geometry",
                str(geometry_path),
                "--capture-session-manifest",
                str(capture_manifest),
                "--source-calibration-json",
                str(source_calibration),
                "--confirm-unchanged-camera-board-setup",
                "--output",
                str(output_csv),
                "--sidecar",
                str(sidecar_path),
            ]
            with mock.patch.object(
                APPLY,
                "_load_target_capture",
                return_value=capture,
            ), mock.patch.object(
                APPLY,
                "_load_calibration_capture",
                return_value=capture,
            ), mock.patch.object(sys, "argv", argv):
                self.assertEqual(APPLY.main(), 0)

            with mock.patch.object(
                trajectory_calibration.camera_capture_fingerprint,
                "revalidate_capture_fingerprint",
                return_value=capture,
            ) as revalidate:
                verified = (
                    trajectory_calibration.verify_height_corrected_trajectory(
                        output_csv, sidecar_path
                    )
                )
            self.assertEqual(revalidate.call_count, 2)
            self.assertTrue(verified["qualification"]["absolute_scene_eligible"])
            self.assertEqual(verified["correction"]["sample_count"], len(rows))
            self.assertEqual(
                verified["output"]["sha256"],
                trajectory_calibration.sha256_file(output_csv),
            )
            self.assertAlmostEqual(
                verified["correction"][
                    "corrected_front_label_distance_mm"
                ]["median"],
                24.0,
                places=8,
            )
            with output_csv.open(newline="", encoding="ascii") as stream:
                first = next(csv.DictReader(stream))
            self.assertAlmostEqual(float(first["corrected_red_x_mm"]), 204.0)
            self.assertAlmostEqual(float(first["corrected_red_y_mm"]), 270.0)

    def test_corrected_csv_columns_recover_linear_blue_path_and_red_heading(self):
        layout = board_layout.load(
            REPO_ROOT / "tools/vision/data/board_layout_8x8_60mm.json",
            900,
        )
        camera = (250.0, 740.0, 620.0)
        reference_height = 3.0
        geometry = label_plane_geometry.LabelPlaneGeometry(
            camera_center_x_mm=camera[0],
            camera_center_y_mm=camera[1],
            camera_height_mm=camera[2],
            reference_plane_height_mm=reference_height,
            blue_label_height_mm=10.0,
            red_label_height_mm=2.0,
            front_label_distance_mm=24.0,
            board_layout_sha256="12" * 32,
            tracking_calibration_sha256="ab" * 32,
            safety_qualified=True,
        )
        bounds = layout.raw["canvas_bounds_mm"]
        rows = []
        expected_x = []
        expected_y = []
        for index in range(121):
            blue = (180.0 + index * 0.5, 270.0 + index * 0.25)
            red = (blue[0] + 24.0, blue[1])
            apparent_blue = _apparent(blue, 10.0, camera, reference_height)
            apparent_red = _apparent(red, 2.0, camera, reference_height)

            def canonical(point: tuple[float, float]) -> tuple[float, float]:
                return (
                    (point[0] - float(bounds["x_min"])) * layout.pixels_per_mm,
                    (float(bounds["y_max"]) - point[1]) * layout.pixels_per_mm,
                )

            blue_px = canonical(apparent_blue)
            red_px = canonical(apparent_red)
            rows.append(
                {
                    "frame": str(index),
                    "video_pts_s": f"{index / 120.0:.9f}",
                    "label_x_px": f"{blue_px[0]:.12f}",
                    "label_y_px": f"{blue_px[1]:.12f}",
                    "front_label_x_px": f"{red_px[0]:.12f}",
                    "front_label_y_px": f"{red_px[1]:.12f}",
                    "x_mm": f"{apparent_blue[0]:.12f}",
                    "y_mm": f"{apparent_blue[1]:.12f}",
                    "yaw_deg_unwrapped": "0",
                    "yaw_deg_raw_unwrapped": "0",
                    "yaw_deg": "0",
                    "pose_valid": "1",
                    "heading_valid": "1",
                }
            )
            expected_x.append(blue[0])
            expected_y.append(blue[1])

        corrected, metadata = APPLY.correct_rows(rows, layout, geometry, 9)
        actual_x = np.asarray([float(row["x_mm"]) for row in corrected])
        actual_y = np.asarray([float(row["y_mm"]) for row in corrected])
        actual_yaw = np.asarray(
            [float(row["yaw_deg_unwrapped"]) for row in corrected]
        )
        # The production Savitzky-Golay helper intentionally tapers its four
        # edge samples; the fully supported interior must recover the line.
        np.testing.assert_allclose(actual_x[4:-4], expected_x[4:-4], atol=1e-6)
        np.testing.assert_allclose(actual_y[4:-4], expected_y[4:-4], atol=1e-6)
        np.testing.assert_allclose(actual_yaw[4:-4], 0.0, atol=1e-6)
        self.assertTrue(
            all(
                row["position_source"] == "blue_label_height_corrected"
                for row in corrected
            )
        )
        self.assertGreater(
            metadata["blue_position_correction_mm"]["p95"], 1.0
        )
        self.assertAlmostEqual(
            metadata["corrected_front_label_distance_mm"]["median"],
            24.0,
            places=8,
        )

    def test_source_invalid_flags_are_never_resurrected(self):
        layout = board_layout.load(
            REPO_ROOT / "tools/vision/data/board_layout_8x8_60mm.json",
            900,
        )
        geometry = label_plane_geometry.LabelPlaneGeometry(
            camera_center_x_mm=360.0,
            camera_center_y_mm=360.0,
            camera_height_mm=700.0,
            reference_plane_height_mm=5.0,
            blue_label_height_mm=10.0,
            red_label_height_mm=2.0,
            front_label_distance_mm=24.0,
            board_layout_sha256="12" * 32,
            tracking_calibration_sha256="ab" * 32,
            safety_qualified=True,
        )
        rows = []
        for index in range(11):
            rows.append(
                {
                    "video_pts_s": f"{index / 240.0:.9f}",
                    "label_x_px": "450",
                    "label_y_px": "450",
                    "front_label_x_px": "474",
                    "front_label_y_px": "450",
                    "pose_valid": "1" if index not in (4, 5) else "0",
                    "heading_valid": "1" if index != 6 else "0",
                }
            )
        corrected, metadata = APPLY.correct_rows(rows, layout, geometry, 5)
        self.assertEqual(corrected[4]["pose_valid"], "0")
        self.assertEqual(corrected[5]["pose_valid"], "0")
        self.assertEqual(corrected[6]["heading_valid"], "0")
        self.assertEqual(metadata["pose_valid_count"], 9)
        self.assertEqual(metadata["heading_valid_count"], 8)


if __name__ == "__main__":
    unittest.main()
