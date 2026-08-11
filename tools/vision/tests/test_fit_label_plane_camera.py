#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import csv
import json
import math
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

import numpy as np


VISION_ROOT = Path(__file__).parents[1]
REPO_ROOT = VISION_ROOT.parents[1]
sys.path.insert(0, str(VISION_ROOT))


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


FIT = _load_module(
    "test_fit_label_plane_camera_module",
    VISION_ROOT / "fit_label_plane_camera.py",
)


def _apparent(
    physical_xy: tuple[float, float],
    absolute_height_mm: float,
    camera_xyz: tuple[float, float, float],
    reference_height_mm: float,
) -> tuple[float, float]:
    """Independent inverse ray/plane projection used to build fit fixtures."""

    cx, cy, camera_height = camera_xyz
    scale = (camera_height - reference_height_mm) / (
        camera_height - absolute_height_mm
    )
    return (
        cx + scale * (physical_xy[0] - cx),
        cy + scale * (physical_xy[1] - cy),
    )


class LabelPlaneCameraFitTest(unittest.TestCase):
    def test_committed_manifest_template_has_four_fit_and_one_holdout(self):
        raw = json.loads(
            (
                VISION_ROOT
                / "data/label_plane_known_pose_manifest.template.json"
            ).read_text(encoding="utf-8")
        )
        self.assertEqual(raw["schema"], FIT.MANIFEST_SCHEMA)
        self.assertIsNone(raw["reference_plane_absolute_height_mm"])
        self.assertEqual(
            [item["role"] for item in raw["placements"]].count("fit"),
            4,
        )
        self.assertEqual(
            [item["role"] for item in raw["placements"]].count("validation"),
            1,
        )

    def test_four_board_corners_recover_camera_and_held_out_pose(self):
        camera = (287.0, 431.0, 812.0)
        reference_height = 3.0
        blue_height = 10.0
        red_height = 2.0
        distance = 24.0
        specifications = (
            ("fit-sw", "fit", (45.0, 45.0), 0.0),
            ("fit-se", "fit", (675.0, 45.0), 180.0),
            ("fit-nw", "fit", (45.0, 675.0), 0.0),
            ("fit-ne", "fit", (675.0, 675.0), 180.0),
            ("holdout", "validation", (315.0, 405.0), 90.0),
        )
        placements = []
        for placement_id, role, blue, heading_deg in specifications:
            heading = math.radians(heading_deg)
            red = (
                blue[0] + distance * math.cos(heading),
                blue[1] + distance * math.sin(heading),
            )
            placements.append(
                FIT.Placement(
                    placement_id=placement_id,
                    role=role,
                    apparent_blue_xy_mm=_apparent(
                        blue, blue_height, camera, reference_height
                    ),
                    apparent_red_xy_mm=_apparent(
                        red, red_height, camera, reference_height
                    ),
                    true_blue_xy_mm=blue,
                    true_red_xy_mm=red,
                    source={},
                    stability_p95_mm=0.0,
                    stationary_duration_s=2.0,
                )
            )

        geometry, condition = FIT.fit_camera(
            placements,
            reference_height,
            blue_height,
            red_height,
            huber_delta_mm=0.5,
        )
        self.assertLess(condition, 1.0e8)
        np.testing.assert_allclose(
            (
                geometry.camera_center_x_mm,
                geometry.camera_center_y_mm,
                geometry.camera_height_mm,
            ),
            camera,
            atol=1e-8,
        )
        held_out = [item for item in placements if item.role == "validation"]
        self.assertLess(max(FIT._errors(held_out, geometry)), 1e-8)

    def test_fewer_than_four_fit_poses_are_rejected(self):
        placement = FIT.Placement(
            placement_id="same",
            role="fit",
            apparent_blue_xy_mm=(100.0, 100.0),
            apparent_red_xy_mm=(124.0, 100.0),
            true_blue_xy_mm=(100.0, 100.0),
            true_red_xy_mm=(124.0, 100.0),
            source={},
            stability_p95_mm=0.0,
            stationary_duration_s=2.0,
        )
        with self.assertRaisesRegex(ValueError, "at least four"):
            FIT.fit_camera(
                [placement] * 3,
                reference_height_mm=0.0,
                blue_height_mm=10.0,
                red_height_mm=2.0,
                huber_delta_mm=0.5,
            )

    def test_nonfinite_source_validity_flags_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trajectory.csv"
            with path.open("w", newline="", encoding="ascii") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=(
                        "video_pts_s",
                        "label_x_px",
                        "label_y_px",
                        "front_label_x_px",
                        "front_label_y_px",
                        "pose_valid",
                        "heading_valid",
                    ),
                )
                writer.writeheader()
                for index in range(120):
                    writer.writerow(
                        {
                            "video_pts_s": index / 240.0,
                            "label_x_px": 400,
                            "label_y_px": 400,
                            "front_label_x_px": 424,
                            "front_label_y_px": 400,
                            "pose_valid": "nan",
                            "heading_valid": "inf",
                        }
                    )
            layout = FIT.board_layout.load(
                VISION_ROOT / "data/board_layout_8x8_60mm.json",
                900,
            )
            with self.assertRaisesRegex(ValueError, "fewer than 100"):
                FIT._trajectory_label_centres(path, layout)

    def test_manifest_to_qualified_geometry_end_to_end(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            board = REPO_ROOT / "tools/vision/data/board_layout_8x8_60mm.json"
            tracking = REPO_ROOT / "tools/tuning/data/mini_r2_0_footprint.json"
            layout = FIT.board_layout.load(board, 900)
            bounds = layout.raw["canvas_bounds_mm"]
            camera = (287.0, 431.0, 812.0)
            reference_height = 3.0
            specifications = (
                ("fit-sw", "fit", (45.0, 45.0), 0.0, 0.0),
                ("fit-se", "fit", (675.0, 45.0), 180.0, 2.5),
                ("fit-nw", "fit", (45.0, 675.0), 0.0, 5.0),
                ("fit-ne", "fit", (675.0, 675.0), 180.0, 7.5),
                ("holdout", "validation", (315.0, 405.0), 90.0, 10.0),
            )
            trajectory = root / "trajectory.csv"
            with trajectory.open("w", newline="", encoding="ascii") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=(
                        "video_pts_s",
                        "label_x_px",
                        "label_y_px",
                        "front_label_x_px",
                        "front_label_y_px",
                        "pose_valid",
                        "heading_valid",
                    ),
                )
                writer.writeheader()
                for _identifier, _role, blue, heading_deg, start_s in specifications:
                    heading = math.radians(heading_deg)
                    red = (
                        blue[0] + 24.0 * math.cos(heading),
                        blue[1] + 24.0 * math.sin(heading),
                    )
                    apparent_blue = _apparent(
                        blue, 10.0, camera, reference_height
                    )
                    apparent_red = _apparent(
                        red, 2.0, camera, reference_height
                    )

                    def canonical(point):
                        return (
                            (point[0] - float(bounds["x_min"]))
                            * layout.pixels_per_mm,
                            (float(bounds["y_max"]) - point[1])
                            * layout.pixels_per_mm,
                        )

                    blue_px = canonical(apparent_blue)
                    red_px = canonical(apparent_red)
                    for frame in range(481):
                        writer.writerow(
                            {
                                "video_pts_s": f"{start_s + frame / 240.0:.9f}",
                                "label_x_px": f"{blue_px[0]:.12f}",
                                "label_y_px": f"{blue_px[1]:.12f}",
                                "front_label_x_px": f"{red_px[0]:.12f}",
                                "front_label_y_px": f"{red_px[1]:.12f}",
                                "pose_valid": "1",
                                "heading_valid": "1",
                            }
                        )
            source_calibration = root / "calibration.json"
            source_calibration.write_text(
                json.dumps(
                    {
                        "schema": "nightfall_markerless_board_calibration_v3",
                        "measured_layout": {
                            "sha256": FIT.sha256_file(board),
                        },
                        "canonical": {"size_px": 900},
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            manifest = root / "manifest.json"
            manifest.write_text(
                json.dumps(
                    {
                        "schema": FIT.MANIFEST_SCHEMA,
                        "board_layout": str(board),
                        "tracking_geometry": str(tracking),
                        "canonical_size_px": 900,
                        "reference_plane_absolute_height_mm": reference_height,
                        "placements": [
                            {
                                "id": identifier,
                                "role": role,
                                "trajectory_csv": str(trajectory),
                                "start_s": start_s,
                                "end_s": start_s + 2.0,
                                "true_blue_center_mm": list(blue),
                                "true_heading_deg": heading,
                            }
                            for identifier, role, blue, heading, start_s in specifications
                        ],
                    }
                )
                + "\n",
                encoding="utf-8",
            )
            output = root / "geometry.json"
            with mock.patch.object(
                sys,
                "argv",
                [
                    "fit_label_plane_camera.py",
                    str(manifest),
                    "--output",
                    str(output),
                ],
            ):
                self.assertEqual(FIT.main(), 0)
            geometry = FIT.label_plane_geometry.load_geometry(
                output,
                expected_board_layout_sha256=FIT.sha256_file(board),
                expected_tracking_calibration_sha256=FIT.sha256_file(tracking),
            )
            np.testing.assert_allclose(
                (
                    geometry.camera_center_x_mm,
                    geometry.camera_center_y_mm,
                    geometry.camera_height_mm,
                ),
                camera,
                atol=1e-6,
            )


if __name__ == "__main__":
    unittest.main()
