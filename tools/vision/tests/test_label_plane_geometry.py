#!/usr/bin/env python3

from __future__ import annotations

import copy
import importlib.util
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).parents[1] / "label_plane_geometry.py"
SPEC = importlib.util.spec_from_file_location(
    "test_label_plane_geometry_module", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
GEOMETRY = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = GEOMETRY
SPEC.loader.exec_module(GEOMETRY)


BOARD_SHA = "12" * 32
TRACKING_SHA = "ab" * 32


def _look_at_rotation(
    camera_center: np.ndarray, target: np.ndarray
) -> np.ndarray:
    """Return world-to-camera rotation with image y pointing down."""

    forward = target - camera_center
    forward /= np.linalg.norm(forward)
    up_hint = np.asarray([0.0, 1.0, 0.0])
    right = np.cross(forward, up_hint)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    return np.vstack([right, -up, forward])


def _project(
    point_xyz: tuple[float, float, float],
    camera_center: np.ndarray,
    rotation: np.ndarray,
    intrinsic: np.ndarray,
) -> np.ndarray:
    camera_point = rotation @ (np.asarray(point_xyz) - camera_center)
    if camera_point[2] <= 0.0:
        raise AssertionError("synthetic point is behind the camera")
    pixel_h = intrinsic @ camera_point
    return pixel_h[:2] / pixel_h[2]


def _homography(source_xy: np.ndarray, destination_xy: np.ndarray) -> np.ndarray:
    """Solve a four-point homography without using production geometry."""

    rows = []
    values = []
    for (u, v), (x, y) in zip(source_xy, destination_xy):
        rows.append([u, v, 1.0, 0.0, 0.0, 0.0, -x * u, -x * v])
        values.append(x)
        rows.append([0.0, 0.0, 0.0, u, v, 1.0, -y * u, -y * v])
        values.append(y)
    solved = np.linalg.solve(np.asarray(rows), np.asarray(values))
    return np.asarray(
        [
            [solved[0], solved[1], solved[2]],
            [solved[3], solved[4], solved[5]],
            [solved[6], solved[7], 1.0],
        ]
    )


def _apply_homography(point_xy: np.ndarray, transform: np.ndarray) -> np.ndarray:
    homogeneous = transform @ np.asarray([point_xy[0], point_xy[1], 1.0])
    return homogeneous[:2] / homogeneous[2]


class SyntheticCamera:
    def __init__(
        self,
        camera_center_xyz: tuple[float, float, float],
        target_xyz: tuple[float, float, float],
        reference_height_mm: float = 0.0,
    ) -> None:
        self.center = np.asarray(camera_center_xyz, dtype=float)
        self.reference_height_mm = reference_height_mm
        self.rotation = _look_at_rotation(
            self.center,
            np.asarray(target_xyz, dtype=float),
        )
        self.intrinsic = np.asarray(
            [[1300.0, 0.0, 960.0], [0.0, 1250.0, 540.0], [0.0, 0.0, 1.0]]
        )
        board_xy = np.asarray(
            [[0.0, 0.0], [720.0, 0.0], [720.0, 720.0], [0.0, 720.0]]
        )
        image_xy = np.asarray(
            [
                _project(
                    (point[0], point[1], reference_height_mm),
                    self.center,
                    self.rotation,
                    self.intrinsic,
                )
                for point in board_xy
            ]
        )
        self.image_to_reference = _homography(image_xy, board_xy)

    def apparent_floor_xy(
        self, physical_xy: tuple[float, float], absolute_height_mm: float
    ) -> np.ndarray:
        pixel = _project(
            (physical_xy[0], physical_xy[1], absolute_height_mm),
            self.center,
            self.rotation,
            self.intrinsic,
        )
        return _apply_homography(pixel, self.image_to_reference)


def _geometry(
    camera_center: tuple[float, float, float],
    reference_height_mm: float = 0.0,
    blue_height_mm: float = 10.0,
    red_height_mm: float = 2.0,
) -> object:
    return GEOMETRY.LabelPlaneGeometry(
        camera_center_x_mm=camera_center[0],
        camera_center_y_mm=camera_center[1],
        camera_height_mm=camera_center[2],
        reference_plane_height_mm=reference_height_mm,
        blue_label_height_mm=blue_height_mm,
        red_label_height_mm=red_height_mm,
        front_label_distance_mm=24.0,
        board_layout_sha256=BOARD_SHA,
        tracking_calibration_sha256=TRACKING_SHA,
        safety_qualified=True,
    )


def _artifact() -> dict[str, object]:
    return {
        "schema": GEOMETRY.SCHEMA,
        "coordinate_system": GEOMETRY.COORDINATE_SYSTEM,
        "camera_center_board_mm": {"x": -200.0, "y": 360.0, "height": 700.0},
        "reference_plane_absolute_height_mm": 0.0,
        "label_absolute_heights_mm": {"blue_center": 10.0, "red_front": 2.0},
        "front_label_distance_mm": 24.0,
        "bindings": {
            "board_layout_sha256": BOARD_SHA,
            "tracking_calibration_sha256": TRACKING_SHA,
        },
        "qualification": {"safety_qualified": True, "failures": []},
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


class LabelPlaneProjectionTest(unittest.TestCase):
    def test_red_label_on_two_mm_marker_plane_is_unchanged(self):
        camera_center = (-200.0, 360.0, 700.0)
        camera = SyntheticCamera(
            camera_center,
            (360.0, 360.0, 2.0),
            2.0,
        )
        geometry = _geometry(
            camera_center,
            reference_height_mm=2.0,
            blue_height_mm=10.0,
            red_height_mm=2.0,
        )
        physical_blue = (270.0, 450.0)
        physical_red = (294.0, 450.0)
        apparent_blue = camera.apparent_floor_xy(physical_blue, 10.0)
        apparent_red = camera.apparent_floor_xy(physical_red, 2.0)

        np.testing.assert_allclose(apparent_red, physical_red, atol=1e-8)
        self.assertGreater(np.linalg.norm(apparent_blue - physical_blue), 1.0)
        corrected = geometry.correct_pair(apparent_blue, apparent_red)
        np.testing.assert_allclose(corrected.blue_xy_mm, physical_blue, atol=1e-8)
        np.testing.assert_allclose(corrected.red_xy_mm, physical_red, atol=1e-8)
        self.assertAlmostEqual(corrected.baseline_mm, 24.0, places=8)
        self.assertAlmostEqual(corrected.yaw_deg, 0.0, places=8)

    def test_oblique_camera_recovers_blue10_red2_baseline_and_yaw90(self):
        camera_center = (-200.0, 360.0, 700.0)
        camera = SyntheticCamera(camera_center, (360.0, 360.0, 0.0))
        geometry = _geometry(camera_center)
        physical_blue = (360.0, 360.0)
        physical_red = (360.0, 384.0)
        apparent_blue = camera.apparent_floor_xy(physical_blue, 10.0)
        apparent_red = camera.apparent_floor_xy(physical_red, 2.0)

        naive_vector = apparent_red - apparent_blue
        naive_yaw = math.degrees(math.atan2(naive_vector[1], naive_vector[0]))
        self.assertGreater(np.linalg.norm(apparent_blue - physical_blue), 8.0)
        self.assertGreater(abs(naive_yaw - 90.0), 15.0)

        corrected = geometry.correct_pair(apparent_blue, apparent_red)
        np.testing.assert_allclose(corrected.blue_xy_mm, physical_blue, atol=1e-8)
        np.testing.assert_allclose(corrected.red_xy_mm, physical_red, atol=1e-8)
        self.assertAlmostEqual(corrected.baseline_mm, 24.0, places=8)
        # Board +y is forward/up, and is therefore +90 degrees from board +x.
        self.assertAlmostEqual(corrected.yaw_deg, 90.0, places=8)

    def test_near_overhead_center_hides_corner_parallax_failure(self):
        camera_center = (360.0, 360.0, 1000.0)
        camera = SyntheticCamera(camera_center, (360.0, 360.0, 0.0))
        geometry = _geometry(camera_center)

        def apparent_pose(blue: tuple[float, float]) -> tuple[np.ndarray, np.ndarray]:
            red = (blue[0], blue[1] + 24.0)
            return (
                camera.apparent_floor_xy(blue, 10.0),
                camera.apparent_floor_xy(red, 2.0),
            )

        center_blue, center_red = apparent_pose((360.0, 360.0))
        center_vector = center_red - center_blue
        center_yaw = math.degrees(math.atan2(center_vector[1], center_vector[0]))
        self.assertAlmostEqual(center_yaw, 90.0, places=8)

        corner_physical = (630.0, 630.0)
        corner_blue, corner_red = apparent_pose(corner_physical)
        corner_vector = corner_red - corner_blue
        corner_yaw = math.degrees(math.atan2(corner_vector[1], corner_vector[0]))
        self.assertGreater(np.linalg.norm(corner_blue - corner_physical), 3.8)
        self.assertGreater(abs(corner_yaw - 90.0), 5.7)

        corrected = geometry.correct_pair(corner_blue, corner_red)
        np.testing.assert_allclose(
            corrected.blue_xy_mm,
            corner_physical,
            atol=1e-8,
        )
        self.assertAlmostEqual(corrected.baseline_mm, 24.0, places=8)
        self.assertAlmostEqual(corrected.yaw_deg, 90.0, places=8)

    def test_red_label_below_elevated_reference_uses_absolute_height(self):
        camera_center = (-200.0, 360.0, 705.0)
        reference_height = 5.0
        camera = SyntheticCamera(
            camera_center,
            (360.0, 360.0, reference_height),
            reference_height,
        )
        geometry = _geometry(
            camera_center,
            reference_height_mm=reference_height,
            blue_height_mm=10.0,
            red_height_mm=2.0,
        )
        physical_blue = (270.0, 450.0)
        physical_red = (294.0, 450.0)
        apparent_blue = camera.apparent_floor_xy(physical_blue, 10.0)
        apparent_red = camera.apparent_floor_xy(physical_red, 2.0)

        corrected = geometry.correct_pair(apparent_blue, apparent_red)
        np.testing.assert_allclose(corrected.blue_xy_mm, physical_blue, atol=1e-8)
        np.testing.assert_allclose(corrected.red_xy_mm, physical_red, atol=1e-8)
        self.assertAlmostEqual(corrected.baseline_mm, 24.0, places=8)
        self.assertAlmostEqual(corrected.yaw_deg, 0.0, places=8)


class LabelPlaneGeometryLoaderTest(unittest.TestCase):
    def _load(self, raw: dict[str, object]):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "label_plane_geometry.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            return GEOMETRY.load_geometry(
                path,
                expected_board_layout_sha256=BOARD_SHA,
                expected_tracking_calibration_sha256=TRACKING_SHA,
            )

    def test_loader_accepts_bound_blue10_red2_geometry(self):
        geometry = self._load(_artifact())
        self.assertEqual(geometry.blue_label_height_mm, 10.0)
        self.assertEqual(geometry.red_label_height_mm, 2.0)
        self.assertEqual(geometry.front_label_distance_mm, 24.0)
        self.assertEqual(geometry.board_layout_sha256, BOARD_SHA)
        self.assertEqual(geometry.tracking_calibration_sha256, TRACKING_SHA)
        self.assertTrue(geometry.safety_qualified)

    def test_loader_allows_label_below_elevated_reference_plane(self):
        artifact = _artifact()
        artifact["reference_plane_absolute_height_mm"] = 5.0
        geometry = self._load(artifact)
        self.assertEqual(geometry.reference_plane_height_mm, 5.0)
        self.assertEqual(geometry.red_label_height_mm, 2.0)

    def test_loader_rejects_schema_nonfinite_and_invalid_absolute_heights(self):
        cases: list[tuple[str, dict[str, object], str]] = []

        wrong_schema = copy.deepcopy(_artifact())
        wrong_schema["schema"] = "wrong"
        cases.append(("schema", wrong_schema, "schema"))

        nonfinite = copy.deepcopy(_artifact())
        nonfinite["camera_center_board_mm"]["x"] = float("nan")  # type: ignore[index]
        cases.append(("nonfinite", nonfinite, "finite"))

        below_floor = copy.deepcopy(_artifact())
        below_floor["label_absolute_heights_mm"]["red_front"] = -0.1  # type: ignore[index]
        cases.append(("below-floor", below_floor, "maze floor"))

        reference_below_floor = copy.deepcopy(_artifact())
        reference_below_floor["reference_plane_absolute_height_mm"] = -0.1
        cases.append(
            ("reference-below-floor", reference_below_floor, "non-negative")
        )

        at_camera = copy.deepcopy(_artifact())
        at_camera["label_absolute_heights_mm"]["blue_center"] = 700.0  # type: ignore[index]
        cases.append(("label-at-camera", at_camera, "below the camera"))

        camera_below_reference = copy.deepcopy(_artifact())
        camera_below_reference["camera_center_board_mm"]["height"] = 0.0  # type: ignore[index]
        cases.append(
            ("camera-below-reference", camera_below_reference, "above the reference")
        )

        implausibly_low = copy.deepcopy(_artifact())
        implausibly_low["camera_center_board_mm"]["height"] = 50.0  # type: ignore[index]
        cases.append(("camera-too-low", implausibly_low, "100..3000"))

        for label, raw, message in cases:
            with self.subTest(label=label):
                with self.assertRaisesRegex(ValueError, message):
                    self._load(raw)

    def test_loader_requires_explicit_safety_qualification(self):
        missing = copy.deepcopy(_artifact())
        del missing["qualification"]
        with self.assertRaisesRegex(ValueError, "fields are invalid"):
            self._load(missing)

        false = copy.deepcopy(_artifact())
        false["qualification"]["safety_qualified"] = False  # type: ignore[index]
        with self.assertRaisesRegex(ValueError, "explicitly true"):
            self._load(false)

        truthy = copy.deepcopy(_artifact())
        truthy["qualification"]["safety_qualified"] = 1  # type: ignore[index]
        with self.assertRaisesRegex(ValueError, "explicitly true"):
            self._load(truthy)

        no_evidence = copy.deepcopy(_artifact())
        del no_evidence["calibration"]
        with self.assertRaisesRegex(ValueError, "fields are invalid"):
            self._load(no_evidence)

    def test_loader_rejects_invalid_or_mismatched_bindings(self):
        invalid = copy.deepcopy(_artifact())
        invalid["bindings"]["board_layout_sha256"] = "not-a-digest"  # type: ignore[index]
        with self.assertRaisesRegex(ValueError, "64-digit"):
            self._load(invalid)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "label_plane_geometry.json"
            path.write_text(json.dumps(_artifact()), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "board-layout"):
                GEOMETRY.load_geometry(
                    path,
                    expected_board_layout_sha256="34" * 32,
                    expected_tracking_calibration_sha256=TRACKING_SHA,
                )
            with self.assertRaisesRegex(ValueError, "tracking-calibration"):
                GEOMETRY.load_geometry(
                    path,
                    expected_board_layout_sha256=BOARD_SHA,
                    expected_tracking_calibration_sha256="cd" * 32,
                )


if __name__ == "__main__":
    unittest.main()
