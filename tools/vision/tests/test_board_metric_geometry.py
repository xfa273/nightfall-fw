#!/usr/bin/env python3

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
VISION_ROOT = REPO_ROOT / "tools/vision"
sys.path.insert(0, str(VISION_ROOT))

import board_metric_geometry as metric  # noqa: E402
import extract_board_metric_lattice as lattice  # noqa: E402
import markerless_trajectory as markerless  # noqa: E402


REFERENCE_PROVENANCE = {
    "surface": "synthetic flush lattice",
    "height_reference": "maze floor",
    "measurement": "test fixture",
    "confirmed_date": "2026-08-12",
    "remeasure_if": "fixture changes",
}


def distorted_canonical(board: np.ndarray) -> np.ndarray:
    """Independent smooth camera/layout error with ~30 mm corner bias."""
    x = board[:, 0]
    y = board[:, 1]
    xn = (x - 360.0) / 360.0
    yn = (y - 360.0) / 360.0
    apparent_x_mm = 360.0 + 0.935 * (x - 360.0) + 2.4 * xn * yn
    apparent_y_mm = 360.0 + 0.932 * (y - 360.0) - 1.8 * xn * yn
    # Existing 840 mm canvas maps -60..780 mm to 0..899 canonical pixels.
    px_per_mm = 899.0 / 840.0
    return np.column_stack(
        [
            (apparent_x_mm + 60.0) * px_per_mm,
            (780.0 - apparent_y_mm) * px_per_mm,
        ]
    )


class BoardMetricGeometryTest(unittest.TestCase):
    def setUp(self) -> None:
        expected = np.asarray(
            [(45.0 + 90.0 * x, 45.0 + 90.0 * y) for y in range(8) for x in range(8)],
            dtype=float,
        )
        observed = distorted_canonical(expected)
        indexes = np.arange(len(expected))
        validation = ((indexes % 8) + 2 * (indexes // 8)) % 5 == 0
        self.expected = expected
        self.observed = observed
        self.validation = validation
        self.geometry = metric.fit_geometry(
            observed,
            expected,
            validation,
            canonical_size_px=900,
            reference_plane_absolute_height_mm=0.0,
            reference_plane_provenance_value=REFERENCE_PROVENANCE,
            board_layout_sha256="12" * 32,
            source_manifest_sha256="34" * 32,
            degree=3,
        )

    def test_dense_fit_recovers_full_board_and_qualifies(self) -> None:
        mapped = self.geometry.map_points(self.observed)
        errors = np.linalg.norm(mapped - self.expected, axis=1)
        self.assertTrue(self.geometry.safety_qualified)
        self.assertLess(float(np.max(errors)), 1e-3)
        self.assertGreater(self.geometry.calibration_hull_mm2, 300_000.0)

    def test_json_round_trip_binds_layout_and_rejects_unqualified(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "geometry.json"
            path.write_text(
                json.dumps(metric.to_json(self.geometry), allow_nan=False),
                encoding="utf-8",
            )
            loaded = metric.load_geometry(
                path,
                expected_board_layout_sha256="12" * 32,
                expected_canonical_size_px=900,
            )
            np.testing.assert_allclose(
                loaded.map_points(self.observed), self.expected, atol=1e-3
            )
            raw = json.loads(path.read_text(encoding="utf-8"))
            raw["qualification"]["safety_qualified"] = False
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "not safety-qualified"):
                metric.load_geometry(path)

    def test_loader_recomputes_evidence_instead_of_trusting_claim(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "geometry.json"
            raw = metric.to_json(self.geometry)
            raw["model"]["x_coefficients_mm"][0] += 20.0
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "inconsistent"):
                metric.load_geometry(path)

    def test_sparse_corner_only_fit_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "at least 32"):
            metric.fit_geometry(
                self.observed[:4],
                self.expected[:4],
                np.asarray([False, False, False, True]),
                canonical_size_px=900,
                reference_plane_absolute_height_mm=0.0,
                reference_plane_provenance_value=REFERENCE_PROVENANCE,
                board_layout_sha256="12" * 32,
                source_manifest_sha256="34" * 32,
            )

    def test_held_out_local_error_prevents_qualification(self) -> None:
        corrupted = self.expected.copy()
        corrupted[self.validation, 0] += 3.0
        geometry = metric.fit_geometry(
            self.observed,
            corrupted,
            self.validation,
            canonical_size_px=900,
            reference_plane_absolute_height_mm=0.0,
            reference_plane_provenance_value=REFERENCE_PROVENANCE,
            board_layout_sha256="12" * 32,
            source_manifest_sha256="34" * 32,
        )
        self.assertFalse(geometry.safety_qualified)
        self.assertGreater(geometry.validation_stats["p95_mm"], 1.0)

    def test_markerless_metric_output_uses_dense_map(self) -> None:
        detections = []
        for index, point in enumerate(self.observed[:12]):
            detections.append(
                markerless.Detection(
                    frame=index,
                    time_s=index * 0.01,
                    position_xy=point,
                    body_xy=point,
                    cue_xy=np.asarray([np.nan, np.nan]),
                    label_xy=point,
                    front_label_xy=np.asarray([np.nan, np.nan]),
                    yaw_unwrapped_deg=0.0,
                    body_pixel_count=100,
                    green_pixel_count=100,
                    cue_pixel_count=0,
                    label_pixel_count=40,
                    front_label_pixel_count=0,
                    cue_brightness=0.0,
                    axis_anisotropy=1.0,
                    pose_confidence=1.0,
                    pose_valid=True,
                    heading_valid=True,
                    heading_source="test",
                    position_source="label",
                )
            )
        grid = markerless.aruco.GridCalibration(
            x_lines_px=np.linspace(64.2, 835.0, 9),
            y_lines_px=np.linspace(64.2, 835.0, 9),
            x_origin_px=64.2,
            y_origin_px=64.2,
            x_pitch_px=96.35,
            y_pitch_px=96.35,
            cells=8,
            x_peak_contrast=float("nan"),
            y_peak_contrast=float("nan"),
        )
        result = markerless.convert_track(
            detections,
            grid,
            np.arange(len(detections), dtype=float) * 0.01,
            smooth_window=99,
            cell_size_mm=90.0,
            metric_geometry=self.geometry,
        )
        np.testing.assert_allclose(
            np.column_stack([result["x_mm"], result["y_mm"]]),
            self.expected[:12],
            atol=1e-3,
        )

    def test_periodic_lattice_assignment_is_anchored_by_board_layout(self) -> None:
        expected = [
            {
                "point_id": f"p{index}",
                "row": index // 4,
                "column": index % 4,
                "predicted_x_px": float(point[0]),
                "predicted_y_px": float(point[1]),
                "board_x_mm": float(self.expected[index, 0]),
                "board_y_mm": float(self.expected[index, 1]),
                "role": "held_out" if index % 5 == 0 else "fit",
            }
            for index, point in enumerate(self.observed[:16])
        ]
        candidates = [
            {
                "x": float(point[0] + (0.3 if index % 2 else -0.2)),
                "y": float(point[1] - 0.1),
                "area": 55.0,
                "circularity": 0.9,
                "area_error": 0.05,
            }
            for index, point in reversed(list(enumerate(self.observed[:16])))
        ]
        candidates.append(
            {
                "x": 450.0,
                "y": 450.0,
                "area": 55.0,
                "circularity": 0.9,
                "area_error": 0.0,
            }
        )
        assigned = lattice.assign_candidates(expected, candidates, 20.0)
        self.assertEqual(len(assigned), 16)
        self.assertEqual({item["point_id"] for item in assigned}, {f"p{i}" for i in range(16)})

    def test_white_circle_detector_rejects_wall_shaped_distractor(self) -> None:
        layout = markerless.board_layout.load(
            VISION_ROOT / "data/board_layout_8x8_60mm.json", 900
        )
        image = np.zeros((900, 900, 3), dtype=np.uint8)
        for point in self.observed:
            cv2.circle(
                image,
                tuple(map(int, np.rint(point))),
                4,
                (245, 245, 245),
                -1,
                cv2.LINE_AA,
            )
        cv2.rectangle(image, (180, 360), (600, 372), (255, 255, 255), -1)
        candidates, _ = lattice._candidate_centres(
            image,
            layout,
            dot_diameter_mm=8.0,
            minimum_value=155,
            maximum_saturation=105,
        )
        self.assertEqual(len(candidates), 64)


if __name__ == "__main__":
    unittest.main()
