#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import sys
import tempfile
import unittest
from collections import deque
from pathlib import Path
from unittest import mock

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
VISION_ROOT = REPO_ROOT / "tools/vision"
sys.path.insert(0, str(VISION_ROOT))


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


FUSE = load_module(
    "test_fuse_trace_video",
    REPO_ROOT / "tools/vision/fuse_trace_video.py",
)
TURN = load_module(
    "test_turn_video_tune",
    REPO_ROOT / "tools/tuning/turn_video_tune.py",
)
try:
    import cv2  # noqa: F401
except ModuleNotFoundError:
    MARKERLESS = None
else:
    load_module(
        "aruco_trajectory",
        REPO_ROOT / "tools/vision/aruco_trajectory.py",
    )
    FRONT_LABEL_CALIBRATION = load_module(
        "test_fit_front_label_heading",
        REPO_ROOT / "tools/vision/fit_front_label_heading.py",
    )
    MARKERLESS = load_module(
        "test_markerless_trajectory",
        REPO_ROOT / "tools/vision/markerless_trajectory.py",
    )


class TraceVideoAlignmentTest(unittest.TestCase):
    def test_recovers_motion_offset_and_comment_only_trace_header(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            video_path = root / "trajectory.csv"
            trace_path = root / "trace.csv"
            video_time = np.arange(0.0, 4.0, 1.0 / 120.0)
            omega = (
                700.0 * np.exp(-(((video_time - 1.1) / 0.12) ** 2))
                - 430.0 * np.exp(-(((video_time - 2.2) / 0.18) ** 2))
                + 280.0 * np.exp(-(((video_time - 3.0) / 0.10) ** 2))
            )
            yaw = np.cumsum(omega) / 120.0
            with video_path.open("w", newline="", encoding="ascii") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=(
                        "video_pts_s",
                        "x_mm",
                        "y_mm",
                        "yaw_deg_unwrapped",
                        "speed_mm_s",
                    ),
                )
                writer.writeheader()
                for index, time_s in enumerate(video_time):
                    writer.writerow(
                        {
                            "video_pts_s": f"{time_s:.9f}",
                            "x_mm": "0",
                            "y_mm": f"{time_s * 100.0:.6f}",
                            "yaw_deg_unwrapped": f"{yaw[index]:.9f}",
                            "speed_mm_s": "100",
                        }
                    )

            known_offset = 0.370
            trace_time = np.arange(0.0, 4.8, 0.001)
            video_query = trace_time - known_offset
            trace_omega = np.interp(
                video_query,
                video_time,
                omega,
                left=0.0,
                right=0.0,
            )
            with trace_path.open("w", newline="", encoding="ascii") as stream:
                stream.write(
                    "#mm_columns=timestamp_ms,real_omega_mdps,"
                    "real_velocity_mm_s,flags\n"
                )
                for index, time_s in enumerate(trace_time):
                    stream.write(
                        "{},{},{},{}\n".format(
                            int(round(time_s * 1000.0)),
                            int(round(trace_omega[index] * 1000.0)),
                            100,
                            0,
                        )
                    )

            video = FUSE.load_video(video_path)
            trace, _, rows = FUSE.load_trace(trace_path)
            self.assertEqual(len(rows), len(trace_time))
            args = argparse.Namespace(
                activity_padding_s=0.30,
                estimate_drift=False,
                maximum_drift_ppm=5000.0,
                drift_steps=21,
                offset_step_ms=2.0,
                offset_search_s=1.0,
                minimum_overlap_s=0.40,
            )
            alignment = FUSE.align(video, trace, "yaw", args)
            self.assertAlmostEqual(alignment.offset_s, known_offset, delta=0.010)
            self.assertGreater(alignment.correlation, 0.97)
            self.assertGreater(alignment.correlation_margin, 0.02)
            self.assertGreater(alignment.signal_gain, 0.5)
            self.assertLess(alignment.signal_gain, 2.0)
            self.assertEqual(alignment.sign, 1)


class TurnCoordinateTest(unittest.TestCase):
    def test_board_pose_is_converted_to_right_forward_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "turn.csv"
            time_s = np.linspace(0.0, 1.6, 161)
            progress = np.clip((time_s - 0.3) / 1.0, 0.0, 1.0)
            x = 100.0 + 10.0 * progress
            y = 200.0 + 20.0 * progress
            yaw = 90.0 + 90.0 * progress
            speed = np.where(
                (time_s >= 0.3) & (time_s <= 1.3),
                math.hypot(10.0, 20.0),
                0.0,
            )
            with path.open("w", newline="", encoding="ascii") as stream:
                writer = csv.DictWriter(
                    stream,
                    fieldnames=(
                        "video_pts_s",
                        "x_mm",
                        "y_mm",
                        "yaw_deg_unwrapped",
                        "speed_mm_s",
                        "pose_valid",
                    ),
                )
                writer.writeheader()
                for index, value in enumerate(time_s):
                    writer.writerow(
                        {
                            "video_pts_s": value,
                            "x_mm": x[index],
                            "y_mm": y[index],
                            "yaw_deg_unwrapped": yaw[index],
                            "speed_mm_s": speed[index],
                            "pose_valid": 1,
                        }
                    )
            args = argparse.Namespace(
                cell_size_mm=None,
                minimum_valid_fraction=0.95,
                minimum_heading_valid_fraction=0.99,
                heading_source="label",
                trajectory_heading_inner_fraction=0.08,
                trajectory_heading_outer_fraction=0.32,
                minimum_trajectory_heading_span_mm=3.0,
                maximum_trajectory_heading_residual_mm=1.0,
                anchor_right_mm=0.0,
                anchor_forward_mm=0.0,
                start_s=None,
                end_s=None,
                motion_threshold_mm_s=20.0,
                pose_window_ms=150.0,
                maximum_pose_window_speed_mm_s=10.0,
                minimum_pose_window_coverage=0.8,
                side="right",
            )
            trial = TURN.analyze_trial(path, args)
            self.assertAlmostEqual(trial.endpoint.x_right_mm, 10.0, delta=0.3)
            self.assertAlmostEqual(
                trial.endpoint.y_forward_mm,
                20.0,
                delta=0.3,
            )
            self.assertAlmostEqual(trial.endpoint.theta_deg, 90.0, delta=1.0)
            self.assertEqual(trial.tracking_valid_fraction, 1.0)
            self.assertEqual(trial.heading_valid_fraction, 1.0)

            args.heading_source = "trajectory"
            trajectory_trial = TURN.analyze_trial(path, args)
            self.assertAlmostEqual(
                trajectory_trial.endpoint.x_right_mm,
                0.0,
                delta=0.3,
            )
            self.assertAlmostEqual(
                trajectory_trial.endpoint.y_forward_mm,
                math.hypot(10.0, 20.0),
                delta=0.3,
            )
            self.assertAlmostEqual(
                trajectory_trial.endpoint.theta_deg,
                0.0,
                delta=0.2,
            )
            self.assertEqual(trajectory_trial.heading_source, "trajectory")
            self.assertGreater(trajectory_trial.start_heading_fit_span_mm, 3.0)
            self.assertLess(
                trajectory_trial.start_heading_fit_residual_p95_mm,
                0.01,
            )

            args.heading_source = "trajectory-start"
            trajectory_start_trial = TURN.analyze_trial(path, args)
            self.assertAlmostEqual(
                trajectory_start_trial.endpoint.x_right_mm,
                0.0,
                delta=0.3,
            )
            self.assertAlmostEqual(
                trajectory_start_trial.endpoint.y_forward_mm,
                math.hypot(10.0, 20.0),
                delta=0.3,
            )
            self.assertAlmostEqual(
                trajectory_start_trial.endpoint.theta_deg,
                116.565,
                delta=1.0,
            )
            self.assertEqual(
                trajectory_start_trial.heading_source,
                "trajectory-start",
            )
            self.assertIsNone(trajectory_start_trial.end_heading_fit_span_mm)

    def test_trajectory_heading_rejects_a_curved_fit_window(self):
        angle = np.linspace(0.0, math.pi / 2.0, 101)
        x = 50.0 * np.cos(angle)
        y = 50.0 * np.sin(angle)
        cumulative = np.concatenate(
            ([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y))))
        )
        with self.assertRaisesRegex(ValueError, "residual p95"):
            TURN._fit_trajectory_heading(
                x,
                y,
                cumulative,
                0.0,
                1.0,
                minimum_span_mm=30.0,
                maximum_residual_mm=3.0,
                label="synthetic",
            )


@unittest.skipUnless(MARKERLESS is not None, "OpenCV contrib is unavailable")
class MarkerlessSafetyGateTest(unittest.TestCase):
    def test_front_label_circle_fit_recovers_parallax_bias(self):
        heading_deg = np.linspace(-100.0, 100.0, 401)
        heading_rad = np.radians(heading_deg)
        expected_bias_px = np.asarray([-3.4, -2.1])
        radius_px = 28.0
        vectors = expected_bias_px + radius_px * np.column_stack(
            [np.cos(heading_rad), -np.sin(heading_rad)]
        )
        result = FRONT_LABEL_CALIBRATION.fit_bias(
            vectors,
            pixels_per_mm=1.1,
        )
        np.testing.assert_allclose(result["bias_px"], expected_bias_px, atol=1e-9)
        self.assertAlmostEqual(result["radius_px"], radius_px)
        self.assertAlmostEqual(result["heading_span_deg"], 200.0)
        self.assertLess(float(np.max(result["radial_error_mm"])), 1e-9)

    def test_front_label_calibration_loader_checks_baseline(self):
        calibration = {
            "schema": MARKERLESS.FRONT_LABEL_CALIBRATION_SCHEMA,
            "coordinate_system": "x_right_y_forward_mm",
            "front_label_distance_mm": 24.0,
            "apparent_vector_bias_mm": {"right": -3.1, "forward": 1.8},
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "front_label.json"
            path.write_text(json.dumps(calibration), encoding="utf-8")
            result = MARKERLESS.load_front_label_calibration(path, 24.0)
            self.assertAlmostEqual(result["bias_right_mm"], -3.1)
            self.assertAlmostEqual(result["bias_forward_mm"], 1.8)
            with self.assertRaisesRegex(ValueError, "distance does not match"):
                MARKERLESS.load_front_label_calibration(path, 30.0)

    def test_tracker_prediction_advances_across_missing_frames(self):
        predicted = MARKERLESS._predict_tracker_position(
            np.asarray([100.0, 200.0]),
            np.asarray([-3.0, 1.5]),
            observed_frame=20,
            frame_index=27,
        )
        np.testing.assert_allclose(predicted, [79.0, 210.5])

    def test_tracker_velocity_uses_elapsed_observation_frames(self):
        updated = MARKERLESS._update_tracker_velocity(
            np.asarray([-3.0, 0.0]),
            np.asarray([100.0, 200.0]),
            np.asarray([82.0, 206.0]),
            previous_frame=20,
            frame_index=26,
        )
        np.testing.assert_allclose(updated, [-3.0, 0.35])

    def test_invalid_ffprobe_timestamps_are_not_silently_retimed(self):
        completed = argparse.Namespace(
            returncode=0,
            stdout="0.000000\n0.008333\n",
            stderr="",
        )
        with (
            mock.patch.object(
                MARKERLESS.shutil,
                "which",
                return_value="/usr/bin/ffprobe",
            ),
            mock.patch.object(
                MARKERLESS.subprocess,
                "run",
                return_value=completed,
            ),
        ):
            with self.assertRaisesRegex(RuntimeError, "2 timestamps"):
                MARKERLESS.video_timestamps(
                    Path("capture.mp4"),
                    frame_count=3,
                    fps=120.0,
                )

    def test_initial_seed_prefers_largest_component_in_gate(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[46:55, 46:55] = 255
        mask[35:65, 68:88] = 255
        mask[70:100, 0:30] = 255
        seed = np.asarray((50.0, 50.0))
        near, near_count, _ = MARKERLESS._component_near(
            mask,
            10,
            1000,
            seed,
            maximum_distance=40.0,
        )
        largest, largest_count, _ = MARKERLESS._component_near(
            mask,
            10,
            1000,
            seed,
            maximum_distance=40.0,
            prefer_largest=True,
        )
        self.assertIsNotNone(near)
        self.assertIsNotNone(largest)
        self.assertEqual(near_count, 81)
        self.assertEqual(largest_count, 600)

    def test_cue_geometry_and_yaw_rate_reject_spurious_flip(self):
        recent = [38.0, 39.0, 37.0, 40.0, 38.5]
        self.assertFalse(
            MARKERLESS._cue_distance_is_valid(
                3.0,
                recent,
                10.0,
                0.90,
            )
        )
        self.assertTrue(
            MARKERLESS._cue_distance_is_valid(
                65.0,
                recent,
                10.0,
                0.90,
            )
        )
        self.assertFalse(
            MARKERLESS._cue_distance_is_valid(
                80.0,
                recent,
                10.0,
                0.90,
            )
        )
        self.assertFalse(
            MARKERLESS._yaw_innovation_is_valid(
                360.0,
                180.0,
                3000.0,
                1.0 / 120.0,
            )
        )
        self.assertTrue(
            MARKERLESS._yaw_innovation_is_valid(
                198.0,
                180.0,
                3000.0,
                1.0 / 120.0,
            )
        )

    def test_rejected_cue_is_not_used_as_position_or_prediction(self):
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        background = np.zeros_like(frame)
        component = np.zeros((100, 100), dtype=np.uint8)
        component[40:61, 40:61] = 255
        grid = sys.modules["aruco_trajectory"].GridCalibration(
            x_lines_px=np.asarray([10.0, 90.0]),
            y_lines_px=np.asarray([10.0, 90.0]),
            x_origin_px=10.0,
            y_origin_px=10.0,
            x_pitch_px=80.0,
            y_pitch_px=80.0,
            cells=1,
            x_peak_contrast=1.0,
            y_peak_contrast=1.0,
        )
        args = argparse.Namespace(
            foreground_blur=3,
            foreground_threshold=1,
            morph_open=1,
            morph_close=1,
            tracking_radius_px=40.0,
            minimum_green_pixels=20,
            minimum_body_pixels=80,
            cue_colour="red",
            label_colour="none",
            front_label_colour="none",
            minimum_label_pixels=20,
            maximum_label_pixels=180,
            minimum_cue_pixels=1,
            maximum_cue_pixels=100,
            minimum_cue_lever_arm_px=10.0,
            cue_distance_relative_tolerance=0.9,
            cue_yaw_offset_deg=0.0,
            initial_yaw_deg=0.0,
            maximum_yaw_rate_deg_s=3000.0,
            minimum_axis_anisotropy=0.5,
            axis_yaw_offset_deg=0.0,
            position_source="cue",
        )
        with (
            mock.patch.object(
                MARKERLESS,
                "green_mask",
                return_value=component,
            ),
            mock.patch.object(
                MARKERLESS,
                "red_mask",
                return_value=component,
            ),
            mock.patch.object(
                MARKERLESS,
                "_component_near",
                side_effect=(
                    (np.asarray([50.0, 50.0]), 441, component),
                    (np.asarray([52.0, 50.0]), 8, component),
                ),
            ),
            mock.patch.object(
                MARKERLESS,
                "_foreground_cluster",
                return_value=(
                    np.asarray([50.0, 50.0]),
                    441,
                    component,
                ),
            ),
            mock.patch.object(
                MARKERLESS,
                "_principal_axis",
                return_value=(0.0, 1.0),
            ),
        ):
            result = MARKERLESS.detect_pose(
                frame,
                background,
                grid,
                args,
                np.asarray([50.0, 50.0]),
                0.0,
                np.asarray([52.0, 50.0]),
                None,
                None,
                50.0,
                50.0,
                24.0,
                8.0,
                np.zeros(2, dtype=float),
                deque([38.0, 39.0, 37.0, 40.0, 38.5]),
                False,
                1.0 / 120.0,
            )
        self.assertFalse(np.allclose(result[0], [52.0, 50.0]))
        self.assertFalse(np.all(np.isfinite(result[2])))
        self.assertEqual(result[-2], "cue_rejected_geometry")
        self.assertNotEqual(result[-1], "cue")

    def test_green_position_source_uses_green_centroid(self):
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        background = np.zeros_like(frame)
        component = np.zeros((100, 100), dtype=np.uint8)
        component[40:61, 40:61] = 255
        grid = sys.modules["aruco_trajectory"].GridCalibration(
            x_lines_px=np.asarray([10.0, 90.0]),
            y_lines_px=np.asarray([10.0, 90.0]),
            x_origin_px=10.0,
            y_origin_px=10.0,
            x_pitch_px=80.0,
            y_pitch_px=80.0,
            cells=1,
            x_peak_contrast=1.0,
            y_peak_contrast=1.0,
        )
        args = argparse.Namespace(
            foreground_blur=3,
            foreground_threshold=1,
            morph_open=1,
            morph_close=1,
            tracking_radius_px=40.0,
            minimum_green_pixels=20,
            minimum_body_pixels=80,
            cue_colour="none",
            label_colour="none",
            front_label_colour="none",
            minimum_label_pixels=20,
            maximum_label_pixels=180,
            minimum_cue_pixels=1,
            maximum_cue_pixels=100,
            minimum_cue_lever_arm_px=10.0,
            cue_distance_relative_tolerance=0.9,
            cue_yaw_offset_deg=0.0,
            initial_yaw_deg=0.0,
            maximum_yaw_rate_deg_s=3000.0,
            minimum_axis_anisotropy=0.5,
            axis_yaw_offset_deg=0.0,
            position_source="green",
        )
        green_xy = np.asarray([45.0, 55.0])
        body_xy = np.asarray([50.0, 50.0])
        with (
            mock.patch.object(
                MARKERLESS,
                "green_mask",
                return_value=component,
            ),
            mock.patch.object(
                MARKERLESS,
                "_component_near",
                return_value=(green_xy, 441, component),
            ),
            mock.patch.object(
                MARKERLESS,
                "_foreground_cluster",
                return_value=(body_xy, 441, component),
            ),
            mock.patch.object(
                MARKERLESS,
                "_principal_axis",
                return_value=(0.0, 1.0),
            ),
        ):
            result = MARKERLESS.detect_pose(
                frame,
                background,
                grid,
                args,
                green_xy,
                0.0,
                None,
                None,
                None,
                50.0,
                50.0,
                24.0,
                8.0,
                np.zeros(2, dtype=float),
                deque(),
                False,
                1.0 / 120.0,
            )
        np.testing.assert_allclose(result[0], green_xy)
        self.assertEqual(result[-1], "green")

    def test_blue_label_mask_excludes_teal_pcb_and_bluer_led(self):
        frame = np.zeros((1, 3, 3), dtype=np.uint8)
        frame[0, 0] = (230, 173, 58)
        frame[0, 1] = (140, 150, 30)
        led_hsv = np.asarray([[[115, 220, 220]]], dtype=np.uint8)
        frame[0, 2] = MARKERLESS.cv2.cvtColor(
            led_hsv,
            MARKERLESS.cv2.COLOR_HSV2BGR,
        )[0, 0]
        mask = MARKERLESS.blue_label_mask(frame)
        self.assertEqual(mask[0].tolist(), [255, 0, 0])

    def test_blue_label_glare_mask_recovers_only_desaturated_blue(self):
        hsv = np.asarray(
            [[[98, 20, 240], [0, 0, 240], [115, 220, 220]]],
            dtype=np.uint8,
        )
        frame = MARKERLESS.cv2.cvtColor(hsv, MARKERLESS.cv2.COLOR_HSV2BGR)
        frame[0, 1] = (140, 150, 30)

        strict = MARKERLESS.blue_label_mask(frame)
        glare = MARKERLESS.blue_label_glare_mask(frame)

        self.assertEqual(strict[0].tolist(), [0, 0, 0])
        self.assertEqual(glare[0].tolist(), [255, 0, 0])

    def test_label_component_prefers_expected_area_and_prediction(self):
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[46:54, 46:54] = 255
        mask[35:46, 60:71] = 255
        center, count, _ = MARKERLESS._label_component(
            mask,
            minimum_pixels=20,
            maximum_pixels=180,
            expected_pixels=64.0,
            prediction_xy=np.asarray([50.0, 50.0]),
            maximum_distance=45.0,
        )
        self.assertEqual(count, 64)
        np.testing.assert_allclose(center, [49.5, 49.5])

    def test_initial_position_seeds_primary_blue_label_tracker(self):
        args = argparse.Namespace(
            initial_x_cell=7.38,
            initial_y_cell=0.75,
            position_source="label",
            label_colour="blue",
        )
        grid = argparse.Namespace(
            x_origin_px=64.0,
            y_origin_px=64.0,
            x_pitch_px=96.0,
            y_pitch_px=96.0,
            cells=8,
        )

        green_seed, label_seed = MARKERLESS._initial_tracking_seeds(args, grid)

        np.testing.assert_allclose(green_seed, (772.48, 760.0), atol=1e-9)
        np.testing.assert_allclose(label_seed, green_seed, atol=1e-9)
        self.assertIsNot(label_seed, green_seed)

    def test_initial_position_does_not_seed_an_unused_label_tracker(self):
        args = argparse.Namespace(
            initial_x_cell=1.0,
            initial_y_cell=2.0,
            position_source="green",
            label_colour="blue",
        )
        grid = argparse.Namespace(
            x_origin_px=10.0,
            y_origin_px=20.0,
            x_pitch_px=30.0,
            y_pitch_px=40.0,
            cells=4,
        )

        green_seed, label_seed = MARKERLESS._initial_tracking_seeds(args, grid)

        np.testing.assert_allclose(green_seed, (40.0, 100.0), atol=1e-9)
        self.assertIsNone(label_seed)

    def test_front_label_component_rejects_rear_led_and_wall(self):
        mask = np.zeros((120, 120), dtype=np.uint8)
        mask[46:54, 70:78] = 255
        mask[46:51, 40:45] = 255
        mask[20:100, 95:99] = 255
        center, count, _ = MARKERLESS._front_label_component(
            mask,
            center_xy=np.asarray([49.5, 49.5]),
            minimum_pixels=20,
            maximum_pixels=180,
            expected_pixels=64.0,
            expected_distance_px=24.0,
            distance_tolerance_px=5.0,
            prediction_xy=None,
            maximum_prediction_distance_px=45.0,
        )
        self.assertEqual(count, 64)
        np.testing.assert_allclose(center, [73.5, 49.5])

    def test_label_pair_rejects_blue_led_adjacent_to_front_label(self):
        blue = np.zeros((120, 120), dtype=np.uint8)
        red = np.zeros_like(blue)
        blue[46:54, 46:54] = 255
        blue[46:56, 68:78] = 255
        red[46:54, 70:78] = 255

        center, center_count, front, front_count = (
            MARKERLESS._blue_front_label_pair(
                blue,
                red,
                minimum_blue_pixels=6,
                maximum_blue_pixels=180,
                expected_blue_pixels=64.0,
                minimum_red_pixels=20,
                maximum_red_pixels=180,
                expected_red_pixels=64.0,
                expected_distance_px=24.0,
                distance_tolerance_px=5.0,
                prediction_xy=np.asarray([55.0, 50.0]),
                maximum_prediction_distance_px=30.0,
            )
        )

        self.assertEqual(center_count, 64)
        self.assertEqual(front_count, 64)
        np.testing.assert_allclose(center, [49.5, 49.5])
        np.testing.assert_allclose(front, [73.5, 49.5])

    def test_front_label_mask_accepts_orange_shifted_red(self):
        hsv = np.zeros((24, 48, 3), dtype=np.uint8)
        hsv[6:18, 6:18] = (21, 48, 220)
        hsv[6:18, 30:42] = (174, 110, 180)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        front = MARKERLESS.front_label_mask(frame)
        legacy = MARKERLESS.red_mask(frame)

        self.assertGreater(np.count_nonzero(front[6:18, 6:18]), 100)
        self.assertEqual(np.count_nonzero(legacy[6:18, 6:18]), 0)
        self.assertGreater(np.count_nonzero(front[6:18, 30:42]), 100)

    def test_front_label_mask_rejects_low_saturation_board(self):
        hsv = np.zeros((24, 24, 3), dtype=np.uint8)
        hsv[6:18, 6:18] = (21, 30, 220)
        frame = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        self.assertEqual(np.count_nonzero(MARKERLESS.front_label_mask(frame)), 0)

    def test_front_label_provides_heading_without_body_segmentation(self):
        frame = np.zeros((120, 120, 3), dtype=np.uint8)
        background = np.zeros_like(frame)
        blue = np.zeros((120, 120), dtype=np.uint8)
        red = np.zeros_like(blue)
        blue[46:54, 46:54] = 255
        red[46:54, 70:78] = 255
        grid = sys.modules["aruco_trajectory"].GridCalibration(
            x_lines_px=np.asarray([10.0, 110.0]),
            y_lines_px=np.asarray([10.0, 110.0]),
            x_origin_px=10.0,
            y_origin_px=10.0,
            x_pitch_px=100.0,
            y_pitch_px=100.0,
            cells=1,
            x_peak_contrast=1.0,
            y_peak_contrast=1.0,
        )
        args = argparse.Namespace(
            foreground_blur=3,
            foreground_threshold=1,
            morph_open=1,
            morph_close=1,
            tracking_radius_px=45.0,
            minimum_green_pixels=20,
            minimum_body_pixels=80,
            cue_colour="none",
            label_colour="blue",
            front_label_colour="red",
            minimum_label_pixels=20,
            maximum_label_pixels=180,
            minimum_front_label_pixels=20,
            maximum_front_label_pixels=180,
            minimum_cue_pixels=1,
            maximum_cue_pixels=100,
            minimum_cue_lever_arm_px=10.0,
            cue_distance_relative_tolerance=0.9,
            cue_yaw_offset_deg=0.0,
            front_label_yaw_offset_deg=0.0,
            initial_yaw_deg=None,
            maximum_yaw_rate_deg_s=3000.0,
            minimum_axis_anisotropy=0.5,
            axis_yaw_offset_deg=0.0,
            position_source="label",
        )
        with (
            mock.patch.object(MARKERLESS, "blue_label_mask", return_value=blue),
            mock.patch.object(MARKERLESS, "front_label_mask", return_value=red),
            mock.patch.object(
                MARKERLESS,
                "green_mask",
                return_value=np.zeros_like(blue),
            ),
        ):
            result = MARKERLESS.detect_pose(
                frame,
                background,
                grid,
                args,
                None,
                None,
                None,
                None,
                None,
                64.0,
                64.0,
                24.0,
                5.0,
                np.zeros(2, dtype=float),
                deque(),
                False,
                1.0 / 240.0,
            )
        np.testing.assert_allclose(result[0], [49.5, 49.5])
        np.testing.assert_allclose(result[4], [73.5, 49.5])
        self.assertAlmostEqual(result[6], 0.0)
        self.assertTrue(result[-3])
        self.assertEqual(result[-2], "front_label")

    def test_green_tracker_seeds_glare_recovery_for_blue_label(self):
        frame = np.zeros((120, 120, 3), dtype=np.uint8)
        background = np.zeros_like(frame)
        strict_blue = np.zeros((120, 120), dtype=np.uint8)
        glare_blue = np.zeros_like(strict_blue)
        green = np.zeros_like(strict_blue)
        red = np.zeros_like(strict_blue)
        glare_blue[46:54, 46:54] = 255
        green[38:62, 38:62] = 255
        red[46:54, 70:78] = 255
        grid = sys.modules["aruco_trajectory"].GridCalibration(
            x_lines_px=np.asarray([10.0, 110.0]),
            y_lines_px=np.asarray([10.0, 110.0]),
            x_origin_px=10.0,
            y_origin_px=10.0,
            x_pitch_px=100.0,
            y_pitch_px=100.0,
            cells=1,
            x_peak_contrast=1.0,
            y_peak_contrast=1.0,
        )
        args = argparse.Namespace(
            foreground_blur=3,
            foreground_threshold=1,
            morph_open=1,
            morph_close=1,
            tracking_radius_px=45.0,
            minimum_green_pixels=20,
            minimum_body_pixels=80,
            cue_colour="none",
            label_colour="blue",
            front_label_colour="red",
            minimum_label_pixels=20,
            maximum_label_pixels=180,
            minimum_front_label_pixels=20,
            maximum_front_label_pixels=180,
            minimum_cue_pixels=1,
            maximum_cue_pixels=100,
            minimum_cue_lever_arm_px=10.0,
            cue_distance_relative_tolerance=0.9,
            cue_yaw_offset_deg=0.0,
            front_label_yaw_offset_deg=0.0,
            initial_yaw_deg=None,
            maximum_yaw_rate_deg_s=3000.0,
            minimum_axis_anisotropy=0.5,
            axis_yaw_offset_deg=0.0,
            position_source="label",
        )
        with (
            mock.patch.object(
                MARKERLESS,
                "blue_label_mask",
                return_value=strict_blue,
            ),
            mock.patch.object(
                MARKERLESS,
                "blue_label_glare_mask",
                return_value=glare_blue,
            ),
            mock.patch.object(MARKERLESS, "green_mask", return_value=green),
            mock.patch.object(MARKERLESS, "front_label_mask", return_value=red),
        ):
            result = MARKERLESS.detect_pose(
                frame,
                background,
                grid,
                args,
                None,
                None,
                None,
                None,
                None,
                64.0,
                64.0,
                24.0,
                5.0,
                np.zeros(2, dtype=float),
                deque(),
                False,
                1.0 / 240.0,
            )

        np.testing.assert_allclose(result[0], [49.5, 49.5])
        np.testing.assert_allclose(result[4], [73.5, 49.5])
        self.assertTrue(result[-3])
        self.assertEqual(result[-2], "front_label")

    def test_position_only_interpolates_without_valid_heading(self):
        detections = []
        for index in range(3):
            position = np.asarray([10.0 + index, 20.0], dtype=float)
            nan_xy = np.full(2, np.nan, dtype=float)
            detections.append(
                MARKERLESS.Detection(
                    frame=index,
                    time_s=index / 240.0,
                    position_xy=position,
                    body_xy=nan_xy,
                    cue_xy=nan_xy,
                    label_xy=position,
                    front_label_xy=nan_xy,
                    yaw_unwrapped_deg=float("nan"),
                    body_pixel_count=0,
                    green_pixel_count=0,
                    cue_pixel_count=0,
                    label_pixel_count=55,
                    front_label_pixel_count=0,
                    cue_brightness=0.0,
                    axis_anisotropy=0.0,
                    pose_confidence=1.0,
                    pose_valid=True,
                    heading_valid=False,
                    heading_source="missing",
                    position_source="blue_label",
                )
            )
        xy, yaw, valid, heading_valid = MARKERLESS.interpolate_detections(
            detections,
            allow_missing_heading=True,
            fallback_yaw_deg=15.0,
        )
        np.testing.assert_allclose(xy[:, 0], [10.0, 11.0, 12.0])
        np.testing.assert_allclose(yaw, [15.0, 15.0, 15.0])
        self.assertTrue(np.all(valid))
        self.assertFalse(np.any(heading_valid))


class TurnProposalSafetyGateTest(unittest.TestCase):
    @staticmethod
    def _proposal_args(code=501):
        return argparse.Namespace(
            minimum_fit_trials=3,
            maximum_endpoint_std_mm=5.0,
            maximum_yaw_std_deg=2.0,
            runner="shortest",
            mode=2,
            code=code,
            search_index=0,
            side="right",
            entry_speed=None,
            out_speed=None,
            maximum_turn_yaw_error_deg=30.0,
            feedback_gain=0.5,
            vary="dist_in,dist_out,angle",
            maximum_velocity_step_mm_s=250.0,
            maximum_alpha_step_deg_s2=5000.0,
            maximum_offset_step_mm=5.0,
            maximum_angle_step_deg=5.0,
        )

    def test_duplicate_trial_path_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trial.csv"
            path.write_text("video_pts_s,x_mm,y_mm,yaw_deg\n", encoding="ascii")
            with mock.patch.object(
                sys,
                "argv",
                ["turn_video_tune.py", str(path), str(path)],
            ):
                args = TURN.parse_args()
            with self.assertRaisesRegex(ValueError, "same trajectory path"):
                TURN.validate_args(args)

    def test_nan_safety_limit_is_rejected_before_analysis(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trial.csv"
            path.write_text(
                "video_pts_s,x_mm,y_mm,yaw_deg\n",
                encoding="ascii",
            )
            with mock.patch.object(
                sys,
                "argv",
                [
                    "turn_video_tune.py",
                    str(path),
                    "--maximum-angle-step-deg",
                    "nan",
                ],
            ):
                args = TURN.parse_args()
            with self.assertRaisesRegex(
                ValueError,
                "--maximum-angle-step-deg must be finite",
            ):
                TURN.validate_args(args)

    def test_copied_trial_content_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = root / "trial01.csv"
            second = root / "trial02.csv"
            content = "video_pts_s,x_mm,y_mm,yaw_deg\n"
            first.write_text(content, encoding="ascii")
            second.write_text(content, encoding="ascii")
            with mock.patch.object(
                sys,
                "argv",
                ["turn_video_tune.py", str(first), str(second)],
            ):
                args = TURN.parse_args()
            with self.assertRaisesRegex(ValueError, "duplicate trajectory content"):
                TURN.validate_args(args)

    def test_parameter_proposal_requires_height_corrected_trajectory(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trial.csv"
            path.write_text(
                "video_pts_s,x_mm,y_mm,yaw_deg\n",
                encoding="ascii",
            )
            with mock.patch.object(
                sys,
                "argv",
                ["turn_video_tune.py", str(path), "--propose-fit"],
            ):
                args = TURN.parse_args()
            with self.assertRaisesRegex(ValueError, "height-correction sidecar"):
                TURN.validate_args(args)

    def test_parameter_proposal_requires_blue_centre_anchor(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trajectory = root / "trial.csv"
            sidecar = root / "trial.calibration.json"
            trajectory.write_text(
                "video_pts_s,x_mm,y_mm,yaw_deg\n",
                encoding="ascii",
            )
            sidecar.write_text("{}\n", encoding="ascii")
            with mock.patch.object(
                sys,
                "argv",
                [
                    "turn_video_tune.py",
                    str(trajectory),
                    "--propose-fit",
                    "--height-correction-sidecar",
                    str(sidecar),
                    "--anchor-right-mm",
                    "1",
                ],
            ):
                args = TURN.parse_args()
            with self.assertRaisesRegex(ValueError, "zero anchor"):
                TURN.validate_args(args)

    def test_parameter_proposal_safety_gates_cannot_be_weakened(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trajectory = root / "trial.csv"
            sidecar = root / "trial.calibration.json"
            trajectory.write_text(
                "video_pts_s,x_mm,y_mm,yaw_deg\n",
                encoding="ascii",
            )
            sidecar.write_text("{}\n", encoding="ascii")
            with mock.patch.object(
                sys,
                "argv",
                [
                    "turn_video_tune.py",
                    str(trajectory),
                    "--propose-fit",
                    "--height-correction-sidecar",
                    str(sidecar),
                    "--minimum-fit-trials",
                    "1",
                ],
            ):
                args = TURN.parse_args()
            with self.assertRaisesRegex(ValueError, "five-trial safety floor"):
                TURN.validate_args(args)

    def test_height_correction_set_must_share_calibration_digests(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trajectories = [root / "trial01.csv", root / "trial02.csv"]
            sidecars = [root / "trial01.json", root / "trial02.json"]
            for index, path in enumerate(trajectories):
                path.write_text(f"trial,{index}\n", encoding="ascii")
            for index, path in enumerate(sidecars):
                path.write_text(f"{{\"trial\": {index}}}\n", encoding="ascii")
            verified = [
                {
                    "bindings": {
                        "board_layout_sha256": "11" * 32,
                        "tracking_geometry_sha256": "22" * 32,
                        "label_plane_geometry_sha256": "33" * 32,
                    }
                },
                {
                    "bindings": {
                        "board_layout_sha256": "11" * 32,
                        "tracking_geometry_sha256": "22" * 32,
                        "label_plane_geometry_sha256": "44" * 32,
                    }
                },
            ]
            with self.assertRaisesRegex(ValueError, "different calibration"):
                TURN._validate_height_correction_set(
                    trajectories,
                    sidecars,
                    verified,
                )

    def test_wrong_turn_sign_is_rejected_before_fit(self):
        endpoint = {
            "x_right_mm": {"median": 90.0, "std": 0.5},
            "y_forward_mm": {"median": 90.0, "std": 0.5},
            "theta_deg": {"median": 90.0, "std": 0.5},
        }
        aggregate = {"count": 3, "endpoint": endpoint}
        with self.assertRaisesRegex(ValueError, "wrong sign"):
            TURN.propose_fit(self._proposal_args(), aggregate)

    def test_implausible_unwrapped_yaw_is_rejected_before_fit(self):
        endpoint = {
            "x_right_mm": {"median": 180.0, "std": 0.5},
            "y_forward_mm": {"median": 220.0, "std": 0.5},
            "theta_deg": {"median": -1792.0, "std": 0.5},
        }
        aggregate = {"count": 3, "endpoint": endpoint}
        with self.assertRaisesRegex(ValueError, "differs"):
            TURN.propose_fit(self._proposal_args(code=502), aggregate)

    def test_candidate_delta_limit_rejects_large_step(self):
        with self.assertRaisesRegex(ValueError, "angle step"):
            TURN._validate_candidate_deltas(
                {"angle": -12.0, "dist_in": 1.0},
                {"angle", "dist_in"},
                {"angle": 5.0, "dist_in": 5.0},
            )

    def test_matching_measurement_produces_bounded_noop_candidate(self):
        args = self._proposal_args()
        turn_tune = TURN._load_turn_tune_module()
        parser = turn_tune.build_arg_parser()
        sim_args = parser.parse_args(
            [
                "simulate",
                "--runner",
                "shortest",
                "--mode",
                "2",
                "--code",
                "501",
                "--json",
            ]
        )
        current = turn_tune.simulate_turn(
            turn_tune.resolve_turn(sim_args),
            turn_tune.load_constants(sim_args),
            sim_args.entry_speed,
            sim_args.out_speed,
        ).final_pose
        endpoint = {
            "x_right_mm": {"median": current.x_mm, "std": 0.5},
            "y_forward_mm": {"median": current.y_mm, "std": 0.5},
            "theta_deg": {"median": current.theta_deg, "std": 0.5},
        }
        proposal = TURN.propose_fit(
            args,
            {"count": 3, "endpoint": endpoint},
        )
        for name in ("angle", "dist_in", "dist_out"):
            self.assertAlmostEqual(
                proposal["parameter_deltas"][name],
                0.0,
                delta=1e-9,
            )


if __name__ == "__main__":
    unittest.main()
