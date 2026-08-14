#!/usr/bin/env python3
"""Measure residual fixed-camera distortion from a known orthogonal maze grid."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

import aruco_trajectory as aruco
import board_layout


SCHEMA = "nightfall_board_line_grid_probe_v1"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Rectify a fixed-camera clip with the four outer ArUco markers, "
            "measure the known 90 mm orthogonal grid, and separate global "
            "layout scale from residual field-dependent distortion."
        )
    )
    parser.add_argument("video", type=Path)
    parser.add_argument("--board-layout", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--canonical-size", type=int, default=900)
    parser.add_argument("--samples", type=int, default=15)
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> None:
    if not args.video.is_file():
        raise ValueError(f"video does not exist: {args.video}")
    if not args.board_layout.is_file():
        raise ValueError(f"board layout does not exist: {args.board_layout}")
    if args.canonical_size < 400:
        raise ValueError("--canonical-size must be at least 400")
    if args.samples < 3 or args.samples > 31 or args.samples % 2 == 0:
        raise ValueError("--samples must be an odd integer in 3..31")


def _validate_marker_grid_coincidence(layout: board_layout.BoardLayout) -> None:
    raw = layout.raw
    origin_x, origin_y = map(float, raw["grid"]["origin_mm"])
    extent = float(raw["grid"]["pitch_mm"]) * int(raw["grid"]["cells"])
    expected = {
        6: (origin_x, origin_y),
        4: (origin_x + extent, origin_y),
        5: (origin_x, origin_y + extent),
        7: (origin_x + extent, origin_y + extent),
    }
    for marker_id, center in expected.items():
        item = raw["markers"].get(str(marker_id))
        if not isinstance(item, dict):
            raise ValueError(f"board layout must contain marker {marker_id}")
        actual = tuple(map(float, item["center_mm"]))
        if not np.allclose(actual, center, atol=1e-6, rtol=0.0):
            raise ValueError(
                f"marker {marker_id} center {actual} must coincide with grid "
                f"intersection {center}"
            )


def _sample_rectified_frames(
    video: Path,
    calibrations: list[aruco.FrameCalibration],
    size: int,
    sample_count: int,
) -> tuple[np.ndarray, list[int]]:
    indexes = sorted(
        set(
            map(
                int,
                np.rint(
                    np.linspace(
                        0.2 * (len(calibrations) - 1),
                        0.8 * (len(calibrations) - 1),
                        sample_count,
                    )
                ),
            )
        )
    )
    wanted = set(indexes)
    frames: list[np.ndarray] = []
    capture = cv2.VideoCapture(str(video))
    frame_index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame_index in wanted:
            frames.append(
                cv2.warpPerspective(
                    frame,
                    calibrations[frame_index].homography,
                    (size, size),
                    flags=cv2.INTER_LINEAR,
                )
            )
        frame_index += 1
    capture.release()
    if len(frames) != len(indexes):
        raise RuntimeError("failed to decode all line-grid sample frames")
    return np.median(np.stack(frames), axis=0).astype(np.uint8), indexes


def _subpixel_peak(profile: np.ndarray, expected: float, radius: float) -> float:
    lo = max(0, int(math.floor(expected - radius)))
    hi = min(len(profile), int(math.ceil(expected + radius)) + 1)
    if hi - lo < 3:
        raise ValueError("line search window is too small")
    peak = lo + int(np.argmax(profile[lo:hi]))
    result = float(peak)
    if 0 < peak < len(profile) - 1:
        before, at, after = map(float, profile[peak - 1 : peak + 2])
        denominator = before - 2.0 * at + after
        if abs(denominator) > 1e-9:
            offset = 0.5 * (before - after) / denominator
            if abs(offset) <= 1.0:
                result += offset
    return result


def _positive_highpass(reference_bgr: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(reference_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    background = cv2.GaussianBlur(gray, (0, 0), sigmaX=5.0, sigmaY=5.0)
    return np.maximum(gray - background, 0.0)


def _detect_axis_positions(
    response: np.ndarray,
    expected_positions: np.ndarray,
    *,
    axis: str,
) -> tuple[np.ndarray, list[float]]:
    if axis not in ("x", "y"):
        raise ValueError("axis must be x or y")
    expected = np.asarray(expected_positions, dtype=float)
    if expected.ndim != 1 or len(expected) < 3:
        raise ValueError("expected_positions must contain at least three lines")
    pitch = float(np.median(np.diff(expected)))
    inner_lo = int(max(0.0, math.floor(expected[0] + 0.34 * pitch)))
    inner_hi = int(min(response.shape[0 if axis == "x" else 1], math.ceil(expected[-1] - 0.34 * pitch)))
    if axis == "x":
        profile = np.mean(response[inner_lo:inner_hi, :], axis=0)
    else:
        profile = np.mean(response[:, inner_lo:inner_hi], axis=1)
    detected = np.asarray(
        [_subpixel_peak(profile, position, 0.18 * pitch) for position in expected],
        dtype=float,
    )

    # The four ArUco centres define the two outer grid lines.  Their printed
    # pattern makes a brightness peak at the corner ambiguous, so use the
    # exact registration targets there rather than re-detecting the chalk line.
    detected[0] = expected[0]
    detected[-1] = expected[-1]

    local_pitch_mm_px: list[float] = []
    other_expected = expected
    for first, second in zip(other_expected[:-1], other_expected[1:]):
        centre = 0.5 * (first + second)
        band_lo = max(0, int(math.floor(centre - 0.16 * pitch)))
        band_hi = min(
            response.shape[0 if axis == "x" else 1],
            int(math.ceil(centre + 0.16 * pitch)) + 1,
        )
        if axis == "x":
            local_profile = np.mean(response[band_lo:band_hi, :], axis=0)
        else:
            local_profile = np.mean(response[:, band_lo:band_hi], axis=1)
        local = np.asarray(
            [
                _subpixel_peak(local_profile, position, 0.18 * pitch)
                for position in expected[1:-1]
            ],
            dtype=float,
        )
        local_slope, _ = np.polyfit(np.arange(1, len(expected) - 1), local, 1)
        local_pitch_mm_px.append(float(local_slope))
    return detected, local_pitch_mm_px


def _axis_report(
    positions_px: np.ndarray,
    expected_px: np.ndarray,
    local_pitches_px: list[float],
    pixels_per_mm: float,
    known_pitch_mm: float,
) -> dict[str, object]:
    indexes = np.arange(len(positions_px), dtype=float)
    slope_px, intercept_px = np.polyfit(indexes, positions_px, 1)
    residual_px = positions_px - (slope_px * indexes + intercept_px)
    layout_residual_px = positions_px - expected_px
    pitch_mm = float(slope_px / pixels_per_mm)
    spacings_mm = np.diff(positions_px) / pixels_per_mm
    local_pitches_mm = np.asarray(local_pitches_px, dtype=float) / pixels_per_mm
    return {
        "positions_px": positions_px.tolist(),
        "intervals_mm": spacings_mm.tolist(),
        "affine_pitch_mm": pitch_mm,
        "affine_scale_error_percent": 100.0 * (pitch_mm / known_pitch_mm - 1.0),
        "uniform_grid_residual_mm_max": float(
            np.max(np.abs(residual_px)) / pixels_per_mm
        ),
        "layout_residual_mm_max": float(
            np.max(np.abs(layout_residual_px)) / pixels_per_mm
        ),
        "local_pitch_mm_min": float(np.min(local_pitches_mm)),
        "local_pitch_mm_max": float(np.max(local_pitches_mm)),
        "local_pitch_spread_mm": float(np.ptp(local_pitches_mm)),
    }


def _draw_overlay(
    reference: np.ndarray,
    x_positions: np.ndarray,
    y_positions: np.ndarray,
) -> np.ndarray:
    overlay = reference.copy()
    for index, value in enumerate(x_positions):
        x = int(round(value))
        cv2.line(overlay, (x, 0), (x, overlay.shape[0] - 1), (0, 255, 0), 1)
        cv2.putText(
            overlay,
            f"x{index}",
            (x + 3, overlay.shape[0] // 2 - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
    for index, value in enumerate(y_positions):
        y = int(round(value))
        cv2.line(overlay, (0, y), (overlay.shape[1] - 1, y), (0, 255, 255), 1)
        cv2.putText(
            overlay,
            f"y{index}",
            (overlay.shape[1] // 2 + 4, y - 3),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return overlay


def main() -> int:
    args = parse_args()
    try:
        _validate_args(args)
        layout = board_layout.load(args.board_layout, args.canonical_size)
        _validate_marker_grid_coincidence(layout)
        info, observations, counts = aruco.collect_fixed_observations(args.video)
        calibrations, _ = aruco.build_calibrations(
            observations,
            args.canonical_size,
            50.0,
            layout.target_corners_px,
        )
        reference, sampled_frames = _sample_rectified_frames(
            args.video,
            calibrations,
            args.canonical_size,
            args.samples,
        )
        response = _positive_highpass(reference)
        x_positions, x_local = _detect_axis_positions(
            response, layout.grid.x_lines_px, axis="x"
        )
        y_positions, y_local = _detect_axis_positions(
            response, layout.grid.y_lines_px, axis="y"
        )
        pitch_mm = layout.grid_pitch_mm
        report = {
            "schema": SCHEMA,
            "source_video": {
                "path": str(args.video.resolve()),
                "sha256": aruco.sha256_file(args.video),
                "width": info.width,
                "height": info.height,
                "fps": info.fps,
                "frames": info.frame_count,
            },
            "board_layout": {
                "path": str(args.board_layout.resolve()),
                "sha256": aruco.sha256_file(args.board_layout),
                "marker_center_span_mm": pitch_mm * layout.grid.cells,
            },
            "sampled_frames": sampled_frames,
            "marker_detection_count": {str(key): value for key, value in counts.items()},
            "known_line_spacing_mm": pitch_mm,
            "x": _axis_report(
                x_positions,
                layout.grid.x_lines_px,
                x_local,
                layout.pixels_per_mm,
                pitch_mm,
            ),
            "y": _axis_report(
                y_positions,
                layout.grid.y_lines_px,
                y_local,
                layout.pixels_per_mm,
                pitch_mm,
            ),
            "interpretation": {
                "uniform_grid_residual_includes_mm": [
                    "lens residual after four-marker homography",
                    "board non-flatness",
                    "drawn-line placement and detector uncertainty",
                ],
                "safety_qualified_metric_map": False,
            },
        }
        args.output_dir.mkdir(parents=True, exist_ok=True)
        report_path = args.output_dir / "line_grid_report.json"
        report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        cv2.imwrite(str(args.output_dir / "rectified_median.png"), reference)
        cv2.imwrite(
            str(args.output_dir / "line_detection_overlay.png"),
            _draw_overlay(reference, x_positions, y_positions),
        )
        print(
            "[LINE-GRID] X pitch={:.3f}mm scale={:+.3f}% residual={:.3f}mm; "
            "Y pitch={:.3f}mm scale={:+.3f}% residual={:.3f}mm".format(
                report["x"]["affine_pitch_mm"],
                report["x"]["affine_scale_error_percent"],
                report["x"]["uniform_grid_residual_mm_max"],
                report["y"]["affine_pitch_mm"],
                report["y"]["affine_scale_error_percent"],
                report["y"]["uniform_grid_residual_mm_max"],
            )
        )
        print(f"[LINE-GRID] report={report_path.resolve()}")
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"[LINE-GRID] ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
