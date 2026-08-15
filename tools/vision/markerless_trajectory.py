#!/usr/bin/env python3
"""Extract a planar micromouse pose without an AR marker on the vehicle.

Fixed ArUco markers remain around the maze to stabilize the maze plane.  The
vehicle center is preferably tracked from its 8 mm blue label.  Background
difference and the green PCB still provide its body silhouette.  A second red
label at a known distance in front of the blue label provides a directed
heading.  A separate coloured LED or the foreground principal axis remains as
a fallback.

The implementation intentionally reuses the proven fixed-marker and grid code
from ``aruco_trajectory.py`` so the original carried-marker workflow remains
unchanged.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import subprocess
import sys
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import cv2
import numpy as np

import aruco_trajectory as aruco
import board_layout
import board_metric_geometry


BLUE_LABEL_HSV_LOW = (88, 50, 50)
BLUE_LABEL_HSV_HIGH = (112, 255, 255)
BLUE_LABEL_MIN_BLUE_GREEN_EXCESS = 8
BLUE_LABEL_MIN_BLUE_RED_EXCESS = 25
BLUE_LABEL_GLARE_HSV_LOW = (82, 5, 150)
BLUE_LABEL_GLARE_HSV_HIGH = (112, 255, 255)
BLUE_LABEL_GLARE_MIN_BLUE_GREEN_EXCESS = -2
BLUE_LABEL_GLARE_MIN_BLUE_RED_EXCESS = 5
FRONT_LABEL_HSV_LOW = (0, 40, 70)
FRONT_LABEL_HSV_HIGH = (26, 255, 255)
FRONT_LABEL_HSV_WRAP_LOW = (168, 40, 70)
FRONT_LABEL_HSV_WRAP_HIGH = (179, 255, 255)
DEFAULT_FRONT_LABEL_DISTANCE_MM = 24.0
FRONT_LABEL_CALIBRATION_SCHEMA = "nightfall_front_label_heading_calibration_v1"


@dataclass
class Detection:
    frame: int
    time_s: float
    position_xy: np.ndarray
    body_xy: np.ndarray
    cue_xy: np.ndarray
    label_xy: np.ndarray
    front_label_xy: np.ndarray
    yaw_unwrapped_deg: float
    body_pixel_count: int
    green_pixel_count: int
    cue_pixel_count: int
    label_pixel_count: int
    front_label_pixel_count: int
    cue_brightness: float
    axis_anisotropy: float
    pose_confidence: float
    pose_valid: bool
    heading_valid: bool
    heading_source: str
    position_source: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Stabilize a marked maze and track a micromouse from its "
            "foreground/PCB/LED, without a marker on the vehicle."
        )
    )
    parser.add_argument("video", type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="default: <video stem>_markerless beside the input",
    )
    parser.add_argument("--canonical-size", type=int, default=900)
    parser.add_argument("--marker-margin", type=float, default=50.0)
    parser.add_argument(
        "--board-layout",
        type=Path,
        default=None,
        help=(
            "measured marker/grid layout JSON; when supplied it defines "
            "the metric transform and grid"
        ),
    )
    parser.add_argument(
        "--board-metric-geometry",
        type=Path,
        default=None,
        help=(
            "qualified dense canonical-pixel to board-mm correction; "
            "requires --board-layout and replaces its linear metric scale"
        ),
    )
    parser.add_argument(
        "--grid-cells",
        type=int,
        default=None,
        help="visible grid-pitch count; default 8 without --board-layout",
    )
    parser.add_argument("--cell-size-mm", type=float, default=None)
    parser.add_argument(
        "--cell-size-confirmed",
        action="store_true",
        help="record that --cell-size-mm was measured rather than assumed",
    )
    parser.add_argument(
        "--background-samples",
        type=int,
        default=41,
        help="number of evenly spaced rectified frames in the median background",
    )
    parser.add_argument(
        "--background-video",
        type=Path,
        default=None,
        help=(
            "separate empty-maze clip captured without moving the camera; "
            "preferred over estimating the background from the run"
        ),
    )
    parser.add_argument("--foreground-threshold", type=int, default=48)
    parser.add_argument("--foreground-blur", type=int, default=5)
    parser.add_argument("--morph-open", type=int, default=3)
    parser.add_argument("--morph-close", type=int, default=11)
    parser.add_argument(
        "--tracking-radius-px",
        type=float,
        default=82.0,
        help="canonical-pixel radius around the PCB/prior pose",
    )
    parser.add_argument("--minimum-body-pixels", type=int, default=250)
    parser.add_argument("--minimum-green-pixels", type=int, default=250)
    parser.add_argument("--minimum-cue-pixels", type=int, default=3)
    parser.add_argument("--maximum-cue-pixels", type=int, default=500)
    parser.add_argument("--minimum-label-pixels", type=int, default=20)
    parser.add_argument("--maximum-label-pixels", type=int, default=180)
    parser.add_argument(
        "--label-diameter-mm",
        type=float,
        default=8.0,
        help="physical diameter of the circular vehicle-center label",
    )
    parser.add_argument(
        "--front-label-colour",
        choices=("red", "none"),
        default="red",
        help="colour of the directed heading label in front of the centre",
    )
    parser.add_argument(
        "--front-label-diameter-mm",
        type=float,
        default=8.0,
    )
    parser.add_argument(
        "--front-label-distance-mm",
        type=float,
        default=DEFAULT_FRONT_LABEL_DISTANCE_MM,
        help="measured centre-to-centre distance from blue to front label",
    )
    parser.add_argument(
        "--front-label-distance-tolerance-mm",
        type=float,
        default=8.0,
        help=(
            "allowed apparent baseline error after floor-plane rectification; "
            "includes label-height parallax"
        ),
    )
    parser.add_argument("--minimum-front-label-pixels", type=int, default=20)
    parser.add_argument("--maximum-front-label-pixels", type=int, default=180)
    parser.add_argument(
        "--front-label-yaw-offset-deg",
        type=float,
        default=0.0,
        help="vehicle-forward yaw minus blue-centre-to-red-label direction",
    )
    parser.add_argument(
        "--front-label-bias-right-mm",
        type=float,
        default=0.0,
        help=(
            "apparent blue-to-red vector bias toward maze +x to subtract; "
            "calibrates differential label-height parallax"
        ),
    )
    parser.add_argument(
        "--front-label-bias-forward-mm",
        type=float,
        default=0.0,
        help=(
            "apparent blue-to-red vector bias toward maze +y to subtract; "
            "calibrates differential label-height parallax"
        ),
    )
    parser.add_argument(
        "--front-label-calibration",
        type=Path,
        default=None,
        help=(
            "heading calibration JSON produced by "
            "fit_front_label_heading.py; supplies parallax-bias correction"
        ),
    )
    parser.add_argument(
        "--minimum-cue-lever-arm-px",
        type=float,
        default=10.0,
        help="minimum body-centroid to colour-cue distance",
    )
    parser.add_argument(
        "--cue-distance-relative-tolerance",
        type=float,
        default=0.90,
        help="allowed relative deviation from the recent cue lever arm",
    )
    parser.add_argument(
        "--minimum-axis-anisotropy",
        type=float,
        default=0.08,
        help="minimum principal-axis anisotropy for a cue-less heading",
    )
    parser.add_argument(
        "--maximum-yaw-rate-deg-s",
        type=float,
        default=3000.0,
        help="reject heading innovations faster than this physical limit",
    )
    parser.add_argument(
        "--position-source",
        choices=("label", "cue", "body", "green"),
        default="label",
        help=(
            "track the blue center label, LED centroid, foreground centroid, "
            "or green-PCB centroid"
        ),
    )
    parser.add_argument(
        "--label-colour",
        choices=("blue", "none"),
        default="blue",
    )
    parser.add_argument(
        "--cue-colour",
        choices=("red", "none"),
        default="red",
    )
    parser.add_argument(
        "--cue-yaw-offset-deg",
        type=float,
        default=180.0,
        help=(
            "vehicle-forward yaw minus the body-centroid-to-cue direction; "
            "default 180 deg assumes the selected LED is at the rear"
        ),
    )
    parser.add_argument(
        "--axis-yaw-offset-deg",
        type=float,
        default=0.0,
        help="vehicle-forward yaw minus the foreground principal-axis yaw",
    )
    parser.add_argument(
        "--initial-yaw-deg",
        type=float,
        default=None,
        help=(
            "known starting yaw in maze coordinates; strongly recommended "
            "when no directed LED cue is available"
        ),
    )
    parser.add_argument(
        "--initial-x-cell",
        type=float,
        default=None,
        help="known initial x cell used to gate the first PCB component",
    )
    parser.add_argument(
        "--initial-y-cell",
        type=float,
        default=None,
        help="known initial y cell used to gate the first PCB component",
    )
    parser.add_argument("--smooth-window", type=int, default=9)
    parser.add_argument(
        "--maximum-missing-fraction",
        type=float,
        default=0.01,
    )
    parser.add_argument(
        "--maximum-homography-fallback-fraction",
        type=float,
        default=0.0,
        help="maximum fraction of run frames reusing a previous homography",
    )
    parser.add_argument(
        "--maximum-consecutive-homography-fallback-frames",
        type=int,
        default=5,
        help=(
            "maximum consecutive run frames reusing the previous homography; "
            "increase only for a fixed camera with temporary marker occlusion"
        ),
    )
    parser.add_argument(
        "--maximum-heading-invalid-fraction",
        type=float,
        default=0.01,
    )
    parser.add_argument(
        "--position-only",
        action="store_true",
        help=(
            "require and export position QA while reporting, but not gating, "
            "foreground-derived heading"
        ),
    )
    parser.add_argument(
        "--no-video",
        action="store_true",
        help="skip the annotated top-view video",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    float_arguments = (
        "marker_margin",
        "cell_size_mm",
        "tracking_radius_px",
        "label_diameter_mm",
        "front_label_diameter_mm",
        "front_label_distance_mm",
        "front_label_distance_tolerance_mm",
        "front_label_yaw_offset_deg",
        "front_label_bias_right_mm",
        "front_label_bias_forward_mm",
        "minimum_cue_lever_arm_px",
        "cue_distance_relative_tolerance",
        "minimum_axis_anisotropy",
        "maximum_yaw_rate_deg_s",
        "cue_yaw_offset_deg",
        "axis_yaw_offset_deg",
        "initial_yaw_deg",
        "initial_x_cell",
        "initial_y_cell",
        "maximum_missing_fraction",
        "maximum_homography_fallback_fraction",
        "maximum_heading_invalid_fraction",
    )
    for name in float_arguments:
        value = getattr(args, name)
        if value is not None and not math.isfinite(value):
            raise ValueError(f"--{name.replace('_', '-')} must be finite")
    if not args.video.is_file():
        raise ValueError(f"video does not exist: {args.video}")
    for name in (
        "board_layout",
        "board_metric_geometry",
        "background_video",
        "front_label_calibration",
    ):
        path = getattr(args, name)
        if path is not None and not path.is_file():
            raise ValueError(f"--{name.replace('_', '-')} does not exist: {path}")
    if (
        args.background_video is not None
        and args.background_video.resolve() == args.video.resolve()
    ):
        raise ValueError("--background-video must be a separate empty-maze clip")
    if args.canonical_size < 400:
        raise ValueError("--canonical-size must be at least 400")
    if not 0 < args.marker_margin < args.canonical_size / 4:
        raise ValueError("--marker-margin is outside the usable range")
    if args.grid_cells is not None and args.grid_cells < 2:
        raise ValueError("--grid-cells must be at least 2")
    if args.cell_size_mm is not None and args.cell_size_mm <= 0:
        raise ValueError("--cell-size-mm must be positive")
    if (
        args.cell_size_confirmed
        and args.cell_size_mm is None
        and args.board_layout is None
    ):
        raise ValueError("--cell-size-confirmed requires --cell-size-mm")
    if args.board_metric_geometry is not None and args.board_layout is None:
        raise ValueError("--board-metric-geometry requires --board-layout")
    if args.background_samples < 5:
        raise ValueError("--background-samples must be at least 5")
    for name in ("foreground_blur", "morph_open", "morph_close"):
        value = getattr(args, name)
        if value < 1 or value % 2 == 0:
            raise ValueError(
                f"--{name.replace('_', '-')} must be a positive odd integer"
            )
    if args.foreground_threshold < 1 or args.foreground_threshold > 255:
        raise ValueError("--foreground-threshold must be in 1..255")
    if args.tracking_radius_px <= 0:
        raise ValueError("--tracking-radius-px must be positive")
    if args.minimum_body_pixels <= 0:
        raise ValueError("--minimum-body-pixels must be positive")
    if args.minimum_green_pixels <= 0:
        raise ValueError("--minimum-green-pixels must be positive")
    if args.minimum_cue_pixels <= 0:
        raise ValueError("--minimum-cue-pixels must be positive")
    if args.maximum_cue_pixels < args.minimum_cue_pixels:
        raise ValueError("--maximum-cue-pixels must be >= --minimum-cue-pixels")
    if args.minimum_label_pixels <= 0:
        raise ValueError("--minimum-label-pixels must be positive")
    if args.maximum_label_pixels < args.minimum_label_pixels:
        raise ValueError(
            "--maximum-label-pixels must be >= --minimum-label-pixels"
        )
    if args.label_diameter_mm <= 0:
        raise ValueError("--label-diameter-mm must be positive")
    if args.position_source == "label" and args.label_colour == "none":
        raise ValueError("--position-source label requires --label-colour blue")
    if args.front_label_diameter_mm <= 0:
        raise ValueError("--front-label-diameter-mm must be positive")
    if args.front_label_distance_mm <= 0:
        raise ValueError("--front-label-distance-mm must be positive")
    if args.front_label_distance_tolerance_mm <= 0:
        raise ValueError(
            "--front-label-distance-tolerance-mm must be positive"
        )
    if args.minimum_front_label_pixels <= 0:
        raise ValueError("--minimum-front-label-pixels must be positive")
    if args.maximum_front_label_pixels < args.minimum_front_label_pixels:
        raise ValueError(
            "--maximum-front-label-pixels must be >= "
            "--minimum-front-label-pixels"
        )
    if args.front_label_calibration is not None and (
        args.front_label_bias_right_mm != 0.0
        or args.front_label_bias_forward_mm != 0.0
    ):
        raise ValueError(
            "--front-label-calibration cannot be combined with direct "
            "front-label bias values"
        )
    if args.minimum_cue_lever_arm_px <= 0:
        raise ValueError("--minimum-cue-lever-arm-px must be positive")
    if not 0 < args.cue_distance_relative_tolerance < 2:
        raise ValueError("--cue-distance-relative-tolerance must be in (0, 2)")
    if not 0 <= args.minimum_axis_anisotropy <= 1:
        raise ValueError("--minimum-axis-anisotropy must be in [0, 1]")
    if args.maximum_yaw_rate_deg_s <= 0:
        raise ValueError("--maximum-yaw-rate-deg-s must be positive")
    if args.smooth_window < 5 or args.smooth_window % 2 == 0:
        raise ValueError("--smooth-window must be an odd integer >= 5")
    if (args.initial_x_cell is None) != (args.initial_y_cell is None):
        raise ValueError(
            "--initial-x-cell and --initial-y-cell must be supplied together"
        )
    if not 0 <= args.maximum_missing_fraction < 1:
        raise ValueError("--maximum-missing-fraction must be in [0, 1)")
    if not 0 <= args.maximum_homography_fallback_fraction < 1:
        raise ValueError("--maximum-homography-fallback-fraction must be in [0, 1)")
    if args.maximum_consecutive_homography_fallback_frames < 0:
        raise ValueError(
            "--maximum-consecutive-homography-fallback-frames must be non-negative"
        )
    if not 0 <= args.maximum_heading_invalid_fraction < 1:
        raise ValueError("--maximum-heading-invalid-fraction must be in [0, 1)")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_front_label_calibration(
    path: Path,
    expected_distance_mm: float,
) -> dict[str, object]:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid front-label calibration JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise ValueError("front-label calibration root must be an object")
    if raw.get("schema") != FRONT_LABEL_CALIBRATION_SCHEMA:
        raise ValueError(
            "front-label calibration schema must be "
            f"{FRONT_LABEL_CALIBRATION_SCHEMA}"
        )
    if raw.get("coordinate_system") != "x_right_y_forward_mm":
        raise ValueError(
            "front-label calibration coordinate_system must be "
            "x_right_y_forward_mm"
        )
    try:
        calibrated_distance = float(raw["front_label_distance_mm"])
        bias = raw["apparent_vector_bias_mm"]
        bias_right = float(bias["right"])
        bias_forward = float(bias["forward"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("front-label calibration fields are invalid") from exc
    if not all(
        math.isfinite(value)
        for value in (calibrated_distance, bias_right, bias_forward)
    ):
        raise ValueError("front-label calibration values must be finite")
    if not math.isclose(
        calibrated_distance,
        expected_distance_mm,
        rel_tol=0.0,
        abs_tol=0.1,
    ):
        raise ValueError(
            "front-label calibration distance does not match "
            "--front-label-distance-mm"
        )
    return {
        "raw": raw,
        "bias_right_mm": bias_right,
        "bias_forward_mm": bias_forward,
    }


def video_timestamps(
    video_path: Path,
    frame_count: int,
    fps: Optional[float] = None,
) -> tuple[np.ndarray, str]:
    """Read strictly increasing presentation timestamps with ffprobe."""
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        raise RuntimeError(
            "ffprobe is required for measurement timestamps; "
            "run tools/vision/setup_host.sh after installing FFmpeg"
        )
    command = [
        ffprobe,
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "frame=best_effort_timestamp_time",
        "-of",
        "csv=p=0",
        str(video_path),
    ]
    completed = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr.strip() or "ffprobe timestamp read failed")
    values: list[float] = []
    for line in completed.stdout.splitlines():
        token = line.strip().split(",", 1)[0]
        try:
            value = float(token)
        except ValueError:
            continue
        if math.isfinite(value):
            values.append(value)
    if len(values) != frame_count:
        raise RuntimeError(
            "ffprobe returned {} timestamps for {} decoded frames".format(
                len(values),
                frame_count,
            )
        )
    result = np.asarray(values, dtype=float)
    result -= result[0]
    if not np.all(np.diff(result) > 0):
        raise RuntimeError("video PTS are not strictly increasing")
    return result, "ffprobe_best_effort_timestamp"


def _sample_indexes(frame_count: int, count: int) -> list[int]:
    return sorted(
        {
            int(round(value))
            for value in np.linspace(0, frame_count - 1, min(count, frame_count))
        }
    )


def build_background(
    video_path: Path,
    calibrations: Sequence[aruco.FrameCalibration],
    info: aruco.VideoInfo,
    canonical_size: int,
    sample_count: int,
) -> np.ndarray:
    wanted = set(_sample_indexes(info.frame_count, sample_count))
    capture, _ = aruco.open_video(video_path)
    samples: list[np.ndarray] = []
    frame_index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame_index in wanted:
            samples.append(
                cv2.warpPerspective(
                    frame,
                    calibrations[frame_index].homography,
                    (canonical_size, canonical_size),
                )
            )
        frame_index += 1
    capture.release()
    if len(samples) < 5:
        raise RuntimeError("could not collect enough background samples")
    return np.median(np.stack(samples, axis=0), axis=0).astype(np.uint8)


def green_mask(frame: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.asarray((25, 55, 28), dtype=np.uint8),
        np.asarray((105, 255, 255), dtype=np.uint8),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (19, 19)),
    )


def red_mask(frame: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low = cv2.inRange(
        hsv,
        np.asarray((0, 100, 80), dtype=np.uint8),
        np.asarray((14, 255, 255), dtype=np.uint8),
    )
    high = cv2.inRange(
        hsv,
        np.asarray((168, 100, 80), dtype=np.uint8),
        np.asarray((179, 255, 255), dtype=np.uint8),
    )
    mask = cv2.bitwise_or(low, high)
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )


def front_label_mask(frame: np.ndarray) -> np.ndarray:
    """Select the red front label even when HFR lighting shifts it orange.

    Pixel 8 recordings made under the current maze lighting move the matte red
    pigment as far as HSV hue 22 during a turn.  The legacy red cue mask is
    deliberately narrower and remains unchanged.  False orange objects are
    rejected later using the label's area, 24 mm lever arm, and predicted
    position.
    """

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low = cv2.inRange(
        hsv,
        np.asarray(FRONT_LABEL_HSV_LOW, dtype=np.uint8),
        np.asarray(FRONT_LABEL_HSV_HIGH, dtype=np.uint8),
    )
    high = cv2.inRange(
        hsv,
        np.asarray(FRONT_LABEL_HSV_WRAP_LOW, dtype=np.uint8),
        np.asarray(FRONT_LABEL_HSV_WRAP_HIGH, dtype=np.uint8),
    )
    mask = cv2.bitwise_or(low, high)
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )


def blue_label_mask(frame: np.ndarray) -> np.ndarray:
    """Select the matte blue center label without accepting the green PCB.

    The deliberately narrow hue range was measured from the Pixel 8 1 ms /
    ISO 800 HFR path.  Blue-channel excess gates keep cyan PCB reflections out
    while the upper hue bound separates the label pigment from the three blue
    optical-token LEDs.  Do not apply an opening operation here: after metric
    rectification the 8 mm label is only about 8 by 8 pixels.
    """

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.asarray(BLUE_LABEL_HSV_LOW, dtype=np.uint8),
        np.asarray(BLUE_LABEL_HSV_HIGH, dtype=np.uint8),
    )
    blue, green, red = cv2.split(frame.astype(np.int16))
    channel_gate = np.where(
        (blue - green >= BLUE_LABEL_MIN_BLUE_GREEN_EXCESS)
        & (blue - red >= BLUE_LABEL_MIN_BLUE_RED_EXCESS),
        255,
        0,
    ).astype(np.uint8)
    return cv2.bitwise_and(mask, channel_gate)


def blue_label_glare_mask(frame: np.ndarray) -> np.ndarray:
    """Recover a specularly desaturated blue label near the tracked mouse.

    The overhead light can drive the matte label close to white for particular
    mouse positions and headings.  This deliberately permissive mask must only
    be used with a spatial prediction from the normal label tracker or green
    PCB tracker; applying it board-wide would admit blue-tinted grid lines.
    """

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.asarray(BLUE_LABEL_GLARE_HSV_LOW, dtype=np.uint8),
        np.asarray(BLUE_LABEL_GLARE_HSV_HIGH, dtype=np.uint8),
    )
    blue, green, red = cv2.split(frame.astype(np.int16))
    channel_gate = np.where(
        (blue - green >= BLUE_LABEL_GLARE_MIN_BLUE_GREEN_EXCESS)
        & (blue - red >= BLUE_LABEL_GLARE_MIN_BLUE_RED_EXCESS),
        255,
        0,
    ).astype(np.uint8)
    return cv2.bitwise_and(mask, channel_gate)


def centroid(mask: np.ndarray) -> Optional[np.ndarray]:
    moments = cv2.moments(mask, binaryImage=True)
    if moments["m00"] <= 0:
        return None
    return np.asarray(
        [
            moments["m10"] / moments["m00"],
            moments["m01"] / moments["m00"],
        ],
        dtype=float,
    )


def _circle_mask(
    shape: tuple[int, int],
    center_xy: np.ndarray,
    radius: float,
) -> np.ndarray:
    result = np.zeros(shape, dtype=np.uint8)
    cv2.circle(
        result,
        tuple(np.rint(center_xy).astype(int)),
        int(round(radius)),
        255,
        -1,
    )
    return result


def _board_mask(
    shape: tuple[int, int],
    grid: aruco.GridCalibration,
) -> np.ndarray:
    result = np.zeros(shape, dtype=np.uint8)
    margin_x = 0.55 * grid.x_pitch_px
    margin_y = 0.55 * grid.y_pitch_px
    left = max(
        0,
        int(round(grid.x_origin_px - margin_x)),
    )
    right = min(
        shape[1] - 1,
        int(round(grid.x_origin_px + grid.cells * grid.x_pitch_px + margin_x)),
    )
    top = max(
        0,
        int(round(grid.y_origin_px - margin_y)),
    )
    bottom = min(
        shape[0] - 1,
        int(round(grid.y_origin_px + grid.cells * grid.y_pitch_px + margin_y)),
    )
    cv2.rectangle(result, (left, top), (right, bottom), 255, -1)
    return result


def _component_near(
    mask: np.ndarray,
    minimum_pixels: int,
    maximum_pixels: int,
    prediction_xy: Optional[np.ndarray],
    maximum_distance: Optional[float] = None,
    prefer_largest: bool = False,
) -> tuple[Optional[np.ndarray], int, Optional[np.ndarray]]:
    count, labels, stats, centers = cv2.connectedComponentsWithStats(
        mask,
        connectivity=8,
    )
    candidates: list[tuple[float, int]] = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if not minimum_pixels <= area <= maximum_pixels:
            continue
        distance = None
        if prediction_xy is not None:
            distance = float(np.linalg.norm(centers[label] - prediction_xy))
            if maximum_distance is not None and distance > maximum_distance:
                continue
        if prediction_xy is None or prefer_largest:
            score = -float(area)
        else:
            assert distance is not None
            score = distance - 0.002 * area
        candidates.append((score, label))
    if not candidates:
        return None, 0, None
    _, selected = min(candidates)
    component = np.where(labels == selected, 255, 0).astype(np.uint8)
    return (
        np.asarray(centers[selected], dtype=float),
        int(stats[selected, cv2.CC_STAT_AREA]),
        component,
    )


def _label_component(
    mask: np.ndarray,
    minimum_pixels: int,
    maximum_pixels: int,
    expected_pixels: float,
    prediction_xy: Optional[np.ndarray],
    maximum_distance: float,
) -> tuple[Optional[np.ndarray], int, Optional[np.ndarray]]:
    """Choose the label over similarly coloured optical-token LED blobs."""

    count, labels, stats, centers = cv2.connectedComponentsWithStats(
        mask,
        connectivity=8,
    )
    candidates: list[tuple[float, int]] = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if not minimum_pixels <= area <= maximum_pixels:
            continue
        distance = 0.0
        if prediction_xy is not None:
            distance = float(np.linalg.norm(centers[label] - prediction_xy))
            if distance > maximum_distance:
                continue
        width = max(1, int(stats[label, cv2.CC_STAT_WIDTH]))
        height = max(1, int(stats[label, cv2.CC_STAT_HEIGHT]))
        area_error = abs(math.log(area / max(1.0, expected_pixels)))
        aspect_error = abs(math.log(width / height))
        score = 0.10 * distance + 2.0 * area_error + 0.20 * aspect_error
        candidates.append((score, label))
    if not candidates:
        return None, 0, None
    _, selected = min(candidates)
    component = np.where(labels == selected, 255, 0).astype(np.uint8)
    return (
        np.asarray(centers[selected], dtype=float),
        int(stats[selected, cv2.CC_STAT_AREA]),
        component,
    )


def _front_label_component(
    mask: np.ndarray,
    center_xy: np.ndarray,
    minimum_pixels: int,
    maximum_pixels: int,
    expected_pixels: float,
    expected_distance_px: float,
    distance_tolerance_px: float,
    prediction_xy: Optional[np.ndarray],
    maximum_prediction_distance_px: float,
) -> tuple[Optional[np.ndarray], int, Optional[np.ndarray]]:
    """Select a compact red label on the expected ring around the centre.

    Maze walls and the rear status LED are also red.  Component area/shape,
    the measured 24 mm centre-to-front baseline, and temporal prediction make
    the front label independently identifiable even when body segmentation is
    unavailable.
    """

    count, labels, stats, centers = cv2.connectedComponentsWithStats(
        mask,
        connectivity=8,
    )
    candidates: list[tuple[float, int]] = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if not minimum_pixels <= area <= maximum_pixels:
            continue
        baseline = float(np.linalg.norm(centers[label] - center_xy))
        baseline_error = abs(baseline - expected_distance_px)
        if baseline_error > distance_tolerance_px:
            continue
        prediction_error = 0.0
        if prediction_xy is not None:
            prediction_error = float(
                np.linalg.norm(centers[label] - prediction_xy)
            )
            if prediction_error > maximum_prediction_distance_px:
                continue
        width = max(1, int(stats[label, cv2.CC_STAT_WIDTH]))
        height = max(1, int(stats[label, cv2.CC_STAT_HEIGHT]))
        area_error = abs(math.log(area / max(1.0, expected_pixels)))
        aspect_error = abs(math.log(width / height))
        score = (
            0.45 * baseline_error
            + 2.0 * area_error
            + 0.35 * aspect_error
            + 0.08 * prediction_error
        )
        candidates.append((score, label))
    if not candidates:
        return None, 0, None
    _, selected = min(candidates)
    component = np.where(labels == selected, 255, 0).astype(np.uint8)
    return (
        np.asarray(centers[selected], dtype=float),
        int(stats[selected, cv2.CC_STAT_AREA]),
        component,
    )


def _blue_front_label_pair(
    blue_mask: np.ndarray,
    red_mask: np.ndarray,
    minimum_blue_pixels: int,
    maximum_blue_pixels: int,
    expected_blue_pixels: float,
    minimum_red_pixels: int,
    maximum_red_pixels: int,
    expected_red_pixels: float,
    expected_distance_px: float,
    distance_tolerance_px: float,
    prediction_xy: np.ndarray,
    maximum_prediction_distance_px: float,
) -> tuple[
    Optional[np.ndarray],
    int,
    Optional[np.ndarray],
    int,
]:
    """Jointly identify the centre and front labels around the mouse.

    A specular blue label may contribute only a small pale component while a
    blue optical-token LED remains strongly saturated.  The LED cannot form
    the measured centre-to-front-label baseline, so scoring the two labels as
    a pair prevents the tracker from jumping from the centre to an LED.
    """

    count, labels, stats, centers = cv2.connectedComponentsWithStats(
        blue_mask,
        connectivity=8,
    )
    candidates: list[
        tuple[float, int, np.ndarray, int, Optional[np.ndarray]]
    ] = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if not minimum_blue_pixels <= area <= maximum_blue_pixels:
            continue
        prediction_error = float(
            np.linalg.norm(centers[label] - prediction_xy)
        )
        if prediction_error > maximum_prediction_distance_px:
            continue
        front_xy, front_count, front_component = _front_label_component(
            red_mask,
            np.asarray(centers[label], dtype=float),
            minimum_red_pixels,
            maximum_red_pixels,
            expected_red_pixels,
            expected_distance_px,
            distance_tolerance_px,
            None,
            maximum_prediction_distance_px,
        )
        if front_xy is None:
            continue
        width = max(1, int(stats[label, cv2.CC_STAT_WIDTH]))
        height = max(1, int(stats[label, cv2.CC_STAT_HEIGHT]))
        baseline_error = abs(
            float(np.linalg.norm(front_xy - centers[label]))
            - expected_distance_px
        )
        score = (
            0.45 * baseline_error
            + 1.5 * abs(math.log(area / max(1.0, expected_blue_pixels)))
            + abs(math.log(front_count / max(1.0, expected_red_pixels)))
            + 0.20 * abs(math.log(width / height))
            + 0.08 * prediction_error
        )
        candidates.append(
            (score, label, front_xy, front_count, front_component)
        )
    if not candidates:
        return None, 0, None, 0
    _, selected, front_xy, front_count, _ = min(candidates, key=lambda x: x[0])
    return (
        np.asarray(centers[selected], dtype=float),
        int(stats[selected, cv2.CC_STAT_AREA]),
        front_xy,
        front_count,
    )


def _foreground_cluster(
    mask: np.ndarray,
    seed_xy: np.ndarray,
    radius: float,
    minimum_pixels: int,
) -> tuple[Optional[np.ndarray], int, Optional[np.ndarray]]:
    restricted = mask & _circle_mask(mask.shape, seed_xy, radius)
    count, labels, stats, centers = cv2.connectedComponentsWithStats(
        restricted,
        connectivity=8,
    )
    candidates: list[tuple[float, int]] = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        distance = float(np.linalg.norm(centers[label] - seed_xy))
        if minimum_pixels <= area <= 20000 and distance <= 55.0:
            candidates.append((distance - area * 0.001, label))
    if not candidates:
        return None, 0, None
    _, selected = min(candidates)
    component = np.where(labels == selected, 255, 0).astype(np.uint8)
    contours, _ = cv2.findContours(
        component,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        return None, 0, None
    hull = cv2.convexHull(max(contours, key=cv2.contourArea))
    silhouette = np.zeros_like(component)
    cv2.fillConvexPoly(silhouette, hull, 255)
    return centroid(silhouette), int(cv2.countNonZero(silhouette)), silhouette


def _principal_axis(mask: np.ndarray) -> tuple[float, float]:
    y, x = np.nonzero(mask)
    if len(x) < 3:
        return float("nan"), 0.0
    points = np.column_stack([x, y]).astype(float)
    covariance = np.cov(points, rowvar=False)
    values, vectors = np.linalg.eigh(covariance)
    order = np.argsort(values)
    major = vectors[:, order[-1]]
    major_value = max(0.0, float(values[order[-1]]))
    minor_value = max(0.0, float(values[order[-2]]))
    angle = math.degrees(math.atan2(-major[1], major[0]))
    anisotropy = (major_value - minor_value) / max(
        1e-9,
        major_value + minor_value,
    )
    return angle, float(max(0.0, min(1.0, anisotropy)))


def _nearest_periodic(
    angle_deg: float,
    reference_deg: Optional[float],
    period_deg: float,
) -> float:
    if reference_deg is None:
        return angle_deg
    turns = round((reference_deg - angle_deg) / period_deg)
    return angle_deg + turns * period_deg


def _cue_distance_is_valid(
    distance: float,
    recent_distances: Sequence[float],
    minimum_distance: float,
    relative_tolerance: float,
) -> bool:
    if distance < minimum_distance:
        return False
    if len(recent_distances) < 5:
        return True
    expected = float(np.median(recent_distances))
    return abs(distance - expected) <= relative_tolerance * expected


def _yaw_innovation_is_valid(
    candidate_yaw: float,
    prior_yaw: Optional[float],
    maximum_rate_deg_s: float,
    frame_period_s: float,
) -> bool:
    if prior_yaw is None:
        return True
    return abs(candidate_yaw - prior_yaw) <= maximum_rate_deg_s * frame_period_s


def _predict_tracker_position(
    observed_xy: Optional[np.ndarray],
    velocity_px_per_frame: np.ndarray,
    observed_frame: Optional[int],
    frame_index: int,
) -> Optional[np.ndarray]:
    if observed_xy is None:
        return None
    elapsed_frames = (
        1 if observed_frame is None else max(1, frame_index - observed_frame)
    )
    return observed_xy + velocity_px_per_frame * elapsed_frames


def _initial_tracking_seeds(
    args: argparse.Namespace,
    grid: aruco.GridCalibration,
) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    if args.initial_x_cell is None:
        return None, None

    bottom_y = grid.y_origin_px + grid.cells * grid.y_pitch_px
    position = np.asarray(
        [
            grid.x_origin_px + args.initial_x_cell * grid.x_pitch_px,
            bottom_y - args.initial_y_cell * grid.y_pitch_px,
        ],
        dtype=float,
    )
    label_seed = None
    if args.position_source == "label" and args.label_colour == "blue":
        label_seed = position.copy()
    return position, label_seed


def _update_tracker_velocity(
    velocity_px_per_frame: np.ndarray,
    previous_xy: np.ndarray,
    observed_xy: np.ndarray,
    previous_frame: int,
    frame_index: int,
) -> np.ndarray:
    elapsed_frames = max(1, frame_index - previous_frame)
    measured_velocity = (observed_xy - previous_xy) / elapsed_frames
    return 0.65 * velocity_px_per_frame + 0.35 * np.clip(
        measured_velocity,
        -30.0,
        30.0,
    )


def _pixels_per_mm(
    grid: aruco.GridCalibration,
    cell_size_mm: Optional[float],
) -> float:
    physical_pitch_mm = cell_size_mm if cell_size_mm is not None else 90.0
    return math.sqrt(grid.x_pitch_px * grid.y_pitch_px) / physical_pitch_mm


def _expected_label_pixels(
    grid: aruco.GridCalibration,
    label_diameter_mm: float,
    cell_size_mm: Optional[float],
) -> float:
    pixels_per_mm = _pixels_per_mm(grid, cell_size_mm)
    radius_px = 0.5 * label_diameter_mm * pixels_per_mm
    return math.pi * radius_px * radius_px


def _cue_brightness(
    frame: np.ndarray,
    search_mask: np.ndarray,
) -> float:
    blue, green, red = cv2.split(frame.astype(np.int16))
    excess = red - np.maximum(green, blue)
    values = excess[search_mask > 0]
    if not len(values):
        return 0.0
    count = max(1, min(25, len(values)))
    top = np.partition(values, len(values) - count)[-count:]
    return float(np.mean(top))


def detect_pose(
    frame: np.ndarray,
    background: np.ndarray,
    grid: aruco.GridCalibration,
    args: argparse.Namespace,
    predicted_green_xy: Optional[np.ndarray],
    prior_yaw_deg: Optional[float],
    predicted_cue_xy: Optional[np.ndarray],
    predicted_label_xy: Optional[np.ndarray],
    predicted_front_label_xy: Optional[np.ndarray],
    expected_label_pixels: float,
    expected_front_label_pixels: float,
    expected_front_label_distance_px: float,
    front_label_distance_tolerance_px: float,
    front_label_bias_xy_px: np.ndarray,
    cue_distances: deque[float],
    prediction_is_seed: bool,
    frame_period_s: float,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    float,
    int,
    int,
    int,
    int,
    int,
    float,
    float,
    float,
    bool,
    bool,
    str,
    str,
]:
    blur_size = (args.foreground_blur, args.foreground_blur)
    blurred = cv2.GaussianBlur(frame, blur_size, 0)
    background_blurred = cv2.GaussianBlur(background, blur_size, 0)
    delta = np.max(
        cv2.absdiff(blurred, background_blurred),
        axis=2,
    )
    foreground = np.where(
        delta >= args.foreground_threshold,
        255,
        0,
    ).astype(np.uint8)
    foreground &= _board_mask(foreground.shape, grid)
    foreground = cv2.morphologyEx(
        foreground,
        cv2.MORPH_OPEN,
        np.ones((args.morph_open, args.morph_open), dtype=np.uint8),
    )
    foreground = cv2.morphologyEx(
        foreground,
        cv2.MORPH_CLOSE,
        np.ones((args.morph_close, args.morph_close), dtype=np.uint8),
    )

    board = _board_mask(foreground.shape, grid)
    label_xy: Optional[np.ndarray] = None
    label_count = 0
    if args.label_colour == "blue":
        label_xy, label_count, _ = _label_component(
            blue_label_mask(frame) & board,
            args.minimum_label_pixels,
            args.maximum_label_pixels,
            expected_label_pixels,
            predicted_label_xy,
            max(20.0, min(45.0, args.tracking_radius_px)),
        )

    green = green_mask(frame) & board
    green_prediction = predicted_green_xy
    green_prediction_is_label_seed = False
    if green_prediction is None and label_xy is not None:
        green_prediction = label_xy
        green_prediction_is_label_seed = True
    green_maximum_distance = None
    if green_prediction is not None:
        green_maximum_distance = (
            args.tracking_radius_px
            if prediction_is_seed or green_prediction_is_label_seed
            else 45.0
        )
    green_xy, green_count, _ = _component_near(
        green,
        args.minimum_green_pixels,
        9000,
        green_prediction,
        maximum_distance=green_maximum_distance,
        prefer_largest=prediction_is_seed or green_prediction_is_label_seed,
    )

    if args.label_colour == "blue" and label_xy is None:
        recovery_prediction = (
            predicted_label_xy
            if predicted_label_xy is not None
            else green_xy
        )
        if recovery_prediction is not None:
            label_xy, label_count, _ = _label_component(
                blue_label_glare_mask(frame) & board,
                args.minimum_label_pixels,
                args.maximum_label_pixels,
                expected_label_pixels,
                recovery_prediction,
                max(20.0, min(30.0, args.tracking_radius_px)),
            )

    front_label_xy: Optional[np.ndarray] = None
    front_label_count = 0
    pair_prediction = (
        predicted_label_xy
        if predicted_label_xy is not None
        else green_xy
    )
    if args.front_label_colour == "red" and pair_prediction is not None:
        paired_label_xy, paired_label_count, paired_front_xy, paired_front_count = (
            _blue_front_label_pair(
                cv2.bitwise_or(
                    blue_label_mask(frame),
                    blue_label_glare_mask(frame),
                )
                & board,
                front_label_mask(frame) & board,
                max(6, args.minimum_label_pixels // 3),
                args.maximum_label_pixels,
                expected_label_pixels,
                args.minimum_front_label_pixels,
                args.maximum_front_label_pixels,
                expected_front_label_pixels,
                expected_front_label_distance_px,
                front_label_distance_tolerance_px,
                pair_prediction,
                max(20.0, min(30.0, args.tracking_radius_px)),
            )
        )
        if paired_label_xy is not None:
            label_xy = paired_label_xy
            label_count = paired_label_count
            front_label_xy = paired_front_xy
            front_label_count = paired_front_count
    if (
        args.front_label_colour == "red"
        and label_xy is not None
        and front_label_xy is None
    ):
        front_label_xy, front_label_count, _ = _front_label_component(
            front_label_mask(frame) & board,
            label_xy,
            args.minimum_front_label_pixels,
            args.maximum_front_label_pixels,
            expected_front_label_pixels,
            expected_front_label_distance_px,
            front_label_distance_tolerance_px,
            predicted_front_label_xy,
            max(20.0, min(45.0, args.tracking_radius_px)),
        )

    body_seed = green_xy if green_xy is not None else label_xy
    body_xy: Optional[np.ndarray] = None
    body_count = 0
    body: Optional[np.ndarray] = None
    search = np.zeros(foreground.shape, dtype=np.uint8)
    if body_seed is not None:
        body_xy, body_count, body = _foreground_cluster(
            foreground,
            body_seed,
            args.tracking_radius_px,
            args.minimum_body_pixels,
        )
        search = _circle_mask(
            foreground.shape,
            body_seed,
            args.tracking_radius_px,
        )

    cue_xy: Optional[np.ndarray] = None
    cue_count = 0
    if args.cue_colour == "red" and body_xy is not None:
        cue_candidates = red_mask(frame) & search & board
        cue_prediction = (
            predicted_cue_xy if predicted_cue_xy is not None else body_seed
        )
        cue_xy, cue_count, _ = _component_near(
            cue_candidates,
            args.minimum_cue_pixels,
            args.maximum_cue_pixels,
            cue_prediction,
            maximum_distance=85.0,
        )
    cue_value = _cue_brightness(frame, search)

    axis_angle, anisotropy = (
        _principal_axis(body)
        if body is not None
        else (float("nan"), 0.0)
    )
    heading_valid = False
    heading_source = "missing"
    yaw = float(prior_yaw_deg) if prior_yaw_deg is not None else float("nan")
    if front_label_xy is not None and label_xy is not None:
        vector = front_label_xy - label_xy - front_label_bias_xy_px
        front_direction = math.degrees(math.atan2(-vector[1], vector[0]))
        candidate_yaw = _nearest_periodic(
            front_direction + args.front_label_yaw_offset_deg,
            prior_yaw_deg if prior_yaw_deg is not None else args.initial_yaw_deg,
            360.0,
        )
        if _yaw_innovation_is_valid(
            candidate_yaw,
            prior_yaw_deg,
            args.maximum_yaw_rate_deg_s,
            frame_period_s,
        ):
            yaw = candidate_yaw
            heading_valid = True
            heading_source = "front_label"
        else:
            heading_source = "front_label_rejected_yaw_rate"

    cue_was_rejected = False
    if not heading_valid and cue_xy is not None and body_xy is not None:
        vector = cue_xy - body_xy
        cue_distance = float(np.linalg.norm(vector))
        distance_valid = _cue_distance_is_valid(
            cue_distance,
            cue_distances,
            args.minimum_cue_lever_arm_px,
            args.cue_distance_relative_tolerance,
        )
        if distance_valid:
            cue_direction = math.degrees(math.atan2(-vector[1], vector[0]))
            candidate_yaw = _nearest_periodic(
                cue_direction + args.cue_yaw_offset_deg,
                prior_yaw_deg if prior_yaw_deg is not None else args.initial_yaw_deg,
                360.0,
            )
            if _yaw_innovation_is_valid(
                candidate_yaw,
                prior_yaw_deg,
                args.maximum_yaw_rate_deg_s,
                frame_period_s,
            ):
                yaw = candidate_yaw
                cue_distances.append(cue_distance)
                heading_valid = True
                heading_source = "colour_cue"
            else:
                cue_was_rejected = True
                heading_source = "cue_rejected_yaw_rate"
        else:
            cue_was_rejected = True
            heading_source = "cue_rejected_geometry"

    if (
        not heading_valid
        and cue_xy is None
        and not cue_was_rejected
        and body_xy is not None
    ):
        if math.isfinite(axis_angle) and anisotropy >= args.minimum_axis_anisotropy:
            candidate_yaw = _nearest_periodic(
                axis_angle + args.axis_yaw_offset_deg,
                prior_yaw_deg if prior_yaw_deg is not None else args.initial_yaw_deg,
                180.0,
            )
            if _yaw_innovation_is_valid(
                candidate_yaw,
                prior_yaw_deg,
                args.maximum_yaw_rate_deg_s,
                frame_period_s,
            ):
                yaw = candidate_yaw
                heading_valid = True
                heading_source = "foreground_axis"
            else:
                heading_source = "axis_rejected_yaw_rate"
        else:
            heading_source = "axis_rejected_geometry"

    accepted_cue_xy = (
        cue_xy
        if cue_xy is not None and heading_valid and heading_source == "colour_cue"
        else None
    )
    position_source = args.position_source
    position_valid = False
    nan_xy = np.full(2, np.nan, dtype=float)
    if args.position_source == "label":
        if label_xy is not None:
            position_xy = label_xy
            position_source = "blue_label"
            position_valid = True
        else:
            position_xy = nan_xy
            position_source = "label_missing"
    elif args.position_source == "cue" and body_xy is not None:
        if accepted_cue_xy is not None:
            position_xy = accepted_cue_xy
            position_valid = True
        elif cue_distances and math.isfinite(yaw):
            cue_direction = math.radians(yaw - args.cue_yaw_offset_deg)
            distance = float(np.median(cue_distances))
            position_xy = body_xy + np.asarray(
                [
                    distance * math.cos(cue_direction),
                    -distance * math.sin(cue_direction),
                ]
            )
            position_source = "cue_from_body"
            position_valid = True
        else:
            position_xy = body_xy
            position_source = "body_fallback"
            position_valid = True
    elif args.position_source == "green":
        if green_xy is not None:
            position_xy = green_xy
            position_valid = True
        else:
            position_xy = nan_xy
            position_source = "green_missing"
    elif body_xy is not None:
        position_xy = body_xy
        position_valid = True
    else:
        position_xy = nan_xy
        position_source = "body_missing"

    area_score = min(1.0, body_count / max(1.0, args.minimum_body_pixels * 4))
    cue_score = 1.0 if accepted_cue_xy is not None else 0.45
    label_score = (
        math.exp(
            -abs(math.log(label_count / max(1.0, expected_label_pixels)))
        )
        if label_xy is not None and label_count > 0
        else 0.0
    )
    if args.position_source == "label":
        confidence = float(
            max(
                0.0,
                min(
                    1.0,
                    0.60 * label_score
                    + 0.25 * area_score
                    + 0.15 * float(heading_valid),
                ),
            )
        )
    else:
        confidence = float(
            max(
                0.0,
                min(
                    1.0,
                    0.45 * area_score
                    + 0.25 * anisotropy
                    + 0.30 * cue_score,
                ),
            )
        )
    return (
        position_xy,
        body_xy if body_xy is not None else nan_xy,
        accepted_cue_xy if accepted_cue_xy is not None else nan_xy,
        label_xy if label_xy is not None else nan_xy,
        front_label_xy if front_label_xy is not None else nan_xy,
        green_xy if green_xy is not None else nan_xy,
        yaw,
        body_count,
        green_count,
        cue_count,
        label_count,
        front_label_count,
        cue_value,
        anisotropy,
        confidence,
        position_valid,
        heading_valid,
        heading_source,
        position_source,
    )


def interpolate_detections(
    detections: Sequence[Detection],
    allow_missing_heading: bool = False,
    fallback_yaw_deg: float = 0.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    valid = np.asarray([item.pose_valid for item in detections], dtype=bool)
    heading_valid = np.asarray(
        [item.heading_valid for item in detections],
        dtype=bool,
    )
    indexes = np.arange(len(detections))
    good = np.flatnonzero(valid)
    if len(good) < 2:
        raise RuntimeError("fewer than two valid markerless poses")
    good_heading = np.flatnonzero(heading_valid)
    if len(good_heading) < 2 and not allow_missing_heading:
        raise RuntimeError("fewer than two valid markerless headings")
    raw_xy = np.asarray([item.position_xy for item in detections], dtype=float)
    raw_yaw = np.asarray(
        [item.yaw_unwrapped_deg for item in detections],
        dtype=float,
    )
    filled_x = np.interp(indexes, good, raw_xy[good, 0])
    filled_y = np.interp(indexes, good, raw_xy[good, 1])
    if len(good_heading) >= 2:
        filled_yaw = np.interp(
            indexes,
            good_heading,
            raw_yaw[good_heading],
        )
    else:
        filled_yaw = np.full(len(detections), fallback_yaw_deg, dtype=float)
    return (
        np.column_stack([filled_x, filled_y]),
        filled_yaw,
        valid,
        heading_valid,
    )


def convert_track(
    detections: Sequence[Detection],
    grid: aruco.GridCalibration,
    timestamps: np.ndarray,
    smooth_window: int,
    cell_size_mm: Optional[float],
    position_only: bool = False,
    initial_yaw_deg: Optional[float] = None,
    metric_geometry: Optional[board_metric_geometry.BoardMetricGeometry] = None,
) -> dict[str, np.ndarray]:
    raw_xy, raw_yaw, valid, heading_valid = interpolate_detections(
        detections,
        allow_missing_heading=position_only,
        fallback_yaw_deg=(initial_yaw_deg if initial_yaw_deg is not None else 0.0),
    )
    smooth_x = aruco.savitzky_golay(raw_xy[:, 0], smooth_window)
    smooth_y = aruco.savitzky_golay(raw_xy[:, 1], smooth_window)
    smooth_yaw = aruco.savitzky_golay(raw_yaw, smooth_window)
    bottom_y = grid.y_origin_px + grid.cells * grid.y_pitch_px
    x_cell_raw = (raw_xy[:, 0] - grid.x_origin_px) / grid.x_pitch_px
    y_cell_raw = (bottom_y - raw_xy[:, 1]) / grid.y_pitch_px
    x_cell = (smooth_x - grid.x_origin_px) / grid.x_pitch_px
    y_cell = (bottom_y - smooth_y) / grid.y_pitch_px
    vx_cell = np.gradient(x_cell, timestamps)
    vy_cell = np.gradient(y_cell, timestamps)
    result = {
        "video_pts_s": timestamps,
        "canonical_x_px_raw": raw_xy[:, 0],
        "canonical_y_px_raw": raw_xy[:, 1],
        "canonical_x_px": smooth_x,
        "canonical_y_px": smooth_y,
        "x_cell_raw": x_cell_raw,
        "y_cell_raw": y_cell_raw,
        "x_cell": x_cell,
        "y_cell": y_cell,
        "yaw_deg_raw_unwrapped": raw_yaw,
        "yaw_deg_unwrapped": smooth_yaw,
        "yaw_deg": np.mod(smooth_yaw, 360.0),
        "vx_cell_s": vx_cell,
        "vy_cell_s": vy_cell,
        "speed_cell_s": np.hypot(vx_cell, vy_cell),
        "pose_valid": valid.astype(int),
        "heading_valid": heading_valid.astype(int),
    }
    if metric_geometry is not None:
        mapped_raw = metric_geometry.map_points(raw_xy)
        mapped = metric_geometry.map_points(np.column_stack([smooth_x, smooth_y]))
        vx_mm = np.gradient(mapped[:, 0], timestamps)
        vy_mm = np.gradient(mapped[:, 1], timestamps)
        result.update(
            {
                "x_mm_raw": mapped_raw[:, 0],
                "y_mm_raw": mapped_raw[:, 1],
                "x_mm": mapped[:, 0],
                "y_mm": mapped[:, 1],
                "vx_mm_s": vx_mm,
                "vy_mm_s": vy_mm,
                "speed_mm_s": np.hypot(vx_mm, vy_mm),
            }
        )
    elif cell_size_mm is not None:
        result.update(
            {
                "x_mm_raw": x_cell_raw * cell_size_mm,
                "y_mm_raw": y_cell_raw * cell_size_mm,
                "x_mm": x_cell * cell_size_mm,
                "y_mm": y_cell * cell_size_mm,
                "vx_mm_s": vx_cell * cell_size_mm,
                "vy_mm_s": vy_cell * cell_size_mm,
                "speed_mm_s": np.hypot(vx_cell, vy_cell) * cell_size_mm,
            }
        )
    return result


def _fmt(value: float, digits: int = 6) -> str:
    if not math.isfinite(float(value)):
        return ""
    return f"{float(value):.{digits}f}"


def write_csv(
    path: Path,
    detections: Sequence[Detection],
    converted: dict[str, np.ndarray],
    calibrations: Sequence[aruco.FrameCalibration],
) -> None:
    fields = [
        "frame",
        "time_s",
        "video_pts_s",
        "video_pts_ns",
        "canonical_x_px_raw",
        "canonical_y_px_raw",
        "canonical_x_px",
        "canonical_y_px",
        "x_cell_raw",
        "y_cell_raw",
        "x_cell",
        "y_cell",
        "x_mm",
        "y_mm",
        "x_mm_raw",
        "y_mm_raw",
        "yaw_deg_raw_unwrapped",
        "yaw_deg_unwrapped",
        "yaw_deg",
        "vx_cell_s",
        "vy_cell_s",
        "speed_cell_s",
        "vx_mm_s",
        "vy_mm_s",
        "speed_mm_s",
        "pose_valid",
        "heading_valid",
        "pose_confidence",
        "body_pixel_count",
        "green_pixel_count",
        "cue_pixel_count",
        "label_pixel_count",
        "front_label_pixel_count",
        "cue_brightness",
        "axis_anisotropy",
        "heading_source",
        "position_source",
        "body_x_px",
        "body_y_px",
        "cue_x_px",
        "cue_y_px",
        "label_x_px",
        "label_y_px",
        "front_label_x_px",
        "front_label_y_px",
        "front_label_distance_px",
        "fixed_ids",
        "homography_rmse_px",
        "homography_used_previous",
    ]
    with path.open("w", encoding="ascii", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, detection in enumerate(detections):
            calibration = calibrations[index]
            row: dict[str, object] = {
                "frame": index,
                "time_s": _fmt(converted["video_pts_s"][index]),
                "video_pts_s": _fmt(converted["video_pts_s"][index]),
                "video_pts_ns": int(
                    round(converted["video_pts_s"][index] * 1_000_000_000)
                ),
                "pose_valid": int(detection.pose_valid),
                "heading_valid": int(detection.heading_valid),
                "pose_confidence": _fmt(detection.pose_confidence),
                "body_pixel_count": detection.body_pixel_count,
                "green_pixel_count": detection.green_pixel_count,
                "cue_pixel_count": detection.cue_pixel_count,
                "label_pixel_count": detection.label_pixel_count,
                "front_label_pixel_count": detection.front_label_pixel_count,
                "cue_brightness": _fmt(detection.cue_brightness, 3),
                "axis_anisotropy": _fmt(detection.axis_anisotropy),
                "heading_source": detection.heading_source,
                "position_source": detection.position_source,
                "body_x_px": _fmt(detection.body_xy[0], 4),
                "body_y_px": _fmt(detection.body_xy[1], 4),
                "cue_x_px": _fmt(detection.cue_xy[0], 4),
                "cue_y_px": _fmt(detection.cue_xy[1], 4),
                "label_x_px": _fmt(detection.label_xy[0], 4),
                "label_y_px": _fmt(detection.label_xy[1], 4),
                "front_label_x_px": _fmt(detection.front_label_xy[0], 4),
                "front_label_y_px": _fmt(detection.front_label_xy[1], 4),
                "front_label_distance_px": _fmt(
                    np.linalg.norm(detection.front_label_xy - detection.label_xy),
                    4,
                ),
                "fixed_ids": "|".join(map(str, calibration.fixed_ids)),
                "homography_rmse_px": _fmt(
                    calibration.reprojection_rmse_px,
                ),
                "homography_used_previous": int(calibration.used_previous_homography),
            }
            for key in (
                "canonical_x_px_raw",
                "canonical_y_px_raw",
                "canonical_x_px",
                "canonical_y_px",
                "x_cell_raw",
                "y_cell_raw",
                "x_cell",
                "y_cell",
                "yaw_deg_raw_unwrapped",
                "yaw_deg_unwrapped",
                "yaw_deg",
                "vx_cell_s",
                "vy_cell_s",
                "speed_cell_s",
            ):
                row[key] = _fmt(converted[key][index])
            for key in (
                "x_mm_raw",
                "y_mm_raw",
                "x_mm",
                "y_mm",
                "vx_mm_s",
                "vy_mm_s",
                "speed_mm_s",
            ):
                row[key] = _fmt(converted[key][index]) if key in converted else ""
            writer.writerow(row)


def colour_for_fraction(fraction: float) -> tuple[int, int, int]:
    fraction = max(0.0, min(1.0, fraction))
    hue = int(round((1.0 - fraction) * 120.0))
    pixel = np.uint8([[[hue // 2, 230, 255]]])
    return tuple(int(value) for value in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0])


def draw_path(
    image: np.ndarray,
    points: np.ndarray,
    upto: int,
) -> None:
    if upto < 1:
        return
    denominator = max(1, len(points) - 1)
    for index in range(1, min(upto + 1, len(points))):
        cv2.line(
            image,
            tuple(np.rint(points[index - 1]).astype(int)),
            tuple(np.rint(points[index]).astype(int)),
            colour_for_fraction(index / denominator),
            3,
            cv2.LINE_AA,
        )


def render_plot(
    path: Path,
    background: np.ndarray,
    converted: dict[str, np.ndarray],
) -> None:
    image = background.copy()
    points = np.column_stack([converted["canonical_x_px"], converted["canonical_y_px"]])
    draw_path(image, points, len(points) - 1)
    if len(points):
        cv2.circle(
            image,
            tuple(np.rint(points[0]).astype(int)),
            8,
            (255, 255, 255),
            -1,
        )
        cv2.circle(
            image,
            tuple(np.rint(points[-1]).astype(int)),
            8,
            (0, 0, 255),
            -1,
        )
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"failed to write {path}")


def render_video(
    path: Path,
    video_path: Path,
    calibrations: Sequence[aruco.FrameCalibration],
    info: aruco.VideoInfo,
    converted: dict[str, np.ndarray],
    detections: Sequence[Detection],
    canonical_size: int,
) -> Path:
    writer, actual_path = aruco.open_writer(
        path,
        info.fps,
        (canonical_size, canonical_size),
    )
    capture, _ = aruco.open_video(video_path)
    points = np.column_stack([converted["canonical_x_px"], converted["canonical_y_px"]])
    frame_index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        top = cv2.warpPerspective(
            frame,
            calibrations[frame_index].homography,
            (canonical_size, canonical_size),
        )
        draw_path(top, points, frame_index)
        center = points[frame_index]
        yaw = math.radians(converted["yaw_deg_unwrapped"][frame_index])
        end = center + np.asarray([38.0 * math.cos(yaw), -38.0 * math.sin(yaw)])
        cv2.arrowedLine(
            top,
            tuple(np.rint(center).astype(int)),
            tuple(np.rint(end).astype(int)),
            (0, 255, 255),
            3,
            cv2.LINE_AA,
            tipLength=0.25,
        )
        detection = detections[frame_index]
        if np.all(np.isfinite(detection.label_xy)):
            cv2.circle(
                top,
                tuple(np.rint(detection.label_xy).astype(int)),
                7,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
        if np.all(np.isfinite(detection.front_label_xy)):
            cv2.line(
                top,
                tuple(np.rint(detection.label_xy).astype(int)),
                tuple(np.rint(detection.front_label_xy).astype(int)),
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.circle(
                top,
                tuple(np.rint(detection.front_label_xy).astype(int)),
                7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        cv2.putText(
            top,
            "t={:.3f}s valid={} conf={:.2f} {} / {}".format(
                converted["video_pts_s"][frame_index],
                "{}/{}".format(
                    int(detection.pose_valid),
                    int(detection.heading_valid),
                ),
                detection.pose_confidence,
                detection.heading_source,
                detection.position_source,
            ),
            (18, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        writer.write(top)
        frame_index += 1
    capture.release()
    writer.release()
    return actual_path


def percentile_summary(values: Sequence[float]) -> dict[str, float]:
    array = np.asarray(
        [value for value in values if math.isfinite(float(value))],
        dtype=float,
    )
    if not len(array):
        return {}
    return {
        "min": float(np.min(array)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        measured_layout: Optional[board_layout.BoardLayout] = None
        dense_metric: Optional[board_metric_geometry.BoardMetricGeometry] = None
        if args.board_layout is not None:
            measured_layout = board_layout.load(
                args.board_layout,
                args.canonical_size,
            )
            if args.board_metric_geometry is not None:
                dense_metric = board_metric_geometry.load_geometry(
                    args.board_metric_geometry,
                    expected_board_layout_sha256=sha256_file(args.board_layout),
                    expected_canonical_size_px=args.canonical_size,
                    require_safety_qualified=True,
                )
            if (
                args.grid_cells is not None
                and args.grid_cells != measured_layout.grid.cells
            ):
                raise ValueError("--grid-cells conflicts with board-layout grid.cells")
            if args.cell_size_mm is not None and not math.isclose(
                args.cell_size_mm,
                measured_layout.grid_pitch_mm,
                rel_tol=1e-6,
                abs_tol=1e-6,
            ):
                raise ValueError(
                    "--cell-size-mm conflicts with board-layout grid.pitch_mm"
                )
            args.grid_cells = measured_layout.grid.cells
            args.cell_size_mm = measured_layout.grid_pitch_mm
            args.cell_size_confirmed = True
        elif args.grid_cells is None:
            args.grid_cells = 8
        front_label_calibration: Optional[dict[str, object]] = None
        if args.front_label_calibration is not None:
            front_label_calibration = load_front_label_calibration(
                args.front_label_calibration,
                args.front_label_distance_mm,
            )
            args.front_label_bias_right_mm = float(
                front_label_calibration["bias_right_mm"]
            )
            args.front_label_bias_forward_mm = float(
                front_label_calibration["bias_forward_mm"]
            )
        output_dir = (
            args.output_dir
            if args.output_dir is not None
            else args.video.with_name(args.video.stem + "_markerless")
        )
        output_dir.mkdir(parents=True, exist_ok=True)
        info, observations, detection_counts = aruco.collect_fixed_observations(
            args.video
        )
        calibrations, target_corners = aruco.build_calibrations(
            observations,
            args.canonical_size,
            args.marker_margin,
            (
                measured_layout.target_corners_px
                if measured_layout is not None
                else None
            ),
            args.maximum_consecutive_homography_fallback_frames,
        )
        homography_fallback_fraction = sum(
            item.used_previous_homography for item in calibrations
        ) / len(calibrations)
        if homography_fallback_fraction > args.maximum_homography_fallback_fraction:
            raise RuntimeError(
                "homography fallback fraction {:.3f} exceeds {:.3f}".format(
                    homography_fallback_fraction,
                    args.maximum_homography_fallback_fraction,
                )
            )
        timestamps, timestamp_source = video_timestamps(
            args.video,
            info.frame_count,
        )
        background_info = info
        background_calibrations = calibrations
        background_path = args.video
        background_fallback_fraction = homography_fallback_fraction
        if args.background_video is not None:
            background_path = args.background_video
            background_info, background_observations, _ = (
                aruco.collect_fixed_observations(background_path)
            )
            background_calibrations, _ = aruco.build_calibrations(
                background_observations,
                args.canonical_size,
                args.marker_margin,
                (
                    measured_layout.target_corners_px
                    if measured_layout is not None
                    else None
                ),
            )
            background_fallback_fraction = sum(
                item.used_previous_homography for item in background_calibrations
            ) / len(background_calibrations)
            if background_fallback_fraction > args.maximum_homography_fallback_fraction:
                raise RuntimeError(
                    "background homography fallback fraction "
                    "{:.3f} exceeds {:.3f}".format(
                        background_fallback_fraction,
                        args.maximum_homography_fallback_fraction,
                    )
                )
        background = build_background(
            background_path,
            background_calibrations,
            background_info,
            args.canonical_size,
            args.background_samples,
        )
        grid = (
            measured_layout.grid
            if measured_layout is not None
            else aruco.detect_grid(
                background,
                args.grid_cells,
                args.marker_margin,
            )
        )
        expected_label_pixels = _expected_label_pixels(
            grid,
            args.label_diameter_mm,
            args.cell_size_mm,
        )
        pixels_per_mm = _pixels_per_mm(grid, args.cell_size_mm)
        expected_front_label_pixels = _expected_label_pixels(
            grid,
            args.front_label_diameter_mm,
            args.cell_size_mm,
        )
        expected_front_label_distance_px = (
            args.front_label_distance_mm * pixels_per_mm
        )
        front_label_distance_tolerance_px = (
            args.front_label_distance_tolerance_mm * pixels_per_mm
        )
        front_label_bias_xy_px = np.asarray(
            [
                args.front_label_bias_right_mm * pixels_per_mm,
                -args.front_label_bias_forward_mm * pixels_per_mm,
            ],
            dtype=float,
        )
        reference_path = output_dir / "reference_topview.png"
        if not cv2.imwrite(str(reference_path), background):
            raise RuntimeError(f"failed to write {reference_path}")

        capture, _ = aruco.open_video(args.video)
        detections: list[Detection] = []
        prior_green, prior_label = _initial_tracking_seeds(args, grid)
        prior_green_is_observed = False
        prior_green_frame: Optional[int] = None
        prior_cue: Optional[np.ndarray] = None
        prior_cue_frame: Optional[int] = None
        prior_label_frame: Optional[int] = None
        prior_front_label: Optional[np.ndarray] = None
        prior_front_label_frame: Optional[int] = None
        tracker_velocity = np.zeros(2, dtype=float)
        label_velocity = np.zeros(2, dtype=float)
        front_label_velocity = np.zeros(2, dtype=float)
        prior_yaw = args.initial_yaw_deg
        cue_distances: deque[float] = deque(maxlen=120)
        frame_index = 0
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            top = cv2.warpPerspective(
                frame,
                calibrations[frame_index].homography,
                (args.canonical_size, args.canonical_size),
            )
            predicted_green = _predict_tracker_position(
                prior_green,
                tracker_velocity,
                prior_green_frame,
                frame_index,
            )
            predicted_cue = _predict_tracker_position(
                prior_cue,
                tracker_velocity,
                prior_cue_frame,
                frame_index,
            )
            predicted_label = _predict_tracker_position(
                prior_label,
                label_velocity,
                prior_label_frame,
                frame_index,
            )
            predicted_front_label = _predict_tracker_position(
                prior_front_label,
                front_label_velocity,
                prior_front_label_frame,
                frame_index,
            )
            (
                position_xy,
                body_xy,
                cue_xy,
                label_xy,
                front_label_xy,
                detected_green,
                yaw,
                body_count,
                green_count,
                cue_count,
                label_count,
                front_label_count,
                cue_value,
                anisotropy,
                confidence,
                valid,
                heading_valid,
                heading_source,
                position_source,
            ) = detect_pose(
                top,
                background,
                grid,
                args,
                predicted_green,
                prior_yaw,
                predicted_cue,
                predicted_label,
                predicted_front_label,
                expected_label_pixels,
                expected_front_label_pixels,
                expected_front_label_distance_px,
                front_label_distance_tolerance_px,
                front_label_bias_xy_px,
                cue_distances,
                prior_green is not None and not prior_green_is_observed,
                (
                    float(timestamps[frame_index] - timestamps[frame_index - 1])
                    if frame_index > 0
                    else 1.0 / info.fps
                ),
            )
            detections.append(
                Detection(
                    frame=frame_index,
                    time_s=float(timestamps[frame_index]),
                    position_xy=position_xy,
                    body_xy=body_xy,
                    cue_xy=cue_xy,
                    label_xy=label_xy,
                    front_label_xy=front_label_xy,
                    yaw_unwrapped_deg=yaw,
                    body_pixel_count=body_count,
                    green_pixel_count=green_count,
                    cue_pixel_count=cue_count,
                    label_pixel_count=label_count,
                    front_label_pixel_count=front_label_count,
                    cue_brightness=cue_value,
                    axis_anisotropy=anisotropy,
                    pose_confidence=confidence,
                    pose_valid=valid,
                    heading_valid=heading_valid,
                    heading_source=heading_source,
                    position_source=position_source,
                )
            )
            if np.all(np.isfinite(detected_green)):
                if (
                    prior_green is not None
                    and prior_green_is_observed
                    and prior_green_frame is not None
                ):
                    tracker_velocity = _update_tracker_velocity(
                        tracker_velocity,
                        prior_green,
                        detected_green,
                        prior_green_frame,
                        frame_index,
                    )
                prior_green = detected_green
                prior_green_is_observed = True
                prior_green_frame = frame_index
            if np.all(np.isfinite(label_xy)):
                if prior_label is not None and prior_label_frame is not None:
                    label_velocity = _update_tracker_velocity(
                        label_velocity,
                        prior_label,
                        label_xy,
                        prior_label_frame,
                        frame_index,
                    )
                prior_label = label_xy
                prior_label_frame = frame_index
            if np.all(np.isfinite(front_label_xy)):
                if (
                    prior_front_label is not None
                    and prior_front_label_frame is not None
                ):
                    front_label_velocity = _update_tracker_velocity(
                        front_label_velocity,
                        prior_front_label,
                        front_label_xy,
                        prior_front_label_frame,
                        frame_index,
                    )
                prior_front_label = front_label_xy
                prior_front_label_frame = frame_index
            if np.all(np.isfinite(cue_xy)):
                prior_cue = cue_xy
                prior_cue_frame = frame_index
            if heading_valid:
                prior_yaw = yaw
            frame_index += 1
        capture.release()
        if frame_index != info.frame_count:
            raise RuntimeError(
                f"decoded {frame_index} frames, expected {info.frame_count}"
            )

        missing_fraction = 1.0 - (
            sum(item.pose_valid for item in detections) / len(detections)
        )
        if missing_fraction > args.maximum_missing_fraction:
            raise RuntimeError(
                "markerless missing fraction {:.3f} exceeds {:.3f}".format(
                    missing_fraction,
                    args.maximum_missing_fraction,
                )
            )
        heading_invalid_fraction = 1.0 - (
            sum(item.heading_valid for item in detections) / len(detections)
        )
        if (
            not args.position_only
            and heading_invalid_fraction > args.maximum_heading_invalid_fraction
        ):
            raise RuntimeError(
                "markerless heading-invalid fraction {:.3f} exceeds {:.3f}".format(
                    heading_invalid_fraction,
                    args.maximum_heading_invalid_fraction,
                )
            )
        converted = convert_track(
            detections,
            grid,
            timestamps,
            args.smooth_window,
            args.cell_size_mm,
            args.position_only,
            args.initial_yaw_deg,
            dense_metric,
        )
        csv_path = output_dir / "trajectory.csv"
        plot_path = output_dir / "trajectory.png"
        write_csv(csv_path, detections, converted, calibrations)
        render_plot(plot_path, background, converted)
        video_output: Optional[Path] = None
        if not args.no_video:
            video_output = render_video(
                output_dir / "trajectory_topview.mp4",
                args.video,
                calibrations,
                info,
                converted,
                detections,
                args.canonical_size,
            )

        source_counts = Counter(item.heading_source for item in detections)
        position_counts = Counter(item.position_source for item in detections)
        cue_lever_arms = [
            float(np.linalg.norm(item.cue_xy - item.body_xy))
            for item in detections
            if np.all(np.isfinite(item.cue_xy)) and np.all(np.isfinite(item.body_xy))
        ]
        front_label_distances_px = [
            float(np.linalg.norm(item.front_label_xy - item.label_xy))
            for item in detections
            if np.all(np.isfinite(item.front_label_xy))
            and np.all(np.isfinite(item.label_xy))
        ]
        front_label_distances_mm = [
            distance / pixels_per_mm for distance in front_label_distances_px
        ]
        homography_errors = [
            item.reprojection_rmse_px
            for item in calibrations
            if math.isfinite(item.reprojection_rmse_px)
        ]
        report = {
            "schema": "nightfall_markerless_trajectory_qa_v3",
            "input": {
                "path": str(args.video.resolve()),
                "sha256": sha256_file(args.video),
                "size_bytes": args.video.stat().st_size,
                "width": info.width,
                "height": info.height,
                "fps_container": info.fps,
                "frames": info.frame_count,
                "duration_pts_s": float(timestamps[-1] - timestamps[0]),
                "timestamp_source": timestamp_source,
            },
            "background": {
                "source": (
                    "separate_empty_clip"
                    if args.background_video is not None
                    else "run_temporal_median"
                ),
                "path": str(background_path.resolve()),
                "sha256": sha256_file(background_path),
                "frames": background_info.frame_count,
                "homography_fallback_fraction": (background_fallback_fraction),
            },
            "fixed_markers": {
                "detection_count": {
                    str(key): value for key, value in sorted(detection_counts.items())
                },
                "homography_marker_ids": list(
                    aruco.FIXED_ORDER
                    if measured_layout is not None
                    else aruco.HOMOGRAPHY_MARKER_IDS
                ),
                "homography_method": (
                    "measured_four_marker_centers"
                    if measured_layout is not None
                    else "legacy_marker_corners"
                ),
                "fallback_frames": sum(
                    item.used_previous_homography for item in calibrations
                ),
                "fallback_fraction": homography_fallback_fraction,
                "maximum_fallback_fraction": (
                    args.maximum_homography_fallback_fraction
                ),
                "reprojection_rmse_px": percentile_summary(homography_errors),
            },
            "grid": {
                "source": (
                    "measured_board_layout"
                    if measured_layout is not None
                    else "detected_bright_lines"
                ),
                "cells": grid.cells,
                "x_origin_px": grid.x_origin_px,
                "y_origin_px": grid.y_origin_px,
                "x_pitch_px": grid.x_pitch_px,
                "y_pitch_px": grid.y_pitch_px,
                "x_peak_contrast": (
                    None if measured_layout is not None else grid.x_peak_contrast
                ),
                "y_peak_contrast": (
                    None if measured_layout is not None else grid.y_peak_contrast
                ),
            },
            "markerless_pose": {
                "method": (
                    "blue center-label position with blue-to-red front-label "
                    "heading; rectified median-background/green-PCB body, "
                    "colour cue, and principal axis are fallbacks"
                ),
                "valid_frames": sum(item.pose_valid for item in detections),
                "missing_frames": sum(not item.pose_valid for item in detections),
                "missing_fraction": missing_fraction,
                "heading_valid_frames": sum(item.heading_valid for item in detections),
                "heading_invalid_frames": sum(
                    not item.heading_valid for item in detections
                ),
                "heading_invalid_fraction": heading_invalid_fraction,
                "heading_gate_applied": not args.position_only,
                "position_only": args.position_only,
                "heading_source_count": dict(source_counts),
                "position_source_count": dict(position_counts),
                "pose_confidence": percentile_summary(
                    [item.pose_confidence for item in detections]
                ),
                "body_pixel_count": percentile_summary(
                    [item.body_pixel_count for item in detections]
                ),
                "cue_pixel_count": percentile_summary(
                    [item.cue_pixel_count for item in detections]
                ),
                "label_detected_frames": sum(
                    item.label_pixel_count > 0 for item in detections
                ),
                "label_missing_frames": sum(
                    item.label_pixel_count == 0 for item in detections
                ),
                "label_pixel_count": percentile_summary(
                    [
                        item.label_pixel_count
                        for item in detections
                        if item.label_pixel_count > 0
                    ]
                ),
                "front_label_detected_frames": sum(
                    item.front_label_pixel_count > 0 for item in detections
                ),
                "front_label_missing_frames": sum(
                    item.front_label_pixel_count == 0 for item in detections
                ),
                "front_label_pixel_count": percentile_summary(
                    [
                        item.front_label_pixel_count
                        for item in detections
                        if item.front_label_pixel_count > 0
                    ]
                ),
                "front_label_distance_px": percentile_summary(
                    front_label_distances_px
                ),
                "front_label_distance_mm_observed": percentile_summary(
                    front_label_distances_mm
                ),
                "accepted_cue_lever_arm_px": percentile_summary(cue_lever_arms),
                "cue_brightness": percentile_summary(
                    [item.cue_brightness for item in detections]
                ),
                "initial_yaw_deg": args.initial_yaw_deg,
                "initial_position_cell": (
                    [args.initial_x_cell, args.initial_y_cell]
                    if args.initial_x_cell is not None
                    else None
                ),
                "cue_yaw_offset_deg": args.cue_yaw_offset_deg,
                "front_label_yaw_offset_deg": (
                    args.front_label_yaw_offset_deg
                ),
                "front_label_bias_right_mm": (
                    args.front_label_bias_right_mm
                ),
                "front_label_bias_forward_mm": (
                    args.front_label_bias_forward_mm
                ),
                "front_label_calibration": (
                    {
                        "path": str(args.front_label_calibration.resolve()),
                        "sha256": sha256_file(args.front_label_calibration),
                        "schema": FRONT_LABEL_CALIBRATION_SCHEMA,
                    }
                    if args.front_label_calibration is not None
                    else None
                ),
                "axis_yaw_offset_deg": args.axis_yaw_offset_deg,
                "minimum_cue_lever_arm_px": (args.minimum_cue_lever_arm_px),
                "cue_distance_relative_tolerance": (
                    args.cue_distance_relative_tolerance
                ),
                "minimum_axis_anisotropy": (args.minimum_axis_anisotropy),
                "maximum_yaw_rate_deg_s": (args.maximum_yaw_rate_deg_s),
                "tracked_point": args.position_source,
                "front_back_ambiguity_resolved": (
                    source_counts.get("front_label", 0) > 0
                    or source_counts.get("colour_cue", 0) > 0
                    or args.initial_yaw_deg is not None
                ),
            },
            "segmentation": {
                "background_samples": args.background_samples,
                "foreground_threshold": args.foreground_threshold,
                "foreground_blur": args.foreground_blur,
                "morph_open": args.morph_open,
                "morph_close": args.morph_close,
                "tracking_radius_px": args.tracking_radius_px,
                "minimum_body_pixels": args.minimum_body_pixels,
                "minimum_green_pixels": args.minimum_green_pixels,
                "label_colour": args.label_colour,
                "label_diameter_mm": args.label_diameter_mm,
                "expected_label_pixels": expected_label_pixels,
                "minimum_label_pixels": args.minimum_label_pixels,
                "maximum_label_pixels": args.maximum_label_pixels,
                "front_label_colour": args.front_label_colour,
                "front_label_diameter_mm": args.front_label_diameter_mm,
                "front_label_distance_mm": args.front_label_distance_mm,
                "front_label_distance_tolerance_mm": (
                    args.front_label_distance_tolerance_mm
                ),
                "expected_front_label_pixels": expected_front_label_pixels,
                "expected_front_label_distance_px": (
                    expected_front_label_distance_px
                ),
                "minimum_front_label_pixels": (
                    args.minimum_front_label_pixels
                ),
                "maximum_front_label_pixels": (
                    args.maximum_front_label_pixels
                ),
                "blue_label_hsv_low": list(BLUE_LABEL_HSV_LOW),
                "blue_label_hsv_high": list(BLUE_LABEL_HSV_HIGH),
                "blue_label_min_blue_green_excess": (
                    BLUE_LABEL_MIN_BLUE_GREEN_EXCESS
                ),
                "blue_label_min_blue_red_excess": (
                    BLUE_LABEL_MIN_BLUE_RED_EXCESS
                ),
                "blue_label_glare_hsv_low": list(BLUE_LABEL_GLARE_HSV_LOW),
                "blue_label_glare_hsv_high": list(BLUE_LABEL_GLARE_HSV_HIGH),
                "blue_label_glare_min_blue_green_excess": (
                    BLUE_LABEL_GLARE_MIN_BLUE_GREEN_EXCESS
                ),
                "blue_label_glare_min_blue_red_excess": (
                    BLUE_LABEL_GLARE_MIN_BLUE_RED_EXCESS
                ),
                "front_label_hsv_low": list(FRONT_LABEL_HSV_LOW),
                "front_label_hsv_high": list(FRONT_LABEL_HSV_HIGH),
                "front_label_hsv_wrap_low": list(FRONT_LABEL_HSV_WRAP_LOW),
                "front_label_hsv_wrap_high": list(FRONT_LABEL_HSV_WRAP_HIGH),
            },
            "metric_scale": {
                "cell_size_mm": args.cell_size_mm,
                "board_layout": (
                    str(args.board_layout.resolve())
                    if args.board_layout is not None
                    else None
                ),
                "board_metric_geometry": (
                    {
                        "path": str(args.board_metric_geometry.resolve()),
                        "sha256": sha256_file(args.board_metric_geometry),
                        "schema": board_metric_geometry.SCHEMA,
                        "fit_p95_mm": dense_metric.fit_stats["p95_mm"],
                        "held_out_p95_mm": dense_metric.validation_stats["p95_mm"],
                        "held_out_max_mm": dense_metric.validation_stats["max_mm"],
                        "safety_qualified": dense_metric.safety_qualified,
                    }
                    if dense_metric is not None
                    and args.board_metric_geometry is not None
                    else None
                ),
                "is_assumed": (
                    dense_metric is None
                    and args.cell_size_mm is not None
                    and not args.cell_size_confirmed
                ),
            },
            "outputs": {
                "trajectory_csv": csv_path.name,
                "trajectory_plot": plot_path.name,
                "reference_topview": reference_path.name,
                "topview_video": (
                    video_output.name if video_output is not None else None
                ),
            },
        }
        calibration = {
            "schema": "nightfall_markerless_board_calibration_v3",
            "fixed_layout": aruco.DEFAULT_FIXED_LAYOUT,
            "measured_layout": (
                {
                    "path": str(args.board_layout.resolve()),
                    "sha256": sha256_file(args.board_layout),
                    "schema": measured_layout.raw.get("schema"),
                    "pixels_per_mm": measured_layout.pixels_per_mm,
                }
                if measured_layout is not None and args.board_layout is not None
                else None
            ),
            "canonical": {
                "size_px": args.canonical_size,
                "marker_margin_px": args.marker_margin,
                "target_corners": {
                    str(marker_id): corners.tolist()
                    for marker_id, corners in target_corners.items()
                },
            },
            "grid": {
                "x_lines_px": grid.x_lines_px.tolist(),
                "y_lines_px": grid.y_lines_px.tolist(),
                "x_origin_px": grid.x_origin_px,
                "y_origin_px": grid.y_origin_px,
                "x_pitch_px": grid.x_pitch_px,
                "y_pitch_px": grid.y_pitch_px,
                "cells": grid.cells,
            },
        }
        (output_dir / "qa_report.json").write_text(
            json.dumps(
                report,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            + "\n",
            encoding="utf-8",
        )
        (output_dir / "calibration.json").write_text(
            json.dumps(
                calibration,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            + "\n",
            encoding="utf-8",
        )
        print(
            "[MARKERLESS] frames={} valid={} missing={} center={} front={} "
            "cue={} timestamp={}".format(
                info.frame_count,
                report["markerless_pose"]["valid_frames"],
                report["markerless_pose"]["missing_frames"],
                report["markerless_pose"]["label_detected_frames"],
                report["markerless_pose"]["front_label_detected_frames"],
                source_counts.get("colour_cue", 0),
                timestamp_source,
            )
        )
        print(f"[MARKERLESS] output={output_dir}")
        return 0
    except (
        FileNotFoundError,
        ValueError,
        RuntimeError,
        cv2.error,
    ) as exc:
        print(f"[MARKERLESS][ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
