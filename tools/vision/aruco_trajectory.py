#!/usr/bin/env python3
"""Extract a planar trajectory from a video using ArUco reference markers.

The default marker layout matches the Nightfall tracking board:

* ID 5: top-left reference
* ID 7: top-right reference
* ID 4: bottom-right reference
* ID 6: bottom-left reference
* ID 3: marker carried by the mouse

The four fixed markers stabilize the board on every frame.  The carried marker
is small and its black border blends into the mouse, so its known ArUco bit
pattern is tracked with rotation/scale-aware template matching inside a
continuity-gated region of interest.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


ARUCO_DICTIONARY_NAME = "DICT_4X4_50"
DEFAULT_FIXED_LAYOUT = {
    5: "top_left",
    7: "top_right",
    4: "bottom_right",
    6: "bottom_left",
}
FIXED_ORDER = (5, 7, 4, 6)
HOMOGRAPHY_MARKER_IDS = (5, 4, 6)


@dataclass
class VideoInfo:
    fps: float
    frame_count: int
    width: int
    height: int

    @property
    def duration_s(self) -> float:
        return self.frame_count / self.fps


@dataclass
class FrameCalibration:
    fixed_corners: Dict[int, np.ndarray]
    homography: np.ndarray
    fixed_ids: Tuple[int, ...]
    homography_ids: Tuple[int, ...]
    reprojection_rmse_px: float
    inlier_corner_count: int
    inlier_marker_count: int
    used_previous_homography: bool


@dataclass
class TemplateSpec:
    image: np.ndarray
    side_px: int
    angle_deg: int


@dataclass
class TemplateHit:
    score: float
    center_xy: np.ndarray
    side_px: int
    angle_deg: int


@dataclass
class TrackSample:
    frame: int
    time_s: float
    canonical_xy_raw: np.ndarray
    template_angle_unwrapped_deg: float
    marker_side_px: int
    marker_score: float
    source: str
    green_seed_xy: np.ndarray


@dataclass
class GridCalibration:
    x_lines_px: np.ndarray
    y_lines_px: np.ndarray
    x_origin_px: float
    y_origin_px: float
    x_pitch_px: float
    y_pitch_px: float
    cells: int
    x_peak_contrast: float
    y_peak_contrast: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Stabilize a marked board and extract the trajectory of a carried "
            "ArUco marker."
        )
    )
    parser.add_argument("video", type=Path, help="input video")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="output directory (default: <video stem>_trajectory beside input)",
    )
    parser.add_argument(
        "--tracked-id", type=int, default=3, help="carried ArUco marker ID"
    )
    parser.add_argument(
        "--yaw-offset-deg",
        type=float,
        default=90.0,
        help=(
            "vehicle-forward offset from the marker x axis in board/math "
            "coordinates (default: 90 deg for IMG_1592 mounting)"
        ),
    )
    parser.add_argument(
        "--canonical-size",
        type=int,
        default=900,
        help="square top-view image size in pixels",
    )
    parser.add_argument(
        "--marker-margin",
        type=float,
        default=50.0,
        help="canonical distance from image edge to fixed-marker centers",
    )
    parser.add_argument(
        "--grid-cells",
        type=int,
        default=8,
        help="number of grid cells per board side",
    )
    parser.add_argument(
        "--cell-size-mm",
        type=float,
        default=None,
        help="physical grid pitch; omit when the metric scale is unknown",
    )
    parser.add_argument(
        "--cell-size-confirmed",
        action="store_true",
        help="mark --cell-size-mm as measured/confirmed rather than assumed",
    )
    parser.add_argument(
        "--smooth-window",
        type=int,
        default=9,
        help="odd Savitzky-Golay smoothing window (default: 9 frames)",
    )
    parser.add_argument(
        "--no-videos",
        action="store_true",
        help="skip annotated MP4 generation",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not args.video.is_file():
        raise ValueError("input video does not exist: {}".format(args.video))
    if args.canonical_size < 400:
        raise ValueError("--canonical-size must be at least 400")
    if args.tracked_id < 0 or args.tracked_id >= 50:
        raise ValueError("--tracked-id must be in DICT_4X4_50 range 0..49")
    if args.tracked_id in FIXED_ORDER:
        raise ValueError("--tracked-id must differ from fixed IDs 4, 5, 6, 7")
    if not np.isfinite(args.yaw_offset_deg):
        raise ValueError("--yaw-offset-deg must be finite")
    if not 0 < args.marker_margin < args.canonical_size / 4.0:
        raise ValueError(
            "--marker-margin must be positive and less than one quarter of "
            "--canonical-size"
        )
    if args.grid_cells < 2:
        raise ValueError("--grid-cells must be at least 2")
    if args.cell_size_mm is not None and (
        not np.isfinite(args.cell_size_mm) or args.cell_size_mm <= 0
    ):
        raise ValueError("--cell-size-mm must be finite and positive")
    if args.cell_size_confirmed and args.cell_size_mm is None:
        raise ValueError("--cell-size-confirmed requires --cell-size-mm")
    if args.smooth_window < 5 or args.smooth_window % 2 == 0:
        raise ValueError("--smooth-window must be an odd integer >= 5")
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "cv2.aruco is unavailable; install opencv-contrib-python-headless"
        )


def open_video(path: Path) -> Tuple[cv2.VideoCapture, VideoInfo]:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError("failed to open video: {}".format(path))
    info = VideoInfo(
        fps=float(capture.get(cv2.CAP_PROP_FPS)),
        frame_count=int(capture.get(cv2.CAP_PROP_FRAME_COUNT)),
        width=int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
        height=int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
    )
    if info.fps <= 0 or info.frame_count <= 0:
        capture.release()
        raise RuntimeError("video reports invalid FPS or frame count")
    return capture, info


def make_detector(relaxed: bool = False) -> cv2.aruco.ArucoDetector:
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.cornerRefinementWinSize = 5
    parameters.cornerRefinementMaxIterations = 50
    parameters.minMarkerPerimeterRate = 0.01
    parameters.maxMarkerPerimeterRate = 0.25
    if relaxed:
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 53
        parameters.adaptiveThreshWinSizeStep = 4
        parameters.minMarkerPerimeterRate = 0.005
        parameters.maxMarkerPerimeterRate = 0.5
        parameters.polygonalApproxAccuracyRate = 0.06
        parameters.minCornerDistanceRate = 0.02
        parameters.minDistanceToBorder = 1
        parameters.errorCorrectionRate = 0.8
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def detect_markers(
    gray: np.ndarray, detector: cv2.aruco.ArucoDetector
) -> Dict[int, np.ndarray]:
    corners, ids, _ = detector.detectMarkers(gray)
    result: Dict[int, np.ndarray] = {}
    if ids is None:
        return result
    for marker_corners, marker_id in zip(corners, ids.ravel()):
        result[int(marker_id)] = marker_corners.reshape(4, 2).astype(np.float32)
    return result


def marker_shape_is_valid(corners: np.ndarray) -> bool:
    polygon = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    edges = np.linalg.norm(np.roll(polygon, -1, axis=0) - polygon, axis=1)
    if np.min(edges) <= 2.0 or np.max(edges) / np.min(edges) > 2.0:
        return False
    return bool(
        cv2.isContourConvex(polygon.astype(np.int32))
        and abs(cv2.contourArea(polygon)) >= 20.0
    )


def enhanced_fixed_detection(
    gray: np.ndarray,
    current: Dict[int, np.ndarray],
    detector: cv2.aruco.ArucoDetector,
) -> Dict[int, np.ndarray]:
    """Recover fixed markers only when the normal pass found fewer than three."""
    if all(marker_id in current for marker_id in HOMOGRAPHY_MARKER_IDS):
        return current
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enlarged = cv2.resize(
        clahe.apply(gray), None, fx=2.0, fy=2.0, interpolation=cv2.INTER_NEAREST
    )
    extra = detect_markers(enlarged, detector)
    merged = dict(current)
    for marker_id, marker_corners in extra.items():
        scaled_corners = marker_corners / 2.0
        if (
            marker_id in DEFAULT_FIXED_LAYOUT
            and marker_id not in merged
            and marker_shape_is_valid(scaled_corners)
        ):
            merged[marker_id] = scaled_corners
    return merged


def collect_fixed_observations(
    video_path: Path,
) -> Tuple[VideoInfo, List[Dict[int, np.ndarray]], Dict[int, int]]:
    capture, info = open_video(video_path)
    detector = make_detector()
    relaxed_detector = make_detector(relaxed=True)
    observations: List[Dict[int, np.ndarray]] = []
    detection_counts: Counter = Counter()

    while True:
        ok, frame = capture.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detected = detect_markers(gray, detector)
        detected = enhanced_fixed_detection(gray, detected, relaxed_detector)
        fixed = {
            marker_id: detected[marker_id]
            for marker_id in FIXED_ORDER
            if marker_id in detected
            and marker_shape_is_valid(detected[marker_id])
        }
        for marker_id in fixed:
            detection_counts[marker_id] += 1
        observations.append(fixed)

    capture.release()
    if len(observations) != info.frame_count:
        info.frame_count = len(observations)
    if len(observations) < 5:
        raise RuntimeError(
            "the video must contain at least five readable frames"
        )
    return info, observations, dict(detection_counts)


def build_calibrations(
    observations: Sequence[Dict[int, np.ndarray]],
    canonical_size: int,
    marker_margin: float,
) -> Tuple[List[FrameCalibration], Dict[int, np.ndarray]]:
    seen_ids = {
        marker_id
        for frame_observations in observations
        for marker_id in frame_observations
    }
    missing_reference = [
        marker_id
        for marker_id in HOMOGRAPHY_MARKER_IDS
        if marker_id not in seen_ids
    ]
    if missing_reference:
        raise RuntimeError(
            "homography marker(s) never detected: {}".format(
                missing_reference
            )
        )

    far = float(canonical_size) - marker_margin
    target_centers = {
        5: (marker_margin, marker_margin),
        7: (far, marker_margin),
        4: (far, far),
        6: (marker_margin, far),
    }
    target_marker_side = canonical_size * 0.044
    half_side = target_marker_side / 2.0
    corner_offsets = np.float32(
        [
            [-half_side, -half_side],
            [half_side, -half_side],
            [half_side, half_side],
            [-half_side, half_side],
        ]
    )
    target_corners = {
        marker_id: np.float32(target_centers[marker_id]) + corner_offsets
        for marker_id in FIXED_ORDER
    }

    calibrations: List[FrameCalibration] = []
    previous_homography: Optional[np.ndarray] = None
    consecutive_fallbacks = 0
    for frame_index, frame_observations in enumerate(observations):
        source_points: List[np.ndarray] = []
        destination_points: List[np.ndarray] = []
        point_marker_ids: List[int] = []
        fixed_ids = tuple(
            marker_id
            for marker_id in FIXED_ORDER
            if marker_id in frame_observations
        )
        homography_ids = tuple(
            marker_id
            for marker_id in HOMOGRAPHY_MARKER_IDS
            if marker_id in frame_observations
        )
        for marker_id in homography_ids:
            source_points.extend(frame_observations[marker_id])
            destination_points.extend(target_corners[marker_id])
            point_marker_ids.extend([marker_id] * 4)

        homography: Optional[np.ndarray] = None
        rmse = float("nan")
        inlier_corner_count = 0
        inlier_marker_count = 0
        used_previous_homography = False
        if len(homography_ids) == len(HOMOGRAPHY_MARKER_IDS):
            source_array = np.float32(source_points)
            destination_array = np.float32(destination_points)
            candidate_homography, inlier_mask = cv2.findHomography(
                source_array,
                destination_array,
                method=cv2.RANSAC,
                ransacReprojThreshold=3.0,
            )
            if candidate_homography is not None and inlier_mask is not None:
                inliers = inlier_mask.ravel().astype(bool)
                inlier_corner_count = int(np.sum(inliers))
                inlier_marker_count = len(
                    {
                        marker_id
                        for marker_id, is_inlier in zip(
                            point_marker_ids, inliers
                        )
                        if is_inlier
                    }
                )
            if (
                candidate_homography is not None
                and inlier_corner_count >= 8
                and inlier_marker_count >= 3
            ):
                homography = candidate_homography
                projected = cv2.perspectiveTransform(
                    source_array[np.newaxis], homography
                )[0]
                residuals = np.linalg.norm(projected - destination_array, axis=1)
                residuals = residuals[inliers]
                rmse = float(np.sqrt(np.mean(np.square(residuals))))

        if homography is None:
            if previous_homography is None:
                raise RuntimeError(
                    "cannot estimate board transform at frame {}".format(
                        frame_index
                    )
                )
            homography = previous_homography.copy()
            used_previous_homography = True
            inlier_corner_count = 0
            inlier_marker_count = 0
            consecutive_fallbacks += 1
            if consecutive_fallbacks > 5:
                raise RuntimeError(
                    "board transform lacks three-marker support for more "
                    "than five consecutive frames at frame {}".format(
                        frame_index
                    )
                )
        else:
            consecutive_fallbacks = 0
        previous_homography = homography
        calibrations.append(
            FrameCalibration(
                fixed_corners=frame_observations,
                homography=homography,
                fixed_ids=fixed_ids,
                homography_ids=homography_ids,
                reprojection_rmse_px=rmse,
                inlier_corner_count=inlier_corner_count,
                inlier_marker_count=inlier_marker_count,
                used_previous_homography=used_previous_homography,
            )
        )

    return calibrations, target_corners


def green_seed(
    warped_bgr: np.ndarray, border: int
) -> Optional[np.ndarray]:
    """Locate the saturated green PCB used only as a marker-search prior."""
    hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array([25, 85, 25], dtype=np.uint8),
        np.array([105, 255, 255], dtype=np.uint8),
    )
    mask[:border, :] = 0
    mask[-border:, :] = 0
    mask[:, :border] = 0
    mask[:, -border:] = 0
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21)),
    )
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        return None
    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[component, cv2.CC_STAT_AREA] < 200:
        return None
    return np.asarray(centroids[component], dtype=float)


def build_template_cache(
    marker_id: int,
    canonical_size: int,
    angle_step_deg: int = 2,
) -> Dict[Tuple[int, int], TemplateSpec]:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    raw_marker = cv2.aruco.generateImageMarker(
        dictionary, marker_id, 120, borderBits=1
    )
    minimum_side = make_even(int(round(canonical_size * 0.040)))
    maximum_side = make_even(int(round(canonical_size * 0.059)))
    cache: Dict[Tuple[int, int], TemplateSpec] = {}

    for side_px in range(minimum_side, maximum_side + 1, 2):
        base = cv2.resize(
            raw_marker, (side_px, side_px), interpolation=cv2.INTER_AREA
        )
        pad = side_px
        canvas = np.zeros((side_px * 3, side_px * 3), dtype=np.uint8)
        square_mask = np.zeros_like(canvas)
        canvas[pad : pad + side_px, pad : pad + side_px] = base
        square_mask[pad : pad + side_px, pad : pad + side_px] = 255
        center = ((canvas.shape[1] - 1) / 2.0, (canvas.shape[0] - 1) / 2.0)

        for angle_deg in range(0, 360, angle_step_deg):
            rotation = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
            rotated = cv2.warpAffine(
                canvas,
                rotation,
                (canvas.shape[1], canvas.shape[0]),
                flags=cv2.INTER_LINEAR,
                borderValue=0,
            )
            rotated_mask = cv2.warpAffine(
                square_mask,
                rotation,
                (canvas.shape[1], canvas.shape[0]),
                flags=cv2.INTER_NEAREST,
                borderValue=0,
            )
            x, y, width, height = cv2.boundingRect(rotated_mask)
            template = rotated[y : y + height, x : x + width]
            cache[(side_px, angle_deg)] = TemplateSpec(
                image=template,
                side_px=side_px,
                angle_deg=angle_deg,
            )
    return cache


def make_even(value: int) -> int:
    return value if value % 2 == 0 else value + 1


def subpixel_peak(
    response: np.ndarray, integer_location: Tuple[int, int]
) -> Tuple[float, float]:
    x, y = integer_location
    dx = 0.0
    dy = 0.0
    if 0 < x < response.shape[1] - 1:
        left = float(response[y, x - 1])
        center = float(response[y, x])
        right = float(response[y, x + 1])
        denominator = left - 2.0 * center + right
        if abs(denominator) > 1e-8:
            dx = float(np.clip(0.5 * (left - right) / denominator, -0.5, 0.5))
    if 0 < y < response.shape[0] - 1:
        upper = float(response[y - 1, x])
        center = float(response[y, x])
        lower = float(response[y + 1, x])
        denominator = upper - 2.0 * center + lower
        if abs(denominator) > 1e-8:
            dy = float(np.clip(0.5 * (upper - lower) / denominator, -0.5, 0.5))
    return dx, dy


def search_templates(
    gray: np.ndarray,
    cache: Dict[Tuple[int, int], TemplateSpec],
    center_xy: np.ndarray,
    radius: float,
    angles_deg: Iterable[int],
    sides_px: Iterable[int],
    continuity_prediction: Optional[np.ndarray],
) -> Optional[TemplateHit]:
    height, width = gray.shape
    x0 = max(0, int(math.floor(center_xy[0] - radius)))
    y0 = max(0, int(math.floor(center_xy[1] - radius)))
    x1 = min(width, int(math.ceil(center_xy[0] + radius)))
    y1 = min(height, int(math.ceil(center_xy[1] + radius)))
    roi = gray[y0:y1, x0:x1]
    if roi.size == 0:
        return None

    best_adjusted_score = -2.0
    best_hit: Optional[TemplateHit] = None
    for side_px in sides_px:
        for angle_deg in angles_deg:
            key = (int(side_px), int(angle_deg) % 360)
            spec = cache.get(key)
            if spec is None:
                continue
            template_height, template_width = spec.image.shape
            if template_height > roi.shape[0] or template_width > roi.shape[1]:
                continue
            response = cv2.matchTemplate(
                roi, spec.image, cv2.TM_CCOEFF_NORMED
            )
            _, score, _, integer_location = cv2.minMaxLoc(response)
            dx, dy = subpixel_peak(response, integer_location)
            center = np.array(
                [
                    x0
                    + integer_location[0]
                    + dx
                    + (template_width - 1) / 2.0,
                    y0
                    + integer_location[1]
                    + dy
                    + (template_height - 1) / 2.0,
                ],
                dtype=float,
            )
            adjusted_score = float(score)
            if continuity_prediction is not None:
                adjusted_score -= 0.00025 * float(
                    np.linalg.norm(center - continuity_prediction)
                )
            if adjusted_score > best_adjusted_score:
                best_adjusted_score = adjusted_score
                best_hit = TemplateHit(
                    score=float(score),
                    center_xy=center,
                    side_px=spec.side_px,
                    angle_deg=spec.angle_deg,
                )
    return best_hit


def unwrap_angle(angle_deg: float, previous_unwrapped_deg: float) -> float:
    return angle_deg + 360.0 * round(
        (previous_unwrapped_deg - angle_deg) / 360.0
    )


def track_carried_marker(
    video_path: Path,
    info: VideoInfo,
    calibrations: Sequence[FrameCalibration],
    marker_id: int,
    canonical_size: int,
    marker_margin: float,
) -> Tuple[List[TrackSample], np.ndarray]:
    cache = build_template_cache(marker_id, canonical_size)
    available_sides = sorted({key[0] for key in cache})
    angle_step = 2
    capture, _ = open_video(video_path)
    samples: List[TrackSample] = []
    previous_center: Optional[np.ndarray] = None
    velocity = np.zeros(2, dtype=float)
    previous_unwrapped_angle: Optional[float] = None
    angular_velocity = 0.0
    previous_side = available_sides[len(available_sides) // 2]
    reference_warp: Optional[np.ndarray] = None

    for frame_index, calibration in enumerate(calibrations):
        ok, frame = capture.read()
        if not ok:
            break
        warped = cv2.warpPerspective(
            frame,
            calibration.homography,
            (canonical_size, canonical_size),
        )
        if reference_warp is None:
            reference_warp = warped.copy()
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        seed: Optional[np.ndarray] = None

        if previous_center is None:
            seed = green_seed(warped, int(round(marker_margin + 15)))
            if seed is None:
                capture.release()
                raise RuntimeError(
                    "green PCB search prior is not visible on the first frame"
                )
            hit = search_templates(
                gray,
                cache,
                seed,
                radius=canonical_size * 0.162,
                angles_deg=range(0, 360, angle_step),
                sides_px=available_sides,
                continuity_prediction=None,
            )
            source = "aruco_template_global"
        else:
            prediction = previous_center + velocity
            predicted_angle = int(
                round((previous_unwrapped_angle + angular_velocity) / angle_step)
                * angle_step
            )
            local_angles = [
                (predicted_angle + offset) % 360
                for offset in range(-16, 17, angle_step)
            ]
            local_sides = [
                side
                for side in available_sides
                if abs(side - previous_side) <= 4
            ]
            hit = search_templates(
                gray,
                cache,
                prediction,
                radius=canonical_size * 0.083,
                angles_deg=local_angles,
                sides_px=local_sides,
                continuity_prediction=prediction,
            )
            source = "aruco_template_local"
            needs_relocalization = hit is None or hit.score < 0.48
            if needs_relocalization:
                seed = green_seed(warped, int(round(marker_margin + 15)))
                if seed is not None:
                    broad_hit = search_templates(
                        gray,
                        cache,
                        seed,
                        radius=canonical_size * 0.15,
                        angles_deg=range(0, 360, 4),
                        sides_px=available_sides,
                        continuity_prediction=prediction,
                    )
                    if broad_hit is not None and (
                        hit is None or broad_hit.score > hit.score
                    ):
                        hit = broad_hit
                        source = "aruco_template_relocalized"

        if hit is None:
            capture.release()
            raise RuntimeError(
                "carried marker cannot be located at frame {}".format(frame_index)
            )
        if hit.score < 0.45:
            capture.release()
            raise RuntimeError(
                "carried marker match is too weak at frame {}: {:.3f}".format(
                    frame_index, hit.score
                )
            )

        if previous_center is not None:
            displacement = hit.center_xy - previous_center
            velocity = 0.65 * velocity + 0.35 * np.clip(
                displacement, -25.0, 25.0
            )
            unwrapped_angle = unwrap_angle(
                float(hit.angle_deg), float(previous_unwrapped_angle)
            )
            angle_delta = unwrapped_angle - float(previous_unwrapped_angle)
            angular_velocity = 0.6 * angular_velocity + 0.4 * float(
                np.clip(angle_delta, -20.0, 20.0)
            )
        else:
            unwrapped_angle = float(hit.angle_deg)

        samples.append(
            TrackSample(
                frame=frame_index,
                time_s=frame_index / info.fps,
                canonical_xy_raw=hit.center_xy,
                template_angle_unwrapped_deg=unwrapped_angle,
                marker_side_px=hit.side_px,
                marker_score=hit.score,
                source=source,
                green_seed_xy=(
                    seed
                    if seed is not None
                    else np.array([float("nan"), float("nan")])
                ),
            )
        )
        previous_center = hit.center_xy
        previous_unwrapped_angle = unwrapped_angle
        previous_side = hit.side_px

    capture.release()
    if len(samples) != len(calibrations):
        raise RuntimeError(
            "tracking stopped after {} of {} frames".format(
                len(samples), len(calibrations)
            )
        )
    if reference_warp is None:
        raise RuntimeError("failed to create a stabilized reference frame")
    return samples, reference_warp


def nonmaximum_line_peaks(
    score: np.ndarray,
    count: int,
    lower: int,
    upper: int,
    minimum_distance: int,
) -> np.ndarray:
    smoothed = np.convolve(score, np.ones(3) / 3.0, mode="same")
    working = smoothed.copy()
    peaks: List[int] = []
    for _ in range(count):
        if upper <= lower:
            raise RuntimeError("invalid grid search range")
        peak = lower + int(np.argmax(working[lower:upper]))
        peaks.append(peak)
        left = max(0, peak - minimum_distance)
        right = min(len(working), peak + minimum_distance + 1)
        working[left:right] = -1.0
    return np.asarray(sorted(peaks), dtype=float)


def fit_uniform_grid(peaks: np.ndarray, axis_name: str) -> Tuple[float, float]:
    indices = np.arange(len(peaks), dtype=float)
    pitch, origin = np.polyfit(indices, peaks, 1)
    fitted = origin + pitch * indices
    residual = np.max(np.abs(peaks - fitted))
    if pitch <= 0 or residual > 0.20 * pitch:
        raise RuntimeError(
            "{} grid-line fit is inconsistent: pitch={:.2f}px, "
            "max residual={:.2f}px".format(axis_name, pitch, residual)
        )
    return float(origin), float(pitch)


def peak_contrast(
    score: np.ndarray, peaks: np.ndarray, lower: int, upper: int
) -> float:
    smoothed = np.convolve(score, np.ones(3) / 3.0, mode="same")
    peak_values = [
        float(smoothed[int(round(peak))]) for peak in peaks
    ]
    baseline = float(np.median(smoothed[lower:upper]))
    return float(np.median(peak_values) - baseline)


def detect_grid(
    reference_warp: np.ndarray,
    grid_cells: int,
    marker_margin: float,
) -> GridCalibration:
    gray = cv2.cvtColor(reference_warp, cv2.COLOR_BGR2GRAY)
    size = gray.shape[0]
    lower = int(round(marker_margin + 5))
    upper = int(round(size - marker_margin + 5))
    strip = slice(lower, min(size, upper))
    threshold = max(135, int(np.percentile(gray[strip, strip], 82)))
    vertical_score = np.mean(gray[strip, :] > threshold, axis=0)
    horizontal_score = np.mean(gray[:, strip] > threshold, axis=1)
    expected_pitch = (size - 2.0 * marker_margin) / (grid_cells + 0.5)
    minimum_distance = max(12, int(round(expected_pitch * 0.38)))
    line_count = grid_cells + 1
    x_peaks = nonmaximum_line_peaks(
        vertical_score, line_count, lower, upper, minimum_distance
    )
    y_peaks = nonmaximum_line_peaks(
        horizontal_score, line_count, lower, upper, minimum_distance
    )
    x_origin, x_pitch = fit_uniform_grid(x_peaks, "vertical")
    y_origin, y_pitch = fit_uniform_grid(y_peaks, "horizontal")
    x_contrast = peak_contrast(vertical_score, x_peaks, lower, upper)
    y_contrast = peak_contrast(horizontal_score, y_peaks, lower, upper)
    minimum_pitch = expected_pitch * 0.75
    maximum_pitch = expected_pitch * 1.25
    if not minimum_pitch <= x_pitch <= maximum_pitch:
        raise RuntimeError(
            "vertical grid pitch {:.2f}px is outside expected range "
            "{:.2f}..{:.2f}px".format(
                x_pitch, minimum_pitch, maximum_pitch
            )
        )
    if not minimum_pitch <= y_pitch <= maximum_pitch:
        raise RuntimeError(
            "horizontal grid pitch {:.2f}px is outside expected range "
            "{:.2f}..{:.2f}px".format(
                y_pitch, minimum_pitch, maximum_pitch
            )
        )
    if x_contrast < 0.12 or y_contrast < 0.12:
        raise RuntimeError(
            "grid-line contrast is too low: x={:.3f}, y={:.3f}"
            .format(x_contrast, y_contrast)
        )
    x_lines = x_origin + x_pitch * np.arange(line_count)
    y_lines = y_origin + y_pitch * np.arange(line_count)
    return GridCalibration(
        x_lines_px=x_lines,
        y_lines_px=y_lines,
        x_origin_px=x_origin,
        y_origin_px=y_origin,
        x_pitch_px=x_pitch,
        y_pitch_px=y_pitch,
        cells=grid_cells,
        x_peak_contrast=x_contrast,
        y_peak_contrast=y_contrast,
    )


def savitzky_golay(
    values: np.ndarray, window: int, polynomial_order: int = 3
) -> np.ndarray:
    if len(values) < window:
        return values.copy()
    half = window // 2
    offsets = np.arange(-half, half + 1, dtype=float)
    design = np.vander(offsets, N=polynomial_order + 1, increasing=True)
    coefficients = np.linalg.pinv(design)[0]
    padded = np.pad(values, (half, half), mode="reflect")
    return np.convolve(padded, coefficients[::-1], mode="valid")


def convert_track(
    samples: Sequence[TrackSample],
    grid: GridCalibration,
    fps: float,
    smooth_window: int,
    cell_size_mm: Optional[float],
    yaw_offset_deg: float,
) -> Dict[str, np.ndarray]:
    raw_xy = np.asarray(
        [sample.canonical_xy_raw for sample in samples], dtype=float
    )
    raw_template_angle = np.asarray(
        [sample.template_angle_unwrapped_deg for sample in samples], dtype=float
    )
    smooth_x = savitzky_golay(raw_xy[:, 0], smooth_window)
    smooth_y = savitzky_golay(raw_xy[:, 1], smooth_window)
    smooth_template_angle = savitzky_golay(
        raw_template_angle, smooth_window
    )

    bottom_y = grid.y_origin_px + grid.cells * grid.y_pitch_px
    x_cell_raw = (raw_xy[:, 0] - grid.x_origin_px) / grid.x_pitch_px
    y_cell_raw = (bottom_y - raw_xy[:, 1]) / grid.y_pitch_px
    x_cell = (smooth_x - grid.x_origin_px) / grid.x_pitch_px
    y_cell = (bottom_y - smooth_y) / grid.y_pitch_px
    heading_unwrapped = smooth_template_angle + yaw_offset_deg
    heading_wrapped = np.mod(heading_unwrapped, 360.0)
    time_s = np.asarray([sample.time_s for sample in samples], dtype=float)

    vx_cell_s = np.gradient(x_cell, 1.0 / fps)
    vy_cell_s = np.gradient(y_cell, 1.0 / fps)
    speed_cell_s = np.hypot(vx_cell_s, vy_cell_s)
    result = {
        "time_s": time_s,
        "canonical_x_px_raw": raw_xy[:, 0],
        "canonical_y_px_raw": raw_xy[:, 1],
        "canonical_x_px": smooth_x,
        "canonical_y_px": smooth_y,
        "x_cell_raw": x_cell_raw,
        "y_cell_raw": y_cell_raw,
        "x_cell": x_cell,
        "y_cell": y_cell,
        "yaw_deg_raw_unwrapped": raw_template_angle + yaw_offset_deg,
        "yaw_deg_unwrapped": heading_unwrapped,
        "yaw_deg": heading_wrapped,
        "vx_cell_s": vx_cell_s,
        "vy_cell_s": vy_cell_s,
        "speed_cell_s": speed_cell_s,
    }
    if cell_size_mm is not None:
        result.update(
            {
                "x_mm": x_cell * cell_size_mm,
                "y_mm": y_cell * cell_size_mm,
                "vx_mm_s": vx_cell_s * cell_size_mm,
                "vy_mm_s": vy_cell_s * cell_size_mm,
                "speed_mm_s": speed_cell_s * cell_size_mm,
            }
        )
    return result


def write_csv(
    path: Path,
    samples: Sequence[TrackSample],
    converted: Dict[str, np.ndarray],
    calibrations: Sequence[FrameCalibration],
    cell_size_mm: Optional[float],
) -> None:
    fieldnames = [
        "frame",
        "time_s",
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
        "yaw_deg_raw_unwrapped",
        "yaw_deg_unwrapped",
        "yaw_deg",
        "vx_cell_s",
        "vy_cell_s",
        "speed_cell_s",
        "vx_mm_s",
        "vy_mm_s",
        "speed_mm_s",
        "marker_score",
        "marker_side_px",
        "tracking_source",
        "fixed_ids",
        "fixed_marker_count",
        "homography_ids",
        "homography_rmse_px",
        "homography_inlier_corner_count",
        "homography_inlier_marker_count",
        "homography_used_previous",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for index, (sample, calibration) in enumerate(
            zip(samples, calibrations)
        ):
            row = {
                "frame": sample.frame,
                "time_s": format_float(sample.time_s, 6),
                "canonical_x_px_raw": format_float(
                    converted["canonical_x_px_raw"][index], 4
                ),
                "canonical_y_px_raw": format_float(
                    converted["canonical_y_px_raw"][index], 4
                ),
                "canonical_x_px": format_float(
                    converted["canonical_x_px"][index], 4
                ),
                "canonical_y_px": format_float(
                    converted["canonical_y_px"][index], 4
                ),
                "x_cell_raw": format_float(
                    converted["x_cell_raw"][index], 6
                ),
                "y_cell_raw": format_float(
                    converted["y_cell_raw"][index], 6
                ),
                "x_cell": format_float(converted["x_cell"][index], 6),
                "y_cell": format_float(converted["y_cell"][index], 6),
                "x_mm": "",
                "y_mm": "",
                "yaw_deg_raw_unwrapped": format_float(
                    converted["yaw_deg_raw_unwrapped"][index], 4
                ),
                "yaw_deg_unwrapped": format_float(
                    converted["yaw_deg_unwrapped"][index], 4
                ),
                "yaw_deg": format_float(converted["yaw_deg"][index], 4),
                "vx_cell_s": format_float(converted["vx_cell_s"][index], 6),
                "vy_cell_s": format_float(converted["vy_cell_s"][index], 6),
                "speed_cell_s": format_float(
                    converted["speed_cell_s"][index], 6
                ),
                "vx_mm_s": "",
                "vy_mm_s": "",
                "speed_mm_s": "",
                "marker_score": format_float(sample.marker_score, 6),
                "marker_side_px": sample.marker_side_px,
                "tracking_source": sample.source,
                "fixed_ids": "|".join(
                    str(marker_id) for marker_id in calibration.fixed_ids
                ),
                "fixed_marker_count": len(calibration.fixed_ids),
                "homography_ids": "|".join(
                    str(marker_id)
                    for marker_id in calibration.homography_ids
                ),
                "homography_rmse_px": format_float(
                    calibration.reprojection_rmse_px, 6
                ),
                "homography_inlier_corner_count": (
                    calibration.inlier_corner_count
                ),
                "homography_inlier_marker_count": (
                    calibration.inlier_marker_count
                ),
                "homography_used_previous": int(
                    calibration.used_previous_homography
                ),
            }
            if cell_size_mm is not None:
                for name in (
                    "x_mm",
                    "y_mm",
                    "vx_mm_s",
                    "vy_mm_s",
                    "speed_mm_s",
                ):
                    row[name] = format_float(converted[name][index], 4)
            writer.writerow(row)


def format_float(value: float, digits: int) -> str:
    if not np.isfinite(value):
        return ""
    return ("{:.%df}" % digits).format(float(value))


def color_for_fraction(fraction: float) -> Tuple[int, int, int]:
    hue = int(round(120.0 * (1.0 - float(np.clip(fraction, 0.0, 1.0)))))
    hsv = np.uint8([[[hue, 230, 245]]])
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


def render_plot(
    path: Path,
    converted: Dict[str, np.ndarray],
    grid: GridCalibration,
    cell_size_mm: Optional[float],
    metric_scale_assumed: bool,
    tracked_id: int,
) -> None:
    width = 1120
    height = 980
    plot_left = 105
    plot_top = 90
    plot_size = 800
    image = np.full((height, width, 3), 250, dtype=np.uint8)
    cv2.putText(
        image,
        "ArUco trajectory (ID {})".format(tracked_id),
        (plot_left, 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.05,
        (25, 25, 25),
        2,
        cv2.LINE_AA,
    )

    for cell in range(grid.cells + 1):
        x = int(round(plot_left + plot_size * cell / grid.cells))
        y = int(round(plot_top + plot_size * (grid.cells - cell) / grid.cells))
        cv2.line(
            image,
            (x, plot_top),
            (x, plot_top + plot_size),
            (190, 190, 190),
            1,
            cv2.LINE_AA,
        )
        cv2.line(
            image,
            (plot_left, y),
            (plot_left + plot_size, y),
            (190, 190, 190),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            str(cell),
            (x - 8, plot_top + plot_size + 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (60, 60, 60),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            str(cell),
            (plot_left - 36, y + 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (60, 60, 60),
            1,
            cv2.LINE_AA,
        )
    cv2.rectangle(
        image,
        (plot_left, plot_top),
        (plot_left + plot_size, plot_top + plot_size),
        (55, 55, 55),
        2,
    )

    x_cell = converted["x_cell"]
    y_cell = converted["y_cell"]
    points = np.column_stack(
        [
            plot_left + plot_size * x_cell / grid.cells,
            plot_top + plot_size * (grid.cells - y_cell) / grid.cells,
        ]
    )
    total_segments = max(1, len(points) - 1)
    for index in range(1, len(points)):
        color = color_for_fraction(index / total_segments)
        cv2.line(
            image,
            tuple(np.round(points[index - 1]).astype(int)),
            tuple(np.round(points[index]).astype(int)),
            color,
            4,
            cv2.LINE_AA,
        )
    start = tuple(np.round(points[0]).astype(int))
    end = tuple(np.round(points[-1]).astype(int))
    cv2.circle(image, start, 11, (40, 175, 40), -1, cv2.LINE_AA)
    cv2.circle(image, start, 11, (20, 90, 20), 2, cv2.LINE_AA)
    cv2.circle(image, end, 11, (210, 70, 200), -1, cv2.LINE_AA)
    cv2.circle(image, end, 11, (100, 20, 90), 2, cv2.LINE_AA)
    cv2.putText(
        image,
        "start",
        (start[0] + 14, start[1] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (20, 90, 20),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        "end",
        (end[0] + 14, end[1] - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (100, 20, 90),
        1,
        cv2.LINE_AA,
    )

    cv2.putText(
        image,
        "x [grid cells]",
        (plot_left + plot_size // 2 - 65, height - 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (40, 40, 40),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        "y [grid cells]",
        (10, plot_top - 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (40, 40, 40),
        1,
        cv2.LINE_AA,
    )

    bar_x = 965
    for row in range(plot_size):
        fraction = 1.0 - row / max(1, plot_size - 1)
        cv2.line(
            image,
            (bar_x, plot_top + row),
            (bar_x + 24, plot_top + row),
            color_for_fraction(fraction),
            1,
        )
    duration = float(converted["time_s"][-1])
    cv2.putText(
        image,
        "{:.1f}s".format(duration),
        (bar_x + 34, plot_top + 7),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (45, 45, 45),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        "0.0s",
        (bar_x + 34, plot_top + plot_size),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (45, 45, 45),
        1,
        cv2.LINE_AA,
    )
    if cell_size_mm is not None:
        scale_text = "metric scale: {:.3g} mm/cell ({})".format(
            cell_size_mm, "assumed" if metric_scale_assumed else "confirmed"
        )
        cv2.putText(
            image,
            scale_text,
            (plot_left + 480, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (70, 70, 70),
            1,
            cv2.LINE_AA,
        )
    if not cv2.imwrite(str(path), image):
        raise RuntimeError("failed to write trajectory plot: {}".format(path))


def open_writer(
    requested_path: Path, fps: float, size: Tuple[int, int]
) -> Tuple[cv2.VideoWriter, Path]:
    writer = cv2.VideoWriter(
        str(requested_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        size,
    )
    if writer.isOpened():
        return writer, requested_path
    writer.release()
    fallback = requested_path.with_suffix(".avi")
    writer = cv2.VideoWriter(
        str(fallback),
        cv2.VideoWriter_fourcc(*"MJPG"),
        fps,
        size,
    )
    if not writer.isOpened():
        raise RuntimeError("failed to open MP4 and AVI video writers")
    return writer, fallback


def transform_points(points: np.ndarray, homography: np.ndarray) -> np.ndarray:
    return cv2.perspectiveTransform(
        np.asarray(points, dtype=np.float32)[np.newaxis], homography
    )[0]


def draw_track_history(
    image: np.ndarray,
    points: np.ndarray,
    upto: int,
    transform: Optional[np.ndarray] = None,
) -> None:
    history = points[: upto + 1]
    if transform is not None:
        history = transform_points(history, transform)
    history_int = np.round(history).astype(np.int32)
    if len(history_int) >= 2:
        cv2.polylines(
            image, [history_int], False, (0, 220, 255), 3, cv2.LINE_AA
        )


def render_videos(
    video_path: Path,
    output_dir: Path,
    info: VideoInfo,
    calibrations: Sequence[FrameCalibration],
    samples: Sequence[TrackSample],
    converted: Dict[str, np.ndarray],
    grid: GridCalibration,
    canonical_size: int,
) -> Dict[str, str]:
    top_path_requested = output_dir / "trajectory_topview.mp4"
    overlay_path_requested = output_dir / "trajectory_overlay.mp4"
    top_writer, top_path = open_writer(
        top_path_requested, info.fps, (canonical_size, canonical_size)
    )
    overlay_writer, overlay_path = open_writer(
        overlay_path_requested, info.fps, (info.width, info.height)
    )
    capture, _ = open_video(video_path)
    canonical_points = np.column_stack(
        [converted["canonical_x_px"], converted["canonical_y_px"]]
    ).astype(np.float32)

    for index, (sample, calibration) in enumerate(
        zip(samples, calibrations)
    ):
        ok, original = capture.read()
        if not ok:
            break
        top = cv2.warpPerspective(
            original,
            calibration.homography,
            (canonical_size, canonical_size),
        )
        for x in grid.x_lines_px:
            cv2.line(
                top,
                (int(round(x)), int(round(grid.y_lines_px[0]))),
                (int(round(x)), int(round(grid.y_lines_px[-1]))),
                (100, 100, 255),
                1,
                cv2.LINE_AA,
            )
        for y in grid.y_lines_px:
            cv2.line(
                top,
                (int(round(grid.x_lines_px[0])), int(round(y))),
                (int(round(grid.x_lines_px[-1])), int(round(y))),
                (100, 100, 255),
                1,
                cv2.LINE_AA,
            )
        draw_track_history(top, canonical_points, index)
        current = canonical_points[index]
        heading_rad = math.radians(float(converted["yaw_deg_unwrapped"][index]))
        arrow_end = current + np.array(
            [45.0 * math.cos(heading_rad), -45.0 * math.sin(heading_rad)]
        )
        current_int = tuple(np.round(current).astype(int))
        cv2.circle(top, current_int, 7, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.arrowedLine(
            top,
            current_int,
            tuple(np.round(arrow_end).astype(int)),
            (0, 255, 255),
            4,
            cv2.LINE_AA,
            tipLength=0.25,
        )
        status = (
            "t={:.2f}s  cell=({:.3f}, {:.3f})  yaw={:.1f}deg  score={:.3f}"
        ).format(
            sample.time_s,
            converted["x_cell"][index],
            converted["y_cell"][index],
            converted["yaw_deg"][index],
            sample.marker_score,
        )
        draw_text_box(top, status, (12, 28))
        top_writer.write(top)

        inverse = np.linalg.inv(calibration.homography)
        draw_track_history(
            original, canonical_points, index, transform=inverse
        )
        original_current = transform_points(
            np.float32([current, arrow_end]), inverse
        )
        original_current_int = tuple(
            np.round(original_current[0]).astype(int)
        )
        cv2.circle(
            original, original_current_int, 7, (0, 0, 255), -1, cv2.LINE_AA
        )
        cv2.arrowedLine(
            original,
            original_current_int,
            tuple(np.round(original_current[1]).astype(int)),
            (0, 255, 255),
            4,
            cv2.LINE_AA,
            tipLength=0.25,
        )
        for marker_id, corners in calibration.fixed_corners.items():
            polygon = np.round(corners).astype(np.int32)
            cv2.polylines(
                original, [polygon], True, (255, 180, 0), 2, cv2.LINE_AA
            )
            label_at = tuple(polygon[0])
            cv2.putText(
                original,
                "ID{}".format(marker_id),
                (int(label_at[0]), int(label_at[1]) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 180, 0),
                1,
                cv2.LINE_AA,
            )
        draw_text_box(original, status, (12, 28))
        overlay_writer.write(original)

    capture.release()
    top_writer.release()
    overlay_writer.release()
    return {
        "topview_video": top_path.name,
        "overlay_video": overlay_path.name,
    }


def draw_text_box(
    image: np.ndarray, text: str, origin: Tuple[int, int]
) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.58
    thickness = 1
    (text_width, text_height), baseline = cv2.getTextSize(
        text, font, scale, thickness
    )
    x, y = origin
    cv2.rectangle(
        image,
        (x - 6, y - text_height - 6),
        (x + text_width + 6, y + baseline + 5),
        (20, 20, 20),
        -1,
    )
    cv2.putText(
        image,
        text,
        (x, y),
        font,
        scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA,
    )


def trajectory_statistics(
    converted: Dict[str, np.ndarray],
    cell_size_mm: Optional[float],
) -> Dict[str, object]:
    x = converted["x_cell"]
    y = converted["y_cell"]
    time_s = converted["time_s"]
    steps = np.hypot(np.diff(x), np.diff(y))
    fps = 1.0 / float(np.median(np.diff(time_s)))
    motion_half_window = max(1, int(round(fps / 3.0)))
    windowed_speed = np.zeros(len(x), dtype=float)
    for index in range(len(x)):
        left = max(0, index - motion_half_window)
        right = min(len(x) - 1, index + motion_half_window)
        elapsed = time_s[right] - time_s[left]
        if elapsed > 0:
            windowed_speed[index] = math.hypot(
                x[right] - x[left], y[right] - y[left]
            ) / elapsed
    active_mask = windowed_speed >= 0.30
    active_first, active_last = longest_true_run(active_mask)
    if active_first is None or active_last is None:
        active_steps = np.asarray([], dtype=float)
        active_start_s: Optional[float] = None
        active_end_s: Optional[float] = None
    else:
        active_steps = steps[active_first:active_last]
        active_start_s = float(time_s[active_first])
        active_end_s = float(time_s[active_last])
    result: Dict[str, object] = {
        "start_cell": [float(x[0]), float(y[0])],
        "end_cell": [float(x[-1]), float(y[-1])],
        "bounds_cell": {
            "x_min": float(np.min(x)),
            "x_max": float(np.max(x)),
            "y_min": float(np.min(y)),
            "y_max": float(np.max(y)),
        },
        "path_length_cells_all_frames": float(np.sum(steps)),
        "path_length_cells_active": float(np.sum(active_steps)),
        "recommended_distance_metric": "path_length_cells_active",
        "active_motion": {
            "method": "longest run above centered-displacement speed threshold",
            "window_frames": 2 * motion_half_window + 1,
            "threshold_cell_s": 0.30,
            "first_frame": active_first,
            "last_frame": active_last,
            "start_s": active_start_s,
            "end_s": active_end_s,
        },
    }
    if cell_size_mm is not None:
        result["start_mm"] = [
            float(converted["x_mm"][0]),
            float(converted["y_mm"][0]),
        ]
        result["end_mm"] = [
            float(converted["x_mm"][-1]),
            float(converted["y_mm"][-1]),
        ]
        result["path_length_mm_all_frames"] = float(
            np.sum(steps) * cell_size_mm
        )
        result["path_length_mm_active"] = float(
            np.sum(active_steps) * cell_size_mm
        )
        result["peak_speed_mm_s"] = float(
            np.max(windowed_speed) * cell_size_mm
        )
        result["peak_speed_mm_s_method"] = (
            "centered displacement over {} frames".format(
                2 * motion_half_window + 1
            )
        )
        result["peak_speed_mm_s_frame_gradient"] = float(
            np.max(converted["speed_mm_s"])
        )
        result["recommended_distance_metric"] = "path_length_mm_active"
    return result


def longest_true_run(
    mask: np.ndarray,
) -> Tuple[Optional[int], Optional[int]]:
    best_start: Optional[int] = None
    best_end: Optional[int] = None
    best_length = 0
    current_start: Optional[int] = None
    for index, enabled in enumerate(mask):
        if enabled and current_start is None:
            current_start = index
        is_last = index == len(mask) - 1
        if current_start is not None and (not enabled or is_last):
            current_end = index if enabled and is_last else index - 1
            current_length = current_end - current_start + 1
            if current_length > best_length:
                best_start = current_start
                best_end = current_end
                best_length = current_length
            current_start = None
    return best_start, best_end


def percentile_summary(values: Sequence[float]) -> Dict[str, float]:
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if not len(array):
        return {}
    return {
        "min": float(np.min(array)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: object) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(
            value,
            handle,
            ensure_ascii=False,
            indent=2,
            allow_nan=False,
        )
        handle.write("\n")


def verify_outputs(
    csv_path: Path,
    image_path: Path,
    reference_path: Path,
    calibration_path: Path,
    report_path: Path,
    video_paths: Iterable[Path],
    expected_frames: int,
) -> None:
    if not csv_path.is_file() or csv_path.stat().st_size == 0:
        raise RuntimeError("CSV output is missing or empty")
    with csv_path.open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    row_count = len(rows)
    if row_count != expected_frames:
        raise RuntimeError(
            "CSV has {} rows, expected {}".format(row_count, expected_frames)
        )
    for column in ("time_s", "x_cell", "y_cell", "yaw_deg", "marker_score"):
        values = np.asarray([float(row[column]) for row in rows], dtype=float)
        if not np.all(np.isfinite(values)):
            raise RuntimeError("CSV column {} contains non-finite values".format(column))
    image = cv2.imread(str(image_path))
    if image is None or image.size == 0:
        raise RuntimeError("trajectory image cannot be decoded")
    reference = cv2.imread(str(reference_path))
    if reference is None or reference.size == 0:
        raise RuntimeError("reference top view cannot be decoded")
    for json_path in (calibration_path, report_path):
        try:
            with json_path.open(encoding="utf-8") as handle:
                document = json.load(handle)
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(
                "JSON output cannot be parsed: {}: {}".format(json_path, error)
            )
        if not isinstance(document, dict) or "schema" not in document:
            raise RuntimeError(
                "JSON output lacks a schema object: {}".format(json_path)
            )
    for video_path in video_paths:
        capture = cv2.VideoCapture(str(video_path))
        if not capture.isOpened():
            raise RuntimeError("video output cannot be opened: {}".format(video_path))
        frames = 0
        while True:
            ok, decoded = capture.read()
            if not ok:
                break
            if decoded is None or decoded.size == 0:
                capture.release()
                raise RuntimeError(
                    "video contains an empty decoded frame: {}".format(video_path)
                )
            frames += 1
        capture.release()
        if frames != expected_frames:
            raise RuntimeError(
                "{} has {} frames, expected {}".format(
                    video_path.name, frames, expected_frames
                )
            )


def main() -> int:
    args = parse_args()
    validate_args(args)
    video_path = args.video.resolve()
    output_dir = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else video_path.with_name(video_path.stem + "_trajectory")
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    print("[1/6] Detecting fixed ArUco markers", flush=True)
    info, observations, fixed_counts = collect_fixed_observations(video_path)
    calibrations, target_corners = build_calibrations(
        observations, args.canonical_size, args.marker_margin
    )
    minimum_fixed = min(len(item.fixed_ids) for item in calibrations)
    if minimum_fixed < 2:
        raise RuntimeError(
            "at least two fixed markers are required on every frame; minimum={}"
            .format(minimum_fixed)
        )
    fallback_homography_frames = sum(
        item.used_previous_homography for item in calibrations
    )
    allowed_fallback_frames = max(1, int(math.floor(0.05 * len(calibrations))))
    if fallback_homography_frames > allowed_fallback_frames:
        raise RuntimeError(
            "{} of {} frames lack a three-marker-supported board transform; "
            "maximum allowed is {}".format(
                fallback_homography_frames,
                len(calibrations),
                allowed_fallback_frames,
            )
        )
    maximum_consecutive_fallbacks = 0
    current_fallbacks = 0
    for calibration in calibrations:
        if calibration.used_previous_homography:
            current_fallbacks += 1
            maximum_consecutive_fallbacks = max(
                maximum_consecutive_fallbacks, current_fallbacks
            )
        else:
            current_fallbacks = 0
    supported_calibrations = [
        item for item in calibrations if not item.used_previous_homography
    ]
    minimum_inlier_markers = min(
        item.inlier_marker_count for item in supported_calibrations
    )

    print("[2/6] Tracking carried ArUco ID {}".format(args.tracked_id), flush=True)
    samples, reference_warp = track_carried_marker(
        video_path,
        info,
        calibrations,
        args.tracked_id,
        args.canonical_size,
        args.marker_margin,
    )
    marker_scores = np.asarray(
        [sample.marker_score for sample in samples], dtype=float
    )

    print("[3/6] Calibrating the board grid", flush=True)
    grid = detect_grid(reference_warp, args.grid_cells, args.marker_margin)
    converted = convert_track(
        samples,
        grid,
        info.fps,
        args.smooth_window,
        args.cell_size_mm,
        args.yaw_offset_deg,
    )
    metric_scale_assumed = (
        args.cell_size_mm is not None and not args.cell_size_confirmed
    )

    csv_path = output_dir / "trajectory.csv"
    plot_path = output_dir / "trajectory.png"
    calibration_path = output_dir / "calibration.json"
    report_path = output_dir / "qa_report.json"
    reference_path = output_dir / "reference_topview.png"

    print("[4/6] Writing CSV, calibration, and plot", flush=True)
    write_csv(
        csv_path,
        samples,
        converted,
        calibrations,
        args.cell_size_mm,
    )
    render_plot(
        plot_path,
        converted,
        grid,
        args.cell_size_mm,
        metric_scale_assumed,
        args.tracked_id,
    )
    if not cv2.imwrite(str(reference_path), reference_warp):
        raise RuntimeError(
            "failed to write reference top view: {}".format(reference_path)
        )
    calibration_document = {
        "schema": "nightfall_aruco_trajectory_calibration_v1",
        "aruco_dictionary": ARUCO_DICTIONARY_NAME,
        "tracked_marker_id": args.tracked_id,
        "fixed_marker_layout": {
            str(marker_id): position
            for marker_id, position in DEFAULT_FIXED_LAYOUT.items()
        },
        "homography_marker_ids": list(HOMOGRAPHY_MARKER_IDS),
        "canonical": {
            "size_px": args.canonical_size,
            "fixed_marker_center_margin_px": args.marker_margin,
            "fixed_marker_target_side_px": args.canonical_size * 0.044,
            "fixed_marker_target_corners_px": {
                str(marker_id): np.asarray(corners).tolist()
                for marker_id, corners in target_corners.items()
            },
        },
        "grid": {
            "cells": grid.cells,
            "x_lines_px": grid.x_lines_px.tolist(),
            "y_lines_px": grid.y_lines_px.tolist(),
            "x_origin_px": grid.x_origin_px,
            "y_origin_px": grid.y_origin_px,
            "x_pitch_px": grid.x_pitch_px,
            "y_pitch_px": grid.y_pitch_px,
            "x_peak_contrast": grid.x_peak_contrast,
            "y_peak_contrast": grid.y_peak_contrast,
            "coordinate_system": (
                "origin at grid bottom-left; +x right; +y up; yaw 0deg right "
                "and positive counter-clockwise"
            ),
        },
        "metric_scale": {
            "cell_size_mm": args.cell_size_mm,
            "is_assumed": metric_scale_assumed,
        },
        "marker_extrinsics": {
            "vehicle_yaw_offset_deg": args.yaw_offset_deg,
            "tracked_position": "marker center (not geometric vehicle center)",
        },
    }
    write_json(calibration_path, calibration_document)

    video_outputs: Dict[str, str] = {}
    if not args.no_videos:
        print("[5/6] Rendering annotated videos", flush=True)
        video_outputs = render_videos(
            video_path,
            output_dir,
            info,
            calibrations,
            samples,
            converted,
            grid,
            args.canonical_size,
        )
    else:
        print("[5/6] Annotated videos skipped", flush=True)

    report = {
        "schema": "nightfall_aruco_trajectory_qa_v1",
        "input": {
            "path": str(video_path),
            "sha256": sha256_file(video_path),
            "size_bytes": video_path.stat().st_size,
            "width": info.width,
            "height": info.height,
            "fps": info.fps,
            "frames": info.frame_count,
            "duration_s": info.duration_s,
        },
        "fixed_markers": {
            "detection_count": {
                str(marker_id): fixed_counts.get(marker_id, 0)
                for marker_id in FIXED_ORDER
            },
            "minimum_markers_per_frame": minimum_fixed,
            "homography_marker_ids": list(HOMOGRAPHY_MARKER_IDS),
            "minimum_inlier_markers_per_fitted_frame": minimum_inlier_markers,
            "previous_homography_fallback_frames": (
                fallback_homography_frames
            ),
            "maximum_consecutive_fallback_frames": (
                maximum_consecutive_fallbacks
            ),
            "homography_reprojection_rmse_px": percentile_summary(
                [item.reprojection_rmse_px for item in calibrations]
            ),
        },
        "carried_marker": {
            "id": args.tracked_id,
            "tracked_frames": len(samples),
            "missing_frames": info.frame_count - len(samples),
            "template_score": percentile_summary(marker_scores),
            "frames_below_score_0_6": int(np.sum(marker_scores < 0.6)),
            "source_count": dict(Counter(sample.source for sample in samples)),
            "maximum_raw_frame_step_px": float(
                np.max(
                    np.linalg.norm(
                        np.diff(
                            np.asarray(
                                [
                                    sample.canonical_xy_raw
                                    for sample in samples
                                ]
                            ),
                            axis=0,
                        ),
                        axis=1,
                    )
                )
            ),
        },
        "marker_extrinsics": {
            "vehicle_yaw_offset_deg": args.yaw_offset_deg,
            "tracked_position": "marker center (not geometric vehicle center)",
        },
        "smoothing": {
            "method": "Savitzky-Golay",
            "window_frames": args.smooth_window,
            "polynomial_order": 3,
        },
        "trajectory": trajectory_statistics(converted, args.cell_size_mm),
        "metric_scale": {
            "cell_size_mm": args.cell_size_mm,
            "is_assumed": metric_scale_assumed,
        },
        "outputs": {
            "trajectory_csv": csv_path.name,
            "trajectory_plot": plot_path.name,
            "calibration": calibration_path.name,
            "reference_topview": reference_path.name,
            **video_outputs,
        },
    }
    write_json(report_path, report)

    print("[6/6] Verifying outputs", flush=True)
    verify_outputs(
        csv_path,
        plot_path,
        reference_path,
        calibration_path,
        report_path,
        [output_dir / name for name in video_outputs.values()],
        info.frame_count,
    )
    print(
        "Done: {} frames, marker score min/median={:.3f}/{:.3f}, output={}"
        .format(
            info.frame_count,
            float(np.min(marker_scores)),
            float(np.median(marker_scores)),
            output_dir,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, ValueError) as error:
        print("error: {}".format(error), file=sys.stderr)
        sys.exit(2)
