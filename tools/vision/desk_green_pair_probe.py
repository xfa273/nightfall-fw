#!/usr/bin/env python3
"""Probe markerless micromouse tracking from a pair of green PCB regions.

This diagnostic intentionally works in raw image pixels and does not replace
the fixed-board-marker metric pipeline.  It is useful before the maze fixture
is ready: two separated green PCB regions give a robust center and an
undirected body axis while the mouse is moved by hand on a desk.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import cv2
import numpy as np

import aruco_trajectory as aruco
import markerless_trajectory as markerless


@dataclass
class GreenComponent:
    area: int
    center_xy: np.ndarray


@dataclass
class PairDetection:
    frame: int
    pts_s: float
    valid: bool
    center_xy: np.ndarray
    yaw_unwrapped_deg: float
    separation_px: float
    first_area: int
    second_area: int
    first_xy: np.ndarray
    second_xy: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Track two green PCB regions in raw image pixels for a "
            "markerless desk feasibility test."
        )
    )
    parser.add_argument("video", type=Path)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--minimum-component-pixels", type=int, default=500)
    parser.add_argument("--maximum-component-pixels", type=int, default=10000)
    parser.add_argument("--minimum-pair-distance-px", type=float, default=85.0)
    parser.add_argument("--maximum-pair-distance-px", type=float, default=210.0)
    parser.add_argument("--initial-pair-distance-px", type=float, default=155.0)
    parser.add_argument("--maximum-tracking-distance-px", type=float, default=70.0)
    parser.add_argument(
        "--minimum-y-fraction",
        type=float,
        default=0.25,
        help="ignore green components above this fraction of image height",
    )
    parser.add_argument("--initial-x-px", type=float, default=None)
    parser.add_argument("--initial-y-px", type=float, default=None)
    parser.add_argument("--initial-yaw-deg", type=float, default=0.0)
    parser.add_argument("--smooth-window", type=int, default=17)
    parser.add_argument("--maximum-missing-fraction", type=float, default=0.01)
    parser.add_argument(
        "--render-fps",
        type=float,
        default=60.0,
        help="annotated diagnostic video FPS; 0 skips video",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not args.video.is_file():
        raise ValueError(f"video does not exist: {args.video}")
    if args.minimum_component_pixels <= 0:
        raise ValueError("--minimum-component-pixels must be positive")
    if args.maximum_component_pixels < args.minimum_component_pixels:
        raise ValueError(
            "--maximum-component-pixels must be >= minimum"
        )
    if not (
        0 < args.minimum_pair_distance_px
        < args.maximum_pair_distance_px
    ):
        raise ValueError("invalid pair-distance range")
    if not (
        args.minimum_pair_distance_px
        <= args.initial_pair_distance_px
        <= args.maximum_pair_distance_px
    ):
        raise ValueError("--initial-pair-distance-px is outside range")
    if args.maximum_tracking_distance_px <= 0:
        raise ValueError("--maximum-tracking-distance-px must be positive")
    if not 0 <= args.minimum_y_fraction < 1:
        raise ValueError("--minimum-y-fraction must be in [0, 1)")
    if (args.initial_x_px is None) != (args.initial_y_px is None):
        raise ValueError("--initial-x-px and --initial-y-px are a pair")
    if args.smooth_window < 3 or args.smooth_window % 2 == 0:
        raise ValueError("--smooth-window must be odd and >= 3")
    if not 0 <= args.maximum_missing_fraction < 1:
        raise ValueError("--maximum-missing-fraction must be in [0, 1)")
    if args.render_fps < 0:
        raise ValueError("--render-fps must be non-negative")


def components(
    frame: np.ndarray,
    args: argparse.Namespace,
) -> list[GreenComponent]:
    mask = markerless.green_mask(frame)
    count, _, stats, centers = cv2.connectedComponentsWithStats(
        mask,
        connectivity=8,
    )
    minimum_y = frame.shape[0] * args.minimum_y_fraction
    output: list[GreenComponent] = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        center = np.asarray(centers[label], dtype=float)
        if not (
            args.minimum_component_pixels
            <= area
            <= args.maximum_component_pixels
        ):
            continue
        if center[1] < minimum_y:
            continue
        output.append(GreenComponent(area=area, center_xy=center))
    return output


def choose_pair(
    candidates: Sequence[GreenComponent],
    prediction_xy: Optional[np.ndarray],
    expected_separation: float,
    args: argparse.Namespace,
) -> Optional[
    tuple[
        GreenComponent,
        GreenComponent,
        np.ndarray,
        float,
    ]
]:
    scored: list[
        tuple[
            float,
            GreenComponent,
            GreenComponent,
            np.ndarray,
            float,
        ]
    ] = []
    for first_index, first in enumerate(candidates):
        for second in candidates[first_index + 1 :]:
            vector = second.center_xy - first.center_xy
            separation = float(np.linalg.norm(vector))
            if not (
                args.minimum_pair_distance_px
                <= separation
                <= args.maximum_pair_distance_px
            ):
                continue
            center = (first.center_xy + second.center_xy) * 0.5
            combined_area = first.area + second.area
            separation_error = abs(separation - expected_separation)
            if prediction_xy is None:
                if args.initial_x_px is None:
                    score = (
                        -float(combined_area)
                        + 2.0 * separation_error
                    )
                else:
                    initial = np.asarray(
                        [args.initial_x_px, args.initial_y_px],
                        dtype=float,
                    )
                    score = (
                        25.0 * float(np.linalg.norm(center - initial))
                        + 4.0 * separation_error
                        - 0.02 * combined_area
                    )
            else:
                tracking_distance = float(
                    np.linalg.norm(center - prediction_xy)
                )
                if tracking_distance > args.maximum_tracking_distance_px:
                    continue
                score = (
                    25.0 * tracking_distance
                    + 4.0 * separation_error
                    - 0.02 * combined_area
                )
            scored.append(
                (
                    score,
                    first,
                    second,
                    center,
                    separation,
                )
            )
    if not scored:
        return None
    _, first, second, center, separation = min(
        scored,
        key=lambda item: item[0],
    )
    return first, second, center, separation


def track(
    video: Path,
    args: argparse.Namespace,
) -> tuple[
    aruco.VideoInfo,
    np.ndarray,
    str,
    list[PairDetection],
]:
    capture, info = aruco.open_video(video)
    timestamps, timestamp_source = markerless.video_timestamps(
        video,
        info.frame_count,
    )
    detections: list[PairDetection] = []
    prior_center: Optional[np.ndarray] = None
    velocity = np.zeros(2, dtype=float)
    separation_history: list[float] = []
    prior_yaw: Optional[float] = args.initial_yaw_deg
    frame_index = 0
    nan_xy = np.full(2, np.nan, dtype=float)
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        prediction = (
            None
            if prior_center is None
            else prior_center + velocity
        )
        expected_separation = (
            float(np.median(separation_history[-240:]))
            if separation_history
            else args.initial_pair_distance_px
        )
        selected = choose_pair(
            components(frame, args),
            prediction,
            expected_separation,
            args,
        )
        if selected is None:
            detections.append(
                PairDetection(
                    frame=frame_index,
                    pts_s=float(timestamps[frame_index]),
                    valid=False,
                    center_xy=nan_xy.copy(),
                    yaw_unwrapped_deg=float("nan"),
                    separation_px=float("nan"),
                    first_area=0,
                    second_area=0,
                    first_xy=nan_xy.copy(),
                    second_xy=nan_xy.copy(),
                )
            )
        else:
            first, second, center, separation = selected
            if prior_center is not None:
                displacement = center - prior_center
                velocity = (
                    0.8 * velocity
                    + 0.2 * np.clip(displacement, -20.0, 20.0)
                )
            prior_center = center
            separation_history.append(separation)
            vector = second.center_xy - first.center_xy
            raw_angle = math.degrees(
                math.atan2(-vector[1], vector[0])
            )
            yaw = markerless._nearest_periodic(
                raw_angle,
                prior_yaw,
                180.0,
            )
            prior_yaw = yaw
            detections.append(
                PairDetection(
                    frame=frame_index,
                    pts_s=float(timestamps[frame_index]),
                    valid=True,
                    center_xy=center,
                    yaw_unwrapped_deg=yaw,
                    separation_px=separation,
                    first_area=first.area,
                    second_area=second.area,
                    first_xy=first.center_xy,
                    second_xy=second.center_xy,
                )
            )
        frame_index += 1
    capture.release()
    if frame_index != info.frame_count:
        raise RuntimeError(
            f"decoded {frame_index} frames, expected {info.frame_count}"
        )
    return info, timestamps, timestamp_source, detections


def interpolate_and_smooth(
    detections: Sequence[PairDetection],
    smooth_window: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    valid = np.asarray([item.valid for item in detections], dtype=bool)
    good = np.flatnonzero(valid)
    if len(good) < 2:
        raise RuntimeError("fewer than two valid green-pair detections")
    indexes = np.arange(len(detections))
    raw_xy = np.asarray(
        [item.center_xy for item in detections],
        dtype=float,
    )
    raw_yaw = np.asarray(
        [item.yaw_unwrapped_deg for item in detections],
        dtype=float,
    )
    filled_x = np.interp(indexes, good, raw_xy[good, 0])
    filled_y = np.interp(indexes, good, raw_xy[good, 1])
    filled_yaw = np.interp(indexes, good, raw_yaw[good])
    smooth_xy = np.column_stack(
        [
            aruco.savitzky_golay(filled_x, smooth_window),
            aruco.savitzky_golay(filled_y, smooth_window),
        ]
    )
    smooth_yaw = aruco.savitzky_golay(
        filled_yaw,
        smooth_window,
    )
    return smooth_xy, smooth_yaw, valid


def write_csv(
    path: Path,
    detections: Sequence[PairDetection],
    smooth_xy: np.ndarray,
    smooth_yaw: np.ndarray,
) -> None:
    fields = (
        "frame",
        "video_pts_s",
        "video_pts_ns",
        "valid",
        "center_x_px_raw",
        "center_y_px_raw",
        "center_x_px",
        "center_y_px",
        "yaw_deg_raw_unwrapped",
        "yaw_deg_unwrapped",
        "pair_separation_px",
        "first_component_pixels",
        "second_component_pixels",
        "first_x_px",
        "first_y_px",
        "second_x_px",
        "second_y_px",
    )
    with path.open("w", encoding="ascii", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, item in enumerate(detections):
            writer.writerow(
                {
                    "frame": item.frame,
                    "video_pts_s": f"{item.pts_s:.9f}",
                    "video_pts_ns": int(
                        round(item.pts_s * 1_000_000_000)
                    ),
                    "valid": int(item.valid),
                    "center_x_px_raw": finite(item.center_xy[0]),
                    "center_y_px_raw": finite(item.center_xy[1]),
                    "center_x_px": f"{smooth_xy[index, 0]:.6f}",
                    "center_y_px": f"{smooth_xy[index, 1]:.6f}",
                    "yaw_deg_raw_unwrapped": finite(
                        item.yaw_unwrapped_deg
                    ),
                    "yaw_deg_unwrapped": f"{smooth_yaw[index]:.6f}",
                    "pair_separation_px": finite(
                        item.separation_px
                    ),
                    "first_component_pixels": item.first_area,
                    "second_component_pixels": item.second_area,
                    "first_x_px": finite(item.first_xy[0]),
                    "first_y_px": finite(item.first_xy[1]),
                    "second_x_px": finite(item.second_xy[0]),
                    "second_y_px": finite(item.second_xy[1]),
                }
            )


def finite(value: float) -> str:
    return f"{float(value):.6f}" if math.isfinite(float(value)) else ""


def path_colour(fraction: float) -> tuple[int, int, int]:
    pixel = np.uint8(
        [[[int(round(179.0 * max(0.0, min(1.0, fraction)))), 230, 255]]]
    )
    bgr = cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0]
    return tuple(int(value) for value in bgr)


def render_path_image(
    path: Path,
    video: Path,
    smooth_xy: np.ndarray,
    smooth_yaw: np.ndarray,
) -> None:
    capture = cv2.VideoCapture(str(video))
    ok, image = capture.read()
    capture.release()
    if not ok:
        raise RuntimeError("failed to read reference frame")
    points = np.rint(smooth_xy).astype(int)
    for index in range(1, len(points)):
        cv2.line(
            image,
            tuple(points[index - 1]),
            tuple(points[index]),
            path_colour(index / max(1, len(points) - 1)),
            3,
            cv2.LINE_AA,
        )
    arrow_step = max(1, len(points) // 16)
    for index in range(0, len(points), arrow_step):
        yaw = math.radians(smooth_yaw[index])
        tip = points[index] + np.rint(
            [45.0 * math.cos(yaw), -45.0 * math.sin(yaw)]
        ).astype(int)
        cv2.arrowedLine(
            image,
            tuple(points[index]),
            tuple(tip),
            (0, 255, 255),
            2,
            cv2.LINE_AA,
            tipLength=0.28,
        )
    cv2.putText(
        image,
        "image-pixel feasibility only: no board scale/homography",
        (24, 44),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"failed to write {path}")


def render_video(
    path: Path,
    source: Path,
    info: aruco.VideoInfo,
    detections: Sequence[PairDetection],
    smooth_xy: np.ndarray,
    smooth_yaw: np.ndarray,
    requested_fps: float,
) -> None:
    stride = max(1, int(round(info.fps / requested_fps)))
    output_fps = info.fps / stride
    capture = cv2.VideoCapture(str(source))
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        output_fps,
        (info.width, info.height),
    )
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(f"failed to create {path}")
    frame_index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame_index % stride != 0:
            frame_index += 1
            continue
        item = detections[frame_index]
        center = np.rint(smooth_xy[frame_index]).astype(int)
        if item.valid:
            first = tuple(np.rint(item.first_xy).astype(int))
            second = tuple(np.rint(item.second_xy).astype(int))
            cv2.line(frame, first, second, (0, 255, 0), 4)
            cv2.circle(frame, first, 9, (255, 255, 0), -1)
            cv2.circle(frame, second, 9, (0, 255, 255), -1)
        cv2.circle(frame, tuple(center), 10, (0, 0, 255), -1)
        yaw = math.radians(smooth_yaw[frame_index])
        tip = center + np.rint(
            [70.0 * math.cos(yaw), -70.0 * math.sin(yaw)]
        ).astype(int)
        cv2.arrowedLine(
            frame,
            tuple(center),
            tuple(tip),
            (0, 255, 255),
            4,
            cv2.LINE_AA,
            tipLength=0.25,
        )
        cv2.putText(
            frame,
            (
                f"frame {frame_index}  "
                f"PTS {item.pts_s:.3f}s  valid={int(item.valid)}"
            ),
            (24, 44),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        writer.write(frame)
        frame_index += 1
    capture.release()
    writer.release()


def statistics(values: Sequence[float]) -> dict[str, float]:
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


def longest_false_run(valid: np.ndarray) -> int:
    longest = 0
    current = 0
    for value in valid:
        if value:
            current = 0
        else:
            current += 1
            longest = max(longest, current)
    return longest


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        output_dir = (
            args.output_dir
            if args.output_dir is not None
            else args.video.with_name(args.video.stem + "_desk_probe")
        )
        output_dir.mkdir(parents=True, exist_ok=True)
        info, timestamps, timestamp_source, detections = track(
            args.video,
            args,
        )
        smooth_xy, smooth_yaw, valid = interpolate_and_smooth(
            detections,
            args.smooth_window,
        )
        missing_fraction = 1.0 - float(np.mean(valid))
        qa_passed = missing_fraction <= args.maximum_missing_fraction
        csv_path = output_dir / "trajectory_image_px.csv"
        image_path = output_dir / "trajectory_image_px.png"
        write_csv(csv_path, detections, smooth_xy, smooth_yaw)
        render_path_image(
            image_path,
            args.video,
            smooth_xy,
            smooth_yaw,
        )
        video_path: Optional[Path] = None
        if args.render_fps > 0:
            video_path = output_dir / "trajectory_image_px.mp4"
            render_video(
                video_path,
                args.video,
                info,
                detections,
                smooth_xy,
                smooth_yaw,
                args.render_fps,
            )
        separations = [
            item.separation_px for item in detections if item.valid
        ]
        areas = [
            float(area)
            for item in detections
            if item.valid
            for area in (item.first_area, item.second_area)
        ]
        report = {
            "schema": "nightfall_desk_green_pair_probe_v1",
            "input": {
                "path": str(args.video.resolve()),
                "sha256": markerless.sha256_file(args.video),
                "size_bytes": args.video.stat().st_size,
                "width": info.width,
                "height": info.height,
                "frames": info.frame_count,
                "container_fps": info.fps,
                "duration_pts_s": float(
                    timestamps[-1] - timestamps[0]
                ),
                "timestamp_source": timestamp_source,
            },
            "configuration": {
                key: getattr(args, key)
                for key in (
                    "minimum_component_pixels",
                    "maximum_component_pixels",
                    "minimum_pair_distance_px",
                    "maximum_pair_distance_px",
                    "initial_pair_distance_px",
                    "maximum_tracking_distance_px",
                    "minimum_y_fraction",
                    "initial_x_px",
                    "initial_y_px",
                    "initial_yaw_deg",
                    "smooth_window",
                    "maximum_missing_fraction",
                    "render_fps",
                )
            },
            "tracking": {
                "valid_frames": int(np.sum(valid)),
                "invalid_frames": int(len(valid) - np.sum(valid)),
                "missing_fraction": missing_fraction,
                "longest_missing_run_frames": longest_false_run(valid),
                "pair_separation_px": statistics(separations),
                "component_pixels": statistics(areas),
                "center_start_px": smooth_xy[0].tolist(),
                "center_end_px": smooth_xy[-1].tolist(),
                "center_range_px": np.ptp(
                    smooth_xy,
                    axis=0,
                ).tolist(),
                "yaw_start_deg": float(smooth_yaw[0]),
                "yaw_end_deg": float(smooth_yaw[-1]),
                "yaw_range_deg": [
                    float(np.min(smooth_yaw)),
                    float(np.max(smooth_yaw)),
                ],
            },
            "qa": {
                "passed": qa_passed,
                "note": (
                    "This proves raw-pixel green-pair visibility only. "
                    "It has no board homography, metric scale, absolute "
                    "heading, lens correction, or ground-truth accuracy."
                ),
            },
            "outputs": {
                "trajectory_csv": csv_path.name,
                "trajectory_image": image_path.name,
                "annotated_video": (
                    video_path.name if video_path is not None else None
                ),
            },
        }
        report_path = output_dir / "desk_green_pair_report.json"
        report_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(
            "[DESK-GREEN-PAIR] "
            f"frames={info.frame_count} valid={int(np.sum(valid))} "
            f"missing_fraction={missing_fraction:.6f} "
            f"qa_passed={int(qa_passed)}"
        )
        print(f"[DESK-GREEN-PAIR] Report: {report_path}")
        return 0 if qa_passed else 2
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        print(f"[DESK-GREEN-PAIR][ERROR] {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
