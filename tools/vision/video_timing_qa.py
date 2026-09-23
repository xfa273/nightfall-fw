#!/usr/bin/env python3
"""Inspect encoded frame PTS before using a phone video as measurement data."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from fractions import Fraction
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=("Check video PTS cadence, frame gaps, and expected real-time FPS")
    )
    parser.add_argument("video", type=Path)
    parser.add_argument(
        "--expected-fps",
        type=float,
        required=True,
        help="required real-time encoded FPS, for example 120 or 240",
    )
    parser.add_argument(
        "--fps-tolerance-percent",
        type=float,
        default=1.0,
    )
    parser.add_argument(
        "--maximum-gap-rate",
        type=float,
        default=0.001,
        help="maximum fraction of PTS intervals above 1.5x the median",
    )
    parser.add_argument(
        "--maximum-cadence-deviation-percent",
        type=float,
        default=10.0,
        help="maximum allowed absolute PTS-interval deviation from median",
    )
    parser.add_argument(
        "--maximum-cadence-deviation-rate",
        type=float,
        default=0.001,
        help="maximum fraction of intervals exceeding cadence deviation",
    )
    parser.add_argument(
        "--maximum-content-duplicate-rate",
        type=float,
        default=0.001,
        help="maximum fraction of decoded adjacent frames that are identical",
    )
    parser.add_argument(
        "--skip-content-check",
        action="store_true",
        help="skip decoded-frame duplicate diagnostics",
    )
    parser.add_argument(
        "--report-json",
        type=Path,
        default=None,
    )
    return parser.parse_args()


def _fraction(value: str) -> float | None:
    try:
        result = float(Fraction(value))
    except (ValueError, ZeroDivisionError):
        return None
    return result if np.isfinite(result) else None


def ffprobe(path: Path) -> dict[str, Any]:
    executable = shutil.which("ffprobe")
    if executable is None:
        raise RuntimeError("ffprobe is required; install FFmpeg first")
    command = [
        executable,
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        (
            "stream=codec_name,width,height,r_frame_rate,avg_frame_rate,"
            "time_base,duration,nb_frames:"
            "frame=best_effort_timestamp_time"
        ),
        "-of",
        "json",
        str(path),
    ]
    completed = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr.strip() or "ffprobe failed")
    return json.loads(completed.stdout)


def content_cadence(path: Path) -> dict[str, Any]:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"failed to decode video for content QA: {path}")
    previous: np.ndarray | None = None
    differences: list[float] = []
    decoded = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        decoded += 1
        height, width = frame.shape[:2]
        target_width = min(192, width)
        target_height = max(1, int(round(height * target_width / width)))
        gray = cv2.cvtColor(
            cv2.resize(
                frame,
                (target_width, target_height),
                interpolation=cv2.INTER_AREA,
            ),
            cv2.COLOR_BGR2GRAY,
        )
        if previous is not None:
            differences.append(
                float(np.mean(cv2.absdiff(previous, gray).astype(np.float32)))
            )
        previous = gray
    capture.release()
    if decoded < 2:
        raise RuntimeError("decoded fewer than two frames for content QA")
    values = np.asarray(differences, dtype=float)
    exact = values <= 1e-6
    near = values <= 0.02
    return {
        "decoded_frames": decoded,
        "adjacent_mean_abs_difference": {
            "min": float(np.min(values)),
            "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)),
            "max": float(np.max(values)),
        },
        "identical_adjacent_frames": int(np.sum(exact)),
        "identical_adjacent_rate": float(np.mean(exact)),
        "near_identical_adjacent_frames": int(np.sum(near)),
        "near_identical_adjacent_rate": float(np.mean(near)),
    }


def analyze(path: Path, args: argparse.Namespace) -> dict[str, Any]:
    payload = ffprobe(path)
    streams = payload.get("streams", [])
    if len(streams) != 1:
        raise RuntimeError(f"expected one video stream, found {len(streams)}")
    stream = streams[0]
    timestamps = np.asarray(
        [
            float(frame["best_effort_timestamp_time"])
            for frame in payload.get("frames", [])
            if "best_effort_timestamp_time" in frame
        ],
        dtype=float,
    )
    if len(timestamps) < 2:
        raise RuntimeError("ffprobe returned fewer than two video timestamps")
    timestamps -= timestamps[0]
    intervals = np.diff(timestamps)
    positive = intervals[intervals > 0]
    monotonic = bool(np.all(intervals > 0))
    median_interval = float(np.median(positive)) if len(positive) else float("nan")
    measured_fps = 1.0 / median_interval if median_interval > 0 else float("nan")
    gaps = intervals > 1.5 * median_interval
    duplicates = intervals <= 0
    gap_rate = float(np.mean(gaps))
    duplicate_rate = float(np.mean(duplicates))
    cadence_deviation_percent = (
        np.abs(intervals - median_interval) / median_interval * 100.0
        if median_interval > 0
        else np.full_like(intervals, float("inf"))
    )
    cadence_deviations = (
        cadence_deviation_percent > args.maximum_cadence_deviation_percent
    )
    cadence_deviation_rate = float(np.mean(cadence_deviations))
    declared_average_fps = _fraction(stream["avg_frame_rate"])
    declared_rate_fps = _fraction(stream["r_frame_rate"])
    expected_match: bool | None = None
    expected_error_percent: float | None = None
    if args.expected_fps is not None:
        expected_error_percent = (
            abs(measured_fps - args.expected_fps) / args.expected_fps * 100.0
        )
        expected_match = expected_error_percent <= args.fps_tolerance_percent
    qa_passed = (
        monotonic
        and gap_rate <= args.maximum_gap_rate
        and cadence_deviation_rate <= args.maximum_cadence_deviation_rate
        and (expected_match is not False)
    )
    retiming_warning = args.expected_fps is not None and expected_match is False
    content = None if args.skip_content_check else content_cadence(path)
    content_duplicate_rate = (
        0.0 if content is None else content["identical_adjacent_rate"]
    )
    content_frame_count_matches = (
        True if content is None else content["decoded_frames"] == len(timestamps)
    )
    content_passed = (
        content_duplicate_rate <= args.maximum_content_duplicate_rate
        and content_frame_count_matches
    )
    qa_passed = qa_passed and content_passed
    return {
        "schema": "nightfall_video_timing_qa_v3",
        "input": {
            "path": str(path.resolve()),
            "size_bytes": path.stat().st_size,
        },
        "stream": {
            "codec": stream.get("codec_name"),
            "width": int(stream.get("width", 0)),
            "height": int(stream.get("height", 0)),
            "declared_r_frame_rate_fps": declared_rate_fps,
            "declared_avg_frame_rate_fps": declared_average_fps,
            "time_base": stream.get("time_base"),
            "declared_duration_s": (
                float(stream["duration"])
                if stream.get("duration") is not None
                else None
            ),
            "declared_frames": (
                int(stream["nb_frames"])
                if stream.get("nb_frames") not in (None, "N/A")
                else None
            ),
        },
        "pts": {
            "frames": int(len(timestamps)),
            "duration_s": float(timestamps[-1] - timestamps[0]),
            "strictly_monotonic": monotonic,
            "measured_median_fps": measured_fps,
            "interval_s": {
                "min": float(np.min(intervals)),
                "median": median_interval,
                "p95": float(np.percentile(intervals, 95)),
                "max": float(np.max(intervals)),
            },
            "duplicate_or_reverse_intervals": int(np.sum(duplicates)),
            "duplicate_or_reverse_rate": duplicate_rate,
            "gap_intervals_over_1_5x_median": int(np.sum(gaps)),
            "gap_rate": gap_rate,
            "cadence_deviation_percent": {
                "p95": float(np.percentile(cadence_deviation_percent, 95)),
                "max": float(np.max(cadence_deviation_percent)),
            },
            "intervals_over_maximum_cadence_deviation": int(np.sum(cadence_deviations)),
            "cadence_deviation_rate": cadence_deviation_rate,
        },
        "expectation": {
            "expected_real_time_fps": args.expected_fps,
            "fps_tolerance_percent": args.fps_tolerance_percent,
            "fps_error_percent": expected_error_percent,
            "matches": expected_match,
        },
        "content_cadence": content,
        "qa": {
            "maximum_gap_rate": args.maximum_gap_rate,
            "maximum_cadence_deviation_percent": (
                args.maximum_cadence_deviation_percent
            ),
            "maximum_cadence_deviation_rate": (args.maximum_cadence_deviation_rate),
            "maximum_content_duplicate_rate": (args.maximum_content_duplicate_rate),
            "content_frame_count_matches_pts": content_frame_count_matches,
            "content_duplicate_gate_passed": content_passed,
            "passed": qa_passed,
            "retimed_or_wrong_fps_warning": retiming_warning,
            "note": (
                "PTS and decoded duplicates cannot prove sensor capture timing "
                "or rule out interpolated frames. "
                "A stock slow-motion file that is retimed for playback must "
                "not be treated as real time without CaptureResult metadata."
            ),
        },
    }


def main() -> int:
    args = parse_args()
    try:
        if not args.video.is_file():
            raise FileNotFoundError(args.video)
        numeric_arguments = (
            "expected_fps",
            "fps_tolerance_percent",
            "maximum_gap_rate",
            "maximum_cadence_deviation_percent",
            "maximum_cadence_deviation_rate",
            "maximum_content_duplicate_rate",
        )
        for name in numeric_arguments:
            if not np.isfinite(getattr(args, name)):
                raise ValueError(f"--{name.replace('_', '-')} must be finite")
        if args.expected_fps <= 0:
            raise ValueError("--expected-fps must be positive")
        if args.fps_tolerance_percent < 0:
            raise ValueError("--fps-tolerance-percent must be non-negative")
        if not 0 <= args.maximum_gap_rate <= 1:
            raise ValueError("--maximum-gap-rate must be in [0, 1]")
        if args.maximum_cadence_deviation_percent < 0:
            raise ValueError("--maximum-cadence-deviation-percent must be non-negative")
        if not 0 <= args.maximum_cadence_deviation_rate <= 1:
            raise ValueError("--maximum-cadence-deviation-rate must be in [0, 1]")
        if not 0 <= args.maximum_content_duplicate_rate <= 1:
            raise ValueError("--maximum-content-duplicate-rate must be in [0, 1]")
        report = analyze(args.video, args)
        if args.report_json is not None:
            args.report_json.parent.mkdir(parents=True, exist_ok=True)
            args.report_json.write_text(
                json.dumps(
                    report,
                    indent=2,
                    sort_keys=True,
                    allow_nan=False,
                )
                + "\n",
                encoding="utf-8",
            )
        pts = report["pts"]
        content = report["content_cadence"]
        content_duplicate_rate = (
            0.0 if content is None else content["identical_adjacent_rate"]
        )
        print(
            "[VIDEO-TIMING] frames={} fps={:.6f} gap_rate={:.6f} "
            "content_duplicate_rate={:.6f} monotonic={} qa_passed={}".format(
                pts["frames"],
                pts["measured_median_fps"],
                pts["gap_rate"],
                content_duplicate_rate,
                int(pts["strictly_monotonic"]),
                int(report["qa"]["passed"]),
            )
        )
        if report["qa"]["retimed_or_wrong_fps_warning"]:
            print(
                "[VIDEO-TIMING][WARN] encoded PTS does not match expected "
                "real-time FPS; treat this as retimed until proven otherwise"
            )
        return 0 if report["qa"]["passed"] else 2
    except (
        FileNotFoundError,
        ValueError,
        RuntimeError,
        json.JSONDecodeError,
    ) as exc:
        print(f"[VIDEO-TIMING][ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
