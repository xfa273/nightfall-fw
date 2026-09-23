#!/usr/bin/env python3
"""Detect a flush white cell-centre calibration lattice in a fixed-maze video."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

import aruco_trajectory as aruco
import board_layout


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Rectify a fixed-camera calibration clip with the four outer "
            "ArUco markers and locate matte white circular marks at all cell centres."
        )
    )
    parser.add_argument("video", type=Path)
    parser.add_argument("--board-layout", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--canonical-size", type=int, default=900)
    parser.add_argument("--samples", type=int, default=9)
    parser.add_argument("--dot-diameter-mm", type=float, default=8.0)
    parser.add_argument("--maximum-assignment-mm", type=float, default=38.0)
    parser.add_argument("--minimum-value", type=int, default=155)
    parser.add_argument("--maximum-saturation", type=int, default=105)
    return parser.parse_args()

def _validate(args: argparse.Namespace) -> None:
    if not args.video.is_file():
        raise ValueError(f"video does not exist: {args.video}")
    if not args.board_layout.is_file():
        raise ValueError(f"board layout does not exist: {args.board_layout}")
    if args.canonical_size < 400:
        raise ValueError("--canonical-size must be at least 400")
    if args.samples < 3 or args.samples > 31 or args.samples % 2 == 0:
        raise ValueError("--samples must be an odd integer in 3..31")
    if not math.isfinite(args.dot_diameter_mm) or not 4.0 <= args.dot_diameter_mm <= 14.0:
        raise ValueError("--dot-diameter-mm must be in 4..14")
    if not math.isfinite(args.maximum_assignment_mm) or not 5.0 <= args.maximum_assignment_mm < 40.0:
        raise ValueError("--maximum-assignment-mm must be in [5, 40)")
    if not 0 <= args.maximum_saturation <= 255:
        raise ValueError("--maximum-saturation must be in 0..255")
    if not 0 <= args.minimum_value <= 255:
        raise ValueError("--minimum-value must be in 0..255")


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
        raise RuntimeError("failed to decode all lattice sample frames")
    return np.median(np.stack(frames), axis=0).astype(np.uint8), indexes


def _candidate_centres(
    reference_bgr: np.ndarray,
    layout: board_layout.BoardLayout,
    dot_diameter_mm: float,
    minimum_value: int,
    maximum_saturation: int,
) -> tuple[list[dict[str, float]], np.ndarray]:
    hsv = cv2.cvtColor(reference_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.asarray([0, 0, minimum_value], dtype=np.uint8),
        np.asarray([179, maximum_saturation, 255], dtype=np.uint8),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )
    expected_diameter_px = dot_diameter_mm * layout.pixels_per_mm
    expected_area = math.pi * (expected_diameter_px / 2.0) ** 2
    minimum_area = max(8.0, 0.28 * expected_area)
    maximum_area = 3.2 * expected_area
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    candidates: list[dict[str, float]] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not minimum_area <= area <= maximum_area:
            continue
        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0.0:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity < 0.52:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        aspect = width / max(1.0, float(height))
        if not 0.58 <= aspect <= 1.72:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] <= 0.0:
            continue
        candidates.append(
            {
                "x": float(moments["m10"] / moments["m00"]),
                "y": float(moments["m01"] / moments["m00"]),
                "area": area,
                "circularity": circularity,
                "area_error": abs(math.log(max(area, 1e-9) / expected_area)),
            }
        )
    return candidates, mask


def _expected_lattice(
    layout: board_layout.BoardLayout,
) -> list[dict[str, float | int | str]]:
    grid = layout.raw["grid"]
    origin_x, origin_y = map(float, grid["origin_mm"])
    pitch = float(grid["pitch_mm"])
    cells = int(grid["cells"])
    bounds = layout.raw["canvas_bounds_mm"]
    x_min = float(bounds["x_min"])
    y_max = float(bounds["y_max"])
    result: list[dict[str, float | int | str]] = []
    for row in range(cells):
        for column in range(cells):
            x_mm = origin_x + (column + 0.5) * pitch
            y_mm = origin_y + (row + 0.5) * pitch
            result.append(
                {
                    "point_id": f"c{column}_r{row}",
                    "column": column,
                    "row": row,
                    "board_x_mm": x_mm,
                    "board_y_mm": y_mm,
                    "predicted_x_px": (x_mm - x_min) * layout.pixels_per_mm,
                    "predicted_y_px": (y_max - y_mm) * layout.pixels_per_mm,
                    "role": "held_out" if (column + 2 * row) % 5 == 0 else "fit",
                }
            )
    return result


def assign_candidates(
    expected: list[dict[str, float | int | str]],
    candidates: list[dict[str, float]],
    maximum_distance_px: float,
) -> list[dict[str, float | int | str]]:
    assignments: list[tuple[float, int, int]] = []
    for expected_index, point in enumerate(expected):
        predicted = np.asarray(
            [point["predicted_x_px"], point["predicted_y_px"]], dtype=float
        )
        for candidate_index, candidate in enumerate(candidates):
            distance = float(
                np.linalg.norm(
                    np.asarray([candidate["x"], candidate["y"]]) - predicted
                )
            )
            if distance <= maximum_distance_px:
                score = distance + 1.5 * candidate["area_error"] + 2.0 * (
                    1.0 - candidate["circularity"]
                )
                assignments.append((score, expected_index, candidate_index))
    used_expected: set[int] = set()
    used_candidates: set[int] = set()
    selected: list[dict[str, float | int | str]] = []
    for score, expected_index, candidate_index in sorted(assignments):
        if expected_index in used_expected or candidate_index in used_candidates:
            continue
        used_expected.add(expected_index)
        used_candidates.add(candidate_index)
        point = dict(expected[expected_index])
        candidate = candidates[candidate_index]
        point.update(
            {
                "canonical_x_px": candidate["x"],
                "canonical_y_px": candidate["y"],
                "area_px": candidate["area"],
                "circularity": candidate["circularity"],
                "assignment_score": score,
            }
        )
        selected.append(point)
    return sorted(selected, key=lambda item: (int(item["row"]), int(item["column"])))


def main() -> int:
    args = parse_args()
    try:
        _validate(args)
        layout = board_layout.load(args.board_layout, args.canonical_size)
        info, observations, counts = aruco.collect_fixed_observations(args.video)
        calibrations, _ = aruco.build_calibrations(
            observations,
            args.canonical_size,
            50.0,
            layout.target_corners_px,
        )
        reference, sampled_frames = _sample_rectified_frames(
            args.video, calibrations, args.canonical_size, args.samples
        )
        candidates, mask = _candidate_centres(
            reference,
            layout,
            args.dot_diameter_mm,
            args.minimum_value,
            args.maximum_saturation,
        )
        expected = _expected_lattice(layout)
        selected = assign_candidates(
            expected,
            candidates,
            args.maximum_assignment_mm * layout.pixels_per_mm,
        )
        fit_count = sum(item["role"] == "fit" for item in selected)
        held_out_count = sum(item["role"] == "held_out" for item in selected)
        if fit_count < 24 or held_out_count < 8 or len(selected) < 32:
            raise RuntimeError(
                "only {} lattice points were accepted (fit={}, held-out={}); "
                "need at least 32/24/8".format(len(selected), fit_count, held_out_count)
            )
        args.output_csv.parent.mkdir(parents=True, exist_ok=True)
        fields = (
            "point_id",
            "canonical_x_px",
            "canonical_y_px",
            "board_x_mm",
            "board_y_mm",
            "role",
            "area_px",
            "circularity",
            "assignment_score",
        )
        with args.output_csv.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(selected)
        overlay = reference.copy()
        for item in selected:
            center = (int(round(float(item["canonical_x_px"]))), int(round(float(item["canonical_y_px"]))))
            colour = (0, 255, 255) if item["role"] == "held_out" else (0, 255, 0)
            cv2.circle(overlay, center, 7, colour, 2, cv2.LINE_AA)
            cv2.putText(
                overlay,
                str(item["point_id"]),
                (center[0] + 5, center[1] - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.30,
                colour,
                1,
                cv2.LINE_AA,
            )
        overlay_path = args.output_csv.with_suffix(".png")
        mask_path = args.output_csv.with_name(args.output_csv.stem + "_mask.png")
        cv2.imwrite(str(overlay_path), overlay)
        cv2.imwrite(str(mask_path), mask)
        qa = {
            "schema": "nightfall_board_metric_lattice_detection_v1",
            "input_video": {
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
            },
            "sampled_frames": sampled_frames,
            "marker_detection_count": {str(key): value for key, value in counts.items()},
            "candidate_count": len(candidates),
            "accepted_count": len(selected),
            "fit_count": fit_count,
            "held_out_count": held_out_count,
            "dot_diameter_mm": args.dot_diameter_mm,
            "maximum_assignment_mm": args.maximum_assignment_mm,
            "outputs": {
                "observations_csv": str(args.output_csv.resolve()),
                "overlay": str(overlay_path.resolve()),
                "mask": str(mask_path.resolve()),
            },
        }
        args.output_csv.with_suffix(".json").write_text(
            json.dumps(qa, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(
            f"[BOARD-LATTICE] accepted={len(selected)}/64 fit={fit_count} "
            f"held_out={held_out_count} candidates={len(candidates)}"
        )
        print(f"[BOARD-LATTICE] output={args.output_csv.resolve()}")
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"[BOARD-LATTICE] ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
