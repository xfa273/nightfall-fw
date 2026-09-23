#!/usr/bin/env python3
"""Fit differential-height parallax bias from blue/red label vectors.

As the mouse turns, the observed front-minus-centre vectors should lie on a
circle.  If the labels are at different heights, floor-plane rectification
adds an approximately constant vector over a local tuning area; that appears
as a displaced circle.  This tool fits its centre and emits the bias that
``markerless_trajectory.py`` subtracts before calculating yaw.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

import board_layout


SCHEMA = "nightfall_front_label_heading_calibration_v1"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit blue-centre to red-front label parallax correction."
    )
    parser.add_argument("trajectories", nargs="+", type=Path)
    parser.add_argument("--board-layout", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--canonical-size", type=int, default=900)
    parser.add_argument("--front-label-distance-mm", type=float, default=24.0)
    parser.add_argument("--minimum-heading-span-deg", type=float, default=60.0)
    parser.add_argument("--maximum-fit-condition", type=float, default=10000.0)
    parser.add_argument("--maximum-radial-p95-mm", type=float, default=2.0)
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_vectors(path: Path) -> np.ndarray:
    with path.open(newline="", encoding="ascii") as stream:
        reader = csv.DictReader(stream)
        required = {
            "label_x_px",
            "label_y_px",
            "front_label_x_px",
            "front_label_y_px",
        }
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f"{path} is missing front-label coordinate columns")
        vectors: list[list[float]] = []
        for row in reader:
            try:
                center = np.asarray(
                    [float(row["label_x_px"]), float(row["label_y_px"])],
                    dtype=float,
                )
                front = np.asarray(
                    [
                        float(row["front_label_x_px"]),
                        float(row["front_label_y_px"]),
                    ],
                    dtype=float,
                )
            except (TypeError, ValueError):
                continue
            vector = front - center
            if np.all(np.isfinite(vector)):
                vectors.append(vector.tolist())
    if len(vectors) < 100:
        raise ValueError(f"{path} has fewer than 100 valid front-label vectors")
    return np.asarray(vectors, dtype=float)


def _circular_span_deg(vectors: np.ndarray) -> float:
    angles = np.mod(
        np.degrees(np.arctan2(-vectors[:, 1], vectors[:, 0])),
        360.0,
    )
    ordered = np.sort(angles)
    gaps = np.diff(np.concatenate([ordered, ordered[:1] + 360.0]))
    return float(360.0 - np.max(gaps))


def fit_bias(
    vectors: np.ndarray,
    pixels_per_mm: float,
) -> dict[str, object]:
    design = np.column_stack(
        [
            2.0 * vectors[:, 0],
            2.0 * vectors[:, 1],
            np.ones(len(vectors)),
        ]
    )
    squared_radius = np.sum(np.square(vectors), axis=1)
    condition = float(np.linalg.cond(design))
    parameters, _, _, _ = np.linalg.lstsq(
        design,
        squared_radius,
        rcond=None,
    )
    bias_px = parameters[:2]
    radius_squared = float(parameters[2] + np.dot(bias_px, bias_px))
    if not math.isfinite(radius_squared) or radius_squared <= 0.0:
        raise ValueError("front-label circle fit produced an invalid radius")
    radius_px = math.sqrt(radius_squared)
    corrected = vectors - bias_px
    radii_px = np.linalg.norm(corrected, axis=1)
    radial_error_mm = np.abs(radii_px - radius_px) / pixels_per_mm
    return {
        "bias_px": bias_px,
        "radius_px": radius_px,
        "condition": condition,
        "heading_span_deg": _circular_span_deg(corrected),
        "radial_error_mm": radial_error_mm,
    }


def main() -> int:
    args = parse_args()
    try:
        if args.canonical_size < 400:
            raise ValueError("--canonical-size must be at least 400")
        for name in (
            "front_label_distance_mm",
            "minimum_heading_span_deg",
            "maximum_fit_condition",
            "maximum_radial_p95_mm",
        ):
            value = float(getattr(args, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"--{name.replace('_', '-')} must be positive")
        paths = [path.resolve() for path in args.trajectories]
        if len(set(paths)) != len(paths):
            raise ValueError("trajectory paths must be unique")
        for path in paths:
            if not path.is_file():
                raise ValueError(f"trajectory does not exist: {path}")
        if not args.board_layout.is_file():
            raise ValueError(f"board layout does not exist: {args.board_layout}")
        layout = board_layout.load(args.board_layout, args.canonical_size)
        vectors_by_path = [load_vectors(path) for path in paths]
        vectors = np.vstack(vectors_by_path)
        fit = fit_bias(vectors, layout.pixels_per_mm)
        if fit["condition"] > args.maximum_fit_condition:
            raise ValueError(
                "front-label fit is ill-conditioned ({:.1f} > {:.1f})".format(
                    fit["condition"],
                    args.maximum_fit_condition,
                )
            )
        if fit["heading_span_deg"] < args.minimum_heading_span_deg:
            raise ValueError(
                "front-label heading span {:.1f} deg is below {:.1f} deg".format(
                    fit["heading_span_deg"],
                    args.minimum_heading_span_deg,
                )
            )
        radial_error = fit["radial_error_mm"]
        radial_p95_mm = float(np.percentile(radial_error, 95))
        if radial_p95_mm > args.maximum_radial_p95_mm:
            raise ValueError(
                "front-label radial fit p95 {:.3f} mm exceeds {:.3f} mm".format(
                    radial_p95_mm,
                    args.maximum_radial_p95_mm,
                )
            )
        bias_px = fit["bias_px"]
        bias_right_mm = float(bias_px[0] / layout.pixels_per_mm)
        bias_forward_mm = float(-bias_px[1] / layout.pixels_per_mm)
        bias_magnitude_mm = math.hypot(bias_right_mm, bias_forward_mm)
        if bias_magnitude_mm > 0.75 * args.front_label_distance_mm:
            raise ValueError("front-label fitted bias is implausibly large")
        apparent_radius_mm = float(fit["radius_px"] / layout.pixels_per_mm)
        if not 0.5 <= apparent_radius_mm / args.front_label_distance_mm <= 1.5:
            raise ValueError("front-label fitted apparent radius is implausible")
        report = {
            "schema": SCHEMA,
            "coordinate_system": "x_right_y_forward_mm",
            "front_label_distance_mm": args.front_label_distance_mm,
            "apparent_vector_bias_mm": {
                "right": bias_right_mm,
                "forward": bias_forward_mm,
            },
            "apparent_radius_mm": apparent_radius_mm,
            "fit": {
                "samples": int(len(vectors)),
                "trajectory_count": len(paths),
                "condition": fit["condition"],
                "heading_span_deg": fit["heading_span_deg"],
                "radial_error_mm": {
                    "median": float(np.median(radial_error)),
                    "p95": radial_p95_mm,
                    "max": float(np.max(radial_error)),
                },
            },
            "board_layout": {
                "path": str(args.board_layout.resolve()),
                "sha256": sha256_file(args.board_layout),
                "canonical_size_px": args.canonical_size,
                "pixels_per_mm": layout.pixels_per_mm,
            },
            "sources": [
                {
                    "path": str(path),
                    "sha256": sha256_file(path),
                    "samples": int(len(values)),
                }
                for path, values in zip(paths, vectors_by_path)
            ],
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(
            "[FRONT-LABEL-CAL] samples={} span={:.1f}deg "
            "bias_right={:.3f}mm bias_forward={:.3f}mm radial_p95={:.3f}mm".format(
                len(vectors),
                fit["heading_span_deg"],
                bias_right_mm,
                bias_forward_mm,
                radial_p95_mm,
            )
        )
        print(f"[FRONT-LABEL-CAL] output={args.output}")
        return 0
    except (FileNotFoundError, ValueError) as exc:
        print(f"[FRONT-LABEL-CAL][ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
