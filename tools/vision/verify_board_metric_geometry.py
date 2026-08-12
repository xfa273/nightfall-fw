#!/usr/bin/env python3
"""Verify that a new fixed-board clip still matches a qualified dense map."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

import board_metric_geometry as metric


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare a newly extracted lattice CSV with the reference points "
            "stored in a qualified board metric geometry."
        )
    )
    parser.add_argument("geometry", type=Path)
    parser.add_argument("observations_csv", type=Path)
    parser.add_argument("--output-json", type=Path, default=None)
    return parser.parse_args()


def _load_observations(path: Path) -> dict[str, tuple[float, float]]:
    result: dict[str, tuple[float, float]] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"point_id", "canonical_x_px", "canonical_y_px"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError("observation CSV is missing point/canonical columns")
        for line, row in enumerate(reader, start=2):
            point_id = row.get("point_id", "").strip()
            if not point_id or point_id in result:
                raise ValueError(f"line {line}: blank or duplicate point_id")
            try:
                point = (
                    float(row["canonical_x_px"]),
                    float(row["canonical_y_px"]),
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(f"line {line}: invalid canonical point") from exc
            if not all(math.isfinite(value) for value in point):
                raise ValueError(f"line {line}: canonical point must be finite")
            result[point_id] = point
    return result


def main() -> int:
    args = parse_args()
    try:
        geometry = metric.load_geometry(args.geometry)
        observed = _load_observations(args.observations_csv)
        expected_by_id = {
            str(item["point_id"]): item for item in geometry.reference_points
        }
        shared_ids = sorted(set(observed).intersection(expected_by_id))
        if len(shared_ids) < metric.MINIMUM_POINTS:
            raise ValueError(
                f"only {len(shared_ids)} reference points are visible; "
                f"need at least {metric.MINIMUM_POINTS}"
            )
        held_out_count = sum(
            expected_by_id[point_id]["role"] == "held_out"
            for point_id in shared_ids
        )
        if held_out_count < metric.MINIMUM_VALIDATION_POINTS:
            raise ValueError(
                f"only {held_out_count} held-out reference points are visible; "
                f"need at least {metric.MINIMUM_VALIDATION_POINTS}"
            )
        canonical = np.asarray([observed[point_id] for point_id in shared_ids])
        expected = np.asarray(
            [expected_by_id[point_id]["board_xy_mm"] for point_id in shared_ids],
            dtype=float,
        )
        mapped = geometry.map_points(canonical)
        errors = np.linalg.norm(mapped - expected, axis=1)
        stats = metric.error_stats(errors)
        passed = (
            stats["p95_mm"] <= metric.MAXIMUM_VALIDATION_P95_MM
            and stats["max_mm"] <= metric.MAXIMUM_VALIDATION_ERROR_MM
        )
        report = {
            "schema": "nightfall_board_metric_verification_v1",
            "geometry": {
                "path": str(args.geometry.resolve()),
                "sha256": metric.sha256_file(args.geometry),
            },
            "observations": {
                "path": str(args.observations_csv.resolve()),
                "sha256": metric.sha256_file(args.observations_csv),
            },
            "matched_points": len(shared_ids),
            "matched_held_out_points": held_out_count,
            "error": stats,
            "limits": {
                "p95_mm": metric.MAXIMUM_VALIDATION_P95_MM,
                "max_mm": metric.MAXIMUM_VALIDATION_ERROR_MM,
            },
            "passed": passed,
        }
        if args.output_json is not None:
            args.output_json.parent.mkdir(parents=True, exist_ok=True)
            args.output_json.write_text(
                json.dumps(report, indent=2, sort_keys=True, allow_nan=False)
                + "\n",
                encoding="utf-8",
            )
        print(
            "[BOARD-METRIC-VERIFY] matched={} held_out={} p95={:.3f}mm "
            "max={:.3f}mm pass={}".format(
                len(shared_ids),
                held_out_count,
                stats["p95_mm"],
                stats["max_mm"],
                int(passed),
            )
        )
        return 0 if passed else 1
    except (OSError, ValueError) as exc:
        print(f"[BOARD-METRIC-VERIFY] ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
