#!/usr/bin/env python3
"""Fit a dense board-coordinate map from distributed known points."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

import board_metric_geometry as metric


MANIFEST_SCHEMA = "nightfall_board_metric_fit_manifest_v1"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fit canonical-pixel to board-mm geometry from a distributed "
            "fit/held-out correspondence CSV."
        )
    )
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--degree", type=int, choices=(1, 2, 3), default=3)
    return parser.parse_args()


def _resolve(base: Path, value: Any, field: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-empty path")
    path = Path(value)
    if not path.is_absolute():
        path = base / path
    path = path.resolve()
    if not path.is_file():
        raise ValueError(f"{field} does not exist: {path}")
    return path


def _finite(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{field} must be a finite number")
    return result


def _load_points(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
    observed: list[tuple[float, float]] = []
    expected: list[tuple[float, float]] = []
    validation: list[bool] = []
    names: list[str] = []
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {
            "point_id",
            "canonical_x_px",
            "canonical_y_px",
            "board_x_mm",
            "board_y_mm",
            "role",
        }
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f"{path} is missing required correspondence columns")
        for line, row in enumerate(reader, start=2):
            point_id = row.get("point_id", "").strip()
            if not point_id:
                raise ValueError(f"line {line}: point_id must not be blank")
            if point_id in names:
                raise ValueError(f"line {line}: duplicate point_id {point_id}")
            role = row.get("role", "").strip()
            if role not in ("fit", "held_out"):
                raise ValueError(f"line {line}: role must be fit or held_out")
            observed.append(
                (
                    _finite(row.get("canonical_x_px"), f"line {line} canonical_x_px"),
                    _finite(row.get("canonical_y_px"), f"line {line} canonical_y_px"),
                )
            )
            expected.append(
                (
                    _finite(row.get("board_x_mm"), f"line {line} board_x_mm"),
                    _finite(row.get("board_y_mm"), f"line {line} board_y_mm"),
                )
            )
            validation.append(role == "held_out")
            names.append(point_id)
    return (
        np.asarray(observed, dtype=float),
        np.asarray(expected, dtype=float),
        np.asarray(validation, dtype=bool),
        names,
    )


def main() -> int:
    args = parse_args()
    try:
        raw = json.loads(args.manifest.read_text(encoding="utf-8"))
        if not isinstance(raw, dict) or raw.get("schema") != MANIFEST_SCHEMA:
            raise ValueError(f"manifest schema must be {MANIFEST_SCHEMA}")
        base = args.manifest.parent.resolve()
        board_layout_path = _resolve(base, raw.get("board_layout"), "board_layout")
        points_path = _resolve(base, raw.get("observations_csv"), "observations_csv")
        canonical_size = raw.get("canonical_size_px")
        if isinstance(canonical_size, bool) or not isinstance(canonical_size, int):
            raise ValueError("canonical_size_px must be an integer")
        reference_height = _finite(
            raw.get("reference_plane_absolute_height_mm"),
            "reference_plane_absolute_height_mm",
        )
        if reference_height < 0.0:
            raise ValueError("reference plane height must be non-negative")
        reference_provenance = metric.reference_plane_provenance(
            raw.get("reference_plane_provenance")
        )
        observed, expected, validation, names = _load_points(points_path)
        geometry = metric.fit_geometry(
            observed,
            expected,
            validation,
            canonical_size_px=canonical_size,
            reference_plane_absolute_height_mm=reference_height,
            reference_plane_provenance_value=reference_provenance,
            board_layout_sha256=metric.sha256_file(board_layout_path),
            source_manifest_sha256=metric.sha256_file(args.manifest),
            degree=args.degree,
        )
        payload = metric.to_json(geometry)
        mapped = geometry.map_points(observed)
        payload["sources"] = {
            "manifest": {
                "path": str(args.manifest.resolve()),
                "sha256": metric.sha256_file(args.manifest),
            },
            "board_layout": {
                "path": str(board_layout_path),
                "sha256": metric.sha256_file(board_layout_path),
            },
            "observations_csv": {
                "path": str(points_path),
                "sha256": metric.sha256_file(points_path),
            },
        }
        records = metric.point_records(observed, expected, validation, mapped)
        for name, record in zip(names, records):
            record["point_id"] = name
        payload["evidence"]["points"] = records
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(
            "[BOARD-METRIC] points={} held_out={} degree={} fit_p95={:.3f}mm "
            "heldout_p95={:.3f}mm max={:.3f}mm qualified={}".format(
                len(observed),
                int(np.count_nonzero(validation)),
                args.degree,
                geometry.fit_stats["p95_mm"],
                geometry.validation_stats["p95_mm"],
                geometry.validation_stats["max_mm"],
                int(geometry.safety_qualified),
            )
        )
        print(f"[BOARD-METRIC] output={args.output.resolve()}")
        if not geometry.safety_qualified:
            for failure in geometry.qualification["failures"]:
                print(f"[BOARD-METRIC] REJECT {failure}")
            return 1
        return 0
    except (OSError, ValueError) as exc:
        print(f"[BOARD-METRIC] ERROR: {exc}", file=__import__("sys").stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
