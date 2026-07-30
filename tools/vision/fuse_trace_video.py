#!/usr/bin/env python3
"""Align a video trajectory with a Nightfall firmware trace.

The primary synchronizer is motion already present in both streams:

* video yaw rate vs. ``real_omega_mdps`` for turns;
* video speed vs. ``real_velocity_mm_s`` for mostly straight runs.

The mapping is ``trace_relative_s = scale * video_pts_s + offset_s``.  For
short tuning trials scale is fixed to one; ``--estimate-drift`` enables a
small scale search for longer sessions.  Outputs contain the alignment QA and
the video pose interpolated at every firmware trace timestamp.
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

import numpy as np


@dataclass
class Series:
    time_s: np.ndarray
    signal: np.ndarray
    x_mm: Optional[np.ndarray] = None
    y_mm: Optional[np.ndarray] = None
    yaw_deg: Optional[np.ndarray] = None
    speed_mm_s: Optional[np.ndarray] = None
    omega_dps: Optional[np.ndarray] = None
    output_time_s: Optional[np.ndarray] = None


@dataclass
class Alignment:
    offset_s: float
    scale: float
    sign: int
    correlation: float
    overlap_s: float
    matched_samples: int
    signal_gain: float
    signal_bias: float
    signal_rmse: float
    normalized_rmse: float
    second_correlation: float
    correlation_margin: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Align markerless video PTS with a Nightfall trace CSV"
    )
    parser.add_argument("trajectory_csv", type=Path)
    parser.add_argument("trace_csv", type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="default: <trajectory parent>/fused_<trace stem>",
    )
    parser.add_argument(
        "--signal",
        choices=("auto", "yaw", "speed"),
        default="auto",
    )
    parser.add_argument(
        "--offset-search-s",
        type=float,
        default=1.5,
        help="search half-width around activity-onset alignment",
    )
    parser.add_argument(
        "--offset-step-ms",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--estimate-drift",
        action="store_true",
        help="estimate camera/firmware clock scale for longer sessions",
    )
    parser.add_argument("--maximum-drift-ppm", type=float, default=5000.0)
    parser.add_argument("--drift-steps", type=int, default=21)
    parser.add_argument("--activity-padding-s", type=float, default=0.30)
    parser.add_argument("--minimum-overlap-s", type=float, default=0.40)
    parser.add_argument(
        "--minimum-activity-overlap-fraction",
        type=float,
        default=0.90,
        help=("minimum overlap relative to the shorter detected activity span"),
    )
    parser.add_argument("--minimum-correlation", type=float, default=0.70)
    parser.add_argument(
        "--yaw-sign",
        choices=("same", "inverted", "auto"),
        default="same",
        help=(
            "expected video/firmware yaw-rate sign; Nightfall and the "
            "markerless coordinate system are both CCW-positive"
        ),
    )
    parser.add_argument("--minimum-signal-gain", type=float, default=0.50)
    parser.add_argument("--maximum-signal-gain", type=float, default=2.00)
    parser.add_argument(
        "--minimum-correlation-margin",
        type=float,
        default=0.02,
        help="required correlation lead over another offset at least 80 ms away",
    )
    parser.add_argument(
        "--allow-low-correlation",
        action="store_true",
        help="write diagnostic output even if an alignment QA gate fails",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    float_arguments = (
        "offset_search_s",
        "offset_step_ms",
        "maximum_drift_ppm",
        "activity_padding_s",
        "minimum_overlap_s",
        "minimum_activity_overlap_fraction",
        "minimum_correlation",
        "minimum_signal_gain",
        "maximum_signal_gain",
        "minimum_correlation_margin",
    )
    for name in float_arguments:
        if not math.isfinite(getattr(args, name)):
            raise ValueError(f"--{name.replace('_', '-')} must be finite")
    for path in (args.trajectory_csv, args.trace_csv):
        if not path.is_file():
            raise FileNotFoundError(path)
    if args.offset_search_s <= 0:
        raise ValueError("--offset-search-s must be positive")
    if args.offset_step_ms <= 0:
        raise ValueError("--offset-step-ms must be positive")
    if args.drift_steps < 1:
        raise ValueError("--drift-steps must be positive")
    if args.maximum_drift_ppm < 0:
        raise ValueError("--maximum-drift-ppm must be non-negative")
    if args.activity_padding_s < 0:
        raise ValueError("--activity-padding-s must be non-negative")
    if args.minimum_overlap_s <= 0:
        raise ValueError("--minimum-overlap-s must be positive")
    if not 0 < args.minimum_activity_overlap_fraction <= 1:
        raise ValueError("--minimum-activity-overlap-fraction must be in (0, 1]")
    if not -1.0 <= args.minimum_correlation <= 1.0:
        raise ValueError("--minimum-correlation must be in [-1, 1]")
    if args.minimum_signal_gain <= 0:
        raise ValueError("--minimum-signal-gain must be positive")
    if args.maximum_signal_gain < args.minimum_signal_gain:
        raise ValueError("--maximum-signal-gain must be >= --minimum-signal-gain")
    if not 0 <= args.minimum_correlation_margin <= 2:
        raise ValueError("--minimum-correlation-margin must be in [0, 2]")


def _read_csv(path: Path) -> tuple[dict[str, str], list[dict[str, str]]]:
    metadata: dict[str, str] = {}
    data_lines: list[str] = []
    with path.open("r", encoding="utf-8", errors="ignore") as stream:
        for raw in stream:
            stripped = raw.strip()
            if not stripped:
                continue
            if stripped.startswith("#"):
                key, separator, value = stripped.partition("=")
                if separator:
                    metadata[key] = value
                continue
            data_lines.append(raw)
    if not data_lines:
        raise ValueError(f"no data rows in {path}")
    if "#mm_columns" in metadata and "timestamp_ms" not in data_lines[0]:
        fieldnames = [
            name.strip() for name in metadata["#mm_columns"].split(",") if name.strip()
        ]
        return metadata, list(csv.DictReader(data_lines, fieldnames=fieldnames))
    return metadata, list(csv.DictReader(data_lines))


def _column(
    rows: Sequence[dict[str, str]],
    names: Sequence[str],
    *,
    required: bool = True,
) -> Optional[np.ndarray]:
    for name in names:
        if name not in rows[0]:
            continue
        values: list[float] = []
        for row in rows:
            try:
                values.append(float(row.get(name, "nan")))
            except ValueError:
                values.append(float("nan"))
        return np.asarray(values, dtype=float)
    if required:
        raise ValueError(f"missing column; tried {', '.join(names)}")
    return None


def _increasing_finite_indexes(
    time_s: np.ndarray,
    *values: np.ndarray,
) -> np.ndarray:
    mask = np.isfinite(time_s)
    for value in values:
        mask &= np.isfinite(value)
    indexes = np.flatnonzero(mask)
    if not len(indexes):
        return indexes
    keep = [int(indexes[0])]
    last_time = float(time_s[indexes[0]])
    for index in indexes[1:]:
        if time_s[index] > last_time:
            keep.append(int(index))
            last_time = float(time_s[index])
    return np.asarray(keep, dtype=int)


def _smooth_by_time(
    values: np.ndarray,
    time_s: np.ndarray,
    window_s: float,
) -> np.ndarray:
    if len(values) < 3:
        return values.copy()
    periods = np.diff(time_s)
    periods = periods[periods > 0]
    if not len(periods):
        return values.copy()
    count = max(1, int(round(window_s / float(np.median(periods)))))
    if count <= 1:
        return values.copy()
    if count % 2 == 0:
        count += 1
    kernel = np.ones(count, dtype=float) / count
    pad = count // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def _gradient(values: np.ndarray, time_s: np.ndarray) -> np.ndarray:
    return np.gradient(values, time_s)


def load_video(path: Path) -> Series:
    _, rows = _read_csv(path)
    time_s = _column(rows, ("video_pts_s", "time_s"))
    yaw = _column(
        rows,
        ("yaw_deg_unwrapped", "yaw_deg", "heading_deg_unwrapped"),
        required=False,
    )
    x_mm = _column(rows, ("x_mm",), required=False)
    y_mm = _column(rows, ("y_mm",), required=False)
    speed = _column(rows, ("speed_mm_s",), required=False)
    assert time_s is not None
    if yaw is not None:
        yaw = np.degrees(np.unwrap(np.radians(yaw)))
    if speed is None and x_mm is not None and y_mm is not None:
        speed = np.hypot(_gradient(x_mm, time_s), _gradient(y_mm, time_s))
    omega = None
    if yaw is not None:
        omega = _smooth_by_time(
            _gradient(yaw, time_s),
            time_s,
            0.050,
        )
    reference = omega if omega is not None else speed
    if reference is None:
        raise ValueError(
            "trajectory needs yaw or metric position/speed for synchronization"
        )
    indexes = _increasing_finite_indexes(time_s, reference)
    if len(indexes) < 5:
        raise ValueError("trajectory has fewer than five usable timestamp rows")

    def select(value: Optional[np.ndarray]) -> Optional[np.ndarray]:
        return None if value is None else value[indexes]

    time_selected = time_s[indexes]
    time_selected = time_selected - time_selected[0]
    return Series(
        time_s=time_selected,
        signal=np.zeros(len(indexes)),
        x_mm=select(x_mm),
        y_mm=select(y_mm),
        yaw_deg=select(yaw),
        speed_mm_s=select(speed),
        omega_dps=select(omega),
    )


def load_trace(path: Path) -> tuple[Series, dict[str, str], list[dict[str, str]]]:
    metadata, rows = _read_csv(path)
    timestamp_ms = _column(rows, ("timestamp_ms",))
    omega_mdps = _column(
        rows,
        ("real_omega_mdps", "omega_z_mdps", "gyro_z_raw_mdps"),
        required=False,
    )
    speed = _column(
        rows,
        ("real_velocity_mm_s",),
        required=False,
    )
    assert timestamp_ms is not None
    omega = None if omega_mdps is None else omega_mdps / 1000.0
    reference = omega if omega is not None else speed
    if reference is None:
        raise ValueError(
            "trace needs real_omega_mdps/omega_z_mdps or real_velocity_mm_s"
        )
    indexes = _increasing_finite_indexes(timestamp_ms, reference)
    if len(indexes) < 5:
        raise ValueError("trace has fewer than five usable timestamp rows")
    time_s = timestamp_ms[indexes] / 1000.0
    time_s -= time_s[0]

    def select(value: Optional[np.ndarray]) -> Optional[np.ndarray]:
        return None if value is None else value[indexes]

    series = Series(
        time_s=time_s,
        signal=np.zeros(len(indexes)),
        speed_mm_s=select(speed),
        omega_dps=select(omega),
    )
    metadata["#sync_trace_timestamp_origin_ms"] = str(
        int(round(timestamp_ms[indexes[0]]))
    )
    series.output_time_s = (timestamp_ms - timestamp_ms[indexes[0]]) / 1000.0
    return series, metadata, rows


def select_signal(
    video: Series,
    trace: Series,
    requested: str,
) -> str:
    yaw_available = video.omega_dps is not None and trace.omega_dps is not None
    speed_available = video.speed_mm_s is not None and trace.speed_mm_s is not None
    if requested == "yaw":
        if not yaw_available:
            raise ValueError("yaw synchronization columns are unavailable")
        return "yaw"
    if requested == "speed":
        if not speed_available:
            raise ValueError("speed synchronization columns are unavailable")
        return "speed"
    if yaw_available:
        assert video.yaw_deg is not None and trace.omega_dps is not None
        if (
            float(np.ptp(video.yaw_deg)) >= 20.0
            and float(np.max(np.abs(trace.omega_dps))) >= 20.0
        ):
            return "yaw"
    if speed_available:
        return "speed"
    if yaw_available:
        return "yaw"
    raise ValueError("no common video/trace synchronization signal")


def _signal_for(series: Series, name: str) -> np.ndarray:
    if name == "yaw":
        assert series.omega_dps is not None
        return series.omega_dps
    assert series.speed_mm_s is not None
    return np.abs(series.speed_mm_s)


def _activity_bounds(
    time_s: np.ndarray,
    signal: np.ndarray,
    padding_s: float,
) -> tuple[float, float]:
    magnitude = np.abs(signal)
    peak = float(np.max(magnitude))
    if peak <= 1e-9:
        raise ValueError("synchronization signal contains no activity")
    threshold = max(0.08 * peak, 8.0)
    indexes = np.flatnonzero(magnitude >= threshold)
    if not len(indexes):
        raise ValueError("could not find synchronization activity")
    return (
        max(float(time_s[0]), float(time_s[indexes[0]]) - padding_s),
        min(float(time_s[-1]), float(time_s[indexes[-1]]) + padding_s),
    )


def _correlation(a: np.ndarray, b: np.ndarray) -> float:
    if len(a) < 3:
        return float("-inf")
    a_centered = a - np.mean(a)
    b_centered = b - np.mean(b)
    denominator = float(np.linalg.norm(a_centered) * np.linalg.norm(b_centered))
    if denominator <= 1e-12:
        return float("-inf")
    return float(np.dot(a_centered, b_centered) / denominator)


def _fit_gain_bias(
    source: np.ndarray,
    target: np.ndarray,
) -> tuple[float, float, float]:
    matrix = np.column_stack([source, np.ones(len(source))])
    gain, bias = np.linalg.lstsq(matrix, target, rcond=None)[0]
    residual = target - (gain * source + bias)
    return (
        float(gain),
        float(bias),
        float(np.sqrt(np.mean(np.square(residual)))),
    )


def align(
    video: Series,
    trace: Series,
    signal_name: str,
    args: argparse.Namespace,
) -> Alignment:
    video_signal = _smooth_by_time(
        _signal_for(video, signal_name),
        video.time_s,
        0.040 if signal_name == "yaw" else 0.060,
    )
    trace_signal = _smooth_by_time(
        _signal_for(trace, signal_name),
        trace.time_s,
        0.020 if signal_name == "yaw" else 0.040,
    )
    video_start, video_end = _activity_bounds(
        video.time_s,
        video_signal,
        args.activity_padding_s,
    )
    trace_start, trace_end = _activity_bounds(
        trace.time_s,
        trace_signal,
        args.activity_padding_s,
    )
    if args.estimate_drift:
        span = args.maximum_drift_ppm * 1e-6
        scales = np.linspace(
            1.0 - span,
            1.0 + span,
            args.drift_steps,
        )
    else:
        scales = np.asarray([1.0])
    step = args.offset_step_ms / 1000.0
    yaw_sign = getattr(args, "yaw_sign", "same")
    if signal_name != "yaw" or yaw_sign == "same":
        signs = (1,)
    elif yaw_sign == "inverted":
        signs = (-1,)
    else:
        signs = (1, -1)
    best: Optional[tuple[float, float, int, float, np.ndarray, np.ndarray]] = None
    candidate_scores: list[tuple[float, float, int, float]] = []
    minimum_activity_overlap_fraction = getattr(
        args,
        "minimum_activity_overlap_fraction",
        0.90,
    )

    for scale in scales:
        maximum_activity_overlap = min(
            trace_end - trace_start,
            scale * (video_end - video_start),
        )
        required_overlap = max(
            args.minimum_overlap_s,
            minimum_activity_overlap_fraction * maximum_activity_overlap,
        )
        onset_offset = trace_start - scale * video_start
        offsets = np.arange(
            onset_offset - args.offset_search_s,
            onset_offset + args.offset_search_s + step * 0.5,
            step,
        )
        for offset in offsets:
            trace_low = max(
                trace_start,
                scale * video_start + offset,
            )
            trace_high = min(
                trace_end,
                scale * video_end + offset,
            )
            if trace_high - trace_low < required_overlap:
                continue
            mask = (trace.time_s >= trace_low) & (trace.time_s <= trace_high)
            trace_times = trace.time_s[mask]
            if len(trace_times) < 10:
                continue
            video_times = (trace_times - offset) / scale
            interpolated = np.interp(
                video_times,
                video.time_s,
                video_signal,
            )
            target = trace_signal[mask]
            for sign in signs:
                score = _correlation(sign * interpolated, target)
                candidate_scores.append((score, float(offset), sign, float(scale)))
                if best is None or score > best[0]:
                    best = (
                        score,
                        float(offset),
                        sign,
                        float(scale),
                        sign * interpolated,
                        target,
                    )
    if best is None:
        raise RuntimeError("no alignment candidate met the overlap gate")
    score, offset, sign, scale, matched_video, matched_trace = best
    gain, bias, rmse = _fit_gain_bias(matched_video, matched_trace)
    trace_std = float(np.std(matched_trace))
    normalized_rmse = rmse / max(trace_std, 1e-12)
    distinct_offset_s = max(0.080, 4.0 * step)
    alternate_scores = [
        candidate_score
        for candidate_score, candidate_offset, candidate_sign, _ in candidate_scores
        if candidate_sign != sign or abs(candidate_offset - offset) >= distinct_offset_s
    ]
    second_correlation = max(alternate_scores) if alternate_scores else float("-inf")
    correlation_margin = (
        score - second_correlation
        if math.isfinite(second_correlation)
        else float("inf")
    )
    overlap_low = max(trace_start, scale * video_start + offset)
    overlap_high = min(trace_end, scale * video_end + offset)
    return Alignment(
        offset_s=offset,
        scale=scale,
        sign=sign,
        correlation=score,
        overlap_s=float(overlap_high - overlap_low),
        matched_samples=len(matched_video),
        signal_gain=gain,
        signal_bias=bias,
        signal_rmse=rmse,
        normalized_rmse=normalized_rmse,
        second_correlation=second_correlation,
        correlation_margin=correlation_margin,
    )


def _interpolate_optional(
    value: Optional[np.ndarray],
    source_time: np.ndarray,
    query_time: np.ndarray,
) -> np.ndarray:
    if value is None:
        return np.full(len(query_time), np.nan)
    return np.interp(query_time, source_time, value)


def _validate_output_does_not_alias_input(
    output_path: Path,
    input_paths: Sequence[Path],
) -> None:
    output_resolved = output_path.resolve()
    for input_path in input_paths:
        if output_resolved == input_path.resolve():
            raise ValueError(
                f"refusing to overwrite input CSV with fused output: {input_path}"
            )


def _unique_field_name(preferred: str, occupied: set[str]) -> str:
    candidate = preferred
    suffix = 2
    while candidate in occupied:
        candidate = f"{preferred}_{suffix}"
        suffix += 1
    occupied.add(candidate)
    return candidate


def _unnamed_values(row: dict[str, str]) -> list[str]:
    """Return surplus values stored under DictReader's ``None`` rest key."""
    value = row.get(None)  # type: ignore[arg-type]
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return ["" if item is None else str(item) for item in value]
    return [str(value)]


def write_fused_csv(
    path: Path,
    video: Series,
    trace: Series,
    trace_rows: Sequence[dict[str, str]],
    alignment: Alignment,
) -> dict[str, str]:
    """Write a lossless trace copy plus uniquely named generated columns.

    Source column names and values are authoritative.  A generated column that
    collides with a source column is renamed with a ``fused_`` prefix (and a
    numeric suffix if needed).  Values beyond the input header, which
    ``csv.DictReader`` stores under a ``None`` key, are expanded into explicit
    ``trace_unnamed_extra_N`` columns rather than discarded.

    The return value maps each canonical generated column name to the actual
    output column name.
    """
    trace_output_time = (
        trace.output_time_s if trace.output_time_s is not None else trace.time_s
    )
    video_time = (trace_output_time - alignment.offset_s) / alignment.scale
    in_video = (
        np.isfinite(video_time)
        & (video_time >= video.time_s[0])
        & (video_time <= video.time_s[-1])
    )
    x = _interpolate_optional(video.x_mm, video.time_s, video_time)
    y = _interpolate_optional(video.y_mm, video.time_s, video_time)
    yaw = _interpolate_optional(video.yaw_deg, video.time_s, video_time)
    speed = _interpolate_optional(
        video.speed_mm_s,
        video.time_s,
        video_time,
    )
    omega = _interpolate_optional(
        video.omega_dps,
        video.time_s,
        video_time,
    )
    video_fields = [
        "trace_timestamp_ms",
        "trace_relative_s",
        "video_pts_s",
        "video_in_range",
        "video_x_mm",
        "video_y_mm",
        "video_yaw_deg_unwrapped",
        "video_speed_mm_s",
        "video_omega_dps",
    ]
    trace_fields: list[str] = []
    seen_fields: set[str] = set()
    for row in trace_rows:
        for name in row:
            if name is not None and name not in seen_fields:
                trace_fields.append(name)
                seen_fields.add(name)
    occupied_fields = set(trace_fields)
    maximum_unnamed = max(
        (_unnamed_values(row) for row in trace_rows),
        key=len,
        default=[],
    )
    unnamed_fields = [
        _unique_field_name(
            f"trace_unnamed_extra_{index + 1}",
            occupied_fields,
        )
        for index in range(len(maximum_unnamed))
    ]
    generated_fields: dict[str, str] = {}
    for name in video_fields:
        preferred = name if name not in occupied_fields else f"fused_{name}"
        generated_fields[name] = _unique_field_name(
            preferred,
            occupied_fields,
        )
    fields = (
        [generated_fields[name] for name in video_fields]
        + trace_fields
        + unnamed_fields
    )
    with path.open("w", encoding="ascii", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, source in enumerate(trace_rows):

            def fmt(value: float) -> str:
                return f"{value:.6f}" if math.isfinite(float(value)) else ""

            output_row = {
                name: (source.get(name, "") if source.get(name, "") is not None else "")
                for name in trace_fields
            }
            unnamed_values = _unnamed_values(source)
            output_row.update(
                {
                    name: (
                        unnamed_values[extra_index]
                        if extra_index < len(unnamed_values)
                        else ""
                    )
                    for extra_index, name in enumerate(unnamed_fields)
                }
            )
            generated_values = {
                "trace_timestamp_ms": source.get("timestamp_ms", ""),
                "trace_relative_s": fmt(trace_output_time[index]),
                "video_pts_s": (fmt(video_time[index]) if in_video[index] else ""),
                "video_in_range": int(in_video[index]),
                "video_x_mm": fmt(x[index]) if in_video[index] else "",
                "video_y_mm": fmt(y[index]) if in_video[index] else "",
                "video_yaw_deg_unwrapped": (fmt(yaw[index]) if in_video[index] else ""),
                "video_speed_mm_s": (fmt(speed[index]) if in_video[index] else ""),
                "video_omega_dps": (fmt(omega[index]) if in_video[index] else ""),
            }
            output_row.update(
                {
                    generated_fields[name]: value
                    for name, value in generated_values.items()
                }
            )
            writer.writerow(output_row)
    return generated_fields


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        video = load_video(args.trajectory_csv)
        trace, trace_metadata, trace_rows = load_trace(args.trace_csv)
        signal_name = select_signal(video, trace, args.signal)
        alignment = align(video, trace, signal_name, args)
        qa_failures: list[str] = []
        if alignment.correlation < args.minimum_correlation:
            qa_failures.append(
                "correlation {:.3f} < {:.3f}".format(
                    alignment.correlation,
                    args.minimum_correlation,
                )
            )
        if alignment.correlation_margin < args.minimum_correlation_margin:
            qa_failures.append(
                "correlation margin {:.3f} < {:.3f}".format(
                    alignment.correlation_margin,
                    args.minimum_correlation_margin,
                )
            )
        if not (
            args.minimum_signal_gain
            <= alignment.signal_gain
            <= args.maximum_signal_gain
        ):
            qa_failures.append(
                "gain {:.3f} outside {:.3f}..{:.3f}".format(
                    alignment.signal_gain,
                    args.minimum_signal_gain,
                    args.maximum_signal_gain,
                )
            )
        qa_passed = not qa_failures
        if not qa_passed and not args.allow_low_correlation:
            raise RuntimeError(
                "alignment QA failed: {}; use --allow-low-correlation only "
                "for diagnostics".format("; ".join(qa_failures))
            )
        output_dir = (
            args.output_dir
            if args.output_dir is not None
            else args.trajectory_csv.parent / f"fused_{args.trace_csv.stem}"
        )
        fused_path = output_dir / "fused.csv"
        _validate_output_does_not_alias_input(
            fused_path,
            (args.trajectory_csv, args.trace_csv),
        )
        output_dir.mkdir(parents=True, exist_ok=True)
        generated_columns = write_fused_csv(
            fused_path,
            video,
            trace,
            trace_rows,
            alignment,
        )
        report = {
            "schema": "nightfall_trace_video_sync_v2",
            "inputs": {
                "trajectory_csv": str(args.trajectory_csv.resolve()),
                "trace_csv": str(args.trace_csv.resolve()),
                "trace_metadata": trace_metadata,
                "trace_rows_total": len(trace_rows),
                "trace_rows_used_for_alignment": len(trace.time_s),
            },
            "mapping": {
                "equation": ("trace_relative_s = scale * video_pts_s + offset_s"),
                "offset_s": alignment.offset_s,
                "scale": alignment.scale,
                "drift_ppm": (alignment.scale - 1.0) * 1_000_000.0,
            },
            "signal": {
                "selected": signal_name,
                "video_sign": alignment.sign,
                "correlation": alignment.correlation,
                "second_correlation": (
                    alignment.second_correlation
                    if math.isfinite(alignment.second_correlation)
                    else None
                ),
                "correlation_margin": (
                    alignment.correlation_margin
                    if math.isfinite(alignment.correlation_margin)
                    else None
                ),
                "gain_trace_per_video": alignment.signal_gain,
                "bias_trace_units": alignment.signal_bias,
                "rmse_trace_units": alignment.signal_rmse,
                "normalized_rmse": alignment.normalized_rmse,
                "matched_samples": alignment.matched_samples,
                "overlap_s": alignment.overlap_s,
            },
            "qa": {
                "minimum_correlation": args.minimum_correlation,
                "minimum_correlation_margin": (args.minimum_correlation_margin),
                "minimum_signal_gain": args.minimum_signal_gain,
                "maximum_signal_gain": args.maximum_signal_gain,
                "minimum_overlap_s": args.minimum_overlap_s,
                "minimum_activity_overlap_fraction": (
                    args.minimum_activity_overlap_fraction
                ),
                "passed": qa_passed,
                "failures": qa_failures,
                "drift_was_estimated": args.estimate_drift,
            },
            "outputs": {
                "fused_csv": fused_path.name,
                "generated_columns": generated_columns,
            },
        }
        (output_dir / "sync_report.json").write_text(
            json.dumps(
                report,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            + "\n",
            encoding="utf-8",
        )
        print(
            "[TRACE-VIDEO] signal={} correlation={:.3f} offset={:.6f}s "
            "scale={:.9f} drift={:.1f}ppm overlap={:.3f}s".format(
                signal_name,
                alignment.correlation,
                alignment.offset_s,
                alignment.scale,
                (alignment.scale - 1.0) * 1_000_000.0,
                alignment.overlap_s,
            )
        )
        print(f"[TRACE-VIDEO] output={output_dir} qa_passed={int(qa_passed)}")
        return 0
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(f"[TRACE-VIDEO][ERROR] {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
