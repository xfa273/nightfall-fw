#!/usr/bin/env python3
"""Fit Nightfall wall-sensor ADC-to-distance LUTs.

Input CSV rows should contain a true distance column and one or more sensor
delta-ADC columns, for example:

distance_mm,fr_delta,fl_delta,r_delta,l_delta
40,4080,4090,,
40,4076,4087,,
45,3890,3340,,
...
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, pstdev
from typing import Callable, Iterable


SENSOR_ALIASES = {
    "fr": ("fr_delta", "wall_read_fr", "fr", "FR"),
    "fl": ("fl_delta", "wall_read_fl", "fl", "FL"),
    "r": ("r_delta", "wall_read_r", "sr_delta", "right_delta", "r", "R"),
    "l": ("l_delta", "wall_read_l", "sl_delta", "left_delta", "l", "L"),
}


@dataclass
class GroupPoint:
    distance_mm: float
    mean_ad: float
    std_ad: float
    count: int
    adjusted_ad: float


@dataclass
class EvalResult:
    name: str
    sample_max_abs_mm: float
    sample_rms_mm: float
    loo_max_abs_mm: float
    loo_rms_mm: float


def _float_or_none(value: str | None) -> float | None:
    if value is None:
        return None
    value = value.strip()
    if value == "":
        return None
    try:
        v = float(value)
    except ValueError:
        return None
    if not math.isfinite(v):
        return None
    return v


def _resolve_column(columns: Iterable[str], requested: str | None, aliases: Iterable[str]) -> str | None:
    cols = list(columns)
    if requested:
        return requested if requested in cols else None
    for alias in aliases:
        if alias in cols:
            return alias
    return None


def _read_rows(path: Path, distance_col: str, sensor_cols: dict[str, str]) -> dict[str, list[tuple[float, float]]]:
    out: dict[str, list[tuple[float, float]]] = {name: [] for name in sensor_cols}
    with path.open(newline="") as f:
        reader = csv.DictReader(row for row in f if not row.lstrip().startswith("#"))
        if reader.fieldnames is None:
            raise SystemExit(f"{path}: no CSV header found")
        for row in reader:
            dist = _float_or_none(row.get(distance_col))
            if dist is None:
                continue
            for name, col in sensor_cols.items():
                ad = _float_or_none(row.get(col))
                if ad is None or ad <= 0:
                    continue
                out[name].append((dist, ad))
    return out


def _pava_non_decreasing(values: list[float], weights: list[float]) -> list[float]:
    blocks: list[list[float]] = []
    for i, (v, w) in enumerate(zip(values, weights)):
        blocks.append([float(i), float(i), w, v * w])
        while len(blocks) >= 2:
            a = blocks[-2]
            b = blocks[-1]
            avg_a = a[3] / a[2]
            avg_b = b[3] / b[2]
            if avg_a <= avg_b:
                break
            merged = [a[0], b[1], a[2] + b[2], a[3] + b[3]]
            blocks[-2:] = [merged]

    out = [0.0] * len(values)
    for start, end, w, yw in blocks:
        avg = yw / w
        for i in range(int(start), int(end) + 1):
            out[i] = avg
    return out


def _pava_non_increasing(values: list[float], weights: list[float]) -> list[float]:
    return [-v for v in _pava_non_decreasing([-v for v in values], weights)]


def _group_sensor(samples: list[tuple[float, float]]) -> list[GroupPoint]:
    by_distance: dict[float, list[float]] = {}
    for dist, ad in samples:
        by_distance.setdefault(dist, []).append(ad)
    distances = sorted(by_distance)
    means = [mean(by_distance[d]) for d in distances]
    weights = [float(len(by_distance[d])) for d in distances]
    adjusted = _pava_non_increasing(means, weights)
    return [
        GroupPoint(
            distance_mm=d,
            mean_ad=means[i],
            std_ad=pstdev(by_distance[d]) if len(by_distance[d]) > 1 else 0.0,
            count=len(by_distance[d]),
            adjusted_ad=adjusted[i],
        )
        for i, d in enumerate(distances)
    ]


def _find_segment_desc(ad: list[float], value: float) -> tuple[int, bool]:
    n = len(ad)
    if value >= ad[0]:
        return 0, True
    if value <= ad[-1]:
        return n - 2, True
    lo, hi = 0, n - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if value > ad[mid]:
            hi = mid
        else:
            lo = mid
    return lo, False


def _linear_from_ad(mm: list[float], ad: list[float], value: float) -> float:
    i, _ = _find_segment_desc(ad, value)
    den = ad[i + 1] - ad[i]
    if den == 0:
        return mm[i]
    t = (value - ad[i]) / den
    return mm[i] + t * (mm[i + 1] - mm[i])


def _linear_from_x(mm: list[float], x: list[float], value: float) -> float:
    # x is monotonically decreasing with distance, same as AD.
    return _linear_from_ad(mm, x, value)


def _pchip_slopes_desc_ad(mm: list[float], ad: list[float]) -> list[float]:
    n = len(mm)
    h = [ad[i] - ad[i + 1] for i in range(n - 1)]
    d = [(mm[i + 1] - mm[i]) / h[i] for i in range(n - 1)]
    if n == 2:
        return [d[0], d[0]]

    m = [0.0] * n
    for i in range(1, n - 1):
        if d[i - 1] * d[i] <= 0:
            m[i] = 0.0
        else:
            w1 = 2 * h[i] + h[i - 1]
            w2 = h[i] + 2 * h[i - 1]
            m[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i])

    def endpoint(h0: float, h1: float, d0: float, d1: float) -> float:
        v = ((2 * h0 + h1) * d0 - h0 * d1) / (h0 + h1)
        if v * d0 <= 0:
            return 0.0
        if abs(v) > abs(3 * d0):
            return 3 * d0
        return v

    m[0] = endpoint(h[0], h[1], d[0], d[1])
    m[-1] = endpoint(h[-1], h[-2], d[-1], d[-2])
    return m


def _pchip_from_ad(mm: list[float], ad: list[float], slopes: list[float], value: float) -> float:
    i, extrap = _find_segment_desc(ad, value)
    x = -value
    if extrap:
        if value >= ad[0]:
            return mm[0] + slopes[0] * (x - (-ad[0]))
        return mm[-1] + slopes[-1] * (x - (-ad[-1]))

    x0 = -ad[i]
    h = ad[i] - ad[i + 1]
    t = (x - x0) / h
    t2 = t * t
    t3 = t2 * t
    h00 = 2 * t3 - 3 * t2 + 1
    h10 = t3 - 2 * t2 + t
    h01 = -2 * t3 + 3 * t2
    h11 = t3 - t2
    return h00 * mm[i] + h10 * h * slopes[i] + h01 * mm[i + 1] + h11 * h * slopes[i + 1]


def _model_eval(name: str, mm: list[float], ad: list[float]) -> Callable[[float], float]:
    if name == "linear":
        return lambda v: _linear_from_ad(mm, ad, v)
    if name == "pchip":
        slopes = _pchip_slopes_desc_ad(mm, ad)
        return lambda v: _pchip_from_ad(mm, ad, slopes, v)
    if name == "log-linear":
        x = [math.log(max(1.0, a)) for a in ad]
        return lambda v: _linear_from_x(mm, x, math.log(max(1.0, v)))
    if name == "invsqrt-linear":
        x = [1.0 / math.sqrt(max(1.0, a)) for a in ad]
        # This axis increases with distance, so reverse it into decreasing form.
        return lambda v: _linear_from_ad(list(reversed(mm)), list(reversed(x)), 1.0 / math.sqrt(max(1.0, v)))
    raise ValueError(name)


def _rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def _evaluate(name: str, points: list[GroupPoint], samples: list[tuple[float, float]]) -> EvalResult:
    mm = [p.distance_mm for p in points]
    ad = [p.adjusted_ad for p in points]
    model = _model_eval(name, mm, ad)
    sample_errors = [model(sample_ad) - true_mm for true_mm, sample_ad in samples]

    loo_errors: list[float] = []
    if len(points) >= 4:
        for i in range(1, len(points) - 1):
            mm2 = mm[:i] + mm[i + 1 :]
            ad2 = ad[:i] + ad[i + 1 :]
            pred = _model_eval(name, mm2, ad2)(ad[i])
            loo_errors.append(pred - mm[i])

    return EvalResult(
        name=name,
        sample_max_abs_mm=max((abs(e) for e in sample_errors), default=0.0),
        sample_rms_mm=_rms(sample_errors),
        loo_max_abs_mm=max((abs(e) for e in loo_errors), default=0.0),
        loo_rms_mm=_rms(loo_errors),
    )


def _adaptive_linear_indices(points: list[GroupPoint], max_error_mm: float) -> list[int]:
    mm = [p.distance_mm for p in points]
    ad = [p.adjusted_ad for p in points]
    keep = {0, len(points) - 1}

    def split(lo: int, hi: int) -> None:
        if hi <= lo + 1:
            return
        segment_mm = [mm[lo], mm[hi]]
        segment_ad = [ad[lo], ad[hi]]
        worst_i = -1
        worst = 0.0
        for i in range(lo + 1, hi):
            pred = _linear_from_ad(segment_mm, segment_ad, ad[i])
            err = abs(pred - mm[i])
            if err > worst:
                worst = err
                worst_i = i
        if worst > max_error_mm and worst_i >= 0:
            keep.add(worst_i)
            split(lo, worst_i)
            split(worst_i, hi)

    split(0, len(points) - 1)
    return sorted(keep)


def _fmt_array(name: str, values: list[int]) -> str:
    chunks = []
    for i in range(0, len(values), 12):
        chunks.append("    " + ", ".join(str(v) for v in values[i : i + 12]))
    return f"  static const uint16_t {name}[] = {{\n" + ",\n".join(chunks) + "\n  };\n"


def _emit_c(sensors: dict[str, list[GroupPoint]]) -> str:
    lines = [
        '#include "sensor_distance.h"',
        "",
        "void sensor_distance_load_profile_luts(void)",
        "{",
    ]
    call_names = {"fr": "fr", "fl": "fl", "r": "r", "l": "l"}
    for name in ("fr", "fl", "r", "l"):
        points = sensors.get(name)
        if not points:
            continue
        mm = [int(round(p.distance_mm)) for p in points]
        ad = [max(1, min(65535, int(round(p.adjusted_ad)))) for p in points]
        lines.append(_fmt_array(f"s_{name}_mm", mm).rstrip())
        lines.append(_fmt_array(f"s_{name}_ad", ad).rstrip())
        lines.append(
            f"  (void)sensor_distance_set_lut_{call_names[name]}(s_{name}_mm, s_{name}_ad, sizeof(s_{name}_mm) / sizeof(s_{name}_mm[0]));"
        )
        lines.append("")

    fr = sensors.get("fr")
    fl = sensors.get("fl")
    if fr and fl:
        fr_by_mm = {int(round(p.distance_mm)): p.adjusted_ad for p in fr}
        fl_by_mm = {int(round(p.distance_mm)): p.adjusted_ad for p in fl}
        common = sorted(set(fr_by_mm) & set(fl_by_mm))
        if len(common) >= 2:
            fsum_ad = [max(1, min(65535, int(round(fr_by_mm[d] + fl_by_mm[d])))) for d in common]
            lines.append(_fmt_array("s_front_sum_mm", common).rstrip())
            lines.append(_fmt_array("s_front_sum_ad", fsum_ad).rstrip())
            lines.append(
                "  (void)sensor_distance_set_lut_front_sum(s_front_sum_mm, s_front_sum_ad, sizeof(s_front_sum_mm) / sizeof(s_front_sum_mm[0]));"
            )
    lines.append("}")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="Fit F413 wall sensor distance LUTs from calibration CSV")
    ap.add_argument("csv", type=Path)
    ap.add_argument("--distance-column", default="distance_mm")
    ap.add_argument("--sensors", default="fr,fl,r,l", help="comma-separated sensor names: fr,fl,r,l")
    ap.add_argument("--max-error-mm", type=float, default=1.0)
    ap.add_argument("--emit-c", type=Path, default=None, help="write params/f413_preorder/sensor_distance_lut.c")
    args = ap.parse_args()

    with args.csv.open(newline="") as f:
        reader = csv.reader(row for row in f if not row.lstrip().startswith("#"))
        header = next(reader, None)
    if header is None:
        raise SystemExit(f"{args.csv}: no CSV header found")
    if args.distance_column not in header:
        raise SystemExit(f"{args.csv}: missing distance column {args.distance_column!r}")

    requested = [s.strip().lower() for s in args.sensors.split(",") if s.strip()]
    sensor_cols: dict[str, str] = {}
    for name in requested:
        if name not in SENSOR_ALIASES:
            raise SystemExit(f"unknown sensor {name!r}")
        col = _resolve_column(header, None, SENSOR_ALIASES[name])
        if col is not None:
            sensor_cols[name] = col
    if not sensor_cols:
        raise SystemExit("no sensor columns found")

    raw = _read_rows(args.csv, args.distance_column, sensor_cols)
    grouped: dict[str, list[GroupPoint]] = {}
    for name, samples in raw.items():
        if len(samples) < 2:
            continue
        points = _group_sensor(samples)
        if len(points) < 2:
            continue
        grouped[name] = points

        print(f"[{name}] points={len(points)} samples={len(samples)} column={sensor_cols[name]}")
        for p in points:
            changed = "" if abs(p.adjusted_ad - p.mean_ad) < 0.5 else f" adjusted={p.adjusted_ad:.1f}"
            print(
                f"  {p.distance_mm:7.2f} mm: mean_ad={p.mean_ad:8.1f} std={p.std_ad:6.1f} n={p.count}{changed}"
            )

        for model_name in ("linear", "pchip", "log-linear", "invsqrt-linear"):
            result = _evaluate(model_name, points, samples)
            print(
                f"  eval {result.name:15s} sample_max={result.sample_max_abs_mm:6.3f} "
                f"sample_rms={result.sample_rms_mm:6.3f} loo_max={result.loo_max_abs_mm:6.3f} "
                f"loo_rms={result.loo_rms_mm:6.3f}"
            )
        adaptive = _adaptive_linear_indices(points, args.max_error_mm)
        print(f"  adaptive-linear knots for <= {args.max_error_mm:.3f} mm: {len(adaptive)}/{len(points)}")

    if args.emit_c is not None:
        args.emit_c.write_text(_emit_c(grouped))
        print(f"[fit_sensor_distance] wrote {args.emit_c}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
