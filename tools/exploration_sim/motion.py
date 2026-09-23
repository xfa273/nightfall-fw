"""motion.py - 走行パラメータから行動別の所要時間を求める時間モデル.

nightfall-fw の `params/<board>/search_run_params_split.c` /
`shortest_run_params_split.c` を正規表現で読み、
「直線 k セル」「旋回 90/180/45 度」の所要時間と、
区間をつないだ際の加減速を台形プロファイルで積分する。

単位系はファームに合わせ mm / mm/s / mm/s^2 / deg / deg/s / deg/s^2。
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

_NUM = r"(-?\d+(?:\.\d+)?)"


def parse_params_file(path) -> list[dict]:
    """C の構造体初期化ブロックを `[{名前: 値}, ...]` に展開する。"""
    text = open(path, "r", encoding="utf-8", errors="ignore").read()
    blocks = []
    for chunk in re.split(r"\{(?=\s*\.)", text):
        pairs = dict((k, float(v)) for k, v in re.findall(r"\.(\w+)\s*=\s*" + _NUM, chunk))
        if len(pairs) >= 3:
            blocks.append(pairs)
    return blocks


def parse_params_paths(paths) -> dict:
    """複数ファイルを順に読み、後のファイルが上書きしないよう「最初に見つかった値」で統合する。"""
    merged, used = {}, {}
    for p in paths:
        try:
            blocks = parse_params_file(p)
        except OSError:
            continue
        for b in blocks:
            for k, v in b.items():
                if k not in merged:
                    merged[k] = v
                    used[k] = "%s" % str(p).split("/")[-1]
    return merged


def pick(params: dict, names, default: float):
    for n in names:
        if n in params:
            return params[n]
    return default


@dataclass
class MotionModel:
    cell_mm: float = 180.0
    v_max: float = 1500.0            # 直線巡航 [mm/s]
    accel: float = 6000.0            # 直線加速度 [mm/s^2]
    decel: float = 6000.0            # 直線減速度 [mm/s^2]
    w90: float = 1000.0              # 90度旋回の角速度 [deg/s]
    a90: float = 19000.0             # 90度旋回の角加速度 [deg/s^2]
    w180: float | None = None
    a180: float | None = None
    w45: float = 500.0
    a45: float = 6000.0
    turn_settle_ms: float = 60.0     # 旋回後の安定化待ち
    turn_in_speed: float = 0.0       # 旋回入口の直線速度 [mm/s]（0=停止旋回）
    allow_45cut: bool = True
    source: str = "builtin-defaults"
    used_keys: dict = field(default_factory=dict)

    # --------------------------------------------------------------- factory
    @classmethod
    def from_params(cls, paths, cell_mm=180.0, turn_settle_ms=60.0,
                    allow_45cut=True, overrides=None) -> "MotionModel":
        params = parse_params_paths(list(paths))
        used = {}

        def g(names, default):
            for n in names:
                if n in params:
                    used[n] = params[n]
                    return params[n]
            return default

        w180 = g(["velocity_l_turn_180"], 0.0)
        a180 = g(["alpha_l_turn_180"], 0.0)
        m = cls(
            cell_mm=cell_mm,
            v_max=g(["velocity_straight", "velocity_max", "velocity_dash"], 1500.0),
            accel=g(["acceleration_straight"], 6000.0),
            decel=g(["acceleration_straight", "acceleration_straight_dash"], 6000.0),
            w90=g(["velocity_turn90"], 1000.0),
            a90=g(["alpha_turn90"], 19000.0),
            w180=(w180 or None),
            a180=(a180 or None),
            w45=g(["velocity_turn45in", "velocity_turn45out"], 500.0),
            a45=g(["alpha_turn45in", "alpha_turn45out"], 6000.0),
            turn_settle_ms=turn_settle_ms,
            allow_45cut=allow_45cut,
            source=("params: " + ", ".join(sorted({v for v in
                    (parse_keys(params) if params else [])})) if False else
                    ("params (%d keys)" % len(params) if params else "builtin-defaults")),
            used_keys=used,
        )
        for k, v in (overrides or {}).items():
            setattr(m, k, v)
        m.used_keys = used
        return m

    # --------------------------------------------------------------- kernels
    @staticmethod
    def trapezoid_time(distance: float, v0: float, v1: float, vmax: float,
                       a_up: float, a_dn: float) -> tuple[float, float]:
        """距離 distance を v0→v1 で結ぶ最小時間。戻り値 (time, v_end)。"""
        if distance <= 0:
            return 0.0, v0
        a = a_up if v1 >= v0 else a_dn
        a_use = a if a > 0 else 1e9
        vp = math.sqrt(max((2 * a_use * distance + v0 * v0 + v1 * v1) / 2.0,
                           max(v0, v1) ** 2))
        if vp > vmax:                     # クルーズ区間あり
            vp = vmax
            d_ramp = (vp * vp - v0 * v0) / (2 * a_up) + (vp * vp - v1 * v1) / (2 * a_dn)
            t = (vp - v0) / a_up + (vp - v1) / a_dn
            if d_ramp < distance:
                t += (distance - d_ramp) / vp
            return t, v1
        t = (vp - v0) / a_up + (vp - v1) / a_dn
        return t, v1

    def turn_time(self, angle_deg: float, v_in: float = None) -> tuple[float, float]:
        """旋回の所要時間（角速度台形）と旋回後の直線速度。"""
        angle = abs(angle_deg)
        if angle <= 0:
            return 0.0, self.turn_in_speed
        if angle >= 135:
            w = self.w180 or self.w90
            a = self.a180 or self.a90
        elif angle <= 46:
            w, a = self.w45, self.a45
        else:
            w, a = self.w90, self.a90
        if a <= 0:
            a = 1e9
        if w <= 0:
            w = 1e9
        # 台形にできるか（角速度 w に到達するか）で式を変える
        peak_tri = math.sqrt(angle * a)       # 三角形プロファイルのピーク角速度
        if peak_tri <= w:
            t = 2 * math.sqrt(angle / a)
        else:
            t = angle / w + w / a
        return t + self.turn_settle_ms / 1000.0, self.turn_in_speed

    def straight_time(self, n_cells: int, v0: float = 0.0, v_end_needed: float = 0.0,
                      is_dash: bool = False) -> tuple[float, float]:
        """直線 n_cells の所要時間。返回 (秒, 終了時速度)。"""
        d = n_cells * self.cell_mm
        return self.trapezoid_time(d, v0, v_end_needed, self.v_max,
                                   self.accel, self.decel)

    def describe(self) -> str:
        return ("time-model: cell=%.0fmm v_max=%.0fmm/s a=%.0fmm/s^2 "
                "90deg=%.0fdeg/s/%.0fdeg/s^2 settle=%.0fms (%s)" % (
                    self.cell_mm, self.v_max, self.accel, self.w90, self.a90,
                    self.turn_settle_ms, self.source))


def parse_keys(params):  # 互換用（未使用）
    return list(params.keys())


class CellCostModel(MotionModel):
    """行動 1 つあたりの固定コスト（`--time-model cells`）。"""

    @classmethod
    def builtin(cls):
        m = cls(cell_mm=180.0)
        return m

    def turn_time(self, angle_deg, v_in=None):
        a = abs(angle_deg)
        if a <= 0:
            return 0.0, 0.0
        if a <= 46:
            return 0.20, 0.0
        if a <= 91:
            return 0.30, 0.0
        return 0.55, 0.0

    def straight_time(self, n_cells, v0=0.0, v_end_needed=0.0, is_dash=False):
        return 0.16 * n_cells, 0.0
