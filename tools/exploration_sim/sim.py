"""sim.py - 探索シミュレータ本体（走行・時間積算・戦略切替）.

現状ファーム相当 (strategy="fw") と代替戦略を、同じ時間モデル・同じセンサ
モデルで比較する。

  strategy  fw        ... make_smap() の 1 セルコスト BFS + 直進/右/左/後退 の降下
            dijkstra  ... 距離場を時間コスト（秒）にしただけ
            gain      ... 「未探索セルに隣接するどこへ行くかが、その後のゴール
                          所要時間を最も短くするか」で目標を選ぶ
  sensing   fw / fw_fixed / ideal3 / ideal4 / ideal4_ahead
  policy    dash_when_reachable ... 既知マップでゴール到達可能になった瞬間にゴールへ
            full_first          ... 全セル既知になるまで探索を優先する
"""

from __future__ import annotations

from dataclasses import dataclass, field

import firmware_model as fm
from firmware_model import (REL_ANGLE, REL_NAME, REL_PRIORITY, add, abs_dir,
                            bfs_field, dijkstra_field, field_at, goal_reachable,
                            seeds_unvisited, sense, write_map, MouseState)
from maze import DIR_DX, DIR_DY, DIR_WALL, DIR_N, turn_left, turn_right


@dataclass
class SimConfig:
    strategy: str = "fw"
    sensing: str = "fw"
    propagate: str = "fw"            # fw=「壁なし」も伝搬（ファーム同等）/ safe
    map_write: str = "overwrite"     # overwrite=ファーム同等 / accumulate=OR 蓄積
    unknown_walls: str = "open"      # open=map_Init 相当（未観測は開口）/ closed
    goal_policy: str = "full_first"
    max_runs: int = 5
    goal_weight: float = 1.0
    end_run_at_goal: str = "never"    # never=機体同等(全面探索はゴールで止めない) / always
    phantom_penalty: float = 1.0     # 未知の壁を疑わずに接近して失う時間 [秒]
    max_steps_per_run: int = 4000


@dataclass
class RunResult:
    index: int
    mode: str
    time: float
    cells: int
    cells_new: int
    turns: dict
    reached_goal: bool
    known_after: float
    phantom: int
    end_reason: str


@dataclass
class SimResult:
    maze_name: str
    config: SimConfig
    motion_desc: str
    runs: list = field(default_factory=list)
    timeline: list = field(default_factory=list)
    visit_order: dict = field(default_factory=dict)
    total_time: float = 0.0
    t_first_goal: float | None = None
    t_full_knowledge: float | None = None
    steps: int = 0
    phantom_total: int = 0
    final_known: float = 0.0
    wrong_walls: int = 0
    missing_walls: int = 0
    error: str = ""
    _touched_goal: bool = False

    def summary_row(self):
        return {
            "maze": self.maze_name,
            "strategy": self.config.strategy,
            "sensing": self.config.sensing,
            "policy": self.config.goal_policy,
            "unknown": self.config.unknown_walls,
            "map_write": self.config.map_write,
            "t_first_goal": (round(self.t_first_goal, 3) if self.t_first_goal is not None else None),
            "runs_to_goal": next((i + 1 for i, r in enumerate(self.runs) if r.reached_goal), None),
            "t_full_known": (round(self.t_full_knowledge, 3) if self.t_full_knowledge is not None else None),
            "total_time": round(self.total_time, 3),
            "best_shortest": round(min([r.time for r in self.runs if r.reached_goal], default=0.0), 3),
            "cells_moved": sum(r.cells for r in self.runs),
            "turns_90": sum(r.turns.get("R", 0) + r.turns.get("L", 0) for r in self.runs),
            "turns_180": sum(r.turns.get("B", 0) for r in self.runs),
            "phantom_hits": self.phantom_total,
            "known%": round(self.final_known, 1),
            "wrong_walls": self.wrong_walls,
            "missing_walls": self.missing_walls,
        }


class Simulator:
    def __init__(self, maze, motion, cfg: SimConfig):
        self.maze = maze
        self.motion = motion
        self.cfg = cfg
        self.n = maze.n

    # --------------------------------------------------------------- 1 手決定
    def choose_rel(self, st: MouseState, mode: str):
        cfg = self.cfg
        if mode == "goal":
            seeds = self.maze.goals
        elif mode == "return":
            seeds = [self.maze.start]
        else:
            seeds = seeds_unvisited(st, self.maze)
            if not seeds:
                return None, None

        if cfg.strategy == "gain" and mode == "explore":
            tgt = self.pick_gain_target(st)
            if tgt is None:
                return None, None
            fld = dijkstra_field(st, [tgt], self.motion)
            return self._descend(st, fld, field_at(st, fld, st.x, st.y)), tgt
        if cfg.strategy == "gain":            # ゴールモードは時間コスト最短
            fld = dijkstra_field(st, self.maze.goals, self.motion)
            cur = field_at(st, fld, st.x, st.y)
            if cur == float("inf"):
                return None, None
            return self._descend(st, fld, cur), self.maze.goals

        if cfg.strategy in ("fw", "gain"):
            fld = bfs_field(st, seeds)
            if fld[st.y][st.x] == 0xFFFF:
                return None, None
            return self._descend(st, fld, fld[st.y][st.x]), seeds

        if cfg.strategy == "dijkstra":
            fld = dijkstra_field(st, seeds, self.motion)
            cur = field_at(st, fld, st.x, st.y)
            if cur == float("inf"):
                return None, None
            return self._descend(st, fld, cur), seeds

        raise ValueError("unknown strategy: %s" % cfg.strategy)

    def pick_gain_target(self, st: MouseState):
        """未探索セルのうち「そこを割くと今後のゴール時間が一番短くなる」ものを選ぶ。"""
        n = self.n
        goal_field = dijkstra_field(st, self.maze.goals, self.motion)
        cands = []
        for y in range(n):
            for x in range(n):
                if st.known_cell[y][x]:
                    continue
                # 既知セルから既知開口で直接进入できるか
                ok = False
                for d in range(4):
                    px, py = add(x, y, d)
                    if 0 <= px < n and 0 <= py < n and st.known_cell[py][px] \
                            and st.known_open(px, py, d):
                        ok = True
                        break
                if ok:
                    cands.append((x, y))
        if not cands:
            return None
        scored = []
        for (tx, ty) in cands:
            reach = field_at(st, dijkstra_field(st, [(tx, ty)], self.motion), st.x, st.y)
            if reach == float("inf"):
                continue
            # 目標セルを割った後に得られる地図でゴールまでにかかる時間の期待値
            after = field_at(st, goal_field, tx, ty)
            scored.append((reach + self.cfg.goal_weight * after, tx, ty))
        if not scored:
            return None
        scored.sort()
        return (scored[0][1], scored[0][2])

    def _descend(self, st, fld, threshold):
        """距離場を 1 つ下る方向を、直進→右→左→後退 の優先順で選ぶ。"""
        best, best_cost = None, None
        for rel in REL_PRIORITY:
            ad = abs_dir(st.d, rel)
            if st.map[st.y][st.x] & DIR_WALL[ad]:
                continue
            nx, ny = add(st.x, st.y, ad)
            if not (0 <= nx < self.n and 0 <= ny < self.n):
                continue
            c = fld[ny][nx]
            if isinstance(c, int) and c == 0xFFFF:
                continue
            if not (c < threshold):
                continue
            if best_cost is None or c < best_cost:
                best, best_cost = rel, c
        return best

    # --------------------------------------------------------------- 本体
    def run(self) -> SimResult:
        maze, cfg = self.maze, self.cfg
        res = SimResult(maze_name=maze.name, config=cfg, motion_desc=self.motion.describe())
        st = MouseState.create(maze, unknown_walls=cfg.unknown_walls, start_dir=DIR_N)
        clock = 0.0
        pending = 0          # 連続直線を台形積分するために滞留
        v_now = 0.0

        def flush(v_end_needed=0.0):
            nonlocal clock, pending, v_now
            if pending <= 0:
                return
            t, v_end = self.motion.straight_time(pending, v0=v_now, v_end_needed=v_end_needed)
            clock += t
            v_now = v_end
            pending = 0

        def log(action, note=""):
            res.timeline.append({
                "t": round(clock, 4), "run": len(res.runs) + 1, "mode": mode,
                "x": st.x, "y": st.y, "dir": st.d, "action": action,
                "known_cells": sum(1 for row in st.known_cell for v in row if v),
                "known%": round(st.known_ratio(maze), 2), "note": note,
            })

        def observe_and_map():
            obs = sense(maze, st.x, st.y, st.d, base_sensing(cfg.sensing))
            write_map(st, maze, obs, propagate=cfg.propagate, map_write=cfg.map_write)
            if cfg.sensing.endswith("_ahead"):
                update_ahead(st, maze)

        for run_idx in range(cfg.max_runs):
            # ラン 1 = SEARCH_MODE_FULL（ゴールで止めない）/ ラン 2〜 = GOAL モード
            mode = "explore" if run_idx == 0 else (
                "return" if (st.x, st.y) in maze.goals else "goal")
            known_before = sum(1 for row in st.known_cell for v in row if v)
            run_cells, run_turns, run_phantom = 0, {"R": 0, "L": 0, "B": 0}, 0
            reached, reason = False, "step-limit"
            pending, v_now = 0, 0.0
            st.x, st.y, st.d = maze.start[0], maze.start[1], DIR_N

            stuck = 0
            last_phantom_pos = None
            for _ in range(cfg.max_steps_per_run):
                observe_and_map()
                known_now = sum(1 for row in st.known_cell for v in row if v)
                if res.t_full_knowledge is None and known_now == self.n * self.n:
                    res.t_full_knowledge = clock

                if (st.x, st.y) in maze.goals:
                    if res.t_first_goal is None:
                        res.t_first_goal = clock      # 最初のゴール到達時刻
                    if cfg.end_run_at_goal == "always":
                        reached, reason = True, "goal"
                        break                          # ルール上ゴール停止を再現する場合のみ
                    if mode == "goal" and cfg.end_run_at_goal == "goal-mode":
                        reached, reason = True, "goal"
                        break
                    if not res._touched_goal:
                        res._touched_goal = True

                # ゴールへ切り替えるタイミング
                if mode == "explore":
                    if known_now == self.n * self.n:
                        mode = "goal"
                    elif cfg.goal_policy == "dash_when_reachable" and goal_reachable(st, maze):
                        mode = "goal"

                rel, _ = self.choose_rel(st, mode)
                if rel is None and mode == "explore":
                    mode = "goal"
                    rel, _ = self.choose_rel(st, mode)
                if rel is None:
                    reason = "no-move"
                    break

                ad = abs_dir(st.d, rel)
                nx, ny = add(st.x, st.y, ad)
                # 認識上は開口、実際は壁 → 接近して判明（減速＋再計画）
                if not maze.open(st.x, st.y, ad):
                    st.map[st.y][st.x] |= DIR_WALL[ad]
                    if 0 <= nx < self.n and 0 <= ny < self.n:
                        st.map[ny][nx] |= OPPOSITE_BIT[ad]   # 隣接側の反対壁も立てる
                    res.phantom_total += 1
                    run_phantom += 1
                    stuck = stuck + 1 if last_phantom_pos == (st.x, st.y) else 1
                    last_phantom_pos = (st.x, st.y)
                    if stuck > 8:
                        reason = "stuck(sensing/map 由来で進行不能)"
                        log("X", note="stuck")
                        break
                    flush(v_end_needed=0.0)
                    clock += cfg.phantom_penalty
                    log(REL_NAME[rel], note="phantom-block")
                    if reason.startswith("stuck"):
                        break
                    continue

                if rel == 0:
                    pending += 1
                    st.x, st.y = nx, ny
                    run_cells += 1
                    log("F")
                else:
                    flush(v_end_needed=self.motion.turn_in_speed)
                    t, _v = self.motion.turn_time(abs(REL_ANGLE[rel]))
                    clock += t
                    st.d = ad
                    run_turns[REL_NAME[rel]] += 1
                    log(REL_NAME[rel])
                    observe_and_map()          # 旋回直後も壁を読み直す
                res.steps += 1
            else:
                reason = "step-limit"

            flush(v_end_needed=0.0)
            known_after = sum(1 for row in st.known_cell for v in row if v)
            res.runs.append(RunResult(
                index=run_idx + 1, mode=mode, time=clock, cells=run_cells,
                cells_new=known_after - known_before, turns=run_turns,
                reached_goal=reached, known_after=st.known_ratio(maze),
                phantom=run_phantom, end_reason=reason))
            if reached and res.t_first_goal is None:
                res.t_first_goal = clock
            res.total_time = clock
            if known_after == self.n * self.n and reached:
                break

        res.visit_order = dict(st.visit_order)
        res.final_known = st.known_ratio(maze)
        res.wrong_walls = st.wrong_wall_count(maze)
        res.missing_walls = st.missing_wall_count(maze)
        return res


OPPOSITE_BIT = [0x02, 0x01, 0x08, 0x04]     # N,E,S,W の反対側の壁ビット


def base_sensing(sensing: str) -> str:
    return sensing.replace("_ahead", "") if sensing != "ideal4_ahead" else "ideal4"


def update_ahead(st: MouseState, maze):
    """前方セルの側壁を読む（上位機の定番センサ構成）。進行方向先セルに書き込む。"""
    ax, ay = add(st.x, st.y, st.d)
    if not (0 <= ax < st.n and 0 <= ay < st.n):
        return
    for side in (turn_left(st.d), turn_right(st.d)):
        if maze.wall(ax, ay, side):
            st.map[ay][ax] |= DIR_WALL[side]


def back_bit(d):
    return OPPOSITE_BIT[d]
