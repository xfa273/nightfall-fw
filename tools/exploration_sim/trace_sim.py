"""trace_sim.py - 可視化用のステップ毎トレース付き実行（現状 / 改善版のプリセット付き）.

`sim.Simulator` の目標選択・時間モデルはそのままに、センサ観測マスクと
地図更新方針を切り替えられるようにした版（`viewer_patch`）。
各ステップで「地図の状態」と「地図差分」を記録する。
"""

from __future__ import annotations

from dataclasses import dataclass, field

from firmware_model import (REL_ANGLE, REL_NAME, MouseState, add, abs_dir,
                            goal_reachable, seeds_unvisited, field_at,
                            bfs_field, dijkstra_field)
from maze import DIR_N, DIR_WALL
import viewer_patch as vp
from sim import Simulator, SimConfig

PRESETS = {
    "current": dict(
        label="現状ソフト",
        desc="sensor fw 変換 (wall_info >> dir) / 現マス上書き / 壁なしも伝搬 / 最近傍未探索を目標",
        sensing="fw", map_write="overwrite", propagate="fw", strategy="fw"),
    "sensor_only": dict(
        label="改善途中: センタ変換のみ修正",
        desc="(wall_info >> dir) を正しい絶対方位変換に替えただけ / 現マス上書きと壁なし伝搬はそのまま",
        sensing="fw_fixed", map_write="overwrite", propagate="fw", strategy="fw"),
    "write_only": dict(
        label="改善途中: 地図更新のみ修正",
        desc="現マスを OR 蓄積にし、観測した方位だけ伝搬（センタ変換は現状のまま）",
        sensing="fw_masked", map_write="accumulate", propagate="aware", strategy="fw"),
    "fixed_map": dict(
        label="改善A: センタ・地図更新のみ修正",
        desc="絶対方位への変換を修正 + 観測した方位だけ伝搬 + 壁は OR 蓄積 / 目標選択は現状のまま",
        sensing="fw_fixed", map_write="accumulate", propagate="aware", strategy="fw"),
    "fixed_gain": dict(
        label="改善B: ＋目標選択（ゴール利得）",
        desc="改善A に加え、割った後のゴール所要時間で探索目標を選択",
        sensing="fw_fixed", map_write="accumulate", propagate="aware", strategy="gain"),
    "topmouse": dict(
        label="参考: 上位機相当",
        desc="4 センサ＋先読み / OR 蓄積 / 利得ベース目標選択",
        sensing="ideal4_ahead", map_write="accumulate", propagate="aware", strategy="gain"),
}


@dataclass
class Trace:
    maze_name: str
    preset: str
    label: str
    desc: str
    n: int
    walls: list
    start: list
    goals: list
    frames: list = field(default_factory=list)
    meta: dict = field(default_factory=dict)


class TracedSim(Simulator):
    """Simulator と同じ目標選択で、観測マスク対応・トレース付きで走らせる。"""

    def choose_rel_fw(self, st, mode):
        """機体の make_smap()/make_route() と同じ「最近傍未探索」で 1 手を選ぶ。

        目標選択アルゴリズムが候補を出せなくても、機体なら make_smap() が未探索セル
        を全部シードにして BFS するので「目標が無い」にはならない。ラン打ち切りは
        地図上の経路が無いときだけ、という条件を再現するための逃げ道にする。
        """
        if mode == "goal":
            seeds = self.maze.goals
        elif mode == "return":
            seeds = [self.maze.start]
        else:
            seeds = seeds_unvisited(st, self.maze)
            if not seeds:
                return None, None
        fld = bfs_field(st, seeds)
        if fld[st.y][st.x] == 0xFFFF:
            return None, None
        return self._descend(st, fld, fld[st.y][st.x]), seeds

    def observe(self, st, maze, cfg):
        bits, mask = vp.sense_masked(maze, st.x, st.y, st.d, cfg["sensing"])
        vp.write_map_masked(st, maze, bits, mask,
                            propagate=cfg["propagate"], map_write=cfg["map_write"])
        if cfg["sensing"].endswith("_ahead"):
            ax, ay = add(st.x, st.y, st.d)
            if 0 <= ax < st.n and 0 <= ay < st.n:
                from maze import turn_left, turn_right
                for side in (turn_left(st.d), turn_right(st.d)):
                    if maze.wall(ax, ay, side):
                        st.map[ay][ax] |= DIR_WALL[side]

    def run_traced(self, maze, cfg, max_frames=8000):
        """機体と同じ流れで走らせる。

        ラン 1  = SEARCH_MODE_FULL: ゴールで止まらず、未探索へ向かい続ける
                   （未探索へ向かう経路が無くならない限り search_end にならない）
        ラン 2~= SEARCH_MODE_GOAL : 居る場所がゴール側ならスタートを、それ以外は
                   ゴールを目的地にする（g_goal_is_start 相当）
        ランの間は係員が機体を開始位置へ戻す（a='P' のフレームとして入れる。
        迷路の壁を無視したテレポートに見えないよう、ここで機体を非表示にする）
        """
        mo = self.motion
        key = self.preset_key
        tr = Trace(maze_name=maze.name, preset=key, label=PRESETS[key]["label"],
                   desc=PRESETS[key]["desc"], n=maze.n,
                   walls=[row[:] for row in maze.walls],
                   start=list(maze.start), goals=[list(g) for g in maze.goals])
        st = MouseState.create(maze, unknown_walls=cfg.get("unknown_walls", "open"),
                               start_dir=DIR_N)
        prev_map = [row[:] for row in st.map]
        clock, pending, v_now = 0.0, 0, 0.0
        phantom = 0
        t_first_goal = None
        t_run_end = None
        full_known_t = None
        frames = []
        end_at_goal = cfg.get("end_run_at_goal", "never")   # never = 機体同等
        max_runs = int(cfg.get("runs", cfg.get("max_runs", 1)))   # 既定は 1 本のみ
        gap = float(cfg.get("run_gap_s", 0.6))

        def diff_and_log(action, mode_code, note=""):
            w, m = vp.map_diff_bits(st, maze)
            d = [[yy * st.n + xx, st.map[yy][xx]] for yy in range(st.n) for xx in range(st.n)
                 if st.map[yy][xx] != prev_map[yy][xx]]
            for yy in range(st.n):
                for xx in range(st.n):
                    prev_map[yy][xx] = st.map[yy][xx]
            frames.append({"t": round(clock, 3), "x": st.x, "y": st.y, "h": st.d,
                           "a": action, "k": sum(1 for r in st.known_cell for c in r if c),
                           "w": w, "s": m, "r": run_idx, "m": mode_code, "n": note, "c": d})

        def flush(v_end=0.0):
            nonlocal clock, pending, v_now
            if pending <= 0:
                return
            t, ve = mo.straight_time(pending, v0=v_now, v_end_needed=v_end)
            clock += t
            v_now = ve
            pending = 0

        run_idx = 1
        runlog = []
        end_reason = "step-limit"
        while run_idx <= max_runs and len(frames) < max_frames:
            if run_idx > 1:                    # 係員が機体を開始位置まで運ぶ
                fx, fy = carry_from            # 停止位置（運搬の始点）
                clock += gap
                tx, ty = maze.start
                frames.append({"t": round(clock, 3), "x": -1, "y": -1, "h": st.d,
                               "a": "C", "k": sum(1 for r in st.known_cell for c in r if c),
                               "w": 0, "s": 0, "r": run_idx, "m": 0, "n": "carry",
                               "c": [], "from": [fx, fy], "to": [tx, ty]})
                st.x, st.y, st.d = tx, ty, DIR_N
                pending, v_now = 0, 0.0
                diff_and_log("P", 0, "pickup")
            at_goal_now = (st.x, st.y) in maze.goals
            mode = "return" if (run_idx > 1 and at_goal_now) else (
                "goal" if run_idx > 1 else "explore")
            mode_code = 0 if mode == "explore" else (2 if mode == "return" else 1)
            stuck, last_pos = 0, None
            end_reason = "exhausted"          # 未探索へ向かう経路が無い＝機体の search_end
            for _ in range(int(cfg.get("max_steps_per_run", 3000))):
                self.observe(st, maze, cfg)
                known = sum(1 for r in st.known_cell for c in r if c)
                if full_known_t is None and known == st.n * st.n:
                    full_known_t = clock
                if (st.x, st.y) in maze.goals:
                    if t_first_goal is None:
                        t_first_goal = clock
                    if end_at_goal == "always":
                        diff_and_log("G", mode_code, "goal")
                        run_idx += 1
                        break
                    if not frames or frames[-1]["n"] != "goal":
                        diff_and_log("G", mode_code, "goal")
                rel, _ = self.choose_rel(st, mode)
                if rel is None and mode == "explore":
                    rel, _ = self.choose_rel_fw(st, mode)   # 機体同等の最近傍未探索へ退避
                if rel is None:
                    end_reason = "no-route"    # 経路無し = 機体の search_end と同じ条件
                    break
                ad = abs_dir(st.d, rel)
                if not maze.open(st.x, st.y, ad):
                    st.map[st.y][st.x] |= DIR_WALL[ad]
                    nx, ny = add(st.x, st.y, ad)
                    if 0 <= nx < st.n and 0 <= ny < st.n:
                        st.map[ny][nx] |= vp.OPP_BIT[ad]
                    flush()
                    clock += float(cfg.get("phantom_penalty", 1.0))
                    diff_and_log(REL_NAME[rel], mode_code, "phantom")
                    phantom += 1
                    stuck = stuck + 1 if last_pos == (st.x, st.y) else 1
                    last_pos = (st.x, st.y)
                    if stuck > 8:
                        diff_and_log("X", mode_code, "stuck")
                        end_reason = "stuck"
                        run_idx = max_runs + 1
                        break
                    continue
                if rel == 0:
                    pending += 1
                    st.x, st.y = add(st.x, st.y, ad)
                    diff_and_log("F", mode_code)
                else:
                    flush(mo.turn_in_speed)
                    t, _v = mo.turn_time(abs(REL_ANGLE[rel]))
                    clock += t
                    st.d = ad
                    diff_and_log(REL_NAME[rel], mode_code)
                    self.observe(st, maze, cfg)
                    if len(frames) >= max_frames:
                        end_reason = "frame-limit"
                        break
            else:
                end_reason = "step-limit"
            flush()
            t_run_end = clock
            carry_from = (st.x, st.y)
            runlog.append({"run": run_idx, "reason": end_reason, "t_end": round(clock, 2),
                           "at": [st.x, st.y], "known_pct": round(st.known_ratio(maze), 1),
                           "mode": mode})
            if run_idx < max_runs and len(frames) < max_frames:
                diff_and_log("S", mode_code, "stop")   # その場で停止（drive_stop 相当）
            if run_idx <= max_runs:
                run_idx += 1
        tr.frames = frames
        w, m = vp.map_diff_bits(st, maze)
        tr.meta = {
            "t_total": round(clock, 2), "t_first_goal": (round(t_first_goal, 2) if t_first_goal
                                                         is not None else None),
            "t_run_end": (round(t_run_end, 2) if t_run_end else None),
            "t_full_known": (round(full_known_t, 2) if full_known_t else None),
            "phantom": phantom, "known_pct": round(st.known_ratio(maze), 1),
            "wrong_walls": w, "missing_walls": m, "frames": len(frames),
            "end_run_at_goal": end_at_goal,
            "config": {k: cfg.get(k) for k in ("sensing", "map_write", "propagate", "strategy")},
            "motion": mo.describe(), "runs": runlog,
        }
        return tr


def run_preset(maze, motion, key, overrides=None):
    cfg = dict(PRESETS[key])
    cfg.update(overrides or {})
    s = TracedSim(maze, motion, SimConfig(strategy=cfg["strategy"], sensing=cfg["sensing"]))
    s.preset_key = key
    return s.run_traced(maze, cfg)
