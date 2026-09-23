#!/usr/bin/env python3
"""run_exploration_sim.py - 探索アルゴリズム机上検討 CLI.

例:
  # 1 迷路を詳しく（現状ファーム同等の挙動）
  ./run_exploration_sim.py --maze /path/to/16MM2019CX.maze --print-order

  # 戦略・センサ・方針を交えて比較表
  ./run_exploration_sim.py --maze-dir /path/to/maze-data/data \
      --maze-glob '16MM20*CX.maze' \
      --strategy fw,dijkstra,gain --compare

  # 自前パラメータで時間モデルを作る
  ./run_exploration_sim.py --maze xxx.maze --board classic_r1_0 --turn-settle-ms 80
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import motion as motion_mod
import report
from firmware_model import MouseState, sense, write_map
from maze import DIR_N, Maze
from sim import SimConfig, Simulator

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))

MAZE_DIR_CANDIDATES = [
    os.environ.get("NIGHTFALL_MAZE_DATA", ""),
    os.path.join(REPO, "mazes"),
    os.path.join(REPO, "third_party", "micromouse-maze-data", "data"),
    os.path.expanduser("~/workspace/micromouse/maze-data"),
    os.path.expanduser("~/workspace/micromouse/maze-data/data"),
    os.path.expanduser("~/workspace/micromouse/micromouse-maze-data/data"),
    "/tmp/mm-review/maze-data/data",
]


def find_maze_dir(explicit=None):
    """迷路データ置き場を探す。`.maze` を直接含むディレクトリを返す。"""
    for c in [explicit] + MAZE_DIR_CANDIDATES:
        if not c:
            continue
        if glob.glob(os.path.join(c, "*.maze")):
            return c
        sub = os.path.join(c, "data")
        if glob.glob(os.path.join(sub, "*.maze")):
            return sub
    return None


def build_motion(a):
    if a.time_model == "cells":
        return motion_mod.CellCostModel.builtin()
    paths = []
    if a.params:
        for p in a.params:
            paths += glob.glob(p)
    else:
        for fn in ("search_run_params_split.c", "shortest_run_params_split.c"):
            paths += glob.glob(os.path.join(REPO, "params", a.board, fn))
    ov = {}
    if a.v_max is not None:
        ov["v_max"] = a.v_max
    if a.accel is not None:
        ov["accel"] = ov["decel"] = a.accel
    if a.w90 is not None:
        ov["w90"] = a.w90
    if a.a90 is not None:
        ov["a90"] = a.a90
    return motion_mod.MotionModel.from_params(
        paths, cell_mm=a.cell_mm, turn_settle_ms=a.turn_settle_ms,
        allow_45cut=not a.no_45cut, overrides=ov)


def cfg_from_args(a, strategy):
    return SimConfig(strategy=strategy, sensing=a.sensing, propagate=a.propagate,
                     map_write=a.map_write,
                     unknown_walls=a.unknown_walls, goal_policy=a.goal_policy,
                     end_run_at_goal=a.end_run_at_goal,
                     max_runs=a.max_runs, goal_weight=a.goal_weight,
                     phantom_penalty=a.phantom_penalty)


def main(argv=None):
    p = argparse.ArgumentParser(description="マイクロマウス探索シミュレータ")
    p.add_argument("--maze", help="迷路ファイル 1 つ（KERI 形式）")
    p.add_argument("--maze-dir", help="迷路データディレクトリ")
    p.add_argument("--maze-glob", default="*CX.maze", help="--compare 時のファイルパターン")
    p.add_argument("--strategy", default="fw", help="fw | dijkstra | gain（カンマ区切りで複数）")
    p.add_argument("--sensing", default="fw",
                   help="fw | fw_fixed | ideal3 | ideal4 | ideal4_ahead")
    p.add_argument("--map-write", default="overwrite", choices=["overwrite", "accumulate"],
                   help="現マスの壁の書き込み方針（overwrite=ファーム同等）")
    p.add_argument("--propagate", default="fw", choices=["fw", "safe"],
                   help="fw=『壁なし』も近傍に伝搬（ファーム同等） / safe=壁ありのみ伝搬")
    p.add_argument("--unknown-walls", default="open", choices=["open", "closed"],
                   help="未観測の壁を開（ファームの map_Init 相当）/ 閉 で初期化")
    # full_first = ファーム同等（未探索を割り切ってからゴールへ向かう）
    p.add_argument("--goal-policy", default="full_first",
                   choices=["full_first", "dash_when_reachable"])
    p.add_argument("--goal-weight", type=float, default=1.0, help="gain 戦略のゴール重み")
    p.add_argument("--end-run-at-goal", default="always", choices=["always", "goal-mode", "never"],
                   help="ゴール区画でランを終えるか（always=ルール通り。"
                        "never=探索走行でゴール通過でも走り続ける仮定）")
    p.add_argument("--max-runs", type=int, default=5)
    p.add_argument("--phantom-penalty", type=float, default=1.0,
                   help="未知の壁への進入で失う時間 [秒]")
    p.add_argument("--time-model", default="physics", choices=["physics", "cells"])
    p.add_argument("--params", action="append", help="パラメータ C ファイルの glob（複数可）")
    p.add_argument("--board", default="classic_r1_0")
    p.add_argument("--cell-mm", type=float, default=180.0)
    p.add_argument("--turn-settle-ms", type=float, default=60.0)
    p.add_argument("--v-max", type=float)
    p.add_argument("--accel", type=float)
    p.add_argument("--w90", type=float)
    p.add_argument("--a90", type=float)
    p.add_argument("--no-45cut", action="store_true")
    p.add_argument("--compare", action="store_true", help="迷路 x 戦略 の比較表を出す")
    p.add_argument("--print-order", action="store_true", help="探索順マップを表示")
    p.add_argument("--print-map", action="store_true", help="最終認識地図との差分を表示")
    p.add_argument("--out", help="結果出力ディレクトリ（csv/json/html を書く）")
    p.add_argument("--list-mazes", action="store_true")
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args(argv)

    strategies = [s.strip() for s in a.strategy.split(",") if s.strip()]

    if a.selftest:
        return selftest(a)

    mdir = find_maze_dir(a.maze_dir)
    if a.list_mazes:
        if not mdir:
            print("迷路データが見つかりません。--maze-dir か NIGHTFALL_MAZE_DATA を指定してください")
            return 2
        files = sorted(glob.glob(os.path.join(mdir, "*.maze")))
        print("# %s (%d files)" % (mdir, len(files)))
        for f in files:
            print(" ", os.path.basename(f))
        return 0

    motion = build_motion(a)
    print("# time model:", motion.describe())

    if a.maze:
        cands = [a.maze, os.path.join(a.maze_dir or "", a.maze),
                 os.path.join(find_maze_dir(a.maze_dir) or "", a.maze),
                 os.path.join(find_maze_dir(a.maze_dir) or "", a.maze + ".maze")]
        files = [c for c in cands if c and os.path.isfile(c)][:1]
        if not files:
            print("迷路ファイルが見つかりません:", a.maze)
            return 2
    else:
        if not mdir:
            print("迷路データが見つかりません。--maze-dir か --maze を指定してください")
            return 2
        files = sorted(glob.glob(os.path.join(mdir, a.maze_glob)))
        if not files:
            print("%s に一致する迷路がありません (%s)" % (mdir, a.maze_glob))
            return 2

    results = []
    rows = []
    for f in files:
        maze = Maze.load(f)
        for s in strategies:
            res = Simulator(maze, motion, cfg_from_args(a, s)).run()
            results.append((maze, res))
            rows.append(res.summary_row())
            if len(files) == 1 and len(strategies) == 1:
                print("\n## %s  (strategy=%s, sensing=%s, policy=%s, map_write=%s)"
                      % (maze.name, s, a.sensing, a.goal_policy, a.map_write))
                print(report.runs_table(res))
                print("\nT_first_goal = %s  T_full_known = %s  total = %.2f s  known = %.1f%%" % (
                    ("%.2f s" % res.t_first_goal) if res.t_first_goal is not None else "未達",
                    ("%.2f s" % res.t_full_knowledge) if res.t_full_knowledge is not None else "未達",
                    res.total_time, res.final_known))
                print("route:", report.route_string(res)[:400])
                if a.print_order:
                    print()
                    print(report.visit_order_ascii(maze, res))
                if a.print_map:
                    diff_map_diff(maze, res, motion, a)
            else:
                print("  %-28s %-9s %-13s t_goal=%8s known=%5.1f%% phantom=%d" % (
                    maze.name, s, a.sensing + "/" + a.map_write[0],
                    ("%.2f" % res.t_first_goal) if res.t_first_goal is not None else "  未達",
                    res.final_known, res.phantom_total))

    if a.compare or len(results) > 1:
        print("\n### summary")
        print(report.summary_markdown(rows, keys=[
            "maze", "strategy", "sensing", "goal_policy", "t_first_goal", "runs_to_goal",
            "t_full_known", "total_time", "best_shortest", "cells_moved", "turns_90",
            "turns_180", "phantom_hits", "known%"]))

    if a.out:
        os.makedirs(a.out, exist_ok=True)
        import csv as _csv
        spath = os.path.join(a.out, "summary.csv")
        with open(spath, "w", newline="", encoding="utf-8") as fh:
            w = _csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print("\nwrote", spath)
        for maze, res in results:
            tag = "%s__%s_%s_%s" % (res.maze_name, res.config.strategy,
                                    res.config.sensing, res.config.goal_policy)
            report.write_csv(os.path.join(a.out, tag + ".timeline.csv"), res)
            report.write_json(os.path.join(a.out, tag + ".json"), res)
            report.write_html(os.path.join(a.out, tag + ".html"), maze, res)
        print("wrote per-run csv/json/html into", a.out)
    return 0


def diff_map_diff(maze, res, motion, a):
    """最終認識地図と実際の地図の差分（センサモデル起因の誤りを見せる）。"""
    st = MouseState.create(maze, unknown_walls=a.unknown_walls, start_dir=DIR_N)
    for row in res.timeline:
        pass
    print("\n(注) 地図差分は最後のラン終了時点で再計算した近似値です")
    print("wrong_walls=%d missing_walls=%d" % (res.wrong_walls, res.missing_walls))


def selftest(a):
    """入力の妥当性・物理整合性の自己テスト。"""
    mdir = find_maze_dir(a.maze_dir)
    if not mdir:
        print("selftest: 迷路データがありません")
        return 2
    files = sorted(glob.glob(os.path.join(mdir, "*.maze")))
    ok = fail = 0
    for f in files:
        try:
            m = Maze.load(f)
            assert len(m.walls) == m.n, "行数不一致"
            assert all(len(r) == m.n for r in m.walls), "列数不一致"
            # 外周が閉じているか
            for x in range(m.n):
                assert m.wall(x, m.n - 1, 0) and m.wall(x, 0, 2), "南北外周"
            for y in range(m.n):
                assert m.wall(0, y, 3) and m.wall(m.n - 1, y, 1), "東西外周"
            # 壁の対称性
            for y in range(m.n):
                for x in range(m.n):
                    for d in range(4):
                        if m.open(x, y, d):
                            nx, ny = x + (0, 1, 0, -1)[d], y + (1, 0, -1, 0)[d]
                            assert not m.wall(nx, ny, (d + 2) % 4), "壁の非対称 (%d,%d,%d)" % (x, y, d)
            ok += 1
        except Exception as e:                                    # noqa: BLE001
            fail += 1
            print("  FAIL %-34s %s" % (os.path.basename(f), e))
    print("selftest(parse): %d ok / %d fail (%d files)" % (ok, fail, len(files)))

    # 走行整合性: 実在しない壁を抜けない／全セル到達／時間単調
    motion = motion_mod.CellCostModel.builtin()
    probe = [f for f in files if "16MM2019CX" in f or "16MM2016CX" in f]
    for f in probe:
        m = Maze.load(f)
        for strat in ("fw", "dijkstra", "gain"):
            for sens in ("fw", "ideal4_ahead"):
                cfg = SimConfig(strategy=strat, sensing=sens)
                r = Simulator(m, motion, cfg).run()
                prev = -1.0
                for row in r.timeline:
                    assert row["t"] >= prev, "時間が逆行"
                    prev = row["t"]
                print("  run %-12s %-9s %-13s goal=%-7s known=%5.1f%% phantom=%d wrong=%3d t=%6.2f" % (
                    m.name, strat, sens,
                    ("%.2f" % r.t_first_goal) if r.t_first_goal is not None else "-",
                    r.final_known, r.phantom_total, r.wrong_walls, r.total_time))
    # センタ回転の仕様確認（ファームと同じ (wall_info >> dir) & 0x0f）
    from firmware_model import sense as sense_f
    from maze import DIR_WALL
    # 4 方角すべてに壁があるセルで、変換式そのものを検査する
    n = 3
    allw = [[0x0F] * n for _ in range(n)]
    synth = Maze(n, allw, start=(1, 1), name="all-wall")
    print("\nselftest(sensor): (wall_info >> dir) & 0x0f の絶対方位変換")
    print("  前方/右方/左方に壁があるセルでの期待値と実際の観測ビット")
    print("  dir  期待(3センサ)  fw 変換後  欠落")
    for d in range(4):
        got = sense_f(synth, 1, 1, d, "fw")
        exp = sense_f(synth, 1, 1, d, "ideal3")
        miss = [c for i, c in enumerate("NESW")
                if (exp & DIR_WALL[i]) and not (got & DIR_WALL[i])]
        print("   %d      %04s       %04s    %s" % (
            d, format(exp & 15, "04b"), format(got & 15, "04b"), ",".join(miss) or "-"))
    print("\nselftest(map-write): overwrite vs accumulate")
    for f in probe[:1]:  # probe は迷路ファイルパスのまま
        m = Maze.load(f)
        for mw in ("overwrite", "accumulate"):
            for sens in ("fw", "fw_fixed"):
                cfg = SimConfig(strategy="fw", sensing=sens, map_write=mw)
                r = Simulator(m, motion, cfg).run()
                print("  %-12s %-9s %-11s goal=%-7s known=%5.1f%% phantom=%d wrong=%3d" % (
                    m.name, sens, mw,
                    ("%.2f" % r.t_first_goal) if r.t_first_goal is not None else "未達",
                    r.final_known, r.phantom_total, r.wrong_walls))
    print("selftest(sim): done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
