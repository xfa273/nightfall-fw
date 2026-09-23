#!/usr/bin/env python3
"""sweep.py - 迷路 x 戦略 x センサ x マップ運用 のマトリクス比較.

    ./sweep.py --maze-glob '16MM20[12][0-9]CX.maze' \
        --strategy fw,dijkstra,gain --sensing fw,fw_fixed,ideal4_ahead \
        --map-write overwrite,accumulate

結果は Markdown 表（標準出力）と `--out` に CSV。
`--pivot t_full_known` で指標を指定すると「行=設定 / 列=迷路」の表にもする。
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import motion as motion_mod
import report
from maze import Maze
from sim import SimConfig, Simulator
import run_exploration_sim as cli


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--maze-dir")
    p.add_argument("--maze-glob", default="16MM20[12][0-9]CX.maze")
    p.add_argument("--mazes", default="", help="カンマ区切りで迷路名を直接指定")
    p.add_argument("--strategy", default="fw,dijkstra,gain")
    p.add_argument("--sensing", default="fw,fw_fixed,ideal4_ahead")
    p.add_argument("--map-write", default="overwrite")
    p.add_argument("--propagate", default="fw")
    p.add_argument("--unknown-walls", default="open")
    p.add_argument("--goal-policy", default="full_first")
    p.add_argument("--board", default="classic_r1_0")
    p.add_argument("--turn-settle-ms", type=float, default=60.0)
    p.add_argument("--pivot", default="", help="例: t_full_known / t_first_goal / phantom_hits")
    p.add_argument("--out")
    a = p.parse_args()

    mdir = cli.find_maze_dir(a.maze_dir)
    if not mdir:
        print("迷路データが見つかりません (--maze-dir / NIGHTFALL_MAZE_DATA)")
        return 2
    names = [s.strip() for s in a.mazes.split(",") if s.strip()]
    if names:
        files = [f if os.path.sep in f else os.path.join(mdir, f + ".maze") for f in names]
    else:
        files = sorted(glob.glob(os.path.join(mdir, a.maze_glob)))
    if not files:
        print("迷路がみつかりません:", mdir, a.maze_glob)
        return 2

    ns = argparse.Namespace(**{
        **vars(a),
        "time_model": "physics", "params": None, "cell_mm": 180.0,
        "v_max": None, "accel": None, "w90": None, "a90": None, "no_45cut": False,
    })
    motion = cli.build_motion(ns)
    print("# time model:", motion.describe())

    rows = []
    for f in files:
        maze = Maze.load(f)
        for strat in a.strategy.split(","):
            for sens in a.sensing.split(","):
                for mw in a.map_write.split(","):
                    cfg = SimConfig(strategy=strat.strip(), sensing=sens.strip(),
                                    map_write=mw.strip(), propagate=a.propagate,
                                    unknown_walls=a.unknown_walls,
                                    goal_policy=a.goal_policy)
                    r = Simulator(maze, motion, cfg).run()
                    rows.append(r.summary_row())
                    if a.out:
                        tag = "%s__%s_%s_%s_%s" % (maze.name, strat, sens, mw, a.propagate)
                        report.write_json(os.path.join(a.out, tag + ".json"), r)
                        if strat == a.strategy.split(",")[0] and sens == a.sensing.split(",")[0] \
                                and mw == a.map_write.split(",")[0]:
                            report.write_html(os.path.join(a.out, tag + ".html"), maze, r)
                            report.write_csv(os.path.join(a.out, tag + ".timeline.csv"), r)
    keys = ["maze", "strategy", "sensing", "map_write", "t_first_goal", "t_full_known",
            "total_time", "cells_moved", "turns_90", "turns_180", "phantom_hits",
            "known%", "wrong_walls"]
    print()
    print(report.summary_markdown(rows, keys=keys))

    if a.pivot:
        mazes = sorted({r["maze"] for r in rows})
        settings = sorted({(r["strategy"], r["sensing"], r["map_write"]) for r in rows})
        print("\n### pivot: %s（行=設定 / 列=迷路）" % a.pivot)
        lines = ["| setting | " + " | ".join(mazes) + " | 平均 |",
                 "|" + "---|" * (len(mazes) + 2)]
        for s in settings:
            cells, vals = [], []
            for m in mazes:
                v = next((r.get(a.pivot) for r in rows
                          if r["maze"] == m and (r["strategy"], r["sensing"],
                                                 r["map_write"]) == s), None)
                cells.append("" if v is None else str(v))
                if isinstance(v, (int, float)):
                    vals.append(float(v))
            avg = "%.1f" % (sum(vals) / len(vals)) if vals else "-"
            lines.append("| %s | " % ("%s/%s/%s" % s) + " | ".join(cells) + " | %s |" % avg)
        print("\n".join(lines))

    if a.out:
        import csv as _csv
        os.makedirs(a.out, exist_ok=True)
        path = os.path.join(a.out, "sweep.csv")
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = _csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print("\nwrote", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
