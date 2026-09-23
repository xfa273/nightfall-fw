"""report.py - 探索シミュレーション結果の出力（ASCII / HTML+SVG / CSV / Markdown）."""

from __future__ import annotations

import csv
import json
import os

DIGITS = "123456789"
ALPHA = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
DOT = "."


def order_char(i):
    if i <= 0:
        return DOT
    if i <= len(DIGITS):
        return DIGITS[i - 1]
    idx = (i - len(DIGITS) - 1) % len(ALPHA)
    return ALPHA[idx].lower() if (i - len(DIGITS) - 1) < len(ALPHA) else "#"


def visit_order_ascii(maze, res):
    ov = {}
    for (x, y), i in res.visit_order.items():
        ov[(x, y)] = order_char(i)
    for (gx, gy) in maze.goals:
        ov[(gx, gy)] = "G"
    ov[maze.start] = "S"
    return maze.to_ascii(ov)


def route_string(res):
    s = ""
    for row in res.timeline:
        a = row["action"]
        if a == "phantom" or row["note"] == "phantom-block":
            s += "!"
        s += {"F": "F", "R": "R", "L": "L", "B": "U"}.get(a, "")
    return s


def runs_table(res):
    head = ("run", "mode", "t[ s ]", "cells", "turns R/L/U", "known%", "goal", "phantom")
    rows = []
    for r in res.runs:
        rows.append((str(r.index), r.mode, "%.2f" % r.time, str(r.cells),
                     "%d/%d/%d" % (r.turns.get("R", 0), r.turns.get("L", 0), r.turns.get("B", 0)),
                     "%.0f%%" % r.known_after, "yes" if r.reached_goal else "-", str(r.phantom)))
    w = [max(len(head[i]), *(len(r[i]) for r in rows)) if rows else len(head[i]) for i in range(len(head))]
    out = ["  ".join(h.ljust(w[i]) for i, h in enumerate(head)),
           "  ".join("-" * w[i] for i in range(len(head)))]
    out += ["  ".join(c.ljust(w[i]) for i, c in enumerate(r)) for r in rows]
    return "\n".join(out)


def summary_markdown(rows, keys=None):
    if not rows:
        return "(no rows)"
    keys = keys or list(rows[0].keys())
    lines = ["| " + " | ".join(keys) + " |",
             "|" + "|".join(["---"] * len(keys)) + "|"]
    for r in rows:
        lines.append("| " + " | ".join("" if r.get(k) is None else str(r.get(k)) for k in keys) + " |")
    return "\n".join(lines)


def write_csv(path, res):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(res.timeline[0].keys()))
        w.writeheader()
        for row in res.timeline:
            w.writerow(row)


def write_json(path, res, extra=None):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    data = {
        "maze": res.maze_name,
        "motion": res.motion_desc,
        "config": vars(res.config),
        "metrics": res.summary_row(),
        "runs": [vars(r) for r in res.runs],
        "error": res.error,
    }
    if extra:
        data.update(extra)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def write_html(path, maze, res, cell=34):
    """迷路 + 探索順 + 経路を 1 枚にまとめた自己完結 HTML。"""
    n, pad = maze.n, 28
    w = n * cell + 2 * pad
    h = n * cell + 2 * pad
    parts = []
    parts.append('<svg width="%d" height="%d" viewBox="0 0 %d %d" '
                 'xmlns="http://www.w3.org/2000/svg" font-family="monospace">' % (w, h, w, h))
    parts.append('<rect width="%d" height="%d" fill="#111"/>' % (w, h))
    # 訪問順の塗り
    max_i = max(res.visit_order.values()) if res.visit_order else 1
    for (x, y), i in res.visit_order.items():
        hue = int(240 * (1 - i / max(1.0, max_i)))
        px = pad + x * cell
        py = pad + (n - 1 - y) * cell
        parts.append('<rect x="%d" y="%d" width="%d" height="%d" fill="hsl(%d,70%%,35%%)" '
                     'opacity="0.55"/>' % (px, py, cell, cell, hue))
    # 壁
    for y in range(n):
        for x in range(n):
            px = pad + x * cell
            py = pad + (n - 1 - y) * cell
            m = maze.walls[y][x]
            if m & 0x08:
                parts.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#eee" stroke-width="2"/>'
                             % (px, py, px + cell, py))
            if m & 0x02:
                parts.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#eee" stroke-width="2"/>'
                             % (px, py + cell, px + cell, py + cell))
            if m & 0x04:
                parts.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#eee" stroke-width="2"/>'
                             % (px + cell, py, px + cell, py + cell))
            if m & 0x01:
                parts.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#eee" stroke-width="2"/>'
                             % (px, py, px, py + cell))
    # 訪問番号
    for (x, y), i in res.visit_order.items():
        px = pad + x * cell + 4
        py = pad + (n - 1 - y) * cell + cell // 2 + 4
        parts.append('<text x="%d" y="%d" font-size="10" fill="#fff">%s</text>'
                     % (px, py, order_char(i)))
    for (gx, gy) in maze.goals:
        px = pad + gx * cell + cell // 2
        py = pad + (n - 1 - gy) * cell + cell // 2 + 4
        parts.append('<text x="%d" y="%d" font-size="12" fill="#ffd166" text-anchor="middle">G</text>'
                     % (px, py))
    sx, sy = maze.start
    parts.append('<text x="%d" y="%d" font-size="12" fill="#8ef" text-anchor="middle">S</text>'
                 % (pad + sx * cell + cell // 2, pad + (n - 1 - sy) * cell + cell // 2 + 4))
    parts.append("</svg>")
    svg = "".join(parts)

    m = res.summary_row()
    metrics = "".join("<tr><th>%s</th><td>%s</td></tr>" % (k, "" if v is None else v)
                      for k, v in m.items())
    runs = "".join("<tr><td>%d</td><td>%s</td><td>%.2f</td><td>%d</td><td>%d/%d/%d</td>"
                   "<td>%.0f%%</td><td>%s</td></tr>" % (
                       r.index, r.mode, r.time, r.cells, r.turns.get("R", 0),
                       r.turns.get("L", 0), r.turns.get("B", 0), r.known_after,
                       "yes" if r.reached_goal else "-") for r in res.runs)
    html = """<!doctype html><meta charset="utf-8">
<title>exploration %(maze)s / %(strat)s</title>
<style>body{background:#181818;color:#eee;font-family:monospace;margin:24px}
table{border-collapse:collapse}td,th{border:1px solid #444;padding:2px 8px;font-size:12px}
th{background:#222;text-align:left}h2{font-size:15px}.cols{display:flex;gap:24px;align-items:flex-start}
pre{background:#111;padding:8px;border:1px solid #333;overflow:auto}</style>
<h2>%(maze)s &nbsp;|&nbsp; strategy=%(strat)s sensing=%(sens)s policy=%(pol)s</h2>
<p>%(motion)s</p>
<div class="cols"><div>%(svg)s</div><div>
<h3>metrics</h3><table>%(metrics)s</table>
<h3>runs</h3><table><tr><th>run</th><th>mode</th><th>t</th><th>cells</th>
<th>R/L/U</th><th>known%%</th><th>goal</th></tr>%(runs)s</table>
</div></div>
<h3>visit order map</h3><pre>%(order)s</pre>
<h3>route string</h3><pre>%(route)s</pre>
""" % dict(maze=res.maze_name, strat=res.config.strategy, sens=res.config.sensing,
           pol=res.config.goal_policy, motion=res.motion_desc, svg=svg,
           metrics=metrics, runs=runs, order=visit_order_ascii(maze, res),
           route=route_string(res))
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)
