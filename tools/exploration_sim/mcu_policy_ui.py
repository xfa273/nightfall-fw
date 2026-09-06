"""Export a saved bounded C trace for the two-pane simulator UI.

The C exploration is never rerun. Without --baseline, only matching Adachi
exploration is generated using the same pinned compact route oracle.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    __package__ = "tools.exploration_sim"

from .engine import Knowledge, DX, DY, simulate
from .mazes import Maze
from .mcu_policy import EngineOracle


def validate_timeline(maze, run):
    """Reject inconsistent exports instead of displaying misleading timelines."""
    events = run.get("events", [])
    if not events:
        raise ValueError("Replay contains no events")
    knowledge = Knowledge(maze["width"], maze["height"])
    previous = None
    for index, event in enumerate(events):
        x, y, heading = event["x"], event["y"], event["heading"]
        if event["step"] != index or not knowledge.inside(x, y) or heading not in range(4):
            raise ValueError("Replay step or pose is inconsistent")
        if not all(isinstance(event.get(key), (float, int)) and math.isfinite(event[key]) and event[key] >= 0
                   for key in ("t", "dt")):
            raise ValueError("Replay time must be finite and nonnegative")
        if previous:
            if not math.isclose(event["t"] - previous["t"], event["dt"], abs_tol=1e-6):
                raise ValueError("Replay time/duration is inconsistent")
            if (x, y) != (previous["x"] + DX[heading], previous["y"] + DY[heading]):
                raise ValueError("Replay movement is not one adjacent cell")
            if not knowledge.open(previous["x"], previous["y"], heading):
                raise ValueError("Replay crosses an unobserved or blocked edge")
            if event["turn"] != (heading - previous["heading"]) % 4:
                raise ValueError("Replay turn is inconsistent")
        elif (x, y) != tuple(maze["start"]) or event["t"] != 0:
            raise ValueError("Replay start is inconsistent")
        for ex, ey, direction, wall in event["changes"]:
            if not knowledge.inside(ex, ey) or direction not in range(4) or wall not in (0, 1):
                raise ValueError("Replay observation is invalid")
            if bool(maze["walls"][ey][ex] & (1 << direction)) != bool(wall):
                raise ValueError("Replay observation disagrees with source maze")
            knowledge.set_edge(ex, ey, direction, bool(wall))
        knowledge.visited[y][x] = True
        if event["known_edges"] != knowledge.count:
            raise ValueError("Replay known-edge count is inconsistent")
        if event["visited_cells"] != sum(sum(row) for row in knowledge.visited):
            raise ValueError("Replay visited-cell count is inconsistent")
        for key in ("lower_s", "upper_s"):
            if event[key] is not None and (not math.isfinite(event[key]) or event[key] < 0):
                raise ValueError("Replay route bound is invalid")
        if event["certified"] and (event["lower_s"] is None or event["upper_s"] != event["lower_s"]):
            raise ValueError("Replay certificate lacks matching exact bounds")
        previous = event
    summary = run["summary"]
    if summary["steps"] != len(events) - 1 or not math.isclose(summary["duration_s"], events[-1]["t"], abs_tol=1e-6):
        raise ValueError("Replay summary does not match its timeline")
    if bool(summary["certified"]) != bool(events[-1]["certified"]):
        raise ValueError("Replay certificate status does not match its timeline")
    first_certificate = next((event["t"] for event in events if event["certified"]), None)
    if summary["certificate_s"] != first_certificate:
        raise ValueError("Replay certificate timestamp is inconsistent")
    if run["final_known"] != knowledge.known or run["final_walls"] != knowledge.walls:
        raise ValueError("Replay final map is inconsistent with observations")


def export_ui_replay(payload, baseline=None, source_max_steps=None):
    maze, profile, source = payload["maze"], payload["profile"], payload["run"]
    if source.get("algorithm") != "c_predictive" or not source.get("scheduling", {}).get("predictive"):
        raise ValueError("Expected a saved predictive C run")
    validate_timeline(maze, source)
    if baseline is None:
        metadata = {key: value for key, value in maze.items() if key not in ("width", "height", "walls", "start", "goals")}
        model = Maze(maze["width"], maze["height"], maze["walls"], tuple(maze["start"]),
                     list(map(tuple, maze["goals"])), metadata)
        baseline = simulate(model, profile, EngineOracle(), "baseline", max_steps=12000)
    if baseline.get("algorithm") != "baseline":
        raise ValueError("Expected an Adachi baseline run")
    validate_timeline(maze, baseline)
    original = deepcopy(baseline)
    original.update(label="従来 · 全域探索", tag="ADACHI")
    ready = deepcopy(source)
    ready.update(algorithm="relevant", source_algorithm="c_predictive", label="C 実装 · 計算量制限", tag="MCU C")
    scheduling = deepcopy(source["scheduling"])
    policy_rate, oracle_rate = scheduling["policy_units_per_ms"], scheduling["oracle_units_per_ms"]
    surrogate = "ON" if scheduling["pending_surrogate"] else "OFF"
    result = {"schema": "nightfall_mcu_ui_replay_v1", "maze": deepcopy(maze), "profile": deepcopy(profile),
            "options": {"shortest_mode": 2, "shortest_case": 8, "epsilon": 0.0, "return_home": False,
                        **{key: scheduling[key] for key in ("policy_units_per_ms", "oracle_units_per_ms", "pending_surrogate")}},
            "runs": [original, ready],
            "metadata": {"schema": "nightfall_exploration_v1", "export_schema": "nightfall_mcu_ui_replay_v1",
                "fw_git_sha": None, "fw_git_dirty": None,
                "replay_kind": "mcu_finite_budget", "scheduling": scheduling,
                "replay_subtitle": f"C 実装の保存結果 · policy {policy_rate:g} / oracle {oracle_rate:g} work/ms · 補助探索 {surrogate}",
                "replay_notice_ja": "固定版 mini_r2 mode 2 case 8 の C 探索実装を再生しています。計算量上限はシナリオ設定であり、実機の所要時間・割込み余裕の保証ではありません。再計算ボタンは通常の Python 比較を実行します。",
                "notes_ja": [
                    "右側は実際の C policy/oracle の非同期 API と到着時検証を使用した保存結果です。左側は同じ探索速度・固定最短走行モデルによる足立法です。",
                    f"走行時間 1 ms 相当の計算量上限は policy {policy_rate:g}、oracle {oracle_rate:g} work。補助探索は {surrogate} です。実測 CPU 時間やリアルタイム動作の証明ではありません。",
                    "到着までに計算が終わらない場合などは足立法へ戻り、新たな観測があるまで方針を維持して往復を防ぎます。確定済みの厳密な終了判定は通過できます。",
                    "終了証明は固定版 mini_r2 mode 2 case 8 の運動グラフに限ります。最新の機体グラフや別の実行パラメータには適用できません。",
                    "C 側の上限は終了証明が得られるまで未導出として表示します。下限が未表示の期間もあり、未来の計算結果で補間しません。最短経路の線描画はこの C リプレイには含まれません。",
                ], "source_sha256": deepcopy(source.get("source_sha256", {})),
                "policy_config": deepcopy(source.get("policy_config", {})),
                "certificate_audit": deepcopy(source.get("certificate_audit")),
                "source_algorithm": source["algorithm"]}}
    if source_max_steps is not None:
        if source_max_steps < source["summary"]["steps"]:
            raise ValueError("Source step limit is smaller than the recorded run")
        result["options"]["max_steps"] = source_max_steps
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, help="Saved --grid trace containing maze/profile/run")
    parser.add_argument("--baseline", type=Path, help="Optional saved comparison containing an Adachi run")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--source-max-steps", type=int, help="Original grid step cap, when known")
    args = parser.parse_args()
    payload = json.loads(args.trace.read_text())
    baseline = None
    if args.baseline:
        comparison = json.loads(args.baseline.read_text())
        if comparison["maze"]["walls"] != payload["maze"]["walls"] or comparison["profile"] != payload["profile"]:
            parser.error("Baseline maze and exploration profile must match the C trace exactly")
        baseline = next(run for run in comparison["runs"] if run["algorithm"] == "baseline")
    result = export_ui_replay(payload, baseline, args.source_max_steps)
    result["metadata"]["source_trace_sha256"] = hashlib.sha256(args.trace.read_bytes()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + "\n")
    print(json.dumps({"output": str(args.output), "runs": [
        {"label": run["label"], "steps": run["summary"]["steps"], "duration_s": run["summary"]["duration_s"],
         "certified": run["summary"]["certified"]} for run in result["runs"]]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
