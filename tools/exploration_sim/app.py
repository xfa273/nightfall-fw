"""Local-only UI server and reproducible command-line comparison runner."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    __package__ = "tools.exploration_sim"

from .engine import simulate
from .mazes import catalog, discover_dataset, load_maze
from .oracle import TimeOracle, PLANNER_SOURCE_COMMIT
from .profiles import get_profile, get_profiles, smooth_turn_duration

ROOT = Path(__file__).resolve().parents[2]
STATIC = Path(__file__).resolve().parent / "static"
EDITABLE = {
    "search_speed_mm_s": (10, 5000), "known_speed_mm_s": (10, 10000),
    "acceleration_mm_s2": (10, 100000), "dash_acceleration_mm_s2": (10, 100000),
    "turn_alpha_deg_s2": (100, 1000000), "turn_rounding": (0.1, 3),
    "turn_entry_mm": (0, 180), "turn_exit_mm": (0, 180),
    "turn_omega_cap_deg_s": (0, 10000), "turn90_s": (0.01, 20),
    "uturn_s": (0.01, 30), "sensor_delay_s": (0, 5),
}


def numeric(value, label, low, high):
    if isinstance(value, bool):
        raise ValueError(f"{label} must be a number")
    try:
        result = float(value)
    except (ValueError, TypeError):
        raise ValueError(f"{label} must be a number") from None
    if not math.isfinite(result) or not low <= result <= high:
        raise ValueError(f"{label} must be between {low} and {high}")
    return result


def configure(request, maze):
    machine = request.get("machine") or ("mini_r2_0" if maze.metadata["family"] == "half" else "classic_r1_0")
    profile = get_profile(machine)
    if profile["family"] != maze.metadata["family"]:
        raise ValueError("Select mini_r2 for half mazes and classic_r1 for classic mazes")
    changes = request.get("profile", {})
    if not isinstance(changes, dict):
        raise ValueError("profile must be an object")
    for key, value in changes.items():
        if key not in EDITABLE:
            raise ValueError(f"Unknown editable profile parameter: {key}")
        profile[key] = numeric(value, key, *EDITABLE[key])
    if profile["known_speed_mm_s"] < profile["search_speed_mm_s"]:
        raise ValueError("Known-section maximum must be at least the search speed")
    if profile["search_speed_mm_s"] ** 2 > profile["acceleration_mm_s2"] * profile["cell_mm"] + 1e-6:
        raise ValueError("探索速度に対して加速度が不足しています。半区画で停止できるよう、加速度を 速度²÷区画長 以上にしてください。")
    # For automatic timing, angular and translational fields change together.
    # An explicit turn90_s is a measured/calibrated duration override.
    if "turn90_s" not in changes:
        profile["turn90_angular_s"] = smooth_turn_duration(90, profile["turn_alpha_deg_s2"],
            profile["turn_rounding"], profile["turn_omega_cap_deg_s"])
        profile["turn90_s"] = (profile["turn90_angular_s"] +
            (profile["turn_entry_mm"] + profile["turn_exit_mm"]) / profile["search_speed_mm_s"])
    mode = numeric(request.get("shortest_mode", 2), "shortest_mode", 2, 5)
    case = numeric(request.get("shortest_case", 1 if profile["family"] == "classic" else 8), "shortest_case", 1, 9)
    max_steps = numeric(request.get("max_steps", 12000), "max_steps", 1, 20000)
    if any(value != int(value) for value in (mode, case, max_steps)):
        raise ValueError("Mode, case and step limit must be integers")
    return_home = request.get("return_home", False)
    if not isinstance(return_home, bool):
        raise ValueError("return_home must be boolean")
    epsilon = numeric(request.get("epsilon", 0.0), "epsilon", 0, 0.5)
    options = dict(shortest_mode=int(mode), shortest_case=int(case),
                   epsilon=epsilon, return_home=return_home, max_steps=int(max_steps))
    return profile, options


def git_metadata():
    def git(*args):
        return subprocess.check_output(["git", *args], cwd=ROOT, text=True, stderr=subprocess.DEVNULL).strip()
    try:
        return {"fw_git_sha": git("rev-parse", "HEAD"), "fw_git_dirty": bool(git("status", "--porcelain"))}
    except (OSError, subprocess.SubprocessError):
        return {"fw_git_sha": "unavailable", "fw_git_dirty": None}


def compare(request, maze_dir=None, progress=None):
    maze = load_maze(request.get("maze_id", "32MM2023HX"), maze_dir)
    if maze.start != (0, 0):
        raise ValueError("Time planner currently requires S at (0,0), heading north")
    profile, options = configure(request, maze)
    started = time.monotonic()
    oracle = TimeOracle(profile["machine"], options["shortest_mode"], options["shortest_case"])
    runs = []
    for algorithm in ("baseline", "relevant"):
        if progress:
            progress({"algorithm": algorithm, "step": 0, "message": "planning"})
        runs.append(simulate(maze, profile, oracle, algorithm, options["epsilon"],
                             options["return_home"], options["max_steps"], progress))
    # Evaluation only: fully known ground truth is never supplied to a policy.
    truth_solution = oracle.solve(maze.walls, maze.goals, details=False)
    metadata = {
        "schema": "nightfall_exploration_v1", "created_at": datetime.now(timezone.utc).isoformat(),
        **git_metadata(), "planner_source_commit": PLANNER_SOURCE_COMMIT,
        "planner": oracle.planner, "objective": "goal_entry_s", "epsilon": options["epsilon"],
        "compute_s": round(time.monotonic() - started, 3), "timing_model": "ideal_cell_kinematics_v1",
        "truth_optimum_s": truth_solution.get("goal_entry_s"),
        "truth_status": truth_solution.get("status"),
        "observations": "front_left_right_at_cell_centre; traversed_back_edge_known; outer_boundary_prior",
        "bounds": "Admissible optimistic graph lower bound (retained during initial Adachi goal phase); feasible known-route upper bound (refreshed every 32 new edges or when optimistic route becomes known).",
        "limitations": [
            "Times are kinematic estimates, not measured run times; no wheel slip, sensor errors, wall alignment, NVM latency or controller delay unless configured.",
            "Optimality is within the imported planner's finite motion graph and commanded trajectory model; mini diagonal geometry is provisional.",
            "Classic shortest evaluation currently uses the orthogonal time planner; its diagonal geometry has no calibrated baseline.",
            "Classic baseline uses the common GOAL→FULL Adachi decision policy with continuous phase transition; the F405 goal-stop/back-up/forced-forward restart is not replayed.",
            "Nominal search speed is the mode1 case1 slalom speed; firmware launch cruise and alignment transients are approximated.",
            "Both policies start from an empty map; optional home return is applied equally. A step-limit result is incomplete.",
        ],
    }
    return {"maze": maze.to_dict(), "profile": profile, "options": options, "runs": runs, "metadata": metadata}


def comparison_csv(result):
    buffer = io.StringIO(newline="")
    fields = ["maze_id", "machine", "fw_git_sha", "fw_git_dirty", "source_revision", "source_sha256",
              "algorithm", "step", "t", "dt", "x", "y", "heading", "turn", "known_straight",
              "known_edges", "visited_cells", "lower_s", "upper_s", "certified", "phase", "target", "changes",
              "profile_json", "options_json", "metadata_json"]
    writer = csv.DictWriter(buffer, fieldnames=fields)
    writer.writeheader()
    for run in result["runs"]:
        for event in run["events"]:
            row = {key: event.get(key) for key in fields}
            row.update(maze_id=result["maze"]["id"], machine=result["profile"]["machine"],
                       fw_git_sha=result["metadata"]["fw_git_sha"], fw_git_dirty=result["metadata"]["fw_git_dirty"],
                       source_revision=result["maze"].get("source_revision"), source_sha256=result["maze"].get("source_sha256"),
                       algorithm=run["algorithm"], target=json.dumps(event.get("target")),
                       changes=json.dumps(event["changes"], separators=(",", ":")),
                       profile_json=json.dumps(result["profile"], ensure_ascii=False),
                       options_json=json.dumps(result["options"]), metadata_json=json.dumps(result["metadata"], ensure_ascii=False))
            writer.writerow(row)
    return buffer.getvalue()


class Application:
    def __init__(self, maze_dir=None, replay=None):
        self.maze_dir = discover_dataset(maze_dir)
        self.catalog = {"mazes": catalog(self.maze_dir), "profiles": get_profiles()}
        self.replay = json.loads(Path(replay).read_text(encoding="utf-8")) if replay else None
        if self.replay is not None and (not isinstance(self.replay, dict) or
                self.replay.get("metadata", {}).get("schema") != "nightfall_exploration_v1"):
            raise ValueError("Replay must be a nightfall_exploration_v1 experiment JSON")
        self.jobs = {}
        self.lock = threading.Lock()
        self.worker = threading.Lock()

    def start_job(self, request):
        # Validate before spawning work so errors are immediate and actionable.
        maze = load_maze(request.get("maze_id", "32MM2023HX"), self.maze_dir)
        configure(request, maze)
        with self.lock:
            if any(job["status"] in ("queued", "running") for job in self.jobs.values()):
                raise ValueError("A comparison is already running; wait for its result")
            # Results can be large: retain only the two most recent comparisons.
            while len(self.jobs) >= 2:
                self.jobs.pop(next(iter(self.jobs)))
            job_id = uuid.uuid4().hex
            self.jobs[job_id] = {"status": "queued", "progress": {"step": 0}}
        def work():
            with self.worker:
                def update(progress):
                    with self.lock:
                        self.jobs[job_id].update(status="running", progress=progress)
                try:
                    result = compare(request, self.maze_dir, update)
                    with self.lock:
                        self.jobs[job_id] = {"status": "done", "result": result}
                except Exception as exc:
                    with self.lock:
                        self.jobs[job_id] = {"status": "error", "error": str(exc)}
        threading.Thread(target=work, daemon=True, name="exploration-comparison").start()
        return job_id


def serve(maze_dir=None, port=8765, replay=None):
    app = Application(maze_dir, replay)
    class Handler(BaseHTTPRequestHandler):
        def respond(self, value, status=200):
            data = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/api/catalog":
                return self.respond(app.catalog)
            if path == "/api/replay":
                return self.respond(app.replay)
            if path.startswith("/api/jobs/"):
                with app.lock:
                    job = app.jobs.get(path.rsplit("/", 1)[1])
                return self.respond(job or {"error": "Unknown job"}, 200 if job else 404)
            assets = {"/": ("index.html", "text/html"), "/index.html": ("index.html", "text/html"),
                      "/app.js": ("app.js", "text/javascript"), "/style.css": ("style.css", "text/css")}
            if path not in assets:
                return self.respond({"error": "Not found"}, 404)
            name, content_type = assets[path]
            data = (STATIC / name).read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", content_type + "; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(data)

        def do_POST(self):
            # Local loopback only; do not permit another origin to queue work.
            origin = self.headers.get("Origin")
            if origin and urlsplit(origin).netloc != self.headers.get("Host"):
                return self.respond({"error": "Cross-origin requests are not allowed"}, 403)
            if urlsplit(self.path).path != "/api/simulate":
                return self.respond({"error": "Not found"}, 404)
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 32768:
                    raise ValueError("Request is empty or too large")
                request = json.loads(self.rfile.read(length))
                if not isinstance(request, dict):
                    raise ValueError("Request must be an object")
                return self.respond({"job_id": app.start_job(request)}, 202)
            except (ValueError, KeyError, FileNotFoundError) as exc:
                return self.respond({"error": str(exc)}, 400)

        def log_message(self, format, *args):
            if len(args) > 1 and str(args[1]) not in ("200", "202", "304"):
                super().log_message(format, *args)
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    print(f"Nightfall exploration simulator: http://127.0.0.1:{server.server_port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--maze-dir", help="Existing KeriLab clone or data directory")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--replay", type=Path, help="Open a previously exported experiment JSON in the UI")
    parser.add_argument("--maze", help="Run a comparison without the UI (e.g. 32MM2023HX)")
    parser.add_argument("--config", type=Path, help="JSON request containing settings overrides")
    parser.add_argument("--json", type=Path, help="Write full replay JSON")
    parser.add_argument("--csv", type=Path, help="Write per-step comparison CSV")
    parser.add_argument("--return-home", action="store_true")
    parser.add_argument("--epsilon", type=float, default=0.0)
    parser.add_argument("--list", action="store_true", help="List available competition mazes")
    args = parser.parse_args()
    if args.list:
        for entry in catalog(args.maze_dir):
            print(entry["id"], entry["name"], "" if entry["available"] else "[incomplete source]")
        return
    if args.maze or args.config:
        request = json.loads(args.config.read_text()) if args.config else {}
        if args.maze:
            request["maze_id"] = args.maze
        if args.return_home:
            request["return_home"] = True
        if args.epsilon:
            request["epsilon"] = args.epsilon
        result = compare(request, args.maze_dir)
        for destination, contents in (
            (args.json, json.dumps(result, ensure_ascii=False, allow_nan=False)),
            (args.csv, comparison_csv(result)),
        ):
            if destination:
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_text(contents, encoding="utf-8")
        print(json.dumps({"maze": result["maze"]["id"], "runs": [
            {"algorithm": run["algorithm"], **run["summary"]} for run in result["runs"]],
            "metadata": result["metadata"]}, ensure_ascii=False, indent=2))
    else:
        try:
            serve(args.maze_dir, args.port, args.replay)
        except OSError as exc:
            parser.exit(1, f"Cannot start local simulator: {exc}. Try --port 8876.\n")


if __name__ == "__main__":
    main()
