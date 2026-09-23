#!/usr/bin/env python3
"""Offline, measured-data turn identification and bounded parameter exploration.

No serial, flash, NVM or firmware mutation is performed by this module.
Coordinates: right +x, forward +y, left-positive yaw (degrees).
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares, minimize

import turn_tune as nominal
from turn_dataset import load_dataset
from turn_dynamics import DynamicsModel, PerformanceLimits, predict

SCHEMA = "nightfall_turn_dynamics_model_v1"
FIT_BOUNDS = {"velocity_gain": (0.7, 1.3), "yaw_gain": (0.8, 1.2),
              "velocity_tau_s": (0.0, 0.15), "yaw_tau_s": (0.0, 0.08),
              "yaw_delay_s": (0.0, 0.05), "lateral_tau_s": (0.0, 0.12)}
DEFAULT_VARY = ("velocity_gain", "lateral_tau_s")
PARAM_FIELDS = {"velocity": "velocity_mm_s", "alpha": "alpha_deg_s2",
                "angle": "signed_angle_deg", "dist_in": "dist_in_mm",
                "dist_out": "dist_out_mm"}
LIMIT_FIELDS = {"limit_velocity": "max_velocity_mm_s", "limit_omega": "max_omega_deg_s",
                "limit_alpha": "max_alpha_deg_s2", "limit_lateral_accel": "max_lateral_accel_mm_s2"}


def performance_limits(parameters):
    return PerformanceLimits(**{field: float(parameters[key]) for key, field in LIMIT_FIELDS.items() if key in parameters})


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def original_parameters(run):
    p = run.provenance.get("original_params", {})
    return {"velocity": run.spec.velocity_mm_s, "alpha": run.spec.alpha_deg_s2,
            "angle": run.spec.signed_angle_deg,
            "dist_in": p.get("dist_in_mm", run.spec.dist_in_mm),
            "dist_out": p.get("dist_out_mm", run.spec.dist_out_mm)}


def select_runs(dataset, regime="positive-entry"):
    selected, excluded = [], list(dataset.exclusions)
    for run in dataset.runs:
        minimum_duration = nominal.simulate_turn(run.spec, run.constants).duration_ms / 1000
        if len(run.time_s) < 8 or run.time_s[-1] < minimum_duration - .002:
            excluded.append({"id": run.id, "reason": "trajectory does not cover the complete commanded turn"})
            continue
        run_regime = run.provenance.get("execution_regime", "positive-entry" if original_parameters(run)["dist_in"] > 0 else "zero-entry")
        if run_regime != regime:
            excluded.append({"id": run.id, "reason": "different entry execution regime"})
        else:
            selected.append(run)
    if not selected:
        raise ValueError("No usable trajectories in the requested entry regime")
    # A model must not silently pool distinct machines or profile definitions.
    constants = {tuple(asdict(r.constants).values()) for r in selected}
    if len(constants) != 1:
        raise ValueError("Split datasets with different omega profile constants")
    machines = {str(r.provenance.get("machine", "unknown")) for r in selected}
    if len(machines) != 1:
        raise ValueError("Split datasets by machine before identification")
    return selected, excluded


def prediction_for_run(run, model):
    times = np.asarray(run.time_s, dtype=float)
    # Preserve the actual video timestamps; no per-model endpoint alignment.
    duration = nominal.simulate_turn(run.spec, run.constants).duration_ms / 1000
    settle = max(0.15, float(times[-1]) - duration + 0.002)
    return predict(run.spec, run.constants, times, model=model, settle_time_s=settle)


def fit_model(runs, vary=DEFAULT_VARY):
    if not vary or len(set(vary)) != len(vary) or set(vary) - FIT_BOUNDS.keys():
        raise ValueError("Choose distinct model coefficients from " + ", ".join(FIT_BOUNDS))
    trusted_heading = all(r.theta_deg is not None and all(v is not None for v in r.theta_deg) for r in runs)
    if not trusted_heading and set(vary) & {"yaw_gain", "yaw_tau_s", "yaw_delay_s"}:
        raise ValueError("Yaw response identification requires trusted body-heading measurements; use velocity_gain,lateral_tau_s for XY-only data")
    base = asdict(DynamicsModel())

    def unpack(values):
        return DynamicsModel(**(base | dict(zip(vary, values))))

    def residual(values):
        output = []
        model = unpack(values)
        for run in runs:
            result = prediction_for_run(run, model)
            # Equal total weight per video; high frame rate is not extra evidence.
            xy = np.column_stack((result.x_mm - run.x_mm, result.y_mm - run.y_mm))
            output.extend((xy / math.sqrt(len(xy))).ravel())
            heading = run.theta_deg
            if heading is not None and all(v is not None for v in heading):
                error = (result.theta_deg - np.asarray(heading) + 180) % 360 - 180
                output.extend(0.5 * error / math.sqrt(len(error)))
        return np.asarray(output)

    low, high = zip(*(FIT_BOUNDS[k] for k in vary))
    initial = [max(low[i] + 1e-4, base[k]) for i, k in enumerate(vary)]
    result = least_squares(residual, initial, bounds=(low, high), loss="soft_l1",
                           f_scale=0.5, x_scale="jac", max_nfev=100,
                           diff_step=1e-3, ftol=1e-6, xtol=1e-6, gtol=1e-6)
    singular = np.linalg.svd(result.jac, compute_uv=False)
    condition = float(singular[0] / singular[-1]) if singular[-1] > 1e-10 else None
    return unpack(result.x), {"converged": bool(result.success), "message": result.message,
                             "evaluations": result.nfev, "jacobian_condition": condition,
                             "at_bound": [k for i, k in enumerate(vary)
                                          if min(result.x[i]-low[i], high[i]-result.x[i]) < 1e-4]}


def evaluate_runs(runs, model):
    rows = []
    for run in runs:
        p = prediction_for_run(run, model)
        error = np.hypot(p.x_mm - run.x_mm, p.y_mm - run.y_mm)
        rows.append({"id": run.id, "group_id": run.group_id,
                     "rmse_mm": float(np.sqrt(np.mean(error**2))),
                     "p95_mm": float(np.percentile(error, 95)),
                     "max_mm": float(np.max(error)), "endpoint_mm": float(error[-1])})
    return {"rmse_mm": float(np.sqrt(np.mean([r["rmse_mm"]**2 for r in rows]))),
            "worst_path_error_mm": max(r["max_mm"] for r in rows),
            "worst_endpoint_mm": max(r["endpoint_mm"] for r in rows), "runs": rows}


def calibrate(dataset, regime="positive-entry", vary=DEFAULT_VARY):
    runs, excluded = select_runs(dataset, regime)
    groups = sorted({r.group_id for r in runs})
    baseline = evaluate_runs(runs, DynamicsModel())
    model, fit = fit_model(runs, vary)
    folds = []
    # Leave an entire parameter condition out, including every repeat/reprocessing.
    if len(groups) >= 3:
        for group in groups:
            training = [r for r in runs if r.group_id != group]
            held = [r for r in runs if r.group_id == group]
            fold_model, fold_fit = fit_model(training, vary)
            folds.append({"held_out_group": group, "training_groups": sorted({r.group_id for r in training}),
                          "fit": fold_fit, "predicted": evaluate_runs(held, fold_model),
                          "ideal": evaluate_runs(held, DynamicsModel())})
    cv_runs = [row for fold in folds for row in fold["predicted"]["runs"]]
    cv_rmse = float(np.sqrt(np.mean([r["rmse_mm"]**2 for r in cv_runs]))) if cv_runs else None
    improvement = cv_rmse is not None and cv_rmse < baseline["rmse_mm"]
    coverage = {key: [min(original_parameters(r)[key] for r in runs),
                      max(original_parameters(r)[key] for r in runs)] for key in PARAM_FIELDS}
    valid_fits = all(f["fit"]["converged"] for f in folds) and fit["converged"] and fit["jacobian_condition"] is not None and fit["jacobian_condition"] < 1e6
    status = "diagnostic_calibrated" if improvement and valid_fits else "diagnostic_insufficient_validation"
    return {"schema": SCHEMA, "status": status, "coefficients": asdict(model),
            "heading_identified": all(r.theta_deg is not None for r in runs),
            "constants": asdict(runs[0].constants), "vary": list(vary), "fit": fit,
            "coverage": coverage, "regime": regime, "groups": groups,
            "observed_angles_deg": sorted({r.spec.signed_angle_deg for r in runs}),
            "representative_parameters": original_parameters(runs[-1]),
            "metrics": {"ideal": baseline, "training": evaluate_runs(runs, model),
                        "held_out_rmse_mm": cv_rmse,
                        "held_out_worst_path_error_mm": max((r["max_mm"] for r in cv_runs), default=None),
                        "held_out_improves_ideal": improvement}, "validation_folds": folds,
            "qualification": {"hardware_validated": False, "safety_qualified": False,
                              "scope": "isolated turn shape; no wall, motor or maze clearance model"},
            "warnings": list(dict.fromkeys(dataset.warnings + [w for r in runs for w in r.warnings] + [
                "同定値は撮影・開始時刻の誤差を含む実効係数です。個別の物理特性とは断定できません。",
                "壁制御・壁切れ・電池・ファン・機体寸法・モータ飽和は未モデル化です。迷路走行は未検証です。",
                "交差検証の最大誤差は観測値であり、将来の誤差上限や衝突回避保証ではありません。"])),
            "exclusions": excluded,
            "sources": [{"id": r.id, "group_id": r.group_id, "provenance": r.provenance,
                         "diagnostic_only": r.diagnostic_only} for r in runs],
            "measured": [{"label": r.id, "parameters": original_parameters(r),
                          "samples": [{"t_ms": t*1000, "x_mm": x, "y_mm": y,
                                       "theta_deg": h} for t, x, y, h in
                                      zip(r.time_s, r.x_mm, r.y_mm,
                                          r.theta_deg if r.theta_deg is not None else [None]*len(r.time_s))],
                          "sample_mode": r.provenance.get("sample_mode", "core")} for r in runs]}


def load_model(path):
    if path is None:
        return None
    artifact = json.loads(Path(path).read_text())
    if artifact.get("schema") != SCHEMA:
        raise ValueError("Unsupported model schema")
    DynamicsModel(**artifact["coefficients"])
    for key in PARAM_FIELDS:
        bounds = artifact["coverage"][key]
        if len(bounds) != 2 or not all(math.isfinite(v) for v in bounds) or bounds[0] > bounds[1]:
            raise ValueError("Invalid model coverage")
    return artifact


def samples(prediction):
    return [{"t_ms": float(t)*1000, "x_mm": float(x), "y_mm": float(y), "theta_deg": float(h), "course_deg": float(c)}
            for t, x, y, h, c in zip(prediction.time_s, prediction.x_mm, prediction.y_mm, prediction.theta_deg, prediction.course_deg)]


def make_turn(parameters):
    values = {k: float(parameters[k]) for k in PARAM_FIELDS}
    if not all(math.isfinite(v) for v in values.values()):
        raise ValueError("Parameters must be finite")
    if not (10 <= values["velocity"] <= 5000 and 100 <= values["alpha"] <= 200000
            and 1 <= abs(values["angle"]) <= 180 and 0 <= values["dist_in"] <= 150
            and 0 <= values["dist_out"] <= 150):
        raise ValueError("Out of supported input bounds (v 10..5000, alpha 100..200000, |angle| 1..180, offsets 0..150)")
    return nominal.TurnSpec("manual", "isolated-turn", **{PARAM_FIELDS[k]: v for k, v in values.items()}, source_fields={})


def simulation(parameters, artifact=None):
    turn = make_turn(parameters)
    constants = nominal.Constants(**artifact["constants"]) if artifact else nominal.Constants(1.2, 2200)
    model = DynamicsModel(**artifact["coefficients"]) if artifact else DynamicsModel()
    ideal = predict(turn, constants, settle_time_s=0)
    predicted = predict(turn, constants, model=model, settle_time_s=0, limits=performance_limits(parameters))
    warnings = ["実機適用前の候補です。実機走行・壁とのクリアランスは未検証です。"]
    outside = []
    if artifact:
        outside = [key for key, (low, high) in artifact["coverage"].items()
                   if not low - 1e-7 <= float(parameters[key]) <= high + 1e-7]
        if artifact.get("observed_angles_deg") and not any(abs(turn.signed_angle_deg-a) < 1e-6 for a in artifact["observed_angles_deg"]):
            outside.append("unobserved_turn_angle")
        if outside:
            warnings.append("学習範囲外: " + ", ".join(outside))
        if artifact["regime"] == "positive-entry" and turn.dist_in_mm <= 0:
            warnings.append("入口距離ゼロは別の実行条件です。")
        warnings.extend(artifact.get("warnings", []))
        if not artifact.get("heading_identified", False):
            warnings.append("機体角度は未同定の理想指令値です。位置から推定した進行方向(course)と区別してください。")
    else:
        warnings.append("未校正: 理想軌道のみ。fitで実測モデルを作成してください。")
    if not predicted.metrics["limits_checked"]:
        warnings.append("機体の性能上限が未指定のため、性能余裕は未評価です。")
    elif not predicted.metrics["limits_passed"]:
        warnings.append("指定した性能上限を超えています: " + ", ".join(predicted.metrics["exceeded_limits"]))
    x, y, theta = float(predicted.x_mm[-1]), float(predicted.y_mm[-1]), float(predicted.theta_deg[-1])
    target = [float(parameters.get("target_x", x)), float(parameters.get("target_y", y)),
              float(parameters.get("target_theta", theta))]
    if not all(math.isfinite(v) and abs(v) < 10000 for v in target):
        raise ValueError("Invalid target pose")
    measured = []
    if artifact:
        # Overlay only the matching recorded parameter condition; never other turns.
        for run in artifact.get("measured", []):
            p = run["parameters"]
            if all(abs(float(parameters[k]) - p[k]) < 1e-6 for k in PARAM_FIELDS):
                offset = p["dist_in"] if run.get("sample_mode", "core") == "core" else 0
                measured.append({"label": run["label"], "samples": [dict(s, y_mm=s["y_mm"]+offset,
                    t_ms=s["t_ms"]+1000*offset/p["velocity"]) for s in run["samples"]
                    if s["t_ms"]+1000*offset/p["velocity"] <= predicted.time_s[-1]*1000]})
    if measured:
        warnings.append("実測コアの重ね合わせは名目入口位置での比較です。絶対位置・壁との距離の検証ではありません。")
    return {"parameters": {k: float(v) for k, v in parameters.items()},
            "ideal": samples(ideal), "predicted": samples(predicted), "measured": measured,
            "metrics": dict(predicted.metrics, endpoint_x_mm=x, endpoint_y_mm=y, endpoint_theta_deg=theta,
                            endpoint_error_mm=math.hypot(x-target[0], y-target[1]),
                            heading_error_deg=(theta-target[2]+180)%360-180,
                            endpoint_course_deg=float(predicted.course_deg[-1]),
                            within_observed_bounds=bool(artifact) and not outside,
                            safe_recommendation_available=False), "warnings": list(dict.fromkeys(warnings)),
            "assignments": ("/* Simulator candidate; not hardware validated.\n"
                            " * angle is the effective executed angle, not necessarily the params field. */\n"
                            f"velocity = {turn.velocity_mm_s:.17g};\nalpha = {turn.alpha_deg_s2:.17g};\n"
                            f"dist_in = {turn.dist_in_mm:.17g};\ndist_out = {turn.dist_out_mm:.17g};")}


def tune(parameters, artifact=None):
    make_turn(parameters)
    for key in ("target_x", "target_y", "target_theta"):
        if key not in parameters:
            raise ValueError("Tuning requires " + key)
    baseline = simulation(parameters, artifact)
    constants = nominal.Constants(**artifact["constants"]) if artifact else nominal.Constants(1.2, 2200)
    model = DynamicsModel(**artifact["coefficients"]) if artifact else DynamicsModel()
    alpha_bounds = artifact["coverage"]["alpha"] if artifact else [1000, 40000]
    bounds = [alpha_bounds, artifact["coverage"]["dist_in"] if artifact else [0.5, 80],
              artifact["coverage"]["dist_out"] if artifact else [0.5, 80]]
    limits = performance_limits(parameters)
    # Quantized firmware ticks make a derivative-free search appropriate.
    best = [float("inf"), None]
    def objective(values):
        p = dict(parameters, alpha=values[0], dist_in=values[1], dist_out=values[2])
        pred = predict(make_turn(p), constants, model=model, settle_time_s=0, limits=limits)
        error = (pred.x_mm[-1]-parameters["target_x"])**2 + (pred.y_mm[-1]-parameters["target_y"])**2
        angle_series = pred.theta_deg if artifact and artifact.get("heading_identified") else pred.course_deg
        heading = (angle_series[-1]-parameters["target_theta"]+180)%360-180
        cost = float(error + (heading * 2)**2)
        cost += 1e6 * sum(max(0, ratio - 1)**2 for ratio in pred.metrics["limit_ratios"].values())
        if cost < best[0]:
            best[:] = [cost, np.asarray(values).copy()]
        return cost
    initial = np.clip([parameters["alpha"], parameters["dist_in"], parameters["dist_out"]],
                      [b[0] for b in bounds], [b[1] for b in bounds])
    objective(initial)
    free = [i for i, (low, high) in enumerate(bounds) if high > low]
    if free:
        def free_objective(values):
            candidate = initial.copy()
            candidate[free] = values
            return objective(candidate)
        result = minimize(free_objective, initial[free], method="Powell", bounds=[bounds[i] for i in free],
                          options={"maxiter": 60, "xtol": 0.002, "ftol": 1e-7})
        converged, message = bool(result.success), str(result.message)
    else:
        converged, message = True, "All parameters fixed by calibration coverage"
    chosen = best[1]
    p = dict(parameters, alpha=float(chosen[0]), dist_in=float(chosen[1]), dist_out=float(chosen[2]))
    response = simulation(p, artifact)
    heading_value = response["metrics"]["endpoint_theta_deg"] if artifact and artifact.get("heading_identified") else response["metrics"]["endpoint_course_deg"]
    response["optimization"] = {"converged": converged, "message": message,
                                "target_reached": response["metrics"]["endpoint_error_mm"] <= 2 and abs((heading_value-parameters["target_theta"]+180)%360-180) <= 2,
                                "initial_error_mm": baseline["metrics"]["endpoint_error_mm"],
                                "objective": best[0], "bounds": bounds,
                                "heading_objective": "body" if artifact and artifact.get("heading_identified") else "course",
                                "fixed": ["velocity", "effective_angle"]}
    return response


def serve(parameters, artifact, port):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, value, status=200):
            payload = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            if self.path == "/api/state":
                self.reply({"defaults": parameters, "model": artifact or {"status": "uncalibrated", "coverage": {}, "metrics": {}}, "measured": []})
            elif self.path in ("/", "/turn_simulator.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(Path(__file__).with_suffix(".html").read_bytes())
            else:
                self.reply({"error": "Not found"}, 404)

        def do_POST(self):
            # No cross-origin controls or file operations are exposed by this server.
            origin = self.headers.get("Origin")
            if origin and origin != f"http://{self.headers.get('Host')}":
                return self.reply({"error": "Cross-origin request rejected"}, 403)
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 0 < length <= 16384:
                    raise ValueError("Invalid request length")
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError("Expected parameter object")
                if self.path == "/api/simulate":
                    self.reply(simulation(parameters | body, artifact))
                elif self.path == "/api/tune":
                    self.reply(tune(parameters | body, artifact))
                else:
                    self.reply({"error": "Not found"}, 404)
            except (ValueError, TypeError, KeyError) as exc:
                self.reply({"error": str(exc)}, 400)

    server = HTTPServer(("127.0.0.1", port), Handler)
    print(f"Turn simulator: http://127.0.0.1:{server.server_port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    for command in ("inventory", "fit"):
        s = sub.add_parser(command)
        s.add_argument("manifest", type=Path)
        s.add_argument("--data-root", type=Path)
        s.add_argument("--output", type=Path, required=command == "fit")
        if command == "fit":
            s.add_argument("--regime", choices=("positive-entry", "zero-entry"), default="positive-entry")
            s.add_argument("--vary", default=",".join(DEFAULT_VARY))
    for command in ("simulate", "tune", "serve"):
        s = sub.add_parser(command)
        s.add_argument("--model", type=Path)
        s.add_argument("--output", type=Path)
        for key in (*PARAM_FIELDS, "target_x", "target_y", "target_theta"):
            s.add_argument("--" + key.replace("_", "-"), type=float)
        for key in LIMIT_FIELDS:
            s.add_argument("--" + key.replace("_", "-"), type=float, help="independently established performance limit (never inferred)")
        if command == "serve":
            s.add_argument("--port", type=int, default=8765)
    return p


def main():
    args = parser().parse_args()
    try:
        if args.command in ("inventory", "fit"):
            dataset = load_dataset(args.manifest, args.data_root)
            if args.command == "inventory":
                output = {"runs": len(dataset.runs), "groups": len({r.group_id for r in dataset.runs}),
                          "warnings": dataset.warnings, "exclusions": dataset.exclusions,
                          "parameters": [original_parameters(r) for r in dataset.runs]}
            else:
                output = calibrate(dataset, args.regime, tuple(args.vary.split(",")))
                output["manifest_sha256"] = hashlib.sha256(args.manifest.read_bytes()).hexdigest()
                output["manifest"] = str(args.manifest.resolve())
        else:
            artifact = load_model(args.model)
            parameters = {"velocity": 500., "alpha": 9750., "angle": -135., "dist_in": 4., "dist_out": 17.5}
            if artifact:
                parameters.update(artifact["representative_parameters"])
            parameters.update({k: getattr(args, k) for k in PARAM_FIELDS if getattr(args, k) is not None})
            parameters.update({k: getattr(args, k) for k in LIMIT_FIELDS if getattr(args, k) is not None})
            initial = simulation(parameters, artifact)["ideal"][-1]
            parameters.update({k: getattr(args, k) if getattr(args, k) is not None else initial[field]
                               for k, field in [("target_x", "x_mm"), ("target_y", "y_mm"), ("target_theta", "theta_deg")]})
            if args.command == "serve":
                serve(parameters, artifact, args.port)
                return 0
            output = tune(parameters, artifact) if args.command == "tune" else simulation(parameters, artifact)
        if args.output:
            write_json(args.output, output)
            print(f"Saved {args.output}")
        else:
            print(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (ValueError, KeyError, TypeError, OSError) as exc:
        print(f"turn_simulator: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
