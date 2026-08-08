#!/usr/bin/env python3
"""Serve a local Mac dashboard for Pixel HFR capture and collection."""

from __future__ import annotations

import argparse
import json
import secrets
import sys
import threading
import time
import webbrowser
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from collect_wifi_runs import (
    ApiClient,
    CaptureBusyError,
    COMPLETE_ARTIFACTS,
    DEFAULT_HTTP_PORT,
    DEFAULT_OUTPUT_ROOT,
    Endpoint,
    WifiCollectorError,
    collect_all,
    default_config_path,
    discover_devices,
    fetch_info,
    load_config,
    parse_host,
    save_config,
)
from hfr_control import (
    CONTROL_START_PATH,
    CONTROL_STOP_PATH,
    capture_state,
    read_status,
    wait_for_start,
    wait_for_stop,
)


DASHBOARD_DIR = Path(__file__).with_name("dashboard")
STATIC_FILES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/dashboard.css": ("dashboard.css", "text/css; charset=utf-8"),
    "/dashboard.js": ("dashboard.js", "text/javascript; charset=utf-8"),
}
VALID_ACTIONS = {"start", "stop", "collect", "stop-and-collect"}


@dataclass
class Operation:
    sequence: int = 0
    name: str | None = None
    state: str = "idle"
    message: str = ""
    started_at_unix_ms: int | None = None
    finished_at_unix_ms: int | None = None

    def to_json(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "name": self.name,
            "state": self.state,
            "message": self.message,
            "started_at_unix_ms": self.started_at_unix_ms,
            "finished_at_unix_ms": self.finished_at_unix_ms,
        }


def latest_run_summary(response: dict[str, Any]) -> dict[str, Any] | None:
    """Return enough inventory detail to tell whether the newest run is complete."""
    runs = response.get("runs")
    if not isinstance(runs, list):
        raise WifiCollectorError("Pixel応答のrunsが不正です")
    valid = [item for item in runs if isinstance(item, dict) and item.get("name")]
    if not valid:
        return None
    latest = max(valid, key=lambda item: str(item["name"]))
    artifacts_value = latest.get("artifacts")
    artifacts: dict[str, int] = {}
    if isinstance(artifacts_value, list):
        for item in artifacts_value:
            if not isinstance(item, dict):
                continue
            name = item.get("name")
            size = item.get("size")
            if isinstance(name, str) and isinstance(size, int) and size >= 0:
                artifacts[name] = size
    missing = sorted(COMPLETE_ARTIFACTS.difference(artifacts))
    status = str(latest.get("status", "missing"))
    video_bytes = artifacts.get("hfr_capture.mp4", 0)
    report_bytes = artifacts.get("hfr_report.json", 0)
    complete = (
        status == "complete"
        and not missing
        and video_bytes > 0
        and report_bytes > 0
    )
    return {
        "name": str(latest["name"]),
        "status": status,
        "acknowledged": bool(latest.get("acknowledged")),
        "complete": complete,
        "missing_artifacts": missing,
        "video_bytes": video_bytes,
        "artifact_bytes": sum(artifacts.values()),
    }


class PixelDashboard:
    """Keep Pixel discovery, status validation, and long operations off the UI."""

    def __init__(
        self,
        *,
        config_path: Path,
        output_root: Path,
        pixel_host: str | None,
        device_id: str | None,
        discover_seconds: float,
        wait_seconds: float,
        recording_warning_seconds: float,
    ) -> None:
        self.config_path = config_path
        self.output_root = output_root
        self.pixel_host = pixel_host
        self.device_id = device_id
        self.discover_seconds = discover_seconds
        self.wait_seconds = wait_seconds
        self.recording_warning_seconds = recording_warning_seconds
        self._connection_lock = threading.Lock()
        self._endpoint: Endpoint | None = None
        self._token: str | None = None
        self._inventory_saved_runs: int | None = None
        self._inventory_acknowledged_runs: int | None = None
        self._inventory_checked_monotonic = 0.0
        self._latest_run: dict[str, Any] | None = None
        self._inventory_error: str | None = None
        self._capture_state_lock = threading.Lock()
        self._last_capture_state: str | None = None
        self._recording_started_monotonic: float | None = None
        self._operation_lock = threading.Lock()
        self._operation = Operation()

    def _select_endpoint(self) -> tuple[Endpoint, str]:
        config = load_config(self.config_path)
        devices = config["devices"]

        if self.pixel_host:
            host, port = parse_host(self.pixel_host, DEFAULT_HTTP_PORT)
            endpoint = fetch_info(host, port, timeout=2.0)
            if self.device_id and endpoint.device_id != self.device_id:
                raise WifiCollectorError(
                    "指定したPixelのdevice IDが保存済み設定と一致しません"
                )
        else:
            endpoint = self._cached_endpoint(devices)
            if endpoint is None:
                found = discover_devices(self.discover_seconds)
                if self.device_id:
                    found = [item for item in found if item.device_id == self.device_id]
                if len(found) > 1:
                    raise WifiCollectorError(
                        "複数のPixelが見つかりました。--device-idで選択してください"
                    )
                if not found:
                    raise WifiCollectorError(
                        "Pixelへ接続できません。MacとPixelを同じWi-Fiへ接続し、"
                        "Nightfall HFR Recorderを開いてください"
                    )
                endpoint = found[0]

        saved = devices.get(endpoint.device_id)
        if not isinstance(saved, dict) or not isinstance(saved.get("token"), str):
            raise WifiCollectorError(
                "このPixelはMacと未ペアです。先にcollect_wifi_runs.py --pairを実行してください"
            )
        saved["last_host"] = endpoint.host
        saved["http_port"] = endpoint.port
        saved["model"] = endpoint.model
        save_config(self.config_path, config)
        return endpoint, str(saved["token"])

    def _cached_endpoint(self, devices: dict[str, Any]) -> Endpoint | None:
        candidates: list[dict[str, Any]] = []
        for saved_device_id, value in devices.items():
            if self.device_id and saved_device_id != self.device_id:
                continue
            if isinstance(value, dict) and value.get("last_host"):
                candidates.append(value)
        if len(candidates) > 1:
            raise WifiCollectorError(
                "複数のペア済みPixelがあります。--device-idで選択してください"
            )
        if not candidates:
            return None
        saved = candidates[0]
        try:
            return fetch_info(
                str(saved["last_host"]),
                int(saved.get("http_port", DEFAULT_HTTP_PORT)),
                timeout=1.5,
            )
        except WifiCollectorError:
            return None

    def _client(self) -> ApiClient:
        with self._connection_lock:
            if self._endpoint is None or self._token is None:
                self._endpoint, self._token = self._select_endpoint()
            return ApiClient(self._endpoint, self._token, timeout=5.0)

    def _invalidate_connection(self) -> None:
        with self._connection_lock:
            self._endpoint = None
            self._token = None

    def _operation_snapshot(self) -> dict[str, Any]:
        with self._operation_lock:
            return self._operation.to_json()

    def status_payload(self) -> dict[str, Any]:
        try:
            client = self._client()
            response = read_status(client)
            capture = capture_state(response)
            saved_runs = int(response.get("saved_runs", 0))
            acknowledged_runs = int(response.get("acknowledged_runs", 0))
            now = time.monotonic()
            incomplete_retry = (
                self._latest_run is not None
                and not self._latest_run.get("complete")
                and now - self._inventory_checked_monotonic >= 1.0
            )
            if (
                self._inventory_saved_runs != saved_runs
                or self._inventory_acknowledged_runs != acknowledged_runs
                or incomplete_retry
            ):
                try:
                    inventory = client.json_request("GET", "/api/v1/runs")
                    self._latest_run = latest_run_summary(inventory)
                    self._inventory_saved_runs = saved_runs
                    self._inventory_acknowledged_runs = acknowledged_runs
                    self._inventory_checked_monotonic = now
                    self._inventory_error = None
                except WifiCollectorError as exc:
                    self._inventory_checked_monotonic = now
                    self._inventory_error = str(exc)
            recording_elapsed = self._recording_elapsed(str(capture["state"]), now)
            return {
                "online": True,
                "checked_at_unix_ms": int(time.time() * 1000),
                "endpoint": {
                    "host": client.endpoint.host,
                    "port": client.endpoint.port,
                },
                "pixel": response,
                "capture": capture,
                "latest_run": self._latest_run,
                "inventory_error": self._inventory_error,
                "operation": self._operation_snapshot(),
                "recording_warning_seconds": self.recording_warning_seconds,
                "recording_elapsed_seconds": recording_elapsed,
            }
        except WifiCollectorError as exc:
            self._invalidate_connection()
            return {
                "online": False,
                "checked_at_unix_ms": int(time.time() * 1000),
                "error": str(exc),
                "operation": self._operation_snapshot(),
                "recording_warning_seconds": self.recording_warning_seconds,
            }

    def _recording_elapsed(self, state: str, now: float) -> float:
        with self._capture_state_lock:
            if state == "recording":
                if self._last_capture_state != "recording":
                    self._recording_started_monotonic = now
                started = self._recording_started_monotonic or now
                elapsed = max(0.0, now - started)
            else:
                self._recording_started_monotonic = None
                elapsed = 0.0
            self._last_capture_state = state
            return elapsed

    def start_action(self, action: str) -> dict[str, Any]:
        if action not in VALID_ACTIONS:
            raise ValueError(f"unsupported action: {action}")
        with self._operation_lock:
            if self._operation.state == "running":
                raise WifiCollectorError(
                    f"{self._operation.name}を実行中です。完了まで待ってください"
                )
            sequence = self._operation.sequence + 1
            self._operation = Operation(
                sequence=sequence,
                name=action,
                state="running",
                message=self._action_label(action) + "を開始しました",
                started_at_unix_ms=int(time.time() * 1000),
            )
        worker = threading.Thread(
            target=self._run_action,
            args=(sequence, action),
            name=f"hfr-dashboard-{action}",
            daemon=True,
        )
        worker.start()
        return self._operation_snapshot()

    def _run_action(self, sequence: int, action: str) -> None:
        try:
            client = self._client()
            messages: list[str] = []
            if action == "start":
                client.json_request("POST", CONTROL_START_PATH, {})
                response = wait_for_start(client, self.wait_seconds)
                messages.append(
                    f"連続撮影スタンバイ開始: {capture_state(response)['state']}"
                )
            if action in {"stop", "stop-and-collect"}:
                client.json_request("POST", CONTROL_STOP_PATH, {})
                response = wait_for_stop(client, self.wait_seconds)
                messages.append(
                    "連続撮影スタンバイ終了"
                    f"（{capture_state(response).get('completed_runs', 0)}本保存）"
                )
            if action in {"collect", "stop-and-collect"}:
                response = read_status(client)
                if response.get("capture_busy"):
                    raise CaptureBusyError(
                        "撮影待機中は転送できません。先に待機終了を実行してください"
                    )
                collected, skipped = collect_all(client, self.output_root)
                messages.append(
                    f"Macへの転送完了: 新規{collected}本・既存{skipped}本"
                )
            self._finish_action(sequence, "success", " / ".join(messages))
        except (WifiCollectorError, OSError) as exc:
            self._finish_action(sequence, "error", str(exc))

    def _finish_action(self, sequence: int, state: str, message: str) -> None:
        with self._operation_lock:
            if self._operation.sequence != sequence:
                return
            self._operation.state = state
            self._operation.message = message
            self._operation.finished_at_unix_ms = int(time.time() * 1000)

    @staticmethod
    def _action_label(action: str) -> str:
        return {
            "start": "待機開始",
            "stop": "待機終了",
            "collect": "Macへ転送",
            "stop-and-collect": "待機終了と転送",
        }[action]


def make_handler(
    dashboard: PixelDashboard,
    csrf_token: str,
    *,
    verbose: bool = False,
) -> type[BaseHTTPRequestHandler]:
    class DashboardHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, format_string: str, *args: object) -> None:
            if verbose:
                super().log_message(format_string, *args)

        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            if path == "/api/status":
                self._send_json(200, dashboard.status_payload())
                return
            asset = STATIC_FILES.get(path)
            if asset is None:
                self._send_json(404, {"error": "not found"})
                return
            filename, content_type = asset
            try:
                payload = (DASHBOARD_DIR / filename).read_bytes()
            except OSError as exc:
                self._send_json(500, {"error": f"asset unavailable: {exc}"})
                return
            if filename == "index.html":
                payload = payload.replace(
                    b"__NIGHTFALL_CSRF_TOKEN__",
                    csrf_token.encode("ascii"),
                )
            self._send_bytes(200, payload, content_type)

        def do_POST(self) -> None:
            path = urlsplit(self.path).path
            prefix = "/api/action/"
            if not path.startswith(prefix):
                self._send_json(404, {"error": "not found"})
                return
            if self.headers.get("X-Nightfall-Dashboard-Token") != csrf_token:
                self._send_json(403, {"error": "dashboard token required"})
                return
            length_text = self.headers.get("Content-Length", "0")
            try:
                length = int(length_text)
            except ValueError:
                self._send_json(400, {"error": "invalid content length"})
                return
            if length < 0 or length > 4096:
                self._send_json(413, {"error": "request body too large"})
                return
            if length:
                self.rfile.read(length)
            action = path[len(prefix):]
            try:
                operation = dashboard.start_action(action)
            except ValueError as exc:
                self._send_json(404, {"error": str(exc)})
                return
            except WifiCollectorError as exc:
                self._send_json(409, {"error": str(exc)})
                return
            self._send_json(202, {"accepted": True, "operation": operation})

        def _send_json(self, status: int, value: dict[str, Any]) -> None:
            payload = json.dumps(value, ensure_ascii=False).encode("utf-8")
            self._send_bytes(status, payload, "application/json; charset=utf-8")

        def _send_bytes(
            self,
            status: int,
            payload: bytes,
            content_type: str,
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Referrer-Policy", "no-referrer")
            self.send_header(
                "Content-Security-Policy",
                "default-src 'self'; script-src 'self'; style-src 'self'; "
                "connect-src 'self'; img-src 'self' data:; base-uri 'none'; "
                "frame-ancestors 'none'",
            )
            self.end_headers()
            self.wfile.write(payload)

    return DashboardHandler


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Mac上でPixel HFR撮影状態を常時表示・操作します。",
    )
    parser.add_argument("--listen", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--host", help="Pixel IP[:port]（通常は自動接続）")
    parser.add_argument("--device-id", help="複数のペア済みPixelがある場合のID")
    parser.add_argument(
        "--config",
        type=Path,
        default=default_config_path(),
        help="ペア情報ファイル",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help="Macへの動画転送先",
    )
    parser.add_argument("--discover-seconds", type=float, default=1.0)
    parser.add_argument("--wait-seconds", type=float, default=90.0)
    parser.add_argument(
        "--recording-warning-seconds",
        type=float,
        default=15.0,
        help="STOP信号見逃し警告を出す録画継続秒数",
    )
    parser.add_argument("--no-open", action="store_true", help="ブラウザを自動で開かない")
    parser.add_argument("--verbose-http", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if not (0 <= args.port <= 65535):
        print("[HFR-DASHBOARD][ERROR] --portは0..65535で指定してください", file=sys.stderr)
        return 2
    dashboard = PixelDashboard(
        config_path=args.config,
        output_root=args.output_root,
        pixel_host=args.host,
        device_id=args.device_id,
        discover_seconds=args.discover_seconds,
        wait_seconds=args.wait_seconds,
        recording_warning_seconds=args.recording_warning_seconds,
    )
    csrf_token = secrets.token_urlsafe(32)
    handler = make_handler(dashboard, csrf_token, verbose=args.verbose_http)
    try:
        server = ThreadingHTTPServer((args.listen, args.port), handler)
    except OSError as exc:
        print(f"[HFR-DASHBOARD][ERROR] 画面を開始できません: {exc}", file=sys.stderr)
        return 2
    actual_host, actual_port = server.server_address[:2]
    browser_host = "127.0.0.1" if actual_host in {"0.0.0.0", "::"} else actual_host
    url = f"http://{browser_host}:{actual_port}/"
    print(f"[HFR-DASHBOARD] {url}")
    print("[HFR-DASHBOARD] 終了するにはこのターミナルでControl-Cを押します")
    if not args.no_open:
        threading.Timer(0.3, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\n[HFR-DASHBOARD] 終了しました")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
