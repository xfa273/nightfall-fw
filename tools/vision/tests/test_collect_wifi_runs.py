#!/usr/bin/env python3
"""Tests for authenticated, resumable Pixel Wi-Fi collection."""

from __future__ import annotations

import json
import sys
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock
from urllib.parse import unquote


TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR / "android_camera_probe"))

import collect_wifi_runs as wifi  # noqa: E402


class FakePixelHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    token = "a" * 64
    pair_code = "123456"
    run_name = "manual-1000-run0001"
    source_files: dict[str, bytes] = {}
    acknowledged: list[str] = []
    ranges: list[tuple[str, str]] = []

    def log_message(self, format_string: str, *args: object) -> None:
        pass

    def do_GET(self) -> None:
        if self.path == "/api/v1/info":
            self.send_json(self.info())
            return
        if self.headers.get("Authorization") != f"Bearer {self.token}":
            self.send_json({"schema": wifi.SCHEMA, "error": "auth"}, 401)
            return
        if self.path == "/api/v1/runs":
            artifacts = [
                {"name": name, "size": len(payload)}
                for name, payload in self.source_files.items()
            ]
            body = self.info()
            body["runs"] = [{
                "name": self.run_name,
                "status": "complete",
                "acknowledged": False,
                "artifacts": artifacts,
            }]
            self.send_json(body)
            return
        prefix = f"/api/v1/runs/{self.run_name}/"
        if self.path.startswith(prefix):
            artifact = unquote(self.path[len(prefix):])
            payload = self.source_files.get(artifact)
            if payload is None:
                self.send_json({"schema": wifi.SCHEMA, "error": "missing"}, 404)
                return
            start = 0
            range_header = self.headers.get("Range")
            if range_header:
                self.ranges.append((artifact, range_header))
                start = int(range_header.removeprefix("bytes=").removesuffix("-"))
            body = payload[start:]
            status = 206 if start else 200
            self.send_response(status)
            self.send_header("Content-Length", str(len(body)))
            if start:
                self.send_header(
                    "Content-Range",
                    f"bytes {start}-{len(payload) - 1}/{len(payload)}",
                )
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_json({"schema": wifi.SCHEMA, "error": "not found"}, 404)

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        body = json.loads(self.rfile.read(length)) if length else {}
        if self.path == "/api/v1/pair":
            if body.get("code") != self.pair_code:
                self.send_json({"schema": wifi.SCHEMA, "error": "pair"}, 403)
                return
            response = self.info()
            response["access_token"] = self.token
            self.send_json(response)
            return
        if self.headers.get("Authorization") != f"Bearer {self.token}":
            self.send_json({"schema": wifi.SCHEMA, "error": "auth"}, 401)
            return
        prefix = f"/api/v1/runs/{self.run_name}/ack"
        if self.path == prefix:
            self.acknowledged.append(self.run_name)
            self.send_json({
                "schema": wifi.SCHEMA,
                "run_name": self.run_name,
                "acknowledged_at_unix_ms": 1,
            })
            return
        self.send_json({"schema": wifi.SCHEMA, "error": "not found"}, 404)

    def info(self) -> dict[str, object]:
        return {
            "schema": wifi.SCHEMA,
            "device_id": "12345678-abcd-0000-0000-123456789abc",
            "model": "Pixel 8",
            "app_version": "0.4.0",
            "http_port": self.server.server_port,
            "paired": True,
            "saved_runs": 1,
            "acknowledged_runs": 0,
            "capture_busy": False,
            "active_transfers": 0,
        }

    def send_json(self, value: dict[str, object], status: int = 200) -> None:
        payload = json.dumps(value).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


class WifiCollectorTest(unittest.TestCase):
    def setUp(self) -> None:
        report = {
            "schema": wifi.REPORT_SCHEMA,
            "record_nonce": FakePixelHandler.run_name,
            "status": "complete",
        }
        FakePixelHandler.source_files = {
            "hfr_report.json": json.dumps(report).encode("utf-8"),
            "capture_results.jsonl": b'{"frame":1}\n',
            "encoder_samples.jsonl": b'{"sample":1}\n',
            "hfr_capture.mp4": b"mock-mp4-payload-" * 1024,
        }
        FakePixelHandler.acknowledged = []
        FakePixelHandler.ranges = []
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), FakePixelHandler)
        self.thread = threading.Thread(target=self.server.serve_forever)
        self.thread.start()

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def test_pair_resume_collect_ack_and_idempotency(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config_path = root / "config.json"
            output = root / "runs"
            endpoint = wifi.fetch_info("127.0.0.1", self.server.server_port)
            config = wifi.load_config(config_path)
            token = wifi.pair_device(
                endpoint,
                FakePixelHandler.pair_code,
                config,
                config_path,
            )
            self.assertEqual(token, FakePixelHandler.token)
            self.assertEqual(config_path.stat().st_mode & 0o777, 0o600)

            model = wifi.safe_component(endpoint.model)
            device = wifi.safe_component(endpoint.device_id[:12])
            target_name = f"{model}_wifi-{device}_{FakePixelHandler.run_name}"
            partial = output / f".{target_name}.partial"
            partial.mkdir(parents=True)
            video = FakePixelHandler.source_files["hfr_capture.mp4"]
            resume_bytes = 257
            (partial / "hfr_capture.mp4.part").write_bytes(video[:resume_bytes])

            client = wifi.ApiClient(endpoint, token)
            with mock.patch.object(wifi.shutil, "which", return_value=None):
                collected, skipped = wifi.collect_all(client, output)
            self.assertEqual((collected, skipped), (1, 0))
            final = output / target_name
            self.assertEqual((final / "hfr_capture.mp4").read_bytes(), video)
            self.assertIn(
                ("hfr_capture.mp4", f"bytes={resume_bytes}-"),
                FakePixelHandler.ranges,
            )
            self.assertEqual(FakePixelHandler.acknowledged, [FakePixelHandler.run_name])

            with mock.patch.object(wifi.shutil, "which", return_value=None):
                collected, skipped = wifi.collect_all(client, output)
            self.assertEqual((collected, skipped), (0, 1))
            self.assertEqual(
                FakePixelHandler.acknowledged,
                [FakePixelHandler.run_name, FakePixelHandler.run_name],
            )

    def test_parse_host(self) -> None:
        self.assertEqual(wifi.parse_host("192.0.2.4"), ("192.0.2.4", 46052))
        self.assertEqual(wifi.parse_host("192.0.2.4:1234"), ("192.0.2.4", 1234))


if __name__ == "__main__":
    unittest.main()
