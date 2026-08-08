#!/usr/bin/env python3
"""Tests for the Mac-side Pixel HFR monitoring dashboard."""

from __future__ import annotations

import json
import sys
import threading
import unittest
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1]
PROBE_DIR = TOOLS_DIR / "android_camera_probe"
sys.path.insert(0, str(PROBE_DIR))

import hfr_dashboard as dashboard  # noqa: E402


class FakeDashboard:
    def __init__(self) -> None:
        self.actions: list[str] = []

    def status_payload(self) -> dict[str, object]:
        return {
            "online": True,
            "checked_at_unix_ms": 1234,
            "capture": {"state": "armed", "completed_runs": 2},
        }

    def start_action(self, action: str) -> dict[str, object]:
        if action not in dashboard.VALID_ACTIONS:
            raise ValueError("unsupported action")
        self.actions.append(action)
        return {"sequence": len(self.actions), "name": action, "state": "running"}


class DashboardTest(unittest.TestCase):
    def test_latest_run_requires_complete_artifact_set(self) -> None:
        complete = dashboard.latest_run_summary({
            "runs": [{
                "name": "manual-20-run0001",
                "status": "complete",
                "acknowledged": False,
                "artifacts": [
                    {"name": "hfr_report.json", "size": 500},
                    {"name": "capture_results.jsonl", "size": 1000},
                    {"name": "encoder_samples.jsonl", "size": 1000},
                    {"name": "hfr_capture.mp4", "size": 12_000_000},
                ],
            }],
        })
        self.assertIsNotNone(complete)
        self.assertTrue(complete["complete"])
        self.assertEqual(complete["video_bytes"], 12_000_000)

        incomplete = dashboard.latest_run_summary({
            "runs": [{
                "name": "manual-20-run0002",
                "status": "complete",
                "artifacts": [
                    {"name": "hfr_report.json", "size": 500},
                    {"name": "hfr_capture.mp4", "size": 12_000_000},
                ],
            }],
        })
        self.assertIsNotNone(incomplete)
        self.assertFalse(incomplete["complete"])
        self.assertEqual(
            incomplete["missing_artifacts"],
            ["capture_results.jsonl", "encoder_samples.jsonl"],
        )

    def test_latest_run_uses_newest_nonce(self) -> None:
        summary = dashboard.latest_run_summary({
            "runs": [
                {"name": "manual-10-run0009", "status": "cancelled"},
                {"name": "manual-20-run0001", "status": "complete"},
            ],
        })
        self.assertEqual(summary["name"], "manual-20-run0001")

    def test_dashboard_serves_ui_status_and_protects_actions(self) -> None:
        model = FakeDashboard()
        token = "test-csrf-token"
        handler = dashboard.make_handler(model, token)
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        try:
            connection = HTTPConnection("127.0.0.1", server.server_port)
            connection.request("GET", "/")
            response = connection.getresponse()
            page = response.read().decode("utf-8")
            self.assertEqual(response.status, 200)
            self.assertIn("走行撮影モニター", page)
            self.assertIn(token, page)
            self.assertIn("default-src 'self'", response.getheader("Content-Security-Policy"))
            connection.close()

            connection = HTTPConnection("127.0.0.1", server.server_port)
            connection.request("GET", "/api/status")
            response = connection.getresponse()
            status = json.loads(response.read())
            self.assertEqual(response.status, 200)
            self.assertTrue(status["online"])
            connection.close()

            connection = HTTPConnection("127.0.0.1", server.server_port)
            connection.request("POST", "/api/action/start", body=b"{}")
            response = connection.getresponse()
            response.read()
            self.assertEqual(response.status, 403)
            self.assertEqual(model.actions, [])
            connection.close()

            connection = HTTPConnection("127.0.0.1", server.server_port)
            connection.request(
                "POST",
                "/api/action/start",
                body=b"{}",
                headers={
                    "Content-Type": "application/json",
                    "X-Nightfall-Dashboard-Token": token,
                },
            )
            response = connection.getresponse()
            accepted = json.loads(response.read())
            self.assertEqual(response.status, 202)
            self.assertTrue(accepted["accepted"])
            self.assertEqual(model.actions, ["start"])
            connection.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_dashboard_assets_exist(self) -> None:
        for filename, _content_type in dashboard.STATIC_FILES.values():
            self.assertTrue((dashboard.DASHBOARD_DIR / filename).is_file())


if __name__ == "__main__":
    unittest.main()
