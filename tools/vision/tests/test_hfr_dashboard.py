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
from unittest.mock import patch


TOOLS_DIR = Path(__file__).resolve().parents[1]
PROBE_DIR = TOOLS_DIR / "android_camera_probe"
sys.path.insert(0, str(PROBE_DIR))

import hfr_dashboard as dashboard  # noqa: E402


class FakePixelClient:
    def __init__(self, responses: list[dict[str, object]]) -> None:
        self.responses = iter(responses)
        self.requests: list[tuple[str, str, object | None]] = []

    def json_request(
        self,
        method: str,
        path: str,
        body: object | None = None,
    ) -> dict[str, object]:
        self.requests.append((method, path, body))
        return next(self.responses)


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

    def test_latest_run_order_survives_mixed_one_shot_names(self) -> None:
        summary = dashboard.latest_run_summary({
            "runs": [
                {
                    "name": "manual-1785641124561-one-shot-run0001",
                    "status": "complete",
                },
                {
                    "name": "manual-1785641124562-run0001",
                    "status": "complete",
                },
            ],
        })
        self.assertEqual(
            summary["name"],
            "manual-1785641124562-run0001",
        )

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
            self.assertIn("この撮影だけ終了", page)
            self.assertIn('data-action="manual-start"', page)
            self.assertIn("手動録画を開始", page)
            self.assertIn('data-action="manual-stop"', page)
            self.assertIn("手動録画を終了", page)
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

            connection = HTTPConnection("127.0.0.1", server.server_port)
            connection.request(
                "POST",
                "/api/action/finish-run",
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
            self.assertEqual(model.actions, ["start", "finish-run"])
            connection.close()

            for manual_action in ("manual-start", "manual-stop"):
                connection = HTTPConnection("127.0.0.1", server.server_port)
                connection.request(
                    "POST",
                    f"/api/action/{manual_action}",
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
                connection.close()
            self.assertEqual(
                model.actions,
                ["start", "finish-run", "manual-start", "manual-stop"],
            )
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_dashboard_assets_exist(self) -> None:
        for filename, _content_type in dashboard.STATIC_FILES.values():
            self.assertTrue((dashboard.DASHBOARD_DIR / filename).is_file())

    def test_dashboard_registers_manual_one_shot_actions(self) -> None:
        self.assertIn("manual-start", dashboard.VALID_ACTIONS)
        self.assertIn("manual-stop", dashboard.VALID_ACTIONS)
        javascript = (
            dashboard.DASHBOARD_DIR / "dashboard.js"
        ).read_text(encoding="utf-8")
        self.assertIn('captureMode === "manual_one_shot"', javascript)
        self.assertIn("elements.manualStartButton.disabled", javascript)
        self.assertIn("elements.manualStopButton.disabled", javascript)
        self.assertIn("!continuousOptical", javascript)

    def test_dashboard_manual_actions_use_dedicated_pixel_endpoints(self) -> None:
        model = dashboard.PixelDashboard(
            config_path=Path("/tmp/unused-hfr-config.json"),
            output_root=Path("/tmp/unused-hfr-output"),
            pixel_host=None,
            device_id=None,
            discover_seconds=0.0,
            wait_seconds=2.0,
            recording_warning_seconds=15.0,
        )
        model._operation = dashboard.Operation(
            sequence=1,
            name="manual-start",
            state="running",
        )
        start_client = FakePixelClient([{}])
        recording = {
            "capture_busy": True,
            "capture_control": {
                "state": "recording",
                "capture_mode": "manual_one_shot",
                "continuous_standby": False,
                "recording": True,
                "completed_runs": 0,
            },
        }
        with (
            patch.object(model, "_client", return_value=start_client),
            patch.object(
                dashboard,
                "wait_for_manual_start",
                return_value=recording,
            ),
        ):
            model._run_action(1, "manual-start")
        self.assertEqual(
            start_client.requests,
            [("POST", dashboard.CONTROL_MANUAL_START_PATH, {})],
        )
        self.assertEqual(model._operation.state, "success")

        model._operation = dashboard.Operation(
            sequence=2,
            name="manual-stop",
            state="running",
        )
        stop_client = FakePixelClient([recording, {}])
        idle = {
            "capture_busy": False,
            "capture_control": {
                "state": "idle",
                "capture_mode": "idle",
                "continuous_standby": False,
                "recording": False,
                "completed_runs": 1,
            },
        }
        with (
            patch.object(model, "_client", return_value=stop_client),
            patch.object(
                dashboard,
                "wait_for_manual_stop",
                return_value=idle,
            ) as wait_stop,
        ):
            model._run_action(2, "manual-stop")
        self.assertEqual(
            stop_client.requests,
            [
                ("GET", dashboard.CONTROL_STATUS_PATH, None),
                ("POST", dashboard.CONTROL_MANUAL_STOP_PATH, {}),
            ],
        )
        wait_stop.assert_called_once_with(stop_client, 2.0, 0)
        self.assertEqual(model._operation.state, "success")


if __name__ == "__main__":
    unittest.main()
