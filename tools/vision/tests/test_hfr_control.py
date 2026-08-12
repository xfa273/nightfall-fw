#!/usr/bin/env python3
"""Tests for current-run-only Pixel HFR capture control."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


TOOLS_DIR = Path(__file__).resolve().parents[1]
PROBE_DIR = TOOLS_DIR / "android_camera_probe"
sys.path.insert(0, str(PROBE_DIR))

import hfr_control as control  # noqa: E402


class FakeClient:
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


def response(
    state: str,
    *,
    continuous: bool,
    completed: int,
    mode: str | None = None,
    busy: bool = True,
) -> dict[str, object]:
    if mode is None:
        mode = (
            control.CAPTURE_MODE_CONTINUOUS_OPTICAL
            if continuous
            else control.CAPTURE_MODE_IDLE
        )
    return {
        "capture_busy": busy,
        "capture_control": {
            "state": state,
            "capture_mode": mode,
            "continuous_standby": continuous,
            "recording": state in {"recording", "finishing-run"},
            "completed_runs": completed,
            "message": state,
        },
    }


class CurrentRunFinishTest(unittest.TestCase):
    @patch.object(control.time, "sleep", return_value=None)
    def test_waits_for_save_and_preserves_continuous_standby(
        self,
        _sleep: object,
    ) -> None:
        client = FakeClient([
            response("finishing-run", continuous=True, completed=3),
            response("rearming", continuous=True, completed=4),
        ])
        result = control.wait_for_current_run_finish(client, 2.0, 3)
        state = control.capture_state(result)
        self.assertEqual(state["state"], "rearming")
        self.assertTrue(state["continuous_standby"])
        self.assertEqual(state["completed_runs"], 4)

    def test_rejects_unexpected_standby_shutdown(self) -> None:
        client = FakeClient([
            response("idle", continuous=False, completed=4),
        ])
        with self.assertRaisesRegex(
            control.WifiCollectorError,
            "連続撮影スタンバイが解除",
        ):
            control.wait_for_current_run_finish(client, 2.0, 3)


class ContinuousStandbyRegressionTest(unittest.TestCase):
    @patch.object(control.time, "sleep", return_value=None)
    def test_existing_start_wait_accepts_optical_armed(
        self,
        _sleep: object,
    ) -> None:
        client = FakeClient([
            response("starting", continuous=True, completed=0),
            response("armed", continuous=True, completed=0),
        ])
        result = control.wait_for_start(client, 2.0)
        state = control.capture_state(result)
        self.assertEqual(state["state"], "armed")
        self.assertEqual(
            state["capture_mode"],
            control.CAPTURE_MODE_CONTINUOUS_OPTICAL,
        )
        self.assertTrue(state["continuous_standby"])

    @patch.object(control.time, "sleep", return_value=None)
    def test_existing_stop_waits_for_idle_and_lease_release(
        self,
        _sleep: object,
    ) -> None:
        client = FakeClient([
            response(
                "stopping",
                continuous=False,
                completed=3,
                mode=control.CAPTURE_MODE_CONTINUOUS_OPTICAL,
            ),
            response(
                "idle",
                continuous=False,
                completed=3,
                busy=False,
            ),
        ])
        result = control.wait_for_stop(client, 2.0)
        self.assertEqual(control.capture_state(result)["state"], "idle")
        self.assertFalse(result["capture_busy"])


class ManualOneShotWaitTest(unittest.TestCase):
    @patch.object(control.time, "sleep", return_value=None)
    def test_waits_for_immediate_manual_recording(
        self,
        _sleep: object,
    ) -> None:
        client = FakeClient([
            response(
                "starting",
                continuous=False,
                completed=0,
                mode=control.CAPTURE_MODE_MANUAL_ONE_SHOT,
            ),
            response(
                "recording",
                continuous=False,
                completed=0,
                mode=control.CAPTURE_MODE_MANUAL_ONE_SHOT,
            ),
        ])
        result = control.wait_for_manual_start(client, 2.0)
        state = control.capture_state(result)
        self.assertEqual(state["state"], "recording")
        self.assertEqual(
            state["capture_mode"],
            control.CAPTURE_MODE_MANUAL_ONE_SHOT,
        )
        self.assertFalse(state["continuous_standby"])
        self.assertTrue(result["capture_busy"])

    def test_manual_start_rejects_optical_armed_state(self) -> None:
        client = FakeClient([
            response(
                "armed",
                continuous=True,
                completed=0,
                mode=control.CAPTURE_MODE_CONTINUOUS_OPTICAL,
            ),
        ])
        with self.assertRaisesRegex(
            control.WifiCollectorError,
            "capture_mode",
        ):
            control.wait_for_manual_start(client, 2.0)

    def test_manual_start_requires_capture_lease(self) -> None:
        client = FakeClient([
            response(
                "recording",
                continuous=False,
                completed=0,
                mode=control.CAPTURE_MODE_MANUAL_ONE_SHOT,
                busy=False,
            ),
        ])
        with self.assertRaisesRegex(
            control.WifiCollectorError,
            "撮影排他",
        ):
            control.wait_for_manual_start(client, 2.0)

    @patch.object(control.time, "sleep", return_value=None)
    def test_manual_stop_waits_for_saved_run_idle_and_lease_release(
        self,
        _sleep: object,
    ) -> None:
        client = FakeClient([
            response(
                "stopping",
                continuous=False,
                completed=2,
                mode=control.CAPTURE_MODE_MANUAL_ONE_SHOT,
            ),
            response(
                "idle",
                continuous=False,
                completed=3,
                mode=control.CAPTURE_MODE_IDLE,
                busy=False,
            ),
        ])
        result = control.wait_for_manual_stop(client, 2.0, 2)
        state = control.capture_state(result)
        self.assertEqual(state["state"], "idle")
        self.assertEqual(state["capture_mode"], control.CAPTURE_MODE_IDLE)
        self.assertEqual(state["completed_runs"], 3)
        self.assertFalse(result["capture_busy"])

    def test_manual_stop_rejects_idle_without_saved_run(self) -> None:
        client = FakeClient([
            response(
                "idle",
                continuous=False,
                completed=2,
                mode=control.CAPTURE_MODE_IDLE,
                busy=False,
            ),
        ])
        with self.assertRaisesRegex(
            control.WifiCollectorError,
            "保存本数が増えていません",
        ):
            control.wait_for_manual_stop(client, 2.0, 2)


class ManualOneShotCliTest(unittest.TestCase):
    def run_main(
        self,
        action: str,
        client: FakeClient,
    ) -> int:
        endpoint = SimpleNamespace(host="pixel.test", port=8088)
        with (
            patch.object(control, "load_config", return_value={}),
            patch.object(control, "choose_endpoint", return_value=endpoint),
            patch.object(control, "configured_token", return_value="token"),
            patch.object(control, "ApiClient", return_value=client),
        ):
            return control.main([action, "--config", "/tmp/test-config.json"])

    def test_manual_start_posts_dedicated_endpoint(self) -> None:
        client = FakeClient([
            {},
            response(
                "recording",
                continuous=False,
                completed=0,
                mode=control.CAPTURE_MODE_MANUAL_ONE_SHOT,
            ),
        ])
        self.assertEqual(self.run_main("manual-start", client), 0)
        self.assertEqual(
            client.requests[0],
            ("POST", control.CONTROL_MANUAL_START_PATH, {}),
        )

    def test_manual_stop_posts_dedicated_endpoint_and_waits_for_save(
        self,
    ) -> None:
        client = FakeClient([
            response(
                "recording",
                continuous=False,
                completed=0,
                mode=control.CAPTURE_MODE_MANUAL_ONE_SHOT,
            ),
            {},
            response(
                "idle",
                continuous=False,
                completed=1,
                mode=control.CAPTURE_MODE_IDLE,
                busy=False,
            ),
        ])
        self.assertEqual(self.run_main("manual-stop", client), 0)
        self.assertEqual(
            client.requests[1],
            ("POST", control.CONTROL_MANUAL_STOP_PATH, {}),
        )

    def test_manual_stop_rejects_continuous_mode_without_posting(self) -> None:
        client = FakeClient([
            response(
                "armed",
                continuous=True,
                completed=0,
                mode=control.CAPTURE_MODE_CONTINUOUS_OPTICAL,
            ),
        ])
        self.assertEqual(self.run_main("manual-stop", client), 2)
        self.assertEqual(
            client.requests,
            [("GET", control.CONTROL_STATUS_PATH, None)],
        )


if __name__ == "__main__":
    unittest.main()
