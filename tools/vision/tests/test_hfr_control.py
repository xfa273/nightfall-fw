#!/usr/bin/env python3
"""Tests for current-run-only Pixel HFR capture control."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch


TOOLS_DIR = Path(__file__).resolve().parents[1]
PROBE_DIR = TOOLS_DIR / "android_camera_probe"
sys.path.insert(0, str(PROBE_DIR))

import hfr_control as control  # noqa: E402


class FakeClient:
    def __init__(self, responses: list[dict[str, object]]) -> None:
        self.responses = iter(responses)

    def json_request(
        self,
        method: str,
        path: str,
        body: object | None = None,
    ) -> dict[str, object]:
        del method, path, body
        return next(self.responses)


def response(state: str, *, continuous: bool, completed: int) -> dict[str, object]:
    return {
        "capture_busy": True,
        "capture_control": {
            "state": state,
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


if __name__ == "__main__":
    unittest.main()
