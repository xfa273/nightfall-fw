#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[3]
VISION_ROOT = REPO_ROOT / "tools/vision"
sys.path.insert(0, str(VISION_ROOT))

import video_timing_qa as timing  # noqa: E402


class VideoTimingQaTest(unittest.TestCase):
    def test_unknown_declared_frame_rate_is_not_an_exception(self):
        self.assertIsNone(timing._fraction("0/0"))
        self.assertIsNone(timing._fraction("N/A"))
        self.assertAlmostEqual(timing._fraction("120/1"), 120.0)

    def test_short_pts_interval_fails_cadence_deviation_gate(self):
        payload = {
            "streams": [
                {
                    "codec_name": "h264",
                    "width": 1920,
                    "height": 1080,
                    "r_frame_rate": "120/1",
                    "avg_frame_rate": "120/1",
                    "time_base": "1/120000",
                    "duration": "0.020833",
                    "nb_frames": "4",
                }
            ],
            "frames": [
                {"best_effort_timestamp_time": value}
                for value in ("0", "0.008333", "0.012500", "0.020833")
            ],
        }
        args = argparse.Namespace(
            expected_fps=120.0,
            fps_tolerance_percent=1.0,
            maximum_gap_rate=1.0,
            maximum_cadence_deviation_percent=10.0,
            maximum_cadence_deviation_rate=0.0,
            maximum_content_duplicate_rate=0.0,
            skip_content_check=True,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.mp4"
            path.touch()
            with mock.patch.object(
                timing,
                "ffprobe",
                return_value=payload,
            ):
                report = timing.analyze(path, args)
        self.assertEqual(
            report["pts"]["intervals_over_maximum_cadence_deviation"],
            1,
        )
        self.assertFalse(report["qa"]["passed"])


if __name__ == "__main__":
    unittest.main()
