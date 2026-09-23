#!/usr/bin/env python3

from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[3]
VISION_ROOT = REPO_ROOT / "tools/vision"
sys.path.insert(0, str(VISION_ROOT))

import fuse_trace_video as fuse  # noqa: E402


class FusedTracePreservationTest(unittest.TestCase):
    @staticmethod
    def _series_and_alignment(
        output_time_s: np.ndarray,
    ) -> tuple[fuse.Series, fuse.Series, fuse.Alignment]:
        video = fuse.Series(
            time_s=np.asarray([0.0, 1.0]),
            signal=np.zeros(2),
            x_mm=np.asarray([10.0, 20.0]),
            y_mm=np.asarray([30.0, 40.0]),
            yaw_deg=np.asarray([0.0, 90.0]),
            speed_mm_s=np.asarray([0.0, 100.0]),
            omega_dps=np.asarray([0.0, 90.0]),
        )
        trace = fuse.Series(
            time_s=np.asarray([0.0, 1.0]),
            signal=np.zeros(2),
            output_time_s=output_time_s,
        )
        alignment = fuse.Alignment(
            offset_s=0.0,
            scale=1.0,
            sign=1,
            correlation=1.0,
            overlap_s=1.0,
            matched_samples=2,
            signal_gain=1.0,
            signal_bias=0.0,
            signal_rmse=0.0,
            normalized_rmse=0.0,
            second_correlation=0.0,
            correlation_margin=1.0,
        )
        return video, trace, alignment

    def test_output_may_not_overwrite_either_input(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trajectory = root / "trajectory.csv"
            trace = root / "fused.csv"
            trajectory.touch()
            trace.touch()
            with self.assertRaisesRegex(
                ValueError,
                "refusing to overwrite input CSV",
            ):
                fuse._validate_output_does_not_alias_input(
                    root / "fused.csv",
                    (trajectory, trace),
                )

    def test_all_rows_and_source_columns_are_preserved(self):
        video, trace, alignment = self._series_and_alignment(
            np.asarray([0.0, np.nan, 1.0]),
        )
        rows = [
            {
                "timestamp_ms": "1000",
                "seq": "1",
                "event_type": "start",
                "gyro_z_raw_mdps": "10",
            },
            {
                "timestamp_ms": "",
                "seq": "2",
                "event_type": "metadata",
                "gyro_z_raw_mdps": "",
            },
            {
                "timestamp_ms": "2000",
                "seq": "3",
                "event_type": "stop",
                "gyro_z_raw_mdps": "20",
            },
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fused.csv"
            fuse.write_fused_csv(path, video, trace, rows, alignment)
            with path.open(newline="", encoding="ascii") as stream:
                output = list(csv.DictReader(stream))
        self.assertEqual(len(output), 3)
        self.assertEqual(output[1]["event_type"], "metadata")
        self.assertEqual(output[2]["gyro_z_raw_mdps"], "20")
        self.assertEqual(output[1]["video_in_range"], "0")

    def test_extra_values_and_generated_name_collisions_are_preserved(self):
        video, trace, alignment = self._series_and_alignment(
            np.asarray([0.0, 1.0]),
        )
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            source_path = directory_path / "source.csv"
            source_path.write_text(
                "timestamp_ms,video_in_range,fused_video_in_range,"
                "trace_unnamed_extra_1\n"
                "1000,source-a,source-fused-a,source-extra-name-a,"
                "unnamed-a-1,unnamed-a-2\n"
                "2000,source-b,source-fused-b,source-extra-name-b,"
                "unnamed-b-1,unnamed-b-2\n",
                encoding="ascii",
            )
            _, rows = fuse._read_csv(source_path)
            self.assertEqual(
                rows[0][None],  # type: ignore[index]
                ["unnamed-a-1", "unnamed-a-2"],
            )

            output_path = directory_path / "fused.csv"
            generated_columns = fuse.write_fused_csv(
                output_path,
                video,
                trace,
                rows,
                alignment,
            )
            with output_path.open(newline="", encoding="ascii") as stream:
                reader = csv.DictReader(stream)
                output_fields = reader.fieldnames
                output = list(reader)

        assert output_fields is not None
        self.assertEqual(len(output_fields), len(set(output_fields)))
        self.assertEqual(output[0]["video_in_range"], "source-a")
        self.assertEqual(
            output[0]["fused_video_in_range"],
            "source-fused-a",
        )
        self.assertEqual(
            output[0]["trace_unnamed_extra_1"],
            "source-extra-name-a",
        )
        self.assertEqual(
            generated_columns["video_in_range"],
            "fused_video_in_range_2",
        )
        self.assertEqual(output[0]["fused_video_in_range_2"], "1")
        self.assertEqual(
            output[0]["trace_unnamed_extra_1_2"],
            "unnamed-a-1",
        )
        self.assertEqual(
            output[0]["trace_unnamed_extra_2"],
            "unnamed-a-2",
        )
        self.assertEqual(
            output[1]["trace_unnamed_extra_1_2"],
            "unnamed-b-1",
        )
        self.assertEqual(
            output[1]["trace_unnamed_extra_2"],
            "unnamed-b-2",
        )


if __name__ == "__main__":
    unittest.main()
