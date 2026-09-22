"""Multi-run decode and production dump/capture tests; fake UART only."""
import csv
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import time
import unittest

import trace_bin_dump as decoder
import serial_capture_csv as capture
from analyze_turn_csv import _infer_target


def read_csv(path):
    meta = {}
    data = []
    columns = []
    for line in path.read_text().splitlines():
        if line.startswith('#') and '=' in line:
            key, value = line[1:].split('=', 1)
            meta[key] = value
            if key == 'mm_columns':
                columns = value.split(',')
        elif line and not line.startswith('#'):
            data.append(dict(zip(columns, next(csv.reader([line])))))
    return meta, data


def frame_bytes(layout, sequence):
    records = []
    for seq, timestamp in sequence:
        values = list(layout.unpack(bytes(layout.size)))
        values[:2] = seq, timestamp
        records.append(layout.pack(*values))
    fields = [0x544C4F47, 0x60000, 32, 0, layout.size, 100, len(records), len(records)]
    fields[3] = sum(decoder.HEADER_STRUCT.pack(*fields)[16:])
    payload = decoder.HEADER_STRUCT.pack(*fields) + b''.join(records)
    return decoder.FRAME_STRUCT.pack(0x4254464E, 1, 0x60000, 32, layout.size,
                                     len(records), len(records), sum(payload)) + payload


class RetentionDecoderTest(unittest.TestCase):
    def test_turn_target_uses_retained_run_instead_of_latest_test(self):
        meta = {'#fw_metadata_scope': 'dump_time', '#last_test_id': '4'}
        self.assertIsNone(_infer_target(meta))
        self.assertEqual(_infer_target(dict(meta, **{'#op_test_id': str(ord('3'))})), -90.0)
        self.assertEqual(_infer_target({'#last_test_id': '4'}), 90.0)

    def test_legacy_layouts_single_sample_runs_and_tick_wrap(self):
        for version in (3, 4, 5, 6):
            layout = getattr(decoder, f'RECORD_STRUCT_V{version}')
            # First run is a partial fragment; tick wrap stays within it.
            raw = frame_bytes(layout, [(8, 0xFFFFFFFF), (9, 0), (0, 100), (0, 10)])
            frame, header, rows, _ = decoder.extract_frame(raw)
            self.assertEqual(capture._binary_frame_total_len(raw[:-1]), None)
            self.assertEqual(capture._binary_frame_total_len(raw), len(raw))
            with tempfile.TemporaryDirectory() as directory:
                paths = decoder.write_run_csvs(Path(directory) / 'runs.csv', frame, header, rows)
                self.assertEqual(len(paths), 3)
                self.assertEqual([len(read_csv(p)[1]) for p in paths], [2, 1, 1])
                self.assertEqual(read_csv(paths[0])[0]['trace_run_partial_start'], '1')
                self.assertEqual(read_csv(paths[-1])[0]['trace_run_partial_start'], '0')

    def test_single_run_filename_and_checksum_rejection(self):
        raw = frame_bytes(decoder.RECORD_STRUCT_V6, [(0, 10), (1, 11)])
        frame, header, rows, _ = decoder.extract_frame(raw)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'single.csv'
            self.assertEqual(decoder.write_run_csvs(path, frame, header, rows), [path])
        damaged = raw[:-1] + bytes([raw[-1] ^ 1])
        self.assertEqual(capture._binary_frame_total_len(damaged), -1)
        with self.assertRaises(ValueError):
            decoder.extract_frame(damaged)


@unittest.skipUnless(os.environ.get('TRACE_RETENTION_FIXTURES'), 'run test_f413_trace_retention.sh for C fixtures')
class FirmwareDumpTest(unittest.TestCase):
    def fixture(self, name):
        return (Path(os.environ['TRACE_RETENTION_FIXTURES']) / name).read_bytes()

    def test_binary_all_runs_and_oldest_fragment(self):
        for name, sizes, partial in [('full', [4, 1, 1], '0'), ('wrapped', [2, 1, 1, 4], '1')]:
            raw = self.fixture(name + '.raw')
            frame, header, rows, offset = decoder.extract_frame(raw)
            self.assertEqual(frame['record_count'], sum(sizes))
            with tempfile.TemporaryDirectory() as directory:
                capture._save_binary_frame(Path(directory), raw[offset:offset + 64 + len(rows) * 104])
                paths = sorted(Path(directory).glob('*.csv'))
                self.assertEqual([len(read_csv(p)[1]) for p in paths], sizes)
                self.assertEqual(read_csv(paths[0])[0]['trace_run_partial_start'], partial)
                self.assertEqual([read_csv(p)[0]['op_mode'] for p in paths], ['3', '4', '5', '7'][:len(paths)])
                self.assertTrue(all('fw_git_sha' not in read_csv(p)[0] for p in paths))
                self.assertEqual(len(list(Path(directory).glob('*.raw'))), 1)

    def test_full_capacity_frame(self):
        frame, header, rows, _ = decoder.extract_frame(self.fixture('capacity.raw'))
        self.assertEqual(frame['record_count'], 6301)
        self.assertEqual(header['total_records'], 6304)
        self.assertEqual(rows[0][1], '3')
        with tempfile.TemporaryDirectory() as directory:
            paths = decoder.write_run_csvs(Path(directory) / 'capacity.csv', frame, header, rows)
            self.assertEqual(len(paths), 7)
            self.assertEqual(sum(len(read_csv(p)[1]) for p in paths), 6301)

    def test_csv_capture_through_fake_uart(self):
        # Exercise actual stream parsing and file/metadata resets, not a mirror parser.
        master, slave = pty.openpty()
        process = None
        try:
            with tempfile.TemporaryDirectory() as directory:
                with tempfile.TemporaryFile() as errors:
                    process = subprocess.Popen(
                        [sys.executable, str(Path(capture.__file__)), directory,
                         os.ttyname(slave), '115200', '--send', 'V', '--send-interval-ms', '0'],
                        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=errors)
                    ready, _, _ = select.select([master], [], [], 5)
                    self.assertTrue(ready, 'fake UART was not opened')
                    self.assertIn(b'V', os.read(master, 128))
                    os.write(master, self.fixture('wrapped.csv'))
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline:
                        paths = sorted(Path(directory).glob('*.csv'))
                        if len(paths) == 4 and [len(read_csv(p)[1]) for p in paths] == [2, 1, 1, 4]:
                            break
                        time.sleep(0.02)
                    else:
                        errors.seek(0)
                        self.fail(errors.read().decode())
                    for i, path in enumerate(paths):
                        meta, rows = read_csv(path)
                        self.assertEqual(meta['op_mode'], ['3', '4', '5', '7'][i])
                        self.assertEqual(meta['fw_metadata_scope'], 'dump_time')
                        self.assertEqual(meta['fw_git_sha'], 'host-test')
                        self.assertEqual(meta['trace_run_index'], str(i + 1))
                        self.assertEqual(meta['trace_run_partial_start'], '1' if i == 0 else '0')
        finally:
            if process is not None:
                process.terminate()
                process.wait(timeout=5)
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    unittest.main()
