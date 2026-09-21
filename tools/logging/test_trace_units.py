"""Host regression tests for trace display units (no serial/hardware access)."""
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from export_plotjuggler_csv import (
    FLAG_TUNE, _build_output, _build_display_output, _infer_tune_axis,
    _load_nightfall_csv,
)
from trace_bin_dump import RECORD_COLUMNS_V3, RECORD_COLUMNS_V4, RECORD_COLUMNS_V5, RECORD_COLUMNS_V6
from visualizer.web_visualizer import _build_nightfall_plot, _load_nightfall_trace_df


def sample(**updates):
    row = dict(timestamp_ms=10000, flags=0, target_distance_mm=450,
               distance_mm=400, angle_mdeg=-90000, target_angle_mdeg=-85000,
               target_velocity_mm_s=1500, real_velocity_mm_s=1400,
               accel_velocity_mm_s=1450, accel_forward_mm_s2=25000,
               target_omega_mdps=-1500000, real_omega_mdps=-1490000,
               gyro_z_raw_mdps=-1510000, encoder_l=-50, encoder_r=60,
               motor_out_l=500, motor_out_r=-250, adc_vbat=2100,
               reserved_i32_0=-1, reserved_i32_1=0, reserved_u16_0=33792)
    row.update(updates)
    return {k: str(v) for k, v in row.items()}


def convert(row, meta=None, derived=True):
    columns, values = _build_display_output(list(row), [row], meta or {}, derived)
    assert len(columns) == len(values[0]) == len(set(columns))
    return dict(zip(columns, values[0]))


class TraceUnitsTest(unittest.TestCase):
    def test_millidegrees_and_dps_become_one_degree_channel(self):
        out = convert(sample())
        self.assertAlmostEqual(float(out['target_omega_dps']), -1500, places=6)
        self.assertAlmostEqual(float(out['angle_deg']), -90, places=7)
        self.assertAlmostEqual(float(out['omega_error_dps']), -10, places=8)
        self.assertFalse(any(c.endswith(('_mdps', '_mdeg')) for c in out))

    def test_translation_time_motor_and_raw_counters(self):
        row = sample()
        cols, values = _build_display_output(list(row), [row, sample(timestamp_ms=11250)], {})
        out = dict(zip(cols, values[1]))
        expected = {'time': 1.25, 'timestamp_s': 11.25, 'distance_m': .4,
                    'target_distance_m': .45, 'distance_error_m': .05,
                    'target_velocity_m_s': 1.5, 'velocity_error_m_s': .1,
                    'accel_forward_m_s2': 25, 'motor_duty_l': .5,
                    'motor_duty_r': -.25, 'motor_duty_avg': .125,
                    'motor_duty_diff': .75, 'encoder_l': -50, 'adc_vbat': 2100}
        for key, value in expected.items():
            self.assertAlmostEqual(float(out[key]), value, msg=key)
        self.assertEqual(row, sample())  # Conversion must not rewrite source values.

    def test_wall_observation_is_not_a_tune_reference(self):
        # Reproduces the latest mode6 log: wall derivative -1, other derivative 0.
        row = sample(test_id=80)
        for meta in ({}, {'tune_axis': 'velocity'}):
            self.assertIsNone(_infer_tune_axis([row], meta))
            self.assertFalse(any(c.startswith('tune_') for c in convert(row, meta)))

    def test_four_tuning_axes_and_inactive_rows(self):
        for axis, suffix, factor in [(0, 'm_s', .001), (1, 'dps', 1),
                                     (2, 'm', .001), (3, 'deg', 1)]:
            with self.subTest(axis=axis):
                active = sample(flags=FLAG_TUNE, reserved_i32_0=90000, reserved_i32_1=axis)
                cols, values = _build_display_output(list(active), [active, sample(timestamp_ms=10001)], {})
                self.assertAlmostEqual(float(values[0][cols.index('tune_ref_'+suffix)]), 90*factor, places=8)
                self.assertEqual(values[1][cols.index('tune_ref_'+suffix)], '')
                active['reserved_i32_0'] = '0'
                self.assertIn('tune_ref_'+suffix, convert(active))

    def test_no_derived_and_legacy_escape_hatch(self):
        row = sample()
        out = convert(row, derived=False)
        self.assertNotIn('omega_error_dps', out)
        self.assertIn('real_omega_dps', out)
        cols, values = _build_output(list(row), [row], {}, True)
        old = dict(zip(cols, values[0]))
        self.assertEqual(old['target_omega_mdps'], '-1500000')
        self.assertEqual(old['target_omega_dps'], '-1500')

    def test_older_schema_has_no_invented_zero_channels(self):
        out = convert(dict(timestamp_ms='5', flags='0', omega_z_mdps='180000', distance_mm='90'))
        self.assertAlmostEqual(float(out['omega_z_dps']), 180, places=8)
        for absent in ['real_omega_dps', 'target_omega_dps', 'target_distance_m',
                       'gyro_z_raw_dps', 'target_velocity_m_s', 'motor_duty_avg']:
            self.assertNotIn(absent, out)

    def test_supported_decoded_schemas_keep_row_alignment(self):
        for schema in [RECORD_COLUMNS_V3, RECORD_COLUMNS_V4, RECORD_COLUMNS_V5, RECORD_COLUMNS_V6]:
            row = {key: sample().get(key, '0') for key in schema}
            out = convert(row)
            self.assertAlmostEqual(float(out['real_omega_dps']), -1490, places=6)
            self.assertEqual(out['flags'], '0')
            self.assertEqual(out['reserved_u16_0'], '33792')

    def test_event_duration_and_wall_distance_error(self):
        out = convert(sample(event_motion_duration_ms=250,
                             event_front_match_1_position_error_x1000=-1250,
                             event_front_match_1_yaw_error_x1000=500))
        self.assertEqual(float(out['event_motion_duration_s']), .25)
        self.assertEqual(float(out['event_front_match_1_position_error_m']), -.00125)
        self.assertEqual(float(out['event_front_match_1_yaw_error_m']), .0005)

    def test_csv_to_figure_preserves_metadata_and_display_units(self):
        row = sample()
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'capture.csv'
            path.write_text('#fw_git_sha=fixture-sha\n#fw_git_dirty=1\n#mm_columns=' + ','.join(row) + '\n' +
                            ','.join(row.values()) + '\n' +
                            ','.join(sample(timestamp_ms=11250).values()) + '\n')
            original = path.read_bytes()
            df, meta = _load_nightfall_trace_df(path)
            self.assertEqual(meta['fw_git_sha'], 'fixture-sha')
            self.assertEqual(meta['fw_git_dirty'], '1')
            fig = _build_nightfall_plot(df)
            expected_units = {'target_omega_dps': 'Omega [deg/s]', 'angle_deg': 'Angle [deg]',
                              'distance_m': 'Distance [m]', 'real_velocity_m_s': 'Velocity [m/s]',
                              'accel_forward_m_s2': 'IMU Accel [m/s²]', 'motor_duty_l': 'Motor duty [1]'}
            self.assertEqual(len(fig.data), len({t.name for t in fig.data}))
            for trace in fig.data:
                self.assertEqual(list(trace.x), [0, 1.25])
                if trace.name in expected_units:
                    axis = 'yaxis'+trace.yaxis[1:]
                    self.assertEqual(fig.layout[axis].title.text, expected_units[trace.name])
                self.assertFalse(trace.name.endswith(('_mdps', '_mdeg', '_mm', '_mm_s')))
            for axis in fig.select_xaxes():
                self.assertEqual(axis.title.text, 'Time [s]')
            self.assertEqual(path.read_bytes(), original)
            cli = Path(__file__).with_name('export_plotjuggler_csv.py')
            dest = Path(temp) / 'si.csv'
            subprocess.run([sys.executable, str(cli), str(path), '-o', str(dest)], check=True, capture_output=True)
            columns, exported, _ = _load_nightfall_csv(dest)
            self.assertEqual(len(exported), 2)
            self.assertIn('real_omega_dps', columns)
            self.assertEqual(exported[1]['time'], '1.25')

    def test_plotjuggler_template_uses_available_display_channels(self):
        out = convert(sample())
        template = Path(__file__).parent/'plotjuggler/nightfall_f413_tune.xml'
        root = ET.parse(template).getroot()
        for curve in root.findall('.//curve'):
            self.assertIn(curve.attrib['name'], out)
        for plot in root.findall('.//plot'):
            names = [c.attrib['name'] for c in plot.findall('curve')]
            self.assertFalse(any(n.endswith('_deg') for n in names) and any(n.endswith('_dps') for n in names))


if __name__ == '__main__':
    unittest.main()
