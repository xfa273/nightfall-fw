#!/usr/bin/env python3

from __future__ import annotations

import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import sys


VISION_DIR = Path(__file__).resolve().parents[1]
if str(VISION_DIR) not in sys.path:
    sys.path.insert(0, str(VISION_DIR))

import camera_capture_fingerprint as FINGERPRINT  # noqa: E402
from camera_capture_fingerprint import (  # noqa: E402
    BOARD_CALIBRATION_SCHEMA,
    CAPTURE_FINGERPRINT_SCHEMA,
    CAPTURE_SESSION_SCHEMA,
    QUALIFICATION_STATUS_LEGACY_UNVERIFIED,
    QUALIFICATION_STATUS_QUALIFIED,
    CaptureExpectations,
    CaptureFingerprintError,
    CaptureSessionPaths,
    load_capture_session,
    revalidate_capture_fingerprint,
    validate_capture_session,
)


def _rect(left: int, top: int, right: int, bottom: int) -> dict[str, int]:
    return {
        "left": left,
        "top": top,
        "right": right,
        "bottom": bottom,
        "width": right - left,
        "height": bottom - top,
    }


class _CaptureFixture:
    def __init__(self, base: Path) -> None:
        self.base = base
        self.capture_dir = base / "capture"
        self.analysis_dir = base / "analysis"
        self.capture_dir.mkdir(parents=True)
        self.analysis_dir.mkdir()
        self.paths = CaptureSessionPaths(
            hfr_report=self.capture_dir / "hfr_report.json",
            capture_results_jsonl=self.capture_dir / "capture_results.jsonl",
            qa_report=self.analysis_dir / "qa_report.json",
            raw_video=self.capture_dir / "hfr_capture.mp4",
            trajectory_csv=self.analysis_dir / "trajectory.csv",
            source_board_calibration=self.analysis_dir / "calibration.json",
        )
        self.paths.raw_video.write_bytes(b"synthetic-avc-payload\x00\x01")
        raw = self.paths.raw_video.read_bytes()
        raw_sha = hashlib.sha256(raw).hexdigest()
        self.rows = [self._capture_row(100, 1_000_000_000)]
        self.rows.append(self._capture_row(101, 1_004_166_667))
        self.report = {
            "schema": "nightfall_android_hfr_recording_v1",
            "app_version": "0.5.7",
            "status": "complete",
            "error": None,
            "device": {
                "manufacturer": "Google",
                "brand": "google",
                "model": "Pixel 8",
                "device": "shiba",
                "product": "shiba",
                "fingerprint": "google/shiba/test:user/release-keys",
                "android_release": "17",
                "sdk_int": 37,
            },
            "config": {
                "camera_id": "0",
                "width": 1920,
                "height": 1080,
                "fps": 240,
                "video_stabilization_requested": "OFF",
                "optical_stabilization_requested": "OFF",
                "recording_surface_enabled": True,
                "recording_backend": "MediaRecorder",
                "media_recorder_capture_rate_fps": 240,
                "audio_recorded": False,
                "focus_mode": "fixed",
                "requested_focus_distance_diopters": 1.2,
                "focus_settle_minimum_ms": 250,
                "focus_settle_timeout_ms": 2000,
            },
            "camera_static_geometry": {
                "camera_id": "0",
                "physical_camera_ids": ["3"],
                "sensor_orientation_deg": 90,
                "active_array": _rect(0, 0, 4000, 3000),
                "pre_correction_active_array": _rect(0, 0, 4000, 3000),
                "pixel_array_size": {"width": 4080, "height": 3072},
                "sensor_physical_size_mm": {"width": 6.4, "height": 4.8},
                "available_focal_lengths_mm": [6.9],
                "lens_intrinsic_calibration": [3000.0, 3001.0, 2000.0, 1500.0, 0.0],
                "lens_distortion": [0.01, -0.02, 0.0, 0.0, 0.001],
                "minimum_focus_distance_diopters": 10.0,
                "focus_distance_calibration": 2,
                "autofocus_modes": [0, 1, 3, 4],
                "focus_distance_request_supported": True,
                "focus_distance_result_supported": True,
                "lens_state_result_supported": True,
            },
            "orientation_hint_deg": 0,
            "focus_settle": {
                "required": True,
                "settled": True,
                "requested_distance_diopters": 1.2,
                "applied_distance_diopters": 1.25,
                "minimum_stable_ms": 250,
                "timeout_ms": 2000,
                "observation_count": 72,
                "stable_observation_count": 60,
                "control_af_mode": 0,
                "lens_state": 0,
                "started_elapsed_realtime_ns": 1_000_000_000,
                "settled_elapsed_realtime_ns": 1_300_000_000,
                "settle_elapsed_ms": 300.0,
            },
            "capture_results": {
                "callback_count": 2,
                "capture_failure_count": 0,
                "sensor_timestamp_count": 2,
                "unique_sensor_frame_count": 2,
                "duplicate_sensor_timestamp_count": 0,
                "measured_sensor_fps": 240.0,
                "video_stabilization_mode": {
                    "count": 2,
                    "min": 0,
                    "median": 0,
                    "p95": 0,
                    "max": 0,
                },
                "optical_stabilization_mode": {
                    "count": 2,
                    "min": 0,
                    "median": 0,
                    "p95": 0,
                    "max": 0,
                },
                "first_sensor_timestamp_ns": 1_000_000_000,
                "last_sensor_timestamp_ns": 1_004_166_667,
            },
            "encoded_video": {
                "codec": "video/avc",
                "sample_count": 2,
                "measured_encoded_fps": 239.98,
                "file_size_bytes": len(raw),
            },
            "outputs": {
                "video": "hfr_capture.mp4",
                "capture_results_jsonl": "capture_results.jsonl",
            },
            "artifact_integrity": {},
        }
        self.qa = {
            "schema": "nightfall_markerless_trajectory_qa_v3",
            "input": {
                "path": str(self.paths.raw_video),
                "sha256": raw_sha,
                "size_bytes": len(raw),
                "width": 1920,
                "height": 1080,
                "fps_container": 239.9,
                "frames": 2,
            },
            "outputs": {"trajectory_csv": "trajectory.csv"},
            "markerless_pose": {
                "valid_frames": 2,
                "missing_frames": 0,
                "missing_fraction": 0.0,
                "heading_valid_frames": 2,
                "heading_invalid_frames": 0,
                "heading_invalid_fraction": 0.0,
                "heading_gate_applied": True,
                "position_only": False,
                "front_back_ambiguity_resolved": True,
            },
        }
        self.calibration = {
            "schema": BOARD_CALIBRATION_SCHEMA,
            "homography": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        }
        self.write_all()

    @staticmethod
    def _capture_row(frame: int, timestamp: int) -> dict[str, object]:
        return {
            "frame_number": frame,
            "sensor_timestamp_ns": timestamp,
            "video_stabilization_mode": 0,
            "optical_stabilization_mode": 0,
            "focus_distance_diopters": 1.25,
            "control_af_mode": 0,
            "lens_state": 0,
            "lens_focal_length_mm": 6.9,
            "lens_intrinsic_calibration": [1500.0, 1501.0, 960.0, 540.0, 0.0],
            "lens_distortion": [0.01, -0.02, 0.0, 0.0, 0.001],
            "scaler_crop_region": _rect(0, 0, 4000, 3000),
            "control_zoom_ratio": 1.0,
            "active_physical_camera_id": "3",
        }

    def write_all(self) -> None:
        self.write_rows()
        self.write_qa()
        self.paths.source_board_calibration.write_text(
            json.dumps(self.calibration), encoding="utf-8"
        )
        header = (
            "frame,label_x_px,label_y_px,front_label_x_px,front_label_y_px,"
            "pose_valid,heading_valid\n"
        )
        self.paths.trajectory_csv.write_text(
            header + "0,10,20,30,40,1,1\n1,11,21,31,41,1,1\n",
            encoding="utf-8",
        )

    def write_report(self) -> None:
        self.paths.hfr_report.write_text(
            json.dumps(self.report, allow_nan=False), encoding="utf-8"
        )

    def write_rows(self) -> None:
        self.paths.capture_results_jsonl.write_text(
            "".join(json.dumps(row, allow_nan=False) + "\n" for row in self.rows),
            encoding="utf-8",
        )
        self._sync_artifact_integrity()
        self.write_report()

    def _sync_artifact_integrity(self) -> None:
        def item(path: Path) -> dict[str, object]:
            payload = path.read_bytes()
            return {
                "filename": path.name,
                "size_bytes": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
            }

        self.report["artifact_integrity"] = {
            "video": item(self.paths.raw_video),
            "capture_results_jsonl": item(self.paths.capture_results_jsonl),
        }

    def write_qa(self) -> None:
        self.paths.qa_report.write_text(
            json.dumps(self.qa, allow_nan=False), encoding="utf-8"
        )

    def manifest(self) -> dict[str, str]:
        result = {"schema": CAPTURE_SESSION_SCHEMA}
        for name, path in self.paths.items():
            result[name] = str(path.relative_to(self.base))
        return result

    def load(self, **kwargs: object):
        return load_capture_session(self.base, self.manifest(), **kwargs)


class CameraCaptureFingerprintTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.fixture = _CaptureFixture(Path(self.temporary.name))

    def test_current_capture_is_qualified_and_hashes_are_stable(self) -> None:
        result = self.fixture.load()
        self.assertTrue(result.safety_qualified)
        self.assertEqual(result.qualification_status, QUALIFICATION_STATUS_QUALIFIED)
        self.assertEqual(result.capture_result_count, 2)
        self.assertEqual(
            result.configuration.requested_focus_distance_diopters, 1.2
        )
        self.assertEqual(result.frame_metadata.focus_distance_diopters, 1.25)
        self.assertEqual(result.frame_metadata.control_af_mode, 0)
        self.assertEqual(result.frame_metadata.lens_state, 0)
        self.assertEqual(result.configuration.focus_settle_minimum_ms, 250)
        self.assertEqual(result.configuration.focus_settle_timeout_ms, 2000)
        self.assertEqual(
            result.static_geometry.minimum_focus_distance_diopters, 10.0
        )
        self.assertEqual(len(result.fingerprint_sha256), 64)
        self.assertEqual(len(result.camera_setup_sha256 or ""), 64)
        self.assertEqual(result.fingerprint_sha256, self.fixture.load().fingerprint_sha256)
        serialized = result.to_json()
        self.assertEqual(serialized["schema"], CAPTURE_FINGERPRINT_SCHEMA)
        self.assertEqual(
            set(serialized["artifacts"]),
            set(name for name, _ in self.fixture.paths.items()),
        )
        self.assertEqual(
            result.fingerprint_sha256,
            validate_capture_session(self.fixture.paths).fingerprint_sha256,
        )
        expected = CaptureExpectations(
            manufacturer="Google", model="Pixel 8", device="shiba"
        )
        self.assertTrue(self.fixture.load(expectations=expected).safety_qualified)
        self.assertEqual(
            revalidate_capture_fingerprint(result.to_json()).fingerprint_sha256,
            result.fingerprint_sha256,
        )

    def test_artifact_content_changes_fingerprint_not_camera_setup(self) -> None:
        before = self.fixture.load()
        self.fixture.calibration["diagnostic_note"] = "different bound artifact"
        self.fixture.paths.source_board_calibration.write_text(
            json.dumps(self.fixture.calibration), encoding="utf-8"
        )
        after = self.fixture.load()
        self.assertNotEqual(before.fingerprint_sha256, after.fingerprint_sha256)
        self.assertEqual(before.camera_setup_sha256, after.camera_setup_sha256)

    def test_legacy_capture_and_missing_block_are_never_qualified(self) -> None:
        self.fixture.report.pop("camera_static_geometry")
        for row in self.fixture.rows:
            for field in (
                "lens_focal_length_mm",
                "lens_intrinsic_calibration",
                "lens_distortion",
                "scaler_crop_region",
                "control_zoom_ratio",
                "active_physical_camera_id",
            ):
                row.pop(field)
        self.fixture.write_report()
        self.fixture.write_rows()
        legacy = self.fixture.load()
        self.assertFalse(legacy.safety_qualified)
        self.assertEqual(
            legacy.qualification_status, QUALIFICATION_STATUS_LEGACY_UNVERIFIED
        )
        self.assertIsNone(legacy.camera_setup_sha256)
        absent = load_capture_session(self.fixture.base, None)
        self.assertFalse(absent.safety_qualified)
        self.assertEqual(absent.artifacts, ())

    def test_partial_null_and_varying_frame_metadata_fail_closed(self) -> None:
        mutations = {
            "partial row": lambda rows: rows[1].pop("lens_distortion"),
            "null focal": lambda rows: rows[0].update(lens_focal_length_mm=None),
            "null focus": lambda rows: rows[0].update(
                focus_distance_diopters=None
            ),
            "negative focus": lambda rows: rows[0].update(
                focus_distance_diopters=-0.1
            ),
            "varying focus": lambda rows: rows[1].update(
                focus_distance_diopters=1.5
            ),
            "focus exceeds lens limit": lambda rows: rows[0].update(
                focus_distance_diopters=10.1
            ),
            "missing AF mode": lambda rows: rows[0].pop("control_af_mode"),
            "AF mode changed": lambda rows: rows[1].update(control_af_mode=1),
            "missing lens state": lambda rows: rows[0].pop("lens_state"),
            "lens moved": lambda rows: rows[1].update(lens_state=1),
            "varying intrinsics": lambda rows: rows[1].update(
                lens_intrinsic_calibration=[1502.0, 1501.0, 960.0, 540.0, 0.0]
            ),
            "varying crop": lambda rows: rows[1].update(
                scaler_crop_region=_rect(1, 0, 4000, 3000)
            ),
            "unknown physical camera": lambda rows: rows[0].update(
                active_physical_camera_id="4"
            ),
        }
        original = copy.deepcopy(self.fixture.rows)
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                self.fixture.rows = copy.deepcopy(original)
                mutate(self.fixture.rows)
                self.fixture.write_rows()
                with self.assertRaises(CaptureFingerprintError):
                    self.fixture.load()

    def test_static_geometry_is_required_complete_and_consistent(self) -> None:
        originals = copy.deepcopy(self.fixture.report)
        mutations = {
            "missing": lambda report: report.pop("camera_static_geometry"),
            "null active array": lambda report: report["camera_static_geometry"].update(
                active_array=None
            ),
            "no physical IDs": lambda report: report["camera_static_geometry"].update(
                physical_camera_ids=[]
            ),
            "wrong camera": lambda report: report["camera_static_geometry"].update(
                camera_id="1"
            ),
            "bad rect copy": lambda report: report["camera_static_geometry"][
                "active_array"
            ].update(width=3999),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                self.fixture.report = copy.deepcopy(originals)
                mutate(self.fixture.report)
                self.fixture.write_report()
                with self.assertRaises(CaptureFingerprintError):
                    self.fixture.load()

    def test_current_metadata_requires_recorder_0_5_7_or_newer(self) -> None:
        for version in (None, "0.5.5", "0.5.6", "unknown"):
            with self.subTest(version=version):
                if version is None:
                    self.fixture.report.pop("app_version", None)
                else:
                    self.fixture.report["app_version"] = version
                self.fixture.write_report()
                with self.assertRaises(CaptureFingerprintError):
                    self.fixture.load()

    def test_fixed_focus_config_and_static_contract_fail_closed(self) -> None:
        original = copy.deepcopy(self.fixture.report)

        def remove_config(field: str):
            return lambda report: report["config"].pop(field)

        def config_value(field: str, value: object):
            return lambda report: report["config"].update({field: value})

        def remove_static(field: str):
            return lambda report: report["camera_static_geometry"].pop(field)

        def static_value(field: str, value: object):
            return lambda report: report["camera_static_geometry"].update(
                {field: value}
            )

        mutations = {
            "focus mode missing": remove_config("focus_mode"),
            "focus mode not fixed": config_value("focus_mode", "continuous"),
            "requested focus missing": remove_config(
                "requested_focus_distance_diopters"
            ),
            "requested focus negative": config_value(
                "requested_focus_distance_diopters", -0.1
            ),
            "requested focus above limit": config_value(
                "requested_focus_distance_diopters", 10.1
            ),
            "settle minimum missing": remove_config(
                "focus_settle_minimum_ms"
            ),
            "settle minimum too short": config_value(
                "focus_settle_minimum_ms", 249
            ),
            "settle timeout missing": remove_config("focus_settle_timeout_ms"),
            "settle timeout before minimum": config_value(
                "focus_settle_timeout_ms", 200
            ),
            "minimum focus missing": remove_static(
                "minimum_focus_distance_diopters"
            ),
            "minimum focus negative": static_value(
                "minimum_focus_distance_diopters", -1.0
            ),
            "focus calibration uncalibrated": static_value(
                "focus_distance_calibration", 0
            ),
            "AF OFF unavailable": static_value("autofocus_modes", [1, 3, 4]),
            "focus request unsupported": static_value(
                "focus_distance_request_supported", False
            ),
            "focus result unsupported": static_value(
                "focus_distance_result_supported", False
            ),
            "lens state unsupported": static_value(
                "lens_state_result_supported", False
            ),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                self.fixture.report = copy.deepcopy(original)
                mutate(self.fixture.report)
                self.fixture.write_report()
                with self.assertRaises(CaptureFingerprintError):
                    self.fixture.load()

    def test_approximate_focus_calibration_and_quantized_result_are_allowed(self) -> None:
        self.fixture.report["camera_static_geometry"][
            "focus_distance_calibration"
        ] = 1
        self.fixture.report["config"][
            "requested_focus_distance_diopters"
        ] = 1.2
        for row in self.fixture.rows:
            row["focus_distance_diopters"] = 1.25
        self.fixture.write_rows()
        result = self.fixture.load()
        self.assertTrue(result.safety_qualified)
        self.assertEqual(result.static_geometry.focus_distance_calibration, 1)
        self.assertNotEqual(
            result.configuration.requested_focus_distance_diopters,
            result.frame_metadata.focus_distance_diopters,
        )

    def test_focus_settle_evidence_fails_closed(self) -> None:
        original = copy.deepcopy(self.fixture.report)

        def remove(field: str):
            return lambda report: report["focus_settle"].pop(field)

        def value(field: str, item: object):
            return lambda report: report["focus_settle"].update({field: item})

        mutations = {
            "settle report missing": lambda report: report.pop("focus_settle"),
            "required false": value("required", False),
            "settled false": value("settled", False),
            "requested mismatch": value("requested_distance_diopters", 1.3),
            "applied mismatch": value("applied_distance_diopters", 1.3),
            "minimum mismatch": value("minimum_stable_ms", 251),
            "timeout mismatch": value("timeout_ms", 1999),
            "observation count missing": remove("observation_count"),
            "observation count zero": value("observation_count", 0),
            "stable count zero": value("stable_observation_count", 0),
            "stable exceeds observations": value(
                "stable_observation_count", 73
            ),
            "settle AF not off": value("control_af_mode", 1),
            "settle lens moving": value("lens_state", 1),
            "started timestamp missing": remove("started_elapsed_realtime_ns"),
            "timestamps reversed": value(
                "settled_elapsed_realtime_ns", 999_999_999
            ),
            "elapsed mismatch": value("settle_elapsed_ms", 301.0),
            "settled before minimum": value(
                "settled_elapsed_realtime_ns", 1_200_000_000
            ),
            "settled after timeout": value(
                "settled_elapsed_realtime_ns", 3_100_000_000
            ),
        }
        for label, mutate in mutations.items():
            with self.subTest(label=label):
                self.fixture.report = copy.deepcopy(original)
                mutate(self.fixture.report)
                if label == "settled before minimum":
                    self.fixture.report["focus_settle"]["settle_elapsed_ms"] = (
                        200.0
                    )
                elif label == "settled after timeout":
                    self.fixture.report["focus_settle"]["settle_elapsed_ms"] = (
                        2100.0
                    )
                self.fixture.write_report()
                with self.assertRaises(CaptureFingerprintError):
                    self.fixture.load()

    def test_focus_settle_config_participates_in_camera_setup_fingerprint(self) -> None:
        first = self.fixture.load()
        second = _CaptureFixture(self.fixture.base / "settle-config-run")
        second.report["config"]["focus_settle_minimum_ms"] = 300
        second.report["focus_settle"]["minimum_stable_ms"] = 300
        second.report["focus_settle"][
            "settled_elapsed_realtime_ns"
        ] = 1_350_000_000
        second.report["focus_settle"]["settle_elapsed_ms"] = 350.0
        second.write_report()
        qualified = second.load()
        self.assertTrue(qualified.safety_qualified)
        self.assertNotEqual(
            first.camera_setup_sha256, qualified.camera_setup_sha256
        )

    def test_output_mode_device_and_stabilization_fail_closed(self) -> None:
        cases = (
            ("camera", "config", "camera_id", "1"),
            ("width", "config", "width", 1280),
            ("fps", "config", "fps", 120),
            ("requested EIS", "config", "video_stabilization_requested", "ON"),
            ("actual OIS", "row", "optical_stabilization_mode", 1),
        )
        original_report = copy.deepcopy(self.fixture.report)
        original_rows = copy.deepcopy(self.fixture.rows)
        for label, target, field, value in cases:
            with self.subTest(label=label):
                self.fixture.report = copy.deepcopy(original_report)
                self.fixture.rows = copy.deepcopy(original_rows)
                if target == "config":
                    self.fixture.report["config"][field] = value
                else:
                    self.fixture.rows[0][field] = value
                self.fixture.write_report()
                self.fixture.write_rows()
                with self.assertRaises(CaptureFingerprintError):
                    self.fixture.load()
        with self.assertRaises(CaptureFingerprintError):
            self.fixture.load(expectations=CaptureExpectations(model="Pixel 7"))

    def test_sha_paths_calibration_and_trajectory_are_bound(self) -> None:
        self.fixture.qa["input"]["sha256"] = "0" * 64
        self.fixture.write_qa()
        with self.assertRaises(CaptureFingerprintError):
            self.fixture.load()

        self.fixture = _CaptureFixture(Path(self.temporary.name) / "second")
        self.fixture.qa["outputs"]["trajectory_csv"] = "other.csv"
        self.fixture.write_qa()
        with self.assertRaises(CaptureFingerprintError):
            self.fixture.load()

        self.fixture = _CaptureFixture(Path(self.temporary.name) / "third")
        self.fixture.calibration["schema"] = "old"
        self.fixture.paths.source_board_calibration.write_text(
            json.dumps(self.fixture.calibration), encoding="utf-8"
        )
        with self.assertRaises(CaptureFingerprintError):
            self.fixture.load()

        self.fixture = _CaptureFixture(Path(self.temporary.name) / "fourth")
        self.fixture.paths.trajectory_csv.write_text(
            "frame,label_x_px,label_y_px,front_label_x_px,front_label_y_px,"
            "pose_valid,heading_valid\n0,1,2,3,4,1,1\n",
            encoding="utf-8",
        )
        with self.assertRaises(CaptureFingerprintError):
            self.fixture.load()

    def test_report_artifact_integrity_hashes_are_required(self) -> None:
        original = copy.deepcopy(self.fixture.report["artifact_integrity"])
        for name in ("video", "capture_results_jsonl"):
            with self.subTest(name=name):
                self.fixture.report["artifact_integrity"] = copy.deepcopy(
                    original
                )
                self.fixture.report["artifact_integrity"][name]["sha256"] = (
                    "0" * 64
                )
                self.fixture.write_report()
                with self.assertRaisesRegex(
                    CaptureFingerprintError, "SHA-256 does not match"
                ):
                    self.fixture.load()

        self.fixture.report["artifact_integrity"] = copy.deepcopy(original)
        self.fixture.report.pop("artifact_integrity")
        self.fixture.write_report()
        with self.assertRaisesRegex(CaptureFingerprintError, "artifact_integrity"):
            self.fixture.load()

    def test_capture_artifacts_from_different_runs_are_rejected(self) -> None:
        second = _CaptureFixture(self.fixture.base / "second-run")
        mixed = CaptureSessionPaths(
            hfr_report=self.fixture.paths.hfr_report,
            capture_results_jsonl=second.paths.capture_results_jsonl,
            qa_report=second.paths.qa_report,
            raw_video=second.paths.raw_video,
            trajectory_csv=second.paths.trajectory_csv,
            source_board_calibration=second.paths.source_board_calibration,
        )
        with self.assertRaisesRegex(
            CaptureFingerprintError, "one capture directory"
        ):
            validate_capture_session(mixed)

    def test_focus_distance_participates_in_camera_setup_fingerprint(self) -> None:
        first = self.fixture.load()
        second = _CaptureFixture(self.fixture.base / "focus-run")
        for row in second.rows:
            row["focus_distance_diopters"] = 1.5
        second.report["focus_settle"]["applied_distance_diopters"] = 1.5
        second.write_rows()
        qualified = second.load()
        self.assertTrue(qualified.safety_qualified)
        self.assertNotEqual(
            first.camera_setup_sha256, qualified.camera_setup_sha256
        )

    def test_capture_summary_duplicates_and_mixed_generations_fail_closed(self) -> None:
        original_report = copy.deepcopy(self.fixture.report)
        original_rows = copy.deepcopy(self.fixture.rows)
        # Camera2 HAL frame numbers need not be unique.  Sensor timestamps are
        # the capture identity used here and must remain unique.
        self.fixture.rows[1]["frame_number"] = self.fixture.rows[0]["frame_number"]
        self.fixture.write_rows()
        self.assertTrue(self.fixture.load().safety_qualified)

        self.fixture.rows = copy.deepcopy(original_rows)
        self.fixture.report["capture_results"]["capture_failure_count"] = 1
        self.fixture.write_report()
        with self.assertRaises(CaptureFingerprintError):
            self.fixture.load()

        self.fixture.report = original_report
        self.fixture.rows = copy.deepcopy(original_rows)
        self.fixture.rows[1]["sensor_timestamp_ns"] = self.fixture.rows[0][
            "sensor_timestamp_ns"
        ]
        self.fixture.write_report()
        self.fixture.write_rows()
        with self.assertRaises(CaptureFingerprintError):
            self.fixture.load()

        self.fixture.rows = copy.deepcopy(original_rows)
        self.fixture.rows[1].pop("lens_focal_length_mm")
        self.fixture.rows[1].pop("lens_intrinsic_calibration")
        self.fixture.rows[1].pop("lens_distortion")
        self.fixture.rows[1].pop("scaler_crop_region")
        self.fixture.rows[1].pop("control_zoom_ratio")
        self.fixture.rows[1].pop("active_physical_camera_id")
        self.fixture.write_rows()
        with self.assertRaises(CaptureFingerprintError):
            self.fixture.load()

    def test_current_capture_failures_are_bounded_to_one_percent(self) -> None:
        self.fixture.rows = [
            self.fixture._capture_row(index, 1_000_000_000 + index * 4_166_667)
            for index in range(100)
        ]
        summary = self.fixture.report["capture_results"]
        summary.update(
            callback_count=100,
            capture_failure_count=1,
            sensor_timestamp_count=100,
            unique_sensor_frame_count=100,
            duplicate_sensor_timestamp_count=0,
            first_sensor_timestamp_ns=1_000_000_000,
            last_sensor_timestamp_ns=1_000_000_000 + 99 * 4_166_667,
        )
        summary["video_stabilization_mode"]["count"] = 100
        summary["optical_stabilization_mode"]["count"] = 100
        self.fixture.report["encoded_video"]["sample_count"] = 100
        self.fixture.qa["input"]["frames"] = 100
        self.fixture.qa["markerless_pose"].update(
            valid_frames=100,
            missing_frames=0,
            missing_fraction=0.0,
            heading_valid_frames=100,
            heading_invalid_frames=0,
            heading_invalid_fraction=0.0,
        )
        self.fixture.paths.trajectory_csv.write_text(
            "frame,label_x_px,label_y_px,front_label_x_px,front_label_y_px,"
            "pose_valid,heading_valid\n"
            + "".join(f"{index},1,2,3,4,1,1\n" for index in range(100)),
            encoding="utf-8",
        )
        self.fixture.write_rows()
        self.fixture.write_report()
        self.fixture.write_qa()
        self.assertTrue(self.fixture.load().safety_qualified)
        summary["capture_failure_count"] = 2
        self.fixture.write_report()
        with self.assertRaises(CaptureFingerprintError):
            self.fixture.load()

    def test_manifest_and_json_are_strict(self) -> None:
        with self.assertRaises(CaptureFingerprintError):
            load_capture_session(self.fixture.base, {})
        malformed = self.fixture.manifest()
        malformed["raw_video"] = malformed["qa_report"]
        with self.assertRaises(CaptureFingerprintError):
            load_capture_session(self.fixture.base, malformed)

        self.fixture.paths.hfr_report.write_text(
            '{"schema":"a","schema":"b"}', encoding="utf-8"
        )
        with self.assertRaises(CaptureFingerprintError):
            self.fixture.load()

    def test_known_pose_movement_interval_is_allowed_by_fingerprint(self) -> None:
        path = self.fixture.paths.trajectory_csv
        invalid = set(range(400, 406))
        path.write_text(
            "frame,label_x_px,label_y_px,front_label_x_px,front_label_y_px,"
            "pose_valid,heading_valid\n"
            + "".join(
                f"{index},1,2,3,4,{0 if index in invalid else 1},1\n"
                for index in range(1000)
            ),
            encoding="utf-8",
        )
        qa = {
            "input": {"frames": 1000},
            "markerless_pose": {
                "valid_frames": 994,
                "missing_frames": 6,
                "missing_fraction": 0.006,
                "heading_valid_frames": 1000,
                "heading_invalid_frames": 0,
                "heading_invalid_fraction": 0.0,
                "heading_gate_applied": True,
                "position_only": False,
                "front_back_ambiguity_resolved": True,
            },
        }
        FINGERPRINT._validate_trajectory(path, qa)


if __name__ == "__main__":
    unittest.main()
