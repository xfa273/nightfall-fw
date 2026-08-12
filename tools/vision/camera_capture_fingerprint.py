#!/usr/bin/env python3
"""Fail-closed camera/session fingerprinting for vision calibration inputs.

The qualified path intentionally accepts only the self-integrity-bound metadata
emitted by HFR Recorder 0.5.7 or newer.  Older reports remain loadable where
their legacy shape permits diagnostics, but can never become safety-qualified
through this module.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence


CAPTURE_SESSION_SCHEMA = "nightfall_camera_capture_session_v1"
CAPTURE_FINGERPRINT_SCHEMA = "nightfall_camera_capture_fingerprint_v1"
HFR_REPORT_SCHEMA = "nightfall_android_hfr_recording_v1"
TRAJECTORY_QA_SCHEMA = "nightfall_markerless_trajectory_qa_v3"
BOARD_CALIBRATION_SCHEMA = "nightfall_markerless_board_calibration_v3"

QUALIFICATION_STATUS_QUALIFIED = "qualified"
QUALIFICATION_STATUS_LEGACY_UNVERIFIED = "legacy_unverified"

_PATH_FIELDS = (
    "hfr_report",
    "capture_results_jsonl",
    "qa_report",
    "raw_video",
    "trajectory_csv",
    "source_board_calibration",
)
_CURRENT_CAPTURE_FIELDS = (
    "lens_focal_length_mm",
    "lens_intrinsic_calibration",
    "lens_distortion",
    "scaler_crop_region",
    "control_zoom_ratio",
    "active_physical_camera_id",
)
_QUALIFIED_CAPTURE_FIELDS = (
    *_CURRENT_CAPTURE_FIELDS,
    "focus_distance_diopters",
    "control_af_mode",
    "lens_state",
)
_DEVICE_STRING_FIELDS = (
    "manufacturer",
    "brand",
    "model",
    "device",
    "product",
    "fingerprint",
    "android_release",
)
_TRAJECTORY_REQUIRED_COLUMNS = frozenset(
    {
        "label_x_px",
        "label_y_px",
        "front_label_x_px",
        "front_label_y_px",
        "pose_valid",
        "heading_valid",
    }
)


class CaptureFingerprintError(ValueError):
    """Raised when a declared capture session cannot be trusted."""


@dataclass(frozen=True)
class CaptureExpectations:
    """Expected acquisition mode for one calibration capture session."""

    camera_id: str = "0"
    width: int = 1920
    height: int = 1080
    fps: int = 240
    video_stabilization_mode: int = 0
    optical_stabilization_mode: int = 0
    fps_tolerance: float = 1.0
    manufacturer: str | None = None
    model: str | None = None
    device: str | None = None


@dataclass(frozen=True)
class CaptureSessionPaths:
    hfr_report: Path
    capture_results_jsonl: Path
    qa_report: Path
    raw_video: Path
    trajectory_csv: Path
    source_board_calibration: Path

    def items(self) -> tuple[tuple[str, Path], ...]:
        return tuple((field, getattr(self, field)) for field in _PATH_FIELDS)


@dataclass(frozen=True)
class ArtifactBinding:
    name: str
    path: Path
    sha256: str
    size_bytes: int

    def to_json(self) -> dict[str, Any]:
        return {
            "path": str(self.path),
            "sha256": self.sha256,
            "size_bytes": self.size_bytes,
        }


@dataclass(frozen=True)
class DeviceIdentity:
    manufacturer: str
    brand: str
    model: str
    device: str
    product: str
    fingerprint: str
    android_release: str
    sdk_int: int

    def to_json(self) -> dict[str, Any]:
        return {
            "manufacturer": self.manufacturer,
            "brand": self.brand,
            "model": self.model,
            "device": self.device,
            "product": self.product,
            "fingerprint": self.fingerprint,
            "android_release": self.android_release,
            "sdk_int": self.sdk_int,
        }


@dataclass(frozen=True)
class CaptureConfiguration:
    camera_id: str
    width: int
    height: int
    fps: int
    orientation_hint_deg: int
    video_stabilization_requested: str
    optical_stabilization_requested: str
    video_stabilization_mode: int
    optical_stabilization_mode: int
    focus_mode: str | None
    requested_focus_distance_diopters: float | None
    focus_settle_minimum_ms: int | None
    focus_settle_timeout_ms: int | None

    def to_json(self) -> dict[str, Any]:
        return {
            "camera_id": self.camera_id,
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "orientation_hint_deg": self.orientation_hint_deg,
            "video_stabilization_requested": (
                self.video_stabilization_requested
            ),
            "optical_stabilization_requested": (
                self.optical_stabilization_requested
            ),
            "video_stabilization_mode": self.video_stabilization_mode,
            "optical_stabilization_mode": self.optical_stabilization_mode,
            "focus_mode": self.focus_mode,
            "requested_focus_distance_diopters": (
                self.requested_focus_distance_diopters
            ),
            "focus_settle_minimum_ms": self.focus_settle_minimum_ms,
            "focus_settle_timeout_ms": self.focus_settle_timeout_ms,
        }


@dataclass(frozen=True)
class RectFingerprint:
    left: int
    top: int
    right: int
    bottom: int

    @property
    def width(self) -> int:
        return self.right - self.left

    @property
    def height(self) -> int:
        return self.bottom - self.top

    def to_json(self) -> dict[str, int]:
        return {
            "left": self.left,
            "top": self.top,
            "right": self.right,
            "bottom": self.bottom,
            "width": self.width,
            "height": self.height,
        }


@dataclass(frozen=True)
class SizeFingerprint:
    width: int
    height: int

    def to_json(self) -> dict[str, int]:
        return {"width": self.width, "height": self.height}


@dataclass(frozen=True)
class SizeFFingerprint:
    width: float
    height: float

    def to_json(self) -> dict[str, float]:
        return {"width": self.width, "height": self.height}


@dataclass(frozen=True)
class StaticCameraGeometry:
    camera_id: str
    physical_camera_ids: tuple[str, ...]
    sensor_orientation_deg: int
    active_array: RectFingerprint
    pre_correction_active_array: RectFingerprint
    pixel_array_size: SizeFingerprint
    sensor_physical_size_mm: SizeFFingerprint
    available_focal_lengths_mm: tuple[float, ...]
    lens_intrinsic_calibration: tuple[float, ...]
    lens_distortion: tuple[float, ...]
    minimum_focus_distance_diopters: float
    focus_distance_calibration: int
    autofocus_modes: tuple[int, ...]
    focus_distance_request_supported: bool
    focus_distance_result_supported: bool
    lens_state_result_supported: bool

    def to_json(self) -> dict[str, Any]:
        return {
            "camera_id": self.camera_id,
            "physical_camera_ids": list(self.physical_camera_ids),
            "sensor_orientation_deg": self.sensor_orientation_deg,
            "active_array": self.active_array.to_json(),
            "pre_correction_active_array": (
                self.pre_correction_active_array.to_json()
            ),
            "pixel_array_size": self.pixel_array_size.to_json(),
            "sensor_physical_size_mm": self.sensor_physical_size_mm.to_json(),
            "available_focal_lengths_mm": list(
                self.available_focal_lengths_mm
            ),
            "lens_intrinsic_calibration": list(
                self.lens_intrinsic_calibration
            ),
            "lens_distortion": list(self.lens_distortion),
            "minimum_focus_distance_diopters": (
                self.minimum_focus_distance_diopters
            ),
            "focus_distance_calibration": self.focus_distance_calibration,
            "autofocus_modes": list(self.autofocus_modes),
            "focus_distance_request_supported": (
                self.focus_distance_request_supported
            ),
            "focus_distance_result_supported": (
                self.focus_distance_result_supported
            ),
            "lens_state_result_supported": self.lens_state_result_supported,
        }


@dataclass(frozen=True)
class FrameCameraMetadata:
    active_physical_camera_id: str
    focus_distance_diopters: float
    lens_focal_length_mm: float
    lens_intrinsic_calibration: tuple[float, ...]
    lens_distortion: tuple[float, ...]
    scaler_crop_region: RectFingerprint
    control_zoom_ratio: float
    control_af_mode: int
    lens_state: int

    def to_json(self) -> dict[str, Any]:
        return {
            "active_physical_camera_id": self.active_physical_camera_id,
            "focus_distance_diopters": self.focus_distance_diopters,
            "lens_focal_length_mm": self.lens_focal_length_mm,
            "lens_intrinsic_calibration": list(
                self.lens_intrinsic_calibration
            ),
            "lens_distortion": list(self.lens_distortion),
            "scaler_crop_region": self.scaler_crop_region.to_json(),
            "control_zoom_ratio": self.control_zoom_ratio,
            "control_af_mode": self.control_af_mode,
            "lens_state": self.lens_state,
        }


@dataclass(frozen=True)
class CameraCaptureFingerprint:
    qualification_status: str
    safety_qualified: bool
    reasons: tuple[str, ...]
    metadata_generation: str
    artifacts: tuple[ArtifactBinding, ...]
    device: DeviceIdentity | None
    configuration: CaptureConfiguration | None
    static_geometry: StaticCameraGeometry | None
    frame_metadata: FrameCameraMetadata | None
    capture_result_count: int

    def artifact(self, name: str) -> ArtifactBinding:
        for item in self.artifacts:
            if item.name == name:
                return item
        raise CaptureFingerprintError(f"capture fingerprint lacks artifact: {name}")

    def _camera_setup_payload(self) -> dict[str, Any] | None:
        if not self.safety_qualified:
            return None
        if (
            self.device is None
            or self.configuration is None
            or self.static_geometry is None
            or self.frame_metadata is None
        ):
            raise CaptureFingerprintError(
                "qualified fingerprint is missing camera setup metadata"
            )
        return {
            "device": self.device.to_json(),
            "configuration": self.configuration.to_json(),
            "static_geometry": self.static_geometry.to_json(),
            "frame_metadata": self.frame_metadata.to_json(),
        }

    @property
    def camera_setup_sha256(self) -> str | None:
        payload = self._camera_setup_payload()
        return None if payload is None else _canonical_sha256(payload)

    @property
    def fingerprint_sha256(self) -> str:
        payload = {
            "schema": CAPTURE_FINGERPRINT_SCHEMA,
            "qualification_status": self.qualification_status,
            "safety_qualified": self.safety_qualified,
            "reasons": list(self.reasons),
            "metadata_generation": self.metadata_generation,
            "artifacts": {
                item.name: {
                    "sha256": item.sha256,
                    "size_bytes": item.size_bytes,
                }
                for item in sorted(self.artifacts, key=lambda item: item.name)
            },
            "camera_setup": self._camera_setup_payload(),
            "capture_result_count": self.capture_result_count,
        }
        return _canonical_sha256(payload)

    def to_json(self) -> dict[str, Any]:
        return {
            "schema": CAPTURE_FINGERPRINT_SCHEMA,
            "qualification_status": self.qualification_status,
            "safety_qualified": self.safety_qualified,
            "reasons": list(self.reasons),
            "metadata_generation": self.metadata_generation,
            "fingerprint_sha256": self.fingerprint_sha256,
            "camera_setup_sha256": self.camera_setup_sha256,
            "artifacts": {
                item.name: item.to_json()
                for item in sorted(self.artifacts, key=lambda item: item.name)
            },
            "device": None if self.device is None else self.device.to_json(),
            "configuration": (
                None
                if self.configuration is None
                else self.configuration.to_json()
            ),
            "static_geometry": (
                None
                if self.static_geometry is None
                else self.static_geometry.to_json()
            ),
            "frame_metadata": (
                None
                if self.frame_metadata is None
                else self.frame_metadata.to_json()
            ),
            "capture_result_count": self.capture_result_count,
        }


def load_capture_session(
    base: Path,
    manifest_capture_session: Mapping[str, Any] | None,
    *,
    expectations: CaptureExpectations = CaptureExpectations(),
) -> CameraCaptureFingerprint:
    """Load a manifest capture block and return its verified fingerprint.

    A missing block represents a legacy manifest and is deliberately returned
    as unverified.  A present but malformed block is an error rather than a
    legacy fallback, preventing misspelled artifact fields from bypassing the
    safety checks.
    """

    if manifest_capture_session is None:
        return _legacy_without_artifacts(
            "capture_session is absent (legacy manifest)"
        )
    if not isinstance(manifest_capture_session, Mapping):
        raise CaptureFingerprintError("capture_session must be an object")
    if manifest_capture_session.get("schema") != CAPTURE_SESSION_SCHEMA:
        raise CaptureFingerprintError(
            "capture_session.schema must be " + CAPTURE_SESSION_SCHEMA
        )
    root = Path(base).expanduser().resolve()
    resolved: dict[str, Path] = {}
    for field in _PATH_FIELDS:
        raw = manifest_capture_session.get(field)
        if not isinstance(raw, str) or not raw.strip():
            raise CaptureFingerprintError(
                f"capture_session.{field} must be a non-empty path"
            )
        path = Path(raw).expanduser()
        resolved[field] = (path if path.is_absolute() else root / path).resolve()
    if len(set(resolved.values())) != len(resolved):
        raise CaptureFingerprintError(
            "capture_session artifact paths must identify distinct files"
        )
    return validate_capture_session(
        CaptureSessionPaths(**resolved), expectations=expectations
    )


def validate_capture_session(
    paths: CaptureSessionPaths,
    *,
    expectations: CaptureExpectations = CaptureExpectations(),
) -> CameraCaptureFingerprint:
    """Validate explicitly supplied capture paths and build a fingerprint."""

    _validate_expectations(expectations)
    capture_parent = paths.hfr_report.expanduser().resolve().parent
    for name, path in (
        ("capture_results_jsonl", paths.capture_results_jsonl),
        ("raw_video", paths.raw_video),
    ):
        if path.expanduser().resolve().parent != capture_parent:
            raise CaptureFingerprintError(
                "hfr_report, capture_results_jsonl, and raw_video must share "
                f"one capture directory ({name} differs)"
            )
    artifacts = tuple(_bind_artifact(name, path) for name, path in paths.items())
    artifact_by_name = {item.name: item for item in artifacts}

    report = _load_json(paths.hfr_report, "hfr_report")
    qa = _load_json(paths.qa_report, "qa_report")
    calibration = _load_json(
        paths.source_board_calibration, "source_board_calibration"
    )
    rows = _load_jsonl(paths.capture_results_jsonl)

    row_has_current = [
        any(field in row for field in _CURRENT_CAPTURE_FIELDS) for row in rows
    ]
    report_has_current = "camera_static_geometry" in report
    if any(row_has_current) and not all(row_has_current):
        raise CaptureFingerprintError(
            "capture_results mixes legacy and current metadata rows"
        )
    if report_has_current != all(row_has_current):
        raise CaptureFingerprintError(
            "hfr_report and capture_results metadata generations disagree"
        )

    device = _parse_device(report, expectations)
    configuration = _validate_report_and_artifacts(
        report,
        qa,
        calibration,
        rows,
        paths,
        artifact_by_name,
        expectations,
        safety_strict=report_has_current,
    )
    _validate_trajectory(paths.trajectory_csv, qa)

    if not report_has_current:
        return CameraCaptureFingerprint(
            qualification_status=QUALIFICATION_STATUS_LEGACY_UNVERIFIED,
            safety_qualified=False,
            reasons=(
                "HFR Recorder 0.5.5-or-earlier lacks qualified camera metadata",
            ),
            metadata_generation="legacy_0_5_5_or_earlier",
            artifacts=artifacts,
            device=device,
            configuration=configuration,
            static_geometry=None,
            frame_metadata=None,
            capture_result_count=len(rows),
        )

    version = _parse_app_version(report.get("app_version"))
    if version < (0, 5, 7):
        raise CaptureFingerprintError(
            "qualified camera metadata requires HFR Recorder 0.5.7 or newer"
        )
    _validate_artifact_integrity(report, paths, artifact_by_name)
    static_geometry = _parse_static_geometry(
        report["camera_static_geometry"], configuration.camera_id
    )
    requested_focus = configuration.requested_focus_distance_diopters
    if requested_focus is None:
        raise CaptureFingerprintError(
            "qualified configuration lacks requested focus distance"
        )
    if requested_focus > static_geometry.minimum_focus_distance_diopters:
        raise CaptureFingerprintError(
            "requested focus distance exceeds the lens minimum-focus limit"
        )
    frame_metadata = _parse_fixed_frame_metadata(rows, static_geometry)
    _validate_focus_settle(report, configuration, frame_metadata)
    return CameraCaptureFingerprint(
        qualification_status=QUALIFICATION_STATUS_QUALIFIED,
        safety_qualified=True,
        reasons=(),
        metadata_generation="hfr_recorder_0_5_7_or_newer",
        artifacts=artifacts,
        device=device,
        configuration=configuration,
        static_geometry=static_geometry,
        frame_metadata=frame_metadata,
        capture_result_count=len(rows),
    )


def revalidate_capture_fingerprint(
    value: Mapping[str, Any],
    *,
    expectations: CaptureExpectations = CaptureExpectations(),
) -> CameraCaptureFingerprint:
    """Revalidate every artifact referenced by an embedded fingerprint."""

    if not isinstance(value, Mapping):
        raise CaptureFingerprintError("embedded capture fingerprint must be an object")
    if value.get("schema") != CAPTURE_FINGERPRINT_SCHEMA:
        raise CaptureFingerprintError("embedded capture fingerprint schema is invalid")
    if value.get("safety_qualified") is not True:
        raise CaptureFingerprintError(
            "embedded capture fingerprint is not safety-qualified"
        )
    artifacts = _object(value.get("artifacts"), "capture fingerprint artifacts")
    paths: dict[str, Path] = {}
    for name in _PATH_FIELDS:
        binding = _object(artifacts.get(name), f"capture fingerprint artifact {name}")
        path = binding.get("path")
        if not isinstance(path, str) or not path:
            raise CaptureFingerprintError(
                f"capture fingerprint artifact {name} path is missing"
            )
        paths[name] = Path(path).expanduser().resolve()
    result = validate_capture_session(
        CaptureSessionPaths(**paths), expectations=expectations
    )
    expected_fingerprint = _nonempty_string(
        value.get("fingerprint_sha256"),
        "capture fingerprint fingerprint_sha256",
    ).lower()
    expected_setup = _nonempty_string(
        value.get("camera_setup_sha256"),
        "capture fingerprint camera_setup_sha256",
    ).lower()
    if result.fingerprint_sha256 != expected_fingerprint:
        raise CaptureFingerprintError(
            "embedded capture fingerprint SHA-256 does not revalidate"
        )
    if result.camera_setup_sha256 != expected_setup:
        raise CaptureFingerprintError(
            "embedded camera setup SHA-256 does not revalidate"
        )
    return result


def _legacy_without_artifacts(reason: str) -> CameraCaptureFingerprint:
    return CameraCaptureFingerprint(
        qualification_status=QUALIFICATION_STATUS_LEGACY_UNVERIFIED,
        safety_qualified=False,
        reasons=(reason,),
        metadata_generation="legacy_manifest_without_capture_session",
        artifacts=(),
        device=None,
        configuration=None,
        static_geometry=None,
        frame_metadata=None,
        capture_result_count=0,
    )


def _validate_expectations(value: CaptureExpectations) -> None:
    if not value.camera_id:
        raise CaptureFingerprintError("expected camera_id must not be empty")
    for name in ("width", "height", "fps"):
        if _strict_int(getattr(value, name), f"expectations.{name}") <= 0:
            raise CaptureFingerprintError(f"expectations.{name} must be positive")
    tolerance = _strict_float(value.fps_tolerance, "expectations.fps_tolerance")
    if tolerance < 0.0:
        raise CaptureFingerprintError(
            "expectations.fps_tolerance must be non-negative"
        )


def _bind_artifact(name: str, path: Path) -> ArtifactBinding:
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise CaptureFingerprintError(f"{name} is not a file: {resolved}")
    size = resolved.stat().st_size
    if size <= 0:
        raise CaptureFingerprintError(f"{name} is empty: {resolved}")
    digest = hashlib.sha256()
    with resolved.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return ArtifactBinding(name, resolved, digest.hexdigest(), size)


def _reject_duplicate_pairs(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise CaptureFingerprintError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _reject_nonfinite_json(value: str) -> Any:
    raise CaptureFingerprintError(f"non-finite JSON number: {value}")


def _json_loads(text: str, label: str) -> Any:
    try:
        return json.loads(
            text,
            object_pairs_hook=_reject_duplicate_pairs,
            parse_constant=_reject_nonfinite_json,
        )
    except CaptureFingerprintError:
        raise
    except (json.JSONDecodeError, TypeError) as exc:
        raise CaptureFingerprintError(f"invalid {label}: {exc}") from exc


def _load_json(path: Path, label: str) -> Mapping[str, Any]:
    try:
        value = _json_loads(Path(path).read_text(encoding="utf-8"), label)
    except UnicodeError as exc:
        raise CaptureFingerprintError(f"{label} is not UTF-8: {exc}") from exc
    if not isinstance(value, Mapping):
        raise CaptureFingerprintError(f"{label} root must be an object")
    return value


def _load_jsonl(path: Path) -> list[Mapping[str, Any]]:
    rows: list[Mapping[str, Any]] = []
    try:
        with Path(path).open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, start=1):
                if not line.strip():
                    raise CaptureFingerprintError(
                        f"capture_results_jsonl line {line_number} is blank"
                    )
                row = _json_loads(
                    line, f"capture_results_jsonl line {line_number}"
                )
                if not isinstance(row, Mapping):
                    raise CaptureFingerprintError(
                        f"capture_results_jsonl line {line_number} is not an object"
                    )
                rows.append(row)
    except UnicodeError as exc:
        raise CaptureFingerprintError(
            f"capture_results_jsonl is not UTF-8: {exc}"
        ) from exc
    if not rows:
        raise CaptureFingerprintError("capture_results_jsonl has no rows")
    return rows


def _parse_device(
    report: Mapping[str, Any], expectations: CaptureExpectations
) -> DeviceIdentity:
    raw = _object(report.get("device"), "hfr_report.device")
    values = {
        field: _nonempty_string(raw.get(field), f"hfr_report.device.{field}")
        for field in _DEVICE_STRING_FIELDS
    }
    sdk_int = _strict_int(raw.get("sdk_int"), "hfr_report.device.sdk_int")
    if sdk_int <= 0:
        raise CaptureFingerprintError("hfr_report.device.sdk_int must be positive")
    result = DeviceIdentity(**values, sdk_int=sdk_int)
    for field in ("manufacturer", "model", "device"):
        expected = getattr(expectations, field)
        if expected is not None and getattr(result, field) != expected:
            raise CaptureFingerprintError(
                f"device {field} {getattr(result, field)!r} != {expected!r}"
            )
    return result


def _validate_report_and_artifacts(
    report: Mapping[str, Any],
    qa: Mapping[str, Any],
    calibration: Mapping[str, Any],
    rows: Sequence[Mapping[str, Any]],
    paths: CaptureSessionPaths,
    artifacts: Mapping[str, ArtifactBinding],
    expectations: CaptureExpectations,
    *,
    safety_strict: bool,
) -> CaptureConfiguration:
    if report.get("schema") != HFR_REPORT_SCHEMA:
        raise CaptureFingerprintError("unsupported hfr_report schema")
    if report.get("status") != "complete" or report.get("error") is not None:
        raise CaptureFingerprintError("hfr_report does not describe a complete run")
    config = _object(report.get("config"), "hfr_report.config")
    camera_id = _nonempty_string(
        config.get("camera_id"), "hfr_report.config.camera_id"
    )
    width = _strict_int(config.get("width"), "hfr_report.config.width")
    height = _strict_int(config.get("height"), "hfr_report.config.height")
    fps = _strict_int(config.get("fps"), "hfr_report.config.fps")
    for field, actual, expected in (
        ("camera_id", camera_id, expectations.camera_id),
        ("width", width, expectations.width),
        ("height", height, expectations.height),
        ("fps", fps, expectations.fps),
    ):
        if actual != expected:
            raise CaptureFingerprintError(
                f"hfr_report.config.{field} {actual!r} != {expected!r}"
            )
    for field in (
        "video_stabilization_requested",
        "optical_stabilization_requested",
    ):
        if config.get(field) != "OFF":
            raise CaptureFingerprintError(f"hfr_report.config.{field} is not OFF")
    if config.get("recording_surface_enabled") is not True:
        raise CaptureFingerprintError("recording surface was not enabled")
    if config.get("recording_backend") != "MediaRecorder":
        raise CaptureFingerprintError("recording backend is not MediaRecorder")
    if config.get("audio_recorded") is not False:
        raise CaptureFingerprintError("capture must not contain audio")
    focus_mode: str | None = None
    requested_focus: float | None = None
    focus_settle_minimum_ms: int | None = None
    focus_settle_timeout_ms: int | None = None
    if safety_strict:
        focus_mode = _nonempty_string(
            config.get("focus_mode"), "hfr_report.config.focus_mode"
        )
        if focus_mode != "fixed":
            raise CaptureFingerprintError(
                "hfr_report.config.focus_mode must be fixed"
            )
        requested_focus = _strict_float(
            config.get("requested_focus_distance_diopters"),
            "hfr_report.config.requested_focus_distance_diopters",
        )
        if requested_focus < 0.0:
            raise CaptureFingerprintError(
                "requested focus distance must be non-negative"
            )
        focus_settle_minimum_ms = _strict_int(
            config.get("focus_settle_minimum_ms"),
            "hfr_report.config.focus_settle_minimum_ms",
        )
        if focus_settle_minimum_ms < 250:
            raise CaptureFingerprintError(
                "focus settle minimum must be at least 250 ms"
            )
        focus_settle_timeout_ms = _strict_int(
            config.get("focus_settle_timeout_ms"),
            "hfr_report.config.focus_settle_timeout_ms",
        )
        if focus_settle_timeout_ms < focus_settle_minimum_ms:
            raise CaptureFingerprintError(
                "focus settle timeout must not precede its minimum"
            )
    capture_rate = _strict_float(
        config.get("media_recorder_capture_rate_fps"),
        "hfr_report.config.media_recorder_capture_rate_fps",
    )
    _require_rate(capture_rate, expectations.fps, expectations, "capture rate")
    orientation = _strict_int(
        report.get("orientation_hint_deg"), "hfr_report.orientation_hint_deg"
    )
    if orientation not in (0, 90, 180, 270):
        raise CaptureFingerprintError("orientation_hint_deg is invalid")

    _validate_capture_rows_and_summary(
        report, rows, expectations, safety_strict=safety_strict
    )
    outputs = _object(report.get("outputs"), "hfr_report.outputs")
    if outputs.get("video") != paths.raw_video.name:
        raise CaptureFingerprintError("hfr_report output video does not match")
    if outputs.get("capture_results_jsonl") != paths.capture_results_jsonl.name:
        raise CaptureFingerprintError(
            "hfr_report capture_results_jsonl output does not match"
        )

    encoded = _object(report.get("encoded_video"), "hfr_report.encoded_video")
    if encoded.get("codec") != "video/avc":
        raise CaptureFingerprintError("encoded video codec is not video/avc")
    encoded_samples = _strict_int(
        encoded.get("sample_count"), "hfr_report.encoded_video.sample_count"
    )
    if encoded_samples <= 0:
        raise CaptureFingerprintError("encoded video has no samples")
    encoded_size = _strict_int(
        encoded.get("file_size_bytes"),
        "hfr_report.encoded_video.file_size_bytes",
    )
    if encoded_size != artifacts["raw_video"].size_bytes:
        raise CaptureFingerprintError("encoded video size does not match raw video")
    encoded_fps = _strict_float(
        encoded.get("measured_encoded_fps"),
        "hfr_report.encoded_video.measured_encoded_fps",
    )
    _require_rate(encoded_fps, expectations.fps, expectations, "encoded FPS")

    if qa.get("schema") != TRAJECTORY_QA_SCHEMA:
        raise CaptureFingerprintError("unsupported trajectory QA schema")
    qa_input = _object(qa.get("input"), "qa_report.input")
    if qa_input.get("sha256") != artifacts["raw_video"].sha256:
        raise CaptureFingerprintError("QA input SHA-256 does not match raw video")
    for field, expected in (
        ("size_bytes", artifacts["raw_video"].size_bytes),
        ("width", expectations.width),
        ("height", expectations.height),
        ("frames", encoded_samples),
    ):
        if _strict_int(qa_input.get(field), f"qa_report.input.{field}") != expected:
            raise CaptureFingerprintError(f"QA input {field} does not match")
    qa_fps = _strict_float(
        qa_input.get("fps_container"), "qa_report.input.fps_container"
    )
    _require_rate(qa_fps, expectations.fps, expectations, "QA container FPS")
    qa_outputs = _object(qa.get("outputs"), "qa_report.outputs")
    trajectory_output = qa_outputs.get("trajectory_csv")
    if not isinstance(trajectory_output, str) or not trajectory_output.strip():
        raise CaptureFingerprintError("QA trajectory_csv output is missing")
    output_path = Path(trajectory_output).expanduser()
    if not output_path.is_absolute():
        output_path = paths.qa_report.parent / output_path
    if output_path.resolve() != paths.trajectory_csv.resolve():
        raise CaptureFingerprintError("QA trajectory_csv output does not match")
    if calibration.get("schema") != BOARD_CALIBRATION_SCHEMA:
        raise CaptureFingerprintError("unsupported board calibration schema")

    return CaptureConfiguration(
        camera_id=camera_id,
        width=width,
        height=height,
        fps=fps,
        orientation_hint_deg=orientation,
        video_stabilization_requested="OFF",
        optical_stabilization_requested="OFF",
        video_stabilization_mode=expectations.video_stabilization_mode,
        optical_stabilization_mode=expectations.optical_stabilization_mode,
        focus_mode=focus_mode,
        requested_focus_distance_diopters=requested_focus,
        focus_settle_minimum_ms=focus_settle_minimum_ms,
        focus_settle_timeout_ms=focus_settle_timeout_ms,
    )


def _validate_capture_rows_and_summary(
    report: Mapping[str, Any],
    rows: Sequence[Mapping[str, Any]],
    expectations: CaptureExpectations,
    *,
    safety_strict: bool,
) -> None:
    timestamps: list[int] = []
    timestamp_set: set[int] = set()
    for index, row in enumerate(rows):
        prefix = f"capture_results[{index}]"
        _strict_int(row.get("frame_number"), prefix + ".frame_number")
        timestamp = _strict_int(
            row.get("sensor_timestamp_ns"), prefix + ".sensor_timestamp_ns"
        )
        if timestamp in timestamp_set:
            raise CaptureFingerprintError("sensor timestamps are not unique")
        timestamps.append(timestamp)
        timestamp_set.add(timestamp)
        for field, expected in (
            ("video_stabilization_mode", expectations.video_stabilization_mode),
            ("optical_stabilization_mode", expectations.optical_stabilization_mode),
        ):
            actual = _strict_int(row.get(field), prefix + "." + field)
            if actual != expected:
                raise CaptureFingerprintError(
                    f"{prefix}.{field} {actual} != required {expected}"
                )
    summary = _object(report.get("capture_results"), "hfr_report.capture_results")
    count = len(rows)
    for field, expected in (
        ("callback_count", count),
        ("sensor_timestamp_count", count),
        ("unique_sensor_frame_count", count),
        ("duplicate_sensor_timestamp_count", 0),
        ("first_sensor_timestamp_ns", min(timestamps)),
        ("last_sensor_timestamp_ns", max(timestamps)),
    ):
        actual = _strict_int(summary.get(field), f"capture_results.{field}")
        if actual != expected:
            raise CaptureFingerprintError(
                f"capture_results.{field} {actual} != {expected}"
            )
    failures = _strict_int(
        summary.get("capture_failure_count"),
        "capture_results.capture_failure_count",
    )
    maximum_failures = math.floor(count * 0.01)
    if failures < 0 or (safety_strict and failures > maximum_failures):
        raise CaptureFingerprintError(
            "capture_results.capture_failure_count exceeds 1% of callbacks"
        )
    measured = _strict_float(
        summary.get("measured_sensor_fps"),
        "capture_results.measured_sensor_fps",
    )
    _require_rate(measured, expectations.fps, expectations, "sensor FPS")
    for field, expected in (
        ("video_stabilization_mode", expectations.video_stabilization_mode),
        ("optical_stabilization_mode", expectations.optical_stabilization_mode),
    ):
        stats = _object(summary.get(field), "capture_results." + field)
        if _strict_int(stats.get("count"), field + ".count") != count:
            raise CaptureFingerprintError(f"{field} summary count does not match")
        for statistic in ("min", "median", "p95", "max"):
            actual = _strict_float(
                stats.get(statistic), f"{field}.{statistic}"
            )
            if actual != float(expected):
                raise CaptureFingerprintError(
                    f"{field}.{statistic} is not fixed at {expected}"
                )


def _validate_artifact_integrity(
    report: Mapping[str, Any],
    paths: CaptureSessionPaths,
    artifacts: Mapping[str, ArtifactBinding],
) -> None:
    integrity = _object(
        report.get("artifact_integrity"), "hfr_report.artifact_integrity"
    )
    for report_name, artifact_name, path in (
        ("video", "raw_video", paths.raw_video),
        (
            "capture_results_jsonl",
            "capture_results_jsonl",
            paths.capture_results_jsonl,
        ),
    ):
        item = _object(
            integrity.get(report_name),
            f"hfr_report.artifact_integrity.{report_name}",
        )
        expected = artifacts[artifact_name]
        filename = _nonempty_string(
            item.get("filename"),
            f"hfr_report.artifact_integrity.{report_name}.filename",
        )
        if filename != path.name:
            raise CaptureFingerprintError(
                f"artifact_integrity.{report_name} filename does not match"
            )
        size = _strict_int(
            item.get("size_bytes"),
            f"hfr_report.artifact_integrity.{report_name}.size_bytes",
        )
        if size != expected.size_bytes:
            raise CaptureFingerprintError(
                f"artifact_integrity.{report_name} size does not match"
            )
        digest = _nonempty_string(
            item.get("sha256"),
            f"hfr_report.artifact_integrity.{report_name}.sha256",
        ).lower()
        if len(digest) != 64 or any(
            character not in "0123456789abcdef" for character in digest
        ):
            raise CaptureFingerprintError(
                f"artifact_integrity.{report_name} SHA-256 is invalid"
            )
        if digest != expected.sha256:
            raise CaptureFingerprintError(
                f"artifact_integrity.{report_name} SHA-256 does not match"
            )


def _validate_focus_settle(
    report: Mapping[str, Any],
    configuration: CaptureConfiguration,
    frame: FrameCameraMetadata,
) -> None:
    settle = _object(report.get("focus_settle"), "hfr_report.focus_settle")
    if settle.get("required") is not True:
        raise CaptureFingerprintError("fixed-focus settling was not required")
    if settle.get("settled") is not True:
        raise CaptureFingerprintError("fixed focus did not settle")
    requested = _strict_float(
        settle.get("requested_distance_diopters"),
        "hfr_report.focus_settle.requested_distance_diopters",
    )
    configured_requested = configuration.requested_focus_distance_diopters
    if configured_requested is None or not math.isclose(
        requested, configured_requested, rel_tol=0.0, abs_tol=1e-9
    ):
        raise CaptureFingerprintError(
            "focus-settle requested distance does not match config"
        )
    applied = _strict_float(
        settle.get("applied_distance_diopters"),
        "hfr_report.focus_settle.applied_distance_diopters",
    )
    if applied < 0.0 or not math.isclose(
        applied,
        frame.focus_distance_diopters,
        rel_tol=1e-6,
        abs_tol=1e-6,
    ):
        raise CaptureFingerprintError(
            "focus-settle applied distance does not match captured focus"
        )
    minimum_ms = _strict_int(
        settle.get("minimum_stable_ms"),
        "hfr_report.focus_settle.minimum_stable_ms",
    )
    timeout_ms = _strict_int(
        settle.get("timeout_ms"), "hfr_report.focus_settle.timeout_ms"
    )
    if (
        configuration.focus_settle_minimum_ms is None
        or minimum_ms != configuration.focus_settle_minimum_ms
    ):
        raise CaptureFingerprintError(
            "focus-settle minimum does not match config"
        )
    if (
        configuration.focus_settle_timeout_ms is None
        or timeout_ms != configuration.focus_settle_timeout_ms
    ):
        raise CaptureFingerprintError(
            "focus-settle timeout does not match config"
        )
    observations = _strict_int(
        settle.get("observation_count"),
        "hfr_report.focus_settle.observation_count",
    )
    stable_observations = _strict_int(
        settle.get("stable_observation_count"),
        "hfr_report.focus_settle.stable_observation_count",
    )
    if (
        observations <= 0
        or stable_observations <= 0
        or stable_observations > observations
    ):
        raise CaptureFingerprintError(
            "focus-settle observation counts do not prove stability"
        )
    if _strict_int(
        settle.get("control_af_mode"),
        "hfr_report.focus_settle.control_af_mode",
    ) != 0:
        raise CaptureFingerprintError("focus-settle autofocus mode is not OFF")
    if _strict_int(
        settle.get("lens_state"), "hfr_report.focus_settle.lens_state"
    ) != 0:
        raise CaptureFingerprintError("focus-settle lens is not STATIONARY")
    started_ns = _strict_int(
        settle.get("started_elapsed_realtime_ns"),
        "hfr_report.focus_settle.started_elapsed_realtime_ns",
    )
    settled_ns = _strict_int(
        settle.get("settled_elapsed_realtime_ns"),
        "hfr_report.focus_settle.settled_elapsed_realtime_ns",
    )
    if started_ns <= 0 or settled_ns <= started_ns:
        raise CaptureFingerprintError("focus-settle timestamps are invalid")
    elapsed_ms = _strict_float(
        settle.get("settle_elapsed_ms"),
        "hfr_report.focus_settle.settle_elapsed_ms",
    )
    timestamp_elapsed_ms = (settled_ns - started_ns) / 1_000_000.0
    if not math.isclose(
        elapsed_ms, timestamp_elapsed_ms, rel_tol=0.0, abs_tol=1e-6
    ):
        raise CaptureFingerprintError(
            "focus-settle elapsed time does not match timestamps"
        )
    if elapsed_ms < minimum_ms or elapsed_ms > timeout_ms:
        raise CaptureFingerprintError(
            "focus-settle elapsed time is outside configured bounds"
        )


def _parse_static_geometry(
    raw_value: Any, expected_camera_id: str
) -> StaticCameraGeometry:
    raw = _object(raw_value, "hfr_report.camera_static_geometry")
    camera_id = _nonempty_string(
        raw.get("camera_id"), "camera_static_geometry.camera_id"
    )
    if camera_id != expected_camera_id:
        raise CaptureFingerprintError("static geometry camera_id does not match")
    ids_raw = raw.get("physical_camera_ids")
    if not isinstance(ids_raw, list) or not ids_raw:
        raise CaptureFingerprintError(
            "camera_static_geometry.physical_camera_ids must be non-empty"
        )
    ids = tuple(
        _nonempty_string(item, "camera_static_geometry.physical_camera_ids[]")
        for item in ids_raw
    )
    if len(set(ids)) != len(ids):
        raise CaptureFingerprintError("physical camera IDs are not unique")
    sensor_orientation = _strict_int(
        raw.get("sensor_orientation_deg"),
        "camera_static_geometry.sensor_orientation_deg",
    )
    if sensor_orientation not in (0, 90, 180, 270):
        raise CaptureFingerprintError("static sensor orientation is invalid")
    focal_lengths = _float_tuple(
        raw.get("available_focal_lengths_mm"),
        "camera_static_geometry.available_focal_lengths_mm",
        minimum_length=1,
        positive=True,
    )
    intrinsics = _float_tuple(
        raw.get("lens_intrinsic_calibration"),
        "camera_static_geometry.lens_intrinsic_calibration",
        exact_length=5,
    )
    if intrinsics[0] <= 0.0 or intrinsics[1] <= 0.0:
        raise CaptureFingerprintError("static focal intrinsics must be positive")
    minimum_focus = _strict_float(
        raw.get("minimum_focus_distance_diopters"),
        "camera_static_geometry.minimum_focus_distance_diopters",
    )
    if minimum_focus < 0.0:
        raise CaptureFingerprintError(
            "static minimum focus distance must be non-negative"
        )
    focus_calibration = _strict_int(
        raw.get("focus_distance_calibration"),
        "camera_static_geometry.focus_distance_calibration",
    )
    if focus_calibration not in (1, 2):
        raise CaptureFingerprintError(
            "focus-distance calibration must be APPROXIMATE or CALIBRATED"
        )
    autofocus_raw = raw.get("autofocus_modes")
    if not isinstance(autofocus_raw, list) or not autofocus_raw:
        raise CaptureFingerprintError(
            "camera_static_geometry.autofocus_modes must be non-empty"
        )
    autofocus_modes = tuple(
        _strict_int(item, "camera_static_geometry.autofocus_modes[]")
        for item in autofocus_raw
    )
    if len(set(autofocus_modes)) != len(autofocus_modes):
        raise CaptureFingerprintError("static autofocus modes are not unique")
    if 0 not in autofocus_modes:
        raise CaptureFingerprintError("static autofocus modes do not include AF OFF")
    request_supported = _strict_bool(
        raw.get("focus_distance_request_supported"),
        "camera_static_geometry.focus_distance_request_supported",
    )
    result_supported = _strict_bool(
        raw.get("focus_distance_result_supported"),
        "camera_static_geometry.focus_distance_result_supported",
    )
    lens_state_supported = _strict_bool(
        raw.get("lens_state_result_supported"),
        "camera_static_geometry.lens_state_result_supported",
    )
    if not request_supported or not result_supported or not lens_state_supported:
        raise CaptureFingerprintError(
            "fixed-focus request/result/lens-state metadata must be supported"
        )
    return StaticCameraGeometry(
        camera_id=camera_id,
        physical_camera_ids=ids,
        sensor_orientation_deg=sensor_orientation,
        active_array=_parse_rect(
            raw.get("active_array"), "camera_static_geometry.active_array"
        ),
        pre_correction_active_array=_parse_rect(
            raw.get("pre_correction_active_array"),
            "camera_static_geometry.pre_correction_active_array",
        ),
        pixel_array_size=_parse_size(
            raw.get("pixel_array_size"),
            "camera_static_geometry.pixel_array_size",
        ),
        sensor_physical_size_mm=_parse_size_f(
            raw.get("sensor_physical_size_mm"),
            "camera_static_geometry.sensor_physical_size_mm",
        ),
        available_focal_lengths_mm=focal_lengths,
        lens_intrinsic_calibration=intrinsics,
        lens_distortion=_float_tuple(
            raw.get("lens_distortion"),
            "camera_static_geometry.lens_distortion",
            exact_length=5,
        ),
        minimum_focus_distance_diopters=minimum_focus,
        focus_distance_calibration=focus_calibration,
        autofocus_modes=autofocus_modes,
        focus_distance_request_supported=request_supported,
        focus_distance_result_supported=result_supported,
        lens_state_result_supported=lens_state_supported,
    )


def _parse_fixed_frame_metadata(
    rows: Sequence[Mapping[str, Any]], static: StaticCameraGeometry
) -> FrameCameraMetadata:
    parsed: list[FrameCameraMetadata] = []
    for index, row in enumerate(rows):
        prefix = f"capture_results[{index}]"
        missing = [
            field for field in _QUALIFIED_CAPTURE_FIELDS if field not in row
        ]
        if missing:
            raise CaptureFingerprintError(
                f"{prefix} lacks current metadata: {', '.join(missing)}"
            )
        physical_id = _nonempty_string(
            row.get("active_physical_camera_id"),
            prefix + ".active_physical_camera_id",
        )
        if physical_id not in static.physical_camera_ids:
            raise CaptureFingerprintError(
                f"{prefix} active physical camera is not in static geometry"
            )
        focus = _strict_float(
            row.get("focus_distance_diopters"),
            prefix + ".focus_distance_diopters",
        )
        if focus < 0.0:
            raise CaptureFingerprintError(
                f"{prefix} focus distance must be non-negative"
            )
        if focus > static.minimum_focus_distance_diopters:
            raise CaptureFingerprintError(
                f"{prefix} focus distance exceeds the lens limit"
            )
        control_af_mode = _strict_int(
            row.get("control_af_mode"), prefix + ".control_af_mode"
        )
        if control_af_mode != 0:
            raise CaptureFingerprintError(f"{prefix} autofocus mode is not OFF")
        lens_state = _strict_int(row.get("lens_state"), prefix + ".lens_state")
        if lens_state != 0:
            raise CaptureFingerprintError(f"{prefix} lens is not STATIONARY")
        focal = _strict_float(
            row.get("lens_focal_length_mm"), prefix + ".lens_focal_length_mm"
        )
        if focal <= 0.0:
            raise CaptureFingerprintError(f"{prefix} focal length is not positive")
        if not any(
            math.isclose(focal, item, rel_tol=1e-6, abs_tol=1e-6)
            for item in static.available_focal_lengths_mm
        ):
            raise CaptureFingerprintError(
                f"{prefix} focal length is absent from static geometry"
            )
        intrinsics = _float_tuple(
            row.get("lens_intrinsic_calibration"),
            prefix + ".lens_intrinsic_calibration",
            exact_length=5,
        )
        if intrinsics[0] <= 0.0 or intrinsics[1] <= 0.0:
            raise CaptureFingerprintError(
                f"{prefix} focal intrinsics must be positive"
            )
        zoom = _strict_float(
            row.get("control_zoom_ratio"), prefix + ".control_zoom_ratio"
        )
        if zoom <= 0.0:
            raise CaptureFingerprintError(f"{prefix} zoom ratio is not positive")
        parsed.append(
            FrameCameraMetadata(
                active_physical_camera_id=physical_id,
                focus_distance_diopters=focus,
                lens_focal_length_mm=focal,
                lens_intrinsic_calibration=intrinsics,
                lens_distortion=_float_tuple(
                    row.get("lens_distortion"),
                    prefix + ".lens_distortion",
                    exact_length=5,
                ),
                scaler_crop_region=_parse_rect(
                    row.get("scaler_crop_region"),
                    prefix + ".scaler_crop_region",
                ),
                control_zoom_ratio=zoom,
                control_af_mode=control_af_mode,
                lens_state=lens_state,
            )
        )
    first = parsed[0]
    for index, item in enumerate(parsed[1:], start=1):
        if item != first:
            raise CaptureFingerprintError(
                "per-frame camera metadata changed at capture_results["
                f"{index}]"
            )
    return first


def _validate_trajectory(path: Path, qa: Mapping[str, Any]) -> None:
    row_count = 0
    try:
        with Path(path).open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            fields = reader.fieldnames
            if not fields or len(fields) != len(set(fields)):
                raise CaptureFingerprintError(
                    "trajectory CSV header is missing or contains duplicates"
                )
            missing = sorted(_TRAJECTORY_REQUIRED_COLUMNS.difference(fields))
            if missing:
                raise CaptureFingerprintError(
                    "trajectory CSV lacks required columns: " + ", ".join(missing)
                )
            row_count = sum(1 for _ in reader)
    except UnicodeError as exc:
        raise CaptureFingerprintError(f"trajectory CSV is not UTF-8: {exc}") from exc
    expected = _strict_int(
        _object(qa.get("input"), "qa_report.input").get("frames"),
        "qa_report.input.frames",
    )
    if row_count != expected:
        raise CaptureFingerprintError(
            f"trajectory CSV row count {row_count} != QA frame count {expected}"
        )


def _parse_rect(value: Any, label: str) -> RectFingerprint:
    raw = _object(value, label)
    left = _strict_int(raw.get("left"), label + ".left")
    top = _strict_int(raw.get("top"), label + ".top")
    right = _strict_int(raw.get("right"), label + ".right")
    bottom = _strict_int(raw.get("bottom"), label + ".bottom")
    result = RectFingerprint(left, top, right, bottom)
    if result.width <= 0 or result.height <= 0:
        raise CaptureFingerprintError(f"{label} must have positive dimensions")
    if _strict_int(raw.get("width"), label + ".width") != result.width:
        raise CaptureFingerprintError(f"{label}.width is inconsistent")
    if _strict_int(raw.get("height"), label + ".height") != result.height:
        raise CaptureFingerprintError(f"{label}.height is inconsistent")
    return result


def _parse_size(value: Any, label: str) -> SizeFingerprint:
    raw = _object(value, label)
    width = _strict_int(raw.get("width"), label + ".width")
    height = _strict_int(raw.get("height"), label + ".height")
    if width <= 0 or height <= 0:
        raise CaptureFingerprintError(f"{label} must be positive")
    return SizeFingerprint(width, height)


def _parse_size_f(value: Any, label: str) -> SizeFFingerprint:
    raw = _object(value, label)
    width = _strict_float(raw.get("width"), label + ".width")
    height = _strict_float(raw.get("height"), label + ".height")
    if width <= 0.0 or height <= 0.0:
        raise CaptureFingerprintError(f"{label} must be positive")
    return SizeFFingerprint(width, height)


def _float_tuple(
    value: Any,
    label: str,
    *,
    exact_length: int | None = None,
    minimum_length: int | None = None,
    positive: bool = False,
) -> tuple[float, ...]:
    if not isinstance(value, list):
        raise CaptureFingerprintError(f"{label} must be an array")
    if exact_length is not None and len(value) != exact_length:
        raise CaptureFingerprintError(
            f"{label} must contain exactly {exact_length} values"
        )
    if minimum_length is not None and len(value) < minimum_length:
        raise CaptureFingerprintError(
            f"{label} must contain at least {minimum_length} values"
        )
    result = tuple(_strict_float(item, f"{label}[]") for item in value)
    if positive and any(item <= 0.0 for item in result):
        raise CaptureFingerprintError(f"{label} values must be positive")
    return result


def _object(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CaptureFingerprintError(f"{label} must be an object")
    return value


def _nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise CaptureFingerprintError(f"{label} must be a non-empty string")
    return value


def _parse_app_version(value: Any) -> tuple[int, int, int]:
    text = _nonempty_string(value, "hfr_report.app_version")
    parts = text.split(".")
    if len(parts) != 3 or any(not part.isdigit() for part in parts):
        raise CaptureFingerprintError(
            "hfr_report.app_version must be a numeric MAJOR.MINOR.PATCH version"
        )
    return (int(parts[0]), int(parts[1]), int(parts[2]))


def _strict_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise CaptureFingerprintError(f"{label} must be an integer")
    return value


def _strict_bool(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise CaptureFingerprintError(f"{label} must be a boolean")
    return value


def _strict_float(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CaptureFingerprintError(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise CaptureFingerprintError(f"{label} must be a finite number")
    return result


def _require_rate(
    actual: float,
    expected: float,
    expectations: CaptureExpectations,
    label: str,
) -> None:
    if abs(actual - expected) > expectations.fps_tolerance:
        raise CaptureFingerprintError(
            f"{label} {actual} is outside {expected} +/- "
            f"{expectations.fps_tolerance}"
        )


def _canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


__all__ = [
    "ArtifactBinding",
    "BOARD_CALIBRATION_SCHEMA",
    "CAPTURE_FINGERPRINT_SCHEMA",
    "CAPTURE_SESSION_SCHEMA",
    "CameraCaptureFingerprint",
    "CaptureConfiguration",
    "CaptureExpectations",
    "CaptureFingerprintError",
    "CaptureSessionPaths",
    "DeviceIdentity",
    "FrameCameraMetadata",
    "HFR_REPORT_SCHEMA",
    "QUALIFICATION_STATUS_LEGACY_UNVERIFIED",
    "QUALIFICATION_STATUS_QUALIFIED",
    "RectFingerprint",
    "SizeFFingerprint",
    "SizeFingerprint",
    "StaticCameraGeometry",
    "TRAJECTORY_QA_SCHEMA",
    "load_capture_session",
    "revalidate_capture_fingerprint",
    "validate_capture_session",
]
