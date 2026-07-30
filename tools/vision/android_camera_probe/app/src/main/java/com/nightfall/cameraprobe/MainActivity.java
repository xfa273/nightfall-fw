package com.nightfall.cameraprobe;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;
import android.graphics.Rect;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.CamcorderProfile;
import android.media.EncoderProfiles;
import android.os.Build;
import android.os.Bundle;
import android.util.Range;
import android.util.Size;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

public final class MainActivity extends Activity {
    private static final int CAMERA_PERMISSION_REQUEST = 1001;
    private static final String REPORT_FILENAME = "camera_capabilities.json";
    private static final String REPORT_TEMP_FILENAME =
            REPORT_FILENAME + ".tmp";
    private static final String PROBE_NONCE_EXTRA = "probe_nonce";
    private TextView reportView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(createContentView());
        prepareProbe();
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        setIntent(intent);
        prepareProbe();
    }

    private View createContentView() {
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        int padding = dp(12);
        root.setPadding(padding, padding, padding, padding);

        Button refresh = new Button(this);
        refresh.setText("Refresh camera report");
        refresh.setOnClickListener(view -> prepareProbe());
        root.addView(refresh);

        reportView = new TextView(this);
        reportView.setTextIsSelectable(true);
        reportView.setTextSize(11.0f);
        reportView.setPadding(0, dp(8), 0, 0);
        ScrollView scroll = new ScrollView(this);
        scroll.addView(reportView);
        root.addView(
                scroll,
                new LinearLayout.LayoutParams(
                        LinearLayout.LayoutParams.MATCH_PARENT,
                        0,
                        1.0f
                )
        );
        return root;
    }

    private void prepareProbe() {
        try {
            clearReportFiles();
        } catch (IOException exception) {
            reportView.setText(
                    "Unable to remove the previous report: "
                            + exception.getMessage()
            );
            return;
        }
        if (checkSelfPermission(Manifest.permission.CAMERA)
                == PackageManager.PERMISSION_GRANTED) {
            runProbe();
        } else {
            requestPermissions(
                    new String[]{Manifest.permission.CAMERA},
                    CAMERA_PERMISSION_REQUEST
            );
        }
    }

    private int dp(int value) {
        return Math.round(
                value * getResources().getDisplayMetrics().density
        );
    }

    @Override
    public void onRequestPermissionsResult(
            int requestCode,
            String[] permissions,
            int[] grantResults
    ) {
        super.onRequestPermissionsResult(
                requestCode,
                permissions,
                grantResults
        );
        if (requestCode == CAMERA_PERMISSION_REQUEST
                && grantResults.length > 0
                && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            runProbe();
        } else {
            reportView.setText(
                    "Camera permission is required to enumerate all cameras."
            );
        }
    }

    private void runProbe() {
        reportView.setText("Reading Camera2 characteristics...");
        try {
            clearReportFiles();
            JSONObject report = buildReport();
            String formatted = report.toString(2);
            writeReportAtomically(formatted);
            reportView.setText(
                    "Saved to internal files/" + REPORT_FILENAME + "\n\n"
                            + formatted
            );
        } catch (Exception exception) {
            String cleanupFailure = clearReportFilesAfterFailure();
            reportView.setText(
                    exception.getClass().getSimpleName() + ": "
                            + exception.getMessage()
                            + cleanupFailure
            );
        }
    }

    private JSONObject buildReport() throws Exception {
        JSONObject root = new JSONObject();
        root.put("schema", "nightfall_android_camera_probe_v2");
        root.put("generated_at_utc", Instant.now().toString());
        root.put(
                "probe_nonce",
                valueOrNull(getIntent().getStringExtra(PROBE_NONCE_EXTRA))
        );
        root.put("probe_app", buildProbeApp());
        root.put("device", buildDevice());
        CameraManager manager = (CameraManager) getSystemService(
                Context.CAMERA_SERVICE
        );
        JSONArray cameras = new JSONArray();
        String[] cameraIds = manager.getCameraIdList();
        Arrays.sort(cameraIds);
        for (String cameraId : cameraIds) {
            try {
                cameras.put(
                        buildCamera(
                                cameraId,
                                manager.getCameraCharacteristics(cameraId)
                        )
                );
            } catch (Exception exception) {
                cameras.put(buildCameraError(cameraId, exception));
            }
        }
        root.put("cameras", cameras);
        root.put(
                "measurement_note",
                "Static capability report only. A real recording must still "
                        + "verify CaptureResult timestamps, exposure, frame "
                        + "duration, rolling-shutter skew, encoder PTS, and "
                        + "dropped frames."
        );
        return root;
    }

    private JSONObject buildProbeApp() throws Exception {
        PackageInfo packageInfo;
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            packageInfo = getPackageManager().getPackageInfo(
                    getPackageName(),
                    PackageManager.PackageInfoFlags.of(0)
            );
        } else {
            packageInfo = getPackageManager().getPackageInfo(
                    getPackageName(),
                    0
            );
        }
        JSONObject object = new JSONObject();
        object.put("package_name", getPackageName());
        object.put("version_name", valueOrNull(packageInfo.versionName));
        object.put("version_code", packageInfo.getLongVersionCode());
        return object;
    }

    private JSONObject buildDevice() throws JSONException {
        JSONObject object = new JSONObject();
        object.put("manufacturer", Build.MANUFACTURER);
        object.put("brand", Build.BRAND);
        object.put("model", Build.MODEL);
        object.put("device", Build.DEVICE);
        object.put("product", Build.PRODUCT);
        object.put("fingerprint", Build.FINGERPRINT);
        object.put("android_release", Build.VERSION.RELEASE);
        object.put("sdk_int", Build.VERSION.SDK_INT);
        return object;
    }

    private JSONObject buildCamera(
            String cameraId,
            CameraCharacteristics characteristics
    ) throws JSONException {
        JSONObject object = new JSONObject();
        object.put("camera_id", cameraId);
        object.put("probe_status", "ok");
        object.put(
                "lens_facing",
                lensFacingName(
                        characteristics.get(
                                CameraCharacteristics.LENS_FACING
                        )
                )
        );
        Integer hardwareLevel = characteristics.get(
                CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL
        );
        object.put(
                "hardware_level",
                hardwareLevelName(hardwareLevel)
        );
        object.put(
                "sensor_orientation_deg",
                valueOrNull(
                        characteristics.get(
                                CameraCharacteristics.SENSOR_ORIENTATION
                        )
                )
        );
        object.put(
                "sensor_timestamp_source",
                timestampSourceName(
                        characteristics.get(
                                CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE
                        )
                )
        );
        object.put(
                "physical_camera_ids",
                strings(
                        characteristics.getPhysicalCameraIds()
                )
        );
        int[] capabilities = characteristics.get(
                CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES
        );
        boolean camcorderProfileQueryEligible =
                isNumericCameraId(cameraId)
                        && contains(
                                capabilities,
                                CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_BACKWARD_COMPATIBLE
                        )
                        && hardwareLevel != null
                        && hardwareLevel.intValue()
                        != CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_EXTERNAL;
        object.put("capabilities", integers(capabilities));
        object.put(
                "supports_constrained_high_speed",
                contains(
                        capabilities,
                        CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO
                )
        );
        object.put(
                "supports_manual_sensor",
                contains(
                        capabilities,
                        CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR
                )
        );
        object.put(
                "exposure_time_range_ns",
                rangeLong(
                        characteristics.get(
                                CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE
                        )
                )
        );
        object.put(
                "sensitivity_iso_range",
                rangeInteger(
                        characteristics.get(
                                CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE
                        )
                )
        );
        object.put(
                "active_array",
                rectangle(
                        characteristics.get(
                                CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE
                        )
                )
        );
        object.put(
                "ae_target_fps_ranges",
                rangesInteger(
                        characteristics.get(
                                CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES
                        )
                )
        );
        object.put(
                "video_stabilization_modes",
                integers(
                        characteristics.get(
                                CameraCharacteristics.CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES
                        )
                )
        );
        object.put(
                "optical_stabilization_modes",
                integers(
                        characteristics.get(
                                CameraCharacteristics.LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION
                        )
                )
        );
        object.put(
                "af_modes",
                integers(
                        characteristics.get(
                                CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES
                        )
                )
        );
        object.put(
                "available_capture_request_keys",
                requestKeys(
                        characteristics.getAvailableCaptureRequestKeys()
                )
        );
        object.put(
                "available_capture_result_keys",
                resultKeys(
                        characteristics.getAvailableCaptureResultKeys()
                )
        );

        StreamConfigurationMap map = characteristics.get(
                CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP
        );
        object.put(
                "constrained_high_speed",
                buildHighSpeedConfigurations(map)
        );
        object.put(
                "camcorder_profile_query_eligible",
                camcorderProfileQueryEligible
        );
        object.put(
                "high_speed_encoder_profiles",
                camcorderProfileQueryEligible
                        ? buildEncoderProfiles(cameraId)
                        : new JSONArray()
        );
        return object;
    }

    private JSONObject buildCameraError(
            String cameraId,
            Exception exception
    ) throws JSONException {
        JSONObject object = new JSONObject();
        object.put("camera_id", cameraId);
        object.put("probe_status", "error");
        object.put("error_type", exception.getClass().getName());
        object.put("error_message", valueOrNull(exception.getMessage()));
        return object;
    }

    private JSONArray buildHighSpeedConfigurations(
            StreamConfigurationMap map
    ) throws JSONException {
        JSONArray configurations = new JSONArray();
        if (map == null) {
            return configurations;
        }
        Size[] sizes = map.getHighSpeedVideoSizes();
        if (sizes == null) {
            return configurations;
        }
        Arrays.sort(
                sizes,
                (left, right) -> {
                    int width = Integer.compare(left.getWidth(), right.getWidth());
                    return width != 0
                            ? width
                            : Integer.compare(left.getHeight(), right.getHeight());
                }
        );
        for (Size size : sizes) {
            JSONObject item = new JSONObject();
            item.put("width", size.getWidth());
            item.put("height", size.getHeight());
            item.put(
                    "fps_ranges",
                    rangesInteger(
                            map.getHighSpeedVideoFpsRangesFor(size)
                    )
            );
            configurations.put(item);
        }
        return configurations;
    }

    private JSONArray buildEncoderProfiles(String cameraId)
            throws JSONException {
        int[] qualities = {
                CamcorderProfile.QUALITY_HIGH_SPEED_LOW,
                CamcorderProfile.QUALITY_HIGH_SPEED_HIGH,
                CamcorderProfile.QUALITY_HIGH_SPEED_480P,
                CamcorderProfile.QUALITY_HIGH_SPEED_720P,
                CamcorderProfile.QUALITY_HIGH_SPEED_1080P,
                CamcorderProfile.QUALITY_HIGH_SPEED_2160P,
                CamcorderProfile.QUALITY_HIGH_SPEED_CIF,
                CamcorderProfile.QUALITY_HIGH_SPEED_VGA,
                CamcorderProfile.QUALITY_HIGH_SPEED_4KDCI
        };
        String[] names = {
                "HIGH_SPEED_LOW",
                "HIGH_SPEED_HIGH",
                "HIGH_SPEED_480P",
                "HIGH_SPEED_720P",
                "HIGH_SPEED_1080P",
                "HIGH_SPEED_2160P",
                "HIGH_SPEED_CIF",
                "HIGH_SPEED_VGA",
                "HIGH_SPEED_4KDCI"
        };
        JSONArray result = new JSONArray();
        for (int index = 0; index < qualities.length; index++) {
            EncoderProfiles profiles = CamcorderProfile.getAll(
                    cameraId,
                    qualities[index]
            );
            if (profiles == null) {
                continue;
            }
            JSONObject item = new JSONObject();
            item.put("quality", names[index]);
            item.put(
                    "recommended_file_format",
                    profiles.getRecommendedFileFormat()
            );
            JSONArray videos = new JSONArray();
            List<EncoderProfiles.VideoProfile> videoProfiles =
                    new ArrayList<>(profiles.getVideoProfiles());
            videoProfiles.sort(MainActivity::compareVideoProfiles);
            for (EncoderProfiles.VideoProfile profile : videoProfiles) {
                JSONObject video = new JSONObject();
                video.put("codec", profile.getCodec());
                video.put("media_type", profile.getMediaType());
                video.put("width", profile.getWidth());
                video.put("height", profile.getHeight());
                video.put("frame_rate", profile.getFrameRate());
                video.put("bitrate", profile.getBitrate());
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                    video.put("bit_depth", profile.getBitDepth());
                    video.put(
                            "chroma_subsampling",
                            profile.getChromaSubsampling()
                    );
                    video.put("hdr_format", profile.getHdrFormat());
                }
                videos.put(video);
            }
            item.put("video_profiles", videos);
            result.put(item);
        }
        return result;
    }

    private JSONArray requestKeys(List<CaptureRequest.Key<?>> keys) {
        JSONArray result = new JSONArray();
        if (keys == null) {
            return result;
        }
        String[] names = new String[keys.size()];
        for (int index = 0; index < keys.size(); index++) {
            names[index] = keys.get(index).getName();
        }
        Arrays.sort(names);
        for (String name : names) {
            result.put(name);
        }
        return result;
    }

    private JSONArray resultKeys(List<CaptureResult.Key<?>> keys) {
        JSONArray result = new JSONArray();
        if (keys == null) {
            return result;
        }
        String[] names = new String[keys.size()];
        for (int index = 0; index < keys.size(); index++) {
            names[index] = keys.get(index).getName();
        }
        Arrays.sort(names);
        for (String name : names) {
            result.put(name);
        }
        return result;
    }

    private static int compareVideoProfiles(
            EncoderProfiles.VideoProfile left,
            EncoderProfiles.VideoProfile right
    ) {
        int comparison = Integer.compare(left.getWidth(), right.getWidth());
        if (comparison != 0) {
            return comparison;
        }
        comparison = Integer.compare(left.getHeight(), right.getHeight());
        if (comparison != 0) {
            return comparison;
        }
        comparison = Integer.compare(
                left.getFrameRate(),
                right.getFrameRate()
        );
        if (comparison != 0) {
            return comparison;
        }
        comparison = Integer.compare(left.getCodec(), right.getCodec());
        if (comparison != 0) {
            return comparison;
        }
        comparison = compareNullableStrings(
                left.getMediaType(),
                right.getMediaType()
        );
        if (comparison != 0) {
            return comparison;
        }
        comparison = Integer.compare(left.getBitrate(), right.getBitrate());
        if (comparison != 0
                || Build.VERSION.SDK_INT < Build.VERSION_CODES.TIRAMISU) {
            return comparison;
        }
        comparison = Integer.compare(
                left.getBitDepth(),
                right.getBitDepth()
        );
        if (comparison != 0) {
            return comparison;
        }
        comparison = Integer.compare(
                left.getChromaSubsampling(),
                right.getChromaSubsampling()
        );
        return comparison != 0
                ? comparison
                : Integer.compare(left.getHdrFormat(), right.getHdrFormat());
    }

    private static int compareNullableStrings(String left, String right) {
        if (left == null) {
            return right == null ? 0 : -1;
        }
        if (right == null) {
            return 1;
        }
        return left.compareTo(right);
    }

    private static boolean isNumericCameraId(String cameraId) {
        try {
            return Integer.parseInt(cameraId) >= 0;
        } catch (NumberFormatException exception) {
            return false;
        }
    }

    private static boolean contains(int[] values, int target) {
        if (values == null) {
            return false;
        }
        for (int value : values) {
            if (value == target) {
                return true;
            }
        }
        return false;
    }

    private static Object valueOrNull(Object value) {
        return value == null ? JSONObject.NULL : value;
    }

    private static JSONArray integers(int[] values) {
        JSONArray result = new JSONArray();
        if (values != null) {
            int[] sorted = values.clone();
            Arrays.sort(sorted);
            for (int value : sorted) {
                result.put(value);
            }
        }
        return result;
    }

    private static JSONArray strings(Set<String> values) {
        JSONArray result = new JSONArray();
        if (values != null) {
            String[] sorted = values.toArray(new String[0]);
            Arrays.sort(sorted);
            for (String value : sorted) {
                result.put(value);
            }
        }
        return result;
    }

    private static JSONArray rangeLong(Range<Long> range) {
        JSONArray result = new JSONArray();
        if (range != null) {
            result.put(range.getLower());
            result.put(range.getUpper());
        }
        return result;
    }

    private static JSONArray rangeInteger(Range<Integer> range) {
        JSONArray result = new JSONArray();
        if (range != null) {
            result.put(range.getLower());
            result.put(range.getUpper());
        }
        return result;
    }

    private static JSONArray rangesInteger(Range<Integer>[] ranges) {
        JSONArray result = new JSONArray();
        if (ranges != null) {
            Range<Integer>[] sorted = ranges.clone();
            Arrays.sort(
                    sorted,
                    (left, right) -> {
                        int lower = Integer.compare(
                                left.getLower(),
                                right.getLower()
                        );
                        return lower != 0
                                ? lower
                                : Integer.compare(
                                        left.getUpper(),
                                        right.getUpper()
                                );
                    }
            );
            for (Range<Integer> range : sorted) {
                result.put(rangeInteger(range));
            }
        }
        return result;
    }

    private static Object rectangle(Rect rect) throws JSONException {
        if (rect == null) {
            return JSONObject.NULL;
        }
        JSONObject result = new JSONObject();
        result.put("left", rect.left);
        result.put("top", rect.top);
        result.put("right", rect.right);
        result.put("bottom", rect.bottom);
        result.put("width", rect.width());
        result.put("height", rect.height());
        return result;
    }

    private static String lensFacingName(Integer value) {
        if (value == null) {
            return "UNKNOWN";
        }
        switch (value) {
            case CameraCharacteristics.LENS_FACING_BACK:
                return "BACK";
            case CameraCharacteristics.LENS_FACING_FRONT:
                return "FRONT";
            case CameraCharacteristics.LENS_FACING_EXTERNAL:
                return "EXTERNAL";
            default:
                return "UNKNOWN_" + value;
        }
    }

    private static String hardwareLevelName(Integer value) {
        if (value == null) {
            return "UNKNOWN";
        }
        switch (value) {
            case CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY:
                return "LEGACY";
            case CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED:
                return "LIMITED";
            case CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_FULL:
                return "FULL";
            case CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_3:
                return "LEVEL_3";
            case CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL_EXTERNAL:
                return "EXTERNAL";
            default:
                return "UNKNOWN_" + value;
        }
    }

    private static String timestampSourceName(Integer value) {
        if (value == null) {
            return "UNKNOWN";
        }
        switch (value) {
            case CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME:
                return "REALTIME";
            case CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN:
                return "UNKNOWN";
            default:
                return "UNKNOWN_" + value;
        }
    }

    private void clearReportFiles() throws IOException {
        Files.deleteIfExists(
                new File(getFilesDir(), REPORT_FILENAME).toPath()
        );
        Files.deleteIfExists(
                new File(getFilesDir(), REPORT_TEMP_FILENAME).toPath()
        );
    }

    private String clearReportFilesAfterFailure() {
        try {
            clearReportFiles();
            return "";
        } catch (IOException cleanupException) {
            return "\nReport cleanup also failed: "
                    + cleanupException.getMessage();
        }
    }

    private void writeReportAtomically(String contents) throws IOException {
        File report = new File(getFilesDir(), REPORT_FILENAME);
        File temporary = new File(getFilesDir(), REPORT_TEMP_FILENAME);
        try {
            try (FileOutputStream stream = new FileOutputStream(temporary)) {
                stream.write(contents.getBytes(StandardCharsets.UTF_8));
                stream.write('\n');
                stream.getFD().sync();
            }
            Files.move(
                    temporary.toPath(),
                    report.toPath(),
                    StandardCopyOption.ATOMIC_MOVE
            );
        } catch (IOException exception) {
            Files.deleteIfExists(temporary.toPath());
            throw exception;
        }
    }
}
