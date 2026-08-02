package com.nightfall.hfrrecorder;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Context;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraConstrainedHighSpeedCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureFailure;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.OutputConfiguration;
import android.hardware.camera2.params.SessionConfiguration;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.MediaExtractor;
import android.media.MediaFormat;
import android.media.MediaRecorder;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.util.Range;
import android.util.Size;
import android.view.Surface;
import android.view.TextureView;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.TreeSet;
import java.util.concurrent.atomic.AtomicBoolean;

final class HfrRecorder {
    private static final boolean ENABLE_RECORDING_SURFACE = true;

    static final String REPORT_FILENAME = "hfr_report.json";
    static final String VIDEO_FILENAME = "hfr_capture.mp4";
    static final String CAPTURE_SIDECAR_FILENAME =
            "capture_results.jsonl";
    static final String ENCODER_SIDECAR_FILENAME =
            "encoder_samples.jsonl";

    interface Listener {
        void onStatus(String message);

        void onArmed();

        void onRecordingStarted();

        void onCancelled(String message);

        void onFinished(String message);

        void onError(String message);
    }

    static final class Config {
        final String nonce;
        final String cameraId;
        final int width;
        final int height;
        final int fps;
        final int durationSeconds;
        final int bitrate;
        final int exposureUs;
        final int iso;
        final boolean enablePreview;
        final boolean opticalTrigger;
        final int opticalTriggerScore;
        final int opticalTriggerHotPixels;
        final int opticalStopTailMs;
        final boolean retainRunOutput;

        Config(
                String nonce,
                String cameraId,
                int width,
                int height,
                int fps,
                int durationSeconds,
                int bitrate,
                int exposureUs,
                int iso,
                boolean enablePreview,
                boolean opticalTrigger,
                int opticalTriggerScore,
                int opticalTriggerHotPixels,
                int opticalStopTailMs,
                boolean retainRunOutput
        ) {
            this.nonce = nonce;
            this.cameraId = cameraId;
            this.width = width;
            this.height = height;
            this.fps = fps;
            this.durationSeconds = durationSeconds;
            this.bitrate = bitrate;
            this.exposureUs = exposureUs;
            this.iso = iso;
            this.enablePreview = enablePreview;
            this.opticalTrigger = opticalTrigger;
            this.opticalTriggerScore = opticalTriggerScore;
            this.opticalTriggerHotPixels = opticalTriggerHotPixels;
            this.opticalStopTailMs = opticalStopTailMs;
            this.retainRunOutput = retainRunOutput;
        }

        void validate() {
            if (nonce.isBlank()) {
                throw new IllegalArgumentException(
                        "record_nonce must not be blank"
                );
            }
            if (cameraId == null || cameraId.isBlank()) {
                throw new IllegalArgumentException(
                        "camera_id must not be blank"
                );
            }
            if (width < 320 || height < 240) {
                throw new IllegalArgumentException(
                        "recording dimensions are too small"
                );
            }
            if (fps < 30 || fps > 480) {
                throw new IllegalArgumentException(
                        "fps must be in 30..480"
                );
            }
            if (durationSeconds < 1 || durationSeconds > 60) {
                throw new IllegalArgumentException(
                        "duration_seconds must be in 1..60"
                );
            }
            if (bitrate < 1_000_000 || bitrate > 220_000_000) {
                throw new IllegalArgumentException(
                        "bitrate must be in 1M..220M"
                );
            }
            if (exposureUs < 0) {
                throw new IllegalArgumentException(
                        "exposure_us must be non-negative"
                );
            }
            if (exposureUs > 0 && exposureUs * fps >= 1_000_000) {
                throw new IllegalArgumentException(
                        "manual exposure must be shorter than frame period"
                );
            }
            if (iso < 50 || iso > 6400) {
                throw new IllegalArgumentException(
                        "iso must be in 50..6400"
                );
            }
            if (opticalTrigger && !enablePreview) {
                throw new IllegalArgumentException(
                        "optical trigger requires preview"
                );
            }
            if (opticalTriggerScore < 1
                    || opticalTriggerHotPixels < 1) {
                throw new IllegalArgumentException(
                        "optical trigger thresholds must be positive"
                );
            }
            if (opticalStopTailMs < 0 || opticalStopTailMs > 5000) {
                throw new IllegalArgumentException(
                        "optical stop tail must be in 0..5000 ms"
                );
            }
        }

        JSONObject toJson() throws Exception {
            JSONObject object = new JSONObject();
            object.put("camera_id", cameraId);
            object.put("width", width);
            object.put("height", height);
            object.put("fps", fps);
            object.put("duration_seconds", durationSeconds);
            object.put("bitrate_bps", bitrate);
            object.put(
                    "exposure_mode",
                    exposureUs > 0 ? "manual" : "auto"
            );
            object.put(
                    "requested_exposure_ns",
                    exposureUs > 0 ? exposureUs * 1000L : JSONObject.NULL
            );
            object.put(
                    "requested_iso",
                    exposureUs > 0 ? iso : JSONObject.NULL
            );
            object.put("video_stabilization_requested", "OFF");
            object.put("optical_stabilization_requested", "OFF");
            object.put(
                    "preview_surface_enabled",
                    enablePreview
            );
            object.put(
                    "recording_surface_enabled",
                    ENABLE_RECORDING_SURFACE
            );
            object.put("optical_trigger_enabled", opticalTrigger);
            object.put("optical_trigger_score", opticalTriggerScore);
            object.put(
                    "optical_trigger_hot_pixels",
                    opticalTriggerHotPixels
            );
            object.put("optical_stop_tail_ms", opticalStopTailMs);
            object.put("retained_run_output", retainRunOutput);
            object.put("recording_backend", "MediaRecorder");
            object.put("media_recorder_capture_rate_fps", fps);
            object.put("audio_recorded", false);
            return object;
        }
    }

    private static final class CaptureMetadata {
        final long frameNumber;
        final long callbackElapsedRealtimeNs;
        final Long sensorTimestampNs;
        final Long exposureNs;
        final Long frameDurationNs;
        final Long rollingShutterSkewNs;
        final Integer sensitivityIso;
        final Float focusDistanceDiopters;
        final Integer aeMode;
        final Integer videoStabilizationMode;
        final Integer opticalStabilizationMode;

        CaptureMetadata(
                long frameNumber,
                long callbackElapsedRealtimeNs,
                TotalCaptureResult result
        ) {
            this.frameNumber = frameNumber;
            this.callbackElapsedRealtimeNs = callbackElapsedRealtimeNs;
            sensorTimestampNs = result.get(CaptureResult.SENSOR_TIMESTAMP);
            exposureNs = result.get(CaptureResult.SENSOR_EXPOSURE_TIME);
            frameDurationNs = result.get(
                    CaptureResult.SENSOR_FRAME_DURATION
            );
            rollingShutterSkewNs = result.get(
                    CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW
            );
            sensitivityIso = result.get(
                    CaptureResult.SENSOR_SENSITIVITY
            );
            focusDistanceDiopters = result.get(
                    CaptureResult.LENS_FOCUS_DISTANCE
            );
            aeMode = result.get(CaptureResult.CONTROL_AE_MODE);
            videoStabilizationMode = result.get(
                    CaptureResult.CONTROL_VIDEO_STABILIZATION_MODE
            );
            opticalStabilizationMode = result.get(
                    CaptureResult.LENS_OPTICAL_STABILIZATION_MODE
            );
        }

        JSONObject toJson() throws Exception {
            JSONObject object = new JSONObject();
            object.put("frame_number", frameNumber);
            object.put(
                    "callback_elapsed_realtime_ns",
                    callbackElapsedRealtimeNs
            );
            putNullable(object, "sensor_timestamp_ns", sensorTimestampNs);
            putNullable(object, "exposure_time_ns", exposureNs);
            putNullable(object, "frame_duration_ns", frameDurationNs);
            putNullable(
                    object,
                    "rolling_shutter_skew_ns",
                    rollingShutterSkewNs
            );
            putNullable(object, "sensitivity_iso", sensitivityIso);
            putNullable(
                    object,
                    "focus_distance_diopters",
                    focusDistanceDiopters
            );
            putNullable(object, "ae_mode", aeMode);
            putNullable(
                    object,
                    "video_stabilization_mode",
                    videoStabilizationMode
            );
            putNullable(
                    object,
                    "optical_stabilization_mode",
                    opticalStabilizationMode
            );
            return object;
        }
    }

    private static final class EncodedSample {
        final long presentationTimeUs;
        final int sizeBytes;
        final int flags;

        EncodedSample(
                long presentationTimeUs,
                int sizeBytes,
                int flags
        ) {
            this.presentationTimeUs = presentationTimeUs;
            this.sizeBytes = sizeBytes;
            this.flags = flags;
        }

        JSONObject toJson() throws Exception {
            JSONObject object = new JSONObject();
            object.put("presentation_time_us", presentationTimeUs);
            object.put("size_bytes", sizeBytes);
            object.put("flags", flags);
            return object;
        }
    }

    private final Activity activity;
    private final TextureView preview;
    private final Handler cameraHandler;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final Listener listener;
    private final AtomicBoolean active = new AtomicBoolean(false);
    private final AtomicBoolean stopping = new AtomicBoolean(false);
    private final List<CaptureMetadata> captureMetadata =
            Collections.synchronizedList(new ArrayList<>());
    private final List<EncodedSample> encodedSamples =
            Collections.synchronizedList(new ArrayList<>());

    private Config config;
    private CameraCharacteristics characteristics;
    private CameraDevice camera;
    private CameraConstrainedHighSpeedCaptureSession captureSession;
    private Surface previewSurface;
    private Surface encoderSurface;
    private MediaRecorder mediaRecorder;
    private volatile boolean mediaRecorderStarted;
    private volatile boolean opticalArmed;
    private volatile boolean cancelledBeforeRecording;
    private int orientationHintDeg;
    private long recordingStartElapsedNs;
    private long recordingStopElapsedNs;
    private long opticalStartDetectedElapsedNs;
    private long opticalStopDetectedElapsedNs;
    private int opticalStartScore;
    private int opticalStartHotPixels;
    private int opticalStartThreshold;
    private int opticalStartMatchedLeds;
    private int opticalStartRequiredRises;
    private int opticalStartCenterX;
    private int opticalStartCenterY;
    private long opticalMotionDetectedElapsedNs;
    private int opticalMotionChangedPixels;
    private int opticalStopScore;
    private int opticalStopHotPixels;
    private int opticalStopThreshold;
    private int opticalStopMatchedLeds;
    private int opticalStopRequiredRises;
    private int captureFailureCount;
    private File videoFile;
    private File videoTempFile;
    private File reportFile;
    private File captureSidecarFile;
    private File encoderSidecarFile;
    private String outputRelativeDirectory;

    HfrRecorder(
            Activity activity,
            TextureView preview,
            Handler cameraHandler,
            Listener listener
    ) {
        this.activity = activity;
        this.preview = preview;
        this.cameraHandler = cameraHandler;
        this.listener = listener;
    }

    boolean isActive() {
        return active.get();
    }

    void start(Config requestedConfig) {
        if (!active.compareAndSet(false, true)) {
            return;
        }
        stopping.set(false);
        config = requestedConfig;
        captureMetadata.clear();
        encodedSamples.clear();
        captureFailureCount = 0;
        mediaRecorderStarted = false;
        opticalArmed = false;
        cancelledBeforeRecording = false;
        recordingStartElapsedNs = 0;
        recordingStopElapsedNs = 0;
        opticalStartDetectedElapsedNs = 0;
        opticalStopDetectedElapsedNs = 0;
        opticalStartScore = 0;
        opticalStartHotPixels = 0;
        opticalStartThreshold = 0;
        opticalStartMatchedLeds = 0;
        opticalStartRequiredRises = 0;
        opticalStartCenterX = -1;
        opticalStartCenterY = -1;
        opticalMotionDetectedElapsedNs = 0;
        opticalMotionChangedPixels = 0;
        opticalStopScore = 0;
        opticalStopHotPixels = 0;
        opticalStopThreshold = 0;
        opticalStopMatchedLeds = 0;
        opticalStopRequiredRises = 0;
        prepareOutputFiles();
        listener.onStatus(
                String.format(
                        "Preparing %dx%d @ %d fps...",
                        config.width,
                        config.height,
                        config.fps
                )
        );
        cameraHandler.post(() -> {
            try {
                validateCameraConfiguration();
                prepareRecorder();
                openCamera();
            } catch (Exception exception) {
                fail("unable to start recording", exception);
            }
        });
    }

    void stop() {
        if (!active.get() || !stopping.compareAndSet(false, true)) {
            return;
        }
        mainHandler.removeCallbacksAndMessages(this);
        cameraHandler.post(() -> {
            recordingStopElapsedNs = SystemClock.elapsedRealtimeNanos();
            closeCameraPipeline();
            finalizeAsync(null);
        });
    }

    void cancelArmed() {
        if (!active.get()
                || mediaRecorderStarted
                || !stopping.compareAndSet(false, true)) {
            return;
        }
        cancelledBeforeRecording = true;
        mainHandler.removeCallbacksAndMessages(this);
        cameraHandler.post(() -> {
            recordingStopElapsedNs = SystemClock.elapsedRealtimeNanos();
            closeCameraPipeline();
            finalizeAsync(null);
        });
    }

    void triggerRecording(
            long detectedElapsedNs,
            int score,
            int hotPixels,
            int threshold,
            int matchedLeds,
            int requiredRises,
            int centerX,
            int centerY
    ) {
        if (!active.get() || stopping.get()) {
            return;
        }
        cameraHandler.post(() -> {
            if (!opticalArmed || mediaRecorderStarted || stopping.get()) {
                return;
            }
            opticalStartDetectedElapsedNs = detectedElapsedNs;
            opticalStartScore = score;
            opticalStartHotPixels = hotPixels;
            opticalStartThreshold = threshold;
            opticalStartMatchedLeds = matchedLeds;
            opticalStartRequiredRises = requiredRises;
            opticalStartCenterX = centerX;
            opticalStartCenterY = centerY;
            opticalArmed = false;
            try {
                /*
                 * setRepeatingBurst() replaces the armed preview request.
                 * Do not flush this constrained high-speed session here:
                 * Pixel 8's camera HAL can tear down the pipeline and report
                 * CAMERA_ERROR/Broken pipe when abortCaptures() is followed
                 * immediately by a new high-speed burst.
                 */
                startEncodedRecording();
            } catch (Exception exception) {
                fail("unable to start optically triggered recording", exception);
            }
        });
    }

    void noteMotionGate(
            long detectedElapsedNs,
            int changedPixels
    ) {
        if (!active.get()
                || stopping.get()
                || !mediaRecorderStarted
                || opticalMotionDetectedElapsedNs != 0L) {
            return;
        }
        opticalMotionDetectedElapsedNs = detectedElapsedNs;
        opticalMotionChangedPixels = changedPixels;
    }

    void triggerStop(
            long detectedElapsedNs,
            int tailMs,
            int score,
            int hotPixels,
            int threshold,
            int matchedLeds,
            int requiredRises
    ) {
        if (!active.get()
                || stopping.get()
                || !mediaRecorderStarted
                || opticalStopDetectedElapsedNs != 0L) {
            return;
        }
        opticalStopDetectedElapsedNs = detectedElapsedNs;
        opticalStopScore = score;
        opticalStopHotPixels = hotPixels;
        opticalStopThreshold = threshold;
        opticalStopMatchedLeds = matchedLeds;
        opticalStopRequiredRises = requiredRises;
        mainHandler.postAtTime(
                this::stop,
                this,
                SystemClock.uptimeMillis() + Math.max(0, tailMs)
        );
    }

    void close() {
        if (active.get()) {
            if (mediaRecorderStarted) {
                stop();
            } else {
                cancelArmed();
            }
        } else {
            closeCameraPipeline();
        }
    }

    private void prepareOutputFiles() {
        File root = activity.getFilesDir();
        outputRelativeDirectory = "";
        if (config.retainRunOutput) {
            if (!config.nonce.matches("[A-Za-z0-9._-]+")) {
                throw new IllegalArgumentException(
                        "retained record_nonce contains unsafe characters"
                );
            }
            File retainedRoot = new File(root, "manual_runs");
            root = new File(retainedRoot, config.nonce);
            if (root.exists() || !root.mkdirs()) {
                throw new IllegalStateException(
                        "unable to create retained run directory: " + root
                );
            }
            outputRelativeDirectory = "manual_runs/" + config.nonce;
        }
        videoFile = new File(root, VIDEO_FILENAME);
        videoTempFile = new File(root, VIDEO_FILENAME + ".tmp");
        reportFile = new File(root, REPORT_FILENAME);
        captureSidecarFile = new File(
                root,
                CAPTURE_SIDECAR_FILENAME
        );
        encoderSidecarFile = new File(
                root,
                ENCODER_SIDECAR_FILENAME
        );
        for (File file : Arrays.asList(
                videoFile,
                videoTempFile,
                reportFile,
                new File(root, REPORT_FILENAME + ".tmp"),
                captureSidecarFile,
                new File(root, CAPTURE_SIDECAR_FILENAME + ".tmp"),
                encoderSidecarFile,
                new File(root, ENCODER_SIDECAR_FILENAME + ".tmp")
        )) {
            if (file.exists() && !file.delete()) {
                throw new IllegalStateException(
                        "unable to remove prior output: " + file
                );
            }
        }
    }

    private void validateCameraConfiguration() throws Exception {
        CameraManager manager = (CameraManager) activity.getSystemService(
                Context.CAMERA_SERVICE
        );
        characteristics = manager.getCameraCharacteristics(config.cameraId);
        Boolean supportsHighSpeed = hasCapability(
                characteristics,
                CameraCharacteristics
                        .REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO
        );
        if (!supportsHighSpeed) {
            throw new IllegalArgumentException(
                    "camera does not support constrained high-speed video"
            );
        }
        StreamConfigurationMap map = characteristics.get(
                CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP
        );
        if (map == null) {
            throw new IllegalArgumentException(
                    "camera has no stream configuration map"
            );
        }
        Size size = new Size(config.width, config.height);
        boolean sizeFound = Arrays.asList(
                map.getHighSpeedVideoSizes()
        ).contains(size);
        if (!sizeFound) {
            throw new IllegalArgumentException(
                    "camera does not expose high-speed size " + size
            );
        }
        boolean fixedRateFound = false;
        for (Range<Integer> range
                : map.getHighSpeedVideoFpsRangesFor(size)) {
            if (range.getLower() == config.fps
                    && range.getUpper() == config.fps) {
                fixedRateFound = true;
                break;
            }
        }
        if (!fixedRateFound) {
            throw new IllegalArgumentException(
                    "camera does not expose fixed " + config.fps
                            + " fps at " + size
            );
        }
        Range<Long> exposureRange = characteristics.get(
                CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE
        );
        Range<Integer> isoRange = characteristics.get(
                CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE
        );
        if (config.exposureUs > 0) {
            long exposureNs = config.exposureUs * 1000L;
            if (exposureRange == null
                    || !exposureRange.contains(exposureNs)) {
                throw new IllegalArgumentException(
                        "requested exposure is outside camera range"
                );
            }
            if (isoRange == null || !isoRange.contains(config.iso)) {
                throw new IllegalArgumentException(
                        "requested ISO is outside camera range"
                );
            }
        }
        orientationHintDeg = calculateOrientationHint(characteristics);
    }

    private static boolean hasCapability(
            CameraCharacteristics cameraCharacteristics,
            int target
    ) {
        int[] capabilities = cameraCharacteristics.get(
                CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES
        );
        if (capabilities == null) {
            return false;
        }
        for (int capability : capabilities) {
            if (capability == target) {
                return true;
            }
        }
        return false;
    }

    private int calculateOrientationHint(
            CameraCharacteristics cameraCharacteristics
    ) {
        Integer sensorOrientation = cameraCharacteristics.get(
                CameraCharacteristics.SENSOR_ORIENTATION
        );
        if (sensorOrientation == null || activity.getDisplay() == null) {
            return 0;
        }
        int deviceDegrees;
        switch (activity.getDisplay().getRotation()) {
            case Surface.ROTATION_90:
                deviceDegrees = 90;
                break;
            case Surface.ROTATION_180:
                deviceDegrees = 180;
                break;
            case Surface.ROTATION_270:
                deviceDegrees = 270;
                break;
            default:
                deviceDegrees = 0;
                break;
        }
        Integer facing = cameraCharacteristics.get(
                CameraCharacteristics.LENS_FACING
        );
        if (facing != null
                && facing == CameraCharacteristics.LENS_FACING_FRONT) {
            return (sensorOrientation + deviceDegrees) % 360;
        }
        return (sensorOrientation - deviceDegrees + 360) % 360;
    }

    private void prepareRecorder() throws Exception {
        mediaRecorder = new MediaRecorder(activity);
        mediaRecorder.setVideoSource(MediaRecorder.VideoSource.SURFACE);
        mediaRecorder.setOutputFormat(
                MediaRecorder.OutputFormat.MPEG_4
        );
        mediaRecorder.setOutputFile(videoTempFile.getAbsolutePath());
        mediaRecorder.setVideoEncodingBitRate(config.bitrate);
        mediaRecorder.setVideoFrameRate(config.fps);
        mediaRecorder.setCaptureRate((double) config.fps);
        mediaRecorder.setVideoSize(config.width, config.height);
        mediaRecorder.setVideoEncoder(MediaRecorder.VideoEncoder.H264);
        mediaRecorder.setOrientationHint(orientationHintDeg);
        mediaRecorder.prepare();
        encoderSurface = mediaRecorder.getSurface();
    }

    @SuppressLint("MissingPermission")
    private void openCamera() throws CameraAccessException {
        CameraManager manager = (CameraManager) activity.getSystemService(
                Context.CAMERA_SERVICE
        );
        manager.openCamera(
                config.cameraId,
                new CameraDevice.StateCallback() {
                    @Override
                    public void onOpened(CameraDevice openedCamera) {
                        camera = openedCamera;
                        createCaptureSession();
                    }

                    @Override
                    public void onDisconnected(CameraDevice disconnected) {
                        disconnected.close();
                        fail("camera disconnected", null);
                    }

                    @Override
                    public void onError(
                            CameraDevice failedCamera,
                            int error
                    ) {
                        failedCamera.close();
                        fail("camera error " + error, null);
                    }
                },
                cameraHandler
        );
    }

    private void createCaptureSession() {
        try {
            List<Surface> outputs = new ArrayList<>();
            if (config.enablePreview) {
                SurfaceTexture texture = preview.getSurfaceTexture();
                if (texture == null) {
                    throw new IllegalStateException(
                            "preview SurfaceTexture disappeared"
                    );
                }
                texture.setDefaultBufferSize(config.width, config.height);
                previewSurface = new Surface(texture);
                outputs.add(previewSurface);
            }
            if (ENABLE_RECORDING_SURFACE) {
                outputs.add(encoderSurface);
            }
            List<OutputConfiguration> outputConfigurations =
                    new ArrayList<>();
            for (Surface output : outputs) {
                outputConfigurations.add(
                        new OutputConfiguration(output)
                );
            }
            SessionConfiguration sessionConfiguration =
                    new SessionConfiguration(
                            SessionConfiguration.SESSION_HIGH_SPEED,
                            outputConfigurations,
                            command -> cameraHandler.post(command),
                            new CameraCaptureSession.StateCallback() {
                        @Override
                        public void onConfigured(
                                CameraCaptureSession configuredSession
                        ) {
                            if (!(configuredSession
                                    instanceof
                                    CameraConstrainedHighSpeedCaptureSession)) {
                                fail(
                                        "configured session is not high-speed",
                                        null
                                );
                                return;
                            }
                            captureSession =
                                    (CameraConstrainedHighSpeedCaptureSession)
                                            configuredSession;
                            startRepeatingBurst();
                        }

                        @Override
                        public void onConfigureFailed(
                                CameraCaptureSession failedSession
                        ) {
                            fail(
                                    "high-speed session configuration failed",
                                    null
                                );
                        }
                    }
            );
            camera.createCaptureSession(sessionConfiguration);
        } catch (Exception exception) {
            fail("unable to create high-speed session", exception);
        }
    }

    private void startRepeatingBurst() {
        try {
            if (config.opticalTrigger) {
                submitRepeatingBurst(false);
                opticalArmed = true;
                listener.onArmed();
                listener.onStatus(
                        "ARMED: waiting for F413 LED token"
                );
            } else {
                startEncodedRecording();
            }
        } catch (Exception exception) {
            fail("unable to start high-speed burst", exception);
        }
    }

    private void startEncodedRecording() throws Exception {
        captureMetadata.clear();
        captureFailureCount = 0;
        submitRepeatingBurst(true);
        if (ENABLE_RECORDING_SURFACE) {
            mediaRecorder.start();
            mediaRecorderStarted = true;
        }
        recordingStartElapsedNs = SystemClock.elapsedRealtimeNanos();
        listener.onRecordingStarted();
        listener.onStatus(
                String.format(
                        "Recording %dx%d @ %d fps for up to %d s",
                        config.width,
                        config.height,
                        config.fps,
                        config.durationSeconds
                )
        );
        mainHandler.postAtTime(
                this::stop,
                this,
                SystemClock.uptimeMillis()
                        + config.durationSeconds * 1000L
        );
    }

    private void submitRepeatingBurst(boolean includeEncoder)
            throws Exception {
        CaptureRequest.Builder request = camera.createCaptureRequest(
                CameraDevice.TEMPLATE_RECORD
        );
        if (previewSurface != null) {
            request.addTarget(previewSurface);
        }
        if (includeEncoder && ENABLE_RECORDING_SURFACE) {
            request.addTarget(encoderSurface);
        }
        request.set(
                CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE,
                new Range<>(config.fps, config.fps)
        );
        request.set(
                CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,
                CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF
        );
        request.set(
                CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE,
                CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF
        );
        request.set(
                CaptureRequest.CONTROL_AF_MODE,
                CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO
        );
        request.set(
                CaptureRequest.CONTROL_AWB_MODE,
                CaptureRequest.CONTROL_AWB_MODE_AUTO
        );
        if (config.exposureUs > 0) {
            request.set(
                    CaptureRequest.CONTROL_AE_MODE,
                    CaptureRequest.CONTROL_AE_MODE_OFF
            );
            request.set(
                    CaptureRequest.SENSOR_EXPOSURE_TIME,
                    config.exposureUs * 1000L
            );
            request.set(
                    CaptureRequest.SENSOR_FRAME_DURATION,
                    1_000_000_000L / config.fps
            );
            request.set(
                    CaptureRequest.SENSOR_SENSITIVITY,
                    config.iso
            );
        } else {
            request.set(
                    CaptureRequest.CONTROL_AE_MODE,
                    CaptureRequest.CONTROL_AE_MODE_ON
            );
        }
        List<CaptureRequest> burst =
                captureSession.createHighSpeedRequestList(
                        request.build()
                );
        captureSession.setRepeatingBurst(
                burst,
                captureCallback,
                cameraHandler
        );
    }

    private final CameraCaptureSession.CaptureCallback captureCallback =
            new CameraCaptureSession.CaptureCallback() {
                @Override
                public void onCaptureCompleted(
                        CameraCaptureSession session,
                        CaptureRequest request,
                        TotalCaptureResult result
                ) {
                    if (mediaRecorderStarted) {
                        captureMetadata.add(
                                new CaptureMetadata(
                                        result.getFrameNumber(),
                                        SystemClock.elapsedRealtimeNanos(),
                                        result
                                )
                        );
                    }
                }

                @Override
                public void onCaptureFailed(
                        CameraCaptureSession session,
                        CaptureRequest request,
                        CaptureFailure failure
                ) {
                    captureFailureCount += 1;
                }
            };

    private void readEncodedSamples(File input) throws Exception {
        MediaExtractor extractor = new MediaExtractor();
        try {
            extractor.setDataSource(input.getAbsolutePath());
            int videoTrack = -1;
            for (int index = 0;
                 index < extractor.getTrackCount();
                 index += 1) {
                MediaFormat format = extractor.getTrackFormat(index);
                String mime = format.getString(MediaFormat.KEY_MIME);
                if (mime != null && mime.startsWith("video/")) {
                    videoTrack = index;
                    break;
                }
            }
            if (videoTrack < 0) {
                throw new IllegalStateException(
                        "MP4 contains no video track"
                );
            }
            extractor.selectTrack(videoTrack);
            while (extractor.getSampleTime() >= 0) {
                encodedSamples.add(
                        new EncodedSample(
                                extractor.getSampleTime(),
                                (int) extractor.getSampleSize(),
                                extractor.getSampleFlags()
                        )
                );
                if (!extractor.advance()) {
                    break;
                }
            }
        } finally {
            extractor.release();
        }
    }

    private void closeCameraPipeline() {
        opticalArmed = false;
        try {
            if (captureSession != null) {
                captureSession.stopRepeating();
                captureSession.abortCaptures();
            }
        } catch (Exception ignored) {
        }
        if (captureSession != null) {
            captureSession.close();
            captureSession = null;
        }
        if (camera != null) {
            camera.close();
            camera = null;
        }
        if (previewSurface != null) {
            previewSurface.release();
            previewSurface = null;
        }
    }

    private void fail(String message, Throwable throwable) {
        if (!active.get()) {
            return;
        }
        if (throwable != null) {
            message += ": " + throwable.getClass().getSimpleName()
                    + ": " + throwable.getMessage();
        }
        final String finalMessage = message;
        if (stopping.compareAndSet(false, true)) {
            mainHandler.removeCallbacksAndMessages(this);
            recordingStopElapsedNs = SystemClock.elapsedRealtimeNanos();
            closeCameraPipeline();
            finalizeAsync(finalMessage);
        }
    }

    private void finalizeAsync(String startError) {
        new Thread(() -> {
            String error = startError;
            boolean cancelled = cancelledBeforeRecording;
            try {
                if (mediaRecorderStarted && mediaRecorder != null) {
                    mediaRecorder.stop();
                }
            } catch (RuntimeException exception) {
                error = combineErrors(
                        error,
                        "MediaRecorder stop failed: "
                                + exception.getMessage()
                );
            } finally {
                mediaRecorderStarted = false;
                if (mediaRecorder != null) {
                    try {
                        mediaRecorder.reset();
                    } catch (RuntimeException ignored) {
                    }
                    try {
                        mediaRecorder.release();
                    } catch (RuntimeException ignored) {
                    }
                    mediaRecorder = null;
                }
                encoderSurface = null;
            }
            if (cancelled && videoTempFile.isFile()) {
                videoTempFile.delete();
            }
            if (!cancelled
                    && error == null
                    && (!videoTempFile.isFile()
                    || videoTempFile.length() == 0)) {
                error = "MediaRecorder produced no MP4 data";
            }
            if (!cancelled && error == null) {
                try {
                    readEncodedSamples(videoTempFile);
                } catch (Exception exception) {
                    error = "MP4 sample scan failed: "
                            + exception.getMessage();
                }
            }
            try {
                if (cancelled && config.retainRunOutput) {
                    removeCancelledRetainedOutput();
                } else {
                    if (!cancelled && error == null) {
                        moveReplace(videoTempFile, videoFile);
                    }
                    writeSidecars();
                    writeReport(error);
                }
            } catch (Exception exception) {
                error = combineErrors(
                        error,
                        "artifact write failed: " + exception.getMessage()
                );
                try {
                    writeReport(error);
                } catch (Exception ignored) {
                }
            }
            active.set(false);
            stopping.set(false);
            final String finalError = error;
            if (cancelled && finalError == null) {
                listener.onCancelled("Standby cancelled");
            } else if (finalError == null) {
                listener.onFinished(
                        "Complete: " + encodedSamples.size()
                                + " encoded samples"
                );
            } else {
                listener.onError(finalError);
            }
        }, "nightfall-hfr-finalize").start();
    }

    private void removeCancelledRetainedOutput() throws IOException {
        File directory = reportFile.getParentFile();
        for (File file : Arrays.asList(
                videoFile,
                videoTempFile,
                reportFile,
                new File(directory, REPORT_FILENAME + ".tmp"),
                captureSidecarFile,
                new File(directory, CAPTURE_SIDECAR_FILENAME + ".tmp"),
                encoderSidecarFile,
                new File(directory, ENCODER_SIDECAR_FILENAME + ".tmp")
        )) {
            if (file.exists() && !file.delete()) {
                throw new IOException(
                        "unable to remove cancelled output: " + file
                );
            }
        }
        if (directory.exists() && !directory.delete()) {
            throw new IOException(
                    "unable to remove cancelled run directory: " + directory
            );
        }
    }

    private static String combineErrors(
            String first,
            String second
    ) {
        return first == null ? second : first + "; " + second;
    }

    private void writeSidecars() throws Exception {
        writeJsonLinesAtomically(
                captureSidecarFile,
                captureMetadataToJson()
        );
        writeJsonLinesAtomically(
                encoderSidecarFile,
                encodedSamplesToJson()
        );
    }

    private List<JSONObject> captureMetadataToJson() throws Exception {
        List<CaptureMetadata> snapshot;
        synchronized (captureMetadata) {
            snapshot = new ArrayList<>(captureMetadata);
        }
        List<JSONObject> output = new ArrayList<>();
        for (CaptureMetadata item : snapshot) {
            output.add(item.toJson());
        }
        return output;
    }

    private List<JSONObject> encodedSamplesToJson() throws Exception {
        List<EncodedSample> snapshot;
        synchronized (encodedSamples) {
            snapshot = new ArrayList<>(encodedSamples);
        }
        List<JSONObject> output = new ArrayList<>();
        for (EncodedSample item : snapshot) {
            output.add(item.toJson());
        }
        return output;
    }

    private static void writeJsonLinesAtomically(
            File output,
            List<JSONObject> rows
    ) throws Exception {
        File temporary = new File(
                output.getParentFile(),
                output.getName() + ".tmp"
        );
        try (BufferedWriter writer = new BufferedWriter(
                new OutputStreamWriter(
                        new FileOutputStream(temporary),
                        StandardCharsets.UTF_8
                )
        )) {
            for (JSONObject row : rows) {
                writer.write(row.toString());
                writer.newLine();
            }
        }
        moveReplace(temporary, output);
    }

    private void writeReport(String error) throws Exception {
        JSONObject report = buildReport(error);
        File temporary = new File(
                reportFile.getParentFile(),
                REPORT_FILENAME + ".tmp"
        );
        try (FileOutputStream stream = new FileOutputStream(temporary)) {
            stream.write(
                    (report.toString(2) + "\n").getBytes(
                            StandardCharsets.UTF_8
                    )
            );
            stream.getFD().sync();
        }
        moveReplace(temporary, reportFile);
    }

    private JSONObject buildReport(String error) throws Exception {
        JSONObject report = new JSONObject();
        report.put("schema", "nightfall_android_hfr_recording_v1");
        report.put("generated_at_utc", Instant.now().toString());
        report.put("record_nonce", config.nonce);
        report.put(
                "status",
                cancelledBeforeRecording
                        ? "cancelled"
                        : (error == null ? "complete" : "error")
        );
        report.put("error", error == null ? JSONObject.NULL : error);
        report.put("device", buildDevice());
        report.put("config", config.toJson());
        report.put("orientation_hint_deg", orientationHintDeg);
        report.put(
                "recording_elapsed_s",
                recordingStartElapsedNs > 0
                        && recordingStopElapsedNs > recordingStartElapsedNs
                        ? (
                        recordingStopElapsedNs
                                - recordingStartElapsedNs
                ) / 1_000_000_000.0
                        : JSONObject.NULL
        );
        JSONObject opticalTrigger = new JSONObject();
        opticalTrigger.put("enabled", config.opticalTrigger);
        opticalTrigger.put(
                "start_detected_elapsed_realtime_ns",
                opticalStartDetectedElapsedNs > 0
                        ? opticalStartDetectedElapsedNs
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "recording_start_elapsed_realtime_ns",
                recordingStartElapsedNs > 0
                        ? recordingStartElapsedNs
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "start_detection_to_recording_ms",
                opticalStartDetectedElapsedNs > 0
                        && recordingStartElapsedNs
                        >= opticalStartDetectedElapsedNs
                        ? (recordingStartElapsedNs
                        - opticalStartDetectedElapsedNs) / 1_000_000.0
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "start_score",
                opticalStartScore > 0
                        ? opticalStartScore
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "start_hot_pixels",
                opticalStartHotPixels > 0
                        ? opticalStartHotPixels
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "start_threshold",
                opticalStartThreshold > 0
                        ? opticalStartThreshold
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "start_matched_leds",
                opticalStartMatchedLeds > 0
                        ? opticalStartMatchedLeds
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "start_required_rises",
                opticalStartRequiredRises > 0
                        ? opticalStartRequiredRises
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "start_center_x",
                opticalStartCenterX >= 0
                        ? opticalStartCenterX
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "start_center_y",
                opticalStartCenterY >= 0
                        ? opticalStartCenterY
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "motion_detected_elapsed_realtime_ns",
                opticalMotionDetectedElapsedNs > 0
                        ? opticalMotionDetectedElapsedNs
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "motion_displacement_px",
                JSONObject.NULL
        );
        opticalTrigger.put(
                "motion_target_pixels",
                JSONObject.NULL
        );
        opticalTrigger.put(
                "motion_changed_pixels",
                opticalMotionDetectedElapsedNs > 0
                        ? opticalMotionChangedPixels
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "stop_detected_elapsed_realtime_ns",
                opticalStopDetectedElapsedNs > 0
                        ? opticalStopDetectedElapsedNs
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "recording_stop_elapsed_realtime_ns",
                recordingStopElapsedNs > 0
                        ? recordingStopElapsedNs
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "stop_score",
                opticalStopScore > 0
                        ? opticalStopScore
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "stop_hot_pixels",
                opticalStopHotPixels > 0
                        ? opticalStopHotPixels
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "stop_threshold",
                opticalStopThreshold > 0
                        ? opticalStopThreshold
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "stop_matched_leds",
                opticalStopMatchedLeds > 0
                        ? opticalStopMatchedLeds
                        : JSONObject.NULL
        );
        opticalTrigger.put(
                "stop_required_rises",
                opticalStopRequiredRises > 0
                        ? opticalStopRequiredRises
                        : JSONObject.NULL
        );
        report.put("optical_trigger", opticalTrigger);
        report.put(
                "capture_results",
                buildCaptureSummary()
        );
        report.put("encoded_video", buildEncoderSummary(error));
        JSONObject outputs = new JSONObject();
        outputs.put("directory", outputRelativeDirectory);
        outputs.put("video", VIDEO_FILENAME);
        outputs.put(
                "capture_results_jsonl",
                CAPTURE_SIDECAR_FILENAME
        );
        outputs.put(
                "encoder_samples_jsonl",
                ENCODER_SIDECAR_FILENAME
        );
        report.put("outputs", outputs);
        report.put(
                "measurement_note",
                "A successful report verifies Camera2 session and encoder "
                        + "operation. Host ffprobe/content QA is still "
                        + "required before measurement use."
        );
        return report;
    }

    private JSONObject buildDevice() throws Exception {
        JSONObject device = new JSONObject();
        device.put("manufacturer", Build.MANUFACTURER);
        device.put("brand", Build.BRAND);
        device.put("model", Build.MODEL);
        device.put("device", Build.DEVICE);
        device.put("product", Build.PRODUCT);
        device.put("fingerprint", Build.FINGERPRINT);
        device.put("android_release", Build.VERSION.RELEASE);
        device.put("sdk_int", Build.VERSION.SDK_INT);
        return device;
    }

    private JSONObject buildCaptureSummary() throws Exception {
        List<CaptureMetadata> snapshot;
        synchronized (captureMetadata) {
            snapshot = new ArrayList<>(captureMetadata);
        }
        List<Long> sensorTimestamps = new ArrayList<>();
        List<Long> exposureTimes = new ArrayList<>();
        List<Long> frameDurations = new ArrayList<>();
        List<Long> rollingShutterSkews = new ArrayList<>();
        List<Long> sensitivities = new ArrayList<>();
        List<Long> videoStabilizationModes = new ArrayList<>();
        List<Long> opticalStabilizationModes = new ArrayList<>();
        for (CaptureMetadata item : snapshot) {
            addIfNotNull(sensorTimestamps, item.sensorTimestampNs);
            addIfNotNull(exposureTimes, item.exposureNs);
            addIfNotNull(frameDurations, item.frameDurationNs);
            addIfNotNull(
                    rollingShutterSkews,
                    item.rollingShutterSkewNs
            );
            if (item.sensitivityIso != null) {
                sensitivities.add(item.sensitivityIso.longValue());
            }
            if (item.videoStabilizationMode != null) {
                videoStabilizationModes.add(
                        item.videoStabilizationMode.longValue()
                );
            }
            if (item.opticalStabilizationMode != null) {
                opticalStabilizationModes.add(
                        item.opticalStabilizationMode.longValue()
                );
            }
        }
        TreeSet<Long> uniqueTimestamps = new TreeSet<>(sensorTimestamps);
        List<Long> unique = new ArrayList<>(uniqueTimestamps);
        List<Long> intervals = adjacentDifferences(unique);
        JSONObject summary = new JSONObject();
        summary.put("callback_count", snapshot.size());
        summary.put("capture_failure_count", captureFailureCount);
        summary.put("sensor_timestamp_count", sensorTimestamps.size());
        summary.put("unique_sensor_frame_count", unique.size());
        summary.put(
                "duplicate_sensor_timestamp_count",
                sensorTimestamps.size() - unique.size()
        );
        summary.put(
                "sensor_interval_ns",
                statistics(intervals)
        );
        summary.put(
                "measured_sensor_fps",
                measuredRate(intervals, 1_000_000_000.0)
        );
        summary.put("exposure_time_ns", statistics(exposureTimes));
        summary.put("frame_duration_ns", statistics(frameDurations));
        summary.put(
                "rolling_shutter_skew_ns",
                statistics(rollingShutterSkews)
        );
        summary.put("sensitivity_iso", statistics(sensitivities));
        summary.put(
                "video_stabilization_mode",
                statistics(videoStabilizationModes)
        );
        summary.put(
                "optical_stabilization_mode",
                statistics(opticalStabilizationModes)
        );
        if (!unique.isEmpty()) {
            summary.put("first_sensor_timestamp_ns", unique.get(0));
            summary.put(
                    "last_sensor_timestamp_ns",
                    unique.get(unique.size() - 1)
            );
        }
        return summary;
    }

    private JSONObject buildEncoderSummary(String error) throws Exception {
        List<EncodedSample> snapshot;
        synchronized (encodedSamples) {
            snapshot = new ArrayList<>(encodedSamples);
        }
        List<Long> presentationTimes = new ArrayList<>();
        int keyFrames = 0;
        boolean strictlyIncreasing = true;
        Long prior = null;
        for (EncodedSample item : snapshot) {
            presentationTimes.add(item.presentationTimeUs);
            if ((item.flags & MediaExtractor.SAMPLE_FLAG_SYNC) != 0) {
                keyFrames += 1;
            }
            if (prior != null && item.presentationTimeUs <= prior) {
                strictlyIncreasing = false;
            }
            prior = item.presentationTimeUs;
        }
        TreeSet<Long> uniqueSet = new TreeSet<>(presentationTimes);
        List<Long> unique = new ArrayList<>(uniqueSet);
        List<Long> intervals = adjacentDifferences(unique);
        JSONObject summary = new JSONObject();
        summary.put("codec", MediaFormat.MIMETYPE_VIDEO_AVC);
        summary.put("sample_count", snapshot.size());
        summary.put("key_frame_count", keyFrames);
        summary.put(
                "unique_presentation_timestamp_count",
                unique.size()
        );
        summary.put(
                "duplicate_presentation_timestamp_count",
                presentationTimes.size() - unique.size()
        );
        summary.put(
                "presentation_timestamps_strictly_increasing",
                strictlyIncreasing
        );
        summary.put("pts_interval_us", statistics(intervals));
        summary.put(
                "measured_encoded_fps",
                measuredRate(intervals, 1_000_000.0)
        );
        summary.put(
                "file_size_bytes",
                error == null && videoFile.isFile()
                        ? videoFile.length()
                        : 0
        );
        if (!presentationTimes.isEmpty()) {
            long first = presentationTimes.get(0);
            long last = presentationTimes.get(
                    presentationTimes.size() - 1
            );
            summary.put("first_presentation_time_us", first);
            summary.put("last_presentation_time_us", last);
            summary.put(
                    "duration_pts_s",
                    (last - first) / 1_000_000.0
            );
        }
        return summary;
    }

    private static void addIfNotNull(
            List<Long> output,
            Long value
    ) {
        if (value != null) {
            output.add(value);
        }
    }

    private static List<Long> adjacentDifferences(
            List<Long> sortedValues
    ) {
        List<Long> output = new ArrayList<>();
        for (int index = 1; index < sortedValues.size(); index += 1) {
            output.add(
                    sortedValues.get(index)
                            - sortedValues.get(index - 1)
            );
        }
        return output;
    }

    private static JSONObject statistics(
            List<Long> source
    ) throws Exception {
        JSONObject result = new JSONObject();
        if (source.isEmpty()) {
            result.put("count", 0);
            return result;
        }
        List<Long> values = new ArrayList<>(source);
        Collections.sort(values);
        result.put("count", values.size());
        result.put("min", values.get(0));
        result.put("median", percentile(values, 50.0));
        result.put("p95", percentile(values, 95.0));
        result.put("max", values.get(values.size() - 1));
        return result;
    }

    private static double percentile(
            List<Long> sorted,
            double percentile
    ) {
        if (sorted.size() == 1) {
            return sorted.get(0);
        }
        double position = (
                sorted.size() - 1
        ) * percentile / 100.0;
        int lower = (int) Math.floor(position);
        int upper = (int) Math.ceil(position);
        if (lower == upper) {
            return sorted.get(lower);
        }
        double fraction = position - lower;
        return sorted.get(lower) * (1.0 - fraction)
                + sorted.get(upper) * fraction;
    }

    private static Object measuredRate(
            List<Long> intervals,
            double timeUnitsPerSecond
    ) {
        if (intervals.isEmpty()) {
            return JSONObject.NULL;
        }
        List<Long> sorted = new ArrayList<>(intervals);
        Collections.sort(sorted);
        double median = percentile(sorted, 50.0);
        return median > 0
                ? timeUnitsPerSecond / median
                : JSONObject.NULL;
    }

    private static void putNullable(
            JSONObject object,
            String name,
            Object value
    ) throws Exception {
        object.put(name, value == null ? JSONObject.NULL : value);
    }

    private static void moveReplace(
            File source,
            File destination
    ) throws IOException {
        Files.move(
                source.toPath(),
                destination.toPath(),
                StandardCopyOption.REPLACE_EXISTING,
                StandardCopyOption.ATOMIC_MOVE
        );
    }
}
