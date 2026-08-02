package com.nightfall.hfrrecorder;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.SystemClock;
import android.view.Gravity;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.TextView;

public final class MainActivity extends Activity {
    private static final int CAMERA_PERMISSION_REQUEST = 2001;
    private static final String EXTRA_AUTO_RECORD = "auto_record";
    private static final String EXTRA_NONCE = "record_nonce";
    private static final String EXTRA_CAMERA_ID = "camera_id";
    private static final String EXTRA_WIDTH = "width";
    private static final String EXTRA_HEIGHT = "height";
    private static final String EXTRA_FPS = "fps";
    private static final String EXTRA_DURATION_SECONDS = "duration_seconds";
    private static final String EXTRA_BITRATE = "bitrate";
    private static final String EXTRA_EXPOSURE_US = "exposure_us";
    private static final String EXTRA_ISO = "iso";
    private static final String EXTRA_ENABLE_PREVIEW = "enable_preview";
    private static final String EXTRA_OPTICAL_TRIGGER = "optical_trigger";
    private static final String EXTRA_OPTICAL_TRIGGER_SCORE =
            "optical_trigger_score";
    private static final String EXTRA_OPTICAL_TRIGGER_HOT_PIXELS =
            "optical_trigger_hot_pixels";
    private static final String EXTRA_OPTICAL_STOP_TAIL_MS =
            "optical_stop_tail_ms";
    // HIL-only overrides let the repeat state machine be exercised quickly
    // without an LED token or any command to the mouse.
    private static final String EXTRA_MANUAL_DURATION_SECONDS =
            "manual_duration_seconds";
    private static final String EXTRA_MANUAL_OPTICAL_TRIGGER =
            "manual_optical_trigger";
    private static final int OPTICAL_SAMPLE_WIDTH = 480;
    private static final int OPTICAL_SAMPLE_HEIGHT = 270;
    private static final long OPTICAL_SAMPLE_INTERVAL_NS = 25_000_000L;
    private static final long OPTICAL_STATUS_INTERVAL_NS = 500_000_000L;
    private static final int MANUAL_WIDTH = 1920;
    private static final int MANUAL_HEIGHT = 1080;
    private static final int MANUAL_FPS = 240;
    private static final int MANUAL_DURATION_SECONDS = 60;
    private static final int MANUAL_BITRATE = 72_000_000;
    private static final int MANUAL_EXPOSURE_US = 1000;
    private static final int MANUAL_ISO = 800;
    private static final int MANUAL_OPTICAL_TRIGGER_SCORE = 180;
    private static final int MANUAL_OPTICAL_TRIGGER_HOT_PIXELS = 2;
    private static final int MANUAL_OPTICAL_STOP_TAIL_MS = 900;
    private static final long MANUAL_REARM_DELAY_MS = 2000L;
    private static final long MINIMUM_FREE_STORAGE_BYTES =
            1024L * 1024L * 1024L;

    private TextureView preview;
    private TextView status;
    private Button startButton;
    private Button stopButton;
    private HandlerThread cameraThread;
    private Handler cameraHandler;
    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private HfrRecorder recorder;
    private boolean autoPending;
    private HfrRecorder.Config activeConfig;
    private OpticalTriggerDetector opticalDetector;
    private MotionGateDetector motionGateDetector;
    private boolean opticalDetectionEnabled;
    private boolean opticalWaitingForStart;
    private boolean opticalWaitingForMotion;
    private int opticalStartCenterX = -1;
    private int opticalStartCenterY = -1;
    private long lastOpticalSampleNs;
    private long lastOpticalStatusNs;
    private int[] opticalPixels;
    private boolean manualContinuousStandby;
    private boolean manualStopRequested;
    private String manualSessionNonce;
    private int manualRunSequence;
    private int manualCompletedRuns;
    private final Runnable manualRearmRunnable = () -> {
        if (!manualContinuousStandby || isFinishing() || isDestroyed()) {
            return;
        }
        setStatus(
                String.format(
                        "%d本保存済み。次の走行を待機します...",
                        manualCompletedRuns
                ),
                true
        );
        startRecording(false);
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        startCameraThread();
        setContentView(createContentView());
        autoPending = getIntent().getBooleanExtra(EXTRA_AUTO_RECORD, false);
        preparePermissionAndMaybeStart();
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        setIntent(intent);
        autoPending = intent.getBooleanExtra(EXTRA_AUTO_RECORD, false);
        preparePermissionAndMaybeStart();
    }

    private View createContentView() {
        FrameLayout root = new FrameLayout(this);
        root.setBackgroundColor(Color.BLACK);

        preview = new TextureView(this);
        preview.setSurfaceTextureListener(
                new TextureView.SurfaceTextureListener() {
                    @Override
                    public void onSurfaceTextureAvailable(
                            SurfaceTexture surface,
                            int width,
                            int height
                    ) {
                        maybeStartAutomatically();
                    }

                    @Override
                    public void onSurfaceTextureSizeChanged(
                            SurfaceTexture surface,
                            int width,
                            int height
                    ) {
                    }

                    @Override
                    public boolean onSurfaceTextureDestroyed(
                            SurfaceTexture surface
                    ) {
                        return true;
                    }

                    @Override
                    public void onSurfaceTextureUpdated(
                            SurfaceTexture surface
                    ) {
                        processOpticalTriggerFrame();
                    }
                }
        );
        root.addView(
                preview,
                new FrameLayout.LayoutParams(
                        FrameLayout.LayoutParams.MATCH_PARENT,
                        FrameLayout.LayoutParams.MATCH_PARENT
                )
        );

        LinearLayout controls = new LinearLayout(this);
        controls.setOrientation(LinearLayout.HORIZONTAL);
        controls.setGravity(Gravity.CENTER_VERTICAL);
        controls.setPadding(dp(12), dp(8), dp(12), dp(8));
        controls.setBackgroundColor(0xcc000000);

        startButton = new Button(this);
        startButton.setText("連続撮影スタンバイ (240 fps)");
        startButton.setOnClickListener(
                view -> startManualContinuousStandby()
        );
        controls.addView(startButton);

        stopButton = new Button(this);
        stopButton.setText("待機をキャンセル");
        stopButton.setEnabled(false);
        stopButton.setOnClickListener(view -> stopFromPixel());
        controls.addView(stopButton);

        status = new TextView(this);
        status.setText("「連続撮影スタンバイ」を押してください");
        status.setTextColor(Color.WHITE);
        status.setTextSize(14.0f);
        status.setPadding(dp(12), 0, 0, 0);
        controls.addView(
                status,
                new LinearLayout.LayoutParams(
                        0,
                        LinearLayout.LayoutParams.WRAP_CONTENT,
                        1.0f
                )
        );

        FrameLayout.LayoutParams controlLayout = new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.WRAP_CONTENT,
                Gravity.BOTTOM
        );
        root.addView(controls, controlLayout);
        return root;
    }

    private int dp(int value) {
        return Math.round(
                value * getResources().getDisplayMetrics().density
        );
    }

    private void startCameraThread() {
        cameraThread = new HandlerThread("nightfall-hfr-camera");
        cameraThread.start();
        cameraHandler = new Handler(cameraThread.getLooper());
    }

    private void preparePermissionAndMaybeStart() {
        if (checkSelfPermission(Manifest.permission.CAMERA)
                == PackageManager.PERMISSION_GRANTED) {
            maybeStartAutomatically();
        } else {
            requestPermissions(
                    new String[]{Manifest.permission.CAMERA},
                    CAMERA_PERMISSION_REQUEST
            );
        }
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
            maybeStartAutomatically();
        } else {
            setStatus("Camera permission is required.", false);
        }
    }

    private void maybeStartAutomatically() {
        if (!autoPending
                || preview == null
                || !preview.isAvailable()
                || checkSelfPermission(Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
            return;
        }
        autoPending = false;
        startRecording(true);
    }

    private void startManualContinuousStandby() {
        if (manualContinuousStandby
                || (recorder != null && recorder.isActive())) {
            return;
        }
        manualContinuousStandby = true;
        manualStopRequested = false;
        manualSessionNonce = "manual-" + System.currentTimeMillis();
        manualRunSequence = 0;
        manualCompletedRuns = 0;
        startRecording(false);
    }

    private void stopFromPixel() {
        boolean stoppingContinuous = manualContinuousStandby;
        if (stoppingContinuous) {
            manualContinuousStandby = false;
            manualStopRequested = true;
            uiHandler.removeCallbacks(manualRearmRunnable);
        }
        if (recorder == null || !recorder.isActive()) {
            resetControlsAfterRun();
            setStatus(
                    stoppingContinuous
                            ? String.format(
                            "連続撮影を終了しました（%d本保存）",
                            manualCompletedRuns
                    )
                            : "撮影を停止しました",
                    true
            );
            return;
        }
        opticalDetectionEnabled = false;
        stopButton.setEnabled(false);
        if (opticalWaitingForStart) {
            setStatus("撮影スタンバイを終了しています...", true);
            recorder.cancelArmed();
        } else {
            setStatus(
                    stoppingContinuous
                            ? "連続撮影を終了し、動画を保存しています..."
                            : "録画を停止しています...",
                    true
            );
            recorder.stop();
        }
    }

    private void startRecording(boolean useIntentConfig) {
        if (recorder != null && recorder.isActive()) {
            return;
        }
        if (!preview.isAvailable()) {
            setStatus("Preview surface is not ready.", false);
            return;
        }
        HfrRecorder.Config config;
        try {
            config = useIntentConfig
                    ? readConfig(getIntent())
                    : createManualOpticalConfig();
        } catch (IllegalArgumentException exception) {
            setStatus(exception.getMessage(), false);
            return;
        }
        if (config.retainRunOutput
                && getFilesDir().getUsableSpace()
                < MINIMUM_FREE_STORAGE_BYTES) {
            manualContinuousStandby = false;
            resetControlsAfterRun();
            setStatus(
                    "空き容量が1 GiB未満のため連続撮影を開始できません",
                    false
            );
            return;
        }
        recorder = new HfrRecorder(
                this,
                preview,
                cameraHandler,
                new HfrRecorder.Listener() {
                    @Override
                    public void onStatus(String message) {
                        runOnUiThread(() -> setStatus(message, true));
                    }

                    @Override
                    public void onArmed() {
                        runOnUiThread(() -> {
                            opticalWaitingForStart = true;
                            opticalWaitingForMotion = false;
                            opticalStartCenterX = -1;
                            opticalStartCenterY = -1;
                            opticalDetectionEnabled = true;
                            lastOpticalSampleNs = 0L;
                            lastOpticalStatusNs = 0L;
                            if (opticalDetector != null) {
                                opticalDetector.reset();
                            }
                            startButton.setEnabled(false);
                            stopButton.setText(
                                    config.retainRunOutput
                                            ? "連続待機を終了"
                                            : "待機をキャンセル"
                            );
                            stopButton.setEnabled(true);
                        });
                    }

                    @Override
                    public void onRecordingStarted() {
                        runOnUiThread(() -> {
                            if (activeConfig != null
                                    && activeConfig.opticalTrigger) {
                                opticalWaitingForStart = false;
                                opticalWaitingForMotion = true;
                                opticalDetectionEnabled = true;
                                lastOpticalSampleNs = 0L;
                                lastOpticalStatusNs = 0L;
                                if (opticalDetector != null) {
                                    opticalDetector.setRequiredRises(4);
                                    opticalDetector.rearm();
                                }
                                motionGateDetector = new MotionGateDetector();
                                motionGateDetector.arm(
                                        opticalStartCenterX,
                                        opticalStartCenterY
                                );
                            }
                            startButton.setEnabled(false);
                            stopButton.setText(
                                    config.retainRunOutput
                                            ? "連続待機を終了"
                                            : "録画を停止"
                            );
                            stopButton.setEnabled(true);
                        });
                    }

                    @Override
                    public void onCancelled(String message) {
                        runOnUiThread(() -> {
                            resetControlsAfterRun();
                            setStatus(
                                    config.retainRunOutput
                                            ? String.format(
                                            "連続撮影を終了しました（%d本保存）",
                                            manualCompletedRuns
                                    )
                                            : "撮影スタンバイを終了しました",
                                    true
                            );
                        });
                    }

                    @Override
                    public void onFinished(String message) {
                        runOnUiThread(() -> {
                            if (config.retainRunOutput) {
                                manualCompletedRuns += 1;
                            }
                            if (config.retainRunOutput
                                    && manualContinuousStandby) {
                                prepareControlsForRearm();
                                setStatus(
                                        String.format(
                                                "%d本目を保存しました。2秒後に再待機します...",
                                                manualCompletedRuns
                                        ),
                                        true
                                );
                                uiHandler.postDelayed(
                                        manualRearmRunnable,
                                        MANUAL_REARM_DELAY_MS
                                );
                            } else {
                                resetControlsAfterRun();
                                setStatus(
                                        config.retainRunOutput
                                                && manualStopRequested
                                                ? String.format(
                                                "連続撮影を終了しました（%d本保存）",
                                                manualCompletedRuns
                                        )
                                                : message,
                                        true
                                );
                            }
                        });
                    }

                    @Override
                    public void onError(String message) {
                        runOnUiThread(() -> {
                            manualContinuousStandby = false;
                            uiHandler.removeCallbacks(manualRearmRunnable);
                            resetControlsAfterRun();
                            setStatus("ERROR: " + message, false);
                        });
                    }
                }
        );
        startButton.setEnabled(false);
        stopButton.setEnabled(false);
        if (config.retainRunOutput) {
            stopButton.setText("連続待機を終了");
        }
        activeConfig = config;
        if (config.opticalTrigger) {
            opticalDetector = new OpticalTriggerDetector(
                    config.opticalTriggerScore,
                    config.opticalTriggerHotPixels,
                    3
            );
            motionGateDetector = null;
        } else {
            opticalDetector = null;
            opticalDetectionEnabled = false;
        }
        recorder.start(config);
    }

    private HfrRecorder.Config createManualOpticalConfig() {
        manualRunSequence += 1;
        int durationSeconds = getIntent().getIntExtra(
                EXTRA_MANUAL_DURATION_SECONDS,
                MANUAL_DURATION_SECONDS
        );
        boolean opticalTrigger = getIntent().getBooleanExtra(
                EXTRA_MANUAL_OPTICAL_TRIGGER,
                true
        );
        HfrRecorder.Config config = new HfrRecorder.Config(
                String.format(
                        "%s-run%04d",
                        manualSessionNonce,
                        manualRunSequence
                ),
                "0",
                MANUAL_WIDTH,
                MANUAL_HEIGHT,
                MANUAL_FPS,
                durationSeconds,
                MANUAL_BITRATE,
                MANUAL_EXPOSURE_US,
                MANUAL_ISO,
                true,
                opticalTrigger,
                MANUAL_OPTICAL_TRIGGER_SCORE,
                MANUAL_OPTICAL_TRIGGER_HOT_PIXELS,
                MANUAL_OPTICAL_STOP_TAIL_MS,
                true
        );
        config.validate();
        return config;
    }

    private void resetControlsAfterRun() {
        opticalDetectionEnabled = false;
        opticalWaitingForStart = false;
        opticalWaitingForMotion = false;
        manualContinuousStandby = false;
        uiHandler.removeCallbacks(manualRearmRunnable);
        startButton.setEnabled(true);
        startButton.setText("連続撮影スタンバイ (240 fps)");
        stopButton.setText("待機をキャンセル");
        stopButton.setEnabled(false);
    }

    private void prepareControlsForRearm() {
        opticalDetectionEnabled = false;
        opticalWaitingForStart = false;
        opticalWaitingForMotion = false;
        startButton.setEnabled(false);
        stopButton.setText("連続待機を終了");
        stopButton.setEnabled(true);
    }

    private HfrRecorder.Config readConfig(Intent intent) {
        String nonce = intent.getStringExtra(EXTRA_NONCE);
        if (nonce == null || nonce.isBlank()) {
            nonce = "manual-" + System.currentTimeMillis();
        }
        HfrRecorder.Config config = new HfrRecorder.Config(
                nonce,
                intent.getStringExtra(EXTRA_CAMERA_ID) == null
                        ? "0"
                        : intent.getStringExtra(EXTRA_CAMERA_ID),
                intent.getIntExtra(EXTRA_WIDTH, 1920),
                intent.getIntExtra(EXTRA_HEIGHT, 1080),
                intent.getIntExtra(EXTRA_FPS, 120),
                intent.getIntExtra(EXTRA_DURATION_SECONDS, 5),
                intent.getIntExtra(EXTRA_BITRATE, 40_000_000),
                intent.getIntExtra(EXTRA_EXPOSURE_US, 0),
                intent.getIntExtra(EXTRA_ISO, 400),
                intent.getBooleanExtra(EXTRA_ENABLE_PREVIEW, true),
                intent.getBooleanExtra(EXTRA_OPTICAL_TRIGGER, false),
                intent.getIntExtra(EXTRA_OPTICAL_TRIGGER_SCORE, 180),
                intent.getIntExtra(
                        EXTRA_OPTICAL_TRIGGER_HOT_PIXELS,
                        2
                ),
                intent.getIntExtra(EXTRA_OPTICAL_STOP_TAIL_MS, 900),
                false
        );
        config.validate();
        return config;
    }

    private void setStatus(String message, boolean normal) {
        status.setText(message);
        status.setTextColor(normal ? Color.WHITE : 0xffff8080);
    }

    private void processOpticalTriggerFrame() {
        if (!opticalDetectionEnabled
                || opticalDetector == null
                || recorder == null
                || activeConfig == null) {
            return;
        }
        long nowNs = SystemClock.elapsedRealtimeNanos();
        if (nowNs - lastOpticalSampleNs < OPTICAL_SAMPLE_INTERVAL_NS) {
            return;
        }
        lastOpticalSampleNs = nowNs;
        Bitmap bitmap = preview.getBitmap(
                OPTICAL_SAMPLE_WIDTH,
                OPTICAL_SAMPLE_HEIGHT
        );
        if (bitmap == null) {
            return;
        }
        int pixelCount = OPTICAL_SAMPLE_WIDTH * OPTICAL_SAMPLE_HEIGHT;
        if (opticalPixels == null || opticalPixels.length != pixelCount) {
            opticalPixels = new int[pixelCount];
        }
        bitmap.getPixels(
                opticalPixels,
                0,
                OPTICAL_SAMPLE_WIDTH,
                0,
                0,
                OPTICAL_SAMPLE_WIDTH,
                OPTICAL_SAMPLE_HEIGHT
        );
        bitmap.recycle();

        OpticalTriggerDetector.Result result = opticalDetector.process(
                opticalPixels,
                OPTICAL_SAMPLE_WIDTH,
                OPTICAL_SAMPLE_HEIGHT,
                nowNs
        );
        if (!opticalWaitingForStart && opticalWaitingForMotion) {
            MotionGateDetector.Result motion = motionGateDetector.process(
                    opticalPixels,
                    OPTICAL_SAMPLE_WIDTH,
                    OPTICAL_SAMPLE_HEIGHT
            );
            if (motion.motionDetected) {
                opticalWaitingForMotion = false;
                opticalDetector.rearm();
                recorder.noteMotionGate(
                        nowNs,
                        motion.changedPixels
                );
                setStatus(
                        String.format(
                                "Motion detected (%d changed px); STOP 4-pulse armed",
                                motion.changedPixels
                        ),
                        true
                );
                return;
            }
            if (nowNs - lastOpticalStatusNs
                    >= OPTICAL_STATUS_INTERVAL_NS) {
                lastOpticalStatusNs = nowNs;
                setStatus(
                        String.format(
                                "REC MOVE: phase=%s baseline=%s changed=%d LED=%s",
                                motion.phase,
                                motion.baselineReady ? "yes" : "no",
                                motion.changedPixels,
                                result.phase
                        ),
                        true
                );
            }
            return;
        }
        if (result.triggered) {
            opticalDetectionEnabled = false;
            if (opticalWaitingForStart) {
                opticalStartCenterX = result.centerX;
                opticalStartCenterY = result.centerY;
                setStatus("LED START token detected; starting recorder...", true);
                recorder.triggerRecording(
                        nowNs,
                        result.score,
                        result.hotPixels,
                        result.threshold,
                        result.matchedLeds,
                        result.requiredRises,
                        result.centerX,
                        result.centerY
                );
            } else {
                setStatus("LED STOP token detected; saving tail...", true);
                recorder.triggerStop(
                        nowNs,
                        activeConfig.opticalStopTailMs,
                        result.score,
                        result.hotPixels,
                        result.threshold,
                        result.matchedLeds,
                        result.requiredRises
                );
            }
        } else if (nowNs - lastOpticalStatusNs
                >= OPTICAL_STATUS_INTERVAL_NS) {
            lastOpticalStatusNs = nowNs;
            setStatus(
                    String.format(
                            "%s LED: phase=%s score=%d/%d hot=%d matched=%d token=%d",
                            opticalWaitingForStart ? "ARMED" : "REC",
                            result.phase,
                            result.score,
                            result.threshold,
                            result.hotPixels,
                            result.matchedLeds,
                            result.requiredRises
                    ),
                    true
            );
        }
    }

    @Override
    protected void onDestroy() {
        manualContinuousStandby = false;
        uiHandler.removeCallbacks(manualRearmRunnable);
        opticalDetectionEnabled = false;
        if (recorder != null) {
            recorder.close();
        }
        if (cameraThread != null) {
            cameraThread.quitSafely();
        }
        super.onDestroy();
    }
}
