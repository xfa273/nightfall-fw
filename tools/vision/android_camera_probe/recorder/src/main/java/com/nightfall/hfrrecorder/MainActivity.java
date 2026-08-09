package com.nightfall.hfrrecorder;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
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
import android.util.Log;
import android.view.Gravity;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.TextView;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public final class MainActivity extends Activity {
    private static final String LOG_TAG = "HfrOptical";
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
    private TextView wifiStatus;
    private Button startButton;
    private Button finishRunButton;
    private Button stopButton;
    private Button wifiPairButton;
    private Button wifiDeleteButton;
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
    private WifiTransferServer wifiTransferServer;
    private boolean wifiCaptureSlotOwned;
    private volatile WifiTransferServer.CaptureControlState captureControlState =
            new WifiTransferServer.CaptureControlState(
                    "idle",
                    false,
                    false,
                    0,
                    null,
                    "連続撮影は停止中です"
            );
    private final Runnable manualRearmRunnable = () -> {
        if (!manualContinuousStandby || isFinishing() || isDestroyed()) {
            return;
        }
        String message = String.format(
                "%d本保存済み。次の走行を待機します...",
                manualCompletedRuns
        );
        setStatus(message, true);
        publishCaptureControlState("starting", false, message);
        startRecording(false);
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        startCameraThread();
        setContentView(createContentView());
        wifiTransferServer = new WifiTransferServer(
                this,
                snapshot -> runOnUiThread(
                        () -> updateWifiStatus(snapshot)
                ),
                new WifiTransferServer.CaptureControlHandler() {
                    @Override
                    public WifiTransferServer.CaptureControlResult
                    startContinuousStandby() {
                        return runRemoteCaptureControl(true);
                    }

                    @Override
                    public WifiTransferServer.CaptureControlResult
                    stopContinuousStandby() {
                        return runRemoteCaptureControl(false);
                    }

                    @Override
                    public WifiTransferServer.CaptureControlResult
                    finishCurrentRecording() {
                        return runRemoteFinishCurrentRecording();
                    }

                    @Override
                    public WifiTransferServer.CaptureControlState
                    captureControlState() {
                        return captureControlState;
                    }
                }
        );
        wifiTransferServer.start();
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

        LinearLayout wifiControls = new LinearLayout(this);
        wifiControls.setOrientation(LinearLayout.HORIZONTAL);
        wifiControls.setGravity(Gravity.CENTER_VERTICAL);
        wifiControls.setPadding(dp(12), dp(5), dp(12), dp(5));
        wifiControls.setBackgroundColor(0xcc000000);

        wifiPairButton = new Button(this);
        wifiPairButton.setText("ペアコード更新");
        wifiPairButton.setOnClickListener(view -> {
            if (wifiTransferServer != null) {
                wifiTransferServer.issuePairingCode();
            }
        });
        wifiControls.addView(wifiPairButton);

        wifiDeleteButton = new Button(this);
        wifiDeleteButton.setText("転送済みを削除");
        wifiDeleteButton.setOnClickListener(
                view -> confirmDeleteAcknowledgedRuns()
        );
        wifiControls.addView(wifiDeleteButton);

        wifiStatus = new TextView(this);
        wifiStatus.setText("Wi-Fi転送を初期化しています...");
        wifiStatus.setTextColor(Color.WHITE);
        wifiStatus.setTextSize(13.0f);
        wifiStatus.setPadding(dp(12), 0, 0, 0);
        wifiControls.addView(
                wifiStatus,
                new LinearLayout.LayoutParams(
                        0,
                        LinearLayout.LayoutParams.WRAP_CONTENT,
                        1.0f
                )
        );

        FrameLayout.LayoutParams wifiLayout = new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.WRAP_CONTENT,
                Gravity.TOP
        );
        root.addView(wifiControls, wifiLayout);

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

        finishRunButton = new Button(this);
        finishRunButton.setText("この撮影だけ終了");
        finishRunButton.setEnabled(false);
        finishRunButton.setOnClickListener(
                view -> finishCurrentRecording()
        );
        controls.addView(finishRunButton);

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

    private void updateWifiStatus(WifiTransferServer.Snapshot snapshot) {
        if (wifiStatus == null) {
            return;
        }
        String endpoint = snapshot.addresses.isEmpty()
                ? "Wi-Fi未接続"
                : snapshot.addresses.get(0)
                + ":"
                + WifiTransferProtocol.HTTP_PORT;
        String pairing = snapshot.pairingCode == null
                ? "ペア済み（再設定はコード更新）"
                : String.format(
                "ペアコード %s（残り%d分）",
                snapshot.pairingCode,
                Math.max(1L, (snapshot.pairingSecondsRemaining + 59L) / 60L)
        );
        String activity;
        if (snapshot.activeTransfers > 0) {
            activity = " / 転送中";
        } else if (snapshot.captureBusy) {
            activity = " / 撮影優先（転送停止中）";
        } else {
            activity = " / 転送可能";
        }
        String message = String.format(
                "Wi-Fi %s / %s / 保存%d・転送済み%d%s",
                endpoint,
                pairing,
                snapshot.savedRuns,
                snapshot.acknowledgedRuns,
                activity
        );
        if (snapshot.error != null) {
            message += " / ERROR: " + snapshot.error;
        }
        wifiStatus.setText(message);
        wifiStatus.setTextColor(
                snapshot.error == null ? Color.WHITE : 0xffff8080
        );
        wifiDeleteButton.setEnabled(
                snapshot.acknowledgedRuns > 0
                        && !snapshot.captureBusy
                        && snapshot.activeTransfers == 0
        );
    }

    private void confirmDeleteAcknowledgedRuns() {
        if (wifiTransferServer == null) {
            return;
        }
        WifiTransferServer.Snapshot snapshot = wifiTransferServer.snapshot();
        if (snapshot.acknowledgedRuns <= 0) {
            setStatus("Macで検証済みの動画はありません", true);
            return;
        }
        new AlertDialog.Builder(this)
                .setTitle("転送済み動画を削除")
                .setMessage(String.format(
                        "Macが受領・検証済みと通知した%d本をPixelから削除します。"
                                + " 未転送の動画は削除しません。",
                        snapshot.acknowledgedRuns
                ))
                .setNegativeButton("キャンセル", null)
                .setPositiveButton("削除", (dialog, which) -> {
                    wifiDeleteButton.setEnabled(false);
                    Thread worker = new Thread(() -> {
                        WifiTransferServer.DeleteResult result =
                                wifiTransferServer.deleteAcknowledgedRuns();
                        runOnUiThread(() -> {
                            if (result.error != null) {
                                setStatus(result.error, false);
                            } else {
                                setStatus(
                                        String.format(
                                                "転送済み%d本（%s）をPixelから削除しました",
                                                result.runs,
                                                formatBytes(result.bytes)
                                        ),
                                        true
                                );
                            }
                        });
                    }, "nightfall-hfr-delete");
                    worker.start();
                })
                .show();
    }

    private static String formatBytes(long bytes) {
        if (bytes >= 1024L * 1024L * 1024L) {
            return String.format(
                    "%.2f GiB",
                    bytes / (1024.0 * 1024.0 * 1024.0)
            );
        }
        return String.format("%.1f MiB", bytes / (1024.0 * 1024.0));
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

    private boolean startManualContinuousStandby() {
        if (manualContinuousStandby) {
            return true;
        }
        if (recorder != null && recorder.isActive()) {
            setStatus("別の撮影セッションが動作中です", false);
            return false;
        }
        if (wifiTransferServer != null
                && wifiTransferServer.isTransferActive()) {
            setStatus(
                    "Wi-Fi転送の完了後に撮影スタンバイを開始してください",
                    false
            );
            publishCaptureControlState(
                    "idle",
                    false,
                    "Wi-Fi転送の完了後に撮影スタンバイを開始してください"
            );
            return false;
        }
        manualContinuousStandby = true;
        manualStopRequested = false;
        manualSessionNonce = "manual-" + System.currentTimeMillis();
        manualRunSequence = 0;
        manualCompletedRuns = 0;
        publishCaptureControlState(
                "starting",
                false,
                "240 fps連続撮影スタンバイを開始しています"
        );
        startRecording(false);
        return manualContinuousStandby;
    }

    private WifiTransferServer.CaptureControlResult runRemoteCaptureControl(
            boolean start
    ) {
        if (Looper.myLooper() == Looper.getMainLooper()) {
            return applyRemoteCaptureControl(start);
        }
        AtomicReference<WifiTransferServer.CaptureControlResult> result =
                new AtomicReference<>();
        CountDownLatch completed = new CountDownLatch(1);
        uiHandler.post(() -> {
            try {
                result.set(applyRemoteCaptureControl(start));
            } finally {
                completed.countDown();
            }
        });
        try {
            if (!completed.await(5L, TimeUnit.SECONDS)) {
                return new WifiTransferServer.CaptureControlResult(
                        false,
                        "Pixel UI did not respond to capture control",
                        captureControlState
                );
            }
        } catch (InterruptedException exception) {
            Thread.currentThread().interrupt();
            return new WifiTransferServer.CaptureControlResult(
                    false,
                    "capture control was interrupted",
                    captureControlState
            );
        }
        return result.get();
    }

    private WifiTransferServer.CaptureControlResult applyRemoteCaptureControl(
            boolean start
    ) {
        if (isFinishing() || isDestroyed()) {
            return rejectedRemoteControl("HFR Recorder is closing");
        }
        if (start) {
            if (manualContinuousStandby) {
                return acceptedRemoteControl();
            }
            if (checkSelfPermission(Manifest.permission.CAMERA)
                    != PackageManager.PERMISSION_GRANTED) {
                return rejectedRemoteControl(
                        "Pixelでカメラ権限を許可してください"
                );
            }
            if (preview == null || !preview.isAvailable()) {
                return rejectedRemoteControl(
                        "カメラプレビューの準備ができていません"
                );
            }
            if (!startManualContinuousStandby()) {
                return rejectedRemoteControl(captureControlState.message);
            }
            return acceptedRemoteControl();
        }
        if (!manualContinuousStandby) {
            if (recorder != null && recorder.isActive()) {
                return rejectedRemoteControl(
                        "連続撮影以外のセッションが動作中です"
                );
            }
            return acceptedRemoteControl();
        }
        stopFromPixel();
        return acceptedRemoteControl();
    }

    private WifiTransferServer.CaptureControlResult
    runRemoteFinishCurrentRecording() {
        if (Looper.myLooper() == Looper.getMainLooper()) {
            return applyRemoteFinishCurrentRecording();
        }
        AtomicReference<WifiTransferServer.CaptureControlResult> result =
                new AtomicReference<>();
        CountDownLatch completed = new CountDownLatch(1);
        uiHandler.post(() -> {
            try {
                result.set(applyRemoteFinishCurrentRecording());
            } finally {
                completed.countDown();
            }
        });
        try {
            if (!completed.await(5L, TimeUnit.SECONDS)) {
                return new WifiTransferServer.CaptureControlResult(
                        false,
                        "Pixel UI did not respond to recording stop",
                        captureControlState
                );
            }
        } catch (InterruptedException exception) {
            Thread.currentThread().interrupt();
            return new WifiTransferServer.CaptureControlResult(
                    false,
                    "recording stop was interrupted",
                    captureControlState
            );
        }
        return result.get();
    }

    private WifiTransferServer.CaptureControlResult
    applyRemoteFinishCurrentRecording() {
        if (isFinishing() || isDestroyed()) {
            return rejectedRemoteControl("HFR Recorder is closing");
        }
        if (!manualContinuousStandby) {
            return rejectedRemoteControl(
                    "連続撮影スタンバイ中ではありません"
            );
        }
        if (!"recording".equals(captureControlState.state)
                || recorder == null
                || !recorder.isActive()
                || opticalWaitingForStart) {
            return rejectedRemoteControl(
                    "現在録画中の動画はありません"
            );
        }
        finishCurrentRecording();
        return acceptedRemoteControl();
    }

    private WifiTransferServer.CaptureControlResult acceptedRemoteControl() {
        return new WifiTransferServer.CaptureControlResult(
                true,
                null,
                captureControlState
        );
    }

    private WifiTransferServer.CaptureControlResult rejectedRemoteControl(
            String message
    ) {
        return new WifiTransferServer.CaptureControlResult(
                false,
                message,
                captureControlState
        );
    }

    private void stopFromPixel() {
        boolean stoppingContinuous = manualContinuousStandby;
        if (stoppingContinuous) {
            manualContinuousStandby = false;
            manualStopRequested = true;
            uiHandler.removeCallbacks(manualRearmRunnable);
            publishCaptureControlState(
                    "stopping",
                    recorder != null
                            && recorder.isActive()
                            && !opticalWaitingForStart,
                    "連続撮影を終了しています"
            );
        }
        if (recorder == null || !recorder.isActive()) {
            resetControlsAfterRun();
            String message = stoppingContinuous
                    ? String.format(
                    "連続撮影を終了しました（%d本保存）",
                    manualCompletedRuns
            )
                    : "撮影を停止しました";
            setStatus(message, true);
            publishCaptureControlState("idle", false, message);
            return;
        }
        opticalDetectionEnabled = false;
        finishRunButton.setEnabled(false);
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

    private void finishCurrentRecording() {
        if (!manualContinuousStandby
                || recorder == null
                || !recorder.isActive()
                || opticalWaitingForStart
                || !"recording".equals(captureControlState.state)) {
            setStatus("現在録画中の動画はありません", false);
            return;
        }
        opticalDetectionEnabled = false;
        opticalWaitingForMotion = false;
        finishRunButton.setEnabled(false);
        stopButton.setEnabled(false);
        String message = "現在の動画だけを終了して保存しています。連続待機は継続します";
        setStatus(message, true);
        publishCaptureControlState("finishing-run", true, message);
        recorder.stop();
    }

    private void startRecording(boolean useIntentConfig) {
        if (recorder != null && recorder.isActive()) {
            return;
        }
        if (!preview.isAvailable()) {
            manualContinuousStandby = false;
            resetControlsAfterRun();
            setStatus("Preview surface is not ready.", false);
            publishCaptureControlState(
                    "error",
                    false,
                    "Preview surface is not ready."
            );
            return;
        }
        HfrRecorder.Config config;
        try {
            config = useIntentConfig
                    ? readConfig(getIntent())
                    : createManualOpticalConfig();
        } catch (IllegalArgumentException exception) {
            manualContinuousStandby = false;
            resetControlsAfterRun();
            setStatus(exception.getMessage(), false);
            publishCaptureControlState(
                    "error",
                    false,
                    exception.getMessage()
            );
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
            publishCaptureControlState(
                    "error",
                    false,
                    "空き容量が1 GiB未満のため連続撮影を開始できません"
            );
            return;
        }
        if (wifiTransferServer != null && !wifiCaptureSlotOwned) {
            if (!wifiTransferServer.tryBeginCapture()) {
                manualContinuousStandby = false;
                resetControlsAfterRun();
                setStatus(
                        "Wi-Fi転送の完了後に撮影スタンバイを開始してください",
                        false
                );
                publishCaptureControlState(
                        "idle",
                        false,
                        "Wi-Fi転送の完了後に撮影スタンバイを開始してください"
                );
                return;
            }
            wifiCaptureSlotOwned = true;
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
                            finishRunButton.setEnabled(false);
                            stopButton.setText(
                                    config.retainRunOutput
                                            ? "連続待機を終了"
                                            : "待機をキャンセル"
                            );
                            stopButton.setEnabled(true);
                            publishCaptureControlState(
                                    "armed",
                                    false,
                                    config.retainRunOutput
                                            ? "LED STARTシグナルを待機しています"
                                            : "撮影開始を待機しています"
                            );
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
                                    opticalDetector.rearm();
                                }
                                motionGateDetector = new MotionGateDetector();
                                motionGateDetector.arm(
                                        opticalStartCenterX,
                                        opticalStartCenterY
                                );
                            }
                            startButton.setEnabled(false);
                            finishRunButton.setEnabled(
                                    config.retainRunOutput
                            );
                            stopButton.setText(
                                    config.retainRunOutput
                                            ? "連続待機を終了"
                                            : "録画を停止"
                            );
                            stopButton.setEnabled(true);
                            publishCaptureControlState(
                                    "recording",
                                    true,
                                    "走行を録画しています"
                            );
                        });
                    }

                    @Override
                    public void onCancelled(String message) {
                        runOnUiThread(() -> {
                            finishRunButton.setEnabled(false);
                            resetControlsAfterRun();
                            if (wifiTransferServer != null) {
                                wifiTransferServer.notifyRunsChanged();
                            }
                            String statusMessage = config.retainRunOutput
                                    ? String.format(
                                    "連続撮影を終了しました（%d本保存）",
                                    manualCompletedRuns
                            )
                                    : "撮影スタンバイを終了しました";
                            setStatus(statusMessage, true);
                            publishCaptureControlState(
                                    "idle",
                                    false,
                                    statusMessage
                            );
                        });
                    }

                    @Override
                    public void onFinished(String message) {
                        runOnUiThread(() -> {
                            finishRunButton.setEnabled(false);
                            if (config.retainRunOutput) {
                                manualCompletedRuns += 1;
                            }
                            if (config.retainRunOutput
                                    && manualContinuousStandby) {
                                prepareControlsForRearm();
                                String statusMessage = String.format(
                                        "%d本目を保存しました。2秒後に再待機します...",
                                        manualCompletedRuns
                                );
                                setStatus(statusMessage, true);
                                publishCaptureControlState(
                                        "rearming",
                                        false,
                                        statusMessage
                                );
                                uiHandler.postDelayed(
                                        manualRearmRunnable,
                                        MANUAL_REARM_DELAY_MS
                                );
                            } else {
                                resetControlsAfterRun();
                                String statusMessage = config.retainRunOutput
                                        && manualStopRequested
                                        ? String.format(
                                        "連続撮影を終了しました（%d本保存）",
                                        manualCompletedRuns
                                )
                                        : message;
                                setStatus(statusMessage, true);
                                publishCaptureControlState(
                                        "idle",
                                        false,
                                        statusMessage
                                );
                            }
                            if (wifiTransferServer != null) {
                                wifiTransferServer.notifyRunsChanged();
                            }
                        });
                    }

                    @Override
                    public void onError(String message) {
                        runOnUiThread(() -> {
                            finishRunButton.setEnabled(false);
                            manualContinuousStandby = false;
                            uiHandler.removeCallbacks(manualRearmRunnable);
                            resetControlsAfterRun();
                            if (wifiTransferServer != null) {
                                wifiTransferServer.notifyRunsChanged();
                            }
                            String statusMessage = "ERROR: " + message;
                            setStatus(statusMessage, false);
                            publishCaptureControlState(
                                    "error",
                                    false,
                                    statusMessage
                            );
                        });
                    }
                }
        );
        startButton.setEnabled(false);
        finishRunButton.setEnabled(false);
        stopButton.setEnabled(false);
        if (config.retainRunOutput) {
            stopButton.setText("連続待機を終了");
        }
        activeConfig = config;
        if (config.opticalTrigger) {
            opticalDetector = new OpticalTriggerDetector(
                    config.opticalTriggerScore,
                    config.opticalTriggerHotPixels
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
        finishRunButton.setEnabled(false);
        stopButton.setText("待機をキャンセル");
        stopButton.setEnabled(false);
        if (wifiTransferServer != null && wifiCaptureSlotOwned) {
            wifiCaptureSlotOwned = false;
            wifiTransferServer.endCapture();
        }
    }

    private void prepareControlsForRearm() {
        opticalDetectionEnabled = false;
        opticalWaitingForStart = false;
        opticalWaitingForMotion = false;
        startButton.setEnabled(false);
        finishRunButton.setEnabled(false);
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

    private void publishCaptureControlState(
            String state,
            boolean recording,
            String message
    ) {
        captureControlState = new WifiTransferServer.CaptureControlState(
                state,
                manualContinuousStandby,
                recording,
                manualCompletedRuns,
                manualSessionNonce,
                message == null ? "" : message
        );
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

        MotionGateDetector.Result motion = null;
        if (!opticalWaitingForStart
                && opticalWaitingForMotion
                && motionGateDetector != null) {
            motion = motionGateDetector.process(
                    opticalPixels,
                    OPTICAL_SAMPLE_WIDTH,
                    OPTICAL_SAMPLE_HEIGHT
            );
            if (motion.motionDetected) {
                opticalWaitingForMotion = false;
                recorder.noteMotionGate(
                        nowNs,
                        motion.changedPixels
                );
                setStatus(
                        String.format(
                                "Motion detected (%d changed px); STOP decoder remains armed",
                                motion.changedPixels
                        ),
                        true
                );
            }
        }
        OpticalTriggerDetector.Result result = opticalDetector.process(
                opticalPixels,
                OPTICAL_SAMPLE_WIDTH,
                OPTICAL_SAMPLE_HEIGHT,
                nowNs
        );
        if (result.tokenType != OpticalTriggerDetector.TokenType.NONE) {
            boolean expectedStart = opticalWaitingForStart
                    && result.tokenType
                    == OpticalTriggerDetector.TokenType.START;
            boolean expectedStop = !opticalWaitingForStart
                    && result.tokenType
                    == OpticalTriggerDetector.TokenType.STOP;
            Log.i(LOG_TAG, String.format(
                    "token=%s state=%s short=%d long=%d erasure=%d expected=%s",
                    result.tokenType,
                    opticalWaitingForStart ? "ARMED" : "REC",
                    result.shortVotes,
                    result.longVotes,
                    result.erasureVotes,
                    expectedStart || expectedStop
            ));
            if (expectedStart) {
                opticalDetectionEnabled = false;
                opticalStartCenterX = result.centerX;
                opticalStartCenterY = result.centerY;
                setStatus("LED START token detected; starting recorder...", true);
                recorder.triggerRecording(
                        nowNs,
                        result.score,
                        result.hotPixels,
                        result.threshold,
                        result.matchedLeds,
                        result.requiredVotes,
                        result.shortVotes,
                        result.longVotes,
                        result.erasureVotes,
                        result.centerX,
                        result.centerY
                );
            } else if (expectedStop) {
                opticalDetectionEnabled = false;
                finishRunButton.setEnabled(false);
                stopButton.setEnabled(false);
                String message = "LED STOP token detected; saving tail...";
                setStatus(message, true);
                publishCaptureControlState("finishing-run", true, message);
                recorder.triggerStop(
                        nowNs,
                        activeConfig.opticalStopTailMs,
                        result.score,
                        result.hotPixels,
                        result.threshold,
                        result.matchedLeds,
                        result.requiredVotes,
                        result.shortVotes,
                        result.longVotes,
                        result.erasureVotes
                );
            } else {
                String ignored = result.tokenType
                        == OpticalTriggerDetector.TokenType.INVALID
                        ? String.format(
                                "Invalid LED token ignored (S=%d L=%d E=%d)",
                                result.shortVotes,
                                result.longVotes,
                                result.erasureVotes
                        )
                        : String.format(
                                "%s token ignored while %s",
                                result.tokenType,
                                opticalWaitingForStart
                                        ? "waiting for START"
                                        : "recording"
                        );
                setStatus(ignored, false);
                lastOpticalStatusNs = nowNs;
            }
        } else if (nowNs - lastOpticalStatusNs
                >= OPTICAL_STATUS_INTERVAL_NS) {
            lastOpticalStatusNs = nowNs;
            String motionStatus = motion == null
                    ? ""
                    : String.format(
                            " motion=%s/%d",
                            motion.phase,
                            motion.changedPixels
                    );
            setStatus(String.format(
                    "%s LED: phase=%s score=%d/%d hot=%d matched=%d votes=%d/%d/%d%s",
                    opticalWaitingForStart ? "ARMED" : "REC",
                    result.phase,
                    result.score,
                    result.threshold,
                    result.hotPixels,
                    result.matchedLeds,
                    result.shortVotes,
                    result.longVotes,
                    result.erasureVotes,
                    motionStatus
            ), true);
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
        if (wifiTransferServer != null) {
            wifiTransferServer.close();
        }
        if (cameraThread != null) {
            cameraThread.quitSafely();
        }
        super.onDestroy();
    }
}
