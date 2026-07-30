package com.nightfall.hfrrecorder;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.SurfaceTexture;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
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

    private TextureView preview;
    private TextView status;
    private Button startButton;
    private Button stopButton;
    private HandlerThread cameraThread;
    private Handler cameraHandler;
    private HfrRecorder recorder;
    private boolean autoPending;

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
        startButton.setText("Record 1080p120");
        startButton.setOnClickListener(view -> startRecording());
        controls.addView(startButton);

        stopButton = new Button(this);
        stopButton.setText("Stop");
        stopButton.setEnabled(false);
        stopButton.setOnClickListener(view -> {
            if (recorder != null) {
                recorder.stop();
            }
        });
        controls.addView(stopButton);

        status = new TextView(this);
        status.setText("Waiting for camera...");
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
        startRecording();
    }

    private void startRecording() {
        if (recorder != null && recorder.isActive()) {
            return;
        }
        if (!preview.isAvailable()) {
            setStatus("Preview surface is not ready.", false);
            return;
        }
        HfrRecorder.Config config;
        try {
            config = readConfig(getIntent());
        } catch (IllegalArgumentException exception) {
            setStatus(exception.getMessage(), false);
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
                    public void onRecordingStarted() {
                        runOnUiThread(() -> {
                            startButton.setEnabled(false);
                            stopButton.setEnabled(true);
                        });
                    }

                    @Override
                    public void onFinished(String message) {
                        runOnUiThread(() -> {
                            startButton.setEnabled(true);
                            stopButton.setEnabled(false);
                            setStatus(message, true);
                        });
                    }

                    @Override
                    public void onError(String message) {
                        runOnUiThread(() -> {
                            startButton.setEnabled(true);
                            stopButton.setEnabled(false);
                            setStatus("ERROR: " + message, false);
                        });
                    }
                }
        );
        startButton.setEnabled(false);
        stopButton.setEnabled(false);
        recorder.start(config);
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
                intent.getIntExtra(EXTRA_ISO, 400)
        );
        config.validate();
        return config;
    }

    private void setStatus(String message, boolean normal) {
        status.setText(message);
        status.setTextColor(normal ? Color.WHITE : 0xffff8080);
    }

    @Override
    protected void onDestroy() {
        if (recorder != null) {
            recorder.close();
        }
        if (cameraThread != null) {
            cameraThread.quitSafely();
        }
        super.onDestroy();
    }
}
