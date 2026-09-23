package com.nightfall.hfrrecorder;

import android.content.Context;
import android.content.SharedPreferences;
import android.os.Build;
import android.os.SystemClock;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.ByteArrayOutputStream;
import java.io.EOFException;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.RandomAccessFile;
import java.io.UnsupportedEncodingException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.SecureRandom;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

final class WifiTransferServer implements AutoCloseable {
    interface Listener {
        void onSnapshot(Snapshot snapshot);
    }

    interface CaptureControlHandler {
        CaptureControlResult startContinuousStandby();

        CaptureControlResult stopContinuousStandby();

        CaptureControlResult finishCurrentRecording();

        CaptureControlResult startManualOneShot();

        CaptureControlResult stopManualOneShot();

        CaptureControlState captureControlState();
    }

    static final class CaptureControlState {
        final String state;
        final String captureMode;
        final boolean continuousStandby;
        final boolean recording;
        final int completedRuns;
        final String sessionNonce;
        final String message;

        CaptureControlState(
                String state,
                String captureMode,
                boolean continuousStandby,
                boolean recording,
                int completedRuns,
                String sessionNonce,
                String message
        ) {
            this.state = state;
            this.captureMode = captureMode;
            this.continuousStandby = continuousStandby;
            this.recording = recording;
            this.completedRuns = completedRuns;
            this.sessionNonce = sessionNonce;
            this.message = message;
        }

        JSONObject toJson() throws JSONException {
            JSONObject result = new JSONObject();
            result.put("state", state);
            result.put("capture_mode", captureMode);
            result.put("continuous_standby", continuousStandby);
            result.put("recording", recording);
            result.put("completed_runs", completedRuns);
            result.put("session_nonce", sessionNonce == null
                    ? JSONObject.NULL
                    : sessionNonce);
            result.put("message", message);
            return result;
        }
    }

    static final class CaptureControlResult {
        final boolean accepted;
        final String error;
        final CaptureControlState state;

        CaptureControlResult(
                boolean accepted,
                String error,
                CaptureControlState state
        ) {
            this.accepted = accepted;
            this.error = error;
            this.state = state;
        }
    }

    static final class Snapshot {
        final boolean running;
        final List<String> addresses;
        final String pairingCode;
        final long pairingSecondsRemaining;
        final int savedRuns;
        final int acknowledgedRuns;
        final boolean captureBusy;
        final int activeTransfers;
        final String error;

        Snapshot(
                boolean running,
                List<String> addresses,
                String pairingCode,
                long pairingSecondsRemaining,
                int savedRuns,
                int acknowledgedRuns,
                boolean captureBusy,
                int activeTransfers,
                String error
        ) {
            this.running = running;
            this.addresses = addresses;
            this.pairingCode = pairingCode;
            this.pairingSecondsRemaining = pairingSecondsRemaining;
            this.savedRuns = savedRuns;
            this.acknowledgedRuns = acknowledgedRuns;
            this.captureBusy = captureBusy;
            this.activeTransfers = activeTransfers;
            this.error = error;
        }
    }

    static final class DeleteResult {
        final int runs;
        final long bytes;
        final String error;

        DeleteResult(int runs, long bytes, String error) {
            this.runs = runs;
            this.bytes = bytes;
            this.error = error;
        }
    }

    private static final String PREFS_NAME = "wifi_transfer";
    private static final String PREF_DEVICE_ID = "device_id";
    private static final String PREF_ACCESS_TOKEN = "access_token";
    private static final String PREF_PAIRED = "paired";
    private static final String PREF_ACK_PREFIX = "ack.";
    private static final long PAIRING_LIFETIME_MS = 10L * 60L * 1000L;
    private static final int MAX_PAIR_ATTEMPTS = 8;
    private static final int MAX_HEADER_LINE_BYTES = 8192;
    private static final int MAX_PAIR_BODY_BYTES = 2048;
    private static final int FILE_BUFFER_BYTES = 256 * 1024;

    private final Context context;
    private final File retainedRoot;
    private final SharedPreferences preferences;
    private final Listener listener;
    private final CaptureControlHandler captureControlHandler;
    private final SecureRandom random = new SecureRandom();
    private final AtomicInteger activeTransfers = new AtomicInteger();
    private final ExecutorService clientExecutor =
            Executors.newFixedThreadPool(2);
    private final String deviceId;
    private final String accessToken;
    private final String appVersion;

    private volatile boolean running;
    private volatile boolean captureBusy;
    private volatile String error;
    private volatile ServerSocket httpSocket;
    private volatile DatagramSocket discoverySocket;
    private Thread httpThread;
    private Thread discoveryThread;
    private String pairingCode;
    private long pairingExpiresElapsedMs;
    private int pairingAttempts;

    WifiTransferServer(
            Context context,
            Listener listener,
            CaptureControlHandler captureControlHandler
    ) {
        this.context = context.getApplicationContext();
        this.listener = listener;
        this.captureControlHandler = captureControlHandler;
        retainedRoot = new File(context.getFilesDir(), "manual_runs");
        preferences = context.getSharedPreferences(
                PREFS_NAME,
                Context.MODE_PRIVATE
        );
        deviceId = loadOrCreatePreference(PREF_DEVICE_ID, false);
        accessToken = loadOrCreatePreference(PREF_ACCESS_TOKEN, true);
        appVersion = resolveAppVersion(context);
    }

    synchronized void start() {
        if (running) {
            return;
        }
        running = true;
        error = null;
        issuePairingCodeLocked();
        httpThread = new Thread(this::serveHttp, "nightfall-hfr-http");
        discoveryThread = new Thread(
                this::serveDiscovery,
                "nightfall-hfr-discovery"
        );
        httpThread.start();
        discoveryThread.start();
        notifyListener();
    }

    synchronized void issuePairingCode() {
        issuePairingCodeLocked();
        notifyListener();
    }

    synchronized boolean tryBeginCapture() {
        if (captureBusy || activeTransfers.get() > 0) {
            return false;
        }
        captureBusy = true;
        notifyListener();
        return true;
    }

    synchronized void endCapture() {
        captureBusy = false;
        notifyListener();
    }

    boolean isTransferActive() {
        return activeTransfers.get() > 0;
    }

    void notifyRunsChanged() {
        notifyListener();
    }

    DeleteResult deleteAcknowledgedRuns() {
        synchronized (this) {
            if (captureBusy) {
                return new DeleteResult(0, 0L, "撮影待機中は削除できません");
            }
            if (activeTransfers.get() > 0) {
                return new DeleteResult(0, 0L, "転送中は削除できません");
            }
            captureBusy = true;
        }
        notifyListener();
        try {
            File[] directories = retainedRoot.listFiles(File::isDirectory);
            if (directories == null) {
                return new DeleteResult(0, 0L, null);
            }
            int deletedRuns = 0;
            long deletedBytes = 0L;
            SharedPreferences.Editor editor = preferences.edit();
            for (File directory : directories) {
                String runName = directory.getName();
                if (!WifiTransferProtocol.isSafeRunName(runName)
                        || acknowledgedAt(runName) <= 0L
                        || !isContainedRunDirectory(directory)) {
                    continue;
                }
                long bytes = directoryBytes(directory);
                if (deleteRecursively(directory)) {
                    deletedRuns += 1;
                    deletedBytes += bytes;
                    editor.remove(PREF_ACK_PREFIX + runName);
                }
            }
            editor.apply();
            return new DeleteResult(deletedRuns, deletedBytes, null);
        } finally {
            endCapture();
        }
    }

    Snapshot snapshot() {
        long remainingMs;
        String code;
        synchronized (this) {
            remainingMs = pairingExpiresElapsedMs
                    - SystemClock.elapsedRealtime();
            code = remainingMs > 0L ? pairingCode : null;
        }
        InventoryCounts counts = inventoryCounts();
        return new Snapshot(
                running,
                localIpv4Addresses(),
                code,
                Math.max(0L, (remainingMs + 999L) / 1000L),
                counts.saved,
                counts.acknowledged,
                captureBusy,
                activeTransfers.get(),
                error
        );
    }

    @Override
    public synchronized void close() {
        if (!running) {
            return;
        }
        running = false;
        closeQuietly(httpSocket);
        if (discoverySocket != null) {
            discoverySocket.close();
        }
        clientExecutor.shutdownNow();
        joinQuietly(httpThread);
        joinQuietly(discoveryThread);
        notifyListener();
    }

    private String loadOrCreatePreference(String key, boolean secret) {
        String value = preferences.getString(key, null);
        if (value != null && !value.isBlank()) {
            return value;
        }
        if (secret) {
            byte[] bytes = new byte[32];
            random.nextBytes(bytes);
            StringBuilder builder = new StringBuilder(bytes.length * 2);
            for (byte item : bytes) {
                builder.append(String.format(Locale.US, "%02x", item & 0xff));
            }
            value = builder.toString();
        } else {
            value = UUID.randomUUID().toString();
        }
        preferences.edit().putString(key, value).apply();
        return value;
    }

    private void issuePairingCodeLocked() {
        pairingCode = String.format(
                Locale.US,
                "%06d",
                random.nextInt(1_000_000)
        );
        pairingExpiresElapsedMs = SystemClock.elapsedRealtime()
                + PAIRING_LIFETIME_MS;
        pairingAttempts = 0;
    }

    private void serveHttp() {
        try (ServerSocket server = new ServerSocket()) {
            server.setReuseAddress(true);
            server.bind(new InetSocketAddress(WifiTransferProtocol.HTTP_PORT));
            httpSocket = server;
            notifyListener();
            while (running) {
                Socket client;
                try {
                    client = server.accept();
                } catch (SocketException exception) {
                    if (!running) {
                        break;
                    }
                    throw exception;
                }
                clientExecutor.execute(() -> handleHttp(client));
            }
        } catch (IOException exception) {
            if (running) {
                setError("HTTP server: " + exception.getMessage());
            }
        } finally {
            httpSocket = null;
        }
    }

    private void serveDiscovery() {
        byte[] buffer = new byte[1024];
        try (DatagramSocket socket = new DatagramSocket(null)) {
            socket.setReuseAddress(true);
            socket.bind(new InetSocketAddress(
                    "0.0.0.0",
                    WifiTransferProtocol.DISCOVERY_PORT
            ));
            discoverySocket = socket;
            while (running) {
                DatagramPacket packet = new DatagramPacket(
                        buffer,
                        buffer.length
                );
                try {
                    socket.receive(packet);
                } catch (SocketException exception) {
                    if (!running) {
                        break;
                    }
                    throw exception;
                }
                String request = new String(
                        packet.getData(),
                        packet.getOffset(),
                        packet.getLength(),
                        StandardCharsets.US_ASCII
                ).trim();
                if (!WifiTransferProtocol.DISCOVERY_REQUEST.equals(request)) {
                    continue;
                }
                byte[] response = infoJson().toString()
                        .getBytes(StandardCharsets.UTF_8);
                DatagramPacket reply = new DatagramPacket(
                        response,
                        response.length,
                        packet.getAddress(),
                        packet.getPort()
                );
                socket.send(reply);
            }
        } catch (IOException | JSONException exception) {
            if (running) {
                setError("Discovery server: " + exception.getMessage());
            }
        } finally {
            discoverySocket = null;
        }
    }

    private void handleHttp(Socket socket) {
        try (Socket client = socket;
             BufferedInputStream input = new BufferedInputStream(
                     client.getInputStream()
             );
             BufferedOutputStream output = new BufferedOutputStream(
                     client.getOutputStream()
             )) {
            client.setSoTimeout(60_000);
            client.setTcpNoDelay(true);
            String requestLine = readLine(input);
            if (requestLine == null || requestLine.isBlank()) {
                return;
            }
            String[] parts = requestLine.split(" ");
            if (parts.length != 3 || !parts[2].startsWith("HTTP/1.")) {
                sendError(output, 400, "invalid request line");
                return;
            }
            String method = parts[0];
            String path = parts[1];
            Map<String, String> headers = readHeaders(input);

            if ("GET".equals(method) && "/api/v1/info".equals(path)) {
                sendJson(output, 200, infoJson());
                return;
            }
            if ("POST".equals(method) && "/api/v1/pair".equals(path)) {
                handlePair(input, output, headers);
                return;
            }
            if (!authorized(headers)) {
                sendError(output, 401, "authorization required");
                return;
            }
            if ("GET".equals(method) && "/api/v1/runs".equals(path)) {
                sendJson(output, 200, runsJson());
                return;
            }
            if ("GET".equals(method)
                    && "/api/v1/control/status".equals(path)) {
                sendJson(output, 200, infoJson());
                return;
            }
            if ("POST".equals(method)
                    && "/api/v1/control/standby/start".equals(path)) {
                handleCaptureControl(true, output);
                return;
            }
            if ("POST".equals(method)
                    && "/api/v1/control/standby/stop".equals(path)) {
                handleCaptureControl(false, output);
                return;
            }
            if ("POST".equals(method)
                    && "/api/v1/control/recording/stop".equals(path)) {
                handleCurrentRecordingStop(output);
                return;
            }
            if ("POST".equals(method)
                    && "/api/v1/control/manual/start".equals(path)) {
                handleManualOneShotControl(true, output);
                return;
            }
            if ("POST".equals(method)
                    && "/api/v1/control/manual/stop".equals(path)) {
                handleManualOneShotControl(false, output);
                return;
            }
            if (path.startsWith("/api/v1/runs/")) {
                handleRunRequest(method, path, headers, output);
                return;
            }
            sendError(output, 404, "not found");
        } catch (Exception ignored) {
            // A disconnected collector must never terminate the server.
        }
    }

    private void handleCaptureControl(
            boolean start,
            OutputStream output
    ) throws IOException, JSONException {
        if (captureControlHandler == null) {
            sendError(output, 503, "capture control is unavailable");
            return;
        }
        CaptureControlResult result = start
                ? captureControlHandler.startContinuousStandby()
                : captureControlHandler.stopContinuousStandby();
        if (!result.accepted) {
            sendError(
                    output,
                    409,
                    result.error == null
                            ? "capture control request was rejected"
                            : result.error
            );
            return;
        }
        JSONObject response = infoJson();
        response.put("control_accepted", true);
        response.put("capture_control", result.state.toJson());
        sendJson(output, 200, response);
    }

    private void handleCurrentRecordingStop(
            OutputStream output
    ) throws IOException, JSONException {
        if (captureControlHandler == null) {
            sendError(output, 503, "capture control is unavailable");
            return;
        }
        CaptureControlResult result =
                captureControlHandler.finishCurrentRecording();
        if (!result.accepted) {
            sendError(
                    output,
                    409,
                    result.error == null
                            ? "recording stop request was rejected"
                            : result.error
            );
            return;
        }
        JSONObject response = infoJson();
        response.put("control_accepted", true);
        response.put("capture_control", result.state.toJson());
        sendJson(output, 200, response);
    }

    private void handleManualOneShotControl(
            boolean start,
            OutputStream output
    ) throws IOException, JSONException {
        if (captureControlHandler == null) {
            sendError(output, 503, "capture control is unavailable");
            return;
        }
        CaptureControlResult result = start
                ? captureControlHandler.startManualOneShot()
                : captureControlHandler.stopManualOneShot();
        if (!result.accepted) {
            sendError(
                    output,
                    409,
                    result.error == null
                            ? "manual one-shot request was rejected"
                            : result.error
            );
            return;
        }
        JSONObject response = infoJson();
        response.put("control_accepted", true);
        response.put("capture_control", result.state.toJson());
        sendJson(output, 200, response);
    }

    private void handlePair(
            BufferedInputStream input,
            OutputStream output,
            Map<String, String> headers
    ) throws IOException, JSONException {
        int length = parseContentLength(headers, MAX_PAIR_BODY_BYTES);
        byte[] body = readExactly(input, length);
        String supplied = new JSONObject(
                new String(body, StandardCharsets.UTF_8)
        ).optString("code", "");
        boolean accepted;
        synchronized (this) {
            long now = SystemClock.elapsedRealtime();
            accepted = now < pairingExpiresElapsedMs
                    && pairingAttempts < MAX_PAIR_ATTEMPTS
                    && constantEquals(pairingCode, supplied);
            if (accepted) {
                pairingExpiresElapsedMs = 0L;
                pairingCode = null;
                preferences.edit().putBoolean(PREF_PAIRED, true).apply();
            } else {
                pairingAttempts += 1;
            }
        }
        notifyListener();
        if (!accepted) {
            sendError(output, 403, "pairing code invalid or expired");
            return;
        }
        JSONObject response = infoJson();
        response.put("access_token", accessToken);
        sendJson(output, 200, response);
    }

    private void handleRunRequest(
            String method,
            String path,
            Map<String, String> headers,
            OutputStream output
    ) throws IOException, JSONException {
        String relative = path.substring("/api/v1/runs/".length());
        int slash = relative.indexOf('/');
        if (slash <= 0 || slash == relative.length() - 1) {
            sendError(output, 404, "not found");
            return;
        }
        String runName = decodePathPart(relative.substring(0, slash));
        String resource = decodePathPart(relative.substring(slash + 1));
        if (!WifiTransferProtocol.isSafeRunName(runName)) {
            sendError(output, 400, "unsafe run name");
            return;
        }
        if ("POST".equals(method) && "ack".equals(resource)) {
            acknowledgeRun(runName, output);
            return;
        }
        if (!"GET".equals(method)
                || !WifiTransferProtocol.isArtifactName(resource)) {
            sendError(output, 404, "not found");
            return;
        }
        File runDirectory;
        try {
            runDirectory = resolveRunDirectory(runName);
        } catch (IOException exception) {
            sendError(output, 404, "run not found");
            return;
        }
        File artifact = new File(runDirectory, resource);
        if (!artifact.isFile()) {
            sendError(output, 404, "artifact not found");
            return;
        }
        sendArtifact(output, artifact, resource, headers.get("range"));
    }

    private void acknowledgeRun(String runName, OutputStream output)
            throws IOException, JSONException {
        File directory;
        try {
            directory = resolveRunDirectory(runName);
        } catch (IOException exception) {
            sendError(output, 404, "run not found");
            return;
        }
        File report = new File(directory, "hfr_report.json");
        if (!reportMatchesRun(report, runName)) {
            sendError(output, 409, "run report is incomplete or invalid");
            return;
        }
        long acknowledgedAt = System.currentTimeMillis();
        preferences.edit().putLong(
                PREF_ACK_PREFIX + runName,
                acknowledgedAt
        ).apply();
        notifyListener();
        JSONObject response = new JSONObject();
        response.put("schema", WifiTransferProtocol.SCHEMA);
        response.put("run_name", runName);
        response.put("acknowledged_at_unix_ms", acknowledgedAt);
        sendJson(output, 200, response);
    }

    private void sendArtifact(
            OutputStream output,
            File file,
            String artifactName,
            String rangeHeader
    ) throws IOException {
        if (!tryBeginTransfer()) {
            sendError(
                    output,
                    503,
                    "capture is active; stop continuous standby before transfer",
                    Map.of("Retry-After", "3")
            );
            return;
        }
        long fileLength = file.length();
        WifiTransferProtocol.ByteRange range;
        try {
            range = WifiTransferProtocol.parseRange(rangeHeader, fileLength);
        } catch (IllegalArgumentException exception) {
            try {
                sendError(
                        output,
                        416,
                        "range not satisfiable",
                        Map.of("Content-Range", "bytes */" + fileLength)
                );
            } finally {
                endTransfer();
            }
            return;
        }
        try (RandomAccessFile input = new RandomAccessFile(file, "r")) {
            Map<String, String> headers = new HashMap<>();
            headers.put("Accept-Ranges", "bytes");
            headers.put(
                    "Content-Type",
                    artifactName.endsWith(".mp4")
                            ? "video/mp4"
                            : "application/json"
            );
            headers.put(
                    "ETag",
                    String.format(
                            Locale.US,
                            "\"%x-%x\"",
                            fileLength,
                            file.lastModified()
                    )
            );
            if (range.partial) {
                headers.put(
                        "Content-Range",
                        String.format(
                                Locale.US,
                                "bytes %d-%d/%d",
                                range.start,
                                range.end,
                                fileLength
                        )
                );
            }
            sendHeaders(
                    output,
                    range.partial ? 206 : 200,
                    range.length(),
                    headers
            );
            input.seek(range.start);
            byte[] buffer = new byte[FILE_BUFFER_BYTES];
            long remaining = range.length();
            while (remaining > 0L) {
                int count = input.read(
                        buffer,
                        0,
                        (int) Math.min(buffer.length, remaining)
                );
                if (count < 0) {
                    throw new EOFException("artifact shortened during transfer");
                }
                output.write(buffer, 0, count);
                remaining -= count;
            }
            output.flush();
        } finally {
            endTransfer();
        }
    }

    private synchronized boolean tryBeginTransfer() {
        if (captureBusy) {
            return false;
        }
        activeTransfers.incrementAndGet();
        notifyListener();
        return true;
    }

    private synchronized void endTransfer() {
        activeTransfers.decrementAndGet();
        notifyListener();
    }

    private JSONObject infoJson() throws JSONException {
        InventoryCounts counts = inventoryCounts();
        JSONObject result = new JSONObject();
        result.put("schema", WifiTransferProtocol.SCHEMA);
        result.put("device_id", deviceId);
        result.put("model", Build.MODEL);
        result.put("app_version", appVersion);
        result.put("http_port", WifiTransferProtocol.HTTP_PORT);
        result.put("paired", preferences.getBoolean(PREF_PAIRED, false));
        result.put("saved_runs", counts.saved);
        result.put("acknowledged_runs", counts.acknowledged);
        result.put("capture_busy", captureBusy);
        result.put("active_transfers", activeTransfers.get());
        if (captureControlHandler != null) {
            CaptureControlState controlState =
                    captureControlHandler.captureControlState();
            if (controlState != null) {
                result.put("capture_control", controlState.toJson());
            }
        }
        return result;
    }

    private JSONObject runsJson() throws JSONException {
        JSONArray runs = new JSONArray();
        for (File directory : validRunDirectories()) {
            String runName = directory.getName();
            File reportFile = new File(directory, "hfr_report.json");
            JSONObject report = readReport(reportFile);
            if (report == null
                    || !runName.equals(report.optString("record_nonce"))) {
                continue;
            }
            JSONObject run = new JSONObject();
            run.put("name", runName);
            run.put("status", report.optString("status", "missing"));
            run.put("acknowledged", acknowledgedAt(runName) > 0L);
            run.put("acknowledged_at_unix_ms", acknowledgedAt(runName));
            JSONArray artifacts = new JSONArray();
            for (String artifactName : WifiTransferProtocol.ARTIFACT_NAMES) {
                File artifact = new File(directory, artifactName);
                if (!artifact.isFile()) {
                    continue;
                }
                JSONObject item = new JSONObject();
                item.put("name", artifactName);
                item.put("size", artifact.length());
                item.put("last_modified_unix_ms", artifact.lastModified());
                artifacts.put(item);
            }
            run.put("artifacts", artifacts);
            runs.put(run);
        }
        JSONObject result = infoJson();
        result.put("runs", runs);
        return result;
    }

    private boolean authorized(Map<String, String> headers) {
        String authorization = headers.get("authorization");
        if (authorization == null || !authorization.startsWith("Bearer ")) {
            return false;
        }
        return constantEquals(
                accessToken,
                authorization.substring("Bearer ".length())
        );
    }

    private File resolveRunDirectory(String runName) throws IOException {
        File directory = new File(retainedRoot, runName);
        if (!directory.isDirectory() || !isContainedRunDirectory(directory)) {
            throw new IOException("run directory not found");
        }
        return directory;
    }

    private boolean isContainedRunDirectory(File directory) {
        try {
            String rootPath = retainedRoot.getCanonicalPath()
                    + File.separator;
            String directoryPath = directory.getCanonicalPath()
                    + File.separator;
            return directoryPath.startsWith(rootPath);
        } catch (IOException exception) {
            return false;
        }
    }

    private List<File> validRunDirectories() {
        File[] directories = retainedRoot.listFiles(File::isDirectory);
        if (directories == null) {
            return List.of();
        }
        List<File> result = new ArrayList<>();
        for (File directory : directories) {
            if (WifiTransferProtocol.isSafeRunName(directory.getName())
                    && isContainedRunDirectory(directory)) {
                result.add(directory);
            }
        }
        result.sort(Comparator.comparing(File::getName));
        return result;
    }

    private InventoryCounts inventoryCounts() {
        int saved = 0;
        int acknowledged = 0;
        for (File directory : validRunDirectories()) {
            if (!new File(directory, "hfr_report.json").isFile()) {
                continue;
            }
            saved += 1;
            if (acknowledgedAt(directory.getName()) > 0L) {
                acknowledged += 1;
            }
        }
        return new InventoryCounts(saved, acknowledged);
    }

    private long acknowledgedAt(String runName) {
        return preferences.getLong(PREF_ACK_PREFIX + runName, 0L);
    }

    private boolean reportMatchesRun(File file, String runName) {
        JSONObject report = readReport(file);
        return report != null
                && "nightfall_android_hfr_recording_v1".equals(
                report.optString("schema")
        )
                && runName.equals(report.optString("record_nonce"));
    }

    private JSONObject readReport(File file) {
        if (!file.isFile() || file.length() > 1024L * 1024L) {
            return null;
        }
        try {
            int length = (int) file.length();
            byte[] bytes = new byte[length];
            int offset = 0;
            try (FileInputStream input = new FileInputStream(file)) {
                while (offset < length) {
                    int count = input.read(bytes, offset, length - offset);
                    if (count < 0) {
                        return null;
                    }
                    offset += count;
                }
            }
            return new JSONObject(new String(bytes, StandardCharsets.UTF_8));
        } catch (IOException | JSONException exception) {
            return null;
        }
    }

    private static String resolveAppVersion(Context context) {
        try {
            return context.getPackageManager()
                    .getPackageInfo(context.getPackageName(), 0)
                    .versionName;
        } catch (Exception exception) {
            return "unknown";
        }
    }

    private static Map<String, String> readHeaders(BufferedInputStream input)
            throws IOException {
        Map<String, String> headers = new HashMap<>();
        while (true) {
            String line = readLine(input);
            if (line == null || line.isEmpty()) {
                return headers;
            }
            int colon = line.indexOf(':');
            if (colon <= 0) {
                throw new IOException("invalid HTTP header");
            }
            headers.put(
                    line.substring(0, colon).trim().toLowerCase(Locale.US),
                    line.substring(colon + 1).trim()
            );
        }
    }

    private static String readLine(BufferedInputStream input)
            throws IOException {
        ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        while (bytes.size() <= MAX_HEADER_LINE_BYTES) {
            int value = input.read();
            if (value < 0) {
                return bytes.size() == 0
                        ? null
                        : new String(
                        bytes.toByteArray(),
                        StandardCharsets.US_ASCII
                );
            }
            if (value == '\n') {
                byte[] result = bytes.toByteArray();
                int length = result.length;
                if (length > 0 && result[length - 1] == '\r') {
                    length -= 1;
                }
                return new String(
                        result,
                        0,
                        length,
                        StandardCharsets.US_ASCII
                );
            }
            bytes.write(value);
        }
        throw new IOException("HTTP header line too long");
    }

    private static int parseContentLength(
            Map<String, String> headers,
            int maximum
    ) throws IOException {
        String value = headers.get("content-length");
        if (value == null) {
            throw new IOException("missing Content-Length");
        }
        try {
            int length = Integer.parseInt(value);
            if (length < 0 || length > maximum) {
                throw new IOException("request body too large");
            }
            return length;
        } catch (NumberFormatException exception) {
            throw new IOException("invalid Content-Length", exception);
        }
    }

    private static String decodePathPart(String value) {
        try {
            return URLDecoder.decode(value, "UTF-8");
        } catch (UnsupportedEncodingException exception) {
            throw new IllegalStateException(exception);
        }
    }

    private static byte[] readExactly(
            BufferedInputStream input,
            int length
    ) throws IOException {
        byte[] result = new byte[length];
        int offset = 0;
        while (offset < length) {
            int count = input.read(result, offset, length - offset);
            if (count < 0) {
                throw new EOFException("short request body");
            }
            offset += count;
        }
        return result;
    }

    private static boolean constantEquals(String expected, String supplied) {
        if (expected == null || supplied == null) {
            return false;
        }
        return MessageDigest.isEqual(
                expected.getBytes(StandardCharsets.UTF_8),
                supplied.getBytes(StandardCharsets.UTF_8)
        );
    }

    private static void sendJson(
            OutputStream output,
            int status,
            JSONObject body
    ) throws IOException {
        byte[] bytes = body.toString().getBytes(StandardCharsets.UTF_8);
        sendHeaders(
                output,
                status,
                bytes.length,
                Map.of("Content-Type", "application/json; charset=utf-8")
        );
        output.write(bytes);
        output.flush();
    }

    private static void sendError(
            OutputStream output,
            int status,
            String message
    ) throws IOException {
        sendError(output, status, message, Map.of());
    }

    private static void sendError(
            OutputStream output,
            int status,
            String message,
            Map<String, String> headers
    ) throws IOException {
        JSONObject body = new JSONObject();
        try {
            body.put("schema", WifiTransferProtocol.SCHEMA);
            body.put("error", message);
        } catch (JSONException exception) {
            throw new IOException(exception);
        }
        byte[] bytes = body.toString().getBytes(StandardCharsets.UTF_8);
        Map<String, String> combined = new HashMap<>(headers);
        combined.put("Content-Type", "application/json; charset=utf-8");
        sendHeaders(output, status, bytes.length, combined);
        output.write(bytes);
        output.flush();
    }

    private static void sendHeaders(
            OutputStream output,
            int status,
            long contentLength,
            Map<String, String> headers
    ) throws IOException {
        StringBuilder builder = new StringBuilder();
        builder.append("HTTP/1.1 ")
                .append(status)
                .append(' ')
                .append(reason(status))
                .append("\r\n");
        builder.append("Connection: close\r\n");
        builder.append("Cache-Control: no-store\r\n");
        builder.append("Content-Length: ")
                .append(contentLength)
                .append("\r\n");
        for (Map.Entry<String, String> entry : headers.entrySet()) {
            builder.append(entry.getKey())
                    .append(": ")
                    .append(entry.getValue())
                    .append("\r\n");
        }
        builder.append("\r\n");
        output.write(builder.toString().getBytes(StandardCharsets.US_ASCII));
    }

    private static String reason(int status) {
        return switch (status) {
            case 200 -> "OK";
            case 206 -> "Partial Content";
            case 400 -> "Bad Request";
            case 401 -> "Unauthorized";
            case 403 -> "Forbidden";
            case 404 -> "Not Found";
            case 409 -> "Conflict";
            case 416 -> "Range Not Satisfiable";
            case 503 -> "Service Unavailable";
            default -> "Error";
        };
    }

    private static List<String> localIpv4Addresses() {
        List<String> result = new ArrayList<>();
        try {
            Enumeration<NetworkInterface> interfaces =
                    NetworkInterface.getNetworkInterfaces();
            if (interfaces == null) {
                return result;
            }
            while (interfaces.hasMoreElements()) {
                NetworkInterface network = interfaces.nextElement();
                if (!network.isUp() || network.isLoopback()) {
                    continue;
                }
                Enumeration<InetAddress> addresses =
                        network.getInetAddresses();
                while (addresses.hasMoreElements()) {
                    InetAddress address = addresses.nextElement();
                    if (address instanceof Inet4Address
                            && !address.isLoopbackAddress()) {
                        result.add(address.getHostAddress());
                    }
                }
            }
        } catch (SocketException ignored) {
        }
        Collections.sort(result);
        return result;
    }

    private static long directoryBytes(File file) {
        if (file.isFile()) {
            return file.length();
        }
        File[] children = file.listFiles();
        if (children == null) {
            return 0L;
        }
        long total = 0L;
        for (File child : children) {
            total += directoryBytes(child);
        }
        return total;
    }

    private static boolean deleteRecursively(File file) {
        File[] children = file.listFiles();
        if (children != null) {
            for (File child : children) {
                if (!deleteRecursively(child)) {
                    return false;
                }
            }
        }
        return file.delete();
    }

    private void setError(String message) {
        error = message;
        notifyListener();
    }

    private void notifyListener() {
        if (listener != null) {
            listener.onSnapshot(snapshot());
        }
    }

    private static void closeQuietly(ServerSocket socket) {
        if (socket == null) {
            return;
        }
        try {
            socket.close();
        } catch (IOException ignored) {
        }
    }

    private static void joinQuietly(Thread thread) {
        if (thread == null || thread == Thread.currentThread()) {
            return;
        }
        try {
            thread.join(TimeUnit.SECONDS.toMillis(2L));
        } catch (InterruptedException exception) {
            Thread.currentThread().interrupt();
        }
    }

    private static final class InventoryCounts {
        final int saved;
        final int acknowledged;

        InventoryCounts(int saved, int acknowledged) {
            this.saved = saved;
            this.acknowledged = acknowledged;
        }
    }
}
