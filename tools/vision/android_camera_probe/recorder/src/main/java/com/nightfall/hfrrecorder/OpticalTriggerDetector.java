package com.nightfall.hfrrecorder;

/**
 * Detects the three-pulse visible-LED token emitted by the F413 firmware.
 *
 * <p>Only rising edges are decoded. Consecutive preview frames are compared,
 * then three local brightening events must occur at the same image location
 * about 500 ms apart. A short calibration period derives a threshold from
 * real preview noise. This class has no Android dependencies so its state
 * machine can be exercised by the host JDK.</p>
 */
final class OpticalTriggerDetector {
    private static final int TILE_SIZE = 8;
    private static final int PIXEL_DELTA_THRESHOLD = 20;
    private static final long CALIBRATION_WARMUP_NS = 800_000_000L;
    private static final long CALIBRATION_SAMPLE_NS = 1_200_000_000L;
    private static final int CALIBRATION_CAPACITY = 64;
    private static final int CALIBRATION_PERCENTILE_NUMERATOR = 9;
    private static final int CALIBRATION_PERCENTILE_DENOMINATOR = 10;
    private static final int CALIBRATION_MARGIN = 200;
    private static final int CALIBRATION_MULTIPLIER = 3;
    private static final long RISE_REFRACTORY_NS = 150_000_000L;
    private static final long RISE_GAP_MIN_NS = 320_000_000L;
    private static final long RISE_GAP_MAX_NS = 750_000_000L;
    private static final int MAX_TILE_DISTANCE = 6;

    static final class Result {
        final boolean triggered;
        final int score;
        final int hotPixels;
        final int threshold;
        final String phase;

        Result(
                boolean triggered,
                int score,
                int hotPixels,
                int threshold,
                String phase
        ) {
            this.triggered = triggered;
            this.score = score;
            this.hotPixels = hotPixels;
            this.threshold = threshold;
            this.phase = phase;
        }
    }

    private enum Phase {
        CALIBRATING,
        WAIT_FIRST_RISE,
        WAIT_SECOND_RISE,
        WAIT_THIRD_RISE
    }

    private final int configuredScoreThreshold;
    private final int hotPixelThreshold;
    private int[] previous;
    private int width;
    private int height;
    private int[] tileScores;
    private int[] tileHotPixels;
    private int tileColumns;
    private int tileRows;
    private final int[] calibrationScores =
            new int[CALIBRATION_CAPACITY];
    private int calibrationCount;
    private long calibrationStartedNs = -1L;
    private boolean calibrationComplete;
    private int effectiveScoreThreshold;
    private Phase phase = Phase.CALIBRATING;
    private long lastRiseNs;
    private int candidateTileX = -1;
    private int candidateTileY = -1;

    OpticalTriggerDetector(int scoreThreshold, int hotPixelThreshold) {
        if (scoreThreshold < 1 || hotPixelThreshold < 1) {
            throw new IllegalArgumentException(
                    "optical trigger thresholds must be positive"
            );
        }
        configuredScoreThreshold = scoreThreshold;
        effectiveScoreThreshold = scoreThreshold;
        this.hotPixelThreshold = hotPixelThreshold;
    }

    void reset() {
        previous = null;
        tileScores = null;
        tileHotPixels = null;
        width = 0;
        height = 0;
        tileColumns = 0;
        tileRows = 0;
        calibrationCount = 0;
        calibrationStartedNs = -1L;
        calibrationComplete = false;
        effectiveScoreThreshold = configuredScoreThreshold;
        phase = Phase.CALIBRATING;
        resetSequence();
    }

    void rearm() {
        previous = null;
        resetSequence();
        phase = calibrationComplete
                ? Phase.WAIT_FIRST_RISE
                : Phase.CALIBRATING;
    }

    Result process(int[] pixels, int frameWidth, int frameHeight, long nowNs) {
        if (pixels == null
                || frameWidth < 1
                || frameHeight < 1
                || pixels.length != frameWidth * frameHeight) {
            throw new IllegalArgumentException("invalid optical trigger frame");
        }
        if (ensureBuffers(pixels, frameWidth, frameHeight, nowNs)) {
            return result(false, 0, 0);
        }

        java.util.Arrays.fill(tileScores, 0);
        java.util.Arrays.fill(tileHotPixels, 0);
        for (int y = 0; y < height; y += 1) {
            int tileY = y / TILE_SIZE;
            int rowOffset = y * width;
            for (int x = 0; x < width; x += 1) {
                int index = rowOffset + x;
                int current = brightness(pixels[index]);
                int delta = current - previous[index];
                if (delta >= PIXEL_DELTA_THRESHOLD) {
                    int tileIndex = tileY * tileColumns + x / TILE_SIZE;
                    tileScores[tileIndex] += delta;
                    tileHotPixels[tileIndex] += 1;
                }
                previous[index] = current;
            }
        }

        int bestScore = 0;
        int bestHotPixels = 0;
        int bestTileX = 0;
        int bestTileY = 0;
        for (int tileY = 0; tileY < tileRows; tileY += 1) {
            for (int tileX = 0; tileX < tileColumns; tileX += 1) {
                int score = 0;
                int hotPixels = 0;
                for (int offsetY = -1; offsetY <= 1; offsetY += 1) {
                    int neighborY = tileY + offsetY;
                    if (neighborY < 0 || neighborY >= tileRows) {
                        continue;
                    }
                    for (int offsetX = -1; offsetX <= 1; offsetX += 1) {
                        int neighborX = tileX + offsetX;
                        if (neighborX < 0 || neighborX >= tileColumns) {
                            continue;
                        }
                        int index = neighborY * tileColumns + neighborX;
                        score += tileScores[index];
                        hotPixels += tileHotPixels[index];
                    }
                }
                if (score > bestScore) {
                    bestScore = score;
                    bestHotPixels = hotPixels;
                    bestTileX = tileX;
                    bestTileY = tileY;
                }
            }
        }

        if (phase == Phase.CALIBRATING) {
            long calibrationElapsedNs = nowNs - calibrationStartedNs;
            if (calibrationElapsedNs >= CALIBRATION_WARMUP_NS) {
                addCalibrationScore(bestScore);
            }
            if (calibrationElapsedNs
                    >= CALIBRATION_WARMUP_NS + CALIBRATION_SAMPLE_NS) {
                finishCalibration();
                calibrationComplete = true;
                phase = Phase.WAIT_FIRST_RISE;
            }
            return result(false, bestScore, bestHotPixels);
        }

        boolean risingEdge = bestScore >= effectiveScoreThreshold
                && bestHotPixels >= hotPixelThreshold;
        boolean triggered = false;
        if (risingEdge
                && (lastRiseNs == 0L
                || nowNs - lastRiseNs >= RISE_REFRACTORY_NS)) {
            triggered = acceptRise(bestTileX, bestTileY, nowNs);
        }
        if (phase != Phase.WAIT_FIRST_RISE
                && nowNs - lastRiseNs > RISE_GAP_MAX_NS) {
            resetSequence();
        }
        return result(triggered, bestScore, bestHotPixels);
    }

    private boolean ensureBuffers(
            int[] pixels,
            int frameWidth,
            int frameHeight,
            long nowNs
    ) {
        if (previous != null
                && width == frameWidth
                && height == frameHeight) {
            return false;
        }
        width = frameWidth;
        height = frameHeight;
        previous = new int[pixels.length];
        for (int index = 0; index < pixels.length; index += 1) {
            previous[index] = brightness(pixels[index]);
        }
        tileColumns = (width + TILE_SIZE - 1) / TILE_SIZE;
        tileRows = (height + TILE_SIZE - 1) / TILE_SIZE;
        tileScores = new int[tileColumns * tileRows];
        tileHotPixels = new int[tileColumns * tileRows];
        if (calibrationStartedNs < 0L) {
            calibrationStartedNs = nowNs;
        }
        return true;
    }

    private void addCalibrationScore(int score) {
        if (calibrationCount < calibrationScores.length) {
            calibrationScores[calibrationCount] = score;
            calibrationCount += 1;
        }
    }

    private void finishCalibration() {
        if (calibrationCount == 0) {
            effectiveScoreThreshold = configuredScoreThreshold;
            return;
        }
        int[] sorted = java.util.Arrays.copyOf(
                calibrationScores,
                calibrationCount
        );
        java.util.Arrays.sort(sorted);
        int index = Math.min(
                sorted.length - 1,
                sorted.length * CALIBRATION_PERCENTILE_NUMERATOR
                        / CALIBRATION_PERCENTILE_DENOMINATOR
        );
        int noiseThreshold = sorted[index] * CALIBRATION_MULTIPLIER
                + CALIBRATION_MARGIN;
        effectiveScoreThreshold = Math.max(
                configuredScoreThreshold,
                noiseThreshold
        );
    }

    private boolean acceptRise(int tileX, int tileY, long nowNs) {
        if (phase == Phase.WAIT_FIRST_RISE) {
            candidateTileX = tileX;
            candidateTileY = tileY;
            lastRiseNs = nowNs;
            phase = Phase.WAIT_SECOND_RISE;
            return false;
        }

        long gapNs = nowNs - lastRiseNs;
        if (gapNs < RISE_GAP_MIN_NS) {
            return false;
        }
        if (gapNs > RISE_GAP_MAX_NS
                || tileDistance(
                tileX,
                tileY,
                candidateTileX,
                candidateTileY
        ) > MAX_TILE_DISTANCE) {
            candidateTileX = tileX;
            candidateTileY = tileY;
            lastRiseNs = nowNs;
            phase = Phase.WAIT_SECOND_RISE;
            return false;
        }

        lastRiseNs = nowNs;
        if (phase == Phase.WAIT_SECOND_RISE) {
            phase = Phase.WAIT_THIRD_RISE;
            return false;
        }
        resetSequence();
        return true;
    }

    private Result result(boolean triggered, int score, int hotPixels) {
        return new Result(
                triggered,
                score,
                hotPixels,
                effectiveScoreThreshold,
                phase.name()
        );
    }

    private void resetSequence() {
        lastRiseNs = 0L;
        candidateTileX = -1;
        candidateTileY = -1;
        if (phase != Phase.CALIBRATING) {
            phase = Phase.WAIT_FIRST_RISE;
        }
    }

    private static int tileDistance(
            int x1,
            int y1,
            int x2,
            int y2
    ) {
        return Math.max(Math.abs(x1 - x2), Math.abs(y1 - y2));
    }

    private static int brightness(int argb) {
        int red = (argb >> 16) & 0xff;
        int green = (argb >> 8) & 0xff;
        int blue = argb & 0xff;
        return Math.max(red, Math.max(green, blue));
    }
}
