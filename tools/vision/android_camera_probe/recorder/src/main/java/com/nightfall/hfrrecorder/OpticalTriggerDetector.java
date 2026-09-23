package com.nightfall.hfrrecorder;

/**
 * Decodes the framed visible-LED START/STOP protocol emitted by the F413.
 *
 * <p>A long rise-free preamble and a common SYNC pulse establish a time origin.
 * Five fixed-width payload slots then carry either short (START) or long
 * (STOP) multi-LED pulses.  The complete payload is classified before a
 * token is reported, so no prefix of STOP can be interpreted as START. A
 * missing slot is an erasure and any mixture of short and long votes is
 * rejected. The rising edge of SYNC learns two or preferably three LED
 * locations; later samples need two known locations above their learned OFF
 * baselines, so a dim, compressed, or occluded LED does not discard the token.
 * This class has no Android dependencies so its state machine can be exercised
 * by the host JDK.</p>
 */
final class OpticalTriggerDetector {
    private static final int TILE_SIZE = 8;
    private static final int PIXEL_DELTA_THRESHOLD = 20;
    private static final int ABSOLUTE_BLUE_CHROMA_THRESHOLD = 48;
    private static final long CALIBRATION_WARMUP_NS = 800_000_000L;
    private static final long CALIBRATION_SAMPLE_NS = 1_200_000_000L;
    private static final int CALIBRATION_CAPACITY = 64;
    private static final int CALIBRATION_PERCENTILE_NUMERATOR = 9;
    private static final int CALIBRATION_PERCENTILE_DENOMINATOR = 10;
    private static final int CALIBRATION_MARGIN = 200;
    private static final int CALIBRATION_MULTIPLIER = 3;
    private static final int MAX_ADAPTIVE_THRESHOLD_MULTIPLIER = 5;
    private static final long PREAMBLE_LOW_NS = 1_800_000_000L;
    private static final long LOW_CONFIRM_NS = 175_000_000L;
    private static final long SYNC_HIGH_MIN_NS = 650_000_000L;
    private static final long SYNC_HIGH_MAX_NS = 1_150_000_000L;
    private static final long SYNC_TO_PAYLOAD_NS = 300_000_000L;
    private static final int PAYLOAD_SLOTS = 5;
    private static final long PAYLOAD_SLOT_NS = 1_100_000_000L;
    private static final long EARLY_WINDOW_START_NS = 75_000_000L;
    private static final long EARLY_WINDOW_END_NS = 300_000_000L;
    private static final long LATE_WINDOW_START_NS = 500_000_000L;
    private static final long LATE_WINDOW_END_NS = 725_000_000L;
    private static final int MIN_WINDOW_SAMPLES = 3;
    private static final int MIN_TOKEN_VOTES = 3;
    private static final int MIN_ACQUISITION_COMPONENT_SCORE =
            ABSOLUTE_BLUE_CHROMA_THRESHOLD;
    private static final int MIN_ACQUISITION_COMPONENT_HOT_PIXELS = 1;
    private static final int KNOWN_LED_CONTRAST_ON = 14;
    private static final int KNOWN_LED_CONTRAST_HOLD = 8;
    private static final int MIN_KNOWN_LED_HOT_PIXELS = 1;
    private static final int MAX_COMPONENTS = 12;
    private static final int MIN_LED_SEPARATION_SQUARED = 36;
    private static final int MAX_LED_SEPARATION_SQUARED = 10_000;
    private static final int MIN_TRIANGLE_DOUBLE_AREA = 30;
    private static final int KNOWN_LED_RADIUS = 5;
    private static final int KNOWN_LED_DIAMETER = 2 * KNOWN_LED_RADIUS + 1;
    private static final int KNOWN_LED_BASELINE_PIXELS =
            KNOWN_LED_DIAMETER * KNOWN_LED_DIAMETER;
    private static final int MIN_KNOWN_LED_MATCHES = 2;

    enum TokenType {
        NONE,
        START,
        STOP,
        INVALID
    }

    private enum DecoderState {
        WAIT_PREAMBLE,
        SYNC_HIGH,
        WAIT_PAYLOAD,
        PAYLOAD
    }

    static final class Result {
        final boolean triggered;
        final TokenType tokenType;
        final int score;
        final int hotPixels;
        final int threshold;
        final int matchedLeds;
        final int centerX;
        final int centerY;
        final int classifiedVotes;
        final int requiredVotes;
        final int shortVotes;
        final int longVotes;
        final int erasureVotes;
        final String phase;

        Result(
                TokenType tokenType,
                int score,
                int hotPixels,
                int threshold,
                int matchedLeds,
                int centerX,
                int centerY,
                int classifiedVotes,
                int requiredVotes,
                int shortVotes,
                int longVotes,
                int erasureVotes,
                String phase
        ) {
            this.tokenType = tokenType;
            this.triggered = tokenType == TokenType.START
                    || tokenType == TokenType.STOP;
            this.score = score;
            this.hotPixels = hotPixels;
            this.threshold = threshold;
            this.matchedLeds = matchedLeds;
            this.centerX = centerX;
            this.centerY = centerY;
            this.classifiedVotes = classifiedVotes;
            this.requiredVotes = requiredVotes;
            this.shortVotes = shortVotes;
            this.longVotes = longVotes;
            this.erasureVotes = erasureVotes;
            this.phase = phase;
        }
    }

    private static final class Pulse {
        final int[] x = new int[3];
        final int[] y = new int[3];
        int ledCount;
        int matchedLeds;

        Pulse() {
            java.util.Arrays.fill(x, -1);
            java.util.Arrays.fill(y, -1);
        }
    }

    private final int configuredScoreThreshold;
    private final int hotPixelThreshold;
    private int[] previous;
    private int width;
    private int height;
    private int[] tileScores;
    private int[] tileHotPixels;
    private int[] pixelDeltas;
    private int[] currentBlueChroma;
    private int[] componentQueue;
    private int tileColumns;
    private int tileRows;
    private final int[] componentScores = new int[MAX_COMPONENTS];
    private final int[] componentHotPixels = new int[MAX_COMPONENTS];
    private final int[] componentX = new int[MAX_COMPONENTS];
    private final int[] componentY = new int[MAX_COMPONENTS];
    private final int[] calibrationScores =
            new int[CALIBRATION_CAPACITY];
    private int calibrationCount;
    private long calibrationStartedNs = -1L;
    private long calibrationQuietStartedNs = -1L;
    private boolean calibrationComplete;
    private int effectiveScoreThreshold;
    private DecoderState decoderState = DecoderState.WAIT_PREAMBLE;
    private long lowStartedNs = -1L;
    private long syncStartedNs = -1L;
    private long payloadStartedNs = -1L;
    private int payloadSlot;
    private int earlySamples;
    private int earlyHighSamples;
    private int lateSamples;
    private int lateHighSamples;
    private int shortVotes;
    private int longVotes;
    private int erasureVotes;
    private int tokenScore;
    private int tokenHotPixels;
    private int tokenMatchedLeds;
    private int tokenCenterX = -1;
    private int tokenCenterY = -1;
    private final int[] candidateLedX = new int[3];
    private final int[] candidateLedY = new int[3];
    private final int[][] candidateLedBaseline =
            new int[3][KNOWN_LED_BASELINE_PIXELS];
    private final boolean[] candidateLedOn = new boolean[3];
    private int candidateLedCount;

    OpticalTriggerDetector(int scoreThreshold, int hotPixelThreshold) {
        if (scoreThreshold < 1 || hotPixelThreshold < 1) {
            throw new IllegalArgumentException(
                    "optical trigger thresholds must be positive"
            );
        }
        configuredScoreThreshold = scoreThreshold;
        effectiveScoreThreshold = scoreThreshold;
        this.hotPixelThreshold = hotPixelThreshold;
        resetSequence();
    }

    void reset() {
        previous = null;
        tileScores = null;
        tileHotPixels = null;
        pixelDeltas = null;
        currentBlueChroma = null;
        componentQueue = null;
        width = 0;
        height = 0;
        tileColumns = 0;
        tileRows = 0;
        calibrationCount = 0;
        calibrationStartedNs = -1L;
        calibrationQuietStartedNs = -1L;
        calibrationComplete = false;
        effectiveScoreThreshold = configuredScoreThreshold;
        resetSequence();
    }

    void rearm() {
        previous = null;
        resetSequence();
    }

    Result process(int[] pixels, int frameWidth, int frameHeight, long nowNs) {
        if (pixels == null
                || frameWidth < 1
                || frameHeight < 1
                || pixels.length != frameWidth * frameHeight) {
            throw new IllegalArgumentException("invalid optical trigger frame");
        }
        if (ensureBuffers(pixels, frameWidth, frameHeight, nowNs)) {
            return result(0, 0, null);
        }

        java.util.Arrays.fill(tileScores, 0);
        java.util.Arrays.fill(tileHotPixels, 0);
        for (int y = 0; y < height; y += 1) {
            int tileY = y / TILE_SIZE;
            int rowOffset = y * width;
            for (int x = 0; x < width; x += 1) {
                int index = rowOffset + x;
                int current = blueChroma(pixels[index]);
                currentBlueChroma[index] = current;
                int delta = current - previous[index];
                if (delta >= PIXEL_DELTA_THRESHOLD) {
                    pixelDeltas[index] = delta;
                    int tileIndex = tileY * tileColumns + x / TILE_SIZE;
                    tileScores[tileIndex] += delta;
                    tileHotPixels[tileIndex] += 1;
                } else {
                    pixelDeltas[index] = 0;
                }
            }
        }

        int bestScore = 0;
        int bestHotPixels = 0;
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
                }
            }
        }

        // SYNC acquisition is deliberately based only on newly-risen blue
        // components. The mouse carries a permanent blue centre label, and
        // the surrounding scene may contain other static blue objects; none
        // of those may prevent the rise-free preamble or become an LED anchor.
        // Do not gate this discovery with the adaptive aggregate score. At the
        // overhead endpoints two real LEDs can each occupy one preview pixel,
        // while calibration noise elsewhere can raise that aggregate gate.
        Pulse acquisitionPulse = findAcquisitionPulse(pixelDeltas);

        if (!calibrationComplete) {
            if (acquisitionPulse == null) {
                if (calibrationQuietStartedNs < 0L) {
                    calibrationQuietStartedNs = nowNs;
                }
            } else {
                calibrationQuietStartedNs = -1L;
            }
            long calibrationElapsedNs = nowNs - calibrationStartedNs;
            if (calibrationElapsedNs >= CALIBRATION_WARMUP_NS
                    && acquisitionPulse == null) {
                addCalibrationScore(bestScore);
            }
            if (calibrationElapsedNs
                    >= CALIBRATION_WARMUP_NS + CALIBRATION_SAMPLE_NS) {
                finishCalibration();
                calibrationComplete = true;
                lowStartedNs = calibrationQuietStartedNs;
            }
            Result calibrationResult = result(
                    bestScore,
                    bestHotPixels,
                    acquisitionPulse
            );
            rememberCurrentFrame();
            return calibrationResult;
        }

        Pulse levelPulse = candidateLedCount > 0
                ? findKnownLedPulse(currentBlueChroma)
                : null;
        Result decoded = decodeFrame(
                acquisitionPulse,
                levelPulse,
                nowNs,
                bestScore,
                bestHotPixels
        );
        Result output = decoded != null
                ? decoded
                : result(
                        bestScore,
                        bestHotPixels,
                        candidateLedCount > 0 ? levelPulse : acquisitionPulse
                );
        rememberCurrentFrame();
        return output;
    }

    private Result decodeFrame(
            Pulse acquisitionPulse,
            Pulse levelPulse,
            long nowNs,
            int score,
            int hotPixels
    ) {
        switch (decoderState) {
            case WAIT_PREAMBLE:
                if (acquisitionPulse == null) {
                    if (lowStartedNs < 0L) {
                        lowStartedNs = nowNs;
                    }
                    return null;
                }
                if (lowStartedNs < 0L
                        || nowNs - lowStartedNs < PREAMBLE_LOW_NS) {
                    lowStartedNs = -1L;
                    return null;
                }
                beginSync(acquisitionPulse, nowNs, score, hotPixels);
                return null;

            case SYNC_HIGH:
                if (levelPulse != null) {
                    lowStartedNs = -1L;
                    noteTokenPulse(levelPulse, score, hotPixels);
                    if (nowNs - syncStartedNs
                            > SYNC_HIGH_MAX_NS + LOW_CONFIRM_NS) {
                        resetSequence();
                    }
                    return null;
                }
                if (lowStartedNs < 0L) {
                    lowStartedNs = nowNs;
                    return null;
                }
                if (nowNs - lowStartedNs < LOW_CONFIRM_NS) {
                    return null;
                }
                long syncHighNs = lowStartedNs - syncStartedNs;
                if (syncHighNs < SYNC_HIGH_MIN_NS
                        || syncHighNs > SYNC_HIGH_MAX_NS) {
                    long confirmedLowStartedNs = lowStartedNs;
                    resetSequence();
                    lowStartedNs = confirmedLowStartedNs;
                    return null;
                }
                payloadStartedNs = lowStartedNs + SYNC_TO_PAYLOAD_NS;
                lowStartedNs = -1L;
                decoderState = DecoderState.WAIT_PAYLOAD;
                return null;

            case WAIT_PAYLOAD:
                if (nowNs < payloadStartedNs) {
                    return null;
                }
                decoderState = DecoderState.PAYLOAD;
                // Fall through so the first payload sample is retained.

            case PAYLOAD:
                noteTokenPulse(levelPulse, score, hotPixels);
                return decodePayloadFrame(levelPulse != null, nowNs);

            default:
                throw new IllegalStateException("unknown optical decoder state");
        }
    }

    private void beginSync(
            Pulse pulse,
            long nowNs,
            int score,
            int hotPixels
    ) {
        resetTokenMeasurements();
        rememberCandidate(pulse);
        tokenCenterX = pulseCenterX(pulse);
        tokenCenterY = pulseCenterY(pulse);
        noteTokenPulse(pulse, score, hotPixels);
        syncStartedNs = nowNs;
        lowStartedNs = -1L;
        decoderState = DecoderState.SYNC_HIGH;
    }

    private void noteTokenPulse(Pulse pulse, int score, int hotPixels) {
        if (pulse == null) {
            return;
        }
        tokenScore = Math.max(tokenScore, score);
        tokenHotPixels = Math.max(tokenHotPixels, hotPixels);
        tokenMatchedLeds = tokenMatchedLeds == 0
                ? pulse.matchedLeds
                : Math.min(tokenMatchedLeds, pulse.matchedLeds);
    }

    private Result decodePayloadFrame(boolean high, long nowNs) {
        long elapsedNs = nowNs - payloadStartedNs;
        if (elapsedNs < 0L) {
            return null;
        }
        while (payloadSlot < PAYLOAD_SLOTS
                && elapsedNs >= (payloadSlot + 1L) * PAYLOAD_SLOT_NS) {
            classifyPayloadSlot();
            payloadSlot += 1;
            resetSlotMeasurements();
        }
        if (payloadSlot >= PAYLOAD_SLOTS) {
            return completeToken();
        }

        long slotOffsetNs = elapsedNs - payloadSlot * PAYLOAD_SLOT_NS;
        if (slotOffsetNs >= EARLY_WINDOW_START_NS
                && slotOffsetNs < EARLY_WINDOW_END_NS) {
            earlySamples += 1;
            if (high) {
                earlyHighSamples += 1;
            }
        }
        if (slotOffsetNs >= LATE_WINDOW_START_NS
                && slotOffsetNs < LATE_WINDOW_END_NS) {
            lateSamples += 1;
            if (high) {
                lateHighSamples += 1;
            }
        }
        return null;
    }

    private void classifyPayloadSlot() {
        boolean earlyOn = isWindowOn(earlyHighSamples, earlySamples);
        boolean lateOn = isWindowOn(lateHighSamples, lateSamples);
        boolean lateOff = lateSamples >= MIN_WINDOW_SAMPLES
                && lateHighSamples * 4 <= lateSamples;
        if (earlyOn && lateOff) {
            shortVotes += 1;
        } else if (earlyOn && lateOn) {
            longVotes += 1;
        } else {
            erasureVotes += 1;
        }
    }

    private static boolean isWindowOn(int highSamples, int samples) {
        return samples >= MIN_WINDOW_SAMPLES
                && highSamples >= 2
                && highSamples * 2 >= samples;
    }

    private Result completeToken() {
        TokenType tokenType = TokenType.INVALID;
        if (shortVotes >= MIN_TOKEN_VOTES && longVotes == 0) {
            tokenType = TokenType.START;
        } else if (longVotes >= MIN_TOKEN_VOTES && shortVotes == 0) {
            tokenType = TokenType.STOP;
        }
        Result completed = new Result(
                tokenType,
                tokenScore,
                tokenHotPixels,
                effectiveScoreThreshold,
                tokenMatchedLeds,
                tokenCenterX,
                tokenCenterY,
                shortVotes + longVotes,
                MIN_TOKEN_VOTES,
                shortVotes,
                longVotes,
                erasureVotes,
                "TOKEN_" + tokenType.name()
        );
        resetSequence();
        return completed;
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
            previous[index] = blueChroma(pixels[index]);
        }
        tileColumns = (width + TILE_SIZE - 1) / TILE_SIZE;
        tileRows = (height + TILE_SIZE - 1) / TILE_SIZE;
        tileScores = new int[tileColumns * tileRows];
        tileHotPixels = new int[tileColumns * tileRows];
        pixelDeltas = new int[pixels.length];
        currentBlueChroma = new int[pixels.length];
        componentQueue = new int[pixels.length];
        if (calibrationStartedNs < 0L) {
            calibrationStartedNs = nowNs;
        }
        return true;
    }

    private void rememberCurrentFrame() {
        System.arraycopy(
                currentBlueChroma,
                0,
                previous,
                0,
                currentBlueChroma.length
        );
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
        int maximumThreshold = (int) Math.min(
                Integer.MAX_VALUE,
                (long) configuredScoreThreshold
                        * MAX_ADAPTIVE_THRESHOLD_MULTIPLIER
        );
        effectiveScoreThreshold = Math.max(
                configuredScoreThreshold,
                Math.min(noiseThreshold, maximumThreshold)
        );
    }

    private Pulse findAcquisitionPulse(int[] componentPixels) {
        int componentCount = 0;
        java.util.Arrays.fill(componentScores, 0);
        java.util.Arrays.fill(componentHotPixels, 0);
        for (int seed = 0; seed < componentPixels.length; seed += 1) {
            if (componentPixels[seed] == 0) {
                continue;
            }
            int queueRead = 0;
            int queueWrite = 0;
            int score = 0;
            int hotPixels = 0;
            int sumX = 0;
            int sumY = 0;
            componentPixels[seed] = -componentPixels[seed];
            componentQueue[queueWrite++] = seed;
            while (queueRead < queueWrite) {
                int index = componentQueue[queueRead++];
                int delta = -componentPixels[index];
                if (delta <= 0) {
                    continue;
                }
                componentPixels[index] = 0;
                int x = index % width;
                int y = index / width;
                score += delta;
                hotPixels += 1;
                sumX += x;
                sumY += y;
                int minX = Math.max(0, x - 1);
                int maxX = Math.min(width - 1, x + 1);
                int minY = Math.max(0, y - 1);
                int maxY = Math.min(height - 1, y + 1);
                for (int neighborY = minY; neighborY <= maxY; neighborY += 1) {
                    int rowOffset = neighborY * width;
                    for (int neighborX = minX; neighborX <= maxX; neighborX += 1) {
                        int neighbor = rowOffset + neighborX;
                        if (componentPixels[neighbor] > 0) {
                            componentQueue[queueWrite++] = neighbor;
                            componentPixels[neighbor] =
                                    -componentPixels[neighbor];
                        }
                    }
                }
            }
            if (score < MIN_ACQUISITION_COMPONENT_SCORE
                    || hotPixels
                    < MIN_ACQUISITION_COMPONENT_HOT_PIXELS) {
                continue;
            }
            componentCount = storeComponent(
                    componentCount,
                    score,
                    hotPixels,
                    (sumX + hotPixels / 2) / hotPixels,
                    (sumY + hotPixels / 2) / hotPixels
            );
        }

        // Prefer the full three-LED triangle whenever the preview resolves it.
        // Fall back to a separated pair because one LED is frequently hidden
        // or merged into a neighbour near the edge of the overhead frame.
        Pulse best = null;
        int bestScore = -1;
        for (int first = 0; first < componentCount - 2; first += 1) {
            for (int second = first + 1; second < componentCount - 1; second += 1) {
                for (int third = second + 1; third < componentCount; third += 1) {
                    if (!isLedTriangle(first, second, third)) {
                        continue;
                    }
                    int score = componentScores[first]
                            + componentScores[second]
                            + componentScores[third];
                    int hotPixels = componentHotPixels[first]
                            + componentHotPixels[second]
                            + componentHotPixels[third];
                    if (score < configuredScoreThreshold
                            || hotPixels < hotPixelThreshold) {
                        continue;
                    }
                    if (score <= bestScore) {
                        continue;
                    }
                    bestScore = score;
                    best = pulseFromComponents(first, second, third);
                }
            }
        }
        if (best != null) {
            return best;
        }

        for (int first = 0; first < componentCount - 1; first += 1) {
            for (int second = first + 1;
                    second < componentCount;
                    second += 1) {
                if (!isLedPair(first, second)) {
                    continue;
                }
                int score = componentScores[first] + componentScores[second];
                int hotPixels = componentHotPixels[first]
                        + componentHotPixels[second];
                if (score < configuredScoreThreshold
                        || hotPixels < hotPixelThreshold) {
                    continue;
                }
                if (score <= bestScore) {
                    continue;
                }
                bestScore = score;
                best = pulseFromComponents(first, second);
            }
        }
        return best;
    }

    private Pulse pulseFromComponents(int... indexes) {
        Pulse pulse = new Pulse();
        pulse.ledCount = indexes.length;
        pulse.matchedLeds = indexes.length;
        for (int led = 0; led < indexes.length; led += 1) {
            pulse.x[led] = componentX[indexes[led]];
            pulse.y[led] = componentY[indexes[led]];
        }
        return pulse;
    }

    private int storeComponent(
            int componentCount,
            int score,
            int hotPixels,
            int x,
            int y
    ) {
        if (componentCount == MAX_COMPONENTS
                && score <= componentScores[MAX_COMPONENTS - 1]) {
            return componentCount;
        }
        int insertAt = Math.min(componentCount, MAX_COMPONENTS - 1);
        while (insertAt > 0 && componentScores[insertAt - 1] < score) {
            if (insertAt < MAX_COMPONENTS) {
                componentScores[insertAt] = componentScores[insertAt - 1];
                componentHotPixels[insertAt] =
                        componentHotPixels[insertAt - 1];
                componentX[insertAt] = componentX[insertAt - 1];
                componentY[insertAt] = componentY[insertAt - 1];
            }
            insertAt -= 1;
        }
        if (insertAt < MAX_COMPONENTS) {
            componentScores[insertAt] = score;
            componentHotPixels[insertAt] = hotPixels;
            componentX[insertAt] = x;
            componentY[insertAt] = y;
        }
        return Math.min(MAX_COMPONENTS, componentCount + 1);
    }

    private Pulse findKnownLedPulse(int[] currentPixels) {
        Pulse pulse = new Pulse();
        int matched = 0;
        for (int led = 0; led < candidateLedCount; led += 1) {
            int minX = Math.max(0, candidateLedX[led] - KNOWN_LED_RADIUS);
            int maxX = Math.min(width - 1,
                    candidateLedX[led] + KNOWN_LED_RADIUS);
            int minY = Math.max(0, candidateLedY[led] - KNOWN_LED_RADIUS);
            int maxY = Math.min(height - 1,
                    candidateLedY[led] + KNOWN_LED_RADIUS);
            int score = 0;
            int hotPixels = 0;
            int sumX = 0;
            int sumY = 0;
            for (int y = minY; y <= maxY; y += 1) {
                int row = y * width;
                for (int x = minX; x <= maxX; x += 1) {
                    int baselineIndex = (y - candidateLedY[led]
                            + KNOWN_LED_RADIUS) * KNOWN_LED_DIAMETER
                            + x - candidateLedX[led]
                            + KNOWN_LED_RADIUS;
                    int contrast = currentPixels[row + x]
                            - candidateLedBaseline[led][baselineIndex];
                    int contrastThreshold = candidateLedOn[led]
                            ? KNOWN_LED_CONTRAST_HOLD
                            : KNOWN_LED_CONTRAST_ON;
                    if (contrast < contrastThreshold) {
                        continue;
                    }
                    score += contrast;
                    hotPixels += 1;
                    sumX += x;
                    sumY += y;
                }
            }
            // Once a rising edge has fixed the LED coordinates, sustain the
            // pulse by contrast from that coordinate's OFF snapshot rather
            // than by absolute blue chroma. Real endpoint LEDs can peak below
            // the old absolute floor while still being 20+ chroma above OFF.
            // Hysteresis keeps a dim one-pixel LED stable through compression.
            if (hotPixels < MIN_KNOWN_LED_HOT_PIXELS) {
                candidateLedOn[led] = false;
                continue;
            }
            candidateLedOn[led] = true;
            pulse.x[matched] = (sumX + hotPixels / 2) / hotPixels;
            pulse.y[matched] = (sumY + hotPixels / 2) / hotPixels;
            matched += 1;
        }
        if (matched < MIN_KNOWN_LED_MATCHES) {
            return null;
        }
        boolean separatedPair = false;
        for (int first = 0; first < matched - 1; first += 1) {
            for (int second = first + 1; second < matched; second += 1) {
                int deltaX = pulse.x[first] - pulse.x[second];
                int deltaY = pulse.y[first] - pulse.y[second];
                if (deltaX * deltaX + deltaY * deltaY
                        >= MIN_LED_SEPARATION_SQUARED) {
                    separatedPair = true;
                }
            }
        }
        if (!separatedPair) {
            return null;
        }
        pulse.ledCount = matched;
        pulse.matchedLeds = matched;
        return pulse;
    }

    private boolean isLedPair(int first, int second) {
        int separation = distanceSquared(first, second);
        return separation >= MIN_LED_SEPARATION_SQUARED
                && separation <= MAX_LED_SEPARATION_SQUARED;
    }

    private boolean isLedTriangle(int first, int second, int third) {
        int firstSecond = distanceSquared(first, second);
        int firstThird = distanceSquared(first, third);
        int secondThird = distanceSquared(second, third);
        if (firstSecond < MIN_LED_SEPARATION_SQUARED
                || firstThird < MIN_LED_SEPARATION_SQUARED
                || secondThird < MIN_LED_SEPARATION_SQUARED
                || firstSecond > MAX_LED_SEPARATION_SQUARED
                || firstThird > MAX_LED_SEPARATION_SQUARED
                || secondThird > MAX_LED_SEPARATION_SQUARED) {
            return false;
        }
        int doubleArea = Math.abs(
                (componentX[second] - componentX[first])
                        * (componentY[third] - componentY[first])
                        - (componentY[second] - componentY[first])
                        * (componentX[third] - componentX[first])
        );
        return doubleArea >= MIN_TRIANGLE_DOUBLE_AREA;
    }

    private int distanceSquared(int first, int second) {
        int deltaX = componentX[first] - componentX[second];
        int deltaY = componentY[first] - componentY[second];
        return deltaX * deltaX + deltaY * deltaY;
    }

    private void rememberCandidate(Pulse pulse) {
        java.util.Arrays.fill(candidateLedX, -1);
        java.util.Arrays.fill(candidateLedY, -1);
        candidateLedCount = pulse.ledCount;
        System.arraycopy(pulse.x, 0, candidateLedX, 0, pulse.ledCount);
        System.arraycopy(pulse.y, 0, candidateLedY, 0, pulse.ledCount);
        java.util.Arrays.fill(candidateLedOn, false);
        for (int led = 0; led < candidateLedCount; led += 1) {
            candidateLedOn[led] = true;
            java.util.Arrays.fill(candidateLedBaseline[led], 0);
            for (int offsetY = -KNOWN_LED_RADIUS;
                    offsetY <= KNOWN_LED_RADIUS;
                    offsetY += 1) {
                int y = candidateLedY[led] + offsetY;
                if (y < 0 || y >= height) {
                    continue;
                }
                int row = y * width;
                for (int offsetX = -KNOWN_LED_RADIUS;
                        offsetX <= KNOWN_LED_RADIUS;
                        offsetX += 1) {
                    int x = candidateLedX[led] + offsetX;
                    if (x < 0 || x >= width) {
                        continue;
                    }
                    int baselineIndex = (offsetY + KNOWN_LED_RADIUS)
                            * KNOWN_LED_DIAMETER
                            + offsetX + KNOWN_LED_RADIUS;
                    candidateLedBaseline[led][baselineIndex] =
                            previous[row + x];
                }
            }
        }
    }

    private Result result(
            int score,
            int hotPixels,
            Pulse pulse
    ) {
        int centerX = -1;
        int centerY = -1;
        if (pulse != null) {
            centerX = pulseCenterX(pulse);
            centerY = pulseCenterY(pulse);
        }
        return new Result(
                TokenType.NONE,
                score,
                hotPixels,
                effectiveScoreThreshold,
                pulse == null ? 0 : pulse.matchedLeds,
                centerX,
                centerY,
                shortVotes + longVotes,
                MIN_TOKEN_VOTES,
                shortVotes,
                longVotes,
                erasureVotes,
                phaseName()
        );
    }

    private String phaseName() {
        if (!calibrationComplete) {
            return "CALIBRATING";
        }
        switch (decoderState) {
            case WAIT_PREAMBLE:
                return "WAIT_PREAMBLE";
            case SYNC_HIGH:
                return "SYNC_HIGH";
            case WAIT_PAYLOAD:
                return "WAIT_PAYLOAD";
            case PAYLOAD:
                return String.format(
                        "PAYLOAD_%d_OF_%d",
                        payloadSlot + 1,
                        PAYLOAD_SLOTS
                );
            default:
                return "UNKNOWN";
        }
    }

    private void resetSequence() {
        decoderState = DecoderState.WAIT_PREAMBLE;
        lowStartedNs = -1L;
        syncStartedNs = -1L;
        payloadStartedNs = -1L;
        resetTokenMeasurements();
        java.util.Arrays.fill(candidateLedX, -1);
        java.util.Arrays.fill(candidateLedY, -1);
        java.util.Arrays.fill(candidateLedOn, false);
        candidateLedCount = 0;
    }

    private static int pulseCenterX(Pulse pulse) {
        int sum = 0;
        for (int led = 0; led < pulse.ledCount; led += 1) {
            sum += pulse.x[led];
        }
        return (sum + pulse.ledCount / 2) / pulse.ledCount;
    }

    private static int pulseCenterY(Pulse pulse) {
        int sum = 0;
        for (int led = 0; led < pulse.ledCount; led += 1) {
            sum += pulse.y[led];
        }
        return (sum + pulse.ledCount / 2) / pulse.ledCount;
    }

    private void resetTokenMeasurements() {
        payloadSlot = 0;
        resetSlotMeasurements();
        shortVotes = 0;
        longVotes = 0;
        erasureVotes = 0;
        tokenScore = 0;
        tokenHotPixels = 0;
        tokenMatchedLeds = 0;
        tokenCenterX = -1;
        tokenCenterY = -1;
    }

    private void resetSlotMeasurements() {
        earlySamples = 0;
        earlyHighSamples = 0;
        lateSamples = 0;
        lateHighSamples = 0;
    }

    private static int blueChroma(int argb) {
        int red = (argb >> 16) & 0xff;
        int green = (argb >> 8) & 0xff;
        int blue = argb & 0xff;
        return Math.max(0, blue - Math.max(red, green));
    }
}
