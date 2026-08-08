package com.nightfall.hfrrecorder;

/**
 * Decodes the framed visible-LED START/STOP protocol emitted by the F413.
 *
 * <p>A long all-off preamble and a common SYNC pulse establish a time origin.
 * Five fixed-width payload slots then carry either short (START) or long
 * (STOP) all-three-LED pulses.  The complete payload is classified before a
 * token is reported, so no prefix of STOP can be interpreted as START. A
 * missing slot is an erasure and any mixture of short and long votes is
 * rejected. The SYNC pulse learns the three LED locations; payload slots may
 * retain two of the three so a dim or compressed LED does not discard the
 * token. This class has no Android dependencies so its state machine can be
 * exercised by the host JDK.</p>
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
    private static final int MIN_COMPONENT_SCORE = 60;
    private static final int MIN_COMPONENT_HOT_PIXELS = 2;
    private static final int MAX_COMPONENTS = 12;
    private static final int MIN_LED_SEPARATION_SQUARED = 36;
    private static final int MAX_LED_SEPARATION_SQUARED = 10_000;
    private static final int MIN_TRIANGLE_DOUBLE_AREA = 30;
    private static final int KNOWN_LED_RADIUS = 5;
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
        int matchedLeds = 3;
    }

    private final int configuredScoreThreshold;
    private final int hotPixelThreshold;
    private int[] previous;
    private int width;
    private int height;
    private int[] tileScores;
    private int[] tileHotPixels;
    private int[] pixelDeltas;
    private int[] absoluteBlueChroma;
    private int[] componentQueue;
    private int tileColumns;
    private int tileRows;
    private final int[] componentScores = new int[MAX_COMPONENTS];
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
        absoluteBlueChroma = null;
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
                absoluteBlueChroma[index] = current
                        >= ABSOLUTE_BLUE_CHROMA_THRESHOLD
                        ? current
                        : 0;
                int delta = current - previous[index];
                if (delta >= PIXEL_DELTA_THRESHOLD) {
                    pixelDeltas[index] = delta;
                    int tileIndex = tileY * tileColumns + x / TILE_SIZE;
                    tileScores[tileIndex] += delta;
                    tileHotPixels[tileIndex] += 1;
                } else {
                    pixelDeltas[index] = 0;
                }
                previous[index] = current;
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

        if (!calibrationComplete) {
            Pulse calibrationPulse = findThreeLedPulse(
                    absoluteBlueChroma,
                    absoluteComponentScoreThreshold()
            );
            if (calibrationPulse == null) {
                if (calibrationQuietStartedNs < 0L) {
                    calibrationQuietStartedNs = nowNs;
                }
            } else {
                calibrationQuietStartedNs = -1L;
            }
            long calibrationElapsedNs = nowNs - calibrationStartedNs;
            if (calibrationElapsedNs >= CALIBRATION_WARMUP_NS) {
                addCalibrationScore(bestScore);
            }
            if (calibrationElapsedNs
                    >= CALIBRATION_WARMUP_NS + CALIBRATION_SAMPLE_NS) {
                finishCalibration();
                calibrationComplete = true;
                lowStartedNs = calibrationQuietStartedNs;
            }
            return result(bestScore, bestHotPixels, calibrationPulse);
        }

        boolean risingEdge = bestScore >= effectiveScoreThreshold
                && bestHotPixels >= hotPixelThreshold;
        Pulse deltaPulse = null;
        if (risingEdge) {
            deltaPulse = findThreeLedPulse(
                    pixelDeltas,
                    Math.max(
                            MIN_COMPONENT_SCORE,
                            effectiveScoreThreshold / 3
                    )
            );
        }
        boolean candidateKnown = candidateLedX[0] >= 0;
        Pulse levelPulse = candidateKnown
                ? findKnownLedPulse(absoluteBlueChroma)
                : findThreeLedPulse(
                        absoluteBlueChroma,
                        absoluteComponentScoreThreshold()
                );
        Pulse pulse = levelPulse != null
                ? levelPulse
                : (!candidateKnown ? deltaPulse : null);
        Result decoded = decodeFrame(
                pulse,
                nowNs,
                bestScore,
                bestHotPixels
        );
        return decoded != null
                ? decoded
                : result(bestScore, bestHotPixels, pulse);
    }

    private Result decodeFrame(
            Pulse pulse,
            long nowNs,
            int score,
            int hotPixels
    ) {
        switch (decoderState) {
            case WAIT_PREAMBLE:
                if (pulse == null) {
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
                beginSync(pulse, nowNs, score, hotPixels);
                return null;

            case SYNC_HIGH:
                if (pulse != null) {
                    lowStartedNs = -1L;
                    noteTokenPulse(pulse, score, hotPixels);
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
                noteTokenPulse(pulse, score, hotPixels);
                return decodePayloadFrame(pulse != null, nowNs);

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
        tokenCenterX = (pulse.x[0] + pulse.x[1] + pulse.x[2] + 1) / 3;
        tokenCenterY = (pulse.y[0] + pulse.y[1] + pulse.y[2] + 1) / 3;
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
        absoluteBlueChroma = new int[pixels.length];
        componentQueue = new int[pixels.length];
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

    private int absoluteComponentScoreThreshold() {
        return Math.max(
                MIN_COMPONENT_SCORE,
                configuredScoreThreshold / 3
        );
    }

    private Pulse findThreeLedPulse(
            int[] componentPixels,
            int componentScoreThreshold
    ) {
        int componentCount = 0;
        java.util.Arrays.fill(componentScores, 0);
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
            if (score < componentScoreThreshold
                    || hotPixels < MIN_COMPONENT_HOT_PIXELS) {
                continue;
            }
            componentCount = storeComponent(
                    componentCount,
                    score,
                    (sumX + hotPixels / 2) / hotPixels,
                    (sumY + hotPixels / 2) / hotPixels
            );
        }

        Pulse best = null;
        int bestScore = 0;
        for (int first = 0; first < componentCount - 2; first += 1) {
            for (int second = first + 1; second < componentCount - 1; second += 1) {
                for (int third = second + 1; third < componentCount; third += 1) {
                    if (!isLedTriangle(first, second, third)) {
                        continue;
                    }
                    int score = componentScores[first]
                            + componentScores[second]
                            + componentScores[third];
                    if (score <= bestScore) {
                        continue;
                    }
                    bestScore = score;
                    best = new Pulse();
                    int[] indexes = {first, second, third};
                    for (int led = 0; led < indexes.length; led += 1) {
                        best.x[led] = componentX[indexes[led]];
                        best.y[led] = componentY[indexes[led]];
                    }
                }
            }
        }
        return best;
    }

    private int storeComponent(
            int componentCount,
            int score,
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
                componentX[insertAt] = componentX[insertAt - 1];
                componentY[insertAt] = componentY[insertAt - 1];
            }
            insertAt -= 1;
        }
        if (insertAt < MAX_COMPONENTS) {
            componentScores[insertAt] = score;
            componentX[insertAt] = x;
            componentY[insertAt] = y;
        }
        return Math.min(MAX_COMPONENTS, componentCount + 1);
    }

    private Pulse findKnownLedPulse(int[] componentPixels) {
        Pulse pulse = new Pulse();
        boolean[] matchedKnown = new boolean[candidateLedX.length];
        int matched = 0;
        int componentScoreThreshold = absoluteComponentScoreThreshold();
        for (int led = 0; led < candidateLedX.length; led += 1) {
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
                    int value = componentPixels[row + x];
                    if (value <= 0) {
                        continue;
                    }
                    score += value;
                    hotPixels += 1;
                    sumX += x;
                    sumY += y;
                }
            }
            if (score < componentScoreThreshold
                    || hotPixels < MIN_COMPONENT_HOT_PIXELS) {
                pulse.x[led] = candidateLedX[led];
                pulse.y[led] = candidateLedY[led];
                continue;
            }
            pulse.x[led] = (sumX + hotPixels / 2) / hotPixels;
            pulse.y[led] = (sumY + hotPixels / 2) / hotPixels;
            matchedKnown[led] = true;
            matched += 1;
        }
        if (matched < MIN_KNOWN_LED_MATCHES) {
            return null;
        }
        boolean separatedPair = false;
        for (int first = 0; first < matchedKnown.length - 1; first += 1) {
            if (!matchedKnown[first]) {
                continue;
            }
            for (int second = first + 1;
                    second < matchedKnown.length;
                    second += 1) {
                if (!matchedKnown[second]) {
                    continue;
                }
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
        pulse.matchedLeds = matched;
        return pulse;
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
        System.arraycopy(pulse.x, 0, candidateLedX, 0, candidateLedX.length);
        System.arraycopy(pulse.y, 0, candidateLedY, 0, candidateLedY.length);
    }

    private Result result(
            int score,
            int hotPixels,
            Pulse pulse
    ) {
        int centerX = -1;
        int centerY = -1;
        if (pulse != null) {
            centerX = (pulse.x[0] + pulse.x[1] + pulse.x[2] + 1) / 3;
            centerY = (pulse.y[0] + pulse.y[1] + pulse.y[2] + 1) / 3;
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
