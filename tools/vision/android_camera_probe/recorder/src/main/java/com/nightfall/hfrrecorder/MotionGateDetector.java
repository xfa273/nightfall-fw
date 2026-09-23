package com.nightfall.hfrrecorder;

/**
 * Records when the micromouse leaves its START appearance.
 *
 * <p>The START LED triangle supplies a local image anchor.  After the final
 * START flash settles, this detector averages a small baseline around that
 * anchor and looks for a sustained, spatially local appearance change. Blue
 * LED pixels and blue-dominant changes are excluded, so ordinary status-LED
 * activity does not become motion telemetry. This detector no longer gates
 * STOP decoding; a missed motion observation cannot prevent video saving. It
 * has no Android dependencies and is host tested.</p>
 */
final class MotionGateDetector {
    private static final int REGION_HALF_SIZE = 48;
    private static final int SETTLE_FRAMES = 16;
    private static final int BASELINE_FRAMES = 8;
    private static final int PIXEL_DELTA_THRESHOLD = 18;
    private static final int BLUE_CHROMA_THRESHOLD = 10;
    private static final int BLUE_DELTA_MARGIN = 8;
    private static final int MIN_CHANGED_PIXELS = 120;
    private static final int MOTION_CONFIRM_FRAMES = 3;

    static final class Result {
        final boolean motionDetected;
        final boolean baselineReady;
        final int changedPixels;
        final String phase;

        Result(
                boolean motionDetected,
                boolean baselineReady,
                int changedPixels,
                String phase
        ) {
            this.motionDetected = motionDetected;
            this.baselineReady = baselineReady;
            this.changedPixels = changedPixels;
            this.phase = phase;
        }
    }

    private boolean armed;
    private int anchorX;
    private int anchorY;
    private int frameWidth;
    private int frameHeight;
    private int regionMinX;
    private int regionMaxX;
    private int regionMinY;
    private int regionMaxY;
    private int regionWidth;
    private int regionPixels;
    private int processedFrames;
    private int baselineFrames;
    private int[] baselineRed;
    private int[] baselineGreen;
    private int[] baselineBlue;
    private int motionFrames;
    private boolean detected;

    void arm(int valueX, int valueY) {
        if (valueX < 0 || valueY < 0) {
            throw new IllegalArgumentException("invalid motion-gate anchor");
        }
        armed = true;
        anchorX = valueX;
        anchorY = valueY;
        frameWidth = 0;
        frameHeight = 0;
        processedFrames = 0;
        baselineFrames = 0;
        baselineRed = null;
        baselineGreen = null;
        baselineBlue = null;
        motionFrames = 0;
        detected = false;
    }

    Result process(int[] pixels, int width, int height) {
        if (!armed) {
            throw new IllegalStateException("motion gate is not armed");
        }
        if (pixels == null
                || width < 1
                || height < 1
                || pixels.length != width * height) {
            throw new IllegalArgumentException("invalid motion-gate frame");
        }
        if (frameWidth == 0) {
            initializeRegion(width, height);
        } else if (frameWidth != width || frameHeight != height) {
            throw new IllegalArgumentException("motion-gate frame size changed");
        }

        processedFrames += 1;
        if (processedFrames <= SETTLE_FRAMES) {
            return new Result(false, false, 0, "SETTLE_START");
        }
        if (baselineFrames < BASELINE_FRAMES) {
            accumulateBaseline(pixels);
            baselineFrames += 1;
            if (baselineFrames == BASELINE_FRAMES) {
                finishBaseline();
            }
            return new Result(
                    false,
                    baselineFrames == BASELINE_FRAMES,
                    0,
                    baselineFrames == BASELINE_FRAMES
                            ? "WAIT_MOTION"
                            : "ACQUIRE_BASELINE"
            );
        }

        int changedPixels = countChangedPixels(pixels);
        int maximumLocalChange = regionPixels / 2;
        if (changedPixels >= MIN_CHANGED_PIXELS
                && changedPixels <= maximumLocalChange) {
            motionFrames += 1;
            if (motionFrames >= MOTION_CONFIRM_FRAMES) {
                detected = true;
            }
        } else {
            motionFrames = 0;
        }
        return new Result(
                detected,
                true,
                changedPixels,
                detected ? "MOTION_DETECTED" : "WAIT_MOTION"
        );
    }

    private void initializeRegion(int width, int height) {
        frameWidth = width;
        frameHeight = height;
        regionMinX = Math.max(0, anchorX - REGION_HALF_SIZE);
        regionMaxX = Math.min(width - 1, anchorX + REGION_HALF_SIZE);
        regionMinY = Math.max(0, anchorY - REGION_HALF_SIZE);
        regionMaxY = Math.min(height - 1, anchorY + REGION_HALF_SIZE);
        regionWidth = regionMaxX - regionMinX + 1;
        regionPixels = regionWidth * (regionMaxY - regionMinY + 1);
        baselineRed = new int[regionPixels];
        baselineGreen = new int[regionPixels];
        baselineBlue = new int[regionPixels];
    }

    private void accumulateBaseline(int[] pixels) {
        int destination = 0;
        for (int y = regionMinY; y <= regionMaxY; y += 1) {
            int row = y * frameWidth;
            for (int x = regionMinX; x <= regionMaxX; x += 1) {
                int argb = pixels[row + x];
                baselineRed[destination] += (argb >> 16) & 0xff;
                baselineGreen[destination] += (argb >> 8) & 0xff;
                baselineBlue[destination] += argb & 0xff;
                destination += 1;
            }
        }
    }

    private void finishBaseline() {
        for (int index = 0; index < regionPixels; index += 1) {
            baselineRed[index] = roundedAverage(baselineRed[index]);
            baselineGreen[index] = roundedAverage(baselineGreen[index]);
            baselineBlue[index] = roundedAverage(baselineBlue[index]);
        }
    }

    private int roundedAverage(int sum) {
        return (sum + BASELINE_FRAMES / 2) / BASELINE_FRAMES;
    }

    private int countChangedPixels(int[] pixels) {
        int changed = 0;
        int source = regionMinY * frameWidth + regionMinX;
        int baseline = 0;
        for (int y = regionMinY; y <= regionMaxY; y += 1) {
            for (int x = 0; x < regionWidth; x += 1) {
                int argb = pixels[source + x];
                int red = (argb >> 16) & 0xff;
                int green = (argb >> 8) & 0xff;
                int blue = argb & 0xff;
                int redDelta = Math.abs(red - baselineRed[baseline]);
                int greenDelta = Math.abs(
                        green - baselineGreen[baseline]
                );
                int blueDelta = Math.abs(blue - baselineBlue[baseline]);
                boolean baselineBlueChroma = isBlueChroma(
                        baselineRed[baseline],
                        baselineGreen[baseline],
                        baselineBlue[baseline]
                );
                boolean currentBlueChroma = isBlueChroma(
                        red,
                        green,
                        blue
                );
                boolean blueDominantDelta = blueDelta
                        >= redDelta + BLUE_DELTA_MARGIN
                        && blueDelta >= greenDelta + BLUE_DELTA_MARGIN;
                if (!baselineBlueChroma
                        && !currentBlueChroma
                        && !blueDominantDelta
                        && Math.max(
                        redDelta,
                        Math.max(greenDelta, blueDelta)
                ) >= PIXEL_DELTA_THRESHOLD) {
                    changed += 1;
                }
                baseline += 1;
            }
            source += frameWidth;
        }
        return changed;
    }

    private boolean isBlueChroma(int red, int green, int blue) {
        return blue - Math.max(red, green) >= BLUE_CHROMA_THRESHOLD;
    }
}
