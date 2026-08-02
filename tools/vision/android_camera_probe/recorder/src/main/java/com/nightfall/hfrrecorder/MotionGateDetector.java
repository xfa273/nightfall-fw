package com.nightfall.hfrrecorder;

/**
 * Opens the STOP decoder only after the green micromouse body has moved.
 *
 * <p>The detector deliberately fails closed: loss of the green target keeps
 * recording until the configured maximum duration instead of accepting an
 * LED-only event as STOP. It has no Android dependencies and is host tested.</p>
 */
final class MotionGateDetector {
    private static final int SEARCH_HALF_SIZE = 80;
    private static final int MIN_GREEN_CHROMA = 18;
    private static final int MIN_GREEN_VALUE = 40;
    private static final int MIN_GREEN_PIXELS = 12;
    private static final int BASELINE_FRAMES = 5;
    private static final int MOTION_DISTANCE_SQUARED = 36;
    private static final int MOTION_CONFIRM_FRAMES = 3;

    static final class Result {
        final boolean motionDetected;
        final boolean targetVisible;
        final double displacementPx;
        final int targetPixels;
        final String phase;

        Result(
                boolean motionDetected,
                boolean targetVisible,
                double displacementPx,
                int targetPixels,
                String phase
        ) {
            this.motionDetected = motionDetected;
            this.targetVisible = targetVisible;
            this.displacementPx = displacementPx;
            this.targetPixels = targetPixels;
            this.phase = phase;
        }
    }

    private boolean armed;
    private int searchX;
    private int searchY;
    private long baselineSumX;
    private long baselineSumY;
    private int baselineFrames;
    private int baselineX;
    private int baselineY;
    private int motionFrames;
    private boolean detected;

    void arm(int anchorX, int anchorY) {
        if (anchorX < 0 || anchorY < 0) {
            throw new IllegalArgumentException("invalid motion-gate anchor");
        }
        armed = true;
        searchX = anchorX;
        searchY = anchorY;
        baselineSumX = 0L;
        baselineSumY = 0L;
        baselineFrames = 0;
        baselineX = anchorX;
        baselineY = anchorY;
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

        int minX = Math.max(0, searchX - SEARCH_HALF_SIZE);
        int maxX = Math.min(width - 1, searchX + SEARCH_HALF_SIZE);
        int minY = Math.max(0, searchY - SEARCH_HALF_SIZE);
        int maxY = Math.min(height - 1, searchY + SEARCH_HALF_SIZE);
        long sumX = 0L;
        long sumY = 0L;
        int count = 0;
        for (int y = minY; y <= maxY; y += 1) {
            int row = y * width;
            for (int x = minX; x <= maxX; x += 1) {
                int argb = pixels[row + x];
                int red = (argb >> 16) & 0xff;
                int green = (argb >> 8) & 0xff;
                int blue = argb & 0xff;
                if (green >= MIN_GREEN_VALUE
                        && green - Math.max(red, blue)
                        >= MIN_GREEN_CHROMA) {
                    sumX += x;
                    sumY += y;
                    count += 1;
                }
            }
        }

        if (count < MIN_GREEN_PIXELS) {
            motionFrames = 0;
            return new Result(
                    detected,
                    false,
                    0.0,
                    count,
                    baselineFrames < BASELINE_FRAMES
                            ? "ACQUIRE_BASELINE"
                            : "TARGET_LOST"
            );
        }

        int centroidX = (int) ((sumX + count / 2L) / count);
        int centroidY = (int) ((sumY + count / 2L) / count);
        searchX = centroidX;
        searchY = centroidY;

        if (baselineFrames < BASELINE_FRAMES) {
            baselineSumX += centroidX;
            baselineSumY += centroidY;
            baselineFrames += 1;
            if (baselineFrames == BASELINE_FRAMES) {
                baselineX = (int) ((baselineSumX + BASELINE_FRAMES / 2L)
                        / BASELINE_FRAMES);
                baselineY = (int) ((baselineSumY + BASELINE_FRAMES / 2L)
                        / BASELINE_FRAMES);
            }
            return new Result(
                    false,
                    true,
                    0.0,
                    count,
                    baselineFrames == BASELINE_FRAMES
                            ? "WAIT_MOTION"
                            : "ACQUIRE_BASELINE"
            );
        }

        int deltaX = centroidX - baselineX;
        int deltaY = centroidY - baselineY;
        int displacementSquared = deltaX * deltaX + deltaY * deltaY;
        double displacement = Math.sqrt(displacementSquared);
        if (displacementSquared >= MOTION_DISTANCE_SQUARED) {
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
                displacement,
                count,
                detected ? "MOTION_DETECTED" : "WAIT_MOTION"
        );
    }
}
