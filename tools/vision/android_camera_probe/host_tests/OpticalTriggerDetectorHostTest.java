package com.nightfall.hfrrecorder;

import java.util.Arrays;

public final class OpticalTriggerDetectorHostTest {
    private static final int WIDTH = 480;
    private static final int HEIGHT = 270;
    private static final long FRAME_NS = 25_000_000L;

    private OpticalTriggerDetectorHostTest() {
    }

    public static void main(String[] args) {
        OpticalTriggerDetector detector = new OpticalTriggerDetector(180, 2);
        int[] frame = new int[WIDTH * HEIGHT];
        long nowNs = 0L;

        nowNs = feed(detector, frame, nowNs, false, 90, false);
        nowNs = feed(detector, frame, nowNs, true, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        nowNs = feed(detector, frame, nowNs, true, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        feed(detector, frame, nowNs, true, 24, true);

        detector.reset();
        nowNs = 0L;
        nowNs = feed(detector, frame, nowNs, false, 90, false);
        nowNs = feed(detector, frame, nowNs, true, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        nowNs = feed(detector, frame, nowNs, true, 40, false);

        detector.reset();
        nowNs = 0L;
        nowNs = feed(detector, frame, nowNs, false, 4, false);
        nowNs = feed(detector, frame, nowNs, true, 8, false);
        nowNs = feed(detector, frame, nowNs, false, 78, false);
        nowNs = feed(detector, frame, nowNs, true, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        nowNs = feed(detector, frame, nowNs, true, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        feed(detector, frame, nowNs, true, 24, true);

        detector.reset();
        nowNs = 0L;
        nowNs = feed(detector, frame, nowNs, false, 90, false);
        nowNs = feedWhiteFlash(detector, frame, nowNs, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        nowNs = feedWhiteFlash(detector, frame, nowNs, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        feedWhiteFlash(detector, frame, nowNs, 24, false);

        detector.reset();
        nowNs = 0L;
        nowNs = feed(detector, frame, nowNs, false, 90, false);
        nowNs = feedLedMask(detector, frame, nowNs, 1, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        nowNs = feedLedMask(detector, frame, nowNs, 1, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        feedLedMask(detector, frame, nowNs, 1, 24, false);

        detector.reset();
        nowNs = 0L;
        nowNs = feed(detector, frame, nowNs, false, 90, false);
        nowNs = feedLedMask(detector, frame, nowNs, 1, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        nowNs = feedLedMask(detector, frame, nowNs, 2, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        feedLedMask(detector, frame, nowNs, 4, 24, false);

        detector.reset();
        nowNs = 0L;
        nowNs = feed(detector, frame, nowNs, false, 90, false);
        nowNs = feed(detector, frame, nowNs, true, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        nowNs = feedLedMask(detector, frame, nowNs, 1, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        feed(detector, frame, nowNs, true, 24, false);

        detector.reset();
        nowNs = 0L;
        nowNs = feed(detector, frame, nowNs, false, 90, false);
        nowNs = feedAtOffset(detector, frame, nowNs, true, 12, 0, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        nowNs = feedAtOffset(detector, frame, nowNs, true, 12, 20, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        feedAtOffset(detector, frame, nowNs, true, 24, 0, false);

        detector = new OpticalTriggerDetector(180, 2, 4);
        nowNs = 0L;
        nowNs = feed(detector, frame, nowNs, false, 90, false);
        nowNs = feed(detector, frame, nowNs, true, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        nowNs = feed(detector, frame, nowNs, true, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        nowNs = feed(detector, frame, nowNs, true, 12, false);
        nowNs = feed(detector, frame, nowNs, false, 8, false);
        feed(detector, frame, nowNs, true, 24, true);

        // A mode-selection all-LED indication must expire before the token.
        detector = new OpticalTriggerDetector(180, 2);
        nowNs = 0L;
        nowNs = feed(detector, frame, nowNs, false, 90, false);
        nowNs = feed(detector, frame, nowNs, true, 14, false);
        nowNs = feed(detector, frame, nowNs, false, 60, false);
        nowNs = feed(detector, frame, nowNs, true, 14, false);
        nowNs = feed(detector, frame, nowNs, false, 12, false);
        nowNs = feed(detector, frame, nowNs, true, 14, false);
        nowNs = feed(detector, frame, nowNs, false, 12, false);
        feed(detector, frame, nowNs, true, 24, true);

        // Slowly brightening LEDs have no >=20-level single-frame edge. The
        // absolute blue level must still decode all three pulses exactly once.
        detector = new OpticalTriggerDetector(180, 2);
        nowNs = 0L;
        nowNs = feed(detector, frame, nowNs, false, 90, false);
        nowNs = feedGradualPulse(detector, frame, nowNs, false);
        nowNs = feed(detector, frame, nowNs, false, 12, false);
        nowNs = feedGradualPulse(detector, frame, nowNs, false);
        nowNs = feed(detector, frame, nowNs, false, 12, false);
        feedGradualPulse(detector, frame, nowNs, true);
        System.out.println("OpticalTriggerDetectorHostTest PASS");
    }

    private static long feedGradualPulse(
            OpticalTriggerDetector detector,
            int[] frame,
            long nowNs,
            boolean expectTrigger
    ) {
        boolean triggered = false;
        for (int step = 0; step < 8; step += 1) {
            makeFrame(frame, 7, 0, 0x20 + (step + 1) * 16);
            OpticalTriggerDetector.Result result = detector.process(
                    frame,
                    WIDTH,
                    HEIGHT,
                    nowNs
            );
            triggered |= result.triggered;
            nowNs += FRAME_NS;
        }
        for (int hold = 0; hold < 6; hold += 1) {
            makeFrame(frame, 7, 0, 0xa0);
            OpticalTriggerDetector.Result result = detector.process(
                    frame,
                    WIDTH,
                    HEIGHT,
                    nowNs
            );
            triggered |= result.triggered;
            nowNs += FRAME_NS;
        }
        if (triggered != expectTrigger) {
            throw new AssertionError(
                    "gradual-pulse triggered=" + triggered
                            + " expected=" + expectTrigger
            );
        }
        return nowNs;
    }

    private static long feedWhiteFlash(
            OpticalTriggerDetector detector,
            int[] frame,
            long nowNs,
            int frames,
            boolean expectTrigger
    ) {
        boolean triggered = false;
        for (int index = 0; index < frames; index += 1) {
            Arrays.fill(frame, 0xffffffff);
            OpticalTriggerDetector.Result result = detector.process(
                    frame,
                    WIDTH,
                    HEIGHT,
                    nowNs
            );
            triggered |= result.triggered;
            nowNs += FRAME_NS;
        }
        if (triggered != expectTrigger) {
            throw new AssertionError(
                    "white-flash triggered=" + triggered
                            + " expected=" + expectTrigger
            );
        }
        return nowNs;
    }

    private static long feed(
            OpticalTriggerDetector detector,
            int[] frame,
            long nowNs,
            boolean ledsOn,
            int frames,
            boolean expectTrigger
    ) {
        return feedAtOffset(
                detector,
                frame,
                nowNs,
                ledsOn,
                frames,
                0,
                expectTrigger
        );
    }

    private static long feedLedMask(
            OpticalTriggerDetector detector,
            int[] frame,
            long nowNs,
            int ledMask,
            int frames,
            boolean expectTrigger
    ) {
        boolean triggered = false;
        for (int index = 0; index < frames; index += 1) {
            makeFrame(frame, ledMask, 0);
            OpticalTriggerDetector.Result result = detector.process(
                    frame,
                    WIDTH,
                    HEIGHT,
                    nowNs
            );
            triggered |= result.triggered;
            nowNs += FRAME_NS;
        }
        if (triggered != expectTrigger) {
            throw new AssertionError(
                    "led-mask=" + ledMask + " triggered=" + triggered
                            + " expected=" + expectTrigger
            );
        }
        return nowNs;
    }

    private static long feedAtOffset(
            OpticalTriggerDetector detector,
            int[] frame,
            long nowNs,
            boolean ledsOn,
            int frames,
            int offsetX,
            boolean expectTrigger
    ) {
        boolean triggered = false;
        for (int index = 0; index < frames; index += 1) {
            makeFrame(frame, ledsOn ? 7 : 0, offsetX);
            OpticalTriggerDetector.Result result = detector.process(
                    frame,
                    WIDTH,
                    HEIGHT,
                    nowNs
            );
            triggered |= result.triggered;
            nowNs += FRAME_NS;
        }
        if (triggered != expectTrigger) {
            throw new AssertionError(
                    "triggered=" + triggered + " expected=" + expectTrigger
            );
        }
        return nowNs;
    }

    private static void makeFrame(int[] frame, int ledMask, int offsetX) {
        makeFrame(frame, ledMask, offsetX, 0xff);
    }

    private static void makeFrame(
            int[] frame,
            int ledMask,
            int offsetX,
            int blue
    ) {
        Arrays.fill(frame, 0xff202020);
        if ((ledMask & 1) != 0) {
            setLed(frame, 232 + offsetX, 134, blue);
        }
        if ((ledMask & 2) != 0) {
            setLed(frame, 240 + offsetX, 134, blue);
        }
        if ((ledMask & 4) != 0) {
            setLed(frame, 236 + offsetX, 126, blue);
        }
    }

    private static void setLed(
            int[] frame,
            int centerX,
            int centerY,
            int blue
    ) {
        for (int y = centerY - 1; y <= centerY + 1; y += 1) {
            for (int x = centerX - 1; x <= centerX + 1; x += 1) {
                frame[y * WIDTH + x] = 0xff202000 | blue;
            }
        }
    }
}
