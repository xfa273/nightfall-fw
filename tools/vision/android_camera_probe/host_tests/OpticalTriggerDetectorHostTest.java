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
        System.out.println("OpticalTriggerDetectorHostTest PASS");
    }

    private static long feed(
            OpticalTriggerDetector detector,
            int[] frame,
            long nowNs,
            boolean ledsOn,
            int frames,
            boolean expectTrigger
    ) {
        boolean triggered = false;
        for (int index = 0; index < frames; index += 1) {
            makeFrame(frame, ledsOn);
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

    private static void makeFrame(int[] frame, boolean ledsOn) {
        Arrays.fill(frame, 0xff202020);
        if (!ledsOn) {
            return;
        }
        setLed(frame, 232, 134);
        setLed(frame, 240, 134);
        setLed(frame, 236, 126);
    }

    private static void setLed(int[] frame, int centerX, int centerY) {
        for (int y = centerY - 1; y <= centerY + 1; y += 1) {
            for (int x = centerX - 1; x <= centerX + 1; x += 1) {
                frame[y * WIDTH + x] = 0xffffffff;
            }
        }
    }
}
