package com.nightfall.hfrrecorder;

import java.util.Arrays;

public final class MotionGateDetectorHostTest {
    private static final int WIDTH = 480;
    private static final int HEIGHT = 270;
    private static final int CENTER_X = 240;
    private static final int CENTER_Y = 135;

    private MotionGateDetectorHostTest() {
    }

    public static void main(String[] args) {
        verifyStationaryBodyAndLedRejection();
        verifyConfirmedBodyDeparture();
        verifyUniformIlluminationChangeIsRejected();
        System.out.println("MotionGateDetectorHostTest PASS");
    }

    private static void verifyStationaryBodyAndLedRejection() {
        MotionGateDetector detector = new MotionGateDetector();
        int[] frame = new int[WIDTH * HEIGHT];
        detector.arm(CENTER_X, CENTER_Y);
        MotionGateDetector.Result result = null;
        for (int index = 0; index < 80; index += 1) {
            makeFrame(frame, 0, (index & 1) != 0, 0xff202020);
            result = detector.process(frame, WIDTH, HEIGHT);
            if (result.motionDetected) {
                throw new AssertionError("stationary LED activity opened gate");
            }
        }
        if (result == null || !result.baselineReady) {
            throw new AssertionError("stationary baseline was not acquired");
        }
    }

    private static void verifyConfirmedBodyDeparture() {
        MotionGateDetector detector = new MotionGateDetector();
        int[] frame = new int[WIDTH * HEIGHT];
        detector.arm(CENTER_X, CENTER_Y);
        for (int index = 0; index < 30; index += 1) {
            makeFrame(frame, 0, (index & 1) != 0, 0xff202020);
            detector.process(frame, WIDTH, HEIGHT);
        }
        for (int index = 0; index < 2; index += 1) {
            makeFrame(frame, 20, false, 0xff202020);
            if (detector.process(frame, WIDTH, HEIGHT).motionDetected) {
                throw new AssertionError("motion gate lacked confirmation");
            }
        }
        makeFrame(frame, 20, true, 0xff202020);
        MotionGateDetector.Result result = detector.process(
                frame,
                WIDTH,
                HEIGHT
        );
        if (!result.motionDetected || result.changedPixels < 120) {
            throw new AssertionError("confirmed body departure was not detected");
        }
    }

    private static void verifyUniformIlluminationChangeIsRejected() {
        MotionGateDetector detector = new MotionGateDetector();
        int[] frame = new int[WIDTH * HEIGHT];
        detector.arm(CENTER_X, CENTER_Y);
        for (int index = 0; index < 30; index += 1) {
            makeFrame(frame, 0, false, 0xff202020);
            detector.process(frame, WIDTH, HEIGHT);
        }
        for (int index = 0; index < 10; index += 1) {
            makeFrame(frame, 0, false, 0xff606060);
            if (detector.process(frame, WIDTH, HEIGHT).motionDetected) {
                throw new AssertionError("uniform light change opened gate");
            }
        }
    }

    private static void makeFrame(
            int[] frame,
            int offsetX,
            boolean ledsOn,
            int background
    ) {
        Arrays.fill(frame, background);
        for (int y = 124; y <= 146; y += 1) {
            for (int x = 224 + offsetX; x <= 256 + offsetX; x += 1) {
                frame[y * WIDTH + x] = 0xff364236;
            }
        }
        int ledColor = ledsOn ? 0xff2020ff : 0xff202020;
        paintLed(frame, 230 + offsetX, 140, ledColor);
        paintLed(frame, 250 + offsetX, 140, ledColor);
        paintLed(frame, 240 + offsetX, 126, ledColor);
    }

    private static void paintLed(int[] frame, int centerX, int centerY, int color) {
        for (int y = centerY - 2; y <= centerY + 2; y += 1) {
            for (int x = centerX - 2; x <= centerX + 2; x += 1) {
                frame[y * WIDTH + x] = color;
            }
        }
    }
}
