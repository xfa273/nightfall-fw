package com.nightfall.hfrrecorder;

import java.util.Arrays;

public final class MotionGateDetectorHostTest {
    private static final int WIDTH = 480;
    private static final int HEIGHT = 270;

    private MotionGateDetectorHostTest() {
    }

    public static void main(String[] args) {
        MotionGateDetector detector = new MotionGateDetector();
        int[] frame = new int[WIDTH * HEIGHT];
        detector.arm(240, 135);

        for (int index = 0; index < 10; index += 1) {
            makeFrame(frame, 0, (index & 1) != 0);
            MotionGateDetector.Result result = detector.process(
                    frame,
                    WIDTH,
                    HEIGHT
            );
            if (result.motionDetected || !result.targetVisible) {
                throw new AssertionError("stationary target opened motion gate");
            }
        }

        for (int index = 0; index < 2; index += 1) {
            makeFrame(frame, 10, false);
            if (detector.process(frame, WIDTH, HEIGHT).motionDetected) {
                throw new AssertionError("motion gate lacked confirmation");
            }
        }
        makeFrame(frame, 10, true);
        MotionGateDetector.Result result = detector.process(
                frame,
                WIDTH,
                HEIGHT
        );
        if (!result.motionDetected || result.displacementPx < 9.0) {
            throw new AssertionError("confirmed body motion was not detected");
        }
        System.out.println("MotionGateDetectorHostTest PASS");
    }

    private static void makeFrame(
            int[] frame,
            int offsetX,
            boolean ledsOn
    ) {
        Arrays.fill(frame, 0xff202020);
        for (int y = 128; y <= 142; y += 1) {
            for (int x = 230 + offsetX; x <= 250 + offsetX; x += 1) {
                frame[y * WIDTH + x] = 0xff207020;
            }
        }
        int ledColor = ledsOn ? 0xff2020ff : 0xff202020;
        frame[132 * WIDTH + 236 + offsetX] = ledColor;
        frame[132 * WIDTH + 244 + offsetX] = ledColor;
        frame[124 * WIDTH + 240 + offsetX] = ledColor;
    }
}
