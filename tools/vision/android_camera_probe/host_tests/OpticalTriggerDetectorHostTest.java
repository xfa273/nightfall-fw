package com.nightfall.hfrrecorder;

import java.util.Arrays;

public final class OpticalTriggerDetectorHostTest {
    private static final int WIDTH = 480;
    private static final int HEIGHT = 270;
    private static final long FRAME_NS = 25_000_000L;
    private static final int CALIBRATION_FRAMES = 90;
    private static final int PREAMBLE_FRAMES = 100;
    private static final int SYNC_FRAMES = 36;
    private static final int SYNC_GAP_FRAMES = 12;
    private static final int SLOT_FRAMES = 44;
    private static final int START_HIGH_FRAMES = 14;
    private static final int STOP_HIGH_FRAMES = 32;
    private static final int SHORT_SLOT = 1;
    private static final int LONG_SLOT = 2;
    private static final int PAIR_THREE_BY_THREE = 0;
    private static final int PAIR_ONE_PIXEL = 1;
    private static final int PAIR_MIXED_RESOLUTION = 2;
    private static final int PAIR_DIM_CONTRAST = 3;
    private static final int PAIR_BELOW_BASE_GATE = 4;

    private OpticalTriggerDetectorHostTest() {
    }

    public static void main(String[] args) {
        testImmediateTokenAfterArm();
        testDelayedFirstSampleAfterArm();
        testNoisyCalibrationCannotBlindToken();
        testCleanStart();
        testCleanStopAndEveryPrefix();
        testEveryMissingStartSlot();
        testEveryMissingStopSlot();
        testEveryTwoMissingStartSlots();
        testEveryTwoMissingStopSlots();
        testMixedPayloadIsInvalid();
        testInvalidTokenThenValidTokenRecovers();
        testModeIndicationWithoutFramingIsIgnored();
        testWhiteWaveformIsIgnored();
        testSingleLedWaveformIsIgnored();
        testWeakPairBelowConfiguredBaseGateIsIgnored();
        testTwoLedSyncAndPayloadAreAccepted();
        testMixedResolutionPairIsAccepted();
        testDimAbsoluteChromaUsesOffBaseline();
        testStaticBlueTriangleDoesNotBlockPreamble();
        testCalibrationPulseFramesDoNotRaiseNoiseFloor();
        testAdaptiveThresholdCannotBlindDeltaPair();
        testTwoOfThreePayloadLedsAreAccepted();
        testPersistentBlueMarkerAndSinglePixelLeds();
        testShortPreviewDropoutIsTolerated();
        testFirmwareSyncReedgeIsAccepted();
        testSkippedCallbacksInSyncAndPayloadAreTolerated();
        testSkippedCallbacksAtProtocolBoundariesAreTolerated();
        System.out.println("OpticalTriggerDetectorHostTest PASS");
    }

    private static void testImmediateTokenAfterArm() {
        Simulation simulation = new Simulation(false);
        OpticalTriggerDetector.Result result = simulation.emitToken(
                filledSlots(SHORT_SLOT),
                7
        );
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.START,
                "START immediately after arm"
        );
    }

    private static void testDelayedFirstSampleAfterArm() {
        int skippedPreambleFrames = 16;
        Simulation simulation = new Simulation(false);
        simulation.skipFrames(skippedPreambleFrames);
        OpticalTriggerDetector.Result result = simulation.emitToken(
                filledSlots(SHORT_SLOT),
                7,
                PREAMBLE_FRAMES - skippedPreambleFrames
        );
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.START,
                "START with 400 ms delayed first sample"
        );
    }

    private static void testNoisyCalibrationCannotBlindToken() {
        Simulation simulation = new Simulation(false);
        simulation.feedNoisyCalibration(CALIBRATION_FRAMES);
        OpticalTriggerDetector.Result result = simulation.emitToken(
                filledSlots(SHORT_SLOT),
                7
        );
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.START,
                "START after noisy calibration"
        );
        if (result.threshold != 900) {
            throw new AssertionError(
                    "noisy calibration threshold=" + result.threshold
                            + " expected capped threshold=900"
            );
        }
    }

    private static void testCleanStart() {
        Simulation simulation = new Simulation();
        OpticalTriggerDetector.Result result = simulation.emitToken(
                filledSlots(SHORT_SLOT),
                7
        );
        assertToken(result, OpticalTriggerDetector.TokenType.START, "clean START");
        assertVotes(result, 5, 0, 0, "clean START");
    }

    private static void testCleanStopAndEveryPrefix() {
        Simulation simulation = new Simulation();
        simulation.beginToken();
        simulation.assertNoToken("STOP before payload");
        for (int slot = 0; slot < 5; slot += 1) {
            simulation.feedSlot(LONG_SLOT, 7);
            simulation.assertNoToken("STOP prefix through slot " + (slot + 1));
        }
        simulation.feedDark(1);
        OpticalTriggerDetector.Result result = simulation.takeToken(
                "complete STOP"
        );
        assertToken(result, OpticalTriggerDetector.TokenType.STOP, "clean STOP");
        assertVotes(result, 0, 5, 0, "clean STOP");
    }

    private static void testEveryMissingStartSlot() {
        for (int missing = 0; missing < 5; missing += 1) {
            int[] slots = filledSlots(SHORT_SLOT);
            slots[missing] = 0;
            OpticalTriggerDetector.Result result = new Simulation().emitToken(
                    slots,
                    7
            );
            String context = "START missing slot " + (missing + 1);
            assertToken(result, OpticalTriggerDetector.TokenType.START, context);
            assertVotes(result, 4, 0, 1, context);
        }
    }

    private static void testEveryMissingStopSlot() {
        for (int missing = 0; missing < 5; missing += 1) {
            int[] slots = filledSlots(LONG_SLOT);
            slots[missing] = 0;
            OpticalTriggerDetector.Result result = new Simulation().emitToken(
                    slots,
                    7
            );
            String context = "STOP missing slot " + (missing + 1);
            assertToken(result, OpticalTriggerDetector.TokenType.STOP, context);
            assertVotes(result, 0, 4, 1, context);
        }
    }

    private static void testEveryTwoMissingStartSlots() {
        testEveryTwoMissingSlots(
                SHORT_SLOT,
                OpticalTriggerDetector.TokenType.START,
                "START"
        );
    }

    private static void testEveryTwoMissingStopSlots() {
        testEveryTwoMissingSlots(
                LONG_SLOT,
                OpticalTriggerDetector.TokenType.STOP,
                "STOP"
        );
    }

    private static void testEveryTwoMissingSlots(
            int slotType,
            OpticalTriggerDetector.TokenType expected,
            String tokenName
    ) {
        for (int first = 0; first < 4; first += 1) {
            for (int second = first + 1; second < 5; second += 1) {
                int[] slots = filledSlots(slotType);
                slots[first] = 0;
                slots[second] = 0;
                OpticalTriggerDetector.Result result =
                        new Simulation().emitToken(slots, 7);
                String context = tokenName + " missing slots "
                        + (first + 1) + "/" + (second + 1);
                assertToken(result, expected, context);
                if (slotType == SHORT_SLOT) {
                    assertVotes(result, 3, 0, 2, context);
                } else {
                    assertVotes(result, 0, 3, 2, context);
                }
            }
        }
    }

    private static void testMixedPayloadIsInvalid() {
        int[][] payloads = {
                {SHORT_SLOT, LONG_SLOT, SHORT_SLOT, LONG_SLOT, SHORT_SLOT},
                {LONG_SLOT, SHORT_SLOT, LONG_SLOT, SHORT_SLOT, LONG_SLOT}
        };
        for (int index = 0; index < payloads.length; index += 1) {
            OpticalTriggerDetector.Result result = new Simulation().emitToken(
                    payloads[index],
                    7
            );
            String context = "mixed payload " + (index + 1);
            assertToken(result, OpticalTriggerDetector.TokenType.INVALID, context);
            if (result.triggered) {
                throw new AssertionError(context + " must never trigger recording");
            }
        }
    }

    private static void testInvalidTokenThenValidTokenRecovers() {
        Simulation simulation = new Simulation();
        OpticalTriggerDetector.Result invalid = simulation.emitToken(
                new int[] {
                        SHORT_SLOT,
                        LONG_SLOT,
                        SHORT_SLOT,
                        LONG_SLOT,
                        SHORT_SLOT
                },
                7
        );
        assertToken(
                invalid,
                OpticalTriggerDetector.TokenType.INVALID,
                "mixed token before recovery"
        );
        OpticalTriggerDetector.Result recovered = simulation.emitToken(
                filledSlots(SHORT_SLOT),
                7
        );
        assertToken(
                recovered,
                OpticalTriggerDetector.TokenType.START,
                "START after invalid token"
        );
        assertVotes(recovered, 5, 0, 0, "START after invalid token");
    }

    private static void testModeIndicationWithoutFramingIsIgnored() {
        Simulation simulation = new Simulation();
        simulation.feedDark(PREAMBLE_FRAMES);
        simulation.feedMask(7, START_HIGH_FRAMES);
        simulation.feedDark(PREAMBLE_FRAMES);
        simulation.assertNoToken("unframed mode indication");

        OpticalTriggerDetector.Result result = simulation.emitToken(
                filledSlots(SHORT_SLOT),
                7
        );
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.START,
                "recovery after mode indication"
        );
    }

    private static void testWhiteWaveformIsIgnored() {
        Simulation simulation = new Simulation();
        simulation.feedDark(PREAMBLE_FRAMES);
        simulation.feedWhite(SYNC_FRAMES);
        simulation.feedDark(SYNC_GAP_FRAMES);
        for (int slot = 0; slot < 5; slot += 1) {
            simulation.feedWhite(START_HIGH_FRAMES);
            simulation.feedDark(SLOT_FRAMES - START_HIGH_FRAMES);
        }
        simulation.feedDark(1);
        simulation.assertNoToken("white framed waveform");
    }

    private static void testSingleLedWaveformIsIgnored() {
        Simulation simulation = new Simulation();
        simulation.feedDark(PREAMBLE_FRAMES);
        simulation.feedMask(1, SYNC_FRAMES);
        simulation.feedDark(SYNC_GAP_FRAMES);
        for (int slot = 0; slot < 5; slot += 1) {
            simulation.feedSlot(SHORT_SLOT, 1);
        }
        simulation.feedDark(1);
        simulation.assertNoToken("single-LED framed waveform");
    }

    private static void testWeakPairBelowConfiguredBaseGateIsIgnored() {
        Simulation simulation = new Simulation();
        simulation.feedPairFrame(
                false,
                PREAMBLE_FRAMES,
                PAIR_BELOW_BASE_GATE,
                false
        );
        simulation.feedPairFrame(
                true,
                SYNC_FRAMES,
                PAIR_BELOW_BASE_GATE,
                false
        );
        simulation.feedPairFrame(
                false,
                SYNC_GAP_FRAMES,
                PAIR_BELOW_BASE_GATE,
                false
        );
        for (int slot = 0; slot < 5; slot += 1) {
            simulation.feedPairFrame(
                    true,
                    START_HIGH_FRAMES,
                    PAIR_BELOW_BASE_GATE,
                    false
            );
            simulation.feedPairFrame(
                    false,
                    SLOT_FRAMES - START_HIGH_FRAMES,
                    PAIR_BELOW_BASE_GATE,
                    false
            );
        }
        simulation.feedPairFrame(
                false,
                1,
                PAIR_BELOW_BASE_GATE,
                false
        );
        simulation.assertNoToken("two-component waveform below base score gate");
    }

    private static void testTwoLedSyncAndPayloadAreAccepted() {
        for (int slotType : new int[] {SHORT_SLOT, LONG_SLOT}) {
            OpticalTriggerDetector.Result result =
                    new Simulation().emitPairToken(
                            slotType,
                            PAIR_ONE_PIXEL,
                            false
                    );
            OpticalTriggerDetector.TokenType expected = slotType == SHORT_SLOT
                    ? OpticalTriggerDetector.TokenType.START
                    : OpticalTriggerDetector.TokenType.STOP;
            String context = "two one-pixel LEDs throughout " + expected;
            assertToken(result, expected, context);
            if (result.matchedLeds != 2) {
                throw new AssertionError(
                        context + " matchedLeds=" + result.matchedLeds
                                + " expected=2"
                );
            }
            if (result.centerX != 230 || result.centerY != 134) {
                throw new AssertionError(
                        context + " center=" + result.centerX + ","
                                + result.centerY + " expected=230,134"
                );
            }
        }
    }

    private static void testMixedResolutionPairIsAccepted() {
        OpticalTriggerDetector.Result result =
                new Simulation().emitPairToken(
                        LONG_SLOT,
                        PAIR_MIXED_RESOLUTION,
                        false
                );
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.STOP,
                "3x3 plus one-pixel endpoint pair"
        );
        assertVotes(result, 0, 5, 0, "mixed-resolution pair");
        if (result.matchedLeds != 2) {
            throw new AssertionError(
                    "mixed-resolution pair matchedLeds=" + result.matchedLeds
                            + " expected=2"
            );
        }
    }

    private static void testDimAbsoluteChromaUsesOffBaseline() {
        OpticalTriggerDetector.Result result =
                new Simulation().emitPairToken(
                        LONG_SLOT,
                        PAIR_DIM_CONTRAST,
                        false
                );
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.STOP,
                "pair below the former absolute chroma floor"
        );
        assertVotes(result, 0, 5, 0, "dim baseline-relative pair");
        if (result.matchedLeds != 2) {
            throw new AssertionError(
                    "dim baseline-relative pair matchedLeds="
                            + result.matchedLeds + " expected=2"
            );
        }
    }

    private static void testStaticBlueTriangleDoesNotBlockPreamble() {
        Simulation simulation = new Simulation(false);
        simulation.feedPairFrame(
                false,
                CALIBRATION_FRAMES,
                PAIR_ONE_PIXEL,
                true
        );
        OpticalTriggerDetector.Result result = simulation.emitPairToken(
                SHORT_SLOT,
                PAIR_ONE_PIXEL,
                true
        );
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.START,
                "START beside a permanent blue triangle"
        );
        assertVotes(result, 5, 0, 0, "static blue triangle");
    }

    private static void testAdaptiveThresholdCannotBlindDeltaPair() {
        Simulation simulation = new Simulation(false);
        simulation.feedNoisyCalibration(CALIBRATION_FRAMES);
        OpticalTriggerDetector.Result result = simulation.emitPairToken(
                SHORT_SLOT,
                PAIR_ONE_PIXEL,
                false
        );
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.START,
                "one-pixel delta pair with capped adaptive threshold"
        );
        if (result.threshold != 900) {
            throw new AssertionError(
                    "delta-pair threshold=" + result.threshold
                            + " expected capped threshold=900"
            );
        }
        if (result.score >= result.threshold) {
            throw new AssertionError(
                    "fixture must exercise acquisition below the adaptive gate: "
                            + result.score + "/" + result.threshold
            );
        }
    }

    private static void testCalibrationPulseFramesDoNotRaiseNoiseFloor() {
        Simulation simulation = new Simulation(false);
        simulation.feedDark(32);
        for (int frame = 32; frame < CALIBRATION_FRAMES; frame += 1) {
            simulation.feedPairFrame(
                    (frame & 1) == 0,
                    1,
                    PAIR_ONE_PIXEL,
                    false
            );
        }
        OpticalTriggerDetector.Result result = simulation.emitPairToken(
                SHORT_SLOT,
                PAIR_ONE_PIXEL,
                false
        );
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.START,
                "START after calibration-time framed rises"
        );
        if (result.threshold != 200) {
            throw new AssertionError(
                    "calibration rises contaminated noise threshold="
                            + result.threshold + " expected=200"
            );
        }
    }

    private static void testTwoOfThreePayloadLedsAreAccepted() {
        for (int slotType : new int[] {SHORT_SLOT, LONG_SLOT}) {
            OpticalTriggerDetector.Result result = new Simulation().emitToken(
                    filledSlots(slotType),
                    3
            );
            OpticalTriggerDetector.TokenType expected = slotType == SHORT_SLOT
                    ? OpticalTriggerDetector.TokenType.START
                    : OpticalTriggerDetector.TokenType.STOP;
            String context = "two-of-three " + expected;
            assertToken(result, expected, context);
            if (result.matchedLeds != 2) {
                throw new AssertionError(
                        context + " matchedLeds=" + result.matchedLeds
                                + " expected=2"
                );
            }
        }
    }

    private static void testPersistentBlueMarkerAndSinglePixelLeds() {
        Simulation simulation = new Simulation(false);
        simulation.feedMarkerMask(0, CALIBRATION_FRAMES, false);
        simulation.feedMarkerMask(0, PREAMBLE_FRAMES, false);
        // Only the first SYNC sample is large enough for unconstrained
        // component discovery. The remaining signal projects to one pixel
        // per LED, matching the difficult endpoint in the real 240 fps run.
        simulation.feedMarkerMask(7, 1, false);
        simulation.feedMarkerMask(7, SYNC_FRAMES - 1, true);
        simulation.feedMarkerMask(0, SYNC_GAP_FRAMES, true);
        for (int slot = 0; slot < 5; slot += 1) {
            simulation.feedMarkerMask(7, STOP_HIGH_FRAMES, true);
            simulation.feedMarkerMask(
                    0,
                    SLOT_FRAMES - STOP_HIGH_FRAMES,
                    true
            );
        }
        simulation.feedMarkerMask(0, 1, true);
        OpticalTriggerDetector.Result result = simulation.takeToken(
                "STOP beside persistent blue marker with one-pixel LEDs"
        );
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.STOP,
                "persistent marker and one-pixel LEDs"
        );
        assertVotes(result, 0, 5, 0, "persistent marker and one-pixel LEDs");
        if (result.centerX != 233 || result.centerY != 134) {
            throw new AssertionError(
                    "SYNC learned marker instead of three real LEDs: center="
                            + result.centerX + "," + result.centerY
            );
        }
    }

    private static void testShortPreviewDropoutIsTolerated() {
        Simulation simulation = new Simulation();
        simulation.beginToken();
        simulation.feedMask(7, 8);
        simulation.feedDark(5);
        simulation.feedMask(7, 1);
        simulation.feedDark(SLOT_FRAMES - START_HIGH_FRAMES);
        for (int slot = 1; slot < 5; slot += 1) {
            simulation.feedSlot(SHORT_SLOT, 7);
        }
        simulation.feedDark(1);
        OpticalTriggerDetector.Result result = simulation.takeToken(
                "START with 125 ms preview dropout"
        );
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.START,
                "START with 125 ms preview dropout"
        );
        assertVotes(result, 5, 0, 0, "START with preview dropout");
    }

    private static void testSkippedCallbacksInSyncAndPayloadAreTolerated() {
        Simulation simulation = new Simulation();
        OpticalTriggerDetector.Result result =
                simulation.emitPairStartWithSkippedCallbacks();
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.START,
                "START with omitted preview callbacks"
        );
        assertVotes(result, 5, 0, 0, "omitted preview callbacks");
    }

    private static void testFirmwareSyncReedgeIsAccepted() {
        Simulation simulation = new Simulation();
        OpticalTriggerDetector.Result result =
                simulation.emitPairStartWithFirmwareSyncReedge();
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.START,
                "START after exact 75/50/825 ms firmware SYNC"
        );
        assertVotes(result, 5, 0, 0, "firmware SYNC re-edge");
    }

    private static void testSkippedCallbacksAtProtocolBoundariesAreTolerated() {
        Simulation simulation = new Simulation();
        OpticalTriggerDetector.Result result =
                simulation.emitPairStartWithBoundaryCallbacksSkipped();
        assertToken(
                result,
                OpticalTriggerDetector.TokenType.START,
                "START with callbacks omitted at SYNC fall and payload origin"
        );
        assertVotes(result, 5, 0, 0, "boundary callback omissions");
    }

    private static int[] filledSlots(int slotType) {
        int[] slots = new int[5];
        Arrays.fill(slots, slotType);
        return slots;
    }

    private static void assertToken(
            OpticalTriggerDetector.Result result,
            OpticalTriggerDetector.TokenType expected,
            String context
    ) {
        if (result.tokenType != expected) {
            throw new AssertionError(
                    context + " token=" + result.tokenType
                            + " expected=" + expected
                            + " phase=" + result.phase
            );
        }
        boolean expectedTriggered = expected == OpticalTriggerDetector.TokenType.START
                || expected == OpticalTriggerDetector.TokenType.STOP;
        if (result.triggered != expectedTriggered) {
            throw new AssertionError(
                    context + " triggered=" + result.triggered
                            + " expected=" + expectedTriggered
            );
        }
    }

    private static void assertVotes(
            OpticalTriggerDetector.Result result,
            int expectedShort,
            int expectedLong,
            int expectedErasure,
            String context
    ) {
        if (result.shortVotes != expectedShort
                || result.longVotes != expectedLong
                || result.erasureVotes != expectedErasure) {
            throw new AssertionError(
                    context + " votes=" + result.shortVotes
                            + "/" + result.longVotes
                            + "/" + result.erasureVotes
                            + " expected=" + expectedShort
                            + "/" + expectedLong
                            + "/" + expectedErasure
            );
        }
    }

    private static final class Simulation {
        private final OpticalTriggerDetector detector =
                new OpticalTriggerDetector(180, 2);
        private final int[] frame = new int[WIDTH * HEIGHT];
        private long nowNs;
        private OpticalTriggerDetector.Result emitted;

        Simulation() {
            this(true);
        }

        Simulation(boolean finishCalibrationFirst) {
            if (finishCalibrationFirst) {
                feedDark(CALIBRATION_FRAMES);
                assertNoToken("calibration");
            }
        }

        OpticalTriggerDetector.Result emitToken(int[] slots, int payloadMask) {
            return emitToken(slots, payloadMask, PREAMBLE_FRAMES);
        }

        OpticalTriggerDetector.Result emitToken(
                int[] slots,
                int payloadMask,
                int preambleFrames
        ) {
            if (slots.length != 5) {
                throw new IllegalArgumentException("payload must have five slots");
            }
            beginToken(preambleFrames);
            for (int slot : slots) {
                feedSlot(slot, payloadMask);
            }
            assertNoToken("complete payload before terminal sample");
            feedDark(1);
            return takeToken("complete payload");
        }

        OpticalTriggerDetector.Result emitPairToken(
                int slotType,
                int pairStyle,
                boolean staticTriangle
        ) {
            feedPairFrame(
                    false,
                    PREAMBLE_FRAMES,
                    pairStyle,
                    staticTriangle
            );
            feedPairFrame(
                    true,
                    SYNC_FRAMES,
                    pairStyle,
                    staticTriangle
            );
            feedPairFrame(
                    false,
                    SYNC_GAP_FRAMES,
                    pairStyle,
                    staticTriangle
            );
            int highFrames = slotType == SHORT_SLOT
                    ? START_HIGH_FRAMES
                    : STOP_HIGH_FRAMES;
            for (int slot = 0; slot < 5; slot += 1) {
                feedPairFrame(
                        true,
                        highFrames,
                        pairStyle,
                        staticTriangle
                );
                feedPairFrame(
                        false,
                        SLOT_FRAMES - highFrames,
                        pairStyle,
                        staticTriangle
                );
            }
            assertNoToken("pair payload before terminal sample");
            feedPairFrame(false, 1, pairStyle, staticTriangle);
            return takeToken("complete pair payload");
        }

        OpticalTriggerDetector.Result emitPairStartWithSkippedCallbacks() {
            feedPairFrame(
                    false,
                    PREAMBLE_FRAMES,
                    PAIR_ONE_PIXEL,
                    false
            );
            feedPairFrame(true, 8, PAIR_ONE_PIXEL, false);
            skipFrames(5);
            feedPairFrame(
                    true,
                    SYNC_FRAMES - 8 - 5,
                    PAIR_ONE_PIXEL,
                    false
            );
            feedPairFrame(
                    false,
                    SYNC_GAP_FRAMES,
                    PAIR_ONE_PIXEL,
                    false
            );
            for (int slot = 0; slot < 5; slot += 1) {
                if (slot == 0) {
                    feedPairFrame(true, 4, PAIR_ONE_PIXEL, false);
                    skipFrames(5);
                    feedPairFrame(
                            true,
                            START_HIGH_FRAMES - 4 - 5,
                            PAIR_ONE_PIXEL,
                            false
                    );
                } else {
                    feedPairFrame(
                            true,
                            START_HIGH_FRAMES,
                            PAIR_ONE_PIXEL,
                            false
                    );
                }
                feedPairFrame(
                        false,
                        SLOT_FRAMES - START_HIGH_FRAMES,
                        PAIR_ONE_PIXEL,
                        false
                );
            }
            assertNoToken("skipped-callback payload before terminal sample");
            feedPairFrame(false, 1, PAIR_ONE_PIXEL, false);
            return takeToken("complete skipped-callback payload");
        }

        OpticalTriggerDetector.Result emitPairStartWithFirmwareSyncReedge() {
            feedPairFrame(
                    false,
                    PREAMBLE_FRAMES,
                    PAIR_ONE_PIXEL,
                    false
            );
            // This is the exact F413 SYNC waveform: the short re-edge gap is
            // not a pulse end because it is shorter than LOW_CONFIRM_NS.
            feedPairFrame(true, 3, PAIR_ONE_PIXEL, false);  // 75 ms ON
            feedPairFrame(false, 2, PAIR_ONE_PIXEL, false); // 50 ms OFF
            feedPairFrame(true, 33, PAIR_ONE_PIXEL, false); // 825 ms ON
            feedPairFrame(
                    false,
                    SYNC_GAP_FRAMES,
                    PAIR_ONE_PIXEL,
                    false
            );
            for (int slot = 0; slot < 5; slot += 1) {
                feedPairFrame(
                        true,
                        START_HIGH_FRAMES,
                        PAIR_ONE_PIXEL,
                        false
                );
                feedPairFrame(
                        false,
                        SLOT_FRAMES - START_HIGH_FRAMES,
                        PAIR_ONE_PIXEL,
                        false
                );
            }
            assertNoToken("firmware-reedge payload before terminal sample");
            feedPairFrame(false, 1, PAIR_ONE_PIXEL, false);
            return takeToken("complete firmware-reedge payload");
        }

        OpticalTriggerDetector.Result emitPairStartWithBoundaryCallbacksSkipped() {
            feedPairFrame(
                    false,
                    PREAMBLE_FRAMES,
                    PAIR_ONE_PIXEL,
                    false
            );
            feedPairFrame(true, SYNC_FRAMES, PAIR_ONE_PIXEL, false);

            // Four physically-OFF frames pass without a preview callback at
            // the SYNC falling edge. Only the remaining 200 ms of the 300 ms
            // gap are observed, shifting the learned payload origin by 100 ms.
            skipFrames(4);
            feedPairFrame(
                    false,
                    SYNC_GAP_FRAMES - 4,
                    PAIR_ONE_PIXEL,
                    false
            );

            // The first four physically-ON payload frames are also omitted.
            // The detector must recover from both boundary losses using frame
            // timestamps rather than assuming an uninterrupted 25 ms cadence.
            skipFrames(4);
            feedPairFrame(
                    true,
                    START_HIGH_FRAMES - 4,
                    PAIR_ONE_PIXEL,
                    false
            );
            feedPairFrame(
                    false,
                    SLOT_FRAMES - START_HIGH_FRAMES,
                    PAIR_ONE_PIXEL,
                    false
            );
            for (int slot = 1; slot < 5; slot += 1) {
                feedPairFrame(
                        true,
                        START_HIGH_FRAMES,
                        PAIR_ONE_PIXEL,
                        false
                );
                feedPairFrame(
                        false,
                        SLOT_FRAMES - START_HIGH_FRAMES,
                        PAIR_ONE_PIXEL,
                        false
                );
            }
            assertNoToken("boundary-skip payload before terminal sample");
            // SYNC-fall callbacks were absent, so the detector's timestamped
            // payload origin is 100 ms later than the physical origin. Let
            // the trailing OFF level cover that bounded timing uncertainty.
            feedPairFrame(false, 5, PAIR_ONE_PIXEL, false);
            return takeToken("complete boundary-skip payload");
        }

        void beginToken() {
            beginToken(PREAMBLE_FRAMES);
        }

        void beginToken(int preambleFrames) {
            feedDark(preambleFrames);
            feedMask(7, SYNC_FRAMES);
            feedDark(SYNC_GAP_FRAMES);
            assertNoToken("preamble and SYNC");
        }

        void skipFrames(int frames) {
            nowNs += frames * FRAME_NS;
        }

        void feedSlot(int slotType, int ledMask) {
            if (slotType == SHORT_SLOT) {
                feedMask(ledMask, START_HIGH_FRAMES);
                feedDark(SLOT_FRAMES - START_HIGH_FRAMES);
            } else if (slotType == LONG_SLOT) {
                feedMask(ledMask, STOP_HIGH_FRAMES);
                feedDark(SLOT_FRAMES - STOP_HIGH_FRAMES);
            } else if (slotType == 0) {
                feedDark(SLOT_FRAMES);
            } else {
                throw new IllegalArgumentException("unknown slot type");
            }
        }

        void feedDark(int frames) {
            feedMask(0, frames);
        }

        void feedMask(int ledMask, int frames) {
            for (int index = 0; index < frames; index += 1) {
                makeFrame(frame, ledMask);
                record(detector.process(frame, WIDTH, HEIGHT, nowNs));
                nowNs += FRAME_NS;
            }
        }

        void feedWhite(int frames) {
            for (int index = 0; index < frames; index += 1) {
                Arrays.fill(frame, 0xffffffff);
                record(detector.process(frame, WIDTH, HEIGHT, nowNs));
                nowNs += FRAME_NS;
            }
        }

        void feedNoisyCalibration(int frames) {
            for (int index = 0; index < frames; index += 1) {
                makeFrame(frame, 0);
                if ((index & 1) == 0) {
                    setBlueBlock(frame, 40, 40, 6);
                }
                record(detector.process(frame, WIDTH, HEIGHT, nowNs));
                nowNs += FRAME_NS;
            }
        }

        void feedMarkerMask(int ledMask, int frames, boolean onePixelLeds) {
            for (int index = 0; index < frames; index += 1) {
                makeMarkerFrame(frame, ledMask, onePixelLeds);
                record(detector.process(frame, WIDTH, HEIGHT, nowNs));
                nowNs += FRAME_NS;
            }
        }

        void feedPairFrame(
                boolean ledsOn,
                int frames,
                int pairStyle,
                boolean staticTriangle
        ) {
            for (int index = 0; index < frames; index += 1) {
                makePairFrame(frame, ledsOn, pairStyle, staticTriangle);
                record(detector.process(frame, WIDTH, HEIGHT, nowNs));
                nowNs += FRAME_NS;
            }
        }

        void assertNoToken(String context) {
            if (emitted != null) {
                throw new AssertionError(
                        context + " unexpectedly emitted " + emitted.tokenType
                );
            }
        }

        OpticalTriggerDetector.Result takeToken(String context) {
            if (emitted == null) {
                throw new AssertionError(context + " emitted no token");
            }
            OpticalTriggerDetector.Result result = emitted;
            emitted = null;
            return result;
        }

        private void record(OpticalTriggerDetector.Result result) {
            if (result.tokenType == OpticalTriggerDetector.TokenType.NONE) {
                return;
            }
            if (emitted != null) {
                throw new AssertionError(
                        "multiple tokens emitted: " + emitted.tokenType
                                + " then " + result.tokenType
                );
            }
            emitted = result;
        }
    }

    private static void makeFrame(int[] frame, int ledMask) {
        Arrays.fill(frame, 0xff202020);
        if ((ledMask & 1) != 0) {
            setLed(frame, 232, 134);
        }
        if ((ledMask & 2) != 0) {
            setLed(frame, 240, 134);
        }
        if ((ledMask & 4) != 0) {
            setLed(frame, 236, 126);
        }
    }

    private static void setLed(int[] frame, int centerX, int centerY) {
        for (int y = centerY - 1; y <= centerY + 1; y += 1) {
            for (int x = centerX - 1; x <= centerX + 1; x += 1) {
                frame[y * WIDTH + x] = 0xff2020ff;
            }
        }
    }

    private static void makeMarkerFrame(
            int[] frame,
            int ledMask,
            boolean onePixelLeds
    ) {
        Arrays.fill(frame, 0xff202020);
        // This static, stronger component models the 8 mm blue trajectory
        // label located between the optical signalling LEDs.
        setBlueBlock(frame, 230, 134, 2);
        int[][] leds = {
                {220, 134},
                {240, 126},
                {240, 142}
        };
        for (int led = 0; led < leds.length; led += 1) {
            if ((ledMask & (1 << led)) == 0) {
                continue;
            }
            if (onePixelLeds) {
                frame[leds[led][1] * WIDTH + leds[led][0]] = 0xff202060;
            } else {
                setLed(frame, leds[led][0], leds[led][1]);
            }
        }
    }

    private static void makePairFrame(
            int[] frame,
            boolean ledsOn,
            int pairStyle,
            boolean staticTriangle
    ) {
        Arrays.fill(
                frame,
                pairStyle == PAIR_DIM_CONTRAST
                        ? 0xff202025
                        : 0xff202020
        );
        if (staticTriangle) {
            setLed(frame, 50, 50);
            setLed(frame, 70, 50);
            setLed(frame, 60, 68);
        }
        if (!ledsOn) {
            return;
        }
        if (pairStyle == PAIR_THREE_BY_THREE) {
            setLed(frame, 220, 134);
            setLed(frame, 240, 134);
        } else if (pairStyle == PAIR_ONE_PIXEL) {
            frame[134 * WIDTH + 220] = 0xff2020ff;
            frame[134 * WIDTH + 240] = 0xff2020ff;
        } else if (pairStyle == PAIR_MIXED_RESOLUTION) {
            setLed(frame, 220, 134);
            frame[134 * WIDTH + 240] = 0xff2020ff;
        } else if (pairStyle == PAIR_DIM_CONTRAST) {
            // Absolute blue chroma is 49 and 25 over a chroma-5 OFF frame. The
            // second LED is below the historical fixed floor of 48, but both
            // remain well above baseline, matching the failed real STOP clip.
            setLedColor(frame, 220, 134, 0xff202051);
            setLedColor(frame, 240, 134, 0xff202039);
        } else if (pairStyle == PAIR_BELOW_BASE_GATE) {
            // Each one-pixel component reaches the component floor (48), but
            // their score sum is only 96 and must not bypass the configured
            // aggregate acquisition gate of 180.
            frame[134 * WIDTH + 220] = 0xff202050;
            frame[134 * WIDTH + 240] = 0xff202050;
        } else {
            throw new IllegalArgumentException("unknown pair style");
        }
    }

    private static void setLedColor(
            int[] frame,
            int centerX,
            int centerY,
            int color
    ) {
        for (int y = centerY - 1; y <= centerY + 1; y += 1) {
            for (int x = centerX - 1; x <= centerX + 1; x += 1) {
                frame[y * WIDTH + x] = color;
            }
        }
    }

    private static void setBlueBlock(
            int[] frame,
            int centerX,
            int centerY,
            int radius
    ) {
        for (int y = centerY - radius; y <= centerY + radius; y += 1) {
            for (int x = centerX - radius; x <= centerX + radius; x += 1) {
                frame[y * WIDTH + x] = 0xff2020ff;
            }
        }
    }
}
