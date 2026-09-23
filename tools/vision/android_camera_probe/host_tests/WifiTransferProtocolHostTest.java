package com.nightfall.hfrrecorder;

public final class WifiTransferProtocolHostTest {
    private WifiTransferProtocolHostTest() {
    }

    public static void main(String[] args) {
        require(WifiTransferProtocol.isSafeRunName(
                "manual-1785641124561-run0001"
        ));
        require(!WifiTransferProtocol.isSafeRunName("../run"));
        require(!WifiTransferProtocol.isSafeRunName("."));
        require(WifiTransferProtocol.isArtifactName("hfr_capture.mp4"));
        require(!WifiTransferProtocol.isArtifactName("../secret"));

        WifiTransferProtocol.ByteRange all =
                WifiTransferProtocol.parseRange(null, 100L);
        require(all.start == 0L && all.end == 99L && !all.partial);
        require(all.length() == 100L);

        WifiTransferProtocol.ByteRange tail =
                WifiTransferProtocol.parseRange("bytes=40-", 100L);
        require(tail.start == 40L && tail.end == 99L && tail.partial);
        require(tail.length() == 60L);

        WifiTransferProtocol.ByteRange bounded =
                WifiTransferProtocol.parseRange("bytes=5-9", 100L);
        require(bounded.start == 5L && bounded.end == 9L);
        require(bounded.length() == 5L);

        expectRangeFailure("bytes=-5", 100L);
        expectRangeFailure("bytes=100-", 100L);
        expectRangeFailure("bytes=1-2,4-5", 100L);
        System.out.println("WifiTransferProtocolHostTest: PASS");
    }

    private static void expectRangeFailure(String value, long length) {
        try {
            WifiTransferProtocol.parseRange(value, length);
            throw new AssertionError("range unexpectedly accepted: " + value);
        } catch (IllegalArgumentException expected) {
            // Expected.
        }
    }

    private static void require(boolean condition) {
        if (!condition) {
            throw new AssertionError("requirement failed");
        }
    }
}
