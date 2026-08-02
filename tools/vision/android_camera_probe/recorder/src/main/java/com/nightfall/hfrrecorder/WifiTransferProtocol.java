package com.nightfall.hfrrecorder;

import java.util.Set;
import java.util.regex.Pattern;

final class WifiTransferProtocol {
    static final String SCHEMA = "nightfall_hfr_wifi_v1";
    static final int DISCOVERY_PORT = 46051;
    static final int HTTP_PORT = 46052;
    static final String DISCOVERY_REQUEST = "NIGHTFALL_HFR_DISCOVER_V1";
    static final Set<String> ARTIFACT_NAMES = Set.of(
            "hfr_report.json",
            "capture_results.jsonl",
            "encoder_samples.jsonl",
            "hfr_capture.mp4"
    );

    private static final Pattern RUN_NAME = Pattern.compile(
            "[A-Za-z0-9._-]{1,160}"
    );

    private WifiTransferProtocol() {
    }

    static boolean isSafeRunName(String value) {
        return value != null
                && RUN_NAME.matcher(value).matches()
                && !value.equals(".")
                && !value.equals("..");
    }

    static boolean isArtifactName(String value) {
        return ARTIFACT_NAMES.contains(value);
    }

    static ByteRange parseRange(String header, long length) {
        if (length < 0L) {
            throw new IllegalArgumentException("negative content length");
        }
        if (header == null || header.isBlank()) {
            if (length == 0L) {
                return new ByteRange(0L, -1L, false);
            }
            return new ByteRange(0L, length - 1L, false);
        }
        if (!header.startsWith("bytes=") || header.indexOf(',') >= 0) {
            throw new IllegalArgumentException("unsupported byte range");
        }
        String value = header.substring("bytes=".length()).trim();
        int dash = value.indexOf('-');
        if (dash <= 0 || dash != value.lastIndexOf('-')) {
            throw new IllegalArgumentException("invalid byte range");
        }
        long start;
        long end;
        try {
            start = Long.parseLong(value.substring(0, dash));
            String endText = value.substring(dash + 1);
            end = endText.isEmpty()
                    ? length - 1L
                    : Long.parseLong(endText);
        } catch (NumberFormatException exception) {
            throw new IllegalArgumentException("invalid byte range", exception);
        }
        if (start < 0L || start >= length || end < start) {
            throw new IllegalArgumentException("unsatisfiable byte range");
        }
        end = Math.min(end, length - 1L);
        return new ByteRange(start, end, true);
    }

    static final class ByteRange {
        final long start;
        final long end;
        final boolean partial;

        ByteRange(long start, long end, boolean partial) {
            this.start = start;
            this.end = end;
            this.partial = partial;
        }

        long length() {
            return end < start ? 0L : end - start + 1L;
        }
    }
}
