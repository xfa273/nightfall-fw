#include "legacy_path_validator.h"

#include <stdio.h>

static int failures;

static void expect_result(const char *name,
                          const uint16_t *path,
                          size_t capacity,
                          legacy_path_validation_error_t expected_error,
                          size_t expected_index,
                          uint16_t expected_code,
                          legacy_path_state_t expected_state) {
    const legacy_path_validation_result_t actual =
        legacy_path_validate(path, capacity);

    if (actual.error == expected_error && actual.index == expected_index &&
        actual.code == expected_code && actual.state == expected_state) {
        return;
    }

    fprintf(stderr,
            "FAIL %s: got {%s, index=%zu, code=%u, state=%d}; "
            "expected {%s, index=%zu, code=%u, state=%d}\n",
            name, legacy_path_validation_error_string(actual.error),
            actual.index, (unsigned int)actual.code, (int)actual.state,
            legacy_path_validation_error_string(expected_error),
            expected_index, (unsigned int)expected_code,
            (int)expected_state);
    failures++;
}

static void test_valid_paths(void) {
    static const uint16_t empty[] = {0U};
    static const uint16_t every_supported_code[] = {
        201U, 299U, 300U, 400U, 501U, 502U, 601U, 602U,
        701U, 1001U, 801U, 1002U, 703U,
        702U, 802U, 704U,
        901U, 1099U, 903U,
        902U, 1001U, 904U,
        0U,
    };

    expect_result("empty path", empty, sizeof(empty) / sizeof(empty[0]),
                  LEGACY_PATH_VALIDATION_OK, 0U, 0U,
                  LEGACY_PATH_STATE_ORTHOGONAL);
    expect_result("every supported code", every_supported_code,
                  sizeof(every_supported_code) /
                      sizeof(every_supported_code[0]),
                  LEGACY_PATH_VALIDATION_OK, 22U, 0U,
                  LEGACY_PATH_STATE_ORTHOGONAL);
}

static void test_unknown_codes(void) {
    static const uint16_t unknown_codes[] = {
        1U, 200U, 301U, 499U, 500U, 503U, 599U, 600U, 603U,
        700U, 705U, 800U, 803U, 900U, 905U, 1000U, 1100U, 65535U,
    };
    static const uint16_t zero_length_diagonal[] = {701U, 1000U, 0U};

    for (size_t i = 0U;
         i < sizeof(unknown_codes) / sizeof(unknown_codes[0]); ++i) {
        uint16_t path[] = {unknown_codes[i], 0U};
        char name[64];

        (void)snprintf(name, sizeof(name), "unknown code %u",
                       (unsigned int)unknown_codes[i]);
        expect_result(name, path, sizeof(path) / sizeof(path[0]),
                      LEGACY_PATH_VALIDATION_UNKNOWN_CODE, 0U,
                      unknown_codes[i], LEGACY_PATH_STATE_ORTHOGONAL);
    }

    expect_result("zero-length diagonal straight", zero_length_diagonal,
                  sizeof(zero_length_diagonal) /
                      sizeof(zero_length_diagonal[0]),
                  LEGACY_PATH_VALIDATION_UNKNOWN_CODE, 1U, 1000U,
                  LEGACY_PATH_STATE_DIAGONAL);
}

static void test_invalid_transitions(void) {
    static const uint16_t orth_to_exit[] = {703U, 0U};
    static const uint16_t orth_to_v90[] = {801U, 0U};
    static const uint16_t orth_to_diag_straight[] = {1001U, 0U};
    static const uint16_t diag_to_orth_straight[] = {701U, 201U, 0U};
    static const uint16_t diag_to_small_turn[] = {701U, 300U, 0U};
    static const uint16_t diag_to_large_turn[] = {701U, 501U, 0U};
    static const uint16_t diag_to_second_entry[] = {701U, 702U, 0U};

    expect_result("orthogonal to diagonal exit", orth_to_exit,
                  sizeof(orth_to_exit) / sizeof(orth_to_exit[0]),
                  LEGACY_PATH_VALIDATION_INVALID_TRANSITION, 0U, 703U,
                  LEGACY_PATH_STATE_ORTHOGONAL);
    expect_result("orthogonal to V90", orth_to_v90,
                  sizeof(orth_to_v90) / sizeof(orth_to_v90[0]),
                  LEGACY_PATH_VALIDATION_INVALID_TRANSITION, 0U, 801U,
                  LEGACY_PATH_STATE_ORTHOGONAL);
    expect_result("orthogonal to diagonal straight", orth_to_diag_straight,
                  sizeof(orth_to_diag_straight) /
                      sizeof(orth_to_diag_straight[0]),
                  LEGACY_PATH_VALIDATION_INVALID_TRANSITION, 0U, 1001U,
                  LEGACY_PATH_STATE_ORTHOGONAL);
    expect_result("diagonal to orthogonal straight", diag_to_orth_straight,
                  sizeof(diag_to_orth_straight) /
                      sizeof(diag_to_orth_straight[0]),
                  LEGACY_PATH_VALIDATION_INVALID_TRANSITION, 1U, 201U,
                  LEGACY_PATH_STATE_DIAGONAL);
    expect_result("diagonal to small turn", diag_to_small_turn,
                  sizeof(diag_to_small_turn) / sizeof(diag_to_small_turn[0]),
                  LEGACY_PATH_VALIDATION_INVALID_TRANSITION, 1U, 300U,
                  LEGACY_PATH_STATE_DIAGONAL);
    expect_result("diagonal to large turn", diag_to_large_turn,
                  sizeof(diag_to_large_turn) / sizeof(diag_to_large_turn[0]),
                  LEGACY_PATH_VALIDATION_INVALID_TRANSITION, 1U, 501U,
                  LEGACY_PATH_STATE_DIAGONAL);
    expect_result("diagonal to second entry", diag_to_second_entry,
                  sizeof(diag_to_second_entry) /
                      sizeof(diag_to_second_entry[0]),
                  LEGACY_PATH_VALIDATION_INVALID_TRANSITION, 1U, 702U,
                  LEGACY_PATH_STATE_DIAGONAL);
}

static void test_termination_and_bounds(void) {
    static const uint16_t ends_diagonal[] = {201U, 901U, 1001U, 0U};
    static const uint16_t no_terminator[] = {201U, 300U};
    static const uint16_t diagonal_no_terminator[] = {701U, 1001U};
    static const uint16_t terminator_beyond_capacity[] = {201U, 0U};
    static const uint16_t unused[] = {0U};

    expect_result("ends diagonal", ends_diagonal,
                  sizeof(ends_diagonal) / sizeof(ends_diagonal[0]),
                  LEGACY_PATH_VALIDATION_ENDS_DIAGONAL, 3U, 0U,
                  LEGACY_PATH_STATE_DIAGONAL);
    expect_result("no terminator", no_terminator,
                  sizeof(no_terminator) / sizeof(no_terminator[0]),
                  LEGACY_PATH_VALIDATION_ARRAY_LIMIT_EXCEEDED, 2U, 0U,
                  LEGACY_PATH_STATE_ORTHOGONAL);
    expect_result("diagonal without terminator", diagonal_no_terminator,
                  sizeof(diagonal_no_terminator) /
                      sizeof(diagonal_no_terminator[0]),
                  LEGACY_PATH_VALIDATION_ARRAY_LIMIT_EXCEEDED, 2U, 0U,
                  LEGACY_PATH_STATE_DIAGONAL);
    expect_result("terminator beyond capacity", terminator_beyond_capacity,
                  1U, LEGACY_PATH_VALIDATION_ARRAY_LIMIT_EXCEEDED, 1U, 0U,
                  LEGACY_PATH_STATE_ORTHOGONAL);
    expect_result("zero capacity", unused, 0U,
                  LEGACY_PATH_VALIDATION_ARRAY_LIMIT_EXCEEDED, 0U, 0U,
                  LEGACY_PATH_STATE_ORTHOGONAL);
    expect_result("null path", NULL, 1U,
                  LEGACY_PATH_VALIDATION_NULL_PATH, 0U, 0U,
                  LEGACY_PATH_STATE_ORTHOGONAL);
}

int main(void) {
    test_valid_paths();
    test_unknown_codes();
    test_invalid_transitions();
    test_termination_and_bounds();

    if (failures != 0) {
        fprintf(stderr, "%d legacy path validator test(s) failed\n", failures);
        return 1;
    }

    puts("legacy path validator tests passed");
    return 0;
}
