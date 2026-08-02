#include "path.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef ROUTE_MAX_LEN
#define ROUTE_MAX_LEN 1024
#endif

uint16_t path[ROUTE_MAX_LEN];

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

static unsigned int checks;
static unsigned int failures;

static void reset_path(const uint16_t *input, size_t input_length)
{
    memset(path, 0, sizeof(path));
    memcpy(path, input, input_length * sizeof(path[0]));
}

static void expect_path(const char *name,
                        const uint16_t *expected,
                        size_t expected_length)
{
    checks++;
    if (memcmp(path, expected, expected_length * sizeof(path[0])) != 0) {
        fprintf(stderr, "FAIL %s:", name);
        for (size_t i = 0U; i < expected_length; i++) {
            fprintf(stderr, " [%zu]=%u/%u", i, (unsigned int)path[i],
                    (unsigned int)expected[i]);
        }
        fputc('\n', stderr);
        failures++;
    }
}

static void test_simplify_boundaries(void)
{
    static const uint16_t empty[] = {0U};
    static const uint16_t leading_turn[] = {300U, 0U};
    static const uint16_t raw[] = {
        200U, 200U, 300U, 200U, 400U, 0U,
    };
    static const uint16_t simplified[] = {
        203U, 300U, 202U, 400U, 0U,
    };
    uint16_t long_raw[53] = {0U};
    static const uint16_t long_49[] = {297U, 0U};
    static const uint16_t long_50[] = {299U, 0U};
    static const uint16_t long_51[] = {299U, 202U, 0U};

    reset_path(empty, ARRAY_LEN(empty));
    simplifyPath();
    expect_path("empty simplify", empty, ARRAY_LEN(empty));

    reset_path(leading_turn, ARRAY_LEN(leading_turn));
    simplifyPath();
    expect_path("leading turn simplify", leading_turn,
                ARRAY_LEN(leading_turn));

    reset_path(raw, ARRAY_LEN(raw));
    simplifyPath();
    expect_path("straight simplify", simplified, ARRAY_LEN(simplified));

    for (size_t i = 0U; i < 51U; i++) {
        long_raw[i] = 200U;
    }
    long_raw[49] = 0U;
    reset_path(long_raw, 50U);
    simplifyPath();
    expect_path("49-cell leading straight", long_49, ARRAY_LEN(long_49));

    long_raw[49] = 200U;
    long_raw[50] = 0U;
    reset_path(long_raw, 51U);
    simplifyPath();
    expect_path("50-cell leading straight boundary", long_50,
                ARRAY_LEN(long_50));

    long_raw[50] = 200U;
    long_raw[51] = 0U;
    reset_path(long_raw, 52U);
    simplifyPath();
    expect_path("51-cell leading straight split", long_51,
                ARRAY_LEN(long_51));
}

static void test_large_turn_conversion(void)
{
    static const uint16_t right90[] = {202U, 300U, 202U, 0U};
    static const uint16_t right90_expected[] = {201U, 501U, 201U, 0U};
    static const uint16_t left180[] = {202U, 400U, 400U, 202U, 0U};
    static const uint16_t left180_expected[] = {201U, 602U, 201U, 0U};
    static const uint16_t consumed_connectors[] = {201U, 300U, 201U, 0U};
    static const uint16_t consumed_expected[] = {501U, 0U};
    static const uint16_t terminal_pair[] = {201U, 300U, 300U, 0U};

    reset_path(right90, ARRAY_LEN(right90));
    convertLTurn();
    expect_path("right large 90", right90_expected,
                ARRAY_LEN(right90_expected));

    reset_path(left180, ARRAY_LEN(left180));
    convertLTurn();
    expect_path("left large 180", left180_expected,
                ARRAY_LEN(left180_expected));

    reset_path(consumed_connectors, ARRAY_LEN(consumed_connectors));
    convertLTurn();
    expect_path("consumed S1 connectors", consumed_expected,
                ARRAY_LEN(consumed_expected));

    reset_path(terminal_pair, ARRAY_LEN(terminal_pair));
    convertLTurn();
    expect_path("terminal turns stay small", terminal_pair,
                ARRAY_LEN(terminal_pair));
}

static void test_diagonal_conversion(void)
{
    static const uint16_t golden[] = {
        203U, 300U, 400U, 400U, 300U, 203U, 0U,
    };
    static const uint16_t golden_expected[] = {
        202U, 701U, 802U, 703U, 202U, 0U,
    };
    static const uint16_t terminal_pair[] = {201U, 300U, 300U, 0U};
    static const uint16_t leading_turn[] = {300U, 0U};

    reset_path(golden, ARRAY_LEN(golden));
    convertDiagonal();
    expect_path("diagonal wrapper", golden_expected,
                ARRAY_LEN(golden_expected));

    reset_path(terminal_pair, ARRAY_LEN(terminal_pair));
    convertDiagonal();
    expect_path("terminal diagonal fallback", terminal_pair,
                ARRAY_LEN(terminal_pair));

    reset_path(leading_turn, ARRAY_LEN(leading_turn));
    convertLTurn();
    convertDiagonal();
    expect_path("turn-first pipeline", leading_turn, ARRAY_LEN(leading_turn));
}

static void test_start_large_turn_normalization(void)
{
    static const uint16_t right_with_straight[] = {501U, 203U, 0U};
    static const uint16_t right_expected[] = {201U, 300U, 204U, 0U};
    static const uint16_t left_without_straight[] = {601U, 300U, 0U};
    static const uint16_t left_expected[] = {
        201U, 400U, 201U, 300U, 0U,
    };

    reset_path(right_with_straight, ARRAY_LEN(right_with_straight));
    normalizeStartLargeTurnException();
    expect_path("start right large turn", right_expected,
                ARRAY_LEN(right_expected));

    reset_path(left_without_straight, ARRAY_LEN(left_without_straight));
    normalizeStartLargeTurnException();
    expect_path("start left large turn", left_expected,
                ARRAY_LEN(left_expected));
}

static void test_unterminated_input_is_unchanged(void)
{
    static uint16_t before[ROUTE_MAX_LEN];

    for (size_t i = 0U; i < ROUTE_MAX_LEN; i++) {
        path[i] = 201U;
    }
    memcpy(before, path, sizeof(path));
    simplifyPath();
    convertLTurn();
    convertDiagonal();
    expect_path("unterminated pipeline is transactional", before,
                ARRAY_LEN(before));

    path[0] = 501U;
    memcpy(before, path, sizeof(path));
    normalizeStartLargeTurnException();
    expect_path("unterminated start normalization is transactional", before,
                ARRAY_LEN(before));
}

int main(void)
{
    test_simplify_boundaries();
    test_large_turn_conversion();
    test_diagonal_conversion();
    test_start_large_turn_normalization();
    test_unterminated_input_is_unchanged();

    if (failures != 0U) {
        fprintf(stderr, "%u/%u path pipeline checks failed\n",
                failures, checks);
        return 1;
    }
    printf("path_pipeline_tests: all %u checks passed\n", checks);
    return 0;
}
