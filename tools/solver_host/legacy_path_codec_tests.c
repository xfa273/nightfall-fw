#include "legacy_path_codec.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

static unsigned int checks;
static unsigned int failures;

static void fail(const char *name, const char *detail)
{
    fprintf(stderr, "FAIL %s: %s\n", name, detail);
    failures++;
}

static void expect_status(const char *name,
                          NfLegacyPathStatus expected,
                          NfLegacyPathStatus actual)
{
    checks++;
    if (actual != expected) {
        char detail[160];
        (void)snprintf(detail, sizeof(detail), "status=%s expected=%s",
                       nf_legacy_path_status_name(actual),
                       nf_legacy_path_status_name(expected));
        fail(name, detail);
    }
}

static void expect_path(const char *name,
                        const uint16_t *actual,
                        const uint16_t *expected,
                        size_t expected_capacity)
{
    checks++;
    for (size_t i = 0U; i < expected_capacity; i++) {
        if (actual[i] != expected[i]) {
            char detail[160];
            (void)snprintf(detail, sizeof(detail),
                           "code[%zu]=%u expected=%u", i,
                           (unsigned int)actual[i],
                           (unsigned int)expected[i]);
            fail(name, detail);
            return;
        }
    }
}

static NfLegacyPathResult normalize(const uint16_t *input,
                                    size_t input_capacity,
                                    uint16_t *output,
                                    size_t output_capacity)
{
    return nf_legacy_path_normalize_diagonal(input, input_capacity,
                                             output, output_capacity);
}

static void expect_normalized(const char *name,
                              const uint16_t *input,
                              size_t input_capacity,
                              const uint16_t *expected,
                              size_t expected_capacity)
{
    uint16_t output[256];
    NfLegacyPathResult result;

    for (size_t i = 0U; i < ARRAY_LEN(output); i++) {
        output[i] = 0xA55AU;
    }
    result = normalize(input, input_capacity, output, ARRAY_LEN(output));
    expect_status(name, NF_LEGACY_PATH_OK, result.status);
    if (result.status != NF_LEGACY_PATH_OK) {
        return;
    }
    checks++;
    if (result.length + 1U != expected_capacity ||
        result.required_capacity != expected_capacity) {
        char detail[160];
        (void)snprintf(detail, sizeof(detail),
                       "length=%zu required=%zu expected_capacity=%zu",
                       result.length, result.required_capacity,
                       expected_capacity);
        fail(name, detail);
    }
    expect_path(name, output, expected, expected_capacity);
    expect_status(name, NF_LEGACY_PATH_OK,
                  nf_legacy_path_validate(output, expected_capacity).status);
}

static void test_golden_runs(void)
{
    static const uint16_t rl[] = {203U, 300U, 400U, 203U, 0U};
    static const uint16_t rl_out[] = {202U, 701U, 704U, 202U, 0U};
    static const uint16_t lr[] = {203U, 400U, 300U, 203U, 0U};
    static const uint16_t lr_out[] = {202U, 702U, 703U, 202U, 0U};

    static const uint16_t rlr[] = {203U, 300U, 400U, 300U, 203U, 0U};
    static const uint16_t rlr_out[] = {202U, 701U, 1001U, 703U, 202U, 0U};
    static const uint16_t lrl[] = {203U, 400U, 300U, 400U, 203U, 0U};
    static const uint16_t lrl_out[] = {202U, 702U, 1001U, 704U, 202U, 0U};

    static const uint16_t rrl[] = {203U, 300U, 300U, 400U, 203U, 0U};
    static const uint16_t rrl_out[] = {202U, 901U, 704U, 202U, 0U};
    static const uint16_t llr[] = {203U, 400U, 400U, 300U, 203U, 0U};
    static const uint16_t llr_out[] = {202U, 902U, 703U, 202U, 0U};

    static const uint16_t rll[] = {203U, 300U, 400U, 400U, 203U, 0U};
    static const uint16_t rll_out[] = {202U, 701U, 904U, 202U, 0U};
    static const uint16_t lrr[] = {203U, 400U, 300U, 300U, 203U, 0U};
    static const uint16_t lrr_out[] = {202U, 702U, 903U, 202U, 0U};

    static const uint16_t rllr[] = {
        203U, 300U, 400U, 400U, 300U, 203U, 0U,
    };
    static const uint16_t rllr_out[] = {
        202U, 701U, 802U, 703U, 202U, 0U,
    };
    static const uint16_t lrrl[] = {
        203U, 400U, 300U, 300U, 400U, 203U, 0U,
    };
    static const uint16_t lrrl_out[] = {
        202U, 702U, 801U, 704U, 202U, 0U,
    };

    expect_normalized("R L", rl, ARRAY_LEN(rl), rl_out, ARRAY_LEN(rl_out));
    expect_normalized("L R", lr, ARRAY_LEN(lr), lr_out, ARRAY_LEN(lr_out));
    expect_normalized("R L R", rlr, ARRAY_LEN(rlr),
                      rlr_out, ARRAY_LEN(rlr_out));
    expect_normalized("L R L", lrl, ARRAY_LEN(lrl),
                      lrl_out, ARRAY_LEN(lrl_out));
    expect_normalized("R R L", rrl, ARRAY_LEN(rrl),
                      rrl_out, ARRAY_LEN(rrl_out));
    expect_normalized("L L R", llr, ARRAY_LEN(llr),
                      llr_out, ARRAY_LEN(llr_out));
    expect_normalized("R L L", rll, ARRAY_LEN(rll),
                      rll_out, ARRAY_LEN(rll_out));
    expect_normalized("L R R", lrr, ARRAY_LEN(lrr),
                      lrr_out, ARRAY_LEN(lrr_out));
    expect_normalized("R L L R", rllr, ARRAY_LEN(rllr),
                      rllr_out, ARRAY_LEN(rllr_out));
    expect_normalized("L R R L", lrrl, ARRAY_LEN(lrrl),
                      lrrl_out, ARRAY_LEN(lrrl_out));
}

static void test_incomplete_and_unsafe_runs_fall_back(void)
{
    static const uint16_t known_terminal[] = {201U, 300U, 300U, 0U};
    static const uint16_t known_terminal_left[] = {201U, 400U, 400U, 0U};
    static const uint16_t terminal_single[] = {203U, 300U, 0U};
    static const uint16_t leading_run[] = {300U, 400U, 203U, 0U};
    static const uint16_t same_three[] = {
        203U, 300U, 300U, 300U, 203U, 0U,
    };

    expect_normalized("known terminal R R fallback",
                      known_terminal, ARRAY_LEN(known_terminal),
                      known_terminal, ARRAY_LEN(known_terminal));
    expect_normalized("known terminal L L fallback",
                      known_terminal_left, ARRAY_LEN(known_terminal_left),
                      known_terminal_left, ARRAY_LEN(known_terminal_left));
    expect_normalized("terminal single fallback",
                      terminal_single, ARRAY_LEN(terminal_single),
                      terminal_single, ARRAY_LEN(terminal_single));
    expect_normalized("leading run fallback", leading_run, ARRAY_LEN(leading_run),
                      leading_run, ARRAY_LEN(leading_run));
    expect_normalized("heading-changing triple fallback",
                      same_three, ARRAY_LEN(same_three),
                      same_three, ARRAY_LEN(same_three));
}

static void test_connectors_and_existing_diagonal(void)
{
    static const uint16_t two_runs[] = {
        203U, 300U, 400U, 203U, 300U, 400U, 203U, 0U,
    };
    static const uint16_t two_runs_out[] = {
        202U, 701U, 704U, 201U, 701U, 704U, 202U, 0U,
    };
    static const uint16_t short_connectors[] = {
        201U, 300U, 400U, 201U, 0U,
    };
    static const uint16_t shared_s2[] = {
        203U, 300U, 400U, 202U, 300U, 400U, 203U, 0U,
    };
    static const uint16_t shared_s2_out[] = {
        202U, 701U, 704U, 701U, 704U, 202U, 0U,
    };
    static const uint16_t shared_s2_chain[] = {
        203U, 300U, 400U, 202U, 300U, 400U, 202U,
        300U, 400U, 203U, 0U,
    };
    static const uint16_t shared_s2_chain_out[] = {
        202U, 701U, 704U, 701U, 704U, 701U, 704U, 202U, 0U,
    };
    static const uint16_t canonical[] = {
        202U, 701U, 1002U, 801U, 1001U, 904U, 202U, 0U,
    };

    expect_normalized("shared connector shortened twice",
                      two_runs, ARRAY_LEN(two_runs),
                      two_runs_out, ARRAY_LEN(two_runs_out));
    expect_normalized("S1-bounded run stays orthogonal",
                      short_connectors, ARRAY_LEN(short_connectors),
                      short_connectors, ARRAY_LEN(short_connectors));
    expect_normalized("shared S2 is owned by both diagonal runs",
                      shared_s2, ARRAY_LEN(shared_s2),
                      shared_s2_out, ARRAY_LEN(shared_s2_out));
    expect_normalized("shared S2 diagonal chain",
                      shared_s2_chain, ARRAY_LEN(shared_s2_chain),
                      shared_s2_chain_out, ARRAY_LEN(shared_s2_chain_out));
    expect_normalized("canonical diagonal is idempotent",
                      canonical, ARRAY_LEN(canonical),
                      canonical, ARRAY_LEN(canonical));
}

static void test_long_diagonal_split(void)
{
    uint16_t input[125];
    uint16_t output[16];
    static const uint16_t expected[] = {
        202U, 701U, 1099U, 1021U, 704U, 202U, 0U,
    };

    input[0] = 203U;
    for (size_t i = 0U; i < 122U; i++) {
        input[i + 1U] = ((i & 1U) == 0U) ? 300U : 400U;
    }
    input[123] = 203U;
    input[124] = 0U;
    for (size_t i = 0U; i < ARRAY_LEN(output); i++) {
        output[i] = 0xA55AU;
    }

    const NfLegacyPathResult result = normalize(
        input, ARRAY_LEN(input), output, ARRAY_LEN(output));
    expect_status("long diagonal status", NF_LEGACY_PATH_OK, result.status);
    if (result.status == NF_LEGACY_PATH_OK) {
        expect_path("long diagonal split", output, expected, ARRAY_LEN(expected));
        expect_status("long diagonal validates", NF_LEGACY_PATH_OK,
                      nf_legacy_path_validate(output, result.required_capacity).status);
    }
}

static void test_strict_validation(void)
{
    static const uint16_t empty[] = {0U};
    static const uint16_t valid[] = {
        201U, 501U, 701U, 1001U, 802U, 1099U, 903U, 602U, 0U,
    };
    static const uint16_t ds_zero[] = {701U, 1000U, 703U, 0U};
    static const uint16_t ds_100[] = {701U, 1100U, 703U, 0U};
    static const uint16_t ds_huge[] = {701U, 65535U, 703U, 0U};
    static const uint16_t terminal_entry[] = {201U, 901U, 0U};
    static const uint16_t exit_in_orthogonal[] = {703U, 0U};
    static const uint16_t orthogonal_in_diagonal[] = {701U, 201U, 0U};
    static const uint16_t unterminated[] = {201U, 300U};

    expect_status("validate empty", NF_LEGACY_PATH_OK,
                  nf_legacy_path_validate(empty, ARRAY_LEN(empty)).status);
    expect_status("validate full grammar", NF_LEGACY_PATH_OK,
                  nf_legacy_path_validate(valid, ARRAY_LEN(valid)).status);
    expect_status("reject DS0", NF_LEGACY_PATH_UNKNOWN_CODE,
                  nf_legacy_path_validate(ds_zero, ARRAY_LEN(ds_zero)).status);
    expect_status("reject DS100", NF_LEGACY_PATH_UNKNOWN_CODE,
                  nf_legacy_path_validate(ds_100, ARRAY_LEN(ds_100)).status);
    expect_status("reject huge DS", NF_LEGACY_PATH_UNKNOWN_CODE,
                  nf_legacy_path_validate(ds_huge, ARRAY_LEN(ds_huge)).status);
    expect_status("reject diagonal terminal", NF_LEGACY_PATH_ENDS_DIAGONAL,
                  nf_legacy_path_validate(terminal_entry,
                                          ARRAY_LEN(terminal_entry)).status);
    expect_status("reject exit in orthogonal",
                  NF_LEGACY_PATH_INVALID_TRANSITION,
                  nf_legacy_path_validate(exit_in_orthogonal,
                                          ARRAY_LEN(exit_in_orthogonal)).status);
    expect_status("reject orthogonal in diagonal",
                  NF_LEGACY_PATH_INVALID_TRANSITION,
                  nf_legacy_path_validate(orthogonal_in_diagonal,
                                          ARRAY_LEN(orthogonal_in_diagonal)).status);
    expect_status("reject unterminated", NF_LEGACY_PATH_INPUT_NOT_TERMINATED,
                  nf_legacy_path_validate(unterminated,
                                          ARRAY_LEN(unterminated)).status);
    expect_status("reject null", NF_LEGACY_PATH_INVALID_ARGUMENT,
                  nf_legacy_path_validate(NULL, 1U).status);
}

static void test_failure_is_transactional(void)
{
    static const uint16_t input[] = {203U, 300U, 400U, 203U, 0U};
    static const uint16_t invalid[] = {701U, 0U};
    uint16_t too_small[4] = {11U, 22U, 33U, 44U};
    const uint16_t too_small_before[4] = {11U, 22U, 33U, 44U};
    uint16_t invalid_output[8] = {
        11U, 22U, 33U, 44U, 55U, 66U, 77U, 88U,
    };
    const uint16_t invalid_before[8] = {
        11U, 22U, 33U, 44U, 55U, 66U, 77U, 88U,
    };
    uint16_t in_place[5] = {203U, 300U, 400U, 203U, 0U};
    const uint16_t in_place_before[5] = {203U, 300U, 400U, 203U, 0U};
    NfLegacyPathResult result;

    result = normalize(input, ARRAY_LEN(input),
                       too_small, ARRAY_LEN(too_small));
    expect_status("capacity failure", NF_LEGACY_PATH_OUTPUT_CAPACITY,
                  result.status);
    expect_path("capacity failure leaves output unchanged",
                too_small, too_small_before, ARRAY_LEN(too_small));
    checks++;
    if (result.required_capacity != 5U) {
        fail("capacity reports requirement", "required capacity was not 5");
    }

    result = normalize(invalid, ARRAY_LEN(invalid),
                       invalid_output, ARRAY_LEN(invalid_output));
    expect_status("invalid input failure", NF_LEGACY_PATH_ENDS_DIAGONAL,
                  result.status);
    expect_path("invalid input leaves output unchanged",
                invalid_output, invalid_before, ARRAY_LEN(invalid_output));

    result = normalize(in_place, ARRAY_LEN(in_place),
                       in_place, ARRAY_LEN(in_place));
    expect_status("in-place rejected", NF_LEGACY_PATH_OVERLAPPING_BUFFERS,
                  result.status);
    expect_path("overlap leaves buffer unchanged", in_place, in_place_before,
                ARRAY_LEN(in_place));
}

static uint16_t mirror_code(uint16_t code)
{
    switch (code) {
    case 300U: return 400U;
    case 400U: return 300U;
    case 501U: return 601U;
    case 502U: return 602U;
    case 601U: return 501U;
    case 602U: return 502U;
    case 701U: return 702U;
    case 702U: return 701U;
    case 703U: return 704U;
    case 704U: return 703U;
    case 801U: return 802U;
    case 802U: return 801U;
    case 901U: return 902U;
    case 902U: return 901U;
    case 903U: return 904U;
    case 904U: return 903U;
    default: return code;
    }
}

static uint8_t path_heading(const uint16_t *path)
{
    uint8_t heading = 0U;
    for (size_t i = 0U; path[i] != 0U; i++) {
        unsigned int delta = 0U;
        bool right = true;
        switch (path[i]) {
        case 300U: delta = 2U; right = true; break;
        case 400U: delta = 2U; right = false; break;
        case 501U: delta = 2U; right = true; break;
        case 601U: delta = 2U; right = false; break;
        case 502U: delta = 4U; right = true; break;
        case 602U: delta = 4U; right = false; break;
        case 701U: case 703U: delta = 1U; right = true; break;
        case 702U: case 704U: delta = 1U; right = false; break;
        case 801U: delta = 2U; right = true; break;
        case 802U: delta = 2U; right = false; break;
        case 901U: case 903U: delta = 3U; right = true; break;
        case 902U: case 904U: delta = 3U; right = false; break;
        default: continue;
        }
        heading = (uint8_t)((heading +
                             (right ? delta : (8U - delta))) & 7U);
    }
    return heading;
}

typedef struct {
    int x;
    int y;
    uint8_t heading;
    bool valid;
} LogicalPose;

static uint8_t turn_heading(uint8_t heading,
                            bool right,
                            unsigned int eighth_turns)
{
    const unsigned int delta = eighth_turns & 7U;
    return (uint8_t)((heading + (right ? delta : (8U - delta))) & 7U);
}

static void add_heading_vector(LogicalPose *pose,
                               uint8_t heading,
                               unsigned int count)
{
    static const int dx[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    static const int dy[8] = {1, 1, 0, -1, -1, -1, 0, 1};

    pose->x += dx[heading & 7U] * (int)count;
    pose->y += dy[heading & 7U] * (int)count;
}

/*
 * Integrate the nominal half-grid footprint of every legacy action. Cardinal
 * basis vectors have unit length; a diagonal basis vector is the sum of its
 * two adjacent cardinal vectors. A turn spans the incoming/outgoing basis
 * vectors consumed by that action. This models geometry, not tuned arc shape.
 */
static LogicalPose logical_path_pose(const uint16_t *actions)
{
    LogicalPose pose = {0, 0, 0U, true};

    for (size_t i = 0U; actions[i] != 0U && pose.valid; i++) {
        const uint16_t code = actions[i];
        uint8_t next;
        uint8_t middle;
        bool right;

        if (code > 200U && code < 300U) {
            if ((pose.heading & 1U) != 0U) {
                pose.valid = false;
                break;
            }
            add_heading_vector(&pose, pose.heading,
                               (unsigned int)(code - 200U));
            continue;
        }
        if (code > 1000U && code <= 1099U) {
            if ((pose.heading & 1U) == 0U) {
                pose.valid = false;
                break;
            }
            add_heading_vector(&pose, pose.heading,
                               (unsigned int)(code - 1000U));
            continue;
        }

        switch (code) {
        case 300U:
        case 400U:
            if ((pose.heading & 1U) != 0U) {
                pose.valid = false;
                break;
            }
            right = code == 300U;
            next = turn_heading(pose.heading, right, 2U);
            add_heading_vector(&pose, pose.heading, 1U);
            add_heading_vector(&pose, next, 1U);
            pose.heading = next;
            break;

        case 501U:
        case 601U:
            if ((pose.heading & 1U) != 0U) {
                pose.valid = false;
                break;
            }
            right = code == 501U;
            next = turn_heading(pose.heading, right, 2U);
            add_heading_vector(&pose, pose.heading, 2U);
            add_heading_vector(&pose, next, 2U);
            pose.heading = next;
            break;

        case 502U:
        case 602U:
            if ((pose.heading & 1U) != 0U) {
                pose.valid = false;
                break;
            }
            right = code == 502U;
            middle = turn_heading(pose.heading, right, 2U);
            next = turn_heading(pose.heading, right, 4U);
            add_heading_vector(&pose, middle, 2U);
            pose.heading = next;
            break;

        case 701U:
        case 702U:
            if ((pose.heading & 1U) != 0U) {
                pose.valid = false;
                break;
            }
            right = code == 701U;
            next = turn_heading(pose.heading, right, 1U);
            add_heading_vector(&pose, pose.heading, 1U);
            add_heading_vector(&pose, next, 1U);
            pose.heading = next;
            break;

        case 901U:
        case 902U:
            if ((pose.heading & 1U) != 0U) {
                pose.valid = false;
                break;
            }
            right = code == 901U;
            middle = turn_heading(pose.heading, right, 2U);
            next = turn_heading(pose.heading, right, 3U);
            add_heading_vector(&pose, pose.heading, 1U);
            add_heading_vector(&pose, middle, 2U);
            pose.heading = next;
            break;

        case 703U:
        case 704U:
            if ((pose.heading & 1U) == 0U) {
                pose.valid = false;
                break;
            }
            right = code == 703U;
            next = turn_heading(pose.heading, right, 1U);
            add_heading_vector(&pose, pose.heading, 1U);
            add_heading_vector(&pose, next, 1U);
            pose.heading = next;
            break;

        case 903U:
        case 904U:
            if ((pose.heading & 1U) == 0U) {
                pose.valid = false;
                break;
            }
            right = code == 903U;
            middle = turn_heading(pose.heading, right, 1U);
            next = turn_heading(pose.heading, right, 3U);
            add_heading_vector(&pose, middle, 2U);
            add_heading_vector(&pose, next, 1U);
            pose.heading = next;
            break;

        case 801U:
        case 802U:
            if ((pose.heading & 1U) == 0U) {
                pose.valid = false;
                break;
            }
            right = code == 801U;
            next = turn_heading(pose.heading, right, 2U);
            add_heading_vector(&pose, pose.heading, 1U);
            add_heading_vector(&pose, next, 1U);
            pose.heading = next;
            break;

        default:
            pose.valid = false;
            break;
        }
    }
    return pose;
}

static void test_exhaustive_turn_words(void)
{
    for (size_t turn_count = 1U; turn_count <= 8U; turn_count++) {
        const unsigned int combinations = 1U << turn_count;
        for (unsigned int bits = 0U; bits < combinations; bits++) {
            uint16_t input[12] = {0U};
            uint16_t mirrored_input[12] = {0U};
            uint16_t output[24] = {0U};
            uint16_t mirrored_output[24] = {0U};
            uint16_t expected_mirror[24] = {0U};
            NfLegacyPathResult result;
            NfLegacyPathResult mirrored_result;
            LogicalPose input_pose;
            LogicalPose output_pose;

            input[0] = 203U;
            mirrored_input[0] = 203U;
            for (size_t i = 0U; i < turn_count; i++) {
                input[i + 1U] = ((bits >> i) & 1U) ? 400U : 300U;
                mirrored_input[i + 1U] = mirror_code(input[i + 1U]);
            }
            input[turn_count + 1U] = 203U;
            mirrored_input[turn_count + 1U] = 203U;

            result = normalize(input, turn_count + 3U,
                               output, ARRAY_LEN(output));
            mirrored_result = normalize(mirrored_input, turn_count + 3U,
                                        mirrored_output,
                                        ARRAY_LEN(mirrored_output));
            expect_status("exhaustive normalize", NF_LEGACY_PATH_OK,
                          result.status);
            expect_status("exhaustive mirror normalize", NF_LEGACY_PATH_OK,
                          mirrored_result.status);
            if (result.status != NF_LEGACY_PATH_OK ||
                mirrored_result.status != NF_LEGACY_PATH_OK) {
                continue;
            }
            expect_status("exhaustive output validates", NF_LEGACY_PATH_OK,
                          nf_legacy_path_validate(
                              output, result.required_capacity).status);
            checks++;
            if (path_heading(input) != path_heading(output)) {
                fail("exhaustive final heading", "heading changed");
            }
            input_pose = logical_path_pose(input);
            output_pose = logical_path_pose(output);
            checks++;
            if (!input_pose.valid || !output_pose.valid ||
                input_pose.x != output_pose.x ||
                input_pose.y != output_pose.y ||
                input_pose.heading != output_pose.heading) {
                char detail[200];
                (void)snprintf(
                    detail, sizeof(detail),
                    "turns=%zu bits=0x%x input=(%d,%d,h%u,v%d) "
                    "output=(%d,%d,h%u,v%d)",
                    turn_count, bits, input_pose.x, input_pose.y,
                    (unsigned int)input_pose.heading, input_pose.valid ? 1 : 0,
                    output_pose.x, output_pose.y,
                    (unsigned int)output_pose.heading, output_pose.valid ? 1 : 0);
                fail("exhaustive half-grid endpoint", detail);
            }

            for (size_t i = 0U; i <= result.length; i++) {
                expected_mirror[i] = mirror_code(output[i]);
            }
            checks++;
            if (result.length != mirrored_result.length ||
                memcmp(expected_mirror, mirrored_output,
                       (result.length + 1U) * sizeof(output[0])) != 0) {
                fail("exhaustive mirror property", "mirrored output differs");
            }

            {
                uint16_t second_pass[24] = {0U};
                const NfLegacyPathResult second = normalize(
                    output, result.required_capacity,
                    second_pass, ARRAY_LEN(second_pass));
                expect_status("exhaustive idempotence status",
                              NF_LEGACY_PATH_OK, second.status);
                checks++;
                if (second.status != NF_LEGACY_PATH_OK ||
                    second.length != result.length ||
                    memcmp(output, second_pass,
                           (result.length + 1U) * sizeof(output[0])) != 0) {
                    fail("exhaustive idempotence", "second pass changed output");
                }
            }
        }
    }
}

static void test_exhaustive_connector_geometry(void)
{
    for (size_t turn_count = 1U; turn_count <= 8U; turn_count++) {
        const unsigned int combinations = 1U << turn_count;
        for (unsigned int bits = 0U; bits < combinations; bits++) {
            for (uint16_t before = 201U; before <= 203U; before++) {
                for (uint16_t after = 201U; after <= 203U; after++) {
                    uint16_t input[12] = {0U};
                    uint16_t output[24] = {0U};
                    NfLegacyPathResult result;
                    LogicalPose input_pose;
                    LogicalPose output_pose;

                    input[0] = before;
                    for (size_t i = 0U; i < turn_count; i++) {
                        input[i + 1U] = ((bits >> i) & 1U) ? 400U : 300U;
                    }
                    input[turn_count + 1U] = after;

                    result = normalize(input, turn_count + 3U,
                                       output, ARRAY_LEN(output));
                    expect_status("connector geometry normalize",
                                  NF_LEGACY_PATH_OK, result.status);
                    if (result.status != NF_LEGACY_PATH_OK) {
                        continue;
                    }
                    expect_status(
                        "connector geometry validates", NF_LEGACY_PATH_OK,
                        nf_legacy_path_validate(
                            output, result.required_capacity).status);

                    input_pose = logical_path_pose(input);
                    output_pose = logical_path_pose(output);
                    checks++;
                    if (!input_pose.valid || !output_pose.valid ||
                        input_pose.x != output_pose.x ||
                        input_pose.y != output_pose.y ||
                        input_pose.heading != output_pose.heading) {
                        char detail[220];
                        (void)snprintf(
                            detail, sizeof(detail),
                            "turns=%zu bits=0x%x S%u/S%u "
                            "input=(%d,%d,h%u,v%d) output=(%d,%d,h%u,v%d)",
                            turn_count, bits, (unsigned int)(before - 200U),
                            (unsigned int)(after - 200U),
                            input_pose.x, input_pose.y,
                            (unsigned int)input_pose.heading,
                            input_pose.valid ? 1 : 0,
                            output_pose.x, output_pose.y,
                            (unsigned int)output_pose.heading,
                            output_pose.valid ? 1 : 0);
                        fail("exhaustive connector endpoint", detail);
                    }
                }
            }
        }
    }
}

static void test_empty_and_exact_capacity(void)
{
    static const uint16_t empty[] = {0U};
    static const uint16_t input[] = {203U, 300U, 400U, 203U, 0U};
    static const uint16_t expected[] = {202U, 701U, 704U, 202U, 0U};
    uint16_t empty_output[1] = {0xFFFFU};
    uint16_t exact_output[ARRAY_LEN(expected)];
    NfLegacyPathResult result;

    result = normalize(empty, ARRAY_LEN(empty),
                       empty_output, ARRAY_LEN(empty_output));
    expect_status("empty normalize", NF_LEGACY_PATH_OK, result.status);
    expect_path("empty output", empty_output, empty, ARRAY_LEN(empty));

    for (size_t i = 0U; i < ARRAY_LEN(exact_output); i++) {
        exact_output[i] = 0xFFFFU;
    }
    result = normalize(input, ARRAY_LEN(input),
                       exact_output, ARRAY_LEN(exact_output));
    expect_status("exact capacity", NF_LEGACY_PATH_OK, result.status);
    expect_path("exact capacity output", exact_output, expected,
                ARRAY_LEN(expected));
}

int main(void)
{
    test_golden_runs();
    test_incomplete_and_unsafe_runs_fall_back();
    test_connectors_and_existing_diagonal();
    test_long_diagonal_split();
    test_strict_validation();
    test_failure_is_transactional();
    test_empty_and_exact_capacity();
    test_exhaustive_turn_words();
    test_exhaustive_connector_geometry();

    if (failures != 0U) {
        fprintf(stderr, "%u/%u legacy path codec checks failed\n",
                failures, checks);
        return 1;
    }
    printf("legacy_path_codec_tests: all %u checks passed\n", checks);
    return 0;
}
