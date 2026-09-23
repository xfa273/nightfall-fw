#include "legacy_path_codec.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    NF_LEGACY_CODE_UNKNOWN = 0,
    NF_LEGACY_CODE_ORTHOGONAL,
    NF_LEGACY_CODE_DIAGONAL_ENTRY,
    NF_LEGACY_CODE_DIAGONAL_BODY,
    NF_LEGACY_CODE_DIAGONAL_EXIT,
} NfLegacyCodeClass;

typedef struct {
    uint16_t *output;
    size_t capacity;
    size_t count;
    uint16_t last_code;
    bool write;
} NfLegacyEmitter;

static NfLegacyPathResult nf_legacy_result(NfLegacyPathStatus status,
                                           size_t index,
                                           uint16_t code,
                                           NfLegacyPathState state,
                                           size_t length,
                                           size_t required_capacity)
{
    NfLegacyPathResult result;
    result.status = status;
    result.index = index;
    result.code = code;
    result.state = state;
    result.length = length;
    result.required_capacity = required_capacity;
    return result;
}

static bool nf_legacy_is_orthogonal_straight(uint16_t code)
{
    return code > NF_LEGACY_PATH_STRAIGHT_BASE &&
           code <= NF_LEGACY_PATH_STRAIGHT_MAX;
}

static bool nf_legacy_is_small_turn(uint16_t code)
{
    return code == NF_LEGACY_PATH_SMALL_RIGHT_90 ||
           code == NF_LEGACY_PATH_SMALL_LEFT_90;
}

static bool nf_legacy_turn_is_right(uint16_t code)
{
    return code == NF_LEGACY_PATH_SMALL_RIGHT_90;
}

static NfLegacyCodeClass nf_legacy_classify(uint16_t code)
{
    if (nf_legacy_is_orthogonal_straight(code)) {
        return NF_LEGACY_CODE_ORTHOGONAL;
    }

    switch (code) {
    case NF_LEGACY_PATH_SMALL_RIGHT_90:
    case NF_LEGACY_PATH_SMALL_LEFT_90:
    case NF_LEGACY_PATH_LARGE_RIGHT_90:
    case NF_LEGACY_PATH_LARGE_RIGHT_180:
    case NF_LEGACY_PATH_LARGE_LEFT_90:
    case NF_LEGACY_PATH_LARGE_LEFT_180:
        return NF_LEGACY_CODE_ORTHOGONAL;

    case NF_LEGACY_PATH_RIGHT_45_IN:
    case NF_LEGACY_PATH_LEFT_45_IN:
    case NF_LEGACY_PATH_RIGHT_135_IN:
    case NF_LEGACY_PATH_LEFT_135_IN:
        return NF_LEGACY_CODE_DIAGONAL_ENTRY;

    case NF_LEGACY_PATH_RIGHT_V90:
    case NF_LEGACY_PATH_LEFT_V90:
        return NF_LEGACY_CODE_DIAGONAL_BODY;

    case NF_LEGACY_PATH_RIGHT_45_OUT:
    case NF_LEGACY_PATH_LEFT_45_OUT:
    case NF_LEGACY_PATH_RIGHT_135_OUT:
    case NF_LEGACY_PATH_LEFT_135_OUT:
        return NF_LEGACY_CODE_DIAGONAL_EXIT;

    default:
        if (code > NF_LEGACY_PATH_DIAGONAL_STRAIGHT_BASE &&
            code <= NF_LEGACY_PATH_DIAGONAL_STRAIGHT_MAX) {
            return NF_LEGACY_CODE_DIAGONAL_BODY;
        }
        return NF_LEGACY_CODE_UNKNOWN;
    }
}

NfLegacyPathResult nf_legacy_path_validate(const uint16_t *path,
                                           size_t capacity)
{
    NfLegacyPathState state = NF_LEGACY_PATH_STATE_ORTHOGONAL;

    if (path == NULL) {
        return nf_legacy_result(NF_LEGACY_PATH_INVALID_ARGUMENT, 0U, 0U,
                                state, 0U, 0U);
    }

    for (size_t index = 0U; index < capacity; index++) {
        const uint16_t code = path[index];
        const NfLegacyCodeClass code_class = nf_legacy_classify(code);

        if (code == 0U) {
            if (state == NF_LEGACY_PATH_STATE_DIAGONAL) {
                return nf_legacy_result(NF_LEGACY_PATH_ENDS_DIAGONAL,
                                        index, code, state, index, index + 1U);
            }
            return nf_legacy_result(NF_LEGACY_PATH_OK, index, code, state,
                                    index, index + 1U);
        }
        if (code_class == NF_LEGACY_CODE_UNKNOWN) {
            return nf_legacy_result(NF_LEGACY_PATH_UNKNOWN_CODE, index, code,
                                    state, index, 0U);
        }

        if (state == NF_LEGACY_PATH_STATE_ORTHOGONAL) {
            if (code_class == NF_LEGACY_CODE_ORTHOGONAL) {
                continue;
            }
            if (code_class == NF_LEGACY_CODE_DIAGONAL_ENTRY) {
                state = NF_LEGACY_PATH_STATE_DIAGONAL;
                continue;
            }
        } else {
            if (code_class == NF_LEGACY_CODE_DIAGONAL_BODY) {
                continue;
            }
            if (code_class == NF_LEGACY_CODE_DIAGONAL_EXIT) {
                state = NF_LEGACY_PATH_STATE_ORTHOGONAL;
                continue;
            }
        }

        return nf_legacy_result(NF_LEGACY_PATH_INVALID_TRANSITION, index,
                                code, state, index, 0U);
    }

    return nf_legacy_result(NF_LEGACY_PATH_INPUT_NOT_TERMINATED, capacity,
                            0U, state, capacity, 0U);
}

static bool nf_legacy_ranges_overlap(const uint16_t *input,
                                     size_t input_capacity,
                                     const uint16_t *output,
                                     size_t output_capacity)
{
    uintptr_t input_begin;
    uintptr_t output_begin;
    uintptr_t input_bytes;
    uintptr_t output_bytes;
    uintptr_t input_end;
    uintptr_t output_end;

    if (input_capacity == 0U || output_capacity == 0U) {
        return false;
    }
    if (input_capacity > UINTPTR_MAX / sizeof(*input) ||
        output_capacity > UINTPTR_MAX / sizeof(*output)) {
        return true;
    }

    input_begin = (uintptr_t)input;
    output_begin = (uintptr_t)output;
    input_bytes = (uintptr_t)(input_capacity * sizeof(*input));
    output_bytes = (uintptr_t)(output_capacity * sizeof(*output));
    if (input_begin > UINTPTR_MAX - input_bytes ||
        output_begin > UINTPTR_MAX - output_bytes) {
        return true;
    }
    input_end = input_begin + input_bytes;
    output_end = output_begin + output_bytes;
    return input_begin < output_end && output_begin < input_end;
}

static bool nf_legacy_emit(NfLegacyEmitter *emitter, uint16_t code)
{
    if (emitter->count >= emitter->capacity) {
        return false;
    }
    if (emitter->write) {
        emitter->output[emitter->count] = code;
    }
    emitter->last_code = code;
    emitter->count++;
    return true;
}

static void nf_legacy_shorten_previous_straight(NfLegacyEmitter *emitter)
{
    if (emitter->count == 0U ||
        !nf_legacy_is_orthogonal_straight(emitter->last_code) ||
        emitter->last_code == NF_LEGACY_PATH_STRAIGHT_BASE + 1U) {
        return;
    }
    emitter->last_code--;
    if (emitter->write) {
        emitter->output[emitter->count - 1U] = emitter->last_code;
    }
}

static uint8_t nf_legacy_add_heading_delta(uint8_t heading,
                                           bool right,
                                           unsigned int eighth_turns)
{
    const unsigned int delta = eighth_turns & 7U;
    return (uint8_t)((heading + (right ? delta : (8U - delta))) & 7U);
}

typedef struct {
    int64_t x;
    int64_t y;
    uint8_t heading;
} NfLegacyLogicalPose;

static bool nf_legacy_add_component(int64_t *value, int delta)
{
    if ((delta > 0 && *value > INT64_MAX - delta) ||
        (delta < 0 && *value < INT64_MIN - delta)) {
        return false;
    }
    *value += delta;
    return true;
}

static bool nf_legacy_add_heading_vector(NfLegacyLogicalPose *pose,
                                         uint8_t heading,
                                         unsigned int count)
{
    static const int8_t dx[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    static const int8_t dy[8] = {1, 1, 0, -1, -1, -1, 0, 1};

    for (unsigned int i = 0U; i < count; i++) {
        if (!nf_legacy_add_component(&pose->x, dx[heading & 7U]) ||
            !nf_legacy_add_component(&pose->y, dy[heading & 7U])) {
            return false;
        }
    }
    return true;
}

static bool nf_legacy_integrate_small_turn(NfLegacyLogicalPose *pose,
                                           bool right)
{
    const uint8_t next = nf_legacy_add_heading_delta(
        pose->heading, right, 2U);

    if (!nf_legacy_add_heading_vector(pose, pose->heading, 1U) ||
        !nf_legacy_add_heading_vector(pose, next, 1U)) {
        return false;
    }
    pose->heading = next;
    return true;
}

static bool nf_legacy_integrate_entry(NfLegacyLogicalPose *pose,
                                      bool right,
                                      size_t count)
{
    const uint8_t original = pose->heading;

    if (count == 2U) {
        const uint8_t middle = nf_legacy_add_heading_delta(
            original, right, 2U);
        const uint8_t next = nf_legacy_add_heading_delta(
            original, right, 3U);
        if (!nf_legacy_add_heading_vector(pose, original, 1U) ||
            !nf_legacy_add_heading_vector(pose, middle, 2U)) {
            return false;
        }
        pose->heading = next;
        return true;
    }

    const uint8_t next = nf_legacy_add_heading_delta(original, right, 1U);
    if (!nf_legacy_add_heading_vector(pose, original, 1U) ||
        !nf_legacy_add_heading_vector(pose, next, 1U)) {
        return false;
    }
    pose->heading = next;
    return true;
}

static bool nf_legacy_integrate_exit(NfLegacyLogicalPose *pose,
                                     bool right,
                                     size_t count)
{
    const uint8_t original = pose->heading;

    if (count == 2U) {
        const uint8_t middle = nf_legacy_add_heading_delta(
            original, right, 1U);
        const uint8_t next = nf_legacy_add_heading_delta(
            original, right, 3U);
        if (!nf_legacy_add_heading_vector(pose, middle, 2U) ||
            !nf_legacy_add_heading_vector(pose, next, 1U)) {
            return false;
        }
        pose->heading = next;
        return true;
    }

    const uint8_t next = nf_legacy_add_heading_delta(original, right, 1U);
    if (!nf_legacy_add_heading_vector(pose, original, 1U) ||
        !nf_legacy_add_heading_vector(pose, next, 1U)) {
        return false;
    }
    pose->heading = next;
    return true;
}

static bool nf_legacy_integrate_v90(NfLegacyLogicalPose *pose, bool right)
{
    const uint8_t next = nf_legacy_add_heading_delta(
        pose->heading, right, 2U);

    if (!nf_legacy_add_heading_vector(pose, pose->heading, 1U) ||
        !nf_legacy_add_heading_vector(pose, next, 1U)) {
        return false;
    }
    pose->heading = next;
    return true;
}

static bool nf_legacy_diagonal_run_is_complete(const uint16_t *input,
                                                size_t input_length,
                                                size_t start,
                                                size_t end,
                                                bool shorten_before,
                                                bool shorten_after,
                                                size_t *entry_count,
                                                size_t *exit_count)
{
    NfLegacyLogicalPose raw = {0, 0, 0U};
    NfLegacyLogicalPose action = {0, 0, 0U};
    size_t middle;
    size_t middle_end;

    if (start == 0U || end >= input_length || end - start < 2U ||
        !shorten_before || !shorten_after ||
        !nf_legacy_is_orthogonal_straight(input[start - 1U]) ||
        !nf_legacy_is_orthogonal_straight(input[end])) {
        return false;
    }

    *entry_count = (start + 1U < end && input[start] == input[start + 1U])
                       ? 2U
                       : 1U;
    *exit_count = (end >= start + 2U && input[end - 2U] == input[end - 1U])
                      ? 2U
                      : 1U;
    if (*entry_count + *exit_count > end - start) {
        return false;
    }

    for (size_t i = start; i < end; i++) {
        if (!nf_legacy_integrate_small_turn(
                &raw, nf_legacy_turn_is_right(input[i]))) {
            return false;
        }
    }

    if (!nf_legacy_integrate_entry(
            &action, nf_legacy_turn_is_right(input[start]), *entry_count)) {
        return false;
    }

    middle = start + *entry_count;
    middle_end = end - *exit_count;
    while (middle < middle_end) {
        if (middle + 1U < middle_end &&
            input[middle] == input[middle + 1U]) {
            if (!nf_legacy_integrate_v90(
                    &action, nf_legacy_turn_is_right(input[middle]))) {
                return false;
            }
            middle += 2U;
        } else {
            /* A remaining alternating small turn becomes one diagonal unit. */
            if (!nf_legacy_add_heading_vector(
                    &action, action.heading, 1U)) {
                return false;
            }
            middle++;
        }
    }

    if (!nf_legacy_integrate_exit(
            &action, nf_legacy_turn_is_right(input[end - 1U]), *exit_count)) {
        return false;
    }

    /* Diagonal entry/exit consume one half section from available connectors. */
    if ((shorten_before &&
         !nf_legacy_add_heading_vector(&raw, 0U, 1U)) ||
        (shorten_after &&
         !nf_legacy_add_heading_vector(&raw, raw.heading, 1U))) {
        return false;
    }
    return action.heading == raw.heading &&
           action.x == raw.x && action.y == raw.y;
}

static uint16_t nf_legacy_entry_code(bool right, size_t count)
{
    if (count == 2U) {
        return right ? NF_LEGACY_PATH_RIGHT_135_IN
                     : NF_LEGACY_PATH_LEFT_135_IN;
    }
    return right ? NF_LEGACY_PATH_RIGHT_45_IN
                 : NF_LEGACY_PATH_LEFT_45_IN;
}

static uint16_t nf_legacy_exit_code(bool right, size_t count)
{
    if (count == 2U) {
        return right ? NF_LEGACY_PATH_RIGHT_135_OUT
                     : NF_LEGACY_PATH_LEFT_135_OUT;
    }
    return right ? NF_LEGACY_PATH_RIGHT_45_OUT
                 : NF_LEGACY_PATH_LEFT_45_OUT;
}

static bool nf_legacy_emit_diagonal_units(NfLegacyEmitter *emitter,
                                          size_t units)
{
    while (units != 0U) {
        const size_t chunk = (units > 99U) ? 99U : units;
        if (!nf_legacy_emit(
                emitter,
                (uint16_t)(NF_LEGACY_PATH_DIAGONAL_STRAIGHT_BASE + chunk))) {
            return false;
        }
        units -= chunk;
    }
    return true;
}

static bool nf_legacy_emit_diagonal_run(NfLegacyEmitter *emitter,
                                        const uint16_t *input,
                                        size_t start,
                                        size_t end,
                                        size_t entry_count,
                                        size_t exit_count,
                                        bool shorten_previous)
{
    size_t middle = start + entry_count;
    const size_t middle_end = end - exit_count;
    size_t diagonal_units = 0U;

    if (shorten_previous) {
        nf_legacy_shorten_previous_straight(emitter);
    }
    if (!nf_legacy_emit(emitter,
                        nf_legacy_entry_code(
                            nf_legacy_turn_is_right(input[start]),
                            entry_count))) {
        return false;
    }

    while (middle < middle_end) {
        if (middle + 1U < middle_end &&
            input[middle] == input[middle + 1U]) {
            if (!nf_legacy_emit_diagonal_units(emitter, diagonal_units) ||
                !nf_legacy_emit(
                    emitter,
                    nf_legacy_turn_is_right(input[middle])
                        ? NF_LEGACY_PATH_RIGHT_V90
                        : NF_LEGACY_PATH_LEFT_V90)) {
                return false;
            }
            diagonal_units = 0U;
            middle += 2U;
        } else {
            diagonal_units++;
            middle++;
        }
    }

    if (!nf_legacy_emit_diagonal_units(emitter, diagonal_units) ||
        !nf_legacy_emit(
            emitter,
            nf_legacy_exit_code(
                nf_legacy_turn_is_right(input[end - 1U]), exit_count))) {
        return false;
    }
    return true;
}

static bool nf_legacy_normalize_pass(const uint16_t *input,
                                     size_t input_length,
                                     NfLegacyEmitter *emitter)
{
    size_t index = 0U;

    while (index < input_length) {
        if (nf_legacy_is_small_turn(input[index])) {
            const size_t run_start = index;
            size_t run_end = index + 1U;
            size_t entry_count = 0U;
            size_t exit_count = 0U;

            while (run_end < input_length &&
                   nf_legacy_is_small_turn(input[run_end])) {
                run_end++;
            }
            if (nf_legacy_diagonal_run_is_complete(
                    input, input_length, run_start, run_end,
                    emitter->count != 0U &&
                        nf_legacy_is_orthogonal_straight(emitter->last_code) &&
                        emitter->last_code > NF_LEGACY_PATH_STRAIGHT_BASE + 1U,
                    input[run_end] > NF_LEGACY_PATH_STRAIGHT_BASE + 1U,
                    &entry_count, &exit_count)) {
                uint16_t following_straight;
                size_t converted_end = run_end;
                if (!nf_legacy_emit_diagonal_run(
                        emitter, input, run_start, run_end,
                        entry_count, exit_count, true)) {
                    return false;
                }

                /*
                 * An S2 shared by two complete runs is entirely owned by
                 * the first exit and second entry.  Convert the whole chain
                 * transactionally instead of greedily emitting S1 and making
                 * the following run ineligible.
                 */
                while (input[converted_end] ==
                           NF_LEGACY_PATH_STRAIGHT_BASE + 2U &&
                       converted_end + 1U < input_length &&
                       nf_legacy_is_small_turn(input[converted_end + 1U])) {
                    const size_t next_start = converted_end + 1U;
                    size_t next_end = next_start + 1U;
                    size_t next_entry_count = 0U;
                    size_t next_exit_count = 0U;

                    while (next_end < input_length &&
                           nf_legacy_is_small_turn(input[next_end])) {
                        next_end++;
                    }
                    if (!nf_legacy_diagonal_run_is_complete(
                            input, input_length, next_start, next_end,
                            true,
                            input[next_end] >
                                NF_LEGACY_PATH_STRAIGHT_BASE + 1U,
                            &next_entry_count, &next_exit_count)) {
                        break;
                    }
                    if (!nf_legacy_emit_diagonal_run(
                            emitter, input, next_start, next_end,
                            next_entry_count, next_exit_count, false)) {
                        return false;
                    }
                    converted_end = next_end;
                }

                following_straight = input[converted_end];
                if (following_straight >
                    NF_LEGACY_PATH_STRAIGHT_BASE + 1U) {
                    following_straight--;
                }
                if (!nf_legacy_emit(emitter, following_straight)) {
                    return false;
                }
                index = converted_end + 1U;
                continue;
            }

            while (index < run_end) {
                if (!nf_legacy_emit(emitter, input[index])) {
                    return false;
                }
                index++;
            }
            continue;
        }

        if (!nf_legacy_emit(emitter, input[index])) {
            return false;
        }
        index++;
    }
    return true;
}

NfLegacyPathResult nf_legacy_path_normalize_diagonal(
    const uint16_t *input,
    size_t input_capacity,
    uint16_t *output,
    size_t output_capacity)
{
    NfLegacyPathResult validation;
    NfLegacyEmitter sizing = {NULL, SIZE_MAX, 0U, 0U, false};
    NfLegacyEmitter writer;
    size_t required_capacity;

    if (input == NULL || output == NULL) {
        return nf_legacy_result(NF_LEGACY_PATH_INVALID_ARGUMENT, 0U, 0U,
                                NF_LEGACY_PATH_STATE_ORTHOGONAL, 0U, 0U);
    }

    validation = nf_legacy_path_validate(input, input_capacity);
    if (validation.status != NF_LEGACY_PATH_OK) {
        return validation;
    }
    if (nf_legacy_ranges_overlap(input, input_capacity,
                                 output, output_capacity)) {
        return nf_legacy_result(NF_LEGACY_PATH_OVERLAPPING_BUFFERS, 0U, 0U,
                                NF_LEGACY_PATH_STATE_ORTHOGONAL, 0U, 0U);
    }
    if (!nf_legacy_normalize_pass(input, validation.length, &sizing) ||
        sizing.count == SIZE_MAX) {
        return nf_legacy_result(NF_LEGACY_PATH_OUTPUT_CAPACITY,
                                validation.length, 0U,
                                NF_LEGACY_PATH_STATE_ORTHOGONAL,
                                sizing.count, SIZE_MAX);
    }

    required_capacity = sizing.count + 1U;
    if (output_capacity < required_capacity) {
        return nf_legacy_result(NF_LEGACY_PATH_OUTPUT_CAPACITY,
                                validation.length, 0U,
                                NF_LEGACY_PATH_STATE_ORTHOGONAL,
                                sizing.count, required_capacity);
    }

    writer.output = output;
    writer.capacity = output_capacity - 1U;
    writer.count = 0U;
    writer.last_code = 0U;
    writer.write = true;
    if (!nf_legacy_normalize_pass(input, validation.length, &writer) ||
        writer.count != sizing.count) {
        /* The sizing pass guarantees this cannot occur for unchanged input. */
        return nf_legacy_result(NF_LEGACY_PATH_OUTPUT_CAPACITY,
                                validation.length, 0U,
                                NF_LEGACY_PATH_STATE_ORTHOGONAL,
                                sizing.count, required_capacity);
    }
    output[writer.count] = 0U;
    return nf_legacy_result(NF_LEGACY_PATH_OK, writer.count, 0U,
                            NF_LEGACY_PATH_STATE_ORTHOGONAL,
                            writer.count, required_capacity);
}

const char *nf_legacy_path_status_name(NfLegacyPathStatus status)
{
    switch (status) {
    case NF_LEGACY_PATH_OK: return "ok";
    case NF_LEGACY_PATH_INVALID_ARGUMENT: return "invalid argument";
    case NF_LEGACY_PATH_INPUT_NOT_TERMINATED: return "input not terminated";
    case NF_LEGACY_PATH_UNKNOWN_CODE: return "unknown code";
    case NF_LEGACY_PATH_INVALID_TRANSITION: return "invalid transition";
    case NF_LEGACY_PATH_ENDS_DIAGONAL: return "path ends in diagonal state";
    case NF_LEGACY_PATH_OUTPUT_CAPACITY: return "output capacity";
    case NF_LEGACY_PATH_OVERLAPPING_BUFFERS: return "overlapping buffers";
    default: return "invalid status";
    }
}
