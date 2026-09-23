#include "slalom_plan_legacy_codec.h"

#include <stdlib.h>
#include <string.h>

typedef struct {
    uint16_t *codes;
    size_t count;
    size_t capacity;
    bool write;
    NfSlalomAnchor anchor;
    NfSlalomHeading8 heading;
} NfSlalomLegacyEmitter;

static const int8_t k_legacy_dx[8] = {0, 1, 1, 1, 0, -1, -1, -1};
static const int8_t k_legacy_dy[8] = {1, 1, 0, -1, -1, -1, 0, 1};

static NfSlalomLegacyResult nf_slalom_legacy_result(
    NfSlalomLegacyStatus status,
    size_t action_index,
    size_t length,
    size_t required_capacity,
    bool geometry_compatible)
{
    const NfSlalomLegacyResult result = {
        status,
        action_index,
        length,
        required_capacity,
        geometry_compatible,
        false,
    };
    return result;
}

static NfSlalomHeading8 nf_slalom_legacy_heading_add(
    NfSlalomHeading8 heading,
    int delta)
{
    int value = ((int)heading + delta) % 8;
    if (value < 0) {
        value += 8;
    }
    return (NfSlalomHeading8)value;
}

static bool nf_slalom_legacy_same_anchor(NfSlalomAnchor left,
                                         NfSlalomAnchor right)
{
    return left.half_x == right.half_x && left.half_y == right.half_y;
}

static bool nf_slalom_legacy_emit_code(NfSlalomLegacyEmitter *emitter,
                                       uint16_t code)
{
    if (emitter->count >= emitter->capacity) {
        return false;
    }
    if (emitter->write) {
        emitter->codes[emitter->count] = code;
    }
    emitter->count++;
    return true;
}

static void nf_slalom_legacy_advance(NfSlalomLegacyEmitter *emitter,
                                     NfSlalomHeading8 heading,
                                     uint16_t steps)
{
    emitter->anchor.half_x = (int16_t)(
        emitter->anchor.half_x +
        ((int)steps * k_legacy_dx[(unsigned int)heading]));
    emitter->anchor.half_y = (int16_t)(
        emitter->anchor.half_y +
        ((int)steps * k_legacy_dy[(unsigned int)heading]));
}

static bool nf_slalom_legacy_emit_connector(
    NfSlalomLegacyEmitter *emitter,
    bool diagonal,
    uint16_t steps)
{
    uint16_t code;
    if (steps == 0U) {
        return true;
    }
    if (steps > 99U) {
        return false;
    }
    code = (uint16_t)((diagonal ?
        NF_LEGACY_PATH_DIAGONAL_STRAIGHT_BASE :
        NF_LEGACY_PATH_STRAIGHT_BASE) + steps);
    if (!nf_slalom_legacy_emit_code(emitter, code)) {
        return false;
    }
    nf_slalom_legacy_advance(emitter, emitter->heading, steps);
    return true;
}

static uint16_t nf_slalom_legacy_turn_code(NfSlalomActionKind kind,
                                           NfRouteSide side)
{
    const bool right = side == NF_ROUTE_SIDE_RIGHT;
    if (side != NF_ROUTE_SIDE_RIGHT && side != NF_ROUTE_SIDE_LEFT) {
        return 0U;
    }
    switch (kind) {
    case NF_SLALOM_ACTION_SMALL_90:
        return right ? NF_LEGACY_PATH_SMALL_RIGHT_90 :
                       NF_LEGACY_PATH_SMALL_LEFT_90;
    case NF_SLALOM_ACTION_LARGE_90:
        return right ? NF_LEGACY_PATH_LARGE_RIGHT_90 :
                       NF_LEGACY_PATH_LARGE_LEFT_90;
    case NF_SLALOM_ACTION_LARGE_180:
        return right ? NF_LEGACY_PATH_LARGE_RIGHT_180 :
                       NF_LEGACY_PATH_LARGE_LEFT_180;
    case NF_SLALOM_ACTION_45_IN:
        return right ? NF_LEGACY_PATH_RIGHT_45_IN :
                       NF_LEGACY_PATH_LEFT_45_IN;
    case NF_SLALOM_ACTION_45_OUT:
        return right ? NF_LEGACY_PATH_RIGHT_45_OUT :
                       NF_LEGACY_PATH_LEFT_45_OUT;
    case NF_SLALOM_ACTION_V90:
        return right ? NF_LEGACY_PATH_RIGHT_V90 :
                       NF_LEGACY_PATH_LEFT_V90;
    case NF_SLALOM_ACTION_135_IN:
        return right ? NF_LEGACY_PATH_RIGHT_135_IN :
                       NF_LEGACY_PATH_LEFT_135_IN;
    case NF_SLALOM_ACTION_135_OUT:
        return right ? NF_LEGACY_PATH_RIGHT_135_OUT :
                       NF_LEGACY_PATH_LEFT_135_OUT;
    default:
        return 0U;
    }
}

static bool nf_slalom_legacy_apply_turn(NfSlalomLegacyEmitter *emitter,
                                        NfSlalomActionKind kind,
                                        NfRouteSide side)
{
    const int sign = (side == NF_ROUTE_SIDE_RIGHT) ? 1 : -1;
    NfSlalomHeading8 end;
    NfSlalomHeading8 intermediate;

    switch (kind) {
    case NF_SLALOM_ACTION_SMALL_90:
        end = nf_slalom_legacy_heading_add(emitter->heading, 2 * sign);
        nf_slalom_legacy_advance(emitter, emitter->heading, 1U);
        nf_slalom_legacy_advance(emitter, end, 1U);
        break;
    case NF_SLALOM_ACTION_LARGE_90:
        end = nf_slalom_legacy_heading_add(emitter->heading, 2 * sign);
        nf_slalom_legacy_advance(emitter, emitter->heading, 2U);
        nf_slalom_legacy_advance(emitter, end, 2U);
        break;
    case NF_SLALOM_ACTION_LARGE_180:
        intermediate = nf_slalom_legacy_heading_add(
            emitter->heading, 2 * sign);
        end = nf_slalom_legacy_heading_add(emitter->heading, 4 * sign);
        nf_slalom_legacy_advance(emitter, intermediate, 2U);
        break;
    case NF_SLALOM_ACTION_45_IN:
    case NF_SLALOM_ACTION_45_OUT:
        end = nf_slalom_legacy_heading_add(emitter->heading, sign);
        nf_slalom_legacy_advance(emitter, emitter->heading, 1U);
        nf_slalom_legacy_advance(emitter, end, 1U);
        break;
    case NF_SLALOM_ACTION_V90:
        end = nf_slalom_legacy_heading_add(emitter->heading, 2 * sign);
        nf_slalom_legacy_advance(emitter, emitter->heading, 1U);
        nf_slalom_legacy_advance(emitter, end, 1U);
        break;
    case NF_SLALOM_ACTION_135_IN:
        intermediate = nf_slalom_legacy_heading_add(
            emitter->heading, sign);
        end = nf_slalom_legacy_heading_add(emitter->heading, 3 * sign);
        nf_slalom_legacy_advance(emitter, emitter->heading, 1U);
        nf_slalom_legacy_advance(emitter, intermediate, 1U);
        nf_slalom_legacy_advance(emitter, end, 1U);
        break;
    case NF_SLALOM_ACTION_135_OUT:
        intermediate = nf_slalom_legacy_heading_add(
            emitter->heading, sign);
        end = nf_slalom_legacy_heading_add(emitter->heading, 3 * sign);
        nf_slalom_legacy_advance(emitter, intermediate, 2U);
        nf_slalom_legacy_advance(emitter, end, 1U);
        break;
    default:
        return false;
    }
    emitter->heading = end;
    return true;
}

static NfSlalomLegacyStatus nf_slalom_legacy_encode_pass(
    const NfSlalomRoutePlan *plan,
    const NfSlalomLegacyContract *contract,
    NfSlalomLegacyEmitter *emitter,
    size_t *out_action_index)
{
    for (size_t i = 1U; i < plan->action_count; i++) {
        const NfSlalomAction *action = &plan->actions[i];
        uint16_t connector_steps = action->connector_steps;

        *out_action_index = i;
        if (!nf_slalom_legacy_same_anchor(emitter->anchor,
                                           action->start_anchor) ||
            emitter->heading != action->start_heading) {
            return NF_SLALOM_LEGACY_INTERNAL_MISMATCH;
        }
        if (action->kind == NF_SLALOM_ACTION_GOAL_STOP) {
            if (action->connector_is_diagonal) {
                return NF_SLALOM_LEGACY_TERMINAL_DIAGONAL_UNSUPPORTED;
            }
            if (contract->implicit_orthogonal_stop_steps == 0U ||
                connector_steps < contract->implicit_orthogonal_stop_steps) {
                return NF_SLALOM_LEGACY_TERMINAL_OWNERSHIP;
            }
            connector_steps = (uint16_t)(
                connector_steps - contract->implicit_orthogonal_stop_steps);
            if (!nf_slalom_legacy_emit_connector(emitter, false,
                                                  connector_steps)) {
                return (connector_steps > 99U) ?
                    NF_SLALOM_LEGACY_CONNECTOR_CODE_RANGE :
                    NF_SLALOM_LEGACY_CODE_LIMIT;
            }
            nf_slalom_legacy_advance(
                emitter, emitter->heading,
                contract->implicit_orthogonal_stop_steps);
            if (!nf_slalom_legacy_same_anchor(emitter->anchor,
                                               action->end_anchor) ||
                emitter->heading != action->end_heading) {
                return NF_SLALOM_LEGACY_INTERNAL_MISMATCH;
            }
            continue;
        }
        if ((unsigned int)action->kind >= NF_SLALOM_ACTION_START_OFFSET) {
            return NF_SLALOM_LEGACY_INTERNAL_MISMATCH;
        }
        if (!nf_slalom_legacy_emit_connector(
                emitter, action->connector_is_diagonal, connector_steps)) {
            return (connector_steps > 99U) ?
                NF_SLALOM_LEGACY_CONNECTOR_CODE_RANGE :
                NF_SLALOM_LEGACY_CODE_LIMIT;
        }
        if (!nf_slalom_legacy_same_anchor(emitter->anchor,
                                           action->connector_end_anchor)) {
            return NF_SLALOM_LEGACY_INTERNAL_MISMATCH;
        }
        {
            const uint16_t turn_code = nf_slalom_legacy_turn_code(
                action->kind, action->side);
            if (turn_code == 0U ||
                !nf_slalom_legacy_emit_code(emitter, turn_code) ||
                !nf_slalom_legacy_apply_turn(emitter, action->kind,
                                              action->side)) {
                return NF_SLALOM_LEGACY_CODE_LIMIT;
            }
        }
        if (!nf_slalom_legacy_same_anchor(emitter->anchor,
                                           action->end_anchor) ||
            emitter->heading != action->end_heading) {
            return NF_SLALOM_LEGACY_INTERNAL_MISMATCH;
        }
    }
    return NF_SLALOM_LEGACY_OK;
}

NfSlalomLegacyResult nf_slalom_plan_to_legacy(
    const NfRouteMaze *maze,
    const NfSlalomPlannerConfig *config,
    const NfSlalomPlannerRequest *request,
    const NfSlalomRoutePlan *plan,
    const NfSlalomLegacyContract *contract,
    uint16_t *output,
    size_t output_capacity)
{
    NfSlalomValidation validation;
    NfSlalomLegacyEmitter sizing;
    NfSlalomLegacyEmitter writer;
    NfSlalomLegacyStatus status;
    NfLegacyPathResult grammar;
    uint16_t *temporary;
    size_t action_index = 0U;
    size_t required_capacity;

    if (maze == NULL || config == NULL || request == NULL || plan == NULL ||
        contract == NULL || output == NULL ||
        contract->max_nonzero_codes == 0U) {
        return nf_slalom_legacy_result(
            NF_SLALOM_LEGACY_INVALID_ARGUMENT, 0U, 0U, 0U, false);
    }
    if (!nf_slalom_route_validate(maze, config, request, plan,
                                  &validation)) {
        return nf_slalom_legacy_result(
            NF_SLALOM_LEGACY_ROUTE_INVALID, validation.action_index,
            0U, 0U, false);
    }
    if (plan->action_count == 0U) {
        return nf_slalom_legacy_result(
            NF_SLALOM_LEGACY_NO_RUN_REQUIRED, 0U, 0U, 1U, false);
    }
    if (plan->actions[0].kind != NF_SLALOM_ACTION_START_OFFSET) {
        return nf_slalom_legacy_result(
            NF_SLALOM_LEGACY_INTERNAL_MISMATCH, 0U, 0U, 0U, false);
    }

    memset(&sizing, 0, sizeof(sizing));
    sizing.capacity = contract->max_nonzero_codes;
    sizing.anchor = plan->actions[0].end_anchor;
    sizing.heading = plan->actions[0].end_heading;
    status = nf_slalom_legacy_encode_pass(plan, contract, &sizing,
                                           &action_index);
    required_capacity = sizing.count + 1U;
    if (status != NF_SLALOM_LEGACY_OK) {
        return nf_slalom_legacy_result(status, action_index, sizing.count,
                                       required_capacity, false);
    }
    if (output_capacity < required_capacity) {
        return nf_slalom_legacy_result(
            NF_SLALOM_LEGACY_OUTPUT_CAPACITY, plan->action_count,
            sizing.count, required_capacity, false);
    }
    temporary = (uint16_t *)malloc(required_capacity * sizeof(*temporary));
    if (temporary == NULL) {
        return nf_slalom_legacy_result(
            NF_SLALOM_LEGACY_OUTPUT_CAPACITY, plan->action_count,
            sizing.count, required_capacity, false);
    }
    memset(&writer, 0, sizeof(writer));
    writer.codes = temporary;
    writer.capacity = sizing.count;
    writer.write = true;
    writer.anchor = plan->actions[0].end_anchor;
    writer.heading = plan->actions[0].end_heading;
    status = nf_slalom_legacy_encode_pass(plan, contract, &writer,
                                           &action_index);
    temporary[writer.count] = 0U;
    grammar = nf_legacy_path_validate(temporary, required_capacity);
    if (status != NF_SLALOM_LEGACY_OK || writer.count != sizing.count ||
        grammar.status != NF_LEGACY_PATH_OK) {
        free(temporary);
        return nf_slalom_legacy_result(
            NF_SLALOM_LEGACY_INTERNAL_MISMATCH, action_index,
            writer.count, required_capacity, false);
    }
    memcpy(output, temporary, required_capacity * sizeof(*output));
    free(temporary);
    return nf_slalom_legacy_result(
        NF_SLALOM_LEGACY_OK, plan->action_count, sizing.count,
        required_capacity, true);
}

const char *nf_slalom_legacy_status_name(NfSlalomLegacyStatus status)
{
    switch (status) {
    case NF_SLALOM_LEGACY_OK: return "ok";
    case NF_SLALOM_LEGACY_INVALID_ARGUMENT: return "invalid-argument";
    case NF_SLALOM_LEGACY_ROUTE_INVALID: return "route-invalid";
    case NF_SLALOM_LEGACY_NO_RUN_REQUIRED: return "no-run-required";
    case NF_SLALOM_LEGACY_TERMINAL_DIAGONAL_UNSUPPORTED:
        return "terminal-diagonal-unsupported";
    case NF_SLALOM_LEGACY_TERMINAL_OWNERSHIP:
        return "terminal-ownership";
    case NF_SLALOM_LEGACY_CONNECTOR_CODE_RANGE:
        return "connector-code-range";
    case NF_SLALOM_LEGACY_CODE_LIMIT: return "code-limit";
    case NF_SLALOM_LEGACY_OUTPUT_CAPACITY: return "output-capacity";
    case NF_SLALOM_LEGACY_INTERNAL_MISMATCH: return "internal-mismatch";
    default: return "unknown";
    }
}
