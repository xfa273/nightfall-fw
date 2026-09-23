/*
 * path.c
 *
 *  Created on: Mar 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "legacy_path_codec.h"

#include <string.h>

static bool path_find_terminator(size_t *length)
{
    size_t index = 0U;

    while (index < ROUTE_MAX_LEN && path[index] != 0U) {
        index++;
    }
    if (index == ROUTE_MAX_LEN) {
        return false;
    }
    *length = index;
    return true;
}

static bool path_is_straight(uint16_t code)
{
    return code > STRAIGHT && code < TURN_R;
}

static bool path_emit_simplified(uint16_t *output,
                                 size_t *output_length,
                                 uint16_t action,
                                 size_t count,
                                 bool first_run)
{
    if (action != STRAIGHT) {
        if (*output_length >= ROUTE_MAX_LEN - 1U) {
            return false;
        }
        output[(*output_length)++] = action;
        return true;
    }

    {
        size_t half_steps = 2U * count;
        if (first_run) {
            /* first_sectionA already owns the first half section */
            half_steps--;
        }
        while (half_steps != 0U) {
            const size_t chunk = (half_steps > 99U) ? 99U : half_steps;
            if (*output_length >= ROUTE_MAX_LEN - 1U) {
                return false;
            }
            output[(*output_length)++] =
                (uint16_t)(STRAIGHT + chunk);
            half_steps -= chunk;
        }
    }
    return true;
}

void simplifyPath(void)
{
    static uint16_t simplifiedPath[ROUTE_MAX_LEN];
    size_t path_length;
    size_t current_index = 0U;
    size_t count = 1U;
    size_t i;
    uint16_t current_action;
    bool first_run = true;

    if (!path_find_terminator(&path_length) || path_length == 0U) {
        return;
    }

    current_action = path[0];
    for (i = 1U; i < path_length; i++) {
        if (path[i] == current_action && current_action == STRAIGHT) {
            count++;
        } else {
            if (!path_emit_simplified(simplifiedPath, &current_index,
                                      current_action, count, first_run)) {
                return;
            }
            first_run = false;
            current_action = path[i];
            count = 1U;
        }
    }
    if (!path_emit_simplified(simplifiedPath, &current_index,
                              current_action, count, first_run)) {
        return;
    }

    memcpy(path, simplifiedPath, current_index * sizeof(path[0]));
    memset(&path[current_index], 0,
           (ROUTE_MAX_LEN - current_index) * sizeof(path[0]));

}

void convertLTurn(void)
{
    static uint16_t convertedPath[ROUTE_MAX_LEN];
    size_t path_length;
    size_t i = 0U;
    size_t j = 0U;
    size_t output_length = 0U;

    if (!path_find_terminator(&path_length)) {
        return;
    }

    while (i < path_length) {
        if (path[i] == 300U && i + 1U < path_length &&
            path[i + 1U] == 300U) {
            if (i > 0U && j > 0U && i + 2U < path_length &&
                path_is_straight(path[i - 1U]) &&
                path_is_straight(path[i + 2U])) {
                convertedPath[j - 1U]--;
                convertedPath[j] = 502U;
                path[i + 2U]--;
                i++;
            } else {
                convertedPath[j] = path[i];
            }
        } else if (path[i] == 400U && i + 1U < path_length &&
                   path[i + 1U] == 400U) {
            if (i > 0U && j > 0U && i + 2U < path_length &&
                path_is_straight(path[i - 1U]) &&
                path_is_straight(path[i + 2U])) {
                convertedPath[j - 1U]--;
                convertedPath[j] = 602U;
                path[i + 2U]--;
                i++;
            } else {
                convertedPath[j] = path[i];
            }
        } else if (path[i] == 300U) {
            if (i > 0U && j > 0U && i + 1U < path_length &&
                path_is_straight(path[i - 1U]) &&
                path_is_straight(path[i + 1U])) {
                convertedPath[j - 1U]--;
                convertedPath[j] = 501U;
                path[i + 1U]--;
            } else {
                convertedPath[j] = path[i];
            }
        } else if (path[i] == 400U) {
            if (i > 0U && j > 0U && i + 1U < path_length &&
                path_is_straight(path[i - 1U]) &&
                path_is_straight(path[i + 1U])) {
                convertedPath[j - 1U]--;
                convertedPath[j] = 601U;
                path[i + 1U]--;
            } else {
                convertedPath[j] = path[i];
            }
        } else {
            convertedPath[j] = path[i];
        }
        i++;
        j++;
    }

    /* A large turn may consume an entire S1 connector, encoded temporarily as 200. */
    for (i = 0U; i < j; i++) {
        if (convertedPath[i] != STRAIGHT) {
            path[output_length++] = convertedPath[i];
        }
    }
    memset(&path[output_length], 0,
           (ROUTE_MAX_LEN - output_length) * sizeof(path[0]));
}

void normalizeStartLargeTurnException(void)
{
    uint16_t small_turn = 0U;
    size_t end;
    bool has_straight_after_turn;
    size_t required_space;

    if (path[0] == 501U) {
        small_turn = 300U;
    } else if (path[0] == 601U) {
        small_turn = 400U;
    } else {
        return;
    }

    if (!path_find_terminator(&end)) {
        return;
    }
    has_straight_after_turn = path_is_straight(path[1]);
    required_space = has_straight_after_turn ? 1U : 2U;
    if (end >= ROUTE_MAX_LEN - required_space) {
        return;
    }

    if (has_straight_after_turn) {
        path[1]++;
    }
    for (size_t i = end + 1U; i > 0U; i--) {
        path[i] = path[i - 1U];
    }
    path[0] = 201U;
    path[1] = small_turn;

    if (!has_straight_after_turn) {
        end++;
        for (size_t i = end + 1U; i > 2U; i--) {
            path[i] = path[i - 1U];
        }
        path[2] = 201U;
    }
}

void convertDiagonal(void)
{
    static uint16_t convertedPath[ROUTE_MAX_LEN];
    const NfLegacyPathResult result = nf_legacy_path_normalize_diagonal(
        path, ROUTE_MAX_LEN, convertedPath, ROUTE_MAX_LEN);

    if (result.status != NF_LEGACY_PATH_OK) {
        return;
    }
    memcpy(path, convertedPath,
           (result.length + 1U) * sizeof(path[0]));
    if (result.length + 1U < ROUTE_MAX_LEN) {
        memset(&path[result.length + 1U], 0,
               (ROUTE_MAX_LEN - result.length - 1U) * sizeof(path[0]));
    }
}

/* makePath() was removed; route derivation uses solver_build_path(). */
