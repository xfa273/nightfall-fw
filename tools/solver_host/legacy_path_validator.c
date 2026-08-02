#include "legacy_path_validator.h"

#include "path.h"

enum {
    LEGACY_PATH_CODE_R45_IN = 701,
    LEGACY_PATH_CODE_L45_IN = 702,
    LEGACY_PATH_CODE_R45_OUT = 703,
    LEGACY_PATH_CODE_L45_OUT = 704,
    LEGACY_PATH_CODE_RV90 = 801,
    LEGACY_PATH_CODE_LV90 = 802,
    LEGACY_PATH_CODE_R135_IN = 901,
    LEGACY_PATH_CODE_L135_IN = 902,
    LEGACY_PATH_CODE_R135_OUT = 903,
    LEGACY_PATH_CODE_L135_OUT = 904,
    LEGACY_PATH_CODE_DIAGONAL_STRAIGHT_BASE = 1000,
};

_Static_assert(STRAIGHT == 200, "legacy straight code changed");
_Static_assert(TURN_R == 300, "legacy right small-turn code changed");
_Static_assert(TURN_L == 400, "legacy left small-turn code changed");
_Static_assert(L_TURN_R == 500, "legacy right large-turn base changed");
_Static_assert(L_TURN_L == 600, "legacy left large-turn base changed");

typedef enum {
    LEGACY_PATH_CODE_CLASS_UNKNOWN = 0,
    LEGACY_PATH_CODE_CLASS_ORTHOGONAL,
    LEGACY_PATH_CODE_CLASS_DIAGONAL_ENTRY,
    LEGACY_PATH_CODE_CLASS_DIAGONAL,
    LEGACY_PATH_CODE_CLASS_DIAGONAL_EXIT,
} legacy_path_code_class_t;

static legacy_path_validation_result_t make_result(
    legacy_path_validation_error_t error,
    size_t index,
    uint16_t code,
    legacy_path_state_t state) {
    legacy_path_validation_result_t result;

    result.error = error;
    result.index = index;
    result.code = code;
    result.state = state;
    return result;
}

static legacy_path_code_class_t classify_code(uint16_t code) {
    if (code > STRAIGHT && code < TURN_R) {
        return LEGACY_PATH_CODE_CLASS_ORTHOGONAL;
    }

    switch (code) {
        case TURN_R:
        case TURN_L:
        case L_TURN_R + 1:
        case L_TURN_R + 2:
        case L_TURN_L + 1:
        case L_TURN_L + 2:
            return LEGACY_PATH_CODE_CLASS_ORTHOGONAL;

        case LEGACY_PATH_CODE_R45_IN:
        case LEGACY_PATH_CODE_L45_IN:
        case LEGACY_PATH_CODE_R135_IN:
        case LEGACY_PATH_CODE_L135_IN:
            return LEGACY_PATH_CODE_CLASS_DIAGONAL_ENTRY;

        case LEGACY_PATH_CODE_RV90:
        case LEGACY_PATH_CODE_LV90:
            return LEGACY_PATH_CODE_CLASS_DIAGONAL;

        case LEGACY_PATH_CODE_R45_OUT:
        case LEGACY_PATH_CODE_L45_OUT:
        case LEGACY_PATH_CODE_R135_OUT:
        case LEGACY_PATH_CODE_L135_OUT:
            return LEGACY_PATH_CODE_CLASS_DIAGONAL_EXIT;

        default:
            if (code > LEGACY_PATH_CODE_DIAGONAL_STRAIGHT_BASE) {
                return LEGACY_PATH_CODE_CLASS_DIAGONAL;
            }
            return LEGACY_PATH_CODE_CLASS_UNKNOWN;
    }
}

legacy_path_validation_result_t legacy_path_validate(const uint16_t *path,
                                                     size_t capacity) {
    legacy_path_state_t state = LEGACY_PATH_STATE_ORTHOGONAL;

    if (path == NULL) {
        return make_result(LEGACY_PATH_VALIDATION_NULL_PATH, 0U, 0U, state);
    }

    for (size_t index = 0U; index < capacity; ++index) {
        const uint16_t code = path[index];
        legacy_path_code_class_t code_class;

        if (code == 0U) {
            if (state == LEGACY_PATH_STATE_DIAGONAL) {
                return make_result(LEGACY_PATH_VALIDATION_ENDS_DIAGONAL,
                                   index, code, state);
            }
            return make_result(LEGACY_PATH_VALIDATION_OK, index, code, state);
        }

        code_class = classify_code(code);
        if (code_class == LEGACY_PATH_CODE_CLASS_UNKNOWN) {
            return make_result(LEGACY_PATH_VALIDATION_UNKNOWN_CODE, index,
                               code, state);
        }

        if (state == LEGACY_PATH_STATE_ORTHOGONAL) {
            if (code_class == LEGACY_PATH_CODE_CLASS_ORTHOGONAL) {
                continue;
            }
            if (code_class == LEGACY_PATH_CODE_CLASS_DIAGONAL_ENTRY) {
                state = LEGACY_PATH_STATE_DIAGONAL;
                continue;
            }
        } else {
            if (code_class == LEGACY_PATH_CODE_CLASS_DIAGONAL) {
                continue;
            }
            if (code_class == LEGACY_PATH_CODE_CLASS_DIAGONAL_EXIT) {
                state = LEGACY_PATH_STATE_ORTHOGONAL;
                continue;
            }
        }

        return make_result(LEGACY_PATH_VALIDATION_INVALID_TRANSITION, index,
                           code, state);
    }

    return make_result(LEGACY_PATH_VALIDATION_ARRAY_LIMIT_EXCEEDED, capacity,
                       0U, state);
}

const char *legacy_path_validation_error_string(
    legacy_path_validation_error_t error) {
    switch (error) {
        case LEGACY_PATH_VALIDATION_OK:
            return "ok";
        case LEGACY_PATH_VALIDATION_NULL_PATH:
            return "null path";
        case LEGACY_PATH_VALIDATION_UNKNOWN_CODE:
            return "unknown code";
        case LEGACY_PATH_VALIDATION_INVALID_TRANSITION:
            return "invalid transition";
        case LEGACY_PATH_VALIDATION_ENDS_DIAGONAL:
            return "path ends in diagonal state";
        case LEGACY_PATH_VALIDATION_ARRAY_LIMIT_EXCEEDED:
            return "array limit exceeded before terminator";
        default:
            return "invalid validation error";
    }
}
