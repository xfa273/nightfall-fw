#include "legacy_path_validator.h"

#include "legacy_path_codec.h"

static legacy_path_validation_error_t map_status(NfLegacyPathStatus status)
{
    switch (status) {
    case NF_LEGACY_PATH_OK:
        return LEGACY_PATH_VALIDATION_OK;
    case NF_LEGACY_PATH_INVALID_ARGUMENT:
        return LEGACY_PATH_VALIDATION_NULL_PATH;
    case NF_LEGACY_PATH_INPUT_NOT_TERMINATED:
        return LEGACY_PATH_VALIDATION_ARRAY_LIMIT_EXCEEDED;
    case NF_LEGACY_PATH_UNKNOWN_CODE:
        return LEGACY_PATH_VALIDATION_UNKNOWN_CODE;
    case NF_LEGACY_PATH_INVALID_TRANSITION:
        return LEGACY_PATH_VALIDATION_INVALID_TRANSITION;
    case NF_LEGACY_PATH_ENDS_DIAGONAL:
        return LEGACY_PATH_VALIDATION_ENDS_DIAGONAL;
    case NF_LEGACY_PATH_OUTPUT_CAPACITY:
    case NF_LEGACY_PATH_OVERLAPPING_BUFFERS:
    default:
        return LEGACY_PATH_VALIDATION_INVALID_TRANSITION;
    }
}

legacy_path_validation_result_t legacy_path_validate(const uint16_t *path,
                                                     size_t capacity)
{
    const NfLegacyPathResult common =
        nf_legacy_path_validate(path, capacity);
    legacy_path_validation_result_t result;

    result.error = map_status(common.status);
    result.index = common.index;
    result.code = common.code;
    result.state = (common.state == NF_LEGACY_PATH_STATE_DIAGONAL)
                       ? LEGACY_PATH_STATE_DIAGONAL
                       : LEGACY_PATH_STATE_ORTHOGONAL;
    return result;
}

const char *legacy_path_validation_error_string(
    legacy_path_validation_error_t error)
{
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
