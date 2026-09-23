#ifndef NIGHTFALL_TOOLS_SOLVER_HOST_LEGACY_PATH_VALIDATOR_H_
#define NIGHTFALL_TOOLS_SOLVER_HOST_LEGACY_PATH_VALIDATOR_H_

#include <stddef.h>
#include <stdint.h>

typedef enum {
    LEGACY_PATH_STATE_ORTHOGONAL = 0,
    LEGACY_PATH_STATE_DIAGONAL,
} legacy_path_state_t;

typedef enum {
    LEGACY_PATH_VALIDATION_OK = 0,
    LEGACY_PATH_VALIDATION_NULL_PATH,
    LEGACY_PATH_VALIDATION_UNKNOWN_CODE,
    LEGACY_PATH_VALIDATION_INVALID_TRANSITION,
    LEGACY_PATH_VALIDATION_ENDS_DIAGONAL,
    LEGACY_PATH_VALIDATION_ARRAY_LIMIT_EXCEEDED,
} legacy_path_validation_error_t;

typedef struct {
    legacy_path_validation_error_t error;
    size_t index;
    uint16_t code;
    legacy_path_state_t state;
} legacy_path_validation_result_t;

/*
 * Validate a legacy, zero-terminated path without modifying it.
 *
 * capacity is the number of readable uint16_t elements, including the slot
 * that must contain the terminating zero. If no zero is found within capacity,
 * LEGACY_PATH_VALIDATION_ARRAY_LIMIT_EXCEEDED is returned without reading past
 * the supplied array.
 */
legacy_path_validation_result_t legacy_path_validate(const uint16_t *path,
                                                     size_t capacity);

const char *legacy_path_validation_error_string(
    legacy_path_validation_error_t error);

#endif /* NIGHTFALL_TOOLS_SOLVER_HOST_LEGACY_PATH_VALIDATOR_H_ */
