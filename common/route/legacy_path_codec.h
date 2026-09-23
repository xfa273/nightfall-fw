#ifndef NIGHTFALL_COMMON_ROUTE_LEGACY_PATH_CODEC_H_
#define NIGHTFALL_COMMON_ROUTE_LEGACY_PATH_CODEC_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Canonical, zero-terminated legacy path codes shared by F405 and F413.
 * Straight values store a positive number of half sections in the low part.
 */
enum {
    NF_LEGACY_PATH_STRAIGHT_BASE = 200,
    NF_LEGACY_PATH_STRAIGHT_MAX = 299,
    NF_LEGACY_PATH_SMALL_RIGHT_90 = 300,
    NF_LEGACY_PATH_SMALL_LEFT_90 = 400,
    NF_LEGACY_PATH_LARGE_RIGHT_90 = 501,
    NF_LEGACY_PATH_LARGE_RIGHT_180 = 502,
    NF_LEGACY_PATH_LARGE_LEFT_90 = 601,
    NF_LEGACY_PATH_LARGE_LEFT_180 = 602,
    NF_LEGACY_PATH_RIGHT_45_IN = 701,
    NF_LEGACY_PATH_LEFT_45_IN = 702,
    NF_LEGACY_PATH_RIGHT_45_OUT = 703,
    NF_LEGACY_PATH_LEFT_45_OUT = 704,
    NF_LEGACY_PATH_RIGHT_V90 = 801,
    NF_LEGACY_PATH_LEFT_V90 = 802,
    NF_LEGACY_PATH_RIGHT_135_IN = 901,
    NF_LEGACY_PATH_LEFT_135_IN = 902,
    NF_LEGACY_PATH_RIGHT_135_OUT = 903,
    NF_LEGACY_PATH_LEFT_135_OUT = 904,
    NF_LEGACY_PATH_DIAGONAL_STRAIGHT_BASE = 1000,
    NF_LEGACY_PATH_DIAGONAL_STRAIGHT_MAX = 1099,
};

typedef enum {
    NF_LEGACY_PATH_STATE_ORTHOGONAL = 0,
    NF_LEGACY_PATH_STATE_DIAGONAL,
} NfLegacyPathState;

typedef enum {
    NF_LEGACY_PATH_OK = 0,
    NF_LEGACY_PATH_INVALID_ARGUMENT,
    NF_LEGACY_PATH_INPUT_NOT_TERMINATED,
    NF_LEGACY_PATH_UNKNOWN_CODE,
    NF_LEGACY_PATH_INVALID_TRANSITION,
    NF_LEGACY_PATH_ENDS_DIAGONAL,
    NF_LEGACY_PATH_OUTPUT_CAPACITY,
    NF_LEGACY_PATH_OVERLAPPING_BUFFERS,
} NfLegacyPathStatus;

typedef struct {
    NfLegacyPathStatus status;
    size_t index;
    uint16_t code;
    NfLegacyPathState state;
    /* Number of non-zero codes in the validated or normalized path. */
    size_t length;
    /* Output elements required including the zero terminator. */
    size_t required_capacity;
} NfLegacyPathResult;

/*
 * Validate the strict shared path grammar without modifying path.
 * capacity includes the slot which must contain the zero terminator.
 */
NfLegacyPathResult nf_legacy_path_validate(const uint16_t *path,
                                           size_t capacity);

/*
 * Convert complete, straight-bounded runs of small turns to diagonal actions.
 *
 * Runs without both an entry and an exit (including a run at path end), runs
 * whose connectors cannot supply the required half sections, and runs whose
 * nominal half-grid endpoint would change are copied as orthogonal small
 * turns. Generated diagonal straights are split so each code remains in the
 * shared 1001..1099 range. Input may already contain canonical diagonal
 * actions; those actions are preserved.
 *
 * input and output must not overlap. The function performs a sizing pass first
 * and leaves output completely unchanged for every failure status.
 */
NfLegacyPathResult nf_legacy_path_normalize_diagonal(
    const uint16_t *input,
    size_t input_capacity,
    uint16_t *output,
    size_t output_capacity);

const char *nf_legacy_path_status_name(NfLegacyPathStatus status);

#ifdef __cplusplus
}
#endif

#endif /* NIGHTFALL_COMMON_ROUTE_LEGACY_PATH_CODEC_H_ */
