#ifndef NIGHTFALL_COMMON_ROUTE_SLALOM_PLAN_LEGACY_CODEC_H
#define NIGHTFALL_COMMON_ROUTE_SLALOM_PLAN_LEGACY_CODEC_H

#include "legacy_path_codec.h"
#include "slalom_time_planner.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NF_SLALOM_LEGACY_OK = 0,
    NF_SLALOM_LEGACY_INVALID_ARGUMENT,
    NF_SLALOM_LEGACY_ROUTE_INVALID,
    NF_SLALOM_LEGACY_NO_RUN_REQUIRED,
    NF_SLALOM_LEGACY_TERMINAL_DIAGONAL_UNSUPPORTED,
    NF_SLALOM_LEGACY_TERMINAL_OWNERSHIP,
    NF_SLALOM_LEGACY_CONNECTOR_CODE_RANGE,
    NF_SLALOM_LEGACY_CODE_LIMIT,
    NF_SLALOM_LEGACY_OUTPUT_CAPACITY,
    NF_SLALOM_LEGACY_INTERNAL_MISMATCH,
} NfSlalomLegacyStatus;

typedef struct {
    /* Existing F413 command ingestion accepts at most 255 non-zero codes. */
    size_t max_nonzero_codes;
    /* Both current runners append one 45 mm orthogonal stop half-section. */
    uint16_t implicit_orthogonal_stop_steps;
} NfSlalomLegacyContract;

typedef struct {
    NfSlalomLegacyStatus status;
    size_t action_index;
    size_t length;
    size_t required_capacity;
    bool geometry_compatible;
    /* False for the current runners, which re-plan each straight code. */
    bool time_equivalent;
} NfSlalomLegacyResult;

/*
 * Transactionally encode a validated typed route for the current legacy
 * runner contract.  This is a geometry-compatibility gate only: the legacy
 * runners do not preserve the typed connector entry/exit velocity profile.
 *
 * A diagonal terminal is rejected because the existing grammar must return
 * to orthogonal state and both runners append an orthogonal 45 mm stop.
 */
NfSlalomLegacyResult nf_slalom_plan_to_legacy(
    const NfRouteMaze *maze,
    const NfSlalomPlannerConfig *config,
    const NfSlalomPlannerRequest *request,
    const NfSlalomRoutePlan *plan,
    const NfSlalomLegacyContract *contract,
    uint16_t *output,
    size_t output_capacity);

const char *nf_slalom_legacy_status_name(NfSlalomLegacyStatus status);

#ifdef __cplusplus
}
#endif

#endif
