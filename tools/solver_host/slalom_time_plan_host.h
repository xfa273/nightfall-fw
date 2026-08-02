#ifndef NIGHTFALL_TOOLS_SOLVER_HOST_SLALOM_TIME_PLAN_HOST_H
#define NIGHTFALL_TOOLS_SOLVER_HOST_SLALOM_TIME_PLAN_HOST_H

#include "slalom_profile_baseline.h"
#include "slalom_time_planner.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Build the host-only planner configuration from the compiled shortest-run
 * table.  Orthogonal turns retain calibrated total timing; all eight turns
 * also receive explicitly provisional, exact-closing PC geometry seeds.
 */
bool nf_host_slalom_make_config(
    const char *profile_name,
    uint8_t case_index,
    uint32_t enabled_actions,
    NfSlalomPlannerConfig *out_config,
    const NfAuditProfile **out_profile,
    char *error,
    size_t error_size);

int nf_host_run_slalom_time_plan(const char *maze_path,
                                 const char *profile_name,
                                 uint8_t case_index,
                                 bool compare_orthogonal,
                                 bool assert_valid,
                                 bool summary_only,
                                 bool check_turn_clearance);

#ifdef __cplusplus
}
#endif

#endif
