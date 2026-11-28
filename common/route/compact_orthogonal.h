#ifndef NF_COMPACT_ORTHOGONAL_H
#define NF_COMPACT_ORTHOGONAL_H

#include "orthogonal_time_planner.h"

/* This workspace is for classic's 16x16 maze, not a silently truncated 32x32. */
#define NF_COMPACT_MAX_SIZE 16U
#define NF_COMPACT_STATES (NF_COMPACT_MAX_SIZE * NF_COMPACT_MAX_SIZE * 12U + 1U)
#define NF_COMPACT_PATH_CAPACITY 1024U

typedef struct {
    uint32_t distance_us[NF_COMPACT_STATES];
    uint32_t parent[NF_COMPACT_STATES];
    uint16_t heap[NF_COMPACT_STATES];
    uint16_t position[NF_COMPACT_STATES];
} NfCompactWorkspace;

typedef struct {
    uint32_t goal_entry_us;
    uint32_t stop_us;
    uint32_t expanded_states;
    uint32_t relaxed_edges;
    uint16_t path_length;
    uint8_t goal_x, goal_y, goal_heading;
    uint8_t post_goal_cells;
} NfCompactResult;

/* Foreground only. No allocation, HAL, NVM, globals, or motor access.
 * Output is canonical orthogonal legacy path[] with implicit first_sectionA
 * and final half_sectionD. Only successful calls modify output/result.
 * Costs are the shared NOMINAL motion model, not measured runner times.
 * Caller-owned input/workspace/output/result must not overlap. Reentrant
 * with separate workspaces. Start-in-goal returns an empty path; do not run it.
 */
NfRoutePlanStatus nf_compact_orthogonal_plan(
    const NfRouteMaze *maze, const NfOrthogonalPlannerConfig *config,
    const NfOrthogonalPlannerRequest *request, NfCompactWorkspace *workspace,
    uint16_t *output, size_t capacity, NfCompactResult *result);

#endif
