#include "time_plan_host.h"

#include "maze_ascii.h"
#include "orthogonal_time_planner.h"

#include "params.h"
#include "shortest_run_params.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#define NF_HOST_F413_OMEGA_CAP_DEG_S 2200.0

static const ShortestRunModeParams_t *nf_host_mode_params(uint8_t mode)
{
    switch (mode) {
    case 2U: return &shortestRunModeParams2;
    case 3U: return &shortestRunModeParams3;
    case 4U: return &shortestRunModeParams4;
    case 5U: return &shortestRunModeParams5;
    case 6U: return &shortestRunModeParams6;
    case 7U: return &shortestRunModeParams7;
    default: return NULL;
    }
}

static const ShortestRunCaseParams_t *nf_host_case_params(uint8_t mode,
                                                           uint8_t case_index)
{
    const size_t index = (size_t)(case_index - 1U);
    switch (mode) {
    case 2U: return &shortestRunCaseParamsMode2[index];
    case 3U: return &shortestRunCaseParamsMode3[index];
    case 4U: return &shortestRunCaseParamsMode4[index];
    case 5U: return &shortestRunCaseParamsMode5[index];
    case 6U: return &shortestRunCaseParamsMode6[index];
    case 7U: return &shortestRunCaseParamsMode7[index];
    default: return NULL;
    }
}

static size_t nf_host_add_compiled_goals(NfRouteMaze *maze)
{
    static const uint8_t goals[9][2] = {
        {GOAL1_X, GOAL1_Y}, {GOAL2_X, GOAL2_Y}, {GOAL3_X, GOAL3_Y},
        {GOAL4_X, GOAL4_Y}, {GOAL5_X, GOAL5_Y}, {GOAL6_X, GOAL6_Y},
        {GOAL7_X, GOAL7_Y}, {GOAL8_X, GOAL8_Y}, {GOAL9_X, GOAL9_Y},
    };
    size_t count = 0U;

    for (size_t i = 0U; i < 9U; i++) {
        const uint8_t x = goals[i][0];
        const uint8_t y = goals[i][1];
        if ((x == 0U && y == 0U) || x >= maze->width || y >= maze->height ||
            maze->goals[y][x]) {
            continue;
        }
        maze->goals[y][x] = true;
        count++;
    }
    return count;
}

static bool nf_host_build_internal_maze(NfRouteMaze *maze,
                                        NfMazeAsciiInfo *info)
{
    if (!nf_route_maze_init(maze, 16U, 16U) ||
        !nf_route_maze_add_boundaries(maze)) {
        return false;
    }
    info->start_x = START_X;
    info->start_y = START_Y;
    info->goal_count = nf_host_add_compiled_goals(maze);
    if (info->start_x >= maze->width || info->start_y >= maze->height ||
        info->goal_count == 0U) {
        return false;
    }
    /* Preserve the standard micromouse start-cell east wall in the sample. */
    (void)nf_route_maze_set_wall(maze, info->start_x, info->start_y,
                                 NF_ROUTE_DIR_EAST);
    return true;
}

static bool nf_host_allow_large(const ShortestRunModeParams_t *mode_params,
                                uint8_t case_index,
                                NfHostTurnSet turn_set)
{
    if (turn_set == NF_HOST_TURN_SET_SMALL) {
        return false;
    }
    if (turn_set == NF_HOST_TURN_SET_ALL) {
        return true;
    }
    return (case_index == 1U) ?
        (mode_params->makepath_type_case3 != 0) :
        (mode_params->makepath_type_case47 != 0);
}

static NfOrthogonalPlannerConfig nf_host_config(
    const ShortestRunModeParams_t *mode_params,
    const ShortestRunCaseParams_t *case_params,
    bool allow_large)
{
    NfOrthogonalPlannerConfig config;
    memset(&config, 0, sizeof(config));
    config.half_cell_mm = (double)DIST_HALF_SEC;
    config.start_offset_mm = (double)DIST_FIRST_SEC;
    config.straight.vmax_mm_s = case_params->velocity_straight;
    config.straight.switch_velocity_mm_s = mode_params->accel_switch_velocity;
    config.straight.accel_low_mm_s2 = case_params->acceleration_straight;
    config.straight.accel_high_mm_s2 = case_params->acceleration_straight_dash;
    config.turn_environment.omega_cap_deg_s = NF_HOST_F413_OMEGA_CAP_DEG_S;
    config.turn_environment.rounding_scale = TURN_OMEGA_PROFILE_ROUNDING_SCALE;

    config.small_90.enabled = true;
    config.small_90.velocity_mm_s = mode_params->velocity_turn90;
    config.small_90.alpha_deg_s2 = mode_params->alpha_turn90;
    config.small_90.angle_deg = 90.0;
    config.small_90.dist_in_mm = mode_params->dist_offset_in;
    config.small_90.dist_out_mm = mode_params->dist_offset_out;

    config.large_90.enabled = true;
    config.large_90.velocity_mm_s = mode_params->velocity_l_turn_90;
    config.large_90.alpha_deg_s2 = mode_params->alpha_l_turn_90;
    config.large_90.angle_deg = 90.0;
    config.large_90.dist_in_mm = mode_params->dist_l_turn_in_90;
    config.large_90.dist_out_mm = mode_params->dist_l_turn_out_90;

    config.large_180.enabled = true;
    config.large_180.velocity_mm_s = mode_params->velocity_l_turn_180;
    config.large_180.alpha_deg_s2 = mode_params->alpha_l_turn_180;
    config.large_180.angle_deg = 180.0;
    config.large_180.dist_in_mm = mode_params->dist_l_turn_in_180;
    config.large_180.dist_out_mm = mode_params->dist_l_turn_out_180;
    config.allow_large_turns = allow_large;
    return config;
}

const char *nf_host_turn_set_name(NfHostTurnSet turn_set)
{
    switch (turn_set) {
    case NF_HOST_TURN_SET_PROFILE: return "profile";
    case NF_HOST_TURN_SET_SMALL: return "small";
    case NF_HOST_TURN_SET_ALL: return "all";
    default: return "unknown";
    }
}

static void nf_host_print_turn_geometry(const char *name,
                                        const NfTurnSpec *turn,
                                        const NfTurnEnvironment *environment,
                                        double expected_forward_mm,
                                        double expected_lateral_mm)
{
    NfTurnPlan turn_plan;
    if (nf_motion_turn_plan(turn, environment, &turn_plan) != NF_MOTION_OK) {
        return;
    }
    printf("[time-plan] geometry=%s displacement_mm=(forward=%.3f,lateral=%.3f) "
           "anchor_mm=(forward=%.3f,lateral=%.3f) residual_mm=(%.3f,%.3f)\n",
           name, turn_plan.displacement_forward_mm,
           turn_plan.displacement_lateral_mm, expected_forward_mm,
           expected_lateral_mm,
           turn_plan.displacement_forward_mm - expected_forward_mm,
           turn_plan.displacement_lateral_mm - expected_lateral_mm);
}

static void nf_host_print_geometry_summary(
    const NfOrthogonalPlannerConfig *config)
{
    nf_host_print_turn_geometry("small90", &config->small_90,
                                &config->turn_environment,
                                config->half_cell_mm, config->half_cell_mm);
    nf_host_print_turn_geometry("large90", &config->large_90,
                                &config->turn_environment,
                                2.0 * config->half_cell_mm,
                                2.0 * config->half_cell_mm);
    nf_host_print_turn_geometry("large180", &config->large_180,
                                &config->turn_environment,
                                0.0, 2.0 * config->half_cell_mm);
}

static void nf_host_print_plan(const NfRouteMaze *maze,
                               const NfMazeAsciiInfo *info,
                               uint8_t mode,
                               uint8_t case_index,
                               NfHostTurnSet turn_set,
                               bool allow_large,
                               const NfOrthogonalRoutePlan *plan)
{
    printf("[time-plan] result=ok maze=%ux%u start=(%u,%u,N) goals=%zu "
           "mode=%u case=%u turn_set=%s large=%s\n",
           (unsigned int)maze->width, (unsigned int)maze->height,
           (unsigned int)info->start_x, (unsigned int)info->start_y,
           info->goal_count, (unsigned int)mode, (unsigned int)case_index,
           nf_host_turn_set_name(turn_set), allow_large ? "yes" : "no");
    printf("[time-plan] selected_goal=(%u,%u,%s) goal_entry_us=%" PRIu64
           " stop_us=%" PRIu64 " goal_entry_velocity_mm_s=%.3f "
           "post_goal_extension_cells=%u expanded_states=%" PRIu32
           " relaxed_edges=%" PRIu32 "\n",
           (unsigned int)plan->goal_x, (unsigned int)plan->goal_y,
           nf_route_direction_name(plan->goal_heading),
           plan->goal_entry_us, plan->stop_us,
           plan->goal_entry_velocity_mm_s,
           (unsigned int)plan->post_goal_extension_cells,
           plan->expanded_states, plan->relaxed_edges);

    for (size_t i = 0U; i < plan->action_count; i++) {
        const NfRouteMotion *action = &plan->actions[i];
        printf("[time-plan] action=%zu kind=%s pose=(%u,%u,%s)->(%u,%u,%s) "
               "cells=%u distance_mm=%.3f velocity_mm_s=%.3f->%.3f "
               "duration_us=%" PRIu64,
               i, nf_route_motion_name(action->kind, action->side),
               (unsigned int)action->start_x, (unsigned int)action->start_y,
               nf_route_direction_name(action->start_heading),
               (unsigned int)action->end_x, (unsigned int)action->end_y,
               nf_route_direction_name(action->end_heading),
               (unsigned int)action->logical_cells, action->distance_mm,
               action->entry_velocity_mm_s, action->exit_velocity_mm_s,
               action->duration_us);
        if (action->has_goal_cross) {
            printf(" goal_cross=(%u,%u) after_cells=%u at_action_us=%" PRIu64,
                   (unsigned int)action->goal_x, (unsigned int)action->goal_y,
                   (unsigned int)action->goal_cross_after_cells,
                   action->goal_cross_time_us);
        }
        printf("\n");
    }
    printf("[time-plan] action_count=%zu diagonal_actions=0\n",
           plan->action_count);
}

int nf_host_run_time_plan(const char *maze_path,
                          uint8_t mode,
                          uint8_t case_index,
                          NfHostTurnSet turn_set,
                          bool assert_valid)
{
    const ShortestRunModeParams_t *mode_params;
    const ShortestRunCaseParams_t *case_params;
    NfRouteMaze maze;
    NfMazeAsciiInfo info;
    NfOrthogonalPlannerConfig config;
    NfOrthogonalPlannerRequest request;
    NfOrthogonalRoutePlan plan;
    NfOrthogonalRoutePlan repeated_plan;
    NfRouteValidation validation;
    NfRoutePlanStatus status;
    bool allow_large;

    if (mode < 2U || mode > 7U || case_index < 1U || case_index > 9U ||
        turn_set > NF_HOST_TURN_SET_ALL) {
        fprintf(stderr, "[time-plan] invalid mode/case/turn-set\n");
        return 2;
    }
    mode_params = nf_host_mode_params(mode);
    case_params = nf_host_case_params(mode, case_index);

    memset(&info, 0, sizeof(info));
    if (maze_path != NULL) {
        char error[192];
        const NfMazeAsciiStatus parse_status =
            nf_maze_ascii_load(maze_path, &maze, &info, error, sizeof(error));
        if (parse_status != NF_MAZE_ASCII_OK) {
            fprintf(stderr, "[time-plan] result=failed parser=%s reason=%s file=%s\n",
                    nf_maze_ascii_status_name(parse_status), error, maze_path);
            return 1;
        }
        printf("[time-plan] loaded maze=%s format=kerilab-strict\n", maze_path);
    } else if (!nf_host_build_internal_maze(&maze, &info)) {
        fprintf(stderr, "[time-plan] result=failed reason=internal-maze\n");
        return 1;
    } else {
        printf("[time-plan] loaded internal open 16x16 sample\n");
    }

    allow_large = nf_host_allow_large(mode_params, case_index, turn_set);
    config = nf_host_config(mode_params, case_params, allow_large);
    request.start_x = info.start_x;
    request.start_y = info.start_y;
    request.start_heading = NF_ROUTE_DIR_NORTH;

    status = nf_orthogonal_time_plan(&maze, &config, &request, &plan);
    if (status != NF_ROUTE_PLAN_OK) {
        fprintf(stderr, "[time-plan] result=failed planner=%s\n",
                nf_route_plan_status_name(status));
        return 1;
    }
    if (!nf_orthogonal_route_validate(&maze, &config, &request, &plan,
                                      &validation)) {
        fprintf(stderr, "[time-plan] result=failed validation_action=%zu reason=%s\n",
                validation.action_index, validation.message);
        return 1;
    }
    if (assert_valid) {
        status = nf_orthogonal_time_plan(&maze, &config, &request, &repeated_plan);
        if (status != NF_ROUTE_PLAN_OK ||
            memcmp(&plan, &repeated_plan, sizeof(plan)) != 0) {
            fprintf(stderr, "[time-plan] result=failed reason=nondeterministic\n");
            return 1;
        }
    }

    nf_host_print_plan(&maze, &info, mode, case_index, turn_set,
                       allow_large, &plan);
    nf_host_print_geometry_summary(&config);
    printf("[time-plan] validation=ok deterministic=%s objective=first-goal-entry\n",
           assert_valid ? "checked" : "not-checked");
    return 0;
}
