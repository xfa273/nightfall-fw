#include "slalom_time_plan_host.h"

#include "maze_ascii.h"
#include "slalom_plan_legacy_codec.h"

#include "params.h"
#include "shortest_run_params.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define NF_HOST_SLALOM_ORTHOGONAL_MASK \
    (NF_SLALOM_ENABLE_SMALL_90 | NF_SLALOM_ENABLE_LARGE_90 | \
     NF_SLALOM_ENABLE_LARGE_180)

static void nf_host_slalom_error(char *error,
                                 size_t error_size,
                                 const char *message)
{
    if (error != NULL && error_size != 0U) {
        (void)snprintf(error, error_size, "%s", message);
    }
}

static const ShortestRunModeParams_t *nf_host_slalom_mode_params(uint8_t mode)
{
    switch (mode) {
    case 2U: return &shortestRunModeParams2;
    case 3U: return &shortestRunModeParams3;
    case 4U: return &shortestRunModeParams4;
    case 5U: return &shortestRunModeParams5;
    default: return NULL;
    }
}

static const ShortestRunCaseParams_t *nf_host_slalom_case_params(
    uint8_t mode,
    uint8_t case_index)
{
    const size_t index = (size_t)(case_index - 1U);

    if (case_index < 1U || case_index > 9U) {
        return NULL;
    }
    switch (mode) {
    case 2U: return &shortestRunCaseParamsMode2[index];
    case 3U: return &shortestRunCaseParamsMode3[index];
    case 4U: return &shortestRunCaseParamsMode4[index];
    case 5U: return &shortestRunCaseParamsMode5[index];
    default: return NULL;
    }
}

static NfTurnSpec *nf_host_slalom_turn(NfSlalomPlannerConfig *config,
                                        NfPrimitiveId primitive)
{
    switch (primitive) {
    case NF_PRIMITIVE_SMALL_90: return &config->small_90;
    case NF_PRIMITIVE_LARGE_90: return &config->large_90;
    case NF_PRIMITIVE_LARGE_180: return &config->large_180;
    case NF_PRIMITIVE_45_IN: return &config->turn_45_in;
    case NF_PRIMITIVE_45_OUT: return &config->turn_45_out;
    case NF_PRIMITIVE_V90: return &config->v_90;
    case NF_PRIMITIVE_135_IN: return &config->turn_135_in;
    case NF_PRIMITIVE_135_OUT: return &config->turn_135_out;
    default: return NULL;
    }
}

bool nf_host_slalom_make_config(
    const char *profile_name,
    uint8_t case_index,
    uint32_t enabled_actions,
    NfSlalomPlannerConfig *out_config,
    const NfAuditProfile **out_profile,
    char *error,
    size_t error_size)
{
    const NfAuditProfile *profile = nf_slalom_profile_find(profile_name);
    const ShortestRunModeParams_t *mode_params;
    const ShortestRunCaseParams_t *case_params;

    if (out_config == NULL || out_profile == NULL) {
        nf_host_slalom_error(error, error_size, "invalid output argument");
        return false;
    }
    *out_profile = NULL;
    memset(out_config, 0, sizeof(*out_config));
    if (profile == NULL) {
        nf_host_slalom_error(error, error_size, "unknown slalom profile");
        return false;
    }
    if (fabs((double)DIST_HALF_SEC - NF_SLALOM_HALF_CELL_MM) > 1.0e-9 ||
        fabs((double)DIST_D_HALF_SEC - NF_SLALOM_COMMAND_DIAGONAL_MM) >
            1.0e-6) {
        nf_host_slalom_error(error, error_size,
                             "compiled distance constants drifted");
        return false;
    }
    mode_params = nf_host_slalom_mode_params(profile->shortest_run_mode);
    case_params = nf_host_slalom_case_params(profile->shortest_run_mode,
                                              case_index);
    if (mode_params == NULL || case_params == NULL) {
        nf_host_slalom_error(error, error_size, "profile/case is unavailable");
        return false;
    }
    if (case_params->velocity_straight <= 0.0f ||
        case_params->acceleration_straight <= 0.0f ||
        case_params->acceleration_straight_dash <= 0.0f ||
        case_params->velocity_d_straight <= 0.0f ||
        case_params->acceleration_d_straight_dash <= 0.0f) {
        nf_host_slalom_error(
            error, error_size,
            "case has no executable orthogonal/diagonal straight limits");
        return false;
    }
    if ((enabled_actions & ~NF_SLALOM_ENABLE_ALL) != 0U ||
        enabled_actions == 0U) {
        nf_host_slalom_error(error, error_size, "invalid action mask");
        return false;
    }

    out_config->half_cell_mm = NF_SLALOM_HALF_CELL_MM;
    out_config->diagonal_half_command_mm = NF_SLALOM_COMMAND_DIAGONAL_MM;
    out_config->start_offset_mm = profile->start_offset_mm;
    out_config->start_velocity_mm_s = 0.0;
    out_config->orthogonal.vmax_mm_s = case_params->velocity_straight;
    out_config->orthogonal.switch_velocity_mm_s =
        mode_params->accel_switch_velocity;
    out_config->orthogonal.accel_low_mm_s2 =
        case_params->acceleration_straight;
    out_config->orthogonal.accel_high_mm_s2 =
        case_params->acceleration_straight_dash;

    /*
     * Both current firmware runners model a diagonal code with the dash
     * acceleration only.  Keep that behavior here instead of inventing a
     * two-stage diagonal executor which does not yet exist on either MCU.
     */
    out_config->diagonal.vmax_mm_s = case_params->velocity_d_straight;
    out_config->diagonal.switch_velocity_mm_s = 0.0;
    out_config->diagonal.accel_low_mm_s2 =
        case_params->acceleration_d_straight;
    out_config->diagonal.accel_high_mm_s2 =
        case_params->acceleration_d_straight_dash;
    out_config->turn_environment = profile->environment;
    out_config->enabled_actions = enabled_actions;
    out_config->orthogonal_anchor_closure_tolerance_mm = 0.001;
    out_config->diagonal_anchor_closure_tolerance_mm = 0.001;
    out_config->heading_closure_tolerance_deg = 1.0e-6;
    out_config->max_turn_sample_step_mm = 1.0;
    out_config->minimum_post_goal_connector_steps = 1U;

    /* Board outline only: this is an optional PC diagnostic, not HIL safety. */
    out_config->clearance.cell_pitch_mm = 90.0;
    out_config->clearance.wall_thickness_mm = 6.0;
    out_config->clearance.robot_half_length_mm = 35.0;
    out_config->clearance.robot_half_width_mm = 19.5;
    out_config->clearance.max_translation_step_mm = 0.25;
    out_config->clearance.max_heading_step_deg = 0.25;

    for (size_t i = 0U; i < NF_PRIMITIVE_COUNT; i++) {
        const NfPrimitiveGeometry *geometry =
            &nf_slalom_primitive_geometry[i];
        const NfCurrentPrimitive *current = &profile->current[i];
        const NfProvisionalSeed *seed = &profile->seeds[i];
        NfTurnSpec *turn = nf_host_slalom_turn(
            out_config, (NfPrimitiveId)i);
        NfTurnSpec *geometry_turn = &out_config->geometry_turns[i];

        if (turn == NULL || !seed->available) {
            nf_host_slalom_error(error, error_size,
                                 "primitive geometry seed is unavailable");
            return false;
        }
        turn->enabled = true;
        turn->angle_deg = geometry->angle_deg;
        geometry_turn->enabled = true;
        geometry_turn->angle_deg = geometry->angle_deg;
        geometry_turn->velocity_mm_s = seed->velocity_mm_s;
        geometry_turn->alpha_deg_s2 = seed->alpha_deg_s2;
        geometry_turn->dist_in_mm = seed->dist_in_mm;
        geometry_turn->dist_out_mm = seed->dist_out_mm;
        if (i <= (size_t)NF_PRIMITIVE_LARGE_180) {
            if (!current->available) {
                nf_host_slalom_error(error, error_size,
                                     "orthogonal primitive is unavailable");
                return false;
            }
            turn->velocity_mm_s = current->velocity_mm_s;
            turn->alpha_deg_s2 = current->alpha_deg_s2;
            turn->dist_in_mm = current->dist_in_mm;
            turn->dist_out_mm = current->dist_out_mm;
        } else {
            turn->velocity_mm_s = seed->velocity_mm_s;
            turn->alpha_deg_s2 = seed->alpha_deg_s2;
            turn->dist_in_mm = seed->dist_in_mm;
            turn->dist_out_mm = seed->dist_out_mm;
        }
    }

    /*
     * #0 is not emitted by the diagonal planner, but its calibrated boundary
     * speed is a conservative common low-speed realization of #1--#5.
     */
    out_config->low_speed_turn_velocity_mm_s =
        out_config->small_90.velocity_mm_s;

    *out_profile = profile;
    nf_host_slalom_error(error, error_size, "ok");
    return true;
}

static bool nf_host_slalom_action_is_diagonal(
    const NfSlalomAction *action)
{
    return action->connector_is_diagonal ||
           action->kind == NF_SLALOM_ACTION_45_IN ||
           action->kind == NF_SLALOM_ACTION_45_OUT ||
           action->kind == NF_SLALOM_ACTION_V90 ||
           action->kind == NF_SLALOM_ACTION_135_IN ||
           action->kind == NF_SLALOM_ACTION_135_OUT;
}

static size_t nf_host_slalom_diagonal_action_count(
    const NfSlalomRoutePlan *plan)
{
    size_t count = 0U;
    for (size_t i = 0U; i < plan->action_count; i++) {
        if (nf_host_slalom_action_is_diagonal(&plan->actions[i])) {
            count++;
        }
    }
    return count;
}

static size_t nf_host_slalom_zero_step_diagonal_turn_count(
    const NfSlalomRoutePlan *plan)
{
    size_t count = 0U;
    for (size_t i = 0U; i < plan->action_count; i++) {
        const NfSlalomAction *action = &plan->actions[i];
        if ((unsigned int)action->kind <
                (unsigned int)NF_SLALOM_ACTION_START_OFFSET &&
            (((unsigned int)action->start_heading & 1U) != 0U) &&
            action->connector_steps == 0U) {
            count++;
        }
    }
    return count;
}


static size_t nf_host_slalom_reduced_turn_count(
    const NfSlalomRoutePlan *plan)
{
    size_t count = 0U;
    for (size_t i = 0U; i < plan->action_count; i++) {
        if ((unsigned int)plan->actions[i].kind <
                (unsigned int)NF_SLALOM_ACTION_START_OFFSET &&
            plan->actions[i].turn_speed_mode !=
                NF_SLALOM_TURN_SPEED_NOMINAL) {
            count++;
        }
    }
    return count;
}

static size_t nf_host_slalom_turn_speed_mode_count(
    const NfSlalomRoutePlan *plan,
    NfSlalomTurnSpeedMode mode)
{
    size_t count = 0U;
    for (size_t i = 0U; i < plan->action_count; i++) {
        if ((unsigned int)plan->actions[i].kind <
                (unsigned int)NF_SLALOM_ACTION_START_OFFSET &&
            plan->actions[i].turn_speed_mode == mode) {
            count++;
        }
    }
    return count;
}

static size_t nf_host_slalom_action_kind_count(
    const NfSlalomRoutePlan *plan,
    NfSlalomActionKind kind)
{
    size_t count = 0U;
    for (size_t i = 0U; i < plan->action_count; i++) {
        if (plan->actions[i].kind == kind) {
            count++;
        }
    }
    return count;
}

static void nf_host_slalom_print_action(size_t index,
                                        const NfSlalomAction *action)
{
    printf("[slalom-plan] action=%zu kind=%s anchor=(%d,%d)->(%d,%d)->(%d,%d) "
           "heading=%s->%s connector=%s:%u geometry_mm=%.3f command_mm=%.3f "
           "velocity_mm_s=%.3f->%.3f->%.3f duration_us=%" PRIu64,
           index, nf_slalom_action_name(action->kind, action->side),
           (int)action->start_anchor.half_x,
           (int)action->start_anchor.half_y,
           (int)action->connector_end_anchor.half_x,
           (int)action->connector_end_anchor.half_y,
           (int)action->end_anchor.half_x,
           (int)action->end_anchor.half_y,
           nf_slalom_heading_name(action->start_heading),
           nf_slalom_heading_name(action->end_heading),
           action->connector_is_diagonal ? "diagonal" : "orthogonal",
           (unsigned int)action->connector_steps,
           action->connector_geometry_distance_mm,
           action->connector_command_distance_mm,
           action->entry_velocity_mm_s, action->turn_velocity_mm_s,
           action->exit_velocity_mm_s, action->duration_us);
    if (action->turn_speed_mode == NF_SLALOM_TURN_SPEED_LOW) {
        printf(" turn_speed=low");
    } else if (action->turn_speed_mode == NF_SLALOM_TURN_SPEED_CRAWL) {
        printf(" turn_speed=crawl");
    }
    if (action->has_goal_cross) {
        printf(" goal_cross=(%u,%u,%s) phase=%s at_action_us=%" PRIu64,
               (unsigned int)action->goal_x,
               (unsigned int)action->goal_y,
               nf_slalom_heading_name(action->goal_cross_heading),
               action->goal_phase == NF_SLALOM_GOAL_TURN ? "turn" :
                   "connector",
               action->goal_cross_time_us);
    }
    printf("\n");
}

int nf_host_run_slalom_time_plan(const char *maze_path,
                                 const char *profile_name,
                                 uint8_t case_index,
                                 bool compare_orthogonal,
                                 bool assert_valid,
                                 bool summary_only,
                                 bool check_turn_clearance)
{
    NfRouteMaze maze;
    NfMazeAsciiInfo info;
    NfSlalomPlannerConfig config;
    NfSlalomPlannerRequest request;
    NfSlalomRoutePlan plan;
    NfSlalomRoutePlan repeated;
    NfSlalomRoutePlan orthogonal;
    NfSlalomValidation validation;
    NfSlalomLegacyResult legacy_result;
    const NfSlalomLegacyContract legacy_contract = {255U, 1U};
    uint16_t legacy_path[256];
    const NfAuditProfile *profile;
    NfSlalomPlanStatus status;
    char error[192];
    uint64_t orthogonal_goal_entry_us = 0U;
    bool orthogonal_available = false;
    const uint32_t full_mask = NF_SLALOM_ENABLE_SHORTEST_1_TO_5;

    if (maze_path == NULL) {
        fprintf(stderr,
                "[slalom-plan] result=failed reason=maze file is required\n");
        return 2;
    }
    if (case_index < 8U || case_index > 9U) {
        fprintf(stderr,
                "[slalom-plan] result=failed reason=case must be 8 or 9\n");
        return 2;
    }
    if (!nf_host_slalom_make_config(profile_name, case_index, full_mask,
                                     &config, &profile, error,
                                     sizeof(error))) {
        fprintf(stderr, "[slalom-plan] result=failed reason=%s\n", error);
        return 2;
    }
    config.check_swept_clearance = check_turn_clearance;
    memset(&info, 0, sizeof(info));
    {
        const NfMazeAsciiStatus parse_status = nf_maze_ascii_load(
            maze_path, &maze, &info, error, sizeof(error));
        if (parse_status != NF_MAZE_ASCII_OK) {
            fprintf(stderr,
                    "[slalom-plan] result=failed parser=%s reason=%s file=%s\n",
                    nf_maze_ascii_status_name(parse_status), error, maze_path);
            return 1;
        }
    }
    request.start_x = info.start_x;
    request.start_y = info.start_y;
    request.start_heading = NF_SLALOM_HEADING_NORTH;

    status = nf_slalom_time_plan(&maze, &config, &request, &plan);
    if (status != NF_SLALOM_PLAN_OK) {
        fprintf(stderr, "[slalom-plan] result=failed planner=%s\n",
                nf_slalom_plan_status_name(status));
        return 1;
    }
    if (!nf_slalom_route_validate(&maze, &config, &request, &plan,
                                  &validation)) {
        fprintf(stderr,
                "[slalom-plan] result=failed validation_action=%zu reason=%s\n",
                validation.action_index, validation.message);
        return 1;
    }
    if (assert_valid) {
        status = nf_slalom_time_plan(&maze, &config, &request, &repeated);
        if (status != NF_SLALOM_PLAN_OK ||
            memcmp(&plan, &repeated, sizeof(plan)) != 0) {
            fprintf(stderr,
                    "[slalom-plan] result=failed reason=nondeterministic\n");
            return 1;
        }
    }

    legacy_result = nf_slalom_plan_to_legacy(
        &maze, &config, &request, &plan, &legacy_contract, legacy_path,
        sizeof(legacy_path) / sizeof(legacy_path[0]));
    if (legacy_result.status != NF_SLALOM_LEGACY_OK &&
        legacy_result.status !=
            NF_SLALOM_LEGACY_TERMINAL_DIAGONAL_UNSUPPORTED &&
        legacy_result.status != NF_SLALOM_LEGACY_NO_RUN_REQUIRED) {
        fprintf(stderr,
                "[slalom-plan] result=failed legacy_action=%zu legacy=%s\n",
                legacy_result.action_index,
                nf_slalom_legacy_status_name(legacy_result.status));
        return 1;
    }

    if (compare_orthogonal) {
        NfSlalomPlannerConfig orthogonal_config;
        const NfAuditProfile *ignored_profile;
        if (!nf_host_slalom_make_config(
                profile_name, case_index, NF_HOST_SLALOM_ORTHOGONAL_MASK,
                &orthogonal_config, &ignored_profile, error, sizeof(error))) {
            fprintf(stderr,
                    "[slalom-plan] result=failed orthogonal_config=%s\n",
                    error);
            return 1;
        }
        status = nf_slalom_time_plan(&maze, &orthogonal_config, &request,
                                     &orthogonal);
        if (status != NF_SLALOM_PLAN_OK ||
            !nf_slalom_route_validate(&maze, &orthogonal_config, &request,
                                      &orthogonal, &validation)) {
            fprintf(stderr,
                    "[slalom-plan] result=failed orthogonal_planner=%s\n",
                    nf_slalom_plan_status_name(status));
            return 1;
        }
        orthogonal_available = true;
        orthogonal_goal_entry_us = orthogonal.goal_entry_us;
        if (plan.goal_entry_us > orthogonal_goal_entry_us) {
            fprintf(stderr,
                    "[slalom-plan] result=failed "
                    "reason=patterns-1-to-5-slower-than-orthogonal-baseline "
                    "diagonal_us=%" PRIu64 " orthogonal_us=%" PRIu64 "\n",
                    plan.goal_entry_us, orthogonal_goal_entry_us);
            return 1;
        }
    }

    if (!summary_only) {
        printf("[slalom-plan] loaded maze=%s size=%ux%u goals=%zu\n",
               maze_path, (unsigned int)maze.width, (unsigned int)maze.height,
               info.goal_count);
        printf("[slalom-plan] profile=%s role=%s mode=%u case=%u "
               "start_offset_mm=%.3f diagonal_turns=PC_PROVISIONAL "
               "patterns=KERI-1-to-5 low_speed_turn_mm_s=%.3f "
               "objective=first-goal-entry\n",
               profile->name, profile->primary ? "primary" : "comparison",
               (unsigned int)profile->shortest_run_mode,
               (unsigned int)case_index, profile->start_offset_mm,
               config.low_speed_turn_velocity_mm_s);
        printf("[slalom-plan] selected_goal=(%u,%u,%s) goal_entry_us=%" PRIu64
               " stop_us=%" PRIu64 " goal_entry_velocity_mm_s=%.3f "
               "expanded_states=%" PRIu32 " relaxed_edges=%" PRIu32 "\n",
               (unsigned int)plan.goal_x, (unsigned int)plan.goal_y,
               nf_slalom_heading_name(plan.goal_heading), plan.goal_entry_us,
               plan.stop_us, plan.goal_entry_velocity_mm_s,
               plan.expanded_states, plan.relaxed_edges);
        for (size_t i = 0U; i < plan.action_count; i++) {
            nf_host_slalom_print_action(i, &plan.actions[i]);
        }
        printf("[slalom-plan] legacy_geometry=%s legacy_time_equivalent=no",
               nf_slalom_legacy_status_name(legacy_result.status));
        if (legacy_result.status == NF_SLALOM_LEGACY_OK) {
            printf(" legacy_codes=");
            for (size_t i = 0U; i < legacy_result.length; i++) {
                printf("%s%u", (i == 0U) ? "" : ",",
                       (unsigned int)legacy_path[i]);
            }
        }
        printf("\n");
        printf("[slalom-plan] validation=ok deterministic=%s "
               "anchor_closure=ok required_open=ok turn_clearance=%s\n",
               assert_valid ? "checked" : "not-checked",
               check_turn_clearance ? "board-envelope-only" : "not-checked");
    }

    printf("[slalom-summary] profile=%s mode=%u case=%u maze=%s "
           "goal_entry_us=%" PRIu64 " stop_us=%" PRIu64
           " orthogonal_goal_entry_us=%" PRIu64
           " improvement_us=%" PRIu64 " actions=%zu diagonal_actions=%zu "
           "zero_step_diagonal_turns=%zu reduced_turns=%zu "
           "low_turns=%zu crawl_turns=%zu small90_actions=%zu "
           "legacy_geometry=%s legacy_time_equivalent=no "
           "validation=ok deterministic=%s objective=first-goal-entry\n",
           profile->name, (unsigned int)profile->shortest_run_mode,
           (unsigned int)case_index, maze_path, plan.goal_entry_us,
           plan.stop_us, orthogonal_goal_entry_us,
           orthogonal_available ? orthogonal_goal_entry_us - plan.goal_entry_us
                                : 0U,
           plan.action_count, nf_host_slalom_diagonal_action_count(&plan),
           nf_host_slalom_zero_step_diagonal_turn_count(&plan),
           nf_host_slalom_reduced_turn_count(&plan),
           nf_host_slalom_turn_speed_mode_count(
               &plan, NF_SLALOM_TURN_SPEED_LOW),
           nf_host_slalom_turn_speed_mode_count(
               &plan, NF_SLALOM_TURN_SPEED_CRAWL),
           nf_host_slalom_action_kind_count(
               &plan, NF_SLALOM_ACTION_SMALL_90),
           nf_slalom_legacy_status_name(legacy_result.status),
           assert_valid ? "checked" : "not-checked");
    return 0;
}
