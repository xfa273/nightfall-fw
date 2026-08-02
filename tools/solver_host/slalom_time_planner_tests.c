#include "slalom_profile_baseline.h"
#include "slalom_time_planner.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static unsigned int g_checks;
static unsigned int g_failures;

#define CHECK_TRUE(condition)                                                   \
    do {                                                                        \
        g_checks++;                                                             \
        if (!(condition)) {                                                     \
            fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, \
                    #condition);                                                \
            g_failures++;                                                       \
        }                                                                       \
    } while (0)

#define REQUIRE_TRUE(condition)                                                 \
    do {                                                                        \
        g_checks++;                                                             \
        if (!(condition)) {                                                     \
            fprintf(stderr, "%s:%d: requirement failed: %s\n",               \
                    __FILE__, __LINE__, #condition);                            \
            g_failures++;                                                       \
            return;                                                             \
        }                                                                       \
    } while (0)

static NfTurnSpec *config_turn(NfSlalomPlannerConfig *config,
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

static bool make_config(NfSlalomPlannerConfig *config)
{
    const NfAuditProfile *profile =
        nf_slalom_profile_find("f413-preorder-mode2");

    if (config == NULL || profile == NULL) {
        return false;
    }
    memset(config, 0, sizeof(*config));
    config->half_cell_mm = NF_SLALOM_HALF_CELL_MM;
    config->diagonal_half_command_mm = NF_SLALOM_COMMAND_DIAGONAL_MM;
    config->start_offset_mm = 5.0;
    config->start_velocity_mm_s = 0.0;
    config->orthogonal = (NfLinearLimits){
        1800.0, 700.0, 5000.0, 8000.0,
    };
    config->diagonal = (NfLinearLimits){
        1600.0, 700.0, 4500.0, 7000.0,
    };
    config->turn_environment = profile->environment;
    config->enabled_actions = NF_SLALOM_ENABLE_ALL;
    config->orthogonal_anchor_closure_tolerance_mm = 0.001;
    config->diagonal_anchor_closure_tolerance_mm = 0.001;
    config->heading_closure_tolerance_deg = 1.0e-6;
    config->max_turn_sample_step_mm = 1.0;
    config->minimum_post_goal_connector_steps = 1U;

    for (size_t i = 0U; i < NF_PRIMITIVE_COUNT; i++) {
        const NfPrimitiveGeometry *geometry =
            &nf_slalom_primitive_geometry[i];
        const NfCurrentPrimitive *current = &profile->current[i];
        const NfProvisionalSeed *seed = &profile->seeds[i];
        NfTurnSpec *turn = config_turn(config, (NfPrimitiveId)i);
        NfTurnSpec *geometry_turn = &config->geometry_turns[i];

        if (turn == NULL || !current->available || !seed->available) {
            return false;
        }
        memset(turn, 0, sizeof(*turn));
        turn->enabled = true;
        turn->angle_deg = geometry->angle_deg;
        geometry_turn->enabled = true;
        geometry_turn->angle_deg = geometry->angle_deg;
        geometry_turn->velocity_mm_s = seed->velocity_mm_s;
        geometry_turn->alpha_deg_s2 = seed->alpha_deg_s2;
        geometry_turn->dist_in_mm = seed->dist_in_mm;
        geometry_turn->dist_out_mm = seed->dist_out_mm;
        if (i <= NF_PRIMITIVE_LARGE_180) {
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
    return true;
}

static bool make_open_maze(NfRouteMaze *maze, uint8_t width, uint8_t height)
{
    return nf_route_maze_init(maze, width, height) &&
           nf_route_maze_add_boundaries(maze);
}

static void close_all_internal_walls(NfRouteMaze *maze)
{
    for (uint8_t y = 0U; y < maze->height; y++) {
        for (uint8_t x = 0U; x + 1U < maze->width; x++) {
            (void)nf_route_maze_set_wall(maze, x, y, NF_ROUTE_DIR_EAST);
        }
    }
    for (uint8_t y = 0U; y + 1U < maze->height; y++) {
        for (uint8_t x = 0U; x < maze->width; x++) {
            (void)nf_route_maze_set_wall(maze, x, y, NF_ROUTE_DIR_NORTH);
        }
    }
}

static void open_between(NfRouteMaze *maze,
                         uint8_t x0,
                         uint8_t y0,
                         uint8_t x1,
                         uint8_t y1)
{
    if (x1 == (uint8_t)(x0 + 1U) && y0 == y1) {
        maze->walls[y0][x0] &= (uint8_t)~NF_ROUTE_WALL_EAST;
        maze->walls[y1][x1] &= (uint8_t)~NF_ROUTE_WALL_WEST;
    } else if (x0 == (uint8_t)(x1 + 1U) && y0 == y1) {
        open_between(maze, x1, y1, x0, y0);
    } else if (y1 == (uint8_t)(y0 + 1U) && x0 == x1) {
        maze->walls[y0][x0] &= (uint8_t)~NF_ROUTE_WALL_NORTH;
        maze->walls[y1][x1] &= (uint8_t)~NF_ROUTE_WALL_SOUTH;
    } else if (y0 == (uint8_t)(y1 + 1U) && x0 == x1) {
        open_between(maze, x1, y1, x0, y0);
    }
}

static bool is_diagonal_kind(NfSlalomActionKind kind)
{
    return kind == NF_SLALOM_ACTION_45_IN ||
           kind == NF_SLALOM_ACTION_45_OUT ||
           kind == NF_SLALOM_ACTION_V90 ||
           kind == NF_SLALOM_ACTION_135_IN ||
           kind == NF_SLALOM_ACTION_135_OUT;
}

static bool plan_has_diagonal(const NfSlalomRoutePlan *plan)
{
    for (size_t i = 0U; i < plan->action_count; i++) {
        if (is_diagonal_kind(plan->actions[i].kind) ||
            plan->actions[i].connector_is_diagonal) {
            return true;
        }
    }
    return false;
}

static void close_anchor_wall(NfRouteMaze *maze, NfSlalomAnchor anchor)
{
    if ((anchor.half_x & 1) == 0) {
        const uint8_t right_x = (uint8_t)(anchor.half_x / 2);
        const uint8_t y = (uint8_t)((anchor.half_y - 1) / 2);
        (void)nf_route_maze_set_wall(maze, (uint8_t)(right_x - 1U), y,
                                     NF_ROUTE_DIR_EAST);
    } else {
        const uint8_t x = (uint8_t)((anchor.half_x - 1) / 2);
        const uint8_t upper_y = (uint8_t)(anchor.half_y / 2);
        (void)nf_route_maze_set_wall(maze, x, (uint8_t)(upper_y - 1U),
                                     NF_ROUTE_DIR_NORTH);
    }
}

static const NfTurnSpec *action_turn(const NfSlalomPlannerConfig *config,
                                     NfSlalomActionKind kind)
{
    switch (kind) {
    case NF_SLALOM_ACTION_SMALL_90: return &config->small_90;
    case NF_SLALOM_ACTION_LARGE_90: return &config->large_90;
    case NF_SLALOM_ACTION_LARGE_180: return &config->large_180;
    case NF_SLALOM_ACTION_45_IN: return &config->turn_45_in;
    case NF_SLALOM_ACTION_45_OUT: return &config->turn_45_out;
    case NF_SLALOM_ACTION_V90: return &config->v_90;
    case NF_SLALOM_ACTION_135_IN: return &config->turn_135_in;
    case NF_SLALOM_ACTION_135_OUT: return &config->turn_135_out;
    default: return NULL;
    }
}

static void check_plan_contract(const NfRouteMaze *maze,
                                const NfSlalomPlannerConfig *config,
                                const NfSlalomPlannerRequest *request,
                                const NfSlalomRoutePlan *plan)
{
    NfSlalomValidation validation;
    REQUIRE_TRUE(plan->action_count >= 2U);
    CHECK_TRUE(plan->actions[0].kind == NF_SLALOM_ACTION_START_OFFSET);
    CHECK_TRUE(plan->actions[0].duration_us > 0U);
    CHECK_TRUE(plan->goal_entry_us <= plan->stop_us);
    CHECK_TRUE(plan->anchor_closure_validated);
    CHECK_TRUE(plan->required_open_validated);
    {
        size_t goal_actions = 0U;
        uint64_t elapsed_us = 0U;
        for (size_t i = 0U; i < plan->action_count; i++) {
            const NfSlalomAction *action = &plan->actions[i];
            if (action->has_goal_cross) {
                goal_actions++;
                CHECK_TRUE(action->goal_cross_time_us <= action->duration_us);
                CHECK_TRUE(elapsed_us + action->goal_cross_time_us ==
                           plan->goal_entry_us);
                if ((unsigned int)action->kind <
                    (unsigned int)NF_SLALOM_ACTION_START_OFFSET) {
                    CHECK_TRUE(action->goal_phase == NF_SLALOM_GOAL_TURN);
                } else {
                    CHECK_TRUE(action->kind == NF_SLALOM_ACTION_GOAL_STOP);
                    CHECK_TRUE(action->goal_phase ==
                               NF_SLALOM_GOAL_CONNECTOR);
                }
            }
            elapsed_us += action->duration_us;
        }
        CHECK_TRUE(goal_actions == 1U);
        CHECK_TRUE(elapsed_us == plan->stop_us);
    }
    CHECK_TRUE(nf_slalom_route_validate(maze, config, request, plan,
                                         &validation));
    CHECK_TRUE(validation.valid);
    CHECK_TRUE(validation.recomputed_goal_entry_us == plan->goal_entry_us);
    CHECK_TRUE(validation.recomputed_stop_us == plan->stop_us);
}

static void test_config_closure_gate(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    NfSlalomPlannerRequest request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan plan;

    REQUIRE_TRUE(make_open_maze(&maze, 1U, 4U));
    maze.goals[2][0] = true;
    REQUIRE_TRUE(make_config(&config));
    config.diagonal_anchor_closure_tolerance_mm = 1.0e-8;
    CHECK_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
               NF_SLALOM_PLAN_INVALID_CONFIG);

    REQUIRE_TRUE(make_config(&config));
    config.geometry_turns[NF_SLALOM_ACTION_45_IN].dist_out_mm += 2.0;
    CHECK_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
               NF_SLALOM_PLAN_INVALID_CONFIG);

    /* A timing-only residual is allowed; it must not move the centre line. */
    REQUIRE_TRUE(make_config(&config));
    config.turn_45_in.dist_out_mm += 2.0;
    CHECK_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
               NF_SLALOM_PLAN_OK);

    REQUIRE_TRUE(make_config(&config));
    config.orthogonal_anchor_closure_tolerance_mm = 16.0;
    CHECK_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
               NF_SLALOM_PLAN_INVALID_CONFIG);

    REQUIRE_TRUE(make_config(&config));
    config.heading_closure_tolerance_deg = 1.0;
    CHECK_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
               NF_SLALOM_PLAN_INVALID_CONFIG);
}

static void test_fine_turn_sampling_is_topology_stable(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    NfSlalomPrimitiveCheck check;

    REQUIRE_TRUE(make_open_maze(&maze, 2U, 2U));
    maze.goals[1][1] = true;
    REQUIRE_TRUE(make_config(&config));
    config.enabled_actions = NF_SLALOM_ENABLE_SMALL_90;
    config.max_turn_sample_step_mm = 0.02;
    REQUIRE_TRUE(nf_slalom_primitive_check(
                     &maze, &config, (NfSlalomAnchor){1, 2},
                     NF_SLALOM_HEADING_NORTH,
                     NF_SLALOM_ACTION_SMALL_90, NF_ROUTE_SIDE_RIGHT,
                     &check) == NF_SLALOM_PLAN_OK);
    CHECK_TRUE(check.feasible);
    CHECK_TRUE(check.destination_anchor.half_x == 2);
    CHECK_TRUE(check.destination_anchor.half_y == 3);
    CHECK_TRUE(check.destination_heading == NF_SLALOM_HEADING_EAST);
}

static void test_maze_bits_and_start_goal_contract(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    const NfSlalomPlannerRequest request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan plan;
    NfSlalomValidation validation;

    REQUIRE_TRUE(make_open_maze(&maze, 1U, 1U));
    maze.goals[0][0] = true;
    REQUIRE_TRUE(make_config(&config));
    maze.walls[0][0] |= 0x80U;
    CHECK_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
               NF_SLALOM_PLAN_INVALID_MAZE);

    maze.walls[0][0] &= 0x0fU;
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
                 NF_SLALOM_PLAN_OK);
    CHECK_TRUE(plan.action_count == 0U);
    CHECK_TRUE(plan.goal_entry_us == 0U);
    CHECK_TRUE(plan.stop_us == 0U);
    CHECK_TRUE(plan.goal_x == request.start_x);
    CHECK_TRUE(plan.goal_y == request.start_y);
    CHECK_TRUE(plan.goal_heading == request.start_heading);
    CHECK_TRUE(plan.anchor_closure_validated);
    CHECK_TRUE(plan.required_open_validated);
    CHECK_TRUE(nf_slalom_route_validate(&maze, &config, &request, &plan,
                                         &validation));
    CHECK_TRUE(validation.valid);

    {
        NfSlalomPlannerRequest invalid_request = request;
        NfSlalomRoutePlan invalid_plan = plan;
        invalid_request.start_heading = (NfSlalomHeading8)8;
        invalid_plan.goal_heading = invalid_request.start_heading;
        CHECK_TRUE(!nf_slalom_route_validate(
            &maze, &config, &invalid_request, &invalid_plan, &validation));
        CHECK_TRUE(!validation.valid);
    }
}

static void test_all_primitive_transitions_and_wall_rejection(void)
{
    static const NfSlalomAnchor expected_destination[8][2] = {
        {{8, 7}, {6, 7}},   /* small90: N -> E/W */
        {{9, 9}, {5, 9}},   /* large90: N -> E/W */
        {{9, 7}, {5, 7}},   /* large180: N -> S */
        {{8, 9}, {6, 9}},   /* 45in: N -> NE/NW */
        {{9, 7}, {7, 9}},   /* 45out: NE -> E/N */
        {{9, 6}, {6, 9}},   /* V90: NE -> SE/NW */
        {{9, 8}, {5, 8}},   /* 135in: N -> SE/SW */
        {{9, 5}, {5, 9}},   /* 135out: NE -> S/W */
    };
    static const NfSlalomHeading8 expected_heading[8][2] = {
        {NF_SLALOM_HEADING_EAST, NF_SLALOM_HEADING_WEST},
        {NF_SLALOM_HEADING_EAST, NF_SLALOM_HEADING_WEST},
        {NF_SLALOM_HEADING_SOUTH, NF_SLALOM_HEADING_SOUTH},
        {NF_SLALOM_HEADING_NORTH_EAST,
         NF_SLALOM_HEADING_NORTH_WEST},
        {NF_SLALOM_HEADING_EAST, NF_SLALOM_HEADING_NORTH},
        {NF_SLALOM_HEADING_SOUTH_EAST,
         NF_SLALOM_HEADING_NORTH_WEST},
        {NF_SLALOM_HEADING_SOUTH_EAST,
         NF_SLALOM_HEADING_SOUTH_WEST},
        {NF_SLALOM_HEADING_SOUTH, NF_SLALOM_HEADING_WEST},
    };
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;

    REQUIRE_TRUE(make_open_maze(&maze, 7U, 7U));
    maze.goals[6][6] = true;
    REQUIRE_TRUE(make_config(&config));

    for (unsigned int kind_value = 0U;
         kind_value < (unsigned int)NF_SLALOM_ACTION_START_OFFSET;
         kind_value++) {
        const NfSlalomActionKind kind = (NfSlalomActionKind)kind_value;
        for (unsigned int side_value = 0U; side_value < 2U; side_value++) {
            const NfRouteSide side = (side_value == 0U) ?
                NF_ROUTE_SIDE_RIGHT : NF_ROUTE_SIDE_LEFT;
            NfSlalomAnchor source;
            NfSlalomHeading8 heading;
            NfSlalomPrimitiveCheck check;

            if (kind == NF_SLALOM_ACTION_SMALL_90) {
                source = (NfSlalomAnchor){7, 6};
                heading = NF_SLALOM_HEADING_NORTH;
            } else if (kind == NF_SLALOM_ACTION_LARGE_90 ||
                       kind == NF_SLALOM_ACTION_LARGE_180 ||
                       kind == NF_SLALOM_ACTION_45_IN ||
                       kind == NF_SLALOM_ACTION_135_IN) {
                source = (NfSlalomAnchor){7, 7};
                heading = NF_SLALOM_HEADING_NORTH;
            } else {
                source = (side == NF_ROUTE_SIDE_RIGHT) ?
                    (NfSlalomAnchor){7, 6} :
                    (NfSlalomAnchor){6, 7};
                heading = NF_SLALOM_HEADING_NORTH_EAST;
            }
            REQUIRE_TRUE(nf_slalom_primitive_check(
                             &maze, &config, source, heading, kind, side,
                             &check) == NF_SLALOM_PLAN_OK);
            if (!check.feasible) {
                fprintf(stderr, "open primitive rejected kind=%u side=%u\n",
                        kind_value, side_value);
            }
            CHECK_TRUE(check.feasible);
            CHECK_TRUE(check.required_open_checked);
            CHECK_TRUE(check.turn_time_us > 0U);
            CHECK_TRUE(check.destination_anchor.half_x ==
                       expected_destination[kind_value][side_value].half_x);
            CHECK_TRUE(check.destination_anchor.half_y ==
                       expected_destination[kind_value][side_value].half_y);
            CHECK_TRUE(check.destination_heading ==
                       expected_heading[kind_value][side_value]);
        }

        {
            NfRouteMaze blocked = maze;
            NfSlalomPrimitiveCheck open_check;
            NfSlalomPrimitiveCheck blocked_check;
            NfSlalomAnchor source;
            NfSlalomHeading8 heading;

            if (kind == NF_SLALOM_ACTION_SMALL_90) {
                source = (NfSlalomAnchor){7, 6};
                heading = NF_SLALOM_HEADING_NORTH;
            } else if (kind == NF_SLALOM_ACTION_LARGE_90 ||
                       kind == NF_SLALOM_ACTION_LARGE_180 ||
                       kind == NF_SLALOM_ACTION_45_IN ||
                       kind == NF_SLALOM_ACTION_135_IN) {
                source = (NfSlalomAnchor){7, 7};
                heading = NF_SLALOM_HEADING_NORTH;
            } else {
                source = (NfSlalomAnchor){7, 6};
                heading = NF_SLALOM_HEADING_NORTH_EAST;
            }
            REQUIRE_TRUE(nf_slalom_primitive_check(
                             &maze, &config, source, heading, kind,
                             NF_ROUTE_SIDE_RIGHT,
                             &open_check) == NF_SLALOM_PLAN_OK);
            if (!open_check.feasible) {
                fprintf(stderr, "open wall fixture rejected kind=%u\n",
                        kind_value);
            }
            REQUIRE_TRUE(open_check.feasible);
            if (kind == NF_SLALOM_ACTION_LARGE_90) {
                (void)nf_route_maze_set_wall(&blocked, 3U, 3U,
                                             NF_ROUTE_DIR_NORTH);
            } else if (kind == NF_SLALOM_ACTION_LARGE_180) {
                (void)nf_route_maze_set_wall(&blocked, 4U, 3U,
                                             NF_ROUTE_DIR_NORTH);
            } else if (kind == NF_SLALOM_ACTION_45_IN ||
                       kind == NF_SLALOM_ACTION_135_IN) {
                close_anchor_wall(&blocked, open_check.destination_anchor);
            } else {
                close_anchor_wall(&blocked, source);
            }
            REQUIRE_TRUE(nf_slalom_primitive_check(
                             &blocked, &config, source, heading, kind,
                             NF_ROUTE_SIDE_RIGHT,
                             &blocked_check) == NF_SLALOM_PLAN_OK);
            if (blocked_check.feasible) {
                fprintf(stderr, "primitive wall rejection missed kind=%u\n",
                        kind_value);
            }
            CHECK_TRUE(!blocked_check.feasible);
        }
    }
}

/*
 * Independent exhaustive oracle: enumerate every centre anchor at which the
 * machine can stop beyond the first goal boundary, then compare the minimum
 * crossing time with the planner.  This does not call planner internals.
 */
static void test_exhaustive_straight_oracle(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    NfSlalomPlannerRequest request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan plan;
    NfLinearPlan start;
    uint64_t start_us;
    double start_exit;
    uint64_t oracle_us = UINT64_MAX;

    REQUIRE_TRUE(make_open_maze(&maze, 1U, 4U));
    maze.goals[2][0] = true;
    REQUIRE_TRUE(make_config(&config));
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
                 NF_SLALOM_PLAN_OK);
    check_plan_contract(&maze, &config, &request, &plan);
    CHECK_TRUE(plan.goal_x == 0U && plan.goal_y == 2U);

    REQUIRE_TRUE(nf_motion_accelerating_exit_velocity(
                     &config.orthogonal, config.start_offset_mm, 0.0,
                     &start_exit) == NF_MOTION_OK);
    REQUIRE_TRUE(nf_motion_linear_plan(&config.orthogonal,
                                       config.start_offset_mm, 0.0,
                                       start_exit, &start) == NF_MOTION_OK);
    REQUIRE_TRUE(nf_motion_seconds_to_us(start.total_time_s, &start_us) ==
                 NF_MOTION_OK);
    for (uint16_t stop_steps = 4U; stop_steps <= 6U; stop_steps += 2U) {
        NfLinearPlan terminal;
        double cross_time_s;
        double cross_velocity;
        uint64_t cross_us;
        if (nf_motion_linear_plan(&config.orthogonal,
                                  stop_steps * config.half_cell_mm,
                                  start_exit, 0.0, &terminal) != NF_MOTION_OK ||
            nf_motion_linear_time_at_distance(
                &terminal, 3.0 * config.half_cell_mm,
                &cross_time_s, &cross_velocity) != NF_MOTION_OK ||
            nf_motion_seconds_to_us(cross_time_s, &cross_us) != NF_MOTION_OK) {
            continue;
        }
        (void)cross_velocity;
        if (start_us + cross_us < oracle_us) {
            oracle_us = start_us + cross_us;
        }
    }
    CHECK_TRUE(oracle_us != UINT64_MAX);
    CHECK_TRUE(plan.goal_entry_us == oracle_us);
}

static void test_multiple_goals_first_entry(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    const NfSlalomPlannerRequest request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan plan;

    REQUIRE_TRUE(make_open_maze(&maze, 1U, 6U));
    maze.goals[2][0] = true;
    maze.goals[4][0] = true;
    REQUIRE_TRUE(make_config(&config));
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
                 NF_SLALOM_PLAN_OK);
    check_plan_contract(&maze, &config, &request, &plan);
    CHECK_TRUE(plan.goal_x == 0U && plan.goal_y == 2U);
}

static void test_turn_goal_uses_nominal_trace_and_shortest_tail(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    const NfSlalomPlannerRequest request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan plan;
    NfTurnPlan timing_plan;
    NfTurnPlan geometry_plan;
    NfTurnPose pose;
    double lower_s;
    double upper_s;
    uint64_t turn_cross_us;
    uint64_t expected_action_cross_us;

    REQUIRE_TRUE(make_open_maze(&maze, 4U, 4U));
    maze.goals[1][1] = true;
    REQUIRE_TRUE(make_config(&config));
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
                 NF_SLALOM_PLAN_OK);
    check_plan_contract(&maze, &config, &request, &plan);
    REQUIRE_TRUE(plan.action_count == 3U);
    CHECK_TRUE(plan.actions[1].kind == NF_SLALOM_ACTION_SMALL_90);
    CHECK_TRUE(plan.actions[1].side == NF_ROUTE_SIDE_RIGHT);
    CHECK_TRUE(plan.actions[1].has_goal_cross);
    CHECK_TRUE(plan.actions[1].goal_phase == NF_SLALOM_GOAL_TURN);
    CHECK_TRUE(plan.actions[2].kind == NF_SLALOM_ACTION_GOAL_STOP);
    CHECK_TRUE(plan.actions[2].connector_steps == 1U);

    REQUIRE_TRUE(nf_motion_turn_plan(&config.small_90,
                                     &config.turn_environment,
                                     &timing_plan) == NF_MOTION_OK);
    REQUIRE_TRUE(nf_motion_turn_plan(
                     &config.geometry_turns[NF_SLALOM_ACTION_SMALL_90],
                     &config.turn_environment, &geometry_plan) ==
                 NF_MOTION_OK);
    CHECK_TRUE(fabs((geometry_plan.travel_distance_mm /
                     timing_plan.total_time_s) -
                    config.small_90.velocity_mm_s) > 1.0);
    CHECK_TRUE(fabs(plan.goal_entry_velocity_mm_s -
                    config.small_90.velocity_mm_s) < 1.0e-6);
    lower_s = 0.0;
    upper_s = geometry_plan.total_time_s;
    for (unsigned int i = 0U; i < 64U; i++) {
        const double middle_s = 0.5 * (lower_s + upper_s);
        REQUIRE_TRUE(nf_motion_turn_pose_at_time(
                         &config.geometry_turns[
                             NF_SLALOM_ACTION_SMALL_90],
                         &geometry_plan, middle_s, &pose) ==
                     NF_MOTION_OK);
        if (pose.lateral_mm < config.half_cell_mm) {
            lower_s = middle_s;
        } else {
            upper_s = middle_s;
        }
    }
    REQUIRE_TRUE(nf_motion_seconds_to_us(
                     (0.5 * (lower_s + upper_s) /
                      geometry_plan.total_time_s) * timing_plan.total_time_s,
                     &turn_cross_us) == NF_MOTION_OK);
    expected_action_cross_us =
        plan.actions[1].connector_time_us + turn_cross_us;
    CHECK_TRUE(plan.actions[1].goal_cross_time_us >=
               expected_action_cross_us - 20U);
    CHECK_TRUE(plan.actions[1].goal_cross_time_us <=
               expected_action_cross_us + 20U);
}

static void test_turn_goal_interior_speed_profile(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    const NfSlalomPlannerRequest request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan plan;
    NfTurnPlan timing_plan;
    NfTurnPlan geometry_plan;
    NfTurnPose pose;
    double geometry_lower_s = 0.0;
    double geometry_upper_s;
    double time_lower_fraction = 0.0;
    double time_upper_fraction = 1.0;
    double geometry_fraction;
    double time_fraction;
    double amplitude;
    double expected_velocity;
    uint64_t expected_cross_us;

    REQUIRE_TRUE(make_open_maze(&maze, 4U, 4U));
    maze.goals[1][1] = true;
    REQUIRE_TRUE(make_config(&config));
    config.enabled_actions = NF_SLALOM_ENABLE_LARGE_90;
    config.start_offset_mm = 25.0;
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
                 NF_SLALOM_PLAN_OK);
    check_plan_contract(&maze, &config, &request, &plan);
    REQUIRE_TRUE(plan.action_count == 3U);
    REQUIRE_TRUE(plan.actions[1].kind == NF_SLALOM_ACTION_LARGE_90);
    CHECK_TRUE(plan.actions[1].side == NF_ROUTE_SIDE_RIGHT);
    CHECK_TRUE(plan.actions[1].connector_steps == 0U);
    CHECK_TRUE(plan.actions[1].has_goal_cross);

    REQUIRE_TRUE(nf_motion_turn_plan(&config.large_90,
                                     &config.turn_environment,
                                     &timing_plan) == NF_MOTION_OK);
    REQUIRE_TRUE(nf_motion_turn_plan(
                     &config.geometry_turns[NF_SLALOM_ACTION_LARGE_90],
                     &config.turn_environment, &geometry_plan) ==
                 NF_MOTION_OK);
    geometry_upper_s = geometry_plan.total_time_s;
    for (unsigned int i = 0U; i < 64U; i++) {
        const double middle_s =
            0.5 * (geometry_lower_s + geometry_upper_s);
        REQUIRE_TRUE(nf_motion_turn_pose_at_time(
                         &config.geometry_turns[
                             NF_SLALOM_ACTION_LARGE_90],
                         &geometry_plan, middle_s, &pose) == NF_MOTION_OK);
        if (pose.lateral_mm < config.half_cell_mm) {
            geometry_lower_s = middle_s;
        } else {
            geometry_upper_s = middle_s;
        }
    }
    geometry_fraction = 0.5 * (geometry_lower_s + geometry_upper_s) /
                        geometry_plan.total_time_s;
    amplitude = 2.0 *
        ((geometry_plan.travel_distance_mm / timing_plan.total_time_s) -
         config.large_90.velocity_mm_s);
    for (unsigned int i = 0U; i < 64U; i++) {
        const double middle =
            0.5 * (time_lower_fraction + time_upper_fraction);
        const double distance_mm = timing_plan.total_time_s *
            ((config.large_90.velocity_mm_s * middle) +
             (amplitude *
              ((0.5 * middle) -
               (sin(2.0 * 3.14159265358979323846 * middle) /
                (4.0 * 3.14159265358979323846)))));
        if (distance_mm <
                geometry_fraction * geometry_plan.travel_distance_mm) {
            time_lower_fraction = middle;
        } else {
            time_upper_fraction = middle;
        }
    }
    time_fraction = 0.5 * (time_lower_fraction + time_upper_fraction);
    expected_velocity = config.large_90.velocity_mm_s +
        (amplitude *
         sin(3.14159265358979323846 * time_fraction) *
         sin(3.14159265358979323846 * time_fraction));
    REQUIRE_TRUE(nf_motion_seconds_to_us(
                     timing_plan.total_time_s * time_fraction,
                     &expected_cross_us) == NF_MOTION_OK);
    CHECK_TRUE(plan.actions[1].goal_cross_time_us >= expected_cross_us - 20U);
    CHECK_TRUE(plan.actions[1].goal_cross_time_us <= expected_cross_us + 20U);
    CHECK_TRUE(fabs(plan.goal_entry_velocity_mm_s - expected_velocity) < 0.1);
    CHECK_TRUE(fabs(plan.goal_entry_velocity_mm_s -
                    config.large_90.velocity_mm_s) > 1.0);
}

static void test_diagonal_is_adopted(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    const NfSlalomPlannerRequest request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan plan;

    REQUIRE_TRUE(make_open_maze(&maze, 10U, 10U));
    maze.goals[8][8] = true;
    REQUIRE_TRUE(make_config(&config));
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
                 NF_SLALOM_PLAN_OK);
    check_plan_contract(&maze, &config, &request, &plan);
    CHECK_TRUE(plan.goal_x == 8U && plan.goal_y == 8U);
    CHECK_TRUE(plan_has_diagonal(&plan));
}

static void make_l_corridor(NfRouteMaze *maze, bool mirror)
{
    (void)make_open_maze(maze, 5U, 5U);
    close_all_internal_walls(maze);
    if (!mirror) {
        for (uint8_t y = 0U; y < 4U; y++) {
            open_between(maze, 0U, y, 0U, (uint8_t)(y + 1U));
        }
        for (uint8_t x = 0U; x < 4U; x++) {
            open_between(maze, x, 4U, (uint8_t)(x + 1U), 4U);
        }
        maze->goals[4][3] = true;
    } else {
        for (uint8_t y = 0U; y < 4U; y++) {
            open_between(maze, 4U, y, 4U, (uint8_t)(y + 1U));
        }
        for (uint8_t x = 4U; x > 0U; x--) {
            open_between(maze, x, 4U, (uint8_t)(x - 1U), 4U);
        }
        maze->goals[4][1] = true;
    }
}

static void test_walls_forbid_diagonal_and_mirror(void)
{
    NfRouteMaze right_maze;
    NfRouteMaze left_maze;
    NfSlalomPlannerConfig config;
    const NfSlalomPlannerRequest right_request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    const NfSlalomPlannerRequest left_request = {
        4U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan right_plan;
    NfSlalomRoutePlan left_plan;

    make_l_corridor(&right_maze, false);
    make_l_corridor(&left_maze, true);
    REQUIRE_TRUE(make_config(&config));
    REQUIRE_TRUE(nf_slalom_time_plan(&right_maze, &config, &right_request,
                                     &right_plan) == NF_SLALOM_PLAN_OK);
    REQUIRE_TRUE(nf_slalom_time_plan(&left_maze, &config, &left_request,
                                     &left_plan) == NF_SLALOM_PLAN_OK);
    check_plan_contract(&right_maze, &config, &right_request, &right_plan);
    check_plan_contract(&left_maze, &config, &left_request, &left_plan);
    CHECK_TRUE(!plan_has_diagonal(&right_plan));
    CHECK_TRUE(!plan_has_diagonal(&left_plan));
    CHECK_TRUE(right_plan.goal_entry_us == left_plan.goal_entry_us);
    CHECK_TRUE(right_plan.stop_us == left_plan.stop_us);
}

static void test_turn_route_oracle_and_validator_tamper(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    const NfSlalomPlannerRequest request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan plan;
    NfSlalomRoutePlan tampered;
    NfSlalomValidation validation;
    NfLinearPlan start_plan;
    double velocity;
    uint64_t elapsed_us = 0U;
    uint64_t oracle_goal_us = 0U;
    bool saw_turn = false;
    bool saw_goal = false;

    make_l_corridor(&maze, false);
    REQUIRE_TRUE(make_config(&config));
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
                 NF_SLALOM_PLAN_OK);
    REQUIRE_TRUE(nf_motion_accelerating_exit_velocity(
                     &config.orthogonal, config.start_offset_mm,
                     config.start_velocity_mm_s, &velocity) == NF_MOTION_OK);
    REQUIRE_TRUE(nf_motion_linear_plan(
                     &config.orthogonal, config.start_offset_mm,
                     config.start_velocity_mm_s, velocity,
                     &start_plan) == NF_MOTION_OK);
    REQUIRE_TRUE(nf_motion_seconds_to_us(start_plan.total_time_s,
                                         &elapsed_us) == NF_MOTION_OK);

    for (size_t i = 1U; i < plan.action_count; i++) {
        const NfSlalomAction *action = &plan.actions[i];
        const NfLinearLimits *limits = action->connector_is_diagonal ?
            &config.diagonal : &config.orthogonal;
        NfLinearPlan connector;
        uint64_t connector_us;

        if ((unsigned int)action->kind <
            (unsigned int)NF_SLALOM_ACTION_START_OFFSET) {
            const NfTurnSpec *turn = action_turn(&config, action->kind);
            NfTurnPlan turn_plan;
            uint64_t turn_us;
            saw_turn = true;
            REQUIRE_TRUE(turn != NULL);
            REQUIRE_TRUE(nf_motion_linear_plan(
                             limits, action->connector_command_distance_mm,
                             velocity, turn->velocity_mm_s,
                             &connector) == NF_MOTION_OK);
            REQUIRE_TRUE(nf_motion_seconds_to_us(connector.total_time_s,
                                                 &connector_us) ==
                         NF_MOTION_OK);
            REQUIRE_TRUE(nf_motion_turn_plan(
                             turn, &config.turn_environment,
                             &turn_plan) == NF_MOTION_OK);
            REQUIRE_TRUE(nf_motion_seconds_to_us(turn_plan.total_time_s,
                                                 &turn_us) == NF_MOTION_OK);
            elapsed_us += connector_us + turn_us;
            velocity = turn->velocity_mm_s;
        } else {
            double cross_distance = 0.0;
            double cross_time_s;
            double cross_velocity;
            uint64_t cross_us;
            REQUIRE_TRUE(action->kind == NF_SLALOM_ACTION_GOAL_STOP);
            REQUIRE_TRUE(nf_motion_linear_plan(
                             limits, action->connector_command_distance_mm,
                             velocity, 0.0, &connector) == NF_MOTION_OK);
            REQUIRE_TRUE(nf_motion_seconds_to_us(connector.total_time_s,
                                                 &connector_us) ==
                         NF_MOTION_OK);
            if (action->has_goal_cross) {
                int half_steps;
                REQUIRE_TRUE(!action->connector_is_diagonal);
                if (action->start_heading == NF_SLALOM_HEADING_EAST) {
                    half_steps = (2 * (int)action->goal_x) -
                                 action->start_anchor.half_x;
                } else if (action->start_heading == NF_SLALOM_HEADING_WEST) {
                    half_steps = action->start_anchor.half_x -
                                 (2 * ((int)action->goal_x + 1));
                } else if (action->start_heading == NF_SLALOM_HEADING_NORTH) {
                    half_steps = (2 * (int)action->goal_y) -
                                 action->start_anchor.half_y;
                } else {
                    half_steps = action->start_anchor.half_y -
                                 (2 * ((int)action->goal_y + 1));
                }
                REQUIRE_TRUE(half_steps >= 0);
                cross_distance = half_steps * config.half_cell_mm;
                REQUIRE_TRUE(nf_motion_linear_time_at_distance(
                                 &connector, cross_distance, &cross_time_s,
                                 &cross_velocity) == NF_MOTION_OK);
                REQUIRE_TRUE(nf_motion_seconds_to_us(cross_time_s,
                                                     &cross_us) ==
                             NF_MOTION_OK);
                oracle_goal_us = elapsed_us + cross_us;
                saw_goal = true;
                (void)cross_velocity;
            }
            elapsed_us += connector_us;
            velocity = 0.0;
        }
    }
    CHECK_TRUE(saw_turn);
    CHECK_TRUE(saw_goal);
    CHECK_TRUE(oracle_goal_us == plan.goal_entry_us);
    CHECK_TRUE(elapsed_us == plan.stop_us);

    tampered = plan;
    for (size_t i = 1U; i < tampered.action_count; i++) {
        if ((unsigned int)tampered.actions[i].kind <
            (unsigned int)NF_SLALOM_ACTION_START_OFFSET) {
            tampered.actions[i].duration_us++;
            break;
        }
    }
    CHECK_TRUE(!nf_slalom_route_validate(&maze, &config, &request,
                                          &tampered, &validation));
    CHECK_TRUE(!validation.valid);

    tampered = plan;
    tampered.actions[0].turn_start_x_mm += 1.0;
    CHECK_TRUE(!nf_slalom_route_validate(&maze, &config, &request,
                                          &tampered, &validation));
    CHECK_TRUE(!validation.valid);

    tampered = plan;
    tampered.actions[0].required_open_checked = false;
    CHECK_TRUE(!nf_slalom_route_validate(&maze, &config, &request,
                                          &tampered, &validation));
    CHECK_TRUE(!validation.valid);

    tampered = plan;
    tampered.actions[0].start_anchor.half_x += 2;
    CHECK_TRUE(!nf_slalom_route_validate(&maze, &config, &request,
                                          &tampered, &validation));
    CHECK_TRUE(!validation.valid);

    tampered = plan;
    for (size_t i = 1U; i < tampered.action_count; i++) {
        if ((unsigned int)tampered.actions[i].kind <
            (unsigned int)NF_SLALOM_ACTION_START_OFFSET) {
            tampered.actions[i].turn_start_x_mm += 1.0;
            break;
        }
    }
    CHECK_TRUE(!nf_slalom_route_validate(&maze, &config, &request,
                                          &tampered, &validation));
    CHECK_TRUE(!validation.valid);
}

static void test_rotation_symmetry(void)
{
    NfRouteMaze vertical;
    NfRouteMaze horizontal;
    NfSlalomPlannerConfig config;
    const NfSlalomPlannerRequest north = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    const NfSlalomPlannerRequest east = {
        0U, 0U, NF_SLALOM_HEADING_EAST,
    };
    NfSlalomRoutePlan north_plan;
    NfSlalomRoutePlan east_plan;

    REQUIRE_TRUE(make_open_maze(&vertical, 1U, 4U));
    REQUIRE_TRUE(make_open_maze(&horizontal, 4U, 1U));
    vertical.goals[2][0] = true;
    horizontal.goals[0][2] = true;
    REQUIRE_TRUE(make_config(&config));
    REQUIRE_TRUE(nf_slalom_time_plan(&vertical, &config, &north,
                                     &north_plan) == NF_SLALOM_PLAN_OK);
    REQUIRE_TRUE(nf_slalom_time_plan(&horizontal, &config, &east,
                                     &east_plan) == NF_SLALOM_PLAN_OK);
    CHECK_TRUE(north_plan.goal_entry_us == east_plan.goal_entry_us);
    CHECK_TRUE(north_plan.stop_us == east_plan.stop_us);
}

static void test_optional_swept_clearance_gate(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    const NfSlalomPlannerRequest request = {
        1U, 1U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan plan;
    NfSlalomRoutePlan tampered;
    NfSlalomValidation validation;
    bool saw_turn = false;

    REQUIRE_TRUE(make_open_maze(&maze, 9U, 9U));
    maze.goals[7][7] = true;
    REQUIRE_TRUE(make_config(&config));
    config.check_swept_clearance = true;
    config.clearance = (NfClearanceConfig){
        90.0, 1.0, 1.0, 1.0, 2.0, 2.0,
    };
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
                 NF_SLALOM_PLAN_OK);
    check_plan_contract(&maze, &config, &request, &plan);
    CHECK_TRUE(plan.swept_clearance_validated);
    for (size_t i = 1U; i < plan.action_count; i++) {
        if ((unsigned int)plan.actions[i].kind <
            (unsigned int)NF_SLALOM_ACTION_START_OFFSET) {
            CHECK_TRUE(plan.actions[i].swept_clearance_checked);
            saw_turn = true;
        }
    }
    REQUIRE_TRUE(saw_turn);

    tampered = plan;
    for (size_t i = 1U; i < tampered.action_count; i++) {
        if ((unsigned int)tampered.actions[i].kind <
            (unsigned int)NF_SLALOM_ACTION_START_OFFSET) {
            tampered.actions[i].swept_clearance_checked = false;
            break;
        }
    }
    CHECK_TRUE(!nf_slalom_route_validate(&maze, &config, &request,
                                          &tampered, &validation));

    REQUIRE_TRUE(make_config(&config));
    config.check_swept_clearance = true;
    config.clearance = (NfClearanceConfig){
        90.0, 1.0, 200.0, 200.0, 2.0, 2.0,
    };
    CHECK_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
               NF_SLALOM_PLAN_UNSUPPORTED_SAFETY);

    REQUIRE_TRUE(make_config(&config));
    config.require_swept_clearance = true;
    CHECK_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
               NF_SLALOM_PLAN_UNSUPPORTED_SAFETY);
}

static void test_determinism(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    const NfSlalomPlannerRequest request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan first;

    REQUIRE_TRUE(make_open_maze(&maze, 7U, 7U));
    maze.goals[5][5] = true;
    REQUIRE_TRUE(make_config(&config));
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &first) ==
                 NF_SLALOM_PLAN_OK);
    for (unsigned int repetition = 0U; repetition < 4U; repetition++) {
        NfSlalomRoutePlan repeated;
        REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request,
                                         &repeated) == NF_SLALOM_PLAN_OK);
        CHECK_TRUE(repeated.goal_entry_us == first.goal_entry_us);
        CHECK_TRUE(repeated.stop_us == first.stop_us);
        CHECK_TRUE(repeated.action_count == first.action_count);
        if (repeated.action_count == first.action_count) {
            for (size_t i = 0U; i < first.action_count; i++) {
                CHECK_TRUE(repeated.actions[i].kind == first.actions[i].kind);
                CHECK_TRUE(repeated.actions[i].side == first.actions[i].side);
                CHECK_TRUE(repeated.actions[i].connector_steps ==
                           first.actions[i].connector_steps);
                CHECK_TRUE(repeated.actions[i].end_anchor.half_x ==
                           first.actions[i].end_anchor.half_x);
                CHECK_TRUE(repeated.actions[i].end_anchor.half_y ==
                           first.actions[i].end_anchor.half_y);
            }
        }
    }
}

int main(void)
{
    test_config_closure_gate();
    test_fine_turn_sampling_is_topology_stable();
    test_maze_bits_and_start_goal_contract();
    test_all_primitive_transitions_and_wall_rejection();
    test_exhaustive_straight_oracle();
    test_multiple_goals_first_entry();
    test_turn_goal_uses_nominal_trace_and_shortest_tail();
    test_turn_goal_interior_speed_profile();
    test_diagonal_is_adopted();
    test_walls_forbid_diagonal_and_mirror();
    test_turn_route_oracle_and_validator_tamper();
    test_rotation_symmetry();
    test_optional_swept_clearance_gate();
    test_determinism();

    if (g_failures != 0U) {
        fprintf(stderr, "slalom_time_planner_tests: %u/%u checks failed\n",
                g_failures, g_checks);
        return 1;
    }
    printf("slalom_time_planner_tests: all %u checks passed\n", g_checks);
    return 0;
}
