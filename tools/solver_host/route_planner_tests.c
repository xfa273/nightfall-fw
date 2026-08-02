#include "maze_ascii.h"
#include "orthogonal_time_planner.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static unsigned int g_checks;
static unsigned int g_failures;

#define CHECK_TRUE(expression)                                                     \
    do {                                                                           \
        g_checks++;                                                                \
        if (!(expression)) {                                                       \
            fprintf(stderr, "%s:%d: check failed: %s\n",                        \
                    __FILE__, __LINE__, #expression);                              \
            g_failures++;                                                          \
        }                                                                          \
    } while (0)

#define REQUIRE_TRUE(expression)                                                   \
    do {                                                                           \
        g_checks++;                                                                \
        if (!(expression)) {                                                       \
            fprintf(stderr, "%s:%d: required check failed: %s\n",               \
                    __FILE__, __LINE__, #expression);                              \
            g_failures++;                                                          \
            return;                                                                \
        }                                                                          \
    } while (0)

_Static_assert(NF_ROUTE_MOTION_START_OFFSET == 0,
               "orthogonal action enum must start at zero");
_Static_assert(NF_ROUTE_MOTION_LARGE_180 == 4,
               "unexpected non-orthogonal action inserted into action enum");

static void check_plan_status(const char *label,
                              NfRoutePlanStatus actual,
                              NfRoutePlanStatus expected)
{
    g_checks++;
    if (actual != expected) {
        fprintf(stderr, "%s: planner=%s expected=%s\n",
                label, nf_route_plan_status_name(actual),
                nf_route_plan_status_name(expected));
        g_failures++;
    }
}

static void check_ascii_status(const char *label,
                               NfMazeAsciiStatus actual,
                               NfMazeAsciiStatus expected,
                               const char *error)
{
    g_checks++;
    if (actual != expected) {
        fprintf(stderr, "%s: parser=%s expected=%s reason=%s\n",
                label, nf_maze_ascii_status_name(actual),
                nf_maze_ascii_status_name(expected),
                (error != NULL) ? error : "");
        g_failures++;
    }
}

static NfOrthogonalPlannerConfig base_config(bool allow_large)
{
    NfOrthogonalPlannerConfig config;
    memset(&config, 0, sizeof(config));
    config.half_cell_mm = 45.0;
    config.start_offset_mm = 5.0;
    config.straight.vmax_mm_s = 1500.0;
    config.straight.switch_velocity_mm_s = 2000.0;
    config.straight.accel_low_mm_s2 = 2000.0;
    config.straight.accel_high_mm_s2 = 2000.0;
    config.turn_environment.omega_cap_deg_s = 2200.0;
    config.turn_environment.rounding_scale = 1.2;

    config.small_90.enabled = true;
    config.small_90.velocity_mm_s = 300.0;
    config.small_90.alpha_deg_s2 = 8920.0;
    config.small_90.angle_deg = 90.0;
    config.small_90.dist_in_mm = 10.0;
    config.small_90.dist_out_mm = 14.2;

    config.large_90.enabled = true;
    config.large_90.velocity_mm_s = 500.0;
    config.large_90.alpha_deg_s2 = 4700.0;
    config.large_90.angle_deg = 90.0;
    config.large_90.dist_in_mm = 5.0;
    config.large_90.dist_out_mm = 15.0;

    config.large_180.enabled = true;
    config.large_180.velocity_mm_s = 500.0;
    config.large_180.alpha_deg_s2 = 4697.0;
    config.large_180.angle_deg = 180.0;
    config.large_180.dist_in_mm = 12.0;
    config.large_180.dist_out_mm = 19.0;
    config.allow_large_turns = allow_large;
    return config;
}

static NfOrthogonalPlannerConfig artificial_large_config(
    NfRouteMotionKind fast_kind)
{
    NfOrthogonalPlannerConfig config = base_config(true);
    NfTurnSpec slow = {
        .enabled = true,
        .velocity_mm_s = 100.0,
        .alpha_deg_s2 = 100.0,
        .angle_deg = 90.0,
        .dist_in_mm = 10.0,
        .dist_out_mm = 10.0,
    };
    NfTurnSpec fast90 = {
        .enabled = true,
        .velocity_mm_s = 500.0,
        .alpha_deg_s2 = 50000.0,
        .angle_deg = 90.0,
        .dist_in_mm = 0.0,
        /* Keep the synthetic endpoint-to-goal-boundary geometry feasible. */
        .dist_out_mm = 45.0,
    };
    NfTurnSpec fast180 = fast90;
    fast180.angle_deg = 180.0;

    config.small_90 = slow;
    config.large_90 = slow;
    config.large_180 = slow;
    config.large_180.angle_deg = 180.0;
    if (fast_kind == NF_ROUTE_MOTION_LARGE_90) {
        config.large_90 = fast90;
    } else {
        config.large_180 = fast180;
    }
    return config;
}

static bool open_maze(NfRouteMaze *maze, uint8_t width, uint8_t height)
{
    return nf_route_maze_init(maze, width, height) &&
           nf_route_maze_add_boundaries(maze);
}

static NfOrthogonalPlannerRequest request_at(uint8_t x, uint8_t y,
                                             NfRouteDirection heading)
{
    const NfOrthogonalPlannerRequest request = {
        .start_x = x,
        .start_y = y,
        .start_heading = heading,
    };
    return request;
}

static bool validate_plan(const char *label,
                          const NfRouteMaze *maze,
                          const NfOrthogonalPlannerConfig *config,
                          const NfOrthogonalPlannerRequest *request,
                          const NfOrthogonalRoutePlan *plan)
{
    NfRouteValidation validation;
    const bool valid = nf_orthogonal_route_validate(maze, config, request, plan,
                                                    &validation);
    g_checks++;
    if (!valid) {
        fprintf(stderr, "%s: validation failed at action %zu: %s\n",
                label, validation.action_index, validation.message);
        g_failures++;
    }
    return valid;
}

static size_t count_kind(const NfOrthogonalRoutePlan *plan,
                         NfRouteMotionKind kind)
{
    size_t count = 0U;
    for (size_t i = 0U; i < plan->action_count; i++) {
        if (plan->actions[i].kind == kind) {
            count++;
        }
    }
    return count;
}

static const NfRouteMotion *goal_action(const NfOrthogonalRoutePlan *plan)
{
    for (size_t i = 0U; i < plan->action_count; i++) {
        if (plan->actions[i].has_goal_cross) {
            return &plan->actions[i];
        }
    }
    return NULL;
}

static void check_actions_are_orthogonal(const char *label,
                                         const NfOrthogonalRoutePlan *plan)
{
    for (size_t i = 0U; i < plan->action_count; i++) {
        const NfRouteMotionKind kind = plan->actions[i].kind;
        g_checks++;
        if (kind < NF_ROUTE_MOTION_START_OFFSET ||
            kind > NF_ROUTE_MOTION_LARGE_180 ||
            strcmp(nf_route_motion_name(kind, plan->actions[i].side),
                   "UNKNOWN") == 0) {
            fprintf(stderr, "%s: action %zu has non-orthogonal/unknown kind %d\n",
                    label, i, (int)kind);
            g_failures++;
        }
    }
}

static void test_open_straight(void)
{
    NfRouteMaze maze;
    const NfOrthogonalPlannerConfig config = base_config(false);
    const NfOrthogonalPlannerRequest request =
        request_at(0U, 0U, NF_ROUTE_DIR_NORTH);
    NfOrthogonalRoutePlan plan;
    const NfRouteMotion *cross;

    REQUIRE_TRUE(open_maze(&maze, 1U, 4U));
    maze.goals[2][0] = true;
    check_plan_status("open straight",
                      nf_orthogonal_time_plan(&maze, &config, &request, &plan),
                      NF_ROUTE_PLAN_OK);
    REQUIRE_TRUE(validate_plan("open straight", &maze, &config, &request, &plan));
    CHECK_TRUE(plan.goal_x == 0U && plan.goal_y == 2U);
    CHECK_TRUE(plan.post_goal_extension_cells == 1U);
    CHECK_TRUE(count_kind(&plan, NF_ROUTE_MOTION_SMALL_90) == 0U);
    CHECK_TRUE(count_kind(&plan, NF_ROUTE_MOTION_LARGE_90) == 0U);
    CHECK_TRUE(count_kind(&plan, NF_ROUTE_MOTION_LARGE_180) == 0U);
    cross = goal_action(&plan);
    REQUIRE_TRUE(cross != NULL);
    CHECK_TRUE(cross->kind == NF_ROUTE_MOTION_STRAIGHT);
    CHECK_TRUE(cross->goal_cross_after_cells == 2U);
    CHECK_TRUE(cross->logical_cells == 3U);
    CHECK_TRUE(plan.goal_entry_us < plan.stop_us);
    check_actions_are_orthogonal("open straight", &plan);
}

static void test_wall_avoidance(void)
{
    NfRouteMaze maze;
    const NfOrthogonalPlannerConfig config = base_config(false);
    const NfOrthogonalPlannerRequest request =
        request_at(1U, 0U, NF_ROUTE_DIR_NORTH);
    NfOrthogonalRoutePlan plan;

    REQUIRE_TRUE(open_maze(&maze, 3U, 4U));
    REQUIRE_TRUE(nf_route_maze_set_wall(&maze, 1U, 1U,
                                        NF_ROUTE_DIR_NORTH));
    maze.goals[3][1] = true;
    check_plan_status("wall avoidance",
                      nf_orthogonal_time_plan(&maze, &config, &request, &plan),
                      NF_ROUTE_PLAN_OK);
    REQUIRE_TRUE(validate_plan("wall avoidance", &maze, &config, &request, &plan));
    CHECK_TRUE(count_kind(&plan, NF_ROUTE_MOTION_SMALL_90) >= 2U);
    CHECK_TRUE(plan.goal_x == 1U && plan.goal_y == 3U);
    check_actions_are_orthogonal("wall avoidance", &plan);
}

static void test_small_turn(void)
{
    NfRouteMaze maze;
    const NfOrthogonalPlannerConfig config = base_config(false);
    const NfOrthogonalPlannerRequest request =
        request_at(0U, 0U, NF_ROUTE_DIR_NORTH);
    NfOrthogonalRoutePlan plan;

    REQUIRE_TRUE(open_maze(&maze, 3U, 4U));
    maze.goals[2][1] = true;
    check_plan_status("small turn",
                      nf_orthogonal_time_plan(&maze, &config, &request, &plan),
                      NF_ROUTE_PLAN_OK);
    REQUIRE_TRUE(validate_plan("small turn", &maze, &config, &request, &plan));
    CHECK_TRUE(count_kind(&plan, NF_ROUTE_MOTION_SMALL_90) >= 1U);
    CHECK_TRUE(count_kind(&plan, NF_ROUTE_MOTION_LARGE_90) == 0U);
    CHECK_TRUE(count_kind(&plan, NF_ROUTE_MOTION_LARGE_180) == 0U);
    check_actions_are_orthogonal("small turn", &plan);
}

static void test_large_90_candidate(void)
{
    NfRouteMaze maze;
    const NfOrthogonalPlannerConfig config =
        artificial_large_config(NF_ROUTE_MOTION_LARGE_90);
    const NfOrthogonalPlannerRequest request =
        request_at(0U, 0U, NF_ROUTE_DIR_NORTH);
    NfOrthogonalRoutePlan plan;

    REQUIRE_TRUE(open_maze(&maze, 3U, 4U));
    REQUIRE_TRUE(nf_route_maze_set_wall(&maze, 0U, 1U,
                                        NF_ROUTE_DIR_EAST));
    maze.goals[2][1] = true;
    check_plan_status("large 90 candidate",
                      nf_orthogonal_time_plan(&maze, &config, &request, &plan),
                      NF_ROUTE_PLAN_OK);
    REQUIRE_TRUE(validate_plan("large 90 candidate", &maze, &config,
                               &request, &plan));
    CHECK_TRUE(count_kind(&plan, NF_ROUTE_MOTION_LARGE_90) >= 1U);
    check_actions_are_orthogonal("large 90 candidate", &plan);
}

static void test_large_180_candidate(void)
{
    NfRouteMaze maze;
    const NfOrthogonalPlannerConfig config =
        artificial_large_config(NF_ROUTE_MOTION_LARGE_180);
    const NfOrthogonalPlannerRequest request =
        request_at(0U, 0U, NF_ROUTE_DIR_NORTH);
    NfOrthogonalRoutePlan plan;

    REQUIRE_TRUE(open_maze(&maze, 3U, 4U));
    REQUIRE_TRUE(nf_route_maze_set_wall(&maze, 0U, 1U,
                                        NF_ROUTE_DIR_EAST));
    maze.goals[1][1] = true;
    check_plan_status("large 180 candidate",
                      nf_orthogonal_time_plan(&maze, &config, &request, &plan),
                      NF_ROUTE_PLAN_OK);
    REQUIRE_TRUE(validate_plan("large 180 candidate", &maze, &config,
                               &request, &plan));
    CHECK_TRUE(count_kind(&plan, NF_ROUTE_MOTION_LARGE_180) >= 1U);
    check_actions_are_orthogonal("large 180 candidate", &plan);
}

static void test_first_of_multiple_goals(void)
{
    NfRouteMaze maze;
    const NfOrthogonalPlannerConfig config = base_config(false);
    const NfOrthogonalPlannerRequest request =
        request_at(0U, 0U, NF_ROUTE_DIR_NORTH);
    NfOrthogonalRoutePlan plan;
    const NfRouteMotion *cross;

    REQUIRE_TRUE(open_maze(&maze, 1U, 5U));
    maze.goals[2][0] = true;
    maze.goals[3][0] = true;
    check_plan_status("multiple goals",
                      nf_orthogonal_time_plan(&maze, &config, &request, &plan),
                      NF_ROUTE_PLAN_OK);
    REQUIRE_TRUE(validate_plan("multiple goals", &maze, &config,
                               &request, &plan));
    CHECK_TRUE(plan.goal_x == 0U && plan.goal_y == 2U);
    cross = goal_action(&plan);
    REQUIRE_TRUE(cross != NULL);
    CHECK_TRUE(cross->goal_y == 2U);
    CHECK_TRUE(cross->goal_cross_after_cells == 2U);
    CHECK_TRUE(cross->logical_cells == 4U);
}

static void test_goal_extension_timing(void)
{
    NfRouteMaze no_extension_maze;
    NfRouteMaze extension_maze;
    const NfOrthogonalPlannerConfig config = base_config(false);
    const NfOrthogonalPlannerRequest request =
        request_at(0U, 0U, NF_ROUTE_DIR_NORTH);
    NfOrthogonalRoutePlan no_extension;
    NfOrthogonalRoutePlan extension;

    REQUIRE_TRUE(open_maze(&no_extension_maze, 1U, 3U));
    REQUIRE_TRUE(open_maze(&extension_maze, 1U, 3U));
    no_extension_maze.goals[1][0] = true;
    extension_maze.goals[1][0] = true;
    REQUIRE_TRUE(nf_route_maze_set_wall(&no_extension_maze, 0U, 1U,
                                        NF_ROUTE_DIR_NORTH));

    check_plan_status("goal no extension",
                      nf_orthogonal_time_plan(&no_extension_maze, &config,
                                              &request, &no_extension),
                      NF_ROUTE_PLAN_OK);
    check_plan_status("goal with extension",
                      nf_orthogonal_time_plan(&extension_maze, &config,
                                              &request, &extension),
                      NF_ROUTE_PLAN_OK);
    REQUIRE_TRUE(validate_plan("goal no extension", &no_extension_maze,
                               &config, &request, &no_extension));
    REQUIRE_TRUE(validate_plan("goal with extension", &extension_maze,
                               &config, &request, &extension));
    CHECK_TRUE(no_extension.post_goal_extension_cells == 0U);
    CHECK_TRUE(extension.post_goal_extension_cells == 1U);
    CHECK_TRUE(extension.goal_entry_us < no_extension.goal_entry_us);
    CHECK_TRUE(extension.stop_us > no_extension.stop_us);
    CHECK_TRUE(extension.goal_entry_velocity_mm_s >
               no_extension.goal_entry_velocity_mm_s);
}

static void test_no_path(void)
{
    NfRouteMaze maze;
    const NfOrthogonalPlannerConfig config = base_config(false);
    const NfOrthogonalPlannerRequest request =
        request_at(0U, 0U, NF_ROUTE_DIR_NORTH);
    NfOrthogonalRoutePlan plan;

    REQUIRE_TRUE(open_maze(&maze, 2U, 2U));
    REQUIRE_TRUE(nf_route_maze_set_wall(&maze, 0U, 0U,
                                        NF_ROUTE_DIR_NORTH));
    REQUIRE_TRUE(nf_route_maze_set_wall(&maze, 0U, 0U,
                                        NF_ROUTE_DIR_EAST));
    maze.goals[1][1] = true;
    check_plan_status("unreachable goal",
                      nf_orthogonal_time_plan(&maze, &config, &request, &plan),
                      NF_ROUTE_PLAN_NO_PATH);
}

static void test_asymmetric_wall_invalid(void)
{
    NfRouteMaze maze;
    const NfOrthogonalPlannerConfig config = base_config(false);
    const NfOrthogonalPlannerRequest request =
        request_at(0U, 0U, NF_ROUTE_DIR_NORTH);
    NfOrthogonalRoutePlan plan;

    REQUIRE_TRUE(open_maze(&maze, 2U, 2U));
    maze.goals[0][1] = true;
    maze.walls[0][0] |= NF_ROUTE_WALL_EAST;
    check_plan_status("asymmetric wall",
                      nf_orthogonal_time_plan(&maze, &config, &request, &plan),
                      NF_ROUTE_PLAN_INVALID_MAZE);
}

static void test_start_is_goal(void)
{
    NfRouteMaze maze;
    const NfOrthogonalPlannerConfig config = base_config(false);
    NfOrthogonalPlannerConfig invalid_config = config;
    const NfOrthogonalPlannerRequest request =
        request_at(0U, 0U, NF_ROUTE_DIR_NORTH);
    NfOrthogonalRoutePlan plan;

    REQUIRE_TRUE(open_maze(&maze, 2U, 2U));
    maze.goals[0][0] = true;
    check_plan_status("start is goal",
                      nf_orthogonal_time_plan(&maze, &config, &request, &plan),
                      NF_ROUTE_PLAN_OK);
    CHECK_TRUE(plan.action_count == 0U);
    CHECK_TRUE(plan.goal_entry_us == 0U);
    CHECK_TRUE(plan.stop_us == 0U);
    CHECK_TRUE(plan.goal_x == 0U && plan.goal_y == 0U);
    REQUIRE_TRUE(validate_plan("start is goal", &maze, &config,
                               &request, &plan));

    invalid_config.half_cell_mm = 0.0;
    check_plan_status("start goal still validates config",
                      nf_orthogonal_time_plan(&maze, &invalid_config,
                                              &request, &plan),
                      NF_ROUTE_PLAN_INVALID_CONFIG);
    invalid_config = config;
    invalid_config.small_90.angle_deg = 80.0;
    check_plan_status("logical turn angle contract",
                      nf_orthogonal_time_plan(&maze, &invalid_config,
                                              &request, &plan),
                      NF_ROUTE_PLAN_INVALID_CONFIG);
}

static void test_time_overflow_status(void)
{
    NfRouteMaze maze;
    NfOrthogonalPlannerConfig turn_overflow = base_config(false);
    NfOrthogonalPlannerConfig terminal_overflow = base_config(false);
    NfOrthogonalPlannerConfig arithmetic_overflow = base_config(false);
    const NfOrthogonalPlannerRequest request =
        request_at(0U, 0U, NF_ROUTE_DIR_NORTH);
    NfOrthogonalRoutePlan plan;

    REQUIRE_TRUE(open_maze(&maze, 1U, 2U));
    maze.goals[1][0] = true;

    turn_overflow.small_90.alpha_deg_s2 = 1.0e-30;
    check_plan_status("turn microseconds overflow",
                      nf_orthogonal_time_plan(&maze, &turn_overflow,
                                              &request, &plan),
                      NF_ROUTE_PLAN_OVERFLOW);

    terminal_overflow.straight.vmax_mm_s = 1.0e-12;
    terminal_overflow.straight.switch_velocity_mm_s = 0.0;
    terminal_overflow.straight.accel_low_mm_s2 = 1.0e-25;
    terminal_overflow.straight.accel_high_mm_s2 = 1.0e-25;
    terminal_overflow.small_90.velocity_mm_s = 1.0e-12;
    terminal_overflow.small_90.dist_in_mm = 0.0;
    terminal_overflow.small_90.dist_out_mm = 0.0;
    check_plan_status("goal terminal microseconds overflow",
                      nf_orthogonal_time_plan(&maze, &terminal_overflow,
                                              &request, &plan),
                      NF_ROUTE_PLAN_OVERFLOW);

    arithmetic_overflow.small_90.velocity_mm_s = DBL_MIN;
    check_plan_status("turn arithmetic overflow",
                      nf_orthogonal_time_plan(&maze, &arithmetic_overflow,
                                              &request, &plan),
                      NF_ROUTE_PLAN_OVERFLOW);
}

static void test_invalid_public_inputs(void)
{
    NfRouteMaze maze;
    const NfOrthogonalPlannerConfig config = base_config(false);
    NfOrthogonalPlannerRequest request =
        request_at(0U, 0U, NF_ROUTE_DIR_NORTH);
    NfOrthogonalRoutePlan plan;
    NfRouteValidation validation;

    REQUIRE_TRUE(open_maze(&maze, 2U, 2U));
    maze.goals[1][1] = true;
    CHECK_TRUE(!nf_route_maze_set_wall(&maze, 0U, 0U,
                                       (NfRouteDirection)99));
    CHECK_TRUE(!nf_route_maze_can_move(&maze, 0U, 0U,
                                       (NfRouteDirection)99));
    request.start_heading = (NfRouteDirection)99;
    check_plan_status("invalid start heading",
                      nf_orthogonal_time_plan(&maze, &config, &request, &plan),
                      NF_ROUTE_PLAN_INVALID_ARGUMENT);
    CHECK_TRUE(!nf_orthogonal_route_validate(&maze, &config, &request, &plan,
                                             &validation));
    CHECK_TRUE(strstr(validation.message, "invalid maze or start") != NULL);

    memset(&maze, 0, sizeof(maze));
    maze.width = UINT8_MAX;
    maze.height = UINT8_MAX;
    CHECK_TRUE(!nf_route_maze_set_wall(&maze, 0U, 0U,
                                       NF_ROUTE_DIR_NORTH));
    CHECK_TRUE(!nf_route_maze_add_boundaries(&maze));
    CHECK_TRUE(!nf_route_maze_can_move(&maze, 0U, 0U,
                                       NF_ROUTE_DIR_NORTH));
}

static void test_determinism_and_tamper_detection(void)
{
    NfRouteMaze maze;
    const NfOrthogonalPlannerConfig config = base_config(false);
    const NfOrthogonalPlannerRequest request =
        request_at(0U, 0U, NF_ROUTE_DIR_NORTH);
    NfOrthogonalRoutePlan first;
    NfOrthogonalRoutePlan second;
    NfOrthogonalRoutePlan tampered;
    NfRouteValidation validation;

    REQUIRE_TRUE(open_maze(&maze, 3U, 4U));
    maze.goals[2][1] = true;
    check_plan_status("determinism first",
                      nf_orthogonal_time_plan(&maze, &config, &request, &first),
                      NF_ROUTE_PLAN_OK);
    check_plan_status("determinism second",
                      nf_orthogonal_time_plan(&maze, &config, &request, &second),
                      NF_ROUTE_PLAN_OK);
    CHECK_TRUE(memcmp(&first, &second, sizeof(first)) == 0);
    REQUIRE_TRUE(validate_plan("determinism", &maze, &config, &request, &first));
    REQUIRE_TRUE(first.action_count >= 2U);

    tampered = first;
    tampered.actions[1].end_x = (uint8_t)(tampered.actions[1].end_x + 1U);
    CHECK_TRUE(!nf_orthogonal_route_validate(&maze, &config, &request, &tampered,
                                             &validation));
    CHECK_TRUE(!validation.valid);
    CHECK_TRUE(strstr(validation.message, "pose") != NULL);

    tampered = first;
    tampered.actions[1].duration_us++;
    CHECK_TRUE(!nf_orthogonal_route_validate(&maze, &config, &request, &tampered,
                                             &validation));
    CHECK_TRUE(strstr(validation.message, "timing") != NULL);

    tampered = first;
    tampered.actions[1].distance_mm += 1.0;
    CHECK_TRUE(!nf_orthogonal_route_validate(&maze, &config, &request, &tampered,
                                             &validation));
    CHECK_TRUE(strstr(validation.message, "timing") != NULL);
}

static void test_ascii_parser(void)
{
    static const char valid[] =
        "+---+---+\n"
        "|   | G |\n"
        "+   +   +\n"
        "| S     |\n"
        "+---+---+\n";
    static const char unknown[] =
        "+---+---+\n"
        "|   | G |\n"
        "+ . +   +\n"
        "| S     |\n"
        "+---+---+\n";
    static const char duplicate_start[] =
        "+---+---+\n"
        "| S | G |\n"
        "+   +   +\n"
        "| S     |\n"
        "+---+---+\n";
    static const char missing_goal[] =
        "+---+---+\n"
        "|   |   |\n"
        "+   +   +\n"
        "| S     |\n"
        "+---+---+\n";
    static const char invalid_junction[] =
        "+---x---+\n"
        "|   | G |\n"
        "+   +   +\n"
        "| S     |\n"
        "+---+---+\n";
    static const char invalid_cell_glyph[] =
        "+---+---+\n"
        "| X | G |\n"
        "+   +   +\n"
        "| S     |\n"
        "+---+---+\n";
    NfRouteMaze maze;
    NfMazeAsciiInfo info;
    char error[192];

    memset(&info, 0, sizeof(info));
    check_ascii_status("valid S/G",
                       nf_maze_ascii_parse(valid, &maze, &info,
                                           error, sizeof(error)),
                       NF_MAZE_ASCII_OK, error);
    CHECK_TRUE(info.start_x == 0U && info.start_y == 0U);
    CHECK_TRUE(info.goal_count == 1U);
    CHECK_TRUE(maze.goals[1][1]);
    CHECK_TRUE(nf_route_maze_can_move(&maze, 0U, 0U, NF_ROUTE_DIR_EAST));

    check_ascii_status("unknown dot",
                       nf_maze_ascii_parse(unknown, &maze, &info,
                                           error, sizeof(error)),
                       NF_MAZE_ASCII_UNKNOWN_WALL, error);
    check_ascii_status("duplicate S",
                       nf_maze_ascii_parse(duplicate_start, &maze, &info,
                                           error, sizeof(error)),
                       NF_MAZE_ASCII_INVALID_MARKERS, error);
    check_ascii_status("missing G",
                       nf_maze_ascii_parse(missing_goal, &maze, &info,
                                           error, sizeof(error)),
                       NF_MAZE_ASCII_INVALID_MARKERS, error);
    check_ascii_status("invalid junction",
                       nf_maze_ascii_parse(invalid_junction, &maze, &info,
                                           error, sizeof(error)),
                       NF_MAZE_ASCII_INVALID_FORMAT, error);
    check_ascii_status("invalid cell glyph",
                       nf_maze_ascii_parse(invalid_cell_glyph, &maze, &info,
                                           error, sizeof(error)),
                       NF_MAZE_ASCII_INVALID_FORMAT, error);
}

int main(void)
{
    test_open_straight();
    test_wall_avoidance();
    test_small_turn();
    test_large_90_candidate();
    test_large_180_candidate();
    test_first_of_multiple_goals();
    test_goal_extension_timing();
    test_no_path();
    test_asymmetric_wall_invalid();
    test_start_is_goal();
    test_time_overflow_status();
    test_invalid_public_inputs();
    test_determinism_and_tamper_detection();
    test_ascii_parser();

    if (g_failures != 0U) {
        fprintf(stderr, "route_planner_tests: %u/%u checks failed\n",
                g_failures, g_checks);
        return 1;
    }
    printf("route_planner_tests: all %u checks passed\n", g_checks);
    return 0;
}
