#include "route_clearance.h"

#include <math.h>
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

static void check_status(const char *label,
                         NfClearanceStatus actual,
                         NfClearanceStatus expected)
{
    g_checks++;
    if (actual != expected) {
        fprintf(stderr, "%s: status=%s expected=%s\n", label,
                nf_clearance_status_name(actual),
                nf_clearance_status_name(expected));
        g_failures++;
    }
}

static NfClearanceConfig mini_footprint(void)
{
    const NfClearanceConfig config = {
        .cell_pitch_mm = 90.0,
        .wall_thickness_mm = 6.0,
        .robot_half_length_mm = 35.0,
        .robot_half_width_mm = 19.5,
        .max_translation_step_mm = 0.25,
        .max_heading_step_deg = 0.25,
    };
    return config;
}

static NfTurnSpec exact_large90(void)
{
    const NfTurnSpec turn = {
        .enabled = true,
        .velocity_mm_s = 500.0,
        .alpha_deg_s2 = 4700.0,
        .angle_deg = 90.0,
        .dist_in_mm = 0.353,
        .dist_out_mm = 0.353,
    };
    return turn;
}

static bool open_maze(NfRouteMaze *maze, uint8_t width, uint8_t height)
{
    return nf_route_maze_init(maze, width, height) &&
           nf_route_maze_add_boundaries(maze);
}

static void test_clear_and_blocked_turn(void)
{
    const NfClearanceConfig config = mini_footprint();
    const NfTurnSpec turn = exact_large90();
    const NfTurnEnvironment environment = {
        .omega_cap_deg_s = 2200.0,
        .rounding_scale = 1.2,
    };
    NfRouteMaze maze;
    NfTurnPlan plan;
    NfClearanceResult result;

    CHECK_TRUE(open_maze(&maze, 4U, 4U));
    CHECK_TRUE(nf_motion_turn_plan(&turn, &environment, &plan) == NF_MOTION_OK);
    check_status("open left large90",
                 nf_route_turn_clearance(&maze, &config, &turn, &plan,
                                         225.0, 45.0, 90.0, true, &result),
                 NF_CLEARANCE_OK);
    CHECK_TRUE(result.clear);
    CHECK_TRUE(result.sample_count > 100U);

    CHECK_TRUE(nf_route_maze_set_wall(&maze, 2U, 0U,
                                      NF_ROUTE_DIR_NORTH));
    check_status("wall blocks left large90",
                 nf_route_turn_clearance(&maze, &config, &turn, &plan,
                                         225.0, 45.0, 90.0, true, &result),
                 NF_CLEARANCE_COLLISION);
    CHECK_TRUE(!result.clear);
    CHECK_TRUE(result.wall_x == 2U && result.wall_y == 0U &&
               result.wall_direction == NF_ROUTE_DIR_NORTH);
    CHECK_TRUE(result.first_collision_time_s >= 0.0 &&
               result.first_collision_time_s <= plan.total_time_s);
}

static void test_mirror_symmetry(void)
{
    const NfClearanceConfig config = mini_footprint();
    const NfTurnSpec turn = exact_large90();
    const NfTurnEnvironment environment = {
        .omega_cap_deg_s = 2200.0,
        .rounding_scale = 1.2,
    };
    NfRouteMaze maze;
    NfTurnPlan plan;
    NfClearanceResult left;
    NfClearanceResult right;

    CHECK_TRUE(open_maze(&maze, 5U, 5U));
    CHECK_TRUE(nf_motion_turn_plan(&turn, &environment, &plan) == NF_MOTION_OK);
    check_status("symmetric left",
                 nf_route_turn_clearance(&maze, &config, &turn, &plan,
                                         225.0, 135.0, 90.0, true, &left),
                 NF_CLEARANCE_OK);
    check_status("symmetric right",
                 nf_route_turn_clearance(&maze, &config, &turn, &plan,
                                         225.0, 135.0, 90.0, false, &right),
                 NF_CLEARANCE_OK);
    CHECK_TRUE(left.sample_count == right.sample_count);
}

static void test_outer_wall_and_invalid_input(void)
{
    NfClearanceConfig config = mini_footprint();
    const NfClearanceConfig valid_config = mini_footprint();
    const NfTurnSpec turn = exact_large90();
    const NfTurnEnvironment environment = {
        .omega_cap_deg_s = 2200.0,
        .rounding_scale = 1.2,
    };
    NfRouteMaze maze;
    NfTurnPlan plan;
    NfClearanceResult result;

    CHECK_TRUE(open_maze(&maze, 4U, 4U));
    CHECK_TRUE(nf_motion_turn_plan(&turn, &environment, &plan) == NF_MOTION_OK);
    check_status("outer wall collision",
                 nf_route_turn_clearance(&maze, &config, &turn, &plan,
                                         45.0, 45.0, 180.0, true, &result),
                 NF_CLEARANCE_COLLISION);
    config.wall_thickness_mm = 0.0;
    check_status("invalid wall thickness",
                 nf_route_turn_clearance(&maze, &config, &turn, &plan,
                                         45.0, 45.0, 0.0, true, &result),
                 NF_CLEARANCE_INVALID_ARGUMENT);
    check_status("null output",
                 nf_route_turn_clearance(&maze, &valid_config, &turn, &plan,
                                         45.0, 45.0, 0.0, true, NULL),
                 NF_CLEARANCE_INVALID_ARGUMENT);
}

static void test_invalid_maze_is_rejected(void)
{
    const NfClearanceConfig config = mini_footprint();
    const NfTurnSpec turn = exact_large90();
    const NfTurnEnvironment environment = {
        .omega_cap_deg_s = 2200.0,
        .rounding_scale = 1.2,
    };
    NfRouteMaze maze;
    NfTurnPlan plan;
    NfClearanceResult result;

    CHECK_TRUE(open_maze(&maze, 4U, 4U));
    CHECK_TRUE(nf_motion_turn_plan(&turn, &environment, &plan) == NF_MOTION_OK);
    maze.walls[1][1] |= NF_ROUTE_WALL_EAST;
    check_status("asymmetric internal wall rejected",
                 nf_route_turn_clearance(&maze, &config, &turn, &plan,
                                         225.0, 135.0, 90.0, true, &result),
                 NF_CLEARANCE_INVALID_ARGUMENT);

    CHECK_TRUE(open_maze(&maze, 4U, 4U));
    maze.walls[0][0] &= (uint8_t)~NF_ROUTE_WALL_SOUTH;
    check_status("missing boundary rejected",
                 nf_route_turn_clearance(&maze, &config, &turn, &plan,
                                         225.0, 135.0, 90.0, true, &result),
                 NF_CLEARANCE_INVALID_ARGUMENT);

    CHECK_TRUE(open_maze(&maze, 4U, 4U));
    maze.walls[1][1] |= 0x80U;
    check_status("unknown wall bits rejected",
                 nf_route_turn_clearance(&maze, &config, &turn, &plan,
                                         225.0, 135.0, 90.0, true, &result),
                 NF_CLEARANCE_INVALID_ARGUMENT);
}

static void test_diagonal_primitives_on_maximum_maze(void)
{
    static const NfTurnSpec turns[] = {
        {true, 500.0, 7234.4, 45.0, 0.0, 18.640},
        {true, 500.0, 7234.4, 45.0, 18.640, 0.0},
        {true, 500.0, 12200.0, 90.0, 7.997, 7.997},
        {true, 500.0, 8500.0, 135.0, 18.932, 11.212},
        {true, 500.0, 8500.0, 135.0, 11.212, 18.932},
    };
    const NfClearanceConfig config = mini_footprint();
    const NfTurnEnvironment environment = {
        .omega_cap_deg_s = 2200.0,
        .rounding_scale = 1.2,
    };
    NfRouteMaze maze;

    CHECK_TRUE(open_maze(&maze, NF_ROUTE_MAZE_MAX_SIZE,
                          NF_ROUTE_MAZE_MAX_SIZE));
    for (size_t i = 0U; i < sizeof(turns) / sizeof(turns[0]); i++) {
        NfTurnPlan plan;
        NfClearanceResult left;
        NfClearanceResult right;
        CHECK_TRUE(nf_motion_turn_plan(&turns[i], &environment, &plan) ==
                   NF_MOTION_OK);
        check_status("diagonal primitive left clearance",
                     nf_route_turn_clearance(
                         &maze, &config, &turns[i], &plan,
                         16.0 * config.cell_pitch_mm,
                         16.0 * config.cell_pitch_mm, 90.0, true, &left),
                     NF_CLEARANCE_OK);
        check_status("diagonal primitive right clearance",
                     nf_route_turn_clearance(
                         &maze, &config, &turns[i], &plan,
                         16.0 * config.cell_pitch_mm,
                         16.0 * config.cell_pitch_mm, 90.0, false, &right),
                     NF_CLEARANCE_OK);
        CHECK_TRUE(left.clear && right.clear);
        CHECK_TRUE(left.sample_count == right.sample_count);
    }
}

int main(void)
{
    test_clear_and_blocked_turn();
    test_mirror_symmetry();
    test_outer_wall_and_invalid_input();
    test_invalid_maze_is_rejected();
    test_diagonal_primitives_on_maximum_maze();
    if (g_failures != 0U) {
        fprintf(stderr, "route_clearance_tests: %u/%u checks failed\n",
                g_failures, g_checks);
        return 1;
    }
    printf("route_clearance_tests: all %u checks passed\n", g_checks);
    return 0;
}
