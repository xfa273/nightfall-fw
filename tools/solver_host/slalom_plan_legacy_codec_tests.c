#include "slalom_plan_legacy_codec.h"
#include "slalom_time_plan_host.h"

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

static bool make_config(NfSlalomPlannerConfig *config)
{
    const NfAuditProfile *profile = NULL;
    char error[128];
    return nf_host_slalom_make_config(
        "f413-preorder-mode2", 8U,
        NF_SLALOM_ENABLE_SHORTEST_1_TO_5, config,
        &profile, error, sizeof(error));
}

static bool open_maze(NfRouteMaze *maze, uint8_t width, uint8_t height)
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
    } else if (y1 == (uint8_t)(y0 + 1U) && x0 == x1) {
        maze->walls[y0][x0] &= (uint8_t)~NF_ROUTE_WALL_NORTH;
        maze->walls[y1][x1] &= (uint8_t)~NF_ROUTE_WALL_SOUTH;
    }
}

static void test_straight_distance_ownership(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    const NfSlalomPlannerRequest request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan plan;
    const NfSlalomLegacyContract contract = {255U, 1U};
    uint16_t output[16];
    NfSlalomLegacyResult result;

    REQUIRE_TRUE(open_maze(&maze, 1U, 2U));
    maze.goals[1][0] = true;
    REQUIRE_TRUE(make_config(&config));
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
                 NF_SLALOM_PLAN_OK);
    memset(output, 0xA5, sizeof(output));
    result = nf_slalom_plan_to_legacy(
        &maze, &config, &request, &plan, &contract, output,
        sizeof(output) / sizeof(output[0]));
    CHECK_TRUE(result.status == NF_SLALOM_LEGACY_OK);
    CHECK_TRUE(result.geometry_compatible);
    CHECK_TRUE(!result.time_equivalent);
    CHECK_TRUE(result.length == 1U && result.required_capacity == 2U);
    CHECK_TRUE(output[0] == 201U && output[1] == 0U);

    output[0] = 0xCAFEU;
    result = nf_slalom_plan_to_legacy(
        &maze, &config, &request, &plan, &contract, output, 1U);
    CHECK_TRUE(result.status == NF_SLALOM_LEGACY_OUTPUT_CAPACITY);
    CHECK_TRUE(output[0] == 0xCAFEU);
}

static void test_orthogonal_turn_route(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    const NfSlalomPlannerRequest request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan plan;
    const NfSlalomLegacyContract contract = {255U, 1U};
    NfSlalomLegacyContract short_contract = {1U, 1U};
    uint16_t output[64];
    NfSlalomLegacyResult result;

    REQUIRE_TRUE(open_maze(&maze, 5U, 5U));
    close_all_internal_walls(&maze);
    for (uint8_t y = 0U; y < 4U; y++) {
        open_between(&maze, 0U, y, 0U, (uint8_t)(y + 1U));
    }
    for (uint8_t x = 0U; x < 4U; x++) {
        open_between(&maze, x, 4U, (uint8_t)(x + 1U), 4U);
    }
    maze.goals[4][3] = true;
    REQUIRE_TRUE(make_config(&config));
    config.enabled_actions = NF_SLALOM_ENABLE_SMALL_90 |
                             NF_SLALOM_ENABLE_LARGE_90 |
                             NF_SLALOM_ENABLE_LARGE_180;
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
                 NF_SLALOM_PLAN_OK);
    result = nf_slalom_plan_to_legacy(
        &maze, &config, &request, &plan, &contract, output,
        sizeof(output) / sizeof(output[0]));
    CHECK_TRUE(result.status == NF_SLALOM_LEGACY_OK);
    CHECK_TRUE(nf_legacy_path_validate(output,
                                       sizeof(output) / sizeof(output[0]))
                   .status == NF_LEGACY_PATH_OK);

    output[0] = 0xBEEFU;
    result = nf_slalom_plan_to_legacy(
        &maze, &config, &request, &plan, &short_contract, output,
        sizeof(output) / sizeof(output[0]));
    CHECK_TRUE(result.status == NF_SLALOM_LEGACY_CODE_LIMIT);
    CHECK_TRUE(output[0] == 0xBEEFU);

    plan.actions[1].end_anchor.half_x++;
    output[0] = 0x1234U;
    result = nf_slalom_plan_to_legacy(
        &maze, &config, &request, &plan, &contract, output,
        sizeof(output) / sizeof(output[0]));
    CHECK_TRUE(result.status == NF_SLALOM_LEGACY_ROUTE_INVALID);
    CHECK_TRUE(output[0] == 0x1234U);
}

static void test_diagonal_terminal_and_start_goal(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    const NfSlalomPlannerRequest request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan plan;
    const NfSlalomLegacyContract contract = {255U, 1U};
    uint16_t output[64];
    NfSlalomLegacyResult result;

    REQUIRE_TRUE(open_maze(&maze, 10U, 10U));
    maze.goals[2][2] = true;
    REQUIRE_TRUE(make_config(&config));
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
                 NF_SLALOM_PLAN_OK);
    REQUIRE_TRUE(plan.actions[plan.action_count - 1U].connector_is_diagonal);
    output[0] = 0x55AAU;
    result = nf_slalom_plan_to_legacy(
        &maze, &config, &request, &plan, &contract, output,
        sizeof(output) / sizeof(output[0]));
    CHECK_TRUE(result.status ==
               NF_SLALOM_LEGACY_TERMINAL_DIAGONAL_UNSUPPORTED);
    CHECK_TRUE(output[0] == 0x55AAU);

    REQUIRE_TRUE(open_maze(&maze, 2U, 2U));
    maze.goals[0][0] = true;
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
                 NF_SLALOM_PLAN_OK);
    output[0] = 0xAA55U;
    result = nf_slalom_plan_to_legacy(
        &maze, &config, &request, &plan, &contract, output,
        sizeof(output) / sizeof(output[0]));
    CHECK_TRUE(result.status == NF_SLALOM_LEGACY_NO_RUN_REQUIRED);
    CHECK_TRUE(result.required_capacity == 1U);
    CHECK_TRUE(output[0] == 0xAA55U);
}

int main(void)
{
    test_straight_distance_ownership();
    test_orthogonal_turn_route();
    test_diagonal_terminal_and_start_goal();
    if (g_failures != 0U) {
        fprintf(stderr,
                "slalom_plan_legacy_codec_tests: %u/%u checks failed\n",
                g_failures, g_checks);
        return 1;
    }
    printf("slalom_plan_legacy_codec_tests: all %u checks passed\n",
           g_checks);
    return 0;
}
