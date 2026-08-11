#include "slalom_time_plan_host.h"

#include "shortest_run_params.h"

#include <math.h>
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
            fprintf(stderr, "%s:%d: requirement failed: %s\n",            \
                    __FILE__, __LINE__, #condition);                            \
            g_failures++;                                                       \
            return;                                                             \
        }                                                                       \
    } while (0)

static bool close_value(double left, double right)
{
    return isfinite(left) && isfinite(right) &&
           fabs(left - right) <= 1.0e-6 *
               fmax(1.0, fmax(fabs(left), fabs(right)));
}

static const ShortestRunCaseParams_t *case_table(uint8_t mode)
{
    switch (mode) {
    case 2U: return shortestRunCaseParamsMode2;
    case 3U: return shortestRunCaseParamsMode3;
    case 4U: return shortestRunCaseParamsMode4;
    case 5U: return shortestRunCaseParamsMode5;
    default: return NULL;
    }
}

static void test_profile_case_mapping(void)
{
    for (size_t profile_index = 0U;
         profile_index < nf_slalom_profile_count; profile_index++) {
        const NfAuditProfile *expected =
            &nf_slalom_profiles[profile_index];
        const ShortestRunCaseParams_t *cases =
            case_table(expected->shortest_run_mode);
        CHECK_TRUE(cases != NULL);
        if (cases == NULL) {
            continue;
        }
        for (uint8_t case_index = 8U; case_index <= 9U; case_index++) {
            NfSlalomPlannerConfig config;
            const NfAuditProfile *actual = NULL;
            char error[128];
            const ShortestRunCaseParams_t *run_case =
                &cases[case_index - 1U];

            CHECK_TRUE(nf_host_slalom_make_config(
                expected->name, case_index, NF_SLALOM_ENABLE_ALL,
                &config, &actual, error, sizeof(error)));
            CHECK_TRUE(actual == expected);
            CHECK_TRUE(strcmp(error, "ok") == 0);
            CHECK_TRUE(close_value(config.start_offset_mm,
                                   expected->start_offset_mm));
            CHECK_TRUE(close_value(config.orthogonal.vmax_mm_s,
                                   run_case->velocity_straight));
            CHECK_TRUE(close_value(config.orthogonal.accel_low_mm_s2,
                                   run_case->acceleration_straight));
            CHECK_TRUE(close_value(config.orthogonal.accel_high_mm_s2,
                                   run_case->acceleration_straight_dash));
            CHECK_TRUE(close_value(config.diagonal.vmax_mm_s,
                                   run_case->velocity_d_straight));
            CHECK_TRUE(config.diagonal.switch_velocity_mm_s == 0.0);
            CHECK_TRUE(close_value(config.diagonal.accel_high_mm_s2,
                                   run_case->acceleration_d_straight_dash));
            CHECK_TRUE(close_value(config.small_90.velocity_mm_s,
                                   expected->current[NF_PRIMITIVE_SMALL_90]
                                       .velocity_mm_s));
            CHECK_TRUE(close_value(config.low_speed_turn_velocity_mm_s,
                                   expected->current[NF_PRIMITIVE_SMALL_90]
                                       .velocity_mm_s));
            CHECK_TRUE(close_value(
                config.turn_45_in.velocity_mm_s,
                expected->primary ?
                    expected->current[NF_PRIMITIVE_45_IN].velocity_mm_s :
                    expected->seeds[NF_PRIMITIVE_45_IN].velocity_mm_s));
            CHECK_TRUE(close_value(
                config.turn_45_in.alpha_deg_s2,
                expected->primary ?
                    expected->current[NF_PRIMITIVE_45_IN].alpha_deg_s2 :
                    expected->seeds[NF_PRIMITIVE_45_IN].alpha_deg_s2));
            CHECK_TRUE(close_value(
                config.geometry_turns[NF_PRIMITIVE_SMALL_90].alpha_deg_s2,
                expected->seeds[NF_PRIMITIVE_SMALL_90].alpha_deg_s2));
            CHECK_TRUE(close_value(
                config.geometry_turns[NF_PRIMITIVE_45_IN].dist_out_mm,
                expected->seeds[NF_PRIMITIVE_45_IN].dist_out_mm));
            CHECK_TRUE(config.orthogonal_anchor_closure_tolerance_mm == 0.001);
            CHECK_TRUE(config.diagonal_anchor_closure_tolerance_mm == 0.001);
        }
    }
}

static void test_invalid_selection(void)
{
    NfSlalomPlannerConfig config;
    const NfAuditProfile *profile = NULL;
    char error[128];

    CHECK_TRUE(!nf_host_slalom_make_config(
        "missing", 8U, NF_SLALOM_ENABLE_ALL, &config, &profile,
        error, sizeof(error)));
    CHECK_TRUE(strstr(error, "unknown") != NULL);
    CHECK_TRUE(!nf_host_slalom_make_config(
        "f413-preorder-mode2", 0U, NF_SLALOM_ENABLE_ALL, &config, &profile,
        error, sizeof(error)));
    CHECK_TRUE(!nf_host_slalom_make_config(
        "f413-preorder-mode2", 8U, 0U, &config, &profile,
        error, sizeof(error)));
    CHECK_TRUE(!nf_host_slalom_make_config(
        "f413-preorder-mode2", 8U, UINT32_MAX, &config, &profile,
        error, sizeof(error)));
}

static void test_shortest_pattern_mask(void)
{
    CHECK_TRUE((NF_SLALOM_ENABLE_SHORTEST_1_TO_5 &
                NF_SLALOM_ENABLE_SMALL_90) == 0U);
    CHECK_TRUE((NF_SLALOM_ENABLE_SHORTEST_1_TO_5 &
                NF_SLALOM_ENABLE_LARGE_90) != 0U);
    CHECK_TRUE((NF_SLALOM_ENABLE_SHORTEST_1_TO_5 &
                NF_SLALOM_ENABLE_LARGE_180) != 0U);
    CHECK_TRUE((NF_SLALOM_ENABLE_SHORTEST_1_TO_5 &
                NF_SLALOM_ENABLE_45_IN) != 0U);
    CHECK_TRUE((NF_SLALOM_ENABLE_SHORTEST_1_TO_5 &
                NF_SLALOM_ENABLE_45_OUT) != 0U);
    CHECK_TRUE((NF_SLALOM_ENABLE_SHORTEST_1_TO_5 &
                NF_SLALOM_ENABLE_V90) != 0U);
    CHECK_TRUE((NF_SLALOM_ENABLE_SHORTEST_1_TO_5 &
                NF_SLALOM_ENABLE_135_IN) != 0U);
    CHECK_TRUE((NF_SLALOM_ENABLE_SHORTEST_1_TO_5 &
                NF_SLALOM_ENABLE_135_OUT) != 0U);
    CHECK_TRUE(NF_SLALOM_ENABLE_SHORTEST_1_TO_5 ==
               (NF_SLALOM_ENABLE_ALL & ~NF_SLALOM_ENABLE_SMALL_90));
}

static void test_shortest_pattern_route_excludes_small_90(void)
{
    NfRouteMaze maze;
    NfSlalomPlannerConfig config;
    const NfAuditProfile *profile = NULL;
    const NfSlalomPlannerRequest request = {
        0U, 0U, NF_SLALOM_HEADING_NORTH,
    };
    NfSlalomRoutePlan plan;
    NfSlalomValidation validation;
    size_t turn_count = 0U;
    char error[128];

    REQUIRE_TRUE(nf_route_maze_init(&maze, 10U, 10U));
    REQUIRE_TRUE(nf_route_maze_add_boundaries(&maze));
    maze.goals[4][3] = true;
    REQUIRE_TRUE(nf_host_slalom_make_config(
        "f413-preorder-mode2", 8U, NF_SLALOM_ENABLE_SHORTEST_1_TO_5,
        &config, &profile, error, sizeof(error)));
    REQUIRE_TRUE(profile != NULL);
    REQUIRE_TRUE(nf_slalom_time_plan(&maze, &config, &request, &plan) ==
                 NF_SLALOM_PLAN_OK);
    REQUIRE_TRUE(nf_slalom_route_validate(
        &maze, &config, &request, &plan, &validation));
    for (size_t i = 1U; i < plan.action_count; i++) {
        const NfSlalomAction *action = &plan.actions[i];
        if ((unsigned int)action->kind <
                (unsigned int)NF_SLALOM_ACTION_START_OFFSET) {
            turn_count++;
        }
        CHECK_TRUE(action->kind != NF_SLALOM_ACTION_SMALL_90);
    }
    CHECK_TRUE(turn_count > 0U);
    CHECK_TRUE(nf_slalom_route_validate(
        &maze, &config, &request, &plan, &validation));
}

int main(void)
{
    test_profile_case_mapping();
    test_invalid_selection();
    test_shortest_pattern_mask();
    test_shortest_pattern_route_excludes_small_90();
    if (g_failures != 0U) {
        fprintf(stderr,
                "slalom_time_plan_host_tests: %u/%u checks failed\n",
                g_failures, g_checks);
        return 1;
    }
    printf("slalom_time_plan_host_tests: all %u checks passed\n", g_checks);
    return 0;
}
