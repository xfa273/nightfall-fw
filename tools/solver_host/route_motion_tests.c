#include "motion_time.h"

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

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
                         NfMotionStatus actual,
                         NfMotionStatus expected)
{
    g_checks++;
    if (actual != expected) {
        fprintf(stderr, "%s: status=%s expected=%s\n",
                label, nf_motion_status_name(actual),
                nf_motion_status_name(expected));
        g_failures++;
    }
}

static void check_near(const char *label,
                       double actual,
                       double expected,
                       double tolerance)
{
    g_checks++;
    if (!isfinite(actual) || fabs(actual - expected) > tolerance) {
        fprintf(stderr, "%s: actual=%.12f expected=%.12f tolerance=%.3g\n",
                label, actual, expected, tolerance);
        g_failures++;
    }
}

static uint64_t plan_time_us(const char *label, double seconds)
{
    uint64_t value = 0U;
    check_status(label, nf_motion_seconds_to_us(seconds, &value), NF_MOTION_OK);
    return value;
}

static void check_us(const char *label, double seconds, uint64_t expected)
{
    const uint64_t actual = plan_time_us(label, seconds);
    g_checks++;
    if (actual != expected) {
        fprintf(stderr, "%s: actual=%llu expected=%llu\n",
                label,
                (unsigned long long)actual,
                (unsigned long long)expected);
        g_failures++;
    }
}

static void test_single_stage_straight(void)
{
    const NfLinearLimits limits = {
        .vmax_mm_s = 1500.0,
        .switch_velocity_mm_s = 2000.0,
        .accel_low_mm_s2 = 2000.0,
        .accel_high_mm_s2 = 2000.0,
    };
    NfLinearPlan plan;
    double sample_time;
    double sample_velocity;

    check_status("single-stage straight",
                 nf_motion_linear_plan(&limits, 90.0, 0.0, 300.0, &plan),
                 NF_MOTION_OK);
    check_near("single peak", plan.peak_velocity_mm_s,
               474.341649025257, 1.0e-9);
    check_near("single total", plan.total_time_s,
               0.324341649025257, 1.0e-12);
    CHECK_TRUE(plan.phase_count == 2U);
    CHECK_TRUE(plan.phases[0].kind == NF_LINEAR_PHASE_ACCEL_LOW);
    CHECK_TRUE(plan.phases[1].kind == NF_LINEAR_PHASE_DECEL_LOW);
    check_near("single accel distance", plan.phases[0].distance_mm,
               56.25, 1.0e-9);
    check_near("single decel distance", plan.phases[1].distance_mm,
               33.75, 1.0e-9);
    check_us("single total us", plan.total_time_s, 324342U);

    check_status("single sample peak",
                 nf_motion_linear_time_at_distance(&plan, 56.25,
                                                   &sample_time,
                                                   &sample_velocity),
                 NF_MOTION_OK);
    check_near("single sampled peak velocity", sample_velocity,
               plan.peak_velocity_mm_s, 1.0e-9);
    check_near("single sampled peak time", sample_time,
               plan.phases[0].duration_s, 1.0e-12);

    check_status("single sample end",
                 nf_motion_linear_time_at_distance(&plan, 90.0,
                                                   &sample_time,
                                                   &sample_velocity),
                 NF_MOTION_OK);
    check_near("single sampled end velocity", sample_velocity, 300.0, 1.0e-9);
    check_near("single sampled end time", sample_time, plan.total_time_s, 1.0e-12);
}

static void test_two_stage_straight(void)
{
    const NfLinearLimits limits = {
        .vmax_mm_s = 2000.0,
        .switch_velocity_mm_s = 1000.0,
        .accel_low_mm_s2 = 1000.0,
        .accel_high_mm_s2 = 3000.0,
    };
    static const struct {
        double distance_mm;
        double peak_velocity_mm_s;
        double total_time_s;
        size_t phase_count;
    } cases[] = {
        {1000.0, 1000.0, 2.0, 2U},
        {2000.0, 2000.0, 2.666666666666667, 4U},
        {3000.0, 2000.0, 3.166666666666667, 5U},
    };

    for (size_t i = 0U; i < sizeof(cases) / sizeof(cases[0]); i++) {
        NfLinearPlan plan;
        char label[64];
        (void)snprintf(label, sizeof(label), "two-stage %.0fmm",
                       cases[i].distance_mm);
        check_status(label,
                     nf_motion_linear_plan(&limits, cases[i].distance_mm,
                                           0.0, 0.0, &plan),
                     NF_MOTION_OK);
        check_near("two-stage peak", plan.peak_velocity_mm_s,
                   cases[i].peak_velocity_mm_s, 1.0e-8);
        check_near("two-stage total", plan.total_time_s,
                   cases[i].total_time_s, 1.0e-12);
        CHECK_TRUE(plan.phase_count == cases[i].phase_count);
    }
}

static void test_one_stage_layout_contract(void)
{
    const NfLinearLimits low_only = {
        .vmax_mm_s = 500.0,
        .switch_velocity_mm_s = 600.0,
        .accel_low_mm_s2 = 1000.0,
        .accel_high_mm_s2 = 100.0,
    };
    const NfLinearLimits high_only = {
        .vmax_mm_s = 500.0,
        .switch_velocity_mm_s = 0.0,
        .accel_low_mm_s2 = 100.0,
        .accel_high_mm_s2 = 1000.0,
    };
    NfLinearPlan plan;

    check_status("switch above vmax uses low",
                 nf_motion_linear_plan(&low_only, 100.0, 0.0, 0.0, &plan),
                 NF_MOTION_OK);
    check_near("low-only peak", plan.peak_velocity_mm_s,
               316.227766016838, 1.0e-9);
    check_near("low-only time", plan.total_time_s,
               0.632455532033676, 1.0e-12);

    check_status("zero switch uses high",
                 nf_motion_linear_plan(&high_only, 100.0, 0.0, 0.0, &plan),
                 NF_MOTION_OK);
    check_near("high-only peak", plan.peak_velocity_mm_s,
               316.227766016838, 1.0e-9);
    check_near("high-only time", plan.total_time_s,
               0.632455532033676, 1.0e-12);
}

static void test_accelerating_exit_velocity(void)
{
    const NfLinearLimits limits = {
        .vmax_mm_s = 2000.0,
        .switch_velocity_mm_s = 1000.0,
        .accel_low_mm_s2 = 1000.0,
        .accel_high_mm_s2 = 3000.0,
    };
    double velocity;

    check_status("accelerating exit at switch",
                 nf_motion_accelerating_exit_velocity(&limits, 500.0, 0.0,
                                                      &velocity),
                 NF_MOTION_OK);
    check_near("accelerating switch velocity", velocity, 1000.0, 1.0e-9);
    check_status("accelerating exit above switch",
                 nf_motion_accelerating_exit_velocity(&limits, 750.0, 0.0,
                                                      &velocity),
                 NF_MOTION_OK);
    check_near("accelerating two-stage velocity", velocity,
               1581.138830084190, 1.0e-9);
    check_status("accelerating exit capped",
                 nf_motion_accelerating_exit_velocity(&limits, 2000.0, 0.0,
                                                      &velocity),
                 NF_MOTION_OK);
    check_near("accelerating capped velocity", velocity, 2000.0, 0.0);
}

static void check_mode2_turn(const char *label,
                             const NfTurnSpec *turn,
                             uint64_t expected_us,
                             double expected_distance_mm)
{
    const NfTurnEnvironment environment = {
        .omega_cap_deg_s = 2200.0,
        .rounding_scale = 1.2,
    };
    NfTurnPlan plan;

    check_status(label, nf_motion_turn_plan(turn, &environment, &plan),
                 NF_MOTION_OK);
    check_us(label, plan.total_time_s, expected_us);
    check_near("turn travel distance", plan.travel_distance_mm,
               expected_distance_mm, 1.0e-8);
}

static void test_mode2_turns(void)
{
    const NfTurnSpec small90 = {
        .enabled = true,
        .velocity_mm_s = 300.0,
        .alpha_deg_s2 = 8920.0,
        .angle_deg = 90.0,
        .dist_in_mm = 10.0,
        .dist_out_mm = 14.2,
    };
    const NfTurnSpec large90 = {
        .enabled = true,
        .velocity_mm_s = 500.0,
        .alpha_deg_s2 = 4700.0,
        .angle_deg = 90.0,
        .dist_in_mm = 5.0,
        .dist_out_mm = 15.0,
    };
    const NfTurnSpec large180 = {
        .enabled = true,
        .velocity_mm_s = 500.0,
        .alpha_deg_s2 = 4697.0,
        .angle_deg = 180.0,
        .dist_in_mm = 12.0,
        .dist_out_mm = 19.0,
    };
    NfTurnEnvironment clamped_environment = {
        .omega_cap_deg_s = 2200.0,
        .rounding_scale = 0.05,
    };
    NfTurnEnvironment explicit_environment = {
        .omega_cap_deg_s = 2200.0,
        .rounding_scale = 0.1,
    };
    NfTurnPlan clamped;
    NfTurnPlan explicit_minimum;

    check_mode2_turn("mode2 small90", &small90, 302107U,
                     90.632136045378);
    check_mode2_turn("mode2 large90", &large90, 345064U,
                     172.531824374190);
    check_mode2_turn("mode2 large180", &large180, 493563U,
                     246.781452136396);

    check_status("turn rounding clamp",
                 nf_motion_turn_plan(&small90, &clamped_environment, &clamped),
                 NF_MOTION_OK);
    check_status("turn explicit minimum rounding",
                 nf_motion_turn_plan(&small90, &explicit_environment,
                                     &explicit_minimum),
                 NF_MOTION_OK);
    check_near("turn minimum rounding parity", clamped.total_time_s,
               explicit_minimum.total_time_s, 1.0e-12);
}

static void test_turn_boundary_cross(void)
{
    const NfTurnEnvironment environment = {
        .omega_cap_deg_s = 2200.0,
        .rounding_scale = 1.2,
    };
    const NfTurnSpec large90 = {
        .enabled = true,
        .velocity_mm_s = 500.0,
        .alpha_deg_s2 = 4700.0,
        .angle_deg = 90.0,
        .dist_in_mm = 5.0,
        .dist_out_mm = 15.0,
    };
    const NfTurnSpec large180 = {
        .enabled = true,
        .velocity_mm_s = 500.0,
        .alpha_deg_s2 = 4697.0,
        .angle_deg = 180.0,
        .dist_in_mm = 12.0,
        .dist_out_mm = 19.0,
    };
    NfTurnSpec straight_exit = large90;
    NfTurnSpec too_short = large90;
    NfTurnPlan plan;
    double cross_s;
    double lateral_mm;

    check_status("large90 boundary plan",
                 nf_motion_turn_plan(&large90, &environment, &plan),
                 NF_MOTION_OK);
    check_near("large90 forward displacement",
               plan.displacement_forward_mm, 94.6470875821, 1.0e-8);
    check_near("large90 lateral displacement",
               plan.displacement_lateral_mm, 104.6470875821, 1.0e-8);
    check_status("large90 boundary cross",
                 nf_motion_turn_exit_boundary_cross(&large90, &plan, 45.0,
                                                     &cross_s, &lateral_mm),
                 NF_MOTION_OK);
    check_near("large90 boundary time", cross_s, 0.2550339661, 1.0e-9);
    check_near("large90 boundary lateral", lateral_mm, 0.6308746734, 1.0e-8);

    check_status("large180 boundary plan",
                 nf_motion_turn_plan(&large180, &environment, &plan),
                 NF_MOTION_OK);
    check_near("large180 forward displacement",
               plan.displacement_forward_mm, -7.0, 1.0e-8);
    check_near("large180 lateral displacement",
               plan.displacement_lateral_mm, 87.3277085857, 1.0e-8);
    check_status("large180 boundary cross",
                 nf_motion_turn_exit_boundary_cross(&large180, &plan, 45.0,
                                                     &cross_s, &lateral_mm),
                 NF_MOTION_OK);
    check_near("large180 boundary time", cross_s, 0.4035569439, 1.0e-9);
    check_near("large180 boundary lateral", lateral_mm, 0.2614300632, 1.0e-8);

    straight_exit.dist_out_mm = 60.0;
    check_status("straight-exit boundary plan",
                 nf_motion_turn_plan(&straight_exit, &environment, &plan),
                 NF_MOTION_OK);
    check_status("straight-exit boundary cross",
                 nf_motion_turn_exit_boundary_cross(
                     &straight_exit, &plan, 45.0, &cross_s, &lateral_mm),
                 NF_MOTION_OK);
    check_near("straight-exit analytic time", cross_s,
               plan.total_time_s - (45.0 / straight_exit.velocity_mm_s),
               1.0e-12);
    check_near("straight-exit analytic lateral", lateral_mm, 0.0, 0.0);

    too_short.alpha_deg_s2 = 50000.0;
    too_short.dist_in_mm = 0.0;
    too_short.dist_out_mm = 0.0;
    check_status("short turn boundary plan",
                 nf_motion_turn_plan(&too_short, &environment, &plan),
                 NF_MOTION_OK);
    check_status("short turn cannot reach exit boundary",
                 nf_motion_turn_exit_boundary_cross(&too_short, &plan, 45.0,
                                                     &cross_s, &lateral_mm),
                 NF_MOTION_INFEASIBLE);
}

static void test_goal_terminal(void)
{
    const NfLinearLimits limits = {
        .vmax_mm_s = 1500.0,
        .switch_velocity_mm_s = 2000.0,
        .accel_low_mm_s2 = 2000.0,
        .accel_high_mm_s2 = 2000.0,
    };
    NfGoalTerminalPlan no_extension;
    NfGoalTerminalPlan one_cell_extension;

    check_status("goal no extension",
                 nf_motion_goal_terminal_plan(&limits, 180.0, 45.0, 300.0,
                                              &no_extension),
                 NF_MOTION_OK);
    check_near("goal no extension peak",
               no_extension.full_plan.peak_velocity_mm_s,
               703.562363973514, 1.0e-9);
    check_near("goal no extension velocity",
               no_extension.goal_cross_velocity_mm_s,
               424.264068711928, 1.0e-9);
    check_us("goal no extension crossing",
             no_extension.goal_cross_time_s, 341430U);
    check_us("goal no extension stop",
             no_extension.full_plan.total_time_s, 553562U);

    check_status("goal one cell extension",
                 nf_motion_goal_terminal_plan(&limits, 180.0, 135.0, 300.0,
                                              &one_cell_extension),
                 NF_MOTION_OK);
    check_near("goal one extension peak",
               one_cell_extension.full_plan.peak_velocity_mm_s,
               821.583836257749, 1.0e-9);
    check_near("goal one extension velocity",
               one_cell_extension.goal_cross_velocity_mm_s,
               734.846922834954, 1.0e-9);
    check_us("goal one extension crossing",
             one_cell_extension.goal_cross_time_s, 304160U);
    check_us("goal one extension stop",
             one_cell_extension.full_plan.total_time_s, 671584U);
    CHECK_TRUE(one_cell_extension.goal_cross_time_s <
               no_extension.goal_cross_time_s);
}

static void test_invalid_inputs(void)
{
    const NfLinearLimits limits = {
        .vmax_mm_s = 1500.0,
        .switch_velocity_mm_s = 2000.0,
        .accel_low_mm_s2 = 2000.0,
        .accel_high_mm_s2 = 2000.0,
    };
    const NfTurnEnvironment turn_environment = {
        .omega_cap_deg_s = 2200.0,
        .rounding_scale = 1.2,
    };
    NfTurnSpec turn = {
        .enabled = true,
        .velocity_mm_s = 300.0,
        .alpha_deg_s2 = 8920.0,
        .angle_deg = 90.0,
        .dist_in_mm = 10.0,
        .dist_out_mm = 14.2,
    };
    NfLinearPlan linear_plan;
    NfTurnPlan turn_plan;
    uint64_t converted;
    const NfLinearLimits tiny_velocity_limits = {
        .vmax_mm_s = DBL_MIN,
        .switch_velocity_mm_s = 0.0,
        .accel_low_mm_s2 = 1.0,
        .accel_high_mm_s2 = 1.0,
    };

    check_status("insufficient braking distance",
                 nf_motion_linear_plan(&limits, 10.0, 500.0, 0.0,
                                       &linear_plan),
                 NF_MOTION_INFEASIBLE);
    check_status("velocity above vmax",
                 nf_motion_linear_plan(&limits, 100.0,
                                       nextafter(1500.0, INFINITY), 0.0,
                                       &linear_plan),
                 NF_MOTION_INVALID_ARGUMENT);
    check_status("zero distance velocity change",
                 nf_motion_linear_plan(&limits, 0.0, 10.0, 0.0,
                                       &linear_plan),
                 NF_MOTION_INFEASIBLE);
    check_status("linear duration overflow",
                 nf_motion_linear_plan(&tiny_velocity_limits, 10.0, 0.0, 0.0,
                                       &linear_plan),
                 NF_MOTION_OVERFLOW);

    turn.velocity_mm_s = 0.0;
    check_status("zero turn velocity",
                 nf_motion_turn_plan(&turn, &turn_environment, &turn_plan),
                 NF_MOTION_INVALID_ARGUMENT);
    turn.velocity_mm_s = 300.0;
    turn.alpha_deg_s2 = 0.0;
    check_status("zero turn alpha",
                 nf_motion_turn_plan(&turn, &turn_environment, &turn_plan),
                 NF_MOTION_INVALID_ARGUMENT);
    turn.alpha_deg_s2 = 8920.0;
    turn.enabled = false;
    check_status("disabled turn",
                 nf_motion_turn_plan(&turn, &turn_environment, &turn_plan),
                 NF_MOTION_INFEASIBLE);
    turn.enabled = true;
    turn.alpha_deg_s2 = 1.0e307;
    turn.angle_deg = 180.0;
    {
        NfTurnEnvironment uncapped = turn_environment;
        uncapped.omega_cap_deg_s = 0.0;
        check_status("turn intermediate product stays finite",
                     nf_motion_turn_plan(&turn, &uncapped, &turn_plan),
                     NF_MOTION_OK);
        CHECK_TRUE(isfinite(turn_plan.total_time_s));
        CHECK_TRUE(isfinite(turn_plan.displacement_forward_mm));
        CHECK_TRUE(isfinite(turn_plan.displacement_lateral_mm));
    }
    turn.alpha_deg_s2 = 8920.0;
    turn.angle_deg = 90.0;
    turn.velocity_mm_s = DBL_MIN;
    check_status("turn duration overflow",
                 nf_motion_turn_plan(&turn, &turn_environment, &turn_plan),
                 NF_MOTION_OVERFLOW);

    check_status("negative seconds",
                 nf_motion_seconds_to_us(-1.0, &converted),
                 NF_MOTION_INVALID_ARGUMENT);
    check_status("nan seconds",
                 nf_motion_seconds_to_us(NAN, &converted),
                 NF_MOTION_INVALID_ARGUMENT);
    check_status("sub-microsecond nonzero",
                 nf_motion_seconds_to_us(0.4e-6, &converted),
                 NF_MOTION_INVALID_ARGUMENT);
    check_status("microsecond conversion overflow",
                 nf_motion_seconds_to_us(18446744073710.0, &converted),
                 NF_MOTION_OVERFLOW);
}

static void test_exact_end_sample(void)
{
    const NfLinearLimits limits = {
        .vmax_mm_s = 0x1.bac59aff3d43cp+12,
        .switch_velocity_mm_s = 0x1.b6470e145b854p+10,
        .accel_low_mm_s2 = 0x1.3d4d0c34820d8p+17,
        .accel_high_mm_s2 = 0x1.e7826c478cf2cp+7,
    };
    NfLinearPlan plan;
    double sampled_time_s;
    double sampled_velocity_mm_s;

    check_status("exact endpoint plan",
                 nf_motion_linear_plan(&limits, 0x1.798d0cd55abb6p+16,
                                       0x1.bac59aff3d43cp+12, 0.0, &plan),
                 NF_MOTION_OK);
    check_status("exact endpoint sample",
                 nf_motion_linear_time_at_distance(&plan, plan.distance_mm,
                                                   &sampled_time_s,
                                                   &sampled_velocity_mm_s),
                 NF_MOTION_OK);
    CHECK_TRUE(sampled_time_s == plan.total_time_s);
    CHECK_TRUE(sampled_velocity_mm_s == plan.exit_velocity_mm_s);
}

int main(void)
{
    test_single_stage_straight();
    test_two_stage_straight();
    test_one_stage_layout_contract();
    test_accelerating_exit_velocity();
    test_mode2_turns();
    test_turn_boundary_cross();
    test_goal_terminal();
    test_invalid_inputs();
    test_exact_end_sample();

    if (g_failures != 0U) {
        fprintf(stderr, "route_motion_tests: %u/%u checks failed\n",
                g_failures, g_checks);
        return 1;
    }
    printf("route_motion_tests: all %u checks passed\n", g_checks);
    return 0;
}
