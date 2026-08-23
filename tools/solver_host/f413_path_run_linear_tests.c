#include "shortest_run_params.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define NIGHTFALL_F413_PATH_LINEAR_PLAN_HOST_TEST (1U)
#define NIGHTFALL_F413_PATH_DISTANCE_CURSOR_HOST_TEST (1U)
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_path_run.c"

static unsigned int g_checks;
static unsigned int g_failures;

#define CHECK(condition)                                                        \
  do                                                                            \
  {                                                                             \
    g_checks++;                                                                 \
    if (!(condition))                                                           \
    {                                                                           \
      g_failures++;                                                             \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);    \
    }                                                                           \
  } while (0)

static bool close_value(double actual, double expected, double tolerance)
{
  return isfinite(actual) && isfinite(expected) &&
         fabs(actual - expected) <= tolerance;
}

static void check_boundary_plan(const char* label,
                                double distance_mm,
                                const NfLinearLimits* limits)
{
  NfLinearPlan plan;
  double distance_sum = 0.0;
  double previous_velocity = 500.0;

  if (!f413_path_run_make_linear_plan((float)distance_mm,
                                      500.0f,
                                      500.0f,
                                      limits,
                                      &plan))
  {
    g_checks++;
    g_failures++;
    fprintf(stderr, "FAIL %s: no 500->500 plan\n", label);
    return;
  }

  CHECK(plan.phase_count >= 1U);
  CHECK(close_value(plan.distance_mm, distance_mm, 1.0e-4));
  CHECK(close_value(plan.entry_velocity_mm_s, 500.0, 1.0e-9));
  CHECK(close_value(plan.exit_velocity_mm_s, 500.0, 1.0e-9));
  CHECK(plan.peak_velocity_mm_s >= 500.0);
  CHECK(plan.peak_velocity_mm_s <= limits->vmax_mm_s + 1.0e-9);

  for (size_t phase_index = 0U; phase_index < plan.phase_count; phase_index++)
  {
    const NfLinearPhase* phase = &plan.phases[phase_index];
    CHECK(phase->distance_mm > 0.0);
    CHECK(close_value(phase->entry_velocity_mm_s,
                      previous_velocity,
                      1.0e-7));
    CHECK(phase->entry_velocity_mm_s >= 500.0 - 1.0e-7);
    CHECK(phase->exit_velocity_mm_s >= 500.0 - 1.0e-7);
    distance_sum += phase->distance_mm;
    previous_velocity = phase->exit_velocity_mm_s;
  }
  CHECK(close_value(distance_sum, distance_mm, 1.0e-4));
  CHECK(close_value(previous_velocity, 500.0, 1.0e-7));
}

static void check_mode2_case_profiles(void)
{
  static const double expected_straight_vmax[3] = {1000.0, 1250.0, 1500.0};
  static const double expected_diagonal_vmax[3] = {800.0, 900.0, 1000.0};
  static const double expected_acceleration[3] = {1000.0, 3000.0, 4000.0};

  for (size_t offset = 0U; offset < 3U; offset++)
  {
    const ShortestRunCaseParams_t* run_case =
        &shortestRunCaseParamsMode2[5U + offset];
    NfLinearLimits straight;
    NfLinearLimits diagonal;
    char label[40];

    CHECK(close_value(run_case->velocity_straight,
                      expected_straight_vmax[offset], 1.0e-6));
    CHECK(close_value(run_case->velocity_d_straight,
                      expected_diagonal_vmax[offset], 1.0e-6));
    CHECK(close_value(run_case->acceleration_straight,
                      expected_acceleration[offset], 1.0e-6));
    CHECK(close_value(run_case->acceleration_straight_dash,
                      expected_acceleration[offset], 1.0e-6));
    CHECK(close_value(run_case->acceleration_d_straight,
                      expected_acceleration[offset], 1.0e-6));
    CHECK(close_value(run_case->acceleration_d_straight_dash,
                      expected_acceleration[offset], 1.0e-6));
    CHECK(run_case->velocity_straight <= NIGHTFALL_F413_PATH_VELOCITY_CAP);
    CHECK(run_case->velocity_d_straight <=
          NIGHTFALL_F413_PATH_DIAGONAL_VELOCITY_CAP);

    straight = (NfLinearLimits){
        run_case->velocity_straight,
        shortestRunModeParams2.accel_switch_velocity,
        run_case->acceleration_straight,
        run_case->acceleration_straight_dash,
    };
    diagonal = (NfLinearLimits){
        run_case->velocity_d_straight,
        0.0,
        run_case->acceleration_d_straight,
        run_case->acceleration_d_straight_dash,
    };

    (void)snprintf(label, sizeof(label), "case%u S1",
                   (unsigned int)(offset + 6U));
    check_boundary_plan(label, 45.0, &straight);
    (void)snprintf(label, sizeof(label), "case%u DS1",
                   (unsigned int)(offset + 6U));
    check_boundary_plan(label, 67.279, &diagonal);
  }

  CHECK(shortestRunCaseParamsMode2[5].velocity_straight <
        shortestRunCaseParamsMode2[6].velocity_straight);
  CHECK(shortestRunCaseParamsMode2[6].velocity_straight <
        shortestRunCaseParamsMode2[7].velocity_straight);
  CHECK(shortestRunCaseParamsMode2[5].velocity_d_straight <
        shortestRunCaseParamsMode2[6].velocity_d_straight);
  CHECK(shortestRunCaseParamsMode2[6].velocity_d_straight <
        shortestRunCaseParamsMode2[7].velocity_d_straight);
}

static void check_turn_offset_trace_contract(void)
{
  const uint16_t base = (uint16_t)(NIGHTFALL_F413_TRACE_MODE_SHORTEST_SAFE_FLAG |
                                   NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                                   NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG |
                                   NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG);
  const uint16_t flags = f413_path_run_turn_offset_trace_flags(base);

  CHECK((flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG) != 0U);
  CHECK((flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG) == 0U);
  CHECK((flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG) == 0U);
  CHECK((flags & NIGHTFALL_F413_TRACE_MODE_SHORTEST_SAFE_FLAG) != 0U);
  CHECK((flags & NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG) != 0U);
}

static void check_start_runup_contract(void)
{
  NfLinearPlan plan;
  NfConstantAccelProfile terminal_profile;
  const NfLinearLimits case6 = {1000.0, 2000.0, 1000.0, 1000.0};
  const NfLinearLimits case7 = {1250.0, 2000.0, 3000.0, 3000.0};
  const NfLinearLimits case8 = {1500.0, 2000.0, 4000.0, 4000.0};
  const float case6_start = sqrtf(
      2.0f * 1000.0f * (float)DIST_FIRST_SEC);
  const float case7_start = sqrtf(
      2.0f * 3000.0f * (float)DIST_FIRST_SEC);
  const float case8_start = sqrtf(
      2.0f * 4000.0f * (float)DIST_FIRST_SEC);

  CHECK(close_value(DIST_FIRST_SEC, 10.0, 1.0e-9));
  CHECK(close_value(DIST_FIRST_SEC,
                    DIST_HALF_SEC - ROBOT_REAR_OVERHANG, 1.0e-9));
  CHECK(f413_path_run_make_linear_plan(45.0f, case6_start, 300.0f,
                                       &case6, &plan));
  CHECK(!f413_path_run_make_linear_plan(45.0f, case6_start, 500.0f,
                                        &case6, &plan));
  CHECK(!f413_path_run_make_linear_plan(90.0f, case6_start, 500.0f,
                                        &case6, &plan));
  CHECK(f413_path_run_make_linear_plan(45.0f, case7_start, 500.0f,
                                       &case7, &plan));
  CHECK(f413_path_run_make_linear_plan(45.0f, case8_start, 500.0f,
                                       &case8, &plan));
  CHECK(f413_path_run_make_test_terminal_profile(
      500.0f, &case6, &terminal_profile));
  CHECK(close_value(terminal_profile.distance_mm, 45.0, 1.0e-9));
  CHECK(close_value(terminal_profile.acceleration_mm_s2,
                    -2777.777777777778, 1.0e-6));
}

static void check_wall_end_approach_contract(void)
{
  const ShortestRunCaseParams_t* case6 = &shortestRunCaseParamsMode2[5];
  const ShortestRunCaseParams_t* case7 = &shortestRunCaseParamsMode2[6];
  const float case6_start = sqrtf(
      2.0f * case6->acceleration_straight * (float)DIST_FIRST_SEC);
  const float case7_start = sqrtf(
      2.0f * case7->acceleration_straight * (float)DIST_FIRST_SEC);
  NfLinearLimits limits;
  NfLinearPlan plan;
  bool execute_before_buffer = true;

  CHECK(f413_path_run_straight_limits(
      &shortestRunModeParams2, case6, &limits));
  CHECK(f413_path_run_make_wall_end_approach_plan(
      1U, case6_start, 300.0f, &limits, &plan,
      &execute_before_buffer));
  CHECK(!execute_before_buffer);
  {
    NfConstantAccelProfile profile;
    CHECK(nf_motion_constant_accel_profile(
              &limits, 45.0, case6_start, 300.0, &profile) == NF_MOTION_OK);
    CHECK(close_value(profile.acceleration_mm_s2,
                      (300.0 * 300.0 - case6_start * case6_start) / 90.0,
                      1.0e-4));
    CHECK(close_value(profile.duration_s,
                      90.0 / (case6_start + 300.0), 1.0e-7));
  }
  CHECK(!f413_path_run_make_wall_end_approach_plan(
      1U, case6_start, 500.0f, &limits, &plan,
      &execute_before_buffer));
  CHECK(f413_path_run_make_wall_end_approach_plan(
      2U, case6_start, 300.0f, &limits, &plan,
      &execute_before_buffer));
  CHECK(execute_before_buffer);
  CHECK(close_value(plan.distance_mm, 45.0, 1.0e-6));
  CHECK(!f413_path_run_make_wall_end_approach_plan(
      3U, case6_start, 500.0f, &limits, &plan,
      &execute_before_buffer));

  CHECK(f413_path_run_straight_limits(
      &shortestRunModeParams2, case7, &limits));
  CHECK(f413_path_run_make_wall_end_approach_plan(
      2U, case7_start, 500.0f, &limits, &plan,
      &execute_before_buffer));
  CHECK(execute_before_buffer);
}

static void expect_preflight(
    const char* label,
    const uint16_t* codes,
    size_t capacity,
    const ShortestRunCaseParams_t* run_case,
    bool wall_end_correction_enabled,
    bool test_mode_run,
    f413_path_run_preflight_status_t expected_status,
    size_t expected_index)
{
  const float initial_velocity = sqrtf(
      2.0f * run_case->acceleration_straight * (float)DIST_FIRST_SEC);
  const f413_path_run_preflight_result_t result = f413_path_run_preflight(
      codes, capacity, &shortestRunModeParams2, run_case,
      initial_velocity, wall_end_correction_enabled, test_mode_run);

  g_checks++;
  if ((result.status != expected_status) ||
      ((expected_status != F413_PATH_RUN_PREFLIGHT_OK) &&
       (result.index != expected_index)))
  {
    g_failures++;
    fprintf(stderr,
            "FAIL %s: status=%u/%u index=%zu/%zu legacy=%u code=%u\n",
            label,
            (unsigned int)result.status,
            (unsigned int)expected_status,
            result.index,
            expected_index,
            (unsigned int)result.legacy_status,
            (unsigned int)result.code);
  }
}

static void check_preflight_boundaries(void)
{
  static const uint16_t non_diagonal_case1[] = {
      202U, 300U, 202U, 0U};
  static const uint16_t case6_small_s1[] = {
      201U, 300U, 201U, 0U};
  static const uint16_t case6_large_s1[] = {
      201U, 501U, 203U, 0U};
  static const uint16_t case6_small_s3[] = {
      203U, 300U, 201U, 0U};
  static const uint16_t case7_large_s2[] = {
      202U, 501U, 202U, 0U};
  static const uint16_t continuous_orthogonal[] = {
      299U, 202U, 501U, 203U, 0U};
  static const uint16_t continuous_diagonal[] = {
      203U, 701U, 1099U, 1002U, 703U, 203U, 0U};
  static const uint16_t terminal_diagonal[] = {
      203U, 701U, 1001U, 0U};
  static const uint16_t diagonal_without_entry[] = {
      203U, 1001U, 0U};
  static const uint16_t leading_turn[] = {
      300U, 201U, 0U};
  static const uint16_t large_s1_small[] = {
      202U, 501U, 201U, 300U, 202U, 0U};
  static const uint16_t case6_open_floor_diagonal[] = {
      203U, 701U, 1001U, 802U, 1001U, 703U, 202U, 0U};

  expect_preflight("non-diagonal case1 keeps zero diagonal params",
                   non_diagonal_case1,
                   sizeof(non_diagonal_case1) /
                       sizeof(non_diagonal_case1[0]),
                   &shortestRunCaseParamsMode2[0], true,
                   false,
                   F413_PATH_RUN_PREFLIGHT_OK, 0U);
  expect_preflight("case6 S1 small fallback",
                   case6_small_s1,
                   sizeof(case6_small_s1) / sizeof(case6_small_s1[0]),
                   &shortestRunCaseParamsMode2[5], true,
                   false,
                   F413_PATH_RUN_PREFLIGHT_OK, 0U);
  expect_preflight("case6 S1 large rejected",
                   case6_large_s1,
                   sizeof(case6_large_s1) / sizeof(case6_large_s1[0]),
                   &shortestRunCaseParamsMode2[5], true,
                   false,
                   F413_PATH_RUN_PREFLIGHT_INFEASIBLE_LINEAR, 0U);
  expect_preflight("case6 S3 small fallback",
                   case6_small_s3,
                   sizeof(case6_small_s3) / sizeof(case6_small_s3[0]),
                   &shortestRunCaseParamsMode2[5], true,
                   false,
                   F413_PATH_RUN_PREFLIGHT_OK, 0U);
  expect_preflight("case7 S2 large",
                   case7_large_s2,
                   sizeof(case7_large_s2) / sizeof(case7_large_s2[0]),
                   &shortestRunCaseParamsMode2[6], true,
                   false,
                   F413_PATH_RUN_PREFLIGHT_OK, 0U);
  expect_preflight("coalesced orthogonal codes",
                   continuous_orthogonal,
                   sizeof(continuous_orthogonal) /
                       sizeof(continuous_orthogonal[0]),
                   &shortestRunCaseParamsMode2[5], true,
                   false,
                   F413_PATH_RUN_PREFLIGHT_OK, 0U);
  expect_preflight("coalesced diagonal codes",
                   continuous_diagonal,
                   sizeof(continuous_diagonal) /
                       sizeof(continuous_diagonal[0]),
                   &shortestRunCaseParamsMode2[5], true,
                   false,
                   F413_PATH_RUN_PREFLIGHT_OK, 0U);
  expect_preflight("terminal diagonal rejected",
                   terminal_diagonal,
                   sizeof(terminal_diagonal) / sizeof(terminal_diagonal[0]),
                   &shortestRunCaseParamsMode2[5], true,
                   false,
                   F413_PATH_RUN_PREFLIGHT_INVALID_LEGACY_PATH, 3U);
  expect_preflight("test mode still rejects invalid diagonal entry",
                   diagonal_without_entry,
                   sizeof(diagonal_without_entry) /
                       sizeof(diagonal_without_entry[0]),
                   &shortestRunCaseParamsMode2[7], false,
                   true,
                   F413_PATH_RUN_PREFLIGHT_INVALID_LEGACY_PATH, 1U);
  expect_preflight("leading turn speed discontinuity",
                   leading_turn,
                   sizeof(leading_turn) / sizeof(leading_turn[0]),
                   &shortestRunCaseParamsMode2[5], true,
                   false,
                   F413_PATH_RUN_PREFLIGHT_SPEED_DISCONTINUITY, 0U);
  expect_preflight("large-S1-small skip wall-end",
                   large_s1_small,
                   sizeof(large_s1_small) / sizeof(large_s1_small[0]),
                   &shortestRunCaseParamsMode2[6], true,
                   false,
                   F413_PATH_RUN_PREFLIGHT_OK, 0U);
  expect_preflight("case6 open-floor diagonal wall correction off",
                   case6_open_floor_diagonal,
                   sizeof(case6_open_floor_diagonal) /
                       sizeof(case6_open_floor_diagonal[0]),
                   &shortestRunCaseParamsMode2[5], false,
                   false,
                   F413_PATH_RUN_PREFLIGHT_OK, 0U);
}

static void check_mode2_case0_paths(void)
{
  static const uint16_t paths[10][6] = {
      {203U, 300U, 0U},
      {203U, 501U, 0U},
      {203U, 502U, 0U},
      {203U, 701U, 1001U, 0U},
      {203U, 701U, 1001U, 704U, 0U},
      {203U, 701U, 1001U, 802U, 1001U, 0U},
      {203U, 901U, 1001U, 0U},
      {203U, 901U, 1001U, 904U, 0U},
      {209U, 0U},
      {209U, 0U},
  };
  static const uint8_t case_indices[10] = {
      3U, 3U, 3U, 6U, 6U, 6U, 6U, 6U, 1U, 5U};

  for (size_t sub = 0U; sub < 10U; sub++)
  {
    char label[40];
    const uint8_t case_index = case_indices[sub];
    (void)snprintf(label, sizeof(label), "mode2 case0 sub%zu", sub);
    expect_preflight(label,
                     paths[sub],
                     sizeof(paths[sub]) / sizeof(paths[sub][0]),
                     &shortestRunCaseParamsMode2[case_index - 1U],
                     false,
                     true,
                     F413_PATH_RUN_PREFLIGHT_OK,
                     0U);
  }

  CHECK(close_value(
      f413_path_run_next_diagonal_exit_velocity(
          0U, &shortestRunModeParams2, &shortestRunCaseParamsMode2[7], 500.0f),
      500.0,
      1.0e-6));
  CHECK(close_value(
      f413_path_run_next_diagonal_exit_velocity(
          0U, &shortestRunModeParams2, &shortestRunCaseParamsMode2[7], 0.0f),
      0.0,
      1.0e-6));
}

static double prepared_distance_sum(
    const f413_path_run_prepared_linear_t* action)
{
  double sum = 0.0;
  for (size_t index = 0U; index < action->phase_count; index++)
  {
    sum += action->phase_distance_mm[index];
  }
  return sum;
}

static void check_prepared_execution_plans(void)
{
  static const uint16_t sub3[] = {203U, 701U, 1001U, 0U};
  static const uint16_t case6_small_s1[] = {201U, 300U, 201U, 0U};
  const ShortestRunCaseParams_t* case6 = &shortestRunCaseParamsMode2[5];
  f413_path_run_prepared_path_t prepared;
  float initial_velocity = sqrtf(
      2.0f * case6->acceleration_straight * (float)DIST_FIRST_SEC);
  f413_path_run_preflight_result_t result = f413_path_run_preflight_prepare(
      sub3, sizeof(sub3) / sizeof(sub3[0]), &shortestRunModeParams2,
      case6, initial_velocity, false, true, &prepared);

  CHECK(result.status == F413_PATH_RUN_PREFLIGHT_OK);
  CHECK(prepared.count == 2U);
  CHECK(prepared.actions[0].path_index == 0U);
  CHECK(prepared.actions[0].execute_plan);
  CHECK(prepared.actions[0].phase_count > 0U);
  CHECK(close_value(prepared.actions[0].entry_velocity_mm_s,
                    initial_velocity, 1.0e-6));
  CHECK(close_value(prepared.actions[0].exit_velocity_mm_s, 500.0, 1.0e-6));
  CHECK(close_value(prepared_distance_sum(&prepared.actions[0]),
                    3.0 * DIST_HALF_SEC, 1.0e-4));
  CHECK(prepared.actions[1].path_index == 2U);
  CHECK(prepared.actions[1].execute_plan);
  CHECK(prepared.actions[1].phase_count > 0U);
  CHECK(close_value(prepared.actions[1].entry_velocity_mm_s, 500.0, 1.0e-6));
  CHECK(close_value(prepared.actions[1].exit_velocity_mm_s, 500.0, 1.0e-6));
  CHECK(close_value(prepared_distance_sum(&prepared.actions[1]),
                    DIST_D_HALF_SEC, 1.0e-4));

  initial_velocity = sqrtf(
      2.0f * case6->acceleration_straight * (float)DIST_FIRST_SEC);
  result = f413_path_run_preflight_prepare(
      case6_small_s1,
      sizeof(case6_small_s1) / sizeof(case6_small_s1[0]),
      &shortestRunModeParams2, case6, initial_velocity, true, false,
      &prepared);
  CHECK(result.status == F413_PATH_RUN_PREFLIGHT_OK);
  CHECK(prepared.count == 2U);
  CHECK(prepared.actions[0].path_index == 0U);
  CHECK(!prepared.actions[0].execute_plan);
  CHECK(prepared.actions[0].phase_count == 0U);
  CHECK(close_value(prepared.actions[0].entry_velocity_mm_s,
                    initial_velocity, 1.0e-6));
  CHECK(close_value(prepared.actions[0].exit_velocity_mm_s, 300.0, 1.0e-6));
}

static void check_distance_cursor_transition_credit(void)
{
  f413_path_run_distance_cursor_t cursor;
  float target = 0.0f;
  float remaining = 0.0f;

  f413_path_run_distance_cursor_invalidate(&cursor);
  CHECK(!cursor.active);
  CHECK(!f413_path_run_distance_cursor_advance(
      &cursor, 0.0f, 5.0f, &target, &remaining));

  f413_path_run_distance_cursor_reset(&cursor, 0.0f);
  CHECK(cursor.active);
  CHECK(f413_path_run_distance_cursor_advance(
      &cursor, 0.0f, 5.0f, &target, &remaining));
  CHECK(close_value(target, 5.0, 1.0e-6));
  CHECK(close_value(remaining, 5.0, 1.0e-6));

  /* 0.4 mm traveled while switching profiles consumes the S3 action. */
  CHECK(f413_path_run_distance_cursor_advance(
      &cursor, 5.4f, 135.0f, &target, &remaining));
  CHECK(close_value(target, 140.0, 1.0e-6));
  CHECK(close_value(remaining, 134.6, 1.0e-5));

  /* A further 1.0 mm transition consumes the turn entry offset. */
  CHECK(f413_path_run_distance_cursor_advance(
      &cursor, 141.0f, 10.0f, &target, &remaining));
  CHECK(close_value(target, 150.0, 1.0e-6));
  CHECK(close_value(remaining, 9.0, 1.0e-6));

  /* The empirical timed turn core establishes a fresh measured origin. */
  f413_path_run_distance_cursor_reset(&cursor, 217.5f);
  CHECK(f413_path_run_distance_cursor_advance(
      &cursor, 219.0f, 29.0f, &target, &remaining));
  CHECK(close_value(target, 246.5, 1.0e-6));
  CHECK(close_value(remaining, 27.5, 1.0e-6));
  CHECK(f413_path_run_distance_cursor_advance(
      &cursor, 247.0f, (float)DIST_D_HALF_SEC, &target, &remaining));
  CHECK(close_value(target, 246.5 + DIST_D_HALF_SEC, 1.0e-5));
  CHECK(close_value(remaining, DIST_D_HALF_SEC - 0.5, 1.0e-5));
  CHECK(f413_path_run_distance_cursor_advance(
      &cursor, target + 2.0f, (float)DIST_HALF_SEC, &target, &remaining));
  CHECK(close_value(remaining, DIST_HALF_SEC - 2.0, 1.0e-5));

  /* Overshoot larger than one portion carries into the following portion. */
  f413_path_run_distance_cursor_reset(&cursor, 0.0f);
  CHECK(f413_path_run_distance_cursor_advance(
      &cursor, 0.0f, 5.0f, &target, &remaining));
  CHECK(f413_path_run_distance_cursor_advance(
      &cursor, 8.0f, 2.0f, &target, &remaining));
  CHECK(close_value(target, 7.0, 1.0e-6));
  CHECK(close_value(remaining, 0.0, 1.0e-6));
  CHECK(f413_path_run_distance_cursor_advance(
      &cursor, 8.0f, 10.0f, &target, &remaining));
  CHECK(close_value(target, 17.0, 1.0e-6));
  CHECK(close_value(remaining, 9.0, 1.0e-6));
}

int main(void)
{
  check_mode2_case_profiles();
  check_turn_offset_trace_contract();
  check_start_runup_contract();
  check_wall_end_approach_contract();
  check_preflight_boundaries();
  check_mode2_case0_paths();
  check_prepared_execution_plans();
  check_distance_cursor_transition_credit();

  if (g_failures != 0U)
  {
    fprintf(stderr, "f413 path linear tests: %u/%u failed\n",
            g_failures, g_checks);
    return 1;
  }
  printf("f413 path linear tests: %u checks passed\n", g_checks);
  return 0;
}
