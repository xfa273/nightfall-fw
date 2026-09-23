#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* Reuse the generator's calculation as the independent runtime reference. */
#define main f413_route_motion_generator_program_main
#include "generate_f413_route_motion.c"
#undef main

#include "f413_route_motion_table.h"

static unsigned int g_checks;
static unsigned int g_failures;
static double g_max_absolute_error;

static void check_bool(bool condition, const char* label)
{
  g_checks++;
  if (!condition)
  {
    g_failures++;
    fprintf(stderr, "FAIL: %s\n", label);
  }
}

static void check_double(double actual, double expected, const char* label)
{
  const double error = fabs(actual - expected);
  const double limit = 1.0e-13 * fmax(1.0, fabs(expected));
  g_checks++;
  if (error > g_max_absolute_error)
  {
    g_max_absolute_error = error;
  }
  if (!isfinite(actual) || !isfinite(expected) || (error > limit))
  {
    g_failures++;
    fprintf(stderr,
            "FAIL: %s actual=%.17g expected=%.17g error=%.17g\n",
            label, actual, expected, error);
  }
}

static void check_spec(const NfTurnSpec* actual,
                       const NfTurnSpec* expected,
                       const char* label)
{
  char field[128];
  snprintf(field, sizeof(field), "%s.enabled", label);
  check_bool(actual->enabled == expected->enabled, field);
#define CHECK_SPEC_FIELD(name)                                                 \
  do                                                                           \
  {                                                                            \
    snprintf(field, sizeof(field), "%s.%s", label, #name);                    \
    check_double(actual->name, expected->name, field);                          \
  } while (0)
  CHECK_SPEC_FIELD(velocity_mm_s);
  CHECK_SPEC_FIELD(alpha_deg_s2);
  CHECK_SPEC_FIELD(angle_deg);
  CHECK_SPEC_FIELD(dist_in_mm);
  CHECK_SPEC_FIELD(dist_out_mm);
#undef CHECK_SPEC_FIELD
}

static void check_plan(const NfTurnPlan* actual,
                       const NfTurnPlan* expected,
                       const char* label)
{
  char field[128];
#define CHECK_PLAN_FIELD(name)                                                 \
  do                                                                           \
  {                                                                            \
    snprintf(field, sizeof(field), "%s.%s", label, #name);                    \
    check_double(actual->name, expected->name, field);                          \
  } while (0)
  CHECK_PLAN_FIELD(omega_peak_deg_s);
  CHECK_PLAN_FIELD(accel_time_s);
  CHECK_PLAN_FIELD(cruise_time_s);
  CHECK_PLAN_FIELD(angular_time_s);
  CHECK_PLAN_FIELD(total_time_s);
  CHECK_PLAN_FIELD(travel_distance_mm);
  CHECK_PLAN_FIELD(displacement_forward_mm);
  CHECK_PLAN_FIELD(displacement_lateral_mm);
#undef CHECK_PLAN_FIELD
}

static void check_limits(const NfLinearLimits* actual,
                         const NfLinearLimits* expected,
                         const char* label)
{
  char field[128];
#define CHECK_LIMIT_FIELD(name)                                                \
  do                                                                           \
  {                                                                            \
    snprintf(field, sizeof(field), "%s.%s", label, #name);                    \
    check_double(actual->name, expected->name, field);                          \
  } while (0)
  CHECK_LIMIT_FIELD(vmax_mm_s);
  CHECK_LIMIT_FIELD(switch_velocity_mm_s);
  CHECK_LIMIT_FIELD(accel_low_mm_s2);
  CHECK_LIMIT_FIELD(accel_high_mm_s2);
#undef CHECK_LIMIT_FIELD
}

int main(void)
{
  Geometry expected_geometry[KIND_COUNT] = {0};
  MotionCase expected_cases[CASE_COUNT] = {0};
  size_t pose_bytes = 0U;

  calculate(expected_geometry, expected_cases);
  check_bool(F413_ROUTE_PRECOMPUTED_KIND_COUNT == KIND_COUNT,
             "generated kind count");
  check_bool(F413_ROUTE_PRECOMPUTED_SPEED_COUNT == SPEED_COUNT,
             "generated speed count");
  check_bool(F413_ROUTE_PRECOMPUTED_CASE_COUNT == CASE_COUNT,
             "generated case count");
  check_bool(F413_ROUTE_PRECOMPUTED_CONNECTOR_STEP_COUNT ==
                 CONNECTOR_STEP_COUNT,
             "generated connector step count");
  check_bool(f413_route_precomputed_find_case(CASE_FIRST - 1U) == NULL,
             "lookup rejects low case");
  check_bool(f413_route_precomputed_find_case(CASE_LAST + 1U) == NULL,
             "lookup rejects high case");

  for (size_t kind = 0U; kind < KIND_COUNT; kind++)
  {
    const F413RoutePrecomputedGeometry* actual =
        &g_f413_route_precomputed_geometry[kind];
    const NfTurnPose* actual_poses =
        &g_f413_route_precomputed_poses[actual->pose_offset];
    char label[128];
    snprintf(label, sizeof(label), "geometry[%zu].spec", kind);
    check_spec(&actual->spec, &expected_geometry[kind].spec, label);
    snprintf(label, sizeof(label), "geometry[%zu].plan", kind);
    check_plan(&actual->plan, &expected_geometry[kind].plan, label);
    snprintf(label, sizeof(label), "geometry[%zu].intervals", kind);
    check_bool(actual->intervals == expected_geometry[kind].intervals, label);
    pose_bytes += ((size_t)actual->intervals + 1U) * sizeof(NfTurnPose);
    for (uint16_t pose = 0U; pose <= actual->intervals; pose++)
    {
      snprintf(label, sizeof(label), "geometry[%zu].pose[%u].forward",
               kind, (unsigned int)pose);
      check_double(actual_poses[pose].forward_mm,
                   expected_geometry[kind].poses[pose].forward_mm, label);
      snprintf(label, sizeof(label), "geometry[%zu].pose[%u].lateral",
               kind, (unsigned int)pose);
      check_double(actual_poses[pose].lateral_mm,
                   expected_geometry[kind].poses[pose].lateral_mm, label);
      snprintf(label, sizeof(label), "geometry[%zu].pose[%u].heading",
               kind, (unsigned int)pose);
      check_double(actual_poses[pose].heading_deg,
                   expected_geometry[kind].poses[pose].heading_deg, label);
    }
  }

  for (size_t case_offset = 0U; case_offset < CASE_COUNT; case_offset++)
  {
    const uint8_t case_index = (uint8_t)(CASE_FIRST + case_offset);
    const F413RoutePrecomputedCase* actual =
        f413_route_precomputed_find_case(case_index);
    const MotionCase* expected = &expected_cases[case_offset];
    char label[128];
    check_bool(actual == &g_f413_route_precomputed_cases[case_offset],
               "case lookup identity");
    if (actual == NULL)
    {
      continue;
    }
    snprintf(label, sizeof(label), "case[%u].index",
             (unsigned int)case_index);
    check_bool(actual->case_index == case_index, label);
    snprintf(label, sizeof(label), "case[%u].orthogonal",
             (unsigned int)case_index);
    check_limits(&actual->orthogonal, &expected->orthogonal, label);
    snprintf(label, sizeof(label), "case[%u].diagonal",
             (unsigned int)case_index);
    check_limits(&actual->diagonal, &expected->diagonal, label);
    snprintf(label, sizeof(label), "case[%u].start_time_us",
             (unsigned int)case_index);
    check_bool(actual->start_time_us == expected->start_time_us, label);
    for (size_t speed = 0U; speed < SPEED_COUNT; speed++)
    {
      snprintf(label, sizeof(label), "case[%u].speed[%zu]",
               (unsigned int)case_index, speed);
      check_double(actual->speed_mm_s[speed], expected->speed_mm_s[speed],
                   label);
      for (size_t kind = 0U; kind < KIND_COUNT; kind++)
      {
        snprintf(label, sizeof(label), "case[%u].timing[%zu][%zu]",
                 (unsigned int)case_index, speed, kind);
        check_spec(&actual->timing[speed][kind],
                   &expected->timing[speed][kind], label);
        snprintf(label, sizeof(label), "case[%u].plan[%zu][%zu]",
                 (unsigned int)case_index, speed, kind);
        check_plan(&actual->timing_plan[speed][kind],
                   &expected->timing_plan[speed][kind], label);
        snprintf(label, sizeof(label), "case[%u].turn-time[%zu][%zu]",
                 (unsigned int)case_index, speed, kind);
        check_bool(actual->turn_time_us[speed][kind] ==
                       expected->turn_time_us[speed][kind],
                   label);
      }
    }
    for (size_t heading = 0U; heading < HEADING_CLASS_COUNT; heading++)
    {
      for (size_t entry = 0U; entry < SPEED_COUNT; entry++)
      {
        for (size_t exit_speed = 0U;
             exit_speed < EXIT_SPEED_COUNT; exit_speed++)
        {
          for (size_t step = 0U; step < CONNECTOR_STEP_COUNT; step++)
          {
            snprintf(label, sizeof(label),
                     "case[%u].connector[%zu][%zu][%zu][%zu]",
                     (unsigned int)case_index, heading, entry, exit_speed,
                     step);
            check_bool(
                actual->connector_time_us[heading][entry][exit_speed][step] ==
                    expected->connector_time_us[heading][entry][exit_speed]
                                                       [step],
                label);
          }
        }
      }
    }
    for (size_t entry = 0U; entry < SPEED_COUNT; entry++)
    {
      for (size_t exit_speed = 0U; exit_speed < SPEED_COUNT; exit_speed++)
      {
        for (size_t step = 0U; step < CONNECTOR_STEP_COUNT; step++)
        {
          snprintf(label, sizeof(label),
                   "case[%u].approach[%zu][%zu][%zu]",
                   (unsigned int)case_index, entry, exit_speed, step);
          check_bool(
              actual->orthogonal_approach_time_us[entry][exit_speed][step] ==
                  expected->orthogonal_approach_time_us[entry][exit_speed]
                                                       [step],
              label);
        }
      }
    }
  }

  {
    const F413RoutePrecomputedCase* case6 =
        f413_route_precomputed_find_case(6U);
    const F413RoutePrecomputedCase* case7 =
        f413_route_precomputed_find_case(7U);
    const F413RoutePrecomputedCase* case8 =
        f413_route_precomputed_find_case(8U);
    const uint32_t case6_crawl_low_time_us =
        (case6 == NULL) ? UINT32_MAX : seconds_to_u32_us(
            (2.0 * (double)DIST_HALF_SEC) /
            (case6->speed_mm_s[SPEED_CRAWL] +
             case6->speed_mm_s[SPEED_LOW]));
    check_bool(case6 != NULL &&
                   case6->orthogonal_approach_time_us[SPEED_CRAWL]
                       [SPEED_LOW][1U] != UINT32_MAX,
               "case6 exact one-step crawl-to-small approach is feasible");
    check_bool(case6 != NULL &&
                   case6->orthogonal_approach_time_us[SPEED_CRAWL]
                       [SPEED_LOW][1U] == case6_crawl_low_time_us,
               "case6 exact one-step crawl-to-small duration");
    check_bool(case6 != NULL &&
                   case6->orthogonal_approach_time_us[SPEED_LOW]
                       [SPEED_CRAWL][1U] == case6_crawl_low_time_us,
               "case6 exact one-step small-to-crawl duration");
    check_bool(case6 != NULL &&
                   case6->orthogonal_approach_time_us[SPEED_CRAWL]
                       [SPEED_NOMINAL][1U] == UINT32_MAX,
               "case6 exact one-step crawl-to-large exceeds acceleration");
    check_bool(case7 != NULL &&
                   case7->orthogonal_approach_time_us[SPEED_CRAWL]
                       [SPEED_NOMINAL][1U] != UINT32_MAX,
               "case7 exact one-step crawl-to-large approach is feasible");
    check_bool(case8 != NULL &&
                   case8->orthogonal_approach_time_us[SPEED_CRAWL]
                       [SPEED_NOMINAL][1U] != UINT32_MAX,
               "case8 exact one-step crawl-to-large approach is feasible");
  }

  printf("f413 route precompute: %u checks, %u failures, "
         "max-double-error=%.3g, table-data=%zu bytes\n",
         g_checks, g_failures, g_max_absolute_error,
         sizeof(g_f413_route_precomputed_cases) +
             sizeof(g_f413_route_precomputed_geometry) + pose_bytes);
  return (g_failures == 0U) ? EXIT_SUCCESS : EXIT_FAILURE;
}
