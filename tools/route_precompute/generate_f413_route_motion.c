#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion_time.h"
#include "f413_path_run.h"
#include "params.h"
#include "shortest_run_params.h"

#define CASE_FIRST 6U
#define CASE_LAST 9U
#define CASE_COUNT (CASE_LAST - CASE_FIRST + 1U)
#define SPEED_COUNT 3U
#define KIND_COUNT 8U
#define HEADING_CLASS_COUNT 2U
#define EXIT_SPEED_COUNT 4U
#define CONNECTOR_STEP_COUNT 69U
#define MAX_INTERVALS 384U

enum
{
  SPEED_NOMINAL = 0,
  SPEED_LOW,
  SPEED_CRAWL,
};

enum
{
  HEADING_ORTHOGONAL = 0,
  HEADING_DIAGONAL,
};

enum
{
  EXIT_NOMINAL = 0,
  EXIT_LOW,
  EXIT_CRAWL,
  EXIT_STOP,
};

enum
{
  KIND_LARGE_90 = 0,
  KIND_LARGE_180,
  KIND_45_IN,
  KIND_45_OUT,
  KIND_V90,
  KIND_135_IN,
  KIND_135_OUT,
  KIND_SMALL_90,
};

typedef struct
{
  NfTurnSpec spec;
  NfTurnPlan plan;
  NfTurnPose poses[MAX_INTERVALS + 1U];
  uint16_t intervals;
  double expected_forward_mm;
  double expected_lateral_mm;
  double expected_heading_deg;
} Geometry;

typedef struct
{
  NfTurnSpec timing[SPEED_COUNT][KIND_COUNT];
  NfTurnPlan timing_plan[SPEED_COUNT][KIND_COUNT];
  uint32_t turn_time_us[SPEED_COUNT][KIND_COUNT];
  NfLinearLimits orthogonal;
  NfLinearLimits diagonal;
  double speed_mm_s[SPEED_COUNT];
  uint32_t start_time_us;
  uint32_t connector_time_us[HEADING_CLASS_COUNT][SPEED_COUNT]
                            [EXIT_SPEED_COUNT][CONNECTOR_STEP_COUNT];
  uint32_t orthogonal_approach_time_us[SPEED_COUNT][SPEED_COUNT]
                                      [CONNECTOR_STEP_COUNT];
} MotionCase;

static void fail(const char* message)
{
  fprintf(stderr, "generate_f413_route_motion: %s\n", message);
  exit(EXIT_FAILURE);
}

static void check_write(FILE* stream)
{
  if (ferror(stream))
  {
    fail("output write failed");
  }
}

static NfTurnSpec scaled_turn(const NfTurnSpec* source, double scale)
{
  NfTurnSpec result = *source;
  result.velocity_mm_s *= scale;
  result.alpha_deg_s2 *= scale * scale;
  return result;
}

static double capped_positive(double candidate, double fallback, double cap)
{
  double value = candidate;
  if (!isfinite(value) || (value <= 0.0))
  {
    value = fallback;
  }
  return (value > cap) ? cap : value;
}

static uint32_t seconds_to_u32_us(double seconds)
{
  uint64_t value = 0U;
  if ((nf_motion_seconds_to_us(seconds, &value) != NF_MOTION_OK) ||
      (value > UINT32_MAX))
  {
    fail("duration cannot be represented as uint32 microseconds");
  }
  return (uint32_t)value;
}

static void make_specs(NfTurnSpec timing[KIND_COUNT],
                       NfTurnSpec geometry[KIND_COUNT],
                       double expected_forward[KIND_COUNT],
                       double expected_lateral[KIND_COUNT],
                       double expected_heading[KIND_COUNT])
{
  const ShortestRunModeParams_t* mode = &shortestRunModeParams2;
  const double half = (double)DIST_HALF_SEC;
  const double diagonal_half = half / sqrt(2.0);

  timing[KIND_LARGE_90] = (NfTurnSpec){
      true, mode->velocity_l_turn_90, mode->alpha_l_turn_90, 90.0,
      mode->dist_l_turn_in_90, mode->dist_l_turn_out_90};
  timing[KIND_LARGE_180] = (NfTurnSpec){
      true, mode->velocity_l_turn_180, mode->alpha_l_turn_180, 180.0,
      mode->dist_l_turn_in_180, mode->dist_l_turn_out_180};
  timing[KIND_45_IN] = (NfTurnSpec){
      true, mode->velocity_turn45in, mode->alpha_turn45in, 45.0,
      mode->dist_turn45in_in, mode->dist_turn45in_out};
  timing[KIND_45_OUT] = (NfTurnSpec){
      true, mode->velocity_turn45out, mode->alpha_turn45out, 45.0,
      mode->dist_turn45out_in, mode->dist_turn45out_out};
  timing[KIND_V90] = (NfTurnSpec){
      true, mode->velocity_turnV90, mode->alpha_turnV90, 90.0,
      mode->dist_turnV90_in, mode->dist_turnV90_out};
  timing[KIND_135_IN] = (NfTurnSpec){
      true, mode->velocity_turn135in, mode->alpha_turn135in, 135.0,
      mode->dist_turn135in_in, mode->dist_turn135in_out};
  timing[KIND_135_OUT] = (NfTurnSpec){
      true, mode->velocity_turn135out, mode->alpha_turn135out, 135.0,
      mode->dist_turn135out_in, mode->dist_turn135out_out};
  timing[KIND_SMALL_90] = (NfTurnSpec){
      true, mode->velocity_turn90, mode->alpha_turn90, 90.0,
      mode->dist_offset_in, mode->dist_offset_out};

  for (size_t kind = 0U; kind < KIND_COUNT; kind++)
  {
    timing[kind].velocity_mm_s = capped_positive(
        timing[kind].velocity_mm_s, NIGHTFALL_F413_PATH_VELOCITY,
        NIGHTFALL_F413_PATH_TURN_VELOCITY_CAP);
  }

  geometry[KIND_LARGE_90] = (NfTurnSpec){
      true, timing[KIND_LARGE_90].velocity_mm_s, 4700.0, 90.0,
      0.352912418, 0.352912418};
  geometry[KIND_LARGE_180] = (NfTurnSpec){
      true, timing[KIND_LARGE_180].velocity_mm_s, 4422.213141, 180.0,
      15.500, 15.500};
  geometry[KIND_45_IN] = (NfTurnSpec){
      true, timing[KIND_45_IN].velocity_mm_s, 7234.4, 45.0, 0.0, 18.640};
  geometry[KIND_45_OUT] = (NfTurnSpec){
      true, timing[KIND_45_OUT].velocity_mm_s, 7234.4, 45.0, 18.640, 0.0};
  geometry[KIND_V90] = (NfTurnSpec){
      true, timing[KIND_V90].velocity_mm_s, 12200.0, 90.0, 7.997, 7.997};
  geometry[KIND_135_IN] = (NfTurnSpec){
      true, timing[KIND_135_IN].velocity_mm_s, 8500.0, 135.0, 18.932, 11.212};
  geometry[KIND_135_OUT] = (NfTurnSpec){
      true, timing[KIND_135_OUT].velocity_mm_s, 8500.0, 135.0, 11.212, 18.932};
  geometry[KIND_SMALL_90] = (NfTurnSpec){
      true, 300.0, 8920.0, 90.0, 5.956032993, 5.956032993};

  expected_forward[KIND_LARGE_90] = 2.0 * half;
  expected_lateral[KIND_LARGE_90] = 2.0 * half;
  expected_heading[KIND_LARGE_90] = 90.0;
  expected_forward[KIND_LARGE_180] = 0.0;
  expected_lateral[KIND_LARGE_180] = 2.0 * half;
  expected_heading[KIND_LARGE_180] = 180.0;
  expected_forward[KIND_45_IN] = 2.0 * half;
  expected_lateral[KIND_45_IN] = half;
  expected_heading[KIND_45_IN] = 45.0;
  expected_forward[KIND_45_OUT] = 3.0 * diagonal_half;
  expected_lateral[KIND_45_OUT] = diagonal_half;
  expected_heading[KIND_45_OUT] = 45.0;
  expected_forward[KIND_V90] = half * sqrt(2.0);
  expected_lateral[KIND_V90] = half * sqrt(2.0);
  expected_heading[KIND_V90] = 90.0;
  expected_forward[KIND_135_IN] = half;
  expected_lateral[KIND_135_IN] = 2.0 * half;
  expected_heading[KIND_135_IN] = 135.0;
  expected_forward[KIND_135_OUT] = diagonal_half;
  expected_lateral[KIND_135_OUT] = 3.0 * diagonal_half;
  expected_heading[KIND_135_OUT] = 135.0;
  expected_forward[KIND_SMALL_90] = half;
  expected_lateral[KIND_SMALL_90] = half;
  expected_heading[KIND_SMALL_90] = 90.0;
}

static void calculate(Geometry geometry[KIND_COUNT],
                      MotionCase cases[CASE_COUNT])
{
  NfTurnSpec nominal_timing[KIND_COUNT];
  NfTurnSpec geometry_specs[KIND_COUNT];
  double expected_forward[KIND_COUNT];
  double expected_lateral[KIND_COUNT];
  double expected_heading[KIND_COUNT];
  const NfTurnEnvironment geometry_environment = {
      NIGHTFALL_F413_PATH_OMEGA_CAP, 1.2};

  make_specs(nominal_timing, geometry_specs, expected_forward,
             expected_lateral, expected_heading);

  for (size_t kind = 0U; kind < KIND_COUNT; kind++)
  {
    double residual;
    double required_intervals;
    uint16_t intervals;
    geometry[kind].spec = geometry_specs[kind];
    if (nf_motion_turn_plan(&geometry[kind].spec, &geometry_environment,
                            &geometry[kind].plan) != NF_MOTION_OK)
    {
      fail("geometry turn planning failed");
    }
    residual = hypot(geometry[kind].plan.displacement_forward_mm -
                         expected_forward[kind],
                     geometry[kind].plan.displacement_lateral_mm -
                         expected_lateral[kind]);
    if (!isfinite(residual) || (residual > 0.00100001))
    {
      fprintf(stderr, "geometry kind %zu closure residual %.12g mm\n",
              kind, residual);
      fail("geometry closure validation failed");
    }
    required_intervals = fmax(geometry[kind].plan.travel_distance_mm,
                              expected_heading[kind] / 2.0);
    intervals = (uint16_t)ceil(required_intervals);
    if (intervals < 2U)
    {
      intervals = 2U;
    }
    if (intervals > MAX_INTERVALS)
    {
      fail("geometry sample count exceeds table format");
    }
    if (nf_motion_turn_pose_uniform(&geometry[kind].spec,
                                    &geometry[kind].plan, intervals,
                                    geometry[kind].poses,
                                    MAX_INTERVALS + 1U) != NF_MOTION_OK)
    {
      fail("geometry pose sampling failed");
    }
    geometry[kind].poses[intervals].forward_mm = expected_forward[kind];
    geometry[kind].poses[intervals].lateral_mm = expected_lateral[kind];
    geometry[kind].poses[intervals].heading_deg = expected_heading[kind];
    geometry[kind].intervals = intervals;
    geometry[kind].expected_forward_mm = expected_forward[kind];
    geometry[kind].expected_lateral_mm = expected_lateral[kind];
    geometry[kind].expected_heading_deg = expected_heading[kind];
  }

  for (uint8_t case_index = CASE_FIRST; case_index <= CASE_LAST; case_index++)
  {
    const ShortestRunModeParams_t* mode = &shortestRunModeParams2;
    const ShortestRunCaseParams_t* run_case =
        &shortestRunCaseParamsMode2[case_index - 1U];
    MotionCase* output = &cases[case_index - CASE_FIRST];
    NfLinearPlan start_plan;
    double crawl_velocity;

    memset(output, 0, sizeof(*output));
    output->orthogonal = (NfLinearLimits){
        capped_positive(run_case->velocity_straight,
                        NIGHTFALL_F413_PATH_VELOCITY,
                        NIGHTFALL_F413_PATH_VELOCITY_CAP),
        mode->accel_switch_velocity,
        run_case->acceleration_straight,
        run_case->acceleration_straight_dash};
    output->diagonal = (NfLinearLimits){
        capped_positive(run_case->velocity_d_straight,
                        output->orthogonal.vmax_mm_s,
                        NIGHTFALL_F413_PATH_DIAGONAL_VELOCITY_CAP),
        0.0,
        run_case->acceleration_d_straight,
        run_case->acceleration_d_straight_dash};
    if ((nf_motion_accelerating_exit_velocity(
             &output->orthogonal, (double)DIST_FIRST_SEC, 0.0,
             &crawl_velocity) != NF_MOTION_OK) ||
        (nf_motion_linear_plan(&output->orthogonal,
                               (double)DIST_FIRST_SEC, 0.0,
                               crawl_velocity, &start_plan) != NF_MOTION_OK))
    {
      fail("start-offset linear planning failed");
    }
    output->start_time_us = seconds_to_u32_us(start_plan.total_time_s);
    output->speed_mm_s[SPEED_NOMINAL] =
        nominal_timing[KIND_LARGE_90].velocity_mm_s;
    output->speed_mm_s[SPEED_LOW] = mode->velocity_turn90;
    output->speed_mm_s[SPEED_CRAWL] = crawl_velocity;
    if ((output->speed_mm_s[SPEED_NOMINAL] <=
         output->speed_mm_s[SPEED_LOW]) ||
        (output->speed_mm_s[SPEED_LOW] <=
         output->speed_mm_s[SPEED_CRAWL]))
    {
      fail("turn speed ordering is invalid");
    }
    for (size_t heading_class = 0U;
         heading_class < HEADING_CLASS_COUNT; heading_class++)
    {
      const NfLinearLimits* limits = (heading_class == HEADING_ORTHOGONAL) ?
          &output->orthogonal : &output->diagonal;
      const double unit_mm = (heading_class == HEADING_ORTHOGONAL) ?
          (double)DIST_HALF_SEC : (double)DIST_D_HALF_SEC;
      for (size_t entry = 0U; entry < SPEED_COUNT; entry++)
      {
        for (size_t exit_speed = 0U;
             exit_speed < EXIT_SPEED_COUNT; exit_speed++)
        {
          const double exit_velocity = (exit_speed == EXIT_STOP) ? 0.0 :
              output->speed_mm_s[exit_speed];
          for (size_t step = 0U; step < CONNECTOR_STEP_COUNT; step++)
          {
            NfLinearPlan connector;
            uint64_t duration_us = 0U;
            uint32_t value = UINT32_MAX;
            if ((nf_motion_linear_plan(
                     limits, step * unit_mm, output->speed_mm_s[entry],
                     exit_velocity, &connector) == NF_MOTION_OK) &&
                (nf_motion_seconds_to_us(connector.total_time_s,
                                         &duration_us) == NF_MOTION_OK) &&
                (duration_us < UINT32_MAX))
            {
              value = (uint32_t)duration_us;
            }
            output->connector_time_us[heading_class][entry][exit_speed][step] =
                value;
          }
        }
      }
    }
    for (size_t entry = 0U; entry < SPEED_COUNT; entry++)
    {
      for (size_t exit_speed = 0U; exit_speed < SPEED_COUNT; exit_speed++)
      {
        const double exit_velocity = output->speed_mm_s[exit_speed];
        for (size_t step = 0U; step < CONNECTOR_STEP_COUNT; step++)
        {
          uint32_t value = UINT32_MAX;
          if (step == 0U)
          {
            if (output->speed_mm_s[entry] == exit_velocity)
            {
              value = 0U;
            }
          }
          else if (step == 1U)
          {
            NfConstantAccelProfile approach;
            uint64_t duration_us = 0U;
            if ((nf_motion_constant_accel_profile(
                     &output->orthogonal, (double)DIST_HALF_SEC,
                     output->speed_mm_s[entry], exit_velocity,
                     &approach) == NF_MOTION_OK) &&
                (nf_motion_seconds_to_us(
                     approach.duration_s, &duration_us) == NF_MOTION_OK) &&
                (duration_us < UINT32_MAX))
            {
              value = (uint32_t)duration_us;
            }
          }
          else
          {
            NfLinearPlan runup;
            uint64_t duration_us = 0U;
            if ((nf_motion_linear_plan(
                     &output->orthogonal,
                     (step - 1U) * (double)DIST_HALF_SEC,
                     output->speed_mm_s[entry], exit_velocity,
                     &runup) == NF_MOTION_OK) &&
                (nf_motion_seconds_to_us(
                     runup.total_time_s +
                         ((double)DIST_HALF_SEC / exit_velocity),
                     &duration_us) == NF_MOTION_OK) &&
                (duration_us < UINT32_MAX))
            {
              value = (uint32_t)duration_us;
            }
          }
          output->orthogonal_approach_time_us[entry][exit_speed][step] =
              value;
        }
      }
    }
    for (size_t speed = 0U; speed < SPEED_COUNT; speed++)
    {
      for (size_t kind = 0U; kind < KIND_COUNT; kind++)
      {
        NfTurnEnvironment environment = {
            NIGHTFALL_F413_PATH_OMEGA_CAP, 1.2};
        const double scale = output->speed_mm_s[speed] /
            nominal_timing[kind].velocity_mm_s;
        output->timing[speed][kind] =
            scaled_turn(&nominal_timing[kind], scale);
        environment.omega_cap_deg_s *= scale;
        if (nf_motion_turn_plan(&output->timing[speed][kind], &environment,
                                &output->timing_plan[speed][kind]) !=
            NF_MOTION_OK)
        {
          fail("timing turn planning failed");
        }
        output->turn_time_us[speed][kind] = seconds_to_u32_us(
            output->timing_plan[speed][kind].total_time_s);
      }
    }
  }
}

static void print_double(FILE* stream, double value)
{
  if (!isfinite(value))
  {
    fail("non-finite generated value");
  }
  fprintf(stream, "%a", value);
}

static void print_turn_spec(FILE* stream, const NfTurnSpec* spec)
{
  fprintf(stream, "{ %s, ", spec->enabled ? "true" : "false");
  print_double(stream, spec->velocity_mm_s);
  fprintf(stream, ", ");
  print_double(stream, spec->alpha_deg_s2);
  fprintf(stream, ", ");
  print_double(stream, spec->angle_deg);
  fprintf(stream, ", ");
  print_double(stream, spec->dist_in_mm);
  fprintf(stream, ", ");
  print_double(stream, spec->dist_out_mm);
  fprintf(stream, " }");
}

static void print_turn_plan(FILE* stream, const NfTurnPlan* plan)
{
  fprintf(stream, "{ ");
  print_double(stream, plan->omega_peak_deg_s);
  fprintf(stream, ", ");
  print_double(stream, plan->accel_time_s);
  fprintf(stream, ", ");
  print_double(stream, plan->cruise_time_s);
  fprintf(stream, ", ");
  print_double(stream, plan->angular_time_s);
  fprintf(stream, ", ");
  print_double(stream, plan->total_time_s);
  fprintf(stream, ", ");
  print_double(stream, plan->travel_distance_mm);
  fprintf(stream, ", ");
  print_double(stream, plan->displacement_forward_mm);
  fprintf(stream, ", ");
  print_double(stream, plan->displacement_lateral_mm);
  fprintf(stream, " }");
}

static void print_linear_limits(FILE* stream, const NfLinearLimits* limits)
{
  fprintf(stream, "{ ");
  print_double(stream, limits->vmax_mm_s);
  fprintf(stream, ", ");
  print_double(stream, limits->switch_velocity_mm_s);
  fprintf(stream, ", ");
  print_double(stream, limits->accel_low_mm_s2);
  fprintf(stream, ", ");
  print_double(stream, limits->accel_high_mm_s2);
  fprintf(stream, " }");
}

static size_t geometry_pose_count(const Geometry geometry[KIND_COUNT])
{
  size_t count = 0U;
  for (size_t kind = 0U; kind < KIND_COUNT; kind++)
  {
    count += (size_t)geometry[kind].intervals + 1U;
  }
  return count;
}

static void write_header(FILE* stream,
                         const char* input_sha,
                         const Geometry geometry[KIND_COUNT])
{
  fprintf(stream,
      "/* Generated by tools/route_precompute/generate.py; DO NOT EDIT. */\n"
      "#ifndef NIGHTFALL_F413_ROUTE_MOTION_TABLE_H\n"
      "#define NIGHTFALL_F413_ROUTE_MOTION_TABLE_H\n\n"
      "#include <stddef.h>\n"
      "#include <stdint.h>\n\n"
      "#include \"motion_time.h\"\n\n"
      "#ifdef __cplusplus\nextern \"C\" {\n#endif\n\n"
      "#define F413_ROUTE_PRECOMPUTED_SCHEMA_VERSION (1U)\n"
      "#define F413_ROUTE_PRECOMPUTED_CASE_FIRST (6U)\n"
      "#define F413_ROUTE_PRECOMPUTED_CASE_LAST (9U)\n"
      "#define F413_ROUTE_PRECOMPUTED_CASE_COUNT (4U)\n"
      "#define F413_ROUTE_PRECOMPUTED_SPEED_COUNT (3U)\n"
      "#define F413_ROUTE_PRECOMPUTED_KIND_COUNT (8U)\n"
      "#define F413_ROUTE_PRECOMPUTED_HEADING_CLASS_COUNT (2U)\n"
      "#define F413_ROUTE_PRECOMPUTED_EXIT_SPEED_COUNT (4U)\n"
      "#define F413_ROUTE_PRECOMPUTED_MAX_CONNECTOR_STEPS (68U)\n"
      "#define F413_ROUTE_PRECOMPUTED_CONNECTOR_STEP_COUNT (69U)\n"
      "#define F413_ROUTE_PRECOMPUTED_INPUT_SHA256 \"%s\"\n"
      "#define F413_ROUTE_PRECOMPUTED_POSE_COUNT (%zuU)\n\n"
      "typedef enum\n"
      "{\n"
      "  F413_ROUTE_PRECOMPUTED_SPEED_NOMINAL = 0,\n"
      "  F413_ROUTE_PRECOMPUTED_SPEED_LOW,\n"
      "  F413_ROUTE_PRECOMPUTED_SPEED_CRAWL,\n"
      "} F413RoutePrecomputedSpeed;\n\n"
      "typedef enum\n"
      "{\n"
      "  F413_ROUTE_PRECOMPUTED_HEADING_ORTHOGONAL = 0,\n"
      "  F413_ROUTE_PRECOMPUTED_HEADING_DIAGONAL,\n"
      "} F413RoutePrecomputedHeadingClass;\n\n"
      "typedef enum\n"
      "{\n"
      "  F413_ROUTE_PRECOMPUTED_EXIT_NOMINAL = 0,\n"
      "  F413_ROUTE_PRECOMPUTED_EXIT_LOW,\n"
      "  F413_ROUTE_PRECOMPUTED_EXIT_CRAWL,\n"
      "  F413_ROUTE_PRECOMPUTED_EXIT_STOP,\n"
      "} F413RoutePrecomputedExitSpeed;\n\n"
      "typedef enum\n"
      "{\n"
      "  F413_ROUTE_PRECOMPUTED_LARGE_90 = 0,\n"
      "  F413_ROUTE_PRECOMPUTED_LARGE_180,\n"
      "  F413_ROUTE_PRECOMPUTED_45_IN,\n"
      "  F413_ROUTE_PRECOMPUTED_45_OUT,\n"
      "  F413_ROUTE_PRECOMPUTED_V90,\n"
      "  F413_ROUTE_PRECOMPUTED_135_IN,\n"
      "  F413_ROUTE_PRECOMPUTED_135_OUT,\n"
      "  F413_ROUTE_PRECOMPUTED_SMALL_90,\n"
      "} F413RoutePrecomputedKind;\n\n"
      "typedef struct\n"
      "{\n"
      "  NfTurnSpec spec;\n"
      "  NfTurnPlan plan;\n"
      "  uint16_t pose_offset;\n"
      "  uint16_t intervals;\n"
      "} F413RoutePrecomputedGeometry;\n\n"
      "typedef struct\n"
      "{\n"
      "  uint8_t case_index;\n"
      "  NfTurnSpec timing[F413_ROUTE_PRECOMPUTED_SPEED_COUNT]"
      "[F413_ROUTE_PRECOMPUTED_KIND_COUNT];\n"
      "  NfTurnPlan timing_plan[F413_ROUTE_PRECOMPUTED_SPEED_COUNT]"
      "[F413_ROUTE_PRECOMPUTED_KIND_COUNT];\n"
      "  uint32_t turn_time_us[F413_ROUTE_PRECOMPUTED_SPEED_COUNT]"
      "[F413_ROUTE_PRECOMPUTED_KIND_COUNT];\n"
      "  NfLinearLimits orthogonal;\n"
      "  NfLinearLimits diagonal;\n"
      "  double speed_mm_s[F413_ROUTE_PRECOMPUTED_SPEED_COUNT];\n"
      "  uint32_t start_time_us;\n"
      "  /* Full linear-plan duration in microseconds. Distance is steps *\n"
      "   * the firmware DIST_HALF_SEC value for ORTHOGONAL or steps *\n"
      "   * DIST_D_HALF_SEC for DIAGONAL.\n"
      "   * Entry indexes speed_mm_s; exit indexes NOMINAL/LOW/CRAWL or\n"
      "   * zero velocity (STOP). UINT32_MAX means infeasible. Goal-cross\n"
      "   * time inside a selected plan remains position-dependent. */\n"
      "  uint32_t connector_time_us"
      "[F413_ROUTE_PRECOMPUTED_HEADING_CLASS_COUNT]"
      "[F413_ROUTE_PRECOMPUTED_SPEED_COUNT]"
      "[F413_ROUTE_PRECOMPUTED_EXIT_SPEED_COUNT]"
      "[F413_ROUTE_PRECOMPUTED_CONNECTOR_STEP_COUNT];\n"
      "  /* Orthogonal approach for large/small turns. At steps==0 only an\n"
      "   * unchanged speed is feasible. A one-step approach uses the exact\n"
      "   * constant-acceleration controller profile within DIST_HALF_SEC.\n"
      "   * For steps>=2, plan\n"
      "   * the leading (steps-1)*DIST_HALF_SEC to exit speed, then reserve\n"
      "   * the final DIST_HALF_SEC at\n"
      "   * constant speed. UINT32_MAX means infeasible. */\n"
      "  uint32_t orthogonal_approach_time_us"
      "[F413_ROUTE_PRECOMPUTED_SPEED_COUNT]"
      "[F413_ROUTE_PRECOMPUTED_SPEED_COUNT]"
      "[F413_ROUTE_PRECOMPUTED_CONNECTOR_STEP_COUNT];\n"
      "} F413RoutePrecomputedCase;\n\n"
      "extern const char g_f413_route_precomputed_input_sha256[];\n"
      "extern const NfTurnPose "
      "g_f413_route_precomputed_poses[F413_ROUTE_PRECOMPUTED_POSE_COUNT];\n"
      "extern const F413RoutePrecomputedGeometry "
      "g_f413_route_precomputed_geometry[F413_ROUTE_PRECOMPUTED_KIND_COUNT];\n"
      "extern const F413RoutePrecomputedCase "
      "g_f413_route_precomputed_cases[F413_ROUTE_PRECOMPUTED_CASE_COUNT];\n\n"
      "const F413RoutePrecomputedCase*\n"
      "f413_route_precomputed_find_case(uint8_t case_index);\n\n"
      "#ifdef __cplusplus\n}\n#endif\n\n"
      "#endif\n",
      input_sha, geometry_pose_count(geometry));
}

static void write_source(FILE* stream,
                         const char* input_sha,
                         const Geometry geometry[KIND_COUNT],
                         const MotionCase cases[CASE_COUNT])
{
  fprintf(stream,
      "/* Generated by tools/route_precompute/generate.py; DO NOT EDIT. */\n"
      "#include \"f413_route_motion_table.h\"\n\n"
      "const char g_f413_route_precomputed_input_sha256[] = \"%s\";\n\n",
      input_sha);

  fprintf(stream,
      "const NfTurnPose "
      "g_f413_route_precomputed_poses[F413_ROUTE_PRECOMPUTED_POSE_COUNT] = {\n");
  for (size_t kind = 0U; kind < KIND_COUNT; kind++)
  {
    fprintf(stream, "  /* kind %zu */\n", kind);
    for (uint16_t index = 0U; index <= geometry[kind].intervals; index++)
    {
      fprintf(stream, "  { ");
      print_double(stream, geometry[kind].poses[index].forward_mm);
      fprintf(stream, ", ");
      print_double(stream, geometry[kind].poses[index].lateral_mm);
      fprintf(stream, ", ");
      print_double(stream, geometry[kind].poses[index].heading_deg);
      fprintf(stream, " },\n");
    }
  }
  fprintf(stream, "};\n\n");

  fprintf(stream,
      "const F413RoutePrecomputedGeometry "
      "g_f413_route_precomputed_geometry[F413_ROUTE_PRECOMPUTED_KIND_COUNT] = {\n");
  size_t pose_offset = 0U;
  for (size_t kind = 0U; kind < KIND_COUNT; kind++)
  {
    fprintf(stream, "  {\n    ");
    print_turn_spec(stream, &geometry[kind].spec);
    fprintf(stream, ",\n    ");
    print_turn_plan(stream, &geometry[kind].plan);
    fprintf(stream, ",\n    %zuU,\n    %uU,\n  },\n",
            pose_offset, (unsigned int)geometry[kind].intervals);
    pose_offset += (size_t)geometry[kind].intervals + 1U;
  }
  fprintf(stream, "};\n\n");

  fprintf(stream,
      "const F413RoutePrecomputedCase "
      "g_f413_route_precomputed_cases[F413_ROUTE_PRECOMPUTED_CASE_COUNT] = {\n");
  for (size_t case_offset = 0U; case_offset < CASE_COUNT; case_offset++)
  {
    const MotionCase* item = &cases[case_offset];
    fprintf(stream, "  {\n    %uU,\n    {\n",
            (unsigned int)(CASE_FIRST + case_offset));
    for (size_t speed = 0U; speed < SPEED_COUNT; speed++)
    {
      fprintf(stream, "      {\n");
      for (size_t kind = 0U; kind < KIND_COUNT; kind++)
      {
        fprintf(stream, "        ");
        print_turn_spec(stream, &item->timing[speed][kind]);
        fprintf(stream, ",\n");
      }
      fprintf(stream, "      },\n");
    }
    fprintf(stream, "    },\n    {\n");
    for (size_t speed = 0U; speed < SPEED_COUNT; speed++)
    {
      fprintf(stream, "      {\n");
      for (size_t kind = 0U; kind < KIND_COUNT; kind++)
      {
        fprintf(stream, "        ");
        print_turn_plan(stream, &item->timing_plan[speed][kind]);
        fprintf(stream, ",\n");
      }
      fprintf(stream, "      },\n");
    }
    fprintf(stream, "    },\n    {\n");
    for (size_t speed = 0U; speed < SPEED_COUNT; speed++)
    {
      fprintf(stream, "      { ");
      for (size_t kind = 0U; kind < KIND_COUNT; kind++)
      {
        if (kind != 0U)
        {
          fprintf(stream, ", ");
        }
        fprintf(stream, "%uU", item->turn_time_us[speed][kind]);
      }
      fprintf(stream, " },\n");
    }
    fprintf(stream, "    },\n    ");
    print_linear_limits(stream, &item->orthogonal);
    fprintf(stream, ",\n    ");
    print_linear_limits(stream, &item->diagonal);
    fprintf(stream, ",\n    { ");
    for (size_t speed = 0U; speed < SPEED_COUNT; speed++)
    {
      if (speed != 0U)
      {
        fprintf(stream, ", ");
      }
      print_double(stream, item->speed_mm_s[speed]);
    }
    fprintf(stream, " },\n    %uU,\n    {\n", item->start_time_us);
    for (size_t heading_class = 0U;
         heading_class < HEADING_CLASS_COUNT; heading_class++)
    {
      fprintf(stream, "      {\n");
      for (size_t entry = 0U; entry < SPEED_COUNT; entry++)
      {
        fprintf(stream, "        {\n");
        for (size_t exit_speed = 0U;
             exit_speed < EXIT_SPEED_COUNT; exit_speed++)
        {
          fprintf(stream, "          {\n            ");
          for (size_t step = 0U; step < CONNECTOR_STEP_COUNT; step++)
          {
            const uint32_t duration =
                item->connector_time_us[heading_class][entry][exit_speed][step];
            if ((step != 0U) && ((step % 8U) == 0U))
            {
              fprintf(stream, "\n            ");
            }
            if (duration == UINT32_MAX)
            {
              fprintf(stream, "UINT32_MAX");
            }
            else
            {
              fprintf(stream, "%uU", duration);
            }
            if (step + 1U != CONNECTOR_STEP_COUNT)
            {
              fprintf(stream, ",");
              if (((step + 1U) % 8U) != 0U)
              {
                fprintf(stream, " ");
              }
            }
          }
          fprintf(stream, "\n          },\n");
        }
        fprintf(stream, "        },\n");
      }
      fprintf(stream, "      },\n");
    }
    fprintf(stream, "    },\n    {\n");
    for (size_t entry = 0U; entry < SPEED_COUNT; entry++)
    {
      fprintf(stream, "      {\n");
      for (size_t exit_speed = 0U; exit_speed < SPEED_COUNT; exit_speed++)
      {
        fprintf(stream, "        {\n          ");
        for (size_t step = 0U; step < CONNECTOR_STEP_COUNT; step++)
        {
          const uint32_t duration =
              item->orthogonal_approach_time_us[entry][exit_speed][step];
          if ((step != 0U) && ((step % 8U) == 0U))
          {
            fprintf(stream, "\n          ");
          }
          if (duration == UINT32_MAX)
          {
            fprintf(stream, "UINT32_MAX");
          }
          else
          {
            fprintf(stream, "%uU", duration);
          }
          if (step + 1U != CONNECTOR_STEP_COUNT)
          {
            fprintf(stream, ",");
            if (((step + 1U) % 8U) != 0U)
            {
              fprintf(stream, " ");
            }
          }
        }
        fprintf(stream, "\n        },\n");
      }
      fprintf(stream, "      },\n");
    }
    fprintf(stream, "    },\n  },\n");
  }
  fprintf(stream, "};\n\n");
  fprintf(stream,
      "const F413RoutePrecomputedCase*\n"
      "f413_route_precomputed_find_case(uint8_t case_index)\n"
      "{\n"
      "  if ((case_index < F413_ROUTE_PRECOMPUTED_CASE_FIRST) ||\n"
      "      (case_index > F413_ROUTE_PRECOMPUTED_CASE_LAST))\n"
      "  {\n"
      "    return NULL;\n"
      "  }\n"
      "  return &g_f413_route_precomputed_cases[\n"
      "      case_index - F413_ROUTE_PRECOMPUTED_CASE_FIRST];\n"
      "}\n");
}

int main(int argc, char** argv)
{
  Geometry geometry[KIND_COUNT] = {0};
  MotionCase cases[CASE_COUNT] = {0};
  FILE* header;
  FILE* source;

  if (argc != 4)
  {
    fprintf(stderr, "usage: %s OUTPUT_HEADER OUTPUT_SOURCE INPUT_SHA256\n",
            argv[0]);
    return EXIT_FAILURE;
  }
  calculate(geometry, cases);
  header = fopen(argv[1], "wb");
  if (header == NULL)
  {
    fprintf(stderr, "cannot open %s: %s\n", argv[1], strerror(errno));
    return EXIT_FAILURE;
  }
  write_header(header, argv[3], geometry);
  check_write(header);
  if (fclose(header) != 0)
  {
    fail("cannot close generated header");
  }
  source = fopen(argv[2], "wb");
  if (source == NULL)
  {
    fprintf(stderr, "cannot open %s: %s\n", argv[2], strerror(errno));
    return EXIT_FAILURE;
  }
  write_source(source, argv[3], geometry, cases);
  check_write(source);
  if (fclose(source) != 0)
  {
    fail("cannot close generated source");
  }
  return EXIT_SUCCESS;
}
