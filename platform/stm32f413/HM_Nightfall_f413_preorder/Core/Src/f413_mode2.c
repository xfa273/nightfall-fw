#include "f413_mode2.h"

#include "f413_mode_shortest.h"

typedef struct {
  uint8_t case_index;
  const char* params_ref;
  f413_run_features_t features;
} f413_mode2_case_t;

typedef struct {
  uint8_t case_index;
  const char* label;
  uint16_t codes[5];
  uint16_t code_count;
} f413_mode2_case0_sub_t;

#define F413_MODE2_FEATURES_ALL_ON_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = true, \
    .front_wall_correction_enabled = true, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

#define F413_MODE2_FEATURES_FRONT_OFF_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = true, \
    .front_wall_correction_enabled = false, \
    .angle_accum_mode = false, \
    .test_mode_run = false, \
  }

#define F413_MODE2_FEATURES_FRONT_ON_NO_ACCUM_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = true, \
    .front_wall_correction_enabled = true, \
    .angle_accum_mode = false, \
    .test_mode_run = false, \
  }

#define F413_MODE2_FEATURES_NO_WALL_CORRECTION_INIT \
  { \
    .wall_control_enabled = false, \
    .wall_end_correction_enabled = false, \
    .front_wall_correction_enabled = false, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

static const f413_mode2_case_t k_cases[9] = {
  {1U, "shortestRunModeParams2 + shortestRunCaseParamsMode2[0]", F413_MODE2_FEATURES_FRONT_ON_NO_ACCUM_INIT},
  {2U, "shortestRunModeParams2 + shortestRunCaseParamsMode2[1]", F413_MODE2_FEATURES_FRONT_OFF_INIT},
  {4U, "shortestRunModeParams2 + shortestRunCaseParamsMode2[3] (F405 case3 mapping)", F413_MODE2_FEATURES_ALL_ON_INIT},
  {4U, "shortestRunModeParams2 + shortestRunCaseParamsMode2[3]", F413_MODE2_FEATURES_ALL_ON_INIT},
  {5U, "shortestRunModeParams2 + shortestRunCaseParamsMode2[4]", F413_MODE2_FEATURES_ALL_ON_INIT},
  {6U, "shortestRunModeParams2 + shortestRunCaseParamsMode2[5]", F413_MODE2_FEATURES_ALL_ON_INIT},
  {7U, "shortestRunModeParams2 + shortestRunCaseParamsMode2[6]", F413_MODE2_FEATURES_ALL_ON_INIT},
  {8U, "shortestRunModeParams2 + shortestRunCaseParamsMode2[7]", F413_MODE2_FEATURES_ALL_ON_INIT},
  {9U, "shortestRunModeParams2 + shortestRunCaseParamsMode2[8]", F413_MODE2_FEATURES_ALL_ON_INIT},
};

static const f413_mode2_case0_sub_t k_case0_subs[10] = {
  {3U, "mode2-case0-sub0 small R90",             {203U, 300U},                         2U},
  {3U, "mode2-case0-sub1 large R90",             {203U, 501U},                         2U},
  {3U, "mode2-case0-sub2 large R180",            {203U, 502U},                         2U},
  /* Match the low-speed diagonal shortest-run profile used by mode2 case6.
   * The previous case8 acceleration made the calibration paths sensitive to
   * approach slip before the 500mm/s turn-speed boundary was reached. */
  {6U, "mode2-case0-sub3 R45 in (case6 approach)",                {203U, 701U, 1001U},                  3U},
  {6U, "mode2-case0-sub4 L45 out after R45 in (case6 approach)",  {203U, 701U, 1001U, 704U},            4U},
  {6U, "mode2-case0-sub5 L-V90 after R45 in (case6 approach)",    {203U, 701U, 1001U, 802U, 1001U},     5U},
  {6U, "mode2-case0-sub6 R135 in (case6 approach)",               {203U, 901U, 1001U},                  3U},
  {6U, "mode2-case0-sub7 L135 out after R135 in (case6 approach)", {203U, 901U, 1001U, 904U},            4U},
  {1U, "mode2-case0-sub8 straight case1",        {209U},                               1U},
  {5U, "mode2-case0-sub9 straight case5",        {209U},                               1U},
};

/*
 * Deterministic open-floor validation route for the low-speed diagonal
 * profile.  It enters a right diagonal, crosses a left V90, and exits right
 * back to a cardinal heading.  The explicit diagonal straights separate the
 * primitives so their measured swept paths can be compared with the intended
 * centre lines without relying on a saved maze or on wall/post feedback.
 */
static const uint16_t k_case6_open_floor_diagonal_path[] = {
  203U, 701U, 1001U, 802U, 1001U, 703U, 202U,
};

static const f413_run_features_t k_case6_no_wall_correction_features =
    F413_MODE2_FEATURES_NO_WALL_CORRECTION_INIT;

void f413_mode2_run_case(uint8_t op_case)
{
  f413_shortest_case_config_t config;

  if (op_case == 6U)
  {
    f413_mode_shortest_run_path_config(
        "mode2-case6 open-floor diagonal (wall corrections off)",
        2U,
        6U,
        k_case6_open_floor_diagonal_path,
        (uint16_t)(sizeof(k_case6_open_floor_diagonal_path) /
                   sizeof(k_case6_open_floor_diagonal_path[0])),
        &k_case6_no_wall_correction_features);
    return;
  }

  if ((op_case < 1U) || (op_case > 9U))
  {
    f413_mode_shortest_run_case(2U, op_case);
    return;
  }

  config.mode = 2U;
  config.op_case = k_cases[op_case - 1U].case_index;
  config.label = k_cases[op_case - 1U].params_ref;
  config.features = k_cases[op_case - 1U].features;
  config.diagonal_time_plan = op_case >= 6U;
  f413_mode_shortest_run_config(&config);
}

void f413_mode2_run_case0_sub(uint8_t sub)
{
  if (sub >= 10U)
  {
    return;
  }

  f413_mode_shortest_run_case0_path(k_case0_subs[sub].label,
                                    2U,
                                    k_case0_subs[sub].case_index,
                                    k_case0_subs[sub].codes,
                                    k_case0_subs[sub].code_count);
}
