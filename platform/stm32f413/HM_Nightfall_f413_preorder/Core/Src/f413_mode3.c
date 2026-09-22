#include "f413_mode3.h"

#include "f413_mode_shortest.h"
#include "params.h"
#include "shortest_run_params.h"

typedef struct {
  uint8_t case_index;
  const char* params_ref;
  f413_run_features_t features;
} f413_mode3_case_t;

typedef struct {
  uint8_t case_index;
  const char* label;
  uint16_t codes[5];
  uint16_t code_count;
} f413_mode3_case0_sub_t;

#define F413_MODE3_FEATURES_ALL_ON_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = true, \
    .front_wall_correction_enabled = true, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

#define F413_MODE3_FEATURES_FRONT_WALLEND_OFF_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = false, \
    .front_wall_correction_enabled = false, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

static const f413_mode3_case_t k_cases[9] = {
  {1U, "shortestRunModeParams3 + shortestRunCaseParamsMode3[0]", F413_MODE3_FEATURES_FRONT_WALLEND_OFF_INIT},
  {2U, "shortestRunModeParams3 + shortestRunCaseParamsMode3[1]", F413_MODE3_FEATURES_FRONT_WALLEND_OFF_INIT},
  {3U, "shortestRunModeParams3 + shortestRunCaseParamsMode3[2]", F413_MODE3_FEATURES_ALL_ON_INIT},
  {4U, "shortestRunModeParams3 + shortestRunCaseParamsMode3[3]", F413_MODE3_FEATURES_ALL_ON_INIT},
  {5U, "shortestRunModeParams3 + shortestRunCaseParamsMode3[4]", F413_MODE3_FEATURES_ALL_ON_INIT},
  {6U, "shortestRunModeParams3 + shortestRunCaseParamsMode3[5]", F413_MODE3_FEATURES_ALL_ON_INIT},
  {7U, "shortestRunModeParams3 + shortestRunCaseParamsMode3[6]", F413_MODE3_FEATURES_ALL_ON_INIT},
  {8U, "shortestRunModeParams3 + shortestRunCaseParamsMode3[7]", F413_MODE3_FEATURES_ALL_ON_INIT},
  {9U, "shortestRunModeParams3 + shortestRunCaseParamsMode3[8]", F413_MODE3_FEATURES_ALL_ON_INIT},
};

static const f413_mode3_case0_sub_t k_case0_subs[10] = {
  {3U, "mode3-case0-sub0 small R90",      {203U, 300U, 0U, 0U},       2U},
  {3U, "mode3-case0-sub1 large R90",      {203U, 501U, 0U, 0U},       2U},
  {3U, "mode3-case0-sub2 large R180",     {205U, 502U, 0U, 0U},       2U},
  {8U, "mode3-case0-sub3 R135 in",        {203U, 901U, 1001U, 0U},    3U},
  {8U, "mode3-case0-sub4 R135 out",       {203U, 1001U, 904U, 1001U}, 4U},
  {8U, "mode3-case0-sub5 L45 in",         {203U, 1001U, 702U, 1001U}, 4U},
  {8U, "mode3-case0-sub6 R135 out",       {203U, 903U, 1001U, 0U},    3U},
  {8U, "mode3-case0-sub7 R135 out",       {203U, 1001U, 904U, 1001U}, 4U},
  {1U, "mode3-case0-sub8 straight case1", {205U, 0U, 0U, 0U},         1U},
  {5U, "mode3-case0-sub9 straight case5", {205U, 0U, 0U, 0U},         1U},
};

/* Valid cardinal/diagonal approaches and explicit braking tails for suction tuning. */
static const f413_mode3_case0_sub_t k_suction_case0_subs[10] = {
  {1U, "mode3-case0-sub0 small R90",  {203U, 300U, 203U},                3U},
  {2U, "mode3-case0-sub1 large R90",  {204U, 501U, 203U},                3U},
  {2U, "mode3-case0-sub2 large R180", {206U, 502U, 203U},                3U},
  {8U, "mode3-case0-sub3 R45 in",     {204U, 701U, 1003U},               3U},
  {8U, "mode3-case0-sub4 L45 out",    {204U, 701U, 1003U, 704U, 203U},   5U},
  {8U, "mode3-case0-sub5 L-V90",      {204U, 701U, 1003U, 802U, 1003U},  5U},
  {8U, "mode3-case0-sub6 R135 in",    {204U, 901U, 1003U},               3U},
  {8U, "mode3-case0-sub7 L135 out",   {204U, 901U, 1003U, 904U, 203U},   5U},
  {1U, "mode3-case0-sub8 straight case1", {209U},                       1U},
  {5U, "mode3-case0-sub9 straight case5", {209U},                       1U},
};

void f413_mode3_run_case(uint8_t op_case)
{
  f413_shortest_case_config_t config;

  if ((op_case < 1U) || (op_case > 9U))
  {
    f413_mode_shortest_run_case(3U, op_case);
    return;
  }

  config.mode = 3U;
  config.op_case = k_cases[op_case - 1U].case_index;
  config.label = k_cases[op_case - 1U].params_ref;
  config.features = k_cases[op_case - 1U].features;
  config.diagonal_time_plan = false;
  f413_mode_shortest_run_config(&config);
}

void f413_mode3_run_case0_sub(uint8_t sub)
{
  if (sub >= 10U)
  {
    return;
  }

  const f413_mode3_case0_sub_t* tests = (shortestRunModeParams3.fan_power > 0)
      ? k_suction_case0_subs : k_case0_subs;
  f413_mode_shortest_run_case0_path(tests[sub].label,
                                    3U,
                                    tests[sub].case_index,
                                    tests[sub].codes,
                                    tests[sub].code_count);
}
