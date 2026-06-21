#include "f413_mode5.h"

#include "f413_mode_shortest.h"

typedef struct {
  uint8_t case_index;
  const char* params_ref;
  f413_run_features_t features;
} f413_mode5_case_t;

typedef struct {
  uint8_t case_index;
  const char* label;
  uint16_t codes[4];
  uint16_t code_count;
} f413_mode5_case0_sub_t;

#define F413_MODE5_FEATURES_ALL_ON_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = true, \
    .front_wall_correction_enabled = true, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

#define F413_MODE5_FEATURES_FRONT_WALLEND_OFF_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = false, \
    .front_wall_correction_enabled = false, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

static const f413_mode5_case_t k_cases[9] = {
  {1U, "shortestRunModeParams5 + shortestRunCaseParamsMode5[0]", F413_MODE5_FEATURES_FRONT_WALLEND_OFF_INIT},
  {2U, "shortestRunModeParams5 + shortestRunCaseParamsMode5[1]", F413_MODE5_FEATURES_FRONT_WALLEND_OFF_INIT},
  {3U, "shortestRunModeParams5 + shortestRunCaseParamsMode5[2]", F413_MODE5_FEATURES_ALL_ON_INIT},
  {4U, "shortestRunModeParams5 + shortestRunCaseParamsMode5[3]", F413_MODE5_FEATURES_ALL_ON_INIT},
  {5U, "shortestRunModeParams5 + shortestRunCaseParamsMode5[4]", F413_MODE5_FEATURES_ALL_ON_INIT},
  {6U, "shortestRunModeParams5 + shortestRunCaseParamsMode5[5]", F413_MODE5_FEATURES_ALL_ON_INIT},
  {7U, "shortestRunModeParams5 + shortestRunCaseParamsMode5[6]", F413_MODE5_FEATURES_ALL_ON_INIT},
  {8U, "shortestRunModeParams5 + shortestRunCaseParamsMode5[7]", F413_MODE5_FEATURES_ALL_ON_INIT},
  {9U, "shortestRunModeParams5 + shortestRunCaseParamsMode5[8]", F413_MODE5_FEATURES_ALL_ON_INIT},
};

static const f413_mode5_case0_sub_t k_case0_subs[10] = {
  {3U, "mode5-case0-sub0 small R90",      {205U, 300U, 0U, 0U},       2U},
  {3U, "mode5-case0-sub1 large R90",      {204U, 501U, 0U, 0U},       2U},
  {3U, "mode5-case0-sub2 large R180",     {204U, 502U, 0U, 0U},       2U},
  {8U, "mode5-case0-sub3 R135 in",        {204U, 901U, 1001U, 0U},    3U},
  {8U, "mode5-case0-sub4 R135 out",       {204U, 1001U, 904U, 1001U}, 4U},
  {8U, "mode5-case0-sub5 L45 in",         {204U, 1001U, 702U, 1001U}, 4U},
  {8U, "mode5-case0-sub6 R135 out",       {204U, 903U, 1001U, 0U},    3U},
  {8U, "mode5-case0-sub7 R135 out",       {204U, 1001U, 904U, 1001U}, 4U},
  {1U, "mode5-case0-sub8 straight case1", {203U, 0U, 0U, 0U},         1U},
  {5U, "mode5-case0-sub9 straight case5", {203U, 0U, 0U, 0U},         1U},
};

void f413_mode5_run_case(uint8_t op_case)
{
  f413_shortest_case_config_t config;

  if ((op_case < 1U) || (op_case > 9U))
  {
    f413_mode_shortest_run_case(5U, op_case);
    return;
  }

  config.mode = 5U;
  config.op_case = op_case;
  config.label = k_cases[op_case - 1U].params_ref;
  config.features = k_cases[op_case - 1U].features;
  f413_mode_shortest_run_config(&config);
}

void f413_mode5_run_case0_sub(uint8_t sub)
{
  if (sub >= 10U)
  {
    return;
  }

  f413_mode_shortest_run_case0_path(k_case0_subs[sub].label,
                                    5U,
                                    k_case0_subs[sub].case_index,
                                    k_case0_subs[sub].codes,
                                    k_case0_subs[sub].code_count);
}
