#include "f413_mode6.h"

#include "f413_mode_shortest.h"

typedef struct {
  uint8_t case_index;
  const char* params_ref;
  f413_run_features_t features;
} f413_mode6_case_t;

typedef struct {
  uint8_t case_index;
  const char* label;
  uint16_t codes[4];
  uint16_t code_count;
} f413_mode6_case0_sub_t;

#define F413_MODE6_FEATURES_ALL_ON_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = true, \
    .front_wall_correction_enabled = true, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

#define F413_MODE6_FEATURES_FRONT_WALLEND_OFF_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = false, \
    .front_wall_correction_enabled = false, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

#define F413_MODE6_FEATURES_FRONT_OFF_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = true, \
    .front_wall_correction_enabled = false, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

static const f413_mode6_case_t k_cases[9] = {
  {1U, "shortestRunModeParams6 + shortestRunCaseParamsMode6[0]", F413_MODE6_FEATURES_FRONT_WALLEND_OFF_INIT},
  {2U, "shortestRunModeParams6 + shortestRunCaseParamsMode6[1]", F413_MODE6_FEATURES_FRONT_WALLEND_OFF_INIT},
  {3U, "shortestRunModeParams6 + shortestRunCaseParamsMode6[2]", F413_MODE6_FEATURES_ALL_ON_INIT},
  {4U, "shortestRunModeParams6 + shortestRunCaseParamsMode6[3]", F413_MODE6_FEATURES_FRONT_OFF_INIT},
  {5U, "shortestRunModeParams6 + shortestRunCaseParamsMode6[4]", F413_MODE6_FEATURES_FRONT_OFF_INIT},
  {6U, "shortestRunModeParams6 + shortestRunCaseParamsMode6[5]", F413_MODE6_FEATURES_FRONT_OFF_INIT},
  {7U, "shortestRunModeParams6 + shortestRunCaseParamsMode6[6]", F413_MODE6_FEATURES_FRONT_OFF_INIT},
  {8U, "shortestRunModeParams6 + shortestRunCaseParamsMode6[7]", F413_MODE6_FEATURES_FRONT_OFF_INIT},
  {9U, "shortestRunModeParams6 + shortestRunCaseParamsMode6[8]", F413_MODE6_FEATURES_FRONT_OFF_INIT},
};

static const f413_mode6_case0_sub_t k_case0_subs[10] = {
  {1U, "mode6-case0-sub0 small R90",      {205U, 300U, 0U, 0U},       2U},
  {1U, "mode6-case0-sub1 large R90",      {204U, 501U, 0U, 0U},       2U},
  {1U, "mode6-case0-sub2 large R180",     {204U, 502U, 0U, 0U},       2U},
  {8U, "mode6-case0-sub3 R45 in",         {204U, 701U, 1001U, 0U},    3U},
  {8U, "mode6-case0-sub4 R45 out",        {204U, 1001U, 704U, 1001U}, 4U},
  {8U, "mode6-case0-sub5 V90",            {204U, 1001U, 802U, 1001U}, 4U},
  {8U, "mode6-case0-sub6 R135 in",        {204U, 901U, 1001U, 0U},    3U},
  {8U, "mode6-case0-sub7 R135 out",       {204U, 1001U, 904U, 1001U}, 4U},
  {1U, "mode6-case0-sub8 straight case1", {203U, 0U, 0U, 0U},         1U},
  {5U, "mode6-case0-sub9 straight case5", {203U, 0U, 0U, 0U},         1U},
};

void f413_mode6_run_case(uint8_t op_case)
{
  f413_shortest_case_config_t config;

  if ((op_case < 1U) || (op_case > 9U))
  {
    f413_mode_shortest_run_case(6U, op_case);
    return;
  }

  config.mode = 6U;
  config.op_case = k_cases[op_case - 1U].case_index;
  config.label = k_cases[op_case - 1U].params_ref;
  config.features = k_cases[op_case - 1U].features;
  f413_mode_shortest_run_config(&config);
}

void f413_mode6_run_case0_sub(uint8_t sub)
{
  if (sub >= 10U)
  {
    return;
  }

  f413_mode_shortest_run_case0_path(k_case0_subs[sub].label,
                                    6U,
                                    k_case0_subs[sub].case_index,
                                    k_case0_subs[sub].codes,
                                    k_case0_subs[sub].code_count);
}
