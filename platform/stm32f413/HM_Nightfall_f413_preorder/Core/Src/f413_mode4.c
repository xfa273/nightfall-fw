#include "f413_mode4.h"

#include "f413_mode_shortest.h"
#include "params.h"
#include "shortest_run_params.h"

typedef struct {
  uint8_t case_index;
  const char* params_ref;
  f413_run_features_t features;
} f413_mode4_case_t;

typedef struct {
  uint8_t case_index;
  const char* label;
  uint16_t codes[5];
  uint16_t code_count;
} f413_mode4_case0_sub_t;

#define F413_MODE4_FEATURES_ALL_ON_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = true, \
    .front_wall_correction_enabled = true, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

#define F413_MODE4_FEATURES_FRONT_WALLEND_OFF_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = false, \
    .front_wall_correction_enabled = false, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

static const f413_mode4_case_t k_cases[9] = {
  {1U, "shortestRunModeParams4 + shortestRunCaseParamsMode4[0]", F413_MODE4_FEATURES_FRONT_WALLEND_OFF_INIT},
  {2U, "shortestRunModeParams4 + shortestRunCaseParamsMode4[1]", F413_MODE4_FEATURES_FRONT_WALLEND_OFF_INIT},
  {3U, "shortestRunModeParams4 + shortestRunCaseParamsMode4[2]", F413_MODE4_FEATURES_ALL_ON_INIT},
  {4U, "shortestRunModeParams4 + shortestRunCaseParamsMode4[3]", F413_MODE4_FEATURES_ALL_ON_INIT},
  {5U, "shortestRunModeParams4 + shortestRunCaseParamsMode4[4]", F413_MODE4_FEATURES_ALL_ON_INIT},
  {6U, "shortestRunModeParams4 + shortestRunCaseParamsMode4[5]", F413_MODE4_FEATURES_ALL_ON_INIT},
  {7U, "shortestRunModeParams4 + shortestRunCaseParamsMode4[6]", F413_MODE4_FEATURES_ALL_ON_INIT},
  {8U, "shortestRunModeParams4 + shortestRunCaseParamsMode4[7]", F413_MODE4_FEATURES_ALL_ON_INIT},
  {9U, "shortestRunModeParams4 + shortestRunCaseParamsMode4[8]", F413_MODE4_FEATURES_ALL_ON_INIT},
};

static const f413_mode4_case0_sub_t k_case0_subs[10] = {
  {3U, "mode4-case0-sub0 small R90",      {203U, 300U, 0U, 0U},       2U},
  {3U, "mode4-case0-sub1 large R90",      {204U, 501U, 0U, 0U},       2U},
  {3U, "mode4-case0-sub2 large R180",     {204U, 502U, 0U, 0U},       2U},
  {8U, "mode4-case0-sub3 R135 in",        {204U, 901U, 1001U, 0U},    3U},
  {8U, "mode4-case0-sub4 R45 in",         {204U, 1001U, 701U, 1001U}, 4U},
  {8U, "mode4-case0-sub5 R45 in",         {204U, 1001U, 701U, 1001U}, 4U},
  {8U, "mode4-case0-sub6 R135 out",       {204U, 903U, 1001U, 0U},    3U},
  {8U, "mode4-case0-sub7 R135 out",       {204U, 1001U, 904U, 1001U}, 4U},
  {1U, "mode4-case0-sub8 straight case1", {209U, 0U, 0U, 0U},         1U},
  {5U, "mode4-case0-sub9 straight case5", {209U, 0U, 0U, 0U},         1U},
};

/* Same sub-number meanings as mode2. A diagonal exit/V90 is approached
 * through an entry turn, never from a cardinal heading. S4 gives 1200 mm/s
 * sufficient run-up with mode4 case8; DS3 gives room for terminal braking. */
static const f413_mode4_case0_sub_t k_suction_case0_subs[10] = {
  {3U, "mode4-case0-sub0 small R90",  {203U, 300U, 203U},                3U},
  {3U, "mode4-case0-sub1 large R90",  {204U, 501U, 203U},                3U},
  {3U, "mode4-case0-sub2 large R180", {204U, 502U, 203U},                3U},
  {8U, "mode4-case0-sub3 R45 in",     {204U, 701U, 1003U},               3U},
  {8U, "mode4-case0-sub4 L45 out",    {204U, 701U, 1003U, 704U, 203U},   5U},
  {8U, "mode4-case0-sub5 L-V90",      {204U, 701U, 1003U, 802U, 1003U},  5U},
  {8U, "mode4-case0-sub6 R135 in",    {204U, 901U, 1003U},               3U},
  {8U, "mode4-case0-sub7 L135 out",   {204U, 901U, 1003U, 904U, 203U},   5U},
  {1U, "mode4-case0-sub8 straight case1", {209U},                       1U},
  {5U, "mode4-case0-sub9 straight case5", {209U},                       1U},
};

void f413_mode4_run_case(uint8_t op_case)
{
  f413_shortest_case_config_t config;

  if ((op_case < 1U) || (op_case > 9U))
  {
    f413_mode_shortest_run_case(4U, op_case);
    return;
  }

  config.mode = 4U;
  config.op_case = k_cases[op_case - 1U].case_index;
  config.label = k_cases[op_case - 1U].params_ref;
  config.features = k_cases[op_case - 1U].features;
  config.diagonal_time_plan = false;
  f413_mode_shortest_run_config(&config);
}

void f413_mode4_run_case0_sub(uint8_t sub)
{
  if (sub >= 10U)
  {
    return;
  }

  const f413_mode4_case0_sub_t* tests = (shortestRunModeParams4.fan_power > 0)
      ? k_suction_case0_subs : k_case0_subs;
  f413_mode_shortest_run_case0_path(tests[sub].label,
                                    4U,
                                    tests[sub].case_index,
                                    tests[sub].codes,
                                    tests[sub].code_count);
}
