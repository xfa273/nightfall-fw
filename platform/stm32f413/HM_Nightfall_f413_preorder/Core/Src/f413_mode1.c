#include "f413_mode1.h"

#include "f413_run_features.h"
#include "f413_search_step.h"

typedef struct {
  uint8_t case_index;
  const char* params_ref;
  f413_search_step_case_config_t run;
} f413_mode1_case_t;

typedef struct {
  uint8_t sub;
  const char* params_ref;
  f413_search_step_case0_test_config_t run;
} f413_mode1_case0_sub_t;

#define F413_MODE1_FEATURES_ALL_ON_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = true, \
    .front_wall_correction_enabled = true, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

#define F413_MODE1_FEATURES_WALLEND_OFF_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = false, \
    .front_wall_correction_enabled = true, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

#define F413_MODE1_FEATURES_FRONTWALL_OFF_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = true, \
    .front_wall_correction_enabled = false, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

#define F413_MODE1_FEATURES_WALLEND_FRONTWALL_OFF_INIT \
  { \
    .wall_control_enabled = true, \
    .wall_end_correction_enabled = false, \
    .front_wall_correction_enabled = false, \
    .angle_accum_mode = true, \
    .test_mode_run = false, \
  }

#define F413_MODE1_FEATURES_CASE0_TEST_INIT \
  { \
    .wall_control_enabled = false, \
    .wall_end_correction_enabled = false, \
    .front_wall_correction_enabled = false, \
    .angle_accum_mode = false, \
    .test_mode_run = true, \
  }

static const f413_mode1_case0_sub_t k_case0_subs[4] = {
  {
    0U,
    "reserved",
    {0U, F413_SEARCH_STEP_CASE0_TEST_NONE,
     "mode1-case0-sub0 reserved", F413_MODE1_FEATURES_CASE0_TEST_INIT},
  },
  {
    1U,
    "searchRunParams[0] standard: half accel -> small R90 -> half decel",
    {0U, F413_SEARCH_STEP_CASE0_TEST_TURN_R90,
     "mode1-case0-sub1 standard R90", F413_MODE1_FEATURES_CASE0_TEST_INIT},
  },
  {
    2U,
    "searchRunParams[1] low-speed: half accel -> small R90 -> half decel",
    {1U, F413_SEARCH_STEP_CASE0_TEST_TURN_R90,
     "mode1-case0-sub2 low R90", F413_MODE1_FEATURES_CASE0_TEST_INIT},
  },
  {
    3U,
    "searchRunParams[0] standard: first section -> two full sections",
    {0U, F413_SEARCH_STEP_CASE0_TEST_STRAIGHT_3,
     "mode1-case0-sub3 standard straight 3-section", F413_MODE1_FEATURES_CASE0_TEST_INIT},
  },
};

static const f413_mode1_case_t k_cases[8] = {
  {
    1U,
    "searchRunParams[0] standard: GOAL -> FULL",
    {0U, 2U, {F413_SEARCH_STEP_TARGET_GOAL, F413_SEARCH_STEP_TARGET_FULL},
     "mode1-case1 standard goal-full", F413_MODE1_FEATURES_WALLEND_OFF_INIT},
  },
  {
    2U,
    "searchRunParams[0] standard: FULL",
    {0U, 1U, {F413_SEARCH_STEP_TARGET_FULL, 0U},
     "mode1-case2 standard full", F413_MODE1_FEATURES_ALL_ON_INIT},
  },
  {
    3U,
    "searchRunParams[0] standard: GOAL -> START",
    {0U, 2U, {F413_SEARCH_STEP_TARGET_GOAL, F413_SEARCH_STEP_TARGET_START},
     "mode1-case3 standard goal-start", F413_MODE1_FEATURES_WALLEND_OFF_INIT},
  },
  {
    4U,
    "searchRunParams[0] standard: GOAL",
    {0U, 1U, {F413_SEARCH_STEP_TARGET_GOAL, 0U},
     "mode1-case4 standard goal", F413_MODE1_FEATURES_WALLEND_OFF_INIT},
  },
  {
    5U,
    "searchRunParams[1] low-speed: GOAL -> FULL",
    {1U, 2U, {F413_SEARCH_STEP_TARGET_GOAL, F413_SEARCH_STEP_TARGET_FULL},
     "mode1-case5 low goal-full", F413_MODE1_FEATURES_ALL_ON_INIT},
  },
  {
    6U,
    "searchRunParams[1] low-speed: FULL",
    {1U, 1U, {F413_SEARCH_STEP_TARGET_FULL, 0U},
     "mode1-case6 low full", F413_MODE1_FEATURES_ALL_ON_INIT},
  },
  {
    7U,
    "searchRunParams[1] low-speed: GOAL -> START",
    {1U, 2U, {F413_SEARCH_STEP_TARGET_GOAL, F413_SEARCH_STEP_TARGET_START},
     "mode1-case7 low goal-start", F413_MODE1_FEATURES_ALL_ON_INIT},
  },
  {
    8U,
    "searchRunParams[1] low-speed: GOAL",
    {1U, 1U, {F413_SEARCH_STEP_TARGET_GOAL, 0U},
     "mode1-case8 low goal", F413_MODE1_FEATURES_ALL_ON_INIT},
  },
};

void f413_mode1_run_case(uint8_t op_case)
{
  if ((op_case < 1U) || (op_case > 8U))
  {
    f413_search_step_run_search_case_once(op_case);
    return;
  }

  f413_search_step_run_config_once(op_case, &k_cases[op_case - 1U].run);
}

void f413_mode1_run_case0_sub(uint8_t sub)
{
  if (sub >= 4U)
  {
    f413_search_step_run_case0_test_once(sub, NULL);
    return;
  }

  f413_search_step_run_case0_test_once(sub, &k_case0_subs[sub].run);
}
