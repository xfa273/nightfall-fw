#include <assert.h>
#include <stdio.h>
#define NIGHTFALL_F413_PATH_LINEAR_PLAN_HOST_TEST (1U)
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_path_run.c"

static void check_mode(const ShortestRunModeParams_t* m, const ShortestRunCaseParams_t* cases)
{
  const uint16_t codes[] = {300,400,501,601,502,602,701,702,703,704,801,802,901,902,903,904};
  const float expected[] = {m->velocity_turn90,m->velocity_turn90,
    m->velocity_l_turn_90,m->velocity_l_turn_90,m->velocity_l_turn_180,m->velocity_l_turn_180,
    m->velocity_turn45in,m->velocity_turn45in,m->velocity_turn45out,m->velocity_turn45out,
    m->velocity_turnV90,m->velocity_turnV90,m->velocity_turn135in,m->velocity_turn135in,
    m->velocity_turn135out,m->velocity_turn135out};
  for (unsigned t=0; t<16; ++t)
  {
    float velocity;
    assert(f413_path_run_turn_velocity_from_code(codes[t],m,&velocity) == (expected[t] > 0));
    if (expected[t] > 0) assert(velocity == expected[t]);
  }
  for (unsigned c=0; c<9; ++c)
  {
    NfLinearLimits limits;
    assert(f413_path_run_straight_limits(m,&cases[c],&limits));
    assert(limits.vmax_mm_s == cases[c].velocity_straight);
    assert(limits.accel_low_mm_s2 == cases[c].acceleration_straight);
    assert(limits.accel_high_mm_s2 == cases[c].acceleration_straight_dash);
    if (cases[c].velocity_d_straight > 0)
    {
      assert(f413_path_run_diagonal_limits(m,&cases[c],&limits));
      assert(limits.vmax_mm_s == cases[c].velocity_d_straight);
      assert(limits.accel_low_mm_s2 == cases[c].acceleration_d_straight);
      assert(limits.accel_high_mm_s2 == cases[c].acceleration_d_straight_dash);
    }
  }
}
int main(void)
{
  check_mode(&shortestRunModeParams2,shortestRunCaseParamsMode2);
  check_mode(&shortestRunModeParams3,shortestRunCaseParamsMode3);
  check_mode(&shortestRunModeParams4,shortestRunCaseParamsMode4);
  check_mode(&shortestRunModeParams5,shortestRunCaseParamsMode5);
  check_mode(&shortestRunModeParams6,shortestRunCaseParamsMode6);
  check_mode(&shortestRunModeParams7,shortestRunCaseParamsMode7);
  /* Values above every former special-case ceiling must reach the planner. */
  ShortestRunModeParams_t m = shortestRunModeParams2;
  ShortestRunCaseParams_t c = shortestRunCaseParamsMode2[0];
  m.velocity_l_turn_90=4300; m.accel_switch_velocity=2000;
  c.velocity_straight=5000; c.velocity_d_straight=4500;
  c.acceleration_straight=c.acceleration_straight_dash=20000;
  c.acceleration_d_straight=c.acceleration_d_straight_dash=18000;
  float velocity;
  NfLinearLimits limits;
  NfLinearPlan plan;
  assert(f413_path_run_turn_velocity_from_code(501,&m,&velocity) && velocity == 4300);
  assert(f413_path_run_straight_limits(&m,&c,&limits));
  assert(f413_path_run_make_linear_plan(4500,0,0,&limits,&plan));
  assert(plan.peak_velocity_mm_s == 5000);
  assert(f413_path_run_diagonal_limits(&m,&c,&limits));
  assert(f413_path_run_make_linear_plan(4500,0,0,&limits,&plan));
  assert(plan.peak_velocity_mm_s == 4500);
  c.velocity_straight=100;
  assert(f413_path_run_boundary_speed(&m,&c,DIST_FIRST_SEC) == 100);
  c.velocity_straight=5000; m.accel_switch_velocity=100;
  c.acceleration_straight_dash=1000;
  assert(fabsf(f413_path_run_boundary_speed(&m,&c,DIST_FIRST_SEC)-sqrtf(2000*DIST_FIRST_SEC)) < .001f);
  m.velocity_l_turn_90=500; m.accel_switch_velocity=2000;
  c.velocity_straight=1000;
  c.acceleration_straight=c.acceleration_straight_dash=1000;
  const uint16_t tail_test[] = {203,501,0};
  f413_path_run_prepared_path_t prepared;
  const float first = f413_path_run_boundary_speed(&m,&c,DIST_FIRST_SEC);
  assert(f413_path_run_preflight_prepare(tail_test,3,&m,&c,first,false,true,&prepared).status
         == F413_PATH_RUN_PREFLIGHT_OK);
  assert(prepared.stop_distance_mm == 125);
  assert(f413_path_run_preflight(tail_test,3,&m,&c,first,false,false).status
         == F413_PATH_RUN_PREFLIGHT_INFEASIBLE_LINEAR);
  m.velocity_l_turn_90=-4300;
  assert(!f413_path_run_turn_velocity_from_code(501,&m,&velocity));
  m.velocity_l_turn_90=NAN;
  assert(!f413_path_run_turn_velocity_from_code(501,&m,&velocity));
  puts("PASS: all 6 modes / 9 cases use configured velocities and accelerations, high-speed and start-vmax regressions");
}
