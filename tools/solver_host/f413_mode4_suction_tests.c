#include <assert.h>
#include <stdio.h>
#include <string.h>
#define NIGHTFALL_F413_PATH_LINEAR_PLAN_HOST_TEST (1U)
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_path_run.c"
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_mode4.c"

static unsigned calls, selected_sub;
void f413_mode_shortest_run_case(uint8_t mode, uint8_t c) { (void)mode; (void)c; }
void f413_mode_shortest_run_config(const f413_shortest_case_config_t* c) { (void)c; }
void f413_mode_shortest_run_case0_path(const char* label, uint8_t mode, uint8_t c,
                                     const uint16_t* codes, uint16_t count)
{
  (void)label;
  assert(mode == 4 && c >= 1 && c <= 9);
  uint16_t terminated[16] = {0};
  memcpy(terminated, codes, count * sizeof(*codes));
  const ShortestRunCaseParams_t selected = f413_path_run_session_case_params(
      terminated, 16, &shortestRunModeParams4, &shortestRunCaseParamsMode4[c-1], true);
  const ShortestRunCaseParams_t* cp = &selected;
  const float initial = f413_path_run_boundary_speed(&shortestRunModeParams4, cp, DIST_FIRST_SEC);
  f413_path_run_prepared_path_t prepared;
  f413_path_run_preflight_result_t r = f413_path_run_preflight_prepare(
      terminated, 16, &shortestRunModeParams4, cp, initial, false, true, &prepared);
  if (r.status != F413_PATH_RUN_PREFLIGHT_OK)
    fprintf(stderr,"sub%u preflight=%u index=%zu code=%u\n",selected_sub,r.status,r.index,r.code);
  assert(r.status == F413_PATH_RUN_PREFLIGHT_OK);
  assert(prepared.count > 0);
  for (unsigned i=0; i<count; ++i)
  {
    float v;
    if (f413_path_run_turn_velocity_from_code(codes[i], &shortestRunModeParams4, &v))
      assert(v == (codes[i] < 500 ? 1000 : 1400));
  }
  calls++;
}
int main(void)
{
  assert(shortestRunModeParams4.fan_power > 0 && shortestRunModeParams4.fan_power <= 1000);
  assert(shortestRunModeParams2.fan_power == 0);
  for (selected_sub=0; selected_sub<10; selected_sub++) f413_mode4_run_case0_sub(selected_sub);
  assert(calls == 10);
  f413_mode4_run_case0_sub(10);
  assert(calls == 10);
  puts("mode4 suction: all 10 dispatch/preflight paths and turn speeds PASS");
  return 0;
}
