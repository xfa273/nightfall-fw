#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define NIGHTFALL_F413_PATH_LINEAR_PLAN_HOST_TEST (1U)
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_path_run.c"
#if TEST_MODE == 3
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_mode3.c"
#define MODE_PARAMS shortestRunModeParams3
#define CASE_PARAMS shortestRunCaseParamsMode3
#define RUN_SUB f413_mode3_run_case0_sub
#elif TEST_MODE == 4
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_mode4.c"
#define MODE_PARAMS shortestRunModeParams4
#define CASE_PARAMS shortestRunCaseParamsMode4
#define RUN_SUB f413_mode4_run_case0_sub
#elif TEST_MODE == 5
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_mode5.c"
#define MODE_PARAMS shortestRunModeParams5
#define CASE_PARAMS shortestRunCaseParamsMode5
#define RUN_SUB f413_mode5_run_case0_sub
#elif TEST_MODE == 6
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_mode6.c"
#define MODE_PARAMS shortestRunModeParams6
#define CASE_PARAMS shortestRunCaseParamsMode6
#define RUN_SUB f413_mode6_run_case0_sub
#elif TEST_MODE == 7
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_mode7.c"
#define MODE_PARAMS shortestRunModeParams7
#define CASE_PARAMS shortestRunCaseParamsMode7
#define RUN_SUB f413_mode7_run_case0_sub
#else
#error unsupported suction test mode
#endif
static const float small_speeds[] = {800,1000,1200,1200,1400};
static const float large_speeds[] = {1000,1400,1700,2000,2200};
static const float diagonal_speeds[] = {1000,1400,1500,1500,1500};
#define SMALL_SPEED small_speeds[TEST_MODE-3]
#define LARGE_SPEED large_speeds[TEST_MODE-3]
#define DIAGONAL_SPEED diagonal_speeds[TEST_MODE-3]

static unsigned calls, selected_sub;
#if !LEGACY_PROFILE
static const int fan_duties[] = {500,700,1000,1000,1000};
static unsigned connections;
static const uint16_t turns[] = {300,400,501,601,502,602,701,702,703,704,801,802,901,902,903,904};
#endif
void f413_mode_shortest_run_case(uint8_t mode, uint8_t c) { (void)mode; (void)c; }
void f413_mode_shortest_run_config(const f413_shortest_case_config_t* c) { (void)c; }
static void check_path(const uint16_t* codes, unsigned count, unsigned c, bool walls, bool test)
{
  uint16_t terminated[20] = {0};
  memcpy(terminated, codes, count * sizeof(*codes));
  const ShortestRunCaseParams_t selected = f413_path_run_session_case_params(
      terminated, 20, &MODE_PARAMS, &CASE_PARAMS[c-1], test);
  const ShortestRunCaseParams_t* cp = &selected;
  const float initial = f413_path_run_boundary_speed(&MODE_PARAMS, cp, DIST_FIRST_SEC);
  f413_path_run_prepared_path_t prepared;
  f413_path_run_preflight_result_t r = f413_path_run_preflight_prepare(
      terminated, 20, &MODE_PARAMS, cp, initial, walls, test, &prepared);
  if (r.status != F413_PATH_RUN_PREFLIGHT_OK)
  {
    fprintf(stderr,"sub%u case%u walls%u test%u preflight=%u index=%zu code=%u path:",
            selected_sub,c,walls,test,r.status,r.index,r.code);
    for (unsigned i=0; i<count; ++i) fprintf(stderr," %u", codes[i]);
    fputc('\n',stderr);
  }
  assert(r.status == F413_PATH_RUN_PREFLIGHT_OK && prepared.count > 0);
  if (test)
  {
    const float expected = selected_sub == 0 ? SMALL_SPEED :
        selected_sub >= 3 && selected_sub <= 7 ? DIAGONAL_SPEED :
        selected_sub <= 2 ? LARGE_SPEED : CASE_PARAMS[c-1].velocity_straight;
    assert(cp->velocity_straight == expected);
    float peak = initial;
    for (size_t a=0; a<prepared.count; ++a)
    {
      const f413_path_run_prepared_linear_t* action = &prepared.actions[a];
      assert(action->entry_velocity_mm_s <= expected);
      assert(action->exit_velocity_mm_s <= expected);
      for (unsigned phase=0; phase<action->phase_count; ++phase)
      {
        float v = action->phase_exit_velocity_mm_s[phase];
        assert(v <= expected);
        if (v > peak) peak = v;
      }
    }
    assert(peak == expected); /* Enough run-up, no overspeed before/after turns. */
  }
  for (unsigned i=0; i<count; ++i)
  {
    float v;
    if (f413_path_run_turn_velocity_from_code(codes[i], &MODE_PARAMS, &v))
      assert(v == (codes[i] < 500 ? SMALL_SPEED : codes[i] < 700 ? LARGE_SPEED : DIAGONAL_SPEED));
  }
}
void f413_mode_shortest_run_case0_path(const char* label, uint8_t mode, uint8_t c,
                                     const uint16_t* codes, uint16_t count)
{
  (void)label;
  assert(mode == TEST_MODE && c >= 1 && c <= 9);
#if LEGACY_PROFILE
  assert(c == k_case0_subs[selected_sub].case_index);
  assert(codes == k_case0_subs[selected_sub].codes);
  assert(count == k_case0_subs[selected_sub].code_count);
#else
  assert(c == (selected_sub == 0 || selected_sub == 8 ? 1 :
               selected_sub <= 2 ? 2 : selected_sub == 9 ? 5 : 8));
  check_path(codes,count,c,false,true);
#endif
  calls++;
}
static bool needs_diagonal(uint16_t code)
{
  return f413_path_run_turn_exits_diagonal(code) || code == 801 || code == 802;
}
static bool leaves_diagonal(uint16_t code)
{
  return f413_path_run_turn_enters_diagonal(code) || code == 801 || code == 802;
}
int main(void)
{
#if LEGACY_PROFILE
  assert(MODE_PARAMS.fan_power == 0);
  for (selected_sub=0; selected_sub<10; ++selected_sub) RUN_SUB(selected_sub);
  assert(calls == 10); RUN_SUB(10); assert(calls == 10);
  printf("mini_r2 mode%u: all 10 legacy case0 dispatch paths preserved PASS\n",TEST_MODE);
#else
  assert(MODE_PARAMS.fan_power == fan_duties[TEST_MODE-3]);
  assert(MODE_PARAMS.turn_omega_max == 3000);
  for (selected_sub=0; selected_sub<10; ++selected_sub) RUN_SUB(selected_sub);
  assert(calls == 10); RUN_SUB(10); assert(calls == 10);
  for (unsigned c=1; c<=9; ++c)
  {
    const ShortestRunCaseParams_t* cp = &CASE_PARAMS[c-1];
    const float base_v = c == 1 ? SMALL_SPEED : LARGE_SPEED;
    const float base_a = ceilf(base_v * base_v / (2 * DIST_HALF_SEC) / 1000) * 1000;
    const unsigned step = c <= 2 ? 0 : c - 2;
    assert(cp->velocity_straight == base_v + 200 * step);
    assert(cp->acceleration_straight == base_a + 1000 * step);
    assert(cp->acceleration_straight_dash == cp->acceleration_straight);
    assert(cp->velocity_d_straight == DIAGONAL_SPEED + 100 * step);
    assert(cp->acceleration_d_straight == ceilf(DIAGONAL_SPEED * DIAGONAL_SPEED /
        (2 * DIST_HALF_SEC) / 1000) * 1000 + 1000 * step);
    assert(cp->acceleration_d_straight_dash == cp->acceleration_d_straight);
    if (c <= 2)
    {
      /* The next lower 1 m/s^2 step cannot stop this turn within half a cell. */
      ShortestRunCaseParams_t insufficient = *cp;
      insufficient.acceleration_straight -= 1000;
      insufficient.acceleration_straight_dash -= 1000;
      const uint16_t path[] = {203,c == 1 ? 300 : 501,0};
      const float initial = f413_path_run_boundary_speed(&MODE_PARAMS,&insufficient,DIST_FIRST_SEC);
      assert(f413_path_run_preflight(path,3,&MODE_PARAMS,&insufficient,initial,false,false).status
             == F413_PATH_RUN_PREFLIGHT_INFEASIBLE_LINEAR);
    }
    for (unsigned wall=0; wall<2; ++wall)
      for (unsigned a=0; a<sizeof(turns)/sizeof(turns[0]); ++a)
      {
        if (c == 1 && turns[a] >= 500) continue; /* case1 builds small90 only. */
        uint16_t path[20] = {201}; unsigned n=1;
        if (needs_diagonal(turns[a])) { path[n++]=701; path[n++]=1001; }
        path[n++]=turns[a];
        if (leaves_diagonal(turns[a])) { path[n++]=1001; path[n++]=703; }
        /* Earliest S1 launch, then stop directly in the implicit 45 mm tail. */
        check_path(path,n,c,wall,false); connections++;
        for (unsigned b=0; b<sizeof(turns)/sizeof(turns[0]); ++b)
        {
          if (c == 1 && turns[b] >= 500) continue;
          if (leaves_diagonal(turns[a]) != needs_diagonal(turns[b])) continue;
          n=1;
          if (needs_diagonal(turns[a])) { path[n++]=701; path[n++]=1001; }
          path[n++]=turns[a];
          path[n++]=leaves_diagonal(turns[a]) ? 1001 : 201;
          path[n++]=turns[b];
          if (leaves_diagonal(turns[b])) { path[n++]=1001; path[n++]=703; }
          check_path(path,n,c,wall,false); connections++;
        }
      }
  }
  printf("mode%u suction: all 10 case0 paths and peak speeds, %u minimal start/connector/stop paths across 9 cases PASS\n",TEST_MODE,connections);
#endif
  return 0;
}
