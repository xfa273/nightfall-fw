#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define NIGHTFALL_F413_PATH_LINEAR_PLAN_HOST_TEST (1U)
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_path_run.c"
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_mode6.c"

static unsigned calls, selected_sub, connections;
static const uint16_t turns[] = {300,400,501,601,502,602,701,702,703,704,801,802,901,902,903,904};
void f413_mode_shortest_run_case(uint8_t mode, uint8_t c) { (void)mode; (void)c; }
void f413_mode_shortest_run_config(const f413_shortest_case_config_t* c) { (void)c; }
static void check_path(const uint16_t* codes, unsigned count, unsigned c, bool walls, bool test)
{
  uint16_t terminated[20] = {0};
  memcpy(terminated, codes, count * sizeof(*codes));
  const ShortestRunCaseParams_t* cp = &shortestRunCaseParamsMode6[c-1];
  const float initial = sqrtf(2 * cp->acceleration_straight * DIST_FIRST_SEC);
  f413_path_run_prepared_path_t prepared;
  f413_path_run_preflight_result_t r = f413_path_run_preflight_prepare(
      terminated, 20, &shortestRunModeParams6, cp, initial, walls, test, &prepared);
  if (r.status != F413_PATH_RUN_PREFLIGHT_OK)
  {
    fprintf(stderr,"sub%u case%u walls%u test%u preflight=%u index=%zu code=%u path:",
            selected_sub,c,walls,test,r.status,r.index,r.code);
    for (unsigned i=0; i<count; ++i) fprintf(stderr," %u", codes[i]);
    fputc('\n',stderr);
  }
  assert(r.status == F413_PATH_RUN_PREFLIGHT_OK && prepared.count > 0);
  for (unsigned i=0; i<count; ++i)
  {
    float v;
    if (f413_path_run_turn_velocity_from_code(codes[i], &shortestRunModeParams6, &v))
      assert(v == (codes[i] < 500 ? 1200 : 1500));
  }
}
void f413_mode_shortest_run_case0_path(const char* label, uint8_t mode, uint8_t c,
                                     const uint16_t* codes, uint16_t count)
{
  (void)label;
  assert(mode == 6 && c >= 1 && c <= 9);
  check_path(codes,count,c,false,true);
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
  assert(shortestRunModeParams6.fan_power > 0 && shortestRunModeParams6.fan_power <= 1000);
  for (selected_sub=0; selected_sub<10; ++selected_sub) f413_mode6_run_case0_sub(selected_sub);
  assert(calls == 10); f413_mode6_run_case0_sub(10); assert(calls == 10);
  for (unsigned c=1; c<=9; ++c)
  {
    const ShortestRunCaseParams_t* cp = &shortestRunCaseParamsMode6[c-1];
    assert(cp->velocity_straight == 1500 && cp->velocity_d_straight == 1500);
    assert(cp->acceleration_straight == 25000 && cp->acceleration_straight_dash == 25000);
    assert(cp->acceleration_d_straight == 25000 && cp->acceleration_d_straight_dash == 25000);
    for (unsigned wall=0; wall<2; ++wall)
      for (unsigned a=0; a<sizeof(turns)/sizeof(turns[0]); ++a)
      {
        uint16_t path[20] = {201}; unsigned n=1;
        if (needs_diagonal(turns[a])) { path[n++]=701; path[n++]=1001; }
        path[n++]=turns[a];
        if (leaves_diagonal(turns[a])) { path[n++]=1001; path[n++]=703; }
        /* Earliest S1 launch, then stop directly in the implicit 45 mm tail. */
        check_path(path,n,c,wall,false); connections++;
        for (unsigned b=0; b<sizeof(turns)/sizeof(turns[0]); ++b)
        {
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
  printf("mode6 suction: all 10 case0 paths, 1200/1500 speeds, %u minimal start/connector/stop paths across 9 cases PASS\n",connections);
  return 0;
}
