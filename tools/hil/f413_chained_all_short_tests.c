#define ALL_SHORT_WALL_TESTS
#include "f413_chained_short_wall_tests.c"

static bool slow_fall;
static bool normal_snapshot(f413_wall_sensor_snapshot_t* out)
{
  sample.sample_sequence = tick / 2U;
  sample.r_delta = slow_fall ? (int)fmaxf(40, 600 - (float)tick * 5) : tick < 12 ? 600 : 40;
  sample.l_delta = 600;
  *out = sample; return true;
}
static void normal_setup(bool slow)
{
  setup(); slow_fall = slow;
  const f413_wall_runtime_config_t config = {normal_snapshot, HAL_Delay, HAL_GetTick, 40, 20, 4};
  f413_wall_runtime_config(&config);
  f413_wall_runtime_set_wall_end_thresholds(100,1,100,1);
}
static void normal_paths(void)
{
  for (unsigned slow = 0; slow < 2; slow++)
  {
    normal_setup(slow); ShortestRunModeParams_t mode = shortestRunModeParams4;
    mode.velocity_turn90 = 1000; mode.dist_wall_end = -23; mode.dist_wall_end_short_add = 10;
    ShortestRunCaseParams_t cp = {0};
    f413_path_run_prepared_linear_t prepared = {.entry_velocity_mm_s=1000,
        .exit_velocity_mm_s=f413_path_run_next_straight_exit_velocity(300,&mode,&cp)};
    f413_run_session_guard_t guard = {0}; float speed = 1000;
    assert(f413_path_run_run_straight_steps(2,0,300,&mode,&cp,&prepared,&speed,&guard,4) == 0);
    float hit, unused; assert(f413_wall_runtime_wall_end_detected(&hit,&unused));
    const bool fast = !slow && F413_MOTION_ENABLED(F413_MOTION_SHORT_WALL_END_ALL);
    assert(f413_wall_runtime_wall_end_detected_by_short() == fast);
    near(position,hit+22+(fast?10:0));
    if (fast) { near(hit,14); assert(hit<20); }
    assert(!g_wall_end_gate_active);
  }
  normal_setup(false); f413_run_features_t f = f413_run_features_get();
  f.wall_end_correction_enabled = false; f413_run_features_set(&f);
  float speed=1000;bool found=true;f413_run_session_guard_t guard={0};
  assert(f413_path_run_drive_wallend_segment(45,1000,&speed,&guard,4,&found)==0);
  near(position,45);assert(!found && !short_hit());
}
static void gate_lifecycle(void)
{
  setup();
  for (unsigned t=0;t<=8;t+=2) {tick=t;sample.sample_sequence++;f413_wall_runtime_poll_straight(true);}
  assert(!short_hit() && !g_short_end.active);
  const unsigned saved = g_short_end.count;
  f413_wall_runtime_end_begin();
  if (F413_MOTION_ENABLED(F413_MOTION_SHORT_WALL_END_ALL)) assert(saved>0 && saved==g_short_end.count);
  else assert(!g_short_end.count);
  observe(10,400,600,true); assert(!short_hit());
  /* Closing/reopening a gate discards the previous confirmation, even when
   * no unarmed poll ran between them and history is still fresh. */
  f413_wall_runtime_control_apply(false);
  observe(12,200,600,true);assert(!short_hit());
  observe(14,40,600,true);
  assert(short_hit()==F413_MOTION_ENABLED(F413_MOTION_SHORT_WALL_END_ALL));
  if (short_hit()) {
    assert(f413_wall_runtime_wall_end_detected_by_short());
    g_wall_end.detected_l=true; assert(!f413_wall_runtime_wall_end_detected_by_short());
  }
  f413_wall_runtime_end_begin(); assert(!g_wall_end.detected_r && !g_wall_end.detected_l);
  observe(16,40,600,true);assert(!short_hit());
  setup();f413_run_features_t f=f413_run_features_get();f.test_mode_run=true;f413_run_features_set(&f);
  for(unsigned t=0;t<=20;t+=2) observe(t,t<12?600:40,600,true);
  assert(!short_hit());
}
static void diagnostic(void)
{
  normal_setup(false); f413_wall_runtime_run_end_monitor_once();
  assert(short_hit()==F413_MOTION_ENABLED(F413_MOTION_SHORT_WALL_END_ALL));
  assert(!g_wall_end_gate_active && !g_short_end.active);
}
int main(void)
{
  normal_paths();gate_lifecycle();diagnostic();
  puts("PASS: all wall-end gates, unarmed history, reopening/clearing, normal approach short-only +10mm, slow legacy fallback, OFF/test/r2 and diagnostic cadence");
}
