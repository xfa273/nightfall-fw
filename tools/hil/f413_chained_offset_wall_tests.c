/* Real wall-end derivative + real connector; synthetic 2 ms optical samples. */
#define CHAINED_REAL_WALL_RUNTIME
#define main chained_front_regression_main
#include "f413_chained_front_tests.c"
#undef main
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_wall_runtime.c"

static bool drop_left, valid_snapshot;
static float edge_mm, heading_peak;
static unsigned sensor_period_ms;
void f413_ctrl_set_heading_omega_correction(float v)
{ if (fabsf(v) > heading_peak) heading_peak = fabsf(v); }
void f413_wall_sensor_get_control_base(uint16_t* l, uint16_t* r, uint16_t* f)
{ if (l) *l = 600; if (r) *r = 600; if (f) *f = 1000; }
static bool snapshot(f413_wall_sensor_snapshot_t* out)
{
  if (!valid_snapshot) return false;
  memset(out, 0, sizeof(*out));
  out->sample_sequence = tick / sensor_period_ms;
  const uint32_t sampled_ms = (tick / sensor_period_ms) * sensor_period_ms;
  out->r_delta = 600; out->l_delta = 600;
  if ((float)sampled_ms >= edge_mm)
  {
    if (drop_left) out->l_delta = 40;
    else out->r_delta = 40;
  }
  out->right_wall = out->r_delta > 100;
  out->left_wall = out->l_delta > 100;
  return true;
}
static void setup(void)
{
  reset(); cores = 1; velocity = step_velocity = 1000;
  valid_snapshot = true; heading_peak = 0; sensor_period_ms = 2;
  const f413_wall_runtime_config_t config = {snapshot, HAL_Delay, HAL_GetTick, 0, 0, 4};
  f413_wall_runtime_config(&config);
  f413_wall_runtime_set_control_gains(2, 0);
  f413_wall_runtime_set_wall_end_thresholds(100, 1, 100, 1);
}
int main(void)
{
  for (unsigned side = 0; side < 2; ++side)
  for (unsigned control = 0; control < 2; ++control)
  {
    setup(); drop_left = side != 0; edge_mm = 12;
    f413_path_run_turn_t previous = test_turn(), next = test_turn();
    previous.front_wall_entry = next.front_wall_entry = false;
    previous.wall_control_offsets = next.wall_control_offsets = control != 0;
    previous.dist_out_mm = 13; next.dist_in_mm = 20;
    float speed = 1000; bool consumed; f413_run_session_guard_t guard = {0};
    assert(f413_path_run_drive_chained_offsets(&previous, &next, -23, &speed,
        &guard, 4, &consumed) == 0);
    assert(consumed); near(position, 20); /* Two distinct updates below -200. */
    assert(drop_left ? g_wall_end.detected_l : g_wall_end.detected_r);
    assert((drop_left ? g_wall_end.detected_deriv_l : g_wall_end.detected_deriv_r) == -311);
    if (!control) assert(heading_peak == 0); /* Diagonal offsets never enable cardinal control. */
    nvm_trace_log_record_t rec = {0};
    assert(f413_wall_runtime_fill_observe(&rec, 0));
    assert(!(rec.reserved_u16_0 & 0x0100)); /* Gate off after connector. */
    assert(rec.reserved_u16_0 & (drop_left ? 0x0080 : 0x0040));
    /* The latch belongs to this connector only. A subsequent low, unchanged
     * wall sample must not cut the next pair short. */
    assert(f413_path_run_drive_chained_offsets(&previous, &next, -23, &speed,
        &guard, 4, &consumed) == 0);
    assert(consumed); near(position, 53);
    assert(!g_wall_end.detected_l && !g_wall_end.detected_r);

    setup(); drop_left = side != 0; edge_mm = 12;
    bool detected = true;
    assert(f413_path_run_drive_wallend_segment(13, 1000, &speed, &guard, 4, &detected) == 0);
    assert(!detected); /* The old exit-only path misses the same edge. */
  }
  setup(); drop_left = true; edge_mm = 0;
  f413_wall_runtime_end_clear();
  /* Seed high, then reuse the same low snapshot: duplicate polls are not confirmations. */
  edge_mm = 2; assert(!f413_wall_runtime_poll_wall_end_with_control(false));
  tick = 2;
  for (unsigned i = 0; i < 100; ++i)
    assert(!f413_wall_runtime_poll_wall_end_with_control(false));
  assert(!g_wall_end.detected_l);
  nvm_trace_log_record_t rec = {0}; assert(f413_wall_runtime_fill_observe(&rec, 0));
  assert(rec.reserved_u16_0 & 0x0100); assert(!(rec.reserved_u16_0 & 0x0200));
  valid_snapshot = false;
  assert(!f413_wall_runtime_poll_wall_end_with_control(false));
  assert(!g_wall_end_gate_active);
  /* Extending 13 mm by a 1 mm next entry cannot catch an arbitrarily late
   * edge. No hit must still finish at the combined endpoint, without creep. */
  setup(); edge_mm = 12;
  f413_path_run_turn_t previous = test_turn(), next = test_turn();
  previous.front_wall_entry = next.front_wall_entry = false;
  previous.dist_out_mm = 13; next.dist_in_mm = 1;
  float speed = 1000; bool consumed; f413_run_session_guard_t guard = {0};
  assert(f413_path_run_drive_chained_offsets(&previous, &next, -23, &speed,
      &guard, 4, &consumed) == 0);
  assert(consumed); near(position, 14);
  assert(!g_wall_end.detected_l && !g_wall_end.detected_r);
  puts("PASS: real derivative across exit/entry boundary, both sides, High/Low auxiliary, duplicate samples, missing sensor, detection-only gate/control separation");
}
