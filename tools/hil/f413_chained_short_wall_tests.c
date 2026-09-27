/* Actual wall runtime and path executor; no MCU or device I/O. */
#define CHAINED_REAL_WALL_RUNTIME
#define main chained_front_regression_main
#include "f413_chained_front_tests.c"
#undef main
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_wall_runtime.c"

static f413_wall_sensor_snapshot_t sample;
static bool valid_sample, scripted;
static uint32_t expected_core_end;
static unsigned prepare_reads;
static float heading_peak;
void f413_ctrl_set_heading_omega_correction(float value)
{ heading_peak = fmaxf(heading_peak, fabsf(value)); }
void f413_wall_sensor_get_control_base(uint16_t* l, uint16_t* r, uint16_t* f)
{ if (l) *l = 600; if (r) *r = 600; if (f) *f = 1000; }
static bool snapshot(f413_wall_sensor_snapshot_t* out)
{
  if (!valid_sample) return false;
  if (scripted)
  {
    const uint32_t sampled_ms = tick / 2U * 2U;
    const int32_t relative = (int32_t)(sampled_ms - expected_core_end);
    sample.sample_sequence = tick / 2U;
    sample.l_delta = relative < -4 ? 700 : relative < 0 ? 650 :
                     relative < 4 ? 500 - relative * 60 :
                     relative < 8 ? 260 - (relative - 4) * 55 : 40;
    if (in_core)
    {
      assert(!g_short_end.active);
      assert((int32_t)(tick - expected_core_end) >= -8);
      prepare_reads++;
    }
  }
  *out = sample;
  return true;
}
static void setup(void)
{
  reset(); memset(&sample, 0, sizeof(sample));
  sample.r_delta = sample.l_delta = 600;
  valid_sample = true; scripted = false; prepare_reads = 0; heading_peak = 0;
  const f413_wall_runtime_config_t config = {snapshot, HAL_Delay, HAL_GetTick, 0, 0, 4};
  f413_wall_runtime_config(&config);
  f413_wall_runtime_set_wall_end_thresholds(100, 1, 100, 1);
  f413_wall_runtime_set_control_gains(2, 0);
  f413_wall_runtime_chained_prepare_begin();
}
static bool observe(uint32_t ms, int r, int l, bool active)
{
  tick = ms; sample.sample_sequence++; sample.r_delta = r; sample.l_delta = l;
  position = (float)ms;
  if (active) return f413_wall_runtime_poll_wall_end_with_control(false);
  f413_wall_runtime_chained_prepare_sample(); return false;
}
static void warm(uint32_t start, int r, int l)
{
  for (unsigned i = 0; i <= 8; i += 2) observe(start + i, r, l, false);
  f413_wall_runtime_chained_monitor_begin();
}
static bool short_hit(void) { return g_wall_end.short_detected_r || g_wall_end.short_detected_l; }

static void boundaries(void)
{
  for (unsigned side = 0; side < 2; side++)
  for (unsigned wrap = 0; wrap < 2; wrap++)
  {
    setup(); const uint32_t start = wrap ? UINT32_MAX - 8U : 0U;
    warm(start, 600, 600);
    assert(!observe(start + 10U, side ? 600 : 400, side ? 400 : 600, true));
    for (unsigned i = 0; i < 20; i++)
      assert(!f413_wall_runtime_poll_wall_end_with_control(false));
    assert(!short_hit()); /* Re-reading the first falling sample is not confirmation. */
    assert(observe(start + 12U, side ? 600 : 200, side ? 200 : 600, true));
    assert(side ? g_wall_end.short_detected_l : g_wall_end.short_detected_r);
    assert(heading_peak == 0);
    nvm_trace_log_record_t record = {0};
    assert(f413_wall_runtime_fill_observe(&record, 0));
    assert(record.reserved_u16_0 & 0x0800);
    assert(record.reserved_u16_0 & (side ? 0x2000 : 0x1000));
    assert(record.reserved_u16_0 & (side ? 0x0080 : 0x0040));
    /* The long derivative field retains its original units. */
    assert((side ? record.reserved_i32_3 : record.reserved_i32_2) > -200);
    f413_wall_runtime_chained_monitor_end();
    f413_wall_runtime_control_apply(false);
    assert(f413_wall_runtime_fill_observe(&record, 0));
    assert(!(record.reserved_u16_0 & 0x0900));
    assert(record.reserved_u16_0 & (side ? 0x2000 : 0x1000));
    f413_wall_runtime_end_clear();
    assert(!short_hit() && !g_short_end.active);
  }
  /* Strict drop >150, past >=300. */
  for (int drop = 150; drop <= 151; drop++)
  {
    setup(); warm(0, 600, 600);
    observe(10, 600, 600 - drop, true); observe(12, 600, 600 - drop, true);
    assert(short_hit() == (drop == 151));
  }
  for (int past = 299; past <= 300; past++)
  {
    setup(); warm(0, 600, past);
    observe(10, 600, 100, true); observe(12, 600, 100, true);
    assert(short_hit() == (past == 300));
  }
}
static void reject_invalid(void)
{
  /* Flat, rising, slow drift, and one-sample pulse. */
  for (unsigned kind = 0; kind < 4; kind++)
  {
    setup(); warm(0, 600, 600);
    for (unsigned t = 10; t <= 40; t += 2)
    {
      const int l = kind == 0 ? 600 : kind == 1 ? 600 + (int)t * 5 :
                    kind == 2 ? 600 - ((int)t - 8) * 5 : t == 10 ? 200 : 600;
      observe(t, 600, l, true); assert(!short_hit());
    }
  }
  /* Nothing before the gate, including completed earlier edges, is a hit. */
  setup();
  for (unsigned t = 0; t <= 12; t += 2) observe(t, 600, t < 4 ? 600 : 40, false);
  assert(!short_hit()); f413_wall_runtime_chained_monitor_begin();
  observe(14, 600, 40, true); observe(16, 600, 40, true); assert(!short_hit());
  /* A warmup observation of a fall cannot count as the first gated update. */
  setup(); observe(0,600,600,false); observe(2,600,600,false); observe(4,600,400,false);
  f413_wall_runtime_chained_monitor_begin();
  assert(!f413_wall_runtime_poll_wall_end_with_control(false));
  observe(6,600,200,true); assert(!short_hit());

  for (unsigned kind = 0; kind < 4; kind++)
  {
    setup(); warm(0,600,600); observe(10,600,400,true);
    if (kind == 0) { valid_sample = false; f413_wall_runtime_poll_wall_end_with_control(false); valid_sample = true; }
    if (kind == 1) { sample.saturated = true; f413_wall_runtime_poll_wall_end_with_control(false); sample.saturated = false; }
    if (kind == 2) { tick = 14; f413_wall_runtime_poll_wall_end_with_control(false); }
    observe(kind >= 2 ? 16 : 12,600,200,true);
    assert(!short_hit()); /* Invalid/stale/long-gap update discarded history and count. */
  }
  /* Irregular 1/3 ms spacing brackets t-4ms instead of treating two polls as 4ms. */
  setup(); observe(0,600,600,false);observe(1,600,600,false);observe(4,600,600,false);
  f413_wall_runtime_chained_monitor_begin();
  observe(7,600,300,true);assert(!short_hit());observe(8,600,200,true);assert(short_hit());
}
static void executor(void)
{
  for (unsigned bypass = 0; bypass < 4; bypass++)
  {
    setup(); scripted = true;
    f413_path_run_turn_t previous = test_turn(), next = test_turn();
    previous.front_wall_entry = next.front_wall_entry = false;
    previous.dist_out_mm = 13; next.dist_in_mm = 1;
    const f413_path_run_smooth_turn_t p = f413_path_run_build_smooth_turn(
        previous.signed_angle_deg, previous.alpha_deg_s2, previous.omega_max_deg_s);
    expected_core_end = (uint32_t)ceilf(p.t_total_s * 1000.0f);
    f413_run_features_t features = f413_run_features_get();
    if (bypass == 1) features.wall_end_correction_enabled = false;
    if (bypass == 2) features.test_mode_run = true;
    f413_run_features_set(&features);
    bool consumed = false;
    assert(run_turn(&previous, bypass == 3 ? NULL : &next, true, &consumed) == 0);
    const bool enabled = F413_MOTION_ENABLED(F413_MOTION_CHAINED_SHORT_WALL_END) && bypass == 0;
    assert((prepare_reads > 0) == enabled);
    assert(!g_short_end.active && g_short_end.count == 0);
    if (enabled)
    {
      assert(consumed && g_wall_end.short_detected_l);
      assert(position - core_end[0] < 8); /* Short enough to matter in a 14mm connector. */
      const float endpoint = position; scripted = false;
      assert(run_turn(&next, NULL, consumed, &consumed) == 0);
      near(core_start[1], endpoint); /* Entry already consumed, no added millimetre. */
    }
  }
  setup(); scripted = true; expected_core_end = 1000; abort_core = 1;
  f413_path_run_turn_t previous = test_turn(), next = test_turn();bool consumed = false;
  injected_abort = F413_RUN_SESSION_ABORT_SWITCH;
  assert(run_turn(&previous,&next,true,&consumed) != F413_RUN_SESSION_ABORT_NONE);
  assert(!g_short_end.active && !g_short_end.count && !consumed);
}
static void recorded_waveforms(void)
{
  /* Actual saved left ADC at 4ms intervals around the same chained R/L180.
   * Intermediate values are an offline interpolation, not new measurements.
   * End/core positions and three poll phases come from the 19:14/20:10 traces. */
  static const struct { int first, end, count; int adc[9]; } waves[] = {
    {-28,-13,9,{768,768,749,703,540,307,48,46,50}},
    {-26,-13,8,{792,776,783,742,605,329,54,45}},
    {-27,-12,8,{771,757,731,691,567,333,50,38}},
    {-27,-11,8,{714,693,640,611,422,244,41,46}},
  };
  for (unsigned w = 0; w < sizeof(waves)/sizeof(waves[0]); w++)
  for (int phase = 0; phase <= 2; phase++)
  {
    setup(); const int gate = waves[w].end + phase; int detected = 999;
    for (int t = gate - 8; t < 0; t += 2)
    {
      const int offset = t - waves[w].first;
      assert(offset >= 0 && offset / 4 + 1 < waves[w].count);
      const int i = offset / 4;
      const int value = (int)lroundf((float)waves[w].adc[i] +
          .25f * (float)(offset % 4) * (float)(waves[w].adc[i + 1] - waves[w].adc[i]));
      if (t == gate) f413_wall_runtime_chained_monitor_begin();
      observe((uint32_t)(100 + t),600,value,t >= gate);
      if (short_hit() && detected == 999) detected = t;
    }
    assert(g_wall_end.short_detected_l && !g_wall_end.short_detected_r);
    assert(detected <= -6 && detected >= -10);
    if (w >= 2) assert(detected <= -7 && detected >= -9);
  }
}
int main(void)
{
  boundaries(); reject_invalid(); executor(); recorded_waveforms();
  puts("PASS: short 4ms derivative, thresholds, both sides, post-gate confirmations, wrap/jitter/stale/noise, trace tags/path lifecycle; 4 recorded waves x 3 phases, latest detection 7-9ms before original core");
}
