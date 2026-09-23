#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct TIM_HandleTypeDef TIM_HandleTypeDef;
#include <stdint.h>
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#include "params.h"
/* Exercise both boot-selected settings without duplicating the session. */
static uint32_t test_auto_video_capture;
#undef ENABLE_AUTO_VIDEO_CAPTURE
#define ENABLE_AUTO_VIDEO_CAPTURE test_auto_video_capture
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_path_run.c"
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_run_features.c"

uint16_t path[ROUTE_MAX_LEN];
typedef enum { LEAD, RAMP, SPINUP, DRIVE, CLEANUP } phase_t;
static uint32_t tick, control_tick, fan_tick, full_duty_tick, first_drive_tick;
static unsigned starts, fan_starts, fan_stops, trace_stops, profiles, duty_updates;
static unsigned completed_sessions;
static unsigned video_starts, video_stops, control_stops;
static uint32_t trace_start_tick;
static unsigned wait_extra_ms;
static uint16_t fan_duty, requested_duty;
static uint32_t expected_ramp_ms;
static uint16_t flags;
static uint8_t selected_mode, selected_case;
static bool running, fan, tracing, pressed, fan_fail, stuck, holding, cleanup;
static bool suction, disturbed, duty_fail;
static bool stop_profile_done;
static uint32_t stop_profile_finish_tick;
static float position, velocity, target, angle, omega, peak_command;
static f413_run_session_abort_reason_t injected_abort;
static phase_t abort_phase;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
int trace_printf(const char* fmt, ...)
{
  if (strstr(fmt,"[RUN-TEST] path end (") != NULL) completed_sessions++;
  return 0;
}
bool f413_hw_stop_switch_pressed(void) { return pressed; }
bool f413_hw_fan_start(uint16_t duty)
{
  assert(running && holding && duty == 1 && profiles == 0);
  assert(tick - control_tick >= F413_PATH_RUN_SUCTION_CONTROL_LEAD_MS);
  assert(velocity == 0 && target == 0);
  fan_starts++; fan_tick = tick; fan_duty = duty; fan = !fan_fail; return fan;
}
bool f413_hw_fan_set_duty(uint16_t duty)
{
  assert(running && holding && fan && profiles == 0);
  assert(duty >= fan_duty && duty <= requested_duty);
  uint32_t elapsed = tick - fan_tick;
  uint16_t expected = elapsed >= expected_ramp_ms ? requested_duty :
      (uint16_t)((requested_duty * elapsed + expected_ramp_ms - 1) / expected_ramp_ms);
  assert(duty == expected);
  if (duty_fail) return false;
  duty_updates++; fan_duty = duty;
  if (duty == requested_duty)
  {
    assert(elapsed >= expected_ramp_ms);
    assert(elapsed < expected_ramp_ms + F413_PATH_RUN_SUCTION_RAMP_STEP_MS + wait_extra_ms);
    full_duty_tick = tick;
  }
  return true;
}
void f413_hw_fan_stop(void) { assert(!running); fan_stops++; fan = false; }
void f413_hw_emit_video_sync_start_pattern(void)
{ assert(ENABLE_AUTO_VIDEO_CAPTURE && !running && !fan); video_starts++; tick += 9750U; }
void f413_hw_emit_video_sync_stop_pattern(void)
{ assert(ENABLE_AUTO_VIDEO_CAPTURE && !running && !fan); video_stops++; tick += 9750U; }
void f413_ctrl_start(void)
{
  assert(!fan && !running); /* Includes the blocking IMU calibration. */
  if (suction) assert(flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG);
  tick += 1200; control_tick = tick; starts++; running = true;
  position = velocity = target = angle = omega = 0;
}
void f413_ctrl_stop(void) { running = false; control_stops++; }
void f413_ctrl_set_velocity(float v)
{
  velocity = v;
  if (v > peak_command) peak_command = v;
  if (v == 0) cleanup = true;
}
void f413_ctrl_set_velocity_profile(float start, float end, float distance)
{
  (void)start;
  stop_profile_done = false;
  if (profiles++ == 0)
  {
    first_drive_tick = tick;
    assert(starts == 1 && running && fan == suction);
    if (suction)
    {
      assert(tick - fan_tick >= expected_ramp_ms + 300U);
      assert(fan_duty == requested_duty && duty_updates > 0);
      assert(tick - full_duty_tick == 300U + wait_extra_ms);
      if (disturbed) assert(angle == 2.655f); /* Preserve the measured start origin. */
    }
  }
  target = position + distance; velocity = end;
  if (start > peak_command) peak_command = start;
  if (end > peak_command) peak_command = end;
}
void f413_ctrl_set_omega(float v) { (void)v; }
void f413_ctrl_start_omega_profile(float p, float a, float c) { (void)p; (void)a; (void)c; }
void f413_ctrl_stop_omega_profile(void) {}
void f413_ctrl_reset_angle(void) { assert(!suction || profiles > 0); angle = 0; }
void f413_ctrl_set_angle_target(float v) { assert(running && !fan && v == 0); holding = true; }
void f413_ctrl_clear_angle_target(void) { holding = false; }
float f413_ctrl_get_distance(void) { return position; }
float f413_ctrl_get_angle(void) { return angle; }
float f413_ctrl_get_real_omega(void) { return omega; }
float f413_ctrl_get_real_velocity(void) { return velocity; }
bool f413_ctrl_stop_profile_complete(void) { return stop_profile_done; }
bool f413_trace_log_auto_is_enabled(void) { return tracing; }
void f413_trace_log_auto_start(void) { tracing = true; trace_start_tick = tick; }
void f413_trace_log_auto_step(void) {}
void f413_trace_log_set_mode_flags(uint16_t f) { flags = f; }
void f413_trace_log_auto_stop_after_tail(uint32_t ms)
{ (void)ms; assert(!fan && !running); tracing = false; trace_stops++; }
bool f413_run_session_guard_prepare(f413_run_session_guard_t* g) { memset(g,0,sizeof(*g)); return true; }
void f413_run_session_guard_cleanup(f413_run_session_guard_t* g) { (void)g; }
f413_run_session_abort_reason_t f413_run_session_wait_with_auto_step_guarded(uint32_t ms, f413_run_session_guard_t* g)
{
  (void)g;
  /* Cover guarded polling overruns as well as exact millisecond waits. */
  tick += ms + wait_extra_ms;
  phase_t phase = cleanup ? CLEANUP : profiles ? DRIVE :
      !fan ? LEAD : fan_duty < requested_duty ? RAMP : SPINUP;
  if (phase == LEAD || phase == RAMP || phase == SPINUP)
  {
    assert(running && holding && profiles == 0);
    assert(flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG);
  }
  if (injected_abort && phase == abort_phase) return injected_abort;
  if (disturbed && (phase == RAMP || phase == SPINUP))
  {
    /* 22:07 trace at the old gate timeout: these observations must not add
     * another startup timeout. This is a sequencing regression, not a model
     * of the physical response to the new fan ramp.
     */
    angle = 2.655f; omega = -1.0f; position = .666f; velocity = -3.0f;
  }
  if (phase == DRIVE && !stuck) position = target;
  if (phase == DRIVE && stop_profile_finish_tick && tick >= stop_profile_finish_tick)
    stop_profile_done = true;
  return F413_RUN_SESSION_ABORT_NONE;
}
uint16_t f413_run_session_abort_reason_to_trace_flag(f413_run_session_abort_reason_t r) { return (uint16_t)r; }
const char* f413_run_session_abort_reason_to_text(f413_run_session_abort_reason_t r) { (void)r; return "injected"; }
void f413_wall_runtime_set_control_gains(float a, float b) { (void)a; (void)b; }
void f413_wall_runtime_set_wall_end_thresholds(uint16_t a, uint16_t b, uint16_t c, uint16_t d)
{ (void)a; (void)b; (void)c; (void)d; }
void f413_wall_runtime_reset_wall_end_thresholds(void) {}
void f413_wall_runtime_control_clear(void) {}
void f413_wall_runtime_end_clear(void) {}
void f413_wall_runtime_control_apply(bool b) { (void)b; }
void f413_wall_runtime_poll_diagonal(bool b) { (void)b; }
void f413_wall_runtime_poll_straight(bool b) { (void)b; }
bool f413_wall_runtime_poll_wall_end(bool b) { (void)b; return false; }
bool f413_wall_distance_front_unwarped_mm(float* out) { (void)out; return false; }
static void reset(void)
{
  tick = control_tick = fan_tick = full_duty_tick = first_drive_tick = 0;
  starts = fan_starts = fan_stops = trace_stops = profiles = duty_updates = flags = 0;
  completed_sessions = 0;
  video_starts = video_stops = control_stops = trace_start_tick = test_auto_video_capture = 0;
  selected_mode = 3; selected_case = 1; wait_extra_ms = 0; fan_duty = 0; requested_duty = 500; expected_ramp_ms = 600;
  running = fan = tracing = pressed = fan_fail = stuck = holding = cleanup = false;
  disturbed = duty_fail = false; suction = true;
  stop_profile_done = false; stop_profile_finish_tick = 0;
  position = velocity = target = angle = omega = peak_command = 0;
  injected_abort = F413_RUN_SESSION_ABORT_NONE; abort_phase = DRIVE;
  memset(path,0,sizeof(path)); path[0]=209;
  const f413_run_features_t features={false,false,false,true,true};
  f413_run_features_set(&features);
}
static void run(void)
{
  requested_duty = f413_path_run_mode_params(selected_mode)->fan_power;
  expected_ramp_ms = f413_path_run_suction_ramp_ms(requested_duty);
  f413_path_run_session_once(suction ? selected_mode : 2,selected_case,0,"host suction");
}
static void stopped(unsigned expected_fan_starts)
{
  assert(!running && !fan && !tracing);
  assert(fan_starts == expected_fan_starts && fan_stops == 1 && trace_stops == 1);
}
int main(void)
{
  /* OFF restores trace-only hooks; ON stops control before START and retains
   * the complete optical tokens and camera guard. Logging always runs. */
  for (unsigned enabled = 0; enabled <= 1; ++enabled)
  {
    reset(); test_auto_video_capture = enabled; running = true;
    f413_path_run_trace_on_run_start();
    assert(tracing && video_starts == enabled && control_stops == enabled);
    assert(running == !enabled && trace_start_tick == enabled * 10050U);
    f413_ctrl_stop();
    f413_path_run_trace_on_run_stop();
    assert(!tracing && trace_stops == 1 && video_stops == enabled);
    assert(tick == enabled * 19800U);
  }
  /* Both suction and fan-off sessions must differ only by camera timing.
   * Existing ramp/hold/cleanup assertions remain active for both settings. */
  for (unsigned use_fan = 0; use_fan <= 1; ++use_fan)
  {
    uint32_t manual_first_drive = 0, manual_finish = 0;
    for (unsigned enabled = 0; enabled <= 1; ++enabled)
    {
      reset(); suction = use_fan != 0; test_auto_video_capture = enabled;
      run();
      assert(!running && !fan && !tracing && trace_stops == 1 && starts == 1);
      assert(fan_starts == use_fan && completed_sessions == 1);
      assert(video_starts == enabled && video_stops == enabled);
      if (!enabled) { manual_first_drive = first_drive_tick; manual_finish = tick; }
      assert(first_drive_tick == manual_first_drive + enabled * 10050U);
      assert(tick == manual_finish + enabled * 19800U);
    }
  }
  puts("auto video: OFF trace-only hooks, ON standby/tokens/guard, suction and fan-off sessions PASS");
  /* No common angular ceiling; an explicit mode limit preserves tuned shapes. */
  f413_path_run_turn_t turn;
  ShortestRunModeParams_t mode = shortestRunModeParams6;
  mode.turn_omega_max = 0;
  assert(f413_path_run_turn_from_code(300, &mode, &turn));
  f413_path_run_smooth_turn_t p = f413_path_run_build_smooth_turn(
      turn.signed_angle_deg, turn.alpha_deg_s2, turn.omega_max_deg_s);
  assert(p.omega_peak_deg_s > 2200);
  mode.turn_omega_max = 2600;
  assert(f413_path_run_turn_from_code(300, &mode, &turn));
  p = f413_path_run_build_smooth_turn(turn.signed_angle_deg, turn.alpha_deg_s2, turn.omega_max_deg_s);
  assert(p.omega_peak_deg_s == 2600);
  mode.alpha_turn90 = 0;
  assert(!f413_path_run_turn_from_code(300, &mode, &turn));
  /* The user specifies a rate, not a fixed duration for every target. */
  const struct { uint16_t duty; uint32_t ms; } ramps[] = {
    {250, 300}, {500, 600}, {700, 840}, {750, 900}, {1000, 1200}
  };
  for (unsigned i = 0; i < sizeof(ramps) / sizeof(ramps[0]); ++i)
  {
    reset(); requested_duty = ramps[i].duty; expected_ramp_ms = ramps[i].ms;
    flags = NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG;
    f413_ctrl_start(); f413_ctrl_set_angle_target(0);
    f413_run_session_guard_t guard = {0};
    assert(f413_run_session_wait_with_auto_step_guarded(20, &guard) == F413_RUN_SESSION_ABORT_NONE);
    assert(f413_path_run_start_suction(requested_duty, &guard) == F413_RUN_SESSION_ABORT_NONE);
    assert(fan_duty == requested_duty && full_duty_tick - fan_tick == ramps[i].ms);
    f413_ctrl_stop(); f413_hw_fan_stop();
  }
  const float small[] = {800,1000,1200,1200,1400};
  const float large[] = {1000,1400,1700,2000,2200};
  const unsigned duties[] = {500,700,1000,1000,1000};
  for (unsigned m=3; m<=7; ++m)
  {
    reset(); selected_mode=m; run(); stopped(1);
    assert(starts == 1 && position > 400 && peak_command == small[m-3]);
    assert(requested_duty == duties[m-3]);
    /* Exercise all current cardinal trials through the production session. */
    for (unsigned t=0; t<3; ++t)
    {
      reset(); selected_mode=m; selected_case=t == 0 ? 1 : 2;
      path[0]=t == 0 ? 203 : t == 1 ? 204 : 206; path[1]=t == 0 ? 300 : t == 1 ? 501 : 502; path[2]=203;
      run(); stopped(1);
      assert(peak_command == (t == 0 ? small[m-3] : large[m-3]));
      assert(completed_sessions == 1 && profiles >= 6);
      assert(requested_duty == duties[m-3]);
    }
  }
  reset(); wait_extra_ms=3; run(); stopped(1); assert(starts == 1 && position > 400);
  reset(); tick=UINT32_MAX-1610U; run(); stopped(1); assert(position > 400);
  reset(); fan_fail=true; run(); stopped(1); assert(starts == 1 && profiles == 0);
  reset(); duty_fail=true; run(); stopped(1); assert(starts == 1 && profiles == 0);
  for (unsigned reason=F413_RUN_SESSION_ABORT_SWITCH; reason<=F413_RUN_SESSION_ABORT_TIMEOUT; ++reason)
    for (phase_t phase=LEAD; phase<=DRIVE; ++phase)
    {
      reset(); injected_abort=reason; abort_phase=phase; run();
      stopped(phase == LEAD ? 0 : 1); assert(starts == 1);
      if (phase != DRIVE) assert(profiles == 0);
    }
  reset(); disturbed=true; run(); stopped(1); assert(profiles > 0 && position > 400);
  reset(); stuck=true; run(); stopped(1); assert(tick >= NIGHTFALL_F413_PATH_TIMEOUT_MS);
  reset(); pressed=true; run(); assert(fan_starts == 0 && starts == 0);
  reset(); path[0]=1001; run(); assert(fan_starts == 0 && starts == 0);
  reset(); tracing=true; run(); assert(fan_starts == 0 && starts == 0);
  reset(); suction=false; run();
  assert(starts == 1 && fan_starts == 0 && fan_stops == 0 && profiles > 0);
  assert(first_drive_tick == control_tick && !running && !tracing);

  /* A stopped case0 trial need not creep across its exact endpoint before
   * reaching the existing 0.75 mm / 20 mm/s / 250 ms completion check. */
  f413_run_session_guard_t guard = {0};
  reset(); profiles=1; running=true; stuck=true;
  position=99.5f; velocity=0; stop_profile_finish_tick=20;
  f413_path_run_distance_cursor_reset(&g_f413_path_run_distance_cursor,100);
  assert(f413_path_run_wait_ctrl_target(100,false,&guard,0,false,false) == F413_RUN_SESSION_ABORT_NONE);
  assert(tick == 20 && stop_profile_done && position == 99.5f);
  assert(f413_path_run_settle_test_stop(&guard,0) == F413_RUN_SESSION_ABORT_NONE);
  assert(tick == 20);
  position=99.0f; /* Beyond position tolerance still times out; no bypass. */
  assert(f413_path_run_settle_test_stop(&guard,0) == F413_RUN_SESSION_ABORT_TIMEOUT);
  assert(tick == 20 + F413_PATH_RUN_TEST_STOP_SETTLE_MAX_MS);
  position=100; velocity=30; /* Nor can completion ignore motion. */
  assert(f413_path_run_settle_test_stop(&guard,0) == F413_RUN_SESSION_ABORT_TIMEOUT);
  /* Normal maze runs retain their original endpoint crossing behavior. */
  const f413_run_features_t maze_features={false,false,false,true,false};
  f413_run_features_set(&maze_features);
  position=99.5f; velocity=0;
  assert(f413_path_run_wait_ctrl_target(100,false,&guard,0,false,false) == F413_RUN_SESSION_ABORT_TIMEOUT);
  puts("suction session: hold before fan, 25/50/70/75/100 percent slew rate, 300 ms post-ramp wait, observed yaw regression, all-phase aborts, cleanup and fan-off PASS");
}
