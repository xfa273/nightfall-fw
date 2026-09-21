#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct TIM_HandleTypeDef TIM_HandleTypeDef;
#include <stdint.h>
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_path_run.c"
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_run_features.c"

uint16_t path[ROUTE_MAX_LEN];
typedef enum { LEAD, RAMP, SPINUP, DRIVE, CLEANUP } phase_t;
static uint32_t tick, control_tick, fan_tick, full_duty_tick, first_drive_tick;
static unsigned starts, fan_starts, fan_stops, trace_stops, profiles, duty_updates;
static unsigned wait_extra_ms;
static uint16_t fan_duty, requested_duty;
static uint32_t expected_ramp_ms;
static uint16_t flags;
static uint8_t selected_mode;
static bool running, fan, tracing, pressed, fan_fail, stuck, holding, cleanup;
static bool suction, disturbed, duty_fail;
static float position, velocity, target, angle, omega;
static f413_run_session_abort_reason_t injected_abort;
static phase_t abort_phase;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
int trace_printf(const char* fmt, ...) { (void)fmt; return 0; }
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
void f413_hw_emit_video_sync_start_pattern(void) { assert(!running && !fan); }
void f413_hw_emit_video_sync_stop_pattern(void) { assert(!running && !fan); }
void f413_ctrl_start(void)
{
  assert(!fan && !running); /* Includes the blocking IMU calibration. */
  if (suction) assert(flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG);
  tick += 1200; control_tick = tick; starts++; running = true;
  position = velocity = target = angle = omega = 0;
}
void f413_ctrl_stop(void) { running = false; }
void f413_ctrl_set_velocity(float v) { velocity = v; if (v == 0) cleanup = true; }
void f413_ctrl_set_velocity_profile(float start, float end, float distance)
{
  (void)start;
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
bool f413_trace_log_auto_is_enabled(void) { return tracing; }
void f413_trace_log_auto_start(void) { tracing = true; }
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
  selected_mode = 4; wait_extra_ms = 0; fan_duty = 0; requested_duty = 500; expected_ramp_ms = 600;
  running = fan = tracing = pressed = fan_fail = stuck = holding = cleanup = false;
  disturbed = duty_fail = false; suction = true;
  position = velocity = target = angle = omega = 0;
  injected_abort = F413_RUN_SESSION_ABORT_NONE; abort_phase = DRIVE;
  memset(path,0,sizeof(path)); path[0]=209;
  const f413_run_features_t features={false,false,false,true,true};
  f413_run_features_set(&features);
}
static void run(void) { f413_path_run_session_once(suction ? selected_mode : 2,1,0,"host suction"); }
static void stopped(unsigned expected_fan_starts)
{
  assert(!running && !fan && !tracing);
  assert(fan_starts == expected_fan_starts && fan_stops == 1 && trace_stops == 1);
}
int main(void)
{
  /* The user specifies a rate, not a fixed duration for every target. */
  const struct { uint16_t duty; uint32_t ms; } ramps[] = {
    {250, 300}, {500, 600}, {750, 900}, {1000, 1200}
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
  reset(); run(); stopped(1); assert(starts == 1 && position > 400);
  reset(); selected_mode=6; run(); stopped(1); assert(starts == 1 && position > 400);
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
  puts("suction session: hold before fan, 25/50/75/100 percent slew rate, 300 ms post-ramp wait, observed yaw regression, all-phase aborts, cleanup and fan-off PASS");
}
