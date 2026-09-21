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
static uint32_t tick;
static unsigned starts, fan_starts, fan_stops, trace_stops;
static bool running, fan, tracing, pressed, fan_fail, stuck;
static float position, velocity, target;
static f413_run_session_abort_reason_t injected_abort;
static bool abort_during_spinup;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
int trace_printf(const char* fmt, ...) { (void)fmt; return 0; }
bool f413_hw_stop_switch_pressed(void) { return pressed; }
bool f413_hw_fan_start(uint16_t duty)
{ assert(!running); assert(duty == 500); fan_starts++; fan = !fan_fail; return fan; }
void f413_hw_fan_stop(void) { assert(!running); fan_stops++; fan = false; }
void f413_hw_emit_video_sync_start_pattern(void) { assert(!running && !fan); }
void f413_hw_emit_video_sync_stop_pattern(void) { assert(!running && !fan); }
void f413_ctrl_start(void) { assert(fan); starts++; running = true; }
void f413_ctrl_stop(void) { running = false; }
void f413_ctrl_set_velocity(float v) { velocity = v; }
void f413_ctrl_set_velocity_profile(float start, float end, float distance)
{ (void)start; target = position + distance; velocity = end; }
void f413_ctrl_set_omega(float v) { (void)v; }
void f413_ctrl_start_omega_profile(float p, float a, float c) { (void)p; (void)a; (void)c; }
void f413_ctrl_stop_omega_profile(void) {}
void f413_ctrl_reset_angle(void) {}
void f413_ctrl_clear_angle_target(void) {}
float f413_ctrl_get_distance(void) { return position; }
float f413_ctrl_get_angle(void) { return 0; }
float f413_ctrl_get_real_velocity(void) { return velocity; }
bool f413_trace_log_auto_is_enabled(void) { return tracing; }
void f413_trace_log_auto_start(void) { tracing = true; }
void f413_trace_log_auto_step(void) {}
void f413_trace_log_set_mode_flags(uint16_t f) { (void)f; }
void f413_trace_log_auto_stop_after_tail(uint32_t ms)
{ (void)ms; assert(!fan && !running); tracing = false; trace_stops++; }
bool f413_run_session_guard_prepare(f413_run_session_guard_t* g) { memset(g,0,sizeof(*g)); return true; }
void f413_run_session_guard_cleanup(f413_run_session_guard_t* g) { (void)g; }
f413_run_session_abort_reason_t f413_run_session_wait_with_auto_step_guarded(uint32_t ms, f413_run_session_guard_t* g)
{
  (void)g; tick += ms;
  if (injected_abort && (running || abort_during_spinup)) return injected_abort;
  if (running && !stuck) position = target;
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
 tick=starts=fan_starts=fan_stops=trace_stops=0;
 running=fan=tracing=pressed=fan_fail=stuck=abort_during_spinup=false;
 position=velocity=target=0; injected_abort=F413_RUN_SESSION_ABORT_NONE;
 memset(path,0,sizeof(path)); path[0]=209;
 const f413_run_features_t features={false,false,false,true,true};
 f413_run_features_set(&features);
}
static void run(void) { f413_path_run_session_once(4,1,0,"host suction"); }
static void stopped(void)
{ assert(!running && !fan && !tracing); assert(fan_starts == 1 && fan_stops == 1 && trace_stops == 1); }
int main(void)
{
 reset(); run(); stopped(); assert(starts == 1 && position > 400);
 reset(); fan_fail=true; run(); stopped(); assert(starts == 0);
 for (unsigned reason=F413_RUN_SESSION_ABORT_SWITCH; reason<=F413_RUN_SESSION_ABORT_TIMEOUT; ++reason)
  for (unsigned spinup=0; spinup<2; ++spinup)
  { reset(); injected_abort=reason; abort_during_spinup=spinup; run(); stopped(); assert(starts == (spinup ? 0 : 1)); }
 reset(); stuck=true; run(); stopped(); assert(tick >= NIGHTFALL_F413_PATH_TIMEOUT_MS);
 reset(); pressed=true; run(); assert(fan_starts == 0 && starts == 0);
 reset(); path[0]=1001; run(); assert(fan_starts == 0 && starts == 0);
 reset(); tracing=true; run(); assert(fan_starts == 0 && starts == 0);
 puts("suction session: finish, PWM failure, spinup/run aborts, timeout and preflight refusal PASS");
}
