#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* Include the production runner: mocks replace hardware, not the wait loop. */
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_search_step.c"

static uint32_t tick;
static float position, speed, reference, acceleration, endpoint;
static bool running, cancelled, profile_done, missing_wall, stale_wall;
static unsigned clears, reversals, feedback_resets;
static float command, omega_command;
static f413_run_session_abort_reason_t injected_abort;
static f413_wall_distance_snapshot_t sample;
static enum { FIXED, FOLLOW, FRONT_HANDOFF, FRONT_LOSS, ALIGN_FOLLOW, BACKOFF_STUCK_SENSOR } scenario;

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
int trace_printf(const char* fmt, ...)
{
  if (strstr(fmt, "unsafe-front") != NULL || strstr(fmt, "[SEARCH-STOP] timeout") != NULL)
    assert(!running); /* Logging latency must never delay emergency disable. */
  return 0;
}
float f413_ctrl_get_distance(void) { return position; }
float f413_ctrl_get_real_velocity(void) { return speed; }
float f413_ctrl_get_angle(void) { return 0.0f; }
bool f413_ctrl_stop_profile_complete(void) { return profile_done && !cancelled; }
void f413_ctrl_set_velocity(float value)
{
  assert(fabsf(value) <= MATCH_POS_VEL_MAX);
  if (value < 0) reversals++;
  if (scenario == ALIGN_FOLLOW && f413_search_step_front_match_too_close(&sample))
    assert(value <= 0); /* Never push toward the wall during near/saturated recovery. */
  command = value; cancelled = true; reference = acceleration = 0.0f;
}
void f413_ctrl_clear_velocity_feedback(void) { feedback_resets++; }
void f413_ctrl_set_omega(float value) { omega_command = value; }
void f413_ctrl_set_angle_target(float value) { (void)value; }
void f413_ctrl_clear_angle_target(void) {}
void f413_ctrl_start(void) { running = true; }
void f413_ctrl_stop(void) { running = false; cancelled = true; command = omega_command = 0; }
void f413_wall_runtime_control_clear(void) { clears++; }
bool f413_wall_distance_read_snapshot(f413_wall_distance_snapshot_t* out)
{ *out = sample; return !missing_wall; }
static bool read_wall(f413_wall_sensor_snapshot_t* out)
{ *out = sample.adc; return !missing_wall; }
bool f413_wall_distance_convert_snapshot(const f413_wall_sensor_snapshot_t* adc,
                                         f413_wall_distance_snapshot_t* out)
{ *out = sample; out->adc = *adc; return !missing_wall; }
f413_run_session_abort_reason_t f413_run_session_guard_check(f413_run_session_guard_t* guard)
{ (void)guard; return injected_abort; }
f413_run_session_abort_reason_t f413_run_session_wait_with_auto_step_guarded(
    uint32_t ms, f413_run_session_guard_t* guard)
{
  (void)guard;
  if (ms > 1U) { tick += ms; return injected_abort; } /* Post-completion dwell. */
  tick++;
  if (!stale_wall) sample.adc.sample_sequence++;
  if (scenario == FOLLOW && !cancelled)
  {
    f413_motion_profile_advance(&reference, &acceleration, 0.0f, 0.001f);
    profile_done = reference == 0.0f && acceleration == 0.0f;
    /* Simple ideal follower, not a physical motor qualification. */
    speed = profile_done ? f413_motion_profile_limit(2.0f * (endpoint - position),
                                                     0.0f, -1.0f, true) : reference;
    position += speed * 0.001f;
  }
  if ((scenario == FRONT_HANDOFF || scenario == FRONT_LOSS) && tick >= 10U)
  {
    sample.fr_mm_unwarped = sample.fl_mm_unwarped = sample.front_sum_mm_unwarped = 45.0f;
    speed = cancelled ? 0.0f : 30.0f;
    if (scenario == FRONT_LOSS && cancelled) sample.front_valid = false;
  }
  if (scenario == ALIGN_FOLLOW || scenario == BACKOFF_STUCK_SENSOR)
  {
    speed = command;
    position += command * .001f;
    if (scenario == ALIGN_FOLLOW)
    {
      sample.fr_mm_unwarped -= command * .001f + omega_command * .0005f;
      sample.fl_mm_unwarped -= command * .001f - omega_command * .0005f;
      sample.front_sum_mm_unwarped = .5f * (sample.fr_mm_unwarped + sample.fl_mm_unwarped);
      sample.front_valid = sample.fr_mm_unwarped >= 40 && sample.fl_mm_unwarped >= 40;
      sample.saturated_mask = sample.front_valid ? 0 : F413_WALL_DISTANCE_CH_FR;
    }
  }
  return injected_abort;
}

static void reset_fixture(void)
{
  memset(&g_config, 0, sizeof(g_config));
  g_config.path_timeout_ms = 5000U;
  g_config.get_tick_ms = HAL_GetTick;
  g_config.read_wall_snapshot = read_wall;
  tick = clears = reversals = feedback_resets = 0U;
  command = omega_command = 0;
  position = 0.0f; speed = 0.0f; endpoint = 45.0f;
  reference = 331.662f; acceleration = -reference * reference / 90.0f;
  running = true; cancelled = profile_done = missing_wall = stale_wall = false;
  injected_abort = F413_RUN_SESSION_ABORT_NONE;
  scenario = FIXED;
  memset(&sample, 0, sizeof(sample));
  sample.adc.sample_sequence = 1U;
  sample.adc.fr_delta = sample.adc.fl_delta = 2500;
  sample.fr_mm_unwarped = sample.fl_mm_unwarped = sample.front_sum_mm_unwarped = 90.0f;
  sample.front_valid = true;
}

static f413_run_session_abort_reason_t wait_stop(bool wall_handoff)
{
  f413_run_session_guard_t guard = {0};
  return f413_search_step_wait_stop_approach(endpoint, wall_handoff, &guard, 0U);
}

static void profile_tests(void)
{
  float v = 300.0f, a = -1000.0f;
  for (unsigned i = 0; i < 400; ++i) f413_motion_profile_advance(&v, &a, 0.0f, 0.001f);
  assert(v == 0.0f && a == 0.0f);
  v = 0.0f; a = 1000.0f;
  for (unsigned i = 0; i < 400; ++i) f413_motion_profile_advance(&v, &a, 300.0f, 0.001f);
  assert(v == 300.0f && a == 0.0f);
  v = 600.0f; a = -1000.0f;
  for (unsigned i = 0; i < 400; ++i) f413_motion_profile_advance(&v, &a, 300.0f, 0.001f);
  assert(v == 300.0f && a == 0.0f);
  assert(f413_motion_profile_limit(330, 300, 1, false) == 300);
  assert(f413_motion_profile_limit(270, 300, -1, false) == 300);
  assert(f413_motion_profile_limit(330, 300, -1, false) == 330);
  assert(f413_motion_profile_limit(7, 0, -1, true) == 7);
  assert(f413_motion_profile_limit(100, 0, -1, true) == 30);
  assert(f413_motion_profile_limit(-100, 0, -1, true) == -30);
}

static void alignment_tests(void)
{
  f413_run_session_guard_t guard = {0};
  f413_search_step_front_match_result_t result;
  /* Near extrapolation and saturation: retreat first, then complete alignment. */
  const float starts[] = {38.0f, 41.75f, 42.49f, 43.4f, 45.0f, 50.0f};
  for (unsigned i = 0; i < sizeof(starts)/sizeof(starts[0]); ++i)
  {
    reset_fixture(); scenario = ALIGN_FOLLOW;
    sample.fr_mm_unwarped = sample.fl_mm_unwarped = sample.front_sum_mm_unwarped = starts[i];
    sample.front_valid = starts[i] >= 40;
    sample.saturated_mask = sample.front_valid ? 0 : F413_WALL_DISTANCE_CH_FR;
    assert(f413_search_step_match_front_position(&guard, &result) == F413_RUN_SESSION_ABORT_NONE);
    assert(result.status == F413_SEARCH_FRONT_MATCH_COMPLETE || result.status == F413_SEARCH_FRONT_MATCH_RELAXED);
    assert(tick < MATCH_POS_MAX_DURATION_MS + MATCH_POS_POST_COMPLETE_DELAY_MS);
    assert(command == 0 && omega_command == 0 && running);
    if (starts[i] < 43.5f) assert(reversals > 0 && feedback_resets >= 2);
  }
  reset_fixture(); sample.fr_mm_unwarped = sample.fl_mm_unwarped = sample.front_sum_mm_unwarped = 50;
  /* Miscalibrated/contact-stalled forward correction must not succeed or spin. */
  assert(f413_search_step_match_front_position(&guard, &result) == F413_RUN_SESSION_ABORT_TIMEOUT);
  assert(!running && tick == MATCH_POS_MAX_DURATION_MS && result.status == F413_SEARCH_FRONT_MATCH_TIMEOUT);
  reset_fixture(); sample.fr_mm_unwarped = sample.fl_mm_unwarped = sample.front_sum_mm_unwarped = 50;
  assert(f413_search_step_front_match_continuous(&guard) == F413_RUN_SESSION_ABORT_TIMEOUT);
  assert(!running && tick == MATCH_POS_MAX_DURATION_MS);
  reset_fixture(); scenario = ALIGN_FOLLOW;
  sample.fr_mm_unwarped = 41.75f; sample.fl_mm_unwarped = 43.18f;
  sample.front_sum_mm_unwarped = 42.48f;
  assert(f413_search_step_match_front_position(&guard, &result) == F413_RUN_SESSION_ABORT_NONE);
  assert(reversals > 0 && result.status == F413_SEARCH_FRONT_MATCH_COMPLETE);
  reset_fixture(); sample.fr_mm_unwarped = 39;
  assert(f413_search_step_match_front_position(&guard, &result) == F413_RUN_SESSION_ABORT_TIMEOUT);
  assert(reversals > 0 && !running && tick == MATCH_POS_MAX_DURATION_MS);
  reset_fixture(); scenario = BACKOFF_STUCK_SENSOR; sample.fr_mm_unwarped = 39;
  assert(f413_search_step_match_front_position(&guard, &result) == F413_RUN_SESSION_ABORT_TIMEOUT);
  assert(!running && -position >= MATCH_POS_TOO_CLOSE_RECOVERY_MAX_MM && tick < 500);
  reset_fixture(); stale_wall = true; sample.fr_mm_unwarped = 39;
  assert(f413_search_step_match_front_position(&guard, &result) == F413_RUN_SESSION_ABORT_WALL_FAULT);
  assert(!running && tick == 20);
  reset_fixture(); injected_abort = F413_RUN_SESSION_ABORT_SWITCH;
  assert(f413_search_step_match_front_position(&guard, &result) == F413_RUN_SESSION_ABORT_SWITCH);
  assert(!running && reversals == 0);
  reset_fixture(); missing_wall = true;
  assert(f413_search_step_front_match_continuous(&guard) == F413_RUN_SESSION_ABORT_WALL_FAULT);
  assert(!running);
}

int main(void)
{
  profile_tests();
  reset_fixture(); scenario = FOLLOW;
  assert(wait_stop(false) == F413_RUN_SESSION_ABORT_NONE);
  assert(tick < 5000 && fabsf(endpoint - position) <= 1.0f && cancelled);

  reset_fixture(); position = 44.5f; speed = 0; profile_done = true;
  assert(wait_stop(false) == F413_RUN_SESSION_ABORT_NONE);
  assert(tick == F413_STOP_SETTLE_MS - 1U);
  reset_fixture(); position = 44.5f; speed = 50; profile_done = true;
  assert(wait_stop(false) == F413_RUN_SESSION_ABORT_TIMEOUT && !running);
  reset_fixture(); position = 44.5f; speed = 0; profile_done = false;
  assert(wait_stop(false) == F413_RUN_SESSION_ABORT_TIMEOUT && !running);

  /* Do not hide real encoder stalls by widening the tolerance to fit logs. */
  const float gaps[] = {3.473f, 6.384f};
  for (unsigned i = 0; i < 2; ++i)
  {
    reset_fixture(); position = endpoint - gaps[i]; profile_done = true;
    assert(wait_stop(false) == F413_RUN_SESSION_ABORT_TIMEOUT && !running);
    reset_fixture(); position = endpoint - gaps[i]; scenario = FRONT_HANDOFF;
    assert(wait_stop(true) == F413_RUN_SESSION_ABORT_NONE);
    assert(tick < 100 && cancelled);
  }
  reset_fixture(); scenario = FRONT_LOSS;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_NONE && running);
  reset_fixture(); sample.fr_mm_unwarped = 39.0f; sample.front_valid = false;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_NONE && running && cancelled);
  reset_fixture(); sample.fl_mm_unwarped = 42.4f;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_NONE && running && cancelled);
  reset_fixture(); sample.saturated_mask = F413_WALL_DISTANCE_CH_FL;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_NONE && running && cancelled);
  reset_fixture(); missing_wall = true;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_WALL_FAULT && !running);
  reset_fixture(); sample.fr_mm_unwarped = NAN;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_WALL_FAULT && !running);
  reset_fixture(); stale_wall = true;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_WALL_FAULT && !running && tick == 20U);
  reset_fixture(); injected_abort = F413_RUN_SESSION_ABORT_SWITCH;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_SWITCH && !running && tick == 0);
  reset_fixture(); sample.front_valid = false;
  sample.fr_mm_unwarped = sample.fl_mm_unwarped = sample.front_sum_mm_unwarped = 160.0f;
  scenario = FOLLOW;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_NONE);
  alignment_tests();
  puts("PASS: production stop/alignment, near/saturation retreat, handoff, bounded stall/recovery, sensor/stop abort");
  return 0;
}
