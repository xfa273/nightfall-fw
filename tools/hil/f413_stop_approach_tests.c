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
static unsigned clears, handoffs;
static f413_run_session_abort_reason_t injected_abort;
static f413_wall_distance_snapshot_t sample;
static enum { FIXED, FOLLOW, FRONT_HANDOFF, FRONT_LOSS } scenario;

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
int trace_printf(const char* fmt, ...)
{
  if (strstr(fmt, "front-wall handoff") != NULL) handoffs++;
  if (strstr(fmt, "unsafe-front") != NULL || strstr(fmt, "[SEARCH-STOP] timeout") != NULL)
    assert(!running); /* Logging latency must never delay emergency disable. */
  return 0;
}
float f413_ctrl_get_distance(void) { return position; }
float f413_ctrl_get_real_velocity(void) { return speed; }
float f413_ctrl_get_angle(void) { return 0.0f; }
bool f413_ctrl_stop_profile_complete(void) { return profile_done && !cancelled; }
void f413_ctrl_set_velocity(float value)
{ assert(value == 0.0f); cancelled = true; reference = acceleration = 0.0f; }
void f413_ctrl_set_omega(float value) { assert(value == 0.0f); }
void f413_ctrl_set_angle_target(float value) { (void)value; }
void f413_ctrl_stop(void) { running = false; cancelled = true; }
void f413_wall_runtime_control_clear(void) { clears++; }
bool f413_wall_distance_read_snapshot(f413_wall_distance_snapshot_t* out)
{ *out = sample; return !missing_wall; }
f413_run_session_abort_reason_t f413_run_session_guard_check(f413_run_session_guard_t* guard)
{ (void)guard; return injected_abort; }
f413_run_session_abort_reason_t f413_run_session_wait_with_auto_step_guarded(
    uint32_t ms, f413_run_session_guard_t* guard)
{
  (void)guard;
  assert(ms == 1U);
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
  return injected_abort;
}

static void reset_fixture(void)
{
  memset(&g_config, 0, sizeof(g_config));
  g_config.path_timeout_ms = 5000U;
  g_config.get_tick_ms = HAL_GetTick;
  tick = clears = handoffs = 0U;
  position = 0.0f; speed = 0.0f; endpoint = 45.0f;
  reference = 331.662f; acceleration = -reference * reference / 90.0f;
  running = true; cancelled = profile_done = missing_wall = stale_wall = false;
  injected_abort = F413_RUN_SESSION_ABORT_NONE;
  scenario = FIXED;
  memset(&sample, 0, sizeof(sample));
  sample.adc.sample_sequence = 1U;
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
    assert(tick < 100 && handoffs == 1 && cancelled);
  }
  reset_fixture(); scenario = FRONT_LOSS;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_WALL_FAULT && !running);
  reset_fixture(); sample.fr_mm_unwarped = 39.0f; sample.front_valid = false;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_WALL_FAULT && !running && tick == 0);
  reset_fixture(); sample.fl_mm_unwarped = 42.4f;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_WALL_FAULT && !running);
  reset_fixture(); sample.saturated_mask = F413_WALL_DISTANCE_CH_FL;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_WALL_FAULT && !running);
  reset_fixture(); missing_wall = true;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_WALL_FAULT && !running);
  reset_fixture(); stale_wall = true;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_WALL_FAULT && !running && tick == 20U);
  reset_fixture(); injected_abort = F413_RUN_SESSION_ABORT_SWITCH;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_SWITCH && !running && tick == 0);
  reset_fixture(); sample.front_valid = false;
  sample.fr_mm_unwarped = sample.fl_mm_unwarped = sample.front_sum_mm_unwarped = 160.0f;
  scenario = FOLLOW;
  assert(wait_stop(true) == F413_RUN_SESSION_ABORT_NONE && handoffs == 0);
  puts("PASS: production stop wait, 45mm deceleration, log gaps, handoff, near/saturation/loss/stall/abort");
  return 0;
}
