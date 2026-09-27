/* Run the production path executor with ideal 1 ms odometry and scripted
 * valid/invalid front distances. No MCU, UART, motor or fan is accessed. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct TIM_HandleTypeDef TIM_HandleTypeDef;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_path_run.c"
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_run_features.c"

uint16_t path[ROUTE_MAX_LEN];
static uint32_t tick;
static float position, velocity, target, step_velocity;
static bool running, fan, tracing, in_core, stop_profile;
static unsigned cores, profiles, front_reads, completed, aborted, wall_polls;
static unsigned wall_end_polls;
static float core_start[8], core_end[8];
static float hit_after[8], valid_after[8], front_target;
static unsigned abort_exit, stuck_exit;
static f413_run_session_abort_reason_t injected_abort;
static bool turn_lead, core_lead[8];
static unsigned lead_begins, lead_ends, abort_core, abort_entry;
static float lead_begin_pos[8], lead_end_pos[8], wall_end_after;

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
int trace_printf(const char* fmt, ...)
{
  if (strstr(fmt, "[RUN-TEST] path end (") != NULL) completed++;
  if (strstr(fmt, "[RUN-TEST] path aborted") != NULL) aborted++;
  return 0;
}
bool f413_hw_stop_switch_pressed(void) { return false; }
bool f413_hw_fan_start(uint16_t duty) { (void)duty; fan = true; return true; }
bool f413_hw_fan_set_duty(uint16_t duty) { (void)duty; return true; }
void f413_hw_fan_stop(void) { fan = false; }
void f413_hw_emit_video_sync_start_pattern(void) {}
void f413_hw_emit_video_sync_stop_pattern(void) {}
void f413_ctrl_start(void) { running = true; position = velocity = step_velocity = 0; }
void f413_ctrl_stop(void) { running = false; turn_lead = false; }
void f413_ctrl_set_velocity(float v) { velocity = step_velocity = v; stop_profile = false; }
void f413_ctrl_set_velocity_profile(float start, float end, float distance)
{
  profiles++;
  target = position + distance;
  velocity = end;
  step_velocity = fmaxf(1.0f, fmaxf(start, end));
  stop_profile = end == 0.0f;
}
void f413_ctrl_set_omega(float v) { (void)v; }
void f413_ctrl_set_mode4_180_turn(bool enabled)
{
  enabled = enabled && (F413_MOTION_ENABLED(F413_MOTION_MODE4_180_LEAD) ||
      F413_MOTION_ENABLED(F413_MOTION_MODE4_180_TRAJECTORY_FF));
  if (enabled && !turn_lead) { assert(lead_begins < 8); lead_begin_pos[lead_begins++] = position; }
  if (!enabled && turn_lead) { assert(lead_ends < 8); lead_end_pos[lead_ends++] = position; }
  turn_lead = enabled;
}
void f413_ctrl_start_omega_profile(float p, float a, float c)
{
  (void)p; (void)a; (void)c;
  assert(cores < 8);
  core_lead[cores] = turn_lead;
  core_start[cores++] = position;
  in_core = true;
}
void f413_ctrl_stop_omega_profile(void)
{
  if (cores == abort_core) assert(!turn_lead);
  else assert(turn_lead == core_lead[cores - 1]);
  in_core = false; core_end[cores - 1] = position;
}
void f413_ctrl_reset_angle(void) {}
void f413_ctrl_set_angle_target(float v) { (void)v; }
void f413_ctrl_clear_angle_target(void) {}
float f413_ctrl_get_distance(void) { return position; }
float f413_ctrl_get_angle(void) { return 0; }
float f413_ctrl_get_real_omega(void) { return 0; }
float f413_ctrl_get_real_velocity(void) { return velocity; }
bool f413_ctrl_stop_profile_complete(void) { return stop_profile && position >= target; }
bool f413_trace_log_auto_is_enabled(void) { return tracing; }
void f413_trace_log_auto_start(void) { tracing = true; }
void f413_trace_log_auto_step(void) {}
void f413_trace_log_set_mode_flags(uint16_t f) { (void)f; }
void f413_trace_log_auto_stop_after_tail(uint32_t ms) { (void)ms; tracing = false; }
bool f413_run_session_guard_prepare(f413_run_session_guard_t* g)
{ memset(g, 0, sizeof(*g)); return true; }
void f413_run_session_guard_cleanup(f413_run_session_guard_t* g) { (void)g; }
f413_run_session_abort_reason_t f413_run_session_wait_with_auto_step_guarded(
    uint32_t ms, f413_run_session_guard_t* g)
{
  (void)g;
  tick += ms;
  if (in_core && cores == abort_core) return injected_abort;
  if (!in_core && cores == 0 && abort_entry) return injected_abort;
  if (!in_core && cores != 0 && cores == abort_exit) return injected_abort;
  if (!in_core && cores != 0 && cores == stuck_exit) return F413_RUN_SESSION_ABORT_NONE;
  if (running)
  {
    position += step_velocity * (float)ms * 0.001f;
    if (stop_profile && position >= target) { position = target; step_velocity = 0; }
  }
  return F413_RUN_SESSION_ABORT_NONE;
}
uint16_t f413_run_session_abort_reason_to_trace_flag(f413_run_session_abort_reason_t r)
{ return (uint16_t)r; }
const char* f413_run_session_abort_reason_to_text(f413_run_session_abort_reason_t r)
{ (void)r; return "injected"; }
void f413_wall_runtime_set_control_gains(float a, float b) { (void)a; (void)b; }
void f413_wall_runtime_set_wall_end_thresholds(uint16_t a, uint16_t b, uint16_t c, uint16_t d)
{ (void)a; (void)b; (void)c; (void)d; }
void f413_wall_runtime_reset_wall_end_thresholds(void) {}
void f413_wall_runtime_control_clear(void) {}
void f413_wall_runtime_end_clear(void) {}
void f413_wall_runtime_control_apply(bool b) { (void)b; }
void f413_wall_runtime_poll_diagonal(bool b) { (void)b; }
void f413_wall_runtime_poll_straight(bool b) { if (b) wall_polls++; }
bool f413_wall_runtime_poll_wall_end(bool b)
{ (void)b; wall_end_polls++; return cores > 0 && position - core_end[cores - 1] >= wall_end_after; }
bool f413_wall_distance_front_unwarped_mm(float* out)
{
  front_reads++;
  if (in_core || cores == 0 || !isfinite(hit_after[cores - 1])) return false;
  const float travel = position - core_end[cores - 1];
  if (travel < valid_after[cores - 1]) return false;
  *out = front_target + hit_after[cores - 1] - travel;
  return true;
}

static void near(float actual, float expected) { assert(fabsf(actual - expected) < 0.002f); }
static void reset(void)
{
  tick = 0;
  position = velocity = target = step_velocity = 0;
  running = true; fan = tracing = in_core = stop_profile = false;
  cores = profiles = front_reads = completed = aborted = wall_polls = 0;
  wall_end_polls = 0;
  abort_exit = stuck_exit = 0;
  injected_abort = F413_RUN_SESSION_ABORT_NONE;
  turn_lead = false; lead_begins = lead_ends = abort_core = abort_entry = 0;
  wall_end_after = INFINITY;
  memset(core_lead, 0, sizeof(core_lead));
  memset(path, 0, sizeof(path));
  memset(core_start, 0, sizeof(core_start));
  memset(core_end, 0, sizeof(core_end));
  for (unsigned i = 0; i < 8; i++) { hit_after[i] = INFINITY; valid_after[i] = 0; }
  const f413_run_features_t features = {true, true, true, true, false};
  f413_run_features_set(&features);
  f413_path_run_distance_cursor_reset(&g_f413_path_run_distance_cursor, position);
}
static f413_path_run_turn_t test_turn(void)
{
  /* Integral millimetres per tick make exact boundary checks reproducible. */
  const f413_path_run_turn_t turn = {90, 48000, 0, 1000, 3, 4, true, true, false};
  return turn;
}
static f413_run_session_abort_reason_t run_turn(const f413_path_run_turn_t* turn,
    const f413_path_run_turn_t* next, bool skip, bool* next_skip)
{
  float speed = turn->velocity_mm_s;
  f413_run_session_guard_t guard = {0};
  return f413_path_run_wait_smooth_turn_profile(turn, next, false, skip, next_skip,
      -15, &speed, &guard, NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG);
}
static void pair_test(float hit, float valid, bool front_enabled, bool test_mode)
{
  reset();
  f413_path_run_turn_t turn = test_turn();
  /* Different entries detect accidentally using the preceding turn's target. */
  f413_path_run_turn_t next = turn; next.dist_in_mm = 2;
  front_target = F_ALIGN_TARGET_MM + DIST_HALF_SEC - next.dist_in_mm;
  hit_after[0] = hit; valid_after[0] = valid;
  f413_run_features_t features = f413_run_features_get();
  features.front_wall_correction_enabled = front_enabled;
  features.test_mode_run = test_mode;
  f413_run_features_set(&features);
  const bool enabled = F413_MOTION_ENABLED(F413_MOTION_CHAINED_FRONT_ENTRY) &&
      front_enabled && !test_mode;
  const bool early = enabled && isfinite(hit) && fmaxf(hit, valid) <= turn.dist_out_mm;
  bool skip = true;
  assert(run_turn(&turn, &next, false, &skip) == F413_RUN_SESSION_ABORT_NONE);
  assert(skip == early);
  near(position - core_end[0], early ? fmaxf(0, fmaxf(hit, valid)) : turn.dist_out_mm);
  if (early)
  {
    near(g_f413_path_run_distance_cursor.endpoint_mm, position);
    /* Reaching the target is latched even if the sensor disappears between calls. */
    hit_after[0] = INFINITY;
  }
  const float end = position;
  const unsigned before = profiles;
  bool unused;
  assert(run_turn(&next, NULL, skip, &unused) == F413_RUN_SESSION_ABORT_NONE);
  assert(!unused && cores == 2);
  if (early) { near(core_start[1], end); assert(profiles == before + 1); }
  else if (!front_enabled || test_mode || !isfinite(hit))
    near(core_start[1] - end, next.dist_in_mm);
  else if (hit <= turn.dist_out_mm)
    near(core_start[1], end); /* Legacy entry sees the already-passed target. */
  else
    near(core_start[1] - core_end[0], fminf(fmaxf(hit, valid),
        turn.dist_out_mm + next.dist_in_mm + WALL_END_EXTEND_MAX_MM));
  /* Later distances start at the measured core endpoint, never repay a skipped offset. */
  near(position - core_end[1], next.dist_out_mm);
}
static void direct_tests(void)
{
  const float hits[] = {-1, 0, 1, 4, 5, 7, 1000, INFINITY};
  for (unsigned i = 0; i < sizeof(hits) / sizeof(hits[0]); i++)
    pair_test(hits[i], 0, true, false);
  pair_test(1, 2, true, false); /* Wall becomes valid during the exit. */
  pair_test(1, 0, false, false); /* Cases without front correction. */
  pair_test(1, 0, true, true); /* case0/test bypass. */

  reset();
  f413_path_run_turn_t turn = test_turn(), next = turn;
  front_target = F_ALIGN_TARGET_MM + DIST_HALF_SEC - next.dist_in_mm;
  hit_after[0] = 0; turn.dist_out_mm = 0;
  bool skip;
  assert(run_turn(&turn, &next, false, &skip) == F413_RUN_SESSION_ABORT_NONE);
  assert(skip == F413_MOTION_ENABLED(F413_MOTION_CHAINED_FRONT_ENTRY));
  near(position, core_end[0]);

  reset(); turn = test_turn(); next = turn; next.front_wall_entry = false;
  hit_after[0] = 0;
  assert(run_turn(&turn, &next, false, &skip) == F413_RUN_SESSION_ABORT_NONE);
  assert(!skip && front_reads == 1); /* Only the current entry reads the front. */
  near(position - core_end[0], turn.dist_out_mm);

  /* Exit wall-control gating and the existing large-to-large wall-end path. */
  reset(); turn = test_turn(); next = turn;
  turn.front_wall_entry = turn.wall_control_offsets = false; turn.large_turn = true;
  hit_after[0] = 1;
  assert(run_turn(&turn, &next, false, &skip) == F413_RUN_SESSION_ABORT_NONE);
  assert(wall_polls == 0 && wall_end_polls == 0);
  assert(skip == F413_MOTION_ENABLED(F413_MOTION_CHAINED_FRONT_ENTRY));
  reset(); next = turn; hit_after[0] = 0;
  assert(run_turn(&turn, &next, false, &skip) == F413_RUN_SESSION_ABORT_NONE);
  assert(!skip && front_reads == 0 && wall_polls == 0 && wall_end_polls > 0);
  near(position - core_end[0], turn.dist_out_mm);

  /* Preserve all abort reasons, including timeouts while the exit is stuck. */
  for (unsigned r = F413_RUN_SESSION_ABORT_SWITCH; r <= F413_RUN_SESSION_ABORT_TIMEOUT; r++)
  {
    reset(); turn = test_turn(); next = turn; abort_exit = 1; injected_abort = r;
    hit_after[0] = 2;
    assert(run_turn(&turn, &next, false, &skip) == (f413_run_session_abort_reason_t)r);
    assert(!skip && cores == 1);
  }
  reset(); turn = test_turn(); next = turn; stuck_exit = 1; hit_after[0] = 2;
  tick = UINT32_MAX - 100; /* Deadline arithmetic must survive rollover. */
  assert(run_turn(&turn, &next, false, &skip) == F413_RUN_SESSION_ABORT_TIMEOUT);
  assert(!skip && cores == 1);
}
static void session_tests(void)
{
  for (uint8_t mode = 3; mode <= 4; mode++)
  {
    const ShortestRunModeParams_t* params = f413_path_run_mode_params(mode);
    f413_path_run_turn_t turn;
    assert(f413_path_run_turn_from_code(300, params, &turn));
    for (unsigned directions = 0; directions < 8; directions++)
    {
      reset();
      front_target = F_ALIGN_TARGET_MM + DIST_HALF_SEC - turn.dist_in_mm;
      path[0] = 203;
      for (unsigned i = 0; i < 3; i++) path[i + 1] = directions & (1U << i) ? 300 : 400;
      path[4] = 203;
      hit_after[0] = hit_after[1] = 0.5f;
      f413_path_run_session_once(mode, 3, 0, "chained host");
      assert(completed == 1 && aborted == 0 && cores == 3);
      assert(!running && !fan && !tracing);
      for (unsigned i = 1; i < 3; i++)
      {
        const float gap = core_start[i] - core_end[i - 1];
        if (F413_MOTION_ENABLED(F413_MOTION_CHAINED_FRONT_ENTRY))
          assert(gap >= 0.49f && gap < 0.5f + turn.velocity_mm_s * 0.001f + 0.002f);
        else
          assert(gap + 0.002f >= turn.dist_out_mm);
      }
    }
    /* One early hit must not skip the entry of a later turn without a wall. */
    reset();
    front_target = F_ALIGN_TARGET_MM + DIST_HALF_SEC - turn.dist_in_mm;
    path[0] = 203; path[1] = 300; path[2] = 400; path[3] = 300; path[4] = 203;
    hit_after[0] = 0;
    f413_path_run_session_once(mode, 3, 0, "one hit host");
    assert(completed == 1 && cores == 3 && aborted == 0);
    assert(core_start[2] - core_end[1] + 0.002f >= turn.dist_out_mm + turn.dist_in_mm);
    /* An explicit straight separates turns even with a visible near wall. */
    reset();
    front_target = F_ALIGN_TARGET_MM + DIST_HALF_SEC - turn.dist_in_mm;
    path[0] = 203; path[1] = 300; path[2] = 203; path[3] = 400; path[4] = 203;
    hit_after[0] = 0;
    f413_path_run_session_once(mode, 3, 0, "separated host");
    assert(completed == 1 && cores == 2 && aborted == 0);
    assert(core_start[1] - core_end[0] >= turn.dist_out_mm + 3 * DIST_HALF_SEC - 0.01f);
    /* Abort during the monitored exit never starts the following turn. */
    reset();
    path[0] = 203; path[1] = 300; path[2] = 400; path[3] = 203;
    abort_exit = 1; injected_abort = F413_RUN_SESSION_ABORT_SWITCH; hit_after[0] = 2;
    f413_path_run_session_once(mode, 3, 0, "abort host");
    assert(completed == 0 && aborted == 1 && cores == 1);
    assert(!running && !fan && !tracing);
  }
}
int main(void)
{
  direct_tests();
  session_tests();
  printf("chained front: policy=%#x, target crossing/boundary/invalid/bypass/guard, mode3/4 all 3-turn directions and separate straights PASS\n",
      (unsigned)F413_MOTION_FEATURES);
  return 0;
}
