#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Production search primitives; an ideal 1 ms follower replaces only hardware.
 * This qualifies command geometry/latency accounting, not physical tracking. */
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_search_step.c"

uint16_t map[MAZE_SIZE][MAZE_SIZE], smap[MAZE_SIZE][MAZE_SIZE], wall_info;
bool visited[MAZE_SIZE][MAZE_SIZE];
volatile struct coordinate_and_direction mouse;
static uint32_t tick, abort_at;
static float position, velocity, reference, acceleration, profile_target, profile_end;
static float angle, target_angle, angular_rate, front_wall_position, edge_position;
static float last_profile_distance, last_profile_accel, last_turn_entry, last_turn_exit;
static float spot_spin_encoder_drift;
static unsigned profile_calls, foreground_delay_ms, read_delay_ms, spin_count;
static bool running, profile_active, stop_complete, edge_fired, front_available;
static bool wall_align_model;
static f413_run_session_abort_reason_t injected_abort;
static f413_wall_sensor_snapshot_t wall;

static void advance(unsigned ms)
{
  for (unsigned i = 0; i < ms; ++i) {
    tick++;
    wall.sample_sequence++;
    if (!running) continue;
    if (profile_active) {
      const float previous = reference;
      f413_motion_profile_advance(&reference, &acceleration, profile_target, 0.001f);
      velocity = 0.5f * (previous + reference);
      position += velocity * 0.001f;
      if (reference == 0 && acceleration == 0) {
        stop_complete = true;
        position = profile_end; /* Ideal final position servo. */
        velocity = 0;
      }
    } else {
      position += velocity * 0.001f;
    }
    angle += angular_rate * 0.001f;
  }
}
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { advance(ms); }
int trace_printf(const char *fmt, ...) { (void)fmt; advance(foreground_delay_ms); return 0; }
float f413_ctrl_get_distance(void) { return position; }
float f413_ctrl_get_real_velocity(void) { return velocity; }
float f413_ctrl_get_angle(void) { return angle; }
float f413_ctrl_get_target_angle(void) { return target_angle; }
void f413_ctrl_reset_distance(void) { position = 0; }
void f413_ctrl_reset_angle(void) { angle = target_angle = 0; }
void f413_ctrl_set_velocity(float v) { velocity = reference = v; profile_active = stop_complete = false; }
void f413_ctrl_set_velocity_profile(float v0, float v1, float mm)
{
  assert(isfinite(mm) && mm > 0.001f && running);
  last_profile_distance = mm;
  last_profile_accel = acceleration = (v1*v1-v0*v0)/(2*mm);
  profile_calls++;
  reference = velocity = v0; profile_target = v1; profile_end = position + mm;
  profile_active = true; stop_complete = false;
}
bool f413_ctrl_stop_profile_complete(void) { return profile_active && stop_complete; }
void f413_ctrl_set_omega(float omega) { angular_rate = omega; }
void f413_ctrl_set_angle_target(float a) { target_angle = a; angle = a; }
void f413_ctrl_clear_angle_target(void) {}
void f413_ctrl_clear_velocity_feedback(void) {}
void f413_ctrl_start(void) { running = true; }
void f413_ctrl_stop(void) { running = false; velocity = angular_rate = 0; profile_active = false; }
void f413_ctrl_start_omega_profile(float omega, float ta, float tc)
{
  target_angle = angle + omega * (ta + tc);
  angular_rate = omega * (ta + tc) / (2*ta+tc);
  last_turn_entry = position;
  if (velocity == 0) position += spot_spin_encoder_drift;
  spin_count++;
}
void f413_ctrl_stop_omega_profile(void) { angle = target_angle; angular_rate = 0; last_turn_exit = position; }
void f413_wall_runtime_control_clear(void) {}
void f413_wall_runtime_end_clear(void) { edge_fired = false; }
float f413_wall_runtime_latest_error(void) { return 0; }
bool f413_wall_runtime_poll_wall_end(bool gate)
{
  if (gate && !edge_fired && position >= edge_position) { edge_fired = true; return true; }
  return false;
}
bool f413_wall_runtime_wall_end_detected(float *r, float *l) { *r = position; *l = -1; return edge_fired; }
static bool read_wall(f413_wall_sensor_snapshot_t *out) { advance(read_delay_ms); *out = wall; return true; }
bool f413_wall_distance_front_unwarped_mm(float *out)
{ *out = front_wall_position - position; return front_available; }
bool f413_wall_distance_convert_snapshot(const f413_wall_sensor_snapshot_t *adc, f413_wall_distance_snapshot_t *out)
{
  memset(out, 0, sizeof(*out)); out->adc = *adc;
  const float mm = wall_align_model ? front_wall_position - position : 90;
  out->fr_mm_unwarped = out->fl_mm_unwarped = out->front_sum_mm_unwarped = mm;
  out->front_valid = true;
  return true;
}
bool f413_wall_distance_read_snapshot(f413_wall_distance_snapshot_t *out)
{ return f413_wall_distance_convert_snapshot(&wall, out); }
f413_run_session_abort_reason_t f413_run_session_guard_check(f413_run_session_guard_t *g)
{ (void)g; return abort_at && tick >= abort_at ? F413_RUN_SESSION_ABORT_SWITCH : injected_abort; }
f413_run_session_abort_reason_t f413_run_session_wait_with_auto_step_guarded(uint32_t ms, f413_run_session_guard_t *g)
{ advance(ms); return f413_run_session_guard_check(g); }
/* Real event append exercises read/FRAM/logging delay; writes are host-only. */
nvm_status_t nvm_trace_log_append_cached(nvm_trace_log_header_t *h, const nvm_trace_log_record_t *r, uint8_t commit)
{ (void)h; (void)r; (void)commit; advance(foreground_delay_ms); return NVM_STATUS_OK; }
HAL_StatusTypeDef nvm_maze_save_map(const uint16_t *cells, uint32_t n)
{ (void)cells; (void)n; abort(); }
float f413_ctrl_get_target_distance(void) { return profile_end; }
float f413_ctrl_get_target_velocity(void) { return reference; }
float f413_ctrl_get_accel_velocity(void) { return velocity; }
float f413_ctrl_get_target_omega(void) { return angular_rate; }
float f413_ctrl_get_log_angle(void) { return angle; }
float f413_ctrl_get_log_real_omega(void) { return angular_rate; }
float f413_ctrl_get_gyro_z_raw(void) { return angular_rate; }
float f413_ctrl_get_accel_forward(void) { return acceleration; }
int16_t f413_ctrl_get_log_encoder_delta_l(void) { return 0; }
int16_t f413_ctrl_get_log_encoder_delta_r(void) { return 0; }
int16_t f413_ctrl_get_motor_out_l(void) { return 0; }
int16_t f413_ctrl_get_motor_out_r(void) { return 0; }
bool f413_ctrl_angle_target_enabled(void) { return false; }

static void reset(void)
{
  tick = abort_at = profile_calls = foreground_delay_ms = read_delay_ms = spin_count = 0;
  position = velocity = reference = acceleration = angle = target_angle = angular_rate = 0;
  profile_active = stop_complete = edge_fired = front_available = wall_align_model = false;
  spot_spin_encoder_drift = 0;
  running = true; edge_position = INFINITY; front_wall_position = 900;
  injected_abort = F413_RUN_SESSION_ABORT_NONE;
  memset(&wall, 0, sizeof(wall)); wall.sample_sequence = 1;
  wall.fr_delta = wall.fl_delta = 1000;
  memset(&g_config, 0, sizeof(g_config));
  g_config.path_timeout_ms = 5000; g_config.step_turn_deg = 90;
  g_config.get_tick_ms = HAL_GetTick; g_config.read_wall_snapshot = read_wall;
  g_search_event_log_active = false; g_search_wall_read_valid = false;
  g_post_goal_active = g_post_goal_save_pending = false;
  memset(map, 0, sizeof(map)); memset(visited, 0, sizeof(visited));
  mouse.x = mouse.y = mouse.dir = 0;
  f413_run_features_reset();
  f413_search_step_reset_distance();
}
static void close_to(float got, float expected)
{ if (fabsf(got-expected) > 0.01f) fprintf(stderr, "got %.6f expected %.6f\n", got, expected); assert(fabsf(got-expected) <= 0.01f); }

static void corridor_tests(void)
{
  /* Logged run: entry55 +7 cells90 +stop45 =730mm. Background work used to
   * move the commanded endpoint to756.387mm. Test without shortening logs. */
  const unsigned delays[] = {0, 3, 8, 20};
  const unsigned cells[] = {7, 15, 60};
  for (unsigned d = 0; d < 4; ++d) for (unsigned c = 0; c < 3; ++c) {
    reset(); foreground_delay_ms = delays[d]; read_delay_ms = delays[d];
    SearchRunParams_t params = searchRunParams[0]; params.wall_align_enable = 0;
    f413_run_session_guard_t guard = {0}; float v = 0; bool acceled = false;
    assert(f413_search_step_run_entry_section(2, F413_SEARCH_STEP_TARGET_FULL, &params, &v, &guard) == F413_RUN_SESSION_ABORT_NONE);
    const float entry = (float)(DIST_FIRST_SEC + DIST_HALF_SEC);
    close_to(g_search_distance_endpoint_mm, entry);
    for (unsigned i = 0; i < cells[c]; ++i) {
      /* Walls, decision and record writes continue while the controller runs. */
      advance(delays[d]);
      f413_search_event_context_t ctx = {0};
      f413_search_step_motion_detail_t detail = {0};
      assert(f413_search_step_run_forward_section(&params, &v, &guard, &acceled, false, false, &ctx, &detail) == F413_RUN_SESSION_ABORT_NONE);
      close_to(g_search_distance_endpoint_mm, entry + (i+1)*90.0f);
    }
    advance(delays[d]);
    assert(f413_search_step_run_final_stop(&v, &guard) == F413_RUN_SESSION_ABORT_NONE);
    close_to(profile_end, entry + cells[c]*90.0f + DIST_HALF_SEC);
    close_to(position, profile_end);
  }
}
static void boundary_tests(void)
{
  reset(); float v = 300; velocity = 300;
  f413_run_session_guard_t guard = {0};
  advance(10); /*3mm during previous decision*/
  assert(f413_search_step_drive_accel_distance_with_accel(90, 1000, &v, &guard, 0) == F413_RUN_SESSION_ABORT_NONE);
  close_to(last_profile_distance, 87); close_to(last_profile_accel, 1000);
  close_to(profile_end, 90);
  advance(8);
  assert(f413_search_step_drive_decel_distance_with_accel(90, 500, &v, &guard, 0) == F413_RUN_SESSION_ABORT_NONE);
  close_to(last_profile_accel, -500); close_to(profile_end, 180);
  /* Known-corridor boost, then either whole-cell or half-cell+buffer braking. */
  for (unsigned next_turn = 0; next_turn < 2; ++next_turn) {
    reset(); velocity = v = 300; bool acceled = false;
    SearchRunParams_t params = searchRunParams[0]; params.wall_align_enable = 0;
    f413_search_step_motion_detail_t detail = {0};
    advance(8);
    assert(f413_search_step_run_forward_section(&params, &v, &guard, &acceled,
        true, false, NULL, &detail) == F413_RUN_SESSION_ABORT_NONE);
    assert(acceled); close_to(profile_end, 90);
    advance(7);
    assert(f413_search_step_run_forward_section(&params, &v, &guard, &acceled,
        false, next_turn != 0, NULL, &detail) == F413_RUN_SESSION_ABORT_NONE);
    assert(!acceled); close_to(profile_end, 180);
  }
  /* Consumed turn entry must stop instead of appending another offset. */
  reset(); position = 12; velocity = v = 300;
  assert(f413_search_step_drive_segment(10, 300, &v, &guard, 0) == F413_RUN_SESSION_ABORT_TIMEOUT);
  assert(!running && profile_calls == 0);
  reset(); position = 50; velocity = v = 300;
  assert(f413_search_step_run_final_stop(&v, &guard) == F413_RUN_SESSION_ABORT_TIMEOUT);
  assert(!running && profile_calls == 0);
  reset(); position = NAN;
  assert(f413_search_step_drive_segment(90, 300, &v, &guard, 0) == F413_RUN_SESSION_ABORT_IMU_FAULT);
  assert(!running);
  /* Odometer reset must discard a preceding run's target. */
  reset(); g_search_distance_endpoint_mm = 700; position = 705;
  f413_search_step_reset_distance(); v = 300;
  assert(f413_search_step_drive_segment(90, 300, &v, &guard, 0) == F413_RUN_SESSION_ABORT_NONE);
  close_to(profile_end, 90);
}
static void turn_and_wall_tests(void)
{
  f413_run_session_guard_t guard = {0};
  for (unsigned rel = 1; rel <= 3; rel += 2) {
    reset(); SearchRunParams_t params = searchRunParams[0]; float v = params.velocity_turn90;
    velocity = v; advance(5);
    assert(f413_search_step_run_smooth_turn(rel, &params, &v, &guard) == F413_RUN_SESSION_ABORT_NONE);
    assert(last_turn_entry >= params.dist_offset_in && last_turn_entry < params.dist_offset_in + 0.4f);
    close_to(profile_end, last_turn_exit + params.dist_offset_out);
    const float previous_end = g_search_distance_endpoint_mm;
    advance(8);
    assert(f413_search_step_drive_segment(90, v, &v, &guard, 0) == F413_RUN_SESSION_ABORT_NONE);
    close_to(profile_end, previous_end + 90);
  }
  reset(); float v = 300; velocity = v; position = 3;
  f413_run_features_t features = {.wall_end_correction_enabled = true};
  f413_run_features_set(&features);
  edge_position = 60; foreground_delay_ms = 5; read_delay_ms = 4;
  g_search_event_log_active = true;
  f413_search_event_context_t ctx = {0}; bool found = false;
  assert(f413_search_step_drive_wallend_segment(90, v, &v, &guard, 0, &found, &searchRunParams[0], &ctx) == F413_RUN_SESSION_ABORT_NONE);
  assert(found); const float detected = g_search_distance_endpoint_mm;
  assert(position > detected + 2); /* Event writing moved the robot. */
  assert(f413_search_step_drive_segment(45, v, &v, &guard, 0) == F413_RUN_SESSION_ABORT_NONE);
  close_to(profile_end, detected + 45);
  /* A missing wall end retains the nominal boundary. */
  reset(); f413_run_features_set(&features); velocity = v = 300; position = 3;
  assert(f413_search_step_drive_wallend_segment(90, v, &v, &guard, 0, &found, &searchRunParams[0], NULL) == F413_RUN_SESSION_ABORT_NONE);
  assert(!found); close_to(profile_end, 90);
  /* Front correction keeps the nominal sensor threshold after shortening entry. */
  for (unsigned late = 0; late < 2; ++late) {
    reset(); features.wall_end_correction_enabled = false; features.front_wall_correction_enabled = true;
    f413_run_features_set(&features); front_available = true; velocity = v = 300; position = 2;
    front_wall_position = F_ALIGN_TARGET_MM + DIST_HALF_SEC + (late ? 3 : -3);
    assert(f413_search_step_drive_front_wall_entry_segment(10, v, &v, &guard, 0) == F413_RUN_SESSION_ABORT_NONE);
    const float corrected = g_search_distance_endpoint_mm;
    assert(corrected >= (late ? 13 : 7) && corrected < (late ? 13.4f : 7.4f));
    advance(4);
    assert(f413_search_step_drive_segment(20, v, &v, &guard, 0) == F413_RUN_SESSION_ABORT_NONE);
    close_to(profile_end, corrected + 20);
  }
}
static void lifecycle_tests(void)
{
  f413_run_session_guard_t guard = {0};
  SearchRunParams_t params = searchRunParams[0]; params.wall_align_enable = 0;
  f413_search_step_motion_detail_t detail = {0};
  reset(); float v = 300; velocity = v;
  g_search_distance_endpoint_mm = 90; position = 93;
  spot_spin_encoder_drift = 7;
  assert(f413_search_step_run_back_turn(&params, &v, &guard, &detail) == F413_RUN_SESSION_ABORT_NONE);
  assert(spin_count == 1);
  close_to(profile_end, 90 + 45 + 7 + 45);
  const float after_back = profile_end;
  advance(10);
  assert(f413_search_step_drive_segment(90, v, &v, &guard, 0) == F413_RUN_SESSION_ABORT_NONE);
  close_to(profile_end, after_back + 90);

  /* A physical front alignment supersedes the old encoder endpoint. */
  reset(); position = 110; g_search_distance_endpoint_mm = 120;
  wall_align_model = true; front_wall_position = position + F_ALIGN_TARGET_MM + 5;
  f413_search_step_front_match_result_t match;
  assert(f413_search_step_match_front_position(&guard, &match) == F413_RUN_SESSION_ABORT_NONE);
  assert(match.status == F413_SEARCH_FRONT_MATCH_COMPLETE || match.status == F413_SEARCH_FRONT_MATCH_RELAXED);
  const float aligned = position;
  assert(aligned > 110 && aligned < 116);
  close_to(g_search_distance_endpoint_mm, aligned);
  v = 0;
  assert(f413_search_step_drive_accel_distance(45, &params, &v, &guard, 0) == F413_RUN_SESSION_ABORT_NONE);
  close_to(profile_end, aligned + 45);

  /* Early wall handoff also cancels the nominal encoder endpoint. */
  reset(); velocity = v = 300; wall_align_model = true;
  front_wall_position = F_ALIGN_TARGET_MM + 25;
  assert(f413_search_step_drive_segment_impl(45, 0, &v, &guard, 0, true) == F413_RUN_SESSION_ABORT_NONE);
  assert(position < 30);
  close_to(g_search_distance_endpoint_mm, position);

  /* Full phase restart: stop, reverse, odometer reset, 55 mm entry. */
  reset(); g_search_distance_endpoint_mm = 700; position = 704;
  velocity = v = 300; uint16_t action = 9;
  assert(f413_search_step_run_phase_restart_entry(2, &action, 1,
      F413_SEARCH_STEP_TARGET_GOAL, &params, &v, &guard, false) == F413_RUN_SESSION_ABORT_NONE);
  close_to(profile_end, DIST_FIRST_SEC + DIST_HALF_SEC);
  close_to(g_search_distance_endpoint_mm, DIST_FIRST_SEC + DIST_HALF_SEC);
  assert(mouse.y == 1);

  /* A failed motion never silently continues; a later session starts fresh. */
  reset(); velocity = v = 300; abort_at = 10;
  assert(f413_search_step_drive_segment(90, v, &v, &guard, 0) == F413_RUN_SESSION_ABORT_SWITCH);
  f413_ctrl_stop(); abort_at = 0;
  f413_ctrl_start(); f413_search_step_reset_distance();
  assert(f413_search_step_drive_segment(90, v, &v, &guard, 0) == F413_RUN_SESSION_ABORT_NONE);
  close_to(profile_end, 90);
}
int main(void)
{
  corridor_tests(); boundary_tests(); turn_and_wall_tests(); lifecycle_tests();
  puts("PASS: production search distance, delayed corridor/stop, accel limits, turns, wall correction, spin/alignment/reverse/restart, abort/reset");
  return 0;
}
