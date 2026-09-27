/* Production executor with the common host odometry fixture. */
#define main chained_front_regression_main
#include "f413_chained_front_tests.c"
#undef main

static void connector(float exit_mm, float entry_mm, float hit_mm,
                      float correction_mm, float expected_mm)
{
  reset();
  f413_path_run_turn_t previous = test_turn(), next = test_turn();
  previous.front_wall_entry = next.front_wall_entry = false;
  previous.dist_out_mm = exit_mm; next.dist_in_mm = entry_mm;
  cores = 1; velocity = step_velocity = 1000;
  wall_end_after = hit_mm;
  float speed = 1000;
  f413_run_session_guard_t guard = {0}; bool consumed = false;
  assert(f413_path_run_drive_chained_offsets(&previous, &next, correction_mm,
      &speed, &guard, NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG, &consumed) == 0);
  assert(consumed && wall_end_polls > 0);
  near(position, expected_mm);
  near(speed, next.velocity_mm_s);
  near(g_f413_path_run_distance_cursor.endpoint_mm, position);
  const unsigned before = profiles;
  assert(run_turn(&next, NULL, consumed, &consumed) == 0);
  near(core_start[1], expected_mm);
  assert(!consumed && profiles == before + 1); /* Only next exit, no entry. */
}

static void all_turn_types(void)
{
  const uint16_t codes[] = {300,400,501,601,502,602,701,702,703,704,801,802,901,902,903,904};
  if (!F413_MOTION_ENABLED(F413_MOTION_CHAINED_OFFSET_WALL_END)) return;
  for (unsigned mode = 2; mode <= 7; ++mode)
  for (unsigned a = 0; a < sizeof(codes)/sizeof(codes[0]); ++a)
  for (unsigned b = 0; b < sizeof(codes)/sizeof(codes[0]); ++b)
  for (unsigned hit = 0; hit < 2; ++hit)
  {
    reset();
    f413_run_features_t features = f413_run_features_get();
    features.front_wall_correction_enabled = false;
    f413_run_features_set(&features);
    f413_path_run_turn_t previous, next;
    assert(f413_path_run_turn_from_code(codes[a], f413_path_run_mode_params(mode), &previous));
    assert(f413_path_run_turn_from_code(codes[b], f413_path_run_mode_params(mode), &next));
    /* A pulse in the next entry must work even when the previous exit is zero.
     * Force an ample entry for every geometry, retaining type/control/speed. */
    previous.dist_out_mm = 4;
    next.dist_in_mm = 12;
    wall_end_after = hit ? 6 : INFINITY;
    float speed = previous.velocity_mm_s;
    f413_run_session_guard_t guard = {0}; bool consumed = false;
    assert(f413_path_run_wait_smooth_turn_profile(&previous, &next, false, false,
        &consumed, -100, &speed, &guard, 0) == 0);
    assert(consumed && wall_end_polls > 0);
    const float travel = position - core_end[0];
    const float step = fmaxf(previous.velocity_mm_s, next.velocity_mm_s) * .001f;
    assert(travel >= (hit ? 6 : 16) - .002f);
    assert(travel < (hit ? 6 : 16) + step + .002f);
    const float endpoint = position;
    assert(run_turn(&next, NULL, consumed, &consumed) == 0);
    near(core_start[1], endpoint);
    assert(!consumed);
  }
}

static void bypass_and_front(void)
{
  for (unsigned bypass = 0; bypass < 2; ++bypass)
  {
    reset();
    f413_path_run_turn_t previous = test_turn(), next = test_turn();
    previous.front_wall_entry = next.front_wall_entry = false;
    f413_run_features_t features = f413_run_features_get();
    if (bypass) features.test_mode_run = true;
    else features.wall_end_correction_enabled = false;
    f413_run_features_set(&features);
    wall_end_after = 0;
    bool consumed = true;
    assert(run_turn(&previous, &next, false, &consumed) == 0);
    assert(!consumed && wall_end_polls == 0);
    near(position - core_end[0], previous.dist_out_mm);
  }
  /* The front target may win after a side-wall hit starts positive follow. */
  reset();
  f413_path_run_turn_t previous = test_turn(), next = test_turn();
  previous.dist_out_mm = 4; next.dist_in_mm = 2;
  front_target = F_ALIGN_TARGET_MM + DIST_HALF_SEC - next.dist_in_mm;
  cores = 1; velocity = step_velocity = 1000;
  hit_after[0] = 3; wall_end_after = 1;
  float speed = 1000; f413_run_session_guard_t guard = {0}; bool consumed;
  assert(f413_path_run_drive_chained_offsets(&previous, &next, 2, &speed,
      &guard, 0, &consumed) == 0);
  assert(consumed); near(position, 3);
  /* Same-sample front and side triggers: front ends the connector immediately. */
  reset(); cores = 1; velocity = step_velocity = speed = 1000;
  hit_after[0] = wall_end_after = 0;
  assert(f413_path_run_drive_chained_offsets(&previous, &next, 2, &speed,
      &guard, 0, &consumed) == 0);
  assert(consumed && wall_end_polls == 0 && profiles == 0); near(position, 0);
}

static void aborts(void)
{
  f413_path_run_turn_t previous = test_turn(), next = test_turn();
  previous.front_wall_entry = next.front_wall_entry = false;
  previous.dist_out_mm = 0; next.dist_in_mm = 20;
  for (unsigned r = F413_RUN_SESSION_ABORT_SWITCH; r <= F413_RUN_SESSION_ABORT_TIMEOUT; ++r)
  {
    reset(); cores = 1; velocity = step_velocity = 1000;
    abort_exit = 1; injected_abort = r;
    float speed = 1000; f413_run_session_guard_t guard = {0}; bool consumed = true;
    assert(f413_path_run_drive_chained_offsets(&previous, &next, 0, &speed,
        &guard, 0, &consumed) == (f413_run_session_abort_reason_t)r);
    assert(!consumed && cores == 1);
  }
  reset(); cores = 1; velocity = step_velocity = 1000; stuck_exit = 1;
  tick = UINT32_MAX - 50;
  float speed = 1000; f413_run_session_guard_t guard = {0}; bool consumed = true;
  assert(f413_path_run_drive_chained_offsets(&previous, &next, 0, &speed,
      &guard, 0, &consumed) == F413_RUN_SESSION_ABORT_TIMEOUT);
  assert(!consumed);
}
int main(void)
{
  connector(4, 6, INFINITY, -23, 10);
  connector(4, 6, 0, -23, 0);
  connector(4, 6, 2, -23, 2);
  connector(4, 6, 4, -23, 4);
  connector(4, 6, 7, -23, 7); /* Detection in the next entry. */
  connector(4, 6, 10, -23, 10); /* Detection on the combined endpoint. */
  connector(4, 6, 2, 3, 11); /* Detection + correction + next entry, once. */
  connector(0, 6, 3, -23, 3);
  connector(4, 0, 2, 3, 5);
  connector(0, 0, INFINITY, -23, 0);
  all_turn_types();
  bypass_and_front();
  aborts();
  puts("PASS: combined offsets, all 16x16 turn pairs/mode2-7, boundaries, signed follow, one-shot entry, front priority, bypass and guards");
}
