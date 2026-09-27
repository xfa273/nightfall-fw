/* Production path/session, with the existing suction order/guard fixture. */
#define main suction_fixture_main
#include "f413_suction_session_tests.c"
#undef main

static f413_run_session_guard_t goal_guard;
static uint32_t goal_begin;
static float stop_shortfall;
static bool force_moving, goal_started;
static f413_run_session_abort_reason_t goal_abort;

static void prepare_goal(void)
{
  reset();
  profiles = 1; running = stuck = true;
  position = 98; target = 100; velocity = 0;
  f413_path_run_distance_cursor_reset(&g_f413_path_run_distance_cursor, 100);
}

static f413_run_session_abort_reason_t wait_goal(bool profile_required)
{
  return f413_path_run_wait_goal_stop(100, profile_required, &goal_guard,
      NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG, true, false);
}

static void speed_spike(void) { velocity = tick == 30 ? 21 : 0; }

/* 21:31 case5, seq2890..2958, 16994..17062 ms. Distance is quantized to
 * 1 mm in the log; all saved positions are 3695/3696 for target3697.108.
 * Replay the velocity excursions conservatively at the larger deficit.
 * Profile completion is assumed at replay entry; its exact timestamp was
 * not logged, and this replay does not model the physical closed loop.
 */
static const int16_t logged_velocity[] = {
  -60,-61,-59,-61,-65,-61,-60,-47,-38,-21,-12,-20,-25,-43,-62,-75,-66,
  -49,-28,-4,19,40,49,49,37,23,10,-3,-1,8,4,-8,-15,-21,-22,-22,
  -25,-33,-41,-50,-58,-48,-36,-11,11,32,33,25,17,9,6,3,1,0,-1,
  0,-1,-3,-6,-8,-8,-6,-2,1,-1,-5,-5,-4,-3
};
static void replay_stop(void)
{
  assert(tick < sizeof(logged_velocity) / sizeof(logged_velocity[0]));
  velocity = logged_velocity[tick];
}

static void observe_session_stop(void)
{
  if (profile_target_velocity != 0) return;
  if (!goal_started) { goal_started = true; goal_begin = tick; }
  position = target - stop_shortfall;
  stop_profile_done = tick - goal_begin >= 40;
  velocity = (!stop_profile_done || force_moving) ? 200 : 0;
  if (goal_abort && stop_profile_done) injected_abort = goal_abort;
}

static void prepare_session(float shortfall)
{
  reset();
  selected_mode = 4; selected_case = 5;
  const f413_run_features_t maze = {false, false, false, true, false};
  f413_run_features_set(&maze);
  suction = f413_path_run_mode_params(selected_mode)->fan_power > 0;
  goal_begin = 0; goal_started = force_moving = false;
  goal_abort = F413_RUN_SESSION_ABORT_NONE;
  stop_shortfall = shortfall;
  drive_observer = observe_session_stop;
}

int main(void)
{
  /* Both compiled machine profiles exercise the actual final-tail caller. */
  for (unsigned video = 0; video <= 1; ++video)
  {
    prepare_session(2); test_auto_video_capture = video;
    run();
    if (suction) stopped(1);
    else assert(!running && !tracing && fan_starts == 0 && fan_stops == 0);
    assert(goal_started);
    if (F413_MOTION_ENABLED(F413_MOTION_PATH_GOAL_STOP))
    {
      assert(completed_sessions == 1);
      assert(fan_stop_tick - goal_begin == 40 + 20 + NIGHTFALL_F413_PATH_COAST_MS);
    }
    else
    {
      assert(completed_sessions == 0);
      assert(tick - goal_begin >= NIGHTFALL_F413_PATH_TIMEOUT_MS);
    }
    if (suction) assert(tick - fan_stop_tick == video * 9750U); /* STOP after fan-off. */
  }
  if (!F413_MOTION_ENABLED(F413_MOTION_PATH_GOAL_STOP))
  {
    puts("goal stop: mini_r2 exact-crossing behavior retained PASS");
    return 0;
  }

  prepare_goal(); stop_profile_finish_tick = 20;
  assert(wait_goal(true) == F413_RUN_SESSION_ABORT_NONE && tick == 40);
  prepare_goal(); position = 103; stop_profile_finish_tick = 20;
  assert(wait_goal(true) == F413_RUN_SESSION_ABORT_NONE && tick == 40);
  /* An early encoder crossing cannot skip the zero-speed profile. */
  prepare_goal(); position = 101;
  assert(wait_goal(true) == F413_RUN_SESSION_ABORT_TIMEOUT);
  assert(tick == NIGHTFALL_F413_PATH_TIMEOUT_MS);
  prepare_goal(); stop_profile_finish_tick = 20; velocity = 21;
  assert(wait_goal(true) == F413_RUN_SESSION_ABORT_TIMEOUT && tick == 270);
  prepare_goal(); position = 96.99f; stop_profile_finish_tick = 20;
  assert(wait_goal(true) == F413_RUN_SESSION_ABORT_TIMEOUT && tick == 270);
  prepare_goal(); position = 103.01f; stop_profile_finish_tick = 20;
  assert(wait_goal(true) == F413_RUN_SESSION_ABORT_TIMEOUT && tick == 270);
  prepare_goal(); stop_profile_finish_tick = 20; drive_observer = speed_spike;
  assert(wait_goal(true) == F413_RUN_SESSION_ABORT_NONE && tick == 51);
  prepare_goal(); stop_profile_done = true; wait_extra_ms = 3;
  assert(wait_goal(true) == F413_RUN_SESSION_ABORT_NONE && tick == 20);
  prepare_goal(); velocity = 30;
  assert(wait_goal(false) == F413_RUN_SESSION_ABORT_TIMEOUT && tick == 250);
  prepare_goal(); stop_profile_done = true; tick = UINT32_MAX - 10;
  assert(wait_goal(true) == F413_RUN_SESSION_ABORT_NONE && tick == 9);
  prepare_goal(); stop_profile_done = true; tick = UINT32_MAX - 10; velocity = 30;
  assert(wait_goal(true) == F413_RUN_SESSION_ABORT_TIMEOUT && tick == 239);
  for (unsigned invalid = 0; invalid < 3; ++invalid)
  {
    prepare_goal(); stop_profile_done = true;
    if (invalid == 0) position = NAN;
    if (invalid == 1) velocity = INFINITY;
    if (invalid == 2) g_f413_path_run_distance_cursor.active = false;
    assert(wait_goal(true) == F413_RUN_SESSION_ABORT_IMU_FAULT && tick == 0);
  }
  prepare_goal(); position = 97.892f; stop_profile_done = true;
  velocity = logged_velocity[0]; drive_observer = replay_stop;
  assert(wait_goal(true) == F413_RUN_SESSION_ABORT_NONE && tick == 68);

  /* If polling already consumed the final distance, commanding zero must
   * still wait for measured speed; no stale profile-complete bit is needed. */
  prepare_goal(); position = 145; target = 145;
  float speed = 500;
  assert(f413_path_run_drive_segment_impl(45, 0, &speed, &goal_guard, 0,
      false, false, true) == F413_RUN_SESSION_ABORT_NONE);
  assert(tick == 20 && speed == 0 && profiles == 1);

  for (unsigned failure = 0; failure < 2; ++failure)
  {
    prepare_session(failure == 0 ? 4 : 2); force_moving = failure == 1;
    run(); stopped(1);
    assert(completed_sessions == 0);
    assert(fan_stop_tick - goal_begin == 40 + 250 + NIGHTFALL_F413_PATH_COAST_MS);
  }
  for (unsigned reason = F413_RUN_SESSION_ABORT_SWITCH;
       reason <= F413_RUN_SESSION_ABORT_TIMEOUT; ++reason)
  {
    prepare_session(2); goal_abort = reason;
    run(); stopped(1);
    assert(completed_sessions == 0 && fan_stop_tick - goal_begin <= 101);
  }
  puts("goal stop: logged stop ripple, shortfall/overshoot, profile/speed/dwell, wrap, faults, guards, bounded cleanup/fan/video order PASS");
}
