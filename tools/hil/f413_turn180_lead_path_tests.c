/* Production executor, synthetic odometry/sensors; no hardware operations. */
#define main chained_front_regression_main
#include "f413_chained_front_tests.c"
#undef main

static void session_matrix(void)
{
  const uint16_t codes[] = {300, 400, 501, 601, 502, 602};
  for (uint8_t mode = 2; mode <= 7; ++mode)
  for (unsigned code = 0; code < sizeof(codes)/sizeof(codes[0]); ++code)
  for (unsigned test = 0; test < 2; ++test)
  {
    reset();
    f413_run_features_t features = f413_run_features_get();
    features.test_mode_run = test != 0;
    f413_run_features_set(&features);
    path[0] = 220; path[1] = codes[code]; path[2] = 220;
    f413_path_run_session_once(mode, 2, 0, "lead matrix");
    assert(completed == 1 && aborted == 0 && cores == 1);
    const bool expected = F413_MOTION_ENABLED(F413_MOTION_MODE4_180_LEAD) &&
        mode == 4 && (codes[code] == 502 || codes[code] == 602);
    assert(core_lead[0] == expected);
    assert(lead_begins == (unsigned)expected && lead_ends == lead_begins && !turn_lead);
    if (expected) {
      near(lead_begin_pos[0], core_start[0]); /* Entry is still the default. */
      assert(lead_end_pos[0] >= core_end[0]);
      f413_path_run_turn_t turn;
      assert(f413_path_run_turn_from_code(codes[code], f413_path_run_mode_params(mode), &turn));
      assert(lead_end_pos[0] - core_end[0] <= turn.dist_out_mm + turn.velocity_mm_s * .001f + .002f);
    }
  }
  /* Consecutive 180s and a following non-target turn cannot inherit the context. */
  reset();
  path[0] = 220; path[1] = 502; path[2] = 602; path[3] = 501; path[4] = 220;
  f413_path_run_session_once(4, 3, 0, "lead chain");
  assert(completed == 1 && aborted == 0 && cores == 3);
  assert(core_lead[0] == F413_MOTION_ENABLED(F413_MOTION_MODE4_180_LEAD));
  assert(core_lead[1] == core_lead[0] && !core_lead[2] && !turn_lead);
  assert(lead_begins == lead_ends);
}

static void exit_and_abort_tests(void)
{
  for (unsigned exit_kind = 0; exit_kind < 5; ++exit_kind)
  for (unsigned phase = 0; phase < 4; ++phase)
  for (unsigned failure = F413_RUN_SESSION_ABORT_SWITCH; failure <= F413_RUN_SESSION_ABORT_TIMEOUT; ++failure)
  {
    reset();
    f413_path_run_turn_t turn, next;
    assert(f413_path_run_turn_from_code(502, f413_path_run_mode_params(4), &turn));
    assert(f413_path_run_turn_from_code(exit_kind == 2 ? 300 : 602,
        f413_path_run_mode_params(4), &next));
    turn.dist_in_mm = 3; turn.dist_out_mm = exit_kind == 4 ? 0 : 13;
    turn.wall_control_offsets = exit_kind == 3;
    front_target = F_ALIGN_TARGET_MM + DIST_HALF_SEC - next.dist_in_mm;
    hit_after[0] = 4;
    wall_end_after = 4;
    if (phase == 1) abort_entry = 1;
    if (phase == 2) abort_core = 1;
    if (phase == 3) abort_exit = 1;
    injected_abort = failure;
    float speed = turn.velocity_mm_s;
    f413_run_session_guard_t guard = {0}; bool skip;
    const int result = f413_path_run_wait_smooth_turn_profile(&turn,
        exit_kind == 1 || exit_kind == 2 ? &next : NULL, true, false, &skip,
        2, &speed, &guard, NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG);
    const bool should_abort = phase != 0 && !(exit_kind == 4 && phase == 3);
    assert(result == (should_abort ? (int)failure : F413_RUN_SESSION_ABORT_NONE));
    assert(!turn_lead && lead_begins == lead_ends);
    if (phase == 1) assert(lead_begins == 0 && cores == 0);
    else if (F413_MOTION_ENABLED(F413_MOTION_MODE4_180_LEAD)) {
      assert(lead_begins == 1);
      near(lead_begin_pos[0], core_start[0]);
      near(lead_end_pos[0], position);
      if (phase == 0 && exit_kind != 4) assert(position > core_end[0]);
    }
  }
}

int main(void)
{
  session_matrix();
  exit_and_abort_tests();
  printf("PASS: mode2..7 R/L90/180, test/maze, chained/normal/wall-end/zero exits and all abort phases; policy=%#x\n",
      (unsigned)F413_MOTION_FEATURES);
  return 0;
}
