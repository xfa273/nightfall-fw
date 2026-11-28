/* Frozen motion implementation from 57a9c3b (2026-08-29).
 * Included only by f413_search_step.c. Keep r2 behavior changes explicit;
 * re-adopt the newer recovery through F413_MOTION_FRONT_RECOVERY.
 * Shared sensor conversion, filter and controller helpers remain unchanged. */

static f413_run_session_abort_reason_t f413_search_step_match_front_position_aug29(
    f413_run_session_guard_t* guard,
    f413_search_step_front_match_result_t* result);
static bool f413_search_step_front_match_wall_detected_aug29(
    const f413_wall_distance_snapshot_t* distance);
static bool f413_search_step_front_match_too_close_aug29(
    const f413_wall_distance_snapshot_t* distance);
static uint16_t f413_search_step_front_match_motor_flags_aug29(float v_cmd,
                                                         float w_cmd);
static f413_run_session_abort_reason_t f413_search_step_front_match_stop_and_wait_aug29(
    f413_run_session_guard_t* guard,
    const char* reason_text,
    f413_front_match_phase_t pause_phase,
    bool* control_running);
static f413_run_session_abort_reason_t f413_search_step_front_match_backoff_too_close_aug29(
    f413_run_session_guard_t* guard,
    bool* control_running,
    bool wait_after_limit);
static f413_run_session_abort_reason_t f413_search_step_front_match_continuous_aug29(
    f413_run_session_guard_t* guard);

static f413_run_session_abort_reason_t f413_search_step_match_front_position_aug29(
    f413_run_session_guard_t* guard,
    f413_search_step_front_match_result_t* result)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  f413_search_step_front_match_filter_t filter = {0};
  f413_front_match_controller_t controller;
  bool control_running = true;
  uint16_t relaxed_stable_ms = 0U;
  uint32_t start_ms = f413_search_step_tick();
  uint32_t count;

  if (result != NULL)
  {
    memset(result, 0, sizeof(*result));
    result->status = F413_SEARCH_FRONT_MATCH_NOT_RUN;
  }
  if (!f413_search_step_front_wall_strong_present())
  {
    return F413_RUN_SESSION_ABORT_NONE;
  }
  if (result != NULL)
  {
    result->status = F413_SEARCH_FRONT_MATCH_TIMEOUT;
  }

  f413_wall_runtime_control_clear();
  f413_ctrl_clear_angle_target();
  f413_front_match_init(&controller);

  for (count = 0U; count < MATCH_POS_MAX_DURATION_MS; count++)
  {
    f413_wall_sensor_snapshot_t wall;
    f413_wall_distance_snapshot_t distance;
    float e_pos_mm;
    float e_yaw_mm;
    f413_front_match_output_t output;

    if (!f413_search_step_read_wall(&wall))
    {
      reason = F413_RUN_SESSION_ABORT_WALL_FAULT;
      if (result != NULL)
      {
        result->status = F413_SEARCH_FRONT_MATCH_ABORTED;
      }
      break;
    }
    if (!f413_wall_distance_convert_snapshot(&wall, &distance))
    {
      reason = F413_RUN_SESSION_ABORT_WALL_FAULT;
      if (result != NULL)
      {
        result->status = F413_SEARCH_FRONT_MATCH_ABORTED;
      }
      break;
    }
    if (f413_search_step_front_match_too_close_aug29(&distance))
    {
      reason = f413_search_step_front_match_backoff_too_close_aug29(
          guard,
          &control_running,
          false);
      f413_search_step_front_match_filter_reset(&filter);
      f413_front_match_init(&controller);
      f413_ctrl_clear_angle_target();
      relaxed_stable_ms = 0U;
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        if (result != NULL)
        {
          result->status = F413_SEARCH_FRONT_MATCH_ABORTED;
        }
        break;
      }
      continue;
    }
    if (!distance.front_valid ||
        ((float)distance.adc.fr_delta <= ((float)WALL_BASE_FR * 1.5f)) ||
        ((float)distance.adc.fl_delta <= ((float)WALL_BASE_FL * 1.5f)))
    {
      if (result != NULL)
      {
        result->status = F413_SEARCH_FRONT_MATCH_WALL_LOST;
      }
      break;
    }

    f413_search_step_front_match_filter_update(&filter, &distance);
    e_pos_mm = f413_search_step_front_match_error_position(&filter);
    e_yaw_mm = f413_search_step_front_match_error_yaw(&filter);
    if (result != NULL)
    {
      result->position_error_mm = e_pos_mm;
      result->yaw_error_mm = e_yaw_mm;
    }
    f413_front_match_step(&controller, e_pos_mm, e_yaw_mm, 1U, &output);
    f413_search_step_front_match_sync_angle_target(&output);
    if (output.complete)
    {
      if (result != NULL)
      {
        result->status = F413_SEARCH_FRONT_MATCH_COMPLETE;
      }
      break;
    }
    if ((count + 1U) >= MATCH_POS_RELAXED_AFTER_MS)
    {
      if ((fabsf(e_pos_mm) <= MATCH_POS_RELAXED_TRANS_TOL_MM) &&
          (fabsf(e_yaw_mm) <= MATCH_POS_RELAXED_YAW_TOL_MM))
      {
        if (relaxed_stable_ms < MATCH_POS_RELAXED_SETTLE_MS)
        {
          relaxed_stable_ms++;
        }
      }
      else
      {
        relaxed_stable_ms = 0U;
      }
      if (relaxed_stable_ms >= MATCH_POS_RELAXED_SETTLE_MS)
      {
        if (result != NULL)
        {
          result->status = F413_SEARCH_FRONT_MATCH_RELAXED;
        }
        break;
      }
    }

    f413_ctrl_set_velocity(output.velocity_mm_s);
    f413_ctrl_set_omega(output.omega_deg_s);
    f413_search_step_set_mode_flags(
        f413_search_step_front_match_motor_flags_aug29(output.velocity_mm_s,
                                                 output.omega_deg_s));
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }
  }

  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  f413_ctrl_set_angle_target(f413_ctrl_get_angle());
  if (result != NULL)
  {
    const uint32_t elapsed_ms = f413_search_step_tick() - start_ms;
    result->elapsed_ms = (elapsed_ms > UINT16_MAX) ? UINT16_MAX : (uint16_t)elapsed_ms;
  }
  if ((reason == F413_RUN_SESSION_ABORT_NONE) &&
      (result != NULL) &&
      ((result->status == F413_SEARCH_FRONT_MATCH_COMPLETE) ||
       (result->status == F413_SEARCH_FRONT_MATCH_RELAXED)) &&
      (MATCH_POS_POST_COMPLETE_DELAY_MS > 0U))
  {
    f413_search_step_set_mode_flags((uint16_t)(g_config.trace_search_safe_flag |
                                               g_config.trace_motor_coast_flag));
    reason = f413_run_session_wait_with_auto_step_guarded(
        MATCH_POS_POST_COMPLETE_DELAY_MS,
        guard);
  }
  return reason;
}

static bool f413_search_step_front_match_wall_detected_aug29(
    const f413_wall_distance_snapshot_t* distance)
{
  return (distance != NULL) && distance->front_valid;
}

static bool f413_search_step_front_match_too_close_aug29(
    const f413_wall_distance_snapshot_t* distance)
{
  return (distance != NULL) &&
         ((distance->fr_mm_unwarped < F_ALIGN_TOO_CLOSE_MM) ||
          (distance->fl_mm_unwarped < F_ALIGN_TOO_CLOSE_MM));
}

static uint16_t f413_search_step_front_match_motor_flags_aug29(float v_cmd,
                                                         float w_cmd)
{
  uint16_t flags = g_config.trace_search_safe_flag;

  if ((fabsf(v_cmd) <= 0.01f) && (fabsf(w_cmd) <= 0.01f))
  {
    flags = (uint16_t)(flags | g_config.trace_motor_coast_flag);
  }
  else if (v_cmd < -0.01f)
  {
    flags = (uint16_t)(flags | g_config.trace_motor_rev_flag);
  }
  else
  {
    flags = (uint16_t)(flags | g_config.trace_motor_fwd_flag);
  }
  return flags;
}

static f413_run_session_abort_reason_t f413_search_step_front_match_stop_and_wait_aug29(
    f413_run_session_guard_t* guard,
    const char* reason_text,
    f413_front_match_phase_t pause_phase,
    bool* control_running)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  uint16_t resume_count = 0U;
  uint32_t last_print_ms = f413_search_step_tick();
  const uint16_t stop_flags = g_config.trace_search_safe_flag;

  f413_ctrl_stop();
  if (control_running != NULL)
  {
    *control_running = false;
  }
  f413_search_step_set_trace_period(MATCH_POS_TRACE_IDLE_PERIOD_MS);
  f413_search_step_set_mode_flags(stop_flags);
  trace_printf("[SEARCH-TEST] front-match pause(%s)\r\n", reason_text);

  while (resume_count < MATCH_POS_RECOVERY_VALID_MS)
  {
    f413_wall_sensor_snapshot_t wall;
    f413_wall_distance_snapshot_t distance;
    uint32_t now_ms;

    if (!f413_search_step_read_wall(&wall))
    {
      return F413_RUN_SESSION_ABORT_WALL_FAULT;
    }
    if (!f413_wall_distance_convert_snapshot(&wall, &distance))
    {
      return F413_RUN_SESSION_ABORT_WALL_FAULT;
    }
    f413_search_step_set_front_match_trace(
        true,
        pause_phase,
        distance.fr_mm_unwarped,
        distance.fl_mm_unwarped,
        distance.front_sum_mm_unwarped - F_ALIGN_TARGET_MM,
        distance.fr_mm_unwarped - distance.fl_mm_unwarped,
        resume_count);
    if (f413_search_step_front_match_wall_detected_aug29(&distance) &&
        !f413_search_step_front_match_too_close_aug29(&distance))
    {
      resume_count++;
    }
    else
    {
      resume_count = 0U;
    }

    now_ms = f413_search_step_tick();
    if ((now_ms - last_print_ms) >= 500U)
    {
      trace_printf("[SEARCH-TEST] front-match waiting FR=%ld/%.2fmm FL=%ld/%.2fmm valid=%u\r\n",
                   (long)distance.adc.fr_delta,
                   (double)distance.fr_mm_unwarped,
                   (long)distance.adc.fl_delta,
                   (double)distance.fl_mm_unwarped,
                   (unsigned int)distance.front_valid);
      last_print_ms = now_ms;
    }

    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
  }

  f413_ctrl_start();
  if (control_running != NULL)
  {
    *control_running = true;
  }
  f413_ctrl_clear_angle_target();
  trace_printf("[SEARCH-TEST] front-match resume\r\n");
  return F413_RUN_SESSION_ABORT_NONE;
}

static f413_run_session_abort_reason_t f413_search_step_front_match_backoff_too_close_aug29(
    f413_run_session_guard_t* guard,
    bool* control_running,
    bool wait_after_limit)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  uint16_t recovery_count = 0U;
  uint32_t start_ms;
  float start_distance;

  if ((control_running == NULL) || !*control_running)
  {
    f413_search_step_set_mode_flags(g_config.trace_search_safe_flag);
    f413_ctrl_start();
    if (control_running != NULL)
    {
      *control_running = true;
    }
  }

  start_ms = f413_search_step_tick();
  /* Outside the near LUT edge, only the safe retreat direction is authoritative. */
  f413_ctrl_clear_angle_target();
  f413_ctrl_set_omega(0.0f);
  f413_ctrl_set_velocity(-fabsf(MATCH_POS_TOO_CLOSE_RECOVERY_VEL_MM_S));
  f413_search_step_set_trace_period(MATCH_POS_TRACE_PERIOD_MS);
  f413_search_step_set_mode_flags(
      f413_search_step_front_match_motor_flags_aug29(
          -fabsf(MATCH_POS_TOO_CLOSE_RECOVERY_VEL_MM_S),
          0.0f));
  start_distance = f413_ctrl_get_distance();
  trace_printf("[SEARCH-TEST] front-match backoff-too-close\r\n");

  while (reason == F413_RUN_SESSION_ABORT_NONE)
  {
    f413_wall_sensor_snapshot_t wall;
    f413_wall_distance_snapshot_t distance;
    const uint32_t elapsed_ms = f413_search_step_tick() - start_ms;
    const float backed_off_mm = start_distance - f413_ctrl_get_distance();
    const uint16_t trace_elapsed_ms =
        (elapsed_ms > UINT16_MAX) ? UINT16_MAX : (uint16_t)elapsed_ms;

    if (!f413_search_step_read_wall(&wall) ||
        !f413_wall_distance_convert_snapshot(&wall, &distance))
    {
      reason = F413_RUN_SESSION_ABORT_WALL_FAULT;
      break;
    }

    f413_search_step_set_front_match_trace(
        true,
        F413_FRONT_MATCH_PHASE_BACKOFF_TOO_CLOSE,
        distance.fr_mm_unwarped,
        distance.fl_mm_unwarped,
        distance.front_sum_mm_unwarped - F_ALIGN_TARGET_MM,
        distance.fr_mm_unwarped - distance.fl_mm_unwarped,
        trace_elapsed_ms);

    if (f413_search_step_front_match_wall_detected_aug29(&distance) &&
        !f413_search_step_front_match_too_close_aug29(&distance))
    {
      recovery_count++;
    }
    else
    {
      recovery_count = 0U;
    }

    if (recovery_count >= MATCH_POS_RECOVERY_VALID_MS)
    {
      trace_printf("[SEARCH-TEST] front-match backoff complete dist=%.2fmm time=%lums\r\n",
                   (double)backed_off_mm,
                   (unsigned long)elapsed_ms);
      break;
    }

    if ((backed_off_mm >= MATCH_POS_TOO_CLOSE_RECOVERY_MAX_MM) ||
        (elapsed_ms >= MATCH_POS_TOO_CLOSE_RECOVERY_MAX_MS))
    {
      f413_ctrl_set_velocity(0.0f);
      f413_ctrl_set_omega(0.0f);
      trace_printf("[SEARCH-TEST] front-match backoff limit dist=%.2fmm time=%lums\r\n",
                   (double)backed_off_mm,
                   (unsigned long)elapsed_ms);
      if (wait_after_limit)
      {
        return f413_search_step_front_match_stop_and_wait_aug29(
            guard,
            "too-close-backoff-limit",
            F413_FRONT_MATCH_PHASE_PAUSED_TOO_CLOSE,
            control_running);
      }
      return F413_RUN_SESSION_ABORT_WALL_FAULT;
    }

    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
  }

  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  return reason;
}

static f413_run_session_abort_reason_t f413_search_step_front_match_continuous_aug29(
    f413_run_session_guard_t* guard)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  f413_search_step_front_match_filter_t filter = {0};
  f413_front_match_controller_t controller;
  bool control_running = true;

  f413_wall_runtime_control_clear();
  f413_ctrl_clear_angle_target();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  f413_front_match_init(&controller);

  trace_printf("[SEARCH-TEST] front-match continuous: stop switch exits, reset also safe\r\n");

  while (reason == F413_RUN_SESSION_ABORT_NONE)
  {
    f413_wall_sensor_snapshot_t wall;
    f413_wall_distance_snapshot_t distance;
    float e_pos_mm;
    float e_yaw_mm;
    f413_front_match_output_t output;

    if (!f413_search_step_read_wall(&wall))
    {
      return F413_RUN_SESSION_ABORT_WALL_FAULT;
    }
    if (!f413_wall_distance_convert_snapshot(&wall, &distance))
    {
      return F413_RUN_SESSION_ABORT_WALL_FAULT;
    }
    if (f413_search_step_front_match_too_close_aug29(&distance))
    {
      reason = f413_search_step_front_match_backoff_too_close_aug29(
          guard,
          &control_running,
          true);
      f413_search_step_front_match_filter_reset(&filter);
      f413_front_match_init(&controller);
      f413_ctrl_clear_angle_target();
      f413_search_step_set_trace_period(MATCH_POS_TRACE_PERIOD_MS);
      continue;
    }
    if (!f413_search_step_front_match_wall_detected_aug29(&distance))
    {
      reason = f413_search_step_front_match_stop_and_wait_aug29(
          guard,
          "front-wall-lost",
          F413_FRONT_MATCH_PHASE_PAUSED_WALL_LOST,
          &control_running);
      f413_search_step_front_match_filter_reset(&filter);
      f413_front_match_init(&controller);
      f413_ctrl_clear_angle_target();
      f413_search_step_set_trace_period(MATCH_POS_TRACE_PERIOD_MS);
      continue;
    }

    f413_search_step_front_match_filter_update(&filter, &distance);
    e_pos_mm = f413_search_step_front_match_error_position(&filter);
    e_yaw_mm = f413_search_step_front_match_error_yaw(&filter);
    f413_front_match_step(&controller, e_pos_mm, e_yaw_mm, 1U, &output);
    if (!control_running && !output.holding)
    {
      f413_ctrl_start();
      control_running = true;
      if (output.phase == F413_FRONT_MATCH_PHASE_ALIGN_YAW)
      {
        f413_ctrl_clear_angle_target();
      }
      else
      {
        f413_ctrl_set_angle_target(f413_ctrl_get_angle());
      }
    }
    else if (control_running)
    {
      f413_search_step_front_match_sync_angle_target(&output);
    }
    if (output.phase_changed)
    {
      trace_printf("[SEARCH-TEST] front-match phase=%s pos=%.2fmm yaw=%.2fmm\r\n",
                   f413_front_match_phase_name(output.phase),
                   (double)e_pos_mm,
                   (double)e_yaw_mm);
    }

    if (output.holding)
    {
      if (control_running)
      {
        f413_ctrl_stop();
        control_running = false;
      }
      f413_search_step_set_mode_flags(g_config.trace_search_safe_flag);
    }
    else
    {
      f413_ctrl_set_velocity(output.velocity_mm_s);
      f413_ctrl_set_omega(output.omega_deg_s);
      f413_search_step_set_mode_flags(
          f413_search_step_front_match_motor_flags_aug29(output.velocity_mm_s,
                                                   output.omega_deg_s));
    }
    f413_search_step_set_trace_period(output.holding
                                          ? MATCH_POS_TRACE_IDLE_PERIOD_MS
                                          : MATCH_POS_TRACE_PERIOD_MS);
    f413_search_step_set_front_match_trace(
        true,
        output.phase,
        filter.fr_mm,
        filter.fl_mm,
        e_pos_mm,
        e_yaw_mm,
        f413_search_step_front_match_state_elapsed(&controller));
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
  }

  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  return reason;
}
