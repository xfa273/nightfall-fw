#include "f413_path_run.h"

#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)

#include <math.h>

#include "f413_control.h"
#include "f413_hw.h"
#include "f413_run_features.h"
#include "f413_run_session.h"
#include "f413_trace_flags.h"
#include "f413_trace_log.h"
#include "f413_trace_sample.h"
#include "f413_wall_runtime.h"
#include "main.h"
#include "params.h"
#include "shortest_run_params.h"
#include "trace.h"

typedef struct {
  float signed_angle_deg;
  float alpha_deg_s2;
  float velocity_mm_s;
  float dist_in_mm;
  float dist_out_mm;
  bool front_wall_entry;
  bool wall_control_offsets;
} f413_path_run_turn_t;

typedef struct {
  float omega_peak_deg_s;
  float t_acc_s;
  float t_cruise_s;
  float t_total_s;
} f413_path_run_smooth_turn_t;

extern uint16_t path[];

static const ShortestRunModeParams_t* f413_path_run_mode_params(uint8_t mode)
{
  switch (mode)
  {
    case 2U: return &shortestRunModeParams2;
    case 3U: return &shortestRunModeParams3;
    case 4U: return &shortestRunModeParams4;
    case 5U: return &shortestRunModeParams5;
    case 6U: return &shortestRunModeParams6;
    case 7U: return &shortestRunModeParams7;
    default: return &shortestRunModeParams2;
  }
}

static const ShortestRunCaseParams_t* f413_path_run_case_params(uint8_t mode, uint8_t case_index)
{
  uint8_t idx = 0U;

  if ((case_index >= 1U) && (case_index <= 9U))
  {
    idx = (uint8_t)(case_index - 1U);
  }

  switch (mode)
  {
    case 2U: return &shortestRunCaseParamsMode2[idx];
    case 3U: return &shortestRunCaseParamsMode3[idx];
    case 4U: return &shortestRunCaseParamsMode4[idx];
    case 5U: return &shortestRunCaseParamsMode5[idx];
    case 6U: return &shortestRunCaseParamsMode6[idx];
    case 7U: return &shortestRunCaseParamsMode7[idx];
    default: return &shortestRunCaseParamsMode2[idx];
  }
}

static float f413_path_run_velocity_or_cap(float candidate, float fallback)
{
  float v = candidate;

  if (v <= 0.0f)
  {
    v = fallback;
  }
  if (v <= 0.0f)
  {
    v = NIGHTFALL_F413_PATH_VELOCITY;
  }
  if ((NIGHTFALL_F413_PATH_VELOCITY_CAP > 0.0f) &&
      (v > NIGHTFALL_F413_PATH_VELOCITY_CAP))
  {
    v = NIGHTFALL_F413_PATH_VELOCITY_CAP;
  }

  return v;
}

static float f413_path_run_cap_positive(float value, float cap)
{
  if (value < 0.0f)
  {
    value = -value;
  }
  if ((cap > 0.0f) && (value > cap))
  {
    value = cap;
  }
  return value;
}

static uint16_t f413_path_run_motor_phase_flags(uint16_t trace_flags, uint16_t motor_flag)
{
  const uint16_t motor_mask = (uint16_t)(NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG |
                                        NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG |
                                        NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG);
  return (uint16_t)((trace_flags & (uint16_t)~motor_mask) | motor_flag);
}

static void f413_path_run_prepare_straight_angle_control(void)
{
  if (!f413_run_features_angle_accum_mode())
  {
    f413_ctrl_reset_angle();
  }
  f413_ctrl_clear_angle_target();
}

static void f413_path_run_prepare_turn_angle_control(void)
{
  if (!f413_run_features_angle_accum_mode())
  {
    f413_ctrl_reset_angle();
  }
  f413_ctrl_clear_angle_target();
  f413_wall_runtime_control_clear();
}

static bool f413_path_run_turn_from_code(uint16_t code,
                                         const ShortestRunModeParams_t* params,
                                         f413_path_run_turn_t* turn)
{
  float angle = 0.0f;
  float accum_angle = 0.0f;
  float alpha = 0.0f;
  float velocity = 0.0f;
  bool right = false;

  if ((params == NULL) || (turn == NULL))
  {
    return false;
  }
  turn->front_wall_entry = false;
  turn->wall_control_offsets = false;

  switch (code)
  {
    case 300U:
      angle = params->angle_turn_90; alpha = params->alpha_turn90;
      accum_angle = 90.0f;
      velocity = params->velocity_turn90;
      turn->dist_in_mm = params->dist_offset_in;
      turn->dist_out_mm = params->dist_offset_out;
      turn->front_wall_entry = true;
      turn->wall_control_offsets = true;
      right = true; break;
    case 400U:
      angle = params->angle_turn_90; alpha = params->alpha_turn90;
      accum_angle = 90.0f;
      velocity = params->velocity_turn90;
      turn->dist_in_mm = params->dist_offset_in;
      turn->dist_out_mm = params->dist_offset_out;
      turn->front_wall_entry = true;
      turn->wall_control_offsets = true;
      right = false; break;
    case 501U:
      angle = params->angle_l_turn_90; alpha = params->alpha_l_turn_90;
      accum_angle = 90.0f;
      velocity = params->velocity_l_turn_90;
      turn->dist_in_mm = params->dist_l_turn_in_90;
      turn->dist_out_mm = params->dist_l_turn_out_90;
      turn->front_wall_entry = false;
      turn->wall_control_offsets = true;
      right = true; break;
    case 601U:
      angle = params->angle_l_turn_90; alpha = params->alpha_l_turn_90;
      accum_angle = 90.0f;
      velocity = params->velocity_l_turn_90;
      turn->dist_in_mm = params->dist_l_turn_in_90;
      turn->dist_out_mm = params->dist_l_turn_out_90;
      turn->front_wall_entry = false;
      turn->wall_control_offsets = true;
      right = false; break;
    case 502U:
      angle = params->angle_l_turn_180; alpha = params->alpha_l_turn_180;
      accum_angle = 180.0f;
      velocity = params->velocity_l_turn_180;
      turn->dist_in_mm = params->dist_l_turn_in_180;
      turn->dist_out_mm = params->dist_l_turn_out_180;
      turn->front_wall_entry = false;
      turn->wall_control_offsets = true;
      right = true; break;
    case 602U:
      angle = params->angle_l_turn_180; alpha = params->alpha_l_turn_180;
      accum_angle = 180.0f;
      velocity = params->velocity_l_turn_180;
      turn->dist_in_mm = params->dist_l_turn_in_180;
      turn->dist_out_mm = params->dist_l_turn_out_180;
      turn->front_wall_entry = false;
      turn->wall_control_offsets = true;
      right = false; break;
    case 701U:
      angle = params->angle_turn45in; alpha = params->alpha_turn45in;
      accum_angle = 45.0f;
      velocity = params->velocity_turn45in;
      turn->dist_in_mm = params->dist_turn45in_in;
      turn->dist_out_mm = params->dist_turn45in_out;
      turn->front_wall_entry = false;
      right = true; break;
    case 702U:
      angle = params->angle_turn45in; alpha = params->alpha_turn45in;
      accum_angle = 45.0f;
      velocity = params->velocity_turn45in;
      turn->dist_in_mm = params->dist_turn45in_in;
      turn->dist_out_mm = params->dist_turn45in_out;
      turn->front_wall_entry = false;
      right = false; break;
    case 703U:
      angle = params->angle_turn45out; alpha = params->alpha_turn45out;
      accum_angle = 45.0f;
      velocity = params->velocity_turn45out;
      turn->dist_in_mm = params->dist_turn45out_in;
      turn->dist_out_mm = params->dist_turn45out_out;
      turn->front_wall_entry = false;
      right = true; break;
    case 704U:
      angle = params->angle_turn45out; alpha = params->alpha_turn45out;
      accum_angle = 45.0f;
      velocity = params->velocity_turn45out;
      turn->dist_in_mm = params->dist_turn45out_in;
      turn->dist_out_mm = params->dist_turn45out_out;
      turn->front_wall_entry = false;
      right = false; break;
    case 801U:
      angle = params->angle_turnV90; alpha = params->alpha_turnV90;
      accum_angle = 90.0f;
      velocity = params->velocity_turnV90;
      turn->dist_in_mm = params->dist_turnV90_in;
      turn->dist_out_mm = params->dist_turnV90_out;
      turn->front_wall_entry = false;
      right = true; break;
    case 802U:
      angle = params->angle_turnV90; alpha = params->alpha_turnV90;
      accum_angle = 90.0f;
      velocity = params->velocity_turnV90;
      turn->dist_in_mm = params->dist_turnV90_in;
      turn->dist_out_mm = params->dist_turnV90_out;
      turn->front_wall_entry = false;
      right = false; break;
    case 901U:
      angle = params->angle_turn135in; alpha = params->alpha_turn135in;
      accum_angle = 135.0f;
      velocity = params->velocity_turn135in;
      turn->dist_in_mm = params->dist_turn135in_in;
      turn->dist_out_mm = params->dist_turn135in_out;
      turn->front_wall_entry = false;
      right = true; break;
    case 902U:
      angle = params->angle_turn135in; alpha = params->alpha_turn135in;
      accum_angle = 135.0f;
      velocity = params->velocity_turn135in;
      turn->dist_in_mm = params->dist_turn135in_in;
      turn->dist_out_mm = params->dist_turn135in_out;
      turn->front_wall_entry = false;
      right = false; break;
    case 903U:
      angle = params->angle_turn135out; alpha = params->alpha_turn135out;
      accum_angle = 135.0f;
      velocity = params->velocity_turn135out;
      turn->dist_in_mm = params->dist_turn135out_in;
      turn->dist_out_mm = params->dist_turn135out_out;
      turn->front_wall_entry = false;
      right = true; break;
    case 904U:
      angle = params->angle_turn135out; alpha = params->alpha_turn135out;
      accum_angle = 135.0f;
      velocity = params->velocity_turn135out;
      turn->dist_in_mm = params->dist_turn135out_in;
      turn->dist_out_mm = params->dist_turn135out_out;
      turn->front_wall_entry = false;
      right = false; break;
    default: return false;
  }

  if (angle <= 0.0f)
  {
    angle = 90.0f;
  }
  if (f413_run_features_angle_accum_mode() && (accum_angle > 0.0f))
  {
    angle = accum_angle;
  }
  if (alpha <= 0.0f)
  {
    alpha = 10000.0f;
  }

  turn->signed_angle_deg = right ? -fabsf(angle) : fabsf(angle);
  turn->alpha_deg_s2 = alpha;
  turn->velocity_mm_s = f413_path_run_cap_positive(velocity, NIGHTFALL_F413_PATH_VELOCITY_CAP);
  return true;
}

static f413_path_run_smooth_turn_t f413_path_run_build_smooth_turn(float angle_deg,
                                                                   float alpha_deg_s2,
                                                                   float omega_cap_deg_s)
{
  f413_path_run_smooth_turn_t profile = {0.0f, 0.0f, 0.0f, 0.0f};
  const float angle_abs = fabsf(angle_deg);
  float rounding_scale = TURN_OMEGA_PROFILE_ROUNDING_SCALE;

  if ((angle_abs <= 0.0f) || (alpha_deg_s2 <= 0.0f))
  {
    return profile;
  }

  profile.omega_peak_deg_s = sqrtf((2.0f * alpha_deg_s2 * angle_abs) / 3.0f);
  profile.omega_peak_deg_s = f413_path_run_cap_positive(profile.omega_peak_deg_s, omega_cap_deg_s);
  if (profile.omega_peak_deg_s <= 0.0f)
  {
    return profile;
  }

  if (rounding_scale < 0.1f)
  {
    rounding_scale = 0.1f;
  }

  profile.t_acc_s = (profile.omega_peak_deg_s / alpha_deg_s2) * rounding_scale;
  profile.t_cruise_s = (angle_abs / profile.omega_peak_deg_s) - profile.t_acc_s;
  if (profile.t_cruise_s < 0.0f)
  {
    profile.t_cruise_s = 0.0f;
    profile.omega_peak_deg_s = angle_abs / profile.t_acc_s;
    profile.omega_peak_deg_s = f413_path_run_cap_positive(profile.omega_peak_deg_s, omega_cap_deg_s);
    profile.t_cruise_s = (angle_abs / profile.omega_peak_deg_s) - profile.t_acc_s;
    if (profile.t_cruise_s < 0.0f)
    {
      profile.t_cruise_s = 0.0f;
    }
  }
  profile.t_total_s = (2.0f * profile.t_acc_s) + profile.t_cruise_s;
  return profile;
}

static void f413_path_run_trace_on_run_start(void)
{
  trace_printf("[TRACE-LOG] run-hook: start\r\n");
  f413_trace_log_auto_start();
}

static void f413_path_run_trace_on_run_stop(void)
{
  trace_printf("[TRACE-LOG] run-hook: stop tail=%u ms\r\n",
               (unsigned int)F413_TRACE_LOG_STOP_TAIL_MS_DEFAULT);
  f413_trace_log_auto_stop_after_tail(F413_TRACE_LOG_STOP_TAIL_MS_DEFAULT);
}

static f413_run_session_abort_reason_t f413_path_run_wait_ctrl_target(
    float target, bool is_angle,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags,
    bool wall_control_gate,
    bool diagonal_control_gate)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  uint32_t deadline = HAL_GetTick() + NIGHTFALL_F413_PATH_TIMEOUT_MS;

  while (1)
  {
    if (is_angle)
    {
      float current = f413_ctrl_get_angle();
      if (((target >= 0.0f) && (current >= target)) ||
          ((target < 0.0f) && (current <= target)))
      {
        break;
      }
    }
    else
    {
      float current = fabsf(f413_ctrl_get_distance());
      if (current >= fabsf(target))
      {
        break;
      }
    }

    if (HAL_GetTick() >= deadline)
    {
      break;
    }

    f413_trace_log_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (!is_angle && diagonal_control_gate)
    {
      f413_wall_runtime_poll_diagonal(true);
    }
    else
    {
      f413_wall_runtime_control_apply(!is_angle &&
          wall_control_gate &&
          ((trace_flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG) != 0U));
    }
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
  }
  return F413_RUN_SESSION_ABORT_NONE;
}

static f413_run_session_abort_reason_t f413_path_run_drive_segment_ex(float distance_mm,
                                                                      float target_velocity_mm_s,
                                                                      float* speed_now_mm_s,
                                                                      f413_run_session_guard_t* guard,
                                                                      uint16_t trace_flags,
                                                                      bool wall_control_gate,
                                                                      bool diagonal_control_gate)
{
  float start_velocity;
  float target_distance;
  f413_run_session_abort_reason_t reason;

  if (speed_now_mm_s == NULL)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
  if (distance_mm <= 0.0f)
  {
    *speed_now_mm_s = target_velocity_mm_s;
    return F413_RUN_SESSION_ABORT_NONE;
  }

  start_velocity = *speed_now_mm_s;
  target_distance = f413_ctrl_get_distance() + distance_mm;
  if (!wall_control_gate)
  {
    f413_wall_runtime_control_clear();
  }
  f413_path_run_prepare_straight_angle_control();
  f413_ctrl_set_velocity_profile(start_velocity, target_velocity_mm_s, distance_mm);
  f413_ctrl_set_omega(0.0f);

  reason = f413_path_run_wait_ctrl_target(target_distance, false, guard, trace_flags,
                                          wall_control_gate, diagonal_control_gate);
  *speed_now_mm_s = target_velocity_mm_s;
  return reason;
}

static f413_run_session_abort_reason_t f413_path_run_drive_segment(float distance_mm,
                                                                   float target_velocity_mm_s,
                                                                   float* speed_now_mm_s,
                                                                   f413_run_session_guard_t* guard,
                                                                   uint16_t trace_flags)
{
  return f413_path_run_drive_segment_ex(distance_mm,
                                        target_velocity_mm_s,
                                        speed_now_mm_s,
                                        guard,
                                        trace_flags,
                                        true,
                                        false);
}

static f413_run_session_abort_reason_t f413_path_run_drive_segment_no_wall(float distance_mm,
                                                                           float target_velocity_mm_s,
                                                                           float* speed_now_mm_s,
                                                                           f413_run_session_guard_t* guard,
                                                                           uint16_t trace_flags)
{
  return f413_path_run_drive_segment_ex(distance_mm,
                                        target_velocity_mm_s,
                                        speed_now_mm_s,
                                        guard,
                                        trace_flags,
                                        false,
                                        false);
}

static f413_run_session_abort_reason_t f413_path_run_drive_diagonal_segment(float distance_mm,
                                                                           float target_velocity_mm_s,
                                                                           float* speed_now_mm_s,
                                                                           f413_run_session_guard_t* guard,
                                                                           uint16_t trace_flags)
{
  return f413_path_run_drive_segment_ex(distance_mm,
                                        target_velocity_mm_s,
                                        speed_now_mm_s,
                                        guard,
                                        trace_flags,
                                        false,
                                        true);
}

static f413_run_session_abort_reason_t f413_path_run_drive_front_wall_entry_segment(
    float distance_mm,
    float target_velocity_mm_s,
    float front_threshold,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  float target_distance;
  float start_velocity;

  if (speed_now_mm_s == NULL)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
  if ((distance_mm <= 0.0f) || !f413_run_features_front_wall_correction_enabled() ||
      f413_run_features_test_mode_run())
  {
    return f413_path_run_drive_segment(distance_mm, target_velocity_mm_s,
                                      speed_now_mm_s, guard, trace_flags);
  }

  start_velocity = *speed_now_mm_s;
  target_distance = f413_ctrl_get_distance() + distance_mm;
  f413_path_run_prepare_straight_angle_control();
  f413_ctrl_set_velocity_profile(start_velocity, target_velocity_mm_s, distance_mm);
  f413_ctrl_set_omega(0.0f);

  while (1)
  {
    if (fabsf(f413_ctrl_get_distance()) >= fabsf(target_distance) ||
        f413_wall_runtime_front_wall_reached(front_threshold))
    {
      break;
    }
    f413_trace_log_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    f413_wall_runtime_poll_wall_end((trace_flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG) != 0U);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      *speed_now_mm_s = target_velocity_mm_s;
      return reason;
    }
  }

  if (!f413_wall_runtime_front_wall_reached(front_threshold) &&
      (WALL_END_EXTEND_MAX_MM > 0.0F))
  {
    const float extend_target = f413_ctrl_get_distance() + WALL_END_EXTEND_MAX_MM;
    f413_ctrl_set_velocity(target_velocity_mm_s);
    while (fabsf(f413_ctrl_get_distance()) < fabsf(extend_target))
    {
      if (f413_wall_runtime_front_wall_reached(front_threshold))
      {
        break;
      }
      f413_trace_log_set_mode_flags(trace_flags);
      reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
      f413_wall_runtime_poll_wall_end((trace_flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG) != 0U);
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }
    }
  }

  *speed_now_mm_s = target_velocity_mm_s;
  return reason;
}

static f413_run_session_abort_reason_t f413_path_run_drive_wallend_segment(
    float distance_mm,
    float target_velocity_mm_s,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags,
    bool* wall_end_found)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  float target_distance;

  if (wall_end_found != NULL)
  {
    *wall_end_found = false;
  }
  if (speed_now_mm_s == NULL)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
  if ((distance_mm <= 0.0f) || !f413_run_features_wall_end_correction_enabled())
  {
    return f413_path_run_drive_segment(distance_mm, target_velocity_mm_s,
                                      speed_now_mm_s, guard, trace_flags);
  }

  target_distance = f413_ctrl_get_distance() + distance_mm;
  f413_wall_runtime_end_clear();
  f413_path_run_prepare_straight_angle_control();
  f413_ctrl_set_velocity_profile(*speed_now_mm_s, target_velocity_mm_s, distance_mm);
  f413_ctrl_set_omega(0.0f);

  while (1)
  {
    if (f413_wall_runtime_poll_wall_end(true))
    {
      if (wall_end_found != NULL)
      {
        *wall_end_found = true;
      }
      break;
    }
    if (fabsf(f413_ctrl_get_distance()) >= fabsf(target_distance))
    {
      break;
    }
    f413_trace_log_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }
  }

  *speed_now_mm_s = target_velocity_mm_s;
  return reason;
}

static f413_run_session_abort_reason_t f413_path_run_wait_smooth_turn_profile(
    const f413_path_run_turn_t* turn,
    float front_threshold,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  f413_path_run_smooth_turn_t profile;
  uint16_t straight_trace_flags;
  uint16_t turn_trace_flags;
  uint32_t start_ms;
  int8_t turn_sign;

  if (turn == NULL)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  profile = f413_path_run_build_smooth_turn(turn->signed_angle_deg,
                                            turn->alpha_deg_s2,
                                            NIGHTFALL_F413_PATH_OMEGA_CAP);
  if (profile.t_total_s <= 0.0f)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  turn_sign = (turn->signed_angle_deg < 0.0f) ? -1 : 1;
  trace_printf("[OP-UI][PATH-TEST] turn profile angle=%.1f alpha=%.0f omega=%.0f v=%.0f t=%.0fms\r\n",
               (double)turn->signed_angle_deg,
               (double)turn->alpha_deg_s2,
               (double)profile.omega_peak_deg_s,
               (double)turn->velocity_mm_s,
               (double)(profile.t_total_s * 1000.0f));

  straight_trace_flags = f413_path_run_motor_phase_flags(
      trace_flags,
      turn->wall_control_offsets ? NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG : 0U);
  turn_trace_flags = f413_path_run_motor_phase_flags(trace_flags,
                                                     NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG);

  if (turn->front_wall_entry)
  {
    reason = f413_path_run_drive_front_wall_entry_segment(turn->dist_in_mm,
                                                          turn->velocity_mm_s,
                                                          front_threshold,
                                                          speed_now_mm_s,
                                                          guard,
                                                          straight_trace_flags);
  }
  else if (turn->wall_control_offsets)
  {
    reason = f413_path_run_drive_segment(turn->dist_in_mm,
                                         turn->velocity_mm_s,
                                         speed_now_mm_s,
                                         guard,
                                         straight_trace_flags);
  }
  else
  {
    reason = f413_path_run_drive_segment_no_wall(turn->dist_in_mm,
                                                 turn->velocity_mm_s,
                                                 speed_now_mm_s,
                                                 guard,
                                                 straight_trace_flags);
  }
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  f413_path_run_prepare_turn_angle_control();
  f413_ctrl_set_velocity(turn->velocity_mm_s);
  f413_ctrl_start_omega_profile((float)turn_sign * profile.omega_peak_deg_s,
                                profile.t_acc_s,
                                profile.t_cruise_s);
  start_ms = HAL_GetTick();

  while (1)
  {
    const float t_s = (float)(HAL_GetTick() - start_ms) * 0.001f;

    if (t_s >= profile.t_total_s)
    {
      break;
    }

    f413_trace_log_set_mode_flags(turn_trace_flags);
    f413_ctrl_set_velocity(turn->velocity_mm_s);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      f413_ctrl_stop_omega_profile();
      return reason;
    }
  }

  f413_ctrl_stop_omega_profile();
  if (turn->wall_control_offsets)
  {
    return f413_path_run_drive_segment(turn->dist_out_mm,
                                       turn->velocity_mm_s,
                                       speed_now_mm_s,
                                       guard,
                                       straight_trace_flags);
  }
  return f413_path_run_drive_segment_no_wall(turn->dist_out_mm,
                                             turn->velocity_mm_s,
                                             speed_now_mm_s,
                                             guard,
                                             straight_trace_flags);
}

static float f413_path_run_goal_entry_speed(const ShortestRunCaseParams_t* case_params)
{
  float speed;

  if (case_params == NULL)
  {
    return 0.0f;
  }
  speed = sqrtf(fmaxf(0.0f, 2.0f * case_params->acceleration_straight * (float)DIST_HALF_SEC));
  if (speed > case_params->velocity_straight)
  {
    speed = case_params->velocity_straight;
  }
  return speed;
}

static float f413_path_run_next_straight_exit_velocity(uint16_t next_code,
                                                       const ShortestRunModeParams_t* mode_params,
                                                       const ShortestRunCaseParams_t* case_params)
{
  f413_path_run_turn_t next_turn;

  if (f413_path_run_turn_from_code(next_code, mode_params, &next_turn))
  {
    return next_turn.velocity_mm_s;
  }
  if (next_code == 0U)
  {
    return f413_path_run_goal_entry_speed(case_params);
  }
  return 0.0f;
}

static f413_run_session_abort_reason_t f413_path_run_run_straight_code(
    uint16_t code,
    uint16_t prev_code,
    uint16_t next_code,
    const ShortestRunModeParams_t* mode_params,
    const ShortestRunCaseParams_t* case_params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  const float straight_mm = (float)(code - 200U) * (float)DIST_HALF_SEC;
  const float v_next =
      f413_path_run_next_straight_exit_velocity(next_code, mode_params, case_params);
  const float v_start = 0.0f;
  const float v_switch = (mode_params != NULL) ? mode_params->accel_switch_velocity : 0.0f;
  const float v_max = (case_params != NULL) ? case_params->velocity_straight : 0.0f;
  const float accel_low = (case_params != NULL) ? case_params->acceleration_straight : 0.0f;
  const float accel_high = (case_params != NULL) ? case_params->acceleration_straight_dash : 0.0f;
  bool two_stage;
  float d_acc = 0.0f;
  float d_constant = 0.0f;
  float max_reached_speed = v_max;
  f413_run_session_abort_reason_t reason;
  bool next_is_small_turn;
  bool next_is_large_turn;
  bool prev_is_small_turn;
  bool prev_is_large_turn;
  bool skip_wallend;

  if ((straight_mm <= 0.0f) || (v_max <= 0.0f))
  {
    return F413_RUN_SESSION_ABORT_NONE;
  }

  two_stage = (v_switch > 0.0f) && (v_switch < v_max) &&
              (accel_low > 0.0f) && (accel_high > 0.0f);
  if (two_stage)
  {
    const float d1 = ((v_switch * v_switch) - (v_start * v_start)) / (2.0f * accel_low);
    const float d2 = ((v_max * v_max) - (v_switch * v_switch)) / (2.0f * accel_high);
    const float d_acc_full = d1 + d2;
    const float d_total_acc_dec = 2.0f * d_acc_full;

    if (d_total_acc_dec <= straight_mm)
    {
      d_acc = d_acc_full;
      d_constant = straight_mm - d_total_acc_dec;
      max_reached_speed = v_max;
    }
    else
    {
      const float d_to_switch_and_back = 2.0f * d1;
      if (d_to_switch_and_back <= straight_mm)
      {
        const float remain_half = (straight_mm - d_to_switch_and_back) * 0.5f;
        const float v_reached_sq = (v_switch * v_switch) + (2.0f * accel_high * remain_half);
        max_reached_speed = sqrtf(v_reached_sq);
        d_acc = d1 + remain_half;
        d_constant = 0.0f;
      }
      else
      {
        d_acc = straight_mm * 0.5f;
        d_constant = 0.0f;
        max_reached_speed = sqrtf((v_start * v_start) + (2.0f * accel_low * d_acc));
      }
    }
  }
  else
  {
    const float accel = (accel_high > 0.0f) ? accel_high : accel_low;
    if (accel <= 0.0f)
    {
      return f413_path_run_drive_segment(straight_mm, v_next, speed_now_mm_s, guard, trace_flags);
    }
    {
      const float t_acc_time = v_max / accel;
      const float d_acc_full = 0.5f * accel * t_acc_time * t_acc_time;
      const float d_total_acc_dec = 2.0f * d_acc_full;
      if (d_total_acc_dec > straight_mm)
      {
        d_acc = straight_mm * 0.5f;
        d_constant = 0.0f;
        max_reached_speed = sqrtf(2.0f * accel * d_acc);
      }
      else
      {
        d_acc = d_acc_full;
        d_constant = straight_mm - d_total_acc_dec;
        max_reached_speed = v_max;
      }
    }
  }

  if (d_acc > 0.0f)
  {
    /* handled below */
  }

  next_is_small_turn = (next_code >= 300U) && (next_code < 500U);
  next_is_large_turn = (next_code >= 500U) && (next_code < 700U);
  prev_is_small_turn = (prev_code >= 300U) && (prev_code < 500U);
  prev_is_large_turn = (prev_code >= 500U) && (prev_code < 700U);
  skip_wallend = (prev_is_large_turn && (code == 201U) && next_is_small_turn) ||
                 (prev_is_small_turn && (code == 201U) && next_is_large_turn) ||
                 !f413_run_features_wall_end_correction_enabled();

  if ((next_is_small_turn || next_is_large_turn) && !skip_wallend)
  {
    const float wall_end_buffer_mm = (float)DIST_HALF_SEC;
    const float buffer_max_mm = (float)DIST_HALF_SEC;
    const float min_main_for_accel = (float)DIST_HALF_SEC * 2.0f;
    float main_mm = straight_mm - wall_end_buffer_mm;
    bool wall_end_found = false;

    if (main_mm < 0.0f)
    {
      main_mm = 0.0f;
    }

    if (main_mm >= min_main_for_accel)
    {
      if (d_acc > 0.0f)
      {
        const float acc_run = (d_acc < main_mm) ? d_acc : main_mm;
        reason = f413_path_run_drive_segment(acc_run, max_reached_speed,
                                             speed_now_mm_s, guard, trace_flags);
        if (reason != F413_RUN_SESSION_ABORT_NONE)
        {
          return reason;
        }
      }
      if ((main_mm - d_acc) > 0.0f && d_constant > 0.0f)
      {
        float const_run = main_mm - d_acc;
        if (const_run > d_constant)
        {
          const_run = d_constant;
        }
        reason = f413_path_run_drive_segment(const_run, max_reached_speed,
                                             speed_now_mm_s, guard, trace_flags);
        if (reason != F413_RUN_SESSION_ABORT_NONE)
        {
          return reason;
        }
      }
      {
        const float dec_in_main = main_mm - d_acc - d_constant;
        if (dec_in_main > 0.0f)
        {
          reason = f413_path_run_drive_segment(dec_in_main, v_next,
                                               speed_now_mm_s, guard, trace_flags);
          if (reason != F413_RUN_SESSION_ABORT_NONE)
          {
            return reason;
          }
        }
      }
    }
    else if (main_mm > 0.0f)
    {
      reason = f413_path_run_drive_segment(main_mm, v_next,
                                           speed_now_mm_s, guard, trace_flags);
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        return reason;
      }
    }

    reason = f413_path_run_drive_wallend_segment(buffer_max_mm,
                                                 v_next,
                                                 speed_now_mm_s,
                                                 guard,
                                                 trace_flags,
                                                 &wall_end_found);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
    if (wall_end_found)
    {
      const float follow_dist = next_is_small_turn
          ? (wall_end_buffer_mm + mode_params->dist_wall_end)
          : mode_params->dist_wall_end;
      if (follow_dist > 0.0f)
      {
        return f413_path_run_drive_segment(follow_dist,
                                           v_next,
                                           speed_now_mm_s,
                                           guard,
                                           trace_flags);
      }
    }
    return F413_RUN_SESSION_ABORT_NONE;
  }

  if (d_acc > 0.0f)
  {
    reason = f413_path_run_drive_segment(d_acc, max_reached_speed, speed_now_mm_s,
                                         guard, trace_flags);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
  }
  if (d_constant > 0.0f)
  {
    reason = f413_path_run_drive_segment(d_constant, max_reached_speed, speed_now_mm_s,
                                         guard, trace_flags);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
  }
  if (d_acc > 0.0f)
  {
    reason = f413_path_run_drive_segment(d_acc, v_next, speed_now_mm_s,
                                         guard, trace_flags);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
  }
  return F413_RUN_SESSION_ABORT_NONE;
}

static float f413_path_run_next_diagonal_exit_velocity(uint16_t next_code,
                                                       const ShortestRunModeParams_t* mode_params)
{
  if (mode_params == NULL)
  {
    return 0.0f;
  }
  if (next_code < 800U)
  {
    return mode_params->velocity_turn45out;
  }
  if (next_code < 900U)
  {
    return mode_params->velocity_turnV90;
  }
  if (next_code < 1000U)
  {
    return mode_params->velocity_turn135out;
  }
  return 0.0f;
}

static f413_run_session_abort_reason_t f413_path_run_run_diagonal_code(
    uint16_t code,
    uint16_t next_code,
    const ShortestRunModeParams_t* mode_params,
    const ShortestRunCaseParams_t* case_params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  const float straight_mm = (float)(code - 1000U) * (float)DIST_D_HALF_SEC;
  const float v_max = (case_params != NULL) ? case_params->velocity_d_straight : 0.0f;
  const float accel = (case_params != NULL) ? case_params->acceleration_d_straight_dash : 0.0f;
  const float v_next = f413_path_run_next_diagonal_exit_velocity(next_code, mode_params);
  float d_acc;
  float d_constant = 0.0f;
  float max_reached_speed = v_max;
  f413_run_session_abort_reason_t reason;

  if ((straight_mm <= 0.0f) || (v_max <= 0.0f) || (accel <= 0.0f))
  {
    return F413_RUN_SESSION_ABORT_NONE;
  }

  {
    const float t_acc = v_max / accel;
    d_acc = 0.5f * accel * t_acc * t_acc;
  }
  if ((2.0f * d_acc) > straight_mm)
  {
    d_acc = straight_mm * 0.5f;
    max_reached_speed = accel * sqrtf((2.0f * d_acc) / accel);
  }
  else
  {
    d_constant = straight_mm - (2.0f * d_acc);
  }

  reason = f413_path_run_drive_diagonal_segment(d_acc, max_reached_speed, speed_now_mm_s,
                                                guard, trace_flags);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }
  reason = f413_path_run_drive_diagonal_segment(d_constant, max_reached_speed, speed_now_mm_s,
                                                guard, trace_flags);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }
  return f413_path_run_drive_diagonal_segment(d_acc, v_next, speed_now_mm_s, guard, trace_flags);
}

void f413_path_run_print_preview(void)
{
  uint16_t count = 0U;
  uint16_t limit;
  uint16_t i;

  while ((count < 1024U) && (path[count] != 0U))
  {
    count++;
  }

  limit = (count > NIGHTFALL_F413_PATH_PREVIEW_MAX) ? NIGHTFALL_F413_PATH_PREVIEW_MAX : count;
  trace_printf("[RUN-TEST] solver-path codes(%u): ", (unsigned int)count);

  for (i = 0U; i < limit; i++)
  {
    uint16_t code = path[i];
    if ((code > 200U) && (code < 300U))
    {
      trace_printf("S%u", (unsigned int)(code - 200U));
    }
    else if (code == 300U)
    {
      trace_printf("s-R90");
    }
    else if (code == 400U)
    {
      trace_printf("s-L90");
    }
    else if (code == 501U || code == 502U)
    {
      trace_printf("L-R%s", (code == 501U) ? "90" : "180");
    }
    else if (code == 601U || code == 602U)
    {
      trace_printf("L-L%s", (code == 601U) ? "90" : "180");
    }
    else if ((code >= 701U) && (code <= 704U))
    {
      trace_printf("D45-%u", (unsigned int)(code - 700U));
    }
    else if ((code >= 801U) && (code <= 802U))
    {
      trace_printf("V90-%u", (unsigned int)(code - 800U));
    }
    else if ((code >= 901U) && (code <= 904U))
    {
      trace_printf("D135-%u", (unsigned int)(code - 900U));
    }
    else if (code > 1000U)
    {
      trace_printf("DS%u", (unsigned int)(code - 1000U));
    }
    else
    {
      trace_printf("%u", (unsigned int)code);
    }

    if ((i + 1U) < limit)
    {
      trace_printf(",");
    }
  }
  if (count > limit)
  {
    trace_printf(",...");
  }
  trace_printf("\r\n");
}

void f413_path_run_session_once(uint8_t mode,
                                uint8_t case_index,
                                uint16_t base_trace_flag,
                                const char* label)
{
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};
  const ShortestRunModeParams_t* mode_params = f413_path_run_mode_params(mode);
  const ShortestRunCaseParams_t* case_params = f413_path_run_case_params(mode, case_index);
  const float straight_velocity = f413_path_run_velocity_or_cap(case_params->velocity_straight,
                                                               NIGHTFALL_F413_PATH_VELOCITY);
  const float diagonal_velocity = f413_path_run_velocity_or_cap(case_params->velocity_d_straight,
                                                               straight_velocity);
  float speed_now = 0.0f;
  uint16_t pi;
  uint16_t code;

  if (f413_trace_log_auto_is_enabled())
  {
    trace_printf("[RUN-TEST] busy(auto already running)\r\n");
    return;
  }
  if (f413_hw_stop_switch_pressed())
  {
    trace_printf("[RUN-TEST] path canceled(start switch pressed)\r\n");
    return;
  }
  if (path[0] == 0U)
  {
    trace_printf("[RUN-TEST] path canceled(empty path)\r\n");
    return;
  }
  trace_printf("[RUN-TEST] path session start %s mode=%u case=%u v=%.0f diag_v=%.0f cap=%.0f, press switch to abort\r\n",
               (label != NULL) ? label : "path",
               (unsigned int)mode,
               (unsigned int)case_index,
               (double)straight_velocity,
               (double)diagonal_velocity,
               (double)NIGHTFALL_F413_PATH_VELOCITY_CAP);

  if (!f413_run_session_guard_prepare(&guard))
  {
    trace_printf("[RUN-TEST] path canceled(guard init fail)\r\n");
    return;
  }

  f413_ctrl_start();
  f413_wall_runtime_set_control_gains(case_params->kp_wall, case_params->kp_diagonal);
  f413_path_run_trace_on_run_start();

  {
    const float first_speed =
        sqrtf(fmaxf(0.0f, 2.0f * case_params->acceleration_straight * (float)DIST_FIRST_SEC));
    abort_reason = f413_path_run_drive_segment((float)DIST_FIRST_SEC,
                                               first_speed,
                                               &speed_now,
                                               &guard,
                                               (uint16_t)(base_trace_flag |
                                                          NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                                                          NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
  }

  for (pi = 0U; pi < NIGHTFALL_F413_PATH_MAX_CODES; pi++)
  {
    const uint16_t next_code = path[pi + 1U];

    if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }

    code = path[pi];
    if (code == 0U)
    {
      break;
    }

    if ((code > 200U) && (code < 300U))
    {
      abort_reason = f413_path_run_run_straight_code(code,
                                                     (pi > 0U) ? path[pi - 1U] : 0U,
                                                     next_code,
                                                     mode_params,
                                                     case_params,
                                                     &speed_now,
                                                     &guard,
          (uint16_t)(base_trace_flag | NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                     NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
    }
    else if (code > 1000U)
    {
      abort_reason = f413_path_run_run_diagonal_code(code,
                                                     next_code,
                                                     mode_params,
                                                     case_params,
                                                     &speed_now,
                                                     &guard,
          (uint16_t)(base_trace_flag | NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                     NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
    }
    else
    {
      f413_path_run_turn_t turn;

      if (f413_path_run_turn_from_code(code, mode_params, &turn))
      {
        abort_reason = f413_path_run_wait_smooth_turn_profile(&turn,
                                                              mode_params->val_offset_in,
                                                              &speed_now,
                                                              &guard,
            (uint16_t)(base_trace_flag | NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                       NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG));
      }
      else
      {
        trace_printf("[RUN-TEST] unsupported path code %u at [%u]\r\n",
                     (unsigned int)code, (unsigned int)pi);
        abort_reason = F413_RUN_SESSION_ABORT_IMU_FAULT;
      }
    }

    if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }

    f413_ctrl_clear_angle_target();
  }

  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    abort_reason = f413_path_run_drive_segment((float)DIST_HALF_SEC,
                                               0.0f,
                                               &speed_now,
                                               &guard,
                                               (uint16_t)(base_trace_flag |
                                                          NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                                                          NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
  }

  f413_ctrl_clear_angle_target();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  f413_trace_log_set_mode_flags((uint16_t)(base_trace_flag |
                                           NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                                           NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG));
  (void)f413_run_session_wait_with_auto_step_guarded(NIGHTFALL_F413_PATH_COAST_MS, &guard);
  f413_ctrl_stop();

  if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    f413_trace_log_set_mode_flags((uint16_t)(base_trace_flag |
                                             NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                                             f413_run_session_abort_reason_to_trace_flag(abort_reason)));
    f413_trace_log_auto_step();
  }

  f413_trace_log_set_mode_flags(0U);
  f413_path_run_trace_on_run_stop();
  f413_run_session_guard_cleanup(&guard);

  if (abort_reason == F413_RUN_SESSION_ABORT_SWITCH)
  {
    trace_printf("[RUN-TEST] path aborted by switch at code[%u]\r\n", (unsigned int)pi);
  }
  else if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    trace_printf("[RUN-TEST] path aborted(%s) at code[%u]\r\n",
                 f413_run_session_abort_reason_to_text(abort_reason),
                 (unsigned int)pi);
  }
  else
  {
    trace_printf("[RUN-TEST] path end (%u codes, dist=%.0fmm, angle=%.0fdeg)\r\n",
                 (unsigned int)pi,
                 (double)f413_ctrl_get_distance(),
                 (double)f413_ctrl_get_angle());
  }
}

void f413_path_run_solver_session_once(uint16_t base_trace_flag)
{
  f413_path_run_session_once(NIGHTFALL_F413_SOLVER_MODE,
                             NIGHTFALL_F413_SOLVER_CASE,
                             base_trace_flag,
                             "solver-path");
}

void f413_path_run_custom_path_session_once(const char* label,
                                            uint8_t mode,
                                            uint8_t case_index,
                                            const uint16_t* codes,
                                            uint16_t code_count,
                                            uint16_t base_trace_flag)
{
  uint16_t i;

  if ((codes == NULL) || (code_count == 0U))
  {
    trace_printf("[OP-UI][PATH-TEST] no sequence\r\n");
    return;
  }
  if (code_count >= NIGHTFALL_F413_PATH_MAX_CODES)
  {
    trace_printf("[OP-UI][PATH-TEST] too many codes(%u)\r\n", (unsigned int)code_count);
    return;
  }

  trace_printf("[OP-UI][PATH-TEST] set path %s mode=%u case=%u codes=",
               label,
               (unsigned int)mode,
               (unsigned int)case_index);
  for (i = 0U; i < code_count; i++)
  {
    trace_printf("%u%s", (unsigned int)codes[i], ((i + 1U) < code_count) ? "," : "");
    path[i] = codes[i];
  }
  path[code_count] = 0U;
  trace_printf("\r\n");

  for (i = (uint16_t)(code_count + 1U); i < NIGHTFALL_F413_PATH_MAX_CODES; i++)
  {
    path[i] = 0U;
  }

  f413_path_run_session_once(mode, case_index, base_trace_flag, label);
}

#endif
