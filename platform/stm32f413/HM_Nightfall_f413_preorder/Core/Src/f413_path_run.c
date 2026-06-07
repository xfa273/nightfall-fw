#include "f413_path_run.h"

#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)

#include <math.h>

#include "f413_control.h"
#include "f413_hw.h"
#include "f413_run_session.h"
#include "f413_trace_flags.h"
#include "f413_trace_log.h"
#include "f413_trace_sample.h"
#include "f413_wall_runtime.h"
#include "main.h"
#include "params.h"
#include "shortest_run_params.h"
#include "trace.h"

#ifndef NIGHTFALL_F413_PATH_DIAG_HALF_CELL_MM
#define NIGHTFALL_F413_PATH_DIAG_HALF_CELL_MM ((float)DIST_D_HALF_SEC)
#endif

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
  if (v > NIGHTFALL_F413_PATH_VELOCITY_CAP)
  {
    v = NIGHTFALL_F413_PATH_VELOCITY_CAP;
  }

  return v;
}

static bool f413_path_run_turn_from_code(uint16_t code,
                                         const ShortestRunModeParams_t* params,
                                         float* signed_angle_deg,
                                         float* abs_angle_deg)
{
  float angle = 0.0f;
  bool right = false;

  if ((params == NULL) || (signed_angle_deg == NULL) || (abs_angle_deg == NULL))
  {
    return false;
  }

  switch (code)
  {
    case 300U: angle = params->angle_turn_90; right = true; break;
    case 400U: angle = params->angle_turn_90; right = false; break;
    case 501U: angle = params->angle_l_turn_90; right = true; break;
    case 601U: angle = params->angle_l_turn_90; right = false; break;
    case 502U: angle = params->angle_l_turn_180; right = true; break;
    case 602U: angle = params->angle_l_turn_180; right = false; break;
    case 701U: angle = params->angle_turn45in; right = true; break;
    case 702U: angle = params->angle_turn45in; right = false; break;
    case 703U: angle = params->angle_turn45out; right = true; break;
    case 704U: angle = params->angle_turn45out; right = false; break;
    case 801U: angle = params->angle_turnV90; right = true; break;
    case 802U: angle = params->angle_turnV90; right = false; break;
    case 901U: angle = params->angle_turn135in; right = true; break;
    case 902U: angle = params->angle_turn135in; right = false; break;
    case 903U: angle = params->angle_turn135out; right = true; break;
    case 904U: angle = params->angle_turn135out; right = false; break;
    default: return false;
  }

  if (angle <= 0.0f)
  {
    angle = 90.0f;
  }

  *abs_angle_deg = fabsf(angle);
  *signed_angle_deg = right ? -*abs_angle_deg : *abs_angle_deg;
  return true;
}

static void f413_path_run_trace_on_run_start(void)
{
  trace_printf("[TRACE-LOG] run-hook: start\r\n");
  f413_trace_log_auto_start();
}

static void f413_path_run_trace_on_run_stop(void)
{
  trace_printf("[TRACE-LOG] run-hook: stop\r\n");
  f413_trace_log_auto_stop();
}

static f413_run_session_abort_reason_t f413_path_run_wait_ctrl_target(
    float target, bool is_angle,
    f413_run_session_guard_t* guard, uint16_t trace_flags)
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
    reason = f413_run_session_wait_with_auto_step_guarded(10U, guard);
    f413_wall_runtime_control_apply(!is_angle &&
        ((trace_flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG) != 0U));
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
  }
  return F413_RUN_SESSION_ABORT_NONE;
}

float f413_path_run_test_velocity_for_mode(uint8_t mode, bool fast)
{
  float v = 150.0f + ((float)((mode > 2U) ? (mode - 2U) : 0U) * 50.0f);
  if (fast)
  {
    v += 75.0f;
  }
  if (v > NIGHTFALL_F413_OP_TEST_VEL_CAP)
  {
    v = NIGHTFALL_F413_OP_TEST_VEL_CAP;
  }
  return v;
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

void f413_path_run_solver_session_once(uint16_t base_trace_flag)
{
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};
  const ShortestRunModeParams_t* mode_params =
      f413_path_run_mode_params(NIGHTFALL_F413_SOLVER_MODE);
  const ShortestRunCaseParams_t* case_params =
      f413_path_run_case_params(NIGHTFALL_F413_SOLVER_MODE, NIGHTFALL_F413_SOLVER_CASE);
  const float straight_velocity =
      f413_path_run_velocity_or_cap(case_params->velocity_straight, NIGHTFALL_F413_PATH_VELOCITY);
  const float diagonal_velocity =
      f413_path_run_velocity_or_cap(case_params->velocity_d_straight, straight_velocity);
  uint16_t pi;
  uint16_t code;

  if (f413_trace_log_auto_is_enabled())
  {
    trace_printf("[RUN-TEST] busy(auto already running)\r\n");
    return;
  }
  if (f413_hw_stop_switch_pressed())
  {
    trace_printf("[RUN-TEST] solver-path canceled(start switch pressed)\r\n");
    return;
  }

  trace_printf("[RUN-TEST] solver-path session start (mode=%u case=%u v=%.0f diag_v=%.0f cap=%.0f, press switch to abort)\r\n",
               (unsigned int)NIGHTFALL_F413_SOLVER_MODE,
               (unsigned int)NIGHTFALL_F413_SOLVER_CASE,
               (double)straight_velocity,
               (double)diagonal_velocity,
               (double)NIGHTFALL_F413_PATH_VELOCITY_CAP);

  if (!f413_run_session_guard_prepare(&guard))
  {
    trace_printf("[RUN-TEST] solver-path canceled(guard init fail)\r\n");
    return;
  }

  f413_path_run_trace_on_run_start();
  f413_ctrl_start();

  for (pi = 0U; pi < NIGHTFALL_F413_PATH_MAX_CODES; pi++)
  {
    code = path[pi];
    if (code == 0U)
    {
      break;
    }

    f413_ctrl_reset_distance();
    f413_ctrl_reset_angle();

    if ((code > 200U) && (code < 300U))
    {
      uint16_t half_cells = (uint16_t)(code - 200U);
      float target_mm = (float)half_cells * NIGHTFALL_F413_PATH_HALF_CELL_MM;

      f413_ctrl_set_velocity(straight_velocity);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);

      abort_reason = f413_path_run_wait_ctrl_target(target_mm, false, &guard,
          (uint16_t)(base_trace_flag | NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                     NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
    }
    else if (code > 1000U)
    {
      uint16_t diag_half = (uint16_t)(code - 1000U);
      float target_mm = (float)diag_half * NIGHTFALL_F413_PATH_DIAG_HALF_CELL_MM;

      f413_ctrl_set_velocity(diagonal_velocity);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);

      abort_reason = f413_path_run_wait_ctrl_target(target_mm, false, &guard,
          (uint16_t)(base_trace_flag | NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                     NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
    }
    else
    {
      float signed_angle = 0.0f;
      float abs_angle = 0.0f;

      if (f413_path_run_turn_from_code(code, mode_params, &signed_angle, &abs_angle))
      {
        f413_ctrl_set_velocity(0.0f);
        f413_ctrl_set_omega(0.0f);
        f413_ctrl_set_angle_target(signed_angle);

        abort_reason = f413_path_run_wait_ctrl_target(signed_angle, true, &guard,
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
    f413_ctrl_set_velocity(0.0f);
    f413_ctrl_set_omega(0.0f);
    f413_trace_log_set_mode_flags((uint16_t)(base_trace_flag |
                                             NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                                             NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG));
    abort_reason = f413_run_session_wait_with_auto_step_guarded(NIGHTFALL_F413_PATH_COAST_MS, &guard);
    if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }
  }

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
    trace_printf("[RUN-TEST] solver-path aborted by switch at code[%u]\r\n", (unsigned int)pi);
  }
  else if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    trace_printf("[RUN-TEST] solver-path aborted(%s) at code[%u]\r\n",
                 f413_run_session_abort_reason_to_text(abort_reason),
                 (unsigned int)pi);
  }
  else
  {
    trace_printf("[RUN-TEST] solver-path end (%u codes, dist=%.0fmm, angle=%.0fdeg)\r\n",
                 (unsigned int)pi,
                 (double)f413_ctrl_get_distance(),
                 (double)f413_ctrl_get_angle());
  }
}

void f413_path_run_code_sequence_once(const char* label,
                                      uint8_t mode,
                                      uint8_t case_index,
                                      const uint16_t* codes,
                                      uint16_t code_count,
                                      float straight_velocity,
                                      float diagonal_velocity)
{
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};
  const ShortestRunModeParams_t* mode_params = f413_path_run_mode_params(mode);
  uint16_t i;
  uint16_t code = 0U;

  if ((codes == NULL) || (code_count == 0U))
  {
    trace_printf("[OP-UI][PATH-TEST] no sequence\r\n");
    return;
  }
  if (f413_trace_log_auto_is_enabled())
  {
    trace_printf("[OP-UI][PATH-TEST] busy(auto already running)\r\n");
    return;
  }
  if (f413_hw_stop_switch_pressed())
  {
    trace_printf("[OP-UI][PATH-TEST] canceled(start switch pressed)\r\n");
    return;
  }
  if (!f413_run_session_guard_prepare(&guard))
  {
    trace_printf("[OP-UI][PATH-TEST] canceled(guard init fail)\r\n");
    return;
  }

  trace_printf("[OP-UI][PATH-TEST] start %s mode=%u case=%u v=%.0f diag_v=%.0f codes=",
               label,
               (unsigned int)mode,
               (unsigned int)case_index,
               (double)straight_velocity,
               (double)diagonal_velocity);
  for (i = 0U; i < code_count; i++)
  {
    trace_printf("%u%s", (unsigned int)codes[i], ((i + 1U) < code_count) ? "," : "");
  }
  trace_printf("\r\n");

  f413_path_run_trace_on_run_start();
  f413_ctrl_start();

  for (i = 0U; i < code_count; i++)
  {
    code = codes[i];
    f413_ctrl_reset_distance();
    f413_ctrl_reset_angle();

    if ((code > 200U) && (code < 300U))
    {
      uint16_t half_cells = (uint16_t)(code - 200U);
      float target_mm = (float)half_cells * NIGHTFALL_F413_PATH_HALF_CELL_MM;
      f413_ctrl_set_velocity(straight_velocity);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);
      abort_reason = f413_path_run_wait_ctrl_target(target_mm, false, &guard,
          (uint16_t)(NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG | NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
    }
    else if (code > 1000U)
    {
      uint16_t diag_half = (uint16_t)(code - 1000U);
      float target_mm = (float)diag_half * NIGHTFALL_F413_PATH_DIAG_HALF_CELL_MM;
      f413_ctrl_set_velocity(diagonal_velocity);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);
      abort_reason = f413_path_run_wait_ctrl_target(target_mm, false, &guard,
          (uint16_t)(NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG | NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
    }
    else
    {
      float signed_angle = 0.0f;
      float abs_angle = 0.0f;
      if (f413_path_run_turn_from_code(code, mode_params, &signed_angle, &abs_angle))
      {
        f413_ctrl_set_velocity(0.0f);
        f413_ctrl_set_omega(0.0f);
        f413_ctrl_set_angle_target(signed_angle);
        abort_reason = f413_path_run_wait_ctrl_target(signed_angle, true, &guard,
            (uint16_t)(NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG | NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG));
      }
      else
      {
        abort_reason = F413_RUN_SESSION_ABORT_IMU_FAULT;
      }
    }

    if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }

    f413_ctrl_clear_angle_target();
    f413_ctrl_set_velocity(0.0f);
    f413_ctrl_set_omega(0.0f);
    f413_trace_log_set_mode_flags((uint16_t)(NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                                             NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG));
    abort_reason = f413_run_session_wait_with_auto_step_guarded(NIGHTFALL_F413_PATH_COAST_MS, &guard);
    if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }
  }

  f413_ctrl_clear_angle_target();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  f413_ctrl_stop();
  if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    f413_trace_log_set_mode_flags((uint16_t)(NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                                             f413_run_session_abort_reason_to_trace_flag(abort_reason)));
    f413_trace_log_auto_step();
  }
  f413_trace_log_set_mode_flags(0U);
  f413_path_run_trace_on_run_stop();
  f413_run_session_guard_cleanup(&guard);
  {
    const float distance_mm = f413_ctrl_get_distance();
    const float angle_deg = f413_ctrl_get_angle();
    f413_trace_sample_record_result((uint8_t)'P', abort_reason, distance_mm, angle_deg);
    trace_printf("[OP-UI][PATH-TEST] %s %s at code[%u]=%u dist=%.0fmm angle=%.0fdeg\r\n",
                 label,
                 (abort_reason == F413_RUN_SESSION_ABORT_NONE) ? "OK" :
                 f413_run_session_abort_reason_to_text(abort_reason),
                 (unsigned int)i,
                 (unsigned int)code,
                 (double)distance_mm,
                 (double)angle_deg);
  }
}

#endif
