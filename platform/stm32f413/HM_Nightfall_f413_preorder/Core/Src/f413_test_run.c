#include "f413_test_run.h"

#include <math.h>

#include "f413_control.h"
#include "f413_hw.h"
#include "trace.h"

#define F413_TEST_RUN_VEL_MM_S       (200.0f)
#define F413_TEST_RUN_HALF_CELL_MM   (45.0f)
#define F413_TEST_RUN_COAST_MS       (200U)
#define F413_TEST_RUN_TIMEOUT_MS     (5000U)
#define F413_TEST_RUN_ARMED_NONE     (0U)
#define F413_TEST_RUN_MOTOR_DUTY     (120U)
#define F413_TEST_RUN_MOTOR_MS       (500U)
#define F413_TEST_RUN_MOTOR_COAST_MS (300U)
#define F413_TEST_RUN_ENCODER_SIGN_L (1L)
#define F413_TEST_RUN_ENCODER_SIGN_R (-1L)

static f413_test_run_config_t g_config;
static volatile uint8_t g_armed_id = F413_TEST_RUN_ARMED_NONE;

static void f413_test_run_delay(uint32_t ms)
{
  if (g_config.delay_ms != NULL)
  {
    g_config.delay_ms(ms);
  }
}

static uint32_t f413_test_run_tick(void)
{
  if (g_config.get_tick_ms != NULL)
  {
    return g_config.get_tick_ms();
  }
  return 0U;
}

static void f413_test_run_set_flags(uint16_t flags)
{
  if (g_config.trace_set_mode_flags != NULL)
  {
    g_config.trace_set_mode_flags(flags);
  }
}

static void f413_test_run_record(uint8_t test_id,
                                 f413_run_session_abort_reason_t abort_reason)
{
  if (g_config.record_result != NULL)
  {
    g_config.record_result(test_id,
                           abort_reason,
                           f413_ctrl_get_distance(),
                           f413_ctrl_get_angle());
  }
}

static f413_run_session_abort_reason_t f413_test_run_wait_target(float target_mm,
                                                                 bool reverse,
                                                                 f413_run_session_guard_t* guard,
                                                                 uint16_t flags)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  uint32_t deadline = f413_test_run_tick() + F413_TEST_RUN_TIMEOUT_MS;

  while (fabsf(f413_ctrl_get_distance()) < fabsf(target_mm))
  {
    if (f413_test_run_tick() >= deadline)
    {
      break;
    }
    f413_test_run_set_flags(flags);
    reason = f413_run_session_wait_with_auto_step_guarded(10U, guard);
    if ((g_config.wall_control_apply_straight != NULL) &&
        ((flags & g_config.trace_motor_fwd_flag) != 0U))
    {
      g_config.wall_control_apply_straight();
    }
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
    f413_ctrl_set_velocity(reverse ? -F413_TEST_RUN_VEL_MM_S : F413_TEST_RUN_VEL_MM_S);
    f413_ctrl_set_omega(0.0f);
  }
  return F413_RUN_SESSION_ABORT_NONE;
}

static f413_run_session_abort_reason_t f413_test_run_wait_angle_signed(float target_deg,
                                                                       f413_run_session_guard_t* guard,
                                                                       uint16_t flags)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  uint32_t deadline = f413_test_run_tick() + F413_TEST_RUN_TIMEOUT_MS;

  if (target_deg >= 0.0f)
  {
    while (f413_ctrl_get_angle() < target_deg)
    {
      if (f413_test_run_tick() >= deadline)
      {
        break;
      }
      f413_test_run_set_flags(flags);
      reason = f413_run_session_wait_with_auto_step_guarded(10U, guard);
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        return reason;
      }
    }
  }
  else
  {
    while (f413_ctrl_get_angle() > target_deg)
    {
      if (f413_test_run_tick() >= deadline)
      {
        break;
      }
      f413_test_run_set_flags(flags);
      reason = f413_run_session_wait_with_auto_step_guarded(10U, guard);
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        return reason;
      }
    }
  }
  return F413_RUN_SESSION_ABORT_NONE;
}

void f413_test_run_config(const f413_test_run_config_t* config)
{
  if (config != NULL)
  {
    g_config = *config;
  }
}

const char* f413_test_run_name(uint8_t test_id)
{
  switch (test_id)
  {
    case '1': return "straight 3 cells heading hold (270mm)";
    case '2': return "straight 6 cells heading hold (540mm)";
    case '3': return "right 90 deg turn";
    case '4': return "left 90 deg turn";
    case '5': return "S3 + R90 + S3 heading hold";
    case '6': return "HW: L-motor fwd only";
    case '7': return "HW: R-motor fwd only";
    case '8': return "HW: L-motor rev only";
    case '9': return "HW: R-motor rev only";
    default: return "?";
  }
}

const char* f413_test_run_arm_name(uint8_t test_id)
{
  switch (test_id)
  {
    case '1': return "straight 3 cells heading hold";
    case '2': return "straight 6 cells heading hold";
    case '3': return "right 90 deg";
    case '4': return "left 90 deg";
    case '5': return "S3+R90+S3 heading hold";
    default: return "unknown";
  }
}

bool f413_test_run_is_button_test(uint8_t test_id)
{
  return (test_id >= (uint8_t)'1') && (test_id <= (uint8_t)'5');
}

bool f413_test_run_is_armed(void)
{
  return f413_test_run_is_button_test(g_armed_id);
}

uint8_t f413_test_run_armed_id(void)
{
  return g_armed_id;
}

void f413_test_run_clear_arm(void)
{
  g_armed_id = F413_TEST_RUN_ARMED_NONE;
}

void f413_test_run_arm_for_button(uint8_t test_id)
{
  g_armed_id = test_id;
  if (g_config.trace_set_context != NULL)
  {
    g_config.trace_set_context(8U, 0xFFU, 0xFFU, test_id);
  }

  trace_printf("[TEST] armed for button: '%c' (%s)\r\n",
               (char)test_id,
               f413_test_run_arm_name(test_id));
  trace_printf("[TEST] disconnect UART, press PUSH button to start, reconnect and 'V' to get CSV\r\n");
}

static void f413_test_run_single_motor(uint8_t test_id, const char* test_name)
{
  bool use_left = (test_id == (uint8_t)'6') || (test_id == (uint8_t)'8');
  bool forward = (test_id == (uint8_t)'6') || (test_id == (uint8_t)'7');
  int32_t final_enc_l = 0;
  int32_t final_enc_r = 0;

  if (g_config.encoder_stop_all != NULL)
  {
    g_config.encoder_stop_all();
  }
  if (g_config.encoder_reset_all != NULL)
  {
    g_config.encoder_reset_all();
  }
  if (g_config.encoder_start_l != NULL)
  {
    (void)g_config.encoder_start_l();
  }
  if (g_config.encoder_start_r != NULL)
  {
    (void)g_config.encoder_start_r();
  }

  trace_printf("[TEST] motor=%s dir=%s duty=%u ms=%lu\r\n",
               use_left ? "LEFT" : "RIGHT",
               forward ? "FWD" : "REV",
               (unsigned int)F413_TEST_RUN_MOTOR_DUTY,
               (unsigned long)F413_TEST_RUN_MOTOR_MS);
  trace_printf("[TEST] PWM: CH1(TIM2)=%s, CH3(TIM2)=%s\r\n",
               use_left ? (forward ? "duty" : "inv-duty") : "0",
               use_left ? "0" : (forward ? "inv-duty" : "duty"));
  trace_printf("[TEST] IN2: L=%s, R=%s\r\n",
               (use_left && forward) ? "RESET" :
               (use_left && !forward) ? "SET" : "RESET",
               (!use_left && forward) ? "SET" :
               (!use_left && !forward) ? "RESET" : "RESET");

  if (g_config.motor_set != NULL)
  {
    if (use_left)
    {
      g_config.motor_set(true, forward, true, F413_TEST_RUN_MOTOR_DUTY, 0U);
    }
    else
    {
      g_config.motor_set(true, true, forward, 0U, F413_TEST_RUN_MOTOR_DUTY);
    }
  }

  f413_test_run_delay(F413_TEST_RUN_MOTOR_MS);
  if (g_config.motor_set != NULL)
  {
    g_config.motor_set(false, true, true, 0U, 0U);
  }
  f413_test_run_delay(F413_TEST_RUN_MOTOR_COAST_MS);

  if (g_config.encoder_l_count != NULL)
  {
    final_enc_l = F413_TEST_RUN_ENCODER_SIGN_L *
        f413_hw_encoder_delta_signed((uint32_t)(uint16_t)g_config.encoder_l_count(), 0U);
  }
  if (g_config.encoder_r_count != NULL)
  {
    final_enc_r = F413_TEST_RUN_ENCODER_SIGN_R *
        f413_hw_encoder_delta_signed((uint32_t)(uint16_t)g_config.encoder_r_count(), 0U);
  }

  if (g_config.encoder_stop_all != NULL)
  {
    g_config.encoder_stop_all();
  }
  if (g_config.encoder_center_all != NULL)
  {
    g_config.encoder_center_all();
  }
  if (g_config.encoder_start_l != NULL)
  {
    (void)g_config.encoder_start_l();
  }
  if (g_config.encoder_start_r != NULL)
  {
    (void)g_config.encoder_start_r();
  }

  trace_printf("[TEST] === %s === OK enc_l=%ld enc_r=%ld\r\n",
               test_name,
               (long)final_enc_l,
               (long)final_enc_r);
}

void f413_test_run_run_now(uint8_t test_id)
{
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};
  const uint16_t tf = g_config.trace_solver_path_flag;
  const char* test_name = f413_test_run_name(test_id);

  if (test_name[0] == '?')
  {
    trace_printf("[TEST] unknown test_id '%c'\r\n", (char)test_id);
    return;
  }

  trace_printf("[TEST] === %s === start\r\n", test_name);

  if ((test_id >= (uint8_t)'6') && (test_id <= (uint8_t)'9'))
  {
    f413_test_run_single_motor(test_id, test_name);
    return;
  }

  if (!f413_run_session_guard_prepare(&guard))
  {
    trace_printf("[TEST] guard init fail\r\n");
    return;
  }

  f413_ctrl_start();
  if (g_config.trace_on_run_start != NULL)
  {
    g_config.trace_on_run_start();
  }

  switch (test_id)
  {
    case '1':
      f413_ctrl_reset_distance();
      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(F413_TEST_RUN_VEL_MM_S);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);
      abort_reason = f413_test_run_wait_target(
          6.0f * F413_TEST_RUN_HALF_CELL_MM, false, &guard,
          (uint16_t)(tf | g_config.trace_motor_fwd_flag));
      break;

    case '2':
      f413_ctrl_reset_distance();
      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(F413_TEST_RUN_VEL_MM_S);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);
      abort_reason = f413_test_run_wait_target(
          12.0f * F413_TEST_RUN_HALF_CELL_MM, false, &guard,
          (uint16_t)(tf | g_config.trace_motor_fwd_flag));
      break;

    case '3':
      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(0.0f);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(-90.0f);
      abort_reason = f413_test_run_wait_angle_signed(
          -90.0f, &guard,
          (uint16_t)(tf | g_config.trace_motor_rev_flag));
      break;

    case '4':
      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(0.0f);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(90.0f);
      abort_reason = f413_test_run_wait_angle_signed(
          90.0f, &guard,
          (uint16_t)(tf | g_config.trace_motor_rev_flag));
      break;

    case '5':
      f413_ctrl_reset_distance();
      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(F413_TEST_RUN_VEL_MM_S);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);
      abort_reason = f413_test_run_wait_target(
          6.0f * F413_TEST_RUN_HALF_CELL_MM, false, &guard,
          (uint16_t)(tf | g_config.trace_motor_fwd_flag));
      if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }

      f413_ctrl_clear_angle_target();
      f413_ctrl_set_velocity(0.0f);
      f413_ctrl_set_omega(0.0f);
      f413_test_run_set_flags((uint16_t)(tf | g_config.trace_motor_coast_flag));
      abort_reason = f413_run_session_wait_with_auto_step_guarded(
          F413_TEST_RUN_COAST_MS, &guard);
      if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }

      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(0.0f);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(-90.0f);
      abort_reason = f413_test_run_wait_angle_signed(
          -90.0f, &guard,
          (uint16_t)(tf | g_config.trace_motor_rev_flag));
      if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }

      f413_ctrl_clear_angle_target();
      f413_ctrl_set_velocity(0.0f);
      f413_ctrl_set_omega(0.0f);
      f413_test_run_set_flags((uint16_t)(tf | g_config.trace_motor_coast_flag));
      abort_reason = f413_run_session_wait_with_auto_step_guarded(
          F413_TEST_RUN_COAST_MS, &guard);
      if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }

      f413_ctrl_reset_distance();
      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(F413_TEST_RUN_VEL_MM_S);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);
      abort_reason = f413_test_run_wait_target(
          6.0f * F413_TEST_RUN_HALF_CELL_MM, false, &guard,
          (uint16_t)(tf | g_config.trace_motor_fwd_flag));
      break;

    default:
      break;
  }

  f413_ctrl_clear_angle_target();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);

  f413_test_run_set_flags((uint16_t)(tf | g_config.trace_motor_coast_flag));
  (void)f413_run_session_wait_with_auto_step_guarded(F413_TEST_RUN_COAST_MS, &guard);

  f413_ctrl_stop();

  if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    f413_test_run_set_flags((uint16_t)(tf |
        f413_run_session_abort_reason_to_trace_flag(abort_reason)));
    if (g_config.trace_auto_step != NULL)
    {
      g_config.trace_auto_step();
    }
  }
  f413_test_run_set_flags(0U);
  if (g_config.trace_on_run_stop != NULL)
  {
    g_config.trace_on_run_stop();
  }
  f413_run_session_guard_cleanup(&guard);

  f413_test_run_record(test_id, abort_reason);

  trace_printf("[TEST] === %s === %s dist=%.0fmm angle=%.0fdeg\r\n",
               test_name,
               (abort_reason == F413_RUN_SESSION_ABORT_NONE) ? "OK" :
               f413_run_session_abort_reason_to_text(abort_reason),
               (double)f413_ctrl_get_distance(),
               (double)f413_ctrl_get_angle());
}

void f413_test_run_step_button_armed(void)
{
  uint8_t tid;

  if (!f413_test_run_is_armed())
  {
    return;
  }
  if ((g_config.stop_switch_pressed == NULL) || !g_config.stop_switch_pressed())
  {
    return;
  }

  f413_test_run_delay(50U);
  while ((g_config.stop_switch_pressed != NULL) && g_config.stop_switch_pressed())
  {
    f413_test_run_delay(10U);
  }
  f413_test_run_delay(50U);
  f413_test_run_delay(2000U);

  tid = g_armed_id;
  g_armed_id = F413_TEST_RUN_ARMED_NONE;
  f413_test_run_run_now(tid);
}
