#include "f413_run_session.h"

#include <stdlib.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "trace.h"

#define F413_RUN_SESSION_GUARD_WALL_CHECK_MS (20U)
#define F413_RUN_SESSION_GUARD_IMU_CHECK_MS (100U)
#define F413_RUN_SESSION_GUARD_ENCODER_DELTA_MAX (6000)
#define F413_RUN_SESSION_TRACE_ABORT_SWITCH_FLAG (0x0100U)
#define F413_RUN_SESSION_TRACE_ABORT_WALL_FAULT_FLAG (0x0200U)
#define F413_RUN_SESSION_TRACE_ABORT_ENCODER_FAULT_FLAG (0x0400U)
#define F413_RUN_SESSION_TRACE_ABORT_IMU_FAULT_FLAG (0x0800U)

static f413_run_session_config_t s_config;

static int32_t f413_run_session_abs_i32(int32_t v)
{
  return (v < 0) ? -v : v;
}

static bool f413_run_session_stop_switch_pressed(void)
{
  return (s_config.stop_switch_pressed != NULL) && s_config.stop_switch_pressed();
}

static int16_t f413_run_session_encoder_l_count(void)
{
  if (s_config.encoder_l_count == NULL)
  {
    return 0;
  }
  return s_config.encoder_l_count();
}

static int16_t f413_run_session_encoder_r_count(void)
{
  if (s_config.encoder_r_count == NULL)
  {
    return 0;
  }
  return s_config.encoder_r_count();
}

static bool f413_run_session_wall_sensor_ok(void)
{
  return (s_config.wall_sensor_ok == NULL) || s_config.wall_sensor_ok();
}

static bool f413_run_session_imu_ok(void)
{
  return (s_config.imu_ok == NULL) || s_config.imu_ok();
}

static void f413_run_session_trace_auto_step(void)
{
  if (s_config.trace_auto_step != NULL)
  {
    s_config.trace_auto_step();
  }
}

static bool f413_run_session_trace_auto_is_enabled(void)
{
  return (s_config.trace_auto_is_enabled != NULL) && s_config.trace_auto_is_enabled();
}

static void f413_run_session_trace_on_run_start(void)
{
  if (s_config.trace_on_run_start != NULL)
  {
    s_config.trace_on_run_start();
  }
}

static void f413_run_session_trace_on_run_stop(void)
{
  if (s_config.trace_on_run_stop != NULL)
  {
    s_config.trace_on_run_stop();
  }
}

static void f413_run_session_trace_set_mode_flags(uint16_t mode_flags)
{
  if (s_config.trace_set_mode_flags != NULL)
  {
    s_config.trace_set_mode_flags(mode_flags);
  }
}

static void f413_run_session_encoder_stop_all(void)
{
  if (s_config.encoder_stop_all != NULL)
  {
    s_config.encoder_stop_all();
  }
}

static void f413_run_session_encoder_reset_all(void)
{
  if (s_config.encoder_reset_all != NULL)
  {
    s_config.encoder_reset_all();
  }
}

static bool f413_run_session_encoder_start_l(void)
{
  return (s_config.encoder_start_l != NULL) && s_config.encoder_start_l();
}

static bool f413_run_session_encoder_start_r(void)
{
  return (s_config.encoder_start_r != NULL) && s_config.encoder_start_r();
}

static void f413_run_session_encoder_stop_l(void)
{
  if (s_config.encoder_stop_l != NULL)
  {
    s_config.encoder_stop_l();
  }
}

static void f413_run_session_encoder_stop_r(void)
{
  if (s_config.encoder_stop_r != NULL)
  {
    s_config.encoder_stop_r();
  }
}

static void f413_run_session_motor_set(bool enable,
                                       bool left_forward,
                                       bool right_forward,
                                       uint16_t left_duty,
                                       uint16_t right_duty)
{
  if (s_config.motor_set != NULL)
  {
    s_config.motor_set(enable, left_forward, right_forward, left_duty, right_duty);
  }
}

void f413_run_session_config(const f413_run_session_config_t* config)
{
  if (config == NULL)
  {
    memset(&s_config, 0, sizeof(s_config));
    return;
  }
  s_config = *config;
}

bool f413_run_session_guard_prepare(f413_run_session_guard_t* guard)
{
  if (guard == NULL)
  {
    return false;
  }

  memset(guard, 0, sizeof(*guard));
  guard->prev_encoder_l = f413_run_session_encoder_l_count();
  guard->prev_encoder_r = f413_run_session_encoder_r_count();
  guard->next_wall_check_ms = HAL_GetTick();
  guard->next_imu_check_ms = HAL_GetTick();
  return true;
}

void f413_run_session_guard_cleanup(f413_run_session_guard_t* guard)
{
  (void)guard;
}

f413_run_session_abort_reason_t f413_run_session_guard_check(f413_run_session_guard_t* guard)
{
  int16_t enc_l_now;
  int16_t enc_r_now;
  int32_t d_l;
  int32_t d_r;
  uint32_t now;

  if (guard == NULL)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  if (f413_run_session_stop_switch_pressed())
  {
    return F413_RUN_SESSION_ABORT_SWITCH;
  }

  enc_l_now = f413_run_session_encoder_l_count();
  enc_r_now = f413_run_session_encoder_r_count();
  d_l = (int32_t)enc_l_now - (int32_t)guard->prev_encoder_l;
  d_r = (int32_t)enc_r_now - (int32_t)guard->prev_encoder_r;
  guard->prev_encoder_l = enc_l_now;
  guard->prev_encoder_r = enc_r_now;

  if ((f413_run_session_abs_i32(d_l) > F413_RUN_SESSION_GUARD_ENCODER_DELTA_MAX) ||
      (f413_run_session_abs_i32(d_r) > F413_RUN_SESSION_GUARD_ENCODER_DELTA_MAX))
  {
    return F413_RUN_SESSION_ABORT_ENCODER_FAULT;
  }

  now = HAL_GetTick();
  if ((int32_t)(now - guard->next_wall_check_ms) >= 0)
  {
    guard->next_wall_check_ms = now + F413_RUN_SESSION_GUARD_WALL_CHECK_MS;
    if (!f413_run_session_wall_sensor_ok())
    {
      return F413_RUN_SESSION_ABORT_WALL_FAULT;
    }
  }

  if ((int32_t)(now - guard->next_imu_check_ms) >= 0)
  {
    guard->next_imu_check_ms = now + F413_RUN_SESSION_GUARD_IMU_CHECK_MS;
    if (!f413_run_session_imu_ok())
    {
      return F413_RUN_SESSION_ABORT_IMU_FAULT;
    }
  }

  return F413_RUN_SESSION_ABORT_NONE;
}

f413_run_session_abort_reason_t f413_run_session_wait_with_auto_step_guarded(uint32_t duration_ms,
                                                                             f413_run_session_guard_t* guard)
{
  uint32_t deadline = HAL_GetTick() + duration_ms;

  while ((int32_t)(HAL_GetTick() - deadline) < 0)
  {
    f413_run_session_abort_reason_t reason = f413_run_session_guard_check(guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
    f413_run_session_trace_auto_step();
    HAL_Delay(1U);
  }

  f413_run_session_trace_auto_step();
  return f413_run_session_guard_check(guard);
}

void f413_run_session_wait_with_auto_step(uint32_t duration_ms)
{
  uint32_t deadline = HAL_GetTick() + duration_ms;

  while ((int32_t)(HAL_GetTick() - deadline) < 0)
  {
    f413_run_session_trace_auto_step();
    HAL_Delay(1U);
  }
  f413_run_session_trace_auto_step();
}

void f413_run_session_run_idle_trace_once(uint32_t duration_ms, uint16_t idle_mode_flag)
{
  if (f413_run_session_trace_auto_is_enabled())
  {
    trace_printf("[RUN-TEST] busy(auto already running)\r\n");
    return;
  }

  trace_printf("[RUN-TEST] idle trace session start %lu ms\r\n",
               (unsigned long)duration_ms);

  f413_run_session_trace_on_run_start();
  f413_run_session_trace_set_mode_flags(idle_mode_flag);
  f413_run_session_wait_with_auto_step(duration_ms);
  f413_run_session_trace_set_mode_flags(0U);
  f413_run_session_trace_on_run_stop();

  trace_printf("[RUN-TEST] idle trace session end\r\n");
}

void f413_run_session_run_motor_trace_once(uint16_t motor_fwd_flag,
                                           uint16_t motor_coast_flag,
                                           uint16_t motor_rev_flag,
                                           uint16_t motor_duty,
                                           uint32_t pulse_ms,
                                           uint32_t coast_ms)
{
  bool enc_started_l = false;
  bool enc_started_r = false;

  if (f413_run_session_trace_auto_is_enabled())
  {
    trace_printf("[RUN-TEST] busy(auto already running)\r\n");
    return;
  }

  trace_printf("[RUN-TEST] motor trace session start (lift robot before test)\r\n");

  f413_run_session_encoder_stop_all();
  f413_run_session_encoder_reset_all();

  if (f413_run_session_encoder_start_l())
  {
    enc_started_l = true;
  }
  if (f413_run_session_encoder_start_r())
  {
    enc_started_r = true;
  }

  f413_run_session_trace_on_run_start();
  f413_run_session_trace_set_mode_flags(motor_fwd_flag);
  f413_run_session_motor_set(true, true, true, motor_duty, motor_duty);
  f413_run_session_wait_with_auto_step(pulse_ms);

  f413_run_session_trace_set_mode_flags(motor_coast_flag);
  f413_run_session_motor_set(false, true, true, 0U, 0U);
  f413_run_session_wait_with_auto_step(coast_ms);

  f413_run_session_trace_set_mode_flags(motor_rev_flag);
  f413_run_session_motor_set(true, false, false, motor_duty, motor_duty);
  f413_run_session_wait_with_auto_step(pulse_ms);

  f413_run_session_trace_set_mode_flags(motor_coast_flag);
  f413_run_session_motor_set(false, false, false, 0U, 0U);
  f413_run_session_wait_with_auto_step(coast_ms);

  f413_run_session_trace_set_mode_flags(0U);
  f413_run_session_trace_on_run_stop();

  if (enc_started_l)
  {
    f413_run_session_encoder_stop_l();
  }
  if (enc_started_r)
  {
    f413_run_session_encoder_stop_r();
  }

  trace_printf("[RUN-TEST] motor trace session end\r\n");
}

void f413_run_session_run_search_safe_trace_once(uint16_t search_safe_flag,
                                                 uint16_t motor_fwd_flag,
                                                 uint16_t motor_coast_flag,
                                                 uint16_t safe_duty,
                                                 uint32_t forward_ms,
                                                 uint32_t coast_ms,
                                                 uint32_t explore_steps)
{
  bool enc_started_l = false;
  bool enc_started_r = false;
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};
  uint32_t step;

  if (f413_run_session_trace_auto_is_enabled())
  {
    trace_printf("[RUN-TEST] busy(auto already running)\r\n");
    return;
  }

  if (f413_run_session_stop_switch_pressed())
  {
    trace_printf("[RUN-TEST] search-safe canceled(start switch pressed)\r\n");
    return;
  }

  trace_printf("[RUN-TEST] search-safe start (low speed, press switch to abort)\r\n");

  f413_run_session_encoder_stop_all();
  f413_run_session_encoder_reset_all();

  if (f413_run_session_encoder_start_l())
  {
    enc_started_l = true;
  }
  if (f413_run_session_encoder_start_r())
  {
    enc_started_r = true;
  }

  if (!f413_run_session_guard_prepare(&guard))
  {
    if (enc_started_l)
    {
      f413_run_session_encoder_stop_l();
    }
    if (enc_started_r)
    {
      f413_run_session_encoder_stop_r();
    }
    trace_printf("[RUN-TEST] search-safe canceled(guard init fail)\r\n");
    return;
  }

  f413_run_session_trace_on_run_start();

  for (step = 0U; step < explore_steps; step++)
  {
    f413_run_session_trace_set_mode_flags((uint16_t)(search_safe_flag | motor_fwd_flag));
    f413_run_session_motor_set(true, true, true, safe_duty, safe_duty);
    abort_reason = f413_run_session_wait_with_auto_step_guarded(forward_ms, &guard);
    if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }

    f413_run_session_trace_set_mode_flags((uint16_t)(search_safe_flag | motor_coast_flag));
    f413_run_session_motor_set(false, true, true, 0U, 0U);
    abort_reason = f413_run_session_wait_with_auto_step_guarded(coast_ms, &guard);
    if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }
  }

  f413_run_session_motor_set(false, true, true, 0U, 0U);
  if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    f413_run_session_trace_set_mode_flags((uint16_t)(search_safe_flag |
        f413_run_session_abort_reason_to_trace_flag(abort_reason)));
    f413_run_session_trace_auto_step();
  }
  f413_run_session_trace_set_mode_flags(0U);
  f413_run_session_trace_on_run_stop();

  if (enc_started_l)
  {
    f413_run_session_encoder_stop_l();
  }
  if (enc_started_r)
  {
    f413_run_session_encoder_stop_r();
  }

  f413_run_session_guard_cleanup(&guard);

  if (abort_reason == F413_RUN_SESSION_ABORT_SWITCH)
  {
    trace_printf("[RUN-TEST] search-safe aborted by switch\r\n");
  }
  else if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    trace_printf("[RUN-TEST] search-safe aborted(%s)\r\n",
                 f413_run_session_abort_reason_to_text(abort_reason));
  }
  else
  {
    trace_printf("[RUN-TEST] search-safe end\r\n");
  }
}

void f413_run_session_run_shortest_safe_trace_once(uint16_t shortest_safe_flag,
                                                   uint16_t motor_fwd_flag,
                                                   uint16_t motor_coast_flag,
                                                   uint16_t motor_rev_flag,
                                                   uint16_t safe_duty,
                                                   uint32_t forward_ms,
                                                   uint32_t turn_ms,
                                                   uint32_t coast_ms)
{
  bool enc_started_l = false;
  bool enc_started_r = false;
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};

  if (f413_run_session_trace_auto_is_enabled())
  {
    trace_printf("[RUN-TEST] busy(auto already running)\r\n");
    return;
  }

  if (f413_run_session_stop_switch_pressed())
  {
    trace_printf("[RUN-TEST] shortest-safe canceled(start switch pressed)\r\n");
    return;
  }

  trace_printf("[RUN-TEST] shortest-safe start (low speed, press switch to abort)\r\n");

  f413_run_session_encoder_stop_all();
  f413_run_session_encoder_reset_all();

  if (f413_run_session_encoder_start_l())
  {
    enc_started_l = true;
  }
  if (f413_run_session_encoder_start_r())
  {
    enc_started_r = true;
  }

  if (!f413_run_session_guard_prepare(&guard))
  {
    if (enc_started_l)
    {
      f413_run_session_encoder_stop_l();
    }
    if (enc_started_r)
    {
      f413_run_session_encoder_stop_r();
    }
    trace_printf("[RUN-TEST] shortest-safe canceled(guard init fail)\r\n");
    return;
  }

  f413_run_session_trace_on_run_start();

  f413_run_session_trace_set_mode_flags((uint16_t)(shortest_safe_flag | motor_fwd_flag));
  f413_run_session_motor_set(true, true, true, safe_duty, safe_duty);
  abort_reason = f413_run_session_wait_with_auto_step_guarded(forward_ms, &guard);

  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    f413_run_session_trace_set_mode_flags((uint16_t)(shortest_safe_flag | motor_coast_flag));
    f413_run_session_motor_set(false, true, true, 0U, 0U);
    abort_reason = f413_run_session_wait_with_auto_step_guarded(coast_ms, &guard);
  }

  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    f413_run_session_trace_set_mode_flags((uint16_t)(shortest_safe_flag | motor_rev_flag));
    f413_run_session_motor_set(true, true, false, safe_duty, safe_duty);
    abort_reason = f413_run_session_wait_with_auto_step_guarded(turn_ms, &guard);
  }

  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    f413_run_session_trace_set_mode_flags((uint16_t)(shortest_safe_flag | motor_coast_flag));
    f413_run_session_motor_set(false, true, true, 0U, 0U);
    abort_reason = f413_run_session_wait_with_auto_step_guarded(coast_ms, &guard);
  }

  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    f413_run_session_trace_set_mode_flags((uint16_t)(shortest_safe_flag | motor_fwd_flag));
    f413_run_session_motor_set(true, true, true, safe_duty, safe_duty);
    abort_reason = f413_run_session_wait_with_auto_step_guarded(forward_ms, &guard);
  }

  f413_run_session_motor_set(false, true, true, 0U, 0U);
  if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    f413_run_session_trace_set_mode_flags((uint16_t)(shortest_safe_flag |
        f413_run_session_abort_reason_to_trace_flag(abort_reason)));
    f413_run_session_trace_auto_step();
  }
  f413_run_session_trace_set_mode_flags(0U);
  f413_run_session_trace_on_run_stop();

  if (enc_started_l)
  {
    f413_run_session_encoder_stop_l();
  }
  if (enc_started_r)
  {
    f413_run_session_encoder_stop_r();
  }

  f413_run_session_guard_cleanup(&guard);

  if (abort_reason == F413_RUN_SESSION_ABORT_SWITCH)
  {
    trace_printf("[RUN-TEST] shortest-safe aborted by switch\r\n");
  }
  else if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    trace_printf("[RUN-TEST] shortest-safe aborted(%s)\r\n",
                 f413_run_session_abort_reason_to_text(abort_reason));
  }
  else
  {
    trace_printf("[RUN-TEST] shortest-safe end\r\n");
  }
}

uint16_t f413_run_session_abort_reason_to_trace_flag(f413_run_session_abort_reason_t reason)
{
  switch (reason)
  {
    case F413_RUN_SESSION_ABORT_SWITCH:
      return F413_RUN_SESSION_TRACE_ABORT_SWITCH_FLAG;
    case F413_RUN_SESSION_ABORT_WALL_FAULT:
      return F413_RUN_SESSION_TRACE_ABORT_WALL_FAULT_FLAG;
    case F413_RUN_SESSION_ABORT_ENCODER_FAULT:
      return F413_RUN_SESSION_TRACE_ABORT_ENCODER_FAULT_FLAG;
    case F413_RUN_SESSION_ABORT_IMU_FAULT:
      return F413_RUN_SESSION_TRACE_ABORT_IMU_FAULT_FLAG;
    case F413_RUN_SESSION_ABORT_NONE:
    default:
      return 0U;
  }
}

const char* f413_run_session_abort_reason_to_text(f413_run_session_abort_reason_t reason)
{
  switch (reason)
  {
    case F413_RUN_SESSION_ABORT_SWITCH:
      return "switch pressed";
    case F413_RUN_SESSION_ABORT_WALL_FAULT:
      return "wall sensor fault";
    case F413_RUN_SESSION_ABORT_ENCODER_FAULT:
      return "encoder jump fault";
    case F413_RUN_SESSION_ABORT_IMU_FAULT:
      return "imu fault";
    case F413_RUN_SESSION_ABORT_NONE:
    default:
      return "none";
  }
}
