#include "f413_control_tune_run.h"

#include "f413_control.h"
#include "trace.h"

#define F413_CONTROL_TUNE_DEFAULT_TIMEOUT_MS (1500U)

static f413_control_tune_run_config_t g_config;
static bool g_last_tune_valid = false;
static uint8_t g_last_tune_axis = 0U;
static uint8_t g_last_tune_set = 0U;
static uint8_t g_last_tune_pattern = 0U;

static uint32_t f413_control_tune_tick(void)
{
  if (g_config.get_tick_ms != NULL)
  {
    return g_config.get_tick_ms();
  }
  return 0U;
}

static uint32_t f413_control_tune_timeout_ms(void)
{
  return (g_config.timeout_ms != 0U) ? g_config.timeout_ms : F413_CONTROL_TUNE_DEFAULT_TIMEOUT_MS;
}

static void f413_control_tune_set_mode_flags(uint16_t flags)
{
  if (g_config.trace_set_mode_flags != NULL)
  {
    g_config.trace_set_mode_flags(flags);
  }
}

static void f413_control_tune_trace_auto_step(void)
{
  if (g_config.trace_auto_step != NULL)
  {
    g_config.trace_auto_step();
  }
}

static void f413_control_tune_trace_on_run_start(void)
{
  if (g_config.trace_on_run_start != NULL)
  {
    g_config.trace_on_run_start();
  }
}

static void f413_control_tune_trace_on_run_stop(void)
{
  if (g_config.trace_on_run_stop != NULL)
  {
    g_config.trace_on_run_stop();
  }
}

static void f413_control_tune_record_result(f413_run_session_abort_reason_t abort_reason)
{
  if (g_config.record_result != NULL)
  {
    g_config.record_result((uint8_t)'0',
                           abort_reason,
                           f413_ctrl_get_distance(),
                           f413_ctrl_get_angle());
  }
}

void f413_control_tune_run_config(const f413_control_tune_run_config_t* config)
{
  if (config != NULL)
  {
    g_config = *config;
  }
}

const char* f413_control_tune_axis_name(uint8_t axis)
{
  switch (axis)
  {
    case F413_CTRL_TUNE_AXIS_VELOCITY: return "velocity";
    case F413_CTRL_TUNE_AXIS_OMEGA: return "omega";
    case F413_CTRL_TUNE_AXIS_DISTANCE: return "distance";
    case F413_CTRL_TUNE_AXIS_ANGLE: return "angle";
    default: return "unknown";
  }
}

const char* f413_control_tune_pattern_name(uint8_t pattern)
{
  switch (pattern)
  {
    case F413_CTRL_TUNE_PATTERN_STEP: return "step";
    case F413_CTRL_TUNE_PATTERN_TRIANGLE: return "triangle";
    case F413_CTRL_TUNE_PATTERN_TRAPEZOID: return "trapezoid";
    default: return "unknown";
  }
}

void f413_control_tune_run_once(uint8_t axis, uint8_t set, uint8_t pattern)
{
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};
  uint32_t deadline;
  uint16_t flags = g_config.trace_tune_flag;

  if ((g_config.trace_auto_is_enabled != NULL) && g_config.trace_auto_is_enabled())
  {
    trace_printf("[TUNE] busy(auto already running)\r\n");
    return;
  }
  if ((g_config.stop_switch_pressed != NULL) && g_config.stop_switch_pressed())
  {
    trace_printf("[TUNE] canceled(start switch pressed)\r\n");
    return;
  }
  if (!f413_run_session_guard_prepare(&guard))
  {
    trace_printf("[TUNE] canceled(guard init fail)\r\n");
    return;
  }

  if ((axis == F413_CTRL_TUNE_AXIS_VELOCITY) ||
      (axis == F413_CTRL_TUNE_AXIS_DISTANCE))
  {
    flags |= g_config.trace_motor_fwd_flag;
  }
  else
  {
    flags |= g_config.trace_motor_rev_flag;
  }

  g_last_tune_valid = true;
  g_last_tune_axis = axis;
  g_last_tune_set = set;
  g_last_tune_pattern = pattern;

  trace_printf("[TUNE] start axis=%s set=%u pattern=%s\r\n",
               f413_control_tune_axis_name(axis),
               (unsigned int)set,
               f413_control_tune_pattern_name(pattern));

  f413_control_tune_trace_on_run_start();
  f413_ctrl_start();
  f413_ctrl_tune_clear_done();
  f413_ctrl_tune_start(axis, set, pattern);

  deadline = f413_control_tune_tick() + f413_control_tune_timeout_ms();
  while (!f413_ctrl_tune_is_done())
  {
    if (f413_control_tune_tick() >= deadline)
    {
      abort_reason = F413_RUN_SESSION_ABORT_IMU_FAULT;
      break;
    }
    f413_control_tune_set_mode_flags(flags);
    abort_reason = f413_run_session_wait_with_auto_step_guarded(10U, &guard);
    if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }
  }

  f413_ctrl_tune_stop();
  f413_ctrl_stop();
  if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    f413_control_tune_set_mode_flags((uint16_t)(flags |
        f413_run_session_abort_reason_to_trace_flag(abort_reason)));
    f413_control_tune_trace_auto_step();
  }
  f413_control_tune_set_mode_flags(0U);
  f413_control_tune_trace_on_run_stop();
  f413_run_session_guard_cleanup(&guard);
  f413_control_tune_record_result(abort_reason);

  trace_printf("[TUNE] %s axis=%s set=%u pattern=%s dist=%.0fmm angle=%.0fdeg\r\n",
               (abort_reason == F413_RUN_SESSION_ABORT_NONE) ? "OK" :
               f413_run_session_abort_reason_to_text(abort_reason),
               f413_control_tune_axis_name(axis),
               (unsigned int)set,
               f413_control_tune_pattern_name(pattern),
               (double)f413_ctrl_get_distance(),
               (double)f413_ctrl_get_angle());
}

void f413_control_tune_emit_extra_csv_meta(void)
{
  if (!g_last_tune_valid)
  {
    return;
  }

  trace_printf("#tune_axis=%s\r\n", f413_control_tune_axis_name(g_last_tune_axis));
  trace_printf("#tune_set=%u\r\n", (unsigned int)g_last_tune_set);
  trace_printf("#tune_pattern=%s\r\n", f413_control_tune_pattern_name(g_last_tune_pattern));
  trace_printf("#tune_reserved_i32=primary_ref_x1000,axis,target_distance_x1000,target_angle_x1000\r\n");
  trace_printf("#tune_reserved_u16_0=axis_pattern\r\n");
  trace_printf("#tune_reserved_u16_1=set\r\n");
}
