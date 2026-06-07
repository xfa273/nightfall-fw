#ifndef F413_CONTROL_TUNE_RUN_H_
#define F413_CONTROL_TUNE_RUN_H_

#include <stdbool.h>
#include <stdint.h>

#include "f413_run_session.h"

typedef bool (*f413_control_tune_bool_fn)(void);
typedef uint32_t (*f413_control_tune_tick_fn)(void);
typedef void (*f413_control_tune_void_fn)(void);
typedef void (*f413_control_tune_trace_flags_fn)(uint16_t flags);
typedef void (*f413_control_tune_result_fn)(uint8_t test_id,
                                            f413_run_session_abort_reason_t abort_reason,
                                            float distance_mm,
                                            float angle_deg);

typedef struct {
  f413_control_tune_bool_fn stop_switch_pressed;
  f413_control_tune_tick_fn get_tick_ms;
  f413_control_tune_bool_fn trace_auto_is_enabled;
  f413_control_tune_void_fn trace_on_run_start;
  f413_control_tune_void_fn trace_on_run_stop;
  f413_control_tune_trace_flags_fn trace_set_mode_flags;
  f413_control_tune_void_fn trace_auto_step;
  f413_control_tune_result_fn record_result;
  uint32_t timeout_ms;
  uint16_t trace_tune_flag;
  uint16_t trace_motor_fwd_flag;
  uint16_t trace_motor_rev_flag;
} f413_control_tune_run_config_t;

void f413_control_tune_run_config(const f413_control_tune_run_config_t* config);
const char* f413_control_tune_axis_name(uint8_t axis);
const char* f413_control_tune_pattern_name(uint8_t pattern);
void f413_control_tune_run_once(uint8_t axis, uint8_t set, uint8_t pattern);
void f413_control_tune_emit_extra_csv_meta(void);

#endif
