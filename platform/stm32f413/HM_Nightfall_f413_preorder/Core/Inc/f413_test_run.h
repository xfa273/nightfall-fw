#ifndef F413_TEST_RUN_H_
#define F413_TEST_RUN_H_

#include <stdbool.h>
#include <stdint.h>

#include "f413_run_session.h"

typedef void (*f413_test_run_void_fn)(void);
typedef void (*f413_test_run_delay_fn)(uint32_t ms);
typedef uint32_t (*f413_test_run_tick_fn)(void);
typedef bool (*f413_test_run_bool_fn)(void);
typedef int16_t (*f413_test_run_encoder_count_fn)(void);
typedef bool (*f413_test_run_encoder_start_fn)(void);
typedef void (*f413_test_run_trace_context_fn)(uint8_t mode,
                                               uint8_t op_case,
                                               uint8_t sub,
                                               uint8_t test_id);
typedef void (*f413_test_run_trace_flags_fn)(uint16_t mode_flags);
typedef void (*f413_test_run_result_fn)(uint8_t test_id,
                                        f413_run_session_abort_reason_t abort_reason,
                                        float distance_mm,
                                        float angle_deg);
typedef void (*f413_test_run_motor_set_fn)(bool enable,
                                           bool left_forward,
                                           bool right_forward,
                                           uint16_t left_duty,
                                           uint16_t right_duty);

typedef struct {
  f413_test_run_bool_fn stop_switch_pressed;
  f413_test_run_delay_fn delay_ms;
  f413_test_run_tick_fn get_tick_ms;
  f413_test_run_trace_context_fn trace_set_context;
  f413_test_run_void_fn trace_on_run_start;
  f413_test_run_void_fn trace_on_run_stop;
  f413_test_run_trace_flags_fn trace_set_mode_flags;
  f413_test_run_void_fn trace_auto_step;
  f413_test_run_void_fn wall_control_apply_straight;
  f413_test_run_result_fn record_result;
  f413_test_run_void_fn encoder_stop_all;
  f413_test_run_void_fn encoder_reset_all;
  f413_test_run_void_fn encoder_center_all;
  f413_test_run_encoder_start_fn encoder_start_l;
  f413_test_run_encoder_start_fn encoder_start_r;
  f413_test_run_encoder_count_fn encoder_l_count;
  f413_test_run_encoder_count_fn encoder_r_count;
  f413_test_run_motor_set_fn motor_set;
  uint16_t trace_solver_path_flag;
  uint16_t trace_motor_fwd_flag;
  uint16_t trace_motor_coast_flag;
  uint16_t trace_motor_rev_flag;
} f413_test_run_config_t;

void f413_test_run_config(const f413_test_run_config_t* config);
const char* f413_test_run_name(uint8_t test_id);
const char* f413_test_run_arm_name(uint8_t test_id);
bool f413_test_run_is_button_test(uint8_t test_id);
bool f413_test_run_is_armed(void);
uint8_t f413_test_run_armed_id(void);
void f413_test_run_clear_arm(void);
void f413_test_run_arm_for_button(uint8_t test_id);
void f413_test_run_run_now(uint8_t test_id);
void f413_test_run_step_button_armed(void);

#endif
