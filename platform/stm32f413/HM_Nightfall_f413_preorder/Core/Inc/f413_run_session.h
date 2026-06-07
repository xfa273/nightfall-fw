#ifndef F413_RUN_SESSION_H_
#define F413_RUN_SESSION_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  F413_RUN_SESSION_ABORT_NONE = 0,
  F413_RUN_SESSION_ABORT_SWITCH,
  F413_RUN_SESSION_ABORT_WALL_FAULT,
  F413_RUN_SESSION_ABORT_ENCODER_FAULT,
  F413_RUN_SESSION_ABORT_IMU_FAULT,
} f413_run_session_abort_reason_t;

typedef struct {
  int16_t prev_encoder_l;
  int16_t prev_encoder_r;
  uint32_t next_wall_check_ms;
  uint32_t next_imu_check_ms;
} f413_run_session_guard_t;

typedef bool (*f413_run_session_bool_fn)(void);
typedef int16_t (*f413_run_session_encoder_count_fn)(void);
typedef void (*f413_run_session_void_fn)(void);
typedef void (*f413_run_session_set_mode_flags_fn)(uint16_t mode_flags);

typedef struct {
  f413_run_session_bool_fn stop_switch_pressed;
  f413_run_session_encoder_count_fn encoder_l_count;
  f413_run_session_encoder_count_fn encoder_r_count;
  f413_run_session_bool_fn wall_sensor_ok;
  f413_run_session_bool_fn imu_ok;
  f413_run_session_void_fn trace_auto_step;
  f413_run_session_bool_fn trace_auto_is_enabled;
  f413_run_session_void_fn trace_on_run_start;
  f413_run_session_void_fn trace_on_run_stop;
  f413_run_session_set_mode_flags_fn trace_set_mode_flags;
} f413_run_session_config_t;

void f413_run_session_config(const f413_run_session_config_t* config);
bool f413_run_session_guard_prepare(f413_run_session_guard_t* guard);
void f413_run_session_guard_cleanup(f413_run_session_guard_t* guard);
f413_run_session_abort_reason_t f413_run_session_guard_check(f413_run_session_guard_t* guard);
f413_run_session_abort_reason_t f413_run_session_wait_with_auto_step_guarded(uint32_t duration_ms,
                                                                             f413_run_session_guard_t* guard);
void f413_run_session_wait_with_auto_step(uint32_t duration_ms);
void f413_run_session_run_idle_trace_once(uint32_t duration_ms, uint16_t idle_mode_flag);
uint16_t f413_run_session_abort_reason_to_trace_flag(f413_run_session_abort_reason_t reason);
const char* f413_run_session_abort_reason_to_text(f413_run_session_abort_reason_t reason);

#endif
