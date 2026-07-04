#ifndef F413_WALL_RUNTIME_H_
#define F413_WALL_RUNTIME_H_

#include <stdbool.h>
#include <stdint.h>

#include "f413_wall_sensor.h"
#include "nvm_trace_log.h"

#define F413_WALL_RUNTIME_TRACE_VERSION (1U)

typedef bool (*f413_wall_runtime_snapshot_fn)(f413_wall_sensor_snapshot_t* out);
typedef void (*f413_wall_runtime_delay_fn)(uint32_t ms);
typedef uint32_t (*f413_wall_runtime_tick_fn)(void);

typedef struct {
  f413_wall_runtime_snapshot_fn read_wall_snapshot;
  f413_wall_runtime_delay_fn delay_ms;
  f413_wall_runtime_tick_fn get_tick_ms;
  uint32_t monitor_ms;
  uint32_t monitor_sample_ms;
  uint16_t trace_motor_fwd_flag;
} f413_wall_runtime_config_t;

void f413_wall_runtime_config(const f413_wall_runtime_config_t* config);
void f413_wall_runtime_end_clear(void);
void f413_wall_runtime_set_control_gains(float kp_wall, float kp_diagonal);
void f413_wall_runtime_control_clear(void);
float f413_wall_runtime_latest_error(void);
void f413_wall_runtime_control_apply(bool straight_gate);
bool f413_wall_runtime_poll_wall_end(bool straight_gate);
void f413_wall_runtime_poll_diagonal(bool diagonal_gate);
bool f413_wall_runtime_wall_end_detected(float* right_dist_mm, float* left_dist_mm);
bool f413_wall_runtime_front_wall_reached(float ad_sum_threshold);
uint16_t f413_wall_runtime_trace_flags_from_snapshot(const f413_wall_sensor_snapshot_t* wall,
                                                     bool gate_on);
bool f413_wall_runtime_fill_observe(nvm_trace_log_record_t* out, uint16_t mode_flags);
bool f413_wall_runtime_fill_snapshot_fields(nvm_trace_log_record_t* out, uint16_t mode_flags);
void f413_wall_runtime_run_end_monitor_once(void);

#endif
