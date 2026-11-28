#ifndef F413_WALL_RUNTIME_H_
#define F413_WALL_RUNTIME_H_

#include <stdbool.h>
#include <stdint.h>

#include "f413_wall_sensor.h"
#include "nvm_trace_log.h"

#define F413_WALL_RUNTIME_TRACE_VERSION              (2U)
#define F413_WALL_RUNTIME_END_DERIV_WINDOW_SAMPLES   (18U)
#define F413_WALL_RUNTIME_END_DERIV_BUFFER_SAMPLES   (36U)
#define F413_WALL_RUNTIME_END_DERIV_DIVISOR          (9)
#define F413_WALL_RUNTIME_END_DERIV_CONFIRM_SAMPLES  (2U)

/* Short difference supplements the existing detector in enabled wall-end gates. */
#define F413_WALL_RUNTIME_SHORT_WINDOW_MS                (4U)
#define F413_WALL_RUNTIME_SHORT_DROP_ADC                 (150)
#define F413_WALL_RUNTIME_SHORT_PRESENT_ADC              (300)
#define F413_WALL_RUNTIME_SHORT_CONFIRM_SAMPLES          (2U)
#define F413_WALL_RUNTIME_CHAINED_PREPARE_MS             (8U)
#define F413_WALL_RUNTIME_SHORT_MAX_SAMPLE_GAP_MS        (3U)

#if F413_WALL_RUNTIME_END_DERIV_BUFFER_SAMPLES != \
    (2U * F413_WALL_RUNTIME_END_DERIV_WINDOW_SAMPLES)
#error "wall-end derivative buffer must contain two equal windows"
#endif

#if F413_WALL_RUNTIME_END_DERIV_DIVISOR <= 0
#error "wall-end derivative divisor must be positive"
#endif

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
/* Start a new correction window; keep only fresh short history from the
 * preceding straight when the all-gates policy is enabled. */
void f413_wall_runtime_end_begin(void);
/* Foreground only: collect history without correction in the final turn tail,
 * then require fresh confirmations after entering the combined offsets. */
void f413_wall_runtime_chained_prepare_begin(void);
void f413_wall_runtime_chained_prepare_sample(void);
void f413_wall_runtime_chained_monitor_begin(void);
void f413_wall_runtime_chained_monitor_end(void);
void f413_wall_runtime_set_wall_end_thresholds(uint16_t right_high,
                                               uint16_t right_low,
                                               uint16_t left_high,
                                               uint16_t left_low);
void f413_wall_runtime_reset_wall_end_thresholds(void);
void f413_wall_runtime_set_control_gains(float kp_wall, float kp_diagonal);
void f413_wall_runtime_control_clear(void);
float f413_wall_runtime_latest_error(void);
void f413_wall_runtime_control_apply(bool straight_gate);
void f413_wall_runtime_poll_straight(bool wall_control_gate);
bool f413_wall_runtime_poll_wall_end(bool straight_gate);
/* Always monitor wall ends, independently of cardinal heading correction. */
bool f413_wall_runtime_poll_wall_end_with_control(bool wall_control_gate);
void f413_wall_runtime_poll_diagonal(bool diagonal_gate);
bool f413_wall_runtime_wall_end_detected(float* right_dist_mm, float* left_dist_mm);
/* False for a legacy or mixed-method latch; used for optional short-only follow. */
bool f413_wall_runtime_wall_end_detected_by_short(void);
bool f413_wall_runtime_front_wall_reached(float ad_sum_threshold);
uint16_t f413_wall_runtime_trace_flags_from_snapshot(const f413_wall_sensor_snapshot_t* wall,
                                                     bool gate_on);
bool f413_wall_runtime_fill_observe(nvm_trace_log_record_t* out, uint16_t mode_flags);
bool f413_wall_runtime_fill_snapshot_fields(nvm_trace_log_record_t* out, uint16_t mode_flags);
void f413_wall_runtime_run_end_monitor_once(void);

#endif
