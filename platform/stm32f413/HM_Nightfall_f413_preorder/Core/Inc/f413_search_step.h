#ifndef F413_SEARCH_STEP_H_
#define F413_SEARCH_STEP_H_

#include <stdbool.h>
#include <stdint.h>

#include "f413_run_features.h"
#include "f413_wall_sensor.h"

#define F413_SEARCH_STEP_TARGET_GOAL  (0U)
#define F413_SEARCH_STEP_TARGET_FULL  (1U)
#define F413_SEARCH_STEP_TARGET_START (2U)

#define F413_SEARCH_STEP_CASE0_TEST_NONE       (0U)
#define F413_SEARCH_STEP_CASE0_TEST_TURN_R90   (1U)
#define F413_SEARCH_STEP_CASE0_TEST_STRAIGHT_3 (2U)
#define F413_SEARCH_STEP_CASE0_TEST_SPIN_R720  (3U)

typedef bool (*f413_search_step_bool_fn)(void);
typedef void (*f413_search_step_void_fn)(void);
typedef uint32_t (*f413_search_step_tick_fn)(void);
typedef bool (*f413_search_step_wall_snapshot_fn)(f413_wall_sensor_snapshot_t* out);
typedef void (*f413_search_step_trace_context_fn)(uint8_t mode,
                                                  uint8_t op_case,
                                                  uint8_t sub,
                                                  uint8_t test_id);
typedef void (*f413_search_step_trace_flags_fn)(uint16_t mode_flags);

typedef struct {
  f413_search_step_bool_fn stop_switch_pressed;
  f413_search_step_tick_fn get_tick_ms;
  f413_search_step_wall_snapshot_fn read_wall_snapshot;
  f413_search_step_bool_fn trace_auto_is_enabled;
  f413_search_step_trace_context_fn trace_set_context;
  f413_search_step_void_fn trace_on_run_start;
  f413_search_step_void_fn trace_on_run_stop;
  f413_search_step_trace_flags_fn trace_set_mode_flags;
  f413_search_step_void_fn trace_auto_step;
  f413_search_step_void_fn wall_control_apply_straight;
  uint32_t path_timeout_ms;
  uint32_t path_coast_ms;
  float step_velocity_mm_s;
  float step_target_mm;
  float step_turn_deg;
  uint16_t trace_search_safe_flag;
  uint16_t trace_motor_fwd_flag;
  uint16_t trace_motor_coast_flag;
  uint16_t trace_motor_rev_flag;
} f413_search_step_config_t;

typedef struct {
  uint8_t param_index;
  uint8_t phase_count;
  uint8_t phases[2];
  const char* label;
  f413_run_features_t features;
} f413_search_step_case_config_t;

typedef struct {
  uint8_t param_index;
  uint8_t test_kind;
  const char* label;
  f413_run_features_t features;
} f413_search_step_case0_test_config_t;

void f413_search_step_config(const f413_search_step_config_t* config);
void f413_search_step_session_reset(void);
void f413_search_step_run_config_once(uint8_t op_case,
                                      const f413_search_step_case_config_t* case_config);
void f413_search_step_run_case0_test_once(
    uint8_t sub,
    const f413_search_step_case0_test_config_t* test_config);
void f413_search_step_run_search_case_once(uint8_t op_case);
void f413_search_step_run_decision_preview_once(void);
void f413_search_step_run_once(void);
void f413_search_step_run_map_probe_once(void);
void f413_search_step_run_status_once(void);
void f413_search_step_run_map_clear_once(void);
void f413_search_step_run_map_dump_once(void);

#endif
