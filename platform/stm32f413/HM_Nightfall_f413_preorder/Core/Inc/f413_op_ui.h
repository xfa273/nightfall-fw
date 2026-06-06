#ifndef F413_OP_UI_H_
#define F413_OP_UI_H_

#include <stdbool.h>
#include <stdint.h>

#define F413_OP_UI_LEVEL_TOP  (0U)
#define F413_OP_UI_LEVEL_CASE (1U)
#define F413_OP_UI_LEVEL_SUB  (2U)

typedef bool (*f413_op_ui_bool_fn)(void);

typedef enum {
  F413_OP_UI_ACTION_NONE = 0,
  F413_OP_UI_ACTION_SEARCH_TRACE_ENTRY,
  F413_OP_UI_ACTION_SHORTEST_TRACE_ENTRY,
  F413_OP_UI_ACTION_TEST_RUN_1,
  F413_OP_UI_ACTION_TEST_RUN_2,
  F413_OP_UI_ACTION_TEST_RUN_3,
  F413_OP_UI_ACTION_TEST_RUN_5,
  F413_OP_UI_ACTION_IMU_TEST,
  F413_OP_UI_ACTION_ENCODER_TEST,
  F413_OP_UI_ACTION_WALL_SENSOR_TEST,
  F413_OP_UI_ACTION_FAN_PWM_TEST,
  F413_OP_UI_ACTION_TRACE_DUMP_BIN_ALL,
  F413_OP_UI_ACTION_NVM_STATUS,
  F413_OP_UI_ACTION_IDENTITY_STATUS,
  F413_OP_UI_ACTION_SENSOR_PARAMS_STATUS,
  F413_OP_UI_ACTION_CONTROL_TUNE_SUB,
  F413_OP_UI_ACTION_PATH_CASE0_SUB
} f413_op_ui_action_t;

typedef void (*f413_op_ui_execute_action_fn)(f413_op_ui_action_t action,
                                             uint8_t mode,
                                             uint8_t op_case,
                                             uint8_t sub);

typedef struct {
  f413_op_ui_bool_fn can_accept_input;
  f413_op_ui_bool_fn stop_switch_pressed;
  f413_op_ui_bool_fn enter_sensor_active;
  f413_op_ui_execute_action_fn execute_action;
} f413_op_ui_config_t;

void f413_op_ui_config(const f413_op_ui_config_t* config);
const char* f413_op_ui_mode_name(uint8_t mode);
const char* f413_op_ui_level_name(uint8_t level);
const char* f413_op_ui_case_name(uint8_t mode, uint8_t op_case);
const char* f413_op_ui_sub_name(uint8_t mode, uint8_t sub);
uint8_t f413_op_ui_get_mode(void);
uint8_t f413_op_ui_get_case(void);
uint8_t f413_op_ui_get_sub(void);
uint8_t f413_op_ui_get_level(void);
uint8_t f413_op_ui_selected_value(void);
void f413_op_ui_print_selection(uint8_t level, uint8_t mode, uint8_t op_case, uint8_t sub);
void f413_op_ui_print_current_selection(void);
void f413_op_ui_increment_selection(void);
void f413_op_ui_enter_selection(void);
void f413_op_ui_step(uint32_t now_ms);
void f413_op_ui_uart_push_once(void);
void f413_op_ui_uart_enter_once(void);

#endif
