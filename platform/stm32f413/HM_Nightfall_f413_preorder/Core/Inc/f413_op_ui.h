#ifndef F413_OP_UI_H_
#define F413_OP_UI_H_

#include <stdbool.h>
#include <stdint.h>

#define F413_OP_UI_LEVEL_TOP  (0U)
#define F413_OP_UI_LEVEL_CASE (1U)
#define F413_OP_UI_LEVEL_SUB  (2U)

typedef bool (*f413_op_ui_bool_fn)(void);
typedef void (*f413_op_ui_execute_case_fn)(uint8_t mode, uint8_t op_case);
typedef void (*f413_op_ui_execute_sub_fn)(uint8_t mode, uint8_t sub);

typedef struct {
  f413_op_ui_bool_fn can_accept_input;
  f413_op_ui_bool_fn stop_switch_pressed;
  f413_op_ui_bool_fn enter_sensor_active;
  f413_op_ui_execute_case_fn execute_case;
  f413_op_ui_execute_sub_fn execute_sub;
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
