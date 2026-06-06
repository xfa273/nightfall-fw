#ifndef F413_OP_UI_H_
#define F413_OP_UI_H_

#include <stdint.h>

#define F413_OP_UI_LEVEL_TOP  (0U)
#define F413_OP_UI_LEVEL_CASE (1U)
#define F413_OP_UI_LEVEL_SUB  (2U)

const char* f413_op_ui_mode_name(uint8_t mode);
const char* f413_op_ui_level_name(uint8_t level);
const char* f413_op_ui_case_name(uint8_t mode, uint8_t op_case);
const char* f413_op_ui_sub_name(uint8_t mode, uint8_t sub);
void f413_op_ui_print_selection(uint8_t level, uint8_t mode, uint8_t op_case, uint8_t sub);

#endif
