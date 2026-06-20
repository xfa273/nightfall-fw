#include "f413_op_ui.h"

#include "f413_hw.h"
#include "stm32f4xx_hal.h"
#include "trace.h"

#define F413_OP_UI_SELECT_MAX (9U)
#define F413_OP_UI_POLL_MS (40U)
#define F413_OP_UI_BUTTON_LOCK_MS (250U)
#define F413_OP_UI_ENTER_STREAK (3U)

static f413_op_ui_config_t s_config;
static uint8_t s_mode = 0U;
static uint8_t s_case = 0U;
static uint8_t s_sub = 0U;
static uint8_t s_level = F413_OP_UI_LEVEL_TOP;
static uint32_t s_next_ui_poll_ms = 0U;
static uint32_t s_button_lock_until_ms = 0U;
static uint8_t s_enter_streak = 0U;
static bool s_button_prev_pressed = false;
static bool s_enter_latched = false;

static bool f413_op_ui_can_accept_input(void)
{
  return (s_config.can_accept_input == NULL) || s_config.can_accept_input();
}

static bool f413_op_ui_stop_switch_pressed(void)
{
  return (s_config.stop_switch_pressed != NULL) && s_config.stop_switch_pressed();
}

static bool f413_op_ui_enter_sensor_active(void)
{
  return (s_config.enter_sensor_active != NULL) && s_config.enter_sensor_active();
}

static void f413_op_ui_beep_mode(uint8_t mode)
{
  uint16_t period = (uint16_t)((11U - (uint16_t)mode) * 400U);
  f413_hw_buzzer_beep_ms(period, 200U);
}

static void f413_op_ui_set_selected_value(uint8_t value)
{
  if (s_level == F413_OP_UI_LEVEL_CASE)
  {
    s_case = value;
  }
  else if (s_level == F413_OP_UI_LEVEL_SUB)
  {
    s_sub = value;
  }
  else
  {
    s_mode = value;
  }
}

static void f413_op_ui_after_execute(void)
{
  s_enter_streak = 0U;
  s_button_lock_until_ms = HAL_GetTick() + F413_OP_UI_BUTTON_LOCK_MS;
  f413_hw_show_mode_leds(f413_op_ui_selected_value());
}

static void f413_op_ui_execute_action(f413_op_ui_action_t action,
                                      uint8_t mode,
                                      uint8_t op_case,
                                      uint8_t sub)
{
  if ((action != F413_OP_UI_ACTION_NONE) && (s_config.execute_action != NULL))
  {
    s_config.execute_action(action, mode, op_case, sub);
  }
}

static void f413_op_ui_execute_case(uint8_t mode, uint8_t op_case)
{
  trace_printf("[OP-UI] execute mode=%u %s case=%u %s\r\n",
               (unsigned int)mode,
               f413_op_ui_mode_name(mode),
               (unsigned int)op_case,
               f413_op_ui_case_name(mode, op_case));

  switch (mode)
  {
    case 1U:
      if (op_case == 4U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_SEARCH_TRACE_ENTRY, mode, op_case, 0xFFU);
      }
      else
      {
        trace_printf("[OP-UI] no-op: F405 mode1 case%u is not fully ported on F413 yet\r\n",
                     (unsigned int)op_case);
      }
      break;
    case 2U:
    case 3U:
    case 4U:
    case 5U:
    case 6U:
    case 7U:
      if ((op_case >= 1U) && (op_case <= 9U))
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_SHORTEST_TRACE_ENTRY, mode, op_case, 0xFFU);
      }
      else
      {
        trace_printf("[OP-UI] no-op: shortest case%u is not assigned\r\n",
                     (unsigned int)op_case);
      }
      break;
    case 8U:
      if (op_case == 1U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_TEST_RUN_1, mode, op_case, 0xFFU);
      }
      else if (op_case == 2U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_TEST_RUN_2, mode, op_case, 0xFFU);
      }
      else if (op_case == 3U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_TEST_RUN_3, mode, op_case, 0xFFU);
      }
      else if (op_case == 4U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_TEST_RUN_5, mode, op_case, 0xFFU);
      }
      else
      {
        trace_printf("[OP-UI] no-op: F405 test_run case%u is not fully ported on F413 yet\r\n",
                     (unsigned int)op_case);
      }
      break;
    case 9U:
      if (op_case == 1U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_IMU_TEST, mode, op_case, 0xFFU);
      }
      else if (op_case == 2U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_ENCODER_TEST, mode, op_case, 0xFFU);
      }
      else if (op_case == 3U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_WALL_SENSOR_TEST, mode, op_case, 0xFFU);
      }
      else if (op_case == 4U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_FAN_PWM_TEST, mode, op_case, 0xFFU);
      }
      else if (op_case == 5U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_TRACE_DUMP_BIN_ALL, mode, op_case, 0xFFU);
      }
      else if (op_case == 6U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_WALL_SENSOR_TEST, mode, op_case, 0xFFU);
      }
      else if (op_case == 7U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_NVM_STATUS, mode, op_case, 0xFFU);
      }
      else if (op_case == 8U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_IDENTITY_STATUS, mode, op_case, 0xFFU);
      }
      else if (op_case == 9U)
      {
        f413_op_ui_execute_action(F413_OP_UI_ACTION_SENSOR_PARAMS_STATUS, mode, op_case, 0xFFU);
      }
      else
      {
        trace_printf("[OP-UI] no-op: F405 test_mode case%u is not fully ported on F413 yet\r\n",
                     (unsigned int)op_case);
      }
      break;
    default:
      trace_printf("[OP-UI] no-op: F405 mode%u case%u is not fully ported on F413 yet\r\n",
                   (unsigned int)mode,
                   (unsigned int)op_case);
      break;
  }
}

static void f413_op_ui_execute_sub(uint8_t mode, uint8_t sub)
{
  trace_printf("[OP-UI] execute mode=%u %s case=0 sub=%u %s\r\n",
               (unsigned int)mode,
               f413_op_ui_mode_name(mode),
               (unsigned int)sub,
               f413_op_ui_sub_name(mode, sub));

  if (mode == 9U)
  {
    f413_op_ui_execute_action(F413_OP_UI_ACTION_CONTROL_TUNE_SUB, mode, 0U, sub);
  }
  else if ((mode >= 2U) && (mode <= 7U))
  {
    f413_op_ui_execute_action(F413_OP_UI_ACTION_PATH_CASE0_SUB, mode, 0U, sub);
  }
  else if (mode == 1U)
  {
    if (sub == 3U)
    {
      f413_op_ui_execute_action(F413_OP_UI_ACTION_TEST_RUN_1, mode, 0U, sub);
    }
    else
    {
      trace_printf("[OP-UI] no-op: F405 mode1 case0 sub%u is not fully ported on F413 yet\r\n",
                   (unsigned int)sub);
    }
  }
  else
  {
    trace_printf("[OP-UI] no-op: unsupported sub selection\r\n");
  }
}

void f413_op_ui_config(const f413_op_ui_config_t* config)
{
  if (config == NULL)
  {
    s_config.can_accept_input = NULL;
    s_config.stop_switch_pressed = NULL;
    s_config.enter_sensor_active = NULL;
    s_config.execute_action = NULL;
    return;
  }

  s_config = *config;
}

const char* f413_op_ui_mode_name(uint8_t mode)
{
  switch (mode)
  {
    case 0U: return "idle";
    case 1U: return "search";
    case 2U: return "shortest-mode2";
    case 3U: return "shortest-mode3";
    case 4U: return "shortest-mode4";
    case 5U: return "shortest-mode5";
    case 6U: return "shortest-mode6";
    case 7U: return "shortest-mode7";
    case 8U: return "test-run";
    case 9U: return "test-mode";
    default: return "unknown";
  }
}

const char* f413_op_ui_level_name(uint8_t level)
{
  switch (level)
  {
    case F413_OP_UI_LEVEL_TOP: return "mode";
    case F413_OP_UI_LEVEL_CASE: return "case";
    case F413_OP_UI_LEVEL_SUB: return "sub";
    default: return "unknown";
  }
}

const char* f413_op_ui_case_name(uint8_t mode, uint8_t op_case)
{
  if (mode == 1U)
  {
    switch (op_case)
    {
      case 0U: return "search test tune";
      case 1U: return "standard goal-full search";
      case 2U: return "standard full search";
      case 3U: return "standard goal-return search";
      case 4U: return "standard goal-only search";
      case 5U: return "low goal-full search";
      case 6U: return "low full search";
      case 7U: return "low goal-return search";
      case 8U: return "low goal-only search";
      default: return "not assigned";
    }
  }
  if ((mode >= 2U) && (mode <= 7U))
  {
    if (op_case == 0U)
    {
      return "turn/diagonal/straight test";
    }
    return "shortest case";
  }
  if (mode == 8U)
  {
    switch (op_case)
    {
      case 0U: return "set position";
      case 1U: return "get log short straight";
      case 2U: return "get log long straight";
      case 3U: return "right 90 turn log";
      case 4U: return "straight plus turn";
      case 5U: return "large right 90";
      case 6U: return "rotate right 90";
      case 7U: return "large turn check";
      case 8U: return "front match position";
      case 9U: return "print log";
      default: return "unknown";
    }
  }
  if (mode == 9U)
  {
    switch (op_case)
    {
      case 0U: return "inner loop tune";
      case 1U: return "IMU check";
      case 2U: return "encoder check";
      case 3U: return "sensor AD check";
      case 4U: return "fan noise check";
      case 5U: return "dump latest full log bin";
      case 6U: return "wall threshold check";
      case 7U: return "NVM check";
      case 8U: return "identity check";
      case 9U: return "sensor parameter save";
      default: return "unknown";
    }
  }
  return "unknown";
}

const char* f413_op_ui_sub_name(uint8_t mode, uint8_t sub)
{
  if (mode == 9U)
  {
    switch (sub)
    {
      case 0U: return "velocity step 300";
      case 1U: return "velocity trapezoid 300";
      case 2U: return "omega step 1000";
      case 3U: return "omega trapezoid 1000";
      case 4U: return "distance rate-step 270";
      case 5U: return "distance rate-trapezoid 270";
      case 6U: return "angle rate-step 360";
      case 7U: return "angle rate-trapezoid 360";
      case 8U: return "velocity trapezoid 1000";
      case 9U: return "omega trapezoid 1000";
      default: return "unknown";
    }
  }
  if (mode == 1U)
  {
    switch (sub)
    {
      case 0U: return "reserved";
      case 1U: return "standard speed turn test";
      case 2U: return "low speed turn test";
      case 3U: return "straight 3-section test";
      default: return "not assigned";
    }
  }
  if ((mode >= 2U) && (mode <= 7U))
  {
    switch (sub)
    {
      case 0U: return "normal R90 turn";
      case 1U: return "large 90 turn";
      case 2U: return "large 180 turn";
      case 3U: return "diagonal 45-in";
      case 4U: return "diagonal 45-out";
      case 5U: return "diagonal V90";
      case 6U: return "diagonal 135-in";
      case 7U: return "diagonal 135-out";
      case 8U: return "straight slow";
      case 9U: return "straight fast";
      default: return "unknown";
    }
  }
  return "unknown";
}

uint8_t f413_op_ui_get_mode(void)
{
  return s_mode;
}

uint8_t f413_op_ui_get_case(void)
{
  return s_case;
}

uint8_t f413_op_ui_get_sub(void)
{
  return s_sub;
}

uint8_t f413_op_ui_get_level(void)
{
  return s_level;
}

uint8_t f413_op_ui_selected_value(void)
{
  if (s_level == F413_OP_UI_LEVEL_CASE)
  {
    return s_case;
  }
  if (s_level == F413_OP_UI_LEVEL_SUB)
  {
    return s_sub;
  }
  return s_mode;
}

void f413_op_ui_print_selection(uint8_t level, uint8_t mode, uint8_t op_case, uint8_t sub)
{
  if (level == F413_OP_UI_LEVEL_CASE)
  {
    trace_printf("[OP-UI] mode=%u %s case=%u %s\r\n",
                 (unsigned int)mode,
                 f413_op_ui_mode_name(mode),
                 (unsigned int)op_case,
                 f413_op_ui_case_name(mode, op_case));
  }
  else if (level == F413_OP_UI_LEVEL_SUB)
  {
    trace_printf("[OP-UI] mode=%u %s case=0 sub=%u %s\r\n",
                 (unsigned int)mode,
                 f413_op_ui_mode_name(mode),
                 (unsigned int)sub,
                 f413_op_ui_sub_name(mode, sub));
  }
  else
  {
    trace_printf("[OP-UI] mode=%u %s\r\n",
                 (unsigned int)mode,
                 f413_op_ui_mode_name(mode));
  }
}

void f413_op_ui_print_current_selection(void)
{
  f413_op_ui_print_selection(s_level, s_mode, s_case, s_sub);
}

void f413_op_ui_increment_selection(void)
{
  uint8_t selected = f413_op_ui_selected_value();
  selected++;
  if (selected > F413_OP_UI_SELECT_MAX)
  {
    selected = 0U;
  }
  f413_op_ui_set_selected_value(selected);
  s_button_lock_until_ms = HAL_GetTick() + F413_OP_UI_BUTTON_LOCK_MS;
  s_enter_streak = 0U;
  f413_hw_show_mode_leds(selected);
  f413_op_ui_beep_mode(selected);
  f413_op_ui_print_current_selection();
}

void f413_op_ui_enter_selection(void)
{
  uint8_t selected = f413_op_ui_selected_value();

  f413_hw_op_beep_enter();
  f413_hw_set_all_leds(GPIO_PIN_SET);
  HAL_Delay(120U);

  if (s_level == F413_OP_UI_LEVEL_TOP)
  {
    if (s_mode == 0U)
    {
      trace_printf("[OP-UI] execute mode=0 idle\r\n");
      f413_op_ui_after_execute();
      return;
    }

    s_level = F413_OP_UI_LEVEL_CASE;
    s_case = 0U;
    trace_printf("[OP-UI] enter mode=%u %s; select case 0..9\r\n",
                 (unsigned int)s_mode,
                 f413_op_ui_mode_name(s_mode));
    f413_op_ui_print_current_selection();
    f413_op_ui_after_execute();
    return;
  }

  if (s_level == F413_OP_UI_LEVEL_CASE)
  {
    if ((s_case == 0U) &&
        (((s_mode >= 1U) && (s_mode <= 7U)) || (s_mode == 9U)))
    {
      s_level = F413_OP_UI_LEVEL_SUB;
      s_sub = 0U;
      trace_printf("[OP-UI] enter mode=%u case=0; select sub 0..9\r\n",
                   (unsigned int)s_mode);
      f413_op_ui_print_current_selection();
      f413_op_ui_after_execute();
      return;
    }
    f413_op_ui_execute_case(s_mode, selected);
    f413_op_ui_after_execute();
    return;
  }

  f413_op_ui_execute_sub(s_mode, selected);
  s_level = F413_OP_UI_LEVEL_CASE;
  f413_op_ui_after_execute();
}

void f413_op_ui_step(uint32_t now_ms)
{
  bool button_pressed;
  bool enter_active;

  if ((int32_t)(now_ms - s_next_ui_poll_ms) < 0)
  {
    return;
  }
  s_next_ui_poll_ms = now_ms + F413_OP_UI_POLL_MS;

  if (!f413_op_ui_can_accept_input())
  {
    return;
  }

  button_pressed = f413_op_ui_stop_switch_pressed();
  if (button_pressed)
  {
    if (!s_button_prev_pressed && ((int32_t)(now_ms - s_button_lock_until_ms) >= 0))
    {
      f413_op_ui_increment_selection();
    }
    s_button_prev_pressed = true;
    return;
  }
  s_button_prev_pressed = false;

  enter_active = f413_op_ui_enter_sensor_active();
  if (enter_active)
  {
    if (!s_enter_latched)
    {
      if (s_enter_streak < F413_OP_UI_ENTER_STREAK)
      {
        s_enter_streak++;
      }
      if (s_enter_streak >= F413_OP_UI_ENTER_STREAK)
      {
        s_enter_latched = true;
        f413_op_ui_enter_selection();
      }
    }
  }
  else
  {
    s_enter_streak = 0U;
    s_enter_latched = false;
  }
}

void f413_op_ui_uart_push_once(void)
{
  if (!f413_op_ui_can_accept_input())
  {
    trace_printf("[OP-UART] ignored: operation UI is busy\r\n");
    return;
  }
  s_button_prev_pressed = false;
  f413_op_ui_increment_selection();
}

void f413_op_ui_uart_enter_once(void)
{
  if (!f413_op_ui_can_accept_input())
  {
    trace_printf("[OP-UART] ignored: operation UI is busy\r\n");
    return;
  }
  s_enter_streak = 0U;
  s_enter_latched = true;
  f413_op_ui_enter_selection();
}
