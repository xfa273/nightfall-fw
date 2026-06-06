#include "f413_op_ui.h"

#include "trace.h"

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
