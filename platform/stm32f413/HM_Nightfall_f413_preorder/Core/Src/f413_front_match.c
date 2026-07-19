#include "f413_front_match.h"

#include <math.h>
#include <stddef.h>

#include "params.h"

static float f413_front_match_clampf(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static uint16_t f413_front_match_elapsed_add(uint16_t value, uint16_t elapsed_ms)
{
  const uint32_t sum = (uint32_t)value + (uint32_t)elapsed_ms;

  return (sum > UINT16_MAX) ? UINT16_MAX : (uint16_t)sum;
}

static void f413_front_match_change_phase(f413_front_match_controller_t* controller,
                                          f413_front_match_phase_t phase,
                                          f413_front_match_output_t* output)
{
  if ((controller == NULL) || (output == NULL) || (controller->phase == phase))
  {
    return;
  }
  controller->phase = phase;
  controller->phase_elapsed_ms = 0U;
  controller->release_elapsed_ms = 0U;
  output->phase_changed = true;
}

static bool f413_front_match_within_target(float position_error_mm,
                                           float yaw_error_mm)
{
  return (fabsf(position_error_mm) <= MATCH_POS_TRANS_TOL_MM) &&
         (fabsf(yaw_error_mm) <= MATCH_POS_YAW_TOL_MM);
}

void f413_front_match_init(f413_front_match_controller_t* controller)
{
  if (controller == NULL)
  {
    return;
  }
  controller->phase = F413_FRONT_MATCH_PHASE_ALIGN_YAW;
  controller->phase_elapsed_ms = 0U;
  controller->release_elapsed_ms = 0U;
}

void f413_front_match_step(f413_front_match_controller_t* controller,
                           float position_error_mm,
                           float yaw_error_mm,
                           uint16_t elapsed_ms,
                           f413_front_match_output_t* output)
{
  if ((controller == NULL) || (output == NULL))
  {
    return;
  }

  output->velocity_mm_s = 0.0f;
  output->omega_deg_s = 0.0f;
  output->phase = controller->phase;
  output->phase_changed = false;
  output->complete = false;
  output->holding = false;

  switch (controller->phase)
  {
    case F413_FRONT_MATCH_PHASE_ALIGN_YAW:
      if (fabsf(yaw_error_mm) <= MATCH_POS_YAW_TOL_MM)
      {
        f413_front_match_change_phase(controller,
                                      F413_FRONT_MATCH_PHASE_SETTLE_YAW,
                                      output);
      }
      else
      {
        output->omega_deg_s = f413_front_match_clampf(
            MATCH_POS_KP_ROT_MM * yaw_error_mm,
            -MATCH_POS_OMEGA_MAX,
            MATCH_POS_OMEGA_MAX);
      }
      break;

    case F413_FRONT_MATCH_PHASE_SETTLE_YAW:
      controller->phase_elapsed_ms = f413_front_match_elapsed_add(
          controller->phase_elapsed_ms,
          elapsed_ms);
      if (controller->phase_elapsed_ms >= MATCH_POS_YAW_SETTLE_MS)
      {
        if (fabsf(yaw_error_mm) <= MATCH_POS_YAW_TOL_MM)
        {
          f413_front_match_change_phase(controller,
                                        F413_FRONT_MATCH_PHASE_ALIGN_POSITION,
                                        output);
        }
        else
        {
          f413_front_match_change_phase(controller,
                                        F413_FRONT_MATCH_PHASE_ALIGN_YAW,
                                        output);
        }
      }
      break;

    case F413_FRONT_MATCH_PHASE_ALIGN_POSITION:
      if (fabsf(yaw_error_mm) > MATCH_POS_YAW_RESTART_MM)
      {
        f413_front_match_change_phase(controller,
                                      F413_FRONT_MATCH_PHASE_ALIGN_YAW,
                                      output);
      }
      else if (fabsf(position_error_mm) <= MATCH_POS_TRANS_TOL_MM)
      {
        f413_front_match_change_phase(controller,
                                      F413_FRONT_MATCH_PHASE_SETTLE_FINAL,
                                      output);
      }
      else
      {
        output->velocity_mm_s = f413_front_match_clampf(
            MATCH_POS_KP_TRANS_MM * position_error_mm,
            -MATCH_POS_VEL_MAX,
            MATCH_POS_VEL_MAX);
      }
      break;

    case F413_FRONT_MATCH_PHASE_SETTLE_FINAL:
      controller->phase_elapsed_ms = f413_front_match_elapsed_add(
          controller->phase_elapsed_ms,
          elapsed_ms);
      if (controller->phase_elapsed_ms >= MATCH_POS_FINAL_SETTLE_MS)
      {
        if (f413_front_match_within_target(position_error_mm, yaw_error_mm))
        {
          f413_front_match_change_phase(controller,
                                        F413_FRONT_MATCH_PHASE_HOLD,
                                        output);
        }
        else if (fabsf(yaw_error_mm) > MATCH_POS_YAW_TOL_MM)
        {
          f413_front_match_change_phase(controller,
                                        F413_FRONT_MATCH_PHASE_ALIGN_YAW,
                                        output);
        }
        else
        {
          f413_front_match_change_phase(controller,
                                        F413_FRONT_MATCH_PHASE_ALIGN_POSITION,
                                        output);
        }
      }
      break;

    case F413_FRONT_MATCH_PHASE_HOLD:
      output->complete = true;
      output->holding = true;
      if ((fabsf(position_error_mm) > MATCH_POS_REACQUIRE_TRANS_MM) ||
          (fabsf(yaw_error_mm) > MATCH_POS_REACQUIRE_YAW_MM))
      {
        controller->release_elapsed_ms = f413_front_match_elapsed_add(
            controller->release_elapsed_ms,
            elapsed_ms);
      }
      else
      {
        controller->release_elapsed_ms = 0U;
      }

      if (controller->release_elapsed_ms >= MATCH_POS_REACQUIRE_MS)
      {
        output->complete = false;
        output->holding = false;
        f413_front_match_change_phase(
            controller,
            (fabsf(yaw_error_mm) > MATCH_POS_YAW_RESTART_MM)
                ? F413_FRONT_MATCH_PHASE_ALIGN_YAW
                : F413_FRONT_MATCH_PHASE_ALIGN_POSITION,
            output);
      }
      break;

    default:
      f413_front_match_init(controller);
      output->phase_changed = true;
      break;
  }

  output->phase = controller->phase;
  if (controller->phase == F413_FRONT_MATCH_PHASE_HOLD)
  {
    output->complete = true;
    output->holding = true;
  }
}

const char* f413_front_match_phase_name(f413_front_match_phase_t phase)
{
  switch (phase)
  {
    case F413_FRONT_MATCH_PHASE_ALIGN_YAW: return "align-yaw";
    case F413_FRONT_MATCH_PHASE_SETTLE_YAW: return "settle-yaw";
    case F413_FRONT_MATCH_PHASE_ALIGN_POSITION: return "align-position";
    case F413_FRONT_MATCH_PHASE_SETTLE_FINAL: return "settle-final";
    case F413_FRONT_MATCH_PHASE_HOLD: return "hold";
    default: return "unknown";
  }
}
