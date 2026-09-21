#ifndef F413_MOTION_STOP_H_
#define F413_MOTION_STOP_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* These are termination/safety bounds, not replacements for tuned PID gains. */
#define F413_STOP_POSITION_TOL_MM (1.0f)
#define F413_STOP_SPEED_TOL_MM_S (10.0f)
#define F413_STOP_SETTLE_MS (20U)
#define F413_STOP_WALL_MAX_AGE_MS (20U)
#define F413_STOP_CORRECTION_MAX_MM_S (30.0f)

static inline void f413_motion_profile_advance(float* velocity, float* acceleration,
                                               float target, float dt)
{
  const float next = *velocity + *acceleration * dt;
  if (((*acceleration > 0.0f) && (next >= target)) ||
      ((*acceleration < 0.0f) && (next <= target)))
  {
    *velocity = target;
    *acceleration = 0.0f; /* No residual braking/acceleration feedforward. */
  }
  else
  {
    *velocity = next;
  }
}

static inline float f413_motion_profile_limit(float command, float target,
                                               float direction, bool stopped)
{
  if (stopped)
  {
    /* Permit bounded position correction after a zero-speed endpoint. */
    return fmaxf(-F413_STOP_CORRECTION_MAX_MM_S,
                 fminf(command, F413_STOP_CORRECTION_MAX_MM_S));
  }
  if (((direction > 0.0f) && (command > target)) ||
      ((direction < 0.0f) && (command < target)) ||
      ((direction == 0.0f) &&
       (((target >= 0.0f) && (command > target)) ||
        ((target < 0.0f) && (command < target)))))
  {
    return target;
  }
  return command;
}

typedef enum {
  F413_STOP_WAIT,
  F413_STOP_BRAKE,
  F413_STOP_COMPLETE,
  F413_STOP_WALL_FAULT
} f413_stop_action_t;

typedef struct {
  bool wall_handoff;
  uint16_t settled_ms;
} f413_stop_state_t;

/* Called once per guarded 1 ms wait. Near extrapolation is used only to stop,
 * never to authorize forward travel or a successful wall handoff. */
static inline f413_stop_action_t f413_stop_approach_step(
    f413_stop_state_t* state, float remaining_mm, float velocity_mm_s,
    bool profile_complete, bool allow_wall_handoff, bool front_valid,
    bool front_saturated, float fr_mm, float fl_mm, float sum_mm,
    float wall_target_mm, float too_close_mm)
{
  if (!isfinite(fr_mm) || !isfinite(fl_mm) || !isfinite(sum_mm) ||
      front_saturated || fr_mm < too_close_mm || fl_mm < too_close_mm)
  {
    return F413_STOP_WALL_FAULT;
  }
  if (allow_wall_handoff && front_valid && sum_mm <= wall_target_mm)
  {
    state->wall_handoff = true;
  }
  /* A handoff must still be valid when we finish braking. */
  if (state->wall_handoff && !front_valid)
  {
    return F413_STOP_WALL_FAULT;
  }
  const bool at_target = state->wall_handoff ||
      (profile_complete && isfinite(remaining_mm) &&
       fabsf(remaining_mm) <= F413_STOP_POSITION_TOL_MM);
  if (at_target && isfinite(velocity_mm_s) &&
      fabsf(velocity_mm_s) <= F413_STOP_SPEED_TOL_MM_S)
  {
    if (++state->settled_ms >= F413_STOP_SETTLE_MS)
    {
      return F413_STOP_COMPLETE;
    }
  }
  else
  {
    state->settled_ms = 0U;
  }
  return state->wall_handoff ? F413_STOP_BRAKE : F413_STOP_WAIT;
}

#endif
