#include "f413_path_run.h"

#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)

#include <math.h>

#include "legacy_path_codec.h"
#include "motion_time.h"
#include "params.h"
#include "shortest_run_params.h"

typedef enum
{
  F413_PATH_RUN_PREFLIGHT_OK = 0,
  F413_PATH_RUN_PREFLIGHT_INVALID_ARGUMENT,
  F413_PATH_RUN_PREFLIGHT_INVALID_LEGACY_PATH,
  F413_PATH_RUN_PREFLIGHT_INFEASIBLE_LINEAR,
  F413_PATH_RUN_PREFLIGHT_SPEED_DISCONTINUITY,
  F413_PATH_RUN_PREFLIGHT_PREPARED_CAPACITY,
} f413_path_run_preflight_status_t;

typedef struct
{
  f413_path_run_preflight_status_t status;
  NfLegacyPathStatus legacy_status;
  size_t index;
  uint16_t code;
} f413_path_run_preflight_result_t;

/*
 * A 16x16 shortest path cannot contain more than 256 distinct linear runs.
 * Keep only the controller inputs needed at execution time instead of the
 * double-precision host plan.  Building nf_motion_linear_plan() while the
 * motors are already running otherwise leaves the preceding velocity command
 * active for several milliseconds on the F413 and changes the physical path.
 */
#define F413_PATH_RUN_MAX_PREPARED_LINEAR_ACTIONS (256U)

typedef struct
{
  uint16_t path_index;
  uint8_t phase_count;
  bool execute_plan;
  float entry_velocity_mm_s;
  float exit_velocity_mm_s;
  float phase_distance_mm[NF_MOTION_LINEAR_MAX_PHASES];
  float phase_exit_velocity_mm_s[NF_MOTION_LINEAR_MAX_PHASES];
} f413_path_run_prepared_linear_t;

typedef struct
{
  f413_path_run_prepared_linear_t actions[F413_PATH_RUN_MAX_PREPARED_LINEAR_ACTIONS];
  size_t count;
} f413_path_run_prepared_path_t;

#if !defined(NIGHTFALL_F413_PATH_LINEAR_PLAN_HOST_TEST) || \
    defined(NIGHTFALL_F413_PATH_DISTANCE_CURSOR_HOST_TEST)
/*
 * The distance controller keeps the preceding velocity active while the main
 * loop changes from one path action to the next.  Starting every following
 * action from the then-current encoder position makes that transition travel
 * additive: several individually small gaps can move a complete path by more
 * than one centimetre.  Keep an absolute endpoint for consecutive straight
 * portions instead.  Any forward travel between calls therefore consumes the
 * following portion rather than extending the path.
 *
 * A timed turn core is intentionally an empirical primitive.  Its measured
 * end position becomes the next cursor origin; only the straight in/out and
 * path-code transition gaps are compensated here.
 */
typedef struct
{
  float endpoint_mm;
  bool active;
} f413_path_run_distance_cursor_t;

static void f413_path_run_distance_cursor_reset(
    f413_path_run_distance_cursor_t* cursor,
    float position_mm)
{
  if (cursor == NULL)
  {
    return;
  }
  cursor->endpoint_mm = position_mm;
  cursor->active = isfinite(position_mm);
}

static void f413_path_run_distance_cursor_invalidate(
    f413_path_run_distance_cursor_t* cursor)
{
  if (cursor != NULL)
  {
    cursor->endpoint_mm = 0.0f;
    cursor->active = false;
  }
}

static bool f413_path_run_distance_cursor_advance(
    f413_path_run_distance_cursor_t* cursor,
    float current_position_mm,
    float requested_distance_mm,
    float* target_position_mm,
    float* remaining_distance_mm)
{
  float remaining;

  if ((cursor == NULL) || !cursor->active ||
      !isfinite(cursor->endpoint_mm) ||
      !isfinite(current_position_mm) ||
      !isfinite(requested_distance_mm) ||
      (requested_distance_mm < 0.0f) ||
      (target_position_mm == NULL) || (remaining_distance_mm == NULL))
  {
    return false;
  }

  cursor->endpoint_mm += requested_distance_mm;
  remaining = cursor->endpoint_mm - current_position_mm;
  *target_position_mm = cursor->endpoint_mm;
  *remaining_distance_mm = (remaining > 0.0f) ? remaining : 0.0f;
  return isfinite(cursor->endpoint_mm) && isfinite(*remaining_distance_mm);
}
#endif

static bool f413_path_run_store_prepared_linear(
    f413_path_run_prepared_path_t* prepared,
    size_t path_index,
    float entry_velocity_mm_s,
    float exit_velocity_mm_s,
    const NfLinearPlan* plan,
    bool execute_plan)
{
  f413_path_run_prepared_linear_t* action;
  size_t phase_index;

  if (prepared == NULL)
  {
    return true;
  }
  if ((prepared->count >= F413_PATH_RUN_MAX_PREPARED_LINEAR_ACTIONS) ||
      (path_index > UINT16_MAX) || !isfinite(entry_velocity_mm_s) ||
      !isfinite(exit_velocity_mm_s) ||
      (execute_plan && ((plan == NULL) ||
                        (plan->phase_count > NF_MOTION_LINEAR_MAX_PHASES))))
  {
    return false;
  }

  action = &prepared->actions[prepared->count];
  action->path_index = (uint16_t)path_index;
  action->phase_count = execute_plan ? (uint8_t)plan->phase_count : 0U;
  action->execute_plan = execute_plan;
  action->entry_velocity_mm_s = entry_velocity_mm_s;
  action->exit_velocity_mm_s = exit_velocity_mm_s;
  for (phase_index = 0U; phase_index < NF_MOTION_LINEAR_MAX_PHASES; phase_index++)
  {
    action->phase_distance_mm[phase_index] = 0.0f;
    action->phase_exit_velocity_mm_s[phase_index] = 0.0f;
  }
  for (phase_index = 0U; phase_index < action->phase_count; phase_index++)
  {
    const double distance_mm = plan->phases[phase_index].distance_mm;
    const double phase_exit_velocity_mm_s =
        plan->phases[phase_index].exit_velocity_mm_s;
    if (!isfinite(distance_mm) || (distance_mm < 0.0) ||
        !isfinite(phase_exit_velocity_mm_s) ||
        (phase_exit_velocity_mm_s < 0.0))
    {
      return false;
    }
    action->phase_distance_mm[phase_index] = (float)distance_mm;
    action->phase_exit_velocity_mm_s[phase_index] =
        (float)phase_exit_velocity_mm_s;
  }
  prepared->count++;
  return true;
}

static bool f413_path_run_make_linear_plan(float distance_mm,
                                            float entry_velocity_mm_s,
                                            float exit_velocity_mm_s,
                                            const NfLinearLimits* limits,
                                            NfLinearPlan* out)
{
  if ((limits == NULL) || (out == NULL) ||
      !isfinite(distance_mm) || (distance_mm <= 0.0f) ||
      !isfinite(entry_velocity_mm_s) || (entry_velocity_mm_s < 0.0f) ||
      !isfinite(exit_velocity_mm_s) || (exit_velocity_mm_s < 0.0f))
  {
    return false;
  }

  return nf_motion_linear_plan(limits,
                               (double)distance_mm,
                               (double)entry_velocity_mm_s,
                               (double)exit_velocity_mm_s,
                               out) == NF_MOTION_OK;
}

static float f413_path_run_cap_positive(float value, float cap)
{
  if (value < 0.0f)
  {
    value = -value;
  }
  if ((cap > 0.0f) && (value > cap))
  {
    value = cap;
  }
  return value;
}

static bool f413_path_run_is_straight_code(uint16_t code)
{
  return (code > NF_LEGACY_PATH_STRAIGHT_BASE) &&
         (code <= NF_LEGACY_PATH_STRAIGHT_MAX);
}

static bool f413_path_run_is_diagonal_code(uint16_t code)
{
  return (code > NF_LEGACY_PATH_DIAGONAL_STRAIGHT_BASE) &&
         (code <= NF_LEGACY_PATH_DIAGONAL_STRAIGHT_MAX);
}

static bool f413_path_run_is_small_turn_code(uint16_t code)
{
  return (code == NF_LEGACY_PATH_SMALL_RIGHT_90) ||
         (code == NF_LEGACY_PATH_SMALL_LEFT_90);
}

static bool f413_path_run_is_large_turn_code(uint16_t code)
{
  return (code == NF_LEGACY_PATH_LARGE_RIGHT_90) ||
         (code == NF_LEGACY_PATH_LARGE_RIGHT_180) ||
         (code == NF_LEGACY_PATH_LARGE_LEFT_90) ||
         (code == NF_LEGACY_PATH_LARGE_LEFT_180);
}

static bool f413_path_run_turn_velocity_from_code(
    uint16_t code,
    const ShortestRunModeParams_t* params,
    float* out_velocity_mm_s)
{
  float velocity;

  if ((params == NULL) || (out_velocity_mm_s == NULL))
  {
    return false;
  }
  switch (code)
  {
    case NF_LEGACY_PATH_SMALL_RIGHT_90:
    case NF_LEGACY_PATH_SMALL_LEFT_90:
      velocity = params->velocity_turn90;
      break;
    case NF_LEGACY_PATH_LARGE_RIGHT_90:
    case NF_LEGACY_PATH_LARGE_LEFT_90:
      velocity = params->velocity_l_turn_90;
      break;
    case NF_LEGACY_PATH_LARGE_RIGHT_180:
    case NF_LEGACY_PATH_LARGE_LEFT_180:
      velocity = params->velocity_l_turn_180;
      break;
    case NF_LEGACY_PATH_RIGHT_45_IN:
    case NF_LEGACY_PATH_LEFT_45_IN:
      velocity = params->velocity_turn45in;
      break;
    case NF_LEGACY_PATH_RIGHT_45_OUT:
    case NF_LEGACY_PATH_LEFT_45_OUT:
      velocity = params->velocity_turn45out;
      break;
    case NF_LEGACY_PATH_RIGHT_V90:
    case NF_LEGACY_PATH_LEFT_V90:
      velocity = params->velocity_turnV90;
      break;
    case NF_LEGACY_PATH_RIGHT_135_IN:
    case NF_LEGACY_PATH_LEFT_135_IN:
      velocity = params->velocity_turn135in;
      break;
    case NF_LEGACY_PATH_RIGHT_135_OUT:
    case NF_LEGACY_PATH_LEFT_135_OUT:
      velocity = params->velocity_turn135out;
      break;
    default:
      return false;
  }
  *out_velocity_mm_s = f413_path_run_cap_positive(
      velocity, NIGHTFALL_F413_PATH_TURN_VELOCITY_CAP);
  return isfinite(*out_velocity_mm_s) && (*out_velocity_mm_s > 0.0f);
}

static bool f413_path_run_straight_limits(
    const ShortestRunModeParams_t* mode_params,
    const ShortestRunCaseParams_t* case_params,
    NfLinearLimits* out)
{
  if ((mode_params == NULL) || (case_params == NULL) || (out == NULL))
  {
    return false;
  }

  out->vmax_mm_s = f413_path_run_cap_positive(
      case_params->velocity_straight, NIGHTFALL_F413_PATH_VELOCITY_CAP);
  out->switch_velocity_mm_s = mode_params->accel_switch_velocity;
  out->accel_low_mm_s2 = case_params->acceleration_straight;
  out->accel_high_mm_s2 = case_params->acceleration_straight_dash;
  return (out->vmax_mm_s > 0.0) && (out->accel_low_mm_s2 > 0.0) &&
         (out->accel_high_mm_s2 > 0.0);
}

static bool f413_path_run_diagonal_limits(
    const ShortestRunCaseParams_t* case_params,
    NfLinearLimits* out)
{
  if ((case_params == NULL) || (out == NULL))
  {
    return false;
  }

  out->vmax_mm_s = f413_path_run_cap_positive(
      case_params->velocity_d_straight,
      NIGHTFALL_F413_PATH_DIAGONAL_VELOCITY_CAP);
  out->switch_velocity_mm_s = 0.0;
  out->accel_low_mm_s2 = case_params->acceleration_d_straight;
  out->accel_high_mm_s2 = case_params->acceleration_d_straight_dash;
  return (out->vmax_mm_s > 0.0) && (out->accel_high_mm_s2 > 0.0);
}

static float f413_path_run_goal_entry_speed(
    const ShortestRunCaseParams_t* case_params)
{
  float speed;

  if (case_params == NULL)
  {
    return 0.0f;
  }
  speed = sqrtf(fmaxf(
      0.0f,
      2.0f * case_params->acceleration_straight * (float)DIST_HALF_SEC));
  if (speed > f413_path_run_cap_positive(
                  case_params->velocity_straight,
                  NIGHTFALL_F413_PATH_VELOCITY_CAP))
  {
    speed = f413_path_run_cap_positive(
        case_params->velocity_straight,
        NIGHTFALL_F413_PATH_VELOCITY_CAP);
  }
  return speed;
}

static float f413_path_run_next_straight_exit_velocity(
    uint16_t next_code,
    const ShortestRunModeParams_t* mode_params,
    const ShortestRunCaseParams_t* case_params)
{
  float turn_velocity;

  if (f413_path_run_turn_velocity_from_code(
          next_code, mode_params, &turn_velocity))
  {
    return turn_velocity;
  }
  if (next_code == 0U)
  {
    return f413_path_run_goal_entry_speed(case_params);
  }
  if (f413_path_run_is_straight_code(next_code) && (case_params != NULL))
  {
    return f413_path_run_cap_positive(
        case_params->velocity_straight,
        NIGHTFALL_F413_PATH_VELOCITY_CAP);
  }
  return 0.0f;
}

static float f413_path_run_next_diagonal_exit_velocity(
    uint16_t next_code,
    const ShortestRunModeParams_t* mode_params,
    const ShortestRunCaseParams_t* case_params,
    float test_terminal_velocity_mm_s)
{
  float turn_velocity;

  if (f413_path_run_turn_velocity_from_code(
          next_code, mode_params, &turn_velocity))
  {
    return turn_velocity;
  }
  if (f413_path_run_is_diagonal_code(next_code) && (case_params != NULL))
  {
    return f413_path_run_cap_positive(
        case_params->velocity_d_straight,
        NIGHTFALL_F413_PATH_DIAGONAL_VELOCITY_CAP);
  }
  if ((next_code == 0U) && isfinite(test_terminal_velocity_mm_s) &&
      (test_terminal_velocity_mm_s > 0.0f))
  {
    /*
     * A case0 primitive test may intentionally finish on a diagonal heading.
     * Keep the tuned turn velocity through its explicit DS tail; the common
     * implicit 45 mm tail below then performs the stop.  Normal shortest paths
     * pass zero here and retain the cardinal-terminal invariant.
     */
    return test_terminal_velocity_mm_s;
  }
  return 0.0f;
}

static bool f413_path_run_make_test_terminal_profile(
    float entry_velocity_mm_s,
    const NfLinearLimits* limits,
    NfConstantAccelProfile* out)
{
  NfLinearLimits terminal_limits;
  double acceleration_limit;

  if ((limits == NULL) || (out == NULL))
  {
    return false;
  }
  terminal_limits = *limits;
  acceleration_limit = fmax(limits->accel_low_mm_s2,
                            limits->accel_high_mm_s2);
  if (!isfinite(acceleration_limit) || (acceleration_limit <= 0.0))
  {
    return false;
  }

  /*
   * The legacy case0 runner commands one monotonic velocity profile over its
   * final half cell.  Bound that direct controller command by the larger of
   * the selected case's two declared acceleration limits; do not apply this
   * relaxed terminal rule to solver-generated shortest paths.
   */
  terminal_limits.switch_velocity_mm_s = 0.0;
  terminal_limits.accel_low_mm_s2 = acceleration_limit;
  terminal_limits.accel_high_mm_s2 = acceleration_limit;
  return nf_motion_constant_accel_profile(
             &terminal_limits,
             (double)DIST_HALF_SEC,
             (double)entry_velocity_mm_s,
             0.0,
             out) == NF_MOTION_OK;
}

static bool f413_path_run_make_wall_end_approach_plan(
    uint32_t straight_steps,
    float entry_velocity_mm_s,
    float turn_velocity_mm_s,
    const NfLinearLimits* limits,
    NfLinearPlan* out_plan,
    bool* out_execute_before_buffer)
{
  if ((straight_steps == 0U) || (out_plan == NULL) ||
      (out_execute_before_buffer == NULL))
  {
    return false;
  }

  /*
   * S1 is wholly owned by the wall-end segment, which performs the entry to
   * turn-speed profile.  S2+ reaches turn speed before reserving the final S1
   * for wall-end detection at constant speed.
   */
  *out_execute_before_buffer = straight_steps > 1U;
  if (!*out_execute_before_buffer)
  {
    NfConstantAccelProfile buffer_profile;
    return nf_motion_constant_accel_profile(
               limits,
               (double)DIST_HALF_SEC,
               (double)entry_velocity_mm_s,
               (double)turn_velocity_mm_s,
               &buffer_profile) == NF_MOTION_OK;
  }
  return f413_path_run_make_linear_plan(
      (float)(straight_steps - 1U) * (float)DIST_HALF_SEC,
      entry_velocity_mm_s,
      turn_velocity_mm_s,
      limits,
      out_plan);
}

static bool f413_path_run_turn_enters_diagonal(uint16_t code)
{
  return (code == NF_LEGACY_PATH_RIGHT_45_IN) ||
         (code == NF_LEGACY_PATH_LEFT_45_IN) ||
         (code == NF_LEGACY_PATH_RIGHT_135_IN) ||
         (code == NF_LEGACY_PATH_LEFT_135_IN);
}

static bool f413_path_run_turn_exits_diagonal(uint16_t code)
{
  return (code == NF_LEGACY_PATH_RIGHT_45_OUT) ||
         (code == NF_LEGACY_PATH_LEFT_45_OUT) ||
         (code == NF_LEGACY_PATH_RIGHT_135_OUT) ||
         (code == NF_LEGACY_PATH_LEFT_135_OUT);
}

static f413_path_run_preflight_result_t f413_path_run_preflight_prepare(
    const uint16_t* codes,
    size_t capacity,
    const ShortestRunModeParams_t* mode_params,
    const ShortestRunCaseParams_t* case_params,
    float initial_velocity_mm_s,
    bool wall_end_correction_enabled,
    bool test_mode_run,
    f413_path_run_prepared_path_t* prepared)
{
  f413_path_run_preflight_result_t result = {
      F413_PATH_RUN_PREFLIGHT_OK, NF_LEGACY_PATH_OK, 0U, 0U};
  NfLegacyPathResult legacy;
  NfLinearLimits straight_limits;
  NfLinearLimits diagonal_limits;
  float speed_now = initial_velocity_mm_s;
  bool diagonal = false;
  size_t index = 0U;

  if (prepared != NULL)
  {
    prepared->count = 0U;
  }

  if ((codes == NULL) || (capacity == 0U) || (mode_params == NULL) ||
      (case_params == NULL) || !isfinite(initial_velocity_mm_s) ||
      (initial_velocity_mm_s < 0.0f) ||
      !f413_path_run_straight_limits(
          mode_params, case_params, &straight_limits))
  {
    result.status = F413_PATH_RUN_PREFLIGHT_INVALID_ARGUMENT;
    return result;
  }
  if (codes[0] == 0U)
  {
    result.status = F413_PATH_RUN_PREFLIGHT_INVALID_LEGACY_PATH;
    result.code = 0U;
    return result;
  }

  legacy = nf_legacy_path_validate(codes, capacity);
  if ((legacy.status != NF_LEGACY_PATH_OK) &&
      !(test_mode_run &&
        (legacy.status == NF_LEGACY_PATH_ENDS_DIAGONAL)))
  {
    result.status = F413_PATH_RUN_PREFLIGHT_INVALID_LEGACY_PATH;
    result.legacy_status = legacy.status;
    result.index = legacy.index;
    result.code = legacy.code;
    return result;
  }

  while (codes[index] != 0U)
  {
    const uint16_t code = codes[index];

    if (f413_path_run_is_straight_code(code))
    {
      uint32_t steps = 0U;
      size_t end = index;
      const uint16_t previous_code = (index > 0U) ? codes[index - 1U] : 0U;
      uint16_t next_code;
      float exit_velocity;
      NfLinearPlan plan;
      bool skip_wall_end;
      bool execute_plan = true;

      while (f413_path_run_is_straight_code(codes[end]))
      {
        steps += (uint32_t)(codes[end] - NF_LEGACY_PATH_STRAIGHT_BASE);
        end++;
      }
      next_code = codes[end];
      exit_velocity = f413_path_run_next_straight_exit_velocity(
          next_code, mode_params, case_params);
      skip_wall_end =
          (steps == 1U) &&
          ((f413_path_run_is_large_turn_code(previous_code) &&
            f413_path_run_is_small_turn_code(next_code)) ||
           (f413_path_run_is_small_turn_code(previous_code) &&
            f413_path_run_is_large_turn_code(next_code)));
      if (wall_end_correction_enabled && !skip_wall_end &&
          (f413_path_run_is_small_turn_code(next_code) ||
           f413_path_run_is_large_turn_code(next_code)))
      {
        bool execute_before_buffer;
        if (!f413_path_run_make_wall_end_approach_plan(
                steps, speed_now, exit_velocity, &straight_limits, &plan,
                &execute_before_buffer))
        {
          result.status = F413_PATH_RUN_PREFLIGHT_INFEASIBLE_LINEAR;
        }
        execute_plan = execute_before_buffer;
      }
      else if (!f413_path_run_make_linear_plan(
                   (float)steps * (float)DIST_HALF_SEC,
                   speed_now, exit_velocity, &straight_limits, &plan))
      {
        result.status = F413_PATH_RUN_PREFLIGHT_INFEASIBLE_LINEAR;
      }
      if (result.status != F413_PATH_RUN_PREFLIGHT_OK)
      {
        result.index = index;
        result.code = code;
        return result;
      }
      if (!f413_path_run_store_prepared_linear(
              prepared, index, speed_now, exit_velocity,
              execute_plan ? &plan : NULL, execute_plan))
      {
        result.status = F413_PATH_RUN_PREFLIGHT_PREPARED_CAPACITY;
        result.index = index;
        result.code = code;
        return result;
      }
      speed_now = exit_velocity;
      index = end;
      continue;
    }

    if (f413_path_run_is_diagonal_code(code))
    {
      uint32_t steps = 0U;
      size_t end = index;
      float exit_velocity;
      NfLinearPlan plan;

      if (!f413_path_run_diagonal_limits(case_params, &diagonal_limits))
      {
        result.status = F413_PATH_RUN_PREFLIGHT_INVALID_ARGUMENT;
        result.index = index;
        result.code = code;
        return result;
      }
      while (f413_path_run_is_diagonal_code(codes[end]))
      {
        steps +=
            (uint32_t)(codes[end] - NF_LEGACY_PATH_DIAGONAL_STRAIGHT_BASE);
        end++;
      }
      exit_velocity = f413_path_run_next_diagonal_exit_velocity(
          codes[end], mode_params, case_params,
          (test_mode_run && (codes[end] == 0U)) ? speed_now : 0.0f);
      if (!f413_path_run_make_linear_plan(
              (float)steps * (float)DIST_D_HALF_SEC,
              speed_now, exit_velocity, &diagonal_limits, &plan))
      {
        result.status = F413_PATH_RUN_PREFLIGHT_INFEASIBLE_LINEAR;
        result.index = index;
        result.code = code;
        return result;
      }
      if (!f413_path_run_store_prepared_linear(
              prepared, index, speed_now, exit_velocity, &plan, true))
      {
        result.status = F413_PATH_RUN_PREFLIGHT_PREPARED_CAPACITY;
        result.index = index;
        result.code = code;
        return result;
      }
      speed_now = exit_velocity;
      index = end;
      continue;
    }

    {
      float turn_velocity;
      if (!f413_path_run_turn_velocity_from_code(
              code, mode_params, &turn_velocity))
      {
        result.status = F413_PATH_RUN_PREFLIGHT_INVALID_LEGACY_PATH;
        result.index = index;
        result.code = code;
        return result;
      }
      if (fabsf(speed_now - turn_velocity) > 0.5f)
      {
        result.status = F413_PATH_RUN_PREFLIGHT_SPEED_DISCONTINUITY;
        result.index = index;
        result.code = code;
        return result;
      }
      speed_now = turn_velocity;
      if (f413_path_run_turn_enters_diagonal(code))
      {
        diagonal = true;
      }
      else if (f413_path_run_turn_exits_diagonal(code))
      {
        diagonal = false;
      }
      index++;
    }
  }

  if (diagonal && !test_mode_run)
  {
    /* Normally caught by the shared grammar; keep the execution invariant. */
    result.status = F413_PATH_RUN_PREFLIGHT_INVALID_LEGACY_PATH;
    result.legacy_status = NF_LEGACY_PATH_ENDS_DIAGONAL;
    result.index = index;
    return result;
  }
  {
    NfConstantAccelProfile stop_profile;
    const bool valid_stop = test_mode_run
        ? f413_path_run_make_test_terminal_profile(
              speed_now, &straight_limits, &stop_profile)
        : (nf_motion_constant_accel_profile(
               &straight_limits, (double)DIST_HALF_SEC, (double)speed_now,
               0.0, &stop_profile) == NF_MOTION_OK);
    if (!valid_stop)
    {
      result.status = F413_PATH_RUN_PREFLIGHT_INFEASIBLE_LINEAR;
      result.index = index;
      result.code = 0U;
      return result;
    }
  }
  return result;
}

#if defined(NIGHTFALL_F413_PATH_LINEAR_PLAN_HOST_TEST)
static f413_path_run_preflight_result_t f413_path_run_preflight(
    const uint16_t* codes,
    size_t capacity,
    const ShortestRunModeParams_t* mode_params,
    const ShortestRunCaseParams_t* case_params,
    float initial_velocity_mm_s,
    bool wall_end_correction_enabled,
    bool test_mode_run)
{
  return f413_path_run_preflight_prepare(
      codes, capacity, mode_params, case_params, initial_velocity_mm_s,
      wall_end_correction_enabled, test_mode_run, NULL);
}
#endif

#if !defined(NIGHTFALL_F413_PATH_LINEAR_PLAN_HOST_TEST)

#include "f413_control.h"
#include "f413_hw.h"
#include "f413_run_features.h"
#include "f413_run_session.h"
#include "f413_trace_flags.h"
#include "f413_trace_log.h"
#include "f413_trace_sample.h"
#include "f413_wall_distance.h"
#include "f413_wall_runtime.h"
#include "main.h"
#include "params.h"
#include "search.h"
#include "shortest_run_params.h"
#include "trace.h"

typedef struct {
  float signed_angle_deg;
  float alpha_deg_s2;
  float velocity_mm_s;
  float dist_in_mm;
  float dist_out_mm;
  bool front_wall_entry;
  bool wall_control_offsets;
  bool large_turn;
} f413_path_run_turn_t;

typedef struct {
  float omega_peak_deg_s;
  float t_acc_s;
  float t_cruise_s;
  float t_total_s;
} f413_path_run_smooth_turn_t;

extern uint16_t path[];

static f413_path_run_prepared_path_t g_f413_path_run_prepared_path;
static f413_path_run_distance_cursor_t g_f413_path_run_distance_cursor;

#if (NIGHTFALL_F413_PATH_MAX_CODES > ROUTE_MAX_LEN)
#error "F413 path execution limit exceeds path[] capacity"
#endif

static const ShortestRunModeParams_t* f413_path_run_mode_params(uint8_t mode)
{
  switch (mode)
  {
    case 2U: return &shortestRunModeParams2;
    case 3U: return &shortestRunModeParams3;
    case 4U: return &shortestRunModeParams4;
    case 5U: return &shortestRunModeParams5;
    case 6U: return &shortestRunModeParams6;
    case 7U: return &shortestRunModeParams7;
    default: return &shortestRunModeParams2;
  }
}

static const ShortestRunCaseParams_t* f413_path_run_case_params(uint8_t mode, uint8_t case_index)
{
  uint8_t idx = 0U;

  if ((case_index >= 1U) && (case_index <= 9U))
  {
    idx = (uint8_t)(case_index - 1U);
  }

  switch (mode)
  {
    case 2U: return &shortestRunCaseParamsMode2[idx];
    case 3U: return &shortestRunCaseParamsMode3[idx];
    case 4U: return &shortestRunCaseParamsMode4[idx];
    case 5U: return &shortestRunCaseParamsMode5[idx];
    case 6U: return &shortestRunCaseParamsMode6[idx];
    case 7U: return &shortestRunCaseParamsMode7[idx];
    default: return &shortestRunCaseParamsMode2[idx];
  }
}

static float f413_path_run_velocity_or_cap(float candidate, float fallback, float cap)
{
  float v = candidate;

  if (v <= 0.0f)
  {
    v = fallback;
  }
  if (v <= 0.0f)
  {
    v = NIGHTFALL_F413_PATH_VELOCITY;
  }
  if ((cap > 0.0f) && (v > cap))
  {
    v = cap;
  }

  return v;
}

static uint16_t f413_path_run_motor_phase_flags(uint16_t trace_flags, uint16_t motor_flag)
{
  const uint16_t motor_mask = (uint16_t)(NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG |
                                        NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG |
                                        NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG);
  return (uint16_t)((trace_flags & (uint16_t)~motor_mask) | motor_flag);
}

static void f413_path_run_prepare_straight_angle_control(void)
{
  if (!f413_run_features_angle_accum_mode())
  {
    f413_ctrl_reset_angle();
  }
  f413_ctrl_clear_angle_target();
}

static void f413_path_run_prepare_turn_angle_control(void)
{
  if (!f413_run_features_angle_accum_mode())
  {
    f413_ctrl_reset_angle();
  }
  f413_ctrl_clear_angle_target();
  f413_wall_runtime_control_clear();
}

static bool f413_path_run_turn_from_code(uint16_t code,
                                         const ShortestRunModeParams_t* params,
                                         f413_path_run_turn_t* turn)
{
  float angle = 0.0f;
  float accum_angle = 0.0f;
  float alpha = 0.0f;
  float velocity;
  bool right = false;

  if ((params == NULL) || (turn == NULL))
  {
    return false;
  }
  turn->front_wall_entry = false;
  turn->wall_control_offsets = false;
  turn->large_turn = false;

  switch (code)
  {
    case 300U:
      angle = params->angle_turn_90; alpha = params->alpha_turn90;
      accum_angle = 90.0f;
      turn->dist_in_mm = params->dist_offset_in;
      turn->dist_out_mm = params->dist_offset_out;
      turn->front_wall_entry = true;
      turn->wall_control_offsets = true;
      right = true; break;
    case 400U:
      angle = params->angle_turn_90; alpha = params->alpha_turn90;
      accum_angle = 90.0f;
      turn->dist_in_mm = params->dist_offset_in;
      turn->dist_out_mm = params->dist_offset_out;
      turn->front_wall_entry = true;
      turn->wall_control_offsets = true;
      right = false; break;
    case 501U:
      angle = params->angle_l_turn_90; alpha = params->alpha_l_turn_90;
      accum_angle = 90.0f;
      turn->dist_in_mm = params->dist_l_turn_in_90;
      turn->dist_out_mm = params->dist_l_turn_out_90;
      turn->front_wall_entry = false;
      turn->wall_control_offsets = true;
      turn->large_turn = true;
      right = true; break;
    case 601U:
      angle = params->angle_l_turn_90; alpha = params->alpha_l_turn_90;
      accum_angle = 90.0f;
      turn->dist_in_mm = params->dist_l_turn_in_90;
      turn->dist_out_mm = params->dist_l_turn_out_90;
      turn->front_wall_entry = false;
      turn->wall_control_offsets = true;
      turn->large_turn = true;
      right = false; break;
    case 502U:
      angle = params->angle_l_turn_180; alpha = params->alpha_l_turn_180;
      accum_angle = 180.0f;
      turn->dist_in_mm = params->dist_l_turn_in_180;
      turn->dist_out_mm = params->dist_l_turn_out_180;
      turn->front_wall_entry = false;
      turn->wall_control_offsets = true;
      turn->large_turn = true;
      right = true; break;
    case 602U:
      angle = params->angle_l_turn_180; alpha = params->alpha_l_turn_180;
      accum_angle = 180.0f;
      turn->dist_in_mm = params->dist_l_turn_in_180;
      turn->dist_out_mm = params->dist_l_turn_out_180;
      turn->front_wall_entry = false;
      turn->wall_control_offsets = true;
      turn->large_turn = true;
      right = false; break;
    case 701U:
      angle = params->angle_turn45in; alpha = params->alpha_turn45in;
      accum_angle = 45.0f;
      turn->dist_in_mm = params->dist_turn45in_in;
      turn->dist_out_mm = params->dist_turn45in_out;
      turn->front_wall_entry = false;
      right = true; break;
    case 702U:
      angle = params->angle_turn45in; alpha = params->alpha_turn45in;
      accum_angle = 45.0f;
      turn->dist_in_mm = params->dist_turn45in_in;
      turn->dist_out_mm = params->dist_turn45in_out;
      turn->front_wall_entry = false;
      right = false; break;
    case 703U:
      angle = params->angle_turn45out; alpha = params->alpha_turn45out;
      accum_angle = 45.0f;
      turn->dist_in_mm = params->dist_turn45out_in;
      turn->dist_out_mm = params->dist_turn45out_out;
      turn->front_wall_entry = false;
      right = true; break;
    case 704U:
      angle = params->angle_turn45out; alpha = params->alpha_turn45out;
      accum_angle = 45.0f;
      turn->dist_in_mm = params->dist_turn45out_in;
      turn->dist_out_mm = params->dist_turn45out_out;
      turn->front_wall_entry = false;
      right = false; break;
    case 801U:
      angle = params->angle_turnV90; alpha = params->alpha_turnV90;
      accum_angle = 90.0f;
      turn->dist_in_mm = params->dist_turnV90_in;
      turn->dist_out_mm = params->dist_turnV90_out;
      turn->front_wall_entry = false;
      right = true; break;
    case 802U:
      angle = params->angle_turnV90; alpha = params->alpha_turnV90;
      accum_angle = 90.0f;
      turn->dist_in_mm = params->dist_turnV90_in;
      turn->dist_out_mm = params->dist_turnV90_out;
      turn->front_wall_entry = false;
      right = false; break;
    case 901U:
      angle = params->angle_turn135in; alpha = params->alpha_turn135in;
      accum_angle = 135.0f;
      turn->dist_in_mm = params->dist_turn135in_in;
      turn->dist_out_mm = params->dist_turn135in_out;
      turn->front_wall_entry = false;
      right = true; break;
    case 902U:
      angle = params->angle_turn135in; alpha = params->alpha_turn135in;
      accum_angle = 135.0f;
      turn->dist_in_mm = params->dist_turn135in_in;
      turn->dist_out_mm = params->dist_turn135in_out;
      turn->front_wall_entry = false;
      right = false; break;
    case 903U:
      angle = params->angle_turn135out; alpha = params->alpha_turn135out;
      accum_angle = 135.0f;
      turn->dist_in_mm = params->dist_turn135out_in;
      turn->dist_out_mm = params->dist_turn135out_out;
      turn->front_wall_entry = false;
      right = true; break;
    case 904U:
      angle = params->angle_turn135out; alpha = params->alpha_turn135out;
      accum_angle = 135.0f;
      turn->dist_in_mm = params->dist_turn135out_in;
      turn->dist_out_mm = params->dist_turn135out_out;
      turn->front_wall_entry = false;
      right = false; break;
    default: return false;
  }

  if (angle <= 0.0f)
  {
    angle = 90.0f;
  }
  if (f413_run_features_angle_accum_mode() && (accum_angle > 0.0f))
  {
    angle = accum_angle;
  }
  if (alpha <= 0.0f)
  {
    alpha = 10000.0f;
  }

  if (!f413_path_run_turn_velocity_from_code(code, params, &velocity))
  {
    return false;
  }
  turn->signed_angle_deg = right ? -fabsf(angle) : fabsf(angle);
  turn->alpha_deg_s2 = alpha;
  turn->velocity_mm_s = velocity;
  return true;
}

static f413_path_run_smooth_turn_t f413_path_run_build_smooth_turn(float angle_deg,
                                                                   float alpha_deg_s2,
                                                                   float omega_cap_deg_s)
{
  f413_path_run_smooth_turn_t profile = {0.0f, 0.0f, 0.0f, 0.0f};
  const float angle_abs = fabsf(angle_deg);
  float rounding_scale = TURN_OMEGA_PROFILE_ROUNDING_SCALE;

  if ((angle_abs <= 0.0f) || (alpha_deg_s2 <= 0.0f))
  {
    return profile;
  }

  profile.omega_peak_deg_s = sqrtf((2.0f * alpha_deg_s2 * angle_abs) / 3.0f);
  profile.omega_peak_deg_s = f413_path_run_cap_positive(profile.omega_peak_deg_s, omega_cap_deg_s);
  if (profile.omega_peak_deg_s <= 0.0f)
  {
    return profile;
  }

  if (rounding_scale < 0.1f)
  {
    rounding_scale = 0.1f;
  }

  profile.t_acc_s = (profile.omega_peak_deg_s / alpha_deg_s2) * rounding_scale;
  profile.t_cruise_s = (angle_abs / profile.omega_peak_deg_s) - profile.t_acc_s;
  if (profile.t_cruise_s < 0.0f)
  {
    profile.t_cruise_s = 0.0f;
    profile.omega_peak_deg_s = angle_abs / profile.t_acc_s;
    profile.omega_peak_deg_s = f413_path_run_cap_positive(profile.omega_peak_deg_s, omega_cap_deg_s);
    profile.t_cruise_s = (angle_abs / profile.omega_peak_deg_s) - profile.t_acc_s;
    if (profile.t_cruise_s < 0.0f)
    {
      profile.t_cruise_s = 0.0f;
    }
  }
  profile.t_total_s = (2.0f * profile.t_acc_s) + profile.t_cruise_s;
  return profile;
}

static bool f413_path_run_print_turn_profiles_before_run(
    const ShortestRunModeParams_t* mode_params)
{
  uint16_t index;

  for (index = 0U; index < NIGHTFALL_F413_PATH_MAX_CODES; index++)
  {
    const uint16_t code = path[index];
    f413_path_run_turn_t turn;
    f413_path_run_smooth_turn_t profile;

    if (code == 0U)
    {
      return true;
    }
    if (f413_path_run_is_straight_code(code) ||
        f413_path_run_is_diagonal_code(code))
    {
      continue;
    }
    if (!f413_path_run_turn_from_code(code, mode_params, &turn))
    {
      return false;
    }
    profile = f413_path_run_build_smooth_turn(turn.signed_angle_deg,
                                              turn.alpha_deg_s2,
                                              NIGHTFALL_F413_PATH_OMEGA_CAP);
    if (profile.t_total_s <= 0.0f)
    {
      return false;
    }
    /*
     * Float formatting is intentionally completed while motor standby is
     * still off.  Printing this line at the live straight-to-turn boundary
     * previously extended the approach distance at the old velocity.
     */
    trace_printf(
        "[OP-UI][PATH-TEST] turn[%u] code=%u angle=%.1f alpha=%.0f omega=%.0f v=%.0f t=%.0fms\r\n",
        (unsigned int)index,
        (unsigned int)code,
        (double)turn.signed_angle_deg,
        (double)turn.alpha_deg_s2,
        (double)profile.omega_peak_deg_s,
        (double)turn.velocity_mm_s,
        (double)(profile.t_total_s * 1000.0f));
  }
  return false;
}

static void f413_path_run_trace_on_run_start(void)
{
  /* Keep the driver in standby for the complete optical START token. */
  f413_ctrl_stop();
  trace_printf("[RUN-START] control stopped, motor standby during optical START\r\n");
  trace_printf("[VIDEO-SYNC] optical START fixed-slot SHORT token\r\n");
  f413_hw_emit_video_sync_start_pattern();
  HAL_Delay(F413_HW_VIDEO_SYNC_START_GUARD_MS);
  trace_printf("[TRACE-LOG] run-hook: start\r\n");
  f413_trace_log_auto_start();
}

static void f413_path_run_trace_on_run_stop(void)
{
  trace_printf("[TRACE-LOG] run-hook: stop tail=%u ms\r\n",
               (unsigned int)F413_TRACE_LOG_STOP_TAIL_MS_DEFAULT);
  f413_trace_log_auto_stop_after_tail(F413_TRACE_LOG_STOP_TAIL_MS_DEFAULT);
  trace_printf("[VIDEO-SYNC] optical STOP fixed-slot LONG token\r\n");
  f413_hw_emit_video_sync_stop_pattern();
}

static f413_run_session_abort_reason_t f413_path_run_wait_ctrl_target(
    float target, bool is_angle,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags,
    bool wall_control_gate,
    bool diagonal_control_gate)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  uint32_t deadline = HAL_GetTick() + NIGHTFALL_F413_PATH_TIMEOUT_MS;

  while (1)
  {
    if (is_angle)
    {
      float current = f413_ctrl_get_angle();
      if (((target >= 0.0f) && (current >= target)) ||
          ((target < 0.0f) && (current <= target)))
      {
        break;
      }
    }
    else
    {
      float current = fabsf(f413_ctrl_get_distance());
      if (current >= fabsf(target))
      {
        break;
      }
    }

    if ((int32_t)(HAL_GetTick() - deadline) >= 0)
    {
      return F413_RUN_SESSION_ABORT_TIMEOUT;
    }

    f413_trace_log_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (!is_angle && diagonal_control_gate)
    {
      f413_wall_runtime_poll_diagonal(true);
    }
    else if (!is_angle && wall_control_gate &&
             ((trace_flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG) != 0U))
    {
      /* Keep shortest-run wall control on the same live update path as search. */
      f413_wall_runtime_poll_straight(true);
    }
    else
    {
      f413_wall_runtime_control_apply(false);
    }
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
  }
  return F413_RUN_SESSION_ABORT_NONE;
}

#define F413_PATH_RUN_TEST_STOP_SETTLE_MAX_MS       (250U)
#define F413_PATH_RUN_TEST_STOP_VELOCITY_MM_S       (20.0f)
#define F413_PATH_RUN_TEST_STOP_POSITION_ERROR_MM   (0.75f)

static f413_run_session_abort_reason_t f413_path_run_settle_test_stop(
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  const uint32_t deadline =
      HAL_GetTick() + F413_PATH_RUN_TEST_STOP_SETTLE_MAX_MS;

  while (1)
  {
    const float velocity_mm_s = f413_ctrl_get_real_velocity();
    const float position_error_mm =
        g_f413_path_run_distance_cursor.endpoint_mm - f413_ctrl_get_distance();
    f413_run_session_abort_reason_t reason;

    if (!g_f413_path_run_distance_cursor.active ||
        !isfinite(position_error_mm))
    {
      return F413_RUN_SESSION_ABORT_IMU_FAULT;
    }
    if ((fabsf(velocity_mm_s) <= F413_PATH_RUN_TEST_STOP_VELOCITY_MM_S) &&
        (fabsf(position_error_mm) <=
         F413_PATH_RUN_TEST_STOP_POSITION_ERROR_MM))
    {
      return F413_RUN_SESSION_ABORT_NONE;
    }
    if ((int32_t)(HAL_GetTick() - deadline) >= 0)
    {
      return F413_RUN_SESSION_ABORT_TIMEOUT;
    }
    f413_trace_log_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    f413_wall_runtime_control_apply(false);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
  }
}

static f413_run_session_abort_reason_t f413_path_run_drive_segment_ex(float distance_mm,
                                                                      float target_velocity_mm_s,
                                                                      float* speed_now_mm_s,
                                                                      f413_run_session_guard_t* guard,
                                                                      uint16_t trace_flags,
                                                                      bool wall_control_gate,
                                                                      bool diagonal_control_gate)
{
  float start_velocity;
  float profile_distance;
  float target_distance;
  f413_run_session_abort_reason_t reason;

  if (speed_now_mm_s == NULL)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
  if (distance_mm <= 0.0f)
  {
    *speed_now_mm_s = target_velocity_mm_s;
    return F413_RUN_SESSION_ABORT_NONE;
  }

  start_velocity = *speed_now_mm_s;
  if (!f413_path_run_distance_cursor_advance(
          &g_f413_path_run_distance_cursor,
          f413_ctrl_get_distance(),
          distance_mm,
          &target_distance,
          &profile_distance))
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
  if (!wall_control_gate)
  {
    f413_wall_runtime_control_clear();
  }
  f413_path_run_prepare_straight_angle_control();
  if (profile_distance <= 0.001f)
  {
    f413_ctrl_set_velocity(target_velocity_mm_s);
    f413_ctrl_set_omega(0.0f);
    *speed_now_mm_s = target_velocity_mm_s;
    return F413_RUN_SESSION_ABORT_NONE;
  }
  f413_ctrl_set_velocity_profile(start_velocity,
                                 target_velocity_mm_s,
                                 profile_distance);
  f413_ctrl_set_omega(0.0f);

  reason = f413_path_run_wait_ctrl_target(target_distance, false, guard, trace_flags,
                                          wall_control_gate, diagonal_control_gate);
  *speed_now_mm_s = target_velocity_mm_s;
  return reason;
}

static f413_run_session_abort_reason_t f413_path_run_drive_segment(float distance_mm,
                                                                   float target_velocity_mm_s,
                                                                   float* speed_now_mm_s,
                                                                   f413_run_session_guard_t* guard,
                                                                   uint16_t trace_flags)
{
  return f413_path_run_drive_segment_ex(distance_mm,
                                        target_velocity_mm_s,
                                        speed_now_mm_s,
                                        guard,
                                        trace_flags,
                                        true,
                                        false);
}

static f413_run_session_abort_reason_t f413_path_run_drive_segment_no_wall(float distance_mm,
                                                                           float target_velocity_mm_s,
                                                                           float* speed_now_mm_s,
                                                                           f413_run_session_guard_t* guard,
                                                                           uint16_t trace_flags)
{
  return f413_path_run_drive_segment_ex(distance_mm,
                                        target_velocity_mm_s,
                                        speed_now_mm_s,
                                        guard,
                                        trace_flags,
                                        false,
                                        false);
}

static f413_run_session_abort_reason_t f413_path_run_execute_prepared_linear(
    const f413_path_run_prepared_linear_t* action,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags,
    bool wall_control_gate,
    bool diagonal_control_gate)
{
  size_t phase_index;

  if ((action == NULL) || !action->execute_plan ||
      (speed_now_mm_s == NULL) ||
      !isfinite(action->entry_velocity_mm_s) ||
      !isfinite(action->exit_velocity_mm_s) ||
      (action->phase_count > NF_MOTION_LINEAR_MAX_PHASES) ||
      (fabsf(*speed_now_mm_s - action->entry_velocity_mm_s) > 0.5f))
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  for (phase_index = 0U; phase_index < action->phase_count; phase_index++)
  {
    const float phase_distance_mm = action->phase_distance_mm[phase_index];
    const float phase_exit_velocity_mm_s =
        action->phase_exit_velocity_mm_s[phase_index];
    f413_run_session_abort_reason_t reason;

    if (!isfinite(phase_distance_mm) || (phase_distance_mm < 0.0f) ||
        !isfinite(phase_exit_velocity_mm_s) ||
        (phase_exit_velocity_mm_s < 0.0f))
    {
      return F413_RUN_SESSION_ABORT_IMU_FAULT;
    }
    if (phase_distance_mm <= 0.0f)
    {
      *speed_now_mm_s = phase_exit_velocity_mm_s;
      continue;
    }
    reason = f413_path_run_drive_segment_ex(phase_distance_mm,
                                            phase_exit_velocity_mm_s,
                                            speed_now_mm_s,
                                            guard,
                                            trace_flags,
                                            wall_control_gate,
                                            diagonal_control_gate);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
  }

  *speed_now_mm_s = action->exit_velocity_mm_s;
  return F413_RUN_SESSION_ABORT_NONE;
}

static f413_run_session_abort_reason_t f413_path_run_drive_front_wall_entry_segment(
    float distance_mm,
    float target_velocity_mm_s,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  float front_distance_mm;
  float front_target_mm;
  float target_distance;
  float start_velocity;
  bool front_reached = false;
  const uint32_t deadline = HAL_GetTick() + NIGHTFALL_F413_PATH_TIMEOUT_MS;

  if (speed_now_mm_s == NULL)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
  front_target_mm = F_ALIGN_TARGET_MM + (float)DIST_HALF_SEC - distance_mm;
  if ((distance_mm <= 0.0f) || !f413_run_features_front_wall_correction_enabled() ||
      f413_run_features_test_mode_run() || (front_target_mm <= 0.0f) ||
      !f413_wall_distance_front_unwarped_mm(&front_distance_mm))
  {
    return f413_path_run_drive_segment(distance_mm, target_velocity_mm_s,
                                      speed_now_mm_s, guard, trace_flags);
  }

  start_velocity = *speed_now_mm_s;
  target_distance = f413_ctrl_get_distance() + distance_mm;
  f413_path_run_prepare_straight_angle_control();
  f413_ctrl_set_velocity_profile(start_velocity, target_velocity_mm_s, distance_mm);
  f413_ctrl_set_omega(0.0f);

  while (1)
  {
    if (f413_wall_distance_front_unwarped_mm(&front_distance_mm) &&
        (front_distance_mm <= front_target_mm))
    {
      front_reached = true;
      break;
    }
    if (fabsf(f413_ctrl_get_distance()) >= fabsf(target_distance))
    {
      break;
    }
    if ((int32_t)(HAL_GetTick() - deadline) >= 0)
    {
      *speed_now_mm_s = target_velocity_mm_s;
      f413_path_run_distance_cursor_reset(
          &g_f413_path_run_distance_cursor, f413_ctrl_get_distance());
      return F413_RUN_SESSION_ABORT_TIMEOUT;
    }
    f413_trace_log_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    f413_wall_runtime_poll_straight(
        (trace_flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG) != 0U);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      *speed_now_mm_s = target_velocity_mm_s;
      f413_path_run_distance_cursor_reset(
          &g_f413_path_run_distance_cursor, f413_ctrl_get_distance());
      return reason;
    }
  }

  if (!front_reached && (WALL_END_EXTEND_MAX_MM > 0.0F))
  {
    const float extend_target = f413_ctrl_get_distance() + WALL_END_EXTEND_MAX_MM;
    f413_ctrl_set_velocity(target_velocity_mm_s);
    while (fabsf(f413_ctrl_get_distance()) < fabsf(extend_target))
    {
      if (!f413_wall_distance_front_unwarped_mm(&front_distance_mm))
      {
        break;
      }
      if (front_distance_mm <= front_target_mm)
      {
        break;
      }
      if ((int32_t)(HAL_GetTick() - deadline) >= 0)
      {
        reason = F413_RUN_SESSION_ABORT_TIMEOUT;
        break;
      }
      f413_trace_log_set_mode_flags(trace_flags);
      reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
      f413_wall_runtime_poll_straight(
          (trace_flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG) != 0U);
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }
    }
  }

  *speed_now_mm_s = target_velocity_mm_s;
  f413_path_run_distance_cursor_reset(
      &g_f413_path_run_distance_cursor, f413_ctrl_get_distance());
  return reason;
}

static f413_run_session_abort_reason_t f413_path_run_drive_wallend_segment(
    float distance_mm,
    float target_velocity_mm_s,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags,
    bool* wall_end_found)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  float target_distance;
  const uint32_t deadline = HAL_GetTick() + NIGHTFALL_F413_PATH_TIMEOUT_MS;

  if (wall_end_found != NULL)
  {
    *wall_end_found = false;
  }
  if (speed_now_mm_s == NULL)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
  if ((distance_mm <= 0.0f) || !f413_run_features_wall_end_correction_enabled())
  {
    return f413_path_run_drive_segment(distance_mm, target_velocity_mm_s,
                                      speed_now_mm_s, guard, trace_flags);
  }

  target_distance = f413_ctrl_get_distance() + distance_mm;
  f413_wall_runtime_end_clear();
  f413_path_run_prepare_straight_angle_control();
  f413_ctrl_set_velocity_profile(*speed_now_mm_s, target_velocity_mm_s, distance_mm);
  f413_ctrl_set_omega(0.0f);

  while (1)
  {
    if (f413_wall_runtime_poll_wall_end(true))
    {
      if (wall_end_found != NULL)
      {
        *wall_end_found = true;
      }
      break;
    }
    if (fabsf(f413_ctrl_get_distance()) >= fabsf(target_distance))
    {
      break;
    }
    if ((int32_t)(HAL_GetTick() - deadline) >= 0)
    {
      reason = F413_RUN_SESSION_ABORT_TIMEOUT;
      break;
    }
    f413_trace_log_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }
  }

  f413_wall_runtime_control_apply(false);
  *speed_now_mm_s = target_velocity_mm_s;
  f413_path_run_distance_cursor_reset(
      &g_f413_path_run_distance_cursor, f413_ctrl_get_distance());
  return reason;
}

static f413_run_session_abort_reason_t f413_path_run_wait_smooth_turn_profile(
    const f413_path_run_turn_t* turn,
    bool next_is_large_turn,
    float dist_wall_end_mm,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  f413_path_run_smooth_turn_t profile;
  uint16_t straight_trace_flags;
  uint16_t turn_trace_flags;
  uint32_t start_ms;
  int8_t turn_sign;

  if (turn == NULL)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  profile = f413_path_run_build_smooth_turn(turn->signed_angle_deg,
                                            turn->alpha_deg_s2,
                                            NIGHTFALL_F413_PATH_OMEGA_CAP);
  if (profile.t_total_s <= 0.0f)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  turn_sign = (turn->signed_angle_deg < 0.0f) ? -1 : 1;
  straight_trace_flags = f413_path_run_motor_phase_flags(
      trace_flags,
      turn->wall_control_offsets ? NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG : 0U);
  turn_trace_flags = f413_path_run_motor_phase_flags(trace_flags,
                                                     NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG);

  if (turn->front_wall_entry)
  {
    reason = f413_path_run_drive_front_wall_entry_segment(turn->dist_in_mm,
                                                          turn->velocity_mm_s,
                                                          speed_now_mm_s,
                                                          guard,
                                                          straight_trace_flags);
  }
  else if (turn->wall_control_offsets)
  {
    reason = f413_path_run_drive_segment(turn->dist_in_mm,
                                         turn->velocity_mm_s,
                                         speed_now_mm_s,
                                         guard,
                                         straight_trace_flags);
  }
  else
  {
    reason = f413_path_run_drive_segment_no_wall(turn->dist_in_mm,
                                                 turn->velocity_mm_s,
                                                 speed_now_mm_s,
                                                 guard,
                                                 straight_trace_flags);
  }
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  f413_path_run_prepare_turn_angle_control();
  f413_ctrl_set_velocity(turn->velocity_mm_s);
  f413_ctrl_start_omega_profile((float)turn_sign * profile.omega_peak_deg_s,
                                profile.t_acc_s,
                                profile.t_cruise_s);
  start_ms = HAL_GetTick();

  while (1)
  {
    const float t_s = (float)(HAL_GetTick() - start_ms) * 0.001f;

    if (t_s >= profile.t_total_s)
    {
      break;
    }

    f413_trace_log_set_mode_flags(turn_trace_flags);
    f413_ctrl_set_velocity(turn->velocity_mm_s);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      f413_ctrl_stop_omega_profile();
      return reason;
    }
  }

  f413_ctrl_stop_omega_profile();
  f413_path_run_distance_cursor_reset(
      &g_f413_path_run_distance_cursor, f413_ctrl_get_distance());
  if (turn->large_turn && next_is_large_turn &&
      f413_run_features_wall_end_correction_enabled() &&
      (turn->dist_out_mm > 0.0f))
  {
    bool wall_end_found = false;

    reason = f413_path_run_drive_wallend_segment(turn->dist_out_mm,
                                                 turn->velocity_mm_s,
                                                 speed_now_mm_s,
                                                 guard,
                                                 straight_trace_flags,
                                                 &wall_end_found);
    if ((reason == F413_RUN_SESSION_ABORT_NONE) && wall_end_found &&
        (dist_wall_end_mm > 0.0f))
    {
      reason = f413_path_run_drive_segment_no_wall(dist_wall_end_mm,
                                                   turn->velocity_mm_s,
                                                   speed_now_mm_s,
                                                   guard,
                                                   straight_trace_flags);
    }
    return reason;
  }
  if (turn->wall_control_offsets)
  {
    return f413_path_run_drive_segment(turn->dist_out_mm,
                                       turn->velocity_mm_s,
                                       speed_now_mm_s,
                                       guard,
                                       straight_trace_flags);
  }
  return f413_path_run_drive_segment_no_wall(turn->dist_out_mm,
                                             turn->velocity_mm_s,
                                             speed_now_mm_s,
                                             guard,
                                             straight_trace_flags);
}

static f413_run_session_abort_reason_t f413_path_run_run_straight_steps(
    uint32_t straight_steps,
    uint16_t prev_code,
    uint16_t next_code,
    const ShortestRunModeParams_t* mode_params,
    const ShortestRunCaseParams_t* case_params,
    const f413_path_run_prepared_linear_t* prepared,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  const float straight_mm = (float)straight_steps * (float)DIST_HALF_SEC;
  const float v_next =
      f413_path_run_next_straight_exit_velocity(next_code, mode_params, case_params);
  f413_run_session_abort_reason_t reason;
  bool next_is_small_turn;
  bool next_is_large_turn;
  bool prev_is_small_turn;
  bool prev_is_large_turn;
  bool skip_wallend;

  if ((straight_mm <= 0.0f) || (speed_now_mm_s == NULL) ||
      (prepared == NULL) ||
      (fabsf(prepared->entry_velocity_mm_s - *speed_now_mm_s) > 0.5f) ||
      (fabsf(prepared->exit_velocity_mm_s - v_next) > 0.5f))
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  next_is_small_turn = f413_path_run_is_small_turn_code(next_code);
  next_is_large_turn = f413_path_run_is_large_turn_code(next_code);
  prev_is_small_turn = f413_path_run_is_small_turn_code(prev_code);
  prev_is_large_turn = f413_path_run_is_large_turn_code(prev_code);
  skip_wallend = (prev_is_large_turn && (straight_steps == 1U) &&
                  next_is_small_turn) ||
                 (prev_is_small_turn && (straight_steps == 1U) &&
                  next_is_large_turn) ||
                 !f413_run_features_wall_end_correction_enabled();

  if ((next_is_small_turn || next_is_large_turn) && !skip_wallend)
  {
    const float buffer_max_mm = (float)DIST_HALF_SEC;
    bool wall_end_found = false;

    if (prepared->execute_plan)
    {
      reason = f413_path_run_execute_prepared_linear(prepared,
                                                     speed_now_mm_s,
                                                     guard,
                                                     trace_flags,
                                                     true,
                                                     false);
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        return reason;
      }
    }
    reason = f413_path_run_drive_wallend_segment(buffer_max_mm,
                                                 v_next,
                                                 speed_now_mm_s,
                                                 guard,
                                                 trace_flags,
                                                 &wall_end_found);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
    if (wall_end_found)
    {
      const float follow_dist = next_is_small_turn
          ? ((float)DIST_HALF_SEC + mode_params->dist_wall_end)
          : mode_params->dist_wall_end;
      if (follow_dist > 0.0f)
      {
        /* A disappearing wall is not a reliable heading reference after detection. */
        return f413_path_run_drive_segment_no_wall(follow_dist,
                                                   v_next,
                                                   speed_now_mm_s,
                                                   guard,
                                                   trace_flags);
      }
    }
    return F413_RUN_SESSION_ABORT_NONE;
  }

  return f413_path_run_execute_prepared_linear(prepared,
                                               speed_now_mm_s,
                                               guard,
                                               trace_flags,
                                               true,
                                               false);
}

static f413_run_session_abort_reason_t f413_path_run_run_diagonal_steps(
    uint32_t diagonal_steps,
    uint16_t next_code,
    const ShortestRunModeParams_t* mode_params,
    const ShortestRunCaseParams_t* case_params,
    const f413_path_run_prepared_linear_t* prepared,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  const float straight_mm =
      (float)diagonal_steps * (float)DIST_D_HALF_SEC;
  float v_next;

  if ((straight_mm <= 0.0f) || (speed_now_mm_s == NULL) ||
      (prepared == NULL))
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  v_next = f413_path_run_next_diagonal_exit_velocity(
      next_code, mode_params, case_params,
      (f413_run_features_test_mode_run() && (next_code == 0U))
          ? *speed_now_mm_s
          : 0.0f);

  if ((fabsf(prepared->entry_velocity_mm_s - *speed_now_mm_s) > 0.5f) ||
      (fabsf(prepared->exit_velocity_mm_s - v_next) > 0.5f))
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
  return f413_path_run_execute_prepared_linear(prepared,
                                               speed_now_mm_s,
                                               guard,
                                               trace_flags,
                                               false,
                                               true);
}

void f413_path_run_print_preview(void)
{
  uint16_t count = 0U;
  uint16_t limit;
  uint16_t i;

  while ((count < NIGHTFALL_F413_PATH_MAX_CODES) && (path[count] != 0U))
  {
    count++;
  }

  limit = (count > NIGHTFALL_F413_PATH_PREVIEW_MAX) ? NIGHTFALL_F413_PATH_PREVIEW_MAX : count;
  trace_printf("[RUN-TEST] solver-path codes(%u): ", (unsigned int)count);

  for (i = 0U; i < limit; i++)
  {
    uint16_t code = path[i];
    if ((code > 200U) && (code < 300U))
    {
      trace_printf("S%u", (unsigned int)(code - 200U));
    }
    else if (code == 300U)
    {
      trace_printf("s-R90");
    }
    else if (code == 400U)
    {
      trace_printf("s-L90");
    }
    else if (code == 501U || code == 502U)
    {
      trace_printf("L-R%s", (code == 501U) ? "90" : "180");
    }
    else if (code == 601U || code == 602U)
    {
      trace_printf("L-L%s", (code == 601U) ? "90" : "180");
    }
    else if ((code >= 701U) && (code <= 704U))
    {
      trace_printf("D45-%u", (unsigned int)(code - 700U));
    }
    else if ((code >= 801U) && (code <= 802U))
    {
      trace_printf("V90-%u", (unsigned int)(code - 800U));
    }
    else if ((code >= 901U) && (code <= 904U))
    {
      trace_printf("D135-%u", (unsigned int)(code - 900U));
    }
    else if (code > 1000U)
    {
      trace_printf("DS%u", (unsigned int)(code - 1000U));
    }
    else
    {
      trace_printf("%u", (unsigned int)code);
    }

    if ((i + 1U) < limit)
    {
      trace_printf(",");
    }
  }
  if (count > limit)
  {
    trace_printf(",...");
  }
  trace_printf("\r\n");
}

void f413_path_run_session_once(uint8_t mode,
                                uint8_t case_index,
                                uint16_t base_trace_flag,
                                const char* label)
{
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};
  const ShortestRunModeParams_t* mode_params = f413_path_run_mode_params(mode);
  const ShortestRunCaseParams_t* case_params = f413_path_run_case_params(mode, case_index);
  const float straight_velocity = f413_path_run_velocity_or_cap(
      case_params->velocity_straight,
      NIGHTFALL_F413_PATH_VELOCITY,
      NIGHTFALL_F413_PATH_VELOCITY_CAP);
  const float diagonal_velocity = f413_path_run_velocity_or_cap(
      case_params->velocity_d_straight,
      straight_velocity,
      NIGHTFALL_F413_PATH_DIAGONAL_VELOCITY_CAP);
  const float first_speed = f413_path_run_cap_positive(
      sqrtf(fmaxf(0.0f,
                  2.0f * case_params->acceleration_straight *
                      (float)DIST_FIRST_SEC)),
      NIGHTFALL_F413_PATH_VELOCITY_CAP);
  f413_path_run_preflight_result_t preflight;
  float speed_now = 0.0f;
  bool diagonal = false;
  uint16_t pi;
  uint16_t code;
  size_t prepared_linear_index = 0U;

  if (f413_trace_log_auto_is_enabled())
  {
    trace_printf("[RUN-TEST] busy(auto already running)\r\n");
    return;
  }
  if (f413_hw_stop_switch_pressed())
  {
    trace_printf("[RUN-TEST] path canceled(start switch pressed)\r\n");
    return;
  }
  if (path[0] == 0U)
  {
    trace_printf("[RUN-TEST] path canceled(empty path)\r\n");
    return;
  }
  preflight = f413_path_run_preflight_prepare(
      path, NIGHTFALL_F413_PATH_MAX_CODES, mode_params, case_params,
      first_speed, f413_run_features_wall_end_correction_enabled(),
      f413_run_features_test_mode_run(), &g_f413_path_run_prepared_path);
  if (preflight.status != F413_PATH_RUN_PREFLIGHT_OK)
  {
    trace_printf(
        "[RUN-TEST] path canceled(preflight=%u legacy=%s index=%lu code=%u)\r\n",
        (unsigned int)preflight.status,
        nf_legacy_path_status_name(preflight.legacy_status),
        (unsigned long)preflight.index,
        (unsigned int)preflight.code);
    return;
  }
  trace_printf("[RUN-TEST] path session start %s mode=%u case=%u v=%.0f diag_v=%.0f caps=%.0f/%.0f/%.0f, press switch to abort\r\n",
               (label != NULL) ? label : "path",
               (unsigned int)mode,
               (unsigned int)case_index,
               (double)straight_velocity,
               (double)diagonal_velocity,
               (double)NIGHTFALL_F413_PATH_VELOCITY_CAP,
               (double)NIGHTFALL_F413_PATH_DIAGONAL_VELOCITY_CAP,
               (double)NIGHTFALL_F413_PATH_TURN_VELOCITY_CAP);

  if (!f413_path_run_print_turn_profiles_before_run(mode_params))
  {
    trace_printf("[RUN-TEST] path canceled(turn profile preparation failed)\r\n");
    return;
  }

  if (!f413_run_session_guard_prepare(&guard))
  {
    trace_printf("[RUN-TEST] path canceled(guard init fail)\r\n");
    return;
  }

  f413_wall_runtime_set_control_gains(case_params->kp_wall, case_params->kp_diagonal);
  f413_wall_runtime_set_wall_end_thresholds(mode_params->wall_end_thr_r_high,
                                            mode_params->wall_end_thr_r_low,
                                            mode_params->wall_end_thr_l_high,
                                            mode_params->wall_end_thr_l_low);
  f413_path_run_trace_on_run_start();
  f413_ctrl_start();
  f413_path_run_distance_cursor_reset(
      &g_f413_path_run_distance_cursor, f413_ctrl_get_distance());

  abort_reason = f413_path_run_drive_segment(
      (float)DIST_FIRST_SEC,
      first_speed,
      &speed_now,
      &guard,
      (uint16_t)(base_trace_flag |
                 NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                 NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));

  for (pi = 0U; pi < NIGHTFALL_F413_PATH_MAX_CODES; pi++)
  {
    uint16_t next_code;

    if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }

    code = path[pi];
    if (code == 0U)
    {
      break;
    }
    next_code = ((pi + 1U) < NIGHTFALL_F413_PATH_MAX_CODES) ? path[pi + 1U] : 0U;

    if (f413_path_run_is_straight_code(code))
    {
      const f413_path_run_prepared_linear_t* prepared;
      uint32_t straight_steps = 0U;
      uint16_t end = pi;

      if ((prepared_linear_index >= g_f413_path_run_prepared_path.count) ||
          (g_f413_path_run_prepared_path.actions[prepared_linear_index].path_index != pi))
      {
        abort_reason = F413_RUN_SESSION_ABORT_IMU_FAULT;
        break;
      }
      prepared =
          &g_f413_path_run_prepared_path.actions[prepared_linear_index++];

      while ((end < NIGHTFALL_F413_PATH_MAX_CODES) &&
             f413_path_run_is_straight_code(path[end]))
      {
        straight_steps +=
            (uint32_t)(path[end] - NF_LEGACY_PATH_STRAIGHT_BASE);
        end++;
      }
      next_code = (end < NIGHTFALL_F413_PATH_MAX_CODES) ? path[end] : 0U;
      abort_reason = f413_path_run_run_straight_steps(
          straight_steps,
          (pi > 0U) ? path[pi - 1U] : 0U,
          next_code,
          mode_params,
          case_params,
          prepared,
          &speed_now,
          &guard,
          (uint16_t)(base_trace_flag | NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                     NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
      pi = (uint16_t)(end - 1U);
    }
    else if (f413_path_run_is_diagonal_code(code))
    {
      const f413_path_run_prepared_linear_t* prepared;
      uint32_t diagonal_steps = 0U;
      uint16_t end = pi;

      if ((prepared_linear_index >= g_f413_path_run_prepared_path.count) ||
          (g_f413_path_run_prepared_path.actions[prepared_linear_index].path_index != pi))
      {
        abort_reason = F413_RUN_SESSION_ABORT_IMU_FAULT;
        break;
      }
      prepared =
          &g_f413_path_run_prepared_path.actions[prepared_linear_index++];

      while ((end < NIGHTFALL_F413_PATH_MAX_CODES) &&
             f413_path_run_is_diagonal_code(path[end]))
      {
        diagonal_steps +=
            (uint32_t)(path[end] - NF_LEGACY_PATH_DIAGONAL_STRAIGHT_BASE);
        end++;
      }
      next_code = (end < NIGHTFALL_F413_PATH_MAX_CODES) ? path[end] : 0U;
      abort_reason = f413_path_run_run_diagonal_steps(
          diagonal_steps,
          next_code,
          mode_params,
          case_params,
          prepared,
          &speed_now,
          &guard,
          (uint16_t)(base_trace_flag | NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                     NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
      pi = (uint16_t)(end - 1U);
    }
    else
    {
      f413_path_run_turn_t turn;

      if (f413_path_run_turn_from_code(code, mode_params, &turn))
      {
        abort_reason = f413_path_run_wait_smooth_turn_profile(&turn,
                                                              (next_code >= 500U) && (next_code < 700U),
                                                              mode_params->dist_wall_end,
                                                              &speed_now,
                                                              &guard,
            (uint16_t)(base_trace_flag | NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                       NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG));
        if (f413_path_run_turn_enters_diagonal(code))
        {
          diagonal = true;
        }
        else if (f413_path_run_turn_exits_diagonal(code))
        {
          diagonal = false;
        }
      }
      else
      {
        trace_printf("[RUN-TEST] unsupported path code %u at [%u]\r\n",
                     (unsigned int)code, (unsigned int)pi);
        abort_reason = F413_RUN_SESSION_ABORT_IMU_FAULT;
      }
    }

    if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }

    f413_ctrl_clear_angle_target();
  }

  if ((abort_reason == F413_RUN_SESSION_ABORT_NONE) &&
      (pi >= NIGHTFALL_F413_PATH_MAX_CODES))
  {
    trace_printf("[RUN-TEST] path aborted(unterminated path)\r\n");
    abort_reason = F413_RUN_SESSION_ABORT_TIMEOUT;
  }

  if ((abort_reason == F413_RUN_SESSION_ABORT_NONE) &&
      (prepared_linear_index != g_f413_path_run_prepared_path.count))
  {
    trace_printf("[RUN-TEST] path aborted(prepared linear mismatch %lu/%lu)\r\n",
                 (unsigned long)prepared_linear_index,
                 (unsigned long)g_f413_path_run_prepared_path.count);
    abort_reason = F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    abort_reason = f413_path_run_drive_segment_ex(
        (float)DIST_HALF_SEC,
        0.0f,
        &speed_now,
        &guard,
        (uint16_t)(base_trace_flag |
                   NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                   NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG),
        !diagonal,
        diagonal);
  }

  if ((abort_reason == F413_RUN_SESSION_ABORT_NONE) &&
      f413_run_features_test_mode_run())
  {
    abort_reason = f413_path_run_settle_test_stop(
        &guard,
        (uint16_t)(base_trace_flag |
                   NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                   NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
  }

  f413_ctrl_clear_angle_target();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  f413_trace_log_set_mode_flags((uint16_t)(base_trace_flag |
                                           NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                                           NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG));
  (void)f413_run_session_wait_with_auto_step_guarded(NIGHTFALL_F413_PATH_COAST_MS, &guard);
  f413_ctrl_stop();
  f413_path_run_distance_cursor_invalidate(
      &g_f413_path_run_distance_cursor);
  f413_wall_runtime_reset_wall_end_thresholds();

  if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    f413_trace_log_set_mode_flags((uint16_t)(base_trace_flag |
                                             NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                                             f413_run_session_abort_reason_to_trace_flag(abort_reason)));
    f413_trace_log_auto_step();
  }

  f413_trace_log_set_mode_flags(0U);
  f413_path_run_trace_on_run_stop();
  f413_run_session_guard_cleanup(&guard);

  if (abort_reason == F413_RUN_SESSION_ABORT_SWITCH)
  {
    trace_printf("[RUN-TEST] path aborted by switch at code[%u]\r\n", (unsigned int)pi);
  }
  else if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    trace_printf("[RUN-TEST] path aborted(%s) at code[%u]\r\n",
                 f413_run_session_abort_reason_to_text(abort_reason),
                 (unsigned int)pi);
  }
  else
  {
    trace_printf("[RUN-TEST] path end (%u codes, dist=%.0fmm, angle=%.0fdeg)\r\n",
                 (unsigned int)pi,
                 (double)f413_ctrl_get_distance(),
                 (double)f413_ctrl_get_angle());
  }
}

void f413_path_run_solver_session_once(uint16_t base_trace_flag)
{
  f413_path_run_session_once(NIGHTFALL_F413_SOLVER_MODE,
                             NIGHTFALL_F413_SOLVER_CASE,
                             base_trace_flag,
                             "solver-path");
}

void f413_path_run_custom_path_session_once(const char* label,
                                            uint8_t mode,
                                            uint8_t case_index,
                                            const uint16_t* codes,
                                            uint16_t code_count,
                                            uint16_t base_trace_flag)
{
  uint16_t i;

  if ((codes == NULL) || (code_count == 0U))
  {
    trace_printf("[OP-UI][PATH-TEST] no sequence\r\n");
    return;
  }
  if (code_count >= NIGHTFALL_F413_PATH_MAX_CODES)
  {
    trace_printf("[OP-UI][PATH-TEST] too many codes(%u)\r\n", (unsigned int)code_count);
    return;
  }

  trace_printf("[OP-UI][PATH-TEST] set path %s mode=%u case=%u codes=",
               label,
               (unsigned int)mode,
               (unsigned int)case_index);
  for (i = 0U; i < code_count; i++)
  {
    trace_printf("%u%s", (unsigned int)codes[i], ((i + 1U) < code_count) ? "," : "");
    path[i] = codes[i];
  }
  path[code_count] = 0U;
  trace_printf("\r\n");

  for (i = (uint16_t)(code_count + 1U); i < NIGHTFALL_F413_PATH_MAX_CODES; i++)
  {
    path[i] = 0U;
  }

  f413_path_run_session_once(mode, case_index, base_trace_flag, label);
}

#endif /* !NIGHTFALL_F413_PATH_LINEAR_PLAN_HOST_TEST */

#endif
