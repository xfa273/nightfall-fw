#include "f413_search_step.h"

#include <math.h>
#include <string.h>

#include "f413_control.h"
#include "f413_path_run.h"
#include "f413_run_features.h"
#include "f413_run_session.h"
#include "f413_wall_runtime.h"
#include "nvm_params.h"
#include "params.h"
#include "search.h"
#include "search_run_params.h"
#include "trace.h"

#define F413_SEARCH_STEP_MAZE_WALL_W (0x01U)
#define F413_SEARCH_STEP_MAZE_WALL_S (0x02U)
#define F413_SEARCH_STEP_MAZE_WALL_E (0x04U)
#define F413_SEARCH_STEP_MAZE_WALL_N (0x08U)
#define F413_SEARCH_STEP_MAZE_WALL_KNOWN_MASK (0x0FU)
#define F413_SEARCH_STEP_MAZE_START_FORCED_WALLS (0x07U)
#define F413_SEARCH_STEP_CELL_COUNT ((uint32_t)(MAZE_SIZE * MAZE_SIZE))
#ifndef F413_SEARCH_STEP_AUTO_MAX_ACTIONS
#define F413_SEARCH_STEP_AUTO_MAX_ACTIONS (256U)
#endif

typedef struct {
  float omega_peak_deg_s;
  float t_acc_s;
  float t_cruise_s;
  float t_total_s;
} f413_search_step_smooth_turn_t;

static f413_search_step_config_t g_config;
static bool g_session_active = false;
static uint8_t g_search_dual_wall_streak = 0U;
static uint8_t g_search_right_wall_streak = 0U;
static uint8_t g_search_left_wall_streak = 0U;

static uint32_t f413_search_step_tick(void)
{
  if (g_config.get_tick_ms != NULL)
  {
    return g_config.get_tick_ms();
  }
  return 0U;
}

static bool f413_search_step_stop_switch_pressed(void)
{
  return (g_config.stop_switch_pressed != NULL) && g_config.stop_switch_pressed();
}

static bool f413_search_step_read_wall(f413_wall_sensor_snapshot_t* wall)
{
  return (g_config.read_wall_snapshot != NULL) && g_config.read_wall_snapshot(wall);
}

static void f413_search_step_set_mode_flags(uint16_t flags)
{
  if (g_config.trace_set_mode_flags != NULL)
  {
    g_config.trace_set_mode_flags(flags);
  }
}

static void f413_search_step_trace_start(void)
{
  if (g_config.trace_on_run_start != NULL)
  {
    g_config.trace_on_run_start();
  }
}

static void f413_search_step_trace_stop(void)
{
  if (g_config.trace_on_run_stop != NULL)
  {
    g_config.trace_on_run_stop();
  }
}

static void f413_search_step_trace_auto_step(void)
{
  if (g_config.trace_auto_step != NULL)
  {
    g_config.trace_auto_step();
  }
}

static bool f413_search_step_trace_auto_is_enabled(void)
{
  return (g_config.trace_auto_is_enabled != NULL) && g_config.trace_auto_is_enabled();
}

static void f413_search_step_set_context(uint8_t mode, uint8_t op_case, uint8_t sub, uint8_t test_id)
{
  if (g_config.trace_set_context != NULL)
  {
    g_config.trace_set_context(mode, op_case, sub, test_id);
  }
}

static void f413_search_step_set_action_context(uint8_t op_case,
                                                uint8_t x,
                                                uint8_t y,
                                                uint8_t dir,
                                                uint8_t next_rel)
{
  uint8_t packed_xy = (uint8_t)(((y & 0x0FU) << 4U) | (x & 0x0FU));
  uint8_t packed_action = (uint8_t)(0x80U |
                                    ((next_rel & 0x03U) << 2U) |
                                    (dir & 0x03U));

  f413_search_step_set_context(1U, op_case, packed_xy, packed_action);
}

static void f413_search_step_prepare_straight_angle_control(void)
{
  if (!f413_run_features_angle_accum_mode())
  {
    f413_ctrl_reset_angle();
  }
  f413_ctrl_clear_angle_target();
}

static void f413_search_step_prepare_turn_angle_control(void)
{
  if (!f413_run_features_angle_accum_mode())
  {
    f413_ctrl_reset_angle();
  }
  f413_ctrl_clear_angle_target();
  f413_wall_runtime_control_clear();
}

static void f413_search_step_angle_reset_streak_clear(void)
{
  g_search_dual_wall_streak = 0U;
  g_search_right_wall_streak = 0U;
  g_search_left_wall_streak = 0U;
}

static void f413_search_step_angle_reset_streak_update(void)
{
  const bool right_wall = ((wall_info & 0x44U) != 0U);
  const bool left_wall = ((wall_info & 0x11U) != 0U);

  if (!f413_run_features_angle_accum_mode())
  {
    f413_search_step_angle_reset_streak_clear();
    return;
  }

  if (right_wall && left_wall)
  {
    if (g_search_dual_wall_streak < UINT8_MAX)
    {
      g_search_dual_wall_streak++;
    }
  }
  else
  {
    g_search_dual_wall_streak = 0U;
  }

  if (right_wall)
  {
    if (g_search_right_wall_streak < UINT8_MAX)
    {
      g_search_right_wall_streak++;
    }
  }
  else
  {
    g_search_right_wall_streak = 0U;
  }

  if (left_wall)
  {
    if (g_search_left_wall_streak < UINT8_MAX)
    {
      g_search_left_wall_streak++;
    }
  }
  else
  {
    g_search_left_wall_streak = 0U;
  }

  if ((g_search_dual_wall_streak >= SEARCH_ANGLE_RESET_DUAL_WALL_STREAK_CELLS) ||
      (g_search_right_wall_streak >= SEARCH_ANGLE_RESET_SINGLE_WALL_STREAK_CELLS) ||
      (g_search_left_wall_streak >= SEARCH_ANGLE_RESET_SINGLE_WALL_STREAK_CELLS))
  {
    f413_ctrl_reset_angle();
    f413_search_step_angle_reset_streak_clear();
  }
}

static bool f413_search_step_is_goal_cell(uint8_t x, uint8_t y)
{
  static const uint8_t goals[9][2] = {
      {GOAL1_X, GOAL1_Y}, {GOAL2_X, GOAL2_Y}, {GOAL3_X, GOAL3_Y},
      {GOAL4_X, GOAL4_Y}, {GOAL5_X, GOAL5_Y}, {GOAL6_X, GOAL6_Y},
      {GOAL7_X, GOAL7_Y}, {GOAL8_X, GOAL8_Y}, {GOAL9_X, GOAL9_Y},
  };
  uint8_t i;

  for (i = 0U; i < 9U; i++)
  {
    uint8_t gx = goals[i][0];
    uint8_t gy = goals[i][1];
    if (((gx != 0U) || (gy != 0U)) && (gx == x) && (gy == y))
    {
      return true;
    }
  }
  return false;
}

static const char* f413_search_step_target_name(uint8_t target)
{
  switch (target)
  {
    case F413_SEARCH_STEP_TARGET_GOAL: return "goal";
    case F413_SEARCH_STEP_TARGET_FULL: return "full";
    case F413_SEARCH_STEP_TARGET_START: return "start";
    default: return "?";
  }
}

static bool f413_search_step_target_reached(uint8_t target)
{
  if (target == F413_SEARCH_STEP_TARGET_GOAL)
  {
    return f413_search_step_is_goal_cell((uint8_t)mouse.x, (uint8_t)mouse.y);
  }
  if (target == F413_SEARCH_STEP_TARGET_START)
  {
    return (mouse.x == START_X) && (mouse.y == START_Y);
  }
  return false;
}

static bool f413_search_step_plan_from_case(uint8_t op_case,
                                            f413_search_step_case_config_t* out)
{
  if (out == NULL)
  {
    return false;
  }
  memset(out, 0, sizeof(*out));

  switch (op_case)
  {
    case 1U:
      out->param_index = 0U;
      out->phase_count = 2U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_GOAL;
      out->phases[1] = F413_SEARCH_STEP_TARGET_FULL;
      return true;
    case 2U:
      out->param_index = 0U;
      out->phase_count = 1U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_FULL;
      return true;
    case 3U:
      out->param_index = 0U;
      out->phase_count = 2U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_GOAL;
      out->phases[1] = F413_SEARCH_STEP_TARGET_START;
      return true;
    case 4U:
      out->param_index = 0U;
      out->phase_count = 1U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_GOAL;
      return true;
    case 5U:
      out->param_index = 1U;
      out->phase_count = 2U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_GOAL;
      out->phases[1] = F413_SEARCH_STEP_TARGET_FULL;
      return true;
    case 6U:
      out->param_index = 1U;
      out->phase_count = 1U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_FULL;
      return true;
    case 7U:
      out->param_index = 1U;
      out->phase_count = 2U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_GOAL;
      out->phases[1] = F413_SEARCH_STEP_TARGET_START;
      return true;
    case 8U:
      out->param_index = 1U;
      out->phase_count = 1U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_GOAL;
      return true;
    default:
      return false;
  }
}

static void f413_search_step_map_init_empty(void)
{
  uint8_t x;
  uint8_t y;

  for (y = 0U; y < MAZE_SIZE; y++)
  {
    for (x = 0U; x < MAZE_SIZE; x++)
    {
      map[y][x] = 0xF0U;
      visited[y][x] = false;
    }
  }

  for (y = 0U; y < MAZE_SIZE; y++)
  {
    map[y][0] |= 0xF1U;
    map[y][MAZE_SIZE - 1U] |= 0xF4U;
  }
  for (x = 0U; x < MAZE_SIZE; x++)
  {
    map[0][x] |= 0xF2U;
    map[MAZE_SIZE - 1U][x] |= 0xF8U;
  }

  map[START_Y][START_X] |= 0x44U;
  if (START_X > 0U)
  {
    map[START_Y][START_X - 1U] |= 0x44U;
  }
  visited[START_Y][START_X] = true;
}

static uint16_t f413_search_step_wall_info_from_snapshot(const f413_wall_sensor_snapshot_t* wall)
{
  uint16_t info = 0U;

  if (wall == NULL)
  {
    return 0U;
  }
  if (wall->front_wall)
  {
    info |= 0x88U;
  }
  if (wall->right_wall)
  {
    info |= 0x44U;
  }
  if (wall->left_wall)
  {
    info |= 0x11U;
  }
  return info;
}

static void f413_search_step_write_map_cell(uint8_t x, uint8_t y, uint8_t dir, uint16_t relative_wall_info)
{
  uint16_t m_temp;

  if ((x >= MAZE_SIZE) || (y >= MAZE_SIZE))
  {
    return;
  }

  m_temp = (relative_wall_info >> (dir & 0x03U)) & F413_SEARCH_STEP_MAZE_WALL_KNOWN_MASK;
  if ((x == START_X) && (y == START_Y))
  {
    m_temp |= F413_SEARCH_STEP_MAZE_START_FORCED_WALLS;
  }
  m_temp |= (uint16_t)(m_temp << 4U);

  map[y][x] = m_temp;
  if (y != (MAZE_SIZE - 1U))
  {
    if ((m_temp & 0x88U) != 0U)
    {
      map[y + 1U][x] |= 0x22U;
    }
    else
    {
      map[y + 1U][x] &= (uint16_t)~0x22U;
    }
  }
  if (x != (MAZE_SIZE - 1U))
  {
    if ((m_temp & 0x44U) != 0U)
    {
      map[y][x + 1U] |= 0x11U;
    }
    else
    {
      map[y][x + 1U] &= (uint16_t)~0x11U;
    }
  }
  if (y != 0U)
  {
    if ((m_temp & 0x22U) != 0U)
    {
      map[y - 1U][x] |= 0x88U;
    }
    else
    {
      map[y - 1U][x] &= (uint16_t)~0x88U;
    }
  }
  if (x != 0U)
  {
    if ((m_temp & 0x11U) != 0U)
    {
      map[y][x - 1U] |= 0x44U;
    }
    else
    {
      map[y][x - 1U] &= (uint16_t)~0x44U;
    }
  }

  map[START_Y][START_X] |= 0x77U;
  if ((START_X + 1U) < MAZE_SIZE)
  {
    map[START_Y][START_X + 1U] |= 0x11U;
  }
}

static int f413_search_step_make_smap(uint8_t x, uint8_t y, uint8_t target)
{
  static uint16_t q[F413_SEARCH_STEP_CELL_COUNT];
  uint16_t head = 0U;
  uint16_t tail = 0U;
  uint8_t ix;
  uint8_t iy;
  static const uint8_t goals[9][2] = {
      {GOAL1_X, GOAL1_Y}, {GOAL2_X, GOAL2_Y}, {GOAL3_X, GOAL3_Y},
      {GOAL4_X, GOAL4_Y}, {GOAL5_X, GOAL5_Y}, {GOAL6_X, GOAL6_Y},
      {GOAL7_X, GOAL7_Y}, {GOAL8_X, GOAL8_Y}, {GOAL9_X, GOAL9_Y},
  };

  if ((x >= MAZE_SIZE) || (y >= MAZE_SIZE))
  {
    return -1;
  }

  for (iy = 0U; iy < MAZE_SIZE; iy++)
  {
    for (ix = 0U; ix < MAZE_SIZE; ix++)
    {
      smap[iy][ix] = 0xFFFFU;
    }
  }

  if (target == F413_SEARCH_STEP_TARGET_START)
  {
    smap[START_Y][START_X] = 0U;
    q[tail++] = (uint16_t)(START_Y * MAZE_SIZE + START_X);
  }
  else if (target == F413_SEARCH_STEP_TARGET_FULL)
  {
    bool any_unvisited = false;

    visited[START_Y][START_X] = true;
    for (iy = 0U; iy < MAZE_SIZE; iy++)
    {
      for (ix = 0U; ix < MAZE_SIZE; ix++)
      {
        if (!visited[iy][ix])
        {
          any_unvisited = true;
          smap[iy][ix] = 0U;
          q[tail++] = (uint16_t)(iy * MAZE_SIZE + ix);
        }
      }
    }
    if (!any_unvisited)
    {
      return -2;
    }
  }
  else
  {
    for (ix = 0U; ix < 9U; ix++)
    {
      uint8_t gx = goals[ix][0];
      uint8_t gy = goals[ix][1];
      if (((gx == 0U) && (gy == 0U)) || (gx >= MAZE_SIZE) || (gy >= MAZE_SIZE))
      {
        continue;
      }
      smap[gy][gx] = 0U;
      q[tail++] = (uint16_t)(gy * MAZE_SIZE + gx);
    }
  }

  while ((head < tail) && (smap[y][x] == 0xFFFFU))
  {
    uint16_t idx = q[head++];
    uint8_t cy = (uint8_t)(idx / MAZE_SIZE);
    uint8_t cx = (uint8_t)(idx - (uint16_t)(cy * MAZE_SIZE));
    uint16_t step = smap[cy][cx];
    uint8_t cell = (uint8_t)(map[cy][cx] & F413_SEARCH_STEP_MAZE_WALL_KNOWN_MASK);

    if (((cell & F413_SEARCH_STEP_MAZE_WALL_N) == 0U) && (cy != (MAZE_SIZE - 1U)) && (smap[cy + 1U][cx] == 0xFFFFU))
    {
      smap[cy + 1U][cx] = (uint16_t)(step + 1U);
      q[tail++] = (uint16_t)((cy + 1U) * MAZE_SIZE + cx);
    }
    if (((cell & F413_SEARCH_STEP_MAZE_WALL_E) == 0U) && (cx != (MAZE_SIZE - 1U)) && (smap[cy][cx + 1U] == 0xFFFFU))
    {
      smap[cy][cx + 1U] = (uint16_t)(step + 1U);
      q[tail++] = (uint16_t)(cy * MAZE_SIZE + (cx + 1U));
    }
    if (((cell & F413_SEARCH_STEP_MAZE_WALL_S) == 0U) && (cy != 0U) && (smap[cy - 1U][cx] == 0xFFFFU))
    {
      smap[cy - 1U][cx] = (uint16_t)(step + 1U);
      q[tail++] = (uint16_t)((cy - 1U) * MAZE_SIZE + cx);
    }
    if (((cell & F413_SEARCH_STEP_MAZE_WALL_W) == 0U) && (cx != 0U) && (smap[cy][cx - 1U] == 0xFFFFU))
    {
      smap[cy][cx - 1U] = (uint16_t)(step + 1U);
      q[tail++] = (uint16_t)(cy * MAZE_SIZE + (cx - 1U));
    }
  }

  return (smap[y][x] == 0xFFFFU) ? -1 : (int)smap[y][x];
}

static int f413_search_step_make_goal_smap(uint8_t x, uint8_t y)
{
  return f413_search_step_make_smap(x, y, F413_SEARCH_STEP_TARGET_GOAL);
}

static bool f413_search_step_choose_next_relative(uint8_t x, uint8_t y, uint8_t dir, uint8_t* out_rel)
{
  static const uint8_t rel_priority[4] = {0U, 1U, 3U, 2U};
  static const int8_t dx[4] = {0, 1, 0, -1};
  static const int8_t dy[4] = {1, 0, -1, 0};
  static const uint8_t wall_mask[4] = {
      F413_SEARCH_STEP_MAZE_WALL_N,
      F413_SEARCH_STEP_MAZE_WALL_E,
      F413_SEARCH_STEP_MAZE_WALL_S,
      F413_SEARCH_STEP_MAZE_WALL_W
  };
  uint8_t i;
  uint16_t current_step;

  if ((out_rel == NULL) || (x >= MAZE_SIZE) || (y >= MAZE_SIZE) || (smap[y][x] == 0xFFFFU))
  {
    return false;
  }

  current_step = smap[y][x];
  for (i = 0U; i < 4U; i++)
  {
    uint8_t rel = rel_priority[i];
    uint8_t abs_dir = (uint8_t)((dir + rel) & 0x03U);
    int16_t nx = (int16_t)x + dx[abs_dir];
    int16_t ny = (int16_t)y + dy[abs_dir];
    uint8_t cell = (uint8_t)(map[y][x] & F413_SEARCH_STEP_MAZE_WALL_KNOWN_MASK);

    if ((cell & wall_mask[abs_dir]) != 0U)
    {
      continue;
    }
    if ((nx < 0) || (ny < 0) || (nx >= (int16_t)MAZE_SIZE) || (ny >= (int16_t)MAZE_SIZE))
    {
      continue;
    }
    if (smap[ny][nx] < current_step)
    {
      *out_rel = rel;
      return true;
    }
  }
  return false;
}

static const char* f413_search_step_relative_name(uint8_t rel)
{
  switch (rel)
  {
    case 0U: return "forward";
    case 1U: return "right";
    case 2U: return "back";
    case 3U: return "left";
    default: return "?";
  }
}

static bool f413_search_step_forward_destination(uint8_t x,
                                                 uint8_t y,
                                                 uint8_t dir,
                                                 uint8_t* out_x,
                                                 uint8_t* out_y)
{
  int16_t nx = (int16_t)x;
  int16_t ny = (int16_t)y;

  switch (dir & 0x03U)
  {
    case 0U: ny++; break;
    case 1U: nx++; break;
    case 2U: ny--; break;
    case 3U: nx--; break;
    default: break;
  }
  if ((nx < 0) || (ny < 0) || (nx >= (int16_t)MAZE_SIZE) || (ny >= (int16_t)MAZE_SIZE))
  {
    return false;
  }
  if (out_x != NULL)
  {
    *out_x = (uint8_t)nx;
  }
  if (out_y != NULL)
  {
    *out_y = (uint8_t)ny;
  }
  return true;
}

static bool f413_search_step_forward_is_known_straight(uint8_t x,
                                                       uint8_t y,
                                                       uint8_t dir,
                                                       uint8_t* out_next_after_forward)
{
  uint8_t nx;
  uint8_t ny;
  uint8_t next_rel = 0U;

  if (out_next_after_forward != NULL)
  {
    *out_next_after_forward = 0xFFU;
  }
  if (!f413_search_step_forward_destination(x, y, dir, &nx, &ny))
  {
    return false;
  }
  if (!visited[ny][nx])
  {
    return false;
  }
  if (!f413_search_step_choose_next_relative(nx, ny, dir, &next_rel))
  {
    return false;
  }
  if (out_next_after_forward != NULL)
  {
    *out_next_after_forward = next_rel;
  }
  return next_rel == 0U;
}

static void f413_search_step_advance_position(void)
{
  switch (mouse.dir & 0x03U)
  {
    case 0U:
      if (mouse.y < (MAZE_SIZE - 1U))
      {
        mouse.y++;
      }
      break;
    case 1U:
      if (mouse.x < (MAZE_SIZE - 1U))
      {
        mouse.x++;
      }
      break;
    case 2U:
      if (mouse.y > 0U)
      {
        mouse.y--;
      }
      break;
    case 3U:
      if (mouse.x > 0U)
      {
        mouse.x--;
      }
      break;
    default:
      break;
  }
}

static float f413_search_step_turn_target_deg(uint8_t rel)
{
  switch (rel)
  {
    case 1U:
      return -g_config.step_turn_deg;
    case 2U:
      return 2.0f * g_config.step_turn_deg;
    case 3U:
      return g_config.step_turn_deg;
    default:
      return 0.0f;
  }
}

static float f413_search_step_accel_positive(const SearchRunParams_t* params)
{
  if ((params != NULL) && (params->acceleration_straight > 0.0f))
  {
    return params->acceleration_straight;
  }
  return 1000.0f;
}

static float f413_search_step_speed_after_accel(float speed_mm_s,
                                                float accel_mm_s2,
                                                float distance_mm)
{
  if ((accel_mm_s2 <= 0.0f) || (distance_mm <= 0.0f))
  {
    return speed_mm_s;
  }
  return sqrtf(fmaxf(0.0f,
                     (speed_mm_s * speed_mm_s) +
                         (2.0f * accel_mm_s2 * distance_mm)));
}

static f413_search_step_smooth_turn_t f413_search_step_build_smooth_turn(float angle_deg,
                                                                         float alpha_deg_s2)
{
  f413_search_step_smooth_turn_t profile = {0.0f, 0.0f, 0.0f, 0.0f};
  const float angle_abs = fabsf(angle_deg);
  float rounding_scale = TURN_OMEGA_PROFILE_ROUNDING_SCALE;

  if ((angle_abs <= 0.0f) || (alpha_deg_s2 <= 0.0f))
  {
    return profile;
  }
  if (rounding_scale < 0.1f)
  {
    rounding_scale = 0.1f;
  }

  profile.omega_peak_deg_s = sqrtf((2.0f * alpha_deg_s2 * angle_abs) / 3.0f);
  if ((NIGHTFALL_F413_PATH_OMEGA_CAP > 0.0f) &&
      (profile.omega_peak_deg_s > NIGHTFALL_F413_PATH_OMEGA_CAP))
  {
    profile.omega_peak_deg_s = NIGHTFALL_F413_PATH_OMEGA_CAP;
  }
  if (profile.omega_peak_deg_s <= 0.0f)
  {
    return profile;
  }

  profile.t_acc_s = (profile.omega_peak_deg_s / alpha_deg_s2) * rounding_scale;
  profile.t_cruise_s = (angle_abs / profile.omega_peak_deg_s) - profile.t_acc_s;
  if (profile.t_cruise_s < 0.0f)
  {
    profile.t_cruise_s = 0.0f;
    profile.omega_peak_deg_s = angle_abs / profile.t_acc_s;
    if ((NIGHTFALL_F413_PATH_OMEGA_CAP > 0.0f) &&
        (profile.omega_peak_deg_s > NIGHTFALL_F413_PATH_OMEGA_CAP))
    {
      profile.omega_peak_deg_s = NIGHTFALL_F413_PATH_OMEGA_CAP;
    }
    profile.t_cruise_s = (angle_abs / profile.omega_peak_deg_s) - profile.t_acc_s;
    if (profile.t_cruise_s < 0.0f)
    {
      profile.t_cruise_s = 0.0f;
    }
  }
  profile.t_total_s = (2.0f * profile.t_acc_s) + profile.t_cruise_s;
  return profile;
}

static f413_run_session_abort_reason_t f413_search_step_wait_ctrl_target(float target,
                                                                         bool is_angle,
                                                                         f413_run_session_guard_t* guard,
                                                                         uint16_t trace_flags)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  uint32_t deadline = f413_search_step_tick() + g_config.path_timeout_ms;

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

    if (f413_search_step_tick() >= deadline)
    {
      break;
    }

    f413_search_step_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if ((g_config.wall_control_apply_straight != NULL) &&
        !is_angle &&
        ((trace_flags & g_config.trace_motor_fwd_flag) != 0U))
    {
      g_config.wall_control_apply_straight();
    }
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
  }
  return F413_RUN_SESSION_ABORT_NONE;
}

static f413_run_session_abort_reason_t f413_search_step_drive_segment(float distance_mm,
                                                                      float target_velocity_mm_s,
                                                                      float* speed_now_mm_s,
                                                                      f413_run_session_guard_t* guard,
                                                                      uint16_t trace_flags)
{
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

  target_distance = f413_ctrl_get_distance() + distance_mm;
  f413_search_step_prepare_straight_angle_control();
  f413_ctrl_set_velocity_profile(*speed_now_mm_s, target_velocity_mm_s, distance_mm);
  f413_ctrl_set_omega(0.0f);

  reason = f413_search_step_wait_ctrl_target(target_distance, false, guard, trace_flags);
  *speed_now_mm_s = target_velocity_mm_s;
  return reason;
}

static bool f413_search_step_front_wall_present(void)
{
  f413_wall_sensor_snapshot_t wall;

  return f413_search_step_read_wall(&wall) && wall.front_wall;
}

static void f413_search_step_apply_straight_runtime(uint16_t trace_flags)
{
  if ((g_config.wall_control_apply_straight != NULL) &&
      ((trace_flags & g_config.trace_motor_fwd_flag) != 0U))
  {
    g_config.wall_control_apply_straight();
  }
}

static f413_run_session_abort_reason_t f413_search_step_drive_front_wall_entry_segment(
    float distance_mm,
    float target_velocity_mm_s,
    float front_threshold,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  float target_distance;

  if (speed_now_mm_s == NULL)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
  if ((distance_mm <= 0.0f) || !f413_run_features_front_wall_correction_enabled() ||
      f413_run_features_test_mode_run() || !f413_search_step_front_wall_present())
  {
    return f413_search_step_drive_segment(distance_mm, target_velocity_mm_s,
                                          speed_now_mm_s, guard, trace_flags);
  }

  target_distance = f413_ctrl_get_distance() + distance_mm;
  f413_search_step_prepare_straight_angle_control();
  f413_ctrl_set_velocity_profile(*speed_now_mm_s, target_velocity_mm_s, distance_mm);
  f413_ctrl_set_omega(0.0f);

  while (1)
  {
    if ((fabsf(f413_ctrl_get_distance()) >= fabsf(target_distance)) ||
        f413_wall_runtime_front_wall_reached(front_threshold))
    {
      break;
    }
    f413_search_step_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    f413_search_step_apply_straight_runtime(trace_flags);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      *speed_now_mm_s = target_velocity_mm_s;
      return reason;
    }
  }

  if (!f413_wall_runtime_front_wall_reached(front_threshold) &&
      (WALL_END_EXTEND_MAX_MM > 0.0F))
  {
    const float extend_target = f413_ctrl_get_distance() + WALL_END_EXTEND_MAX_MM;
    f413_ctrl_set_velocity(target_velocity_mm_s);
    while (fabsf(f413_ctrl_get_distance()) < fabsf(extend_target))
    {
      if (f413_wall_runtime_front_wall_reached(front_threshold))
      {
        break;
      }
      f413_search_step_set_mode_flags(trace_flags);
      reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
      f413_search_step_apply_straight_runtime(trace_flags);
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }
    }
  }

  *speed_now_mm_s = target_velocity_mm_s;
  return reason;
}

static f413_run_session_abort_reason_t f413_search_step_drive_wallend_segment(
    float distance_mm,
    float target_velocity_mm_s,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags,
    bool* wall_end_found)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  float target_distance;

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
    return f413_search_step_drive_segment(distance_mm, target_velocity_mm_s,
                                          speed_now_mm_s, guard, trace_flags);
  }

  target_distance = f413_ctrl_get_distance() + distance_mm;
  f413_wall_runtime_end_clear();
  f413_search_step_prepare_straight_angle_control();
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
    f413_search_step_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }
  }

  *speed_now_mm_s = target_velocity_mm_s;
  return reason;
}

static f413_run_session_abort_reason_t f413_search_step_drive_accel_distance(
    float distance_mm,
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  const float speed_out =
      f413_search_step_speed_after_accel((speed_now_mm_s != NULL) ? *speed_now_mm_s : 0.0f,
                                         f413_search_step_accel_positive(params),
                                         distance_mm);
  return f413_search_step_drive_segment(distance_mm, speed_out, speed_now_mm_s,
                                        guard, trace_flags);
}

static f413_run_session_abort_reason_t f413_search_step_drive_decel_distance(
    float distance_mm,
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  float speed_in = (speed_now_mm_s != NULL) ? *speed_now_mm_s : 0.0f;
  float accel = f413_search_step_accel_positive(params);
  float speed_sq = (speed_in * speed_in) - (2.0f * accel * distance_mm);
  float speed_out = (speed_sq > 0.0f) ? sqrtf(speed_sq) : 0.0f;

  return f413_search_step_drive_segment(distance_mm, speed_out, speed_now_mm_s,
                                        guard, trace_flags);
}

static f413_run_session_abort_reason_t f413_search_step_read_and_write_current_wall(
    bool force_start_front_open)
{
  f413_wall_sensor_snapshot_t wall;

  if (!f413_search_step_read_wall(&wall))
  {
    trace_printf("[SEARCH-RUN] FAIL(read wall snapshot)\r\n");
    return F413_RUN_SESSION_ABORT_WALL_FAULT;
  }
  wall_info = f413_search_step_wall_info_from_snapshot(&wall);
  if (force_start_front_open)
  {
    wall_info &= (uint16_t)~0x88U;
  }
  f413_search_step_write_map_cell((uint8_t)mouse.x, (uint8_t)mouse.y,
                                  (uint8_t)mouse.dir, wall_info);
  visited[mouse.y][mouse.x] = true;
  return F413_RUN_SESSION_ABORT_NONE;
}

static f413_run_session_abort_reason_t f413_search_step_run_entry_section(
    uint8_t op_case,
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard)
{
  f413_run_session_abort_reason_t reason;
  const uint16_t trace_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                         g_config.trace_motor_fwd_flag);

  f413_search_step_set_action_context(op_case,
                                      (uint8_t)mouse.x,
                                      (uint8_t)mouse.y,
                                      (uint8_t)mouse.dir,
                                      0U);

  reason = f413_search_step_drive_accel_distance((float)DIST_FIRST_SEC, params,
                                                 speed_now_mm_s, guard,
                                                 trace_flags);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }
  reason = f413_search_step_drive_accel_distance((float)DIST_HALF_SEC, params,
                                                 speed_now_mm_s, guard,
                                                 trace_flags);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  f413_search_step_advance_position();
  return f413_search_step_read_and_write_current_wall(false);
}

static f413_run_session_abort_reason_t f413_search_step_run_forward_section(
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    bool* acceled,
    bool known_straight,
    bool next_is_turn90)
{
  f413_run_session_abort_reason_t reason;
  bool wall_end_found = false;
  const uint16_t trace_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                         g_config.trace_motor_fwd_flag);

  if ((acceled != NULL) && known_straight && !*acceled)
  {
    reason = f413_search_step_drive_accel_distance((float)(DIST_HALF_SEC * 2),
                                                   params,
                                                   speed_now_mm_s,
                                                   guard,
                                                   trace_flags);
    if (reason == F413_RUN_SESSION_ABORT_NONE)
    {
      *acceled = true;
    }
    return reason;
  }

  if ((acceled != NULL) && !known_straight && *acceled)
  {
    if (next_is_turn90)
    {
      reason = f413_search_step_drive_decel_distance((float)DIST_HALF_SEC,
                                                     params,
                                                     speed_now_mm_s,
                                                     guard,
                                                     trace_flags);
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        return reason;
      }
      reason = f413_search_step_drive_wallend_segment((float)DIST_HALF_SEC,
                                                      (speed_now_mm_s != NULL) ? *speed_now_mm_s : 0.0f,
                                                      speed_now_mm_s,
                                                      guard,
                                                      trace_flags,
                                                      &wall_end_found);
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        return reason;
      }
      if ((params != NULL) && wall_end_found && (params->dist_wall_end > 0.0f))
      {
        reason = f413_search_step_drive_segment(params->dist_wall_end,
                                                (speed_now_mm_s != NULL) ? *speed_now_mm_s : 0.0f,
                                                speed_now_mm_s,
                                                guard,
                                                trace_flags);
      }
    }
    else
    {
      reason = f413_search_step_drive_decel_distance((float)(DIST_HALF_SEC * 2),
                                                     params,
                                                     speed_now_mm_s,
                                                     guard,
                                                     trace_flags);
    }
    if (reason == F413_RUN_SESSION_ABORT_NONE)
    {
      *acceled = false;
    }
    return reason;
  }

  reason = f413_search_step_drive_wallend_segment(
      (float)(DIST_HALF_SEC * 2),
      (speed_now_mm_s != NULL) ? *speed_now_mm_s : 0.0f,
      speed_now_mm_s,
      guard,
      trace_flags,
      &wall_end_found);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }
  if ((params != NULL) && wall_end_found && (params->dist_wall_end > 0.0f))
  {
    return f413_search_step_drive_segment(params->dist_wall_end,
                                          (speed_now_mm_s != NULL) ? *speed_now_mm_s : 0.0f,
                                          speed_now_mm_s,
                                          guard,
                                          trace_flags);
  }
  return F413_RUN_SESSION_ABORT_NONE;
}

static f413_run_session_abort_reason_t f413_search_step_run_smooth_turn(
    uint8_t next_rel,
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard)
{
  const uint16_t straight_trace_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                                  g_config.trace_motor_fwd_flag);
  const uint16_t turn_trace_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                              g_config.trace_motor_rev_flag);
  f413_search_step_smooth_turn_t profile;
  f413_run_session_abort_reason_t reason;
  uint32_t start_ms;
  float signed_angle;
  float angle_90;
  float entry_speed;
  int8_t turn_sign;

  if ((params == NULL) || (speed_now_mm_s == NULL))
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  angle_90 = f413_run_features_angle_accum_mode() ? 90.0f : params->angle_turn_90;
  signed_angle = (next_rel == 1U) ? -fabsf(angle_90) : fabsf(angle_90);
  if (fabsf(signed_angle) <= 0.0f)
  {
    signed_angle = (next_rel == 1U) ? -90.0f : 90.0f;
  }
  profile = f413_search_step_build_smooth_turn(signed_angle, params->alpha_turn90);
  if (profile.t_total_s <= 0.0f)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  entry_speed = *speed_now_mm_s;
  reason = f413_search_step_drive_front_wall_entry_segment(params->dist_offset_in,
                                                           params->velocity_turn90,
                                                           params->val_offset_in,
                                                           speed_now_mm_s,
                                                           guard,
                                                           straight_trace_flags);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  turn_sign = (signed_angle < 0.0f) ? -1 : 1;
  f413_search_step_prepare_turn_angle_control();
  f413_ctrl_set_velocity(params->velocity_turn90);
  f413_ctrl_start_omega_profile((float)turn_sign * profile.omega_peak_deg_s,
                                profile.t_acc_s,
                                profile.t_cruise_s);
  start_ms = f413_search_step_tick();
  while (1)
  {
    const float t_s = (float)(f413_search_step_tick() - start_ms) * 0.001f;
    if (t_s >= profile.t_total_s)
    {
      break;
    }
    f413_search_step_set_mode_flags(turn_trace_flags);
    f413_ctrl_set_velocity(params->velocity_turn90);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      f413_ctrl_stop_omega_profile();
      return reason;
    }
  }
  f413_ctrl_stop_omega_profile();
  *speed_now_mm_s = params->velocity_turn90;

  return f413_search_step_drive_segment(params->dist_offset_out,
                                        entry_speed,
                                        speed_now_mm_s,
                                        guard,
                                        straight_trace_flags);
}

static f413_run_session_abort_reason_t f413_search_step_run_back_turn(
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard)
{
  f413_run_session_abort_reason_t reason;
  const uint16_t fwd_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                       g_config.trace_motor_fwd_flag);

  reason = f413_search_step_drive_segment((float)DIST_HALF_SEC, 0.0f,
                                          speed_now_mm_s, guard, fwd_flags);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  f413_wall_runtime_control_clear();
  f413_ctrl_reset_angle();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  f413_ctrl_set_angle_target(2.0f * g_config.step_turn_deg);
  reason = f413_search_step_wait_ctrl_target(2.0f * g_config.step_turn_deg,
                                             true,
                                             guard,
                                             (uint16_t)(g_config.trace_search_safe_flag |
                                                        g_config.trace_motor_rev_flag));
  f413_ctrl_clear_angle_target();
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  return f413_search_step_drive_accel_distance((float)DIST_HALF_SEC, params,
                                               speed_now_mm_s, guard,
                                               fwd_flags);
}

static f413_run_session_abort_reason_t f413_search_step_run_search_motion(
    uint8_t next_rel,
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    bool* acceled,
    bool known_straight,
    bool next_is_turn90)
{
  switch (next_rel)
  {
    case 0U:
      return f413_search_step_run_forward_section(params, speed_now_mm_s, guard,
                                                 acceled, known_straight,
                                                 next_is_turn90);
    case 1U:
    case 3U:
      if (acceled != NULL)
      {
        *acceled = false;
      }
      return f413_search_step_run_smooth_turn(next_rel, params, speed_now_mm_s, guard);
    case 2U:
      if (acceled != NULL)
      {
        *acceled = false;
      }
      return f413_search_step_run_back_turn(params, speed_now_mm_s, guard);
    default:
      return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
}

static f413_run_session_abort_reason_t f413_search_step_run_final_stop(
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard)
{
  if ((speed_now_mm_s == NULL) || (fabsf(*speed_now_mm_s) <= 1.0f))
  {
    return F413_RUN_SESSION_ABORT_NONE;
  }
  return f413_search_step_drive_segment((float)DIST_HALF_SEC, 0.0f,
                                        speed_now_mm_s,
                                        guard,
                                        (uint16_t)(g_config.trace_search_safe_flag |
                                                   g_config.trace_motor_fwd_flag));
}

static f413_run_session_abort_reason_t f413_search_step_wait_stop_tail(
    f413_run_session_guard_t* guard)
{
  f413_ctrl_stop_omega_profile();
  f413_ctrl_clear_angle_target();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  if (g_config.path_coast_ms == 0U)
  {
    return F413_RUN_SESSION_ABORT_NONE;
  }

  f413_search_step_set_mode_flags((uint16_t)(g_config.trace_search_safe_flag |
                                             g_config.trace_motor_coast_flag));
  return f413_run_session_wait_with_auto_step_guarded(g_config.path_coast_ms,
                                                      guard);
}

void f413_search_step_run_case0_test_once(
    uint8_t sub,
    const f413_search_step_case0_test_config_t* test_config)
{
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};
  const SearchRunParams_t* params;
  float speed_now_mm_s = 0.0f;
  bool trace_started = false;
  bool completed = false;
  const uint16_t fwd_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                       g_config.trace_motor_fwd_flag);

  if ((test_config == NULL) || (test_config->param_index >= 2U) ||
      (test_config->test_kind == F413_SEARCH_STEP_CASE0_TEST_NONE))
  {
    trace_printf("[SEARCH-TEST] no-op: unsupported mode1 case0 sub%u\r\n",
                 (unsigned int)sub);
    return;
  }
  if (f413_search_step_trace_auto_is_enabled())
  {
    trace_printf("[SEARCH-TEST] busy(auto already running)\r\n");
    return;
  }
  if (f413_search_step_stop_switch_pressed())
  {
    trace_printf("[SEARCH-TEST] canceled(start switch pressed)\r\n");
    return;
  }
  if (!f413_run_session_guard_prepare(&guard))
  {
    trace_printf("[SEARCH-TEST] canceled(guard init fail)\r\n");
    return;
  }

  params = &searchRunParams[test_config->param_index];
  f413_run_features_set(&test_config->features);
  f413_wall_runtime_set_control_gains(params->kp_wall, 0.0f);
  f413_search_step_map_init_empty();
  f413_search_step_angle_reset_streak_clear();
  mouse.x = START_X;
  mouse.y = START_Y;
  mouse.dir = 0U;
  g_session_active = true;

  trace_printf("[SEARCH-TEST] start mode1 case0 sub%u %s param=%u v_turn=%.0f alpha=%.0f in=%.1f out=%.1f\r\n",
               (unsigned int)sub,
               (test_config->label != NULL) ? test_config->label : "search-test",
               (unsigned int)test_config->param_index,
               (double)params->velocity_turn90,
               (double)params->alpha_turn90,
               (double)params->dist_offset_in,
               (double)params->dist_offset_out);

  f413_ctrl_start();
  f413_ctrl_reset_distance();
  f413_ctrl_reset_angle();
  f413_search_step_set_context(1U, 0U, sub, (uint8_t)'T');
  f413_search_step_trace_start();
  trace_started = true;

  if (test_config->test_kind == F413_SEARCH_STEP_CASE0_TEST_TURN_R90)
  {
    abort_reason = f413_search_step_drive_accel_distance((float)DIST_HALF_SEC,
                                                         params,
                                                         &speed_now_mm_s,
                                                         &guard,
                                                         fwd_flags);
    if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
    {
      abort_reason = f413_search_step_run_smooth_turn(1U,
                                                      params,
                                                      &speed_now_mm_s,
                                                      &guard);
    }
  }
  else if (test_config->test_kind == F413_SEARCH_STEP_CASE0_TEST_STRAIGHT_3)
  {
    abort_reason = f413_search_step_drive_accel_distance((float)DIST_FIRST_SEC,
                                                         params,
                                                         &speed_now_mm_s,
                                                         &guard,
                                                         fwd_flags);
    if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
    {
      abort_reason = f413_search_step_drive_segment((float)(DIST_HALF_SEC * 2),
                                                    speed_now_mm_s,
                                                    &speed_now_mm_s,
                                                    &guard,
                                                    fwd_flags);
    }
    if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
    {
      abort_reason = f413_search_step_drive_segment((float)(DIST_HALF_SEC * 2),
                                                    speed_now_mm_s,
                                                    &speed_now_mm_s,
                                                    &guard,
                                                    fwd_flags);
    }
  }
  else
  {
    abort_reason = F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    abort_reason = f413_search_step_run_final_stop(&speed_now_mm_s, &guard);
  }
  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    abort_reason = f413_search_step_wait_stop_tail(&guard);
  }
  completed = (abort_reason == F413_RUN_SESSION_ABORT_NONE);

  f413_ctrl_stop();
  if (trace_started && (abort_reason != F413_RUN_SESSION_ABORT_NONE))
  {
    f413_search_step_set_mode_flags((uint16_t)(g_config.trace_search_safe_flag |
        f413_run_session_abort_reason_to_trace_flag(abort_reason)));
    f413_search_step_trace_auto_step();
  }
  if (trace_started)
  {
    f413_search_step_set_mode_flags(0U);
    f413_search_step_trace_stop();
  }
  f413_run_session_guard_cleanup(&guard);
  f413_run_features_reset();

  trace_printf("[SEARCH-TEST] %s mode1 case0 sub%u dist=%.0fmm angle=%.0fdeg%s%s\r\n",
               completed ? "OK" : "STOP",
               (unsigned int)sub,
               (double)f413_ctrl_get_distance(),
               (double)f413_ctrl_get_angle(),
               (abort_reason != F413_RUN_SESSION_ABORT_NONE) ? " abort=" : "",
               (abort_reason != F413_RUN_SESSION_ABORT_NONE)
                   ? f413_run_session_abort_reason_to_text(abort_reason)
                   : "");
}

void f413_search_step_config(const f413_search_step_config_t* config)
{
  if (config != NULL)
  {
    g_config = *config;
  }
}

void f413_search_step_session_reset(void)
{
  g_session_active = false;
  f413_search_step_angle_reset_streak_clear();
  mouse.x = START_X;
  mouse.y = START_Y;
  mouse.dir = 0U;
}

void f413_search_step_run_config_once(uint8_t op_case,
                                      const f413_search_step_case_config_t* case_config)
{
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};
  const SearchRunParams_t* params;
  float speed_now_mm_s = 0.0f;
  uint16_t action_count = 0U;
  uint8_t phase_index;
  bool completed = false;
  bool route_failed = false;
  bool trace_started = false;
  bool acceled = false;
  HAL_StatusTypeDef save_status = HAL_OK;

  if ((case_config == NULL) || (case_config->phase_count == 0U) ||
      (case_config->phase_count > 2U) || (case_config->param_index >= 2U))
  {
    trace_printf("[SEARCH-RUN] no-op: unsupported case%u\r\n", (unsigned int)op_case);
    return;
  }
  if (f413_search_step_trace_auto_is_enabled())
  {
    trace_printf("[SEARCH-RUN] busy(auto already running)\r\n");
    return;
  }
  if (f413_search_step_stop_switch_pressed())
  {
    trace_printf("[SEARCH-RUN] canceled(start switch pressed)\r\n");
    return;
  }
  if (!f413_run_session_guard_prepare(&guard))
  {
    trace_printf("[SEARCH-RUN] canceled(guard init fail)\r\n");
    return;
  }

  params = &searchRunParams[case_config->param_index];
  f413_run_features_set(&case_config->features);
  f413_wall_runtime_set_control_gains(params->kp_wall, 0.0f);
  f413_search_step_map_init_empty();
  f413_search_step_angle_reset_streak_clear();
  mouse.x = START_X;
  mouse.y = START_Y;
  mouse.dir = 0U;
  g_session_active = true;

  trace_printf("[SEARCH-RUN] start mode1 case%u param=%u v=%.0f target=%.0fmm phases=",
               (unsigned int)op_case,
               (unsigned int)case_config->param_index,
               (double)params->velocity_turn90,
               (double)g_config.step_target_mm);
  for (phase_index = 0U; phase_index < case_config->phase_count; phase_index++)
  {
    trace_printf("%s%s",
                 f413_search_step_target_name(case_config->phases[phase_index]),
                 ((phase_index + 1U) < case_config->phase_count) ? "," : "");
  }
  trace_printf("\r\n");

  f413_ctrl_start();
  f413_ctrl_reset_distance();
  f413_ctrl_reset_angle();

  abort_reason = f413_search_step_read_and_write_current_wall(true);
  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    f413_search_step_set_context(1U, op_case, 0xFFU, (uint8_t)'S');
    f413_search_step_trace_start();
    trace_started = true;
    abort_reason = f413_search_step_run_entry_section(op_case, params,
                                                      &speed_now_mm_s,
                                                      &guard);
    if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
    {
      action_count++;
    }
  }

  for (phase_index = 0U; phase_index < case_config->phase_count; phase_index++)
  {
    const uint8_t target = case_config->phases[phase_index];
    bool phase_done = false;

    trace_printf("[SEARCH-RUN] phase%u target=%s pos=(%u,%u,%u)\r\n",
                 (unsigned int)phase_index,
                 f413_search_step_target_name(target),
                 (unsigned int)mouse.x,
                 (unsigned int)mouse.y,
                 (unsigned int)mouse.dir);

    while ((abort_reason == F413_RUN_SESSION_ABORT_NONE) && !phase_done)
    {
      uint8_t next_rel = 0U;
      uint8_t next_after_forward = 0xFFU;
      bool known_straight = false;
      bool next_is_turn90 = false;
      int step;

      if (action_count >= F413_SEARCH_STEP_AUTO_MAX_ACTIONS)
      {
        trace_printf("[SEARCH-RUN] stop(max actions=%u)\r\n",
                     (unsigned int)F413_SEARCH_STEP_AUTO_MAX_ACTIONS);
        route_failed = true;
        break;
      }
      if (f413_search_step_stop_switch_pressed())
      {
        abort_reason = F413_RUN_SESSION_ABORT_SWITCH;
        break;
      }

      if (f413_search_step_target_reached(target))
      {
        trace_printf("[SEARCH-RUN] phase%u reached target=%s pos=(%u,%u,%u)\r\n",
                     (unsigned int)phase_index,
                     f413_search_step_target_name(target),
                     (unsigned int)mouse.x,
                     (unsigned int)mouse.y,
                     (unsigned int)mouse.dir);
        phase_done = true;
        break;
      }

      step = f413_search_step_make_smap((uint8_t)mouse.x, (uint8_t)mouse.y, target);
      if (step == -2)
      {
        trace_printf("[SEARCH-RUN] phase%u full complete pos=(%u,%u,%u)\r\n",
                     (unsigned int)phase_index,
                     (unsigned int)mouse.x,
                     (unsigned int)mouse.y,
                     (unsigned int)mouse.dir);
        phase_done = true;
        break;
      }
      if ((step < 0) ||
          !f413_search_step_choose_next_relative((uint8_t)mouse.x,
                                                 (uint8_t)mouse.y,
                                                 (uint8_t)mouse.dir,
                                                 &next_rel))
      {
        trace_printf("[SEARCH-RUN] FAIL(no route) phase=%u target=%s pos=(%u,%u,%u) wall=0x%04X cell=0x%04X step=%d\r\n",
                     (unsigned int)phase_index,
                     f413_search_step_target_name(target),
                     (unsigned int)mouse.x,
                     (unsigned int)mouse.y,
                     (unsigned int)mouse.dir,
                     (unsigned int)wall_info,
                     (unsigned int)map[mouse.y][mouse.x],
                     step);
        route_failed = true;
        break;
      }

      f413_search_step_set_action_context(op_case,
                                          (uint8_t)mouse.x,
                                          (uint8_t)mouse.y,
                                          (uint8_t)mouse.dir,
                                          next_rel);
      if (next_rel == 0U)
      {
        known_straight = f413_search_step_forward_is_known_straight((uint8_t)mouse.x,
                                                                    (uint8_t)mouse.y,
                                                                    (uint8_t)mouse.dir,
                                                                    &next_after_forward);
        next_is_turn90 = (next_after_forward == 1U) || (next_after_forward == 3U);
      }
      trace_printf("[SEARCH-RUN] action%u phase=%u next=%s(%u) pos=(%u,%u,%u) smap=%d wall=0x%04X cell=0x%04X\r\n",
                   (unsigned int)action_count,
                   (unsigned int)phase_index,
                   f413_search_step_relative_name(next_rel),
                   (unsigned int)next_rel,
                   (unsigned int)mouse.x,
                   (unsigned int)mouse.y,
                   (unsigned int)mouse.dir,
                   step,
                   (unsigned int)wall_info,
                   (unsigned int)map[mouse.y][mouse.x]);

      abort_reason = f413_search_step_run_search_motion(next_rel,
                                                        params,
                                                        &speed_now_mm_s,
                                                        &guard,
                                                        &acceled,
                                                        known_straight,
                                                        next_is_turn90);
      if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }

      mouse.dir = (uint8_t)((mouse.dir + next_rel) & 0x03U);
      f413_search_step_advance_position();
      abort_reason = f413_search_step_read_and_write_current_wall(false);
      if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }
      if (next_rel == 0U)
      {
        f413_search_step_angle_reset_streak_update();
      }
      else
      {
        f413_search_step_angle_reset_streak_clear();
      }
      action_count++;
    }

    if ((abort_reason != F413_RUN_SESSION_ABORT_NONE) || route_failed)
    {
      break;
    }
  }

  completed = (abort_reason == F413_RUN_SESSION_ABORT_NONE) && !route_failed;
  if (completed)
  {
    abort_reason = f413_search_step_run_final_stop(&speed_now_mm_s, &guard);
    completed = (abort_reason == F413_RUN_SESSION_ABORT_NONE);
  }
  if (completed)
  {
    abort_reason = f413_search_step_wait_stop_tail(&guard);
    completed = (abort_reason == F413_RUN_SESSION_ABORT_NONE);
  }

  f413_ctrl_stop();
  if (trace_started && (abort_reason != F413_RUN_SESSION_ABORT_NONE))
  {
    f413_search_step_set_mode_flags((uint16_t)(g_config.trace_search_safe_flag |
        f413_run_session_abort_reason_to_trace_flag(abort_reason)));
    f413_search_step_trace_auto_step();
  }
  if (trace_started)
  {
    f413_search_step_set_mode_flags(0U);
    f413_search_step_trace_stop();
  }
  f413_run_session_guard_cleanup(&guard);

  save_status = nvm_maze_save_map(&map[0][0], F413_SEARCH_STEP_CELL_COUNT);
  trace_printf("[SEARCH-RUN] %s case%u actions=%u pos=(%u,%u,%u) save=%s%s%s\r\n",
               completed ? "OK" : "STOP",
               (unsigned int)op_case,
               (unsigned int)action_count,
               (unsigned int)mouse.x,
               (unsigned int)mouse.y,
               (unsigned int)mouse.dir,
               (save_status == HAL_OK) ? "OK" : "FAIL",
               (abort_reason != F413_RUN_SESSION_ABORT_NONE) ? " abort=" : "",
               (abort_reason != F413_RUN_SESSION_ABORT_NONE)
                   ? f413_run_session_abort_reason_to_text(abort_reason)
                   : "");
}

void f413_search_step_run_search_case_once(uint8_t op_case)
{
  f413_search_step_case_config_t config;
  const f413_run_features_t default_features = {
    .wall_control_enabled = true,
    .wall_end_correction_enabled = true,
    .front_wall_correction_enabled = true,
    .angle_accum_mode = true,
    .test_mode_run = false,
  };

  if (!f413_search_step_plan_from_case(op_case, &config))
  {
    trace_printf("[SEARCH-RUN] no-op: unsupported case%u\r\n", (unsigned int)op_case);
    return;
  }
  config.label = "mode1 fallback";
  config.features = default_features;
  if ((op_case == 1U) || (op_case == 3U) || (op_case == 4U))
  {
    config.features.wall_end_correction_enabled = false;
  }
  f413_search_step_run_config_once(op_case, &config);
}

void f413_search_step_run_decision_preview_once(void)
{
  f413_wall_sensor_snapshot_t wall;
  uint8_t next_rel = 0U;
  int step;
  HAL_StatusTypeDef st;

  if (!f413_search_step_read_wall(&wall))
  {
    trace_printf("[SEARCH-PREVIEW] FAIL(read wall snapshot)\r\n");
    return;
  }

  f413_search_step_map_init_empty();
  mouse.x = START_X;
  mouse.y = START_Y;
  mouse.dir = 0U;
  wall_info = f413_search_step_wall_info_from_snapshot(&wall);
  f413_search_step_write_map_cell((uint8_t)mouse.x, (uint8_t)mouse.y, (uint8_t)mouse.dir, wall_info);
  step = f413_search_step_make_goal_smap((uint8_t)mouse.x, (uint8_t)mouse.y);

  if ((step < 0) || !f413_search_step_choose_next_relative((uint8_t)mouse.x, (uint8_t)mouse.y, (uint8_t)mouse.dir, &next_rel))
  {
    trace_printf("[SEARCH-PREVIEW] FAIL(no route) wall_info=0x%04X cell=0x%04X\r\n",
                 (unsigned int)wall_info,
                 (unsigned int)map[START_Y][START_X]);
    return;
  }

  st = nvm_maze_save_map(&map[0][0], F413_SEARCH_STEP_CELL_COUNT);
  trace_printf("[SEARCH-PREVIEW] next=%s(%u) smap=%d save=%s wall_info=0x%04X cell=0x%04X dFR=%ld dR=%ld dFL=%ld dL=%ld\r\n",
               f413_search_step_relative_name(next_rel),
               (unsigned int)next_rel,
               step,
               (st == HAL_OK) ? "OK" : "FAIL",
               (unsigned int)wall_info,
               (unsigned int)map[START_Y][START_X],
               (long)wall.fr_delta,
               (long)wall.r_delta,
               (long)wall.fl_delta,
               (long)wall.l_delta);
}

void f413_search_step_run_once(void)
{
  f413_wall_sensor_snapshot_t wall;
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};
  uint8_t next_rel = 0U;
  int step;
  HAL_StatusTypeDef st;
  bool is_turn;
  float turn_target_deg;

  if (f413_search_step_trace_auto_is_enabled())
  {
    trace_printf("[SEARCH-STEP] busy(auto already running)\r\n");
    return;
  }
  if (f413_search_step_stop_switch_pressed())
  {
    trace_printf("[SEARCH-STEP] canceled(start switch pressed)\r\n");
    return;
  }
  if (!f413_search_step_read_wall(&wall))
  {
    trace_printf("[SEARCH-STEP] FAIL(read wall snapshot)\r\n");
    return;
  }

  if (!g_session_active)
  {
    f413_search_step_map_init_empty();
    f413_search_step_angle_reset_streak_clear();
    mouse.x = START_X;
    mouse.y = START_Y;
    mouse.dir = 0U;
    g_session_active = true;
    trace_printf("[SEARCH-STEP] session start pos=(%u,%u,%u)\r\n",
                 (unsigned int)mouse.x,
                 (unsigned int)mouse.y,
                 (unsigned int)mouse.dir);
  }

  wall_info = f413_search_step_wall_info_from_snapshot(&wall);
  f413_search_step_write_map_cell((uint8_t)mouse.x, (uint8_t)mouse.y, (uint8_t)mouse.dir, wall_info);
  visited[mouse.y][mouse.x] = true;
  step = f413_search_step_make_goal_smap((uint8_t)mouse.x, (uint8_t)mouse.y);

  if ((step < 0) || !f413_search_step_choose_next_relative((uint8_t)mouse.x, (uint8_t)mouse.y, (uint8_t)mouse.dir, &next_rel))
  {
    trace_printf("[SEARCH-STEP] FAIL(no route) pos=(%u,%u,%u) wall_info=0x%04X cell=0x%04X\r\n",
                 (unsigned int)mouse.x,
                 (unsigned int)mouse.y,
                 (unsigned int)mouse.dir,
                 (unsigned int)wall_info,
                 (unsigned int)map[mouse.y][mouse.x]);
    return;
  }

  st = nvm_maze_save_map(&map[0][0], F413_SEARCH_STEP_CELL_COUNT);
  if (st != HAL_OK)
  {
    trace_printf("[SEARCH-STEP] FAIL(save HAL=%d)\r\n", (int)st);
    return;
  }
  if (!f413_run_session_guard_prepare(&guard))
  {
    trace_printf("[SEARCH-STEP] canceled(guard init fail)\r\n");
    return;
  }

  is_turn = (next_rel != 0U);
  turn_target_deg = f413_search_step_turn_target_deg(next_rel);
  trace_printf("[SEARCH-STEP] run next=%s(%u) pos=(%u,%u,%u) smap=%d wall_info=0x%04X cell=0x%04X\r\n",
               f413_search_step_relative_name(next_rel),
               (unsigned int)next_rel,
               (unsigned int)mouse.x,
               (unsigned int)mouse.y,
               (unsigned int)mouse.dir,
               step,
               (unsigned int)wall_info,
               (unsigned int)map[mouse.y][mouse.x]);

  f413_search_step_set_action_context(4U,
                                      (uint8_t)mouse.x,
                                      (uint8_t)mouse.y,
                                      (uint8_t)mouse.dir,
                                      next_rel);
  f413_search_step_trace_start();
  f413_ctrl_start();
  f413_ctrl_reset_distance();
  f413_ctrl_reset_angle();
  f413_wall_runtime_set_control_gains(searchRunParams[0].kp_wall, 0.0f);

  if (is_turn)
  {
    f413_wall_runtime_control_clear();
    f413_ctrl_set_velocity(0.0f);
    f413_ctrl_set_omega(0.0f);
    f413_ctrl_set_angle_target(turn_target_deg);
    abort_reason = f413_search_step_wait_ctrl_target(turn_target_deg, true, &guard,
        (uint16_t)(g_config.trace_search_safe_flag |
                   g_config.trace_motor_rev_flag));
  }
  else
  {
    f413_ctrl_set_velocity(g_config.step_velocity_mm_s);
    f413_ctrl_set_omega(0.0f);
    f413_ctrl_clear_angle_target();
    abort_reason = f413_search_step_wait_ctrl_target(g_config.step_target_mm, false, &guard,
        (uint16_t)(g_config.trace_search_safe_flag |
                   g_config.trace_motor_fwd_flag));
  }

  f413_ctrl_clear_angle_target();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    f413_search_step_set_mode_flags((uint16_t)(g_config.trace_search_safe_flag |
                                               g_config.trace_motor_coast_flag));
    abort_reason = f413_run_session_wait_with_auto_step_guarded(g_config.path_coast_ms, &guard);
  }
  f413_ctrl_stop();
  if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    f413_search_step_set_mode_flags((uint16_t)(g_config.trace_search_safe_flag |
        f413_run_session_abort_reason_to_trace_flag(abort_reason)));
    f413_search_step_trace_auto_step();
  }
  f413_search_step_set_mode_flags(0U);
  f413_search_step_trace_stop();
  f413_run_session_guard_cleanup(&guard);

  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    if (is_turn)
    {
      mouse.dir = (uint8_t)((mouse.dir + next_rel) & 0x03U);
    }
    else
    {
      f413_search_step_advance_position();
    }
    trace_printf("[SEARCH-STEP] OK next_pos=(%u,%u,%u) dist=%.0fmm angle=%.0fdeg\r\n",
                 (unsigned int)mouse.x,
                 (unsigned int)mouse.y,
                 (unsigned int)mouse.dir,
                 (double)f413_ctrl_get_distance(),
                 (double)f413_ctrl_get_angle());
  }
  else
  {
    trace_printf("[SEARCH-STEP] aborted(%s) pos=(%u,%u,%u) dist=%.0fmm angle=%.0fdeg\r\n",
                 f413_run_session_abort_reason_to_text(abort_reason),
                 (unsigned int)mouse.x,
                 (unsigned int)mouse.y,
                 (unsigned int)mouse.dir,
                 (double)f413_ctrl_get_distance(),
                 (double)f413_ctrl_get_angle());
  }
}

void f413_search_step_run_map_probe_once(void)
{
  static uint16_t loaded[F413_SEARCH_STEP_CELL_COUNT];
  f413_wall_sensor_snapshot_t wall;
  HAL_StatusTypeDef st;
  uint32_t i;

  if (!f413_search_step_read_wall(&wall))
  {
    trace_printf("[SEARCH-MAP] FAIL(read wall snapshot)\r\n");
    return;
  }

  f413_search_step_map_init_empty();
  mouse.x = START_X;
  mouse.y = START_Y;
  mouse.dir = 0U;
  wall_info = f413_search_step_wall_info_from_snapshot(&wall);
  f413_search_step_write_map_cell((uint8_t)mouse.x, (uint8_t)mouse.y, (uint8_t)mouse.dir, wall_info);

  st = nvm_maze_save_map(&map[0][0], F413_SEARCH_STEP_CELL_COUNT);
  if (st != HAL_OK)
  {
    trace_printf("[SEARCH-MAP] FAIL(save HAL=%d)\r\n", (int)st);
    return;
  }

  memset(loaded, 0, sizeof(loaded));
  if (!nvm_maze_load_map(loaded, F413_SEARCH_STEP_CELL_COUNT))
  {
    trace_printf("[SEARCH-MAP] FAIL(load)\r\n");
    return;
  }
  for (i = 0U; i < F413_SEARCH_STEP_CELL_COUNT; i++)
  {
    if (loaded[i] != (&map[0][0])[i])
    {
      trace_printf("[SEARCH-MAP] FAIL(verify i=%lu got=0x%04X exp=0x%04X)\r\n",
                   (unsigned long)i,
                   (unsigned int)loaded[i],
                   (unsigned int)(&map[0][0])[i]);
      return;
    }
  }

  trace_printf("[SEARCH-MAP] PASS start=(%u,%u,%u) wall_info=0x%04X cell=0x%04X dFR=%ld dR=%ld dFL=%ld dL=%ld\r\n",
               (unsigned int)mouse.x,
               (unsigned int)mouse.y,
               (unsigned int)mouse.dir,
               (unsigned int)wall_info,
               (unsigned int)map[START_Y][START_X],
               (long)wall.fr_delta,
               (long)wall.r_delta,
               (long)wall.fl_delta,
               (long)wall.l_delta);
}

void f413_search_step_run_status_once(void)
{
  static uint16_t loaded[F413_SEARCH_STEP_CELL_COUNT];
  uint32_t visited_count = 0U;
  uint32_t current_known_count = 0U;
  uint32_t loaded_known_count = 0U;
  uint32_t mismatch_count = 0U;
  bool loaded_ok;
  uint8_t x;
  uint8_t y;

  loaded_ok = nvm_maze_load_map(loaded, F413_SEARCH_STEP_CELL_COUNT);
  for (y = 0U; y < MAZE_SIZE; y++)
  {
    for (x = 0U; x < MAZE_SIZE; x++)
    {
      uint32_t idx = (uint32_t)y * MAZE_SIZE + x;
      if (visited[y][x])
      {
        visited_count++;
      }
      if ((map[y][x] & F413_SEARCH_STEP_MAZE_WALL_KNOWN_MASK) != 0U)
      {
        current_known_count++;
      }
      if (loaded_ok)
      {
        if ((loaded[idx] & F413_SEARCH_STEP_MAZE_WALL_KNOWN_MASK) != 0U)
        {
          loaded_known_count++;
        }
        if (loaded[idx] != map[y][x])
        {
          mismatch_count++;
        }
      }
    }
  }

  trace_printf("[SEARCH-STATE] active=%u mouse=(%u,%u,%u) cell=0x%04X visited=%lu known=%lu fram=%s fram_known=%lu mismatch=%lu\r\n",
               (unsigned int)g_session_active,
               (unsigned int)mouse.x,
               (unsigned int)mouse.y,
               (unsigned int)mouse.dir,
               (unsigned int)(((mouse.x < MAZE_SIZE) && (mouse.y < MAZE_SIZE)) ? map[mouse.y][mouse.x] : 0U),
               (unsigned long)visited_count,
               (unsigned long)current_known_count,
               loaded_ok ? "OK" : "MISS",
               (unsigned long)loaded_known_count,
               (unsigned long)mismatch_count);
}

void f413_search_step_run_map_clear_once(void)
{
  HAL_StatusTypeDef st;

  f413_search_step_map_init_empty();
  f413_search_step_session_reset();
  st = nvm_maze_save_map(&map[0][0], F413_SEARCH_STEP_CELL_COUNT);
  trace_printf("[SEARCH-CLEAR] save=%s mouse=(%u,%u,%u) start_cell=0x%04X\r\n",
               (st == HAL_OK) ? "OK" : "FAIL",
               (unsigned int)mouse.x,
               (unsigned int)mouse.y,
               (unsigned int)mouse.dir,
               (unsigned int)map[START_Y][START_X]);
}

void f413_search_step_run_map_dump_once(void)
{
  static uint16_t loaded[F413_SEARCH_STEP_CELL_COUNT];
  const uint16_t* cells = &map[0][0];
  bool loaded_ok = nvm_maze_load_map(loaded, F413_SEARCH_STEP_CELL_COUNT);
  int y;
  uint8_t x;

  if (loaded_ok)
  {
    cells = loaded;
  }

  trace_printf("[SEARCH-DUMP] source=%s size=%u active=%u mouse=(%u,%u,%u)\r\n",
               loaded_ok ? "FRAM" : "RAM",
               (unsigned int)MAZE_SIZE,
               (unsigned int)g_session_active,
               (unsigned int)mouse.x,
               (unsigned int)mouse.y,
               (unsigned int)mouse.dir);
  for (y = (int)MAZE_SIZE - 1; y >= 0; y--)
  {
    trace_printf("[SEARCH-DUMP] y=%02d:", y);
    for (x = 0U; x < MAZE_SIZE; x++)
    {
      trace_printf("%02X", (unsigned int)(cells[(uint32_t)y * MAZE_SIZE + x] & 0xFFU));
    }
    trace_printf("\r\n");
  }
}
