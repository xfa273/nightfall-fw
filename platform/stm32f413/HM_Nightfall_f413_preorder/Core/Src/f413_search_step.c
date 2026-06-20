#include "f413_search_step.h"

#include <math.h>
#include <string.h>

#include "f413_control.h"
#include "f413_run_session.h"
#include "nvm_params.h"
#include "params.h"
#include "search.h"
#include "trace.h"

#define F413_SEARCH_STEP_MAZE_WALL_W (0x01U)
#define F413_SEARCH_STEP_MAZE_WALL_S (0x02U)
#define F413_SEARCH_STEP_MAZE_WALL_E (0x04U)
#define F413_SEARCH_STEP_MAZE_WALL_N (0x08U)
#define F413_SEARCH_STEP_MAZE_WALL_KNOWN_MASK (0x0FU)
#define F413_SEARCH_STEP_MAZE_START_FORCED_WALLS (0x07U)
#define F413_SEARCH_STEP_CELL_COUNT ((uint32_t)(MAZE_SIZE * MAZE_SIZE))

static f413_search_step_config_t g_config;
static bool g_session_active = false;

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

static void f413_search_step_set_action_context(uint8_t x,
                                                uint8_t y,
                                                uint8_t dir,
                                                uint8_t next_rel)
{
  uint8_t packed_xy = (uint8_t)(((y & 0x0FU) << 4U) | (x & 0x0FU));
  uint8_t packed_action = (uint8_t)(0x80U |
                                    ((next_rel & 0x03U) << 2U) |
                                    (dir & 0x03U));

  f413_search_step_set_context(1U, 4U, packed_xy, packed_action);
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

static int f413_search_step_make_goal_smap(uint8_t x, uint8_t y)
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
    reason = f413_run_session_wait_with_auto_step_guarded(10U, guard);
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
  mouse.x = START_X;
  mouse.y = START_Y;
  mouse.dir = 0U;
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

  f413_search_step_set_action_context((uint8_t)mouse.x,
                                      (uint8_t)mouse.y,
                                      (uint8_t)mouse.dir,
                                      next_rel);
  f413_search_step_trace_start();
  f413_ctrl_start();
  f413_ctrl_reset_distance();
  f413_ctrl_reset_angle();

  if (is_turn)
  {
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
    f413_ctrl_set_angle_target(0.0f);
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
