#include "f413_route_preview.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "f413_trace_log.h"
#include "legacy_path_codec.h"
#include "motion_time.h"
#include "nvm_params.h"
#include "params.h"
#include "shortest_run_params.h"
#include "trace.h"

#ifndef ROUTE_MAX_LEN
#define ROUTE_MAX_LEN (1024U)
#endif

extern uint16_t path[ROUTE_MAX_LEN];

/*
 * Fixed-memory 16x16 KERI #1--#5 planner shared by the read-only preview and
 * mode2 case6..9 path generation.  The preview may fall back to a pinned maze;
 * run-path generation requires the saved FRAM maze and compiled goals.  This
 * module never starts control/motors and never writes NVM.  Executable output
 * is additionally limited to nominal-speed turns and a cardinal stop tail so
 * it can be represented by the current legacy path runner without a sidecar.
 */

#define F413_RP_WIDTH (16U)
#define F413_RP_HEIGHT (16U)
#define F413_RP_CELL_COUNT (F413_RP_WIDTH * F413_RP_HEIGHT)
#define F413_RP_CENTER_POSES (F413_RP_CELL_COUNT * 4U)
#define F413_RP_INTERNAL_WALLS \
  (((F413_RP_WIDTH - 1U) * F413_RP_HEIGHT) + \
   (F413_RP_WIDTH * (F413_RP_HEIGHT - 1U)))
#define F413_RP_POSE_COUNT (F413_RP_CENTER_POSES + (F413_RP_INTERNAL_WALLS * 4U))
#define F413_RP_SPEED_COUNT (3U)
#define F413_RP_STATE_COUNT (F413_RP_POSE_COUNT * F413_RP_SPEED_COUNT)
#define F413_RP_SETTLED_BYTES ((F413_RP_STATE_COUNT + 7U) / 8U)
#define F413_RP_INF (UINT32_MAX)
#define F413_RP_HALF_CELL_MM (45.0)
#define F413_RP_DIAGONAL_COMMAND_MM (67.279)
#define F413_RP_PI (3.14159265358979323846264338327950288)
#define F413_RP_EPS (1.0e-8)
#define F413_RP_MAX_TURN_INTERVALS (384U)
#define F413_RP_SCRATCH_MIN_BYTES (180U * 1024U)
#define F413_RP_BUILTIN_DATA_REV \
  "762ed2b68735ea29148c6a1251a90ed0651ff26b"
#define F413_RP_PARENT_VALID (0x80000000UL)
#define F413_RP_PARENT_PREV_MASK (0x00003FFFUL)
#define F413_RP_PARENT_CONNECTOR_SHIFT (14U)
#define F413_RP_PARENT_CONNECTOR_MASK (0x000FC000UL)
#define F413_RP_PARENT_KIND_SHIFT (20U)
#define F413_RP_PARENT_KIND_MASK (0x00700000UL)
#define F413_RP_PARENT_LEFT (0x00800000UL)
#define F413_RP_PARENT_SPEED_SHIFT (24U)
#define F413_RP_PARENT_SPEED_MASK (0x03000000UL)

_Static_assert(MAZE_SIZE == F413_RP_WIDTH,
               "F413 compact route preview is fixed to a 16x16 maze");
_Static_assert(F413_RP_POSE_COUNT == 2944U,
               "compact KERI pose count changed");
_Static_assert(F413_RP_STATE_COUNT < (1U << 14U),
               "packed parent state field is too small");
_Static_assert(F413_RP_STATE_COUNT < UINT16_MAX,
               "indexed heap uses UINT16_MAX as its absent sentinel");
_Static_assert(F413_RP_SCRATCH_MIN_BYTES <= F413_TRACE_LOG_IDLE_SCRATCH_BYTES,
               "route preview does not fit the idle trace workspace");

typedef enum
{
  F413_RP_HEADING_NORTH = 0,
  F413_RP_HEADING_NORTH_EAST,
  F413_RP_HEADING_EAST,
  F413_RP_HEADING_SOUTH_EAST,
  F413_RP_HEADING_SOUTH,
  F413_RP_HEADING_SOUTH_WEST,
  F413_RP_HEADING_WEST,
  F413_RP_HEADING_NORTH_WEST,
} f413_rp_heading_t;

typedef enum
{
  F413_RP_KIND_LARGE_90 = 0,
  F413_RP_KIND_LARGE_180,
  F413_RP_KIND_45_IN,
  F413_RP_KIND_45_OUT,
  F413_RP_KIND_V90,
  F413_RP_KIND_135_IN,
  F413_RP_KIND_135_OUT,
  F413_RP_KIND_COUNT,
} f413_rp_kind_t;

typedef enum
{
  F413_RP_SPEED_NOMINAL = 0,
  F413_RP_SPEED_LOW,
  F413_RP_SPEED_CRAWL,
} f413_rp_speed_t;

typedef enum
{
  F413_RP_SIDE_RIGHT = 0,
  F413_RP_SIDE_LEFT,
} f413_rp_side_t;

typedef enum
{
  F413_RP_PLAN_OK = 0,
  F413_RP_PLAN_NO_PATH,
  F413_RP_PLAN_NO_FEASIBLE_TERMINAL,
  F413_RP_PLAN_ERROR,
} f413_rp_plan_status_t;

typedef enum
{
  F413_RP_MAZE_SOURCE_FRAM = 0,
  F413_RP_MAZE_SOURCE_BUILTIN_16MM2014CX,
} f413_rp_maze_source_t;

typedef struct
{
  int16_t half_x;
  int16_t half_y;
} f413_rp_anchor_t;

typedef struct
{
  f413_rp_anchor_t anchor;
  f413_rp_heading_t heading;
} f413_rp_pose_t;

typedef struct
{
  uint8_t* cursor;
  uint8_t* end;
} f413_rp_arena_t;

typedef struct
{
  NfTurnSpec timing[F413_RP_SPEED_COUNT][F413_RP_KIND_COUNT];
  NfTurnPlan timing_plan[F413_RP_SPEED_COUNT][F413_RP_KIND_COUNT];
  NfTurnSpec geometry[F413_RP_KIND_COUNT];
  NfTurnPlan geometry_plan[F413_RP_KIND_COUNT];
  NfTurnPose* geometry_poses[F413_RP_KIND_COUNT];
  uint16_t geometry_intervals[F413_RP_KIND_COUNT];
  NfLinearLimits orthogonal;
  NfLinearLimits diagonal;
  double speed_mm_s[F413_RP_SPEED_COUNT];
  uint32_t start_time_us;
} f413_rp_motion_t;

typedef struct
{
  uint8_t walls[F413_RP_CELL_COUNT];
  uint8_t goal_bits[F413_RP_CELL_COUNT / 8U];
  uint8_t goal_x[9U];
  uint8_t goal_y[9U];
  uint8_t goal_count;
  uint16_t mismatch_count;
} f413_rp_maze_t;

typedef struct
{
  bool valid;
  bool direct_connector;
  uint16_t source_state;
  uint16_t connector_steps;
  uint16_t goal_step;
  uint16_t stop_steps;
  f413_rp_kind_t kind;
  f413_rp_side_t side;
  f413_rp_speed_t speed;
  uint8_t goal_x;
  uint8_t goal_y;
  uint32_t connector_us;
  uint32_t turn_us;
  uint32_t goal_cross_edge_us;
  uint32_t stop_tail_us;
  uint32_t goal_entry_us;
  uint32_t stop_us;
  uint16_t turn_count;
} f413_rp_goal_t;

typedef struct
{
  f413_rp_maze_t* maze;
  f413_rp_motion_t* motion;
  uint32_t* distances;
  uint16_t* turn_counts;
  uint32_t* parents;
  uint8_t* settled;
  uint16_t* heap_states;
  uint16_t* heap_positions;
  uint16_t heap_size;
  uint16_t heap_peak;
  uint32_t expanded_states;
  uint32_t relaxed_edges;
  bool goal_cross_reachable;
  bool execution_compatible;
  f413_rp_goal_t goal;
} f413_rp_context_t;

typedef struct
{
  bool present;
  uint16_t step;
  uint8_t x;
  uint8_t y;
} f413_rp_ray_goal_t;

typedef struct
{
  uint16_t stop_steps;
  f413_rp_anchor_t stop_anchor;
  f413_rp_ray_goal_t first_goal;
} f413_rp_ray_t;

typedef struct
{
  bool feasible;
  bool has_goal;
  double goal_geometry_fraction;
  uint8_t goal_x;
  uint8_t goal_y;
} f413_rp_turn_trace_t;

static const int8_t g_f413_rp_dx[8] = {0, 1, 1, 1, 0, -1, -1, -1};
static const int8_t g_f413_rp_dy[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const uint8_t g_f413_rp_wall_mask[4] = {0x08U, 0x04U, 0x02U, 0x01U};

/*
 * 16MM2014CX.maze, pinned from micromouse-maze-data revision
 * F413_RP_BUILTIN_DATA_REV.  One NESW wall nibble per cell, ordered y=0..15
 * then x=0..15.  static const keeps the 256-byte fallback in program Flash.
 */
static const uint8_t g_f413_rp_builtin_16mm2014cx_walls[] = {
  0x7U, 0x3U, 0x2U, 0x2U, 0xEU, 0x3U, 0xAU, 0xAU,
  0xAU, 0xAU, 0xAU, 0xAU, 0xAU, 0x6U, 0x3U, 0x6U, /* y=0 */
  0x5U, 0x5U, 0x5U, 0x9U, 0x2U, 0xCU, 0x3U, 0xAU,
  0x6U, 0x7U, 0x3U, 0x6U, 0x3U, 0x4U, 0x5U, 0x5U, /* y=1 */
  0x5U, 0x5U, 0x5U, 0x7U, 0xDU, 0xBU, 0x0U, 0xEU,
  0x9U, 0x0U, 0xCU, 0x1U, 0x4U, 0x5U, 0x5U, 0x5U, /* y=2 */
  0x5U, 0x5U, 0x5U, 0x1U, 0x2U, 0xEU, 0x9U, 0x2U,
  0xEU, 0x9U, 0x2U, 0xCU, 0x9U, 0x4U, 0x5U, 0x5U, /* y=3 */
  0x5U, 0x5U, 0x5U, 0x5U, 0x1U, 0x2U, 0xEU, 0x9U,
  0x2U, 0xEU, 0x9U, 0x6U, 0x3U, 0xCU, 0x5U, 0x5U, /* y=4 */
  0x5U, 0x5U, 0x5U, 0x5U, 0xDU, 0x9U, 0x2U, 0xEU,
  0x9U, 0x2U, 0xEU, 0x9U, 0x0U, 0xEU, 0x5U, 0x5U, /* y=5 */
  0x5U, 0x5U, 0x5U, 0x5U, 0x3U, 0x6U, 0x9U, 0x2U,
  0xEU, 0x9U, 0x2U, 0xEU, 0x9U, 0x2U, 0xCU, 0x5U, /* y=6 */
  0x5U, 0x5U, 0x5U, 0x5U, 0x5U, 0x9U, 0x6U, 0x1U,
  0x6U, 0x7U, 0x9U, 0x2U, 0xEU, 0xDU, 0x3U, 0xCU, /* y=7 */
  0x5U, 0x5U, 0x5U, 0x9U, 0xCU, 0xBU, 0x4U, 0x9U,
  0x8U, 0x4U, 0x7U, 0x9U, 0x2U, 0xAU, 0x0U, 0xEU, /* y=8 */
  0x5U, 0x5U, 0x5U, 0x3U, 0xAU, 0xAU, 0x0U, 0x2U,
  0x6U, 0x9U, 0x4U, 0x7U, 0x9U, 0x2U, 0x8U, 0x6U, /* y=9 */
  0x5U, 0x5U, 0x5U, 0x9U, 0x2U, 0xEU, 0x5U, 0xDU,
  0x9U, 0x6U, 0x9U, 0x4U, 0x7U, 0x1U, 0x6U, 0x5U, /* y=10 */
  0x5U, 0x5U, 0x5U, 0xBU, 0x0U, 0xEU, 0x5U, 0x3U,
  0xAU, 0xCU, 0xBU, 0x8U, 0x4U, 0xDU, 0x1U, 0x4U, /* y=11 */
  0x5U, 0x5U, 0x5U, 0xBU, 0x0U, 0xEU, 0x5U, 0x9U,
  0xAU, 0xAU, 0xAU, 0xAU, 0xCU, 0x3U, 0xCU, 0xDU, /* y=12 */
  0x5U, 0x5U, 0x5U, 0xBU, 0x0U, 0xEU, 0x9U, 0xAU,
  0xAU, 0xAU, 0xAU, 0xAU, 0xAU, 0xCU, 0x3U, 0x6U, /* y=13 */
  0x5U, 0x5U, 0x1U, 0xEU, 0x9U, 0xAU, 0xAU, 0xAU,
  0xAU, 0xAU, 0xAU, 0xAU, 0xAU, 0xAU, 0xCU, 0x5U, /* y=14 */
  0x9U, 0xCU, 0x9U, 0xAU, 0xAU, 0xAU, 0xAU, 0xAU,
  0xAU, 0xAU, 0xAU, 0xAU, 0xAU, 0xAU, 0xAU, 0xCU, /* y=15 */
};

_Static_assert(sizeof(g_f413_rp_builtin_16mm2014cx_walls) ==
                   F413_RP_CELL_COUNT,
               "built-in 16MM2014CX wall fixture must cover all cells");

static void* f413_rp_arena_alloc(f413_rp_arena_t* arena,
                                 size_t bytes,
                                 size_t alignment)
{
  uintptr_t current;
  uintptr_t aligned;

  if ((arena == NULL) || (alignment == 0U) ||
      ((alignment & (alignment - 1U)) != 0U))
  {
    return NULL;
  }
  current = (uintptr_t)arena->cursor;
  aligned = (current + alignment - 1U) & ~(uintptr_t)(alignment - 1U);
  if ((aligned > (uintptr_t)arena->end) ||
      (bytes > (size_t)((uintptr_t)arena->end - aligned)))
  {
    return NULL;
  }
  arena->cursor = (uint8_t*)(aligned + bytes);
  return (void*)aligned;
}

static bool f413_rp_is_cardinal(f413_rp_heading_t heading)
{
  return (((unsigned int)heading & 1U) == 0U);
}

static f413_rp_heading_t f413_rp_heading_add(f413_rp_heading_t heading,
                                              int delta)
{
  int result = ((int)heading + delta) % 8;
  if (result < 0)
  {
    result += 8;
  }
  return (f413_rp_heading_t)result;
}

static bool f413_rp_anchor_is_center(f413_rp_anchor_t anchor)
{
  return ((anchor.half_x & 1) != 0) && ((anchor.half_y & 1) != 0);
}

static bool f413_rp_cell_in_bounds(int x, int y)
{
  return (x >= 0) && (y >= 0) &&
         (x < (int)F413_RP_WIDTH) && (y < (int)F413_RP_HEIGHT);
}

static size_t f413_rp_cell_index(int x, int y)
{
  return ((size_t)y * F413_RP_WIDTH) + (size_t)x;
}

static bool f413_rp_goal_at(const f413_rp_maze_t* maze, int x, int y)
{
  size_t index;
  if ((maze == NULL) || !f413_rp_cell_in_bounds(x, y))
  {
    return false;
  }
  index = f413_rp_cell_index(x, y);
  return (maze->goal_bits[index >> 3U] & (uint8_t)(1U << (index & 7U))) != 0U;
}

static void f413_rp_goal_set(f413_rp_maze_t* maze, uint8_t x, uint8_t y)
{
  const size_t index = f413_rp_cell_index(x, y);
  maze->goal_bits[index >> 3U] |= (uint8_t)(1U << (index & 7U));
}

static bool f413_rp_cells_open(const f413_rp_maze_t* maze,
                               int from_x,
                               int from_y,
                               int to_x,
                               int to_y)
{
  unsigned int direction;
  unsigned int opposite;

  if ((maze == NULL) || !f413_rp_cell_in_bounds(from_x, from_y) ||
      !f413_rp_cell_in_bounds(to_x, to_y))
  {
    return false;
  }
  if ((to_x == from_x) && (to_y == from_y + 1))
  {
    direction = 0U;
  }
  else if ((to_x == from_x + 1) && (to_y == from_y))
  {
    direction = 1U;
  }
  else if ((to_x == from_x) && (to_y == from_y - 1))
  {
    direction = 2U;
  }
  else if ((to_x == from_x - 1) && (to_y == from_y))
  {
    direction = 3U;
  }
  else
  {
    return false;
  }
  opposite = (direction + 2U) & 3U;
  return (maze->walls[f413_rp_cell_index(from_x, from_y)] &
          g_f413_rp_wall_mask[direction]) == 0U &&
         (maze->walls[f413_rp_cell_index(to_x, to_y)] &
          g_f413_rp_wall_mask[opposite]) == 0U;
}

static bool f413_rp_anchor_id(f413_rp_anchor_t anchor, uint16_t* out_id)
{
  const int hx = anchor.half_x;
  const int hy = anchor.half_y;
  const uint16_t centers = F413_RP_CELL_COUNT;
  const uint16_t vertical = (F413_RP_WIDTH - 1U) * F413_RP_HEIGHT;

  if ((out_id == NULL) || (hx <= 0) || (hy <= 0) ||
      (hx >= (int)(2U * F413_RP_WIDTH)) ||
      (hy >= (int)(2U * F413_RP_HEIGHT)) ||
      (((hx & 1) == 0) && ((hy & 1) == 0)))
  {
    return false;
  }
  if (((hx & 1) != 0) && ((hy & 1) != 0))
  {
    const uint16_t x = (uint16_t)(hx - 1) / 2U;
    const uint16_t y = (uint16_t)(hy - 1) / 2U;
    *out_id = (uint16_t)((y * F413_RP_WIDTH) + x);
    return true;
  }
  if ((hx & 1) == 0)
  {
    const uint16_t x_line = (uint16_t)hx / 2U;
    const uint16_t y = (uint16_t)(hy - 1) / 2U;
    if ((x_line == 0U) || (x_line >= F413_RP_WIDTH))
    {
      return false;
    }
    *out_id = (uint16_t)(centers + (y * (F413_RP_WIDTH - 1U)) +
                         (x_line - 1U));
    return true;
  }
  {
    const uint16_t x = (uint16_t)(hx - 1) / 2U;
    const uint16_t y_line = (uint16_t)hy / 2U;
    if ((y_line == 0U) || (y_line >= F413_RP_HEIGHT))
    {
      return false;
    }
    *out_id = (uint16_t)(centers + vertical +
                         ((y_line - 1U) * F413_RP_WIDTH) + x);
  }
  return true;
}

static bool f413_rp_anchor_from_id(uint16_t id, f413_rp_anchor_t* out)
{
  const uint16_t centers = F413_RP_CELL_COUNT;
  const uint16_t vertical = (F413_RP_WIDTH - 1U) * F413_RP_HEIGHT;

  if ((out == NULL) || (id >= (F413_RP_CELL_COUNT + F413_RP_INTERNAL_WALLS)))
  {
    return false;
  }
  if (id < centers)
  {
    const uint16_t x = id % F413_RP_WIDTH;
    const uint16_t y = id / F413_RP_WIDTH;
    out->half_x = (int16_t)((2U * x) + 1U);
    out->half_y = (int16_t)((2U * y) + 1U);
    return true;
  }
  id = (uint16_t)(id - centers);
  if (id < vertical)
  {
    const uint16_t x_line = (id % (F413_RP_WIDTH - 1U)) + 1U;
    const uint16_t y = id / (F413_RP_WIDTH - 1U);
    out->half_x = (int16_t)(2U * x_line);
    out->half_y = (int16_t)((2U * y) + 1U);
    return true;
  }
  id = (uint16_t)(id - vertical);
  out->half_x = (int16_t)((2U * (id % F413_RP_WIDTH)) + 1U);
  out->half_y = (int16_t)(2U * ((id / F413_RP_WIDTH) + 1U));
  return true;
}

static bool f413_rp_anchor_open(const f413_rp_maze_t* maze,
                                f413_rp_anchor_t anchor)
{
  if (f413_rp_anchor_is_center(anchor))
  {
    return true;
  }
  if ((anchor.half_x & 1) == 0)
  {
    const int right_x = anchor.half_x / 2;
    const int y = (anchor.half_y - 1) / 2;
    return f413_rp_cells_open(maze, right_x - 1, y, right_x, y);
  }
  {
    const int x = (anchor.half_x - 1) / 2;
    const int upper_y = anchor.half_y / 2;
    return f413_rp_cells_open(maze, x, upper_y - 1, x, upper_y);
  }
}

static bool f413_rp_travel_pose_valid(const f413_rp_maze_t* maze,
                                      f413_rp_anchor_t anchor,
                                      f413_rp_heading_t heading)
{
  uint16_t ignored;
  if (((unsigned int)heading >= 8U) || !f413_rp_anchor_id(anchor, &ignored))
  {
    return false;
  }
  if (f413_rp_anchor_is_center(anchor))
  {
    return f413_rp_is_cardinal(heading);
  }
  if (!f413_rp_anchor_open(maze, anchor))
  {
    return false;
  }
  if (!f413_rp_is_cardinal(heading))
  {
    return true;
  }
  if ((anchor.half_x & 1) == 0)
  {
    return (heading == F413_RP_HEADING_EAST) ||
           (heading == F413_RP_HEADING_WEST);
  }
  return (heading == F413_RP_HEADING_NORTH) ||
         (heading == F413_RP_HEADING_SOUTH);
}

static bool f413_rp_state_pose_valid(const f413_rp_maze_t* maze,
                                     f413_rp_anchor_t anchor,
                                     f413_rp_heading_t heading)
{
  if (!f413_rp_travel_pose_valid(maze, anchor, heading))
  {
    return false;
  }
  return f413_rp_anchor_is_center(anchor) ?
         f413_rp_is_cardinal(heading) : !f413_rp_is_cardinal(heading);
}

static bool f413_rp_anchor_region(const f413_rp_maze_t* maze,
                                  f413_rp_anchor_t anchor,
                                  f413_rp_heading_t heading,
                                  int* out_x,
                                  int* out_y)
{
  int x;
  int y;
  if ((out_x == NULL) || (out_y == NULL) ||
      !f413_rp_travel_pose_valid(maze, anchor, heading))
  {
    return false;
  }
  if (f413_rp_anchor_is_center(anchor))
  {
    x = (anchor.half_x - 1) / 2;
    y = (anchor.half_y - 1) / 2;
  }
  else if ((anchor.half_x & 1) == 0)
  {
    const int line = anchor.half_x / 2;
    x = (g_f413_rp_dx[(unsigned int)heading] > 0) ? line : line - 1;
    y = (anchor.half_y - 1) / 2;
  }
  else
  {
    const int line = anchor.half_y / 2;
    x = (anchor.half_x - 1) / 2;
    y = (g_f413_rp_dy[(unsigned int)heading] > 0) ? line : line - 1;
  }
  if (!f413_rp_cell_in_bounds(x, y))
  {
    return false;
  }
  *out_x = x;
  *out_y = y;
  return true;
}

static bool f413_rp_advance_connector(const f413_rp_maze_t* maze,
                                      f413_rp_anchor_t current,
                                      f413_rp_heading_t heading,
                                      f413_rp_anchor_t* out)
{
  f413_rp_anchor_t next;
  uint16_t ignored;

  if ((out == NULL) || !f413_rp_travel_pose_valid(maze, current, heading))
  {
    return false;
  }
  next.half_x = (int16_t)(current.half_x + g_f413_rp_dx[(unsigned int)heading]);
  next.half_y = (int16_t)(current.half_y + g_f413_rp_dy[(unsigned int)heading]);
  if (!f413_rp_anchor_id(next, &ignored) ||
      !f413_rp_travel_pose_valid(maze, next, heading))
  {
    return false;
  }
  *out = next;
  return true;
}

static bool f413_rp_pose_index(f413_rp_anchor_t anchor,
                               f413_rp_heading_t heading,
                               uint16_t* out_pose)
{
  uint16_t anchor_id;
  if ((out_pose == NULL) || !f413_rp_anchor_id(anchor, &anchor_id))
  {
    return false;
  }
  if (anchor_id < F413_RP_CELL_COUNT)
  {
    if (!f413_rp_is_cardinal(heading))
    {
      return false;
    }
    *out_pose = (uint16_t)((anchor_id * 4U) + ((unsigned int)heading / 2U));
    return true;
  }
  if (f413_rp_is_cardinal(heading))
  {
    return false;
  }
  *out_pose = (uint16_t)(F413_RP_CENTER_POSES +
                         ((anchor_id - F413_RP_CELL_COUNT) * 4U) +
                         (((unsigned int)heading - 1U) / 2U));
  return true;
}

static bool f413_rp_pose_decode(uint16_t pose,
                                f413_rp_anchor_t* out_anchor,
                                f413_rp_heading_t* out_heading)
{
  uint16_t anchor_id;
  if ((out_anchor == NULL) || (out_heading == NULL) ||
      (pose >= F413_RP_POSE_COUNT))
  {
    return false;
  }
  if (pose < F413_RP_CENTER_POSES)
  {
    anchor_id = pose / 4U;
    *out_heading = (f413_rp_heading_t)((pose % 4U) * 2U);
  }
  else
  {
    const uint16_t wall_pose = (uint16_t)(pose - F413_RP_CENTER_POSES);
    anchor_id = (uint16_t)(F413_RP_CELL_COUNT + (wall_pose / 4U));
    *out_heading = (f413_rp_heading_t)(((wall_pose % 4U) * 2U) + 1U);
  }
  return f413_rp_anchor_from_id(anchor_id, out_anchor);
}

static uint16_t f413_rp_state_index(f413_rp_anchor_t anchor,
                                    f413_rp_heading_t heading,
                                    f413_rp_speed_t speed)
{
  uint16_t pose = 0U;
  (void)f413_rp_pose_index(anchor, heading, &pose);
  return (uint16_t)((pose * F413_RP_SPEED_COUNT) + (uint16_t)speed);
}

static bool f413_rp_state_decode(uint16_t state,
                                 f413_rp_anchor_t* out_anchor,
                                 f413_rp_heading_t* out_heading,
                                 f413_rp_speed_t* out_speed)
{
  if ((state >= F413_RP_STATE_COUNT) || (out_speed == NULL))
  {
    return false;
  }
  *out_speed = (f413_rp_speed_t)(state % F413_RP_SPEED_COUNT);
  return f413_rp_pose_decode((uint16_t)(state / F413_RP_SPEED_COUNT),
                             out_anchor, out_heading);
}

static f413_rp_pose_t f413_rp_keri_opposite(f413_rp_pose_t pose)
{
  pose.heading = f413_rp_heading_add(pose.heading, 4);
  return pose;
}

static bool f413_rp_same_pose(f413_rp_pose_t left, f413_rp_pose_t right)
{
  return (left.anchor.half_x == right.anchor.half_x) &&
         (left.anchor.half_y == right.anchor.half_y) &&
         (left.heading == right.heading);
}

/* Port of KERI StepMapSlalom::Index::next() in 45 mm half-grid units. */
static f413_rp_pose_t f413_rp_keri_next(f413_rp_pose_t current,
                                        f413_rp_heading_t next_heading)
{
  f413_rp_pose_t next = {current.anchor, next_heading};

  if (f413_rp_is_cardinal(current.heading))
  {
    if (f413_rp_is_cardinal(next_heading))
    {
      next.anchor.half_x = (int16_t)(next.anchor.half_x +
          (2 * g_f413_rp_dx[(unsigned int)next_heading]));
      next.anchor.half_y = (int16_t)(next.anchor.half_y +
          (2 * g_f413_rp_dy[(unsigned int)next_heading]));
    }
    else
    {
      next.anchor.half_x = (int16_t)(next.anchor.half_x +
          g_f413_rp_dx[(unsigned int)current.heading] +
          g_f413_rp_dx[(unsigned int)next_heading]);
      next.anchor.half_y = (int16_t)(next.anchor.half_y +
          g_f413_rp_dy[(unsigned int)current.heading] +
          g_f413_rp_dy[(unsigned int)next_heading]);
    }
    return next;
  }

  if (!f413_rp_is_cardinal(next_heading))
  {
    next.anchor.half_x = (int16_t)(next.anchor.half_x +
        g_f413_rp_dx[(unsigned int)next_heading]);
    next.anchor.half_y = (int16_t)(next.anchor.half_y +
        g_f413_rp_dy[(unsigned int)next_heading]);
    return next;
  }

  {
    const bool vertical_wall = ((current.anchor.half_x & 1) == 0);
    const int wall_x = vertical_wall ?
        (current.anchor.half_x / 2) - 1 : (current.anchor.half_x - 1) / 2;
    const int wall_y = vertical_wall ?
        (current.anchor.half_y - 1) / 2 : (current.anchor.half_y / 2) - 1;
    int cell_x = wall_x;
    int cell_y = wall_y;

    switch (current.heading)
    {
      case F413_RP_HEADING_NORTH_EAST:
        if ((next_heading == F413_RP_HEADING_EAST) ||
            (next_heading == F413_RP_HEADING_NORTH))
        {
          cell_x++;
          cell_y++;
        }
        else if (next_heading == F413_RP_HEADING_WEST)
        {
          cell_y++;
        }
        else
        {
          cell_x++;
        }
        break;
      case F413_RP_HEADING_NORTH_WEST:
        if (next_heading == F413_RP_HEADING_EAST)
        {
          cell_x++;
          cell_y++;
        }
        else if (next_heading == F413_RP_HEADING_NORTH)
        {
          cell_y++;
        }
        else if (next_heading == F413_RP_HEADING_WEST)
        {
          cell_x--;
          cell_y++;
        }
        else
        {
          cell_x--;
        }
        break;
      case F413_RP_HEADING_SOUTH_WEST:
        if (next_heading == F413_RP_HEADING_EAST)
        {
          cell_x++;
          cell_y--;
        }
        else if (next_heading == F413_RP_HEADING_NORTH)
        {
          cell_x--;
          cell_y++;
        }
        else if (next_heading == F413_RP_HEADING_WEST)
        {
          cell_x--;
        }
        else
        {
          cell_y--;
        }
        break;
      case F413_RP_HEADING_SOUTH_EAST:
        if (next_heading == F413_RP_HEADING_EAST)
        {
          cell_x++;
        }
        else if (next_heading == F413_RP_HEADING_NORTH)
        {
          cell_x++;
          cell_y++;
        }
        else if (next_heading == F413_RP_HEADING_WEST)
        {
          cell_y--;
        }
        else
        {
          cell_x++;
          cell_y--;
        }
        break;
      default:
        break;
    }
    next.anchor.half_x = (int16_t)((2 * cell_x) + 1);
    next.anchor.half_y = (int16_t)((2 * cell_y) + 1);
  }
  return next;
}

static bool f413_rp_keri_can_go(const f413_rp_maze_t* maze,
                                f413_rp_pose_t pose)
{
  f413_rp_anchor_t wall = pose.anchor;

  if (f413_rp_is_cardinal(pose.heading))
  {
    if (!f413_rp_anchor_is_center(pose.anchor))
    {
      return false;
    }
    wall.half_x = (int16_t)(wall.half_x +
        g_f413_rp_dx[(unsigned int)pose.heading]);
    wall.half_y = (int16_t)(wall.half_y +
        g_f413_rp_dy[(unsigned int)pose.heading]);
  }
  else if (f413_rp_anchor_is_center(pose.anchor))
  {
    return false;
  }
  return f413_rp_anchor_open(maze, wall);
}

static int f413_rp_keri_diag_to_cardinal_delta(f413_rp_pose_t pose)
{
  const bool vertical_wall = ((pose.anchor.half_x & 1) == 0);

  if ((pose.heading == F413_RP_HEADING_NORTH_EAST) ||
      (pose.heading == F413_RP_HEADING_SOUTH_WEST))
  {
    return vertical_wall ? -1 : 1;
  }
  return vertical_wall ? 1 : -1;
}

static bool f413_rp_keri_turn_guard(const f413_rp_maze_t* maze,
                                    f413_rp_anchor_t source,
                                    f413_rp_heading_t start_heading,
                                    f413_rp_kind_t kind,
                                    f413_rp_anchor_t destination,
                                    f413_rp_heading_t end_heading)
{
  const f413_rp_pose_t focus = f413_rp_keri_opposite(
      (f413_rp_pose_t){destination, end_heading});
  const f413_rp_pose_t wanted = f413_rp_keri_opposite(
      (f413_rp_pose_t){source, start_heading});

  if (f413_rp_is_cardinal(focus.heading))
  {
    static const int deltas[2] = {-1, 1};
    if (!f413_rp_keri_can_go(maze, focus))
    {
      return false;
    }
    for (size_t index = 0U; index < 2U; index++)
    {
      const int delta = deltas[index];
      const f413_rp_heading_t d45 =
          f413_rp_heading_add(focus.heading, delta);
      const f413_rp_heading_t d90 =
          f413_rp_heading_add(focus.heading, 2 * delta);
      const f413_rp_heading_t d135 =
          f413_rp_heading_add(focus.heading, 3 * delta);
      const f413_rp_heading_t d180 =
          f413_rp_heading_add(focus.heading, 4 * delta);
      const f413_rp_pose_t i45 = f413_rp_keri_next(focus, d45);
      f413_rp_pose_t v90;
      f413_rp_pose_t i135;

      if (!f413_rp_keri_can_go(maze, i45))
      {
        continue;
      }
      if (((kind == F413_RP_KIND_45_IN) ||
           (kind == F413_RP_KIND_45_OUT)) &&
          f413_rp_keri_can_go(maze, f413_rp_keri_next(i45, i45.heading)) &&
          f413_rp_same_pose(i45, wanted))
      {
        return true;
      }
      v90 = (f413_rp_pose_t){focus.anchor, d90};
      v90.anchor.half_x = (int16_t)(v90.anchor.half_x +
          (2 * g_f413_rp_dx[(unsigned int)focus.heading]) +
          (2 * g_f413_rp_dx[(unsigned int)d90]));
      v90.anchor.half_y = (int16_t)(v90.anchor.half_y +
          (2 * g_f413_rp_dy[(unsigned int)focus.heading]) +
          (2 * g_f413_rp_dy[(unsigned int)d90]));
      if ((kind == F413_RP_KIND_LARGE_90) &&
          f413_rp_same_pose(v90, wanted))
      {
        return true;
      }
      i135 = f413_rp_keri_next(i45, d135);
      if (!f413_rp_keri_can_go(maze, i135))
      {
        continue;
      }
      if (((kind == F413_RP_KIND_135_IN) ||
           (kind == F413_RP_KIND_135_OUT)) &&
          f413_rp_keri_can_go(maze, f413_rp_keri_next(i135, i135.heading)) &&
          f413_rp_same_pose(i135, wanted))
      {
        return true;
      }
      if (kind == F413_RP_KIND_LARGE_180)
      {
        f413_rp_pose_t v180 = v90;
        v180.heading = d180;
        v180.anchor.half_x = (int16_t)(v180.anchor.half_x +
            (2 * g_f413_rp_dx[(unsigned int)d180]));
        v180.anchor.half_y = (int16_t)(v180.anchor.half_y +
            (2 * g_f413_rp_dy[(unsigned int)d180]));
        if (f413_rp_same_pose(v180, wanted))
        {
          return true;
        }
      }
    }
    return false;
  }

  {
    const f413_rp_pose_t i_f = f413_rp_keri_next(focus, focus.heading);
    const int delta = f413_rp_keri_diag_to_cardinal_delta(focus);
    const f413_rp_heading_t d45 =
        f413_rp_heading_add(focus.heading, delta);
    const f413_rp_heading_t d90 =
        f413_rp_heading_add(focus.heading, 2 * delta);
    const f413_rp_heading_t d135 =
        f413_rp_heading_add(focus.heading, 3 * delta);
    f413_rp_pose_t i90;

    if (!f413_rp_keri_can_go(maze, i_f))
    {
      return false;
    }
    if (((kind == F413_RP_KIND_45_IN) ||
         (kind == F413_RP_KIND_45_OUT)) &&
        f413_rp_same_pose(f413_rp_keri_next(focus, d45), wanted))
    {
      return true;
    }
    i90 = f413_rp_keri_next(i_f, d90);
    if (!f413_rp_keri_can_go(maze, i90))
    {
      return false;
    }
    if ((kind == F413_RP_KIND_V90) &&
        f413_rp_keri_can_go(maze, f413_rp_keri_next(i90, i90.heading)) &&
        f413_rp_same_pose(i90, wanted))
    {
      return true;
    }
    return ((kind == F413_RP_KIND_135_IN) ||
            (kind == F413_RP_KIND_135_OUT)) &&
           f413_rp_same_pose(f413_rp_keri_next(focus, d135), wanted);
  }
}

static bool f413_rp_turn_source_valid(const f413_rp_maze_t* maze,
                                      f413_rp_anchor_t anchor,
                                      f413_rp_heading_t heading,
                                      f413_rp_kind_t kind)
{
  if (!f413_rp_state_pose_valid(maze, anchor, heading))
  {
    return false;
  }
  switch (kind)
  {
    case F413_RP_KIND_LARGE_90:
    case F413_RP_KIND_LARGE_180:
    case F413_RP_KIND_45_IN:
    case F413_RP_KIND_135_IN:
      return f413_rp_anchor_is_center(anchor) &&
             f413_rp_is_cardinal(heading);
    case F413_RP_KIND_45_OUT:
    case F413_RP_KIND_V90:
    case F413_RP_KIND_135_OUT:
      return !f413_rp_anchor_is_center(anchor) &&
             !f413_rp_is_cardinal(heading);
    default:
      return false;
  }
}

static bool f413_rp_turn_destination(const f413_rp_maze_t* maze,
                                     f413_rp_anchor_t source,
                                     f413_rp_heading_t start_heading,
                                     f413_rp_kind_t kind,
                                     f413_rp_side_t side,
                                     f413_rp_anchor_t* out_anchor,
                                     f413_rp_heading_t* out_heading)
{
  const int sign = (side == F413_RP_SIDE_RIGHT) ? 1 : -1;
  f413_rp_heading_t end_heading;
  f413_rp_heading_t intermediate;
  int dx = 0;
  int dy = 0;

  if ((out_anchor == NULL) || (out_heading == NULL) ||
      !f413_rp_turn_source_valid(maze, source, start_heading, kind))
  {
    return false;
  }
  switch (kind)
  {
    case F413_RP_KIND_LARGE_90:
      end_heading = f413_rp_heading_add(start_heading, 2 * sign);
      dx = 2 * (g_f413_rp_dx[(unsigned int)start_heading] +
                g_f413_rp_dx[(unsigned int)end_heading]);
      dy = 2 * (g_f413_rp_dy[(unsigned int)start_heading] +
                g_f413_rp_dy[(unsigned int)end_heading]);
      break;
    case F413_RP_KIND_LARGE_180:
      intermediate = f413_rp_heading_add(start_heading, 2 * sign);
      end_heading = f413_rp_heading_add(start_heading, 4 * sign);
      dx = 2 * g_f413_rp_dx[(unsigned int)intermediate];
      dy = 2 * g_f413_rp_dy[(unsigned int)intermediate];
      break;
    case F413_RP_KIND_45_IN:
    case F413_RP_KIND_45_OUT:
      end_heading = f413_rp_heading_add(start_heading, sign);
      dx = g_f413_rp_dx[(unsigned int)start_heading] +
           g_f413_rp_dx[(unsigned int)end_heading];
      dy = g_f413_rp_dy[(unsigned int)start_heading] +
           g_f413_rp_dy[(unsigned int)end_heading];
      break;
    case F413_RP_KIND_V90:
      end_heading = f413_rp_heading_add(start_heading, 2 * sign);
      dx = g_f413_rp_dx[(unsigned int)start_heading] +
           g_f413_rp_dx[(unsigned int)end_heading];
      dy = g_f413_rp_dy[(unsigned int)start_heading] +
           g_f413_rp_dy[(unsigned int)end_heading];
      break;
    case F413_RP_KIND_135_IN:
      intermediate = f413_rp_heading_add(start_heading, sign);
      end_heading = f413_rp_heading_add(start_heading, 3 * sign);
      dx = g_f413_rp_dx[(unsigned int)start_heading] +
           g_f413_rp_dx[(unsigned int)intermediate] +
           g_f413_rp_dx[(unsigned int)end_heading];
      dy = g_f413_rp_dy[(unsigned int)start_heading] +
           g_f413_rp_dy[(unsigned int)intermediate] +
           g_f413_rp_dy[(unsigned int)end_heading];
      break;
    case F413_RP_KIND_135_OUT:
      intermediate = f413_rp_heading_add(start_heading, sign);
      end_heading = f413_rp_heading_add(start_heading, 3 * sign);
      dx = (2 * g_f413_rp_dx[(unsigned int)intermediate]) +
           g_f413_rp_dx[(unsigned int)end_heading];
      dy = (2 * g_f413_rp_dy[(unsigned int)intermediate]) +
           g_f413_rp_dy[(unsigned int)end_heading];
      break;
    default:
      return false;
  }
  out_anchor->half_x = (int16_t)(source.half_x + dx);
  out_anchor->half_y = (int16_t)(source.half_y + dy);
  *out_heading = end_heading;
  return f413_rp_state_pose_valid(maze, *out_anchor, *out_heading) &&
         f413_rp_keri_turn_guard(maze, source, start_heading, kind,
                                 *out_anchor, *out_heading);
}

static bool f413_rp_add_goal(f413_rp_maze_t* maze, uint8_t x, uint8_t y)
{
  if ((maze == NULL) || (x >= F413_RP_WIDTH) || (y >= F413_RP_HEIGHT) ||
      ((x == START_X) && (y == START_Y)))
  {
    return false;
  }
  for (uint8_t index = 0U; index < maze->goal_count; index++)
  {
    if ((maze->goal_x[index] == x) && (maze->goal_y[index] == y))
    {
      return true;
    }
  }
  if (maze->goal_count >= 9U)
  {
    return false;
  }
  maze->goal_x[maze->goal_count] = x;
  maze->goal_y[maze->goal_count] = y;
  maze->goal_count++;
  f413_rp_goal_set(maze, x, y);
  return true;
}

static bool f413_rp_load_walls(f413_rp_maze_t* maze,
                               f413_rp_maze_source_t* out_source,
                               bool allow_builtin_fallback)
{
  uint16_t cells[F413_RP_CELL_COUNT];
  bool fram_loaded;

  if ((maze == NULL) || (out_source == NULL))
  {
    return false;
  }
  fram_loaded = nvm_maze_load_map(cells, (uint32_t)F413_RP_CELL_COUNT);
  if (!fram_loaded && !allow_builtin_fallback)
  {
    return false;
  }
  memset(maze, 0, sizeof(*maze));
  for (uint8_t y = 0U; y < F413_RP_HEIGHT; y++)
  {
    for (uint8_t x = 0U; x < F413_RP_WIDTH; x++)
    {
      const size_t index = f413_rp_cell_index(x, y);
      maze->walls[index] = fram_loaded ?
          (uint8_t)((cells[index] >> 4U) & 0x0FU) :
          g_f413_rp_builtin_16mm2014cx_walls[index];
      if (x == 0U)
      {
        maze->walls[index] |= 0x01U;
      }
      if (x + 1U == F413_RP_WIDTH)
      {
        maze->walls[index] |= 0x04U;
      }
      if (y == 0U)
      {
        maze->walls[index] |= 0x02U;
      }
      if (y + 1U == F413_RP_HEIGHT)
      {
        maze->walls[index] |= 0x08U;
      }
    }
  }

  /* A one-sided wall is conservatively normalized to present on both cells. */
  for (uint8_t y = 0U; y < F413_RP_HEIGHT; y++)
  {
    for (uint8_t x = 0U; x < F413_RP_WIDTH; x++)
    {
      const size_t index = f413_rp_cell_index(x, y);
      if (x + 1U < F413_RP_WIDTH)
      {
        const size_t east = f413_rp_cell_index((int)x + 1, y);
        const bool here = (maze->walls[index] & 0x04U) != 0U;
        const bool there = (maze->walls[east] & 0x01U) != 0U;
        if (here != there)
        {
          maze->mismatch_count++;
        }
        if (here || there)
        {
          maze->walls[index] |= 0x04U;
          maze->walls[east] |= 0x01U;
        }
      }
      if (y + 1U < F413_RP_HEIGHT)
      {
        const size_t north = f413_rp_cell_index(x, (int)y + 1);
        const bool here = (maze->walls[index] & 0x08U) != 0U;
        const bool there = (maze->walls[north] & 0x02U) != 0U;
        if (here != there)
        {
          maze->mismatch_count++;
        }
        if (here || there)
        {
          maze->walls[index] |= 0x08U;
          maze->walls[north] |= 0x02U;
        }
      }
    }
  }
  *out_source = fram_loaded ? F413_RP_MAZE_SOURCE_FRAM :
                              F413_RP_MAZE_SOURCE_BUILTIN_16MM2014CX;
  return true;
}

static bool f413_rp_load_maze(f413_rp_maze_t* maze,
                               f413_rp_maze_source_t* out_source)
{
  /* Preview diagnostics intentionally keep the conventional centre goal. */
  static const uint8_t goals[4][2] = {
      {7U, 7U}, {8U, 7U}, {7U, 8U}, {8U, 8U},
  };

  if (!f413_rp_load_walls(maze, out_source, true))
  {
    return false;
  }
  for (size_t index = 0U; index < 4U; index++)
  {
    (void)f413_rp_add_goal(maze, goals[index][0], goals[index][1]);
  }
  return maze->goal_count != 0U;
}

static bool f413_rp_load_run_maze(f413_rp_maze_t* maze,
                                   f413_rp_maze_source_t* out_source)
{
  static const uint8_t goals[9][2] = {
      {GOAL1_X, GOAL1_Y}, {GOAL2_X, GOAL2_Y}, {GOAL3_X, GOAL3_Y},
      {GOAL4_X, GOAL4_Y}, {GOAL5_X, GOAL5_Y}, {GOAL6_X, GOAL6_Y},
      {GOAL7_X, GOAL7_Y}, {GOAL8_X, GOAL8_Y}, {GOAL9_X, GOAL9_Y},
  };

  if (!f413_rp_load_walls(maze, out_source, false))
  {
    return false;
  }
  for (size_t index = 0U; index < 9U; index++)
  {
    const uint8_t x = goals[index][0];
    const uint8_t y = goals[index][1];

    /* Keep the existing solver's (0,0) unused-slot convention. */
    if (((x != 0U) || (y != 0U)) &&
        (x < F413_RP_WIDTH) && (y < F413_RP_HEIGHT) &&
        !f413_rp_add_goal(maze, x, y))
    {
      return false;
    }
  }
  return maze->goal_count != 0U;
}

static NfTurnSpec f413_rp_scaled_turn(const NfTurnSpec* source, double scale)
{
  NfTurnSpec result = *source;
  result.velocity_mm_s *= scale;
  result.alpha_deg_s2 *= scale * scale;
  return result;
}

static void f413_rp_expected_closure(f413_rp_kind_t kind,
                                     double* out_forward,
                                     double* out_lateral,
                                     double* out_angle)
{
  const double half = F413_RP_HALF_CELL_MM;
  switch (kind)
  {
    case F413_RP_KIND_LARGE_90:
      *out_forward = 2.0 * half;
      *out_lateral = 2.0 * half;
      *out_angle = 90.0;
      break;
    case F413_RP_KIND_LARGE_180:
      *out_forward = 0.0;
      *out_lateral = 2.0 * half;
      *out_angle = 180.0;
      break;
    case F413_RP_KIND_45_IN:
      *out_forward = 2.0 * half;
      *out_lateral = half;
      *out_angle = 45.0;
      break;
    case F413_RP_KIND_45_OUT:
      *out_forward = 3.0 * half / sqrt(2.0);
      *out_lateral = half / sqrt(2.0);
      *out_angle = 45.0;
      break;
    case F413_RP_KIND_V90:
      *out_forward = half * sqrt(2.0);
      *out_lateral = half * sqrt(2.0);
      *out_angle = 90.0;
      break;
    case F413_RP_KIND_135_IN:
      *out_forward = half;
      *out_lateral = 2.0 * half;
      *out_angle = 135.0;
      break;
    case F413_RP_KIND_135_OUT:
      *out_forward = half / sqrt(2.0);
      *out_lateral = 3.0 * half / sqrt(2.0);
      *out_angle = 135.0;
      break;
    default:
      *out_forward = 0.0;
      *out_lateral = 0.0;
      *out_angle = 0.0;
      break;
  }
}

static bool f413_rp_seconds_to_us(double seconds, uint32_t* out_us)
{
  uint64_t value = 0U;
  if ((out_us == NULL) ||
      (nf_motion_seconds_to_us(seconds, &value) != NF_MOTION_OK) ||
      (value > UINT32_MAX))
  {
    return false;
  }
  *out_us = (uint32_t)value;
  return true;
}

static bool f413_rp_prepare_motion(f413_rp_motion_t* motion,
                                   f413_rp_arena_t* arena,
                                   uint8_t case_index)
{
  const ShortestRunModeParams_t* mode = &shortestRunModeParams2;
  const ShortestRunCaseParams_t* run_case;
  NfTurnSpec nominal_timing[F413_RP_KIND_COUNT] = {
      {true, mode->velocity_l_turn_90, mode->alpha_l_turn_90, 90.0,
       mode->dist_l_turn_in_90, mode->dist_l_turn_out_90},
      {true, mode->velocity_l_turn_180, mode->alpha_l_turn_180, 180.0,
       mode->dist_l_turn_in_180, mode->dist_l_turn_out_180},
      {true, mode->velocity_turn45in, mode->alpha_turn45in, 45.0,
       mode->dist_turn45in_in, mode->dist_turn45in_out},
      {true, mode->velocity_turn45out, mode->alpha_turn45out, 45.0,
       mode->dist_turn45out_in, mode->dist_turn45out_out},
      {true, mode->velocity_turnV90, mode->alpha_turnV90, 90.0,
       mode->dist_turnV90_in, mode->dist_turnV90_out},
      {true, mode->velocity_turn135in, mode->alpha_turn135in, 135.0,
       mode->dist_turn135in_in, mode->dist_turn135in_out},
      {true, mode->velocity_turn135out, mode->alpha_turn135out, 135.0,
       mode->dist_turn135out_in, mode->dist_turn135out_out},
  };
  const NfTurnSpec nominal_geometry[F413_RP_KIND_COUNT] = {
      {true, mode->velocity_l_turn_90, 4700.0, 90.0,
       0.352912418, 0.352912418},
      {true, mode->velocity_l_turn_180, 4422.213141, 180.0, 15.500, 15.500},
      {true, mode->velocity_turn45in, 7234.4, 45.0, 0.0, 18.640},
      {true, mode->velocity_turn45out, 7234.4, 45.0, 18.640, 0.0},
      {true, mode->velocity_turnV90, 12200.0, 90.0, 7.997, 7.997},
      {true, mode->velocity_turn135in, 8500.0, 135.0, 18.932, 11.212},
      {true, mode->velocity_turn135out, 8500.0, 135.0, 11.212, 18.932},
  };
  NfLinearPlan start_plan;
  double crawl_velocity;

  if ((motion == NULL) || (arena == NULL) ||
      (case_index < 1U) || (case_index > 9U))
  {
    return false;
  }
  run_case = &shortestRunCaseParamsMode2[case_index - 1U];
  memset(motion, 0, sizeof(*motion));
  motion->orthogonal = (NfLinearLimits){
      run_case->velocity_straight,
      mode->accel_switch_velocity,
      run_case->acceleration_straight,
      run_case->acceleration_straight_dash,
  };
  motion->diagonal = (NfLinearLimits){
      run_case->velocity_d_straight,
      0.0,
      run_case->acceleration_d_straight,
      run_case->acceleration_d_straight_dash,
  };
  if ((motion->orthogonal.vmax_mm_s <= 0.0) ||
      (motion->diagonal.vmax_mm_s <= 0.0) ||
      (nf_motion_accelerating_exit_velocity(&motion->orthogonal, 5.0, 0.0,
                                             &crawl_velocity) != NF_MOTION_OK) ||
      (nf_motion_linear_plan(&motion->orthogonal, 5.0, 0.0,
                             crawl_velocity, &start_plan) != NF_MOTION_OK) ||
      !f413_rp_seconds_to_us(start_plan.total_time_s,
                            &motion->start_time_us))
  {
    return false;
  }
  motion->speed_mm_s[F413_RP_SPEED_NOMINAL] =
      nominal_timing[F413_RP_KIND_LARGE_90].velocity_mm_s;
  motion->speed_mm_s[F413_RP_SPEED_LOW] = mode->velocity_turn90;
  motion->speed_mm_s[F413_RP_SPEED_CRAWL] = crawl_velocity;
  if ((motion->speed_mm_s[F413_RP_SPEED_NOMINAL] <=
       motion->speed_mm_s[F413_RP_SPEED_LOW]) ||
      (motion->speed_mm_s[F413_RP_SPEED_LOW] <=
       motion->speed_mm_s[F413_RP_SPEED_CRAWL]))
  {
    return false;
  }
  for (size_t kind = 0U; kind < F413_RP_KIND_COUNT; kind++)
  {
    if (fabs(nominal_timing[kind].velocity_mm_s -
             motion->speed_mm_s[F413_RP_SPEED_NOMINAL]) > F413_RP_EPS)
    {
      return false;
    }
    motion->geometry[kind] = nominal_geometry[kind];
    {
      const NfTurnEnvironment environment = {2200.0, 1.2};
      double expected_forward;
      double expected_lateral;
      double expected_angle;
      double residual;
      double required_intervals;
      uint16_t intervals;

      if (nf_motion_turn_plan(&motion->geometry[kind], &environment,
                              &motion->geometry_plan[kind]) != NF_MOTION_OK)
      {
        return false;
      }
      f413_rp_expected_closure((f413_rp_kind_t)kind, &expected_forward,
                               &expected_lateral, &expected_angle);
      residual = hypot(motion->geometry_plan[kind].displacement_forward_mm -
                           expected_forward,
                       motion->geometry_plan[kind].displacement_lateral_mm -
                           expected_lateral);
      if (!isfinite(residual) || (residual > 0.001 + F413_RP_EPS))
      {
        return false;
      }
      required_intervals = fmax(motion->geometry_plan[kind].travel_distance_mm,
                                expected_angle / 2.0);
      intervals = (uint16_t)ceil(required_intervals);
      if (intervals < 2U)
      {
        intervals = 2U;
      }
      if (intervals > F413_RP_MAX_TURN_INTERVALS)
      {
        return false;
      }
      motion->geometry_poses[kind] = (NfTurnPose*)f413_rp_arena_alloc(
          arena, ((size_t)intervals + 1U) * sizeof(NfTurnPose), 8U);
      if ((motion->geometry_poses[kind] == NULL) ||
          (nf_motion_turn_pose_uniform(&motion->geometry[kind],
                                       &motion->geometry_plan[kind], intervals,
                                       motion->geometry_poses[kind],
                                       (size_t)intervals + 1U) != NF_MOTION_OK))
      {
        return false;
      }
      motion->geometry_poses[kind][intervals].forward_mm = expected_forward;
      motion->geometry_poses[kind][intervals].lateral_mm = expected_lateral;
      motion->geometry_poses[kind][intervals].heading_deg = expected_angle;
      motion->geometry_intervals[kind] = intervals;
    }
    for (size_t speed = 0U; speed < F413_RP_SPEED_COUNT; speed++)
    {
      NfTurnEnvironment environment = {2200.0, 1.2};
      const double scale = motion->speed_mm_s[speed] /
                           motion->speed_mm_s[F413_RP_SPEED_NOMINAL];
      motion->timing[speed][kind] =
          f413_rp_scaled_turn(&nominal_timing[kind], scale);
      environment.omega_cap_deg_s *= scale;
      if (nf_motion_turn_plan(&motion->timing[speed][kind], &environment,
                              &motion->timing_plan[speed][kind]) != NF_MOTION_OK)
      {
        return false;
      }
    }
  }
  return true;
}

static const NfLinearLimits* f413_rp_connector_limits(
    const f413_rp_motion_t* motion,
    f413_rp_heading_t heading)
{
  return f413_rp_is_cardinal(heading) ?
         &motion->orthogonal : &motion->diagonal;
}

static double f413_rp_connector_unit(f413_rp_heading_t heading)
{
  return f413_rp_is_cardinal(heading) ?
         F413_RP_HALF_CELL_MM : F413_RP_DIAGONAL_COMMAND_MM;
}

static bool f413_rp_scan_ray(const f413_rp_maze_t* maze,
                             f413_rp_anchor_t source,
                             f413_rp_heading_t heading,
                             f413_rp_ray_t* out)
{
  f413_rp_anchor_t current = source;
  int current_x;
  int current_y;
  uint16_t step = 0U;

  if ((out == NULL) ||
      !f413_rp_anchor_region(maze, source, heading, &current_x, &current_y))
  {
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->stop_anchor = source;
  if (f413_rp_goal_at(maze, current_x, current_y))
  {
    out->first_goal.present = true;
    out->first_goal.x = (uint8_t)current_x;
    out->first_goal.y = (uint8_t)current_y;
  }
  while (step < 68U)
  {
    f413_rp_anchor_t next;
    int next_x;
    int next_y;
    bool safe_stop;
    if (!f413_rp_advance_connector(maze, current, heading, &next) ||
        !f413_rp_anchor_region(maze, next, heading, &next_x, &next_y))
    {
      break;
    }
    step++;
    if (!out->first_goal.present &&
        !f413_rp_goal_at(maze, current_x, current_y) &&
        f413_rp_goal_at(maze, next_x, next_y))
    {
      out->first_goal.present = true;
      out->first_goal.step = step;
      out->first_goal.x = (uint8_t)next_x;
      out->first_goal.y = (uint8_t)next_y;
    }
    safe_stop = !f413_rp_is_cardinal(heading) ||
                f413_rp_anchor_is_center(next);
    if (safe_stop)
    {
      out->stop_steps = step;
      out->stop_anchor = next;
    }
    current = next;
    current_x = next_x;
    current_y = next_y;
  }
  return true;
}

typedef struct
{
  double fraction;
  bool vertical;
  int line;
} f413_rp_boundary_event_t;

static void f413_rp_sort_events(f413_rp_boundary_event_t* events,
                                size_t count)
{
  for (size_t i = 1U; i < count; i++)
  {
    const f413_rp_boundary_event_t value = events[i];
    size_t j = i;
    while ((j > 0U) && (events[j - 1U].fraction > value.fraction))
    {
      events[j] = events[j - 1U];
      j--;
    }
    events[j] = value;
  }
}

static bool f413_rp_trace_segment(const f413_rp_maze_t* maze,
                                   double x0,
                                   double y0,
                                   double x1,
                                   double y1,
                                   double fraction0,
                                   double fraction1,
                                   int* cell_x,
                                   int* cell_y,
                                   f413_rp_turn_trace_t* trace)
{
  f413_rp_boundary_event_t events[36U];
  size_t event_count = 0U;
  const double pitch = 2.0 * F413_RP_HALF_CELL_MM;
  const double dx = x1 - x0;
  const double dy = y1 - y0;

  if (fabs(dx) > F413_RP_EPS)
  {
    for (int line = 0; line <= (int)F413_RP_WIDTH; line++)
    {
      const double fraction = (((double)line * pitch) - x0) / dx;
      if ((fraction > F413_RP_EPS) &&
          (fraction <= 1.0 + F413_RP_EPS))
      {
        if (event_count >= 36U)
        {
          return false;
        }
        events[event_count++] = (f413_rp_boundary_event_t){
            fmin(fraction, 1.0), true, line};
      }
    }
  }
  if (fabs(dy) > F413_RP_EPS)
  {
    for (int line = 0; line <= (int)F413_RP_HEIGHT; line++)
    {
      const double fraction = (((double)line * pitch) - y0) / dy;
      if ((fraction > F413_RP_EPS) &&
          (fraction <= 1.0 + F413_RP_EPS))
      {
        if (event_count >= 36U)
        {
          return false;
        }
        events[event_count++] = (f413_rp_boundary_event_t){
            fmin(fraction, 1.0), false, line};
      }
    }
  }
  f413_rp_sort_events(events, event_count);
  for (size_t index = 0U; index < event_count; index++)
  {
    int next_x = *cell_x;
    int next_y = *cell_y;
    const bool from_goal = f413_rp_goal_at(maze, *cell_x, *cell_y);
    const double fraction = events[index].fraction;

    if ((index + 1U < event_count) &&
        (fabs(events[index + 1U].fraction - fraction) <= 1.0e-7) &&
        (events[index + 1U].vertical != events[index].vertical))
    {
      return false;
    }
    if (events[index].vertical)
    {
      if (dx > 0.0)
      {
        if (*cell_x == events[index].line)
        {
          continue;
        }
        if (*cell_x != events[index].line - 1)
        {
          return false;
        }
        next_x++;
      }
      else
      {
        if (*cell_x == events[index].line - 1)
        {
          continue;
        }
        if (*cell_x != events[index].line)
        {
          return false;
        }
        next_x--;
      }
    }
    else if (dy > 0.0)
    {
      if (*cell_y == events[index].line)
      {
        continue;
      }
      if (*cell_y != events[index].line - 1)
      {
        return false;
      }
      next_y++;
    }
    else
    {
      if (*cell_y == events[index].line - 1)
      {
        continue;
      }
      if (*cell_y != events[index].line)
      {
        return false;
      }
      next_y--;
    }
    if (!f413_rp_cells_open(maze, *cell_x, *cell_y, next_x, next_y))
    {
      return false;
    }
    if (!trace->has_goal && !from_goal &&
        f413_rp_goal_at(maze, next_x, next_y))
    {
      trace->has_goal = true;
      trace->goal_x = (uint8_t)next_x;
      trace->goal_y = (uint8_t)next_y;
      trace->goal_geometry_fraction = fraction0 +
          (fraction * (fraction1 - fraction0));
    }
    *cell_x = next_x;
    *cell_y = next_y;
  }
  return true;
}

static bool f413_rp_turn_bbox_has_goal(const f413_rp_maze_t* maze,
                                       f413_rp_anchor_t source,
                                       f413_rp_anchor_t destination)
{
  const int min_hx = ((source.half_x < destination.half_x) ?
                      source.half_x : destination.half_x) - 4;
  const int max_hx = ((source.half_x > destination.half_x) ?
                      source.half_x : destination.half_x) + 4;
  const int min_hy = ((source.half_y < destination.half_y) ?
                      source.half_y : destination.half_y) - 4;
  const int max_hy = ((source.half_y > destination.half_y) ?
                      source.half_y : destination.half_y) + 4;

  for (uint8_t index = 0U; index < maze->goal_count; index++)
  {
    const int hx = (2 * maze->goal_x[index]) + 1;
    const int hy = (2 * maze->goal_y[index]) + 1;
    if ((hx >= min_hx) && (hx <= max_hx) &&
        (hy >= min_hy) && (hy <= max_hy))
    {
      return true;
    }
  }
  return false;
}

static bool f413_rp_trace_turn(const f413_rp_context_t* context,
                               f413_rp_anchor_t source,
                               f413_rp_heading_t start_heading,
                               f413_rp_kind_t kind,
                               f413_rp_side_t side,
                               f413_rp_anchor_t destination,
                               f413_rp_heading_t end_heading,
                               f413_rp_turn_trace_t* out)
{
  const uint16_t intervals = context->motion->geometry_intervals[kind];
  const NfTurnPose* poses = context->motion->geometry_poses[kind];
  const double start_x = source.half_x * F413_RP_HALF_CELL_MM;
  const double start_y = source.half_y * F413_RP_HALF_CELL_MM;
  const double start_heading_deg = 90.0 - (45.0 * (double)start_heading);
  const double heading_rad = start_heading_deg * (F413_RP_PI / 180.0);
  int current_x;
  int current_y;
  double previous_x = start_x;
  double previous_y = start_y;

  if ((out == NULL) || (poses == NULL) || (intervals < 2U) ||
      !f413_rp_anchor_region(context->maze, source, start_heading,
                              &current_x, &current_y))
  {
    return false;
  }
  memset(out, 0, sizeof(*out));
  out->feasible = true;
  if (!f413_rp_turn_bbox_has_goal(context->maze, source, destination))
  {
    return true;
  }
  if (f413_rp_goal_at(context->maze, current_x, current_y))
  {
    out->has_goal = true;
    out->goal_x = (uint8_t)current_x;
    out->goal_y = (uint8_t)current_y;
  }
  for (uint16_t index = 1U; index <= intervals; index++)
  {
    const NfTurnPose pose = poses[index];
    const double signed_lateral = (side == F413_RP_SIDE_LEFT) ?
                                  pose.lateral_mm : -pose.lateral_mm;
    const double x = start_x + (pose.forward_mm * cos(heading_rad)) -
                     (signed_lateral * sin(heading_rad));
    const double y = start_y + (pose.forward_mm * sin(heading_rad)) +
                     (signed_lateral * cos(heading_rad));
    if (!f413_rp_trace_segment(context->maze, previous_x, previous_y, x, y,
                                (double)(index - 1U) / (double)intervals,
                                (double)index / (double)intervals,
                                &current_x, &current_y, out))
    {
      out->feasible = false;
      return true;
    }
    previous_x = x;
    previous_y = y;
  }
  {
    int expected_x;
    int expected_y;
    if (!f413_rp_anchor_region(context->maze, destination, end_heading,
                                &expected_x, &expected_y) ||
        (current_x != expected_x) || (current_y != expected_y))
    {
      out->feasible = false;
    }
  }
  return true;
}

static bool f413_rp_turn_elapsed_us(const f413_rp_motion_t* motion,
                                    f413_rp_kind_t kind,
                                    f413_rp_speed_t speed,
                                    double geometry_fraction,
                                    uint32_t* out_us)
{
  const NfTurnSpec* timing = &motion->timing[speed][kind];
  const NfTurnPlan* timing_plan = &motion->timing_plan[speed][kind];
  const NfTurnPlan* geometry_plan = &motion->geometry_plan[kind];
  const double duration_s = timing_plan->total_time_s;
  const double average_velocity = geometry_plan->travel_distance_mm /
                                  duration_s;
  const double amplitude = 2.0 * (average_velocity - timing->velocity_mm_s);
  double lower = 0.0;
  double upper = 1.0;

  if ((out_us == NULL) || !isfinite(geometry_fraction) ||
      (geometry_fraction < 0.0) || (geometry_fraction > 1.0) ||
      (duration_s <= 0.0) || (average_velocity <= 0.0) ||
      ((amplitude < 0.0) &&
       (timing->velocity_mm_s + amplitude <= 0.0)))
  {
    return false;
  }
  for (unsigned int iteration = 0U; iteration < 48U; iteration++)
  {
    const double middle = 0.5 * (lower + upper);
    const double distance_mm = duration_s *
        ((timing->velocity_mm_s * middle) +
         (amplitude * ((0.5 * middle) -
          (sin(2.0 * F413_RP_PI * middle) / (4.0 * F413_RP_PI)))));
    if (distance_mm < geometry_fraction * geometry_plan->travel_distance_mm)
    {
      lower = middle;
    }
    else
    {
      upper = middle;
    }
  }
  return f413_rp_seconds_to_us(duration_s * 0.5 * (lower + upper), out_us);
}

static bool f413_rp_u32_add(uint32_t left, uint32_t right, uint32_t* out)
{
  if ((out == NULL) || (left > UINT32_MAX - right))
  {
    return false;
  }
  *out = left + right;
  return true;
}

static bool f413_rp_is_settled(const f413_rp_context_t* context,
                               uint16_t state)
{
  return (context->settled[state >> 3U] &
          (uint8_t)(1U << (state & 7U))) != 0U;
}

static void f413_rp_set_settled(f413_rp_context_t* context, uint16_t state)
{
  context->settled[state >> 3U] |= (uint8_t)(1U << (state & 7U));
}

static bool f413_rp_heap_less(const f413_rp_context_t* context,
                              uint16_t left,
                              uint16_t right)
{
  if (context->distances[left] != context->distances[right])
  {
    return context->distances[left] < context->distances[right];
  }
  if (context->turn_counts[left] != context->turn_counts[right])
  {
    return context->turn_counts[left] < context->turn_counts[right];
  }
  return left < right;
}

static void f413_rp_heap_swap(f413_rp_context_t* context,
                               uint16_t left_position,
                               uint16_t right_position)
{
  const uint16_t left_state = context->heap_states[left_position];
  const uint16_t right_state = context->heap_states[right_position];
  context->heap_states[left_position] = right_state;
  context->heap_states[right_position] = left_state;
  context->heap_positions[left_state] = right_position;
  context->heap_positions[right_state] = left_position;
}

static bool f413_rp_heap_push_or_decrease(f413_rp_context_t* context,
                                           uint16_t state)
{
  uint16_t position;

  if ((context == NULL) || (state >= F413_RP_STATE_COUNT) ||
      (context->heap_states == NULL) || (context->heap_positions == NULL))
  {
    return false;
  }
  position = context->heap_positions[state];
  if (position == UINT16_MAX)
  {
    if (context->heap_size >= F413_RP_STATE_COUNT)
    {
      return false;
    }
    position = context->heap_size;
    context->heap_states[position] = state;
    context->heap_positions[state] = position;
    context->heap_size++;
    if (context->heap_size > context->heap_peak)
    {
      context->heap_peak = context->heap_size;
    }
  }
  else if ((position >= context->heap_size) ||
           (context->heap_states[position] != state))
  {
    return false;
  }

  while (position != 0U)
  {
    const uint16_t parent = (uint16_t)((position - 1U) / 2U);
    if (!f413_rp_heap_less(context, state,
                           context->heap_states[parent]))
    {
      break;
    }
    f413_rp_heap_swap(context, position, parent);
    position = parent;
  }
  return true;
}

static bool f413_rp_heap_peek(const f413_rp_context_t* context,
                              uint16_t* out_state)
{
  if ((context == NULL) || (out_state == NULL) ||
      (context->heap_size == 0U) || (context->heap_states == NULL) ||
      (context->heap_positions == NULL) ||
      (context->heap_positions[context->heap_states[0]] != 0U))
  {
    return false;
  }
  *out_state = context->heap_states[0];
  return true;
}

static bool f413_rp_heap_pop(f413_rp_context_t* context, uint16_t* out_state)
{
  uint16_t root;
  uint16_t position = 0U;

  if (!f413_rp_heap_peek(context, &root))
  {
    return false;
  }
  context->heap_size--;
  context->heap_positions[root] = UINT16_MAX;
  if (context->heap_size != 0U)
  {
    const uint16_t moved = context->heap_states[context->heap_size];
    context->heap_states[0] = moved;
    context->heap_positions[moved] = 0U;
    while (true)
    {
      const uint32_t left_candidate = ((uint32_t)position * 2U) + 1U;
      uint16_t child;
      uint16_t right;
      if (left_candidate >= context->heap_size)
      {
        break;
      }
      child = (uint16_t)left_candidate;
      right = (uint16_t)(left_candidate + 1U);
      if ((right < context->heap_size) &&
          f413_rp_heap_less(context, context->heap_states[right],
                            context->heap_states[child]))
      {
        child = right;
      }
      if (!f413_rp_heap_less(context, context->heap_states[child],
                             context->heap_states[position]))
      {
        break;
      }
      f413_rp_heap_swap(context, position, child);
      position = child;
    }
  }
  *out_state = root;
  return true;
}

static uint32_t f413_rp_pack_parent(uint16_t previous_state,
                                    uint16_t connector_steps,
                                    f413_rp_kind_t kind,
                                    f413_rp_side_t side,
                                    f413_rp_speed_t speed)
{
  uint32_t parent = F413_RP_PARENT_VALID |
                    ((uint32_t)previous_state & F413_RP_PARENT_PREV_MASK) |
                    (((uint32_t)connector_steps <<
                      F413_RP_PARENT_CONNECTOR_SHIFT) &
                     F413_RP_PARENT_CONNECTOR_MASK) |
                    (((uint32_t)kind << F413_RP_PARENT_KIND_SHIFT) &
                     F413_RP_PARENT_KIND_MASK) |
                    (((uint32_t)speed << F413_RP_PARENT_SPEED_SHIFT) &
                     F413_RP_PARENT_SPEED_MASK);
  if (side == F413_RP_SIDE_LEFT)
  {
    parent |= F413_RP_PARENT_LEFT;
  }
  return parent;
}

static uint16_t f413_rp_parent_previous(uint32_t parent)
{
  return (uint16_t)(parent & F413_RP_PARENT_PREV_MASK);
}

static uint16_t f413_rp_parent_connector(uint32_t parent)
{
  return (uint16_t)((parent & F413_RP_PARENT_CONNECTOR_MASK) >>
                    F413_RP_PARENT_CONNECTOR_SHIFT);
}

static f413_rp_kind_t f413_rp_parent_kind(uint32_t parent)
{
  return (f413_rp_kind_t)((parent & F413_RP_PARENT_KIND_MASK) >>
                          F413_RP_PARENT_KIND_SHIFT);
}

static f413_rp_side_t f413_rp_parent_side(uint32_t parent)
{
  return (parent & F413_RP_PARENT_LEFT) != 0U ?
         F413_RP_SIDE_LEFT : F413_RP_SIDE_RIGHT;
}

static f413_rp_speed_t f413_rp_parent_speed(uint32_t parent)
{
  return (f413_rp_speed_t)((parent & F413_RP_PARENT_SPEED_MASK) >>
                           F413_RP_PARENT_SPEED_SHIFT);
}

static bool f413_rp_goal_better(const f413_rp_goal_t* candidate,
                                const f413_rp_goal_t* current)
{
  if (!current->valid)
  {
    return true;
  }
  if (candidate->goal_entry_us != current->goal_entry_us)
  {
    return candidate->goal_entry_us < current->goal_entry_us;
  }
  if (candidate->stop_us != current->stop_us)
  {
    return candidate->stop_us < current->stop_us;
  }
  if (candidate->turn_count != current->turn_count)
  {
    return candidate->turn_count < current->turn_count;
  }
  if (candidate->goal_y != current->goal_y)
  {
    return candidate->goal_y < current->goal_y;
  }
  if (candidate->goal_x != current->goal_x)
  {
    return candidate->goal_x < current->goal_x;
  }
  if (candidate->kind != current->kind)
  {
    return candidate->kind < current->kind;
  }
  if (candidate->side != current->side)
  {
    return candidate->side < current->side;
  }
  return candidate->source_state < current->source_state;
}

static bool f413_rp_brake_tail(const f413_rp_context_t* context,
                               f413_rp_anchor_t source,
                               f413_rp_heading_t heading,
                               double entry_velocity,
                               uint16_t* out_steps,
                               uint32_t* out_us)
{
  f413_rp_ray_t ray;
  f413_rp_anchor_t current = source;
  uint32_t best_us = UINT32_MAX;
  uint16_t best_steps = 0U;
  const double unit = f413_rp_connector_unit(heading);

  if ((out_steps == NULL) || (out_us == NULL) ||
      !f413_rp_scan_ray(context->maze, source, heading, &ray) ||
      (ray.stop_steps == 0U))
  {
    return false;
  }
  for (uint16_t step = 1U; step <= ray.stop_steps; step++)
  {
    f413_rp_anchor_t next;
    NfLinearPlan linear;
    uint32_t duration_us;
    if (!f413_rp_advance_connector(context->maze, current, heading, &next))
    {
      return false;
    }
    current = next;
    if ((f413_rp_is_cardinal(heading) &&
         !f413_rp_anchor_is_center(current)) ||
        (nf_motion_linear_plan(
             f413_rp_connector_limits(context->motion, heading),
             step * unit, entry_velocity, 0.0, &linear) != NF_MOTION_OK) ||
        !f413_rp_seconds_to_us(linear.total_time_s, &duration_us))
    {
      continue;
    }
    if (duration_us < best_us)
    {
      best_us = duration_us;
      best_steps = step;
    }
  }
  if (best_steps == 0U)
  {
    return false;
  }
  *out_steps = best_steps;
  *out_us = best_us;
  return true;
}

static bool f413_rp_consider_direct_goal(f413_rp_context_t* context,
                                         uint16_t state,
                                         f413_rp_anchor_t source,
                                         f413_rp_heading_t heading,
                                         f413_rp_speed_t speed)
{
  f413_rp_ray_t ray;
  f413_rp_anchor_t current = source;
  const double unit = f413_rp_connector_unit(heading);
  const double entry_velocity = context->motion->speed_mm_s[speed];

  if (!f413_rp_scan_ray(context->maze, source, heading, &ray))
  {
    return false;
  }
  context->goal_cross_reachable =
      context->goal_cross_reachable || ray.first_goal.present;
  if ((context->execution_compatible && !f413_rp_is_cardinal(heading)) ||
      !ray.first_goal.present || (ray.first_goal.step == 0U) ||
      (ray.stop_steps < ray.first_goal.step) ||
      ((uint16_t)(ray.stop_steps - ray.first_goal.step) < 1U))
  {
    return true;
  }
  for (uint16_t step = 1U; step <= ray.stop_steps; step++)
  {
    f413_rp_anchor_t next;
    NfLinearPlan linear;
    double cross_time_s;
    double cross_velocity;
    uint32_t cross_us;
    uint32_t duration_us;
    f413_rp_goal_t candidate;

    if (!f413_rp_advance_connector(context->maze, current, heading, &next))
    {
      return false;
    }
    current = next;
    if ((step < ray.first_goal.step) ||
        ((uint16_t)(step - ray.first_goal.step) < 1U) ||
        (f413_rp_is_cardinal(heading) &&
         !f413_rp_anchor_is_center(current)) ||
        (nf_motion_linear_plan(
             f413_rp_connector_limits(context->motion, heading),
             step * unit, entry_velocity, 0.0, &linear) != NF_MOTION_OK) ||
        (nf_motion_linear_time_at_distance(
             &linear, ray.first_goal.step * unit,
             &cross_time_s, &cross_velocity) != NF_MOTION_OK) ||
        !f413_rp_seconds_to_us(cross_time_s, &cross_us) ||
        !f413_rp_seconds_to_us(linear.total_time_s, &duration_us))
    {
      continue;
    }
    (void)cross_velocity;
    memset(&candidate, 0, sizeof(candidate));
    candidate.valid = true;
    candidate.direct_connector = true;
    candidate.source_state = state;
    candidate.connector_steps = step;
    candidate.goal_step = ray.first_goal.step;
    candidate.stop_steps = step;
    candidate.speed = speed;
    candidate.goal_x = ray.first_goal.x;
    candidate.goal_y = ray.first_goal.y;
    candidate.connector_us = duration_us;
    candidate.goal_cross_edge_us = cross_us;
    candidate.turn_count = context->turn_counts[state];
    if (!f413_rp_u32_add(context->distances[state], cross_us,
                         &candidate.goal_entry_us) ||
        !f413_rp_u32_add(context->distances[state], duration_us,
                         &candidate.stop_us))
    {
      return false;
    }
    if (f413_rp_goal_better(&candidate, &context->goal))
    {
      context->goal = candidate;
    }
  }
  return true;
}

static bool f413_rp_relax(f413_rp_context_t* context,
                          uint16_t source_state,
                          uint16_t destination_state,
                          uint16_t connector_steps,
                          f413_rp_kind_t kind,
                          f413_rp_side_t side,
                          f413_rp_speed_t speed,
                          uint32_t duration_us)
{
  uint32_t distance;
  uint16_t turn_count;
  /* Every edge has positive duration, so an improving shortest label is
   * cycle-free and uses fewer than F413_RP_STATE_COUNT edges. */
  if (!f413_rp_u32_add(context->distances[source_state], duration_us,
                       &distance) ||
      (context->turn_counts[source_state] >= F413_RP_STATE_COUNT))
  {
    return false;
  }
  turn_count = (uint16_t)(context->turn_counts[source_state] + 1U);
  if ((distance > context->distances[destination_state]) ||
      ((distance == context->distances[destination_state]) &&
       (turn_count >= context->turn_counts[destination_state])))
  {
    return true;
  }
  if (f413_rp_is_settled(context, destination_state))
  {
    /* A better label for a settled state violates Dijkstra's invariant. */
    return false;
  }
  context->distances[destination_state] = distance;
  context->turn_counts[destination_state] = turn_count;
  context->parents[destination_state] = f413_rp_pack_parent(
      source_state, connector_steps, kind, side, speed);
  context->relaxed_edges++;
  return f413_rp_heap_push_or_decrease(context, destination_state);
}

static bool f413_rp_build_turn_edge(f413_rp_context_t* context,
                                    uint16_t source_state,
                                    f413_rp_heading_t start_heading,
                                    f413_rp_speed_t source_speed,
                                    uint16_t connector_steps,
                                    f413_rp_anchor_t connector_end,
                                    f413_rp_kind_t kind,
                                    f413_rp_side_t side,
                                    f413_rp_speed_t turn_speed)
{
  f413_rp_anchor_t destination;
  f413_rp_heading_t end_heading;
  f413_rp_turn_trace_t turn_trace;
  NfLinearPlan connector_plan;
  uint32_t connector_us;
  uint32_t turn_us;
  uint32_t edge_us;
  const double connector_distance = connector_steps *
      f413_rp_connector_unit(start_heading);
  const double entry_velocity = context->motion->speed_mm_s[source_speed];
  const double turn_velocity = context->motion->speed_mm_s[turn_speed];

  if (!f413_rp_turn_destination(context->maze, connector_end, start_heading,
                                 kind, side, &destination, &end_heading))
  {
    return true;
  }
  if (turn_velocity > f413_rp_connector_limits(
          context->motion, start_heading)->vmax_mm_s + F413_RP_EPS)
  {
    return true;
  }
  if (nf_motion_linear_plan(
          f413_rp_connector_limits(context->motion, start_heading),
          connector_distance, entry_velocity, turn_velocity,
          &connector_plan) != NF_MOTION_OK)
  {
    /* This speed realization simply needs more connector run-up. */
    return true;
  }
  if (!f413_rp_seconds_to_us(connector_plan.total_time_s, &connector_us) ||
      !f413_rp_seconds_to_us(
          context->motion->timing_plan[turn_speed][kind].total_time_s,
          &turn_us) ||
      !f413_rp_u32_add(connector_us, turn_us, &edge_us) ||
      !f413_rp_trace_turn(context, connector_end, start_heading, kind, side,
                          destination, end_heading, &turn_trace))
  {
    return false;
  }
  if (!turn_trace.feasible)
  {
    return true;
  }
  if (turn_trace.has_goal)
  {
    f413_rp_goal_t candidate;
    uint32_t turn_cross_us;
    uint32_t cross_edge_us;
    uint32_t edge_end_us;
    uint16_t stop_steps;
    uint32_t stop_tail_us;

    context->goal_cross_reachable = true;

    if ((context->execution_compatible && !f413_rp_is_cardinal(end_heading)) ||
        !f413_rp_turn_elapsed_us(context->motion, kind, turn_speed,
                                  turn_trace.goal_geometry_fraction,
                                  &turn_cross_us) ||
        !f413_rp_u32_add(connector_us, turn_cross_us, &cross_edge_us) ||
        !f413_rp_brake_tail(context, destination, end_heading, turn_velocity,
                            &stop_steps, &stop_tail_us))
    {
      return true;
    }
    memset(&candidate, 0, sizeof(candidate));
    candidate.valid = true;
    candidate.source_state = source_state;
    candidate.connector_steps = connector_steps;
    candidate.stop_steps = stop_steps;
    candidate.kind = kind;
    candidate.side = side;
    candidate.speed = turn_speed;
    candidate.goal_x = turn_trace.goal_x;
    candidate.goal_y = turn_trace.goal_y;
    candidate.connector_us = connector_us;
    candidate.turn_us = turn_us;
    candidate.goal_cross_edge_us = cross_edge_us;
    candidate.stop_tail_us = stop_tail_us;
    candidate.turn_count = (uint16_t)(context->turn_counts[source_state] + 1U);
    if (!f413_rp_u32_add(context->distances[source_state], cross_edge_us,
                         &candidate.goal_entry_us) ||
        !f413_rp_u32_add(context->distances[source_state], edge_us,
                         &edge_end_us) ||
        !f413_rp_u32_add(edge_end_us, stop_tail_us, &candidate.stop_us))
    {
      return false;
    }
    if (f413_rp_goal_better(&candidate, &context->goal))
    {
      context->goal = candidate;
    }
    return true;
  }
  {
    const uint16_t destination_state = f413_rp_state_index(
        destination, end_heading, turn_speed);
    return f413_rp_relax(context, source_state, destination_state,
                          connector_steps, kind, side, turn_speed, edge_us);
  }
}

static bool f413_rp_expand_state(f413_rp_context_t* context, uint16_t state)
{
  f413_rp_anchor_t anchor;
  f413_rp_heading_t heading;
  f413_rp_speed_t speed;
  f413_rp_anchor_t connector_anchor;
  int current_x;
  int current_y;
  bool connector_entered_goal;
  uint16_t connector_steps = 0U;

  if (!f413_rp_state_decode(state, &anchor, &heading, &speed) ||
      !f413_rp_state_pose_valid(context->maze, anchor, heading) ||
      !f413_rp_anchor_region(context->maze, anchor, heading,
                              &current_x, &current_y) ||
      !f413_rp_consider_direct_goal(context, state, anchor, heading, speed))
  {
    return false;
  }
  connector_anchor = anchor;
  connector_entered_goal = f413_rp_goal_at(context->maze,
                                            current_x, current_y);
  while (connector_steps <= 63U)
  {
    if (connector_entered_goal)
    {
      break;
    }
    for (f413_rp_kind_t kind = F413_RP_KIND_LARGE_90;
         kind < F413_RP_KIND_COUNT; kind++)
    {
      if (!f413_rp_turn_source_valid(context->maze, connector_anchor,
                                      heading, kind))
      {
        continue;
      }
      for (f413_rp_speed_t turn_speed = F413_RP_SPEED_NOMINAL;
           turn_speed < F413_RP_SPEED_COUNT; turn_speed++)
      {
        if (context->execution_compatible &&
            (turn_speed != F413_RP_SPEED_NOMINAL))
        {
          continue;
        }
        if (!f413_rp_build_turn_edge(context, state, heading, speed,
                                      connector_steps, connector_anchor, kind,
                                      F413_RP_SIDE_RIGHT, turn_speed) ||
            !f413_rp_build_turn_edge(context, state, heading, speed,
                                      connector_steps, connector_anchor, kind,
                                      F413_RP_SIDE_LEFT, turn_speed))
        {
          return false;
        }
      }
    }
    {
      f413_rp_anchor_t next;
      int next_x;
      int next_y;
      if (!f413_rp_advance_connector(context->maze, connector_anchor,
                                      heading, &next) ||
          !f413_rp_anchor_region(context->maze, next, heading,
                                  &next_x, &next_y))
      {
        break;
      }
      connector_steps++;
      if (!f413_rp_goal_at(context->maze, current_x, current_y) &&
          f413_rp_goal_at(context->maze, next_x, next_y))
      {
        connector_entered_goal = true;
      }
      connector_anchor = next;
      current_x = next_x;
      current_y = next_y;
    }
  }
  return true;
}

static f413_rp_plan_status_t f413_rp_plan(f413_rp_context_t* context,
                                          uint16_t* out_start_state)
{
  const f413_rp_anchor_t start_anchor = {
      (int16_t)((2U * START_X) + 1U),
      (int16_t)((2U * START_Y) + 1U),
  };
  const uint16_t start_state = f413_rp_state_index(
      start_anchor, F413_RP_HEADING_NORTH, F413_RP_SPEED_CRAWL);

  if ((context == NULL) || (out_start_state == NULL) ||
      (context->maze == NULL) || (context->motion == NULL) ||
      (context->distances == NULL) || (context->turn_counts == NULL) ||
      (context->parents == NULL) || (context->settled == NULL) ||
      (context->heap_states == NULL) ||
      (context->heap_positions == NULL) ||
      (start_state >= F413_RP_STATE_COUNT) ||
      !f413_rp_state_pose_valid(context->maze, start_anchor,
                                 F413_RP_HEADING_NORTH))
  {
    return F413_RP_PLAN_ERROR;
  }
  memset(context->distances, 0xFF,
         F413_RP_STATE_COUNT * sizeof(context->distances[0]));
  memset(context->turn_counts, 0xFF,
         F413_RP_STATE_COUNT * sizeof(context->turn_counts[0]));
  memset(context->parents, 0,
         F413_RP_STATE_COUNT * sizeof(context->parents[0]));
  memset(context->settled, 0, F413_RP_SETTLED_BYTES);
  memset(context->heap_positions, 0xFF,
         F413_RP_STATE_COUNT * sizeof(context->heap_positions[0]));
  memset(&context->goal, 0, sizeof(context->goal));
  context->heap_size = 0U;
  context->heap_peak = 0U;
  context->expanded_states = 0U;
  context->relaxed_edges = 0U;
  context->goal_cross_reachable = false;
  context->distances[start_state] = context->motion->start_time_us;
  context->turn_counts[start_state] = 0U;
  *out_start_state = start_state;
  if (!f413_rp_heap_push_or_decrease(context, start_state))
  {
    return F413_RP_PLAN_ERROR;
  }

  while (context->heap_size != 0U)
  {
    uint16_t best_state;
    uint16_t popped_state;
    uint32_t best_distance;

    if (!f413_rp_heap_peek(context, &best_state))
    {
      return F413_RP_PLAN_ERROR;
    }
    best_distance = context->distances[best_state];
    if (context->goal.valid && (best_distance > context->goal.goal_entry_us))
    {
      break;
    }
    if ((best_distance == F413_RP_INF) ||
        (context->expanded_states >= F413_RP_STATE_COUNT) ||
        !f413_rp_heap_pop(context, &popped_state) ||
        (popped_state != best_state) ||
        f413_rp_is_settled(context, best_state))
    {
      return F413_RP_PLAN_ERROR;
    }
    f413_rp_set_settled(context, best_state);
    context->expanded_states++;
    if (!f413_rp_expand_state(context, best_state))
    {
      return F413_RP_PLAN_ERROR;
    }
  }
  if (context->goal.valid)
  {
    return F413_RP_PLAN_OK;
  }
  return context->goal_cross_reachable ?
         F413_RP_PLAN_NO_FEASIBLE_TERMINAL : F413_RP_PLAN_NO_PATH;
}

static const char* f413_rp_heading_name(f413_rp_heading_t heading)
{
  static const char* const names[8] = {
      "N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return ((unsigned int)heading < 8U) ? names[(unsigned int)heading] : "?";
}

static const char* f413_rp_speed_name(f413_rp_speed_t speed)
{
  static const char* const names[F413_RP_SPEED_COUNT] = {
      "nominal", "low", "crawl"};
  return ((unsigned int)speed < F413_RP_SPEED_COUNT) ?
         names[(unsigned int)speed] : "?";
}

static const char* f413_rp_kind_name(f413_rp_kind_t kind)
{
  static const char* const names[F413_RP_KIND_COUNT] = {
      "large90", "large180", "45in", "45out", "V90", "135in", "135out"};
  return ((unsigned int)kind < F413_RP_KIND_COUNT) ?
         names[(unsigned int)kind] : "?";
}

static unsigned int f413_rp_pattern_number(f413_rp_kind_t kind)
{
  switch (kind)
  {
    case F413_RP_KIND_45_IN:
    case F413_RP_KIND_45_OUT:
      return 1U;
    case F413_RP_KIND_LARGE_90:
      return 2U;
    case F413_RP_KIND_135_IN:
    case F413_RP_KIND_135_OUT:
      return 3U;
    case F413_RP_KIND_LARGE_180:
      return 4U;
    case F413_RP_KIND_V90:
      return 5U;
    default:
      return 0U;
  }
}

static bool f413_rp_action_is_diagonal(f413_rp_heading_t connector_heading,
                                        f413_rp_kind_t kind)
{
  return !f413_rp_is_cardinal(connector_heading) ||
         ((kind != F413_RP_KIND_LARGE_90) &&
          (kind != F413_RP_KIND_LARGE_180));
}

static bool f413_rp_advance_steps(const f413_rp_maze_t* maze,
                                  f413_rp_anchor_t source,
                                  f413_rp_heading_t heading,
                                  uint16_t steps,
                                  f413_rp_anchor_t* out)
{
  f413_rp_anchor_t current = source;
  for (uint16_t step = 0U; step < steps; step++)
  {
    f413_rp_anchor_t next;
    if (!f413_rp_advance_connector(maze, current, heading, &next))
    {
      return false;
    }
    current = next;
  }
  *out = current;
  return true;
}

static uint16_t f413_rp_legacy_turn_code(f413_rp_kind_t kind,
                                          f413_rp_side_t side)
{
  static const uint16_t right_codes[F413_RP_KIND_COUNT] = {
      NF_LEGACY_PATH_LARGE_RIGHT_90,
      NF_LEGACY_PATH_LARGE_RIGHT_180,
      NF_LEGACY_PATH_RIGHT_45_IN,
      NF_LEGACY_PATH_RIGHT_45_OUT,
      NF_LEGACY_PATH_RIGHT_V90,
      NF_LEGACY_PATH_RIGHT_135_IN,
      NF_LEGACY_PATH_RIGHT_135_OUT,
  };
  static const uint16_t left_codes[F413_RP_KIND_COUNT] = {
      NF_LEGACY_PATH_LARGE_LEFT_90,
      NF_LEGACY_PATH_LARGE_LEFT_180,
      NF_LEGACY_PATH_LEFT_45_IN,
      NF_LEGACY_PATH_LEFT_45_OUT,
      NF_LEGACY_PATH_LEFT_V90,
      NF_LEGACY_PATH_LEFT_135_IN,
      NF_LEGACY_PATH_LEFT_135_OUT,
  };

  if ((unsigned int)kind >= F413_RP_KIND_COUNT)
  {
    return 0U;
  }
  return (side == F413_RP_SIDE_LEFT) ? left_codes[kind] : right_codes[kind];
}

static bool f413_rp_legacy_append(uint16_t* output,
                                  size_t capacity,
                                  size_t* count,
                                  uint16_t code)
{
  if ((output == NULL) || (count == NULL) || (code == 0U) ||
      (*count >= capacity) || ((*count + 1U) >= capacity))
  {
    return false;
  }
  output[*count] = code;
  (*count)++;
  return true;
}

static bool f413_rp_legacy_emit_connector(uint16_t* output,
                                           size_t capacity,
                                           size_t* count,
                                           bool diagonal,
                                           uint16_t steps)
{
  const uint16_t base = diagonal ?
      NF_LEGACY_PATH_DIAGONAL_STRAIGHT_BASE :
      NF_LEGACY_PATH_STRAIGHT_BASE;

  while (steps != 0U)
  {
    const uint16_t chunk = (steps > 99U) ? 99U : steps;
    if (!f413_rp_legacy_append(output, capacity, count,
                                (uint16_t)(base + chunk)))
    {
      return false;
    }
    steps = (uint16_t)(steps - chunk);
  }
  return true;
}

static bool f413_rp_emit_legacy_path(f413_rp_context_t* context,
                                      uint16_t start_state,
                                      uint16_t* output,
                                      size_t output_capacity,
                                      size_t* out_count)
{
  uint16_t* chain;
  uint16_t* temporary;
  size_t chain_count = 0U;
  size_t count = 0U;
  uint16_t state;
  NfLegacyPathResult validation;

  if ((context == NULL) || !context->goal.valid ||
      (context->heap_states == NULL) || (context->heap_positions == NULL) ||
      (output == NULL) || (output_capacity < 2U))
  {
    return false;
  }
  chain = context->heap_states;
  temporary = context->heap_positions;
  state = context->goal.source_state;
  while (true)
  {
    uint32_t parent;
    if (chain_count >= F413_RP_STATE_COUNT)
    {
      return false;
    }
    chain[chain_count++] = state;
    if (state == start_state)
    {
      break;
    }
    parent = context->parents[state];
    if ((parent & F413_RP_PARENT_VALID) == 0U)
    {
      return false;
    }
    state = f413_rp_parent_previous(parent);
  }

  for (size_t cursor = chain_count; cursor > 1U; cursor--)
  {
    const uint16_t destination_state = chain[cursor - 2U];
    const uint32_t parent = context->parents[destination_state];
    f413_rp_anchor_t source;
    f413_rp_heading_t heading;
    f413_rp_speed_t source_speed;
    const f413_rp_speed_t turn_speed = f413_rp_parent_speed(parent);
    const uint16_t turn_code = f413_rp_legacy_turn_code(
        f413_rp_parent_kind(parent), f413_rp_parent_side(parent));

    if ((turn_speed != F413_RP_SPEED_NOMINAL) || (turn_code == 0U) ||
        !f413_rp_state_decode(f413_rp_parent_previous(parent), &source,
                               &heading, &source_speed) ||
        !f413_rp_legacy_emit_connector(
            temporary, output_capacity, &count,
            !f413_rp_is_cardinal(heading),
            f413_rp_parent_connector(parent)) ||
        !f413_rp_legacy_append(temporary, output_capacity, &count, turn_code))
    {
      return false;
    }
  }

  if (context->goal.direct_connector)
  {
    f413_rp_anchor_t source;
    f413_rp_heading_t heading;
    f413_rp_speed_t speed;
    if ((context->goal.connector_steps < 1U) ||
        !f413_rp_state_decode(context->goal.source_state, &source,
                               &heading, &speed) ||
        !f413_rp_is_cardinal(heading) ||
        !f413_rp_legacy_emit_connector(
            temporary, output_capacity, &count, false,
            (uint16_t)(context->goal.connector_steps - 1U)))
    {
      return false;
    }
  }
  else
  {
    f413_rp_anchor_t source;
    f413_rp_anchor_t connector_end;
    f413_rp_anchor_t destination;
    f413_rp_heading_t start_heading;
    f413_rp_heading_t end_heading;
    f413_rp_speed_t source_speed;
    const uint16_t turn_code = f413_rp_legacy_turn_code(
        context->goal.kind, context->goal.side);

    if ((context->goal.speed != F413_RP_SPEED_NOMINAL) ||
        (context->goal.stop_steps < 1U) || (turn_code == 0U) ||
        !f413_rp_state_decode(context->goal.source_state, &source,
                               &start_heading, &source_speed) ||
        !f413_rp_advance_steps(context->maze, source, start_heading,
                               context->goal.connector_steps,
                               &connector_end) ||
        !f413_rp_turn_destination(context->maze, connector_end, start_heading,
                                   context->goal.kind, context->goal.side,
                                   &destination, &end_heading) ||
        !f413_rp_is_cardinal(end_heading) ||
        !f413_rp_legacy_emit_connector(
            temporary, output_capacity, &count,
            !f413_rp_is_cardinal(start_heading),
            context->goal.connector_steps) ||
        !f413_rp_legacy_append(temporary, output_capacity, &count, turn_code) ||
        !f413_rp_legacy_emit_connector(
            temporary, output_capacity, &count, false,
            (uint16_t)(context->goal.stop_steps - 1U)))
    {
      return false;
    }
  }

  temporary[count] = 0U;
  validation = nf_legacy_path_validate(temporary, count + 1U);
  if ((validation.status != NF_LEGACY_PATH_OK) ||
      (validation.length != count) || (count >= output_capacity))
  {
    return false;
  }
  memcpy(output, temporary, (count + 1U) * sizeof(output[0]));
  if (out_count != NULL)
  {
    *out_count = count;
  }
  return true;
}

static bool f413_rp_print_turn_action(const f413_rp_context_t* context,
                                      unsigned int action_index,
                                      uint16_t source_state,
                                      uint16_t connector_steps,
                                      f413_rp_kind_t kind,
                                      f413_rp_side_t side,
                                      f413_rp_speed_t turn_speed,
                                      uint32_t cumulative_us,
                                      bool goal_cross)
{
  f413_rp_anchor_t source;
  f413_rp_anchor_t connector_end;
  f413_rp_anchor_t destination;
  f413_rp_heading_t start_heading;
  f413_rp_heading_t end_heading;
  f413_rp_speed_t source_speed;
  NfLinearPlan connector_plan;
  uint32_t connector_us;
  uint32_t turn_us;
  uint32_t action_us;
  bool diagonal_connector;

  if (!f413_rp_state_decode(source_state, &source, &start_heading,
                             &source_speed) ||
      !f413_rp_advance_steps(context->maze, source, start_heading,
                             connector_steps, &connector_end) ||
      !f413_rp_turn_destination(context->maze, connector_end, start_heading,
                                 kind, side, &destination, &end_heading) ||
      (nf_motion_linear_plan(
           f413_rp_connector_limits(context->motion, start_heading),
           connector_steps * f413_rp_connector_unit(start_heading),
           context->motion->speed_mm_s[source_speed],
           context->motion->speed_mm_s[turn_speed],
           &connector_plan) != NF_MOTION_OK) ||
      !f413_rp_seconds_to_us(connector_plan.total_time_s, &connector_us) ||
      !f413_rp_seconds_to_us(
          context->motion->timing_plan[turn_speed][kind].total_time_s,
          &turn_us) ||
      !f413_rp_u32_add(connector_us, turn_us, &action_us))
  {
    return false;
  }
  diagonal_connector = !f413_rp_is_cardinal(start_heading);
  trace_printf(
      "[KERI-PREVIEW] A%u #%u %s-%c conn=%c%u speed=%s(%lu) "
      "H=%s->%s anchor=(%d,%d)->(%d,%d) dt=%lu.%06lu s total=%lu.%06lu s%s\r\n",
      action_index,
      f413_rp_pattern_number(kind),
      f413_rp_kind_name(kind),
      (side == F413_RP_SIDE_LEFT) ? 'L' : 'R',
      diagonal_connector ? 'D' : 'O',
      (unsigned int)connector_steps,
      f413_rp_speed_name(turn_speed),
      (unsigned long)context->motion->speed_mm_s[turn_speed],
      f413_rp_heading_name(start_heading),
      f413_rp_heading_name(end_heading),
      (int)source.half_x, (int)source.half_y,
      (int)destination.half_x, (int)destination.half_y,
      (unsigned long)(action_us / 1000000U),
      (unsigned long)(action_us % 1000000U),
      (unsigned long)(cumulative_us / 1000000U),
      (unsigned long)(cumulative_us % 1000000U),
      goal_cross ? " GOAL-CROSS" : "");
  return true;
}

static bool f413_rp_print_plan(const f413_rp_context_t* context,
                               uint16_t start_state)
{
  /* Planning is complete, so the heap storage becomes the parent chain. */
  uint16_t* chain = context->heap_states;
  size_t chain_count = 0U;
  uint16_t state = context->goal.source_state;
  unsigned int action_index = 1U;
  unsigned int diagonal_actions = 0U;

  if (chain == NULL)
  {
    return false;
  }
  while (true)
  {
    uint32_t parent;
    if (chain_count >= F413_RP_STATE_COUNT)
    {
      return false;
    }
    chain[chain_count++] = state;
    if (state == start_state)
    {
      break;
    }
    parent = context->parents[state];
    if ((parent & F413_RP_PARENT_VALID) == 0U)
    {
      return false;
    }
    state = f413_rp_parent_previous(parent);
  }

  trace_printf("[KERI-PREVIEW] A0 start-offset O=5.000mm speed=0->crawl(%lu) "
               "total=%lu.%06lu s\r\n",
               (unsigned long)context->motion->speed_mm_s[F413_RP_SPEED_CRAWL],
               (unsigned long)(context->motion->start_time_us / 1000000U),
               (unsigned long)(context->motion->start_time_us % 1000000U));

  for (size_t cursor = chain_count; cursor > 1U; cursor--)
  {
    const uint16_t destination_state = chain[cursor - 2U];
    const uint32_t parent = context->parents[destination_state];
    f413_rp_anchor_t ignored_anchor;
    f413_rp_heading_t start_heading;
    f413_rp_speed_t ignored_speed;
    if (!f413_rp_state_decode(f413_rp_parent_previous(parent),
                               &ignored_anchor, &start_heading,
                               &ignored_speed) ||
        !f413_rp_print_turn_action(
            context, action_index++, f413_rp_parent_previous(parent),
            f413_rp_parent_connector(parent), f413_rp_parent_kind(parent),
            f413_rp_parent_side(parent), f413_rp_parent_speed(parent),
            context->distances[destination_state], false))
    {
      return false;
    }
    if (f413_rp_action_is_diagonal(start_heading,
                                    f413_rp_parent_kind(parent)))
    {
      diagonal_actions++;
    }
  }

  if (context->goal.direct_connector)
  {
    f413_rp_anchor_t source;
    f413_rp_heading_t heading;
    f413_rp_speed_t speed;
    if (!f413_rp_state_decode(context->goal.source_state, &source, &heading,
                               &speed))
    {
      return false;
    }
    trace_printf(
        "[KERI-PREVIEW] A%u GOAL-STOP conn=%c%u entry-step=%u "
        "speed=%s(%lu)->0 H=%s goal=G(%u,%u) "
        "entry=%lu.%06lu s stop=%lu.%06lu s\r\n",
        action_index,
        f413_rp_is_cardinal(heading) ? 'O' : 'D',
        (unsigned int)context->goal.connector_steps,
        (unsigned int)context->goal.goal_step,
        f413_rp_speed_name(speed),
        (unsigned long)context->motion->speed_mm_s[speed],
        f413_rp_heading_name(heading),
        (unsigned int)context->goal.goal_x,
        (unsigned int)context->goal.goal_y,
        (unsigned long)(context->goal.goal_entry_us / 1000000U),
        (unsigned long)(context->goal.goal_entry_us % 1000000U),
        (unsigned long)(context->goal.stop_us / 1000000U),
        (unsigned long)(context->goal.stop_us % 1000000U));
    if (!f413_rp_is_cardinal(heading))
    {
      diagonal_actions++;
    }
  }
  else
  {
    uint32_t edge_end_us;
    uint32_t goal_turn_edge_us;
    f413_rp_anchor_t source;
    f413_rp_anchor_t connector_end;
    f413_rp_anchor_t destination;
    f413_rp_heading_t start_heading;
    f413_rp_heading_t end_heading;
    f413_rp_speed_t source_speed;
    if (!f413_rp_u32_add(context->goal.connector_us,
                          context->goal.turn_us, &goal_turn_edge_us) ||
        !f413_rp_u32_add(context->distances[context->goal.source_state],
                          goal_turn_edge_us,
                          &edge_end_us) ||
        !f413_rp_print_turn_action(
            context, action_index++, context->goal.source_state,
            context->goal.connector_steps, context->goal.kind,
            context->goal.side, context->goal.speed, edge_end_us, true) ||
        !f413_rp_state_decode(context->goal.source_state, &source,
                               &start_heading, &source_speed) ||
        !f413_rp_advance_steps(context->maze, source, start_heading,
                               context->goal.connector_steps,
                               &connector_end) ||
        !f413_rp_turn_destination(context->maze, connector_end, start_heading,
                                   context->goal.kind, context->goal.side,
                                   &destination, &end_heading))
    {
      return false;
    }
    if (f413_rp_action_is_diagonal(start_heading, context->goal.kind))
    {
      diagonal_actions++;
    }
    trace_printf(
        "[KERI-PREVIEW] A%u GOAL-STOP conn=%c%u speed=%s(%lu)->0 H=%s "
        "goal=G(%u,%u) entry=%lu.%06lu s stop=%lu.%06lu s\r\n",
        action_index,
        f413_rp_is_cardinal(end_heading) ? 'O' : 'D',
        (unsigned int)context->goal.stop_steps,
        f413_rp_speed_name(context->goal.speed),
        (unsigned long)context->motion->speed_mm_s[context->goal.speed],
        f413_rp_heading_name(end_heading),
        (unsigned int)context->goal.goal_x,
        (unsigned int)context->goal.goal_y,
        (unsigned long)(context->goal.goal_entry_us / 1000000U),
        (unsigned long)(context->goal.goal_entry_us % 1000000U),
        (unsigned long)(context->goal.stop_us / 1000000U),
        (unsigned long)(context->goal.stop_us % 1000000U));
    if (!f413_rp_is_cardinal(end_heading))
    {
      diagonal_actions++;
    }
  }
  trace_printf("[KERI-PREVIEW] OK turns=%u actions=%u diagonal-actions=%u "
               "expanded=%lu relaxed=%lu heap-peak=%u "
               "goal-entry=%lu.%06lu s stop=%lu.%06lu s\r\n",
               (unsigned int)context->goal.turn_count,
               action_index + 1U,
               diagonal_actions,
               (unsigned long)context->expanded_states,
               (unsigned long)context->relaxed_edges,
               (unsigned int)context->heap_peak,
               (unsigned long)(context->goal.goal_entry_us / 1000000U),
               (unsigned long)(context->goal.goal_entry_us % 1000000U),
               (unsigned long)(context->goal.stop_us / 1000000U),
               (unsigned long)(context->goal.stop_us % 1000000U));
  return true;
}

void f413_route_preview_run_once(void)
{
  void* scratch = NULL;
  size_t scratch_bytes = 0U;
  size_t scratch_used = 0U;
  f413_rp_arena_t arena = {0};
  f413_rp_context_t context;
  f413_rp_maze_t* maze;
  f413_rp_motion_t* motion;
  uint16_t start_state;
  f413_rp_plan_status_t plan_status;
  f413_rp_maze_source_t maze_source;
  const uint32_t start_ms = HAL_GetTick();
  bool ok = false;

  trace_printf("[KERI-PREVIEW] START read-only/no-motor mode=2 case=8 "
               "patterns=#1..#5 "
               "maze-policy=FRAM-first fallback=builtin-16MM2014CX "
               "goal-source=diagnostic-center-2x2\r\n");
  if (!f413_trace_log_try_borrow_idle_scratch(&scratch, &scratch_bytes))
  {
    trace_printf("[KERI-PREVIEW] FAIL trace capture active or scratch busy\r\n");
    return;
  }
  arena.cursor = (uint8_t*)scratch;
  arena.end = arena.cursor + scratch_bytes;
  if (scratch_bytes < F413_RP_SCRATCH_MIN_BYTES)
  {
    trace_printf("[KERI-PREVIEW] FAIL scratch=%lu required>=%lu\r\n",
                 (unsigned long)scratch_bytes,
                 (unsigned long)F413_RP_SCRATCH_MIN_BYTES);
    goto cleanup;
  }
  maze = (f413_rp_maze_t*)f413_rp_arena_alloc(
      &arena, sizeof(f413_rp_maze_t), 8U);
  motion = (f413_rp_motion_t*)f413_rp_arena_alloc(
      &arena, sizeof(f413_rp_motion_t), 8U);
  memset(&context, 0, sizeof(context));
  context.maze = maze;
  context.motion = motion;
  context.distances = (uint32_t*)f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  context.turn_counts = (uint16_t*)f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  context.parents = (uint32_t*)f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  context.settled = (uint8_t*)f413_rp_arena_alloc(
      &arena, F413_RP_SETTLED_BYTES, 1U);
  context.heap_states = (uint16_t*)f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  context.heap_positions = (uint16_t*)f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  if ((maze == NULL) || (motion == NULL) || (context.distances == NULL) ||
      (context.turn_counts == NULL) || (context.parents == NULL) ||
      (context.settled == NULL) || (context.heap_states == NULL) ||
      (context.heap_positions == NULL))
  {
    trace_printf("[KERI-PREVIEW] FAIL scratch layout\r\n");
    goto cleanup;
  }
  if (!f413_rp_load_maze(maze, &maze_source))
  {
    trace_printf("[KERI-PREVIEW] FAIL maze preparation or diagnostic goals\r\n");
    goto cleanup;
  }
  if (maze_source == F413_RP_MAZE_SOURCE_FRAM)
  {
    trace_printf("[KERI-PREVIEW] maze-source=FRAM fram-load=ok\r\n");
  }
  else
  {
    trace_printf("[KERI-PREVIEW] maze-source=builtin-16MM2014CX "
                 "data-rev=%s fram-load=fail\r\n",
                 F413_RP_BUILTIN_DATA_REV);
  }
  trace_printf("[KERI-PREVIEW] goals goal-source=diagnostic-center-2x2 ");
  for (uint8_t index = 0U; index < maze->goal_count; index++)
  {
    trace_printf("%sG(%u,%u)", (index == 0U) ? "" : ",",
                 (unsigned int)maze->goal_x[index],
                 (unsigned int)maze->goal_y[index]);
  }
  trace_printf(" wall-mismatch-normalized=%u scratch=%lu\r\n",
               (unsigned int)maze->mismatch_count,
               (unsigned long)scratch_bytes);
  if (!f413_rp_prepare_motion(motion, &arena, 8U))
  {
    trace_printf("[KERI-PREVIEW] FAIL mode2/case8 motion preparation\r\n");
    goto cleanup;
  }
  trace_printf("[KERI-PREVIEW] boundary-speeds nominal=%lu low=%lu crawl=%lu "
               "mm/s states=%u\r\n",
               (unsigned long)motion->speed_mm_s[F413_RP_SPEED_NOMINAL],
               (unsigned long)motion->speed_mm_s[F413_RP_SPEED_LOW],
               (unsigned long)motion->speed_mm_s[F413_RP_SPEED_CRAWL],
               (unsigned int)F413_RP_STATE_COUNT);
  plan_status = f413_rp_plan(&context, &start_state);
  if (plan_status == F413_RP_PLAN_NO_PATH)
  {
    trace_printf("[KERI-PREVIEW] NO-PATH expanded=%lu relaxed=%lu "
                 "(strict KERI guards + stoppable tail)\r\n",
                 (unsigned long)context.expanded_states,
                 (unsigned long)context.relaxed_edges);
    goto cleanup;
  }
  if (plan_status == F413_RP_PLAN_NO_FEASIBLE_TERMINAL)
  {
    trace_printf("[KERI-PREVIEW] NO-FEASIBLE-TERMINAL expanded=%lu "
                 "relaxed=%lu (goal crossed, no stoppable tail)\r\n",
                 (unsigned long)context.expanded_states,
                 (unsigned long)context.relaxed_edges);
    goto cleanup;
  }
  if (plan_status != F413_RP_PLAN_OK)
  {
    trace_printf("[KERI-PREVIEW] FAIL planner-internal expanded=%lu "
                 "relaxed=%lu\r\n",
                 (unsigned long)context.expanded_states,
                 (unsigned long)context.relaxed_edges);
    goto cleanup;
  }
  if (!f413_rp_print_plan(&context, start_state))
  {
    trace_printf("[KERI-PREVIEW] FAIL route reconstruction\r\n");
    goto cleanup;
  }
  ok = true;

cleanup:
  if ((scratch != NULL) && (arena.cursor != NULL))
  {
    scratch_used = (size_t)(arena.cursor - (uint8_t*)scratch);
  }
  f413_trace_log_release_idle_scratch(scratch);
  trace_printf("[KERI-PREVIEW] END status=%s scratch=released "
               "used=%lu/%lu bytes elapsed=%lu ms motors=off "
               "nvm=read-only\r\n",
               ok ? "ok" : "fail", (unsigned long)scratch_used,
               (unsigned long)scratch_bytes,
               (unsigned long)(HAL_GetTick() - start_ms));
}

bool f413_route_build_mode2_path(uint8_t case_index)
{
  void* scratch = NULL;
  size_t scratch_bytes = 0U;
  size_t scratch_used = 0U;
  size_t path_count = 0U;
  f413_rp_arena_t arena = {0};
  f413_rp_context_t context;
  f413_rp_maze_t* maze;
  f413_rp_motion_t* motion;
  f413_rp_maze_source_t maze_source;
  f413_rp_plan_status_t plan_status;
  uint16_t start_state;
  unsigned int diagonal_codes = 0U;
  bool ok = false;

  memset(path, 0, ROUTE_MAX_LEN * sizeof(path[0]));
  if ((case_index < 6U) || (case_index > 9U))
  {
    trace_printf("[KERI-RUN-PATH] FAIL unsupported mode2 case=%u (expected 6..9)\r\n",
                 (unsigned int)case_index);
    return false;
  }
  trace_printf("[KERI-RUN-PATH] START mode=2 case=%u source=FRAM goals=compiled "
               "turn-speed=nominal terminal=cardinal\r\n",
               (unsigned int)case_index);
  if (!f413_trace_log_try_borrow_idle_scratch(&scratch, &scratch_bytes))
  {
    trace_printf("[KERI-RUN-PATH] FAIL trace capture active or scratch busy\r\n");
    return false;
  }
  arena.cursor = (uint8_t*)scratch;
  arena.end = arena.cursor + scratch_bytes;
  if (scratch_bytes < F413_RP_SCRATCH_MIN_BYTES)
  {
    trace_printf("[KERI-RUN-PATH] FAIL scratch=%lu required>=%lu\r\n",
                 (unsigned long)scratch_bytes,
                 (unsigned long)F413_RP_SCRATCH_MIN_BYTES);
    goto cleanup;
  }

  maze = (f413_rp_maze_t*)f413_rp_arena_alloc(
      &arena, sizeof(f413_rp_maze_t), 8U);
  motion = (f413_rp_motion_t*)f413_rp_arena_alloc(
      &arena, sizeof(f413_rp_motion_t), 8U);
  memset(&context, 0, sizeof(context));
  context.maze = maze;
  context.motion = motion;
  context.execution_compatible = true;
  context.distances = (uint32_t*)f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  context.turn_counts = (uint16_t*)f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  context.parents = (uint32_t*)f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  context.settled = (uint8_t*)f413_rp_arena_alloc(
      &arena, F413_RP_SETTLED_BYTES, 1U);
  context.heap_states = (uint16_t*)f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  context.heap_positions = (uint16_t*)f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  if ((maze == NULL) || (motion == NULL) || (context.distances == NULL) ||
      (context.turn_counts == NULL) || (context.parents == NULL) ||
      (context.settled == NULL) || (context.heap_states == NULL) ||
      (context.heap_positions == NULL))
  {
    trace_printf("[KERI-RUN-PATH] FAIL scratch layout\r\n");
    goto cleanup;
  }
  if (!f413_rp_load_run_maze(maze, &maze_source) ||
      (maze_source != F413_RP_MAZE_SOURCE_FRAM))
  {
    trace_printf("[KERI-RUN-PATH] FAIL saved FRAM maze unavailable\r\n");
    goto cleanup;
  }
  if (!f413_rp_prepare_motion(motion, &arena, case_index))
  {
    trace_printf("[KERI-RUN-PATH] FAIL mode2/case%u motion preparation\r\n",
                 (unsigned int)case_index);
    goto cleanup;
  }
  plan_status = f413_rp_plan(&context, &start_state);
  if (plan_status != F413_RP_PLAN_OK)
  {
    trace_printf("[KERI-RUN-PATH] FAIL plan status=%u expanded=%lu relaxed=%lu\r\n",
                 (unsigned int)plan_status,
                 (unsigned long)context.expanded_states,
                 (unsigned long)context.relaxed_edges);
    goto cleanup;
  }
  if (!f413_rp_emit_legacy_path(&context, start_state, path,
                                 ROUTE_MAX_LEN, &path_count))
  {
    trace_printf("[KERI-RUN-PATH] FAIL executable path conversion\r\n");
    goto cleanup;
  }
  for (size_t index = 0U; index < path_count; index++)
  {
    if ((path[index] >= NF_LEGACY_PATH_RIGHT_45_IN) &&
        (path[index] <= NF_LEGACY_PATH_LEFT_135_OUT))
    {
      diagonal_codes++;
    }
  }
  trace_printf("[KERI-RUN-PATH] OK goal=G(%u,%u) codes=%u diagonal=%u "
               "turns=%u expanded=%lu heap-peak=%u\r\n",
               (unsigned int)context.goal.goal_x,
               (unsigned int)context.goal.goal_y,
               (unsigned int)path_count,
               diagonal_codes,
               (unsigned int)context.goal.turn_count,
               (unsigned long)context.expanded_states,
               (unsigned int)context.heap_peak);
  ok = true;

cleanup:
  if ((scratch != NULL) && (arena.cursor != NULL))
  {
    scratch_used = (size_t)(arena.cursor - (uint8_t*)scratch);
  }
  f413_trace_log_release_idle_scratch(scratch);
  if (!ok)
  {
    path[0] = 0U;
  }
  trace_printf("[KERI-RUN-PATH] END status=%s scratch=released used=%lu/%lu "
               "motors=off nvm=read-only\r\n",
               ok ? "ok" : "fail",
               (unsigned long)scratch_used,
               (unsigned long)scratch_bytes);
  return ok;
}
