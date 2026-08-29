#include "f413_trace_log.h"
#include "maze_ascii.h"
#include "slalom_time_plan_host.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Keep the production source's only HAL/NVM dependencies as host stubs. */
#define NIGHTFALL_NVM_PARAMS_H_
bool nvm_maze_load_map(uint16_t* cells, uint32_t count);
uint32_t HAL_GetTick(void);
#define ROUTE_MAX_LEN (1024U)
uint16_t path[ROUTE_MAX_LEN];

#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_route_preview.c"

/*
 * Compile the production runner's pure planning boundary into this test
 * translation unit.  This keeps the preflight private in firmware while the
 * route-to-runner integration test exercises the exact function called before
 * f413_ctrl_start().
 */
#define NIGHTFALL_F413_PATH_LINEAR_PLAN_HOST_TEST (1U)
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_path_run.c"

static uint8_t g_scratch[F413_TRACE_LOG_IDLE_SCRATCH_BYTES];
static NfRouteMaze g_host_maze;
static unsigned int g_checks;
static unsigned int g_failures;
static char g_trace_output[32768U];
static size_t g_trace_output_length;
static bool g_lease_available = true;
static bool g_nvm_load_available = true;
static unsigned int g_nvm_load_count;
static size_t g_reported_scratch_bytes = sizeof(g_scratch);
static unsigned int g_release_count;
static size_t g_verified_scratch_peak;
static bool g_runup_first_kind_valid;
static f413_rp_kind_t g_runup_first_kind;
static bool g_runup_chained_small_after_straight;
static bool g_runup_wall_state_rejoined_base;
static bool g_runup_used_recovery_pass;

#define CHECK(condition)                                                        \
  do                                                                            \
  {                                                                             \
    g_checks++;                                                                 \
    if (!(condition))                                                           \
    {                                                                           \
      g_failures++;                                                             \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);     \
    }                                                                           \
  } while (0)

#define REQUIRE(condition)                                                      \
  do                                                                            \
  {                                                                             \
    if (!(condition))                                                           \
    {                                                                           \
      g_checks++;                                                               \
      g_failures++;                                                             \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);     \
      return false;                                                             \
    }                                                                           \
    g_checks++;                                                                 \
  } while (0)

bool f413_trace_log_try_borrow_idle_scratch(void** out, size_t* out_bytes)
{
  if ((out == NULL) || (out_bytes == NULL))
  {
    return false;
  }
  if (!g_lease_available)
  {
    *out = NULL;
    *out_bytes = 0U;
    return false;
  }
  *out = g_scratch;
  *out_bytes = g_reported_scratch_bytes;
  return true;
}

void f413_trace_log_release_idle_scratch(void* scratch)
{
  CHECK(scratch == g_scratch);
  g_release_count++;
}

bool nvm_maze_load_map(uint16_t* cells, uint32_t count)
{
  g_nvm_load_count++;
  if (!g_nvm_load_available || (cells == NULL) ||
      (count != F413_RP_CELL_COUNT))
  {
    return false;
  }
  for (uint8_t y = 0U; y < F413_RP_HEIGHT; y++)
  {
    for (uint8_t x = 0U; x < F413_RP_WIDTH; x++)
    {
      cells[(size_t)y * F413_RP_WIDTH + x] =
          (uint16_t)((g_host_maze.walls[y][x] & 0x0FU) << 4U);
    }
  }
  return true;
}

uint32_t HAL_GetTick(void)
{
  return 0U;
}

int trace_printf(const char* format, ...)
{
  int written;
  va_list args;
  const size_t remaining = sizeof(g_trace_output) - g_trace_output_length;

  if (remaining == 0U)
  {
    return 0;
  }
  va_start(args, format);
  written = vsnprintf(&g_trace_output[g_trace_output_length], remaining,
                      format, args);
  va_end(args);
  if (written < 0)
  {
    return written;
  }
  if ((size_t)written >= remaining)
  {
    g_trace_output_length = sizeof(g_trace_output) - 1U;
  }
  else
  {
    g_trace_output_length += (size_t)written;
  }
  return written;
}

static unsigned int f413_test_common_diagonal_actions(
    const NfSlalomRoutePlan* plan)
{
  unsigned int count = 0U;
  for (size_t index = 0U; index < plan->action_count; index++)
  {
    const NfSlalomAction* action = &plan->actions[index];
    if (action->connector_is_diagonal ||
        (action->kind == NF_SLALOM_ACTION_45_IN) ||
        (action->kind == NF_SLALOM_ACTION_45_OUT) ||
        (action->kind == NF_SLALOM_ACTION_V90) ||
        (action->kind == NF_SLALOM_ACTION_135_IN) ||
        (action->kind == NF_SLALOM_ACTION_135_OUT))
    {
      count++;
    }
  }
  return count;
}

static void f413_test_reset_trace_capture(void)
{
  memset(g_trace_output, 0, sizeof(g_trace_output));
  g_trace_output_length = 0U;
}

static int f413_test_hex_nibble(char value)
{
  if ((value >= '0') && (value <= '9'))
  {
    return value - '0';
  }
  if ((value >= 'A') && (value <= 'F'))
  {
    return value - 'A' + 10;
  }
  if ((value >= 'a') && (value <= 'f'))
  {
    return value - 'a' + 10;
  }
  return -1;
}

static bool f413_test_load_search_dump(const char* path_name)
{
  FILE* input;
  char line[256U];
  bool rows[F413_RP_HEIGHT] = {false};
  unsigned int row_count = 0U;

  if (path_name == NULL)
  {
    return false;
  }
  input = fopen(path_name, "r");
  if (input == NULL)
  {
    return false;
  }
  memset(&g_host_maze, 0, sizeof(g_host_maze));
  g_host_maze.width = F413_RP_WIDTH;
  g_host_maze.height = F413_RP_HEIGHT;
  while (fgets(line, sizeof(line), input) != NULL)
  {
    unsigned int y;
    char data[(2U * F413_RP_WIDTH) + 1U];

    if (sscanf(line, "[SEARCH-DUMP] y=%u:%32s", &y, data) != 2)
    {
      continue;
    }
    if ((y >= F413_RP_HEIGHT) || rows[y] ||
        (strlen(data) != 2U * F413_RP_WIDTH))
    {
      fclose(input);
      return false;
    }
    for (uint8_t x = 0U; x < F413_RP_WIDTH; x++)
    {
      /* UART dumps one full map byte: high nibble=search flags, low=NESW. */
      const int wall_nibble = f413_test_hex_nibble(data[(2U * x) + 1U]);
      if (wall_nibble < 0)
      {
        fclose(input);
        return false;
      }
      g_host_maze.walls[y][x] = (uint8_t)wall_nibble;
    }
    rows[y] = true;
    row_count++;
  }
  fclose(input);
  g_host_maze.goals[GOAL1_Y][GOAL1_X] = true;
  return row_count == F413_RP_HEIGHT;
}

static NfSlalomActionKind f413_test_common_kind(f413_rp_kind_t kind)
{
  static const NfSlalomActionKind kinds[F413_RP_KIND_COUNT] = {
      NF_SLALOM_ACTION_LARGE_90,
      NF_SLALOM_ACTION_LARGE_180,
      NF_SLALOM_ACTION_45_IN,
      NF_SLALOM_ACTION_45_OUT,
      NF_SLALOM_ACTION_V90,
      NF_SLALOM_ACTION_135_IN,
      NF_SLALOM_ACTION_135_OUT,
      NF_SLALOM_ACTION_SMALL_90,
  };
  return kinds[(unsigned int)kind];
}

static NfSlalomTurnSpeedMode f413_test_common_speed(f413_rp_speed_t speed)
{
  static const NfSlalomTurnSpeedMode speeds[F413_RP_SPEED_COUNT] = {
      NF_SLALOM_TURN_SPEED_NOMINAL,
      NF_SLALOM_TURN_SPEED_LOW,
      NF_SLALOM_TURN_SPEED_CRAWL,
  };
  return speeds[(unsigned int)speed];
}

static bool f413_test_copy_fixture_goals(f413_rp_maze_t* maze)
{
  memset(maze->goal_bits, 0, sizeof(maze->goal_bits));
  maze->goal_count = 0U;
  for (uint8_t y = 0U; y < F413_RP_HEIGHT; y++)
  {
    for (uint8_t x = 0U; x < F413_RP_WIDTH; x++)
    {
      if (g_host_maze.goals[y][x] && !f413_rp_add_goal(maze, x, y))
      {
        return false;
      }
    }
  }
  return maze->goal_count != 0U;
}

static bool f413_test_action_parity(const f413_rp_context_t* compact,
                                    uint16_t start_state,
                                    const NfSlalomRoutePlan* common)
{
  uint16_t* chain = compact->heap_states;
  size_t chain_count = 0U;
  uint16_t state = compact->goal.source_state;
  size_t common_index = 1U;

  REQUIRE(chain != NULL);
  while (true)
  {
    REQUIRE(chain_count < F413_RP_STATE_COUNT);
    chain[chain_count++] = state;
    if (state == start_state)
    {
      break;
    }
    REQUIRE((compact->parents[state] & F413_RP_PARENT_VALID) != 0U);
    state = f413_rp_parent_previous(compact->parents[state]);
  }

  REQUIRE(common->action_count == chain_count + 1U);
  CHECK(common->actions[0].kind == NF_SLALOM_ACTION_START_OFFSET);
  CHECK(common->actions[0].duration_us ==
        compact->motion->precomputed->start_time_us);
  CHECK(compact->distances[start_state] ==
        compact->motion->precomputed->start_time_us);

  for (size_t cursor = chain_count; cursor > 1U; cursor--, common_index++)
  {
    const uint16_t destination_state = chain[cursor - 2U];
    const uint32_t parent = compact->parents[destination_state];
    const uint16_t source_state = f413_rp_parent_previous(parent);
    const f413_rp_kind_t kind = f413_rp_parent_kind(parent);
    const f413_rp_side_t side = f413_rp_parent_side(parent);
    const f413_rp_speed_t turn_speed = f413_rp_parent_speed(parent);
    const uint16_t connector_steps = f413_rp_parent_connector(parent);
    const NfSlalomAction* action = &common->actions[common_index];
    f413_rp_anchor_t source;
    f413_rp_anchor_t connector_end;
    f413_rp_anchor_t destination;
    f413_rp_heading_t start_heading;
    f413_rp_heading_t end_heading;
    f413_rp_speed_t source_speed;
    uint32_t connector_us;
    uint32_t turn_us;

    REQUIRE(source_state == chain[cursor - 1U]);
    REQUIRE(f413_rp_state_decode(source_state, &source, &start_heading,
                                  &source_speed));
    REQUIRE(f413_rp_advance_steps(compact->maze, source, start_heading,
                                   connector_steps, &connector_end));
    REQUIRE(f413_rp_turn_destination(compact->maze, connector_end,
                                      start_heading, kind, side,
                                      &destination, &end_heading));
    REQUIRE(f413_rp_turn_connector_time_us(
                compact->motion, start_heading, source_speed, turn_speed,
                connector_steps, kind, &connector_us));
    REQUIRE(connector_us != F413_RP_INF);
    REQUIRE(f413_rp_seconds_to_us(
                compact->motion->precomputed->timing_plan[turn_speed][kind]
                    .total_time_s,
                &turn_us));

    CHECK(action->kind == f413_test_common_kind(kind));
    CHECK(action->side == ((side == F413_RP_SIDE_LEFT) ?
                           NF_ROUTE_SIDE_LEFT : NF_ROUTE_SIDE_RIGHT));
    CHECK(action->turn_speed_mode == f413_test_common_speed(turn_speed));
    CHECK(action->connector_steps == connector_steps);
    CHECK(action->connector_is_diagonal ==
          !f413_rp_is_cardinal(start_heading));
    CHECK(action->start_anchor.half_x == source.half_x);
    CHECK(action->start_anchor.half_y == source.half_y);
    CHECK(action->connector_end_anchor.half_x == connector_end.half_x);
    CHECK(action->connector_end_anchor.half_y == connector_end.half_y);
    CHECK(action->end_anchor.half_x == destination.half_x);
    CHECK(action->end_anchor.half_y == destination.half_y);
    CHECK((unsigned int)action->start_heading ==
          (unsigned int)start_heading);
    CHECK((unsigned int)action->end_heading == (unsigned int)end_heading);
    if (!f413_rp_turn_uses_orthogonal_approach(kind))
    {
      CHECK(action->connector_time_us == connector_us);
      CHECK(action->duration_us == (uint64_t)connector_us + turn_us);
    }
    CHECK(action->turn_time_us == turn_us);
    CHECK(compact->distances[destination_state] ==
          compact->distances[source_state] + connector_us + turn_us);
  }

  {
    const NfSlalomAction* terminal = &common->actions[common_index];
    f413_rp_anchor_t source;
    f413_rp_heading_t heading;
    f413_rp_speed_t speed;

    REQUIRE(compact->goal.direct_connector);
    REQUIRE(f413_rp_state_decode(compact->goal.source_state, &source,
                                  &heading, &speed));
    CHECK(terminal->kind == NF_SLALOM_ACTION_GOAL_STOP);
    CHECK(terminal->connector_steps == compact->goal.connector_steps);
    CHECK(terminal->connector_is_diagonal ==
          !f413_rp_is_cardinal(heading));
    CHECK(terminal->start_anchor.half_x == source.half_x);
    CHECK(terminal->start_anchor.half_y == source.half_y);
    CHECK(terminal->connector_time_us == compact->goal.connector_us);
    CHECK(terminal->duration_us == compact->goal.connector_us);
    CHECK(terminal->has_goal_cross);
    CHECK(terminal->goal_phase == NF_SLALOM_GOAL_CONNECTOR);
    CHECK(terminal->goal_cross_time_us == compact->goal.goal_cross_edge_us);
    CHECK(terminal->goal_x == compact->goal.goal_x);
    CHECK(terminal->goal_y == compact->goal.goal_y);
  }
  return true;
}

static bool f413_test_kerilab_2014_parity(const char* maze_path)
{
  NfMazeAsciiInfo info;
  char error[192];
  NfSlalomPlannerConfig config;
  const NfAuditProfile* profile;
  const NfSlalomPlannerRequest request = {
      0U, 0U, NF_SLALOM_HEADING_NORTH,
  };
  NfSlalomRoutePlan common;
  NfSlalomValidation validation;
  f413_rp_arena_t arena = {g_scratch, g_scratch + sizeof(g_scratch)};
  f413_rp_context_t compact = {0};
  f413_rp_maze_t* compact_maze;
  f413_rp_motion_t* motion;
  f413_rp_maze_source_t maze_source;
  uint16_t start_state;
  uint8_t* reconstruction_mark;
  const char* diagonal_text;
  unsigned int reported_diagonal_actions = 0U;

  REQUIRE(nf_maze_ascii_load(maze_path, &g_host_maze, &info, error,
                              sizeof(error)) == NF_MAZE_ASCII_OK);
  REQUIRE(g_host_maze.width == F413_RP_WIDTH);
  REQUIRE(g_host_maze.height == F413_RP_HEIGHT);
  REQUIRE(info.start_x == START_X);
  REQUIRE(info.start_y == START_Y);
  REQUIRE(nf_host_slalom_make_config(
      "f413-preorder-mode2", 8U, NF_SLALOM_ENABLE_SHORTEST_1_TO_5,
      &config, &profile, error, sizeof(error)));
  (void)profile;
  REQUIRE(nf_slalom_time_plan(&g_host_maze, &config, &request, &common) ==
          NF_SLALOM_PLAN_OK);
  REQUIRE(nf_slalom_route_validate(&g_host_maze, &config, &request, &common,
                                    &validation));

  compact_maze = f413_rp_arena_alloc(&arena, sizeof(*compact_maze), 8U);
  motion = f413_rp_arena_alloc(&arena, sizeof(*motion), 8U);
  compact.maze = compact_maze;
  compact.motion = motion;
  compact.distances = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  compact.turn_counts = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  compact.parents = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  compact.settled = f413_rp_arena_alloc(
      &arena, F413_RP_SETTLED_BYTES, 1U);
  compact.heap_states = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  compact.heap_positions = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  REQUIRE(compact_maze != NULL);
  REQUIRE(motion != NULL);
  REQUIRE(compact.distances != NULL);
  REQUIRE(compact.turn_counts != NULL);
  REQUIRE(compact.parents != NULL);
  REQUIRE(compact.settled != NULL);
  REQUIRE(compact.heap_states != NULL);
  REQUIRE(compact.heap_positions != NULL);
  REQUIRE(f413_rp_load_maze(compact_maze, &maze_source));
  CHECK(maze_source == F413_RP_MAZE_SOURCE_FRAM);
  CHECK(compact_maze->goal_count == 4U);
  for (uint8_t y = 0U; y < F413_RP_HEIGHT; y++)
  {
    for (uint8_t x = 0U; x < F413_RP_WIDTH; x++)
    {
      const size_t cell_index = (size_t)y * F413_RP_WIDTH + x;
      CHECK(g_f413_rp_builtin_16mm2014cx_walls[cell_index] ==
            (g_host_maze.walls[y][x] & 0x0FU));
      CHECK(f413_rp_goal_at(compact_maze, x, y) ==
            g_host_maze.goals[y][x]);
    }
  }
  REQUIRE(f413_rp_prepare_motion(motion, &arena, 8U));
  g_verified_scratch_peak = (size_t)(arena.cursor - g_scratch);
  CHECK(g_verified_scratch_peak <= F413_RP_SCRATCH_MIN_BYTES);
  CHECK(g_verified_scratch_peak <= (140U * 1024U));
  REQUIRE(f413_rp_plan(&compact, &start_state) == F413_RP_PLAN_OK);
  CHECK(compact.heap_peak > 0U);
  CHECK(compact.heap_peak <= F413_RP_STATE_COUNT);
  CHECK(compact.goal.goal_x == common.goal_x);
  CHECK(compact.goal.goal_y == common.goal_y);
  /* The runner reserves wall-end approach time on orthogonal large turns. */
  CHECK(compact.goal.goal_entry_us == common.goal_entry_us + 300000ULL);
  CHECK(compact.goal.stop_us == common.stop_us + 300000ULL);
  CHECK(common.goal_x == 8U);
  CHECK(common.goal_y == 8U);
  CHECK(common.goal_entry_us == 10464909ULL);
  CHECK(common.stop_us == 10724467ULL);
  CHECK(compact.goal.goal_entry_us == 10764909UL);
  CHECK(compact.goal.stop_us == 11024467UL);
  CHECK(common.action_count == 15U);
  CHECK(f413_test_common_diagonal_actions(&common) == 8U);
  reconstruction_mark = arena.cursor;
  REQUIRE(f413_test_action_parity(&compact, start_state, &common));
  f413_test_reset_trace_capture();
  REQUIRE(f413_rp_print_plan(&compact, start_state));
  CHECK(arena.cursor == reconstruction_mark);
  diagonal_text = strstr(g_trace_output, "diagonal-actions=");
  REQUIRE(diagonal_text != NULL);
  REQUIRE(sscanf(diagonal_text, "diagonal-actions=%u",
                 &reported_diagonal_actions) == 1);
  CHECK(reported_diagonal_actions ==
        f413_test_common_diagonal_actions(&common));
  CHECK((size_t)(arena.cursor - g_scratch) <= sizeof(g_scratch));
  return true;
}

static bool f413_test_public_cleanup_and_metadata(const char* maze_path)
{
  NfMazeAsciiInfo info;
  char error[192];

  REQUIRE(nf_maze_ascii_load(maze_path, &g_host_maze, &info, error,
                              sizeof(error)) == NF_MAZE_ASCII_OK);
  g_lease_available = true;
  g_nvm_load_available = true;
  g_nvm_load_count = 0U;
  g_reported_scratch_bytes = sizeof(g_scratch);
  g_release_count = 0U;
  f413_test_reset_trace_capture();
  f413_route_preview_run_once();
  CHECK(g_release_count == 1U);
  CHECK(g_nvm_load_count == 1U);
  CHECK(strstr(g_trace_output,
               "START read-only/no-motor mode=2 case=8") != NULL);
  CHECK(strstr(g_trace_output,
               "patterns=#1..#5+gated-small90") != NULL);
  CHECK(strstr(g_trace_output,
               "maze-policy=FRAM-first fallback=builtin-16MM2014CX") !=
        NULL);
  CHECK(strstr(g_trace_output, "maze-source=FRAM fram-load=ok") != NULL);
  CHECK(strstr(g_trace_output,
               "goal-source=diagnostic-center-2x2") != NULL);
  CHECK(strstr(g_trace_output, "[KERI-PREVIEW] OK ") != NULL);
  CHECK(strstr(g_trace_output,
               "END status=ok scratch=released used=") != NULL);
  CHECK(strstr(g_trace_output, "elapsed=0 ms motors=off nvm=read-only") !=
        NULL);

  g_nvm_load_available = false;
  g_nvm_load_count = 0U;
  g_release_count = 0U;
  f413_test_reset_trace_capture();
  f413_route_preview_run_once();
  CHECK(g_release_count == 1U);
  CHECK(g_nvm_load_count == 1U);
  CHECK(strstr(g_trace_output,
               "maze-source=builtin-16MM2014CX data-rev="
               F413_RP_BUILTIN_DATA_REV " fram-load=fail") != NULL);
  CHECK(strstr(g_trace_output, "goal=G(8,8)") != NULL);
  CHECK(strstr(g_trace_output,
               "OK turns=13 actions=15 diagonal-actions=8") != NULL);
  CHECK(strstr(g_trace_output,
               "goal-entry=10.764909 s stop=11.024467 s") != NULL);
  CHECK(strstr(g_trace_output,
               "wall-mismatch-normalized=0") != NULL);
  CHECK(strstr(g_trace_output,
               "END status=ok scratch=released used=") != NULL);
  CHECK(strstr(g_trace_output, "motors=off nvm=read-only") != NULL);
  g_nvm_load_available = true;

  g_reported_scratch_bytes = F413_RP_SCRATCH_MIN_BYTES - 1U;
  g_nvm_load_count = 0U;
  g_release_count = 0U;
  f413_test_reset_trace_capture();
  f413_route_preview_run_once();
  CHECK(g_release_count == 1U);
  CHECK(g_nvm_load_count == 0U);
  CHECK(strstr(g_trace_output, "FAIL scratch=") != NULL);
  CHECK(strstr(g_trace_output, "END status=fail scratch=released") != NULL);
  g_reported_scratch_bytes = sizeof(g_scratch);

  g_lease_available = false;
  g_nvm_load_count = 0U;
  g_release_count = 0U;
  f413_test_reset_trace_capture();
  f413_route_preview_run_once();
  CHECK(g_release_count == 0U);
  CHECK(g_nvm_load_count == 0U);
  CHECK(strstr(g_trace_output,
               "FAIL trace capture active or scratch busy") != NULL);
  g_lease_available = true;
  return true;
}

static bool f413_test_expect_status(f413_rp_plan_status_t expected)
{
  f413_rp_arena_t arena = {g_scratch, g_scratch + sizeof(g_scratch)};
  f413_rp_context_t context = {0};
  f413_rp_maze_source_t maze_source;
  f413_rp_maze_t* maze = f413_rp_arena_alloc(
      &arena, sizeof(f413_rp_maze_t), 8U);
  f413_rp_motion_t* motion = f413_rp_arena_alloc(
      &arena, sizeof(f413_rp_motion_t), 8U);
  uint16_t start_state;

  context.maze = maze;
  context.motion = motion;
  context.distances = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  context.turn_counts = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  context.parents = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  context.settled = f413_rp_arena_alloc(
      &arena, F413_RP_SETTLED_BYTES, 1U);
  context.heap_states = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  context.heap_positions = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  REQUIRE(maze != NULL);
  REQUIRE(motion != NULL);
  REQUIRE(context.distances != NULL);
  REQUIRE(context.turn_counts != NULL);
  REQUIRE(context.parents != NULL);
  REQUIRE(context.settled != NULL);
  REQUIRE(context.heap_states != NULL);
  REQUIRE(context.heap_positions != NULL);
  REQUIRE(f413_rp_load_maze(maze, &maze_source));
  CHECK(maze_source == F413_RP_MAZE_SOURCE_FRAM);
  REQUIRE(f413_test_copy_fixture_goals(maze));
  REQUIRE(f413_rp_prepare_motion(motion, &arena, 8U));
  CHECK(f413_rp_plan(&context, &start_state) == expected);
  return true;
}

static bool f413_test_terminal_statuses(void)
{
  REQUIRE(nf_route_maze_init(&g_host_maze, F413_RP_WIDTH,
                              F413_RP_HEIGHT));
  REQUIRE(nf_route_maze_add_boundaries(&g_host_maze));
  for (uint8_t coordinate = 0U; coordinate < 2U; coordinate++)
  {
    REQUIRE(nf_route_maze_set_wall(&g_host_maze, 1U, coordinate,
                                    NF_ROUTE_DIR_EAST));
    REQUIRE(nf_route_maze_set_wall(&g_host_maze, coordinate, 1U,
                                    NF_ROUTE_DIR_NORTH));
  }
  g_host_maze.goals[1][1] = true;
  REQUIRE(f413_test_expect_status(F413_RP_PLAN_NO_FEASIBLE_TERMINAL));

  REQUIRE(nf_route_maze_set_wall(&g_host_maze, 0U, 0U,
                                  NF_ROUTE_DIR_NORTH));
  REQUIRE(nf_route_maze_set_wall(&g_host_maze, 0U, 0U,
                                  NF_ROUTE_DIR_EAST));
  REQUIRE(f413_test_expect_status(F413_RP_PLAN_NO_PATH));
  return true;
}

static bool f413_test_heap_order_and_relax(void)
{
  f413_rp_arena_t arena = {g_scratch, g_scratch + sizeof(g_scratch)};
  f413_rp_context_t context = {0};
  uint16_t popped;
  uint32_t first_parent;

  context.distances = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  context.turn_counts = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  context.parents = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  context.settled = f413_rp_arena_alloc(
      &arena, F413_RP_SETTLED_BYTES, 1U);
  context.heap_states = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  context.heap_positions = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  REQUIRE(context.distances != NULL);
  REQUIRE(context.turn_counts != NULL);
  REQUIRE(context.parents != NULL);
  REQUIRE(context.settled != NULL);
  REQUIRE(context.heap_states != NULL);
  REQUIRE(context.heap_positions != NULL);
  memset(context.distances, 0xFF,
         F413_RP_STATE_COUNT * sizeof(uint32_t));
  memset(context.turn_counts, 0xFF,
         F413_RP_STATE_COUNT * sizeof(uint16_t));
  memset(context.parents, 0, F413_RP_STATE_COUNT * sizeof(uint32_t));
  memset(context.settled, 0, F413_RP_SETTLED_BYTES);
  memset(context.heap_positions, 0xFF,
         F413_RP_STATE_COUNT * sizeof(uint16_t));

  context.distances[30U] = 100U;
  context.turn_counts[30U] = 2U;
  context.distances[20U] = 100U;
  context.turn_counts[20U] = 1U;
  context.distances[10U] = 100U;
  context.turn_counts[10U] = 1U;
  REQUIRE(f413_rp_heap_push_or_decrease(&context, 30U));
  REQUIRE(f413_rp_heap_push_or_decrease(&context, 20U));
  REQUIRE(f413_rp_heap_push_or_decrease(&context, 10U));
  REQUIRE(f413_rp_heap_peek(&context, &popped));
  CHECK(popped == 10U);
  context.distances[30U] = 99U;
  REQUIRE(f413_rp_heap_push_or_decrease(&context, 30U));
  REQUIRE(f413_rp_heap_pop(&context, &popped));
  CHECK(popped == 30U);
  CHECK(context.heap_positions[30U] == UINT16_MAX);
  REQUIRE(f413_rp_heap_pop(&context, &popped));
  CHECK(popped == 10U);
  REQUIRE(f413_rp_heap_pop(&context, &popped));
  CHECK(popped == 20U);
  CHECK(context.heap_size == 0U);
  CHECK(context.heap_peak == 3U);

  memset(context.distances, 0xFF,
         F413_RP_STATE_COUNT * sizeof(uint32_t));
  memset(context.turn_counts, 0xFF,
         F413_RP_STATE_COUNT * sizeof(uint16_t));
  memset(context.parents, 0, F413_RP_STATE_COUNT * sizeof(uint32_t));
  memset(context.settled, 0, F413_RP_SETTLED_BYTES);
  memset(context.heap_positions, 0xFF,
         F413_RP_STATE_COUNT * sizeof(uint16_t));
  context.heap_size = 0U;
  context.heap_peak = 0U;
  context.relaxed_edges = 0U;
  context.distances[1U] = 10U;
  context.turn_counts[1U] = 2U;
  REQUIRE(f413_rp_relax(&context, 1U, 2U, 0U,
                         F413_RP_KIND_LARGE_90, F413_RP_SIDE_RIGHT,
                         F413_RP_SPEED_CRAWL, 5U));
  first_parent = context.parents[2U];
  CHECK(context.distances[2U] == 15U);
  CHECK(context.turn_counts[2U] == 3U);
  CHECK(context.relaxed_edges == 1U);

  context.distances[6U] = 15U;
  context.turn_counts[6U] = 2U;
  REQUIRE(f413_rp_heap_push_or_decrease(&context, 6U));
  REQUIRE(f413_rp_heap_peek(&context, &popped));
  CHECK(popped == 6U);

  context.distances[3U] = 10U;
  context.turn_counts[3U] = 2U;
  REQUIRE(f413_rp_relax(&context, 3U, 2U, 1U,
                         F413_RP_KIND_LARGE_180, F413_RP_SIDE_LEFT,
                         F413_RP_SPEED_LOW, 5U));
  CHECK(context.parents[2U] == first_parent);
  CHECK(context.relaxed_edges == 1U);

  context.distances[4U] = 10U;
  context.turn_counts[4U] = 1U;
  REQUIRE(f413_rp_relax(&context, 4U, 2U, 2U,
                         F413_RP_KIND_45_IN, F413_RP_SIDE_LEFT,
                         F413_RP_SPEED_LOW, 5U));
  CHECK(context.distances[2U] == 15U);
  CHECK(context.turn_counts[2U] == 2U);
  CHECK(context.parents[2U] != first_parent);
  CHECK(f413_rp_parent_previous(context.parents[2U]) == 4U);
  CHECK(context.relaxed_edges == 2U);
  REQUIRE(f413_rp_heap_peek(&context, &popped));
  CHECK(popped == 2U);

  f413_rp_set_settled(&context, 2U);
  context.distances[5U] = 9U;
  context.turn_counts[5U] = 0U;
  CHECK(!f413_rp_relax(&context, 5U, 2U, 0U,
                        F413_RP_KIND_LARGE_90, F413_RP_SIDE_RIGHT,
                        F413_RP_SPEED_CRAWL, 5U));
  CHECK(context.distances[2U] == 15U);
  CHECK(context.turn_counts[2U] == 2U);
  CHECK(f413_rp_parent_previous(context.parents[2U]) == 4U);
  return true;
}

static bool f413_test_public_run_path_build(const char* maze_path)
{
  NfMazeAsciiInfo info;
  char error[192];

  REQUIRE(nf_maze_ascii_load(maze_path, &g_host_maze, &info, error,
                              sizeof(error)) == NF_MAZE_ASCII_OK);
  g_lease_available = true;
  g_nvm_load_available = true;
  for (uint8_t case_index = 6U; case_index <= 9U; case_index++)
  {
    NfLegacyPathResult validation;
    g_release_count = 0U;
    f413_test_reset_trace_capture();
    REQUIRE(f413_route_build_mode2_path(case_index));
    CHECK(g_release_count == 1U);
    validation = nf_legacy_path_validate(path, ROUTE_MAX_LEN);
    CHECK(validation.status == NF_LEGACY_PATH_OK);
    CHECK(strstr(g_trace_output, "[KERI-RUN-PATH] OK ") != NULL);
    CHECK(strstr(g_trace_output,
                 "turn-speed=nominal+gated-small90(low)") != NULL);
    CHECK(strstr(g_trace_output, "diagonal-turns=") != NULL);
    CHECK(strstr(g_trace_output, "diagonal-straights=") != NULL);
    CHECK(strstr(g_trace_output, "small-recovery=") != NULL);
    CHECK(strstr(g_trace_output, "elapsed=0 ms") != NULL);
    CHECK(strstr(g_trace_output, "motors=off nvm=read-only") != NULL);
  }

  g_nvm_load_available = false;
  path[0] = 999U;
  g_release_count = 0U;
  f413_test_reset_trace_capture();
  CHECK(!f413_route_build_mode2_path(8U));
  CHECK(path[0] == 0U);
  CHECK(g_release_count == 1U);
  CHECK(strstr(g_trace_output, "saved FRAM maze unavailable") != NULL);
  g_nvm_load_available = true;
  return true;
}

static bool f413_test_execution_diagonal_path_exists(void)
{
  f413_rp_arena_t arena = {g_scratch, g_scratch + sizeof(g_scratch)};
  f413_rp_context_t context = {0};
  f413_rp_maze_source_t maze_source;
  f413_rp_maze_t* maze = f413_rp_arena_alloc(
      &arena, sizeof(f413_rp_maze_t), 8U);
  f413_rp_motion_t* motion = f413_rp_arena_alloc(
      &arena, sizeof(f413_rp_motion_t), 8U);
  bool found = false;

  REQUIRE(nf_route_maze_init(&g_host_maze, F413_RP_WIDTH,
                              F413_RP_HEIGHT));
  REQUIRE(nf_route_maze_add_boundaries(&g_host_maze));
  context.maze = maze;
  context.motion = motion;
  context.execution_compatible = true;
  context.allow_small_fallback = true;
  context.distances = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  context.turn_counts = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  context.parents = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  context.settled = f413_rp_arena_alloc(
      &arena, F413_RP_SETTLED_BYTES, 1U);
  context.heap_states = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  context.heap_positions = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  REQUIRE(maze != NULL);
  REQUIRE(motion != NULL);
  REQUIRE(context.distances != NULL);
  REQUIRE(context.turn_counts != NULL);
  REQUIRE(context.parents != NULL);
  REQUIRE(context.settled != NULL);
  REQUIRE(context.heap_states != NULL);
  REQUIRE(context.heap_positions != NULL);
  REQUIRE(f413_rp_load_walls(maze, &maze_source, false));
  REQUIRE(maze_source == F413_RP_MAZE_SOURCE_FRAM);
  REQUIRE(f413_rp_prepare_motion(motion, &arena, 8U));

  for (uint8_t y = 0U; (y < F413_RP_HEIGHT) && !found; y++)
  {
    for (uint8_t x = 0U; (x < F413_RP_WIDTH) && !found; x++)
    {
      uint16_t start_state;
      size_t path_count = 0U;

      if ((x == START_X) && (y == START_Y))
      {
        continue;
      }
      memset(maze->goal_bits, 0, sizeof(maze->goal_bits));
      maze->goal_count = 0U;
      REQUIRE(f413_rp_add_goal(maze, x, y));
      if ((f413_rp_plan_execution(&context, &start_state) !=
           F413_RP_PLAN_OK) ||
          !f413_rp_emit_legacy_path(&context, start_state, path,
                                     ROUTE_MAX_LEN, &path_count))
      {
        continue;
      }
      for (size_t index = 0U; index < path_count; index++)
      {
        if ((path[index] > NF_LEGACY_PATH_DIAGONAL_STRAIGHT_BASE) &&
            (path[index] <= NF_LEGACY_PATH_DIAGONAL_STRAIGHT_MAX))
        {
          found = true;
          break;
        }
      }
    }
  }
  CHECK(found);
  if (found)
  {
    const NfLegacyPathResult validation =
        nf_legacy_path_validate(path, ROUTE_MAX_LEN);
    CHECK(validation.status == NF_LEGACY_PATH_OK);
  }
  return true;
}

static bool f413_test_build_runup_case(uint8_t case_index,
                                       bool allow_small_fallback,
                                       f413_rp_plan_status_t* out_status,
                                       uint32_t* out_expanded,
                                       uint32_t* out_relaxed)
{
  f413_rp_arena_t arena = {g_scratch, g_scratch + sizeof(g_scratch)};
  f413_rp_context_t context = {0};
  f413_rp_maze_source_t maze_source;
  f413_rp_maze_t* maze = f413_rp_arena_alloc(
      &arena, sizeof(f413_rp_maze_t), 8U);
  f413_rp_motion_t* motion = f413_rp_arena_alloc(
      &arena, sizeof(f413_rp_motion_t), 8U);
  uint16_t start_state;
  size_t path_count = 0U;

  memset(path, 0, sizeof(path));
  g_runup_first_kind_valid = false;
  g_runup_chained_small_after_straight = false;
  g_runup_wall_state_rejoined_base = false;
  g_runup_used_recovery_pass = false;
  context.maze = maze;
  context.motion = motion;
  context.execution_compatible = true;
  context.allow_small_fallback = allow_small_fallback;
  context.distances = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  context.turn_counts = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  context.parents = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint32_t), 4U);
  context.settled = f413_rp_arena_alloc(
      &arena, F413_RP_SETTLED_BYTES, 1U);
  context.heap_states = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  context.heap_positions = f413_rp_arena_alloc(
      &arena, F413_RP_STATE_COUNT * sizeof(uint16_t), 2U);
  REQUIRE(maze != NULL);
  REQUIRE(motion != NULL);
  REQUIRE(context.distances != NULL);
  REQUIRE(context.turn_counts != NULL);
  REQUIRE(context.parents != NULL);
  REQUIRE(context.settled != NULL);
  REQUIRE(context.heap_states != NULL);
  REQUIRE(context.heap_positions != NULL);
  REQUIRE(f413_rp_load_run_maze(maze, &maze_source));
  REQUIRE(maze_source == F413_RP_MAZE_SOURCE_FRAM);
  REQUIRE(f413_rp_prepare_motion(motion, &arena, case_index));
  *out_status = f413_rp_plan_execution(&context, &start_state);
  g_runup_used_recovery_pass = context.small_recovery_pass;
  *out_expanded = context.expanded_states;
  *out_relaxed = context.relaxed_edges;
  for (uint16_t state = 0U; state < F413_RP_BASE_STATE_COUNT; state++)
  {
    const uint32_t parent = context.parents[state];
    if (((parent & F413_RP_PARENT_VALID) != 0U) &&
        (f413_rp_parent_previous(parent) >= F413_RP_BASE_STATE_COUNT))
    {
      g_runup_wall_state_rejoined_base = true;
      break;
    }
  }
  if (*out_status == F413_RP_PLAN_OK)
  {
    uint16_t state = context.goal.source_state;
    while (state != start_state)
    {
      const uint32_t parent = context.parents[state];
      const uint16_t previous = f413_rp_parent_previous(parent);
      const f413_rp_kind_t kind = f413_rp_parent_kind(parent);

      REQUIRE((parent & F413_RP_PARENT_VALID) != 0U);
      if (previous == start_state)
      {
        g_runup_first_kind = kind;
        g_runup_first_kind_valid = true;
      }
      if ((state >= F413_RP_BASE_STATE_COUNT) &&
          (previous >= F413_RP_BASE_STATE_COUNT) &&
          (kind == F413_RP_KIND_SMALL_90) &&
          (f413_rp_parent_connector(parent) != 0U))
      {
        g_runup_chained_small_after_straight = true;
      }
      if ((state < F413_RP_BASE_STATE_COUNT) &&
          (previous >= F413_RP_BASE_STATE_COUNT))
      {
        g_runup_wall_state_rejoined_base = true;
      }
      state = previous;
    }
    REQUIRE(f413_rp_emit_legacy_path(&context, start_state, path,
                                      ROUTE_MAX_LEN, &path_count));
    REQUIRE(path_count != 0U);
  }
  return true;
}

static bool f413_test_path_has_diagonal_turn(void)
{
  for (size_t index = 0U; index < ROUTE_MAX_LEN; index++)
  {
    const uint16_t code = path[index];
    if (code == 0U)
    {
      break;
    }
    if ((code >= NF_LEGACY_PATH_RIGHT_45_IN) &&
        (code <= NF_LEGACY_PATH_LEFT_135_OUT))
    {
      return true;
    }
  }
  return false;
}

static bool f413_test_path_has_diagonal_straight(void)
{
  for (size_t index = 0U; index < ROUTE_MAX_LEN; index++)
  {
    const uint16_t code = path[index];
    if (code == 0U)
    {
      break;
    }
    if ((code > NF_LEGACY_PATH_DIAGONAL_STRAIGHT_BASE) &&
        (code <= NF_LEGACY_PATH_DIAGONAL_STRAIGHT_MAX))
    {
      return true;
    }
  }
  return false;
}

static bool f413_test_no_adjacent_orthogonal_straights(void)
{
  for (size_t index = 1U; index < ROUTE_MAX_LEN; index++)
  {
    const uint16_t previous = path[index - 1U];
    const uint16_t current = path[index];
    if (current == 0U)
    {
      break;
    }
    if ((previous > NF_LEGACY_PATH_STRAIGHT_BASE) &&
        (previous <= NF_LEGACY_PATH_STRAIGHT_MAX) &&
        (current > NF_LEGACY_PATH_STRAIGHT_BASE) &&
        (current <= NF_LEGACY_PATH_STRAIGHT_MAX))
    {
      return false;
    }
  }
  return true;
}

static bool f413_test_state_and_parent_roundtrip(void)
{
  for (uint16_t state = 0U; state < F413_RP_STATE_COUNT; state++)
  {
    f413_rp_anchor_t anchor;
    f413_rp_heading_t heading;
    f413_rp_speed_t speed;
    uint16_t roundtrip;

    REQUIRE(f413_rp_state_decode(state, &anchor, &heading, &speed));
    REQUIRE(f413_rp_state_index(anchor, heading, speed, &roundtrip));
    CHECK(roundtrip == state);
    if (state >= F413_RP_BASE_STATE_COUNT)
    {
      CHECK(speed == F413_RP_SPEED_LOW);
      CHECK(!f413_rp_state_index(anchor, heading, F413_RP_SPEED_NOMINAL,
                                  &roundtrip));
      CHECK(!f413_rp_state_index(anchor, heading, F413_RP_SPEED_CRAWL,
                                  &roundtrip));
    }
  }

  {
    const uint16_t previous = (uint16_t)(F413_RP_STATE_COUNT - 1U);
    const uint32_t packed = f413_rp_pack_parent(
        previous, 63U, F413_RP_KIND_SMALL_90, F413_RP_SIDE_LEFT,
        F413_RP_SPEED_CRAWL);
    CHECK((packed & F413_RP_PARENT_VALID) != 0U);
    CHECK(f413_rp_parent_previous(packed) == previous);
    CHECK(f413_rp_parent_connector(packed) == 63U);
    CHECK(f413_rp_parent_kind(packed) == F413_RP_KIND_SMALL_90);
    CHECK(f413_rp_parent_side(packed) == F413_RP_SIDE_LEFT);
    CHECK(f413_rp_parent_speed(packed) == F413_RP_SPEED_CRAWL);
  }
  return true;
}

static bool f413_test_runup_small_fallback(const char* search_dump_path)
{
  f413_rp_plan_status_t status;
  uint32_t expanded;
  uint32_t relaxed;
  NfLegacyPathResult validation;

  REQUIRE(f413_test_load_search_dump(search_dump_path));

  /* The captured low-acceleration case has no executable #1--#5 first edge. */
  REQUIRE(f413_test_build_runup_case(6U, false, &status, &expanded,
                                      &relaxed));
  CHECK(status == F413_RP_PLAN_NO_PATH);
  CHECK(expanded == 1U);
  CHECK(relaxed == 0U);

  f413_test_reset_trace_capture();
  REQUIRE(f413_test_build_runup_case(6U, true, &status, &expanded,
                                      &relaxed));
  REQUIRE(status == F413_RP_PLAN_OK);
  CHECK(g_runup_used_recovery_pass);
  CHECK(g_runup_first_kind_valid);
  CHECK(g_runup_first_kind == F413_RP_KIND_SMALL_90);
  CHECK(g_runup_chained_small_after_straight);
  CHECK(g_runup_wall_state_rejoined_base);
  CHECK(f413_test_path_has_diagonal_turn());
  CHECK(f413_test_path_has_diagonal_straight());
  validation = nf_legacy_path_validate(path, ROUTE_MAX_LEN);
  CHECK(validation.status == NF_LEGACY_PATH_OK);
  CHECK(f413_test_no_adjacent_orthogonal_straights());

  /* With enough acceleration the same first turn must remain nominal. */
  for (uint8_t case_index = 7U; case_index <= 8U; case_index++)
  {
    REQUIRE(f413_test_build_runup_case(case_index, true, &status, &expanded,
                                        &relaxed));
    REQUIRE(status == F413_RP_PLAN_OK);
    CHECK(!g_runup_used_recovery_pass);
    CHECK(g_runup_first_kind_valid);
    CHECK(g_runup_first_kind != F413_RP_KIND_SMALL_90);
    CHECK(f413_test_path_has_diagonal_turn());
    CHECK(f413_test_path_has_diagonal_straight());
    validation = nf_legacy_path_validate(path, ROUTE_MAX_LEN);
    CHECK(validation.status == NF_LEGACY_PATH_OK);
    CHECK(f413_test_no_adjacent_orthogonal_straights());
  }
  return true;
}

static f413_path_run_preflight_result_t
f413_test_runner_preflight_mode2_case(uint8_t case_index)
{
  const ShortestRunCaseParams_t* run_case =
      &shortestRunCaseParamsMode2[case_index - 1U];
  const float first_speed = f413_path_run_cap_positive(
      sqrtf(fmaxf(0.0f,
                  2.0f * run_case->acceleration_straight *
                      (float)DIST_FIRST_SEC)),
      NIGHTFALL_F413_PATH_VELOCITY_CAP);

  return f413_path_run_preflight(
      path, ROUTE_MAX_LEN, &shortestRunModeParams2, run_case, first_speed,
      true, false);
}

static bool f413_test_saved_maze_runner_preflight(
    const char* search_dump_path)
{
  REQUIRE(f413_test_load_search_dump(search_dump_path));
  g_lease_available = true;
  g_nvm_load_available = true;

  for (uint8_t case_index = 6U; case_index <= 8U; case_index++)
  {
    f413_path_run_preflight_result_t preflight;
    NfLegacyPathResult validation;

    g_release_count = 0U;
    f413_test_reset_trace_capture();
    REQUIRE(f413_route_build_mode2_path(case_index));
    CHECK(g_release_count == 1U);
    validation = nf_legacy_path_validate(path, ROUTE_MAX_LEN);
    REQUIRE(validation.status == NF_LEGACY_PATH_OK);

    preflight = f413_test_runner_preflight_mode2_case(case_index);
    g_checks++;
    if (preflight.status != F413_PATH_RUN_PREFLIGHT_OK)
    {
      g_failures++;
      fprintf(stderr,
              "FAIL saved-maze runner preflight case%u: status=%u "
              "legacy=%u index=%zu code=%u\n",
              (unsigned int)case_index,
              (unsigned int)preflight.status,
              (unsigned int)preflight.legacy_status,
              preflight.index,
              (unsigned int)preflight.code);
      return false;
    }
  }
  return true;
}

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    fprintf(stderr, "usage: %s 16MM2014CX.maze runup.search_dump\n", argv[0]);
    return 2;
  }
  (void)f413_test_kerilab_2014_parity(argv[1]);
  (void)f413_test_public_cleanup_and_metadata(argv[1]);
  (void)f413_test_terminal_statuses();
  (void)f413_test_heap_order_and_relax();
  (void)f413_test_public_run_path_build(argv[1]);
  (void)f413_test_execution_diagonal_path_exists();
  (void)f413_test_state_and_parent_roundtrip();
  (void)f413_test_runup_small_fallback(argv[2]);
  (void)f413_test_saved_maze_runner_preflight(argv[2]);
  if (g_failures != 0U)
  {
    fprintf(stderr, "f413 route preview host tests: %u/%u failed\n",
            g_failures, g_checks);
    return 1;
  }
  printf("f413 route preview host tests: %u checks passed "
         "scratch_peak=%zu/%zu bytes\n",
         g_checks, g_verified_scratch_peak,
         (size_t)F413_TRACE_LOG_IDLE_SCRATCH_BYTES);
  return 0;
}
