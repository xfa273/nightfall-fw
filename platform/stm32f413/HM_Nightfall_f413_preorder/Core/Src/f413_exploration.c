#include "f413_exploration.h"
#include "exploration_policy.h"
#include "mcu_slalom_time_planner.h"
#include "f413_search_step.h"
#include "f413_trace_log.h"
#include "params.h"
#include "search.h"
#include "stm32f4xx_hal.h"
#include "trace.h"
#include <string.h>

#define CELLS (MAZE_SIZE * MAZE_SIZE)
/* Conservative initial scheduling; measured per-work-unit bounds are required
 * before a floor run. Interrupts remain enabled throughout planner work. */
#define POLL_CYCLES (SystemCoreClock / 2000U)
#define POLL_UNITS 256U

static NfExplorationWorkspace policy;
static NfExplorationSnapshot snapshot;
static NfExplorationSnapshot predicted;
static NfMcuSlalom *planner;
static void *storage;
static size_t storage_bytes;
static uint8_t known[CELLS], walls[CELLS], seen[CELLS], goals[CELLS];
static uint8_t job_known[CELLS], job_walls[CELLS], job_seen[CELLS];
static uint32_t planner_generation, planner_epoch;
static uint32_t solve_count, fallback_count, move_count, surrogate_move_count, max_poll_cycles;
static uint64_t compute_cycles;
static bool active, map_valid, job_started;
static NfMcuSlalomStatus planner_status;
static NfExplorationStatus policy_status;

static uint8_t reverse_nibble(uint8_t n)
{
  return (uint8_t)(((n & 1U) << 3) | ((n & 2U) << 1) |
                   ((n & 4U) >> 1) | ((n & 8U) >> 3));
}

static bool capture_map(void)
{
  static const int8_t dx[4] = {0, 1, 0, -1};
  static const int8_t dy[4] = {1, 0, -1, 0};
  bool changed = false, corrected = false;
  for (unsigned y = 0; y < MAZE_SIZE; ++y) {
    for (unsigned x = 0; x < MAZE_SIZE; ++x) {
      unsigned cell = y * MAZE_SIZE + x;
      uint8_t raw = (uint8_t)map[y][x];
      uint8_t k = reverse_nibble((uint8_t)~((raw >> 4) ^ raw) & 15U);
      uint8_t w = reverse_nibble(raw & 15U);
      for (unsigned d = 0; d < 4; ++d) {
        int nx = (int)x + dx[d], ny = (int)y + dy[d];
        uint8_t b = (uint8_t)(1U << d);
        if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE) {
          k |= b; w |= b;
        } else {
          uint8_t other = (uint8_t)map[ny][nx];
          uint8_t ob = (uint8_t)(8U >> ((d + 2U) & 3U));
          bool ok = ((other ^ (other >> 4)) & ob) == 0;
          bool ow = (other & ob) != 0;
          if (ok && (k & b) && ow != ((w & b) != 0)) return false;
          if (ok) { k |= b; if (ow) w |= b; else w &= (uint8_t)~b; }
        }
      }
      if ((known[cell] & (uint8_t)~k) || ((walls[cell] ^ w) & known[cell])) corrected = true;
      changed |= known[cell] != k || walls[cell] != w || seen[cell] != (visited[y][x] ? 1U : 0U);
      if (seen[cell] && !visited[y][x]) corrected = true;
      known[cell] = k; walls[cell] = w;
      seen[cell] = visited[y][x] ? 1U : 0U;
    }
  }
  if (corrected) ++snapshot.epoch;
  if (changed) {
    if (snapshot.generation == UINT32_MAX) { ++snapshot.epoch; snapshot.generation = 0U; }
    ++snapshot.generation;
  }
  snapshot.x = (uint8_t)mouse.x;
  snapshot.y = (uint8_t)mouse.y;
  snapshot.heading = (uint8_t)mouse.dir;
  return true;
}

void f413_exploration_poll(void)
{
  if (!active || !job_started) return;
  uint32_t begin = DWT->CYCCNT;
  for (unsigned i = 0; i < POLL_UNITS; ++i) {
    if (planner && planner_status == NF_MCU_SLALOM_PENDING)
      planner_status = nf_mcu_slalom_step(planner, 1U);
    if ((uint32_t)(DWT->CYCCNT - begin) >= POLL_CYCLES) break;
    if (policy_status == NF_EXPLORATION_PENDING)
      policy_status = nf_exploration_step(&policy, 1U);
    if ((uint32_t)(DWT->CYCCNT - begin) >= POLL_CYCLES) break;
    if (planner_status != NF_MCU_SLALOM_PENDING && policy_status != NF_EXPLORATION_PENDING) break;
  }
  uint32_t elapsed = DWT->CYCCNT - begin;
  compute_cycles += elapsed;
  if (elapsed > max_poll_cycles) max_poll_cycles = elapsed;
}

static void oracle(void *unused, const NfExplorationSnapshot *s, bool restart,
                   uint32_t budget, uint8_t *required, NfExplorationOracleResult *out)
{
  (void)unused; (void)budget;
  NfMcuSlalomResult result = {0};
  if (restart) {
    planner_status = nf_mcu_slalom_begin(storage, storage_bytes, s->width, s->height,
                        s->walls, s->goals, s->start_x, s->start_y, 0U, &planner);
    planner_generation = s->generation; planner_epoch = s->epoch;
    ++solve_count;
  }
  NfMcuSlalomStatus status = nf_mcu_slalom_result(planner, &result, required, CELLS);
  out->generation = planner_generation; out->epoch = planner_epoch;
  out->goal_entry_us = result.goal_entry_us; out->stop_us = result.stop_us;
  out->dependencies_complete = result.requirements_complete;
  switch (status) {
    case NF_MCU_SLALOM_PENDING: out->status = NF_EXPLORATION_ORACLE_PENDING; break;
    case NF_MCU_SLALOM_EXACT: out->status = NF_EXPLORATION_ORACLE_EXACT; break;
    case NF_MCU_SLALOM_NO_PATH:
    case NF_MCU_SLALOM_NO_FEASIBLE_TERMINAL: out->status = NF_EXPLORATION_ORACLE_NO_PATH; break;
    case NF_MCU_SLALOM_CAPACITY: out->status = NF_EXPLORATION_ORACLE_CAPACITY; break;
    default: out->status = NF_EXPLORATION_ORACLE_INVALID; break;
  }
}

void f413_exploration_begin(uint8_t op_case, uint8_t param_index)
{
  static const uint8_t goal_xy[9][2] = {
    {GOAL1_X, GOAL1_Y}, {GOAL2_X, GOAL2_Y}, {GOAL3_X, GOAL3_Y},
    {GOAL4_X, GOAL4_Y}, {GOAL5_X, GOAL5_Y}, {GOAL6_X, GOAL6_Y},
    {GOAL7_X, GOAL7_Y}, {GOAL8_X, GOAL8_Y}, {GOAL9_X, GOAL9_Y}
  };
  f413_exploration_end();
  /* Only the requested mode1/case1 GOAL->FULL search profile is replaced. */
  if (op_case != 1U || param_index != 0U) return;
#if defined(NIGHTFALL_F413_RUNTIME_CONFIG)
  /* The newer runtime planner changes the start/turn profiles AND enables
     recovery edges only after a primary no-path result. Its precomputed-route
     compatibility flag does not prove equivalence to this pinned oracle.
     A separate monotone lower bound / executable upper bound is required. */
  trace_printf("[EXPLORE] unsupported runtime route model; retaining Adachi\r\n");
  return;
#endif
  if (!f413_trace_log_borrow_exploration(&storage, &storage_bytes)) return;
  if (storage_bytes < nf_mcu_slalom_workspace_bytes_for(MAZE_SIZE, MAZE_SIZE)) {
    f413_trace_log_release_exploration(storage); storage = NULL; return;
  }
  memset(known, 0, sizeof(known)); memset(walls, 0, sizeof(walls));
  memset(seen, 0, sizeof(seen));
  memset(goals, 0, sizeof(goals));
  for (unsigned i = 0; i < 9; ++i) {
    unsigned x = goal_xy[i][0], y = goal_xy[i][1];
    if ((x || y) && x < MAZE_SIZE && y < MAZE_SIZE) goals[y * MAZE_SIZE + x] = 1U;
  }
  snapshot = (NfExplorationSnapshot){.known = known, .walls = walls, .visited = seen,
    .goals = goals, .width = MAZE_SIZE, .height = MAZE_SIZE, .epoch = 1U,
    .start_x = START_X, .start_y = START_Y};
  NfExplorationConfig config = nf_exploration_default_config();
  nf_exploration_reset(&policy, &config);
  planner = NULL; solve_count = 0; fallback_count = 0; move_count = 0; surrogate_move_count = 0;
  job_started = false; planner_status = NF_MCU_SLALOM_INVALID;
  policy_status = NF_EXPLORATION_INVALID;
  compute_cycles = 0; max_poll_cycles = 0; map_valid = true; active = true;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  trace_printf("[EXPLORE] enabled profile=%s workspace=%lu policy=%lu\r\n",
    NF_MCU_SLALOM_PROFILE_ID, (unsigned long)storage_bytes,
    (unsigned long)sizeof(policy));
}

bool f413_exploration_observe_goal(void)
{
  if (!active) return true;
  if (!map_valid || !capture_map()) { map_valid = false; return false; }
  (void)nf_exploration_note_goal_reached(&policy, &snapshot);
  return true;
}

uint8_t f413_exploration_decide(uint8_t target, bool accelerated, uint8_t *next_relative,
                              bool *known_straight, bool *next_is_turn90)
{
  if (!active || !map_valid || target != F413_SEARCH_STEP_TARGET_FULL) return 0U;
  if (!capture_map()) { map_valid = false; return 3U; }
  (void)nf_exploration_note_goal_reached(&policy, &snapshot);
  NfExplorationDecision decision = {0};
  bool improved = false;
  if (job_started && policy_status != NF_EXPLORATION_PENDING) {
    nf_exploration_result(&policy, &decision);
    NfExplorationStatus status = nf_exploration_apply_result(&predicted, &snapshot, &decision);
    improved = status == NF_EXPLORATION_MOVE || status == NF_EXPLORATION_COMPLETE;
  }
  /* Mixing late relevant decisions with immediate Adachi can reverse the
     same corridor forever. After a miss, keep Adachi until real facts change. */
  NfExplorationStatus progress = nf_exploration_guard_progress(&policy, &snapshot,
      job_started && !improved, &decision);
  improved = improved && (progress == NF_EXPLORATION_MOVE || progress == NF_EXPLORATION_COMPLETE);
  if (!improved) {
    ++fallback_count;
    decision.status = NF_EXPLORATION_MOVE;
    decision.direction = (uint8_t)((snapshot.heading + *next_relative) & 3U);
    decision.known_straight = *known_straight;
    decision.next_is_turn90 = *next_is_turn90;
    decision.certified = false;
  }
  /* Honor a straight/braking obligation created by either planner. */
  if (nf_exploration_guard_acceleration(&snapshot, accelerated, &decision) == NF_EXPLORATION_INVALID)
    return 3U;
  if (decision.status == NF_EXPLORATION_COMPLETE && decision.certified) {
    trace_printf("[EXPLORE] certified lower_us=%lu epoch=%lu generation=%lu\r\n",
      (unsigned long)decision.lower_us, (unsigned long)snapshot.epoch,
      (unsigned long)snapshot.generation);
    return 2U;
  }
  if (decision.status != NF_EXPLORATION_MOVE) return 0U;
  *next_relative = (uint8_t)((decision.direction + 4U - snapshot.heading) & 3U);
  *known_straight = decision.known_straight;
  *next_is_turn90 = decision.next_is_turn90;
  if (improved) {
    ++move_count;
    if (decision.reason == NF_EXPLORATION_REASON_SURROGATE) ++surrogate_move_count;
  }
  return 1U;
}

void f413_exploration_prepare(uint8_t target, uint8_t next_relative)
{
  static const int8_t dx[4] = {0, 1, 0, -1};
  static const int8_t dy[4] = {1, 0, -1, 0};
  if (!active || !map_valid || target != F413_SEARCH_STEP_TARGET_FULL) return;
  uint8_t heading = (uint8_t)((snapshot.heading + next_relative) & 3U);
  int x = (int)snapshot.x + dx[heading], y = (int)snapshot.y + dy[heading];
  if (x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE) return;
  /* Only these buffers are passed to the asynchronous policy. Live map
     observations never modify a job's immutable snapshot. */
  memcpy(job_known, known, sizeof(known)); memcpy(job_walls, walls, sizeof(walls));
  memcpy(job_seen, seen, sizeof(seen));
  predicted = snapshot;
  predicted.known = job_known; predicted.walls = job_walls; predicted.visited = job_seen;
  predicted.x = (uint8_t)x; predicted.y = (uint8_t)y; predicted.heading = heading;
  policy_status = nf_exploration_begin(&policy, &predicted, oracle, NULL, true);
  job_started = true;
}

void f413_exploration_end(void)
{
  if (active) trace_printf("[EXPLORE] solves=%lu moves=%lu surrogate=%lu fallback=%lu cpu_ms=%lu max_poll_us=%lu\r\n",
    (unsigned long)solve_count, (unsigned long)move_count, (unsigned long)surrogate_move_count,
    (unsigned long)fallback_count,
    (unsigned long)(compute_cycles / (SystemCoreClock / 1000U)),
    (unsigned long)(max_poll_cycles / (SystemCoreClock / 1000000U)));
  active = false; planner = NULL; job_started = false;
  if (storage) f413_trace_log_release_exploration(storage);
  storage = NULL; storage_bytes = 0;
}
