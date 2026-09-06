/* SRAM-only test of the actual F413 adapter. No drive/control/HAL functions
 * are linked. DWT/CoreDebug are the only registers touched by the adapter.
 * The included production source exposes its file-private scheduling state
 * inside this test translation unit; no production test API is introduced.
 * Actual f413_preorder params are used: 16x16, goal (1,0), start (0,0).
 */
#include "bench.h"
#include "params.h"
#include "search.h"
#include "f413_trace_log.h"
#include "trace.h"
#include "mcu_slalom_time_planner.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

uint32_t SystemCoreClock = 100000000U;
uint16_t map[MAZE_SIZE][MAZE_SIZE];
bool visited[MAZE_SIZE][MAZE_SIZE];
volatile struct coordinate_and_direction mouse;

#ifndef NF_BENCH_WORKSPACE_BYTES
#define NF_BENCH_WORKSPACE_BYTES NF_MCU_SLALOM_WORKSPACE_BYTES
#endif
static uint8_t leased_pool[NF_BENCH_WORKSPACE_BYTES]
    __attribute__((aligned(NF_MCU_SLALOM_WORKSPACE_ALIGNMENT)));
static size_t lease_limit = sizeof(leased_pool);
static bool lease_in_use;
static uint32_t lease_borrows, lease_releases, trace_calls;

int trace_printf(const char *format, ...)
{
    (void)format;
    ++trace_calls;
    return 0;
}

bool f413_trace_log_borrow_exploration(void **out, size_t *bytes)
{
    if (out == NULL || bytes == NULL || lease_in_use) return false;
    *out = leased_pool;
    *bytes = lease_limit;
    lease_in_use = true;
    ++lease_borrows;
    return true;
}

void f413_trace_log_release_exploration(void *pointer)
{
    if (pointer == leased_pool && lease_in_use) {
        lease_in_use = false;
        ++lease_releases;
    }
}

#include "../../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_exploration.c"

static void fixture(void)
{
    for (unsigned y = 0U; y < MAZE_SIZE; ++y) {
        for (unsigned x = 0U; x < MAZE_SIZE; ++x) {
            uint16_t wall = (uint16_t)((y + 1U == MAZE_SIZE ? 8U : 0U) |
                (x + 1U == MAZE_SIZE ? 4U : 0U) | (y == 0U ? 2U : 0U) |
                (x == 0U ? 1U : 0U));
            map[y][x] = (uint16_t)(wall | (wall << 4U));
            visited[y][x] = true;
        }
    }
    mouse.x = GOAL1_X;
    mouse.y = GOAL1_Y;
    mouse.dir = 1U;
}

static void row_start(nf_bench_row *row, uint32_t id)
{
    memset(row, 0, sizeof(*row));
    row->case_id = id;
    row->workspace_bytes = (uint32_t)sizeof(policy);
}

static void row_add(nf_bench_row *row, uint32_t elapsed)
{
    uint64_t total = ((uint64_t)row->cycles_high << 32U) | row->cycles_low;
    total += elapsed;
    row->cycles_low = (uint32_t)total;
    row->cycles_high = (uint32_t)(total >> 32U);
    ++row->slices;
    if (elapsed > row->max_slice_cycles) row->max_slice_cycles = elapsed;
}

static void row_finish(nf_bench_row *row, bool pass, uint32_t observed)
{
    row->status = observed | (pass ? 0U : 0x80000000U);
    row->checksum = lease_borrows | (lease_releases << 8U) | (trace_calls << 16U);
    uint32_t index = nf_bench_output.row_count;
    nf_bench_output.rows[index] = *row;
    nf_bench_output.row_count = index + 1U;
}

static bool poll_quiet(nf_bench_row *row)
{
    for (unsigned i = 0U; i < 100000U; ++i) {
        if (policy_status != NF_EXPLORATION_PENDING && planner_status != NF_MCU_SLALOM_PENDING)
            return true;
        uint32_t before = nf_bench_cycles();
        f413_exploration_poll();
        row_add(row, nf_bench_cycles() - before);
    }
    return false;
}

void nf_bench_run(void)
{
    nf_bench_row row;
    uint32_t before;
    uint8_t choice = 0U, relative = 0U;
    bool fast = false, turn90 = false, pass;
    nf_bench_output.reserved = sizeof(leased_pool);
    fixture();

    row_start(&row, 4000U); /* One-time startup/reset and workspace lease. */
    before = nf_bench_cycles();
    f413_exploration_begin(1U, 0U);
    row_add(&row, nf_bench_cycles() - before);
    row_finish(&row, active && lease_in_use && lease_borrows == 1U, active ? 1U : 0U);

    row_start(&row, 4001U); /* Full capture + actual-goal latch + Adachi fallback. */
    pass = true;
    for (unsigned i = 0U; i < 16U; ++i) {
        relative = 0U; fast = true; turn90 = false;
        before = nf_bench_cycles();
        choice = f413_exploration_decide(F413_SEARCH_STEP_TARGET_FULL, false,
                                        &relative, &fast, &turn90);
        row_add(&row, nf_bench_cycles() - before);
        pass = pass && choice == 1U && relative == 0U && fast && policy.reached_goal;
    }
    row_finish(&row, pass && move_count == 0U && surrogate_move_count == 0U, choice);

    row_start(&row, 4002U); /* Snapshot copy + predicted-pose begin, no solver drain. */
    pass = true;
    for (unsigned i = 0U; i < 16U; ++i) {
        before = nf_bench_cycles();
        f413_exploration_prepare(F413_SEARCH_STEP_TARGET_FULL, 0U);
        row_add(&row, nf_bench_cycles() - before);
        pass = pass && job_started && policy_status == NF_EXPLORATION_PENDING &&
            predicted.x == GOAL1_X + 1U && predicted.y == GOAL1_Y &&
            predicted.heading == 1U && solve_count == 0U;
    }
    row_finish(&row, pass, (uint32_t)policy_status);

    row_start(&row, 4003U); /* Guarded-wait polling, including real oracle begin/result. */
    pass = poll_quiet(&row);
    /* The first job falls back while its oracle is pending. A following job
     * consumes the exact result after the foreground solver has finished. */
    if (policy_status != NF_EXPLORATION_COMPLETE) {
        f413_exploration_prepare(F413_SEARCH_STEP_TARGET_FULL, 0U);
        pass = poll_quiet(&row) && pass;
    }
    NfMcuSlalomResult exact = {0};
    nf_mcu_slalom_result(planner, &exact, NULL, 0U);
    row.expanded = exact.work_units;
    row.goal_entry_us = exact.goal_entry_us;
    row.stop_us = exact.stop_us;
    row.required_edges = solve_count;
    row_finish(&row, pass && policy_status == NF_EXPLORATION_COMPLETE &&
               exact.requirements_complete && solve_count == 1U, (uint32_t)policy_status);

    mouse.x = GOAL1_X + 1U; mouse.y = GOAL1_Y; mouse.dir = 1U;
    row_start(&row, 4004U); /* Arrival capture + predictive certificate application. */
    pass = true;
    for (unsigned i = 0U; i < 16U; ++i) {
        relative = 1U; fast = false; turn90 = true;
        before = nf_bench_cycles();
        choice = f413_exploration_decide(F413_SEARCH_STEP_TARGET_FULL, false,
                                        &relative, &fast, &turn90);
        row_add(&row, nf_bench_cycles() - before);
        pass = pass && choice == 2U;
    }
    row_finish(&row, pass, choice);

    row_start(&row, 4005U); /* Certificate cannot violate the acceleration promise. */
    relative = 1U; fast = true; turn90 = true;
    before = nf_bench_cycles();
    choice = f413_exploration_decide(F413_SEARCH_STEP_TARGET_FULL, true,
                                    &relative, &fast, &turn90);
    row_add(&row, nf_bench_cycles() - before);
    row_finish(&row, choice == 1U && relative == 0U && !fast && !turn90 &&
               move_count == 1U && surrogate_move_count == 0U, choice);

    row_start(&row, 4006U); /* Corrected front wall invalidates epoch and braking route. */
    uint32_t old_epoch = snapshot.epoch;
    map[mouse.y][mouse.x] |= 0x44U;
    map[mouse.y][mouse.x + 1U] |= 0x11U;
    relative = 1U; fast = false; turn90 = false;
    before = nf_bench_cycles();
    choice = f413_exploration_decide(F413_SEARCH_STEP_TARGET_FULL, true,
                                    &relative, &fast, &turn90);
    row_add(&row, nf_bench_cycles() - before);
    row_finish(&row, choice == 3U && snapshot.epoch > old_epoch, choice);

    row_start(&row, 4007U); /* Cleanup releases the borrowed memory exactly once. */
    before = nf_bench_cycles();
    f413_exploration_end();
    row_add(&row, nf_bench_cycles() - before);
    row_finish(&row, !active && !job_started && planner == NULL && !lease_in_use &&
               lease_borrows == lease_releases, lease_releases);

    row_start(&row, 4008U); /* Capacity failure leaves the integration disabled. */
    lease_limit = nf_mcu_slalom_workspace_bytes_for(MAZE_SIZE, MAZE_SIZE) - 1U;
    before = nf_bench_cycles();
    f413_exploration_begin(1U, 0U);
    row_add(&row, nf_bench_cycles() - before);
    row_finish(&row, !active && !lease_in_use && lease_borrows == lease_releases, active ? 1U : 0U);

    lease_limit = sizeof(leased_pool);
    fixture();
    f413_exploration_begin(1U, 0U);
    row_start(&row, 4009U); /* Inconsistent shared observations cause explicit abort. */
    map[GOAL1_Y][GOAL1_X] |= 0x44U; /* Opposite side deliberately remains open. */
    before = nf_bench_cycles();
    choice = f413_exploration_decide(F413_SEARCH_STEP_TARGET_FULL, false,
                                    &relative, &fast, &turn90);
    row_add(&row, nf_bench_cycles() - before);
    row_finish(&row, choice == 3U && !map_valid, choice);
    f413_exploration_end();

    fixture();
    f413_exploration_begin(1U, 0U);
    row_start(&row, 4010U); /* Actual goal latch survives a phase-restart move. */
    before = nf_bench_cycles();
    pass = f413_exploration_observe_goal();
    row_add(&row, nf_bench_cycles() - before);
    pass = pass && policy.reached_goal;
    mouse.x = GOAL1_X + 1U; /* Emulate movement before the first FULL boundary. */
    relative = 0U; fast = false; turn90 = false;
    before = nf_bench_cycles();
    choice = f413_exploration_decide(F413_SEARCH_STEP_TARGET_FULL, false,
                                    &relative, &fast, &turn90);
    row_add(&row, nf_bench_cycles() - before);
    row_finish(&row, pass && choice == 1U && policy.reached_goal && !job_started, choice);

    f413_exploration_prepare(F413_SEARCH_STEP_TARGET_FULL, 0U);
    row_start(&row, 4011U); /* Misses on an unchanged map hold the legacy policy. */
    uint32_t unchanged_generation = snapshot.generation;
    pass = true;
    for (unsigned i = 0U; i < 4U; ++i) {
        mouse.x = (uint8_t)(GOAL1_X + 1U + (i & 1U));
        relative = 2U; fast = false; turn90 = false;
        before = nf_bench_cycles();
        choice = f413_exploration_decide(F413_SEARCH_STEP_TARGET_FULL, false,
                                        &relative, &fast, &turn90);
        row_add(&row, nf_bench_cycles() - before);
        pass = pass && choice == 1U && relative == 2U && policy.fallback_held &&
            snapshot.generation == unchanged_generation;
    }
    row_finish(&row, pass, choice);

    row_start(&row, 4012U); /* Exact completion still passes while Adachi is held. */
    pass = poll_quiet(&row);
    if (policy_status != NF_EXPLORATION_COMPLETE) {
        f413_exploration_prepare(F413_SEARCH_STEP_TARGET_FULL, 0U);
        pass = poll_quiet(&row) && pass;
    }
    mouse.x = predicted.x; mouse.y = predicted.y; mouse.dir = predicted.heading;
    relative = 1U; fast = false; turn90 = true;
    before = nf_bench_cycles();
    choice = f413_exploration_decide(F413_SEARCH_STEP_TARGET_FULL, false,
                                    &relative, &fast, &turn90);
    row_add(&row, nf_bench_cycles() - before);
    row_finish(&row, pass && choice == 2U && policy.fallback_held &&
               snapshot.generation == unchanged_generation, choice);
    f413_exploration_end();
}
