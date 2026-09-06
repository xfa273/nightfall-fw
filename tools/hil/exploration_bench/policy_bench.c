/* Decision-policy benchmark only: no route solver, HAL, Flash, FRAM or motion.
 * The synthetic oracle is exact for the stated one-dimensional toy model:
 * start (0,0), goal-entry (0,1), fixed north stopping tail through (0,3).
 * Three open north edges are its complete sufficient dependency set. This
 * measures policy navigation/validation and failure handling, not solver cost.
 */
#include "bench.h"
#include "exploration_policy.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define REPETITIONS 16U

static NfExplorationWorkspace workspace;
static uint8_t known[NF_EXPLORATION_MAX_CELLS];
static uint8_t walls[NF_EXPLORATION_MAX_CELLS];
static uint8_t visited[NF_EXPLORATION_MAX_CELLS];
static uint8_t goals[NF_EXPLORATION_MAX_CELLS];

typedef struct {
    NfExplorationOracleStatus status;
    bool complete;
    bool stale;
} ToyOracle;

static void set_edge(uint8_t n, uint8_t x, uint8_t y, uint8_t d, bool wall)
{
    static const int8_t dx[4] = {0, 1, 0, -1};
    static const int8_t dy[4] = {1, 0, -1, 0};
    uint16_t cell = (uint16_t)((uint16_t)y * n + x);
    int nx = x + dx[d], ny = y + dy[d];
    uint8_t bit = (uint8_t)(1U << d);
    known[cell] |= bit;
    if (wall) walls[cell] |= bit;
    else walls[cell] &= (uint8_t)~bit;
    if (nx >= 0 && ny >= 0 && nx < n && ny < n) {
        cell = (uint16_t)((uint16_t)ny * n + (uint16_t)nx);
        bit = (uint8_t)(1U << ((d + 2U) & 3U));
        known[cell] |= bit;
        if (wall) walls[cell] |= bit;
        else walls[cell] &= (uint8_t)~bit;
    }
}

static NfExplorationSnapshot make_fixture(uint8_t n)
{
    uint8_t x;
    NfExplorationSnapshot snapshot = {
        known, walls, visited, goals, 2U, 1U, n, n, 0U, 1U, 0U, 0U, 0U
    };
    memset(known, 0, sizeof(known));
    memset(walls, 0, sizeof(walls));
    memset(visited, 0, sizeof(visited));
    memset(goals, 0, sizeof(goals));
    for (x = 0U; x < n; ++x) {
        set_edge(n, x, 0U, 2U, true);
        set_edge(n, x, (uint8_t)(n - 1U), 0U, true);
        set_edge(n, 0U, x, 3U, true);
        set_edge(n, (uint8_t)(n - 1U), x, 1U, true);
    }
    set_edge(n, 0U, 0U, 0U, false);
    set_edge(n, 0U, 0U, 1U, true);
    set_edge(n, 0U, 1U, 0U, false);
    set_edge(n, 0U, 1U, 1U, false);
    visited[0] = 1U;
    visited[n] = 1U;
    goals[n] = 1U;
    return snapshot;
}

static void toy_oracle(void *context, const NfExplorationSnapshot *s,
                       bool restart, uint32_t budget, uint8_t *required,
                       NfExplorationOracleResult *result)
{
    ToyOracle *toy = context;
    (void)restart;
    (void)budget;
    result->status = toy->status;
    result->epoch = toy->stale ? s->epoch + 1U : s->epoch;
    result->generation = s->generation;
    result->goal_entry_us = 300000U;
    result->stop_us = 900000U;
    result->dependencies_complete = toy->complete;
    if (toy->status == NF_EXPLORATION_ORACLE_EXACT) {
        memset(required, 0, (size_t)s->width * s->height);
        required[0] = 1U;
        required[s->width] = 1U;
        required[2U * s->width] = 1U;
    }
}

/* kind: 0 GOAL, 1 probe, 2 certificate, 3 navigation budget, 4 pending,
 *       5 stale oracle, 6 incomplete dependencies, 7 bad map correction,
 *       8 cached exact result promoted after additive observation.
 */
static void run_case(uint32_t row_index, uint8_t size, uint8_t kind)
{
    nf_bench_row row;
    uint64_t total = 0U;
    uint32_t repeat;
    memset(&row, 0, sizeof(row));
    row.case_id = (uint32_t)size * 100U + kind;
    row.workspace_bytes = (uint32_t)nf_exploration_workspace_bytes();
    row.slices = REPETITIONS;
    row.required_edges = 3U;
    for (repeat = 0U; repeat < REPETITIONS; ++repeat) {
        NfExplorationSnapshot snapshot = make_fixture(size);
        NfExplorationConfig config = nf_exploration_default_config();
        NfExplorationDecision decision;
        ToyOracle toy = {NF_EXPLORATION_ORACLE_EXACT, true, false};
        NfExplorationStatus expected = NF_EXPLORATION_MOVE;
        NfExplorationStatus status;
        uint32_t begin, elapsed;
        if (kind == 0U) snapshot.y = 0U;
        if (kind == 2U) {
            set_edge(size, 0U, 2U, 0U, false);
            expected = NF_EXPLORATION_COMPLETE;
        }
        if (kind == 3U) {
            /* Keep below even the pruned navigation workload. */
            config.navigation_edge_budget = 4U;
            expected = NF_EXPLORATION_FALLBACK;
        }
        if (kind == 4U) {
            toy.status = NF_EXPLORATION_ORACLE_PENDING;
            expected = NF_EXPLORATION_FALLBACK;
        }
        if (kind == 5U) {
            toy.stale = true;
            expected = NF_EXPLORATION_FALLBACK;
        }
        if (kind == 6U) {
            toy.complete = false;
            expected = NF_EXPLORATION_FALLBACK;
        }
        nf_exploration_reset(&workspace, &config);
        if (kind == 7U || kind == 8U) {
            (void)nf_exploration_decide(&workspace, &snapshot, toy_oracle, &toy, &decision);
            if (kind == 7U) {
                set_edge(size, 0U, 1U, 0U, true);
                expected = NF_EXPLORATION_INVALID;
            } else {
                set_edge(size, 0U, 2U, 0U, false);
                expected = NF_EXPLORATION_COMPLETE;
            }
            ++snapshot.generation;
        }
        begin = nf_bench_cycles();
        status = nf_exploration_decide(&workspace, &snapshot, toy_oracle, &toy, &decision);
        elapsed = nf_bench_cycles() - begin;
        total += elapsed;
        if (elapsed > row.max_slice_cycles) row.max_slice_cycles = elapsed;
        row.status = status == expected ? (uint32_t)status : 0x80000000U | (uint32_t)status;
        if (decision.certified != (status == NF_EXPLORATION_COMPLETE)) row.status |= 0x40000000U;
        if (status == NF_EXPLORATION_MOVE && decision.direction != 0U) row.status |= 0x20000000U;
        if (kind == 3U && decision.navigation_edges != 4U) row.status |= 0x10000000U;
        row.expanded = decision.navigation_edges;
        row.goal_entry_us = (uint32_t)decision.lower_us;
        row.stop_us = decision.certified ? 900000U : 0U;
        row.checksum ^= (uint32_t)decision.reason | ((uint32_t)decision.direction << 8U) |
                        ((uint32_t)decision.phase << 16U) | ((repeat + 1U) << 24U);
    }
    row.cycles_low = (uint32_t)total;
    row.cycles_high = (uint32_t)(total >> 32U);
    nf_bench_output.rows[row_index] = row;
    nf_bench_output.row_count = row_index + 1U;
}

/* kinds 9/10: probe, one/eight policy work units per slice; 11: certificate;
 * 12: pending oracle with already observed cardinal route;
 * 13/14: pending oracle with useful unknown cardinal probes, one/eight units.
 * cycles_* sum 16 entire decisions' measured slices;
 * slices counts begin+step+result calls, max_slice_cycles is the worst call.
 * A step unit includes at most one bounded heap operation or one cell/edge.
 */
static void run_sliced_case(uint32_t row_index, uint8_t size, uint8_t kind)
{
    nf_bench_row row;
    uint64_t total = 0U;
    uint32_t repeat;
    memset(&row, 0, sizeof(row));
    row.case_id = (uint32_t)size * 100U + kind;
    row.workspace_bytes = (uint32_t)nf_exploration_workspace_bytes();
    row.required_edges = 3U;
    for (repeat = 0U; repeat < REPETITIONS; ++repeat) {
        NfExplorationSnapshot s = make_fixture(size);
        NfExplorationConfig config = nf_exploration_default_config();
        NfExplorationDecision d;
        ToyOracle toy = {NF_EXPLORATION_ORACLE_EXACT, true, false};
        NfExplorationStatus status, expected = NF_EXPLORATION_MOVE;
        uint32_t begin, elapsed, work_budget = kind == 10U || kind == 14U ? 8U : 1U;
        if (kind == 11U) {
            set_edge(size, 0U, 2U, 0U, false);
            expected = NF_EXPLORATION_COMPLETE;
        }
        if (kind == 12U) {
            toy.status = NF_EXPLORATION_ORACLE_PENDING;
            expected = NF_EXPLORATION_FALLBACK;
        }
        if (kind == 13U || kind == 14U) {
            config.pending_surrogate = true;
            goals[size] = 0U;
            goals[(uint16_t)size * size - 1U] = 1U;
            s.x = (uint8_t)(size - 1U); s.y = (uint8_t)(size - 1U); s.heading = 3U;
            visited[(uint16_t)size * size - 1U] = 1U;
            set_edge(size, s.x, s.y, 2U, false);
            set_edge(size, s.x, s.y, 3U, false);
            toy.status = NF_EXPLORATION_ORACLE_PENDING;
        }
        nf_exploration_reset(&workspace, &config);
        begin = nf_bench_cycles();
        status = nf_exploration_begin(&workspace, &s, toy_oracle, &toy, false);
        elapsed = nf_bench_cycles() - begin;
        total += elapsed;
        ++row.slices;
        if (elapsed > row.max_slice_cycles) row.max_slice_cycles = elapsed;
        while (status == NF_EXPLORATION_PENDING) {
            begin = nf_bench_cycles();
            status = nf_exploration_step(&workspace, work_budget);
            elapsed = nf_bench_cycles() - begin;
            total += elapsed;
            ++row.slices;
            if (elapsed > row.max_slice_cycles) row.max_slice_cycles = elapsed;
        }
        begin = nf_bench_cycles();
        status = nf_exploration_result(&workspace, &d);
        elapsed = nf_bench_cycles() - begin;
        total += elapsed;
        ++row.slices;
        if (elapsed > row.max_slice_cycles) row.max_slice_cycles = elapsed;
        row.status = status == expected ? (uint32_t)status : 0x80000000U | (uint32_t)status;
        if (d.certified != (status == NF_EXPLORATION_COMPLETE)) row.status |= 0x40000000U;
        if (status == NF_EXPLORATION_MOVE && d.direction != (kind >= 13U ? 3U : 0U)) row.status |= 0x20000000U;
        if (kind >= 13U && (d.reason != NF_EXPLORATION_REASON_SURROGATE || d.lower_us != 0U))
            row.status |= 0x10000000U;
        row.expanded = d.navigation_edges;
        row.goal_entry_us = (uint32_t)d.lower_us;
        row.stop_us = d.certified ? 900000U : 0U;
        row.checksum ^= (uint32_t)d.reason | ((repeat + 1U) << 24U);
    }
    row.cycles_low = (uint32_t)total;
    row.cycles_high = (uint32_t)(total >> 32U);
    nf_bench_output.rows[row_index] = row;
    nf_bench_output.row_count = row_index + 1U;
}

void nf_bench_run(void)
{
    uint32_t row = 0U;
    uint8_t kind;
    nf_bench_output.reserved = sizeof(known) + sizeof(walls) + sizeof(visited) + sizeof(goals);
    for (kind = 0U; kind <= 8U; ++kind) run_case(row++, 16U, kind);
    for (kind = 9U; kind <= 14U; ++kind) run_sliced_case(row++, 16U, kind);
#if NF_EXPLORATION_MAX_SIZE >= 32U
    run_case(row++, 32U, 1U);
    run_case(row++, 32U, 2U);
    run_case(row++, 32U, 3U);
    for (kind = 9U; kind <= 14U; ++kind) run_sliced_case(row++, 32U, kind);
#endif
}
