/* Motor-off SRAM benchmark. No HAL, NVM, UART, timers or motion commands.
 * Inputs are partial-map fixtures, independent of the current machine's map.
 * Compile with NF_BENCH_CASES_HEADER=".../mcu-*-cases.h". Define
 * NF_BENCH_CASE_INDEX=n to retain only that fixture in the generated header.
 * Row case_id: low16=(original fixture index<<1)|projection (0 optimistic,
 * 1 conservative). Bit16 marks a begin() timing row. Solve rows use step(1),
 * sum wrap-safe per-call DWT deltas into64bits, and report maximum single call.
 * Bit31 in status means the benchmark's work-unit cap stopped a pending solve.
 * Init/solve exclude input conversion; final result/dependency copying is also
 * outside the timing interval. Initialization and solve are reported separately.
 */
#include "bench.h"
#include "mcu_slalom_time_planner.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#ifndef NF_BENCH_CASES_HEADER
#error "Set NF_BENCH_CASES_HEADER to an exported MCU fixture header"
#endif
#include NF_BENCH_CASES_HEADER
#ifndef NF_BENCH_MAX_SLICES
#define NF_BENCH_MAX_SLICES 50000000U
#endif
#ifndef NF_BENCH_WORKSPACE_BYTES
#define NF_BENCH_WORKSPACE_BYTES NF_MCU_SLALOM_WORKSPACE_BYTES
#endif

static uint8_t workspace[NF_BENCH_WORKSPACE_BYTES] __attribute__((aligned(NF_MCU_SLALOM_WORKSPACE_ALIGNMENT)));
static uint8_t projected[NF_MCU_SLALOM_CELLS];
static uint8_t required[NF_MCU_SLALOM_CELLS];

static uint32_t checksum(const uint8_t *bytes, size_t length)
{
    uint32_t hash = 2166136261U;
    while (length--) { hash ^= *bytes++; hash *= 16777619U; }
    return hash;
}
static uint32_t physical_required_count(const uint8_t *masks, uint8_t width, uint8_t height)
{
    uint32_t count = 0U;
    for (uint8_t y = 0; y < height; ++y) for (uint8_t x = 0; x < width; ++x) {
        const uint8_t mask = masks[(size_t)y * width + x];
        if (y + 1U < height && (mask & 1U)) ++count;
        if (x + 1U < width && (mask & 2U)) ++count;
    }
    return count;
}
static void run_case(const nf_mcu_benchmark_case_t *fixture, uint32_t original_index, unsigned projection)
{
    nf_bench_row row;
    NfMcuSlalom *planner = NULL;
    NfMcuSlalomResult result;
    NfMcuSlalomStatus status;
    const size_t cells = (size_t)fixture->width * fixture->height;
    const uint32_t case_id = (original_index << 1U) | projection;
    uint32_t before, elapsed, init_cycles;
    uint64_t total = 0U;
    uint32_t row_index;
    if (cells > NF_MCU_SLALOM_CELLS || nf_bench_output.row_count + 2U > NF_BENCH_MAX_ROWS) return;
    for (size_t i = 0; i < cells; ++i)
        projected[i] = fixture->walls[i] | (projection ? (uint8_t)(15U ^ fixture->known[i]) : 0U);
    memset(required, 0, sizeof(required));
    memset(&row, 0, sizeof(row));
    memset(&result, 0, sizeof(result));
    row.case_id = case_id | 0x10000U;
    row.workspace_bytes = (uint32_t)nf_mcu_slalom_workspace_bytes_for(fixture->width, fixture->height);
    before = nf_bench_cycles();
    status = nf_mcu_slalom_begin(workspace, sizeof(workspace), fixture->width, fixture->height,
                                 projected, fixture->goals, 0U, 0U, 0U, &planner);
    init_cycles = nf_bench_cycles() - before;
    row.status = (uint32_t)status;
    row.cycles_low = init_cycles;
    row.max_slice_cycles = init_cycles;
    row.slices = 1U;
    row.checksum = checksum(projected, cells);
    row_index = nf_bench_output.row_count;
    nf_bench_output.rows[row_index] = row;
    nf_bench_output.row_count = row_index + 1U;

    memset(&row, 0, sizeof(row));
    row.case_id = case_id;
    row.workspace_bytes = (uint32_t)nf_mcu_slalom_workspace_bytes_for(fixture->width, fixture->height);
    row.status = (uint32_t)status;
    row_index = nf_bench_output.row_count;
    nf_bench_output.rows[row_index] = row;
    nf_bench_output.row_count = row_index + 1U;
    while (status == NF_MCU_SLALOM_PENDING && row.slices < NF_BENCH_MAX_SLICES) {
        before = nf_bench_cycles();
        status = nf_mcu_slalom_step(planner, 1U);
        elapsed = nf_bench_cycles() - before;
        total += elapsed;
        ++row.slices;
        if (elapsed > row.max_slice_cycles) row.max_slice_cycles = elapsed;
        if ((row.slices & 0x3FFFU) == 0U) {
            row.status = (uint32_t)status;
            row.cycles_low = (uint32_t)total;
            row.cycles_high = (uint32_t)(total >> 32U);
            nf_bench_output.rows[row_index] = row;
        }
    }
    row.cycles_low = (uint32_t)total;
    row.cycles_high = (uint32_t)(total >> 32U);
    if (planner != NULL) {
        (void)nf_mcu_slalom_result(planner, &result, required, sizeof(required));
        row.workspace_bytes = (uint32_t)result.workspace_used;
        row.goal_entry_us = result.goal_entry_us;
        row.stop_us = result.stop_us;
        row.expanded = result.expanded_states;
        /* slices counts step(1) calls. The separate counters row records
         * actual work units, including zero-work phase transitions. */
        row.required_edges = result.requirements_complete ? physical_required_count(required, fixture->width, fixture->height) : 0U;
        row.checksum = result.requirements_complete ? checksum(required, cells) : 0U;
        if (status == NF_MCU_SLALOM_EXACT && !result.requirements_complete)
            row.status = 0x40000000U | (uint32_t)status;
        else row.status = (uint32_t)status;
    }
    if (status == NF_MCU_SLALOM_PENDING) row.status |= 0x80000000U;
    nf_bench_output.rows[row_index] = row;
    /* Per-case work-unit and relaxed-edge counters are retained without
     * changing the mailbox ABI: bit17 marks a counters-only row. */
    if (nf_bench_output.row_count < NF_BENCH_MAX_ROWS) {
        memset(&row, 0, sizeof(row));
        row.case_id = case_id | 0x20000U;
        row.status = (uint32_t)status;
        row.slices = result.work_units;
        row.expanded = result.expanded_states;
        row.required_edges = result.relaxed_edges;
        row.workspace_bytes = (uint32_t)result.workspace_used;
        row.checksum = result.heap_peak;
        row_index = nf_bench_output.row_count;
        nf_bench_output.rows[row_index] = row;
        nf_bench_output.row_count = row_index + 1U;
    }
}
void nf_bench_run(void)
{
    nf_bench_output.reserved = sizeof(projected) + sizeof(required);
    for (size_t i = 0; i < NF_MCU_BENCHMARK_CASE_COUNT; ++i) {
#ifdef NF_BENCH_CASE_INDEX
        const uint32_t original_index = NF_BENCH_CASE_INDEX;
#else
        const uint32_t original_index = (uint32_t)i;
#endif
        run_case(&nf_mcu_benchmark_cases[i], original_index, 0U);
        run_case(&nf_mcu_benchmark_cases[i], original_index, 1U);
    }
}
