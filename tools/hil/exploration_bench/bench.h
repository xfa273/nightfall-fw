#ifndef NIGHTFALL_EXPLORATION_BENCH_H
#define NIGHTFALL_EXPLORATION_BENCH_H
#include <stdint.h>

#define NF_BENCH_MAGIC 0x4E464542U
#define NF_BENCH_MAX_ROWS 64U
typedef struct {
  uint32_t case_id, status, cycles_low, cycles_high;
  uint32_t max_slice_cycles, slices, workspace_bytes;
  uint32_t goal_entry_us, stop_us, expanded, required_edges, checksum;
} nf_bench_row;
typedef struct {
  uint32_t magic, version, status, row_count;
  uint32_t cpu_hz, rcc_cr, rcc_pllcfgr, rcc_cfgr;
  uint32_t cfsr, hfsr, fault_pc, reserved;
  nf_bench_row rows[NF_BENCH_MAX_ROWS];
} nf_bench_mailbox;
extern volatile nf_bench_mailbox nf_bench_output;
static inline uint32_t nf_bench_cycles(void) {
  return *(volatile uint32_t *)0xE0001004U;
}
void nf_bench_run(void);
#endif
