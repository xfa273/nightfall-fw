#include "bench.h"
void nf_bench_run(void) {
  volatile uint32_t checksum = 0U;
  uint32_t begin = nf_bench_cycles();
  for (uint32_t i = 0; i < 100000U; ++i) checksum += i;
  nf_bench_output.rows[0].cycles_low = nf_bench_cycles() - begin;
  nf_bench_output.rows[0].checksum = checksum;
  nf_bench_output.rows[0].status = 1U;
  nf_bench_output.row_count = 1U;
}
