/* Actual trace workspace exclusion, with NVM functions replaced by counters.
 * No HAL implementation, persistent storage or motion implementation is linked. */
#include "bench.h"
#include "f413_trace_log.h"
#include "trace.h"
#include <string.h>

static unsigned format_calls, header_calls;

int trace_printf(const char *format, ...)
{
    (void)format;
    return 0;
}

nvm_status_t nvm_trace_log_format(void)
{
    ++format_calls;
    return NVM_STATUS_OK;
}

nvm_status_t nvm_trace_log_get_header(nvm_trace_log_header_t *out)
{
    ++header_calls;
    memset(out, 0, sizeof(*out));
    return NVM_STATUS_OK;
}

#include "../../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_trace_log.c"

void nf_bench_run(void)
{
    void *first = NULL, *second = NULL;
    size_t bytes = 0, second_bytes = 0;
    uint32_t checks = 0;
    if (!f413_trace_log_borrow_exploration(NULL, &bytes)) checks |= 1U;
    if (f413_trace_log_borrow_exploration(&first, &bytes) && first != NULL &&
        bytes >= NF_MCU_SLALOM_WORKSPACE_BYTES && ((uintptr_t)first & 7U) == 0U) checks |= 2U;
    if (!f413_trace_log_borrow_exploration(&second, &second_bytes) &&
        second == NULL && second_bytes == 0U) checks |= 4U;
    /* This must return before even the stub format function is called. */
    f413_trace_log_auto_start();
    if (!f413_trace_log_auto_is_enabled() && format_calls == 0U && header_calls == 0U) checks |= 8U;
    f413_trace_log_release_exploration((void *)((uintptr_t)first + 8U));
    if (!f413_trace_log_borrow_exploration(&second, &second_bytes)) checks |= 16U;
    f413_trace_log_release_exploration(first);
    f413_trace_log_auto_start();
    if (f413_trace_log_auto_is_enabled() && format_calls == 1U && header_calls == 1U) checks |= 32U;
    if (!f413_trace_log_borrow_exploration(&second, &second_bytes)) checks |= 64U;
    f413_trace_log_auto_abort();
    if (f413_trace_log_borrow_exploration(&second, &second_bytes) && second == first) checks |= 128U;
    f413_trace_log_release_exploration(second);
    nf_bench_row *row = (nf_bench_row *)&nf_bench_output.rows[0];
    memset(row, 0, sizeof(*row));
    row->case_id = 5000U;
    row->status = checks == 255U ? 0U : 0x80000000U;
    row->workspace_bytes = (uint32_t)bytes;
    row->checksum = checks;
    row->required_edges = format_calls;
    row->expanded = header_calls;
    nf_bench_output.row_count = 1U;
}
