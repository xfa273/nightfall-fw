#ifndef F413_TRACE_LOG_H_
#define F413_TRACE_LOG_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "f413_trace_compact.h"
#include "nvm.h"
#include "nvm_trace_log.h"

#define F413_TRACE_LOG_IDLE_SCRATCH_BYTES \
  (((size_t)F413_TRACE_COMPACT_FAST_RECORDS * \
    sizeof(f413_trace_compact_fast_t)) + \
   ((size_t)F413_TRACE_COMPACT_SLOW_RECORDS * \
    sizeof(f413_trace_compact_slow_t)))

typedef void (*f413_trace_log_fill_control_sample_fn)(nvm_trace_log_record_t* out,
                                                     uint32_t seq,
                                                     uint32_t timestamp_ms,
                                                     uint16_t mode_flags);
typedef void (*f413_trace_log_void_callback_t)(void);

#define F413_TRACE_LOG_STOP_TAIL_MS_DEFAULT (500U)

void f413_trace_log_config(f413_trace_log_fill_control_sample_fn fill_control_sample,
                           f413_trace_log_void_callback_t update_observe_cache,
                           f413_trace_log_void_callback_t reset_observe_state);
bool f413_trace_log_auto_is_enabled(void);
uint16_t f413_trace_log_get_mode_flags(void);
void f413_trace_log_set_mode_flags(uint16_t mode_flags);
void f413_trace_log_set_period_ms(uint32_t period_ms);
void f413_trace_log_auto_abort(void);
void f413_trace_log_auto_start(void);
void f413_trace_log_auto_stop(void);
void f413_trace_log_auto_stop_after_tail(uint32_t tail_ms);
void f413_trace_log_auto_step(void);
void f413_trace_log_auto_tick_sample(uint32_t timestamp_ms);

/*
 * The large auto-trace staging buffer may be used as temporary workspace by
 * foreground-only diagnostics or path planning before a run, while automatic
 * capture is stopped.  A successful borrower must release the lease on every
 * return path.  The storage contents are disposable and never represent
 * persistent FRAM data.
 */
bool f413_trace_log_try_borrow_idle_scratch(void** out, size_t* out_bytes);
void f413_trace_log_release_idle_scratch(void* scratch);

#endif
