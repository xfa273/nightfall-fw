#ifndef F413_TRACE_LOG_H_
#define F413_TRACE_LOG_H_

#include <stdbool.h>
#include <stdint.h>

#include "nvm.h"
#include "nvm_trace_log.h"

typedef void (*f413_trace_log_fill_control_sample_fn)(nvm_trace_log_record_t* out,
                                                     uint32_t seq,
                                                     uint32_t timestamp_ms,
                                                     uint16_t mode_flags);
typedef void (*f413_trace_log_void_callback_t)(void);

void f413_trace_log_config(f413_trace_log_fill_control_sample_fn fill_control_sample,
                           f413_trace_log_void_callback_t update_observe_cache,
                           f413_trace_log_void_callback_t reset_observe_state);
bool f413_trace_log_auto_is_enabled(void);
uint16_t f413_trace_log_get_mode_flags(void);
void f413_trace_log_set_mode_flags(uint16_t mode_flags);
void f413_trace_log_auto_abort(void);
void f413_trace_log_auto_start(void);
void f413_trace_log_auto_stop(void);
void f413_trace_log_auto_step(void);
void f413_trace_log_auto_tick_sample(uint32_t timestamp_ms);

#endif
