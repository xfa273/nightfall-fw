#include "f413_trace_log.h"

#include "f413_trace_compact.h"
#include "f413_trace_flags.h"
#include "stm32f4xx_hal.h"
#include "trace.h"

#define F413_TRACE_LOG_AUTO_FLUSH_RECORDS_PER_STEP (8U)
#define F413_TRACE_LOG_AUTO_HEADER_COMMIT_RECORDS (8U)
#define F413_TRACE_LOG_AUTO_FLAG (0x8000U)

static volatile uint8_t g_trace_log_auto_enabled = 0U;
static volatile uint8_t g_trace_log_auto_starting = 0U;
static volatile uint8_t g_trace_log_auto_stopping = 0U;
static volatile uint8_t g_trace_log_idle_scratch_borrowed = 0U;
static uint32_t g_trace_log_auto_period_ms = 1U;
static volatile uint32_t g_trace_log_auto_last_sample_ms = 0U;
static volatile uint8_t g_trace_log_auto_last_sample_valid = 0U;
static volatile uint16_t g_trace_log_auto_mode_flags = 0U;
typedef union
{
  struct
  {
    f413_trace_compact_fast_t fast[F413_TRACE_COMPACT_FAST_RECORDS];
    f413_trace_compact_slow_t slow[F413_TRACE_COMPACT_SLOW_RECORDS];
  } trace;
  uint8_t scratch[F413_TRACE_LOG_IDLE_SCRATCH_BYTES];
} f413_trace_log_workspace_t;

static f413_trace_log_workspace_t g_trace_log_workspace;
static volatile uint32_t g_trace_log_auto_buffer_head = 0U;
static volatile uint32_t g_trace_log_auto_buffer_tail = 0U;
static volatile uint8_t g_trace_log_auto_buffer_overflow = 0U;
static volatile uint32_t g_trace_log_auto_buffer_overflow_count = 0U;
static volatile uint32_t g_trace_log_auto_compact_clipped_count = 0U;
static nvm_trace_log_header_t g_trace_log_auto_nvm_header;
static uint8_t g_trace_log_auto_nvm_header_valid = 0U;
static uint32_t g_trace_log_auto_uncommitted_records = 0U;
static uint32_t g_trace_log_auto_flushed_records = 0U;
static uint8_t g_trace_log_auto_nvm_error = 0U;
static nvm_status_t g_trace_log_auto_nvm_status = NVM_STATUS_OK;
static f413_trace_log_fill_control_sample_fn g_fill_control_sample = 0;
static f413_trace_log_void_callback_t g_update_observe_cache = 0;
static f413_trace_log_void_callback_t g_reset_observe_state = 0;

static void f413_trace_log_auto_expand_record(uint32_t index,
                                               nvm_trace_log_record_t* out)
{
  const f413_trace_compact_fast_t* fast =
      &g_trace_log_workspace.trace.fast[
          index % F413_TRACE_COMPACT_FAST_RECORDS];
  const f413_trace_compact_slow_t* slow =
      &g_trace_log_workspace.trace.slow[
          (index / F413_TRACE_COMPACT_SLOW_PERIOD_RECORDS) %
          F413_TRACE_COMPACT_SLOW_RECORDS];

  /* Auto-sample sequence is identical to its monotonic staging index. */
  f413_trace_compact_expand(fast, slow, index, out);
}

_Static_assert(sizeof(g_trace_log_workspace) == F413_TRACE_LOG_IDLE_SCRATCH_BYTES,
               "idle scratch size must match the auto-trace staging buffer");

static bool f413_trace_log_auto_defer_nvm_flush(void)
{
  const uint16_t motor_mask = (uint16_t)(NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG |
                                         NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG |
                                         NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG);

  return (g_trace_log_auto_mode_flags & motor_mask) != 0U;
}

static nvm_status_t f413_trace_log_auto_flush_buffer(void)
{
  nvm_status_t st;

  if (g_trace_log_auto_nvm_header_valid == 0U)
  {
    return NVM_STATUS_INTEGRITY_ERROR;
  }

  while (g_trace_log_auto_buffer_tail != g_trace_log_auto_buffer_head)
  {
    nvm_trace_log_record_t rec;
    uint8_t commit_header =
        (g_trace_log_auto_uncommitted_records + 1U >= F413_TRACE_LOG_AUTO_HEADER_COMMIT_RECORDS) ? 1U : 0U;

    f413_trace_log_auto_expand_record(g_trace_log_auto_buffer_tail, &rec);
    st = nvm_trace_log_append_cached(&g_trace_log_auto_nvm_header, &rec, commit_header);
    if (st != NVM_STATUS_OK)
    {
      g_trace_log_auto_nvm_status = st;
      g_trace_log_auto_nvm_error = 1U;
      return st;
    }
    g_trace_log_auto_buffer_tail += 1U;
    g_trace_log_auto_flushed_records += 1U;
    g_trace_log_auto_uncommitted_records += 1U;
    if (commit_header != 0U)
    {
      g_trace_log_auto_uncommitted_records = 0U;
    }
  }

  if (g_trace_log_auto_uncommitted_records != 0U)
  {
    st = nvm_trace_log_commit_header(&g_trace_log_auto_nvm_header);
    if (st != NVM_STATUS_OK)
    {
      g_trace_log_auto_nvm_status = st;
      g_trace_log_auto_nvm_error = 1U;
      return st;
    }
    g_trace_log_auto_uncommitted_records = 0U;
  }
  return NVM_STATUS_OK;
}

static nvm_status_t f413_trace_log_auto_flush_step(void)
{
  nvm_trace_log_record_t rec;
  nvm_status_t st;
  uint8_t commit_header;

  if (g_trace_log_auto_buffer_tail == g_trace_log_auto_buffer_head)
  {
    return NVM_STATUS_OK;
  }
  if (g_trace_log_auto_nvm_header_valid == 0U)
  {
    g_trace_log_auto_nvm_status = NVM_STATUS_INTEGRITY_ERROR;
    g_trace_log_auto_nvm_error = 1U;
    return NVM_STATUS_INTEGRITY_ERROR;
  }

  f413_trace_log_auto_expand_record(g_trace_log_auto_buffer_tail, &rec);
  commit_header =
      (g_trace_log_auto_uncommitted_records + 1U >= F413_TRACE_LOG_AUTO_HEADER_COMMIT_RECORDS) ? 1U : 0U;
  st = nvm_trace_log_append_cached(&g_trace_log_auto_nvm_header, &rec, commit_header);
  if (st != NVM_STATUS_OK)
  {
    g_trace_log_auto_nvm_status = st;
    g_trace_log_auto_nvm_error = 1U;
    return st;
  }

  g_trace_log_auto_buffer_tail += 1U;
  g_trace_log_auto_flushed_records += 1U;
  g_trace_log_auto_uncommitted_records += 1U;
  if (commit_header != 0U)
  {
    g_trace_log_auto_uncommitted_records = 0U;
  }
  return NVM_STATUS_OK;
}

void f413_trace_log_config(f413_trace_log_fill_control_sample_fn fill_control_sample,
                           f413_trace_log_void_callback_t update_observe_cache,
                           f413_trace_log_void_callback_t reset_observe_state)
{
  g_fill_control_sample = fill_control_sample;
  g_update_observe_cache = update_observe_cache;
  g_reset_observe_state = reset_observe_state;
}

bool f413_trace_log_auto_is_enabled(void)
{
  return g_trace_log_auto_enabled != 0U;
}

uint16_t f413_trace_log_get_mode_flags(void)
{
  return g_trace_log_auto_mode_flags;
}

void f413_trace_log_set_mode_flags(uint16_t mode_flags)
{
  g_trace_log_auto_mode_flags = mode_flags;
}

void f413_trace_log_set_period_ms(uint32_t period_ms)
{
  if (period_ms < 1U)
  {
    period_ms = 1U;
  }
  else if (period_ms > 1000U)
  {
    period_ms = 1000U;
  }
  g_trace_log_auto_period_ms = period_ms;
}

void f413_trace_log_auto_abort(void)
{
  g_trace_log_auto_enabled = 0U;
  g_trace_log_auto_mode_flags = 0U;
}

void f413_trace_log_auto_start(void)
{
  nvm_status_t st;
  uint32_t primask;
  bool already_running;
  bool workspace_busy;

  primask = __get_PRIMASK();
  __disable_irq();
  already_running = g_trace_log_auto_enabled != 0U;
  workspace_busy = (g_trace_log_auto_starting != 0U) ||
                   (g_trace_log_auto_stopping != 0U) ||
                   (g_trace_log_idle_scratch_borrowed != 0U);
  if (!already_running && !workspace_busy)
  {
    g_trace_log_auto_starting = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (already_running)
  {
    trace_printf("[TRACE-LOG] auto: already running\r\n");
    return;
  }
  if (workspace_busy)
  {
    trace_printf("[TRACE-LOG] auto: busy (idle scratch borrowed)\r\n");
    return;
  }

  st = nvm_trace_log_format();
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] auto: FAIL(format NVM=%d)\r\n", (int)st);
    goto release_start_gate;
  }

  st = nvm_trace_log_get_header(&g_trace_log_auto_nvm_header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] auto: FAIL(header NVM=%d)\r\n", (int)st);
    goto release_start_gate;
  }

  g_trace_log_auto_mode_flags = 0U;
  g_trace_log_auto_buffer_head = 0U;
  g_trace_log_auto_buffer_tail = 0U;
  g_trace_log_auto_buffer_overflow = 0U;
  g_trace_log_auto_buffer_overflow_count = 0U;
  g_trace_log_auto_compact_clipped_count = 0U;
  g_trace_log_auto_last_sample_ms = 0U;
  g_trace_log_auto_last_sample_valid = 0U;
  g_trace_log_auto_nvm_header_valid = 1U;
  g_trace_log_auto_uncommitted_records = 0U;
  g_trace_log_auto_flushed_records = 0U;
  g_trace_log_auto_nvm_error = 0U;
  g_trace_log_auto_nvm_status = NVM_STATUS_OK;
  if (g_reset_observe_state != 0)
  {
    g_reset_observe_state();
  }
  if (g_update_observe_cache != 0)
  {
    g_update_observe_cache();
  }
  primask = __get_PRIMASK();
  __disable_irq();
  g_trace_log_auto_enabled = 1U;
  g_trace_log_auto_starting = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
  trace_printf("[TRACE-LOG] auto: START period=%lu ms cap=%lu rec staging=%u fast/%u slow@%ums\r\n",
               (unsigned long)g_trace_log_auto_period_ms,
               (unsigned long)g_trace_log_auto_nvm_header.record_capacity,
               (unsigned int)F413_TRACE_COMPACT_USABLE_RECORDS,
               (unsigned int)F413_TRACE_COMPACT_SLOW_RECORDS,
               (unsigned int)(F413_TRACE_COMPACT_SLOW_PERIOD_RECORDS *
                              g_trace_log_auto_period_ms));
  return;

release_start_gate:
  primask = __get_PRIMASK();
  __disable_irq();
  g_trace_log_auto_starting = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

bool f413_trace_log_try_borrow_idle_scratch(void** out, size_t* out_bytes)
{
  uint32_t primask;
  bool available;

  if ((out == NULL) || (out_bytes == NULL))
  {
    return false;
  }
  *out = NULL;
  *out_bytes = 0U;

  primask = __get_PRIMASK();
  __disable_irq();
  available = (g_trace_log_auto_enabled == 0U) &&
              (g_trace_log_auto_starting == 0U) &&
              (g_trace_log_auto_stopping == 0U) &&
              (g_trace_log_idle_scratch_borrowed == 0U);
  if (available)
  {
    g_trace_log_idle_scratch_borrowed = 1U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (!available)
  {
    return false;
  }
  *out = (void*)g_trace_log_workspace.scratch;
  *out_bytes = sizeof(g_trace_log_workspace.scratch);
  return true;
}

void f413_trace_log_release_idle_scratch(void* scratch)
{
  uint32_t primask;

  if (scratch != (void*)g_trace_log_workspace.scratch)
  {
    return;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  g_trace_log_idle_scratch_borrowed = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void f413_trace_log_auto_stop(void)
{
  nvm_trace_log_header_t header;
  nvm_status_t st;
  uint32_t primask;
  uint32_t buffered;
  uint32_t flushed;
  uint8_t overflow;
  uint8_t nvm_error;
  bool was_enabled;
  bool workspace_busy;

  primask = __get_PRIMASK();
  __disable_irq();
  was_enabled = g_trace_log_auto_enabled != 0U;
  workspace_busy = (g_trace_log_auto_starting != 0U) ||
                   (g_trace_log_auto_stopping != 0U) ||
                   (g_trace_log_idle_scratch_borrowed != 0U);
  if (was_enabled && !workspace_busy)
  {
    g_trace_log_auto_stopping = 1U;
    g_trace_log_auto_enabled = 0U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (!was_enabled)
  {
    trace_printf("[TRACE-LOG] auto: already stopped\r\n");
    return;
  }
  if (workspace_busy)
  {
    trace_printf("[TRACE-LOG] auto: busy (workspace transition)\r\n");
    return;
  }

  buffered = g_trace_log_auto_buffer_head - g_trace_log_auto_buffer_tail;
  flushed = g_trace_log_auto_flushed_records;
  overflow = g_trace_log_auto_buffer_overflow;
  nvm_error = g_trace_log_auto_nvm_error;
  g_trace_log_auto_mode_flags = 0U;

  st = f413_trace_log_auto_flush_buffer();
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] auto: STOP (flush NVM=%d pending=%lu flushed=%lu)\r\n",
                 (int)st,
                 (unsigned long)buffered,
                 (unsigned long)flushed);
    goto release_stop_gate;
  }

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] auto: STOP (header NVM=%d)\r\n", (int)st);
    goto release_stop_gate;
  }

  trace_printf("[TRACE-LOG] auto: STOP total=%lu stored=%lu pending_start=%lu flushed_start=%lu flushed_total=%lu overflow=%u dropped=%lu compact_clipped=%lu nvm_error=%u\r\n",
               (unsigned long)header.total_records,
               (unsigned long)((header.total_records > header.record_capacity) ? header.record_capacity : header.total_records),
               (unsigned long)buffered,
               (unsigned long)flushed,
               (unsigned long)g_trace_log_auto_flushed_records,
               (unsigned int)overflow,
               (unsigned long)g_trace_log_auto_buffer_overflow_count,
               (unsigned long)g_trace_log_auto_compact_clipped_count,
               (unsigned int)nvm_error);

release_stop_gate:
  primask = __get_PRIMASK();
  __disable_irq();
  g_trace_log_auto_stopping = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void f413_trace_log_auto_stop_after_tail(uint32_t tail_ms)
{
  if ((g_trace_log_auto_enabled != 0U) && (tail_ms != 0U))
  {
    uint32_t deadline = HAL_GetTick() + tail_ms;

    while ((int32_t)(HAL_GetTick() - deadline) < 0)
    {
      f413_trace_log_auto_step();
      HAL_Delay(1U);
    }
    f413_trace_log_auto_step();
  }

  f413_trace_log_auto_stop();
}

void f413_trace_log_auto_step(void)
{
  uint32_t i;

  if (g_trace_log_auto_enabled == 0U)
  {
    return;
  }

  if (g_update_observe_cache != 0)
  {
    g_update_observe_cache();
  }

  if (f413_trace_log_auto_defer_nvm_flush())
  {
    return;
  }

  if (g_trace_log_auto_nvm_error == 0U)
  {
    for (i = 0U; i < F413_TRACE_LOG_AUTO_FLUSH_RECORDS_PER_STEP; i++)
    {
      if (g_trace_log_auto_buffer_tail == g_trace_log_auto_buffer_head)
      {
        break;
      }
      if (f413_trace_log_auto_flush_step() != NVM_STATUS_OK)
      {
        break;
      }
    }
  }
}

void f413_trace_log_auto_tick_sample(uint32_t timestamp_ms)
{
  nvm_trace_log_record_t rec;
  f413_trace_compact_fast_t* fast;
  f413_trace_compact_slow_t* slow;
  uint32_t head;
  uint32_t tail;

  if (g_trace_log_auto_enabled == 0U)
  {
    return;
  }
  if (g_fill_control_sample == 0)
  {
    return;
  }
  if ((g_trace_log_auto_last_sample_valid != 0U) &&
      ((timestamp_ms - g_trace_log_auto_last_sample_ms) < g_trace_log_auto_period_ms))
  {
    return;
  }
  g_trace_log_auto_last_sample_ms = timestamp_ms;
  g_trace_log_auto_last_sample_valid = 1U;

  head = g_trace_log_auto_buffer_head;
  tail = g_trace_log_auto_buffer_tail;
  if ((head - tail) >= F413_TRACE_COMPACT_USABLE_RECORDS)
  {
    g_trace_log_auto_buffer_overflow = 1U;
    g_trace_log_auto_buffer_overflow_count += 1U;
    return;
  }

  g_fill_control_sample(&rec, head, timestamp_ms, g_trace_log_auto_mode_flags);
  rec.flags |= (uint16_t)(F413_TRACE_LOG_AUTO_FLAG | g_trace_log_auto_mode_flags);
  fast = &g_trace_log_workspace.trace.fast[
      head % F413_TRACE_COMPACT_FAST_RECORDS];
  if (!f413_trace_compact_pack_fast(&rec, fast))
  {
    g_trace_log_auto_compact_clipped_count += 1U;
  }
  if ((head % F413_TRACE_COMPACT_SLOW_PERIOD_RECORDS) == 0U)
  {
    slow = &g_trace_log_workspace.trace.slow[
        (head / F413_TRACE_COMPACT_SLOW_PERIOD_RECORDS) %
        F413_TRACE_COMPACT_SLOW_RECORDS];
    f413_trace_compact_pack_slow(&rec, slow);
  }
  g_trace_log_auto_buffer_head = head + 1U;
}
