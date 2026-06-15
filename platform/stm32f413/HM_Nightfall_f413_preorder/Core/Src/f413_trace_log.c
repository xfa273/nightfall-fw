#include "f413_trace_log.h"

#include "trace.h"

#define F413_TRACE_LOG_AUTO_BUFFER_RECORDS (1536U)
#define F413_TRACE_LOG_AUTO_FLUSH_RECORDS_PER_STEP (8U)
#define F413_TRACE_LOG_AUTO_HEADER_COMMIT_RECORDS (8U)
#define F413_TRACE_LOG_AUTO_FLAG (0x8000U)

static volatile uint8_t g_trace_log_auto_enabled = 0U;
static uint32_t g_trace_log_auto_period_ms = 1U;
static volatile uint32_t g_trace_log_auto_seq = 0U;
static volatile uint16_t g_trace_log_auto_mode_flags = 0U;
static nvm_trace_log_record_t g_trace_log_auto_buffer[F413_TRACE_LOG_AUTO_BUFFER_RECORDS];
static volatile uint32_t g_trace_log_auto_buffer_head = 0U;
static volatile uint32_t g_trace_log_auto_buffer_tail = 0U;
static volatile uint8_t g_trace_log_auto_buffer_overflow = 0U;
static nvm_trace_log_header_t g_trace_log_auto_nvm_header;
static uint8_t g_trace_log_auto_nvm_header_valid = 0U;
static uint32_t g_trace_log_auto_uncommitted_records = 0U;
static uint32_t g_trace_log_auto_flushed_records = 0U;
static uint8_t g_trace_log_auto_nvm_error = 0U;
static nvm_status_t g_trace_log_auto_nvm_status = NVM_STATUS_OK;
static f413_trace_log_fill_control_sample_fn g_fill_control_sample = 0;
static f413_trace_log_void_callback_t g_update_observe_cache = 0;
static f413_trace_log_void_callback_t g_reset_observe_state = 0;

static nvm_status_t f413_trace_log_auto_flush_buffer(void)
{
  nvm_status_t st;

  if (g_trace_log_auto_nvm_header_valid == 0U)
  {
    return NVM_STATUS_INTEGRITY_ERROR;
  }

  while (g_trace_log_auto_buffer_tail != g_trace_log_auto_buffer_head)
  {
    nvm_trace_log_record_t* rec =
        &g_trace_log_auto_buffer[g_trace_log_auto_buffer_tail % F413_TRACE_LOG_AUTO_BUFFER_RECORDS];
    uint8_t commit_header =
        (g_trace_log_auto_uncommitted_records + 1U >= F413_TRACE_LOG_AUTO_HEADER_COMMIT_RECORDS) ? 1U : 0U;

    st = nvm_trace_log_append_cached(&g_trace_log_auto_nvm_header, rec, commit_header);
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
  nvm_trace_log_record_t* rec;
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

  rec = &g_trace_log_auto_buffer[g_trace_log_auto_buffer_tail % F413_TRACE_LOG_AUTO_BUFFER_RECORDS];
  commit_header =
      (g_trace_log_auto_uncommitted_records + 1U >= F413_TRACE_LOG_AUTO_HEADER_COMMIT_RECORDS) ? 1U : 0U;
  st = nvm_trace_log_append_cached(&g_trace_log_auto_nvm_header, rec, commit_header);
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

void f413_trace_log_auto_abort(void)
{
  g_trace_log_auto_enabled = 0U;
  g_trace_log_auto_mode_flags = 0U;
}

void f413_trace_log_auto_start(void)
{
  nvm_status_t st;

  if (g_trace_log_auto_enabled != 0U)
  {
    trace_printf("[TRACE-LOG] auto: already running\r\n");
    return;
  }

  st = nvm_trace_log_format();
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] auto: FAIL(format NVM=%d)\r\n", (int)st);
    return;
  }

  st = nvm_trace_log_get_header(&g_trace_log_auto_nvm_header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] auto: FAIL(header NVM=%d)\r\n", (int)st);
    return;
  }

  g_trace_log_auto_seq = 0U;
  g_trace_log_auto_mode_flags = 0U;
  g_trace_log_auto_buffer_head = 0U;
  g_trace_log_auto_buffer_tail = 0U;
  g_trace_log_auto_buffer_overflow = 0U;
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
  g_trace_log_auto_enabled = 1U;
  trace_printf("[TRACE-LOG] auto: START period=%lu ms cap=%lu rec (streaming FRAM)\r\n",
               (unsigned long)g_trace_log_auto_period_ms,
               (unsigned long)g_trace_log_auto_nvm_header.record_capacity);
}

void f413_trace_log_auto_stop(void)
{
  nvm_trace_log_header_t header;
  nvm_status_t st;
  uint32_t buffered;
  uint32_t flushed;
  uint8_t overflow;
  uint8_t nvm_error;

  if (g_trace_log_auto_enabled == 0U)
  {
    trace_printf("[TRACE-LOG] auto: already stopped\r\n");
    return;
  }

  g_trace_log_auto_enabled = 0U;
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
    return;
  }

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] auto: STOP (header NVM=%d)\r\n", (int)st);
    return;
  }

  trace_printf("[TRACE-LOG] auto: STOP total=%lu stored=%lu pending_start=%lu flushed_start=%lu flushed_total=%lu overflow=%u nvm_error=%u\r\n",
               (unsigned long)header.total_records,
               (unsigned long)((header.total_records > header.record_capacity) ? header.record_capacity : header.total_records),
               (unsigned long)buffered,
               (unsigned long)flushed,
               (unsigned long)g_trace_log_auto_flushed_records,
               (unsigned int)overflow,
               (unsigned int)nvm_error);
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
  nvm_trace_log_record_t* rec;
  uint32_t head;
  uint32_t tail;
  uint32_t seq;

  if (g_trace_log_auto_enabled == 0U)
  {
    return;
  }
  if (g_fill_control_sample == 0)
  {
    return;
  }

  head = g_trace_log_auto_buffer_head;
  tail = g_trace_log_auto_buffer_tail;
  if ((head - tail) >= F413_TRACE_LOG_AUTO_BUFFER_RECORDS)
  {
    g_trace_log_auto_buffer_overflow = 1U;
    return;
  }

  seq = g_trace_log_auto_seq;
  rec = &g_trace_log_auto_buffer[head % F413_TRACE_LOG_AUTO_BUFFER_RECORDS];
  g_fill_control_sample(rec, seq, timestamp_ms, g_trace_log_auto_mode_flags);
  rec->flags |= (uint16_t)(F413_TRACE_LOG_AUTO_FLAG | g_trace_log_auto_mode_flags);
  g_trace_log_auto_seq = seq + 1U;
  g_trace_log_auto_buffer_head = head + 1U;
}
