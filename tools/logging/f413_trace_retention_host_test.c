/* Real FRAM serializer, auto logger and dump entry points; no hardware access. */
#include "f413_trace_log.h"
#include "f413_trace_diag.h"
#include "f413_trace_flags.h"
#include "trace.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t storage[640U * 1024U], before[sizeof(storage)];
static uint32_t area_bytes, tick;
static unsigned writes, erases;
static nvm_status_t read_status = NVM_STATUS_OK, write_status = NVM_STATUS_OK;
static uint8_t output[4U * 1024U * 1024U];
static size_t output_len;
static uint8_t mode;

nvm_status_t nvm_get_area_info(nvm_area_t area, nvm_area_info_t* out)
{
  assert(area == NVM_AREA_TRACE_LOG);
  *out = (nvm_area_info_t){area, 0x60000, area_bytes, 0};
  return NVM_STATUS_OK;
}
nvm_status_t nvm_read(nvm_area_t area, uint32_t off, void* out, size_t len)
{
  assert(area == NVM_AREA_TRACE_LOG && off <= area_bytes && len <= area_bytes - off);
  if (read_status != NVM_STATUS_OK) return read_status;
  memcpy(out, storage + off, len);
  return NVM_STATUS_OK;
}
nvm_status_t nvm_write(nvm_area_t area, uint32_t off, const void* data, size_t len)
{
  assert(area == NVM_AREA_TRACE_LOG && off <= area_bytes && len <= area_bytes - off);
  if (write_status != NVM_STATUS_OK) return write_status;
  memcpy(storage + off, data, len);
  writes++;
  return NVM_STATUS_OK;
}
nvm_status_t nvm_erase(nvm_area_t area)
{
  assert(area == NVM_AREA_TRACE_LOG);
  erases++; /* Actual F413 FRAM erase is a no-op. */
  return NVM_STATUS_OK;
}
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
bool f413_diag_require_nvm_writes(void) { return false; }
void trace_write(const char* data, size_t len)
{
  assert(len < sizeof(output) - output_len);
  memcpy(output + output_len, data, len);
  output_len += len;
  output[output_len] = 0;
}
int trace_printf(const char* fmt, ...)
{
  char line[2048];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  assert(n >= 0 && (size_t)n < sizeof(line));
  trace_write(line, (size_t)n);
  return n;
}
static void sample(nvm_trace_log_record_t* out, uint32_t seq, uint32_t ts, uint16_t flags)
{
  memset(out, 0, sizeof(*out));
  out->seq = seq;
  out->timestamp_ms = ts;
  out->op_mode = mode;
  out->op_case = 0;
  out->op_sub = 2;
  out->test_id = 12;
  out->target_velocity_mm_s = 2200;
  out->real_omega_mdps = -1500000;
  out->flags = flags;
}
static void capture(uint8_t op_mode, uint32_t ts, uint32_t count)
{
  mode = op_mode;
  f413_trace_log_auto_start();
  assert(f413_trace_log_auto_is_enabled());
  f413_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG);
  unsigned saved_writes = writes;
  for (uint32_t i = 0; i < count; ++i)
  {
    f413_trace_log_auto_tick_sample(ts + i);
    f413_trace_log_auto_step();
  }
  assert(writes == saved_writes); /* No FRAM traffic during motion. */
  f413_trace_log_auto_stop();
  assert(!f413_trace_log_auto_is_enabled());
}
static void save_dump(const char* dir, const char* name, bool binary)
{
  unsigned saved_writes = writes, saved_erases = erases;
  output_len = 0;
  if (binary) f413_trace_diag_run_dump_bin_all_once();
  else f413_trace_diag_run_dump_csv_all_once();
  assert(writes == saved_writes && erases == saved_erases);
  assert(output_len > 0);
  char path[1024];
  int n = snprintf(path, sizeof(path), "%s/%s", dir, name);
  assert(n > 0 && (size_t)n < sizeof(path));
  FILE* file = fopen(path, "wb");
  assert(file && fwrite(output, 1, output_len, file) == output_len);
  assert(fclose(file) == 0);
}
static void finalize(nvm_trace_log_header_t* h)
{
  const uint8_t* bytes = (const uint8_t*)h;
  h->crc = 0;
  for (size_t i = 16; i < sizeof(*h); ++i) h->crc += bytes[i];
}
int main(int argc, char** argv)
{
  assert(argc == 2);
  nvm_trace_log_header_t h;
  nvm_trace_log_record_t rec;
  area_bytes = sizeof(h) + 8U * sizeof(rec);
  f413_trace_log_config(sample, NULL, NULL);
  assert(nvm_trace_log_open(NULL) == NVM_STATUS_INVALID_ARG);
  for (unsigned blank = 0; blank < 2; ++blank)
  {
    memset(storage, blank ? 0xFF : 0, sizeof(storage));
    writes = erases = 0;
    assert(nvm_trace_log_open(&h) == NVM_STATUS_OK);
    assert(h.total_records == 0 && h.record_capacity == 8);
    assert(writes == 1 && erases == 1);
  }

  /* Existing v6 records survive reopening, matching an upgrade or reboot. */
  capture(3, 1000, 4);
  memcpy(before, storage, sizeof(storage));
  unsigned saved_writes = writes, saved_erases = erases;
  assert(nvm_trace_log_open(&h) == NVM_STATUS_OK);
  assert(writes == saved_writes && erases == saved_erases);
  assert(memcmp(before, storage, sizeof(storage)) == 0);
  capture(4, 2000, 1);
  capture(5, 10, 1); /* Reboot timestamp + consecutive one-sample sessions. */
  assert(erases == saved_erases);
  assert(memcmp(before + sizeof(h), storage + sizeof(h), 4 * sizeof(rec)) == 0);
  assert(nvm_trace_log_get_header(&h) == NVM_STATUS_OK && h.total_records == 6);
  assert(nvm_trace_log_read_latest(5, &rec) == NVM_STATUS_OK && rec.seq == 0 && rec.op_mode == 3);
  assert(nvm_trace_log_read_latest(0, &rec) == NVM_STATUS_OK && rec.seq == 0 && rec.op_mode == 5);
  save_dump(argv[1], "full.raw", true);
  save_dump(argv[1], "full.csv", false);
  /* An empty run must not delete records or create a phantom session. */
  capture(6, 3000, 0);
  assert(nvm_trace_log_get_header(&h) == NVM_STATUS_OK && h.total_records == 6);

  capture(7, 4000, 4);
  assert(nvm_trace_log_get_header(&h) == NVM_STATUS_OK && h.total_records == 10);
  assert(nvm_trace_log_read_latest(7, &rec) == NVM_STATUS_OK && rec.seq == 2 && rec.op_mode == 3);
  assert(nvm_trace_log_read_latest(8, &rec) == NVM_STATUS_NOT_FOUND);
  save_dump(argv[1], "wrapped.raw", true);
  save_dump(argv[1], "wrapped.csv", false);

  /* Format/version/checksum errors must never silently wipe retained data. */
  for (unsigned kind = 0; kind < 3; ++kind)
  {
    nvm_trace_log_header_t bad = h;
    if (kind == 0) bad.magic ^= 1U;
    if (kind == 1) bad.version -= 0x10000U;
    if (kind == 2) bad.crc ^= 1U;
    memcpy(storage, &bad, sizeof(bad));
    memcpy(before, storage, sizeof(storage));
    saved_writes = writes; saved_erases = erases;
    assert(nvm_trace_log_open(&bad) != NVM_STATUS_OK);
    f413_trace_log_auto_start();
    assert(!f413_trace_log_auto_is_enabled());
    assert(writes == saved_writes && erases == saved_erases);
    assert(memcmp(before, storage, sizeof(storage)) == 0);
  }
  memcpy(storage, &h, sizeof(h));
  read_status = NVM_STATUS_HW_ERROR;
  assert(nvm_trace_log_open(&h) == NVM_STATUS_HW_ERROR);
  read_status = NVM_STATUS_NOT_FOUND;
  assert(nvm_trace_log_open(&h) == NVM_STATUS_NOT_FOUND);
  read_status = NVM_STATUS_OK;
  write_status = NVM_STATUS_HW_ERROR;
  nvm_trace_log_header_t prior = h;
  assert(nvm_trace_log_append_cached(&h, &rec, 1) == NVM_STATUS_HW_ERROR);
  assert(memcmp(&h, &prior, sizeof(h)) == 0);
  write_status = NVM_STATUS_OK;
  assert(writes == saved_writes && erases == saved_erases);

  h.total_records = UINT32_MAX;
  finalize(&h);
  assert(nvm_trace_log_commit_header(&h) == NVM_STATUS_OK);
  assert(nvm_trace_log_append(&rec) == NVM_STATUS_OK);
  assert(nvm_trace_log_get_header(&h) == NVM_STATUS_OK && h.total_records == UINT32_MAX);
  assert(nvm_trace_log_read_latest(7, &rec) == NVM_STATUS_OK);

  /* Full physical ring and production binary frame size/checksum. */
  area_bytes = sizeof(storage);
  assert(nvm_trace_log_format() == NVM_STATUS_OK);
  assert(nvm_trace_log_get_header(&h) == NVM_STATUS_OK && h.record_capacity == 6301);
  for (uint32_t i = 0; i < h.record_capacity + 3; ++i)
  {
    sample(&rec, i % 1000, i, 0);
    assert(nvm_trace_log_append_cached(&h, &rec, 0) == NVM_STATUS_OK);
  }
  assert(nvm_trace_log_commit_header(&h) == NVM_STATUS_OK);
  save_dump(argv[1], "capacity.raw", true);
  puts("PASS: trace retention, auto capture, blank/corrupt/reboot/wrap/I/O and full dumps");
  return 0;
}
