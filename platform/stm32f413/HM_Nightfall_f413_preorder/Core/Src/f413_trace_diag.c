#include "f413_trace_diag.h"

#include <string.h>

#include "build_info.h"
#include "f413_trace_log.h"
#include "nvm.h"
#include "trace.h"

#define F413_TRACE_DIAG_DUMP_MAX_RECORDS (8U)
#define F413_TRACE_DIAG_CSV_MAX_RECORDS (256U)
#define F413_TRACE_DIAG_SELFTEST_RECORDS (16U)
#define F413_TRACE_BIN_MAGIC (0x4254464EUL)
#define F413_TRACE_BIN_VERSION (1UL)

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t version;
  uint32_t schema;
  uint32_t header_size;
  uint32_t record_size;
  uint32_t record_count;
  uint32_t available_count;
  uint32_t payload_checksum;
} f413_trace_bin_frame_t;

static f413_trace_diag_config_t s_config;

static const char* f413_trace_diag_unknown_name(uint8_t value)
{
  (void)value;
  return "unknown";
}

static const char* f413_trace_diag_unknown_case_name(uint8_t mode, uint8_t value)
{
  (void)mode;
  (void)value;
  return "unknown";
}

void f413_trace_diag_config(const f413_trace_diag_config_t* config)
{
  if (config == NULL)
  {
    memset(&s_config, 0, sizeof(s_config));
    return;
  }

  s_config = *config;
}

static void f413_trace_diag_get_context(uint8_t* mode, uint8_t* op_case, uint8_t* sub, uint8_t* test_id)
{
  if (mode != NULL)
  {
    *mode = 0xFFU;
  }
  if (op_case != NULL)
  {
    *op_case = 0xFFU;
  }
  if (sub != NULL)
  {
    *sub = 0xFFU;
  }
  if (test_id != NULL)
  {
    *test_id = 0xFFU;
  }

  if (s_config.get_context != NULL)
  {
    s_config.get_context(mode, op_case, sub, test_id);
  }
}

static const char* f413_trace_diag_op_mode_name(uint8_t mode)
{
  return (s_config.op_mode_name != NULL) ? s_config.op_mode_name(mode) : f413_trace_diag_unknown_name(mode);
}

static const char* f413_trace_diag_op_case_name(uint8_t mode, uint8_t op_case)
{
  return (s_config.op_case_name != NULL) ? s_config.op_case_name(mode, op_case) : f413_trace_diag_unknown_case_name(mode, op_case);
}

static const char* f413_trace_diag_op_sub_name(uint8_t mode, uint8_t sub)
{
  return (s_config.op_sub_name != NULL) ? s_config.op_sub_name(mode, sub) : f413_trace_diag_unknown_case_name(mode, sub);
}

void f413_trace_diag_print_header(const nvm_trace_log_header_t* header)
{
  uint32_t stored;

  if (header == NULL)
  {
    return;
  }

  stored = header->total_records;
  if (stored > header->record_capacity)
  {
    stored = header->record_capacity;
  }

  trace_printf("[TRACE-LOG] header ver=0x%08lX rec_size=%lu cap=%lu write=%lu total=%lu stored=%lu\r\n",
               (unsigned long)header->version,
               (unsigned long)header->record_size,
               (unsigned long)header->record_capacity,
               (unsigned long)header->write_index,
               (unsigned long)header->total_records,
               (unsigned long)stored);
}

void f413_trace_diag_run_format_once(void)
{
  nvm_trace_log_header_t header;
  nvm_status_t st;

  f413_trace_log_auto_abort();

  st = nvm_trace_log_format();
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] format: FAIL (NVM=%d)\r\n", (int)st);
    return;
  }

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] format: FAIL(read header NVM=%d)\r\n", (int)st);
    return;
  }

  trace_printf("[TRACE-LOG] format: PASS\r\n");
  f413_trace_diag_print_header(&header);
}

void f413_trace_diag_run_append_sample_once(void)
{
  nvm_trace_log_header_t header;
  nvm_trace_log_record_t rec;
  nvm_status_t st;

  if (s_config.fill_sample == NULL)
  {
    trace_printf("[TRACE-LOG] append: FAIL(no sample callback)\r\n");
    return;
  }

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] append: FAIL(read header NVM=%d, run q first)\r\n", (int)st);
    return;
  }

  s_config.fill_sample(&rec, header.total_records);

  st = nvm_trace_log_append(&rec);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] append: FAIL(write NVM=%d)\r\n", (int)st);
    return;
  }

  trace_printf("[TRACE-LOG] append: PASS seq=%lu ts=%lu dist=%ld angle=%ld enc=(%d,%d) motor=(%d,%d) flags=0x%04X\r\n",
               (unsigned long)rec.seq,
               (unsigned long)rec.timestamp_ms,
               (long)rec.distance_mm,
               (long)rec.angle_mdeg,
               (int)rec.encoder_l,
               (int)rec.encoder_r,
               (int)rec.motor_out_l,
               (int)rec.motor_out_r,
               (unsigned int)rec.flags);
}

void f413_trace_diag_run_dump_latest_once(void)
{
  nvm_trace_log_header_t header;
  nvm_status_t st;
  uint32_t available;
  uint32_t dump_count;
  uint32_t i;

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] dump: FAIL(read header NVM=%d, run q first)\r\n", (int)st);
    return;
  }

  f413_trace_diag_print_header(&header);

  available = header.total_records;
  if (available > header.record_capacity)
  {
    available = header.record_capacity;
  }
  if (available == 0U)
  {
    trace_printf("[TRACE-LOG] dump: no records\r\n");
    return;
  }

  dump_count = available;
  if (dump_count > F413_TRACE_DIAG_DUMP_MAX_RECORDS)
  {
    dump_count = F413_TRACE_DIAG_DUMP_MAX_RECORDS;
  }

  trace_printf("[TRACE-LOG] dump latest %lu/%lu\r\n",
               (unsigned long)dump_count,
               (unsigned long)available);

  for (i = 0U; i < dump_count; i++)
  {
    nvm_trace_log_record_t rec;

    st = nvm_trace_log_read_latest(i, &rec);
    if (st != NVM_STATUS_OK)
    {
      trace_printf("[TRACE-LOG] dump: FAIL(read idx=%lu NVM=%d)\r\n",
                   (unsigned long)i,
                   (int)st);
      return;
    }

    trace_printf("[TRACE-LOG] rec[%lu] seq=%lu ts=%lu mode=%u case=%u sub=%u test=%u dist=%ld angle=%ld v=(%ld/%ld) omega=(%ld/%ld) enc=(%d,%d) motor=(%d,%d) flags=0x%04X\r\n",
                 (unsigned long)i,
                 (unsigned long)rec.seq,
                 (unsigned long)rec.timestamp_ms,
                 (unsigned int)rec.op_mode,
                 (unsigned int)rec.op_case,
                 (unsigned int)rec.op_sub,
                 (unsigned int)rec.test_id,
                 (long)rec.distance_mm,
                 (long)rec.angle_mdeg,
                 (long)rec.target_velocity_mm_s,
                 (long)rec.real_velocity_mm_s,
                 (long)rec.target_omega_mdps,
                 (long)rec.real_omega_mdps,
                 (int)rec.encoder_l,
                 (int)rec.encoder_r,
                 (int)rec.motor_out_l,
                 (int)rec.motor_out_r,
                 (unsigned int)rec.flags);
  }
}

static void f413_trace_diag_run_dump_csv_impl(uint32_t max_records)
{
  nvm_trace_log_header_t header;
  nvm_status_t st;
  uint32_t available;
  uint32_t dump_count;
  uint32_t i;
  uint8_t meta_mode;
  uint8_t meta_case;
  uint8_t meta_sub;
  uint8_t meta_test_id;

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] csv: FAIL(read header NVM=%d, run q first)\r\n", (int)st);
    return;
  }

  available = header.total_records;
  if (available > header.record_capacity)
  {
    available = header.record_capacity;
  }
  if (available == 0U)
  {
    trace_printf("[TRACE-LOG] csv: no records\r\n");
    return;
  }

  dump_count = available;
  if ((max_records > 0U) && (dump_count > max_records))
  {
    dump_count = max_records;
  }

  f413_trace_diag_get_context(&meta_mode, &meta_case, &meta_sub, &meta_test_id);
  for (i = 0U; i < dump_count; i++)
  {
    nvm_trace_log_record_t meta_rec;
    st = nvm_trace_log_read_latest(i, &meta_rec);
    if ((st == NVM_STATUS_OK) && (meta_rec.op_mode != 0xFFU))
    {
      meta_mode = meta_rec.op_mode;
      meta_case = meta_rec.op_case;
      meta_sub = meta_rec.op_sub;
      meta_test_id = meta_rec.test_id;
      break;
    }
  }

  trace_printf("[TRACE-LOG] csv latest %lu/%lu (oldest->newest)\r\n",
               (unsigned long)dump_count,
               (unsigned long)available);
  trace_printf("#log_format=nightfall_trace_csv_v5\r\n");
  trace_printf("#fw_target=%s\r\n", FW_TARGET);
  trace_printf("#fw_version=%s\r\n", FW_VERSION);
  trace_printf("#fw_build_type=%s\r\n", FW_BUILD_TYPE);
  trace_printf("#fw_git_sha=%s\r\n", FW_GIT_SHA);
  trace_printf("#fw_git_dirty=%d\r\n", FW_GIT_DIRTY);
  trace_printf("#fw_log_schema=0x%08lX\r\n", (unsigned long)NVM_TRACE_LOG_SCHEMA_VERSION);
  trace_printf("#op_mode=%u\r\n", (unsigned int)meta_mode);
  trace_printf("#op_mode_name=%s\r\n", f413_trace_diag_op_mode_name(meta_mode));
  trace_printf("#op_case=%u\r\n", (unsigned int)meta_case);
  trace_printf("#op_case_name=%s\r\n", f413_trace_diag_op_case_name(meta_mode, meta_case));
  trace_printf("#op_sub=%u\r\n", (unsigned int)meta_sub);
  trace_printf("#op_sub_name=%s\r\n", f413_trace_diag_op_sub_name(meta_mode, meta_sub));
  trace_printf("#op_test_id=%u\r\n", (unsigned int)meta_test_id);
  trace_printf("#op_label=mode%u %s / case%u %s / sub%u %s\r\n",
               (unsigned int)meta_mode,
               f413_trace_diag_op_mode_name(meta_mode),
               (unsigned int)meta_case,
               f413_trace_diag_op_case_name(meta_mode, meta_case),
               (unsigned int)meta_sub,
               f413_trace_diag_op_sub_name(meta_mode, meta_sub));

  if (s_config.emit_extra_csv_meta != NULL)
  {
    s_config.emit_extra_csv_meta();
  }

  trace_printf("#mm_columns=timestamp_ms,seq,op_mode,op_case,op_sub,test_id,");
  trace_printf("target_distance_mm,distance_mm,angle_mdeg,target_velocity_mm_s,real_velocity_mm_s,accel_velocity_mm_s,");
  trace_printf("target_omega_mdps,real_omega_mdps,gyro_z_raw_mdps,target_angle_mdeg,accel_forward_mm_s2,");
  trace_printf("encoder_l,encoder_r,motor_out_l,motor_out_r,adc_fr,adc_r,adc_fl,adc_l,adc_vbat,");
  trace_printf("flags,reserved_i32_0,reserved_i32_1,reserved_i32_2,reserved_i32_3,reserved_u16_0,reserved_u16_1\r\n");

  for (i = dump_count; i > 0U; i--)
  {
    nvm_trace_log_record_t rec;

    st = nvm_trace_log_read_latest(i - 1U, &rec);
    if (st != NVM_STATUS_OK)
    {
      trace_printf("[TRACE-LOG] csv: FAIL(read idx=%lu NVM=%d)\r\n",
                   (unsigned long)(i - 1U),
                   (int)st);
      return;
    }

    trace_printf("%lu,%lu,%u,%u,%u,%u,%.3f,%ld,%ld,%ld,%ld,",
                 (unsigned long)rec.timestamp_ms,
                 (unsigned long)rec.seq,
                 (unsigned int)rec.op_mode,
                 (unsigned int)rec.op_case,
                 (unsigned int)rec.op_sub,
                 (unsigned int)rec.test_id,
                 (double)rec.target_distance_x1000 / 1000.0,
                 (long)rec.distance_mm,
                 (long)rec.angle_mdeg,
                 (long)rec.target_velocity_mm_s,
                 (long)rec.real_velocity_mm_s);
    trace_printf("%ld,%ld,%ld,%ld,%ld,%ld,",
                 (long)rec.accel_velocity_mm_s,
                 (long)rec.target_omega_mdps,
                 (long)rec.real_omega_mdps,
                 (long)rec.gyro_z_raw_mdps,
                 (long)rec.target_angle_mdeg,
                 (long)rec.accel_forward_mm_s2);
    trace_printf("%d,%d,%d,%d,%u,",
                 (int)rec.encoder_l,
                 (int)rec.encoder_r,
                 (int)rec.motor_out_l,
                 (int)rec.motor_out_r,
                 (unsigned int)rec.adc_fr);
    trace_printf("%u,%u,%u,%u,%u,%ld,%ld,%ld,%ld,%u,%u\r\n",
                 (unsigned int)rec.adc_r,
                 (unsigned int)rec.adc_fl,
                 (unsigned int)rec.adc_l,
                 (unsigned int)rec.adc_vbat,
                 (unsigned int)rec.flags,
                 (long)rec.reserved_i32_0,
                 (long)rec.reserved_i32_1,
                 (long)rec.reserved_i32_2,
                 (long)rec.reserved_i32_3,
                 (unsigned int)rec.reserved_u16_0,
                 (unsigned int)rec.reserved_u16_1);
  }

  trace_printf("[TRACE-LOG] csv: done\r\n");
}

void f413_trace_diag_run_dump_csv_once(void)
{
  f413_trace_diag_run_dump_csv_impl(F413_TRACE_DIAG_CSV_MAX_RECORDS);
}

void f413_trace_diag_run_dump_csv_all_once(void)
{
  f413_trace_diag_run_dump_csv_impl(0U);
}

static uint32_t f413_trace_bin_checksum_update(uint32_t sum, const void* data, uint32_t len)
{
  const uint8_t* p = (const uint8_t*)data;
  uint32_t i;

  if (p == NULL)
  {
    return sum;
  }

  for (i = 0U; i < len; i++)
  {
    sum += p[i];
  }
  return sum;
}

static void f413_trace_diag_run_dump_bin_impl(uint32_t max_records)
{
  nvm_trace_log_header_t header;
  f413_trace_bin_frame_t frame;
  nvm_status_t st;
  uint32_t available;
  uint32_t dump_count;
  uint32_t checksum;
  uint32_t i;

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] bin: FAIL(read header NVM=%d, run q first)\r\n", (int)st);
    return;
  }

  available = header.total_records;
  if (available > header.record_capacity)
  {
    available = header.record_capacity;
  }
  if (available == 0U)
  {
    trace_printf("[TRACE-LOG] bin: no records\r\n");
    return;
  }

  dump_count = available;
  if ((max_records > 0U) && (dump_count > max_records))
  {
    dump_count = max_records;
  }

  checksum = f413_trace_bin_checksum_update(0U, &header, (uint32_t)sizeof(header));
  for (i = dump_count; i > 0U; i--)
  {
    nvm_trace_log_record_t rec;
    st = nvm_trace_log_read_latest(i - 1U, &rec);
    if (st != NVM_STATUS_OK)
    {
      trace_printf("[TRACE-LOG] bin: FAIL(read idx=%lu NVM=%d)\r\n",
                   (unsigned long)(i - 1U),
                   (int)st);
      return;
    }
    checksum = f413_trace_bin_checksum_update(checksum, &rec, (uint32_t)sizeof(rec));
  }

  frame.magic = F413_TRACE_BIN_MAGIC;
  frame.version = F413_TRACE_BIN_VERSION;
  frame.schema = NVM_TRACE_LOG_SCHEMA_VERSION;
  frame.header_size = (uint32_t)sizeof(nvm_trace_log_header_t);
  frame.record_size = (uint32_t)sizeof(nvm_trace_log_record_t);
  frame.record_count = dump_count;
  frame.available_count = available;
  frame.payload_checksum = checksum;

  trace_printf("[TRACE-LOG] bin latest %lu/%lu bytes=%lu\r\n",
               (unsigned long)dump_count,
               (unsigned long)available,
               (unsigned long)(sizeof(frame) + sizeof(header) + (dump_count * sizeof(nvm_trace_log_record_t))));
  trace_write((const char*)&frame, sizeof(frame));
  trace_write((const char*)&header, sizeof(header));
  for (i = dump_count; i > 0U; i--)
  {
    nvm_trace_log_record_t rec;
    st = nvm_trace_log_read_latest(i - 1U, &rec);
    if (st != NVM_STATUS_OK)
    {
      trace_printf("[TRACE-LOG] bin: FAIL(read2 idx=%lu NVM=%d)\r\n",
                   (unsigned long)(i - 1U),
                   (int)st);
      return;
    }
    trace_write((const char*)&rec, sizeof(rec));
  }
  trace_printf("\r\n[TRACE-LOG] bin: done\r\n");
}

void f413_trace_diag_run_dump_bin_once(void)
{
  f413_trace_diag_run_dump_bin_impl(F413_TRACE_DIAG_CSV_MAX_RECORDS);
}

void f413_trace_diag_run_dump_bin_all_once(void)
{
  f413_trace_diag_run_dump_bin_impl(0U);
}

static void f413_trace_diag_fill_selftest_record(nvm_trace_log_record_t* out, uint32_t seq)
{
  if (out == NULL)
  {
    return;
  }

  memset(out, 0, sizeof(*out));
  out->seq = seq;
  out->timestamp_ms = 1000U + seq * 10U;
  out->target_distance_x1000 = (int32_t)(5000 + (int32_t)seq);
  out->distance_mm = (int32_t)(10 + (int32_t)seq);
  out->angle_mdeg = (int32_t)(-1000 - (int32_t)seq);
  out->target_velocity_mm_s = (int32_t)(200 + (int32_t)seq);
  out->real_velocity_mm_s = (int32_t)(190 + (int32_t)seq);
  out->accel_velocity_mm_s = (int32_t)(185 + (int32_t)seq);
  out->target_omega_mdps = (int32_t)(3000 + (int32_t)seq);
  out->real_omega_mdps = (int32_t)(2900 + (int32_t)seq);
  out->gyro_z_raw_mdps = (int32_t)(3100 + (int32_t)seq);
  out->target_angle_mdeg = (int32_t)(90000 + (int32_t)seq);
  out->accel_forward_mm_s2 = (int32_t)(50 + (int32_t)seq);
  out->reserved_i32_0 = (int32_t)(1000 + (int32_t)seq);
  out->reserved_i32_1 = (int32_t)(2000 + (int32_t)seq);
  out->reserved_i32_2 = (int32_t)(3000 + (int32_t)seq);
  out->reserved_i32_3 = (int32_t)(4000 + (int32_t)seq);
  out->encoder_l = (int16_t)(100 + (int32_t)seq);
  out->encoder_r = (int16_t)(-100 - (int32_t)seq);
  out->motor_out_l = (int16_t)(200 + (int32_t)(seq * 2U));
  out->motor_out_r = (int16_t)(-200 - (int32_t)(seq * 2U));
  out->adc_fr = (uint16_t)(1000U + seq);
  out->adc_r = (uint16_t)(1100U + seq);
  out->adc_fl = (uint16_t)(1200U + seq);
  out->adc_l = (uint16_t)(1300U + seq);
  out->adc_vbat = (uint16_t)(1400U + seq);
  out->flags = (uint16_t)(0xA500U | (seq & 0x00FFU));
  out->op_mode = 9U;
  out->op_case = 5U;
  out->op_sub = (uint8_t)(seq & 0xFFU);
  out->test_id = (uint8_t)'T';
  out->reserved_u16_0 = (uint16_t)(0x5500U | (seq & 0x00FFU));
  out->reserved_u16_1 = (uint16_t)(0x6600U | (seq & 0x00FFU));
}

static uint8_t f413_trace_diag_record_equals(const nvm_trace_log_record_t* lhs,
                                             const nvm_trace_log_record_t* rhs)
{
  if ((lhs == NULL) || (rhs == NULL))
  {
    return 0U;
  }

  return (memcmp(lhs, rhs, sizeof(*lhs)) == 0) ? 1U : 0U;
}

void f413_trace_diag_run_selftest_once(void)
{
  nvm_trace_log_header_t header;
  nvm_status_t st;
  uint32_t i;

  f413_trace_log_auto_abort();

  st = nvm_trace_log_format();
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG][SELFTEST] FAIL(format NVM=%d)\r\n", (int)st);
    return;
  }

  for (i = 0U; i < F413_TRACE_DIAG_SELFTEST_RECORDS; i++)
  {
    nvm_trace_log_record_t rec;
    f413_trace_diag_fill_selftest_record(&rec, i);

    st = nvm_trace_log_append(&rec);
    if (st != NVM_STATUS_OK)
    {
      trace_printf("[TRACE-LOG][SELFTEST] FAIL(append i=%lu NVM=%d)\r\n",
                   (unsigned long)i,
                   (int)st);
      return;
    }
  }

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG][SELFTEST] FAIL(read header NVM=%d)\r\n", (int)st);
    return;
  }

  if (header.total_records < F413_TRACE_DIAG_SELFTEST_RECORDS)
  {
    trace_printf("[TRACE-LOG][SELFTEST] FAIL(total=%lu)\r\n", (unsigned long)header.total_records);
    return;
  }

  for (i = 0U; i < F413_TRACE_DIAG_SELFTEST_RECORDS; i++)
  {
    nvm_trace_log_record_t got;
    nvm_trace_log_record_t expected;
    uint32_t expected_seq = F413_TRACE_DIAG_SELFTEST_RECORDS - 1U - i;

    st = nvm_trace_log_read_latest(i, &got);
    if (st != NVM_STATUS_OK)
    {
      trace_printf("[TRACE-LOG][SELFTEST] FAIL(read latest i=%lu NVM=%d)\r\n",
                   (unsigned long)i,
                   (int)st);
      return;
    }

    f413_trace_diag_fill_selftest_record(&expected, expected_seq);
    if (f413_trace_diag_record_equals(&got, &expected) == 0U)
    {
      trace_printf("[TRACE-LOG][SELFTEST] FAIL(mismatch i=%lu got_seq=%lu exp_seq=%lu)\r\n",
                   (unsigned long)i,
                   (unsigned long)got.seq,
                   (unsigned long)expected.seq);
      return;
    }
  }

  trace_printf("[TRACE-LOG][SELFTEST] PASS records=%lu\r\n",
               (unsigned long)F413_TRACE_DIAG_SELFTEST_RECORDS);
}
