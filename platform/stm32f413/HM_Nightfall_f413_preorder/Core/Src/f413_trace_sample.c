#include "f413_trace_sample.h"

#include <math.h>
#include <string.h>

#include "f413_control.h"
#include "f413_control_tune_run.h"
#include "f413_hw.h"
#include "f413_nvm_diag.h"
#include "f413_trace_flags.h"
#include "f413_trace_log.h"
#include "f413_wall_runtime.h"
#include "trace.h"

static f413_trace_sample_config_t g_config;
static nvm_status_t g_identity_status = NVM_STATUS_UNSUPPORTED;
static nvm_identity_block_t g_identity;
static bool g_identity_valid = false;
static volatile uint8_t g_context_mode = 0xFFU;
static volatile uint8_t g_context_case = 0xFFU;
static volatile uint8_t g_context_sub = 0xFFU;
static volatile uint8_t g_context_test_id = 0U;
static volatile uint16_t g_adc_fr = 0U;
static volatile uint16_t g_adc_r = 0U;
static volatile uint16_t g_adc_fl = 0U;
static volatile uint16_t g_adc_l = 0U;
static volatile uint16_t g_adc_vbat = 0U;
static volatile int32_t g_reserved_i32_0 = 0;
static volatile int32_t g_reserved_i32_1 = 0;
static volatile int32_t g_reserved_i32_2 = 0;
static volatile int32_t g_reserved_i32_3 = 0;
static volatile uint16_t g_reserved_u16_0 = 0U;
static volatile uint16_t g_reserved_u16_1 = 0U;
static uint8_t g_last_test_id = 0U;
static f413_run_session_abort_reason_t g_last_test_abort_reason = F413_RUN_SESSION_ABORT_NONE;
static float g_last_test_distance_mm = 0.0f;
static float g_last_test_angle_deg = 0.0f;

static uint32_t f413_trace_sample_tick(void)
{
  if (g_config.get_tick_ms != NULL)
  {
    return g_config.get_tick_ms();
  }
  return 0U;
}

static int32_t f413_trace_sample_scale_float(float value, float scale)
{
  float scaled = value * scale;

  if (scaled > 2147483000.0f)
  {
    return 2147483000L;
  }
  if (scaled < -2147483000.0f)
  {
    return -2147483000L;
  }
  return (int32_t)lrintf(scaled);
}

static bool f413_trace_sample_read_adc(uint16_t* fr,
                                       uint16_t* r,
                                       uint16_t* fl,
                                       uint16_t* l,
                                       uint16_t* vbat)
{
  if (g_config.read_adc_raw != NULL)
  {
    return g_config.read_adc_raw(fr, r, fl, l, vbat);
  }
  return false;
}

static void f413_trace_sample_fill_common(nvm_trace_log_record_t* out)
{
  out->target_distance_x1000 = f413_trace_sample_scale_float(f413_ctrl_get_target_distance(), 1000.0f);
  out->distance_mm = f413_trace_sample_scale_float(f413_ctrl_get_distance(), 1.0f);
  out->angle_mdeg = f413_trace_sample_scale_float(f413_ctrl_get_log_angle(), 1000.0f);
  out->target_velocity_mm_s = f413_trace_sample_scale_float(f413_ctrl_get_target_velocity(), 1.0f);
  out->real_velocity_mm_s = f413_trace_sample_scale_float(f413_ctrl_get_real_velocity(), 1.0f);
  out->accel_velocity_mm_s = f413_trace_sample_scale_float(f413_ctrl_get_accel_velocity(), 1.0f);
  out->target_omega_mdps = f413_trace_sample_scale_float(f413_ctrl_get_target_omega(), 1000.0f);
  out->real_omega_mdps = f413_trace_sample_scale_float(f413_ctrl_get_log_real_omega(), 1000.0f);
  out->target_angle_mdeg = f413_trace_sample_scale_float(f413_ctrl_get_target_angle(), 1000.0f);
  out->accel_forward_mm_s2 = f413_trace_sample_scale_float(f413_ctrl_get_accel_forward(), 1.0f);
  out->motor_out_l = f413_ctrl_get_motor_out_l();
  out->motor_out_r = f413_ctrl_get_motor_out_r();
}

static void f413_trace_sample_fill_tune_fields(nvm_trace_log_record_t* out)
{
  out->reserved_i32_0 = f413_trace_sample_scale_float(f413_ctrl_tune_get_reference(), 1000.0f);
  out->reserved_i32_1 = (int32_t)f413_ctrl_tune_get_axis();
  out->reserved_i32_2 = f413_trace_sample_scale_float(f413_ctrl_get_target_distance(), 1000.0f);
  out->reserved_i32_3 = f413_trace_sample_scale_float(f413_ctrl_get_target_angle(), 1000.0f);
  out->reserved_u16_0 = (uint16_t)(((uint16_t)f413_ctrl_tune_get_axis() << 8U) |
                                   (uint16_t)f413_ctrl_tune_get_pattern());
  out->reserved_u16_1 = (uint16_t)f413_ctrl_tune_get_set();
}

static void f413_trace_sample_fill_flags(nvm_trace_log_record_t* out)
{
  out->flags = f413_hw_stop_switch_pressed()
                   ? NIGHTFALL_F413_TRACE_SWITCH_FLAG
                   : 0U;
  if (f413_ctrl_angle_target_enabled())
  {
    out->flags |= NIGHTFALL_F413_TRACE_ANGLE_TARGET_FLAG;
  }
}

static void f413_trace_sample_fill_context(nvm_trace_log_record_t* out)
{
  out->op_mode = g_context_mode;
  out->op_case = g_context_case;
  out->op_sub = g_context_sub;
  out->test_id = g_context_test_id;
}

void f413_trace_sample_config(const f413_trace_sample_config_t* config)
{
  if (config != NULL)
  {
    g_config = *config;
  }
}

void f413_trace_sample_set_identity(nvm_status_t status, const nvm_identity_block_t* identity)
{
  g_identity_status = status;
  g_identity_valid = false;
  if (identity != NULL)
  {
    g_identity = *identity;
    g_identity_valid = true;
  }
}

void f413_trace_sample_set_context(uint8_t mode, uint8_t op_case, uint8_t sub, uint8_t test_id)
{
  g_context_mode = mode;
  g_context_case = op_case;
  g_context_sub = sub;
  g_context_test_id = test_id;
}

void f413_trace_sample_get_context(uint8_t* mode, uint8_t* op_case, uint8_t* sub, uint8_t* test_id)
{
  if (mode != NULL)
  {
    *mode = g_context_mode;
  }
  if (op_case != NULL)
  {
    *op_case = g_context_case;
  }
  if (sub != NULL)
  {
    *sub = g_context_sub;
  }
  if (test_id != NULL)
  {
    *test_id = g_context_test_id;
  }
}

void f413_trace_sample_update_observe_cache(void)
{
  nvm_trace_log_record_t rec;
  uint16_t adc_fr = 0U;
  uint16_t adc_r = 0U;
  uint16_t adc_fl = 0U;
  uint16_t adc_l = 0U;
  uint16_t adc_vbat = 0U;

#if (NIGHTFALL_F413_DISABLE_WALL_TRACE_OBSERVE == 0U)
  memset(&rec, 0, sizeof(rec));
  if (f413_wall_runtime_fill_observe(&rec, f413_trace_log_get_mode_flags()))
  {
    g_adc_fr = rec.adc_fr;
    g_adc_r = rec.adc_r;
    g_adc_fl = rec.adc_fl;
    g_adc_l = rec.adc_l;
    g_adc_vbat = rec.adc_vbat;
    g_reserved_i32_0 = rec.reserved_i32_0;
    g_reserved_i32_1 = rec.reserved_i32_1;
    g_reserved_i32_2 = rec.reserved_i32_2;
    g_reserved_i32_3 = rec.reserved_i32_3;
    g_reserved_u16_0 = rec.reserved_u16_0;
    g_reserved_u16_1 = rec.reserved_u16_1;
    return;
  }
#endif

  if (f413_trace_sample_read_adc(&adc_fr, &adc_r, &adc_fl, &adc_l, &adc_vbat))
  {
    g_adc_fr = adc_fr;
    g_adc_r = adc_r;
    g_adc_fl = adc_fl;
    g_adc_l = adc_l;
    g_adc_vbat = adc_vbat;
  }
}

void f413_trace_sample_emit_extra_csv_meta(void)
{
#if (NIGHTFALL_F413_DISABLE_WALL_TRACE_OBSERVE == 0U)
  trace_printf("#wall_trace_observe=%u\r\n", (unsigned int)F413_WALL_RUNTIME_TRACE_VERSION);
  trace_printf("#wall_trace_reserved_i32=delta_fr,delta_r,delta_fl,delta_l\r\n");
  trace_printf("#wall_trace_reserved_u16_0=flags\r\n");
  trace_printf("#wall_trace_reserved_u16_1=dist_q4_lr\r\n");
#else
  trace_printf("#wall_trace_observe=disabled\r\n");
#endif
  f413_control_tune_emit_extra_csv_meta();
  if ((g_identity_status == NVM_STATUS_OK) && g_identity_valid)
  {
    f413_nvm_diag_emit_identity_meta(&g_identity);
  }
  else
  {
    trace_printf("#fw_identity_status=%d\r\n", (int)g_identity_status);
  }
  if (g_last_test_id != 0U)
  {
    trace_printf("#last_test_id=%c\r\n", (char)g_last_test_id);
    trace_printf("#last_test_status=%s\r\n",
                 (g_last_test_abort_reason == F413_RUN_SESSION_ABORT_NONE) ? "OK" :
                 f413_run_session_abort_reason_to_text(g_last_test_abort_reason));
    trace_printf("#last_test_dist_mm=%.0f\r\n", (double)g_last_test_distance_mm);
    trace_printf("#last_test_angle_deg=%.0f\r\n", (double)g_last_test_angle_deg);
  }
}

void f413_trace_sample_fill(nvm_trace_log_record_t* out, uint32_t seq)
{
  uint16_t adc_fr = 0U;
  uint16_t adc_r = 0U;
  uint16_t adc_fl = 0U;
  uint16_t adc_l = 0U;
  uint16_t adc_vbat = 0U;

  if (out == NULL)
  {
    return;
  }

  memset(out, 0, sizeof(*out));
  out->seq = seq;
  out->timestamp_ms = f413_trace_sample_tick();
  f413_trace_sample_fill_common(out);
  if (f413_ctrl_is_running())
  {
    out->encoder_l = f413_ctrl_get_log_encoder_delta_l();
    out->encoder_r = f413_ctrl_get_log_encoder_delta_r();
  }
  else
  {
    if (g_config.encoder_l_count != NULL)
    {
      out->encoder_l = g_config.encoder_l_count();
    }
    if (g_config.encoder_r_count != NULL)
    {
      out->encoder_r = g_config.encoder_r_count();
    }
  }
#if (NIGHTFALL_F413_DISABLE_WALL_TRACE_OBSERVE == 0U)
  if (f413_wall_runtime_fill_observe(out, f413_trace_log_get_mode_flags()))
  {
    adc_fr = out->adc_fr;
    adc_r = out->adc_r;
    adc_fl = out->adc_fl;
    adc_l = out->adc_l;
    adc_vbat = out->adc_vbat;
  }
  else
#endif
  if (f413_trace_sample_read_adc(&adc_fr, &adc_r, &adc_fl, &adc_l, &adc_vbat))
  {
    out->adc_fr = adc_fr;
    out->adc_r = adc_r;
    out->adc_fl = adc_fl;
    out->adc_l = adc_l;
    out->adc_vbat = adc_vbat;
  }
  if ((f413_trace_log_get_mode_flags() & NIGHTFALL_F413_TRACE_MODE_TUNE_FLAG) != 0U)
  {
    f413_trace_sample_fill_tune_fields(out);
  }
  f413_trace_sample_fill_flags(out);
  f413_trace_sample_fill_context(out);
}

void f413_trace_sample_fill_control(nvm_trace_log_record_t* out,
                                    uint32_t seq,
                                    uint32_t timestamp_ms,
                                    uint16_t mode_flags)
{
  if (out == NULL)
  {
    return;
  }

  memset(out, 0, sizeof(*out));
  out->seq = seq;
  out->timestamp_ms = timestamp_ms;
  f413_trace_sample_fill_common(out);
  out->encoder_l = f413_ctrl_get_log_encoder_delta_l();
  out->encoder_r = f413_ctrl_get_log_encoder_delta_r();
  out->adc_fr = g_adc_fr;
  out->adc_r = g_adc_r;
  out->adc_fl = g_adc_fl;
  out->adc_l = g_adc_l;
  out->adc_vbat = g_adc_vbat;
  out->reserved_i32_0 = g_reserved_i32_0;
  out->reserved_i32_1 = g_reserved_i32_1;
  out->reserved_i32_2 = g_reserved_i32_2;
  out->reserved_i32_3 = g_reserved_i32_3;
  out->reserved_u16_0 = g_reserved_u16_0;
  out->reserved_u16_1 = g_reserved_u16_1;
  (void)f413_wall_runtime_fill_snapshot_fields(out, mode_flags);
  if ((mode_flags & NIGHTFALL_F413_TRACE_MODE_TUNE_FLAG) != 0U)
  {
    f413_trace_sample_fill_tune_fields(out);
  }
  f413_trace_sample_fill_flags(out);
  f413_trace_sample_fill_context(out);
}

void f413_trace_sample_record_result(uint8_t test_id,
                                     f413_run_session_abort_reason_t abort_reason,
                                     float distance_mm,
                                     float angle_deg)
{
  g_last_test_id = test_id;
  g_last_test_abort_reason = abort_reason;
  g_last_test_distance_mm = distance_mm;
  g_last_test_angle_deg = angle_deg;
}
