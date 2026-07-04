#include "f413_wall_runtime.h"

#include <math.h>
#include <string.h>

#include "f413_control.h"
#include "f413_run_features.h"
#include "params.h"
#include "trace.h"

#define F413_WALL_RUNTIME_END_MONITOR_MS        (4000U)
#define F413_WALL_RUNTIME_END_MONITOR_SAMPLE_MS (50U)

#define F413_WALL_RUNTIME_TRACE_FRONT_FLAG      (0x0001U)
#define F413_WALL_RUNTIME_TRACE_RIGHT_FLAG      (0x0002U)
#define F413_WALL_RUNTIME_TRACE_LEFT_FLAG       (0x0004U)
#define F413_WALL_RUNTIME_TRACE_SAT_FLAG        (0x0008U)
#define F413_WALL_RUNTIME_TRACE_END_WALL_R_FLAG (0x0010U)
#define F413_WALL_RUNTIME_TRACE_END_WALL_L_FLAG (0x0020U)
#define F413_WALL_RUNTIME_TRACE_END_R_FLAG      (0x0040U)
#define F413_WALL_RUNTIME_TRACE_END_L_FLAG      (0x0080U)
#define F413_WALL_RUNTIME_TRACE_GATE_FLAG       (0x0100U)
#define F413_WALL_RUNTIME_TRACE_CTRL_FLAG       (0x0200U)
#define F413_WALL_RUNTIME_TRACE_ENABLED_FLAG    (0x8000U)

#ifndef NIGHTFALL_F413_DISABLE_WALL_CONTROL
#define NIGHTFALL_F413_DISABLE_WALL_CONTROL (0U)
#endif

#define F413_WALL_RUNTIME_CTRL_MAX_DEG        (WALL_CTRL_MAX)
#define F413_WALL_RUNTIME_CTRL_LPF_ALPHA      (F413_WALL_CTRL_LPF_ALPHA)
#define F413_WALL_RUNTIME_CTRL_SLEW_DEG       (WALL_CTRL_SLEW_MAX)
#define F413_WALL_RUNTIME_CTRL_MIN_VEL_MM_S   (20.0f)
#define F413_WALL_RUNTIME_CTRL_DERIV_FALL_THR (WALL_CTRL_DERIV_FALL_THR)
#define F413_WALL_RUNTIME_CTRL_DIFF_MARGIN    (30)
#ifndef F413_WALL_RUNTIME_DIAGONAL_THR
#define F413_WALL_RUNTIME_DIAGONAL_THR        (0.0f)
#endif

typedef struct
{
  bool right_wall;
  bool left_wall;
  bool prev_right_wall;
  bool prev_left_wall;
  bool detected_r;
  bool detected_l;
  float dist_r_mm;
  float dist_l_mm;
  int32_t deriv_r;
  int32_t deriv_l;
  int32_t prev_r_delta;
  int32_t prev_l_delta;
  uint8_t deriv_fall_count_r;
  uint8_t deriv_fall_count_l;
  bool initialized;
} f413_wall_runtime_end_state_t;

static f413_wall_runtime_config_t g_config;
static f413_wall_runtime_end_state_t g_wall_end;
static float g_wall_ctrl_angle_deg = 0.0f;
static float g_wall_ctrl_error_lpf = 0.0f;
static float g_wall_ctrl_latest_error = 0.0f;
static bool g_wall_ctrl_active = false;
static float g_wall_ctrl_kp_deg_per_adc = KP_DEFAULT;
static float g_diagonal_ctrl_omega_deg_s = 0.0f;
static float g_diagonal_ctrl_kp_deg_per_adc = 0.0f;
static float g_diagonal_ctrl_thr = F413_WALL_RUNTIME_DIAGONAL_THR;
static bool g_diagonal_ctrl_active = false;

static bool f413_wall_runtime_read_snapshot(f413_wall_sensor_snapshot_t* wall)
{
  return (g_config.read_wall_snapshot != NULL) && g_config.read_wall_snapshot(wall);
}

static void f413_wall_runtime_delay(uint32_t ms)
{
  if (g_config.delay_ms != NULL)
  {
    g_config.delay_ms(ms);
  }
}

static uint32_t f413_wall_runtime_tick(void)
{
  if (g_config.get_tick_ms != NULL)
  {
    return g_config.get_tick_ms();
  }
  return 0U;
}

static uint32_t f413_wall_runtime_monitor_ms(void)
{
  return (g_config.monitor_ms != 0U) ? g_config.monitor_ms : F413_WALL_RUNTIME_END_MONITOR_MS;
}

static uint32_t f413_wall_runtime_monitor_sample_ms(void)
{
  return (g_config.monitor_sample_ms != 0U)
             ? g_config.monitor_sample_ms
             : F413_WALL_RUNTIME_END_MONITOR_SAMPLE_MS;
}

static int32_t f413_wall_runtime_scale_float(float value, float scale)
{
  float scaled;

  if (scale == 0.0f)
  {
    return 0;
  }
  scaled = value / scale;
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

static bool f413_wall_runtime_gate_on(uint16_t mode_flags)
{
  return (g_config.trace_motor_fwd_flag != 0U) &&
         ((mode_flags & g_config.trace_motor_fwd_flag) != 0U);
}

static void f413_wall_runtime_end_reset_from_snapshot(const f413_wall_sensor_snapshot_t* wall)
{
  memset(&g_wall_end, 0, sizeof(g_wall_end));
  if (wall == NULL)
  {
    return;
  }

  g_wall_end.right_wall = wall->r_delta > WALL_END_THR_R_HIGH;
  g_wall_end.left_wall = wall->l_delta > WALL_END_THR_L_HIGH;
  g_wall_end.prev_right_wall = g_wall_end.right_wall;
  g_wall_end.prev_left_wall = g_wall_end.left_wall;
  g_wall_end.prev_r_delta = wall->r_delta;
  g_wall_end.prev_l_delta = wall->l_delta;
  g_wall_end.initialized = true;
}

static void f413_wall_runtime_end_update(const f413_wall_sensor_snapshot_t* wall, bool gate_on)
{
  bool right_wall;
  bool left_wall;

  if (wall == NULL)
  {
    return;
  }
  if (!g_wall_end.initialized)
  {
    f413_wall_runtime_end_reset_from_snapshot(wall);
  }

  g_wall_end.deriv_r = wall->r_delta - g_wall_end.prev_r_delta;
  g_wall_end.deriv_l = wall->l_delta - g_wall_end.prev_l_delta;
  g_wall_end.prev_r_delta = wall->r_delta;
  g_wall_end.prev_l_delta = wall->l_delta;

  right_wall = g_wall_end.right_wall;
  left_wall = g_wall_end.left_wall;

  if (right_wall)
  {
    if (wall->r_delta < WALL_END_THR_R_LOW)
    {
      right_wall = false;
    }
  }
  else if (wall->r_delta > WALL_END_THR_R_HIGH)
  {
    right_wall = true;
  }

  if (left_wall)
  {
    if (wall->l_delta < WALL_END_THR_L_LOW)
    {
      left_wall = false;
    }
  }
  else if (wall->l_delta > WALL_END_THR_L_HIGH)
  {
    left_wall = true;
  }

  if (right_wall && (g_wall_end.deriv_r < -(int32_t)WALL_END_DERIV_FALL_THR))
  {
    if (g_wall_end.deriv_fall_count_r < 255U)
    {
      g_wall_end.deriv_fall_count_r++;
    }
  }
  else
  {
    g_wall_end.deriv_fall_count_r = 0U;
  }

  if (left_wall && (g_wall_end.deriv_l < -(int32_t)WALL_END_DERIV_FALL_THR))
  {
    if (g_wall_end.deriv_fall_count_l < 255U)
    {
      g_wall_end.deriv_fall_count_l++;
    }
  }
  else
  {
    g_wall_end.deriv_fall_count_l = 0U;
  }

  if (g_wall_end.deriv_fall_count_r >= 2U)
  {
    right_wall = false;
    g_wall_end.deriv_fall_count_r = 0U;
  }
  if (g_wall_end.deriv_fall_count_l >= 2U)
  {
    left_wall = false;
    g_wall_end.deriv_fall_count_l = 0U;
  }

  if (g_wall_end.prev_right_wall && !right_wall && gate_on && !g_wall_end.detected_r)
  {
    g_wall_end.detected_r = true;
    g_wall_end.dist_r_mm = f413_ctrl_get_distance();
  }
  if (g_wall_end.prev_left_wall && !left_wall && gate_on && !g_wall_end.detected_l)
  {
    g_wall_end.detected_l = true;
    g_wall_end.dist_l_mm = f413_ctrl_get_distance();
  }

  g_wall_end.right_wall = right_wall;
  g_wall_end.left_wall = left_wall;
  g_wall_end.prev_right_wall = right_wall;
  g_wall_end.prev_left_wall = left_wall;
}

static void f413_wall_runtime_control_reset(void)
{
  g_wall_ctrl_angle_deg = 0.0f;
  g_wall_ctrl_error_lpf = 0.0f;
  g_wall_ctrl_latest_error = 0.0f;
  g_wall_ctrl_active = false;
}

static void f413_wall_runtime_diagonal_reset(void)
{
  g_diagonal_ctrl_omega_deg_s = 0.0f;
  g_diagonal_ctrl_active = false;
}

static bool f413_wall_runtime_control_wall_present(int32_t delta,
                                                   uint16_t base,
                                                   int32_t deriv)
{
  int32_t threshold = (int32_t)base;

  if (deriv < -(int32_t)F413_WALL_RUNTIME_CTRL_DERIV_FALL_THR)
  {
    threshold += F413_WALL_RUNTIME_CTRL_DIFF_MARGIN;
  }
  return delta > threshold;
}

static void f413_wall_runtime_apply_heading_correction(bool straight_gate,
                                                       bool diagonal_gate)
{
  float control_deg_s = 0.0f;

  if (straight_gate && g_wall_ctrl_active &&
      f413_run_features_wall_control_enabled() &&
      !f413_run_features_test_mode_run())
  {
    control_deg_s += g_wall_ctrl_angle_deg;
  }
  if (diagonal_gate && g_diagonal_ctrl_active)
  {
    control_deg_s += g_diagonal_ctrl_omega_deg_s;
  }

  f413_ctrl_set_heading_omega_correction(-control_deg_s);
}

static void f413_wall_runtime_diagonal_update(const f413_wall_sensor_snapshot_t* wall,
                                              bool gate_on)
{
  float diag_control = 0.0f;

  if ((wall == NULL) || !gate_on || wall->saturated ||
      (fabsf(f413_ctrl_get_target_velocity()) < F413_WALL_RUNTIME_CTRL_MIN_VEL_MM_S) ||
      (g_diagonal_ctrl_kp_deg_per_adc == 0.0f))
  {
    f413_wall_runtime_diagonal_reset();
    return;
  }

  if ((wall->fr_delta > g_diagonal_ctrl_thr) && (wall->fl_delta > g_diagonal_ctrl_thr))
  {
    if (wall->fr_delta > wall->fl_delta)
    {
      diag_control = -g_diagonal_ctrl_kp_deg_per_adc *
                     (float)(wall->fr_delta - wall->fl_delta);
    }
    else
    {
      diag_control = g_diagonal_ctrl_kp_deg_per_adc *
                     (float)(wall->fl_delta - wall->fr_delta);
    }
  }
  else if (wall->fr_delta > g_diagonal_ctrl_thr)
  {
    diag_control = -g_diagonal_ctrl_kp_deg_per_adc * (float)wall->fr_delta;
  }
  else if (wall->fl_delta > g_diagonal_ctrl_thr)
  {
    diag_control = g_diagonal_ctrl_kp_deg_per_adc * (float)wall->fl_delta;
  }

  g_diagonal_ctrl_omega_deg_s = diag_control;
  g_diagonal_ctrl_active = (diag_control != 0.0f);
}

static void f413_wall_runtime_control_update(const f413_wall_sensor_snapshot_t* wall, bool gate_on)
{
  float wall_error = 0.0f;
  float target_deg;
  float delta;
  bool right_wall;
  bool left_wall;

#if (NIGHTFALL_F413_DISABLE_WALL_CONTROL != 0U)
  (void)wall;
  (void)gate_on;
  f413_wall_runtime_control_reset();
  return;
#else
  if ((wall == NULL) || !gate_on || wall->saturated ||
      (fabsf(f413_ctrl_get_target_velocity()) < F413_WALL_RUNTIME_CTRL_MIN_VEL_MM_S))
  {
    f413_wall_runtime_control_reset();
    return;
  }

  right_wall = f413_wall_runtime_control_wall_present(wall->r_delta,
                                                      WALL_BASE_R,
                                                      g_wall_end.deriv_r);
  left_wall = f413_wall_runtime_control_wall_present(wall->l_delta,
                                                     WALL_BASE_L,
                                                     g_wall_end.deriv_l);

  if (right_wall && left_wall)
  {
    uint16_t base_l;
    uint16_t base_r;

    f413_wall_sensor_get_control_base(&base_l, &base_r, NULL);
    wall_error = (float)(wall->l_delta - (int32_t)base_l) -
                 (float)(wall->r_delta - (int32_t)base_r);
    g_wall_ctrl_latest_error = wall_error;
  }
  else if (right_wall && !left_wall)
  {
    uint16_t base_r;

    f413_wall_sensor_get_control_base(NULL, &base_r, NULL);
    wall_error = -2.0f * (float)(wall->r_delta - (int32_t)base_r);
    g_wall_ctrl_latest_error = -1.0f * (float)(wall->r_delta - (int32_t)base_r);
  }
  else if (!right_wall && left_wall)
  {
    uint16_t base_l;

    f413_wall_sensor_get_control_base(&base_l, NULL, NULL);
    wall_error = 2.0f * (float)(wall->l_delta - (int32_t)base_l);
    g_wall_ctrl_latest_error = (float)(wall->l_delta - (int32_t)base_l);
  }
  else
  {
    wall_error = 0.0f;
    g_wall_ctrl_latest_error = 0.0f;
  }

  g_wall_ctrl_error_lpf += F413_WALL_RUNTIME_CTRL_LPF_ALPHA *
                           (wall_error - g_wall_ctrl_error_lpf);
  target_deg = g_wall_ctrl_error_lpf * g_wall_ctrl_kp_deg_per_adc;
  if (fabsf(target_deg) < WALL_CTRL_MIN)
  {
    target_deg = 0.0f;
  }
  if (target_deg > F413_WALL_RUNTIME_CTRL_MAX_DEG)
  {
    target_deg = F413_WALL_RUNTIME_CTRL_MAX_DEG;
  }
  else if (target_deg < -F413_WALL_RUNTIME_CTRL_MAX_DEG)
  {
    target_deg = -F413_WALL_RUNTIME_CTRL_MAX_DEG;
  }

  delta = target_deg - g_wall_ctrl_angle_deg;
  if (delta > F413_WALL_RUNTIME_CTRL_SLEW_DEG)
  {
    delta = F413_WALL_RUNTIME_CTRL_SLEW_DEG;
  }
  else if (delta < -F413_WALL_RUNTIME_CTRL_SLEW_DEG)
  {
    delta = -F413_WALL_RUNTIME_CTRL_SLEW_DEG;
  }

  g_wall_ctrl_angle_deg += delta;
  g_wall_ctrl_active = (target_deg != 0.0f) ||
                       (g_wall_ctrl_angle_deg != 0.0f) ||
                       (g_wall_ctrl_error_lpf != 0.0f);
#endif
}

static void f413_wall_runtime_fill_snapshot(nvm_trace_log_record_t* out,
                                            const f413_wall_sensor_snapshot_t* wall,
                                            bool gate_on)
{
  out->adc_fr = (uint16_t)wall->fr_delta;
  out->adc_r = (uint16_t)wall->r_delta;
  out->adc_fl = (uint16_t)wall->fl_delta;
  out->adc_l = (uint16_t)wall->l_delta;
  out->adc_vbat = wall->vbat_on;
  out->reserved_i32_0 = wall->fr_delta;
  out->reserved_i32_1 = wall->r_delta;
  out->reserved_i32_2 = wall->fl_delta;
  out->reserved_i32_3 = wall->l_delta;
  out->reserved_u16_0 = f413_wall_runtime_trace_flags_from_snapshot(wall, gate_on);
}

void f413_wall_runtime_config(const f413_wall_runtime_config_t* config)
{
  if (config != NULL)
  {
    g_config = *config;
  }
}

void f413_wall_runtime_end_clear(void)
{
  memset(&g_wall_end, 0, sizeof(g_wall_end));
}

void f413_wall_runtime_set_control_gains(float kp_wall, float kp_diagonal)
{
  g_wall_ctrl_kp_deg_per_adc = kp_wall;
  g_diagonal_ctrl_kp_deg_per_adc = kp_diagonal;
  g_diagonal_ctrl_thr = F413_WALL_RUNTIME_DIAGONAL_THR;
}

void f413_wall_runtime_control_clear(void)
{
  f413_wall_runtime_control_reset();
  f413_wall_runtime_diagonal_reset();
  f413_wall_runtime_apply_heading_correction(false, false);
}

float f413_wall_runtime_latest_error(void)
{
  return g_wall_ctrl_latest_error;
}

void f413_wall_runtime_control_apply(bool straight_gate)
{
#if (NIGHTFALL_F413_DISABLE_WALL_CONTROL != 0U)
  (void)straight_gate;
  f413_wall_runtime_control_clear();
#else
  if (!straight_gate ||
      !f413_run_features_wall_control_enabled() ||
      f413_run_features_test_mode_run())
  {
    f413_wall_runtime_control_reset();
    f413_wall_runtime_diagonal_reset();
  }
  else
  {
    f413_wall_runtime_diagonal_reset();
  }
  f413_wall_runtime_apply_heading_correction(straight_gate, false);
#endif
}

bool f413_wall_runtime_poll_wall_end(bool straight_gate)
{
  f413_wall_sensor_snapshot_t wall;

  if (!f413_wall_runtime_read_snapshot(&wall))
  {
    f413_wall_runtime_control_reset();
    f413_wall_runtime_apply_heading_correction(false, false);
    return false;
  }

  f413_wall_runtime_end_update(&wall, straight_gate);
  f413_wall_runtime_control_update(&wall,
                                   straight_gate &&
                                   f413_run_features_wall_control_enabled() &&
                                   !f413_run_features_test_mode_run());
  f413_wall_runtime_control_apply(straight_gate);

  return straight_gate && (g_wall_end.detected_r || g_wall_end.detected_l);
}

void f413_wall_runtime_poll_diagonal(bool diagonal_gate)
{
  f413_wall_sensor_snapshot_t wall;

  if (diagonal_gate)
  {
    f413_wall_runtime_control_reset();
  }
  if (!f413_wall_runtime_read_snapshot(&wall))
  {
    f413_wall_runtime_diagonal_reset();
    f413_wall_runtime_apply_heading_correction(false, false);
    return;
  }

  f413_wall_runtime_diagonal_update(&wall, diagonal_gate);
  if (!diagonal_gate)
  {
    f413_wall_runtime_diagonal_reset();
  }
  f413_wall_runtime_apply_heading_correction(false, diagonal_gate);
}

bool f413_wall_runtime_wall_end_detected(float* right_dist_mm, float* left_dist_mm)
{
  if (right_dist_mm != NULL)
  {
    *right_dist_mm = g_wall_end.detected_r ? g_wall_end.dist_r_mm : -1.0f;
  }
  if (left_dist_mm != NULL)
  {
    *left_dist_mm = g_wall_end.detected_l ? g_wall_end.dist_l_mm : -1.0f;
  }
  return g_wall_end.detected_r || g_wall_end.detected_l;
}

bool f413_wall_runtime_front_wall_reached(float ad_sum_threshold)
{
  f413_wall_sensor_snapshot_t wall;
  int32_t threshold;

  if (!f413_wall_runtime_read_snapshot(&wall))
  {
    return false;
  }
  if (ad_sum_threshold <= 0.0f)
  {
    threshold = WALL_BASE_FR + WALL_BASE_FL;
  }
  else
  {
    threshold = (int32_t)lrintf(ad_sum_threshold);
  }
  return wall.front_wall && ((wall.fr_delta + wall.fl_delta) >= threshold);
}

uint16_t f413_wall_runtime_trace_flags_from_snapshot(const f413_wall_sensor_snapshot_t* wall,
                                                     bool gate_on)
{
  uint16_t flags = F413_WALL_RUNTIME_TRACE_ENABLED_FLAG;

  if (wall == NULL)
  {
    return flags;
  }
  if (wall->front_wall)
  {
    flags |= F413_WALL_RUNTIME_TRACE_FRONT_FLAG;
  }
  if (wall->right_wall)
  {
    flags |= F413_WALL_RUNTIME_TRACE_RIGHT_FLAG;
  }
  if (wall->left_wall)
  {
    flags |= F413_WALL_RUNTIME_TRACE_LEFT_FLAG;
  }
  if (wall->saturated)
  {
    flags |= F413_WALL_RUNTIME_TRACE_SAT_FLAG;
  }
  if (g_wall_end.right_wall)
  {
    flags |= F413_WALL_RUNTIME_TRACE_END_WALL_R_FLAG;
  }
  if (g_wall_end.left_wall)
  {
    flags |= F413_WALL_RUNTIME_TRACE_END_WALL_L_FLAG;
  }
  if (g_wall_end.detected_r)
  {
    flags |= F413_WALL_RUNTIME_TRACE_END_R_FLAG;
  }
  if (g_wall_end.detected_l)
  {
    flags |= F413_WALL_RUNTIME_TRACE_END_L_FLAG;
  }
  if (gate_on)
  {
    flags |= F413_WALL_RUNTIME_TRACE_GATE_FLAG;
  }

  if (g_wall_ctrl_active || g_diagonal_ctrl_active)
  {
    flags |= F413_WALL_RUNTIME_TRACE_CTRL_FLAG;
  }
  return flags;
}

bool f413_wall_runtime_fill_observe(nvm_trace_log_record_t* out, uint16_t mode_flags)
{
  f413_wall_sensor_snapshot_t wall;
  bool gate_on;
  int32_t dist_r_q4 = 0;
  int32_t dist_l_q4 = 0;

  if (out == NULL)
  {
    return false;
  }
  if (!f413_wall_runtime_read_snapshot(&wall))
  {
    return false;
  }

  gate_on = f413_wall_runtime_gate_on(mode_flags);

  f413_wall_runtime_fill_snapshot(out, &wall, gate_on);

  if (g_wall_end.detected_r)
  {
    dist_r_q4 = f413_wall_runtime_scale_float(g_wall_end.dist_r_mm, 0.25f);
  }
  if (g_wall_end.detected_l)
  {
    dist_l_q4 = f413_wall_runtime_scale_float(g_wall_end.dist_l_mm, 0.25f);
  }
  if (dist_r_q4 < 0)
  {
    dist_r_q4 = 0;
  }
  if (dist_l_q4 < 0)
  {
    dist_l_q4 = 0;
  }
  if (dist_r_q4 > 255)
  {
    dist_r_q4 = 255;
  }
  if (dist_l_q4 > 255)
  {
    dist_l_q4 = 255;
  }
  out->reserved_u16_1 = (uint16_t)(((uint16_t)dist_l_q4 << 8U) | (uint16_t)dist_r_q4);

  return true;
}

bool f413_wall_runtime_fill_snapshot_fields(nvm_trace_log_record_t* out, uint16_t mode_flags)
{
  f413_wall_sensor_snapshot_t wall;

  if (out == NULL)
  {
    return false;
  }
  if (!f413_wall_runtime_read_snapshot(&wall))
  {
    return false;
  }

  f413_wall_runtime_fill_snapshot(out, &wall, f413_wall_runtime_gate_on(mode_flags));
  return true;
}

void f413_wall_runtime_run_end_monitor_once(void)
{
  f413_wall_sensor_snapshot_t wall;
  uint32_t start_ms;
  uint32_t now;
  const uint32_t monitor_ms = f413_wall_runtime_monitor_ms();
  const uint32_t sample_ms = f413_wall_runtime_monitor_sample_ms();

  if (!f413_wall_runtime_read_snapshot(&wall))
  {
    trace_printf("[HW-TEST][WallEnd] FAIL(read initial)\r\n");
    return;
  }

  f413_wall_runtime_end_reset_from_snapshot(&wall);
  trace_printf("[HW-TEST][WallEnd] start %lums sample=%lums R=%ld L=%ld wallR=%u wallL=%u\r\n",
               (unsigned long)monitor_ms,
               (unsigned long)sample_ms,
               (long)wall.r_delta,
               (long)wall.l_delta,
               (unsigned int)g_wall_end.right_wall,
               (unsigned int)g_wall_end.left_wall);

  start_ms = f413_wall_runtime_tick();
  do
  {
    f413_wall_runtime_delay(sample_ms);
    now = f413_wall_runtime_tick();
    if (!f413_wall_runtime_read_snapshot(&wall))
    {
      trace_printf("[HW-TEST][WallEnd] FAIL(read sample)\r\n");
      return;
    }

    f413_wall_runtime_end_update(&wall, true);
    trace_printf("[HW-TEST][WallEnd] t=%lu R=%ld L=%ld dR=%ld dL=%ld wallR=%u wallL=%u endR=%u endL=%u\r\n",
                 (unsigned long)(now - start_ms),
                 (long)wall.r_delta,
                 (long)wall.l_delta,
                 (long)g_wall_end.deriv_r,
                 (long)g_wall_end.deriv_l,
                 (unsigned int)g_wall_end.right_wall,
                 (unsigned int)g_wall_end.left_wall,
                 (unsigned int)g_wall_end.detected_r,
                 (unsigned int)g_wall_end.detected_l);
  } while ((now - start_ms) < monitor_ms);

  trace_printf("[HW-TEST][WallEnd] done endR=%u distR=%.0f endL=%u distL=%.0f\r\n",
               (unsigned int)g_wall_end.detected_r,
               (double)g_wall_end.dist_r_mm,
               (unsigned int)g_wall_end.detected_l,
               (double)g_wall_end.dist_l_mm);
}
