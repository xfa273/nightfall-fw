#include "f413_search_step.h"

#include <math.h>
#include <string.h>

#include "f413_control.h"
#include "f413_front_match.h"
#include "f413_path_run.h"
#include "f413_run_features.h"
#include "f413_run_session.h"
#include "f413_trace_flags.h"
#include "f413_wall_distance.h"
#include "f413_wall_runtime.h"
#include "nvm_params.h"
#include "nvm_trace_log.h"
#include "params.h"
#include "search.h"
#include "search_run_params.h"
#include "trace.h"

#define F413_SEARCH_STEP_MAZE_WALL_W (0x01U)
#define F413_SEARCH_STEP_MAZE_WALL_S (0x02U)
#define F413_SEARCH_STEP_MAZE_WALL_E (0x04U)
#define F413_SEARCH_STEP_MAZE_WALL_N (0x08U)
#define F413_SEARCH_STEP_MAZE_WALL_KNOWN_MASK (0x0FU)
#define F413_SEARCH_STEP_MAZE_START_FORCED_WALLS (0x07U)
#define F413_SEARCH_STEP_CELL_COUNT ((uint32_t)(MAZE_SIZE * MAZE_SIZE))
#define F413_SEARCH_STEP_SPIN_R720_TARGET_DEG (-720.0f)
#define F413_SEARCH_STEP_DRIVE_WAIT_MS (200U)
#define F413_SEARCH_STEP_REVERSE_VELOCITY_MM_S (-60.0f)
#ifndef F413_SEARCH_STEP_AUTO_MAX_ACTIONS
#define F413_SEARCH_STEP_AUTO_MAX_ACTIONS (3000U)
#endif

#define F413_SEARCH_EVENT_MARKER (0x5345U)
#define F413_SEARCH_EVENT_HEADER_COMMIT_RECORDS (8U)
#define F413_SEARCH_EVENT_SESSION_START (0xE0U)
#define F413_SEARCH_EVENT_PHASE (0xE1U)
#define F413_SEARCH_EVENT_DECISION (0xE2U)
#define F413_SEARCH_EVENT_MOTION_END (0xE3U)
#define F413_SEARCH_EVENT_SESSION_END (0xE4U)
#define F413_SEARCH_EVENT_ROUTE_FAIL (0xE5U)

#define F413_SEARCH_EVENT_PHASE_START (1U)
#define F413_SEARCH_EVENT_PHASE_REACHED (2U)
#define F413_SEARCH_EVENT_PHASE_FULL_COMPLETE (3U)
#define F413_SEARCH_EVENT_PHASE_FULL_UNREACHABLE (4U)

#define F413_SEARCH_EVENT_ROUTE_FAIL_MAX_ACTIONS (1U)
#define F413_SEARCH_EVENT_ROUTE_FAIL_NO_CURRENT_STEP (2U)
#define F413_SEARCH_EVENT_ROUTE_FAIL_NO_NEXT_REL (3U)
#define F413_SEARCH_EVENT_ROUTE_FAIL_MAP_SAVE_GUARD (4U)

#define F413_SEARCH_EVENT_MOTION_ENTRY (1U)
#define F413_SEARCH_EVENT_MOTION_FORWARD (2U)
#define F413_SEARCH_EVENT_MOTION_TURN_R90 (3U)
#define F413_SEARCH_EVENT_MOTION_BACK_TURN (4U)
#define F413_SEARCH_EVENT_MOTION_TURN_L90 (5U)
#define F413_SEARCH_EVENT_MOTION_FINAL_STOP (6U)
#define F413_SEARCH_EVENT_MOTION_REVERSE (7U)

#define F413_SEARCH_EVENT_FLAG_KNOWN_STRAIGHT (0x01U)
#define F413_SEARCH_EVENT_FLAG_ACCELED_BEFORE (0x02U)
#define F413_SEARCH_EVENT_FLAG_ACCELED_AFTER (0x04U)
#define F413_SEARCH_EVENT_FLAG_NEXT_TURN90 (0x08U)

typedef struct {
  float omega_peak_deg_s;
  float t_acc_s;
  float t_cruise_s;
  float t_total_s;
} f413_search_step_smooth_turn_t;

typedef struct {
  bool initialized;
  float fr_mm;
  float fl_mm;
  float front_sum_mm;
} f413_search_step_front_match_filter_t;

static f413_search_step_config_t g_config;
static bool g_session_active = false;
static uint8_t g_search_dual_wall_streak = 0U;
static uint8_t g_search_right_wall_streak = 0U;
static uint8_t g_search_left_wall_streak = 0U;
static bool g_search_event_log_active = false;
static uint32_t g_search_event_log_seq = 0U;
static uint32_t g_search_event_log_uncommitted = 0U;
static nvm_trace_log_header_t g_search_event_log_header;
static nvm_status_t g_search_event_log_status = NVM_STATUS_OK;
static float g_search_sensor_kx = 1.0f;
static bool g_search_wall_read_valid = false;
static f413_wall_sensor_snapshot_t g_search_wall_read_snapshot;
static bool g_search_no_path_exit = false;
static bool g_post_goal_active = false;
static bool g_post_goal_save_pending = false;
static uint16_t g_post_goal_newly_known_cells = 0U;
static uint8_t g_post_goal_known_snapshot[MAZE_SIZE][MAZE_SIZE];
static bool g_post_goal_cell_counted[MAZE_SIZE][MAZE_SIZE];

static uint32_t f413_search_step_tick(void)
{
  if (g_config.get_tick_ms != NULL)
  {
    return g_config.get_tick_ms();
  }
  return 0U;
}

static bool f413_search_step_stop_switch_pressed(void)
{
  return (g_config.stop_switch_pressed != NULL) && g_config.stop_switch_pressed();
}

static bool f413_search_step_read_wall(f413_wall_sensor_snapshot_t* wall)
{
  return (g_config.read_wall_snapshot != NULL) && g_config.read_wall_snapshot(wall);
}

static void f413_search_step_set_mode_flags(uint16_t flags)
{
  if (g_config.trace_set_mode_flags != NULL)
  {
    g_config.trace_set_mode_flags(flags);
  }
}

static void f413_search_step_set_trace_period(uint32_t period_ms)
{
  if (g_config.trace_set_period_ms != NULL)
  {
    g_config.trace_set_period_ms(period_ms);
  }
}

static void f413_search_step_set_front_match_trace(bool active,
                                                    f413_front_match_phase_t phase,
                                                    float fr_mm,
                                                    float fl_mm,
                                                    float position_error_mm,
                                                    float yaw_error_mm,
                                                    uint16_t state_elapsed_ms)
{
  if (g_config.trace_set_front_match != NULL)
  {
    g_config.trace_set_front_match(active,
                                   (uint8_t)phase,
                                   fr_mm,
                                   fl_mm,
                                   position_error_mm,
                                   yaw_error_mm,
                                   state_elapsed_ms);
  }
}

static void f413_search_step_trace_start(void)
{
  if (g_config.trace_on_run_start != NULL)
  {
    g_config.trace_on_run_start();
  }
}

static void f413_search_step_trace_stop(void)
{
  if (g_config.trace_on_run_stop != NULL)
  {
    g_config.trace_on_run_stop();
  }
}

static void f413_search_step_trace_auto_step(void)
{
  if (g_config.trace_auto_step != NULL)
  {
    g_config.trace_auto_step();
  }
}

static bool f413_search_step_trace_auto_is_enabled(void)
{
  return (g_config.trace_auto_is_enabled != NULL) && g_config.trace_auto_is_enabled();
}

static void f413_search_step_set_context(uint8_t mode, uint8_t op_case, uint8_t sub, uint8_t test_id)
{
  if (g_config.trace_set_context != NULL)
  {
    g_config.trace_set_context(mode, op_case, sub, test_id);
  }
}

static void f413_search_step_set_action_context(uint8_t op_case,
                                                uint8_t x,
                                                uint8_t y,
                                                uint8_t dir,
                                                uint8_t next_rel)
{
  uint8_t packed_xy = (uint8_t)(((y & 0x0FU) << 4U) | (x & 0x0FU));
  uint8_t packed_action = (uint8_t)(0x80U |
                                    ((next_rel & 0x03U) << 2U) |
                                    (dir & 0x03U));

  f413_search_step_set_context(1U, op_case, packed_xy, packed_action);
}

static float f413_search_step_sensor_kx_sanitize(float sensor_kx)
{
  if ((sensor_kx < 0.3f) || (sensor_kx > 2.0f))
  {
    return 1.0f;
  }
  return sensor_kx;
}

static bool f413_search_step_wall_delta_present(int32_t delta, uint16_t base)
{
  return (float)delta > ((float)base * g_search_sensor_kx);
}

static int32_t f413_search_step_i32_round(float value)
{
  if (value > 2147483000.0f)
  {
    return 2147483000L;
  }
  if (value < -2147483000.0f)
  {
    return -2147483000L;
  }
  return (int32_t)lrintf(value);
}

static int32_t f413_search_event_pack_pose(uint8_t x,
                                           uint8_t y,
                                           uint8_t dir,
                                           uint8_t next_rel,
                                           uint8_t phase,
                                           uint8_t target,
                                           uint8_t event_flags)
{
  uint32_t packed = 0U;

  packed |= (uint32_t)x;
  packed |= (uint32_t)y << 8U;
  packed |= ((uint32_t)dir & 0x03U) << 16U;
  packed |= ((uint32_t)next_rel & 0x03U) << 18U;
  packed |= ((uint32_t)phase & 0x0FU) << 20U;
  packed |= ((uint32_t)target & 0x0FU) << 24U;
  packed |= ((uint32_t)event_flags & 0x0FU) << 28U;
  return (int32_t)packed;
}

static int32_t f413_search_event_pack_wall(uint16_t rel_wall_info, uint16_t map_cell)
{
  uint32_t packed = (uint32_t)rel_wall_info | ((uint32_t)map_cell << 16U);
  return (int32_t)packed;
}

static int32_t f413_search_event_pack_motion(uint8_t motion_kind,
                                             uint8_t status,
                                             uint16_t duration_ms)
{
  uint32_t packed = (uint32_t)duration_ms;

  packed |= ((uint32_t)status) << 16U;
  packed |= ((uint32_t)motion_kind) << 24U;
  return (int32_t)packed;
}

static void f413_search_event_fill_control(nvm_trace_log_record_t* rec,
                                           const f413_wall_sensor_snapshot_t* wall,
                                           uint16_t mode_flags)
{
  if (rec == NULL)
  {
    return;
  }

  rec->timestamp_ms = f413_search_step_tick();
  rec->target_distance_x1000 = f413_search_step_i32_round(f413_ctrl_get_target_distance() * 1000.0f);
  rec->distance_mm = f413_search_step_i32_round(f413_ctrl_get_distance());
  rec->angle_mdeg = f413_search_step_i32_round(f413_ctrl_get_log_angle() * 1000.0f);
  rec->target_velocity_mm_s = f413_search_step_i32_round(f413_ctrl_get_target_velocity());
  rec->real_velocity_mm_s = f413_search_step_i32_round(f413_ctrl_get_real_velocity());
  rec->accel_velocity_mm_s = f413_search_step_i32_round(f413_ctrl_get_accel_velocity());
  rec->target_omega_mdps = f413_search_step_i32_round(f413_ctrl_get_target_omega() * 1000.0f);
  rec->real_omega_mdps = f413_search_step_i32_round(f413_ctrl_get_log_real_omega() * 1000.0f);
  rec->gyro_z_raw_mdps = f413_search_step_i32_round(f413_ctrl_get_gyro_z_raw() * 1000.0f);
  rec->target_angle_mdeg = f413_search_step_i32_round(f413_ctrl_get_target_angle() * 1000.0f);
  rec->accel_forward_mm_s2 = f413_search_step_i32_round(f413_ctrl_get_accel_forward());
  rec->encoder_l = f413_ctrl_get_log_encoder_delta_l();
  rec->encoder_r = f413_ctrl_get_log_encoder_delta_r();
  rec->motor_out_l = f413_ctrl_get_motor_out_l();
  rec->motor_out_r = f413_ctrl_get_motor_out_r();
  rec->flags = mode_flags;
  if (f413_ctrl_angle_target_enabled())
  {
    rec->flags |= NIGHTFALL_F413_TRACE_ANGLE_TARGET_FLAG;
  }
  if (wall != NULL)
  {
    rec->adc_fr = (uint16_t)wall->fr_delta;
    rec->adc_r = (uint16_t)wall->r_delta;
    rec->adc_fl = (uint16_t)wall->fl_delta;
    rec->adc_l = (uint16_t)wall->l_delta;
    rec->adc_vbat = wall->vbat_on;
  }
  if (g_search_wall_read_valid)
  {
    rec->wall_read_fr = (uint16_t)g_search_wall_read_snapshot.fr_delta;
    rec->wall_read_r = (uint16_t)g_search_wall_read_snapshot.r_delta;
    rec->wall_read_fl = (uint16_t)g_search_wall_read_snapshot.fl_delta;
    rec->wall_read_l = (uint16_t)g_search_wall_read_snapshot.l_delta;
  }
}

static bool f413_search_event_read_wall_optional(f413_wall_sensor_snapshot_t* wall)
{
  if (wall == NULL)
  {
    return false;
  }
  memset(wall, 0, sizeof(*wall));
  return f413_search_step_read_wall(wall);
}

static void f413_search_event_append_record(const nvm_trace_log_record_t* rec)
{
  nvm_status_t st;
  uint8_t commit_header;

  if (!g_search_event_log_active || (rec == NULL))
  {
    return;
  }

  commit_header =
      (g_search_event_log_uncommitted + 1U >= F413_SEARCH_EVENT_HEADER_COMMIT_RECORDS) ? 1U : 0U;
  st = nvm_trace_log_append_cached(&g_search_event_log_header, rec, commit_header);
  if (st != NVM_STATUS_OK)
  {
    if (g_search_event_log_status == NVM_STATUS_OK)
    {
      trace_printf("[SEARCH-EVENT] append: FAIL NVM=%d seq=%lu\r\n",
                   (int)st,
                   (unsigned long)rec->seq);
    }
    g_search_event_log_status = st;
    g_search_event_log_active = false;
    return;
  }
  g_search_event_log_uncommitted++;
  if (commit_header != 0U)
  {
    g_search_event_log_uncommitted = 0U;
  }
}

static void f413_search_event_append(uint8_t event_id,
                                     uint8_t op_case,
                                     uint16_t action,
                                     uint8_t x,
                                     uint8_t y,
                                     uint8_t dir,
                                     uint8_t next_rel,
                                     uint8_t phase,
                                     uint8_t target,
                                     uint8_t event_flags,
                                     int32_t reserved_1,
                                     int32_t reserved_2,
                                     int32_t reserved_3,
                                     uint16_t mode_flags)
{
  nvm_trace_log_record_t rec;
  f413_wall_sensor_snapshot_t wall;
  const f413_wall_sensor_snapshot_t* wall_ptr = NULL;

  memset(&rec, 0, sizeof(rec));
  if (f413_search_event_read_wall_optional(&wall))
  {
    wall_ptr = &wall;
  }

  rec.seq = g_search_event_log_seq++;
  f413_search_event_fill_control(&rec, wall_ptr, mode_flags);
  rec.op_mode = 1U;
  rec.op_case = op_case;
  rec.op_sub = (uint8_t)(action & 0xFFU);
  rec.test_id = event_id;
  rec.reserved_u16_0 = F413_SEARCH_EVENT_MARKER;
  rec.reserved_u16_1 = action;
  rec.reserved_i32_0 = f413_search_event_pack_pose(x, y, dir, next_rel, phase, target, event_flags);
  rec.reserved_i32_1 = reserved_1;
  rec.reserved_i32_2 = reserved_2;
  rec.reserved_i32_3 = reserved_3;

  f413_search_event_append_record(&rec);
}

static bool f413_search_event_start(uint8_t op_case,
                                    uint8_t param_index,
                                    uint8_t phase_count,
                                    const uint8_t* phases)
{
  nvm_status_t st;
  int32_t phase_pack = 0;
  uint8_t i;

  st = nvm_trace_log_format();
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[SEARCH-EVENT] format: FAIL NVM=%d\r\n", (int)st);
    return false;
  }
  st = nvm_trace_log_get_header(&g_search_event_log_header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[SEARCH-EVENT] header: FAIL NVM=%d\r\n", (int)st);
    return false;
  }

  g_search_event_log_seq = 0U;
  g_search_event_log_uncommitted = 0U;
  g_search_event_log_status = NVM_STATUS_OK;
  g_search_event_log_active = true;
  g_search_wall_read_valid = false;

  for (i = 0U; (phases != NULL) && (i < phase_count) && (i < 4U); i++)
  {
    phase_pack |= (int32_t)((uint32_t)(phases[i] & 0x0FU) << (i * 4U));
  }

  f413_search_event_append(F413_SEARCH_EVENT_SESSION_START,
                           op_case,
                           0U,
                           (uint8_t)mouse.x,
                           (uint8_t)mouse.y,
                           (uint8_t)mouse.dir,
                           0U,
                           0U,
                           (phase_count > 0U && phases != NULL) ? phases[0] : 0U,
                           0U,
                           phase_pack,
                           (int32_t)param_index,
                           (int32_t)F413_SEARCH_STEP_AUTO_MAX_ACTIONS,
                           g_config.trace_search_safe_flag);
  trace_printf("[SEARCH-EVENT] start cap=%lu max_actions=%lu\r\n",
               (unsigned long)g_search_event_log_header.record_capacity,
               (unsigned long)F413_SEARCH_STEP_AUTO_MAX_ACTIONS);
  return true;
}

static void f413_search_event_finish(uint8_t op_case,
                                     uint16_t action_count,
                                     bool completed,
                                     bool route_failed,
                                     f413_run_session_abort_reason_t abort_reason)
{
  nvm_status_t st;

  if (!g_search_event_log_active)
  {
    return;
  }

  f413_search_event_append(F413_SEARCH_EVENT_SESSION_END,
                           op_case,
                           action_count,
                           (uint8_t)mouse.x,
                           (uint8_t)mouse.y,
                           (uint8_t)mouse.dir,
                           0U,
                           0U,
                           0U,
                           completed ? 0U : 1U,
                           (int32_t)abort_reason,
                           (int32_t)completed,
                           (int32_t)route_failed,
                           g_config.trace_search_safe_flag);

  if (g_search_event_log_active && (g_search_event_log_uncommitted != 0U))
  {
    st = nvm_trace_log_commit_header(&g_search_event_log_header);
    if (st != NVM_STATUS_OK)
    {
      trace_printf("[SEARCH-EVENT] commit: FAIL NVM=%d\r\n", (int)st);
      g_search_event_log_status = st;
    }
    g_search_event_log_uncommitted = 0U;
  }

  trace_printf("[SEARCH-EVENT] stop total=%lu stored=%lu status=%d\r\n",
               (unsigned long)g_search_event_log_header.total_records,
               (unsigned long)((g_search_event_log_header.total_records > g_search_event_log_header.record_capacity)
                                   ? g_search_event_log_header.record_capacity
                                   : g_search_event_log_header.total_records),
               (int)g_search_event_log_status);
  g_search_event_log_active = false;
}

static uint8_t f413_search_event_flags(bool known_straight,
                                       bool acceled_before,
                                       bool acceled_after,
                                       bool next_is_turn90)
{
  uint8_t flags = 0U;

  if (known_straight)
  {
    flags |= F413_SEARCH_EVENT_FLAG_KNOWN_STRAIGHT;
  }
  if (acceled_before)
  {
    flags |= F413_SEARCH_EVENT_FLAG_ACCELED_BEFORE;
  }
  if (acceled_after)
  {
    flags |= F413_SEARCH_EVENT_FLAG_ACCELED_AFTER;
  }
  if (next_is_turn90)
  {
    flags |= F413_SEARCH_EVENT_FLAG_NEXT_TURN90;
  }
  return flags;
}

static uint8_t f413_search_event_motion_kind_from_rel(uint8_t next_rel)
{
  switch (next_rel)
  {
    case 0U: return F413_SEARCH_EVENT_MOTION_FORWARD;
    case 1U: return F413_SEARCH_EVENT_MOTION_TURN_R90;
    case 2U: return F413_SEARCH_EVENT_MOTION_BACK_TURN;
    case 3U: return F413_SEARCH_EVENT_MOTION_TURN_L90;
    default: return 0U;
  }
}

static void f413_search_event_append_route_fail(uint8_t op_case,
                                                uint16_t action,
                                                uint8_t x,
                                                uint8_t y,
                                                uint8_t dir,
                                                uint8_t phase,
                                                uint8_t target,
                                                bool acceled,
                                                uint8_t reason,
                                                int step)
{
  f413_search_event_append(F413_SEARCH_EVENT_ROUTE_FAIL,
                           op_case,
                           action,
                           x,
                           y,
                           dir,
                           0U,
                           phase,
                           target,
                           f413_search_event_flags(false, acceled, acceled, false),
                           f413_search_event_pack_wall(wall_info, map[y][x]),
                           (int32_t)step,
                           (int32_t)reason,
                           g_config.trace_search_safe_flag);
}

static void f413_search_step_prepare_straight_angle_control(void)
{
  if (!f413_run_features_angle_accum_mode())
  {
    f413_ctrl_reset_angle();
  }
  f413_ctrl_clear_angle_target();
}

static void f413_search_step_prepare_turn_angle_control(void)
{
  if (!f413_run_features_angle_accum_mode())
  {
    f413_ctrl_reset_angle();
  }
  f413_ctrl_clear_angle_target();
  f413_wall_runtime_control_clear();
}

static void f413_search_step_angle_reset_streak_clear(void)
{
  g_search_dual_wall_streak = 0U;
  g_search_right_wall_streak = 0U;
  g_search_left_wall_streak = 0U;
}

static void f413_search_step_angle_reset_streak_update(void)
{
  const bool right_wall = ((wall_info & 0x44U) != 0U);
  const bool left_wall = ((wall_info & 0x11U) != 0U);

  if (!f413_run_features_angle_accum_mode())
  {
    f413_search_step_angle_reset_streak_clear();
    return;
  }

  if (right_wall && left_wall)
  {
    if (g_search_dual_wall_streak < UINT8_MAX)
    {
      g_search_dual_wall_streak++;
    }
  }
  else
  {
    g_search_dual_wall_streak = 0U;
  }

  if (right_wall)
  {
    if (g_search_right_wall_streak < UINT8_MAX)
    {
      g_search_right_wall_streak++;
    }
  }
  else
  {
    g_search_right_wall_streak = 0U;
  }

  if (left_wall)
  {
    if (g_search_left_wall_streak < UINT8_MAX)
    {
      g_search_left_wall_streak++;
    }
  }
  else
  {
    g_search_left_wall_streak = 0U;
  }

  if ((g_search_dual_wall_streak >= SEARCH_ANGLE_RESET_DUAL_WALL_STREAK_CELLS) ||
      (g_search_right_wall_streak >= SEARCH_ANGLE_RESET_SINGLE_WALL_STREAK_CELLS) ||
      (g_search_left_wall_streak >= SEARCH_ANGLE_RESET_SINGLE_WALL_STREAK_CELLS))
  {
    f413_ctrl_reset_angle();
    f413_search_step_angle_reset_streak_clear();
  }
}

static bool f413_search_step_is_goal_cell(uint8_t x, uint8_t y)
{
  static const uint8_t goals[9][2] = {
      {GOAL1_X, GOAL1_Y}, {GOAL2_X, GOAL2_Y}, {GOAL3_X, GOAL3_Y},
      {GOAL4_X, GOAL4_Y}, {GOAL5_X, GOAL5_Y}, {GOAL6_X, GOAL6_Y},
      {GOAL7_X, GOAL7_Y}, {GOAL8_X, GOAL8_Y}, {GOAL9_X, GOAL9_Y},
  };
  uint8_t i;

  for (i = 0U; i < 9U; i++)
  {
    uint8_t gx = goals[i][0];
    uint8_t gy = goals[i][1];
    if (((gx != 0U) || (gy != 0U)) && (gx == x) && (gy == y))
    {
      return true;
    }
  }
  return false;
}

static uint8_t f413_search_step_cell_known_mask(uint16_t cell)
{
  uint8_t mask = 0U;

  if (((cell & 0x80U) != 0U) == ((cell & 0x08U) != 0U))
  {
    mask |= 0x08U;
  }
  if (((cell & 0x40U) != 0U) == ((cell & 0x04U) != 0U))
  {
    mask |= 0x04U;
  }
  if (((cell & 0x20U) != 0U) == ((cell & 0x02U) != 0U))
  {
    mask |= 0x02U;
  }
  if (((cell & 0x10U) != 0U) == ((cell & 0x01U) != 0U))
  {
    mask |= 0x01U;
  }
  return mask;
}

static void f413_search_step_post_goal_reset_baseline(void)
{
  uint8_t x;
  uint8_t y;

  for (y = 0U; y < MAZE_SIZE; y++)
  {
    for (x = 0U; x < MAZE_SIZE; x++)
    {
      g_post_goal_known_snapshot[y][x] = f413_search_step_cell_known_mask(map[y][x]);
      g_post_goal_cell_counted[y][x] = false;
    }
  }
  g_post_goal_newly_known_cells = 0U;
  g_post_goal_save_pending = false;
}

static void f413_search_step_post_goal_track_cell(uint8_t x, uint8_t y)
{
  uint8_t now_known;
  uint8_t prev_known;
  uint8_t newly_known_bits;

  if ((x >= MAZE_SIZE) || (y >= MAZE_SIZE))
  {
    return;
  }

  now_known = f413_search_step_cell_known_mask(map[y][x]);
  prev_known = g_post_goal_known_snapshot[y][x];
  if (now_known == prev_known)
  {
    return;
  }

  g_post_goal_known_snapshot[y][x] = now_known;
  newly_known_bits = (uint8_t)(now_known & (uint8_t)(~prev_known));
  if ((newly_known_bits != 0U) && !g_post_goal_cell_counted[y][x])
  {
    g_post_goal_cell_counted[y][x] = true;
    if (g_post_goal_newly_known_cells < UINT16_MAX)
    {
      g_post_goal_newly_known_cells++;
    }
  }

  if (g_post_goal_newly_known_cells >= SEARCH_POST_GOAL_SAVE_NEW_CELL_THRESHOLD)
  {
    g_post_goal_save_pending = true;
  }
}

static void f413_search_step_post_goal_track_around_mouse(void)
{
  uint8_t x = (uint8_t)mouse.x;
  uint8_t y = (uint8_t)mouse.y;

  if (!g_post_goal_active)
  {
    return;
  }

  f413_search_step_post_goal_track_cell(x, y);
  if ((uint8_t)(y + 1U) < MAZE_SIZE)
  {
    f413_search_step_post_goal_track_cell(x, (uint8_t)(y + 1U));
  }
  if ((uint8_t)(x + 1U) < MAZE_SIZE)
  {
    f413_search_step_post_goal_track_cell((uint8_t)(x + 1U), y);
  }
  if (y > 0U)
  {
    f413_search_step_post_goal_track_cell(x, (uint8_t)(y - 1U));
  }
  if (x > 0U)
  {
    f413_search_step_post_goal_track_cell((uint8_t)(x - 1U), y);
  }

  f413_search_step_post_goal_track_cell(START_X, START_Y);
  if ((START_X + 1U) < MAZE_SIZE)
  {
    f413_search_step_post_goal_track_cell((uint8_t)(START_X + 1U), START_Y);
  }
}

static void f413_search_step_note_goal_visit(uint8_t target)
{
  if (g_post_goal_active ||
      !f413_search_step_is_goal_cell((uint8_t)mouse.x, (uint8_t)mouse.y))
  {
    return;
  }

  g_post_goal_active = true;
  f413_search_step_post_goal_reset_baseline();
  if (target == F413_SEARCH_STEP_TARGET_FULL)
  {
    g_post_goal_save_pending = true;
  }
}

static const char* f413_search_step_target_name(uint8_t target)
{
  switch (target)
  {
    case F413_SEARCH_STEP_TARGET_GOAL: return "goal";
    case F413_SEARCH_STEP_TARGET_FULL: return "full";
    case F413_SEARCH_STEP_TARGET_START: return "start";
    default: return "?";
  }
}

static bool f413_search_step_target_reached(uint8_t target)
{
  if (target == F413_SEARCH_STEP_TARGET_GOAL)
  {
    return f413_search_step_is_goal_cell((uint8_t)mouse.x, (uint8_t)mouse.y);
  }
  if (target == F413_SEARCH_STEP_TARGET_START)
  {
    return (mouse.x == START_X) && (mouse.y == START_Y);
  }
  return false;
}

static bool f413_search_step_plan_from_case(uint8_t op_case,
                                            f413_search_step_case_config_t* out)
{
  if (out == NULL)
  {
    return false;
  }
  memset(out, 0, sizeof(*out));

  switch (op_case)
  {
    case 1U:
      out->param_index = 0U;
      out->phase_count = 2U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_GOAL;
      out->phases[1] = F413_SEARCH_STEP_TARGET_FULL;
      return true;
    case 2U:
      out->param_index = 0U;
      out->phase_count = 1U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_FULL;
      return true;
    case 3U:
      out->param_index = 0U;
      out->phase_count = 2U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_GOAL;
      out->phases[1] = F413_SEARCH_STEP_TARGET_START;
      return true;
    case 4U:
      out->param_index = 0U;
      out->phase_count = 1U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_GOAL;
      return true;
    case 5U:
      out->param_index = 1U;
      out->phase_count = 2U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_GOAL;
      out->phases[1] = F413_SEARCH_STEP_TARGET_FULL;
      return true;
    case 6U:
      out->param_index = 1U;
      out->phase_count = 1U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_FULL;
      return true;
    case 7U:
      out->param_index = 1U;
      out->phase_count = 2U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_GOAL;
      out->phases[1] = F413_SEARCH_STEP_TARGET_START;
      return true;
    case 8U:
      out->param_index = 1U;
      out->phase_count = 1U;
      out->phases[0] = F413_SEARCH_STEP_TARGET_GOAL;
      return true;
    default:
      return false;
  }
}

static void f413_search_step_map_init_empty(void)
{
  uint8_t x;
  uint8_t y;

  for (y = 0U; y < MAZE_SIZE; y++)
  {
    for (x = 0U; x < MAZE_SIZE; x++)
    {
      map[y][x] = 0xF0U;
      visited[y][x] = false;
    }
  }

  for (y = 0U; y < MAZE_SIZE; y++)
  {
    map[y][0] |= 0xF1U;
    map[y][MAZE_SIZE - 1U] |= 0xF4U;
  }
  for (x = 0U; x < MAZE_SIZE; x++)
  {
    map[0][x] |= 0xF2U;
    map[MAZE_SIZE - 1U][x] |= 0xF8U;
  }

  map[START_Y][START_X] |= 0x44U;
  if (START_X > 0U)
  {
    map[START_Y][START_X - 1U] |= 0x44U;
  }
  visited[START_Y][START_X] = true;
}

static uint16_t f413_search_step_wall_info_from_snapshot(const f413_wall_sensor_snapshot_t* wall)
{
  uint16_t info = 0U;

  if (wall == NULL)
  {
    return 0U;
  }
  if (f413_search_step_wall_delta_present(wall->fr_delta, WALL_BASE_FR) ||
      f413_search_step_wall_delta_present(wall->fl_delta, WALL_BASE_FL))
  {
    info |= 0x88U;
  }
  if (f413_search_step_wall_delta_present(wall->r_delta, WALL_BASE_R))
  {
    info |= 0x44U;
  }
  if (f413_search_step_wall_delta_present(wall->l_delta, WALL_BASE_L))
  {
    info |= 0x11U;
  }
  return info;
}

static void f413_search_step_write_map_cell(uint8_t x, uint8_t y, uint8_t dir, uint16_t relative_wall_info)
{
  uint16_t m_temp;

  if ((x >= MAZE_SIZE) || (y >= MAZE_SIZE))
  {
    return;
  }

  m_temp = (relative_wall_info >> (dir & 0x03U)) & F413_SEARCH_STEP_MAZE_WALL_KNOWN_MASK;
  if ((x == START_X) && (y == START_Y))
  {
    m_temp |= F413_SEARCH_STEP_MAZE_START_FORCED_WALLS;
  }
  m_temp |= (uint16_t)(m_temp << 4U);

  map[y][x] = m_temp;
  if (y != (MAZE_SIZE - 1U))
  {
    if ((m_temp & 0x88U) != 0U)
    {
      map[y + 1U][x] |= 0x22U;
    }
    else
    {
      map[y + 1U][x] &= (uint16_t)~0x22U;
    }
  }
  if (x != (MAZE_SIZE - 1U))
  {
    if ((m_temp & 0x44U) != 0U)
    {
      map[y][x + 1U] |= 0x11U;
    }
    else
    {
      map[y][x + 1U] &= (uint16_t)~0x11U;
    }
  }
  if (y != 0U)
  {
    if ((m_temp & 0x22U) != 0U)
    {
      map[y - 1U][x] |= 0x88U;
    }
    else
    {
      map[y - 1U][x] &= (uint16_t)~0x88U;
    }
  }
  if (x != 0U)
  {
    if ((m_temp & 0x11U) != 0U)
    {
      map[y][x - 1U] |= 0x44U;
    }
    else
    {
      map[y][x - 1U] &= (uint16_t)~0x44U;
    }
  }

  map[START_Y][START_X] |= 0x77U;
  if ((START_X + 1U) < MAZE_SIZE)
  {
    map[START_Y][START_X + 1U] |= 0x11U;
  }
}

static int f413_search_step_make_smap(uint8_t x, uint8_t y, uint8_t target)
{
  static uint16_t q[F413_SEARCH_STEP_CELL_COUNT];
  uint16_t head = 0U;
  uint16_t tail = 0U;
  uint8_t ix;
  uint8_t iy;
  static const uint8_t goals[9][2] = {
      {GOAL1_X, GOAL1_Y}, {GOAL2_X, GOAL2_Y}, {GOAL3_X, GOAL3_Y},
      {GOAL4_X, GOAL4_Y}, {GOAL5_X, GOAL5_Y}, {GOAL6_X, GOAL6_Y},
      {GOAL7_X, GOAL7_Y}, {GOAL8_X, GOAL8_Y}, {GOAL9_X, GOAL9_Y},
  };

  if ((x >= MAZE_SIZE) || (y >= MAZE_SIZE))
  {
    return -1;
  }

  for (iy = 0U; iy < MAZE_SIZE; iy++)
  {
    for (ix = 0U; ix < MAZE_SIZE; ix++)
    {
      smap[iy][ix] = 0xFFFFU;
    }
  }

  if (target == F413_SEARCH_STEP_TARGET_START)
  {
    smap[START_Y][START_X] = 0U;
    q[tail++] = (uint16_t)(START_Y * MAZE_SIZE + START_X);
  }
  else if (target == F413_SEARCH_STEP_TARGET_FULL)
  {
    bool any_unvisited = false;

    visited[START_Y][START_X] = true;
    for (iy = 0U; iy < MAZE_SIZE; iy++)
    {
      for (ix = 0U; ix < MAZE_SIZE; ix++)
      {
        if (!visited[iy][ix])
        {
          any_unvisited = true;
          smap[iy][ix] = 0U;
          q[tail++] = (uint16_t)(iy * MAZE_SIZE + ix);
        }
      }
    }
    if (!any_unvisited)
    {
      return -2;
    }
  }
  else
  {
    for (ix = 0U; ix < 9U; ix++)
    {
      uint8_t gx = goals[ix][0];
      uint8_t gy = goals[ix][1];
      if (((gx == 0U) && (gy == 0U)) || (gx >= MAZE_SIZE) || (gy >= MAZE_SIZE))
      {
        continue;
      }
      smap[gy][gx] = 0U;
      q[tail++] = (uint16_t)(gy * MAZE_SIZE + gx);
    }
  }

  while ((head < tail) && (smap[y][x] == 0xFFFFU))
  {
    uint16_t idx = q[head++];
    uint8_t cy = (uint8_t)(idx / MAZE_SIZE);
    uint8_t cx = (uint8_t)(idx - (uint16_t)(cy * MAZE_SIZE));
    uint16_t step = smap[cy][cx];
    uint8_t cell = (uint8_t)(map[cy][cx] & F413_SEARCH_STEP_MAZE_WALL_KNOWN_MASK);

    if (((cell & F413_SEARCH_STEP_MAZE_WALL_N) == 0U) && (cy != (MAZE_SIZE - 1U)) && (smap[cy + 1U][cx] == 0xFFFFU))
    {
      smap[cy + 1U][cx] = (uint16_t)(step + 1U);
      q[tail++] = (uint16_t)((cy + 1U) * MAZE_SIZE + cx);
    }
    if (((cell & F413_SEARCH_STEP_MAZE_WALL_E) == 0U) && (cx != (MAZE_SIZE - 1U)) && (smap[cy][cx + 1U] == 0xFFFFU))
    {
      smap[cy][cx + 1U] = (uint16_t)(step + 1U);
      q[tail++] = (uint16_t)(cy * MAZE_SIZE + (cx + 1U));
    }
    if (((cell & F413_SEARCH_STEP_MAZE_WALL_S) == 0U) && (cy != 0U) && (smap[cy - 1U][cx] == 0xFFFFU))
    {
      smap[cy - 1U][cx] = (uint16_t)(step + 1U);
      q[tail++] = (uint16_t)((cy - 1U) * MAZE_SIZE + cx);
    }
    if (((cell & F413_SEARCH_STEP_MAZE_WALL_W) == 0U) && (cx != 0U) && (smap[cy][cx - 1U] == 0xFFFFU))
    {
      smap[cy][cx - 1U] = (uint16_t)(step + 1U);
      q[tail++] = (uint16_t)(cy * MAZE_SIZE + (cx - 1U));
    }
  }

  return (smap[y][x] == 0xFFFFU) ? -1 : (int)smap[y][x];
}

static int f413_search_step_make_goal_smap(uint8_t x, uint8_t y)
{
  return f413_search_step_make_smap(x, y, F413_SEARCH_STEP_TARGET_GOAL);
}

static bool f413_search_step_path_exists_from_current_map(void)
{
  const int step = f413_search_step_make_goal_smap((uint8_t)mouse.x, (uint8_t)mouse.y);

  return (step >= 0) && (step <= (int)(MAZE_SIZE * MAZE_SIZE - (MAZE_SIZE - 1U)));
}

static bool f413_search_step_try_save_map_safely(HAL_StatusTypeDef* out_status)
{
  HAL_StatusTypeDef st;

  if (out_status != NULL)
  {
    *out_status = HAL_ERROR;
  }
  if (!f413_search_step_path_exists_from_current_map())
  {
    trace_printf("[SEARCH-RUN] skip maze save: no safe route to goal pos=(%u,%u,%u) cell=0x%04X\r\n",
                 (unsigned int)mouse.x,
                 (unsigned int)mouse.y,
                 (unsigned int)mouse.dir,
                 (unsigned int)map[mouse.y][mouse.x]);
    return false;
  }

  st = nvm_maze_save_map(&map[0][0], F413_SEARCH_STEP_CELL_COUNT);
  if (out_status != NULL)
  {
    *out_status = st;
  }
  if (st != HAL_OK)
  {
    trace_printf("[SEARCH-RUN] maze save: FAIL HAL=%d\r\n", (int)st);
    return false;
  }
  f413_search_step_post_goal_reset_baseline();
  trace_printf("[SEARCH-RUN] maze save: OK pos=(%u,%u,%u)\r\n",
               (unsigned int)mouse.x,
               (unsigned int)mouse.y,
               (unsigned int)mouse.dir);
  return true;
}

static bool f413_search_step_choose_next_relative(uint8_t x, uint8_t y, uint8_t dir, uint8_t* out_rel)
{
  static const uint8_t rel_priority[4] = {0U, 1U, 3U, 2U};
  static const int8_t dx[4] = {0, 1, 0, -1};
  static const int8_t dy[4] = {1, 0, -1, 0};
  static const uint8_t wall_mask[4] = {
      F413_SEARCH_STEP_MAZE_WALL_N,
      F413_SEARCH_STEP_MAZE_WALL_E,
      F413_SEARCH_STEP_MAZE_WALL_S,
      F413_SEARCH_STEP_MAZE_WALL_W
  };
  uint8_t i;
  uint16_t current_step;

  if ((out_rel == NULL) || (x >= MAZE_SIZE) || (y >= MAZE_SIZE) || (smap[y][x] == 0xFFFFU))
  {
    return false;
  }

  current_step = smap[y][x];
  for (i = 0U; i < 4U; i++)
  {
    uint8_t rel = rel_priority[i];
    uint8_t abs_dir = (uint8_t)((dir + rel) & 0x03U);
    int16_t nx = (int16_t)x + dx[abs_dir];
    int16_t ny = (int16_t)y + dy[abs_dir];
    uint8_t cell = (uint8_t)(map[y][x] & F413_SEARCH_STEP_MAZE_WALL_KNOWN_MASK);

    if ((cell & wall_mask[abs_dir]) != 0U)
    {
      continue;
    }
    if ((nx < 0) || (ny < 0) || (nx >= (int16_t)MAZE_SIZE) || (ny >= (int16_t)MAZE_SIZE))
    {
      continue;
    }
    if (smap[ny][nx] < current_step)
    {
      *out_rel = rel;
      return true;
    }
  }
  return false;
}

static const char* f413_search_step_relative_name(uint8_t rel)
{
  switch (rel)
  {
    case 0U: return "forward";
    case 1U: return "right";
    case 2U: return "back";
    case 3U: return "left";
    default: return "?";
  }
}

static bool f413_search_step_forward_destination(uint8_t x,
                                                 uint8_t y,
                                                 uint8_t dir,
                                                 uint8_t* out_x,
                                                 uint8_t* out_y)
{
  int16_t nx = (int16_t)x;
  int16_t ny = (int16_t)y;

  switch (dir & 0x03U)
  {
    case 0U: ny++; break;
    case 1U: nx++; break;
    case 2U: ny--; break;
    case 3U: nx--; break;
    default: break;
  }
  if ((nx < 0) || (ny < 0) || (nx >= (int16_t)MAZE_SIZE) || (ny >= (int16_t)MAZE_SIZE))
  {
    return false;
  }
  if (out_x != NULL)
  {
    *out_x = (uint8_t)nx;
  }
  if (out_y != NULL)
  {
    *out_y = (uint8_t)ny;
  }
  return true;
}

static bool f413_search_step_forward_is_known_straight(uint8_t x,
                                                       uint8_t y,
                                                       uint8_t dir,
                                                       uint8_t* out_next_after_forward)
{
  uint8_t nx;
  uint8_t ny;
  uint8_t next_rel = 0U;

  if (out_next_after_forward != NULL)
  {
    *out_next_after_forward = 0xFFU;
  }
  if (!f413_search_step_forward_destination(x, y, dir, &nx, &ny))
  {
    return false;
  }
  if (!visited[ny][nx])
  {
    return false;
  }
  if (!f413_search_step_choose_next_relative(nx, ny, dir, &next_rel))
  {
    return false;
  }
  if (out_next_after_forward != NULL)
  {
    *out_next_after_forward = next_rel;
  }
  return next_rel == 0U;
}

static void f413_search_step_advance_position(void)
{
  switch (mouse.dir & 0x03U)
  {
    case 0U:
      if (mouse.y < (MAZE_SIZE - 1U))
      {
        mouse.y++;
      }
      break;
    case 1U:
      if (mouse.x < (MAZE_SIZE - 1U))
      {
        mouse.x++;
      }
      break;
    case 2U:
      if (mouse.y > 0U)
      {
        mouse.y--;
      }
      break;
    case 3U:
      if (mouse.x > 0U)
      {
        mouse.x--;
      }
      break;
    default:
      break;
  }
}

static float f413_search_step_turn_target_deg(uint8_t rel)
{
  switch (rel)
  {
    case 1U:
      return -g_config.step_turn_deg;
    case 2U:
      return 2.0f * g_config.step_turn_deg;
    case 3U:
      return g_config.step_turn_deg;
    default:
      return 0.0f;
  }
}

static float f413_search_step_accel_positive(const SearchRunParams_t* params)
{
  if ((params != NULL) && (params->acceleration_straight > 0.0f))
  {
    return params->acceleration_straight;
  }
  return 1000.0f;
}

static float f413_search_step_accel_dash(const SearchRunParams_t* params)
{
  if ((params != NULL) &&
      (params->acceleration_straight_dash > 0.0f) &&
      (fabsf(params->acceleration_straight_dash - params->acceleration_straight) > 1e-3f))
  {
    return params->acceleration_straight_dash;
  }
  return f413_search_step_accel_positive(params);
}

static float f413_search_step_speed_after_accel(float speed_mm_s,
                                                float accel_mm_s2,
                                                float distance_mm)
{
  if ((accel_mm_s2 <= 0.0f) || (distance_mm <= 0.0f))
  {
    return speed_mm_s;
  }
  return sqrtf(fmaxf(0.0f,
                     (speed_mm_s * speed_mm_s) +
                         (2.0f * accel_mm_s2 * distance_mm)));
}

static f413_search_step_smooth_turn_t f413_search_step_build_smooth_turn(float angle_deg,
                                                                         float alpha_deg_s2)
{
  f413_search_step_smooth_turn_t profile = {0.0f, 0.0f, 0.0f, 0.0f};
  const float angle_abs = fabsf(angle_deg);
  float rounding_scale = TURN_OMEGA_PROFILE_ROUNDING_SCALE;

  if ((angle_abs <= 0.0f) || (alpha_deg_s2 <= 0.0f))
  {
    return profile;
  }
  if (rounding_scale < 0.1f)
  {
    rounding_scale = 0.1f;
  }

  profile.omega_peak_deg_s = sqrtf((2.0f * alpha_deg_s2 * angle_abs) / 3.0f);
  if ((NIGHTFALL_F413_PATH_OMEGA_CAP > 0.0f) &&
      (profile.omega_peak_deg_s > NIGHTFALL_F413_PATH_OMEGA_CAP))
  {
    profile.omega_peak_deg_s = NIGHTFALL_F413_PATH_OMEGA_CAP;
  }
  if (profile.omega_peak_deg_s <= 0.0f)
  {
    return profile;
  }

  profile.t_acc_s = (profile.omega_peak_deg_s / alpha_deg_s2) * rounding_scale;
  profile.t_cruise_s = (angle_abs / profile.omega_peak_deg_s) - profile.t_acc_s;
  if (profile.t_cruise_s < 0.0f)
  {
    profile.t_cruise_s = 0.0f;
    profile.omega_peak_deg_s = angle_abs / profile.t_acc_s;
    if ((NIGHTFALL_F413_PATH_OMEGA_CAP > 0.0f) &&
        (profile.omega_peak_deg_s > NIGHTFALL_F413_PATH_OMEGA_CAP))
    {
      profile.omega_peak_deg_s = NIGHTFALL_F413_PATH_OMEGA_CAP;
    }
    profile.t_cruise_s = (angle_abs / profile.omega_peak_deg_s) - profile.t_acc_s;
    if (profile.t_cruise_s < 0.0f)
    {
      profile.t_cruise_s = 0.0f;
    }
  }
  profile.t_total_s = (2.0f * profile.t_acc_s) + profile.t_cruise_s;
  return profile;
}

static f413_run_session_abort_reason_t f413_search_step_wait_ctrl_target(float target,
                                                                         bool is_angle,
                                                                         f413_run_session_guard_t* guard,
                                                                         uint16_t trace_flags)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  uint32_t deadline = f413_search_step_tick() + g_config.path_timeout_ms;

  while (1)
  {
    if (is_angle)
    {
      float current = f413_ctrl_get_angle();
      if (((target >= 0.0f) && (current >= target)) ||
          ((target < 0.0f) && (current <= target)))
      {
        break;
      }
    }
    else
    {
      float current = fabsf(f413_ctrl_get_distance());
      if (current >= fabsf(target))
      {
        break;
      }
    }

    if (f413_search_step_tick() >= deadline)
    {
      break;
    }

    f413_search_step_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if ((g_config.wall_control_apply_straight != NULL) &&
        !is_angle &&
        ((trace_flags & g_config.trace_motor_fwd_flag) != 0U))
    {
      g_config.wall_control_apply_straight();
    }
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
  }
  return F413_RUN_SESSION_ABORT_NONE;
}

static f413_run_session_abort_reason_t f413_search_step_drive_segment(float distance_mm,
                                                                      float target_velocity_mm_s,
                                                                      float* speed_now_mm_s,
                                                                      f413_run_session_guard_t* guard,
                                                                      uint16_t trace_flags)
{
  float target_distance;
  f413_run_session_abort_reason_t reason;

  if (speed_now_mm_s == NULL)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
  if (distance_mm <= 0.0f)
  {
    *speed_now_mm_s = target_velocity_mm_s;
    return F413_RUN_SESSION_ABORT_NONE;
  }

  target_distance = f413_ctrl_get_distance() + distance_mm;
  f413_search_step_prepare_straight_angle_control();
  f413_ctrl_set_velocity_profile(*speed_now_mm_s, target_velocity_mm_s, distance_mm);
  f413_ctrl_set_omega(0.0f);

  reason = f413_search_step_wait_ctrl_target(target_distance, false, guard, trace_flags);
  *speed_now_mm_s = target_velocity_mm_s;
  return reason;
}

static bool f413_search_step_front_wall_distance_mm(float* distance_mm)
{
  f413_wall_distance_snapshot_t distance;

  if ((distance_mm == NULL) ||
      !f413_wall_distance_read_snapshot(&distance) ||
      !f413_wall_distance_front_present(&distance))
  {
    return false;
  }

  *distance_mm = distance.front_sum_mm_unwarped;
  return isfinite(*distance_mm);
}

static bool f413_search_step_front_match_too_close(
    const f413_wall_distance_snapshot_t* distance);

static bool f413_search_step_front_wall_strong_present(void)
{
  f413_wall_sensor_snapshot_t wall;
  f413_wall_distance_snapshot_t distance;

  if (!f413_search_step_read_wall(&wall))
  {
    return false;
  }
  if (!f413_wall_distance_convert_snapshot(&wall, &distance))
  {
    return false;
  }
  if (!distance.front_valid)
  {
    return f413_search_step_front_match_too_close(&distance);
  }
  return ((float)distance.adc.fr_delta > ((float)WALL_BASE_FR * 1.5f)) &&
         ((float)distance.adc.fl_delta > ((float)WALL_BASE_FL * 1.5f));
}

static float f413_search_step_clampf(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static float f413_search_step_front_match_lpf_alpha(void)
{
  return f413_search_step_clampf(MATCH_POS_SENSOR_LPF_ALPHA, 0.0f, 1.0f);
}

static void f413_search_step_front_match_filter_reset(
    f413_search_step_front_match_filter_t* filter)
{
  if (filter != NULL)
  {
    filter->initialized = false;
    filter->fr_mm = 0.0f;
    filter->fl_mm = 0.0f;
    filter->front_sum_mm = 0.0f;
  }
}

static void f413_search_step_front_match_filter_update(
    f413_search_step_front_match_filter_t* filter,
    const f413_wall_distance_snapshot_t* distance)
{
  float alpha;

  if ((filter == NULL) || (distance == NULL))
  {
    return;
  }
  if (!filter->initialized)
  {
    filter->initialized = true;
    /* The measured profile LUT remains authoritative until NVM warp anchors are calibrated. */
    filter->fr_mm = distance->fr_mm_unwarped;
    filter->fl_mm = distance->fl_mm_unwarped;
    filter->front_sum_mm = distance->front_sum_mm_unwarped;
    return;
  }

  alpha = f413_search_step_front_match_lpf_alpha();
  filter->fr_mm += alpha * (distance->fr_mm_unwarped - filter->fr_mm);
  filter->fl_mm += alpha * (distance->fl_mm_unwarped - filter->fl_mm);
  filter->front_sum_mm += alpha * (distance->front_sum_mm_unwarped - filter->front_sum_mm);
}

static float f413_search_step_front_match_error_position(
    const f413_search_step_front_match_filter_t* filter)
{
  if ((filter == NULL) || !filter->initialized)
  {
    return 0.0f;
  }
  return filter->front_sum_mm - F_ALIGN_TARGET_MM;
}

static float f413_search_step_front_match_error_yaw(
    const f413_search_step_front_match_filter_t* filter)
{
  if ((filter == NULL) || !filter->initialized)
  {
    return 0.0f;
  }
  return filter->fr_mm - filter->fl_mm;
}

static uint16_t f413_search_step_front_match_motor_flags(float v_cmd,
                                                         float w_cmd);
static f413_run_session_abort_reason_t f413_search_step_front_match_backoff_too_close(
    f413_run_session_guard_t* guard,
    bool* control_running,
    bool wait_after_limit);

static uint16_t f413_search_step_front_match_state_elapsed(
    const f413_front_match_controller_t* controller)
{
  if (controller == NULL)
  {
    return 0U;
  }
  return (controller->phase == F413_FRONT_MATCH_PHASE_HOLD)
             ? controller->release_elapsed_ms
             : controller->phase_elapsed_ms;
}

static void f413_search_step_front_match_sync_angle_target(
    const f413_front_match_output_t* output)
{
  if ((output == NULL) || !output->phase_changed)
  {
    return;
  }

  if (output->phase == F413_FRONT_MATCH_PHASE_ALIGN_YAW)
  {
    f413_ctrl_clear_angle_target();
  }
  else if ((output->phase == F413_FRONT_MATCH_PHASE_SETTLE_YAW) ||
           (output->phase == F413_FRONT_MATCH_PHASE_SETTLE_FINAL) ||
           (output->phase == F413_FRONT_MATCH_PHASE_HOLD))
  {
    f413_ctrl_set_angle_target(f413_ctrl_get_angle());
  }
}

static f413_run_session_abort_reason_t f413_search_step_match_front_position(
    f413_run_session_guard_t* guard)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  f413_search_step_front_match_filter_t filter = {0};
  f413_front_match_controller_t controller;
  bool control_running = true;
  uint32_t count;

  if (!f413_search_step_front_wall_strong_present())
  {
    return F413_RUN_SESSION_ABORT_NONE;
  }

  f413_wall_runtime_control_clear();
  f413_ctrl_clear_angle_target();
  f413_front_match_init(&controller);

  for (count = 0U; count < MATCH_POS_MAX_DURATION_MS; count++)
  {
    f413_wall_sensor_snapshot_t wall;
    f413_wall_distance_snapshot_t distance;
    float e_pos_mm;
    float e_yaw_mm;
    f413_front_match_output_t output;

    if (!f413_search_step_read_wall(&wall))
    {
      return F413_RUN_SESSION_ABORT_WALL_FAULT;
    }
    if (!f413_wall_distance_convert_snapshot(&wall, &distance))
    {
      return F413_RUN_SESSION_ABORT_WALL_FAULT;
    }
    if (f413_search_step_front_match_too_close(&distance))
    {
      reason = f413_search_step_front_match_backoff_too_close(
          guard,
          &control_running,
          false);
      f413_search_step_front_match_filter_reset(&filter);
      f413_front_match_init(&controller);
      f413_ctrl_clear_angle_target();
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }
      continue;
    }
    if (!distance.front_valid ||
        ((float)distance.adc.fr_delta <= ((float)WALL_BASE_FR * 1.5f)) ||
        ((float)distance.adc.fl_delta <= ((float)WALL_BASE_FL * 1.5f)))
    {
      break;
    }

    f413_search_step_front_match_filter_update(&filter, &distance);
    e_pos_mm = f413_search_step_front_match_error_position(&filter);
    e_yaw_mm = f413_search_step_front_match_error_yaw(&filter);
    f413_front_match_step(&controller, e_pos_mm, e_yaw_mm, 1U, &output);
    f413_search_step_front_match_sync_angle_target(&output);
    if (output.complete)
    {
      break;
    }

    f413_ctrl_set_velocity(output.velocity_mm_s);
    f413_ctrl_set_omega(output.omega_deg_s);
    f413_search_step_set_mode_flags(
        f413_search_step_front_match_motor_flags(output.velocity_mm_s,
                                                 output.omega_deg_s));
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }
  }

  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  if ((reason == F413_RUN_SESSION_ABORT_NONE) &&
      (MATCH_POS_POST_COMPLETE_DELAY_MS > 0U))
  {
    f413_ctrl_set_angle_target(f413_ctrl_get_angle());
    f413_search_step_set_mode_flags((uint16_t)(g_config.trace_search_safe_flag |
                                               g_config.trace_motor_coast_flag));
    reason = f413_run_session_wait_with_auto_step_guarded(
        MATCH_POS_POST_COMPLETE_DELAY_MS,
        guard);
  }
  return reason;
}

static bool f413_search_step_front_match_wall_detected(
    const f413_wall_distance_snapshot_t* distance)
{
  return (distance != NULL) && distance->front_valid;
}

static bool f413_search_step_front_match_too_close(
    const f413_wall_distance_snapshot_t* distance)
{
  return (distance != NULL) &&
         ((distance->fr_mm_unwarped < F_ALIGN_TOO_CLOSE_MM) ||
          (distance->fl_mm_unwarped < F_ALIGN_TOO_CLOSE_MM));
}

static uint16_t f413_search_step_front_match_motor_flags(float v_cmd,
                                                         float w_cmd)
{
  uint16_t flags = g_config.trace_search_safe_flag;

  if ((fabsf(v_cmd) <= 0.01f) && (fabsf(w_cmd) <= 0.01f))
  {
    flags = (uint16_t)(flags | g_config.trace_motor_coast_flag);
  }
  else if (v_cmd < -0.01f)
  {
    flags = (uint16_t)(flags | g_config.trace_motor_rev_flag);
  }
  else
  {
    flags = (uint16_t)(flags | g_config.trace_motor_fwd_flag);
  }
  return flags;
}

static f413_run_session_abort_reason_t f413_search_step_front_match_stop_and_wait(
    f413_run_session_guard_t* guard,
    const char* reason_text,
    f413_front_match_phase_t pause_phase,
    bool* control_running)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  uint16_t resume_count = 0U;
  uint32_t last_print_ms = f413_search_step_tick();
  const uint16_t stop_flags = g_config.trace_search_safe_flag;

  f413_ctrl_stop();
  if (control_running != NULL)
  {
    *control_running = false;
  }
  f413_search_step_set_trace_period(MATCH_POS_TRACE_IDLE_PERIOD_MS);
  f413_search_step_set_mode_flags(stop_flags);
  trace_printf("[SEARCH-TEST] front-match pause(%s)\r\n", reason_text);

  while (resume_count < MATCH_POS_RECOVERY_VALID_MS)
  {
    f413_wall_sensor_snapshot_t wall;
    f413_wall_distance_snapshot_t distance;
    uint32_t now_ms;

    if (!f413_search_step_read_wall(&wall))
    {
      return F413_RUN_SESSION_ABORT_WALL_FAULT;
    }
    if (!f413_wall_distance_convert_snapshot(&wall, &distance))
    {
      return F413_RUN_SESSION_ABORT_WALL_FAULT;
    }
    f413_search_step_set_front_match_trace(
        true,
        pause_phase,
        distance.fr_mm_unwarped,
        distance.fl_mm_unwarped,
        distance.front_sum_mm_unwarped - F_ALIGN_TARGET_MM,
        distance.fr_mm_unwarped - distance.fl_mm_unwarped,
        resume_count);
    if (f413_search_step_front_match_wall_detected(&distance) &&
        !f413_search_step_front_match_too_close(&distance))
    {
      resume_count++;
    }
    else
    {
      resume_count = 0U;
    }

    now_ms = f413_search_step_tick();
    if ((now_ms - last_print_ms) >= 500U)
    {
      trace_printf("[SEARCH-TEST] front-match waiting FR=%ld/%.2fmm FL=%ld/%.2fmm valid=%u\r\n",
                   (long)distance.adc.fr_delta,
                   (double)distance.fr_mm_unwarped,
                   (long)distance.adc.fl_delta,
                   (double)distance.fl_mm_unwarped,
                   (unsigned int)distance.front_valid);
      last_print_ms = now_ms;
    }

    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
  }

  f413_ctrl_start();
  if (control_running != NULL)
  {
    *control_running = true;
  }
  f413_ctrl_clear_angle_target();
  trace_printf("[SEARCH-TEST] front-match resume\r\n");
  return F413_RUN_SESSION_ABORT_NONE;
}

static f413_run_session_abort_reason_t f413_search_step_front_match_backoff_too_close(
    f413_run_session_guard_t* guard,
    bool* control_running,
    bool wait_after_limit)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  uint16_t recovery_count = 0U;
  uint32_t start_ms;
  float start_distance;

  if ((control_running == NULL) || !*control_running)
  {
    f413_search_step_set_mode_flags(g_config.trace_search_safe_flag);
    f413_ctrl_start();
    if (control_running != NULL)
    {
      *control_running = true;
    }
  }

  start_ms = f413_search_step_tick();
  /* Outside the near LUT edge, only the safe retreat direction is authoritative. */
  f413_ctrl_clear_angle_target();
  f413_ctrl_set_omega(0.0f);
  f413_ctrl_set_velocity(-fabsf(MATCH_POS_TOO_CLOSE_RECOVERY_VEL_MM_S));
  f413_search_step_set_trace_period(MATCH_POS_TRACE_PERIOD_MS);
  f413_search_step_set_mode_flags(
      f413_search_step_front_match_motor_flags(
          -fabsf(MATCH_POS_TOO_CLOSE_RECOVERY_VEL_MM_S),
          0.0f));
  start_distance = f413_ctrl_get_distance();
  trace_printf("[SEARCH-TEST] front-match backoff-too-close\r\n");

  while (reason == F413_RUN_SESSION_ABORT_NONE)
  {
    f413_wall_sensor_snapshot_t wall;
    f413_wall_distance_snapshot_t distance;
    const uint32_t elapsed_ms = f413_search_step_tick() - start_ms;
    const float backed_off_mm = start_distance - f413_ctrl_get_distance();
    const uint16_t trace_elapsed_ms =
        (elapsed_ms > UINT16_MAX) ? UINT16_MAX : (uint16_t)elapsed_ms;

    if (!f413_search_step_read_wall(&wall) ||
        !f413_wall_distance_convert_snapshot(&wall, &distance))
    {
      reason = F413_RUN_SESSION_ABORT_WALL_FAULT;
      break;
    }

    f413_search_step_set_front_match_trace(
        true,
        F413_FRONT_MATCH_PHASE_BACKOFF_TOO_CLOSE,
        distance.fr_mm_unwarped,
        distance.fl_mm_unwarped,
        distance.front_sum_mm_unwarped - F_ALIGN_TARGET_MM,
        distance.fr_mm_unwarped - distance.fl_mm_unwarped,
        trace_elapsed_ms);

    if (f413_search_step_front_match_wall_detected(&distance) &&
        !f413_search_step_front_match_too_close(&distance))
    {
      recovery_count++;
    }
    else
    {
      recovery_count = 0U;
    }

    if (recovery_count >= MATCH_POS_RECOVERY_VALID_MS)
    {
      trace_printf("[SEARCH-TEST] front-match backoff complete dist=%.2fmm time=%lums\r\n",
                   (double)backed_off_mm,
                   (unsigned long)elapsed_ms);
      break;
    }

    if ((backed_off_mm >= MATCH_POS_TOO_CLOSE_RECOVERY_MAX_MM) ||
        (elapsed_ms >= MATCH_POS_TOO_CLOSE_RECOVERY_MAX_MS))
    {
      f413_ctrl_set_velocity(0.0f);
      f413_ctrl_set_omega(0.0f);
      trace_printf("[SEARCH-TEST] front-match backoff limit dist=%.2fmm time=%lums\r\n",
                   (double)backed_off_mm,
                   (unsigned long)elapsed_ms);
      if (wait_after_limit)
      {
        return f413_search_step_front_match_stop_and_wait(
            guard,
            "too-close-backoff-limit",
            F413_FRONT_MATCH_PHASE_PAUSED_TOO_CLOSE,
            control_running);
      }
      return F413_RUN_SESSION_ABORT_WALL_FAULT;
    }

    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
  }

  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  return reason;
}

static f413_run_session_abort_reason_t f413_search_step_front_match_continuous(
    f413_run_session_guard_t* guard)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  f413_search_step_front_match_filter_t filter = {0};
  f413_front_match_controller_t controller;
  bool control_running = true;

  f413_wall_runtime_control_clear();
  f413_ctrl_clear_angle_target();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  f413_front_match_init(&controller);

  trace_printf("[SEARCH-TEST] front-match continuous: stop switch exits, reset also safe\r\n");

  while (reason == F413_RUN_SESSION_ABORT_NONE)
  {
    f413_wall_sensor_snapshot_t wall;
    f413_wall_distance_snapshot_t distance;
    float e_pos_mm;
    float e_yaw_mm;
    f413_front_match_output_t output;

    if (!f413_search_step_read_wall(&wall))
    {
      return F413_RUN_SESSION_ABORT_WALL_FAULT;
    }
    if (!f413_wall_distance_convert_snapshot(&wall, &distance))
    {
      return F413_RUN_SESSION_ABORT_WALL_FAULT;
    }
    if (f413_search_step_front_match_too_close(&distance))
    {
      reason = f413_search_step_front_match_backoff_too_close(
          guard,
          &control_running,
          true);
      f413_search_step_front_match_filter_reset(&filter);
      f413_front_match_init(&controller);
      f413_ctrl_clear_angle_target();
      f413_search_step_set_trace_period(MATCH_POS_TRACE_PERIOD_MS);
      continue;
    }
    if (!f413_search_step_front_match_wall_detected(&distance))
    {
      reason = f413_search_step_front_match_stop_and_wait(
          guard,
          "front-wall-lost",
          F413_FRONT_MATCH_PHASE_PAUSED_WALL_LOST,
          &control_running);
      f413_search_step_front_match_filter_reset(&filter);
      f413_front_match_init(&controller);
      f413_ctrl_clear_angle_target();
      f413_search_step_set_trace_period(MATCH_POS_TRACE_PERIOD_MS);
      continue;
    }

    f413_search_step_front_match_filter_update(&filter, &distance);
    e_pos_mm = f413_search_step_front_match_error_position(&filter);
    e_yaw_mm = f413_search_step_front_match_error_yaw(&filter);
    f413_front_match_step(&controller, e_pos_mm, e_yaw_mm, 1U, &output);
    if (!control_running && !output.holding)
    {
      f413_ctrl_start();
      control_running = true;
      if (output.phase == F413_FRONT_MATCH_PHASE_ALIGN_YAW)
      {
        f413_ctrl_clear_angle_target();
      }
      else
      {
        f413_ctrl_set_angle_target(f413_ctrl_get_angle());
      }
    }
    else if (control_running)
    {
      f413_search_step_front_match_sync_angle_target(&output);
    }
    if (output.phase_changed)
    {
      trace_printf("[SEARCH-TEST] front-match phase=%s pos=%.2fmm yaw=%.2fmm\r\n",
                   f413_front_match_phase_name(output.phase),
                   (double)e_pos_mm,
                   (double)e_yaw_mm);
    }

    if (output.holding)
    {
      if (control_running)
      {
        f413_ctrl_stop();
        control_running = false;
      }
      f413_search_step_set_mode_flags(g_config.trace_search_safe_flag);
    }
    else
    {
      f413_ctrl_set_velocity(output.velocity_mm_s);
      f413_ctrl_set_omega(output.omega_deg_s);
      f413_search_step_set_mode_flags(
          f413_search_step_front_match_motor_flags(output.velocity_mm_s,
                                                   output.omega_deg_s));
    }
    f413_search_step_set_trace_period(output.holding
                                          ? MATCH_POS_TRACE_IDLE_PERIOD_MS
                                          : MATCH_POS_TRACE_PERIOD_MS);
    f413_search_step_set_front_match_trace(
        true,
        output.phase,
        filter.fr_mm,
        filter.fl_mm,
        e_pos_mm,
        e_yaw_mm,
        f413_search_step_front_match_state_elapsed(&controller));
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
  }

  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  return reason;
}

static void f413_search_step_apply_straight_runtime(uint16_t trace_flags)
{
  if ((g_config.wall_control_apply_straight != NULL) &&
      ((trace_flags & g_config.trace_motor_fwd_flag) != 0U))
  {
    g_config.wall_control_apply_straight();
  }
}

static f413_run_session_abort_reason_t f413_search_step_turn_in_place_to_angle(
    float target_angle_deg,
    f413_run_session_guard_t* guard);

static f413_run_session_abort_reason_t f413_search_step_drive_front_wall_entry_segment(
    float distance_mm,
    float target_velocity_mm_s,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  float front_distance_mm;
  float front_target_mm;
  float target_distance;
  bool front_reached = false;

  if (speed_now_mm_s == NULL)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
  /* At a cell entrance the nose-to-wall distance is the centered distance
     plus half a cell. The turn should begin at the nominal offset endpoint. */
  front_target_mm = F_ALIGN_TARGET_MM + (float)DIST_HALF_SEC - distance_mm;
  if ((distance_mm <= 0.0f) || !f413_run_features_front_wall_correction_enabled() ||
      f413_run_features_test_mode_run() || (front_target_mm <= 0.0f) ||
      !f413_search_step_front_wall_distance_mm(&front_distance_mm))
  {
    return f413_search_step_drive_segment(distance_mm, target_velocity_mm_s,
                                          speed_now_mm_s, guard, trace_flags);
  }

  target_distance = f413_ctrl_get_distance() + distance_mm;
  f413_search_step_prepare_straight_angle_control();
  f413_ctrl_set_velocity_profile(*speed_now_mm_s, target_velocity_mm_s, distance_mm);
  f413_ctrl_set_omega(0.0f);

  while (1)
  {
    if (f413_search_step_front_wall_distance_mm(&front_distance_mm) &&
        (front_distance_mm <= front_target_mm))
    {
      front_reached = true;
      break;
    }
    if (fabsf(f413_ctrl_get_distance()) >= fabsf(target_distance))
    {
      break;
    }
    f413_search_step_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    f413_search_step_apply_straight_runtime(trace_flags);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      *speed_now_mm_s = target_velocity_mm_s;
      return reason;
    }
  }

  if (!front_reached && (WALL_END_EXTEND_MAX_MM > 0.0F))
  {
    const float extend_target = f413_ctrl_get_distance() + WALL_END_EXTEND_MAX_MM;
    f413_ctrl_set_velocity(target_velocity_mm_s);
    while (fabsf(f413_ctrl_get_distance()) < fabsf(extend_target))
    {
      if (!f413_search_step_front_wall_distance_mm(&front_distance_mm))
      {
        break;
      }
      if (front_distance_mm <= front_target_mm)
      {
        break;
      }
      f413_search_step_set_mode_flags(trace_flags);
      reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
      f413_search_step_apply_straight_runtime(trace_flags);
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }
    }
  }

  *speed_now_mm_s = target_velocity_mm_s;
  return reason;
}

static f413_run_session_abort_reason_t f413_search_step_drive_wallend_segment(
    float distance_mm,
    float target_velocity_mm_s,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags,
    bool* wall_end_found)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  float target_distance;

  if (wall_end_found != NULL)
  {
    *wall_end_found = false;
  }
  if (speed_now_mm_s == NULL)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
  if ((distance_mm <= 0.0f) || !f413_run_features_wall_end_correction_enabled())
  {
    return f413_search_step_drive_segment(distance_mm, target_velocity_mm_s,
                                          speed_now_mm_s, guard, trace_flags);
  }

  target_distance = f413_ctrl_get_distance() + distance_mm;
  f413_wall_runtime_end_clear();
  f413_search_step_prepare_straight_angle_control();
  f413_ctrl_set_velocity_profile(*speed_now_mm_s, target_velocity_mm_s, distance_mm);
  f413_ctrl_set_omega(0.0f);

  while (1)
  {
    if (f413_wall_runtime_poll_wall_end(true))
    {
      if (wall_end_found != NULL)
      {
        *wall_end_found = true;
      }
      break;
    }
    if (fabsf(f413_ctrl_get_distance()) >= fabsf(target_distance))
    {
      break;
    }
    f413_search_step_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }
  }

  *speed_now_mm_s = target_velocity_mm_s;
  return reason;
}

static f413_run_session_abort_reason_t f413_search_step_drive_accel_distance(
    float distance_mm,
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  const float speed_out =
      f413_search_step_speed_after_accel((speed_now_mm_s != NULL) ? *speed_now_mm_s : 0.0f,
                                         f413_search_step_accel_positive(params),
                                         distance_mm);
  return f413_search_step_drive_segment(distance_mm, speed_out, speed_now_mm_s,
                                        guard, trace_flags);
}

static f413_run_session_abort_reason_t f413_search_step_drive_accel_distance_with_accel(
    float distance_mm,
    float accel_mm_s2,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  const float speed_out =
      f413_search_step_speed_after_accel((speed_now_mm_s != NULL) ? *speed_now_mm_s : 0.0f,
                                         accel_mm_s2,
                                         distance_mm);
  return f413_search_step_drive_segment(distance_mm, speed_out, speed_now_mm_s,
                                        guard, trace_flags);
}

static f413_run_session_abort_reason_t f413_search_step_drive_decel_distance_with_accel(
    float distance_mm,
    float accel_mm_s2,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags)
{
  float speed_in = (speed_now_mm_s != NULL) ? *speed_now_mm_s : 0.0f;
  float speed_sq = (speed_in * speed_in) - (2.0f * accel_mm_s2 * distance_mm);
  float speed_out = (speed_sq > 0.0f) ? sqrtf(speed_sq) : 0.0f;

  return f413_search_step_drive_segment(distance_mm, speed_out, speed_now_mm_s,
                                        guard, trace_flags);
}

static f413_run_session_abort_reason_t f413_search_step_read_and_write_current_wall(
    bool force_start_front_open)
{
  f413_wall_sensor_snapshot_t wall;

  if (!f413_search_step_read_wall(&wall))
  {
    trace_printf("[SEARCH-RUN] FAIL(read wall snapshot)\r\n");
    return F413_RUN_SESSION_ABORT_WALL_FAULT;
  }
  wall_info = f413_search_step_wall_info_from_snapshot(&wall);
  g_search_wall_read_snapshot = wall;
  g_search_wall_read_valid = true;
  if (force_start_front_open)
  {
    wall_info &= (uint16_t)~0x88U;
  }
  f413_search_step_write_map_cell((uint8_t)mouse.x, (uint8_t)mouse.y,
                                  (uint8_t)mouse.dir, wall_info);
  visited[mouse.y][mouse.x] = true;
  f413_search_step_post_goal_track_around_mouse();
  return F413_RUN_SESSION_ABORT_NONE;
}

static f413_run_session_abort_reason_t f413_search_step_run_entry_section(
    uint8_t op_case,
    uint8_t target,
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard)
{
  f413_run_session_abort_reason_t reason;
  const uint16_t trace_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                         g_config.trace_motor_fwd_flag);

  f413_search_step_set_action_context(op_case,
                                      (uint8_t)mouse.x,
                                      (uint8_t)mouse.y,
                                      (uint8_t)mouse.dir,
                                      0U);

  reason = f413_search_step_drive_accel_distance((float)DIST_FIRST_SEC, params,
                                                 speed_now_mm_s, guard,
                                                 trace_flags);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }
  reason = f413_search_step_drive_accel_distance((float)DIST_HALF_SEC, params,
                                                 speed_now_mm_s, guard,
                                                 trace_flags);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  f413_search_step_advance_position();
  reason = f413_search_step_read_and_write_current_wall(false);
  if (reason == F413_RUN_SESSION_ABORT_NONE)
  {
    f413_search_step_note_goal_visit(target);
  }
  return reason;
}

static f413_run_session_abort_reason_t f413_search_step_run_forward_wall_align(
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    uint16_t trace_flags,
    bool* aligned)
{
  f413_wall_sensor_snapshot_t wall;
  f413_run_session_abort_reason_t reason;
  bool align_right = false;
  bool align_left = false;
  float base_target_angle;
  float side_angle;
  float side_target_angle;
  float return_target_angle;

  if (aligned != NULL)
  {
    *aligned = false;
  }
  if ((params == NULL) || (speed_now_mm_s == NULL) ||
      (params->wall_align_enable == 0U) ||
      (fabsf(f413_wall_runtime_latest_error()) <= (float)WALL_ALIGN_ERR_THR))
  {
    return F413_RUN_SESSION_ABORT_NONE;
  }
  if (!f413_search_step_read_wall(&wall))
  {
    return F413_RUN_SESSION_ABORT_WALL_FAULT;
  }

  align_right = (wall.r_delta > (int32_t)((float)WALL_BASE_R * 1.3f));
  align_left = !align_right && (wall.l_delta > (int32_t)((float)WALL_BASE_L * 1.3f));
  if (!align_right && !align_left)
  {
    return F413_RUN_SESSION_ABORT_NONE;
  }

  reason = f413_search_step_drive_segment((float)DIST_HALF_SEC, 0.0f,
                                          speed_now_mm_s, guard, trace_flags);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  base_target_angle = f413_run_features_angle_accum_mode() ? f413_ctrl_get_target_angle() : 0.0f;
  side_angle = align_right ? -fabsf(g_config.step_turn_deg) : fabsf(g_config.step_turn_deg);
  side_target_angle = f413_run_features_angle_accum_mode() ? (base_target_angle + side_angle)
                                                           : side_angle;
  return_target_angle = f413_run_features_angle_accum_mode() ? base_target_angle
                                                             : -side_angle;

  reason = f413_search_step_turn_in_place_to_angle(side_target_angle, guard);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }
  reason = f413_search_step_match_front_position(guard);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }
  reason = f413_search_step_turn_in_place_to_angle(return_target_angle, guard);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  reason = f413_search_step_drive_accel_distance((float)DIST_HALF_SEC, params,
                                                 speed_now_mm_s, guard,
                                                 trace_flags);
  if ((reason == F413_RUN_SESSION_ABORT_NONE) && (aligned != NULL))
  {
    *aligned = true;
  }
  return reason;
}

static f413_run_session_abort_reason_t f413_search_step_run_forward_section(
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    bool* acceled,
    bool known_straight,
    bool next_is_turn90)
{
  f413_run_session_abort_reason_t reason;
  bool wall_end_found = false;
  bool wall_aligned = false;
  const uint16_t trace_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                         g_config.trace_motor_fwd_flag);

  if ((acceled != NULL) && known_straight && !*acceled)
  {
    reason = f413_search_step_drive_accel_distance_with_accel(
        (float)(DIST_HALF_SEC * 2),
        f413_search_step_accel_dash(params),
        speed_now_mm_s,
        guard,
        trace_flags);
    if (reason == F413_RUN_SESSION_ABORT_NONE)
    {
      *acceled = true;
    }
    return reason;
  }

  if ((acceled != NULL) && !known_straight && *acceled)
  {
    if (next_is_turn90)
    {
      reason = f413_search_step_drive_decel_distance_with_accel(
          (float)DIST_HALF_SEC,
          f413_search_step_accel_dash(params),
          speed_now_mm_s,
          guard,
          trace_flags);
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        return reason;
      }
      reason = f413_search_step_drive_wallend_segment((float)DIST_HALF_SEC,
                                                      (speed_now_mm_s != NULL) ? *speed_now_mm_s : 0.0f,
                                                      speed_now_mm_s,
                                                      guard,
                                                      trace_flags,
                                                      &wall_end_found);
      if (reason != F413_RUN_SESSION_ABORT_NONE)
      {
        return reason;
      }
      if ((params != NULL) && wall_end_found && (params->dist_wall_end > 0.0f))
      {
        reason = f413_search_step_drive_segment(params->dist_wall_end,
                                                (speed_now_mm_s != NULL) ? *speed_now_mm_s : 0.0f,
                                                speed_now_mm_s,
                                                guard,
                                                trace_flags);
      }
    }
    else
    {
      reason = f413_search_step_drive_decel_distance_with_accel(
          (float)(DIST_HALF_SEC * 2),
          f413_search_step_accel_dash(params),
          speed_now_mm_s,
          guard,
          trace_flags);
    }
    if (reason == F413_RUN_SESSION_ABORT_NONE)
    {
      *acceled = false;
    }
    return reason;
  }

  if ((acceled == NULL) || !*acceled)
  {
    reason = f413_search_step_run_forward_wall_align(params, speed_now_mm_s,
                                                     guard, trace_flags,
                                                     &wall_aligned);
    if ((reason != F413_RUN_SESSION_ABORT_NONE) || wall_aligned)
    {
      return reason;
    }
  }

  reason = f413_search_step_drive_wallend_segment(
      (float)(DIST_HALF_SEC * 2),
      (speed_now_mm_s != NULL) ? *speed_now_mm_s : 0.0f,
      speed_now_mm_s,
      guard,
      trace_flags,
      &wall_end_found);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }
  if ((params != NULL) && wall_end_found && (params->dist_wall_end > 0.0f))
  {
    return f413_search_step_drive_segment(params->dist_wall_end,
                                          (speed_now_mm_s != NULL) ? *speed_now_mm_s : 0.0f,
                                          speed_now_mm_s,
                                          guard,
                                          trace_flags);
  }
  return F413_RUN_SESSION_ABORT_NONE;
}

static f413_run_session_abort_reason_t f413_search_step_run_smooth_turn(
    uint8_t next_rel,
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard)
{
  const uint16_t straight_trace_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                                  g_config.trace_motor_fwd_flag);
  const uint16_t turn_trace_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                              g_config.trace_motor_rev_flag);
  f413_search_step_smooth_turn_t profile;
  f413_run_session_abort_reason_t reason;
  uint32_t start_ms;
  float signed_angle;
  float angle_90;
  float entry_speed;
  int8_t turn_sign;

  if ((params == NULL) || (speed_now_mm_s == NULL))
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  angle_90 = f413_run_features_angle_accum_mode() ? 90.0f : params->angle_turn_90;
  signed_angle = (next_rel == 1U) ? -fabsf(angle_90) : fabsf(angle_90);
  if (fabsf(signed_angle) <= 0.0f)
  {
    signed_angle = (next_rel == 1U) ? -90.0f : 90.0f;
  }
  profile = f413_search_step_build_smooth_turn(signed_angle, params->alpha_turn90);
  if (profile.t_total_s <= 0.0f)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  entry_speed = *speed_now_mm_s;
  reason = f413_search_step_drive_front_wall_entry_segment(params->dist_offset_in,
                                                           params->velocity_turn90,
                                                           speed_now_mm_s,
                                                           guard,
                                                           straight_trace_flags);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  turn_sign = (signed_angle < 0.0f) ? -1 : 1;
  f413_search_step_prepare_turn_angle_control();
  f413_ctrl_set_velocity(params->velocity_turn90);
  f413_ctrl_start_omega_profile((float)turn_sign * profile.omega_peak_deg_s,
                                profile.t_acc_s,
                                profile.t_cruise_s);
  start_ms = f413_search_step_tick();
  while (1)
  {
    const float t_s = (float)(f413_search_step_tick() - start_ms) * 0.001f;
    if (t_s >= profile.t_total_s)
    {
      break;
    }
    f413_search_step_set_mode_flags(turn_trace_flags);
    f413_ctrl_set_velocity(params->velocity_turn90);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      f413_ctrl_stop_omega_profile();
      return reason;
    }
  }
  f413_ctrl_stop_omega_profile();
  *speed_now_mm_s = params->velocity_turn90;

  return f413_search_step_drive_segment(params->dist_offset_out,
                                        entry_speed,
                                        speed_now_mm_s,
                                        guard,
                                        straight_trace_flags);
}

static f413_run_session_abort_reason_t f413_search_step_run_spot_spin_profile(
    float target_angle_deg,
    float alpha_deg_s2,
    f413_run_session_guard_t* guard)
{
  const uint16_t turn_trace_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                              g_config.trace_motor_rev_flag);
  f413_search_step_smooth_turn_t profile;
  f413_run_session_abort_reason_t reason;
  uint32_t start_ms;
  float start_target_angle_deg;
  float turn_angle_deg;
  int8_t turn_sign;

  f413_search_step_prepare_turn_angle_control();
  start_target_angle_deg = f413_ctrl_get_target_angle();
  turn_angle_deg = target_angle_deg - start_target_angle_deg;
  if (fabsf(turn_angle_deg) <= 0.01f)
  {
    f413_ctrl_set_angle_target(target_angle_deg);
    return F413_RUN_SESSION_ABORT_NONE;
  }

  profile = f413_search_step_build_smooth_turn(turn_angle_deg, alpha_deg_s2);
  if (profile.t_total_s <= 0.0f)
  {
    return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  turn_sign = (turn_angle_deg < 0.0f) ? -1 : 1;
  trace_printf("[SEARCH] spin target=%.0fdeg delta=%.0fdeg alpha=%.0f omega_peak=%.0fdps total=%.2fs\r\n",
               (double)target_angle_deg,
               (double)turn_angle_deg,
               (double)alpha_deg_s2,
               (double)((float)turn_sign * profile.omega_peak_deg_s),
               (double)profile.t_total_s);

  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  f413_ctrl_start_omega_profile((float)turn_sign * profile.omega_peak_deg_s,
                                profile.t_acc_s,
                                profile.t_cruise_s);

  start_ms = f413_search_step_tick();
  while (1)
  {
    const float t_s = (float)(f413_search_step_tick() - start_ms) * 0.001f;
    if (t_s >= profile.t_total_s)
    {
      break;
    }
    f413_search_step_set_mode_flags(turn_trace_flags);
    f413_ctrl_set_velocity(0.0f);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      f413_ctrl_stop_omega_profile();
      f413_ctrl_clear_angle_target();
      f413_ctrl_set_velocity(0.0f);
      f413_ctrl_set_omega(0.0f);
      return reason;
    }
  }

  f413_ctrl_stop_omega_profile();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  f413_ctrl_set_angle_target(target_angle_deg);

  reason = f413_search_step_wait_ctrl_target(target_angle_deg,
                                             true,
                                             guard,
                                             turn_trace_flags);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    f413_ctrl_clear_angle_target();
  }
  return reason;
}

static f413_run_session_abort_reason_t f413_search_step_drive_wait(
    f413_run_session_guard_t* guard)
{
  if (F413_SEARCH_STEP_DRIVE_WAIT_MS == 0U)
  {
    return F413_RUN_SESSION_ABORT_NONE;
  }

  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  f413_search_step_set_mode_flags((uint16_t)(g_config.trace_search_safe_flag |
                                             g_config.trace_motor_coast_flag));
  return f413_run_session_wait_with_auto_step_guarded(F413_SEARCH_STEP_DRIVE_WAIT_MS,
                                                      guard);
}

static f413_run_session_abort_reason_t f413_search_step_reverse_distance(
    float distance_mm,
    f413_run_session_guard_t* guard)
{
  f413_run_session_abort_reason_t reason = F413_RUN_SESSION_ABORT_NONE;
  uint32_t deadline;
  const uint16_t trace_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                         g_config.trace_motor_rev_flag);

  if (distance_mm <= 0.0f)
  {
    return F413_RUN_SESSION_ABORT_NONE;
  }

  f413_search_step_prepare_straight_angle_control();
  f413_ctrl_reset_distance();
  f413_ctrl_set_velocity(F413_SEARCH_STEP_REVERSE_VELOCITY_MM_S);
  f413_ctrl_set_omega(0.0f);
  deadline = f413_search_step_tick() + g_config.path_timeout_ms;

  while (fabsf(f413_ctrl_get_distance()) < distance_mm)
  {
    if (f413_search_step_tick() >= deadline)
    {
      break;
    }
    f413_search_step_set_mode_flags(trace_flags);
    reason = f413_run_session_wait_with_auto_step_guarded(1U, guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      break;
    }
  }

  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  f413_ctrl_reset_distance();
  return reason;
}

static f413_run_session_abort_reason_t f413_search_step_turn_in_place_to_angle(
    float target_angle_deg,
    f413_run_session_guard_t* guard)
{
  const f413_run_session_abort_reason_t reason =
      f413_search_step_run_spot_spin_profile(target_angle_deg,
                                             (float)ALPHA_ROTATE_90,
                                             guard);
  f413_ctrl_clear_angle_target();
  return reason;
}

static f413_run_session_abort_reason_t f413_search_step_run_back_turn(
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard)
{
  f413_run_session_abort_reason_t reason;
  float target_angle_deg;
  bool waited_after_stop = false;
  const uint16_t fwd_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                       g_config.trace_motor_fwd_flag);

  reason = f413_search_step_drive_segment((float)DIST_HALF_SEC, 0.0f,
                                          speed_now_mm_s, guard, fwd_flags);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  if (g_post_goal_active && g_post_goal_save_pending)
  {
    HAL_StatusTypeDef save_status;

    reason = f413_search_step_drive_wait(guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
    waited_after_stop = true;
    if (!f413_search_step_try_save_map_safely(&save_status))
    {
      g_search_no_path_exit = true;
      return F413_RUN_SESSION_ABORT_NONE;
    }
  }

  if (!waited_after_stop)
  {
    reason = f413_search_step_drive_wait(guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
  }

  if ((params != NULL) &&
      (params->wall_align_enable != 0U) &&
      f413_search_step_front_wall_strong_present())
  {
    const bool right_wall = ((wall_info & 0x44U) != 0U);
    const bool left_wall = ((wall_info & 0x11U) != 0U);
    const float right90 = -fabsf(g_config.step_turn_deg);
    const float left90 = fabsf(g_config.step_turn_deg);

    reason = f413_search_step_match_front_position(guard);
    if (reason != F413_RUN_SESSION_ABORT_NONE)
    {
      return reason;
    }
    f413_ctrl_reset_angle();

    if (right_wall)
    {
      reason = f413_search_step_turn_in_place_to_angle(right90, guard);
      if (reason == F413_RUN_SESSION_ABORT_NONE)
      {
        reason = f413_search_step_match_front_position(guard);
      }
      if (reason == F413_RUN_SESSION_ABORT_NONE)
      {
        f413_ctrl_reset_angle();
        reason = f413_search_step_turn_in_place_to_angle(right90, guard);
      }
    }
    else if (left_wall)
    {
      reason = f413_search_step_turn_in_place_to_angle(left90, guard);
      if (reason == F413_RUN_SESSION_ABORT_NONE)
      {
        reason = f413_search_step_match_front_position(guard);
      }
      if (reason == F413_RUN_SESSION_ABORT_NONE)
      {
        f413_ctrl_reset_angle();
        reason = f413_search_step_turn_in_place_to_angle(left90, guard);
      }
    }
    else
    {
      reason = f413_search_step_turn_in_place_to_angle(2.0f * g_config.step_turn_deg, guard);
    }
  }
  else
  {
    target_angle_deg = f413_run_features_angle_accum_mode()
                           ? (f413_ctrl_get_target_angle() + (2.0f * g_config.step_turn_deg))
                           : (2.0f * g_config.step_turn_deg);
    reason = f413_search_step_turn_in_place_to_angle(target_angle_deg, guard);
  }
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  reason = f413_search_step_drive_wait(guard);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  return f413_search_step_drive_accel_distance((float)DIST_HALF_SEC, params,
                                               speed_now_mm_s, guard,
                                               fwd_flags);
}

static f413_run_session_abort_reason_t f413_search_step_run_search_motion(
    uint8_t next_rel,
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    bool* acceled,
    bool known_straight,
    bool next_is_turn90)
{
  switch (next_rel)
  {
    case 0U:
      return f413_search_step_run_forward_section(params, speed_now_mm_s, guard,
                                                 acceled, known_straight,
                                                 next_is_turn90);
    case 1U:
    case 3U:
      if (acceled != NULL)
      {
        *acceled = false;
      }
      return f413_search_step_run_smooth_turn(next_rel, params, speed_now_mm_s, guard);
    case 2U:
      if (acceled != NULL)
      {
        *acceled = false;
      }
      return f413_search_step_run_back_turn(params, speed_now_mm_s, guard);
    default:
      return F413_RUN_SESSION_ABORT_IMU_FAULT;
  }
}

static f413_run_session_abort_reason_t f413_search_step_run_final_stop(
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard)
{
  if ((speed_now_mm_s == NULL) || (fabsf(*speed_now_mm_s) <= 1.0f))
  {
    return F413_RUN_SESSION_ABORT_NONE;
  }
  return f413_search_step_drive_segment((float)DIST_HALF_SEC, 0.0f,
                                        speed_now_mm_s,
                                        guard,
                                        (uint16_t)(g_config.trace_search_safe_flag |
                                                   g_config.trace_motor_fwd_flag));
}

static f413_run_session_abort_reason_t f413_search_step_wait_stop_tail(
    f413_run_session_guard_t* guard)
{
  f413_ctrl_stop_omega_profile();
  f413_ctrl_clear_angle_target();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  if (g_config.path_coast_ms == 0U)
  {
    return F413_RUN_SESSION_ABORT_NONE;
  }

  f413_search_step_set_mode_flags((uint16_t)(g_config.trace_search_safe_flag |
                                             g_config.trace_motor_coast_flag));
  return f413_run_session_wait_with_auto_step_guarded(g_config.path_coast_ms,
                                                      guard);
}

static f413_run_session_abort_reason_t f413_search_step_run_phase_restart_entry(
    uint8_t op_case,
    uint16_t* action_count,
    uint8_t phase_index,
    uint8_t target,
    const SearchRunParams_t* params,
    float* speed_now_mm_s,
    f413_run_session_guard_t* guard,
    bool acceled)
{
  f413_run_session_abort_reason_t reason;
  uint32_t motion_start_ms;
  uint16_t action = (action_count != NULL) ? *action_count : 0U;

  motion_start_ms = f413_search_step_tick();
  reason = f413_search_step_run_final_stop(speed_now_mm_s, guard);
  f413_search_event_append(F413_SEARCH_EVENT_MOTION_END,
                           op_case,
                           action,
                           (uint8_t)mouse.x,
                           (uint8_t)mouse.y,
                           (uint8_t)mouse.dir,
                           0U,
                           phase_index,
                           target,
                           f413_search_event_flags(false, acceled, acceled, false),
                           f413_search_step_i32_round((float)DIST_HALF_SEC * 1000.0f),
                           0,
                           f413_search_event_pack_motion(F413_SEARCH_EVENT_MOTION_FINAL_STOP,
                                                         (uint8_t)reason,
                                                         (uint16_t)(f413_search_step_tick() -
                                                                    motion_start_ms)),
                           g_config.trace_search_safe_flag);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  reason = f413_search_step_drive_wait(guard);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  if (speed_now_mm_s != NULL)
  {
    *speed_now_mm_s = 0.0f;
  }
  f413_ctrl_stop_omega_profile();
  f413_ctrl_clear_angle_target();
  f413_ctrl_reset_distance();
  f413_ctrl_reset_angle();
  f413_wall_runtime_control_clear();
  f413_search_step_angle_reset_streak_clear();

  reason = f413_search_step_read_and_write_current_wall(true);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  motion_start_ms = f413_search_step_tick();
  reason = f413_search_step_reverse_distance((float)DIST_FIRST_SEC, guard);
  f413_search_event_append(F413_SEARCH_EVENT_MOTION_END,
                           op_case,
                           action,
                           (uint8_t)mouse.x,
                           (uint8_t)mouse.y,
                           (uint8_t)mouse.dir,
                           0U,
                           phase_index,
                           target,
                           f413_search_event_flags(false, false, false, false),
                           -f413_search_step_i32_round((float)DIST_FIRST_SEC * 1000.0f),
                           0,
                           f413_search_event_pack_motion(F413_SEARCH_EVENT_MOTION_REVERSE,
                                                         (uint8_t)reason,
                                                         (uint16_t)(f413_search_step_tick() -
                                                                    motion_start_ms)),
                           g_config.trace_search_safe_flag);
  if (reason != F413_RUN_SESSION_ABORT_NONE)
  {
    return reason;
  }

  if (speed_now_mm_s != NULL)
  {
    *speed_now_mm_s = 0.0f;
  }
  f413_ctrl_reset_distance();
  f413_ctrl_reset_angle();
  f413_search_step_angle_reset_streak_clear();

  f413_search_event_append(F413_SEARCH_EVENT_DECISION,
                           op_case,
                           action,
                           (uint8_t)mouse.x,
                           (uint8_t)mouse.y,
                           (uint8_t)mouse.dir,
                           0U,
                           phase_index,
                           target,
                           f413_search_event_flags(false, false, false, false),
                           f413_search_event_pack_wall(wall_info, map[mouse.y][mouse.x]),
                           -1,
                           0,
                           g_config.trace_search_safe_flag);
  motion_start_ms = f413_search_step_tick();
  reason = f413_search_step_run_entry_section(op_case,
                                              target,
                                              params,
                                              speed_now_mm_s,
                                              guard);
  f413_search_event_append(F413_SEARCH_EVENT_MOTION_END,
                           op_case,
                           action,
                           (uint8_t)mouse.x,
                           (uint8_t)mouse.y,
                           (uint8_t)mouse.dir,
                           0U,
                           phase_index,
                           target,
                           f413_search_event_flags(false, false, false, false),
                           f413_search_step_i32_round((float)(DIST_FIRST_SEC + DIST_HALF_SEC) *
                                                      1000.0f),
                           f413_search_step_i32_round(((speed_now_mm_s != NULL) ?
                                                           *speed_now_mm_s : 0.0f) *
                                                      1000.0f),
                           f413_search_event_pack_motion(F413_SEARCH_EVENT_MOTION_ENTRY,
                                                         (uint8_t)reason,
                                                         (uint16_t)(f413_search_step_tick() -
                                                                    motion_start_ms)),
                           g_config.trace_search_safe_flag);
  if ((reason == F413_RUN_SESSION_ABORT_NONE) && (action_count != NULL))
  {
    (*action_count)++;
  }
  return reason;
}

void f413_search_step_run_case0_test_once(
    uint8_t sub,
    const f413_search_step_case0_test_config_t* test_config)
{
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};
  const SearchRunParams_t* params;
  float speed_now_mm_s = 0.0f;
  bool trace_started = false;
  bool completed = false;
  const uint16_t fwd_flags = (uint16_t)(g_config.trace_search_safe_flag |
                                       g_config.trace_motor_fwd_flag);

  if ((test_config == NULL) || (test_config->param_index >= 2U) ||
      (test_config->test_kind == F413_SEARCH_STEP_CASE0_TEST_NONE))
  {
    trace_printf("[SEARCH-TEST] no-op: unsupported mode1 case0 sub%u\r\n",
                 (unsigned int)sub);
    return;
  }
  if (f413_search_step_trace_auto_is_enabled())
  {
    trace_printf("[SEARCH-TEST] busy(auto already running)\r\n");
    return;
  }
  if (f413_search_step_stop_switch_pressed())
  {
    trace_printf("[SEARCH-TEST] canceled(start switch pressed)\r\n");
    return;
  }
  if (!f413_run_session_guard_prepare(&guard))
  {
    trace_printf("[SEARCH-TEST] canceled(guard init fail)\r\n");
    return;
  }

  params = &searchRunParams[test_config->param_index];
  g_search_sensor_kx = f413_search_step_sensor_kx_sanitize(params->sensor_kx);
  f413_run_features_set(&test_config->features);
  f413_wall_runtime_set_control_gains(params->kp_wall, 0.0f);
  f413_search_step_map_init_empty();
  f413_search_step_angle_reset_streak_clear();
  mouse.x = START_X;
  mouse.y = START_Y;
  mouse.dir = 0U;
  g_session_active = true;

  trace_printf("[SEARCH-TEST] start mode1 case0 sub%u %s param=%u v_turn=%.0f alpha=%.0f in=%.1f out=%.1f\r\n",
               (unsigned int)sub,
               (test_config->label != NULL) ? test_config->label : "search-test",
               (unsigned int)test_config->param_index,
               (double)params->velocity_turn90,
               (double)params->alpha_turn90,
               (double)params->dist_offset_in,
               (double)params->dist_offset_out);

  f413_ctrl_start();
  f413_ctrl_reset_distance();
  f413_ctrl_reset_angle();
  f413_search_step_set_context(1U, 0U, sub, (uint8_t)'T');
  f413_search_step_set_front_match_trace(false,
                                         F413_FRONT_MATCH_PHASE_ALIGN_YAW,
                                         0.0f,
                                         0.0f,
                                         0.0f,
                                         0.0f,
                                         0U);
  f413_search_step_set_trace_period(
      (test_config->test_kind == F413_SEARCH_STEP_CASE0_TEST_FRONT_MATCH_CONTINUOUS)
          ? MATCH_POS_TRACE_PERIOD_MS
          : 1U);
  f413_search_step_trace_start();
  trace_started = true;

  if (test_config->test_kind == F413_SEARCH_STEP_CASE0_TEST_TURN_R90)
  {
    abort_reason = f413_search_step_drive_accel_distance((float)DIST_HALF_SEC,
                                                         params,
                                                         &speed_now_mm_s,
                                                         &guard,
                                                         fwd_flags);
    if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
    {
      abort_reason = f413_search_step_run_smooth_turn(1U,
                                                      params,
                                                      &speed_now_mm_s,
                                                      &guard);
    }
  }
  else if (test_config->test_kind == F413_SEARCH_STEP_CASE0_TEST_SPIN_R720)
  {
    abort_reason = f413_search_step_run_spot_spin_profile(
        F413_SEARCH_STEP_SPIN_R720_TARGET_DEG,
        (float)ALPHA_ROTATE_90,
        &guard);
  }
  else if (test_config->test_kind == F413_SEARCH_STEP_CASE0_TEST_STRAIGHT_3)
  {
    abort_reason = f413_search_step_drive_accel_distance((float)DIST_FIRST_SEC,
                                                         params,
                                                         &speed_now_mm_s,
                                                         &guard,
                                                         fwd_flags);
    if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
    {
      abort_reason = f413_search_step_drive_segment((float)(DIST_HALF_SEC * 2),
                                                    speed_now_mm_s,
                                                    &speed_now_mm_s,
                                                    &guard,
                                                    fwd_flags);
    }
    if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
    {
      abort_reason = f413_search_step_drive_segment((float)(DIST_HALF_SEC * 2),
                                                    speed_now_mm_s,
                                                    &speed_now_mm_s,
                                                    &guard,
                                                    fwd_flags);
    }
  }
  else if (test_config->test_kind == F413_SEARCH_STEP_CASE0_TEST_FRONT_MATCH_CONTINUOUS)
  {
    abort_reason = f413_search_step_front_match_continuous(&guard);
  }
  else
  {
    abort_reason = F413_RUN_SESSION_ABORT_IMU_FAULT;
  }

  if ((abort_reason == F413_RUN_SESSION_ABORT_NONE) &&
      (test_config->test_kind != F413_SEARCH_STEP_CASE0_TEST_FRONT_MATCH_CONTINUOUS))
  {
    abort_reason = f413_search_step_run_final_stop(&speed_now_mm_s, &guard);
  }
  if ((abort_reason == F413_RUN_SESSION_ABORT_NONE) &&
      (test_config->test_kind != F413_SEARCH_STEP_CASE0_TEST_FRONT_MATCH_CONTINUOUS))
  {
    abort_reason = f413_search_step_wait_stop_tail(&guard);
  }
  completed = (abort_reason == F413_RUN_SESSION_ABORT_NONE);

  f413_ctrl_stop();
  if (trace_started && (abort_reason != F413_RUN_SESSION_ABORT_NONE))
  {
    f413_search_step_set_mode_flags((uint16_t)(g_config.trace_search_safe_flag |
        f413_run_session_abort_reason_to_trace_flag(abort_reason)));
    f413_search_step_trace_auto_step();
  }
  if (trace_started)
  {
    f413_search_step_set_mode_flags(0U);
    f413_search_step_trace_stop();
  }
  f413_search_step_set_front_match_trace(false,
                                         F413_FRONT_MATCH_PHASE_ALIGN_YAW,
                                         0.0f,
                                         0.0f,
                                         0.0f,
                                         0.0f,
                                         0U);
  f413_search_step_set_trace_period(1U);
  f413_run_session_guard_cleanup(&guard);
  f413_run_features_reset();

  trace_printf("[SEARCH-TEST] %s mode1 case0 sub%u dist=%.0fmm angle=%.0fdeg%s%s\r\n",
               completed ? "OK" : "STOP",
               (unsigned int)sub,
               (double)f413_ctrl_get_distance(),
               (double)f413_ctrl_get_angle(),
               (abort_reason != F413_RUN_SESSION_ABORT_NONE) ? " abort=" : "",
               (abort_reason != F413_RUN_SESSION_ABORT_NONE)
                   ? f413_run_session_abort_reason_to_text(abort_reason)
                   : "");
}

void f413_search_step_config(const f413_search_step_config_t* config)
{
  if (config != NULL)
  {
    g_config = *config;
  }
}

void f413_search_step_session_reset(void)
{
  g_session_active = false;
  g_search_sensor_kx = 1.0f;
  g_search_wall_read_valid = false;
  f413_search_step_angle_reset_streak_clear();
  mouse.x = START_X;
  mouse.y = START_Y;
  mouse.dir = 0U;
}

void f413_search_step_run_config_once(uint8_t op_case,
                                      const f413_search_step_case_config_t* case_config)
{
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};
  const SearchRunParams_t* params;
  float speed_now_mm_s = 0.0f;
  uint16_t action_count = 0U;
  uint8_t phase_index;
  bool completed = false;
  bool route_failed = false;
  bool event_log_started = false;
  bool acceled = false;
  HAL_StatusTypeDef save_status = HAL_OK;
  bool save_attempted = false;

  if ((case_config == NULL) || (case_config->phase_count == 0U) ||
      (case_config->phase_count > 2U) || (case_config->param_index >= 2U))
  {
    trace_printf("[SEARCH-RUN] no-op: unsupported case%u\r\n", (unsigned int)op_case);
    return;
  }
  if (f413_search_step_trace_auto_is_enabled())
  {
    trace_printf("[SEARCH-RUN] busy(auto already running)\r\n");
    return;
  }
  if (f413_search_step_stop_switch_pressed())
  {
    trace_printf("[SEARCH-RUN] canceled(start switch pressed)\r\n");
    return;
  }
  if (!f413_run_session_guard_prepare(&guard))
  {
    trace_printf("[SEARCH-RUN] canceled(guard init fail)\r\n");
    return;
  }

  params = &searchRunParams[case_config->param_index];
  g_search_sensor_kx = f413_search_step_sensor_kx_sanitize(params->sensor_kx);
  f413_run_features_set(&case_config->features);
  f413_wall_runtime_set_control_gains(params->kp_wall, 0.0f);
  f413_search_step_map_init_empty();
  f413_search_step_angle_reset_streak_clear();
  g_search_no_path_exit = false;
  g_post_goal_active = false;
  f413_search_step_post_goal_reset_baseline();
  mouse.x = START_X;
  mouse.y = START_Y;
  mouse.dir = 0U;
  g_session_active = true;

  trace_printf("[SEARCH-RUN] start mode1 case%u param=%u v=%.0f target=%.0fmm phases=",
               (unsigned int)op_case,
               (unsigned int)case_config->param_index,
               (double)params->velocity_turn90,
               (double)g_config.step_target_mm);
  for (phase_index = 0U; phase_index < case_config->phase_count; phase_index++)
  {
    trace_printf("%s%s",
                 f413_search_step_target_name(case_config->phases[phase_index]),
                 ((phase_index + 1U) < case_config->phase_count) ? "," : "");
  }
  trace_printf("\r\n");

  f413_ctrl_start();
  f413_ctrl_reset_distance();
  f413_ctrl_reset_angle();

  event_log_started = f413_search_event_start(op_case,
                                              case_config->param_index,
                                              case_config->phase_count,
                                              case_config->phases);
  abort_reason = f413_search_step_read_and_write_current_wall(true);
  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    uint32_t motion_start_ms;

    f413_search_event_append(F413_SEARCH_EVENT_DECISION,
                             op_case,
                             action_count,
                             (uint8_t)mouse.x,
                             (uint8_t)mouse.y,
                             (uint8_t)mouse.dir,
                             0U,
                             0x0FU,
                             case_config->phases[0],
                             f413_search_event_flags(false, false, acceled, false),
                             f413_search_event_pack_wall(wall_info, map[mouse.y][mouse.x]),
                             -1,
                             (int32_t)((uint32_t)case_config->param_index << 16U),
                             g_config.trace_search_safe_flag);
    motion_start_ms = f413_search_step_tick();
    abort_reason = f413_search_step_run_entry_section(op_case,
                                                      case_config->phases[0],
                                                      params,
                                                      &speed_now_mm_s,
                                                      &guard);
    f413_search_event_append(F413_SEARCH_EVENT_MOTION_END,
                             op_case,
                             action_count,
                             (uint8_t)mouse.x,
                             (uint8_t)mouse.y,
                             (uint8_t)mouse.dir,
                             0U,
                             0x0FU,
                             case_config->phases[0],
                             f413_search_event_flags(false, false, acceled, false),
                             f413_search_step_i32_round((float)(DIST_FIRST_SEC + DIST_HALF_SEC) * 1000.0f),
                             f413_search_step_i32_round(speed_now_mm_s * 1000.0f),
                             f413_search_event_pack_motion(F413_SEARCH_EVENT_MOTION_ENTRY,
                                                           (uint8_t)abort_reason,
                                                           (uint16_t)(f413_search_step_tick() - motion_start_ms)),
                             g_config.trace_search_safe_flag);
    if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
    {
      action_count++;
    }
  }

  for (phase_index = 0U; phase_index < case_config->phase_count; phase_index++)
  {
    const uint8_t target = case_config->phases[phase_index];
    bool phase_done = false;

    trace_printf("[SEARCH-RUN] phase%u target=%s pos=(%u,%u,%u)\r\n",
                 (unsigned int)phase_index,
                 f413_search_step_target_name(target),
                 (unsigned int)mouse.x,
                 (unsigned int)mouse.y,
                 (unsigned int)mouse.dir);
    f413_search_event_append(F413_SEARCH_EVENT_PHASE,
                             op_case,
                             action_count,
                             (uint8_t)mouse.x,
                             (uint8_t)mouse.y,
                             (uint8_t)mouse.dir,
                             0U,
                             phase_index,
                             target,
                             f413_search_event_flags(false, acceled, acceled, false),
                             f413_search_event_pack_wall(wall_info, map[mouse.y][mouse.x]),
                             (int32_t)F413_SEARCH_EVENT_PHASE_START,
                             0,
                             g_config.trace_search_safe_flag);

    while ((abort_reason == F413_RUN_SESSION_ABORT_NONE) && !phase_done)
    {
      uint8_t next_rel = 0U;
      uint8_t next_after_forward = 0xFFU;
      bool known_straight = false;
      bool next_is_turn90 = false;
      bool acceled_before = acceled;
      uint32_t motion_start_ms;
      int step;

      if (action_count >= F413_SEARCH_STEP_AUTO_MAX_ACTIONS)
      {
        trace_printf("[SEARCH-RUN] stop(max actions=%u)\r\n",
                     (unsigned int)F413_SEARCH_STEP_AUTO_MAX_ACTIONS);
        f413_search_event_append_route_fail(op_case,
                                            action_count,
                                            (uint8_t)mouse.x,
                                            (uint8_t)mouse.y,
                                            (uint8_t)mouse.dir,
                                            phase_index,
                                            target,
                                            acceled,
                                            F413_SEARCH_EVENT_ROUTE_FAIL_MAX_ACTIONS,
                                            -1);
        route_failed = true;
        break;
      }
      if (f413_search_step_stop_switch_pressed())
      {
        abort_reason = F413_RUN_SESSION_ABORT_SWITCH;
        break;
      }

      if (f413_search_step_target_reached(target))
      {
        trace_printf("[SEARCH-RUN] phase%u reached target=%s pos=(%u,%u,%u)\r\n",
                     (unsigned int)phase_index,
                     f413_search_step_target_name(target),
                     (unsigned int)mouse.x,
                     (unsigned int)mouse.y,
                     (unsigned int)mouse.dir);
        f413_search_event_append(F413_SEARCH_EVENT_PHASE,
                                 op_case,
                                 action_count,
                                 (uint8_t)mouse.x,
                                 (uint8_t)mouse.y,
                                 (uint8_t)mouse.dir,
                                 0U,
                                 phase_index,
                                 target,
                                 f413_search_event_flags(false, acceled, acceled, false),
                                 f413_search_event_pack_wall(wall_info, map[mouse.y][mouse.x]),
                                 (int32_t)F413_SEARCH_EVENT_PHASE_REACHED,
                                 0,
                                 g_config.trace_search_safe_flag);
        phase_done = true;
        break;
      }

      step = f413_search_step_make_smap((uint8_t)mouse.x, (uint8_t)mouse.y, target);
      if (step == -2)
      {
        trace_printf("[SEARCH-RUN] phase%u full complete pos=(%u,%u,%u)\r\n",
                     (unsigned int)phase_index,
                     (unsigned int)mouse.x,
                     (unsigned int)mouse.y,
                     (unsigned int)mouse.dir);
        f413_search_event_append(F413_SEARCH_EVENT_PHASE,
                                 op_case,
                                 action_count,
                                 (uint8_t)mouse.x,
                                 (uint8_t)mouse.y,
                                 (uint8_t)mouse.dir,
                                 0U,
                                 phase_index,
                                 target,
                                 f413_search_event_flags(false, acceled, acceled, false),
                                 f413_search_event_pack_wall(wall_info, map[mouse.y][mouse.x]),
                                 (int32_t)F413_SEARCH_EVENT_PHASE_FULL_COMPLETE,
                                 0,
                                 g_config.trace_search_safe_flag);
        phase_done = true;
        break;
      }
      if ((target == F413_SEARCH_STEP_TARGET_FULL) && (step < 0))
      {
        trace_printf("[SEARCH-RUN] phase%u full unreachable pos=(%u,%u,%u) wall=0x%04X cell=0x%04X step=%d\r\n",
                     (unsigned int)phase_index,
                     (unsigned int)mouse.x,
                     (unsigned int)mouse.y,
                     (unsigned int)mouse.dir,
                     (unsigned int)wall_info,
                     (unsigned int)map[mouse.y][mouse.x],
                     step);
        f413_search_event_append(F413_SEARCH_EVENT_PHASE,
                                 op_case,
                                 action_count,
                                 (uint8_t)mouse.x,
                                 (uint8_t)mouse.y,
                                 (uint8_t)mouse.dir,
                                 0U,
                                 phase_index,
                                 target,
                                 f413_search_event_flags(false, acceled, acceled, false),
                                 f413_search_event_pack_wall(wall_info, map[mouse.y][mouse.x]),
                                 (int32_t)F413_SEARCH_EVENT_PHASE_FULL_UNREACHABLE,
                                 (int32_t)step,
                                 g_config.trace_search_safe_flag);
        phase_done = true;
        break;
      }
      if (step < 0)
      {
        trace_printf("[SEARCH-RUN] FAIL(no current step) phase=%u target=%s pos=(%u,%u,%u) wall=0x%04X cell=0x%04X step=%d\r\n",
                     (unsigned int)phase_index,
                     f413_search_step_target_name(target),
                     (unsigned int)mouse.x,
                     (unsigned int)mouse.y,
                     (unsigned int)mouse.dir,
                     (unsigned int)wall_info,
                     (unsigned int)map[mouse.y][mouse.x],
                     step);
        f413_search_event_append_route_fail(op_case,
                                            action_count,
                                            (uint8_t)mouse.x,
                                            (uint8_t)mouse.y,
                                            (uint8_t)mouse.dir,
                                            phase_index,
                                            target,
                                            acceled,
                                            F413_SEARCH_EVENT_ROUTE_FAIL_NO_CURRENT_STEP,
                                            step);
        route_failed = true;
        break;
      }
      if (!f413_search_step_choose_next_relative((uint8_t)mouse.x,
                                                 (uint8_t)mouse.y,
                                                 (uint8_t)mouse.dir,
                                                 &next_rel))
      {
        trace_printf("[SEARCH-RUN] FAIL(no route) phase=%u target=%s pos=(%u,%u,%u) wall=0x%04X cell=0x%04X step=%d\r\n",
                     (unsigned int)phase_index,
                     f413_search_step_target_name(target),
                     (unsigned int)mouse.x,
                     (unsigned int)mouse.y,
                     (unsigned int)mouse.dir,
                     (unsigned int)wall_info,
                     (unsigned int)map[mouse.y][mouse.x],
                     step);
        f413_search_event_append_route_fail(op_case,
                                            action_count,
                                            (uint8_t)mouse.x,
                                            (uint8_t)mouse.y,
                                            (uint8_t)mouse.dir,
                                            phase_index,
                                            target,
                                            acceled,
                                            F413_SEARCH_EVENT_ROUTE_FAIL_NO_NEXT_REL,
                                            step);
        route_failed = true;
        break;
      }

      f413_search_step_set_action_context(op_case,
                                          (uint8_t)mouse.x,
                                          (uint8_t)mouse.y,
                                          (uint8_t)mouse.dir,
                                          next_rel);
      if (next_rel == 0U)
      {
        known_straight = f413_search_step_forward_is_known_straight((uint8_t)mouse.x,
                                                                    (uint8_t)mouse.y,
                                                                    (uint8_t)mouse.dir,
                                                                    &next_after_forward);
        next_is_turn90 = (next_after_forward == 1U) || (next_after_forward == 3U);
      }
      trace_printf("[SEARCH-RUN] action%u phase=%u next=%s(%u) pos=(%u,%u,%u) smap=%d wall=0x%04X cell=0x%04X\r\n",
                   (unsigned int)action_count,
                   (unsigned int)phase_index,
                   f413_search_step_relative_name(next_rel),
                   (unsigned int)next_rel,
                   (unsigned int)mouse.x,
                   (unsigned int)mouse.y,
                   (unsigned int)mouse.dir,
                   step,
                   (unsigned int)wall_info,
                   (unsigned int)map[mouse.y][mouse.x]);

      f413_search_event_append(F413_SEARCH_EVENT_DECISION,
                               op_case,
                               action_count,
                               (uint8_t)mouse.x,
                               (uint8_t)mouse.y,
                               (uint8_t)mouse.dir,
                               next_rel,
                               phase_index,
                               target,
                               f413_search_event_flags(known_straight,
                                                       acceled_before,
                                                       acceled,
                                                       next_is_turn90),
                               f413_search_event_pack_wall(wall_info, map[mouse.y][mouse.x]),
                               (int32_t)step,
                               (int32_t)((uint32_t)next_after_forward |
                                         ((uint32_t)case_config->param_index << 16U)),
                               g_config.trace_search_safe_flag);
      motion_start_ms = f413_search_step_tick();
      abort_reason = f413_search_step_run_search_motion(next_rel,
                                                        params,
                                                        &speed_now_mm_s,
                                                        &guard,
                                                        &acceled,
                                                        known_straight,
                                                        next_is_turn90);
      f413_search_event_append(F413_SEARCH_EVENT_MOTION_END,
                               op_case,
                               action_count,
                               (uint8_t)mouse.x,
                               (uint8_t)mouse.y,
                               (uint8_t)mouse.dir,
                               next_rel,
                               phase_index,
                               target,
                               f413_search_event_flags(known_straight,
                                                       acceled_before,
                                                       acceled,
                                                       next_is_turn90),
                               f413_search_step_i32_round(
                                   ((next_rel == 0U) ? (float)(DIST_HALF_SEC * 2)
                                                     : f413_search_step_turn_target_deg(next_rel)) *
                                   1000.0f),
                               f413_search_step_i32_round(speed_now_mm_s * 1000.0f),
                               f413_search_event_pack_motion(f413_search_event_motion_kind_from_rel(next_rel),
                                                             (uint8_t)abort_reason,
                                                             (uint16_t)(f413_search_step_tick() - motion_start_ms)),
                               g_config.trace_search_safe_flag);
      if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }
      if (g_search_no_path_exit)
      {
        trace_printf("[SEARCH-RUN] FAIL(map save guard) phase=%u target=%s pos=(%u,%u,%u) wall=0x%04X cell=0x%04X\r\n",
                     (unsigned int)phase_index,
                     f413_search_step_target_name(target),
                     (unsigned int)mouse.x,
                     (unsigned int)mouse.y,
                     (unsigned int)mouse.dir,
                     (unsigned int)wall_info,
                     (unsigned int)map[mouse.y][mouse.x]);
        f413_search_event_append_route_fail(op_case,
                                            action_count,
                                            (uint8_t)mouse.x,
                                            (uint8_t)mouse.y,
                                            (uint8_t)mouse.dir,
                                            phase_index,
                                            target,
                                            acceled,
                                            F413_SEARCH_EVENT_ROUTE_FAIL_MAP_SAVE_GUARD,
                                            step);
        route_failed = true;
        break;
      }

      mouse.dir = (uint8_t)((mouse.dir + next_rel) & 0x03U);
      f413_search_step_advance_position();
      abort_reason = f413_search_step_read_and_write_current_wall(false);
      if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }
      f413_search_step_note_goal_visit(target);
      if (next_rel == 0U)
      {
        f413_search_step_angle_reset_streak_update();
      }
      else
      {
        f413_search_step_angle_reset_streak_clear();
      }
      action_count++;
    }

    if ((abort_reason != F413_RUN_SESSION_ABORT_NONE) || route_failed)
    {
      break;
    }
    if (phase_done && ((phase_index + 1U) < case_config->phase_count))
    {
      const uint8_t next_phase = (uint8_t)(phase_index + 1U);
      const uint8_t next_target = case_config->phases[next_phase];

      acceled = false;
      trace_printf("[SEARCH-RUN] phase%u restart for target=%s pos=(%u,%u,%u)\r\n",
                   (unsigned int)next_phase,
                   f413_search_step_target_name(next_target),
                   (unsigned int)mouse.x,
                   (unsigned int)mouse.y,
                   (unsigned int)mouse.dir);
      abort_reason = f413_search_step_run_phase_restart_entry(op_case,
                                                              &action_count,
                                                              next_phase,
                                                              next_target,
                                                              params,
                                                              &speed_now_mm_s,
                                                              &guard,
                                                              acceled);
      if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
      {
        break;
      }
    }
  }

  completed = (abort_reason == F413_RUN_SESSION_ABORT_NONE) && !route_failed;
  if (completed)
  {
    uint32_t motion_start_ms = f413_search_step_tick();
    abort_reason = f413_search_step_run_final_stop(&speed_now_mm_s, &guard);
    f413_search_event_append(F413_SEARCH_EVENT_MOTION_END,
                             op_case,
                             action_count,
                             (uint8_t)mouse.x,
                             (uint8_t)mouse.y,
                             (uint8_t)mouse.dir,
                             0U,
                             0x0EU,
                             0U,
                             f413_search_event_flags(false, acceled, acceled, false),
                             f413_search_step_i32_round((float)DIST_HALF_SEC * 1000.0f),
                             0,
                             f413_search_event_pack_motion(F413_SEARCH_EVENT_MOTION_FINAL_STOP,
                                                           (uint8_t)abort_reason,
                                                           (uint16_t)(f413_search_step_tick() - motion_start_ms)),
                             g_config.trace_search_safe_flag);
    completed = (abort_reason == F413_RUN_SESSION_ABORT_NONE);
  }
  if (completed)
  {
    abort_reason = f413_search_step_wait_stop_tail(&guard);
    completed = (abort_reason == F413_RUN_SESSION_ABORT_NONE);
  }
  if (completed)
  {
    save_attempted = true;
    if (!f413_search_step_try_save_map_safely(&save_status))
    {
      route_failed = true;
      completed = false;
      f413_search_event_append_route_fail(op_case,
                                          action_count,
                                          (uint8_t)mouse.x,
                                          (uint8_t)mouse.y,
                                          (uint8_t)mouse.dir,
                                          0x0EU,
                                          0U,
                                          acceled,
                                          F413_SEARCH_EVENT_ROUTE_FAIL_MAP_SAVE_GUARD,
                                          -1);
    }
  }

  f413_ctrl_stop();
  if (event_log_started)
  {
    f413_search_step_set_mode_flags(0U);
    f413_search_event_finish(op_case, action_count, completed, route_failed, abort_reason);
  }
  f413_run_session_guard_cleanup(&guard);

  trace_printf("[SEARCH-RUN] %s case%u actions=%u pos=(%u,%u,%u) save=%s%s%s%s\r\n",
               completed ? "OK" : "STOP",
               (unsigned int)op_case,
               (unsigned int)action_count,
               (unsigned int)mouse.x,
               (unsigned int)mouse.y,
               (unsigned int)mouse.dir,
               save_attempted ? ((save_status == HAL_OK) ? "OK" : "FAIL") : "SKIP",
               route_failed ? " route_failed" : "",
               (abort_reason != F413_RUN_SESSION_ABORT_NONE) ? " abort=" : "",
               (abort_reason != F413_RUN_SESSION_ABORT_NONE)
                   ? f413_run_session_abort_reason_to_text(abort_reason)
                   : "");
}

void f413_search_step_run_search_case_once(uint8_t op_case)
{
  f413_search_step_case_config_t config;
  const f413_run_features_t default_features = {
    .wall_control_enabled = true,
    .wall_end_correction_enabled = true,
    .front_wall_correction_enabled = true,
    .angle_accum_mode = true,
    .test_mode_run = false,
  };

  if (!f413_search_step_plan_from_case(op_case, &config))
  {
    trace_printf("[SEARCH-RUN] no-op: unsupported case%u\r\n", (unsigned int)op_case);
    return;
  }
  config.label = "mode1 fallback";
  config.features = default_features;
  if ((op_case == 1U) || (op_case == 3U) || (op_case == 4U))
  {
    config.features.wall_end_correction_enabled = false;
  }
  f413_search_step_run_config_once(op_case, &config);
}

void f413_search_step_run_decision_preview_once(void)
{
  f413_wall_sensor_snapshot_t wall;
  uint8_t next_rel = 0U;
  int step;
  HAL_StatusTypeDef st;

  g_search_sensor_kx = f413_search_step_sensor_kx_sanitize(searchRunParams[0].sensor_kx);
  if (!f413_search_step_read_wall(&wall))
  {
    trace_printf("[SEARCH-PREVIEW] FAIL(read wall snapshot)\r\n");
    return;
  }

  f413_search_step_map_init_empty();
  mouse.x = START_X;
  mouse.y = START_Y;
  mouse.dir = 0U;
  wall_info = f413_search_step_wall_info_from_snapshot(&wall);
  f413_search_step_write_map_cell((uint8_t)mouse.x, (uint8_t)mouse.y, (uint8_t)mouse.dir, wall_info);
  step = f413_search_step_make_goal_smap((uint8_t)mouse.x, (uint8_t)mouse.y);

  if ((step < 0) || !f413_search_step_choose_next_relative((uint8_t)mouse.x, (uint8_t)mouse.y, (uint8_t)mouse.dir, &next_rel))
  {
    trace_printf("[SEARCH-PREVIEW] FAIL(no route) wall_info=0x%04X cell=0x%04X\r\n",
                 (unsigned int)wall_info,
                 (unsigned int)map[START_Y][START_X]);
    return;
  }

  st = nvm_maze_save_map(&map[0][0], F413_SEARCH_STEP_CELL_COUNT);
  trace_printf("[SEARCH-PREVIEW] next=%s(%u) smap=%d save=%s wall_info=0x%04X cell=0x%04X dFR=%ld dR=%ld dFL=%ld dL=%ld\r\n",
               f413_search_step_relative_name(next_rel),
               (unsigned int)next_rel,
               step,
               (st == HAL_OK) ? "OK" : "FAIL",
               (unsigned int)wall_info,
               (unsigned int)map[START_Y][START_X],
               (long)wall.fr_delta,
               (long)wall.r_delta,
               (long)wall.fl_delta,
               (long)wall.l_delta);
}

void f413_search_step_run_once(void)
{
  f413_wall_sensor_snapshot_t wall;
  f413_run_session_abort_reason_t abort_reason = F413_RUN_SESSION_ABORT_NONE;
  f413_run_session_guard_t guard = {0};
  uint8_t next_rel = 0U;
  int step;
  HAL_StatusTypeDef st;
  bool is_turn;
  float turn_target_deg;

  g_search_sensor_kx = f413_search_step_sensor_kx_sanitize(searchRunParams[0].sensor_kx);
  if (f413_search_step_trace_auto_is_enabled())
  {
    trace_printf("[SEARCH-STEP] busy(auto already running)\r\n");
    return;
  }
  if (f413_search_step_stop_switch_pressed())
  {
    trace_printf("[SEARCH-STEP] canceled(start switch pressed)\r\n");
    return;
  }
  if (!f413_search_step_read_wall(&wall))
  {
    trace_printf("[SEARCH-STEP] FAIL(read wall snapshot)\r\n");
    return;
  }

  if (!g_session_active)
  {
    f413_search_step_map_init_empty();
    f413_search_step_angle_reset_streak_clear();
    mouse.x = START_X;
    mouse.y = START_Y;
    mouse.dir = 0U;
    g_session_active = true;
    trace_printf("[SEARCH-STEP] session start pos=(%u,%u,%u)\r\n",
                 (unsigned int)mouse.x,
                 (unsigned int)mouse.y,
                 (unsigned int)mouse.dir);
  }

  wall_info = f413_search_step_wall_info_from_snapshot(&wall);
  f413_search_step_write_map_cell((uint8_t)mouse.x, (uint8_t)mouse.y, (uint8_t)mouse.dir, wall_info);
  visited[mouse.y][mouse.x] = true;
  step = f413_search_step_make_goal_smap((uint8_t)mouse.x, (uint8_t)mouse.y);

  if ((step < 0) || !f413_search_step_choose_next_relative((uint8_t)mouse.x, (uint8_t)mouse.y, (uint8_t)mouse.dir, &next_rel))
  {
    trace_printf("[SEARCH-STEP] FAIL(no route) pos=(%u,%u,%u) wall_info=0x%04X cell=0x%04X\r\n",
                 (unsigned int)mouse.x,
                 (unsigned int)mouse.y,
                 (unsigned int)mouse.dir,
                 (unsigned int)wall_info,
                 (unsigned int)map[mouse.y][mouse.x]);
    return;
  }

  st = nvm_maze_save_map(&map[0][0], F413_SEARCH_STEP_CELL_COUNT);
  if (st != HAL_OK)
  {
    trace_printf("[SEARCH-STEP] FAIL(save HAL=%d)\r\n", (int)st);
    return;
  }
  if (!f413_run_session_guard_prepare(&guard))
  {
    trace_printf("[SEARCH-STEP] canceled(guard init fail)\r\n");
    return;
  }

  is_turn = (next_rel != 0U);
  turn_target_deg = f413_search_step_turn_target_deg(next_rel);
  trace_printf("[SEARCH-STEP] run next=%s(%u) pos=(%u,%u,%u) smap=%d wall_info=0x%04X cell=0x%04X\r\n",
               f413_search_step_relative_name(next_rel),
               (unsigned int)next_rel,
               (unsigned int)mouse.x,
               (unsigned int)mouse.y,
               (unsigned int)mouse.dir,
               step,
               (unsigned int)wall_info,
               (unsigned int)map[mouse.y][mouse.x]);

  f413_search_step_set_action_context(4U,
                                      (uint8_t)mouse.x,
                                      (uint8_t)mouse.y,
                                      (uint8_t)mouse.dir,
                                      next_rel);
  f413_search_step_trace_start();
  f413_ctrl_start();
  f413_ctrl_reset_distance();
  f413_ctrl_reset_angle();
  f413_wall_runtime_set_control_gains(searchRunParams[0].kp_wall, 0.0f);

  if (is_turn)
  {
    f413_wall_runtime_control_clear();
    f413_ctrl_set_velocity(0.0f);
    f413_ctrl_set_omega(0.0f);
    f413_ctrl_set_angle_target(turn_target_deg);
    abort_reason = f413_search_step_wait_ctrl_target(turn_target_deg, true, &guard,
        (uint16_t)(g_config.trace_search_safe_flag |
                   g_config.trace_motor_rev_flag));
  }
  else
  {
    f413_ctrl_set_velocity(g_config.step_velocity_mm_s);
    f413_ctrl_set_omega(0.0f);
    f413_ctrl_clear_angle_target();
    abort_reason = f413_search_step_wait_ctrl_target(g_config.step_target_mm, false, &guard,
        (uint16_t)(g_config.trace_search_safe_flag |
                   g_config.trace_motor_fwd_flag));
  }

  f413_ctrl_clear_angle_target();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    f413_search_step_set_mode_flags((uint16_t)(g_config.trace_search_safe_flag |
                                               g_config.trace_motor_coast_flag));
    abort_reason = f413_run_session_wait_with_auto_step_guarded(g_config.path_coast_ms, &guard);
  }
  f413_ctrl_stop();
  if (abort_reason != F413_RUN_SESSION_ABORT_NONE)
  {
    f413_search_step_set_mode_flags((uint16_t)(g_config.trace_search_safe_flag |
        f413_run_session_abort_reason_to_trace_flag(abort_reason)));
    f413_search_step_trace_auto_step();
  }
  f413_search_step_set_mode_flags(0U);
  f413_search_step_trace_stop();
  f413_run_session_guard_cleanup(&guard);

  if (abort_reason == F413_RUN_SESSION_ABORT_NONE)
  {
    if (is_turn)
    {
      mouse.dir = (uint8_t)((mouse.dir + next_rel) & 0x03U);
    }
    else
    {
      f413_search_step_advance_position();
    }
    trace_printf("[SEARCH-STEP] OK next_pos=(%u,%u,%u) dist=%.0fmm angle=%.0fdeg\r\n",
                 (unsigned int)mouse.x,
                 (unsigned int)mouse.y,
                 (unsigned int)mouse.dir,
                 (double)f413_ctrl_get_distance(),
                 (double)f413_ctrl_get_angle());
  }
  else
  {
    trace_printf("[SEARCH-STEP] aborted(%s) pos=(%u,%u,%u) dist=%.0fmm angle=%.0fdeg\r\n",
                 f413_run_session_abort_reason_to_text(abort_reason),
                 (unsigned int)mouse.x,
                 (unsigned int)mouse.y,
                 (unsigned int)mouse.dir,
                 (double)f413_ctrl_get_distance(),
                 (double)f413_ctrl_get_angle());
  }
}

void f413_search_step_run_map_probe_once(void)
{
  static uint16_t loaded[F413_SEARCH_STEP_CELL_COUNT];
  f413_wall_sensor_snapshot_t wall;
  HAL_StatusTypeDef st;
  uint32_t i;

  g_search_sensor_kx = f413_search_step_sensor_kx_sanitize(searchRunParams[0].sensor_kx);
  if (!f413_search_step_read_wall(&wall))
  {
    trace_printf("[SEARCH-MAP] FAIL(read wall snapshot)\r\n");
    return;
  }

  f413_search_step_map_init_empty();
  mouse.x = START_X;
  mouse.y = START_Y;
  mouse.dir = 0U;
  wall_info = f413_search_step_wall_info_from_snapshot(&wall);
  f413_search_step_write_map_cell((uint8_t)mouse.x, (uint8_t)mouse.y, (uint8_t)mouse.dir, wall_info);

  st = nvm_maze_save_map(&map[0][0], F413_SEARCH_STEP_CELL_COUNT);
  if (st != HAL_OK)
  {
    trace_printf("[SEARCH-MAP] FAIL(save HAL=%d)\r\n", (int)st);
    return;
  }

  memset(loaded, 0, sizeof(loaded));
  if (!nvm_maze_load_map(loaded, F413_SEARCH_STEP_CELL_COUNT))
  {
    trace_printf("[SEARCH-MAP] FAIL(load)\r\n");
    return;
  }
  for (i = 0U; i < F413_SEARCH_STEP_CELL_COUNT; i++)
  {
    if (loaded[i] != (&map[0][0])[i])
    {
      trace_printf("[SEARCH-MAP] FAIL(verify i=%lu got=0x%04X exp=0x%04X)\r\n",
                   (unsigned long)i,
                   (unsigned int)loaded[i],
                   (unsigned int)(&map[0][0])[i]);
      return;
    }
  }

  trace_printf("[SEARCH-MAP] PASS start=(%u,%u,%u) wall_info=0x%04X cell=0x%04X dFR=%ld dR=%ld dFL=%ld dL=%ld\r\n",
               (unsigned int)mouse.x,
               (unsigned int)mouse.y,
               (unsigned int)mouse.dir,
               (unsigned int)wall_info,
               (unsigned int)map[START_Y][START_X],
               (long)wall.fr_delta,
               (long)wall.r_delta,
               (long)wall.fl_delta,
               (long)wall.l_delta);
}

void f413_search_step_run_status_once(void)
{
  static uint16_t loaded[F413_SEARCH_STEP_CELL_COUNT];
  uint32_t visited_count = 0U;
  uint32_t current_known_count = 0U;
  uint32_t loaded_known_count = 0U;
  uint32_t mismatch_count = 0U;
  bool loaded_ok;
  uint8_t x;
  uint8_t y;

  loaded_ok = nvm_maze_load_map(loaded, F413_SEARCH_STEP_CELL_COUNT);
  for (y = 0U; y < MAZE_SIZE; y++)
  {
    for (x = 0U; x < MAZE_SIZE; x++)
    {
      uint32_t idx = (uint32_t)y * MAZE_SIZE + x;
      if (visited[y][x])
      {
        visited_count++;
      }
      if ((map[y][x] & F413_SEARCH_STEP_MAZE_WALL_KNOWN_MASK) != 0U)
      {
        current_known_count++;
      }
      if (loaded_ok)
      {
        if ((loaded[idx] & F413_SEARCH_STEP_MAZE_WALL_KNOWN_MASK) != 0U)
        {
          loaded_known_count++;
        }
        if (loaded[idx] != map[y][x])
        {
          mismatch_count++;
        }
      }
    }
  }

  trace_printf("[SEARCH-STATE] active=%u mouse=(%u,%u,%u) cell=0x%04X visited=%lu known=%lu fram=%s fram_known=%lu mismatch=%lu\r\n",
               (unsigned int)g_session_active,
               (unsigned int)mouse.x,
               (unsigned int)mouse.y,
               (unsigned int)mouse.dir,
               (unsigned int)(((mouse.x < MAZE_SIZE) && (mouse.y < MAZE_SIZE)) ? map[mouse.y][mouse.x] : 0U),
               (unsigned long)visited_count,
               (unsigned long)current_known_count,
               loaded_ok ? "OK" : "MISS",
               (unsigned long)loaded_known_count,
               (unsigned long)mismatch_count);
}

void f413_search_step_run_map_clear_once(void)
{
  HAL_StatusTypeDef st;

  f413_search_step_map_init_empty();
  f413_search_step_session_reset();
  st = nvm_maze_save_map(&map[0][0], F413_SEARCH_STEP_CELL_COUNT);
  trace_printf("[SEARCH-CLEAR] save=%s mouse=(%u,%u,%u) start_cell=0x%04X\r\n",
               (st == HAL_OK) ? "OK" : "FAIL",
               (unsigned int)mouse.x,
               (unsigned int)mouse.y,
               (unsigned int)mouse.dir,
               (unsigned int)map[START_Y][START_X]);
}

void f413_search_step_run_map_dump_once(void)
{
  static uint16_t loaded[F413_SEARCH_STEP_CELL_COUNT];
  const uint16_t* cells = &map[0][0];
  bool loaded_ok = nvm_maze_load_map(loaded, F413_SEARCH_STEP_CELL_COUNT);
  int y;
  uint8_t x;

  if (loaded_ok)
  {
    cells = loaded;
  }

  trace_printf("[SEARCH-DUMP] source=%s size=%u active=%u mouse=(%u,%u,%u)\r\n",
               loaded_ok ? "FRAM" : "RAM",
               (unsigned int)MAZE_SIZE,
               (unsigned int)g_session_active,
               (unsigned int)mouse.x,
               (unsigned int)mouse.y,
               (unsigned int)mouse.dir);
  for (y = (int)MAZE_SIZE - 1; y >= 0; y--)
  {
    trace_printf("[SEARCH-DUMP] y=%02d:", y);
    for (x = 0U; x < MAZE_SIZE; x++)
    {
      trace_printf("%02X", (unsigned int)(cells[(uint32_t)y * MAZE_SIZE + x] & 0xFFU));
    }
    trace_printf("\r\n");
  }
}
