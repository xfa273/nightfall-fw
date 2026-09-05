#include "f413_machine.h"
#include <math.h>
#include <string.h>

static f413_machine_status_t s_status;
static f413_machine_selection_t s_selection;
static f413_runtime_params_t s_params;

f413_machine_status_t f413_machine_resolve(
    nvm_status_t status, const nvm_identity_block_t *id,
    const uint32_t uid[3], const f413_board_config_t *boards, size_t board_count,
    const f413_unit_config_t *units, size_t unit_count,
    f413_machine_selection_t *out)
{
  f413_machine_selection_t candidate = {0};
  size_t i;
  bool bound;
  if (out == NULL) return F413_MACHINE_CONFIG_INVALID;
  memset(out, 0, sizeof(*out));
  if (status != NVM_STATUS_OK || id == NULL ||
      nvm_identity_validate(id) != NVM_STATUS_OK ||
      id->unit_serial == 0U || id->hw_rev_major > 255U || id->hw_rev_minor > 255U ||
      (id->board_id >> 8) != (((uint32_t)id->hw_rev_major << 8) | id->hw_rev_minor))
    return F413_MACHINE_ID_INVALID;
  if (boards == NULL || units == NULL || uid == NULL) return F413_MACHINE_CONFIG_INVALID;
  for (i = 0; i < board_count; ++i) {
    if (boards[i].family == id->family && boards[i].board_id == id->board_id) {
      if (candidate.board != NULL) return F413_MACHINE_CONFIG_INVALID;
      candidate.board = &boards[i];
    }
  }
  if (candidate.board == NULL) return F413_MACHINE_BOARD_UNKNOWN;
  if (candidate.board->layout != F413_LAYOUT_MINI_R2) return F413_MACHINE_LAYOUT_UNSUPPORTED;
  for (i = 0; i < unit_count; ++i) {
    if (units[i].family == id->family && units[i].board_id == id->board_id &&
        units[i].unit_serial == id->unit_serial) {
      if (candidate.unit != NULL) return F413_MACHINE_CONFIG_INVALID;
      candidate.unit = &units[i];
    }
  }
  if (candidate.unit == NULL) return F413_MACHINE_UNIT_UNKNOWN;
  bound = (id->mcu_uid[0] | id->mcu_uid[1] | id->mcu_uid[2]) != 0U;
  if ((candidate.unit->require_uid && !bound) ||
      (bound && (id->mcu_uid[0] != uid[0] || id->mcu_uid[1] != uid[1] || id->mcu_uid[2] != uid[2])))
    return F413_MACHINE_UID_MISMATCH;
  if (id->capability_flags != 0U && id->capability_flags != candidate.board->capabilities)
    return F413_MACHINE_CONFIG_INVALID; /* Legacy zero = inherit board capabilities. */
  candidate.profile = candidate.unit->profile_override != NULL ?
      candidate.unit->profile_override : candidate.board->default_profile;
  if (candidate.profile == NULL || candidate.profile->family != id->family ||
      candidate.profile->board_id != id->board_id || candidate.profile->id == 0U ||
      (id->default_param_profile != 0U && id->default_param_profile != candidate.profile->id))
    return F413_MACHINE_PROFILE_MISMATCH;
  candidate.hardware = candidate.unit->hardware_override != NULL ?
      *candidate.unit->hardware_override : candidate.board->hardware;
  const f413_hardware_config_t *hw = &candidate.hardware;
  const f413_param_profile_t *p = candidate.profile;
  if ((hw->encoder_sign_l != 1 && hw->encoder_sign_l != -1) ||
      (hw->encoder_sign_r != 1 && hw->encoder_sign_r != -1) ||
      (hw->imu_forward_accel_sign != 1 && hw->imu_forward_accel_sign != -1) ||
      (hw->imu_forward_accel_reg != 0x28U && hw->imu_forward_accel_reg != 0x2AU) ||
      !isfinite(hw->encoder_cpr) || hw->encoder_cpr <= 0.0f ||
      !isfinite(hw->tread_mm) || hw->tread_mm <= 0.0f ||
      p->scalar == NULL || p->search == NULL || p->name == NULL || p->load_sensor_luts == NULL)
    return F413_MACHINE_CONFIG_INVALID;
#define X(type, name) if (!isfinite((double)p->scalar->v_##name)) return F413_MACHINE_CONFIG_INVALID;
#include "f413_param_fields.def"
#undef X
  const f413_scalar_params_t *s = p->scalar;
  if (s->v_D_TIRE <= 0.0 || s->v_DIST_HALF_SEC <= 0.0 || s->v_DIST_D_HALF_SEC <= 0.0 ||
      s->v_DIST_FIRST_SEC < 0.0 || s->v_VELOCITY_ACCEL_COMP_WINDOW_MS < 1U ||
      s->v_VELOCITY_ACCEL_COMP_WINDOW_MS > 64U ||
      (id->family == NVM_FAMILY_MINI && s->v_DIST_HALF_SEC != 45.0) ||
      (id->family == NVM_FAMILY_CLASSIC && s->v_DIST_HALF_SEC != 90.0) ||
      (id->family != NVM_FAMILY_MINI && id->family != NVM_FAMILY_CLASSIC))
    return F413_MACHINE_CONFIG_INVALID;
  for (i = 0; i < 6U; ++i) {
    if (p->modes[i] == NULL || p->cases[i] == NULL) return F413_MACHINE_CONFIG_INVALID;
  }
  *out = candidate;
  return F413_MACHINE_OK;
}

f413_machine_status_t f413_machine_boot(nvm_status_t status,
    const nvm_identity_block_t *id, const uint32_t uid[3])
{
  if (s_status != F413_MACHINE_NOT_SELECTED) return s_status;
  s_status = f413_machine_resolve(status, id, uid, f413_boards, f413_board_count,
      f413_units, f413_unit_count, &s_selection);
  if (s_status == F413_MACHINE_OK) {
    const f413_param_profile_t *p = s_selection.profile;
    s_params.scalar = *p->scalar;
    memcpy(s_params.search, p->search, sizeof(s_params.search));
    for (size_t i = 0; i < 6U; ++i) {
      s_params.modes[i] = *p->modes[i];
      memcpy(s_params.cases[i], p->cases[i], sizeof(s_params.cases[i]));
    }
  }
  return s_status;
}

f413_machine_status_t f413_machine_status(void) { return s_status; }
const char *f413_machine_status_name(f413_machine_status_t status)
{
  static const char *const names[] = {"not-selected", "ok", "identity-invalid",
    "board-unknown", "layout-unsupported", "unit-unknown", "uid-mismatch",
    "profile-mismatch", "config-invalid"};
  return (unsigned)status < sizeof(names) / sizeof(names[0]) ? names[status] : "invalid-status";
}
bool f413_machine_has(uint32_t capabilities)
{
  return s_status == F413_MACHINE_OK &&
      (s_selection.board->capabilities & capabilities) == capabilities;
}
const f413_hardware_config_t *f413_machine_hardware(void) { return &s_selection.hardware; }
const f413_runtime_params_t *f413_machine_params(void) { return &s_params; }
const char *f413_machine_profile_name(void)
{ return s_status == F413_MACHINE_OK ? s_selection.profile->name : "unselected"; }
uint32_t f413_machine_profile_id(void)
{ return s_status == F413_MACHINE_OK ? s_selection.profile->id : 0U; }
bool f413_machine_route_precomputed_compatible(void)
{ return s_status == F413_MACHINE_OK && s_selection.profile->route_precomputed_compatible; }

/* Overrides sensor_distance.c's weak hook without coupling the shared F405 code
   to an F413 identity. Individual FRAM warps remain a separate calibration layer. */
void sensor_distance_load_profile_luts(void)
{
  if (s_status == F413_MACHINE_OK) s_selection.profile->load_sensor_luts();
}
