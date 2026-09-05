#include "f413_machine.h"
#include "f413_motor_pwm.h"
#include "params.h"
#include "sensor_distance.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Any persistence operation is a test failure. Identity validation is real. */
nvm_status_t nvm_read(nvm_area_t a, uint32_t o, void *p, size_t n)
{ (void)a; (void)o; (void)p; (void)n; abort(); }
nvm_status_t nvm_write(nvm_area_t a, uint32_t o, const void *p, size_t n)
{ (void)a; (void)o; (void)p; (void)n; abort(); }
nvm_status_t nvm_erase(nvm_area_t a) { (void)a; abort(); }

static const uint32_t uid[] = {0x00280047U, 0x31335117U, 0x34313932U};
static void seal(nvm_identity_block_t *id)
{
  id->magic = NVM_IDENTITY_MAGIC;
  id->schema_version = NVM_IDENTITY_SCHEMA_VERSION;
  id->length = sizeof(*id);
  id->crc = 0U;
  for (size_t i = 16; i < sizeof(*id); ++i) id->crc += ((const uint8_t *)id)[i];
}
static nvm_identity_block_t identity(unsigned rev)
{
  nvm_identity_block_t id = {0};
  id.family = NVM_FAMILY_MINI;
  id.board_id = rev << 16;
  id.hw_rev_major = rev;
  id.unit_serial = 1U;
  if (rev == 3U) memcpy(id.mcu_uid, uid, sizeof(uid));
  seal(&id);
  return id;
}
static f413_machine_status_t resolve(const nvm_identity_block_t *id,
                                     f413_machine_selection_t *out)
{
  return f413_machine_resolve(NVM_STATUS_OK, id, uid, f413_boards,
      f413_board_count, f413_units, f413_unit_count, out);
}
static void expect(nvm_identity_block_t id, f413_machine_status_t status)
{
  f413_machine_selection_t out;
  memset(&out, 0xA5, sizeof(out));
  seal(&id);
  assert(resolve(&id, &out) == status);
  if (status != F413_MACHINE_OK) {
    assert(out.board == NULL && out.unit == NULL && out.profile == NULL);
  }
}
static void resolver_tests(void)
{
  nvm_identity_block_t r2 = identity(2), r3 = identity(3), bad;
  f413_machine_selection_t out;
  assert(resolve(&r2, &out) == F413_MACHINE_OK);
  assert(!out.hardware.left_forward_in2_high && out.hardware.right_forward_in2_high);
  assert(out.profile == &f413_profile_mini_r2);
  assert(resolve(&r3, &out) == F413_MACHINE_OK);
  assert(out.hardware.left_forward_in2_high && out.hardware.right_forward_in2_high);
  assert(out.profile == &f413_profile_mini_r3);
  bad = r3; bad.unit_serial = 2; expect(bad, F413_MACHINE_UNIT_UNKNOWN);
  bad = r3; bad.unit_serial = 0; expect(bad, F413_MACHINE_ID_INVALID);
  bad = r3; bad.hw_rev_minor = 1; expect(bad, F413_MACHINE_ID_INVALID);
  bad = r3; bad.board_id |= 1; expect(bad, F413_MACHINE_BOARD_UNKNOWN);
  bad = r3; bad.mcu_uid[0] ^= 1; expect(bad, F413_MACHINE_UID_MISMATCH);
  bad = r3; memset(bad.mcu_uid, 0, sizeof(bad.mcu_uid)); expect(bad, F413_MACHINE_UID_MISMATCH);
  bad = r2; bad.mcu_uid[0] = 123; expect(bad, F413_MACHINE_UID_MISMATCH);
  bad = r3; bad.default_param_profile = f413_profile_mini_r2.id; expect(bad, F413_MACHINE_PROFILE_MISMATCH);
  bad = r3; bad.default_param_profile = f413_profile_mini_r3.id; expect(bad, F413_MACHINE_OK);
  bad = r3; bad.capability_flags = 0x80000000U; expect(bad, F413_MACHINE_CONFIG_INVALID);
  bad = r2; bad.family = NVM_FAMILY_CLASSIC; expect(bad, F413_MACHINE_BOARD_UNKNOWN);
  bad = r3; bad.magic ^= 1; assert(resolve(&bad, &out) == F413_MACHINE_ID_INVALID);
  bad = r3; bad.schema_version++; assert(resolve(&bad, &out) == F413_MACHINE_ID_INVALID);
  bad = r3; bad.length--; assert(resolve(&bad, &out) == F413_MACHINE_ID_INVALID);
  bad = r3; bad.crc++; assert(resolve(&bad, &out) == F413_MACHINE_ID_INVALID);
  for (unsigned i = NVM_STATUS_INVALID_ARG; i <= NVM_STATUS_HW_ERROR; ++i)
    assert(f413_machine_resolve((nvm_status_t)i, &r3, uid, f413_boards,
        f413_board_count, f413_units, f413_unit_count, &out) == F413_MACHINE_ID_INVALID);

  /* Future classic shares board_id 0x00020000 but NOT mini geometry/profile. */
  f413_scalar_params_t scalar = *f413_profile_mini_r2.scalar;
  scalar.v_DIST_HALF_SEC = 90.0;
  scalar.v_DIST_D_HALF_SEC = 127.279;
  scalar.v_D_TIRE = 24.0;
  scalar.v_KP_VELOCITY_FAN_OFF = 0.123f;
  f413_param_profile_t profile = f413_profile_mini_r2;
  profile.family = NVM_FAMILY_CLASSIC;
  profile.scalar = &scalar;
  profile.route_precomputed_compatible = false;
  f413_board_config_t boards[2] = {f413_boards[0], f413_boards[0]};
  boards[1].family = NVM_FAMILY_CLASSIC;
  boards[1].default_profile = &profile;
  f413_unit_config_t units[3] = {f413_units[0], f413_units[0], f413_units[0]};
  units[1].family = NVM_FAMILY_CLASSIC;
  bad = r2; bad.family = NVM_FAMILY_CLASSIC; seal(&bad);
  assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 2, units, 2, &out) == F413_MACHINE_OK);
  assert(out.profile->scalar->v_DIST_HALF_SEC == 90.0);
  assert(out.profile->scalar->v_KP_VELOCITY_FAN_OFF == 0.123f);
  boards[1].default_profile = &f413_profile_mini_r2;
  assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 2, units, 2, &out) == F413_MACHINE_PROFILE_MISMATCH);
  boards[1].default_profile = &profile;
  scalar.v_DIST_HALF_SEC = 45.0;
  assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 2, units, 2, &out) == F413_MACHINE_CONFIG_INVALID);
  scalar.v_DIST_HALF_SEC = 90.0;
  boards[1].layout = 999;
  assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 2, units, 2, &out) == F413_MACHINE_LAYOUT_UNSUPPORTED);

  /* Two units of one board: full tune and hardware overrides are independent. */
  profile.family = NVM_FAMILY_MINI;
  scalar.v_DIST_HALF_SEC = 45.0;
  units[2].unit_serial = 2;
  units[2].profile_override = &profile;
  units[2].hardware_override = &f413_boards[1].hardware;
  bad = r2; bad.unit_serial = 2; seal(&bad);
  assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 1, units, 3, &out) == F413_MACHINE_OK);
  assert(out.profile == &profile && out.profile->scalar->v_D_TIRE == 24.0);
  assert(resolve(&r2, &out) == F413_MACHINE_OK && out.profile == &f413_profile_mini_r2);
  scalar.v_D_TIRE = NAN;
  assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 1, units, 3, &out) == F413_MACHINE_CONFIG_INVALID);
}

int main(int argc, char **argv)
{
  assert(argc == 2);
  resolver_tests();
  assert(!f413_machine_has(F413_CAP_DRIVE));
  assert(f413_motor_pwm_encode(true, true, 120).compare == 0);
  unsigned rev = (unsigned)atoi(argv[1]);
  nvm_identity_block_t id = identity(rev);
  if (rev == 0U) {
    assert(f413_machine_boot(NVM_STATUS_NOT_FOUND, &id, uid) == F413_MACHINE_ID_INVALID);
    id = identity(3);
    assert(f413_machine_boot(NVM_STATUS_OK, &id, uid) == F413_MACHINE_ID_INVALID);
    assert(!f413_machine_has(F413_CAP_DRIVE | F413_CAP_FAN));
  } else {
    assert(f413_machine_boot(NVM_STATUS_OK, &id, uid) == F413_MACHINE_OK);
    assert(f413_machine_has(F413_CAP_DRIVE));
    assert(DIST_HALF_SEC == 45.0 && D_TIRE == 14.13);
    assert(KP_VELOCITY_FAN_OFF == 0.8f);
    assert(searchRunParams[0].velocity_turn90 == 300.0f);
    assert(shortestRunModeParams2.velocity_l_turn_90 == 500.0f);
    assert(shortestRunCaseParamsMode2[5].velocity_straight == 1000.0f);
    sensor_distance_init();
    assert(sensor_distance_lut_size_fl() == 13U);
    assert(f413_machine_has(F413_CAP_FAN) == (rev == 3U));
    assert(f413_machine_route_precomputed_compatible() == (rev == 2U));
    const f413_param_profile_t *p = rev == 2U ? &f413_profile_mini_r2 : &f413_profile_mini_r3;
    assert(memcmp(&f413_machine_params()->scalar, p->scalar, sizeof(*p->scalar)) == 0);
    assert(memcmp(f413_machine_params()->search, p->search, sizeof(f413_machine_params()->search)) == 0);
    for (unsigned m = 0; m < 6; ++m) {
      assert(memcmp(&f413_machine_params()->modes[m], p->modes[m], sizeof(*p->modes[m])) == 0);
      assert(memcmp(f413_machine_params()->cases[m], p->cases[m], sizeof(f413_machine_params()->cases[m])) == 0);
    }
    assert(f413_motor_pwm_encode(true, true, 120).in2_high == (rev == 3U));
    assert(f413_motor_pwm_encode(false, true, 120).in2_high);
    assert(!f413_motor_pwm_encode(false, false, 120).in2_high);
    assert(f413_motor_pwm_encode(true, false, 0).compare == 0);
    /* A second boot call cannot switch a live controller's settings. */
    id = identity(rev == 2U ? 3U : 2U);
    assert(f413_machine_boot(NVM_STATUS_OK, &id, uid) == F413_MACHINE_OK);
    assert(f413_machine_profile_id() == p->id);
  }
  printf("PASS: resolver, mini/classic collision, unit overrides, fail-closed, boot %u\n", rev);
  return 0;
}
