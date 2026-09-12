#include "f413_machine.h"
#include "f413_motor_pwm.h"
#include "f413_measurements.h"
#include "f413_wall_distance.h"
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
/* Real persistence/warp acceptance is covered by f413_nvm_params_tests. */
bool nvm_params_distance_load_and_apply(void) { return false; }
bool f413_wall_sensor_read_snapshot(f413_wall_sensor_snapshot_t *out)
{ (void)out; return false; }

static void front_distance_tests(unsigned rev)
{
  assert(f413_machine_front_distance_body_centre() == (rev == 3U));
  assert(F_ALIGN_TARGET_MM == (rev == 3U ? 45.0f : 7.0f));
  assert(F_ALIGN_TOO_CLOSE_MM == F_ALIGN_TARGET_MM - 2.5f);
  f413_wall_distance_init();
  assert(sensor_distance_get_interpolation() == SENSOR_DISTANCE_INTERP_PCHIP);
  assert(sensor_distance_lut_size_fl() == (rev == 3U ? 9U : 13U));
  if (rev == 2U) {
    assert(fabsf(sensor_distance_from_fr(1680) - 7.0f) < 0.001f);
    assert(fabsf(sensor_distance_from_fl(2050) - 7.0f) < 0.001f);
    assert(fabsf(sensor_distance_from_fsum(3730) - 7.0f) < 0.001f);
    return;
  }
  /* Independent transcription after optical shielding, 2026-09-12. */
  const uint16_t fr[] = {3048,2508,1936,1507,1196,969,797,669,565};
  const uint16_t fl[] = {3043,2494,1924,1516,1231,1004,835,699,595};
  assert(sensor_distance_lut_size_fr() == 9U);
  assert(sensor_distance_lut_size_front_sum() == 9U);
  f413_wall_sensor_snapshot_t adc = {.front_wall = true};
  f413_wall_distance_snapshot_t distance;
  const unsigned front_mask = F413_WALL_DISTANCE_CH_FR | F413_WALL_DISTANCE_CH_FL | F413_WALL_DISTANCE_CH_FSUM;
  for (unsigned i = 0; i < 9; ++i) {
    adc.fr_delta = fr[i]; adc.fl_delta = fl[i];
    /* Synthetic unsaturated raw ADC, independent of persisted offsets. */
    adc.fr_on = fr[i] + 100; adc.fl_on = fl[i] + 100;
    assert(f413_wall_distance_convert_snapshot(&adc, &distance));
    const float mm = 40.0f + 5.0f * i;
    assert(fabsf(distance.fr_mm - mm) < 0.001f);
    assert(fabsf(distance.fl_mm - mm) < 0.001f);
    assert(fabsf(distance.front_sum_mm - mm) < 0.001f);
    assert(distance.fr_mm == distance.fr_mm_unwarped);
    assert(distance.fl_mm == distance.fl_mm_unwarped);
    assert(distance.front_sum_mm == distance.front_sum_mm_unwarped);
    assert(f413_wall_distance_front_present(&distance));
    assert((distance.valid_mask & front_mask) == front_mask);
  }
  /* Every in-range integer ADC value: monotone and bounded PCHIP. */
  float (*convert[])(uint16_t) = {sensor_distance_from_fr, sensor_distance_from_fl, sensor_distance_from_fsum};
  bool (*in_range[])(uint16_t) = {sensor_distance_ad_in_range_fr, sensor_distance_ad_in_range_fl, sensor_distance_ad_in_range_fsum};
  const uint16_t low[] = {565,595,1160}, high[] = {3048,3043,6091};
  for (unsigned ch = 0; ch < 3; ++ch) {
    float previous = 80.0f;
    for (unsigned ad = low[ch]; ad <= high[ch]; ++ad) {
      const float mm = convert[ch]((uint16_t)ad);
      assert(isfinite(mm) && mm >= 40.0f && mm <= previous);
      assert(in_range[ch]((uint16_t)ad));
      previous = mm;
    }
    assert(!in_range[ch](0) && !in_range[ch](low[ch] - 1));
    assert(!in_range[ch](high[ch] + 1) && !in_range[ch](UINT16_MAX));
  }
  /* No wall, beyond either endpoint, and saturated raw ADC cannot be trusted. */
  const int32_t invalid[][2] = {{0,0}, {564,594}, {3049,3044}, {-1,-1},
                              {458,478}, {198,209}}; /* Old 85/110 mm data must not extend the new LUT. */
  for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
    adc.fr_delta = invalid[i][0]; adc.fl_delta = invalid[i][1];
    assert(f413_wall_distance_convert_snapshot(&adc, &distance));
    assert(!f413_wall_distance_front_present(&distance));
    assert((distance.extrapolated_mask & front_mask) == front_mask);
  }
  adc.fr_delta = 2508; adc.fl_delta = 2494; adc.fr_on = 4090;
  assert(f413_wall_distance_convert_snapshot(&adc, &distance));
  assert(!f413_wall_distance_front_present(&distance));
  /* Loading the legacy front-only r2 table leaves side conversion untouched. */
  const float left = sensor_distance_from_l(1000), right = sensor_distance_from_r(1000);
  f413_profile_mini_r2.load_sensor_luts();
  assert(left == sensor_distance_from_l(1000) && right == sensor_distance_from_r(1000));
  f413_profile_mini_r3.load_sensor_luts();
}

static void side_distance_tests(unsigned rev)
{
  assert(f413_machine_side_distance_body_centre() == (rev == 3U));
  f413_wall_distance_init();
  assert(WALL_BASE_R == 300 && WALL_BASE_L == 300);
  if (rev == 2U) {
    assert(sensor_distance_lut_size_r() == 28U && sensor_distance_lut_size_l() == 28U);
    assert(fabsf(sensor_distance_from_r(576) - 20.0f) < 0.001f);
    assert(fabsf(sensor_distance_from_l(532) - 20.0f) < 0.001f);
    return;
  }
  const uint16_t mm[] = {23,30,35,40,45,50,55,60,65,70,75,80};
  const uint16_t right[] = {2507,1572,1138,848,640,492,387,316,253,211,176,150};
  const uint16_t left[] = {2919,1905,1361,990,733,567,448,361,298,249,210,176};
  assert(sensor_distance_lut_size_r() == 12U && sensor_distance_lut_size_l() == 12U);
  f413_wall_sensor_snapshot_t adc = {.right_wall = true, .left_wall = true};
  f413_wall_distance_snapshot_t distance;
  const unsigned side_mask = F413_WALL_DISTANCE_CH_R | F413_WALL_DISTANCE_CH_L;
  for (unsigned i = 0; i < 12; ++i) {
    adc.r_delta = right[i]; adc.l_delta = left[i];
    adc.r_on = right[i] + 100; adc.l_on = left[i] + 100;
    assert(f413_wall_distance_convert_snapshot(&adc, &distance));
    assert(fabsf(distance.r_mm - mm[i]) < 0.001f);
    assert(fabsf(distance.l_mm - mm[i]) < 0.001f);
    assert((distance.extrapolated_mask & side_mask) == 0U);
    assert(distance.right_valid == (right[i] > WALL_BASE_R));
    assert(distance.left_valid == (left[i] > WALL_BASE_L));
    assert(f413_wall_distance_side_present(&distance, true) == distance.right_valid);
    assert(f413_wall_distance_side_present(&distance, false) == distance.left_valid);
  }
  float (*convert[])(uint16_t) = {sensor_distance_from_r, sensor_distance_from_l};
  bool (*in_range[])(uint16_t) = {sensor_distance_ad_in_range_r, sensor_distance_ad_in_range_l};
  const uint16_t low[] = {150,176}, high[] = {2507,2919};
  for (unsigned ch = 0; ch < 2; ++ch) {
    float previous = 80.0f;
    for (unsigned ad = low[ch]; ad <= high[ch]; ++ad) {
      const float value = convert[ch]((uint16_t)ad);
      assert(isfinite(value) && value >= 23.0f && value <= previous);
      assert(in_range[ch]((uint16_t)ad));
      previous = value;
    }
    assert(!in_range[ch](0) && !in_range[ch](low[ch] - 1));
    assert(!in_range[ch](high[ch] + 1) && !in_range[ch](UINT16_MAX));
  }
  const int32_t invalid[][2] = {{0,0}, {149,175}, {2508,2920}, {-1,-1}};
  for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
    adc.r_delta = invalid[i][0]; adc.l_delta = invalid[i][1];
    assert(f413_wall_distance_convert_snapshot(&adc, &distance));
    assert(!distance.right_valid && !distance.left_valid);
    assert((distance.extrapolated_mask & side_mask) == side_mask);
  }
  adc.r_delta = 640; adc.l_delta = 733; adc.r_on = 4090;
  assert(f413_wall_distance_convert_snapshot(&adc, &distance));
  assert(!distance.right_valid && distance.left_valid);
  adc.r_on = 640 + 100; adc.l_on = 4090;
  assert(f413_wall_distance_convert_snapshot(&adc, &distance));
  assert(distance.right_valid && !distance.left_valid);
  /* A LUT update must not silently relax the existing low-signal safety gate. */
  adc.r_on = adc.l_on = 1000;
  adc.r_delta = 300; adc.l_delta = 301;
  assert(f413_wall_distance_convert_snapshot(&adc, &distance));
  assert(!distance.right_valid && distance.left_valid);
}

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
  assert(out.hardware.imu_forward_accel_sign == 1 && out.hardware.imu_forward_offset_mm == 0.0f);
  assert(resolve(&r3, &out) == F413_MACHINE_OK);
  assert(out.hardware.left_forward_in2_high && out.hardware.right_forward_in2_high);
  assert(out.profile == &f413_profile_mini_r3);
  assert(out.hardware.imu_forward_accel_reg == 0x2AU);
  assert(out.hardware.imu_forward_accel_sign == -1 && out.hardware.imu_forward_offset_mm == -2.5f);
  assert(f413_boards[1].hardware.imu_forward_accel_sign == -1);
  assert(out.hardware.encoder_cpr == 200.0f && out.hardware.tread_mm == 34.5f);
  /* 180 deg yaw mounting reverses X/Y but not Z; only accel Y changes here.
     At omega=10 rad/s, a rear IMU sees +250 mm/s2 from centripetal motion. */
  for (int direction = -1; direction <= 1; direction += 2) {
    float dps = direction * 572.9577951308232f;
    assert(fabsf(f413_imu_centre_forward_accel(350.0f, dps, -2.5f) - 100.0f) < 0.001f);
    assert(fabsf(f413_imu_centre_forward_accel(-150.0f, dps, 2.5f) - 100.0f) < 0.001f);
    assert(f413_imu_centre_forward_accel(12.5f, dps, 0.0f) == 12.5f);
  }
  assert(f413_imu_centre_forward_accel(12.5f, 0.0f, -2.5f) == 12.5f);
  assert(fabsf(f413_battery_voltage(2111U, 3.3f, out.hardware.battery_divider_ratio) - 8.0f) < 0.01f);
  assert(fabsf(f413_battery_voltage(3324U, 3.3f, out.hardware.battery_divider_ratio) - 12.6f) < 0.01f);
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
  scalar.v_D_TIRE = 24.0;
  f413_hardware_config_t hw = f413_boards[1].hardware;
  units[2].hardware_override = &hw;
  hw.imu_forward_offset_mm = NAN;
  assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 1, units, 3, &out) == F413_MACHINE_CONFIG_INVALID);
  hw.imu_forward_offset_mm = -2.5f;
  hw.battery_divider_ratio = 0.0f;
  assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 1, units, 3, &out) == F413_MACHINE_CONFIG_INVALID);
}

int main(int argc, char **argv)
{
  assert(argc == 2);
  resolver_tests();
  assert(!f413_machine_has(F413_CAP_DRIVE));
  assert(!f413_machine_side_distance_body_centre());
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
    front_distance_tests(rev);
    side_distance_tests(rev);
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
