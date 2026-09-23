#include "f413_machine.h"
#include "f413_motor_pwm.h"
#include "f413_measurements.h"
#include "f413_wall_distance.h"
#include "f413_motion_stop.h"
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
static bool s_test_wall_available;
static f413_wall_sensor_snapshot_t s_test_wall;
bool f413_wall_sensor_read_snapshot(f413_wall_sensor_snapshot_t *out)
{
  if (!s_test_wall_available || out == NULL) return false;
  *out = s_test_wall;
  return true;
}

static void front_distance_tests(unsigned rev)
{
  assert(f413_machine_front_distance_body_centre() == (rev == 3U));
  assert(F_ALIGN_TARGET_MM == (rev == 3U ? 45.0f : 7.0f));
  assert(F_ALIGN_TOO_CLOSE_MM == F_ALIGN_TARGET_MM - 2.5f);
  f413_wall_distance_init();
  assert(sensor_distance_get_interpolation() == SENSOR_DISTANCE_INTERP_PCHIP);
  assert(sensor_distance_lut_size_fl() == (rev == 3U ? 15U : 13U));
  if (rev == 2U) {
    assert(fabsf(sensor_distance_from_fr(1680) - 7.0f) < 0.001f);
    assert(fabsf(sensor_distance_from_fl(2050) - 7.0f) < 0.001f);
    assert(fabsf(sensor_distance_from_fsum(3730) - 7.0f) < 0.001f);
    return;
  }
  /* Shielded40..75mm (09-12), latest80..110mm (09-21); no averaging
   * of the superseded80mm point and no reuse of pre-shielding data. */
  const uint16_t fr[] = {3048,2508,1936,1507,1196,969,797,669,599,504,422,357,306,260,223};
  const uint16_t fl[] = {3043,2494,1924,1516,1231,1004,835,699,568,488,422,364,312,269,233};
  assert(sensor_distance_lut_size_fr() == 15U);
  assert(sensor_distance_lut_size_front_sum() == 15U);
  f413_wall_sensor_snapshot_t adc = {.front_wall = true};
  f413_wall_distance_snapshot_t distance;
  const unsigned front_mask = F413_WALL_DISTANCE_CH_FR | F413_WALL_DISTANCE_CH_FL | F413_WALL_DISTANCE_CH_FSUM;
  for (unsigned i = 0; i < 15; ++i) {
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
  const uint16_t low[] = {223,233,456}, high[] = {3048,3043,6091};
  for (unsigned ch = 0; ch < 3; ++ch) {
    float previous = 110.0f;
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
  const int32_t invalid[][2] = {{0,0}, {222,232}, {3049,3044}, {-1,-1},
                              {198,209}}; /* Beyond the new measured far endpoint. */
  for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
    adc.fr_delta = invalid[i][0]; adc.fl_delta = invalid[i][1];
    assert(f413_wall_distance_convert_snapshot(&adc, &distance));
    assert(!f413_wall_distance_front_present(&distance));
    assert((distance.extrapolated_mask & front_mask) == front_mask);
  }
  adc.fr_delta = 2508; adc.fl_delta = 2494; adc.fr_on = 4090;
  assert(f413_wall_distance_convert_snapshot(&adc, &distance));
  assert(!f413_wall_distance_front_present(&distance));
  adc.fr_on = 2608; adc.fl_on = 4090;
  assert(f413_wall_distance_convert_snapshot(&adc, &distance));
  assert(!f413_wall_distance_front_present(&distance));
  /* Nominal90mm entry and the82mm turn target are now in the measured range.
   * Integer ADC pairs around the interpolated target must bracket its crossing. */
  adc.fr_on = adc.fl_on = 1000;
  adc.fr_delta = adc.fl_delta = 422;
  assert(f413_wall_distance_convert_snapshot(&adc, &distance));
  assert(f413_wall_distance_front_present(&distance));
  assert(fabsf(distance.front_sum_mm_unwarped - 90.0f) < 0.001f);
  s_test_wall_available = true;
  s_test_wall = adc;
  float entry_distance;
  assert(f413_wall_distance_front_unwarped_mm(&entry_distance));
  assert(fabsf(entry_distance - 90.0f) < 0.001f);
  bool crossed82 = false;
  for (unsigned sum = 992; sum <= 1167; ++sum) {
    /* Interpolate ADC ratio only to construct unsaturated test input. */
    adc.fr_delta = 504 + (int32_t)((sum - 992) * 95 / 175);
    adc.fl_delta = (int32_t)sum - adc.fr_delta;
    assert(f413_wall_distance_convert_snapshot(&adc, &distance));
    assert(f413_wall_distance_front_present(&distance));
    if (distance.front_sum_mm_unwarped <= 82.0f) {
      crossed82 = true;
    } else {
      assert(!crossed82);
    }
  }
  assert(crossed82);
  /* All three failed-run snapshots must hand off to recovery, not abort. */
  const int32_t stop_adc[][2] = {{2508,2494}, {3086,3021}, {2901,3174}, {2862,2696}};
  for (unsigned i = 0; i < 4; ++i) {
    adc.fr_delta = stop_adc[i][0]; adc.fl_delta = stop_adc[i][1];
    adc.fr_on = (uint16_t)(adc.fr_delta + 100);
    adc.fl_on = (uint16_t)(adc.fl_delta + 100);
    assert(f413_wall_distance_convert_snapshot(&adc, &distance));
    f413_stop_state_t state = {0};
    const f413_stop_action_t action = f413_stop_approach_step(&state, 6.4f, 3.0f,
        true, true, distance.front_valid, distance.saturated_mask != 0,
        distance.fr_mm_unwarped, distance.fl_mm_unwarped,
        distance.front_sum_mm_unwarped, F_ALIGN_TARGET_MM);
    assert(action == F413_STOP_BRAKE);
  }
  /* One invalid channel must still reject a plausible front sum. */
  adc.fr_delta = 222; adc.fl_delta = 500;
  assert(f413_wall_distance_convert_snapshot(&adc, &distance));
  assert(!f413_wall_distance_front_present(&distance));
  s_test_wall = adc;
  assert(!f413_wall_distance_front_unwarped_mm(&entry_distance));
  s_test_wall_available = false;
  /* Loading the legacy front-only r2 table leaves side conversion untouched. */
  const float left = sensor_distance_from_l(1000), right = sensor_distance_from_r(1000);
  f413_profile_mini_r2.load_sensor_luts();
  assert(left == sensor_distance_from_l(1000) && right == sensor_distance_from_r(1000));
  f413_profile_mini_r3.load_sensor_luts();
}

/* Audit the existing distance-domain turn references; raw val_offset_in is
 * not consumed by the F413 entry correction. This does not qualify floor use. */
static void front_entry_reference_tests(void)
{
  const f413_param_profile_t *r2 = &f413_profile_mini_r2;
  const f413_param_profile_t *r3 = &f413_profile_mini_r3;
  const float datum_delta = r3->scalar->v_F_ALIGN_TARGET_MM - r2->scalar->v_F_ALIGN_TARGET_MM;
  assert(datum_delta == 38.0f);
  assert(r2->scalar->v_DIST_HALF_SEC == r3->scalar->v_DIST_HALF_SEC);
  const float search_targets[] = {82.0f, 80.0f};
  const float mode_targets[] = {82.8f, 89.4f, 89.2f, 89.1f, 89.1f, 88.6f};
  for (unsigned i = 0; i < 2; ++i) {
    assert(r2->search[i].dist_offset_in == r3->search[i].dist_offset_in);
    const float old_target = r2->scalar->v_F_ALIGN_TARGET_MM +
        (float)r2->scalar->v_DIST_HALF_SEC - r2->search[i].dist_offset_in;
    const float new_target = r3->scalar->v_F_ALIGN_TARGET_MM +
        (float)r3->scalar->v_DIST_HALF_SEC - r3->search[i].dist_offset_in;
    assert(fabsf(new_target - old_target - datum_delta) < 0.001f);
    assert(fabsf(new_target - search_targets[i]) < 0.001f);
  }
  const float small_v[] = {300,800,1000,1200,1200,1400};
  const float large_v[] = {500,1000,1400,1700,2000,2200};
  const float diagonal_v[] = {500,1000,1400,1500,1500,1500};
  const int fan[] = {0,500,700,1000,1000,1000};
  for (unsigned i = 0; i < 6; ++i) {
    assert(r2->modes[i]->turn_omega_max == 0);
    assert(r3->modes[i]->turn_omega_max == (i == 0 ? 0 : 3000));
    if (i == 0) {
      assert(memcmp(r2->modes[i], r3->modes[i], sizeof(*r2->modes[i])) == 0);
    } else {
      assert(r2->modes[i]->fan_power == 0 && r3->modes[i]->fan_power == fan[i]);
      assert(r3->modes[i]->velocity_turn90 == small_v[i]);
      assert(r3->modes[i]->velocity_l_turn_90 == large_v[i]);
      assert(r3->modes[i]->velocity_l_turn_180 == large_v[i]);
      assert(r3->modes[i]->velocity_turn45in == diagonal_v[i]);
      assert(r3->modes[i]->velocity_turn45out == diagonal_v[i]);
      assert(r3->modes[i]->velocity_turnV90 == diagonal_v[i]);
      assert(r3->modes[i]->velocity_turn135in == diagonal_v[i]);
      assert(r3->modes[i]->velocity_turn135out == diagonal_v[i]);
    }
    const float old_target = r2->scalar->v_F_ALIGN_TARGET_MM +
        (float)r2->scalar->v_DIST_HALF_SEC - r2->modes[i]->dist_offset_in;
    const float new_target = r3->scalar->v_F_ALIGN_TARGET_MM +
        (float)r3->scalar->v_DIST_HALF_SEC - r3->modes[i]->dist_offset_in;
    const float entry_change = r2->modes[i]->dist_offset_in - r3->modes[i]->dist_offset_in;
    assert(fabsf(new_target - old_target - datum_delta - entry_change) < 0.001f);
    assert(fabsf(new_target - mode_targets[i]) < 0.001f);
  }
  /* The nominal entry is90mm. The09-21 LUT extension covers both the entry
   * and all80..89.4mm turn references; the runner's validity guard is unchanged. */
  assert(r3->scalar->v_F_ALIGN_TARGET_MM + r3->scalar->v_DIST_HALF_SEC == 90.0);
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
  /* Unit002's independently confirmed left wiring matches unit001.
     Keep the model default and encoder signs unchanged. */
  nvm_identity_block_t r3_unit2 = r3;
  const uint32_t uid2[] = {0x001D0038U, 0x32345108U, 0x36383936U};
  r3_unit2.unit_serial = 2;
  memcpy(r3_unit2.mcu_uid, uid2, sizeof(uid2));
  seal(&r3_unit2);
  assert(f413_machine_resolve(NVM_STATUS_OK, &r3_unit2, uid2, f413_boards,
      f413_board_count, f413_units, f413_unit_count, &out) == F413_MACHINE_OK);
  assert(out.hardware.left_forward_in2_high && out.hardware.right_forward_in2_high);
  assert(!out.board->hardware.left_forward_in2_high);
  assert(out.hardware.encoder_sign_l == 1 && out.hardware.encoder_sign_r == -1);
  assert(out.hardware.motor_pwm_prescaler == 0U);
  assert(out.profile == &f413_profile_mini_r3);
  assert(out.hardware.imu_forward_accel_sign == -1 && out.hardware.imu_forward_offset_mm == -2.5f);
  expect(r3_unit2, F413_MACHINE_UID_MISMATCH);
  memset(r3_unit2.mcu_uid, 0, sizeof(r3_unit2.mcu_uid));
  expect(r3_unit2, F413_MACHINE_UID_MISMATCH);
  bad = r3; bad.unit_serial = 3; expect(bad, F413_MACHINE_UNIT_UNKNOWN);
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
  int32_t *coordinates[] = {&scalar.v_START_X, &scalar.v_START_Y,
      &scalar.v_GOAL_X, &scalar.v_GOAL_Y,
      &scalar.v_GOAL1_X, &scalar.v_GOAL1_Y,
      &scalar.v_GOAL2_X, &scalar.v_GOAL2_Y,
      &scalar.v_GOAL3_X, &scalar.v_GOAL3_Y,
      &scalar.v_GOAL4_X, &scalar.v_GOAL4_Y,
      &scalar.v_GOAL5_X, &scalar.v_GOAL5_Y,
      &scalar.v_GOAL6_X, &scalar.v_GOAL6_Y,
      &scalar.v_GOAL7_X, &scalar.v_GOAL7_Y,
      &scalar.v_GOAL8_X, &scalar.v_GOAL8_Y,
      &scalar.v_GOAL9_X, &scalar.v_GOAL9_Y,
  };
  for (size_t i = 0; i < sizeof(coordinates) / sizeof(coordinates[0]); ++i) {
    const int32_t original = *coordinates[i];
    *coordinates[i] = -1;
    assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 1, units, 3, &out) == F413_MACHINE_CONFIG_INVALID);
    *coordinates[i] = scalar.v_MAZE_SIZE;
    assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 1, units, 3, &out) == F413_MACHINE_CONFIG_INVALID);
    *coordinates[i] = original;
  }
  scalar.v_MAZE_SIZE = 32;
  assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 1, units, 3, &out) == F413_MACHINE_CONFIG_INVALID);
  scalar.v_MAZE_SIZE = F413_COMPILED_MAZE_SIZE;
  int32_t saved_goals[18];
  for (size_t i = 0; i < 18; ++i) {
    saved_goals[i] = *coordinates[i + 4];
    *coordinates[i + 4] = 0;
  }
  assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 1, units, 3, &out) == F413_MACHINE_CONFIG_INVALID);
  for (size_t i = 0; i < 18; ++i) *coordinates[i + 4] = saved_goals[i];
  f413_hardware_config_t hw = f413_boards[1].hardware;
  scalar.v_ENABLE_AUTO_VIDEO_CAPTURE = 2U;
  assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 1, units, 3, &out) == F413_MACHINE_CONFIG_INVALID);
  scalar.v_ENABLE_AUTO_VIDEO_CAPTURE = 1U;
  units[2].hardware_override = &hw;
  hw.imu_forward_offset_mm = NAN;
  assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 1, units, 3, &out) == F413_MACHINE_CONFIG_INVALID);
  hw.imu_forward_offset_mm = -2.5f;
  hw.battery_divider_ratio = 0.0f;
  assert(f413_machine_resolve(NVM_STATUS_OK, &bad, uid, boards, 1, units, 3, &out) == F413_MACHINE_CONFIG_INVALID);
}

int main(int argc, char **argv)
{
  assert(argc == 2 || argc == 3);
  resolver_tests();
  assert(!f413_machine_has(F413_CAP_DRIVE));
  assert(!f413_machine_side_distance_body_centre());
  assert(f413_motor_pwm_encode(true, true, 120).compare == 0);
  unsigned rev = (unsigned)atoi(argv[1]);
  nvm_identity_block_t id = identity(rev);
  const unsigned unit = argc == 3 ? (unsigned)atoi(argv[2]) : 1U;
  id.unit_serial = unit;
  seal(&id);
  if (rev == 0U) {
    assert(f413_machine_boot(NVM_STATUS_NOT_FOUND, &id, uid) == F413_MACHINE_ID_INVALID);
    id = identity(3);
    assert(f413_machine_boot(NVM_STATUS_OK, &id, uid) == F413_MACHINE_ID_INVALID);
    assert(!f413_machine_has(F413_CAP_DRIVE | F413_CAP_FAN));
    assert(ENABLE_AUTO_VIDEO_CAPTURE == 0U);
  } else {
    assert(f413_machine_boot(NVM_STATUS_OK, &id, uid) == F413_MACHINE_OK);
    assert(f413_machine_has(F413_CAP_DRIVE));
    assert(ENABLE_AUTO_VIDEO_CAPTURE == (rev == 2U ? 1U : 0U));
    assert(DIST_HALF_SEC == 45.0);
    assert(KP_VELOCITY_FAN_OFF == (rev == 3U ? 0.24f : 0.8f));
    assert(KI_VELOCITY_FAN_OFF == (rev == 3U ? 0.001f : 0.012f));
    assert(FF_TRANSLATION_STATIC_PWM_FAN_OFF == (rev == 3U ? 35.0f : 45.0f));
    assert(KP_DISTANCE_FAN_OFF == (rev == 3U ? 2.0f : 6.0f));
    assert(KI_DISTANCE_FAN_OFF == (rev == 3U ? 0.0f : 0.05f));
    assert(KP_OMEGA_FAN_OFF == (rev == 3U ? 0.45f : 1.35f));
    if (rev == 3U) {
      assert(KP_VELOCITY_FAN_ON == f413_profile_mini_r3.scalar->v_KP_VELOCITY_FAN_ON);
      assert(KI_VELOCITY_FAN_ON == f413_profile_mini_r3.scalar->v_KI_VELOCITY_FAN_ON);
      assert(KD_VELOCITY_FAN_ON == f413_profile_mini_r3.scalar->v_KD_VELOCITY_FAN_ON);
      assert(FF_TRANSLATION_STATIC_PWM_FAN_ON == f413_profile_mini_r3.scalar->v_FF_TRANSLATION_STATIC_PWM_FAN_ON);
      assert(FF_TRANSLATION_VELOCITY_PWM_FAN_ON == f413_profile_mini_r3.scalar->v_FF_TRANSLATION_VELOCITY_PWM_FAN_ON);
      assert(FF_TRANSLATION_ACCEL_PWM_FAN_ON == f413_profile_mini_r3.scalar->v_FF_TRANSLATION_ACCEL_PWM_FAN_ON);
      assert(KP_DISTANCE_FAN_ON == f413_profile_mini_r3.scalar->v_KP_DISTANCE_FAN_ON);
      assert(KI_DISTANCE_FAN_ON == f413_profile_mini_r3.scalar->v_KI_DISTANCE_FAN_ON);
      assert(KD_DISTANCE_FAN_ON == f413_profile_mini_r3.scalar->v_KD_DISTANCE_FAN_ON);
      assert(FF_DISTANCE_FAN_ON == f413_profile_mini_r3.scalar->v_FF_DISTANCE_FAN_ON);
      assert(KP_ANGLE_FAN_ON == f413_profile_mini_r3.scalar->v_KP_ANGLE_FAN_ON);
      assert(KI_ANGLE_FAN_ON == f413_profile_mini_r3.scalar->v_KI_ANGLE_FAN_ON);
      assert(KD_ANGLE_FAN_ON == f413_profile_mini_r3.scalar->v_KD_ANGLE_FAN_ON);
      assert(FF_ANGLE_FAN_ON == f413_profile_mini_r3.scalar->v_FF_ANGLE_FAN_ON);
      assert(KP_OMEGA_FAN_ON == f413_profile_mini_r3.scalar->v_KP_OMEGA_FAN_ON);
      assert(KI_OMEGA_FAN_ON == f413_profile_mini_r3.scalar->v_KI_OMEGA_FAN_ON);
      assert(KD_OMEGA_FAN_ON == f413_profile_mini_r3.scalar->v_KD_OMEGA_FAN_ON);
      assert(FF_OMEGA_PWM_FAN_ON == f413_profile_mini_r3.scalar->v_FF_OMEGA_PWM_FAN_ON);
      assert(FF_OMEGA_ACCEL_PWM_FAN_ON == f413_profile_mini_r3.scalar->v_FF_OMEGA_ACCEL_PWM_FAN_ON);
    }
    assert(VELOCITY_ACCEL_COMP_ENABLE_CONTROL == 1U);
    assert(VELOCITY_ACCEL_COMP_ENABLE_DURING_OMEGA_PROFILE == 0U);
    assert(searchRunParams[0].velocity_turn90 == 300.0f);
    assert(shortestRunModeParams2.velocity_l_turn_90 == 500.0f);
    assert(shortestRunCaseParamsMode2[5].velocity_straight == 1000.0f);
    front_distance_tests(rev);
    front_entry_reference_tests();
    side_distance_tests(rev);
    assert(f413_machine_has(F413_CAP_FAN) == (rev == 3U));
    assert(f413_machine_route_precomputed_compatible() == (rev == 2U));
    const f413_param_profile_t *p = rev == 2U ? &f413_profile_mini_r2 : &f413_profile_mini_r3;
    /* Every public parameter must resolve to the selected profile, including
       derived values and goals. The build audit detects missing field entries. */
#define X(type, name) assert(name == p->scalar->v_##name);
#include "f413_param_fields.def"
#undef X
    assert(strcmp(PARAMS_TUNE_VERSION, p->name) == 0);
    assert(DIR_ENC_L == f413_machine_hardware()->encoder_sign_l);
    assert(DIR_ENC_R == f413_machine_hardware()->encoder_sign_r);
    assert(DIR_FWD_L == f413_machine_hardware()->left_forward_in2_high);
    assert(DIR_FWD_R == f413_machine_hardware()->right_forward_in2_high);
    assert(DIR_BACK_L != DIR_FWD_L && DIR_BACK_R != DIR_FWD_R);
    assert(memcmp(&f413_machine_params()->scalar, p->scalar, sizeof(*p->scalar)) == 0);
    assert(memcmp(f413_machine_params()->search, p->search, sizeof(f413_machine_params()->search)) == 0);
    for (unsigned m = 0; m < 6; ++m) {
      assert(memcmp(&f413_machine_params()->modes[m], p->modes[m], sizeof(*p->modes[m])) == 0);
      assert(memcmp(f413_machine_params()->cases[m], p->cases[m], sizeof(f413_machine_params()->cases[m])) == 0);
    }
    assert(f413_motor_pwm_encode(true, true, 120).in2_high == (rev == 3U));
    assert(f413_motor_pwm_encode(true, false, 120).in2_high == (rev != 3U));
    assert(f413_motor_pwm_encode(true, true, 120).compare == (rev == 3U ? 880U : 120U));
    assert(f413_machine_hardware()->encoder_sign_l == 1 &&
           f413_machine_hardware()->encoder_sign_r == -1);
    assert(f413_motor_pwm_encode(false, true, 120).in2_high);
    assert(!f413_motor_pwm_encode(false, false, 120).in2_high);
    assert(f413_motor_pwm_encode(true, false, 0).compare == 0);
    /* A second boot call cannot switch a live controller's settings. */
    id = identity(rev == 2U ? 3U : 2U);
    assert(f413_machine_boot(NVM_STATUS_OK, &id, uid) == F413_MACHINE_OK);
    assert(f413_machine_profile_id() == p->id);
  }
  printf("PASS: resolver, mini/classic collision, unit overrides, fail-closed, boot %u unit %u\n", rev, unit);
  return 0;
}
