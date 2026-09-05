#ifndef F413_MACHINE_H
#define F413_MACHINE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "nvm_identity.h"
#include "search_run_params.h"
#include "shortest_run_params.h"

#define F413_LAYOUT_MINI_R2 (1U) /* Current CubeMX pin/peripheral contract. */
#define F413_CAP_DRIVE (1U << 0)
#define F413_CAP_FAN (1U << 1)
#define F413_CAP_IMU (1U << 2)
#define F413_CAP_WALL (1U << 3)
#define F413_CAP_FRAM (1U << 4)

typedef struct {
#define X(type, name) type v_##name;
#include "f413_param_fields.def"
#undef X
} f413_scalar_params_t;

typedef struct {
  f413_scalar_params_t scalar;
  SearchRunParams_t search[2];
  ShortestRunModeParams_t modes[6];
  ShortestRunCaseParams_t cases[6][9];
} f413_runtime_params_t;

typedef struct {
  uint32_t id;
  uint32_t family;
  uint32_t board_id;
  const char *name;
  const f413_scalar_params_t *scalar;
  const SearchRunParams_t *search;
  const ShortestRunModeParams_t *modes[6];
  const ShortestRunCaseParams_t *cases[6];
  bool route_precomputed_compatible;
  void (*load_sensor_luts)(void);
} f413_param_profile_t;

typedef struct {
  bool left_forward_in2_high;
  bool right_forward_in2_high;
  int8_t encoder_sign_l;
  int8_t encoder_sign_r;
  uint16_t motor_pwm_prescaler; /* ARR stays 1000: command units do not change. */
  float encoder_cpr;
  float tread_mm;
  uint8_t imu_forward_accel_reg;
  int8_t imu_forward_accel_sign;
} f413_hardware_config_t;

typedef struct {
  uint32_t family;
  uint32_t board_id;
  uint32_t layout;
  uint32_t capabilities;
  const char *name;
  f413_hardware_config_t hardware;
  const f413_param_profile_t *default_profile;
} f413_board_config_t;

typedef struct {
  uint32_t family;
  uint32_t board_id;
  uint32_t unit_serial;
  bool require_uid;
  const f413_hardware_config_t *hardware_override;
  const f413_param_profile_t *profile_override;
} f413_unit_config_t;

typedef enum {
  F413_MACHINE_NOT_SELECTED = 0,
  F413_MACHINE_OK,
  F413_MACHINE_ID_INVALID,
  F413_MACHINE_BOARD_UNKNOWN,
  F413_MACHINE_LAYOUT_UNSUPPORTED,
  F413_MACHINE_UNIT_UNKNOWN,
  F413_MACHINE_UID_MISMATCH,
  F413_MACHINE_PROFILE_MISMATCH,
  F413_MACHINE_CONFIG_INVALID
} f413_machine_status_t;

typedef struct {
  const f413_board_config_t *board;
  const f413_unit_config_t *unit;
  const f413_param_profile_t *profile;
  f413_hardware_config_t hardware;
} f413_machine_selection_t;

/* Pure resolver; no HAL, NVM write, output enable, or runtime mutation. */
f413_machine_status_t f413_machine_resolve(
    nvm_status_t identity_status, const nvm_identity_block_t *id,
    const uint32_t actual_uid[3],
    const f413_board_config_t *boards, size_t board_count,
    const f413_unit_config_t *units, size_t unit_count,
    f413_machine_selection_t *out);

/* Boot-only, before board GPIO and interrupts. Selection is immutable until reset. */
f413_machine_status_t f413_machine_boot(nvm_status_t status,
    const nvm_identity_block_t *id, const uint32_t actual_uid[3]);
f413_machine_status_t f413_machine_status(void);
const char *f413_machine_status_name(f413_machine_status_t status);
bool f413_machine_has(uint32_t capabilities);
const f413_hardware_config_t *f413_machine_hardware(void);
const f413_runtime_params_t *f413_machine_params(void);
const char *f413_machine_profile_name(void);
uint32_t f413_machine_profile_id(void);
bool f413_machine_route_precomputed_compatible(void);

extern const f413_board_config_t f413_boards[];
extern const size_t f413_board_count;
extern const f413_unit_config_t f413_units[];
extern const size_t f413_unit_count;
extern const f413_param_profile_t f413_profile_mini_r2;
extern const f413_param_profile_t f413_profile_mini_r3;

#endif
