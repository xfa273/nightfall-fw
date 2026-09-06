#include "f413_machine.h"

/* Model defaults describe standard wiring, NOT one particular reworked unit. */
#define MINI_R2_HW {false, true, 1, -1, 0U, 200.0f, 34.5f, 0x2AU, 1, 0.0f, 3.2f}
/* U4: F.Cu 180 degrees versus r2's 0 degrees. Z is unchanged, Y reverses.
   User confirms the same chassis/drivetrain and a 2.5 mm rearward IMU offset.
   R4/R5: 100k/27k, versus r2's 22k/10k battery divider. */
#define MINI_R3_HW(left_swapped) {left_swapped, true, 1, -1, 0U, 200.0f, 34.5f, 0x2AU, -1, -2.5f, (127.0f / 27.0f)}
#define MINI_CAPS (F413_CAP_DRIVE | F413_CAP_IMU | F413_CAP_WALL | F413_CAP_FRAM)
const f413_board_config_t f413_boards[] = {
  {NVM_FAMILY_MINI, 0x00020000U, F413_LAYOUT_MINI_R2, MINI_CAPS,
   "mini_r2_0", MINI_R2_HW, &f413_profile_mini_r2},
  {NVM_FAMILY_MINI, 0x00030000U, F413_LAYOUT_MINI_R2, MINI_CAPS | F413_CAP_FAN,
   "mini_r3_0", MINI_R3_HW(false), &f413_profile_mini_r3},
  /* Future classic: register only after its pin contract and parameters are known.
     The resolver always keys by (family, board_id), never board_id alone. */
};
const size_t f413_board_count = sizeof(f413_boards) / sizeof(f413_boards[0]);

static const f413_hardware_config_t mini_r3_unit001_hw =
    MINI_R3_HW(true);
const f413_unit_config_t f413_units[] = {
  /* Legacy r2 identity allowed zero UID; a bound UID is always checked. */
  {NVM_FAMILY_MINI, 0x00020000U, 1U, false, NULL, NULL},
  /* Left leads physically swapped on 2026-09-06. R34/R35 = 0 ohm. */
  {NVM_FAMILY_MINI, 0x00030000U, 1U, true, &mini_r3_unit001_hw, NULL},
};
const size_t f413_unit_count = sizeof(f413_units) / sizeof(f413_units[0]);
