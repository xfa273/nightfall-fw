#include "shortest_run_params.h"
#include "solver_params.h"

/* mini r3: mode2 is unchanged; suction ladder mode3..7 is documented in
 * docs/MINI_R3_SUCTION_MODE_LADDER.md. Diagonal tuning is operationally deferred. */

// ========================= Mode 2 =========================
const ShortestRunModeParams_t shortestRunModeParams2 = {
    // 90deg
    .velocity_turn90 = 300.0f,
    .alpha_turn90 = 10200.0f,
    .acceleration_turn = 0.0f,
    .dist_offset_in = 7.2f,
    .dist_offset_out = 11.1f,
    .val_offset_in = 630.0f,  // F413 uses calibrated distance; compatibility only
    .fwall_kx = 1.1f,
    .angle_turn_90 = 90.0f,
    // Large 90deg
    .velocity_l_turn_90 = 500.0f,
    .alpha_l_turn_90    = 6900.0f,
    .angle_l_turn_90    = 90.0f,
    .dist_l_turn_in_90  = 13.0f,
    .dist_l_turn_out_90 = 20.5f,
    // Large 180deg
    .velocity_l_turn_180= 500.0f,
    .alpha_l_turn_180   = 4150.0f,
    .angle_l_turn_180   = 180.0f,
    .dist_l_turn_in_180 = 6.5f,
    .dist_l_turn_out_180= 20.0f,
    // 45deg In
    .velocity_turn45in   = 500.0f,
    .alpha_turn45in      = 11200.0f,
    .angle_turn45in      = 44.6f,
    .dist_turn45in_in    = 5.5f,
    .dist_turn45in_out   = 29.6f,
    // 45deg Out
    .velocity_turn45out= 500.0f,
    .alpha_turn45out   = 9500.0f,
    .angle_turn45out   = 44.4f,
    .dist_turn45out_in = 24.0f,
    .dist_turn45out_out= 6.0f,
    // V90deg
    .velocity_turnV90  = 500.0f,
    .alpha_turnV90     = 17500.0f,
    .angle_turnV90     = 88.3f,
    .dist_turnV90_in   = 17.0f,
    .dist_turnV90_out  = 17.0f,
    // 135deg In
    .velocity_turn135in = 500.0f,
    .alpha_turn135in    = 8300.0f,
    .angle_turn135in    = 133.9f,
    .dist_turn135in_in  = 17.0f,
    .dist_turn135in_out = 22.0f,
    // 135deg Out
    .velocity_turn135out = 500.0f,
    .alpha_turn135out    = 8500.0f,
    .angle_turn135out    = 133.3f,
    .dist_turn135out_in  = 12.5f,
    .dist_turn135out_out = 22.0f,
    // Fan
    .fan_power          = 0,
    // Makepath
    .makepath_type_case3 = 0,
    .makepath_type_case47= 1,
    // 壁切れ検出しきい値（ヒステリシス付き）
    .wall_end_thr_r_high = 100, .wall_end_thr_r_low = 1,
    .wall_end_thr_l_high = 100, .wall_end_thr_l_low = 1,
    // 加速度切り替え速度
    .accel_switch_velocity = 2000.0f
};

const ShortestRunCaseParams_t shortestRunCaseParamsMode2[9] = {
    // case1 (index 0): independent (initially same as former case3)
    {
        .acceleration_straight = 2000.0f, .acceleration_straight_dash = 2000.0f,
        .velocity_straight = 1500.0f, .kp_wall = 0.06f,
        .solver_profile = SOLVER_PROFILE_STANDARD
    },
    // case2 (index 1): independent (initially same as former case4)
    {
        .acceleration_straight = 2000.0f, .acceleration_straight_dash = 2000.0f,
        .velocity_straight = 1500.0f, .kp_wall = 0.06f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case3 (index 2)
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 3000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case4 (index 3)
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 3000.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case5 (index 4)
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 3500.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case6 (index 5)
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 1000.0f,
        .velocity_straight = 1000.0f, .kp_wall = 0.025f, .kp_diagonal = 0.2f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 1000.0f, .acceleration_d_straight_dash = 1000.0f,
        .velocity_d_straight = 800.0f
    },
    // case7 (index 6)
    {
        .acceleration_straight = 3000.0f, .acceleration_straight_dash = 3000.0f,
        .velocity_straight = 1250.0f, .kp_wall = 0.025f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 3000.0f, .acceleration_d_straight_dash = 3000.0f,
        .velocity_d_straight = 900.0f
    },
    // case8 (index 7): diagonal use
    {
        .acceleration_straight = 4000.0f, .acceleration_straight_dash = 4000.0f,
        .velocity_straight = 1500.0f, .kp_wall = 0.025f, .kp_diagonal = 0.2f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 4000.0f, .acceleration_d_straight_dash = 4000.0f,
        .velocity_d_straight = 1000.0f
    },
    // case9 (index 8): legacy high-speed comparison profile
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 3500.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.025f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 1000.0f, .acceleration_d_straight_dash = 3000.0f,
        .velocity_d_straight = 3000.0f
    },
};

// ========================= Mode 3 =========================
/* PR #21 ideal simulation, 1 kHz / rounding 1.2 / omega 3000 deg/s. */
const ShortestRunModeParams_t shortestRunModeParams3 = {
    /* Explicit simulator input; preserves the tuned turn geometry. */
    .turn_omega_max = 3000.0f,
    // 90deg
    .velocity_turn90 = 800.0f,
    .alpha_turn90 = 50000.0f,
    .acceleration_turn = 0.0f,
    .dist_offset_in = 0.6f,
    .dist_offset_out = 1.0f,
    // F413 front target = 45 + 45 - 0.6 = 89.4 mm (calibrated centre LUT).
    .val_offset_in = 98.0f, // Legacy F405 compatibility; unused by F413.
    .fwall_kx = 1.1f,      // Legacy F405 compatibility; unused by F413.
    .angle_turn_90 = 90.0f,
    .dist_wall_end = 0.0f, // Provisional with 350/300 thresholds; see tuning note.
    // Large 90deg
    .velocity_l_turn_90 = 1000.0f,
    .alpha_l_turn_90 = 20500.0f,
    .angle_l_turn_90 = 90.0f,
    .dist_l_turn_in_90 = 3.6f,
    .dist_l_turn_out_90 = 3.7f,
    // Large 180deg
    .velocity_l_turn_180 = 1000.0f,
    .alpha_l_turn_180 = 18000.0f,
    .angle_l_turn_180 = 180.0f,
    .dist_l_turn_in_180 = 1.0f,
    .dist_l_turn_out_180 = 1.5f,
    // Diagonal placeholders: defer use until mini_r2 validation.
    // 45deg In
    .velocity_turn45in = 1000.0f,
    .alpha_turn45in = 30500.0f,
    .angle_turn45in = 45.0f,
    .dist_turn45in_in = 0.7f,
    .dist_turn45in_out = 20.0f,
    // 45deg Out
    .velocity_turn45out = 1000.0f,
    .alpha_turn45out = 30000.0f,
    .angle_turn45out = 45.0f,
    .dist_turn45out_in = 18.9f,
    .dist_turn45out_out = 0.7f,
    // V90deg
    .velocity_turnV90 = 1000.0f,
    .alpha_turnV90 = 77000.0f,
    .angle_turnV90 = 90.0f,
    .dist_turnV90_in = 18.8f,
    .dist_turnV90_out = 19.2f,
    // 135deg In
    .velocity_turn135in = 1000.0f,
    .alpha_turn135in = 33000.0f,
    .angle_turn135in = 135.0f,
    .dist_turn135in_in = 16.7f,
    .dist_turn135in_out = 9.0f,
    // 135deg Out
    .velocity_turn135out = 1000.0f,
    .alpha_turn135out = 35000.0f,
    .angle_turn135out = 135.0f,
    .dist_turn135out_in = 12.4f,
    .dist_turn135out_out = 21.0f,
    .fan_power = 500,
    .makepath_type_case3 = 0,
    .makepath_type_case47 = 1,
    // 壁切れ検出しきい値（ヒステリシス付き）
    // 2026-09-26 trace estimate; left mirrors right pending measured cuts.
    // docs/MINI_R3_MODE34_WALL_CORRECTION.md (case1/2 remain correction-off).
    .wall_end_thr_r_high = 350, .wall_end_thr_r_low = 300,
    .wall_end_thr_l_high = 350, .wall_end_thr_l_low = 300,
    // 加速度切り替え速度
    .accel_switch_velocity = 1000.0f
};

/* case1: small90 only; case2: minimum large-turn connectors and stop.
 * Accel is ceil(v_turn^2 / (2 * 45 mm)) to the next 1000 mm/s^2.
 * case3..9: +200 mm/s and +1000 mm/s^2 per case (diagonal +100 mm/s). */
const ShortestRunCaseParams_t shortestRunCaseParamsMode3[9] = {
    // case1
    {
        .acceleration_straight = 8000.0f, .acceleration_straight_dash = 8000.0f,
        .velocity_straight = 800.0f, .kp_wall = 0.15f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 12000.0f, .acceleration_d_straight_dash = 12000.0f,
        .velocity_d_straight = 1000.0f, .kp_diagonal = 0.0f
    },
    // case2
    {
        .acceleration_straight = 12000.0f, .acceleration_straight_dash = 12000.0f,
        .velocity_straight = 1000.0f, .kp_wall = 0.15f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 12000.0f, .acceleration_d_straight_dash = 12000.0f,
        .velocity_d_straight = 1000.0f, .kp_diagonal = 0.0f
    },
    // case3
    {
        .acceleration_straight = 13000.0f, .acceleration_straight_dash = 13000.0f,
        .velocity_straight = 1200.0f, .kp_wall = 0.15f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 13000.0f, .acceleration_d_straight_dash = 13000.0f,
        .velocity_d_straight = 1100.0f, .kp_diagonal = 0.0f
    },
    // case4
    {
        .acceleration_straight = 14000.0f, .acceleration_straight_dash = 14000.0f,
        .velocity_straight = 1400.0f, .kp_wall = 0.15f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 14000.0f, .acceleration_d_straight_dash = 14000.0f,
        .velocity_d_straight = 1200.0f, .kp_diagonal = 0.0f
    },
    // case5
    {
        .acceleration_straight = 15000.0f, .acceleration_straight_dash = 15000.0f,
        .velocity_straight = 1600.0f, .kp_wall = 0.15f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 15000.0f, .acceleration_d_straight_dash = 15000.0f,
        .velocity_d_straight = 1300.0f, .kp_diagonal = 0.0f
    },
    // case6
    {
        .acceleration_straight = 16000.0f, .acceleration_straight_dash = 16000.0f,
        .velocity_straight = 1800.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 16000.0f, .acceleration_d_straight_dash = 16000.0f,
        .velocity_d_straight = 1400.0f, .kp_diagonal = 0.05f
    },
    // case7
    {
        .acceleration_straight = 17000.0f, .acceleration_straight_dash = 17000.0f,
        .velocity_straight = 2000.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 17000.0f, .acceleration_d_straight_dash = 17000.0f,
        .velocity_d_straight = 1500.0f, .kp_diagonal = 0.05f
    },
    // case8 (diagonal reserved; not for current runs)
    {
        .acceleration_straight = 18000.0f, .acceleration_straight_dash = 18000.0f,
        .velocity_straight = 2200.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 18000.0f, .acceleration_d_straight_dash = 18000.0f,
        .velocity_d_straight = 1600.0f, .kp_diagonal = 0.05f
    },
    // case9 (diagonal reserved; not for current runs)
    {
        .acceleration_straight = 19000.0f, .acceleration_straight_dash = 19000.0f,
        .velocity_straight = 2400.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 19000.0f, .acceleration_d_straight_dash = 19000.0f,
        .velocity_d_straight = 1700.0f, .kp_diagonal = 0.05f
    },
};

// ========================= Mode 4 =========================
/* PR #21 ideal simulation, 1 kHz / rounding 1.2 / omega 3000 deg/s. */
const ShortestRunModeParams_t shortestRunModeParams4 = {
    /* Explicit simulator input; preserves the tuned turn geometry. */
    .turn_omega_max = 3000.0f,
    // 90deg
    .velocity_turn90 = 1000.0f,
    .alpha_turn90 = 79000.0f,
    .acceleration_turn = 0.0f,
    .dist_offset_in = 0.8f,
    .dist_offset_out = 1.2f,
    // F413 front target = 45 + 45 - 0.8 = 89.2 mm (calibrated centre LUT).
    .val_offset_in = 98.0f, // Legacy F405 compatibility; unused by F413.
    .fwall_kx = 0.6f,      // Legacy F405 compatibility; unused by F413.
    .angle_turn_90 = 90.0f,
    .dist_wall_end = 0.0f, // Provisional with 350/300 thresholds; see tuning note.
    // Large 90deg
    .velocity_l_turn_90 = 1400.0f,
    .alpha_l_turn_90 = 40500.0f,
    .angle_l_turn_90 = 90.0f,
    .dist_l_turn_in_90 = 3.8f,
    .dist_l_turn_out_90 = 5.1f,
    // Large 180deg
    .velocity_l_turn_180 = 1400.0f,
    .alpha_l_turn_180 = 35000.0f,
    .angle_l_turn_180 = 180.0f,
    .dist_l_turn_in_180 = 1.0f,
    .dist_l_turn_out_180 = 1.1f,
    // Diagonal placeholders: defer use until mini_r2 validation.
    // 45deg In
    .velocity_turn45in = 1400.0f,
    .alpha_turn45in = 61500.0f,
    .angle_turn45in = 45.0f,
    .dist_turn45in_in = 1.1f,
    .dist_turn45in_out = 20.6f,
    // 45deg Out
    .velocity_turn45out = 1400.0f,
    .alpha_turn45out = 57000.0f,
    .angle_turn45out = 45.0f,
    .dist_turn45out_in = 18.1f,
    .dist_turn45out_out = 0.7f,
    // V90deg
    .velocity_turnV90 = 1400.0f,
    .alpha_turnV90 = 146000.0f,
    .angle_turnV90 = 90.0f,
    .dist_turnV90_in = 17.9f,
    .dist_turnV90_out = 18.9f,
    // 135deg In
    .velocity_turn135in = 1400.0f,
    .alpha_turn135in = 66500.0f,
    .angle_turn135in = 135.0f,
    .dist_turn135in_in = 18.1f,
    .dist_turn135in_out = 10.8f,
    // 135deg Out
    .velocity_turn135out = 1400.0f,
    .alpha_turn135out = 68000.0f,
    .angle_turn135out = 135.0f,
    .dist_turn135out_in = 11.7f,
    .dist_turn135out_out = 19.7f,
    .fan_power = 700,
    .makepath_type_case3 = 0,
    .makepath_type_case47 = 1,
    // 壁切れ検出しきい値（ヒステリシス付き）
    // 2026-09-26 trace estimate; left mirrors right pending measured cuts.
    // docs/MINI_R3_MODE34_WALL_CORRECTION.md (case1/2 remain correction-off).
    .wall_end_thr_r_high = 350, .wall_end_thr_r_low = 300,
    .wall_end_thr_l_high = 350, .wall_end_thr_l_low = 300,
    // 加速度切り替え速度
    .accel_switch_velocity = 1400.0f
};

/* case1: small90 only; case2: minimum large-turn connectors and stop.
 * Accel is ceil(v_turn^2 / (2 * 45 mm)) to the next 1000 mm/s^2.
 * case3..9: +200 mm/s and +1000 mm/s^2 per case (diagonal +100 mm/s). */
const ShortestRunCaseParams_t shortestRunCaseParamsMode4[9] = {
    // case1
    {
        .acceleration_straight = 12000.0f, .acceleration_straight_dash = 12000.0f,
        .velocity_straight = 1000.0f, .kp_wall = 0.2f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 22000.0f, .acceleration_d_straight_dash = 22000.0f,
        .velocity_d_straight = 1400.0f, .kp_diagonal = 0.0f
    },
    // case2
    {
        .acceleration_straight = 22000.0f, .acceleration_straight_dash = 22000.0f,
        .velocity_straight = 1400.0f, .kp_wall = 0.2f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 22000.0f, .acceleration_d_straight_dash = 22000.0f,
        .velocity_d_straight = 1400.0f, .kp_diagonal = 0.0f
    },
    // case3
    {
        .acceleration_straight = 23000.0f, .acceleration_straight_dash = 23000.0f,
        .velocity_straight = 1600.0f, .kp_wall = 0.2f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 23000.0f, .acceleration_d_straight_dash = 23000.0f,
        .velocity_d_straight = 1500.0f, .kp_diagonal = 0.0f
    },
    // case4
    {
        .acceleration_straight = 24000.0f, .acceleration_straight_dash = 24000.0f,
        .velocity_straight = 1800.0f, .kp_wall = 0.16f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 24000.0f, .acceleration_d_straight_dash = 24000.0f,
        .velocity_d_straight = 1600.0f, .kp_diagonal = 0.0f
    },
    // case5
    {
        .acceleration_straight = 25000.0f, .acceleration_straight_dash = 25000.0f,
        .velocity_straight = 2000.0f, .kp_wall = 0.14f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 25000.0f, .acceleration_d_straight_dash = 25000.0f,
        .velocity_d_straight = 1700.0f, .kp_diagonal = 0.0f
    },
    // case6
    {
        .acceleration_straight = 26000.0f, .acceleration_straight_dash = 26000.0f,
        .velocity_straight = 2200.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 26000.0f, .acceleration_d_straight_dash = 26000.0f,
        .velocity_d_straight = 1800.0f, .kp_diagonal = 0.05f
    },
    // case7
    {
        .acceleration_straight = 27000.0f, .acceleration_straight_dash = 27000.0f,
        .velocity_straight = 2400.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 27000.0f, .acceleration_d_straight_dash = 27000.0f,
        .velocity_d_straight = 1900.0f, .kp_diagonal = 0.05f
    },
    // case8 (diagonal reserved; not for current runs)
    {
        .acceleration_straight = 28000.0f, .acceleration_straight_dash = 28000.0f,
        .velocity_straight = 2600.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 28000.0f, .acceleration_d_straight_dash = 28000.0f,
        .velocity_d_straight = 2000.0f, .kp_diagonal = 0.05f
    },
    // case9 (diagonal reserved; not for current runs)
    {
        .acceleration_straight = 29000.0f, .acceleration_straight_dash = 29000.0f,
        .velocity_straight = 2800.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 29000.0f, .acceleration_d_straight_dash = 29000.0f,
        .velocity_d_straight = 2100.0f, .kp_diagonal = 0.05f
    },
};

// ========================= Mode 5 =========================
/* PR #21 ideal simulation, 1 kHz / rounding 1.2 / omega 3000 deg/s. */
const ShortestRunModeParams_t shortestRunModeParams5 = {
    /* Explicit simulator input; preserves the tuned turn geometry. */
    .turn_omega_max = 3000.0f,
    // 90deg
    .velocity_turn90 = 1200.0f,
    .alpha_turn90 = 115000.0f,
    .acceleration_turn = 0.0f,
    .dist_offset_in = 0.9f,
    .dist_offset_out = 1.7f,
    .val_offset_in = 98.0f,
    .fwall_kx = 0.6f,
    .angle_turn_90 = 90.0f,
    .dist_wall_end = 0.0f,
    // Large 90deg
    .velocity_l_turn_90 = 1700.0f,
    .alpha_l_turn_90 = 56000.0f,
    .angle_l_turn_90 = 90.0f,
    .dist_l_turn_in_90 = 0.8f,
    .dist_l_turn_out_90 = 1.5f,
    // Large 180deg
    .velocity_l_turn_180 = 1700.0f,
    .alpha_l_turn_180 = 52000.0f,
    .angle_l_turn_180 = 180.0f,
    .dist_l_turn_in_180 = 1.0f,
    .dist_l_turn_out_180 = 2.2f,
    // Diagonal placeholders: defer use until mini_r2 validation.
    // 45deg In
    .velocity_turn45in = 1500.0f,
    .alpha_turn45in = 71000.0f,
    .angle_turn45in = 45.0f,
    .dist_turn45in_in = 1.2f,
    .dist_turn45in_out = 20.5f,
    // 45deg Out
    .velocity_turn45out = 1500.0f,
    .alpha_turn45out = 65500.0f,
    .angle_turn45out = 45.0f,
    .dist_turn45out_in = 18.0f,
    .dist_turn45out_out = 0.6f,
    // V90deg
    .velocity_turnV90 = 1500.0f,
    .alpha_turnV90 = 144500.0f,
    .angle_turnV90 = 90.0f,
    .dist_turnV90_in = 14.4f,
    .dist_turnV90_out = 14.4f,
    // 135deg In
    .velocity_turn135in = 1500.0f,
    .alpha_turn135in = 74000.0f,
    .angle_turn135in = 135.0f,
    .dist_turn135in_in = 16.2f,
    .dist_turn135in_out = 8.8f,
    // 135deg Out
    .velocity_turn135out = 1500.0f,
    .alpha_turn135out = 77000.0f,
    .angle_turn135out = 135.0f,
    .dist_turn135out_in = 10.8f,
    .dist_turn135out_out = 19.0f,
    .fan_power = 1000,
    .makepath_type_case3 = 0,
    .makepath_type_case47 = 1,
    // 壁切れ検出しきい値（ヒステリシス付き）
    .wall_end_thr_r_high = 100, .wall_end_thr_r_low = 1,
    .wall_end_thr_l_high = 100, .wall_end_thr_l_low = 1,
    // 加速度切り替え速度
    .accel_switch_velocity = 1700.0f
};

/* case1: small90 only; case2: minimum large-turn connectors and stop.
 * Accel is ceil(v_turn^2 / (2 * 45 mm)) to the next 1000 mm/s^2.
 * case3..9: +200 mm/s and +1000 mm/s^2 per case (diagonal +100 mm/s). */
const ShortestRunCaseParams_t shortestRunCaseParamsMode5[9] = {
    // case1
    {
        .acceleration_straight = 16000.0f, .acceleration_straight_dash = 16000.0f,
        .velocity_straight = 1200.0f, .kp_wall = 0.2f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 25000.0f, .acceleration_d_straight_dash = 25000.0f,
        .velocity_d_straight = 1500.0f, .kp_diagonal = 0.0f
    },
    // case2
    {
        .acceleration_straight = 33000.0f, .acceleration_straight_dash = 33000.0f,
        .velocity_straight = 1700.0f, .kp_wall = 0.2f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 25000.0f, .acceleration_d_straight_dash = 25000.0f,
        .velocity_d_straight = 1500.0f, .kp_diagonal = 0.0f
    },
    // case3
    {
        .acceleration_straight = 34000.0f, .acceleration_straight_dash = 34000.0f,
        .velocity_straight = 1900.0f, .kp_wall = 0.2f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 26000.0f, .acceleration_d_straight_dash = 26000.0f,
        .velocity_d_straight = 1600.0f, .kp_diagonal = 0.0f
    },
    // case4
    {
        .acceleration_straight = 35000.0f, .acceleration_straight_dash = 35000.0f,
        .velocity_straight = 2100.0f, .kp_wall = 0.16f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 27000.0f, .acceleration_d_straight_dash = 27000.0f,
        .velocity_d_straight = 1700.0f, .kp_diagonal = 0.0f
    },
    // case5
    {
        .acceleration_straight = 36000.0f, .acceleration_straight_dash = 36000.0f,
        .velocity_straight = 2300.0f, .kp_wall = 0.12f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 28000.0f, .acceleration_d_straight_dash = 28000.0f,
        .velocity_d_straight = 1800.0f, .kp_diagonal = 0.0f
    },
    // case6
    {
        .acceleration_straight = 37000.0f, .acceleration_straight_dash = 37000.0f,
        .velocity_straight = 2500.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 29000.0f, .acceleration_d_straight_dash = 29000.0f,
        .velocity_d_straight = 1900.0f, .kp_diagonal = 0.05f
    },
    // case7
    {
        .acceleration_straight = 38000.0f, .acceleration_straight_dash = 38000.0f,
        .velocity_straight = 2700.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 30000.0f, .acceleration_d_straight_dash = 30000.0f,
        .velocity_d_straight = 2000.0f, .kp_diagonal = 0.05f
    },
    // case8 (diagonal reserved; not for current runs)
    {
        .acceleration_straight = 39000.0f, .acceleration_straight_dash = 39000.0f,
        .velocity_straight = 2900.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 31000.0f, .acceleration_d_straight_dash = 31000.0f,
        .velocity_d_straight = 2100.0f, .kp_diagonal = 0.05f
    },
    // case9 (diagonal reserved; not for current runs)
    {
        .acceleration_straight = 40000.0f, .acceleration_straight_dash = 40000.0f,
        .velocity_straight = 3100.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 32000.0f, .acceleration_d_straight_dash = 32000.0f,
        .velocity_d_straight = 2200.0f, .kp_diagonal = 0.05f
    },
};

// ========================= Mode 6 =========================
/* PR #21 ideal simulation, 1 kHz / rounding 1.2 / omega 3000 deg/s. */
const ShortestRunModeParams_t shortestRunModeParams6 = {
    /* Explicit simulator input; preserves the tuned turn geometry. */
    .turn_omega_max = 3000.0f,
    // 90deg
    .velocity_turn90 = 1200.0f,
    .alpha_turn90 = 115000.0f,
    .acceleration_turn = 0.0f,
    .dist_offset_in = 0.9f,
    .dist_offset_out = 1.7f,
    .val_offset_in = 98.0f,
    .fwall_kx = 0.6f,
    .angle_turn_90 = 90.0f,
    .dist_wall_end = 0.0f,
    // Large 90deg
    .velocity_l_turn_90 = 2000.0f,
    .alpha_l_turn_90 = 87000.0f,
    .angle_l_turn_90 = 90.0f,
    .dist_l_turn_in_90 = 5.7f,
    .dist_l_turn_out_90 = 7.5f,
    // Large 180deg
    .velocity_l_turn_180 = 2000.0f,
    .alpha_l_turn_180 = 72000.0f,
    .angle_l_turn_180 = 180.0f,
    .dist_l_turn_in_180 = 1.0f,
    .dist_l_turn_out_180 = 1.5f,
    // Diagonal placeholders: defer use until mini_r2 validation.
    // 45deg In
    .velocity_turn45in = 1500.0f,
    .alpha_turn45in = 71000.0f,
    .angle_turn45in = 45.0f,
    .dist_turn45in_in = 1.2f,
    .dist_turn45in_out = 20.5f,
    // 45deg Out
    .velocity_turn45out = 1500.0f,
    .alpha_turn45out = 65500.0f,
    .angle_turn45out = 45.0f,
    .dist_turn45out_in = 18.0f,
    .dist_turn45out_out = 0.6f,
    // V90deg
    .velocity_turnV90 = 1500.0f,
    .alpha_turnV90 = 144500.0f,
    .angle_turnV90 = 90.0f,
    .dist_turnV90_in = 14.4f,
    .dist_turnV90_out = 14.4f,
    // 135deg In
    .velocity_turn135in = 1500.0f,
    .alpha_turn135in = 74000.0f,
    .angle_turn135in = 135.0f,
    .dist_turn135in_in = 16.2f,
    .dist_turn135in_out = 8.8f,
    // 135deg Out
    .velocity_turn135out = 1500.0f,
    .alpha_turn135out = 77000.0f,
    .angle_turn135out = 135.0f,
    .dist_turn135out_in = 10.8f,
    .dist_turn135out_out = 19.0f,
    .fan_power = 1000,
    .makepath_type_case3 = 0,
    .makepath_type_case47 = 1,
    // 壁切れ検出しきい値（ヒステリシス付き）
    .wall_end_thr_r_high = 100, .wall_end_thr_r_low = 1,
    .wall_end_thr_l_high = 100, .wall_end_thr_l_low = 1,
    // 加速度切り替え速度
    .accel_switch_velocity = 2000.0f
};

/* case1: small90 only; case2: minimum large-turn connectors and stop.
 * Accel is ceil(v_turn^2 / (2 * 45 mm)) to the next 1000 mm/s^2.
 * case3..9: +200 mm/s and +1000 mm/s^2 per case (diagonal +100 mm/s). */
const ShortestRunCaseParams_t shortestRunCaseParamsMode6[9] = {
    // case1
    {
        .acceleration_straight = 16000.0f, .acceleration_straight_dash = 16000.0f,
        .velocity_straight = 1200.0f, .kp_wall = 0.25f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 25000.0f, .acceleration_d_straight_dash = 25000.0f,
        .velocity_d_straight = 1500.0f, .kp_diagonal = 0.05f
    },
    // case2
    {
        .acceleration_straight = 45000.0f, .acceleration_straight_dash = 45000.0f,
        .velocity_straight = 2000.0f, .kp_wall = 0.25f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 25000.0f, .acceleration_d_straight_dash = 25000.0f,
        .velocity_d_straight = 1500.0f, .kp_diagonal = 0.05f
    },
    // case3
    {
        .acceleration_straight = 46000.0f, .acceleration_straight_dash = 46000.0f,
        .velocity_straight = 2200.0f, .kp_wall = 0.5f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 26000.0f, .acceleration_d_straight_dash = 26000.0f,
        .velocity_d_straight = 1600.0f, .kp_diagonal = 0.05f
    },
    // case4
    {
        .acceleration_straight = 47000.0f, .acceleration_straight_dash = 47000.0f,
        .velocity_straight = 2400.0f, .kp_wall = 0.5f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 27000.0f, .acceleration_d_straight_dash = 27000.0f,
        .velocity_d_straight = 1700.0f, .kp_diagonal = 0.05f
    },
    // case5
    {
        .acceleration_straight = 48000.0f, .acceleration_straight_dash = 48000.0f,
        .velocity_straight = 2600.0f, .kp_wall = 0.5f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 28000.0f, .acceleration_d_straight_dash = 28000.0f,
        .velocity_d_straight = 1800.0f, .kp_diagonal = 0.05f
    },
    // case6
    {
        .acceleration_straight = 49000.0f, .acceleration_straight_dash = 49000.0f,
        .velocity_straight = 2800.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 29000.0f, .acceleration_d_straight_dash = 29000.0f,
        .velocity_d_straight = 1900.0f, .kp_diagonal = 0.05f
    },
    // case7
    {
        .acceleration_straight = 50000.0f, .acceleration_straight_dash = 50000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 30000.0f, .acceleration_d_straight_dash = 30000.0f,
        .velocity_d_straight = 2000.0f, .kp_diagonal = 0.05f
    },
    // case8 (diagonal reserved; not for current runs)
    {
        .acceleration_straight = 51000.0f, .acceleration_straight_dash = 51000.0f,
        .velocity_straight = 3200.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 31000.0f, .acceleration_d_straight_dash = 31000.0f,
        .velocity_d_straight = 2100.0f, .kp_diagonal = 0.05f
    },
    // case9 (diagonal reserved; not for current runs)
    {
        .acceleration_straight = 52000.0f, .acceleration_straight_dash = 52000.0f,
        .velocity_straight = 3400.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 32000.0f, .acceleration_d_straight_dash = 32000.0f,
        .velocity_d_straight = 2200.0f, .kp_diagonal = 0.05f
    },
};

// ========================= Mode 7 =========================
/* PR #21 ideal simulation, 1 kHz / rounding 1.2 / omega 3000 deg/s. */
const ShortestRunModeParams_t shortestRunModeParams7 = {
    /* Explicit simulator input; preserves the tuned turn geometry. */
    .turn_omega_max = 3000.0f,
    // 90deg
    .velocity_turn90 = 1400.0f,
    .alpha_turn90 = 163500.0f,
    .acceleration_turn = 0.0f,
    .dist_offset_in = 1.4f,
    .dist_offset_out = 1.4f,
    .val_offset_in = 98.0f,
    .fwall_kx = 0.6f,
    .angle_turn_90 = 90.0f,
    .dist_wall_end = 0.0f,
    // Large 90deg
    .velocity_l_turn_90 = 2200.0f,
    .alpha_l_turn_90 = 107000.0f,
    .angle_l_turn_90 = 90.0f,
    .dist_l_turn_in_90 = 6.2f,
    .dist_l_turn_out_90 = 8.3f,
    // Large 180deg
    .velocity_l_turn_180 = 2200.0f,
    .alpha_l_turn_180 = 105000.0f,
    .angle_l_turn_180 = 180.0f,
    .dist_l_turn_in_180 = 3.0f,
    .dist_l_turn_out_180 = 3.6f,
    // Diagonal placeholders: defer use until mini_r2 validation.
    // 45deg In
    .velocity_turn45in = 1500.0f,
    .alpha_turn45in = 71000.0f,
    .angle_turn45in = 45.0f,
    .dist_turn45in_in = 1.2f,
    .dist_turn45in_out = 20.5f,
    // 45deg Out
    .velocity_turn45out = 1500.0f,
    .alpha_turn45out = 65500.0f,
    .angle_turn45out = 45.0f,
    .dist_turn45out_in = 18.0f,
    .dist_turn45out_out = 0.6f,
    // V90deg
    .velocity_turnV90 = 1500.0f,
    .alpha_turnV90 = 144500.0f,
    .angle_turnV90 = 90.0f,
    .dist_turnV90_in = 14.4f,
    .dist_turnV90_out = 14.4f,
    // 135deg In
    .velocity_turn135in = 1500.0f,
    .alpha_turn135in = 74000.0f,
    .angle_turn135in = 135.0f,
    .dist_turn135in_in = 16.2f,
    .dist_turn135in_out = 8.8f,
    // 135deg Out
    .velocity_turn135out = 1500.0f,
    .alpha_turn135out = 77000.0f,
    .angle_turn135out = 135.0f,
    .dist_turn135out_in = 10.8f,
    .dist_turn135out_out = 19.0f,
    .fan_power = 1000,
    .makepath_type_case3 = 0,
    .makepath_type_case47 = 1,
    // 壁切れ検出しきい値（ヒステリシス付き）
    .wall_end_thr_r_high = 100, .wall_end_thr_r_low = 1,
    .wall_end_thr_l_high = 100, .wall_end_thr_l_low = 1,
    // 加速度切り替え速度
    .accel_switch_velocity = 2200.0f
};

/* case1: small90 only; case2: minimum large-turn connectors and stop.
 * Accel is ceil(v_turn^2 / (2 * 45 mm)) to the next 1000 mm/s^2.
 * case3..9: +200 mm/s and +1000 mm/s^2 per case (diagonal +100 mm/s). */
const ShortestRunCaseParams_t shortestRunCaseParamsMode7[9] = {
    // case1
    {
        .acceleration_straight = 22000.0f, .acceleration_straight_dash = 22000.0f,
        .velocity_straight = 1400.0f, .kp_wall = 0.25f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 25000.0f, .acceleration_d_straight_dash = 25000.0f,
        .velocity_d_straight = 1500.0f, .kp_diagonal = 0.05f
    },
    // case2
    {
        .acceleration_straight = 54000.0f, .acceleration_straight_dash = 54000.0f,
        .velocity_straight = 2200.0f, .kp_wall = 0.25f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 25000.0f, .acceleration_d_straight_dash = 25000.0f,
        .velocity_d_straight = 1500.0f, .kp_diagonal = 0.05f
    },
    // case3
    {
        .acceleration_straight = 55000.0f, .acceleration_straight_dash = 55000.0f,
        .velocity_straight = 2400.0f, .kp_wall = 0.5f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 26000.0f, .acceleration_d_straight_dash = 26000.0f,
        .velocity_d_straight = 1600.0f, .kp_diagonal = 0.05f
    },
    // case4
    {
        .acceleration_straight = 56000.0f, .acceleration_straight_dash = 56000.0f,
        .velocity_straight = 2600.0f, .kp_wall = 0.5f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 27000.0f, .acceleration_d_straight_dash = 27000.0f,
        .velocity_d_straight = 1700.0f, .kp_diagonal = 0.05f
    },
    // case5
    {
        .acceleration_straight = 57000.0f, .acceleration_straight_dash = 57000.0f,
        .velocity_straight = 2800.0f, .kp_wall = 0.5f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 28000.0f, .acceleration_d_straight_dash = 28000.0f,
        .velocity_d_straight = 1800.0f, .kp_diagonal = 0.05f
    },
    // case6
    {
        .acceleration_straight = 58000.0f, .acceleration_straight_dash = 58000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 29000.0f, .acceleration_d_straight_dash = 29000.0f,
        .velocity_d_straight = 1900.0f, .kp_diagonal = 0.05f
    },
    // case7
    {
        .acceleration_straight = 59000.0f, .acceleration_straight_dash = 59000.0f,
        .velocity_straight = 3200.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 30000.0f, .acceleration_d_straight_dash = 30000.0f,
        .velocity_d_straight = 2000.0f, .kp_diagonal = 0.05f
    },
    // case8 (diagonal reserved; not for current runs)
    {
        .acceleration_straight = 60000.0f, .acceleration_straight_dash = 60000.0f,
        .velocity_straight = 3400.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 31000.0f, .acceleration_d_straight_dash = 31000.0f,
        .velocity_d_straight = 2100.0f, .kp_diagonal = 0.05f
    },
    // case9 (diagonal reserved; not for current runs)
    {
        .acceleration_straight = 61000.0f, .acceleration_straight_dash = 61000.0f,
        .velocity_straight = 3600.0f, .kp_wall = 0.025f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 32000.0f, .acceleration_d_straight_dash = 32000.0f,
        .velocity_d_straight = 2200.0f, .kp_diagonal = 0.05f
    },
};
