#include "shortest_run_params.h"
#include "solver_params.h"

// ========================= Mode 2 =========================
const ShortestRunModeParams_t shortestRunModeParams2 = {
    // 90deg
    .velocity_turn90 = 300.0f,
    .alpha_turn90 = 8820.0f,
    .acceleration_turn = 0.0f,
    .dist_offset_in = 10.0f,
    .dist_offset_out = 18.0f,
    .val_offset_in = 1750.0f,
    .angle_turn_90 = 89.0f,
    // Large 90deg
    .velocity_l_turn_90 = 500.0f,
    .alpha_l_turn_90    = 4700.0f,  // 4250
    .angle_l_turn_90    = 89.0f,
    .dist_l_turn_in_90  = 5.0f,
    .dist_l_turn_out_90 = 12.7f,
    // Large 180deg
    .velocity_l_turn_180= 500.0f,
    .alpha_l_turn_180   = 4697.0f,  // 4640
    .angle_l_turn_180   = 179.0f,
    .dist_l_turn_in_180 = 12.0f,
    .dist_l_turn_out_180= 19.0f,
    // 45deg In
    .velocity_turn45in   = 500.0f,
    .alpha_turn45in      = 6360.0f,
    .angle_turn45in      = 44.6f,
    .dist_turn45in_in    = 0.0f,
    .dist_turn45in_out   = 28.0f,
    // 45deg Out
    .velocity_turn45out= 500.0f,
    .alpha_turn45out   = 7700.0f,
    .angle_turn45out   = 44.4f,
    .dist_turn45out_in = 15.0f,
    .dist_turn45out_out= 0.0f,
    // V90deg
    .velocity_turnV90  = 500.0f,
    .alpha_turnV90     = 12200.0f,  // 10492
    .angle_turnV90     = 88.3f,
    .dist_turnV90_in   = 2.0f,
    .dist_turnV90_out  = 28.0f,
    // 135deg In
    .velocity_turn135in = 500.0f,
    .alpha_turn135in    = 6888.0f,  // 6938
    .angle_turn135in    = 133.9f,
    .dist_turn135in_in  = 9.0f,
    .dist_turn135in_out = 17.0f,
    // 135deg Out
    .velocity_turn135out = 500.0f,
    .alpha_turn135out    = 6950.0f,
    .angle_turn135out    = 133.3f,
    .dist_turn135out_in  = 0.0f,
    .dist_turn135out_out = 12.0f,
    // Fan
    .fan_power          = 0,
    // Makepath
    .makepath_type_case3 = 0,
    .makepath_type_case47= 1,
    // 壁切れ検出しきい値（ヒステリシス付き）
    .wall_end_thr_r_high = 140, .wall_end_thr_r_low = 100,
    .wall_end_thr_l_high = 140, .wall_end_thr_l_low = 100,
    // 加速度切り替え速度
    .accel_switch_velocity = 2000.0f
};

const ShortestRunCaseParams_t shortestRunCaseParamsMode2[9] = {
    // case1 (index 0): independent (initially same as former case3)
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 1000.0f,
        .velocity_straight = 500.0f, .kp_wall = 0.12f,
        .solver_profile = SOLVER_PROFILE_STANDARD
    },
    // case2 (index 1): independent (initially same as former case4)
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 2000.0f,
        .velocity_straight = 500.0f, .kp_wall = 0.12f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case3 (index 2)
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 3000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case4 (index 3)
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 3000.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case5 (index 4)
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 3500.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case6 (index 5)
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 2000.0f,
        .velocity_straight = 1000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.2f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 1000.0f, .acceleration_d_straight_dash = 2000.0f,
        .velocity_d_straight = 1000.0f
    },
    // case7 (index 6)
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 3500.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 1000.0f, .acceleration_d_straight_dash = 3000.0f,
        .velocity_d_straight = 3000.0f
    },
    // case8 (index 7): diagonal use
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 2000.0f,
        .velocity_straight = 1000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.2f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 1000.0f, .acceleration_d_straight_dash = 2000.0f,
        .velocity_d_straight = 1000.0f
    },
    // case9 (index 8): diagonal use (same as case8 initial)
    {
        .acceleration_straight = 1000.0f, .acceleration_straight_dash = 3500.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 1000.0f, .acceleration_d_straight_dash = 3000.0f,
        .velocity_d_straight = 3000.0f
    },
};

// ========================= Mode 3 =========================
const ShortestRunModeParams_t shortestRunModeParams3 = {
    // 90deg
    .velocity_turn90    = 600.0f,
    .alpha_turn90       = 33900.0f,
    .acceleration_turn  = 0.0f,
    .dist_offset_in     = 8.0f,
    .dist_offset_out    = 19.5f,
    .val_offset_in      = 1000.0f,
    .angle_turn_90      = 87.3f,
    .dist_wall_end      = 1.0f,
    // Large 90deg
    .velocity_l_turn_90 = 1000.0f,
    .alpha_l_turn_90    = 17300.0f,
    .angle_l_turn_90    = 86.7f,
    .dist_l_turn_in_90  = 0.0f,
    .dist_l_turn_out_90 = 20.0f,
    // Large 180deg
    .velocity_l_turn_180= 1000.0f,
    .alpha_l_turn_180   = 16300.0f,
    .angle_l_turn_180   = 176.5f,
    .dist_l_turn_in_180 = 2.0f,
    .dist_l_turn_out_180= 26.0f,
    // 45deg In
    .velocity_turn45in   = 1000.0f,
    .alpha_turn45in      = 27200.0f,
    .angle_turn45in      = 43.0f,
    .dist_turn45in_in    = 0.0f,
    .dist_turn45in_out   = 35.0f,
    // 45deg Out
    .velocity_turn45out= 1000.0f,
    .alpha_turn45out   = 28000.0f,
    .angle_turn45out   = 43.5f,
    .dist_turn45out_in = 17.0f,
    .dist_turn45out_out= 0.0f,
    // V90deg
    .velocity_turnV90  = 1000.0f,
    .alpha_turnV90     = 43000.0f,
    .angle_turnV90     = 85.0f,
    .dist_turnV90_in   = 6.0f,
    .dist_turnV90_out  = 23.0f,
    // 135deg In
    .velocity_turn135in = 1000.0f,
    .alpha_turn135in    = 26500.0f,
    .angle_turn135in    = 133.8f,
    .dist_turn135in_in  = 5.0f,
    .dist_turn135in_out = 25.0f,
    // 135deg Out
    .velocity_turn135out = 1000.0f,
    .alpha_turn135out    = 29000.0f,
    .angle_turn135out    = 132.0f,
    .dist_turn135out_in  = 0.0f,
    .dist_turn135out_out = 26.0f,
    // Fan
    .fan_power          = 600,
    // Makepath
    .makepath_type_case3 = 0,
    .makepath_type_case47= 1,
    .wall_end_thr_r_high = 140, .wall_end_thr_r_low = 130,
    .wall_end_thr_l_high = 140, .wall_end_thr_l_low = 130,
    // 加速度切り替え速度
    .accel_switch_velocity = 1500.0f
};

const ShortestRunCaseParams_t shortestRunCaseParamsMode3[9] = {
    // case1 (index 0): independent (initially same as former case3)
    {
        .acceleration_straight = 4000.0f, .acceleration_straight_dash = 4000.0f,
        .velocity_straight = 1000.0f, .kp_wall = 0.30f,
        .solver_profile = SOLVER_PROFILE_STANDARD
    },
    // case2 (index 1): independent (initially same as former case4)
    {
        .acceleration_straight = 4000.0f, .acceleration_straight_dash = 4000.0f,
        .velocity_straight = 1000.0f, .kp_wall = 0.30f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case3 (index 2)
    {
        .acceleration_straight = 8000.0f, .acceleration_straight_dash = 4000.0f,
        .velocity_straight = 2500.0f, .kp_wall = 0.30f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case4 (index 3)
    {
        .acceleration_straight = 10000.0f, .acceleration_straight_dash = 4000.0f,
        .velocity_straight = 2500.0f, .kp_wall = 0.30f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case5 (index 4)
    {
        .acceleration_straight = 12000.0f, .acceleration_straight_dash = 4000.0f,
        .velocity_straight = 2500.0f, .kp_wall = 0.30f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case6 (index 5)
    {
        .acceleration_straight = 4000.0f, .acceleration_straight_dash = 4000.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 4000.0f, .acceleration_d_straight_dash = 8000.0f,
        .velocity_d_straight = 2500.0f
    },
    // case7 (index 6)
    {
        .acceleration_straight = 4000.0f, .acceleration_straight_dash = 20000.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 4000.0f, .acceleration_d_straight_dash = 8000.0f,
        .velocity_d_straight = 2500.0f
    },
    // case8 (index 7): diagonal use
    {
        .acceleration_straight = 4000.0f, .acceleration_straight_dash = 10000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 4000.0f, .acceleration_d_straight_dash = 8000.0f,
        .velocity_d_straight = 2500.0f
    },
    // case9 (index 8): diagonal use
    {
        .acceleration_straight = 4000.0f, .acceleration_straight_dash = 14000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 4000.0f, .acceleration_d_straight_dash = 8000.0f,
        .velocity_d_straight = 2500.0f
    },
};

// ========================= Mode 4 =========================
const ShortestRunModeParams_t shortestRunModeParams4 = {
    // 90deg
    .velocity_turn90    = 800.0f,
    .alpha_turn90       = 50200.0f,
    .acceleration_turn  = 0.0f,
    .dist_offset_in     = 4.0f,
    .dist_offset_out    = 20.0f,
    .val_offset_in      = 10.0f,
    .angle_turn_90      = 86.0f,
    .dist_wall_end      = 1.0f,
    // Large 90deg
    .velocity_l_turn_90 = 1000.0f,
    .alpha_l_turn_90    = 17300.0f,
    .angle_l_turn_90    = 86.7f,
    .dist_l_turn_in_90  = 0.0f,
    .dist_l_turn_out_90 = 20.0f,
    // Large 180deg
    .velocity_l_turn_180= 1000.0f,
    .alpha_l_turn_180   = 16300.0f,
    .angle_l_turn_180   = 176.5f,
    .dist_l_turn_in_180 = 2.0f,
    .dist_l_turn_out_180= 26.0f,
    // 45deg In
    .velocity_turn45in   = 1200.0f,
    .alpha_turn45in      = 16422.0f,
    .angle_turn45in      = 45.0f,
    .dist_turn45in_in    = 0.0f,
    .dist_turn45in_out   = 20.0f,
    // 45deg Out
    .velocity_turn45out = 1200.0f,
    .alpha_turn45out    = 16422.0f,
    .angle_turn45out    = 45.0f,
    .dist_turn45out_in  = 18.0f,
    .dist_turn45out_out = 2.0f,
    // V90deg
    .velocity_turnV90   = 1200.0f,
    .alpha_turnV90      = 25838.0f,
    .angle_turnV90      = 90.0f,
    .dist_turnV90_in    = 5.0f,
    .dist_turnV90_out   = 7.0f,
    // 135deg In
    .velocity_turn135in = 1200.0f,
    .alpha_turn135in    = 17395.0f,
    .angle_turn135in    = 135.0f,
    .dist_turn135in_in  = 8.0f,
    .dist_turn135in_out = 2.0f,
    // 135deg Out
    .velocity_turn135out = 1200.0f,
    .alpha_turn135out    = 17736.0f,
    .angle_turn135out    = 135.0f,
    .dist_turn135out_in  = 2.0f,
    .dist_turn135out_out = 10.0f,
    .fan_power          = 550,
    .makepath_type_case3 = 0,
    .makepath_type_case47= 1,
    // 壁切れ検出しきい値（ヒステリシス付き）
    .wall_end_thr_r_high = 140, .wall_end_thr_r_low = 130,
    .wall_end_thr_l_high = 140, .wall_end_thr_l_low = 130,
    // 加速度切り替え速度
    .accel_switch_velocity = 1500.0f
};

const ShortestRunCaseParams_t shortestRunCaseParamsMode4[9] = {
    // case1 (index 0)
    {
        .acceleration_straight = 6000.0f, .acceleration_straight_dash = 6000.0f,
        .velocity_straight = 1200.0f, .kp_wall = 0.4f,
        .solver_profile = SOLVER_PROFILE_STANDARD
    },
    // case2 (index 1)
    {
        .acceleration_straight = 6000.0f, .acceleration_straight_dash = 6000.0f,
        .velocity_straight = 1200.0f, .kp_wall = 0.4f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case3 (index 2)
    {
        .acceleration_straight = 6000.0f, .acceleration_straight_dash = 6000.0f,
        .velocity_straight = 2500.0f, .kp_wall = 0.4f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case4 (index 3)
    {
        .acceleration_straight = 8000.0f, .acceleration_straight_dash = 6000.0f,
        .velocity_straight = 2500.0f, .kp_wall = 0.4f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case5 (index 4)
    {
        .acceleration_straight = 10000.0f, .acceleration_straight_dash = 6000.0f,
        .velocity_straight = 2500.0f, .kp_wall = 0.4f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case6 (index 5): 
    {
        .acceleration_straight = 7111.11f, .acceleration_straight_dash = 16000.0f,
        .velocity_straight = 3500.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 4000.0f, .acceleration_d_straight_dash = 8000.0f,
        .velocity_d_straight = 2000.0f
    },
    // case7 (index 6): 
    {
        .acceleration_straight = 7111.11f, .acceleration_straight_dash = 18000.0f,
        .velocity_straight = 3500.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 4500.0f, .acceleration_d_straight_dash = 9000.0f,
        .velocity_d_straight = 2500.0f
    },
    // case8 (index 7): diagonal-use
    {
        .acceleration_straight = 7111.11f, .acceleration_straight_dash = 12000.0f,
        .velocity_straight = 2000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 4000.0f, .acceleration_d_straight_dash = 8000.0f,
        .velocity_d_straight = 2000.0f
    },
    // case9 (index 8): diagonal-use
    {
        .acceleration_straight = 7111.11f, .acceleration_straight_dash = 12000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 5000.0f, .acceleration_d_straight_dash = 8000.0f,
        .velocity_d_straight = 2000.0f
    },
};

// ========================= Mode 5 =========================
const ShortestRunModeParams_t shortestRunModeParams5 = {
    // 90deg
    .velocity_turn90    = 1000.0f,
    .alpha_turn90       = 74000.0f,
    .acceleration_turn  = 0.0f,
    .dist_offset_in     = 2.0f,
    .dist_offset_out    = 21.5f,
    .val_offset_in      = 15.0f,
    .angle_turn_90      = 82.6f,
    .dist_wall_end      = 1.0f,
    // Large 90deg
    .velocity_l_turn_90 = 1200.0f,
    .alpha_l_turn_90    = 23500.0f,
    .angle_l_turn_90    = 88.0f,
    .dist_l_turn_in_90  = 0.0f,
    .dist_l_turn_out_90 = 18.0f,
    // Large 180deg
    .velocity_l_turn_180= 1200.0f,
    .alpha_l_turn_180   = 24000.0f,
    .angle_l_turn_180   = 176.0f,
    .dist_l_turn_in_180 = 5.0f,
    .dist_l_turn_out_180= 40.0f,
    .fan_power          = 900,
    .makepath_type_case3 = 0,
    .makepath_type_case47= 1,
    // 壁切れ検出しきい値（ヒステリシス付き）
    .wall_end_thr_r_high = 140, .wall_end_thr_r_low = 130,
    .wall_end_thr_l_high = 140, .wall_end_thr_l_low = 130,
    // 加速度切り替え速度
    .accel_switch_velocity = 1500.0f
};

const ShortestRunCaseParams_t shortestRunCaseParamsMode5[9] = {
    // case1 (index 0)
    {
        .acceleration_straight = 12000.0f, .acceleration_straight_dash = 6000.0f,
        .velocity_straight = 1200.0f, .kp_wall = 0.5f,
        .solver_profile = SOLVER_PROFILE_STANDARD
    },
    // case2 (index 1)
    {
        .acceleration_straight = 12000.0f, .acceleration_straight_dash = 6000.0f,
        .velocity_straight = 1200.0f, .kp_wall = 0.5f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case3 (index 2)
    {
        .acceleration_straight = 12000.0f, .acceleration_straight_dash = 6000.0f,
        .velocity_straight = 2500.0f, .kp_wall = 1.0f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case4 (index 3)
    {
        .acceleration_straight = 12000.0f, .acceleration_straight_dash = 6000.0f,
        .velocity_straight = 2500.0f, .kp_wall = 1.0f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case5 (index 4)
    {
        .acceleration_straight = 12000.0f, .acceleration_straight_dash = 6000.0f,
        .velocity_straight = 2500.0f, .kp_wall = 1.0f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case6 (index 5)
    {
        .acceleration_straight = 11111.11f, .acceleration_straight_dash = 28000.0f,
        .velocity_straight = 5000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 6000.0f, .acceleration_d_straight_dash = 12000.0f,
        .velocity_d_straight = 3000.0f
    },
    // case7 (index 6)
    {
        .acceleration_straight = 11111.11f, .acceleration_straight_dash = 30000.0f,
        .velocity_straight = 5200.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 7000.0f, .acceleration_d_straight_dash = 14000.0f,
        .velocity_d_straight = 3500.0f
    },
    // case8 (index 7): diagonal-use
    {
        .acceleration_straight = 11111.11f, .acceleration_straight_dash = 20000.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 6000.0f, .acceleration_d_straight_dash = 12000.0f,
        .velocity_d_straight = 3000.0f
    },
    // case9 (index 8): diagonal-use
    {
        .acceleration_straight = 11111.11f, .acceleration_straight_dash = 23000.0f,
        .velocity_straight = 4500.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 7000.0f, .acceleration_d_straight_dash = 14000.0f,
        .velocity_d_straight = 3500.0f
    },
};

// ========================= Mode 6 =========================
const ShortestRunModeParams_t shortestRunModeParams6 = {
    // 90deg
    .velocity_turn90    = 1000.0f,
    .alpha_turn90       = 74000.0f,
    .acceleration_turn  = 0.0f,
    .dist_offset_in     = 2.0f,
    .dist_offset_out    = 21.5f,
    .val_offset_in      = 15.0f,
    .angle_turn_90      = 82.6f,
    .dist_wall_end      = 1.0f,
    // Large 90deg
    .velocity_l_turn_90 = 1400.0f,
    .alpha_l_turn_90    = 34800.0f,
    .angle_l_turn_90    = 87.0f,
    .dist_l_turn_in_90  = 0.0f,
    .dist_l_turn_out_90 = 23.5f,
    // Large 180deg
    .velocity_l_turn_180= 1400.0f,
    .alpha_l_turn_180   = 32000.0f,
    .angle_l_turn_180   = 174.3f,
    .dist_l_turn_in_180 = 5.0f,
    .dist_l_turn_out_180= 50.0f,
    .fan_power          = 1000,
    .makepath_type_case3 = 0,
    .makepath_type_case47= 1,
    // 壁切れ検出しきい値（ヒステリシス付き）
    .wall_end_thr_r_high = 140, .wall_end_thr_r_low = 130,
    .wall_end_thr_l_high = 140, .wall_end_thr_l_low = 130,
    // 加速度切り替え速度
    .accel_switch_velocity = 1500.0f
};

const ShortestRunCaseParams_t shortestRunCaseParamsMode6[9] = {
    // case1 (index 0) - mode5と同じ値
    {
        .acceleration_straight = 11111.11f, .acceleration_straight_dash = 8000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 0.50f,
        .solver_profile = SOLVER_PROFILE_STANDARD
    },
    // case2 (index 1)
    {
        .acceleration_straight = 11111.11f, .acceleration_straight_dash = 8000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 0.50f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case3 (index 2)
    {
        .acceleration_straight = 15000.0f, .acceleration_straight_dash = 8000.0f,
        .velocity_straight = 1400.0f, .kp_wall = 1.0f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case4 (index 3)
    {
        .acceleration_straight = 11111.11f, .acceleration_straight_dash = 8000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 1.0f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case5 (index 4)
    {
        .acceleration_straight = 11111.11f, .acceleration_straight_dash = 8000.0f,
        .velocity_straight = 3000.0f, .kp_wall = 1.0f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    // case6 (index 5)
    {
        .acceleration_straight = 11111.11f, .acceleration_straight_dash = 28000.0f,
        .velocity_straight = 5000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 6000.0f, .acceleration_d_straight_dash = 12000.0f,
        .velocity_d_straight = 3000.0f
    },
    // case7 (index 6)
    {
        .acceleration_straight = 11111.11f, .acceleration_straight_dash = 30000.0f,
        .velocity_straight = 5200.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG,
        .acceleration_d_straight = 7000.0f, .acceleration_d_straight_dash = 14000.0f,
        .velocity_d_straight = 3500.0f
    },
    // case8 (index 7): diagonal-use
    {
        .acceleration_straight = 11111.11f, .acceleration_straight_dash = 20000.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 6000.0f, .acceleration_d_straight_dash = 12000.0f,
        .velocity_d_straight = 3000.0f
    },
    // case9 (index 8): diagonal-use
    {
        .acceleration_straight = 11111.11f, .acceleration_straight_dash = 23000.0f,
        .velocity_straight = 4500.0f, .kp_wall = 0.05f, .kp_diagonal = 0.05f,
        .solver_profile = SOLVER_PROFILE_STANDARD,
        .acceleration_d_straight = 7000.0f, .acceleration_d_straight_dash = 14000.0f,
        .velocity_d_straight = 3500.0f
    },
};

// ========================= Mode 7 =========================
const ShortestRunModeParams_t shortestRunModeParams7 = {
    .velocity_turn90    = 1400.0f,
    .alpha_turn90       = 50000.0f,
    .acceleration_turn  = 0.0f,
    .dist_offset_in     = 10.0f,
    .dist_offset_out    = 58.0f,
    .val_offset_in      = 1050.0f,
    .angle_turn_90      = 80.5f,
    .dist_wall_end      = 12.0f,
    .velocity_l_turn_90 = 2200.0f,
    .alpha_l_turn_90    = 28500.0f,
    .angle_l_turn_90    = 85.0f,
    .dist_l_turn_in_90  = 0.0f,
    .dist_l_turn_out_90 = 101.0f,
    .velocity_l_turn_180= 2200.0f,
    .alpha_l_turn_180   = 24000.0f,
    .angle_l_turn_180   = 176.8f,
    .dist_l_turn_in_180 = 0.0f,
    .dist_l_turn_out_180= 169.0f,
    .fan_power          = 200,
    .makepath_type_case3 = 0,
    .makepath_type_case47= 1,
    // 壁切れ検出しきい値（ヒステリシス付き）
    .wall_end_thr_r_high = 120, .wall_end_thr_r_low = 115,
    .wall_end_thr_l_high = 120, .wall_end_thr_l_low = 115,
    // 加速度切り替え速度
    .accel_switch_velocity = 2000.0f
};

const ShortestRunCaseParams_t shortestRunCaseParamsMode7[5] = {
    {
        .acceleration_straight = 14222.2f, .acceleration_straight_dash = 25000.0f,
        .velocity_straight = 2300.0f, .kp_wall = 0.13f,
        .solver_profile = SOLVER_PROFILE_STANDARD
    },
    {
        .acceleration_straight = 14222.2f, .acceleration_straight_dash = 25000.0f,
        .velocity_straight = 2300.0f, .kp_wall = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    {
        .acceleration_straight = 14222.2f, .acceleration_straight_dash = 25000.0f,
        .velocity_straight = 4000.0f, .kp_wall = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    {
        .acceleration_straight = 10888.9f, .acceleration_straight_dash = 26000.0f,
        .velocity_straight = 4800.0f, .kp_wall = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
    {
        .acceleration_straight = 10888.9f, .acceleration_straight_dash = 28000.0f,
        .velocity_straight = 5000.0f, .kp_wall = 0.05f,
        .solver_profile = SOLVER_PROFILE_STRAIGHT_STRONG
    },
};
