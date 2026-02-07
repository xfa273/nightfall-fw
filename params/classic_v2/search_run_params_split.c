#include "search_run_params.h"

// 探索走行パラメータ配列
const SearchRunParams_t searchRunParams[2] = {
    // case1 (index 0): 標準速度 (旧mode1 case1相当)
    {
        // 直線パラメータ
        .acceleration_straight = 5555.6f,
        .acceleration_straight_dash = 0.0f,
        
        // ターンパラメータ
        .velocity_turn90 = 1000.0f,
        .alpha_turn90 = 22700.0f,
        .acceleration_turn = 0.0f,
        .dist_offset_in = 10.0f,
        .dist_offset_out = 40.0f,
        .val_offset_in = 730.0f,
        .angle_turn_90 = 89.0f,
        
        // 壁切れ後の追従距離
        .dist_wall_end = 90.0f,
        
        // 壁制御パラメータ
        .kp_wall = 0.12f,
        .duty_setposition = 40.0f,
        
        // センサパラメータ
        .sensor_kx = 1.0f,
        
        // 壁切れ検出しきい値（ヒステリシス付き）
        .wall_end_thr_r_high = 280, .wall_end_thr_r_low = 200,
        .wall_end_thr_l_high = 280, .wall_end_thr_l_low = 200,

        .fan_duty = 130,
        
        // フラグ
        .wall_align_enable = 1  // 壁合わせ有効
    },
    
    // case5 (index 1): 遅い速度 (旧mode1 case5相当)
    {
        // 直線パラメータ
        .acceleration_straight = 2000.0f,
        .acceleration_straight_dash = 0.0f,
        
        // ターンパラメータ
        .velocity_turn90 = 600.0f,
        .alpha_turn90 = 10300.0f,
        .acceleration_turn = 0.0f,
        .dist_offset_in = 20.0f,
        .dist_offset_out = 28.0f,
        .val_offset_in = 730.0f,
        .angle_turn_90 = 89.0f,
        
        // 壁切れ後の追従距離
        .dist_wall_end = 90.0f,
        
        // 壁制御パラメータ
        .kp_wall = 0.10f,
        .duty_setposition = 40.0f,
        
        // センサパラメータ
        .sensor_kx = 1.0f,
        
        // 壁切れ検出しきい値（ヒステリシス付き）
        .wall_end_thr_r_high = 280, .wall_end_thr_r_low = 200,
        .wall_end_thr_l_high = 280, .wall_end_thr_l_low = 200,

        .fan_duty = 0,
        
        // フラグ
        .wall_align_enable = 0  // 壁合わせ無効
    }
};
