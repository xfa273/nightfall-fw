#include "search_run_params.h"

// 探索走行パラメータ配列
const SearchRunParams_t searchRunParams[2] = {
    // case1 (index 0): 標準速度 (旧mode1 case1相当)
    {
        // 直線パラメータ
        .acceleration_straight = 1000.0f,
        .acceleration_straight_dash = 1500.0f,
        
        // ターンパラメータ
        .velocity_turn90 = 300.0f,
        .alpha_turn90 = 8820.0f,
        .acceleration_turn = 0.0f,
        .dist_offset_in = 10.0f,
        .dist_offset_out = 14.5f,
        .val_offset_in = 650.0f,
        .angle_turn_90 = 90.0f,
        
        // 壁切れ後の追従距離
        .dist_wall_end = 44.0f,
        
        // 壁制御パラメータ
        .kp_wall = 0.10f,
        .duty_setposition = 40.0f,
        
        // センサパラメータ
        .sensor_kx = 1.0f,
        .fwall_kx = 1.1f,
        
        // 壁切れ検出しきい値（ヒステリシス付き）
        .wall_end_thr_r_high = 150, .wall_end_thr_r_low = 100,
        .wall_end_thr_l_high = 150, .wall_end_thr_l_low = 100,

        .fan_duty = 0,
        
        // フラグ
        .wall_align_enable = 1  // 壁合わせ有効
    },
    
    // case5 (index 1): 遅い速度 (旧mode1 case5相当)
    {
        // 直線パラメータ
        .acceleration_straight = 694.44f,
        .acceleration_straight_dash = 1000.0f,
        
        // ターンパラメータ
        .velocity_turn90 = 250.0f,
        .alpha_turn90 = 6150.0f,
        .acceleration_turn = 0.0f,
        .dist_offset_in = 10.0f,
        .dist_offset_out = 14.5f,
        .val_offset_in = 650.0f,
        .angle_turn_90 = 90.0f,
        
        // 壁切れ後の追従距離
        .dist_wall_end = 44.0f,
        
        // 壁制御パラメータ
        .kp_wall = 0.10f,
        .duty_setposition = 40.0f,
        
        // センサパラメータ
        .sensor_kx = 1.0f,
        .fwall_kx = 1.1f,
        
        // 壁切れ検出しきい値（ヒステリシス付き）
        .wall_end_thr_r_high = 150, .wall_end_thr_r_low = 100,
        .wall_end_thr_l_high = 150, .wall_end_thr_l_low = 100,

        .fan_duty = 0,
        
        // フラグ
        .wall_align_enable = 0  // 壁合わせ無効
    }
};
