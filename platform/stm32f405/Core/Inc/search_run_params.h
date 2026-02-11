/**
 * @file search_run_params.h
 * @brief 探索走行パラメータ定義
 */

#ifndef SEARCH_RUN_PARAMS_H
#define SEARCH_RUN_PARAMS_H

#include <stdint.h>

/**
 * @brief 探索走行パラメータ構造体
 */
typedef struct {
    // 直線パラメータ
    float acceleration_straight;      ///< 直線加速度 (mm/s^2)
    float acceleration_straight_dash; ///< 二段階直線加速度 (mm/s^2)
    
    // ターンパラメータ
    float velocity_turn90;            ///< 90°ターン速度 (mm/s)
    float alpha_turn90;               ///< 90°ターンハンドリングパラメータ
    float acceleration_turn;          ///< ターン加速度 (m/s^2)
    float dist_offset_in;             ///< 内側オフセット (mm)
    float dist_offset_out;            ///< 外側オフセット (mm)
    float val_offset_in;              ///< 内部位置補正値
    float angle_turn_90;              ///< 90°ターン角度設定 (度)
    
    // 壁切れ後の追従距離
    float dist_wall_end;              ///< 壁切れ検知後にターン開始までに直進する距離 (mm)
    
    // 壁制御パラメータ
    float kp_wall;                    ///< 壁制御比例ゲイン
    float duty_setposition;           ///< ケツ当て制御duty値
    
    // センサパラメータ
    float sensor_kx;                  ///< 壁判断しきい値の係数
    float fwall_kx;                   ///< 前壁検出係数
    
    // 壁切れ検出しきい値（速度依存、ヒステリシス付き）
    uint16_t wall_end_thr_r_high;     ///< 右壁切れ検出Highしきい値（壁あり判定）
    uint16_t wall_end_thr_r_low;      ///< 右壁切れ検出Lowしきい値（壁なし判定）
    uint16_t wall_end_thr_l_high;     ///< 左壁切れ検出Highしきい値（壁あり判定）
    uint16_t wall_end_thr_l_low;      ///< 左壁切れ検出Lowしきい値（壁なし判定）
    
    uint16_t fan_duty;                ///< 探索時ファンドューティ
    
    // フラグ
    uint8_t wall_align_enable;        ///< 壁合わせ有効フラグ (0:無効, 1:有効)
} SearchRunParams_t;

// 探索走行パラメータ配列（case1, case5の2種類）
extern const SearchRunParams_t searchRunParams[2];

#endif // SEARCH_RUN_PARAMS_H
