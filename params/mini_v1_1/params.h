/*
 * params.h
 *
 *  Created on: Feb 28, 2022
 *      Author: yuho-
 */

#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

/*============================================================
    各種定数（パラメータ）設定
============================================================*/
/*------------------------------------------------------------
    走行系
------------------------------------------------------------*/
/*走行パラメータ*/
#define D_TIRE            13.35F  // タイヤ直径[mm] 13.75F
#define DIST_HALF_SEC     45     // 迷路の半区間距離[mm]
#define DIST_D_HALF_SEC   67.279 // 斜めの半区間距離[mm]
#define DIST_FIRST_SEC    13     // 最初の区画の距離[mm]
#define DIST_SET_POSITION 13     // 壁当て後の前進距離[mm]

#ifndef VELOCITY_LPF_TAU
#define VELOCITY_LPF_TAU 0.003F
#endif

#ifndef OMEGA_LPF_TAU
#define OMEGA_LPF_TAU 0.002F
#endif

// 探索直進(one_sectionU)のステップ幅[mm]
// 壁切れ監視のチェック間隔にも影響。大きくするとdriveA呼び出し回数が減り、振動低減が期待できる。
// ただし大きすぎると壁切れ検知後の追従距離算出に遅れが生じ得るため、10mm程度から評価してください。
#ifndef SEARCH_STEP_MM
#define SEARCH_STEP_MM 10.0F
#endif

// 探索の角度保持モード: 角度ゼロリセット条件（連続壁区画数）
#ifndef SEARCH_ANGLE_RESET_DUAL_WALL_STREAK_CELLS
#define SEARCH_ANGLE_RESET_DUAL_WALL_STREAK_CELLS 3u
#endif
#ifndef SEARCH_ANGLE_RESET_SINGLE_WALL_STREAK_CELLS
#define SEARCH_ANGLE_RESET_SINGLE_WALL_STREAK_CELLS 5u
#endif

// ゴール後の探索で、何区画分の新規壁情報が判明したら次の180deg停止で保存するか
#ifndef SEARCH_POST_GOAL_SAVE_NEW_CELL_THRESHOLD
#define SEARCH_POST_GOAL_SAVE_NEW_CELL_THRESHOLD 128u
#endif

#define ALPHA_ROTATE_90   3000  // 超信地旋回の角加速度[deg/sec^2]
#define ANGLE_ROTATE_90_R 90.0F // 超信地旋回の角度[deg]
#define ANGLE_ROTATE_90_L 90.0F // 超信地旋回の角度[deg]

#define DIFF_SETPOSITION 1500 // スラロームを位置合わせに変更する制御量

/*PIDパラメータ*/
#define KP_DISTANCE 1.5F // 並進位置制御のP項  28.0F 30.0
#define KI_DISTANCE 0.03F // 並進位置制御のI項  0.01F 0.04
#define KD_DISTANCE 0.0F // 並進位置制御のD項  28.0F 150.0

#define KP_VELOCITY 0.03F // 並進速度制御のP項  50.0F 10.0
#define KI_VELOCITY 0.30F// 並進速度制御のI項  0.05F 0.04
#define KD_VELOCITY 0.0F // 並進速度制御のD項  60.0F 100.0

/*
 * 吸引ファン ON/OFF で使い分ける PID ゲイン（v1最終の値を使用）
 * control.c の各PIDが drive_use_fan_on_gains() で吸引強度閾値を参照して切替
 */
#ifndef KP_VELOCITY_FAN_ON
#define KP_VELOCITY_FAN_ON  2.5F   
#endif
#ifndef KI_VELOCITY_FAN_ON
#define KI_VELOCITY_FAN_ON  0.03F   
#endif
#ifndef KD_VELOCITY_FAN_ON
#define KD_VELOCITY_FAN_ON  0.0
#endif

#ifndef KP_VELOCITY_FAN_OFF
#define KP_VELOCITY_FAN_OFF 2.0F
#endif
#ifndef KI_VELOCITY_FAN_OFF
#define KI_VELOCITY_FAN_OFF 0.02F
#endif
#ifndef KD_VELOCITY_FAN_OFF
#define KD_VELOCITY_FAN_OFF 0.0F
#endif

#ifndef KP_DISTANCE_FAN_ON
#define KP_DISTANCE_FAN_ON  0.10F    
#endif
#ifndef KI_DISTANCE_FAN_ON
#define KI_DISTANCE_FAN_ON  0.0F
#endif
#ifndef KD_DISTANCE_FAN_ON
#define KD_DISTANCE_FAN_ON  0.0F   
#endif

#ifndef KP_DISTANCE_FAN_OFF
#define KP_DISTANCE_FAN_OFF 0.07F
#endif
#ifndef KI_DISTANCE_FAN_OFF
#define KI_DISTANCE_FAN_OFF 0.0F
#endif
#ifndef KD_DISTANCE_FAN_OFF
#define KD_DISTANCE_FAN_OFF 0.0F
#endif

#define SUCTION_FAN_STABILIZE_DELAY_MS 100

#ifndef SUCTION_GAIN_ON_THRESHOLD_PERCENT
#define SUCTION_GAIN_ON_THRESHOLD_PERCENT 50U // [%] 以上でFAN_ON用ゲインを適用
#endif

#ifndef KP_ANGLE_FAN_ON
#define KP_ANGLE_FAN_ON 80.0F // 角度制御のP項（ファンON）
#endif
#ifndef KI_ANGLE_FAN_ON
#define KI_ANGLE_FAN_ON 8.0F // 角度制御のI項（ファンON）
#endif
#ifndef KD_ANGLE_FAN_ON
#define KD_ANGLE_FAN_ON 0.0F // 角度制御のD項（ファンON）
#endif

#ifndef KP_ANGLE_FAN_OFF
#define KP_ANGLE_FAN_OFF 20.0F // 角度制御のP項（ファンOFF）
#endif
#ifndef KI_ANGLE_FAN_OFF
#define KI_ANGLE_FAN_OFF 0.1F // 角度制御のI項（ファンOFF）
#endif
#ifndef KD_ANGLE_FAN_OFF
#define KD_ANGLE_FAN_OFF 0.0F // 角度制御のD項（ファンOFF）
#endif

#ifndef TURN_OMEGA_PROFILE_ROUNDING_SCALE
#define TURN_OMEGA_PROFILE_ROUNDING_SCALE 1.2F // >1で滑らか、<1で鋭い
#endif

#ifndef KP_OMEGA_FAN_ON
#define KP_OMEGA_FAN_ON  1.0F  // 角速度制御のP項（ファンON） 0.9F
#endif
#ifndef KI_OMEGA_FAN_ON
#define KI_OMEGA_FAN_ON  0.035F // 角速度制御のI項（ファンON）0.01F
#endif
#ifndef KD_OMEGA_FAN_ON
#define KD_OMEGA_FAN_ON  1.0F  // 角速度制御のD項（ファンON）
#endif

#ifndef KP_OMEGA_FAN_OFF
#define KP_OMEGA_FAN_OFF 0.65F  // 角速度制御のP項（ファンOFF）0.7F
#endif
#ifndef KI_OMEGA_FAN_OFF
#define KI_OMEGA_FAN_OFF 0.008F // 角速度制御のI項（ファンOFF）0.01F
#endif
#ifndef KD_OMEGA_FAN_OFF
#define KD_OMEGA_FAN_OFF 0.0F  // 角速度制御のD項（ファンOFF）
#endif

#define FF_OMEGA 0.0F // 角速度制御のFF項 0.043F

#define KP_IMU 1.0F // IMUの角速度の補正係数

#define FAIL_COUNT_LR  20    // 左右差フェイルセーフ発動までのカウント数[ms]
#define FAIL_LR_ERROR  4000 // 左右差フェイルセーフ発動のモータ出力左右差
#define FAIL_COUNT_ACC 20    // 衝突フェイルセーフ発動までのカウント数[ms]
#define FAIL_ACC       17000 // 衝突フェイルセーフ発動の加速度

#define FAIL_TURN_ANGLE_MARGIN_DEG 90
#define FAIL_TURN_ANGLE_COUNT 2

/*動作方向関連*/

#define DIR_FWD_L  GPIO_PIN_RESET // CW/CCWで前に進む出力（左）
#define DIR_BACK_L GPIO_PIN_SET   // CW/CCWで後ろに進む出力（左）
#define DIR_FWD_R  GPIO_PIN_SET   // CW/CCWで前に進む出力（右）
#define DIR_BACK_R GPIO_PIN_RESET // CW/CCWで後ろに進む出力（右）
#define DIR_ENC_R  -1             // エンコーダ方向（右）
#define DIR_ENC_L  -1             // エンコーダ方向（左）

/*------------------------------------------------------------
    センサ系
------------------------------------------------------------*/
/*壁判断閾値*/
#define WALL_BASE_FR  160   // 前壁右センサ    //700
#define WALL_BASE_FL  160   // 前壁左センサ    //700
#define WALL_BASE_R   200   // 右壁センサ  //800
#define WALL_BASE_L   200   // 左壁センサ  //800
#define WALL_DIFF_THR 22   // 壁センサ値の変化量のしきい値
#define K_SENSOR      1.00F // センサの補正値 0.94F

// センサ距離換算の一括調整係数（全センサ共通、mmスケール）
// LUTから得た距離[mm]に対して、mm_out = SENSOR_DIST_GAIN * mm_in
#ifndef SENSOR_DIST_GAIN
#define SENSOR_DIST_GAIN 1.0F
#endif

// 壁切れ判定専用しきい値（横壁有無判定、ヒステリシス付き）
// High: これを超えたら「壁あり」と判定
// Low: これを下回ったら「壁なし」と判定
#ifndef WALL_END_THR_R_HIGH
#define WALL_END_THR_R_HIGH  280
#endif
#ifndef WALL_END_THR_R_LOW
#define WALL_END_THR_R_LOW   200
#endif
#ifndef WALL_END_THR_L_HIGH
#define WALL_END_THR_L_HIGH  280
#endif
#ifndef WALL_END_THR_L_LOW
#define WALL_END_THR_L_LOW   200
#endif

#define WALL_END_DERIV_FALL_THR 200

// 前壁補正：未検知時の最大延長距離[mm]
#ifndef WALL_END_EXTEND_MAX_MM
#define WALL_END_EXTEND_MAX_MM  10.0F
#endif

#define WALL_CTRL_BASE_L 1941 // 壁制御の基準値（左） 2135
#define WALL_CTRL_BASE_R 1989 // 壁制御の基準値（右） 2100
// 小鷺田寮: L1941 R1989
// 九州: L1861 R2060

// バッテリー電圧の警告しきい値（ADCカウント）
// 例: 3000。割り込み内/起動時チェックで共通利用。
#ifndef BAT_WARN_ADC_THR
#define BAT_WARN_ADC_THR 2150
#endif

/*制御閾値*/
#define CTRL_BASE_L   1     // 左制御閾値
#define CTRL_BASE_R   1     // 右制御閾値
#define WALL_CTRL_MAX 100 // 制御量上限値
#ifndef WALL_CTRL_MIN
#define WALL_CTRL_MIN 0.2F  // 制御量デッドバンド（絶対値がこの値未満なら0）
#endif
#ifndef WALL_LPF_ALPHA
#define WALL_LPF_ALPHA 0.1F // 壁誤差の一次LPF係数（1ms周期） 0.1~0.2推奨
#endif
#ifndef WALL_CTRL_SLEW_MAX
#define WALL_CTRL_SLEW_MAX 5.0F // 壁制御のスルーレート上限[deg/s per 1ms]
#endif
#define KP_DEFAULT    0.1F  // 比例制御係数
#define KP_TURN_AP    0.3F  // スラロームのオフセット区間用比例制御係数

//----赤外線（赤色）LED発光待機時間（単位はマイクロ秒）
#define IR_WAIT_US 30

// 探索中の横壁ズレ検出しきい値（wall_PIDで算出するlatest_wall_error[ADcount]の絶対値）
#define WALL_ALIGN_ERR_THR  700

/* 前壁センサを用いた中央合わせ（非接触）用パラメータ */
// 区画中央における前壁センサの目標値（実機で調整）
#define F_ALIGN_TARGET_FR    1420
#define F_ALIGN_TARGET_FL    1400
// 小鷺田寮: FR3750 FL3790
// 九州: FR3587 FL3587

// アライン実行条件（前壁が十分に見えているか判定する閾値）
#define F_ALIGN_DETECT_THR   500

// 閉ループ制御ゲイン（実機調整用）
#define MATCH_POS_KP_TRANS   -0.4F   // [mm/s] / [ADcount]
#define MATCH_POS_KP_ROT     0.2F   // [deg/s] / [ADcount]

// 飽和・許容値・タイムアウト
#define MATCH_POS_VEL_MAX     200.0F   // [mm/s]
#define MATCH_POS_OMEGA_MAX   300.0F   // [deg/s]
#define MATCH_POS_TOL         100       // [ADcount]
#define MATCH_POS_TOL_ANGLE   40       // [ADcount]
#define MATCH_POS_TIMEOUT_MS  20     // [ms]
// 収束判定：FR/FL が目標±MATCH_POS_TOL 内に連続して入る必要回数（2ms/loop前提）
#define MATCH_POS_STABLE_COUNT 100      // [loop] ≒ 400ms

//============================================================
// 距離ワープ補正（3点アンカー）デフォルト（test_mode case7で使用）
// 近・中・遠の3点。実機で調整したい場合はここを編集してください。
#ifndef SENSOR_WARP_ANCHOR0_MM
#define SENSOR_WARP_ANCHOR0_MM  0.0f
#endif
#ifndef SENSOR_WARP_ANCHOR1_MM
#define SENSOR_WARP_ANCHOR1_MM  26.0f
#endif
#ifndef SENSOR_WARP_ANCHOR2_MM
#define SENSOR_WARP_ANCHOR2_MM  113.0f
#endif

/*------------------------------------------------------------
    探索系
------------------------------------------------------------*/
//----ゴール座標----
#define GOAL_X   13 // 7
#define GOAL_Y   14// 7
#define MAZE_SIZE 32
#define START_X   0
#define START_Y   0

// 複数ゴール設定（最大3x3=9個）。
// 1セル/2x2で使用する場合は未使用分を (0,0) とし、無視します。
// 既定では GOAL1 に従来の GOAL_X/GOAL_Y を入れ、その他は (0,0)。
#ifndef GOAL1_X
#define GOAL1_X GOAL_X
#define GOAL1_Y GOAL_Y
#endif

#ifndef GOAL2_X
#define GOAL2_X 13
#define GOAL2_Y 15
#endif

#ifndef GOAL3_X
#define GOAL3_X 13
#define GOAL3_Y 16
#endif

#ifndef GOAL4_X
#define GOAL4_X 14
#define GOAL4_Y 14
#endif

#ifndef GOAL5_X
#define GOAL5_X 14
#define GOAL5_Y 15
#endif

#ifndef GOAL6_X
#define GOAL6_X 14
#define GOAL6_Y 16
#endif

#ifndef GOAL7_X
#define GOAL7_X 15
#define GOAL7_Y 14
#endif

#ifndef GOAL8_X
#define GOAL8_X 15
#define GOAL8_Y 15
#endif

#ifndef GOAL9_X
#define GOAL9_X 15
#define GOAL9_Y 16
#endif

#endif /* INC_PARAMS_H_ */
