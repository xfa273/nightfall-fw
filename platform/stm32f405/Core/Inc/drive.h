/*
 * drive.h
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

/*============================================================
    各種定数・変数宣言
============================================================*/

#define FORWARD 0x00  // 前進向き
#define BACK 0x11     // 後退
#define ROTATE_L 0x01 // 回転向き（左）
#define ROTATE_R 0x10 // 回転向き（右）

//====変数====
#ifdef MAIN_C_ // main.cからこのファイルが呼ばれている場合
/*グローバル変数の定義*/

/*走行パラメータ*/
//  並進用
volatile float velocity_straight;     // 直線の速度[mm/s]
volatile float acceleration_straight; // 直線の加速度[mm/s^2]
volatile float acceleration_straight_dash; // 直線の半区画当たりの加速量[mm/s]
volatile float accel_switch_velocity;     // 加速度切り替え速度[mm/s]（低速/高速域切り替え）
volatile float velocity_d_straight;     // 斜め直線の速度[mm/s]
volatile float acceleration_d_straight; // 斜め直線の加速度[mm/s^2]
volatile float
    acceleration_d_straight_dash; // 斜めs直線の半区画当たりの加速量[mm/s]
volatile float acceleration_turn; //  ターンの減速度[mm/s^2]
volatile float thr_f_wall; // 探索中に停止するための前壁閾値
volatile float duty_setposition; // 壁当てのDuty[%]

// 旋回用
volatile float velocity_turn90; // 90°ターンの速度[mm/s]
volatile float alpha_turn90;    // 90°ターンの角加速度[deg/s^2]
volatile float dist_offset_in;  // 90°ターンの入オフセット距離[mm]
volatile float dist_offset_out; // 90°ターンの出オフセット距離[mm]
volatile float val_offset_in; // 90°ターンの入前壁補正センサー値
volatile float angle_turn_90; // 90°ターンの旋回角度[deg]

// 補正用
volatile float dist_wall_end; // 壁切れ後の直進距離
volatile bool r_wall;         // 右壁の有無
volatile bool l_wall;         // 左壁の有無

// 大回り旋回用
volatile float velocity_l_turn_90; // 90°大回りターンの速度[mm/s]
volatile float alpha_l_turn_90; // 90°大回りターンの角加速度[deg/sec^2]
volatile float angle_l_turn_90; // 90°大回りターンの旋回角度[deg]
volatile float dist_l_turn_in_90;  // 90°大回りターンの入オフセット距離[mm]
volatile float dist_l_turn_out_90; // 90°大回りターンの出オフセット距離[mm]
volatile float velocity_l_turn_180; // 180°大回りターンの速度[mm/s]
volatile float alpha_l_turn_180; // 180°大回りターンの角加速度[deg/sec^2]
volatile float angle_l_turn_180; // 180°大回りターンの旋回角度[deg]
volatile float dist_l_turn_in_180;  // 180°大回りターンの入オフセット距離[mm]
volatile float dist_l_turn_out_180; // 180°大回りターンの出オフセット距離[mm]

// 斜め45度用
volatile float velocity_turn45in; // 45°ターン入りの速度[mm/s]
volatile float alpha_turn45in; // 45°ターン入りの角加速度[deg/sec^2]
volatile float angle_turn45in; // 45°ターン入りの旋回角度[deg]
volatile float dist_turn45in_in;  // 45°ターン入りの前直進距離[mm]
volatile float dist_turn45in_out; // 45°ターン入りの出オフセット距離[mm]
volatile float velocity_turn45out; // 45°ターン出の速度[mm/s]
volatile float alpha_turn45out;    // 45°ターン出の角加速度[deg/sec^2]
volatile float angle_turn45out;    // 45°ターン出の旋回角度[deg]
volatile float dist_turn45out_in; // 45°ターン出の入りオフセット距離[mm]
volatile float dist_turn45out_out; // 45°ターン出の出オフセット距離[mm]

// 斜めV90度用
volatile float velocity_turnV90; // V90°ターンの速度[mm/s]
volatile float alpha_turnV90;    // V90°ターンの角加速度[deg/sec^2]
volatile float angle_turnV90;    // V90°ターンの旋回角度[deg]
volatile float dist_turnV90_in; // V90°ターンの入りオフセット距離[mm]
volatile float dist_turnV90_out; // V90°ターンの出オフセット距離[mm]

// 斜め135度用
volatile float velocity_turn135in; // 135°ターン入りの速度[mm/s]
volatile float alpha_turn135in; // 135°ターン入りの角加速度[deg/sec^2]
volatile float angle_turn135in; // 135°ターン入りの旋回角度[deg]
volatile float dist_turn135in_in; // 135°ターン入りの入りオフセット距離[mm]
volatile float dist_turn135in_out; // 135°ターン入りの出オフセット距離[mm]
volatile float velocity_turn135out; // 135°ターン出の速度[mm/s]
volatile float alpha_turn135out; // 135°ターン出の角加速度[deg/sec^2]
volatile float angle_turn135out; // 135°ターン出の旋回角度[deg]
volatile float dist_turn135out_in; // 135°ターン出の入りオフセット距離[mm]
volatile float dist_turn135out_out; // 135°ターン出の出オフセット距離[mm]

/*現在の値を保持する用*/
volatile float speed_now; // 現在の並進速度[mm/s]
volatile float omega_now; // 現在の角速度[deg/s]

/*モータ駆動用*/
volatile float out_r;
volatile float out_l;

/*パス→動作に使う変数*/
volatile float velocity_next_turn; // 次のターンの並進速度[mm/s]

/*目標位置生成用*/
volatile float acceleration_interrupt; // 割込み内の計算用の並進加速度[mm/s^2]
volatile float velocity_interrupt; // 割込み内の計算用の並進速度[mm/s]
volatile float velocity_profile_target;
volatile uint8_t velocity_profile_clamp_enabled;
volatile float target_distance; // 割込み内の計算用の並進位置[mm]

/*目標角度生成用*/
volatile float alpha_interrupt; // 割込み内の計算用の並進角加速度[deg/s^2]
volatile float omega_interrupt; // 割込み内の計算用の角速度[deg/s]
volatile float target_angle;    // 割込み内の計算用の角度[deg]

/*並進位置制御用*/
volatile float real_distance;        // 実際の並進距離[mm]
volatile float distance_error;       // 並進距離の偏差[mm]
volatile float distance_error_error; // 並進距離の偏差の偏差[mm]
volatile float previous_distance_error; // 1ループ前の並進距離の偏差[mm]
volatile float distance_integral;       // 並進距離の積分項

/*並進速度制御用*/
volatile float target_velocity;      // 目標並進速度[mm/s]
volatile float real_velocity;        // 実際の並進速度[mm/s]
volatile float velocity_error;       // 並進速度の偏差[mm/s]
volatile float velocity_error_error; // 並進速度の偏差の偏差[mm/s]
volatile float previous_velocity_error; // 1ループ前の並進速度の偏差[mm]
volatile float velocity_integral;       // 並進速度の積分項
volatile float out_translation;         // 並進方向の出力

/*角度制御用*/
volatile float real_angle;           // 実際の角度[deg]
volatile float angle_error;          // 角度の偏差[deg]
volatile float angle_error_error;    // 角度の偏差の偏差[deg]
volatile float previous_angle_error; // 1ループ前の角度の偏差[deg]
volatile float angle_integral;       // 角度の積分項

/*角速度制御用*/
volatile float target_omega;         // 目標角速度[deg/s]
volatile float real_omega;           // 実際の角速度[deg/s]
volatile float omega_error;          // 角速度の偏差[deg/s]
volatile float omega_error_error;    // 角速度の偏差の偏差[deg/s]
volatile float previous_omega_error; // 1ループ前の角速度の偏差[deg/s]
volatile float omega_integral;       // 角速度の積分項
volatile float out_rotate;           // 回転方向の出力

/*エンコーダからの速度取得用*/
volatile float encoder_count_r; // エンコーダのパルスカウント（右）
volatile float encoder_count_l; // エンコーダのパルスカウント（左）
volatile float
    previous_encoder_count_r; // 1ループ前のエンコーダのパルスカウント（右）
volatile float
    previous_encoder_count_l; // 1ループ前のエンコーダのパルスカウント（左）
volatile float encoder_speed_r; // エンコーダから取得した速度（右）
volatile float encoder_speed_l; // エンコーダから取得した速度（左）
volatile float encoder_distance_r; // エンコーダから取得した距離（右）
volatile float encoder_distance_l; // エンコーダから取得した距離（左）

/*IMUからの角度取得用*/
volatile float IMU_angle; // IMUから取得した角度[deg]

/*壁制御用*/
volatile float wall_control;
volatile float previous_ad_r;
volatile float previous_ad_l;
volatile float kp_wall; // 壁制御ゲイン
volatile float latest_wall_error; // 壁偏差（探索側トリガ用）

/*斜め制御用*/
volatile float diagonal_control_thr;
volatile float kp_diagonal;
volatile float diagonal_control;

/*櫛対策（前壁センサ左右非対称補正）*/
volatile float kushi_control;

/*フェイルセーフ用*/
volatile uint16_t fail_count_lr;
volatile uint16_t fail_count_acc;
volatile float IMU_acceleration;

/*割込内カウンター*/
volatile uint16_t buzzer_count;
volatile uint16_t wall_end_count;


#else // main.c以外からこのファイルが呼ばれている場合
/*グローバル変数の定義*/

/*走行パラメータ*/
//  並進用
extern volatile float velocity_straight;     // 直線の速度[mm/s]
extern volatile float acceleration_straight; // 直線の加速度[mm/s^2]
extern volatile float acceleration_straight_dash; // 直線の半区画当たりの加速量[mm/s]
extern volatile float accel_switch_velocity;     // 加速度切り替え速度[mm/s]
extern volatile float velocity_d_straight;     // 斜め直線の速度[mm/s]
extern volatile float acceleration_d_straight; // 斜め直線の加速度[mm/s^2]
extern volatile float acceleration_d_straight_dash; // 斜めs直線の半区画当たりの加速量[mm/s]
extern volatile float acceleration_turn; //  ターンの減速度[mm/s^2]
extern volatile float thr_f_wall; // 探索中に停止するための前壁閾値
extern volatile float duty_setposition; // 壁当てのDuty[%]

// 旋回用
extern volatile float velocity_turn90; // 90°ターンの速度[mm/s]
extern volatile float alpha_turn90;    // 90°ターンの角加速度[deg/s^2]
extern volatile float dist_offset_in; // 90°ターンの入オフセット距離[mm]
extern volatile float dist_offset_out; // 90°ターンの出オフセット距離[mm]
extern volatile float val_offset_in; // 90°ターンの入前壁補正センサー値
extern volatile float angle_turn_90; // 90°ターンの旋回角度[deg]

// 補正用
extern volatile float dist_wall_end; // 壁切れ後の直進距離
extern volatile bool r_wall;         // 右壁の有無
extern volatile bool l_wall;         // 左壁の有無

// 大回り旋回用
extern volatile float velocity_l_turn_90; // 90°大回りターンの速度[mm/s]
extern volatile float alpha_l_turn_90; // 90°大回りターンの角加速度[deg/sec^2]
extern volatile float angle_l_turn_90; // 90°大回りターンの旋回角度[deg]
extern volatile float dist_l_turn_in_90;  // 90°大回りターンの入オフセット距離[mm]
extern volatile float dist_l_turn_out_90; // 90°大回りターンの出オフセット距離[mm]
extern volatile float velocity_l_turn_180; // 180°大回りターンの速度[mm/s]
extern volatile float alpha_l_turn_180; // 180°大回りターンの角加速度[deg/sec^2]
extern volatile float angle_l_turn_180; // 180°大回りターンの旋回角度[deg]
extern volatile float dist_l_turn_in_180;  // 180°大回りターンの入オフセット距離[mm]
extern volatile float dist_l_turn_out_180; // 180°大回りターンの出オフセット距離[mm]

// 斜め45度用
extern volatile float velocity_turn45in; // 45°ターン入りの速度[mm/s]
extern volatile float alpha_turn45in; // 45°ターン入りの角加速度[deg/sec^2]
extern volatile float angle_turn45in; // 45°ターン入りの旋回角度[deg]
extern volatile float dist_turn45in_in;  // 45°ターン入りの前直進距離[mm]
extern volatile float dist_turn45in_out; // 45°ターン入りの出オフセット距離[mm]
extern volatile float velocity_turn45out; // 45°ターン出の速度[mm/s]
extern volatile float alpha_turn45out; // 45°ターン出の角加速度[deg/sec^2]
extern volatile float angle_turn45out; // 45°ターン出の旋回角度[deg]
extern volatile float dist_turn45out_in; // 45°ターン出の入りオフセット距離[mm]
extern volatile float dist_turn45out_out; // 45°ターン出の出オフセット距離[mm]

// 斜めV90度用
extern volatile float velocity_turnV90; // V90°ターンの速度[mm/s]
extern volatile float alpha_turnV90; // V90°ターンの角加速度[deg/sec^2]
extern volatile float angle_turnV90; // V90°ターンの旋回角度[deg]
extern volatile float dist_turnV90_in; // V90°ターンの入りオフセット距離[mm]
extern volatile float dist_turnV90_out; // V90°ターンの出オフセット距離[mm]

// 斜め135度用
extern volatile float velocity_turn135in; // 135°ターン入りの速度[mm/s]
extern volatile float alpha_turn135in; // 135°ターン入りの角加速度[deg/sec^2]
extern volatile float angle_turn135in; // 135°ターン入りの旋回角度[deg]
extern volatile float dist_turn135in_in; // 135°ターン入りの入りオフセット距離[mm]
extern volatile float dist_turn135in_out; // 135°ターン入りの出オフセット距離[mm]
extern volatile float velocity_turn135out; // 135°ターン出の速度[mm/s]
extern volatile float alpha_turn135out; // 135°ターン出の角加速度[deg/sec^2]
extern volatile float angle_turn135out; // 135°ターン出の旋回角度[deg]
extern volatile float dist_turn135out_in; // 135°ターン出の入りオフセット距離[mm]
extern volatile float dist_turn135out_out; // 135°ターン出の出オフセット距離[mm]

/*現在の値を保持する用*/
extern volatile float speed_now; // 現在の並進速度[mm/s]
extern volatile float omega_now; // 現在の角速度[deg/s]

/*モータ駆動用*/
extern volatile float out_r;
extern volatile float out_l;

/*パス→動作に使う変数*/
extern volatile float velocity_next_turn; // 次のターンの並進速度[mm/s]

/*目標位置生成用*/
extern volatile float acceleration_interrupt; // 割込み内の計算用の並進加速度[mm/s^2]
extern volatile float velocity_interrupt; // 割込み内の計算用の並進速度[mm/s]
extern volatile float velocity_profile_target;
extern volatile uint8_t velocity_profile_clamp_enabled;
extern volatile float target_distance; // 割込み内の計算用の並進位置[mm]

/*目標角度生成用*/
extern volatile float alpha_interrupt; // 割込み内の計算用の並進角加速度[deg/s^2]
extern volatile float omega_interrupt; // 割込み内の計算用の角速度[deg/s]
extern volatile float target_angle; // 割込み内の計算用の角度[deg]

/*並進位置制御用*/
extern volatile float real_distance;        // 実際の並進距離[mm]
extern volatile float distance_error;       // 並進距離の偏差[mm]
extern volatile float distance_error_error; // 並進距離の偏差の偏差[mm]
extern volatile float previous_distance_error; // 1ループ前の並進距離の偏差[mm]
extern volatile float distance_integral; // 並進距離の積分項

/*並進速度制御用*/
extern volatile float target_velocity;      // 目標並進速度[mm/s]
extern volatile float real_velocity;        // 実際の並進速度[mm/s]
extern volatile float velocity_error;       // 並進速度の偏差[mm/s]
extern volatile float velocity_error_error; // 並進速度の偏差の偏差[mm/s]
extern volatile float previous_velocity_error; // 1ループ前の並進速度の偏差[mm]
extern volatile float velocity_integral; // 並進速度の積分項
extern volatile float out_translation;   // 並進方向の出力

/*角度制御用*/
extern volatile float real_angle;           // 実際の角度[deg]
extern volatile float angle_error;          // 角度の偏差[deg]
extern volatile float angle_error_error;    // 角度の偏差の偏差[deg]
extern volatile float previous_angle_error; // 1ループ前の角度の偏差[deg]
extern volatile float angle_integral;       // 角度の積分項

/*角速度制御用*/
extern volatile float target_omega;      // 目標角速度[deg/s]
extern volatile float real_omega;        // 実際の角速度[deg/s]
extern volatile float omega_error;       // 角速度の偏差[deg/s]
extern volatile float omega_error_error; // 角速度の偏差の偏差[deg/s]
extern volatile float previous_omega_error; // 1ループ前の角速度の偏差[deg/s]
extern volatile float omega_integral; // 角速度の積分項
extern volatile float out_rotate;     // 回転方向の出力

/*エンコーダからの速度取得用*/
extern volatile float encoder_count_r; // エンコーダのパルスカウント（右）
extern volatile float encoder_count_l; // エンコーダのパルスカウント（左）
extern volatile float previous_encoder_count_r; // 1ループ前のエンコーダのパルスカウント（右）
extern volatile float previous_encoder_count_l; // 1ループ前のエンコーダのパルスカウント（左）
extern volatile float encoder_speed_r; // エンコーダから取得した速度（右）[mm/s]
extern volatile float encoder_speed_l; // エンコーダから取得した速度（左）[mm/s]
extern volatile float encoder_distance_r; // エンコーダから取得した距離（右）[mm]
extern volatile float encoder_distance_l; // エンコーダから取得した距離（左）[mm]

/*IMUからの角度取得用*/
extern volatile float IMU_angle; // IMUから取得した角度[deg]

/*壁制御用*/
extern volatile float wall_control;
extern volatile float previous_ad_r;
extern volatile float previous_ad_l;
extern volatile float kp_wall; // 壁制御ゲイン
extern volatile float latest_wall_error; // 壁偏差（探索側トリガ用）
extern volatile uint32_t fan_last_off_ms; // 最後にファンを停止した時刻（ms）

/*斜め制御用*/
extern volatile float diagonal_control_thr;
extern volatile float kp_diagonal;
extern volatile float diagonal_control;

/*櫛対策（前壁センサ左右非対称補正）*/
extern volatile float kushi_control;

/*フェイルセーフ用*/
extern volatile uint16_t fail_count_lr;
extern volatile uint16_t fail_count_acc;
extern volatile float IMU_acceleration;

/*割込内カウンター*/
extern volatile uint16_t buzzer_count;
extern volatile uint16_t wall_end_count;


#endif

#define drive_wait() HAL_Delay(200)

/*============================================================
    関数プロトタイプ宣言
============================================================*/
void drive_init(void);
void drive_variable_reset(void);
void drive_reset_before_run(void);
void drive_enable_motor(void);
void drive_disable_motor(void);
void drive_start(void);
void drive_stop(void);
void drive_brake(bool enable);

void drive_set_dir(uint8_t); // 進む方向の設定
void drive_motor(void);

void drive_fan(uint16_t);


//====走行系====
//----基幹関数----
void driveA(float, float, float, float); // 並進加減速走行
void driveR(float);                      // 等速走行
void driveSR(float, float);
void driveSL(float, float);
void driveR(float);
void driveFWall(float, float, float);
bool driveC_wallend(float, float);       // 等速走行（壁切れ検出で即終了）

//----上位関数----
void first_sectionA(void);     // 最初の一区画
void half_sectionA(uint16_t);  // 加速半区画
void half_sectionD(uint16_t);  // 減速半区画
void half_sectionD(uint16_t val);
void half_sectionDD(uint16_t val);
void half_sectionU(void);
void reverse_distance(float distance_mm);
void one_sectionU(float section, float spd_out);
void diag_sectionU(float section, float spd_out); // 等速一区画
void one_sectionA(void);
void one_sectionD(void);
void run_straight(float, float, float); // 指定区画を指定速度で走行
void rotate_R90(void);                  // 右90回転
void rotate_L90(void);                  // 左90回転
void rotate_180(void);                  // 180度回転
void turn_R90(uint8_t);                 // 90度スラロームで右旋回
void turn_L90(uint8_t);                 // 90度スラロームで左旋回
void l_turn_R90(bool next_is_large);    // 90度大回り右旋回（次が大回りなら壁切れ補正）
void l_turn_L90(bool next_is_large);    // 90度大回り左旋回（次が大回りなら壁切れ補正）
void l_turn_R180(bool next_is_large);   // 180度大回り右旋回（次が大回りなら壁切れ補正）
void l_turn_L180(bool next_is_large);   // 180度大回り左旋回（次が大回りなら壁切れ補正）
void turn_R45_In(void);                 // 右45度入り
void turn_R45_Out(void);                // 右45度出
void turn_L45_In(void);                 // 左45度入り
void turn_L45_Out(void);                // 左45度出
void turn_RV90(void);                   // 右V90度
void turn_LV90(void);                   // 左V90度
void turn_R135_In(void);                // 右135度入り
void turn_R135_Out(void);               // 右135度出
void turn_L135_In(void);                // 左135度入り
void turn_L135_Out(void);               // 左135度出
void run_diagonal(float, float); // 指定区画を指定速度で斜め走行
void turn_setposition_R90(void);
void turn_setposition_L90(void);
void set_position(void); // 位置合わせ
void match_position(uint16_t);
void test_run(void);     // テスト走行
void reset_failed(void); // フェイルセーフ時のリセット

#endif /* INC_DRIVE_H_ */
