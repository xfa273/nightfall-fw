/*
 * sensor.h
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

/*============================================================
    各種定数・変数宣言
============================================================*/

// IMUモデル種別
#define IMU_MODEL_NONE      0
#define IMU_MODEL_ICM20689  1
#define IMU_MODEL_ISM330    2

#ifdef MAIN_C_ // main.cからこのファイルが呼ばれている場合

/*グローバル変数の定義*/
//----その他----
uint8_t tp;                                         // タスクポインタ
volatile uint16_t ad_r, ad_r_off, ad_fr, ad_fr_off, ad_fl, ad_fl_off, ad_l, ad_l_off, ad_r_raw, ad_l_raw, ad_fr_raw, ad_fl_raw, ad_bat; // A-D値格納
volatile uint16_t wall_offset_r, wall_offset_l, wall_offset_fr, wall_offset_fl; // LED ON時の壁なし基準値
volatile uint16_t base_l, base_r, base_f;           // 基準値を格納
volatile int16_t dif_l, dif_r;                      // AD値と基準との差
float imu_offset_z;

float omega_x_raw, omega_y_raw, omega_z_raw;
float accel_x_raw, accel_y_raw, accel_z_raw;
float omega_x_true, omega_y_true, omega_z_true;
float accel_x_true, accel_y_true, accel_z_true;
float omega_x_offset, omega_y_offset, omega_z_offset;
float accel_x_offset, accel_y_offset, accel_z_offset;

uint8_t set_flag;
uint16_t gyro_calib_cnt;
uint8_t gyro_calib_flag;

// 現在使用しているIMUモデル
uint8_t imu_model;

// 壁切れ検出用変数
volatile bool wall_end_detected_r;   // 右壁切れ検出済みフラグ
volatile bool wall_end_detected_l;   // 左壁切れ検出済みフラグ
volatile float wall_end_dist_r;      // 右壁切れ検出時の走行距離[mm]
volatile float wall_end_dist_l;      // 左壁切れ検出時の走行距離[mm]
volatile bool wall_end_reset_request; // 壁切れ検出状態リセット要求
uint16_t wall_end_thr_r_high;        // 壁切れ検出Highしきい値（右）- 壁ありと判定
uint16_t wall_end_thr_r_low;         // 壁切れ検出Lowしきい値（右）- 壁なしと判定
uint16_t wall_end_thr_l_high;        // 壁切れ検出Highしきい値（左）- 壁ありと判定
uint16_t wall_end_thr_l_low;         // 壁切れ検出Lowしきい値（左）- 壁なしと判定
volatile uint32_t wall_end_rl_update_seq;
volatile uint8_t wall_end_detect_mode;
volatile int32_t wall_end_deriv_r;
volatile int32_t wall_end_deriv_l;

#else // main.c以外からこのファイルが呼ばれている場合

extern uint8_t tp;
extern volatile uint16_t ad_r, ad_r_off, ad_fr, ad_fr_off, ad_fl, ad_fl_off, ad_l, ad_l_off, ad_r_raw, ad_l_raw, ad_fr_raw, ad_fl_raw, ad_bat; // A-D値格納
extern volatile uint16_t wall_offset_r, wall_offset_l, wall_offset_fr, wall_offset_fl; // LED ON時の壁なし基準値
extern volatile uint16_t base_l, base_r, base_f;
extern volatile int16_t dif_l, dif_r;
extern float imu_offset_z;

extern float omega_x_raw, omega_y_raw, omega_z_raw;
extern float accel_x_raw, accel_y_raw, accel_z_raw;
extern float omega_x_true, omega_y_true, omega_z_true;
extern float accel_x_true, accel_y_true, accel_z_true;
extern float omega_x_offset, omega_y_offset, omega_z_offset;
extern float accel_x_offset, accel_y_offset, accel_z_offset;

extern uint8_t set_flag;
extern uint16_t gyro_calib_cnt;
extern uint8_t gyro_calib_flag;

// 現在使用しているIMUモデル
extern uint8_t imu_model;

// 壁切れ検出用変数
extern volatile bool wall_end_detected_r;   // 右壁切れ検出済みフラグ
extern volatile bool wall_end_detected_l;   // 左壁切れ検出済みフラグ
extern volatile float wall_end_dist_r;      // 右壁切れ検出時の走行距離[mm]
extern volatile float wall_end_dist_l;      // 左壁切れ検出時の走行距離[mm]
extern volatile bool wall_end_reset_request; // 壁切れ検出状態リセット要求
extern uint16_t wall_end_thr_r_high;        // 壁切れ検出Highしきい値（右）
extern uint16_t wall_end_thr_r_low;         // 壁切れ検出Lowしきい値（右）
extern uint16_t wall_end_thr_l_high;        // 壁切れ検出Highしきい値（左）
extern uint16_t wall_end_thr_l_low;         // 壁切れ検出Lowしきい値（左）

extern volatile uint32_t wall_end_rl_update_seq;
extern volatile uint8_t wall_end_detect_mode;
extern volatile int32_t wall_end_deriv_r;
extern volatile int32_t wall_end_deriv_l;

#endif

/*============================================================
    関数プロトタイプ宣言
============================================================*/

void sensor_init(void);
int get_adc_value(ADC_HandleTypeDef *, uint32_t);
int get_sensor_value_r(void);
int get_sensor_value_fr(void);
int get_sensor_value_fl(void);
int get_sensor_value_l(void);
int get_battery_value(void);

uint8_t read_byte(uint8_t);
void write_byte(uint8_t, uint8_t);
void ICM20689_Init(void);
float ICM20689_GYRO_READ(uint8_t);
float ICM20689_ACCEL_READ(uint8_t);
void ICM20689_DataUpdate(void);

// ISM330DHCX
void ISM330_Init(void);
void ISM330_DataUpdate(void);

// 自動検出および共通アップデート
void IMU_Init_Auto(void);
void IMU_DataUpdate(void);
// WHO_AM_Iのデバッグプローブ
void IMU_ProbeWHOAMI_Debug(void);
void get_sensor_offsets(void);
void IMU_GetOffset(void);
uint8_t get_base();   // センサ基準値を取得
void get_wall_info(); // 壁情報を読む
void indicate_sensor();
// 壁切れ検知（横壁の立ち下がりエッジ検出）
void detect_wall_end(void);
// 壁切れ検出フラグをリセット（直進開始時に呼び出す）
void wall_end_reset(void);

#define WALL_END_DETECT_MODE_RAW   0u
#define WALL_END_DETECT_MODE_DERIV 1u

void wall_end_set_detect_mode(uint8_t mode);
uint8_t wall_end_get_detect_mode(void);

//============================================================
// センサログ機能（壁切れデバッグ用）
//============================================================
#define SENSOR_LOG_MAX_ENTRIES 500  // 約83ms分（6kHz）

typedef struct {
    uint32_t timestamp;  // タイムスタンプ（us相当のカウンタ）
    uint16_t ad_r;       // 右横センサ
    uint16_t ad_l;       // 左横センサ
    uint16_t ad_fr;      // 右前センサ
    uint16_t ad_fl;      // 左前センサ
    float distance;      // 走行距離[mm]
} SensorLogEntry;

typedef struct {
    SensorLogEntry entries[SENSOR_LOG_MAX_ENTRIES];
    volatile uint16_t head;
    volatile uint16_t count;
    volatile uint8_t logging_active;
    uint32_t start_tick;
} SensorLogBuffer;

#ifdef MAIN_C_
SensorLogBuffer sensor_log_buffer;
#else
extern SensorLogBuffer sensor_log_buffer;
#endif

// センサログ関数
void sensor_log_init(void);
void sensor_log_start(void);
void sensor_log_stop(void);
void sensor_log_capture(void);  // ADCコールバックから呼ばれる
void sensor_log_print(void);

// ADC DMA 連続スキャン用の共有バッファ（9エントリ: R,L,R,L,FR,FL,FR,FL,BAT）
#ifdef MAIN_C_
volatile uint16_t adc_dma_buf_off[9];
volatile uint16_t adc_dma_buf_on[9];
#else
extern volatile uint16_t adc_dma_buf_off[9];
extern volatile uint16_t adc_dma_buf_on[9];
#endif

// ADC DMA ワンショットスキャンを開始（dst に 9サンプルを格納）
HAL_StatusTypeDef sensor_adc_dma_start(volatile uint16_t *dst);

// フラッシュ保存/読込API（迷路用領域とは別セクタに保存）
// センサ基準値・オフセットなどを保存/読込する
// 戻り値: 読込はtrueで有効データ、falseで未初期化または破損
bool sensor_params_load_from_flash(void);
HAL_StatusTypeDef sensor_params_save_to_flash(void);
HAL_StatusTypeDef sensor_recalibrate_and_save(void);

#endif /* INC_SENSOR_H_ */
