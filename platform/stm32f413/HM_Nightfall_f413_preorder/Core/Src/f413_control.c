/**
 * @file f413_control.c
 * @brief F413 クローズドループ速度/角速度制御
 *
 * TIM5 1kHz 割り込みで以下を実行する:
 *   1. TIM3(左)/TIM4(右) エンコーダの差分から並進速度を算出
 *   2. ISM330DHCX (SPI2) から角速度を読み取り（IMUベース）
 *   3. P+FF 制御で並進/回転の出力を計算
 *   4. TIM2 CH1(左)/CH3(右) PWM + GPIO 方向ピンでモータを駆動
 *
 * 旋回制御は F405 と同様に IMU を正本とし、エンコーダは並進のみ使用する。
 */

#include "f413_control.h"
#include "main.h"
#include "params.h"
#include <math.h>
#include <string.h>

/* ---------- ハードウェア定数 ---------- */
#define F413_CTRL_DT              (0.001f)   /* 制御周期 [s] (1kHz) */
#define F413_CTRL_ENCODER_CENTER  (30000U)   /* エンコーダカウンタ中央値 */
#define F413_CTRL_CPR_WHEEL       (400.0f)   /* NJL5820 100PPR × 4逓倍 */
#define F413_CTRL_D_TIRE          (13.35f)   /* タイヤ直径 [mm] */
#define F413_CTRL_TREAD           (33.5f)    /* 左右タイヤ中心間距離 [mm] */
#define F413_CTRL_PWM_MAX         (1000U)    /* TIM2 ARR */
#define F413_CTRL_ENCODER_SIGN_L  (1.0f)
#define F413_CTRL_ENCODER_SIGN_R  (-1.0f)

/* ---------- 制御ゲイン ---------- */
#define F413_CTRL_KP_VEL          (0.5f)     /* [PWM counts / (mm/s)] */
#define F413_CTRL_KP_ANGLE        (3.0f)     /* [deg/s / deg] */
#define F413_CTRL_KI_ANGLE        (0.0f)
#define F413_CTRL_KD_ANGLE        (0.0f)
#define F413_CTRL_ANGLE_OMEGA_MAX (300.0f)
#define F413_CTRL_KP_OMEGA        (1.4f)     /* [PWM counts / (deg/s)] */
#define F413_CTRL_KI_OMEGA        (0.06f)
#define F413_CTRL_KD_OMEGA        (0.0f)
#define F413_CTRL_FF_VEL          (0.8f)     /* フィードフォワード速度→PWM (CPR=400用) */
#define F413_CTRL_FF_OMEGA        (0.45f)    /* フィードフォワード角速度→PWM */
#define F413_CTRL_OMEGA_I_LIMIT   (6000.0f)
#define F413_CTRL_ROT_PWM_MIN     (100.0f)
#define F413_CTRL_ROT_MIN_OMEGA_REF (10.0f)
#define F413_CTRL_ROT_MIN_VEL_ABS (1.0f)
#ifndef CTRL_ENABLE_ANTI_WINDUP
#define CTRL_ENABLE_ANTI_WINDUP 0
#endif
#ifndef CTRL_OUTPUT_MAX
#define CTRL_OUTPUT_MAX 1000.0f
#endif
#ifndef CTRL_DISTANCE_OUTER_DIV
#define CTRL_DISTANCE_OUTER_DIV 1
#endif
#if (CTRL_DISTANCE_OUTER_DIV < 1)
#undef CTRL_DISTANCE_OUTER_DIV
#define CTRL_DISTANCE_OUTER_DIV 1
#endif
#ifndef CTRL_ANGLE_OUTER_DIV
#define CTRL_ANGLE_OUTER_DIV 1
#endif
#if (CTRL_ANGLE_OUTER_DIV < 1)
#undef CTRL_ANGLE_OUTER_DIV
#define CTRL_ANGLE_OUTER_DIV 1
#endif

/* ---------- IMU (ISM330DHCX) 定数 ---------- */
#define F413_IMU_WHO_AM_I_REG     (0x0FU)
#define F413_IMU_WHO_AM_I_VAL     (0x6BU)
#define F413_IMU_CTRL1_XL         (0x10U)    /* 加速度設定 */
#define F413_IMU_CTRL2_G          (0x11U)    /* ジャイロ設定 */
#define F413_IMU_CTRL3_C          (0x12U)    /* 制御レジスタ3 */
#define F413_IMU_OUTZ_G_L         (0x26U)    /* ジャイロ Z軸 LOW byte */
#define F413_IMU_OUTZ_G_H         (0x27U)    /* ジャイロ Z軸 HIGH byte */
#define F413_IMU_OUTX_XL_L        (0x28U)
#define F413_IMU_OUTY_XL_L        (0x2AU)
#define F413_IMU_GYRO_SENSITIVITY (0.14f)    /* FS=4000dps → 140mdps/LSB [deg/s/LSB] */
#define F413_IMU_ACCEL_SENS_MG    (0.488f)
#define F413_IMU_GRAVITY_MM_S2    (9.80665f)
#define F413_IMU_OMEGA_LPF_TAU    (0.020f)   /* 角速度 LPF 時定数 [s] (振動ノイズ除去) */
#define F413_IMU_ACCEL_LPF_TAU    (0.010f)
#define F413_IMU_OFFSET_SAMPLES   (500U)     /* オフセット測定回数 */
#define F413_IMU_OFFSET_SETTLE_MS (200U)     /* 静定待ち [ms] */
#define F413_IMU_FORWARD_ACCEL_REG  (F413_IMU_OUTY_XL_L)
#define F413_IMU_FORWARD_ACCEL_SIGN (1.0f)
#define F413_CTRL_VEL_CORR_TAU    (0.035f)
#define F413_CTRL_VEL_EST_MAX     (1200.0f)

/* ---------- エンコーダ変換定数 ---------- */
static const float s_enc_to_mm =
    (F413_CTRL_D_TIRE * 3.14159265f) / F413_CTRL_CPR_WHEEL;

/* ---------- HAL ハンドル (main.c で定義) ---------- */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern SPI_HandleTypeDef hspi2;

/* ---------- 状態変数 ---------- */
static volatile bool s_running = false;
static bool s_imu_ok = false;

static volatile float s_acceleration_interrupt = 0.0f;
static volatile float s_velocity_interrupt = 0.0f;
static volatile float s_velocity_profile_target = 0.0f;
static volatile uint8_t s_velocity_profile_clamp_enabled = 0U;
static volatile float s_omega_interrupt = 0.0f;
static volatile float s_target_distance = 0.0f;
static volatile float s_target_velocity = 0.0f;
static volatile float s_target_angle = 0.0f;
static volatile float s_target_omega = 0.0f;
static volatile float s_heading_omega_correction = 0.0f;
static volatile bool s_angle_target_enabled = false;

static volatile float s_real_distance = 0.0f;
static volatile float s_real_velocity = 0.0f;
static volatile float s_real_angle = 0.0f;
static volatile float s_real_omega = 0.0f;
static volatile float s_encoder_distance_l = 0.0f;
static volatile float s_encoder_distance_r = 0.0f;
static volatile float s_encoder_speed_l = 0.0f;
static volatile float s_encoder_speed_r = 0.0f;
static volatile float s_distance_error = 0.0f;
static volatile float s_distance_error_error = 0.0f;
static volatile float s_previous_distance_error = 0.0f;
static volatile float s_distance_integral = 0.0f;
static volatile float s_velocity_error = 0.0f;
static volatile float s_velocity_error_error = 0.0f;
static volatile float s_previous_velocity_error = 0.0f;
static volatile float s_velocity_integral = 0.0f;
static volatile float s_angle_error = 0.0f;
static volatile float s_angle_error_error = 0.0f;
static volatile float s_previous_angle_error = 0.0f;
static volatile float s_angle_integral = 0.0f;
static volatile float s_omega_error = 0.0f;
static volatile float s_omega_error_error = 0.0f;
static volatile float s_previous_omega_error = 0.0f;
static volatile float s_omega_integral = 0.0f;
static volatile float s_out_translation = 0.0f;
static volatile float s_out_rotate = 0.0f;
static volatile int16_t s_motor_out_l = 0;
static volatile int16_t s_motor_out_r = 0;

static float s_omega_z_offset  = 0.0f;
static float s_omega_z_filtered = 0.0f;
static bool  s_omega_z_lpf_inited = false;
static float s_accel_forward_offset = 0.0f;
static float s_accel_forward_filtered = 0.0f;
static bool  s_accel_forward_lpf_inited = false;
static float s_accel_velocity = 0.0f;
static float s_real_velocity_lpf = 0.0f;
static bool s_real_velocity_lpf_inited = false;
static uint16_t s_distance_outer_count = 0U;
static float s_distance_velocity_feedback = 0.0f;
static uint16_t s_angle_outer_count = 0U;

static volatile bool s_spi2_busy = false;

static bool f413_ctrl_use_fan_on_gains(void)
{
    return false;
}

static void f413_ctrl_reset_pid_state(void)
{
    s_target_distance = 0.0f;
    s_target_velocity = 0.0f;
    s_target_angle = 0.0f;
    s_target_omega = 0.0f;
    s_heading_omega_correction = 0.0f;
    s_real_distance = 0.0f;
    s_real_velocity = 0.0f;
    s_real_angle = 0.0f;
    s_real_omega = 0.0f;
    s_encoder_distance_l = 0.0f;
    s_encoder_distance_r = 0.0f;
    s_encoder_speed_l = 0.0f;
    s_encoder_speed_r = 0.0f;
    s_distance_error = 0.0f;
    s_distance_error_error = 0.0f;
    s_previous_distance_error = 0.0f;
    s_distance_integral = 0.0f;
    s_velocity_error = 0.0f;
    s_velocity_error_error = 0.0f;
    s_previous_velocity_error = 0.0f;
    s_velocity_integral = 0.0f;
    s_angle_error = 0.0f;
    s_angle_error_error = 0.0f;
    s_previous_angle_error = 0.0f;
    s_angle_integral = 0.0f;
    s_omega_error = 0.0f;
    s_omega_error_error = 0.0f;
    s_previous_omega_error = 0.0f;
    s_omega_integral = 0.0f;
    s_out_translation = 0.0f;
    s_out_rotate = 0.0f;
    s_motor_out_l = 0;
    s_motor_out_r = 0;
    s_omega_z_filtered = 0.0f;
    s_omega_z_lpf_inited = false;
    s_accel_forward_filtered = 0.0f;
    s_accel_forward_lpf_inited = false;
    s_accel_velocity = 0.0f;
    s_real_velocity_lpf = 0.0f;
    s_real_velocity_lpf_inited = false;
    s_distance_outer_count = 0U;
    s_distance_velocity_feedback = 0.0f;
    s_angle_outer_count = 0U;
}

static void f413_ctrl_reset_profile_state(void)
{
    s_acceleration_interrupt = 0.0f;
    s_velocity_interrupt = 0.0f;
    s_velocity_profile_target = 0.0f;
    s_velocity_profile_clamp_enabled = 0U;
    s_omega_interrupt = 0.0f;
    s_heading_omega_correction = 0.0f;
    s_angle_target_enabled = false;
}

/* ========================================================== */
/* IMU SPI helpers (ISM330DHCX on SPI2, CS = IMU_CS)          */
/* ========================================================== */

static uint8_t imu_read_byte(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80U), 0x00U };
    uint8_t rx[2] = { 0U, 0U };

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, 10U);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return rx[1];
}

static void imu_write_byte(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), val };

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_Transmit(&hspi2, tx, 2U, 10U);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static int16_t imu_read16_le(uint8_t reg_l)
{
    /* バーストリード: 1回のCS cycleで lo+hi を連続読取。
       ISM330DHCX は IF_INC=1 (CTRL3_C) なので自動アドレスインクリメントされ、
       BDU=1 により読取完了まで出力レジスタが更新されない。 */
    uint8_t tx[3] = { (uint8_t)(reg_l | 0x80U), 0x00U, 0x00U };
    uint8_t rx[3] = { 0U, 0U, 0U };

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(&hspi2, tx, rx, 3U, 10U);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    return (int16_t)((uint16_t)rx[2] << 8U | (uint16_t)rx[1]);
}

static float imu_read_gyro_z_dps(void)
{
    return (float)imu_read16_le(F413_IMU_OUTZ_G_L) * F413_IMU_GYRO_SENSITIVITY;
}

static float imu_read_accel_forward_mm_s2(void)
{
    return (float)imu_read16_le(F413_IMU_FORWARD_ACCEL_REG) *
           F413_IMU_ACCEL_SENS_MG *
           F413_IMU_GRAVITY_MM_S2 *
           F413_IMU_FORWARD_ACCEL_SIGN;
}

static bool imu_init_ism330(void)
{
    uint8_t who = imu_read_byte(F413_IMU_WHO_AM_I_REG);
    if (who != F413_IMU_WHO_AM_I_VAL)
    {
        return false;
    }

    /* CTRL3_C: BDU=1, IF_INC=1 */
    imu_write_byte(F413_IMU_CTRL3_C, 0x44U);
    HAL_Delay(10U);

    /* CTRL2_G: ODR=833Hz, FS=4000dps (0x70 | 0x01 = 0x71) */
    imu_write_byte(F413_IMU_CTRL2_G, 0x71U);
    HAL_Delay(10U);

    /* CTRL1_XL: ODR=833Hz, FS=±16g (0x7C) */
    imu_write_byte(F413_IMU_CTRL1_XL, 0x7CU);
    HAL_Delay(10U);

    return true;
}

static void imu_get_motion_offsets(void)
{
    float gyro_sum = 0.0f;
    float accel_sum = 0.0f;
    uint32_t i;

    HAL_Delay(F413_IMU_OFFSET_SETTLE_MS);

    for (i = 0U; i < F413_IMU_OFFSET_SAMPLES; i++)
    {
        gyro_sum += imu_read_gyro_z_dps();
        accel_sum += imu_read_accel_forward_mm_s2();
        HAL_Delay(1U);
    }

    s_omega_z_offset = gyro_sum / (float)F413_IMU_OFFSET_SAMPLES;
    s_omega_z_filtered = 0.0f;
    s_omega_z_lpf_inited = false;
    s_accel_forward_offset = accel_sum / (float)F413_IMU_OFFSET_SAMPLES;
    s_accel_forward_filtered = 0.0f;
    s_accel_forward_lpf_inited = false;
    s_accel_velocity = 0.0f;
}

/* ========================================================== */
/* Public API                                                  */
/* ========================================================== */

void f413_ctrl_init(void)
{
    s_running = false;
    f413_ctrl_reset_profile_state();
    f413_ctrl_reset_pid_state();
    s_accel_forward_offset = 0.0f;

    /* IMU 初期化 + ジャイロオフセット取得 */
    s_imu_ok = imu_init_ism330();
    if (s_imu_ok)
    {
        imu_get_motion_offsets();
    }

    /* エンコーダ開始 */
    __HAL_TIM_SET_COUNTER(&htim3, F413_CTRL_ENCODER_CENTER);
    __HAL_TIM_SET_COUNTER(&htim4, F413_CTRL_ENCODER_CENTER);
    (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* TIM5 割り込み開始 (1kHz) */
    (void)HAL_TIM_Base_Start_IT(&htim5);
}

void f413_ctrl_start(void)
{
    /* 走行直前に IMU オフセットを再取得（静止状態で校正） */
    if (s_imu_ok)
    {
        imu_get_motion_offsets();
    }

    /* 状態をリセットしてから有効化 */
    f413_ctrl_reset_profile_state();
    f413_ctrl_reset_pid_state();

    __HAL_TIM_SET_COUNTER(&htim3, F413_CTRL_ENCODER_CENTER);
    __HAL_TIM_SET_COUNTER(&htim4, F413_CTRL_ENCODER_CENTER);

    /* モータ有効化 (IN1/IN2=0 で停止状態にしてから STBY = HIGH, PWM開始) */
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_RESET);
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);

    s_running = true;
}

void f413_ctrl_stop(void)
{
    s_running = false;
    f413_ctrl_reset_profile_state();
    s_target_velocity = 0.0f;
    s_target_omega    = 0.0f;
    s_out_translation = 0.0f;
    s_out_rotate = 0.0f;
    s_velocity_integral = 0.0f;
    s_omega_integral = 0.0f;
    s_motor_out_l = 0;
    s_motor_out_r = 0;

    /* モータ停止 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
    HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_RESET);
    (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
}

void f413_ctrl_set_velocity(float velocity_mm_s)
{
    s_velocity_interrupt = velocity_mm_s;
    s_target_velocity = velocity_mm_s;
}

void f413_ctrl_set_omega(float omega_deg_s)
{
    s_omega_interrupt = omega_deg_s;
    s_target_omega = 0.0f;
}

void f413_ctrl_set_angle_target(float angle_deg)
{
    s_target_angle = angle_deg;
    s_angle_target_enabled = true;
    s_target_omega = 0.0f;
    s_angle_integral = 0.0f;
    s_previous_angle_error = 0.0f;
    s_angle_outer_count = 0U;
}

void f413_ctrl_clear_angle_target(void)
{
    s_angle_target_enabled = false;
    s_target_omega = 0.0f;
    s_angle_integral = 0.0f;
    s_previous_angle_error = 0.0f;
    s_angle_outer_count = 0U;
}

void f413_ctrl_set_heading_omega_correction(float omega_deg_s)
{
    if (omega_deg_s > WALL_CTRL_MAX)
    {
        omega_deg_s = WALL_CTRL_MAX;
    }
    else if (omega_deg_s < -WALL_CTRL_MAX)
    {
        omega_deg_s = -WALL_CTRL_MAX;
    }
    s_heading_omega_correction = omega_deg_s;
}

float f413_ctrl_get_distance(void)      { return s_real_distance; }
float f413_ctrl_get_angle(void)         { return s_real_angle; }
void  f413_ctrl_reset_distance(void)
{
    s_real_distance = 0.0f;
    s_encoder_distance_l = 0.0f;
    s_encoder_distance_r = 0.0f;
    s_target_distance = 0.0f;
    s_distance_error = 0.0f;
    s_distance_error_error = 0.0f;
    s_previous_distance_error = 0.0f;
    s_distance_integral = 0.0f;
    s_distance_velocity_feedback = 0.0f;
    s_distance_outer_count = 0U;
}
void  f413_ctrl_reset_angle(void)
{
    s_real_angle = 0.0f;
    s_target_angle = 0.0f;
    s_angle_integral = 0.0f;
    s_previous_angle_error = 0.0f;
    s_angle_error = 0.0f;
    s_angle_error_error = 0.0f;
    s_target_omega = 0.0f;
    s_angle_outer_count = 0U;
}
float f413_ctrl_get_real_velocity(void) { return s_real_velocity; }
float f413_ctrl_get_real_omega(void)    { return s_real_omega; }
float f413_ctrl_get_target_velocity(void) { return s_target_velocity; }
float f413_ctrl_get_target_omega(void)    { return s_target_omega; }
float f413_ctrl_get_target_angle(void)    { return s_target_angle; }
float f413_ctrl_get_accel_velocity(void)  { return s_accel_velocity; }
float f413_ctrl_get_accel_forward(void)   { return s_accel_forward_filtered; }
int16_t f413_ctrl_get_motor_out_l(void)   { return s_motor_out_l; }
int16_t f413_ctrl_get_motor_out_r(void)   { return s_motor_out_r; }
bool f413_ctrl_angle_target_enabled(void) { return s_angle_target_enabled; }
bool  f413_ctrl_is_running(void)        { return s_running; }
bool  f413_ctrl_spi2_busy(void)         { return s_spi2_busy; }

/* ========================================================== */
/* 1kHz 割り込みハンドラ                                       */
/* ========================================================== */

void f413_ctrl_tick(void)
{
    float real_velocity_raw;
    float velocity_encoder;
    float omega_raw = s_real_omega;
    float out_l;
    float out_r;
    uint32_t duty_l;
    uint32_t duty_r;
    uint32_t compare_l = 0U;
    uint32_t compare_r = 0U;
    GPIO_PinState in2_l = GPIO_PIN_RESET;
    GPIO_PinState in2_r = GPIO_PIN_RESET;
    const bool use_fan_on_gains = f413_ctrl_use_fan_on_gains();
    const float kp_d = use_fan_on_gains ? KP_DISTANCE_FAN_ON : KP_DISTANCE_FAN_OFF;
    const float ki_d = use_fan_on_gains ? KI_DISTANCE_FAN_ON : KI_DISTANCE_FAN_OFF;
    const float kd_d = use_fan_on_gains ? KD_DISTANCE_FAN_ON : KD_DISTANCE_FAN_OFF;
    const float kp_v = use_fan_on_gains ? KP_VELOCITY_FAN_ON : KP_VELOCITY_FAN_OFF;
    const float ki_v = use_fan_on_gains ? KI_VELOCITY_FAN_ON : KI_VELOCITY_FAN_OFF;
    const float kd_v = use_fan_on_gains ? KD_VELOCITY_FAN_ON : KD_VELOCITY_FAN_OFF;
    const float kp_a = use_fan_on_gains ? KP_ANGLE_FAN_ON : KP_ANGLE_FAN_OFF;
    const float ki_a = use_fan_on_gains ? KI_ANGLE_FAN_ON : KI_ANGLE_FAN_OFF;
    const float kd_a = use_fan_on_gains ? KD_ANGLE_FAN_ON : KD_ANGLE_FAN_OFF;
    const float kp_o = use_fan_on_gains ? KP_OMEGA_FAN_ON : KP_OMEGA_FAN_OFF;
    const float ki_o = use_fan_on_gains ? KI_OMEGA_FAN_ON : KI_OMEGA_FAN_OFF;
    const float kd_o = use_fan_on_gains ? KD_OMEGA_FAN_ON : KD_OMEGA_FAN_OFF;

    if (!s_running)
    {
        return;
    }

    /* ---- エンコーダ読取 ---- */
    int32_t enc_l = (int32_t)__HAL_TIM_GET_COUNTER(&htim3) - (int32_t)F413_CTRL_ENCODER_CENTER;
    int32_t enc_r = (int32_t)__HAL_TIM_GET_COUNTER(&htim4) - (int32_t)F413_CTRL_ENCODER_CENTER;
    __HAL_TIM_SET_COUNTER(&htim3, F413_CTRL_ENCODER_CENTER);
    __HAL_TIM_SET_COUNTER(&htim4, F413_CTRL_ENCODER_CENTER);

    /* 方向補正: 前進でカウンタ減少 → enc_l<0 → (-1.0f)で正速度に変換
       （実機ログ 2026-05-01 で確認済み: forward→counter decrease） */
    float dist_l = (float)enc_l * s_enc_to_mm * F413_CTRL_ENCODER_SIGN_L;
    float dist_r = (float)enc_r * s_enc_to_mm * F413_CTRL_ENCODER_SIGN_R;
    s_encoder_speed_l = dist_l / F413_CTRL_DT;
    s_encoder_speed_r = dist_r / F413_CTRL_DT;
    s_encoder_distance_l += dist_l;
    s_encoder_distance_r += dist_r;
    real_velocity_raw = (s_encoder_speed_l + s_encoder_speed_r) * 0.5f;
    velocity_encoder = real_velocity_raw;

    if (!s_real_velocity_lpf_inited)
    {
        s_real_velocity_lpf = real_velocity_raw;
        s_real_velocity_lpf_inited = true;
    }
    else
    {
        float alpha_velocity = F413_CTRL_DT / (VELOCITY_LPF_TAU + F413_CTRL_DT);
        if (alpha_velocity < 0.0f)
        {
            alpha_velocity = 0.0f;
        }
        else if (alpha_velocity > 1.0f)
        {
            alpha_velocity = 1.0f;
        }
        s_real_velocity_lpf += alpha_velocity * (real_velocity_raw - s_real_velocity_lpf);
    }
    s_real_velocity = s_real_velocity_lpf;
    s_real_distance = (s_encoder_distance_l + s_encoder_distance_r) * 0.5f;

    /* 角速度 [deg/s] — IMU ジャイロから (F405 read_IMU() と同等)
       SPI2 が FRAM 操作中（HAL busy）ならこの tick の IMU 読取をスキップし前回値を維持 */
    if (s_imu_ok && hspi2.State == HAL_SPI_STATE_READY)
    {
        s_spi2_busy = true;
        /* IMU Z軸: CCW(左旋回)=正, CW(右旋回)=負 → 制御系と一致。符号反転不要。
           （2026-05-01: 旋回逆転の原因はPWMチャネル左右逆だった） */
        float omega_raw = imu_read_gyro_z_dps() - s_omega_z_offset;
        if (!s_omega_z_lpf_inited)
        {
            s_omega_z_filtered = omega_raw;
            s_omega_z_lpf_inited = true;
        }
        else
        {
            float alpha = F413_CTRL_DT / (F413_IMU_OMEGA_LPF_TAU + F413_CTRL_DT);
            s_omega_z_filtered += alpha * (omega_raw - s_omega_z_filtered);
        }
        omega_raw = s_omega_z_filtered;

        float accel_raw = imu_read_accel_forward_mm_s2() - s_accel_forward_offset;
        if (!s_accel_forward_lpf_inited)
        {
            s_accel_forward_filtered = accel_raw;
            s_accel_forward_lpf_inited = true;
        }
        else
        {
            float alpha_accel = F413_CTRL_DT / (F413_IMU_ACCEL_LPF_TAU + F413_CTRL_DT);
            s_accel_forward_filtered += alpha_accel * (accel_raw - s_accel_forward_filtered);
        }
        s_accel_velocity += s_accel_forward_filtered * F413_CTRL_DT;
        float alpha_vel = F413_CTRL_DT / (F413_CTRL_VEL_CORR_TAU + F413_CTRL_DT);
        s_accel_velocity += alpha_vel * (velocity_encoder - s_accel_velocity);
        if (s_accel_velocity > F413_CTRL_VEL_EST_MAX)
        {
            s_accel_velocity = F413_CTRL_VEL_EST_MAX;
        }
        else if (s_accel_velocity < -F413_CTRL_VEL_EST_MAX)
        {
            s_accel_velocity = -F413_CTRL_VEL_EST_MAX;
        }
        s_spi2_busy = false;
    }

    s_real_omega = omega_raw;
    s_real_angle += s_real_omega * F413_CTRL_DT;

    s_velocity_interrupt += s_acceleration_interrupt * F413_CTRL_DT;
    if (s_velocity_profile_clamp_enabled && (s_acceleration_interrupt != 0.0f))
    {
        if ((s_acceleration_interrupt > 0.0f) &&
            (s_velocity_interrupt > s_velocity_profile_target))
        {
            s_velocity_interrupt = s_velocity_profile_target;
        }
        else if ((s_acceleration_interrupt < 0.0f) &&
                 (s_velocity_interrupt < s_velocity_profile_target))
        {
            s_velocity_interrupt = s_velocity_profile_target;
        }
    }
    s_target_distance += s_velocity_interrupt * F413_CTRL_DT;

    s_distance_error = s_target_distance - s_real_distance;
    s_distance_outer_count++;
    if (s_distance_outer_count >= (uint16_t)CTRL_DISTANCE_OUTER_DIV)
    {
        s_distance_outer_count = 0U;
        s_distance_integral += s_distance_error;
        s_distance_error_error = s_distance_error - s_previous_distance_error;
        s_previous_distance_error = s_distance_error;
        s_distance_velocity_feedback = (kp_d * s_distance_error) +
                                       (ki_d * s_distance_integral) +
                                       (kd_d * s_distance_error_error);
    }
    s_target_velocity = s_velocity_interrupt + s_distance_velocity_feedback;

    s_velocity_error = s_target_velocity - s_real_velocity;
    {
        float v_i_next = s_velocity_integral + s_velocity_error;
        s_velocity_error_error = s_velocity_error - s_previous_velocity_error;
        if (CTRL_ENABLE_ANTI_WINDUP)
        {
            const float out_candidate = (kp_v * s_velocity_error) +
                                        (ki_v * v_i_next) +
                                        (kd_v * s_velocity_error_error);
            const int would_saturate = (fabsf(out_candidate) > CTRL_OUTPUT_MAX) ? 1 : 0;
            const int drives_further = (((out_candidate > 0.0f) && (s_velocity_error > 0.0f)) ||
                                        ((out_candidate < 0.0f) && (s_velocity_error < 0.0f))) ? 1 : 0;
            if (!(would_saturate && drives_further))
            {
                s_velocity_integral = v_i_next;
            }
        }
        else
        {
            s_velocity_integral = v_i_next;
        }
        s_out_translation = (kp_v * s_velocity_error) +
                            (ki_v * s_velocity_integral) +
                            (kd_v * s_velocity_error_error);
        s_previous_velocity_error = s_velocity_error;
    }

    if (!s_angle_target_enabled)
    {
        s_target_angle += s_omega_interrupt * F413_CTRL_DT;
    }
    if ((kp_a == 0.0f) && (ki_a == 0.0f) && (kd_a == 0.0f))
    {
        s_target_omega = 0.0f;
        s_angle_outer_count = 0U;
        s_angle_error = 0.0f;
        s_previous_angle_error = 0.0f;
        s_angle_error_error = 0.0f;
        s_angle_integral = 0.0f;
    }
    else
    {
        s_angle_error = s_target_angle - s_real_angle;
        s_angle_outer_count++;
        if (s_angle_outer_count >= (uint16_t)CTRL_ANGLE_OUTER_DIV)
        {
            s_angle_outer_count = 0U;
            s_angle_integral += s_angle_error;
            s_angle_error_error = s_angle_error - s_previous_angle_error;
            s_previous_angle_error = s_angle_error;
            s_target_omega = (kp_a * s_angle_error) +
                             (ki_a * s_angle_integral) +
                             (kd_a * s_angle_error_error);
        }
    }

    {
        const float omega_ref = s_omega_interrupt + s_target_omega + s_heading_omega_correction;
        float o_i_next;
        s_omega_error = s_real_omega - omega_ref;
        o_i_next = s_omega_integral + s_omega_error;
        s_omega_error_error = s_omega_error - s_previous_omega_error;
        if (CTRL_ENABLE_ANTI_WINDUP)
        {
            const float out_candidate = (kp_o * s_omega_error) +
                                        (ki_o * o_i_next) +
                                        (kd_o * s_omega_error_error);
            const int would_saturate = (fabsf(out_candidate) > CTRL_OUTPUT_MAX) ? 1 : 0;
            const int drives_further = (((out_candidate > 0.0f) && (s_omega_error > 0.0f)) ||
                                        ((out_candidate < 0.0f) && (s_omega_error < 0.0f))) ? 1 : 0;
            if (!(would_saturate && drives_further))
            {
                s_omega_integral = o_i_next;
            }
        }
        else
        {
            s_omega_integral = o_i_next;
        }
        s_out_rotate = (kp_o * s_omega_error) +
                       (ki_o * s_omega_integral) +
                       (kd_o * s_omega_error_error);
        s_previous_omega_error = s_omega_error;
    }

    /* 左右出力 */
    out_l = s_out_translation + s_out_rotate;
    out_r = s_out_translation - s_out_rotate;
    s_motor_out_l = (int16_t)lrintf(fmaxf(fminf(out_l, (float)F413_CTRL_PWM_MAX), -(float)F413_CTRL_PWM_MAX));
    s_motor_out_r = (int16_t)lrintf(fmaxf(fminf(out_r, (float)F413_CTRL_PWM_MAX), -(float)F413_CTRL_PWM_MAX));

    /* ---- モータ出力 ---- */
    duty_l = (uint32_t)fminf(fabsf(out_l), (float)F413_CTRL_PWM_MAX);
    duty_r = (uint32_t)fminf(fabsf(out_r), (float)F413_CTRL_PWM_MAX);

    if (duty_l != 0U)
    {
        if (out_l >= 0.0f)
        {
            compare_l = duty_l;
            in2_l = GPIO_PIN_RESET;
        }
        else
        {
            compare_l = F413_CTRL_PWM_MAX - duty_l;
            in2_l = GPIO_PIN_SET;
        }
    }

    if (duty_r != 0U)
    {
        if (out_r >= 0.0f)
        {
            compare_r = F413_CTRL_PWM_MAX - duty_r;
            in2_r = GPIO_PIN_SET;
        }
        else
        {
            compare_r = duty_r;
            in2_r = GPIO_PIN_RESET;
        }
    }

    /* ハードウェア配線: TIM2_CH1=左モータ, TIM2_CH3=右モータ
       MP6551 入力: PWM pin=IN1, DIR pin=IN2, STBY pin=EN1/EN2 */
    if (in2_l == GPIO_PIN_SET)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare_l);
        HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, compare_l);
    }

    if (in2_r == GPIO_PIN_SET)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, compare_r);
        HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, compare_r);
    }
}
