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
#include <math.h>
#include <string.h>

/* ---------- ハードウェア定数 ---------- */
#define F413_CTRL_DT              (0.001f)   /* 制御周期 [s] (1kHz) */
#define F413_CTRL_ENCODER_CENTER  (30000U)   /* エンコーダカウンタ中央値 */
#define F413_CTRL_CPR_WHEEL       (400.0f)   /* NJL5820 100PPR × 4逓倍 */
#define F413_CTRL_D_TIRE          (13.35f)   /* タイヤ直径 [mm] */
#define F413_CTRL_TREAD           (33.5f)    /* 左右タイヤ中心間距離 [mm] */
#define F413_CTRL_PWM_MAX         (1000U)    /* TIM2 ARR */

/* ---------- 制御ゲイン（保守的な P のみ） ---------- */
#define F413_CTRL_KP_VEL          (0.5f)     /* [PWM counts / (mm/s)] */
#define F413_CTRL_KP_OMEGA        (1.0f)     /* [PWM counts / (deg/s)] */
#define F413_CTRL_FF_VEL          (0.8f)     /* フィードフォワード速度→PWM (CPR=400用) */
#define F413_CTRL_FF_OMEGA        (0.3f)     /* フィードフォワード角速度→PWM */

/* ---------- IMU (ISM330DHCX) 定数 ---------- */
#define F413_IMU_WHO_AM_I_REG     (0x0FU)
#define F413_IMU_WHO_AM_I_VAL     (0x6BU)
#define F413_IMU_CTRL1_XL         (0x10U)    /* 加速度設定 */
#define F413_IMU_CTRL2_G          (0x11U)    /* ジャイロ設定 */
#define F413_IMU_CTRL3_C          (0x12U)    /* 制御レジスタ3 */
#define F413_IMU_OUTZ_G_L         (0x26U)    /* ジャイロ Z軸 LOW byte */
#define F413_IMU_OUTZ_G_H         (0x27U)    /* ジャイロ Z軸 HIGH byte */
#define F413_IMU_GYRO_SENSITIVITY (0.14f)    /* FS=4000dps → 140mdps/LSB [deg/s/LSB] */
#define F413_IMU_OMEGA_LPF_TAU    (0.020f)   /* 角速度 LPF 時定数 [s] (振動ノイズ除去) */
#define F413_IMU_OFFSET_SAMPLES   (500U)     /* オフセット測定回数 */
#define F413_IMU_OFFSET_SETTLE_MS (200U)     /* 静定待ち [ms] */

/* ---------- エンコーダ変換定数 ---------- */
static const float s_enc_to_mms =
    (F413_CTRL_D_TIRE * 3.14159265f) / (F413_CTRL_CPR_WHEEL * F413_CTRL_DT);

/* ---------- HAL ハンドル (main.c で定義) ---------- */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern SPI_HandleTypeDef hspi2;

/* ---------- 状態変数 ---------- */
static volatile bool s_running = false;
static bool s_imu_ok = false;

static volatile float s_target_velocity = 0.0f;
static volatile float s_target_omega    = 0.0f;

static volatile float s_real_velocity   = 0.0f;
static volatile float s_real_omega      = 0.0f;
static volatile float s_distance        = 0.0f;
static volatile float s_angle           = 0.0f;

static float s_omega_z_offset  = 0.0f;
static float s_omega_z_filtered = 0.0f;
static bool  s_omega_z_lpf_inited = false;

static volatile bool s_spi2_busy = false;

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

static void imu_get_gyro_z_offset(void)
{
    float sum = 0.0f;
    uint32_t i;

    HAL_Delay(F413_IMU_OFFSET_SETTLE_MS);

    for (i = 0U; i < F413_IMU_OFFSET_SAMPLES; i++)
    {
        sum += imu_read_gyro_z_dps();
        HAL_Delay(1U);
    }

    s_omega_z_offset = sum / (float)F413_IMU_OFFSET_SAMPLES;
    s_omega_z_filtered = 0.0f;
    s_omega_z_lpf_inited = false;
}

/* ========================================================== */
/* Public API                                                  */
/* ========================================================== */

void f413_ctrl_init(void)
{
    s_running = false;
    s_target_velocity = 0.0f;
    s_target_omega    = 0.0f;
    s_real_velocity   = 0.0f;
    s_real_omega      = 0.0f;
    s_distance        = 0.0f;
    s_angle           = 0.0f;

    /* IMU 初期化 + ジャイロオフセット取得 */
    s_imu_ok = imu_init_ism330();
    if (s_imu_ok)
    {
        imu_get_gyro_z_offset();
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
        imu_get_gyro_z_offset();
    }

    /* 状態をリセットしてから有効化 */
    s_target_velocity = 0.0f;
    s_target_omega    = 0.0f;
    s_real_velocity   = 0.0f;
    s_real_omega      = 0.0f;
    s_distance        = 0.0f;
    s_angle           = 0.0f;
    s_omega_z_filtered = 0.0f;
    s_omega_z_lpf_inited = false;

    __HAL_TIM_SET_COUNTER(&htim3, F413_CTRL_ENCODER_CENTER);
    __HAL_TIM_SET_COUNTER(&htim4, F413_CTRL_ENCODER_CENTER);

    /* モータ有効化 (STBY = HIGH, PWM開始) */
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);

    s_running = true;
}

void f413_ctrl_stop(void)
{
    s_running = false;
    s_target_velocity = 0.0f;
    s_target_omega    = 0.0f;

    /* モータ停止 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
    (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
}

void f413_ctrl_set_velocity(float velocity_mm_s)
{
    s_target_velocity = velocity_mm_s;
}

void f413_ctrl_set_omega(float omega_deg_s)
{
    s_target_omega = omega_deg_s;
}

float f413_ctrl_get_distance(void)      { return s_distance; }
float f413_ctrl_get_angle(void)         { return s_angle; }
void  f413_ctrl_reset_distance(void)    { s_distance = 0.0f; }
void  f413_ctrl_reset_angle(void)       { s_angle = 0.0f; }
float f413_ctrl_get_real_velocity(void) { return s_real_velocity; }
float f413_ctrl_get_real_omega(void)    { return s_real_omega; }
bool  f413_ctrl_is_running(void)        { return s_running; }
bool  f413_ctrl_spi2_busy(void)         { return s_spi2_busy; }

/* ========================================================== */
/* 1kHz 割り込みハンドラ                                       */
/* ========================================================== */

void f413_ctrl_tick(void)
{
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
    float vel_l = (float)enc_l * s_enc_to_mms * (-1.0f);
    float vel_r = (float)enc_r * s_enc_to_mms * (-1.0f);

    /* 並進速度 [mm/s] — エンコーダから */
    s_real_velocity = (vel_l + vel_r) * 0.5f;

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
        s_real_omega = s_omega_z_filtered;
        s_spi2_busy = false;
    }

    /* 累積 */
    s_distance += s_real_velocity * F413_CTRL_DT;
    s_angle    += s_real_omega    * F413_CTRL_DT;

    /* ---- P 制御 + フィードフォワード ---- */
    float vel_err   = s_target_velocity - s_real_velocity;
    float omega_err = s_target_omega    - s_real_omega;

    float out_trans = F413_CTRL_FF_VEL   * s_target_velocity + F413_CTRL_KP_VEL   * vel_err;
    float out_rot   = F413_CTRL_FF_OMEGA * s_target_omega    + F413_CTRL_KP_OMEGA * omega_err;

    /* 左右出力 */
    float out_l = out_trans - out_rot;
    float out_r = out_trans + out_rot;

    /* ---- モータ出力 ---- */
    /* 方向設定 */
    HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin,
                      (out_l >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin,
                      (out_r >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* PWMデューティ (0〜PWM_MAX) */
    uint32_t duty_l = (uint32_t)fminf(fabsf(out_l), (float)F413_CTRL_PWM_MAX);
    uint32_t duty_r = (uint32_t)fminf(fabsf(out_r), (float)F413_CTRL_PWM_MAX);

    /* ハードウェア配線: TIM2_CH1=右モータ, TIM2_CH3=左モータ
       （2026-05-01 テスト3で確認: CHを入れ替えないと旋回方向が逆転する） */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_l);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_r);
}
