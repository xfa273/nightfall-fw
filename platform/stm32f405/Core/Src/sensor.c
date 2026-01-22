/*
 * sensor.c
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#include "global.h"
#include "flash_params.h"
#include "build_info.h"

// デバッグ出力ヘルパ：壁センサオフセットを表示
static void print_wall_offsets(const char* label)
{
    printf("%s: L=%u, R=%u, FR=%u, FL=%u\r\n",
           label,
           (unsigned)wall_offset_l,
           (unsigned)wall_offset_r,
           (unsigned)wall_offset_fr,
           (unsigned)wall_offset_fl);
}

void wall_end_set_detect_mode(uint8_t mode) {
    if (mode == WALL_END_DETECT_MODE_DERIV) {
        wall_end_detect_mode = WALL_END_DETECT_MODE_DERIV;
    } else {
        wall_end_detect_mode = WALL_END_DETECT_MODE_RAW;
    }
}

uint8_t wall_end_get_detect_mode(void) {
    return wall_end_detect_mode;
}

void wall_end_update_deriv(void) {
    enum { W = 18, N = 36 };
    static bool s_init = false;
    static uint16_t s_buf_r[N] = {0};
    static uint16_t s_buf_l[N] = {0};
    static uint32_t s_sum_old_r = 0;
    static uint32_t s_sum_new_r = 0;
    static uint32_t s_sum_old_l = 0;
    static uint32_t s_sum_new_l = 0;
    static uint8_t s_idx = 0;

    const uint16_t x_r = ad_r;
    const uint16_t x_l = ad_l;

    if (wall_end_deriv_reset_request || !s_init) {
        s_init = true;
        s_idx = 0;
        for (int i = 0; i < N; i++) {
            s_buf_r[i] = x_r;
            s_buf_l[i] = x_l;
        }
        s_sum_old_r = (uint32_t)W * (uint32_t)x_r;
        s_sum_new_r = (uint32_t)W * (uint32_t)x_r;
        s_sum_old_l = (uint32_t)W * (uint32_t)x_l;
        s_sum_new_l = (uint32_t)W * (uint32_t)x_l;
        wall_end_deriv_reset_request = false;
    } else {
        const uint8_t idx = s_idx;
        const uint8_t idx_move = (uint8_t)((uint16_t)idx + (uint16_t)W);
        const uint8_t pos_move = (uint8_t)(idx_move % (uint8_t)N);

        const uint16_t x_drop_r = s_buf_r[idx];
        const uint16_t x_move_r = s_buf_r[pos_move];
        s_sum_old_r = s_sum_old_r - (uint32_t)x_drop_r + (uint32_t)x_move_r;
        s_sum_new_r = s_sum_new_r - (uint32_t)x_move_r + (uint32_t)x_r;

        const uint16_t x_drop_l = s_buf_l[idx];
        const uint16_t x_move_l = s_buf_l[pos_move];
        s_sum_old_l = s_sum_old_l - (uint32_t)x_drop_l + (uint32_t)x_move_l;
        s_sum_new_l = s_sum_new_l - (uint32_t)x_move_l + (uint32_t)x_l;

        s_buf_r[idx] = x_r;
        s_buf_l[idx] = x_l;
        s_idx = (uint8_t)((uint8_t)(idx + 1u) % (uint8_t)N);
    }

    if (wall_end_detect_mode == WALL_END_DETECT_MODE_DERIV) {
        const int32_t diff_r = (int32_t)s_sum_new_r - (int32_t)s_sum_old_r;
        const int32_t diff_l = (int32_t)s_sum_new_l - (int32_t)s_sum_old_l;
        wall_end_deriv_r = diff_r / (int32_t)(W / 2);
        wall_end_deriv_l = diff_l / (int32_t)(W / 2);
    } else {
        wall_end_deriv_r = 0;
        wall_end_deriv_l = 0;
    }
}
//+++++++++++++++++++++++++++++++++++++++++++++++
// ADC DMA helpers (commit 89a941a based)
//+++++++++++++++++++++++++++++++++++++++++++++++
HAL_StatusTypeDef sensor_adc_dma_start(volatile uint16_t *dst)
{
    // Ensure previous DMA is stopped
    (void)HAL_ADC_Stop_DMA(&hadc1);

    // Start regular group conversion with DMA into provided buffer (9 ranks)
    return HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dst, 9);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// sensor_init
// センサ系の変数の初期化，ADコンバータの設定とセンサ値取得に使用するタイマの設定をする
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void sensor_init(void) {
    //====センサ系の変数の初期化====
    tp = 0;
    ad_l = ad_r = ad_fr = ad_fl = 0;
    base_l = base_r = base_f = 0;
    ADC_task_counter = 0;

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim5);
    // センサスケジューラ（TIM6）も開始（非同期DMA駆動）
    HAL_TIM_Base_Start_IT(&htim6);

    // センサのオフセット値をフラッシュから読み込み（有効なら使用）
    // 失敗した場合のみ測定を実施
    bool loaded = sensor_params_load_from_flash();
    if (!loaded) {
        get_sensor_offsets();
        print_wall_offsets("Wall offsets (measured at init)");
    } else {
        print_wall_offsets("Wall offsets (loaded at init)");
    }

    IMU_Init_Auto();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// Flash parameter I/O (separate from maze EEPROM emulation)
// Save/Load sensor baselines and offsets to FLASH_SECTOR_10
//+++++++++++++++++++++++++++++++++++++++++++++++

bool sensor_params_load_from_flash(void)
{
    flash_params_t p;
    if (!flash_params_load(&p)) {
        return false;
    }

    // Apply loaded values to globals
    base_l = (uint16_t)p.base_l;
    base_r = (uint16_t)p.base_r;
    base_f = (uint16_t)p.base_f;

    wall_offset_r  = (uint16_t)p.wall_offset_r;
    wall_offset_l  = (uint16_t)p.wall_offset_l;
    wall_offset_fr = (uint16_t)p.wall_offset_fr;
    wall_offset_fl = (uint16_t)p.wall_offset_fl;

    imu_offset_z = p.imu_offset_z;
    return true;
}

HAL_StatusTypeDef sensor_params_save_to_flash(void)
{
    flash_params_t p;
    flash_params_defaults(&p);

    // Capture current values
    p.base_l = base_l;
    p.base_r = base_r;
    p.base_f = base_f;

    p.wall_offset_r  = wall_offset_r;
    p.wall_offset_l  = wall_offset_l;
    p.wall_offset_fr = wall_offset_fr;
    p.wall_offset_fl = wall_offset_fl;

    p.imu_offset_z = imu_offset_z;

    return flash_params_save(&p);
}

// 任意タイミングで再測定してフラッシュへ保存するヘルパ（壁センサのみ）
HAL_StatusTypeDef sensor_recalibrate_and_save(void)
{
    // 壁センサオフセットの再測定（壁無し・静止条件で実施すること）
    get_sensor_offsets();
    // 制御基準値の更新（機体姿勢が適正な状態で）
    (void)get_base();
    // 測定したオフセットを表示
    print_wall_offsets("Wall offsets (measured)");

    return sensor_params_save_to_flash();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_adc_value
// 指定されたチャンネルのアナログ電圧値を取り出す
// 引数1：hadc …… 電圧値を取り出すチャンネルが属すADCのHandler
// 引数2：channel …… 電圧値を取り出すチャンネル
// 戻り値：電圧値（12bit分解能）
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_adc_value(ADC_HandleTypeDef *hadc, uint32_t channel) {
    (void)channel;
    HAL_ADC_Start(hadc);                  // AD変換を開始する
    HAL_ADC_PollForConversion(hadc, 150); // AD変換終了まで待機する
    return (HAL_ADC_GetValue(hadc) * K_SENSOR); // AD変換結果を取得する
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_sensor_value_r
// Sensor R の値を読み取る
// 引数：無し
// 戻り値：電圧値（12bit分解能）
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_sensor_value_r(void) { return get_adc_value(&hadc1, ADC_CHANNEL_1); }

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_sensor_value_fr
// Sensor FR の値を読み取る
// 引数：無し
// 戻り値：電圧値（12bit分解能）
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_sensor_value_fr(void) { return get_adc_value(&hadc1, ADC_CHANNEL_0); }

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_sensor_value_fl
// Sensor FL の値を読み取る
// 引数：無し
// 戻り値：電圧値（12bit分解能）
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_sensor_value_fl(void) { return get_adc_value(&hadc1, ADC_CHANNEL_2); }

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_sensor_value_l
// Sensor L の値を読み取る
// 引数：無し
// 戻り値：電圧値（12bit分解能）
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_sensor_value_l(void) { return get_adc_value(&hadc1, ADC_CHANNEL_3); }

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_battery_value
// VOL Check の値を読み取る
// 引数：無し
// 戻り値：電圧値（12bit分解能）
//+++++++++++++++++++++++++++++++++++++++++++++++
int get_battery_value(void) { return get_adc_value(&hadc1, ADC_CHANNEL_10); }

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_base
// 壁制御用の基準値を取得する
// 引数：なし
// 戻り値：理想的な値を取得できたか　1:できた　0:できなかった
//+++++++++++++++++++++++++++++++++++++++++++++++
uint8_t get_base() {
    uint8_t res = 1; // 理想的な値を取得できたか

    //----制御用の基準を取得----
    // フラッシュからロード済み/テストモードで計測済みの base_l/base_r が非ゼロならそれを優先
    // 未設定（0）の場合のみ、コンパイル時マクロにフォールバック
    if (base_l == 0 || base_r == 0) {
        base_l = WALL_CTRL_BASE_L;
        base_r = WALL_CTRL_BASE_R;
    }
    // 前壁合成は毎回現在値から算出
    base_f = ad_fl + ad_fr;

    // printf("base: %d,%d\n", base_l, base_r);

    return res; // 理想的な値を取得できたかを返す
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// get_wall_info
// 壁情報を取得する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void get_wall_info() {

    // センサ補正値が未定義なら1.0にする
    if (sensor_kx < 0.3 || sensor_kx > 2.0) {
        sensor_kx = 1.0;
    }
    //----壁情報の初期化----
    wall_info = 0x00; // 壁情報を初期化
    //----前壁を見る----
    if (ad_fr > WALL_BASE_FR * sensor_kx || ad_fl > WALL_BASE_FL * sensor_kx) {
        // AD値が閾値より大きい（=壁があって光が跳ね返ってきている）場合
        wall_info |= 0x88; // 壁情報を更新
    }
    //----右壁を見る----
    if (ad_r > WALL_BASE_R * sensor_kx) {
        // AD値が閾値より大きい（=壁があって光が跳ね返ってきている）場合
        wall_info |= 0x44; // 壁情報を更新
        r_wall = true;
    } else {
        r_wall = false;
    }
    //----左壁を見る----
    if (ad_l > WALL_BASE_L * sensor_kx) {
        // AD値が閾値より大きい（=壁があって光が跳ね返ってきている）場合
        wall_info |= 0x11; // 壁情報を更新
        l_wall = true;
    } else {
        l_wall = false;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// ISM330DHCX support
//+++++++++++++++++++++++++++++++++++++++++++++++

// 読み出しヘルパ（リトルエンディアン16bit）
static int16_t read16_le(uint8_t reg_l) {
    uint8_t lo = read_byte(reg_l);
    uint8_t hi = read_byte(reg_l + 1);
    return (int16_t)((int16_t)hi << 8 | lo);
}

// ISM330DHCX 初期化
void ISM330_Init(void) {
    uint8_t who = read_byte(0x0F); // WHO_AM_I
    printf("ISM330 WHO_AM_I = 0x%02x\r\n", who); // 期待値: 0x6B

    if (who != 0x6B) {
        // 必要最小限の表示のみ。失敗時はブザーのみ。
        buzzer_beep(2500);
        // 検出失敗でも継続は可能だが、以降の読み出しは無効
    }

    // CTRL3_C (0x12): BDU=1, IF_INC=1
    // BDU(6)=1, IF_INC(2)=1 -> 0b0100_0100 = 0x44
    write_byte(0x12, 0x44);
    HAL_Delay(10);

    // CTRL2_G (0x11): Gyro設定
    // ODR_G=833Hz(0x7<<4)=0x70
    // FS_4000=1で±4000 dpsに設定（FS_Gは未使用扱い）
    // 推奨: 0x71 (= 0x70 | 0x01)
    write_byte(0x11, 0x71);
    // 以前の設定（参考・ロールバック用）: ±2000 dps
    // write_byte(0x11, 0x7C); // ODR=833Hz, FS_G=2000 dps
    HAL_Delay(10);

    // CTRL1_XL (0x10): ODR_XL=833Hz, FS_XL=±16g -> 0x7C
    write_byte(0x10, 0x7C);
    HAL_Delay(10);

    set_flag = 1;
}

// 感度換算
static inline float ism330_gyro_dps(int16_t raw) {
    // FS=4000 dps => 140 mdps/LSB = 0.14 dps/LSB
    return (float)raw * 0.14f; // [deg/s]
    // 参考（以前の設定）: FS=2000 dps => 70 mdps/LSB
    // return (float)raw * 0.07f; // [deg/s]
}
static inline float ism330_accel_g(int16_t raw) { // FS=16g => 0.488 mg/LSB
    return (float)raw * 0.000488f; // [g]
}

void ISM330_DataUpdate(void) {
    if (set_flag == 1) {
        // Gyro OUT*_G: X 0x22/0x23, Y 0x24/0x25, Z 0x26/0x27
        float gx = ism330_gyro_dps(read16_le(0x22));
        float gy = ism330_gyro_dps(read16_le(0x24));
        float gz = ism330_gyro_dps(read16_le(0x26));

        // Accel OUT*_A: X 0x28/0x29, Y 0x2A/0x2B, Z 0x2C/0x2D
        float ax = ism330_accel_g(read16_le(0x28));
        float ay = ism330_accel_g(read16_le(0x2A));
        float az = ism330_accel_g(read16_le(0x2C));

        // 既存座標系との整合（ICM実装と同様の符号を暫定適用）
        omega_x_raw = -gx;
        omega_y_raw = gy;
        omega_z_raw = gz;

        accel_x_raw = -ax;
        accel_y_raw = ay;
        accel_z_raw = az;

        // オフセット補正
        omega_x_true = omega_x_raw - omega_x_offset;
        omega_y_true = omega_y_raw - omega_y_offset;
        omega_z_true = omega_z_raw - omega_z_offset;
        accel_x_true = accel_x_raw - accel_x_offset;
        accel_y_true = accel_y_raw - accel_y_offset;
        accel_z_true = accel_z_raw - accel_z_offset;
    }
}

// IMU自動検出と初期化
void IMU_Init_Auto(void) {
    // まずICM20689を試す
    uint8_t who_icm = read_byte(0x75);
    if (who_icm == 0x98) {
        imu_model = 1; // ICM20689
        ICM20689_Init();
        return;
    }

    // 次にISM330DHCXを試す
    uint8_t who_ism = read_byte(0x0F);
    if (who_ism == 0x6B) {
        imu_model = 2; // ISM330DHCX
        ISM330_Init();
        return;
    }

    // いずれも検出できない場合はエラー
    imu_model = 0;
    buzzer_beep(3000);
}

// アクティブなIMUから最新値を取得（ラッパー）
void IMU_DataUpdate(void) {
    if (imu_model == 1) {
        ICM20689_DataUpdate();
    } else if (imu_model == 2) {
        ISM330_DataUpdate();
    } else {
        // 未検出時は何もしない
    }
}

uint8_t read_byte(uint8_t reg) {
    uint8_t ret, val;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // cs = 0;
    ret = reg | 0x80;
    HAL_SPI_Transmit(&hspi3, &ret, 1, 100);
    HAL_SPI_Receive(&hspi3, &val, 1, 100);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // cs = 1;
    return val;
}

// SPI3を動的に再初期化（モード/速度切替用）
static void spi3_reinit(uint32_t pol, uint32_t phase, uint32_t presc) {
    HAL_SPI_DeInit(&hspi3);
    hspi3.Init.CLKPolarity = pol;
    hspi3.Init.CLKPhase = phase;
    hspi3.Init.BaudRatePrescaler = presc;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) {
        printf("HAL_SPI_Init failed in debug reinit.\r\n");
    }
}

// WHO_AM_IをSPIモードを切替えつつ読み取って表示（デバッグ用）
void IMU_ProbeWHOAMI_Debug(void) {
    uint8_t icm_reg = 0x75; // ICM20689 WHO_AM_I
    uint8_t ism_reg = 0x0F; // ISM330DHCX WHO_AM_I

    printf("=== IMU WHO_AM_I probe start ===\r\n");

    // 電源投入直後の安定待ち
    HAL_Delay(20);

    // 低速で検証（ライン品質の影響を減らす）
    // Mode 3 (CPOL=1, CPHA=2)
    spi3_reinit(SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_128);
    uint8_t icm_m3 = read_byte(icm_reg);
    uint8_t ism_m3 = read_byte(ism_reg);
    printf("Mode3/Presc128 -> ICM:0x%02X, ISM:0x%02X\r\n", icm_m3, ism_m3);

    // Mode 0 (CPOL=0, CPHA=1)
    spi3_reinit(SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_128);
    uint8_t icm_m0 = read_byte(icm_reg);
    uint8_t ism_m0 = read_byte(ism_reg);
    printf("Mode0/Presc128 -> ICM:0x%02X, ISM:0x%02X\r\n", icm_m0, ism_m0);

    // Mode 1 (CPOL=0, CPHA=2)
    spi3_reinit(SPI_POLARITY_LOW, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_128);
    uint8_t icm_m1 = read_byte(icm_reg);
    uint8_t ism_m1 = read_byte(ism_reg);
    printf("Mode1/Presc128 -> ICM:0x%02X, ISM:0x%02X\r\n", icm_m1, ism_m1);

    // Mode 2 (CPOL=1, CPHA=1)
    spi3_reinit(SPI_POLARITY_HIGH, SPI_PHASE_1EDGE, SPI_BAUDRATEPRESCALER_128);
    uint8_t icm_m2 = read_byte(icm_reg);
    uint8_t ism_m2 = read_byte(ism_reg);
    printf("Mode2/Presc128 -> ICM:0x%02X, ISM:0x%02X\r\n", icm_m2, ism_m2);

    // 既定へ復帰（Mode3/Presc32）
    spi3_reinit(SPI_POLARITY_HIGH, SPI_PHASE_2EDGE, SPI_BAUDRATEPRESCALER_32);
    printf("=== IMU WHO_AM_I probe end ===\r\n");
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// write_byte関数
/*SPI Operational Features
1.Data is delivered MSB first and LSB last
2.Data is latched on the rising edge of SCLK
3.Data should be transitioned on the falling edge of SPC
4.The maximum frequency of SPC is 10MHz
5.SPI read and write operations are completed in 16 or more clock cycles(two or
more bytes.) The first byte conains the SPI Adress The following bytes contain
the SPI data The first bit of the first byte contains the Read/Write bit and
indicates the Read(1) or Write(0) operation. The following 7 bits is the
Resister Address.
*/
//+++++++++++++++++++++++++++++++++++++++++++++++

void write_byte(uint8_t reg, uint8_t val) {
    uint8_t ret;
    ret = reg & 0x7F;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // cs = 0;
    HAL_SPI_Transmit(&hspi3, &ret, 1, 100);
    HAL_SPI_Transmit(&hspi3, &val, 1, 100);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // cs = 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// ICM20602_Init
//+++++++++++++++++++++++++++++++++++++++++++++++

void ICM20689_Init(void) {
    uint8_t who_am_i = 0;
    who_am_i = read_byte(0x75);              // check WHO_AM_I (0x75)
    printf("ICM20689 WHO_AM_I = 0x%02x\r\n", who_am_i); // 期待値: 0x98

    if (who_am_i != 0x98) { // recheck 0x98
        HAL_Delay(100);
        who_am_i = read_byte(0x75);
        if (who_am_i != 0x98) {
            // 必要最小限の表示のみ。失敗時はブザーのみ。
            buzzer_beep(3000);
            buzzer_beep(3000);
        }
    }

    // PWR_MIGHT_1 0x6B
    write_byte(0x6B, 0x00); // Set pwr might
    HAL_Delay(50);
    // PWR_MIGHT_2 0x6C
    write_byte(0x6C, 0x00);
    HAL_Delay(50);
    // set gyro config
    // GYRO_CONFIG 0x1B
    write_byte(0x1B, 0x18); // use 2000 dps
    HAL_Delay(50);
    // ACCEL_CONFIG 0x1C
    write_byte(0x1B, 0x18); // use pm 16g
    HAL_Delay(50);

    set_flag = 1;
}

float ICM20689_GYRO_READ(uint8_t H_reg) {
    int16_t data = (int16_t)(((uint8_t)read_byte(H_reg) << 8) |
                             (uint8_t)read_byte(H_reg + 1));
    float omega =
        (float)(data / 16.4f); //[deg/s] FS_SEL=3-> Scale Factor=16.4[LSB/(dps)]
    return omega;
}

float ICM20689_ACCEL_READ(uint8_t H_reg) {
    int16_t data = (int16_t)(((uint8_t)read_byte(H_reg) << 8) |
                             (uint8_t)read_byte(H_reg + 1));
    float accel = (float)(data / 2048.0f);
    return accel;
}

void ICM20689_DataUpdate(void) {
    if (set_flag == 1) {
        // get yawrate
        omega_x_raw = -1 * ICM20689_GYRO_READ(0x43);
        omega_y_raw = ICM20689_GYRO_READ(0x45);
        omega_z_raw = ICM20689_GYRO_READ(0x47);

        // get accel

        accel_x_raw = -1 * ICM20689_ACCEL_READ(0x3B);
        accel_y_raw = ICM20689_ACCEL_READ(0x3D);
        accel_z_raw = ICM20689_ACCEL_READ(0x3F);

        // True Value(Consider Offset)
        omega_x_true = omega_x_raw - omega_x_offset;
        omega_y_true = omega_y_raw - omega_y_offset;
        omega_z_true = omega_z_raw - omega_z_offset;
        accel_x_true = accel_x_raw - accel_x_offset;
        accel_y_true = accel_y_raw - accel_y_offset;
        accel_z_true = accel_z_raw - accel_z_offset;
    }
}

// センサオフセット値を起動時に複数回測定して平均化する
void get_sensor_offsets(void) {
    const int NUM_SAMPLES = 10;  // 測定回数
    int i;
    uint32_t sum_r = 0, sum_l = 0, sum_fr = 0, sum_fl = 0;

    wall_offset_l = 0;
    wall_offset_r = 0;
    wall_offset_fr = 0;
    wall_offset_fl = 0;
    
    // interrupt.c のLED ON/OFF差分更新が進むのを待ってサンプルを収集
    for (i = 0; i < NUM_SAMPLES; i++) {
        // ADCタスクカウンタが一周するのを待つ（センサ値が更新されるのを待つ）
        uint8_t current_counter = ADC_task_counter;
        uint32_t t0 = HAL_GetTick();
        while (current_counter == ADC_task_counter) {
            HAL_Delay(1);
            if ((HAL_GetTick() - t0) > 200) {
                // タイムアウト: 割り込みがまだ動いていない/停止している可能性
                break;
            }
        }

        // LED ON/OFF 差分（ad_on - ad_off）のみを加算（負値は0に丸め）
        int32_t dr  = (int32_t)ad_r_raw  - (int32_t)ad_r_off;  if (dr  < 0) dr  = 0;
        int32_t dl  = (int32_t)ad_l_raw  - (int32_t)ad_l_off;  if (dl  < 0) dl  = 0;
        int32_t dfr = (int32_t)ad_fr_raw - (int32_t)ad_fr_off; if (dfr < 0) dfr = 0;
        int32_t dfl = (int32_t)ad_fl_raw - (int32_t)ad_fl_off; if (dfl < 0) dfl = 0;

        sum_r  += (uint32_t)dr;
        sum_l  += (uint32_t)dl;
        sum_fr += (uint32_t)dfr;
        sum_fl += (uint32_t)dfl;

        HAL_Delay(10); // 少し待機して次のサンプルを取得
    }
    
    // 平均値を計算して設定（LED同期差分の基本成分をオフセット値として設定）
    wall_offset_r  = (uint16_t)(sum_r  / (uint32_t)NUM_SAMPLES);
    wall_offset_l  = (uint16_t)(sum_l  / (uint32_t)NUM_SAMPLES);
    wall_offset_fr = (uint16_t)(sum_fr / (uint32_t)NUM_SAMPLES);
    wall_offset_fl = (uint16_t)(sum_fl / (uint32_t)NUM_SAMPLES);
}

// 静止状態でIMUのオフセット値を取得
void IMU_GetOffset(void) {
    HAL_Delay(400);
    omega_x_offset = omega_x_raw;
    omega_y_offset = omega_y_raw;
    omega_z_offset = omega_z_raw;
    accel_x_offset = accel_x_raw;
    accel_y_offset = accel_y_raw;
    accel_z_offset = accel_z_raw;

    printf("offset: %f, %f, %f\n", omega_x_offset, omega_y_offset,
           omega_z_offset);
}

// センサからの壁判断をLEDで表示
void indicate_sensor(void) {

    // 左センサ
    if (ad_l > WALL_BASE_L) {
        //HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    } else {
        // HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    }

    // 右センサ
    if (ad_r > WALL_BASE_R) {
        //HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    } else {
        // HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    }

    // 前壁センサ
    if (ad_fr > WALL_BASE_FR && ad_fl > WALL_BASE_FL) {
        //HAL_GPIO_WritePin(LED_7_GPIO_Port, LED_7_Pin, GPIO_PIN_SET);
    } else {
        // HAL_GPIO_WritePin(LED_7_GPIO_Port, LED_7_Pin, GPIO_PIN_RESET);
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// detect_wall_end
// 横壁センサの「有り → 無し」の立ち下がりで壁切れを検出
// ヒステリシス付き: Highしきい値を超えたら「壁あり」、Lowしきい値を下回ったら「壁なし」
// KERISE v4/Astraea参考: 直進中のみ検出、検出時の走行距離を記録
//+++++++++++++++++++++++++++++++++++++++++++++++
void detect_wall_end(void) {
    // 有効なしきい値係数を決定
    float kx = sensor_kx;
    if (kx < 0.3f || kx > 2.0f) {
        kx = 1.0f;
    }

    // しきい値（パラメータ設定の値を使用、未設定時はデフォルト値）
    uint16_t thr_r_high = (wall_end_thr_r_high > 0) ? wall_end_thr_r_high : WALL_END_THR_R_HIGH;
    uint16_t thr_r_low  = (wall_end_thr_r_low > 0)  ? wall_end_thr_r_low  : WALL_END_THR_R_LOW;
    uint16_t thr_l_high = (wall_end_thr_l_high > 0) ? wall_end_thr_l_high : WALL_END_THR_L_HIGH;
    uint16_t thr_l_low  = (wall_end_thr_l_low > 0)  ? wall_end_thr_l_low  : WALL_END_THR_L_LOW;

    // ヒステリシス付き壁状態（staticで状態を保持）
    static bool s_wall_r = false;  // 右壁の状態（true=壁あり）
    static bool s_wall_l = false;  // 左壁の状態（true=壁あり）
    static bool s_prev_r = false;  // 前回の壁状態
    static bool s_prev_l = false;  // 前回の壁状態

    static uint8_t s_deriv_fall_cnt_r = 0;
    static uint8_t s_deriv_fall_cnt_l = 0;
    // リセット要求があれば、現在の壁状態で初期化
    if (wall_end_reset_request) {
        // 初期化時はHighしきい値で判定
        s_wall_r = (ad_r > (uint16_t)(thr_r_high * kx));
        s_wall_l = (ad_l > (uint16_t)(thr_l_high * kx));
        s_prev_r = s_wall_r;
        s_prev_l = s_wall_l;

        s_deriv_fall_cnt_r = 0;
        s_deriv_fall_cnt_l = 0;
        wall_end_reset_request = false;
    }

    // ヒステリシス付き壁判定
    // 右センサ
    if (s_wall_r) {
        // 現在「壁あり」→ Lowしきい値を下回ったら「壁なし」
        if (ad_r < (uint16_t)(thr_r_low * kx)) {
            s_wall_r = false;
        }
    } else {
        // 現在「壁なし」→ Highしきい値を超えたら「壁あり」
        if (ad_r > (uint16_t)(thr_r_high * kx)) {
            s_wall_r = true;
        }
    }
    // 左センサ
    if (s_wall_l) {
        // 現在「壁あり」→ Lowしきい値を下回ったら「壁なし」
        if (ad_l < (uint16_t)(thr_l_low * kx)) {
            s_wall_l = false;
        }
    } else {
        // 現在「壁なし」→ Highしきい値を超えたら「壁あり」
        if (ad_l > (uint16_t)(thr_l_high * kx)) {
            s_wall_l = true;
        }
    }

    if (wall_end_detect_mode == WALL_END_DETECT_MODE_DERIV) {
        const uint16_t thr_d_fall = (uint16_t)(WALL_END_DERIV_FALL_THR * kx);

        if (s_wall_r && (wall_end_deriv_r < -(int32_t)thr_d_fall)) {
            if (s_deriv_fall_cnt_r < 255u) {
                s_deriv_fall_cnt_r++;
            }
        } else {
            s_deriv_fall_cnt_r = 0;
        }
        if (s_wall_l && (wall_end_deriv_l < -(int32_t)thr_d_fall)) {
            if (s_deriv_fall_cnt_l < 255u) {
                s_deriv_fall_cnt_l++;
            }
        } else {
            s_deriv_fall_cnt_l = 0;
        }

        if (s_deriv_fall_cnt_r >= 2u) {
            s_wall_r = false;
            s_deriv_fall_cnt_r = 0;
        }
        if (s_deriv_fall_cnt_l >= 2u) {
            s_wall_l = false;
            s_deriv_fall_cnt_l = 0;
        }
    } else {
        s_deriv_fall_cnt_r = 0;
        s_deriv_fall_cnt_l = 0;
    }

    // ゲート条件:
    // - 壁切れアーム中（WALL_END）
    // - スラローム中は無効（直進中のみ検出）
    const bool is_straight = (!MF.FLAG.SLALOM_R && !MF.FLAG.SLALOM_L);
    const bool gate_on = (MF.FLAG.WALL_END && is_straight);

    // 右側の立ち下がり（壁あり→壁なし）
    if (s_prev_r && !s_wall_r) {
        if (gate_on && !wall_end_detected_r) {
            wall_end_detected_r = true;
            wall_end_dist_r = real_distance;
            MF.FLAG.R_WALL_END = 1;
            HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
            wall_end_count = 200;
        }
    }

    // 左側の立ち下がり（壁あり→壁なし）
    if (s_prev_l && !s_wall_l) {
        if (gate_on && !wall_end_detected_l) {
            wall_end_detected_l = true;
            wall_end_dist_l = real_distance;
            MF.FLAG.L_WALL_END = 1;
            HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
            wall_end_count = 200;
        }
    }

    // 状態を更新
    s_prev_r = s_wall_r;
    s_prev_l = s_wall_l;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// wall_end_reset
// 壁切れ検出フラグをリセット（直進開始時に呼び出す）
// s_prev_r/lも現在の壁状態で再初期化するため、リセット要求フラグを設定
//+++++++++++++++++++++++++++++++++++++++++++++++
void wall_end_reset(void) {
    wall_end_detected_r = false;
    wall_end_detected_l = false;
    wall_end_dist_r = 0.0f;
    wall_end_dist_l = 0.0f;
    MF.FLAG.R_WALL_END = 0;
    MF.FLAG.L_WALL_END = 0;
    // detect_wall_end()内のs_prev_r/lを現在の壁状態で再初期化する要求
    wall_end_reset_request = true;
    wall_end_deriv_reset_request = true;
}

//============================================================
// センサログ機能（壁切れデバッグ用）
//============================================================

static volatile uint32_t s_sensor_log_tick = 0;  // ログ用カウンタ（6kHz相当）

//+++++++++++++++++++++++++++++++++++++++++++++++
// sensor_log_init
// センサログバッファを初期化
//+++++++++++++++++++++++++++++++++++++++++++++++
void sensor_log_init(void) {
    sensor_log_buffer.head = 0;
    sensor_log_buffer.count = 0;
    sensor_log_buffer.logging_active = 0;
    sensor_log_buffer.start_tick = 0;
    s_sensor_log_tick = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// sensor_log_start
// センサログ取得を開始
//+++++++++++++++++++++++++++++++++++++++++++++++
void sensor_log_start(void) {
    sensor_log_buffer.head = 0;
    sensor_log_buffer.count = 0;
    sensor_log_buffer.start_tick = HAL_GetTick();
    s_sensor_log_tick = 0;
    sensor_log_buffer.logging_active = 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// sensor_log_stop
// センサログ取得を停止
//+++++++++++++++++++++++++++++++++++++++++++++++
void sensor_log_stop(void) {
    sensor_log_buffer.logging_active = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// sensor_log_capture
// センサ値を記録（ADCコールバックから呼ばれる、約6kHz）
//+++++++++++++++++++++++++++++++++++++++++++++++
void sensor_log_capture(void) {
    if (!sensor_log_buffer.logging_active) {
        return;
    }
    if (sensor_log_buffer.count >= SENSOR_LOG_MAX_ENTRIES) {
        return;  // バッファフル
    }
    
    uint16_t pos = sensor_log_buffer.head;
    sensor_log_buffer.entries[pos].timestamp = s_sensor_log_tick;
    sensor_log_buffer.entries[pos].ad_r = ad_r;
    sensor_log_buffer.entries[pos].ad_l = ad_l;
    sensor_log_buffer.entries[pos].ad_fr = ad_fr;
    sensor_log_buffer.entries[pos].ad_fl = ad_fl;
    sensor_log_buffer.entries[pos].distance = real_distance;
    sensor_log_buffer.entries[pos].wall_end_deriv_r = (int32_t)wall_end_deriv_r;
    sensor_log_buffer.entries[pos].wall_end_deriv_l = (int32_t)wall_end_deriv_l;
    
    sensor_log_buffer.head = (pos + 1) % SENSOR_LOG_MAX_ENTRIES;
    sensor_log_buffer.count++;
    s_sensor_log_tick++;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// sensor_log_print
// センサログをCSV形式で出力
//+++++++++++++++++++++++++++++++++++++++++++++++
void sensor_log_print(void) {
    printf("=== Sensor Log Data (CSV Format) ===\n");
    printf("Total entries: %d\n", sensor_log_buffer.count);
    printf("CSV Format: timestamp,ad_r,ad_l,ad_fr,ad_fl,distance,wall_end_deriv_r,wall_end_deriv_l\n");
    printf("--- CSV Data Start ---\n");
    printf("#fw_target=%s\n", FW_TARGET);
    printf("#fw_build_type=%s\n", FW_BUILD_TYPE);
    printf("#fw_build_time_utc=%s\n", FW_BUILD_TIME_UTC);
    printf("#fw_git_describe=%s\n", FW_GIT_DESCRIBE);
    printf("#fw_git_sha=%s\n", FW_GIT_SHA);
    printf("#fw_git_branch=%s\n", FW_GIT_BRANCH);
    printf("#fw_git_dirty=%d\n", (int)FW_GIT_DIRTY);
    printf("#mm_columns=timestamp,ad_r,ad_l,ad_fr,ad_fl,distance,wall_end_deriv_r,wall_end_deriv_l\n");
    
    uint16_t count = sensor_log_buffer.count > SENSOR_LOG_MAX_ENTRIES 
                     ? SENSOR_LOG_MAX_ENTRIES : sensor_log_buffer.count;
    
    for (uint16_t i = 0; i < count; i++) {
        SensorLogEntry *entry = (SensorLogEntry *)&sensor_log_buffer.entries[i];
        printf("%lu,%d,%d,%d,%d,%.3f,%ld,%ld\n",
               entry->timestamp,
               entry->ad_r,
               entry->ad_l,
               entry->ad_fr,
               entry->ad_fl,
               entry->distance,
               (long)entry->wall_end_deriv_r,
               (long)entry->wall_end_deriv_l);
    }
    
    printf("--- CSV Data End ---\n");
    printf("=== End of Sensor Log ===\n");
}
