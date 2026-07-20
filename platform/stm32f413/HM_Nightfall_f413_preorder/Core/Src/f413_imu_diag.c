#include "f413_imu_diag.h"

#include "f413_control.h"
#include "main.h"
#include "params.h"
#include "trace.h"

#define F413_IMU_DIAG_WHO_AM_I_REG (0x0FU)
#define F413_IMU_DIAG_WHO_AM_I_EXPECTED (0x6BU)
#define F413_IMU_DIAG_CTRL1_XL (0x10U)
#define F413_IMU_DIAG_CTRL2_G (0x11U)
#define F413_IMU_DIAG_CTRL3_C (0x12U)
#define F413_IMU_DIAG_OUTZ_G_L (0x26U)
#define F413_IMU_DIAG_OUTX_XL_L (0x28U)
#define F413_IMU_DIAG_OUTY_XL_L (0x2AU)
#define F413_IMU_DIAG_OUTZ_XL_L (0x2CU)
#define F413_IMU_DIAG_GYRO_SENSITIVITY (0.14f)
#define F413_IMU_DIAG_ACCEL_SENS_MG (0.488f)
#define F413_IMU_DIAG_GRAVITY_MM_S2 (9.80665f)
#define F413_IMU_DIAG_MANUAL_OFFSET_SAMPLES (500U)
#define F413_IMU_DIAG_MANUAL_TEST_MS (8000U)
#define F413_IMU_DIAG_MANUAL_SAMPLE_MS (10U)
#define F413_IMU_DIAG_MANUAL_PRINT_MS (200U)

extern SPI_HandleTypeDef hspi2;

static uint32_t f413_imu_diag_lock_spi2(void)
{
  const uint32_t tim5_irq_enabled = NVIC_GetEnableIRQ(TIM5_IRQn);

  /* TIM5 owns SPI2 while sampling the IMU. Keep diagnostic transfers atomic
     so the 1 kHz control interrupt cannot enter between CS and HAL SPI setup. */
  HAL_NVIC_DisableIRQ(TIM5_IRQn);
  return tim5_irq_enabled;
}

static void f413_imu_diag_unlock_spi2(uint32_t tim5_irq_enabled)
{
  if (tim5_irq_enabled != 0U)
  {
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  }
}

bool f413_imu_diag_read_reg(uint8_t reg, uint8_t* out)
{
  uint8_t tx[2];
  uint8_t rx[2] = {0U, 0U};
  uint32_t tim5_irq_enabled;
  HAL_StatusTypeDef status;

  if (out == NULL)
  {
    return false;
  }

  tx[0] = (uint8_t)(reg | 0x80U);
  tx[1] = 0x00U;

  tim5_irq_enabled = f413_imu_diag_lock_spi2();
  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, 20U);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  f413_imu_diag_unlock_spi2(tim5_irq_enabled);

  if (status != HAL_OK)
  {
    return false;
  }

  *out = rx[1];
  return true;
}

static bool f413_imu_diag_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t tx[2];
  uint32_t tim5_irq_enabled;
  HAL_StatusTypeDef status;

  tx[0] = (uint8_t)(reg & 0x7FU);
  tx[1] = val;

  tim5_irq_enabled = f413_imu_diag_lock_spi2();
  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(&hspi2, tx, 2U, 20U);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  f413_imu_diag_unlock_spi2(tim5_irq_enabled);

  return status == HAL_OK;
}

static bool f413_imu_diag_read_i16_le(uint8_t reg_l, int16_t* out)
{
  uint8_t tx[3];
  uint8_t rx[3] = {0U, 0U, 0U};
  uint32_t tim5_irq_enabled;
  HAL_StatusTypeDef status;

  if (out == NULL)
  {
    return false;
  }

  tx[0] = (uint8_t)(reg_l | 0x80U);
  tx[1] = 0x00U;
  tx[2] = 0x00U;

  tim5_irq_enabled = f413_imu_diag_lock_spi2();
  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 3U, 20U);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  f413_imu_diag_unlock_spi2(tim5_irq_enabled);

  if (status != HAL_OK)
  {
    return false;
  }

  *out = (int16_t)((uint16_t)rx[2] << 8U | (uint16_t)rx[1]);
  return true;
}

static bool f413_imu_diag_config_for_gyro(void)
{
  uint8_t who = 0U;

  if (!f413_imu_diag_read_reg(F413_IMU_DIAG_WHO_AM_I_REG, &who) ||
      (who != F413_IMU_DIAG_WHO_AM_I_EXPECTED))
  {
    trace_printf("[HW-TEST][IMU-ANGLE] FAIL(who=0x%02X expected=0x%02X)\r\n",
                 (unsigned int)who,
                 (unsigned int)F413_IMU_DIAG_WHO_AM_I_EXPECTED);
    return false;
  }

  if (!f413_imu_diag_write_reg(F413_IMU_DIAG_CTRL3_C, 0x44U))
  {
    trace_printf("[HW-TEST][IMU-ANGLE] FAIL(write CTRL3_C)\r\n");
    return false;
  }
  HAL_Delay(10U);

  if (!f413_imu_diag_write_reg(F413_IMU_DIAG_CTRL2_G, 0x71U))
  {
    trace_printf("[HW-TEST][IMU-ANGLE] FAIL(write CTRL2_G)\r\n");
    return false;
  }
  HAL_Delay(10U);

  if (!f413_imu_diag_write_reg(F413_IMU_DIAG_CTRL1_XL, 0x7CU))
  {
    trace_printf("[HW-TEST][IMU-ANGLE] FAIL(write CTRL1_XL)\r\n");
    return false;
  }
  HAL_Delay(10U);

  return true;
}

static bool f413_imu_diag_read_gyro_z_dps(float* out)
{
  int16_t raw_z = 0;

  if (out == NULL)
  {
    return false;
  }

  if (!f413_imu_diag_read_i16_le(F413_IMU_DIAG_OUTZ_G_L, &raw_z))
  {
    return false;
  }

  *out = (float)raw_z * F413_IMU_DIAG_GYRO_SENSITIVITY * F413_IMU_GYRO_Z_SCALE;
  return true;
}

static bool f413_imu_diag_read_accel_xyz_mm_s2(float* ax, float* ay, float* az)
{
  int16_t raw_x = 0;
  int16_t raw_y = 0;
  int16_t raw_z = 0;

  if ((ax == NULL) || (ay == NULL) || (az == NULL))
  {
    return false;
  }

  if (!f413_imu_diag_read_i16_le(F413_IMU_DIAG_OUTX_XL_L, &raw_x) ||
      !f413_imu_diag_read_i16_le(F413_IMU_DIAG_OUTY_XL_L, &raw_y) ||
      !f413_imu_diag_read_i16_le(F413_IMU_DIAG_OUTZ_XL_L, &raw_z))
  {
    return false;
  }

  *ax = (float)raw_x * F413_IMU_DIAG_ACCEL_SENS_MG * F413_IMU_DIAG_GRAVITY_MM_S2;
  *ay = (float)raw_y * F413_IMU_DIAG_ACCEL_SENS_MG * F413_IMU_DIAG_GRAVITY_MM_S2;
  *az = (float)raw_z * F413_IMU_DIAG_ACCEL_SENS_MG * F413_IMU_DIAG_GRAVITY_MM_S2;
  return true;
}

bool f413_imu_diag_whoami_ok(void)
{
  uint8_t who = 0U;

  return f413_imu_diag_read_reg(F413_IMU_DIAG_WHO_AM_I_REG, &who) &&
         (who == F413_IMU_DIAG_WHO_AM_I_EXPECTED);
}

void f413_imu_diag_run_accel_test_once(void)
{
  float off_x = 0.0f;
  float off_y = 0.0f;
  float off_z = 0.0f;
  float vel_x = 0.0f;
  float vel_y = 0.0f;
  float vel_z = 0.0f;
  uint32_t i;
  uint32_t start_ms;
  uint32_t last_ms;
  uint32_t next_print_ms;

  if (f413_ctrl_is_running())
  {
    trace_printf("[HW-TEST][IMU-ACCEL] FAIL(control running)\r\n");
    return;
  }

  if (!f413_imu_diag_config_for_gyro())
  {
    return;
  }

  trace_printf("[HW-TEST][IMU-ACCEL] keep still: offset sampling\r\n");
  HAL_Delay(200U);

  for (i = 0U; i < F413_IMU_DIAG_MANUAL_OFFSET_SAMPLES; i++)
  {
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    if (!f413_imu_diag_read_accel_xyz_mm_s2(&ax, &ay, &az))
    {
      trace_printf("[HW-TEST][IMU-ACCEL] FAIL(read offset)\r\n");
      return;
    }
    off_x += ax;
    off_y += ay;
    off_z += az;
    HAL_Delay(1U);
  }
  off_x /= (float)F413_IMU_DIAG_MANUAL_OFFSET_SAMPLES;
  off_y /= (float)F413_IMU_DIAG_MANUAL_OFFSET_SAMPLES;
  off_z /= (float)F413_IMU_DIAG_MANUAL_OFFSET_SAMPLES;

  trace_printf("[HW-TEST][IMU-ACCEL] offset ax=%.1f ay=%.1f az=%.1f mm/s2\r\n",
               (double)off_x, (double)off_y, (double)off_z);
  trace_printf("[HW-TEST][IMU-ACCEL] start: move forward/backward; control forward axis is Y sign +\r\n");

  start_ms = HAL_GetTick();
  last_ms = start_ms;
  next_print_ms = start_ms;

  while ((uint32_t)(HAL_GetTick() - start_ms) < F413_IMU_DIAG_MANUAL_TEST_MS)
  {
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    uint32_t now_ms;
    float dt;

    if (!f413_imu_diag_read_accel_xyz_mm_s2(&ax, &ay, &az))
    {
      trace_printf("[HW-TEST][IMU-ACCEL] FAIL(read sample)\r\n");
      return;
    }

    now_ms = HAL_GetTick();
    dt = (float)(uint32_t)(now_ms - last_ms) * 0.001f;
    last_ms = now_ms;
    ax -= off_x;
    ay -= off_y;
    az -= off_z;
    vel_x += ax * dt;
    vel_y += ay * dt;
    vel_z += az * dt;

    if ((uint32_t)(now_ms - next_print_ms) >= F413_IMU_DIAG_MANUAL_PRINT_MS)
    {
      trace_printf("[HW-TEST][IMU-ACCEL] t=%lums ax=%.0f ay=%.0f az=%.0f vx=%.0f vy=%.0f vz=%.0f\r\n",
                   (unsigned long)(uint32_t)(now_ms - start_ms),
                   (double)ax, (double)ay, (double)az,
                   (double)vel_x, (double)vel_y, (double)vel_z);
      next_print_ms = now_ms;
    }

    HAL_Delay(F413_IMU_DIAG_MANUAL_SAMPLE_MS);
  }

  trace_printf("[HW-TEST][IMU-ACCEL] done vx=%.0f vy=%.0f vz=%.0f mm/s\r\n",
               (double)vel_x, (double)vel_y, (double)vel_z);
}

void f413_imu_diag_run_manual_turn_test_once(void)
{
  float offset = 0.0f;
  float angle = 0.0f;
  uint32_t i;
  uint32_t start_ms;
  uint32_t last_ms;
  uint32_t next_print_ms;

  if (f413_ctrl_is_running())
  {
    trace_printf("[HW-TEST][IMU-ANGLE] FAIL(control running)\r\n");
    return;
  }

  if (!f413_imu_diag_config_for_gyro())
  {
    return;
  }

  trace_printf("[HW-TEST][IMU-ANGLE] keep still: offset sampling\r\n");
  HAL_Delay(200U);

  for (i = 0U; i < F413_IMU_DIAG_MANUAL_OFFSET_SAMPLES; i++)
  {
    float omega = 0.0f;
    if (!f413_imu_diag_read_gyro_z_dps(&omega))
    {
      trace_printf("[HW-TEST][IMU-ANGLE] FAIL(read offset)\r\n");
      return;
    }
    offset += omega;
    HAL_Delay(1U);
  }
  offset /= (float)F413_IMU_DIAG_MANUAL_OFFSET_SAMPLES;

  trace_printf("[HW-TEST][IMU-ANGLE] offset=%.2fdps\r\n", (double)offset);
  trace_printf("[HW-TEST][IMU-ANGLE] start: CCW/left turn should be positive, CW/right negative\r\n");

  start_ms = HAL_GetTick();
  last_ms = start_ms;
  next_print_ms = start_ms;

  while ((uint32_t)(HAL_GetTick() - start_ms) < F413_IMU_DIAG_MANUAL_TEST_MS)
  {
    float omega = 0.0f;
    uint32_t now_ms;
    float dt;

    if (!f413_imu_diag_read_gyro_z_dps(&omega))
    {
      trace_printf("[HW-TEST][IMU-ANGLE] FAIL(read sample)\r\n");
      return;
    }

    now_ms = HAL_GetTick();
    dt = (float)(uint32_t)(now_ms - last_ms) * 0.001f;
    last_ms = now_ms;
    omega -= offset;
    angle += omega * dt;

    if ((uint32_t)(now_ms - next_print_ms) >= F413_IMU_DIAG_MANUAL_PRINT_MS)
    {
      trace_printf("[HW-TEST][IMU-ANGLE] t=%lums omega_z=%.2fdps angle=%.1fdeg\r\n",
                   (unsigned long)(uint32_t)(now_ms - start_ms),
                   (double)omega,
                   (double)angle);
      next_print_ms = now_ms;
    }

    HAL_Delay(F413_IMU_DIAG_MANUAL_SAMPLE_MS);
  }

  trace_printf("[HW-TEST][IMU-ANGLE] done angle=%.1fdeg\r\n", (double)angle);
}

void f413_imu_diag_run_whoami_test_once(void)
{
  uint8_t who = 0U;
  if (!f413_imu_diag_read_reg(F413_IMU_DIAG_WHO_AM_I_REG, &who))
  {
    trace_printf("[HW-TEST][IMU] FAIL(spi)\r\n");
    return;
  }

  trace_printf("[HW-TEST][IMU] WHO_AM_I=0x%02X expected=0x%02X => %s\r\n",
               (unsigned int)who,
               (unsigned int)F413_IMU_DIAG_WHO_AM_I_EXPECTED,
               (who == F413_IMU_DIAG_WHO_AM_I_EXPECTED) ? "PASS" : "FAIL");
}
