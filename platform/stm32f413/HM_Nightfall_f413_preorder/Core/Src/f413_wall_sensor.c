#include "f413_wall_sensor.h"

#include <string.h>

#include "main.h"
#include "nvm_params.h"
#include "params.h"

#define F413_WALL_SENSOR_ADC_CHANNELS (9U)
#define F413_WALL_SENSOR_SAT_ADC (4090U)

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim6;

static volatile uint8_t g_wall_sensor_adc_phase = 0U;
static volatile uint8_t g_wall_sensor_adc_inflight = 0U;
static volatile uint8_t g_wall_sensor_ready_mask = 0U;
static volatile uint16_t g_wall_adc_dma_off[F413_WALL_SENSOR_ADC_CHANNELS];
static volatile uint16_t g_wall_adc_dma_on[F413_WALL_SENSOR_ADC_CHANNELS];
static volatile uint16_t g_wall_adc_r_off = 0U;
static volatile uint16_t g_wall_adc_l_off = 0U;
static volatile uint16_t g_wall_adc_fr_off = 0U;
static volatile uint16_t g_wall_adc_fl_off = 0U;
static volatile uint16_t g_wall_adc_r_raw = 0U;
static volatile uint16_t g_wall_adc_l_raw = 0U;
static volatile uint16_t g_wall_adc_fr_raw = 0U;
static volatile uint16_t g_wall_adc_fl_raw = 0U;
static volatile uint16_t g_wall_adc_r = 0U;
static volatile uint16_t g_wall_adc_l = 0U;
static volatile uint16_t g_wall_adc_fr = 0U;
static volatile uint16_t g_wall_adc_fl = 0U;
static volatile uint16_t g_wall_adc_vbat = 0U;
static volatile uint16_t g_wall_offset_r = 0U;
static volatile uint16_t g_wall_offset_l = 0U;
static volatile uint16_t g_wall_offset_fr = 0U;
static volatile uint16_t g_wall_offset_fl = 0U;

static uint16_t f413_wall_sensor_subtract_u16(uint16_t on, uint16_t off, uint16_t offset)
{
  int32_t value = (int32_t)on - (int32_t)off - (int32_t)offset;

  if (value <= 0)
  {
    return 0U;
  }
  if (value > 65535)
  {
    return 65535U;
  }
  return (uint16_t)value;
}

static bool f413_sensor_params_is_nvm_test_blob(const nvm_sensor_params_t* params)
{
  if (params == NULL)
  {
    return false;
  }

  return (params->base_l == 1111U) &&
         (params->base_r == 1222U) &&
         (params->base_f == 1333U) &&
         (params->wall_offset_r == 200U) &&
         (params->wall_offset_l == 210U) &&
         (params->wall_offset_fr == 220U) &&
         (params->wall_offset_fl == 230U);
}

static HAL_StatusTypeDef f413_wall_sensor_adc_dma_start(volatile uint16_t* dst)
{
  if (dst == NULL)
  {
    return HAL_ERROR;
  }

  (void)HAL_ADC_Stop_DMA(&hadc1);
  return HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dst, F413_WALL_SENSOR_ADC_CHANNELS);
}

static void f413_wall_sensor_load_params(void)
{
  nvm_sensor_params_t params;

  if (!nvm_params_sensor_load(&params) ||
      f413_sensor_params_is_nvm_test_blob(&params))
  {
    nvm_params_sensor_defaults(&params);
  }

  g_wall_offset_r = params.wall_offset_r;
  g_wall_offset_l = params.wall_offset_l;
  g_wall_offset_fr = params.wall_offset_fr;
  g_wall_offset_fl = params.wall_offset_fl;
}

static void f413_ir_emitters_set(GPIO_PinState fr,
                                 GPIO_PinState r,
                                 GPIO_PinState fl,
                                 GPIO_PinState l)
{
  HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, fr);
  HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, r);
  HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, fl);
  HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, l);
}

bool f413_wall_sensor_start_async(void)
{
  f413_wall_sensor_load_params();
  g_wall_sensor_adc_phase = 0U;
  g_wall_sensor_adc_inflight = 0U;
  g_wall_sensor_ready_mask = 0U;
  memset((void*)g_wall_adc_dma_off, 0, sizeof(g_wall_adc_dma_off));
  memset((void*)g_wall_adc_dma_on, 0, sizeof(g_wall_adc_dma_on));
  f413_ir_emitters_set(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  return HAL_TIM_Base_Start_IT(&htim6) == HAL_OK;
}

bool f413_wall_sensor_read_snapshot(f413_wall_sensor_snapshot_t* out)
{
  if (out == NULL)
  {
    return false;
  }
  if ((g_wall_sensor_ready_mask & 0x03U) != 0x03U)
  {
    return false;
  }

  memset(out, 0, sizeof(*out));
  out->fr_off = g_wall_adc_fr_off;
  out->r_off = g_wall_adc_r_off;
  out->fl_off = g_wall_adc_fl_off;
  out->l_off = g_wall_adc_l_off;
  out->vbat_off = g_wall_adc_vbat;
  out->fr_on = g_wall_adc_fr_raw;
  out->r_on = g_wall_adc_r_raw;
  out->fl_on = g_wall_adc_fl_raw;
  out->l_on = g_wall_adc_l_raw;
  out->vbat_on = g_wall_adc_vbat;
  out->fr_delta = (int32_t)g_wall_adc_fr;
  out->r_delta = (int32_t)g_wall_adc_r;
  out->fl_delta = (int32_t)g_wall_adc_fl;
  out->l_delta = (int32_t)g_wall_adc_l;
  out->front_wall = (out->fr_delta > WALL_BASE_FR) || (out->fl_delta > WALL_BASE_FL);
  out->right_wall = out->r_delta > WALL_BASE_R;
  out->left_wall = out->l_delta > WALL_BASE_L;
  out->saturated = (out->fr_on >= F413_WALL_SENSOR_SAT_ADC) ||
                   (out->r_on >= F413_WALL_SENSOR_SAT_ADC) ||
                   (out->fl_on >= F413_WALL_SENSOR_SAT_ADC) ||
                   (out->l_on >= F413_WALL_SENSOR_SAT_ADC);
  return true;
}

bool f413_wall_sensor_read_adc_raw(uint16_t* fr, uint16_t* r, uint16_t* fl, uint16_t* l, uint16_t* vbat)
{
  if ((fr == NULL) || (r == NULL) || (fl == NULL) || (l == NULL) || (vbat == NULL))
  {
    return false;
  }

  if ((g_wall_sensor_ready_mask & 0x03U) != 0x03U)
  {
    return false;
  }

  *fr = g_wall_adc_fr;
  *r = g_wall_adc_r;
  *fl = g_wall_adc_fl;
  *l = g_wall_adc_l;
  *vbat = g_wall_adc_vbat;
  return true;
}

void f413_wall_sensor_get_debug_state(uint16_t* off_r,
                                      uint16_t* off_l,
                                      uint16_t* off_fr,
                                      uint16_t* off_fl,
                                      uint8_t* ready_mask,
                                      uint8_t* phase,
                                      uint8_t* inflight)
{
  if (off_r != NULL)
  {
    *off_r = g_wall_offset_r;
  }
  if (off_l != NULL)
  {
    *off_l = g_wall_offset_l;
  }
  if (off_fr != NULL)
  {
    *off_fr = g_wall_offset_fr;
  }
  if (off_fl != NULL)
  {
    *off_fl = g_wall_offset_fl;
  }
  if (ready_mask != NULL)
  {
    *ready_mask = g_wall_sensor_ready_mask;
  }
  if (phase != NULL)
  {
    *phase = g_wall_sensor_adc_phase;
  }
  if (inflight != NULL)
  {
    *inflight = g_wall_sensor_adc_inflight;
  }
}

void f413_wall_sensor_tim6_tick(void)
{
  HAL_StatusTypeDef st = HAL_OK;
  uint8_t start_dma = 0U;

  if (g_wall_sensor_adc_inflight != 0U)
  {
    return;
  }

  switch (g_wall_sensor_adc_phase & 0x07U)
  {
    case 0U:
      HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_RESET);
      break;
    case 1U:
      st = f413_wall_sensor_adc_dma_start(g_wall_adc_dma_off);
      start_dma = 1U;
      break;
    case 2U:
      HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_SET);
      break;
    case 3U:
      st = f413_wall_sensor_adc_dma_start(g_wall_adc_dma_on);
      start_dma = 1U;
      break;
    case 4U:
      HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_RESET);
      break;
    case 5U:
      st = f413_wall_sensor_adc_dma_start(g_wall_adc_dma_off);
      start_dma = 1U;
      break;
    case 6U:
      HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_SET);
      break;
    case 7U:
      st = f413_wall_sensor_adc_dma_start(g_wall_adc_dma_on);
      start_dma = 1U;
      break;
    default:
      g_wall_sensor_adc_phase = 0U;
      break;
  }

  if ((start_dma != 0U) && (st == HAL_OK))
  {
    g_wall_sensor_adc_inflight = 1U;
  }
  else if (start_dma == 0U)
  {
    g_wall_sensor_adc_phase = (uint8_t)((g_wall_sensor_adc_phase + 1U) & 0x07U);
  }
}

void f413_wall_sensor_adc_complete(ADC_HandleTypeDef* hadc)
{
  if ((hadc == NULL) || (hadc->Instance != ADC1))
  {
    return;
  }

  switch (g_wall_sensor_adc_phase & 0x07U)
  {
    case 3U:
    {
      uint16_t r_off = (uint16_t)(((uint32_t)g_wall_adc_dma_off[0] + (uint32_t)g_wall_adc_dma_off[2]) / 2U);
      uint16_t l_off = (uint16_t)(((uint32_t)g_wall_adc_dma_off[1] + (uint32_t)g_wall_adc_dma_off[3]) / 2U);
      uint16_t r_on = (uint16_t)(((uint32_t)g_wall_adc_dma_on[0] + (uint32_t)g_wall_adc_dma_on[2]) / 2U);
      uint16_t l_on = (uint16_t)(((uint32_t)g_wall_adc_dma_on[1] + (uint32_t)g_wall_adc_dma_on[3]) / 2U);

      g_wall_adc_r_off = r_off;
      g_wall_adc_l_off = l_off;
      g_wall_adc_r_raw = r_on;
      g_wall_adc_l_raw = l_on;
      g_wall_adc_r = f413_wall_sensor_subtract_u16(r_on, r_off, g_wall_offset_r);
      g_wall_adc_l = f413_wall_sensor_subtract_u16(l_on, l_off, g_wall_offset_l);
      HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, GPIO_PIN_RESET);
      g_wall_sensor_ready_mask |= 0x01U;
      break;
    }
    case 7U:
    {
      uint16_t fr_off = (uint16_t)(((uint32_t)g_wall_adc_dma_off[4] + (uint32_t)g_wall_adc_dma_off[6]) / 2U);
      uint16_t fl_off = (uint16_t)(((uint32_t)g_wall_adc_dma_off[5] + (uint32_t)g_wall_adc_dma_off[7]) / 2U);
      uint16_t fr_on = (uint16_t)(((uint32_t)g_wall_adc_dma_on[4] + (uint32_t)g_wall_adc_dma_on[6]) / 2U);
      uint16_t fl_on = (uint16_t)(((uint32_t)g_wall_adc_dma_on[5] + (uint32_t)g_wall_adc_dma_on[7]) / 2U);

      g_wall_adc_fr_off = fr_off;
      g_wall_adc_fl_off = fl_off;
      g_wall_adc_fr_raw = fr_on;
      g_wall_adc_fl_raw = fl_on;
      g_wall_adc_fr = f413_wall_sensor_subtract_u16(fr_on, fr_off, g_wall_offset_fr);
      g_wall_adc_fl = f413_wall_sensor_subtract_u16(fl_on, fl_off, g_wall_offset_fl);
      g_wall_adc_vbat = g_wall_adc_dma_off[8];
      HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, GPIO_PIN_RESET);
      g_wall_sensor_ready_mask |= 0x02U;
      break;
    }
    default:
      break;
  }

  g_wall_sensor_adc_inflight = 0U;
  g_wall_sensor_adc_phase = (uint8_t)((g_wall_sensor_adc_phase + 1U) & 0x07U);
}
