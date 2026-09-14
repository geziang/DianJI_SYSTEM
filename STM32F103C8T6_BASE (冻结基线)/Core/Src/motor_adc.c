/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_adc.c
  * @brief   Raw ADC readout for F103 phase-current channels.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motor_adc.h"

#include "board_config.h"
#include <stddef.h>

#define MOTOR_ADC_POLL_TIMEOUT_MS  (5U)

/*
 * ADC 采样层。
 *
 * F103 当前只有 ADC1 扫描两个 rank：
 * - Rank 1: ADC1_IN8 / PB0 / M0_OUT1_CS
 * - Rank 2: ADC1_IN9 / PB1 / M0_OUT2_CS
 *
 * 文档中明确 VBUS/POWER_ADC 未确认接入，因此这里不读取母线电压。
 */

/* 等待一次转换完成并取值；由 motor_adc_read_raw() 控制 Start/Stop 生命周期。 */
static motor_adc_status_t motor_adc_poll_value(ADC_HandleTypeDef *adc, uint16_t *raw)
{
  uint32_t value;

  if ((adc == NULL) || (raw == NULL))
  {
    return MOTOR_ADC_STATUS_INVALID_ARG;
  }

  if (HAL_ADC_PollForConversion(adc, MOTOR_ADC_POLL_TIMEOUT_MS) != HAL_OK)
  {
    return MOTOR_ADC_STATUS_HAL_ERROR;
  }

  value = HAL_ADC_GetValue(adc);
  *raw = (uint16_t)value;
  return MOTOR_ADC_STATUS_OK;
}

/*
 * 读取两相电流原始值。
 * 当前采用阻塞式软件触发，适合 BL2 静态采样；后续 FOC 前再评估 TIM 触发/DMA。
 */
motor_adc_status_t motor_adc_read_raw(motor_adc_raw_sample_t *sample)
{
  ADC_HandleTypeDef *adc;

  if (sample == NULL)
  {
    return MOTOR_ADC_STATUS_INVALID_ARG;
  }

  adc = board_config_get_motor_adc();
  sample->phase_u_raw = 0U;
  sample->phase_v_raw = 0U;
  sample->vbus_raw = 0U;
  /* 当前硬件基线未确认 VBUS ADC，诊断层会打印 vbus=NA。 */
  sample->vbus_available = 0U;

  if (HAL_ADC_Start(adc) != HAL_OK)
  {
    return MOTOR_ADC_STATUS_HAL_ERROR;
  }

  if (motor_adc_poll_value(adc, &sample->phase_u_raw) != MOTOR_ADC_STATUS_OK)
  {
    (void)HAL_ADC_Stop(adc);
    return MOTOR_ADC_STATUS_HAL_ERROR;
  }

  if (motor_adc_poll_value(adc, &sample->phase_v_raw) != MOTOR_ADC_STATUS_OK)
  {
    (void)HAL_ADC_Stop(adc);
    return MOTOR_ADC_STATUS_HAL_ERROR;
  }

  (void)HAL_ADC_Stop(adc);
  return MOTOR_ADC_STATUS_OK;
}
