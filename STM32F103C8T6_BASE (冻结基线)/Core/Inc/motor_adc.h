/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_adc.h
  * @brief   Raw ADC readout for F103 phase-current channels.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef MOTOR_ADC_H
#define MOTOR_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  /* 采样成功。 */
  MOTOR_ADC_STATUS_OK = 0,

  /* 调用参数为空。 */
  MOTOR_ADC_STATUS_INVALID_ARG,

  /* HAL ADC 启动或等待转换失败。 */
  MOTOR_ADC_STATUS_HAL_ERROR
} motor_adc_status_t;

typedef struct
{
  /* PB0 / ADC1_IN8，对应文档中的 M0_OUT1_CS。 */
  uint16_t phase_u_raw;

  /* PB1 / ADC1_IN9，对应文档中的 M0_OUT2_CS。 */
  uint16_t phase_v_raw;

  /* 当前 F103 基线没有确认 VBUS 接入 ADC，保留字段但标记不可用。 */
  uint16_t vbus_raw;
  uint8_t vbus_available;
} motor_adc_raw_sample_t;

/* 启动 ADC1 扫描转换，顺序读取 IN8/IN9 两个 rank。 */
motor_adc_status_t motor_adc_read_raw(motor_adc_raw_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ADC_H */
