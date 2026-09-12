/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    board_config.c
  * @brief   Board-level binding for the STM32F103C8T6 baseline firmware.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "board_config.h"

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;

/*
 * 板级适配层。
 *
 * 业务/诊断代码不直接使用 hadc1、hi2c2、htim2、huart1 这些 CubeMX 变量名。
 * 如果后续 F103 配置调整，只优先改这里，减少上层代码对具体外设的耦合。
 */

ADC_HandleTypeDef *board_config_get_motor_adc(void)
{
  return &hadc1;
}

I2C_HandleTypeDef *board_config_get_encoder_i2c(void)
{
  return &hi2c2;
}

TIM_HandleTypeDef *board_config_get_motor_pwm_timer(void)
{
  return &htim2;
}

TIM_HandleTypeDef *board_config_get_current_sample_timer(void)
{
  return &htim3;
}

UART_HandleTypeDef *board_config_get_debug_uart(void)
{
  return &huart1;
}

/*
 * 全局安全输出钳位：
 * - PA8 MOTOR_EN 写入安全电平。
 * - PA3/PA6/PA7 对应未使用的 M1 PWM 输入，保持低电平。
 * - TIM2 三路 compare 清零，确保 M0 PWM 占空比为 0。
 */
void board_config_apply_safe_outputs(void)
{
  HAL_GPIO_WritePin(BOARD_CONFIG_MOTOR_EN_GPIO_PORT,
                    BOARD_CONFIG_MOTOR_EN_PIN,
                    BOARD_CONFIG_MOTOR_EN_SAFE_LEVEL);

  HAL_GPIO_WritePin(GPIOA,
                    M1_IN1_Pin | M1_IN2_Pin | M1_IN3_Pin,
                    GPIO_PIN_RESET);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
}
