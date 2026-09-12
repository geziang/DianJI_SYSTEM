/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_pwm.h
  * @brief   TIM2 three-phase PWM driver for M0.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  /* 操作成功。 */
  MOTOR_PWM_STATUS_OK = 0,

  /* 输入参数越界，例如 compare ticks 大于 ARR。 */
  MOTOR_PWM_STATUS_INVALID_ARG,

  /* HAL 启动/停止 PWM 通道失败。 */
  MOTOR_PWM_STATUS_HAL_ERROR,

  /* PWM 通道尚未启动，禁止写入主动调制量。 */
  MOTOR_PWM_STATUS_NOT_STARTED
} motor_pwm_status_t;

/* 安全初始化：清零 TIM2 三路 compare，并停止 PWM 输出。 */
void motor_pwm_init_safe(void);

/* 只把三相 compare 清零，不改变 PWM 通道是否已经启动。 */
void motor_pwm_force_zero(void);

/* 后续示波器测试用入口：先清零，再启动 TIM2 CH1/CH2/CH3。 */
motor_pwm_status_t motor_pwm_start_test_output(void);

/* 停止三相 PWM，并再次清零 compare。 */
void motor_pwm_stop_output(void);

/* 设置三相原始 compare ticks，调用者必须使用 BOARD_CONFIG_PWM_PERIOD_TICKS 为上限。 */
motor_pwm_status_t motor_pwm_set_raw(uint32_t ch1_ticks,
                                     uint32_t ch2_ticks,
                                     uint32_t ch3_ticks);

/* PWM 三通道均已成功启动时返回非零。 */
uint8_t motor_pwm_is_output_started(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PWM_H */
