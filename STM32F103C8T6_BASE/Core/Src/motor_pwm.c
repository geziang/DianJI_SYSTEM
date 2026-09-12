/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_pwm.c
  * @brief   TIM2 three-phase PWM driver for M0.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motor_pwm.h"

#include "board_config.h"

static uint8_t motor_pwm_output_started = 0U;

/*
 * PWM 输出层。
 *
 * F103 基线使用 TIM2 普通定时器的 CH1/CH2/CH3 输出 M0 三相 PWM。
 * 这里不实现 SVPWM/FOC，只提供安全启动、停止、清零和原始 compare 写入。
 */
static TIM_HandleTypeDef *motor_pwm_timer(void)
{
  return board_config_get_motor_pwm_timer();
}

/* 初始化阶段先清零，再停止通道，保证默认没有有效 PWM 输出。 */
void motor_pwm_init_safe(void)
{
  motor_pwm_output_started = 0U;
  motor_pwm_force_zero();
  motor_pwm_stop_output();
}

/* 将三相 compare 清零；如果 PWM 已启动，会立刻回到 0 占空比。 */
void motor_pwm_force_zero(void)
{
  TIM_HandleTypeDef *timer;

  timer = motor_pwm_timer();
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, 0U);
}

/*
 * 启动三相 PWM 测试输出。
 * 当前静态诊断不会主动调用它，保留给 BL3 示波器验证阶段使用。
 */
motor_pwm_status_t motor_pwm_start_test_output(void)
{
  TIM_HandleTypeDef *timer;

  timer = motor_pwm_timer();
  motor_pwm_force_zero();

  if (HAL_TIM_PWM_Start(timer, TIM_CHANNEL_1) != HAL_OK)
  {
    return MOTOR_PWM_STATUS_HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(timer, TIM_CHANNEL_2) != HAL_OK)
  {
    (void)HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_1);
    return MOTOR_PWM_STATUS_HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(timer, TIM_CHANNEL_3) != HAL_OK)
  {
    (void)HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_2);
    return MOTOR_PWM_STATUS_HAL_ERROR;
  }

  motor_pwm_output_started = 1U;

  return MOTOR_PWM_STATUS_OK;
}

/* 停止 TIM2 三个 PWM 通道，并清零 compare 作为收尾钳位。 */
void motor_pwm_stop_output(void)
{
  TIM_HandleTypeDef *timer;

  timer = motor_pwm_timer();
  (void)HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(timer, TIM_CHANNEL_3);
  motor_pwm_output_started = 0U;
  motor_pwm_force_zero();
}

/* 写入原始 compare 值前先做 ARR 上限检查，避免写出无意义占空比。 */
motor_pwm_status_t motor_pwm_set_raw(uint32_t ch1_ticks,
                                     uint32_t ch2_ticks,
                                     uint32_t ch3_ticks)
{
  TIM_HandleTypeDef *timer;

  if ((ch1_ticks > BOARD_CONFIG_PWM_PERIOD_TICKS) ||
      (ch2_ticks > BOARD_CONFIG_PWM_PERIOD_TICKS) ||
      (ch3_ticks > BOARD_CONFIG_PWM_PERIOD_TICKS))
  {
    return MOTOR_PWM_STATUS_INVALID_ARG;
  }

  if (motor_pwm_output_started == 0U)
  {
    return MOTOR_PWM_STATUS_NOT_STARTED;
  }

  timer = motor_pwm_timer();
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1, ch1_ticks);
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_2, ch2_ticks);
  __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, ch3_ticks);

  return MOTOR_PWM_STATUS_OK;
}

uint8_t motor_pwm_is_output_started(void)
{
  return motor_pwm_output_started;
}
