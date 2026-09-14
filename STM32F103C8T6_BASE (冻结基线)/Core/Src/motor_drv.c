/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_drv.c
  * @brief   Motor enable/disable driver.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motor_drv.h"

#include "board_config.h"
#include "motor_pwm.h"

/*
 * MOTOR_EN 控制层。
 *
 * 这里只管理“驱动允许/禁止输出”这一件事，PWM 占空比和控制算法不放在这里。
 * F103 当前绑定为 PA8；BL9342 EN 已确认高电平使能、低电平关闭。
 * 实际 10 V/FD6287T 失能响应仍需 BL3 实测。
 */
static motor_drv_state_t motor_drv_state = MOTOR_DRV_STATE_DISABLED;

/* 安全初始化统一复用 disable，避免初始化路径和失能路径行为不一致。 */
void motor_drv_init_safe(void)
{
  motor_drv_disable();
}

/* 写入安全电平，并同步软件状态。 */
void motor_drv_disable(void)
{
  HAL_GPIO_WritePin(BOARD_CONFIG_MOTOR_EN_GPIO_PORT,
                    BOARD_CONFIG_MOTOR_EN_PIN,
                    BOARD_CONFIG_MOTOR_EN_SAFE_LEVEL);
  motor_drv_state = MOTOR_DRV_STATE_DISABLED;
}

/* 只作为后续显式测试入口保留；调用者必须先保证 PWM 处于安全状态。 */
motor_drv_status_t motor_drv_enable_for_test(void)
{
  if (motor_pwm_is_output_started() == 0U)
  {
    return MOTOR_DRV_STATUS_PWM_NOT_READY;
  }

  HAL_GPIO_WritePin(BOARD_CONFIG_MOTOR_EN_GPIO_PORT,
                    BOARD_CONFIG_MOTOR_EN_PIN,
                    BOARD_CONFIG_MOTOR_EN_ACTIVE_LEVEL);
  motor_drv_state = MOTOR_DRV_STATE_ENABLED;
  return MOTOR_DRV_STATUS_OK;
}

/* 返回本模块最近一次写入意图，不等价于硬件故障反馈。 */
motor_drv_state_t motor_drv_get_state(void)
{
  return motor_drv_state;
}

uint8_t motor_drv_is_enabled(void)
{
  return (uint8_t)(motor_drv_state == MOTOR_DRV_STATE_ENABLED);
}
