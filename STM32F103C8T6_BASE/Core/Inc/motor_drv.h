/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor_drv.h
  * @brief   Motor enable/disable driver.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef MOTOR_DRV_H
#define MOTOR_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  /* 软件认为驱动处于失能状态，对应 MOTOR_EN 安全电平。 */
  MOTOR_DRV_STATE_DISABLED = 0,

  /* 仅明确测试路径允许进入；当前静态诊断不会主动调用使能。 */
  MOTOR_DRV_STATE_ENABLED
} motor_drv_state_t;

typedef enum
{
  MOTOR_DRV_STATUS_OK = 0,
  MOTOR_DRV_STATUS_PWM_NOT_READY
} motor_drv_status_t;

/* 初始化时强制失能，避免上电误输出。 */
void motor_drv_init_safe(void);

/* 写入 MOTOR_EN 安全电平，关闭驱动输出能力。 */
void motor_drv_disable(void);

/* 预留给后续门控测试；调用前必须确认 PWM 为安全占空比。 */
motor_drv_status_t motor_drv_enable_for_test(void);

/* 查询软件记录状态，供诊断日志使用。 */
motor_drv_state_t motor_drv_get_state(void);
uint8_t motor_drv_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRV_H */
