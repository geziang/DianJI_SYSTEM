/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_baseline.c
  * @brief   F103 baseline application entry points called from CubeMX main.c.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "app_baseline.h"

#include "baseline_diag.h"
#include "board_config.h"
#include "control_loop.h"
#include "debug_log.h"
#include "motor_drv.h"
#include "motor_pwm.h"

/*
 * 应用层总入口。
 *
 * 这个文件只负责把各个自定义模块串起来，不直接操作具体引脚。
 * F103 的 TIM2、ADC1、I2C2、USART1、PA8 等底层绑定集中在 board_config 中。
 */

/*
 * 初始化顺序对应基线文档 BL1/BL2：
 * 1. 先钳住安全输出，保证 MOTOR_EN 关闭、PWM 比较值为 0、M1 输出低。
 * 2. 初始化驱动使能层和 PWM 层，默认不启动功率输出。
 * 3. 打开 USART1 日志，输出启动证据。
 * 4. 运行静态诊断流水线，检查 ADC/I2C/PWM/控制状态。
 */
void app_baseline_init(void)
{
  board_config_apply_safe_outputs();
  motor_drv_init_safe();
  motor_pwm_init_safe();

  debug_log_init(board_config_get_debug_uart());
  debug_log_write_line("STM32F103C8T6 baseline start");
  debug_log_write_line("version: " APP_BASELINE_VERSION);
  debug_log_write_line("hardware: V3P supply <=10V; 12V prohibited");
  debug_log_write_line("state: safe idle, motor output disabled");
  debug_log_write_line("commands: s=start open loop, x=stop, c=clear fault, ?=status");

  baseline_diag_init();
}

static void app_baseline_log_control_status(void)
{
  debug_log_write("[CTRL] state=");
  debug_log_write(control_loop_state_text(control_loop_get_state()));
  debug_log_write(" fault=");
  debug_log_write_line(control_loop_fault_text(control_loop_get_fault()));
}

static void app_baseline_handle_command(uint8_t command)
{
  control_loop_command_status_t status;

  switch (command)
  {
    case 's':
    case 'S':
      status = control_loop_request_open_loop_start();
      debug_log_write_line((status == CONTROL_LOOP_COMMAND_ACCEPTED) ?
                           "[CMD] start accepted" : "[CMD] start rejected");
      break;
    case 'x':
    case 'X':
      control_loop_request_stop();
      debug_log_write_line("[CMD] stop requested");
      break;
    case 'c':
    case 'C':
      status = control_loop_request_fault_clear();
      debug_log_write_line((status == CONTROL_LOOP_COMMAND_ACCEPTED) ?
                           "[CMD] fault cleared" : "[CMD] clear rejected");
      break;
    case '?':
      app_baseline_log_control_status();
      break;
    default:
      break;
  }
}

static void app_baseline_poll_uart_command(void)
{
  uint8_t command;

  if (HAL_UART_Receive(board_config_get_debug_uart(), &command, 1U, 0U) == HAL_OK)
  {
    app_baseline_handle_command(command);
  }
}

/* 主循环入口。只接受显式串口命令进入短时开环，不进入闭环控制。 */
void app_baseline_poll(void)
{
  app_baseline_poll_uart_command();
  baseline_diag_poll();
  HAL_Delay(1U);
}
