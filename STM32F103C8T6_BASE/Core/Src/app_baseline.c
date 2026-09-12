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
#include "encoder_cache.h"
#include "foc_runtime.h"
#include "motor_adc.h"
#include "motor_drv.h"
#include "motor_pwm.h"
#include "safety_manager.h"

#include <stdio.h>

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
  debug_log_write_line("STM32F103C8T6 identification test " APP_BASELINE_VERSION);
  debug_log_write_line("safety: V3P supply <=10V (12V PROHIBITED); motor output disabled at boot");
  debug_log_write_line("cmds: p=confirm step r=restart a=apply d=discard x=stop c=clear(fault|storage) i/?=diag");

  safety_manager_init();
  encoder_cache_init();
  foc_runtime_init();
  baseline_diag_init();
}

static void app_baseline_log_control_status(void)
{
  debug_log_write("[CTRL] state=");
  debug_log_write(control_loop_state_text(control_loop_get_state()));
  debug_log_write(" fault=");
  debug_log_write(control_loop_fault_text(control_loop_get_fault()));
  debug_log_write(" safety=");
  debug_log_write(safety_manager_state_text(safety_manager_get_state()));
  debug_log_write(" safety_fault=");
  debug_log_write_line(safety_manager_fault_text(safety_manager_get_fault()));
}

static void app_baseline_log_current_diagnostic(void)
{
  motor_adc_current_diagnostic_t current;
  motor_adc_raw_sample_t raw;
  const motor_current_sample_t *synchronised;
  motor_adc_status_t status;
  char line[160];

  motor_adc_get_current_diagnostic(&current);
  synchronised = motor_adc_get_latest_sample();
  status = motor_adc_read_raw(&raw);
  if (status == MOTOR_ADC_STATUS_OK)
  {
    (void)snprintf(line, sizeof(line),
                   "[CUR] raw=%u,%u zero=%u,%u raw_p2p=%u,%u filtered_p2p=%u,%u quality=%s state=%s closed_loop=%u",
                   (unsigned int)raw.phase_u_raw,
                   (unsigned int)raw.phase_v_raw,
                   (unsigned int)current.zero.phase_u_zero_raw,
                   (unsigned int)current.zero.phase_v_zero_raw,
                   (unsigned int)current.zero.phase_u_peak_to_peak_raw,
                   (unsigned int)current.zero.phase_v_peak_to_peak_raw,
                   (unsigned int)current.zero.phase_u_filtered_peak_to_peak_raw,
                   (unsigned int)current.zero.phase_v_filtered_peak_to_peak_raw,
                   motor_adc_zero_quality_text(current.zero.quality),
                   motor_adc_current_state_text(current.state),
                   (unsigned int)current.closed_loop_allowed);
    debug_log_write_line(line);
    (void)snprintf(line, sizeof(line),
                   "[SAMPLE] cycle=%lu valid=%u saturated=%u timeout=%u age_ms=%lu",
                   (unsigned long)synchronised->pwm_cycle,
                   (unsigned int)synchronised->valid,
                   (unsigned int)synchronised->saturated,
                   (unsigned int)synchronised->timeout,
                   (unsigned long)(HAL_GetTick() - synchronised->timestamp_ms));
    debug_log_write_line(line);
  }
  else
  {
    debug_log_write_line("[CUR] raw read failed");
  }
}

static void app_baseline_handle_command(uint8_t command)
{
  control_loop_command_status_t status;
  uint8_t safety_cleared;

  switch (command)
  {
    case 's':
    case 'S':
      debug_log_write_line("[CMD] open-loop path is disabled in identification firmware; use r then p");
      break;
    case 'x':
    case 'X':
      control_loop_request_stop();
      debug_log_write_line("[CMD] stop requested");
      break;
    case 'r':
    case 'R':
      status = control_loop_request_identification_start();
      debug_log_write_line((status == CONTROL_LOOP_COMMAND_ACCEPTED) ?
                           "[CMD] identification restart accepted" :
                           "[CMD] restart rejected");
      break;
    case 'p':
    case 'P':
      /* accepted 的启动证据由 control_loop 的 [RUN] 行给出，这里只回显拒绝原因。 */
      status = control_loop_request_power_confirm();
      if (status != CONTROL_LOOP_COMMAND_ACCEPTED)
      {
        debug_log_write_line("[CMD] power step rejected: wait for POWER_ARMED");
      }
      break;
    case 'a':
    case 'A':
      status = control_loop_request_candidate_apply();
      debug_log_write_line((status == CONTROL_LOOP_COMMAND_ACCEPTED) ?
                           "[CMD] candidate apply accepted" :
                           "[CMD] apply rejected: no approved candidate");
      break;
    case 'd':
    case 'D':
      status = control_loop_request_candidate_discard();
      debug_log_write_line((status == CONTROL_LOOP_COMMAND_ACCEPTED) ?
                           "[CMD] candidate discarded" :
                           "[CMD] discard rejected: no candidate review");
      break;
    case 'c':
    case 'C':
      /* 方案 A（SPEC-RUN FR-2.5）：FAULT 态 c= 清故障回到 TEST_BOOT；
       * 非 FAULT 态 c= 清参数存储——双页擦除 + RAM 学习值/注入包失效，
       * 清除后等同未标定，需重跑 r=p 标定序列。 */
      if (control_loop_get_state() == CONTROL_LOOP_STATE_FAULT)
      {
        status = control_loop_request_fault_clear();
        safety_cleared = safety_manager_request_fault_clear();
        debug_log_write_line(((status == CONTROL_LOOP_COMMAND_ACCEPTED) ||
                              (safety_cleared != 0U)) ?
                             "[CMD] fault cleared" : "[CMD] clear rejected");
      }
      else
      {
        status = control_loop_request_clear_storage();
        debug_log_write_line((status == CONTROL_LOOP_COMMAND_ACCEPTED) ?
                             "[CMD] parameter storage cleared (uncalibrated)" :
                             "[CMD] clear rejected: silent states only (SAFE_IDLE/TEST_READY/...)");
      }
      break;
    case 'i':
    case 'I':
      app_baseline_log_current_diagnostic();
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

/* Test firmware polls the low-frequency state machine; PWM output still only
 * occurs inside a confirmed power step. */
void app_baseline_poll(void)
{
  app_baseline_poll_uart_command();
  encoder_cache_poll();
  safety_manager_poll();
  baseline_diag_poll();
  HAL_Delay(1U);
}
