/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    debug_log.c
  * @brief   Minimal blocking UART log service for baseline evidence.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "debug_log.h"

#include <stddef.h>
#include <string.h>

#define DEBUG_LOG_TX_TIMEOUT_MS  (100U)

/* 当前日志串口，由 app_baseline_init() 注入 USART1 句柄。 */
static UART_HandleTypeDef *debug_log_uart;

/* 日志模块不初始化 UART，只记录 CubeMX 已经初始化完成的句柄。 */
void debug_log_init(UART_HandleTypeDef *uart)
{
  debug_log_uart = uart;
}

/*
 * 最小阻塞式日志输出。
 * 静态诊断阶段日志频率低，先用阻塞发送换取实现简单和可观察性。
 */
void debug_log_write(const char *text)
{
  size_t length;

  if ((debug_log_uart == NULL) || (text == NULL))
  {
    return;
  }

  length = strlen(text);
  if (length == 0U)
  {
    return;
  }

  (void)HAL_UART_Transmit(debug_log_uart,
                          (uint8_t *)text,
                          (uint16_t)length,
                          DEBUG_LOG_TX_TIMEOUT_MS);
}

/* 统一使用 CRLF，适配常见 Windows 串口助手显示。 */
void debug_log_write_line(const char *text)
{
  debug_log_write(text);
  debug_log_write("\r\n");
}
