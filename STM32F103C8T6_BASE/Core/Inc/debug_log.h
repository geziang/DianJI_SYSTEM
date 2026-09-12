/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    debug_log.h
  * @brief   Minimal blocking UART log service for baseline evidence.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 保存 CubeMX 已初始化好的 UART 句柄；当前 F103 绑定为 USART1。 */
void debug_log_init(UART_HandleTypeDef *uart);

/* 阻塞发送字符串，不自动补换行；用于 BL0/BL1 小日志量验证。 */
void debug_log_write(const char *text);

/* 发送字符串并补 CRLF，便于串口助手按行查看诊断结果。 */
void debug_log_write_line(const char *text);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_LOG_H */
