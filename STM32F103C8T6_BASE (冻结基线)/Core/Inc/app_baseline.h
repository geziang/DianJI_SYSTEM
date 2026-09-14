/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_baseline.h
  * @brief   F103 baseline application entry points called from CubeMX main.c.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef APP_BASELINE_H
#define APP_BASELINE_H

#ifdef __cplusplus
extern "C" {
#endif

/* 基线版本号会通过 USART1 日志打印，便于和测试记录对齐。 */
#define APP_BASELINE_VERSION  "F103-BL-static-diag-0.1"

/* CubeMX 外设初始化完成后调用：进入安全态并启动静态诊断。 */
void app_baseline_init(void);

/* main.c 的 while(1) 周期调用：驱动诊断流水线和低频监视。 */
void app_baseline_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BASELINE_H */
