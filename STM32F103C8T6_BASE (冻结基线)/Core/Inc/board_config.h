/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    board_config.h
  * @brief   Board-level binding for the STM32F103C8T6 baseline firmware.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 功率级适配事实：DengFOC V3Pro 4.0 为双通道三相功率板。
 * 本 F103 基线只使用 M0；V3P 的完整接口和验证边界见
 * ../V3P驱动板硬件适配说明.md。
 */
#define BOARD_CONFIG_POWER_STAGE_ID                 "DengFOC-V3Pro-4.0"
#define BOARD_CONFIG_ACTIVE_MOTOR_CHANNEL           (0U)
#define BOARD_CONFIG_POWER_STAGE_MOTOR_CHANNELS     (2U)

/* TIM2 运行在 72 MHz，ARR=3599，对应约 20 kHz 三相 PWM。 */
#define BOARD_CONFIG_PWM_PERIOD_TICKS       (3599U)

/*
 * V3P 原理图确认每一路 M0_INx 同时接入 FD6287T 的 HINx/LINx#。
 * FD6287T 负责互补 HO/LO 输出和约 100~300 ns 固定死区；F103 不生成六路
 * 互补 PWM，也不配置软件死区。输入/输出波形、死区适配性和实际失能行为
 * 当前基线按接口契约使用；若空载开环异常，再回溯波形、死区和失能响应。
 */
#define BOARD_CONFIG_PWM_INTERFACE_VERIFIED         (0U)
#define BOARD_CONFIG_PWM_DEADTIME_REQUIREMENT_KNOWN (0U)

/*
 * MOTOR_EN 对应 F103 的 PA8，并接入 V3P 的 BL9342 EN。
 * BL9342 EN 为高电平使能：PA8 高电平建立 10 V 栅极驱动电源，低电平关闭。
 * 逻辑极性已由器件资料和原理图确认。当前低速空载开环按此接口契约使用；
 * 若发生停止无效或异常转动，再回溯失能响应和功率级外部行为。
 */
#define BOARD_CONFIG_MOTOR_EN_GPIO_PORT     GPIOA
#define BOARD_CONFIG_MOTOR_EN_PIN           GPIO_PIN_8
#define BOARD_CONFIG_MOTOR_EN_ACTIVE_LEVEL  GPIO_PIN_SET
#define BOARD_CONFIG_MOTOR_EN_SAFE_LEVEL    GPIO_PIN_RESET
#define BOARD_CONFIG_MOTOR_EN_POLARITY_VERIFIED      (1U)

/*
 * V3P M0 使用两路低侧分流电流采样：0.01 Ohm 分流电阻和 INA240A2。
 * 比例、零偏、方向、量程及饱和阈值均未标定，因此原始 ADC 值不能直接
 * 当作安培值或保护依据。
 */
#define BOARD_CONFIG_PHASE_CURRENT_SENSE_CHANNELS   (2U)
#define BOARD_CONFIG_PHASE_CURRENT_SHUNT_OHMS        (0.01F)
#define BOARD_CONFIG_PHASE_CURRENT_CALIBRATED        (0U)

/*
 * V3P 原理图存在 POWER_ADC 母线电压采样网络；当前 F103 转接板和 CubeMX
 * 没有把该网络接入 ADC。母线电压软件保护因此不可用。
 */
#define BOARD_CONFIG_POWER_ADC_PRESENT_ON_V3P        (1U)
#define BOARD_CONFIG_VBUS_ADC_AVAILABLE              (0U)

/* F103 当前用 ADC1 扫描 PB0/PB1 两个 rank，读取两相电流采样原始值。 */
ADC_HandleTypeDef *board_config_get_motor_adc(void);

/* MT6701 or equivalent I2C encoder is bound to I2C2: PB10/PB11. */
I2C_HandleTypeDef *board_config_get_encoder_i2c(void);

/* M0 三相 PWM 绑定到 TIM2 CH1/CH2/CH3：PA0/PA1/PA2。 */
TIM_HandleTypeDef *board_config_get_motor_pwm_timer(void);

/* 调试日志绑定到 USART1：PA9/PA10，通过 CH340 输出到上位机。 */
UART_HandleTypeDef *board_config_get_debug_uart(void);

/* 启动和异常路径优先调用：关闭使能、清零 PWM、保持 M1 未用通道为低。 */
void board_config_apply_safe_outputs(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CONFIG_H */
