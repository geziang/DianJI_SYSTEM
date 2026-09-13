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

/*
 * 时间基准单一事实源：TIM2/TIM3 运行在 72 MHz。电流环/开关频率暂由 20 kHz
 * 降为 10 kHz——无 FPU 的 M3 跑软浮点 FOC，单周期预算由 3600 提升到 7200 cycles。
 * ARR = 72e6/10e3 - 1 = 7199，且由频率自动推导，改频率只改这一个宏。
 * 电流环周期、电感 dI/dt 时间基准一律引用 BOARD_CONFIG_CURRENT_LOOP_PERIOD_S，
 * 禁止再散落 1/频率 字面量。
 * 注意：main.c 中 htim2/htim3 的 Init.Period 是 CubeMX 生成段，必须与此手工对齐。
 */
#define BOARD_CONFIG_TIMER_CLOCK_HZ          (72000000UL)
#define BOARD_CONFIG_PWM_FREQUENCY_HZ        (10000UL)
#define BOARD_CONFIG_CURRENT_LOOP_PERIOD_S  (1.0F / (float)BOARD_CONFIG_PWM_FREQUENCY_HZ)
#define BOARD_CONFIG_PWM_PERIOD_TICKS       ((uint32_t)(BOARD_CONFIG_TIMER_CLOCK_HZ / BOARD_CONFIG_PWM_FREQUENCY_HZ - 1UL))
#define BOARD_CONFIG_MAX_BUS_VOLTAGE_V       (10.0F)

/* Long-lived calibration is bound to this hardware, motor and phase wiring revision. */
#define BOARD_CONFIG_HARDWARE_ID             (0x56335001UL)
#define BOARD_CONFIG_MOTOR_ID                (0x32383034UL)
#define BOARD_CONFIG_WIRING_REVISION         (1UL)

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
 * V3P M0 使用两路低侧分流电流采样，已对 V3Pro4.0 原理图与 TI datasheet 确认：
 * 分流电阻 R15/R16 = 0.01Ω，放大芯片 INA240A2PWR 固定增益 50 V/V（增益误差
 * ≤0.20%、零漂移、双向、增强 PWM 抑制），即 0.5 V/A。故增益为板级设计常量，
 * 1 个 ADC code = 3.3/4095/(0.01*50) ≈ 1.612mA，不再按"候选"对待。
 * 仍需实测的只有：运行零偏(S0)、电流符号/通道(S1a)；外部电流表复核为可选项。
 * CALIBRATED=0 仅表示运行零偏尚未在本次上电完成（由 S0 置 calibrated），不代表增益未确认。
 */
#define BOARD_CONFIG_PHASE_CURRENT_SENSE_CHANNELS   (2U)
#define BOARD_CONFIG_PHASE_CURRENT_SHUNT_OHMS        (0.01F)
#define BOARD_CONFIG_PHASE_CURRENT_AMPLIFIER_GAIN    (50.0F)
#define BOARD_CONFIG_PHASE_CURRENT_VOLTS_PER_AMP     (0.5F)
#define BOARD_CONFIG_ADC_REFERENCE_V_NOMINAL          (3.3F)
#define BOARD_CONFIG_ADC_FULL_SCALE_RAW               (4095U)
#define BOARD_CONFIG_PHASE_CURRENT_CALIBRATED        (0U)
#define BOARD_CONFIG_PWM_ADC_TRIGGER_CONFIGURED      (1U)

/*
 * ADC 触发时刻相对 TIM2 PWM 的相位偏移（TIM3 计数预置 ticks @72MHz，
 * ARR=7199 对应一个 100us PWM 周期）。0 = 保持现状。
 * L4 判决若指向采样时刻（VJ 静止注入干净 / 旋转扫描脏），用此宏扫触发
 * 相位窗（如 1800=1/4 周期、3600=半周期，或 ±1000/±3000 ≈ ±14/±42us），
 * 每次改值重烧，对比 [VJT]/[S7DBG] 数据随相位的变化。
 */
#define BOARD_CONFIG_ADC_TRIGGER_OFFSET_TICKS        (0U)

/*
 * V3P 原理图存在 POWER_ADC 母线电压采样网络；当前 F103 转接板和 CubeMX
 * 没有把该网络接入 ADC。母线电压软件保护因此不可用。
 */
#define BOARD_CONFIG_POWER_ADC_PRESENT_ON_V3P        (1U)
#define BOARD_CONFIG_VBUS_ADC_AVAILABLE              (0U)

/*
 * Flash 参数页布局（SPEC-RUN FR-2.1：64 KB 保守设计、末两页双页 A/B）。
 * F103C8T6 一律按 64 KB/1 KB 每页设计，不假设拆机片为 128 KB：
 *   代码/只读区 0x08000000 - 0x0800F7FF（62 KB，scatter 同步收缩为 0xF800）
 *   参数页 A   0x0800F800（1 KB，与 B 交替活跃）
 *   参数页 B   0x0800FC00（1 KB）
 * 擦写只允许在 SAFE_IDLE、驱动失能、控制环路停止后进行（FR-2.2）。
 */
#define BOARD_CONFIG_FLASH_TOTAL_SIZE        (0x00010000UL)
#define BOARD_CONFIG_FLASH_PAGE_SIZE         (0x400U)
#define BOARD_CONFIG_FLASH_PAGE_A_ADDR       (0x0800F800UL)
#define BOARD_CONFIG_FLASH_PAGE_B_ADDR       (0x0800FC00UL)
#define BOARD_CONFIG_CODE_REGION_END         (0x0800F800UL)

/* F103 当前用 ADC1 扫描 PB0/PB1 两个 rank，读取两相电流采样原始值。 */
ADC_HandleTypeDef *board_config_get_motor_adc(void);

/* MT6701 or equivalent I2C encoder is bound to I2C2: PB10/PB11. */
I2C_HandleTypeDef *board_config_get_encoder_i2c(void);

/* M0 三相 PWM 绑定到 TIM2 CH1/CH2/CH3：PA0/PA1/PA2。 */
TIM_HandleTypeDef *board_config_get_motor_pwm_timer(void);

/* TIM3 不输出到引脚，只提供 ADC1 触发原型；相对 PWM 相位尚未验证，不放行功率。 */
TIM_HandleTypeDef *board_config_get_current_sample_timer(void);

/* 调试日志绑定到 USART1：PA9/PA10，通过 CH340 输出到上位机。 */
UART_HandleTypeDef *board_config_get_debug_uart(void);

/* 启动和异常路径优先调用：关闭使能、清零 PWM、保持 M1 未用通道为低。 */
void board_config_apply_safe_outputs(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CONFIG_H */
