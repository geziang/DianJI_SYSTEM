/*
 * FOC 基础类型定义。
 *
 * 所属层：FOC 公共接口层。
 * 本文件只定义单位、数据结构和状态码，不访问 HAL、ADC、I2C 或 PWM。
 * 基础数学库和面向系统的 FOC 中间库都通过这些类型交换数据。
 */

#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  FOC_STATUS_OK = 0,
  FOC_STATUS_INVALID_ARG,
  FOC_STATUS_NOT_READY,
  FOC_STATUS_OUT_OF_RANGE,
  FOC_STATUS_INVALID_SAMPLE
} foc_status_t;

/* 三相量，单位由调用模块定义；电流使用 A，电压使用 V。 */
typedef struct
{
  float phase_a;
  float phase_b;
  float phase_c;
} foc_three_phase_t;

/* 静止坐标系 alpha/beta 量。 */
typedef struct
{
  float alpha;
  float beta;
} foc_alpha_beta_t;

/* 转子坐标系 d/q 量。 */
typedef struct
{
  float d;
  float q;
} foc_dq_t;

/* PWM compare 结果，单位是对应定时器的 timer ticks。 */
typedef struct
{
  uint32_t phase_a_ticks;
  uint32_t phase_b_ticks;
  uint32_t phase_c_ticks;
} foc_pwm_output_t;

/* 两通道原始 ADC 输入（通道 U/V）。这是【通道】语义，由硬件接线决定、
 * 与 phase_map 无关；逻辑相电流由电流模型按 phase_map 路由重构。
 * 命名必须用 channel 而非 phase——曾因 phase 命名掩盖"测量/输出映射
 * 不对称"bug（S7 闭环即失控），见 20260912 调试日志。 */
typedef struct
{
  uint16_t channel_u_raw;
  uint16_t channel_v_raw;
} foc_raw_current_sample_t;

/* 电流模型输出，单位为 A。 */
typedef struct
{
  foc_three_phase_t phase_current_a;
  uint8_t valid;
} foc_current_sample_t;

/* 编码器和电角度模型输出；角度单位为 rad。 */
typedef struct
{
  uint16_t mechanical_raw;
  float mechanical_angle_rad;
  float electrical_angle_rad;
  uint8_t valid;
} foc_rotor_sample_t;

/* PI 参数；积分和输出上下限单位与对应控制量一致。
 * ki_times_t = ki * 控制周期 T，是 ISR 计算重构的预算量：由参数装载阶段
 * (foc_parameter_set_precompute) 一次性算好，10kHz 快路径里不再每拍做 ki*T。
 * 若在线修改 ki 或控制周期，必须重新预算本字段。 */
typedef struct
{
  float kp;
  float ki;
  float ki_times_t;
  float integrator_min;
  float integrator_max;
  float output_min;
  float output_max;
} foc_pi_config_t;

/* PI 运行时状态；不包含硬件状态。 */
typedef struct
{
  foc_pi_config_t config;
  float integrator;
  uint8_t initialized;
} foc_pi_state_t;

/* 控制模式：
 * CURRENT  = Id/Iq 电流闭环，PI 由电流误差产生 Vd/Vq；
 * VOLTAGE  = 开环电压注入，跳过 PI 直接采用给定 Vd/Vq（仍做电压限幅）。
 * VOLTAGE 用于电流符号/相序尚未验证前的静态注入，避免用未验证方向建闭环（正反馈）。 */
typedef enum
{
  FOC_CONTROL_CURRENT = 0,
  FOC_CONTROL_VOLTAGE
} foc_control_mode_t;

/* FOC 单步计算输入，电流单位 A、角度单位 rad、周期单位 s。 */
typedef struct
{
  foc_three_phase_t phase_current_a;
  float electrical_angle_rad;
  foc_dq_t current_target_a;
  foc_dq_t voltage_command_v;   /* VOLTAGE 模式下的直接电压给定 V */
  foc_control_mode_t mode;      /* CURRENT 闭环 / VOLTAGE 开环 */
  float control_period_s;
} foc_control_input_t;

/* FOC 单步计算输出；还没有写入实际 PWM 外设。 */
typedef struct
{
  foc_dq_t measured_current_a;
  foc_three_phase_t phase_current_a; /* 回填三相电流 A，供标定采样窗口采集 */
  foc_dq_t voltage_command_v;
  foc_alpha_beta_t voltage_alpha_beta_v;
  foc_pwm_output_t pwm;
  foc_status_t status;
} foc_control_output_t;

/* 编码器到电角度的系统参数。方向只能使用 +1 或 -1。 */
typedef struct
{
  uint8_t pole_pairs;
  int8_t encoder_direction;
  float electrical_offset_rad;
} foc_rotor_config_t;

/*
 * 软件相序映射。值 0/1/2 表示控制器逻辑相 A/B/C 对应的硬件通道号。
 * 【契约】必须对称应用在两个适配点：PWM 输出（foc_runtime 把逻辑相
 * duty tick 重排到硬件通道）与 ADC 电流输入（foc_current_model 把
 * 通道电流按映射路由回逻辑相）。只应用一侧 = 电流环反馈错位。
 * 校验：三个值互异且 <=2（motor_parameter_core 装载时校验）。
 */
typedef struct
{
  uint8_t phase_a_output;
  uint8_t phase_b_output;
  uint8_t phase_c_output;
} foc_phase_map_t;

/* ADC 通道到电流的系统标定参数。calibrated=0 时模型拒绝输出有效电流。
 * code_to_amp 是 ISR 计算重构的预算量：= ref/fullscale/(shunt*gain)，
 * 单位 A/code，由 foc_parameter_set_precompute 一次算好，快路径只做
 * (raw-zero)*code_to_amp。注意它【不含 sign】——通道符号要晚到 S1a/PROBE3
 * 结果处才翻转，预算在此之前完成，故符号在快路径里用条件取反单独施加。
 * 零偏/符号均为【通道】属性（S0/S1a 绑定通道 U/V，与逻辑相无关）；
 * 通道 -> 逻辑相的对应由 phase_map 在 foc_current_model_convert 里路由。 */
typedef struct
{
  float adc_reference_v;
  float adc_full_scale_raw;
  float shunt_resistance_ohm;
  float amplifier_gain;
  float code_to_amp;
  float channel_u_zero_raw;
  float channel_v_zero_raw;
  float channel_u_sign;
  float channel_v_sign;
  uint8_t calibrated;
} foc_current_calibration_t;

#ifdef __cplusplus
}
#endif

#endif /* FOC_TYPES_H */
