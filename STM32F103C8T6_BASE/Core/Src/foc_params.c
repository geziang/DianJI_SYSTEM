/*
 * FOC 默认参数。
 *
 * 所属层：面向系统的 FOC 参数库。
 * 这里的默认值只用于构造数据结构和离线计算，不能直接作为功率闭环的整定结果。
 */

#include "foc_params.h"
#include "board_config.h"

void foc_parameter_set_load_default(foc_parameter_set_t *params)
{
  if (params == 0)
  {
    return;
  }

  params->rotor.pole_pairs = 7U;
  params->rotor.encoder_direction = 1;
  params->rotor.electrical_offset_rad = 0.0f;

  /* 默认映射仅表示软件通道顺序，实际电机线序需由 FOC-PRE 实验确认。 */
  params->phase_map.phase_a_output = 0U;
  params->phase_map.phase_b_output = 1U;
  params->phase_map.phase_c_output = 2U;

  /* 原理图候选值已知，但比例、方向和采样时序尚未通过实测。 */
  params->current.adc_reference_v = BOARD_CONFIG_ADC_REFERENCE_V_NOMINAL;
  params->current.adc_full_scale_raw = 4095.0f;
  params->current.shunt_resistance_ohm = BOARD_CONFIG_PHASE_CURRENT_SHUNT_OHMS;
  params->current.amplifier_gain = BOARD_CONFIG_PHASE_CURRENT_AMPLIFIER_GAIN;
  params->current.channel_u_zero_raw = 0.0f;
  params->current.channel_v_zero_raw = 0.0f;
  params->current.channel_u_sign = 1.0f;
  params->current.channel_v_sign = 1.0f;
  params->current.calibrated = 0U;

  /* PI 默认关闭输出，避免未整定参数被误用于功率闭环。 */
  params->id_pi.kp = 0.0f;
  params->id_pi.ki = 0.0f;
  params->id_pi.integrator_min = -1.0f;
  params->id_pi.integrator_max = 1.0f;
  params->id_pi.output_min = -1.0f;
  params->id_pi.output_max = 1.0f;
  params->iq_pi = params->id_pi;

  /* 当前 V3P 只允许不超过 10 V；默认值只是软件模型边界。 */
  params->bus_voltage_v = 0.0f;
  params->max_voltage_v = 0.0f;
  params->max_current_a = 0.0f;
  params->pwm_period_ticks = BOARD_CONFIG_PWM_PERIOD_TICKS;
}
