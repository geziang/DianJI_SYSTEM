/*
 * FOC SVPWM 实现。
 *
 * 先通过反 Clarke 得到三相电压，再用最大/最小值注入公共模式，
 * 最后映射到 0..ARR。这里的 compare 只是计算结果，实际功率使能由上层负责。
 */

#include "foc_svpwm.h"

#define FOC_SQRT3_BY_2_F (0.86602540378443864676f)

static float foc_svpwm_max3(float a, float b, float c)
{
  float value = a;
  if (b > value) value = b;
  if (c > value) value = c;
  return value;
}

static float foc_svpwm_min3(float a, float b, float c)
{
  float value = a;
  if (b < value) value = b;
  if (c < value) value = c;
  return value;
}

static float foc_svpwm_clamp(float value)
{
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

foc_status_t foc_svpwm_calculate(const foc_alpha_beta_t *voltage_alpha_beta_v,
                                 float inv_bus_voltage_v,
                                 float pwm_period_f,
                                 foc_pwm_output_t *output)
{
  float phase_a_v;
  float phase_b_v;
  float phase_c_v;
  float common_mode_v;
  float duty_a;
  float duty_b;
  float duty_c;

  if ((voltage_alpha_beta_v == 0) || (output == 0))
  {
    return FOC_STATUS_INVALID_ARG;
  }
  if ((inv_bus_voltage_v <= 0.0f) || (pwm_period_f <= 0.0f))
  {
    output->phase_a_ticks = 0U;
    output->phase_b_ticks = 0U;
    output->phase_c_ticks = 0U;
    return FOC_STATUS_NOT_READY;
  }

  phase_a_v = voltage_alpha_beta_v->alpha;
  phase_b_v = (-0.5f * voltage_alpha_beta_v->alpha) +
              (FOC_SQRT3_BY_2_F * voltage_alpha_beta_v->beta);
  phase_c_v = (-0.5f * voltage_alpha_beta_v->alpha) -
              (FOC_SQRT3_BY_2_F * voltage_alpha_beta_v->beta);
  /* 零序注入（max+min)/2 不能删：删了会改变注入电压、污染标定。 */
  common_mode_v = 0.5f * (foc_svpwm_max3(phase_a_v, phase_b_v, phase_c_v) +
                          foc_svpwm_min3(phase_a_v, phase_b_v, phase_c_v));

  /* 除母线 -> 乘预算倒数；周期整数 -> 预算 float，快路径无除法/无 int->float。 */
  duty_a = foc_svpwm_clamp(0.5f + ((phase_a_v - common_mode_v) * inv_bus_voltage_v));
  duty_b = foc_svpwm_clamp(0.5f + ((phase_b_v - common_mode_v) * inv_bus_voltage_v));
  duty_c = foc_svpwm_clamp(0.5f + ((phase_c_v - common_mode_v) * inv_bus_voltage_v));

  output->phase_a_ticks = (uint32_t)(duty_a * pwm_period_f);
  output->phase_b_ticks = (uint32_t)(duty_b * pwm_period_f);
  output->phase_c_ticks = (uint32_t)(duty_c * pwm_period_f);
  return FOC_STATUS_OK;
}
