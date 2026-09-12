/*
 * FOC 参数预算实现（计算重构）。
 *
 * 纯数学：只根据参数集中的物理量计算派生预算量，不访问任何硬件，
 * 因此可在 PC 端用 gcc 直接编译进单元测试。
 */

#include "foc_precompute.h"

foc_status_t foc_parameter_set_precompute(foc_parameter_set_t *parameters,
                                          float loop_period_s)
{
  float scale_product;

  if (parameters == 0)
  {
    return FOC_STATUS_INVALID_ARG;
  }

  /* 与 foc_runtime_configure / foc_controller_step 的就绪门禁保持同一口径，
   * 任一基础量非法都拒绝预算，避免快路径读到未初始化的倒数/步长。 */
  if ((parameters->current.adc_reference_v <= 0.0f) ||
      (parameters->current.adc_full_scale_raw <= 0.0f) ||
      (parameters->current.shunt_resistance_ohm <= 0.0f) ||
      (parameters->current.amplifier_gain <= 0.0f) ||
      (parameters->bus_voltage_v <= 0.0f) ||
      (parameters->pwm_period_ticks == 0U) ||
      (loop_period_s <= 0.0f))
  {
    return FOC_STATUS_NOT_READY;
  }

  /* ADC code -> A：(raw-zero) 先乘本系数，再按 sign 条件取反。
   * 刻意不含 sign（符号在 S1a/PROBE3 之后才确定，晚于本次预算）。 */
  scale_product = parameters->current.shunt_resistance_ohm *
                  parameters->current.amplifier_gain;
  if (scale_product <= 0.0f)
  {
    return FOC_STATUS_NOT_READY;
  }
  parameters->current.code_to_amp =
      (parameters->current.adc_reference_v /
       parameters->current.adc_full_scale_raw) / scale_product;

  /* SVPWM：除母线变乘倒数；周期 ticks 预算成 float。 */
  parameters->inv_bus_voltage_v = 1.0f / parameters->bus_voltage_v;
  parameters->pwm_period_f = (float)parameters->pwm_period_ticks;

  /* PI 积分步长 ki*T 预算，快路径只做一次乘法。 */
  parameters->id_pi.ki_times_t = parameters->id_pi.ki * loop_period_s;
  parameters->iq_pi.ki_times_t = parameters->iq_pi.ki * loop_period_s;

  return FOC_STATUS_OK;
}
