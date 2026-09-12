/*
 * FOC 参数集合。
 *
 * 所属层：面向系统的 FOC 数学中间库。
 * 本文件集中保存模型和控制器所需参数，但不执行参数辨识，也不访问硬件。
 * 真实相序、零位、采样比例和 PI 参数必须由后续 FOC-PRE 实验写入。
 */

#ifndef FOC_PARAMS_H
#define FOC_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_types.h"

typedef struct
{
  foc_rotor_config_t rotor;
  foc_phase_map_t phase_map;
  foc_current_calibration_t current;
  foc_pi_config_t id_pi;
  foc_pi_config_t iq_pi;
  float bus_voltage_v;
  float max_voltage_v;
  float max_current_a;
  uint32_t pwm_period_ticks;
  /* ISR 计算重构预算量（由 foc_parameter_set_precompute 一次算好，快路径只读）：
   * inv_bus_voltage_v = 1/bus，把 SVPWM 每拍 3 次除法变乘倒数；
   * pwm_period_f = (float)pwm_period_ticks，消除每拍 int->float 转换。 */
  float inv_bus_voltage_v;
  float pwm_period_f;
} foc_parameter_set_t;

/* 装载安全的未标定默认参数；不会宣称当前硬件已经整定。 */
void foc_parameter_set_load_default(foc_parameter_set_t *params);

#ifdef __cplusplus
}
#endif

#endif /* FOC_PARAMS_H */
