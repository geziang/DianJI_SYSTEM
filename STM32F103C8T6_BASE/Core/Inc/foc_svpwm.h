/*
 * FOC SVPWM 数学库。
 *
 * 所属层：基础数学库。
 * 输入是 alpha/beta 电压和母线电压，输出是三相 compare；不会启动定时器。
 */

#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "foc_types.h"

/* 计算重构：为消除 10kHz 快路径里的浮点除法与 int->float 转换，这里不再直接
 * 收母线电压和整数周期，而收装载期预算好的量：
 *   inv_bus_voltage_v = 1/bus_voltage_v，pwm_period_f = (float)pwm_period_ticks。
 * 由 foc_parameter_set_precompute 统一预算。 */
foc_status_t foc_svpwm_calculate(const foc_alpha_beta_t *voltage_alpha_beta_v,
                                 float inv_bus_voltage_v,
                                 float pwm_period_f,
                                 foc_pwm_output_t *output);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SVPWM_H */
