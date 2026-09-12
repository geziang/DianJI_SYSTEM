/*
 * FOC 参数预算（计算重构）。
 *
 * 所属层：面向系统的 FOC 数学中间库（纯函数，不访问 HAL/ADC/PWM，PC gcc 可测）。
 *
 * 作用：把"一次测试/一次配置内不变、却原本散落在 10kHz ISR 每拍现算"的派生量
 * 集中在参数装载阶段算一次，写回参数集，快路径只读不算：
 *   - current.code_to_amp = ref/fullscale/(shunt*gain)   （A/code，不含 sign）
 *   - inv_bus_voltage_v   = 1/bus                         （除法变乘倒数）
 *   - pwm_period_f        = (float)pwm_period_ticks       （消除 int->float）
 *   - id/iq_pi.ki_times_t = ki * loop_period_s            （PI 积分步长预算）
 *
 * 预算前提：这些量在一次运行内不变。若将来 bus / ki / 周期在线可调，
 * 调整后必须重新调用本函数。
 */

#ifndef FOC_PRECOMPUTE_H
#define FOC_PRECOMPUTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_params.h"

/*
 * 就地预算参数集中的全部派生量。
 * loop_period_s：电流环控制周期（s），由调用方从板级配置传入，本层不依赖板级头。
 * 返回 FOC_STATUS_OK 表示全部派生量有效；参数非法时返回 FOC_STATUS_NOT_READY，
 * 且不写入任何预算字段（保留原值，避免快路径用到半成品）。
 */
foc_status_t foc_parameter_set_precompute(foc_parameter_set_t *parameters,
                                          float loop_period_s);

#ifdef __cplusplus
}
#endif

#endif /* FOC_PRECOMPUTE_H */
