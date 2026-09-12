/*
 * FOC 电流系统数学库。
 *
 * 所属层：面向系统的数学中间库。
 * 将两通道 ADC 原始码转换为逻辑相 Ia/Ib/Ic：先按通道各自的零偏/符号
 * 求出通道电流（U/V 及重构第三通道），再按 phase_map 把通道电流路由到
 * 逻辑相——与 foc_runtime 的 PWM 输出侧重排对称，phase_map=NULL 或非法
 * 时回退恒等映射（行为与历史一致）。不负责启动 ADC，也不执行零偏辨识。
 */

#ifndef FOC_CURRENT_MODEL_H
#define FOC_CURRENT_MODEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_types.h"

foc_status_t foc_current_model_convert(const foc_current_calibration_t *calibration,
                                       const foc_raw_current_sample_t *raw,
                                       const foc_phase_map_t *phase_map,
                                       foc_current_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CURRENT_MODEL_H */
