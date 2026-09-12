/*
 * FOC 转子角度系统数学库。
 *
 * 所属层：面向系统的数学中间库。
 * 输入 MT6701 已读取的原始角度，输出机械角和电角；不执行 I2C 通信。
 */

#ifndef FOC_ROTOR_MODEL_H
#define FOC_ROTOR_MODEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_types.h"

foc_status_t foc_rotor_model_convert(const foc_rotor_config_t *config,
                                     uint16_t mechanical_raw,
                                     foc_rotor_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* FOC_ROTOR_MODEL_H */
