/*
 * FOC 角度基础数学库。
 *
 * 所属层：基础数学库。
 * 只处理角度单位和机械角/电角换算，不读取 MT6701，不保存校准参数。
 */

#ifndef FOC_ANGLE_MATH_H
#define FOC_ANGLE_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FOC_PI_F       (3.14159265358979323846f)
#define FOC_TWO_PI_F   (6.28318530717958647692f)

float foc_angle_wrap_rad(float angle_rad);
float foc_angle_wrap_signed_rad(float angle_rad);
float foc_mechanical_raw_to_rad(uint16_t raw_angle);
float foc_mechanical_to_electrical_rad(float mechanical_angle_rad,
                                       uint8_t pole_pairs,
                                       int8_t direction,
                                       float electrical_offset_rad);

#ifdef __cplusplus
}
#endif

#endif /* FOC_ANGLE_MATH_H */
