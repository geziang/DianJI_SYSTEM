/*
 * FOC 坐标变换数学库。
 *
 * 所属层：基础数学库。
 * 实现三相静止坐标与 alpha/beta、d/q 坐标之间的变换，不访问硬件。
 */

#ifndef FOC_TRANSFORM_H
#define FOC_TRANSFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_types.h"

void foc_clarke_transform(const foc_three_phase_t *phase,
                          foc_alpha_beta_t *alpha_beta);
void foc_park_transform(const foc_alpha_beta_t *alpha_beta,
                        float electrical_angle_rad,
                        foc_dq_t *dq);
void foc_inverse_park_transform(const foc_dq_t *dq,
                                float electrical_angle_rad,
                                foc_alpha_beta_t *alpha_beta);

/* 同一电角度的 sin/cos 只算一次，供 Park 与反 Park 复用，降低快路径负载。 */
void foc_sin_cos(float electrical_angle_rad, float *sine, float *cosine);
void foc_park_transform_sc(const foc_alpha_beta_t *alpha_beta,
                           float sine,
                           float cosine,
                           foc_dq_t *dq);
void foc_inverse_park_transform_sc(const foc_dq_t *dq,
                                   float sine,
                                   float cosine,
                                   foc_alpha_beta_t *alpha_beta);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TRANSFORM_H */
