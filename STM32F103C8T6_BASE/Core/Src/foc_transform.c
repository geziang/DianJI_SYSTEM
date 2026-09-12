/*
 * FOC 坐标变换实现。
 *
 * 采用常见的幅值不变 Clarke/Park 约定：
 * alpha = Ia，beta = (Ia + 2*Ib) / sqrt(3)。
 * 具体电流正方向由 FOC-PRE 标定，不在这里修正。
 */

#include "foc_transform.h"
#include "foc_angle_math.h"
#include "foc_trig_table.h"

#define FOC_INV_SQRT3_F  (0.57735026918962576450f)
#define FOC_SQRT3_BY_2_F (0.86602540378443864676f)

void foc_clarke_transform(const foc_three_phase_t *phase,
                          foc_alpha_beta_t *alpha_beta)
{
  if ((phase == 0) || (alpha_beta == 0))
  {
    return;
  }

  alpha_beta->alpha = phase->phase_a;
  /* (a + 2b) 中的 2b 用加法替代一次 soft-float 乘法（数学等价）。 */
  alpha_beta->beta = (phase->phase_a + phase->phase_b + phase->phase_b) *
                     FOC_INV_SQRT3_F;
}

void foc_sin_cos(float electrical_angle_rad, float *sine, float *cosine)
{
  /* 计算重构：Q15 快表 + 线性插值替代 sinf/cosf（无 FPU，单次省约 1400 cycles）。 */
  foc_trig_sin_cos(electrical_angle_rad, sine, cosine);
}

void foc_park_transform_sc(const foc_alpha_beta_t *alpha_beta,
                           float sine,
                           float cosine,
                           foc_dq_t *dq)
{
  if ((alpha_beta == 0) || (dq == 0))
  {
    return;
  }

  dq->d = (alpha_beta->alpha * cosine) + (alpha_beta->beta * sine);
  dq->q = (-alpha_beta->alpha * sine) + (alpha_beta->beta * cosine);
}

void foc_park_transform(const foc_alpha_beta_t *alpha_beta,
                        float electrical_angle_rad,
                        foc_dq_t *dq)
{
  float sine;
  float cosine;

  foc_sin_cos(electrical_angle_rad, &sine, &cosine);
  foc_park_transform_sc(alpha_beta, sine, cosine, dq);
}

void foc_inverse_park_transform_sc(const foc_dq_t *dq,
                                   float sine,
                                   float cosine,
                                   foc_alpha_beta_t *alpha_beta)
{
  if ((dq == 0) || (alpha_beta == 0))
  {
    return;
  }

  alpha_beta->alpha = (dq->d * cosine) - (dq->q * sine);
  alpha_beta->beta = (dq->d * sine) + (dq->q * cosine);
}

void foc_inverse_park_transform(const foc_dq_t *dq,
                                float electrical_angle_rad,
                                foc_alpha_beta_t *alpha_beta)
{
  float sine;
  float cosine;

  foc_sin_cos(electrical_angle_rad, &sine, &cosine);
  foc_inverse_park_transform_sc(dq, sine, cosine, alpha_beta);
}
