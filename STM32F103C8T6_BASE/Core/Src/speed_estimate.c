#include "speed_estimate.h"

#include "foc_angle_math.h"

#include <math.h>

/* 单拍位移合理上限：超过判为野样本（I2C 位错误级跳变，MSB 翻转=π、
 * 次高位=π/2）。真实机械上限 ~0.3rad@5ms 拍（550rpm@10V 母线），1.0rad
 * 留足裕量不误伤真实运动（≈1900rpm 等效）。野样本整体丢弃、基准保持
 * 上一好样本，两侧差分均不受污染（2026-09-15 上板教训：链路无连续失败
 * 但偶发单点野值仍会经滑窗尖峰触发 overspeed 误跳）。 */
#define SPEED_EST_MAX_DELTA_RAD (1.0f)

void speed_estimator_init(speed_estimator_t *est)
{
  uint8_t i;

  if (est == (speed_estimator_t *)0)
  {
    return;
  }
  for (i = 0U; i < SPEED_EST_WINDOW_N; i++)
  {
    est->delta_rad[i] = 0.0f;
  }
  est->idx = 0U;
  est->count = 0U;
  est->last_angle_rad = 0.0f;
  est->have_last = 0U;
}

void speed_estimator_update(speed_estimator_t *est, float angle_rad, float dt_s)
{
  float delta;

  if ((est == (speed_estimator_t *)0) || (dt_s <= 0.0f))
  {
    return;
  }
  if (est->have_last != 0U)
  {
    /* 圆周回绕差分：跨 2π 时取最短路径，正反转都正确。 */
    delta = foc_angle_wrap_signed_rad(angle_rad - est->last_angle_rad);
    if (fabsf(delta) > SPEED_EST_MAX_DELTA_RAD)
    {
      /* 野样本：丢弃本拍，基准保持上一好样本（下一拍从好样本重新起算）。 */
      return;
    }
    est->delta_rad[est->idx] = delta;
    est->idx = (uint8_t)((est->idx + 1U) % SPEED_EST_WINDOW_N);
    if (est->count < SPEED_EST_WINDOW_N)
    {
      est->count++;
    }
  }
  est->last_angle_rad = angle_rad;
  est->have_last = 1U;
}

float speed_estimator_get_rad_s(const speed_estimator_t *est, float dt_s)
{
  float sum;
  uint8_t i;
  uint8_t n;

  if ((est == (speed_estimator_t *)0) || (dt_s <= 0.0f) || (est->count == 0U))
  {
    return 0.0f;
  }
  sum = 0.0f;
  n = SPEED_EST_WINDOW_N;
  if (est->count < n)
  {
    n = est->count;
  }
  for (i = 0U; i < n; i++)
  {
    sum += est->delta_rad[i];
  }
  return sum / ((float)n * dt_s);
}

uint8_t speed_estimator_valid(const speed_estimator_t *est)
{
  if (est == (speed_estimator_t *)0)
  {
    return 0U;
  }
  return (est->count >= SPEED_EST_WINDOW_N) ? 1U : 0U;
}
