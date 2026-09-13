#include "speed_estimate.h"

#include "foc_angle_math.h"

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
