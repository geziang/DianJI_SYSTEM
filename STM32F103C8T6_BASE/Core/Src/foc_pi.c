/*
 * FOC PI 控制器实现。
 *
 * 采用显式积分限幅和输出限幅。停止、故障或重新进入对齐时，
 * 上层逻辑必须调用 foc_pi_reset() 清除历史积分量。
 */

#include "foc_pi.h"

static float foc_pi_clamp(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

void foc_pi_init(foc_pi_state_t *state, const foc_pi_config_t *config)
{
  if ((state == 0) || (config == 0))
  {
    return;
  }

  state->config = *config;
  state->integrator = 0.0f;
  state->initialized = 1U;
}

void foc_pi_reset(foc_pi_state_t *state)
{
  if (state == 0)
  {
    return;
  }

  state->integrator = 0.0f;
}

float foc_pi_update(foc_pi_state_t *state, float error)
{
  float proportional;
  float next_integrator;
  float output;

  if ((state == 0) || (state->initialized == 0U))
  {
    return 0.0f;
  }

  proportional = state->config.kp * error;
  /* ki*T 已在装载期预算为 ki_times_t，快路径只做一次乘法。 */
  next_integrator = state->integrator + (state->config.ki_times_t * error);
  next_integrator = foc_pi_clamp(next_integrator,
                                 state->config.integrator_min,
                                 state->config.integrator_max);
  output = proportional + next_integrator;
  /* Back-calculation anti-windup：输出超出限幅时，按 (u_sat-u_unsat) 把积分
   * 主动回拉到"可输出区"，比单纯条件冻结退出饱和更快、不会在限值上钉死。
   * 回拉系数 Kb*T = (ki*T)/kp = T/Ti（Ti=kp/ki 为积分时间常数），仅用现有
   * 预算量推导，不改配置结构；kp<=0（未整定）时退化为纯输出限幅。 */
  if (((output > state->config.output_max) || (output < state->config.output_min)) &&
      (state->config.kp > 1e-6f))
  {
    float saturated_output = foc_pi_clamp(output,
                                          state->config.output_min,
                                          state->config.output_max);
    float back_gain_t = state->config.ki_times_t / state->config.kp;
    next_integrator += back_gain_t * (saturated_output - output);
    next_integrator = foc_pi_clamp(next_integrator,
                                   state->config.integrator_min,
                                   state->config.integrator_max);
  }
  state->integrator = next_integrator;
  output = proportional + state->integrator;
  output = foc_pi_clamp(output, state->config.output_min, state->config.output_max);

  return output;
}
