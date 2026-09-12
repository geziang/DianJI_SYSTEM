/*
 * 相电阻 R 自学习实现。纯函数、零 HAL 依赖，PC 与 MCU 同一份编译（决策 D6）。
 */
#include "resistance_learning.h"

#include <stddef.h>

static float resistance_learning_absf(float v)
{
  return (v < 0.0f) ? -v : v;
}

float resistance_learning_median3(float a, float b, float c)
{
  if (((a >= b) && (a <= c)) || ((a >= c) && (a <= b)))
  {
    return a;
  }
  if (((b >= a) && (b <= c)) || ((b >= c) && (b <= a)))
  {
    return b;
  }
  return c;
}

float resistance_learning_mad_ratio3(float median, float a, float b, float c)
{
  float mad;

  if (median <= 0.0f)
  {
    return 1.0e9f; /* 中值非法（0/负）视为完全不可信 */
  }
  mad = resistance_learning_median3(resistance_learning_absf(a - median),
                                    resistance_learning_absf(b - median),
                                    resistance_learning_absf(c - median));
  return mad / median;
}

void resistance_learning_load(resistance_learning_state_t *state,
                              uint8_t history_count,
                              const float *history_r_ohm,
                              uint8_t learned_valid,
                              float learned_r_ohm,
                              uint32_t learned_revision)
{
  uint8_t i;

  if (state == NULL)
  {
    return;
  }
  state->history_count = (history_count > 3U) ? 3U : history_count;
  if (history_r_ohm != NULL)
  {
    for (i = 0U; i < state->history_count; i++)
    {
      state->history_r_ohm[i] = history_r_ohm[i];
    }
  }
  else
  {
    state->history_count = 0U;
  }
  state->learned_valid = (learned_valid != 0U) ? 1U : 0U;
  state->learned_r_ohm = learned_r_ohm;
  state->learned_revision = learned_revision;
}

void resistance_learning_store(const resistance_learning_state_t *state,
                               uint8_t *history_count,
                               float *history_r_ohm,
                               uint8_t *learned_valid,
                               float *learned_r_ohm,
                               uint32_t *learned_revision)
{
  uint8_t i;

  if ((state == NULL) || (history_count == NULL) || (history_r_ohm == NULL) ||
      (learned_valid == NULL) || (learned_r_ohm == NULL) ||
      (learned_revision == NULL))
  {
    return;
  }
  *history_count = state->history_count;
  for (i = 0U; i < 3U; i++)
  {
    history_r_ohm[i] = (i < state->history_count) ? state->history_r_ohm[i] : 0.0f;
  }
  *learned_valid = state->learned_valid;
  *learned_r_ohm = state->learned_r_ohm;
  *learned_revision = state->learned_revision;
}

uint8_t resistance_learning_update(const resistance_learning_cfg_t *cfg,
                                   resistance_learning_state_t *state,
                                   float s2_r_ohm,
                                   uint8_t s2_trusted,
                                   float s7_dyn_r_ohm,
                                   uint8_t s7_valid)
{
  uint8_t flags = 0U;
  uint8_t depth;
  uint8_t i;
  float candidate;
  float median;
  float mad_ratio;

  if ((cfg == NULL) || (state == NULL) || (cfg->history_n == 0U) ||
      (cfg->history_n > 3U) || (cfg->nominal_r_ohm <= 0.0f))
  {
    return 0U;
  }
  depth = cfg->history_n;

  /* 第 1 级：单次可信门（caller 已判 r2/|Voff|/样本数）。 */
  if ((s2_trusted == 0U) || (s2_r_ohm <= 0.0f))
  {
    return 0U;
  }

  /* 纳入历史：滑窗右移，保留最近 depth 次可信 S2 R。 */
  if (state->history_count >= depth)
  {
    for (i = 1U; i < depth; i++)
    {
      state->history_r_ohm[i - 1U] = state->history_r_ohm[i];
    }
    state->history_count = (uint8_t)(depth - 1U);
  }
  state->history_r_ohm[state->history_count] = s2_r_ohm;
  state->history_count = (uint8_t)(state->history_count + 1U);
  flags |= RESISTANCE_LEARN_F_ADOPTED;

  if (state->history_count < depth)
  {
    /* 样本还不足：历史照攒，不形成 learned_R。 */
    flags |= RESISTANCE_LEARN_F_REPEAT_FAIL;
    return flags;
  }

  /* 第 2 级：跨启动重复门（中值 + MAD/中值）。 */
  candidate = resistance_learning_median3(state->history_r_ohm[0],
                                          state->history_r_ohm[1],
                                          state->history_r_ohm[2]);
  median = candidate;
  mad_ratio = resistance_learning_mad_ratio3(median,
                                             state->history_r_ohm[0],
                                             state->history_r_ohm[1],
                                             state->history_r_ohm[2]);
  if (mad_ratio > cfg->repeat_dev_max)
  {
    flags |= RESISTANCE_LEARN_F_REPEAT_FAIL;
    return flags;
  }

  /* 第 3 级：双源交叉门——S7 动态 R 与本次 S2 实测 R 比对（FR-1.5）。
   * 无 S7 结果时跳过：历史仍采纳，learned_R 本轮不更新。 */
  if (s7_valid != 0U)
  {
    if ((s7_dyn_r_ohm <= 0.0f) ||
        (resistance_learning_absf(s7_dyn_r_ohm - s2_r_ohm) >
         (cfg->cross_dev_max * s2_r_ohm)))
    {
      flags |= RESISTANCE_LEARN_F_CROSS_FAIL;
      return flags;
    }
  }
  else
  {
    flags |= RESISTANCE_LEARN_F_CROSS_FAIL;
    return flags;
  }

  /* 第 4 级：失控护栏——learned_R 相对编译期兜底标称只允许对称 guard_ratio 倍。
   * 越界只记录、不采纳，要求人工介入（c=clear 清学习值回兜底）。 */
  if ((candidate > (cfg->nominal_r_ohm * cfg->guard_ratio)) ||
      (candidate < (cfg->nominal_r_ohm / cfg->guard_ratio)))
  {
    flags |= RESISTANCE_LEARN_F_GUARD_FAIL;
    return flags;
  }

  state->learned_r_ohm = candidate;
  state->learned_valid = 1U;
  state->learned_revision = state->learned_revision + 1UL;
  flags |= RESISTANCE_LEARN_F_LEARNED;
  return flags;
}

float resistance_learning_effective_r(const resistance_learning_cfg_t *cfg,
                                      const resistance_learning_state_t *state,
                                      float fallback_r_ohm)
{
  if ((cfg == NULL) || (state == NULL) || (state->learned_valid == 0U))
  {
    return fallback_r_ohm;
  }
  /* 生效前再过一次护栏：历史包被误写/漂移时兜底标称仍守住边界。 */
  if ((state->learned_r_ohm > (cfg->nominal_r_ohm * cfg->guard_ratio)) ||
      (state->learned_r_ohm < (cfg->nominal_r_ohm / cfg->guard_ratio)) ||
      (state->learned_r_ohm <= 0.0f))
  {
    return fallback_r_ohm;
  }
  return state->learned_r_ohm;
}

void resistance_learning_reset(resistance_learning_state_t *state)
{
  uint8_t i;

  if (state == NULL)
  {
    return;
  }
  state->history_count = 0U;
  for (i = 0U; i < 3U; i++)
  {
    state->history_r_ohm[i] = 0.0f;
  }
  state->learned_valid = 0U;
  state->learned_r_ohm = 0.0f;
  state->learned_revision = 0UL;
}
