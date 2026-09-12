/*
 * 标定通用 step 时序引擎实现。
 */
#include "calib_step.h"

#include <math.h>

static uint8_t calib_step_cfg_valid(const calib_step_cfg_t *cfg)
{
  if (cfg == 0)
  {
    return 0U;
  }
  if ((cfg->settle_band < 0.0f) || (cfg->settle_hold_cycles == 0U) ||
      (cfg->collect_n == 0U) || (cfg->timeout_ms == 0U))
  {
    return 0U;
  }
  return 1U;
}

/* 把本拍测量压入 SETTLE 环形滑窗，返回当前窗均值（window_cycles=0 时退化为瞬时值）。 */
static float calib_step_push_window(calib_step_t *step, float measure)
{
  uint16_t w = step->cfg.settle_window_cycles;

  if (w == 0U)
  {
    step->window_mean = measure;
    return measure;
  }
  if (w > CALIB_STEP_WINDOW_MAX)
  {
    w = CALIB_STEP_WINDOW_MAX;
  }
  if (step->win_filled < w)
  {
    step->win_buf[step->win_head] = measure;
    step->win_sum += measure;
    step->win_filled++;
  }
  else
  {
    step->win_sum -= step->win_buf[step->win_head];
    step->win_buf[step->win_head] = measure;
    step->win_sum += measure;
  }
  step->win_head = (uint16_t)((step->win_head + 1U) % w);
  step->window_mean = step->win_sum / (float)step->win_filled;
  return step->window_mean;
}

void calib_step_begin(calib_step_t *step, const calib_step_cfg_t *cfg, uint32_t now_ms)
{
  uint16_t i;

  if ((step == 0) || (calib_step_cfg_valid(cfg) == 0U))
  {
    return;
  }
  step->cfg = *cfg;
  if (step->cfg.settle_window_cycles > CALIB_STEP_WINDOW_MAX)
  {
    step->cfg.settle_window_cycles = CALIB_STEP_WINDOW_MAX;
  }
  step->phase = CALIB_STEP_EXCITE;
  step->t0_ms = now_ms;
  step->in_band_run = 0U;
  step->used_samples = 0U;
  step->mean = 0.0f;
  step->spread = 0.0f;
  step->run_m2 = 0.0f;
  step->win_sum = 0.0f;
  step->window_mean = 0.0f;
  step->win_filled = 0U;
  step->win_head = 0U;
  for (i = 0U; i < CALIB_STEP_WINDOW_MAX; i++)
  {
    step->win_buf[i] = 0.0f;
  }
}

calib_step_phase_t calib_step_tick(calib_step_t *step, float measure, uint32_t now_ms)
{
  float delta, delta2;
  uint16_t n;

  if (step == 0)
  {
    return CALIB_STEP_IDLE;
  }

  switch (step->phase)
  {
    case CALIB_STEP_EXCITE:
      /* 激励由外部在 begin 同时施加，这里直接进入稳定判定。 */
      step->phase = CALIB_STEP_SETTLE;
      break;

    case CALIB_STEP_SETTLE:
    {
      float judge_value;
      uint8_t window_ready;

      if ((uint32_t)(now_ms - step->t0_ms) > step->cfg.timeout_ms)
      {
        step->phase = CALIB_STEP_NOGO;
        break;
      }
      /* 抗单拍噪声：在带判定吃滑动窗均值；窗未填满前不计在带（最多多等 W 拍）。 */
      judge_value = calib_step_push_window(step, measure);
      window_ready = (step->cfg.settle_window_cycles == 0U) ||
                     (step->win_filled >= step->cfg.settle_window_cycles);
      if ((window_ready != 0U) &&
          (fabsf(judge_value - step->cfg.target) <= step->cfg.settle_band))
      {
        ++step->in_band_run;
      }
      else
      {
        step->in_band_run = 0U;
      }
      if (step->in_band_run >= step->cfg.settle_hold_cycles)
      {
        step->phase = CALIB_STEP_COLLECT;
        step->used_samples = 0U;
        step->mean = 0.0f;
        step->spread = 0.0f;
        step->run_m2 = 0.0f;
      }
      break;
    }

    case CALIB_STEP_COLLECT:
      /* Welford 在线均值/方差，数值稳定且只需 O(1) 存储。 */
      n = (uint16_t)(step->used_samples + 1U);
      delta = measure - step->mean;
      step->mean += delta / (float)n;
      delta2 = measure - step->mean;
      step->run_m2 += delta * delta2;
      step->used_samples = n;
      step->spread = (n > 1U) ? sqrtf(step->run_m2 / (float)(n - 1U)) : 0.0f;
      if (n >= step->cfg.collect_n)
      {
        /* FR3 纹波门限：均值居中但窗内抖动过大同样 NOGO（spread_max<=0 关闭）。 */
        if ((step->cfg.spread_max > 0.0f) && (step->spread > step->cfg.spread_max))
        {
          step->phase = CALIB_STEP_NOGO;
        }
        else
        {
          step->phase = CALIB_STEP_EVALUATE;
        }
      }
      break;

    case CALIB_STEP_IDLE:
    case CALIB_STEP_EVALUATE:
    case CALIB_STEP_PASS:
    case CALIB_STEP_NOGO:
    default:
      /* 终态/待判定态保持不变。 */
      break;
  }

  return step->phase;
}

void calib_step_mark_pass(calib_step_t *step)
{
  if (step != 0)
  {
    step->phase = CALIB_STEP_PASS;
  }
}

void calib_step_force_nogo(calib_step_t *step)
{
  if (step != 0)
  {
    step->phase = CALIB_STEP_NOGO;
  }
}
