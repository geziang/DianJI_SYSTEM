#include "encoder_cache.h"

#include "main.h"
#include "mt6701.h"

/* 2026-09-13 Phase A 实验（L4 根因=5ms 角度台阶）：轮询 5ms->1ms，I2C 仍 100kHz。
 * 实际更新率由主循环节拍与 0.45ms 阻塞读角共同决定（约 600Hz），角度平均滞后
 * 从 ~17 降到 ~6 电角度（@ωe=130rad/s）。若上板出现 I2C 失败计数上升则回退 5ms。 */
#define ENCODER_CACHE_POLL_PERIOD_MS (1U)

static encoder_cache_sample_t encoder_cache_sample;
static uint32_t encoder_cache_last_poll_ms;

void encoder_cache_init(void)
{
  encoder_cache_sample.raw_angle = 0U;
  encoder_cache_sample.timestamp_ms = 0U;
  encoder_cache_sample.consecutive_failures = 0U;
  encoder_cache_sample.valid = 0U;
  encoder_cache_last_poll_ms = 0U;
}

void encoder_cache_poll(void)
{
  uint16_t raw_angle;
  uint32_t now;

  now = HAL_GetTick();
  if ((uint32_t)(now - encoder_cache_last_poll_ms) < ENCODER_CACHE_POLL_PERIOD_MS)
  {
    return;
  }
  encoder_cache_last_poll_ms = now;
  if (mt6701_read_raw_angle(&raw_angle) == MT6701_STATUS_OK)
  {
    encoder_cache_sample.raw_angle = raw_angle;
    encoder_cache_sample.timestamp_ms = now;
    encoder_cache_sample.consecutive_failures = 0U;
    encoder_cache_sample.valid = 1U;
  }
  else
  {
    encoder_cache_sample.consecutive_failures++;
    encoder_cache_sample.valid = 0U;
  }
}

const encoder_cache_sample_t *encoder_cache_get_latest(void)
{
  return &encoder_cache_sample;
}

uint8_t encoder_cache_is_valid(uint32_t max_age_ms)
{
  return (uint8_t)((encoder_cache_sample.valid != 0U) &&
                   ((uint32_t)(HAL_GetTick() - encoder_cache_sample.timestamp_ms) <= max_age_ms));
}
