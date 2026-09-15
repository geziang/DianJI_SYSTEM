#include "encoder_cache.h"

#include "board_config.h"
#include "main.h"
#include "mt6701.h"

/* 2026-09-13 Phase A 实验（L4 根因=5ms 角度台阶）：轮询 5ms->1ms，I2C 仍 100kHz。
 * 实际更新率由主循环节拍与 0.45ms 阻塞读角共同决定（约 600Hz），角度平均滞后
 * 从 ~17 降到 ~6 电角度（@ωe=130rad/s）。若上板出现 I2C 失败计数上升则回退 5ms。
 * 2026-09-14 回退判据触发：编码器链路间歇失败+总线深挂（跨复位不复原），
 * 事务密度 1ms->5ms 减负（暴露窗口 x0.2）；角度滞后回升 ~16deg@110rad/s elec，
 * 属诊断姿态——硬件整修/R3 ISR 出角后恢复 1ms。 */
#define ENCODER_CACHE_POLL_PERIOD_MS (1U)

static encoder_cache_sample_t encoder_cache_sample;
static uint32_t encoder_cache_last_poll_ms;
static uint8_t encoder_cache_have_good; /* 收到过至少一个好样本（区分 boot 期 age=0 假活） */

void encoder_cache_init(void)
{
  encoder_cache_sample.raw_angle = 0U;
  encoder_cache_sample.timestamp_ms = 0U;
  encoder_cache_sample.consecutive_failures = 0U;
  encoder_cache_sample.valid = 0U;
  encoder_cache_last_poll_ms = 0U;
  encoder_cache_have_good = 0U;
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
    encoder_cache_have_good = 1U;
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
  /* 真正的时间窗语义：距上一个好样本 <= max_age 即活——孤立坏读
   * （含三读一致性拒收）被容忍，持续断供才判死。旧实现查瞬时 valid
   * 标志，单次失败即假死（2026-09-15 上板实锤）。 */
  return (uint8_t)((encoder_cache_have_good != 0U) &&
                   ((uint32_t)(HAL_GetTick() - encoder_cache_sample.timestamp_ms) <= max_age_ms));
}

void encoder_cache_restart_bus(void)
{
  I2C_HandleTypeDef *i2c = board_config_get_encoder_i2c();

  encoder_cache_sample.valid = 0U;
  encoder_cache_sample.consecutive_failures = 0U;
  (void)HAL_I2C_DeInit(i2c);
  (void)HAL_I2C_Init(i2c);
}
