#ifndef ENCODER_CACHE_H
#define ENCODER_CACHE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  uint16_t raw_angle;
  uint32_t timestamp_ms;
  uint16_t consecutive_failures;
  uint8_t valid;
} encoder_cache_sample_t;

void encoder_cache_init(void);
void encoder_cache_poll(void);
const encoder_cache_sample_t *encoder_cache_get_latest(void);
uint8_t encoder_cache_is_valid(uint32_t max_age_ms);

/* I2C 总线复活（挂死自救）：DeInit+Init 外设并清缓存状态。
 * 局限：若从机死锁拉住 SDA，需配合断电（调用方负责重试上限与提示）。 */
void encoder_cache_restart_bus(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_CACHE_H */
