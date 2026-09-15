/*
 * 速度估计器（SPEC-RUN FR-4.1）：编码器机械角 → 机械角速度。
 *
 * 零 HAL 纯函数（输入角度序列+恒定拍周期，输出 rad/s 与有效标志）；
 * PC 单测按 owner 决定暂缓（NFR-3 偏离已登记，2026-09-13），函数形态
 * 保持可测。算法：逐拍圆周回绕差分 + 16 拍滑动窗求和（M 法窗口化）。
 * 量程上限 ±π/dt（@1ms 即 ±3141 rad/s），无混叠盲区。
 */

#ifndef SPEED_ESTIMATE_H
#define SPEED_ESTIMATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SPEED_EST_WINDOW_N (100U) /* 100 拍 @1ms = 100ms 窗（owner 2026-09-15 拍板）：
                                     稀释几十 ms 级垃圾爆发（50ms 爆发最多占窗一半，
                                     读数缓升不瞬跳、不触发合理性门）；群延迟 ~50ms，
                                     速度环带宽降至 ~5Hz——与保守 PI 匹配。 */

typedef struct
{
  float delta_rad[SPEED_EST_WINDOW_N]; /* 每拍回绕位移环形缓冲 */
  uint8_t idx;
  uint8_t count;      /* 已积累拍数（窗口未满时 < N） */
  float last_angle_rad;
  uint8_t have_last;
} speed_estimator_t;

void speed_estimator_init(speed_estimator_t *est);

/* 每控制拍调用一次；dt_s 为恒定拍周期（秒）。角度跨 2π 由回绕差分吸收。 */
void speed_estimator_update(speed_estimator_t *est, float angle_rad, float dt_s);

/* 窗口平均角速度 rad/s；窗口未满时按已积累拍数折算。 */
float speed_estimator_get_rad_s(const speed_estimator_t *est, float dt_s);

/* 窗口填满且至少两拍差分后才有效。 */
uint8_t speed_estimator_valid(const speed_estimator_t *est);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_ESTIMATE_H */
