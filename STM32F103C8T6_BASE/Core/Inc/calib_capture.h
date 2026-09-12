/*
 * 标定快路径定长采样窗口。
 *
 * 设计：20kHz 电流环每拍产出一份 foc_control_output_t，本模块按 stride 抽稀
 * 写入定长环形（实际为顺序）缓冲，写满 depth 后自动停止（硬件式冻结）。
 * 慢路径在激励保持/停机后读取这段已冻结数据做统计，因此快/慢路径之间无需
 * 关中断、无需加锁。
 *
 * 并发契约：
 *  - calib_capture_push 只允许在电流环中断上下文调用，必须 O(1)、非阻塞、
 *    无浮点运算、无 I2C/printf；
 *  - reset/get/count/is_full 只允许在慢路径调用，且必须在窗口冻结后读取。
 */
#ifndef CALIB_CAPTURE_H
#define CALIB_CAPTURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "foc_types.h"

#define CALIB_CAPTURE_MAX_DEPTH (64U)

/* 快路径单拍样本，单位见字段注释；固定 8 个 4 字节量 = 32B。 */
typedef struct
{
  float id;             /* d 轴电流 A，来自 output.measured_current_a.d */
  float iq;             /* q 轴电流 A */
  float vd;             /* d 轴电压 V，来自 output.voltage_command_v.d */
  float vq;             /* q 轴电压 V */
  float ia;             /* A 相电流 A，来自 output.phase_current_a.phase_a */
  float ib;             /* B 相电流 A */
  float ic;             /* C 相电流 A */
  uint32_t cycle;       /* PWM 周期号，时间基准用它，不用 ms */
} calib_sample_t;

/* step 进入时调用：清空窗口并设定深度与抽稀步长。
 * depth 超过 CALIB_CAPTURE_MAX_DEPTH 会被截断；stride 最小为 1。 */
void calib_capture_reset(uint16_t depth, uint16_t stride);

/* 快路径调用：录一帧，录满自停。 */
void calib_capture_push(const foc_control_output_t *output, uint32_t pwm_cycle);

/* 已录样本数（未满为当前条数，满后为 depth）。 */
uint16_t calib_capture_count(void);

/* 窗口是否已录满冻结。 */
uint8_t calib_capture_is_full(void);

/* 返回冻结缓冲首地址，供慢路径遍历（只读）。 */
const calib_sample_t *calib_capture_get(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_CAPTURE_H */
