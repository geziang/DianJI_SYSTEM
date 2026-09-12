/*
 * FOC 三角函数快表（计算重构）。
 *
 * 所属层：基础数学库（纯函数，不访问硬件，PC gcc 可测）。
 *
 * 背景：Cortex-M3 无 FPU，sinf/cosf 走软浮点库（单次约 700 cycles，含区间
 * 归约），放在 10kHz 电流环里是最大的一块每拍开销。这里用 256 点 Q15 正弦表
 * + 线性插值替代：
 *   - 表占 256 * 2 = 512 字节 Flash；
 *   - 线性插值后对 libm sinf 的最大绝对误差约 8.2e-5（远小于 12bit ADC 1 LSB
 *     折算到电流的量级，见 tests/pc/test_foc_isr_refactor.c）；
 *   - 四个整格点 0/pi/2/pi/3pi/2 直接返回精确 0/+1/0/-1，自检恒角（theta=0）无误差；
 *   - cos 由相位索引偏移 1/4 周期得到，不重复归一化。
 *
 * 角度单位 rad，输入任意范围（内部折叠到一个周期），不依赖角度 wrap。
 */

#ifndef FOC_TRIG_TABLE_H
#define FOC_TRIG_TABLE_H

#ifdef __cplusplus
extern "C" {
#endif

/* 一次计算同一角度的 sin/cos（Park 与反 Park 复用）。指针为空直接返回。 */
void foc_trig_sin_cos(float electrical_angle_rad, float *sine, float *cosine);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TRIG_TABLE_H */
