/*
 * 标定小样本统计库（纯函数）。
 *
 * 所属层：标定能力层。本文件只做数学，不访问 HAL/ADC/I2C/PWM，可直接在 PC
 * 上用 gcc 编译单测（见 tests/pc/test_calib_stats.c）。
 *
 * 约束：
 *  - 所有函数运行在慢路径（1ms 主循环），允许 sinf/cosf/atan2f/sqrtf；
 *    严禁把本库函数放进 20kHz 电流环中断。
 *  - 输入样本数上限 CALIB_STATS_MAX_SAMPLES；函数不修改入参数组。
 */
#ifndef CALIB_STATS_H
#define CALIB_STATS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CALIB_STATS_MAX_SAMPLES (64U)

/* 去极值均值：内部排序后两端各丢弃 drop_each 个样本再平均。
 * 非法输入（空指针、n 为 0、丢弃后无样本）返回 0.0f。 */
float calib_trimmed_mean(const float *x, uint16_t n, uint16_t drop_each);

/* 中位数与四分位距（Tukey 上下半中位数定义，抗离群）。
 * median 为结果，iqr = Q3 - Q1，用于衡量离散度。非法输入输出 0。 */
void calib_median_iqr(const float *x, uint16_t n, float *median, float *iqr);

/* 一阶最小二乘拟合结果 y = slope*x + intercept；r2 为决定系数。
 * 样本不足或 x 无方差时 r2 = -1.0f 表示结果无效。 */
typedef struct
{
  float slope;
  float intercept;
  float r2;
} calib_line_t;
calib_line_t calib_least_squares(const float *x, const float *y, uint16_t n);

/* 圆周均值：atan2(mean(sin), mean(cos))，输入/输出单位 rad，结果落在 [-pi,pi]。 */
float calib_circular_mean(const float *ang, uint16_t n);

/* 圆周标准差：sqrt(-2*ln(R))，R 为合矢量模长/样本数，结果单位 rad；越接近 0 越一致。 */
float calib_circular_std(const float *ang, uint16_t n);

/* 角度解绕：prev 为上一时刻连续角，now 为本拍 wrap 到 [-pi,pi] 的角，
 * 返回跨过 +/-pi 后与 prev 连续的角度（单调累加场景用）。 */
float calib_angle_unwrap(float prev, float now);

/* ============ S7 低速动态 R 块统计（SPEC-RUN FR-1.2） ============
 * 目标：把"全样本算术平均"升级为"建立段剔除 + 分块均值 + 块级截尾"，
 * 抗瞬态与离群块；全部零 HAL，PC 与 MCU 同一份编译。
 * 时间基准：feed 每调用一次 = 1 ms（标定慢路径周期），PC 测试按次数喂。 */
#define CALIB_STATS_S7_MAX_BLOCKS (24U)

typedef struct
{
  uint16_t skip_ms;    /* 建立段跳过时长（转速/电流未稳不计入） */
  uint16_t block_ms;   /* 每块时长 */
  uint16_t max_blocks; /* 块数组容量（<= CALIB_STATS_S7_MAX_BLOCKS） */
  float min_iq_a;      /* 样本纳入门：|Iq| >= 此值才累计 */
} calib_s7_dyn_r_cfg_t;

typedef struct
{
  uint16_t skip_remaining_ms; /* 建立段剩余 ms */
  uint16_t block_ms_accum;    /* 当前块已累计 ms */
  uint16_t block_samples;     /* 当前块通过 IQ 门的样本数 */
  uint16_t block_count;       /* 已完成且有样本的块数 */
  uint32_t sample_count;      /* 通过 IQ 门的累计样本数（诊断用） */
  float block_v_sum;          /* 当前块 Vq 累计 */
  float block_i_sum;          /* 当前块 Iq 累计 */
  float block_mean_r[CALIB_STATS_S7_MAX_BLOCKS]; /* 每块 Vq均值/Iq均值 */
} calib_s7_dyn_r_accum_t;

/* 复位到初始态：建立段计时 = cfg->skip_ms、块累计清零。 */
void calib_s7_dyn_r_init(calib_s7_dyn_r_accum_t *acc,
                         const calib_s7_dyn_r_cfg_t *cfg);

/* 喂一个 1ms 拍的 Vq/Iq：建立段内忽略；|Iq| 过门才累计；块满即结算
 * （块内 ratio-of-means：块 Vq 均值 / 块 Iq 均值，比逐拍 V/I 均值更稳）。
 * 空块（整块无过门样本）丢弃；块缓冲满后新块不再记录、样本计数继续。 */
void calib_s7_dyn_r_feed(calib_s7_dyn_r_accum_t *acc,
                         const calib_s7_dyn_r_cfg_t *cfg,
                         float vq_v, float iq_a);

/* 终值：块数 >=5 时两端各截尾 1 块再平均，否则全块均值。
 * 返回 1=有效（至少 1 个有样本块）；0=零样本（caller 区分"样本不足"与"超窗"）。 */
uint8_t calib_s7_dyn_r_finalize(const calib_s7_dyn_r_accum_t *acc,
                                float *dyn_r_out);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_STATS_H */
