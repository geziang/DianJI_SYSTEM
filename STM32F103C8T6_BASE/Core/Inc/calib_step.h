/*
 * 标定通用 step 时序引擎。
 *
 * 用"进入收敛带并连续保持 K 个周期"替代旧的固定延时取值；固定时间仅作为
 * 保护性超时。一个 step 的阶段流转：
 *
 *   EXCITE -> SETTLE -> COLLECT -> EVALUATE ->(调用方判定)-> PASS/NOGO
 *
 * 引擎只负责时序、收敛判定与在线统计（均值/样本标准差）；最终 go/no-go 的
 * 业务判据由 calib_sequence（M2）读取 mean/spread 后调用
 * calib_step_mark_pass / calib_step_force_nogo 决定，引擎不越权判业务。
 *
 * 全部接口运行在慢路径（1ms 主循环）。
 */
#ifndef CALIB_STEP_H
#define CALIB_STEP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* SETTLE 滑动窗物理上限（实际窗宽由 cfg.settle_window_cycles 指定，超过此值截断）。 */
#define CALIB_STEP_WINDOW_MAX          (16U)

typedef enum
{
  CALIB_STEP_IDLE = 0,
  CALIB_STEP_EXCITE,
  CALIB_STEP_SETTLE,
  CALIB_STEP_COLLECT,
  CALIB_STEP_EVALUATE,
  CALIB_STEP_PASS,
  CALIB_STEP_NOGO
} calib_step_phase_t;

typedef struct
{
  float settle_band;             /* |measure - target| <= settle_band 视为在收敛带内 */
  float target;                  /* 收敛目标值 */
  uint16_t settle_hold_cycles;   /* 连续在带内多少个 tick 才认为稳定 */
  uint16_t collect_n;            /* 稳定后采集多少个 tick 做统计 */
  uint32_t timeout_ms;           /* 自 begin 起的保护性超时，超时直接 NOGO */
  /* 抗单拍噪声：SETTLE 用最近 settle_window_cycles 拍的滑动均值判在带；
   * 0 表示退化为逐拍（旧行为）。COLLECT 结束时若 spread 超过 spread_max 直接 NOGO，
   * spread_max<=0 关闭该纹波门限（如 S3 collect_n=1 的阶跃前稳定）。 */
  uint16_t settle_window_cycles;
  float spread_max;
} calib_step_cfg_t;

typedef struct
{
  calib_step_cfg_t cfg;
  calib_step_phase_t phase;
  uint32_t t0_ms;
  uint16_t in_band_run;
  uint16_t used_samples;
  float mean;                    /* COLLECT 在线均值 */
  float spread;                  /* COLLECT 样本标准差 */
  float run_m2;                  /* Welford 中间量，引擎内部使用 */
  /* SETTLE 滑动窗（环形）：win_sum 为窗内和，window_mean 为最近一次窗均值。 */
  float win_buf[CALIB_STEP_WINDOW_MAX];
  float win_sum;
  float window_mean;
  uint16_t win_filled;           /* 已填入样本数（<= cfg.settle_window_cycles） */
  uint16_t win_head;             /* 下一个覆盖写入位置 */
} calib_step_t;

/* 初始化一个 step 并进入 EXCITE（外部应在同一时刻施加激励）。 */
void calib_step_begin(calib_step_t *step, const calib_step_cfg_t *cfg, uint32_t now_ms);

/* 慢路径每 tick 调用，measure 为当前被测量（如 Id），返回当前阶段。
 * 进入 EVALUATE 表示统计已就绪，等待调用方判 go/no-go。 */
calib_step_phase_t calib_step_tick(calib_step_t *step, float measure, uint32_t now_ms);

/* 调用方业务判据合格/不合格时设置终态。 */
void calib_step_mark_pass(calib_step_t *step);
void calib_step_force_nogo(calib_step_t *step);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_STEP_H */
