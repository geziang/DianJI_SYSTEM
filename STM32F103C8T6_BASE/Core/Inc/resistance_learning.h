/*
 * 相电阻 R 纯软件自学习/自裁决（SPEC-RUN FR-1.5，纯函数，PC gcc 可单测）。
 *
 * 设计：不安排任何外部仪表测量动作，用"强信号测量 + 跨启动统计 + 双源交叉 +
 * 偏离护栏"四级门在固件内完成 R 的裁决与更新：
 *  1 单次可信门（caller 判）：S2 三档 V-I 拟合 r2/|Voff|/样本数同时达阈值
 *    才把本次 S2 R 送入本模块（s2_trusted=1）；
 *  2 跨启动重复门：最近 N 次独立上电的可信 S2 R，中值绝对偏差/中值 <=
 *    repeat_dev_max 才允许形成 learned_R；否则维持现值并告警；
 *  3 双源交叉门：S7 动态 R 与本次 S2 实测 R 偏差 <= cross_dev_max 才共同采信；
 *  4 失控护栏：learned_R 相对编译期兜底标称（nominal）偏离超过 guard_ratio
 *    （对称 1/guard_ratio ~ guard_ratio 倍）只记录、不采纳，要求人工介入，
 *    c=clear 可清除学习值回到兜底。
 *
 * 状态持久化：state 的历史/学习值字段由参数包 v2 携带、随 Flash 固化跨启动；
 * 历史的时间升序约定：history[0] 最旧，history[count-1] 最新。
 */
#ifndef RESISTANCE_LEARNING_H
#define RESISTANCE_LEARNING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 本次 update 的判定结果位掩码（可组合）。 */
#define RESISTANCE_LEARN_F_ADOPTED   (0x01U) /* 本次可信 S2 R 已纳入历史 */
#define RESISTANCE_LEARN_F_LEARNED   (0x02U) /* 本次形成/更新了 learned_R */
#define RESISTANCE_LEARN_F_REPEAT_FAIL (0x04U) /* 跨启动重复门未过（离散大或样本不足） */
#define RESISTANCE_LEARN_F_CROSS_FAIL  (0x08U) /* 双源交叉门未过 */
#define RESISTANCE_LEARN_F_GUARD_FAIL  (0x10U) /* 失控护栏拦截（只记录不采纳） */

typedef struct
{
  uint8_t history_n;            /* 历史深度（项目取 3，与参数包 v2 字段对齐） */
  float repeat_dev_max;         /* 中值绝对偏差/中值 上限 */
  float cross_dev_max;          /* S7 动态 R 与 S2 R 相对偏差上限 */
  float guard_ratio;            /* learned_R/nominal 允许倍数（对称） */
  float nominal_r_ohm;          /* 编译期兜底标称 */
} resistance_learning_cfg_t;

typedef struct
{
  uint8_t history_count;        /* 实际历史样本数 0..history_n */
  float history_r_ohm[3];       /* 最近 N 次可信 S2 R（时间升序） */
  uint8_t learned_valid;
  float learned_r_ohm;
  uint32_t learned_revision;    /* 学习值每次更新递增（写回参数包） */
} resistance_learning_state_t;

/* 用参数包 v2 字段重建状态（上电加载后调用；包是 struct 拷贝语义，字段对齐）。 */
void resistance_learning_load(resistance_learning_state_t *state,
                              uint8_t history_count,
                              const float *history_r_ohm,
                              uint8_t learned_valid,
                              float learned_r_ohm,
                              uint32_t learned_revision);

/* 状态写回参数包 v2 字段。 */
void resistance_learning_store(const resistance_learning_state_t *state,
                               uint8_t *history_count,
                               float *history_r_ohm,
                               uint8_t *learned_valid,
                               float *learned_r_ohm,
                               uint32_t *learned_revision);

/* 一次标定序列（S2+S7）结束后调用。
 * s2_trusted=0 时什么都不采纳（返回 0），caller 负责告警。
 * s7_valid=0 时跳过双源交叉门（不更新 learned，但历史仍可累积）。 */
uint8_t resistance_learning_update(const resistance_learning_cfg_t *cfg,
                                   resistance_learning_state_t *state,
                                   float s2_r_ohm,
                                   uint8_t s2_trusted,
                                   float s7_dyn_r_ohm,
                                   uint8_t s7_valid);

/* 生效 R：learned_valid 且通过护栏时取 learned_R，否则取 fallback
 * （fallback=本次 S2 实测或标称，由 caller 决定优先级）。 */
float resistance_learning_effective_r(const resistance_learning_cfg_t *cfg,
                                      const resistance_learning_state_t *state,
                                      float fallback_r_ohm);

/* 清学习值回兜底（c=clear 语义的一部分）。 */
void resistance_learning_reset(resistance_learning_state_t *state);

/* 三样本中值与中值绝对偏差/中值（供日志与单测）。 */
float resistance_learning_median3(float a, float b, float c);
float resistance_learning_mad_ratio3(float median, float a, float b, float c);

#ifdef __cplusplus
}
#endif

#endif /* RESISTANCE_LEARNING_H */
