/*
 * FOC PI 控制器数学库。
 *
 * 所属层：基础数学库。
 * 只保存一个 PI 的离散积分状态，不决定电流环何时启停。
 */

#ifndef FOC_PI_H
#define FOC_PI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_types.h"

void foc_pi_init(foc_pi_state_t *state, const foc_pi_config_t *config);
void foc_pi_reset(foc_pi_state_t *state);
/* 计算重构：积分步长 ki*T 已预算进 config.ki_times_t（装载期算好），
 * 快路径不再传入/每拍计算周期。抗积分饱和采用 back-calculation 反算回拉 +
 * 积分/输出双限幅（回拉系数 Kb*T = ki_times_t/kp，见 foc_pi.c）。 */
float foc_pi_update(foc_pi_state_t *state, float error);

#ifdef __cplusplus
}
#endif

#endif /* FOC_PI_H */
