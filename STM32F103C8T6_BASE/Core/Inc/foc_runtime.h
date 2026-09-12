#ifndef FOC_RUNTIME_H
#define FOC_RUNTIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc_controller.h"
#include "motor_adc.h"

typedef enum
{
  FOC_RUNTIME_STOPPED = 0,
  FOC_RUNTIME_READY,
  FOC_RUNTIME_RUNNING,
  FOC_RUNTIME_FAULT
} foc_runtime_state_t;

typedef enum
{
  FOC_RUNTIME_FAULT_NONE = 0,
  FOC_RUNTIME_FAULT_INVALID_SAMPLE,
  FOC_RUNTIME_FAULT_CURRENT_MODEL,
  FOC_RUNTIME_FAULT_CONTROL,
  FOC_RUNTIME_FAULT_PWM
} foc_runtime_fault_t;

void foc_runtime_init(void);
foc_status_t foc_runtime_configure(const foc_parameter_set_t *parameters);
/* Test mode permits a deliberately bounded identification run before normal
 * current-chain certification is available. The caller still owns power limits. */
void foc_runtime_set_test_mode(uint8_t enabled);
foc_status_t foc_runtime_start(void);
void foc_runtime_stop(void);
void foc_runtime_set_current_target(float id_a, float iq_a);
/* 开环电压注入：跳过 PI 直接给 Vd/Vq（S1a 符号/相序标定用），并切到 VOLTAGE 模式。
 * 仅在 test_mode 下允许启动；电流闭环(CURRENT)必须方向已验证(closed_loop_allowed)。 */
void foc_runtime_set_voltage_target(float vd_v, float vq_v);
/* 当前请求的控制模式，供上层/门禁判断。 */
foc_control_mode_t foc_runtime_get_control_mode(void);
void foc_runtime_set_forced_angle(float electrical_angle_rad);
void foc_runtime_advance_forced_angle(float electrical_angle_step_rad);
void foc_runtime_on_current_sample(const motor_current_sample_t *sample);
foc_runtime_state_t foc_runtime_get_state(void);
foc_runtime_fault_t foc_runtime_get_fault(void);
/* 故障枚举转诊断字符串（FAULT 行子原因用）。 */
const char *foc_runtime_fault_text(foc_runtime_fault_t fault);
const foc_control_output_t *foc_runtime_get_last_output(void);

/* 慢路径（1ms 主循环）每拍取一次"拍均值"：把自上次调用以来全部快路径控制步
 * （PWM 同步采样，10kHz -> 每拍约 10 个样本）的 Id/Iq 平均后给出，并清零累加器。
 * 用途：标定收敛判定只认拍均值，不吃单拍噪声；快路径 PI 与保护路径仍用瞬时值，
 * 不经过本接口、不产生任何延迟。
 * out=均值写出；samples 非空时写出本拍实际累加的快路径样本数（诊断用）。
 * 返回 1=本拍有样本；0=无新样本（调用方应回退 last_output 瞬时值）。 */
uint8_t foc_runtime_consume_tick_mean(foc_dq_t *out, uint16_t *samples);

/* 输出观察钩子：注册后每个 20kHz 控制步在 PWM 更新成功后回调一次，
 * 供标定采样窗口（calib_capture）使用；传 0 注销。
 * 回调运行在电流环中断上下文，必须 O(1)、非阻塞、无浮点、无 I2C/printf。 */
typedef void (*foc_runtime_output_observer_fn)(const foc_control_output_t *output,
                                               uint32_t pwm_cycle);
void foc_runtime_set_output_observer(foc_runtime_output_observer_fn observer);

/* 运行中清电流环 PI 积分（不停止功率级），供 S4 阶跃前制造干净初值。 */
void foc_runtime_reset_controller(void);

/* 临时诊断探针（S1 卡死排查，定位后移除）：快路径进入次数/坏样本次数/
 * 故障次数/最后故障，仅主循环读取，ISR 内只自增、不打印。 */
void foc_runtime_get_probe(uint32_t *enter, uint32_t *bad,
                           uint32_t *faults, foc_runtime_fault_t *last_fault);

#ifdef __cplusplus
}
#endif

#endif /* FOC_RUNTIME_H */
