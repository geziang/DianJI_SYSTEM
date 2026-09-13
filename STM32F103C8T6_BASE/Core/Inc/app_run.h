/*
 * 运行态最小应用骨架（M-E / SPEC-RUN R3 基建）。
 *
 * 定位：F103-run target 专属——读固化参数包、按 DD-01 契约配置 FOC runtime
 * （phase_map 从包取，不经测试参数集），READY→使能（零流目标，FOC-4 第一步）。
 * 完整 R3 功能（遥测定频/指令 hook/故障链细节/目标斜率）在此骨架上迭代。
 * 测试态（identification-test target）不含本模块，标定走 control_loop 全序列。
 */

#ifndef APP_RUN_H
#define APP_RUN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_parameter_core.h"

#define APP_RUN_VERSION "F103-run-0.1"

typedef enum
{
  APP_RUN_STATE_BOOT = 0,   /* 等参数检查/零偏/编码器准入评估 */
  APP_RUN_STATE_NEED_CAL,   /* 无有效固化包：需先烧测试固件标定 */
  APP_RUN_STATE_READY,      /* 准入通过，等 p=enable */
  APP_RUN_STATE_ENABLED,   /* 已使能：零流目标 + 编码器连续出角 */
  APP_RUN_STATE_FAULT
} app_run_state_t;

void app_run_init(void);
void app_run_poll(void);
app_run_state_t app_run_get_state(void);
const char *app_run_state_text(app_run_state_t state);

/* safety_manager PARAMETER_CHECK 的运行态回调：固化包灌入运行参数集。 */
void app_run_load_package(const motor_parameter_package_t *package);

#ifdef __cplusplus
}
#endif

#endif /* APP_RUN_H */
