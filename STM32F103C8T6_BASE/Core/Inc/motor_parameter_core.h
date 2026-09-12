/*
 * 电机参数包纯逻辑核心（零 HAL，PC gcc 可单测）。
 *
 * 所属层：参数/存储能力层的纯函数部分（SPEC-RUN FR-2.6）。
 * 本文件只定义参数包结构与"不碰硬件"的裁决逻辑：CRC32、五道校验（版本/
 * 硬件 ID/电机 ID/接线版本/范围/状态）、双页 A/B 择优。Flash 擦写后端在
 * motor_flash_store，RAM 活动包/候选包管理在 motor_parameter_store。
 *
 * 包格式 v2（SPEC-RUN 附录 A）：
 *  - 新增 learned_phase_resistance_ohm 与学习历史/元字段（FR-1.5 R 自学习）；
 *  - 新增 phase_map 三相换相与 phase_seq_verified（FR-1.3 相序门禁，运行态需要）；
 *  - 为 R4 预留速度环字段（占位，运行态暂用编译期宏，不读取）；
 *  - format_version 兼容策略：加载接受 v1/v2（v1 视为恒等换相、未验证相序、
 *    无学习值的旧包），提交一律写 v2。
 */
#ifndef MOTOR_PARAMETER_CORE_H
#define MOTOR_PARAMETER_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 页选择结果（择优逻辑的纯函数输出）。 */
typedef enum
{
  MOTOR_PARAMETER_PAGE_SELECT_NONE = 0,
  MOTOR_PARAMETER_PAGE_SELECT_A,
  MOTOR_PARAMETER_PAGE_SELECT_B
} motor_parameter_page_select_t;

typedef enum
{
  MOTOR_PARAMETER_STATE_NOMINAL = 0,
  MOTOR_PARAMETER_STATE_STATIC_VALIDATED,
  MOTOR_PARAMETER_STATE_DYNAMIC_CANDIDATE,
  MOTOR_PARAMETER_STATE_COMMITTED,
  MOTOR_PARAMETER_STATE_FROZEN,
  MOTOR_PARAMETER_STATE_INVALID
} motor_parameter_state_t;

/* 状态码与 motor_parameter_store.h 保持同一取值空间（store 层直接透传）。
 * 纯核心新增两个 Flash 后端专用码，由 store 层映射后对外仍可区分。 */
typedef enum
{
  MOTOR_PARAMETER_CORE_OK = 0,
  MOTOR_PARAMETER_CORE_INVALID_ARG,
  MOTOR_PARAMETER_CORE_FORMAT_ERROR,
  MOTOR_PARAMETER_CORE_CRC_ERROR,
  MOTOR_PARAMETER_CORE_ID_MISMATCH,
  MOTOR_PARAMETER_CORE_RANGE_ERROR,
  MOTOR_PARAMETER_CORE_STATE_ERROR
} motor_parameter_core_status_t;

#define MOTOR_PARAMETER_FORMAT_VERSION_1 (1UL)
#define MOTOR_PARAMETER_FORMAT_VERSION_2 (2UL)

/* R 自学习历史深度（与 calib_config CALIB_RLEARN_HISTORY_N 对齐）。 */
#define MOTOR_PARAMETER_LEARN_HISTORY_N (3U)

typedef struct
{
  uint32_t format_version;
  uint32_t hardware_id;
  uint32_t motor_id;
  uint32_t wiring_revision;
  uint32_t parameter_revision;
  uint32_t crc;
  float current_gain_u_a_per_raw;
  float current_gain_v_a_per_raw;
  int8_t current_sign_u;
  int8_t current_sign_v;
  float phase_resistance_ohm;
  float phase_inductance_h;
  float flux_linkage_wb;
  uint16_t pole_pairs;
  int8_t encoder_direction;
  float electrical_offset_rad;
  float id_kp;
  float id_ki;
  float iq_kp;
  float iq_ki;
  float current_limit_a;
  float voltage_limit_v;
  motor_parameter_state_t state;
  /* ---- v2 新增字段（v1 加载时按默认补齐） ---- */
  uint8_t phase_map_a;            /* 控制器 A 相输出到硬件 PWM 通道 0/1/2 */
  uint8_t phase_map_b;
  uint8_t phase_map_c;
  uint8_t phase_seq_verified;     /* 相序 +90° 镜像验证通过（FR-1.3） */
  uint8_t learned_r_valid;        /* learned_R 是否可用 */
  uint8_t learned_r_count;        /* 学习历史实际样本数 0..N */
  float learned_phase_resistance_ohm;
  float learned_r_history_ohm[MOTOR_PARAMETER_LEARN_HISTORY_N];
  uint32_t learned_r_revision;    /* 学习值更新时递增的序号 */
  /* R4 预留占位（运行态不读取，速度环参数暂用编译期宏，FR 附录 A） */
  float speed_kp;
  float speed_ki;
  float speed_limit_rad_s;
  float accel_limit_rad_s2;
  float speed_estim_window_ms;
} motor_parameter_package_t;

/* 编译期静态断言：单份参数包（含头部与保留字段）必须 <= 1 KB（FR-2.1）。 */
typedef char motor_parameter_package_size_check[
    (sizeof(motor_parameter_package_t) <= 1024U) ? 1 : -1];

/* CRC32（多项式 0xEDB88320，反射），覆盖除 crc 字段自身外的全部字节。 */
uint32_t motor_parameter_core_calculate_crc(const motor_parameter_package_t *package);

/* 五道校验（版本/硬件ID/电机ID/接线版本/CRC/范围/状态）。
 * IDs 与电压上限由调用方传入（本层不依赖 board_config，保证 PC 可测）。 */
motor_parameter_core_status_t motor_parameter_core_validate(
    const motor_parameter_package_t *package,
    uint32_t hardware_id,
    uint32_t motor_id,
    uint32_t wiring_revision,
    float max_bus_voltage_v);

/* v1 旧包迁移到 v2：补默认换相/未验证/无学习值。仅当 format_version==1 有效。 */
void motor_parameter_core_migrate_v1(motor_parameter_package_t *package);

/* 双页择优（FR-2.3）：两页都过校验取 parameter_revision 新者；一页坏取另一页；
 * 相等取 A。a_valid/b_valid 由调用方先用 validate 判出。out 可为空（只问选哪页）。 */
motor_parameter_page_select_t motor_parameter_core_select_page(
    const motor_parameter_package_t *page_a, uint8_t a_valid,
    const motor_parameter_package_t *page_b, uint8_t b_valid,
    motor_parameter_package_t *out);

const char *motor_parameter_state_text(motor_parameter_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PARAMETER_CORE_H */
