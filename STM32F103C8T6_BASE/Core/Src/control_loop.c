#include "control_loop.h"

#include "board_config.h"
#include "calib_capture.h"
#include "calib_config.h"
#include "calib_sequence.h"
#include "calib_stats.h"
#include "calib_step.h"
#include "debug_log.h"
#include "encoder_cache.h"
#include "foc_angle_math.h"
#include "foc_identification.h"
#include "foc_params.h"
#include "foc_precompute.h"
#include "foc_rotor_model.h"
#include "foc_runtime.h"
#include "motor_adc.h"
#include "motor_drv.h"
#include "motor_parameter_store.h"
#include "motor_pwm.h"
#include "resistance_learning.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* Test boundaries, not measured motor parameters. Keep these deliberately
 * small until the experiment proves the current chain on this hardware. */
/* 诊断配置：与真实母线对齐到 10V（板上限 MAX_BUS=10V）。标度对齐后命令电压=真实电压，
 * VEND/限幅/电流门限的物理意义不变；正式 6V 标定时改回 6.0f。 */
#define TEST_BUS_VOLTAGE_V             (10.0f)
#define TEST_MAX_VOLTAGE_V             (5.5f)  /* 测试态电压上限：大电流强信号压噪方案；5.5V 留 SVPWM 饱和裕量（10V 母线正弦调制上限≈5.77V），静止时 5.5V/(1.5×R)≈1.6A，靠 TEST_MAX_CURRENT_A 与台 CC≥1.2A 兜底 */
#define TEST_MAX_CURRENT_A             (1.0f)  /* 测试态电流上限：额定 1A，大信号压噪；台电源 CC 须同步≥1.2A */
#define TEST_ALIGN_CURRENT_A           (0.06f) /* B 组 SENSOR 锁角用，A 组 S1 用 calib_config */
#define TEST_SENSOR_TIME_MS            (400U)
#define TEST_CAPTURE_TIME_MS           (2500U)
/* 历史 S7 累计门限；SPEC-RUN FR-1.1 后 S7 样本门改由 CALIB_S7_DYN_R_MIN_IQ_RATIO
 * 与工作点联动（见 calib_s7_dyn_r cfg），本宏仅留档不复用。 */
#define TEST_NOMINAL_R_OHM             (2.30f)
#define TEST_NOMINAL_L_H               (0.00086f)
#define TEST_PI_TIME_CONSTANT_S        (0.010f)
#define TEST_ENCODER_BOOT_MAX_AGE_MS   (50U)

/* 主序列：S1 符号 -> S2 R -> S3 L -> S4 电流环自检 -> 方向 -> 线序 -> S5 零位 -> CAPTURE。
 * 顺序约束（方案 A）：ENCODER_DIR 必须先于 PHASE_SEQ——方向是纯手动、不上电、
 * 不依赖相序的独立测量；PHASE_SEQ 的 +90° verify 用 sign(d_elec)=dir*e*m 判相序
 * 手性，只有 dir 已等于真实编码器方向 e 时，该式才能一次定解 m。若方向在后，
 * verify 只能拿到 e*m 的乘积（一个方程两个未知量），事后再翻转 dir 就必须联动
 * 镜像 phase_map 来补偿，形成双重翻转的路径依赖，故旧联动逻辑已删除。 */
typedef enum
{
  TEST_STEP_SIGN = 0,
  TEST_STEP_R,
  TEST_STEP_L,
  TEST_STEP_CURRENT,
  TEST_STEP_ENCODER_DIR,
  TEST_STEP_PHASE_SEQ,
  TEST_STEP_SENSOR,
  TEST_STEP_CAPTURE,
  TEST_STEP_DONE
} test_step_t;

/* 原 S2 相电阻三档电流；S2 已改"用标称不辨识"，此数组仅被不可达的
 * R_IDENTIFICATION 死代码引用，故保留以满足编译（不参与实际流程）。 */
static const float control_loop_s2_levels[CALIB_S2_LEVEL_COUNT] = CALIB_S2_CURRENT_LEVELS_A;

static control_loop_state_t control_loop_state;
static control_loop_fault_t control_loop_fault;
static uint32_t control_loop_state_tick;
static test_step_t control_loop_next_step;
static foc_parameter_set_t control_loop_parameters;
static motor_parameter_package_t control_loop_candidate;
static uint8_t control_loop_candidate_ready;
static uint8_t control_loop_test_precheck_passed;
static float control_loop_static_r_ohm;
static float control_loop_static_l_h;

/* ============ SPEC-RUN R1/R2：S7 判据 v2 窗统计 + R 自学习 + 参数注入 ============ */

/* S7 判据 v2 窗统计（2026-09-13 L4 结案小包）：50ms 窗内 id/iq 吃 tick 均值、
 * vd/vq 吃每拍输出，建立段后累计；窗尾结算 EMF-free 比值 wvd/wid（vd=R*id-ωL*iq，
 * EMF 只在 q 轴），判决取全部窗比值的中位数（抗离群，FR-1.2 精神）；
 * 全程窗均值另喂调节品质门。旧分块统计（calib_s7_dyn_r_*）已退役，
 * 函数保留在 calib_stats 供 PC 单测与历史追溯。 */
static float control_loop_s7_wvd_sum, control_loop_s7_wvq_sum; /* 50ms 窗电压和 */
static uint16_t control_loop_s7_vn;                             /* 窗内电压累计拍数 */
static float control_loop_s7_wratio[CALIB_S7_WINDOW_COUNT];     /* 每窗 wvd/wid */
static uint16_t control_loop_s7_wratio_n;                       /* 已结算窗比值数 */
static float control_loop_s7_cap_wid_sum, control_loop_s7_cap_wiq_sum; /* 全程窗均值累计 */
static uint16_t control_loop_s7_cap_wn;                         /* 已累计窗数 */
/* 编译期护栏：窗数不得超出统计库中位数容量。 */
typedef char control_loop_s7_window_check[
    (CALIB_S7_WINDOW_COUNT <= CALIB_STATS_MAX_SAMPLES) ? 1 : -1];

/* S7 行程/力矩方向门禁（FR-1.3.2）。 */
static float control_loop_s7_mech_prev;
static float control_loop_s7_mech_delta;
/* 失步早期检测：双条件同时满足的持续计数 + 遥测时间戳。 */
static uint32_t control_loop_s7_los_ms;
static uint32_t control_loop_s7_dbg_ms;
/* L4 判别探针（2026-09-13）：S7 遥测并列 50ms 窗均值——tick 均值按打印周期累计，
 * 与瞬时值对照判别"电角频率×打印周期混叠"：窗均值≈目标(0.10/0.18)而瞬时值偏航
 * = 混叠实锤（修统计+角度外推）；窗均值≈打印值 = 旋转下测量真误差（扫采样相位）。 */
static float control_loop_s7_wid_sum, control_loop_s7_wiq_sum;
static uint16_t control_loop_s7_wn;
/* S5 偏置本次是否可信（S7 入口门禁用：S5 ok=0 时角度链未证实，禁止加功率）。 */
static uint8_t control_loop_s5_ok;

/* 本次上电 S2/S7 结果（喂 R 自学习四级门，FR-1.5）。
 * s2_adopted 实现"独立上电"语义：每次上电最多采纳一个可信 S2 R。 */
static float control_loop_s2_r_ohm;
static uint8_t control_loop_s2_trusted;
static uint8_t control_loop_s2_adopted;
static uint8_t control_loop_s7_valid;

/* R 自学习状态（跨启动：由参数包 v2 携带，FR-1.5.2）。 */
static resistance_learning_cfg_t control_loop_rlearn_cfg;
static resistance_learning_state_t control_loop_rlearn;

/* 相序 +90° 镜像验证结论（FR-1.3：进候选包、决定能否永久固化）。 */
static uint8_t control_loop_phase_seq_verified;

/* 上电加载的已固化参数包（FR-2.4 注入源；prepare_test_parameters 每轮以它为起点）。 */
static motor_parameter_package_t control_loop_loaded;
static uint8_t control_loop_loaded_valid;

/* 通用 step 引擎（同一时刻只有一个标定态在跑，复用一个实例）。 */
static calib_step_t control_loop_step;

/* S1 两段式：S1a 开环电压爬坡扫描（内部档位引擎，用原始 raw + 线性拟合判定），
 * S1b 方向验证通过后建电流闭环自证。 */
typedef enum
{
  S1A_BASELINE = 0,   /* 零矢量基线，取通电态 raw 零点 */
  S1A_SWEEP,          /* 电压档位爬坡扫描（正/反向，内部档内相位驱动） */
  S1A_DECAY,          /* 正→反方向之间回零衰减 */
  S1A_EVAL,           /* 正/负扫描点线性拟合，一次性评估 */
  S1A_HOLD_ALIGN,     /* 【诊断】锁 θ=0 持续注入固定 Vd，按 x 停，供手转感受阻尼 */
  S1A_P3_BASELINE,    /* PROBE3：角度 k 零矢量基线 */
  S1A_P3_INJECT,      /* PROBE3：角度 k 注入 Vtest，SETTLE 后采集均值 */
  S1A_P3_DECAY,       /* PROBE3：角度 k 回零衰减，再换下一角度 */
  S1A_P3_EVAL,        /* PROBE3：三角度齐，解算符号/通道，绑定或故障 */
  S1A_P3_DONE,        /* PROBE3：已绑定并断电，等再按 p 进 S1b */
  S1B_CLOSED_LOOP,    /* 电流闭环自证（走通用 step 引擎） */
  S1_FINISHED
} s1_subphase_t;

/* S1a 单个电压档内的三相位：缓升→跳过建立→采集均值。 */
typedef enum
{
  S1_LVL_RAMP = 0,
  S1_LVL_SETTLE,
  S1_LVL_COLLECT
} s1_level_phase_t;

static s1_subphase_t control_loop_s1_sub;
static uint32_t control_loop_s1_phase_tick;
static float control_loop_s1_base_u, control_loop_s1_base_v;
static uint16_t control_loop_s1_base_n;
/* S1a 爬坡扫描运行态（点缓冲走 static/.bss，不上栈）。 */
static int8_t control_loop_s1_dir;             /* +1 正向 / -1 负向 */
static uint8_t control_loop_s1_level;          /* 当前档索引 */
static uint8_t control_loop_s1_pos_levels;     /* 正向实际扫的档数，反向追平以保证对称 */
static s1_level_phase_t control_loop_s1_lvlph;
static float control_loop_s1_lvl_u, control_loop_s1_lvl_v;
static uint16_t control_loop_s1_lvl_n;
static uint8_t control_loop_s1_reached;        /* 正向电流进过目标带 */
static float control_loop_s1_lock_vd;
static float control_loop_s1_tlo_code, control_loop_s1_hard_code;
static uint8_t control_loop_s1_np, control_loop_s1_nn;
static float control_loop_s1_vdp[CALIB_S1A_LEVEL_MAX], control_loop_s1_dup[CALIB_S1A_LEVEL_MAX],
             control_loop_s1_dvp[CALIB_S1A_LEVEL_MAX];
static float control_loop_s1_vdn[CALIB_S1A_LEVEL_MAX], control_loop_s1_dun[CALIB_S1A_LEVEL_MAX],
             control_loop_s1_dvn[CALIB_S1A_LEVEL_MAX];
static int8_t control_loop_s1_sign_u;
static int8_t control_loop_s1_sign_v;
/* PROBE3 三角度探针运行态（注入均值复用 lvl_u/v/n，基线复用 base_u/v/n）。 */
static uint8_t control_loop_p3_k;                 /* 当前角度索引 0..2 */
static uint8_t control_loop_p3_bound;             /* 三角度已绑定符号（再按 p 直接进 S1b） */
static float control_loop_p3_du[CALIB_S1A_P3_ANGLE_COUNT];
static float control_loop_p3_dv[CALIB_S1A_P3_ANGLE_COUNT];
/* PROBE3 注入电角度：0°/120°/240°（rad）。 */
static const float control_loop_p3_theta_rad[CALIB_S1A_P3_ANGLE_COUNT] =
    { 0.0f, 2.0943951f, 4.1887902f };

/* S2 三档。 */
static uint8_t control_loop_s2_level;
static float control_loop_s2_id[CALIB_S2_LEVEL_COUNT];
static float control_loop_s2_vd[CALIB_S2_LEVEL_COUNT];
static float control_loop_s2_id_sum;
static float control_loop_s2_vd_sum;
static uint16_t control_loop_s2_n;

/* S3 两小阶段：0=等 I0 稳态，1=已阶跃、等快窗口录满。 */
static uint8_t control_loop_s3_step_issued;

/* S4 阶跃跟踪器。 */
static calib_s4_tracker_t control_loop_s4;
/* S1b：COLLECT 段 Iq 拍均值累加（EVALUATE 用均值判 Iq≈0，不吃单拍）。 */
static float control_loop_s1b_iq_sum;
static uint16_t control_loop_s1b_iq_n;
/* S4 诊断心跳节流时间戳。 */
static uint32_t control_loop_s4_hb_ms;
/* S4 建立期状态（对齐 S1b 启动时序）：ISR 先在 FROM(=0) 跑稳一段固定时长，
 * 再阶跃到 TO，并把观测时钟从阶跃时刻起算，排除 0->TO 上升瞬态误判。 */
static uint8_t control_loop_s4_step_pending;   /* 1=等待 FROM 建立后才阶跃到 TO */
static uint32_t control_loop_s4_establish_ms; /* FROM 建立期起点戳 */
static uint32_t control_loop_s4_start_ms;      /* 阶跃后观测起点戳（喂 s4_update 的 elapsed） */
/* 主循环心跳：仅状态变化时打印，避免稳态刷屏。 */
static uint32_t control_loop_last_hb_ms;
static control_loop_state_t control_loop_last_hb_state = CONTROL_LOOP_STATE_SAFE_IDLE;
/* 临时：SIGN 心跳节流时间戳。 */
static uint32_t control_loop_sign_hb_ms;
/* [LIMIT] 安全边界行每轮标定只打一次。 */
static uint8_t control_loop_limits_printed;

/* ============ 批次 B：方向 / 线序 / S5 零位 三标定步（方案 A：方向先于线序） ============ */

/* 6 种换相（S3 全排列）：perm[p][0..2] = phase_a/b/c_output，下标与
 * calib_sequence_phase_pick 输入顺序一致。 */
static const uint8_t control_loop_phase_perms[CALIB_PHASE_SEQ_PERM_COUNT][3] = {
  { 0U, 1U, 2U },
  { 0U, 2U, 1U },
  { 1U, 0U, 2U },
  { 1U, 2U, 0U },
  { 2U, 0U, 1U },
  { 2U, 1U, 0U }
};

/* ---- 线序 phase-seq：6 换相静态 d 轴探针子状态 ---- */
typedef enum
{
  PSEQ_PREP = 0,   /* safe_disable + 写换相 + 开环注入 Vd */
  PSEQ_SETTLE,     /* 注入后跳过建立 */
  PSEQ_COLLECT,    /* 采集 Id/Iq 均值 */
  PSEQ_DECAY,      /* 回零衰减，换下一种换相 */
  PSEQ_EVAL,       /* 6 种齐，选优（不直接提交） */
  PSEQ_VERIFY_PREP,    /* 镜像验证：写 best 换相，θ=0 开环起压（一次性配置） */
  PSEQ_VERIFY_ALIGN,   /* θ=0 保持 Vd，转子对齐后记录编码器基准角 */
  PSEQ_VERIFY_PULSE,   /* 矢量切 θ=+90° 保持，转子机械就位后记录编码器角 */
  PSEQ_VERIFY_DECAY,   /* 验证回零衰减并断电 */
  PSEQ_COMMIT      /* 按编码器角度增量判决（含镜像纠正 idx^1）并写内存 phase_map */
} phase_seq_subphase_t;

static uint8_t control_loop_pseq_idx;
static phase_seq_subphase_t control_loop_pseq_sub;
static uint32_t control_loop_pseq_tick;
static float control_loop_pseq_id_sum, control_loop_pseq_iq_sum;
static uint16_t control_loop_pseq_n;
static float control_loop_pseq_id[CALIB_PHASE_SEQ_PERM_COUNT];
static float control_loop_pseq_iq[CALIB_PHASE_SEQ_PERM_COUNT];
static uint8_t control_loop_pseq_best;                           /* EVAL 选出的 best 排列下标 */
/* 镜像验证探针：θ=0 对齐末 / +90° 保持末 的编码器机械角基准；
 * enc_ok bit0=基准已采、bit1=就位角已采，COMMIT 要求两位齐才判决。 */
static float control_loop_pseq_enc_before, control_loop_pseq_enc_after;
static uint8_t control_loop_pseq_enc_ok;

/* ---- 方向 encoder_direction：手动旋转模式（不上电，用户手转电机，固件读角度变化） ---- */
typedef enum
{
  DIR_MANUAL_WAIT = 0,  /* 等待用户手转，累计角度变化 */
  DIR_MANUAL_DONE        /* 达到阈值，判定方向 */
} dir_subphase_t;

static dir_subphase_t control_loop_dir_sub;
static uint32_t control_loop_dir_tick;
static float control_loop_dir_mech_prev;
static float control_loop_dir_mech_delta;
static uint32_t control_loop_dir_hb_ms; /* 手转等待进度心跳节流 */

/* ---- S5 零位：多次正反向 d 轴对齐子状态 ---- */
typedef enum
{
  S5_ALIGN = 0,     /* 正向 d 轴对齐，读 offset */
  S5_REV_PREMOVE,   /* 反向趋近前预扭转到相反电角 */
  S5_EVAL           /* 圆周统计并写 offset */
} s5_subphase_t;

static s5_subphase_t control_loop_s5_sub;
static uint32_t control_loop_s5_tick;
static uint8_t control_loop_s5_idx;
static float control_loop_s5_offsets[CALIB_S5_REPEAT_COUNT];

/* ---- VJ 电压注入判决模式（L4 诊断，2026-09-13；不在 S0-S8 序列内，不产候选不落盘） ---- */
typedef enum
{
  VJ_SUB_IDLE = 0,   /* 已进入 VJ、未上功率，等 p */
  VJ_SUB_HOLD,       /* 上电保持零压，等注入命令 */
  VJ_SUB_PULSE,      /* 静止单轴脉冲进行中（末段均值打印 [VJ] 行） */
  VJ_SUB_SWEEP       /* vq 旋转扫描进行中（[VJT] 遥测） */
} vj_subphase_t;

static vj_subphase_t control_loop_vj_sub;
static uint32_t control_loop_vj_tick;
static float control_loop_vj_vd, control_loop_vj_vq;
static uint8_t control_loop_vj_axis_q;   /* 本脉冲作用轴：0=vd 1=vq */
static float control_loop_vj_id_sum, control_loop_vj_iq_sum;
static float control_loop_vj_du_sum, control_loop_vj_dv_sum; /* raw 相对零偏偏移累计 */
static uint16_t control_loop_vj_n;
static uint32_t control_loop_vj_tlm_ms;  /* 扫描遥测节流时间戳 */
static float control_loop_vj_theta;      /* 最近一拍电角度（打印用） */
static uint32_t control_loop_vj_enc_bad_ms; /* 编码器连续无效毫秒数（去抖，防使能瞬态误判） */

static void control_loop_log(const char *text)
{
  debug_log_write_line(text);
}

static void control_loop_log_limits(void)
{
  char line[144];

  (void)snprintf(line, sizeof(line),
                 "[LIMIT] assumed_bus=%.1fV max_voltage=%.2fV max_current=%.2fA",
                 (double)TEST_BUS_VOLTAGE_V,
                 (double)TEST_MAX_VOLTAGE_V,
                 (double)TEST_MAX_CURRENT_A);
  control_loop_log(line);
}

static void control_loop_safe_disable(void)
{
  /* 任何步停机都注销快路径观察钩子，避免泄漏到后续状态。 */
  foc_runtime_set_output_observer(0);
  motor_drv_disable();
  foc_runtime_stop();
  motor_adc_stop_synchronized();
  motor_pwm_force_zero();
  motor_pwm_stop_output();
}

static void control_loop_enter_fault(control_loop_fault_t fault)
{
  control_loop_safe_disable();
  control_loop_fault = fault;
  control_loop_state = CONTROL_LOOP_STATE_FAULT;
  control_loop_log("[FAULT] output disabled; check hardware, then use c to retry");
}

static void control_loop_reset_measurements(void)
{
  uint8_t i;

  control_loop_s1_sub = S1A_BASELINE;
  control_loop_s1_phase_tick = 0U;
  control_loop_s1_base_u = 0.0f;
  control_loop_s1_base_v = 0.0f;
  control_loop_s1_base_n = 0U;
  control_loop_p3_k = 0U;
  control_loop_p3_bound = 0U;
  for (i = 0U; i < CALIB_S1A_P3_ANGLE_COUNT; i++)
  {
    control_loop_p3_du[i] = 0.0f;
    control_loop_p3_dv[i] = 0.0f;
  }
  control_loop_s1_dir = 1;
  control_loop_s1_level = 0U;
  control_loop_s1_pos_levels = 0U;
  control_loop_s1_lvlph = S1_LVL_RAMP;
  control_loop_s1_lvl_u = 0.0f;
  control_loop_s1_lvl_v = 0.0f;
  control_loop_s1_lvl_n = 0U;
  control_loop_s1_reached = 0U;
  control_loop_s1_lock_vd = 0.0f;
  control_loop_s1_tlo_code = control_loop_s1_hard_code = 0.0f;
  control_loop_s1_np = 0U;
  control_loop_s1_nn = 0U;
  control_loop_s2_level = 0U;
  control_loop_s2_id_sum = 0.0f;
  control_loop_s2_vd_sum = 0.0f;
  control_loop_s2_n = 0U;
  for (i = 0U; i < CALIB_S2_LEVEL_COUNT; i++)
  {
    control_loop_s2_id[i] = 0.0f;
    control_loop_s2_vd[i] = 0.0f;
  }
  control_loop_s3_step_issued = 0U;
  control_loop_pseq_idx = 0U;
  control_loop_pseq_sub = PSEQ_PREP;
  control_loop_pseq_tick = 0U;
  control_loop_pseq_id_sum = 0.0f;
  control_loop_pseq_iq_sum = 0.0f;
  control_loop_pseq_n = 0U;
  control_loop_pseq_best = 0U;
  control_loop_pseq_enc_before = 0.0f;
  control_loop_pseq_enc_after = 0.0f;
  control_loop_pseq_enc_ok = 0U;
  for (i = 0U; i < CALIB_PHASE_SEQ_PERM_COUNT; i++)
  {
    control_loop_pseq_id[i] = 0.0f;
    control_loop_pseq_iq[i] = 0.0f;
  }
  control_loop_dir_sub = DIR_MANUAL_WAIT;
  control_loop_dir_tick = 0U;
  control_loop_dir_mech_prev = 0.0f;
  control_loop_dir_mech_delta = 0.0f;
  control_loop_dir_hb_ms = 0U;
  control_loop_s5_sub = S5_ALIGN;
  control_loop_s5_tick = 0U;
  control_loop_s5_idx = 0U;
  for (i = 0U; i < CALIB_S5_REPEAT_COUNT; i++)
  {
    control_loop_s5_offsets[i] = 0.0f;
  }
  control_loop_vj_sub = VJ_SUB_IDLE;
  control_loop_vj_tick = 0U;
  control_loop_vj_vd = 0.0f;
  control_loop_vj_vq = 0.0f;
  control_loop_vj_axis_q = 0U;
  control_loop_vj_id_sum = 0.0f;
  control_loop_vj_iq_sum = 0.0f;
  control_loop_vj_du_sum = 0.0f;
  control_loop_vj_dv_sum = 0.0f;
  control_loop_vj_n = 0U;
  control_loop_vj_tlm_ms = 0U;
  control_loop_vj_theta = 0.0f;
  control_loop_vj_enc_bad_ms = 0U;
}

/* 用统一参数 begin 一个 step：target 为收敛目标，band 为在带阈值，
 * collect_n 为稳态统计拍数，spread_max_a 为 COLLECT 样本标准差上限（<=0 关闭纹波门限）。
 * SETTLE 统一走 CALIB_SETTLE_WINDOW_CYCLES 拍滑动均值判在带（抗单拍噪声）。 */
static void control_loop_begin_step(float target, float band, uint16_t collect_n,
                                    float spread_max_a)
{
  calib_step_cfg_t cfg;

  cfg.target = target;
  cfg.settle_band = band;
  cfg.settle_hold_cycles = CALIB_SETTLE_HOLD_CYCLES;
  cfg.collect_n = collect_n;
  cfg.timeout_ms = CALIB_STEP_TIMEOUT_MS;
  cfg.settle_window_cycles = CALIB_SETTLE_WINDOW_CYCLES;
  cfg.spread_max = spread_max_a;
  calib_step_begin(&control_loop_step, &cfg, HAL_GetTick());
}

static uint8_t control_loop_prepare_test_parameters(void)
{
  motor_adc_current_diagnostic_t diagnostic;
  foc_current_pi_result_t pi;

  motor_adc_get_current_diagnostic(&diagnostic);
  if ((diagnostic.adc_hardware_calibrated == 0U) ||
      (diagnostic.zero.quality != MOTOR_ADC_ZERO_QUALITY_OK))
  {
    control_loop_log("[TEST] ADC zero check failed; power remains disabled");
    return 0U;
  }

  /* S0：编码器是后续 S5/S7 的必需项，boot 即硬门禁。 */
  if (encoder_cache_is_valid(TEST_ENCODER_BOOT_MAX_AGE_MS) == 0U)
  {
    control_loop_log("[TEST] encoder not online; check MT6701/I2C, then r to retry");
    return 0U;
  }

  foc_parameter_set_load_default(&control_loop_parameters);
  control_loop_parameters.current.channel_u_zero_raw =
      (float)diagnostic.zero.phase_u_zero_raw;
  control_loop_parameters.current.channel_v_zero_raw =
      (float)diagnostic.zero.phase_v_zero_raw;
  control_loop_parameters.current.calibrated = 1U;
  control_loop_parameters.bus_voltage_v = TEST_BUS_VOLTAGE_V;
  control_loop_parameters.max_voltage_v = TEST_MAX_VOLTAGE_V;
  control_loop_parameters.max_current_a = TEST_MAX_CURRENT_A;
  control_loop_parameters.pwm_period_ticks = BOARD_CONFIG_PWM_PERIOD_TICKS;

  /* S1a 未跑前符号仅作观测层(A 显示)的安全默认 +1，绝不是建电流环的前提：
   * S1a 用开环 VOLTAGE + 原始 raw 判符号，闭环门禁 closed_loop_allowed 此时为 0。 */
  control_loop_parameters.current.channel_u_sign = 1.0f;
  control_loop_parameters.current.channel_v_sign = 1.0f;

  /* SPEC-RUN FR-2.4/1.5.5：上电注入的已固化参数作为测试起点——角度链/
   * 实测符号/静态 R、L；ADC 零偏仍取本拍实测（上面已设）。
   * phase_map 刻意不恢复：P3 判据按恒等映射写死（绑定锚点），注入上靴
   * 换相会让 S1a 出通道交叉假故障（2026-09-13 教训）；PHASE_SEQ 每靴
   * 自会重探并覆写 RAM，运行态 map 走 boot 注入不经此处。 */
  if (control_loop_loaded_valid != 0U)
  {
    control_loop_parameters.rotor.pole_pairs = control_loop_loaded.pole_pairs;
    control_loop_parameters.rotor.encoder_direction = control_loop_loaded.encoder_direction;
    control_loop_parameters.rotor.electrical_offset_rad =
        control_loop_loaded.electrical_offset_rad;
    control_loop_parameters.current.channel_u_sign =
        (float)control_loop_loaded.current_sign_u;
    control_loop_parameters.current.channel_v_sign =
        (float)control_loop_loaded.current_sign_v;
    control_loop_static_r_ohm = control_loop_loaded.phase_resistance_ohm;
    control_loop_static_l_h = control_loop_loaded.phase_inductance_h;
    control_loop_phase_seq_verified = control_loop_loaded.phase_seq_verified;
  }

  /* 电流环 PI 基准：注入且学习值可用时取生效 R（learned_R 优先、护栏兜底），
   * 否则编译期标称（原行为不变）。 */
  {
    float pi_base_r = TEST_NOMINAL_R_OHM;
    float pi_base_l = TEST_NOMINAL_L_H;
    if (control_loop_loaded_valid != 0U)
    {
      pi_base_r = resistance_learning_effective_r(&control_loop_rlearn_cfg,
                                                  &control_loop_rlearn,
                                                  control_loop_loaded.phase_resistance_ohm);
      pi_base_l = control_loop_loaded.phase_inductance_h;
    }
    if (foc_identification_derive_current_pi(pi_base_r,
                                             pi_base_l,
                                             TEST_PI_TIME_CONSTANT_S,
                                             TEST_MAX_VOLTAGE_V,
                                             &pi) != FOC_STATUS_OK)
    {
      return 0U;
    }
  }
  control_loop_parameters.id_pi = pi.id_pi;
  control_loop_parameters.iq_pi = pi.iq_pi;

  /* 计算重构：所有"一次测试内不变"的派生量（code_to_amp/inv_bus/period_f/ki_t）
   * 在进入任何功率步之前一次性预算。sign 晚到 S1a 才翻转，故刻意不进 code_to_amp。
   * 每步 start_power 都会重新 configure 本参数集，预算结果随之带入运行时。 */
  if (foc_parameter_set_precompute(&control_loop_parameters,
                                   BOARD_CONFIG_CURRENT_LOOP_PERIOD_S) != FOC_STATUS_OK)
  {
    return 0U;
  }

  foc_runtime_set_test_mode(1U);
  return (uint8_t)(foc_runtime_configure(&control_loop_parameters) == FOC_STATUS_OK);
}

/* 步骤简称查表（定义在 arm_next_step 前，启动函数先用于 [RUN] 行）。 */
static const char *control_loop_step_name(void);

static uint8_t control_loop_start_power(float id_a, float iq_a)
{
  if (foc_runtime_configure(&control_loop_parameters) != FOC_STATUS_OK)
  {
    control_loop_log("[DBG] sp:cfg FAIL");
    control_loop_enter_fault(CONTROL_LOOP_FAULT_CURRENT_LOOP);
    return 0U;
  }
  if (motor_adc_start_synchronized() != MOTOR_ADC_STATUS_OK)
  {
    control_loop_log("[DBG] sp:sync FAIL");
    control_loop_enter_fault(CONTROL_LOOP_FAULT_ADC);
    return 0U;
  }

  foc_runtime_set_forced_angle(0.0f);
  foc_runtime_set_current_target(id_a, iq_a);
  if (motor_pwm_start_test_output() != MOTOR_PWM_STATUS_OK)
  {
    control_loop_log("[DBG] sp:pwm FAIL");
    control_loop_enter_fault(CONTROL_LOOP_FAULT_PWM_START);
    return 0U;
  }
  if (motor_drv_enable_for_test() != MOTOR_DRV_STATUS_OK)
  {
    control_loop_log("[DBG] sp:drv FAIL");
    control_loop_enter_fault(CONTROL_LOOP_FAULT_DRIVER_ENABLE);
    return 0U;
  }
  if (foc_runtime_start() != FOC_STATUS_OK)
  {
    control_loop_log("[DBG] sp:rt FAIL");
    control_loop_enter_fault(CONTROL_LOOP_FAULT_CURRENT_LOOP);
    return 0U;
  }
  {
    char line[96];
    (void)snprintf(line, sizeof(line), "[RUN] %s started", control_loop_step_name());
    control_loop_log(line);
  }
  return 1U;
}

/* S1a 专用：以开环 VOLTAGE 模式启动功率（绕过 PI，方向未验证也允许）。
 * 启动后初始给定 0V，由 SIGN 子状态机用 set_voltage_target 施加双极性注入。 */
static uint8_t control_loop_start_openloop_power(void)
{
  if (foc_runtime_configure(&control_loop_parameters) != FOC_STATUS_OK)
  {
    control_loop_log("[DBG] sp:cfg FAIL");
    control_loop_enter_fault(CONTROL_LOOP_FAULT_CURRENT_LOOP);
    return 0U;
  }
  foc_runtime_set_voltage_target(0.0f, 0.0f);
  if (motor_adc_start_synchronized() != MOTOR_ADC_STATUS_OK)
  {
    control_loop_log("[DBG] sp:sync FAIL");
    control_loop_enter_fault(CONTROL_LOOP_FAULT_ADC);
    return 0U;
  }
  foc_runtime_set_forced_angle(0.0f);
  foc_runtime_set_voltage_target(0.0f, 0.0f);
  if (motor_pwm_start_test_output() != MOTOR_PWM_STATUS_OK)
  {
    control_loop_log("[DBG] sp:pwm FAIL");
    control_loop_enter_fault(CONTROL_LOOP_FAULT_PWM_START);
    return 0U;
  }
  if (motor_drv_enable_for_test() != MOTOR_DRV_STATUS_OK)
  {
    control_loop_log("[DBG] sp:drv FAIL");
    control_loop_enter_fault(CONTROL_LOOP_FAULT_DRIVER_ENABLE);
    return 0U;
  }
  if (foc_runtime_start() != FOC_STATUS_OK)
  {
    control_loop_log("[DBG] sp:rt FAIL");
    control_loop_enter_fault(CONTROL_LOOP_FAULT_CURRENT_LOOP);
    return 0U;
  }
  {
    char line[112];
    (void)snprintf(line, sizeof(line), "[RUN] %s started (openloop)", control_loop_step_name());
    control_loop_log(line);
  }
  return 1U;
}

static void control_loop_finish_power(const char *reason)
{
  char line[96];

  control_loop_safe_disable();
  (void)snprintf(line, sizeof(line), "[POWER] disabled reason=%s", reason);
  control_loop_log(line);
}

static uint8_t control_loop_runtime_ok(void)
{
  if (foc_runtime_get_state() == FOC_RUNTIME_FAULT)
  {
    const foc_control_output_t *last = foc_runtime_get_last_output();
    const motor_current_sample_t *last_sample = motor_adc_get_latest_sample();
    char fault_line[192];
    /* 传导底层子故障 + 末拍控制快照，避免只有一句通用文案、无法区分
     * 贴轨/模型/控制器/PWM 哪条保护路径动作。 */
    (void)snprintf(fault_line, sizeof(fault_line),
                   "[FAULT] current loop: cause=%s id=%.4f iq=%.4f vd=%.3f vq=%.3f "
                   "raw_u=%u raw_v=%u sat=%u",
                   foc_runtime_fault_text(foc_runtime_get_fault()),
                   (double)last->measured_current_a.d,
                   (double)last->measured_current_a.q,
                   (double)last->voltage_command_v.d,
                   (double)last->voltage_command_v.q,
                   (unsigned)last_sample->phase_u_raw,
                   (unsigned)last_sample->phase_v_raw,
                   (unsigned)last_sample->saturated);
    control_loop_log(fault_line);
    control_loop_enter_fault(CONTROL_LOOP_FAULT_CURRENT_LOOP);
    return 0U;
  }
  return 1U;
}

/* 步骤简称：READY/RUN 行共用；S1 按 PROBE3 绑定状态区分 a/b 段。 */
static const char *control_loop_step_name(void)
{
  switch (control_loop_next_step)
  {
    case TEST_STEP_SIGN:
      return (control_loop_p3_bound != 0U) ? "S1b current self-check" : "S1a sign probe";
    case TEST_STEP_R:           return "S2 resistance";
    case TEST_STEP_L:           return "S3 inductance";
    case TEST_STEP_CURRENT:     return "S4 current step";
    case TEST_STEP_PHASE_SEQ:   return "PHASE_SEQ";
    case TEST_STEP_ENCODER_DIR: return "ENCODER_DIR";
    case TEST_STEP_SENSOR:      return "S5 offset";
    case TEST_STEP_CAPTURE:     return "S7 low-speed capture";
    default:                    return "unknown";
  }
}

static void control_loop_arm_next_step(void)
{
  char line[112];
  control_loop_state = CONTROL_LOOP_STATE_POWER_ARMED;
  control_loop_state_tick = HAL_GetTick();
  if (control_loop_next_step == TEST_STEP_ENCODER_DIR)
  {
    /* 手动判向说明并入 READY 行（0.30rad≈17°，与 CALIB_DIR_MIN_MECH_DELTA_RAD 对齐）。 */
    (void)snprintf(line, sizeof(line),
                   "[READY] ENCODER_DIR: rotate shaft CW by hand (>17 deg); p=confirm x=abort");
  }
  else
  {
    (void)snprintf(line, sizeof(line), "[READY] %s: p=start x=abort", control_loop_step_name());
  }
  control_loop_log(line);
}

static void control_loop_print_result(const char *label, float value, const char *unit)
{
  char line[128];
  (void)snprintf(line, sizeof(line), "[RESULT] %s=%.6f%s", label, (double)value, unit);
  control_loop_log(line);
}

/* S7 收口（SPEC-RUN R1）：行程/力矩方向门禁 -> 块统计终值 -> 候选窗 ->
 * R 自学习四级门 -> v2 候选包。任何一道门失败都不产出 candidate。 */
static void control_loop_judge_s7_and_create_candidate(void)
{
  float dynamic_r = 0.0f;
  float effective_r;
  float window_lo;
  float window_hi;
  float ratio_iqr;
  motor_parameter_store_status_t status;

  /* ---- 门 1：行程/力矩方向（FR-1.3.2）。iq>0 驱动下机械位移应与
   * encoder_direction 同号且达到最小行程；矛盾 => 相序可疑，
   * 禁止产出候选，序列回 PHASE_SEQ 重标。 ---- */
  if ((control_loop_s7_mech_delta *
       (float)control_loop_parameters.rotor.encoder_direction) <
      CALIB_S7_MIN_MECH_TRAVEL_RAD)
  {
    char line[176];
    (void)snprintf(line, sizeof(line),
                   "[S7] torque/travel mismatch: mech_delta=%+.3f rad dir=%+d "
                   "(expect same sign, travel>=%.2f rad); phase sequence suspect",
                   (double)control_loop_s7_mech_delta,
                   (int)control_loop_parameters.rotor.encoder_direction,
                   (double)CALIB_S7_MIN_MECH_TRAVEL_RAD);
    control_loop_log(line);
    control_loop_log("[S7] no candidate; redo PHASE_SEQ (r=restart then p per step)");
    control_loop_phase_seq_verified = 0U;
    control_loop_candidate_ready = 0U;
    control_loop_next_step = TEST_STEP_PHASE_SEQ;
    control_loop_state = CONTROL_LOOP_STATE_TEST_READY;
    return;
  }

  /* ---- 门 2：调节品质（判据 v2，2026-09-13）。建立段后全程窗均值应贴目标——
   * 这是 L4 结案的直接判据：旋转下电流环确实把电流调住了（窗均值滤掉
   * 5ms 台阶激起的 200Hz 振荡混叠），否则不产候选。 ---- */
  {
    float cap_id = (control_loop_s7_cap_wn > 0U)
        ? control_loop_s7_cap_wid_sum / (float)control_loop_s7_cap_wn : 0.0f;
    float cap_iq = (control_loop_s7_cap_wn > 0U)
        ? control_loop_s7_cap_wiq_sum / (float)control_loop_s7_cap_wn : 0.0f;
    char line[176];
    if ((control_loop_s7_cap_wn < CALIB_S7_WRATIO_MIN_WINDOWS) ||
        (cap_id > (CALIB_S7_ID_TARGET_A + CALIB_S7_REG_ID_BAND_A)) ||
        (cap_id < (CALIB_S7_ID_TARGET_A - CALIB_S7_REG_ID_BAND_A)) ||
        (cap_iq < (CALIB_S7_IQ_DRIVE_A * CALIB_S7_REG_IQ_MIN_RATIO)))
    {
      (void)snprintf(line, sizeof(line),
                     "[S7] regulation gate failed: mean wid=%.4f (target %.2f +-%.2f) "
                     "wiq=%.4f (min %.3f) windows=%u",
                     (double)cap_id, (double)CALIB_S7_ID_TARGET_A,
                     (double)CALIB_S7_REG_ID_BAND_A, (double)cap_iq,
                     (double)(CALIB_S7_IQ_DRIVE_A * CALIB_S7_REG_IQ_MIN_RATIO),
                     (unsigned)control_loop_s7_cap_wn);
      control_loop_log(line);
      control_loop_log("[S7] no candidate from this run; r=restart x=stop");
      control_loop_candidate_ready = 0U;
      control_loop_state = CONTROL_LOOP_STATE_CANDIDATE_REVIEW;
      return;
    }
    (void)snprintf(line, sizeof(line),
                   "[RESULT] S7 regulation ok: mean wid=%.4f wiq=%.4f windows=%u",
                   (double)cap_id, (double)cap_iq, (unsigned)control_loop_s7_cap_wn);
    control_loop_log(line);
  }

  /* ---- 门 3：动态 R（判据 v2，EMF-free）。vd = R*id - ωL*iq，交叉项 ~0.01V
   * 可忽略、EMF 只落 q 轴，故每窗 wvd/wid 即阻性比；取全部窗比值的
   * 中位数（抗离群，FR-1.2 精神）对标本次 S2 实测 R 的 [0.5,1.5] 窗。 ---- */
  if (control_loop_s7_wratio_n < CALIB_S7_WRATIO_MIN_WINDOWS)
  {
    char line[160];
    (void)snprintf(line, sizeof(line),
                   "[CANDIDATE] insufficient ratio windows: n=%u (min %u); keeping static parameters",
                   (unsigned)control_loop_s7_wratio_n,
                   (unsigned)CALIB_S7_WRATIO_MIN_WINDOWS);
    control_loop_log(line);
    control_loop_log("[S7] no candidate from this run; r=restart x=stop");
    control_loop_candidate_ready = 0U;
    control_loop_state = CONTROL_LOOP_STATE_CANDIDATE_REVIEW;
    return;
  }
  calib_median_iqr(control_loop_s7_wratio, control_loop_s7_wratio_n,
                   &dynamic_r, &ratio_iqr);
  control_loop_s7_valid = 1U;
  window_lo = control_loop_static_r_ohm * CALIB_S7_DYN_R_LOW_RATIO;
  window_hi = control_loop_static_r_ohm * CALIB_S7_DYN_R_HIGH_RATIO;
  if ((dynamic_r < window_lo) || (dynamic_r > window_hi))
  {
    char line[176];
    (void)snprintf(line, sizeof(line),
                   "[CANDIDATE] dynamic R outside window; discarded: dyn_R=%.4fohm "
                   "windows=%u iqr=%.4f window=[%.4f,%.4f] base_R=%.4fohm",
                   (double)dynamic_r,
                   (unsigned)control_loop_s7_wratio_n, (double)ratio_iqr,
                   (double)window_lo, (double)window_hi,
                   (double)control_loop_static_r_ohm);
    control_loop_log(line);
    control_loop_log("[S7] dynamic R rejected; r=restart x=stop");
    control_loop_candidate_ready = 0U;
    control_loop_state = CONTROL_LOOP_STATE_CANDIDATE_REVIEW;
    return;
  }

  /* ---- 门 4：R 自学习四级门（FR-1.5）。每次上电只采纳首个可信 S2
   * （独立上电语义）；S7 结果同时进双源交叉门。 ---- */
  {
    uint8_t trust = ((control_loop_s2_trusted != 0U) &&
                     (control_loop_s2_adopted == 0U)) ? 1U : 0U;
    uint8_t flags = resistance_learning_update(&control_loop_rlearn_cfg,
                                               &control_loop_rlearn,
                                               control_loop_s2_r_ohm, trust,
                                               dynamic_r, control_loop_s7_valid);
    char line[144];
    if ((flags & RESISTANCE_LEARN_F_ADOPTED) != 0U)
    {
      control_loop_s2_adopted = 1U;
    }
    (void)snprintf(line, sizeof(line),
                   "[RLEARN] flags=0x%02x history=%u learned=%s rev=%lu",
                   (unsigned)flags,
                   (unsigned)control_loop_rlearn.history_count,
                   (control_loop_rlearn.learned_valid != 0U) ? "yes" : "no",
                   (unsigned long)control_loop_rlearn.learned_revision);
    control_loop_log(line);
  }

  /* ---- 组装 v2 候选包（附录 A：相序/换相 + 学习值字段）。 ---- */
  effective_r = resistance_learning_effective_r(&control_loop_rlearn_cfg,
                                                &control_loop_rlearn,
                                                dynamic_r);
  (void)memset(&control_loop_candidate, 0, sizeof(control_loop_candidate));
  control_loop_candidate.format_version = MOTOR_PARAMETER_FORMAT_VERSION_2;
  control_loop_candidate.hardware_id = BOARD_CONFIG_HARDWARE_ID;
  control_loop_candidate.motor_id = BOARD_CONFIG_MOTOR_ID;
  control_loop_candidate.wiring_revision = BOARD_CONFIG_WIRING_REVISION;
  /* revision 链：上电已加载固化包则在其基础上 +1，首次提交为 1。 */
  control_loop_candidate.parameter_revision =
      (control_loop_loaded_valid != 0U)
          ? (control_loop_loaded.parameter_revision + 1UL) : 1UL;
  control_loop_candidate.current_gain_u_a_per_raw =
      BOARD_CONFIG_ADC_REFERENCE_V_NOMINAL / (4095.0f *
      BOARD_CONFIG_PHASE_CURRENT_SHUNT_OHMS * BOARD_CONFIG_PHASE_CURRENT_AMPLIFIER_GAIN);
  control_loop_candidate.current_gain_v_a_per_raw = control_loop_candidate.current_gain_u_a_per_raw;
  /* S1 实测符号，未跑则兜底 +1。 */
  control_loop_candidate.current_sign_u = (control_loop_s1_sign_u != 0) ? control_loop_s1_sign_u : 1;
  control_loop_candidate.current_sign_v = (control_loop_s1_sign_v != 0) ? control_loop_s1_sign_v : 1;
  control_loop_candidate.phase_resistance_ohm = dynamic_r;
  control_loop_candidate.phase_inductance_h = control_loop_static_l_h;
  control_loop_candidate.pole_pairs = control_loop_parameters.rotor.pole_pairs;
  control_loop_candidate.encoder_direction = control_loop_parameters.rotor.encoder_direction;
  control_loop_candidate.electrical_offset_rad = control_loop_parameters.rotor.electrical_offset_rad;
  control_loop_candidate.id_kp = control_loop_parameters.id_pi.kp;
  /* R->PI 传递链改由"经统计门的生效 R"驱动（FR-1.5.5）。 */
  control_loop_candidate.id_ki = effective_r / TEST_PI_TIME_CONSTANT_S;
  control_loop_candidate.iq_kp = control_loop_candidate.id_kp;
  control_loop_candidate.iq_ki = control_loop_candidate.id_ki;
  control_loop_candidate.current_limit_a = TEST_MAX_CURRENT_A;
  control_loop_candidate.voltage_limit_v = TEST_MAX_VOLTAGE_V;
  control_loop_candidate.state = MOTOR_PARAMETER_STATE_DYNAMIC_CANDIDATE;
  /* v2 新增：换相与相序验证结论（FR-1.3）。 */
  control_loop_candidate.phase_map_a = control_loop_parameters.phase_map.phase_a_output;
  control_loop_candidate.phase_map_b = control_loop_parameters.phase_map.phase_b_output;
  control_loop_candidate.phase_map_c = control_loop_parameters.phase_map.phase_c_output;
  control_loop_candidate.phase_seq_verified = control_loop_phase_seq_verified;
  /* v2 新增：R 学习状态随包固化跨启动（FR-1.5.2）。 */
  resistance_learning_store(&control_loop_rlearn,
                            &control_loop_candidate.learned_r_count,
                            control_loop_candidate.learned_r_history_ohm,
                            &control_loop_candidate.learned_r_valid,
                            &control_loop_candidate.learned_phase_resistance_ohm,
                            &control_loop_candidate.learned_r_revision);

  status = motor_parameter_store_stage_candidate(&control_loop_candidate);
  control_loop_candidate_ready = (status == MOTOR_PARAMETER_STORE_OK) ? 1U : 0U;
  {
    char line[160];
    (void)snprintf(line, sizeof(line),
                   "[RESULT] dynamic_R_daxis=%.6fohm windows=%u iqr=%.4f window=[%.4f,%.4f]",
                   (double)dynamic_r,
                   (unsigned)control_loop_s7_wratio_n, (double)ratio_iqr,
                   (double)window_lo, (double)window_hi);
    control_loop_log(line);
  }
  if (control_loop_candidate_ready != 0U)
  {
    control_loop_log((control_loop_phase_seq_verified != 0U)
        ? "[NEXT] a=apply (flash) r=repeat d=discard x=stop"
        : "[NEXT] a=apply (RAM only: phase-seq unverified) r=repeat d=discard x=stop");
  }
  else
  {
    control_loop_log("[CANDIDATE] stage failed; r=repeat d=discard x=stop");
  }
  control_loop_state = CONTROL_LOOP_STATE_CANDIDATE_REVIEW;
}

void control_loop_init(void)
{
  control_loop_safe_disable();
  control_loop_fault = CONTROL_LOOP_FAULT_NONE;
  control_loop_candidate_ready = 0U;
  control_loop_test_precheck_passed = 0U;
  control_loop_limits_printed = 0U;
  control_loop_static_r_ohm = TEST_NOMINAL_R_OHM;
  control_loop_static_l_h = TEST_NOMINAL_L_H;
  control_loop_s1_sign_u = 1;
  control_loop_s1_sign_v = 1;
  control_loop_next_step = TEST_STEP_SIGN;
  control_loop_reset_measurements();
  /* SPEC-RUN R1/R2 上电一次性的状态（r=restart 不清，保证"独立上电"语义）：
   * S7 块统计配置、R 自学习（上电后由 control_loop_load_committed_parameters
   * 从参数包 v2 重建）、S2/S7 本次结果、相序验证结论、注入包。 */
  control_loop_s7_wratio_n = 0U;
  control_loop_s7_cap_wid_sum = 0.0f;
  control_loop_s7_cap_wiq_sum = 0.0f;
  control_loop_s7_cap_wn = 0U;
  control_loop_rlearn_cfg.history_n = CALIB_RLEARN_HISTORY_N;
  control_loop_rlearn_cfg.repeat_dev_max = CALIB_RLEARN_REPEAT_DEV_MAX;
  control_loop_rlearn_cfg.cross_dev_max = CALIB_RLEARN_CROSS_DEV_MAX;
  control_loop_rlearn_cfg.guard_ratio = CALIB_RLEARN_GUARD_RATIO;
  control_loop_rlearn_cfg.nominal_r_ohm = TEST_NOMINAL_R_OHM;
  resistance_learning_reset(&control_loop_rlearn);
  control_loop_s2_r_ohm = 0.0f;
  control_loop_s2_trusted = 0U;
  control_loop_s2_adopted = 0U;
  control_loop_s7_valid = 0U;
  control_loop_s7_mech_prev = 0.0f;
  control_loop_s7_mech_delta = 0.0f;
  control_loop_s7_los_ms = 0U;
  control_loop_s7_dbg_ms = 0U;
  control_loop_s7_wid_sum = 0.0f;
  control_loop_s7_wiq_sum = 0.0f;
  control_loop_s7_wn = 0U;
  control_loop_s7_wvd_sum = 0.0f;
  control_loop_s7_wvq_sum = 0.0f;
  control_loop_s7_vn = 0U;
  control_loop_s7_wratio_n = 0U;
  control_loop_s7_cap_wid_sum = 0.0f;
  control_loop_s7_cap_wiq_sum = 0.0f;
  control_loop_s7_cap_wn = 0U;
  control_loop_s5_ok = 0U;
  control_loop_phase_seq_verified = 0U;
  control_loop_loaded_valid = 0U;
  control_loop_state = CONTROL_LOOP_STATE_TEST_BOOT;
  control_loop_state_tick = HAL_GetTick();
  control_loop_log("[TEST] automatic identification mode; power remains disabled");
}

control_loop_command_status_t control_loop_request_open_loop_start(void)
{
  return CONTROL_LOOP_COMMAND_REJECTED_STATE;
}

control_loop_command_status_t control_loop_request_identification_start(void)
{
  if (control_loop_state == CONTROL_LOOP_STATE_FAULT)
  {
    return CONTROL_LOOP_COMMAND_REJECTED_FAULT;
  }
  control_loop_safe_disable();
  control_loop_candidate_ready = 0U;
  control_loop_test_precheck_passed = 0U;
  control_loop_limits_printed = 0U;
  control_loop_static_r_ohm = TEST_NOMINAL_R_OHM;
  control_loop_static_l_h = TEST_NOMINAL_L_H;
  control_loop_s1_sign_u = 1;
  control_loop_s1_sign_v = 1;
  control_loop_next_step = TEST_STEP_SIGN;
  control_loop_reset_measurements();
  control_loop_state = CONTROL_LOOP_STATE_TEST_BOOT;
  control_loop_state_tick = HAL_GetTick();
  control_loop_log("[TEST] restarting automatic identification sequence");
  return CONTROL_LOOP_COMMAND_ACCEPTED;
}

control_loop_command_status_t control_loop_request_power_confirm(void)
{
  if (control_loop_state != CONTROL_LOOP_STATE_POWER_ARMED)
  {
    return CONTROL_LOOP_COMMAND_REJECTED_STATE;
  }

  if (control_loop_limits_printed == 0U)
  {
    control_loop_log_limits();
    control_loop_limits_printed = 1U;
  }
  switch (control_loop_next_step)
  {
    case TEST_STEP_SIGN:
    {
      float s1_a_per_raw;
#if CALIB_S1A_PROBE3
      if (control_loop_p3_bound != 0U)
      {
        /* PROBE3 已绑定采样符号：再按 p → 以 CURRENT 启动进 S1b 闭环自证（不 reset，保留绑定符号）。 */
        if (control_loop_start_power(CALIB_S1_ALIGN_CURRENT_A, 0.0f) == 0U)
          return CONTROL_LOOP_COMMAND_REJECTED_CURRENT_NOT_READY;
        control_loop_begin_step(CALIB_S1_ALIGN_CURRENT_A,
                                CALIB_S1_SETTLE_BAND_A, CALIB_COLLECT_N,
                                CALIB_S1B_SPREAD_MAX_A);
        control_loop_sign_hb_ms = 0U;
        control_loop_s1b_iq_sum = 0.0f;
        control_loop_s1b_iq_n = 0U;
        control_loop_s1_sub = S1B_CLOSED_LOOP;
        control_loop_s1_phase_tick = HAL_GetTick();
        control_loop_state = CONTROL_LOOP_STATE_SIGN_CALIBRATION;
        break;
      }
#endif
      control_loop_reset_measurements();
      if (control_loop_start_openloop_power() == 0U) return CONTROL_LOOP_COMMAND_REJECTED_CURRENT_NOT_READY;
      /* 标准增益：1 code 对应安培数；目标带/硬电流门限统一由 Imax 比例派生，改 Imax 全联动。 */
      s1_a_per_raw = BOARD_CONFIG_ADC_REFERENCE_V_NOMINAL /
                     (4095.0f * BOARD_CONFIG_PHASE_CURRENT_SHUNT_OHMS *
                      BOARD_CONFIG_PHASE_CURRENT_AMPLIFIER_GAIN);
      control_loop_s1_tlo_code  = (TEST_MAX_CURRENT_A * CALIB_S1A_TARGET_LO_RATIO) / s1_a_per_raw;
      control_loop_s1_hard_code = (TEST_MAX_CURRENT_A * CALIB_S1A_HARD_I_RATIO) / s1_a_per_raw;
      control_loop_sign_hb_ms = 0U;
#if CALIB_S1A_PROBE3
      /* PROBE3 首次：从角度 0 的零矢量基线开始（start_openloop 已锁 θ=0、Vd=0）。 */
      control_loop_p3_k = 0U;
      control_loop_s1_sub = S1A_P3_BASELINE;
#elif CALIB_S1A_MANUAL_HOLD
      /* 诊断：进入持续对齐保持，不跑档位引擎/不判据，由 SIGN_CALIBRATION 轮询持续喂固定 Vd。 */
      control_loop_s1_sub = S1A_HOLD_ALIGN;
#else
      control_loop_s1_sub = S1A_BASELINE;
#endif
      control_loop_s1_phase_tick = HAL_GetTick();
      control_loop_state = CONTROL_LOOP_STATE_SIGN_CALIBRATION;
      break;
    }
    case TEST_STEP_R:
      control_loop_s2_level = 0U;
      control_loop_s2_id_sum = 0.0f;
      control_loop_s2_vd_sum = 0.0f;
      control_loop_s2_n = 0U;
      if (control_loop_start_power(control_loop_s2_levels[0], 0.0f) == 0U)
        return CONTROL_LOOP_COMMAND_REJECTED_CURRENT_NOT_READY;
      control_loop_begin_step(control_loop_s2_levels[0], CALIB_S2_SETTLE_BAND_A,
                              CALIB_COLLECT_N, CALIB_S2_SPREAD_MAX_A);
      control_loop_state = CONTROL_LOOP_STATE_R_IDENTIFICATION;
      break;
    case TEST_STEP_L:
      control_loop_s3_step_issued = 0U;
      if (control_loop_start_power(CALIB_S3_STEP_FROM_A, 0.0f) == 0U)
        return CONTROL_LOOP_COMMAND_REJECTED_CURRENT_NOT_READY;
      control_loop_begin_step(CALIB_S3_STEP_FROM_A, CALIB_S2_SETTLE_BAND_A,
                              CALIB_COLLECT_N, CALIB_S2_SPREAD_MAX_A);
      control_loop_state = CONTROL_LOOP_STATE_L_IDENTIFICATION;
      break;
    case TEST_STEP_CURRENT:
      /* 对齐 S1b 已验证启动时序：start_power 的 configure 已提供全新 PI，且以参数给定
       * FROM(=0) 作为目标；不再于功率/ISR 运行后再 reset_controller + 事后改目标，
       * 避免 ISR 活着时改写控制量造成“抢跑”瞬态。先让 ISR 在 FROM 建立一段固定时长，
       * 进入 CURRENT_VALIDATE 后再阶跃到 TO，观测窗口从阶跃时刻起算（见处理器）。 */
      if (control_loop_start_power(CALIB_S4_STEP_FROM_A, 0.0f) == 0U) return CONTROL_LOOP_COMMAND_REJECTED_CURRENT_NOT_READY;
      control_loop_s4_step_pending = 1U;
      control_loop_s4_establish_ms = HAL_GetTick();
      control_loop_s4_hb_ms = 0U;
      control_loop_state = CONTROL_LOOP_STATE_TEST_CURRENT_VALIDATE;
      break;
    case TEST_STEP_PHASE_SEQ:
      control_loop_pseq_idx = 0U;
      control_loop_pseq_sub = PSEQ_PREP;
      control_loop_pseq_tick = HAL_GetTick();
      control_loop_state = CONTROL_LOOP_STATE_PHASE_SEQ_CALIBRATION;
      break;
    case TEST_STEP_ENCODER_DIR:
      {
        const encoder_cache_sample_t *enc0 = encoder_cache_get_latest();
        control_loop_dir_mech_prev = foc_mechanical_raw_to_rad(enc0->raw_angle);
      }
      control_loop_dir_sub = DIR_MANUAL_WAIT;
      control_loop_dir_tick = HAL_GetTick();
      control_loop_dir_mech_delta = 0.0f;
      control_loop_dir_hb_ms = 0U;
      control_loop_state = CONTROL_LOOP_STATE_ENCODER_DIR_CALIBRATION;
      break;
    case TEST_STEP_SENSOR:
      if (control_loop_start_power(CALIB_S5_ALIGN_CURRENT_A, 0.0f) == 0U) return CONTROL_LOOP_COMMAND_REJECTED_CURRENT_NOT_READY;
      control_loop_s5_sub = S5_ALIGN;
      control_loop_s5_idx = 0U;
      control_loop_s5_tick = HAL_GetTick();
      control_loop_state = CONTROL_LOOP_STATE_SENSOR_CALIBRATION;
      break;
    case TEST_STEP_CAPTURE:
    {
      const encoder_cache_sample_t *enc7 = encoder_cache_get_latest();
      /* 入口门禁：S5 偏置不可信时角度链未证实，加功率必失步->贴轨，直接拒绝。 */
      if (control_loop_s5_ok == 0U)
      {
        control_loop_log("[S7] entry rejected: S5 offset not trusted (ok=0); rerun S5 (r=restart)");
        return CONTROL_LOOP_COMMAND_REJECTED_STATE;
      }
      control_loop_reset_measurements();
      /* S7 判据 v2 统计复位：窗累计/窗比值/全程均值清零；行程跟踪起点（FR-1.3）。 */
      control_loop_s7_valid = 0U;
      control_loop_s7_los_ms = 0U;
      control_loop_s7_dbg_ms = 0U;
      control_loop_s7_wid_sum = 0.0f;
      control_loop_s7_wiq_sum = 0.0f;
      control_loop_s7_wn = 0U;
      control_loop_s7_wvd_sum = 0.0f;
      control_loop_s7_wvq_sum = 0.0f;
      control_loop_s7_vn = 0U;
      control_loop_s7_wratio_n = 0U;
      control_loop_s7_cap_wid_sum = 0.0f;
      control_loop_s7_cap_wiq_sum = 0.0f;
      control_loop_s7_cap_wn = 0U;
      control_loop_s7_mech_prev = foc_mechanical_raw_to_rad(enc7->raw_angle);
      control_loop_s7_mech_delta = 0.0f;
      /* 相序镜像未裁决 = 换相可能反向，加转矩即堵转、PI 顶限压拉垮母线
       *（2026-09-12 S7 硬发散实测；勘误后主因虽定位于测量侧 phase_map 不对称，
       * 错误映射下 S7 会发散仍是既证事实），硬拦不放行；重跑 PHASE_SEQ 拿到
       * verified 再来。规格偏差已在 DD-01 §6.13 登记。 */
      if (control_loop_phase_seq_verified == 0U)
      {
        control_loop_log("[S7] entry rejected: phase-seq mirror not verified; rerun PHASE_SEQ (r=restart)");
        return CONTROL_LOOP_COMMAND_REJECTED_STATE;
      }
      if (control_loop_start_power(CALIB_S7_ID_TARGET_A, CALIB_S7_IQ_DRIVE_A) == 0U) return CONTROL_LOOP_COMMAND_REJECTED_CURRENT_NOT_READY;
      control_loop_state = CONTROL_LOOP_STATE_LOW_SPEED_CAPTURE;
      break;
    }
    default:
      control_loop_log((control_loop_candidate_ready != 0U)
          ? "[DONE] identification complete; a=apply d=discard r=restart x=stop"
          : "[DONE] identification complete; r=restart x=stop");
      return CONTROL_LOOP_COMMAND_REJECTED_STATE;
  }
  control_loop_state_tick = HAL_GetTick();
  return CONTROL_LOOP_COMMAND_ACCEPTED;
}

control_loop_command_status_t control_loop_request_candidate_apply(void)
{
  motor_parameter_store_status_t status;

  if ((control_loop_state != CONTROL_LOOP_STATE_CANDIDATE_REVIEW) ||
      (control_loop_candidate_ready == 0U))
  {
    return CONTROL_LOOP_COMMAND_REJECTED_STATE;
  }
  /* RAM 先激活：无论 Flash 结果如何，候选 PI 参数本次启动生效。 */
  control_loop_parameters.id_pi.ki = control_loop_candidate.id_ki;
  control_loop_parameters.iq_pi.ki = control_loop_candidate.iq_ki;
  if (control_loop_candidate.phase_seq_verified != 0U)
  {
    /* FR-1.3：相序经 +90° 镜像验证的候选才允许永久固化（FR-2.2 commit：
     * 擦非活跃页 -> 写 -> 读回校验 -> 切换，任一步失败返回明确状态码）。 */
    status = motor_parameter_store_commit_candidate(1U);
    if (status == MOTOR_PARAMETER_STORE_OK)
    {
      char line[160];
      /* 同步"已加载包"镜像：后续 r=restart 的测试起点与 revision 链以此为准。 */
      if (motor_parameter_store_has_active() != 0U)
      {
        control_loop_loaded = *motor_parameter_store_get_active();
        control_loop_loaded_valid = 1U;
      }
      (void)snprintf(line, sizeof(line),
                     "[APPLY] committed to flash (source page %s); rev=%lu",
                     motor_parameter_store_get_source_page_text(),
                     (unsigned long)control_loop_candidate.parameter_revision);
      control_loop_log(line);
    }
    else
    {
      /* FR-2.2 失败降级：报具体状态码，候选仅 RAM 激活（本次启动生效）。 */
      char line[176];
      (void)motor_parameter_store_activate_ram_candidate();
      (void)snprintf(line, sizeof(line),
                     "[APPLY] flash commit failed (%s); RAM-only for this boot",
                     motor_parameter_store_status_text(status));
      control_loop_log(line);
    }
  }
  else
  {
    /* FR-1.3：未验证相序的候选不进 Flash，仅 RAM 激活。 */
    (void)motor_parameter_store_activate_ram_candidate();
    control_loop_log("[APPLY] phase-seq unverified; RAM-only for this boot (not persisted)");
  }
  control_loop_candidate_ready = 0U;
  control_loop_state = CONTROL_LOOP_STATE_TEST_READY;
  control_loop_next_step = TEST_STEP_DONE;
  return CONTROL_LOOP_COMMAND_ACCEPTED;
}

control_loop_command_status_t control_loop_request_candidate_discard(void)
{
  if (control_loop_state != CONTROL_LOOP_STATE_CANDIDATE_REVIEW)
  {
    return CONTROL_LOOP_COMMAND_REJECTED_STATE;
  }
  control_loop_candidate_ready = 0U;
  control_loop_next_step = TEST_STEP_DONE;
  control_loop_state = CONTROL_LOOP_STATE_TEST_READY;
  control_loop_log("[CANDIDATE] discarded; r=restart x=stop");
  return CONTROL_LOOP_COMMAND_ACCEPTED;
}

void control_loop_request_stop(void)
{
  control_loop_finish_power("OPERATOR_STOP");
  control_loop_state = CONTROL_LOOP_STATE_SAFE_IDLE;
  control_loop_log("[SAFE] stopped; r=restart test");
}

control_loop_command_status_t control_loop_request_fault_clear(void)
{
  if (control_loop_state != CONTROL_LOOP_STATE_FAULT)
  {
    return CONTROL_LOOP_COMMAND_REJECTED_STATE;
  }
  control_loop_safe_disable();
  control_loop_fault = CONTROL_LOOP_FAULT_NONE;
  control_loop_state = CONTROL_LOOP_STATE_TEST_BOOT;
  control_loop_state_tick = HAL_GetTick();
  return CONTROL_LOOP_COMMAND_ACCEPTED;
}

control_loop_command_status_t control_loop_request_clear_storage(void)
{
  motor_parameter_store_status_t status;
  uint8_t in_silent_state;

  /* FR-2.5：只在静默态允许清参数存储（SAFE_IDLE/TEST_BOOT/TEST_READY/
   * CANDIDATE_REVIEW）；FAULT 态的 c= 已由 app_baseline 分流去清故障，
   * 运行中的标定/功率态一律拒绝。 */
  in_silent_state = 0U;
  if ((control_loop_state == CONTROL_LOOP_STATE_SAFE_IDLE) ||
      (control_loop_state == CONTROL_LOOP_STATE_TEST_BOOT) ||
      (control_loop_state == CONTROL_LOOP_STATE_TEST_READY) ||
      (control_loop_state == CONTROL_LOOP_STATE_CANDIDATE_REVIEW))
  {
    in_silent_state = 1U;
  }
  if (in_silent_state == 0U)
  {
    return CONTROL_LOOP_COMMAND_REJECTED_STATE;
  }
  status = motor_parameter_store_clear_storage(1U);
  if (status != MOTOR_PARAMETER_STORE_OK)
  {
    char line[160];
    (void)snprintf(line, sizeof(line), "[CLEAR] storage erase failed (%s)",
                   motor_parameter_store_status_text(status));
    control_loop_log(line);
    return CONTROL_LOOP_COMMAND_REJECTED_STORAGE;
  }
  /* RAM 侧同步失效：学习值回兜底、注入包失效、标称 R/L 复位（等同未标定）。
   * 注意这里与 r=restart 相反——c=clear 的语义就是抹掉跨启动状态。 */
  resistance_learning_reset(&control_loop_rlearn);
  control_loop_s2_adopted = 0U;
  control_loop_loaded_valid = 0U;
  control_loop_phase_seq_verified = 0U;
  control_loop_static_r_ohm = TEST_NOMINAL_R_OHM;
  control_loop_static_l_h = TEST_NOMINAL_L_H;
  control_loop_parameters.id_pi.ki = TEST_NOMINAL_R_OHM / TEST_PI_TIME_CONSTANT_S;
  control_loop_parameters.iq_pi.ki = control_loop_parameters.id_pi.ki;
  control_loop_candidate_ready = 0U;
  control_loop_state = CONTROL_LOOP_STATE_TEST_BOOT;
  control_loop_next_step = TEST_STEP_SIGN;
  control_loop_state_tick = HAL_GetTick();
  control_loop_log("[CLEAR] parameter storage erased (both pages); uncalibrated now; r=restart test");
  return CONTROL_LOOP_COMMAND_ACCEPTED;
}

/* ============ VJ 电压注入判决模式（L4 诊断）：请求入口 ============ */

control_loop_command_status_t control_loop_request_vj_start(void)
{
  motor_adc_current_diagnostic_t diagnostic;
  uint8_t silent;
  char line[192];

  /* 与 c=clear 同一静默态白名单：运行中/FAULT 一律拒绝。 */
  silent = 0U;
  if ((control_loop_state == CONTROL_LOOP_STATE_SAFE_IDLE) ||
      (control_loop_state == CONTROL_LOOP_STATE_TEST_BOOT) ||
      (control_loop_state == CONTROL_LOOP_STATE_TEST_READY) ||
      (control_loop_state == CONTROL_LOOP_STATE_POWER_ARMED) ||
      (control_loop_state == CONTROL_LOOP_STATE_CANDIDATE_REVIEW))
  {
    silent = 1U;
  }
  if (silent == 0U)
  {
    return CONTROL_LOOP_COMMAND_REJECTED_STATE;
  }
  /* 本靴 S0 预检须已通过（保证参数集/增益至少配置过一次，TEST_BOOT 首拍前拒绝）。 */
  if (control_loop_test_precheck_passed == 0U)
  {
    return CONTROL_LOOP_COMMAND_REJECTED_CURRENT_NOT_READY;
  }
  /* 门禁与 S0 同源：零偏质量 + 编码器在线；转子/换相/符号沿用当前 RAM 配置
   * （判决对象就是"S7 将要用的这套链"），入口打印生效配置供解读。 */
  motor_adc_get_current_diagnostic(&diagnostic);
  if ((diagnostic.adc_hardware_calibrated == 0U) ||
      (diagnostic.zero.quality != MOTOR_ADC_ZERO_QUALITY_OK))
  {
    control_loop_log("[VJ] ADC zero check failed");
    return CONTROL_LOOP_COMMAND_REJECTED_CURRENT_NOT_READY;
  }
  if (encoder_cache_is_valid(TEST_ENCODER_BOOT_MAX_AGE_MS) == 0U)
  {
    control_loop_log("[VJ] encoder not online");
    return CONTROL_LOOP_COMMAND_REJECTED_CURRENT_NOT_READY;
  }
  control_loop_vj_sub = VJ_SUB_IDLE;
  control_loop_vj_vd = 0.0f;
  control_loop_vj_vq = 0.0f;
  control_loop_vj_theta = 0.0f;
  control_loop_vj_enc_bad_ms = 0U;
  control_loop_state = CONTROL_LOOP_STATE_VJ_DIAG;
  control_loop_state_tick = HAL_GetTick();
  (void)snprintf(line, sizeof(line),
                 "[VJ] diag armed: dir=%+d pp=%u offset=%+.4f map=%u/%u/%u sign=%d/%d R=%.3f",
                 (int)control_loop_parameters.rotor.encoder_direction,
                 (unsigned)control_loop_parameters.rotor.pole_pairs,
                 (double)control_loop_parameters.rotor.electrical_offset_rad,
                 (unsigned)control_loop_parameters.phase_map.phase_a_output,
                 (unsigned)control_loop_parameters.phase_map.phase_b_output,
                 (unsigned)control_loop_parameters.phase_map.phase_c_output,
                 (int)control_loop_parameters.current.channel_u_sign,
                 (int)control_loop_parameters.current.channel_v_sign,
                 (double)control_loop_static_r_ohm);
  control_loop_log(line);
  control_loop_log("[READY] VJ diag: p=power-on x=abort (d/e=+-Vd q/w=+-Vq g=rotate)");
  return CONTROL_LOOP_COMMAND_ACCEPTED;
}

control_loop_command_status_t control_loop_request_vj_power(void)
{
  if ((control_loop_state != CONTROL_LOOP_STATE_VJ_DIAG) ||
      (control_loop_vj_sub != VJ_SUB_IDLE))
  {
    return CONTROL_LOOP_COMMAND_REJECTED_STATE;
  }
  foc_runtime_set_test_mode(1U);
  if (control_loop_start_openloop_power() == 0U)
  {
    return CONTROL_LOOP_COMMAND_REJECTED_CURRENT_NOT_READY; /* 内部已 enter_fault */
  }
  control_loop_log("[VJ] power on: VOLTAGE openloop, encoder angle");
  control_loop_vj_sub = VJ_SUB_HOLD;
  control_loop_vj_tick = HAL_GetTick();
  control_loop_vj_enc_bad_ms = 0U;
  return CONTROL_LOOP_COMMAND_ACCEPTED;
}

control_loop_command_status_t control_loop_request_vj_inject(uint8_t axis_q, float sign)
{
  float amplitude;

  if ((control_loop_state != CONTROL_LOOP_STATE_VJ_DIAG) ||
      (control_loop_vj_sub != VJ_SUB_HOLD))
  {
    return CONTROL_LOOP_COMMAND_REJECTED_STATE;
  }
  amplitude = CALIB_VJ_INJECT_V * ((sign < 0.0f) ? -1.0f : 1.0f);
  if (axis_q == 0U)
  {
    control_loop_vj_vd = amplitude;
    control_loop_vj_vq = 0.0f;
  }
  else
  {
    control_loop_vj_vd = 0.0f;
    control_loop_vj_vq = amplitude;
  }
  control_loop_vj_axis_q = axis_q;
  control_loop_vj_id_sum = 0.0f;
  control_loop_vj_iq_sum = 0.0f;
  control_loop_vj_du_sum = 0.0f;
  control_loop_vj_dv_sum = 0.0f;
  control_loop_vj_n = 0U;
  control_loop_vj_sub = VJ_SUB_PULSE;
  control_loop_vj_tick = HAL_GetTick();
  return CONTROL_LOOP_COMMAND_ACCEPTED;
}

control_loop_command_status_t control_loop_request_vj_sweep(void)
{
  if ((control_loop_state != CONTROL_LOOP_STATE_VJ_DIAG) ||
      (control_loop_vj_sub != VJ_SUB_HOLD))
  {
    return CONTROL_LOOP_COMMAND_REJECTED_STATE;
  }
  control_loop_vj_vd = 0.0f;
  control_loop_vj_vq = CALIB_VJ_INJECT_V;
  control_loop_vj_id_sum = 0.0f;
  control_loop_vj_iq_sum = 0.0f;
  control_loop_vj_n = 0U;
  control_loop_vj_tlm_ms = 0U;
  control_loop_vj_sub = VJ_SUB_SWEEP;
  control_loop_vj_tick = HAL_GetTick();
  control_loop_log("[VJT] sweep start: vq=+0.30V, 50ms telemetry");
  return CONTROL_LOOP_COMMAND_ACCEPTED;
}

control_loop_command_status_t control_loop_request_vj_exit(void)
{
  if (control_loop_state != CONTROL_LOOP_STATE_VJ_DIAG)
  {
    return CONTROL_LOOP_COMMAND_REJECTED_STATE;
  }
  control_loop_safe_disable();
  control_loop_vj_sub = VJ_SUB_IDLE;
  control_loop_vj_vd = 0.0f;
  control_loop_vj_vq = 0.0f;
  /* 回 TEST_READY：序列位置（next_step）原样保留，poll 会重新 [READY] 下一步。 */
  control_loop_state = CONTROL_LOOP_STATE_TEST_READY;
  control_loop_state_tick = HAL_GetTick();
  control_loop_log("[VJ] exit; back to sequence");
  return CONTROL_LOOP_COMMAND_ACCEPTED;
}

void control_loop_load_committed_parameters(const motor_parameter_package_t *package)
{
  float effective_r;

  if (package == (const motor_parameter_package_t *)0)
  {
    return;
  }
  /* FR-2.4：PARAMETER_CHECK 通过后注入。先存镜像 + 从 v2 字段重建 R 学习状态
   * （rlearn_cfg 在 control_loop_init 已配好），再灌入控制参数集。
   * phase_map 刻意不写入参数集：P3 恒等契约（2026-09-13）——S1a–S4 必须恒等、
   * PHASE_SEQ 每靴重探覆写；此处写入会在 boot 后不经 r 直接 p 的流程污染 S1a
   * （通道交叉假故障）。运行态取 map 走 loaded 镜像/固化包，不经测试参数集。 */
  control_loop_loaded = *package;
  control_loop_loaded_valid = 1U;
  resistance_learning_load(&control_loop_rlearn,
                           package->learned_r_count,
                           package->learned_r_history_ohm,
                           package->learned_r_valid,
                           package->learned_phase_resistance_ohm,
                           package->learned_r_revision);
  control_loop_parameters.rotor.pole_pairs = package->pole_pairs;
  control_loop_parameters.rotor.encoder_direction = package->encoder_direction;
  control_loop_parameters.rotor.electrical_offset_rad = package->electrical_offset_rad;
  control_loop_parameters.current.channel_u_sign = (float)package->current_sign_u;
  control_loop_parameters.current.channel_v_sign = (float)package->current_sign_v;
  control_loop_static_r_ohm = package->phase_resistance_ohm;
  control_loop_static_l_h = package->phase_inductance_h;
  control_loop_phase_seq_verified = package->phase_seq_verified;
  effective_r = resistance_learning_effective_r(&control_loop_rlearn_cfg,
                                                &control_loop_rlearn,
                                                package->phase_resistance_ohm);
  control_loop_parameters.id_pi.ki = effective_r / TEST_PI_TIME_CONSTANT_S;
  control_loop_parameters.iq_pi.ki = control_loop_parameters.id_pi.ki;
  {
    char line[192];
    (void)snprintf(line, sizeof(line),
                   "[LOAD] parameter package injected (page %s rev=%lu): R=%.3fohm L=%.2fuH "
                   "offset=%+.4frad dir=%+d phase_seq=%u learned_R=%s(%.3f rev=%lu)",
                   motor_parameter_store_get_source_page_text(),
                   (unsigned long)package->parameter_revision,
                   (double)package->phase_resistance_ohm,
                   (double)(package->phase_inductance_h * 1e6f),
                   (double)package->electrical_offset_rad,
                   (int)package->encoder_direction,
                   (unsigned)package->phase_seq_verified,
                   (package->learned_r_valid != 0U) ? "yes" : "no",
                   (double)package->learned_phase_resistance_ohm,
                   (unsigned long)package->learned_r_revision);
    control_loop_log(line);
  }
}

void control_loop_poll(void)
{
  uint32_t elapsed;
  uint32_t now_ms;
  const foc_control_output_t *output;
  calib_step_phase_t phase;
  /* FR1：本拍 10kHz 快路径样本均值；无样本时回退瞬时 last_output。
   * 标定判定（S1b/S2/S3/S4）一律吃 judge_id/judge_iq；保护路径不在此列。 */
  foc_dq_t tick_mean;
  uint16_t tick_samples = 0U;
  uint8_t have_tick_mean;
  float judge_id;
  float judge_iq;

  now_ms = HAL_GetTick();
  elapsed = (uint32_t)(now_ms - control_loop_state_tick);
  /* 主循环心跳：仅状态变化时打印，稳态不刷屏。 */
  if ((uint32_t)(now_ms - control_loop_last_hb_ms) >= 500U)
  {
    control_loop_state_t cur_state = control_loop_get_state();
    if (cur_state != control_loop_last_hb_state)
    {
      char hb_line[64];
      control_loop_last_hb_ms = now_ms;
      control_loop_last_hb_state = cur_state;
      (void)snprintf(hb_line, sizeof(hb_line), "[HBT] state=%s",
                     control_loop_state_text(cur_state));
      control_loop_log(hb_line);
    }
  }
  output = foc_runtime_get_last_output();
  have_tick_mean = foc_runtime_consume_tick_mean(&tick_mean, &tick_samples);
  if (have_tick_mean != 0U)
  {
    judge_id = tick_mean.d;
    judge_iq = tick_mean.q;
  }
  else
  {
    judge_id = output->measured_current_a.d;
    judge_iq = output->measured_current_a.q;
  }

  switch (control_loop_state)
  {
    case CONTROL_LOOP_STATE_TEST_BOOT:
      control_loop_test_precheck_passed = control_loop_prepare_test_parameters();
      if (control_loop_test_precheck_passed == 0U)
      {
        control_loop_state = CONTROL_LOOP_STATE_TEST_READY;
        control_loop_log("[TEST] fix ADC zero / encoder, then press r to retry");
        break;
      }
      control_loop_state = CONTROL_LOOP_STATE_TEST_READY;
      control_loop_log("[TEST] S0 passed: zero + encoder ok; nominal R/L and PI loaded");
      break;

    case CONTROL_LOOP_STATE_TEST_READY:
      if (control_loop_test_precheck_passed == 0U)
      {
        break;
      }
      if (control_loop_next_step == TEST_STEP_DONE)
      {
        break; /* 防御性终态：A 组 S4 后改设 ENCODER_DIR 继续 B 组，正常不会走到这里 */
      }
      control_loop_arm_next_step();
      break;

    case CONTROL_LOOP_STATE_SIGN_CALIBRATION:
      if (control_loop_runtime_ok() == 0U) break;

      if ((control_loop_s1_sub == S1A_BASELINE) || (control_loop_s1_sub == S1A_SWEEP) ||
          (control_loop_s1_sub == S1A_DECAY) || (control_loop_s1_sub == S1A_EVAL) ||
          (control_loop_s1_sub == S1A_HOLD_ALIGN) ||
          (control_loop_s1_sub == S1A_P3_BASELINE) || (control_loop_s1_sub == S1A_P3_INJECT) ||
          (control_loop_s1_sub == S1A_P3_DECAY) || (control_loop_s1_sub == S1A_P3_EVAL))
      {
        /* ---------- S1a：开环电压爬坡扫描（慢路径推进，原始 raw + 线性拟合判定） ---------- */
        uint32_t s1_elapsed = (uint32_t)(now_ms - control_loop_s1_phase_tick);
        const motor_current_sample_t *s1_raw = motor_adc_get_latest_sample();
        uint8_t s1_good = ((s1_raw->valid != 0U) && (s1_raw->saturated == 0U) &&
                           (s1_raw->timeout == 0U)) ? 1U : 0U;

        /* S1a 总看门狗，防止子状态机意外卡死；手动保持模式要无限持续，豁免。 */
        if ((control_loop_s1_sub != S1A_HOLD_ALIGN) &&
            ((uint32_t)(now_ms - control_loop_state_tick) > CALIB_S1A_TOTAL_TIMEOUT_MS))
        {
          control_loop_log("[FAULT] S1a ramp sweep timed out");
          control_loop_enter_fault(CONTROL_LOOP_FAULT_NO_CONVERGE);
          break;
        }

        switch (control_loop_s1_sub)
        {
          case S1A_HOLD_ALIGN:
          {
            /* 诊断：锁 θ=0 持续注入固定 Vd，不推进/不判据；每 500ms 报一次实测电流，按 x 退出。 */
            char hold_line[112];
            foc_runtime_set_voltage_target(CALIB_S1A_HOLD_VOLTAGE_V, 0.0f);
            if ((uint32_t)(now_ms - control_loop_sign_hb_ms) >= 500U)
            {
              control_loop_sign_hb_ms = now_ms;
              (void)snprintf(hold_line, sizeof(hold_line),
                  "[HOLD] vd=%.2f Id=%+.4fA rawU=%u rawV=%u (x=stop)",
                  (double)CALIB_S1A_HOLD_VOLTAGE_V,
                  (double)output->measured_current_a.d,
                  (unsigned)s1_raw->phase_u_raw, (unsigned)s1_raw->phase_v_raw);
              control_loop_log(hold_line);
            }
            break;
          }

          case S1A_P3_BASELINE:
            /* 角度 k：零矢量采通电态基线（θ 由进入方设好：首角 start_openloop=0，换角在 DECAY 设）。 */
            foc_runtime_set_voltage_target(0.0f, 0.0f);
            if (s1_good != 0U)
            {
              control_loop_s1_base_u += (float)s1_raw->phase_u_raw;
              control_loop_s1_base_v += (float)s1_raw->phase_v_raw;
              control_loop_s1_base_n++;
            }
            if (s1_elapsed >= CALIB_S1A_P3_BASELINE_MS)
            {
              if (control_loop_s1_base_n == 0U)
              {
                control_loop_log("[FAULT] S1a P3 baseline collected no valid ADC sample");
                control_loop_enter_fault(CONTROL_LOOP_FAULT_SIGN_MISMATCH);
                break;
              }
              /* 基线累加和就地转成均值，供 INJECT 相减。 */
              control_loop_s1_base_u /= (float)control_loop_s1_base_n;
              control_loop_s1_base_v /= (float)control_loop_s1_base_n;
              control_loop_s1_lvl_u = control_loop_s1_lvl_v = 0.0f;
              control_loop_s1_lvl_n = 0U;
              control_loop_s1_sub = S1A_P3_INJECT;
              control_loop_s1_phase_tick = now_ms;
            }
            break;

          case S1A_P3_INJECT:
          {
            /* 角度 k：持续注入固定 Vd；前 SETTLE 跳过建立，之后 COLLECT 采均值。 */
            char p3l[128];
            foc_runtime_set_voltage_target(CALIB_S1A_P3_VOLTAGE_V, 0.0f);
            if ((s1_elapsed >= CALIB_S1A_P3_SETTLE_MS) && (s1_good != 0U))
            {
              control_loop_s1_lvl_u += (float)s1_raw->phase_u_raw;
              control_loop_s1_lvl_v += (float)s1_raw->phase_v_raw;
              control_loop_s1_lvl_n++;
            }
            if (s1_elapsed >= (CALIB_S1A_P3_SETTLE_MS + CALIB_S1A_P3_COLLECT_MS))
            {
              float mu, mv;
              if (control_loop_s1_lvl_n == 0U)
              {
                control_loop_log("[FAULT] S1a P3 inject collected no valid ADC sample");
                control_loop_enter_fault(CONTROL_LOOP_FAULT_SIGN_MISMATCH);
                break;
              }
              mu = control_loop_s1_lvl_u / (float)control_loop_s1_lvl_n;
              mv = control_loop_s1_lvl_v / (float)control_loop_s1_lvl_n;
              control_loop_p3_du[control_loop_p3_k] = mu - control_loop_s1_base_u;
              control_loop_p3_dv[control_loop_p3_k] = mv - control_loop_s1_base_v;
              (void)snprintf(p3l, sizeof(p3l), "[P3] ang=%u dU=%+.1f dV=%+.1f",
                             (unsigned)control_loop_p3_k,
                             (double)control_loop_p3_du[control_loop_p3_k],
                             (double)control_loop_p3_dv[control_loop_p3_k]);
              control_loop_log(p3l);
              foc_runtime_set_voltage_target(0.0f, 0.0f);
              control_loop_s1_sub = S1A_P3_DECAY;
              control_loop_s1_phase_tick = now_ms;
            }
            break;
          }

          case S1A_P3_DECAY:
            /* 回零衰减后换下一角度；三角采完转 EVAL。 */
            foc_runtime_set_voltage_target(0.0f, 0.0f);
            if (s1_elapsed >= CALIB_S1A_P3_DECAY_MS)
            {
              control_loop_p3_k++;
              if (control_loop_p3_k < CALIB_S1A_P3_ANGLE_COUNT)
              {
                foc_runtime_set_forced_angle(control_loop_p3_theta_rad[control_loop_p3_k]);
                control_loop_s1_base_u = control_loop_s1_base_v = 0.0f;
                control_loop_s1_base_n = 0U;
                control_loop_s1_sub = S1A_P3_BASELINE;
              }
              else
              {
                control_loop_s1_sub = S1A_P3_EVAL;
              }
              control_loop_s1_phase_tick = now_ms;
            }
            break;

          case S1A_P3_EVAL:
          {
            /* 三角度齐：纯函数解算符号/通道，通过才绑定（不重蹈未验证就用）。 */
            calib_probe3_result_t p3r;
            char p3l[160];
            uint8_t p3ok = calib_sequence_s1_probe3(control_loop_p3_du, control_loop_p3_dv, &p3r);
            foc_runtime_set_voltage_target(0.0f, 0.0f);
            /* 方向结论行：只回答"通道对不对、信号够不够、符号是什么、能不能绑"。 */
            (void)snprintf(p3l, sizeof(p3l),
                "[P3] DIR U-peak@%u V-peak@%u ch=%s amp=%.1fcode sign=%d/%d -> %s",
                (unsigned)p3r.peak_u_angle, (unsigned)p3r.peak_v_angle,
                p3r.channel_map_ok ? "OK" : "X",
                (double)p3r.amplitude_code, (int)p3r.sign_u, (int)p3r.sign_v,
                p3ok ? "BINDABLE" : "NOT-BOUND");
            control_loop_log(p3l);

            if (p3ok == 0U)
            {
              /* 只剩方向级硬错误才 NOGO（对称/第三相是软门，不会进到这里）。 */
              if ((p3r.hard_mask & 0x02U) != 0U)
              {
                control_loop_log("[FAULT] P3 U/V channel cross (peak not at 0/120deg); CS wiring hardware mismatch");
                control_loop_enter_fault(CONTROL_LOOP_FAULT_CHANNEL_MISMATCH);
              }
              else if ((p3r.hard_mask & 0x01U) != 0U)
              {
                control_loop_log("[FAULT] P3 weak excitation; signal buried in noise, check power/contact or raise P3 voltage");
                control_loop_enter_fault(CONTROL_LOOP_FAULT_SIGN_MISMATCH);
              }
              else if ((p3r.hard_mask & 0x10U) != 0U)
              {
                control_loop_log("[FAULT] P3 U/V amplitude mismatch too large; check solder/contact/amplifier gain");
                control_loop_enter_fault(CONTROL_LOOP_FAULT_SIGN_MISMATCH);
              }
              else
              {
                control_loop_log("[FAULT] P3 direction not resolvable; retry or check hardware");
                control_loop_enter_fault(CONTROL_LOOP_FAULT_SIGN_MISMATCH);
              }
              break;
            }

            /* 通过：绑定采样符号、置方向验证（联动 closed_loop_allowed=1），断电等确认。
             * sign_u/v 是【通道】符号（S1a 探针只认通道，与 phase_map 无关）。 */
            control_loop_parameters.current.channel_u_sign = (float)p3r.sign_u;
            control_loop_parameters.current.channel_v_sign = (float)p3r.sign_v;
            control_loop_s1_sign_u = p3r.sign_u;
            control_loop_s1_sign_v = p3r.sign_v;
            motor_adc_set_direction_verified(1U);
            control_loop_p3_bound = 1U;
            control_loop_finish_power("S1_PROBE3_BOUND");
            (void)snprintf(p3l, sizeof(p3l),
                "[RESULT] S1a sign_u=%d sign_v=%d bound (%s); press p for S1b closed-loop self-check",
                (int)p3r.sign_u, (int)p3r.sign_v,
                (p3r.fail_mask == 0U) ? "clean" : "direction OK, soft precision warn");
            control_loop_log(p3l);
            control_loop_next_step = TEST_STEP_SIGN;
            control_loop_arm_next_step();
            control_loop_s1_sub = S1A_P3_DONE;
            control_loop_s1_phase_tick = now_ms;
            break;
          }

          case S1A_BASELINE:
            foc_runtime_set_voltage_target(0.0f, 0.0f);
            if (s1_good != 0U)
            {
              control_loop_s1_base_u += (float)s1_raw->phase_u_raw;
              control_loop_s1_base_v += (float)s1_raw->phase_v_raw;
              control_loop_s1_base_n++;
            }
            if (s1_elapsed >= CALIB_S1A_BASELINE_MS)
            {
              /* 基线无任何有效样本则后续 base 均值会除零，直接故障，不进扫描。 */
              if (control_loop_s1_base_n == 0U)
              {
                control_loop_log("[FAULT] S1a baseline collected no valid ADC sample");
                control_loop_enter_fault(CONTROL_LOOP_FAULT_SIGN_MISMATCH);
                break;
              }
              /* 进入正向扫描：第 0 档、RAMP 相位。 */
              control_loop_s1_dir = 1;
              control_loop_s1_level = 0U;
              control_loop_s1_pos_levels = 0U;
              control_loop_s1_np = 0U;
              control_loop_s1_nn = 0U;
              control_loop_s1_reached = 0U;
              control_loop_s1_lock_vd = 0.0f;
              control_loop_s1_lvlph = S1_LVL_RAMP;
              control_loop_s1_lvl_u = control_loop_s1_lvl_v = 0.0f;
              control_loop_s1_lvl_n = 0U;
              control_loop_s1_sub = S1A_SWEEP;
              control_loop_s1_phase_tick = now_ms;
            }
            break;

          case S1A_SWEEP:
          {
            /* 本档电压幅值（运行时由 start/step/VEND 生成，改 config 自动联动）。 */
            float vmag = CALIB_S1A_LEVEL_START_V +
                         (float)control_loop_s1_level * CALIB_S1A_LEVEL_STEP_V;
            float vprev = (control_loop_s1_level == 0U) ? 0.0f :
                          (CALIB_S1A_LEVEL_START_V +
                           (float)(control_loop_s1_level - 1U) * CALIB_S1A_LEVEL_STEP_V);
            float vcmd, vfrom, mean_u, mean_v, du, dv, absdu;
            char lvl_line[120];
            uint8_t end_dir = 0U;
            if (vmag > CALIB_S1A_VEND_V) vmag = CALIB_S1A_VEND_V;
            if (vprev > CALIB_S1A_VEND_V) vprev = CALIB_S1A_VEND_V;
            vcmd = (control_loop_s1_dir > 0) ? vmag : -vmag;

            switch (control_loop_s1_lvlph)
            {
              case S1_LVL_RAMP:
                vfrom = (control_loop_s1_dir > 0) ? vprev : -vprev;
                if (s1_elapsed >= CALIB_S1A_LEVEL_RAMP_MS)
                {
                  foc_runtime_set_voltage_target(vcmd, 0.0f);
                  control_loop_s1_lvlph = S1_LVL_SETTLE;
                  control_loop_s1_phase_tick = now_ms;
                }
                else
                {
                  float frac = (float)s1_elapsed / (float)CALIB_S1A_LEVEL_RAMP_MS;
                  foc_runtime_set_voltage_target(vfrom + (vcmd - vfrom) * frac, 0.0f);
                }
                break;

              case S1_LVL_SETTLE:
                foc_runtime_set_voltage_target(vcmd, 0.0f); /* 保持、不采集，躲建立/开关瞬态 */
                if (s1_elapsed >= CALIB_S1A_LEVEL_SETTLE_MS)
                {
                  control_loop_s1_lvlph = S1_LVL_COLLECT;
                  control_loop_s1_phase_tick = now_ms;
                  control_loop_s1_lvl_u = control_loop_s1_lvl_v = 0.0f;
                  control_loop_s1_lvl_n = 0U;
                }
                break;

              case S1_LVL_COLLECT:
              default:
                foc_runtime_set_voltage_target(vcmd, 0.0f);
                if (s1_good != 0U)
                {
                  control_loop_s1_lvl_u += (float)s1_raw->phase_u_raw;
                  control_loop_s1_lvl_v += (float)s1_raw->phase_v_raw;
                  control_loop_s1_lvl_n++;
                }
                if (s1_elapsed >= CALIB_S1A_LEVEL_COLLECT_MS)
                {
                  if (control_loop_s1_lvl_n == 0U)
                  {
                    control_loop_log("[FAULT] S1a level collected no valid ADC sample");
                    control_loop_enter_fault(CONTROL_LOOP_FAULT_SIGN_MISMATCH);
                    break;
                  }
                  mean_u = control_loop_s1_lvl_u / (float)control_loop_s1_lvl_n;
                  mean_v = control_loop_s1_lvl_v / (float)control_loop_s1_lvl_n;
                  du = mean_u - (control_loop_s1_base_u / (float)control_loop_s1_base_n);
                  dv = mean_v - (control_loop_s1_base_v / (float)control_loop_s1_base_n);
                  absdu = fabsf(du);

                  /* 存扫描点（负向电压带符号、差也带符号，合并后落在同一过零直线）。 */
                  if (control_loop_s1_dir > 0)
                  {
                    uint8_t k = control_loop_s1_np;
                    if (k < CALIB_S1A_LEVEL_MAX)
                    {
                      control_loop_s1_vdp[k] = vmag;
                      control_loop_s1_dup[k] = du;
                      control_loop_s1_dvp[k] = dv;
                      control_loop_s1_np = (uint8_t)(k + 1U);
                    }
                  }
                  else
                  {
                    uint8_t k = control_loop_s1_nn;
                    if (k < CALIB_S1A_LEVEL_MAX)
                    {
                      control_loop_s1_vdn[k] = -vmag;
                      control_loop_s1_dun[k] = du;
                      control_loop_s1_dvn[k] = dv;
                      control_loop_s1_nn = (uint8_t)(k + 1U);
                    }
                  }
                  (void)snprintf(lvl_line, sizeof(lvl_line),
                      "[DBG] S1a %c lvl=%u vd=%+.3f du=%+.1f dv=%+.1f",
                      (control_loop_s1_dir > 0) ? '+' : '-',
                      (unsigned)control_loop_s1_level, (double)vcmd,
                      (double)du, (double)dv);
                  control_loop_log(lvl_line);

                  /* 撞轨保护：用本档均值（非单帧瞬时，抗毛刺）查是否离任一 ADC 轨太近。 */
                  if ((mean_u < (float)CALIB_S1A_RAIL_MARGIN_RAW) ||
                      (mean_v < (float)CALIB_S1A_RAIL_MARGIN_RAW) ||
                      (mean_u > (float)(4095U - CALIB_S1A_RAIL_MARGIN_RAW)) ||
                      (mean_v > (float)(4095U - CALIB_S1A_RAIL_MARGIN_RAW)))
                  {
                    control_loop_log("[FAULT] S1a ADC near rail during sweep; lower Vd / check bias");
                    control_loop_enter_fault(CONTROL_LOOP_FAULT_STEP_VALIDATE);
                    break;
                  }

                  if (control_loop_s1_dir > 0)
                  {
                    /* 正向：进目标带即锁（电流主闸门），硬电流/档数/电压天花板也收。 */
                    if ((absdu >= control_loop_s1_tlo_code) && (control_loop_s1_reached == 0U))
                    {
                      control_loop_s1_reached = 1U;
                      control_loop_s1_lock_vd = vmag;
                    }
                    if ((absdu >= control_loop_s1_hard_code) || (control_loop_s1_reached != 0U) ||
                        (control_loop_s1_level + 1U >= CALIB_S1A_LEVEL_MAX) ||
                        (vmag >= CALIB_S1A_VEND_V - 1e-4f))
                    {
                      end_dir = 1U;
                    }
                  }
                  else
                  {
                    /* 反向：追平正向档数以保证对称，硬电流也立即收。 */
                    if ((absdu >= control_loop_s1_hard_code) ||
                        (control_loop_s1_level + 1U >= control_loop_s1_pos_levels))
                    {
                      end_dir = 1U;
                    }
                  }

                  if (end_dir != 0U)
                  {
                    foc_runtime_set_voltage_target(0.0f, 0.0f);
                    if (control_loop_s1_dir > 0)
                    {
                      control_loop_s1_pos_levels = control_loop_s1_np;
                      control_loop_s1_dir = -1;
                      control_loop_s1_sub = S1A_DECAY;
                    }
                    else
                    {
                      control_loop_s1_sub = S1A_EVAL;
                    }
                    control_loop_s1_phase_tick = now_ms;
                  }
                  else
                  {
                    control_loop_s1_level++;
                    control_loop_s1_lvlph = S1_LVL_RAMP;
                    control_loop_s1_phase_tick = now_ms;
                  }
                }
                break;
            }
          }
          break;

          case S1A_DECAY:
            foc_runtime_set_voltage_target(0.0f, 0.0f); /* 正→反之间回零 */
            if (s1_elapsed >= CALIB_S1A_DECAY_MS)
            {
              control_loop_s1_level = 0U;
              control_loop_s1_lvlph = S1_LVL_RAMP;
              control_loop_s1_lvl_u = control_loop_s1_lvl_v = 0.0f;
              control_loop_s1_lvl_n = 0U;
              control_loop_s1_sub = S1A_SWEEP;
              control_loop_s1_phase_tick = now_ms;
            }
            break;

          case S1A_EVAL:
          {
            calib_s1_result_t s1r;
            char s1line[200];
            float a_per_raw;
            uint8_t s1ok;
            foc_runtime_set_voltage_target(0.0f, 0.0f);
            /* 标准增益：1 code 对应安培数（板级设计常量）。 */
            a_per_raw = BOARD_CONFIG_ADC_REFERENCE_V_NOMINAL /
                        (4095.0f * BOARD_CONFIG_PHASE_CURRENT_SHUNT_OHMS *
                         BOARD_CONFIG_PHASE_CURRENT_AMPLIFIER_GAIN);
            s1ok = calib_sequence_s1a_fit(
                control_loop_s1_vdp, control_loop_s1_dup, control_loop_s1_dvp, control_loop_s1_np,
                control_loop_s1_vdn, control_loop_s1_dun, control_loop_s1_dvn, control_loop_s1_nn,
                a_per_raw, TEST_NOMINAL_R_OHM, control_loop_s1_tlo_code,
                control_loop_s1_lock_vd, &s1r);
            (void)snprintf(s1line, sizeof(s1line),
                "[RESULT] S1a ku=%.1f kv=%.1f ratio=%.3f r2=%.3f Voff=%.3f R=%.2f maxdu=%.1f sign=%d/%d ok=%u%u%u%u%u%u",
                (double)s1r.slope_u_code_per_v, (double)s1r.slope_v_code_per_v,
                (double)s1r.ratio_vu, (double)s1r.r2_u, (double)s1r.v_offset_v,
                (double)s1r.r_est_ohm, (double)s1r.max_abs_du_code,
                (int)s1r.sign_u, (int)s1r.sign_v,
                (unsigned)s1r.linear_ok, (unsigned)s1r.reached_band, (unsigned)s1r.bipolar_ok,
                (unsigned)s1r.symmetry_ok, (unsigned)s1r.channel_ratio_ok,
                (unsigned)s1r.r_range_ok);
            control_loop_log(s1line);

            if (s1ok == 0U)
            {
              if (s1r.linear_ok == 0U)
              {
                control_loop_log("[FAULT] S1a V-I not linear/too few pts; check noise, sample timing, current chain");
                control_loop_enter_fault(CONTROL_LOOP_FAULT_SIGN_MISMATCH);
              }
              else if (s1r.reached_band == 0U)
              {
                control_loop_log("[FAULT] S1a current never reached target band up to VEND; R larger or gain smaller than expected");
                control_loop_enter_fault(CONTROL_LOOP_FAULT_SIGN_MISMATCH);
              }
              else if (s1r.channel_ratio_ok == 0U)
              {
                control_loop_log("[FAULT] S1a kv/ku!=-0.5 channel/phase fail; power off, swap U/V phase then r");
                control_loop_enter_fault(CONTROL_LOOP_FAULT_CHANNEL_MISMATCH);
              }
              else if ((s1r.bipolar_ok == 0U) || (s1r.symmetry_ok == 0U))
              {
                control_loop_log("[FAULT] S1a bipolar/symmetry fail; check dead-time, zero drift, wiring");
                control_loop_enter_fault(CONTROL_LOOP_FAULT_SIGN_MISMATCH);
              }
              else
              {
                control_loop_log("[FAULT] S1a R out of nominal window; check gain/shunt and resistance");
                control_loop_enter_fault(CONTROL_LOOP_FAULT_STEP_VALIDATE);
              }
              break;
            }

            /* S1a 通过：写实测符号、置方向验证（联动 closed_loop_allowed=1）。
             * sign_u/v 是【通道】符号，与 phase_map 无关。 */
            control_loop_parameters.current.channel_u_sign = (float)s1r.sign_u;
            control_loop_parameters.current.channel_v_sign = (float)s1r.sign_v;
            control_loop_s1_sign_u = s1r.sign_u;
            control_loop_s1_sign_v = s1r.sign_v;
            motor_adc_set_direction_verified(1U);

            /* 短暂停功率后以 CURRENT 重启进 S1b（不在运行中切换控制器，符合控制律准入）。 */
            control_loop_safe_disable();
            control_loop_log("[POWER] S1a pass; brief off, restart into S1b current loop");
            if (control_loop_start_power(CALIB_S1_ALIGN_CURRENT_A, 0.0f) == 0U)
            {
              break; /* start_power 内部已 enter_fault */
            }
            control_loop_begin_step(CALIB_S1_ALIGN_CURRENT_A,
                                    CALIB_S1_SETTLE_BAND_A, CALIB_COLLECT_N,
                                    CALIB_S1B_SPREAD_MAX_A);
            control_loop_s1b_iq_sum = 0.0f;
            control_loop_s1b_iq_n = 0U;
            control_loop_s1_sub = S1B_CLOSED_LOOP;
            control_loop_s1_phase_tick = now_ms;
          }
          break;

          default:
            break;
        }
      }
      else if (control_loop_s1_sub == S1B_CLOSED_LOOP)
      {
        /* ---------- S1b：方向已验证，建 Id=0.04A 电流闭环，验证平稳收敛、Iq≈0、无正反馈 ----------
         * 判定输入为快路径拍均值 judge_id/judge_iq（FR1），SETTLE 再叠 8 拍滑窗（FR2）。 */
        phase = calib_step_tick(&control_loop_step, judge_id, now_ms);
        if (phase == CALIB_STEP_COLLECT)
        {
          control_loop_s1b_iq_sum += judge_iq;
          control_loop_s1b_iq_n++;
        }
        if ((uint32_t)(now_ms - control_loop_sign_hb_ms) >= 100U)
        {
          char dbg_line[176];
          control_loop_sign_hb_ms = now_ms;
          (void)snprintf(dbg_line, sizeof(dbg_line),
                         "[DBG] S1b ph=%d id=%.4f iq=%.4f wm=%.4f sp=%.4f run=%u n=%u vd=%.4f mode=%d",
                         (int)phase, (double)judge_id, (double)judge_iq,
                         (double)control_loop_step.window_mean,
                         (double)control_loop_step.spread,
                         (unsigned)control_loop_step.in_band_run,
                         (unsigned)tick_samples,
                         (double)output->voltage_command_v.d,
                         (int)foc_runtime_get_control_mode());
          control_loop_log(dbg_line);
        }
        if (phase == CALIB_STEP_EVALUATE)
        {
          /* Iq 判据用 COLLECT 段拍均值（n 理论上=collect_n，为 0 时回退当前拍均值）。 */
          float mean_iq = (control_loop_s1b_iq_n > 0U)
              ? (control_loop_s1b_iq_sum / (float)control_loop_s1b_iq_n) : judge_iq;
          if (fabsf(mean_iq) <= CALIB_S1B_IQ_MAX_A)
          {
            char line[160];
            calib_step_mark_pass(&control_loop_step);
            (void)snprintf(line, sizeof(line),
                           "[RESULT] S1b closed-loop target=%.4f id=%.4fA err=%+.4fA iq=%.4fA sp=%.4fA settled; feedback direction correct",
                           (double)CALIB_S1_ALIGN_CURRENT_A,
                           (double)control_loop_step.mean,
                           (double)(control_loop_step.mean - CALIB_S1_ALIGN_CURRENT_A),
                           (double)mean_iq, (double)control_loop_step.spread);
            control_loop_log(line);
            control_loop_finish_power("S1_COMPLETE");
            control_loop_next_step = TEST_STEP_R;
            control_loop_arm_next_step();
            control_loop_s1_sub = S1_FINISHED;
          }
          else
          {
            char line[128];
            calib_step_force_nogo(&control_loop_step);
            (void)snprintf(line, sizeof(line),
                           "[FAULT] S1b Id settled but mean Iq=%.4fA too large; check forced angle/phase",
                           (double)mean_iq);
            control_loop_log(line);
            control_loop_enter_fault(CONTROL_LOOP_FAULT_STEP_VALIDATE);
          }
        }
        if (phase == CALIB_STEP_NOGO)
        {
          char line[128];
          (void)snprintf(line, sizeof(line),
                         "[FAULT] S1b no converge (wm=%.4fA sp=%.4fA band=%.4fA); feedback/PI wrong or ripple too large",
                         (double)control_loop_step.window_mean,
                         (double)control_loop_step.spread,
                         (double)CALIB_S1_SETTLE_BAND_A);
          control_loop_log(line);
          control_loop_enter_fault(CONTROL_LOOP_FAULT_NO_CONVERGE);
        }
      }
      break;

    case CONTROL_LOOP_STATE_R_IDENTIFICATION:
      if (control_loop_runtime_ok() == 0U) break;
      phase = calib_step_tick(&control_loop_step, judge_id, now_ms);
      if (phase == CALIB_STEP_COLLECT)
      {
        control_loop_s2_id_sum += judge_id;
        control_loop_s2_vd_sum += output->voltage_command_v.d;
        control_loop_s2_n++;
      }
      if (phase == CALIB_STEP_EVALUATE)
      {
        control_loop_s2_id[control_loop_s2_level] =
            control_loop_s2_id_sum / (float)control_loop_s2_n;
        control_loop_s2_vd[control_loop_s2_level] =
            control_loop_s2_vd_sum / (float)control_loop_s2_n;
        control_loop_s2_level++;
        if (control_loop_s2_level < CALIB_S2_LEVEL_COUNT)
        {
          float next_a = control_loop_s2_levels[control_loop_s2_level];
          control_loop_s2_id_sum = 0.0f;
          control_loop_s2_vd_sum = 0.0f;
          control_loop_s2_n = 0U;
          foc_runtime_set_current_target(next_a, 0.0f);
          control_loop_begin_step(next_a, CALIB_S2_SETTLE_BAND_A, CALIB_COLLECT_N,
                                  CALIB_S2_SPREAD_MAX_A);
        }
        else
        {
          calib_s2_result_t result;
          char line[128];
          uint8_t ok = calib_sequence_s2_evaluate(control_loop_s2_id, control_loop_s2_vd,
                                                  CALIB_S2_R_NOMINAL_OHM, &result);
          (void)snprintf(line, sizeof(line),
                         "[RESULT] S2 R=%.4fohm Voff=%.4fV r2=%.4f",
                         (double)result.resistance_ohm, (double)result.v_offset_v,
                         (double)result.r2);
          control_loop_log(line);
          if (ok != 0U)
          {
            calib_step_mark_pass(&control_loop_step);
            control_loop_static_r_ohm = result.resistance_ohm;
            /* FR-1.5 第 1 级单次可信门：evaluate 的 ok 已含 r2/斜率区间，
             * 此处补 |Voff| 门并记录本次 S2 结果（供 S7 双源交叉与候选窗基准）。 */
            control_loop_s2_r_ohm = result.resistance_ohm;
            control_loop_s2_trusted =
                (fabsf(result.v_offset_v) <= CALIB_S2_VOFF_MAX_V) ? 1U : 0U;
            {
              char tline[128];
              (void)snprintf(tline, sizeof(tline),
                             "[S2] trust gate: trusted=%u (|voff|=%.4fV limit=%.2fV)",
                             (unsigned)control_loop_s2_trusted,
                             (double)fabsf(result.v_offset_v),
                             (double)CALIB_S2_VOFF_MAX_V);
              control_loop_log(tline);
            }
            control_loop_parameters.id_pi.ki = result.resistance_ohm / TEST_PI_TIME_CONSTANT_S;
            control_loop_parameters.iq_pi.ki = control_loop_parameters.id_pi.ki;
            control_loop_finish_power("S2_R_COMPLETE");
            control_loop_next_step = TEST_STEP_L;
            control_loop_arm_next_step();
          }
          else
          {
            (void)snprintf(line, sizeof(line),
                           "[WARN] S2 R fit failed, fallback to nominal R=%.4fohm",
                           (double)TEST_NOMINAL_R_OHM);
            control_loop_log(line);
            control_loop_static_r_ohm = TEST_NOMINAL_R_OHM;
            control_loop_s2_r_ohm = 0.0f;
            control_loop_s2_trusted = 0U;
            control_loop_parameters.id_pi.ki = TEST_NOMINAL_R_OHM / TEST_PI_TIME_CONSTANT_S;
            control_loop_parameters.iq_pi.ki = control_loop_parameters.id_pi.ki;
            control_loop_finish_power("S2_R_FALLBACK");
            control_loop_next_step = TEST_STEP_L;
            control_loop_arm_next_step();
          }
        }
      }
      if (phase == CALIB_STEP_NOGO)
      {
        control_loop_log("[WARN] S2 current level did not converge, fallback nominal R");
        control_loop_static_r_ohm = TEST_NOMINAL_R_OHM;
        control_loop_s2_r_ohm = 0.0f;
        control_loop_s2_trusted = 0U;
        control_loop_parameters.id_pi.ki = TEST_NOMINAL_R_OHM / TEST_PI_TIME_CONSTANT_S;
        control_loop_parameters.iq_pi.ki = control_loop_parameters.id_pi.ki;
        control_loop_finish_power("S2_R_NOGO_FALLBACK");
        control_loop_next_step = TEST_STEP_L;
        control_loop_arm_next_step();
      }
      break;

    case CONTROL_LOOP_STATE_L_IDENTIFICATION:
      if (control_loop_runtime_ok() == 0U) break;
      if (control_loop_s3_step_issued == 0U)
      {
        /* 阶段 1：等 I0 稳定，随后阶跃并打开快路径窗口。 */
        phase = calib_step_tick(&control_loop_step, judge_id, now_ms);
        if ((phase == CALIB_STEP_COLLECT) || (phase == CALIB_STEP_EVALUATE))
        {
          calib_capture_reset(CALIB_CAPTURE_DEPTH_DEFAULT, CALIB_CAPTURE_STRIDE_FAST);
          foc_runtime_set_output_observer(calib_capture_push);
          foc_runtime_set_current_target(CALIB_S3_STEP_TO_A, 0.0f);
          control_loop_s3_step_issued = 1U;
        }
        if (phase == CALIB_STEP_NOGO)
        {
          control_loop_log("[WARN] S3 pre-step current did not settle, fallback nominal L");
          control_loop_static_l_h = TEST_NOMINAL_L_H;
          control_loop_parameters.id_pi.kp = TEST_NOMINAL_L_H / TEST_PI_TIME_CONSTANT_S;
          control_loop_parameters.iq_pi.kp = control_loop_parameters.id_pi.kp;
          control_loop_finish_power("S3_L_NOGO_FALLBACK");
          control_loop_next_step = TEST_STEP_CURRENT;
          control_loop_arm_next_step();
        }
      }
      else
      {
        /* 阶段 2：窗口录满后冻结评估。 */
        if (calib_capture_is_full() != 0U)
        {
          calib_s3_result_t result;
          char line[128];
          uint8_t ok;
          foc_runtime_set_output_observer(0);
          ok = calib_sequence_s3_evaluate(calib_capture_get(),
                                          calib_capture_count(),
                                          CALIB_S3_SKIP_HEAD_CYCLES,
                                          control_loop_static_r_ohm,
                                          BOARD_CONFIG_CURRENT_LOOP_PERIOD_S,
                                          &result);
          (void)snprintf(line, sizeof(line),
                         "[RESULT] S3 L=%.8fH iqr_ratio=%.3f",
                         (double)result.inductance_h, (double)result.iqr_ratio);
          control_loop_log(line);
          if (ok != 0U)
          {
            control_loop_static_l_h = result.inductance_h;
            control_loop_parameters.id_pi.kp = result.inductance_h / TEST_PI_TIME_CONSTANT_S;
            control_loop_parameters.iq_pi.kp = control_loop_parameters.id_pi.kp;
            control_loop_finish_power("S3_L_COMPLETE");
            control_loop_next_step = TEST_STEP_CURRENT;
            control_loop_arm_next_step();
          }
          else
          {
            (void)snprintf(line, sizeof(line),
                           "[WARN] S3 L estimate out of range, fallback nominal L=%.8fH",
                           (double)TEST_NOMINAL_L_H);
            control_loop_log(line);
            control_loop_static_l_h = TEST_NOMINAL_L_H;
            control_loop_parameters.id_pi.kp = TEST_NOMINAL_L_H / TEST_PI_TIME_CONSTANT_S;
            control_loop_parameters.iq_pi.kp = control_loop_parameters.id_pi.kp;
            control_loop_finish_power("S3_L_FALLBACK");
            control_loop_next_step = TEST_STEP_CURRENT;
            control_loop_arm_next_step();
          }
        }
        else if (elapsed > (2U * CALIB_STEP_TIMEOUT_MS))
        {
          control_loop_log("[WARN] S3 capture timeout, fallback nominal L");
          control_loop_static_l_h = TEST_NOMINAL_L_H;
          control_loop_parameters.id_pi.kp = TEST_NOMINAL_L_H / TEST_PI_TIME_CONSTANT_S;
          control_loop_parameters.iq_pi.kp = control_loop_parameters.id_pi.kp;
          control_loop_finish_power("S3_L_TIMEOUT_FALLBACK");
          control_loop_next_step = TEST_STEP_CURRENT;
          control_loop_arm_next_step();
        }
      }
      break;

    case CONTROL_LOOP_STATE_TEST_CURRENT_VALIDATE:
      if (control_loop_runtime_ok() == 0U) break;
      if (control_loop_s4_step_pending != 0U)
      {
        /* 建立期：ISR 保持 FROM(=0)，退出前不观测、不判据。到点后（而非启动时刻）
         * 才阶跃到 TO、初始化跟踪器，观测时钟自此计（elapsed 改用 s4_start_ms）。 */
        if ((uint32_t)(now_ms - control_loop_s4_establish_ms) < CALIB_S4_ESTABLISH_MS)
        {
          /* 建立期心跳：确认主循环/ISR 仍在跑，避免建立期“静默卡死”表象。 */
          if ((uint32_t)(now_ms - control_loop_s4_hb_ms) >= 100U)
          {
            control_loop_s4_hb_ms = now_ms;
            control_loop_log("[DBG] S4 establish idle at FROM; awaiting step");
          }
          break;
        }
        foc_runtime_set_current_target(CALIB_S4_STEP_TO_A, 0.0f);
        calib_sequence_s4_begin(&control_loop_s4, CALIB_S4_STEP_TO_A, TEST_PI_TIME_CONSTANT_S,
                                TEST_MAX_VOLTAGE_V, CALIB_S4_SETTLING_BAND_RATIO,
                                CALIB_SETTLE_HOLD_CYCLES, CALIB_S4_SETTLE_WINDOW_CYCLES,
                                CALIB_S4_SETTLING_TIME_MAX_TC,
                                CALIB_S4_OBSERVE_TC);
        {
          char s4_step_line[96];
          (void)snprintf(s4_step_line, sizeof(s4_step_line),
                         "[DBG] S4 step issued -> TO %.2fA; observe clock started",
                         (double)CALIB_S4_STEP_TO_A);
          control_loop_log(s4_step_line);
        }
        control_loop_s4_start_ms = now_ms;
        control_loop_s4_hb_ms = 0U;
        control_loop_s4_step_pending = 0U;
      }
      {
        uint8_t done = calib_sequence_s4_update(&control_loop_s4,
                                                judge_id,
                                                output->voltage_command_v.d,
                                                (uint32_t)(now_ms - control_loop_s4_start_ms));
        if (done != 0U)
        {
          calib_s4_result_t result;
          char line[160];
          uint8_t ok = calib_sequence_s4_evaluate(&control_loop_s4,
                                                  CALIB_S4_STEADY_ERROR_RATIO,
                                                  CALIB_S4_SAT_DUTY_MAX,
                                                  CALIB_S4_SPREAD_RATIO_MAX,
                                                  CALIB_S4_EXIT_RATIO_MAX,
                                                  CALIB_S4_EXIT_RUN_MAX,
                                                  &result);
          (void)snprintf(line, sizeof(line),
                         "[RESULT] S4 settle=%.1fms steady_err=%.3f%% spread=%.3f%% exit=%.1f%%/run%u osc=%d sat=%d",
                         (double)result.settle_time_ms,
                         (double)(result.steady_err_ratio * 100.0f),
                         (double)(result.window_spread_ratio * 100.0f),
                         (double)(result.exit_ratio * 100.0f),
                         (unsigned)result.max_exit_run,
                         (int)(result.no_osc_ok == 0U),
                         (int)(result.saturation_ok == 0U));
          control_loop_log(line);
          if (ok != 0U)
          {
            motor_adc_set_sample_timing_verified(1U);
            control_loop_finish_power("S4_CURRENT_VALIDATE_COMPLETE");
            /* 方案 A：先手动定编码器方向（不上电），再上电解相序——
             * PHASE_SEQ verify 依赖已知 dir 一次定解，不再事后联动镜像。 */
            control_loop_next_step = TEST_STEP_ENCODER_DIR;
            control_loop_state = CONTROL_LOOP_STATE_TEST_READY;
            control_loop_log("[GROUP] batch A (S1-S4 static electrical) complete; continuing to batch B (direction -> phase-seq -> S5 offset)");
          }
          else
          {
            control_loop_log("[FAULT] S4 step criteria failed; lower bandwidth, then check sampling, R/L");
            control_loop_enter_fault(CONTROL_LOOP_FAULT_STEP_VALIDATE);
          }
        }
      }
      break;

    case CONTROL_LOOP_STATE_PHASE_SEQ_CALIBRATION:
      if (control_loop_runtime_ok() == 0U) break;
      {
        uint32_t pseq_elapsed = (uint32_t)(now_ms - control_loop_pseq_tick);
        const motor_current_sample_t *raw = motor_adc_get_latest_sample();
        uint8_t good = ((raw->valid != 0U) && (raw->saturated == 0U) &&
                        (raw->timeout == 0U)) ? 1U : 0U;

        if ((uint32_t)(now_ms - control_loop_state_tick) > CALIB_PHASE_SEQ_TIMEOUT_MS)
        {
          control_loop_log("[FAULT] phase-seq trial timed out");
          control_loop_enter_fault(CONTROL_LOOP_FAULT_NO_CONVERGE);
          break;
        }

        switch (control_loop_pseq_sub)
        {
          case PSEQ_PREP:
          {
            char pl[96];
            control_loop_safe_disable();
            control_loop_parameters.phase_map.phase_a_output = control_loop_phase_perms[control_loop_pseq_idx][0];
            control_loop_parameters.phase_map.phase_b_output = control_loop_phase_perms[control_loop_pseq_idx][1];
            control_loop_parameters.phase_map.phase_c_output = control_loop_phase_perms[control_loop_pseq_idx][2];
            if (control_loop_start_openloop_power() == 0U)
            {
              break; /* start_openloop_power 内部已 enter_fault */
            }
            (void)snprintf(pl, sizeof(pl), "[PSEQ] perm%u a->%u b->%u c->%u",
                           (unsigned)control_loop_pseq_idx,
                           (unsigned)control_loop_phase_perms[control_loop_pseq_idx][0],
                           (unsigned)control_loop_phase_perms[control_loop_pseq_idx][1],
                           (unsigned)control_loop_phase_perms[control_loop_pseq_idx][2]);
            control_loop_log(pl);
            control_loop_pseq_id_sum = 0.0f;
            control_loop_pseq_iq_sum = 0.0f;
            control_loop_pseq_n = 0U;
            control_loop_pseq_sub = PSEQ_SETTLE;
            control_loop_pseq_tick = HAL_GetTick();
            break;
          }
          case PSEQ_SETTLE:
            foc_runtime_set_voltage_target(CALIB_PHASE_SEQ_ALIGN_VOLTAGE_V, 0.0f);
            if (pseq_elapsed >= CALIB_PHASE_SEQ_SETTLE_MS)
            {
              control_loop_pseq_sub = PSEQ_COLLECT;
              control_loop_pseq_tick = HAL_GetTick();
            }
            break;
          case PSEQ_COLLECT:
            foc_runtime_set_voltage_target(CALIB_PHASE_SEQ_ALIGN_VOLTAGE_V, 0.0f);
            if (good != 0U)
            {
              control_loop_pseq_id_sum += output->measured_current_a.d;
              control_loop_pseq_iq_sum += output->measured_current_a.q;
              control_loop_pseq_n++;
            }
            if (pseq_elapsed >= CALIB_PHASE_SEQ_COLLECT_MS)
            {
              control_loop_pseq_id[control_loop_pseq_idx] =
                  (control_loop_pseq_n > 0U) ? (control_loop_pseq_id_sum / (float)control_loop_pseq_n) : 0.0f;
              control_loop_pseq_iq[control_loop_pseq_idx] =
                  (control_loop_pseq_n > 0U) ? (control_loop_pseq_iq_sum / (float)control_loop_pseq_n) : 0.0f;
              control_loop_pseq_sub = PSEQ_DECAY;
              control_loop_pseq_tick = HAL_GetTick();
            }
            break;
          case PSEQ_DECAY:
            foc_runtime_set_voltage_target(0.0f, 0.0f);
            if (pseq_elapsed >= CALIB_PHASE_SEQ_DECAY_MS)
            {
              control_loop_safe_disable();
              control_loop_pseq_idx++;
              control_loop_pseq_sub =
                  (control_loop_pseq_idx < CALIB_PHASE_SEQ_PERM_COUNT) ? PSEQ_PREP : PSEQ_EVAL;
              control_loop_pseq_tick = HAL_GetTick();
            }
            break;
          case PSEQ_EVAL:
          {
            calib_phase_result_t pr;
            uint8_t pok = calib_sequence_phase_pick(control_loop_pseq_id, control_loop_pseq_iq,
                                                     CALIB_PHASE_SEQ_ID_MIN_A,
                                                     CALIB_PHASE_SEQ_IQ_RATIO_MAX, &pr);
            char line[176];
            (void)snprintf(line, sizeof(line),
                           "[RESULT] phase-seq best=%u pass=%u id=%.4f iq=%.4f score=%.4f",
                           (unsigned)pr.best_index, (unsigned)pr.pass_count,
                           (double)pr.best_id, (double)pr.best_iq, (double)pr.best_score);
            control_loop_log(line);
            if (pok == 0U)
            {
              control_loop_log("[FAULT] phase-seq no permutation consistent (check wiring)");
              control_loop_enter_fault(CONTROL_LOOP_FAULT_STEP_VALIDATE);
              break;
            }
            /* 选优成功不直接提交：先写 best 换相做 θ=+90° 镜像验证（判决在 PSEQ_COMMIT）。
             * θ=0 时 b/c 相电流相等，镜像排列（下标 idx^1）读数相同、原理上不可分；
             * +90° 阶跃后转子重定位，编码器角度增量符号直接暴露镜像（一致为 +，镜像为 −）。 */
            control_loop_pseq_best = pr.best_index;
            control_loop_pseq_sub = PSEQ_VERIFY_PREP;
            control_loop_pseq_tick = HAL_GetTick();
            break;
          }
          case PSEQ_VERIFY_PREP:
          {
            char vl[96];
            /* 一次性配置：写 best 换相、θ=0 起压，先让转子对齐到 best 映射的 0°
             * 作为编码器基准；随后 +90° 保持让转子重定位，用角度增量判镜像。 */
            control_loop_safe_disable();
            control_loop_parameters.phase_map.phase_a_output =
                control_loop_phase_perms[control_loop_pseq_best][0];
            control_loop_parameters.phase_map.phase_b_output =
                control_loop_phase_perms[control_loop_pseq_best][1];
            control_loop_parameters.phase_map.phase_c_output =
                control_loop_phase_perms[control_loop_pseq_best][2];
            if (control_loop_start_openloop_power() == 0U)
            {
              break; /* start_openloop_power 内部已 enter_fault */
            }
            foc_runtime_set_forced_angle(0.0f);
            foc_runtime_set_voltage_target(CALIB_PHASE_SEQ_ALIGN_VOLTAGE_V, 0.0f);
            (void)snprintf(vl, sizeof(vl), "[PSEQ] verify perm%u: align 0deg then hold +90deg, judge by encoder delta",
                           (unsigned)control_loop_pseq_best);
            control_loop_log(vl);
            control_loop_pseq_enc_before = 0.0f;
            control_loop_pseq_enc_after = 0.0f;
            control_loop_pseq_enc_ok = 0U;
            control_loop_pseq_sub = PSEQ_VERIFY_ALIGN;
            control_loop_pseq_tick = HAL_GetTick();
            break;
          }
          case PSEQ_VERIFY_ALIGN:
            /* θ=0 保持 Vd，让转子稳定对齐到 best 映射的物理 0°，末段记录编码器基准角。 */
            foc_runtime_set_forced_angle(0.0f);
            foc_runtime_set_voltage_target(CALIB_PHASE_SEQ_ALIGN_VOLTAGE_V, 0.0f);
            if (pseq_elapsed >= CALIB_PHASE_SEQ_V_ALIGN_MS)
            {
              const encoder_cache_sample_t *enca = encoder_cache_get_latest();
              if (encoder_cache_is_valid(20U) != 0U)
              {
                control_loop_pseq_enc_before = foc_mechanical_raw_to_rad(enca->raw_angle);
                control_loop_pseq_enc_ok |= 1U;
              }
              control_loop_pseq_sub = PSEQ_VERIFY_PULSE;
              control_loop_pseq_tick = HAL_GetTick();
            }
            break;
          case PSEQ_VERIFY_PULSE:
            /* 阶段2：矢量切到 +90° 并保持，等转子机械就位（预期仅转 90°/pp≈13°
             * 机械角，行程安全）；末段记录编码器角。判据是角度增量而非瞬态 Iq——
             * 编码器是数字量，不受电流噪声影响（旧 Iq 判据实测 ±0.03A，低于门限，
             * 永远 inconclusive）。 */
            foc_runtime_set_forced_angle(CALIB_PHASE_SEQ_VERIFY_ANGLE_RAD);
            foc_runtime_set_voltage_target(CALIB_PHASE_SEQ_ALIGN_VOLTAGE_V, 0.0f);
            if (pseq_elapsed >= CALIB_PHASE_SEQ_V_HOLD_MS)
            {
              const encoder_cache_sample_t *encp = encoder_cache_get_latest();
              if (encoder_cache_is_valid(20U) != 0U)
              {
                control_loop_pseq_enc_after = foc_mechanical_raw_to_rad(encp->raw_angle);
                control_loop_pseq_enc_ok |= 2U;
              }
              control_loop_pseq_sub = PSEQ_VERIFY_DECAY;
              control_loop_pseq_tick = HAL_GetTick();
            }
            break;
          case PSEQ_VERIFY_DECAY:
            /* 阶段3：矢量先回到 θ=0、电压归零，给电流衰减时间再断电。 */
            foc_runtime_set_forced_angle(0.0f);
            foc_runtime_set_voltage_target(0.0f, 0.0f);
            if (pseq_elapsed >= CALIB_PHASE_SEQ_DECAY_MS)
            {
              control_loop_safe_disable();
              control_loop_pseq_sub = PSEQ_COMMIT;
              control_loop_pseq_tick = HAL_GetTick();
            }
            break;
          case PSEQ_COMMIT:
          {
            char cl[176];
            if (control_loop_pseq_enc_ok != 3U)
            {
              control_loop_log("[PSEQ] verify encoder read invalid; inconclusive (S7 blocked)");
              control_loop_phase_seq_verified = 0U;
            }
            else
            {
              /* 判决量 = FOC 同口径电角度增量：pp × encoder_direction × Δmech。
               * 方案 A 顺序下 encoder_direction 已由前置的 ENCODER_DIR 标定为真实 e，
               * 故 sign(d_elec) 直接等于当前 best 排列的物理手性 m：
               * +90° 指令 -> +90° 级测量增量为手性一致（保持），反号即镜像（best^=1），
               * 一次定解，不再承担"补偿编码器方向错误"的职责（方向错应在 ENCODER_DIR
               * 步骤解决，手转方向必须与提示的物理 CW 一致）。 */
              float d_mech = foc_angle_wrap_signed_rad(control_loop_pseq_enc_after -
                                                       control_loop_pseq_enc_before);
              float d_elec = d_mech * (float)control_loop_parameters.rotor.pole_pairs *
                             (float)control_loop_parameters.rotor.encoder_direction;
              (void)snprintf(cl, sizeof(cl),
                             "[PSEQ] verify dmech=%+.4frad delec=%+.3frad (expect %+0.3f)",
                             (double)d_mech, (double)d_elec,
                             (double)CALIB_PHASE_SEQ_VERIFY_ANGLE_RAD);
              control_loop_log(cl);
              if ((d_elec > CALIB_PHASE_SEQ_VERIFY_MIN_DELEC_RAD) &&
                  (d_elec < CALIB_PHASE_SEQ_VERIFY_MAX_DELEC_RAD))
              {
                control_loop_log("[PSEQ] verify ok: rotation matches assumed map");
                /* FR-1.3：+90° 镜像验证通过的换相结论才允许进候选包（可固化）。 */
                control_loop_phase_seq_verified = 1U;
              }
              else if ((d_elec < -CALIB_PHASE_SEQ_VERIFY_MIN_DELEC_RAD) &&
                       (d_elec > -CALIB_PHASE_SEQ_VERIFY_MAX_DELEC_RAD))
              {
                /* 镜像排列 = 下标异或 1（交换 b/c 输出），直接切换后提交。 */
                uint8_t mirror = (uint8_t)(control_loop_pseq_best ^ 1U);
                (void)snprintf(cl, sizeof(cl),
                               "[PSEQ] mirror corrected perm%u -> perm%u (delec=%+.3frad)",
                               (unsigned)control_loop_pseq_best, (unsigned)mirror,
                               (double)d_elec);
                control_loop_log(cl);
                control_loop_pseq_best = mirror;
                control_loop_phase_seq_verified = 1U;
              }
              else
              {
                /* 落窗外面：转子没跟踪（卡滞/欠压）或角度链异常，宁可不放行 S7。 */
                (void)snprintf(cl, sizeof(cl),
                               "[PSEQ] verify inconclusive delec=%+.3frad; keep best (S7 blocked)",
                               (double)d_elec);
                control_loop_log(cl);
                control_loop_phase_seq_verified = 0U;
              }
            }
            control_loop_parameters.phase_map.phase_a_output =
                control_loop_phase_perms[control_loop_pseq_best][0];
            control_loop_parameters.phase_map.phase_b_output =
                control_loop_phase_perms[control_loop_pseq_best][1];
            control_loop_parameters.phase_map.phase_c_output =
                control_loop_phase_perms[control_loop_pseq_best][2];
            (void)snprintf(cl, sizeof(cl),
                           "[PSEQ] phase_map in RAM updated (this boot only; verified=%u)",
                           (unsigned)control_loop_phase_seq_verified);
            control_loop_log(cl);
            /* 方案 A：方向已在 PHASE_SEQ 之前标定，相序裁决完成后直接进 S5。 */
            control_loop_next_step = TEST_STEP_SENSOR;
            control_loop_arm_next_step();
            break;
          }
          default:
            break;
        }
      }
      break;

    case CONTROL_LOOP_STATE_ENCODER_DIR_CALIBRATION:
    {
      uint32_t dir_elapsed = (uint32_t)(now_ms - control_loop_dir_tick);
      const encoder_cache_sample_t *enc = encoder_cache_get_latest();
      if (encoder_cache_is_valid(20U) == 0U)
      {
        control_loop_enter_fault(CONTROL_LOOP_FAULT_ENCODER);
        break;
      }
      float mech_now = foc_mechanical_raw_to_rad(enc->raw_angle);

      if (control_loop_dir_sub == DIR_MANUAL_WAIT)
      {
        control_loop_dir_mech_delta +=
            foc_angle_wrap_signed_rad(mech_now - control_loop_dir_mech_prev);
        control_loop_dir_mech_prev = mech_now;

        /* 手转等待进度心跳：每 1.5s 报累计角位移与剩余超时，避免 15s 静默让人误按 p。 */
        if ((uint32_t)(now_ms - control_loop_dir_hb_ms) >= 1500U)
        {
          char dhb_line[112];
          uint32_t left_s = (dir_elapsed < CALIB_S6_MANUAL_TIMEOUT_MS)
              ? ((CALIB_S6_MANUAL_TIMEOUT_MS - dir_elapsed) / 1000U) : 0U;
          control_loop_dir_hb_ms = now_ms;
          (void)snprintf(dhb_line, sizeof(dhb_line),
                         "[DIR] waiting CW rotation: delta=%+.3f rad / need %.2f rad, %us left",
                         (double)control_loop_dir_mech_delta,
                         (double)CALIB_DIR_MIN_MECH_DELTA_RAD,
                         (unsigned)left_s);
          control_loop_log(dhb_line);
        }

        if ((control_loop_dir_mech_delta >= CALIB_DIR_MIN_MECH_DELTA_RAD) ||
            (control_loop_dir_mech_delta <= -CALIB_DIR_MIN_MECH_DELTA_RAD))
        {
          control_loop_dir_sub = DIR_MANUAL_DONE;
        }
        else if (dir_elapsed >= CALIB_S6_MANUAL_TIMEOUT_MS)
        {
          control_loop_log("[FAULT] ENCODER_DIR timeout: no rotation detected (15s)");
          control_loop_enter_fault(CONTROL_LOOP_FAULT_STEP_VALIDATE);
          break;
        }
      }
      else if (control_loop_dir_sub == DIR_MANUAL_DONE)
      {
        int8_t direction = (control_loop_dir_mech_delta > 0.0f) ? 1 : -1;
        char line[128];
        (void)snprintf(line, sizeof(line),
                       "[RESULT] ENCODER_DIR mech_delta=%.4f rad -> direction=%+d",
                       (double)control_loop_dir_mech_delta, (int)direction);
        control_loop_log(line);
        control_loop_parameters.rotor.encoder_direction = direction;
        control_loop_log("[DIR] encoder_direction in RAM updated (this boot only)");
        /* 方案 A：方向先于相序标定，此处只写 encoder_direction，绝不触碰 phase_map
         * （旧实现在此 best^=1 做"联动镜像"，根源是 verify 在 dir 未知时先翻了一次，
         * 形成双重翻转路径依赖，现已删除）。后续 PHASE_SEQ verify 直接用该方向
         * 一次定解相序手性。前提：手转方向必须与提示的物理 CW 一致。
         * 手动模式未上电，无需 finish_power。 */
        control_loop_next_step = TEST_STEP_PHASE_SEQ;
        control_loop_arm_next_step();
      }
      break;
    }

    case CONTROL_LOOP_STATE_SENSOR_CALIBRATION:
      if (control_loop_runtime_ok() == 0U) break;
      {
        uint32_t s5_elapsed = (uint32_t)(now_ms - control_loop_s5_tick);
        uint8_t fwd_half = CALIB_S5_REPEAT_COUNT / 2U;
        const encoder_cache_sample_t *enc = encoder_cache_get_latest();
        if (encoder_cache_is_valid(20U) == 0U)
        {
          control_loop_enter_fault(CONTROL_LOOP_FAULT_ENCODER);
          break;
        }
        switch (control_loop_s5_sub)
        {
          case S5_ALIGN:
            /* 正向 d 轴对齐 θ=0，读一次 offset（方向/线序已在上两步标定）。 */
            foc_runtime_set_forced_angle(0.0f);
            foc_runtime_set_current_target(CALIB_S5_ALIGN_CURRENT_A, 0.0f);
            if (s5_elapsed >= CALIB_S5_ALIGN_SETTLE_MS)
            {
              control_loop_s5_offsets[control_loop_s5_idx] = foc_angle_wrap_rad(
                  -foc_mechanical_raw_to_rad(enc->raw_angle) *
                  (float)control_loop_parameters.rotor.pole_pairs *
                  (float)control_loop_parameters.rotor.encoder_direction);
              control_loop_s5_idx++;
              if (control_loop_s5_idx >= CALIB_S5_REPEAT_COUNT)
              {
                control_loop_s5_sub = S5_EVAL;
              }
              else if (control_loop_s5_idx >= fwd_half)
              {
                /* 后半段：先反向预扭转，实现反向趋近。 */
                control_loop_s5_sub = S5_REV_PREMOVE;
              }
              /* 否则继续正向对齐。 */
              control_loop_s5_tick = HAL_GetTick();
            }
            break;
          case S5_REV_PREMOVE:
            /* 预扭转到相反电角 θ=π，再回 θ=0 从反向趋近对齐点。 */
            foc_runtime_set_forced_angle(CALIB_PI);
            foc_runtime_set_current_target(CALIB_S5_ALIGN_CURRENT_A, 0.0f);
            if (s5_elapsed >= CALIB_S5_REV_PREMOVE_MS)
            {
              control_loop_s5_sub = S5_ALIGN;
              control_loop_s5_tick = HAL_GetTick();
            }
            break;
          case S5_EVAL:
          {
            calib_s5_result_t sr;
            uint8_t sok = calib_sequence_s5_eval(control_loop_s5_offsets,
                                                 CALIB_S5_REPEAT_COUNT, fwd_half,
                                                 CALIB_S5_CIRC_STD_MAX_RAD,
                                                 CALIB_S5_FWD_REV_DIFF_MAX_RAD, &sr);
            char line[176];
            (void)snprintf(line, sizeof(line),
                           "[RESULT] S5 offset=%.6f std=%.4f fwd_rev=%.4f ok=%u",
                           (double)sr.offset_rad, (double)sr.circ_std_rad,
                           (double)sr.fwd_rev_diff_rad, (unsigned)sr.ok);
            control_loop_log(line);
            control_loop_parameters.rotor.electrical_offset_rad = sr.offset_rad;
            control_loop_print_result("encoder_offset", sr.offset_rad, "rad");
            control_loop_finish_power("S5_OFFSET_COMPLETE");
            /* 记录 S5 质量结论供 S7 入口门禁使用（ok=0 拒绝加功率）。 */
            control_loop_s5_ok = sok;
            if (sok == 0U)
            {
              control_loop_log("[S5] warn: circular std / fwd-rev diff above tolerance; offset kept but verify");
            }
            control_loop_next_step = TEST_STEP_CAPTURE;
            control_loop_arm_next_step();
            break;
          }
          default:
            break;
        }
      }
      break;

    case CONTROL_LOOP_STATE_LOW_SPEED_CAPTURE:
      if (control_loop_runtime_ok() == 0U) break;
      {
        const encoder_cache_sample_t *encoder = encoder_cache_get_latest();
        foc_rotor_sample_t rotor;
        float mech_now;
        if ((encoder_cache_is_valid(20U) == 0U) ||
            (foc_rotor_model_convert(&control_loop_parameters.rotor,
                                     encoder->raw_angle, &rotor) != FOC_STATUS_OK))
        {
          control_loop_log("[FAULT] S7 encoder chain: cache stale (>20ms) or rotor model reject; check MT6701 wiring");
          control_loop_enter_fault(CONTROL_LOOP_FAULT_ENCODER);
          break;
        }
        foc_runtime_set_forced_angle(rotor.electrical_angle_rad);
        /* FR-1.3.2：机械行程跟踪（wrap 安全差分累计，供力矩方向门禁）。 */
        mech_now = foc_mechanical_raw_to_rad(encoder->raw_angle);
        control_loop_s7_mech_delta +=
            foc_angle_wrap_signed_rad(mech_now - control_loop_s7_mech_prev);
        control_loop_s7_mech_prev = mech_now;
        /* 判据 v2：窗电压累计（建立段后；id/iq 的 tick 均值累计在遥测块处）。 */
        if (elapsed > (uint32_t)CALIB_S7_SETTLE_SKIP_MS)
        {
          control_loop_s7_wvd_sum += output->voltage_command_v.d;
          control_loop_s7_wvq_sum += output->voltage_command_v.q;
          control_loop_s7_vn++;
        }
      }
      /* 失步早期检测（建立段豁免）：错误电角度下 Iq 永远追不到目标、Vq 持续
       * 贴限；双条件同时保持 HOLD 时长，就在 ADC 贴轨之前温柔撤功率。
       * 这是"误差型贴轨"的上游拦截，标定失败语义：不进硬件 FAULT、不产候选、回 PHASE_SEQ。 */
      if (elapsed > CALIB_S7_SETTLE_SKIP_MS)
      {
        float los_iq_err = output->measured_current_a.q - CALIB_S7_IQ_DRIVE_A;
        float los_vq = output->voltage_command_v.q;
        if (los_iq_err < 0.0f) { los_iq_err = -los_iq_err; }
        if (los_vq < 0.0f) { los_vq = -los_vq; }
        if ((los_iq_err > CALIB_S7_LOS_IQ_ERR_A) &&
            (los_vq >= (TEST_MAX_VOLTAGE_V * CALIB_S7_LOS_VSAT_RATIO)))
        {
          control_loop_s7_los_ms++;
        }
        else
        {
          control_loop_s7_los_ms = 0U;
        }
        if (control_loop_s7_los_ms >= CALIB_S7_LOS_HOLD_MS)
        {
          char los_line[176];
          (void)snprintf(los_line, sizeof(los_line),
                         "[S7] loss of sync: iq_err=%.3fA |vq|=%.3fV held %lums; abort before ADC rail",
                         (double)los_iq_err, (double)los_vq,
                         (unsigned long)control_loop_s7_los_ms);
          control_loop_log(los_line);
          control_loop_log("[S7] no candidate; redo PHASE_SEQ/S5 (r=restart then p per step)");
          control_loop_finish_power("S7_LOSS_OF_SYNC");
          control_loop_candidate_ready = 0U;
          control_loop_phase_seq_verified = 0U;
          control_loop_next_step = TEST_STEP_PHASE_SEQ;
          control_loop_state = CONTROL_LOOP_STATE_TEST_READY;
          break;
        }
      }
      /* S7 周期遥测与判据 v2 窗统计：瞬时值 + 50ms 窗均值并列；
       * 建立段后累计，窗尾结算 EMF-free 比值 wvd/wid 并喂全程累计。 */
      if ((have_tick_mean != 0U) && (elapsed > (uint32_t)CALIB_S7_SETTLE_SKIP_MS))
      {
        control_loop_s7_wid_sum += judge_id;
        control_loop_s7_wiq_sum += judge_iq;
        control_loop_s7_wn++;
      }
      if ((uint32_t)(elapsed - control_loop_s7_dbg_ms) >= CALIB_S7_DBG_PERIOD_MS)
      {
        char dbg_line[176];
        float win_id, win_iq, win_vd;
        control_loop_s7_dbg_ms = elapsed;
        win_id = (control_loop_s7_wn > 0U)
            ? control_loop_s7_wid_sum / (float)control_loop_s7_wn : 0.0f;
        win_iq = (control_loop_s7_wn > 0U)
            ? control_loop_s7_wiq_sum / (float)control_loop_s7_wn : 0.0f;
        win_vd = (control_loop_s7_vn > 0U)
            ? control_loop_s7_wvd_sum / (float)control_loop_s7_vn : 0.0f;
        (void)snprintf(dbg_line, sizeof(dbg_line),
                       "[S7DBG] t=%lums id=%.4f iq=%.4f vd=%.3f vq=%.3f wid=%.4f wiq=%.4f wn=%u",
                       (unsigned long)elapsed,
                       (double)output->measured_current_a.d,
                       (double)output->measured_current_a.q,
                       (double)output->voltage_command_v.d,
                       (double)output->voltage_command_v.q,
                       (double)win_id, (double)win_iq,
                       (unsigned)control_loop_s7_wn);
        control_loop_log(dbg_line);
        /* 全程窗均值累计（每窗等权），供调节品质门。 */
        if (control_loop_s7_wn > 0U)
        {
          control_loop_s7_cap_wid_sum += win_id;
          control_loop_s7_cap_wiq_sum += win_iq;
          control_loop_s7_cap_wn++;
          /* EMF-free 比值：vd 只含阻性项（EMF 落 q 轴），近零 wid 窗不产生比值。 */
          if ((win_id >= CALIB_S7_WRATIO_MIN_WID_A) &&
              (control_loop_s7_wratio_n < CALIB_S7_WINDOW_COUNT))
          {
            control_loop_s7_wratio[control_loop_s7_wratio_n] = win_vd / win_id;
            control_loop_s7_wratio_n++;
          }
        }
        control_loop_s7_wid_sum = 0.0f;
        control_loop_s7_wiq_sum = 0.0f;
        control_loop_s7_wn = 0U;
        control_loop_s7_wvd_sum = 0.0f;
        control_loop_s7_wvq_sum = 0.0f;
        control_loop_s7_vn = 0U;
      }
      if (elapsed >= TEST_CAPTURE_TIME_MS)
      {
        control_loop_finish_power("LOW_SPEED_CAPTURE_COMPLETE");
        /* 判决函数内部决定去留：正常路径 -> CANDIDATE_REVIEW，
         * 行程/方向矛盾 -> TEST_READY + 回 PHASE_SEQ。 */
        control_loop_judge_s7_and_create_candidate();
      }
      break;

    case CONTROL_LOOP_STATE_VJ_DIAG:
    {
      const encoder_cache_sample_t *vj_enc = encoder_cache_get_latest();
      const motor_current_sample_t *vj_raw = motor_adc_get_latest_sample();
      foc_rotor_sample_t vj_rotor;
      uint32_t vj_elapsed;

      if (control_loop_vj_sub == VJ_SUB_IDLE)
      {
        break; /* 未上功率，等 p */
      }
      if (control_loop_runtime_ok() == 0U) break;
      /* 角度供给与 S7 同源：编码器连续出角，保证测的就是 S7 那条链。
       * 去抖（2026-09-13 上板教训）：使能瞬间栅驱 boost 瞬态可能干扰一次 I2C
       * 读数（cache 单次失败即 valid=0），单次无效不判故障——保持上一帧角度
       * 继续运行；连续无效达 CALIB_VJ_ENC_INVALID_FAULT_MS 才是真故障。 */
      if ((encoder_cache_is_valid(20U) == 0U) ||
          (foc_rotor_model_convert(&control_loop_parameters.rotor,
                                   vj_enc->raw_angle, &vj_rotor) != FOC_STATUS_OK))
      {
        control_loop_vj_enc_bad_ms++;
        if (control_loop_vj_enc_bad_ms >= CALIB_VJ_ENC_INVALID_FAULT_MS)
        {
          control_loop_log("[FAULT] VJ encoder chain invalid (held)");
          control_loop_enter_fault(CONTROL_LOOP_FAULT_ENCODER);
          break;
        }
      }
      else
      {
        control_loop_vj_enc_bad_ms = 0U;
        foc_runtime_set_forced_angle(vj_rotor.electrical_angle_rad);
        control_loop_vj_theta = vj_rotor.electrical_angle_rad;
      }
      vj_elapsed = (uint32_t)(now_ms - control_loop_vj_tick);

      switch (control_loop_vj_sub)
      {
        case VJ_SUB_HOLD:
          foc_runtime_set_voltage_target(0.0f, 0.0f);
          break;

        case VJ_SUB_PULSE:
        {
          char line[176];
          float mean_id, mean_iq, mean_du, mean_dv;
          foc_runtime_set_voltage_target(control_loop_vj_vd, control_loop_vj_vq);
          /* 跳过建立段后累计拍均值（judge_id/iq 来自快路径 tick mean）。 */
          if ((vj_elapsed >= (uint32_t)CALIB_VJ_PULSE_SKIP_MS) &&
              (vj_raw->valid != 0U) && (vj_raw->saturated == 0U) &&
              (vj_raw->timeout == 0U))
          {
            control_loop_vj_id_sum += judge_id;
            control_loop_vj_iq_sum += judge_iq;
            control_loop_vj_du_sum += (float)vj_raw->phase_u_raw -
                control_loop_parameters.current.channel_u_zero_raw;
            control_loop_vj_dv_sum += (float)vj_raw->phase_v_raw -
                control_loop_parameters.current.channel_v_zero_raw;
            control_loop_vj_n++;
          }
          if (vj_elapsed >= (uint32_t)CALIB_VJ_PULSE_MS)
          {
            mean_id = (control_loop_vj_n > 0U)
                ? control_loop_vj_id_sum / (float)control_loop_vj_n : 0.0f;
            mean_iq = (control_loop_vj_n > 0U)
                ? control_loop_vj_iq_sum / (float)control_loop_vj_n : 0.0f;
            mean_du = (control_loop_vj_n > 0U)
                ? control_loop_vj_du_sum / (float)control_loop_vj_n : 0.0f;
            mean_dv = (control_loop_vj_n > 0U)
                ? control_loop_vj_dv_sum / (float)control_loop_vj_n : 0.0f;
            (void)snprintf(line, sizeof(line),
                "[VJ] th=%+.3f v%s=%+.2f -> id=%+.4f iq=%+.4f (n=%u) dU=%+.1f dV=%+.1f",
                (double)control_loop_vj_theta,
                (control_loop_vj_axis_q != 0U) ? "q" : "d",
                (double)((control_loop_vj_axis_q != 0U) ? control_loop_vj_vq
                                                        : control_loop_vj_vd),
                (double)mean_id, (double)mean_iq,
                (unsigned)control_loop_vj_n,
                (double)mean_du, (double)mean_dv);
            control_loop_log(line);
            foc_runtime_set_voltage_target(0.0f, 0.0f);
            control_loop_vj_vd = 0.0f;
            control_loop_vj_vq = 0.0f;
            control_loop_vj_sub = VJ_SUB_HOLD;
            control_loop_vj_tick = now_ms;
          }
          break;
        }

        case VJ_SUB_SWEEP:
        {
          char line[176];
          float sweep_id, sweep_iq;
          foc_runtime_set_voltage_target(control_loop_vj_vd, control_loop_vj_vq);
          if ((vj_raw->valid != 0U) && (vj_raw->saturated == 0U) &&
              (vj_raw->timeout == 0U))
          {
            control_loop_vj_id_sum += judge_id;
            control_loop_vj_iq_sum += judge_iq;
            control_loop_vj_n++;
          }
          if ((uint32_t)(now_ms - control_loop_vj_tlm_ms) >=
              (uint32_t)CALIB_VJ_SWEEP_TLM_MS)
          {
            control_loop_vj_tlm_ms = now_ms;
            (void)snprintf(line, sizeof(line),
                "[VJT] t=%lums th=%+.3f id=%+.4f iq=%+.4f vq=%+.2f dU=%+.1f dV=%+.1f",
                (unsigned long)vj_elapsed,
                (double)control_loop_vj_theta,
                (double)output->measured_current_a.d,
                (double)output->measured_current_a.q,
                (double)control_loop_vj_vq,
                (double)((float)vj_raw->phase_u_raw -
                         control_loop_parameters.current.channel_u_zero_raw),
                (double)((float)vj_raw->phase_v_raw -
                         control_loop_parameters.current.channel_v_zero_raw));
            control_loop_log(line);
          }
          if (vj_elapsed >= (uint32_t)CALIB_VJ_SWEEP_MS)
          {
            sweep_id = (control_loop_vj_n > 0U)
                ? control_loop_vj_id_sum / (float)control_loop_vj_n : 0.0f;
            sweep_iq = (control_loop_vj_n > 0U)
                ? control_loop_vj_iq_sum / (float)control_loop_vj_n : 0.0f;
            (void)snprintf(line, sizeof(line),
                "[VJT] sweep done: mean id=%+.4f iq=%+.4f (n=%lu); voltage to zero",
                (double)sweep_id, (double)sweep_iq,
                (unsigned long)control_loop_vj_n);
            control_loop_log(line);
            foc_runtime_set_voltage_target(0.0f, 0.0f);
            control_loop_vj_vd = 0.0f;
            control_loop_vj_vq = 0.0f;
            control_loop_vj_sub = VJ_SUB_HOLD;
            control_loop_vj_tick = now_ms;
          }
          break;
        }

        default:
          break;
      }
      break;
    }

    case CONTROL_LOOP_STATE_POWER_ARMED:
    case CONTROL_LOOP_STATE_TEST_ALIGN: /* 旧独立对齐步已被 S1 取代，保留枚举不可达 */
    case CONTROL_LOOP_STATE_CANDIDATE_REVIEW:
    case CONTROL_LOOP_STATE_SAFE_IDLE:
    case CONTROL_LOOP_STATE_FAULT:
    default:
      break;
  }
}

control_loop_state_t control_loop_get_state(void) { return control_loop_state; }
control_loop_fault_t control_loop_get_fault(void) { return control_loop_fault; }

const char *control_loop_state_text(control_loop_state_t state)
{
  switch (state)
  {
    case CONTROL_LOOP_STATE_SAFE_IDLE: return "SAFE_IDLE";
    case CONTROL_LOOP_STATE_TEST_BOOT: return "TEST_BOOT";
    case CONTROL_LOOP_STATE_TEST_READY: return "TEST_READY";
    case CONTROL_LOOP_STATE_POWER_ARMED: return "POWER_ARMED";
    case CONTROL_LOOP_STATE_TEST_ALIGN: return "TEST_ALIGN";
    case CONTROL_LOOP_STATE_SIGN_CALIBRATION: return "SIGN_CALIBRATION";
    case CONTROL_LOOP_STATE_R_IDENTIFICATION: return "R_IDENTIFICATION";
    case CONTROL_LOOP_STATE_L_IDENTIFICATION: return "L_IDENTIFICATION";
    case CONTROL_LOOP_STATE_TEST_CURRENT_VALIDATE: return "CURRENT_VALIDATE";
    case CONTROL_LOOP_STATE_PHASE_SEQ_CALIBRATION: return "PHASE_SEQ_CALIBRATION";
    case CONTROL_LOOP_STATE_ENCODER_DIR_CALIBRATION: return "ENCODER_DIR_CALIBRATION";
    case CONTROL_LOOP_STATE_SENSOR_CALIBRATION: return "SENSOR_CALIBRATION";
    case CONTROL_LOOP_STATE_LOW_SPEED_CAPTURE: return "LOW_SPEED_CAPTURE";
    case CONTROL_LOOP_STATE_CANDIDATE_REVIEW: return "CANDIDATE_REVIEW";
    case CONTROL_LOOP_STATE_OPEN_LOOP_READY: return "OPEN_LOOP_READY";
    case CONTROL_LOOP_STATE_ALIGN: return "ALIGN";
    case CONTROL_LOOP_STATE_OPEN_LOOP_RUN: return "OPEN_LOOP_RUN";
    case CONTROL_LOOP_STATE_IDENT_PRECHECK: return "IDENT_PRECHECK";
    case CONTROL_LOOP_STATE_FORCED_CURRENT: return "FORCED_CURRENT";
    case CONTROL_LOOP_STATE_PI_VALIDATE: return "PI_VALIDATE";
    case CONTROL_LOOP_STATE_LIMITED_FOC_READY: return "LIMITED_FOC_READY";
    case CONTROL_LOOP_STATE_VJ_DIAG: return "VJ_DIAG";
    case CONTROL_LOOP_STATE_STOPPING: return "STOPPING";
    case CONTROL_LOOP_STATE_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

const char *control_loop_fault_text(control_loop_fault_t fault)
{
  switch (fault)
  {
    case CONTROL_LOOP_FAULT_NONE: return "NONE";
    case CONTROL_LOOP_FAULT_PWM_START: return "PWM_START";
    case CONTROL_LOOP_FAULT_DRIVER_ENABLE: return "DRIVER_ENABLE";
    case CONTROL_LOOP_FAULT_ADC: return "ADC";
    case CONTROL_LOOP_FAULT_ENCODER: return "ENCODER";
    case CONTROL_LOOP_FAULT_TIMEOUT: return "TIMEOUT";
    case CONTROL_LOOP_FAULT_CURRENT_LOOP: return "CURRENT_LOOP";
    case CONTROL_LOOP_FAULT_SIGN_MISMATCH: return "SIGN_MISMATCH";
    case CONTROL_LOOP_FAULT_CHANNEL_MISMATCH: return "CHANNEL_MISMATCH";
    case CONTROL_LOOP_FAULT_NO_CONVERGE: return "NO_CONVERGE";
    case CONTROL_LOOP_FAULT_STEP_VALIDATE: return "STEP_VALIDATE";
    default: return "UNKNOWN";
  }
}
