/*
 * 半自动标定序列判据层（批次 A：S1-S4，静止电气组）。
 *
 * 三层分工中本文件属于"各步业务判据"：calib_step 管通用时序（收敛/采集/超时），
 * control_loop 管激励编排，本模块只做"拿到统计量后如何判 go/no-go、物理结果是多少"。
 *
 * 约束：
 *  - 纯函数 / 纯 C，不访问 HAL/ADC/I2C/PWM/UART，可在 PC 上 gcc 单测；
 *  - 所有阈值取自 calib_config.h，本文件不写字面量阈值；
 *  - 慢路径（1ms 主循环）调用，允许 sqrtf/fabsf，严禁放进 20kHz 中断；
 *  - evaluate 返回 1=PASS、0=NOGO，结果细节写入 out 结构供打印与整定。
 *
 * S5-S8（offset/方向/闭环/出参）属批次 B/C，本文件不实现，仅在此预留归属。
 */
#ifndef CALIB_SEQUENCE_H
#define CALIB_SEQUENCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "calib_capture.h" /* calib_sample_t：快路径冻结样本，POD 结构 */

/* ============ S1a 开环双极性电压爬坡扫描：符号/通道/相序/增益量级（raw 层 + 线性拟合） ============ */
typedef struct
{
  int8_t sign_u;             /* U 通道符号 +1 / -1 / 0 未定（由 V-I 斜率符号定） */
  int8_t sign_v;             /* V 通道符号 +1 / -1 / 0 未定 */
  uint8_t linear_ok;         /* 正/负点足够且 V-I 线性度 r2 达标 */
  uint8_t reached_band;      /* 扫描中最大 |du| 达到目标带下沿（激励足够） */
  uint8_t bipolar_ok;        /* 正/负方向斜率同号（带符号坐标里单调跟随，即响应反相） */
  uint8_t symmetry_ok;       /* 正/负斜率幅值对称度在容差内 */
  uint8_t channel_ratio_ok;  /* slope_v/slope_u 落在 -0.5±tol：通道/相序自洽 */
  uint8_t r_range_ok;        /* 由标准增益换算的等效 R 落在标称窗内 */
  uint8_t pos_pts;           /* 正向参与拟合点数 */
  uint8_t neg_pts;           /* 负向参与拟合点数 */
  float slope_u_code_per_v;  /* 合并拟合 U 通道斜率（code/V） */
  float slope_v_code_per_v;  /* 合并拟合 V 通道斜率（code/V） */
  float intercept_u_code;    /* U 拟合截距（code） */
  float r2_u, r2_v;          /* 合并拟合决定系数 */
  float ratio_vu;            /* slope_v/slope_u（整条线比值，抗单点噪声） */
  float v_offset_v;          /* 死区/管压降等效电压 = |intercept/slope| */
  float symmetry;            /* 正/负斜率幅值不对称度（0 为完全对称） */
  float r_est_ohm;           /* 标准增益换算的等效相电阻（粗值，精确 R 归 S2） */
  float max_abs_du_code;     /* 扫描中最大 |du|，用于判 reached_band */
  float lock_vd_v;           /* 正向进入目标带时的电压档（V），上层填，本函数透传打印 */
} calib_s1_result_t;

/* 爬坡扫描双极性拟合判据（纯函数）。
 * 正向：vd_p 为正电压(V)，du_p/dv_p 为相对零矢量基线的 raw 差；
 * 负向：vd_n 为带符号负电压，du_n/dv_n 为带符号 raw 差（物理上正/负点应落在同一条过零直线）。
 * a_per_raw=标准增益下 1 code 对应安培数（板级常量，上层传入）；r_nominal=标称相电阻；
 * target_lo_code=目标带下沿 code（判激励是否足够）。全程只用原始 raw 差，不依赖待标定 sign。
 * 返回 1=全部判据通过。 */
uint8_t calib_sequence_s1a_fit(const float *vd_p, const float *du_p, const float *dv_p, uint8_t n_p,
                               const float *vd_n, const float *du_n, const float *dv_n, uint8_t n_n,
                               float a_per_raw, float r_nominal_ohm, float target_lo_code,
                               float lock_vd_v, calib_s1_result_t *out);

/* ============ S1a 静态三角度探针 PROBE3：采样符号 + 通道对应（raw 层，电机不转） ============
 * 依次在电角度 0°/120°/240° 注入，du3/dv3 为各角度相对零矢量基线的 raw 偏移（长度 3，带符号）。
 * 理论签名（s1=s2=+1、独大幅值 I）：
 *   角度  0°: U=+I(独大) V=-I/2 ; 120°: U=-I/2 V=+I(独大) ; 240°: U=-I/2 V=-I/2（第三相重构独大）
 *
 * 设计哲学——S1a 是"方向问题"，不是"精度问题"（低端硬件不陷入精度陷阱）：
 *   sign/通道只取决于"独大落在哪个角度(离散) + 独大值符号(离散)"，这是鲁棒的定性结论；
 *   而 -0.5 对称度、第三相重构落角受死区压降、只采两相(第三相靠 -(U+V) 重构叠加误差)、
 *   齿槽等影响，小半幅点天然偏差，属"定量精度"，只做软质量记录、不阻断方向绑定。
 *
 *   硬门 hard_mask（不过=方向不可信，绝不绑定）：bit0 激励弱 / bit1 通道交叉 / bit4 U/V 幅值不一致；
 *   软门（仅 fail_mask 可见、不影响 ok）：bit2 对称度 / bit3 第三相落角。
 *   ok=1 等价 hard_mask==0：方向可绑定；精确量(死区 Voff、精确 R/L)留给后续 S2/S3 拟合。 */
typedef struct
{
  int8_t sign_u;             /* U 通道(半桥1)采样符号 +1/-1/0 未定 */
  int8_t sign_v;             /* V 通道(半桥2)采样符号 +1/-1/0 未定 */
  uint8_t peak_u_angle;      /* U 通道 |偏移| 独大所在角度索引（期望 0=0°） */
  uint8_t peak_v_angle;      /* V 通道 |偏移| 独大所在角度索引（期望 1=120°） */
  uint8_t channel_map_ok;    /* 【硬】U 独大在0°且 V 独大在120°：通道↔半桥对应正确（否则 CS 交叉=硬件错） */
  uint8_t symmetry_ok;       /* 【软】每通道两非独大值/独大值均 ≈ -0.5（三相对称，死区下只作质量记录） */
  uint8_t third_phase_ok;    /* 【软】第三相 -(du+dv) 独大落在240°（两相采样重构派生量，只作质量记录） */
  uint8_t amplitude_ok;      /* 【硬】U/V 独大幅值接近、且都≥最小可用 raw（激励充分、两通道增益一致） */
  float amplitude_code;      /* U/V 独大幅值均值（code） */
  float ratio_mean;          /* 四个"非独大/独大"比值均值（理想 -0.5，仅质量参考） */
  uint8_t fail_mask;         /* 全量位掩码（硬+软）：bit0 激励弱 bit1 通道交叉 bit2 不对称 bit3 第三相 bit4 幅值不一致 */
  uint8_t hard_mask;         /* 仅硬门：bit0/bit1/bit4；hard_mask==0 才允许绑定 sign */
  uint8_t ok;                /* =(hard_mask==0)：方向可信、可绑定 sign_u/v（软门不影响） */
} calib_probe3_result_t;

/* 纯函数解算，返回 1=方向可信（可绑定 sign_u/v）；阈值全取 calib_config.h。 */
uint8_t calib_sequence_s1_probe3(const float du3[3], const float dv3[3], calib_probe3_result_t *out);

/* ============ S2 相电阻（三档最小二乘 V = R*I + Voff） ============ */
typedef struct
{
  float resistance_ohm;      /* 拟合斜率 = R */
  float v_offset_v;         /* 拟合截距，反映死区/管压降偏置 */
  float r2;                  /* 线性度 */
} calib_s2_result_t;

/* 输入：三档稳态 Id 均值与对应 Vd 均值（长度均为 3）。
 * 单次可信门（FR-1.5 第 1 级）：r2 达 CALIB_S2_FIT_R2_MIN、斜率在标称区间、
 * 且 |截距 Voff| <= CALIB_S2_VOFF_MAX_V（样本数=3 档由 S2 引擎保证）。 */
uint8_t calib_sequence_s2_evaluate(const float id3[3], const float vd3[3],
                                   float r_nominal_ohm, calib_s2_result_t *out);

/* ============ S3 相电感（阶跃窗口逐点 L，取中位数） ============ */
typedef struct
{
  float inductance_h;        /* 逐点 L 的中位数 */
  float iqr_ratio;           /* IQR / 中位数，离散度 */
} calib_s3_result_t;

/* 输入：capture 冻结窗口、跳过的阶跃跳头样本数、已估 R、采样周期 Ts。
 * 逐点 L = (Vd - R*Id) / (dId/dt)，时间差用相邻样本 cycle 差 * Ts。 */
uint8_t calib_sequence_s3_evaluate(const calib_sample_t *s, uint16_t n,
                                   uint16_t skip_head,
                                   float resistance_ohm, float ts_s,
                                   calib_s3_result_t *out);

/* ============ S4 电流环阶跃自检（四判据跟踪器，慢路径逐拍喂入） ============ */
/* S4 稳态带判定滑窗物理上限（窗宽由 calib_config 的 CALIB_S4_SETTLE_WINDOW_CYCLES 决定）。 */
#define CALIB_S4_WINDOW_MAX            (16U)

typedef struct
{
  /* 配置 */
  float target_a;
  float tc_s;
  float v_limit_v;
  float band_ratio;          /* 稳态带，如 0.02 = 2% */
  uint16_t band_hold;        /* 连续在带内多少拍算建立 */
  uint16_t window_cycles;    /* 在带判定滑动窗宽（拍），0=逐拍旧行为 */
  uint32_t settle_deadline_ms; /* 建立时间上限 settle_tc * Tc */
  uint32_t observe_ms;       /* 总观测时长 observe_tc * Tc */
  /* 运行态 */
  uint16_t in_band_run;
  uint8_t settled;
  uint32_t settle_time_ms;
  uint16_t band_exit_count;  /* 建立后再穿出稳态带的窗口数（慢振荡/直流偏移） */
  uint16_t exit_run;         /* 当前连续穿出窗口数 */
  uint16_t max_exit_run;     /* 建立后最长连续穿出窗口数（区分噪声孤立穿出与持续振荡） */
  uint16_t post_windows;     /* 建立后经历的窗口数（穿出比例分母） */
  uint16_t saturated_count;  /* |Vd| 顶电压限的拍数 */
  float tail_err_sum;        /* 末段误差累计（静差，用窗均值） */
  uint32_t tail_n;
  float tail_spread_sum;     /* 建立后各窗内 std 累计（spread 门限只统计稳态段，避开上升瞬态） */
  uint8_t done;
  /* 抗单拍噪声滑动窗：环形缓冲，每拍重算窗均值/样本标准差（W<=16，慢路径 O(W)）。 */
  float win_buf[CALIB_S4_WINDOW_MAX];
  float win_sum;
  float window_mean;         /* 最近一拍窗均值（A） */
  float window_spread;       /* 最近一拍窗内样本标准差（A） */
  uint16_t win_filled;
  uint16_t win_head;
} calib_s4_tracker_t;

/* target=阶跃终值，tc=PI 时间常数，window_cycles=稳态带判定滑窗宽，其余容差/倍数由 calib_config 传入。 */
void calib_sequence_s4_begin(calib_s4_tracker_t *t, float target_a, float tc_s,
                             float v_limit_v, float band_ratio,
                             uint16_t band_hold, uint16_t window_cycles,
                             uint16_t settle_tc,
                             uint16_t observe_tc);

/* 每个慢周期喂当前 Id/Vd 与自阶跃起 elapsed_ms；返回 1=观测结束可判定。 */
uint8_t calib_sequence_s4_update(calib_s4_tracker_t *t, float id_a, float vd_v,
                                 uint32_t elapsed_ms);

typedef struct
{
  uint8_t settle_ok;         /* 建立时间 < settle_tc*Tc */
  uint8_t steady_err_ok;     /* 末段静差 < 上限 */
  uint8_t no_osc_ok;         /* 建立后穿出占比与连续穿出数受限（慢振荡） */
  uint8_t saturation_ok;     /* 饱和拍数占比受限 */
  uint8_t spread_ok;         /* 建立后窗内离散度均值低于比例上限（快速对称振荡） */
  float settle_time_ms;
  float steady_err_ratio;
  float window_spread_ratio; /* 建立后窗内 std/目标值 的均值 */
  float exit_ratio;          /* 建立后穿出窗口占比 */
  uint16_t max_exit_run;     /* 建立后最长连续穿出窗口数 */
} calib_s4_result_t;

uint8_t calib_sequence_s4_evaluate(const calib_s4_tracker_t *t,
                                   float steady_err_ratio_max,
                                   float saturation_ratio_max,
                                   float spread_ratio_max,
                                   float exit_ratio_max,
                                   uint16_t exit_run_max,
                                   calib_s4_result_t *out);

/* ============ 线序 Phase-seq（6 换相静态 d 轴一致性探针） ============
 * 输入：6 种换相下，θ=0 注入 +Vd 后测得的 (Id,Iq) 稳态均值（下标与
 * control_loop 内换相表一致）。判据：正确换相（软件 A 输出落在真实 0° 绕组）
 * Id>0 且 |Iq|/Id 小；错误换相 Id<0 或 |Iq|/Id≈0.866。反射（A-B-C vs A-C-B）
 * 两项都可能通过，由后续方向步用旋转方向唯一分辨，故 pass_count 期望 2。 */
typedef struct
{
  uint8_t best_index;   /* 0..5，通过硬门的 argmax 评分 */
  uint8_t pass_count;   /* 通过硬门的换相个数（信息量） */
  uint8_t ok;           /* 至少 1 个通过硬门 */
  float best_id;        /* best 换相的 Id（A） */
  float best_iq;        /* best 换相的 Iq（A） */
  float best_score;     /* best 换相的评分 = Id - |Iq| */
} calib_phase_result_t;

/* 返回 1=至少 1 个换相通过（best_index 有效）。 */
uint8_t calib_sequence_phase_pick(const float id[6], const float iq[6],
                                  float id_min_a, float iq_ratio_max,
                                  calib_phase_result_t *out);

/* ============ 方向 encoder_direction（开环慢扫旋转方向判定） ============ */
typedef struct
{
  int8_t direction;     /* +1 或 -1 */
  uint8_t ok;           /* |mech_delta| 足够大，判定可信 */
  float mech_delta_rad; /* 正扫期间机械角净位移 */
} calib_direction_result_t;

/* 正扫（电角度递增）下机械角净位移为正 → direction=+1、为负 → -1；
 * |delta| 低于 min_delta 判跟踪失败（ok=0）。 */
uint8_t calib_sequence_direction_eval(float mech_delta_rad, float min_delta_rad,
                                      calib_direction_result_t *out);

/* ============ S5 零位 offset（多次正反向圆周统计） ============ */
typedef struct
{
  uint8_t ok;
  float offset_rad;        /* 圆周均值 */
  float circ_std_rad;      /* 圆周标准差 */
  float fwd_rev_diff_rad;  /* 正向均值与反向均值的圆周差 */
} calib_s5_result_t;

/* offsets 长度 n（每次对齐得到一个 offset，前 fwd_half 个正向、其余反向）；
 * 做圆周均值并判圆周标准差与正反差。fwd_half==0 时退化为仅全量统计。
 * 返回 1=全部判据通过。 */
uint8_t calib_sequence_s5_eval(const float *offsets, uint8_t n, uint8_t fwd_half,
                               float circ_std_max_rad, float fwd_rev_diff_max_rad,
                               calib_s5_result_t *out);

/* ============ S7 低速捕获动态 R 统计（SPEC-RUN FR-1.2，纯函数） ============
 * 输入：捕获稳态段按块分组的 Vq/Iq 块均值数组（建立段样本由 caller 剔除），
 * raw_samples 为原始有效样本总数（诊断打印用）。
 * 统计：块均值数组做截尾均值（两端各丢 trim_each 个块）替代全样本均值，
 * 抗个别块受换相/抖动污染；空样本与超窗分列独立状态码，不混为一谈。 */
typedef enum
{
  CALIB_S7_R_OK = 0,
  CALIB_S7_R_NO_SAMPLES,     /* 样本数为零/不足（区分于超窗丢弃） */
  CALIB_S7_R_BELOW_WINDOW,   /* 低于候选窗下界 */
  CALIB_S7_R_ABOVE_WINDOW    /* 高于候选窗上界 */
} calib_s7_r_status_t;

typedef struct
{
  calib_s7_r_status_t status;
  float dyn_r_ohm;           /* 截尾均值估计 */
  uint16_t block_count;      /* 参与统计的块数 */
  uint16_t raw_samples;      /* 原始有效样本总数（透传打印） */
  float window_lo_ohm;       /* 候选窗下界（base*lo_ratio） */
  float window_hi_ohm;       /* 候选窗上界（base*hi_ratio） */
  float base_r_ohm;          /* 窗基准=本次 S2 实测 R（FR-1.5 双源交叉） */
} calib_s7_r_result_t;

/* base_r_ohm 为候选窗基准（本次 S2 实测 R）；返回 status 并填充 out。 */
calib_s7_r_status_t calib_sequence_s7_dynamic_r(const float *block_mean,
                                                uint16_t block_count,
                                                uint16_t raw_samples,
                                                float base_r_ohm,
                                                float lo_ratio, float hi_ratio,
                                                uint16_t trim_each,
                                                calib_s7_r_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_SEQUENCE_H */
