/*
 * 半自动标定序列 S0-S8 的过程参数（激励档位、时长、容差、采样窗口）。
 *
 * 与 foc_experiment_config 的分工：
 *  - foc_experiment_config：本次实验允许的电流/电压"安全天花板"（多源取 min）；
 *  - calib_config：每个标定 step 用多大激励、多长、多严的"过程档位"。
 * 硬约束：本文件任何电流档位都必须 <= foc_experiment_config 给出的安全上限。
 *
 * 下列数值均为首轮起始值（待低压实测整定），集中在此，调阈值不改逻辑。
 */
#ifndef CALIB_CONFIG_H
#define CALIB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============ 通用 ============ */
#define CALIB_PI                       (3.14159265358979f)
#define CALIB_TWO_PI                   (2.0f * CALIB_PI)
#define CALIB_SETTLE_HOLD_CYCLES       (5U)    /* 连续 5 个慢周期在带内算稳定 */
#define CALIB_COLLECT_N                (8U)    /* 稳态后统计样本数 */
#define CALIB_STEP_TIMEOUT_MS          (2000U) /* 单步保护性超时 */

/* ---- 收敛判定抗单拍噪声（情况 A）：判定器吃滑动窗均值，电气门限不降 ----
 * 背景：单拍电流噪声 σ≈10~13mA，逐拍判 ±10mA 带等价掷硬币；
 * W 拍窗均值 σ/√W，W=8 时 ≈3.7~4.7mA，叠加快路径拍均值后更小。
 * 窗宽只增加 W ms 判定延迟，远小于 2000ms 超时预算。【待整定】 */
#define CALIB_SETTLE_WINDOW_CYCLES     (8U)    /* SETTLE 在带判定的滑动窗宽（1ms/拍） */
#define CALIB_S1B_SPREAD_MAX_A         (0.010f)/* S1b COLLECT 样本标准差上限，大信号下收紧，【待整定】 */
#define CALIB_S2_SPREAD_MAX_A          (0.010f)/* S2 每档同上，【待整定】 */
#define CALIB_S4_SETTLE_WINDOW_CYCLES  (8U)    /* S4 判稳滑窗：大信号+长观测，恢复标准 8 拍 */
#define CALIB_S4_SPREAD_RATIO_MAX      (0.30f) /* 建立后窗内 std/目标 均值上限，大信号收紧 */
#define CALIB_S4_EXIT_RATIO_MAX        (0.20f) /* 建立后穿出稳态带窗口占比上限 */
#define CALIB_S4_EXIT_RUN_MAX          (2U)    /* 建立后最多允许连续穿出窗口数 */

/* ============ 快路径采样窗口 ============ */
#define CALIB_CAPTURE_DEPTH_DEFAULT    (64U)   /* 64 帧 * 32B = 2KB */
#define CALIB_CAPTURE_STRIDE_DEFAULT   (4U)    /* S4/S7：每 4 拍录一帧 */
#define CALIB_CAPTURE_STRIDE_FAST      (1U)    /* S3 电感阶跃：每拍录 */

/* ============ S1a 开环双极性电压爬坡扫描：符号/通道/相序/增益量级（raw 层 + 线性拟合） ============
 * 方法：d 轴锁 theta=0，电压从 START 逐档加到 VEND，每档缓升→跳过建立→采集均值，
 *       得到一组 (Vd, du, dv)；正向扫到电流进目标带即收，回零后反向扫相同档数；
 *       EVAL 对正/负点做一阶最小二乘，斜率符号定 sign、斜率比定通道、斜率反推粗 R、
 *       截距反推死区/管压降等效电压。以"实测电流进带"为主闸门，VEND 仅作采样失效兜底。 */
#define CALIB_S1_ALIGN_CURRENT_A       (0.50f) /* S1b 闭环自证目标 Id，临界大信号压噪（0.2→0.5A，2026-09-13 工作点上调），【待整定】 */
#define CALIB_S1A_LEVEL_START_V        (0.05f) /* 爬坡起始电压档（大概率在非线性区，拟合跳过），【待整定】 */
#define CALIB_S1A_LEVEL_STEP_V         (0.03f) /* 每档电压增量，【待整定】 */
#define CALIB_S1A_LEVEL_MAX            (10U)   /* 单方向最多档数（=static 点缓冲维度） */
#define CALIB_S1A_VEND_V               (0.31f) /* 电压天花板=min(硬限流0.15A*Rmin2.3,软件0.5V)*0.9，兜底 */
#define CALIB_S1A_BASELINE_MS          (50U)   /* 零矢量基线时长，取通电态 raw 基线 */
#define CALIB_S1A_LEVEL_RAMP_MS        (3U)    /* 档间线性缓升，抑 di/dt 尖峰/吸动 */
#define CALIB_S1A_LEVEL_SETTLE_MS      (5U)    /* 到档后跳过建立（tau≈0.37ms）+躲开关瞬态 */
#define CALIB_S1A_LEVEL_COLLECT_MS     (35U)   /* 档内采集均值时长（主要为平均压噪声） */
#define CALIB_S1A_DECAY_MS             (15U)   /* 正→反方向之间回零衰减 */
#define CALIB_S1A_TOTAL_TIMEOUT_MS     (2500U) /* S1a 总看门狗（双向扫描约 1s），超时判故障 */
#define CALIB_S1A_TARGET_LO_RATIO      (0.65f) /* 目标带下沿=Imax*65%，达到即收正向（电流主闸门），【待整定】 */
#define CALIB_S1A_TARGET_HI_RATIO      (0.88f) /* 目标带上沿（整定参考：期望停留区间，当前进下沿即收） */
#define CALIB_S1A_HARD_I_RATIO         (1.00f) /* 实测电流达 Imax 立即停止本方向加压（硬过流） */
#define CALIB_S1A_RAIL_MARGIN_RAW      (100U)  /* ADC 撞轨裕量 code（按零偏非对称余量动态判） */
#define CALIB_S1A_MIN_LINEAR_PTS       (3U)    /* 单方向参与拟合的最少有效点数 */
#define CALIB_S1A_MIN_USABLE_RAW       (30.0f) /* 只把 |du|≥此值的档纳入拟合（高于零偏 p2p 噪声带），【待整定】 */
#define CALIB_S1A_FIT_R2_MIN           (0.98f) /* V-I 线性度下限，【待整定】 */
#define CALIB_S1A_SYMMETRY_TOL         (0.25f) /* 正/负斜率幅值不对称度上限，【待整定】 */
#define CALIB_S1A_R_LOW_RATIO          (0.5f)  /* 等效 R 允许 0.5~2 倍标称（粗 sanity，精确 R 归 S2） */
#define CALIB_S1A_R_HIGH_RATIO         (2.0f)
#define CALIB_S1B_IQ_MAX_A             (0.02f) /* S1b 闭环自证 Iq≈0 上限 */
#define CALIB_S1_RATIO_TARGET          (-0.5f) /* 期望 slope_v/slope_u = -1/2 */
#define CALIB_S1_RATIO_TOL             (0.15f) /* 比值容差，【待整定】 */
#define CALIB_S1_SETTLE_BAND_A         (0.015f)/* S1b：Id 进入目标±15mA 算稳定（随 0.5A 目标同步放宽，保持 ~3% 相对容差），【待整定】 */

/* ---- 手动对齐保持（仅诊断用：发 p 后锁 θ=0 持续注入固定 Vd，按 x 才停，供手转感受锁止/阻尼） ----
 * 1=诊断保持模式（不跑自动爬坡、不判据、不自动撤、不受总看门狗限时）；
 * 0=正常 S1a 流程。诊断结束保持 0。 */
#define CALIB_S1A_MANUAL_HOLD          (0U)
#define CALIB_S1A_HOLD_VOLTAGE_V       (1.5f) /* 保持注入电压（对齐 P3 档位 ≈0.45A；2026-09-13 由 0.15V 上调），【诊断】 */

/* ---- S1a 静态三角度相序/符号探针 PROBE3（电流对齐：电机锁定不转，自动解采样符号+校验通道） ----
 * 1=S1a 走 PROBE3（推荐主路径）；它只定两个采样通道符号 sign_u/v 与"U↔半桥1、V↔半桥2"通道
 * 对应及三相对称，不涉及电机旋转线序/编码器方向（那是批次 B 的 S5/S6）。
 * 依次在电角度 0°/120°/240° 各做"零矢量基线→注入→采集→回零衰减"，三角度签名一次性解算。 */
#define CALIB_S1A_PROBE3               (1U)
#define CALIB_S1A_P3_ANGLE_COUNT       (3U)    /* 0°/120°/240° 三角度 */
#define CALIB_S1A_P3_VOLTAGE_V         (2.0f)  /* 注入电压（大电流强信号压噪）：实测等效阻抗 U≈4Ω/V≈7.5Ω，2.0V→U≈500mA/310code、V≈267mA/165code，均<额定1A；台 CC≥1.2A 硬兜底，【待整定】 */
#define CALIB_S1A_P3_BASELINE_MS       (40U)   /* 每角度零矢量基线时长 */
#define CALIB_S1A_P3_SETTLE_MS         (20U)   /* 注入后跳过建立（τ≈0.37ms，余量充足） */
#define CALIB_S1A_P3_COLLECT_MS        (80U)   /* 注入稳态采集均值时长（大信号多平均压噪） */
#define CALIB_S1A_P3_DECAY_MS          (15U)   /* 换角度前回零衰减，抑 di/dt 与转子猛跳 */
#define CALIB_S1A_P3_RATIO_TOL         (0.10f) /* 非独大相/独大相应 ≈ -0.5 的容差，大信号收紧，【待整定】 */
/* 大信号下 U/V 独大幅值一致性良好（实测 2V 注入时比值≈0.89），收紧到 0.20。 */
#define CALIB_S1A_P3_AMP_MATCH_TOL     (0.20f) /* U/V 独大幅值一致性容差，大信号收紧，【待整定】 */

/* ============ S2 相电阻（三档最小二乘） ============ */
#define CALIB_S2_CURRENT_LEVELS_A      { 0.20f, 0.40f, 0.60f } /* 大信号三档，压噪 */
#define CALIB_S2_LEVEL_COUNT           (3U)
#define CALIB_S2_R_NOMINAL_OHM         (2.3f)
#define CALIB_S2_R_LOW_RATIO           (0.5f)  /* 允许 0.5~2 倍标称 */
#define CALIB_S2_R_HIGH_RATIO          (2.0f)
#define CALIB_S2_FIT_R2_MIN            (0.95f) /* 线性度下限，【待整定】 */
#define CALIB_S2_SETTLE_BAND_A         (0.010f)/* 每档 Id 进入±10mA 算稳定，【待整定】 */

/* ============ S3 相电感（阶跃窗口多点中位数） ============ */
#define CALIB_S3_L_NOMINAL_H           (0.00086f)
#define CALIB_S3_L_LOW_RATIO           (0.3f)  /* 电感离散大，放宽到 0.3~3 倍 */
#define CALIB_S3_L_HIGH_RATIO          (3.0f)
#define CALIB_S3_L_IQR_RATIO_MAX       (0.25f) /* IQR/中位数上限，【待整定】 */
#define CALIB_S3_STEP_FROM_A           (0.10f) /* 阶跃起点，大信号避开死区 */
#define CALIB_S3_STEP_TO_A             (0.50f) /* 阶跃终点，<= 安全电流上限 1A */
#define CALIB_S3_SKIP_HEAD_CYCLES      (2U)    /* 跳过阶跃当拍/死区的样本数 */
#define CALIB_S3_MIN_SLOPE_A_PER_S     (1.0f)  /* dId/dt 下限，只取上升沿，【待整定】 */

/* ============ S4 电流环阶跃自检 ============ */
#define CALIB_S4_STEP_FROM_A           (0.0f)
#define CALIB_S4_STEP_TO_A             (0.30f) /* 大信号阶跃目标 0.3A */
#define CALIB_S4_TC_S                  (0.010f)/* PI 时间常数 10ms */
/* 大信号强电流下压噪，S4 判据恢复正常严格度（实测 0.3A 阶跃 settle≈50ms, steady_err≈4%）。 */
#define CALIB_S4_SETTLING_BAND_RATIO   (0.15f) /* 稳态带 ±15% */
#define CALIB_S4_SETTLING_TIME_MAX_TC  (10U)   /* 建立时间 < 10*Tc=100ms */
#define CALIB_S4_STEADY_ERROR_RATIO    (0.15f) /* 末段累计静差 < 15% */
#define CALIB_S4_OBSERVE_TC            (15U)   /* 总观测时长 = 15*Tc = 150ms */
#define CALIB_S4_SATURATION_RATIO       (0.95f) /* |Vd|>=95%Vlim 记一饱和拍 */
#define CALIB_S4_ESTABLISH_MS           (100U)  /* 阶跃前在 FROM(=0) 建立时长 */
#define CALIB_S4_SAT_DUTY_MAX          (0.10f) /* 饱和拍占比上限 10% */

/* ============ S5 零位 offset（简化：1正1反两次 d 轴对齐取平均） ============ */
#define CALIB_S5_ALIGN_CURRENT_A       (0.30f) /* 大电流锁定转子，压噪 */
#define CALIB_S5_REPEAT_COUNT          (2U)    /* 1正1反，简化 */
#define CALIB_S5_CIRC_STD_MAX_RAD      (0.05f) /* 圆周标准差上限，放宽容错，【待整定】 */
#define CALIB_S5_FWD_REV_DIFF_MAX_RAD  (0.08f) /* 正/反向趋近差上限，放宽容错，【待整定】 */
#define CALIB_S5_ALIGN_SETTLE_MS       (300U)  /* 单次 d 轴对齐建立时长 */
#define CALIB_S5_REV_PREMOVE_MS        (150U)  /* 反向趋近前预扭转 */

/* ============ S6 方向（手动旋转模式：不上电，用户手转电机，固件读编码器角度变化） ============ */
#define CALIB_S6_MANUAL_TIMEOUT_MS      (15000U)/* 手动旋转超时 15s */
#define CALIB_S6_EXPECTED_POLE_PAIRS   (7U)

/* ============ 线序 phase-seq（6 换相静态 d 轴一致性探针） ============
 * 依次把 6 种换相写入 phase_map，θ=0 注入 +Vd，稳态读 (Id,Iq)：
 * 正确换相（软件 A 输出落在真实 0° 绕组）Id>0 且 |Iq|/Id 小；
 * 错误换相（±120°/B-C 交换）Id<0 或 |Iq|/Id≈0.866。 */
#define CALIB_PHASE_SEQ_PERM_COUNT      (6U)
#define CALIB_PHASE_SEQ_ALIGN_VOLTAGE_V (1.5f)  /* 注入 Vd：大电流强信号压噪，1.5V/(1.5×R)≈0.43A，远高于 ID_MIN=0.15A 与噪声带，【待整定】 */
#define CALIB_PHASE_SEQ_SETTLE_MS       (80U)   /* 注入后跳过建立，【待整定】 */
#define CALIB_PHASE_SEQ_COLLECT_MS      (80U)   /* 采样均值，【待整定】 */
#define CALIB_PHASE_SEQ_DECAY_MS        (25U)   /* 换相前回零衰减，【待整定】 */
#define CALIB_PHASE_SEQ_ID_MIN_A        (0.15f) /* 正确换相下 d 轴电流下限（随注入 1.5V 比例抬高：预期≈0.43A，门限 0.15A 留余量），【待整定】 */
#define CALIB_PHASE_SEQ_IQ_RATIO_MAX    (0.50f) /* |Iq|/Id 上限（错误换相≈0.866），【待整定】 */
#define CALIB_PHASE_SEQ_TIMEOUT_MS      (5000U) /* 6 换相 + 镜像验证总看门狗 */
#define CALIB_PHASE_SEQ_VERIFY_ANGLE_RAD (1.5707963f) /* 镜像验证探针角 θ=+90°：θ=0 时 b/c 相电流相等、镜像排列（下标 idx^1）读数相同原理上不可分；θ=+90° 阶跃后转子重定位方向直接暴露镜像 */
/* verify 判决：编码器角度增量（电角度 rad）。+90° 阶跃预期 Δelec=+1.571（换相与
 * encoder_direction 一致）或 −1.571（镜像）；落窗内才裁决，窗外 inconclusive。
 * 编码器是数字量，不受电流噪声影响——替代旧"瞬态 Iq 符号"判据（实测 ±0.03A，
 * 低于 0.15A 门限，永远 inconclusive）。 */
#define CALIB_PHASE_SEQ_VERIFY_MIN_DELEC_RAD (0.70f)  /* |Δelec| 下限（≈40°），低于此值=转子未跟踪/编码器异常，【待整定】 */
#define CALIB_PHASE_SEQ_VERIFY_MAX_DELEC_RAD (2.40f)  /* |Δelec| 上限（≈137°），超出=角度链异常，【待整定】 */
/* verify 探针时序：best 换相先在 θ=0 重新对齐并记录编码器基准角 -> 矢量切到
 * θ=+90° 保持 V_HOLD_MS，让转子机械就位（预期仅转 90°/7≈13° 机械角，行程安全）
 * -> 再读编码器角求增量。 */
#define CALIB_PHASE_SEQ_V_ALIGN_MS      (80U)   /* θ=0 重新对齐保持时长，【待整定】 */
#define CALIB_PHASE_SEQ_V_HOLD_MS       (300U)  /* +90° 保持时长（远大于机械时间常数，确保转子就位），【待整定】 */

/* ============ 方向 encoder_direction（手动旋转判定：不上电，读编码器角位移符号） ============ */
#define CALIB_DIR_MIN_MECH_DELTA_RAD    (0.30f) /* 手动旋转最小位移（约17°），高于编码器抖动，【待整定】 */

/* ============ S7 低速编码器闭环交叉验证 ============ */
/* SPEC-RUN FR-1.1：工作点移出小信号区（<0.1A 死区/管压降主导，见 2.2/2.3 节）。
 * 【2026-09-13 二次整定（owner 拍板"临界大信号"）】id 0.10→0.40A：测量轴信号
 * 最大化（R·id≈1.3V，稀释 EMF 经角度滞后的 d 轴泄漏占比）+ 增强磁场阻尼；
 * iq 0.18→0.30A：|I|=0.50A 顶工程窗口上沿；iq 不再高——转速/EMF/滞后角均随
 * iq 增长，弊大于利（>0.6A 进磁饱和+积热区，见 20260910 复盘"不迷信大信号"）。 */
#define CALIB_S7_ID_TARGET_A           (0.40f) /* 低速闭环 Id，【待整定】 */
#define CALIB_S7_IQ_DRIVE_A            (0.30f) /* 低速闭环 Iq 驱动，【待整定】 */
/* 累计门限与新工作点联动（FR-1.1：禁止"目标 0.18A、门限 0.01A"形同虚设）。
 * 【判据 v2 注记 2026-09-13】DYN_R_MIN_IQ_RATIO 与 STAT_BLOCK_MS/STAT_BLOCKS
 * 属旧"瞬时值分块统计"路线，已被 S7 v2 窗均值判据取代（不再被 control_loop
 * 引用）；宏保留供 calib_s7_dyn_r_* 的 PC 单测与历史追溯。 */
#define CALIB_S7_DYN_R_MIN_IQ_RATIO    (0.5f)
#define CALIB_S7_DYN_R_LOW_RATIO       (0.5f)  /* 动态 R 候选窗口（基准=本次 S2 实测 R，FR-1.5 双源交叉） */
#define CALIB_S7_DYN_R_HIGH_RATIO      (1.5f)
/* FR-1.2：剔除建立段（转速/电流未稳不计入）。 */
#define CALIB_S7_SETTLE_SKIP_MS        (500U)  /* 捕获开始后跳过的建立段，【待整定】 */
#define CALIB_S7_STAT_BLOCK_MS         (80U)   /* 【旧判据遗留，见上注记】 */
#define CALIB_S7_STAT_BLOCKS           (24U)   /* 【旧判据遗留，见上注记】 */

/* ---- S7 判据 v2（2026-09-13 L4 结案：混叠实锤 + Vq/Iq 被 EMF 污染） ----
 * 判据改为三道门：①行程/方向（不变）；②调节品质——建立段后全程窗均值
 * wid/wiq 应贴目标（窗均值=tick 均值的 50ms 窗平均，天然滤掉 5ms 台阶激起
 * 的 200Hz 振荡混叠）；③动态 R 改用 EMF-free 的 d 轴：vd=R*id-ωL*iq（交叉项
 * ~0.01V 可忽略，EMF 只在 q 轴），取每窗 wvd/wid 比值的中位数对标 S2 静态 R，
 * 抗离群（FR-1.2 精神）。均【待整定】。 */
#define CALIB_S7_WINDOW_COUNT          (50U)   /* 2500ms/50ms 捕获窗数上限（判据用） */
#define CALIB_S7_WRATIO_MIN_WINDOWS    (5U)    /* 参与判决的最少有效窗数 */
#define CALIB_S7_WRATIO_MIN_WID_A      (0.02f) /* 窗 wid 低于此不产生比值（防除微小数） */
#define CALIB_S7_REG_ID_BAND_A         (0.05f) /* 调节品质门：|mean(wid)-ID_TARGET|<=带 */
#define CALIB_S7_REG_IQ_MIN_RATIO      (0.70f) /* 调节品质门：mean(wiq)>=RATIO*IQ_DRIVE */
/* FR-1.3：S7 力矩方向矛盾判定——iq>0 驱动下机械位移应与 encoder_direction 同号，
 * 且必须有可测行程；行程阈值与方向判定阈值对齐 CALIB_DIR_MIN_MECH_DELTA_RAD。 */
#define CALIB_S7_MIN_MECH_TRAVEL_RAD   (0.30f) /* 判"转子确实在转"的最小机械行程，【待整定】 */
/* 失步早期检测（误差型贴轨的上游拦截：错误电角度下 Iq 永远追不到目标、PI 输出
 * 持续贴限；在电流冲到 ADC 贴轨之前温柔撤功率，不进硬件 FAULT）。两条件同时
 * 成立并保持 HOLD 时长才判失步，建立段内豁免。 */
#define CALIB_S7_LOS_IQ_ERR_A          (0.12f) /* |Iq目标-Iq实测| 跟踪误差门限，【待整定】 */
#define CALIB_S7_LOS_VSAT_RATIO        (0.95f) /* |Vq| >= 电压限幅×该比例视为输出贴限，【待整定】 */
#define CALIB_S7_LOS_HOLD_MS           (80U)   /* 双条件同时持续时长，抗单拍抖动，【待整定】 */
#define CALIB_S7_DBG_PERIOD_MS         (100U)  /* S7 遥测+窗结算周期（2026-09-13 50->100ms：串口阻塞打印 ~9ms/行是新的角度过期源，降占空比 16%->8%；窗数 2.5s/100ms=25>=判据下限 5） */

/* ============ S2 单次可信门补充（FR-1.5 第 1 级） ============ */
#define CALIB_S2_VOFF_MAX_V            (0.10f) /* V-I 拟合截距（死区/管压降等效）上限，超出判本次 S2 不可信，【待整定】 */

/* ============ 相电阻 R 纯软件自学习四级门初值（FR-1.5，均【待整定】） ============ */
/* 1 单次可信门 = S2 r2/|Voff|/样本数（r2 与样本数已由 S2 引擎保证，Voff 见上）；
 * 2 跨启动重复门：最近 N 次可信 S2 R 的中值绝对偏差/中值 <= 10% 才形成 learned_R；
 * 3 双源交叉门：S7 动态 R 与本次 S2 实测 R 偏差 <= 25% 才共同采信；
 * 4 失控护栏：learned_R 相对编译期兜底标称 2.3R 偏离不超过 2 倍（对称 1/2~2 倍），
 *   越界只记录不采纳；c=clear 清学习值回兜底。 */
#define CALIB_RLEARN_HISTORY_N         (3U)
#define CALIB_RLEARN_REPEAT_DEV_MAX    (0.10f)
#define CALIB_RLEARN_CROSS_DEV_MAX     (0.25f)
#define CALIB_RLEARN_GUARD_RATIO       (2.0f)

/* VJ 判决工具已完成 L4 结案使命（调试日志经验8），2026-09-13 ROM 瘦身整体退役；
 * 重新启用改 1（control_loop 请求入口/状态机与 app 的 v 命令、banner 同步恢复）。 */
#define CALIB_VJ_DIAG_ENABLE           (0U)

#if CALIB_VJ_DIAG_ENABLE
/* ============ VJ 电压注入判决模式（L4 诊断工具，2026-09-13） ============
 * 独立于 S0-S8 标定序列：静止/旋转下按命令注入单轴开环电压，读 (id,iq)
 * 响应构成 2x2 矩阵 M，判决电流测量系与电压输出系的帧一致性（S7 失控 L4
 * 挂账的定位实验）。不产生候选、绝不落盘；注入电流 ~0.3/R < 100mA。 */
#define CALIB_VJ_INJECT_V              (0.30f) /* 单轴注入电压幅值（0.3/3.62Ω≈83mA），【待整定】 */
#define CALIB_VJ_PULSE_MS              (200U)  /* 静止注入脉冲总长 */
#define CALIB_VJ_PULSE_SKIP_MS         (100U)  /* 脉冲内跳过建立段后再累计均值，【待整定】 */
#define CALIB_VJ_SWEEP_MS              (2500U) /* g 旋转扫描时长（对齐 S7 捕获窗） */
#define CALIB_VJ_SWEEP_TLM_MS          (50U)   /* 扫描遥测打印周期 */
/* 编码器门禁去抖：连续无效达此时长才判真故障；期间保持上一帧角度继续运行。
 * 背景 2026-09-13 上板：使能瞬间栅驱 boost 瞬态干扰一次 I2C 读数（cache 单次
 * 失败即 valid=0），零去抖门禁在上电第 1ms 误判 FAULT。 */
#define CALIB_VJ_ENC_INVALID_FAULT_MS  (60U)
#endif /* CALIB_VJ_DIAG_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* CALIB_CONFIG_H */
