/*
 * 半自动标定序列判据层实现（批次 A：S1-S4）。纯 C、零 HAL，见同名头文件。
 */
#include "calib_sequence.h"

#include "calib_config.h"
#include "calib_stats.h"

#include <math.h>
#include <stddef.h>

/* ---------------- S1a 爬坡扫描双极性线性拟合：符号/通道/相序/增益量级（raw 层） ---------------- */
uint8_t calib_sequence_s1a_fit(const float *vd_p, const float *du_p, const float *dv_p, uint8_t n_p,
                               const float *vd_n, const float *du_n, const float *dv_n, uint8_t n_n,
                               float a_per_raw, float r_nominal_ohm, float target_lo_code,
                               float lock_vd_v, calib_s1_result_t *out)
{
  /* 工作数组 static 化：本函数只在主循环 EVAL 单实例调用、不进中断、不可重入；
   * 避免在 1KB 共享栈（startup Stack_Size=0x400）上再叠加约 300B。 */
  static float pu_x[CALIB_S1A_LEVEL_MAX], pu_y[CALIB_S1A_LEVEL_MAX];
  static float nu_x[CALIB_S1A_LEVEL_MAX], nu_y[CALIB_S1A_LEVEL_MAX];
  static float mx[2U * CALIB_S1A_LEVEL_MAX], mu[2U * CALIB_S1A_LEVEL_MAX],
               mv[2U * CALIB_S1A_LEVEL_MAX];
  calib_line_t lu_p, lu_n, line_u, line_v;
  uint8_t i, np_eff, nn_eff, total;
  float abs_slope_p, abs_slope_n, slope_mean, i_per_v, r_est, max_du;

  if ((out == NULL) || (vd_p == NULL) || (du_p == NULL) || (dv_p == NULL) ||
      (vd_n == NULL) || (du_n == NULL) || (dv_n == NULL) ||
      (n_p < CALIB_S1A_MIN_LINEAR_PTS) || (n_n < CALIB_S1A_MIN_LINEAR_PTS) ||
      (a_per_raw <= 0.0f) || (r_nominal_ohm <= 0.0f))
  {
    return 0U;
  }

  out->sign_u = 0;
  out->sign_v = 0;
  out->linear_ok = out->reached_band = out->bipolar_ok = out->symmetry_ok = 0U;
  out->channel_ratio_ok = out->r_range_ok = 0U;
  out->pos_pts = 0U;
  out->neg_pts = 0U;
  out->slope_u_code_per_v = out->slope_v_code_per_v = out->intercept_u_code = 0.0f;
  out->r2_u = out->r2_v = out->ratio_vu = out->v_offset_v = 0.0f;
  out->symmetry = out->r_est_ohm = out->max_abs_du_code = 0.0f;
  out->lock_vd_v = lock_vd_v;

  /* 筛点：只保留 |du|≥MIN_USABLE 的档（u/v 同档同时纳入/丢弃），跳过被零偏 p2p
   * 噪声淹没的起步弱信号档，避免它们把最小二乘 r2 拉垮；扫描表仍由上层全量打印。 */
  np_eff = 0U;
  nn_eff = 0U;
  total = 0U;
  for (i = 0U; i < n_p; i++)
  {
    if (fabsf(du_p[i]) >= CALIB_S1A_MIN_USABLE_RAW)
    {
      pu_x[np_eff] = vd_p[i]; pu_y[np_eff] = du_p[i];
      mx[total] = vd_p[i]; mu[total] = du_p[i]; mv[total] = dv_p[i];
      np_eff++; total++;
    }
  }
  for (i = 0U; i < n_n; i++)
  {
    if (fabsf(du_n[i]) >= CALIB_S1A_MIN_USABLE_RAW)
    {
      nu_x[nn_eff] = vd_n[i]; nu_y[nn_eff] = du_n[i];
      mx[total] = vd_n[i]; mu[total] = du_n[i]; mv[total] = dv_n[i];
      nn_eff++; total++;
    }
  }
  /* 筛后有效点不足（起步弱信号档过多）直接判线性失败。 */
  if ((np_eff < CALIB_S1A_MIN_LINEAR_PTS) || (nn_eff < CALIB_S1A_MIN_LINEAR_PTS) ||
      (total < 2U))
  {
    return 0U;
  }

  /* 正/负方向各自拟合 U：用于双极性与对称性判据。带符号坐标里物理线性时两者斜率同号且接近。 */
  lu_p = calib_least_squares(pu_x, pu_y, (uint16_t)np_eff);
  lu_n = calib_least_squares(nu_x, nu_y, (uint16_t)nn_eff);

  /* 合并正/负筛后点（物理上应落在同一条过零直线 du=slope*Vd+intercept）。 */
  line_u = calib_least_squares(mx, mu, (uint16_t)total);
  line_v = calib_least_squares(mx, mv, (uint16_t)total);
  out->pos_pts = np_eff;
  out->neg_pts = nn_eff;
  out->slope_u_code_per_v = line_u.slope;
  out->slope_v_code_per_v = line_v.slope;
  out->intercept_u_code = line_u.intercept;
  out->r2_u = line_u.r2;
  out->r2_v = line_v.r2;

  /* 1) 线性度：合并拟合 r2 达标、斜率非零，否则是噪声堆出来的，不能定符号。 */
  if ((line_u.r2 < CALIB_S1A_FIT_R2_MIN) || (line_v.r2 < CALIB_S1A_FIT_R2_MIN) ||
      (fabsf(line_u.slope) < 1e-6f))
  {
    return 0U;
  }
  out->linear_ok = 1U;

  /* 扫描中最大 |du|（筛后合并点），判激励是否达到目标带（电流反馈主闸门的离线对应）。 */
  max_du = 0.0f;
  for (i = 0U; i < total; i++)
  {
    float a = fabsf(mu[i]);
    if (a > max_du) max_du = a;
  }
  out->max_abs_du_code = max_du;
  out->reached_band = (max_du >= target_lo_code) ? 1U : 0U;

  /* 2) 双极性：带符号坐标中正/负斜率必须同号（物理响应随 Vd 单调，正/负反相）。 */
  out->bipolar_ok = ((lu_p.slope * lu_n.slope > 0.0f) &&
                     (fabsf(lu_p.slope) > 1e-6f) && (fabsf(lu_n.slope) > 1e-6f)) ? 1U : 0U;

  /* 3) 对称性：正/负斜率幅值不对称度，越小越好（查死区/零漂）。 */
  abs_slope_p = fabsf(lu_p.slope);
  abs_slope_n = fabsf(lu_n.slope);
  slope_mean = (abs_slope_p + abs_slope_n) * 0.5f;
  out->symmetry = (slope_mean > 0.0f) ?
                  (fabsf(abs_slope_p - abs_slope_n) / slope_mean) : 1.0f;
  out->symmetry_ok = (out->symmetry <= CALIB_S1A_SYMMETRY_TOL) ? 1U : 0U;

  /* 4) 通道/相序：整条线斜率比 slope_v/slope_u=-1/2（整体反接同时反号、比值不变）。 */
  out->ratio_vu = line_v.slope / line_u.slope;
  out->channel_ratio_ok =
      (fabsf(out->ratio_vu - CALIB_S1_RATIO_TARGET) <= CALIB_S1_RATIO_TOL) ? 1U : 0U;

  /* 5) 符号取合并斜率符号；粗 R：slope[code/V]*a_per_raw[A/code]=A/V=1/R，取绝对值。 */
  out->sign_u = (line_u.slope > 0.0f) ? (int8_t)1 : (int8_t)-1;
  out->sign_v = (line_v.slope > 0.0f) ? (int8_t)1 : (int8_t)-1;
  i_per_v = fabsf(line_u.slope) * a_per_raw;
  r_est = (i_per_v > 0.0f) ? (1.0f / i_per_v) : 0.0f;
  out->r_est_ohm = r_est;
  out->r_range_ok = ((r_est >= CALIB_S1A_R_LOW_RATIO * r_nominal_ohm) &&
                     (r_est <= CALIB_S1A_R_HIGH_RATIO * r_nominal_ohm)) ? 1U : 0U;

  /* 死区/管压降等效电压：令 du=0 所需 |Vd| = |intercept/slope|。 */
  out->v_offset_v = fabsf(line_u.intercept / line_u.slope);

  return (uint8_t)(out->linear_ok && out->reached_band && out->bipolar_ok &&
                   out->symmetry_ok && out->channel_ratio_ok && out->r_range_ok);
}

/* ---------------- S1a 静态三角度探针 PROBE3：采样符号 + 通道对应（纯函数，电机不转） ---------------- */
uint8_t calib_sequence_s1_probe3(const float du3[3], const float dv3[3], calib_probe3_result_t *out)
{
  uint8_t k, ku, kv, kw, weak, sym_u, sym_v;
  float au, av, ru1, ru2, rv1, rv2, amp_mean, amp_diff;
  float iw[3], aw;

  if ((out == NULL) || (du3 == NULL) || (dv3 == NULL))
  {
    return 0U;
  }

  out->sign_u = out->sign_v = 0;
  out->peak_u_angle = out->peak_v_angle = 0U;
  out->channel_map_ok = out->symmetry_ok = out->third_phase_ok = out->amplitude_ok = 0U;
  out->amplitude_code = out->ratio_mean = 0.0f;
  out->fail_mask = 0U;
  out->hard_mask = 0U;
  out->ok = 0U;

  /* 找各通道 |偏移| 独大角度与幅值。 */
  ku = kv = 0U;
  au = fabsf(du3[0]);
  av = fabsf(dv3[0]);
  for (k = 1U; k < CALIB_S1A_P3_ANGLE_COUNT; k++)
  {
    if (fabsf(du3[k]) > au) { au = fabsf(du3[k]); ku = k; }
    if (fabsf(dv3[k]) > av) { av = fabsf(dv3[k]); kv = k; }
  }
  out->peak_u_angle = ku;
  out->peak_v_angle = kv;

  /* 1)【硬门】通道对应：U 独大必须在 0°(idx0)、V 独大必须在 120°(idx1)，否则两 CS 通道交叉/错位（硬件问题），方向不可信。 */
  out->channel_map_ok = ((ku == 0U) && (kv == 1U)) ? 1U : 0U;
  if (out->channel_map_ok == 0U)
  {
    out->fail_mask |= 0x02U;
    out->hard_mask |= 0x02U;
  }

  /* 2)【硬门】激励充分：两独大幅值都要高于零偏噪声带；独大值近零无法定方向/符号。 */
  weak = ((au < CALIB_S1A_MIN_USABLE_RAW) || (av < CALIB_S1A_MIN_USABLE_RAW) ||
          (fabsf(du3[ku]) < 1e-6f) || (fabsf(dv3[kv]) < 1e-6f)) ? 1U : 0U;
  if (weak != 0U)
  {
    out->fail_mask |= 0x01U;
    out->hard_mask |= 0x01U;
    return 0U; /* 弱信号下连方向都定不了，直接返回，不绑定 */
  }

  /* 3)【软门】三相对称：每个通道两个"非独大/独大"比值都应 ≈ -0.5（整体反接不改变比值）。
   *    死区/两相采样下小半幅点会偏小，只记质量、不阻断方向绑定。 */
  ru1 = du3[(ku + 1U) % 3U] / du3[ku];
  ru2 = du3[(ku + 2U) % 3U] / du3[ku];
  rv1 = dv3[(kv + 1U) % 3U] / dv3[kv];
  rv2 = dv3[(kv + 2U) % 3U] / dv3[kv];
  out->ratio_mean = (ru1 + ru2 + rv1 + rv2) * 0.25f;
  sym_u = ((fabsf(ru1 + 0.5f) <= CALIB_S1A_P3_RATIO_TOL) &&
           (fabsf(ru2 + 0.5f) <= CALIB_S1A_P3_RATIO_TOL)) ? 1U : 0U;
  sym_v = ((fabsf(rv1 + 0.5f) <= CALIB_S1A_P3_RATIO_TOL) &&
           (fabsf(rv2 + 0.5f) <= CALIB_S1A_P3_RATIO_TOL)) ? 1U : 0U;
  out->symmetry_ok = (sym_u && sym_v) ? 1U : 0U;
  if (out->symmetry_ok == 0U) out->fail_mask |= 0x04U;

  /* 4)【软门】第三相重构 Iw=-(Iu+Iv)，其独大"期望"落在 240°(idx2)；两相采样相加重构、误差叠加，
   *    只作基尔霍夫/采样自洽的质量记录，不阻断方向绑定。 */
  kw = 0U;
  aw = 0.0f;
  for (k = 0U; k < 3U; k++)
  {
    iw[k] = -(du3[k] + dv3[k]);
    if (fabsf(iw[k]) > aw) { aw = fabsf(iw[k]); kw = k; }
  }
  out->third_phase_ok = (kw == 2U) ? 1U : 0U;
  if (out->third_phase_ok == 0U) out->fail_mask |= 0x08U;

  /* 5)【硬门】幅值一致：U/V 独大应等幅（两通道增益/相电阻一致），差太多说明增益/接触有硬问题。 */
  amp_mean = (au + av) * 0.5f;
  out->amplitude_code = amp_mean;
  amp_diff = (amp_mean > 1e-6f) ? (fabsf(au - av) / amp_mean) : 1.0f;
  out->amplitude_ok = (amp_diff <= CALIB_S1A_P3_AMP_MATCH_TOL) ? 1U : 0U;
  if (out->amplitude_ok == 0U)
  {
    out->fail_mask |= 0x10U;
    out->hard_mask |= 0x10U;
  }

  /* 符号取各自独大值符号（整体反接时符号同时为负，比值判据不变，符号仍正确）。 */
  out->sign_u = (du3[ku] > 0.0f) ? (int8_t)1 : (int8_t)-1;
  out->sign_v = (dv3[kv] > 0.0f) ? (int8_t)1 : (int8_t)-1;

  /* 方向可信 = 硬门全清；对称度/第三相是软质量位，不阻断绑定（不陷入精度陷阱）。 */
  out->ok = (out->hard_mask == 0U) ? 1U : 0U;
  return out->ok;
}

/* ---------------- S2 相电阻三档最小二乘 ---------------- */
uint8_t calib_sequence_s2_evaluate(const float id3[3], const float vd3[3],
                                   float r_nominal_ohm, calib_s2_result_t *out)
{
  calib_line_t line;

  if ((id3 == NULL) || (vd3 == NULL) || (out == NULL))
  {
    return 0U;
  }

  line = calib_least_squares(id3, vd3, 3U);
  /* r2=-1 表示 x 无方差/样本非法；线性度、斜率正、标称区间三道门。 */
  if ((line.r2 < CALIB_S2_FIT_R2_MIN) || (line.slope <= 0.0f))
  {
    return 0U;
  }
  if ((line.slope < r_nominal_ohm * CALIB_S2_R_LOW_RATIO) ||
      (line.slope > r_nominal_ohm * CALIB_S2_R_HIGH_RATIO))
  {
    return 0U;
  }

  out->resistance_ohm = line.slope;
  out->v_offset_v = line.intercept;
  out->r2 = line.r2;
  return 1U;
}

/* ---------------- S3 相电感阶跃窗口 ---------------- */
uint8_t calib_sequence_s3_evaluate(const calib_sample_t *s, uint16_t n,
                                   uint16_t skip_head,
                                   float resistance_ohm, float ts_s,
                                   calib_s3_result_t *out)
{
  float l_samples[CALIB_STATS_MAX_SAMPLES];
  uint16_t i, count;
  float dt, did, didt, inductance, median, iqr;

  if ((s == NULL) || (out == NULL) || (n < 2U) || (ts_s <= 0.0f))
  {
    return 0U;
  }

  count = 0U;
  /* 差分从 skip_head+1 起：跳过阶跃当拍与死区，只取上升沿。 */
  for (i = (uint16_t)(skip_head + 1U); i < n; i++)
  {
    if (count >= CALIB_STATS_MAX_SAMPLES)
    {
      break;
    }
    dt = (float)(s[i].cycle - s[i - 1U].cycle) * ts_s;
    did = s[i].id - s[i - 1U].id;
    if (dt <= 0.0f)
    {
      continue;
    }
    didt = did / dt;
    /* 只取上升且斜率有意义的点，避免平台期噪声被放大成虚假 L。 */
    if (didt <= CALIB_S3_MIN_SLOPE_A_PER_S)
    {
      continue;
    }
    inductance = (s[i].vd - resistance_ohm * s[i].id) / didt;
    if (inductance > 0.0f)
    {
      l_samples[count] = inductance;
      count++;
    }
  }

  if (count < 3U)
  {
    return 0U;
  }

  calib_median_iqr(l_samples, count, &median, &iqr);
  if (median <= 0.0f)
  {
    return 0U;
  }

  out->inductance_h = median;
  out->iqr_ratio = iqr / median;
  if (out->iqr_ratio > CALIB_S3_L_IQR_RATIO_MAX)
  {
    return 0U;
  }
  if ((median < CALIB_S3_L_NOMINAL_H * CALIB_S3_L_LOW_RATIO) ||
      (median > CALIB_S3_L_NOMINAL_H * CALIB_S3_L_HIGH_RATIO))
  {
    return 0U;
  }
  return 1U;
}

/* ---------------- S4 电流环阶跃自检 ---------------- */
/* 把本拍 Id 压入环形滑窗并重算窗均值/样本标准差（W<=16，慢路径允许 O(W)+sqrtf）。 */
static void calib_s4_push_window(calib_s4_tracker_t *t, float id_a)
{
  uint16_t w = t->window_cycles;
  uint16_t i;
  float mean;
  float var_sum = 0.0f;

  if (w == 0U)
  {
    t->window_mean = id_a;
    t->window_spread = 0.0f;
    return;
  }
  if (w > CALIB_S4_WINDOW_MAX)
  {
    w = CALIB_S4_WINDOW_MAX;
  }
  if (t->win_filled < w)
  {
    t->win_buf[t->win_head] = id_a;
    t->win_sum += id_a;
    t->win_filled++;
  }
  else
  {
    t->win_sum -= t->win_buf[t->win_head];
    t->win_buf[t->win_head] = id_a;
    t->win_sum += id_a;
  }
  t->win_head = (uint16_t)((t->win_head + 1U) % w);
  mean = t->win_sum / (float)t->win_filled;
  t->window_mean = mean;
  for (i = 0U; i < t->win_filled; i++)
  {
    float d = t->win_buf[i] - mean;
    var_sum += d * d;
  }
  t->window_spread = (t->win_filled > 1U)
      ? sqrtf(var_sum / (float)(t->win_filled - 1U)) : 0.0f;
}

void calib_sequence_s4_begin(calib_s4_tracker_t *t, float target_a, float tc_s,
                             float v_limit_v, float band_ratio,
                             uint16_t band_hold, uint16_t window_cycles,
                             uint16_t settle_tc,
                             uint16_t observe_tc)
{
  uint16_t i;

  if (t == NULL)
  {
    return;
  }
  t->target_a = target_a;
  t->tc_s = tc_s;
  t->v_limit_v = v_limit_v;
  t->band_ratio = band_ratio;
  t->band_hold = band_hold;
  t->window_cycles = (window_cycles > CALIB_S4_WINDOW_MAX)
      ? CALIB_S4_WINDOW_MAX : window_cycles;
  t->settle_deadline_ms = (uint32_t)((float)settle_tc * tc_s * 1000.0f);
  t->observe_ms = (uint32_t)((float)observe_tc * tc_s * 1000.0f);
  t->in_band_run = 0U;
  t->settled = 0U;
  t->settle_time_ms = 0U;
  t->band_exit_count = 0U;
  t->exit_run = 0U;
  t->max_exit_run = 0U;
  t->post_windows = 0U;
  t->saturated_count = 0U;
  t->tail_err_sum = 0.0f;
  t->tail_n = 0U;
  t->tail_spread_sum = 0.0f;
  t->done = 0U;
  t->win_sum = 0.0f;
  t->window_mean = 0.0f;
  t->window_spread = 0.0f;
  t->win_filled = 0U;
  t->win_head = 0U;
  for (i = 0U; i < CALIB_S4_WINDOW_MAX; i++)
  {
    t->win_buf[i] = 0.0f;
  }
}

uint8_t calib_sequence_s4_update(calib_s4_tracker_t *t, float id_a, float vd_v,
                                 uint32_t elapsed_ms)
{
  float rel_err, abs_err;
  uint8_t window_ready;

  if (t == NULL)
  {
    return 0U;
  }
  if ((t->done != 0U) || (t->target_a == 0.0f))
  {
    return t->done;
  }

  /* 抗单拍噪声：稳态带进入/穿出与静差都吃窗均值；窗未填满前不判在带。 */
  calib_s4_push_window(t, id_a);
  window_ready = (t->window_cycles == 0U) ||
                 (t->win_filled >= t->window_cycles);
  abs_err = t->window_mean - t->target_a;
  rel_err = fabsf(abs_err) / fabsf(t->target_a);

  /* 稳态带进入/保持与建立时刻；建立后再穿出记振荡窗口。
   * 噪声只会造成孤立穿出（连续长度短、占比低），持续慢振荡表现为长连续穿出，
   * 两者用 max_exit_run / 穿出占比区分（evaluate 判），不再要求"零穿出"。 */
  if ((window_ready != 0U) && (rel_err <= t->band_ratio))
  {
    if (t->in_band_run < 0xFFFFU)
    {
      t->in_band_run++;
    }
    t->exit_run = 0U;
    if ((t->settled == 0U) && (t->in_band_run >= t->band_hold))
    {
      t->settled = 1U;
      t->settle_time_ms = elapsed_ms;
    }
  }
  else if (window_ready != 0U)
  {
    if (t->settled != 0U)
    {
      t->band_exit_count++;
      t->exit_run++;
      if (t->exit_run > t->max_exit_run)
      {
        t->max_exit_run = t->exit_run;
      }
    }
    t->in_band_run = 0U;
  }

  /* 饱和：输出电压顶到限幅比例以上（命令量无噪声，保持逐拍）。 */
  if (fabsf(vd_v) >= CALIB_S4_SATURATION_RATIO * t->v_limit_v)
  {
    t->saturated_count++;
  }

  /* 末半段累计静差（窗均值）。 */
  if ((window_ready != 0U) && (elapsed_ms >= (t->observe_ms / 2U)))
  {
    t->tail_err_sum += abs_err;
    t->tail_n++;
  }

  /* 建立后逐窗累计窗内离散度（spread 门限只看稳态段，避开上升瞬态大 spread）。 */
  if ((t->settled != 0U) && (window_ready != 0U))
  {
    t->post_windows++;
    t->tail_spread_sum += t->window_spread;
  }

  if (elapsed_ms >= t->observe_ms)
  {
    t->done = 1U;
  }
  return t->done;
}

uint8_t calib_sequence_s4_evaluate(const calib_s4_tracker_t *t,
                                   float steady_err_ratio_max,
                                   float saturation_ratio_max,
                                   float spread_ratio_max,
                                   float exit_ratio_max,
                                   uint16_t exit_run_max,
                                   calib_s4_result_t *out)
{
  float mean_tail_err, sat_ratio;

  if ((t == NULL) || (out == NULL))
  {
    return 0U;
  }

  out->settle_time_ms = (float)t->settle_time_ms;
  out->steady_err_ratio = 0.0f;
  /* 建立后各窗 std 的均值（只统计稳态段），无建立后窗口时为 0。 */
  out->window_spread_ratio = ((t->post_windows > 0U) && (t->target_a != 0.0f))
      ? (t->tail_spread_sum / (float)t->post_windows) / fabsf(t->target_a) : 0.0f;
  out->exit_ratio = (t->post_windows > 0U)
      ? (float)t->band_exit_count / (float)t->post_windows : 0.0f;
  out->max_exit_run = t->max_exit_run;
  out->settle_ok = 0U;
  out->steady_err_ok = 0U;
  out->no_osc_ok = 0U;
  out->saturation_ok = 0U;
  out->spread_ok = 0U;

  if ((t->done == 0U) || (t->target_a == 0.0f) || (t->observe_ms == 0U))
  {
    return 0U;
  }

  /* 建立时间：必须已建立且不晚于 deadline。 */
  out->settle_ok = (t->settled != 0U) &&
                   (t->settle_time_ms <= t->settle_deadline_ms) ? 1U : 0U;

  /* 末段静差（窗均值）。 */
  if (t->tail_n > 0U)
  {
    mean_tail_err = t->tail_err_sum / (float)t->tail_n;
    out->steady_err_ratio = fabsf(mean_tail_err) / fabsf(t->target_a);
  }
  out->steady_err_ok = (out->steady_err_ratio <= steady_err_ratio_max) ? 1U : 0U;

  /* 慢振荡：建立后允许噪声孤立穿出，但穿出占比与最长连续穿出必须受限；
   * 未建立（post=0）时不单独判 no_osc，由 settle_ok 兜底。 */
  if (t->post_windows == 0U)
  {
    out->no_osc_ok = t->settled;
  }
  else
  {
    out->no_osc_ok = ((t->max_exit_run <= exit_run_max) &&
                      (out->exit_ratio <= exit_ratio_max)) ? 1U : 0U;
  }

  /* 快速对称振荡：窗均值会把它平均掉，靠建立后窗内离散度均值抓
   * （spread_ratio_max<=0 关闭；门限必须高于噪声地板，见 calib_config 注释）。 */
  out->spread_ok = ((spread_ratio_max <= 0.0f) ||
                    (out->window_spread_ratio <= spread_ratio_max)) ? 1U : 0U;

  /* 饱和拍数占总观测拍数（慢路径 1ms 一拍，拍数≈ms）比例受限。 */
  sat_ratio = (float)t->saturated_count / (float)t->observe_ms;
  out->saturation_ok = (sat_ratio <= saturation_ratio_max) ? 1U : 0U;

  return (uint8_t)((out->settle_ok && out->steady_err_ok &&
                    out->no_osc_ok && out->saturation_ok &&
                    out->spread_ok) ? 1U : 0U);
}

/* ---------------- 线序 / 方向 / S5 零位：纯判定函数（批次 B） ---------------- */

/* 圆周角规约到 [0, 2pi)。 */
static float calib_circ_wrap_rad(float a)
{
  while (a < 0.0f) { a += CALIB_TWO_PI; }
  while (a >= CALIB_TWO_PI) { a -= CALIB_TWO_PI; }
  return a;
}

/* 圆周角差规约到 (-pi, pi]。 */
static float calib_circ_diff_signed(float a, float b)
{
  float d = a - b;
  while (d > CALIB_PI) { d -= CALIB_TWO_PI; }
  while (d <= -CALIB_PI) { d += CALIB_TWO_PI; }
  return d;
}

/* ---------------- 线序 Phase-seq：6 换相静态 d 轴一致性 ---------------- */
uint8_t calib_sequence_phase_pick(const float id[6], const float iq[6],
                                  float id_min_a, float iq_ratio_max,
                                  calib_phase_result_t *out)
{
  uint8_t i;
  int8_t best = -1;
  float best_score = -1.0e30f;

  if ((id == NULL) || (iq == NULL) || (out == NULL))
  {
    return 0U;
  }
  out->best_index = 0U;
  out->pass_count = 0U;
  out->best_id = 0.0f;
  out->best_iq = 0.0f;
  out->best_score = 0.0f;
  out->ok = 0U;

  for (i = 0U; i < 6U; i++)
  {
    float ab_iq = fabsf(iq[i]);
    float score;
    if ((id[i] < id_min_a) || (ab_iq > iq_ratio_max * fabsf(id[i])))
    {
      continue;
    }
    out->pass_count++;
    score = id[i] - ab_iq;  /* 正 Id 越大、|Iq| 越小越好 */
    if (score > best_score)
    {
      best_score = score;
      best = (int8_t)i;
    }
  }

  if (best < 0)
  {
    return 0U;
  }
  out->best_index = (uint8_t)best;
  out->best_id = id[best];
  out->best_iq = iq[best];
  out->best_score = best_score;
  out->ok = 1U;
  return 1U;
}

/* ---------------- 方向 encoder_direction ---------------- */
uint8_t calib_sequence_direction_eval(float mech_delta_rad, float min_delta_rad,
                                      calib_direction_result_t *out)
{
  if (out == NULL)
  {
    return 0U;
  }
  out->mech_delta_rad = mech_delta_rad;
  out->direction = 1;
  out->ok = 0U;
  if (fabsf(mech_delta_rad) < min_delta_rad)
  {
    return 0U;
  }
  out->direction = (mech_delta_rad >= 0.0f) ? (int8_t)1 : (int8_t)-1;
  out->ok = 1U;
  return 1U;
}

/* ---------------- S5 零位 offset 圆周统计 ---------------- */
uint8_t calib_sequence_s5_eval(const float *offsets, uint8_t n, uint8_t fwd_half,
                               float circ_std_max_rad, float fwd_rev_diff_max_rad,
                               calib_s5_result_t *out)
{
  float sum_sin = 0.0f, sum_cos = 0.0f;
  float mean_r, circ_std;
  uint8_t i;

  if ((offsets == NULL) || (out == NULL) || (n == 0U))
  {
    return 0U;
  }
  out->offset_rad = 0.0f;
  out->circ_std_rad = 0.0f;
  out->fwd_rev_diff_rad = 0.0f;
  out->ok = 0U;

  /* 全量圆周均值 */
  for (i = 0U; i < n; i++)
  {
    sum_sin += sinf(offsets[i]);
    sum_cos += cosf(offsets[i]);
  }
  out->offset_rad = calib_circ_wrap_rad(atan2f(sum_sin, sum_cos));
  mean_r = sqrtf(sum_sin * sum_sin + sum_cos * sum_cos) / (float)n;
  if (mean_r >= 1.0f)
  {
    circ_std = 0.0f;
  }
  else
  {
    float r = (mean_r < 0.001f) ? 0.001f : mean_r;
    circ_std = sqrtf(-2.0f * logf(r));
  }
  out->circ_std_rad = circ_std;

  /* 正/反向各自圆周均值（fwd_half 在 (0,n) 内才比较正反差）。 */
  if ((fwd_half > 0U) && (fwd_half < n))
  {
    float fs = 0.0f, fc = 0.0f, rs = 0.0f, rc = 0.0f;
    float fwd_ang, rev_ang;
    for (i = 0U; i < fwd_half; i++) { fs += sinf(offsets[i]); fc += cosf(offsets[i]); }
    for (i = fwd_half; i < n; i++) { rs += sinf(offsets[i]); rc += cosf(offsets[i]); }
    fwd_ang = calib_circ_wrap_rad(atan2f(fs, fc));
    rev_ang = calib_circ_wrap_rad(atan2f(rs, rc));
    out->fwd_rev_diff_rad = fabsf(calib_circ_diff_signed(fwd_ang, rev_ang));
  }
  else
  {
    out->fwd_rev_diff_rad = 0.0f;
  }

  out->ok = (uint8_t)((circ_std <= circ_std_max_rad) &&
                      (out->fwd_rev_diff_rad <= fwd_rev_diff_max_rad));
  return out->ok;
}
