/*
 * FOC 电流系统数学库实现。
 *
 * 计算链路：两通道 ADC 原始码 -> 各通道零偏差值 ->（乘预算系数 code_to_amp）
 * -> 通道符号修正 -> 第三通道重构(-ch0-ch1) -> 按 phase_map 路由到逻辑相。
 * code_to_amp = ref/fullscale/(shunt*gain) 在参数装载阶段一次算好
 * (foc_parameter_set_precompute)，10kHz 快路径不再做任何浮点除法；
 * 通道符号 sign 晚于预算确定，故这里只做零成本的条件取反（不乘 float sign）。
 * calibrated=0 或预算系数缺失时拒绝输出有效数据。
 */

#include "foc_current_model.h"

static float foc_current_raw_to_amp(float raw,
                                    float zero_raw,
                                    float sign,
                                    float code_to_amp)
{
  float amp = (raw - zero_raw) * code_to_amp;
  /* sign 仅取 +1/-1：取反只翻转符号位，比一次 soft-float 乘法便宜得多。 */
  return (sign < 0.0f) ? -amp : amp;
}

foc_status_t foc_current_model_convert(const foc_current_calibration_t *calibration,
                                       const foc_raw_current_sample_t *raw,
                                       const foc_phase_map_t *phase_map,
                                       foc_current_sample_t *sample)
{
  float ch[3];
  uint8_t map_a = 0U;
  uint8_t map_b = 1U;

  if ((calibration == 0) || (raw == 0) || (sample == 0))
  {
    return FOC_STATUS_INVALID_ARG;
  }
  if ((calibration->calibrated == 0U) ||
      (calibration->adc_full_scale_raw <= 0.0f) ||
      (calibration->shunt_resistance_ohm <= 0.0f) ||
      (calibration->amplifier_gain <= 0.0f) ||
      (calibration->code_to_amp <= 0.0f))
  {
    sample->valid = 0U;
    return FOC_STATUS_NOT_READY;
  }

  /* 第一步：按通道各自的零偏/符号求通道电流；第三通道由基尔霍夫重构。 */
  ch[0] = foc_current_raw_to_amp((float)raw->channel_u_raw,
                                 calibration->channel_u_zero_raw,
                                 calibration->channel_u_sign,
                                 calibration->code_to_amp);
  ch[1] = foc_current_raw_to_amp((float)raw->channel_v_raw,
                                 calibration->channel_v_zero_raw,
                                 calibration->channel_v_sign,
                                 calibration->code_to_amp);
  ch[2] = -ch[0] - ch[1];

  /* 第二步：测量侧线序映射——与 foc_runtime 的 PWM 输出侧重排对称。
   * 逻辑相 x 的电流 = 通道 phase_map[x] 的电流。排列置换下
   * ic = -ia-ib 恒等于 ch[map_c]（三通道电流和为零），无需显式查 map_c。
   * phase_map 为空或 A/B 映射非法（越界/相同）时回退恒等 = 历史行为。 */
  if (phase_map != 0)
  {
    uint8_t pa = phase_map->phase_a_output;
    uint8_t pb = phase_map->phase_b_output;
    if ((pa <= 2U) && (pb <= 2U) && (pa != pb))
    {
      map_a = pa;
      map_b = pb;
    }
  }
  sample->phase_current_a.phase_a = ch[map_a];
  sample->phase_current_a.phase_b = ch[map_b];
  sample->phase_current_a.phase_c = -sample->phase_current_a.phase_a -
                                    sample->phase_current_a.phase_b;
  sample->valid = 1U;
  return FOC_STATUS_OK;
}
