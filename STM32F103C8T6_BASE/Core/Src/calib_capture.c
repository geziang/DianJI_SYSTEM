/*
 * 标定快路径定长采样窗口实现。缓冲为静态分配，录满后 push 自动停摆。
 */
#include "calib_capture.h"

static calib_sample_t calib_capture_buffer[CALIB_CAPTURE_MAX_DEPTH];
static uint16_t calib_capture_depth;
static uint16_t calib_capture_stride;
static uint16_t calib_capture_head;
static uint16_t calib_capture_skip;
static uint8_t calib_capture_full;

void calib_capture_reset(uint16_t depth, uint16_t stride)
{
  calib_capture_depth = (depth > CALIB_CAPTURE_MAX_DEPTH) ? CALIB_CAPTURE_MAX_DEPTH : depth;
  calib_capture_stride = (stride == 0U) ? 1U : stride;
  calib_capture_head = 0U;
  calib_capture_skip = 0U;
  calib_capture_full = 0U;
}

void calib_capture_push(const foc_control_output_t *output, uint32_t pwm_cycle)
{
  calib_sample_t *slot;

  /* 快路径：先判停、判空，O(1) 返回，不做任何浮点或循环。 */
  if ((calib_capture_full != 0U) || (output == 0) || (calib_capture_depth == 0U))
  {
    return;
  }
  if (calib_capture_skip > 0U)
  {
    --calib_capture_skip;
    return;
  }

  slot = &calib_capture_buffer[calib_capture_head];
  slot->id = output->measured_current_a.d;
  slot->iq = output->measured_current_a.q;
  slot->vd = output->voltage_command_v.d;
  slot->vq = output->voltage_command_v.q;
  slot->ia = output->phase_current_a.phase_a;
  slot->ib = output->phase_current_a.phase_b;
  slot->ic = output->phase_current_a.phase_c;
  slot->cycle = pwm_cycle;

  ++calib_capture_head;
  if (calib_capture_head >= calib_capture_depth)
  {
    calib_capture_head = calib_capture_depth;
    calib_capture_full = 1U;
  }
  else
  {
    /* 本帧已录，接下来跳过 stride-1 拍。 */
    calib_capture_skip = (uint16_t)(calib_capture_stride - 1U);
  }
}

uint16_t calib_capture_count(void)
{
  return (calib_capture_full != 0U) ? calib_capture_depth : calib_capture_head;
}

uint8_t calib_capture_is_full(void)
{
  return calib_capture_full;
}

const calib_sample_t *calib_capture_get(void)
{
  return calib_capture_buffer;
}
