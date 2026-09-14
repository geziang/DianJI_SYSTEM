# 08 电流环 ISR 计算重构需求与详细设计

- 版本：F103-identification-test 配套，固件重构版本（对应 identification-test-0.4 之后）
- 对象：STM32F103C8T6（Cortex-M3，72MHz，**无 FPU**），10kHz 电流环 ISR
- 关联：07_收敛判定抗单拍噪声需求（FR1–FR6，本次不改动，仅共存）
- 状态：代码已按本文落地；本机无 C 编译器，**尚未实编译/上板**，验证状态见第 8 节

---

## 1. 背景与根因

S1a（开环 VOLTAGE）多次通过，S1b（闭环 CURRENT）稳定停在：

```
[DBG] sp:enter
[DBG] sp:cfg ok
[DBG] sp:sync ok
[D            <- 卡死/复位
```

根因不是数学死循环、不是 HardFault、不是看门狗、不是栈溢出（均已逐一排除），而是：

1. `foc_runtime_start()` 成功返回并把状态置 RUNNING 后，全机最高优先级 `(0,0)` 的
   DMA1_Channel1 ISR 第一次以 **CURRENT 模式**跑完整 soft-float FOC；
2. M3 无 FPU，每条 float 都是软浮点库调用：fmul≈25cyc、**fdiv≈110cyc**、
   sinf/cosf（含区间归约）≈700cyc、sqrtf≈200cyc；
3. 重构前 CURRENT 一拍约 4090cyc≈**57µs**，慢路径 90–110µs，越过 100µs 周期后
   ISR tail-chain 连续触发、永不归还 CPU；SysTick（优先级 15）被压制，HAL_GetTick
   冻结，UART 100ms 超时也永不触发，于是永久停在刚发出的 "[D"。
4. S1a 走 VOLTAGE 分支跳过两段 PI（约省 5µs），恰在临界线内，故"S1a 全过、S1b 全死"。

附带发现两个真实缺陷（本次一并修）：

- **启动顺序错误**：原顺序先 `foc_runtime_start()`（置 RUNNING、ISR 开跑），之后才
  `set_forced_angle / set_current_target / drv_enable`，ISR 头约 1.4ms 用上一步残留
  角度与零目标运行；
- **栈全局超标**：Keil 栈报告 Maximum Stack Usage≈1064B > 1KB（0x400）。

## 2. 设计原则（四条）

1. **按时间尺度分层**：编译期常量 / 一次测试内常量 / 慢变量 / 每拍变量；
2. **每拍路径只留每拍变量**：其余外提到初始化、prepare、derive；
3. **数学等价重构**：除法变乘倒数、常量合并、提取公因子、惰性求值；
4. **精度预算内近似**：查表 + 插值，误差远小于 12bit ADC 的 1 LSB（1 code≈1.612mA）。

保护逻辑（PI 抗积分饱和、分量/矢量限幅、ADC valid、SVPWM 0..ARR 钳制、闭环门禁、
FR1 拍均值）逐行保留，不参与"减负"。

## 3. 预算量与失效条件（谁变就何时重算）

| 预算量 | 含义 | 计算位置 | 失效/重算条件 |
|---|---|---|---|
| `current.code_to_amp` | ref/fullscale/(shunt*gain)，A/code，**不含 sign** | prepare→precompute | 增益/参考/零偏链变；sign 翻转**不需要**重算 |
| `inv_bus_voltage_v` | 1/bus | 同上 | bus 在线改变（当前自检恒定） |
| `pwm_period_f` | (float)ARR | 同上 | PWM 频率改变 |
| `id/iq_pi.ki_times_t` | ki*T | 同上 | ki 或控制周期改变 |

**为什么 code_to_amp 不含 sign**：相序符号要晚到 S1a/PROBE3 结果处
（control_loop.c 写 phase_a/b_sign）才翻转，而预算在 S0 通过后的 prepare 就完成。
故快路径用 `(raw-zero)*code_to_amp` 后，对 sign 做零成本条件取反；S1b 启动前
`start_power` 会重新 `foc_runtime_configure` 整体拷贝，把新 sign 带入运行时，
无需在 sign 写入点重算任何预算。

## 4. 逐项改动清单

### 4.1 类型与预算（新增/字段）
- `foc_types.h`：`foc_pi_config_t` 增 `ki_times_t`；`foc_current_calibration_t` 增 `code_to_amp`。
- `foc_params.h`：`foc_parameter_set_t` 增 `inv_bus_voltage_v`、`pwm_period_f`。
- **新增 `foc_precompute.h/.c`（纯函数，不依赖 HAL/board_config，PC 可测）**：
  `foc_parameter_set_precompute(p, loop_period_s)`，集中计算上表 4 类预算量；
  基础量非法返回 `FOC_STATUS_NOT_READY` 且不写半成品。周期由调用方传入，
  故纯数学层不包含板级头。

### 4.2 快路径等价重构
- `foc_current_model.c`：两相各 2 次 fdiv+多 mul → `(raw-zero)*code_to_amp`，
  sign 用条件取反（翻符号位，替代一次 fmul）；门禁增 `code_to_amp>0`。
- `foc_pi.c/.h`：`foc_pi_update(state,error,T)` → `foc_pi_update(state,error)`，
  `ki*e*T` → `ki_times_t*e`；抗积分饱和、输出限幅原样保留。
- `foc_svpwm.c/.h`：签名改为收 `inv_bus_voltage_v, pwm_period_f`；
  3 次 `/bus` → 乘倒数，3 次 `(float)ticks` → 乘预算值；
  零序 `(max+min)/2` 注入**保留不删**（删了会改变注入电压、污染标定）。
- `foc_controller.c`：
  - 矢量限幅改**惰性开根**：先比 `d²+q² > lim²`，仅超限才一次 sqrtf；
  - 两处 PI 调用去周期参数；SVPWM 调用传预算值；
  - 就绪门禁增 `inv_bus>0 / pwm_period_f>0 / code_to_amp>0`（防忘记预算）。
- `foc_transform.c`：Clarke `2.0f*b` → `b+b`；`foc_sin_cos` 改调快表。

### 4.3 三角函数快表（精度内近似）
- **新增 `foc_trig_table.h/.c`**：256 点 Q15 正弦表（512B Flash）+ 线性插值；
  cos 由索引偏移 1/4 周期得到；角度→相位用一次乘法 + float→long 截断 +
  一次比较修正负角，**不调用 floorf/fmod**；四整格点 0/π/2/π/3π/2 直接返回精确
  0/±1，保证自检恒角 θ=0 无量化残差。

### 4.4 装载与启动
- `control_loop.c`：
  - `prepare_test_parameters` 在 derive PI 之后、configure 之前调用 precompute
    （每步 start 都会重新 configure，预算随之带入）；
  - **启动顺序修正**：`start_power` 与 `start_openloop_power` 统一改为
    configure → adc_sync → set_angle/set_target(或零电压) → drv_enable →
    **最后** `foc_runtime_start()` 置 RUNNING。READY 态 ISR 回调首行即返回，
    提前设置无副作用；开环在 sync 前置 VOLTAGE 的门禁前提保留。

### 4.5 配套
- `main.c`：DMA1_Channel1 优先级 `(0,0)` → `(2,0)`，不再占全机唯一最高，
  0/1 留给更紧急故障/保护；控制 ISR 仍高于 SysTick(15)，属常规实时设计。
- `startup_stm32f103xb.s`：`Stack_Size 0x400 → 0x800`（RAM 仅用约 6.7KB/20KB）。
- `STM32F103C8T6_BASE.uvprojx`：登记新增 `foc_trig_table.c`、`foc_precompute.c`
  到 Application/User/Core 组（否则 Keil 不编译）。
- `foc_runtime.c:214` 的 `input.control_period_s=...` **保留**：它仍是
  controller_step 的"周期合法"门禁，只是不再参与 PI 运算（一次 float store，非瓶颈）。

## 5. 周期账（估算，待上板 GPIO 实测终验）

| 模块 | 重构前 | 重构后 |
|---|---|---|
| 电流模型（两相） | ~600 cyc（4 fdiv） | ~60 cyc（2 fmul+取反） |
| sin/cos | ~1400 cyc（sinf+cosf+rredf2） | ~150 cyc（查表+插值） |
| 矢量限幅 sqrtf | ~220 cyc/拍 | 0（不超限）/ ~220（仅超限） |
| SVPWM | ~600 cyc（3 fdiv+3 i2f） | ~250 cyc（6 fmul） |
| 两段 PI | ~320 cyc（含 ki*T） | ~200 cyc |
| Clarke/Park/iPark | ~350 cyc | ~350 cyc（公式已最简） |
| HAL+AAPCS 等 | ~600 cyc | ~500 cyc |
| **CURRENT 一拍合计** | **~4090 cyc ≈57µs** | **~1510 cyc ≈21µs** |

10kHz 周期 100µs，重构后占用约 21%，留出约 4.7 倍余量；目标最坏拍 ≤30µs。

## 6. 精度预算

- 12bit ADC：1 code = 3.3/4095/(0.01×50) ≈ **1.612mA**，这是精度下限参照。
- Q15 256 点 + 线性插值：对 libm sinf/cosf 最大绝对误差 **≈8.17e-5**（20 万随机角，
  覆盖 −π..3π），远小于 1 LSB 折算量；四整格点精确。
- 数学等价非浮点逐位等价：电流模型、PI、SVPWM 的"预算 vs 现算"数值差见第 8 节，
  均在 float 机器精度或取整边界内。

## 7. 明确不做（边界）

- 不做全链 Q15/Q17 定点（仅三角表用 Q15，主链仍 float）——列为第四阶段后续；
- 不降电流环采样率（保持 10kHz，重构后已有约 5 倍余量）；
- 不改 Keil 优化等级（保持当前 Optim，用手动预算控制，避免全局代码生成变化）；
- 不改 PI 的 kp/ki 数值、电气门限、S0 的 32-code 门、FR1–FR6、硬件；
- 不做在线零矢量零偏估计、不改 PROBE3 基线/联合拟合。

## 8. 验证状态与方法（如实披露）

**已完成（本机）**：
- 全部改动文件花括号/小括号配平静态检查：16 文件 0 问题；
- 旧签名残留全局核查：ISR 快路径已无 sinf/cosf，sqrtf 仅剩矢量限幅超限分支；
  PI/SVPWM/电流模型调用点签名已全部一致；慢路径 calib_* 中的三角/开根本就允许，未动；
- Python 数值旁证（等价复刻重构前后公式，非 C 编译）：
  - 查表 sin/cos 最大误差 8.165e-5 < 1.2e-4 预算，四整点精确；
  - 电流模型新旧最大差 1.1e-16（机器精度）；
  - PI 12 步序列新旧最大差 2.8e-17；
  - SVPWM 5 万随机 αβ，新旧三相 ticks 差恒为 0；
  - code_to_amp=1.61172e-3 A/code，与板级 1.612mA/code 一致。

**尚未完成（本机无 gcc/armcc，无法做）**：
- 固件 Keil 编译（目标零 warning）与新增 PC 测试编译运行，需在 Keil / MinGW 执行。

PC 测试：`tests/pc/test_foc_isr_refactor.c`，在 STM32F103C8T6_BASE 目录：

```
gcc -std=c99 -I Core/Inc ^
  Core/Src/foc_precompute.c Core/Src/foc_current_model.c Core/Src/foc_trig_table.c ^
  Core/Src/foc_transform.c Core/Src/foc_angle_math.c Core/Src/foc_pi.c ^
  Core/Src/foc_svpwm.c Core/Src/foc_controller.c ^
  tests/pc/test_foc_isr_refactor.c -lm -o tests/pc/trefactor
tests\pc\trefactor.exe
```
覆盖：precompute 数值与非法拒绝、电流模型等价/符号/缺预算拒绝、PI 等价与限幅、
SVPWM 等价（逐相 ≤1 tick）与范围、查表四象限/整点/周期、controller 集成冒烟。
注意：不编译 `foc_params.c`（它 include board_config.h→HAL），测试内手工构造参数集。

**上板验收（用户侧）**：
1. S1b 完整打出 `sp:rt ok / sp:drv ok / POWER enabled / sp:done / CMD accepted`，不再停 `[D`；
2. ISR 入口/出口各翻转一个空闲 GPIO，示波器测 CURRENT 模式最坏拍脉宽 ≤30µs、占用 ≤30%；
3. S1a/S2/S3/S4 行为与 07 的 FR1–FR6 结果不回退。

## 9. 风险与回滚

- 风险点：预算量若未来在线可变却忘记重算，会用到旧系数——已在 precompute 与
  controller/model 门禁加 `>0` 校验，缺失即 NOT_READY，不会静默错算；
- 浮点结合顺序变化的误差已由数值旁证限定在机器精度/1 LSB 以内；
- 回滚：本次为等价重构，逐文件还原 git 即可恢复 0.4 现算版本（新增文件删除、
  .uvprojx 移除两个 File 节点、栈/优先级还原）。
