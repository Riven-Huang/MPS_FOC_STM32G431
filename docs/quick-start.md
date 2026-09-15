# 上电调试与三环整定

这份文档给出从烧录、静态自检、功率级发波验证到电流环、速度环、位置环闭环整定的操作顺序，以及每一步的判据变量和异常处理方向。项目概览见根目录 [`README.md`](../README.md)。

当前故障清除实现的限制、三环小步进测试及 5 路 VOFA 通道表见 [`program 故障定位与调试`](program-debug.md)。首次位置测试采用其中较低的速度上限，待稳定后再提高到本文所列默认值。

更换电机、减速器或编码器安装方式时，先按 [`适配新电机标准流程.docx`](./适配新电机标准流程.docx) 完成参数辨识与配置，再回到本文执行闭环调试。

## 1. 控制接口与调试方法

全部交互只通过两个全局结构体（定义在 [`program/App/motor.h`](../program/App/motor.h)）：

- 写 `g_cmd` — 模式 `mode`、总开关 `run`、清故障 `clear_fault`、各模式目标值与调参量
- 读 `g_fb` — 状态 `state`、故障 `fault_bits` 及全部测量量

操作顺序固定为**先设 `mode` 和目标值，最后置 `g_cmd.run = 1`**。

运行模式：

- `MOTOR_MODE_DISABLED` — 功率级关闭，安全态
- `MOTOR_MODE_PWM_TEST` — 固定占空比发波
- `MOTOR_MODE_VOLTAGE` — d/q 轴电压直接给定，角度来源由 `volt_open_angle_en` 决定
- `MOTOR_MODE_CURRENT` — 电流环，d/q 轴电流直接给定
- `MOTOR_MODE_SPEED` — 速度环，给定转子侧转速
- `MOTOR_MODE_POSITION` — 位置环，给定输出轴角度

后三种模式需要编码器并执行零位对齐。

## 2. 上电前检查

首次 bring-up 应满足以下条件：

- 电机空载，电源带限流
- 调试器在线（CMSIS-DAP），可直接修改全局变量
- 需要观察波形时，VOFA 已选择 JustFloat 协议

建议加入 Watch Window 的变量分三组：

- 硬件链路 — `g_fb.offset_ready`、`g_fb.ia_a/ib_a/ic_a`、`g_fb.i_sum_a`、`g_fb.vbus_v`、`g_fb.enc_deg`、`g_fb.drv_fault`、`g_fb.fault_bits`
- 控制链 — `g_fb.state`、`g_fb.mode_active`、`g_fb.align_done`、`g_fb.align_ofs_rad`、`g_fb.theta_elec_rad`、`g_fb.id_a`、`g_fb.iq_a`、`g_fb.duty_a/b/c`
- 给定与实时性 — `g_cmd.mode`、`g_cmd.run`、`g_cmd.id_a`、`g_cmd.iq_a`、`g_cmd.ud_v`、`g_cmd.uq_v`、`g_cmd.spd_rpm`、`g_cmd.pos_deg`、`g_fb.id_ref_a`、`g_fb.iq_ref_a`、`g_fb.spd_ref_rpm`、`g_fb.loop_us`、`g_fb.overrun_cnt`

VOFA 遥测共 5 路、约 `333 Hz`（3 ms），依次为两轴实际电流、输出轴转速、输出轴位置、编码器机械角；通道定义见 `program-debug.md`。

## 3. 烧录与静态自检

打开 [`program/MDK-ARM/STM32G431_FOC.uvprojx`](../program/MDK-ARM/STM32G431_FOC.uvprojx)，编译下载后复位，暂停在主循环或在线观察。

程序上电后自动完成 `ADC1`/`ADC2` 校准、启动 `TIM1` 并保持 50% 占空比、累计三相电流零偏（1024 样本，约 100 ms）、启动 `ADC2 + DMA` 采集母线电压与 NTC、读取一次 MA600A 角度。

保持 `g_cmd.run = 0`（上电默认值），确认：

- `g_fb.offset_ready` 上电约 100 ms 后置 `1`
- `g_fb.ia_a` / `ib_a` / `ic_a` 静止时接近 `0 A`，`g_fb.i_sum_a` 接近 `0`
- `g_fb.vbus_v` 与实际母线电压同量级
- `g_fb.enc_deg` 手动转动电机时连续变化
- `g_fb.drv_fault` 与 `g_fb.fault_bits` 为 `0`
- `g_fb.loop_us` 稳定小于 `100 us`，`overrun_cnt` 不增长

只上逻辑电、不上母线电时，欠压故障不会在待机状态下锁存；一旦 `run = 1` 进入非待机状态，母线欠压会锁存 `FAULT_BIT_UNDERVOLT`。上母线电后写 `g_cmd.clear_fault = 1` 清除故障，该操作会同时把 `run` 清 0，需要重新置 `run` 启动。静态检查不通过时不要进入闭环调试。

## 4. 功率级发波链路验证

验证 `TIM1` 到 `MP6539B` 再到三相输出的通路，测试时断开电机：

```c
g_cmd.mode = MOTOR_MODE_PWM_TEST;
g_cmd.duty_a = 0.30f;
g_cmd.duty_b = 0.40f;
g_cmd.duty_c = 0.60f;   /* 三相不相等，便于示波器辨认 */
g_cmd.run = 1;
```

用示波器确认：

- `PA8/PA9/PA10`（上桥）与 `PC13/PB0/PB1`（下桥互补）均有约 `20 kHz` PWM，占空比分别为 30/40/60%
- `MP6539B` 输出侧三相半桥中点按相同占空比开关
- `g_fb.duty_a/b/c` 等于设定值

该模式旁路 FOC 输出，直接下发占空比，但 `N_FAULT` 与相过流保护仍然生效。测试结束后恢复安全态：

```c
g_cmd.run = 0;
g_cmd.mode = MOTOR_MODE_DISABLED;
```

## 5. 编码器对齐与首次出力

首次出力的目的是确认编码器方向、电角度零位、转矩方向和功率级使能链路，不追求转速精度。

### 5.1 编码器闭环角度

```c
g_cmd.mode = MOTOR_MODE_VOLTAGE;
g_cmd.ud_v = 0.0f;
g_cmd.uq_v = 0.5f;   /* 从小值开始 */
g_cmd.run = 1;
```

启动后状态机依次执行：

1. IDLE → ALIGN：以 `CFG_ALIGN_UD_V`（默认 `1.2 V`）的 d 轴电压锁定转子
2. 保持 8000 个快环节拍，约 0.8 s
3. 在末段 512 拍对编码器电角度做正弦/余弦平均，得到零位偏置 `g_fb.align_ofs_rad`
4. 进入 RUN，按编码器角度执行手动 d/q 电压输出

正常现象为 `g_fb.state` 先为 `ALIGN`（1）、约 0.8 s 后变为 `RUN`（2），`g_fb.align_done` 置 `1`，`g_fb.theta_elec_rad` 连续变化。小幅修改 `g_cmd.uq_v`，转矩方向与大小应随之变化。

### 5.2 开环角度验证

需要在不依赖编码器的条件下先确认转动时：

```c
g_cmd.mode = MOTOR_MODE_VOLTAGE;
g_cmd.volt_open_angle_en = 1;          /* 开环积分角度，跳过对齐 */
g_cmd.open_spd_elec_rad_s = 200.0f;    /* 电角速度 ≈ 136 rpm 转子 */
g_cmd.uq_v = 0.5f;
g_cmd.run = 1;
```

### 5.3 异常定位

- 上电即抖动或尖叫 — 检查电流极性 `CFG_CUR_SIGN_*`、编码器方向 `CFG_ENC_DIR_SIGN`、极对数
- 对齐后不出力 — 检查 `N_SLEEP` 电平、`g_fb.drv_fault`、`g_fb.enc_valid`
- `id_a` 明显偏大、`iq_a` 很小 — 零位对齐错误或编码器方向错误
- 出力方向相反 — `CFG_ENC_DIR_SIGN` 取反，或对调任意两根相线

## 6. 电流环调试

电压模式确认出力方向正确后进入电流环。

```c
g_cmd.mode = MOTOR_MODE_CURRENT;
g_cmd.id_a = 0.0f;
g_cmd.iq_a = 0.1f;        /* 0.1 ~ 0.3 A 起步 */
g_cmd.iq_lim_a = 1.0f;    /* 默认上限 5 A，首次闭环仍从 1 A 开始 */
g_cmd.run = 1;
```

重点观察 `g_fb.id_a`、`g_fb.iq_a`、限幅后的 `g_fb.id_ref_a`、`g_fb.iq_ref_a`，以及 `g_fb.ud_v`、`g_fb.uq_v`、`g_fb.volt_lim_v`。

调参顺序：

1. 保持 `g_cmd.id_a = 0`，小步进增加 `g_cmd.iq_a`
2. 先调 `g_cmd.cur_kp`，使 `iq` 快速跟随且无明显振荡
3. 再增加 `g_cmd.cur_ki` 消除稳态误差
4. 最后确认 `id` 仍保持在 `0` 附近

当前默认 `cur_kp = 1.288`、`cur_ki = 2277.65`，由 `500 Hz` 带宽与等效 R/L 自动计算，见 [`program/App/app_config.h`](../program/App/app_config.h)。这组值只适用于当前板子、采样链和快环频率。电流环带宽超过约 `600 Hz` 前需先在 CubeMX 减小 `TIM1` 的 `RCR` 以提高 PWM 更新率，否则相位裕度不足会产生振铃。

## 7. 速度环调试

速度环由测速窗口更新驱动，名义更新率 `2 kHz`（`CFG_SPD_WINDOW_SAMPLES = 5`），测速一阶低通默认 `300 Hz`，目标带宽 `30 Hz`。

```c
g_cmd.run = 0;              /* 停机状态下完成切换 */
g_cmd.mode = MOTOR_MODE_SPEED;
g_cmd.iq_lim_a = 1.0f;      /* 默认上限 5 A，首次闭环从 1 A 开始 */
g_cmd.spd_rpm = 20.0f;      /* 转子侧 rpm，确认方向后再逐级提高 */
g_cmd.run = 1;
```

重点观察目标 `g_cmd.spd_rpm`、斜坡后的实际给定 `g_fb.spd_ref_rpm`、转子侧测量 `g_fb.spd_rot_rpm`、输出轴测量 `g_fb.spd_out_rpm` 和电流给定 `g_fb.iq_ref_a`。

调参顺序：先用较小 `spd_rpm` 确认方向，再调 `g_cmd.spd_kp`，最后增加 `g_cmd.spd_ki`。`iq_lim_a` 是速度环的主要安全夹紧量，不要一开始就放大。

当前默认 `spd_kp = 0.01346`、`spd_ki = 1.26895`，按 `30 Hz` 带宽、`ζ = 1`、`Kt ≈ 0.60`、`J ≈ 3e-4` 整定。出现振荡时优先减小 `CFG_MOTOR_INERTIA_KGM2` 或降低 `CFG_SPD_BW_HZ_DEFAULT`，响应过软时反向调整。

## 8. 位置环调试

位置环运行在 `200 Hz`，工作在输出轴坐标系，`g_cmd.pos_deg` 为输出轴角度（`0~360`），目标带宽 `5 Hz`。

```c
g_cmd.mode = MOTOR_MODE_POSITION;
g_cmd.pos_spd_lim_rad_s = 8.0f;   /* 输出轴速度上限，默认 8.0 ≈ 76 rpm */
g_cmd.run = 1;
/* 进入位置模式时 g_cmd.pos_deg 会被自动写成当前位置，先咬住不动，
 * 之后小步进修改 pos_deg 观察跟随 */
```

当前位置环输出速度上限对应 48 V 空载转速上限（约 95 rpm）以下。急减速时能量回灌母线，若出现 `FAULT_BIT_OVERVOLT` 应调小该限幅。

调参顺序：先保证速度环足够稳定，`pos_ki` 保持 `0`，逐步增加 `g_cmd.pos_kp`；仅在需要消除静差时才增加 `pos_ki`，`pos_kd` 提供测速阻尼。

当前默认 `pos_kp = 31.42`、`pos_ki = 0`、`pos_kd = 0.3`。`pos_hold_en = 1` 时启用保持/爬行逻辑抑制减速箱静摩擦引起的低频抖动；若位置环出现慢速大幅度抖动，运行时将该位置 `0` 可旁路该逻辑做对照。

## 9. 模式切换与故障恢复

- `g_cmd.run = 0` — 立即回到 IDLE，关闭功率级，PWM 回到 50%，环路状态清零
- 运行中修改 `g_cmd.mode` — 环路状态自动复位后继续运行，闭环类模式之间切换保留对齐结果
- 每次 `run = 1` 启动闭环类模式都会重新执行一次零位对齐
- 故障时进入 FAULT 锁存并关闭功率级；故障源消失后写 `g_cmd.clear_fault = 1` 回到 IDLE，该操作同时把 `run` 清 0，防止排除故障后电机无预兆自启

切换模式或清除故障后应重新观察积分量是否被正确拉回，不要把切换前的波形作为切换后的调参依据。

## 10. 参数移植清单

换电机或换板子时只需修改 [`program/App/app_config.h`](../program/App/app_config.h)：

- 电机与机构 — `CFG_MOTOR_POLE_PAIRS`、`CFG_MOTOR_GEAR_RATIO`、`CFG_ENC_DIR_SIGN`、`CFG_ENC_ON_OUTPUT_SHAFT`
- 采样链 — `CFG_SHUNT_OHM`、`CFG_CUR_GAIN`、`CFG_VBUS_R_UP_OHM`、`CFG_VBUS_R_DOWN_OHM`、`CFG_CUR_SIGN_IA/IB/IC`
- 控制默认值 — `CFG_IQ_LIM_A_DEFAULT`、`CFG_CUR_EQ_R_OHM`、`CFG_CUR_EQ_L_H`、`CFG_CUR_BW_HZ_DEFAULT`、`CFG_ALIGN_UD_V`、`CFG_ALIGN_HOLD_TICKS`
- 保护阈值 — `CFG_PROT_PHASE_OC_A`、`CFG_PROT_VBUS_OV_V`、`CFG_PROT_VBUS_UV_V`

外设相关修改在 [`program/STM32G431_FOC.ioc`](../program/STM32G431_FOC.ioc)：PWM 引脚、ADC 通道、SPI 模式与速率、串口引脚与波特率、FDCAN 引脚占用。

NTC 换算假设上拉电阻到 3.3 V、NTC 到地，`CFG_NTC_PULLUP_OHM` 需对照 V1.3 原理图确认后再使用 `g_fb.ntc_c`。

## 11. 故障定位

- `g_fb.offset_ready` 长时间不置位 — 检查 `TIM1` 是否启动、`ADC1` 注入组是否被 `T1_TRGO2` 触发、ADC1 中断是否进入
- `g_fb.enc_valid = 0` — 检查 SPI1 是否为 `CPOL=HIGH`、`CPHA=2EDGE`，`ENC_CS` 时序和编码器供电，以及 `g_fb.enc_err`、`g_fb.enc_reject` 是否持续增长
- `g_fb.id_a` 明显偏大 — 检查编码器方向、零位偏置 `g_fb.align_ofs_rad`、`CFG_MOTOR_POLE_PAIRS` 和电流零偏
- `g_fb.overrun_cnt` 持续增长 — 检查调试输出是否过多、快环内是否加入阻塞逻辑、SPI 读角是否需要进一步减负
- `g_fb.fault_bits` 非零 — 按位解读：`0x01` 驱动器 `N_FAULT`，`0x02` 相过流，`0x04` 母线过压，`0x08` 母线欠压，`0x10` 编码器丢失

## 12. 调试顺序汇总

1. 静态检查电流零偏、母线电压与编码器角度
2. `PWM_TEST` 验证功率级发波，测试时断开电机
3. `VOLTAGE` 开环角度确认转动方向
4. `VOLTAGE` 编码器闭环角度加零位对齐，确认闭环角度可用
5. `CURRENT` 从小电流开始闭合电流环
6. `SPEED` 从小转速开始闭合速度环
7. 最后进入 `POSITION`

调试期间一次只闭合一个环路，并保持硬件参数不变，避免把环路问题与硬件改动混在一起。
