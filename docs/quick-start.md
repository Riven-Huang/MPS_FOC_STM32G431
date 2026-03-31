# 快速开始与三环调试

这份文档默认你已经拿到同款板子，目标是尽快完成：

1. 上电自检
2. 编码器/电流采样确认
3. 第一次稳定出力
4. 电流环调试
5. 速度环调试
6. 位置环调试

如果你只想看项目概览，请先看根目录 [`README.md`](../README.md)。

## 1. 首次上电前检查

建议首次 bring-up 满足以下前提：

- 电机空载
- 电源限流
- ST-Link 在线
- 调试器可直接修改全局变量
- 如果要看串口波形，VOFA 已准备好并选择 JustFloat 协议

### 建议加入 Watch Window 的变量

第一组看硬件链路是否正常：

- `g_program_telemetry.current_offset_ready`
- `g_program_telemetry.ia`
- `g_program_telemetry.ib`
- `g_program_telemetry.ic_meas`
- `g_program_telemetry.i_abc_sum`
- `g_program_telemetry.vbus`
- `g_program_telemetry.ma600a_angle_rad`
- `g_program_telemetry.driver_fault_active`

第二组看控制链是否已经接上：

- `g_program_telemetry.theta_elec`
- `g_program_telemetry.id`
- `g_program_telemetry.iq`
- `g_program_telemetry.duty_a`
- `g_program_telemetry.duty_b`
- `g_program_telemetry.duty_c`
- `g_program_telemetry.fast_loop_time_us`
- `g_program_telemetry.fast_loop_overrun`

第三组看给定和环路状态：

- `g_motor.run_request`
- `g_motor.current_loop_enable`
- `g_motor.speed_loop_enable`
- `g_motor.position_loop_enable`
- `g_motor.id_ref`
- `g_motor.iq_ref`
- `g_motor.ud_ref`
- `g_motor.uq_ref`
- `g_motor.speed_ref_mech_rpm`
- `g_motor.position_ref_mech_deg`

## 2. 烧录与静态启动

### 烧录

- 打开 [`program/MDK-ARM/STM32G431_FOC.uvprojx`](../program/MDK-ARM/STM32G431_FOC.uvprojx)
- 编译并下载
- 复位后暂停在主循环或直接在线观察

### 启动后程序会自动完成的事情

无需手动参与，代码会先完成：

- `ADC1` 校准
- `ADC2` 校准
- TIM1 PWM 启动并保持 50% 占空比
- `ADC1 injected` 开始三相电流零偏统计
- `ADC2 DMA` 开始 `VBUS / NTC` 采样
- MA600A 首次角度读取

### 静态检查通过标准

在 `run_request = 0` 时，先确认：

| 检查项                    | 期望现象               |
| ---------------------- | ------------------ |
| `current_offset_ready` | 上电约 100 ms 后变成 `1` |
| `ia / ib / ic_meas`    | 静止时接近 `0 A`        |
| `i_abc_sum`            | 接近 `0`             |
| `vbus`                 | 与实际母线电压同量级         |
| `ma600a_angle_rad`     | 手动转电机时连续变化         |
| `driver_fault_active`  | `0`                |
| `fast_loop_overrun`    | `0`                |

如果这里都不正常，不要进入闭环调试。

## 3. 第一次稳定出力

第一次出力的目标不是追求速度精度，而是确认：

- 编码器方向正确
- 电角度零位对齐正确
- `uq` 正方向确实输出正扭矩
- 功率级使能链路正确

### 推荐配置

先设置：

- `g_motor.current_loop_enable = 0`
- `g_motor.speed_loop_enable = 0`
- `g_motor.position_loop_enable = 0`
- `g_motor.ud_ref = 0.0f`
- `g_motor.uq_ref = 0.5f`
- `g_motor.run_request = 1`

### 程序会自动发生什么

1. 功率级先保持关闭
2. 对齐阶段用固定 `d` 轴电压锁定转子
3. 对齐保持 `8000` 个快环节拍，约 `0.8 s`
4. 在对齐末段平均编码器电角度，计算 `encoder_elec_offset_rad`
5. 对齐完成后，进入基于编码器角度的手动 `ud/uq` 电压模式

### 正常现象

- `control_state` 先进入对齐态，再进入闭环态
- `encoder_align_done = 1`
- `theta_elec` 连续
- 电机开始平稳出力
- 小幅调 `uq_ref`，转矩方向和大小随之变化

### 如果异常

| 现象              | 优先排查                                                      |
| --------------- | --------------------------------------------------------- |
| 一上电就抖动或尖叫       | 电流极性、编码器方向、极对数                                            |
| 对齐后完全不出力        | `N_SLEEP`、`driver_fault_active`、`ma600a_angle_valid`      |
| `id` 很大、`iq` 很小 | 零位对齐错误、编码器方向错误                                            |
| 反向出力            | `MOTOR_ENCODER_DIRECTION_SIGN` 或 `PROGRAM_CURRENT_SIGN_*` |

## 4. 电流环调试

确认电压模式出力方向正确后，再进入电流环。

### 建议初始配置

- `g_motor.current_loop_enable = 1`
- `g_motor.speed_loop_enable = 0`
- `g_motor.position_loop_enable = 0`
- `g_motor.id_ref = 0.0f`
- `g_motor.iq_ref = 0.1f` 到 `0.3f`
- `g_motor.iq_limit = 0.3f` 到 `0.5f`

### 当前电流环结构

- 电流反馈：`id / iq`
- PI 输出：`ud_ref / uq_ref`
- 积分器：`id_integral_v / iq_integral_v`
- 电压上限：`PROGRAM_VOLTAGE_LIMIT_RATIO * vbus`
- 参考斜坡：`PROGRAM_CURRENT_REF_RAMP_A_PER_S`

### 先看哪些量

- `g_program_telemetry.id`
- `g_program_telemetry.iq`
- `g_program_telemetry.id_ref_cmd`
- `g_program_telemetry.iq_ref_cmd`
- `g_program_telemetry.id_ref_applied_cmd`
- `g_program_telemetry.iq_ref_applied_cmd`
- `g_program_telemetry.ud_ref_cmd`
- `g_program_telemetry.uq_ref_cmd`
- `g_program_telemetry.voltage_limit_v`

### 调参顺序

1. 保持 `id_ref = 0`
2. 小步进给 `iq_ref`
3. 先调 `current_kp`，让 `iq` 能快速跟随但不明显振荡
4. 再加 `current_ki`，去掉稳态误差
5. 最后检查 `id` 是否仍能保持在 `0` 附近

### 当前默认参数

- `current_kp = 2.5761`
- `current_ki = 4555.31`

这组值只适用于当前板子、电流采样链和默认快环频率，换电机后不要直接照搬。

## 5. 速度环调试

当前速度环不是固定定时器硬中断，而是跟随测速窗口更新。

### 当前实现特征

- 编码器每次新样本到来都会更新连续机械角
- 每累计 `20` 个快环样本更新一次速度
- 名义速度环节拍约 `500 Hz`
- 速度测量一阶低通默认截止频率 `150 Hz`

### 建议配置

- `g_motor.current_loop_enable = 1`
- `g_motor.speed_loop_enable = 1`
- `g_motor.position_loop_enable = 0`
- `g_motor.iq_limit` 先限制在安全范围
- `g_motor.speed_ref_mech_rpm` 从小值开始

### 优先观察

- `g_program_telemetry.speed_ref_mech_rpm`
- `g_program_telemetry.speed_ref_mech_applied_rpm`
- `g_program_telemetry.speed_meas_raw_mech_rpm`
- `g_program_telemetry.speed_meas_mech_rpm`
- `g_program_telemetry.speed_loop_dt_s`
- `g_program_telemetry.iq_ref_cmd`

### 调参建议

1. 先用较小 `speed_ref_mech_rpm` 验证方向
2. 先调 `speed_kp`
3. 再逐步增加 `speed_ki`
4. `iq_limit` 是速度环最重要的安全夹紧量，不要一开始就放大

### 当前默认参数

- `speed_kp = 0.0015`
- `speed_ki = 0.015`

## 6. 位置环调试

位置环按 `200 Hz` 运行，并且工作在输出轴坐标系。

### 当前实现特征

- `position_ref_mech_deg` 是外部写入的输出轴目标位置
- `position_ref_mech_rad` 是程序内部换算后的弧度镜像
- `position_meas_mech_rad` 是输出轴实际位置
- 位置环输出的是“输出轴速度命令”
- 输出轴速度命令再乘以 `MOTOR_GEAR_RATIO`，送入转子侧速度环

### 建议配置

- `g_motor.current_loop_enable = 1`
- `g_motor.speed_loop_enable = 1`
- `g_motor.position_loop_enable = 1`
- `g_motor.position_speed_limit_mech_rad_s` 先设置小一些
- `g_motor.position_ref_mech_deg` 小步进变化

### 调参顺序

1. 先保证速度环足够稳定
2. `position_ki` 先保持 `0`
3. 逐步增加 `position_kp`
4. 只有在需要消除静差时，再小心增加 `position_ki`

### 当前默认参数

- `position_kp = 3.0`
- `position_ki = 0.0`

## 7. 运行中切换环路时要知道的事情

当前工程在模式切换时，会自动清理相关积分器和参考斜坡：

- 切换位置环时，会 reset 位置环、速度环和速度参考斜坡
- 切换电流环时，会 reset 速度环和电流环
- `run_request = 0` 时，会关闭功率级并清零各环状态

所以：

- 不要把切换前的波形直接当成切换后的调参依据
- 切环后先重新观察积分量是否被正确拉回

## 8. 移植时必须确认的参数

### 电机参数

修改 [`program/App/motor_params.h`](../program/App/motor_params.h)：

- `MOTOR_POLE_PAIRS`
- `MOTOR_GEAR_RATIO`
- `MOTOR_ENCODER_DIRECTION_SIGN`
- `MOTOR_ENCODER_ON_OUTPUT_SHAFT`

### 采样链参数

修改 [`program/App/program.c`](../program/App/program.c)：

- `PROGRAM_SHUNT_RESISTOR_OHM`
- `PROGRAM_CURRENT_SENSE_GAIN`
- `PROGRAM_VBUS_R_UP_OHM`
- `PROGRAM_VBUS_R_DOWN_OHM`
- `PROGRAM_CURRENT_SIGN_IA`
- `PROGRAM_CURRENT_SIGN_IB`
- `PROGRAM_CURRENT_SIGN_IC`

### 外设配置

修改 [`program/STM32G431_FOC.ioc`](../program/STM32G431_FOC.ioc)：

- PWM 管脚
- ADC 通道
- SPI 模式与速度
- 串口引脚与波特率

## 9. 常见故障定位

### `current_offset_ready` 长时间不置位

优先检查：

- TIM1 是否真的启动
- `ADC1 injected` 是否被 `TIM1_TRGO2` 触发
- ADC1 中断是否正常进入

### `ma600a_angle_valid = 0`

优先检查：

- SPI1 模式是否仍为 `CPOL=HIGH, CPHA=2EDGE`
- `ENC_CS` 是否正确拉低拉高
- 编码器供电是否稳定

### `id` 明显偏大

优先检查：

- 编码器方向
- 电角度零位偏移
- `MOTOR_POLE_PAIRS`
- 电流零偏是否准确

### `fast_loop_overrun = 1`

优先检查：

- 串口/调试输出是否过多
- 是否在快环里加入了额外阻塞逻辑
- SPI 角度读取是否需要改成更轻量的方式

## 10. 建议的调试节奏

如果你的目标是“当天先转起来”，推荐最短路径：

1. 只看静态零偏和编码器角度
2. 手动电压模式确认扭矩方向
3. 电流环小电流闭合
4. 速度环小转速闭合
5. 最后再上位置环

不要一边改硬件参数，一边同时调三环。先把一个环关死，只让一个环说了算，效率最高。
