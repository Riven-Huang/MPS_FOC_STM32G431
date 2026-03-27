# FOC Current Loop Spike Follow-Up

审查日期: 2026-03-25

## 1. 现象对应的代码根因

当前电流环本体已经能跟踪 `iq_ref`，但仍然存在“偶发尖峰”风险，主要来自两类输入突变：

- 外部直接改 `g_motor.iq_ref` / `g_motor.id_ref` 时，命令会原样进入 10 kHz 电流 PI。
- `speed_loop_enable = 0` 的纯电流环测试下，`iq_ref` 本质上就是转矩指令，给自由轴阶跃电流时，电机会立刻产生加速趋势。

原代码里，电流 PI 的参考值没有单独的“应用层缓冲”：

- 原始命令量和真正送进 PI 的量是同一个变量。
- 只要上层单拍改大一点，`uq_ref` 这一拍就可能直接冲到比较高。
- 如果这时编码器角度、反电动势、机械自由转动同时叠加，波形上就容易看到尖峰或短时乱跳。

## 2. 本次新增的抑制措施

本次没有去给电流反馈硬加低通，因为那会直接吃掉带宽，而是改成了“命令整形”：

1. 保留原始命令
   - `g_motor.id_ref`
   - `g_motor.iq_ref`

2. 新增内部应用命令
   - `g_id_ref_applied_a`
   - `g_iq_ref_applied_a`

3. 在每个 10 kHz 快环里，应用命令先经过两步处理再进 PI
   - 有限值检查：`NaN/Inf` 直接清零
   - 电流限幅：限制在 `[-g_motor.iq_limit, +g_motor.iq_limit]`
   - 斜坡限速：按 `PROGRAM_CURRENT_REF_RAMP_A_PER_S = 500 A/s` 推进

对应效果：

- 0.5 A 这种小阶跃仍然是很快的。
- 单拍跳变不会直接把 `uq_ref` 顶满。
- 纯电流模式下，用调试器突然把 `iq_ref` 改大时，转矩建立更平滑，不容易出现电流尖峰和机械抽动同时爆出来。

## 3. 执行位置和频率

| 运算 | 频率 | 执行方式 | 执行位置 |
| --- | --- | --- | --- |
| `program_update_applied_current_references()` | 10 kHz | 直接函数调用 | `ADC1` 注入组完成中断 |
| `program_run_current_loop()` | 10 kHz | 直接函数调用 | `ADC1` 注入组完成中断 |
| 电流参考斜坡更新 | 100 us 一次 | 限速器 | `program_run_current_loop()` 内 |
| 原始命令遥测刷新 | 10 kHz 节拍下同步更新 | 结构体写入 | `program_update_debug_telemetry()` |

## 4. 推荐调试抓取量

建议同时观察下面 6 个量：

- `iq_ref_cmd`
- `iq_ref_applied_cmd`
- `iq`
- `uq_ref_cmd`
- `theta_elec`
- `speed_loop_enable`

判读方法：

- 如果 `iq_ref_cmd` 跳了，但 `iq_ref_applied_cmd` 是平滑上升，说明尖峰已经不是“命令硬阶跃”导致的。
- 如果 `iq_ref_applied_cmd` 很平滑，`iq` 仍然有尖峰，就该继续看采样时刻、相电流零漂、角度抖动或者死区补偿。
- 如果 `speed_loop_enable = 0` 且 `iq_ref_applied_cmd > 0`，电机自由转起来本身是符合物理规律的，不代表电流环出错。

## 5. 建议的调试步骤

1. 先在 `speed_loop_enable = 0` 下做小阶跃
   - 例如 `iq_ref_cmd: 0 -> 0.2 -> 0.5 A`
   - 先不要直接上更大的电流

2. 同时看 `iq_ref_cmd` 和 `iq_ref_applied_cmd`
   - 确认应用命令是否按斜坡推进
   - 如果两者完全重合，说明你看的不是这次新增的遥测变量

3. 观察 `iq` 和 `uq_ref_cmd`
   - 正常情况下，`iq` 应该跟随 `iq_ref_applied_cmd`
   - `uq_ref_cmd` 会有响应，但不应该出现单拍特别高的尖刺

4. 一旦看到异常尖峰，立刻对比 `theta_elec`
   - 如果 `theta_elec` 同拍发生跳变，问题更偏角度链路
   - 如果 `theta_elec` 很平滑，但 `iq` 乱跳，问题更偏采样链路或 PI 参数

5. 再看是否机械自由转动
   - 如果自由轴未锁定，`iq_ref_applied_cmd > 0` 时电机转起来是正常的
   - 想纯看静态电流环，建议锁轴，或者先测 `id` 阶跃

## 6. 这次修改的边界

这次修改主要解决“命令瞬时注入导致的尖峰风险”，还没有做下面这些更激进的处理：

- 不对 `id/iq` 反馈做低通，以免压缩电流环带宽。
- 不对相电流做中值滤波，以免把真实过流也滤掉。
- 不改采样触发点和 PWM 死区配置，这些属于下一层硬件时序优化。

如果你复测后仍然看到尖峰，但 `iq_ref_applied_cmd` 已经很平滑，那么下一步应继续排查：

- 三相电流采样窗口是否卡在开关噪声边沿
- `theta_elec` 是否还存在偶发跳点
- 零电流偏置在热态下是否漂移
- `current_kp/current_ki` 是否对当前母线电压和电机电感偏激
