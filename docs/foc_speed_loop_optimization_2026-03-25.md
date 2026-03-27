# FOC Speed Loop Optimization Follow-Up

审查日期: 2026-03-25

## 1. 这次改了什么

这次没有改换相角主链，也没有改电流环 PI 主结构，重点只放在速度观测和速度环入口上。

原来的速度链路是:

- 每个新编码器样本都进一次 `program_update_speed_measurement()`
- 速度估算内部使用位置 PLL
- PLL 输出再进一阶低通
- 速度 PI 由固定 10 分频计数器触发

这个结构在能转起来以后仍然容易出现两个问题:

- 低速时单样本角度增量太小，编码器量化误差会直接污染速度估计
- 即使这一次没有新速度信息，固定分频到了，速度 PI 还是会拿旧速度值算一次

这次改成了更直接的结构:

- 先把机械角累加成连续角
- 每累计 `10` 个编码器样本，再做一次窗口差分测速
- 窗口差分的原始速度再进一阶低通
- 只有当新速度估计真正更新后，速度 PI 才执行一次
- 速度 PI 的 `dt` 不再硬写死 1 ms，而是使用这次测速窗口的实际时间

这更接近你那个 DSP 参考工程的“差分测速 + 低通”思路，但保留了当前 STM32 工程的电流环结构。

## 2. 这次调整的默认参数

位置: `program/App/program.c`

- `PROGRAM_SPEED_OBSERVER_WINDOW_SAMPLES = 10`
- `PROGRAM_SPEED_MEAS_LPF_CUTOFF_HZ = 150.0f`
- `PROGRAM_DEFAULT_SPEED_KP = 0.03f`
- `PROGRAM_DEFAULT_SPEED_KI = 0.30f`

含义:

- 速度估计名义更新率从“每个编码器样本”收敛到“约 1 kHz”
- 低速量化噪声先靠窗口差分基线压一轮，再靠 150 Hz LPF 压一轮
- 默认速度 PI 比之前保守，减少 `iq_ref_cmd` 被测速噪声带着抖

## 3. 核心函数频率和执行位置

| 运算 / 函数 | 名义频率 | 真正执行方式 | 执行上下文 |
| --- | --- | --- | --- |
| `program_adc_injected_conv_cplt_callback()` | 10 kHz | 每次 ADC1 injected 完成直接执行 | ADC 中断 |
| `ma600a_read_angle()` | 10 kHz 发起 | 非阻塞启动一次 SPI 传输 | ADC 中断 |
| `HAL_SPI_TxRxCpltCallback()` -> `ma600a_spi_txrx_cplt()` | 接近 10 kHz | SPI 完成中断回调 | SPI 中断 |
| `program_update_speed_measurement()` | 每次新编码器样本到达时调用 | 直接函数调用 | ADC 中断 |
| 窗口差分测速结果更新 | 名义约 1 kHz | 每累计 10 个编码器样本才更新一次 | `program_update_speed_measurement()` 内 |
| `program_update_speed_loop()` | 每个快环都被调用 | 只有新速度样本 ready 时 PI 才真的执行 | ADC 中断 |
| `program_run_current_loop()` | 10 kHz | 每次快环直接执行 | ADC 中断 |
| `program_task()` | while 持续轮询，实际约 1 kHz | 由 TIM6 1 ms 节拍放行 | `while(1)` |

重点:

- 速度环仍然和电流环共用同一个 ADC ISR 上下文
- 但这次已经把“速度 PI 触发条件”从固定分频改成了“新速度样本触发”
- 因此 `iq_ref_cmd` 不会再拿旧速度值空算

## 4. 这次新增的调试量

位置: `program/App/program.h`

- `ma600a_sample_counter`
- `speed_observer_window_samples`
- `speed_meas_raw_mech_rad_s`
- `speed_meas_raw_mech_rpm`
- `speed_error_mech_rad_s`
- `speed_loop_dt_s`

建议重点抓下面这些量:

- `speed_ref_mech_applied_rad_s`
- `speed_meas_raw_mech_rad_s`
- `speed_meas_mech_rad_s`
- `speed_error_mech_rad_s`
- `iq_ref_cmd`
- `iq_ref_applied_cmd`
- `iq`
- `uq_ref_cmd`
- `ma600a_sample_counter`
- `speed_loop_dt_s`
- `fast_loop_time_us`
- `fast_loop_overrun_count`

## 5. 如何 debug

先看测速链是不是稳:

- 如果 `speed_meas_raw_mech_rad_s` 很毛，但 `speed_meas_mech_rad_s` 已经平很多，说明新窗口测速已经起作用
- 如果 `speed_loop_dt_s` 长期稳定在约 `0.001 s`，说明编码器样本节拍基本稳定
- 如果 `speed_loop_dt_s` 经常跳大，先查 SPI 更新掉拍或中断抢占

再看速度 PI 有没有还在抖:

- 如果 `speed_error_mech_rad_s` 很小但 `iq_ref_cmd` 仍来回摆，说明速度 PI 还偏激进
- 如果 `iq_ref_cmd` 抖，但 `iq_ref_applied_cmd` 明显更平，说明当前主要是速度指令抖动，电流参考限速已经在帮你兜底
- 如果 `iq_ref_applied_cmd` 和 `iq` 也一起剧烈抖，说明机械上看到的抖动是真实转矩抖动，不是单纯显示噪声

最后看快环预算:

- `fast_loop_time_us` 应稳定小于 `100 us`
- `fast_loop_overrun_count` 不能持续增长

## 6. 如果你上板后还抖，下一步优先怎么调

先按这个顺序调，不要同时乱动:

1. 先看 `speed_meas_raw_mech_rad_s` 和 `speed_meas_mech_rad_s`
2. 如果原始测速还是很毛，把 `PROGRAM_SPEED_OBSERVER_WINDOW_SAMPLES` 从 `10` 提到 `12` 或 `16`
3. 如果过滤后速度还毛，可以把 `PROGRAM_SPEED_MEAS_LPF_CUTOFF_HZ` 从 `150` 往下收
4. 如果速度已经稳了但闭环还是抽，把 `speed_kp` 再降一点
5. 最后再决定要不要继续降 `speed_ki`

不建议一开始就先砍电流环，当前问题的主矛盾不在电流环带宽，而在速度观测输入和速度环触发方式。

## 7. 这次修改的边界

这次仍然没有做下面这些更激进的动作:

- 没把速度环挪到后台慢任务
- 没对 `iq_ref_cmd` 额外再套一层速度环专用 LPF
- 没做速度误差死区
- 没改 VOFA 默认发送通道定义

原因是先把观测链路和速度 PI 触发链路理顺，再决定下一刀切哪里，风险更低。
