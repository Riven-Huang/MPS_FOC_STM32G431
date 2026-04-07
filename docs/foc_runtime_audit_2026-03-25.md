# FOC Runtime Audit

审查日期：2026-03-25

> 注：本文是 `2026-03-25` 的运行时审查历史快照，保留了当时版本的实现名称与排查路径。当前主分支已不再使用文中提到的部分 PLL 变量和函数名，现行实现请以 [`program/App/program.c`](../program/App/program.c) 为准。

审查范围：

- `program/App/program.c`
- `program/App/foc_core.c`
- `program/App/ma600a.c`
- `program/App/cli_uart.c`
- `program/App/motor_state.c`
- `program/App/drv_pid.c`
- `program/App/filter.c`
- `program/Core/Src/main.c`
- `program/Core/Src/adc.c`
- `program/Core/Src/tim.c`
- `program/Core/Src/spi.c`
- `program/Core/Src/usart.c`
- `program/Core/Src/stm32g4xx_it.c`

## 1. 结论摘要

- 已确认一个与“系统运行一段时间后单周期执行时间越来越长，随后速度抖动加重并最终乱转”高度一致的主因。
- 主因是编码器 PLL 的机械角 `g_encoder_pll_pos_mech_rad` 持续累加，而控制角生成路径又要对这个连续角做包角。原来的包角实现是 `while (angle >= 2pi) angle -= 2pi;` 这种与运行时间成正比的循环。电机转得越久，循环次数越多，ADC 快环耗时就越长。
- 这个问题会进一步拖慢 SPI1 编码器完成中断，因为 `ADC1_2_IRQn` 优先级是 0，而 `SPI1_IRQn` 优先级是 2。快环一旦开始超时，SPI 完成回调更难及时执行，`ma600a_read_angle()` 更容易遇到 `transfer_busy`，编码器样本间隔变大，速度估计抖动加重。
- 当前代码在 ADC 中断和 TIM6 中断里，没有发现 `HAL_Delay()`、阻塞式 `printf()`、阻塞式 SPI/ADC 轮询这种“硬阻塞”。
- 但 ADC 快环里有不少高代价操作：`sinf()`、`cosf()`、`sqrtf()`、两次 Park 相关计算、SVPWM、GPIO 写、SPI 启动、遥测结构体更新、DWT 计时。这些不是阻塞，但会实打实占用 100 us 预算。
- 当前 active path 里没有发现明显的分母为 0 风险。`dt` 不是测量出来的，而是编译期常量；速度观测器里 `sample_dt_s <= 0` 也已经做了保护。
- 当前 active path 的 float PI 积分项有硬限幅；未接入的 Q15 PI 也有硬限幅。原代码的问题不是“完全没限幅”，而是“缺少有限值保护”，一旦上游出现 NaN/Inf，会直接污染积分项并扩散。本次已补保护。

## 2. 本次确认并已处理的关键问题

### 2.1 已确认主因：包角耗时随运行时间增长

修复前的实际风险链路是：

`program_adc_injected_conv_cplt_callback()`  
-> `program_run_speed_current_control()`  
-> `program_get_control_elec_angle_rad()`  
-> `program_get_encoder_pll_aligned_elec_angle_rad()`  
-> `motor_params_rotor_mech_to_elec_rad(g_encoder_pll_pos_mech_rad)`  
-> `motor_params_wrap_angle_rad()`

问题本质：

- `g_encoder_pll_pos_mech_rad` 是连续累加量。
- 连续累加量越大，`while(angle >= 2pi)` 的减法次数越多。
- 这个包角函数在 10 kHz ADC 快环里被频繁调用。
- 所以单周期执行时间会随运行时间持续增长。

这正好解释了你观察到的现象：

- 刚上电能跑。
- 跑一段时间后单周期耗时越来越长。
- SPI 编码器更新开始掉拍。
- 速度观测抖动变大。
- 闭环角度和电流环一起失稳，最终乱转。

本次已处理：

- `program/App/program.c:93` 新增常数时间包角 `program_wrap_angle_0_2pi()`
- `program/App/program.c:270` 新增 `program_renormalize_encoder_observer()`，周期性把 PLL 连续角归一化，避免 float 连续增大
- `program/App/foc_core.c:32` 把 `foc_core_wrap_angle()` 改成常数时间
- `program/App/motor_params.h:12` 把 `motor_params_wrap_angle_rad()` 改成常数时间

### 2.2 已确认次级问题：连续机械角长期不归一化会带来 float 精度恶化

即使把 `while` 改掉，`g_encoder_continuous_mech_rad` 和 `g_encoder_pll_pos_mech_rad` 如果一直增长，float 有效分辨率仍会越来越差。

后果：

- 相位误差 `phase_error_rad = continuous - pll_pos` 的有效精度下降
- PLL 速度估计变抖
- 速度环输入抖动加重

本次已在 `program/App/program.c:270` 加入归一化，避免连续角无限增大。

### 2.3 已确认 NaN 风险点：原 float PI 有硬限幅，但没有有限值保护

当前 float PI 在 `program/App/program.c:217`，积分项限幅在：

- `program/App/program.c:249`
- `program/App/program.c:250`

未接入的 Q15 PI 在 `program/App/drv_pid.c:105`，积分项限幅在：

- `program/App/drv_pid.c:123`
- `program/App/drv_pid.c:128`

结论：

- 积分项并不是“完全没硬限幅”。
- 但原来只做大小比较，不做 `isfinite()` 检查。
- 一旦某次上游输入已经是 NaN，比较表达式会全部失效，积分器会被直接写成 NaN。

本次已在 `program/App/program.c:231` 到 `program/App/program.c:245` 增加有限值保护。

## 3. 核心函数执行频率、执行方式、执行上下文

前提时钟：

- `.ioc` 指定 HSE 为 8 MHz：`program/STM32G431_FOC.ioc:271`
- HAL 配置里的 `HSE_VALUE` 也是 8 MHz：`program/Core/Inc/stm32g4xx_hal_conf.h:118`
- 主时钟 PLL 配置在 `program/Core/Src/main.c:137` 到 `program/Core/Src/main.c:160`
- 结合 `PLLM=2, PLLN=85, PLLR=2`，系统时钟是 170 MHz

| 函数/链路                                                    | 频率                                | 执行方式                    | 执行上下文               | 说明                                                                                                                                      |
| -------------------------------------------------------- | --------------------------------- | ----------------------- | ------------------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| `main()` -> `while(1)` -> `program_task()`               | while 持续轮询，实际 1 kHz 生效            | 轮询                      | 主循环                 | 入口在 `program/Core/Src/main.c:111` 到 `program/Core/Src/main.c:116`，真正 1 ms 节拍在 `program/App/program.c:979` 到 `program/App/program.c:992` |
| `TIM6` 基准                                                | 1 kHz                             | 定时器更新中断                 | `TIM6_DAC_IRQn`     | `PSC=169, ARR=999`，见 `program/Core/Src/tim.c:128` 到 `program/Core/Src/tim.c:139`                                                        |
| `program_tim_period_elapsed_callback()`                  | 1 kHz                             | 中断回调                    | TIM6 中断             | 只做 `g_tim6_tick_ms++`，见 `program/App/program.c:1000` 到 `program/App/program.c:1005`                                                     |
| `ADC2` regular + DMA                                     | 1 kHz                             | TIM6 TRGO 触发 + DMA 环形搬运 | 外设硬件 + while 后台读取   | `ADC2` 由 `TIM6_TRGO` 触发，见 `program/Core/Src/adc.c:133` 到 `program/Core/Src/adc.c:147`；后台消费在 `program/App/program.c:990`                 |
| `program_update_measurements()`                          | 1 kHz                             | 直接函数调用                  | `program_task()`    | 读取 `VBUS/NTC` DMA 结果，定义在 `program/App/program.c:842`，调用在 `program/App/program.c:990`                                                    |
| `program_send_wave_if_needed()`                          | 50 Hz                             | 直接函数调用 + UART DMA       | `program_task()`    | 20 ms 节流，见 `program/App/program.c:898`                                                                                                  |
| `TIM1` PWM                                               | 20 kHz 载波                         | 硬件 PWM                  | TIM1                | `CenterAligned1, ARR=4249`，见 `program/Core/Src/tim.c:46` 到 `program/Core/Src/tim.c:68`                                                  |
| `TIM1_TRGO2` -> `ADC1` 注入组                               | 10 kHz                            | 硬件触发                    | TIM1 -> ADC1        | `TRGO2=UPDATE` 且 `RepetitionCounter=3`，对应 10 kHz 电流采样节拍                                                                                 |
| `program_adc_injected_conv_cplt_callback()`              | 10 kHz                            | HAL injected ADC 完成回调   | `ADC1_2_IRQn`       | 快环总入口，见 `program/App/program.c:1022` 到 `program/App/program.c:1077`                                                                     |
| `program_update_encoder_measurements()`                  | 名义 10 kHz                         | 直接函数调用                  | ADC 快环              | 用的是“上一次 SPI 已完成”的编码器样本，见 `program/App/program.c:863`                                                                                    |
| `ma600a_read_angle()`                                    | 名义 10 kHz 发起                      | 非阻塞 SPI 启动              | ADC 快环里发起，SPI 中断里完成 | 启动在 `program/App/ma600a.c:88` 到 `program/App/ma600a.c:105`                                                                              |
| `HAL_SPI_TxRxCpltCallback()` -> `ma600a_spi_txrx_cplt()` | 名义 10 kHz 完成                      | SPI 中断                  | `SPI1_IRQn`         | 完成回调桥接在 `program/App/program.c:1116`，SPI 优先级见 `program/Core/Src/spi.c:89` 到 `program/Core/Src/spi.c:91`                                 |
| `program_update_speed_measurement()`                     | 名义 10 kHz，取决于新编码器样本是否到达           | 直接函数调用                  | ADC 快环              | 仅当 `sample_counter` 变化时执行，见 `program/App/program.c:496`                                                                                 |
| `program_update_speed_loop()`                            | 每 100 us 被调用一次，但 PI 真正每 1 ms 执行一次 | 分频调度                    | ADC 快环              | `PROGRAM_SPEED_LOOP_DIVIDER=10`，见 `program/App/program.c:30`、`program/App/program.c:560` 到 `program/App/program.c:582`                  |
| `program_update_current_feedback_from_raw()`             | 10 kHz                            | 直接函数调用                  | ADC 快环              | 完成电流换算、Clarke、Park，见 `program/App/program.c:588`                                                                                        |
| `program_run_current_loop()`                             | 10 kHz                            | 直接函数调用                  | ADC 快环              | 电流 PI + 电压限幅 + FOC 输出，见 `program/App/program.c:632`                                                                                     |
| `foc_core_run_voltage_open_loop()`                       | 10 kHz                            | 直接函数调用                  | ADC 快环              | 尽管名字叫 open loop，当前也被闭环电流环复用来完成反 Park + SVPWM，见 `program/App/foc_core.c:239`                                                             |
| `program_apply_svpwm_to_tim1()`                          | 10 kHz                            | 直接寄存器写 CCR              | ADC 快环              | 更新 PWM 比较值，见 `program/App/program.c:382` 到 `program/App/program.c:411`                                                                  |

## 4. 中断路径里是否存在耗时阻塞操作

### 4.1 ADC 快环里没有发现“硬阻塞”

10 kHz 快环入口在 `program/App/program.c:1022` 到 `program/App/program.c:1077`。

我没有在这个路径里发现下面这些问题：

- `HAL_Delay()`
- 阻塞式 UART 打印
- 阻塞式 SPI 读写
- `while(flag == 0)` 这种等待外设完成的轮询

### 4.2 但快环里存在一组明显的高开销操作

这些操作都在快环里，会稳定占预算：

- `HAL_ADCEx_InjectedGetValue()` 三次：`program/App/program.c:983` 到 `program/App/program.c:985`
- 发起一次 SPI 非阻塞传输：`program/App/program.c:1005` 到 `program/App/program.c:1007`，底层调用在 `program/App/ma600a.c:105`
- 电流反馈里的 `sinf()/cosf()`：`program/App/program.c:622` 到 `program/App/program.c:623`
- FOC 输出里的 `sinf()/cosf()`：`program/App/foc_core.c:126` 到 `program/App/foc_core.c:127`
- 电压矢量模长 `sqrtf()`：`program/App/program.c:661`
- 每拍都在写 `N_SLEEP`：`program/App/program.c:459`
- 每拍都更新调试遥测：`program_run_speed_current_control()` 内最终调用 `program_update_debug_telemetry()`
- 每拍都做 DWT 周期计时更新：`program/App/program.c:1018` 到 `program/App/program.c:1019`

结论：

- 当前不是“中断里有阻塞式死等”。
- 当前是“中断里有过多数学运算和调试逻辑，而且之前还有一个会随时间增长的包角循环”。

## 5. 除法、dt、积分项、NaN 审查

### 5.1 当前 active path 的除法点

| 位置                                                          | 表达式                              | 风险结论                                                                                 |
| ----------------------------------------------------------- | -------------------------------- | ------------------------------------------------------------------------------------ |
| `program/App/program.c:477`                                 | `sense_voltage / (gain * shunt)` | 当前常量 `20 * 0.01`，分母固定非 0                                                             |
| `program/App/program.c:493`                                 | `... / PROGRAM_VBUS_R_DOWN_OHM`  | 当前常量 `10000`，非 0                                                                     |
| `program/App/program.c:539`                                 | `mech_delta_rad / sample_dt_s`   | 已有 `sample_dt_s <= 0` 保护，见 `program/App/program.c:533` 到 `program/App/program.c:535` |
| `program/App/program.c:661` 到 `program/App/program.c:664`   | `v_limit / v_mag`                | 已有 `v_mag > 0` 保护                                                                    |
| `program/App/foc_core.c:195` 到 `program/App/foc_core.c:204` | `v_alpha/vbus` 等                 | 已有 `vbus < 1` 保护，低电压时直接输出中心占空比                                                       |
| `program/App/program.c:305` 到 `program/App/program.c:328`   | `1000000 / SystemCoreClock`      | 已有 `SystemCoreClock == 0` 保护                                                         |

### 5.2 `dt = 0` 风险

结论：

- 当前快环 `dt` 是编译期常量 `PROGRAM_FAST_LOOP_DT_S = 1 / 10000`，见 `program/App/program.c:22` 到 `program/App/program.c:24`
- 当前速度环 `dt` 是编译期常量 `PROGRAM_SPEED_LOOP_DT_S = 10 * 100 us = 1 ms`，见 `program/App/program.c:30` 到 `program/App/program.c:31`
- 也就是说，当前 PI 本身不存在“通过测时间得到 dt，结果某次 dt=0”的问题
- 真正动态的 `dt` 只出现在编码器速度观测器里，且已有 `sample_dt_s <= 0` 返回保护

### 5.3 PID/PI 积分项是否缺少硬限幅

结论：

- active path 的 float PI：有硬限幅，不缺
- 未接入的 Q15 PI：也有硬限幅，不缺

具体位置：

- float PI：`program/App/program.c:249` 到 `program/App/program.c:250`
- Q15 PI：`program/App/drv_pid.c:126` 到 `program/App/drv_pid.c:128`

真实问题是：

- 原来的 float PI 没有 `isfinite()` 防护
- 一旦上游已经出现 NaN，积分项会被直接写坏
- 本次已在 `program/App/program.c:231` 到 `program/App/program.c:245` 加入保护

## 6. 速度环是否和电流环解耦

结论：

- 逻辑上“解耦了频率”
- 但“没有解耦执行上下文”

具体表现：

- 电流环是 10 kHz，入口在 ADC 注入组完成中断
- 速度环通过 `PROGRAM_SPEED_LOOP_DIVIDER = 10` 做了 10 分频，所以 PI 真正每 1 ms 执行一次，见 `program/App/program.c:30`、`program/App/program.c:571` 到 `program/App/program.c:581`
- 但是速度环仍然放在同一个 ADC 快环 ISR 里

这意味着：

- 速度环不会每 100 us 都做 PI
- 但它仍和电流环共用同一个 100 us 中断预算
- 只要快环过重，速度环也会被连带抖动

所以这里是“频率解耦了，时间隔离没有做”。

## 7. 其他潜在问题

### 7.1 SPI 编码器采样并不是完全独立时基

编码器读取方式是：

- 在 ADC 快环里发起 `HAL_SPI_TransmitReceive_IT()`：`program/App/program.c:1005` 到 `program/App/program.c:1007`
- 在 `SPI1_IRQn` 里完成回调：`program/App/program.c:1116`

注意优先级：

- `ADC1_2_IRQn` 优先级 0：`program/Core/Src/adc.c:220` 到 `program/Core/Src/adc.c:222`
- `SPI1_IRQn` 优先级 2：`program/Core/Src/spi.c:89` 到 `program/Core/Src/spi.c:91`

因此：

- SPI 完成中断不能抢占 ADC 快环
- 快环一旦拖长，SPI 完成回调就会延后
- 下一个 ADC 周期里 `ma600a_read_angle()` 更容易遇到 `transfer_busy`
- 速度观测 `sample_delta_count` 会变大，速度估计抖动明显

### 7.2 `program_set_power_stage_enable()` 每个快环周期都在写一次 GPIO

位置：

- `program/App/program.c:454` 到 `program/App/program.c:461`

现状：

- 即便状态没变，闭环期间也会每 100 us 写一次 `N_SLEEP`

这不是导致崩溃的主因，但属于不必要的快环负担。

### 7.3 共享 ADC 中断里每次都调用了 `HAL_ADC_IRQHandler(&hadc2)`

位置：

- `program/Core/Src/stm32g4xx_it.c:238` 到 `program/Core/Src/stm32g4xx_it.c:244`

影响：

- 这是常量额外开销，不会随时间增长
- 但在 10 kHz 快环里属于可以省掉的固定成本

### 7.4 `cli_uart_send_text()` 是阻塞式串口发送

位置：

- 定义：`program/App/cli_uart.c:40`
- 阻塞发送：`program/App/cli_uart.c:57`

当前结论：

- 当前 active path 没有调用它
- 现在波形发送走的是 DMA：`program/App/cli_uart.c:66` 到 `program/App/cli_uart.c:91`
- 如果后续有人把 `cli_uart_send_text()` 放进快环或中断，会直接引入毫秒级阻塞

### 7.5 `motor_state_task()` 不是当前实际控制路径

位置：

- `program/App/motor_state.c:55`

说明：

- 当前真实控制闭环不走它
- 当前真实控制入口是 `program_adc_injected_conv_cplt_callback()`
- `motor_state.c` 里的注释“由 `program_task()` 1 kHz 调用”现在已经不是实际运行方式，容易误导后续维护

## 8. 本次代码修改

本次已直接修改以下文件：

- `program/App/program.c`
- `program/App/foc_core.c`
- `program/App/motor_params.h`

修改内容：

- 把包角从 `while` 循环改成常数时间实现
- 对编码器 PLL 连续角做归一化，避免连续增长
- 给 float PI 增加有限值保护，降低 NaN 污染风险

## 9. 推荐的下一步排查顺序

建议按下面顺序验证：

1. 先观察 `fast_loop_time_us` 和 `fast_loop_overrun_count` 是否还会随运行时间持续增长。
2. 再观察 `ma600a.sample_counter` 是否仍会周期性掉拍，或者 `speed_loop_ready` 是否会反复丢失。
3. 再看 `speed_meas_mech_rad_s`、`iq_ref_cmd`、`iq` 三个量是否仍有明显抖动。
4. 如果快环时间已经稳定但速度仍抖，下一步优先减少快环里的调试负担：把部分遥测更新移到 1 kHz 后台，把 `N_SLEEP` 改成只在状态变化时写。
5. 如果还要继续压缩快环时间，下一步应考虑减少 `sinf/cosf/sqrtf` 次数，或者利用 STM32G4 的 CORDIC/FM AC 外设。

## 10. 总结

这份代码里最关键的问题不是“中断里有一个显式阻塞函数”，而是“快环里存在一个与运行时间成正比的包角算法”，它会把 10 kHz 电流环逐渐拖慢，并进一步连带编码器 SPI 更新和速度观测抖动，最终表现成你现在看到的“先抖，再乱转，再宕机”。

这个问题已经在代码里做了针对性修正。当前剩余更像是“快环负担偏重”和“速度环虽然分频了，但仍和电流环共用同一个 ISR 预算”的工程优化问题，而不是同级别的致命逻辑错误。
