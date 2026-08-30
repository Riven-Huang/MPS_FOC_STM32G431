# 代码逻辑图讲解提纲

## 1. 上电初始化
- `main.c`：完成 `MX_GPIO_Init / MX_DMA_Init / MX_ADC1_Init / MX_ADC2_Init / MX_SPI1_Init / MX_TIM1_Init / MX_USART1_UART_Init / MX_TIM6_Init`。
- `program_init()`：驱动先休眠，ADC 校准，初始化 `g_foc / g_motor / g_program_telemetry`，初始化 MA600A 和 VOFA 串口。
- 启动两条采样链：`ADC2 DMA + TIM6` 做慢变量，`TIM1 PWM + ADC1 injected` 做三相电流快采样。

## 2. 10kHz 快环
- `TIM1_TRGO2` 触发 `ADC1 injected` 采样 `IA / IB / IC`。
- `program_adc_injected_conv_cplt_callback()` 读取三相原始值，先做 1024 点零电流偏置。
- 同一节拍调度 `ma600a_read_angle()`，由 SPI 回调更新角度缓存。
- `program_update_current_feedback_from_raw()` 将电流 raw 值扣零点、换算为 A，再执行 `Clarke / Park` 得到 `id / iq`。

## 3. 状态机与闭环
- `program_run_speed_current_control()` 统一处理故障、未校准、未读到角度、停机请求和编码器对齐。
- 对齐完成后进入闭环：位置环可选，速度环生成 `iq_ref`，电流环生成 `ud / uq`。
- `foc_core_run_voltage_open_loop()` 内部执行反 Park 和 SVPWM，得到 `duty_a / duty_b / duty_c`。
- `program_apply_svpwm_to_tim1()` 把占空比写入 `TIM1 CCR1/2/3`，再通过 MP6539B 和 MOSFET 驱动电机。

## 4. 后台慢任务
- `TIM6` 提供 1 ms tick。
- `program_task()` 更新 VBUS/NTC、故障标志和调试遥测。
- `cli_uart_send_vofa()` 通过 USART1 DMA 输出 JustFloat 曲线，用于视频里的速度、位置和调参演示。
