# FOC 代码逻辑图：原生矢量版讲解

这版图是 PPT 原生矢量对象，不是截图。放大、改字、改颜色都不会糊。

讲解顺序：

1. `main.c` 初始化 CubeMX 外设。
2. `program_init()` 做驱动休眠、ADC 校准、`g_motor/g_foc/g_program_telemetry` 初始化。
3. 启动两条采样链：`ADC2 DMA + TIM6` 是慢变量，`TIM1 PWM + ADC1 Injected` 是 10kHz 控制快环。
4. `while(1)` 里只跑 `program_task()`，它不是控制快环，只负责遥测和慢任务。
5. 10kHz 快环在 `program_adc_injected_conv_cplt_callback()` 中完成三相电流读取、偏置校准、MA600A 角度调度、电流变换。
6. `program_run_speed_current_control()` 是核心状态机，决定停机、故障、编码器对齐和闭环。
7. 外环生成 `iq_ref / uq_ref`，内环生成 `ud / uq`。
8. `foc_core.c` 做反 Park 和 SVPWM，最后 `program_apply_svpwm_to_tim1()` 写入 `TIM1 CCR1/2/3`。
