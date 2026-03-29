<div align="center">

# MPS-FOC STM32G431

### 开源无刷电机 FOC 控制工程 | Open-Source PMSM / BLDC FOC Project

<p>
  <img src="https://img.shields.io/badge/Platform-STM32G431-2F74C0?style=flat-square" alt="Platform">
  <img src="https://img.shields.io/badge/IDE-Keil%20MDK%20%7C%20CubeMX-0A8F6A?style=flat-square" alt="IDE">
  <img src="https://img.shields.io/badge/Control-FOC%20%7C%20SVPWM-E67E22?style=flat-square" alt="Control">
  <img src="https://img.shields.io/badge/Sensing-3%20Shunt%20%7C%20MA600A-8E44AD?style=flat-square" alt="Sensing">
  <img src="https://img.shields.io/badge/Status-Bringup%20Ready-7CB518?style=flat-square" alt="Status">
</p>

</div>

> 注意，刚说明针对LCEDA项目下的V1.2版本，其余版本为F280023C下设计，未提供程序源码。V1.2为三环闭环完整的STM32G431CBT6所设计

## 目录

- [1. 项目简介](#sec-1-overview)
- [2. MPS大学计划](#sec-2-mps)
- [3. 验收测试与视频](#sec-3-test)
- [4. 硬件说明](#sec-4-hardware)
- [5. 软件架构](#sec-5-software)
- [6. 控制实现说明](#sec-6-control)
- [7. 快速开始](#sec-7-quickstart)
- [8. 项目目录](#sec-8-tree)
- [9. 已知限制与后续计划](#sec-9-plan)
- [10. 相关文件](#sec-10-files)
- [11. 许可证说明](#sec-11-license)

---

<a id="sec-1-overview"></a>

## 1. 项目简介

### 1.1 项目定位

本项目基于 `STM32G431CBT6`，面向 `PMSM / BLDC` 的三相 FOC 控制。仓库目标是提供一套可直接落到板级 bring-up 的开源实现。

适用场景：

- 板子已经打样完成，需要尽快确认“代码怎么烧录、上电后怎么先转起来”。
- 正在做自研 FOC 控制板，需要参考一套完整的“采样 -> 快环 -> 慢环 -> telemetry -> 调参”工程结构。

### 1.2 当前工程状态

当前仓库已经具备完整主链路，不是只有框架：

- 当前工程实现了所有MPS-FOC比赛中的所有测试项目，该项目为开源广场LCEDA中的**V1.2版本**(<mark>一定注意！！！</mark>)，仅仅V**1.2版本**是STM32G431制作，其余系列为TI的F280025C DSP系列下进行迭代产物。

---

<a id="sec-2-mps"></a>

## 2. MPS大学计划

### 2.1 申请说明

MPS 中国大学计划面向高校教学、科研与竞赛项目，适合本项目所用电机控制相关器件申请。若需要 `MP6539B`、`MA600A`、`MIE1W0505`、`MP4583`、`MPM3632S`、`MP20051` 等样片，可通过对应入口申请。

- 在校大学生：使用 [MPS大学计划](https://www.monolithicpower.cn/cn/support/mps-cn-university.html)
- 非在校个人开发者：使用 [MPSNOW](https://www.monolithicpower.cn/cn/support/mps-now.html)
- 申请备注：`MPS-competition-FOC`

### 2.2 二维码入口

<p align="center">
  <img src="https://image.lceda.cn/oshwhub/pullImage/ba8ee1bfab4e41bba548c110dfd5d1bc.png" alt="MPS University Program QR" width="330">
</p>

---

<a id="sec-3-test"></a>

## 3. 验收测试与视频

本节将仓库内已经公开的验收项、测试视频与现有数据集中整理，便于开源发布时快速核对。当前能够直接在仓库中举证的量化结果包括：

- `48V ±15%` 宽输入电压
- `60W` 下稳定持续带载
- 一系列基础测试项
- `180W`下带载情况与热测量

### 3.1 验收测试总表

| #   | 验收项目       | 参考验收标准                                                                                                                                                                                              | 测试数据                                                       | 数据来源                                                |
| --- | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------- | --------------------------------------------------- |
| 1   | 电源输入范围测试   | `48V ±15%` 宽输入电压                                                                                                                                                                                    | 40V-56V宽输入运行                                               | 视频：Test1 Power Supply  Input Voltage Range Test.mp4 |
| 2   | 启停性能测试     | 空载测试，目测电机启动是否丝滑无振动                                                                                                                                                                                  | 见视频运行结果                                                    | 视频：Test2 StartStop Performance.MP4                  |
| 3   | 速度控制精度测试   | 目标转速100RPM、200RPM, 通过MA600A寄存器读对应速度值                                                                                                                                                                | 测试报告记录：电机减速箱后出轴端 `100 rpm`下扰动`3rpm`、输出轴 `200 rpm` 下，`8rpm` | 视频：Test3 Speed Control Accuracy.MP4                 |
| 4   | 位置控制精度测试   | 目标角度90°、180°, 通过MA600A寄存器读对应角度值                                                                                                                                                                     | 几乎无误差                                                      | 视频：Test4 Position Control Accuracy                  |
| 5   | 负载输出能力测试   | 带载持续运行，可结合实际情况选择验证方式<br/>   1.通过10cm力臂实际带载2kg，验证可支持≥6A相电流<br/>   2. R+L模型模拟: 负载星型连接，每相取3Ω+200μH<br/>HSA=L, LSA=H；HSC=L, LSC=H<br/>HSB=20khz PWM 62% 占空比，LSB       与其互补； <br/>测量该条件下IOUTB或48V母线侧电流大小 | 当该项目等效功率60W等级。本仓库对于60W等级和180W等级分别进行了测试，详情可见下面说明。           | 视频：LoadTEST.MP4                                     |
| 6   | 负载输出能力补充测试 | 10cm力臂，2kg负载，48Vbus 265rpm下连续运行30分钟，测量电机NTC热敏电阻(10K 3950)对应阻值                                                                                                                                       | 同上                                                         | 视频：LoadTEST.MP4                                     |

### 3.2 单项测试数据

3.2.1 电源输入测试

略

3.2.2 启停性能测试

略

3.2.3 速度控制精度测试

<img title="" src="docs/images/placeholder-board-photo.jpg" alt="" width="689">

如上图所示为速度控制时的精度与扰动，图示的数据为电机出轴端之前的机械转速RPM。

3.2.4 位置控制精度

<img title="" src="docs/images/placeholder-board-photo.jpg" alt="" width="680">

如图所示，几乎无误差

3.2.5

测试平台如下图，准备了两个一样的电机，一个电机作为驱动，另一个作为发电机，通过增程器整流输出到电子负载。

![](docs/images/placeholder-board-photo.jpg)

负载输出测试，如图所示。可以参考视频，在180W情况下持续带载运行(赛题要求的功率等级是60W等级)，运行2mins时温度如下图

![](docs/images/placeholder-board-photo.jpg)

运行8mins后，壳温来到了74℃，板上最热点为82.9℃。<img src="docs/images/placeholder-board-photo.jpg" title="" alt="" width="450">

---

<a id="sec-4-hardware"></a>

## 4. 硬件说明

### 4.1 硬件实物图

![](docs/images/poster_done.png)

![](docs/images/hardware_double.png)

### 4.2 硬件核心配置

| 🧩 模块  | 📦 当前实现                     | 📝 说明                     |
| ------ | --------------------------- | ------------------------- |
| 主控 MCU | `STM32G431CBT6`             | 170 MHz，带高级定时器，适合 FOC 快环  |
| PWM 输出 | `TIM1 CH1/2/3 + CH1N/2N/3N` | 中心对齐、互补输出、带死区             |
| 功率级使能  | `N_SLEEP`                   | 控制驱动器休眠 / 使能              |
| 故障输入   | `N_FAULT`                   | 当前用于驱动器故障检测               |
| 电流采样   | `ADC1_IN1 / IN3 / IN4`      | 三相独立采样，注入组同步触发            |
| 母线采样   | `ADC2_IN12`                 | `VBUS`                    |
| 温度采样   | `ADC2_IN14`                 | `NTC`                     |
| 编码器    | `MA600A + SPI1`             | 16-bit 绝对值角度反馈            |
| 调试串口   | `USART1 + DMA TX`           | VOFA / JustFloat 波形输出     |
| 预驱动芯片  | `MP6539B`                   | 3 相 N+N 半桥预驱动，支持 100% 占空比 |
| 升降压芯片  | `MP4583`                    | 宽输入电压 DCDC，支持 48V 总线      |
| 栅极驱动供电 | `MIE1W0505`                 | 隔离 DC-DC，为预驱动提供隔离供电       |
| POL 供电 | `MPM3632S`                  | 3.3V/3A POL，为 MCU 和编码器供电  |
| 备份供电   | `MP20051`                   | LDO，为 MCU 待机部分供电          |

### 4.3 电机与机构默认参数

当前默认参数位于 [`program/App/motor_params.h`](program/App/motor_params.h)：

| ⚙️ 参数   | 当前值  | 说明                                  |
| ------- | ---- | ----------------------------------- |
| 极对数     | `14` | `MOTOR_POLE_PAIRS`                  |
| 减速比     | `8`  | `MOTOR_GEAR_RATIO`                  |
| 编码器方向   | `-1` | `MOTOR_ENCODER_DIRECTION_SIGN`      |
| 编码器安装位置 | 转子侧  | `MOTOR_ENCODER_ON_OUTPUT_SHAFT = 0` |

<a id="sec-5-software"></a>

## 5. 软件架构

### 5.1 软件结构图

<p align="center">
  <img src="docs/figures/software-architecture.svg" alt="Software Architecture" width="980">
</p>

### 5.2 控制流程图

<p align="center">
  <img src="docs/figures/control-flow.svg" alt="Control Flow" width="980">
</p>

### 5.3 代码组织

| 📁 目录 / 文件                  | 🧠 职责                           |
| --------------------------- | ------------------------------- |
| `program/Core/`             | HAL 初始化、IRQ、CubeMX 生成外设配置       |
| `program/App/program.c`     | 当前真实控制主链路、快慢环调度、保护、telemetry    |
| `program/App/foc_core.c`    | Clarke / Park / 反 Park / SVPWM  |
| `program/App/ma600a.c`      | MA600A 绝对值编码器驱动                 |
| `program/App/filter.c`      | 一阶低通滤波器                         |
| `program/App/cli_uart.c`    | VOFA / DMA 串口发送                 |
| `program/App/drv_pid.c`     | 通用 Q15 PI 组件，当前不在 10 kHz 热路径主链里 |
| `program/App/motor_state.c` | 状态对象与保留状态机逻辑                    |
| `docs/`                     | 补充说明、调试文档、图像资源                  |

---

<a id="sec-6-control"></a>

## 6. 控制实现说明

### 6.1 控制流程说明：从 ADC 采样到 PWM 输出

当前工程的实际控制链路如下：

1. `TIM1` 输出中心对齐 PWM，并通过 `TRGO2 = UPDATE` 触发 `ADC1 injected`。
2. `ADC1` 一次完成 `IA / IB / IC` 三路电流采样。
3. 在 `HAL_ADCEx_InjectedConvCpltCallback()` 中读取三相原始值。
4. 启动阶段累计 `1024` 个样本，得到每相零偏。
5. 同步调度 `MA600A` 读角，更新机械角、电角度和测速窗口。
6. 原始电流码值转换为安培值，并做一阶低通。
7. 执行 `Clarke / Park`，得到 `id / iq`。
8. 位置环按 `100 Hz` 分频运行，输出机械速度参考。
9. 速度环按测速窗口更新运行，输出 `iq_ref` 或 `uq_ref`。
10. 电流环按 `10 kHz` 运行，输出 `ud_ref / uq_ref`。
11. 反 `Park` 后进入 `SVPWM`。
12. 将 `duty_a / duty_b / duty_c` 写入 `TIM1->CCR1/2/3`。

### 6.2 中断结构 / 控制周期

快环入口调用链：

```text
ADC1_2_IRQHandler
  -> HAL_ADC_IRQHandler(&hadc1)
    -> HAL_ADCEx_InjectedConvCpltCallback()
      -> program_adc_injected_conv_cplt_callback()
```

慢环入口调用链：

```text
TIM6_DAC_IRQHandler
  -> HAL_TIM_IRQHandler(&htim6)
    -> HAL_TIM_PeriodElapsedCallback()
```

当前工程对应的运行频率：

| ⏱️ 项目    | 当前值         | 触发源 / 说明                                 |
| -------- | ----------- | ---------------------------------------- |
| PWM 载波   | 约 `20 kHz`  | `TIM1 center-aligned + ARR=4249 + RCR=3` |
| 电流环      | `10 kHz`    | `ADC1 injected` 完成回调                     |
| 位置环      | `100 Hz`    | 快环分频                                     |
| 速度测速窗口   | `20` 个快环样本  | `PROGRAM_SPEED_OBSERVER_WINDOW_SAMPLES`  |
| 速度环有效更新率 | 名义 `500 Hz` | 由测速窗口更新驱动                                |
| 慢任务节拍    | `1 kHz`     | `TIM6` 中断 + `TIM6 TRGO -> ADC2`          |

慢环主要处理：

- `ADC2 + DMA` 采集 `VBUS / NTC`
- `program_task()` 在 `while(1)` 中按 `TIM6` 节拍执行后台任务和遥测输出

### 6.3 速度环与测速观测器

（内容待补充）

### 6.4 位置环实现

（内容待补充）

### 6.5 状态机与启动时序

（内容待补充）

### 6.6 Telemetry 变量设计

当前统一观测对象是：

```c
extern volatile program_telemetry_t g_program_telemetry;
```

该对象的职责不是“只拿来画波形”，而是把 bring-up 所需的事实状态统一汇总到一个对象里。这样做有两个直接好处：

- 调试器观察和 VOFA 输出共用同一套字段。
- 采样层、控制层、命令层、状态层不再散落成多个临时全局变量。

| 📊 层级 | 典型字段                                                      | 作用            |
| ----- | --------------------------------------------------------- | ------------- |
| 原始采样层 | `ia_raw` `ib_raw` `ic_raw` `vbus_raw`                     | 查 ADC、偏置和量程问题 |
| 工程量层  | `ia` `ib` `ic_meas` `id` `iq` `theta_elec`                | 查控制量是否正确      |
| 命令层   | `id_ref_cmd` `iq_ref_cmd` `ud_ref_cmd` `uq_ref_cmd`       | 查参考值链路是否正确传递  |
| 运行状态层 | `control_state` `driver_fault_active` `fast_loop_time_us` | 查状态机与实时性      |

如果你习惯把这个对象命名为 `g_motor_telemetry`，角色是一样的；但当前仓库实际实现以 `g_program_telemetry` 为准。

---

<a id="sec-7-quickstart"></a>

## 7. 快速开始

此处重点关注变量g_motor,控制参数结构体

### 7.1 开发环境

- IDE：Keil / MDK
- 工程文件：[`program/MDK-ARM/STM32G431_FOC.uvprojx`](program/MDK-ARM/STM32G431_FOC.uvprojx)
- CubeMX 工程：[`program/STM32G431_FOC.ioc`](program/STM32G431_FOC.ioc)
- 下载方式：CMSIS-DAP Debugger

### 7.2 程序上电后自动完成的内容

- `ADC1 / ADC2` 校准
- `TIM1` PWM 启动并保持 `50%` 占空比
- 三相电流零偏累计
- `ADC2 DMA` 开始采集 `VBUS / NTC`
- `MA600A` 首次读角

### 7.3 第一次跑起来的推荐配置

建议第一次出力先不要直接上速度环，而是先做开环电压测试：

```c
g_motor.current_loop_enable = 0;
g_motor.speed_loop_enable = 0;
g_motor.position_loop_enable = 0;
g_motor.ud_ref = 0.0f;
g_motor.uq_ref = 0.5f;   /* 从小值开始 */
g_motor.run_request = 1;   /*  最核心的启动信号，启动这个才开始运行发波 */
```

程序会先自动做对齐，再进入基于编码器角度的手动电压模式。

### 7.4 模式控制说明

该版本program一共有四个模式控制变量，与一个运行启动控制变量。其中每个模式变量负责一个环路，运行启动变量负责整个系统的使能。

```c
g_motor.current_loop_enable = 0; /* 是否用电流环控制，若失能则跑开环电压*/
g_motor.speed_loop_enable = 0;  /* 是否用速度环控制，若失能则单电流环或开环电压*/
g_motor.position_loop_enable = 0;/* 位置环，需使能速度环使用，否则无效果*/
g_motor.
g_motor.run_request = 1;   /*  最核心的启动信号，启动这个才开始运行发波 */
```

各个模式调试时，可以再继续看g_motor内部的ref变量，使能对应控制环的时候，需要手动给定对应的ref变量。

### 7.5 首次需要看的变量

| 👀 变量                  | 正常现象      |
| ---------------------- | --------- |
| `current_offset_ready` | 上电后置位     |
| `ia / ib / ic_meas`    | 静止时接近 `0` |
| `i_abc_sum`            | 接近 `0`    |
| `ma600a_angle_rad`     | 手动转动时连续变化 |
| `driver_fault_active`  | `0`       |
| `fast_loop_overrun`    | `0`       |

### 7.6 三环由内到外的闭环调试顺序

不要跳步，建议严格按“由内到外”执行：

| 阶段      | 🎯 目标  | 🔧 推荐配置           | 👀 重点观察                                          |
| ------- | ------ | ----------------- | ------------------------------------------------ |
| Stage 0 | 静态链路确认 | `run_request = 0` | `ia/ib/ic` `vbus` `ma600a_angle_rad`             |
| Stage 1 | 验证出力方向 | 关速度环、关位置环         | `uq_ref` `theta_elec`                            |
| Stage 2 | 调电流环   | 开电流环，关外环          | `id` `iq` `ud_ref_cmd` `uq_ref_cmd`              |
| Stage 3 | 调速度环   | 开速度环，位置环保持关闭      | `speed_ref_mech_rpm` `speed_meas_mech_rpm`       |
| Stage 4 | 调位置环   | 最后再开位置环           | `position_ref_mech_rad` `position_meas_mech_rad` |

### 7.7 当前运行默认参数

以下为当前主路径实际生效的默认参数，来源于 [`program/App/program.c`](program/App/program.c)：

电流环带宽计算，可以看代码注释，当前电流环带宽为1Khz。

| 🧪 参数         | 当前值       |
| ------------- | --------- |
| `current_kp`  | `2.5761`  |
| `current_ki`  | `4555.31` |
| `speed_kp`    | `0.003`   |
| `speed_ki`    | `0.03`    |
| `position_kp` | `3.0`     |
| `position_ki` | `0.0`     |
| `iq_limit`    | `8.0 A`   |

更详细的 bring-up 和三环调试步骤见 [`docs/quick-start.md`](docs/quick-start.md)。

---

<a id="sec-8-tree"></a>

## 8. 项目目录

```text
.
|-- README.md
|-- LICENSE
|-- circurit/                    # 原理图、器件手册、Gerber（目录名沿用当前仓库拼写）
|-- docs/
|   |-- quick-start.md           # 快速开始与三环调试说明
|   |-- figures/                 # 软件架构图、控制流程图
|   `-- images/                  # README 图片资源
|-- experiment/
|   |-- photo/                   # 波形截图、VOFA 截图、实拍图
|   |-- video/                   # 单项测试视频
|   `-- 测试报告.docx
`-- program/
    |-- STM32G431_FOC.ioc        # CubeMX 工程
    |-- Core/                    # HAL / IRQ / CubeMX 生成代码
    |-- App/                     # 控制算法与项目逻辑
    `-- MDK-ARM/                 # Keil 工程
```

---

<a id="sec-9-plan"></a>

## 9. 已知限制与后续计划

### 9.1 当前已确认的限制

- MA600A角度反馈链路，应当存在一定问题，导致速度环目前有一个波动始终存在，且1/4倍于电机机械转速，也就是2倍于电机出轴端的转速。
- 目前MA600A的反馈链路未做INL校准。

---

<a id="sec-10-files"></a>

## 10. 相关文件

- [快速开始与三环调试文档](docs/quick-start.md)
- [软件结构图](docs/figures/software-architecture.svg)
- [控制流程图](docs/figures/control-flow.svg)
- [CubeMX 工程](program/STM32G431_FOC.ioc)
- [Keil 工程](program/MDK-ARM/STM32G431_FOC.uvprojx)
- [测试报告](experiment/%E6%B5%8B%E8%AF%95%E6%8A%A5%E5%91%8A.docx)
- [实验图片目录](experiment/photo)
- [实验视频目录](experiment/video)

---

<a id="sec-11-license"></a>

## 11. 许可证说明

本项目基于 **GNU General Public License v3.0 (GPL 3.0)** 开源许可协议发布。

详细内容请参阅本仓库根目录下的 [`LICENSE`](LICENSE) 文件。

### 概要

- **允许**：自由使用、修改、商业衍生
- **要求**：衍生作品必须同样以 GPL 3.0 发布，并保留源码
- **禁止**：不以任何方式提供源码

### 引用方式

```text
MPS-FOC STM32G431
Copyright (C) 2026 MPS China University Program
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License.
```
