#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

/* ============================================================================
 * motor_ctrl.h — 控制器总装模块
 *
 * 职责：唯一真实状态机（IDLE/ALIGN/RUN/FAULT）、保护、编码器观测器、
 *       零位对齐、电流/速度/位置三环、模式分发。
 *
 * 单上下文设计：全部控制、保护与慢速采样都在 10kHz 快环中断内执行
 * （VBUS/NTC/派生单位按 CFG_SLOW_DIV 分频到 1kHz），三环只靠带宽区分：
 *   带宽目标在 app_config.h 配置，PI 增益上电时自动计算（推导公式见其注释）。
 *
 * 调用关系：
 *   快环 10kHz：program.c 的 ADC1 注入组回调 → ctrl_fast_loop()
 *   SPI 回调  ：program.c 的 HAL_SPI 回调    → ctrl_spi_*()
 *   后台      ：program.c 的 while(1) 只做 VOFA 遥测，不碰控制量
 * ==========================================================================*/

#include "spi.h"

void ctrl_init(void);
void ctrl_fast_loop(uint16_t ia_raw, uint16_t ib_raw, uint16_t ic_raw,
                    uint16_t vbus_raw, uint16_t ntc_raw);

void ctrl_spi_txrx_cplt(SPI_HandleTypeDef *hspi);
void ctrl_spi_error(SPI_HandleTypeDef *hspi);

#endif /* MOTOR_CTRL_H */
