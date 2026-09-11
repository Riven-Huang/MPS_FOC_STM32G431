#ifndef BOARD_H
#define BOARD_H

/* ============================================================================
 * board.h — 板级硬件抽象
 *
 * 职责：ADC 码值 → 工程量换算、N_SLEEP/N_FAULT GPIO、TIM1 占空比写入、
 *       DWT 周期计数（快环耗时统计）。
 * 所有硬件相关的换算系数都在 app_config.h 里。
 * ==========================================================================*/

#include <stdint.h>

void    board_init(void);

/* ADC 原始码 → 工程量 */
float   board_current_a(uint16_t raw, uint16_t offset_raw);
float   board_vbus_v(uint16_t raw);
float   board_ntc_c(uint16_t raw);

/* 功率级与故障引脚 */
void    board_set_power_en(uint8_t enable); /* 内部写变才写 GPIO */
uint8_t board_power_enabled(void);
uint8_t board_drv_fault(void);              /* 1 = N_FAULT 有效（低电平） */

/* TIM1 三相占空比下发（0.0~1.0，内部限幅并换算 CCR） */
void    board_write_duty(float duty_a, float duty_b, float duty_c);

/* DWT 周期计数（170MHz 下 1us ≈ 170 cycles） */
void    board_dwt_init(void);
uint8_t board_dwt_ok(void);
uint32_t board_cycles(void);

#endif /* BOARD_H */
