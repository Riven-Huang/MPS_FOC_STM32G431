#ifndef PROGRAM_H
#define PROGRAM_H

/* ============================================================================
 * program.h — 应用胶水层
 *
 * 本层只做三件事：
 *   1. program_init()：上电初始化（校准、外设链启动、控制器初始化）
 *   2. program_task()：while(1) 后台任务（只做 VOFA 遥测，不碰控制量）
 *   3. HAL 弱回调转发：ADC/TIM/SPI 中断 → ctrl 层
 *
 * 控制逻辑全部在 motor_ctrl.c，且在 10kHz 快环中断这唯一上下文内执行，
 * 对外接口就是 motor.h 的 g_cmd / g_fb。
 * ==========================================================================*/

#include "main.h"
#include "motor.h"

void program_init(void);
void program_task(void);

#endif /* PROGRAM_H */
