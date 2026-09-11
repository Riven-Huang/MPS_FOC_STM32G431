#ifndef UNITS_H
#define UNITS_H

/* ============================================================================
 * units.h — 单位换算与坐标系变换（全部 static inline，零运行时开销）
 *
 * 坐标系约定（全项目统一，禁止再混用 "mech"）：
 *   _enc  : 编码器原始机械角（MA600A 单圈）
 *   _rot  : 转子机械角（经方向符号/减速比换算后）
 *   _out  : 输出轴机械角（转子 / 减速比）
 *   _elec : 电角度（转子机械角 × 极对数）
 * ==========================================================================*/

#include <math.h>

#include "app_config.h"

#define UNITS_PI        3.14159265359f
#define UNITS_TWO_PI    6.28318530718f

/* 角度归一化到 [0, 2π)，常数时间 */
static inline float units_wrap_2pi(float angle_rad)
{
    float turns;

    if (!isfinite(angle_rad)) {
        return 0.0f;
    }

    turns = floorf(angle_rad / UNITS_TWO_PI);
    angle_rad -= turns * UNITS_TWO_PI;

    if (angle_rad < 0.0f) {
        angle_rad += UNITS_TWO_PI;
    } else if (angle_rad >= UNITS_TWO_PI) {
        angle_rad -= UNITS_TWO_PI;
    }

    return angle_rad;
}

/* 角度差归一化到 (-π, +π]，用于跨零界的最短路径 */
static inline float units_wrap_pm_pi(float delta_rad)
{
    if (!isfinite(delta_rad)) {
        return 0.0f;
    }

    return units_wrap_2pi(delta_rad + UNITS_PI) - UNITS_PI;
}

/* 角度归一化到 [0, 360) 度 */
static inline float units_wrap_360(float angle_deg)
{
    float turns;

    if (!isfinite(angle_deg)) {
        return 0.0f;
    }

    turns = floorf(angle_deg / 360.0f);
    angle_deg -= turns * 360.0f;

    if (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    } else if (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }

    return angle_deg;
}

static inline float units_rpm_to_rad_s(float rpm)
{
    return isfinite(rpm) ? (rpm * (UNITS_TWO_PI / 60.0f)) : 0.0f;
}

static inline float units_rad_s_to_rpm(float rad_s)
{
    return isfinite(rad_s) ? (rad_s * (60.0f / UNITS_TWO_PI)) : 0.0f;
}

static inline float units_deg_to_rad(float deg)
{
    return isfinite(deg) ? (deg * (UNITS_TWO_PI / 360.0f)) : 0.0f;
}

static inline float units_rad_to_deg(float rad)
{
    return isfinite(rad) ? (rad * (360.0f / UNITS_TWO_PI)) : 0.0f;
}

/* 编码器机械角 → 转子机械角 */
static inline float units_enc_to_rotor_rad(float enc_rad)
{
#if CFG_ENC_ON_OUTPUT_SHAFT
    return enc_rad * CFG_ENC_DIR_SIGN * CFG_MOTOR_GEAR_RATIO;
#else
    return enc_rad * CFG_ENC_DIR_SIGN;
#endif
}

/* 转子机械角 → 电角度（包绕到单圈） */
static inline float units_rotor_to_elec_rad(float rotor_rad)
{
    return units_wrap_2pi(rotor_rad * CFG_MOTOR_POLE_PAIRS);
}

/* 转子角速度 → 输出轴角速度 */
static inline float units_rotor_spd_to_out(float rot_rad_s)
{
    return rot_rad_s / CFG_MOTOR_GEAR_RATIO;
}

#endif /* UNITS_H */
