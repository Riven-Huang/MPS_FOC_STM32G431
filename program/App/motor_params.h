#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

#include <math.h>

#define MOTOR_POLE_PAIRS         14.0f
#define MOTOR_GEAR_RATIO         8.0f
#define MOTOR_ENCODER_ON_OUTPUT_SHAFT 0U
#define MOTOR_ENCODER_DIRECTION_SIGN (-1.0f)
#define MOTOR_TWO_PI             6.28318530718f

static inline float motor_params_wrap_angle_rad(float angle_rad)
{
    float turns;

    if (!isfinite(angle_rad)) {
        return 0.0f;
    }

    turns = floorf(angle_rad / MOTOR_TWO_PI);
    angle_rad -= turns * MOTOR_TWO_PI;

    if (angle_rad < 0.0f) {
        angle_rad += MOTOR_TWO_PI;
    } else if (angle_rad >= MOTOR_TWO_PI) {
        angle_rad -= MOTOR_TWO_PI;
    }

    return angle_rad;
}

static inline float motor_params_rotor_mech_to_elec_rad(float rotor_mech_angle_rad)
{
    return motor_params_wrap_angle_rad(rotor_mech_angle_rad * MOTOR_POLE_PAIRS);
}

static inline float motor_params_encoder_mech_to_rotor_mech_rad(float encoder_mech_angle_rad)
{
#if MOTOR_ENCODER_ON_OUTPUT_SHAFT
    return encoder_mech_angle_rad * MOTOR_ENCODER_DIRECTION_SIGN * MOTOR_GEAR_RATIO;
#else
    return encoder_mech_angle_rad * MOTOR_ENCODER_DIRECTION_SIGN;
#endif
}

static inline float motor_params_encoder_speed_to_rotor_speed(float encoder_speed_rad_s)
{
#if MOTOR_ENCODER_ON_OUTPUT_SHAFT
    return encoder_speed_rad_s * MOTOR_ENCODER_DIRECTION_SIGN * MOTOR_GEAR_RATIO;
#else
    return encoder_speed_rad_s * MOTOR_ENCODER_DIRECTION_SIGN;
#endif
}

static inline float motor_params_rotor_speed_to_output_speed(float rotor_speed_rad_s)
{
    return rotor_speed_rad_s / MOTOR_GEAR_RATIO;
}

#endif /* MOTOR_PARAMS_H */
