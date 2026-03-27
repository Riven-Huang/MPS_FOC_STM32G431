#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <stdint.h>

#include "drv_pid.h"
#include "filter.h"
#include "foc_core.h"

typedef enum
{
    MOTOR_STATE_INIT = 0,
    MOTOR_STATE_READY,
    MOTOR_STATE_ALIGN,
    MOTOR_STATE_OPEN_LOOP,
    MOTOR_STATE_CLOSED_LOOP,
    MOTOR_STATE_FAULT
} motor_state_id_t;

typedef enum
{
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_DRIVER = 1,
    MOTOR_FAULT_OVERVOLTAGE = 2
} motor_fault_t;

typedef struct
{
    motor_state_id_t state;
    uint8_t run_request;
    uint8_t speed_loop_enable;
    uint8_t current_loop_enable;
    uint8_t position_loop_enable;
    uint8_t align_done;
    uint8_t fault_code;
    uint32_t state_enter_ms;
    float theta_open_loop;
    float open_loop_speed_elec;
    float id_ref;
    float iq_ref;
    float ud_ref;
    float uq_ref;
    float speed_ref_mech_rpm;
    float speed_ref_mech_rad_s;
    float speed_ref_mech_applied_rad_s;
    float speed_meas_mech_rad_s;
    float speed_ref_elec_rad_s;
    float speed_meas_elec_rad_s;
    float speed_kp;
    float speed_ki;
    float position_ref_mech_rad;
    float position_meas_mech_rad;
    float position_error_mech_rad;
    float position_kp;
    float position_ki;
    float position_integral_speed;
    float position_speed_limit_mech_rad_s;
    float speed_meas_lpf_cutoff_hz;
    float speed_integral_iq;
    float speed_integral_uq;
    float iq_limit;
    float current_kp;
    float current_ki;
    float id_integral_v;
    float iq_integral_v;
    float voltage_limit;
    filter_lpf_f32_t speed_lpf;
    drv_pid_pi_t speed_pi;
} motor_state_t;

void motor_state_init(motor_state_t *motor);
void motor_state_task(motor_state_t *motor, foc_core_t *foc, uint32_t now_ms);
void motor_state_set_run_request(motor_state_t *motor, uint8_t enable);
void motor_state_set_fault(motor_state_t *motor, uint8_t fault_code);
void motor_state_clear_fault(motor_state_t *motor);
const char *motor_state_get_name(motor_state_id_t state);

#endif /* MOTOR_STATE_H */
