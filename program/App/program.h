#ifndef PROGRAM_H
#define PROGRAM_H

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "motor_state.h"
#include "foc_core.h"

typedef struct
{
    uint16_t ia_raw;
    uint16_t ib_raw;
    uint16_t ic_raw;
    uint16_t vbus_raw;
    uint16_t ntc_raw;
    uint16_t ma600a_angle_raw;
    uint16_t ia_offset_raw;
    uint16_t ib_offset_raw;
    uint16_t ic_offset_raw;
    uint16_t current_offset_sample_count;
    uint8_t current_offset_ready;
    uint8_t pwm_enable_cmd;
    uint8_t power_stage_enabled;
    uint8_t control_state;
    uint8_t encoder_align_done;
    uint8_t speed_loop_ready;
    uint8_t current_loop_enable;
    uint8_t position_loop_enable;
    uint8_t driver_fault_active;
    uint8_t ma600a_angle_valid;
    uint8_t fast_loop_overrun;
    uint32_t fast_loop_overrun_count;
    uint32_t fast_loop_cycles;
    uint32_t fast_loop_cycles_max;
    uint32_t ma600a_sample_counter;
    uint32_t speed_observer_window_samples;
    float ia;
    float ib;
    float ic;
    float ic_meas;
    float i_abc_sum;
    float id;
    float iq;
    float theta_elec;
    float duty_a;
    float duty_b;
    float duty_c;
    float vbus;
    float ma600a_angle_deg;
    float ma600a_angle_rad;
    float theta_open_loop;
    float id_ref_cmd;
    float iq_ref_cmd;
    float id_ref_applied_cmd;
    float iq_ref_applied_cmd;
    float ud_ref_cmd;
    float uq_ref_cmd;
    float open_loop_speed_elec;
    float speed_ref_mech_rad_s;
    float speed_ref_mech_applied_rad_s;
    float speed_meas_raw_mech_rad_s;
    float speed_meas_mech_rad_s;
    float speed_ref_mech_rpm;
    float speed_ref_mech_applied_rpm;
    float speed_meas_raw_mech_rpm;
    float speed_meas_mech_rpm;
    float speed_error_mech_rad_s;
    float position_ref_mech_rad;
    float position_meas_mech_rad;
    float position_error_mech_rad;
    float speed_loop_dt_s;
    float speed_meas_lpf_cutoff_hz;
    float speed_ref_elec_rad_s;
    float speed_meas_elec_rad_s;
    float uq_limit_v;
    float iq_limit_a;
    float voltage_limit_v;
    float encoder_elec_offset_rad;
    float fast_loop_time_us;
    float fast_loop_time_max_us;
    float fast_loop_period_us;
} program_telemetry_t;

extern volatile program_telemetry_t g_program_telemetry;
extern motor_state_t g_motor;
extern foc_core_t g_foc;

void program_init(void);
void program_task(void);
void program_tim_period_elapsed_callback(TIM_HandleTypeDef *htim);
void program_adc_conv_cplt_callback(ADC_HandleTypeDef *hadc);
void program_adc_injected_conv_cplt_callback(ADC_HandleTypeDef *hadc);

const volatile program_telemetry_t *program_get_telemetry(void);
motor_state_t *program_get_motor(void);
foc_core_t *program_get_foc(void);

#endif /* PROGRAM_H */
