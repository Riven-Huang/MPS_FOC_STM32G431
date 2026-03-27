#include "motor_state.h"

#define MOTOR_ALIGN_TIME_MS           300U
#define MOTOR_OPEN_LOOP_TO_CLOSE_MS   1500U
#define MOTOR_OPEN_LOOP_SPEED_ELEC    20.0f
#define MOTOR_ALIGN_UD_V              0.8f
#define MOTOR_OPEN_LOOP_UQ_V          1.0f

/* 函数作用：执行一次状态切换并记录进入时间。
 * 输入：motor 为状态机对象，next_state 为目标状态，now_ms 为当前毫秒时间。
 * 输出：无返回值。
 * 调用频率：仅在发生状态切换时调用。
 * 运行内容：写入新的状态值，并记录状态进入时刻。 */
static void motor_state_enter(motor_state_t *motor, motor_state_id_t next_state, uint32_t now_ms)
{
    motor->state = next_state;
    motor->state_enter_ms = now_ms;
}

/* 函数作用：初始化电机状态机对象。
 * 输入：motor 为待初始化的状态机对象。
 * 输出：无返回值。
 * 调用频率：系统启动时调用一次。
 * 运行内容：设置默认状态、清零运行和故障变量，并初始化速度滤波器和 PI 占位对象。 */
void motor_state_init(motor_state_t *motor)
{
    if (motor == 0) {
        return;
    }

    motor->state = MOTOR_STATE_READY;
    motor->run_request = 0U;
    motor->speed_loop_enable = 1U;
    motor->current_loop_enable = 1U;
    motor->position_loop_enable = 0U;
    motor->align_done = 0U;
    motor->fault_code = MOTOR_FAULT_NONE;
    motor->state_enter_ms = 0U;
    motor->theta_open_loop = 0.0f;
    motor->open_loop_speed_elec = MOTOR_OPEN_LOOP_SPEED_ELEC;
    motor->id_ref = 0.0f;
    motor->iq_ref = 0.0f;
    motor->ud_ref = 0.0f;
    motor->uq_ref = 0.0f;
    motor->speed_ref_mech_rpm = 0.0f;
    motor->speed_ref_mech_rad_s = 0.0f;
    motor->speed_ref_mech_applied_rad_s = 0.0f;
    motor->speed_meas_mech_rad_s = 0.0f;
    motor->speed_ref_elec_rad_s = 0.0f;
    motor->speed_meas_elec_rad_s = 0.0f;
    motor->speed_kp = 0.001f;
    motor->speed_ki = 0.004f;
    motor->position_ref_mech_rad = 0.0f;
    motor->position_meas_mech_rad = 0.0f;
    motor->position_error_mech_rad = 0.0f;
    motor->position_kp = 8.0f;
    motor->position_ki = 0.0f;
    motor->position_integral_speed = 0.0f;
    motor->position_speed_limit_mech_rad_s = 20.0f;
    motor->speed_meas_lpf_cutoff_hz = 150.0f;
    motor->speed_integral_iq = 0.0f;
    motor->speed_integral_uq = 0.0f;
    motor->iq_limit = 2.0f;
    motor->current_kp = 0.5f;
    motor->current_ki = 200.0f;
    motor->id_integral_v = 0.0f;
    motor->iq_integral_v = 0.0f;
    motor->voltage_limit = 0.0f;

    filter_lpf_f32_init(&motor->speed_lpf, 0.1f, 0.0f);
    drv_pid_pi_init(&motor->speed_pi, 0, 0, -1000, 1000, 0);
}

/* 函数作用：执行一次电机状态机。
 * 输入：motor 为状态机对象，foc 为 FOC 核心对象，now_ms 为当前毫秒时间。
 * 输出：无返回值，状态变化和 FOC 输出直接写回对象。
 * 调用频率：当前由 program_task() 按约 1 kHz 调用。
 * 运行内容：处理 READY、ALIGN、OPEN_LOOP、CLOSED_LOOP、FAULT 各状态逻辑，并给 FOC 下发对应电压命令。 */
void motor_state_task(motor_state_t *motor, foc_core_t *foc, uint32_t now_ms)
{
    float elapsed_s;

    if ((motor == 0) || (foc == 0)) {
        return;
    }

    if (motor->fault_code != MOTOR_FAULT_NONE) {
        motor_state_enter(motor, MOTOR_STATE_FAULT, now_ms);
    }

    switch (motor->state) {
    case MOTOR_STATE_READY:
        foc_core_reset_output(foc);
        if (motor->run_request != 0U) {
            motor->align_done = 0U;
            motor_state_enter(motor, MOTOR_STATE_ALIGN, now_ms);
        }
        break;

    case MOTOR_STATE_ALIGN:
        /* 对齐阶段先固定电角度，建立转子零位参考。 */
        motor->theta_open_loop = 0.0f;
        motor->ud_ref = MOTOR_ALIGN_UD_V;
        motor->uq_ref = 0.0f;
        foc_core_run_voltage_open_loop(foc, motor->ud_ref, motor->uq_ref, motor->theta_open_loop, foc->vbus);

        if ((now_ms - motor->state_enter_ms) >= MOTOR_ALIGN_TIME_MS) {
            motor->align_done = 1U;
            motor_state_enter(motor, MOTOR_STATE_OPEN_LOOP, now_ms);
        }
        break;

    case MOTOR_STATE_OPEN_LOOP:
        elapsed_s = 0.001f * (float)(now_ms - motor->state_enter_ms);
        motor->theta_open_loop = elapsed_s * motor->open_loop_speed_elec;
        motor->ud_ref = 0.0f;
        motor->uq_ref = MOTOR_OPEN_LOOP_UQ_V;
        foc_core_run_voltage_open_loop(foc, motor->ud_ref, motor->uq_ref, motor->theta_open_loop, foc->vbus);

        if (motor->run_request == 0U) {
            motor_state_enter(motor, MOTOR_STATE_READY, now_ms);
        } else if ((now_ms - motor->state_enter_ms) >= MOTOR_OPEN_LOOP_TO_CLOSE_MS) {
            motor_state_enter(motor, MOTOR_STATE_CLOSED_LOOP, now_ms);
        }
        break;

    case MOTOR_STATE_CLOSED_LOOP:
        /* 当前位置先保留一个最小闭环占位，后续接电流环和速度环。 */
        motor->theta_open_loop += 0.001f * motor->open_loop_speed_elec;
        motor->ud_ref = 0.0f;
        motor->uq_ref = MOTOR_OPEN_LOOP_UQ_V;
        foc_core_run_voltage_open_loop(foc, motor->ud_ref, motor->uq_ref, motor->theta_open_loop, foc->vbus);

        if (motor->run_request == 0U) {
            motor_state_enter(motor, MOTOR_STATE_READY, now_ms);
        }
        break;

    case MOTOR_STATE_FAULT:
        foc_core_reset_output(foc);
        if ((motor->run_request == 0U) && (motor->fault_code == MOTOR_FAULT_NONE)) {
            motor_state_enter(motor, MOTOR_STATE_READY, now_ms);
        }
        break;

    case MOTOR_STATE_INIT:
    default:
        motor_state_enter(motor, MOTOR_STATE_READY, now_ms);
        break;
    }
}

/* 函数作用：设置运行请求标志。
 * 输入：motor 为状态机对象，enable 为 0 或 1。
 * 输出：无返回值。
 * 调用频率：外部控制逻辑按需调用。
 * 运行内容：只更新运行请求，真正的状态切换在 motor_state_task() 里完成。 */
void motor_state_set_run_request(motor_state_t *motor, uint8_t enable)
{
    if (motor == 0) {
        return;
    }

    motor->run_request = enable;
}

/* 函数作用：设置故障码。
 * 输入：motor 为状态机对象，fault_code 为故障类型。
 * 输出：无返回值。
 * 调用频率：检测到故障条件时按需调用。
 * 运行内容：写入故障码，状态机会在下一拍进入 FAULT。 */
void motor_state_set_fault(motor_state_t *motor, uint8_t fault_code)
{
    if (motor == 0) {
        return;
    }

    motor->fault_code = fault_code;
}

/* 函数作用：清除故障码。
 * 输入：motor 为状态机对象。
 * 输出：无返回值。
 * 调用频率：确认故障解除后按需调用。
 * 运行内容：把故障码恢复为 NONE，允许状态机回到 READY。 */
void motor_state_clear_fault(motor_state_t *motor)
{
    if (motor == 0) {
        return;
    }

    motor->fault_code = MOTOR_FAULT_NONE;
}

/* 函数作用：把状态枚举转换成可读字符串。
 * 输入：state 为状态枚举值。
 * 输出：返回状态名字符串常量指针。
 * 调用频率：调试打印时按需调用。
 * 运行内容：提供串口、上位机或日志需要的可读状态名。 */
const char *motor_state_get_name(motor_state_id_t state)
{
    switch (state) {
    case MOTOR_STATE_INIT:
        return "INIT";
    case MOTOR_STATE_READY:
        return "READY";
    case MOTOR_STATE_ALIGN:
        return "ALIGN";
    case MOTOR_STATE_OPEN_LOOP:
        return "OPEN_LOOP";
    case MOTOR_STATE_CLOSED_LOOP:
        return "CLOSED_LOOP";
    case MOTOR_STATE_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}
