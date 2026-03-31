#include "program.h"

#include <math.h>
#include <string.h>

#include "cli_uart.h"
#include "filter.h"
#include "gpio.h"
#include "ma600a.h"
#include "motor_params.h"

#define PROGRAM_ADC_REF_V                     3.3f
#define PROGRAM_ADC_FULL_SCALE_COUNTS         4095.0f
#define PROGRAM_SHUNT_RESISTOR_OHM            0.01f
#define PROGRAM_CURRENT_SENSE_GAIN            20.0f
#define PROGRAM_VBUS_R_UP_OHM                 240000.0f
#define PROGRAM_VBUS_R_DOWN_OHM               10000.0f
#define PROGRAM_DEFAULT_CURRENT_OFFSET_RAW    2048U
#define PROGRAM_CURRENT_OFFSET_TARGET_SAMPLES 1024U
#define PROGRAM_DEFAULT_VBUS_V                48.0f
#define PROGRAM_PI                            3.14159265359f
#define PROGRAM_FAST_LOOP_HZ                  10000.0f
#define PROGRAM_FAST_LOOP_DT_S                (1.0f / PROGRAM_FAST_LOOP_HZ)
#define PROGRAM_FAST_LOOP_PERIOD_US           (1000000.0f / PROGRAM_FAST_LOOP_HZ)
#define PROGRAM_POSITION_LOOP_HZ              200.0f
#define PROGRAM_POSITION_LOOP_DT_S            (1.0f / PROGRAM_POSITION_LOOP_HZ)
#define PROGRAM_SPEED_OBSERVER_WINDOW_SAMPLES 20U
#define PROGRAM_DEFAULT_SPEED_MEAS_LPF_CUTOFF_HZ 150.0f
#define PROGRAM_SPEED_LOOP_DT_S               (PROGRAM_FAST_LOOP_DT_S * (float)PROGRAM_SPEED_OBSERVER_WINDOW_SAMPLES)
#define PROGRAM_ENCODER_LSB_RAD               (MOTOR_TWO_PI / 65536.0f)
#define PROGRAM_SPEED_MEAS_ZERO_HOLD_SCALE    8.0f
#define PROGRAM_SPEED_MEAS_ZERO_HOLD_MIN_MECH_RAD_S 0.35f
#define PROGRAM_OPEN_LOOP_DEFAULT_SPEED_ELEC  2000.0f
#define PROGRAM_OPEN_LOOP_DEFAULT_UD_V        0.0f
#define PROGRAM_OPEN_LOOP_DEFAULT_UQ_V        1.0f
#define PROGRAM_DEFAULT_SPEED_REF_MECH_RAD_S  (PROGRAM_OPEN_LOOP_DEFAULT_SPEED_ELEC / MOTOR_POLE_PAIRS)
#define PROGRAM_DEFAULT_SPEED_KP              0.0015f
#define PROGRAM_DEFAULT_SPEED_KI              0.015f
#define PROGRAM_DEFAULT_POSITION_KP           3.0f
#define PROGRAM_DEFAULT_POSITION_KI           0.0f
#define PROGRAM_DEFAULT_POSITION_KD           0.0f
#define PROGRAM_DEFAULT_POSITION_SPEED_LIMIT_MECH_RAD_S (PROGRAM_DEFAULT_SPEED_REF_MECH_RAD_S / MOTOR_GEAR_RATIO)
#define PROGRAM_POSITION_MEAS_LPF_CUTOFF_HZ   12.0f
#define PROGRAM_POSITION_HOLD_ERR_RAD         0.0040f
#define PROGRAM_POSITION_HOLD_RELEASE_ERR_RAD 0.0150f
#define PROGRAM_POSITION_HOLD_SPEED_MECH_RAD_S 0.50f
#define PROGRAM_POSITION_HOLD_RELEASE_CONFIRM_CYCLES 20U
#define PROGRAM_POSITION_CREEP_ENABLE_ERR_RAD 0.0300f
#define PROGRAM_POSITION_CREEP_SPEED_MECH_RAD_S 0.10f
#define PROGRAM_DEFAULT_IQ_LIMIT_A            12.00f
/* Equivalent plant inferred from the legacy 1 kHz tuning:
 * 1 kHz -> kp 2.5761, ki 4555.31
 * 2 kHz -> kp 5.1522, ki 9110.62
 * 3 kHz -> kp 7.7283, ki 13665.93 */
#define PROGRAM_CURRENT_LOOP_EQ_RESISTANCE_OHM 0.7250f
#define PROGRAM_CURRENT_LOOP_EQ_INDUCTANCE_H   0.0004100f
#define PROGRAM_CURRENT_LOOP_BW_1KHZ_HZ        1000.0f
#define PROGRAM_CURRENT_LOOP_BW_2KHZ_HZ        2000.0f
#define PROGRAM_CURRENT_LOOP_BW_3KHZ_HZ        3000.0f
#define PROGRAM_DEFAULT_CURRENT_LOOP_BANDWIDTH_HZ PROGRAM_CURRENT_LOOP_BW_1KHZ_HZ
#define PROGRAM_CURRENT_REF_RAMP_A_PER_S      150.0f
#define PROGRAM_VOLTAGE_LIMIT_RATIO           0.57735026919f
#define PROGRAM_SPEED_REF_RAMP_RAD_S2         100.0f
#define PROGRAM_ALIGN_UD_V                    1.8f
#define PROGRAM_ALIGN_HOLD_TICKS              8000U
#define PROGRAM_ALIGN_SAMPLE_WINDOW_TICKS     512U
#define PROGRAM_WAVE_PERIOD_MS                2U
#define PROGRAM_ADC2_DMA_LENGTH               2U
#define PROGRAM_ENCODER_OBSERVER_RENORM_TURNS 32.0f
#define PROGRAM_ENCODER_OBSERVER_RENORM_RAD   (PROGRAM_ENCODER_OBSERVER_RENORM_TURNS * MOTOR_TWO_PI)
#define PROGRAM_CURRENT_SIGN_IA               (1.0f)
#define PROGRAM_CURRENT_SIGN_IB               (1.0f)
#define PROGRAM_CURRENT_SIGN_IC               (1.0f)

/* 全局控制对象：
 * 电机状态机、FOC 核心、编码器驱动和调试遥测都由 program 层统一持有。 */
motor_state_t g_motor;
foc_core_t g_foc;
static ma600a_t g_ma600a;
volatile program_telemetry_t g_program_telemetry;
static filter_lpf_f32_t g_vbus_lpf;

/* 慢速采样与后台调度状态：
 * ADC2 DMA 负责慢变量采样，TIM6 提供 1 ms 后台任务节拍。 */
static volatile uint16_t g_adc2_dma_buf[PROGRAM_ADC2_DMA_LENGTH];
static volatile uint32_t g_tim6_tick_ms = 0U;
static volatile uint32_t g_ia_offset_sum = 0U;
static volatile uint32_t g_ib_offset_sum = 0U;
static volatile uint32_t g_ic_offset_sum = 0U;
static uint32_t g_last_slow_task_tick_ms = 0U;
static uint32_t g_last_wave_tick_ms = 0U;
static uint8_t g_tim1_pwm_started = 0U;
static uint8_t g_power_stage_enabled = 0U;

/* 编码器观测器与零位对齐状态：
 * 连续机械角、测速窗口、对齐采样累积量和电角偏置都在快环中维护。 */
static uint32_t g_encoder_last_sample_counter = 0U;
static uint8_t g_encoder_speed_primed = 0U;
static uint8_t g_encoder_speed_ready = 0U;
static uint8_t g_speed_loop_update_pending = 0U;
static float g_encoder_prev_mech_angle_rad = 0.0f;
static float g_encoder_continuous_mech_rad = 0.0f;
static float g_encoder_speed_window_start_mech_rad = 0.0f;
static uint32_t g_encoder_speed_window_sample_count = 0U;
static float g_encoder_speed_raw_mech_rad_s = 0.0f;
static uint8_t g_encoder_align_done = 0U;
static uint32_t g_encoder_align_counter = 0U;
static float g_encoder_elec_offset_rad = 0.0f;
static float g_encoder_align_sum_sin = 0.0f;
static float g_encoder_align_sum_cos = 0.0f;
static uint32_t g_encoder_align_sample_count = 0U;

/* 控制环运行时状态：
 * 包括电流给定斜坡、速度环实际 dt、位置环分频累计和快环耗时统计。 */
static float g_id_ref_applied_a = 0.0f;
static float g_iq_ref_applied_a = 0.0f;
static filter_lpf_f32_t g_speed_meas_lpf;
static filter_lpf_f32_t g_position_meas_lpf;
static float g_speed_loop_dt_s = PROGRAM_SPEED_LOOP_DT_S;
static float g_position_loop_elapsed_s = 0.0f;
static uint8_t g_dwt_cycle_counter_ready = 0U;
static uint32_t g_fast_loop_period_cycles = 1U;
static uint8_t g_position_loop_enable_prev = 0U;
static uint8_t g_position_hold_active = 0U;
static uint8_t g_position_hold_release_counter = 0U;
static uint8_t g_current_loop_enable_prev = 1U;
static float g_position_meas_output_continuous_rad = 0.0f;

static float program_run_pi_f32(float ref,
                                float feedback,
                                float kp,
                                float ki,
                                float dt_s,
                                float *integral,
                                float out_min,
                                float out_max);

/* 函数作用：限制浮点量上下界。
 * 输入：value、min_value、max_value。输出：返回限幅结果。调用频率：各控制环按需调用。运行内容：为电流、电压和速度中间量提供统一限幅。 */
static float program_clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/* 函数作用：由截止频率和采样周期换算一阶低通滤波系数。
 * 输入：cutoff_hz、dt_s。输出：返回 0~1 的 alpha。调用频率：滤波器初始化或更新系数时调用。运行内容：按连续系统指数离散化方式生成 LPF 系数。 */
static float program_lpf_alpha_from_cutoff_hz(float cutoff_hz, float dt_s)
{
    float alpha;

    if ((!isfinite(cutoff_hz)) || (!isfinite(dt_s)) || (cutoff_hz <= 0.0f) || (dt_s <= 0.0f)) {
        return 1.0f;
    }

    alpha = 1.0f - expf(-MOTOR_TWO_PI * cutoff_hz * dt_s);
    return program_clamp_f32(alpha, 0.0f, 1.0f);
}

static float program_current_loop_kp_from_bandwidth_hz(float bandwidth_hz)
{
    if ((!isfinite(bandwidth_hz)) || (bandwidth_hz <= 0.0f)) {
        return 0.0f;
    }

    return MOTOR_TWO_PI * bandwidth_hz * PROGRAM_CURRENT_LOOP_EQ_INDUCTANCE_H;
}

static float program_current_loop_ki_from_bandwidth_hz(float bandwidth_hz)
{
    if ((!isfinite(bandwidth_hz)) || (bandwidth_hz <= 0.0f)) {
        return 0.0f;
    }

    return MOTOR_TWO_PI * bandwidth_hz * PROGRAM_CURRENT_LOOP_EQ_RESISTANCE_OHM;
}

static void program_update_control_angle_open_loop_state(void)
{
    if ((g_motor.control_angle_open_loop_enable == 0U) || (g_encoder_align_done == 0U)) {
        return;
    }

    g_motor.theta_open_loop =
        motor_params_wrap_angle_rad(g_motor.theta_open_loop +
                                    (g_motor.control_angle_open_loop_speed_elec * PROGRAM_FAST_LOOP_DT_S));
}

/* 函数作用：把角度归一化到 0~2pi。
 * 输入：angle_rad。输出：返回归一化后的角度。调用频率：角度采样和测速处理中按需调用。运行内容：通过整圈折返保证角度始终落在单圈范围内。 */
static float program_wrap_angle_0_2pi(float angle_rad)
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

/* 函数作用：把角度差归一化到 -pi~pi。
 * 输入：angle_rad。输出：返回最短角度差。调用频率：位置误差和速度观测按需调用。运行内容：避免跨 0/2pi 边界时出现跳变。 */
static float program_wrap_delta_pm_pi(float angle_rad)
{
    if (!isfinite(angle_rad)) {
        return 0.0f;
    }

    return program_wrap_angle_0_2pi(angle_rad + PROGRAM_PI) - PROGRAM_PI;
}

static float program_wrap_angle_0_360_deg(float angle_deg)
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

/* 函数作用：估算编码器测速量化分辨率。
 * 输入：observer_window_samples 为测速窗口样本数。输出：返回对应的最小速度分辨率，单位 rad/s。调用频率：速度测量更新时调用。运行内容：按编码器 LSB 和测速窗口长度换算速度量化台阶。 */
static float program_get_speed_quantization_rad_s(uint32_t observer_window_samples)
{
    if (observer_window_samples == 0U) {
        return 0.0f;
    }

    return PROGRAM_ENCODER_LSB_RAD / (PROGRAM_FAST_LOOP_DT_S * (float)observer_window_samples);
}

/* 函数作用：在低速附近抑制编码器量化抖动。
 * 输入：speed_mech_rad_s、observer_window_samples。输出：返回保护后的机械角速度。调用频率：每次刷新速度测量后调用。运行内容：当速度指令和测速值都接近量化台阶时强制回零。 */
static float program_apply_speed_quantization_guard(float speed_mech_rad_s,
                                                    uint32_t observer_window_samples)
{
    float zero_hold_threshold_rad_s;

    if (!isfinite(speed_mech_rad_s)) {
        return 0.0f;
    }

    zero_hold_threshold_rad_s =
        PROGRAM_SPEED_MEAS_ZERO_HOLD_SCALE * program_get_speed_quantization_rad_s(observer_window_samples);
    if ((!isfinite(zero_hold_threshold_rad_s)) || (zero_hold_threshold_rad_s <= 0.0f)) {
        return speed_mech_rad_s;
    }
    if (zero_hold_threshold_rad_s < PROGRAM_SPEED_MEAS_ZERO_HOLD_MIN_MECH_RAD_S) {
        zero_hold_threshold_rad_s = PROGRAM_SPEED_MEAS_ZERO_HOLD_MIN_MECH_RAD_S;
    }

    if ((fabsf(g_motor.speed_ref_mech_applied_rad_s) <= zero_hold_threshold_rad_s) &&
        (fabsf(speed_mech_rad_s) <= zero_hold_threshold_rad_s)) {
        return 0.0f;
    }

    return speed_mech_rad_s;
}

/* 函数作用：读取编码器对应的转子机械角。
 * 输入：无。输出：返回转子机械角，单位 rad。调用频率：编码器采样和角度换算时按需调用。运行内容：把减速器输出侧编码器角转换成转子侧机械角。 */
static float program_get_encoder_rotor_mech_angle_rad(void)
{
    return motor_params_encoder_mech_to_rotor_mech_rad(g_ma600a.angle_rad);
}

/* 函数作用：获取输出轴机械角。
 * 输入：无。输出：返回输出轴单圈机械角，单位 rad。调用频率：位置环和调试遥测更新时调用。运行内容：优先使用连续机械角观测值，再按减速比折算回输出轴。 */
static float program_get_encoder_output_continuous_mech_angle_rad(void)
{
    float rotor_mech_angle_rad;

    if (g_encoder_speed_primed != 0U) {
        rotor_mech_angle_rad = g_encoder_continuous_mech_rad;
    } else {
        rotor_mech_angle_rad = program_get_encoder_rotor_mech_angle_rad();
    }

    if ((!isfinite(rotor_mech_angle_rad)) || (MOTOR_GEAR_RATIO <= 0.0f)) {
        return 0.0f;
    }

    return rotor_mech_angle_rad / MOTOR_GEAR_RATIO;
}

static float program_get_encoder_output_mech_angle_rad(void)
{
    return motor_params_wrap_angle_rad(program_get_encoder_output_continuous_mech_angle_rad());
}

/* 函数作用：获取未经零位补偿的原始电角度。
 * 输入：无。输出：返回原始电角度，单位 rad。调用频率：对齐采样和故障前角度观察时调用。运行内容：由转子机械角和极对数直接换算电角度。 */
static float program_get_encoder_raw_elec_angle_rad(void)
{
    return motor_params_rotor_mech_to_elec_rad(program_get_encoder_rotor_mech_angle_rad());
}

/* 函数作用：获取完成零位补偿后的电角度。
 * 输入：无。输出：返回对齐后的控制电角度，单位 rad。调用频率：闭环控制和遥测更新时调用。运行内容：在原始电角度基础上扣除对齐得到的电角偏置。 */
static float program_get_encoder_aligned_elec_angle_rad(void)
{
    return motor_params_wrap_angle_rad(program_get_encoder_raw_elec_angle_rad() - g_encoder_elec_offset_rad);
}

/* 函数作用：提供当前控制使用的电角度。
 * 输入：无。输出：返回控制电角度，单位 rad。调用频率：快环每次执行控制时调用。运行内容：集中封装控制角来源，便于后续切换角度来源。 */
static float program_get_control_elec_angle_rad(void)
{
    if (g_motor.control_angle_open_loop_enable != 0U) {
        return motor_params_wrap_angle_rad(g_motor.theta_open_loop);
    }
    return program_get_encoder_aligned_elec_angle_rad();
}

/* 函数作用：把角速度从 rad/s 转成 rpm。
 * 输入：speed_rad_s。输出：返回 rpm。调用频率：遥测和参数初始化时按需调用。运行内容：统一处理程序内部与上位机常用单位之间的转换。 */
static float program_rad_s_to_rpm(float speed_rad_s)
{
    return speed_rad_s * (60.0f / MOTOR_TWO_PI);
}

static float program_rad_to_deg(float angle_rad)
{
    if (!isfinite(angle_rad)) {
        return 0.0f;
    }

    return angle_rad * (360.0f / MOTOR_TWO_PI);
}

static float program_deg_to_rad(float angle_deg)
{
    if (!isfinite(angle_deg)) {
        return 0.0f;
    }

    return angle_deg * (MOTOR_TWO_PI / 360.0f);
}

/* 函数作用：把转速从 rpm 转成 rad/s。
 * 输入：speed_rpm。输出：返回 rad/s。调用频率：速度给定更新和初始化时调用。运行内容：对非法输入做保护后完成单位换算。 */
static float program_rpm_to_rad_s(float speed_rpm)
{
    if (!isfinite(speed_rpm)) {
        return 0.0f;
    }

    return speed_rpm * (MOTOR_TWO_PI / 60.0f);
}

/* 函数作用：复位电流给定斜坡状态。
 * 输入：无。输出：无返回值。调用频率：停机、模式切换或故障恢复时调用。运行内容：把 d/q 轴实际生效给定拉回 0A。 */
static void program_reset_current_reference_ramp(void)
{
    g_id_ref_applied_a = 0.0f;
    g_iq_ref_applied_a = 0.0f;
}

/* 函数作用：复位速度环内部状态。
 * 输入：无。输出：无返回值。调用频率：启停、模式切换和异常恢复时调用。运行内容：清空速度环积分与挂起标志，避免旧状态带入下一轮控制。 */
static void program_reset_speed_loop(void)
{
    g_motor.speed_integral_iq = 0.0f;
    g_motor.speed_integral_uq = 0.0f;
    g_motor.iq_ref = 0.0f;
    if (g_motor.speed_loop_enable != 0U) {
        g_motor.uq_ref = 0.0f;
    }
    g_speed_loop_update_pending = 0U;
    g_speed_loop_dt_s = PROGRAM_SPEED_LOOP_DT_S;
}

/* 函数作用：复位位置环内部状态。
 * 输入：无。输出：无返回值。调用频率：位置环切换、停机或异常恢复时调用。运行内容：清空位置积分器和累计时间基准。 */
static void program_reset_position_loop(void)
{
    g_motor.position_integral_speed = 0.0f;
    g_motor.position_error_mech_deg = 0.0f;
    g_motor.position_error_mech_rad = 0.0f;
    g_position_loop_elapsed_s = 0.0f;
    g_position_hold_active = 0U;
    g_position_hold_release_counter = 0U;
    g_position_meas_output_continuous_rad = 0.0f;
    filter_lpf_f32_init(&g_position_meas_lpf,
                        program_lpf_alpha_from_cutoff_hz(PROGRAM_POSITION_MEAS_LPF_CUTOFF_HZ,
                                                         PROGRAM_POSITION_LOOP_DT_S),
                        0.0f);
    g_position_meas_lpf.initialized = 0U;
}

/* 函数作用：复位速度参考斜坡。
 * 输入：无。输出：无返回值。调用频率：启停和模式切换时调用。运行内容：把机械、电角速度给定及开环速度同步清零。 */
static void program_reset_speed_reference_ramp(void)
{
    g_motor.speed_ref_mech_applied_rad_s = 0.0f;
    g_motor.speed_ref_elec_rad_s = 0.0f;
    g_motor.open_loop_speed_elec = 0.0f;
}

/* 函数作用：按设定加速度更新实际速度给定。
 * 输入：无。输出：无返回值。调用频率：快环内每次速度控制前调用。运行内容：把上层目标速度平滑过渡到实际生效速度，减少转矩冲击。 */
static void program_update_speed_reference_ramp(void)
{
    float speed_step_rad_s;
    float speed_delta_rad_s;

    if (g_motor.position_loop_enable == 0U) {
        g_motor.speed_ref_mech_rad_s = program_rpm_to_rad_s(g_motor.speed_ref_mech_rpm);
    }

    speed_step_rad_s = PROGRAM_SPEED_REF_RAMP_RAD_S2 * PROGRAM_FAST_LOOP_DT_S;
    speed_delta_rad_s = g_motor.speed_ref_mech_rad_s - g_motor.speed_ref_mech_applied_rad_s;
    speed_delta_rad_s = program_clamp_f32(speed_delta_rad_s, -speed_step_rad_s, speed_step_rad_s);

    g_motor.speed_ref_mech_applied_rad_s += speed_delta_rad_s;
    g_motor.speed_ref_elec_rad_s = g_motor.speed_ref_mech_applied_rad_s * MOTOR_POLE_PAIRS;
    g_motor.open_loop_speed_elec = g_motor.speed_ref_elec_rad_s;
}

/* 函数作用：复位电流环内部状态。
 * 输入：无。输出：无返回值。调用频率：停机、模式切换和故障恢复时调用。运行内容：清零 d/q 轴给定、电压命令和 PI 积分器。 */
static void program_reset_current_loop(void)
{
    g_motor.id_ref = 0.0f;
    g_motor.iq_ref = 0.0f;
    g_motor.ud_ref = 0.0f;
    g_motor.uq_ref = 0.0f;
    g_motor.id_integral_v = 0.0f;
    g_motor.iq_integral_v = 0.0f;
    g_motor.voltage_limit = 0.0f;
    program_reset_current_reference_ramp();
}

/* 函数作用：复位编码器零位对齐结果。
 * 输入：无。输出：无返回值。调用频率：重新对齐、停机或读角失效时调用。运行内容：清空零位偏置和对齐完成标志。 */
static void program_reset_encoder_alignment(void)
{
    g_encoder_align_done = 0U;
    g_encoder_align_counter = 0U;
    g_encoder_elec_offset_rad = 0.0f;
    g_motor.align_done = 0U;
}

/* 函数作用：复位编码器测速观测器。
 * 输入：无。输出：无返回值。调用频率：上电初始化、读角失效或重新对齐时调用。运行内容：清空连续角、测速窗口、滤波器和相关遥测量。 */
static void program_reset_encoder_observer(void)
{
    g_encoder_last_sample_counter = 0U;
    g_encoder_speed_primed = 0U;
    g_encoder_speed_ready = 0U;
    g_encoder_prev_mech_angle_rad = 0.0f;
    g_encoder_continuous_mech_rad = 0.0f;
    g_encoder_speed_window_start_mech_rad = 0.0f;
    g_encoder_speed_window_sample_count = 0U;
    g_encoder_speed_raw_mech_rad_s = 0.0f;
    g_motor.speed_meas_mech_rad_s = 0.0f;
    g_motor.speed_meas_elec_rad_s = 0.0f;
    g_motor.position_meas_mech_deg = 0.0f;
    g_motor.position_meas_mech_rad = 0.0f;
    g_speed_loop_update_pending = 0U;
    g_speed_loop_dt_s = PROGRAM_SPEED_LOOP_DT_S;
    g_position_meas_output_continuous_rad = 0.0f;
    filter_lpf_f32_init(&g_position_meas_lpf,
                        program_lpf_alpha_from_cutoff_hz(PROGRAM_POSITION_MEAS_LPF_CUTOFF_HZ,
                                                         PROGRAM_POSITION_LOOP_DT_S),
                        0.0f);
    g_position_meas_lpf.initialized = 0U;
    filter_lpf_f32_init(&g_speed_meas_lpf,
                        program_lpf_alpha_from_cutoff_hz(g_motor.speed_meas_lpf_cutoff_hz,
                                                         PROGRAM_SPEED_LOOP_DT_S),
                        0.0f);
}

/* 函数作用：执行位置环并生成机械速度指令。
 * 输入：position_loop_dt_s 为本次位置环实际周期。输出：无返回值。调用频率：位置环分频到达时调用。运行内容：计算位置误差并输出限幅后的转子机械速度参考。 */
static void program_update_position_loop(float position_loop_dt_s)
{
    float position_ref_wrapped_rad;
    float position_meas_raw_output_continuous_rad;
    float position_meas_wrapped_rad;
    float position_error_wrapped_rad;
    float position_speed_limit_mech_rad_s;
    float position_speed_cmd_output_mech_rad_s;
    float position_speed_meas_output_mech_rad_s;
    float position_error_abs_rad;
    float position_meas_filter_alpha;
    float speed_meas_abs_rad_s;

    if ((g_motor.position_loop_enable == 0U) || (g_motor.speed_loop_enable == 0U)) {
        g_motor.position_error_mech_deg = 0.0f;
        g_motor.position_error_mech_rad = 0.0f;
        return;
    }

    if (g_encoder_speed_primed == 0U) {
        g_motor.position_integral_speed = 0.0f;
        g_motor.position_error_mech_deg = 0.0f;
        g_motor.position_error_mech_rad = 0.0f;
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }

    if ((!isfinite(position_loop_dt_s)) || (position_loop_dt_s <= 0.0f)) {
        position_loop_dt_s = PROGRAM_SPEED_LOOP_DT_S;
    }

    g_motor.position_ref_mech_deg = program_wrap_angle_0_360_deg(g_motor.position_ref_mech_deg);
    position_ref_wrapped_rad = program_deg_to_rad(g_motor.position_ref_mech_deg);
    position_meas_raw_output_continuous_rad = program_get_encoder_output_continuous_mech_angle_rad();
    if ((!isfinite(position_ref_wrapped_rad)) || (!isfinite(position_meas_raw_output_continuous_rad))) {
        program_reset_position_loop();
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }

    position_meas_filter_alpha =
        program_lpf_alpha_from_cutoff_hz(PROGRAM_POSITION_MEAS_LPF_CUTOFF_HZ, position_loop_dt_s);
    g_position_meas_lpf.alpha = position_meas_filter_alpha;
    if (g_position_meas_lpf.initialized == 0U) {
        filter_lpf_f32_init(&g_position_meas_lpf,
                            position_meas_filter_alpha,
                            position_meas_raw_output_continuous_rad);
    }

    g_position_meas_output_continuous_rad =
        filter_lpf_f32_update(&g_position_meas_lpf, position_meas_raw_output_continuous_rad);
    if (!isfinite(g_position_meas_output_continuous_rad)) {
        program_reset_position_loop();
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }

    position_meas_wrapped_rad = motor_params_wrap_angle_rad(g_position_meas_output_continuous_rad);
    position_error_wrapped_rad = program_wrap_delta_pm_pi(position_ref_wrapped_rad - position_meas_wrapped_rad);
    if (!isfinite(position_error_wrapped_rad)) {
        program_reset_position_loop();
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }

    g_motor.position_ref_mech_rad = position_ref_wrapped_rad;
    g_motor.position_meas_mech_deg = program_wrap_angle_0_360_deg(program_rad_to_deg(position_meas_wrapped_rad));
    g_motor.position_meas_mech_rad = position_meas_wrapped_rad;
    g_motor.position_error_mech_deg = program_rad_to_deg(position_error_wrapped_rad);
    g_motor.position_error_mech_rad = position_error_wrapped_rad;
    position_error_abs_rad = fabsf(position_error_wrapped_rad);
    position_speed_meas_output_mech_rad_s =
        motor_params_rotor_speed_to_output_speed(g_motor.speed_meas_mech_rad_s);
    speed_meas_abs_rad_s = fabsf(position_speed_meas_output_mech_rad_s);

    if ((position_error_abs_rad <= PROGRAM_POSITION_HOLD_ERR_RAD) &&
        (speed_meas_abs_rad_s <= PROGRAM_POSITION_HOLD_SPEED_MECH_RAD_S)) {
        g_position_hold_active = 1U;
        g_position_hold_release_counter = 0U;
        g_motor.position_integral_speed = 0.0f;
        g_motor.position_error_mech_deg = 0.0f;
        g_motor.position_error_mech_rad = 0.0f;
        g_motor.speed_ref_mech_rad_s = 0.0f;
        return;
    }

    if (g_position_hold_active != 0U) {
        if (position_error_abs_rad <= PROGRAM_POSITION_HOLD_RELEASE_ERR_RAD) {
            g_position_hold_release_counter = 0U;
            g_motor.position_integral_speed = 0.0f;
            g_motor.position_error_mech_deg = 0.0f;
            g_motor.position_error_mech_rad = 0.0f;
            g_motor.speed_ref_mech_rad_s = 0.0f;
            return;
        }

        g_position_hold_release_counter++;
        if (g_position_hold_release_counter < PROGRAM_POSITION_HOLD_RELEASE_CONFIRM_CYCLES) {
            g_motor.position_integral_speed = 0.0f;
            g_motor.position_error_mech_deg = 0.0f;
            g_motor.position_error_mech_rad = 0.0f;
            g_motor.speed_ref_mech_rad_s = 0.0f;
            return;
        }

        g_position_hold_active = 0U;
        g_position_hold_release_counter = 0U;
    }

    position_speed_limit_mech_rad_s = g_motor.position_speed_limit_mech_rad_s;
    if ((!isfinite(position_speed_limit_mech_rad_s)) || (position_speed_limit_mech_rad_s <= 0.0f)) {
        position_speed_limit_mech_rad_s = PROGRAM_DEFAULT_POSITION_SPEED_LIMIT_MECH_RAD_S;
    }

    position_speed_cmd_output_mech_rad_s = program_run_pi_f32(position_error_wrapped_rad,
                                                              0.0f,
                                                              g_motor.position_kp,
                                                              g_motor.position_ki,
                                                              position_loop_dt_s,
                                                              &g_motor.position_integral_speed,
                                                              -position_speed_limit_mech_rad_s,
                                                              position_speed_limit_mech_rad_s);
    position_speed_cmd_output_mech_rad_s -=
        g_motor.position_kd * position_speed_meas_output_mech_rad_s;
    position_speed_cmd_output_mech_rad_s =
        program_clamp_f32(position_speed_cmd_output_mech_rad_s,
                          -position_speed_limit_mech_rad_s,
                          position_speed_limit_mech_rad_s);

    /* Without Ki, the command can become too small to overcome friction or the
     * low-speed quantization guard. Keep a tiny crawl command outside hold. */
    if ((position_error_abs_rad > PROGRAM_POSITION_CREEP_ENABLE_ERR_RAD) &&
        (fabsf(position_speed_cmd_output_mech_rad_s) < PROGRAM_POSITION_CREEP_SPEED_MECH_RAD_S) &&
        (speed_meas_abs_rad_s <= PROGRAM_POSITION_HOLD_SPEED_MECH_RAD_S)) {
        position_speed_cmd_output_mech_rad_s =
            copysignf(PROGRAM_POSITION_CREEP_SPEED_MECH_RAD_S, position_error_wrapped_rad);
    }

    /* Position loop runs in output-shaft coordinates.
     * Convert output-shaft speed command to rotor mechanical speed for the speed loop. */
    g_motor.speed_ref_mech_rad_s = position_speed_cmd_output_mech_rad_s * MOTOR_GEAR_RATIO;
}

/* 函数作用：处理位置环使能状态切换。
 * 输入：无。输出：无返回值。调用频率：快环内每次控制前调用。运行内容：检测位置环开关沿并同步复位相关状态与参考量。 */
static void program_handle_position_loop_mode_switch(void)
{
    uint8_t position_loop_enable_now;

    position_loop_enable_now = (g_motor.position_loop_enable != 0U) ? 1U : 0U;
    if (position_loop_enable_now == g_position_loop_enable_prev) {
        return;
    }

    g_position_loop_enable_prev = position_loop_enable_now;
    program_reset_position_loop();
    program_reset_speed_loop();
    program_reset_speed_reference_ramp();

    if (position_loop_enable_now != 0U) {
        if (g_encoder_speed_primed != 0U) {
            g_position_meas_output_continuous_rad = program_get_encoder_output_continuous_mech_angle_rad();
            filter_lpf_f32_init(&g_position_meas_lpf,
                                program_lpf_alpha_from_cutoff_hz(PROGRAM_POSITION_MEAS_LPF_CUTOFF_HZ,
                                                                 PROGRAM_POSITION_LOOP_DT_S),
                                g_position_meas_output_continuous_rad);
            g_motor.position_meas_mech_rad =
                motor_params_wrap_angle_rad(g_position_meas_output_continuous_rad);
            g_motor.position_meas_mech_deg =
                program_wrap_angle_0_360_deg(program_rad_to_deg(g_motor.position_meas_mech_rad));
            g_motor.position_ref_mech_deg = g_motor.position_meas_mech_deg;
            g_motor.position_ref_mech_rad = g_motor.position_meas_mech_rad;
        } else {
            g_motor.position_ref_mech_deg = 0.0f;
            g_motor.position_ref_mech_rad = 0.0f;
            g_motor.position_meas_mech_deg = 0.0f;
            g_motor.position_meas_mech_rad = 0.0f;
        }
    }
}

/* 函数作用：处理电流环使能状态切换。
 * 输入：无。输出：无返回值。调用频率：快环内每次控制前调用。运行内容：在电流环模式切换时清空速度环和电流环残留状态。 */
static void program_handle_current_loop_mode_switch(void)
{
    uint8_t current_loop_enable_now;

    current_loop_enable_now = (g_motor.current_loop_enable != 0U) ? 1U : 0U;
    if (current_loop_enable_now == g_current_loop_enable_prev) {
        return;
    }

    g_current_loop_enable_prev = current_loop_enable_now;
    program_reset_speed_loop();
    program_reset_current_loop();
    g_motor.id_ref = 0.0f;
    g_motor.iq_ref = 0.0f;
    g_motor.ud_ref = 0.0f;
    g_motor.uq_ref = 0.0f;
}

/* 函数作用：执行带抗饱和的浮点 PI 计算。
 * 输入：ref、feedback、kp、ki、dt_s、integral、out_min、out_max。输出：返回本次 PI 输出。调用频率：位置环、速度环和电流环按需调用。运行内容：统一完成比例积分运算、积分限幅与抗积分饱和处理。 */
static float program_run_pi_f32(float ref,
                                float feedback,
                                float kp,
                                float ki,
                                float dt_s,
                                float *integral,
                                float out_min,
                                float out_max)
{
    float error;
    float p_out;
    float i_candidate;
    float out;

    if ((integral == 0) ||
        (!isfinite(ref)) ||
        (!isfinite(feedback)) ||
        (!isfinite(kp)) ||
        (!isfinite(ki)) ||
        (!isfinite(dt_s)) ||
        (!isfinite(*integral)) ||
        (!isfinite(out_min)) ||
        (!isfinite(out_max)) ||
        (out_max < out_min)) {
        if (integral != 0) {
            *integral = 0.0f;
        }
        return 0.0f;
    }

    error = ref - feedback;
    p_out = kp * error;
    i_candidate = *integral + (ki * dt_s * error);
    i_candidate = program_clamp_f32(i_candidate, out_min, out_max);
    out = p_out + i_candidate;

    if (out > out_max) {
        out = out_max;
        if (error < 0.0f) {
            *integral = i_candidate;
        }
    } else if (out < out_min) {
        out = out_min;
        if (error > 0.0f) {
            *integral = i_candidate;
        }
    } else {
        *integral = i_candidate;
    }

    return out;
}

/* 函数作用：清洗并限幅电流给定。
 * 输入：ref_cmd、limit_abs_a。输出：返回合法的电流给定，单位 A。调用频率：更新电流参考斜坡前调用。运行内容：过滤非法数值并把 d/q 轴电流命令限制到允许范围。 */
static float program_sanitize_current_ref_cmd(float ref_cmd, float limit_abs_a)
{
    if ((!isfinite(ref_cmd)) || (!isfinite(limit_abs_a)) || (limit_abs_a <= 0.0f)) {
        return 0.0f;
    }

    return program_clamp_f32(ref_cmd, -limit_abs_a, limit_abs_a);
}

/* 函数作用：对目标量施加单步斜率限制。
 * 输入：target、state、max_step。输出：返回限速后的新状态值。调用频率：电流和速度参考平滑更新时调用。运行内容：限制每次控制更新允许的最大变化量。 */
static float program_apply_slew_limit_f32(float target, float state, float max_step)
{
    float delta;

    if ((!isfinite(target)) || (!isfinite(state)) || (!isfinite(max_step)) || (max_step <= 0.0f)) {
        return 0.0f;
    }

    delta = target - state;
    delta = program_clamp_f32(delta, -max_step, max_step);

    return state + delta;
}

/* 函数作用：根据当前母线电压计算电压矢量幅值上限。
 * 输入：无。输出：返回允许的最大 dq 电压幅值，单位 V。调用频率：速度环和电流环更新时调用。运行内容：按 SVPWM 线性调制区比例从母线电压估算可用电压上限。 */
static float program_get_voltage_limit_v(void)
{
    float vbus;

    vbus = g_program_telemetry.vbus;
    return PROGRAM_VOLTAGE_LIMIT_RATIO * ((vbus > 1.0f) ? vbus : 1.0f);
}

/* 函数作用：限制 dq 电压矢量长度。
 * 输入：ud_ref、uq_ref、v_limit。输出：无返回值，结果回写到输入指针。调用频率：电流环和电压模式更新时调用。运行内容：在保持方向不变的前提下把矢量缩放到允许半径内。 */
static void program_limit_voltage_vector(float *ud_ref, float *uq_ref, float v_limit)
{
    float v_mag;
    float v_scale;

    if ((ud_ref == 0) || (uq_ref == 0)) {
        return;
    }

    if ((!isfinite(*ud_ref)) || (!isfinite(*uq_ref)) || (!isfinite(v_limit)) || (v_limit <= 0.0f)) {
        *ud_ref = 0.0f;
        *uq_ref = 0.0f;
        return;
    }

    v_mag = sqrtf((*ud_ref * *ud_ref) + (*uq_ref * *uq_ref));
    if ((v_mag > v_limit) && (v_mag > 0.0f)) {
        v_scale = v_limit / v_mag;
        *ud_ref *= v_scale;
        *uq_ref *= v_scale;
    }
}

/* 函数作用：平滑更新实际生效的 d/q 轴电流给定。
 * 输入：无。输出：无返回值。调用频率：快环进入电流环前调用。运行内容：先限幅目标值，再按斜率限制更新实际应用到 PI 的电流参考。 */
static void program_update_applied_current_references(void)
{
    float current_limit_a;
    float current_step_a;
    float id_target_a;
    float iq_target_a;

    current_limit_a = g_motor.iq_limit;
    current_step_a = PROGRAM_CURRENT_REF_RAMP_A_PER_S * PROGRAM_FAST_LOOP_DT_S;
    id_target_a = program_sanitize_current_ref_cmd(g_motor.id_ref, current_limit_a);
    iq_target_a = program_sanitize_current_ref_cmd(g_motor.iq_ref, current_limit_a);

    g_id_ref_applied_a = program_apply_slew_limit_f32(id_target_a, g_id_ref_applied_a, current_step_a);
    g_iq_ref_applied_a = program_apply_slew_limit_f32(iq_target_a, g_iq_ref_applied_a, current_step_a);
}

/* 函数作用：对连续机械角观测量做周期性重归一化。
 * 输入：无。输出：无返回值。调用频率：每次更新编码器速度窗口时按需调用。运行内容：在不破坏相对角度和速度计算的前提下防止累计角无限增大。 */
static void program_renormalize_encoder_observer(void)
{
    float anchor_turns;
    float anchor_rad;

    if ((!isfinite(g_encoder_continuous_mech_rad)) ||
        (!isfinite(g_encoder_speed_window_start_mech_rad)) ||
        (!isfinite(g_encoder_speed_raw_mech_rad_s))) {
        program_reset_encoder_observer();
        return;
    }

    if ((fabsf(g_encoder_continuous_mech_rad) < PROGRAM_ENCODER_OBSERVER_RENORM_RAD) &&
        (fabsf(g_encoder_speed_window_start_mech_rad) < PROGRAM_ENCODER_OBSERVER_RENORM_RAD)) {
        return;
    }

    anchor_turns = floorf(g_encoder_continuous_mech_rad / MOTOR_TWO_PI);
    anchor_rad = anchor_turns * MOTOR_TWO_PI;
    g_encoder_continuous_mech_rad -= anchor_rad;
    g_encoder_speed_window_start_mech_rad -= anchor_rad;
}

/* 函数作用：复位编码器对齐过程的运行时累积量。
 * 输入：无。输出：无返回值。调用频率：开始重新对齐或完成对齐后调用。运行内容：清空对齐计数器以及 sin/cos 平均所需的累积和。 */
static void program_reset_encoder_align_runtime(void)
{
    g_encoder_align_counter = 0U;
    g_encoder_align_sum_sin = 0.0f;
    g_encoder_align_sum_cos = 0.0f;
    g_encoder_align_sample_count = 0U;
}

/* 函数作用：在对齐保持阶段采集编码器电角度样本。
 * 输入：无。输出：无返回值。调用频率：零位对齐保持期间的每个快环调用。运行内容：只在采样窗口内累积电角度的 sin/cos，用于后续求平均角。 */
static void program_capture_encoder_alignment_sample(void)
{
    uint32_t sample_window_start_tick;
    float raw_theta_elec;

    sample_window_start_tick = 0U;
    if (PROGRAM_ALIGN_HOLD_TICKS > PROGRAM_ALIGN_SAMPLE_WINDOW_TICKS) {
        sample_window_start_tick = PROGRAM_ALIGN_HOLD_TICKS - PROGRAM_ALIGN_SAMPLE_WINDOW_TICKS;
    }

    if (g_encoder_align_counter < sample_window_start_tick) {
        return;
    }

    raw_theta_elec = program_get_encoder_raw_elec_angle_rad();
    g_encoder_align_sum_sin += sinf(raw_theta_elec);
    g_encoder_align_sum_cos += cosf(raw_theta_elec);
    g_encoder_align_sample_count++;
}

/* 函数作用：计算编码器零位对齐得到的平均电角度。
 * 输入：无。输出：返回对齐采样得到的电角度，单位 rad。调用频率：对齐结束时调用。运行内容：基于累积的 sin/cos 求平均角，降低编码器抖动影响。 */
static float program_get_encoder_alignment_angle_rad(void)
{
    float raw_theta_elec;

    if (g_encoder_align_sample_count == 0U) {
        return program_get_encoder_raw_elec_angle_rad();
    }

    raw_theta_elec = atan2f(g_encoder_align_sum_sin, g_encoder_align_sum_cos);
    return motor_params_wrap_angle_rad(raw_theta_elec);
}

/* 函数作用：初始化 DWT 周期计数器。
 * 输入：无。输出：无返回值。调用频率：系统初始化时调用一次。运行内容：打开 Cortex-M DWT 计数器，为快环耗时和超时监测提供时基。 */
static void program_init_cycle_counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_dwt_cycle_counter_ready = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? 1U : 0U;

    if (SystemCoreClock == 0U) {
        g_fast_loop_period_cycles = 1U;
    } else {
        g_fast_loop_period_cycles = (uint32_t)((float)SystemCoreClock / PROGRAM_FAST_LOOP_HZ);
        if (g_fast_loop_period_cycles == 0U) {
            g_fast_loop_period_cycles = 1U;
        }
    }
}

/* 函数作用：更新快环执行耗时统计。
 * 输入：elapsed_cycles 为本次快环消耗的 CPU 周期数。输出：无返回值。调用频率：每次 10 kHz 控制中断结束时调用。运行内容：刷新当前值、最大值和 overrun 计数，供调试和性能评估使用。 */
static void program_update_fast_loop_timing(uint32_t elapsed_cycles)
{
    float cycles_to_us;
    float elapsed_us;

    g_program_telemetry.fast_loop_cycles = elapsed_cycles;
    if (elapsed_cycles > g_program_telemetry.fast_loop_cycles_max) {
        g_program_telemetry.fast_loop_cycles_max = elapsed_cycles;
    }

    if (SystemCoreClock == 0U) {
        elapsed_us = 0.0f;
    } else {
        cycles_to_us = 1000000.0f / (float)SystemCoreClock;
        elapsed_us = (float)elapsed_cycles * cycles_to_us;
    }

    g_program_telemetry.fast_loop_time_us = elapsed_us;
    if (elapsed_us > g_program_telemetry.fast_loop_time_max_us) {
        g_program_telemetry.fast_loop_time_max_us = elapsed_us;
    }

    g_program_telemetry.fast_loop_period_us = PROGRAM_FAST_LOOP_PERIOD_US;
    if (elapsed_cycles > g_fast_loop_period_cycles) {
        g_program_telemetry.fast_loop_overrun = 1U;
        g_program_telemetry.fast_loop_overrun_count++;
    } else {
        g_program_telemetry.fast_loop_overrun = 0U;
    }
}

/* 函数作用：读取驱动器故障输入脚状态。
 * 输入：无。输出：返回 1 表示驱动故障有效，0 表示正常。调用频率：后台任务和快环保护判断中按需调用。运行内容：统一封装 MP6539 的 nFAULT 电平判定。 */
static uint8_t program_is_driver_fault_active(void)
{
    return (HAL_GPIO_ReadPin(N_FAULT_GPIO_Port, N_FAULT_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

/* 函数作用：刷新上位机与调试器使用的遥测快照。
 * 输入：无。输出：无返回值。调用频率：控制状态变化、采样更新和快环末尾按需调用。运行内容：把当前控制量、测量量和状态标志整理到统一遥测结构中。 */
static void program_update_debug_telemetry(void)
{
    float speed_ref_mech_rad_s_for_telemetry;

    speed_ref_mech_rad_s_for_telemetry = g_motor.speed_ref_mech_rad_s;
    if (g_motor.position_loop_enable == 0U) {
        speed_ref_mech_rad_s_for_telemetry = program_rpm_to_rad_s(g_motor.speed_ref_mech_rpm);
    }

    g_program_telemetry.pwm_enable_cmd = g_motor.run_request;
    g_program_telemetry.power_stage_enabled = g_power_stage_enabled;
    g_program_telemetry.control_state = (uint8_t)g_motor.state;
    g_program_telemetry.encoder_align_done = g_encoder_align_done;
    g_program_telemetry.ma600a_sample_counter = g_ma600a.sample_counter;
    g_program_telemetry.ma600a_reject_count = g_ma600a.reject_count;
    g_program_telemetry.ma600a_comm_error_count = g_ma600a.comm_error_count;
    g_program_telemetry.speed_observer_window_samples = PROGRAM_SPEED_OBSERVER_WINDOW_SAMPLES;
    g_program_telemetry.current_loop_enable = g_motor.current_loop_enable;
    g_program_telemetry.position_loop_enable = g_motor.position_loop_enable;
    g_program_telemetry.control_angle_open_loop_enable = g_motor.control_angle_open_loop_enable;
    g_program_telemetry.theta_open_loop = g_motor.theta_open_loop;
    g_program_telemetry.control_angle_open_loop_speed_elec = g_motor.control_angle_open_loop_speed_elec;
    g_program_telemetry.id_ref_cmd = g_motor.id_ref;
    g_program_telemetry.iq_ref_cmd = g_motor.iq_ref;
    g_program_telemetry.id_ref_applied_cmd = g_id_ref_applied_a;
    g_program_telemetry.iq_ref_applied_cmd = g_iq_ref_applied_a;
    g_program_telemetry.ud_ref_cmd = g_motor.ud_ref;
    g_program_telemetry.uq_ref_cmd = g_motor.uq_ref;
    g_program_telemetry.open_loop_speed_elec = g_motor.open_loop_speed_elec;
    g_program_telemetry.speed_loop_ready = g_encoder_speed_ready;
    g_program_telemetry.speed_ref_mech_rad_s = speed_ref_mech_rad_s_for_telemetry;
    g_program_telemetry.speed_ref_mech_applied_rad_s = g_motor.speed_ref_mech_applied_rad_s;
    g_program_telemetry.speed_meas_raw_mech_rad_s = g_encoder_speed_raw_mech_rad_s;
    g_program_telemetry.speed_meas_mech_rad_s = g_motor.speed_meas_mech_rad_s;
    g_program_telemetry.speed_ref_mech_rpm = program_rad_s_to_rpm(speed_ref_mech_rad_s_for_telemetry);
    g_program_telemetry.speed_ref_mech_applied_rpm = program_rad_s_to_rpm(g_motor.speed_ref_mech_applied_rad_s);
    g_program_telemetry.speed_meas_raw_mech_rpm = program_rad_s_to_rpm(g_encoder_speed_raw_mech_rad_s);
    g_program_telemetry.speed_meas_mech_rpm = program_rad_s_to_rpm(g_motor.speed_meas_mech_rad_s);
    g_program_telemetry.speed_error_mech_rad_s =
        g_motor.speed_ref_mech_applied_rad_s - g_motor.speed_meas_mech_rad_s;
    g_program_telemetry.position_ref_mech_rad = g_motor.position_ref_mech_rad;
    g_program_telemetry.position_meas_mech_rad = g_motor.position_meas_mech_rad;
    g_program_telemetry.position_error_mech_rad = g_motor.position_error_mech_rad;
    g_program_telemetry.position_ref_mech_deg = g_motor.position_ref_mech_deg;
    g_program_telemetry.position_meas_mech_deg = g_motor.position_meas_mech_deg;
    g_program_telemetry.position_error_mech_deg = g_motor.position_error_mech_deg;
    g_program_telemetry.position_kd = g_motor.position_kd;
    g_program_telemetry.speed_loop_dt_s = g_speed_loop_dt_s;
    g_program_telemetry.speed_meas_lpf_cutoff_hz = g_motor.speed_meas_lpf_cutoff_hz;
    g_program_telemetry.speed_ref_elec_rad_s = g_motor.speed_ref_elec_rad_s;
    g_program_telemetry.speed_meas_elec_rad_s = g_motor.speed_meas_elec_rad_s;
    g_program_telemetry.uq_limit_v = g_motor.voltage_limit;
    g_program_telemetry.iq_limit_a = g_motor.iq_limit;
    g_program_telemetry.voltage_limit_v = g_motor.voltage_limit;
    g_program_telemetry.encoder_elec_offset_rad = g_encoder_elec_offset_rad;
    g_program_telemetry.duty_a = g_foc.duty.duty_a;
    g_program_telemetry.duty_b = g_foc.duty.duty_b;
    g_program_telemetry.duty_c = g_foc.duty.duty_c;
    g_program_telemetry.fast_loop_period_us = PROGRAM_FAST_LOOP_PERIOD_US;
}

/* 函数作用：把 SVPWM 占空比写入 TIM1 比较寄存器。
 * 输入：duty 为三相占空比对象。输出：无返回值。调用频率：每次更新 PWM 输出前调用。运行内容：把 0~1 占空比换算为 TIM1 CCR 计数并做边界保护。 */
static void program_apply_svpwm_to_tim1(const foc_svpwm_duty_t *duty)
{
    uint32_t period_counts;
    uint32_t ccr_a;
    uint32_t ccr_b;
    uint32_t ccr_c;

    if (duty == 0) {
        return;
    }

    period_counts = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;

    ccr_a = (uint32_t)(program_clamp_f32(duty->duty_a, 0.0f, 1.0f) * (float)period_counts);
    ccr_b = (uint32_t)(program_clamp_f32(duty->duty_b, 0.0f, 1.0f) * (float)period_counts);
    ccr_c = (uint32_t)(program_clamp_f32(duty->duty_c, 0.0f, 1.0f) * (float)period_counts);

    if (ccr_a >= period_counts) {
        ccr_a = period_counts - 1U;
    }
    if (ccr_b >= period_counts) {
        ccr_b = period_counts - 1U;
    }
    if (ccr_c >= period_counts) {
        ccr_c = period_counts - 1U;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_c);
}

/* 函数作用：把三相 PWM 恢复到中心占空比。
 * 输入：无。输出：无返回值。调用频率：PWM 初启或安全停机时调用。运行内容：统一下发 50% 占空比，避免上桥臂和下桥臂出现残余输出。 */
static void program_apply_center_duty(void)
{
    foc_svpwm_duty_t center_duty;

    center_duty.duty_a = 0.5f;
    center_duty.duty_b = 0.5f;
    center_duty.duty_c = 0.5f;
    program_apply_svpwm_to_tim1(&center_duty);
}

/* 函数作用：启动 TIM1 三相互补 PWM 输出。
 * 输入：无。输出：无返回值。调用频率：系统初始化时调用一次。运行内容：先下发中心占空比，再依次启动主路和互补路 PWM 通道。 */
static void program_start_tim1_pwm_outputs(void)
{
    if (g_tim1_pwm_started != 0U) {
        return;
    }

    program_apply_center_duty();

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    g_tim1_pwm_started = 1U;
}

/* 函数作用：控制功率级休眠/使能引脚。
 * 输入：enable 非 0 表示使能功率级。输出：无返回值。调用频率：启停、保护和控制状态切换时调用。运行内容：统一维护 MP6539 使能引脚和遥测中的功率级状态。 */
static void program_set_power_stage_enable(uint8_t enable)
{
    uint8_t next_state;

    next_state = (enable != 0U) ? 1U : 0U;
    HAL_GPIO_WritePin(N_SLEEP_GPIO_Port, N_SLEEP_Pin, (next_state != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    g_power_stage_enabled = next_state;
    g_program_telemetry.power_stage_enabled = next_state;
}

/* 函数作用：按照原理图分流电阻和电流采样放大器参数，把原始 ADC 码值换算成相电流。
 * 输入：raw 为某一相当前 ADC 原始码值，offset_raw 为该相零电流偏置码值。
 * 输出：返回该相电流估计值，单位 A。
 * 调用频率：后台慢任务中约 1 kHz 调用，用于当前 ADC bring-up 阶段显示量。
 * 运行内容：先扣除零点偏置，再按 3.3V 参考、电流采样增益 20、采样电阻 10mR 计算电流。 */
static float program_convert_current_from_raw(uint16_t raw, uint16_t offset_raw)
{
    float volts_per_count;
    float sense_voltage;

    volts_per_count = PROGRAM_ADC_REF_V / PROGRAM_ADC_FULL_SCALE_COUNTS;
    sense_voltage = ((float)raw - (float)offset_raw) * volts_per_count;

    return sense_voltage / (PROGRAM_CURRENT_SENSE_GAIN * PROGRAM_SHUNT_RESISTOR_OHM);
}

/* 函数作用：按照原理图母线分压比，把原始 ADC 码值换算成母线电压。
 * 输入：raw 为 VBUS 通道 ADC 原始码值。
 * 输出：返回母线电压估计值，单位 V。
 * 调用频率：后台慢任务中约 1 kHz 调用。
 * 运行内容：使用 240k/10k 分压比和 3.3V ADC 参考把原始值换算回母线电压。 */
static float program_convert_vbus_from_raw(uint16_t raw)
{
    float volts_per_count;
    float adc_input_voltage;

    volts_per_count = PROGRAM_ADC_REF_V / PROGRAM_ADC_FULL_SCALE_COUNTS;
    adc_input_voltage = (float)raw * volts_per_count;

    return adc_input_voltage * ((PROGRAM_VBUS_R_UP_OHM + PROGRAM_VBUS_R_DOWN_OHM) / PROGRAM_VBUS_R_DOWN_OHM);
}

/* 函数作用：基于编码器角度更新机械速度测量。
 * 输入：无。输出：无返回值。调用频率：每次获得新的 MA600A 角度样本后调用。运行内容：维护连续机械角和测速窗口，输出原始与滤波后的机械/电角速度。 */
static void program_update_speed_measurement(void)
{
    float mech_angle_rad;
    float mech_delta_rad;
    float observer_dt_s;
    uint32_t observer_window_samples;
    uint32_t sample_delta_count;

    if (g_ma600a.data_valid == 0U) {
        return;
    }

    if (g_ma600a.sample_counter == g_encoder_last_sample_counter) {
        return;
    }

    mech_angle_rad = program_wrap_angle_0_2pi(program_get_encoder_rotor_mech_angle_rad());
    sample_delta_count = g_ma600a.sample_counter - g_encoder_last_sample_counter;
    if (sample_delta_count == 0U) {
        return;
    }

    g_encoder_last_sample_counter = g_ma600a.sample_counter;

    if (g_encoder_speed_primed == 0U) {
        g_encoder_prev_mech_angle_rad = mech_angle_rad;
        g_encoder_continuous_mech_rad = mech_angle_rad;
        g_motor.position_meas_mech_rad = program_get_encoder_output_mech_angle_rad();
        g_encoder_speed_window_start_mech_rad = mech_angle_rad;
        g_encoder_speed_window_sample_count = 0U;
        g_encoder_speed_raw_mech_rad_s = 0.0f;
        g_encoder_speed_primed = 1U;
        g_encoder_speed_ready = 0U;
        g_motor.speed_meas_mech_rad_s = 0.0f;
        g_motor.speed_meas_elec_rad_s = 0.0f;
        return;
    }

    mech_delta_rad = program_wrap_delta_pm_pi(mech_angle_rad - g_encoder_prev_mech_angle_rad);
    if (!isfinite(mech_delta_rad)) {
        program_reset_encoder_observer();
        return;
    }

    g_encoder_prev_mech_angle_rad = mech_angle_rad;
    g_encoder_continuous_mech_rad += mech_delta_rad;
    g_encoder_speed_window_sample_count += sample_delta_count;
    program_renormalize_encoder_observer();
    g_motor.position_meas_mech_rad = program_get_encoder_output_mech_angle_rad();
    g_motor.position_meas_mech_deg =
        program_wrap_angle_0_360_deg(program_rad_to_deg(g_motor.position_meas_mech_rad));

    if (g_encoder_speed_window_sample_count < PROGRAM_SPEED_OBSERVER_WINDOW_SAMPLES) {
        return;
    }

    observer_window_samples = g_encoder_speed_window_sample_count;
    observer_dt_s = PROGRAM_FAST_LOOP_DT_S * (float)observer_window_samples;
    if ((!isfinite(observer_dt_s)) || (observer_dt_s <= 0.0f)) {
        program_reset_encoder_observer();
        return;
    }

    g_encoder_speed_raw_mech_rad_s =
        (g_encoder_continuous_mech_rad - g_encoder_speed_window_start_mech_rad) / observer_dt_s;
    if (!isfinite(g_encoder_speed_raw_mech_rad_s)) {
        program_reset_encoder_observer();
        return;
    }

    g_encoder_speed_window_start_mech_rad = g_encoder_continuous_mech_rad;
    g_encoder_speed_window_sample_count = 0U;
    g_speed_loop_dt_s = observer_dt_s;
    g_speed_meas_lpf.alpha =
        program_lpf_alpha_from_cutoff_hz(g_motor.speed_meas_lpf_cutoff_hz, observer_dt_s);

    if (g_encoder_speed_ready == 0U) {
        filter_lpf_f32_init(&g_speed_meas_lpf,
                            g_speed_meas_lpf.alpha,
                            g_encoder_speed_raw_mech_rad_s);
        g_encoder_speed_ready = 1U;
    }

    g_motor.speed_meas_mech_rad_s =
        filter_lpf_f32_update(&g_speed_meas_lpf, g_encoder_speed_raw_mech_rad_s);
    if (!isfinite(g_motor.speed_meas_mech_rad_s)) {
        program_reset_encoder_observer();
        return;
    }

    g_motor.speed_meas_mech_rad_s =
        program_apply_speed_quantization_guard(g_motor.speed_meas_mech_rad_s, observer_window_samples);
    g_motor.speed_meas_elec_rad_s = g_motor.speed_meas_mech_rad_s * MOTOR_POLE_PAIRS;
    g_speed_loop_update_pending = 1U;
}

/* 函数作用：执行速度环并更新 iq 或 uq 命令。
 * 输入：无。输出：无返回值。调用频率：速度观测窗口完成后按需调用。运行内容：先按分频条件运行位置环，再由速度 PI 生成电流模式或电压模式的输出命令。 */
static void program_update_speed_loop(void)
{
    float output_cmd;
    float position_loop_dt_s;
    float speed_loop_dt_s;
    float speed_output_limit;
    float *speed_integral;

    speed_loop_dt_s = g_speed_loop_dt_s;
    if ((!isfinite(speed_loop_dt_s)) || (speed_loop_dt_s <= 0.0f)) {
        speed_loop_dt_s = PROGRAM_SPEED_LOOP_DT_S;
    }

    if (g_motor.position_loop_enable == 0U) {
        program_reset_position_loop();
    } else if (g_speed_loop_update_pending != 0U) {
        g_position_loop_elapsed_s += speed_loop_dt_s;
        if (g_position_loop_elapsed_s >= PROGRAM_POSITION_LOOP_DT_S) {
            position_loop_dt_s = g_position_loop_elapsed_s;
            g_position_loop_elapsed_s = 0.0f;
            program_update_position_loop(position_loop_dt_s);
        }
    }

    program_update_speed_reference_ramp();

    if (g_encoder_speed_ready == 0U) {
        g_motor.iq_ref = 0.0f;
        g_motor.uq_ref = 0.0f;
        g_speed_loop_update_pending = 0U;
        if (g_motor.position_loop_enable != 0U) {
            g_motor.speed_ref_mech_rad_s = 0.0f;
            program_reset_position_loop();
        }
        return;
    }

    if (g_speed_loop_update_pending == 0U) {
        return;
    }

    g_speed_loop_update_pending = 0U;

    if (g_motor.current_loop_enable != 0U) {
        speed_integral = &g_motor.speed_integral_iq;
        speed_output_limit = g_motor.iq_limit;
    } else {
        speed_integral = &g_motor.speed_integral_uq;
        speed_output_limit = program_get_voltage_limit_v();
        g_motor.voltage_limit = speed_output_limit;
    }

    output_cmd = program_run_pi_f32(g_motor.speed_ref_elec_rad_s,
                                    g_motor.speed_meas_elec_rad_s,
                                    g_motor.speed_kp,
                                    g_motor.speed_ki,
                                    speed_loop_dt_s,
                                    speed_integral,
                                    -speed_output_limit,
                                    speed_output_limit);

    if (g_motor.current_loop_enable != 0U) {
        g_motor.iq_ref = output_cmd;
    } else {
        g_motor.ud_ref = 0.0f;
        g_motor.uq_ref = output_cmd;
        g_motor.iq_ref = 0.0f;
    }
}

/* 函数作用：把三相原始 ADC 采样更新为 abc/dq 电流反馈。
 * 输入：ia_raw、ib_raw、ic_raw 为三相原始 ADC 码值，theta_elec 为当前反馈电角度。输出：无返回值。调用频率：每次 injected ADC 完成后调用。运行内容：完成零偏扣除、极性修正、低通滤波和 Clarke/Park 变换。 */
static void program_update_current_feedback_from_raw(uint16_t ia_raw,
                                                     uint16_t ib_raw,
                                                     uint16_t ic_raw,
                                                     float theta_elec)
{
    foc_alpha_beta_t i_ab;
    foc_dq_t i_dq;
    float ia_meas;
    float ib_meas;
    float ic_meas;
    float sin_theta;
    float cos_theta;

    if (g_program_telemetry.current_offset_ready == 0U) {
        g_program_telemetry.ia = 0.0f;
        g_program_telemetry.ib = 0.0f;
        g_program_telemetry.ic = 0.0f;
        g_program_telemetry.ic_meas = 0.0f;
        g_program_telemetry.i_abc_sum = 0.0f;
        g_program_telemetry.id = 0.0f;
        g_program_telemetry.iq = 0.0f;
        return;
    }

    /* Keep phase-current polarity explicit so it matches the board-level
     * current-sense definition: positive means current flows into the motor. */
    ia_meas = PROGRAM_CURRENT_SIGN_IA *
              program_convert_current_from_raw(ia_raw, g_program_telemetry.ia_offset_raw);
    ib_meas = PROGRAM_CURRENT_SIGN_IB *
              program_convert_current_from_raw(ib_raw, g_program_telemetry.ib_offset_raw);
    ic_meas = PROGRAM_CURRENT_SIGN_IC *
              program_convert_current_from_raw(ic_raw, g_program_telemetry.ic_offset_raw);

    /* Keep current feedback unfiltered in software so the current loop sees
     * the freshest ADC sample after offset removal. */
    g_program_telemetry.ia = ia_meas;
    g_program_telemetry.ib = ib_meas;
    g_program_telemetry.ic_meas = ic_meas;
    g_program_telemetry.i_abc_sum = ia_meas + ib_meas + ic_meas;
    g_program_telemetry.ic = -(ia_meas + ib_meas);

    if (g_power_stage_enabled == 0U) {
        g_program_telemetry.id = 0.0f;
        g_program_telemetry.iq = 0.0f;
        g_foc.i_ab.alpha = 0.0f;
        g_foc.i_ab.beta = 0.0f;
        g_foc.i_dq.d = 0.0f;
        g_foc.i_dq.q = 0.0f;
        return;
    }

    foc_core_clarke(g_program_telemetry.ia, g_program_telemetry.ib, &i_ab);
    sin_theta = sinf(theta_elec);
    cos_theta = cosf(theta_elec);
    foc_core_park(&i_ab, sin_theta, cos_theta, &i_dq);

    g_foc.i_ab = i_ab;
    g_foc.i_dq = i_dq;
    g_program_telemetry.id = i_dq.d;
    g_program_telemetry.iq = i_dq.q;
}

/* 函数作用：执行 d/q 轴电流环并更新 SVPWM 输出。
 * 输入：theta_cmd 为本次控制使用的电角度。输出：无返回值。调用频率：电流环模式下每个快环调用。运行内容：更新实际电流给定、运行 d/q 轴 PI、电压限幅并下发 FOC 电压命令。 */
static void program_run_current_loop(float theta_cmd)
{
    float vbus;
    float v_limit;

    program_update_applied_current_references();

    vbus = g_program_telemetry.vbus;
    v_limit = program_get_voltage_limit_v();
    g_motor.voltage_limit = v_limit;

    g_motor.ud_ref = program_run_pi_f32(g_id_ref_applied_a,
                                        g_program_telemetry.id,
                                        g_motor.current_kp,
                                        g_motor.current_ki,
                                        PROGRAM_FAST_LOOP_DT_S,
                                        &g_motor.id_integral_v,
                                        -v_limit,
                                        v_limit);

    g_motor.uq_ref = program_run_pi_f32(g_iq_ref_applied_a,
                                        g_program_telemetry.iq,
                                        g_motor.current_kp,
                                        g_motor.current_ki,
                                        PROGRAM_FAST_LOOP_DT_S,
                                        &g_motor.iq_integral_v,
                                        -v_limit,
                                        v_limit);

    program_limit_voltage_vector(&g_motor.ud_ref, &g_motor.uq_ref, v_limit);

    foc_core_run_voltage_open_loop(&g_foc, g_motor.ud_ref, g_motor.uq_ref, theta_cmd, vbus);
}

/* 函数作用：以电压模式直接运行 FOC。
 * 输入：theta_cmd 为本次控制使用的电角度。输出：无返回值。调用频率：关闭电流环时每个快环调用。运行内容：对 ud/uq 命令做矢量限幅后直接生成三相 PWM。 */
static void program_run_voltage_mode(float theta_cmd)
{
    float vbus;
    float v_limit;

    vbus = g_program_telemetry.vbus;
    v_limit = program_get_voltage_limit_v();
    g_motor.voltage_limit = v_limit;
    program_limit_voltage_vector(&g_motor.ud_ref, &g_motor.uq_ref, v_limit);
    foc_core_run_voltage_open_loop(&g_foc, g_motor.ud_ref, g_motor.uq_ref, theta_cmd, vbus);
}

/* 函数作用：快环主控制入口。
 * 输入：无。输出：无返回值。调用频率：每次 10 kHz injected ADC 中断完成后调用。运行内容：统一处理故障、使能、编码器对齐、速度环、电流环和 PWM 下发。 */
static void program_run_speed_current_control(void)
{
    float theta_cmd;
    float raw_theta_elec;
    uint8_t driver_fault_active;

    driver_fault_active = program_is_driver_fault_active();
    g_program_telemetry.driver_fault_active = driver_fault_active;

    if ((driver_fault_active != 0U) ||
        (g_program_telemetry.current_offset_ready == 0U) ||
        (g_program_telemetry.ma600a_angle_valid == 0U)) {
        program_set_power_stage_enable(0U);
        foc_core_reset_output(&g_foc);
        foc_core_set_electrical_angle(&g_foc, 0.0f);
        program_apply_svpwm_to_tim1(&g_foc.duty);
        program_reset_speed_loop();
        program_reset_position_loop();
        program_reset_current_loop();
        program_reset_speed_reference_ramp();
        program_reset_encoder_observer();
        program_reset_encoder_alignment();

        if (driver_fault_active != 0U) {
            g_motor.state = MOTOR_STATE_FAULT;
        } else {
            g_motor.state = MOTOR_STATE_READY;
        }

        g_motor.theta_open_loop = 0.0f;
        program_update_debug_telemetry();
        return;
    }

    program_handle_position_loop_mode_switch();
    program_handle_current_loop_mode_switch();

    if (g_motor.run_request == 0U) {
        /* With run_request cleared the align branch is never reached, so the
         * motor is left at zero vector and the power stage stays disabled. */
        program_set_power_stage_enable(0U);
        foc_core_reset_output(&g_foc);
        foc_core_set_electrical_angle(&g_foc, 0.0f);
        program_apply_svpwm_to_tim1(&g_foc.duty);
        program_reset_speed_loop();
        program_reset_position_loop();
        program_reset_current_loop();
        program_reset_speed_reference_ramp();
        /* Keep encoder observer running while PWM is disabled so telemetry still
         * reports mechanical motion when the motor is back-driven by hand. */
        program_reset_encoder_alignment();
        g_motor.state = MOTOR_STATE_READY;
        g_motor.theta_open_loop = 0.0f;
        program_update_debug_telemetry();
        return;
    }

    if (g_encoder_align_done == 0U) {
        g_motor.state = MOTOR_STATE_ALIGN;
        g_motor.align_done = 0U;
        program_reset_speed_loop();
        program_reset_position_loop();
        program_reset_current_loop();
        program_reset_speed_reference_ramp();
        program_reset_encoder_observer();
        g_motor.id_ref = 0.0f;
        g_motor.iq_ref = 0.0f;
        g_motor.theta_open_loop = 0.0f;
        g_motor.ud_ref = PROGRAM_ALIGN_UD_V;
        g_motor.uq_ref = 0.0f;
        g_motor.voltage_limit = PROGRAM_ALIGN_UD_V;

        foc_core_run_voltage_open_loop(&g_foc, g_motor.ud_ref, g_motor.uq_ref, g_motor.theta_open_loop, g_program_telemetry.vbus);
        program_apply_svpwm_to_tim1(&g_foc.duty);
        program_set_power_stage_enable(1U);

        g_encoder_align_counter++;
        program_capture_encoder_alignment_sample();
        if (g_encoder_align_counter >= PROGRAM_ALIGN_HOLD_TICKS) {
            raw_theta_elec = program_get_encoder_alignment_angle_rad();
            g_encoder_elec_offset_rad = motor_params_wrap_angle_rad(raw_theta_elec - g_motor.theta_open_loop);
            g_encoder_align_done = 1U;
            g_motor.align_done = 1U;
            program_reset_encoder_align_runtime();
            program_reset_speed_loop();
            program_reset_position_loop();
            program_reset_current_loop();
            program_reset_encoder_observer();
            g_program_telemetry.theta_elec = program_get_control_elec_angle_rad();
        }

        program_update_debug_telemetry();
        return;
    }

    if ((g_motor.speed_loop_enable != 0U) && (g_encoder_speed_ready == 0U)) {
        theta_cmd = program_get_control_elec_angle_rad();
        g_program_telemetry.theta_elec = theta_cmd;
        g_motor.theta_open_loop = theta_cmd;
        g_motor.state = MOTOR_STATE_READY;
        program_reset_speed_loop();
        program_reset_position_loop();
        program_reset_speed_reference_ramp();
        g_motor.id_ref = 0.0f;
        g_motor.iq_ref = 0.0f;
        g_motor.ud_ref = 0.0f;
        g_motor.uq_ref = 0.0f;
        if (g_motor.current_loop_enable != 0U) {
            program_run_current_loop(theta_cmd);
        } else {
            program_run_voltage_mode(theta_cmd);
        }
        program_apply_svpwm_to_tim1(&g_foc.duty);
        program_set_power_stage_enable(1U);
        program_update_debug_telemetry();
        return;
    }

    g_motor.id_ref = 0.0f;
    if (g_motor.speed_loop_enable != 0U) {
        program_update_speed_loop();
    } else {
        /* In manual mode the upper layer/debugger owns the command:
         * current mode uses iq_ref; voltage mode uses ud_ref/uq_ref directly. */
        g_motor.speed_integral_iq = 0.0f;
        g_motor.speed_integral_uq = 0.0f;
        program_reset_position_loop();
        g_speed_loop_update_pending = 0U;
        g_speed_loop_dt_s = PROGRAM_SPEED_LOOP_DT_S;
        program_reset_speed_reference_ramp();
    }

    g_motor.state = MOTOR_STATE_CLOSED_LOOP;
    theta_cmd = program_get_control_elec_angle_rad();
    g_program_telemetry.theta_elec = theta_cmd;
    g_motor.theta_open_loop = theta_cmd;
    if (g_motor.current_loop_enable != 0U) {
        program_run_current_loop(theta_cmd);
    } else {
        g_motor.iq_ref = 0.0f;
        program_run_voltage_mode(theta_cmd);
    }
    program_apply_svpwm_to_tim1(&g_foc.duty);
    program_set_power_stage_enable(1U);
    program_update_debug_telemetry();
}

/* 函数作用：初始化程序层遥测对象，给调试观察提供稳定初值。
 * 输入：无。
 * 输出：无返回值。
 * 调用频率：系统启动时只调用一次。
 * 运行内容：清零全部 ADC 与控制量缓存，并写入默认偏置和默认母线电压。 */
static void program_init_telemetry(void)
{
    (void)memset((void *)&g_program_telemetry, 0, sizeof(g_program_telemetry));
    g_program_telemetry.ia_offset_raw = PROGRAM_DEFAULT_CURRENT_OFFSET_RAW;
    g_program_telemetry.ib_offset_raw = PROGRAM_DEFAULT_CURRENT_OFFSET_RAW;
    g_program_telemetry.ic_offset_raw = PROGRAM_DEFAULT_CURRENT_OFFSET_RAW;
    g_program_telemetry.vbus = PROGRAM_DEFAULT_VBUS_V;
    g_program_telemetry.duty_a = 0.5f;
    g_program_telemetry.duty_b = 0.5f;
    g_program_telemetry.duty_c = 0.5f;
    g_program_telemetry.id_ref_applied_cmd = 0.0f;
    g_program_telemetry.iq_ref_applied_cmd = 0.0f;
}

/* 函数作用：启动 ADC2 DMA + TIM6 触发链，用于 VBUS/NTC 慢变量采样。
 * 输入：无。
 * 输出：无返回值；任一步失败则进入 Error_Handler()。
 * 调用频率：系统启动时只调用一次。
 * 运行内容：先启动 ADC2 DMA，再启动 TIM6 更新中断和 TRGO，让 ADC2 按 1 kHz 节拍采样两路慢变量。 */
static void program_start_adc2_dma_chain(void)
{
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)g_adc2_dma_buf, PROGRAM_ADC2_DMA_LENGTH) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        Error_Handler();
    }
}

/* 函数作用：启动 ADC1 注入组采样链，用于三相电流同步采样。
 * 输入：无。
 * 输出：无返回值；任一步失败则进入 Error_Handler()。
 * 调用频率：系统启动时只调用一次。
 * 运行内容：先使能 ADC1 注入组完成中断，再启动 TIM1 计数器，让 TIM1_TRGO2 周期性触发 IA/IB/IC 三通道采样。 */
static void program_start_adc1_injected_chain(void)
{
    program_start_tim1_pwm_outputs();

    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
        Error_Handler();
    }
}

/* 函数作用：把最新 ADC 原始数据换算到程序层遥测对象。
 * 输入：无。
 * 输出：无返回值；直接更新全局遥测对象中的 raw 和工程量。
 * 调用频率：后台慢任务中约 1 kHz 调用。
 * 运行内容：读取 ADC2 DMA 缓冲区、读取 ADC1 注入组最新原始值，完成零点扣除、电流换算和母线滤波。 */
static void program_update_measurements(void)
{
    uint16_t vbus_raw;
    uint16_t ntc_raw;

    vbus_raw = g_adc2_dma_buf[0];
    ntc_raw = g_adc2_dma_buf[1];

    g_program_telemetry.vbus_raw = vbus_raw;
    g_program_telemetry.ntc_raw = ntc_raw;

    g_program_telemetry.vbus = filter_lpf_f32_update(&g_vbus_lpf, program_convert_vbus_from_raw(vbus_raw));
    program_update_debug_telemetry();
}

/* 函数作用：读取 MA600A 机械角度并更新到程序层遥测对象。
 * 输入：无。
 * 输出：无返回值；成功时更新角度码值、角度制和弧度制，失败时仅清除有效标志。
 * 调用频率：后台慢任务中约 1 kHz 调用。
 * 运行内容：通过 SPI1 发起一次 MA600A 绝对角度读取，把结果写入统一调试变量，
 * 便于在 debug 和 VOFA 中直接观察转子角度变化。 */
static void program_update_encoder_measurements(void)
{
    if (g_ma600a.data_valid != 0U) {
        g_program_telemetry.ma600a_angle_raw = g_ma600a.angle_raw;
        g_program_telemetry.ma600a_angle_deg = g_ma600a.angle_deg;
        g_program_telemetry.ma600a_angle_rad = g_ma600a.angle_rad;
        g_program_telemetry.ma600a_angle_valid = g_ma600a.data_valid;
        g_program_telemetry.ma600a_consecutive_bad_count = g_ma600a.consecutive_bad_count;
        program_update_speed_measurement();
        if ((g_encoder_align_done != 0U) && (g_motor.control_angle_open_loop_enable != 0U)) {
            g_program_telemetry.theta_elec = motor_params_wrap_angle_rad(g_motor.theta_open_loop);
        } else if (g_encoder_align_done != 0U) {
            g_program_telemetry.theta_elec = program_get_encoder_aligned_elec_angle_rad();
        } else {
            g_program_telemetry.theta_elec = program_get_encoder_raw_elec_angle_rad();
        }
    } else {
        g_program_telemetry.ma600a_angle_valid = 0U;
        g_program_telemetry.ma600a_consecutive_bad_count = g_ma600a.consecutive_bad_count;
        program_reset_encoder_observer();
        program_reset_encoder_alignment();
    }
}

/* 函数作用：更新程序层故障标志位。
 * 输入：无。
 * 输出：无返回值；更新遥测中的 driver_fault_active 字段。
 * 调用频率：后台慢任务中约 1 kHz 调用。
 * 运行内容：当前阶段暂不接入 MP6539B 故障链，统一置 0，避免干扰 ADC bring-up。 */
static void program_update_fault_flags(void)
{
    g_program_telemetry.driver_fault_active = program_is_driver_fault_active();
}

/* 函数作用：按固定节拍把 IA 和 VBUS 发给 VOFA。
 * 输入：now_ms 为当前 1 ms 调度计数。
 * 输出：无返回值；当串口空闲时启动一次 DMA 发送。
 * 调用频率：后台慢任务中约 1 kHz 调用，内部限制为 20 ms 一次。
 * 运行内容：将 IA 电流和 VBUS 电压打包成两通道 CSV 文本，非阻塞送入 USART1 TX DMA。 */
static void program_send_wave_if_needed(uint32_t now_ms)
{
    float wave_buf[4];

    if ((now_ms - g_last_wave_tick_ms) < PROGRAM_WAVE_PERIOD_MS) {
        return;
    }

    g_last_wave_tick_ms = now_ms;
    wave_buf[0] = g_program_telemetry.speed_meas_mech_rpm;
    wave_buf[1] = g_program_telemetry.position_meas_mech_deg;
    wave_buf[2] = g_program_telemetry.position_meas_mech_rad;
    /* 当前发送协议改为 VOFA JustFloat 二进制浮点帧。 */
    (void)cli_uart_send_vofa(wave_buf, 4U);
}

/* 函数作用：程序层初始化入口。
 * 输入：无。
 * 输出：无返回值。
 * 调用频率：系统启动后只调用一次。
 * 运行内容：保持驱动休眠、执行 ADC 硬件校准、初始化滤波和对象、启动 ADC1/ADC2 采样链，并发送启动提示。 */
void program_init(void)
{
    program_set_power_stage_enable(0U);

    program_init_telemetry();

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

    foc_core_init(&g_foc); 
    foc_core_set_bus_voltage(&g_foc, PROGRAM_DEFAULT_VBUS_V);
    motor_state_init(&g_motor);
    g_motor.state = MOTOR_STATE_READY;
    g_motor.run_request = 0U;
    g_motor.speed_loop_enable = 1U;
    g_motor.current_loop_enable = 1U;
    g_motor.position_loop_enable = 0U;
    g_motor.control_angle_open_loop_enable = 0U;
    g_motor.theta_open_loop = 0.0f;
    g_motor.ud_ref = PROGRAM_OPEN_LOOP_DEFAULT_UD_V;
    g_motor.uq_ref = 0.0f;
    g_motor.speed_ref_mech_rpm = program_rad_s_to_rpm(PROGRAM_DEFAULT_SPEED_REF_MECH_RAD_S);
    g_motor.speed_ref_mech_rad_s = program_rpm_to_rad_s(g_motor.speed_ref_mech_rpm);
    g_motor.speed_ref_mech_applied_rad_s = 0.0f;
    g_motor.speed_meas_mech_rad_s = 0.0f;
    g_motor.speed_ref_elec_rad_s = 0.0f;
    g_motor.speed_meas_elec_rad_s = 0.0f;
    g_motor.control_angle_open_loop_speed_elec = PROGRAM_OPEN_LOOP_DEFAULT_SPEED_ELEC;
    g_motor.position_ref_mech_deg = 0.0f;
    g_motor.position_ref_mech_rad = 0.0f;
    g_motor.position_meas_mech_deg = 0.0f;
    g_motor.position_meas_mech_rad = 0.0f;
    g_motor.position_error_mech_deg = 0.0f;
    g_motor.position_error_mech_rad = 0.0f;
    g_motor.position_kp = PROGRAM_DEFAULT_POSITION_KP;
    g_motor.position_ki = PROGRAM_DEFAULT_POSITION_KI;
    g_motor.position_kd = PROGRAM_DEFAULT_POSITION_KD;
    g_motor.position_integral_speed = 0.0f;
    g_motor.position_speed_limit_mech_rad_s = PROGRAM_DEFAULT_POSITION_SPEED_LIMIT_MECH_RAD_S;
    g_motor.open_loop_speed_elec = 0.0f;
    g_motor.speed_kp = PROGRAM_DEFAULT_SPEED_KP;
    g_motor.speed_ki = PROGRAM_DEFAULT_SPEED_KI;
    g_motor.speed_meas_lpf_cutoff_hz = PROGRAM_DEFAULT_SPEED_MEAS_LPF_CUTOFF_HZ;
    g_motor.speed_integral_iq = 0.0f;
    g_motor.speed_integral_uq = 0.0f;
    g_motor.iq_limit = PROGRAM_DEFAULT_IQ_LIMIT_A;
    g_motor.current_kp = program_current_loop_kp_from_bandwidth_hz(PROGRAM_DEFAULT_CURRENT_LOOP_BANDWIDTH_HZ);
    g_motor.current_ki = program_current_loop_ki_from_bandwidth_hz(PROGRAM_DEFAULT_CURRENT_LOOP_BANDWIDTH_HZ);
    g_motor.id_integral_v = 0.0f;
    g_motor.iq_integral_v = 0.0f;
    g_motor.voltage_limit = 0.0f;
    g_position_loop_enable_prev = 0U;
    g_current_loop_enable_prev = 1U;
    program_reset_position_loop();
    program_reset_encoder_alignment();
    program_reset_encoder_observer();
    program_reset_encoder_align_runtime();
    foc_core_set_electrical_angle(&g_foc, 0.0f);
    filter_lpf_f32_init(&g_vbus_lpf, 0.1f, PROGRAM_DEFAULT_VBUS_V);
    cli_uart_init(&huart1);
    ma600a_init(&g_ma600a, &hspi1, ENC_CS_GPIO_Port, ENC_CS_Pin);
    program_init_cycle_counter();
    /* Fetch one valid angle sample before the first control interrupt. */
    (void)ma600a_read_angle(&g_ma600a);

    program_update_fault_flags();
    program_update_debug_telemetry();
    program_start_adc2_dma_chain();
    program_start_adc1_injected_chain();
    /* JustFloat 协议下不要混入文本启动信息，否则会干扰上位机解帧。 */
}

/* 函数作用：程序层后台任务入口。
 * 输入：无。
 * 输出：无返回值。
 * 调用频率：在 while(1) 中持续调用，内部按 TIM6 的 1 ms 节拍真正执行。
 * 运行内容：读取最新 ADC 结果、换算工程量、更新调试缓存，并按固定周期发出 IA 与 VBUS 到 VOFA。 */
void program_task(void)
{
    uint32_t now_ms;

    now_ms = g_tim6_tick_ms;
    if (now_ms == g_last_slow_task_tick_ms) {
        return;
    }

    g_last_slow_task_tick_ms = now_ms;

    program_update_measurements();
    program_update_fault_flags();
    program_send_wave_if_needed(now_ms);
}

/* 函数作用：程序层定时器周期回调转发入口。
 * 输入：htim 为触发中断的定时器句柄。
 * 输出：无返回值。
 * 调用频率：由 HAL 在定时器更新中断中调用；当前主要是 TIM6 1 kHz。
 * 运行内容：识别 TIM6 后递增慢任务时基，供后台任务和 VOFA 发送节拍共用。 */
void program_tim_period_elapsed_callback(TIM_HandleTypeDef *htim)
{
    if ((htim != 0) && (htim->Instance == TIM6)) {
        g_tim6_tick_ms++;
    }
}

/* 函数作用：程序层 regular ADC 完成回调转发入口。
 * 输入：hadc 为完成转换的 ADC 句柄。
 * 输出：无返回值。
 * 调用频率：当 regular ADC 完成转换后由 HAL 调用。
 * 运行内容：当前阶段 ADC2 通过 DMA 环形缓冲读取，因此这里保留空接口，后续需要时再挂逻辑。 */
void program_adc_conv_cplt_callback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
}

/* 函数作用：程序层 injected ADC 完成回调转发入口。
 * 输入：hadc 为完成注入组转换的 ADC 句柄。
 * 输出：无返回值。
 * 调用频率：每次 TIM1 触发 IA/IB/IC 三通道采样完成后调用一次。
 * 运行内容：读取三相最新原始 ADC 值，并在零电流校准阶段累计均值，得到每一相的偏置码值。 */
void program_adc_injected_conv_cplt_callback(ADC_HandleTypeDef *hadc)
{
    uint16_t ia_raw;
    uint16_t ib_raw;
    uint16_t ic_raw;
    uint32_t fast_loop_start_cycles;
    float theta_feedback;

    if ((hadc == 0) || (hadc->Instance != ADC1)) {
        return;
    }

    if (g_dwt_cycle_counter_ready != 0U) {
        fast_loop_start_cycles = DWT->CYCCNT;
    } else {
        fast_loop_start_cycles = 0U;
    }

    ia_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    ib_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    ic_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    g_program_telemetry.ia_raw = ia_raw;
    g_program_telemetry.ib_raw = ib_raw;
    g_program_telemetry.ic_raw = ic_raw;

    if (g_program_telemetry.current_offset_ready == 0U) {
        g_ia_offset_sum += ia_raw;
        g_ib_offset_sum += ib_raw;
        g_ic_offset_sum += ic_raw;

        g_program_telemetry.current_offset_sample_count++;
        if (g_program_telemetry.current_offset_sample_count >= PROGRAM_CURRENT_OFFSET_TARGET_SAMPLES) {
            g_program_telemetry.ia_offset_raw = (uint16_t)(g_ia_offset_sum / PROGRAM_CURRENT_OFFSET_TARGET_SAMPLES);
            g_program_telemetry.ib_offset_raw = (uint16_t)(g_ib_offset_sum / PROGRAM_CURRENT_OFFSET_TARGET_SAMPLES);
            g_program_telemetry.ic_offset_raw = (uint16_t)(g_ic_offset_sum / PROGRAM_CURRENT_OFFSET_TARGET_SAMPLES);
            g_program_telemetry.current_offset_ready = 1U;
        }
    }

    /* Run MA600A scheduling from the 10 kHz injected ADC cadence. */
    (void)ma600a_read_angle(&g_ma600a);
    program_update_encoder_measurements();
    program_update_control_angle_open_loop_state();

    if (g_encoder_align_done != 0U) {
        theta_feedback = program_get_control_elec_angle_rad();
    } else {
        theta_feedback = 0.0f;
    }

    program_update_current_feedback_from_raw(ia_raw, ib_raw, ic_raw, theta_feedback);
    program_run_speed_current_control();

    if (g_dwt_cycle_counter_ready != 0U) {
        program_update_fast_loop_timing(DWT->CYCCNT - fast_loop_start_cycles);
    }
}

/* 函数作用：HAL 定时器周期回调适配层。
 * 输入：htim 为 HAL 传入的定时器句柄。
 * 输出：无返回值。
 * 调用频率：由 HAL 在所有已使能的定时器中断里自动调用。
 * 运行内容：把 HAL 层回调统一转发到 program 层，便于后续集中管理。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    program_tim_period_elapsed_callback(htim);
}

/* 函数作用：HAL regular ADC 完成回调适配层。
 * 输入：hadc 为 HAL 传入的 ADC 句柄。
 * 输出：无返回值。
 * 调用频率：regular ADC 完成转换时由 HAL 自动调用。
 * 运行内容：把 regular ADC 回调统一转发到 program 层。 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    program_adc_conv_cplt_callback(hadc);
}

/* 函数作用：HAL injected ADC 完成回调适配层。
 * 输入：hadc 为 HAL 传入的 ADC 句柄。
 * 输出：无返回值。
 * 调用频率：每次 injected 组完成后由 HAL 自动调用。
 * 运行内容：把 injected ADC 回调统一转发到 program 层，用于三相电流采样和零点校准。 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    program_adc_injected_conv_cplt_callback(hadc);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    ma600a_spi_txrx_cplt_callback(&g_ma600a, hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    ma600a_spi_error_callback(&g_ma600a, hspi);
}

/* 函数作用：获取当前程序层遥测对象。
 * 输入：无。
 * 输出：返回程序层遥测对象常量指针。
 * 调用频率：调试器观察或上层只读访问时按需调用。
 * 运行内容：只返回当前遥测对象地址，不做额外计算。 */
/* Returns the current program telemetry snapshot. */
const volatile program_telemetry_t *program_get_telemetry(void)
{
    return &g_program_telemetry;
}

/* 函数作用：获取当前电机状态机对象。
 * 输入：无。
 * 输出：返回 motor_state_t 指针。
 * 调用频率：上层或调试按需调用。
 * 运行内容：返回内部状态机对象地址，便于后续闭环接入。 */
motor_state_t *program_get_motor(void)
{
    return &g_motor;
}

/* 函数作用：获取当前 FOC 核心对象。
 * 输入：无。
 * 输出：返回 foc_core_t 指针。
 * 调用频率：上层或调试按需调用。
 * 运行内容：返回内部 FOC 对象地址，便于后续接入电流环。 */
foc_core_t *program_get_foc(void)
{
    return &g_foc;
}
