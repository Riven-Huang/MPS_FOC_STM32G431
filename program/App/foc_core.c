#include "foc_core.h"

#include <math.h>

#define FOC_TWO_PI      6.28318530718f
#define FOC_SQRT3       1.73205080757f
#define FOC_INV_SQRT3   0.57735026919f

/* 函数作用：限制浮点量上下界。
 * 输入：value 为待限制值，min_value/max_value 为上下限。
 * 输出：返回限幅后的结果。
 * 调用频率：SVPWM 和角度处理时按需调用。
 * 运行内容：保证占空比等关键量始终在合法范围内。 */
static float foc_core_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/* 函数作用：把角度限制到 0~2pi。
 * 输入：theta 为任意浮点角度，单位 rad。
 * 输出：返回归一化后的角度。
 * 调用频率：每次更新电角度时调用。
 * 运行内容：通过加减 2pi 保持角度连续并落在一圈范围内。 */
static float foc_core_wrap_angle(float theta)
{
    float turns;

    if (!isfinite(theta)) {
        return 0.0f;
    }

    turns = floorf(theta / FOC_TWO_PI);
    theta -= turns * FOC_TWO_PI;

    if (theta < 0.0f) {
        theta += FOC_TWO_PI;
    } else if (theta >= FOC_TWO_PI) {
        theta -= FOC_TWO_PI;
    }

    return theta;
}

/* 函数作用：初始化 FOC 核心对象。
 * 输入：core 为待初始化的 FOC 对象。
 * 输出：无返回值。
 * 调用频率：系统启动时调用一次。
 * 运行内容：清零电流/电压变量，设置默认母线电压和三相 50% 占空比。 */
void foc_core_init(foc_core_t *core)
{
    if (core == 0) {
        return;
    }

    core->vbus = 24.0f;
    core->theta_elec = 0.0f;
    core->sin_theta = 0.0f;
    core->cos_theta = 1.0f;
    core->i_ab.alpha = 0.0f;
    core->i_ab.beta = 0.0f;
    core->i_dq.d = 0.0f;
    core->i_dq.q = 0.0f;
    core->v_dq_cmd.d = 0.0f;
    core->v_dq_cmd.q = 0.0f;
    core->v_ab_cmd.alpha = 0.0f;
    core->v_ab_cmd.beta = 0.0f;
    core->duty.duty_a = 0.5f;
    core->duty.duty_b = 0.5f;
    core->duty.duty_c = 0.5f;
}

/* 函数作用：把 FOC 输出恢复到安全中心态。
 * 输入：core 为 FOC 对象。
 * 输出：无返回值。
 * 调用频率：停机、待机、故障时按需调用。
 * 运行内容：清零 dq/ab 电压命令，并把三相占空比拉回 50%。 */
void foc_core_reset_output(foc_core_t *core)
{
    if (core == 0) {
        return;
    }

    core->v_dq_cmd.d = 0.0f;
    core->v_dq_cmd.q = 0.0f;
    core->v_ab_cmd.alpha = 0.0f;
    core->v_ab_cmd.beta = 0.0f;
    core->duty.duty_a = 0.5f;
    core->duty.duty_b = 0.5f;
    core->duty.duty_c = 0.5f;
}

/* 函数作用：设置当前母线电压。
 * 输入：core 为 FOC 对象，vbus 为母线电压，单位 V。
 * 输出：无返回值。
 * 调用频率：测量量更新后按节拍调用。
 * 运行内容：写入母线电压，并保证最小值不小于 1V。 */
void foc_core_set_bus_voltage(foc_core_t *core, float vbus)
{
    if (core == 0) {
        return;
    }

    core->vbus = vbus > 1.0f ? vbus : 1.0f;
}

/* 函数作用：更新电角度及其正余弦缓存。
 * 输入：core 为 FOC 对象，theta_elec 为电角度，单位 rad。
 * 输出：无返回值。
 * 调用频率：每次执行 FOC 计算时调用。
 * 运行内容：完成角度归一化，并同步计算 sin/cos 供 Park/反 Park 复用。 */
void foc_core_set_electrical_angle(foc_core_t *core, float theta_elec)
{
    if (core == 0) {
        return;
    }

    core->theta_elec = foc_core_wrap_angle(theta_elec);
    core->sin_theta = sinf(core->theta_elec);
    core->cos_theta = cosf(core->theta_elec);
}

/* 函数作用：执行 Clarke 变换。
 * 输入：ia/ib/ic 为三相电流，单位 A。
 * 输出：out 中得到 alpha-beta 坐标电流。
 * 调用频率：当前约 1 kHz，后续应放到电流快环。
 * 运行内容：把三相静止坐标量映射到两相 alpha-beta 坐标。 */
void foc_core_clarke(float ia, float ib, foc_alpha_beta_t *out)
{
    if (out == 0) {
        return;
    }

    /* Two-shunt Clarke: assume ia + ib + ic = 0, so alpha/beta only need ia and ib. */
    out->alpha = ia;
    out->beta = (ia + 2.0f * ib) * FOC_INV_SQRT3;
}

/* 函数作用：执行 Park 变换。
 * 输入：ab 为 alpha-beta 坐标量，sin_theta/cos_theta 为当前角度正余弦。
 * 输出：out 中得到 dq 坐标量。
 * 调用频率：当前约 1 kHz，后续应放到电流快环。
 * 运行内容：把静止坐标量变换到同步旋转坐标系。 */
void foc_core_park(const foc_alpha_beta_t *ab, float sin_theta, float cos_theta, foc_dq_t *out)
{
    if ((ab == 0) || (out == 0)) {
        return;
    }

    out->d = ab->alpha * cos_theta + ab->beta * sin_theta;
    out->q = -ab->alpha * sin_theta + ab->beta * cos_theta;
}

/* 函数作用：执行反 Park 变换。
 * 输入：dq 为同步坐标命令量，sin_theta/cos_theta 为当前角度正余弦。
 * 输出：out 中得到 alpha-beta 坐标量。
 * 调用频率：每次生成调制电压时调用。
 * 运行内容：把 dq 命令转回静止坐标系，供 SVPWM 使用。 */
void foc_core_inv_park(const foc_dq_t *dq, float sin_theta, float cos_theta, foc_alpha_beta_t *out)
{
    if ((dq == 0) || (out == 0)) {
        return;
    }

    out->alpha = dq->d * cos_theta - dq->q * sin_theta;
    out->beta = dq->d * sin_theta + dq->q * cos_theta;
}

/* 函数作用：根据 alpha-beta 电压生成三相 SVPWM 占空比。
 * 输入：core 为 FOC 对象，v_alpha/v_beta 为静止坐标电压，vbus 为母线电压。
 * 输出：结果写回 core->duty。
 * 调用频率：每次更新 PWM 前调用一次。
 * 运行内容：把电压投影到三相，注入零序偏置，并把结果限幅到 0~1。 */
void foc_core_svpwm(foc_core_t *core, float v_alpha, float v_beta, float vbus)
{
    float va_norm;
    float vb_norm;
    float vc_norm;
    float vmax;
    float vmin;
    float offset;

    if (core == 0) {
        return;
    }

    if (vbus < 1.0f) {
        core->duty.duty_a = 0.5f;
        core->duty.duty_b = 0.5f;
        core->duty.duty_c = 0.5f;
        return;
    }

    va_norm = v_alpha / vbus;
    vb_norm = (-0.5f * v_alpha + 0.5f * FOC_SQRT3 * v_beta) / vbus;
    vc_norm = (-0.5f * v_alpha - 0.5f * FOC_SQRT3 * v_beta) / vbus;

    vmax = va_norm;
    if (vb_norm > vmax) {
        vmax = vb_norm;
    }
    if (vc_norm > vmax) {
        vmax = vc_norm;
    }

    vmin = va_norm;
    if (vb_norm < vmin) {
        vmin = vb_norm;
    }
    if (vc_norm < vmin) {
        vmin = vc_norm;
    }

    offset = 0.5f - 0.5f * (vmax + vmin);

    core->duty.duty_a = foc_core_clamp(va_norm + offset, 0.0f, 1.0f);
    core->duty.duty_b = foc_core_clamp(vb_norm + offset, 0.0f, 1.0f);
    core->duty.duty_c = foc_core_clamp(vc_norm + offset, 0.0f, 1.0f);
}

/* 函数作用：执行一次开环电压模式 FOC 输出。
 * 输入：core 为 FOC 对象，ud/uq 为 dq 电压命令，theta_elec 为目标电角度，vbus 为母线电压。
 * 输出：更新 core 内的 dq/ab 电压命令以及三相占空比。
 * 调用频率：当前由状态机约 1 kHz 调用。
 * 运行内容：更新母线和角度，执行反 Park，再用 SVPWM 生成三相占空比。 */
void foc_core_run_voltage_open_loop(foc_core_t *core,
                                    float ud,
                                    float uq,
                                    float theta_elec,
                                    float vbus)
{
    if (core == 0) {
        return;
    }

    foc_core_set_bus_voltage(core, vbus);
    foc_core_set_electrical_angle(core, theta_elec);

    core->v_dq_cmd.d = ud;
    core->v_dq_cmd.q = uq;

    foc_core_inv_park(&core->v_dq_cmd, core->sin_theta, core->cos_theta, &core->v_ab_cmd);
    foc_core_svpwm(core, core->v_ab_cmd.alpha, core->v_ab_cmd.beta, core->vbus);
}
