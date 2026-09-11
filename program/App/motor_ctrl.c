#include "motor_ctrl.h"

#include <math.h>
#include <string.h>

#include "app_config.h"
#include "board.h"
#include "filter.h"
#include "ma600a.h"
#include "main.h"
#include "motor.h"
#include "units.h"

/* ============================================================================
 * motor_ctrl.c — 控制器实现
 *
 * 内存组织原则：
 *   - g_cmd / g_fb：对外接口（motor.h），用户写 g_cmd、读 g_fb
 *   - 本文件 static：环路内部状态（积分器、观测器、对齐累积量），一律不外露
 * ==========================================================================*/

volatile motor_cmd_t g_cmd;
volatile motor_fb_t  g_fb;
foc_core_t g_foc;

/* ---------------- 编码器与测速观测器 ---------------- */
static ma600a_t s_enc;
static uint32_t s_enc_last_sample;      /* 上次消费的编码器样本序号 */
static uint8_t  s_enc_primed;           /* 连续角已初始化 */
static uint8_t  s_spd_ready;            /* 测速滤波器已初始化 */
static uint8_t  s_spd_pending;          /* 有新速度估计待速度环消费 */
static float    s_enc_prev_rad;         /* 上一拍转子机械角（单圈） */
static float    s_enc_cont_rad;         /* 转子连续机械角（定期归一化） */
static float    s_spd_win_start_rad;    /* 测速窗口起点连续角 */
static uint32_t s_spd_win_cnt;          /* 测速窗口内样本计数 */
static float    s_spd_raw_rad_s;        /* 窗口差分原始速度 */
static float    s_spd_dt_s;             /* 速度环实际周期 */
static filter_lpf_f32_t s_spd_lpf;      /* 测速低通 */
static float    s_spd_lpf_hz_cache;     /* 测速 LPF 系数缓存对应的截止频率 */
static float    s_spd_dt_cache_s;       /* 测速 LPF 系数缓存对应的窗口周期 */

/* ---------------- 编码器零位对齐 ---------------- */
static uint8_t  s_align_done;
static uint32_t s_align_cnt;
static float    s_align_sum_sin;
static float    s_align_sum_cos;
static uint32_t s_align_samples;
static float    s_enc_ofs_rad;          /* 电角度零位偏置 */

/* ---------------- 环路内部状态 ---------------- */
static motor_state_id_t s_state;
static motor_mode_t     s_mode_active;
static float    s_theta_open;           /* 开环电压模式积分角度 */
static float    s_id_ref_tgt_a;         /* d 轴电流给定目标 */
static float    s_iq_ref_tgt_a;         /* q 轴电流给定目标 */
static float    s_id_ref_a;             /* 限幅后的 d 轴电流给定 */
static float    s_iq_ref_a;             /* 限幅后的 q 轴电流给定 */
static float    s_id_int_v;             /* d 轴 PI 积分（电压） */
static float    s_iq_int_v;             /* q 轴 PI 积分（电压） */
static float    s_spd_int_a;            /* 速度 PI 积分（电流） */
static float    s_spd_ref_tgt_rad_s;    /* 速度给定目标（转子 rad/s） */
static float    s_spd_ref_applied_rad_s;/* 速度斜坡后实际送入速度环的给定 */
static float    s_pos_int;              /* 位置 PI 积分 */
static float    s_pos_elapsed_s;        /* 位置环分频累计 */
static uint8_t  s_pos_hold;             /* 位置保持态 */
static uint8_t  s_pos_hold_rel_cnt;     /* 退出保持确认计数 */

/* ---------------- 电流零偏校准 ---------------- */
static uint32_t s_ofs_sum_a;
static uint32_t s_ofs_sum_b;
static uint32_t s_ofs_sum_c;
static uint16_t s_ofs_cnt;
static uint16_t s_ofs_a;
static uint16_t s_ofs_b;
static uint16_t s_ofs_c;

/* ---------------- 快环计时与慢任务分频 ---------------- */
static float    s_cycles_to_us;
static uint32_t s_loop_period_cycles;
static uint32_t s_slow_div;             /* 慢任务分频计数（VBUS/NTC/慢保护） */

/* ---------------- 母线滤波 ---------------- */
static filter_lpf_f32_t s_vbus_lpf;

/* ================================ 小工具 ================================ */

static float ctrl_clamp(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/* 单步斜率限制：仅用于速度给定，电流给定保持直接限幅。 */
static float ctrl_slew(float target, float state, float max_step)
{
    float delta;

    if ((!isfinite(target)) || (!isfinite(state)) || (!isfinite(max_step)) || (max_step <= 0.0f)) {
        return 0.0f;
    }

    delta = ctrl_clamp(target - state, -max_step, max_step);
    return state + delta;
}

/* 由截止频率和周期换算一阶低通系数 alpha */
static float ctrl_lpf_alpha(float cutoff_hz, float dt_s)
{
    float alpha;

    if ((!isfinite(cutoff_hz)) || (!isfinite(dt_s)) || (cutoff_hz <= 0.0f) || (dt_s <= 0.0f)) {
        return 1.0f;
    }

    alpha = 1.0f - expf(-UNITS_TWO_PI * cutoff_hz * dt_s);
    return ctrl_clamp(alpha, 0.0f, 1.0f);
}

/* 带抗积分饱和与有限值保护的浮点 PI */
static float ctrl_pi(float ref, float fb, float kp, float ki, float dt_s,
                     float *integ, float out_lo, float out_hi)
{
    float err;
    float p_out;
    float i_next;
    float out;

    if ((integ == 0) ||
        (!isfinite(ref)) || (!isfinite(fb)) || (!isfinite(kp)) || (!isfinite(ki)) ||
        (!isfinite(dt_s)) || (!isfinite(*integ)) ||
        (!isfinite(out_lo)) || (!isfinite(out_hi)) || (out_hi < out_lo)) {
        if (integ != 0) {
            *integ = 0.0f;
        }
        return 0.0f;
    }

    err = ref - fb;
    p_out = kp * err;
    i_next = ctrl_clamp(*integ + ki * dt_s * err, out_lo, out_hi);
    out = p_out + i_next;

    if (out > out_hi) {
        out = out_hi;
        if (err < 0.0f) {
            *integ = i_next;
        }
    } else if (out < out_lo) {
        out = out_lo;
        if (err > 0.0f) {
            *integ = i_next;
        }
    } else {
        *integ = i_next;
    }

    return out;
}

/* ================================ 复位组 ================================ */

static void ctrl_reset_cur_loop(void)
{
    s_id_ref_tgt_a = 0.0f;
    s_iq_ref_tgt_a = 0.0f;
    s_id_ref_a = 0.0f;
    s_iq_ref_a = 0.0f;
    s_id_int_v = 0.0f;
    s_iq_int_v = 0.0f;
    g_fb.id_ref_a = 0.0f;
    g_fb.iq_ref_a = 0.0f;
}

static void ctrl_reset_spd_loop(void)
{
    s_spd_int_a = 0.0f;
    s_spd_ref_tgt_rad_s = 0.0f;
    s_spd_ref_applied_rad_s = 0.0f;
    s_spd_pending = 0U;
    s_spd_dt_s = CFG_SPD_DT_S;
}

static void ctrl_reset_pos_loop(void)
{
    s_pos_int = 0.0f;
    s_pos_elapsed_s = 0.0f;
    s_pos_hold = 0U;
    s_pos_hold_rel_cnt = 0U;
}

static void ctrl_reset_loops(void)
{
    ctrl_reset_cur_loop();
    ctrl_reset_spd_loop();
    ctrl_reset_pos_loop();
}

static void ctrl_reset_align(void)
{
    s_align_done = 0U;
    s_align_cnt = 0U;
    s_align_sum_sin = 0.0f;
    s_align_sum_cos = 0.0f;
    s_align_samples = 0U;
    s_enc_ofs_rad = 0.0f;
    g_fb.align_done = 0U;
}

static void ctrl_reset_observer(void)
{
    s_enc_last_sample = 0U;
    s_enc_primed = 0U;
    s_spd_ready = 0U;
    s_spd_pending = 0U;
    s_enc_prev_rad = 0.0f;
    s_enc_cont_rad = 0.0f;
    s_spd_win_start_rad = 0.0f;
    s_spd_win_cnt = 0U;
    s_spd_raw_rad_s = 0.0f;
    s_spd_dt_s = CFG_SPD_DT_S;
    g_fb.spd_rot_rad_s = 0.0f;
    g_fb.pos_out_rad = 0.0f;
    /* alpha 保持缓存值，不在这里重算 expf（ALIGN 态每拍都会调本函数）；
     * 测速 alpha 由测速窗更新处按需刷新 */
    filter_lpf_f32_init(&s_spd_lpf, s_spd_lpf.alpha, 0.0f);
}

/* ================================ 角度换算 ================================ */

/* 编码器 → 转子机械角（含方向/减速比） */
static float ctrl_enc_rotor_rad(void)
{
    return units_enc_to_rotor_rad(s_enc.angle_rad);
}

/* 未经零位补偿的电角度 */
static float ctrl_enc_raw_elec_rad(void)
{
    return units_rotor_to_elec_rad(ctrl_enc_rotor_rad());
}

/* 对齐后的控制电角度 */
static float ctrl_theta_elec(void)
{
    return units_wrap_2pi(ctrl_enc_raw_elec_rad() - s_enc_ofs_rad);
}

/* 输出轴连续位置：观测器就绪用连续角，否则退回编码器即时角 */
static float ctrl_out_cont_rad(void)
{
    float rotor_rad = (s_enc_primed != 0U) ? s_enc_cont_rad : ctrl_enc_rotor_rad();

    if ((!isfinite(rotor_rad)) || (CFG_MOTOR_GEAR_RATIO <= 0.0f)) {
        return 0.0f;
    }
    return rotor_rad / CFG_MOTOR_GEAR_RATIO;
}

/* 本拍控制用电角度（全快环唯一计算点，O1：sin/cos 每拍只算一次）：
 * 开环角度电压模式用积分角，其余模式用对齐后的编码器角，未对齐为 0。
 * 反馈 Park 与输出反 Park 共用同一角度，保证 id/iq 观测与实际施加的电压矢量一致（O4）。 */
static float ctrl_theta_fast(void)
{
    if ((s_state == MOTOR_STATE_RUN) &&
        (s_mode_active == MOTOR_MODE_VOLTAGE) &&
        (g_cmd.volt_open_angle_en != 0U)) {
        s_theta_open = units_wrap_2pi(s_theta_open +
                                      g_cmd.open_spd_elec_rad_s * CFG_FAST_DT_S);
        return s_theta_open;
    }
    return (s_align_done != 0U) ? ctrl_theta_elec() : 0.0f;
}

/* 当前模式是否需要编码器（决定要不要先对齐、丢编码器要不要报故障） */
static uint8_t ctrl_mode_needs_encoder(motor_mode_t mode)
{
    if ((mode == MOTOR_MODE_CURRENT) ||
        (mode == MOTOR_MODE_SPEED) ||
        (mode == MOTOR_MODE_POSITION)) {
        return 1U;
    }
    if ((mode == MOTOR_MODE_VOLTAGE) && (g_cmd.volt_open_angle_en == 0U)) {
        return 1U;
    }
    return 0U;
}

/* ================================ 状态进出 ================================ */

static void ctrl_enter_idle(void)
{
    s_state = MOTOR_STATE_IDLE;
    s_mode_active = MOTOR_MODE_DISABLED;
    ctrl_reset_loops();
    ctrl_reset_align();         /* 每次重新 run 都重新对齐，与旧版行为一致 */
    s_theta_open = 0.0f;
    board_set_power_en(0U);
    foc_core_reset_output(&g_foc);
    foc_core_set_electrical_angle(&g_foc, 0.0f);
}

static void ctrl_enter_fault(void)
{
    s_state = MOTOR_STATE_FAULT;
    s_mode_active = MOTOR_MODE_DISABLED;
    ctrl_reset_loops();
    ctrl_reset_align();
    ctrl_reset_observer();
    s_theta_open = 0.0f;
    board_set_power_en(0U);
    foc_core_reset_output(&g_foc);
    foc_core_set_electrical_angle(&g_foc, 0.0f);
}

/* 进入位置模式时咬住当前位置，避免上电猛跳 */
static void ctrl_prime_position_ref(void)
{
    float pos_cont = ctrl_out_cont_rad();

    g_cmd.pos_deg = units_wrap_360(units_rad_to_deg(units_wrap_2pi(pos_cont)));
}

/* ================================ 编码器观测器 ================================ */

/* 连续角过大时归一化，防止 float 精度随运行时间恶化（不破坏相对角度与测速） */
static void ctrl_observer_renorm(void)
{
    float anchor_rad;

    if ((!isfinite(s_enc_cont_rad)) || (!isfinite(s_spd_win_start_rad)) ||
        (!isfinite(s_spd_raw_rad_s))) {
        ctrl_reset_observer();
        return;
    }

    if ((fabsf(s_enc_cont_rad) < CFG_ENC_RENORM_RAD) &&
        (fabsf(s_spd_win_start_rad) < CFG_ENC_RENORM_RAD)) {
        return;
    }

    anchor_rad = floorf(s_enc_cont_rad / UNITS_TWO_PI) * UNITS_TWO_PI;
    s_enc_cont_rad -= anchor_rad;
    s_spd_win_start_rad -= anchor_rad;
}

/* 编码器测速量化分辨率 */
static float ctrl_spd_quant_rad_s(uint32_t window_samples)
{
    if (window_samples == 0U) {
        return 0.0f;
    }
    return CFG_ENC_LSB_RAD / (CFG_FAST_DT_S * (float)window_samples);
}

/* 低速量化抖动抑制：给定和测量都在量化台阶附近时强制回零 */
static float ctrl_spd_quant_guard(float spd_rad_s, uint32_t window_samples)
{
    float threshold;

    if (!isfinite(spd_rad_s)) {
        return 0.0f;
    }

    threshold = CFG_SPD_ZERO_HOLD_SCALE * ctrl_spd_quant_rad_s(window_samples);
    if ((!isfinite(threshold)) || (threshold <= 0.0f)) {
        return spd_rad_s;
    }
    if (threshold < CFG_SPD_ZERO_HOLD_MIN_RAD_S) {
        threshold = CFG_SPD_ZERO_HOLD_MIN_RAD_S;
    }

    if ((fabsf(s_spd_ref_applied_rad_s) <= threshold) && (fabsf(spd_rad_s) <= threshold)) {
        return 0.0f;
    }

    return spd_rad_s;
}

/* 每个新编码器样本：更新连续角；每攒满一个窗口：更新一次速度 */
static void ctrl_observer_update(void)
{
    float ang_rad;
    float delta_rad;
    float dt_s;
    uint32_t delta_cnt;
    uint32_t window;

    if (s_enc.data_valid == 0U) {
        return;
    }
    if (s_enc.sample_counter == s_enc_last_sample) {
        return;
    }
    delta_cnt = s_enc.sample_counter - s_enc_last_sample;
    if (delta_cnt == 0U) {
        return;
    }
    s_enc_last_sample = s_enc.sample_counter;

    ang_rad = units_wrap_2pi(ctrl_enc_rotor_rad());

    if (s_enc_primed == 0U) {
        s_enc_prev_rad = ang_rad;
        s_enc_cont_rad = ang_rad;
        s_spd_win_start_rad = ang_rad;
        s_spd_win_cnt = 0U;
        s_spd_raw_rad_s = 0.0f;
        s_enc_primed = 1U;
        s_spd_ready = 0U;
        g_fb.pos_out_rad = ctrl_out_cont_rad();
        return;
    }

    delta_rad = units_wrap_pm_pi(ang_rad - s_enc_prev_rad);
    if (!isfinite(delta_rad)) {
        ctrl_reset_observer();
        return;
    }

    s_enc_prev_rad = ang_rad;
    s_enc_cont_rad += delta_rad;
    s_spd_win_cnt += delta_cnt;
    ctrl_observer_renorm();
    g_fb.pos_out_rad = ctrl_out_cont_rad();

    if (s_spd_win_cnt < CFG_SPD_WINDOW_SAMPLES) {
        return;
    }

    window = s_spd_win_cnt;
    dt_s = CFG_FAST_DT_S * (float)window;
    if ((!isfinite(dt_s)) || (dt_s <= 0.0f)) {
        ctrl_reset_observer();
        return;
    }

    s_spd_raw_rad_s = (s_enc_cont_rad - s_spd_win_start_rad) / dt_s;
    if (!isfinite(s_spd_raw_rad_s)) {
        ctrl_reset_observer();
        return;
    }

    s_spd_win_start_rad = s_enc_cont_rad;
    s_spd_win_cnt = 0U;
    s_spd_dt_s = dt_s;

    /* alpha 只在截止频率或窗口周期变化时重算（expf 开销大，不每窗都算） */
    if ((g_cmd.spd_lpf_hz != s_spd_lpf_hz_cache) || (dt_s != s_spd_dt_cache_s)) {
        s_spd_lpf.alpha = ctrl_lpf_alpha(g_cmd.spd_lpf_hz, dt_s);
        s_spd_lpf_hz_cache = g_cmd.spd_lpf_hz;
        s_spd_dt_cache_s = dt_s;
    }

    if (s_spd_ready == 0U) {
        filter_lpf_f32_init(&s_spd_lpf, s_spd_lpf.alpha, s_spd_raw_rad_s);
        s_spd_ready = 1U;
    }

    g_fb.spd_rot_rad_s = filter_lpf_f32_update(&s_spd_lpf, s_spd_raw_rad_s);
    if (!isfinite(g_fb.spd_rot_rad_s)) {
        ctrl_reset_observer();
        return;
    }

    g_fb.spd_rot_rad_s = ctrl_spd_quant_guard(g_fb.spd_rot_rad_s, window);
    s_spd_pending = 1U;
}

/* ================================ 电流反馈 ================================ */

/* 原始码 → 相电流 → Clarke/Park → id/iq。Park 复用 foc 已缓存的 sin/cos。 */
static void ctrl_current_feedback(uint16_t ia_raw, uint16_t ib_raw, uint16_t ic_raw)
{
    foc_alpha_beta_t i_ab;
    foc_dq_t i_dq;
    float ia;
    float ib;
    float ic;

    if (g_fb.offset_ready == 0U) {
        g_fb.ia_a = 0.0f;
        g_fb.ib_a = 0.0f;
        g_fb.ic_a = 0.0f;
        g_fb.i_sum_a = 0.0f;
        g_fb.id_a = 0.0f;
        g_fb.iq_a = 0.0f;
        return;
    }

    ia = CFG_CUR_SIGN_IA * board_current_a(ia_raw, s_ofs_a);
    ib = CFG_CUR_SIGN_IB * board_current_a(ib_raw, s_ofs_b);
    ic = CFG_CUR_SIGN_IC * board_current_a(ic_raw, s_ofs_c);

    g_fb.ia_a = ia;
    g_fb.ib_a = ib;
    g_fb.ic_a = ic;
    g_fb.i_sum_a = ia + ib + ic;

    if (board_power_enabled() == 0U) {
        g_fb.id_a = 0.0f;
        g_fb.iq_a = 0.0f;
        return;
    }

    foc_core_clarke(ia, ib, &i_ab);
    foc_core_park(&i_ab, g_foc.sin_theta, g_foc.cos_theta, &i_dq);
    g_fb.id_a = i_dq.d;
    g_fb.iq_a = i_dq.q;
}

/* ================================ 三环 ================================ */

static float ctrl_volt_lim_v(void)
{
    float vbus = g_fb.vbus_v;
    return CFG_VOLT_LIMIT_RATIO * ((vbus > 1.0f) ? vbus : 1.0f);
}

/* dq 电压矢量限幅（保持方向不变缩放到圆内） */
static void ctrl_volt_vector_lim(float *ud_v, float *uq_v, float v_lim)
{
    float mag;
    float scale;

    if ((ud_v == 0) || (uq_v == 0)) {
        return;
    }
    if ((!isfinite(*ud_v)) || (!isfinite(*uq_v)) || (!isfinite(v_lim)) || (v_lim <= 0.0f)) {
        *ud_v = 0.0f;
        *uq_v = 0.0f;
        return;
    }

    mag = sqrtf((*ud_v * *ud_v) + (*uq_v * *uq_v));
    if ((mag > v_lim) && (mag > 0.0f)) {
        scale = v_lim / mag;
        *ud_v *= scale;
        *uq_v *= scale;
    }
}

/* 位置环（200Hz 分频调用）：输出轴坐标系，输出转子侧速度给定 */
static void ctrl_pos_loop(float dt_s)
{
    float ref_rad;
    float meas_cont_raw;
    float meas_wrapped;
    float err_rad;
    float err_abs;
    float spd_lim;
    float spd_cmd_out;
    float spd_meas_out;
    float spd_meas_abs;

    if (s_enc_primed == 0U) {
        s_pos_int = 0.0f;
        s_spd_ref_tgt_rad_s = 0.0f;
        return;
    }

    if ((!isfinite(dt_s)) || (dt_s <= 0.0f)) {
        dt_s = CFG_POS_DT_S;
    }

    ref_rad = units_deg_to_rad(units_wrap_360(g_cmd.pos_deg));
    meas_cont_raw = ctrl_out_cont_rad();
    if ((!isfinite(ref_rad)) || (!isfinite(meas_cont_raw))) {
        ctrl_reset_pos_loop();
        s_spd_ref_tgt_rad_s = 0.0f;
        return;
    }

    meas_wrapped = units_wrap_2pi(meas_cont_raw);
    err_rad = units_wrap_pm_pi(ref_rad - meas_wrapped);
    if (!isfinite(err_rad)) {
        ctrl_reset_pos_loop();
        s_spd_ref_tgt_rad_s = 0.0f;
        return;
    }
    err_abs = fabsf(err_rad);

    spd_meas_out = units_rotor_spd_to_out(g_fb.spd_rot_rad_s);
    spd_meas_abs = fabsf(spd_meas_out);

    /* 保持态：误差和速度都足够小时冻结，抑制减速箱静摩擦引起的低频抖动。
     * g_cmd.pos_hold_en=0 时旁路（诊断用），并清掉可能已锁存的保持态 */
    if (g_cmd.pos_hold_en == 0U) {
        s_pos_hold = 0U;
        s_pos_hold_rel_cnt = 0U;
    } else {
        if ((err_abs <= CFG_POS_HOLD_ERR_RAD) && (spd_meas_abs <= CFG_POS_HOLD_SPD_RAD_S)) {
            s_pos_hold = 1U;
            s_pos_hold_rel_cnt = 0U;
            s_pos_int = 0.0f;
            s_spd_ref_tgt_rad_s = 0.0f;
            return;
        }
        if (s_pos_hold != 0U) {
            if (err_abs <= CFG_POS_HOLD_REL_ERR_RAD) {
                s_pos_hold_rel_cnt = 0U;
                s_pos_int = 0.0f;
                s_spd_ref_tgt_rad_s = 0.0f;
                return;
            }
            s_pos_hold_rel_cnt++;
            if (s_pos_hold_rel_cnt < CFG_POS_HOLD_REL_CYCLES) {
                s_pos_int = 0.0f;
                s_spd_ref_tgt_rad_s = 0.0f;
                return;
            }
            s_pos_hold = 0U;
            s_pos_hold_rel_cnt = 0U;
        }
    }

    spd_lim = g_cmd.pos_spd_lim_rad_s;
    if ((!isfinite(spd_lim)) || (spd_lim <= 0.0f)) {
        spd_lim = CFG_POS_SPD_LIM_RAD_S;
    }

    spd_cmd_out = ctrl_pi(err_rad, 0.0f, g_cmd.pos_kp, g_cmd.pos_ki, dt_s,
                          &s_pos_int, -spd_lim, spd_lim);
    spd_cmd_out -= g_cmd.pos_kd * spd_meas_out;
    spd_cmd_out = ctrl_clamp(spd_cmd_out, -spd_lim, spd_lim);

    /* 无 Ki 时指令可能小到推不动静摩擦：给最小爬行速度 */
    if ((g_cmd.pos_hold_en != 0U) &&
        (err_abs > CFG_POS_CREEP_ERR_RAD) &&
        (fabsf(spd_cmd_out) < CFG_POS_CREEP_SPD_RAD_S) &&
        (spd_meas_abs <= CFG_POS_HOLD_SPD_RAD_S)) {
        spd_cmd_out = copysignf(CFG_POS_CREEP_SPD_RAD_S, err_rad);
    }

    /* 输出轴速度命令 → 转子侧速度给定 */
    s_spd_ref_tgt_rad_s = spd_cmd_out * CFG_MOTOR_GEAR_RATIO;
}

/* 速度给定斜坡：限制速度命令变化率，电流给定不经过斜坡。 */
static void ctrl_spd_ref_ramp(void)
{
    if (s_mode_active != MOTOR_MODE_POSITION) {
        s_spd_ref_tgt_rad_s = units_rpm_to_rad_s(g_cmd.spd_rpm);
    }

    s_spd_ref_applied_rad_s = ctrl_slew(s_spd_ref_tgt_rad_s,
                                        s_spd_ref_applied_rad_s,
                                        CFG_SPD_RAMP_RAD_S2 * CFG_FAST_DT_S);
}

/* 速度环（有新速度估计才执行）：输出 iq 目标 */
static void ctrl_spd_loop(void)
{
    float ref_elec;
    float meas_elec;

    if (s_spd_pending == 0U) {
        return;
    }
    s_spd_pending = 0U;

    if (s_spd_ready == 0U) {
        s_iq_ref_tgt_a = 0.0f;
        return;
    }

    ref_elec = s_spd_ref_applied_rad_s * CFG_MOTOR_POLE_PAIRS;
    meas_elec = g_fb.spd_rot_rad_s * CFG_MOTOR_POLE_PAIRS;

    s_iq_ref_tgt_a = ctrl_pi(ref_elec, meas_elec, g_cmd.spd_kp, g_cmd.spd_ki,
                             s_spd_dt_s, &s_spd_int_a,
                             -g_cmd.iq_lim_a, g_cmd.iq_lim_a);
}

/* 电流环（10kHz）：限幅 → PI → 电压限幅 → FOC 输出。角度/sin/cos 已由本拍入口缓存 */
static void ctrl_cur_loop(void)
{
    float v_lim;
    float ud;
    float uq;

    v_lim = ctrl_volt_lim_v();
    g_fb.volt_lim_v = v_lim;

    s_id_ref_a = ctrl_clamp(s_id_ref_tgt_a, -g_cmd.iq_lim_a, g_cmd.iq_lim_a);
    s_iq_ref_a = ctrl_clamp(s_iq_ref_tgt_a, -g_cmd.iq_lim_a, g_cmd.iq_lim_a);
    g_fb.id_ref_a = s_id_ref_a;
    g_fb.iq_ref_a = s_iq_ref_a;

    ud = ctrl_pi(s_id_ref_a, g_fb.id_a, g_cmd.cur_kp, g_cmd.cur_ki,
                 CFG_FAST_DT_S, &s_id_int_v, -v_lim, v_lim);
    uq = ctrl_pi(s_iq_ref_a, g_fb.iq_a, g_cmd.cur_kp, g_cmd.cur_ki,
                 CFG_FAST_DT_S, &s_iq_int_v, -v_lim, v_lim);

    ctrl_volt_vector_lim(&ud, &uq, v_lim);

    g_fb.ud_v = ud;
    g_fb.uq_v = uq;
    foc_core_apply_voltage(&g_foc, ud, uq, g_fb.vbus_v);
}

/* ================================ 模式分发 ================================ */

static void ctrl_run_mode(void)
{
    float ud;
    float uq;
    float v_lim;
    float pos_dt;

    switch (s_mode_active) {
    case MOTOR_MODE_PWM_TEST:
        /* 固定占空比发波：不经过 FOC，直接写三相占空比 */
        board_set_power_en(1U);
        g_foc.duty.duty_a = ctrl_clamp(g_cmd.duty_a, 0.0f, 1.0f);
        g_foc.duty.duty_b = ctrl_clamp(g_cmd.duty_b, 0.0f, 1.0f);
        g_foc.duty.duty_c = ctrl_clamp(g_cmd.duty_c, 0.0f, 1.0f);
        break;

    case MOTOR_MODE_VOLTAGE:
        /* 角度已在快环入口统一计算（闭环编码器角或开环积分角），这里只下发电压 */
        board_set_power_en(1U);
        v_lim = ctrl_volt_lim_v();
        g_fb.volt_lim_v = v_lim;
        ud = g_cmd.ud_v;
        uq = g_cmd.uq_v;
        ctrl_volt_vector_lim(&ud, &uq, v_lim);
        g_fb.ud_v = ud;
        g_fb.uq_v = uq;
        foc_core_apply_voltage(&g_foc, ud, uq, g_fb.vbus_v);
        break;

    case MOTOR_MODE_CURRENT:
    case MOTOR_MODE_SPEED:
    case MOTOR_MODE_POSITION:
        board_set_power_en(1U);
        if (s_mode_active == MOTOR_MODE_CURRENT) {
            s_id_ref_tgt_a = g_cmd.id_a;
            s_iq_ref_tgt_a = g_cmd.iq_a;
        } else {
            s_id_ref_tgt_a = 0.0f;
            if ((s_mode_active == MOTOR_MODE_POSITION) && (s_spd_pending != 0U)) {
                s_pos_elapsed_s += s_spd_dt_s;
                if (s_pos_elapsed_s >= CFG_POS_DT_S) {
                    pos_dt = s_pos_elapsed_s;
                    s_pos_elapsed_s = 0.0f;
                    ctrl_pos_loop(pos_dt);
                }
            }
            ctrl_spd_ref_ramp();
            ctrl_spd_loop();
        }
        ctrl_cur_loop();
        break;

    default:
        ctrl_enter_idle();
        break;
    }
}

/* ================================ 状态机 ================================ */

static void ctrl_fsm_idle(void)
{
    motor_mode_t mode = g_cmd.mode;

    board_set_power_en(0U);
    foc_core_reset_output(&g_foc);

    if ((g_cmd.run == 0U) || (mode == MOTOR_MODE_DISABLED)) {
        return;
    }

    if (ctrl_mode_needs_encoder(mode) != 0U) {
        /* 闭环类模式：编码器与零偏就绪后才允许进对齐 */
        if ((s_enc.data_valid != 0U) && (g_fb.offset_ready != 0U)) {
            s_align_cnt = 0U;
            s_align_sum_sin = 0.0f;
            s_align_sum_cos = 0.0f;
            s_align_samples = 0U;
            s_state = MOTOR_STATE_ALIGN;
        }
    } else {
        /* PWM_TEST / 开环角度电压模式：直接进 RUN */
        ctrl_reset_loops();
        s_theta_open = 0.0f;
        s_mode_active = mode;
        s_state = MOTOR_STATE_RUN;
    }
}

static void ctrl_fsm_align(void)
{
    float raw_elec;

    /* 对齐期间允许反悔：撤销 run 或切成不需要编码器的模式则回 IDLE */
    if ((g_cmd.run == 0U) || (ctrl_mode_needs_encoder(g_cmd.mode) == 0U)) {
        ctrl_enter_idle();
        return;
    }

    /* 对齐期间持续复位观测器，对齐结束后从零位附近重新起步 */
    ctrl_reset_observer();

    board_set_power_en(1U);
    foc_core_apply_voltage(&g_foc, CFG_ALIGN_UD_V, 0.0f, g_fb.vbus_v);

    s_align_cnt++;
    if ((CFG_ALIGN_HOLD_TICKS > CFG_ALIGN_SAMPLE_TICKS) &&
        (s_align_cnt >= (CFG_ALIGN_HOLD_TICKS - CFG_ALIGN_SAMPLE_TICKS))) {
        raw_elec = ctrl_enc_raw_elec_rad();
        s_align_sum_sin += sinf(raw_elec);
        s_align_sum_cos += cosf(raw_elec);
        s_align_samples++;
    }

    if (s_align_cnt >= CFG_ALIGN_HOLD_TICKS) {
        if (s_align_samples > 0U) {
            raw_elec = atan2f(s_align_sum_sin, s_align_sum_cos);
        } else {
            raw_elec = ctrl_enc_raw_elec_rad();
        }
        /* 对齐目标电角度为 0：偏置 = 实测电角度 */
        s_enc_ofs_rad = units_wrap_2pi(raw_elec);
        s_align_done = 1U;
        g_fb.align_done = 1U;
        g_fb.align_ofs_rad = s_enc_ofs_rad;
        ctrl_reset_loops();
        s_mode_active = g_cmd.mode;
        s_state = MOTOR_STATE_RUN;

        if (s_mode_active == MOTOR_MODE_POSITION) {
            ctrl_prime_position_ref();
        }
    }
}

static void ctrl_fsm_run(void)
{
    motor_mode_t mode = g_cmd.mode;

    if ((g_cmd.run == 0U) || (mode == MOTOR_MODE_DISABLED)) {
        ctrl_enter_idle();
        return;
    }

    if (mode != s_mode_active) {
        /* 运行中切模式：清环路；新模式需要编码器但没对齐过则回对齐 */
        ctrl_reset_loops();
        if ((ctrl_mode_needs_encoder(mode) != 0U) && (s_align_done == 0U)) {
            s_align_cnt = 0U;
            s_align_sum_sin = 0.0f;
            s_align_sum_cos = 0.0f;
            s_align_samples = 0U;
            s_state = MOTOR_STATE_ALIGN;
            return;
        }
        if (mode == MOTOR_MODE_POSITION) {
            ctrl_prime_position_ref();
        }
        s_theta_open = 0.0f;
        s_mode_active = mode;
    }

    ctrl_run_mode();
}

/* ================================ 保护 ================================ */

/* 快环保护：当拍生效。返回新产生的故障位。 */
static uint16_t ctrl_check_fast_faults(void)
{
    uint16_t bits = 0U;

    if (board_drv_fault() != 0U) {
        bits |= FAULT_BIT_DRIVER;
    }

    if (g_fb.offset_ready != 0U) {
        if ((fabsf(g_fb.ia_a) > CFG_PROT_PHASE_OC_A) ||
            (fabsf(g_fb.ib_a) > CFG_PROT_PHASE_OC_A) ||
            (fabsf(g_fb.ic_a) > CFG_PROT_PHASE_OC_A)) {
            bits |= FAULT_BIT_OVERCURRENT;
        }
    }

    /* 闭环运行中丢编码器 */
    if (((s_state == MOTOR_STATE_ALIGN) ||
         ((s_state == MOTOR_STATE_RUN) && (ctrl_mode_needs_encoder(s_mode_active) != 0U))) &&
        (s_enc.data_valid == 0U)) {
        bits |= FAULT_BIT_ENCODER;
    }

    return bits;
}

/* 慢任务（1kHz，由快环按 CFG_SLOW_DIV 分频调用，与快环同一中断上下文）：
 * 母线/NTC 换算、过压欠压锁存、派生单位换算。
 * 与快环同上下文后，g_fb.fault_bits 的置位/清除不再有跨上下文竞态。 */
static void ctrl_slow_tasks(uint16_t vbus_raw, uint16_t ntc_raw)
{
    float vbus;

    g_fb.vbus_raw = vbus_raw;
    g_fb.ntc_raw = ntc_raw;

    vbus = filter_lpf_f32_update(&s_vbus_lpf, board_vbus_v(vbus_raw));
    if (!isfinite(vbus)) {
        vbus = 48.0f;
    }
    g_fb.vbus_v = vbus;
    g_fb.ntc_c = board_ntc_c(ntc_raw);

    /* 慢速保护：母线过压始终锁存；欠压只在非待机时锁存，
     * 避免只上逻辑电、不上母线电的调试场景卡在 FAULT */
    if (vbus > CFG_PROT_VBUS_OV_V) {
        g_fb.fault_bits |= FAULT_BIT_OVERVOLT;
    } else if ((vbus < CFG_PROT_VBUS_UV_V) && (s_state != MOTOR_STATE_IDLE)) {
        g_fb.fault_bits |= FAULT_BIT_UNDERVOLT;
    }

    /* 派生单位（显示用，低频换算即可） */
    g_fb.enc_raw = s_enc.angle_raw;
    g_fb.enc_deg = s_enc.angle_deg;
    g_fb.enc_samples = s_enc.sample_counter;
    g_fb.enc_reject = s_enc.reject_count;
    g_fb.enc_err = s_enc.comm_error_count;
    g_fb.spd_rot_rpm = units_rad_s_to_rpm(g_fb.spd_rot_rad_s);
    g_fb.spd_out_rpm = units_rad_s_to_rpm(units_rotor_spd_to_out(g_fb.spd_rot_rad_s));
    g_fb.spd_ref_rpm = units_rad_s_to_rpm(s_spd_ref_applied_rad_s);
    g_fb.pos_out_deg = units_wrap_360(units_rad_to_deg(g_fb.pos_out_rad));
    g_fb.pos_ref_deg = units_wrap_360(g_cmd.pos_deg);
}

/* ================================ 对外接口 ================================ */

void ctrl_init(void)
{
    (void)memset((void *)&g_cmd, 0, sizeof(g_cmd));
    (void)memset((void *)&g_fb, 0, sizeof(g_fb));

    /* g_cmd 上电默认值 */
    g_cmd.mode = MOTOR_MODE_DISABLED;
    g_cmd.run = 0U;
    g_cmd.duty_a = CFG_PWM_TEST_DUTY_A;
    g_cmd.duty_b = CFG_PWM_TEST_DUTY_B;
    g_cmd.duty_c = CFG_PWM_TEST_DUTY_C;
    g_cmd.ud_v = 0.0f;
    g_cmd.uq_v = 0.0f;
    g_cmd.volt_open_angle_en = 0U;
    g_cmd.open_spd_elec_rad_s = CFG_OPEN_SPD_ELEC_RAD_S;
    g_cmd.id_a = 0.0f;
    g_cmd.iq_a = 0.0f;
    g_cmd.spd_rpm = CFG_SPD_RPM_DEFAULT;
    g_cmd.pos_deg = 0.0f;
    g_cmd.iq_lim_a = CFG_IQ_LIM_A_DEFAULT;
    /* 三环带宽整定（公式与数值推导见 app_config.h）：
     *   电流环：Kp=2π·bw·L，Ki=2π·bw·R（模最佳）
     *   速度环：Kp=2ζ·ωn·J/Kt，Ki=ωn²·J/Kt（二阶整定），再 ÷极对数 换算到电角速度域
     *   位置环：Kp=2π·bw（对象近似纯积分） */
    {
        float spd_wn = UNITS_TWO_PI * CFG_SPD_BW_HZ_DEFAULT;

        g_cmd.cur_kp = UNITS_TWO_PI * CFG_CUR_BW_HZ_DEFAULT * CFG_CUR_EQ_L_H;
        g_cmd.cur_ki = UNITS_TWO_PI * CFG_CUR_BW_HZ_DEFAULT * CFG_CUR_EQ_R_OHM;
        g_cmd.spd_kp = (2.0f * CFG_SPD_DAMPING * spd_wn * CFG_MOTOR_INERTIA_KGM2 /
                        CFG_MOTOR_KT_NM_A) / CFG_MOTOR_POLE_PAIRS;
        g_cmd.spd_ki = (spd_wn * spd_wn * CFG_MOTOR_INERTIA_KGM2 /
                        CFG_MOTOR_KT_NM_A) / CFG_MOTOR_POLE_PAIRS;
        g_cmd.pos_kp = UNITS_TWO_PI * CFG_POS_BW_HZ_DEFAULT;
    }
    g_cmd.spd_lpf_hz = CFG_SPD_LPF_HZ_DEFAULT;
    g_cmd.pos_ki = CFG_POS_KI_DEFAULT;
    g_cmd.pos_kd = CFG_POS_KD_DEFAULT;
    g_cmd.pos_hold_en = CFG_POS_HOLD_EN_DEFAULT;
    g_cmd.pos_spd_lim_rad_s = CFG_POS_SPD_LIM_RAD_S;

    /* 内部状态 */
    s_state = MOTOR_STATE_IDLE;
    s_mode_active = MOTOR_MODE_DISABLED;
    s_theta_open = 0.0f;
    s_slow_div = 0U;
    s_ofs_sum_a = 0U;
    s_ofs_sum_b = 0U;
    s_ofs_sum_c = 0U;
    s_ofs_cnt = 0U;
    s_ofs_a = CFG_OFFSET_DEFAULT_RAW;
    s_ofs_b = CFG_OFFSET_DEFAULT_RAW;
    s_ofs_c = CFG_OFFSET_DEFAULT_RAW;
    s_enc_ofs_rad = 0.0f;

    /* 测速 LPF 缓存置无效标记，强制首个测速窗按实际周期计算系数 */
    s_spd_lpf_hz_cache = -1.0f;
    s_spd_dt_cache_s = -1.0f;

    ctrl_reset_align();
    ctrl_reset_loops();

    if (SystemCoreClock != 0U) {
        s_cycles_to_us = 1000000.0f / (float)SystemCoreClock;
        s_loop_period_cycles = SystemCoreClock / (uint32_t)CFG_FAST_LOOP_HZ;
    } else {
        s_cycles_to_us = 0.0f;
        s_loop_period_cycles = 1U;
    }

    foc_core_init(&g_foc);
    foc_core_set_bus_voltage(&g_foc, 48.0f);
    foc_core_set_electrical_angle(&g_foc, 0.0f);

    filter_lpf_f32_init(&s_vbus_lpf,
                        ctrl_lpf_alpha(CFG_VBUS_LPF_HZ, CFG_SLOW_DT_S),
                        48.0f);
    g_fb.vbus_v = 48.0f;
    g_fb.duty_a = 0.5f;
    g_fb.duty_b = 0.5f;
    g_fb.duty_c = 0.5f;

    ma600a_init(&s_enc, &hspi1, ENC_CS_GPIO_Port, ENC_CS_Pin);
    ctrl_reset_observer();

    /* 在第一个控制中断到来前先发起一次读角 */
    (void)ma600a_read_angle(&s_enc);
}

/* 快环入口：10kHz，ADC1 注入组完成中断里调用。
 * 全部控制与保护都在这一个上下文内执行（VBUS/NTC 等慢任务按 CFG_SLOW_DIV 分频），
 * 三环只靠带宽区分（带宽目标在 app_config.h 配置，PI 增益上电时自动计算）。 */
void ctrl_fast_loop(uint16_t ia_raw, uint16_t ib_raw, uint16_t ic_raw,
                    uint16_t vbus_raw, uint16_t ntc_raw)
{
    uint32_t t0_cycles = board_cycles();
    uint16_t new_faults;
    uint32_t elapsed;
    float elapsed_us;
    float theta;

    /* 1. 原始值登记 + 零偏校准 */
    g_fb.ia_raw = ia_raw;
    g_fb.ib_raw = ib_raw;
    g_fb.ic_raw = ic_raw;

    if (g_fb.offset_ready == 0U) {
        s_ofs_sum_a += ia_raw;
        s_ofs_sum_b += ib_raw;
        s_ofs_sum_c += ic_raw;
        s_ofs_cnt++;
        if (s_ofs_cnt >= CFG_OFFSET_SAMPLES) {
            s_ofs_a = (uint16_t)(s_ofs_sum_a / CFG_OFFSET_SAMPLES);
            s_ofs_b = (uint16_t)(s_ofs_sum_b / CFG_OFFSET_SAMPLES);
            s_ofs_c = (uint16_t)(s_ofs_sum_c / CFG_OFFSET_SAMPLES);
            g_fb.offset_ready = 1U;
        }
    }

    /* 2. 发起下一次编码器 SPI 读取（非阻塞），并消费最近一个已完成样本 */
    (void)ma600a_read_angle(&s_enc);
    if (s_enc.data_valid != 0U) {
        ctrl_observer_update();
    } else {
        /* 编码器失效时复位观测与对齐，恢复后需重新对齐 */
        if (s_state != MOTOR_STATE_FAULT) {
            ctrl_reset_observer();
            if ((s_state == MOTOR_STATE_IDLE) || (s_state == MOTOR_STATE_ALIGN)) {
                ctrl_reset_align();
            }
        }
    }

    /* 3. 本拍电角度（全快环唯一计算点）→ 缓存 sin/cos → 电流反馈复用 */
    theta = ctrl_theta_fast();
    foc_core_set_electrical_angle(&g_foc, theta);
    ctrl_current_feedback(ia_raw, ib_raw, ic_raw);

    /* 4. 慢任务分频：VBUS/NTC、过压欠压、派生单位（1kHz，同上下文） */
    s_slow_div++;
    if (s_slow_div >= CFG_SLOW_DIV) {
        s_slow_div = 0U;
        ctrl_slow_tasks(vbus_raw, ntc_raw);
    }

    /* 5. 保护汇聚：快环故障 + 慢任务故障同上下文合并 */
    new_faults = ctrl_check_fast_faults();
    if (new_faults != 0U) {
        g_fb.fault_bits |= new_faults;
    }
    if ((g_fb.fault_bits != 0U) && (s_state != MOTOR_STATE_FAULT)) {
        ctrl_enter_fault();
    }

    /* 6. 故障清除请求：清故障同时把 run 拉 0，
     * 排除故障后必须显式重新置 run，防止电机毫无预兆自启 */
    if (g_cmd.clear_fault != 0U) {
        g_cmd.clear_fault = 0U;
        g_cmd.run = 0U;
        if ((s_state == MOTOR_STATE_FAULT) && (board_drv_fault() == 0U)) {
            g_fb.fault_bits = 0U;
            ctrl_enter_idle();
        }
    }

    /* 7. 状态机 */
    switch (s_state) {
    case MOTOR_STATE_IDLE:
        ctrl_fsm_idle();
        break;
    case MOTOR_STATE_ALIGN:
        ctrl_fsm_align();
        break;
    case MOTOR_STATE_RUN:
        ctrl_fsm_run();
        break;
    case MOTOR_STATE_FAULT:
    default:
        board_set_power_en(0U);
        foc_core_reset_output(&g_foc);
        break;
    }

    /* 8. 统一下发占空比 + 状态镜像 */
    board_write_duty(g_foc.duty.duty_a, g_foc.duty.duty_b, g_foc.duty.duty_c);

    g_fb.state = s_state;
    g_fb.mode_active = s_mode_active;
    g_fb.drv_fault = board_drv_fault();
    g_fb.enc_valid = s_enc.data_valid;
    g_fb.theta_elec_rad = g_foc.theta_elec;
    g_fb.duty_a = g_foc.duty.duty_a;
    g_fb.duty_b = g_foc.duty.duty_b;
    g_fb.duty_c = g_foc.duty.duty_c;

    /* 9. 快环耗时统计 */
    if (board_dwt_ok() != 0U) {
        elapsed = board_cycles() - t0_cycles;
        elapsed_us = (float)elapsed * s_cycles_to_us;
        g_fb.loop_us = elapsed_us;
        if (elapsed_us > g_fb.loop_us_max) {
            g_fb.loop_us_max = elapsed_us;
        }
        if (elapsed > s_loop_period_cycles) {
            g_fb.overrun_cnt++;
        }
    }
}

/* SPI 完成/错误回调（由 program.c 的 HAL 弱回调转发） */
void ctrl_spi_txrx_cplt(SPI_HandleTypeDef *hspi)
{
    ma600a_spi_txrx_cplt_callback(&s_enc, hspi);
}

void ctrl_spi_error(SPI_HandleTypeDef *hspi)
{
    ma600a_spi_error_callback(&s_enc, hspi);
}
