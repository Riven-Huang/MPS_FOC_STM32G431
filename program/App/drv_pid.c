#include "drv_pid.h"

#define DRV_PID_Q15_SHIFT    15
#define DRV_PID_Q15_HALF     (1L << (DRV_PID_Q15_SHIFT - 1))

/* 函数作用：限制 32 位有符号数范围。
 * 输入：value 为待限制值，min_value/max_value 为上下限。
 * 输出：返回限幅后的结果。
 * 调用频率：PI 每次运行时会间接调用。
 * 运行内容：给积分项和最终输出做统一限幅。 */
static int32_t drv_pid_clamp_s32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/* 函数作用：把整数提升到 Q15 定点格式。
 * 输入：value 为普通整数值。
 * 输出：返回 Q15 格式结果。
 * 调用频率：初始化和积分计算时按需调用。
 * 运行内容：通过左移 15 位完成格式转换。 */
static int32_t drv_pid_s32_to_q15(int32_t value)
{
    return value << DRV_PID_Q15_SHIFT;
}

/* 函数作用：把 Q15 定点数四舍五入还原成整数。
 * 输入：value_q15 为 Q15 定点值。
 * 输出：返回整数结果。
 * 调用频率：PI 每次更新时按需调用。
 * 运行内容：对正负数分别做对称四舍五入，减小量化误差。 */
static int32_t drv_pid_q15_to_s32_round(int32_t value_q15)
{
    if (value_q15 >= 0) {
        return (value_q15 + DRV_PID_Q15_HALF) >> DRV_PID_Q15_SHIFT;
    }

    return -(((-value_q15) + DRV_PID_Q15_HALF) >> DRV_PID_Q15_SHIFT);
}

/* PI控制器初始化；输入输出量纲必须在上层先统一好。 */
/* 函数作用：初始化一个 PI 控制器对象。
 * 输入：pid 为控制器对象，kp_q15/ki_q15 为 Q15 增益，out_min/out_max 为输出限幅，out_init 为初始输出。
 * 输出：无返回值。
 * 调用频率：系统启动或控制器创建时调用一次。
 * 运行内容：写入参数并调用 reset 建立初始积分状态。 */
void drv_pid_pi_init(drv_pid_pi_t *pid,
                     int32_t kp_q15,
                     int32_t ki_q15,
                     int32_t out_min,
                     int32_t out_max,
                     int32_t out_init)
{
    if (pid == 0) {
        return;
    }

    pid->kp_q15 = kp_q15;
    pid->ki_q15 = ki_q15;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->ref = 0;
    pid->feedback = 0;
    pid->error = 0;
    pid->p_out = 0;

    drv_pid_pi_reset(pid, out_init);
}

/* 复位积分项；适合模式切换或故障恢复后重新接管。 */
/* 函数作用：复位 PI 控制器内部状态。
 * 输入：pid 为控制器对象，out_init 为希望恢复到的初始输出。
 * 输出：无返回值。
 * 调用频率：模式切换、故障恢复或重新接管时按需调用。
 * 运行内容：清空误差与比例项，并把积分项预装到期望初始输出。 */
void drv_pid_pi_reset(drv_pid_pi_t *pid, int32_t out_init)
{
    if (pid == 0) {
        return;
    }

    out_init = drv_pid_clamp_s32(out_init, pid->out_min, pid->out_max);
    pid->ref = 0;
    pid->feedback = 0;
    pid->error = 0;
    pid->p_out = 0;
    pid->output = out_init;
    pid->i_out = out_init;
    pid->i_term_q15 = drv_pid_s32_to_q15(out_init);
}

/* 执行一次PI更新；ref与feedback量纲必须一致，运行结果会回写到结构体里。 */
/* 函数作用：执行一次 PI 更新。
 * 输入：pid 为控制器对象，ref 为参考值，feedback 为反馈值，两者量纲必须一致。
 * 输出：返回本次 PI 输出，同时内部状态会回写到 pid 结构体。
 * 调用频率：由上层控制环按固定节拍调用。
 * 运行内容：计算误差、比例项和积分项，做抗饱和处理后输出控制量。 */
int32_t drv_pid_pi_step(drv_pid_pi_t *pid, int32_t ref, int32_t feedback)
{
    int32_t p_term_q15;
    int32_t i_candidate_q15;
    int32_t out_q15;
    int32_t out_s32;
    int32_t i_min_q15;
    int32_t i_max_q15;

    if (pid == 0) {
        return 0;
    }

    pid->ref = ref;
    pid->feedback = feedback;
    pid->error = ref - feedback;

    p_term_q15 = pid->error * pid->kp_q15;
    i_candidate_q15 = pid->i_term_q15 + (pid->error * pid->ki_q15);

    /* 积分项先限在输出物理范围内，防止长时间误差把内部状态冲到不可恢复。 */
    i_min_q15 = drv_pid_s32_to_q15(pid->out_min);
    i_max_q15 = drv_pid_s32_to_q15(pid->out_max);
    i_candidate_q15 = drv_pid_clamp_s32(i_candidate_q15, i_min_q15, i_max_q15);

    out_q15 = p_term_q15 + i_candidate_q15;
    out_s32 = drv_pid_q15_to_s32_round(out_q15);

    if (out_s32 > pid->out_max) {
        out_s32 = pid->out_max;
        if (pid->error < 0) {
            pid->i_term_q15 = i_candidate_q15;
        }
    } else if (out_s32 < pid->out_min) {
        out_s32 = pid->out_min;
        if (pid->error > 0) {
            pid->i_term_q15 = i_candidate_q15;
        }
    } else {
        pid->i_term_q15 = i_candidate_q15;
    }

    pid->p_out = drv_pid_q15_to_s32_round(p_term_q15);
    pid->i_out = drv_pid_q15_to_s32_round(pid->i_term_q15);
    pid->output = out_s32;

    return out_s32;
}

//
// End of File
//
