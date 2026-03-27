#include "filter.h"

#define FILTER_Q15_ONE    32768L

/* 函数作用：限制浮点低通滤波系数范围。
 * 输入：alpha 为待检查的滤波系数。
 * 输出：返回 0.0~1.0 之间的合法系数。
 * 调用频率：初始化滤波器时调用。
 * 运行内容：避免无效系数导致滤波器发散。 */
static float filter_clamp_alpha(float alpha)
{
    if (alpha < 0.0f) {
        return 0.0f;
    }

    if (alpha > 1.0f) {
        return 1.0f;
    }

    return alpha;
}

/* 函数作用：限制 Q15 格式低通滤波系数范围。
 * 输入：alpha_q15 为待检查的 Q15 系数。
 * 输出：返回合法 Q15 系数。
 * 调用频率：初始化整数滤波器时调用。
 * 运行内容：把系数限制到 0~1.0 对应的 Q15 范围内。 */
static int32_t filter_clamp_q15(int32_t alpha_q15)
{
    if (alpha_q15 < 0) {
        return 0;
    }

    if (alpha_q15 > FILTER_Q15_ONE) {
        return FILTER_Q15_ONE;
    }

    return alpha_q15;
}

/* 函数作用：初始化浮点一阶低通滤波器。
 * 输入：filter 为滤波器对象，alpha 为滤波系数，initial_value 为初值。
 * 输出：无返回值。
 * 调用频率：通常在系统初始化时调用一次。
 * 运行内容：写入系数和初值，并标记滤波器已完成初始化。 */
void filter_lpf_f32_init(filter_lpf_f32_t *filter, float alpha, float initial_value)
{
    if (filter == 0) {
        return;
    }

    filter->alpha = filter_clamp_alpha(alpha);
    filter->value = initial_value;
    filter->initialized = 1U;
}

/* 函数作用：执行一次浮点一阶低通更新。
 * 输入：filter 为滤波器对象，input 为当前输入值。
 * 输出：返回本次更新后的滤波输出。
 * 调用频率：由上层按采样节拍调用。
 * 运行内容：首次调用直接对齐输入，之后按 y += a(x-y) 更新。 */
float filter_lpf_f32_update(filter_lpf_f32_t *filter, float input)
{
    if (filter == 0) {
        return input;
    }

    if (filter->initialized == 0U) {
        filter_lpf_f32_init(filter, filter->alpha, input);
        return input;
    }

    filter->value += filter->alpha * (input - filter->value);
    return filter->value;
}

/* 函数作用：初始化整数 Q15 一阶低通滤波器。
 * 输入：filter 为滤波器对象，alpha_q15 为 Q15 系数，initial_value 为整数初值。
 * 输出：无返回值。
 * 调用频率：通常在系统初始化时调用一次。
 * 运行内容：把初值提升到 Q15 内部状态，并保存滤波系数。 */
void filter_lpf_s32_init(filter_lpf_s32_t *filter, int32_t alpha_q15, int32_t initial_value)
{
    if (filter == 0) {
        return;
    }

    filter->alpha_q15 = filter_clamp_q15(alpha_q15);
    filter->value_q15 = initial_value << 15;
    filter->initialized = 1U;
}

/* 函数作用：执行一次整数 Q15 一阶低通更新。
 * 输入：filter 为滤波器对象，input 为当前整数输入值。
 * 输出：返回本次更新后的整数输出。
 * 调用频率：由上层按采样节拍调用。
 * 运行内容：先把输入提升到 Q15 精度做滤波，再还原成整数输出。 */
int32_t filter_lpf_s32_update(filter_lpf_s32_t *filter, int32_t input)
{
    int32_t input_q15;
    int32_t delta_q15;

    if (filter == 0) {
        return input;
    }

    if (filter->initialized == 0U) {
        filter_lpf_s32_init(filter, filter->alpha_q15, input);
        return input;
    }

    input_q15 = input << 15;
    delta_q15 = input_q15 - filter->value_q15;
    filter->value_q15 += (int32_t)(((int64_t)filter->alpha_q15 * delta_q15) >> 15);

    return filter->value_q15 >> 15;
}
