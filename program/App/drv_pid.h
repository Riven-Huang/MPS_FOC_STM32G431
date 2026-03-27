#ifndef _DRV_PID_
#define _DRV_PID_

#include <stdint.h>

typedef struct {
    int32_t ref;
    int32_t feedback;
    int32_t error;
    int32_t kp_q15;
    int32_t ki_q15;
    int32_t p_out;
    int32_t i_out;
    int32_t output;
    int32_t i_term_q15;
    int32_t out_min;
    int32_t out_max;
} drv_pid_pi_t;

/* PI控制器初始化；输入输出量纲必须在上层先统一好。 */
void drv_pid_pi_init(drv_pid_pi_t *pid,
                     int32_t kp_q15,
                     int32_t ki_q15,
                     int32_t out_min,
                     int32_t out_max,
                     int32_t out_init);

/* 复位积分项；适合模式切换或故障恢复后重新接管。 */
void drv_pid_pi_reset(drv_pid_pi_t *pid, int32_t out_init);

/* 执行一次PI更新；ref与feedback量纲必须一致，运行结果会回写到结构体里。 */
int32_t drv_pid_pi_step(drv_pid_pi_t *pid, int32_t ref, int32_t feedback);

#endif // _DRV_PID_
//
// End of File
//
