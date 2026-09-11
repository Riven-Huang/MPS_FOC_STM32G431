#ifndef FOC_CORE_H
#define FOC_CORE_H

typedef struct
{
    float alpha;
    float beta;
} foc_alpha_beta_t;

typedef struct
{
    float d;
    float q;
} foc_dq_t;

typedef struct
{
    float duty_a;
    float duty_b;
    float duty_c;
} foc_svpwm_duty_t;

typedef struct
{
    float vbus;
    float theta_elec;
    float sin_theta;
    float cos_theta;
    foc_alpha_beta_t i_ab;
    foc_dq_t i_dq;
    foc_dq_t v_dq_cmd;
    foc_alpha_beta_t v_ab_cmd;
    foc_svpwm_duty_t duty;
} foc_core_t;

void foc_core_init(foc_core_t *core);
void foc_core_reset_output(foc_core_t *core);
void foc_core_set_bus_voltage(foc_core_t *core, float vbus);
void foc_core_set_electrical_angle(foc_core_t *core, float theta_elec);

void foc_core_clarke(float ia, float ib, foc_alpha_beta_t *out);
void foc_core_park(const foc_alpha_beta_t *ab, float sin_theta, float cos_theta, foc_dq_t *out);
void foc_core_inv_park(const foc_dq_t *dq, float sin_theta, float cos_theta, foc_alpha_beta_t *out);
void foc_core_svpwm(foc_core_t *core, float v_alpha, float v_beta, float vbus);

/* 应用 dq 电压到三相占空比（反 Park + SVPWM）。
 * 约定：调用前本拍必须先 foc_core_set_electrical_angle()，
 *       本函数复用已缓存的 sin/cos，不再重复计算角度。 */
void foc_core_apply_voltage(foc_core_t *core, float ud, float uq, float vbus);

#endif /* FOC_CORE_H */
