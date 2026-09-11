#include "board.h"

#include <math.h>

#include "app_config.h"
#include "main.h"
#include "tim.h"

static uint8_t  s_power_en = 0U;
static uint8_t  s_dwt_ok = 0U;

/* 初始化：功率级保持关闭。DWT 由 board_dwt_init 单独开。 */
void board_init(void)
{
    s_power_en = 0U;
    HAL_GPIO_WritePin(N_SLEEP_GPIO_Port, N_SLEEP_Pin, GPIO_PIN_RESET);
    board_dwt_init();
}

/* 相电流：扣零偏后按 (Vref/4095) / (增益 × 分流电阻) 换算。
 * 当前链路（8mR、增益20）：0.16V/A，满量程约 ±10.3A。 */
float board_current_a(uint16_t raw, uint16_t offset_raw)
{
    float volts_per_count = CFG_ADC_REF_V / CFG_ADC_FULL_SCALE;
    float sense_voltage = ((float)raw - (float)offset_raw) * volts_per_count;

    return sense_voltage / (CFG_CUR_GAIN * CFG_SHUNT_OHM);
}

/* 母线电压：240k/10k 分压。 */
float board_vbus_v(uint16_t raw)
{
    float adc_voltage = (float)raw * (CFG_ADC_REF_V / CFG_ADC_FULL_SCALE);

    return adc_voltage * ((CFG_VBUS_R_UP_OHM + CFG_VBUS_R_DOWN_OHM) / CFG_VBUS_R_DOWN_OHM);
}

/* NTC 温度（10K B3950，假设上拉 CFG_NTC_PULLUP_OHM 到 Vref、NTC 到地）：
 *   Rntc = Rup × raw / (FullScale − raw)
 *   T(K) = 1 / ( ln(Rntc/R25)/B + 1/298.15 )
 * 返回 ℃；输入无效时返回 -273.15f 供上层识别。 */
float board_ntc_c(uint16_t raw)
{
    float r_ntc;
    float frac;
    float temp_k;

    if ((raw == 0U) || (raw >= (uint16_t)CFG_ADC_FULL_SCALE)) {
        return -273.15f;
    }

    frac = (float)raw / (CFG_ADC_FULL_SCALE - (float)raw);
    r_ntc = CFG_NTC_PULLUP_OHM * frac;
    if (r_ntc <= 0.0f) {
        return -273.15f;
    }

    temp_k = 1.0f / ((logf(r_ntc / CFG_NTC_R25_OHM) / CFG_NTC_BETA) + (1.0f / 298.15f));
    return temp_k - 273.15f;
}

/* 功率级使能：只在状态变化时真正写 GPIO，避免快环里反复翻转。 */
void board_set_power_en(uint8_t enable)
{
    uint8_t next = (enable != 0U) ? 1U : 0U;

    if (next == s_power_en) {
        return;
    }

    HAL_GPIO_WritePin(N_SLEEP_GPIO_Port, N_SLEEP_Pin,
                      (next != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    s_power_en = next;
}

uint8_t board_power_enabled(void)
{
    return s_power_en;
}

/* N_FAULT 低有效。 */
uint8_t board_drv_fault(void)
{
    return (HAL_GPIO_ReadPin(N_FAULT_GPIO_Port, N_FAULT_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static float board_clamp01(float v)
{
    if (v < 0.0f) {
        return 0.0f;
    }
    if (v > 1.0f) {
        return 1.0f;
    }
    return v;
}

/* 占空比 → TIM1 CCR（中心对齐模式，周期 = ARR+1）。 */
void board_write_duty(float duty_a, float duty_b, float duty_c)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    uint32_t ccr_a = (uint32_t)(board_clamp01(duty_a) * (float)period);
    uint32_t ccr_b = (uint32_t)(board_clamp01(duty_b) * (float)period);
    uint32_t ccr_c = (uint32_t)(board_clamp01(duty_c) * (float)period);

    if (ccr_a >= period) {
        ccr_a = period - 1U;
    }
    if (ccr_b >= period) {
        ccr_b = period - 1U;
    }
    if (ccr_c >= period) {
        ccr_c = period - 1U;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_c);
}

void board_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_dwt_ok = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? 1U : 0U;
}

uint8_t board_dwt_ok(void)
{
    return s_dwt_ok;
}

uint32_t board_cycles(void)
{
    return (s_dwt_ok != 0U) ? DWT->CYCCNT : 0U;
}
