#include "program.h"

/* ============================================================================
 * program.c — 应用胶水层：初始化、后台遥测、HAL 回调转发
 *
 * 中断时序（由 CubeMX 配置决定，勿在代码里改优先级）：
 *   ADC1 注入组（TIM1_TRGO2 触发，10kHz）→ ctrl_fast_loop()，唯一控制上下文
 *                                        （VBUS/NTC 等慢任务在其内部 1/10 分频）
 *   SPI1（编码器读数完成）               → ma600a 回调            [优先级1]
 *   TIM6（1kHz 节拍 + ADC2 TRGO）        → s_tick_ms++           [优先级2]
 *   ADC2 + DMA（VBUS/NTC 慢变量）        → 环形缓冲，由快环分频读取
 * ==========================================================================*/

#include "adc.h"
#include "app_config.h"
#include "board.h"
#include "cli_uart.h"
#include "motor_ctrl.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* ADC2 DMA 缓冲：CH0=VBUS，CH1=NTC */
#define ADC2_DMA_LEN    2U
#define VOFA_CHANNEL_COUNT 5U

static volatile uint16_t s_adc2_buf[ADC2_DMA_LEN];
static volatile uint32_t s_tick_ms;
static uint32_t s_last_vofa_ms;

/* -------------------------------- 初始化 -------------------------------- */

/* 启动 TIM1 三相互补 PWM（先 50% 占空比再开通道） */
static void program_start_pwm(void)
{
    board_write_duty(0.5f, 0.5f, 0.5f);

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
}

/* 启动 ADC2 + DMA（VBUS/NTC）与 TIM6 1kHz 节拍 */
static void program_start_slow_chain(void)
{
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc2_buf, ADC2_DMA_LEN) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        Error_Handler();
    }
}

/* 启动 ADC1 注入组（TIM1_TRGO2 触发的三相电流采样，10kHz 快环） */
static void program_start_fast_chain(void)
{
    program_start_pwm();
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
        Error_Handler();
    }
}

void program_init(void)
{
    /* 功率级先保持关闭，再往下初始化 */
    board_init();
    ctrl_init();

    /* ADC 硬件校准 */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

    cli_uart_init(&huart1);

    program_start_slow_chain();
    program_start_fast_chain();

    /* 注意：USART1 使用 VOFA JustFloat 二进制协议，不要混入文本打印 */
}

/* -------------------------------- 后台任务（只做 VOFA 遥测，控制全部在快环中断内） -------------------------------- */

void program_task(void)
{
    uint32_t now_ms = s_tick_ms;
    uint32_t primask;
    float vofa_buf[VOFA_CHANNEL_COUNT];

    /* VOFA JustFloat，约 333Hz，5 路 + 帧尾 = 24 字节。
     * 115200 8N1 下每帧约 2.08ms，发送周期 3ms。 */
    if ((now_ms - s_last_vofa_ms) < CFG_VOFA_PERIOD_MS) {
        return;
    }
    s_last_vofa_ms = now_ms;

    /* 短临界区只复制反馈，避免快环打断造成一帧内反馈跨拍。
     * 保留原中断屏蔽状态；UART DMA 发送在恢复中断后进行。
     * 慢速派生量仍按原有 1kHz 节拍更新。 */
    primask = __get_PRIMASK();
    __disable_irq();
    vofa_buf[0] = g_fb.id_a;          /* d 轴实际电流 A */
    vofa_buf[1] = g_fb.iq_a;          /* q 轴实际电流 A */
    vofa_buf[2] = g_fb.spd_out_rpm;   /* 输出轴转速 rpm */
    vofa_buf[3] = g_fb.pos_out_deg;   /* 输出轴位置 deg，0~360 */
    vofa_buf[4] = g_fb.enc_deg;       /* 编码器机械角 deg，0~360 */
    __set_PRIMASK(primask);
    (void)cli_uart_send_vofa(vofa_buf, VOFA_CHANNEL_COUNT);
}

/* -------------------------------- HAL 回调转发（弱符号覆盖，CubeMX 安全） -------------------------------- */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim != 0) && (htim->Instance == TIM6)) {
        s_tick_ms++;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;     /* ADC2 走 DMA 环形缓冲，这里无需处理 */
}

/* ADC1 注入组完成 = 10kHz 快环入口；顺带把 ADC2 DMA 的 VBUS/NTC 一起传入 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint16_t ia_raw;
    uint16_t ib_raw;
    uint16_t ic_raw;

    if ((hadc == 0) || (hadc->Instance != ADC1)) {
        return;
    }

    ia_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    ib_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    ic_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    ctrl_fast_loop(ia_raw, ib_raw, ic_raw,
                   (uint16_t)s_adc2_buf[0], (uint16_t)s_adc2_buf[1]);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    ctrl_spi_txrx_cplt(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    ctrl_spi_error(hspi);
}
