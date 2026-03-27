#include "ma600a.h"

#define MA600A_FULL_SCALE_COUNTS 65536.0f
#define MA600A_ANGLE_CMD         0x0000U
#define MA600A_TWO_PI            6.28318530718f

/* 函数作用：把 MA600A 片选脚拉低，开始一次 SPI 帧传输。
 * 输入：sensor 为已完成初始化的 MA600A 设备对象。
 * 输出：无返回值。
 * 运行频率：每次读取角度前调用 1 次。
 * 运行内容：把编码器片选脚拉低，让外部编码器进入当前 SPI 帧。
 */
static void ma600a_cs_low(const ma600a_t *sensor)
{
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);
}

/* 函数作用：把 MA600A 片选脚拉高，结束一次 SPI 帧传输。
 * 输入：sensor 为已完成初始化的 MA600A 设备对象。
 * 输出：无返回值。
 * 运行频率：每次读取角度后调用 1 次。
 * 运行内容：释放编码器片选脚，使总线回到空闲状态。
 */
static void ma600a_cs_high(const ma600a_t *sensor)
{
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);
}

/* 函数作用：把 16bit 原始角度码换算成角度制与弧度制。
 * 输入：sensor 为设备对象，angle_raw 为编码器返回的 16bit 原始角度码。
 * 输出：无返回值。
 * 运行频率：每次成功读到角度后调用 1 次。
 * 运行内容：保存原始计数，并同步更新角度制和弧度制结果，供 debug 与上层控制读取。
 */
static void ma600a_update_angle_cache(ma600a_t *sensor, uint16_t angle_raw)
{
    sensor->angle_raw = angle_raw;
    sensor->angle_deg = ((float)angle_raw * 360.0f) / MA600A_FULL_SCALE_COUNTS;
    sensor->angle_rad = ((float)angle_raw * MA600A_TWO_PI) / MA600A_FULL_SCALE_COUNTS;
    sensor->data_valid = 1U;
    sensor->sample_counter++;
}

/* 函数作用：复位一次 MA600A 读数状态。
 * 输入：sensor 为已初始化的编码器对象。输出：无返回值。调用频率：SPI 读角失败或初始化清零时调用。运行内容：清除接收缓存并撤销当前角度数据有效标志。 */
static void ma600a_reset_state(ma600a_t *sensor)
{
    sensor->rx_word = 0U;
    sensor->data_valid = 0U;
}

/* 函数作用：初始化 MA600A 设备对象并绑定底层 SPI 与片选引脚。
 * 输入：sensor 为设备对象指针，hspi 为已完成 CubeMX 初始化的 SPI 句柄，
 *      cs_port/cs_pin 为编码器片选引脚。
 * 输出：无返回值。
 * 运行频率：系统上电初始化时调用 1 次。
 * 运行内容：保存驱动依赖、清零角度缓存，并把片选脚拉高到空闲状态。
 */
void ma600a_init(ma600a_t *sensor, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if ((sensor == 0) || (hspi == 0) || (cs_port == 0)) {
        return;
    }

    sensor->hspi = hspi;
    sensor->cs_port = cs_port;
    sensor->cs_pin = cs_pin;
    sensor->tx_word = 0U;
    sensor->rx_word = 0U;
    sensor->angle_raw = 0U;
    sensor->angle_deg = 0.0f;
    sensor->angle_rad = 0.0f;
    sensor->sample_counter = 0U;
    ma600a_reset_state(sensor);

    ma600a_cs_high(sensor);
}

/* 函数作用：读取 MA600A 当前机械绝对角度。
 * 输入：sensor 为已完成初始化的 MA600A 设备对象。
 * 输出：返回 1 表示本次 SPI 读取成功，返回 0 表示读取失败。
 * 运行频率：当前建议放在后台慢任务中约 1 kHz 调用，后续也可迁移到控制节拍里。
 * 运行内容：在当前 CubeMX 配置的 16bit SPI 帧下发送 1 个空命令字，
 *      读取返回的 16bit 角度码，并同步换算成角度制与弧度制。
 */
/* Blocking SPI read: used in the 10 kHz fast loop for same-cycle angle feedback. */
uint8_t ma600a_read_angle(ma600a_t *sensor)
{
    HAL_StatusTypeDef status;

    if ((sensor == 0) || (sensor->hspi == 0) || (sensor->cs_port == 0)) {
        return 0U;
    }

    sensor->tx_word = MA600A_ANGLE_CMD;
    sensor->rx_word = 0U;

    ma600a_cs_low(sensor);
    status = HAL_SPI_TransmitReceive(sensor->hspi,
                                     (uint8_t *)&sensor->tx_word,
                                     (uint8_t *)&sensor->rx_word,
                                     1U,
                                     1U);
    ma600a_cs_high(sensor);

    if (status != HAL_OK) {
        ma600a_reset_state(sensor);
        return 0U;
    }

    ma600a_update_angle_cache(sensor, sensor->rx_word);
    return 1U;
}
