#include "cli_uart.h"

#include <string.h>

#define CLI_UART_TX_BUF_SIZE    192U
#define CLI_UART_RX_BUF_SIZE    256U
#define CLI_UART_VOFA_TAIL_SIZE 4U

static const uint8_t g_cli_uart_vofa_tail[CLI_UART_VOFA_TAIL_SIZE] = {0x00U, 0x00U, 0x80U, 0x7FU};

static UART_HandleTypeDef *g_cli_uart = 0;
static volatile uint8_t g_cli_uart_tx_busy = 0U;
static uint8_t g_cli_uart_tx_buf[CLI_UART_TX_BUF_SIZE];
static uint8_t s_rx_byte;
static uint8_t s_rx_buf[CLI_UART_RX_BUF_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint8_t s_rx_overflow;

/* 函数作用：初始化调试串口发送模块。
 * 输入：huart 为要绑定的 UART 句柄，通常传入 USART1 句柄。
 * 输出：无返回值。
 * 调用频率：系统启动时调用 1 次。
 * 运行内容：缓存串口句柄，并清零 DMA 发送忙标志。 */
void cli_uart_init(UART_HandleTypeDef *huart)
{
    g_cli_uart = huart;
    g_cli_uart_tx_busy = 0U;
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_overflow = 0U;
    cli_uart_poll_rx();
}

/* IRQ only queues bytes. Parsing and TX remain in the background. */
void cli_uart_poll_rx(void)
{
    if ((g_cli_uart != 0) && (g_cli_uart->RxState == HAL_UART_STATE_READY)) {
        (void)HAL_UART_Receive_IT(g_cli_uart, &s_rx_byte, 1U);
    }
}

void cli_uart_rx_flush(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_rx_tail = s_rx_head;
    s_rx_overflow = 0U;
    __set_PRIMASK(primask);
}

uint8_t cli_uart_read_byte(uint8_t *value)
{
    if ((value == 0) || (s_rx_head == s_rx_tail)) {
        return 0U;
    }
    if (s_rx_overflow != 0U) {
        cli_uart_rx_flush();
        return 0U;
    }
    *value = s_rx_buf[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1U) % CLI_UART_RX_BUF_SIZE;
    return 1U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;
    if (huart != g_cli_uart) {
        return;
    }
    next = (s_rx_head + 1U) % CLI_UART_RX_BUF_SIZE;
    if (next == s_rx_tail) {
        s_rx_overflow = 1U;
    } else {
        s_rx_buf[s_rx_head] = s_rx_byte;
        s_rx_head = next;
    }
    (void)HAL_UART_Receive_IT(g_cli_uart, &s_rx_byte, 1U);
}

uint8_t cli_uart_send_bytes(const uint8_t *data, uint16_t len)
{
    if ((g_cli_uart == 0) || (data == 0) || (len == 0U) ||
        (len > CLI_UART_TX_BUF_SIZE) || (g_cli_uart_tx_busy != 0U)) {
        return 0U;
    }
    memcpy(g_cli_uart_tx_buf, data, len);
    g_cli_uart_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(g_cli_uart, g_cli_uart_tx_buf, len) != HAL_OK) {
        g_cli_uart_tx_busy = 0U;
        return 0U;
    }
    return 1U;
}

/* 函数作用：查询当前串口 DMA 发送是否忙。
 * 输入：无。
 * 输出：返回 1 表示上一帧仍在发送，返回 0 表示当前空闲。
 * 调用频率：后台任务发波形前按需调用。
 * 运行内容：只读取内部忙标志，不做额外计算。 */
uint8_t cli_uart_is_tx_busy(void)
{
    return g_cli_uart_tx_busy;
}

/* 函数作用：发送一段阻塞式短文本，主要留给串口调试打印使用。
 * 输入：text 为以 '\\0' 结尾的字符串。
 * 输出：无返回值。
 * 调用频率：低频按需调用，不建议和高频波形发送混用。
 * 运行内容：当 DMA 空闲时直接调用 HAL 阻塞发送；若 DMA 正忙则本次丢弃。 */
void cli_uart_send_text(const char *text)
{
    uint16_t len;

    if ((g_cli_uart == 0) || (text == 0)) {
        return;
    }

    if (g_cli_uart_tx_busy != 0U) {
        return;
    }

    len = (uint16_t)strlen(text);
    if (len == 0U) {
        return;
    }

    (void)HAL_UART_Transmit(g_cli_uart, (uint8_t *)text, len, 20U);
}

/* 函数作用：按 VOFA JustFloat 协议发送一帧二进制浮点数据。
 * 输入：values 为待发送浮点数组首地址，count 为通道数量。
 * 输出：返回 1 表示本次已成功启动 DMA 发送，返回 0 表示串口忙或缓冲区不足。
 * 调用频率：后台任务中按固定节拍调用，当前工程约 333 Hz。
 * 运行内容：把 float 数组按 STM32 小端字节序直接拷入发送缓冲区，再追加
 * VOFA 帧尾 {0x00,0x00,0x80,0x7F}，最后启动 UART DMA 非阻塞发送。 */
uint8_t cli_uart_send_vofa(const float *values, uint8_t count)
{
    uint32_t payload_len;
    uint32_t total_len;

    if ((g_cli_uart == 0) || (values == 0) || (count == 0U)) {
        return 0U;
    }

    if (g_cli_uart_tx_busy != 0U) {
        return 0U;
    }

    payload_len = (uint32_t)count * (uint32_t)sizeof(float);
    total_len = payload_len + CLI_UART_VOFA_TAIL_SIZE;
    if (total_len > CLI_UART_TX_BUF_SIZE) {
        return 0U;
    }

    (void)memcpy((void *)g_cli_uart_tx_buf, (const void *)values, payload_len);
    (void)memcpy((void *)&g_cli_uart_tx_buf[payload_len],
                 (const void *)g_cli_uart_vofa_tail,
                 CLI_UART_VOFA_TAIL_SIZE);

    g_cli_uart_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(g_cli_uart, g_cli_uart_tx_buf, (uint16_t)total_len) != HAL_OK) {
        g_cli_uart_tx_busy = 0U;
        return 0U;
    }

    return 1U;
}

/* 函数作用：UART DMA 发送完成回调。
 * 输入：huart 为 HAL 传入的 UART 句柄。
 * 输出：无返回值。
 * 调用频率：每发送完成 1 帧 JustFloat 数据后调用 1 次。
 * 运行内容：如果完成的是当前调试串口，则清零发送忙标志，允许下一帧进入。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((g_cli_uart != 0) && (huart == g_cli_uart)) {
        g_cli_uart_tx_busy = 0U;
    }
}

/* 函数作用：UART 错误回调。
 * 输入：huart 为 HAL 传入的 UART 句柄。
 * 输出：无返回值。
 * 调用频率：UART 或 DMA 发送异常时按需调用。
 * 运行内容：释放发送忙标志，避免后台发送状态永久卡死。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((g_cli_uart != 0) && (huart == g_cli_uart)) {
        /* RX noise must not release a still-active TX DMA buffer. */
        if (huart->gState == HAL_UART_STATE_READY) {
            g_cli_uart_tx_busy = 0U;
        }
        cli_uart_poll_rx();
    }
}
