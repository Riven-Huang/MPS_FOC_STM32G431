#ifndef CLI_UART_H
#define CLI_UART_H

#include <stdint.h>

#include "usart.h"

void cli_uart_init(UART_HandleTypeDef *huart);
void cli_uart_send_text(const char *text);
uint8_t cli_uart_send_vofa(const float *values, uint8_t count);
uint8_t cli_uart_is_tx_busy(void);
uint8_t cli_uart_send_bytes(const uint8_t *data, uint16_t len);
uint8_t cli_uart_read_byte(uint8_t *value);
void cli_uart_poll_rx(void);
void cli_uart_rx_flush(void);

#endif /* CLI_UART_H */
