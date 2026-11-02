#ifndef TEST_USART_H
#define TEST_USART_H
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(v) ((void)(v))
#endif
