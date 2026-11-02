#ifndef HOST_LINK_H
#define HOST_LINK_H

#include <stdint.h>

void host_link_init(void);
/* Background parser/TX; returns 1 while desktop protocol owns the serial TX. */
uint8_t host_link_task(uint32_t now_ms);
/* Called before ctrl_fast_loop in the same ADC interrupt. */
void host_link_control_tick(uint32_t now_ms);

#endif
