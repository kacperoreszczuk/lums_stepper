#ifndef USR_MAIN_H_INCLUDED
#define USR_MAIN_H_INCLUDED

#include "stm32l4xx_hal.h"

void nxt_tick();
void control_tick();
void uart_byte_received(uint8_t byte);
void uart_analyse_buffer();
int usrMain();

#endif /* __MAIN_H */
