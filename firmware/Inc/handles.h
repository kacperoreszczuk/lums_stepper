
#ifndef HANDLES_H_INCLUDED
#define HANDLES_H_INCLUDED

#include "main.h"

//extern uint8_t uart_buffer[];
extern uint32_t uart_count;
extern uint32_t ticks;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
extern uint8_t tx_busy;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;
#endif
