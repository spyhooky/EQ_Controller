#ifndef INTERFACE_H
#define INTERFACE_H

#include "stm32f10x.h"

void Timer_100us_Application(void);
void Timer_1ms_Application(void);
void Timer_100ms_Application(void);
void USART1_RecieveData(uint8_t data);
void USART2_RecieveData(uint8_t data);
void USART3_RecieveData(uint8_t data);
void UART4_RecieveData(uint8_t data);
void UART5_RecieveData(uint8_t data);

#endif

