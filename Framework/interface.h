#ifndef INTERFACE_H
#define INTERFACE_H

#include "stm32f10x.h"

void Timer_100us_Application(void);
void Timer_1ms_Application(void);
void Timer_100ms_Application(void);
void USART1_RecieveData(u8 data);
void USART2_RecieveData(u8 data);
void USART3_RecieveData(u8 data);
void UART4_RecieveData(u8 data);
void UART5_RecieveData(u8 data);

#endif

