#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include "stm32f10x.h"
#include <math.h>

void Framework_Timer1ms(void);
void Calc_CurrentTemp(u16 sch_timer,u16 sch_cycle);
void Calc_Power_5V(u16 sch_timer,u16 sch_cycle);
void UART_CAN_Handler(void *p_arg);


#endif

