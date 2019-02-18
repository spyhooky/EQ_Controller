#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include "stm32f10x.h"
#include <math.h>

void Framework_Timer1ms(void);
void Calc_CurrentTemp(uint16_t sch_timer,uint16_t sch_cycle);
void UART_CAN_Handler(void *p_arg);


#endif

