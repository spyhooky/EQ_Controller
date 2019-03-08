#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include "stm32f10x.h"
#include <math.h>

void Delay_us(u32 n);
void Framework_Timer1ms(void);
void Framework_Timer100ms(void);
void Calc_CurrentTemp(u16 sch_timer,u16 sch_cycle);
void Calc_Power_5V(u16 sch_timer,u16 sch_cycle);
void UART_CAN_Handler(void *p_arg);
void Package_Float(float data,u8 *buf);
void Uart_Transmit(u8 chn,u8 *buf, u16 lenth);



#endif

