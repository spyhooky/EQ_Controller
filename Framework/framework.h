#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#include "stm32f10x.h"
#include <math.h>

#define TEMP_SAMPLES                                   20u

typedef struct UartOpFuncTyp
{
	void (*_send)  (u8 *sendbuf, u16 lenth);
	void (*_recv)  (u8 data);
}UartOpFunc_t;
extern UartOpFunc_t UartOpFunc[NUM_UARTCHANNEL];

void Framework_Init(void);
void Delay_us(u32 n);
void Framework_Timer1ms(void);
void Framework_Timer100ms(void);
void Calculate_Wire_Position(u16 sch_timer,u16 sch_cycle);
void Calc_CurrentTemp(u16 sch_timer,u16 sch_cycle);
void Calc_Power_5V(u16 sch_timer,u16 sch_cycle);
void UART_CAN_Handler(void *p_arg);
void Package_Float(float data,u8 *buf);
void UnPackage_Float(u8 *buf,float *data);
void Error_Indicator(u16 time);
void Para_Download(void);
void ReadFlashData(void);



#endif

