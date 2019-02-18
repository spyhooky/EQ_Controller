#ifndef _TASK_IO_H_
#define _TASK_IO_H_
#include "stm32f10x.h"


void Task_IO(void *p_arg);
void TaskIO_Timer1ms(void);
uint8_t Get_LED_Status(void);
void Blink_LED_Status(uint16_t timer);

#define SET_LED_STATUS(n)     		LED_G(n) 


#endif


