#ifndef _TASK_IO_H_
#define _TASK_IO_H_
#include "stm32f10x.h"

enum RELAY_OUTPUT_CHN{
    RELAY1,RELAY2,RELAY3,RELAY4,OUTPUT1,OUTPUT2,OUTPUT3
};

void Task_IO(void *p_arg);
void TaskIO_Timer1ms(void);
u8 Get_LED_Status(void);
void Blink_LED_Status(u16 mstimer);

#define SET_LED_STATUS(n)     		LED_G(n) 
#define SET_BAND_TYPE_BRAKE_STATUS(n)     	SET_DIGIT_OUTPUT_STATUS(RELAY4,n)


#endif


