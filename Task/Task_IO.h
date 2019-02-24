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

#define SET_LED_STATUS(n)					LED_G(n) 
#define SET_BAND_TYPE_BRAKE_STATUS(s)		SET_DIGIT_OUTPUT_STATUS(RELAY1,s)

extern volatile digitstatus    	Relay_Output_Sts;//
#define Running_Error_Sts(n)    Relay_Output_Sts.bytetype
#define Band_Type_Brake_Out  	Relay_Output_Sts.Bits.bit0 //抱闸延时输出

#if 0
#define Limit_Up_Signal  		Relay_Output_Sts.Bits.bit1 //上限位信号
#define Limit_Down_Signal  		Relay_Output_Sts.Bits.bit2 //下限位信号
#define Limit_Up_SlowDown  		Relay_Output_Sts.Bits.bit3 //上限位减速信号
#define Limit_Down_SlowDown  	Relay_Output_Sts.Bits.bit4 //下限位减速信号
#define Band_Type_Brake       	Relay_Output_Sts.Bits.bit5 //抱闸信号
#endif


#endif


