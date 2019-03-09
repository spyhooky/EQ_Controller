#ifndef __TASK_PC_MSG_RECV_H
#define __TASK_PC_MSG_RECV_H
#include "stm32f10x.h"
#include "variable.h"

//定义串口通道
#define UART_PC_MESSAGE_CHN                RS485_1



typedef struct Polling_Frame
{
    s32 Suspende_Position;//吊杆当前位置,单位mm
    u16 Suspende_Running_Status; //吊杆运行状态
    //float Environment_Temp; //环境温度
    //float Motor_Temp;  //电机温度
    volatile BitStatus  Running_Error_Sts[6];  //故障代码
}Polling_Frame_Info;
extern Polling_Frame_Info Polling_Frame_Respond;

//extern volatile digitstatus    	        _Running_Error_Sts[6];
//#define Running_Error_Sts(n)    _Running_Error_Sts[n].bytetype
#define Suspende_Reset  		        Polling_Frame_Respond.Running_Error_Sts[0].Bits.bit0 //0-吊杆复位
#define Limit_Up_Signal  		        Polling_Frame_Respond.Running_Error_Sts[0].Bits.bit1 //1-上限位信号
#define Limit_Down_Signal  		        Polling_Frame_Respond.Running_Error_Sts[0].Bits.bit2 //2-下限位信号
#define Limit_Up_SlowDown  		        Polling_Frame_Respond.Running_Error_Sts[0].Bits.bit3 //3-上限位减速信号
#define Limit_Down_SlowDown  	        Polling_Frame_Respond.Running_Error_Sts[0].Bits.bit4 //4-下限位减速信号
#define Band_Type_Brake       	        Polling_Frame_Respond.Running_Error_Sts[0].Bits.bit5 //5-抱闸信号
#define Reserve0      	                Polling_Frame_Respond.Running_Error_Sts[0].Bits.bit6 //6-预留
#define Reserve1                        Polling_Frame_Respond.Running_Error_Sts[0].Bits.bit7 //7-预留

#define Reserve2  		                Polling_Frame_Respond.Running_Error_Sts[1].Bits.bit0 //8-预留
#define Reserve3  		                Polling_Frame_Respond.Running_Error_Sts[1].Bits.bit1 //9-预留
#define Reserve4  		                Polling_Frame_Respond.Running_Error_Sts[1].Bits.bit2 //10-预留
#define Reserve5  		                Polling_Frame_Respond.Running_Error_Sts[1].Bits.bit3 //11-预留
#define Reserve6  	                    Polling_Frame_Respond.Running_Error_Sts[1].Bits.bit4 //12-预留
#define Reserve7            	        Polling_Frame_Respond.Running_Error_Sts[1].Bits.bit5 //13-预留
#define Reserve8      	                Polling_Frame_Respond.Running_Error_Sts[1].Bits.bit6 //14-预留
#define Reserve9                        Polling_Frame_Respond.Running_Error_Sts[1].Bits.bit7 //15-预留

#define Err_Stop_Signal 		        Polling_Frame_Respond.Running_Error_Sts[2].Bits.bit0 //16-急停故障
#define Err_Summit_Attempt              Polling_Frame_Respond.Running_Error_Sts[2].Bits.bit1 //17-冲顶故障
#define Err_Loose_Rope  		        Polling_Frame_Respond.Running_Error_Sts[2].Bits.bit2 //18-松绳故障
#define Err_Temp_Hign  		            Polling_Frame_Respond.Running_Error_Sts[2].Bits.bit3 //19-SLAVE高温故障
#define Err_Temp_Low            	    Polling_Frame_Respond.Running_Error_Sts[2].Bits.bit4 //20-SLAVE低温故障
#define Err_Voltage_High     		    Polling_Frame_Respond.Running_Error_Sts[2].Bits.bit5 //21-SLAVE高压故障
#define Err_Voltage_Low  	            Polling_Frame_Respond.Running_Error_Sts[2].Bits.bit6 //22-SLAVE低压故障
#define Inverter_Acc_OverCurrent        Polling_Frame_Respond.Running_Error_Sts[2].Bits.bit7 //23-变频器加速过电流故障

#define Inverter_Slow_OverCurrent  		Polling_Frame_Respond.Running_Error_Sts[3].Bits.bit0 //24-变频器减速过电流故障
#define Inverter_Const_OverCurrent      Polling_Frame_Respond.Running_Error_Sts[3].Bits.bit1 //25-变频器恒速过电流故障
#define Inverter_Acc_OverVoltage  		Polling_Frame_Respond.Running_Error_Sts[3].Bits.bit2 //26-变频器加速过电压故障
#define Inverter_Slow_OverVoltage  		Polling_Frame_Respond.Running_Error_Sts[3].Bits.bit3 //27-变频器减速过电压故障
#define Inverter_Const_OverVoltage     	Polling_Frame_Respond.Running_Error_Sts[3].Bits.bit4 //28-变频器恒速过电压故障
#define Inverter_OverTemp     		    Polling_Frame_Respond.Running_Error_Sts[3].Bits.bit5 //29-变频器过热故障
#define Inverter_OverLoad  	            Polling_Frame_Respond.Running_Error_Sts[3].Bits.bit6 //30-变频器过载故障
#define Inverter_Input_LackPhase        Polling_Frame_Respond.Running_Error_Sts[3].Bits.bit7 //31-变频器输入缺相故障

#define Inverter_Output_LackPhase  		Polling_Frame_Respond.Running_Error_Sts[4].Bits.bit0 //32-变频器输出缺相故障
#define Motor_OverLoad        		    Polling_Frame_Respond.Running_Error_Sts[4].Bits.bit1 //33-电机过载故障
#define Motor_Runing_UnderVoltage  		Polling_Frame_Respond.Running_Error_Sts[4].Bits.bit2 //34-电机运行中欠电压故障
#define Motor_ShortToGND  		        Polling_Frame_Respond.Running_Error_Sts[4].Bits.bit3 //35-电机对地短路故障
#define Motor_OverTemp            	    Polling_Frame_Respond.Running_Error_Sts[4].Bits.bit4 //36-电机过温故障
#define Motor_OverSpeed     		    Polling_Frame_Respond.Running_Error_Sts[4].Bits.bit5 //37-电机过速故障
#define Miss_PID_Respond  	            Polling_Frame_Respond.Running_Error_Sts[4].Bits.bit6 //38-PID反馈丢失故障
#define Suspende_Below_Zero             Polling_Frame_Respond.Running_Error_Sts[4].Bits.bit7 //39-吊杆位置低于零位坐标

#define Reserve10  		                Polling_Frame_Respond.Running_Error_Sts[5].Bits.bit0 //40-预留
#define Reserve11  		                Polling_Frame_Respond.Running_Error_Sts[5].Bits.bit1 //41-预留
#define Reserve12  		                Polling_Frame_Respond.Running_Error_Sts[5].Bits.bit2 //42-预留
#define Reserve13  		                Polling_Frame_Respond.Running_Error_Sts[5].Bits.bit3 //43-预留
#define Reserve14  	                    Polling_Frame_Respond.Running_Error_Sts[5].Bits.bit4 //44-预留
#define Reserve15           	        Polling_Frame_Respond.Running_Error_Sts[5].Bits.bit5 //45-预留
#define Reserve16      	                Polling_Frame_Respond.Running_Error_Sts[5].Bits.bit6 //46-预留
#define Reserve17                       Polling_Frame_Respond.Running_Error_Sts[5].Bits.bit7 //47-预留



void Task_PC_Message_Recv(void *p_arg);
void PC_COM_Timer100ms(void);





#endif


