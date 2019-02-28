#ifndef __TASK_PC_MSG_RECV_H
#define __TASK_PC_MSG_RECV_H


//定义串口通道
#define UART_PC_MESSAGE_CHN                RS485_2


extern volatile digitstatus    	_Running_Error_Sts[4];//#define Running_Error_Sts(n)    _Running_Error_Sts[n].bytetype
#define Suspende_Reset  		_Running_Error_Sts[0].Bits.bit0 //吊杆复位
#define Limit_Up_Signal  		_Running_Error_Sts[0].Bits.bit1 //上限位信号
#define Limit_Down_Signal  		_Running_Error_Sts[0].Bits.bit2 //下限位信号
#define Limit_Up_SlowDown  		_Running_Error_Sts[0].Bits.bit3 //上限位减速信号
#define Limit_Down_SlowDown  	_Running_Error_Sts[0].Bits.bit4 //下限位减速信号
#define Band_Type_Brake       	_Running_Error_Sts[0].Bits.bit5 //抱闸信号

#define Summit_Attempt  		_Running_Error_Sts[2].Bits.bit0 //冲顶
#define Loose_Rope        		_Running_Error_Sts[2].Bits.bit1 //松绳
#define Stop_Signal  		    _Running_Error_Sts[2].Bits.bit2 //停止
#define Temp_Hign  		        _Running_Error_Sts[2].Bits.bit3 //高温
#define Temp_Low            	_Running_Error_Sts[2].Bits.bit4 //低温
#define Voltage_High     		_Running_Error_Sts[2].Bits.bit5 //高压
#define Voltage_Low  	        _Running_Error_Sts[2].Bits.bit6 //低压


void Task_PC_Message_Recv(void *p_arg);
void PC_COM_Timer100ms(void);





#endif


