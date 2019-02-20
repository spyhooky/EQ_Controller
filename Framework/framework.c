
#include "main.h"
#include "Task_TCPIP.h"
#include "Task_ModbusTCp.h"
#include "Task_MQTT.h"
#include "Task_IO.h"
//#include "Task_LED.h"

#ifdef FRAMEWORK_H

void Framework_Init(void)
{
	
}

void Framework_Timer1ms(void)
{
	TaskIO_Timer1ms();
	USART_Timer1ms();
	MQTT_Timer1ms();
    MBTCP_Timer1ms();
    TCPIP_Timer1ms();
}

static float_t Get_CurrentTemp(u16 tempsensor_AD)
{
	float temp;
	temp = (1.42 - tempsensor_AD*3.3/4096)*1000/4.35 + 25;
	return temp;
}

void Calc_CurrentTemp(u16 sch_timer,u16 sch_cycle)
{
	Globle_Framework.CurrentEnvTemp =  Get_CurrentTemp(Global_Driver.AD_Result[TempSensor]);
}

void Calc_Power_5V(u16 sch_timer,u16 sch_cycle)
{
    Globle_Framework.Power_5V =  Global_Driver.AD_Result[Power_5V]*110/4096;
}




void UART_CAN_Handler(void *p_arg)
{
	struct wiz_NetInfo_t * Ethparm;
    Ethparm = (struct wiz_NetInfo_t *)p_arg;
	uint8_t i;
	for(i=0;i<NUM_UARTCAN;i++)//遍历端口所有通道
	{
		if(USARTCAN_Recv[i].newupd==ON)//是否收到新帧
		{
			//LED_BLINK_ONCE(S_NORMAL);//收到新帧了让SYS灯闪烁一次
			if(Ethparm->conn_status)//只有端口建立连接了才会启动转换
			{
				USARTCAN_Recv[i].newupd=OFF;
				if(Ethparm->session_mode==S_mqtt)
				{
					MQTT_DataHandler(i,&USARTCAN_Recv[i]);
				}
				else if((Ethparm->session_mode==S_tcpip_client)||(Ethparm->session_mode==S_tcpip_server))
				{
					TCPIP_DataHandler(i,&USARTCAN_Recv[i],p_arg);
				}
				else if((Ethparm->session_mode==S_mb_client)||(Ethparm->session_mode==S_mb_server))
				{
					MBTCP_DataHandler(i,&USARTCAN_Recv[i],p_arg);
				}
				else
				{
					//LED_BLINK_ONCE(S_FAULT);
					//errorcode
				}
			}
		}
	}
}


#endif

