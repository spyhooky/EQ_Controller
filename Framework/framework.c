
#include "main.h"
#include "Task_TCPIP.h"
#include "Task_ModbusTCp.h"
#include "Task_MQTT.h"
#include "Task_IO.h"
//#include "Task_LED.h"

#ifdef FRAMEWORK_H

/****************************************************************************/
/*函数名：  Framework_Init                                                  */
/*功能说明：framework里变量的初始化                                          */
/*输入参数：无                                                              */
/*输出参数：无                                                              */
/***************************************************************************/
void Framework_Init(void)
{
	
}

/****************************************************************************/
/*函数名：  Framework_Timer1ms                                              */
/*功能说明：1ms定时器的服务函数，每1ms执行一次                                */
/*输入参数：无                                                              */
/*输出参数：无                                                              */
/***************************************************************************/
void Framework_Timer1ms(void)
{
	TaskIO_Timer1ms();
	MQTT_Timer1ms();
    MBTCP_Timer1ms();
    TCPIP_Timer1ms();
}

/****************************************************************************/
/*函数名：  Calc_CurrentTemp                                                 */
/*功能说明：获取当前温度值的接口，根据AD值计算出温度，若要精确测温，
每个芯片均需要标定更新温度值                                             */
/*输出参数：无                                                               */
/****************************************************************************/
void Calc_CurrentTemp(u16 sch_timer,u16 sch_cycle)
{
    float temp;
    temp = (1.42 - GET_ADC_Result(TempSensor)*3.3/4096)*1000/4.35 + 25;
	Globle_Framework.CurrentEnvTemp =  temp;
}

void Calc_Power_5V(u16 sch_timer,u16 sch_cycle)
{
	Globle_Framework.Power_5V =  GET_ADC_Result(Power_5V)*11/4096;
}

void Uart_Transmit(u8 chn,u8 *buf, u16 lenth)
{
    switch(chn)
    {
        case RS232_1:USART1_Send_Data(buf,lenth);break;
        case RS485_1:USART2_Send_Data(buf,lenth);break;
        case RS485_2:USART3_Send_Data(buf,lenth);break;
        case RS485_3:UART4_Send_Data(buf,lenth);break;
        case RS485_4:UART5_Send_Data(buf,lenth);break;
        default:break;
    }
}

void Package_Float(float data,u8 *buf)
{
    u8 *addr;
    addr = (u8 *)&data;
    buf[0]=addr[1];
    buf[1]=addr[0];
    buf[2]=addr[3];
    buf[3]=addr[2];
}


//此函数暂时不使用
void UART_CAN_Handler(void *p_arg)
{
	struct wiz_NetInfo_t * Ethparm;
    Ethparm = (struct wiz_NetInfo_t *)p_arg;
	u8 i;
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

