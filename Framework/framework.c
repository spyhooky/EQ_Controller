
#include "main.h"
#include "Task_TCPIP.h"
#include "Task_ModbusTCp.h"
#include "Task_MQTT.h"
#include "Task_IO.h"
#include "Task_MB_RTU_Master.h"

//#include "Task_LED.h"

#ifdef FRAMEWORK_H

UartOpFunc_t UartOpFunc[NUM_UARTCHANNEL];

static void UartFrame_Timeout_Calc(void);

/****************************************************************************/
/*函数名：  Framework_Init                                                  */
/*功能说明：framework里变量的初始化                                          */
/*输入参数：无                                                              */
/*输出参数：无                                                              */
/***************************************************************************/
void Framework_Init(void)
{
    u8 i;
	UartOpFunc[RS232_1]._send = USART1_Send_Data;
    UartOpFunc[RS485_1]._send = USART3_Send_Data;
    UartOpFunc[RS485_2]._send = USART2_Send_Data;
    UartOpFunc[RS485_3]._send = UART4_Send_Data;
    UartOpFunc[RS485_4]._send = UART5_Send_Data;

    for(i=0;i<NUM_UARTCHANNEL;i++)
    {
        mBOX_Uart_Recv[i] = OSMboxCreate((void *)0);
    }   

    //UartFrame_Timeout_Calc();
}

static void UartFrame_Timeout_Calc(void)
{
    u8 i;
    float temp;//5个字节传输所需的时间，单位10ns
    for(i=0;i<NUM_UARTCHANNEL;i++)
    {
        temp = 100000 * 5 * 1/RS232_baud[USARTCAN.Usart[i][uartBaudrate]]*10;
        USARTCAN.Usart[i][tmout] = (u8) ((temp + 5)/10);
        if(USARTCAN.Usart[i][tmout] < 6)
        {
            USARTCAN.Usart[i][tmout] = 6;
        }
    }
}


/****************************************************************************/
/*函数名：  Delay_us                                                  */
/*功能说明：短延时，单位：us                                          */
/*输入参数：n，1000即为1ms                                                              */
/*输出参数：无                                                              */
/***************************************************************************/

void Delay_us(u32 n)
{
    vu32 j;
    while(n--)
    {
        j=12;
        while(j--);
    }
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
    RTU_Timer1ms_Handler();
    #ifdef MQTT_ENABLE
	MQTT_Timer1ms();
    #endif
    #ifdef MBTCP_ENABLE
    MBTCP_Timer1ms();
    #endif
    #ifdef TCPIP_ENABLE
    TCPIP_Timer1ms();
    #endif
}

/****************************************************************************/
/*函数名：  Framework_Timer1ms                                              */
/*功能说明：1ms定时器的服务函数，每1ms执行一次                                */
/*输入参数：无                                                              */
/*输出参数：无                                                              */
/***************************************************************************/
void Framework_Timer100ms(void)
{
    PC_COM_Timer100ms();
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
    //temp = (1.42 - (GET_ADC_Result(TempSensor)*3.3/4096)/(GET_ADC_Result(VrefInit)*3.3/1.2/4096))*1000/4.35 + 25;
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
#ifdef W5500_ENABLE
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
                    #ifdef MQTT_ENABLE
					MQTT_DataHandler(i,&USARTCAN_Recv[i]);
                    #endif
				}
				else if((Ethparm->session_mode==S_tcpip_client)||(Ethparm->session_mode==S_tcpip_server))
				{
                    #ifdef TCPIP_ENABLE
					TCPIP_DataHandler(i,&USARTCAN_Recv[i],p_arg);
                    #endif
				}
				else if((Ethparm->session_mode==S_mb_client)||(Ethparm->session_mode==S_mb_server))
				{
                    #ifdef MBTCP_ENABLE
					MBTCP_DataHandler(i,&USARTCAN_Recv[i],p_arg);
                    #endif
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


#endif

