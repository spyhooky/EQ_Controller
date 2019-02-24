
#include "main.h"
#include "Task_TCPIP.h"
#include "Task_ModbusTCp.h"
#include "Task_MQTT.h"
#include "Task_IO.h"
//#include "Task_LED.h"

#ifdef FRAMEWORK_H

/****************************************************************************/
/*��������  Framework_Init                                                  */
/*����˵����framework������ĳ�ʼ��                                          */
/*�����������                                                              */
/*�����������                                                              */
/***************************************************************************/
void Framework_Init(void)
{
	
}

/****************************************************************************/
/*��������  Framework_Timer1ms                                              */
/*����˵����1ms��ʱ���ķ�������ÿ1msִ��һ��                                */
/*�����������                                                              */
/*�����������                                                              */
/***************************************************************************/
void Framework_Timer1ms(void)
{
	TaskIO_Timer1ms();
	MQTT_Timer1ms();
    MBTCP_Timer1ms();
    TCPIP_Timer1ms();
}

/****************************************************************************/
/*��������  Calc_CurrentTemp                                                 */
/*����˵������ȡ��ǰ�¶�ֵ�Ľӿڣ�����ADֵ������¶ȣ���Ҫ��ȷ���£�
ÿ��оƬ����Ҫ�궨�����¶�ֵ                                             */
/*�����������                                                               */
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


//�˺�����ʱ��ʹ��
void UART_CAN_Handler(void *p_arg)
{
	struct wiz_NetInfo_t * Ethparm;
    Ethparm = (struct wiz_NetInfo_t *)p_arg;
	u8 i;
	for(i=0;i<NUM_UARTCAN;i++)//�����˿�����ͨ��
	{
		if(USARTCAN_Recv[i].newupd==ON)//�Ƿ��յ���֡
		{
			//LED_BLINK_ONCE(S_NORMAL);//�յ���֡����SYS����˸һ��
			if(Ethparm->conn_status)//ֻ�ж˿ڽ��������˲Ż�����ת��
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

