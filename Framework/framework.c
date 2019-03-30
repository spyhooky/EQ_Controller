
#include "main.h"
#include "Task_TCPIP.h"
#include "Task_ModbusTCp.h"
#include "Task_MQTT.h"
#include "Task_IO.h"
#include "Task_MB_RTU_Master.h"

//#include "Task_LED.h"

#ifdef FRAMEWORK_H

UartOpFunc_t UartOpFunc[NUM_UARTCHANNEL];
static u16 EnviromentTemp[TEMP_SAMPLES];//��������AD����ֵ����10�����ݣ�ÿ10ms����һ�Σ�Ȼ��ȡ�������Сֵ��ƽ��
static u8 TempSample_Index;//���²ɼ�����
static u16 PrePluse_Number;
static s32 Pulse_Total;
static void UartFrame_Timeout_Calc(void);

/****************************************************************************/
/*��������  Framework_Init                                                  */
/*����˵����framework������ĳ�ʼ��                                          */
/*�����������                                                              */
/*�����������                                                              */
/***************************************************************************/
void Framework_Init(void)
{
    u8 i;
    PrePluse_Number = 0;
	UartOpFunc[RS232_1]._send = USART1_Send_Data;
    UartOpFunc[RS485_1]._send = USART3_Send_Data;
    UartOpFunc[RS485_2]._send = USART2_Send_Data;
    UartOpFunc[RS485_3]._send = UART4_Send_Data;
    UartOpFunc[RS485_4]._send = UART5_Send_Data;

    for(i=0;i<NUM_UARTCHANNEL;i++)
    {
        mBOX_Uart_Recv[i] = OSMboxCreate((void *)0);
    }   

    TempSample_Index = 0;
    for(i=0;i<TEMP_SAMPLES;i++)
    {
        EnviromentTemp[i] = 0;
    }

    //UartFrame_Timeout_Calc();
}

static void UartFrame_Timeout_Calc(void)
{
    u8 i;
    float temp;//5���ֽڴ��������ʱ�䣬��λ10ns
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
/*��������  Delay_us                                                  */
/*����˵��������ʱ����λ��us                                          */
/*���������n��1000��Ϊ1ms                                                              */
/*�����������                                                              */
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
/*��������  Framework_Timer1ms                                              */
/*����˵����1ms��ʱ���ķ�������ÿ1msִ��һ��                                */
/*�����������                                                              */
/*�����������                                                              */
/***************************************************************************/
void Framework_Timer1ms(void)
{
	TaskIO_Timer1ms();
    TaskFreq_Timer1ms();
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
/*��������  Framework_Timer1ms                                              */
/*����˵����1ms��ʱ���ķ�������ÿ1msִ��һ��                                */
/*�����������                                                              */
/*�����������                                                              */
/***************************************************************************/
void Framework_Timer100ms(void)
{
    
}


/****************************************************************************/
/*��������  Calc_CurrentTemp                                                 */
/*����˵������ȡ��ǰ�¶�ֵ�Ľӿڣ�����ADֵ������¶ȣ���Ҫ��ȷ���£�
ÿ��оƬ����Ҫ�궨�����¶�ֵ.100ms�ɼ�һ��AD���ݣ�ȡ10���е�8������ƽ��*/
/*�����������                                                               */
/****************************************************************************/
void Calc_CurrentTemp(u16 sch_timer,u16 sch_cycle)
{
    u8 i;
    u16 max=0;
    u16 min=0xffff;
    u16 sum=0;
    EnviromentTemp[TempSample_Index] = GET_ADC_Result(TempSensor);
    if(TempSample_Index >= TEMP_SAMPLES)
    {
        for(i=0;i<TEMP_SAMPLES;i++)
        {
            if(max < EnviromentTemp[i])
            {
                max = EnviromentTemp[i];
            }
            if(min > EnviromentTemp[i])
            {
                min = EnviromentTemp[i];
            }
            sum = sum + EnviromentTemp[i];
        }
        sum = (sum - min - max)/(TEMP_SAMPLES - 2u);
        Global_Variable.CurrentEnvTemp = (1.42 - sum*3.3/4096)*1000/4.35 + 25;
        TempSample_Index = 0;
    }
    TempSample_Index++;
    
    //temp = (1.42 - (GET_ADC_Result(TempSensor)*3.3/4096)/(GET_ADC_Result(VrefInit)*3.3/1.2/4096))*1000/4.35 + 25;
}

/****************************************************************************/
/*��������  Calc_Power_5V                                                   */
/*����˵��������ADֵ���㵱ǰ��Դ��ѹ                                        */
/*�����������                                                              */
/*�����������                                                              */
/***************************************************************************/
void Calc_Power_5V(u16 sch_timer,u16 sch_cycle)
{
	Global_Variable.Power_5V =  GET_ADC_Result(Power_5V)*11/4096;
}

/****************************************************************************/
/*��������  Get_Rotary_Pulze                                                */
/*����˵������ȡ��ǰ����������ֵ                                            */
/*�����������                                                              */
/*�����������                                                              */
/****************************************************************************/
u16 Get_Rotary_Pulze(void)
{
    // ��ȡ��������Ϣ
    return TIM_GetCounter(TIM4);
}

/********************************************************************************/
/*��������  Calculate wire positi                                                   */
/*����˵���������������ȣ�ȷ��λ��                                                         */
/*���������p_arg                                                               */
/*�����������                                                                  */
/*******************************************************************************/
void Calculate_Wire_Position(u16 sch_timer,u16 sch_cycle)
{
    u16 EncodePulse;
    float wireLen;
    
    EncodePulse = Get_Rotary_Pulze();
    if(EncodePulse > PrePluse_Number )
    {//��ǰֵ�����ϴ�ֵ
        if((EncodePulse > 0xC000)&&(PrePluse_Number < 0x2000))
        {//��ת����ת������ֵ��ת
            Pulse_Total = Pulse_Total - (0xffff - EncodePulse + PrePluse_Number);
            Global_Variable.EncodePulse = Pulse_Total/4;
        }
        else
        {//������ת����ֵ�ۼ�δ��ת
            Pulse_Total = Pulse_Total + (EncodePulse - PrePluse_Number);
            Global_Variable.EncodePulse = Pulse_Total/4;
        }
    }
    else
    {//�ϴ�ֵ���ڵ�ǰֵ
        if((EncodePulse < 0x2000)&&(PrePluse_Number > 0xC000))
        {//������ת��ֵ��ת
            Pulse_Total = Pulse_Total + (0xffff - PrePluse_Number + EncodePulse);
            Global_Variable.EncodePulse = Pulse_Total/4;
        }
        else
        {//������ת����ֵ��Сδ��ת
            Pulse_Total = Pulse_Total - (PrePluse_Number - EncodePulse);
            Global_Variable.EncodePulse = Pulse_Total/4;
        }
    }    
    if(EncodePulse != PrePluse_Number)
    {
        Global_Variable.Wire_Position = Global_Variable.EncodePulse * 2 * 3.14 * ENCODER_RADIUS / PULSE_PER_CYCLE;
        PrePluse_Number = EncodePulse;
    }
}


/****************************************************************************/
/*��������  Package_Float                                                   */
/*����˵����������������ת�������飬�������ϴ�                              */
/*�����������                                                              */
/*�����������                                                              */
/****************************************************************************/
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
#ifdef W5500_ENABLE
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

