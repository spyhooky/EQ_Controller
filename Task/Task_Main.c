#include "main.h"
#include "Task_TCPIP.h"
#include "Task_ModbusTCp.h"
#include "Task_MQTT.h"
#include "Task_HTTP.h"
#include "Task_IO.h"
#include "Task_PC_Com.h"

//#include "Task_LED.h"
#include "transport.h"

#define STKSIZE_IO                     	 STK_SIZE_64
#define STKSIZE_PCMSG_RECV               STK_SIZE_1024
#define STKSIZE_FREQ_CONVER              STK_SIZE_1024
#define STKSIZE_ENCODER                  STK_SIZE_256
#define STKSIZE_TCPIP                    STK_SIZE_512
#define STKSIZE_MBTCP                    STK_SIZE_256
#define STKSIZE_MQTT                     STK_SIZE_1024                   
#define STKSIZE_HTTP                     STK_SIZE_512
#define STKSIZE_BACKGRD                  STK_SIZE_1024
#define STKSIZE_ETHERNET                 STK_SIZE_32



typedef  void (*FunVoidType)(void);
#define ApplicationMsp          0x8008000 
#define ApplicationVect         (ApplicationMsp+4)

OS_STK STK_IO[STKSIZE_IO];
OS_STK STK_PCMSG_RECV[STKSIZE_PCMSG_RECV];
OS_STK STK_FREQ_CONVER[STKSIZE_FREQ_CONVER];
OS_STK STK_ENCODER[STKSIZE_ENCODER];

#ifdef W5500_ENABLE
OS_STK STK_ETHERNET[STKSIZE_ETHERNET];
#endif
#ifdef TCPIP_ENABLE
OS_STK STK_TCPIP[STKSIZE_TCPIP];
#endif
#ifdef MBTCP_ENABLE
OS_STK STK_MBTCP[STKSIZE_MBTCP];
#endif
#ifdef MQTT_ENABLE
OS_STK STK_MQTT[STKSIZE_MQTT];
#endif
#ifdef HTTP_ENABLE
OS_STK STK_HTTP[STKSIZE_HTTP];
#endif

OS_STK STK_BACKGRD[STKSIZE_BACKGRD];

static u16 Background_Timer;
static u16 DetectTimer_W5500; //���ڼ��W5500״̬

void f_GenSoftwareReset(void);
void Task_BackGround(void *p_arg);
void Task_Ethernet(void *p_arg);


// #pragma arm section code=".ARM.__at_0x08000000"
// void f_JumpAppl(void)
// {
//     u32   m_JumpAddress;	
//     FunVoidType JumpToApplication;          /* call this function to jump to appl */
//     if (((*(vu32*)ApplicationMsp) & 0x2FFE0000 ) == 0x20000000)/*MSP check*/
//     {
//         /* Jump to user application */
//         m_JumpAddress = *(vu32*) (ApplicationVect);
//         JumpToApplication = (FunVoidType) m_JumpAddress;
//         /* Initialize user application's Stack Pointer */
//         __MSR_MSP(*(vu32*) ApplicationMsp);
//         JumpToApplication();	 
//         while(1);	  
//     }
// }
// #pragma arm section

/****************************************************************************/
/*��������  Task_Main                                                       */
/*����˵����ucos���������еĵ�һ��task���������Ϊ��ʼ����task����Ҫ���ڴ�������*/
/*          task���ʼ����ش���                                             */
/*���������p_arg������Ŀ�ϸò���Ϊ��                                         */
/*�����������                                                              */
/***************************************************************************/
void Config_DrCommon(void)
{
    //RCC_DeInit();
    
    //NVIC_DeInit();
    
    ReadFlashCfg();//��FLASH����
    PERIPH_CLOCK_IO_ALL_ENABLE;//ʹ��IO��ص�ʱ����
    DrGpioInit();
	DrAdc();//��ʼ��AD�Ĵ���
    DrTimer_Init();//��ʼ����ʱ��
    DrUsart_Init();//��ʼ��uartģ��
    DrCAN_Init(); //��ʼ��CAN
        
    /* �������Ź���ʱ��=64/40k*312=500ms */
    
}


/****************************************************************************/
/*��������  Task_Main                                                       */
/*����˵����ucos���������еĵ�һ��task���������Ϊ��ʼ����task����Ҫ���ڴ�������*/
/*          task���ʼ����ش���                                             */
/*���������p_arg������Ŀ�ϸò���Ϊ��                                         */
/*�����������                                                              */
/***************************************************************************/

void Task_Main(void *p_arg)
{
    
    (void)p_arg;
    u8 Ethernet_Init_Flag = FALSE;
    Config_DrCommon();

    //����IO��task������Ĳ���Ϊ��Ŀ������Ϣ
	OSTaskCreate(Task_IO, (void *)&gWIZNETINFO, (OS_STK*)&STK_IO[STKSIZE_IO-1], TASK_PRIO_IO);
	//������PCͨѶ��task������Ĳ���Ϊ��Ŀ������Ϣ
    OSTaskCreate(Task_PC_Message_Recv, (void *)&gWIZNETINFO, (OS_STK*)&STK_PCMSG_RECV[STKSIZE_PCMSG_RECV-1], TASK_PRIO_PCMSG_RECV);
    //������Ƶ��ͭ���task������Ĳ���Ϊ��Ŀ������Ϣ
    //OSTaskCreate(Task_Freq_Convert, (void *)&gWIZNETINFO, (OS_STK*)&STK_FREQ_CONVER[STKSIZE_FREQ_CONVERT-1], TASK_PRIO_FREQ_CONVERT);
    //������������task������Ĳ���Ϊ��Ŀ������Ϣ
    //OSTaskCreate(Task_Encoder, (void *)&gWIZNETINFO, (OS_STK*)&STK_ENCODER[STKSIZE_ENCODER-1], TASK_PRIO_ENCODER);
    OSTaskCreate(Task_BackGround, (void *)&gWIZNETINFO, (OS_STK*)&STK_BACKGRD[STKSIZE_BACKGRD-1], TASK_PRIO_BACKGRD);//������̨task��������ѯ��״̬�ģ���ʱ��Ҫ���Ƿǳ��ߵĹ��ܣ��ɷ��ڴ�task��ִ��
       
    //OSTaskSuspend(TASK_PRIO_MAIN);//�����task
    while (1)
    {
        #ifdef W5500_ENABLE
        if(Ethernet_Init_Flag == FALSE)
        {
            W5500_SPI_Config();//W5500 SPI��ʼ��
            Ethernet_Init();//��̫����ʼ��
            #ifdef TCPIP_ENABLE
            if((gWIZNETINFO.session_mode==S_tcpip_client)||(gWIZNETINFO.session_mode==S_tcpip_server))
            {
                OSTaskCreate(Task_TCPIP, (void *)&gWIZNETINFO, (OS_STK*)&STK_TCPIP[STKSIZE_TCPIP-1], TASK_PRIO_TCPIP);//����TCPIP�����񣬴���Ĳ���Ϊ��Ŀ����
            }
            #endif
            #ifdef MBTCP_ENABLE
            if((gWIZNETINFO.session_mode==S_mb_client)||(gWIZNETINFO.session_mode==S_mb_server))
            {
                OSTaskCreate(Task_ModbusTCP, (void *)&gWIZNETINFO, (OS_STK*)&STK_MBTCP[STKSIZE_MBTCP-1], TASK_PRIO_MBTCP);//����MODBUS TCP���񣬴���Ĳ���Ϊ��Ŀ����
            }
            #endif
            
            #ifdef MQTT_ENABLE
            if(gWIZNETINFO.session_mode==S_mqtt)
            {
                OSTaskCreate(MQTT_task, (void *)&gWIZNETINFO, &STK_MQTT[STKSIZE_MQTT-1], TASK_PRIO_MQTT); //����MQTT������Ĳ���Ϊ��Ŀ����
            }
            #endif
            
            #ifdef HTTP_ENABLE
            OSTaskCreate(Task_HTTP, (void *)&gWIZNETINFO, (OS_STK*)&STK_HTTP[STKSIZE_HTTP-1], TASK_PRIO_HTTP);//������ҳͨѶ���񣬴���Ĳ���Ϊ��Ŀ����
         	#endif
            Ethernet_Init_Flag = TRUE;
        }
        else
        {
            if(DetectTimer_W5500 >= DETECT_CYCLE)
            {
                DetectTimer_W5500 = 0;
                W5500_StatusDetect();//W5500״̬���
            }
        }
        #endif
        //������ȼ��������Ϊ��ͳ�������main���while��ʼ��ѭ��ִ�У������˳�������task�Ͷ�ʱ��������ռ�ó���
        
    }
}


/****************************************************************************/
/*��������  Task_BackGround                                                  */
/*����˵������task���ó�1ms���ڵĳ���*/
/*          task���ʼ����ش���                                             */
/*���������p_arg����Ŀ������ò���                                         */
/*�����������                                                              */
/***************************************************************************/
void Task_BackGround(void *p_arg)
{
    #ifdef WATCHDOG_ENABLE
        IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
        IWDG_SetPrescaler(IWDG_Prescaler_256);//ʱ�ӷ�Ƶ40k/256=156hz��6.4ms��
        IWDG_SetReload(468);//���Ź���ʱʱ��Ϊ3s ���ܴ���0xfff��4095�� 781Ϊ5s
        IWDG_ReloadCounter();
        IWDG_Enable();//������Ź�ʹ��
    #endif

    while(1)
    {
        IWDG_ReloadCounter();//ι��
        
		//UART_CAN_Handler(p_arg);//��ʱ��ʹ�øù���
		
		/******************APPLICATION START******************/
        
		ReadADCAverageValue(Background_Timer,CYCLE_READ_ADC_VALUE);//AD����ֵ��ֵ����
		Calc_CurrentTemp(Background_Timer,CYCLE_CALC_ENV_TEMP);//���㵱ǰ�¶�ֵ
        Calc_Power_5V(Background_Timer,CYCLE_CALC_POWER_VOL);//���㵱ǰ5V��ѹֵ
		
		/******************APPLICATION END******************/
        OSTimeDlyHMSM(0, 0, 0, CYCLE_BACKGROUND);//�ز����٣��������ȼ��͵�����ò������У�ʱ��Ϊ1ms
		Background_Timer++;
        DetectTimer_W5500++;
    }
}

/****************************************************************************/
/*��������  ETH2Usartcan_send                                                  */
/*����˵������̫������ת���ڻ�CAN*/
/*�����������                                                              */
/*�����������                                                              */
/***************************************************************************/
//�ú�����ʱ��ʹ��
void ETH2Usartcan_send(u8 uartcan_chn,u8 *databuf,u16 lenth)
{
    u16 canid;
    OS_ENTER_CRITICAL();
    switch(uartcan_chn)
    {
        case RS232_1:
            if(USARTCAN.Usart[RS232_1][EnUart]==ON)//�ö˿ڱ������ó�ʹ�ܲŻ����ת�������򱨴�
            {    
                USART1_Send_Data(databuf,lenth);
            }
            else
            {
                //LED_BLINK_ONCE(S_FAULT);
                //errorcode
            }
            break;
        case RS485_1:
            if(USARTCAN.Usart[RS485_1][EnUart]==ON)//�ö˿ڱ������ó�ʹ�ܲŻ����ת�������򱨴�
            {
                USART2_Send_Data(databuf,lenth);
            }
            else
            {
                //LED_BLINK_ONCE(S_FAULT);
                //errorcode
            }
            break;
		case RS485_2:
            if(USARTCAN.Usart[RS485_2][EnUart]==ON)//�ö˿ڱ������ó�ʹ�ܲŻ����ת�������򱨴�
            {
                USART3_Send_Data(databuf,lenth);
            }
            else
            {
                //LED_BLINK_ONCE(S_FAULT);
                //errorcode
            }
            break;
        case RS485_3:
            if(USARTCAN.Usart[RS485_3][EnUart]==ON)//�ö˿ڱ������ó�ʹ�ܲŻ����ת�������򱨴�
            {
                UART4_Send_Data(databuf,lenth);
            }
            else
            {
                //LED_BLINK_ONCE(S_FAULT);
                //errorcode
            }
            break;
		case RS485_4:
            if(USARTCAN.Usart[RS485_4][EnUart]==ON)//�ö˿ڱ������ó�ʹ�ܲŻ����ת�������򱨴�
            {
                UART5_Send_Data(databuf,lenth);
            }
            else
            {
                //LED_BLINK_ONCE(S_FAULT);
                //errorcode
            }
            break;
        case CAN_CHN:
            if(USARTCAN.can[EnCAN]==ON)//�ö˿ڱ������ó�ʹ�ܲŻ����ת�������򱨴�
            {
                if(lenth<=8)
                {
                    canid = (databuf[0]<<8) + (databuf[1]);
                    if(CAN1_SendData(canid,&databuf[2],lenth)!=ON)
                    {
                        //LED_BLINK_ONCE(S_FAULT);
                        //errorcode
                    }
                }
                else
                {
                    //LED_BLINK_ONCE(S_FAULT);
                    //errorcode
                }
            }
            else
            {
                //LED_BLINK_ONCE(S_FAULT);
                //errorcode
            }
            break;
        default:
            //LED_BLINK_ONCE(S_FAULT);
            //errorcode
            break;
    }
    OS_EXIT_CRITICAL();
    //LED_BLINK_ONCE(S_NORMAL);
}


/****************************************************************************/
/*��������  ReadFlashCfg                                                    */
/*����˵������flash�����ݣ�˳�����͵�ַ���ñ��һ�£��������ִ���           */
/*�����������                                                              */
/*�����������                                                              */
/***************************************************************************/
void ReadFlashCfg(void)
{
    struct EthernetCfg_t *readdata;
    u8 flashdata[sizeof(struct EthernetCfg_t)];
    FlashReadData(CONFIG_BASEADDR,flashdata,sizeof(struct EthernetCfg_t));
    readdata = (struct EthernetCfg_t *)&flashdata;
    
    if((readdata->cfgflag[0]==0xAA)&&(readdata->cfgflag[1]==0x55)&&(readdata->cfgflag[2]==0xAA)&&(readdata->cfgflag[3]==0x55))
    {
   
    }
    else
    {
        memcpy(&gWIZNETINFO.mac[3],&gWIZNETINFO.iplocal[1],3);//mac����ip�仯
        return;
    }
    
    if(readdata->session_mode > S_boundary)
    {
        memcpy(&gWIZNETINFO.mac[3],&gWIZNETINFO.iplocal[1],3);//mac����ip�仯
        return;
    }
    gWIZNETINFO.session_mode = readdata->session_mode;
    gWIZNETINFO.stationID = readdata->stationID;
    //Ԥ��1
    gWIZNETINFO.mbtcp_addr = readdata->mbtcp_addr;
    USARTCAN.datalen = readdata->mbtcp_datalen;
    gWIZNETINFO.polltime = readdata->polltime;
    //Ԥ��2
    
    memcpy(gWIZNETINFO.iplocal,readdata->localIP,4);
    gWIZNETINFO.portlocal = readdata->localport;
    memcpy(&gWIZNETINFO.mac[3],&gWIZNETINFO.iplocal[1],3);//mac����ip�仯
    //Ԥ��3
    memcpy(gWIZNETINFO.ipgoal,readdata->remoteIP,4);
    gWIZNETINFO.portgoal = readdata->remoteport;
    //Ԥ��4
    memcpy(gWIZNETINFO.sn,readdata->submask,4);
    memcpy(gWIZNETINFO.gw,readdata->gatewayaddr,4);
    //Ԥ��5
    
    USARTCAN.can[EnCAN] = readdata->can_en;
    USARTCAN.can[canBaudrate] = readdata->canbaudrate;
    USARTCAN.can[LocalID] = readdata->can_localID;
    USARTCAN.can[DeviceID] = readdata->can_deviceID;
    USARTCAN.can[IDNum] = readdata->can_device_num;
    USARTCAN.can[canDatatype] = readdata->can_datatype;
    //Ԥ��6
    
    USARTCAN.Usart[RS232_1][EnUart] = readdata->rs232_1_en;
    USARTCAN.Usart[RS232_1][uartBaudrate] = readdata->rs232_1_baudrate;
    USARTCAN.Usart[RS232_1][Databits] = readdata->rs232_1_databit;
    USARTCAN.Usart[RS232_1][Chkbits] = readdata->rs232_1_chkbit;
    USARTCAN.Usart[RS232_1][Stopbits] = readdata->rs232_1_stopbit;
    USARTCAN.Usart[RS232_1][Flowctrl] = readdata->rs232_1_flowctrl;
    USARTCAN.Usart[RS232_1][uartDatatype] = readdata->rs232_1_datatype;
    //Ԥ��7
    
    USARTCAN.Usart[RS485_1][EnUart] = readdata->rs485_1_en;
    USARTCAN.Usart[RS485_1][uartBaudrate] = readdata->rs485_1_baudrate;
    USARTCAN.Usart[RS485_1][Databits] = readdata->rs485_1_databit;
    USARTCAN.Usart[RS485_1][Chkbits] = readdata->rs485_1_chkbit;
    USARTCAN.Usart[RS485_1][Stopbits] = readdata->rs485_1_stopbit;
    USARTCAN.Usart[RS485_1][uartDatatype] = readdata->rs485_1_datatype;
    //Ԥ��8
    
    USARTCAN.Usart[RS485_2][EnUart] = readdata->rs485_2_en;
    USARTCAN.Usart[RS485_2][uartBaudrate] = readdata->rs485_2_baudrate;
    USARTCAN.Usart[RS485_2][Databits] = readdata->rs485_2_databit;
    USARTCAN.Usart[RS485_2][Chkbits] = readdata->rs485_2_chkbit;
    USARTCAN.Usart[RS485_2][Stopbits] = readdata->rs485_2_stopbit;
    USARTCAN.Usart[RS485_2][uartDatatype] = readdata->rs485_2_datatype;
    //Ԥ��9
	
	USARTCAN.Usart[RS485_3][EnUart] = readdata->rs485_3_en;
    USARTCAN.Usart[RS485_3][uartBaudrate] = readdata->rs485_3_baudrate;
    USARTCAN.Usart[RS485_3][Databits] = readdata->rs485_3_databit;
    USARTCAN.Usart[RS485_3][Chkbits] = readdata->rs485_3_chkbit;
    USARTCAN.Usart[RS485_3][Stopbits] = readdata->rs485_3_stopbit;
    USARTCAN.Usart[RS485_3][uartDatatype] = readdata->rs485_3_datatype;
    //Ԥ��10
    
    USARTCAN.Usart[RS485_4][EnUart] = readdata->rs485_4_en;
    USARTCAN.Usart[RS485_4][uartBaudrate] = readdata->rs485_4_baudrate;
    USARTCAN.Usart[RS485_4][Databits] = readdata->rs485_4_databit;
    USARTCAN.Usart[RS485_4][Chkbits] = readdata->rs485_4_chkbit;
    USARTCAN.Usart[RS485_4][Stopbits] = readdata->rs485_4_stopbit;
    USARTCAN.Usart[RS485_4][uartDatatype] = readdata->rs485_4_datatype;
    //Ԥ��11
    
    USARTCAN.tout = readdata->to_thres;
    //Ԥ��12
}

/*****************************************/
void dylms(unsigned int u)
{
		unsigned int x=0;
		unsigned int y=0;
		for(y=0;y<u;y++)
			for(x=0;x<10000;x++);
}

/****************************************************************************/
/*��������  HTTP_DataHandler                                                 */
/*����˵����WEB���ݴ���                                                      */
/*���������p_arg������Ŀ�ϸò���Ϊ��                                         */
/*�����������                                                               */
/****************************************************************************/
/**********************************/
void HTTP_DataHandler(void)
{
    struct EthernetCfg_t flashdata;
    flashdata.cfgflag[0] = 0xAA;
    flashdata.cfgflag[1] = 0x55;
    flashdata.cfgflag[2] = 0xAA;
    flashdata.cfgflag[3] = 0x55;
    flashdata.session_mode=gWIZNETINFO.session_mode;
    flashdata.stationID=gWIZNETINFO.stationID;
    //Ԥ��1
    flashdata.mbtcp_addr=gWIZNETINFO.mbtcp_addr;
    flashdata.mbtcp_datalen=USARTCAN.datalen;
    flashdata.polltime=gWIZNETINFO.polltime;
    //Ԥ��2
    
    memcpy(flashdata.localIP,gWIZNETINFO.iplocal,4);
    flashdata.localport=gWIZNETINFO.portlocal;
    //Ԥ��3
    memcpy(flashdata.remoteIP,gWIZNETINFO.ipgoal,4);
    flashdata.remoteport=gWIZNETINFO.portgoal;
    //Ԥ��4
    memcpy(flashdata.submask,gWIZNETINFO.sn,4);
    memcpy(flashdata.gatewayaddr,gWIZNETINFO.gw,4);
    //Ԥ��5
    
    flashdata.can_en = USARTCAN.can[EnCAN];
    flashdata.canbaudrate = USARTCAN.can[canBaudrate];
    flashdata.can_localID = USARTCAN.can[LocalID];
    flashdata.can_deviceID = USARTCAN.can[DeviceID];
    flashdata.can_device_num = USARTCAN.can[IDNum];
    flashdata.can_datatype = USARTCAN.can[canDatatype];
    //Ԥ��6
    
    flashdata.rs232_1_en = USARTCAN.Usart[RS232_1][EnUart];
    flashdata.rs232_1_baudrate = USARTCAN.Usart[RS232_1][uartBaudrate];
    flashdata.rs232_1_databit = USARTCAN.Usart[RS232_1][Databits];
    flashdata.rs232_1_chkbit = USARTCAN.Usart[RS232_1][Chkbits];
    flashdata.rs232_1_stopbit = USARTCAN.Usart[RS232_1][Stopbits];
    flashdata.rs232_1_flowctrl = USARTCAN.Usart[RS232_1][Flowctrl];
    flashdata.rs232_1_datatype = USARTCAN.Usart[RS232_1][uartDatatype];
    //Ԥ��7
    
	flashdata.rs485_1_en = USARTCAN.Usart[RS485_1][EnUart];
    flashdata.rs485_1_baudrate = USARTCAN.Usart[RS485_1][uartBaudrate];
    flashdata.rs485_1_databit = USARTCAN.Usart[RS485_1][Databits];
    flashdata.rs485_1_chkbit = USARTCAN.Usart[RS485_1][Chkbits];
    flashdata.rs485_1_stopbit = USARTCAN.Usart[RS485_1][Stopbits];
    flashdata.rs485_1_datatype = USARTCAN.Usart[RS485_1][uartDatatype];
    //Ԥ��8
    
    flashdata.rs485_2_en = USARTCAN.Usart[RS485_2][EnUart];
    flashdata.rs485_2_baudrate = USARTCAN.Usart[RS485_2][uartBaudrate];
    flashdata.rs485_2_databit = USARTCAN.Usart[RS485_2][Databits];
    flashdata.rs485_2_chkbit = USARTCAN.Usart[RS485_2][Chkbits];
    flashdata.rs485_2_stopbit = USARTCAN.Usart[RS485_2][Stopbits];
    flashdata.rs485_2_datatype = USARTCAN.Usart[RS485_2][uartDatatype];
    //Ԥ��9
	
	flashdata.rs485_3_en = USARTCAN.Usart[RS485_3][EnUart];
    flashdata.rs485_3_baudrate = USARTCAN.Usart[RS485_3][uartBaudrate];
    flashdata.rs485_3_databit = USARTCAN.Usart[RS485_3][Databits];
    flashdata.rs485_3_chkbit = USARTCAN.Usart[RS485_3][Chkbits];
    flashdata.rs485_3_stopbit = USARTCAN.Usart[RS485_3][Stopbits];
    flashdata.rs485_3_datatype = USARTCAN.Usart[RS485_3][uartDatatype];
    //Ԥ��10
    
    flashdata.rs485_4_en = USARTCAN.Usart[RS485_4][EnUart];
    flashdata.rs485_4_baudrate = USARTCAN.Usart[RS485_4][uartBaudrate];
    flashdata.rs485_4_databit = USARTCAN.Usart[RS485_4][Databits];
    flashdata.rs485_4_chkbit = USARTCAN.Usart[RS485_4][Chkbits];
    flashdata.rs485_4_stopbit = USARTCAN.Usart[RS485_4][Stopbits];
    flashdata.rs485_4_datatype = USARTCAN.Usart[RS485_4][uartDatatype];
    //Ԥ��11
    
    flashdata.to_thres = USARTCAN.tout;
    //Ԥ��12
    
    OS_ENTER_CRITICAL();
    FlashErase(CONFIG_BASEADDR,1);
    FlashWriteData(CONFIG_BASEADDR,(u8 *)&flashdata,sizeof(flashdata));
    OS_EXIT_CRITICAL();
    dylms(500);
    f_GenSoftwareReset();//flash���º�������λ�������µ���������
}

//�����λ
void f_GenSoftwareReset(void)
{
    __set_FAULTMASK(1);
	SCB->AIRCR = AIRCR_VECTKEY_MASK | (u32)0x04;
}

