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
static u16 DetectTimer_W5500; //用于检测W5500状态

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
/*函数名：  Task_Main                                                       */
/*功能说明：ucos启动后运行的第一个task，可以理解为初始化的task，主要用于创建其他*/
/*          task或初始化相关代码                                             */
/*输入参数：p_arg，本项目上该参数为空                                         */
/*输出参数：无                                                              */
/***************************************************************************/
void Config_DrCommon(void)
{
    //RCC_DeInit();
    
    //NVIC_DeInit();
    
    ReadFlashCfg();//读FLASH数据
    PERIPH_CLOCK_IO_ALL_ENABLE;//使能IO相关的时钟域
    DrGpioInit();
	DrAdc();//初始化AD寄存器
    DrTimer_Init();//初始化定时器
    DrUsart_Init();//初始化uart模块
    DrCAN_Init(); //初始化CAN
        
    /* 独立看门狗，时间=64/40k*312=500ms */
    
}


/****************************************************************************/
/*函数名：  Task_Main                                                       */
/*功能说明：ucos启动后运行的第一个task，可以理解为初始化的task，主要用于创建其他*/
/*          task或初始化相关代码                                             */
/*输入参数：p_arg，本项目上该参数为空                                         */
/*输出参数：无                                                              */
/***************************************************************************/

void Task_Main(void *p_arg)
{
    
    (void)p_arg;
    u8 Ethernet_Init_Flag = FALSE;
    Config_DrCommon();

    //创建IO的task，传入的参数为项目配置信息
	OSTaskCreate(Task_IO, (void *)&gWIZNETINFO, (OS_STK*)&STK_IO[STKSIZE_IO-1], TASK_PRIO_IO);
	//创建和PC通讯的task，传入的参数为项目配置信息
    OSTaskCreate(Task_PC_Message_Recv, (void *)&gWIZNETINFO, (OS_STK*)&STK_PCMSG_RECV[STKSIZE_PCMSG_RECV-1], TASK_PRIO_PCMSG_RECV);
    //创建变频器铜须的task，传入的参数为项目配置信息
    //OSTaskCreate(Task_Freq_Convert, (void *)&gWIZNETINFO, (OS_STK*)&STK_FREQ_CONVER[STKSIZE_FREQ_CONVERT-1], TASK_PRIO_FREQ_CONVERT);
    //创建编码器的task，传入的参数为项目配置信息
    //OSTaskCreate(Task_Encoder, (void *)&gWIZNETINFO, (OS_STK*)&STK_ENCODER[STKSIZE_ENCODER-1], TASK_PRIO_ENCODER);
    OSTaskCreate(Task_BackGround, (void *)&gWIZNETINFO, (OS_STK*)&STK_BACKGRD[STKSIZE_BACKGRD-1], TASK_PRIO_BACKGRD);//创建后台task，用于轮询查状态的，对时间要求不是非常高的功能，可放在此task内执行
       
    //OSTaskSuspend(TASK_PRIO_MAIN);//挂起该task
    while (1)
    {
        #ifdef W5500_ENABLE
        if(Ethernet_Init_Flag == FALSE)
        {
            W5500_SPI_Config();//W5500 SPI初始化
            Ethernet_Init();//以太网初始化
            #ifdef TCPIP_ENABLE
            if((gWIZNETINFO.session_mode==S_tcpip_client)||(gWIZNETINFO.session_mode==S_tcpip_server))
            {
                OSTaskCreate(Task_TCPIP, (void *)&gWIZNETINFO, (OS_STK*)&STK_TCPIP[STKSIZE_TCPIP-1], TASK_PRIO_TCPIP);//创建TCPIP的任务，传入的参数为项目配置
            }
            #endif
            #ifdef MBTCP_ENABLE
            if((gWIZNETINFO.session_mode==S_mb_client)||(gWIZNETINFO.session_mode==S_mb_server))
            {
                OSTaskCreate(Task_ModbusTCP, (void *)&gWIZNETINFO, (OS_STK*)&STK_MBTCP[STKSIZE_MBTCP-1], TASK_PRIO_MBTCP);//创建MODBUS TCP任务，传入的参数为项目配置
            }
            #endif
            
            #ifdef MQTT_ENABLE
            if(gWIZNETINFO.session_mode==S_mqtt)
            {
                OSTaskCreate(MQTT_task, (void *)&gWIZNETINFO, &STK_MQTT[STKSIZE_MQTT-1], TASK_PRIO_MQTT); //创建MQTT，传入的参数为项目配置
            }
            #endif
            
            #ifdef HTTP_ENABLE
            OSTaskCreate(Task_HTTP, (void *)&gWIZNETINFO, (OS_STK*)&STK_HTTP[STKSIZE_HTTP-1], TASK_PRIO_HTTP);//创建网页通讯任务，传入的参数为项目配置
         	#endif
            Ethernet_Init_Flag = TRUE;
        }
        else
        {
            if(DetectTimer_W5500 >= DETECT_CYCLE)
            {
                DetectTimer_W5500 = 0;
                W5500_StatusDetect();//W5500状态监测
            }
        }
        #endif
        //最低优先级，可理解为传统裸机程序main里的while，始终循环执行，永不退出，其他task和定时器均能抢占该程序
        
    }
}


/****************************************************************************/
/*函数名：  Task_BackGround                                                  */
/*功能说明：该task配置成1ms周期的程序，*/
/*          task或初始化相关代码                                             */
/*输入参数：p_arg，项目相关配置参数                                         */
/*输出参数：无                                                              */
/***************************************************************************/
void Task_BackGround(void *p_arg)
{
    #ifdef WATCHDOG_ENABLE
        IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
        IWDG_SetPrescaler(IWDG_Prescaler_256);//时钟分频40k/256=156hz（6.4ms）
        IWDG_SetReload(468);//看门狗超时时间为3s 不能大于0xfff（4095） 781为5s
        IWDG_ReloadCounter();
        IWDG_Enable();//软件看门狗使能
    #endif

    while(1)
    {
        IWDG_ReloadCounter();//喂狗
        
		//UART_CAN_Handler(p_arg);//暂时不使用该功能
		
		/******************APPLICATION START******************/
        
		ReadADCAverageValue(Background_Timer,CYCLE_READ_ADC_VALUE);//AD采样值均值计算
		Calc_CurrentTemp(Background_Timer,CYCLE_CALC_ENV_TEMP);//计算当前温度值
        Calc_Power_5V(Background_Timer,CYCLE_CALC_POWER_VOL);//计算当前5V电压值
		
		/******************APPLICATION END******************/
        OSTimeDlyHMSM(0, 0, 0, CYCLE_BACKGROUND);//必不可少，否则优先级低的任务得不到运行，时间为1ms
		Background_Timer++;
        DetectTimer_W5500++;
    }
}

/****************************************************************************/
/*函数名：  ETH2Usartcan_send                                                  */
/*功能说明：以太网数据转串口或CAN*/
/*输入参数：无                                                              */
/*输出参数：无                                                              */
/***************************************************************************/
//该函数暂时不使用
void ETH2Usartcan_send(u8 uartcan_chn,u8 *databuf,u16 lenth)
{
    u16 canid;
    OS_ENTER_CRITICAL();
    switch(uartcan_chn)
    {
        case RS232_1:
            if(USARTCAN.Usart[RS232_1][EnUart]==ON)//该端口必须配置成使能才会进行转换，否则报错
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
            if(USARTCAN.Usart[RS485_1][EnUart]==ON)//该端口必须配置成使能才会进行转换，否则报错
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
            if(USARTCAN.Usart[RS485_2][EnUart]==ON)//该端口必须配置成使能才会进行转换，否则报错
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
            if(USARTCAN.Usart[RS485_3][EnUart]==ON)//该端口必须配置成使能才会进行转换，否则报错
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
            if(USARTCAN.Usart[RS485_4][EnUart]==ON)//该端口必须配置成使能才会进行转换，否则报错
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
            if(USARTCAN.can[EnCAN]==ON)//该端口必须配置成使能才会进行转换，否则报错
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
/*函数名：  ReadFlashCfg                                                    */
/*功能说明：读flash区数据，顺序必须和地址配置表的一致，否则会出现错序           */
/*输入参数：无                                                              */
/*输出参数：无                                                              */
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
        memcpy(&gWIZNETINFO.mac[3],&gWIZNETINFO.iplocal[1],3);//mac跟随ip变化
        return;
    }
    
    if(readdata->session_mode > S_boundary)
    {
        memcpy(&gWIZNETINFO.mac[3],&gWIZNETINFO.iplocal[1],3);//mac跟随ip变化
        return;
    }
    gWIZNETINFO.session_mode = readdata->session_mode;
    gWIZNETINFO.stationID = readdata->stationID;
    //预留1
    gWIZNETINFO.mbtcp_addr = readdata->mbtcp_addr;
    USARTCAN.datalen = readdata->mbtcp_datalen;
    gWIZNETINFO.polltime = readdata->polltime;
    //预留2
    
    memcpy(gWIZNETINFO.iplocal,readdata->localIP,4);
    gWIZNETINFO.portlocal = readdata->localport;
    memcpy(&gWIZNETINFO.mac[3],&gWIZNETINFO.iplocal[1],3);//mac跟随ip变化
    //预留3
    memcpy(gWIZNETINFO.ipgoal,readdata->remoteIP,4);
    gWIZNETINFO.portgoal = readdata->remoteport;
    //预留4
    memcpy(gWIZNETINFO.sn,readdata->submask,4);
    memcpy(gWIZNETINFO.gw,readdata->gatewayaddr,4);
    //预留5
    
    USARTCAN.can[EnCAN] = readdata->can_en;
    USARTCAN.can[canBaudrate] = readdata->canbaudrate;
    USARTCAN.can[LocalID] = readdata->can_localID;
    USARTCAN.can[DeviceID] = readdata->can_deviceID;
    USARTCAN.can[IDNum] = readdata->can_device_num;
    USARTCAN.can[canDatatype] = readdata->can_datatype;
    //预留6
    
    USARTCAN.Usart[RS232_1][EnUart] = readdata->rs232_1_en;
    USARTCAN.Usart[RS232_1][uartBaudrate] = readdata->rs232_1_baudrate;
    USARTCAN.Usart[RS232_1][Databits] = readdata->rs232_1_databit;
    USARTCAN.Usart[RS232_1][Chkbits] = readdata->rs232_1_chkbit;
    USARTCAN.Usart[RS232_1][Stopbits] = readdata->rs232_1_stopbit;
    USARTCAN.Usart[RS232_1][Flowctrl] = readdata->rs232_1_flowctrl;
    USARTCAN.Usart[RS232_1][uartDatatype] = readdata->rs232_1_datatype;
    //预留7
    
    USARTCAN.Usart[RS485_1][EnUart] = readdata->rs485_1_en;
    USARTCAN.Usart[RS485_1][uartBaudrate] = readdata->rs485_1_baudrate;
    USARTCAN.Usart[RS485_1][Databits] = readdata->rs485_1_databit;
    USARTCAN.Usart[RS485_1][Chkbits] = readdata->rs485_1_chkbit;
    USARTCAN.Usart[RS485_1][Stopbits] = readdata->rs485_1_stopbit;
    USARTCAN.Usart[RS485_1][uartDatatype] = readdata->rs485_1_datatype;
    //预留8
    
    USARTCAN.Usart[RS485_2][EnUart] = readdata->rs485_2_en;
    USARTCAN.Usart[RS485_2][uartBaudrate] = readdata->rs485_2_baudrate;
    USARTCAN.Usart[RS485_2][Databits] = readdata->rs485_2_databit;
    USARTCAN.Usart[RS485_2][Chkbits] = readdata->rs485_2_chkbit;
    USARTCAN.Usart[RS485_2][Stopbits] = readdata->rs485_2_stopbit;
    USARTCAN.Usart[RS485_2][uartDatatype] = readdata->rs485_2_datatype;
    //预留9
	
	USARTCAN.Usart[RS485_3][EnUart] = readdata->rs485_3_en;
    USARTCAN.Usart[RS485_3][uartBaudrate] = readdata->rs485_3_baudrate;
    USARTCAN.Usart[RS485_3][Databits] = readdata->rs485_3_databit;
    USARTCAN.Usart[RS485_3][Chkbits] = readdata->rs485_3_chkbit;
    USARTCAN.Usart[RS485_3][Stopbits] = readdata->rs485_3_stopbit;
    USARTCAN.Usart[RS485_3][uartDatatype] = readdata->rs485_3_datatype;
    //预留10
    
    USARTCAN.Usart[RS485_4][EnUart] = readdata->rs485_4_en;
    USARTCAN.Usart[RS485_4][uartBaudrate] = readdata->rs485_4_baudrate;
    USARTCAN.Usart[RS485_4][Databits] = readdata->rs485_4_databit;
    USARTCAN.Usart[RS485_4][Chkbits] = readdata->rs485_4_chkbit;
    USARTCAN.Usart[RS485_4][Stopbits] = readdata->rs485_4_stopbit;
    USARTCAN.Usart[RS485_4][uartDatatype] = readdata->rs485_4_datatype;
    //预留11
    
    USARTCAN.tout = readdata->to_thres;
    //预留12
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
/*函数名：  HTTP_DataHandler                                                 */
/*功能说明：WEB数据处理                                                      */
/*输入参数：p_arg，本项目上该参数为空                                         */
/*输出参数：无                                                               */
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
    //预留1
    flashdata.mbtcp_addr=gWIZNETINFO.mbtcp_addr;
    flashdata.mbtcp_datalen=USARTCAN.datalen;
    flashdata.polltime=gWIZNETINFO.polltime;
    //预留2
    
    memcpy(flashdata.localIP,gWIZNETINFO.iplocal,4);
    flashdata.localport=gWIZNETINFO.portlocal;
    //预留3
    memcpy(flashdata.remoteIP,gWIZNETINFO.ipgoal,4);
    flashdata.remoteport=gWIZNETINFO.portgoal;
    //预留4
    memcpy(flashdata.submask,gWIZNETINFO.sn,4);
    memcpy(flashdata.gatewayaddr,gWIZNETINFO.gw,4);
    //预留5
    
    flashdata.can_en = USARTCAN.can[EnCAN];
    flashdata.canbaudrate = USARTCAN.can[canBaudrate];
    flashdata.can_localID = USARTCAN.can[LocalID];
    flashdata.can_deviceID = USARTCAN.can[DeviceID];
    flashdata.can_device_num = USARTCAN.can[IDNum];
    flashdata.can_datatype = USARTCAN.can[canDatatype];
    //预留6
    
    flashdata.rs232_1_en = USARTCAN.Usart[RS232_1][EnUart];
    flashdata.rs232_1_baudrate = USARTCAN.Usart[RS232_1][uartBaudrate];
    flashdata.rs232_1_databit = USARTCAN.Usart[RS232_1][Databits];
    flashdata.rs232_1_chkbit = USARTCAN.Usart[RS232_1][Chkbits];
    flashdata.rs232_1_stopbit = USARTCAN.Usart[RS232_1][Stopbits];
    flashdata.rs232_1_flowctrl = USARTCAN.Usart[RS232_1][Flowctrl];
    flashdata.rs232_1_datatype = USARTCAN.Usart[RS232_1][uartDatatype];
    //预留7
    
	flashdata.rs485_1_en = USARTCAN.Usart[RS485_1][EnUart];
    flashdata.rs485_1_baudrate = USARTCAN.Usart[RS485_1][uartBaudrate];
    flashdata.rs485_1_databit = USARTCAN.Usart[RS485_1][Databits];
    flashdata.rs485_1_chkbit = USARTCAN.Usart[RS485_1][Chkbits];
    flashdata.rs485_1_stopbit = USARTCAN.Usart[RS485_1][Stopbits];
    flashdata.rs485_1_datatype = USARTCAN.Usart[RS485_1][uartDatatype];
    //预留8
    
    flashdata.rs485_2_en = USARTCAN.Usart[RS485_2][EnUart];
    flashdata.rs485_2_baudrate = USARTCAN.Usart[RS485_2][uartBaudrate];
    flashdata.rs485_2_databit = USARTCAN.Usart[RS485_2][Databits];
    flashdata.rs485_2_chkbit = USARTCAN.Usart[RS485_2][Chkbits];
    flashdata.rs485_2_stopbit = USARTCAN.Usart[RS485_2][Stopbits];
    flashdata.rs485_2_datatype = USARTCAN.Usart[RS485_2][uartDatatype];
    //预留9
	
	flashdata.rs485_3_en = USARTCAN.Usart[RS485_3][EnUart];
    flashdata.rs485_3_baudrate = USARTCAN.Usart[RS485_3][uartBaudrate];
    flashdata.rs485_3_databit = USARTCAN.Usart[RS485_3][Databits];
    flashdata.rs485_3_chkbit = USARTCAN.Usart[RS485_3][Chkbits];
    flashdata.rs485_3_stopbit = USARTCAN.Usart[RS485_3][Stopbits];
    flashdata.rs485_3_datatype = USARTCAN.Usart[RS485_3][uartDatatype];
    //预留10
    
    flashdata.rs485_4_en = USARTCAN.Usart[RS485_4][EnUart];
    flashdata.rs485_4_baudrate = USARTCAN.Usart[RS485_4][uartBaudrate];
    flashdata.rs485_4_databit = USARTCAN.Usart[RS485_4][Databits];
    flashdata.rs485_4_chkbit = USARTCAN.Usart[RS485_4][Chkbits];
    flashdata.rs485_4_stopbit = USARTCAN.Usart[RS485_4][Stopbits];
    flashdata.rs485_4_datatype = USARTCAN.Usart[RS485_4][uartDatatype];
    //预留11
    
    flashdata.to_thres = USARTCAN.tout;
    //预留12
    
    OS_ENTER_CRITICAL();
    FlashErase(CONFIG_BASEADDR,1);
    FlashWriteData(CONFIG_BASEADDR,(u8 *)&flashdata,sizeof(flashdata));
    OS_EXIT_CRITICAL();
    dylms(500);
    f_GenSoftwareReset();//flash更新后启动复位，按照新的配置运行
}

//软件复位
void f_GenSoftwareReset(void)
{
    __set_FAULTMASK(1);
	SCB->AIRCR = AIRCR_VECTKEY_MASK | (u32)0x04;
}

