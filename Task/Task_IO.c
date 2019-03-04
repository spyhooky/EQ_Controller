#include "main.h"
#include "Task_IO.h"
//#include "Task_LED.h"
#include "DrUart.h"

#define DETECT_ONLY_STARTUP

static u8  DIP_Swtch_Update_En;
static u16 LED_Blink_time;
static u8  LED_Blink_End;
volatile BitStatus    	Relay_Output_Sts;//


struct DIPType
{
  union 
  {
    u8 Byte;
    struct {
  		u8  pre_level              :1;
  		u8  cur_status             :1;
  		u8  timer_en               :1;
		u8  reserve                :5;
  	}Bits;
  }digitstatus;
  u16 keep_time;
};
static struct DIPType       	DIP_Switch_Sts[DIP_SWITCH_BITS_NUM];//
#define Pre_DIP_Level(n)		DIP_Switch_Sts[n].digitstatus.Bits.pre_level
#define Cur_DIP_Status(n)		DIP_Switch_Sts[n].digitstatus.Bits.cur_status
#define DIP_TimerEn(n)			DIP_Switch_Sts[n].digitstatus.Bits.timer_en
#define DIP_STS_KEEP_TIME(n)	DIP_Switch_Sts[n].keep_time
#define DIP_SWITCH_FILTER                            80u //拨码开关滤波时间，单位ms

static struct DIPType           DigitInput_Sts[DIGIT_INPUT_CHN_NUM];//
#define Pre_Input_Level(n)		DigitInput_Sts[n].digitstatus.Bits.pre_level
#define Cur_Input_Status(n)		DigitInput_Sts[n].digitstatus.Bits.cur_status
#define DigitInputTimerEn(n)	DigitInput_Sts[n].digitstatus.Bits.timer_en
#define INPUT_STS_KEEP_TIME(n)	DigitInput_Sts[n].keep_time
#define DIGIT_INPUT_FILTER                            100u //数字开关量滤波时间，单位ms

/**********仅仅测试串口时使用************/
static u16 en_test=0;//该值写成1时，默认500ms周期向外部发送数据
static u16 chn_test=2;//0-232通道，2-485通道，靠近编码器的485，板子中间的那个插件，用一个独立隔离DCDC的
static u16 testcnt;
/****************************************/

static void DIP_Switch_Detect(void);
static void DIP_Switch_Mainfunction(void);
static void INPUT_Check_Mainfunction(void);
static void Output_MainFunction(void);
static void LED_MainFunction(void);

u8 Get_LED_Status(void);
void Blink_LED_Status(u16 timer);

void TaskIO_Timer100us(void)
{
    testcnt++;
}


void TaskIO_Timer1ms(void)
{
	u8 i;
	
	for(i=0;i<DIP_SWITCH_BITS_NUM;i++)
	{//遍历所有拨码开关位
		if(DIP_TimerEn(i) == ON)
		{//若计数器使能，则启动自增
			DIP_STS_KEEP_TIME(i)++;
		}
	}

    for(i=0;i<DIGIT_INPUT_CHN_NUM;i++)
	{//遍历所有外部开关量输入状态位
		if(DigitInputTimerEn(i) == ON)
		{//若计数器使能，则启动自增
			INPUT_STS_KEEP_TIME(i)++;
		}
	}
    
	if(LED_Blink_time>0)
	{//若LED闪烁时间未到，则继续自减，直到为0
		LED_Blink_time--;
		if(LED_Blink_time == 0)
		{
			LED_Blink_End = TRUE;
		}
	}
}

void Task_IO_Init(void)
{
    u8 j;
	memset(DIP_Switch_Sts,0,sizeof(DIP_Switch_Sts));
    memset(DigitInput_Sts,0,sizeof(DigitInput_Sts));
	LED_Blink_time = 0;
    LED_Blink_End = OFF;
    #ifdef DETECT_ONLY_STARTUP
    DIP_Switch_Detect();
    DIP_Swtch_Update_En = OFF;
    #else
    DIP_Swtch_Update_En = ON;
    #endif
    for(j=0;j<DIP_SWITCH_BITS_NUM;j++)
    {
        Pre_DIP_Level(j) = 0x01;
    }
    for(j=0;j<DIGIT_INPUT_CHN_NUM;j++)
    {
        Pre_Input_Level(j) = 0x01;
    }
    //Relay_Output_Sts=0;
}

static void Output_MainFunction(void)
{
    if(Band_Type_Brake_Out == ON)
    {
        RELAY_1_OUT(ON);
    }
    else
    {
        RELAY_1_OUT(OFF);
    }

}

/****************************************************************************/
/*函数名：  Task_IO                                                         */
/*功能说明：  IO的主函数，调用输入识别函数和输出控制函数                    */        
/*输入参数：无                                                              */
/*输出参数：无                                                              */
/****************************************************************************/
void Task_IO(void *p_arg)
{
    (void)p_arg;
    Task_IO_Init();
    u8 data[5] ={0x11,0x22,0x33,0x44,0x55};
    while (1)
    {	
        if(en_test==1)
        {
            if(testcnt>=5000)
            {
                testcnt = 0;
                Blink_LED_Status(200);
                Uart_Transmit(chn_test,data,5);
            }
            else
            {
                //testcnt++;
            }
        }
        
        if(DIP_Swtch_Update_En == ON)
        {
		    DIP_Switch_Mainfunction();
        }
		INPUT_Check_Mainfunction();
		LED_MainFunction();
        Output_MainFunction();
        OSTimeDlyHMSM(0, 0, 0, 1);
        
    }
    
}

/****************************************************************************/
/*函数名：  Get_LED_Status                                                  */
/*功能说明：获取当前LED的状态，0-当前LED为熄灭，1-点亮                      */        
/*输入参数：无                                                              */
/*输出参数：无                                                              */
/****************************************************************************/
u8 Get_LED_Status(void)
{
	u8 sts;
	sts = GPIO_ReadOutputDataBit(LEDG_PORT,LEDG_PIN);
	return (sts==ON) ? OFF:ON;
}

/****************************************************************************/
/*函数名：  Blink_LED_Status                                                */
/*功能说明：  控制LED灯闪烁一次，过程为先将LED状态反转，然后保持一段时间    */        
/*输入参数：mstimer，LED状态反转后保持的时间                                */
/*输出参数：无                                                              */
/****************************************************************************/
void Blink_LED_Status(u16 mstimer)
{
	if(Get_LED_Status() == OFF)
	{
		SET_LED_STATUS(ON);
	}
	else
	{
		SET_LED_STATUS(OFF);
	}
	LED_Blink_time = mstimer;
}

/********************************************************************************/
/*函数名：  LED_MainFunction                                                    */
/*功能说明：  LED控制的主函数，主要用于判断一次LED闪烁是否完成，若时间到则熄灭      */        
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/********************************************************************************/
static void LED_MainFunction(void)
{
	if(LED_Blink_End == TRUE)
	{
		LED_Blink_End = FALSE;
		Blink_LED_Status(0);
	}
}

/********************************************************************************/
/*函数名：  DIP_Switch_Detect                                                   */
/*功能说明：  拨码开关状态识别，上电后只读一次                                    */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/********************************************************************************/
static void DIP_Switch_Detect(void)
{
    u8 i,j,temp;
    u8 cur_level[DIP_SWITCH_BITS_NUM];
    memset(cur_level,0,sizeof(cur_level));
    for(i=0;i<10;i++)
    {
        for(j=0;j<DIP_SWITCH_BITS_NUM;j++)
        {
            cur_level[j] += GET_DIP_SWITCH_STATUS(j);//获取当前状态
        }
    }
    
    temp = 0;
    for(j=0;j<DIP_SWITCH_BITS_NUM;j++)
    {
        Cur_DIP_Status(j) = (cur_level[j]>4) ? 1:0;
        temp |= Cur_DIP_Status(j)<<j;
    }
    Globle_Framework.DIP_SwitchStatus = ~temp;//更新拨码开关值
}


/********************************************************************************/
/*函数名：  DIP_Switch_Mainfunction                                             */
/*功能说明：  拨码开关状态识别                                                   */
/*，判断某个位状态发生改变时开始计时，维持80ms恒定则认为是个有效的切换，并记录      */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/********************************************************************************/
static void DIP_Switch_Mainfunction(void)
{
	u8 i,j;
	u8 temp=0;
	u8 cur_level[DIP_SWITCH_BITS_NUM];
    memset(cur_level,0,sizeof(cur_level));
	for(i=0;i<DIP_SWITCH_BITS_NUM;i++)//8个位依次遍历
	{
        cur_level[i] = GET_DIP_SWITCH_STATUS(i);//获取当前状态
		if(Pre_DIP_Level(i) != cur_level[i]) //判断当前状态和上次状态是否一致
		{//不一致的话就启动一个计数器计数
			DIP_STS_KEEP_TIME(i) = 0u;
			DIP_TimerEn(i) = ON;
		}
		
		if(DIP_STS_KEEP_TIME(i)>=DIP_SWITCH_FILTER)
		{//如果计数器计算达到80ms，说明该状态切换后稳定保持了80ms，认为是一个有效的变化
			DIP_TimerEn(i) = OFF;
			DIP_STS_KEEP_TIME(i) = 0u;
			Cur_DIP_Status(i) = cur_level[i];
			for(j=0;j<DIP_SWITCH_BITS_NUM;j++)
			{
				temp |= Cur_DIP_Status(j)<<j;
			}
			Globle_Framework.DIP_SwitchStatus = ~temp;//更新拨码开关值
		}
		Pre_DIP_Level(i) = cur_level[i];
	}
}

/********************************************************************************/
/*函数名：  InputGPIO_Configuration                                             */
/*功能说明：外部输入的开关状态识别                                              */
/*，判断某个位状态发生改变时开始计时，维持100ms恒定则认为是个有效的切换，并记录 */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/********************************************************************************/
static void INPUT_Check_Mainfunction(void)
{
    u8 i;
    u8 cur_level[DIGIT_INPUT_CHN_NUM];
    memset(cur_level,0,sizeof(cur_level));
    for(i=0;i<DIGIT_INPUT_CHN_NUM;i++)//所有输入口按位依次遍历
    {
        cur_level[i] = GET_DIGIT_INPUT_STATUS(i);//获取当前状态
        if(Pre_Input_Level(i) != cur_level[i])//判断当前状态和上次状态是否一致
        {//不一致的话就启动一个计数器计数
            INPUT_STS_KEEP_TIME(i) = 0u;
            DigitInputTimerEn(i) = ON;
        }
        
        if(INPUT_STS_KEEP_TIME(i) >= DIGIT_INPUT_FILTER)
        {//如果计数器计算达到100ms，说明该状态切换后稳定保持了100ms，认为是一个有效的变化
            DigitInputTimerEn(i) = OFF;
            INPUT_STS_KEEP_TIME(i) = 0u;
            Cur_Input_Status(i) = cur_level[i];
            //更新该输入口状态值
            Globle_Framework.Digit_InputStatus = (cur_level[i]==0)? ((1<<i)|Globle_Framework.Digit_InputStatus):((~(1<<i))&Globle_Framework.Digit_InputStatus);
        }
        Pre_Input_Level(i) = cur_level[i];
    }
}






