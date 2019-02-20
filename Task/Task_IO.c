#include "main.h"
#include "Task_IO.h"
//#include "Task_LED.h"
#include "DrUart.h"

static u16 LED_Blink_time;
static u8  LED_Blink_End;

struct DIPType
{
  union 
  {
    u8 bytetype;
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
#define DIGIT_INPUT_FILTER                            100u //拨码开关滤波时间，单位ms



static void DIP_Switch_Mainfunction(void);
static void INPUT_Check_Mainfunction(void);
static void LED_MainFunction(void);

u8 Get_LED_Status(void);
void Blink_LED_Status(u16 timer);

void TaskIO_Timer1ms(void)
{
	u8 i;
	
	for(i=0;i<DIP_SWITCH_BITS_NUM;i++)
	{
		if(DIP_TimerEn(i) == ON)
		{
			DIP_STS_KEEP_TIME(i)++;
		}
	}

    for(i=0;i<DIGIT_INPUT_CHN_NUM;i++)
	{
		if(DigitInputTimerEn(i) == ON)
		{
			INPUT_STS_KEEP_TIME(i)++;
		}
	}
    
	if(LED_Blink_time>0)
	{
		LED_Blink_time--;
		if(LED_Blink_time == 0)
		{
			LED_Blink_End = TRUE;
		}
	}
}

void Task_IO_Init(void)
{
	memset(DIP_Switch_Sts,0,sizeof(DIP_Switch_Sts));
    memset(DigitInput_Sts,0,sizeof(DigitInput_Sts));
	LED_Blink_time = 0;
    LED_Blink_End = OFF;
}

void Task_IO(void *p_arg)
{
    
    (void)p_arg;
    
    DrGpioInit();
    
    while (1)
    {	
		DIP_Switch_Mainfunction();
		INPUT_Check_Mainfunction();
		LED_MainFunction();
        OSTimeDlyHMSM(0, 0, 0, 1);
    }
    
}

u8 Get_LED_Status(void)
{
	u8 sts;
	sts = GPIO_ReadOutputDataBit(LEDG_PORT,LEDG_PIN);
	return (sts==ON) ? OFF:ON;
}

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

static void LED_MainFunction(void)
{
	if(LED_Blink_End == TRUE)
	{
		LED_Blink_End = FALSE;
		Blink_LED_Status(0);
	}
}

static void DIP_Switch_Mainfunction(void)
{
	u8 i,j;
	u8 temp=0;
	u8 cur_level[DIP_SWITCH_BITS_NUM];
	for(i=0;i<DIP_SWITCH_BITS_NUM;i++)
	{
        cur_level[i] = GET_DIP_SWITCH_STATUS(i);
		if(Pre_DIP_Level(i) != cur_level[i])
		{
			DIP_STS_KEEP_TIME(i) = 0u;
			DIP_TimerEn(i) = ON;
		}
		
		if(DIP_STS_KEEP_TIME(i)>=DIP_SWITCH_FILTER)
		{
			DIP_TimerEn(i) = OFF;
			DIP_STS_KEEP_TIME(i) = 0u;
			Cur_DIP_Status(i) = cur_level[i];
			for(j=0;j<DIP_SWITCH_BITS_NUM;j++)
			{
				temp |= Cur_DIP_Status(j)<<j;
			}
			Globle_Framework.DIP_SwitchStatus = ~temp;
		}
		
	}

	for(i=0;i<DIP_SWITCH_BITS_NUM;i++)
	{
		Pre_DIP_Level(i) = cur_level[i];
	}
}

static void INPUT_Check_Mainfunction(void)
{
    u8 i;
    u8 cur_level[DIGIT_INPUT_CHN_NUM];
    memset(cur_level,0,sizeof(cur_level));
    for(i=0;i<DIGIT_INPUT_CHN_NUM;i++)
    {
        cur_level[i] = GET_DIGIT_INPUT_STATUS(i);
        if(Pre_Input_Level(i) != cur_level[i])
        {
            INPUT_STS_KEEP_TIME(i) = 0u;
            DigitInputTimerEn(i) = ON;
        }
        
        if(INPUT_STS_KEEP_TIME(i) >= DIGIT_INPUT_FILTER)
        {
            DigitInputTimerEn(i) = OFF;
            INPUT_STS_KEEP_TIME(i) = 0u;
            Cur_Input_Status(i) = cur_level[i];
            Globle_Framework.Digit_InputStatus = (cur_level[i]==0)? ((1<<i)|Globle_Framework.Digit_InputStatus):((~(1<<i))&Globle_Framework.Digit_InputStatus);
        }
        
    }

    for(i=0;i<DIGIT_INPUT_CHN_NUM;i++)
    {
        Pre_Input_Level(i) = cur_level[i];
    }
}






