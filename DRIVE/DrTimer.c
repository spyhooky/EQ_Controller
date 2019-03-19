#include "main.h"
#include "Task_MQTT.h"
#include "Task_ModbusTCP.h"
#include "Task_TCPIP.h"


static unsigned short  g_u16Timer100usDelayCount  = 0;/*100us定时器计数器*/
static unsigned short  g_u16Timer100usCount  = 0;/*100us定时器计数器*/



/****************************** 中断服务程序 ******************************/
void TIM2_IRQ(void)/* 100uS */
{
    
//    unsigned char l_u8i = 0;
	Timer_100us_Application();
    USART_Timer100us();

    /* 100uS */
    if (g_u16Timer100usDelayCount > 0)
    {
        g_u16Timer100usDelayCount--;
    }
    if (g_u16Timer100usCount > 0)
    {
        g_u16Timer100usCount--;
    }
    
    /* 定时器 */
    if (0 == g_u16Timer100usCount)
    {
        g_u16Timer100usCount = 10;
        
    }

}

void TIM3_IRQ(void)/* 1ms */
{
	Timer_1ms_Application();
	Framework_Timer1ms();
}

void TIM5_IRQ(void)/* 100ms */
{

//    unsigned char l_u8i = 0;
	Timer_100ms_Application();
    Framework_Timer100ms(); 
}



/* 短延时，初始化使用 */
void Delay1ms(u16 l_u16Count)
{

    g_u16Timer100usDelayCount = l_u16Count * 10;
    while (g_u16Timer100usDelayCount)
    {
        if(g_u16Timer100usDelayCount%200==0)
		{
			IWDG_ReloadCounter();
		}
    }
    
}


/****************************** 普通定时器 ******************************/
/* TIM2配置 */
void TIM2_Config(void)/* 100us */
{					
    
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_DeInit(TIM2);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructure.TIM_Period = 19;/* 20和360 100us*/
    TIM_TimeBaseStructure.TIM_Prescaler = 359;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update|TIM_IT_Trigger, ENABLE);
    TIM_Cmd(TIM2, ENABLE); //使能定时器   

    

}

/* TIM3配置 */
void TIM3_Config(void)/* 1ms */
{					
    
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_DeInit(TIM3);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseStructure.TIM_Period = 1;/* 2和18000 1ms*/
    TIM_TimeBaseStructure.TIM_Prescaler = 35999;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update|TIM_IT_Trigger, ENABLE);
    TIM_Cmd(TIM3, ENABLE); //使能定时器   
}

void TIM4_Encoder_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;      

    //PB6: A TMI4_CH1, PB7 B TMI4_CH2 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);                           


    TIM_DeInit(TIM4);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 0xffff; 
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);              

    //TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,  TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6; 
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM4->CNT = 0;
    TIM_Cmd(TIM4, ENABLE);
}


/* TIM5配置 */
void TIM5_Config(void)/* 100ms */
{

    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;		
  
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
    
    TIM_DeInit(TIM5);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    TIM_TimeBaseStructure.TIM_Period = 199;  /* 200和18000 1ms*/
    TIM_TimeBaseStructure.TIM_Prescaler = 17999;
    TIM_TimeBaseStructure.TIM_ClockDivision=0;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM5,TIM_IT_Update|TIM_IT_Trigger,ENABLE);
    TIM_Cmd(TIM5, ENABLE);

}


void DrTimer_Init(void)
{
    TIM2_Config();
    TIM3_Config();
    TIM5_Config();
    TIM4_Encoder_Config();
}

