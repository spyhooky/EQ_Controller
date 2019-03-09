#include "main.h"
#include "Task_MQTT.h"
#include "Task_ModbusTCP.h"
#include "Task_TCPIP.h"


static unsigned short  g_u16Timer100usDelayCount  = 0;/*100us��ʱ��������*/
static unsigned short  g_u16Timer100usCount  = 0;/*100us��ʱ��������*/



/****************************** �жϷ������ ******************************/
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
    
    /* ��ʱ�� */
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

void TIM4_IRQ(void)/* 100ms */
{

//    unsigned char l_u8i = 0;
	Timer_100ms_Application();
    Framework_Timer100ms();
    
}



/* ����ʱ����ʼ��ʹ�� */
void Delay1ms(uint16_t  l_u16Count)
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


/****************************** ��ͨ��ʱ�� ******************************/
/* TIM2���� */
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
    TIM_TimeBaseStructure.TIM_Period = 19;/* 20��360 100us*/
    TIM_TimeBaseStructure.TIM_Prescaler = 359;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update|TIM_IT_Trigger, ENABLE);
    TIM_Cmd(TIM2, ENABLE); //ʹ�ܶ�ʱ��   

    

}

/* TIM3���� */
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
    TIM_TimeBaseStructure.TIM_Period = 1;/* 2��18000 1ms*/
    TIM_TimeBaseStructure.TIM_Prescaler = 17999;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update|TIM_IT_Trigger, ENABLE);
    TIM_Cmd(TIM3, ENABLE); //ʹ�ܶ�ʱ��   

    

}


/* TIM4���� */
void TIM4_Config(void)/* 100ms */
{

    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;		
  
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
    
    TIM_DeInit(TIM4);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseStructure.TIM_Period = 199;  /* 200��18000 1ms*/
    TIM_TimeBaseStructure.TIM_Prescaler = 17999;
    TIM_TimeBaseStructure.TIM_ClockDivision=0;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_Trigger,ENABLE);
    TIM_Cmd(TIM4, ENABLE);

}


void DrTimer_Init(void)
{
    TIM2_Config();
    TIM3_Config();
    TIM4_Config();
}

