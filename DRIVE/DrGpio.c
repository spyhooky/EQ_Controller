#include "main.h" 

#if (defined INPUT_TRIGGER_INTERRUPT)
const GPIO_TypeDef* GPIO_PORT[7] = {//所有GPIO定义的集合
    GPIOA,GPIOB,GPIOC,GPIOD,GPIOE,GPIOF,GPIOG
};
const u16 GPIO_PIN[16] = {//所有PIN定义的集合
    GPIO_Pin_0,GPIO_Pin_1,GPIO_Pin_2,GPIO_Pin_3,GPIO_Pin_4,GPIO_Pin_5,GPIO_Pin_6,GPIO_Pin_7,
    GPIO_Pin_8,GPIO_Pin_9,GPIO_Pin_10,GPIO_Pin_11,GPIO_Pin_12,GPIO_Pin_13,GPIO_Pin_14,GPIO_Pin_15
};
const u8 Exit_PortSrc[7] = {//所有外部中断portsource的集合
    GPIO_PortSourceGPIOA,GPIO_PortSourceGPIOB,GPIO_PortSourceGPIOC,GPIO_PortSourceGPIOD,
    GPIO_PortSourceGPIOE,GPIO_PortSourceGPIOF,GPIO_PortSourceGPIOG
};
const u8 Exit_PinSrc[16] = {//所有外部中断pinsource的集合
    GPIO_PinSource0,GPIO_PinSource1,GPIO_PinSource2,GPIO_PinSource3,
    GPIO_PinSource4,GPIO_PinSource5,GPIO_PinSource6,GPIO_PinSource7,
    GPIO_PinSource8,GPIO_PinSource9,GPIO_PinSource10,GPIO_PinSource11,
    GPIO_PinSource12,GPIO_PinSource13,GPIO_PinSource14,GPIO_PinSource15,
};
const u32 Exit_Line[16]={//所有外部中断LINE的集合
    EXTI_Line0,EXTI_Line1,EXTI_Line2,EXTI_Line3,EXTI_Line4,EXTI_Line5,EXTI_Line6,EXTI_Line7,
    EXTI_Line8,EXTI_Line9,EXTI_Line10,EXTI_Line11,EXTI_Line12,EXTI_Line13,EXTI_Line14,EXTI_Line15
};
#endif

IO_REG_GROUP DIP_SWITCH_REG = DIP_SWITCH_REG_LIST;//拨码开关的初始化配置列表
IO_REG_GROUP DIGIT_INPUT_REG = DIGIT_INPUT_REG_LIST;//外部开关量输入的初始化配置列表
IO_REG_GROUP DIGIT_OUTPUT_REG = DIGIT_OUTPUT_REG_LIST;//开关量输出的初始化配置列表

/****************************************************************************/
/*函数名：  DIP_Switch_Configuration                                        */
/*功能说明：  拨码开关状态相关寄存器初始化                                  */
/*输入参数：无                                                              */
/*输出参数：无                                                              */
/****************************************************************************/
static void DIP_Switch_Configuration(void)
{
    u8 i;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE,ENABLE);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    for(i=0;i<DIP_SWITCH_BITS_NUM;i++)
    {
        GPIO_InitStructure.GPIO_Pin =DIP_SWITCH_REG_PIN(i);
	    GPIO_Init(DIP_SWITCH_REG_PORT(i), &GPIO_InitStructure);
    }
}

/****************************************************************************/
/*函数名：  InputGPIO_Configuration                                         */
/*功能说明：  外部开关量输入状态相关寄存器初始化                            */
/*输入参数：无                                                              */
/*输出参数：无                                                              */
/****************************************************************************/
static void InputGPIO_Configuration(void)
{
    u8 i;
    u8 index=0;
	GPIO_InitTypeDef GPIO_InitStructure;
    #if (defined INPUT_TRIGGER_INTERRUPT)
    u8 portsrc;
    u8 pinsrc;
    u32 line;
    u8 irqchn;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    #endif
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE,ENABLE);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    for(i=0;i<DIGIT_INPUT_CHN_NUM;i++)
    {
        GPIO_InitStructure.GPIO_Pin =DIGIT_INPUT_REG_PIN(i);
	    GPIO_Init(DIGIT_INPUT_REG_PORT(i), &GPIO_InitStructure);
    }
    
    #if (defined INPUT_TRIGGER_INTERRUPT)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    for(i=0;i<DIGIT_INPUT_CHN_NUM;i++)
    {
        for(index=0;index<sizeof(GPIO_PORT);index++)
        {
            if(GPIO_PORT[index] == DIGIT_INPUT_REG_PORT(i))
            {
                portsrc = Exit_PortSrc[index];
                break;
            }
        }
        for(index=0;index<sizeof(GPIO_PIN);index++)
        {
            if(GPIO_PIN[index] == DIGIT_INPUT_REG_PIN(i))
            {
                pinsrc = Exit_PinSrc[index];
                line = Exit_Line[index];
                break;
            }
        }
        if(index<5u)//5-9共用一个中断
        {
            irqchn = EXTI0_IRQn + index;
        }
        else if(index<10u)//10-15共用另一个中断
        {
            irqchn = EXTI9_5_IRQn;
        }
        else
        {
            irqchn = EXTI15_10_IRQn;
        }
        
        GPIO_EXTILineConfig(portsrc,pinsrc);
        EXTI_InitStructure.EXTI_Line = line;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;        
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;    
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;                  
        EXTI_Init(&EXTI_InitStructure);

        NVIC_InitStructure.NVIC_IRQChannel = irqchn;   //
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
        NVIC_Init(&NVIC_InitStructure);
    }    
    #endif
}

/****************************************************************************/
/*函数名：  OutputGPIO_Configuration                                        */
/*功能说明：开关输出相关寄存器初始化                                        */
/*输入参数：无                                                              */
/*输出参数：无                                                              */
/****************************************************************************/
static void OutputGPIO_Configuration(void)
{
    u8 i;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RELAY_GPIO_APB,ENABLE);		 

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    for(i=0;i<DIGIT_OUTPUT_NUM;i++)
    {
        GPIO_InitStructure.GPIO_Pin =DIGIT_OUTPUT_REG_PIN(i);
	    GPIO_Init(DIGIT_OUTPUT_REG_PORT(i), &GPIO_InitStructure);
    }
}

/****************************************************************************/
/*函数名：  LED_Configuration                                               */
/*功能说明：  LED相关寄存器初始化                                           */
/*输入参数：无                                                              */
/*输出参数：无                                                              */
/****************************************************************************/
static void LED_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;    
    RCC_APB2PeriphClockCmd(LED_GPIO_APB, ENABLE);
    
//    GPIO_InitStructure.GPIO_Pin     = LEDR_PIN;
//  	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
//  	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_2MHz;
//  	GPIO_Init(LEDR_PORT, &GPIO_InitStructure);
//    LED_R(ON);
    
    GPIO_InitStructure.GPIO_Pin     = LEDG_PIN;
  	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_2MHz;
  	GPIO_Init(LEDG_PORT, &GPIO_InitStructure);
    LED_G(OFF);
    
}

/****************************************************************************/
/*函数名：  DrGpioInit                                                      */
/*功能说明：IO相关寄存器初始化                                              */
/*输入参数：无                                                              */
/*输出参数：无                                                              */
/****************************************************************************/
void DrGpioInit( void )
{  	
	DIP_Switch_Configuration();
    InputGPIO_Configuration();
	OutputGPIO_Configuration();
	LED_Configuration();
}

void EXIT9_5_IRQ(void)
{
    Globle_Framework.Digit_InputStatus |= 0x8000;
}





