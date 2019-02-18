#include "main.h" 

static void DIP_Switch_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE,ENABLE);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_InitStructure.GPIO_Pin =DIP_SWITCH_1_PIN;
	GPIO_Init(DIP_SWITCH_1_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =DIP_SWITCH_2_PIN;
	GPIO_Init(DIP_SWITCH_2_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =DIP_SWITCH_3_PIN;
	GPIO_Init(DIP_SWITCH_3_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =DIP_SWITCH_4_PIN;
	GPIO_Init(DIP_SWITCH_4_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =DIP_SWITCH_5_PIN;
	GPIO_Init(DIP_SWITCH_5_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =DIP_SWITCH_6_PIN;
	GPIO_Init(DIP_SWITCH_6_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =DIP_SWITCH_7_PIN;
	GPIO_Init(DIP_SWITCH_7_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =DIP_SWITCH_8_PIN;
	GPIO_Init(DIP_SWITCH_8_PORT, &GPIO_InitStructure);
}

static void InputGPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE,ENABLE);	
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    #ifdef INPUT_TRIGGER_FILTER
	GPIO_InitStructure.GPIO_Pin =MCU_IN_1_PIN;
	GPIO_Init(MCU_IN_1_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_2_PIN;
	GPIO_Init(MCU_IN_2_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_3_PIN;
	GPIO_Init(MCU_IN_3_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_4_PIN;
	GPIO_Init(MCU_IN_4_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_5_PIN;
	GPIO_Init(MCU_IN_5_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_6_PIN;
	GPIO_Init(MCU_IN_6_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_7_PIN;
	GPIO_Init(MCU_IN_7_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_8_PIN;
	GPIO_Init(MCU_IN_8_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_9_PIN;
	GPIO_Init(MCU_IN_9_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_10_PIN;
	GPIO_Init(MCU_IN_10_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_11_PIN;
	GPIO_Init(MCU_IN_11_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_12_PIN;
	GPIO_Init(MCU_IN_12_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_13_PIN;
	GPIO_Init(MCU_IN_13_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_14_PIN;
	GPIO_Init(MCU_IN_14_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_15_PIN;
	GPIO_Init(MCU_IN_15_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_16_PIN;
	GPIO_Init(MCU_IN_16_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =MCU_IN_17_PIN;
	GPIO_Init(MCU_IN_17_PORT, &GPIO_InitStructure);
    #endif

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    GPIO_EXTILineConfig(MCU_IN_9_PORTSOURCE,MCU_IN_15_PIN_SOURCE);//
    EXTI_InitStructure.EXTI_Line = MCU_IN_9_EXITLINE ; //PE2 
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;        
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;    
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                  
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = MCU_IN_9_IRQ_CHN;   //
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
    NVIC_Init(&NVIC_InitStructure); 
}

static void OutputGPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC,ENABLE);		  
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD,ENABLE);		  
  
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_3|GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
			
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
 
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_3|GPIO_Pin_7|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOA, GPIO_Pin_3);
  GPIO_ResetBits(GPIOD, GPIO_Pin_3);
}

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





