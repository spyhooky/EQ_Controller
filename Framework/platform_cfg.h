#ifndef __PLATFORM_CFG_H_
#define __PLATFORM_CFG_H_

#include "stm32f10x.h"

#ifdef STM32F10X_LD
    #define FLASH_PAGE_SIZE                 0x400U
    #define CONFIG_BASEADDR                 0x800EC00ul  //常用配置文件的存储起始地址
#endif

#ifdef STM32F10X_MD
    #define FLASH_PAGE_SIZE                 0x400U
    #define CONFIG_BASEADDR                 0x800EC00ul
#endif

#ifdef STM32F10X_HD
    #define FLASH_PAGE_SIZE                 0x800U
    #define CONFIG_BASEADDR                 0x803A000ul
#endif

#ifdef STM32F10X_CL
    #define FLASH_PAGE_SIZE                 0x800U     
    #define CONFIG_BASEADDR                 0x803A000ul
#endif

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

/****************** ADC ***********************/
enum ADC_Channel
{
	ADChannel1=0,ADChannel2,ADChannel3,ADChannel4,Power_5V,TempSensor,
	Channel_Num
};

/***********************************************************************/
/***************************功能使能区**********************************/
/***********************************************************************/
#define DEBUG_ENABLE   //debug模式下禁止看门狗功能和W5500自动keepalive功能
#define W5500_ENABLE                                  //W5500芯片使能，当需要网口进行通讯时该宏定义必须是激活状态

#ifndef DEBUG_ENABLE
 #define WATCHDOG_ENABLE                            //看门狗宏开关，不需要打开看门狗时需要将此宏屏蔽       
#endif 

#if (defined STM32F10X_HD) && (defined STM32F10X_CL)
#define UART_DMA_ENABLE                               //串口DMA使能
#endif

#ifdef W5500_ENABLE
//	#define TCPIP_ENABLE                                  //TCPIP功能使能
	#define MBTCP_ENABLE                                  //MBTCP功能使能
//	#define MQTT_ENABLE                                   //MQTT功能使能
//	#define HTTP_ENABLE                                   //HTTP功能使能
#endif

#ifdef W5500_ENABLE
//	#define CJSON_ENABLE
#endif

/***********************************************************************/

/**************** 中断等级分配 *******************/
#define USART1_PRE					0U
#define USART1_SUB					0U
#define USART2_PRE					0U
#define USART2_SUB					0U
#define USART3_PRE					0U
#define USART3_SUB					0U
#define UART4_PRE					0U
#define UART4_SUB					0U
#define UART5_PRE					0U
#define UART5_SUB					0U

#define TIM2_PRE					0U
#define TIM2_SUB					2U
#define TIM3_PRE					0U
#define TIM3_SUB					2U
#define TIM4_PRE					0U
#define TIM4_SUB					2U

#define CAN1_RX0_PRE				0U
#define CAN1_RX0_SUB				1U

#define WWDG_PRE				    0U
#define WWDG_SUB				    3U


/**************** 本地套接字分配 *******************/
#define SOCK_TCPIP                     0x00U
#define SOCK_MBTCP                     0x01U
#define SOCK_MQTT                      0x06U
#define SOCK_HTTP                      0x07U


/**************** 本地端口号分配 *******************/
#define PORT_LOCAL        502

#define APP_CRC_ADDR                                          0x8024000UL

/* ASCII */
#define SOH                             0x01U
#define STX                             0x02U
#define ETX                             0x03U
#define EOT                             0x04U
#define ENQ                             0x05U
#define ACK                             0x06U
#define NAK                             0x15U
#define SYN                             0x16U
#define ETB                             0x17U

/* */
#define NONE                            0U
#define TRUE                            1U
#define FALSE                           0U
#define ON                              1U
#define OFF                             0U
#define ALL                             0xffU

/**************************************** LED ****************************************/
#define LED_GPIO_APB                RCC_APB2Periph_GPIOE
#define LEDG_PORT                   GPIOE
#define LEDG_PIN                    GPIO_Pin_9
#define LED_G(n)                    { (n) ? ( GPIO_ResetBits(LEDG_PORT,LEDG_PIN)) : (GPIO_SetBits(LEDG_PORT,LEDG_PIN)); }

#define IO_TRIGGER(port,pin)        { if(GPIO_ReadOutputDataBit(port,pin)) GPIO_ResetBits(port,pin); else GPIO_SetBits(port,pin); }

/**************************************** OUTPUT & RELAY ****************************************/
#define DIGIT_OUTPUT_NUM            7U

#define RELAY_GPIO_APB              RCC_APB2Periph_GPIOC
/**********  RELAY 1  ***********/
#define RELAY_1_PORT                GPIOC
#define RELAY_1_PIN                 GPIO_Pin_3
#define RELAY_1_OUT(n)              { (n) ? ( GPIO_SetBits(RELAY_1_PORT,RELAY_1_PIN)) : (GPIO_ResetBits(RELAY_1_PORT,RELAY_1_PIN)); }
/**********  RELAY 2  ***********/
#define RELAY_2_PORT                GPIOC
#define RELAY_2_PIN                 GPIO_Pin_2
#define RELAY_2_OUT(n)              { (n) ? ( GPIO_SetBits(RELAY_2_PORT,RELAY_2_PIN)) : (GPIO_ResetBits(RELAY_2_PORT,RELAY_2_PIN)); }
/**********  RELAY 3  ***********/
#define RELAY_3_PORT                GPIOC
#define RELAY_3_PIN                 GPIO_Pin_1
#define RELAY_3_OUT(n)              { (n) ? ( GPIO_SetBits(RELAY_3_PORT,RELAY_3_PIN)) : (GPIO_ResetBits(RELAY_3_PORT,RELAY_3_PIN)); }
/**********  RELAY 4  ***********/
#define RELAY_4_PORT                GPIOC
#define RELAY_4_PIN                 GPIO_Pin_0
#define RELAY_4_OUT(n)              { (n) ? ( GPIO_SetBits(RELAY_4_PORT,RELAY_4_PIN)) : (GPIO_ResetBits(RELAY_4_PORT,RELAY_4_PIN)); }
/**********  OUTPUT 1  ***********/
#define OUTPUT_1_PORT               GPIOC
#define OUTPUT_1_PIN                GPIO_Pin_15
#define OUTPUT_1_OUT(n)             { (n) ? ( GPIO_SetBits(OUTPUT_1_PORT,OUTPUT_1_PIN)) : (GPIO_ResetBits(OUTPUT_1_PORT,OUTPUT_1_PIN)); }
/**********  OUTPUT 2  ***********/
#define OUTPUT_2_PORT               GPIOC
#define OUTPUT_2_PIN                GPIO_Pin_14
#define OUTPUT_2_OUT(n)             { (n) ? ( GPIO_SetBits(OUTPUT_2_PORT,OUTPUT_2_PIN)) : (GPIO_ResetBits(OUTPUT_2_PORT,OUTPUT_2_PIN)); }
/**********  OUTPUT 1  ***********/
#define OUTPUT_3_PORT               GPIOC
#define OUTPUT_3_PIN                GPIO_Pin_13
#define OUTPUT_3_OUT(n)             { (n) ? ( GPIO_SetBits(OUTPUT_3_PORT,OUTPUT_3_PIN)) : (GPIO_ResetBits(OUTPUT_3_PORT,OUTPUT_3_PIN)); }

#define DIGIT_OUTPUT_REG_LIST \
{\
    {\
        {\
            RELAY_1_PORT,RELAY_1_PIN \
        },\
        {\
            RELAY_2_PORT,RELAY_2_PIN \
        },\
        {\
            RELAY_3_PORT,RELAY_3_PIN \
        },\
        {\
            RELAY_4_PORT,RELAY_4_PIN \
        },\
        {\
            OUTPUT_1_PORT,OUTPUT_1_PIN \
        },\
        {\
            OUTPUT_2_PORT,OUTPUT_2_PIN \
        },\
        {\
            OUTPUT_3_PORT,OUTPUT_3_PIN \
        },\
    },\
    NULL,\
    GPIO_SetBits,\
    GPIO_ResetBits\
}\


/**************************************** USART ****************************************/
//USART1
#define USART1_PORT                     GPIOA
#define USART1_GPIO_APB                 RCC_APB2Periph_GPIOA
#define USART1_TX_PIN                   GPIO_Pin_9
#define USART1_RX_PIN                   GPIO_Pin_10
//#define USART1_RTS_PIN                  GPIO_Pin_7
//#define USART1_CTS_PIN                  GPIO_Pin_7
//USART2
#define USART2_PORT                     GPIOA
#define USART2_GPIO_APB                 RCC_APB2Periph_GPIOA
#define USART2_TX_PIN                   GPIO_Pin_2
#define USART2_RX_PIN                   GPIO_Pin_3
#define USART2_EN_PORT                  GPIOA
#define USART2_EN_PIN                   GPIO_Pin_1
#define USART2_485_TX_ENABLE 	        GPIO_SetBits(USART2_EN_PORT , USART2_EN_PIN)	//发送使能
#define USART2_485_RX_ENABLE	        GPIO_ResetBits(USART2_EN_PORT , USART2_EN_PIN)  	//接收使能
//#define USART2_RTS_PIN                  GPIO_Pin_1
//#define USART2_CTS_PIN                  GPIO_Pin_0
//USART3
#define USART3_PORT                     GPIOB
#define USART3_GPIO_APB                 RCC_APB2Periph_GPIOB
#define USART3_TX_PIN                   GPIO_Pin_10
#define USART3_RX_PIN                   GPIO_Pin_11
#define USART3_EN_PORT                  GPIOE
#define USART3_EN_PIN                   GPIO_Pin_15
#define USART3_485_TX_ENABLE 	        GPIO_SetBits(USART3_EN_PORT , USART3_EN_PIN)	//发送使能
#define USART3_485_RX_ENABLE	        GPIO_ResetBits(USART3_EN_PORT , USART3_EN_PIN)  	//接收使能
//#define USART3_RTS_PIN                  GPIO_Pin_14
//#define USART3_CTS_PIN                  GPIO_Pin_13
//UART4
#define UART4_PORT                      GPIOC
#define UART4_GPIO_APB                  RCC_APB2Periph_GPIOC
#define UART4_TX_PIN                    GPIO_Pin_10
#define UART4_RX_PIN                    GPIO_Pin_11
#define UART4_EN_PORT                   GPIOA
#define UART4_EN_PIN                    GPIO_Pin_12
#define UART4_485_TX_ENABLE 	        GPIO_SetBits(UART4_EN_PORT , UART4_EN_PIN)	//发送使能
#define UART4_485_RX_ENABLE	            GPIO_ResetBits(UART4_EN_PORT , UART4_EN_PIN)  	//接收使能
//UART5
#define UART5_PORT_TX                   GPIOC
#define UART5_GPIO_APB                  RCC_APB2Periph_GPIOC
#define UART5_TX_PIN                    GPIO_Pin_12
#define UART5_PORT_RX                   GPIOD
#define UART5_RX_PIN                    GPIO_Pin_2
#define UART5_EN_PORT                   GPIOE
#define UART5_EN_PIN                    GPIO_Pin_0
#define UART5_485_TX_ENABLE 	        GPIO_ResetBits(UART5_EN_PORT , UART5_EN_PIN)	//发送使能
#define UART5_485_RX_ENABLE	            GPIO_SetBits(UART5_EN_PORT , UART5_EN_PIN)  	//接收使能

/**************************************** CAN ****************************************/
#define CAN_TX_PIN		                GPIO_Pin_1
#define CAN_RX_PIN	                    GPIO_Pin_0
#define CAN_PORT     	                GPIOD
#define CAN_GPIO_APB                    RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO

/**************************************** Serve ****************************************/
///* GPIO Clock */
//#define SPI1_PERIPH_RELATE1             0
//#define SPI1_PERIPH_RELATE2             RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO
///* SPI1 PIN */
//#define SPI1_GPIO_PORT                  GPIOA
//#define SPI1_GPIO_PIN                   GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
///* W5500 CS */ 
//#define TCP_SLAVE_CS_PORT				GPIOD
//#define TCP_SLAVE_CS_PIN				GPIO_Pin_8
//#define TCP_SLAVE_CS_LOW				GPIO_ResetBits(TCP_SLAVE_CS_PORT, TCP_SLAVE_CS_PIN)
//#define TCP_SLAVE_CS_HIGH				GPIO_SetBits(TCP_SLAVE_CS_PORT, TCP_SLAVE_CS_PIN)
///* W5500 RST */
//#define TCP_SLAVE_RST_PORT				GPIOB
//#define TCP_SLAVE_RST_PIN 				GPIO_Pin_12
//#define TCP_SLAVE_RST_LOW               GPIO_ResetBits(TCP_SLAVE_RST_PORT, TCP_SLAVE_RST_PIN)
//#define TCP_SLAVE_RST_HIGH              GPIO_SetBits(TCP_SLAVE_RST_PORT, TCP_SLAVE_RST_PIN)
///* W5500 INT */
//#define TCP_SLAVE_INT_PORT				GPIOD
//#define TCP_SLAVE_INT_PIN				GPIO_Pin_9

/**************************************** SPI1 ****************************************/
/* GPIO Clock */
#define RCC_SPI2_RELATE_IO_ENABLE       RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,ENABLE)
/* SPI Clock */
#define RCC_SPI2_PERIPH_ENABLE          RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE)

/* SPI1 PIN */
#define SPI2_GPIO_PORT                  GPIOB
#define SPI2_GPIO_PIN                   GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15

/**************************************** W5500_1 ****************************************/
/* W5500片选 */
#define W5500_CS_PORT				    GPIOB
#define W5500_CS_PIN				    GPIO_Pin_12
#define W5500_CS_LOW				    GPIO_ResetBits(W5500_CS_PORT, W5500_CS_PIN)
#define W5500_CS_HIGH				    GPIO_SetBits(W5500_CS_PORT, W5500_CS_PIN)

/* W5500复位 */
#define W5500_RST_PORT				    GPIOD
#define W5500_RST_PIN 				    GPIO_Pin_8
#define W5500_RST_ENABLE                GPIO_ResetBits(W5500_RST_PORT, W5500_RST_PIN);
#define W5500_RST_DISABLE               GPIO_SetBits(W5500_RST_PORT, W5500_RST_PIN);

/* W5500中断 */
#define W5500_INT_PORT				    GPIOD
#define W5500_INT_PIN				    GPIO_Pin_9


typedef union {
u8 Byte;
struct {
		u8  bit0             :1;
		u8  bit1             :1;
		u8  bit2             :1;
		u8  bit3             :1;
		u8  bit4             :1;
		u8  bit5             :1;
		u8  bit6             :1;
		u8  bit7             :1;
} Bits;
}Tdef_Byte;

/************************************************************/
#define PERIPH_CLOCK_IO_ALL_ENABLE RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE,ENABLE)

/*********************************   INPUT   *********************************/
//#define INPUT_TRIGGER_INTERRUPT   //INPUT_TRIGGER_FILTER

#define DIGIT_INPUT_CHN_NUM             16U

/****  	MCU_IN 1	****/
#define MCU_IN_1_PORT				    GPIOC
#define MCU_IN_1_PIN 				    GPIO_Pin_7
#define GET_INPUT_1_STATUS              GPIO_ReadInputDataBit(MCU_IN_1_PORT,MCU_IN_1_PIN)

/****  	MCU_IN 2	****/
#define MCU_IN_2_PORT				    GPIOC
#define MCU_IN_2_PIN 				    GPIO_Pin_8
#define GET_INPUT_2_STATUS              GPIO_ReadInputDataBit(MCU_IN_2_PORT,MCU_IN_2_PIN)

/****  	MCU_IN 3	****/
#define MCU_IN_3_PORT				    GPIOC
#define MCU_IN_3_PIN 				    GPIO_Pin_9
#define GET_INPUT_3_STATUS              GPIO_ReadInputDataBit(MCU_IN_3_PORT,MCU_IN_3_PIN)

/****  	MCU_IN 4	****/
#define MCU_IN_4_PORT				    GPIOA
#define MCU_IN_4_PIN 				    GPIO_Pin_8
#define GET_INPUT_4_STATUS              GPIO_ReadInputDataBit(MCU_IN_4_PORT,MCU_IN_4_PIN)

/****  	MCU_IN 5	****/
#define MCU_IN_5_PORT				    GPIOA
#define MCU_IN_5_PIN 				    GPIO_Pin_11
#define GET_INPUT_5_STATUS              GPIO_ReadInputDataBit(MCU_IN_5_PORT,MCU_IN_5_PIN)

/****  	MCU_IN 1	****/
#define MCU_IN_6_PORT				    GPIOD
#define MCU_IN_6_PIN 				    GPIO_Pin_3
#define GET_INPUT_6_STATUS              GPIO_ReadInputDataBit(MCU_IN_6_PORT,MCU_IN_6_PIN)

/****  	MCU_IN 7	****/
#define MCU_IN_7_PORT				    GPIOD
#define MCU_IN_7_PIN 				    GPIO_Pin_4
#define GET_INPUT_7_STATUS              GPIO_ReadInputDataBit(MCU_IN_7_PORT,MCU_IN_7_PIN)

/****  	MCU_IN 8	****/
#define MCU_IN_8_PORT				    GPIOD
#define MCU_IN_8_PIN 				    GPIO_Pin_5
#define GET_INPUT_8_STATUS              GPIO_ReadInputDataBit(MCU_IN_8_PORT,MCU_IN_8_PIN)

/****  	MCU_IN 9	****/
#define MCU_IN_9_PORT				    GPIOD
#define MCU_IN_9_PIN 				    GPIO_Pin_6
#define GET_INPUT_9_STATUS              GPIO_ReadInputDataBit(MCU_IN_9_PORT,MCU_IN_9_PIN)

/****  	MCU_IN 10	****/
#define MCU_IN_10_PORT				    GPIOD
#define MCU_IN_10_PIN 				    GPIO_Pin_7
#define GET_INPUT_10_STATUS             GPIO_ReadInputDataBit(MCU_IN_10_PORT,MCU_IN_10_PIN)

/****  	MCU_IN 11	****/
#define MCU_IN_11_PORT				    GPIOE
#define MCU_IN_11_PIN 				    GPIO_Pin_1
#define GET_INPUT_11_STATUS             GPIO_ReadInputDataBit(MCU_IN_11_PORT,MCU_IN_11_PIN)

/****  	MCU_IN 12	****/
#define MCU_IN_12_PORT				    GPIOE
#define MCU_IN_12_PIN 				    GPIO_Pin_2
#define GET_INPUT_12_STATUS             GPIO_ReadInputDataBit(MCU_IN_12_PORT,MCU_IN_12_PIN)

/****  	MCU_IN 13	****/
#define MCU_IN_13_PORT				    GPIOE
#define MCU_IN_13_PIN 				    GPIO_Pin_3
#define GET_INPUT_13_STATUS             GPIO_ReadInputDataBit(MCU_IN_13_PORT,MCU_IN_13_PIN)

/****  	MCU_IN 14	****/
#define MCU_IN_14_PORT				    GPIOE
#define MCU_IN_14_PIN 				    GPIO_Pin_4
#define GET_INPUT_14_STATUS             GPIO_ReadInputDataBit(MCU_IN_14_PORT,MCU_IN_14_PIN)

/****  	MCU_IN 15	****/
#define MCU_IN_15_PORT				    GPIOE
#define MCU_IN_15_PIN 				    GPIO_Pin_5
#define GET_INPUT_15_STATUS             GPIO_ReadInputDataBit(MCU_IN_15_PORT,MCU_IN_15_PIN)

/****  	MCU_IN 16	****/
#define MCU_IN_16_PORT				    GPIOE
#define MCU_IN_16_PIN 				    GPIO_Pin_6
#define GET_INPUT_16_STATUS             GPIO_ReadInputDataBit(MCU_IN_16_PORT,MCU_IN_16_PIN)

/****  	MCU_IN 17	****/
#define MCU_IN_17_PORT				    GPIOE
#define MCU_IN_17_PIN 				    GPIO_Pin_6
#define GET_INPUT_17_STATUS             GPIO_ReadInputDataBit(MCU_IN_17_PORT,MCU_IN_17_PIN)

#define DIGIT_INPUT_REG_LIST \
{\
    {\
        {\
            MCU_IN_1_PORT,MCU_IN_1_PIN\
        },\
        {\
            MCU_IN_2_PORT,MCU_IN_2_PIN\
        },\
        {\
            MCU_IN_3_PORT,MCU_IN_3_PIN\
        },\
        {\
            MCU_IN_4_PORT,MCU_IN_4_PIN\
        },\
        {\
            MCU_IN_5_PORT,MCU_IN_5_PIN\
        },\
        {\
            MCU_IN_6_PORT,MCU_IN_6_PIN\
        },\
        {\
            MCU_IN_7_PORT,MCU_IN_7_PIN\
        },\
        {\
            MCU_IN_8_PORT,MCU_IN_8_PIN\
        },\
        {\
            MCU_IN_9_PORT,MCU_IN_9_PIN\
        },\
        {\
            MCU_IN_10_PORT,MCU_IN_10_PIN\
        },\
        {\
            MCU_IN_11_PORT,MCU_IN_11_PIN\
        },\
        {\
            MCU_IN_12_PORT,MCU_IN_12_PIN\
        },\
        {\
            MCU_IN_13_PORT,MCU_IN_13_PIN\
        },\
        {\
            MCU_IN_14_PORT,MCU_IN_14_PIN\
        },\
        {\
            MCU_IN_15_PORT,MCU_IN_15_PIN\
        },\
        {\
            MCU_IN_16_PORT,MCU_IN_16_PIN\
        }\
    },\
    GPIO_ReadInputDataBit,\
    NULL,\
    NULL\
}\

/*********************************   DIP_SWITCH   *********************************/
#define DIP_SWITCH_BITS_NUM             8U

/****  	DIP_SWITCH_1	****/
#define DIP_SWITCH_1_PORT				GPIOC
#define DIP_SWITCH_1_PIN 				GPIO_Pin_6
#define GET_DIP_SWITCH_1_STATUS         GPIO_ReadInputDataBit(DIP_SWITCH_1_PORT,DIP_SWITCH_1_PIN)
/****  	DIP_SWITCH_2	****/
#define DIP_SWITCH_2_PORT				GPIOD
#define DIP_SWITCH_2_PIN 				GPIO_Pin_15
#define GET_DIP_SWITCH_2_STATUS         GPIO_ReadInputDataBit(DIP_SWITCH_2_PORT,DIP_SWITCH_2_PIN)
/****  	DIP_SWITCH_3	****/
#define DIP_SWITCH_3_PORT				GPIOD
#define DIP_SWITCH_3_PIN 				GPIO_Pin_14
#define GET_DIP_SWITCH_3_STATUS         GPIO_ReadInputDataBit(DIP_SWITCH_3_PORT,DIP_SWITCH_3_PIN)
/****  	DIP_SWITCH_4	****/
#define DIP_SWITCH_4_PORT				GPIOD
#define DIP_SWITCH_4_PIN 				GPIO_Pin_13
#define GET_DIP_SWITCH_4_STATUS         GPIO_ReadInputDataBit(DIP_SWITCH_4_PORT,DIP_SWITCH_4_PIN)
/****  	DIP_SWITCH_5	****/
#define DIP_SWITCH_5_PORT				GPIOD
#define DIP_SWITCH_5_PIN 				GPIO_Pin_12
#define GET_DIP_SWITCH_5_STATUS         GPIO_ReadInputDataBit(DIP_SWITCH_5_PORT,DIP_SWITCH_5_PIN)
/****  	DIP_SWITCH_6	****/
#define DIP_SWITCH_6_PORT				GPIOD
#define DIP_SWITCH_6_PIN 				GPIO_Pin_11
#define GET_DIP_SWITCH_6_STATUS         GPIO_ReadInputDataBit(DIP_SWITCH_6_PORT,DIP_SWITCH_6_PIN)
/****  	DIP_SWITCH_7	****/
#define DIP_SWITCH_7_PORT				GPIOE
#define DIP_SWITCH_7_PIN 				GPIO_Pin_11
#define GET_DIP_SWITCH_7_STATUS         GPIO_ReadInputDataBit(DIP_SWITCH_7_PORT,DIP_SWITCH_7_PIN)
/****  	DIP_SWITCH_8	****/
#define DIP_SWITCH_8_PORT				GPIOE
#define DIP_SWITCH_8_PIN 				GPIO_Pin_10
#define GET_DIP_SWITCH_8_STATUS         GPIO_ReadInputDataBit(DIP_SWITCH_8_PORT,DIP_SWITCH_8_PIN)


#define DIP_SWITCH_REG_LIST  {\
    {\
        {\
            DIP_SWITCH_1_PORT,DIP_SWITCH_1_PIN\
        },\
        {\
            DIP_SWITCH_2_PORT,DIP_SWITCH_2_PIN\
        },\
        {\
            DIP_SWITCH_3_PORT,DIP_SWITCH_3_PIN\
        },\
        {\
            DIP_SWITCH_4_PORT,DIP_SWITCH_4_PIN\
        },\
        {\
            DIP_SWITCH_5_PORT,DIP_SWITCH_5_PIN\
        },\
        {\
            DIP_SWITCH_6_PORT,DIP_SWITCH_6_PIN\
        },\
        {\
            DIP_SWITCH_7_PORT,DIP_SWITCH_7_PIN\
        },\
        {\
            DIP_SWITCH_7_PORT,DIP_SWITCH_8_PIN\
        }\
    },\
    GPIO_ReadInputDataBit,\
    NULL,\
    NULL\
}\

/*********************************   OUTPUT   *********************************/
/****  	MCU_OUTPUT 1	****/
#define MCU_OUTPUT_1_PORT				GPIOC
#define MCU_OUTPUT_1_PIN 				GPIO_Pin_15
#define SET_OUTPUT_1_LOW             	GPIO_ResetBits(MCU_OUTPUT_1_PORT, MCU_OUTPUT_1_PIN);
#define SET_OUTPUT_1_HIGH             	GPIO_SetBits(MCU_OUTPUT_1_PORT, MCU_OUTPUT_1_PIN);
/****  	MCU_OUTPUT 2	****/
#define MCU_OUTPUT_2_PORT				GPIOC
#define MCU_OUTPUT_2_PIN 				GPIO_Pin_14
#define SET_OUTPUT_2_LOW             	GPIO_ResetBits(MCU_OUTPUT_1_PORT, MCU_OUTPUT_1_PIN);
#define SET_OUTPUT_2_HIGH             	GPIO_SetBits(MCU_OUTPUT_1_PORT, MCU_OUTPUT_1_PIN);
/****  	MCU_OUTPUT 3	****/
#define MCU_OUTPUT_3_PORT				GPIOC
#define MCU_OUTPUT_3_PIN 				GPIO_Pin_13
#define SET_OUTPUT_3_LOW             	GPIO_ResetBits(MCU_OUTPUT_1_PORT, MCU_OUTPUT_1_PIN);
#define SET_OUTPUT_3_HIGH             	GPIO_SetBits(MCU_OUTPUT_1_PORT, MCU_OUTPUT_1_PIN);
/****  	RELAY_OUTPUT 1	****/
#define MCU_RELAY_OUT_1_PORT			GPIOC
#define MCU_RELAY_OUT_1_PIN 			GPIO_Pin_3
#define SET_RELAY_OUT_1_LOW             GPIO_ResetBits(MCU_RELAY_OUT_1_PORT, MCU_RELAY_OUT_1_PIN);
#define SET_RELAY_OUT_1_HIGH            GPIO_SetBits(MCU_RELAY_OUT_1_PORT, MCU_RELAY_OUT_1_PIN);
/****  	RELAY_OUTPUT 2	****/
#define MCU_RELAY_OUT_2_PORT			GPIOC
#define MCU_RELAY_OUT_2_PIN 			GPIO_Pin_2
#define SET_RELAY_OUT_2_LOW             GPIO_ResetBits(MCU_RELAY_OUT_1_PORT, MCU_RELAY_OUT_1_PIN);
#define SET_RELAY_OUT_2_HIGH            GPIO_SetBits(MCU_RELAY_OUT_1_PORT, MCU_RELAY_OUT_1_PIN);
/****  	RELAY_OUTPUT 3	****/
#define MCU_RELAY_OUT_3_PORT			GPIOC
#define MCU_RELAY_OUT_3_PIN 			GPIO_Pin_1
#define SET_RELAY_OUT_3_LOW             GPIO_ResetBits(MCU_RELAY_OUT_1_PORT, MCU_RELAY_OUT_1_PIN);
#define SET_RELAY_OUT_3_HIGH            GPIO_SetBits(MCU_RELAY_OUT_1_PORT, MCU_RELAY_OUT_1_PIN);
/****  	RELAY_OUTPUT 4	****/
#define MCU_RELAY_OUT_4_PORT			GPIOC
#define MCU_RELAY_OUT_4_PIN 			GPIO_Pin_0
#define SET_RELAY_OUT_4_LOW             GPIO_ResetBits(MCU_RELAY_OUT_1_PORT, MCU_RELAY_OUT_1_PIN);
#define SET_RELAY_OUT_4_HIGH            GPIO_SetBits(MCU_RELAY_OUT_1_PORT, MCU_RELAY_OUT_1_PIN);




#endif


