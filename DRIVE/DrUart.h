#ifndef __DR_UART_H
#define __DR_UART_H

#include "stm32f10x.h"

void USART1_IRQ(unsigned char data);
void USART2_IRQ(unsigned char data);
void USART3_IRQ(unsigned char data);
void UART4_IRQ(unsigned char data);
void UART5_IRQ(unsigned char data);

void USART1_Send_Data(u8 *send_buff,u16 length);
void USART2_Send_Data(u8 *send_buff,u16 length);
void USART3_Send_Data(u8 *send_buff,u16 length);
void UART4_Send_Data(u8 *send_buff,u16 length);
void UART5_Send_Data(u8 *send_buff,u16 length);

void DrUsart_Init(void);
void USART_Timer100us(void);

#endif


