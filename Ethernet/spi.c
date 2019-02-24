/**
  ******************************************************************************
  * @file    spi.c
  * $Author: Variable $
  * $Revision: 17 $
  * $Date:: 2016-4-2 11:16:48 +0800 #$
  * @brief   SPI��������ʵ��.
  ******************************************************************************
  * @attention
  *
  *<h3><center>&copy; Copyright 2009-2012, EmbedNet</center>
  *<center><a href="http:\\www.embed-net.com">http://www.embed-net.com</a></center>
  *<center>All Rights Reserved</center></h3>
  * 
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  ʹ��SPIʱ��
  * @retval None
  */
static void SPI_RCC_Configuration(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);	
    RCC_APB1PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
}
/**
  * @brief  ����ָ��SPI������
  * @retval None
  */
static void SPI_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//PA4->CS,PA5->SCK,PA6->MISO,PA7->MOSI		 					 
	GPIO_InitStructure.GPIO_Pin = SPI2_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI2_GPIO_PORT, &GPIO_InitStructure);
	GPIO_SetBits(SPI2_GPIO_PORT,SPI2_GPIO_PIN);
	//��ʼ��Ƭѡ�������
	GPIO_InitStructure.GPIO_Pin = W5500_CS_PIN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Init(W5500_CS_PORT, &GPIO_InitStructure);
	GPIO_SetBits(W5500_CS_PORT, W5500_CS_PIN);
}
/**
  * @brief  �����ⲿSPI�豸����SPI��ز���
  * @retval None
  */
void SPI_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStruct;

	SPI_RCC_Configuration();
	SPI_GPIO_Configuration();
	
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2,&SPI_InitStruct);
	
	SPI_Cmd(SPI2, ENABLE);
}
/**
  * @brief  д1�ֽ����ݵ�SPI����
  * @param  TxData д�����ߵ�����
  * @retval None
  */
void SPI_WriteByte(uint8_t TxData)
{				 
	while((SPI2->SR&SPI_I2S_FLAG_TXE)==0);	//�ȴ���������		  
	SPI2->DR=TxData;	 	  									//����һ��byte 
	while((SPI2->SR&SPI_I2S_FLAG_RXNE)==0); //�ȴ�������һ��byte  
	SPI2->DR;		
}
/**
  * @brief  ��SPI���߶�ȡ1�ֽ�����
  * @retval ����������
  */
uint8_t SPI_ReadByte(void)
{			 
	while((SPI2->SR&SPI_I2S_FLAG_TXE)==0);	//�ȴ���������			  
	SPI2->DR=0xFF;	 	  										//����һ�������ݲ����������ݵ�ʱ�� 
	while((SPI2->SR&SPI_I2S_FLAG_RXNE)==0); //�ȴ�������һ��byte  
	return SPI2->DR;  						    
}
/**
  * @brief  �����ٽ��� 
  * @retval None
  */
void SPI_CrisEnter(void)
{
	__set_PRIMASK(1);
}
/**
  * @brief  �˳��ٽ���
  * @retval None
  */
void SPI_CrisExit(void)
{
	__set_PRIMASK(0);
}

/**
  * @brief  Ƭѡ�ź�����͵�ƽ
  * @retval None
  */
void SPI_CS_Select(void)
{
	GPIO_ResetBits(W5500_CS_PORT,W5500_CS_PIN);
}
/**
  * @brief  Ƭѡ�ź�����ߵ�ƽ
  * @retval None
  */
void SPI_CS_Deselect(void)
{
	GPIO_SetBits(W5500_CS_PORT,W5500_CS_PIN);
}
/*********************************END OF FILE**********************************/

