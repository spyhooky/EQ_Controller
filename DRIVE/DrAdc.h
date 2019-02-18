
#ifndef  _DRADC_H_											 
#define  _DRADC_H_
#include "stm32f10x.h"

#define Sample_Num  10


void DrAdc(void);
unsigned int AD_SingleConvertion(void);
void ReadADCAverageValue(uint8_t sch_timer,uint8_t sch_cycle);

void ADC_SingleModeConfiguration(unsigned char channel);
unsigned int AD_SingleConvertion(void);
void ADC1_GPIO_Config(void);
void ADC1_Mode_Config(void);
void ADC1_DMA_Config(void);

#endif
