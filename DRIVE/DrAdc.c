
#include "main.h"
#ifdef _DRADC_H_

u16 AD_Convert[Sample_Num][Channel_Num];
unsigned int AD_SingleConvertion(void)
{
	/*while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	AD_Convert = ADC_GetConversionValue(ADC1);
	//ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	return AD_Convert;*/
	//ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	//ReadADCAverageValue();
	return 0;
}

void ADC1_DMA_Config(void)
{
 DMA_InitTypeDef DMA_InitStructure;
              
 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
 DMA_DeInit(DMA1_Channel1);
 DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; //ADC��ַ
 DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Convert;  //�ڴ��ַ��ȫ�ֱ�����
 DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;        //����Ϊ����Դ����ΪDMA������˫��ģ���Ϊ��������ԴΪ���裩
 DMA_InitStructure.DMA_BufferSize = Sample_Num*Channel_Num;
 DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ���䣨����ΪADC1��
 DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ����
 DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
 DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
 DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
 DMA_InitStructure.DMA_Priority = DMA_Priority_High;
 DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
 DMA_Init(DMA1_Channel1, &DMA_InitStructure);
 DMA_Cmd(DMA1_Channel1,ENABLE);
}



void DrAdc(void)
{
 GPIO_InitTypeDef GPIO_InitStructure;
 ADC_InitTypeDef ADC_InitStructure;
  
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
  
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_14;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  //ģ������ģʽ
 GPIO_Init(GPIOC, &GPIO_InitStructure);
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  //ģ������ģʽ
 GPIO_Init(GPIOB, &GPIO_InitStructure);
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  //ģ������ģʽ
 GPIO_Init(GPIOA, &GPIO_InitStructure);
 ADC1_DMA_Config();
  
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
// ADC_DeInit(ADC1);
 ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC1��ADC2�����ڶ���ģʽ
 ADC_InitStructure.ADC_ScanConvMode = ENABLE;//��ͨ��
 ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//����ת��
 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//�������ת��
 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ת������Ҷ���
 ADC_InitStructure.ADC_NbrOfChannel = Channel_Num;//ͨ����Ŀ
 ADC_Init(ADC1, &ADC_InitStructure); 
  
 RCC_ADCCLKConfig(RCC_PCLK2_Div6); //PCLK 6��Ƶ
 ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);//ͨ����ת������ת��ʱ��
 ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 2, ADC_SampleTime_55Cycles5);
 ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 3, ADC_SampleTime_55Cycles5);
 ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_55Cycles5);
 ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 5, ADC_SampleTime_55Cycles5);
 ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 6, ADC_SampleTime_55Cycles5);
 ADC_TempSensorVrefintCmd(ENABLE); 
  
 ADC_DMACmd(ADC1, ENABLE);
  
 ADC_Cmd(ADC1, ENABLE);
 ADC_ResetCalibration(ADC1);
 while(ADC_GetResetCalibrationStatus(ADC1));
 ADC_StartCalibration(ADC1);//��ʼУ׼
 while(ADC_GetCalibrationStatus(ADC1));
 ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ʹ��ADC�����ת����������
}

void ReadADCAverageValue(uint8_t sch_timer,uint8_t sch_cycle)
{
	unsigned char i,j;
	unsigned long sum = 0;
	if(sch_timer%sch_cycle == 0)
	{
		for(i=0; i<Channel_Num;i++)
		{
			for(j=0; j<Sample_Num;j++)
			{
				sum+=AD_Convert[j][i]&0xfff;
			}
			Global_Driver.AD_Result[i]=sum/Sample_Num;
			sum=0;
		}
	}
}

#endif

