#ifndef __TASK_FREQ_CONVERT_H
#define __TASK_FREQ_CONVERT_H


enum Invertor_Offset{//��Ƶ��״̬������ַ
    off_FreqByComm=0,   //ͨѶ����Ƶ��-10000~1000 `��ʮ���ƣ�
    off_RunningFreq,
    off_BusVoltage,
    off_OutVoltage,
    off_OutCurrent,
    off_OutPower,

    off_InvertorError=0x21,
};

typedef struct Invertor_Status_Group
{
    s16 Suspende_Position;//���˵�ǰλ��,��λmm
    u16 Suspende_Running_Status; //��������״̬
    volatile BitStatus  Running_Sts[2];  //���ϴ���
}Invertor_Status_Info;
//extern Invertor_Status_Info Invertor_Status;

//extern volatile digitstatus    	        _Running_Error_Sts[6];
//#define Running_Error_Sts(n)    _Running_Error_Sts[n].bytetype
extern volatile BitStatus Invertor_Status;
#define CMD_Rope_Wire  		                Invertor_Status.Bits.bit0 //0-�Ե������˵���������
#define CMD_Suspender_Min  		            Invertor_Status.Bits.bit1 //1-�������������е���λλ��
#define CMD_Suspender_Emergency_Stop  		Invertor_Status.Bits.bit2 //2-�Ե������˵ļ�ͣ����
#define CMD_Suspender_Target  		        Invertor_Status.Bits.bit3 //3-���������е�Ŀ������λ��
#define CMD_ParaDownload_Independent  	    Invertor_Status.Bits.bit4 //4-΢���������Ի��������أ����ݴ�����
#define CMD_Read_Common_Para       	        Invertor_Status.Bits.bit5 //5-��ĳ��΢�������Ĺ��Բ��������ݴ�����
#define CMD_Read_Independent_Para      	    Invertor_Status.Bits.bit6 //6-��ĳ��΢�������ĸ��Ի����������ݴ�����
#define CMD_ParaDownload_Common             Invertor_Status.Bits.bit7 //7-Ԥ��


extern u8 InvertorData[80];

void Task_Freq_Convert(void *p_arg);


#endif


