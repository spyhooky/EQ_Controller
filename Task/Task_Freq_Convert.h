#ifndef __TASK_FREQ_CONVERT_H
#define __TASK_FREQ_CONVERT_H

#define SLAVEID_FREQ                                    1U  //��Ƶ���ӽڵ�ID

#define MAX_RUNNING_FREQ                                50  //����������Ƶ��
#define MOTOR_SPEED                                    995  //���ת��
#define DIAMETER_REDUCER                             126.5  //���ٻ�ֱ��
#define DIAMETER_WIRE                                   10  //����ֱ��
#define PULSE_PER_CYCLE                               2000  //ÿת������
#define ENCODER_DIOMETER                                 1  //�������ϵ�ת��ֱ��mm,1��ʾ������ֱ�ӽӵ����ٻ��ϣ���ֵ����
#define REDUCTION_RATIO                                 50  //���ٱ�

#define INIT_POSITION_WIRE                           30000  //������ʼλ��
#define BAND_TYPE_BRAKE_DELAY_THRES                     20  //2s,������к�2s��բ�ɿ����̵����պϣ�

enum Invertor_Offset{//��Ƶ��״̬������ַ
    off_InvertorError=0,
};

enum Motor_Command{
    Motor_Fardward_Run=1,Motor_Backward_Run,Motor_Fardward_PointMove,Motor_Backward_PointMove,
    Motor_Stop_Reduce,Motor_Stop_Free
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


extern u16 InvertorData[40];

void Task_Freq_Convert(void *p_arg);
void TaskFreq_Timer100ms(void);

#endif


