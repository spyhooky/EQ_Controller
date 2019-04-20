#ifndef __TASK_FREQ_CONVERT_H
#define __TASK_FREQ_CONVERT_H

#define SLAVEID_FREQ                                    1U  //��Ƶ���ӽڵ�ID

#define MAX_RUNNING_FREQ                               120  //����������Ƶ��
#define MOTOR_SPEED                                    995  //���ת��
#define DIAMETER_REDUCER                     (float)0.1265  //���ٻ�ֱ��
#define DIAMETER_WIRE                          (float)0.01  //����ֱ��
#define PULSE_PER_CYCLE                               2000  //ÿת������
#define ENCODER_GEAR_NUM                                47  //������������
#define REDUCTION_GEAR_NUM                              71  //���ٻ�������
#define REDUCTION_RATIO                                 50  //���ٱ�

#define REDUCTION_SPEED             ((float)((float)MOTOR_SPEED/(float)REDUCTION_RATIO))   //���ٻ�ת��
#define LENTH_REDUCTION_PER_MINUTE    ((float)(REDUCTION_SPEED*(DIAMETER_REDUCER+DIAMETER_WIRE)*3.14))   //���ٻ�ÿ�������еĳ���
#define ENCODER_SPEED               (REDUCTION_SPEED/((float)ENCODER_GEAR_NUM/(float)REDUCTION_GEAR_NUM))   //������ת��
#define LENTH_PER_PULSE             ((LENTH_REDUCTION_PER_MINUTE/ENCODER_SPEED)/((float)PULSE_PER_CYCLE/1000)) //������ÿ�����Ӧ��˿�����ߵĳ���L,��λΪmm

#define INIT_POSITION_WIRE                           30000  //������ʼλ��
#define BAND_TYPE_BRAKE_DELAY_THRES                     20  //2s,������к�2s��բ�ɿ����̵����պϣ�

#define FREQ_REDUCE_BASE                                5U  //ÿ�μ��ٵ�Ƶ�ʻ�׼ֵ����λ100ms
#define READ8000_INTERTER                               5U  //��ѯ֡���ڣ���λ100ms
#define FREQ_REDUCE_INTERTER                            5U  //Ƶ�ʼ���ʱ������ÿ��100ms��5hz
#define FORCE_REDUCE_10HZ_KEEPING                       5U  //ǿ�Ƽ��ٵ�10HZʱ��Ҫ���ֵ�ʱ�䣬��λ100ms

enum Invertor_Offset{//��Ƶ��״̬������ַ
    off_InvertorError=0,off_CurrFreq,
    NUM_Read_Total
};
enum Write_Data_Off{
    Control_CMD,Convert_Freq,
    NUM_Write_Total
};

enum Diretor_Info{//��Ƶ��״̬������ַ
    D_FALL=0,D_RISE
};//����-�����˶�����0mm��30000mm������˶�������-�����˶�

enum Motor_Command{
    Motor_Backward_Run=1,Motor_Fardward_Run,Motor_Fardward_PointMove,Motor_Backward_PointMove,
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


extern u16 InvertorData[NUM_Read_Total];

void Task_Freq_Convert(void *p_arg);
void TaskFreq_Timer100ms(void);

#endif


