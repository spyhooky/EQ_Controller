#ifndef __TASK_FREQ_CONVERT_H
#define __TASK_FREQ_CONVERT_H

#define SLAVEID_FREQ                                    1U  //��Ƶ���ӽڵ�ID

#define MAX_RUNNING_FREQ                               100  //����������Ƶ��
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

#define INIT_POSITION_WIRE                           5000  //����Ĭ�ϳ�ʼλ��
#define BAND_TYPE_BRAKE_DELAY_THRES                   300U  //300ms,�������300ms��բ�ɿ����̵����պϣ�

#define FREQ_STARTMOVE_BASE                             10//������е���ʼƵ���Լ����ֹͣʱ��������Ƶ��
#define FREQ_CHANGE_BASE                                2  //ÿ�μ��ٵ�Ƶ�ʻ�׼ֵ����λ5HZ,ʱ�䵥λ��FREQ_REDUCE_INTERTER
#define FREQ_REDUCE_INTERTER                          300U  //Ƶ�ʼ���ʱ������ÿ��100ms��5hz
#define FORCE_REDUCE_10HZ_KEEPING                     500U  //ǿ�Ƽ��ٵ�10HZʱ��Ҫ���ֵ�ʱ�䣬��λ100ms
#define READ8000_INTERTER                             500U  //��ѯ֡���ڣ���λ100ms

#define REMAIN_PULSE_NUMBER_FOR_BRAKE                   50  //��Ҫ�ɱ�բʱʣ��������
#define REMAIN_PULSE_NUMBER_FOR_FREQ_STOP               12  //�б�Ƶ������ʱ��ʼͣ����ʣ��������
#define REMAIN_PULSE_NUMBER_FOR_STOP                    30  //�ޱ�Ƶ������ʱ��ʼͣ����ʣ��������

#define RESET_ERROR_DELAY_THRES                       600U  //��ͣ��λ�������״̬���ӳ�ʱ��

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
    Motor_Stop_Reduce,Motor_Stop_Free,Error_Reset
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
extern volatile BitStatus Invertor_Status[2];
#define CMD_Rope_Wire  		                Invertor_Status[0].Bits.bit0 //0-�Ե������˵���������
#define CMD_Suspender_Min  		            Invertor_Status[0].Bits.bit1 //1-�������������е���λλ��
#define CMD_Suspender_Emergency_Stop  		Invertor_Status[0].Bits.bit2 //2-�Ե������˵ļ�ͣ����
#define CMD_Suspender_Target  		        Invertor_Status[0].Bits.bit3 //3-���������е�Ŀ������λ��
#define CMD_ParaDownload_Independent  	    Invertor_Status[0].Bits.bit4 //4-΢���������Ի��������أ����ݴ�����
#define CMD_Read_Common_Para       	        Invertor_Status[0].Bits.bit5 //5-��ĳ��΢�������Ĺ��Բ��������ݴ�����
#define CMD_Read_Independent_Para      	    Invertor_Status[0].Bits.bit6 //6-��ĳ��΢�������ĸ��Ի����������ݴ�����
#define CMD_Suspender_Init                  Invertor_Status[0].Bits.bit7 //7-���˳�ʼ��

#define CMD_ParaDownload_Common             Invertor_Status[1].Bits.bit0 //0-���ع��Բ���
#define CMD_Limit_Measure                   Invertor_Status[1].Bits.bit1 //0-������λλ�ò���
#define CMD_Read_Limit_Result               Invertor_Status[1].Bits.bit2 //0-��ȡ������λλ������

enum Limit_Measure_STS_T{
    M_IDLE,M_MEASURING,M_SUCCESS,M_FAIL
};

extern u8 Limit_Measure_Status;//�����г���λ����״̬ 0-���У�1-�����У�2-�����ɹ���3-����ʧ��
extern u16 InvertorData[NUM_Read_Total];//��Ƶ���Ĵ�����ȡ���

void Task_Freq_Convert(void *p_arg);
void TaskFreq_Timer1ms(void);

#endif


