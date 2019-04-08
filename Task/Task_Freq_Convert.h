#ifndef __TASK_FREQ_CONVERT_H
#define __TASK_FREQ_CONVERT_H

#define SLAVEID_FREQ                                    1U  //变频器从节点ID

#define MAX_RUNNING_FREQ                                50  //电机最大运行频率
#define MOTOR_SPEED                                    995  //电机转速
#define DIAMETER_REDUCER                             126.5  //减速机直径
#define DIAMETER_WIRE                                   10  //线缆直径
#define PULSE_PER_CYCLE                               2000  //每转脉冲数
#define ENCODER_DIOMETER                                 1  //编码器上的转盘直径mm,1表示编码器直接接到减速机上，该值待定
#define REDUCTION_RATIO                                 50  //减速比

#define INIT_POSITION_WIRE                           30000  //缆绳初始位置
#define BAND_TYPE_BRAKE_DELAY_THRES                     20  //2s,电机运行后2s抱闸松开（继电器闭合）

enum Invertor_Offset{//变频器状态参数地址
    off_InvertorError=0,
};

enum Motor_Command{
    Motor_Fardward_Run=1,Motor_Backward_Run,Motor_Fardward_PointMove,Motor_Backward_PointMove,
    Motor_Stop_Reduce,Motor_Stop_Free
};

typedef struct Invertor_Status_Group
{
    s16 Suspende_Position;//吊杆当前位置,单位mm
    u16 Suspende_Running_Status; //吊杆运行状态
    volatile BitStatus  Running_Sts[2];  //故障代码
}Invertor_Status_Info;
//extern Invertor_Status_Info Invertor_Status;

//extern volatile digitstatus    	        _Running_Error_Sts[6];
//#define Running_Error_Sts(n)    _Running_Error_Sts[n].bytetype
extern volatile BitStatus Invertor_Status;
#define CMD_Rope_Wire  		                Invertor_Status.Bits.bit0 //0-对单个吊杆的收揽命令
#define CMD_Suspender_Min  		            Invertor_Status.Bits.bit1 //1-将单个吊杆运行到零位位置
#define CMD_Suspender_Emergency_Stop  		Invertor_Status.Bits.bit2 //2-对单个吊杆的急停命令
#define CMD_Suspender_Target  		        Invertor_Status.Bits.bit3 //3-将吊杆运行到目标坐标位置
#define CMD_ParaDownload_Independent  	    Invertor_Status.Bits.bit4 //4-微控制器个性化参数下载（数据待定）
#define CMD_Read_Common_Para       	        Invertor_Status.Bits.bit5 //5-读某个微控制器的共性参数（数据待定）
#define CMD_Read_Independent_Para      	    Invertor_Status.Bits.bit6 //6-读某个微控制器的个性化参数（数据待定）
#define CMD_ParaDownload_Common             Invertor_Status.Bits.bit7 //7-预留


extern u16 InvertorData[40];

void Task_Freq_Convert(void *p_arg);
void TaskFreq_Timer100ms(void);

#endif


