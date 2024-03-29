#ifndef __TASK_FREQ_CONVERT_H
#define __TASK_FREQ_CONVERT_H

#define SLAVEID_FREQ                                    1U  //变频器从节点ID

#define MAX_RUNNING_FREQ                               100  //电机最大运行频率
#define MOTOR_SPEED                                    995  //电机转速
#define DIAMETER_REDUCER                     (float)0.1265  //减速机直径
#define DIAMETER_WIRE                          (float)0.01  //线缆直径
#define PULSE_PER_CYCLE                               2000  //每转脉冲数
#define ENCODER_GEAR_NUM                                47  //编码器齿轮数
#define REDUCTION_GEAR_NUM                              71  //减速机齿轮数
#define REDUCTION_RATIO                                 50  //减速比

#define REDUCTION_SPEED             ((float)((float)MOTOR_SPEED/(float)REDUCTION_RATIO))   //减速机转速
#define LENTH_REDUCTION_PER_MINUTE    ((float)(REDUCTION_SPEED*(DIAMETER_REDUCER+DIAMETER_WIRE)*3.14))   //减速机每分钟运行的长度
#define ENCODER_SPEED               (REDUCTION_SPEED/((float)ENCODER_GEAR_NUM/(float)REDUCTION_GEAR_NUM))   //编码器转速
#define LENTH_PER_PULSE             ((LENTH_REDUCTION_PER_MINUTE/ENCODER_SPEED)/((float)PULSE_PER_CYCLE/1000)) //编码器每脉冲对应钢丝绳所走的长度L,单位为mm

#define FREQ_MAX_VALUE                               10000  //发给变频器的电机最大频率
#define INIT_POSITION_WIRE                            5000  //缆绳默认初始位置
#define BAND_TYPE_BRAKE_DELAY_THRES                   300U  //300ms,电机运行300ms后抱闸松开（继电器闭合）

#define FREQ_STARTMOVE_BASE                             10  //电机运行的起始频率以及电机停止时减速最后的频率
#define FREQ_CHANGE_BASE                                 2  //每次减速的频率基准值，单位5HZ,时间单位：FREQ_REDUCE_INTERTER
#define FREQ_REDUCE_INTERTER                          300U  //频率减速时间间隔，每隔100ms减5hz
#define FORCE_REDUCE_10HZ_KEEPING                     500U  //强制减速到10HZ时需要保持的时间，单位100ms
#define READ8000_INTERTER                             500U  //查询帧周期，单位100ms

#define REMAIN_PULSE_NUMBER_FOR_BRAKE                   50  //需要松抱闸时剩余脉冲数
#define REMAIN_PULSE_NUMBER_FOR_FREQ_STOP               12  //有变频器配置时开始停机的剩余脉冲数
#define REMAIN_PULSE_NUMBER_FOR_STOP                    30  //无变频器配置时开始停机的剩余脉冲数

#define RESET_ERROR_DELAY_THRES                       600U  //急停后复位电机故障状态的延迟时间
#define READ_FREQ_POWER_THRES                         200U  //读取当前电机运行功率

enum Invertor_Offset{//变频器状态参数地址
    off_InvertorError=0,off_CurrFreqPower,
    NUM_Read_Total
};
enum Write_Data_Off{
    Control_CMD,Convert_Freq,
    NUM_Write_Total
};

enum MotorRunStatus_Info{//电机运行状态
    Motor_Idle=0,DIR_FALL,DIR_RISE
};//正向-向上运动，即0mm向30000mm方向的运动，反向-向下运动

enum MotorRunCMD_Info{//电机运行状态
    Motor_Normal=0,Motor_Init,Measure_Distance
};

enum MotorCorrentSTS_Info{//电机位置修正请求
    Corrent_None=0,Corrent_Up,Corrent_Down
};


enum Motor_Command{
    Motor_Backward_Run=1,Motor_Fardward_Run,Motor_Fardward_PointMove,Motor_Backward_PointMove,
    Motor_Stop_Reduce,Motor_Stop_Free,Error_Reset
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
extern volatile BitStatus Invertor_Status[2];
#define CMD_Rope_Wire  		                Invertor_Status[0].Bits.bit0 //0-对单个吊杆的收揽命令
#define CMD_Suspender_Min  		            Invertor_Status[0].Bits.bit1 //1-将单个吊杆运行到零位位置
#define CMD_Suspender_Emergency_Stop  		Invertor_Status[0].Bits.bit2 //2-对单个吊杆的急停命令
#define CMD_Suspender_Target  		        Invertor_Status[0].Bits.bit3 //3-将吊杆运行到目标坐标位置
#define CMD_ParaDownload_Independent  	    Invertor_Status[0].Bits.bit4 //4-微控制器个性化参数下载（数据待定）
#define CMD_Read_Common_Para       	        Invertor_Status[0].Bits.bit5 //5-读某个微控制器的共性参数（数据待定）
#define CMD_Read_Independent_Para      	    Invertor_Status[0].Bits.bit6 //6-读某个微控制器的个性化参数（数据待定）
#define CMD_Suspender_Init                  Invertor_Status[0].Bits.bit7 //7-吊杆初始化

#define CMD_ParaDownload_Common             Invertor_Status[1].Bits.bit0 //0-下载共性参数
#define CMD_Limit_Measure                   Invertor_Status[1].Bits.bit1 //0-上下限位位置测量
#define CMD_Read_Limit_Result               Invertor_Status[1].Bits.bit2 //0-读取上下限位位置数据
#define CMD_Download_LocalCfg               Invertor_Status[1].Bits.bit3 //

enum Limit_Measure_STS_T{
    M_IDLE,M_MEASURING,M_SUCCESS,M_FAIL
};

extern u16 InvertorData[NUM_Read_Total];//变频器寄存器读取结果

u8 Get_Limit_Measure_Status(void);//吊杆行程限位测量状态 0-空闲，1-测量中，2-测量成功，3-测量失败
void Task_Freq_Convert(void *p_arg);
void TaskFreq_Timer1ms(void);

#endif


