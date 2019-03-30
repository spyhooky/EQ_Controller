#ifndef __TASK_FREQ_CONVERT_H
#define __TASK_FREQ_CONVERT_H


enum Invertor_Offset{//变频器状态参数地址
    off_FreqByComm=0,   //通讯给定频率-10000~1000 `（十进制）
    off_RunningFreq,
    off_BusVoltage,
    off_OutVoltage,
    off_OutCurrent,
    off_OutPower,

    off_InvertorError=0x21,
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


extern u8 InvertorData[80];

void Task_Freq_Convert(void *p_arg);


#endif


