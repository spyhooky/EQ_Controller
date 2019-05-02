#include "main.h"
#include "Task_IO.h"
#include "Task_PC_Com.h"
#include "Task_MB_RTU_Master.h"
#include "Task_Freq_Convert.h"
#include "Task_MQTT.h"
#include <stdlib.h>


#ifdef __TASK_FREQ_CONVERT_H

volatile BitStatus Invertor_Status[2];
volatile BitStatus Motor_Status[2];
#define MOTOR_RUNNING                     Motor_Status[0].Bits.bit0 //电机运行标志
#define MOTOR_RUN_DELAY                   Motor_Status[0].Bits.bit1 //电机运行延时，用于抱闸
#define MOTOR_DIRECTOR                    Motor_Status[0].Bits.bit2 //吊杆运行方向，上升或者下降
#define MOTOR_REDUCING                    Motor_Status[0].Bits.bit3 //电机减速标志
#define READ_CURR_FREQ_EN                 Motor_Status[0].Bits.bit4 //是否需要发送查询马达当前频率值的标志
#define Reserve_Requrirement              Motor_Status[0].Bits.bit5 //电机需要反向运行，先减速再反向  
#define FORCE_REDUCE_EN                   Motor_Status[0].Bits.bit6 //遇到上下限位开关或者电机需要反向时置此标志
#define FORCE_REDUCE_10HZ                 Motor_Status[0].Bits.bit7 //强制减速时，频率到10HZ的标志
#define MOTOR_Init                        Motor_Status[1].Bits.bit0 //吊杆初始化进行中
#define MOTOR_CORRENT_UP                  Motor_Status[1].Bits.bit1 //电机上位置修正标志
#define MOTOR_CORRENT_DOWN                Motor_Status[1].Bits.bit2 //电机下位置修正标志

#define FREQ_REDUCE_TABLE_NUM               15U

#define SWITCH_LIMIT_DETECT                 1U //检测到限位开关，限位开关是常闭开关，所以正常时值为0
#define SWITCH_LIMIT_UNDETECT               0U //未检测到限位开关

enum Timer_Type{
    Motor_Delay,            //松开抱闸的计时，电机运行1-2s后松开，停止减速时再抱紧
    Read8000,               //查询帧定时计数器
    Read5001,               //读取当前运行频率的计数器
    Freq_Reduce,            //减速间隔时间计数器
    Keep_10HZ,              //反向或者限位减速信号后频率减到10HZ时需要继续维持的时间
    Motor_Correct,          //电机位置修正，先发停机命令，延时后修正值
    
    Timer_Total
};
static u16 cTimer[Timer_Total];

u16 InvertorData[NUM_Read_Total];
u16 Motor_Freq_MIN;

enum Init_Parameter_Off{
    P0_00_CtrlMode=0,           //控制方式选择
    P0_01_Freq_Channel,         //主频率给定通道1选择
    P0_03_RunCmd_Channel,       //运行命令通道选择
    P0_24_CMD_FREQ_CHN,         //命令通道与频率给定通道关系设定
    PC_00_Comm_Baudrate,        //通讯波特率
    PC_01_DataType,             //MODBUS数据格式
    PC_05_ModbusType,           //MODBUS通讯数据格式
                                //
    Init_Group_Total//总数量，此行保持在最后一个，需要添加项目时在上一行
};

typedef struct Init_Para_Type_Info
{
    u16 ParaAddr;   //初始化参数的寄存器地址
    u16 DataValue;  //初始化参数的值
}Init_Para_t;

typedef struct Freq_Reduce_Info
{
    u16 reduce_freq;  //开始减速的频率
    u16 pulse_remain; //剩余脉冲个数
}Freq_Reduce_t;
//本配置表主要用于识别电机对应频率的开始减速的脉冲数，不是电机执行的加速表
const Freq_Reduce_t Table_Freq_Reduce[FREQ_REDUCE_TABLE_NUM]=   
{
    {320,30000},
    {240,30000},
    {180,30000},
    {150,20000},
    {120,16000},//若工作频率为120HZ，则当剩余脉冲数小于20000时就开始减速
    {100,12000},
    {90,10000},
    {80,8000},
    {70,6500},
    {60,5000},
    {50,3500},
    {40,2000},
    {30,1200},
    {20,800},
    {10,500}
};     


u16 WriteData[NUM_Write_Total];//写缓冲区数据

struct RTU_ReqBlock Init_Point[Init_Group_Total]; //初始化序列

Init_Para_t Reg_InitGroup[Init_Group_Total]=//初始化序列配置，modbus地址+数据内容
{
    {0x0000,0x0000},    //控制方式选择 P0_00_CtrlMode
    {0x0001,0x0009},    //主频率给定通道1选择 P0_01_Freq_Channel
    {0x0003,0x0002},    //运行命令通道选择 P0_03_RunCmd_Channel
    {0x0018,9999},      //命令通道与频率给定通道关系设定 P0_24_CMD_FREQ_CHN
    {0x4000,0x0005},    //通讯波特率 PC_00_Comm_Baudrate
    {0x4001,0x0000},    //MODBUS数据格式 PC_01_DataType
    {0x4005,0x0001},    //MODBUS通讯数据格式 PC_05_ModbusType
};


RTU_ReqBlock_t RTU_Req_Read8000 = //RTU数据读请求块-变频器故障地址,变频器通讯异常地址
{
    LIST_HEAD_INIT(RTU_Req_Read8000.Entry),
    1,                                          //执行次数，0-无限次
    UART_CHN_CONVERT_FREQ,                      //执行通道
	SLAVEID_FREQ,                               //从节点站地址
	FUNC_RD_HOLDREG,                            //功能码
	EXCUTE_SUCCESS,                             //执行结果
	0x8000,                                     //操作寄存器地址
	0x01,                                       //操作寄存器数量
	(u16*)&InvertorData[off_InvertorError]      //执行的数据，读取的寄存器数据或写操作的数据
};


struct RTU_ReqBlock RTU_Req_WriteCMD_6000= //RTU数据写请求块-控制命令 1-正转 2-反转 3-正转点动 4-反转点动 5-减速停机 6-自由停机 7-故障复位
{
	LIST_HEAD_INIT(RTU_Req_WriteCMD_6000.Entry),
    1,                                          //执行次数，0-无限次
	UART_CHN_CONVERT_FREQ,                      //执行通道
	SLAVEID_FREQ,                               //从节点站地址
	FUNC_WR_SGREG,                              //功能码06
	EXCUTE_SUCCESS,                             //执行结果
	0x6000,                                     //操作寄存器地址
	0x01,                                       //操作寄存器数量
	(u16*)&WriteData[Control_CMD]               //执行的数据，读取的寄存器数据或写操作的数据
};

struct RTU_ReqBlock RTU_Req_WriteFreq_5000= //RTU数据请求块,设置运行频率
{
	LIST_HEAD_INIT(RTU_Req_WriteFreq_5000.Entry),
    1,                                          //执行次数，0-无限次
	UART_CHN_CONVERT_FREQ,                      //执行通道
	SLAVEID_FREQ,                               //从节点站地址
	FUNC_WR_SGREG,                              //功能码06
	EXCUTE_SUCCESS,                             //执行结果
	0x5000,                                     //操作寄存器地址
	0x01,                                       //操作寄存器数量
	(u16*)&WriteData[Convert_Freq]              //执行的数据，读取的寄存器数据或写操作的数据
};

struct RTU_ReqBlock RTU_Req_ReadFreq_5001= //RTU数据请求块,设置运行频率
{
	LIST_HEAD_INIT(RTU_Req_ReadFreq_5001.Entry),
    1,                                          //执行次数，0-无限次
	UART_CHN_CONVERT_FREQ,                      //执行通道
	SLAVEID_FREQ,                               //从节点站地址
	FUNC_RD_HOLDREG,                            //功能码03
	EXCUTE_SUCCESS,                             //执行结果
	0x5000,                                     //操作寄存器地址
	0x01,                                       //操作寄存器数量
	(u16*)&InvertorData[off_CurrFreq]           //执行的数据，读取的寄存器数据或写操作的数据
};


/********************************************************************************/
/*函数名：  Freq_Convert_Init                                                   */
/*功能说明：模块初始化函数                                                       */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
static void Freq_Convert_Init(void)
{
    u8 i;
    Invertor_Status[0].Byte = 0;
    Invertor_Status[1].Byte = 0;
    memset((u8 *)&cTimer[0],0,sizeof(cTimer));
    Global_Variable.Suspende_PositionTarget = Global_Variable.Para_Independence.Suspende_Limit_Up;
    Global_Variable.Suspende_PositionCurrent = Global_Variable.Para_Independence.Suspende_Limit_Up;
    memset(InvertorData,0,sizeof(InvertorData));
    memset(WriteData,0,sizeof(WriteData));
    Motor_Freq_MIN = ((u32)10*10000/Global_Variable.Para_Independence.Max_Motro_Freq);
    
    for(i=0;i<Init_Group_Total;i++)
    {
        Init_Point[i].Entry.next = &Init_Point[i].Entry;
        Init_Point[i].Entry.prev = &Init_Point[i].Entry;
        Init_Point[i].Excute_Num = 1u;                      //执行次数，0-无限次
        Init_Point[i].chnindex = UART_CHN_CONVERT_FREQ;     //执行通道
        Init_Point[i].sta_addr = SLAVEID_FREQ;              //从节点站地址
        Init_Point[i].FuncCode = FUNC_WR_SGREG;             //功能码06
        Init_Point[i].Status = EXCUTE_SUCCESS;              //执行结果
        Init_Point[i].RegAddr = Reg_InitGroup[i].ParaAddr;  //操作寄存器地址
        Init_Point[i].RegNum = 1u;                          //操作寄存器数量
        Init_Point[i].mappedBuff = (u16*)&Reg_InitGroup[i].DataValue;   //执行的数据，读取的寄存器数据或写操作的数据
        //RTU_AddReqBlock(&rtu_ctx,&Init_Point[i]);
    }
    RTU_AddReqBlock(&rtu_ctx,&RTU_Req_Read8000);//添加读故障信息请求，后台会始终运行读命令
}

/********************************************************************************/
/*函数名：  TaskFreq_Timer100ms                                                   */
/*功能说明：1ms定时函数                                                             */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
void TaskFreq_Timer100ms(void)
{
    cTimer[Read8000]++;
    if(cTimer[Read8000] >= READ8000_INTERTER)
    {//周期读取状态寄存器值
        cTimer[Read8000] = 0;
        if(MOTOR_REDUCING == OFF)
        {
            RTU_AddReqBlock(&rtu_ctx,&RTU_Req_Read8000);
        }
    }
    if(MOTOR_RUNNING == ON)
    {
        if(cTimer[Motor_Delay] >= BAND_TYPE_BRAKE_DELAY_THRES)
        {//电机开始运行时300ms内完成抱闸再断开
            //if(MOTOR_REDUCING == OFF)
            {
                BAND_TYPE_BRAKE_OUT = ON;//抱闸断开
            }
        }
        else
        {
            cTimer[Motor_Delay]++;
        }
    }
    else
    {
        cTimer[Motor_Delay] = 0;
        BAND_TYPE_BRAKE_OUT = OFF;//抱闸开启
    }

    if(MOTOR_REDUCING == ON)
    {//电机当前处于减速状态
        cTimer[Freq_Reduce]++;//减速持续时间计时
        cTimer[Read5001] = 0;
    }
    else
    {
        cTimer[Freq_Reduce] = 0;
        cTimer[Read5001]++;//读取当前电机运行频率的时间计时
    }

    if(FORCE_REDUCE_10HZ == ON)
    {//频率减到10HZ的标志，一般指的是强制减速，如遇到减速开关或者需要反向时，正常到目标位置而进行的自主减速不会置此标志
        cTimer[Keep_10HZ]++;//该计时器用于频率减到10HZ时保持一定时间后让电机停止或反向
    }
    else
    {
        cTimer[Keep_10HZ] = 0;
    }
		
    if((MOTOR_CORRENT_UP == ON)||(MOTOR_CORRENT_DOWN == ON))
    {//遇到上下限位开关
        cTimer[Motor_Correct]++;//该计时器用于遇到限位开关时延迟一定时间后修正脉冲数和吊杆位置值
    }
    else
    {
        cTimer[Motor_Correct] = 0;
    }
		
}

/****************************************************************************************/
/*函数名：  Calculate_Frequence                                                          */
/*功能说明：计算电机的运行频率                                                            */
/*输入参数：无                                                                           */       
/*输出参数：1：输出值是HZ对应的数值，最大为10000，对应最大频率                                                */
/****************************************************************************************/
static u16 Calculate_Frequence(void)
{
    float temp;
    u16 ret_freq;
    temp = Global_Variable.Suspende_SpeedTarget*Global_Variable.Para_Independence.Motor_Freq_Factor;
    temp = temp>Global_Variable.Para_Independence.Max_Motro_Freq?Global_Variable.Para_Independence.Max_Motro_Freq:temp;
    ret_freq = (u16)(temp*10000/Global_Variable.Para_Independence.Max_Motro_Freq);
    ret_freq = ret_freq<Motor_Freq_MIN?Motor_Freq_MIN:ret_freq;
    return ret_freq;
}

/****************************************************************************************/
/*函数名：  Set_Frequence_Start                                                          */
/*功能说明：计算电机启动的运行频率                                                            */
/*输入参数：无                                                                           */       
/*输出参数：
1：f(频率)=（50*X（设定速度））/[995*(D1(减速机直径)+D2（钢丝绳直径）*3.14/减速比)]
*/
/****************************************************************************************/
static void Set_Frequence_Start(void)
{
    u16 motor_freq;
    
    MOTOR_REDUCING = OFF;
    if(MOTOR_DIRECTOR == D_FALL)
    {
        if(Global_Variable.Encode_PulseTarget - Global_Variable.Encode_PulseCurrent >= 2000)
        {
            motor_freq = Calculate_Frequence();
        }
        else
        {
            motor_freq = Motor_Freq_MIN;
            MOTOR_REDUCING = ON;
        }
    }
    else//(MOTOR_DIRECTOR == D_Forward)
    {
        if(MOTOR_Init == OFF)
        {
            if(Global_Variable.Encode_PulseCurrent - Global_Variable.Encode_PulseTarget >= 2000)
            {
                motor_freq = Calculate_Frequence();
            }
            else
            {
                motor_freq = Motor_Freq_MIN;
                MOTOR_REDUCING = ON;
            }
        }
        else
        {
            motor_freq = Calculate_Frequence();
        }
    }
    Global_Variable.Suspende_SpeedCurrent = ((u32)motor_freq*(u32)Global_Variable.Para_Independence.Max_Motro_Freq)/10000;
    Global_Variable.Suspende_SpeedCurrent /= Global_Variable.Para_Independence.Motor_Freq_Factor;
    //if(Wrdata[Convert_Freq] != Freq_Req)
    {
        WriteData[Convert_Freq] = motor_freq;
        RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteFreq_5000);
    }
}

/****************************************************************************************/
/*函数名：  Frequence_Reduce_Logic                                                          */
/*功能说明：判断是否需要减速并设置减速频率                                                            */
/*输入参数：无                                                                           */       
/*输出参数：无
*/
/****************************************************************************************/
static u16 Frequence_Reduce_Logic(u32 Delta_Pulse)
{
    u8 i;
    float curfreq;
    u16 motor_freq;
    
    if(MOTOR_REDUCING == OFF)
    {
        if(cTimer[Read5001] >= 1)
        {//当电机处于非减速状态时周期性读取当前电机频率
            cTimer[Read5001] = 0;
            RTU_AddReqBlock(&rtu_ctx,&RTU_Req_ReadFreq_5001);
            Global_Variable.Suspende_SpeedCurrent = ((InvertorData[off_CurrFreq]*Global_Variable.Para_Independence.Max_Motro_Freq)/10000)/
                Global_Variable.Para_Independence.Motor_Freq_Factor;
            //Global_Variable.Suspende_Current_Speed = (InvertorData[off_CurrFreq]/100)/Global_Variable.Para_Independence.Motor_Freq_Factor;//*10000/100
        }
        curfreq = (Global_Variable.Para_Independence.Max_Motro_Freq * InvertorData[off_CurrFreq]) / 10000;
        
        for(i=0;i<FREQ_REDUCE_TABLE_NUM;i++)
        {
            if(curfreq >= Table_Freq_Reduce[i].reduce_freq)
            {//遍历减速起始的剩余脉冲数
                if((FORCE_REDUCE_EN == ON)||(Table_Freq_Reduce[i].pulse_remain >= Delta_Pulse))
                {//满足减速条件时开始减速，并置位
                    Global_Variable.Suspende_SpeedCurrent = Table_Freq_Reduce[i].reduce_freq/Global_Variable.Para_Independence.Motor_Freq_Factor;
                    motor_freq = Table_Freq_Reduce[i].reduce_freq*10000/Global_Variable.Para_Independence.Max_Motro_Freq;//
                    MOTOR_REDUCING = ON;
                    break;
                }
            }
        }
        if(MOTOR_REDUCING == OFF)
        {
            motor_freq = WriteData[Convert_Freq];
        }
    }
    else
    {
        if(cTimer[Freq_Reduce] >= FREQ_REDUCE_INTERTER)
        {//每隔一定时间减速固定频率
            if(WriteData[Convert_Freq] > Motor_Freq_MIN)//频率还未减到最低的10HZ
            {
                cTimer[Freq_Reduce] = 0;
                Global_Variable.Suspende_SpeedCurrent -= (u16)(FREQ_REDUCE_BASE/Global_Variable.Para_Independence.Motor_Freq_Factor);
                curfreq = Global_Variable.Suspende_SpeedCurrent * Global_Variable.Para_Independence.Motor_Freq_Factor;//HZ
                motor_freq = curfreq*10000/Global_Variable.Para_Independence.Max_Motro_Freq;//
            }
        }
        else
        {
            motor_freq = WriteData[Convert_Freq];
        }
    }
    return motor_freq;//该频率的单位是发给变频器的值，0-10000对应0-最大频率
}


/****************************************************************************************/
/*函数名：  Set_Frequence_Running                                                          */
/*功能说明：计算电机运行中的频率                                                            */
/*输入参数：无                                                                           */       
/*输出参数：
1：f(频率)=（50*X（设定速度））/[995*(D1(减速机直径)+D2（钢丝绳直径）*3.14/减速比)]
*/
/****************************************************************************************/
static void Set_Frequence_Running(u32 Delta_Pulse)
{
    u16 motor_freq;
    motor_freq = Frequence_Reduce_Logic(Delta_Pulse);
    if(motor_freq <= Motor_Freq_MIN)
    {
        MOTOR_REDUCING = OFF;
        motor_freq = Motor_Freq_MIN;
    }

    if(WriteData[Convert_Freq] > motor_freq)
    {
        WriteData[Convert_Freq] = motor_freq;
        RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteFreq_5000);
    }
}


/********************************************************************************/
/*函数名：  MotorMove_Fall                                                        */
/*功能说明：电机向下运动命令                                                          */
/*输入参数：无                                                                   */
/*输出参数：无                                                                  */
/*******************************************************************************/
void MotorMove_Fall(void)
{
    MOTOR_DIRECTOR = D_FALL;
    if(Global_Variable.Para_Independence.Convert_Cfg == ON)
    {//有变频器配置
        Set_Frequence_Start();
        //if(Wrdata[Control_CMD] != Motor_Fardward_Run)
        {
            WriteData[Control_CMD] = Motor_Fardward_Run;
            RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteCMD_6000);
        }
    }
    else
    {//无变频器配置
        CONTACTOR_RISE_OUT = OFF;
        CONTACTOR_FALL_OUT = ON;
        CONTACTOR_STOP_OUT = OFF;
    }
}

/********************************************************************************/
/*函数名：  MotorMove_Rise                                                      */
/*功能说明：电机向上运动命令                                                         */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
void MotorMove_Rise(void)
{
    MOTOR_DIRECTOR = D_RISE;
    if(Global_Variable.Para_Independence.Convert_Cfg == ON)
    {//有变频器配置
        Set_Frequence_Start();
        //if(Wrdata[Control_CMD] != Motor_Backward_Run)
        {
            WriteData[Control_CMD] = Motor_Backward_Run;
            RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteCMD_6000);
        }
    }
    else
    {//无变频器配置
        CONTACTOR_RISE_OUT = ON;
        CONTACTOR_FALL_OUT = OFF;
        CONTACTOR_STOP_OUT = OFF;
    }
}

/********************************************************************************/
/*函数名：  Motor_Stop                                                       */
/*功能说明：电机停机                                                         */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
static void Motor_Stop(u8 stoptype)
{
    u8 stopmode;
    if(Global_Variable.Para_Independence.Convert_Cfg == ON)
    {//有变频器配置
        MOTOR_RUNNING = OFF;
        FORCE_REDUCE_10HZ = OFF;
        cTimer[Keep_10HZ] = 0;
        FORCE_REDUCE_EN = OFF;
        MOTOR_REDUCING = OFF;
        if((stoptype ==Motor_Stop_Reduce)||(stoptype ==Motor_Stop_Free))
        {   
            stopmode = stoptype;
        }
        else
        {
            stopmode = Motor_Stop_Free;
        }
        //if(WriteData[Control_CMD] != stopmode)
        {
            WriteData[Control_CMD] = stopmode;
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteCMD_6000);
        }
    }
    else
    {//无变频器配置
        CONTACTOR_RISE_OUT = OFF;
        CONTACTOR_FALL_OUT = OFF;
        CONTACTOR_STOP_OUT = ON;
    }
}


/********************************************************************************/
/*函数名：  Task_Freq_Convert                                                   */
/*功能说明：变频器主task                                                         */
/*输入参数：p_arg                                                               */
/*输出参数：无                                                                  */
/*******************************************************************************/
void Task_Freq_Convert(void *p_arg)
{
//     struct wiz_NetInfo_t *ethparm;
//     ethparm = (struct wiz_NetInfo_t *)p_arg;
    Freq_Convert_Init();
    s32 Delta_Pulse;
    u8 pre_limit_rise;
    u8 pre_limit_fall;
    while (1)
    {        
        if(CMD_Suspender_Init == ON)
        {//吊杆初始化请求
            CMD_Suspender_Init = OFF;
            Global_Variable.Encode_PulseTarget = (Global_Variable.Suspende_PositionTarget-Global_Variable.Para_Independence.Suspende_Limit_Up) / Global_Variable.Para_Independence.Lenth_Per_Pulse;
            MOTOR_RUNNING = ON;
            MOTOR_Init = ON;
            MotorMove_Rise();
        }
    
        if((FORCE_REDUCE_EN == ON)&&(MOTOR_REDUCING == OFF))//强制减速完成，当前频率是最小频率10HZ
        {//此条件必须放在该while的最上边判断，否则会导致条件无法满足
            if(MOTOR_Init == OFF)
            {//必须是非吊杆初始化状态下
                FORCE_REDUCE_EN = OFF;
                FORCE_REDUCE_10HZ = ON;
                cTimer[Keep_10HZ] = 0;
            }
        }

        if(Global_Variable.Para_Independence.Convert_Cfg == ON)
        {//有变频器配置
            if(cTimer[Keep_10HZ] >= FORCE_REDUCE_10HZ_KEEPING)
            {//强制减速到10HZ且保持固定时间后需要停止电机运行或者开始反向运行
                FORCE_REDUCE_10HZ = OFF;
                if(Reserve_Requrirement == ON)
                {//需要反向运行
                    Reserve_Requrirement = OFF;
                    cTimer[Motor_Delay] = 0;//重新计时500ms再松抱闸
                    Global_Variable.Encode_PulseTarget = (s32)((Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget)/ \
                        Global_Variable.Para_Independence.Lenth_Per_Pulse);
                    if(Global_Variable.Encode_PulseTarget > Global_Variable.Encode_PulseCurrent)
                    {
                        MotorMove_Fall();
                    }
                    else
                    {
                        MotorMove_Rise();
                    }
                }
                else
                {
                    Motor_Stop(Motor_Stop_Reduce);
                }
            }
        }
        else
        {//无变频器配置
            if(FORCE_REDUCE_10HZ == ON)
            {
                Motor_Stop(Motor_Stop_Reduce);
            }
        }


        if(MOTOR_RUNNING == ON)
        {
            if(((Limit_Up_SlowDown == ON)&&(MOTOR_DIRECTOR == D_RISE))||((Limit_Down_SlowDown == ON)&&(MOTOR_DIRECTOR == D_FALL))||
                (Reserve_Requrirement == ON))//运行过程中遇到限位减速信号
            {
                if(MOTOR_Init == OFF)//吊杆初始化时需要运行至限位开关处
                {
                    FORCE_REDUCE_EN = ON;
                }
            }
            else
            {
                FORCE_REDUCE_EN = OFF;
            }

            if(Limit_Rise_Signal == ON)
            {//运行过程中遇到上限位信号
                MOTOR_Init = OFF;
                if(MOTOR_DIRECTOR == D_RISE)
                {//电机上升
                    if(BAND_TYPE_BRAKE_OUT == ON)
                    {
                        BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                    }
                    Motor_Stop(Motor_Stop_Reduce);
                    
                    if(pre_limit_rise == OFF)
                    {//需要进行位置补偿
                        MOTOR_CORRENT_UP = ON; 
                    }
                }
                else
                {//电机下降

                }
            }
            else
            {
#if 0
                if(pre_limit_rise == ON)
                {//需要进行位置补偿
                    OS_ENTER_CRITICAL(); 
                    Global_Variable.Suspende_PositionCurrent = Global_Variable.Para_Independence.Suspende_Limit_Up;
                    Global_Variable.Compensate_Pulse = 0 - Global_Variable.Encode_PulseCurrent;//过冲了需要补偿正值
                    Global_Variable.Compensate_En = ON;
                    OS_EXIT_CRITICAL();
                }
#endif
            }

            if(Limit_Fall_Signal == ON)
            {//运行过程中遇到下限位信号
                if(MOTOR_DIRECTOR == D_FALL)
                {
                    if(BAND_TYPE_BRAKE_OUT == ON)
                    {
                        BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                    }
                    Motor_Stop(Motor_Stop_Reduce);

                    if(pre_limit_fall == OFF)
                    {//需要进行位置补偿
                        MOTOR_CORRENT_DOWN = ON; 
                    }
                }
                else
                {

                }
            }
            else
            {
#if 0
                if(pre_limit_fall == ON)
                {//需要进行位置补偿
                    OS_ENTER_CRITICAL(); 
                    Global_Variable.Suspende_PositionCurrent = 0;
                    Global_Variable.Compensate_Pulse = (Global_Variable.Para_Independence.Suspende_Limit_Up/Global_Variable.Para_Independence.Lenth_Per_Pulse)
                        - Global_Variable.Encode_PulseCurrent;//过冲了就补偿负值
                    Global_Variable.Compensate_En = ON;
                    OS_EXIT_CRITICAL();
                }
#endif
            }
        }
        pre_limit_rise = Limit_Rise_Signal;
        pre_limit_fall = Limit_Fall_Signal;

        if(cTimer[Motor_Correct] >= 6U)//遇到限位开关后等待600ms，再修正脉冲数和当前位置值
        {
            cTimer[Motor_Correct] = 0;
            if(MOTOR_CORRENT_UP == ON)
            {
                MOTOR_CORRENT_UP = OFF;
                OS_ENTER_CRITICAL(); 
                Global_Variable.Suspende_PositionCurrent = Global_Variable.Para_Independence.Suspende_Limit_Up;
                Global_Variable.Compensate_Pulse = 0 - Global_Variable.Encode_PulseCurrent;//过冲了需要补偿正值
                Global_Variable.Compensate_En = ON;
                OS_EXIT_CRITICAL();
            }

            if(MOTOR_CORRENT_DOWN == ON)
            {
                MOTOR_CORRENT_DOWN = OFF;
                OS_ENTER_CRITICAL(); 
                Global_Variable.Suspende_PositionCurrent = 0;
                Global_Variable.Compensate_Pulse = (Global_Variable.Para_Independence.Suspende_Limit_Up/Global_Variable.Para_Independence.Lenth_Per_Pulse)
                    - Global_Variable.Encode_PulseCurrent;//过冲了就补偿负值
                Global_Variable.Compensate_En = ON;
                OS_EXIT_CRITICAL();
            }
        }

        
        if(MOTOR_DIRECTOR == D_FALL)
        {
            Delta_Pulse = Global_Variable.Encode_PulseTarget - Global_Variable.Encode_PulseCurrent;
        }
        else
        {
            Delta_Pulse = Global_Variable.Encode_PulseCurrent - Global_Variable.Encode_PulseTarget;
        }

        if(Global_Variable.Para_Independence.Convert_Cfg == ON)
        { //有变频器配置
#if 0
    	    if((Delta_Pulse < 1000u)&&(MOTOR_REDUCING == ON))
            {//脉冲数小于1000时立即断开抱闸
                if(BAND_TYPE_BRAKE_OUT == ON)
                {
                    BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                }
            }
#endif

            if((Delta_Pulse < REMAIN_PULSE_NUMBER_FOR_FREQ_STOP)||(Delta_Pulse < 0))
            {
                if(BAND_TYPE_BRAKE_OUT == ON)
                {//电机到达目标位置时，就立即抱闸，不要提前抱闸
                    cTimer[Motor_Delay] = 0;
                    BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                }
                if(MOTOR_RUNNING == ON)/*&&(MOTOR_REDUCING == OFF)*/
                {
                    Motor_Stop(Motor_Stop_Reduce);//到达目标位置，执行减速停机
                }
            }
            else
            {
                Set_Frequence_Running(Delta_Pulse);
            }

        }
        else
        { //无变频器配置
            if(Delta_Pulse < REMAIN_PULSE_NUMBER_FOR_STOP)
            {
                Motor_Stop(Motor_Stop_Reduce);
            }
        }
        
        if(CMD_Rope_Wire == ON)//单个吊杆收揽（复位吊杆位置，就是吊杆的最高位置）
        {
            MOTOR_RUNNING = ON;
            CMD_Rope_Wire = OFF;
            Global_Variable.Encode_PulseTarget = 0;
            MotorMove_Rise();
        }
        
        if(CMD_Suspender_Min == ON)//单个吊杆降到零点坐标位置（吊杆的最低位置）
        {
            MOTOR_RUNNING = ON;
            CMD_Suspender_Min = OFF;
            Global_Variable.Encode_PulseTarget = Global_Variable.Para_Independence.Suspende_Limit_Up/Global_Variable.Para_Independence.Lenth_Per_Pulse;
            MotorMove_Fall();
        }
        
        if(CMD_Suspender_Emergency_Stop == ON)//上位机命令：单个吊杆急停
        {
            if(BAND_TYPE_BRAKE_OUT == ON)
            {
                BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
            }
            CMD_Suspender_Emergency_Stop = OFF;
            Motor_Stop(Motor_Stop_Free);            
        }
        
        if(CMD_Suspender_Target == ON)//单个吊杆运行到设定的目标位置
        {
            MOTOR_RUNNING = ON;
            CMD_Suspender_Target = OFF;
            Global_Variable.Encode_PulseTarget = (s32)((Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget)/ \
                Global_Variable.Para_Independence.Lenth_Per_Pulse);
            if(Global_Variable.Encode_PulseTarget > Global_Variable.Encode_PulseCurrent)
            {
                MotorMove_Fall();
            }
            else
            {
                MotorMove_Rise();
            }
        }

        if(Err_Stop_Signal == ON)//急停按钮：吊杆急停
        {
            if(MOTOR_RUNNING == ON)
            {
                if(BAND_TYPE_BRAKE_OUT == ON)
                {
                    BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                }
                Motor_Stop(Motor_Stop_Free);   
            }
        }

        if(Err_Summit_Attempt == ON)//冲顶故障：吊杆急停
        {
            if(MOTOR_RUNNING == ON)
            {
                if(BAND_TYPE_BRAKE_OUT == ON)
                {
                    BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                }
                Motor_Stop(Motor_Stop_Free);   
            }
        }

        OSTimeDlyHMSM(0, 0, 0, 1);
    }
    
}


#endif

