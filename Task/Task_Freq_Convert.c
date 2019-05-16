#include "main.h"
#include "Task_IO.h"
#include "Task_PC_Com.h"
#include "Task_MB_RTU_Master.h"
#include "Task_Freq_Convert.h"
#include "Task_MQTT.h"
#include <stdlib.h>


#ifdef __TASK_FREQ_CONVERT_H

typedef union {
    u16 dword;
    struct {
        u8  motor_runsts            :2;
        u8  accelerate              :1;
        u8  decelerate              :1;
    	u8  reserve_req             :1;
        u8  force_reduce_en         :1;
    	u8  force_reduce_10hz       :1;
        u8  motor_stop_delay        :1;
        u8  motor_runcmd            :2;
    	u8  limit_measure_sts       :2;
        u8  correct_sts             :2;
        u8  reserve1                :1;
        u8  reserve2                :1;
    }Bits;
}MotorStatus_t;
volatile MotorStatus_t MotorStatus;


volatile BitStatus Invertor_Status[2];
volatile BitStatus Motor_Status[2];
#define MOTOR_RUNNING_STS                 MotorStatus.Bits.motor_runsts         //电机运行状态，停止，正向或反向
#define MOTOR_RUNNING_CMD                 MotorStatus.Bits.motor_runcmd         //电机运行命令，正常运行，初始化，测距  
#define MOTOR_ACCELERATE                  MotorStatus.Bits.accelerate           //电机加速标志
#define MOTOR_REDUCING                    MotorStatus.Bits.decelerate           //电机减速标志
#define Reserve_Requrirement              MotorStatus.Bits.reserve_req          //电机需要反向运行，先减速再反向  
#define FORCE_REDUCE_EN                   MotorStatus.Bits.force_reduce_en      //遇到上下限位开关或者电机需要反向时置此标志
#define FORCE_REDUCE_10HZ                 MotorStatus.Bits.force_reduce_10hz    //强制减速时，频率到10HZ的标志
#define MOTOR_CORRENT_STS                 MotorStatus.Bits.correct_sts          //电机位置修正状态，0-无 1-上修正 2-下修正
#define MOTOR_STOP_DELAY                  MotorStatus.Bits.motor_stop_delay     //电机急停后延迟
#define MEASURE_LIMIT_STS                 MotorStatus.Bits.limit_measure_sts    //吊杆行程限位测量状态 0-空闲，1-测量中，2-测量成功，3-测量失败

#define FREQ_REDUCE_TABLE_NUM             100U

enum Timer_Type{
    Motor_Delay,            //松开抱闸的计时，电机运行1-2s后松开，停止减速时再抱紧
    Read8000,               //查询帧定时计数器
    Read5001,               //读取当前运行频率的计数器
    Freq_Acc_Reduce,        //加减速间隔时间计数器
    Keep_10HZ,              //反向或者限位减速信号后频率减到10HZ时需要继续维持的时间
    Motor_Correct,          //电机位置修正，先发停机命令，延时后修正值
    Reset_Error_Delay,      //电机急停后复位故障的延时时间，初步定为500ms
    Read5005,               //读取电机输出功率
    
    Timer_Total
};
static u16 cTimer[Timer_Total];

u16 InvertorData[NUM_Read_Total];
u16 Motor_Freq_MIN; //输出值是HZ对应的数值，最大为10000，对应最大频率
s32 Limit_Position;    //限位信号处位置值mm

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

typedef struct Freq_Change_Info
{
    u16 running_freq;  //运行的频率
    u16 pulse_num;  //剩余脉冲数
    u16 Keep_Timer;  //保持时间
}Freq_Change_t;
//本配置表主要用于识别电机对应频率的开始减速的脉冲数，不是电机执行的加速表
Freq_Change_t Table_Freq_Change[FREQ_REDUCE_TABLE_NUM]=   
{//  FREQ   LENTH
    {10,   8000,    300},
    {12,   8000,    300},
    {14,   8000,    300},
};     

typedef struct Acc_Reduce_Info
{
    s32 Pulse_StartMove;    //开始运行的脉冲数
    u8  Freq_Index;
}Acc_Reduce_t;
Acc_Reduce_t Acc_Reduce_Status;



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

struct RTU_ReqBlock RTU_Req_ReadPower_5005= //RTU数据请求块,读取当前电机运行功率
{
	LIST_HEAD_INIT(RTU_Req_ReadPower_5005.Entry),
    1,                                          //执行次数，0-无限次
	UART_CHN_CONVERT_FREQ,                      //执行通道
	SLAVEID_FREQ,                               //从节点站地址
	FUNC_RD_HOLDREG,                            //功能码03
	EXCUTE_SUCCESS,                             //执行结果
	0x5005,                                     //操作寄存器地址
	0x01,                                       //操作寄存器数量
	(u16*)&InvertorData[off_CurrFreqPower]      //执行的数据，读取的寄存器数据或写操作的数据
};

static void Motor_Stop(u8 stoptype);


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
    MEASURE_LIMIT_STS = M_IDLE;
    memset((u8 *)&cTimer[0],0,sizeof(cTimer));
    Global_Variable.Suspende_PositionTarget = Global_Variable.Para_Independence.Suspende_Limit_Up;
    if(Global_Variable.Suspende_PositionMemory != 0xffff)
    {
        Global_Variable.Suspende_PositionCurrent = Global_Variable.Suspende_PositionMemory;
        Global_Variable.Suspende_PulseMemory = (Global_Variable.Para_Independence.Suspende_Limit_Up - 
            Global_Variable.Suspende_PositionMemory) / Global_Variable.Para_Independence.Lenth_Per_Pulse;
        Global_Variable.Encode_PulseCurrent = Global_Variable.Suspende_PulseMemory;
    }
    else
    {
        Global_Variable.Suspende_PositionMemory = 0;
        Global_Variable.Suspende_PositionCurrent = Global_Variable.Para_Independence.Suspende_Limit_Up;
    }
    memset(InvertorData,0,sizeof(InvertorData));
    memset(WriteData,0,sizeof(WriteData));
    memset(&Acc_Reduce_Status,0,sizeof(Acc_Reduce_Status));
    Motor_Freq_MIN = ((u32)10*10000/Global_Variable.Para_Independence.Max_Motro_Freq);

    for(i=0;i<FREQ_REDUCE_TABLE_NUM;i++)
    {
        Table_Freq_Change[i].running_freq = FREQ_STARTMOVE_BASE + i*Global_Variable.Para_Independence.Step_Size_Base;
        Table_Freq_Change[i].pulse_num = Global_Variable.Para_Independence.Distance_10HZ + i*Global_Variable.Para_Independence.Pulze_NumBase;
        Table_Freq_Change[i].Keep_Timer = Global_Variable.Para_Independence.Freq_Change_Timer;
    }
    
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
/*函数名：  Get_Limit_Measure_Status                                                   */
/*功能说明： 获取当前测高的状态                                                             */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
u8 Get_Limit_Measure_Status(void)
{
    return MEASURE_LIMIT_STS;
}


/********************************************************************************/
/*函数名：  TaskFreq_Timer100ms                                                   */
/*功能说明：1ms定时函数                                                             */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
void TaskFreq_Timer1ms(void)
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
    
    if(MOTOR_RUNNING_STS != OFF) //电机处于运行状态
    {
        cTimer[Read5005] = 0;
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
        if(cTimer[Read5005] >= READ_FREQ_POWER_THRES)
        {
            cTimer[Read5005] = 0;
            if(InvertorData[off_CurrFreqPower] != 0)
            {
                Motor_Stop(Motor_Stop_Free); 
            }
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_ReadPower_5005);
        }
        else
        {
            cTimer[Read5005]++;
        }
    }

    if((MOTOR_ACCELERATE == ON)||(MOTOR_REDUCING == ON))
    {//电机当前处于加减速状态
        cTimer[Freq_Acc_Reduce]++;//减速持续时间计时
        cTimer[Read5001] = 0;
    }
    else
    {
        cTimer[Freq_Acc_Reduce] = 0;
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
		
    if(MOTOR_CORRENT_STS != Corrent_None)
    {//遇到上下限位开关
        cTimer[Motor_Correct]++;//该计时器用于遇到限位开关时延迟一定时间后修正脉冲数和吊杆位置值
    }
    else
    {
        cTimer[Motor_Correct] = 0;
    }

    if(MOTOR_STOP_DELAY == ON)
    {
        cTimer[Reset_Error_Delay]++;
        if(cTimer[Reset_Error_Delay] > RESET_ERROR_DELAY_THRES)
        {
            MOTOR_STOP_DELAY = OFF;
            cTimer[Reset_Error_Delay] = 0;
            WriteData[Control_CMD] = Error_Reset;
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteCMD_6000);
        }
    }
    else
    {
        cTimer[Reset_Error_Delay] = 0;
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
    MOTOR_ACCELERATE = ON;
    Acc_Reduce_Status.Freq_Index = 0;
    Acc_Reduce_Status.Pulse_StartMove = Global_Variable.Encode_PulseCurrent;
    motor_freq = Motor_Freq_MIN;
    Global_Variable.Suspende_SpeedCurrent = ((u32)motor_freq*(u32)Global_Variable.Para_Independence.Max_Motro_Freq)/FREQ_MAX_VALUE;
    Global_Variable.Suspende_SpeedCurrent /= Global_Variable.Para_Independence.Motor_Freq_Factor;
    //if(Wrdata[Convert_Freq] != Freq_Req)
    {
        WriteData[Convert_Freq] = motor_freq;
        RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteFreq_5000);
    }
}

/****************************************************************************************/
/*函数名：  Frequence_Acc_Reduce_Logic                                                      */
/*功能说明：判断是否需要加减速并设置加减速频率                                                            */
/*输入参数：无                                                                           */       
/*输出参数：无
*/
/****************************************************************************************/
static u16 Frequence_Acc_Reduce_Logic(u32 Delta_Pulse)
{
    u8 i;
    float curfreq;//HZ
    u32 reduce_pulse=0;
    u16 motor_freq=0;//输出值是HZ对应的数值，最大为10000，对应最大频率
    
    if(MOTOR_REDUCING == OFF)
    {
        for(i=0;i<=Acc_Reduce_Status.Freq_Index;i++)
        {
            reduce_pulse += Table_Freq_Change[i].pulse_num;
        }
        if((FORCE_REDUCE_EN == ON)||((reduce_pulse + (Table_Freq_Change[Acc_Reduce_Status.Freq_Index+1].pulse_num)) >= Delta_Pulse))
        {//满足减速条件时开始减速，并置位
            Global_Variable.Suspende_SpeedCurrent = Table_Freq_Change[Acc_Reduce_Status.Freq_Index].running_freq/Global_Variable.Para_Independence.Motor_Freq_Factor;
            motor_freq = Table_Freq_Change[Acc_Reduce_Status.Freq_Index].running_freq*FREQ_MAX_VALUE/Global_Variable.Para_Independence.Max_Motro_Freq;//
            MOTOR_REDUCING = ON;
            MOTOR_ACCELERATE = OFF;
            cTimer[Freq_Acc_Reduce] = 0;
        }

        if(MOTOR_REDUCING == OFF)
        {
            if(cTimer[Freq_Acc_Reduce] >= Table_Freq_Change[Acc_Reduce_Status.Freq_Index].Keep_Timer)
            {
                cTimer[Freq_Acc_Reduce] = 0;
                if((Global_Variable.Suspende_SpeedTarget > Global_Variable.Suspende_SpeedCurrent)&&
                    ((Global_Variable.Suspende_SpeedTarget - Global_Variable.Suspende_SpeedCurrent) > (Global_Variable.Para_Independence.Step_Size_Base/Global_Variable.Para_Independence.Motor_Freq_Factor)))
                {
                    Acc_Reduce_Status.Freq_Index++;
                    Global_Variable.Suspende_SpeedCurrent = (u16)(Table_Freq_Change[Acc_Reduce_Status.Freq_Index].running_freq/Global_Variable.Para_Independence.Motor_Freq_Factor); 
                    MOTOR_ACCELERATE = ON;
                }
                else
                {
                    MOTOR_ACCELERATE = OFF;
                    Global_Variable.Suspende_SpeedCurrent = Global_Variable.Suspende_SpeedTarget;
                }
                curfreq = Global_Variable.Suspende_SpeedCurrent*Global_Variable.Para_Independence.Motor_Freq_Factor;
                motor_freq = curfreq*FREQ_MAX_VALUE/Global_Variable.Para_Independence.Max_Motro_Freq;//
            }
            else
            {
                motor_freq = WriteData[Convert_Freq];//保持上次值不变
            }
        }
        else
        {
            /*do nothing*/
        }
    }
    else
    {
        MOTOR_ACCELERATE = OFF;
        if(cTimer[Freq_Acc_Reduce] >= Table_Freq_Change[Acc_Reduce_Status.Freq_Index].Keep_Timer)
        {//每隔一定时间减速固定频率
            if(Acc_Reduce_Status.Freq_Index > 0)//频率还未减到最低的10HZ
            {
                cTimer[Freq_Acc_Reduce] = 0;
                Acc_Reduce_Status.Freq_Index--;
                Global_Variable.Suspende_SpeedCurrent = (u16)(Table_Freq_Change[Acc_Reduce_Status.Freq_Index].running_freq/Global_Variable.Para_Independence.Motor_Freq_Factor); 
                curfreq = Global_Variable.Suspende_SpeedCurrent * Global_Variable.Para_Independence.Motor_Freq_Factor;//HZ
                motor_freq = curfreq*FREQ_MAX_VALUE/Global_Variable.Para_Independence.Max_Motro_Freq;//
            }
            else
            {
                motor_freq = Motor_Freq_MIN;
                MOTOR_REDUCING = OFF;
            }
        }
        else
        {
            motor_freq = WriteData[Convert_Freq];//保持上次值不变
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
    motor_freq = Frequence_Acc_Reduce_Logic(Delta_Pulse);
    if(motor_freq <= Motor_Freq_MIN)
    {
        MOTOR_REDUCING = OFF;
        motor_freq = Motor_Freq_MIN;
    }
    else if(motor_freq >= FREQ_MAX_VALUE)
    {
        MOTOR_ACCELERATE = OFF;
        motor_freq = FREQ_MAX_VALUE;
    }
    else
    {

    }

    if(WriteData[Convert_Freq] != motor_freq)
    {
        WriteData[Convert_Freq] = motor_freq;
        RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteFreq_5000);
    }
}


/********************************************************************************/
/*函数名：  MotorMove_Fall                                                        */
/*功能说明：电机向下运动命令                                                          */
/*输入参数：distance运行距离,基于上限位的相对距离                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
void MotorMove_Fall(s32 distance)
{
    MOTOR_RUNNING_STS = DIR_FALL;
    Global_Variable.Encode_PulseTarget = distance/Global_Variable.Para_Independence.Lenth_Per_Pulse;
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
/*输入参数：distance运行距离,基于上限位的相对距离                                                */
/*输出参数：无                                                                  */
/*******************************************************************************/
void MotorMove_Rise(s32 distance)
{
    MOTOR_RUNNING_STS = DIR_RISE;
    Global_Variable.Encode_PulseTarget = distance/Global_Variable.Para_Independence.Lenth_Per_Pulse;
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
        MOTOR_RUNNING_STS = Motor_Idle;
        MOTOR_RUNNING_CMD = Motor_Normal;
        FORCE_REDUCE_10HZ = OFF;
        cTimer[Keep_10HZ] = 0;
        FORCE_REDUCE_EN = OFF;
        MOTOR_ACCELERATE = OFF;
        MOTOR_REDUCING = OFF;
        if((stoptype ==Motor_Stop_Reduce)||(stoptype ==Motor_Stop_Free))
        {   
            stopmode = stoptype;
        }
        else
        {
            stopmode = Motor_Stop_Free;
        }

        if(stopmode == Motor_Stop_Free)
        {
            if((RTU_Req_Read8000.Status != EXCUTE_FAIL)&&(InvertorData[off_InvertorError] == 0))
            {//停机前电机无故障
                MOTOR_STOP_DELAY = ON;
            }
        }
        else
        {
            //CMD_Download_LocalCfg = ON;//存储当前位置
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
    InvertorData[off_CurrFreqPower] = 1;//停机时先设置当前功率为非零，后续再读取寄存器值更新该值
    RTU_AddReqBlock(&rtu_ctx,&RTU_Req_ReadPower_5005);
    //CMD_Download_LocalCfg = ON;//存储当前位置
}

/********************************************************************************/
/*函数名：  CMD_Freq_Convert                                                   */
/*功能说明：变频器命令处理函数                                                         */
/*输入参数：无                                                                   */
/*输出参数：无                                                                  */
/*******************************************************************************/
void CMD_Freq_Convert(void)
{
    if(CMD_Suspender_Emergency_Stop == ON)//上位机命令：单个吊杆急停
    {
        BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
        CMD_Suspender_Emergency_Stop = OFF;
        Motor_Stop(Motor_Stop_Free);            
    }
    else if(CMD_Suspender_Init == ON)
    {//吊杆初始化请求
        CMD_Suspender_Init = OFF;
        if((Global_Variable.Suspende_PositionCurrent + 10) < Global_Variable.Para_Independence.Suspende_Limit_Up)
        {//当前位置靠近上限位时不执行初始化动作,只有当前位置在上限位以下超过10mm时才执行初始化
            if(MOTOR_RUNNING_CMD != Motor_Init)
            {
                MOTOR_RUNNING_CMD = Motor_Init;
                MotorMove_Rise(Global_Variable.Suspende_PositionTarget-Global_Variable.Para_Independence.Suspende_Limit_Up);
            }
       }
    }
    else if(CMD_Rope_Wire == ON)//单个吊杆收揽（复位吊杆位置，就是吊杆的最高位置）
    {
        CMD_Rope_Wire = OFF;
        MotorMove_Rise(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
    }
    else if(CMD_Suspender_Min == ON)//单个吊杆降到零点坐标位置（吊杆的最低位置）
    {
        CMD_Suspender_Min = OFF;
        MotorMove_Fall(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
    }
    else if(CMD_Suspender_Target == ON)//单个吊杆运行到设定的目标位置
    {
        CMD_Suspender_Target = OFF;
        if(Global_Variable.Suspende_PositionCurrent > Global_Variable.Suspende_PositionTarget)
        {
            if(MOTOR_RUNNING_STS == DIR_RISE)
            {
                Reserve_Requrirement = ON;
            }
            else
            {
                MotorMove_Fall(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
            }
        }
        else
        {
            if(MOTOR_RUNNING_STS == DIR_FALL)
            {
                Reserve_Requrirement = ON;
            }
            else
            {
                MotorMove_Rise(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
            }
        }
    }
    else if(CMD_Limit_Measure == ON)
    {
        CMD_Limit_Measure = OFF;
        if(MOTOR_RUNNING_CMD != Measure_Distance)
        {
            if((Limit_Fall_Signal == OFF)&&(Global_Variable.Suspende_PositionCurrent>0))
            {
                MOTOR_RUNNING_CMD = Measure_Distance;
                MEASURE_LIMIT_STS = M_MEASURING;
                Global_Variable.Suspende_PositionTarget = -60000;//向下走最大行程，直到遇到下限位开关
                MotorMove_Fall(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
            }
            else
            {
                //不执行限位测量
            }
        }
    }
    else
    {

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
    s32 Delta_Pulse;
    u8 pre_limit_rise;
    u8 pre_limit_fall;
    u8 pre_Stop_Signal;
    u8 pre_Summit_Attempt;

    Freq_Convert_Init();
    
    while (1U+1U==2U)
    {          
        if(Err_Stop_Signal == ON)//急停按钮：吊杆急停
        {
            if(pre_Stop_Signal == OFF)
            {
                BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                Motor_Stop(Motor_Stop_Free);   
            }
        }
        pre_Stop_Signal = Err_Stop_Signal;

        if(Err_Summit_Attempt == ON)//冲顶故障：吊杆急停
        {
            if(pre_Summit_Attempt == OFF)
            {
                BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                Motor_Stop(Motor_Stop_Free);   
            }
        }
        pre_Summit_Attempt = Err_Summit_Attempt;
    
        if((FORCE_REDUCE_EN == ON)&&(MOTOR_REDUCING == OFF))//强制减速完成，当前频率是最小频率10HZ
        {//此条件必须放在该while的最上边判断，否则会导致条件无法满足
            if(MOTOR_RUNNING_CMD == Motor_Init)
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
                    if(Global_Variable.Suspende_PositionCurrent > Global_Variable.Suspende_PositionTarget)
                    {
                        MotorMove_Fall(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
                    }
                    else
                    {
                        MotorMove_Rise(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
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


        if(MOTOR_RUNNING_STS != OFF)//电机处于运行状态
        {
            if(((Limit_Up_SlowDown == ON)&&(MOTOR_RUNNING_STS == DIR_RISE))||((Limit_Down_SlowDown == ON)&&(MOTOR_RUNNING_STS == DIR_FALL))||
                (Reserve_Requrirement == ON))//运行过程中遇到限位减速信号
            {
                if(MOTOR_RUNNING_CMD == Motor_Normal)//吊杆初始化和测距需要运行至限位开关处
                {
                    FORCE_REDUCE_EN = ON;
                }
            }
            else
            {
                FORCE_REDUCE_EN = OFF;
            }

            
            if(MOTOR_RUNNING_STS == DIR_RISE)
            {//电机上升
                if(Limit_Rise_Signal == ON)
                {//运行过程中遇到上限位信号
                    MOTOR_RUNNING_CMD = Motor_Normal;
                    if(BAND_TYPE_BRAKE_OUT == ON)
                    {
                        BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                    }
                    Motor_Stop(Motor_Stop_Reduce);
                    
                    if(pre_limit_rise == OFF)
                    {//需要进行位置补偿
                        
                        if(MEASURE_LIMIT_STS == M_MEASURING)
                        {
                            MEASURE_LIMIT_STS = M_SUCCESS;
                            Limit_Position = Global_Variable.Suspende_PositionCurrent-Limit_Position;//记录上限位脉冲数
                            Global_Variable.Para_Independence.Suspende_Limit_Up = Limit_Position;
                            CMD_ParaDownload_Independent = ON;//写个性化参数
                        }
                        else
                        {
                            MOTOR_CORRENT_STS = Corrent_Up; 
                        }
                    }
                }
                else if(Global_Variable.Suspende_PositionCurrent >= Global_Variable.Para_Independence.Suspende_Limit_Up)
                {
                    if((MEASURE_LIMIT_STS != M_MEASURING)&&(MOTOR_RUNNING_CMD != Motor_Init))//吊杆初始化时需要运行至限位开关处
                    {
                        if(BAND_TYPE_BRAKE_OUT == ON)
                        {
                            BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                        }
                        Motor_Stop(Motor_Stop_Reduce);
                    }
                }
                else
                {

                }
            }
            else if(MOTOR_RUNNING_STS == DIR_FALL)
            {//电机下降
                if(Limit_Fall_Signal == ON)
                {//运行过程中遇到下限位信号
                    if(BAND_TYPE_BRAKE_OUT == ON)
                    {
                        BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                    }
                    Motor_Stop(Motor_Stop_Reduce);

                    if(pre_limit_fall == OFF)
                    {//需要进行位置补偿
                        if(MEASURE_LIMIT_STS == M_MEASURING)
                        {
                            Limit_Position = Global_Variable.Suspende_PositionCurrent;//记录下限位脉冲数
                            Global_Variable.Suspende_PositionTarget = 60000;//向下走最大行程，直到遇到下限位开关
                            MotorMove_Rise(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
                        }
                        else
                        {
                            MOTOR_CORRENT_STS = Corrent_Down; 
                        }
                    }
                }
                else if(Global_Variable.Suspende_PositionCurrent <= 0)
                {
                    if(MEASURE_LIMIT_STS != M_MEASURING)
                    {
                        if(BAND_TYPE_BRAKE_OUT == ON)
                        {
                            BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                        }
                        Motor_Stop(Motor_Stop_Reduce);
                    }
                }
                else
                {

                }
            }
            else//电机处于空闲状态
            {

            }
            pre_limit_rise = Limit_Rise_Signal;
            pre_limit_fall = Limit_Fall_Signal;
        }

        if(cTimer[Motor_Correct] >= 6U)//遇到限位开关后等待600ms，再修正脉冲数和当前位置值
        {
            cTimer[Motor_Correct] = 0;
            if(MOTOR_CORRENT_STS == Corrent_Up)
            {
                MOTOR_CORRENT_STS = Corrent_None;
                OS_ENTER_CRITICAL(); 
                Global_Variable.Suspende_PositionCurrent = Global_Variable.Para_Independence.Suspende_Limit_Up;
                Global_Variable.Compensate_Pulse = 0 - Global_Variable.Encode_PulseCurrent;//过冲了需要补偿正值
                Global_Variable.Compensate_En = ON;
                OS_EXIT_CRITICAL();
            }
            else if(MOTOR_CORRENT_STS == Corrent_Down)
            {
                MOTOR_CORRENT_STS = Corrent_None;
                OS_ENTER_CRITICAL(); 
                Global_Variable.Suspende_PositionCurrent = 0;
                Global_Variable.Compensate_Pulse = (Global_Variable.Para_Independence.Suspende_Limit_Up/Global_Variable.Para_Independence.Lenth_Per_Pulse)
                    - Global_Variable.Encode_PulseCurrent;//过冲了就补偿负值
                Global_Variable.Compensate_En = ON;
                OS_EXIT_CRITICAL();
            }
            else
            {

            }
        }

        
        if(MOTOR_RUNNING_STS == DIR_FALL)
        {
            Delta_Pulse = Global_Variable.Encode_PulseTarget - Global_Variable.Encode_PulseCurrent;
        }
        else if(MOTOR_RUNNING_STS == DIR_RISE)
        {
            Delta_Pulse = Global_Variable.Encode_PulseCurrent - Global_Variable.Encode_PulseTarget;
        }
        else
        {//电机处于空闲状态

        }

        if(Global_Variable.Para_Independence.Convert_Cfg == ON)
        { //有变频器配置
            if((Delta_Pulse < REMAIN_PULSE_NUMBER_FOR_FREQ_STOP)||(Delta_Pulse < 0))
            {
                if(MOTOR_RUNNING_CMD != Motor_Init)//吊杆初始化时需要运行至限位开关处
                {
                    if(BAND_TYPE_BRAKE_OUT == ON)
                    {//电机到达目标位置时，就立即抱闸，不要提前抱闸
                        cTimer[Motor_Delay] = 0;
                        BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                    }
                    if(MOTOR_RUNNING_STS != OFF)/*&&(MOTOR_REDUCING == OFF)*/
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

        CMD_Freq_Convert(); 

        OSTimeDlyHMSM(0, 0, 0, 1);
    }
    
}


#endif

