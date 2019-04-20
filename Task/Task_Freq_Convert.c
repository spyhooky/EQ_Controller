#include "main.h"
#include "Task_IO.h"
#include "Task_PC_Com.h"
#include "Task_MB_RTU_Master.h"
#include "Task_Freq_Convert.h"
#include "Task_MQTT.h"
#include <stdlib.h>


#ifdef __TASK_FREQ_CONVERT_H

volatile BitStatus Invertor_Status;
volatile BitStatus Motor_Status;
#define MOTOR_RUNNING                     Motor_Status.Bits.bit0 //电机运行标志
#define MOTOR_RUN_DELAY                   Motor_Status.Bits.bit1 //电机运行延时，用于抱闸
#define MOTOR_DIRECTOR                    Motor_Status.Bits.bit2 //吊杆运行方向，上升或者下降
#define MOTOR_REDUCING                    Motor_Status.Bits.bit3 //电机减速标志
#define READ_CURR_FREQ_EN                 Motor_Status.Bits.bit4 //是否需要发送查询马达当前频率值的标志
#define Reserve_Requrirement              Motor_Status.Bits.bit5 //电机需要反向运行，先减速再反向  
#define FORCE_REDUCE_EN                   Motor_Status.Bits.bit6 //遇到上下限位开关或者电机需要反向时置此标志
#define FORCE_REDUCE_10HZ                 Motor_Status.Bits.bit7 //强制减速时，频率到10HZ的标志



#define FREQ_REDUCE_TABLE_NUM             15U

enum Timer_Type{
    Motor_Delay,            //松开抱闸的计时，电机运行1-2s后松开，停止减速时再抱紧
    Read8000,               //查询帧定时计数器
    Read5001,               //读取当前运行频率的计数器
    Freq_Reduce,            //减速间隔时间计数器
    Keep_10HZ,              //反向或者限位减速信号后频率减到10HZ时需要继续维持的时间
    
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
    u16 ParaAddr;
    u16 DataValue;
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
    {120,20000},//若工作频率为120HZ，则当剩余脉冲数小于20000时就开始减速
    {100,15000},
    {90,10000},
    {80,8000},
    {70,6500},
    {60,5000},
    {50,4000},
    {40,3000},
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
	(u16*)&WriteData[Control_CMD]                  //执行的数据，读取的寄存器数据或写操作的数据
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
	(u16*)&WriteData[Convert_Freq]                 //执行的数据，读取的寄存器数据或写操作的数据
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
    Invertor_Status.Byte = 0;
    memset((u8 *)&cTimer[0],0,sizeof(cTimer));
    Global_Variable.Suspende_Target_Position = INIT_POSITION_WIRE;
    Global_Variable.Suspende_Current_Position = INIT_POSITION_WIRE;
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
        {
            if(MOTOR_REDUCING == OFF)
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
    {
        cTimer[Freq_Reduce]++;
        cTimer[Read5001] = 0;
    }
    else
    {
        cTimer[Freq_Reduce] = 0;
        cTimer[Read5001]++;
    }

    if(FORCE_REDUCE_10HZ == ON)
    {
        cTimer[Keep_10HZ]++;
    }
    else
    {
        cTimer[Keep_10HZ] = 0;
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
    temp = Global_Variable.Suspende_Target_Speed*Global_Variable.Para_Independence.Motor_Freq_Factor;
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
        if(Global_Variable.Encode_TargetPulse - Global_Variable.Encode_CurrentPulse >= 2000)
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
        if(Global_Variable.Encode_CurrentPulse - Global_Variable.Encode_TargetPulse >= 2000)
        {
            motor_freq = Calculate_Frequence();
        }
        else
        {
            motor_freq = Motor_Freq_MIN;
            MOTOR_REDUCING = ON;
        }
    }
    Global_Variable.Suspende_Current_Speed = ((u32)motor_freq*(u32)Global_Variable.Para_Independence.Max_Motro_Freq)/10000;
    Global_Variable.Suspende_Current_Speed /= Global_Variable.Para_Independence.Motor_Freq_Factor;
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
        if(cTimer[Read5001] >= 20)
        {//周期性读取当前电机频率
            cTimer[Read5001] = 0;
            RTU_AddReqBlock(&rtu_ctx,&RTU_Req_ReadFreq_5001);
            Global_Variable.Suspende_Current_Speed = (InvertorData[off_CurrFreq]/100)/Global_Variable.Para_Independence.Motor_Freq_Factor;
            //Global_Variable.Suspende_Current_Speed = (InvertorData[off_CurrFreq]/100)/Global_Variable.Para_Independence.Motor_Freq_Factor;//*10000/100
        }
        
        for(i=0;i<FREQ_REDUCE_TABLE_NUM;i++)
        {
            if((InvertorData[off_CurrFreq]/100) >= Table_Freq_Reduce[i].reduce_freq)
            {
                if((FORCE_REDUCE_EN == ON)||(Table_Freq_Reduce[i].pulse_remain >= Delta_Pulse))
                {
                    Global_Variable.Suspende_Current_Speed = Table_Freq_Reduce[i].reduce_freq/Global_Variable.Para_Independence.Motor_Freq_Factor;
                    motor_freq = Table_Freq_Reduce[i].reduce_freq*10000/Global_Variable.Para_Independence.Max_Motro_Freq;//
                    MOTOR_REDUCING = ON;
                    break;
                }
            }
        }
        motor_freq = WriteData[Convert_Freq];
    }
    else
    {
        if(cTimer[Freq_Reduce] >= FREQ_REDUCE_INTERTER)
        {
            cTimer[Freq_Reduce] = 0;
            Global_Variable.Suspende_Current_Speed -= (u16)(FREQ_REDUCE_BASE/Global_Variable.Para_Independence.Motor_Freq_Factor);
            curfreq = Global_Variable.Suspende_Current_Speed * Global_Variable.Para_Independence.Motor_Freq_Factor;
            motor_freq = curfreq*10000/Global_Variable.Para_Independence.Max_Motro_Freq;//
        }
        else
        {
            motor_freq = WriteData[Convert_Freq];
        }
    }
    return motor_freq;
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
/*函数名：  Motor_Forward                                                        */
/*功能说明：电机正转命令                                                          */
/*输入参数：无                                                                   */
/*输出参数：无                                                                  */
/*******************************************************************************/
void MotorMove_Fall(void)
{
    MOTOR_DIRECTOR = D_FALL;
    Set_Frequence_Start();
    //if(Wrdata[Control_CMD] != Motor_Fardward_Run)
    {
        WriteData[Control_CMD] = Motor_Fardward_Run;
        RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteCMD_6000);
    }
}

/********************************************************************************/
/*函数名：  Motor_Backward                                                      */
/*功能说明：电机反转命令                                                         */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
void MotorMove_Rise(void)
{
    MOTOR_DIRECTOR = D_RISE;
    Set_Frequence_Start();
    //if(Wrdata[Control_CMD] != Motor_Backward_Run)
    {
        WriteData[Control_CMD] = Motor_Backward_Run;
        RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteCMD_6000);
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
    if((stoptype ==Motor_Stop_Reduce)||(stoptype ==Motor_Stop_Free))
    {   
        stopmode = stoptype;
    }
    else
    {
        stopmode = Motor_Stop_Free;
    }
    if(WriteData[Control_CMD] != stopmode)
    {
        WriteData[Control_CMD] = stopmode;
        RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteCMD_6000);
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
    while (1)
    {        
        if((FORCE_REDUCE_EN == ON)&&(MOTOR_REDUCING == OFF))//强制减速完成，当前频率是最小频率10HZ
        {//此条件必须放在该while的最上边判断，否则会导致条件无法满足
            FORCE_REDUCE_EN = OFF;
            FORCE_REDUCE_10HZ = ON;
            cTimer[Keep_10HZ] = 0;
        }

        if(cTimer[Keep_10HZ] >= FORCE_REDUCE_10HZ_KEEPING)
        {
            if(Reserve_Requrirement == ON)
            {
                cTimer[Motor_Delay] = 0;//重新计时2s再松抱闸
                Global_Variable.Encode_TargetPulse = (s32)((Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_Target_Position)/ \
                    Global_Variable.Para_Independence.Lenth_Per_Pulse);
                if(Global_Variable.Encode_TargetPulse > Global_Variable.Encode_CurrentPulse)
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
                MOTOR_RUNNING = OFF;
                Motor_Stop(Motor_Stop_Reduce);
            }
        }
        
        if((Limit_Up_SlowDown == ON)||(Limit_Down_SlowDown == ON)||(Reserve_Requrirement == ON))
        {
            FORCE_REDUCE_EN = ON;
        }
        
        if(MOTOR_DIRECTOR == D_FALL)
        {
            Delta_Pulse = Global_Variable.Encode_TargetPulse - Global_Variable.Encode_CurrentPulse;
        }
        else
        {
            Delta_Pulse = Global_Variable.Encode_CurrentPulse - Global_Variable.Encode_TargetPulse;
        }
        if(Delta_Pulse < 20u)
        {
            if((MOTOR_RUNNING == ON)/*&&(MOTOR_REDUCING == OFF)*/)
            {
                cTimer[Motor_Delay] = 0;
                Motor_Stop(Motor_Stop_Reduce);
                MOTOR_RUNNING = OFF;
            }
        }
        else
        {
            Set_Frequence_Running(Delta_Pulse);
        }
        
        if(CMD_Rope_Wire == ON)//单个吊杆收揽（复位吊杆位置，就是吊杆的最高位置）
        {
            MOTOR_RUNNING = ON;
            CMD_Rope_Wire = OFF;
            Global_Variable.Encode_TargetPulse = 0;
            MotorMove_Rise();
        }
        
        if(CMD_Suspender_Min == ON)//单个吊杆降到零点坐标位置（吊杆的最低位置）
        {
            MOTOR_RUNNING = ON;
            CMD_Suspender_Min = OFF;
            Global_Variable.Encode_TargetPulse = Global_Variable.Para_Independence.Suspende_Limit_Up/Global_Variable.Para_Independence.Lenth_Per_Pulse;
            MotorMove_Fall();
        }
        
        if(CMD_Suspender_Emergency_Stop == ON)//单个吊杆急停
        {
            MOTOR_RUNNING = OFF;
            FORCE_REDUCE_EN = OFF;
            MOTOR_REDUCING = OFF;
            CMD_Suspender_Emergency_Stop = OFF;
            Motor_Stop(Motor_Stop_Free);            
        }
        
        if(CMD_Suspender_Target == ON)//单个吊杆运行到设定的目标位置
        {
            MOTOR_RUNNING = ON;
            CMD_Suspender_Target = OFF;
            Global_Variable.Encode_TargetPulse = (s32)((Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_Target_Position)/ \
                Global_Variable.Para_Independence.Lenth_Per_Pulse);
            if(Global_Variable.Encode_TargetPulse > Global_Variable.Encode_CurrentPulse)
            {
                MotorMove_Fall();
            }
            else
            {
                MotorMove_Rise();
            }
        }

        if(Err_Summit_Attempt == ON)//单个吊杆急停
        {
            if(MOTOR_RUNNING == ON)
            {
                MOTOR_RUNNING = OFF;
                FORCE_REDUCE_EN = OFF;
                MOTOR_REDUCING = OFF;
                Motor_Stop(Motor_Stop_Free);   
            }
        }

        OSTimeDlyHMSM(0, 0, 0, 1);
    }
    
}


#endif


