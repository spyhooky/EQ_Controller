#include "main.h"
#include "Task_IO.h"
#include "Task_MB_RTU_Master.h"
#include "Task_Freq_Convert.h"
#include "Task_MQTT.h"
#include <stdlib.h>

#ifdef __TASK_FREQ_CONVERT_H

OS_EVENT *mBOX_LED_R;
OS_EVENT *mBOX_LED_G;

volatile BitStatus Invertor_Status;
volatile BitStatus Motor_Status;
#define MOTOR_RUNNING                     Motor_Status.Bits.bit0
#define MOTOR_RUN_DELAY                   Motor_Status.Bits.bit1

u16 InvertorData[40];
u16 Motor_DelayTime;

enum Write_Data_Off{
    Control_CMD,Convert_Freq
};
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

u16 Wrdata[4]={0x1234,0x5678,0,0};//写缓冲区数据

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
    0,                                          //执行次数，0-无限次
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
	SLAVEID_FREQ,                                          //从节点站地址
	FUNC_WR_SGREG,                              //功能码06
	EXCUTE_SUCCESS,                             //执行结果
	0x6000,                                     //操作寄存器地址
	0x01,                                       //操作寄存器数量
	(u16*)&Wrdata[Control_CMD]                  //执行的数据，读取的寄存器数据或写操作的数据
};

struct RTU_ReqBlock RTU_Req_WriteFreq_5000= //RTU数据请求块,设置运行频率
{
	LIST_HEAD_INIT(RTU_Req_WriteFreq_5000.Entry),
    1,                                          //执行次数，0-无限次
	UART_CHN_CONVERT_FREQ,                      //执行通道
	SLAVEID_FREQ,                                          //从节点站地址
	FUNC_WR_SGREG,                              //功能码06
	EXCUTE_SUCCESS,                             //执行结果
	0x5000,                                     //操作寄存器地址
	0x01,                                       //操作寄存器数量
	(u16*)&Wrdata[Convert_Freq]                 //执行的数据，读取的寄存器数据或写操作的数据
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
    Motor_DelayTime=0;
    Global_Variable.Suspende_Target_Position = INIT_POSITION_WIRE;
    Global_Variable.Suspende_Current_Position = INIT_POSITION_WIRE;

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
        RTU_AddReqBlock(&rtu_ctx,&Init_Point[i]);
    }
    //RTU_AddReqBlock(&rtu_ctx,&RTU_Req_Read8000);//添加读故障信息请求，后台会始终运行读命令
}

/********************************************************************************/
/*函数名：  TaskFreq_Timer1ms                                                   */
/*功能说明：1ms定时函数                                                         */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
void TaskFreq_Timer1ms(void)
{
    if(MOTOR_RUN_DELAY == ON)
    {
        Motor_DelayTime++;
    }
}

/****************************************************************************************/
/*函数名：  Calculate_Frequence                                                          */
/*功能说明：计算电机的运行频率                                                            */
/*输入参数：无                                                                           */       
/*输出参数：f(频率)=（50*X（设定速度））/[995*(D1(减速机直径)+D2（钢丝绳直径）*3.14/减速比)]*/
/****************************************************************************************/
static u16 Calculate_Frequence(void)
{
    static float temp;
    temp = (MAX_RUNNING_FREQ * Global_Variable.Suspende_Target_Speed)/ 
        (MOTOR_SPEED * (DIAMETER_REDUCER+DIAMETER_WIRE) * 3.14 / REDUCTION_RATIO);
    if(temp > MAX_RUNNING_FREQ)
    {
        temp = MAX_RUNNING_FREQ;
    }
    //目前计算值不正确，暂时按照1000下发至变频器
    temp = 1000;
    return (u16)temp;
}


/********************************************************************************/
/*函数名：  Motor_Forward                                                        */
/*功能说明：电机正转命令                                                          */
/*输入参数：无                                                                   */
/*输出参数：无                                                                  */
/*******************************************************************************/
void Motor_Forward(void)
{
    Wrdata[Control_CMD] = Motor_Fardward_Run;
    Wrdata[Convert_Freq] = Calculate_Frequence();
    RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteCMD_6000);
    RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteFreq_5000);
}

/********************************************************************************/
/*函数名：  Motor_Backward                                                      */
/*功能说明：电机反转命令                                                         */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
void Motor_Backward(void)
{
    Wrdata[Control_CMD] = Motor_Backward_Run;
    Wrdata[Convert_Freq] = Calculate_Frequence();
    RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteCMD_6000);
    RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteFreq_5000);
}

/********************************************************************************/
/*函数名：  Motor_Stop_Free                                                       */
/*功能说明：电机停机                                                         */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
static void Motor_Stop(u8 stoptype)
{
    if((stoptype ==Motor_Stop_Reduce)||(stoptype ==Motor_Stop_Free))
    {   
        Wrdata[Control_CMD] = stoptype;
    }
    else
    {
        Wrdata[Control_CMD] = Motor_Stop_Free;
    }
    RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteCMD_6000);
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
    static u16 cnt=0;
    float Delta_Position;
    while (1)
    {        
        Delta_Position = Global_Variable.Suspende_Target_Position - Global_Variable.Suspende_Current_Position;
        
        if((Delta_Position < 10.0)&&(Delta_Position > -10.0))
        {//判断当前位置是否到，先用此方式粗略处理下，误差在10mm内认为已经到目标位置，后续再根据减速度精确计算
            if(MOTOR_RUNNING == ON)
            {
                Motor_DelayTime = 0;
                Motor_Stop(Motor_Stop_Reduce);
                BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
                MOTOR_RUNNING = OFF;
            }
        }
        
        if(CMD_Rope_Wire == ON)//单个吊杆收揽（复位吊杆位置，就是吊杆的最高位置）
        {
            MOTOR_RUNNING = ON;
            //if(BAND_TYPE_BRAKE_OUT == OFF)
            {
                MOTOR_RUN_DELAY = ON;
                //BAND_TYPE_BRAKE_OUT = ON;//抱闸闭合
            }
            //if(Motor_DelayTime >= 1000)
            {
                //Motor_DelayTime = 0;
                MOTOR_RUN_DELAY = OFF;
                CMD_Rope_Wire = OFF;
                Motor_Forward();
            }
        }
        if(CMD_Suspender_Min == ON)//单个吊杆降到零点坐标位置（吊杆的最低位置）
        {
            MOTOR_RUNNING = ON;
            //if(BAND_TYPE_BRAKE_OUT == OFF)
            {
                MOTOR_RUN_DELAY = ON;
                //BAND_TYPE_BRAKE_OUT = ON;//抱闸闭合
            }
            //if(Motor_DelayTime >= 1000)
            {
                //Motor_DelayTime = 0;
                MOTOR_RUN_DELAY = OFF;
                CMD_Suspender_Min = OFF;
                Motor_Backward();
            }
        }
        if(CMD_Suspender_Emergency_Stop == ON)//单个吊杆急停
        {
            BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
            CMD_Suspender_Emergency_Stop = OFF;
            Motor_Stop(Motor_Stop_Free);            
        }
        if(CMD_Suspender_Target == ON)//单个吊杆运行到设定的目标位置
        {
            MOTOR_RUNNING = ON;
            //if(BAND_TYPE_BRAKE_OUT == OFF)
            {
                MOTOR_RUN_DELAY = ON;
                //BAND_TYPE_BRAKE_OUT = ON;//抱闸闭合
            }
            //if(Motor_DelayTime >= 1000)
            {
                //Motor_DelayTime = 0;
                MOTOR_RUN_DELAY = OFF;
                CMD_Suspender_Target = OFF;
                if(Delta_Position > 0)
{
                    Motor_Forward();
                }
                else
                {
                    Motor_Backward();
                }
            }
        }
        if(CMD_ParaDownload_Independent == ON)//单个微控制器个性化参数下载
        {
            CMD_ParaDownload_Independent = OFF;
            
        }
        
        //cnt++;
        if(cnt==10000)
        {//仅仅为测试
            cnt = 0;
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteFreq_5000);
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteCMD_6000);
        }

        OSTimeDlyHMSM(0, 0, 0, 1);
    }
    
}


#endif


