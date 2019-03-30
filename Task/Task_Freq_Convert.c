#include "main.h"
#include "Task_IO.h"
#include "Task_MB_RTU_Master.h"
#include "Task_Freq_Convert.h"
#include "Task_MQTT.h"

#ifdef __TASK_FREQ_CONVERT_H

OS_EVENT *mBOX_LED_R;
OS_EVENT *mBOX_LED_G;

volatile BitStatus Invertor_Status;
#define MOTOR_RUN_DELAY                   Invertor_Status.Bits.bit0

u8 InvertorData[80];
u16 Motor_DelayTime;

enum Write_Data_Off{
    Control_CMD,Convert_Freq
};
enum Init_Parameter_Off{
    Freq_Channel=0,Protocol_Select,Modbus_Type
    
};

u8 Wrdata[4]={0,0,0,0};
u8 Init_parameter[20]={
    0x00,0x09,//主频率给定通道1选择
    0x00,0x00,//系统通讯协议选择
    0x00,0x00,//MODBUS通讯数据格式
};

static INT8U err;

RTU_ReqBlock_t Init_Freq_Channel = //RTU数据请求块-主频率给定通道1选择
{
    LIST_HEAD_INIT(Init_Freq_Channel.Entry),
    1,                                          //执行次数，0-无限次
    UART_CHN_CONVERT_FREQ,                      //执行通道
	1,                                          //从节点站地址
	FUNC_WR_SGREG,                              //功能码
	EXCUTE_SUCCESS,                             //执行结果
	0x0001,                                     //操作寄存器地址
	0x01,                                       //操作寄存器数量
	(u8*)&InvertorData[Freq_Channel*2]          //执行的数据，读取的寄存器数据或写操作的数据
};

RTU_ReqBlock_t Init_Protocol_Select = //RTU数据请求块-系统通讯协议选择
{
    LIST_HEAD_INIT(Init_Protocol_Select.Entry),
    1,                                          //执行次数，0-无限次
    UART_CHN_CONVERT_FREQ,                      //执行通道
	1,                                          //从节点站地址
	FUNC_WR_SGREG,                              //功能码
	EXCUTE_SUCCESS,                             //执行结果
	0x0027,                                     //操作寄存器地址
	0x01,                                       //操作寄存器数量
	(u8*)&InvertorData[Protocol_Select*2]       //执行的数据，读取的寄存器数据或写操作的数据
};

RTU_ReqBlock_t Init_Modbus_Type = //RTU数据请求块-MODBUS通讯数据格式,0：非标准的MODBUS协议 1：标准的MODBUS协议
{
    LIST_HEAD_INIT(Init_Protocol_Select.Entry),
    1,                                          //执行次数，0-无限次
    UART_CHN_CONVERT_FREQ,                      //执行通道
    1,                                          //从节点站地址
    FUNC_WR_SGREG,                              //功能码
    EXCUTE_SUCCESS,                             //执行结果
    0x0027,                                     //操作寄存器地址
    0x01,                                       //操作寄存器数量
    (u8*)&InvertorData[Modbus_Type*2]       //执行的数据，读取的寄存器数据或写操作的数据
};

RTU_ReqBlock_t RTU_Req_Read5000 = //RTU数据请求块-变频器状态参数地址
{
    LIST_HEAD_INIT(RTU_Req_Read5000.Entry),
    0,                                          //执行次数，0-无限次
    UART_CHN_CONVERT_FREQ,                      //执行通道
	1,                                          //从节点站地址
	FUNC_RD_HOLDREG,                            //功能码
	EXCUTE_SUCCESS,                             //执行结果
	0x5000,                                     //操作寄存器地址
	0x20,                                       //操作寄存器数量
	(u8*)&InvertorData[off_FreqByComm*2]          //执行的数据，读取的寄存器数据或写操作的数据
};

RTU_ReqBlock_t RTU_Req_Read8000 = //RTU数据请求块-变频器故障地址,变频器通讯异常地址
{
    LIST_HEAD_INIT(RTU_Req_Read8000.Entry),
    0,                                          //执行次数，0-无限次
    UART_CHN_CONVERT_FREQ,                      //执行通道
	1,                                          //从节点站地址
	FUNC_RD_HOLDREG,                            //功能码
	EXCUTE_SUCCESS,                             //执行结果
	0x8000,                                     //操作寄存器地址
	0x02,                                       //操作寄存器数量
	(u8*)&InvertorData[off_InvertorError*2]       //执行的数据，读取的寄存器数据或写操作的数据
};


struct RTU_ReqBlock RTU_Req_Write6000= //RTU数据请求块-控制命令 1-正转 2-反转 3-正转点动 4-反转点动 5-减速停机 6-自由停机 7-故障复位
{
	LIST_HEAD_INIT(RTU_Req_Write6000.Entry),
    1,                                          //执行次数，0-无限次
	UART_CHN_CONVERT_FREQ,                      //执行通道
	1,                                          //从节点站地址
	FUNC_WR_SGREG,                              //功能码06
	EXCUTE_SUCCESS,                             //执行结果
	0x6000,                                     //操作寄存器地址
	0x01,                                       //操作寄存器数量
	(u8*)&Wrdata[Control_CMD*2]                   //执行的数据，读取的寄存器数据或写操作的数据
};

struct RTU_ReqBlock RTU_Req_WriteFreq= //RTU数据请求块
{
	LIST_HEAD_INIT(RTU_Req_WriteFreq.Entry),
    1,                                          //执行次数，0-无限次
	UART_CHN_CONVERT_FREQ,                      //执行通道
	1,                                          //从节点站地址
	FUNC_WR_SGREG,                              //功能码06
	EXCUTE_SUCCESS,                             //执行结果
	0x0002,                                     //操作寄存器地址
	0x01,                                       //操作寄存器数量
	(u8*)&Wrdata[Convert_Freq*2]                  //执行的数据，读取的寄存器数据或写操作的数据
};



/********************************************************************************/
/*函数名：  Freq_Convert_Init                                                   */
/*功能说明：模块初始化函数                                                       */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
static void Freq_Convert_Init(void)
{
    Invertor_Status.Byte = 0;
    Motor_DelayTime=0;
    RTU_AddReqBlock(&rtu_ctx,&RTU_Req_Read5000);
    RTU_AddReqBlock(&rtu_ctx,&RTU_Req_Read8000);
    RTU_AddReqBlock(&rtu_ctx,&Init_Freq_Channel);
    RTU_AddReqBlock(&rtu_ctx,&Init_Protocol_Select);
    RTU_AddReqBlock(&rtu_ctx,&Init_Modbus_Type);
}

/********************************************************************************/
/*函数名：  TaskFreq_Timer1ms                                                   */
/*功能说明：1ms定时函数                                                       */
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

/********************************************************************************/
/*函数名：  Motor_Forward                                                   */
/*功能说明：电机正转                                                         */
/*输入参数：speed-频率  position-位置                                                */
/*输出参数：无                                                                  */
/*******************************************************************************/
void Motor_Forward(u16 speed)
{
    Wrdata[Control_CMD*2] = 1;
    Wrdata[Convert_Freq*2] = Global_Variable.Suspende_Target_Speed>>8;
    Wrdata[Convert_Freq*2+1] = Global_Variable.Suspende_Target_Speed;
    RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteFreq);
    RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_Write6000);
}

/********************************************************************************/
/*函数名：  Motor_Backward                                                   */
/*功能说明：电机反转                                                         */
/*输入参数：speed-频率  position-位置                                                */
/*输出参数：无                                                                  */
/*******************************************************************************/
void Motor_Backward(u16 speed)
{
    Wrdata[Control_CMD*2] = 2;
    Wrdata[Convert_Freq*2] = Global_Variable.Suspende_Target_Speed>>8;
    Wrdata[Convert_Freq*2+1] = Global_Variable.Suspende_Target_Speed;
    RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteFreq);
    RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_Write6000);
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
        Delta_Position = Global_Variable.Suspende_Target_Position - Global_Variable.Wire_Position;
        
        if((Delta_Position < 10.0)&&(Delta_Position > -10.0))
        {//判断当前位置是否到，先用此方式粗略处理下，误差在10mm内认为已经到目标位置，后续再精确计算
            Wrdata[Control_CMD] = 6;
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteFreq);
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_Write6000);
            BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
        }
        
        if(CMD_Rope_Wire == ON)//单个吊杆收揽（复位吊杆位置，就是吊杆的最高位置）
        {
            if(BAND_TYPE_BRAKE_OUT == OFF)
            {
                MOTOR_RUN_DELAY = ON;
                BAND_TYPE_BRAKE_OUT = ON;//抱闸闭合
            }
            if(Motor_DelayTime >= 1000)
            {
                //Motor_DelayTime = 0;
                MOTOR_RUN_DELAY = OFF;
                CMD_Rope_Wire = OFF;
                Motor_Forward(Global_Variable.Suspende_Target_Speed);
            }
        }
        if(CMD_Suspender_Min == ON)//单个吊杆降到零点坐标位置（吊杆的最低位置）
        {
            if(BAND_TYPE_BRAKE_OUT == OFF)
            {
                MOTOR_RUN_DELAY = ON;
                BAND_TYPE_BRAKE_OUT = ON;//抱闸闭合
            }
            if(Motor_DelayTime >= 1000)
            {
                //Motor_DelayTime = 0;
                MOTOR_RUN_DELAY = OFF;
                CMD_Suspender_Min = OFF;
                Motor_Backward(Global_Variable.Suspende_Target_Speed);
            }
        }
        if(CMD_Suspender_Emergency_Stop == ON)//单个吊杆急停
        {
            BAND_TYPE_BRAKE_OUT = OFF;//抱闸断开
            CMD_Suspender_Emergency_Stop = OFF;
            
        }
        if(CMD_Suspender_Target == ON)//单个吊杆运行到设定的目标位置
        {
            if(BAND_TYPE_BRAKE_OUT == OFF)
            {
                MOTOR_RUN_DELAY = ON;
                BAND_TYPE_BRAKE_OUT = ON;//抱闸闭合
            }
            if(Motor_DelayTime >= 1000)
            {
                //Motor_DelayTime = 0;
                MOTOR_RUN_DELAY = OFF;
                CMD_Suspender_Target = OFF;
                Wrdata[Convert_Freq*2] = Global_Variable.Suspende_Target_Speed>>8;
                Wrdata[Convert_Freq*2+1] = Global_Variable.Suspende_Target_Speed;
                RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteFreq);
                RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_Write6000);
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
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteFreq);
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_Write6000);
        }

        OSTimeDlyHMSM(0, 0, 0, 1);
    }
    
}


#endif


