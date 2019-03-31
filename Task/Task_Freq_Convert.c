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
    P0_00_CtrlMode=0,           //���Ʒ�ʽѡ��
    P0_01_Freq_Channel,         //��Ƶ�ʸ���ͨ��1ѡ��
    P0_03_RunCmd_Channel,       //��������ͨ��ѡ��
    P0_24_CMD_FREQ_CHN,         //����ͨ����Ƶ�ʸ���ͨ����ϵ�趨
    PC_00_Comm_Baudrate,        //ͨѶ������
    PC_01_DataType,             //MODBUS���ݸ�ʽ
    PC_05_ModbusType,           //MODBUSͨѶ���ݸ�ʽ
                                //
    Init_Group_Total//�����������б��������һ������Ҫ�����Ŀʱ����һ��
};

typedef struct Init_Para_Type_Info
{
    u16 ParaAddr;
    u16 DataValue;
}Init_Para_t;

u16 Wrdata[4]={0x1234,0x5678,0,0};//д����������

struct RTU_ReqBlock Init_Point[Init_Group_Total]; //��ʼ������

Init_Para_t Reg_InitGroup[Init_Group_Total]=//��ʼ���������ã�modbus��ַ+��������
{
    {0x0000,0x0000},    //���Ʒ�ʽѡ�� P0_00_CtrlMode
    {0x0001,0x0009},    //��Ƶ�ʸ���ͨ��1ѡ�� P0_01_Freq_Channel
    {0x0003,0x0002},    //��������ͨ��ѡ�� P0_03_RunCmd_Channel
    {0x0018,9999},      //����ͨ����Ƶ�ʸ���ͨ����ϵ�趨 P0_24_CMD_FREQ_CHN
    {0x4000,0x0005},    //ͨѶ������ PC_00_Comm_Baudrate
    {0x4001,0x0000},    //MODBUS���ݸ�ʽ PC_01_DataType
    {0x4005,0x0001},    //MODBUSͨѶ���ݸ�ʽ PC_05_ModbusType
};


RTU_ReqBlock_t RTU_Req_Read8000 = //RTU���ݶ������-��Ƶ�����ϵ�ַ,��Ƶ��ͨѶ�쳣��ַ
{
    LIST_HEAD_INIT(RTU_Req_Read8000.Entry),
    0,                                          //ִ�д�����0-���޴�
    UART_CHN_CONVERT_FREQ,                      //ִ��ͨ��
	SLAVEID_FREQ,                               //�ӽڵ�վ��ַ
	FUNC_RD_HOLDREG,                            //������
	EXCUTE_SUCCESS,                             //ִ�н��
	0x8000,                                     //�����Ĵ�����ַ
	0x01,                                       //�����Ĵ�������
	(u16*)&InvertorData[off_InvertorError]      //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};


struct RTU_ReqBlock RTU_Req_WriteCMD_6000= //RTU����д�����-�������� 1-��ת 2-��ת 3-��ת�㶯 4-��ת�㶯 5-����ͣ�� 6-����ͣ�� 7-���ϸ�λ
{
	LIST_HEAD_INIT(RTU_Req_WriteCMD_6000.Entry),
    1,                                          //ִ�д�����0-���޴�
	UART_CHN_CONVERT_FREQ,                      //ִ��ͨ��
	SLAVEID_FREQ,                                          //�ӽڵ�վ��ַ
	FUNC_WR_SGREG,                              //������06
	EXCUTE_SUCCESS,                             //ִ�н��
	0x6000,                                     //�����Ĵ�����ַ
	0x01,                                       //�����Ĵ�������
	(u16*)&Wrdata[Control_CMD]                  //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};

struct RTU_ReqBlock RTU_Req_WriteFreq_5000= //RTU���������,��������Ƶ��
{
	LIST_HEAD_INIT(RTU_Req_WriteFreq_5000.Entry),
    1,                                          //ִ�д�����0-���޴�
	UART_CHN_CONVERT_FREQ,                      //ִ��ͨ��
	SLAVEID_FREQ,                                          //�ӽڵ�վ��ַ
	FUNC_WR_SGREG,                              //������06
	EXCUTE_SUCCESS,                             //ִ�н��
	0x5000,                                     //�����Ĵ�����ַ
	0x01,                                       //�����Ĵ�������
	(u16*)&Wrdata[Convert_Freq]                 //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};




/********************************************************************************/
/*��������  Freq_Convert_Init                                                   */
/*����˵����ģ���ʼ������                                                       */
/*�����������                                                                  */
/*�����������                                                                  */
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
        Init_Point[i].Excute_Num = 1u;                      //ִ�д�����0-���޴�
        Init_Point[i].chnindex = UART_CHN_CONVERT_FREQ;     //ִ��ͨ��
        Init_Point[i].sta_addr = SLAVEID_FREQ;              //�ӽڵ�վ��ַ
        Init_Point[i].FuncCode = FUNC_WR_SGREG;             //������06
        Init_Point[i].Status = EXCUTE_SUCCESS;              //ִ�н��
        Init_Point[i].RegAddr = Reg_InitGroup[i].ParaAddr;  //�����Ĵ�����ַ
        Init_Point[i].RegNum = 1u;                          //�����Ĵ�������
        Init_Point[i].mappedBuff = (u16*)&Reg_InitGroup[i].DataValue;   //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
        RTU_AddReqBlock(&rtu_ctx,&Init_Point[i]);
    }
    //RTU_AddReqBlock(&rtu_ctx,&RTU_Req_Read8000);//��Ӷ�������Ϣ���󣬺�̨��ʼ�����ж�����
}

/********************************************************************************/
/*��������  TaskFreq_Timer1ms                                                   */
/*����˵����1ms��ʱ����                                                         */
/*�����������                                                                  */
/*�����������                                                                  */
/*******************************************************************************/
void TaskFreq_Timer1ms(void)
{
    if(MOTOR_RUN_DELAY == ON)
    {
        Motor_DelayTime++;
    }
}

/****************************************************************************************/
/*��������  Calculate_Frequence                                                          */
/*����˵����������������Ƶ��                                                            */
/*�����������                                                                           */       
/*���������f(Ƶ��)=��50*X���趨�ٶȣ���/[995*(D1(���ٻ�ֱ��)+D2����˿��ֱ����*3.14/���ٱ�)]*/
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
    //Ŀǰ����ֵ����ȷ����ʱ����1000�·�����Ƶ��
    temp = 1000;
    return (u16)temp;
}


/********************************************************************************/
/*��������  Motor_Forward                                                        */
/*����˵���������ת����                                                          */
/*�����������                                                                   */
/*�����������                                                                  */
/*******************************************************************************/
void Motor_Forward(void)
{
    Wrdata[Control_CMD] = Motor_Fardward_Run;
    Wrdata[Convert_Freq] = Calculate_Frequence();
    RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteCMD_6000);
    RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteFreq_5000);
}

/********************************************************************************/
/*��������  Motor_Backward                                                      */
/*����˵���������ת����                                                         */
/*�����������                                                                  */
/*�����������                                                                  */
/*******************************************************************************/
void Motor_Backward(void)
{
    Wrdata[Control_CMD] = Motor_Backward_Run;
    Wrdata[Convert_Freq] = Calculate_Frequence();
    RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteCMD_6000);
    RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteFreq_5000);
}

/********************************************************************************/
/*��������  Motor_Stop_Free                                                       */
/*����˵�������ͣ��                                                         */
/*�����������                                                                  */
/*�����������                                                                  */
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
/*��������  Task_Freq_Convert                                                   */
/*����˵������Ƶ����task                                                         */
/*���������p_arg                                                               */
/*�����������                                                                  */
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
        {//�жϵ�ǰλ���Ƿ񵽣����ô˷�ʽ���Դ����£������10mm����Ϊ�Ѿ���Ŀ��λ�ã������ٸ��ݼ��ٶȾ�ȷ����
            if(MOTOR_RUNNING == ON)
            {
                Motor_DelayTime = 0;
                Motor_Stop(Motor_Stop_Reduce);
                BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                MOTOR_RUNNING = OFF;
            }
        }
        
        if(CMD_Rope_Wire == ON)//����������������λ����λ�ã����ǵ��˵����λ�ã�
        {
            MOTOR_RUNNING = ON;
            //if(BAND_TYPE_BRAKE_OUT == OFF)
            {
                MOTOR_RUN_DELAY = ON;
                //BAND_TYPE_BRAKE_OUT = ON;//��բ�պ�
            }
            //if(Motor_DelayTime >= 1000)
            {
                //Motor_DelayTime = 0;
                MOTOR_RUN_DELAY = OFF;
                CMD_Rope_Wire = OFF;
                Motor_Forward();
            }
        }
        if(CMD_Suspender_Min == ON)//�������˽����������λ�ã����˵����λ�ã�
        {
            MOTOR_RUNNING = ON;
            //if(BAND_TYPE_BRAKE_OUT == OFF)
            {
                MOTOR_RUN_DELAY = ON;
                //BAND_TYPE_BRAKE_OUT = ON;//��բ�պ�
            }
            //if(Motor_DelayTime >= 1000)
            {
                //Motor_DelayTime = 0;
                MOTOR_RUN_DELAY = OFF;
                CMD_Suspender_Min = OFF;
                Motor_Backward();
            }
        }
        if(CMD_Suspender_Emergency_Stop == ON)//�������˼�ͣ
        {
            BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
            CMD_Suspender_Emergency_Stop = OFF;
            Motor_Stop(Motor_Stop_Free);            
        }
        if(CMD_Suspender_Target == ON)//�����������е��趨��Ŀ��λ��
        {
            MOTOR_RUNNING = ON;
            //if(BAND_TYPE_BRAKE_OUT == OFF)
            {
                MOTOR_RUN_DELAY = ON;
                //BAND_TYPE_BRAKE_OUT = ON;//��բ�պ�
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
        if(CMD_ParaDownload_Independent == ON)//����΢���������Ի���������
        {
            CMD_ParaDownload_Independent = OFF;
            
        }
        
        //cnt++;
        if(cnt==10000)
        {//����Ϊ����
            cnt = 0;
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteFreq_5000);
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteCMD_6000);
        }

        OSTimeDlyHMSM(0, 0, 0, 1);
    }
    
}


#endif


