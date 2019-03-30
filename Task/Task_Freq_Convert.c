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
    0x00,0x09,//��Ƶ�ʸ���ͨ��1ѡ��
    0x00,0x00,//ϵͳͨѶЭ��ѡ��
    0x00,0x00,//MODBUSͨѶ���ݸ�ʽ
};

static INT8U err;

RTU_ReqBlock_t Init_Freq_Channel = //RTU���������-��Ƶ�ʸ���ͨ��1ѡ��
{
    LIST_HEAD_INIT(Init_Freq_Channel.Entry),
    1,                                          //ִ�д�����0-���޴�
    UART_CHN_CONVERT_FREQ,                      //ִ��ͨ��
	1,                                          //�ӽڵ�վ��ַ
	FUNC_WR_SGREG,                              //������
	EXCUTE_SUCCESS,                             //ִ�н��
	0x0001,                                     //�����Ĵ�����ַ
	0x01,                                       //�����Ĵ�������
	(u8*)&InvertorData[Freq_Channel*2]          //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};

RTU_ReqBlock_t Init_Protocol_Select = //RTU���������-ϵͳͨѶЭ��ѡ��
{
    LIST_HEAD_INIT(Init_Protocol_Select.Entry),
    1,                                          //ִ�д�����0-���޴�
    UART_CHN_CONVERT_FREQ,                      //ִ��ͨ��
	1,                                          //�ӽڵ�վ��ַ
	FUNC_WR_SGREG,                              //������
	EXCUTE_SUCCESS,                             //ִ�н��
	0x0027,                                     //�����Ĵ�����ַ
	0x01,                                       //�����Ĵ�������
	(u8*)&InvertorData[Protocol_Select*2]       //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};

RTU_ReqBlock_t Init_Modbus_Type = //RTU���������-MODBUSͨѶ���ݸ�ʽ,0���Ǳ�׼��MODBUSЭ�� 1����׼��MODBUSЭ��
{
    LIST_HEAD_INIT(Init_Protocol_Select.Entry),
    1,                                          //ִ�д�����0-���޴�
    UART_CHN_CONVERT_FREQ,                      //ִ��ͨ��
    1,                                          //�ӽڵ�վ��ַ
    FUNC_WR_SGREG,                              //������
    EXCUTE_SUCCESS,                             //ִ�н��
    0x0027,                                     //�����Ĵ�����ַ
    0x01,                                       //�����Ĵ�������
    (u8*)&InvertorData[Modbus_Type*2]       //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};

RTU_ReqBlock_t RTU_Req_Read5000 = //RTU���������-��Ƶ��״̬������ַ
{
    LIST_HEAD_INIT(RTU_Req_Read5000.Entry),
    0,                                          //ִ�д�����0-���޴�
    UART_CHN_CONVERT_FREQ,                      //ִ��ͨ��
	1,                                          //�ӽڵ�վ��ַ
	FUNC_RD_HOLDREG,                            //������
	EXCUTE_SUCCESS,                             //ִ�н��
	0x5000,                                     //�����Ĵ�����ַ
	0x20,                                       //�����Ĵ�������
	(u8*)&InvertorData[off_FreqByComm*2]          //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};

RTU_ReqBlock_t RTU_Req_Read8000 = //RTU���������-��Ƶ�����ϵ�ַ,��Ƶ��ͨѶ�쳣��ַ
{
    LIST_HEAD_INIT(RTU_Req_Read8000.Entry),
    0,                                          //ִ�д�����0-���޴�
    UART_CHN_CONVERT_FREQ,                      //ִ��ͨ��
	1,                                          //�ӽڵ�վ��ַ
	FUNC_RD_HOLDREG,                            //������
	EXCUTE_SUCCESS,                             //ִ�н��
	0x8000,                                     //�����Ĵ�����ַ
	0x02,                                       //�����Ĵ�������
	(u8*)&InvertorData[off_InvertorError*2]       //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};


struct RTU_ReqBlock RTU_Req_Write6000= //RTU���������-�������� 1-��ת 2-��ת 3-��ת�㶯 4-��ת�㶯 5-����ͣ�� 6-����ͣ�� 7-���ϸ�λ
{
	LIST_HEAD_INIT(RTU_Req_Write6000.Entry),
    1,                                          //ִ�д�����0-���޴�
	UART_CHN_CONVERT_FREQ,                      //ִ��ͨ��
	1,                                          //�ӽڵ�վ��ַ
	FUNC_WR_SGREG,                              //������06
	EXCUTE_SUCCESS,                             //ִ�н��
	0x6000,                                     //�����Ĵ�����ַ
	0x01,                                       //�����Ĵ�������
	(u8*)&Wrdata[Control_CMD*2]                   //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};

struct RTU_ReqBlock RTU_Req_WriteFreq= //RTU���������
{
	LIST_HEAD_INIT(RTU_Req_WriteFreq.Entry),
    1,                                          //ִ�д�����0-���޴�
	UART_CHN_CONVERT_FREQ,                      //ִ��ͨ��
	1,                                          //�ӽڵ�վ��ַ
	FUNC_WR_SGREG,                              //������06
	EXCUTE_SUCCESS,                             //ִ�н��
	0x0002,                                     //�����Ĵ�����ַ
	0x01,                                       //�����Ĵ�������
	(u8*)&Wrdata[Convert_Freq*2]                  //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};



/********************************************************************************/
/*��������  Freq_Convert_Init                                                   */
/*����˵����ģ���ʼ������                                                       */
/*�����������                                                                  */
/*�����������                                                                  */
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
/*��������  TaskFreq_Timer1ms                                                   */
/*����˵����1ms��ʱ����                                                       */
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

/********************************************************************************/
/*��������  Motor_Forward                                                   */
/*����˵���������ת                                                         */
/*���������speed-Ƶ��  position-λ��                                                */
/*�����������                                                                  */
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
/*��������  Motor_Backward                                                   */
/*����˵���������ת                                                         */
/*���������speed-Ƶ��  position-λ��                                                */
/*�����������                                                                  */
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
        Delta_Position = Global_Variable.Suspende_Target_Position - Global_Variable.Wire_Position;
        
        if((Delta_Position < 10.0)&&(Delta_Position > -10.0))
        {//�жϵ�ǰλ���Ƿ񵽣����ô˷�ʽ���Դ����£������10mm����Ϊ�Ѿ���Ŀ��λ�ã������پ�ȷ����
            Wrdata[Control_CMD] = 6;
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteFreq);
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_Write6000);
            BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
        }
        
        if(CMD_Rope_Wire == ON)//����������������λ����λ�ã����ǵ��˵����λ�ã�
        {
            if(BAND_TYPE_BRAKE_OUT == OFF)
            {
                MOTOR_RUN_DELAY = ON;
                BAND_TYPE_BRAKE_OUT = ON;//��բ�պ�
            }
            if(Motor_DelayTime >= 1000)
            {
                //Motor_DelayTime = 0;
                MOTOR_RUN_DELAY = OFF;
                CMD_Rope_Wire = OFF;
                Motor_Forward(Global_Variable.Suspende_Target_Speed);
            }
        }
        if(CMD_Suspender_Min == ON)//�������˽����������λ�ã����˵����λ�ã�
        {
            if(BAND_TYPE_BRAKE_OUT == OFF)
            {
                MOTOR_RUN_DELAY = ON;
                BAND_TYPE_BRAKE_OUT = ON;//��բ�պ�
            }
            if(Motor_DelayTime >= 1000)
            {
                //Motor_DelayTime = 0;
                MOTOR_RUN_DELAY = OFF;
                CMD_Suspender_Min = OFF;
                Motor_Backward(Global_Variable.Suspende_Target_Speed);
            }
        }
        if(CMD_Suspender_Emergency_Stop == ON)//�������˼�ͣ
        {
            BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
            CMD_Suspender_Emergency_Stop = OFF;
            
        }
        if(CMD_Suspender_Target == ON)//�����������е��趨��Ŀ��λ��
        {
            if(BAND_TYPE_BRAKE_OUT == OFF)
            {
                MOTOR_RUN_DELAY = ON;
                BAND_TYPE_BRAKE_OUT = ON;//��բ�պ�
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
        if(CMD_ParaDownload_Independent == ON)//����΢���������Ի���������
        {
            CMD_ParaDownload_Independent = OFF;
            
        }
        
        //cnt++;
        if(cnt==10000)
        {//����Ϊ����
            cnt = 0;
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteFreq);
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_Write6000);
        }

        OSTimeDlyHMSM(0, 0, 0, 1);
    }
    
}


#endif


