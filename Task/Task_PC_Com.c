#include "main.h"

#include "Task_PC_Com.h"
#include "Task_IO.h"

#ifdef __TASK_PC_MSG_RECV_H
#define BROADCAST                                0U
#define GET_BIT_STATUS(m,n)     {return (m>>n)&0x01}

OS_STK STK_PC_MSG_UPD[STKSIZE_PC_MSG_UPD];

enum Func_BROADCAST{
    Rope_Wire_B=0,Suspender_Min_B,Suspender_Emergency_Stop_B,Suspender_Target_G,Rope_Wire_G,Suspender_Min_G,Suspender_Emergency_Stop_G,ParaDownload_Common
};

enum Func_Node{
    Polling=0,Rope_Wire_F,Suspender_Min_F,Suspender_Emergency_Stop_F,Suspender_Target_F,ParaDownload_Independent,Read_Common_Para,Read_Independent_Para
};

enum Error_Type{
    Error_Func,Error_Check,Error_Timeout
};


OS_EVENT *mBOX_PC_Message_Send;

typedef struct PC_Message_info
{
    u8 databuf[400];
    u16 datalen;
}PC_Message_t;

static PC_Message_t RespondToPC;
volatile BitStatus 	_Running_Error_Sts[6];//
Polling_Frame_Info Polling_Frame_Respond;

INT8U err;

static void Update_Running_ErrorSts(void);
static void Postive_Responde(u8 func);
static void Task_PC_Message_Update(void *p_arg);

static void PC_COM_Init(void)
{
    
}

void PC_COM_Timer100ms(void)
{
    
}

static void Postive_Responde(u8 func)
{
    u16 CrcCheck;
    RespondToPC.datalen = 4;
    RespondToPC.databuf[0] = Globle_Framework.DIP_SwitchStatus;
    RespondToPC.databuf[1] = func;
    CrcCheck = Get_rtuCrc16(RespondToPC.databuf,RespondToPC.datalen-2);
    RespondToPC.databuf[2] = CrcCheck%256;
    RespondToPC.databuf[3] = CrcCheck>>8;
}

static void Negtive_Responde(u8 err)
{
    u16 CrcCheck;
    RespondToPC.datalen = 5;
    RespondToPC.databuf[0] = Globle_Framework.DIP_SwitchStatus;
    RespondToPC.databuf[1] = 0x80;
    RespondToPC.databuf[2] = err;
    CrcCheck = Get_rtuCrc16(RespondToPC.databuf,RespondToPC.datalen-2);
    RespondToPC.databuf[3] = CrcCheck%256;
    RespondToPC.databuf[4] = CrcCheck>>8;
}


static void Update_Running_ErrorSts(void)
{
    //Polling_Frame_Respond.Suspende_Position = 0x12345678;//���˵�ǰλ��,��λmm
    //Polling_Frame_Respond.Suspende_Running_Status = 0xABCD;//��������״̬
    Suspende_Reset = (Globle_Framework.Digit_InputStatus>>0)&0x01;//0-���˸�λ
    Limit_Up_Signal = (Globle_Framework.Digit_InputStatus>>1)&0x01;//1-����λ�ź�
    Limit_Down_Signal = (Globle_Framework.Digit_InputStatus>>2)&0x01;//2-����λ�ź�
    Limit_Up_SlowDown = (Globle_Framework.Digit_InputStatus>>3)&0x01;//3-����λ�����ź�
    Limit_Down_SlowDown = (Globle_Framework.Digit_InputStatus>>4)&0x01;//4-����λ�����ź�
    Band_Type_Brake = (Globle_Framework.Digit_InputStatus>>5)&0x01;//5-��բ�ź�
    Err_Stop_Signal = (Globle_Framework.Digit_InputStatus>>6)&0x01;//16-��ͣ����
    Err_Summit_Attempt = (Globle_Framework.Digit_InputStatus>>7)&0x01;//17-�嶥����
    Err_Loose_Rope = (Globle_Framework.Digit_InputStatus>>8)&0x01;//18-��������
    Err_Temp_Hign = (Globle_Framework.CurrentEnvTemp>70)?1:0;//19-SLAVE���¹���
    Err_Temp_Low = (Globle_Framework.CurrentEnvTemp<-40)?1:0;//20-SLAVE���¹���
    Err_Voltage_High = (Globle_Framework.Power_5V>6)?1:0;//21-SLAVE��ѹ����
    Err_Voltage_Low = (Globle_Framework.Power_5V<4)?1:0;//22-SLAVE��ѹ����
#if 0
    Inverter_Acc_OverCurrent = 0;//23-��Ƶ�����ٹ���������
    Inverter_Slow_OverCurrent = 0; //24-��Ƶ�����ٹ���������
    Inverter_Const_OverCurrent = 0;//25-��Ƶ�����ٹ���������
    Inverter_Acc_OverVoltage = 0;  //26-��Ƶ�����ٹ���ѹ����
    Inverter_Slow_OverVoltage = 0; //27-��Ƶ�����ٹ���ѹ����
    Inverter_Const_OverVoltage = 0;//28-��Ƶ�����ٹ���ѹ����
    Inverter_OverTemp = 0;       //29-��Ƶ�����ȹ��� 
    Inverter_OverLoad = 0;         //30-��Ƶ�����ع���
    Inverter_Input_LackPhase = 0;  //31-��Ƶ������ȱ�����
    Inverter_Output_LackPhase = 0;//32-��Ƶ�����ȱ�����
    Motor_OverLoad = 0;           //33-������ع���
    Motor_Runing_UnderVoltage = 0; //34-���������Ƿ��ѹ����
    Motor_ShortToGND = 0;       //35-����Եض�·����
    Motor_OverTemp = 0;      //36-������¹���    
    Motor_OverSpeed = 0;   //37-������ٹ���        
    Miss_PID_Respond = 0; //38-PID������ʧ����     
    Suspende_Below_Zero = 0; //39-����λ�õ�����λ����    
#endif

    Band_Type_Brake_Out = Band_Type_Brake;
}

void Package_RespData(u8 *data)
{
    u16 index;
    u16 CrcCheck;
    index = 0;
    RespondToPC.datalen = 17;
    RespondToPC.databuf[index++] = data[0];
    RespondToPC.databuf[index++] = data[1];
    RespondToPC.databuf[index++] = 12u;
#if 0
    Package_Float(Globle_Framework.Power_5V,&RespondToPC.databuf[index]);
    index += 4;
    Package_Float(Globle_Framework.CurrentEnvTemp,&RespondToPC.databuf[index]);
    index += 4;
#endif
    RespondToPC.databuf[index++] = (Polling_Frame_Respond.Suspende_Position>>24)&0xff;
    RespondToPC.databuf[index++] = (Polling_Frame_Respond.Suspende_Position>>16)&0xff;
    RespondToPC.databuf[index++] = (Polling_Frame_Respond.Suspende_Position>>8)&0xff;
    RespondToPC.databuf[index++] = (Polling_Frame_Respond.Suspende_Position)&0xff;
    RespondToPC.databuf[index++] = (Polling_Frame_Respond.Suspende_Running_Status>>8)&0xff;
    RespondToPC.databuf[index++] = (Polling_Frame_Respond.Suspende_Running_Status)&0xff;
    RespondToPC.databuf[index++] = Polling_Frame_Respond.Running_Error_Sts[0].Byte;
    RespondToPC.databuf[index++] = Polling_Frame_Respond.Running_Error_Sts[1].Byte;
    RespondToPC.databuf[index++] = Polling_Frame_Respond.Running_Error_Sts[2].Byte;
    RespondToPC.databuf[index++] = Polling_Frame_Respond.Running_Error_Sts[3].Byte;
    RespondToPC.databuf[index++] = Polling_Frame_Respond.Running_Error_Sts[4].Byte;
    RespondToPC.databuf[index++] = Polling_Frame_Respond.Running_Error_Sts[5].Byte;
    CrcCheck = Get_rtuCrc16(RespondToPC.databuf,index);
    RespondToPC.databuf[index++] = CrcCheck%256;
    RespondToPC.databuf[index++] = CrcCheck>>8;
}

void Broadcast_Frame_Parse(u8 *data, u16 len)
{
    switch(data[1])
    {
        case Rope_Wire_B:
            

        break;
        case Suspender_Min_B:

        break;
        case Suspender_Emergency_Stop_B:

        break;
        case Suspender_Target_G:

        break;
        case Rope_Wire_G:

        break;
        case Suspender_Min_G:

        break;
        case Suspender_Emergency_Stop_G:

        break;
        case ParaDownload_Common:

        break;
        default:

        break;
    }
}

void Node_Frame_Parse(u8 *data, u16 len)
{
    u16 CrcCheck;
    u16 index;
    index = 0;
    if(Globle_Framework.DIP_SwitchStatus == data[0])
    {
        switch(data[1])//������
        {
            case Polling:
                if(len == 4U)
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if((CrcCheck%256 == data[2])&&((CrcCheck>>8) == data[3]))
                    {
                        Package_RespData(data);
                    }
                    else
                    {
                        Negtive_Responde(Error_Check);
                    }
                }
                else
                {
                    Negtive_Responde(Error_Timeout);
                }

            break;
            case Rope_Wire_F:
                Band_Type_Brake_Out = 1;
                Postive_Responde(Rope_Wire_F);
            break;
            case Suspender_Min_F:

            break;
            case Suspender_Emergency_Stop_F:
                // ��ǰ���ڵ��ԣ�����λ��δͨѶ�ɹ�������modbus RTU���е��ԣ���߽���ɾ��
                if(len >= 4U)
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if(((CrcCheck>>8) == data[6])&&(CrcCheck%256 == data[7]))
                    {
                        Package_RespData(data);
                    }
                    else
                    {
                        Negtive_Responde(Error_Check);
                    }
                }
                else
                {
                    Negtive_Responde(Error_Timeout);
                }
            break;
            case Suspender_Target_F:

            break;
            case ParaDownload_Independent:

            break;
            case Read_Common_Para:

            break;
            case Read_Independent_Para:

            break;
            default:
                Negtive_Responde(Error_Func);
            break;
        
        }
    }
    else
    {
        RespondToPC.datalen = 0;
    }
}


void Task_PC_Meg_Analysis(void)
{
    u16 i;
    USARTCAN_Recv_t recvmsg;
    RespondToPC.datalen = 0;
    recvmsg = GET_UsartCAN_Recv_Result(UART_PC_MESSAGE_CHN);
    if(recvmsg.lenth >= 4U )//���ݳ��ȴ���3���ֽڣ���Ϊ���ݸ�����Ч
    {
        if(recvmsg.databuf[0] == BROADCAST)
        {
            Broadcast_Frame_Parse(recvmsg.databuf,recvmsg.lenth);
        }
        else
        {
            Node_Frame_Parse(recvmsg.databuf,recvmsg.lenth);
        }
    }
    else
    {
        Negtive_Responde(Error_Timeout);
    }
}


void Task_PC_Message_Recv(void *p_arg)
{
    
    //struct wiz_NetInfo_t *ethparm;
    u8 *recvmsg;
    //ethparm = (struct wiz_NetInfo_t *)p_arg;
    OSTaskCreate(Task_PC_Message_Update, (void *)p_arg, (OS_STK*)&STK_PC_MSG_UPD[STKSIZE_PC_MSG_UPD-1], TASK_PRIO_PC_MSG_UPD);
    
    PC_COM_Init();
    
    while (1)
    {
        recvmsg = (u8*)OSMboxPend(mBOX_Uart_Recv[UART_PC_MESSAGE_CHN],0,&err);
        if(recvmsg[0] == ON)
        {
            Task_PC_Meg_Analysis();
        }

        if(RespondToPC.datalen > 0)
        {
            UartOpFunc[UART_PC_MESSAGE_CHN]._send(RespondToPC.databuf,RespondToPC.datalen);
        }
        //memset(USARTCAN_Recv[UART_PC_MESSAGE_CHN].databuf,0,sizeof(USARTCAN_Recv[UART_PC_MESSAGE_CHN].databuf));
        //OSMboxPost(mBOX_PC_Message_Send,(void *)&RespondToPC);//��������
        //OSTimeDlyHMSM(0, 0, 0, 1);
    }
    
}

void Task_PC_Message_Update(void *p_arg)
{
    mBOX_PC_Message_Send = OSMboxCreate((void *)0);
    PC_Message_t *sedmsg;
    while (1)
    {
        Update_Running_ErrorSts();
        OSTimeDlyHMSM(0, 0, 0, 50);//50ms����һ������
    }
    
}


#endif


