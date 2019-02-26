#include "main.h"

#include "Task_PC_Com.h"
#include "Task_IO.h"

#ifdef __TASK_PC_MSG_RECV_H

enum Frame_Type{
    BROADCAST=0,NODE_FRAME
};


#define GET_BIT_STATUS(m,n)     {return (m>>n)&0x01}


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

static u8 PC_message[400];
static PC_Message_t RespondToPC;
volatile digitstatus    	_Running_Error_Sts[4];//

INT8U err;


static void Postive_Responde(u8 func)
{
    u16 CrcCheck;
    RespondToPC.datalen = 4;
    RespondToPC.databuf[0] = Globle_Framework.DIP_SwitchStatus;
    RespondToPC.databuf[1] = func;
    CrcCheck = Get_Crc16(RespondToPC.databuf,RespondToPC.datalen-2);
    RespondToPC.databuf[2] = CrcCheck%256;
    RespondToPC.databuf[3] = CrcCheck>>8;
}

static void Negtive_Responde(u8 err)
{
    u16 CrcCheck;
    RespondToPC.datalen = 4;
    RespondToPC.databuf[0] = Globle_Framework.DIP_SwitchStatus;
    RespondToPC.databuf[1] = 0x80;
    RespondToPC.databuf[2] = err;
    CrcCheck = Get_Crc16(RespondToPC.databuf,RespondToPC.datalen-2);
    RespondToPC.databuf[3] = CrcCheck%256;
    RespondToPC.databuf[4] = CrcCheck>>8;
}


void Update_InputSts(void)
{
    Suspende_Reset = (Globle_Framework.Digit_InputStatus>>0)&0x01;
    Limit_Up_Signal = (Globle_Framework.Digit_InputStatus>>1)&0x01;
    Limit_Down_Signal = (Globle_Framework.Digit_InputStatus>>2)&0x01;
    Limit_Up_SlowDown = (Globle_Framework.Digit_InputStatus>>3)&0x01;
    Limit_Down_SlowDown = (Globle_Framework.Digit_InputStatus>>4)&0x01;
    Band_Type_Brake = (Globle_Framework.Digit_InputStatus>>5)&0x01;
    Summit_Attempt = (Globle_Framework.Digit_InputStatus>>6)&0x01;
    Loose_Rope = (Globle_Framework.Digit_InputStatus>>7)&0x01;
    Stop_Signal = (Globle_Framework.Digit_InputStatus>>8)&0x01;
    Temp_Hign = (Globle_Framework.CurrentEnvTemp>70)?1:0;
    Temp_Low = (Globle_Framework.CurrentEnvTemp<-40)?1:0;
    Voltage_High = (Globle_Framework.Power_5V>6)?1:0;
    Voltage_Low = (Globle_Framework.Power_5V<4)?1:0;
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
    volatile u8 aaa[4];
    if(Globle_Framework.DIP_SwitchStatus == data[0])
    {
        switch(data[1])
        {
            case Polling:
                if(len == 4U)
                {
                    CrcCheck = Get_Crc16(data,len-2);
                    if(((CrcCheck>>8) == data[3])&&(CrcCheck%256 == data[2]))
                    {
                        RespondToPC.datalen = 17;
                        RespondToPC.databuf[index++] = data[0];
                        RespondToPC.databuf[index++] = data[1];
                        RespondToPC.databuf[index++] = 12u;
                        Package_Float(Globle_Framework.Power_5V,&RespondToPC.databuf[index]);
                        index += 4;
                        Package_Float(Globle_Framework.CurrentEnvTemp,&RespondToPC.databuf[index]);
                        index += 4;
                        RespondToPC.databuf[index++] = _Running_Error_Sts[0].bytetype;
                        RespondToPC.databuf[index++] = _Running_Error_Sts[1].bytetype;
                        RespondToPC.databuf[index++] = _Running_Error_Sts[2].bytetype;
                        RespondToPC.databuf[index++] = _Running_Error_Sts[3].bytetype;
                        CrcCheck = Get_Crc16(RespondToPC.databuf,index);
                        RespondToPC.databuf[index++] = CrcCheck%256;
                        RespondToPC.databuf[index++] = CrcCheck>>8;
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
                // 当前用于调试，若上位机未铜须成功，启用modbus RTU进行调试
                if(len >= 4U)
                {
                    CrcCheck = Get_Crc16(data,len-2);
                    if(((CrcCheck>>8) == data[6])&&(CrcCheck%256 == data[7]))
                    {
                        RespondToPC.datalen = 17;
                        RespondToPC.databuf[index++] = data[0];
                        RespondToPC.databuf[index++] = 3;//data[1];
                        RespondToPC.databuf[index++] = 12u;
                        Package_Float(Globle_Framework.Power_5V,&RespondToPC.databuf[index]);
                        index += 4;
                        Package_Float(Globle_Framework.CurrentEnvTemp,&RespondToPC.databuf[index]);
                        index += 4;
                        RespondToPC.databuf[index++] = _Running_Error_Sts[0].bytetype;
                        RespondToPC.databuf[index++] = _Running_Error_Sts[1].bytetype;
                        RespondToPC.databuf[index++] = _Running_Error_Sts[2].bytetype;
                        RespondToPC.databuf[index++] = _Running_Error_Sts[3].bytetype;
                        CrcCheck = Get_Crc16(RespondToPC.databuf,index);
                        RespondToPC.databuf[index++] = CrcCheck>>8;
                        RespondToPC.databuf[index++] = CrcCheck%256;
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
    recvmsg = GET_UsartCAN_Recv_Result(UART_PC_MESSAGE_CHN);
    Update_InputSts();
    if(recvmsg.lenth >= 4U )//数据长度大于3个字节，认为数据个数有效
    {
        if(recvmsg.databuf[0] == BROADCAST)
        {
            Broadcast_Frame_Parse(recvmsg.databuf,recvmsg.lenth);
        }
        else if(recvmsg.databuf[0] == NODE_FRAME)
        {
            Node_Frame_Parse(recvmsg.databuf,recvmsg.lenth);
        }
        else
        {

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
   
    mBOX_Uart_Recv[UART_PC_MESSAGE_CHN] = OSMboxCreate((void *)0);
    
    while (1)
    {
        recvmsg = (u8*)OSMboxPend(mBOX_Uart_Recv[UART_PC_MESSAGE_CHN],0,&err);
        if(recvmsg[0] == ON)
        {
            Task_PC_Meg_Analysis();
        }
        //memset(USARTCAN_Recv[UART_PC_MESSAGE_CHN].databuf,0,sizeof(USARTCAN_Recv[UART_PC_MESSAGE_CHN].databuf));
        Uart_Transmit(UART_PC_MESSAGE_CHN,RespondToPC.databuf,RespondToPC.datalen);
        //OSMboxPost(mBOX_PC_Message_Send,(void *)&RespondToPC);//启动发送
        //OSTimeDlyHMSM(0, 0, 0, 1);
    }
    
}

void Task_PC_Message_Send(void *p_arg)
{
    mBOX_PC_Message_Send = OSMboxCreate((void *)0);
    PC_Message_t *sedmsg;
    while (1)
    {
        sedmsg=(PC_Message_t*)OSMboxPend(mBOX_PC_Message_Send,0,&err);
        if(OS_ERR_NONE == err)
        {
            Uart_Transmit(UART_PC_MESSAGE_CHN,sedmsg->databuf,sedmsg->datalen);
        }
		else
		{
			OSTimeDlyHMSM(0, 0, 0, 1);
		}
    }
    
}


#endif


