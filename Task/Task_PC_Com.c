#include "main.h"

#include "Task_PC_Com.h"
#include "Task_Freq_Convert.h"
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
    Error_Func,Error_Check,Error_Timeout,Error_Other
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

/********************************************************************************/
/*��������  PC_COM_Init                                                         */
/*����˵������ʼ������                                                           */
/*�����������                                                                  */
/*�����������                                                                  */
/*******************************************************************************/
static void PC_COM_Init(void)
{
    memset(&RespondToPC,0,sizeof(RespondToPC));
    memset((u8 *)&_Running_Error_Sts[0],0,sizeof(_Running_Error_Sts));
    memset(&Polling_Frame_Respond,0,sizeof(Polling_Frame_Respond));
}

/********************************************************************************/
/*��������  Postive_Responde                                                    */
/*����˵�����϶���Ӧ���ݴ��                                                     */
/*���������func����ǰ��������                                                 */
/*�����������                                                                  */
/*******************************************************************************/
static void Postive_Responde(u8 func)
{
    u16 CrcCheck;
    RespondToPC.datalen = 4;
    RespondToPC.databuf[0] = Global_Variable.DIP_SwitchStatus;
    RespondToPC.databuf[1] = func;
    CrcCheck = Get_rtuCrc16(RespondToPC.databuf,RespondToPC.datalen-2);
    RespondToPC.databuf[2] = CrcCheck%256;
    RespondToPC.databuf[3] = CrcCheck>>8;
}

/********************************************************************************/
/*��������  Negtive_Responde                                                    */
/*����˵��������Ӧ���ݴ��                                                     */
/*���������err����ǰ��������                                                    */
/*�����������                                                                  */
/*******************************************************************************/
static void Negtive_Responde(u8 err)
{
    u16 CrcCheck;
    Error_Indicator(30);//LED��30ms����
    RespondToPC.datalen = 5;
    RespondToPC.databuf[0] = Global_Variable.DIP_SwitchStatus;
    RespondToPC.databuf[1] = 0x80;
    RespondToPC.databuf[2] = err;
    CrcCheck = Get_rtuCrc16(RespondToPC.databuf,RespondToPC.datalen-2);
    RespondToPC.databuf[3] = CrcCheck%256;
    RespondToPC.databuf[4] = CrcCheck>>8;
}

/********************************************************************************/
/*��������  Update_Running_ErrorSts                                             */
/*����˵�������ڸ��²�������״̬����                                              */
/*�����������                                                                  */
/*�����������                                                                  */
/*******************************************************************************/
static void Update_Running_ErrorSts(void)
{
    Polling_Frame_Respond.Suspende_Position = Global_Variable.Suspende_Current_Position;//���˵�ǰλ��,��λmm
    Polling_Frame_Respond.Suspende_Running_Status = Global_Variable.Suspende_Current_Speed;//��������״̬
    Suspende_Reset = (Global_Variable.Digit_InputStatus>>0)&0x01;//0-���˸�λ
    Limit_Up_Signal = (Global_Variable.Digit_InputStatus>>1)&0x01;//1-����λ�ź�
    Limit_Down_Signal = (Global_Variable.Digit_InputStatus>>2)&0x01;//2-����λ�ź�
    Limit_Up_SlowDown = (Global_Variable.Digit_InputStatus>>3)&0x01;//3-����λ�����ź�
    Limit_Down_SlowDown = (Global_Variable.Digit_InputStatus>>4)&0x01;//4-����λ�����ź�
    Band_Type_Brake = (Global_Variable.Digit_InputStatus>>5)&0x01;//5-��բ�ź�
    Err_Stop_Signal = (Global_Variable.Digit_InputStatus>>6)&0x01;//16-��ͣ����
    Err_Summit_Attempt = (Global_Variable.Digit_InputStatus>>7)&0x01;//17-�嶥����
    Err_Loose_Rope = (Global_Variable.Digit_InputStatus>>8)&0x01;//18-��������
    Err_Temp_Hign = (Global_Variable.CurrentEnvTemp>70)?1:0;//19-SLAVE���¹���
    Err_Temp_Low = (Global_Variable.CurrentEnvTemp<-40)?1:0;//20-SLAVE���¹���
    Err_Voltage_High = (Global_Variable.Power_5V>6)?1:0;//21-SLAVE��ѹ����
    Err_Voltage_Low = (Global_Variable.Power_5V<4)?1:0;//22-SLAVE��ѹ����
    Inverter_Acc_OverCurrent = InvertorData[off_InvertorError]==0x01 ? 1u:0u;//23-��Ƶ�����ٹ���������
    Inverter_Slow_OverCurrent = InvertorData[off_InvertorError]==0x02 ? 1u:0u; //24-��Ƶ�����ٹ���������
    Inverter_Const_OverCurrent = InvertorData[off_InvertorError]==0x03 ? 1u:0u;//25-��Ƶ�����ٹ���������
    Inverter_Acc_OverVoltage = InvertorData[off_InvertorError]==0x04 ? 1u:0u;  //26-��Ƶ�����ٹ���ѹ����
    Inverter_Slow_OverVoltage = InvertorData[off_InvertorError]==0x05 ? 1u:0u; //27-��Ƶ�����ٹ���ѹ����
    Inverter_Const_OverVoltage = InvertorData[off_InvertorError]==0x06 ? 1u:0u;//28-��Ƶ�����ٹ���ѹ����
    Inverter_OverTemp = InvertorData[off_InvertorError]==0x08 ? 1u:0u;       //29-��Ƶ�����ȹ��� 
    Inverter_OverLoad = InvertorData[off_InvertorError]==0x09 ? 1u:0u;         //30-��Ƶ�����ع���
    Inverter_Input_LackPhase = InvertorData[off_InvertorError]==0x13 ? 1u:0u;  //31-��Ƶ������ȱ�����
    Inverter_Output_LackPhase = InvertorData[off_InvertorError]==0x0C ? 1u:0u;//32-��Ƶ�����ȱ�����
    Motor_OverLoad = InvertorData[off_InvertorError]==0x0A ? 1u:0u;           //33-������ع���
    Motor_Runing_UnderVoltage = InvertorData[off_InvertorError]==0x0B ? 1u:0u; //34-���������Ƿ��ѹ����
    Motor_ShortToGND = InvertorData[off_InvertorError]==0x14 ? 1u:0u;       //35-����Եض�·����
    Motor_OverTemp = InvertorData[off_InvertorError]==0x1A ? 1u:0u;      //36-������¹���    
    Motor_OverSpeed = InvertorData[off_InvertorError]==0x1C ? 1u:0u;   //37-������ٹ���        
    Miss_PID_Respond = InvertorData[off_InvertorError]==0x1E ? 1u:0u; //38-PID������ʧ����     
    Suspende_Below_Zero = InvertorData[off_InvertorError]==0x01 ? 1u:0u; //39-����λ�õ�����λ����    

    //Band_Type_Brake_Out = Band_Type_Brake;
}

/********************************************************************************/
/*��������  Package_RespData                                                     */
/*����˵������ѯ֡��Ӧ���ݴ��                                                    */
/*���������data��֡ǰ�����ֽ�                                                    */
/*�����������                                                                   */
/********************************************************************************/
void Package_RespData(u8 *data)
{
    u16 index;
    u16 CrcCheck;
    index = 0;
    RespondToPC.datalen = 15u;
    RespondToPC.databuf[index++] = data[0];
    RespondToPC.databuf[index++] = data[1];
    RespondToPC.databuf[index++] = RespondToPC.datalen-5;
#if 0
    Package_Float(Globle_Framework.Power_5V,&RespondToPC.databuf[index]);
    index += 4;
    Package_Float(Globle_Framework.CurrentEnvTemp,&RespondToPC.databuf[index]);
    index += 4;
#endif
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

/********************************************************************************/
/*��������  Parameter_Download                                                    */
/*����˵������������                                                     */
/*���������data�����������׵�ַ                                                    */
/*�����������                                                                  */
/*******************************************************************************/
static void Parameter_Download(u8 *data,u8 len)
{
    u16 index=0;
    Global_Variable.Para_Independence.Suspende_Type = data[index++];
    Global_Variable.Para_Independence.Convert_Cfg = data[index++];
    UnPackage_Float(&data[index],&Global_Variable.Para_Independence.Motor_Freq_Factor);
    index += 4;
    UnPackage_Float(&data[index],&Global_Variable.Para_Independence.Lenth_Per_Pulse);
    index += 4;
    Global_Variable.Para_Independence.Max_Motro_Freq = (data[index++]*256);
    Global_Variable.Para_Independence.Max_Motro_Freq += data[index++];
    Global_Variable.Para_Independence.Suspende_Limit_Up = (data[index++]*256);
    Global_Variable.Para_Independence.Suspende_Limit_Up += data[index++];
    Global_Variable.Para_Independence.Reduce_Limit_Up = (data[index++]*256);
    Global_Variable.Para_Independence.Reduce_Limit_Up += data[index++];
    Global_Variable.Para_Independence.Reduce_Limit_Down = (data[index++]*256);
    Global_Variable.Para_Independence.Reduce_Limit_Down += data[index++];
    
}

/********************************************************************************/
/*��������  Parameter_Read                                                    */
/*����˵������������                                                     */
/*���������data�����������׵�ַ                                                    */
/*�����������                                                                  */
/*******************************************************************************/
static void Parameter_Read(void)
{
    u16 CrcCheck;
    u16 index=0;
    RespondToPC.datalen = 5 + sizeof(Global_Variable.Para_Independence);  
    memset(RespondToPC.databuf,0,sizeof(RespondToPC.databuf));
    RespondToPC.databuf[index++] = Global_Variable.DIP_SwitchStatus;
    RespondToPC.databuf[index++] = Read_Common_Para;   
    RespondToPC.databuf[index++] = sizeof(Global_Variable.Para_Independence);
    RespondToPC.databuf[index++] = Global_Variable.Para_Independence.Suspende_Type;
    RespondToPC.databuf[index++] = Global_Variable.Para_Independence.Convert_Cfg;
    Package_Float(Global_Variable.Para_Independence.Motor_Freq_Factor,&RespondToPC.databuf[index]);
    index += 4;
    Package_Float(Global_Variable.Para_Independence.Lenth_Per_Pulse,&RespondToPC.databuf[index]);
    index += 4;
    RespondToPC.databuf[index++] = Global_Variable.Para_Independence.Max_Motro_Freq>>8;
    RespondToPC.databuf[index++] = Global_Variable.Para_Independence.Max_Motro_Freq&0xff;
    RespondToPC.databuf[index++] = Global_Variable.Para_Independence.Suspende_Limit_Up>>8;
    RespondToPC.databuf[index++] = Global_Variable.Para_Independence.Suspende_Limit_Up&0xff;
    RespondToPC.databuf[index++] = Global_Variable.Para_Independence.Reduce_Limit_Up>>8;
    RespondToPC.databuf[index++] = Global_Variable.Para_Independence.Reduce_Limit_Up&0xff;
    RespondToPC.databuf[index++] = Global_Variable.Para_Independence.Reduce_Limit_Down>>8;
    RespondToPC.databuf[index++] = Global_Variable.Para_Independence.Reduce_Limit_Down&0xff;
    CrcCheck = Get_rtuCrc16(RespondToPC.databuf,RespondToPC.datalen-2);
    RespondToPC.databuf[RespondToPC.datalen-2] = CrcCheck%256;
    RespondToPC.databuf[RespondToPC.datalen-1] = CrcCheck>>8;
}


/********************************************************************************/
/*��������  Broadcast_Frame_Parse                                                */
/*����˵�����㲥֡���ݽ���                                                        */
/*���������data���������ݻ�������len,�������ݳ���                                 */
/*�����������                                                                   */
/********************************************************************************/
void Broadcast_Frame_Parse(u8 *data, u16 len)
{
    u16 CrcCheck;
    u16 index=0;
    switch(data[1])
    {
        case Rope_Wire_B://ȫ����������������֡00
            if(len == data[2] + 5)
            {
                CrcCheck = Get_rtuCrc16(data,len-2);
                if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                {
                    Global_Variable.Suspende_Target_Speed = (data[3]<<8)+data[4];
                    CMD_Rope_Wire = ON;
                }
            }
            break;
        case Suspender_Min_B://ȫ�����˽������λ������01
            if(len == data[2] + 5)
            {
                CrcCheck = Get_rtuCrc16(data,len-2);
                if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                {
                    Global_Variable.Suspende_Target_Speed = (data[3]<<8)+data[4];
                    CMD_Suspender_Min = ON;
                }
            }
            break;
        case Suspender_Emergency_Stop_B://ȫ�����˼�ͣ02
            if(len == 4)
            {
                CrcCheck = Get_rtuCrc16(data,len-2);
                if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                {
                    Global_Variable.Suspende_Target_Speed = (data[3]<<8)+data[4];
                    CMD_Suspender_Emergency_Stop = ON;
                }
            }
            break;
        case Suspender_Target_G://��㲥���� 0x03:���Աȫ���������е�Ŀ������λ��03
            if(len == (data[2]<<8) + data[3] + 6)
            {
                for(index=0;index<data[4];index++)
                {
                    if(Global_Variable.DIP_SwitchStatus == data[5+index])//���Ƿ��б��ڵ�����
                    {
                        break;
                    }
                }
                if(index < data[4])
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                    {
                        index = (5+ data[4]) + index * 4;
                        Global_Variable.Suspende_Target_Position = (data[index++]<<8);
                        Global_Variable.Suspende_Target_Position += data[index++];
                        Global_Variable.Suspende_Target_Speed = (data[index++]<<8);
                        Global_Variable.Suspende_Target_Speed += data[index++];
                        CMD_Suspender_Target = ON;
                    }
                }
            }
            break;
        case Rope_Wire_G://��㲥���� 0x04:���Ա����ȫ����λ04
            if(len == (data[2]<<8) + data[3] + 6)
            {
                for(index=0;index<data[4];index++)
                {
                    if(Global_Variable.DIP_SwitchStatus == data[5+index])//���Ƿ��б��ڵ�����
                    {
                        break;
                    }
                }
                if(index < data[4])
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                    {
                        index = (5+ data[4]) + index * 2;
                        Global_Variable.Suspende_Target_Speed = (data[index++]<<8);
                        Global_Variable.Suspende_Target_Speed += data[index++];
                        CMD_Rope_Wire = ON;
                    }
                }
            }
            break;
        case Suspender_Min_G://��㲥���� 0x05:���Ա���˷��µ��������λ��05
            if(len == (data[2]<<8) + data[3] + 6)
            {
                for(index=0;index<data[4];index++)
                {
                    if(Global_Variable.DIP_SwitchStatus == data[5+index])//���Ƿ��б��ڵ�����
                    {
                        break;
                    }
                }
                if(index < data[4])
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                    {
                        index = (5+ data[4]) + index * 2;
                        Global_Variable.Suspende_Target_Speed = (data[index++]<<8);
                        Global_Variable.Suspende_Target_Speed += data[index++];
                        CMD_Suspender_Min = ON;
                    }
                }
            }
            break;
        case Suspender_Emergency_Stop_G://��㲥���� 0x06:���Ա�����ͣ06
            if(len == (data[2]<<8) + data[3] + 6)
            {
                for(index=0;index<data[4];index++)
                {
                    if(Global_Variable.DIP_SwitchStatus == data[5+index])//���Ƿ��б��ڵ�����
                    {
                        break;
                    }
                }
                if(index < data[4])
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                    {
                        CMD_Suspender_Emergency_Stop = ON;
                    }
                }
            }
            break;
        case ParaDownload_Common://΢�������Ĺ��Բ������أ����ݴ�����07
            if(len == (data[2]<<8) + data[3] + 6)
            {
                CrcCheck = Get_rtuCrc16(data,len-2);
                if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                {
                    memcpy(&Global_Variable.Para_Common,&data[3],data[2]);
                    CMD_ParaDownload_Common = ON;
                }
            }
            break;
        default:

        break;
    }
}

/********************************************************************************/
/*��������  Node_Frame_Parse                                                     */
/*����˵������ͨ�ڵ�֡���ݽ���                                                    */
/*���������data���������ݻ�������len,�������ݳ���                                 */
/*�����������                                                                   */
/********************************************************************************/
void Node_Frame_Parse(u8 *data, u16 len)
{
    u16 CrcCheck;

    if(Global_Variable.DIP_SwitchStatus == data[0])
    {
        switch(data[1])//������
        {
            case Polling://����΢�������Ĳ�ѯ����֡
                if(len == 4U)
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
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
                    if(len < 4)
                    {
                        Negtive_Responde(Error_Timeout);
                    }
                    else
                    {
                        Negtive_Responde(Error_Other);
                    }
                }

            break;
            case Rope_Wire_F://����������������λ����λ�ã����ǵ��˵����λ�ã�
                if((len == data[2]+5)&&(data[2] == 2U))
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                    {
                        Global_Variable.Suspende_Target_Position = Global_Variable.Para_Independence.Suspende_Limit_Up;
                        Global_Variable.Suspende_Target_Speed = (data[3]<<8)+data[4];
                        CMD_Rope_Wire = ON;
                        Postive_Responde(Rope_Wire_F);
                    }
                    else
                    {
                        Negtive_Responde(Error_Check);
                    }
                }
                else
                {
                    if(len < data[2]+5)
                    {
                        Negtive_Responde(Error_Timeout);
                    }
                    else
                    {
                        Negtive_Responde(Error_Other);
                    }
                }
                break;
            case Suspender_Min_F://�������˽����������λ�ã����˵����λ�ã�
                if((len == data[2]+5)&&(data[2] == 2U))
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                    {
                        Global_Variable.Suspende_Target_Position = 0;
                        Global_Variable.Suspende_Target_Speed = (data[3]<<8)+data[4];
                        CMD_Suspender_Min = ON;
                        Postive_Responde(Suspender_Min_F);
                    }
                    else
                    {
                        Negtive_Responde(Error_Check);
                    }
                }
                else
                {
                    if(len < data[2]+5)
                    {
                        Negtive_Responde(Error_Timeout);
                    }
                    else
                    {
                        Negtive_Responde(Error_Other);
                    }
                }
                break;
            case Suspender_Emergency_Stop_F://�������˼�ͣ
                if(len == 4U)
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                    {
                        Global_Variable.Suspende_Target_Speed = (data[3]<<8)+data[4];
                        CMD_Suspender_Emergency_Stop = ON;
                        Postive_Responde(Suspender_Emergency_Stop_F);
                    }
                    else
                    {
                        Negtive_Responde(Error_Check);
                    }
                }
                else
                {
                    if(len < 4)
                    {
                        Negtive_Responde(Error_Timeout);
                    }
                    else
                    {
                        Negtive_Responde(Error_Other);
                    }
                }
                break;
            case Suspender_Target_F://�����������е��趨��Ŀ��λ��
                if((len == data[2]+5)&&(data[2] == 4U))
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                    {
                        Global_Variable.Suspende_Target_Position = (data[3]<<8)+data[4];
                        Global_Variable.Suspende_Target_Speed = (data[5]<<8)+data[6];
                        CMD_Suspender_Target = ON;
                        Postive_Responde(Suspender_Target_F);
                    }
                    else
                    {
                        Negtive_Responde(Error_Check);
                    }
                }
                else
                {
                    if(len < data[2]+5)
                    {
                        Negtive_Responde(Error_Timeout);
                    }
                    else
                    {
                        Negtive_Responde(Error_Other);
                    }
                }
                break;
            case ParaDownload_Independent://����΢���������Ի���������
                if(len == data[2] + 5)
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                    {
                        Parameter_Download(&data[3],data[2]);
                        CMD_ParaDownload_Independent = ON;
                        Postive_Responde(ParaDownload_Independent);
                    }
                    else
                    {
                        Negtive_Responde(Error_Check);
                    }
                }
                else
                {
                    if(len < data[2] + 5)
                    {
                        Negtive_Responde(Error_Timeout);
                    }
                    else
                    {
                        Negtive_Responde(Error_Other);
                    }
                }
                break;
            case Read_Common_Para://������΢�������Ĺ��Բ���
                if(len == 4U)
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                    {
                        CMD_Read_Common_Para = ON;
                        //Postive_Responde(Read_Common_Para);//����
                    }
                    else
                    {
                        Negtive_Responde(Error_Check);
                    }
                }
                else
                {
                    if(len < 4u)
                    {
                        Negtive_Responde(Error_Timeout);
                    }
                    else
                    {
                        Negtive_Responde(Error_Other);
                    }
                }
                break;
            case Read_Independent_Para://������΢�������ĸ��Ի�����
                if(len == 4U)
                {
                    CrcCheck = Get_rtuCrc16(data,len-2);
                    if((CrcCheck%256 == data[len-2])&&((CrcCheck>>8) == data[len-1]))
                    {
                        CMD_Read_Independent_Para = ON;
                        Parameter_Read();
                    }
                    else
                    {
                        Negtive_Responde(Error_Check);
                    }
                }
                else
                {
                    if(len < 4)
                    {
                        Negtive_Responde(Error_Timeout);
                    }
                    else
                    {
                        Negtive_Responde(Error_Other);
                    }
                }
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

/********************************************************************************/
/*��������  PC_Msg_Analysis                                                      */
/*����˵����PC������Ϣ����                                                        */
/*�����������                                                                   */
/*�����������                                                                   */
/********************************************************************************/
static void PC_Msg_Analysis(void)
{
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
        if(Global_Variable.DIP_SwitchStatus == recvmsg.databuf[0])
        {
            Negtive_Responde(Error_Timeout);
        }
    }
}

/********************************************************************************/
/*��������  Task_PC_Message_Recv                                                 */
/*����˵����PC������Ϣ��TASK                                                      */
/*�����������                                                                   */
/*�����������                                                                   */
/********************************************************************************/
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
            PC_Msg_Analysis();
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

/********************************************************************************/
/*��������  Task_PC_Message_Update                                               */
/*����˵������Ҫ����λ���������ź�״̬����                                         */
/*�����������                                                                   */
/*�����������                                                                   */
/********************************************************************************/
void Task_PC_Message_Update(void *p_arg)
{
    mBOX_PC_Message_Send = OSMboxCreate((void *)0);
    while (1)
    {
        Update_Running_ErrorSts();
        OSTimeDlyHMSM(0, 0, 0, 50);//50ms����һ������
    }
    
}


#endif


