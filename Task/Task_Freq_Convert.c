#include "main.h"
#include "Task_IO.h"
#include "Task_MB_RTU_Master.h"
#include "Task_Freq_Convert.h"
#include "Task_MQTT.h"
#include <stdlib.h>


#ifdef __TASK_FREQ_CONVERT_H

volatile BitStatus Invertor_Status;
volatile BitStatus Motor_Status;
#define MOTOR_RUNNING                     Motor_Status.Bits.bit0 //������б�־
#define MOTOR_RUN_DELAY                   Motor_Status.Bits.bit1 //���������ʱ�����ڱ�բ
#define MOTOR_DIRECTOR                    Motor_Status.Bits.bit2 //�������з�������Ϊ���϶���
#define MOTOR_REDUCING                    Motor_Status.Bits.bit3 //������ٱ�־
#define READ_CURR_FREQ_EN                 Motor_Status.Bits.bit4 //�Ƿ���Ҫ���Ͳ�ѯ��ﵱǰƵ��ֵ�ı�־


#define FREQ_REDUCE_TABLE_NUM             15

u16 InvertorData[NUM_Read_Total];
u16 Motor_DelayTime;//�ɿ���բ�ļ�ʱ���������1-2s���ɿ���ֹͣ����ʱ�ٱ���
u16 Read8000_Timer;//��ѯ֡��ʱ������
u16 Read5001_Timer;//��ȡ��ǰ����Ƶ�ʵļ�����
u16 Freq_Reduce_Timer;//���ټ��ʱ�������
u16 Motor_Freq_MIN;

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

typedef struct Freq_Reduce_Info
{
    u16 reduce_freq;  //��ʼ���ٵ�Ƶ��
    u16 pulse_remain; //ʣ���������
}Freq_Reduce_t;
const Freq_Reduce_t Table_Freq_Reduce[FREQ_REDUCE_TABLE_NUM]=    //���ü��ٱ�
{
    {320,30000},
    {240,30000},
    {180,30000},
    {150,20000},
    {120,10000},//������Ƶ��Ϊ120HZ����ʣ��������С��10000ʱ�Ϳ�ʼ����
    {100,8000},
    {90,7000},
    {80,6000},
    {70,5000},
    {60,5000},
    {50,4000},
    {40,3000},
    {30,1200},
    {20,800},
    {10,500}
};     


u16 WriteData[NUM_Write_Total];//д����������

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
    1,                                          //ִ�д�����0-���޴�
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
	SLAVEID_FREQ,                               //�ӽڵ�վ��ַ
	FUNC_WR_SGREG,                              //������06
	EXCUTE_SUCCESS,                             //ִ�н��
	0x6000,                                     //�����Ĵ�����ַ
	0x01,                                       //�����Ĵ�������
	(u16*)&WriteData[Control_CMD]                  //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};

struct RTU_ReqBlock RTU_Req_WriteFreq_5000= //RTU���������,��������Ƶ��
{
	LIST_HEAD_INIT(RTU_Req_WriteFreq_5000.Entry),
    1,                                          //ִ�д�����0-���޴�
	UART_CHN_CONVERT_FREQ,                      //ִ��ͨ��
	SLAVEID_FREQ,                               //�ӽڵ�վ��ַ
	FUNC_WR_SGREG,                              //������06
	EXCUTE_SUCCESS,                             //ִ�н��
	0x5000,                                     //�����Ĵ�����ַ
	0x01,                                       //�����Ĵ�������
	(u16*)&WriteData[Convert_Freq]                 //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};

struct RTU_ReqBlock RTU_Req_ReadFreq_5001= //RTU���������,��������Ƶ��
{
	LIST_HEAD_INIT(RTU_Req_ReadFreq_5001.Entry),
    1,                                          //ִ�д�����0-���޴�
	UART_CHN_CONVERT_FREQ,                      //ִ��ͨ��
	SLAVEID_FREQ,                               //�ӽڵ�վ��ַ
	FUNC_RD_HOLDREG,                            //������03
	EXCUTE_SUCCESS,                             //ִ�н��
	0x5000,                                     //�����Ĵ�����ַ
	0x01,                                       //�����Ĵ�������
	(u16*)&InvertorData[off_CurrFreq]           //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
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
    Read8000_Timer = 0;
    Global_Variable.Suspende_Target_Position = INIT_POSITION_WIRE;
    Global_Variable.Suspende_Current_Position = INIT_POSITION_WIRE;
    memset(InvertorData,0,sizeof(InvertorData));
    memset(WriteData,0,sizeof(WriteData));
    Motor_Freq_MIN = ((u32)10*10000/Global_Variable.Para_Independence.Max_Motro_Freq);
    
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
        //RTU_AddReqBlock(&rtu_ctx,&Init_Point[i]);
    }
    RTU_AddReqBlock(&rtu_ctx,&RTU_Req_Read8000);//��Ӷ�������Ϣ���󣬺�̨��ʼ�����ж�����
}

/********************************************************************************/
/*��������  TaskFreq_Timer100ms                                                   */
/*����˵����1ms��ʱ����                                                             */
/*�����������                                                                  */
/*�����������                                                                  */
/*******************************************************************************/
void TaskFreq_Timer100ms(void)
{
    Read8000_Timer++;
    if(Read8000_Timer >= READ8000_INTERTER)
    {//���ڶ�ȡ״̬�Ĵ���ֵ
        Read8000_Timer = 0;
        RTU_AddReqBlock(&rtu_ctx,&RTU_Req_Read8000);
    }
    if(MOTOR_RUNNING == ON)
    {
        if(Motor_DelayTime >= BAND_TYPE_BRAKE_DELAY_THRES)
        {
            BAND_TYPE_BRAKE_OUT = ON;//��բ�Ͽ�
        }
        else
        {
            Motor_DelayTime++;
        }
    }
    else
    {
        Motor_DelayTime = 0;
        BAND_TYPE_BRAKE_OUT = OFF;//��բ����
    }

    if(READ_CURR_FREQ_EN == ON)
    {
        Read5001_Timer++;
    }
    else
    {
        Read5001_Timer = 0;
    }

    if(MOTOR_REDUCING == ON)
    {
        Freq_Reduce_Timer++;
    }
    else
    {
        Freq_Reduce_Timer = 0;
    }
}

/****************************************************************************************/
/*��������  Calculate_Frequence                                                          */
/*����˵����������������Ƶ��                                                            */
/*�����������                                                                           */       
/*���������
1��f(Ƶ��)=��50*X���趨�ٶȣ���/[995*(D1(���ٻ�ֱ��)+D2����˿��ֱ����*3.14/���ٱ�)]
*/
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
/*��������  Set_Frequence_Start                                                          */
/*����˵��������������������Ƶ��                                                            */
/*�����������                                                                           */       
/*���������
1��f(Ƶ��)=��50*X���趨�ٶȣ���/[995*(D1(���ٻ�ֱ��)+D2����˿��ֱ����*3.14/���ٱ�)]
*/
/****************************************************************************************/
static void Set_Frequence_Start(void)
{
    u16 motor_freq;
    
    MOTOR_REDUCING = OFF;
    if(MOTOR_DIRECTOR == D_Forward)
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
    
    Global_Variable.Suspende_Current_Speed = (((u32)motor_freq*(u32)Global_Variable.Para_Independence.Max_Motro_Freq)/10000)/
        Global_Variable.Para_Independence.Motor_Freq_Factor;
    //if(Wrdata[Convert_Freq] != Freq_Req)
    {
        WriteData[Convert_Freq] = motor_freq;
        RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteFreq_5000);
    }
}

/****************************************************************************************/
/*��������  Frequence_Reduce_Logic                                                          */
/*����˵�����ж��Ƿ���Ҫ���ٲ����ü���Ƶ��                                                            */
/*�����������                                                                           */       
/*�����������
*/
/****************************************************************************************/
static u16 Frequence_Reduce_Logic(u32 Delta_Pulse)
{
    u8 i;
    float curfreq;
    u16 motor_freq;
    
    if(MOTOR_REDUCING == OFF)
    {
        if(Read5001_Timer >= 20)
        {//�����Զ�ȡ��ǰ���Ƶ��
            Read5001_Timer = 0;
            RTU_AddReqBlock(&rtu_ctx,&RTU_Req_ReadFreq_5001);
            Global_Variable.Suspende_Current_Speed = (InvertorData[off_CurrFreq]/100)/Global_Variable.Para_Independence.Motor_Freq_Factor;//*10000/100
        }
        
        for(i=0;i<FREQ_REDUCE_TABLE_NUM;i++)
        {
            if((InvertorData[off_CurrFreq]/100) >= Table_Freq_Reduce[i].reduce_freq)
            {
                if(Table_Freq_Reduce[i].pulse_remain >= Delta_Pulse)
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
        if(Freq_Reduce_Timer >= FREQ_REDUCE_INTERTER)
        {
            Freq_Reduce_Timer = 0;
            curfreq = Global_Variable.Suspende_Current_Speed * Global_Variable.Para_Independence.Motor_Freq_Factor - FREQ_REDUCE_BASE;
            Global_Variable.Suspende_Current_Speed = curfreq/Global_Variable.Para_Independence.Motor_Freq_Factor;
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
/*��������  Set_Frequence_Running                                                          */
/*����˵���������������е�Ƶ��                                                            */
/*�����������                                                                           */       
/*���������
1��f(Ƶ��)=��50*X���趨�ٶȣ���/[995*(D1(���ٻ�ֱ��)+D2����˿��ֱ����*3.14/���ٱ�)]
*/
/****************************************************************************************/
static void Set_Frequence_Running(u32 Delta_Pulse)
{
    u16 motor_freq;
    motor_freq = Frequence_Reduce_Logic(Delta_Pulse);
    motor_freq = motor_freq<Motor_Freq_MIN?Motor_Freq_MIN:motor_freq;

    if(WriteData[Convert_Freq] > motor_freq)
    {
        WriteData[Convert_Freq] = motor_freq;
        RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteFreq_5000);
    }
}


/********************************************************************************/
/*��������  Motor_Forward                                                        */
/*����˵���������ת����                                                          */
/*�����������                                                                   */
/*�����������                                                                  */
/*******************************************************************************/
void Motor_Forward(void)
{
    MOTOR_DIRECTOR = D_Forward;
    Set_Frequence_Start();
    //if(Wrdata[Control_CMD] != Motor_Fardward_Run)
    {
        WriteData[Control_CMD] = Motor_Fardward_Run;
        RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteCMD_6000);
    }
}

/********************************************************************************/
/*��������  Motor_Backward                                                      */
/*����˵���������ת����                                                         */
/*�����������                                                                  */
/*�����������                                                                  */
/*******************************************************************************/
void Motor_Backward(void)
{
    MOTOR_DIRECTOR = D_Backward;
    Set_Frequence_Start();
    //if(Wrdata[Control_CMD] != Motor_Backward_Run)
    {
        WriteData[Control_CMD] = Motor_Backward_Run;
        RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteCMD_6000);
    }
}

/********************************************************************************/
/*��������  Motor_Stop                                                       */
/*����˵�������ͣ��                                                         */
/*�����������                                                                  */
/*�����������                                                                  */
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
    s32 Delta_Pulse;
    while (1)
    {        
        if(MOTOR_DIRECTOR == D_Forward)
        {
            Delta_Pulse = Global_Variable.Encode_TargetPulse - Global_Variable.Encode_CurrentPulse;
        }
        else
        {
            Delta_Pulse = Global_Variable.Encode_CurrentPulse - Global_Variable.Encode_TargetPulse;
        }
        if(Delta_Pulse < 10u)
        {
            if(MOTOR_RUNNING == ON)
            {
                Motor_DelayTime = 0;
                Motor_Stop(Motor_Stop_Reduce);
                MOTOR_RUNNING = OFF;
            }
        }
        else
        {
            Set_Frequence_Running(Delta_Pulse);
        }
        
        if(CMD_Rope_Wire == ON)//����������������λ����λ�ã����ǵ��˵����λ�ã�
        {
            MOTOR_RUNNING = ON;
            CMD_Rope_Wire = OFF;
            Global_Variable.Encode_TargetPulse = 0;
            Motor_Backward();
        }
        if(CMD_Suspender_Min == ON)//�������˽����������λ�ã����˵����λ�ã�
        {
            MOTOR_RUNNING = ON;
            CMD_Suspender_Min = OFF;
            Global_Variable.Encode_TargetPulse = INIT_POSITION_WIRE/Global_Variable.Para_Independence.Lenth_Per_Pulse;
            Motor_Forward();
        }
        if(CMD_Suspender_Emergency_Stop == ON)//�������˼�ͣ
        {
            MOTOR_RUNNING = OFF;
            CMD_Suspender_Emergency_Stop = OFF;
            Motor_Stop(Motor_Stop_Free);            
        }
        if(CMD_Suspender_Target == ON)//�����������е��趨��Ŀ��λ��
        {
            MOTOR_RUNNING = ON;
            CMD_Suspender_Target = OFF;
            Global_Variable.Encode_TargetPulse = (s32)((INIT_POSITION_WIRE-Global_Variable.Suspende_Target_Position)/ \
                Global_Variable.Para_Independence.Lenth_Per_Pulse);
            if(Global_Variable.Encode_TargetPulse > Global_Variable.Encode_CurrentPulse)
            {
                Motor_Forward();
            }
            else
            {
                Motor_Backward();
            }
        }
        if(CMD_ParaDownload_Independent == ON)//����΢���������Ի���������
        {
            //CMD_ParaDownload_Independent = OFF;
            
        }

        OSTimeDlyHMSM(0, 0, 0, 1);
    }
    
}


#endif


