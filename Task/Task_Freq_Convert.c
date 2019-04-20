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
#define MOTOR_RUNNING                     Motor_Status.Bits.bit0 //������б�־
#define MOTOR_RUN_DELAY                   Motor_Status.Bits.bit1 //���������ʱ�����ڱ�բ
#define MOTOR_DIRECTOR                    Motor_Status.Bits.bit2 //�������з������������½�
#define MOTOR_REDUCING                    Motor_Status.Bits.bit3 //������ٱ�־
#define READ_CURR_FREQ_EN                 Motor_Status.Bits.bit4 //�Ƿ���Ҫ���Ͳ�ѯ��ﵱǰƵ��ֵ�ı�־
#define Reserve_Requrirement              Motor_Status.Bits.bit5 //�����Ҫ�������У��ȼ����ٷ���  
#define FORCE_REDUCE_EN                   Motor_Status.Bits.bit6 //����������λ���ػ��ߵ����Ҫ����ʱ�ô˱�־
#define FORCE_REDUCE_10HZ                 Motor_Status.Bits.bit7 //ǿ�Ƽ���ʱ��Ƶ�ʵ�10HZ�ı�־



#define FREQ_REDUCE_TABLE_NUM             15U

enum Timer_Type{
    Motor_Delay,            //�ɿ���բ�ļ�ʱ���������1-2s���ɿ���ֹͣ����ʱ�ٱ���
    Read8000,               //��ѯ֡��ʱ������
    Read5001,               //��ȡ��ǰ����Ƶ�ʵļ�����
    Freq_Reduce,            //���ټ��ʱ�������
    Keep_10HZ,              //���������λ�����źź�Ƶ�ʼ���10HZʱ��Ҫ����ά�ֵ�ʱ��
    
    Timer_Total
};
static u16 cTimer[Timer_Total];

u16 InvertorData[NUM_Read_Total];
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
//�����ñ���Ҫ����ʶ������ӦƵ�ʵĿ�ʼ���ٵ������������ǵ��ִ�еļ��ٱ�
const Freq_Reduce_t Table_Freq_Reduce[FREQ_REDUCE_TABLE_NUM]=   
{
    {320,30000},
    {240,30000},
    {180,30000},
    {150,20000},
    {120,20000},//������Ƶ��Ϊ120HZ����ʣ��������С��20000ʱ�Ϳ�ʼ����
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
    cTimer[Read8000]++;
    if(cTimer[Read8000] >= READ8000_INTERTER)
    {//���ڶ�ȡ״̬�Ĵ���ֵ
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
                BAND_TYPE_BRAKE_OUT = ON;//��բ�Ͽ�
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
        BAND_TYPE_BRAKE_OUT = OFF;//��բ����
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
/*��������  Calculate_Frequence                                                          */
/*����˵����������������Ƶ��                                                            */
/*�����������                                                                           */       
/*���������1�����ֵ��HZ��Ӧ����ֵ�����Ϊ10000����Ӧ���Ƶ��                                                */
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
        if(cTimer[Read5001] >= 20)
        {//�����Զ�ȡ��ǰ���Ƶ��
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
/*��������  Motor_Forward                                                        */
/*����˵���������ת����                                                          */
/*�����������                                                                   */
/*�����������                                                                  */
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
/*��������  Motor_Backward                                                      */
/*����˵���������ת����                                                         */
/*�����������                                                                  */
/*�����������                                                                  */
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
        if((FORCE_REDUCE_EN == ON)&&(MOTOR_REDUCING == OFF))//ǿ�Ƽ�����ɣ���ǰƵ������СƵ��10HZ
        {//������������ڸ�while�����ϱ��жϣ�����ᵼ�������޷�����
            FORCE_REDUCE_EN = OFF;
            FORCE_REDUCE_10HZ = ON;
            cTimer[Keep_10HZ] = 0;
        }

        if(cTimer[Keep_10HZ] >= FORCE_REDUCE_10HZ_KEEPING)
        {
            if(Reserve_Requrirement == ON)
            {
                cTimer[Motor_Delay] = 0;//���¼�ʱ2s���ɱ�բ
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
        
        if(CMD_Rope_Wire == ON)//����������������λ����λ�ã����ǵ��˵����λ�ã�
        {
            MOTOR_RUNNING = ON;
            CMD_Rope_Wire = OFF;
            Global_Variable.Encode_TargetPulse = 0;
            MotorMove_Rise();
        }
        
        if(CMD_Suspender_Min == ON)//�������˽����������λ�ã����˵����λ�ã�
        {
            MOTOR_RUNNING = ON;
            CMD_Suspender_Min = OFF;
            Global_Variable.Encode_TargetPulse = Global_Variable.Para_Independence.Suspende_Limit_Up/Global_Variable.Para_Independence.Lenth_Per_Pulse;
            MotorMove_Fall();
        }
        
        if(CMD_Suspender_Emergency_Stop == ON)//�������˼�ͣ
        {
            MOTOR_RUNNING = OFF;
            FORCE_REDUCE_EN = OFF;
            MOTOR_REDUCING = OFF;
            CMD_Suspender_Emergency_Stop = OFF;
            Motor_Stop(Motor_Stop_Free);            
        }
        
        if(CMD_Suspender_Target == ON)//�����������е��趨��Ŀ��λ��
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

        if(Err_Summit_Attempt == ON)//�������˼�ͣ
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


