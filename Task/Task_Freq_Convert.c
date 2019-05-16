#include "main.h"
#include "Task_IO.h"
#include "Task_PC_Com.h"
#include "Task_MB_RTU_Master.h"
#include "Task_Freq_Convert.h"
#include "Task_MQTT.h"
#include <stdlib.h>


#ifdef __TASK_FREQ_CONVERT_H

typedef union {
    u16 dword;
    struct {
        u8  motor_runsts            :2;
        u8  accelerate              :1;
        u8  decelerate              :1;
    	u8  reserve_req             :1;
        u8  force_reduce_en         :1;
    	u8  force_reduce_10hz       :1;
        u8  motor_stop_delay        :1;
        u8  motor_runcmd            :2;
    	u8  limit_measure_sts       :2;
        u8  correct_sts             :2;
        u8  reserve1                :1;
        u8  reserve2                :1;
    }Bits;
}MotorStatus_t;
volatile MotorStatus_t MotorStatus;


volatile BitStatus Invertor_Status[2];
volatile BitStatus Motor_Status[2];
#define MOTOR_RUNNING_STS                 MotorStatus.Bits.motor_runsts         //�������״̬��ֹͣ���������
#define MOTOR_RUNNING_CMD                 MotorStatus.Bits.motor_runcmd         //�����������������У���ʼ�������  
#define MOTOR_ACCELERATE                  MotorStatus.Bits.accelerate           //������ٱ�־
#define MOTOR_REDUCING                    MotorStatus.Bits.decelerate           //������ٱ�־
#define Reserve_Requrirement              MotorStatus.Bits.reserve_req          //�����Ҫ�������У��ȼ����ٷ���  
#define FORCE_REDUCE_EN                   MotorStatus.Bits.force_reduce_en      //����������λ���ػ��ߵ����Ҫ����ʱ�ô˱�־
#define FORCE_REDUCE_10HZ                 MotorStatus.Bits.force_reduce_10hz    //ǿ�Ƽ���ʱ��Ƶ�ʵ�10HZ�ı�־
#define MOTOR_CORRENT_STS                 MotorStatus.Bits.correct_sts          //���λ������״̬��0-�� 1-������ 2-������
#define MOTOR_STOP_DELAY                  MotorStatus.Bits.motor_stop_delay     //�����ͣ���ӳ�
#define MEASURE_LIMIT_STS                 MotorStatus.Bits.limit_measure_sts    //�����г���λ����״̬ 0-���У�1-�����У�2-�����ɹ���3-����ʧ��

#define FREQ_REDUCE_TABLE_NUM             100U

enum Timer_Type{
    Motor_Delay,            //�ɿ���բ�ļ�ʱ���������1-2s���ɿ���ֹͣ����ʱ�ٱ���
    Read8000,               //��ѯ֡��ʱ������
    Read5001,               //��ȡ��ǰ����Ƶ�ʵļ�����
    Freq_Acc_Reduce,        //�Ӽ��ټ��ʱ�������
    Keep_10HZ,              //���������λ�����źź�Ƶ�ʼ���10HZʱ��Ҫ����ά�ֵ�ʱ��
    Motor_Correct,          //���λ���������ȷ�ͣ�������ʱ������ֵ
    Reset_Error_Delay,      //�����ͣ��λ���ϵ���ʱʱ�䣬������Ϊ500ms
    Read5005,               //��ȡ����������
    
    Timer_Total
};
static u16 cTimer[Timer_Total];

u16 InvertorData[NUM_Read_Total];
u16 Motor_Freq_MIN; //���ֵ��HZ��Ӧ����ֵ�����Ϊ10000����Ӧ���Ƶ��
s32 Limit_Position;    //��λ�źŴ�λ��ֵmm

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
    u16 ParaAddr;   //��ʼ�������ļĴ�����ַ
    u16 DataValue;  //��ʼ��������ֵ
}Init_Para_t;

typedef struct Freq_Change_Info
{
    u16 running_freq;  //���е�Ƶ��
    u16 pulse_num;  //ʣ��������
    u16 Keep_Timer;  //����ʱ��
}Freq_Change_t;
//�����ñ���Ҫ����ʶ������ӦƵ�ʵĿ�ʼ���ٵ������������ǵ��ִ�еļ��ٱ�
Freq_Change_t Table_Freq_Change[FREQ_REDUCE_TABLE_NUM]=   
{//  FREQ   LENTH
    {10,   8000,    300},
    {12,   8000,    300},
    {14,   8000,    300},
};     

typedef struct Acc_Reduce_Info
{
    s32 Pulse_StartMove;    //��ʼ���е�������
    u8  Freq_Index;
}Acc_Reduce_t;
Acc_Reduce_t Acc_Reduce_Status;



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
	(u16*)&WriteData[Control_CMD]               //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
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
	(u16*)&WriteData[Convert_Freq]              //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};

struct RTU_ReqBlock RTU_Req_ReadPower_5005= //RTU���������,��ȡ��ǰ������й���
{
	LIST_HEAD_INIT(RTU_Req_ReadPower_5005.Entry),
    1,                                          //ִ�д�����0-���޴�
	UART_CHN_CONVERT_FREQ,                      //ִ��ͨ��
	SLAVEID_FREQ,                               //�ӽڵ�վ��ַ
	FUNC_RD_HOLDREG,                            //������03
	EXCUTE_SUCCESS,                             //ִ�н��
	0x5005,                                     //�����Ĵ�����ַ
	0x01,                                       //�����Ĵ�������
	(u16*)&InvertorData[off_CurrFreqPower]      //ִ�е����ݣ���ȡ�ļĴ������ݻ�д����������
};

static void Motor_Stop(u8 stoptype);


/********************************************************************************/
/*��������  Freq_Convert_Init                                                   */
/*����˵����ģ���ʼ������                                                       */
/*�����������                                                                  */
/*�����������                                                                  */
/*******************************************************************************/
static void Freq_Convert_Init(void)
{
    u8 i;
    Invertor_Status[0].Byte = 0;
    Invertor_Status[1].Byte = 0;
    MEASURE_LIMIT_STS = M_IDLE;
    memset((u8 *)&cTimer[0],0,sizeof(cTimer));
    Global_Variable.Suspende_PositionTarget = Global_Variable.Para_Independence.Suspende_Limit_Up;
    if(Global_Variable.Suspende_PositionMemory != 0xffff)
    {
        Global_Variable.Suspende_PositionCurrent = Global_Variable.Suspende_PositionMemory;
        Global_Variable.Suspende_PulseMemory = (Global_Variable.Para_Independence.Suspende_Limit_Up - 
            Global_Variable.Suspende_PositionMemory) / Global_Variable.Para_Independence.Lenth_Per_Pulse;
        Global_Variable.Encode_PulseCurrent = Global_Variable.Suspende_PulseMemory;
    }
    else
    {
        Global_Variable.Suspende_PositionMemory = 0;
        Global_Variable.Suspende_PositionCurrent = Global_Variable.Para_Independence.Suspende_Limit_Up;
    }
    memset(InvertorData,0,sizeof(InvertorData));
    memset(WriteData,0,sizeof(WriteData));
    memset(&Acc_Reduce_Status,0,sizeof(Acc_Reduce_Status));
    Motor_Freq_MIN = ((u32)10*10000/Global_Variable.Para_Independence.Max_Motro_Freq);

    for(i=0;i<FREQ_REDUCE_TABLE_NUM;i++)
    {
        Table_Freq_Change[i].running_freq = FREQ_STARTMOVE_BASE + i*Global_Variable.Para_Independence.Step_Size_Base;
        Table_Freq_Change[i].pulse_num = Global_Variable.Para_Independence.Distance_10HZ + i*Global_Variable.Para_Independence.Pulze_NumBase;
        Table_Freq_Change[i].Keep_Timer = Global_Variable.Para_Independence.Freq_Change_Timer;
    }
    
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
/*��������  Get_Limit_Measure_Status                                                   */
/*����˵���� ��ȡ��ǰ��ߵ�״̬                                                             */
/*�����������                                                                  */
/*�����������                                                                  */
/*******************************************************************************/
u8 Get_Limit_Measure_Status(void)
{
    return MEASURE_LIMIT_STS;
}


/********************************************************************************/
/*��������  TaskFreq_Timer100ms                                                   */
/*����˵����1ms��ʱ����                                                             */
/*�����������                                                                  */
/*�����������                                                                  */
/*******************************************************************************/
void TaskFreq_Timer1ms(void)
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
    
    if(MOTOR_RUNNING_STS != OFF) //�����������״̬
    {
        cTimer[Read5005] = 0;
        if(cTimer[Motor_Delay] >= BAND_TYPE_BRAKE_DELAY_THRES)
        {//�����ʼ����ʱ300ms����ɱ�բ�ٶϿ�
            //if(MOTOR_REDUCING == OFF)
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
        if(cTimer[Read5005] >= READ_FREQ_POWER_THRES)
        {
            cTimer[Read5005] = 0;
            if(InvertorData[off_CurrFreqPower] != 0)
            {
                Motor_Stop(Motor_Stop_Free); 
            }
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_ReadPower_5005);
        }
        else
        {
            cTimer[Read5005]++;
        }
    }

    if((MOTOR_ACCELERATE == ON)||(MOTOR_REDUCING == ON))
    {//�����ǰ���ڼӼ���״̬
        cTimer[Freq_Acc_Reduce]++;//���ٳ���ʱ���ʱ
        cTimer[Read5001] = 0;
    }
    else
    {
        cTimer[Freq_Acc_Reduce] = 0;
        cTimer[Read5001]++;//��ȡ��ǰ�������Ƶ�ʵ�ʱ���ʱ
    }

    if(FORCE_REDUCE_10HZ == ON)
    {//Ƶ�ʼ���10HZ�ı�־��һ��ָ����ǿ�Ƽ��٣����������ٿ��ػ�����Ҫ����ʱ��������Ŀ��λ�ö����е��������ٲ����ô˱�־
        cTimer[Keep_10HZ]++;//�ü�ʱ������Ƶ�ʼ���10HZʱ����һ��ʱ����õ��ֹͣ����
    }
    else
    {
        cTimer[Keep_10HZ] = 0;
    }
		
    if(MOTOR_CORRENT_STS != Corrent_None)
    {//����������λ����
        cTimer[Motor_Correct]++;//�ü�ʱ������������λ����ʱ�ӳ�һ��ʱ��������������͵���λ��ֵ
    }
    else
    {
        cTimer[Motor_Correct] = 0;
    }

    if(MOTOR_STOP_DELAY == ON)
    {
        cTimer[Reset_Error_Delay]++;
        if(cTimer[Reset_Error_Delay] > RESET_ERROR_DELAY_THRES)
        {
            MOTOR_STOP_DELAY = OFF;
            cTimer[Reset_Error_Delay] = 0;
            WriteData[Control_CMD] = Error_Reset;
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteCMD_6000);
        }
    }
    else
    {
        cTimer[Reset_Error_Delay] = 0;
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
    temp = Global_Variable.Suspende_SpeedTarget*Global_Variable.Para_Independence.Motor_Freq_Factor;
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
    MOTOR_ACCELERATE = ON;
    Acc_Reduce_Status.Freq_Index = 0;
    Acc_Reduce_Status.Pulse_StartMove = Global_Variable.Encode_PulseCurrent;
    motor_freq = Motor_Freq_MIN;
    Global_Variable.Suspende_SpeedCurrent = ((u32)motor_freq*(u32)Global_Variable.Para_Independence.Max_Motro_Freq)/FREQ_MAX_VALUE;
    Global_Variable.Suspende_SpeedCurrent /= Global_Variable.Para_Independence.Motor_Freq_Factor;
    //if(Wrdata[Convert_Freq] != Freq_Req)
    {
        WriteData[Convert_Freq] = motor_freq;
        RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteFreq_5000);
    }
}

/****************************************************************************************/
/*��������  Frequence_Acc_Reduce_Logic                                                      */
/*����˵�����ж��Ƿ���Ҫ�Ӽ��ٲ����üӼ���Ƶ��                                                            */
/*�����������                                                                           */       
/*�����������
*/
/****************************************************************************************/
static u16 Frequence_Acc_Reduce_Logic(u32 Delta_Pulse)
{
    u8 i;
    float curfreq;//HZ
    u32 reduce_pulse=0;
    u16 motor_freq=0;//���ֵ��HZ��Ӧ����ֵ�����Ϊ10000����Ӧ���Ƶ��
    
    if(MOTOR_REDUCING == OFF)
    {
        for(i=0;i<=Acc_Reduce_Status.Freq_Index;i++)
        {
            reduce_pulse += Table_Freq_Change[i].pulse_num;
        }
        if((FORCE_REDUCE_EN == ON)||((reduce_pulse + (Table_Freq_Change[Acc_Reduce_Status.Freq_Index+1].pulse_num)) >= Delta_Pulse))
        {//�����������ʱ��ʼ���٣�����λ
            Global_Variable.Suspende_SpeedCurrent = Table_Freq_Change[Acc_Reduce_Status.Freq_Index].running_freq/Global_Variable.Para_Independence.Motor_Freq_Factor;
            motor_freq = Table_Freq_Change[Acc_Reduce_Status.Freq_Index].running_freq*FREQ_MAX_VALUE/Global_Variable.Para_Independence.Max_Motro_Freq;//
            MOTOR_REDUCING = ON;
            MOTOR_ACCELERATE = OFF;
            cTimer[Freq_Acc_Reduce] = 0;
        }

        if(MOTOR_REDUCING == OFF)
        {
            if(cTimer[Freq_Acc_Reduce] >= Table_Freq_Change[Acc_Reduce_Status.Freq_Index].Keep_Timer)
            {
                cTimer[Freq_Acc_Reduce] = 0;
                if((Global_Variable.Suspende_SpeedTarget > Global_Variable.Suspende_SpeedCurrent)&&
                    ((Global_Variable.Suspende_SpeedTarget - Global_Variable.Suspende_SpeedCurrent) > (Global_Variable.Para_Independence.Step_Size_Base/Global_Variable.Para_Independence.Motor_Freq_Factor)))
                {
                    Acc_Reduce_Status.Freq_Index++;
                    Global_Variable.Suspende_SpeedCurrent = (u16)(Table_Freq_Change[Acc_Reduce_Status.Freq_Index].running_freq/Global_Variable.Para_Independence.Motor_Freq_Factor); 
                    MOTOR_ACCELERATE = ON;
                }
                else
                {
                    MOTOR_ACCELERATE = OFF;
                    Global_Variable.Suspende_SpeedCurrent = Global_Variable.Suspende_SpeedTarget;
                }
                curfreq = Global_Variable.Suspende_SpeedCurrent*Global_Variable.Para_Independence.Motor_Freq_Factor;
                motor_freq = curfreq*FREQ_MAX_VALUE/Global_Variable.Para_Independence.Max_Motro_Freq;//
            }
            else
            {
                motor_freq = WriteData[Convert_Freq];//�����ϴ�ֵ����
            }
        }
        else
        {
            /*do nothing*/
        }
    }
    else
    {
        MOTOR_ACCELERATE = OFF;
        if(cTimer[Freq_Acc_Reduce] >= Table_Freq_Change[Acc_Reduce_Status.Freq_Index].Keep_Timer)
        {//ÿ��һ��ʱ����ٹ̶�Ƶ��
            if(Acc_Reduce_Status.Freq_Index > 0)//Ƶ�ʻ�δ������͵�10HZ
            {
                cTimer[Freq_Acc_Reduce] = 0;
                Acc_Reduce_Status.Freq_Index--;
                Global_Variable.Suspende_SpeedCurrent = (u16)(Table_Freq_Change[Acc_Reduce_Status.Freq_Index].running_freq/Global_Variable.Para_Independence.Motor_Freq_Factor); 
                curfreq = Global_Variable.Suspende_SpeedCurrent * Global_Variable.Para_Independence.Motor_Freq_Factor;//HZ
                motor_freq = curfreq*FREQ_MAX_VALUE/Global_Variable.Para_Independence.Max_Motro_Freq;//
            }
            else
            {
                motor_freq = Motor_Freq_MIN;
                MOTOR_REDUCING = OFF;
            }
        }
        else
        {
            motor_freq = WriteData[Convert_Freq];//�����ϴ�ֵ����
        }
    }
    return motor_freq;//��Ƶ�ʵĵ�λ�Ƿ�����Ƶ����ֵ��0-10000��Ӧ0-���Ƶ��
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
    motor_freq = Frequence_Acc_Reduce_Logic(Delta_Pulse);
    if(motor_freq <= Motor_Freq_MIN)
    {
        MOTOR_REDUCING = OFF;
        motor_freq = Motor_Freq_MIN;
    }
    else if(motor_freq >= FREQ_MAX_VALUE)
    {
        MOTOR_ACCELERATE = OFF;
        motor_freq = FREQ_MAX_VALUE;
    }
    else
    {

    }

    if(WriteData[Convert_Freq] != motor_freq)
    {
        WriteData[Convert_Freq] = motor_freq;
        RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteFreq_5000);
    }
}


/********************************************************************************/
/*��������  MotorMove_Fall                                                        */
/*����˵������������˶�����                                                          */
/*���������distance���о���,��������λ����Ծ���                                                  */
/*�����������                                                                  */
/*******************************************************************************/
void MotorMove_Fall(s32 distance)
{
    MOTOR_RUNNING_STS = DIR_FALL;
    Global_Variable.Encode_PulseTarget = distance/Global_Variable.Para_Independence.Lenth_Per_Pulse;
    if(Global_Variable.Para_Independence.Convert_Cfg == ON)
    {//�б�Ƶ������
        Set_Frequence_Start();
        //if(Wrdata[Control_CMD] != Motor_Fardward_Run)
        {
            WriteData[Control_CMD] = Motor_Fardward_Run;
            RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteCMD_6000);
        }
    }
    else
    {//�ޱ�Ƶ������
        CONTACTOR_RISE_OUT = OFF;
        CONTACTOR_FALL_OUT = ON;
        CONTACTOR_STOP_OUT = OFF;
    }
}

/********************************************************************************/
/*��������  MotorMove_Rise                                                      */
/*����˵������������˶�����                                                         */
/*���������distance���о���,��������λ����Ծ���                                                */
/*�����������                                                                  */
/*******************************************************************************/
void MotorMove_Rise(s32 distance)
{
    MOTOR_RUNNING_STS = DIR_RISE;
    Global_Variable.Encode_PulseTarget = distance/Global_Variable.Para_Independence.Lenth_Per_Pulse;
    if(Global_Variable.Para_Independence.Convert_Cfg == ON)
    {//�б�Ƶ������
        Set_Frequence_Start();
        //if(Wrdata[Control_CMD] != Motor_Backward_Run)
        {
            WriteData[Control_CMD] = Motor_Backward_Run;
            RTU_AddReqBlock(&rtu_ctx,&RTU_Req_WriteCMD_6000);
        }
    }
    else
    {//�ޱ�Ƶ������
        CONTACTOR_RISE_OUT = ON;
        CONTACTOR_FALL_OUT = OFF;
        CONTACTOR_STOP_OUT = OFF;
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
    if(Global_Variable.Para_Independence.Convert_Cfg == ON)
    {//�б�Ƶ������
        MOTOR_RUNNING_STS = Motor_Idle;
        MOTOR_RUNNING_CMD = Motor_Normal;
        FORCE_REDUCE_10HZ = OFF;
        cTimer[Keep_10HZ] = 0;
        FORCE_REDUCE_EN = OFF;
        MOTOR_ACCELERATE = OFF;
        MOTOR_REDUCING = OFF;
        if((stoptype ==Motor_Stop_Reduce)||(stoptype ==Motor_Stop_Free))
        {   
            stopmode = stoptype;
        }
        else
        {
            stopmode = Motor_Stop_Free;
        }

        if(stopmode == Motor_Stop_Free)
        {
            if((RTU_Req_Read8000.Status != EXCUTE_FAIL)&&(InvertorData[off_InvertorError] == 0))
            {//ͣ��ǰ����޹���
                MOTOR_STOP_DELAY = ON;
            }
        }
        else
        {
            //CMD_Download_LocalCfg = ON;//�洢��ǰλ��
        }
        //if(WriteData[Control_CMD] != stopmode)
        {
            WriteData[Control_CMD] = stopmode;
            RTU_AddReqBlock_Front(&rtu_ctx,&RTU_Req_WriteCMD_6000);
        }
    }
    else
    {//�ޱ�Ƶ������
        CONTACTOR_RISE_OUT = OFF;
        CONTACTOR_FALL_OUT = OFF;
        CONTACTOR_STOP_OUT = ON;
    }
    InvertorData[off_CurrFreqPower] = 1;//ͣ��ʱ�����õ�ǰ����Ϊ���㣬�����ٶ�ȡ�Ĵ���ֵ���¸�ֵ
    RTU_AddReqBlock(&rtu_ctx,&RTU_Req_ReadPower_5005);
    //CMD_Download_LocalCfg = ON;//�洢��ǰλ��
}

/********************************************************************************/
/*��������  CMD_Freq_Convert                                                   */
/*����˵������Ƶ���������                                                         */
/*�����������                                                                   */
/*�����������                                                                  */
/*******************************************************************************/
void CMD_Freq_Convert(void)
{
    if(CMD_Suspender_Emergency_Stop == ON)//��λ������������˼�ͣ
    {
        BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
        CMD_Suspender_Emergency_Stop = OFF;
        Motor_Stop(Motor_Stop_Free);            
    }
    else if(CMD_Suspender_Init == ON)
    {//���˳�ʼ������
        CMD_Suspender_Init = OFF;
        if((Global_Variable.Suspende_PositionCurrent + 10) < Global_Variable.Para_Independence.Suspende_Limit_Up)
        {//��ǰλ�ÿ�������λʱ��ִ�г�ʼ������,ֻ�е�ǰλ��������λ���³���10mmʱ��ִ�г�ʼ��
            if(MOTOR_RUNNING_CMD != Motor_Init)
            {
                MOTOR_RUNNING_CMD = Motor_Init;
                MotorMove_Rise(Global_Variable.Suspende_PositionTarget-Global_Variable.Para_Independence.Suspende_Limit_Up);
            }
       }
    }
    else if(CMD_Rope_Wire == ON)//����������������λ����λ�ã����ǵ��˵����λ�ã�
    {
        CMD_Rope_Wire = OFF;
        MotorMove_Rise(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
    }
    else if(CMD_Suspender_Min == ON)//�������˽����������λ�ã����˵����λ�ã�
    {
        CMD_Suspender_Min = OFF;
        MotorMove_Fall(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
    }
    else if(CMD_Suspender_Target == ON)//�����������е��趨��Ŀ��λ��
    {
        CMD_Suspender_Target = OFF;
        if(Global_Variable.Suspende_PositionCurrent > Global_Variable.Suspende_PositionTarget)
        {
            if(MOTOR_RUNNING_STS == DIR_RISE)
            {
                Reserve_Requrirement = ON;
            }
            else
            {
                MotorMove_Fall(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
            }
        }
        else
        {
            if(MOTOR_RUNNING_STS == DIR_FALL)
            {
                Reserve_Requrirement = ON;
            }
            else
            {
                MotorMove_Rise(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
            }
        }
    }
    else if(CMD_Limit_Measure == ON)
    {
        CMD_Limit_Measure = OFF;
        if(MOTOR_RUNNING_CMD != Measure_Distance)
        {
            if((Limit_Fall_Signal == OFF)&&(Global_Variable.Suspende_PositionCurrent>0))
            {
                MOTOR_RUNNING_CMD = Measure_Distance;
                MEASURE_LIMIT_STS = M_MEASURING;
                Global_Variable.Suspende_PositionTarget = -60000;//����������г̣�ֱ����������λ����
                MotorMove_Fall(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
            }
            else
            {
                //��ִ����λ����
            }
        }
    }
    else
    {

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
    s32 Delta_Pulse;
    u8 pre_limit_rise;
    u8 pre_limit_fall;
    u8 pre_Stop_Signal;
    u8 pre_Summit_Attempt;

    Freq_Convert_Init();
    
    while (1U+1U==2U)
    {          
        if(Err_Stop_Signal == ON)//��ͣ��ť�����˼�ͣ
        {
            if(pre_Stop_Signal == OFF)
            {
                BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                Motor_Stop(Motor_Stop_Free);   
            }
        }
        pre_Stop_Signal = Err_Stop_Signal;

        if(Err_Summit_Attempt == ON)//�嶥���ϣ����˼�ͣ
        {
            if(pre_Summit_Attempt == OFF)
            {
                BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                Motor_Stop(Motor_Stop_Free);   
            }
        }
        pre_Summit_Attempt = Err_Summit_Attempt;
    
        if((FORCE_REDUCE_EN == ON)&&(MOTOR_REDUCING == OFF))//ǿ�Ƽ�����ɣ���ǰƵ������СƵ��10HZ
        {//������������ڸ�while�����ϱ��жϣ�����ᵼ�������޷�����
            if(MOTOR_RUNNING_CMD == Motor_Init)
            {//�����Ƿǵ��˳�ʼ��״̬��
                FORCE_REDUCE_EN = OFF;
                FORCE_REDUCE_10HZ = ON;
                cTimer[Keep_10HZ] = 0;
            }
        }

        if(Global_Variable.Para_Independence.Convert_Cfg == ON)
        {//�б�Ƶ������
            if(cTimer[Keep_10HZ] >= FORCE_REDUCE_10HZ_KEEPING)
            {//ǿ�Ƽ��ٵ�10HZ�ұ��̶ֹ�ʱ�����Ҫֹͣ������л��߿�ʼ��������
                FORCE_REDUCE_10HZ = OFF;
                if(Reserve_Requrirement == ON)
                {//��Ҫ��������
                    Reserve_Requrirement = OFF;
                    cTimer[Motor_Delay] = 0;//���¼�ʱ500ms���ɱ�բ
                    if(Global_Variable.Suspende_PositionCurrent > Global_Variable.Suspende_PositionTarget)
                    {
                        MotorMove_Fall(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
                    }
                    else
                    {
                        MotorMove_Rise(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
                    }
                }
                else
                {
                    Motor_Stop(Motor_Stop_Reduce);
                }
            }
        }
        else
        {//�ޱ�Ƶ������
            if(FORCE_REDUCE_10HZ == ON)
            {
                Motor_Stop(Motor_Stop_Reduce);
            }
        }


        if(MOTOR_RUNNING_STS != OFF)//�����������״̬
        {
            if(((Limit_Up_SlowDown == ON)&&(MOTOR_RUNNING_STS == DIR_RISE))||((Limit_Down_SlowDown == ON)&&(MOTOR_RUNNING_STS == DIR_FALL))||
                (Reserve_Requrirement == ON))//���й�����������λ�����ź�
            {
                if(MOTOR_RUNNING_CMD == Motor_Normal)//���˳�ʼ���Ͳ����Ҫ��������λ���ش�
                {
                    FORCE_REDUCE_EN = ON;
                }
            }
            else
            {
                FORCE_REDUCE_EN = OFF;
            }

            
            if(MOTOR_RUNNING_STS == DIR_RISE)
            {//�������
                if(Limit_Rise_Signal == ON)
                {//���й�������������λ�ź�
                    MOTOR_RUNNING_CMD = Motor_Normal;
                    if(BAND_TYPE_BRAKE_OUT == ON)
                    {
                        BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                    }
                    Motor_Stop(Motor_Stop_Reduce);
                    
                    if(pre_limit_rise == OFF)
                    {//��Ҫ����λ�ò���
                        
                        if(MEASURE_LIMIT_STS == M_MEASURING)
                        {
                            MEASURE_LIMIT_STS = M_SUCCESS;
                            Limit_Position = Global_Variable.Suspende_PositionCurrent-Limit_Position;//��¼����λ������
                            Global_Variable.Para_Independence.Suspende_Limit_Up = Limit_Position;
                            CMD_ParaDownload_Independent = ON;//д���Ի�����
                        }
                        else
                        {
                            MOTOR_CORRENT_STS = Corrent_Up; 
                        }
                    }
                }
                else if(Global_Variable.Suspende_PositionCurrent >= Global_Variable.Para_Independence.Suspende_Limit_Up)
                {
                    if((MEASURE_LIMIT_STS != M_MEASURING)&&(MOTOR_RUNNING_CMD != Motor_Init))//���˳�ʼ��ʱ��Ҫ��������λ���ش�
                    {
                        if(BAND_TYPE_BRAKE_OUT == ON)
                        {
                            BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                        }
                        Motor_Stop(Motor_Stop_Reduce);
                    }
                }
                else
                {

                }
            }
            else if(MOTOR_RUNNING_STS == DIR_FALL)
            {//����½�
                if(Limit_Fall_Signal == ON)
                {//���й�������������λ�ź�
                    if(BAND_TYPE_BRAKE_OUT == ON)
                    {
                        BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                    }
                    Motor_Stop(Motor_Stop_Reduce);

                    if(pre_limit_fall == OFF)
                    {//��Ҫ����λ�ò���
                        if(MEASURE_LIMIT_STS == M_MEASURING)
                        {
                            Limit_Position = Global_Variable.Suspende_PositionCurrent;//��¼����λ������
                            Global_Variable.Suspende_PositionTarget = 60000;//����������г̣�ֱ����������λ����
                            MotorMove_Rise(Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget);
                        }
                        else
                        {
                            MOTOR_CORRENT_STS = Corrent_Down; 
                        }
                    }
                }
                else if(Global_Variable.Suspende_PositionCurrent <= 0)
                {
                    if(MEASURE_LIMIT_STS != M_MEASURING)
                    {
                        if(BAND_TYPE_BRAKE_OUT == ON)
                        {
                            BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                        }
                        Motor_Stop(Motor_Stop_Reduce);
                    }
                }
                else
                {

                }
            }
            else//������ڿ���״̬
            {

            }
            pre_limit_rise = Limit_Rise_Signal;
            pre_limit_fall = Limit_Fall_Signal;
        }

        if(cTimer[Motor_Correct] >= 6U)//������λ���غ�ȴ�600ms���������������͵�ǰλ��ֵ
        {
            cTimer[Motor_Correct] = 0;
            if(MOTOR_CORRENT_STS == Corrent_Up)
            {
                MOTOR_CORRENT_STS = Corrent_None;
                OS_ENTER_CRITICAL(); 
                Global_Variable.Suspende_PositionCurrent = Global_Variable.Para_Independence.Suspende_Limit_Up;
                Global_Variable.Compensate_Pulse = 0 - Global_Variable.Encode_PulseCurrent;//��������Ҫ������ֵ
                Global_Variable.Compensate_En = ON;
                OS_EXIT_CRITICAL();
            }
            else if(MOTOR_CORRENT_STS == Corrent_Down)
            {
                MOTOR_CORRENT_STS = Corrent_None;
                OS_ENTER_CRITICAL(); 
                Global_Variable.Suspende_PositionCurrent = 0;
                Global_Variable.Compensate_Pulse = (Global_Variable.Para_Independence.Suspende_Limit_Up/Global_Variable.Para_Independence.Lenth_Per_Pulse)
                    - Global_Variable.Encode_PulseCurrent;//�����˾Ͳ�����ֵ
                Global_Variable.Compensate_En = ON;
                OS_EXIT_CRITICAL();
            }
            else
            {

            }
        }

        
        if(MOTOR_RUNNING_STS == DIR_FALL)
        {
            Delta_Pulse = Global_Variable.Encode_PulseTarget - Global_Variable.Encode_PulseCurrent;
        }
        else if(MOTOR_RUNNING_STS == DIR_RISE)
        {
            Delta_Pulse = Global_Variable.Encode_PulseCurrent - Global_Variable.Encode_PulseTarget;
        }
        else
        {//������ڿ���״̬

        }

        if(Global_Variable.Para_Independence.Convert_Cfg == ON)
        { //�б�Ƶ������
            if((Delta_Pulse < REMAIN_PULSE_NUMBER_FOR_FREQ_STOP)||(Delta_Pulse < 0))
            {
                if(MOTOR_RUNNING_CMD != Motor_Init)//���˳�ʼ��ʱ��Ҫ��������λ���ش�
                {
                    if(BAND_TYPE_BRAKE_OUT == ON)
                    {//�������Ŀ��λ��ʱ����������բ����Ҫ��ǰ��բ
                        cTimer[Motor_Delay] = 0;
                        BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                    }
                    if(MOTOR_RUNNING_STS != OFF)/*&&(MOTOR_REDUCING == OFF)*/
                    {
                        Motor_Stop(Motor_Stop_Reduce);//����Ŀ��λ�ã�ִ�м���ͣ��
                    }
                }
                else
                {
                    Set_Frequence_Running(Delta_Pulse);
                }
            }
            else
            {
                Set_Frequence_Running(Delta_Pulse);
            }

        }
        else
        { //�ޱ�Ƶ������
            if(Delta_Pulse < REMAIN_PULSE_NUMBER_FOR_STOP)
            {
                Motor_Stop(Motor_Stop_Reduce);
            }
        }

        CMD_Freq_Convert(); 

        OSTimeDlyHMSM(0, 0, 0, 1);
    }
    
}


#endif

