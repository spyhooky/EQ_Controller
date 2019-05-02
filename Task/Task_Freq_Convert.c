#include "main.h"
#include "Task_IO.h"
#include "Task_PC_Com.h"
#include "Task_MB_RTU_Master.h"
#include "Task_Freq_Convert.h"
#include "Task_MQTT.h"
#include <stdlib.h>


#ifdef __TASK_FREQ_CONVERT_H

volatile BitStatus Invertor_Status[2];
volatile BitStatus Motor_Status[2];
#define MOTOR_RUNNING                     Motor_Status[0].Bits.bit0 //������б�־
#define MOTOR_RUN_DELAY                   Motor_Status[0].Bits.bit1 //���������ʱ�����ڱ�բ
#define MOTOR_DIRECTOR                    Motor_Status[0].Bits.bit2 //�������з������������½�
#define MOTOR_REDUCING                    Motor_Status[0].Bits.bit3 //������ٱ�־
#define READ_CURR_FREQ_EN                 Motor_Status[0].Bits.bit4 //�Ƿ���Ҫ���Ͳ�ѯ��ﵱǰƵ��ֵ�ı�־
#define Reserve_Requrirement              Motor_Status[0].Bits.bit5 //�����Ҫ�������У��ȼ����ٷ���  
#define FORCE_REDUCE_EN                   Motor_Status[0].Bits.bit6 //����������λ���ػ��ߵ����Ҫ����ʱ�ô˱�־
#define FORCE_REDUCE_10HZ                 Motor_Status[0].Bits.bit7 //ǿ�Ƽ���ʱ��Ƶ�ʵ�10HZ�ı�־
#define MOTOR_Init                        Motor_Status[1].Bits.bit0 //���˳�ʼ��������
#define MOTOR_CORRENT_UP                  Motor_Status[1].Bits.bit1 //�����λ��������־
#define MOTOR_CORRENT_DOWN                Motor_Status[1].Bits.bit2 //�����λ��������־

#define FREQ_REDUCE_TABLE_NUM               15U

#define SWITCH_LIMIT_DETECT                 1U //��⵽��λ���أ���λ�����ǳ��տ��أ���������ʱֵΪ0
#define SWITCH_LIMIT_UNDETECT               0U //δ��⵽��λ����

enum Timer_Type{
    Motor_Delay,            //�ɿ���բ�ļ�ʱ���������1-2s���ɿ���ֹͣ����ʱ�ٱ���
    Read8000,               //��ѯ֡��ʱ������
    Read5001,               //��ȡ��ǰ����Ƶ�ʵļ�����
    Freq_Reduce,            //���ټ��ʱ�������
    Keep_10HZ,              //���������λ�����źź�Ƶ�ʼ���10HZʱ��Ҫ����ά�ֵ�ʱ��
    Motor_Correct,          //���λ���������ȷ�ͣ�������ʱ������ֵ
    
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
    u16 ParaAddr;   //��ʼ�������ļĴ�����ַ
    u16 DataValue;  //��ʼ��������ֵ
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
    {120,16000},//������Ƶ��Ϊ120HZ����ʣ��������С��20000ʱ�Ϳ�ʼ����
    {100,12000},
    {90,10000},
    {80,8000},
    {70,6500},
    {60,5000},
    {50,3500},
    {40,2000},
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
    Invertor_Status[0].Byte = 0;
    Invertor_Status[1].Byte = 0;
    memset((u8 *)&cTimer[0],0,sizeof(cTimer));
    Global_Variable.Suspende_PositionTarget = Global_Variable.Para_Independence.Suspende_Limit_Up;
    Global_Variable.Suspende_PositionCurrent = Global_Variable.Para_Independence.Suspende_Limit_Up;
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
    }

    if(MOTOR_REDUCING == ON)
    {//�����ǰ���ڼ���״̬
        cTimer[Freq_Reduce]++;//���ٳ���ʱ���ʱ
        cTimer[Read5001] = 0;
    }
    else
    {
        cTimer[Freq_Reduce] = 0;
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
		
    if((MOTOR_CORRENT_UP == ON)||(MOTOR_CORRENT_DOWN == ON))
    {//����������λ����
        cTimer[Motor_Correct]++;//�ü�ʱ������������λ����ʱ�ӳ�һ��ʱ��������������͵���λ��ֵ
    }
    else
    {
        cTimer[Motor_Correct] = 0;
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
    if(MOTOR_DIRECTOR == D_FALL)
    {
        if(Global_Variable.Encode_PulseTarget - Global_Variable.Encode_PulseCurrent >= 2000)
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
        if(MOTOR_Init == OFF)
        {
            if(Global_Variable.Encode_PulseCurrent - Global_Variable.Encode_PulseTarget >= 2000)
            {
                motor_freq = Calculate_Frequence();
            }
            else
            {
                motor_freq = Motor_Freq_MIN;
                MOTOR_REDUCING = ON;
            }
        }
        else
        {
            motor_freq = Calculate_Frequence();
        }
    }
    Global_Variable.Suspende_SpeedCurrent = ((u32)motor_freq*(u32)Global_Variable.Para_Independence.Max_Motro_Freq)/10000;
    Global_Variable.Suspende_SpeedCurrent /= Global_Variable.Para_Independence.Motor_Freq_Factor;
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
        if(cTimer[Read5001] >= 1)
        {//��������ڷǼ���״̬ʱ�����Զ�ȡ��ǰ���Ƶ��
            cTimer[Read5001] = 0;
            RTU_AddReqBlock(&rtu_ctx,&RTU_Req_ReadFreq_5001);
            Global_Variable.Suspende_SpeedCurrent = ((InvertorData[off_CurrFreq]*Global_Variable.Para_Independence.Max_Motro_Freq)/10000)/
                Global_Variable.Para_Independence.Motor_Freq_Factor;
            //Global_Variable.Suspende_Current_Speed = (InvertorData[off_CurrFreq]/100)/Global_Variable.Para_Independence.Motor_Freq_Factor;//*10000/100
        }
        curfreq = (Global_Variable.Para_Independence.Max_Motro_Freq * InvertorData[off_CurrFreq]) / 10000;
        
        for(i=0;i<FREQ_REDUCE_TABLE_NUM;i++)
        {
            if(curfreq >= Table_Freq_Reduce[i].reduce_freq)
            {//����������ʼ��ʣ��������
                if((FORCE_REDUCE_EN == ON)||(Table_Freq_Reduce[i].pulse_remain >= Delta_Pulse))
                {//�����������ʱ��ʼ���٣�����λ
                    Global_Variable.Suspende_SpeedCurrent = Table_Freq_Reduce[i].reduce_freq/Global_Variable.Para_Independence.Motor_Freq_Factor;
                    motor_freq = Table_Freq_Reduce[i].reduce_freq*10000/Global_Variable.Para_Independence.Max_Motro_Freq;//
                    MOTOR_REDUCING = ON;
                    break;
                }
            }
        }
        if(MOTOR_REDUCING == OFF)
        {
            motor_freq = WriteData[Convert_Freq];
        }
    }
    else
    {
        if(cTimer[Freq_Reduce] >= FREQ_REDUCE_INTERTER)
        {//ÿ��һ��ʱ����ٹ̶�Ƶ��
            if(WriteData[Convert_Freq] > Motor_Freq_MIN)//Ƶ�ʻ�δ������͵�10HZ
            {
                cTimer[Freq_Reduce] = 0;
                Global_Variable.Suspende_SpeedCurrent -= (u16)(FREQ_REDUCE_BASE/Global_Variable.Para_Independence.Motor_Freq_Factor);
                curfreq = Global_Variable.Suspende_SpeedCurrent * Global_Variable.Para_Independence.Motor_Freq_Factor;//HZ
                motor_freq = curfreq*10000/Global_Variable.Para_Independence.Max_Motro_Freq;//
            }
        }
        else
        {
            motor_freq = WriteData[Convert_Freq];
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
/*��������  MotorMove_Fall                                                        */
/*����˵������������˶�����                                                          */
/*�����������                                                                   */
/*�����������                                                                  */
/*******************************************************************************/
void MotorMove_Fall(void)
{
    MOTOR_DIRECTOR = D_FALL;
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
/*�����������                                                                  */
/*�����������                                                                  */
/*******************************************************************************/
void MotorMove_Rise(void)
{
    MOTOR_DIRECTOR = D_RISE;
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
        MOTOR_RUNNING = OFF;
        FORCE_REDUCE_10HZ = OFF;
        cTimer[Keep_10HZ] = 0;
        FORCE_REDUCE_EN = OFF;
        MOTOR_REDUCING = OFF;
        if((stoptype ==Motor_Stop_Reduce)||(stoptype ==Motor_Stop_Free))
        {   
            stopmode = stoptype;
        }
        else
        {
            stopmode = Motor_Stop_Free;
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
    u8 pre_limit_rise;
    u8 pre_limit_fall;
    while (1)
    {        
        if(CMD_Suspender_Init == ON)
        {//���˳�ʼ������
            CMD_Suspender_Init = OFF;
            Global_Variable.Encode_PulseTarget = (Global_Variable.Suspende_PositionTarget-Global_Variable.Para_Independence.Suspende_Limit_Up) / Global_Variable.Para_Independence.Lenth_Per_Pulse;
            MOTOR_RUNNING = ON;
            MOTOR_Init = ON;
            MotorMove_Rise();
        }
    
        if((FORCE_REDUCE_EN == ON)&&(MOTOR_REDUCING == OFF))//ǿ�Ƽ�����ɣ���ǰƵ������СƵ��10HZ
        {//������������ڸ�while�����ϱ��жϣ�����ᵼ�������޷�����
            if(MOTOR_Init == OFF)
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
                    Global_Variable.Encode_PulseTarget = (s32)((Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget)/ \
                        Global_Variable.Para_Independence.Lenth_Per_Pulse);
                    if(Global_Variable.Encode_PulseTarget > Global_Variable.Encode_PulseCurrent)
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


        if(MOTOR_RUNNING == ON)
        {
            if(((Limit_Up_SlowDown == ON)&&(MOTOR_DIRECTOR == D_RISE))||((Limit_Down_SlowDown == ON)&&(MOTOR_DIRECTOR == D_FALL))||
                (Reserve_Requrirement == ON))//���й�����������λ�����ź�
            {
                if(MOTOR_Init == OFF)//���˳�ʼ��ʱ��Ҫ��������λ���ش�
                {
                    FORCE_REDUCE_EN = ON;
                }
            }
            else
            {
                FORCE_REDUCE_EN = OFF;
            }

            if(Limit_Rise_Signal == ON)
            {//���й�������������λ�ź�
                MOTOR_Init = OFF;
                if(MOTOR_DIRECTOR == D_RISE)
                {//�������
                    if(BAND_TYPE_BRAKE_OUT == ON)
                    {
                        BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                    }
                    Motor_Stop(Motor_Stop_Reduce);
                    
                    if(pre_limit_rise == OFF)
                    {//��Ҫ����λ�ò���
                        MOTOR_CORRENT_UP = ON; 
                    }
                }
                else
                {//����½�

                }
            }
            else
            {
#if 0
                if(pre_limit_rise == ON)
                {//��Ҫ����λ�ò���
                    OS_ENTER_CRITICAL(); 
                    Global_Variable.Suspende_PositionCurrent = Global_Variable.Para_Independence.Suspende_Limit_Up;
                    Global_Variable.Compensate_Pulse = 0 - Global_Variable.Encode_PulseCurrent;//��������Ҫ������ֵ
                    Global_Variable.Compensate_En = ON;
                    OS_EXIT_CRITICAL();
                }
#endif
            }

            if(Limit_Fall_Signal == ON)
            {//���й�������������λ�ź�
                if(MOTOR_DIRECTOR == D_FALL)
                {
                    if(BAND_TYPE_BRAKE_OUT == ON)
                    {
                        BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                    }
                    Motor_Stop(Motor_Stop_Reduce);

                    if(pre_limit_fall == OFF)
                    {//��Ҫ����λ�ò���
                        MOTOR_CORRENT_DOWN = ON; 
                    }
                }
                else
                {

                }
            }
            else
            {
#if 0
                if(pre_limit_fall == ON)
                {//��Ҫ����λ�ò���
                    OS_ENTER_CRITICAL(); 
                    Global_Variable.Suspende_PositionCurrent = 0;
                    Global_Variable.Compensate_Pulse = (Global_Variable.Para_Independence.Suspende_Limit_Up/Global_Variable.Para_Independence.Lenth_Per_Pulse)
                        - Global_Variable.Encode_PulseCurrent;//�����˾Ͳ�����ֵ
                    Global_Variable.Compensate_En = ON;
                    OS_EXIT_CRITICAL();
                }
#endif
            }
        }
        pre_limit_rise = Limit_Rise_Signal;
        pre_limit_fall = Limit_Fall_Signal;

        if(cTimer[Motor_Correct] >= 6U)//������λ���غ�ȴ�600ms���������������͵�ǰλ��ֵ
        {
            cTimer[Motor_Correct] = 0;
            if(MOTOR_CORRENT_UP == ON)
            {
                MOTOR_CORRENT_UP = OFF;
                OS_ENTER_CRITICAL(); 
                Global_Variable.Suspende_PositionCurrent = Global_Variable.Para_Independence.Suspende_Limit_Up;
                Global_Variable.Compensate_Pulse = 0 - Global_Variable.Encode_PulseCurrent;//��������Ҫ������ֵ
                Global_Variable.Compensate_En = ON;
                OS_EXIT_CRITICAL();
            }

            if(MOTOR_CORRENT_DOWN == ON)
            {
                MOTOR_CORRENT_DOWN = OFF;
                OS_ENTER_CRITICAL(); 
                Global_Variable.Suspende_PositionCurrent = 0;
                Global_Variable.Compensate_Pulse = (Global_Variable.Para_Independence.Suspende_Limit_Up/Global_Variable.Para_Independence.Lenth_Per_Pulse)
                    - Global_Variable.Encode_PulseCurrent;//�����˾Ͳ�����ֵ
                Global_Variable.Compensate_En = ON;
                OS_EXIT_CRITICAL();
            }
        }

        
        if(MOTOR_DIRECTOR == D_FALL)
        {
            Delta_Pulse = Global_Variable.Encode_PulseTarget - Global_Variable.Encode_PulseCurrent;
        }
        else
        {
            Delta_Pulse = Global_Variable.Encode_PulseCurrent - Global_Variable.Encode_PulseTarget;
        }

        if(Global_Variable.Para_Independence.Convert_Cfg == ON)
        { //�б�Ƶ������
#if 0
    	    if((Delta_Pulse < 1000u)&&(MOTOR_REDUCING == ON))
            {//������С��1000ʱ�����Ͽ���բ
                if(BAND_TYPE_BRAKE_OUT == ON)
                {
                    BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                }
            }
#endif

            if((Delta_Pulse < REMAIN_PULSE_NUMBER_FOR_FREQ_STOP)||(Delta_Pulse < 0))
            {
                if(BAND_TYPE_BRAKE_OUT == ON)
                {//�������Ŀ��λ��ʱ����������բ����Ҫ��ǰ��բ
                    cTimer[Motor_Delay] = 0;
                    BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                }
                if(MOTOR_RUNNING == ON)/*&&(MOTOR_REDUCING == OFF)*/
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
        { //�ޱ�Ƶ������
            if(Delta_Pulse < REMAIN_PULSE_NUMBER_FOR_STOP)
            {
                Motor_Stop(Motor_Stop_Reduce);
            }
        }
        
        if(CMD_Rope_Wire == ON)//����������������λ����λ�ã����ǵ��˵����λ�ã�
        {
            MOTOR_RUNNING = ON;
            CMD_Rope_Wire = OFF;
            Global_Variable.Encode_PulseTarget = 0;
            MotorMove_Rise();
        }
        
        if(CMD_Suspender_Min == ON)//�������˽����������λ�ã����˵����λ�ã�
        {
            MOTOR_RUNNING = ON;
            CMD_Suspender_Min = OFF;
            Global_Variable.Encode_PulseTarget = Global_Variable.Para_Independence.Suspende_Limit_Up/Global_Variable.Para_Independence.Lenth_Per_Pulse;
            MotorMove_Fall();
        }
        
        if(CMD_Suspender_Emergency_Stop == ON)//��λ������������˼�ͣ
        {
            if(BAND_TYPE_BRAKE_OUT == ON)
            {
                BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
            }
            CMD_Suspender_Emergency_Stop = OFF;
            Motor_Stop(Motor_Stop_Free);            
        }
        
        if(CMD_Suspender_Target == ON)//�����������е��趨��Ŀ��λ��
        {
            MOTOR_RUNNING = ON;
            CMD_Suspender_Target = OFF;
            Global_Variable.Encode_PulseTarget = (s32)((Global_Variable.Para_Independence.Suspende_Limit_Up-Global_Variable.Suspende_PositionTarget)/ \
                Global_Variable.Para_Independence.Lenth_Per_Pulse);
            if(Global_Variable.Encode_PulseTarget > Global_Variable.Encode_PulseCurrent)
            {
                MotorMove_Fall();
            }
            else
            {
                MotorMove_Rise();
            }
        }

        if(Err_Stop_Signal == ON)//��ͣ��ť�����˼�ͣ
        {
            if(MOTOR_RUNNING == ON)
            {
                if(BAND_TYPE_BRAKE_OUT == ON)
                {
                    BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                }
                Motor_Stop(Motor_Stop_Free);   
            }
        }

        if(Err_Summit_Attempt == ON)//�嶥���ϣ����˼�ͣ
        {
            if(MOTOR_RUNNING == ON)
            {
                if(BAND_TYPE_BRAKE_OUT == ON)
                {
                    BAND_TYPE_BRAKE_OUT = OFF;//��բ�Ͽ�
                }
                Motor_Stop(Motor_Stop_Free);   
            }
        }

        OSTimeDlyHMSM(0, 0, 0, 1);
    }
    
}


#endif

