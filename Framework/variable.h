#ifndef __VARIABLE_H_
#define __VARIABLE_H_

#include "wizchip_conf.h"
#include "ucos_ii.h"
#include "MQTTPacket.h"

enum SessionMode//以太网工作模式
{
    S_mqtt=0,S_tcpip_client,S_tcpip_server,S_mb_client,S_mb_server,//后边若有需求，在此往后增加其他工作模式
    S_boundary//边界值，保持在最后，用于程序中识别当前支持的模式数量
};


struct EqController_Cfg_t
{
    u8  cfgflag[4];                       //存放固定的0xAA55AA55，用于识别flash数据是否有效
    u8  session_mode;                     //以太网模式模式
    u16 stationID;                        //MQTT节点ID或MB站地址
    u8  reserve1[1];
    u16 mbtcp_addr;                       //modbus起始地址，大端
    u16 mbtcp_datalen;                    //modbus数据长度，大端
    u16 polltime;                         //modbus轮询时间，大端，单位：1ms
    u8  reserve2[6];
    u8  localIP[4];                       //本地IP地址
    u16 localport;                        //本地端口号，大端
    u8  reserve3[2];
    u8  remoteIP[4];                      //目标IP地址
    u16 remoteport;                       //目标端口号，大端
    u8  reserve4[2];
    u8  submask[4];                       //子网掩码
    u8  gatewayaddr[4];                   //网关地址
    u8  reserve5[4];
    u8  can_en;                           //CAN接口使能
    u8  canbaudrate;                      //CAN波特率，详见配置表
    u16 can_localID;                      //本地CAN-ID，大端
    u16 can_deviceID;                     //响应CAN-ID，大端
    u8  can_device_num;                   //响应CAN-ID的数量
    u8  can_datatype; 
    u8  rs232_1_en;                       //RS232C-1接口使能
    u8  rs232_1_baudrate;                 //RS232C-1接口波特率
    u8  rs232_1_databit;                  //RS232C-1接口数据位
    u8  rs232_1_chkbit;                   //RS232C-1接口校验位
    u8  rs232_1_stopbit;                  //RS232C-1接口停止位
    u8  rs232_1_flowctrl;                 //RS232C-1接口流控
    u8  rs232_1_datatype;                 //RS232C-1数据格式
    u8  rs232_1_tout;                     //RS232C-1超时时间
    u8  reserve7[1];
    u8  rs485_1_en;                         //RS485接口使能
    u8  rs485_1_baudrate;                   //RS485接口波特率
    u8  rs485_1_databit;                    //RS485接口数据位
    u8  rs485_1_chkbit;                     //RS485接口校验位
    u8  rs485_1_stopbit;                    //RS485接口停止位
    u8  rs485_1_datatype;                   //RS485数据格式
    u8  rs485_1_tout;                       //RS485-1超时时间
    u8  reserve8[1];
    u8  rs485_2_en;                         //RS485接口使能
    u8  rs485_2_baudrate;                   //RS485接口波特率
    u8  rs485_2_databit;                    //RS485接口数据位
    u8  rs485_2_chkbit;                     //RS485接口校验位
    u8  rs485_2_stopbit;                    //RS485接口停止位
    u8  rs485_2_datatype;                   //RS485数据格式
    u8  rs485_2_tout;                       //RS485-2超时时间
    u8  reserve9[1];
	u8  rs485_3_en;                         //RS485接口使能
    u8  rs485_3_baudrate;                   //RS485接口波特率
    u8  rs485_3_databit;                    //RS485接口数据位
    u8  rs485_3_chkbit;                     //RS485接口校验位
    u8  rs485_3_stopbit;                    //RS485接口停止位
    u8  rs485_3_datatype;                   //RS485数据格式
    u8  rs485_3_tout;                       //RS485-3超时时间
    u8  reserve10[1];
    u8  rs485_4_en;                         //RS485接口使能
    u8  rs485_4_baudrate;                   //RS485接口波特率
    u8  rs485_4_databit;                    //RS485接口数据位
    u8  rs485_4_chkbit;                     //RS485接口校验位
    u8  rs485_4_stopbit;                    //RS485接口停止位
    u8  rs485_4_datatype;                   //RS485数据格式
    u8  rs485_4_tout;                       //RS485-4超时时间
    u8  reserve11[1];
    u8  to_thres;                         //串口通讯中帧内相邻字符间隔超时时间
};

typedef union {
    u8 Byte;
    struct {
        u8  bit0              :1;
        u8  bit1              :1;
        u8  bit2              :1;
    	u8  bit3              :1;
        u8  bit4              :1;
    	u8  bit5              :1;
        u8  bit6              :1;
    	u8  bit7              :1;
    }Bits;
}BitStatus;


enum UsartType{//串口配置数组各元素的定义，有增加项时请在下边第一行往后增加
    EnUart=0,uartBaudrate,Databits,Chkbits,Stopbits,Flowctrl,uartDatatype,tmout,
    uartcfgnum
};

enum CANType{//CAN的配置数组各元素定义，有增加向时请在下边第一行往后增加
    EnCAN=0,canBaudrate,LocalID,DeviceID,IDNum,canDatatype,
    cancfgnum
};

enum DataType_t{//数据类型，有增加向时请在下边第一行往后增加
    t_HEX=0,t_ASCII,t_BOOL,
    t_typemax
};
extern char DataType[t_typemax][20];

#define NUM_UARTCHANNEL               5  //串口总通道
#define SCI_BUF_MAXLEN              256  //串口发送、接收缓冲区的最大长度,根据需要可能会变更

enum USARTCAN_CHN{//串口和CAN的通道编号
    RS232_1=0,RS485_1,RS485_2,RS485_3,RS485_4,CAN_CHN,
    NUM_UARTCAN
};

/*********************************************************************************************/   
/*********************************************************************************************/ 
/*********************************************************************************************/    
typedef union {
  u16 T_byte;
  struct {
		u8  btn              :4;   //串口帧头或帧尾的字节个数
		u8  bit4             :1;
		u8  bit5             :1;
		u8  bit6             :1;
		u8  en               :1;   //串口帧头或帧尾标志
        u16 reserve          :8;   
	} Bits;
}Tdef_Prot;
#define FrameStartEn                  0x80u  //帧头使能宏
#define FrameEndEn                    0x80u  //帧尾使能宏
enum numbyte{
  byte_1=1,byte_2,byte_3 ,byte_4 ,byte_5 ,byte_6 ,byte_7 ,byte_8 ,byte_9,byte_10     //帧头或帧尾的字节数量
};
enum checksum_t{//校验和方式
    CheckSum_None=0,ChkSum_And,ChkSum_Crc16
};
/*********************************************************************************************/ 
/*********************************************************************************************/ 
/*********************************************************************************************/ 

struct ProtType_t
{
    Tdef_Prot FrameStartInfo;//最高位表示是否使能，低4位表示字节数，只有使能了字节数才有效
    u8 FrameStart[8];        //帧开始符字节数
    Tdef_Prot FrameEndInfo; //最高位表示是否使能，低4位表示字节数，只有使能了字节数才有效
    u8 FrameEnd[8];         //帧结束符字节数
    u8 checksum;          //校验(0-无，1-和，2-crc16)
    u8 inteval;           //字符间隔时间，单位100us
};

typedef struct Interface_Info_t
{
    u8 Usart[NUM_UARTCHANNEL][uartcfgnum];  //串口配置数组
    const struct ProtType_t UsartProt[NUM_UARTCHANNEL];//串口帧头帧尾配置
    u32 can[cancfgnum];    //CAN配置数组
    u32   addr;//modbus起始地址
    u32   datalen;//每个通道mb的长度，实际长度为此值-2，因为有两个字为协议内容，一个为标志位，第二个为字节数量
    u8  sid;//站地址
}Interface_Info;
extern Interface_Info USARTCAN;//

extern wiz_NetInfo gWIZNETINFO;
extern wiz_NetTimeout gWIZNetTimeout;
extern const u8 auchCRC16_Hi[256];
extern const u8 auchCRC16_Low[256];

typedef struct USARTCAN_Recv_info
{
	u8 newupd;//数据更新标志位
    u16 lenth; //字节数量
    u8 datatype;//数据类型
    u8 databuf[SCI_BUF_MAXLEN];//有效数据
}USARTCAN_Recv_t;
extern USARTCAN_Recv_t USARTCAN_Recv[NUM_UARTCAN];

extern u16 g_u16_TCPIPsendlen;           //tcpip报文发送长度

extern u16 cpu_sr;                        //cpu中断状态
extern OS_EVENT *mBOX_Uart_Recv[NUM_UARTCHANNEL];    //所有串口收到消息后需要发送队列给其他task处理

extern char platform_version[];
extern char funcTion[];
extern const u32 RS232_baud[12] ;
extern const u16 RS232_lenth[2] ;
extern const u16 RS232_stop[2] ;
extern const u16 RS232_parity[3] ;
extern const u16 RS232_FlowCntl[4] ;

u16 Get_rtuCrc16(u8 *puchMsg,u16 usDataLen);
USARTCAN_Recv_t GET_UsartCAN_Recv_Result(u8 chanel);
unsigned char AscToHex(unsigned char aChar);
unsigned char HexToAsc(unsigned char aHex);

typedef struct Driver_Variable
{
	
	u16 AD_Result[Channel_Num];//AD采样数据
}Driver_Variable_Info;
extern Driver_Variable_Info Global_Driver;
float GET_ADC_Result(u8 chanel);

typedef struct ParaIndependence_Group
{
    u8 Suspende_Type;//吊杆类型
    u8 Convert_Cfg;//变频器有无，0-无变频器，1-有变频器，通过485控制
    float Motor_Freq_Factor; //电机运行频率的系数C1，本电机的系数为0.3665，实际工程以计算为准
    float Lenth_Per_Pulse;   //每脉冲钢丝绳所走的长度系数C2，本电机所对应的系数为0.1419，实际工程以计算为准
    u8 Max_Motro_Freq;    //电机运行最大频率
    u16 Suspende_Limit_Up;//吊杆上限位高度，单位mm
    u16 Reduce_Limit_Up;//上限位减速信号坐标，单位mm
    u16 Reduce_Limit_Down;//下限位减速信号坐标，单位mm
}ParaIndependence_Info;

typedef struct Global_Variable_Group
{
	u8 DIP_SwitchStatus;                //当前拨码开关状态
	u32 Digit_InputStatus;              //数字开关量状态，0-断开，1-对地闭合
	float CurrentEnvTemp;               //当前环境温度
	s32 Encode_CurrentPulse;            //编码器当前计数
	s32 Encode_TargetPulse;              //编码器目标计数
    float Power_5V;                     //电源电压
    s16 Suspende_Current_Position;      //吊杆当前位置,单位mm
    u16 Suspende_Current_Speed;         //吊杆运行状态,    电机运行的实际    速度，单位：mm/s
    s16 Suspende_Target_Position;       //吊杆目标位置,单位mm
    u16 Suspende_Target_Speed;          //吊杆目标运行的    速度，单位：mm/s
    ParaIndependence_Info  Para_Independence;//微控制器个性化参数
    u8  Para_Common[255];               //读取微控制器共性参数
    u32 powertimer;
}Global_Variable_Info;
extern Global_Variable_Info Global_Variable;



#endif


