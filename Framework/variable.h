#ifndef __VARIABLE_H_
#define __VARIABLE_H_

#include "wizchip_conf.h"
#include "ucos_ii.h"
#include "MQTTPacket.h"

enum SessionMode//��̫������ģʽ
{
    S_mqtt=0,S_tcpip_client,S_tcpip_server,S_mb_client,S_mb_server,//������������ڴ�����������������ģʽ
    S_boundary//�߽�ֵ��������������ڳ�����ʶ��ǰ֧�ֵ�ģʽ����
};


struct EthernetCfg_t
{
    u8  cfgflag[4];                       //��Ź̶���0xAA55AA55������ʶ��flash�����Ƿ���Ч
    u8  session_mode;                     //��̫��ģʽģʽ
    u16 stationID;                        //MQTT�ڵ�ID��MBվ��ַ
    u8  reserve1[1];
    u16 mbtcp_addr;                       //modbus��ʼ��ַ�����
    u16 mbtcp_datalen;                    //modbus���ݳ��ȣ����
    u16 polltime;                         //modbus��ѯʱ�䣬��ˣ���λ��1ms
    u8  reserve2[6];
    u8  localIP[4];                       //����IP��ַ
    u16 localport;                        //���ض˿ںţ����
    u8  reserve3[2];
    u8  remoteIP[4];                      //Ŀ��IP��ַ
    u16 remoteport;                       //Ŀ��˿ںţ����
    u8  reserve4[2];
    u8  submask[4];                       //��������
    u8  gatewayaddr[4];                   //���ص�ַ
    u8  reserve5[4];
    u8  can_en;                           //CAN�ӿ�ʹ��
    u8  canbaudrate;                      //CAN�����ʣ�������ñ�
    u16 can_localID;                      //����CAN-ID�����
    u16 can_deviceID;                     //��ӦCAN-ID�����
    u8  can_device_num;                   //��ӦCAN-ID������
    u8  can_datatype; 
    u8  rs232_1_en;                       //RS232C-1�ӿ�ʹ��
    u8  rs232_1_baudrate;                 //RS232C-1�ӿڲ�����
    u8  rs232_1_databit;                  //RS232C-1�ӿ�����λ
    u8  rs232_1_chkbit;                   //RS232C-1�ӿ�У��λ
    u8  rs232_1_stopbit;                  //RS232C-1�ӿ�ֹͣλ
    u8  rs232_1_flowctrl;                 //RS232C-1�ӿ�����
    u8  rs232_1_datatype;                 //RS232C-1���ݸ�ʽ
    u8  reserve7[1];
    u8  rs485_1_en;                         //RS485�ӿ�ʹ��
    u8  rs485_1_baudrate;                   //RS485�ӿڲ�����
    u8  rs485_1_databit;                    //RS485�ӿ�����λ
    u8  rs485_1_chkbit;                     //RS485�ӿ�У��λ
    u8  rs485_1_stopbit;                    //RS485�ӿ�ֹͣλ
    u8  rs485_1_datatype;                   //RS485���ݸ�ʽ
    u8  reserve8[1];
    u8  rs485_2_en;                         //RS485�ӿ�ʹ��
    u8  rs485_2_baudrate;                   //RS485�ӿڲ�����
    u8  rs485_2_databit;                    //RS485�ӿ�����λ
    u8  rs485_2_chkbit;                     //RS485�ӿ�У��λ
    u8  rs485_2_stopbit;                    //RS485�ӿ�ֹͣλ
    u8  rs485_2_datatype;                   //RS485���ݸ�ʽ
    u8  reserve9[1];
	u8  rs485_3_en;                         //RS485�ӿ�ʹ��
    u8  rs485_3_baudrate;                   //RS485�ӿڲ�����
    u8  rs485_3_databit;                    //RS485�ӿ�����λ
    u8  rs485_3_chkbit;                     //RS485�ӿ�У��λ
    u8  rs485_3_stopbit;                    //RS485�ӿ�ֹͣλ
    u8  rs485_3_datatype;                   //RS485���ݸ�ʽ
    u8  reserve10[1];
    u8  rs485_4_en;                         //RS485�ӿ�ʹ��
    u8  rs485_4_baudrate;                   //RS485�ӿڲ�����
    u8  rs485_4_databit;                    //RS485�ӿ�����λ
    u8  rs485_4_chkbit;                     //RS485�ӿ�У��λ
    u8  rs485_4_stopbit;                    //RS485�ӿ�ֹͣλ
    u8  rs485_4_datatype;                   //RS485���ݸ�ʽ
    u8  reserve11[1];
    u8  to_thres;                         //����ͨѶ��֡�������ַ������ʱʱ��
};


enum UsartType{//�������������Ԫ�صĶ��壬��������ʱ�����±ߵ�һ����������
    EnUart=0,uartBaudrate,Databits,Chkbits,Stopbits,Flowctrl,uartDatatype,
    uartcfgnum
};

enum CANType{//CAN�����������Ԫ�ض��壬��������ʱ�����±ߵ�һ����������
    EnCAN=0,canBaudrate,LocalID,DeviceID,IDNum,canDatatype,
    cancfgnum
};

enum DataType_t{//�������ͣ���������ʱ�����±ߵ�һ����������
    t_HEX=0,t_ASCII,t_BOOL,
    t_typemax
};
extern char DataType[t_typemax][20];

#define NUM_UARTCHANNEL               5  //������ͨ��
#define SCI_BUF_MAXLEN              256  //���ڷ��͡����ջ���������󳤶�,������Ҫ���ܻ���

enum USARTCAN_CHN{//���ں�CAN��ͨ�����
    RS232_1=0,RS485_1,RS485_2,RS485_3,RS485_4,CAN_CHN,
    NUM_UARTCAN
};

/*********************************************************************************************/   
/*********************************************************************************************/ 
/*********************************************************************************************/    
typedef union {
  u16 T_byte;
  struct {
		u8  btn              :4;   //����֡ͷ��֡β���ֽڸ���
		u8  bit4             :1;
		u8  bit5             :1;
		u8  bit6             :1;
		u8  en               :1;   //����֡ͷ��֡β��־
        u16 reserve          :8;   
	} Bits;
}Tdef_Prot;
#define FrameStartEn                  0x80u  //֡ͷʹ�ܺ�
#define FrameEndEn                    0x80u  //֡βʹ�ܺ�
enum numbyte{
  byte_1=1,byte_2,byte_3 ,byte_4 ,byte_5 ,byte_6 ,byte_7 ,byte_8 ,byte_9,byte_10     //֡ͷ��֡β���ֽ�����
};
enum checksum_t{//У��ͷ�ʽ
    CheckSum_None=0,ChkSum_And,ChkSum_Crc16
};
/*********************************************************************************************/ 
/*********************************************************************************************/ 
/*********************************************************************************************/ 

struct ProtType_t
{
    Tdef_Prot FrameStartInfo;//���λ��ʾ�Ƿ�ʹ�ܣ���4λ��ʾ�ֽ�����ֻ��ʹ�����ֽ�������Ч
    u8 FrameStart[8];        //֡��ʼ���ֽ���
    Tdef_Prot FrameEndInfo; //���λ��ʾ�Ƿ�ʹ�ܣ���4λ��ʾ�ֽ�����ֻ��ʹ�����ֽ�������Ч
    u8 FrameEnd[8];         //֡�������ֽ���
    u8 checksum;          //У��(0-�ޣ�1-�ͣ�2-crc16)
};

typedef struct Interface_Info_t
{
    u8 Usart[NUM_UARTCHANNEL][uartcfgnum];  //������������
    const struct ProtType_t UsartProt[NUM_UARTCHANNEL];//����֡ͷ֡β����
    u32 can[cancfgnum];    //CAN��������
    u32   addr;//modbus��ʼ��ַ
    u32   datalen;//ÿ��ͨ��mb�ĳ��ȣ�ʵ�ʳ���Ϊ��ֵ-2����Ϊ��������ΪЭ�����ݣ�һ��Ϊ��־λ���ڶ���Ϊ�ֽ�����
    u8  sid;//վ��ַ
    u8  tout;//����ͨѶ֡��ʱʱ��
}Interface_Info;
extern Interface_Info USARTCAN;//

extern wiz_NetInfo gWIZNETINFO;
extern wiz_NetTimeout gWIZNetTimeout;



struct USARTCAN_Recv_t
{
	u8 newupd;//���ݸ��±�־λ
    u16 lenth; //�ֽ�����
    u8 datatype;//��������
    u8 databuf[SCI_BUF_MAXLEN];//��Ч����
};
extern struct USARTCAN_Recv_t USARTCAN_Recv[NUM_UARTCAN];

extern u16 g_u16_TCPIPsendlen;           //tcpip���ķ��ͳ���

extern u16 cpu_sr;                        //cpu�ж�״̬

extern char platform_version[];
extern char funcTion[];
extern const u32 RS232_baud[12] ;
extern const u16 RS232_lenth[2] ;
extern const u16 RS232_stop[2] ;
extern const u16 RS232_parity[3] ;
extern const u16 RS232_FlowCntl[4] ;

unsigned char AscToHex(unsigned char aChar);
unsigned char HexToAsc(unsigned char aHex);

typedef struct Driver_Variable
{
	
	u16 AD_Result[Channel_Num];//AD��������
}Driver_Variable_Info;
extern Driver_Variable_Info Global_Driver;

typedef struct Framework_Variable
{
	u8 DIP_SwitchStatus;   //��ǰ���뿪��״̬
	u32 Digit_InputStatus; //���ֿ�����״̬��0-�Ͽ���1-�Եرպ�
	u16 CurrentEnvTemp;   //��ǰ�����¶�
    u8  Power_5V;
}Framework_Variable_Info;
extern Framework_Variable_Info Globle_Framework;



#endif


