#ifndef MBRTU_M_H
#define MBRTU_M_H			  

#include <stdint.h>
#include "stm32f10x.h"
#include "t_list.h"
#include "t_list_impl.h"


#define RTU_SEND_LENGTH             120 * 2          //���ͻ�������С
#define RTU_RECE_LENGTH             120 * 2        //���ܻ�������С

#define UART_CHN_CONVERT_FREQ                RS485_2
#define POLLING_INTERVAL                        100u  //RTU���ڷ�������֡����С���ʱ��
#define REQUIRE_TIMEOUT                        1000u  //RTU����ʱʱ�䣬������ʱ����Ϊ�ӽڵ�δ��Ӧ


enum RTU_Add_Type_t
{
	Add_New,Add_Cycle
};

enum RTU_FSM_t
{
	RTU_REQ,RTU_WAITRESP
};

enum RTU_Result_t
{
    EXCUTE_IDLE=0,EXCUTE_SUCCESS=1,EXCUTE_START=2,EXCUTE_FAIL=0x81
};

enum RTU_TYPE_t
{
    READ,WRITE
};

typedef enum MB_FUNC_TYPE{
	FUNC_RD_COILSTATUS=0X01,
	FUNC_RD_INPUTSTATUS,
	FUNC_RD_HOLDREG,
	FUNC_RD_INREG,
	FUNC_WR_SGCOIL,
	FUNC_WR_SGREG,
	FUNC_WR_MULCOIL=0x0F,
	FUNC_WR_MULREG,
	FUNC_CFG_REG='m',
	CFG_RESP_POS=0x55,
	CFG_RESP_NEG=0xFF,
	MB_RESP_NEG=0x80
}MB_FUNC_TYPE_t;

enum MB_TCP_ErrorStatusTyp{
  Sta_OK=0,
  Err_FunCode,
  Err_DataAddr,
  Err_DataValue,
  Err_MBcmd,
  Err_Busy,
  Err_CfgUnmatch=0x0A, 
  Err_FlashWrFail, 
};

enum STATUS_EVENT_t
{
	EV_NONE,EV_REQ,EV_TO,EV_RX_OK
};

enum PPI_FSM {
    FSM_REQ,
    FSM_WAIT_ACK,
    FSM_FETCH,
    FSM_WAIT_DATA
};

typedef struct RTU_ReqBlock {
	struct list_head Entry;
	u8 Excute_Num;   //ִ�д�����������Ϊ0�����ʾ����ִ��
    enum USARTCAN_CHN chnindex;  //����ͨ��
	u8 sta_addr;          //վ��ַ
	MB_FUNC_TYPE_t FuncCode;  //������
	u8 Status;            //ִ��״̬
	u16 RegAddr;          //��Ҫ�������׵�ַ
	u8  RegNum;           //�����ļĴ�������
	u16* mappedBuff;      //��Ŷ��Ĵ����Ľ����д�Ĵ�����ֵ
}RTU_ReqBlock_t;
extern RTU_ReqBlock_t RTU_Req_Read00; //RTU�Ĵ��������


struct RTU_Ctx {
	volatile enum STATUS_EVENT_t event;
	volatile enum RTU_FSM_t fsm_state;
	volatile enum RTU_FSM_t fsm_next_state;
	struct list_head head;
	struct RTU_ReqBlock* curr;//��ǰ�������
	volatile u16 TOtimer;//��ʱ��ʱ��
	volatile u16 Polltimer;//��ѯ��ʱ��
	volatile enum STATUS_EVENT_t Pollevent;//��ѯ״̬
	u16 guard_time;//��ʱʱ��
	u16 poll_interval;//��ѯʱ��
	u16 txindex; //���ͻ�����Ԫ�ر�ţ���0��ʼ�ۼӣ����ֵ��Ϊ���ͳ���
	u8  txbuff[RTU_SEND_LENGTH];//���ͻ�����
	u8  rxindex;//���ջ�����Ԫ�ر�ţ���0��ʼ�ۼӣ����ֵ��Ϊ���ͳ���
	u8  rxbuff[RTU_RECE_LENGTH];//���ջ�����
};
extern struct RTU_Ctx rtu_ctx;

void RTU_AddReqBlock_Front(struct RTU_Ctx* _rtuctx, struct RTU_ReqBlock* _req);
void RTU_AddReqBlock(struct RTU_Ctx* _rtuctx, struct RTU_ReqBlock* _req);
void RTU_Read(struct RTU_Ctx* _rtuctx);
void RTU_Write(struct RTU_Ctx* _rtuctx);
void RTU_CyclicTask(void);
void UART_RTU_Recv(unsigned char  l_u8ReceData);
void RTU_Timer1ms_Handler(void);
void Task_MBRTU_Master(void *p_arg);

#endif
