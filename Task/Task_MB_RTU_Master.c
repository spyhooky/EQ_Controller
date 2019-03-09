#include "main.h"
#include "Task_MB_RTU_Master.h"


static struct RTU_Ctx rtu_ctx;//变量定义
u16 rtutimer=0;
static u8 framestart=OFF;

u8 RTU_485_Enable=OFF;
u8 ADDR[40];
u8 Wrdata[40];
static u8 OP_Mode=READ;

static volatile BitStatus    	        _RTUSystemStatus[NUM_UARTCHANNEL];
#define SystemStatus(n)                 _RTUSystemStatus[n].Byte
#define Manual_Req(n)                   _RTUSystemStatus[n].Bits.bit0



void UART_RTU_Recv(unsigned char  l_u8ReceData);

struct RTU_ReqBlock RTU_AReqGrp= //自动请求块
{
	LIST_HEAD_INIT(RTU_AReqGrp.Entry),
	UART_CHN_RTU_MASTER,
	1,
	M_RdHold,
	EXCUTE_SUCCESS,
	0x0000,
	0x01,
	(u8*)&ADDR
};

struct RTU_ReqBlock RTU_MReqGrp= //手动请求块
{
	LIST_HEAD_INIT(RTU_MReqGrp.Entry),
	UART_CHN_RTU_MASTER,
	2,
	M_WrHold,
	EXCUTE_SUCCESS,
	0x0002,
	0x01,
	(u8*)&Wrdata
};

void RTU_AddReqBlock_Front(struct RTU_Ctx* _rtuctx, struct RTU_ReqBlock* _req)
{//将当前请求放到消息队列的首位
	__disable_irq();
	list_add(&_req->Entry, &_rtuctx->head);
	__enable_irq();
	_rtuctx->event=EV_REQ ;
}

void RTU_AddReqBlock(struct RTU_Ctx* _rtuctx, struct RTU_ReqBlock* _req)
{//将当前请求加入到队列
	__disable_irq();
	list_add_tail(&_req->Entry, &_rtuctx->head);
	__enable_irq();
	_rtuctx->event=EV_REQ ;
}

static struct RTU_ReqBlock* RTU_DelReqBlock(struct RTU_Ctx* _rtu_ctx)
{//删除消息队列的首个请求
    struct RTU_ReqBlock* req = 0;
    __disable_irq();
    if (!list_empty(&_rtu_ctx->head))
    {
        req = (struct RTU_ReqBlock*)_rtu_ctx->head.next;
        if (req)
        {
            list_del(_rtu_ctx->head.next);
        }
    }
    __enable_irq();
    return req;
}

static void BSP_RTU_StartSend(struct RTU_Ctx* _rturtx)
{//启动串口发送
	UartOpFunc[_rturtx->curr->chnindex]._send(_rturtx->txbuff,_rturtx->txindex);
	_rturtx->txindex=0;
	_rturtx->TOtimer = _rturtx->guard_time;
	_rturtx->rxindex = 0;
}
	
void RTU_Init(u8 interval,u16 guard)
{//模块初始化
	memset(&rtu_ctx, 0, sizeof(rtu_ctx));
	rtu_ctx.fsm_state = RTU_REQ;//初始化成请求模式
	rtu_ctx.guard_time = guard;//请求的最大允许响应时间，否则为超时
	rtu_ctx.poll_interval=interval;//连续两个请求间隔时间
	INIT_LIST_HEAD(&rtu_ctx.head);
	RTU_485_Enable = ON;
	RTU_AddReqBlock(&rtu_ctx,&RTU_AReqGrp);
	rtu_ctx.event = EV_RX_OK;
	rtu_ctx.Pollevent = EV_NONE;
	rtu_ctx.Polltimer =100;//用于轮询的定时器初始值
	UartOpFunc[UART_CHN_RTU_MASTER]._recv=UART_RTU_Recv;
    //UartOpFunc[UART_CHN_RTU_MASTER]._send=USART3_Send_Data;
	//UartOpFunc[1]._recv=UART_RTU_Recv;
}

void RTU_HandleReply(struct RTU_Ctx* _rtuctx)
{
	u8 i,j;
	rtu_ctx.TOtimer=0;
	if((rtu_ctx.rxbuff[1]&MB_RESP_NEG)!=MB_RESP_NEG)
	{
		rtu_ctx.curr->Status = EXCUTE_SUCCESS;
	}
	else
	{
		rtu_ctx.curr->Status = EXCUTE_FAIL;
	}
	if(OP_Mode==READ)
	{
		if((rtu_ctx.curr->FuncCode == S_RdCoil)||(rtu_ctx.curr->FuncCode == S_RdInput))
		{
			for(i=0;i<_rtuctx->rxbuff[2];i++)
			{
				for(j=0;j<8;j++)
				{
					_rtuctx->curr->mappedBuff[(8*2*i)+(2*j)]=0x00;
					_rtuctx->curr->mappedBuff[(8*2*i)+(2*j)+1]=(_rtuctx->rxbuff[3+i]>>j)&0x01;
				}
			}
		}
		else
		{
			memcpy(_rtuctx->curr->mappedBuff,&_rtuctx->rxbuff[3],_rtuctx->rxbuff[2]);
		}
	}
	memset(_rtuctx->rxbuff,0,_rtuctx->rxindex+1);
	_rtuctx->rxindex=0;
}

//=====================================================================
void UART_RTU_Recv(unsigned char  l_u8ReceData)
{
	u16 CrcCheck;
	
	//接收
	if((framestart==OFF)&&(l_u8ReceData==rtu_ctx.curr->sta_addr))
	{
		framestart=ON;
		rtu_ctx.rxindex=0;
		rtu_ctx.rxbuff[rtu_ctx.rxindex]=l_u8ReceData;
	}
	
	if(framestart==ON)
	{
		rtu_ctx.rxbuff[rtu_ctx.rxindex]=l_u8ReceData;
	}
	
	if(((rtu_ctx.rxbuff[2]+5)==(rtu_ctx.rxindex+1))&&(OP_Mode==READ))
	{
		CrcCheck=Get_rtuCrc16(rtu_ctx.rxbuff,rtu_ctx.rxbuff[2]+3);
		if(((CrcCheck>>8) == rtu_ctx.rxbuff[rtu_ctx.rxindex-1]) && ((CrcCheck%256) == rtu_ctx.rxbuff[rtu_ctx.rxindex]))
		{
			framestart=OFF;
			rtu_ctx.event = EV_RX_OK;
		}
		else
		{
			
		}
	}
	else if((rtu_ctx.rxindex==7)&&(OP_Mode==WRITE))
	{
		framestart=OFF;
		rtu_ctx.event = EV_RX_OK;
	}
	else
	{
		
	}
	rtu_ctx.rxindex++;
}

void RTU_Read(struct RTU_Ctx* _rtuctx)
{
    u16 CrcCheck;
    OP_Mode=READ;
    _rtuctx->curr->FuncCode =(_rtuctx->curr->FuncCode==(FuncCode_t)0)? (FuncCode_t)0x03 : _rtuctx->curr->FuncCode ;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->sta_addr;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->FuncCode;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->RegAddr/256;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->RegAddr%256;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->RegNum/256;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->RegNum%256;
    CrcCheck = Get_rtuCrc16(_rtuctx->txbuff,_rtuctx->txindex);
    _rtuctx->txbuff[_rtuctx->txindex++] =  CrcCheck>>8;
    _rtuctx->txbuff[_rtuctx->txindex++] =  CrcCheck;
    BSP_RTU_StartSend(_rtuctx);
}

void RTU_Write(struct RTU_Ctx* _rtuctx)
{
    u16 CrcCheck;
    u8 temp,i;
    OP_Mode=WRITE;
    _rtuctx->curr->FuncCode =(_rtuctx->curr->FuncCode==0u)? (FuncCode_t)0x10 : _rtuctx->curr->FuncCode ;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->sta_addr;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->FuncCode;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->RegAddr/256;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->RegAddr%256;
    if(_rtuctx->curr->FuncCode == S_WrCoil)
    {
        temp=2;
        _rtuctx->curr->mappedBuff[0]= (_rtuctx->curr->mappedBuff[1]==0x01)?0xff:0x00;
        _rtuctx->curr->mappedBuff[1]=0x00;
    }
    else if(_rtuctx->curr->FuncCode == M_WrCoil)
    {
        temp = (_rtuctx->curr->RegNum/8)+((_rtuctx->curr->RegNum%8)==0?0:1);
    }
    else if(_rtuctx->curr->FuncCode == M_WrHold)
    {
        _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->RegNum/256;
        _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->RegNum%256;
        temp = _rtuctx->curr->RegNum * 2;
        _rtuctx->txbuff[_rtuctx->txindex++] = temp;
    }
    else
    {
        temp = _rtuctx->curr->RegNum * 2;
    }
    for(i=0;i<temp;i++)
    {
        _rtuctx->txbuff[_rtuctx->txindex++]=_rtuctx->curr->mappedBuff[i];
    }
    CrcCheck = Get_rtuCrc16(_rtuctx->txbuff,_rtuctx->txindex);
    _rtuctx->txbuff[_rtuctx->txindex++] =  CrcCheck>>8;
    _rtuctx->txbuff[_rtuctx->txindex++] =  CrcCheck;
    BSP_RTU_StartSend(_rtuctx);
}

void RTU_CyclicTask(void)
{
    if (rtu_ctx.event == EV_NONE)
    {
        return;
    }
	
	switch (rtu_ctx.fsm_state)
    {
        default:
        case RTU_REQ:
            rtu_ctx.curr = RTU_DelReqBlock(&rtu_ctx);
            if (rtu_ctx.curr == 0)
            {
                return;
            }
            rtu_ctx.curr->Status=EXCUTE_START; //正在处理中
            if ((rtu_ctx.curr->FuncCode == S_RdCoil)||(rtu_ctx.curr->FuncCode == S_RdInput)||
                (rtu_ctx.curr->FuncCode == M_RdHold)||(rtu_ctx.curr->FuncCode == M_RdInRegs))
            {

                RTU_Read(&rtu_ctx);
            }
            else if((rtu_ctx.curr->FuncCode == S_WrCoil)||(rtu_ctx.curr->FuncCode == S_WrHold)||
                (rtu_ctx.curr->FuncCode == M_WrCoil)||(rtu_ctx.curr->FuncCode == M_WrHold))
            {
                RTU_Write(&rtu_ctx);
            }
            else
            {

            }
            rtu_ctx.TOtimer = rtu_ctx.guard_time;//每次发送请求时设置超时时间
            rtu_ctx.fsm_next_state = RTU_WAITRESP;
            break;
    case RTU_WAITRESP:
        if(rtu_ctx.event == EV_TO)
        {
            rtu_ctx.fsm_next_state = RTU_REQ;
            rtu_ctx.curr->Status=EXCUTE_FAIL;
        }
        else if(rtu_ctx.event == EV_RX_OK)
        {
            RTU_HandleReply(&rtu_ctx);
            rtu_ctx.fsm_next_state = RTU_REQ;
        }
        else
        {

        }
        rtu_ctx.TOtimer = rtu_ctx.poll_interval;
        break;
    }
    rtu_ctx.fsm_state = rtu_ctx.fsm_next_state;
    rtu_ctx.event = EV_NONE;
}

void RTU_Timer1ms_Handler(void)
{
	if(RTU_485_Enable == ON)
	{
		if((rtu_ctx.TOtimer>0)&&(!--rtu_ctx.TOtimer))
		{
			rtu_ctx.event = EV_TO;
		}
		
		if(rtu_ctx.fsm_state == FSM_REQ)
		{
			if((rtu_ctx.Polltimer>0)&&(!--rtu_ctx.Polltimer))
			{
				rtu_ctx.Pollevent = EV_TO;
			}
		}
		else
		{
			//rtu_ctx.Polltimer=0;
		}
	}
}

void Task_MBRTU_Master(void *p_arg)
{
    //u8 datatyppe;
    //u16 dataaddr;
	//datatyppe=M_RdHold;
	//API_RTU_HOOK(RTU_MReqGrp);
	u8 func=M_WrHold;
    u8 num=1;
    RTU_Init(100,100);
	while(1)
	{
        if(rtu_ctx.Pollevent==EV_TO)
        {
            rtu_ctx.Polltimer=100;
            if(Manual_Req(0)==OFF)
            {
                RTU_AReqGrp.chnindex=UART_CHN_RTU_MASTER;
                RTU_AReqGrp.sta_addr=0x01;
                RTU_AReqGrp.FuncCode=M_RdHold;
                RTU_AReqGrp.RegAddr=0;
                RTU_AReqGrp.RegNum=num;
                RTU_AReqGrp.mappedBuff=ADDR;
                RTU_AddReqBlock(&rtu_ctx,&RTU_AReqGrp);
            }
            else
            {
                Manual_Req(0)=0;
                RTU_MReqGrp.chnindex=UART_CHN_RTU_MASTER;
                RTU_MReqGrp.sta_addr=0x01;
                RTU_MReqGrp.FuncCode=(FuncCode_t)func;//M_WrHold;
                RTU_MReqGrp.RegAddr=0;
                RTU_MReqGrp.RegNum=num;
                RTU_MReqGrp.mappedBuff=Wrdata;
                RTU_AddReqBlock(&rtu_ctx,&RTU_MReqGrp);
            }
            rtu_ctx.Pollevent=EV_NONE;
        }
    	
    	RTU_CyclicTask();
        OSTimeDlyHMSM(0, 0, 0, 1);
	}
}

