#include "main.h"
#include "Task_MB_RTU_Master.h"


struct RTU_Ctx rtu_ctx;//��������
static u8 framestart=OFF;

u8 RTU_485_Enable=OFF;

static u8 OP_Mode=READ;

static volatile BitStatus    	        _RTUSystemStatus[NUM_UARTCHANNEL];
#define SystemStatus(n)                 _RTUSystemStatus[n].Byte
#define Manual_Req(n)                   _RTUSystemStatus[n].Bits.bit0



void UART_RTU_Recv(unsigned char  l_u8ReceData);

/********************************************************************************/
/*��������  RTU_AddReqBlock_Front                                               */
/*����˵������������ӵ����е�ͷ��                                                */
/*���������_rtuctx-Ŀ����У�_req-������                                         */
/*�����������                                                                   */
/********************************************************************************/
void RTU_AddReqBlock_Front(struct RTU_Ctx* _rtuctx, struct RTU_ReqBlock* _req)
{//����ǰ����ŵ���Ϣ���е���λ
	__disable_irq();
	list_add(&_req->Entry, &_rtuctx->head);
	__enable_irq();
	_rtuctx->event=EV_REQ ;
}

/********************************************************************************/
/*��������  RTU_AddReqBlock                                                      */
/*����˵������������ӵ����е�β��                                                */
/*���������_rtuctx-Ŀ����У�_req-������                                         */
/*�����������                                                                   */
/********************************************************************************/
void RTU_AddReqBlock(struct RTU_Ctx* _rtuctx, struct RTU_ReqBlock* _req)
{//����ǰ������뵽����
	__disable_irq();
	list_add_tail(&_req->Entry, &_rtuctx->head);
	__enable_irq();
	_rtuctx->event=EV_REQ ;
}

/********************************************************************************/
/*��������  RTU_DelReqBlock                                                      */
/*����˵����ɾ�����еĵ�һ��Ԫ��                                                  */
/*���������_rtuctx-Ŀ�����                                                     */
/*���������ɾ����һ����Ա�����ǰ��һ����Ա��ַ                                   */
/********************************************************************************/
static struct RTU_ReqBlock* RTU_DelReqBlock(struct RTU_Ctx* _rtu_ctx)
{//ɾ����Ϣ���е��׸�����
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

/********************************************************************************/
/*��������  BSP_RTU_StartSend                                                   */
/*����˵��������BSP��������                                                      */
/*���������_rtuctx-Ŀ�����                                                     */
/*�����������                                                                   */
/********************************************************************************/
static void BSP_RTU_StartSend(struct RTU_Ctx* _rturtx)
{//�������ڷ���
	UartOpFunc[_rturtx->curr->chnindex]._send(_rturtx->txbuff,_rturtx->txindex);
	_rturtx->txindex=0;
	_rturtx->TOtimer = _rturtx->guard_time;
	_rturtx->rxindex = 0;
}
	
/********************************************************************************/
/*��������  RTU_Init                                                             */
/*����˵����ģ���ʼ������                                                        */
/*���������interval-�������һ���ֽں�ļ��ʱ�䣬guard-����������յ���Ӧ�ĳ�ʱʱ��*/
/*�����������                                                                   */
/********************************************************************************/
void RTU_Init(u16 interval,u16 guard)
{//ģ���ʼ��
	memset(&rtu_ctx, 0, sizeof(rtu_ctx));
	rtu_ctx.fsm_state = RTU_REQ;//��ʼ��������ģʽ
	rtu_ctx.guard_time = guard;//��������������Ӧʱ�䣬����Ϊ��ʱ
	rtu_ctx.poll_interval=interval;//��������������ʱ��
	INIT_LIST_HEAD(&rtu_ctx.head);
	RTU_485_Enable = ON;
	rtu_ctx.event = EV_RX_OK;
	rtu_ctx.Pollevent = EV_NONE;
	UartOpFunc[UART_CHN_CONVERT_FREQ]._recv=UART_RTU_Recv;
}

/********************************************************************************/
/*��������  RTU_HandleReply                                                     */
/*����˵�����������ݽ���                                                         */
/*���������_rtuctx-Ŀ�����                                                     */
/*�����������                                                                   */
/********************************************************************************/
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
		if((rtu_ctx.curr->FuncCode == FUNC_RD_COILSTATUS)||(rtu_ctx.curr->FuncCode == FUNC_RD_INPUTSTATUS))
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

/********************************************************************************/
/*��������  RTU_Timer1ms_Handler                                                 */
/*����˵������ʱ״̬����                                                          */
/*�����������                                                                   */
/*�����������                                                                   */
/********************************************************************************/
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

/********************************************************************************/
/*��������  UART_RTU_Recv                                                        */
/*����˵�������յ������ݽ������ж�ÿ���ֽ��Ƿ���Ч������Ч��������֡����             */
/*���������l_u8ReceData����ǰ���յ�����                                          */
/*�����������                                                                   */
/********************************************************************************/
void UART_RTU_Recv(unsigned char  l_u8ReceData)
{
	u16 CrcCheck;
	
	//����
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

/********************************************************************************/
/*��������  RTU_Read                                                             */
/*����˵�������������ݴ��                                                        */
/*���������_rtuctx����ǰ����������Ϣ                                             */
/*�����������                                                                   */
/********************************************************************************/
void RTU_Read(struct RTU_Ctx* _rtuctx)
{
    u16 CrcCheck;
    OP_Mode=READ;
    _rtuctx->curr->FuncCode =(_rtuctx->curr->FuncCode==(MB_FUNC_TYPE_t)0)? (MB_FUNC_TYPE_t)0x03 : _rtuctx->curr->FuncCode ;
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

/********************************************************************************/
/*��������  RTU_Write                                                            */
/*����˵����д�������ݴ��                                                        */
/*���������_rtuctx����ǰ����������Ϣ                                             */
/*�����������                                                                   */
/********************************************************************************/
void RTU_Write(struct RTU_Ctx* _rtuctx)
{
    u16 CrcCheck;
    u8 temp,i;
    OP_Mode=WRITE;
    _rtuctx->curr->FuncCode =(_rtuctx->curr->FuncCode==0u)? (MB_FUNC_TYPE_t)0x10 : _rtuctx->curr->FuncCode ;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->sta_addr;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->FuncCode;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->RegAddr/256;
    _rtuctx->txbuff[_rtuctx->txindex++] = _rtuctx->curr->RegAddr%256;
    if(_rtuctx->curr->FuncCode == FUNC_WR_SGCOIL)
    {
        temp=2;
        _rtuctx->curr->mappedBuff[0]= (_rtuctx->curr->mappedBuff[1]==0x01)?0xff:0x00;
        _rtuctx->curr->mappedBuff[1]=0x00;
    }
    else if(_rtuctx->curr->FuncCode == FUNC_WR_MULCOIL)
    {
        temp = (_rtuctx->curr->RegNum/8)+((_rtuctx->curr->RegNum%8)==0?0:1);
    }
    else if(_rtuctx->curr->FuncCode == FUNC_WR_MULREG)
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

/********************************************************************************/
/*��������  RTU_CyclicTask                                                      */
/*����˵����RTU״̬������                                                        */
/*�����������                                                                  */
/*�����������                                                                  */
/*******************************************************************************/
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
            rtu_ctx.curr->Status=EXCUTE_START; //���ڴ�����
            if ((rtu_ctx.curr->FuncCode == FUNC_RD_COILSTATUS)||(rtu_ctx.curr->FuncCode == FUNC_RD_INPUTSTATUS)||
                (rtu_ctx.curr->FuncCode == FUNC_RD_HOLDREG)||(rtu_ctx.curr->FuncCode == FUNC_RD_INREG))
            {

                RTU_Read(&rtu_ctx);
            }
            else if((rtu_ctx.curr->FuncCode == FUNC_WR_SGCOIL)||(rtu_ctx.curr->FuncCode == FUNC_WR_SGREG)||
                (rtu_ctx.curr->FuncCode == FUNC_WR_MULCOIL)||(rtu_ctx.curr->FuncCode == FUNC_WR_MULREG))
            {
                RTU_Write(&rtu_ctx);
            }
            else
            {

            }
            rtu_ctx.TOtimer = rtu_ctx.guard_time;//ÿ�η�������ʱ���ó�ʱʱ��
            rtu_ctx.fsm_next_state = RTU_WAITRESP;
            break;
    case RTU_WAITRESP:
        if(rtu_ctx.event == EV_TO)
        {
            rtu_ctx.fsm_next_state = RTU_REQ;
            rtu_ctx.curr->Status=EXCUTE_FAIL;
            rtu_ctx.Polltimer=1000;
            if (rtu_ctx.curr->Excute_Num > 1)
            {
                rtu_ctx.curr->Excute_Num--;
                RTU_AddReqBlock(&rtu_ctx,rtu_ctx.curr);
            }
            else if(rtu_ctx.curr->Excute_Num == 0)
            {
                RTU_AddReqBlock(&rtu_ctx,rtu_ctx.curr);
            }
            else
            {

            }
        }
        else if(rtu_ctx.event == EV_RX_OK)
        {
            RTU_HandleReply(&rtu_ctx);
            rtu_ctx.fsm_next_state = RTU_REQ;
            //rtu_ctx.Polltimer=1000;
            if (rtu_ctx.curr->Excute_Num > 1)
            {
                rtu_ctx.curr->Excute_Num--;
                RTU_AddReqBlock(&rtu_ctx,rtu_ctx.curr);
            }
            else if(rtu_ctx.curr->Excute_Num == 0)
            {
                RTU_AddReqBlock(&rtu_ctx,rtu_ctx.curr);
            }
            else
            {

            }
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

/********************************************************************************/
/*��������  Task_MBRTU_Master                                                   */
/*����˵����RTU master��task                                                    */
/*�����������                                                                  */
/*�����������                                                                  */
/*******************************************************************************/
void Task_MBRTU_Master(void *p_arg)
{
    RTU_Init(100,500);
	while(1)
	{    	
    	RTU_CyclicTask();
        OSTimeDlyHMSM(0, 0, 0, 1);
	}
}

