#include "main.h"
#include "Task_MB_RTU_Master.h"


struct RTU_Ctx rtu_ctx;//变量定义
static u8 framestart=OFF;
vu32 TestTimer_E;


u8 RTU_485_Enable=OFF;

static u8 OP_Mode=READ;

static volatile BitStatus    	        _RTUSystemStatus[NUM_UARTCHANNEL];
#define SystemStatus(n)                 _RTUSystemStatus[n].Byte
#define Manual_Req(n)                   _RTUSystemStatus[n].Bits.bit0

void UART_RTU_Recv(unsigned char  l_u8ReceData);

/********************************************************************************/
/*函数名：  RTU_AddReqBlock_Front                                               */
/*功能说明：将新请求加到队列的头部                                                */
/*输入参数：_rtuctx-目标队列，_req-新请求                                         */
/*输出参数：无                                                                   */
/********************************************************************************/
void RTU_AddReqBlock_Front(struct RTU_Ctx* _rtuctx, struct RTU_ReqBlock* _req)
{//将当前请求放到消息队列的首位
    struct list_head* p;
    p = &_rtuctx->head;
    do
    {
        if(p == _req->Entry.next)
        {
            return;
        }
        p = p->next;
    }
    while(p != &_rtuctx->head);
	__disable_irq();
	list_add(&_req->Entry, &_rtuctx->head);
	__enable_irq();
	_rtuctx->event=EV_REQ ;
}

/********************************************************************************/
/*函数名：  RTU_AddReqBlock                                                      */
/*功能说明：将新请求加到队列的尾部                                                */
/*输入参数：_rtuctx-目标队列，_req-新请求                                         */
/*输出参数：无                                                                   */
/********************************************************************************/
void RTU_AddReqBlock(struct RTU_Ctx* _rtuctx, struct RTU_ReqBlock* _req)
{//将当前请求加入到队列
    struct list_head* p;
    p = &_rtuctx->head;
    do
    {
        if(p == _req->Entry.next)
        {
            return;
        }
        p = p->next;
    }
    while(p != &_rtuctx->head);
	__disable_irq();
    _req->Status = EXCUTE_IDLE;
	list_add_tail(&_req->Entry, &_rtuctx->head);
	__enable_irq();
	_rtuctx->event=EV_REQ ;
}

/********************************************************************************/
/*函数名：  RTU_DelReqBlock                                                      */
/*功能说明：删除队列的第一个元素                                                  */
/*输入参数：_rtuctx-目标队列                                                     */
/*输出参数：删除第一个成员后的最前边一个成员地址                                   */
/********************************************************************************/
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

/********************************************************************************/
/*函数名：  BSP_RTU_StartSend                                                   */
/*功能说明：调用BSP发送数据                                                      */
/*输入参数：_rtuctx-目标队列                                                     */
/*输出参数：无                                                                   */
/********************************************************************************/
static void BSP_RTU_StartSend(struct RTU_Ctx* _rturtx)
{//启动串口发送
    TestTimer_E = Global_Variable.powertimer;
	UartOpFunc[_rturtx->curr->chnindex]._send(_rturtx->txbuff,_rturtx->txindex);
	_rturtx->txindex=0;
	_rturtx->TOtimer = _rturtx->guard_time;
	_rturtx->rxindex = 0;
}
	
/********************************************************************************/
/*函数名：  RTU_Init                                                             */
/*功能说明：模块初始化函数                                                        */
/*输入参数：interval-发送最后一个字节后的间隔时间，guard-发送请求后收到响应的超时时间*/
/*输出参数：无                                                                   */
/********************************************************************************/
void RTU_Init(u16 interval,u16 guard)
{//模块初始化
	memset(&rtu_ctx, 0, sizeof(rtu_ctx));
	rtu_ctx.fsm_state = RTU_REQ;//初始化成请求模式
	rtu_ctx.guard_time = guard;//请求的最大允许响应时间，否则为超时
	rtu_ctx.poll_interval=interval;//连续两个请求间隔时间
	INIT_LIST_HEAD(&rtu_ctx.head);
	RTU_485_Enable = ON;
	rtu_ctx.event = EV_RX_OK;
	rtu_ctx.Pollevent = EV_NONE;
	UartOpFunc[UART_CHN_CONVERT_FREQ]._recv=UART_RTU_Recv;
}

/********************************************************************************/
/*函数名：  RTU_HandleReply                                                     */
/*功能说明：接收数据解析                                                         */
/*输入参数：_rtuctx-目标队列                                                     */
/*输出参数：无                                                                   */
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
					_rtuctx->curr->mappedBuff[(8*2*i)+(2*j)]=0x00 + (_rtuctx->rxbuff[3+i]>>j)&0x01;
				}
			}
		}
		else
		{
            for(j=0;j<_rtuctx->rxbuff[2];j=j+2)
			{
                _rtuctx->curr->mappedBuff[j/2] = (_rtuctx->rxbuff[3+j]<<8) + _rtuctx->rxbuff[3+j+1];
            }
		}
	}
	memset(_rtuctx->rxbuff,0,_rtuctx->rxindex+1);
	_rtuctx->rxindex=0;
}

/********************************************************************************/
/*函数名：  RTU_Timer1ms_Handler                                                 */
/*功能说明：超时状态管理                                                          */
/*输入参数：无                                                                   */
/*输出参数：无                                                                   */
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
/*函数名：  UART_RTU_Recv                                                        */
/*功能说明：接收到的数据解析，判断每个字节是否有效，若无效则抛弃该帧数据             */
/*输入参数：l_u8ReceData，当前接收的数据                                          */
/*输出参数：无                                                                   */
/********************************************************************************/
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
			rtu_ctx.event = EV_RX_CRC_ERR;
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
/*函数名：  RTU_Read                                                             */
/*功能说明：读请求数据打包                                                        */
/*输入参数：_rtuctx，当前队列数据信息                                             */
/*输出参数：无                                                                   */
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
/*函数名：  RTU_Write                                                            */
/*功能说明：写请求数据打包                                                        */
/*输入参数：_rtuctx，当前队列数据信息                                             */
/*输出参数：无                                                                   */
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
        _rtuctx->curr->mappedBuff[0] = (_rtuctx->curr->mappedBuff[0]==0x01)?0xff00:0x0000;
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
        if(i%2 == 0)
        {
            _rtuctx->txbuff[_rtuctx->txindex++]=_rtuctx->curr->mappedBuff[i/2]>>8;
        }
        else
        {
            _rtuctx->txbuff[_rtuctx->txindex++]=_rtuctx->curr->mappedBuff[i/2]&0xff;
        }
    }
    CrcCheck = Get_rtuCrc16(_rtuctx->txbuff,_rtuctx->txindex);
    _rtuctx->txbuff[_rtuctx->txindex++] =  CrcCheck>>8;
    _rtuctx->txbuff[_rtuctx->txindex++] =  CrcCheck;
    BSP_RTU_StartSend(_rtuctx);
}

/********************************************************************************/
/*函数名：  RTU_CyclicTask                                                      */
/*功能说明：RTU状态机管理                                                        */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
void RTU_CyclicTask(void)
{
    if (rtu_ctx.event == EV_NONE)
    {//空闲状态
        return;
    }
	
	switch (rtu_ctx.fsm_state)//有超时事件或者有新block请求
    { 
        case RTU_REQ://请求发送状态
            rtu_ctx.curr = RTU_DelReqBlock(&rtu_ctx);
            if (rtu_ctx.curr == NULL)
            {
                return;
            }
            rtu_ctx.curr->Status=EXCUTE_START; //正在处理中
            if ((rtu_ctx.curr->FuncCode == FUNC_RD_COILSTATUS)||(rtu_ctx.curr->FuncCode == FUNC_RD_INPUTSTATUS)||
                (rtu_ctx.curr->FuncCode == FUNC_RD_HOLDREG)||(rtu_ctx.curr->FuncCode == FUNC_RD_INREG))
            {//读寄存器

                RTU_Read(&rtu_ctx);
            }
            else if((rtu_ctx.curr->FuncCode == FUNC_WR_SGCOIL)||(rtu_ctx.curr->FuncCode == FUNC_WR_SGREG)||
                (rtu_ctx.curr->FuncCode == FUNC_WR_MULCOIL)||(rtu_ctx.curr->FuncCode == FUNC_WR_MULREG))
            {//写寄存器
                RTU_Write(&rtu_ctx);
            }
            else
            {

            }
            rtu_ctx.TOtimer = rtu_ctx.guard_time;//每次发送请求时设置超时时间
            rtu_ctx.fsm_state = RTU_WAITRESP;
            break;
    case RTU_WAITRESP:
        if(rtu_ctx.event == EV_TO)
        {//发送的命令没有收到响应
            rtu_ctx.fsm_state = RTU_REQ;
            rtu_ctx.curr->Status=EXCUTE_FAIL;
            rtu_ctx.TOtimer = rtu_ctx.poll_interval;
            if (rtu_ctx.curr->Excute_Num > 1u)
            {//判断是否有重发的配置，0表示需要周期发送的请求，非0表示需要发送的次数
                rtu_ctx.curr->Excute_Num--;
                RTU_AddReqBlock(&rtu_ctx,rtu_ctx.curr);
            }
            else if(rtu_ctx.curr->Excute_Num == 0u)
            {
                RTU_AddReqBlock(&rtu_ctx,rtu_ctx.curr);
            }
            else
            {

            }
            Error_Indicator(80);  //LED亮80ms后灭
        }
        else if(rtu_ctx.event == EV_RX_OK)
        {//正常收到从节点的响应数据
            RTU_HandleReply(&rtu_ctx);
            rtu_ctx.fsm_state = RTU_REQ;
            rtu_ctx.TOtimer = rtu_ctx.poll_interval;
            if (rtu_ctx.curr->Excute_Num > 1u)
            {//判断是否有重发的配置，0表示需要周期发送的请求，非0表示需要发送的次数
                rtu_ctx.curr->Excute_Num--;
                RTU_AddReqBlock(&rtu_ctx,rtu_ctx.curr);
            }
            else if(rtu_ctx.curr->Excute_Num == 0u)
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
        break;
        default:break;
    }
    rtu_ctx.event = EV_NONE;
}

/********************************************************************************/
/*函数名：  Task_MBRTU_Master                                                   */
/*功能说明：RTU master主task                                                    */
/*输入参数：无                                                                  */
/*输出参数：无                                                                  */
/*******************************************************************************/
void Task_MBRTU_Master(void *p_arg)
{
    RTU_Init(POLLING_INTERVAL,REQUIRE_TIMEOUT);//第一个参数表示发送间隔时间，第二个参数表示超时时间
	while(1)
	{    	
    	RTU_CyclicTask();
        OSTimeDlyHMSM(0, 0, 0, 1);
	}
}

