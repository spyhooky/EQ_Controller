#ifndef __DR_TIMER_H
#define __DR_TIMER_H	  



/* 中断服务程序 */
void TIM2_IRQ(void);
void TIM3_IRQ(void);
void TIM5_IRQ(void);
    

/* 短延时，初始化使用 */
void Delay1ms(uint16_t  l_u8Count);


/***************************初始化*******************************/
void DrTimer_Init(void);


#endif


