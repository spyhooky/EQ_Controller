#ifndef __DR_GPIO_H
#define __DR_GPIO_H
#include "stm32f10x.h"


typedef struct IO_REG_Init_t
{
    GPIO_TypeDef* GPIO_PORT;
    u16 GPIO_PIN;
}IO_REG_Init;

typedef struct IO_REG_GROUP_t
{
    IO_REG_Init Input_Init[DIGIT_INPUT_CHN_NUM];
    u8 (*_INPUT_STS) (GPIO_TypeDef* port, u16 pin);
    u8 (*_SET_STS) (GPIO_TypeDef* port, u16 pin);
    u8 (*_RESET_STS) (GPIO_TypeDef* port, u16 pin);
}IO_REG_GROUP;

#define DIP_SWITCH_REG_PORT(n)          DIP_SWITCH_REG.Input_Init[n].GPIO_PORT
#define DIP_SWITCH_REG_PIN(n)           DIP_SWITCH_REG.Input_Init[n].GPIO_PIN
#define GET_DIP_SWITCH_STATUS(n)        DIP_SWITCH_REG._INPUT_STS(DIP_SWITCH_REG_PORT(n),DIP_SWITCH_REG_PIN(n))

#define DIGIT_INPUT_REG_PORT(n)         DIGIT_INPUT_REG.Input_Init[n].GPIO_PORT
#define DIGIT_INPUT_REG_PIN(n)          DIGIT_INPUT_REG.Input_Init[n].GPIO_PIN
#define GET_DIGIT_INPUT_STATUS(n)       DIGIT_INPUT_REG._INPUT_STS(DIGIT_INPUT_REG_PORT(n),DIGIT_INPUT_REG_PIN(n))

#define DIGIT_OUTPUT_REG_PORT(n)        DIGIT_OUTPUT_REG.Input_Init[n].GPIO_PORT
#define DIGIT_OUTPUT_REG_PIN(n)         DIGIT_OUTPUT_REG.Input_Init[n].GPIO_PIN
#define SET_DIGIT_OUTPUT_STATUS(n,m)    \
    { if(m==0) {DIGIT_OUTPUT_REG._RESET_STS(DIGIT_OUTPUT_REG_PORT(n),DIGIT_OUTPUT_REG_PIN(n));}\
        else {DIGIT_OUTPUT_REG._SET_STS(DIGIT_OUTPUT_REG_PORT(n),DIGIT_OUTPUT_REG_PIN(n)); }\




extern IO_REG_GROUP DIP_SWITCH_REG;
extern IO_REG_GROUP DIGIT_INPUT_REG;
extern IO_REG_GROUP DIGIT_OUTPUT_REG;


void DrGpioInit( void );
void EXIT9_5_IRQ(void);



#endif


