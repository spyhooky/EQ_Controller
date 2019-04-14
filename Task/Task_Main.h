#ifndef __TASK_MAIN_H
#define __TASK_MAIN_H

/************����ջ��С����λΪ OS_STK ��************/
#define STK_SIZE_32                             32u
#define STK_SIZE_64                             64u
#define STK_SIZE_128                            128u
#define STK_SIZE_256                            256u
#define STK_SIZE_384                            384u
#define STK_SIZE_512                            512u
#define STK_SIZE_640                            640u
#define STK_SIZE_768                            768u
#define STK_SIZE_896                            896u
#define STK_SIZE_1024                           1024u
#define STK_SIZE_2048                           2048u
#define STK_SIZE_3072                           3072u
#define STK_SIZE_4096                           4096u


/************�����������ȼ�,����(10-50) *************************/
/* #define OS_LOWEST_PRIO   63u */
#define TASK_PRIO_MAIN                          60u
#define TASK_PRIO_PCMSG_RECV                    10u
#define TASK_PRIO_MBRTU_M                       11u
#define TASK_PRIO_FREQ_CONVERT                  12u

#define TASK_PRIO_ENCODER                       17u

#define TASK_PRIO_IO                            41u
#define TASK_PRIO_FLASH                         42u
#define TASK_PRIO_PC_MSG_UPD                    43u
#define TASK_PRIO_BACKGRD                       49u

#define TASK_PRIO_MBTCP_SEND                    50u
#define TASK_PRIO_TCPIP_SEND                    51u
#define	TASK_PRIO_MQTTPUB                       52u
#define	TASK_PRIO_MQTTSUB                       53u
#define	TASK_PRIO_MQTT                          54u
#define TASK_PRIO_TCPIP                         55u
#define TASK_PRIO_MBTCP                         56u
#define TASK_PRIO_HTTP                          57u

/************����Taskջ��С����λΪ OS_STK ��************/
#define STKSIZE_IO                     	 STK_SIZE_64
#define STKSIZE_PCMSG_RECV               STK_SIZE_1024
#define STKSIZE_FREQ_CONVER              STK_SIZE_1024
#define STKSIZE_ENCODER                  STK_SIZE_256
#define STKSIZE_MBRTU_M                  STK_SIZE_256
#define STKSIZE_FREQ_CONVERT             STK_SIZE_256
#define STKSIZE_TCPIP                    STK_SIZE_512
#define STKSIZE_MBTCP                    STK_SIZE_256
#define STKSIZE_MQTT                     STK_SIZE_1024                   
#define STKSIZE_HTTP                     STK_SIZE_512
#define STKSIZE_BACKGRD                  STK_SIZE_1024
#define STKSIZE_PC_MSG_UPD               STK_SIZE_32

#define CYCLE_BACKGROUND                        1u
#define CYCLE_WRIE_POSITION                     (5U*CYCLE_BACKGROUND)
#define CYCLE_READ_ADC_VALUE                    (100U*CYCLE_BACKGROUND)
#define CYCLE_CALC_ENV_TEMP                     (100U*CYCLE_BACKGROUND)
#define CYCLE_CALC_POWER_VOL                    (1000U*CYCLE_BACKGROUND)


#define DETECT_CYCLE                            50u

void Task_Main(void *p_arg);
void ETH2Usartcan_send(u8 uartcan_chn,u8 *databuf,u16 lenth);
void HTTP_DataHandler(void);
void ReadFlashCfg(void);

#endif


