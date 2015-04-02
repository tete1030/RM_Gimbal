#ifndef __TIM6_H__
#define __TIM6_H__

#include <stm32f4xx.h>

void TIM6_Configuration(void);
void TIM6_Start(void);

extern u8 SYS_INIT_OK;
extern u32 timer2_clock;
void Timer2_Init(u16 arr,u16 psc);   

#endif
