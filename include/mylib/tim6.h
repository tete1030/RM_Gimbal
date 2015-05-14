#ifndef __TIM6_H__
#define __TIM6_H__

#include <stm32f4xx.h>

extern int16_t tv_201;
extern int16_t tp_201;
extern int16_t tp_203;
extern int32_t speed_201;
extern int32_t speed_203;

void TIM6_Configuration(void);
void TIM6_Start(void);

#endif
