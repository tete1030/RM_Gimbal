#ifndef __TIMER_H__
#define __TIMER_H__

#include <stdint.h>

void Timer_Configuration(void);
void Timer_Start(void);
int8_t Timer_Register(uint32_t interval, void (*callback)(void));
void Timer_Unregister(int8_t index);
int8_t Timer_Setup_Task(uint32_t delay, void (*callback)(void));

#endif
