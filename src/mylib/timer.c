#include "timer.h"
#include "stm32f4xx.h"

#define TIMER_TASK_MAX_COUNT 10

typedef struct Timer_Callback
{
    uint8_t active;
    uint32_t start_timer;
    uint32_t interval_ms;
    uint8_t one_time;
    void (*callback)(void);
} Timer_Callback;

Timer_Callback tc_list[TIMER_TASK_MAX_COUNT];
uint8_t tc_count = 0;

uint32_t timer_offset = 0;
uint32_t timer_counter = 0;

void Timer_Configuration(void)
{
	uint8_t i;
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);

    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000;
    TIM_TimeBaseInit(TIM6,&tim);

    for(i = 0; i < TIMER_TASK_MAX_COUNT; i++)
    {
    	tc_list[i].active = 0;
    	tc_list[i].interval_ms = 0;
    	tc_list[i].callback = 0;
    	tc_list[i].start_timer = 0;
    	tc_list[i].one_time = 0;
    }
}

void Timer_Start(void)
{
    TIM_Cmd(TIM6, ENABLE);
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);

}

int8_t Timer_Register(uint32_t interval_ms, void (*callback)(void))
{
    uint8_t index;

    if(tc_count >= TIMER_TASK_MAX_COUNT)
    {
        for(index = 0; index < TIMER_TASK_MAX_COUNT; index ++)
        {
            if(tc_list[index].active == 0) break;
        }
    }
    else
        index = tc_count;

    if (index >= TIMER_TASK_MAX_COUNT) return -1;


    tc_list[index].start_timer = timer_counter + interval_ms + 1;
    tc_list[index].interval_ms = interval_ms;
    tc_list[index].callback = callback;
    tc_list[index].one_time = 0;
    tc_list[index].active = 1;

    if(tc_count == index) tc_count ++;

    return (int8_t)(index);
}

void Timer_Unregister(int8_t index)
{
    if(index == -1) return;
    tc_list[index].active = 0;
    if(index + 1 == tc_count) tc_count--;
}

int8_t Timer_Setup_Task(uint32_t delay, void (*callback)(void))
{
    int8_t ret;
    ret = Timer_Register(delay, callback);
    if(ret >= 0) tc_list[ret].one_time = 1;
    return ret;
}

void TIM6_DAC_IRQHandler(void)  
{
	uint8_t i;
    uint8_t tc_count_tmp;
    int32_t timer_diff;
	if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

        tc_count_tmp = tc_count;
        timer_counter++;

        for (i = 0; i < tc_count_tmp; i++) {
            if (tc_list[i].active) {
                timer_diff = timer_counter - tc_list[i].start_timer;
                if(timer_diff >= 0 && (timer_diff % tc_list[i].interval_ms) == 0) {
                    tc_list[i].callback();
                    if(tc_list[i].one_time) Timer_Unregister(i);

                }
            }
        }
    }
}
