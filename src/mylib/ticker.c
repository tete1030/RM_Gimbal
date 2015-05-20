#include "ticker.h"
#include "stm32f4xx_it.h"

#define TICKER_CYCLE_COUNT 0x01000000

uint64_t ticker = 0;
uint32_t ms_tickcount = 0;

void Ticker_Configuration(void)
{
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = SysTick_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = ITP_SYSTICK_PREEMPTION;
    nvic.NVIC_IRQChannelSubPriority = ITP_SYSTICK_SUB;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    SysTick->LOAD = TICKER_CYCLE_COUNT - 1;
    SysTick->CTRL = (SysTick->CTRL & 0xfffffff8) | 3; // div8, interrupt, enable

    if(SysTick->CALIB & SysTick_CALIB_NOREF_Msk)
        ms_tickcount = SystemCoreClock / 8000;
    else
        ms_tickcount = SysTick->CALIB & SysTick_CALIB_TENMS_Msk;
}

inline uint64_t Ticker_Get_Tick(void)
{
    return ticker + TICKER_CYCLE_COUNT - SysTick->VAL;
}

inline uint32_t Ticker_Get_MS_Tickcount()
{
    return ms_tickcount;
}

void SysTick_Handler(void)
{
    ticker += TICKER_CYCLE_COUNT;
}

