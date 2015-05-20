#ifndef __TICKER_H__
#define __TICKER_H__

#include <stdint.h>

void Ticker_Configuration(void);
uint64_t Ticker_Get_Tick(void);
uint32_t Ticker_Get_MS_Tickcount();

#endif //__TICKER_H__
