#ifndef __CAN_H__
#define __CAN_H__

#include <stm32f4xx.h>


void CAN1_Configuration(void (*send_handler)(uint16_t, int8_t), void (*receive_handler)(CanRxMsg*), uint16_t IdHigh, uint16_t IdHighMask, uint16_t IdLow, uint16_t IdLowMask);
int8_t CAN1_Transmit(uint16_t id, uint16_t addr, char* data, uint8_t size);
int8_t CAN1_AsyncTransmit(uint16_t id, uint16_t addr, char* data, uint8_t size);


#endif 
