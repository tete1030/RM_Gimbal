#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif


#define ITP_SYSTICK_PREEMPTION 0
#define ITP_SYSTICK_SUB 0

#define ITP_CAN1_RX0_PREEMPTION 2
#define ITP_CAN1_RX0_SUB 1

#define ITP_CAN1_TX_PREEMPTION 2
#define ITP_CAN1_TX_SUB 2

#define ITP_CAN2_RX0_PREEMPTION 2
#define ITP_CAN2_RX0_SUB 1

#define ITP_CAN2_TX_PREEMPTION 2
#define ITP_CAN2_TX_SUB 2

#define ITP_MPU_EXTI9_5_PREEMPTION 1
#define ITP_MPU_EXTI9_5_SUB 0


#endif /* __STM32F4xx_IT_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
