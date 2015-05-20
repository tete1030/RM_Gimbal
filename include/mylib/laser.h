#ifndef __LASER_H__
#define __LASER_H__

#include <stm32f4xx.h>


#define LASERON() GPIO_SetBits(GPIOA,GPIO_Pin_1);
#define LASEROFF() GPIO_ResetBits(GPIOA,GPIO_Pin_1);
#define L_PWM  TIM4->CCR4
#define H_PWM  TIM4->CCR3

void Laser_Configuration(void);
static inline void Laser_On(void)
{
    GPIO_SetBits(GPIOA,GPIO_Pin_1);
}

static inline void Laser_Off(void)
{
    GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}

#endif 
