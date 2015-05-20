//
// Created by Texot Qexoq on 5/19/15.
//

#ifndef __SHOOTER_H__
#define __SHOOTER_H__

void Shooter_Configuration(void);

static inline void Shooter_Start(void)
{
    //GPIO_SetBits(GPIOB,GPIO_Pin_8);
    GPIO_ResetBits(GPIOB,GPIO_Pin_9);
}

static inline void Shooter_Stop(void)
{
    //GPIO_SetBits(GPIOB,GPIO_Pin_8);
    GPIO_SetBits(GPIOB,GPIO_Pin_9);
}

#endif //__SHOOTER_H__
