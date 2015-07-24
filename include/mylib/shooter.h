//
// Created by Texot Qexoq on 5/19/15.
//

#ifndef __SHOOTER_H__
#define __SHOOTER_H__

#include "friction.h"
#include "main.h"

#if defined CAR_1
#define SHOOTER_ON_PWM 300
#elif defined CAR_2
#define SHOOTER_ON_PWM 30
#elif defined CAR_3
#define SHOOTER_ON_PWM 150
#endif

typedef enum Shooter_State
{
    Shooter_State_Off,
    Shooter_State_On,
    Shooter_State_On_Ready
} Shooter_State;

volatile extern Shooter_State shooter_state;

void Shooter_Configuration(void);

static inline void Shooter_Start(void)
{
    // TODO: Check if friction ready
    //GPIO_SetBits(GPIOB,GPIO_Pin_8);

    shooter_state = Shooter_State_On_Ready;
    if(Friction_Get_State() == Friction_State_On) {
        shooter_state = Shooter_State_On;
        TIM4->CCR3 = SHOOTER_ON_PWM;
    }
}

static inline void Shooter_Stop(void)
{
    //GPIO_ResetBits(GPIOB,GPIO_Pin_8);
    shooter_state = Shooter_State_Off;
    TIM4->CCR3 = 0;

}

#endif //__SHOOTER_H__
