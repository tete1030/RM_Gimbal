//
// Created by Texot Qexoq on 5/19/15.
//
#include "stm32f4xx.h"
#include "shooter.h"

volatile Shooter_State shooter_state = Shooter_State_Off;

void Shooter_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_8;// | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource8, GPIO_AF_TIM4);

    tim.TIM_Prescaler = (84 * 200) - 1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1000;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4,&tim);

    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC3Init(TIM4,&oc);

    TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4,ENABLE);

    TIM_Cmd(TIM4,ENABLE);


    Shooter_Stop();
}

