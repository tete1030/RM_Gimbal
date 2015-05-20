//
// Created by Texot Qexoq on 5/19/15.
//
#include "stm32f4xx.h"
#include "shooter.h"

void Shooter_Configuration(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;	//cyq
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &gpio);
    Shooter_Stop();
}
