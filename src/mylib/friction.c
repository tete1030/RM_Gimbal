#include "stm32f4xx.h"
#include "delay.h"
#include "friction.h"
#include "ticker.h"
#include "timer.h"

/*-LEFT---(PB3---TIM2_CH2)--*/
/*-RIGHT--(PA15--TIM2_CH1)--*/

#define PWM_COUNTER_VALUE 2500
#define FRICTION_MAX_PWM 2000
#define FRICTION_MIN_PWM 500
#define FRICTION_ON_PWM 500
#define FRICTION_OFF_PWM 300

uint64_t last_friction_on_tick = 0;

uint8_t friction_state = 0;

void PWM_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB,&gpio);
    
    gpio.GPIO_Pin = GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA,&gpio);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource3, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_TIM2);    
    
    tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = PWM_COUNTER_VALUE;   //2.5ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2,&tim);
    
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM2,&oc);
    TIM_OC2Init(TIM2,&oc);
    
    TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM2,ENABLE);

    TIM_Cmd(TIM2,ENABLE);

}

void Friction_Configuration(void)
{
    PWM_Configuration();
}

void Friction_Ready_Callback(uint8_t timer_id)
{
    friction_state = 2;
    Timer_Unregister(timer_id);
}

void Friction_Set_Enable(uint8_t enable)
{
    if(enable && friction_state == 0) {
        friction_state = 1;
        last_friction_on_tick = Ticker_Get_Tick();
        TIM2->CCR1 = TIM2->CCR2 = FRICTION_ON_PWM;
        Timer_Register(1000, Friction_Ready_Callback);
    }
    else {
        friction_state = 0;
        TIM2->CCR1 = TIM2->CCR2 = FRICTION_OFF_PWM;
    }
}

uint8_t Friction_Get_State()
{
    return friction_state;
}

void Friction_Init_Speed_Controller()
{
    // TODO: correct process
    TIM2->CCR1 = TIM2->CCR2 = FRICTION_MAX_PWM;
    delay_ms(5000);
    TIM2->CCR1 = TIM2->CCR2 = FRICTION_MIN_PWM;
    delay_ms(5000);
}

