#include "stm32f4xx.h"
#include "delay.h"
#include "friction.h"
#include "ticker.h"
#include "timer.h"
#include "shooter.h"
#include "laser.h"
#include "main.h"


/*-LEFT---(PB3---TIM2_CH2)--*/
/*-RIGHT--(PA15--TIM2_CH1)--*/

#define PWM_COUNTER_VALUE 20000
#define FRICTION_INIT_PWM 10000
#define FRICTION_MAX_PWM 2000
#define FRICTION_MIN_PWM 1000
#if defined CAR_1
#define FRICTION_ON_PWM 1500
#elif defined CAR_2
#define FRICTION_ON_PWM 1300
#elif defined CAR_3
#define FRICTION_ON_PWM 1350
#endif
#define FRICTION_OFF_PWM 1000

volatile Friction_State friction_state = Friction_State_Prepare;


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

void Friction_Init(Friction_Start_Mode fsm)
{
	uint32_t ms_tick = 0;
	uint64_t end_tick = 0;
    PWM_Configuration();

    if(friction_state != Friction_State_Prepare) return;
    //TIM2->CCR1 = TIM2->CCR2 = FRICTION_INIT_PWM;
    //delay_ms(20);
    ms_tick = (uint32_t) Ticker_Get_MS_Tickcount();

    if(fsm == Friction_Start_Mode_Normal)
    {
    	end_tick = (uint64_t)ms_tick * 4000;
        TIM2->CCR1 = TIM2->CCR2 = FRICTION_OFF_PWM;
        while(end_tick > Ticker_Get_Tick());
        friction_state = Friction_State_Off;
    }
    else if(fsm == Friction_Start_Mode_Set_Range)
    {
        friction_state = Friction_State_Set_Range;
        // 油门行程
        TIM2->CCR1 = TIM2->CCR2 = FRICTION_MAX_PWM;
        delay_ms(2000);
        TIM2->CCR1 = TIM2->CCR2 = FRICTION_MIN_PWM;
        delay_ms(2000);
        friction_state = Friction_State_Off;
    }
    else if(fsm == Friction_Start_Mode_Mannual_Control)
    {
        friction_state = Friction_State_Mannual_Control;
    }


}

void Friction_Turning_On_Callback()
{
    // TODO: require top interrupt priority(disable interrupt?)

    if(friction_state == Friction_State_Turning_On) {
        friction_state = Friction_State_On;

        if(shooter_state == Shooter_State_On_Ready)
            Shooter_Start();
    }
}

void Friction_Turning_Off_Callback()
{
    // TODO: require top interrupt priority(disable interrupt?)
    if(friction_state == Friction_State_Turning_Off) {
        friction_state = Friction_State_Off;
        TIM2->CCR1 = TIM2->CCR2 = FRICTION_OFF_PWM;
    }
}

void Friction_Set_Enable(uint8_t enable)
{
    if(enable && friction_state == Friction_State_Off) {
        friction_state = Friction_State_Turning_On;
        //last_friction_on_tick = Ticker_Get_Tick();
        TIM2->CCR1 = TIM2->CCR2 = FRICTION_ON_PWM;
        Laser_On();
        Timer_Setup_Task(500, Friction_Turning_On_Callback);
    }
    else if(!enable && (friction_state == Friction_State_On || friction_state == Friction_State_Turning_On)) {
        friction_state = Friction_State_Turning_Off;
        Laser_Off();
        if(shooter_state == Shooter_State_On) {
            Shooter_Stop();
            shooter_state = Shooter_State_On_Ready;
        }
        Timer_Setup_Task(500, Friction_Turning_Off_Callback);
    }
}

Friction_State Friction_Get_State()
{
    return friction_state;
}


void Friction_Set_Remoter_Value(uint16_t value)
{
    if(friction_state == Friction_State_Mannual_Control) {
        if(value > 1684) value = 1684;
        else if(value < 1024) value = 1024;
        TIM2->CCR1 = TIM2->CCR2 = 1000 + ((value - 1024) * 1000 / 660);
    }

}
