#include "tim6.h"
#include "can1.h"
#include "app.h"

void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);

    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 84-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000;
    TIM_TimeBaseInit(TIM6,&tim);
}

void TIM6_Start(void)
{
    TIM_Cmd(TIM6, ENABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
}

int16_t tv_201 = 0;
int16_t tp_201 = 3200;
int16_t tp_203 = 1800;

int32_t last_201 = 0;
int32_t last_202 = 0;
int32_t last_203 = 0;

int32_t speed_201;
int32_t speed_203;

//50ms 控制一次
void TIM6_DAC_IRQHandler(void)  
{
	int16_t output_201, output_203;
	if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET)
	{
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
		speed_201 = (0x2000 + (int32_t)encoder_201 - last_201) % 0x2000;
		last_201 = encoder_201;
		if(speed_201 & 0x1000) speed_201 = speed_201 - 0x2000;

		speed_203 = (0x2000 + (int32_t)encoder_203 - last_203) % 0x2000;
		last_203 = encoder_203;
		if(speed_203 & 0x1000) speed_203 = speed_203 - 0x2000;

		output_201 = (int16_t)Position_Control_201(encoder_201, tp_201);
		output_203 = (int16_t)Position_Control_203(encoder_203, tp_203);
		Cmd_ESC(output_201, 0, output_203);
		//printf("						%d\t%d\r\n",speed_201, output);

	}
//    if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
//	{
//        TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
//        TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
//        
//        Set_Fuyang(2000,Fuyang_Number);
//        Set_Xuanzhuan(4000,Xuanzhuan_Number);
//        
//        Fuyang_Angle = Encoder_TIM8_Get_CNT();
//        LCD12864_Printf(0,4,"%2.2f",(Fuyang_Angle-385)/20.0+30.0);
//    }
}
