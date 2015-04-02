#include "tim6.h"
#include "main.h"

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
    tim.TIM_Period = 50000;
    TIM_TimeBaseInit(TIM6,&tim);
}

void TIM6_Start(void)
{
    TIM_Cmd(TIM6, ENABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
}

//50ms 控制一次
void TIM6_DAC_IRQHandler(void)  
{
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

/////////////////////////////////////////
u8 SYS_INIT_OK = 0;
u32 timer2_clock;
u16 timer2_clock1;
u16 timer2_clock2;
u32 timer2_clock3;
u32 timer2_clock4;
void Timer2_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	 
	TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM2, //TIM2
		TIM_IT_Update  |  //TIM 中断源
		TIM_IT_Trigger,   //TIM 触发中断源 
		ENABLE  //使能
		);
		 


	TIM_Cmd(TIM2, ENABLE);  //使能TIMx外设
							 
}

void  TIM2_IRQHandler(void)   //TIM4中断
{
if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 	
		timer2_clock++;
		timer2_clock1++;
		timer2_clock2++;

    if(!SYS_INIT_OK) return;

		if(timer2_clock1>0)				//每2次中断执行一次,1ms
		{
			timer2_clock1=0;
//			Prepare_Data();
		}

		if(timer2_clock2>1)				//每4次中断执行一次,2ms
		{
		    
			timer2_clock2=0;
//			Get_Attitude();		//姿态计算	
			Gimbal_Control(target_pitch_angle,target_yaw_angle);
		}
  }
}
