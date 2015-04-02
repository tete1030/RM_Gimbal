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

//50ms ����һ��
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

	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	 
	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
		TIM2, //TIM2
		TIM_IT_Update  |  //TIM �ж�Դ
		TIM_IT_Trigger,   //TIM �����ж�Դ 
		ENABLE  //ʹ��
		);
		 


	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx����
							 
}

void  TIM2_IRQHandler(void)   //TIM4�ж�
{
if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 	
		timer2_clock++;
		timer2_clock1++;
		timer2_clock2++;

    if(!SYS_INIT_OK) return;

		if(timer2_clock1>0)				//ÿ2���ж�ִ��һ��,1ms
		{
			timer2_clock1=0;
//			Prepare_Data();
		}

		if(timer2_clock2>1)				//ÿ4���ж�ִ��һ��,2ms
		{
		    
			timer2_clock2=0;
//			Get_Attitude();		//��̬����	
			Gimbal_Control(target_pitch_angle,target_yaw_angle);
		}
  }
}
