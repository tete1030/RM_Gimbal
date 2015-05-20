#include "main.h"
#define pitch_offset 0.0
#define yaw_offset 0.0
void MPU6050_Interrupt_Configuration(void)
{
    GPIO_InitTypeDef    gpio;
    NVIC_InitTypeDef    nvic;
    EXTI_InitTypeDef    exti;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,  ENABLE);   
 
	gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);
    
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,GPIO_PinSource5); 
    
    exti.EXTI_Line = EXTI_Line5;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿中断
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    
    nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

float last_201_angle = 0.0;       //上次pitch轴的角度
float this_201_angle = 0.0;       //本次pitch轴的角度
float rate_201_angle = 0.0;       //pitch轴角速度 = 本次pitch轴角度 - 上次pitch轴角度
float velocity_201_output = 0.0;  //pitch轴速度环函数输出值
float position_201_output = 0.0;  //pitch轴位置环函数输出值


float last_202_angle = 0.0;       //上次yaw轴的角度
float this_202_angle = 0.0;       //本次yaw轴的角度
float rate_202_angle = 0.0;       //yaw轴角速度 = 本次yaw轴角度 - 上次yaw轴角度
float velocity_202_output = 0.0;  //yaw轴速度环函数的输出值
float position_202_output = 0.0;  //yaw轴位置环函数的输出值

#define BUF_NUM 10						//编码器：10组滤波，600 ; mpu:10 组滤波

float rate_201_temp[BUF_NUM] = {0};
float rate_201 = 0;

unsigned char buf_cnt = 0;
unsigned int encoder_cnt = 0;

volatile unsigned char InitFlag = 0;
unsigned char TimeCnt = 0;


void Send_angle(float encoder_angle)
{
    uint8_t Data[4];
	  encoder_angle = encoder_angle * 100.0; 
    Data[0] = (uint8_t)((int32_t)encoder_angle >>24);
    Data[1] = (uint8_t)((int32_t)encoder_angle >>16);
    Data[2] = (uint8_t)((int32_t)encoder_angle >>8);
    Data[3] = (uint8_t)((int32_t)encoder_angle);
	  //RS232_VisualScope(USART1, Data, 8);
}

//MPU6050 外部中断处理函数
void EXTI9_5_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line5) == SET)
    {
        
        EXTI_ClearFlag(EXTI_Line5);          //清除标志位
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}
