#include "main.h"
#include "app.h"
#include "timer.h"
#include "report.h"
#include "led.h"
#include "laser.h"
#include "shooter.h"
#include "maincontrol.h"
#include "gimbal_motor.h"
#include "usart1.h"
#include "usart2.h"
#include "friction.h"
#include "ticker.h"
#include "delay.h"
#include "mpu6050/mpu6050.h"
#include "mpu6050/eMPL/inv_mpu.h"

char id[3];
unsigned char USART_BUF[24] = {0};

int main(void)
{
    int i = 0, j = 0, sum;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Ticker_Configuration();
    delay_init();

    Timer_Configuration();
    Timer_Start();

    LED_Configuration();

    LED1_ON();

    Friction_Configuration();
    Shooter_Configuration();
    Laser_Configuration();

    USART1_Configuration();
    USART2_Configuration();

    delay_ms(200);

    Gimbal_Motor_Configuration();
    Maincontrol_Configuration();

    Gimbal_Start_Standard_Position();

    for(i = 0; i < 10; i++)
    {
        LED1_TOGGLE();
        delay_ms(1000);
    }



    LED1_ON();

    for(i = 0; MPU_Init() && i < 10; i++)
	{
		delay_ms(50);
	}

    Friction_Init_Speed_Controller();
    Friction_Set_Enable(1);


    if(i == 10)
	{
		while(1)
		{
			LED1_TOGGLE();
			delay_ms(100);
		}
	}

    //printf("System Start!\r\n");  //测试串口输出信息

    LED1_ON();
    LED2_ON();

    Gimbal_End_Standard_Position();
    Gimbal_Start_Track_Position();

    //LED2_ON();
    while(1)
    {
    	delay_ms(200);
    	LED1_TOGGLE();
        LED2_TOGGLE();
    }

    //Do_Report_MPU();
/*
    while(MPU6050_Initialization() == 0xff) 
    {
    	delay_ms(10);
        i++;     //如果一次初始化没有成功，那就再来一次
        if(i>10) //如果初始化一直不成功，那就没希望了，进入死循环，蜂鸣器一直叫
        {

            while(1)
            {
                LED1_ON();
                delay_ms(50);
                
            }

        }
    }

    LED1_OFF();
    while(1);


    Timer_Configuration();
    Timer_Start();
    */
    /*
    printf("[\r\n");
    for(i=-5000; i<=5000; i+=50)
    {
    	Cmd_ESC(0, 0, i);
    	delay_ms(500);
    	for(j = 0; j < 100; j++)
    	{
    		sum += speed_203;
    		delay_ms(10);
    	}
    	sum /= 100;
    	printf("%d, %d;\r\n", i, sum);
    }
    printf("]\r\n");
    */


}


