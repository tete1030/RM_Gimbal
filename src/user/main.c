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

    USART1_Configuration();
    USART2_Configuration();

    delay_ms(200);

    Gimbal_Motor_Configuration(); // 使用CAN1，内已经初始化CAN1

    Maincontrol_Configuration(); // 使用CAN2，内已初始化CAN2，并且一定要先初始化CAN1再配置CAN2过滤器

    Maincontrol_Set_Enable_Control(0);

    Friction_Init(Friction_Start_Mode_Normal);
    Shooter_Configuration();
    Laser_Configuration();

    //Friction_Set_Enable(1);

    //Laser_On();

//    while(1)
//    {
//        ///if(Friction_Get_State() == Friction_State_On) {
//            LED1_TOGGLE();
//        //}
//        delay_ms(100);
//    }

//    Friction_Set_Enable(1);
//
//    delay_ms(1000);
//
//    Shooter_Start();
//
    //Laser_On();
//
//    while(1)
//    {
//        printf("pitch: %d, yaw: %d \r\n", gms_pitch.angle, gms_yaw.angle);
//        delay_ms(500);
//    }

    GM_Start_Relative_Control();
    GM_Set_Standard_Position();

#if !(defined CAR_1)
    /*
     * 1. 禁止所有人工操控
     * 2. 打开机械角控制
     * 3. 机械角控制到标准位置
     * 4. 打开平稳检测器,等待到平稳,以下过程中间若有震动,重新执行,LED/蜂鸣器提示
     * 5. 设定当前位置为空间标准位置
     * 6. 等待陀螺仪校准
     * 7. 校准完毕,关闭平稳检测器
     * 7. 释放机械角控制,打开空间角控制
     * 8. 打开所有人工操控
     */

    // 等待稳定
    for(i = 0; i < 5; i++)
    {
        LED1_TOGGLE();
        delay_ms(1000);
    }

    LED1_ON();
    LED2_OFF();


    for(i = 0; MPU_Init() && i < 10; i++)
	{
		delay_ms(50);
	}

    if(i == 10)
    {
        while(1)
        {
            LED1_TOGGLE();
            delay_ms(100);
        }
    }

    delay_ms(8000);

    //printf("System Start!\r\n");  //测试串口输出信息

    LED1_ON();
    LED2_ON();



    GM_Stop_Pitch_Relative_Control();

    GM_Start_Absolute_Control();
#endif
    //LED2_ON();

    Maincontrol_Set_Enable_Control(1);

    while(1)
    {
    	delay_ms(200);
    	LED1_TOGGLE();
        LED2_TOGGLE();
    }



}


