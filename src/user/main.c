#include "main.h"
#include "app.h"
#include "tim6.h"
#include "report.h"

//改进方向：加速度滤波，在四元数中增加加速度的校正比重
//遥控器指令平滑处理

char id[3];
unsigned char USART_BUF[24] = {0};

int main(void)
{
    int i = 0, j = 0, sum;
    BSP_Init();

    //printf("System Start!\r\n");  //测试串口输出信息

    LED1_ON();
    //LED2_ON();

    Do_Report_MPU();

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


    TIM6_Configuration();
    TIM6_Start();
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
    tp_201 = 3200;
    tp_203 = 1800;
    delay_ms(2000);
    for(i = 3200; i >= 2300; i-= 20)
    {
    	tp_201 = i;
    	delay_ms(20);
    }
    for(i = 2300; i <= 3500; i+= 20)
	{
		tp_201 = i;
		delay_ms(20);
	}

    for(i = 3500; i >= 3200; i-= 20)
	{
		tp_201 = i;
		delay_ms(20);
	}

    for(i = 1800; i >= 1300; i-= 10)
    {
    	tp_203 = i;
    	delay_ms(10);
    }
    for(i = 1300; i <= 2300; i+= 10)
	{
		tp_203 = i;
		delay_ms(10);
	}
    for(i = 2300; i >= 1800; i-= 10)
	{
		tp_203 = i;
		delay_ms(10);
	}

    while(1);

    while(1)
    {
    	//target_velocity_201 = 20;
    	//target_position_201 = 3200;
    	printf("%d, %d, %d, %d\r\n", encoder_201, current_201, set_current_201, hall_state_201);
    	printf("%d, %d, %d, %d\r\n\r\n", encoder_203, current_203, set_current_203, hall_state_203);
    	LED1_TOGGLE();
    	delay_ms(50);
    }

    MPU6050_Gyro_calibration();
    
    MPU6050_Interrupt_Configuration(); //MPU6050中断初始化
    
    PWM_Configuration();        //输出一组PWM测试
    PWM1 = 1000;
    PWM2 = 1000;
    
    delay_ms(1000);
    
    //BUZZER_ON();
    delay_ms(50);
    //BUZZER_OFF();
    delay_ms(1000);

    //BUZZER_ON();
    delay_ms(50);
    //BUZZER_OFF();
    delay_ms(1000);

    //BUZZER_ON();
    delay_ms(50);
    //BUZZER_OFF();
    
//    PWM1 = 1900;
//    PWM2 = 1900;

//		Motor_Reset(MOTOR_NUM1);
//		
//		delay_ms(30);//延时
//		
//		Motor_Init(MOTOR_NUM1,PWM_MODE);

//		delay_ms(30);	

		


    while(1)
    {
        CurrentProtect();//电调电流保护
//			  LASEROFF();
//			  delay_ms(100);
//			  LASERON();
//			  delay_ms(100);
      
//        USART_BUF[0]  = (u8)((s16)current_201);//(u8)(Acc.X);
//        USART_BUF[1]  = (u8)((s16)current_201 >> 8);//(u8)(Acc.X >> 8);
//        USART_BUF[2]  = (u8)((s16)CurrInt_201);//(u8)(Acc.Y);
//        USART_BUF[3]  = (u8)((s16)CurrInt_201 >> 8);//(u8)(Acc.Y >> 8);
//        USART_BUF[4]  = (u8)((s16)current_202);//(u8)(Acc.Z);
//        USART_BUF[5]  = (u8)((s16)current_202 >> 8);//(u8)(Acc.Z >> 8);
//        USART_BUF[6]  = (u8)((s16)(CurrInt_202));
//        USART_BUF[7]  = (u8)((s16)(CurrInt_202) >> 8) ;
//        USART_BUF[8]  = 0x00;//(u8)(Gyr.Y);
//        USART_BUF[9]  = 0x00;//(u8)(Gyr.Y >> 8);
//        USART_BUF[10] = 0x00;//(u8)(Gyr.Z);
//        USART_BUF[11] = 0x00;//(u8)(Gyr.Z >> 8);
//        USART_BUF[12] = 0x00;//(u8)(Mag.X);
//        USART_BUF[13] = 0x00;//(u8)(Mag.X >> 8);
//        USART_BUF[14] = 0x00;//(u8)(Mag.Y);
//        USART_BUF[15] = 0x00;//(u8)(Mag.Y >> 8);
//        USART_BUF[16] = 0x00;//(u8)(Mag.Z);
//        USART_BUF[17] = 0x00;//(u8)(Mag.Z >> 8);

//        if(1)
//            RS232_VisualScope(USART1, USART_BUF, 8);
//        else
//            RS232_VisualScope(USART1, USART_BUF+10, 8);
     
    }
}


