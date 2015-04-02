#include "main.h"
#include "tim6.h"
#include "imu.h"


int main(void)
{
	  int32_t flag; 
    SYS_INIT_OK=0;
	  SystemInit();		
    NVIC_Configuration();	
    LED_Configuration(); 
	  
	LED1_ON();
	  LED2_ON();
	while(1)
    {
//		  Cmd_ESC(1000,-500,1000);
			delay_ms(1000);
////			
//			if (target_yaw_angle>50.0) 
//			flag=0;
//			if (target_yaw_angle<-50.0) 
//			flag=1;
//			
//			if (flag==1)
//			target_yaw_angle+=5.0;
//			if (flag==0)
//			target_yaw_angle-=5.0;
			
			LED2_TOGGLE();		
    }
    
	  CAN1_Configuration();//云台控制CAN总线
    CAN2_Configuration();//
	  USART1_Configuration(); 
    LED_Configuration(); 
    MPU6050_Initialization();   
    MPU6050_Gyro_Offset();
    Timer2_Init(9,8399);    /////// 168Mhz/10/8400=2000khz 0.5ms一次 	
	  SYS_INIT_OK=1;
	  delay_ms(500);
	  LED2_OFF();
//    target_yaw_angle=10;
//	  flag=1;
	  while(1)
    {
//		  Cmd_ESC(1000,-500,1000);
			delay_ms(1000);
////			
//			if (target_yaw_angle>50.0) 
//			flag=0;
//			if (target_yaw_angle<-50.0) 
//			flag=1;
//			
//			if (flag==1)
//			target_yaw_angle+=5.0;
//			if (flag==0)
//			target_yaw_angle-=5.0;
			
			LED1_TOGGLE();		
    }
}


