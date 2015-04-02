//#include "main.h"

//void MPU6050_Interrupt_Configuration(void)
//{
//    GPIO_InitTypeDef    gpio;
//    NVIC_InitTypeDef    nvic;
//    EXTI_InitTypeDef    exti;

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,  ENABLE);   
// 
//	gpio.GPIO_Pin = GPIO_Pin_5;
//    gpio.GPIO_Mode = GPIO_Mode_IN;
//    gpio.GPIO_OType = GPIO_OType_PP;
//    gpio.GPIO_PuPd = GPIO_PuPd_UP;
//    gpio.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &gpio);
//    
//    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,GPIO_PinSource5); 
//    
//    exti.EXTI_Line = EXTI_Line5;
//    exti.EXTI_Mode = EXTI_Mode_Interrupt;
//    exti.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿中断
//    exti.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&exti);
//    
//    nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 0;
//    nvic.NVIC_IRQChannelSubPriority = 0;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);
//}

//float last_201_angle = 0.0;       //上次pitch轴的角度
//float this_201_angle = 0.0;       //本次pitch轴的角度
//float rate_201_angle = 0.0;       //pitch轴角速度 = 本次pitch轴角度 - 上次pitch轴角度
//float velocity_201_output = 0.0;  //pitch轴速度环函数输出值
//float position_201_output = 0.0;  //pitch轴位置环函数输出值


//float last_203_angle = 0.0;       //上次yaw轴的角度
//float this_203_angle = 0.0;       //本次yaw轴的角度
//float rate_203_angle = 0.0;       //yaw轴角速度 = 本次yaw轴角度 - 上次yaw轴角度
//float velocity_203_output = 0.0;  //yaw轴速度环函数的输出值
//float position_203_output = 0.0;  //yaw轴位置环函数的输出值

//#define BUF_NUM 10						//编码器：10组滤波，600 ; mpu:10 组滤波

//float rate_201_temp[BUF_NUM] = {0};
//float rate_201 = 0;

//unsigned char buf_cnt = 0;
//unsigned int encoder_cnt = 0;

//volatile unsigned char InitFlag = 0;
//unsigned char TimeCnt = 0;


////MPU6050 外部中断处理函数
//void EXTI9_5_IRQHandler(void)
//{
//    if(EXTI_GetITStatus(EXTI_Line5) == SET)
//    {
//        float sum = 0;
//        int i;
//        
//        MPU6050_ReadData();                 //读取MPU6050数据


//        last_201_angle = this_201_angle;
//        this_201_angle = -(encoder_201 - encoder_bias_201) * 0.044;       //从陀螺仪那边取出角度;cyq:转化为欧拉角
//        rate_201_angle = this_201_angle - last_201_angle;   //得到201当前的速度
//        
//        last_203_angle = this_203_angle;
//        this_203_angle = (encoder_202 - encoder_bias_202) * 0.044;                       //从编码器那边取出角度，暂时这么控制//cyq：换成202的编码器
//        rate_203_angle = this_203_angle - last_203_angle;   //得到203当前的速度
//                
//        rate_201_temp[buf_cnt++] = rate_201_angle;//MPU6050_Real_Data.Gyro_X;//滤波针对开枪的时候
//        if(buf_cnt == BUF_NUM)
//        {
//            buf_cnt = 0;
//        }							
//        for(i = 0;i < BUF_NUM;i++)
//        {
//            sum += rate_201_temp[i];
//        }
//        rate_201 = sum/BUF_NUM;    
//            
//        if(encoder_cnt < 10000)//初始化阶段
//        {
//            LED2_TOGGLE();//red
//            
//            position_201_output = Position_Control_201(this_201_angle,target_pitch_angle);  //计算位置闭环输出量，传给速度闭环 cyq:MPU6050_Angle.Pitch 增稳
//            velocity_201_output = Velocity_Control_201(MPU6050_Real_Data.Gyro_X,position_201_output);
//           
//            position_203_output = Position_Control_203(this_203_angle,target_yaw_angle);    //先锁定在初始位置，待陀罗稳定
//            velocity_203_output = Velocity_Control_203(MPU6050_Real_Data.Gyro_Z,position_203_output); 

//            Cmd_ESC((int16_t)velocity_201_output,(int16_t)velocity_203_output,(int16_t)velocity_203_output);//cyq:输出给到pitch yaw电机
//            
//            if(encoder_cnt == 4000){
//                GYRO_RST();
//            }
//            encoder_cnt++;
//        }
//        else if(encoder_cnt >= 10000)//初始化完成
//        {
//            LED1_TOGGLE();//green
//            position_203_output = Position_Control_203(YAW_Angle,target_yaw_angle);    //计算位置闭环输出量，传给速度闭环  - dipan_gyro_angle
//            velocity_203_output = Velocity_Control_203(MPU6050_Real_Data.Gyro_Z,position_203_output);
//            
//            
//            if(shooting_flag == 1)	//shoot
//                {	
//                    position_201_output = Position_Control_201(this_201_angle,target_pitch_angle);  //计算位置闭环输出量，传给速度闭环 cyq:MPU6050_Angle.Pitch this_201_angle
//                    velocity_201_output = Velocity_Control_201(rate_201 * 600,position_201_output); //计算速度闭环输出量, 使用编码器滤波后的数据来做速度环					
//            
//                }
//            else 
//                {
//                    position_201_output = Position_Control_201(this_201_angle,target_pitch_angle);  //计算位置闭环输出量，传给速度闭环 cyq:MPU6050_Angle.Pitch 增稳
//                    velocity_201_output = Velocity_Control_201(MPU6050_Real_Data.Gyro_X,position_201_output); 
//                }
//                
//            if(OverCurr_flag)
//            {
//                Cmd_ESC(0,0,0);
//                Encoder_sent(0); //如果电流保护后就不再将误差传递到底盘           
//            }
//            else 
//            {
//                Cmd_ESC((int16_t)velocity_201_output,(int16_t)velocity_203_output,(int16_t)velocity_203_output);//cyq:输出给到202电机
//                //如果电流保护后就不再将误差传递到底盘
//                Encoder_sent(this_203_angle);//将码盘的偏差传递到底盘            
//            }
//            
//        }     
//        EXTI_ClearFlag(EXTI_Line5);          //清除标志位
//        EXTI_ClearITPendingBit(EXTI_Line5);
//    }
//}
