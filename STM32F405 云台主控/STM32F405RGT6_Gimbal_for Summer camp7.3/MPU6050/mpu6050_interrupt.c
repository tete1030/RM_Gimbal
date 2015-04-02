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
//    exti.EXTI_Trigger = EXTI_Trigger_Falling;//�½����ж�
//    exti.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&exti);
//    
//    nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 0;
//    nvic.NVIC_IRQChannelSubPriority = 0;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);
//}

//float last_201_angle = 0.0;       //�ϴ�pitch��ĽǶ�
//float this_201_angle = 0.0;       //����pitch��ĽǶ�
//float rate_201_angle = 0.0;       //pitch����ٶ� = ����pitch��Ƕ� - �ϴ�pitch��Ƕ�
//float velocity_201_output = 0.0;  //pitch���ٶȻ��������ֵ
//float position_201_output = 0.0;  //pitch��λ�û��������ֵ


//float last_203_angle = 0.0;       //�ϴ�yaw��ĽǶ�
//float this_203_angle = 0.0;       //����yaw��ĽǶ�
//float rate_203_angle = 0.0;       //yaw����ٶ� = ����yaw��Ƕ� - �ϴ�yaw��Ƕ�
//float velocity_203_output = 0.0;  //yaw���ٶȻ����������ֵ
//float position_203_output = 0.0;  //yaw��λ�û����������ֵ

//#define BUF_NUM 10						//��������10���˲���600 ; mpu:10 ���˲�

//float rate_201_temp[BUF_NUM] = {0};
//float rate_201 = 0;

//unsigned char buf_cnt = 0;
//unsigned int encoder_cnt = 0;

//volatile unsigned char InitFlag = 0;
//unsigned char TimeCnt = 0;


////MPU6050 �ⲿ�жϴ�����
//void EXTI9_5_IRQHandler(void)
//{
//    if(EXTI_GetITStatus(EXTI_Line5) == SET)
//    {
//        float sum = 0;
//        int i;
//        
//        MPU6050_ReadData();                 //��ȡMPU6050����


//        last_201_angle = this_201_angle;
//        this_201_angle = -(encoder_201 - encoder_bias_201) * 0.044;       //���������Ǳ�ȡ���Ƕ�;cyq:ת��Ϊŷ����
//        rate_201_angle = this_201_angle - last_201_angle;   //�õ�201��ǰ���ٶ�
//        
//        last_203_angle = this_203_angle;
//        this_203_angle = (encoder_202 - encoder_bias_202) * 0.044;                       //�ӱ������Ǳ�ȡ���Ƕȣ���ʱ��ô����//cyq������202�ı�����
//        rate_203_angle = this_203_angle - last_203_angle;   //�õ�203��ǰ���ٶ�
//                
//        rate_201_temp[buf_cnt++] = rate_201_angle;//MPU6050_Real_Data.Gyro_X;//�˲���Կ�ǹ��ʱ��
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
//        if(encoder_cnt < 10000)//��ʼ���׶�
//        {
//            LED2_TOGGLE();//red
//            
//            position_201_output = Position_Control_201(this_201_angle,target_pitch_angle);  //����λ�ñջ�������������ٶȱջ� cyq:MPU6050_Angle.Pitch ����
//            velocity_201_output = Velocity_Control_201(MPU6050_Real_Data.Gyro_X,position_201_output);
//           
//            position_203_output = Position_Control_203(this_203_angle,target_yaw_angle);    //�������ڳ�ʼλ�ã��������ȶ�
//            velocity_203_output = Velocity_Control_203(MPU6050_Real_Data.Gyro_Z,position_203_output); 

//            Cmd_ESC((int16_t)velocity_201_output,(int16_t)velocity_203_output,(int16_t)velocity_203_output);//cyq:�������pitch yaw���
//            
//            if(encoder_cnt == 4000){
//                GYRO_RST();
//            }
//            encoder_cnt++;
//        }
//        else if(encoder_cnt >= 10000)//��ʼ�����
//        {
//            LED1_TOGGLE();//green
//            position_203_output = Position_Control_203(YAW_Angle,target_yaw_angle);    //����λ�ñջ�������������ٶȱջ�  - dipan_gyro_angle
//            velocity_203_output = Velocity_Control_203(MPU6050_Real_Data.Gyro_Z,position_203_output);
//            
//            
//            if(shooting_flag == 1)	//shoot
//                {	
//                    position_201_output = Position_Control_201(this_201_angle,target_pitch_angle);  //����λ�ñջ�������������ٶȱջ� cyq:MPU6050_Angle.Pitch this_201_angle
//                    velocity_201_output = Velocity_Control_201(rate_201 * 600,position_201_output); //�����ٶȱջ������, ʹ�ñ������˲�������������ٶȻ�					
//            
//                }
//            else 
//                {
//                    position_201_output = Position_Control_201(this_201_angle,target_pitch_angle);  //����λ�ñջ�������������ٶȱջ� cyq:MPU6050_Angle.Pitch ����
//                    velocity_201_output = Velocity_Control_201(MPU6050_Real_Data.Gyro_X,position_201_output); 
//                }
//                
//            if(OverCurr_flag)
//            {
//                Cmd_ESC(0,0,0);
//                Encoder_sent(0); //�������������Ͳ��ٽ����ݵ�����           
//            }
//            else 
//            {
//                Cmd_ESC((int16_t)velocity_201_output,(int16_t)velocity_203_output,(int16_t)velocity_203_output);//cyq:�������202���
//                //�������������Ͳ��ٽ����ݵ�����
//                Encoder_sent(this_203_angle);//�����̵�ƫ��ݵ�����            
//            }
//            
//        }     
//        EXTI_ClearFlag(EXTI_Line5);          //�����־λ
//        EXTI_ClearITPendingBit(EXTI_Line5);
//    }
//}
