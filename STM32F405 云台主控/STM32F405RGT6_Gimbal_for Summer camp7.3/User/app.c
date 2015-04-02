#include "main.h"
#define BUF_NUM 10						//编码器：10组滤波，600 ; mpu:10 组滤波
#define GAP 1.0
float d_error_201;
/********************************************************************************
   给电调板发送指令，ID号为0x200，只用两个电调板，数据回传ID为0x201和0x202
	 cyq:更改为发送三个电调的指令。
*********************************************************************************/
void Cmd_ESC(int16_t current_201,int16_t current_202,int16_t current_203)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(current_201 >> 8);
    tx_message.Data[1] = (unsigned char)current_201;
    tx_message.Data[2] = (unsigned char)(current_202 >> 8);
    tx_message.Data[3] = (unsigned char)current_202;
    tx_message.Data[4] = (unsigned char)(current_203 >> 8);
    tx_message.Data[5] = (unsigned char)current_203;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    
    CAN_Transmit(CAN1,&tx_message);
}

/********************************************************************************
                         pitch轴电调板的速度环控制
                    输入 pitch轴当前速度 pitch轴目标速度
*********************************************************************************/
float Velocity_Control_201(float current_velocity_201,float target_velocity_201)
{
    const float v_p = 0.85;
    const float v_d = 2.5;
    
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
    
    if(abs(current_velocity_201) < GAP)
    {
        current_velocity_201 = 0.0;
    }
    
    error_v[0] = error_v[1];
    error_v[1] = target_velocity_201 - current_velocity_201;
    
    output = error_v[1] * v_p             
             + (error_v[1] - error_v[0]) * v_d;
     
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    
    return output;//cyq:for6015 反向
}


/********************************************************************************
                         pitch轴电调板的位置环控制 
                    输入 pitch轴当前位置 pitch轴目标位置
*********************************************************************************/
float Position_Control_201(float current_position_201,float target_position_201)
{
    
    const float l_p = 1.0;
    const float l_i = 0.0;
    const float l_d = 500.0;

    static float error_l[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;
    
    error_l[0] = error_l[1];
    error_l[1] = target_position_201 - current_position_201;
    inte += error_l[1]; 
    
    output = error_l[1] * l_p 
            + inte * l_i 
            + (error_l[1] - error_l[0]) * l_d;
    
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    		
    return output;
}
/********************************************************************************
                           yaw轴电调板的速度环控制
                      输入 yaw轴当前速度 yaw轴目标速度
*********************************************************************************/
float Velocity_Control_202(float current_velocity_202,float target_velocity_202)
{
    const float v_p = 1.25;
    const float v_d = 0.2;
    
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
		
    if(abs(current_velocity_202) < GAP)
    {
        current_velocity_202 = 0.0;
    }
    
    error_v[0] = error_v[1];
    error_v[1] = target_velocity_202- current_velocity_202;
    
    output = error_v[1] * v_p
             + (error_v[1] - error_v[0]) * v_d;
     
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    
    return output;//cyq:for6015 反向
}

/********************************************************************************
                           yaw轴电调板的位置环控制
                      输入 yaw轴当前位置 yaw轴目标位置
*********************************************************************************/
float Position_Control_202(float current_position_202,float target_position_202)
{
    const float l_p = 50.0;//3#5#:0.760
	  const float l_i = 0.0;//0.000035;
    const float l_d = 100;//3.5;
    
    static float error_l[3] = {0.0,0.0,0.0};
    static float output = 0;
		static float inte=0;
    
    error_l[0] = error_l[1];
    error_l[1] = error_l[2];    
    error_l[2] = target_position_202 - current_position_202;
		inte+=error_l[2];
		if(error_l[2] > 90.0)error_l[2] = 90.0;
		else if(error_l[2] < -90.0)error_l[2] = -90.0;
		
 
    output = error_l[2] * l_p 
							+ inte * l_i 
							+ (error_l[2] - error_l[1]) * l_d;
    
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    
    return output;
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


float rate_201_temp[BUF_NUM] = {0};
float rate_201 = 0;

unsigned char buf_cnt = 0;
unsigned int encoder_cnt = 0;

volatile unsigned char InitFlag = 0;
unsigned char TimeCnt = 0;


void Gimbal_Control(float want_pitch_angle,float want_yaw_angle)
{
        float sum = 0;
        int i;
        
	      last_201_angle = this_201_angle;
        this_201_angle = (encoder_201 - encoder_bias_201) * 0.04395;       //从陀螺仪那边取出角度;cyq:转化为欧拉角
        rate_201_angle = this_201_angle - last_201_angle;   //得到201当前的速度
        
        last_202_angle = this_202_angle;
        this_202_angle = (encoder_202 - encoder_bias_202) * 0.04395;                       //从编码器那边取出角度，暂时这么控制//cyq：换成202的编码器
        rate_202_angle = this_202_angle - last_202_angle;   //得到203当前的速度
                
        rate_201_temp[buf_cnt++] = rate_201_angle;//MPU6050_Real_Data.Gyro_X;//滤波针对开枪的时候
        if(buf_cnt == BUF_NUM)
        {
            buf_cnt = 0;
        }							
        for(i = 0;i < BUF_NUM;i++)
        {
            sum += rate_201_temp[i];
        }
        rate_201 = sum/BUF_NUM;    
            
        if(encoder_cnt < 1000)//初始化阶段
        {
            LED2_TOGGLE();//red
            position_201_output = Position_Control_201(this_201_angle,want_pitch_angle);  //计算位置闭环输出量，传给速度闭环 cyq:MPU6050_Angle.Pitch this_201_angle
            velocity_201_output = Velocity_Control_201(rate_201 * 500,position_201_output); //计算速度闭环输出量, 使用编码器滤波后的数据来做速度环
         
            position_202_output = Position_Control_202(this_202_angle,want_yaw_angle);    //先锁定在初始位置，待陀罗稳定
            velocity_202_output = Velocity_Control_202(rate_202_angle*500,position_202_output); 

 //       Cmd_ESC((int16_t)velocity_201_output,(int16_t)velocity_202_output,(int16_t)velocity_202_output);//cyq:输出给到pitch yaw电机
            
            if(encoder_cnt == 4000){
                GYRO_RST();
            }
            encoder_cnt++;
        }
        else if(encoder_cnt >= 1000)//初始化完成
        {
					  LED2_OFF();
            LED1_TOGGLE();//green
            position_202_output = Position_Control_202(this_202_angle,want_yaw_angle);    //先锁定在初始位置，待陀罗稳定
            velocity_202_output = Velocity_Control_202(rate_202_angle*500,position_202_output); 
            position_201_output = Position_Control_201(this_201_angle,want_pitch_angle);  //计算位置闭环输出量，传给速度闭环 cyq:MPU6050_Angle.Pitch this_201_angle
            velocity_201_output = Velocity_Control_201(rate_201 * 500,position_201_output); //计算速度闭环输出量, 使用编码器滤波后的数据来做速度环
            
//            if(shooting_flag == 1)	//shoot
//                {	
//                    position_201_output = Position_Control_201(this_201_angle,want_pitch_angle);  //计算位置闭环输出量，传给速度闭环 cyq:MPU6050_Angle.Pitch this_201_angle
//                    velocity_201_output = Velocity_Control_201(rate_201 * 600,position_201_output); //计算速度闭环输出量, 使用编码器滤波后的数据来做速度环					
//            
//                }
//            else 
//                {
//                    position_201_output = Position_Control_201(this_201_angle,want_pitch_angle);  //计算位置闭环输出量，传给速度闭环 cyq:MPU6050_Angle.Pitch 增稳
//                    velocity_201_output = Velocity_Control_201(MPU6050_Real_Data.Gyro_X,position_201_output); 
//                }
                
            if(OverCurr_flag)
            {
                Cmd_ESC(0,0,0);
                Encoder_sent(0); //如果电流保护后就不再将误差传递到底盘           
            }
            else 
            {
                Cmd_ESC((int16_t)velocity_201_output,(int16_t)velocity_202_output,(int16_t)velocity_202_output);//cyq:输出给到202电机
                //如果电流保护后就不再将误差传递到底盘
 //               Encoder_sent(this_202_angle);//将码盘的偏差传递到底盘            
            }
            
        }     
}
