#include "main.h"

#define GAP 0.0

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

float get_position(float current_position, float target_position)
{
	const float v_p = 10.0;
	const float v_i = 0.0;
	const float v_d = 0.0;
	return 0.0;
}

/********************************************************************************
                         pitch轴电调板的速度环控制
                    输入 pitch轴当前速度 pitch轴目标速度
*********************************************************************************/
float Velocity_Control_201(float current_velocity_201,float target_velocity_201)
{
    const float v_p = 20;
    const float v_i = 0.0;
    const float v_d = 0.1;

    
    const float v_counterclockwise = 2;

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

    if(output > 0) output /=v_counterclockwise;
     
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
                         pitch轴电调板的位置环控制
                    输入 pitch轴当前位置 pitch轴目标位置
*********************************************************************************/
float Position_Control_201(float current_position_201,float target_position_201)
{
    
    const float l_p = 8;
    const float l_i = 0;
    const float l_d = 10;

    const float output_clockwise = 1;

    static float error_l[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;
    
    error_l[0] = error_l[1];
    error_l[1] = target_position_201 - current_position_201;
    inte += error_l[1];
		
		if(inte>5000.0)inte=5000.0;
		if(inte<-5000.0)inte=-5000.0;
    
    output = error_l[1] * l_p 
            + inte * l_i 
		        //+MPU6050_Real_Data.Gyro_X * l_d;
            + (error_l[1] - error_l[0]) * l_d;
    
    if(output < 0) output /=output_clockwise;

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
    const float v_p = 1.0;
    const float v_d = 0.0;
    
    static float error_v[2] = {0.0,0.0};
    static float output = 0;
		
    if(abs(current_velocity_202) < GAP)
    {
        current_velocity_202 = 0.0;
    }
    
    error_v[0] = error_v[1];
    error_v[1] = target_velocity_202 - current_velocity_202;
    
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
    
    return -output;//cyq:for6015 反向
}

/********************************************************************************
                           yaw轴电调板的位置环控制
                      输入 yaw轴当前位置 yaw轴目标位置
*********************************************************************************/
float Position_Control_202(float current_position_202,float target_position_202)
{
    const float l_p = 3;//100.010;//3#5#:0.760
	const float l_i = 0;//0.000035;
    const float l_d = 10;//50.0;//3.5;
    
    static float error_l[3] = {0.0,0.0,0.0};
    static float output = 0;
    
    error_l[0] = error_l[1];
    error_l[1] = error_l[2];    
    error_l[2] = target_position_202 - current_position_202;
//		if(error_l[2] > 90.0)error_l[2] = 90.0;
//		else if(error_l[2] < -90.0)error_l[2] = -90.0;
		
 
    output = error_l[2] * l_p 
							+ error_l[2] * l_i 
//		          +rate_202_angle * l_d;
//		          +MPU6050_Real_Data.Gyro_Z * l_d;
							+ (error_l[2] - error_l[1]) * l_d;
    
    if(output > ESC_MAX)
    {
        output = ESC_MAX;
    }
    
    if(output < -ESC_MAX)
    {
        output = -ESC_MAX;
    }
    
    return -output;
}


float Position_Control_203(float current_position_203,float target_position_203)
{

    const float l_p = 40;
    const float l_i = 0;
    const float l_d = 80;

    const float output_clockwise = 1;

    static float error_l[2] = {0.0,0.0};
    static float output = 0;
    static float inte = 0;

    error_l[0] = error_l[1];
    error_l[1] = target_position_203 - current_position_203;
    inte += error_l[1];

		if(inte>5000.0)inte=5000.0;
		if(inte<-5000.0)inte=-5000.0;

    output = error_l[1] * l_p
            + inte * l_i
		        //+MPU6050_Real_Data.Gyro_X * l_d;
            + (error_l[1] - error_l[0]) * l_d;

    if(output < 0) output /=output_clockwise;

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
