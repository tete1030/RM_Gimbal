#include "main.h"
#define BUF_NUM 10						//��������10���˲���600 ; mpu:10 ���˲�
#define GAP 1.0
float d_error_201;
/********************************************************************************
   ������巢��ָ�ID��Ϊ0x200��ֻ����������壬���ݻش�IDΪ0x201��0x202
	 cyq:����Ϊ�������������ָ�
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
                         pitch��������ٶȻ�����
                    ���� pitch�ᵱǰ�ٶ� pitch��Ŀ���ٶ�
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
    
    return output;//cyq:for6015 ����
}


/********************************************************************************
                         pitch�������λ�û����� 
                    ���� pitch�ᵱǰλ�� pitch��Ŀ��λ��
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
                           yaw��������ٶȻ�����
                      ���� yaw�ᵱǰ�ٶ� yaw��Ŀ���ٶ�
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
    
    return output;//cyq:for6015 ����
}

/********************************************************************************
                           yaw�������λ�û�����
                      ���� yaw�ᵱǰλ�� yaw��Ŀ��λ��
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



float last_201_angle = 0.0;       //�ϴ�pitch��ĽǶ�
float this_201_angle = 0.0;       //����pitch��ĽǶ�
float rate_201_angle = 0.0;       //pitch����ٶ� = ����pitch��Ƕ� - �ϴ�pitch��Ƕ�
float velocity_201_output = 0.0;  //pitch���ٶȻ��������ֵ
float position_201_output = 0.0;  //pitch��λ�û��������ֵ


float last_202_angle = 0.0;       //�ϴ�yaw��ĽǶ�
float this_202_angle = 0.0;       //����yaw��ĽǶ�
float rate_202_angle = 0.0;       //yaw����ٶ� = ����yaw��Ƕ� - �ϴ�yaw��Ƕ�
float velocity_202_output = 0.0;  //yaw���ٶȻ����������ֵ
float position_202_output = 0.0;  //yaw��λ�û����������ֵ


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
        this_201_angle = (encoder_201 - encoder_bias_201) * 0.04395;       //���������Ǳ�ȡ���Ƕ�;cyq:ת��Ϊŷ����
        rate_201_angle = this_201_angle - last_201_angle;   //�õ�201��ǰ���ٶ�
        
        last_202_angle = this_202_angle;
        this_202_angle = (encoder_202 - encoder_bias_202) * 0.04395;                       //�ӱ������Ǳ�ȡ���Ƕȣ���ʱ��ô����//cyq������202�ı�����
        rate_202_angle = this_202_angle - last_202_angle;   //�õ�203��ǰ���ٶ�
                
        rate_201_temp[buf_cnt++] = rate_201_angle;//MPU6050_Real_Data.Gyro_X;//�˲���Կ�ǹ��ʱ��
        if(buf_cnt == BUF_NUM)
        {
            buf_cnt = 0;
        }							
        for(i = 0;i < BUF_NUM;i++)
        {
            sum += rate_201_temp[i];
        }
        rate_201 = sum/BUF_NUM;    
            
        if(encoder_cnt < 1000)//��ʼ���׶�
        {
            LED2_TOGGLE();//red
            position_201_output = Position_Control_201(this_201_angle,want_pitch_angle);  //����λ�ñջ�������������ٶȱջ� cyq:MPU6050_Angle.Pitch this_201_angle
            velocity_201_output = Velocity_Control_201(rate_201 * 500,position_201_output); //�����ٶȱջ������, ʹ�ñ������˲�������������ٶȻ�
         
            position_202_output = Position_Control_202(this_202_angle,want_yaw_angle);    //�������ڳ�ʼλ�ã��������ȶ�
            velocity_202_output = Velocity_Control_202(rate_202_angle*500,position_202_output); 

 //       Cmd_ESC((int16_t)velocity_201_output,(int16_t)velocity_202_output,(int16_t)velocity_202_output);//cyq:�������pitch yaw���
            
            if(encoder_cnt == 4000){
                GYRO_RST();
            }
            encoder_cnt++;
        }
        else if(encoder_cnt >= 1000)//��ʼ�����
        {
					  LED2_OFF();
            LED1_TOGGLE();//green
            position_202_output = Position_Control_202(this_202_angle,want_yaw_angle);    //�������ڳ�ʼλ�ã��������ȶ�
            velocity_202_output = Velocity_Control_202(rate_202_angle*500,position_202_output); 
            position_201_output = Position_Control_201(this_201_angle,want_pitch_angle);  //����λ�ñջ�������������ٶȱջ� cyq:MPU6050_Angle.Pitch this_201_angle
            velocity_201_output = Velocity_Control_201(rate_201 * 500,position_201_output); //�����ٶȱջ������, ʹ�ñ������˲�������������ٶȻ�
            
//            if(shooting_flag == 1)	//shoot
//                {	
//                    position_201_output = Position_Control_201(this_201_angle,want_pitch_angle);  //����λ�ñջ�������������ٶȱջ� cyq:MPU6050_Angle.Pitch this_201_angle
//                    velocity_201_output = Velocity_Control_201(rate_201 * 600,position_201_output); //�����ٶȱջ������, ʹ�ñ������˲�������������ٶȻ�					
//            
//                }
//            else 
//                {
//                    position_201_output = Position_Control_201(this_201_angle,want_pitch_angle);  //����λ�ñջ�������������ٶȱջ� cyq:MPU6050_Angle.Pitch ����
//                    velocity_201_output = Velocity_Control_201(MPU6050_Real_Data.Gyro_X,position_201_output); 
//                }
                
            if(OverCurr_flag)
            {
                Cmd_ESC(0,0,0);
                Encoder_sent(0); //�������������Ͳ��ٽ����ݵ�����           
            }
            else 
            {
                Cmd_ESC((int16_t)velocity_201_output,(int16_t)velocity_202_output,(int16_t)velocity_202_output);//cyq:�������202���
                //�������������Ͳ��ٽ����ݵ�����
 //               Encoder_sent(this_202_angle);//�����̵�ƫ��ݵ�����            
            }
            
        }     
}
