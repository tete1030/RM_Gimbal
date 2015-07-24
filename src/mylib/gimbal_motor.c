//
// Created by Texot Qexoq on 5/17/15.
//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "stm32f4xx_can.h"
#include "can1.h"
#include "gimbal_motor.h"
#include "can_packet.h"
#include "timer.h"
#include "pid.h"
#include "mpu6050/eMPL/inv_mpu.h"
#include "mpu6050/mpu6050.h"
#include "maincontrol.h"
#include "main.h"

#define GM_MAX_SET_SPEED 5000
#define GM_MIN_SET_SPEED (-5000)

#define CAN_GM_PITCH_ADDR 0x1
#define CAN_GM_YAW_ADDR 0x3

#define GM_MECH_ANGLE_START 0
#define GM_MECH_ANGLE_END 0x1FFF
#define GM_MECH_ANGLE_RANGE_LENGTH (GM_MECH_ANGLE_END - GM_MECH_ANGLE_START)
#define GM_MECH_ANGLE_HALF_LENGTH (GM_MECH_ANGLE_RANGE_LENGTH / 2)
#define GM_MECH_ANGLE_CENTER ((GM_MECH_ANGLE_START + GM_MECH_ANGLE_END) / 2)

#if defined CAR_1
#define GM_MECH_ANGLE_STANDARD_YAW 6000
//#define GM_MECH_ANGLE_STANDARD_PITCH 3200
#elif defined CAR_2
#define GM_MECH_ANGLE_STANDARD_YAW 1800
#define GM_MECH_ANGLE_STANDARD_PITCH 3200
#elif defined CAR_3
#define GM_MECH_ANGLE_STANDARD_YAW 6950
#define GM_MECH_ANGLE_STANDARD_PITCH 8180
#endif

#define GM_VIT_MECH_ANGLE_START (GM_MECH_ANGLE_START - GM_MECH_ANGLE_CENTER)
#define GM_VIT_MECH_ANGLE_END (GM_MECH_ANGLE_END - GM_MECH_ANGLE_CENTER)

#define GM_VIT_MECH_ANGLE_MAX_POSITION_YAW 1000
#define GM_VIT_MECH_ANGLE_MIN_POSITION_YAW (-1000)

#define GM_VIT_MECH_ANGLE_MAX_POSITION_PITCH 1000
#define GM_VIT_MECH_ANGLE_MIN_POSITION_PITCH (-1000)

#define GM_MECH_ANGLE_BUFFER_DISTANCE_YAW 40
#define GM_MECH_ANGLE_BUFFER_DISTANCE_PITCH 40

#define GM_EULAR_ANGLE_OUTSIDE_BACK_STEP_YAW 0
#define GM_EULAR_ANGLE_OUTSIDE_BACK_STEP_PITCH 1

#define GM_EULER_ANGLE_START (-180.0)
#define GM_EULER_ANGLE_END 180.0
#define GM_EULER_ANGLE_RANGE_LENGTH (GM_EULER_ANGLE_END - GM_EULER_ANGLE_START)
#define GM_EULER_ANGLE_HALF_LENGTH (GM_EULER_ANGLE_RANGE_LENGTH / 2)
// 22.7527777
#define GM_RATIO_MECH_EULER_ANGLE ((float)GM_MECH_ANGLE_RANGE_LENGTH / GM_EULER_ANGLE_RANGE_LENGTH)


#define CURRENT_PROTECT_RUN_INTERVAL 100
#define CURRENT_PROTECT_ANGLE_CHANGE_STEP 90.0
#define CURRENT_PROTECT_TRIGGER_THRESHOLD 2000
#define CURRENT_PROTECT_TRIGGER_TIMES 50



#pragma pack(push, 1)
struct Gimbal_Motor_Set_Speed
{
    uint8_t pitch_201_speed_high;
    uint8_t pitch_201_speed_low;
    uint8_t _202_speed_high;
    uint8_t _202_speed_low;
    uint8_t yaw_203_speed_high;
    uint8_t yaw_203_speed_low;
    uint8_t reserved_1;
    uint8_t reserved_2;
};

struct Motor_Gimbal_Status
{
    uint8_t angle_high;
    uint8_t angle_low;
    uint8_t actual_current_high;
    uint8_t actual_current_low;
    uint8_t demand_current_high;
    uint8_t demand_current_low;
    uint8_t hall_state;
};

#pragma pack(pop)

volatile GM_Status gms_yaw;
volatile GM_Status gms_pitch;

volatile DMP_Data dmp_data[2];
volatile uint8_t cur_dmp_data = 0;
volatile uint8_t last_dmp_data = 1;

volatile float target_absolute_yaw_diff = 0.0;
volatile float target_absolute_pitch_diff = 0.0;

volatile int16_t target_relative_yaw = 0;
volatile int16_t target_relative_pitch = 0;

PID_Controller pid_ctrl_yaw_absolute;
PID_Controller pid_ctrl_pitch_absolute;

PID_Controller pid_ctrl_yaw_relative;
PID_Controller pid_ctrl_pitch_relative;

int8_t absolute_control_id = -1;
int8_t relative_control_id = -1;

uint8_t yaw_only = 0;

void Gimbal_Motor_Configuration(void)
{
    PID_Controller_Configuration pcc;
    void Gimbal_Motor_Can_Receive_Handler(CanRxMsg* rx_msg);
    void Gimbal_Motor_Can_Send_Handler(uint16_t id, int8_t code);
    void GM_Absolute_Control_Current_Protect(void);

    CAN1_Configuration(Gimbal_Motor_Can_Send_Handler, Gimbal_Motor_Can_Receive_Handler,
                       CAN_PACKET_DESTTYPE_GIMBAL | CAN_PACKET_PLACETYPE_SPECIFIC | CAN_GM_PITCH_ADDR,
                       CAN_PACKET_TYPE_MASK_DESTTYPE | CAN_PACKET_TYPE_MASK_PLACETYPE | CAN_PACKET_TYPE_MASK_SOURCEADDR_STRICT,
                       CAN_PACKET_DESTTYPE_GIMBAL | CAN_PACKET_PLACETYPE_SPECIFIC | CAN_GM_YAW_ADDR,
                       CAN_PACKET_TYPE_MASK_DESTTYPE | CAN_PACKET_TYPE_MASK_PLACETYPE | CAN_PACKET_TYPE_MASK_SOURCEADDR_STRICT
    );

    gms_yaw.angle = 0;
    gms_yaw.demand_current = 0;
    gms_yaw.actual_current = 0;
    gms_yaw.hall_state = 0;

    gms_pitch.angle = 0;
    gms_pitch.demand_current = 0;
    gms_pitch.actual_current = 0;
    gms_pitch.hall_state = 0;


    pcc.mode = PID_Controller_Mode_Absolute;
    pcc.kp = 24.6;//40.1;
    pcc.ki = 0.205;// ti=0.36;//3.6s/5=0.72
    pcc.kd = 738;// td=0.09;
    pcc.ko = 0;
    pcc.max_output = GM_MAX_SET_SPEED;
    pcc.min_output = GM_MIN_SET_SPEED;
    pcc.max_integral = GM_MAX_SET_SPEED / 5;
    pcc.min_integral = GM_MIN_SET_SPEED / 5;
    PID_Controller_Init(&pid_ctrl_yaw_absolute, &pcc);

    pcc.mode = PID_Controller_Mode_Absolute;
    pcc.kp = 20;
    pcc.ki = 0.001;
    pcc.kd = 40;
    pcc.ko = 0;
    pcc.max_output = GM_MAX_SET_SPEED;
    pcc.min_output = GM_MIN_SET_SPEED;
    pcc.max_integral = GM_MAX_SET_SPEED / 5;
    pcc.min_integral = GM_MIN_SET_SPEED / 5;
    PID_Controller_Init(&pid_ctrl_pitch_absolute, &pcc);

    pcc.mode = PID_Controller_Mode_Absolute;
    pcc.kp = 7;
    pcc.ki = 0;
    pcc.kd = 70;
    pcc.ko = 0;
    pcc.max_output = GM_MAX_SET_SPEED;
    pcc.min_output = GM_MIN_SET_SPEED;
    pcc.max_integral = GM_MAX_SET_SPEED / 5;
    pcc.min_integral = GM_MIN_SET_SPEED / 5;
    PID_Controller_Init(&pid_ctrl_yaw_relative, &pcc);

    pcc.mode = PID_Controller_Mode_Absolute;
    pcc.kp = 6;
    pcc.ki = 0;
    pcc.kd = 30;
    pcc.ko = 0;
    pcc.max_output = GM_MAX_SET_SPEED;
    pcc.min_output = GM_MIN_SET_SPEED;
    pcc.max_integral = GM_MAX_SET_SPEED / 5;
    pcc.min_integral = GM_MIN_SET_SPEED / 5;
    PID_Controller_Init(&pid_ctrl_pitch_relative, &pcc);

#if !(defined CAR_1)
    Timer_Register(CURRENT_PROTECT_RUN_INTERVAL, GM_Absolute_Control_Current_Protect);
#endif
}


void GM_Absolute_Control_Current_Protect(void)
{
	static uint32_t overload_times_yaw = 0, overload_times_pitch = 0;
	if(abs(gms_yaw.actual_current) > CURRENT_PROTECT_TRIGGER_THRESHOLD)
	{
		overload_times_yaw ++;
		if(overload_times_yaw > CURRENT_PROTECT_TRIGGER_TIMES)
		{
			if(target_absolute_yaw_diff > CURRENT_PROTECT_ANGLE_CHANGE_STEP) target_absolute_yaw_diff -= CURRENT_PROTECT_ANGLE_CHANGE_STEP;
			else if(target_absolute_yaw_diff > 0) target_absolute_yaw_diff = 0;
			else if(target_absolute_yaw_diff < -CURRENT_PROTECT_ANGLE_CHANGE_STEP) target_absolute_yaw_diff += CURRENT_PROTECT_ANGLE_CHANGE_STEP;
			else if(target_absolute_yaw_diff < 0) target_absolute_yaw_diff = 0;
		}

	}
	else
		overload_times_yaw = 0;

	if(abs(gms_pitch.actual_current) > CURRENT_PROTECT_TRIGGER_THRESHOLD)
	{
		overload_times_pitch ++;
		if(overload_times_pitch > CURRENT_PROTECT_TRIGGER_TIMES)
		{
			if(target_absolute_pitch_diff > CURRENT_PROTECT_ANGLE_CHANGE_STEP) target_absolute_pitch_diff -= CURRENT_PROTECT_ANGLE_CHANGE_STEP;
			else if(target_absolute_pitch_diff > 0) target_absolute_pitch_diff = 0;
			else if(target_absolute_pitch_diff < -CURRENT_PROTECT_ANGLE_CHANGE_STEP) target_absolute_pitch_diff += CURRENT_PROTECT_ANGLE_CHANGE_STEP;
			else if(target_absolute_pitch_diff < 0) target_absolute_pitch_diff = 0;
		}
	}
	else
		overload_times_pitch = 0;
}

void GM_Set_Control_Target_Relative_Yaw_Pitch(int16_t yaw, int16_t pitch)
{
    if(yaw > GM_VIT_MECH_ANGLE_MAX_POSITION_YAW)
        yaw = GM_VIT_MECH_ANGLE_MAX_POSITION_YAW;
    else if(yaw < GM_VIT_MECH_ANGLE_MIN_POSITION_YAW)
        yaw = GM_VIT_MECH_ANGLE_MIN_POSITION_YAW;

    if(pitch > GM_VIT_MECH_ANGLE_MAX_POSITION_PITCH)
        pitch = GM_VIT_MECH_ANGLE_MAX_POSITION_PITCH;
    else if(pitch < GM_VIT_MECH_ANGLE_MIN_POSITION_PITCH)
        pitch = GM_VIT_MECH_ANGLE_MIN_POSITION_PITCH;

    target_relative_yaw = yaw;
    target_relative_pitch = pitch;
}

void GM_Set_DMP_Data(DMP_Data *dd)
{
	float yaw_diff, pitch_diff;
    uint8_t i = (cur_dmp_data + 1) % 2;
    memcpy(&(dmp_data[i]), dd, sizeof(DMP_Data));
    last_dmp_data = cur_dmp_data;
    cur_dmp_data = i;


    yaw_diff = dmp_data[cur_dmp_data].yaw - dmp_data[last_dmp_data].yaw;
	pitch_diff = dmp_data[cur_dmp_data].pitch - dmp_data[last_dmp_data].pitch;

	if(yaw_diff > GM_EULER_ANGLE_HALF_LENGTH) yaw_diff -= GM_EULER_ANGLE_RANGE_LENGTH;
	if(yaw_diff < -GM_EULER_ANGLE_HALF_LENGTH) yaw_diff += GM_EULER_ANGLE_RANGE_LENGTH;

	if(pitch_diff > GM_EULER_ANGLE_HALF_LENGTH) pitch_diff -= GM_EULER_ANGLE_RANGE_LENGTH;
	if(pitch_diff < -GM_EULER_ANGLE_HALF_LENGTH) pitch_diff += GM_EULER_ANGLE_RANGE_LENGTH;

	target_absolute_yaw_diff -= yaw_diff;
	target_absolute_pitch_diff -= pitch_diff;

    if(target_absolute_yaw_diff > GM_EULER_ANGLE_END) target_absolute_yaw_diff = GM_EULER_ANGLE_END;
    else if(target_absolute_yaw_diff < GM_EULER_ANGLE_START) target_absolute_yaw_diff = GM_EULER_ANGLE_START;

    if(target_absolute_pitch_diff > GM_EULER_ANGLE_END) target_absolute_pitch_diff = GM_EULER_ANGLE_END;
    else if(target_absolute_pitch_diff < GM_EULER_ANGLE_START) target_absolute_pitch_diff = GM_EULER_ANGLE_START;
}


void GM_Set_Control_Target_Absolute_Yaw_Pitch_Speed(float yaw, float pitch)
{
	float yaw_diff, pitch_diff;
    // TODO: set all control priority above data update

	target_absolute_yaw_diff += yaw;
	target_absolute_pitch_diff += pitch;


    if(target_absolute_yaw_diff > GM_EULER_ANGLE_END) target_absolute_yaw_diff = GM_EULER_ANGLE_END;
    else if(target_absolute_yaw_diff < GM_EULER_ANGLE_START) target_absolute_yaw_diff = GM_EULER_ANGLE_START;

    if(target_absolute_pitch_diff > GM_EULER_ANGLE_END) target_absolute_pitch_diff = GM_EULER_ANGLE_END;
    else if(target_absolute_pitch_diff < GM_EULER_ANGLE_START) target_absolute_pitch_diff = GM_EULER_ANGLE_START;
}

volatile int16_t pitch_output_set = 0;

void GM_Absolute_Control() {
    int16_t yaw_output, pitch_output;
#if !defined CAR_1
    //yaw_output = (int16_t) PID_Controller_Calc(&pid_ctrl_yaw_absolute, 0, target_absolute_yaw_diff, 0, NULL, 0);
    //pitch_output = (int16_t) PID_Controller_Calc(&pid_ctrl_pitch_absolute, 0, target_absolute_pitch_diff, 0, NULL, 0);

#if defined CAR_2
    //pitch_output_set = pitch_output;
    //GM_Set_Speed(pitch_output, 0);
#elif defined CAR_3
    //pitch_output_set = -pitch_output;
    //GM_Set_Speed(-pitch_output, 0);
#endif
#endif // !defined CAR_1
}

volatile uint32_t ii = 0;

void GM_Relative_Control()
{
    int16_t yaw_output, pitch_output;


    yaw_output = (int16_t) PID_Controller_Calc(&pid_ctrl_yaw_relative, gms_yaw.angle, target_relative_yaw, 0, NULL, 0);

    if(yaw_only)
    {
    	if(ii >= 10)
    	{
    		pitch_output_set = (int16_t) PID_Controller_Calc(&pid_ctrl_pitch_absolute, 0, target_absolute_pitch_diff, 0, NULL, 0);
    		ii = 0;
    	}
    	ii++;
#if defined CAR_3
    	pitch_output = -pitch_output_set;
#elif defined CAR_2
    	pitch_output = pitch_output_set;
#endif

    }
    else
    	pitch_output = (int16_t) PID_Controller_Calc(&pid_ctrl_pitch_relative, gms_pitch.angle, target_relative_pitch, 0, NULL, 0);


    GM_Set_Speed(pitch_output, yaw_output);
}

void GM_Start_Relative_Control()
{
    if(relative_control_id == -1)
    {
    	yaw_only = 0;
        relative_control_id = Timer_Register(1, GM_Relative_Control);
    }
}

void GM_Stop_Relative_Control()
{
    if(relative_control_id != -1) {
        Timer_Unregister(relative_control_id);
        relative_control_id = -1;
    }
    GM_Set_Speed(0, 0);
}

void GM_Stop_Pitch_Relative_Control()
{
	yaw_only = 1;

}

void GM_Set_Standard_Position()
{
    GM_Stop_Absolute_Control();
    GM_Set_Control_Target_Relative_Yaw_Pitch(0, 0);

}

void GM_Start_Absolute_Control()
{

    if(absolute_control_id == -1)
    {
    	//target_absolute_pitch_diff = 0 -
        absolute_control_id = Timer_Register(10, GM_Absolute_Control);
    }
}

void GM_Stop_Absolute_Control()
{
    if(absolute_control_id != -1) {
        Timer_Unregister(absolute_control_id);
        absolute_control_id = -1;
    }
    GM_Set_Speed(0, 0);
    GM_Set_Control_Target_Relative_Yaw_Pitch(gms_yaw.angle, gms_pitch.angle);
}

void Gimbal_Motor_Can_Receive_Handler(CanRxMsg* rx_msg)
{
    uint16_t type,id;
    struct Motor_Gimbal_Status *mgs;

    type = (uint16_t) (rx_msg->StdId & CAN_PACKET_TYPE_MASK_DATATYPE);
    id = (uint16_t) (rx_msg->StdId & CAN_PACKET_TYPE_MASK_SOURCEADDR_STRICT);
    mgs = (struct Motor_Gimbal_Status *) rx_msg->Data;
#if !(defined CAR_1)
    if(id == CAN_GM_PITCH_ADDR)
    {
        // standardize the angle to -0xFFF ~ 0x1000 with center 0 at STANDARD PITCH POSITION
        gms_pitch.angle = (int16_t)
                (((mgs->angle_high << 8 | mgs->angle_low)
                  + GM_MECH_ANGLE_RANGE_LENGTH + GM_MECH_ANGLE_HALF_LENGTH
                  - GM_MECH_ANGLE_STANDARD_PITCH)
                 % GM_MECH_ANGLE_RANGE_LENGTH
                 - (GM_MECH_ANGLE_CENTER - GM_MECH_ANGLE_START));
//        gms_pitch.angle = mgs->angle_high << 8 | mgs->angle_low;
        gms_pitch.actual_current = mgs->actual_current_high << 8 | mgs->actual_current_low;
        gms_pitch.demand_current = mgs->demand_current_high << 8 | mgs->demand_current_low;
        gms_pitch.hall_state = mgs->hall_state;

    }
    else
#endif
    if(id == CAN_GM_YAW_ADDR)
    {
        // standardize the angle to -0xFFF ~ 0x1000 with center 0 at STANDARD YAW POSITION
        gms_yaw.angle = (int16_t)
                (((mgs->angle_high << 8 | mgs->angle_low)
                  + GM_MECH_ANGLE_RANGE_LENGTH + GM_MECH_ANGLE_HALF_LENGTH
                  - GM_MECH_ANGLE_STANDARD_YAW)
                 % GM_MECH_ANGLE_RANGE_LENGTH
                - (GM_MECH_ANGLE_CENTER - GM_MECH_ANGLE_START));
//        gms_yaw.angle = mgs->angle_high << 8 | mgs->angle_low;
        gms_yaw.actual_current = mgs->actual_current_high << 8 | mgs->actual_current_low;
        gms_yaw.demand_current = mgs->demand_current_high << 8 | mgs->demand_current_low;
        gms_yaw.hall_state = mgs->hall_state;
#if !(defined CAR_1)
        Maincontrol_Send_Mech_Yaw();
#endif
    }

}

void Gimbal_Motor_Can_Send_Handler(uint16_t id, int8_t code)
{
    if(code != 0)printf("id %u send failed with code %d from can1\r\n", id, code);
}

#define GIMBAL_PACKET_ID_SET_SPEED 0x0
#define GIMBAL_BROADCAST_ADDR 0x200

void GM_Set_Speed(int16_t pitch_speed, int16_t yaw_speed)
{
    static struct Gimbal_Motor_Set_Speed gmss;

    if(pitch_speed > GM_MAX_SET_SPEED) pitch_speed = GM_MAX_SET_SPEED;
    else if(pitch_speed < GM_MIN_SET_SPEED) pitch_speed = GM_MIN_SET_SPEED;
    if(yaw_speed > GM_MAX_SET_SPEED) yaw_speed = GM_MAX_SET_SPEED;
    else if(yaw_speed < GM_MIN_SET_SPEED) yaw_speed = GM_MIN_SET_SPEED;


    gmss.pitch_201_speed_high = (uint8_t) ((pitch_speed >> 8) & 0xff);
    gmss.pitch_201_speed_low = (uint8_t) (pitch_speed & 0xff);
    gmss._202_speed_high = 0;
    gmss._202_speed_low = 0;
    gmss.yaw_203_speed_high = (uint8_t) ((yaw_speed >> 8) & 0xff);
    gmss.yaw_203_speed_low = (uint8_t) (yaw_speed & 0xff);
    gmss.reserved_1 = 0;
    gmss.reserved_2 = 0;

    CAN1_AsyncTransmit(GIMBAL_PACKET_ID_SET_SPEED, GIMBAL_BROADCAST_ADDR, (char*) &gmss, sizeof(gmss));
}


#if 0
void GM_Absolute_Control()
{
    int16_t yaw_output, pitch_output;
    float real_target_yaw, real_target_pitch;
    float ratio_ori_target_buffer_yaw, ratio_ori_target_buffer_pitch;
    uint8_t bound_side_pitch, bound_side_yaw;


        ratio_ori_target_buffer_pitch = (float) (GM_VIT_MECH_ANGLE_MAX_POSITION_PITCH - gms_pitch.angle) /
                                 GM_MECH_ANGLE_BUFFER_DISTANCE_PITCH;
        bound_side_pitch = 0;
        if(ratio_ori_target_buffer_pitch > 1) {
            ratio_ori_target_buffer_pitch = (float) (gms_pitch.angle - GM_VIT_MECH_ANGLE_MIN_POSITION_PITCH) /
                                            GM_MECH_ANGLE_BUFFER_DISTANCE_PITCH;
            bound_side_pitch = 1;
        }

        real_target_pitch = target_absolute_pitch;

        // 处于inside和回向inside时不限速
        if(ratio_ori_target_buffer_pitch >= 1 ||
                (bound_side_pitch && target_absolute_pitch > dmp_data.pitch) ||
                (!bound_side_pitch && target_absolute_pitch < dmp_data.pitch))
        {
            // inside or going to inside
            real_target_pitch = target_absolute_pitch;
        }
        else if(ratio_ori_target_buffer_pitch <= -1)
        {
            // outside
            //real_target_pitch = dmp_data.pitch + (bound_side_pitch ? GM_EULAR_ANGLE_OUTSIDE_BACK_STEP_PITCH : -GM_EULAR_ANGLE_OUTSIDE_BACK_STEP_PITCH);

        }
        else if(ratio_ori_target_buffer_pitch >= 0)
        {
            // buffer of inside
            real_target_pitch = dmp_data.pitch +
                    ratio_ori_target_buffer_pitch * (target_absolute_pitch - dmp_data.pitch);

        }
        else
        {
            // buffer of outside
            real_target_pitch = dmp_data.pitch +
                    (-ratio_ori_target_buffer_pitch) * (bound_side_pitch ? GM_EULAR_ANGLE_OUTSIDE_BACK_STEP_PITCH : -GM_EULAR_ANGLE_OUTSIDE_BACK_STEP_PITCH);

        }

        // 角度环处理

        // 不需要经过以下处理, 因为Pitch轴不能翻转
        /*
        // 此处pitch在mpu中经过特殊处理(也就是原Roll换名为Pitch),支持-180~180
        while(real_target_pitch > GM_EULER_ANGLE_END) real_target_pitch -= GM_EULER_ANGLE_RANGE_LENGTH;
        while(real_target_pitch < GM_EULER_ANGLE_START) real_target_pitch += GM_EULER_ANGLE_RANGE_LENGTH;


        if(fabs(real_target_pitch - dmp_data.pitch) > GM_EULER_ANGLE_HALF_LENGTH)
        {
            if(dmp_data.pitch > 0) real_target_pitch += GM_EULER_ANGLE_RANGE_LENGTH;
            else real_target_pitch -= GM_EULER_ANGLE_RANGE_LENGTH;
        }
        */

        // left
        ratio_ori_target_buffer_yaw = (float) (GM_VIT_MECH_ANGLE_MAX_POSITION_YAW - gms_yaw.angle) /
                                      GM_MECH_ANGLE_BUFFER_DISTANCE_YAW;
        bound_side_yaw = 0;
        if(ratio_ori_target_buffer_yaw > 1) {
        	// right
            ratio_ori_target_buffer_yaw = (float) (gms_yaw.angle - GM_VIT_MECH_ANGLE_MIN_POSITION_YAW) /
                                          GM_MECH_ANGLE_BUFFER_DISTANCE_YAW;
            bound_side_yaw = 1;
        }

        if(ratio_ori_target_buffer_yaw >= 1)
        		//|| (bound_side_yaw && target_absolute_yaw > dmp_data.yaw)
				//|| (!bound_side_yaw && target_absolute_yaw < dmp_data.yaw))
        {
            // inside or going to inside
            real_target_yaw = target_absolute_yaw;
        }
        else if(ratio_ori_target_buffer_yaw <= -1)
        {
            // outside
            real_target_yaw = dmp_data.yaw + (bound_side_yaw ? GM_EULAR_ANGLE_OUTSIDE_BACK_STEP_YAW : -GM_EULAR_ANGLE_OUTSIDE_BACK_STEP_YAW);
        }
        else if(ratio_ori_target_buffer_yaw >= 0)
        {
            // buffer of inside
            real_target_yaw = dmp_data.yaw +
                    ratio_ori_target_buffer_yaw * (target_absolute_yaw - dmp_data.yaw);
        }
        else
        {
            real_target_yaw = dmp_data.yaw +
                    (-ratio_ori_target_buffer_yaw) * (bound_side_yaw ? GM_EULAR_ANGLE_OUTSIDE_BACK_STEP_YAW : -GM_EULAR_ANGLE_OUTSIDE_BACK_STEP_YAW);
        }

        // 角度环处理

        while(real_target_yaw > GM_EULER_ANGLE_END)
        	real_target_yaw -= GM_EULER_ANGLE_RANGE_LENGTH;
        while(real_target_yaw < GM_EULER_ANGLE_START)
        	real_target_yaw += GM_EULER_ANGLE_RANGE_LENGTH;

        if(fabs(real_target_yaw - dmp_data.yaw) > GM_EULER_ANGLE_HALF_LENGTH)
        {
            if(dmp_data.yaw > 0) real_target_yaw += GM_EULER_ANGLE_RANGE_LENGTH;
            else real_target_yaw -= GM_EULER_ANGLE_RANGE_LENGTH;
        }

        yaw_output = (int16_t) PID_Controller_Calc(&pid_ctrl_yaw_absolute, dmp_data.yaw, real_target_yaw, 0, NULL);
        pitch_output = (int16_t) PID_Controller_Calc(&pid_ctrl_pitch_absolute, dmp_data.pitch, real_target_pitch,
                                                     0, NULL);
        Gimbal_Motor_Set_Speed(pitch_output, yaw_output);
    }
}

#endif

#if 0

void GM_Absolute_Control() {
    float real_target_yaw, real_target_pitch;
    int16_t yaw_output, pitch_output;

    // TODO: try increment

    /*
    if(fabs(real_target_pitch - dmp_data.pitch) > GM_EULER_ANGLE_HALF_LENGTH)
    {
        if(dmp_data.pitch > 0) real_target_pitch += GM_EULER_ANGLE_RANGE_LENGTH;
        else real_target_pitch -= GM_EULER_ANGLE_RANGE_LENGTH;
    }
    */
    real_target_pitch = target_absolute_pitch;

    if(fabs(target_absolute_yaw - dmp_data.yaw) > GM_EULER_ANGLE_HALF_LENGTH)
    {
        if(dmp_data.yaw > 0) real_target_yaw = target_absolute_yaw + GM_EULER_ANGLE_RANGE_LENGTH;
        else real_target_yaw = target_absolute_yaw - GM_EULER_ANGLE_RANGE_LENGTH;
    }
    else
        real_target_yaw = target_absolute_yaw;


    yaw_output = (int16_t) PID_Controller_Calc(&pid_ctrl_yaw_absolute, dmp_data.yaw, real_target_yaw, 0, NULL);
    pitch_output = (int16_t) PID_Controller_Calc(&pid_ctrl_pitch_absolute, dmp_data.pitch, real_target_pitch, 0, NULL);
    GM_Set_Control_Target_Relative_Yaw_Pitch(yaw_output, pitch_output);
}

#endif /* if 0 */

#if 0
void GM_Absolute_Control()
{
    int16_t yaw_output, pitch_output;
    float ratio_ori_target_buffer_yaw, ratio_ori_target_buffer_pitch;
    uint8_t bound_side_pitch, bound_side_yaw;

    yaw_output = (int16_t) PID_Controller_Calc(&pid_ctrl_yaw_absolute, dmp_data.yaw, target_absolute_yaw, 0, NULL, 0);
    pitch_output = (int16_t) PID_Controller_Calc(&pid_ctrl_pitch_absolute, dmp_data.pitch, target_absolute_pitch, 0, NULL, 0);


	ratio_ori_target_buffer_pitch = (float) (GM_VIT_MECH_ANGLE_MAX_POSITION_PITCH - gms_pitch.angle) /
							 GM_MECH_ANGLE_BUFFER_DISTANCE_PITCH;
	bound_side_pitch = 0;
	if(ratio_ori_target_buffer_pitch > 1) {
		ratio_ori_target_buffer_pitch = (float) (gms_pitch.angle - GM_VIT_MECH_ANGLE_MIN_POSITION_PITCH) /
										GM_MECH_ANGLE_BUFFER_DISTANCE_PITCH;
		bound_side_pitch = 1;
	}


	// 处于inside和回向inside时不限速
	if(ratio_ori_target_buffer_pitch >= 1)
	{
		// inside or going to inside
	}
	else if(ratio_ori_target_buffer_pitch < 0)
	{
		// outside
		if(bound_side_pitch) pitch_output = (1 - ratio_ori_target_buffer_yaw) * 300;
		else pitch_output = (1 - ratio_ori_target_buffer_yaw) * (-300);

	}
	else// if(ratio_ori_target_buffer_pitch >= 0)
	{
		// buffer of inside
		if(bound_side_pitch) pitch_output = ratio_ori_target_buffer_pitch * pitch_output + (1 - ratio_ori_target_buffer_pitch) * 300;
		else pitch_output = pitch_output = ratio_ori_target_buffer_pitch * pitch_output + (1 - ratio_ori_target_buffer_pitch) * (-300);

	}

	if(pitch_output > GM_MAX_SET_SPEED)
		pitch_output = GM_MAX_SET_SPEED;
	else if(pitch_output < GM_MIN_SET_SPEED)
		pitch_output = GM_MIN_SET_SPEED;

	if(ratio_ori_target_buffer_pitch < 0 && ratio_ori_target_buffer_pitch > -1.5)
	{
		if(bound_side_pitch && target_absolute_pitch < dmp_data.pitch)
		{
			target_absolute_pitch = dmp_data.pitch - 5;
		}
		else if(!bound_side_pitch && target_absolute_pitch > dmp_data.pitch)
		{
			target_absolute_pitch = dmp_data.pitch + 5;
		}

		while(target_absolute_pitch < GM_EULER_ANGLE_START) target_absolute_pitch += GM_EULER_ANGLE_RANGE_LENGTH;
		while(target_absolute_pitch > GM_EULER_ANGLE_END) target_absolute_pitch -= GM_EULER_ANGLE_RANGE_LENGTH;
	}



	// left
	ratio_ori_target_buffer_yaw = (float) (GM_VIT_MECH_ANGLE_MAX_POSITION_YAW - gms_yaw.angle) /
								  GM_MECH_ANGLE_BUFFER_DISTANCE_YAW;
	bound_side_yaw = 0;
	if(ratio_ori_target_buffer_yaw > 1) {
		// right
		ratio_ori_target_buffer_yaw = (float) (gms_yaw.angle - GM_VIT_MECH_ANGLE_MIN_POSITION_YAW) /
									  GM_MECH_ANGLE_BUFFER_DISTANCE_YAW;
		bound_side_yaw = 1;
	}

	if(ratio_ori_target_buffer_yaw >= 1)
	{
		// inside or going to inside
	}
	else if(ratio_ori_target_buffer_yaw < 0)
	{
		// outside
		if(bound_side_yaw) yaw_output = (1 - ratio_ori_target_buffer_yaw) * 500;
		else yaw_output = (1 - ratio_ori_target_buffer_yaw) * (-500);
	}
	else
	{
		// buffer of inside
		// at right
		if(bound_side_yaw) yaw_output = ratio_ori_target_buffer_yaw * yaw_output + (1 - ratio_ori_target_buffer_yaw) * 500;
		else yaw_output = ratio_ori_target_buffer_yaw * yaw_output + (1 - ratio_ori_target_buffer_yaw) * -500;
	}

	if(yaw_output > GM_MAX_SET_SPEED)
		yaw_output = GM_MAX_SET_SPEED;
	else if(yaw_output < GM_MIN_SET_SPEED)
		yaw_output = GM_MIN_SET_SPEED;

	if(ratio_ori_target_buffer_yaw < 0 && ratio_ori_target_buffer_yaw > -1.5)
	{
		if(bound_side_yaw && target_absolute_yaw < dmp_data.yaw)
		{
			target_absolute_yaw = dmp_data.yaw - 5;
		}
		else if(!bound_side_yaw && target_absolute_yaw > dmp_data.yaw)
		{
			target_absolute_yaw = dmp_data.yaw + 5;
		}

		while(target_absolute_yaw < GM_EULER_ANGLE_START) target_absolute_yaw += GM_EULER_ANGLE_RANGE_LENGTH;
		while(target_absolute_yaw > GM_EULER_ANGLE_END) target_absolute_yaw -= GM_EULER_ANGLE_RANGE_LENGTH;
	}

	GM_Set_Speed(pitch_output, yaw_output);

}
#endif
