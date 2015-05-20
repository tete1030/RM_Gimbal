//
// Created by Texot Qexoq on 5/17/15.
//
#include <stdio.h>
#include "stm32f4xx_can.h"
#include "can1.h"
#include "gimbal_motor.h"
#include "can_packet.h"
#include "timer.h"
#include "pid.h"
#include "mpu6050/eMPL/inv_mpu.h"
#include "mpu6050/mpu6050.h"


#define GIMBAL_MOTOR_MAX_SET_SPEED 5000
#define GIMBAL_MOTOR_MIN_SET_SPEED -5000

#define CAN_GIMBAL_MOTOR_PITCH_ADDR 0x1
#define CAN_GIMBAL_MOTOR_YAW_ADDR 0x3

#define STANDARD_RELATIVE_YAW_POSITION 1800
#define STANDARD_RELATIVE_PITCH_POSITION 3200

#define MIN_RELATIVE_YAW_POSITION 1500
#define MAX_RELATIVE_YAW_POSITION 2100

#define MIN_RELATIVE_PITCH_POSITION 2900
#define MAX_RELATIVE_PITCH_POSITION 3500

#define MIN_ABSOLUTE_YAW_POSITION -60.0
#define MAX_ABSOLUTE_YAW_POSITION 60.0

#define MIN_ABSOLUTE_PITCH_POSITION -60.0
#define MAX_ABSOLUTE_PITCH_POSITION 60.0

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

Gimbal_Motor_Status gms_yaw;
Gimbal_Motor_Status gms_pitch;

float target_absolute_yaw = 0.0;
float target_absolute_pitch = 0.0;

uint16_t target_relative_yaw = 0;
uint16_t target_relative_pitch = 0;

PID_Controller pid_ctrl_yaw_absolute;
PID_Controller pid_ctrl_pitch_absolute;

PID_Controller pid_ctrl_yaw_relative;
PID_Controller pid_ctrl_pitch_relative;

uint8_t absolute_control_enable = 0;

//int8_t absolute_control_id = -1;
int8_t relative_control_id = -1;

int8_t Can1_Tx_Buffer[8];

void Gimbal_Motor_Configuration(void)
{
    extern void Gimbal_Motor_Can_Receive_Handler(CanRxMsg* rx_msg);
    extern void Gimbal_Motor_Can_Send_Handler(uint16_t id, int8_t code);

    CAN1_Configuration(Gimbal_Motor_Can_Send_Handler, Gimbal_Motor_Can_Receive_Handler,
                       CAN_PACKET_DESTTYPE_GIMBAL | CAN_PACKET_PLACETYPE_SPECIFIC | CAN_GIMBAL_MOTOR_PITCH_ADDR,
                       CAN_PACKET_TYPE_MASK_DESTTYPE | CAN_PACKET_TYPE_MASK_PLACETYPE | CAN_PACKET_TYPE_MASK_SOURCEADDR_STRICT,
                       CAN_PACKET_DESTTYPE_GIMBAL | CAN_PACKET_PLACETYPE_SPECIFIC | CAN_GIMBAL_MOTOR_YAW_ADDR,
                       CAN_PACKET_TYPE_MASK_DESTTYPE | CAN_PACKET_TYPE_MASK_PLACETYPE | CAN_PACKET_TYPE_MASK_SOURCEADDR_STRICT
    );

    gms_yaw.angle = STANDARD_RELATIVE_YAW_POSITION;
    gms_yaw.demand_current = 0;
    gms_yaw.actual_current = 0;
    gms_yaw.hall_state = 0;

    gms_pitch.angle = STANDARD_RELATIVE_PITCH_POSITION;
    gms_pitch.demand_current = 0;
    gms_pitch.actual_current = 0;
    gms_pitch.hall_state = 0;

    PID_Init(&pid_ctrl_yaw_absolute, 100, 0, 400, 0, GIMBAL_MOTOR_MAX_SET_SPEED, GIMBAL_MOTOR_MIN_SET_SPEED, GIMBAL_MOTOR_MAX_SET_SPEED / 20, GIMBAL_MOTOR_MIN_SET_SPEED / 20);
    PID_Init(&pid_ctrl_pitch_absolute, 50, 0, 200, 0, GIMBAL_MOTOR_MAX_SET_SPEED, GIMBAL_MOTOR_MIN_SET_SPEED, GIMBAL_MOTOR_MAX_SET_SPEED / 20, GIMBAL_MOTOR_MIN_SET_SPEED / 20);

    PID_Init(&pid_ctrl_yaw_relative, 10, 0.01, 100, 0, GIMBAL_MOTOR_MAX_SET_SPEED, GIMBAL_MOTOR_MIN_SET_SPEED, GIMBAL_MOTOR_MAX_SET_SPEED / 5, GIMBAL_MOTOR_MIN_SET_SPEED / 5);
    PID_Init(&pid_ctrl_pitch_relative, 20, 0.07, 40, 0, GIMBAL_MOTOR_MAX_SET_SPEED, GIMBAL_MOTOR_MIN_SET_SPEED, GIMBAL_MOTOR_MAX_SET_SPEED / 5, GIMBAL_MOTOR_MIN_SET_SPEED / 5);
}

void Gimbal_Set_Control_Target_Absolute_Yaw_Pitch(float yaw, float pitch)
{
    if(yaw < MIN_ABSOLUTE_YAW_POSITION)
        yaw = (float) MIN_ABSOLUTE_YAW_POSITION;
    else if(yaw > MAX_ABSOLUTE_YAW_POSITION)
        yaw = MAX_ABSOLUTE_YAW_POSITION;

    if(pitch < MIN_ABSOLUTE_PITCH_POSITION)
        pitch = (float) MIN_ABSOLUTE_PITCH_POSITION;
    else if(pitch > MAX_ABSOLUTE_PITCH_POSITION)
        pitch = MAX_ABSOLUTE_PITCH_POSITION;

    target_absolute_yaw = yaw;
    target_absolute_pitch = pitch;
}

void Gimbal_Set_Control_Target_Relative_Yaw_Pitch(uint16_t yaw, uint16_t pitch)
{
    if(yaw < MIN_RELATIVE_YAW_POSITION)
        yaw = MIN_RELATIVE_YAW_POSITION;
    else if(yaw > MAX_RELATIVE_YAW_POSITION)
        yaw = MAX_RELATIVE_YAW_POSITION;

    if(pitch < MIN_RELATIVE_PITCH_POSITION)
        pitch = MIN_RELATIVE_PITCH_POSITION;
    else if(pitch > MAX_RELATIVE_PITCH_POSITION)
        pitch = MAX_RELATIVE_PITCH_POSITION;

    target_relative_yaw = yaw;
    target_relative_pitch = pitch;
}

void Gimbal_Consume_Control_Target_Absolute_Yaw_Pitch_Speed(float yaw, float pitch)
{
    yaw += target_absolute_yaw;
    pitch += target_absolute_pitch;
    Gimbal_Set_Control_Target_Absolute_Yaw_Pitch(yaw, pitch);
}


void Gimbal_Motor_Absolute_Control(DMP_Data *dmp_data)
{
    int16_t yaw_output, pitch_output;

    yaw_output = (int16_t) PID_Absolute_Control(&pid_ctrl_yaw_absolute, dmp_data->yaw, target_absolute_yaw, 0);
    pitch_output = (int16_t) PID_Absolute_Control(&pid_ctrl_pitch_absolute, dmp_data->pitch, target_absolute_pitch, 0);
    Gimbal_Motor_Set_Speed(pitch_output, yaw_output);
}

void Gimbal_Motor_Relative_Control(uint8_t timer_id)
{
    // TODO: center the center
    int16_t yaw_output, pitch_output;
    yaw_output = (int16_t) PID_Absolute_Control(&pid_ctrl_yaw_relative, gms_yaw.angle, target_relative_yaw, 0);
    pitch_output = (int16_t) PID_Absolute_Control(&pid_ctrl_pitch_relative, gms_pitch.angle, target_relative_pitch, 0);
    Gimbal_Motor_Set_Speed(pitch_output, yaw_output);
}

void Gimbal_Start_Standard_Position()
{
    Gimbal_End_Track_Position();
    Gimbal_Set_Control_Target_Relative_Yaw_Pitch(STANDARD_RELATIVE_YAW_POSITION, STANDARD_RELATIVE_PITCH_POSITION);
    if(relative_control_id == -1)
        relative_control_id = Timer_Register(1, Gimbal_Motor_Relative_Control);
}

void Gimbal_End_Standard_Position()
{
    if(relative_control_id != -1) {
        Timer_Unregister(relative_control_id);
        relative_control_id = -1;
    }
}

void Gimbal_Start_Track_Position()
{
    Gimbal_End_Standard_Position();
    absolute_control_enable = 1;
}

void Gimbal_End_Track_Position()
{
    absolute_control_enable = 0;
}

void Gimbal_Motor_Can_Receive_Handler(CanRxMsg* rx_msg)
{
    uint16_t type,id;
    struct Motor_Gimbal_Status *mgs;

    type = (uint16_t) (rx_msg->StdId & CAN_PACKET_TYPE_MASK_DATATYPE);
    id = (uint16_t) (rx_msg->StdId & CAN_PACKET_TYPE_MASK_SOURCEADDR_STRICT);
    mgs = (struct Motor_Gimbal_Status *) rx_msg->Data;

    if(id == CAN_GIMBAL_MOTOR_PITCH_ADDR)
    {
        gms_pitch.angle = mgs->angle_high << 8 | mgs->angle_low;
        gms_pitch.actual_current = mgs->actual_current_high << 8 | mgs->actual_current_low;
        gms_pitch.demand_current = mgs->demand_current_high << 8 | mgs->demand_current_low;
        gms_pitch.hall_state = mgs->hall_state;
    }
    else if(id == CAN_GIMBAL_MOTOR_YAW_ADDR)
    {
        gms_yaw.angle = mgs->angle_high << 8 | mgs->angle_low;
        gms_yaw.actual_current = mgs->actual_current_high << 8 | mgs->actual_current_low;
        gms_yaw.demand_current = mgs->demand_current_high << 8 | mgs->demand_current_low;
        gms_yaw.hall_state = mgs->hall_state;
    }

}

void Gimbal_Motor_Can_Send_Handler(uint16_t id, int8_t code)
{
    if(code == 0)printf("%u send failed from can1\r\n", id);
}

#define GIMBAL_PACKET_ID_SET_SPEED 0x0
#define GIMBAL_BROADCAST_ADDR 0x200

void Gimbal_Motor_Set_Speed(int16_t pitch_speed, int16_t yaw_speed)
{
    static struct Gimbal_Motor_Set_Speed gmss;

    if(pitch_speed > GIMBAL_MOTOR_MAX_SET_SPEED) pitch_speed = GIMBAL_MOTOR_MAX_SET_SPEED;
    else if(pitch_speed < GIMBAL_MOTOR_MIN_SET_SPEED) pitch_speed = GIMBAL_MOTOR_MIN_SET_SPEED;
    if(yaw_speed > GIMBAL_MOTOR_MAX_SET_SPEED) yaw_speed = GIMBAL_MOTOR_MAX_SET_SPEED;
    else if(yaw_speed < GIMBAL_MOTOR_MIN_SET_SPEED) yaw_speed = GIMBAL_MOTOR_MIN_SET_SPEED;


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

