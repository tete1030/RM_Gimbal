//
// Created by Texot Qexoq on 5/17/15.
//

#ifndef __GIMBAL_MOTOR_H__
#define __GIMBAL_MOTOR_H__

#include <stdint.h>
#include "mpu6050/mpu6050.h"

typedef struct Gimbal_Motor_Status
{
    uint16_t angle;
    int16_t actual_current;
    int16_t demand_current;
    uint8_t hall_state;
} Gimbal_Motor_Status;

extern Gimbal_Motor_Status gms_yaw;
extern Gimbal_Motor_Status gms_pitch;

void Gimbal_Motor_Configuration(void);

void Gimbal_Set_Control_Target_Absolute_Yaw_Pitch(float yaw, float pitch);
void Gimbal_Set_Control_Target_Relative_Yaw_Pitch(uint16_t yaw, uint16_t pitch);
void Gimbal_Consume_Control_Target_Absolute_Yaw_Pitch_Speed(float yaw, float pitch);


void Gimbal_Start_Standard_Position();
void Gimbal_End_Standard_Position();
void Gimbal_Start_Track_Position();
void Gimbal_End_Track_Position();

void Gimbal_Motor_Set_Speed(int16_t pitch_speed, int16_t yaw_speed);


void Gimbal_Motor_Absolute_Control(DMP_Data *dmp_data);
extern uint8_t absolute_control_enable;

#endif //__GIMBAL_MOTOR_H__
