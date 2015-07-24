//
// Created by Texot Qexoq on 5/17/15.
//

#ifndef __GIMBAL_MOTOR_H__
#define __GIMBAL_MOTOR_H__

#include <stdint.h>
#include "mpu6050/mpu6050.h"

typedef struct GM_Status
{
    int16_t angle;
    int16_t actual_current;
    int16_t demand_current;
    uint8_t hall_state;
} GM_Status;

extern volatile GM_Status gms_yaw;
extern volatile GM_Status gms_pitch;

void Gimbal_Motor_Configuration(void);
void GM_Set_DMP_Data(DMP_Data *dd);
//void GM_Set_Control_Target_Absolute_Yaw_Pitch(float yaw, float pitch);
void GM_Set_Control_Target_Relative_Yaw_Pitch(int16_t yaw, int16_t pitch);
void GM_Set_Control_Target_Absolute_Yaw_Pitch_Speed(float yaw, float pitch);

void GM_Start_Relative_Control();
void GM_Stop_Relative_Control();
void GM_Stop_Pitch_Relative_Control();
void GM_Set_Standard_Position();

void GM_Start_Absolute_Control();
void GM_Stop_Absolute_Control();

void GM_Set_Speed(int16_t pitch_speed, int16_t yaw_speed);

void GM_Absolute_Control();
void GM_Relative_Control();


#endif //__GIMBAL_MOTOR_H__
