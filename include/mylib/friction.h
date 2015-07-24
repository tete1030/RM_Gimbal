#ifndef __PWM_H__
#define __PWM_H__

typedef enum Friction_State
{
    Friction_State_Off = 0,
    Friction_State_Turning_Off,
    Friction_State_Turning_On,
    Friction_State_On,
    Friction_State_Set_Range,
    Friction_State_Mannual_Control,
    Friction_State_Prepare
} Friction_State;

typedef enum Friction_Start_Mode
{
    Friction_Start_Mode_Normal,
    Friction_Start_Mode_Set_Range,
    Friction_Start_Mode_Mannual_Control
} Friction_Start_Mode;

void Friction_Init(Friction_Start_Mode fsm);

void Friction_Set_Enable(uint8_t enable);
Friction_State Friction_Get_State();
void Friction_Set_Remoter_Value(uint16_t value);

#endif


