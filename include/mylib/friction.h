#ifndef __PWM_H__
#define __PWM_H__

void Friction_Configuration(void);

void Friction_Set_Enable(uint8_t enable);
uint8_t Friction_Get_State();
void Friction_Init_Speed_Controller();

#endif


