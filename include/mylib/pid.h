//
// Created by Texot Qexoq on 5/18/15.
//

#ifndef __PID_H__
#define __PID_H__


typedef struct PID_Controller
{
    float kp;
    float ki;
    float kd;
    float ko;


    float pre_error[3];
    float integral;
    uint8_t cur_error_offset;
    float last_output;

    float max_output;
    float min_output;

    float max_integral;
    float min_integral;
} PID_Controller;

void PID_Init(PID_Controller *pidc, float kp, float ki, float kd, float ko, float max_output, float min_output, float max_integral, float min_integral);
float PID_Incremental_Control(PID_Controller *pidc, float cur_value, float target_value, float other);
float PID_Absolute_Control(PID_Controller *pidc, float cur_value, float target_value, float other);


#endif //__PID_H__
