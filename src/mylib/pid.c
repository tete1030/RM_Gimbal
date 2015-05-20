//
// Created by Texot Qexoq on 5/18/15.
//

#include <stdint.h>
#include <stdio.h>
#include "pid.h"

void PID_Init(PID_Controller *pidc, float kp, float ki, float kd, float ko, float max_output, float min_output, float max_integral, float min_integral)
{
    pidc->kp = kp;
    pidc->ki = ki;
    pidc->kd = kd;
    pidc->ko = ko;

    pidc->pre_error[0] = 0;
    pidc->pre_error[1] = 0;
    pidc->pre_error[2] = 0;

    pidc->integral = 0;

    pidc->cur_error_offset = 0;
    pidc->last_output = 0;

    pidc->max_output = max_output;
    pidc->min_output = min_output;

    pidc->max_integral = max_integral;
    pidc->min_integral = min_integral;
}
extern PID_Controller pid_ctrl_pitch_absolute;


float PID_Incremental_Control(PID_Controller *pidc, float cur_value, float target_value, float other)
{
    float output;
    pidc->pre_error[pidc->cur_error_offset] = target_value - cur_value;

    output = pidc->kp * (pidc->pre_error[pidc->cur_error_offset] - pidc->pre_error[(pidc->cur_error_offset + 2) % 3])
             + pidc->ki * pidc->pre_error[pidc->cur_error_offset]
             + pidc->kd *
                        (pidc->pre_error[pidc->cur_error_offset]
                         - 2 * pidc->pre_error[(pidc->cur_error_offset + 2) % 3]
                         + pidc->pre_error[(pidc->cur_error_offset + 1) % 3])
             + pidc->ko * other;

    pidc->cur_error_offset = (uint8_t) ((pidc->cur_error_offset + 1) % 3);

    output += pidc->last_output;

    if(output > pidc->max_output)
        output = pidc->max_output;
    else if(output < pidc->min_output)
        output = pidc->min_output;

    pidc->last_output = output;

    return output;
};

float PID_Absolute_Control(PID_Controller *pidc, float cur_value, float target_value, float other)
{
    float output;
    pidc->pre_error[pidc->cur_error_offset] = target_value - cur_value;

    pidc->integral += pidc->pre_error[pidc->cur_error_offset];

    if(pidc->integral < pidc->min_integral)
        pidc->integral = pidc->min_integral;
    else if(pidc->integral > pidc->max_integral)
        pidc->integral = pidc->max_integral;

    output = pidc->kp * pidc->pre_error[pidc->cur_error_offset]
             + pidc->ki * pidc->integral
             + pidc->kd *
               (pidc->pre_error[pidc->cur_error_offset]
                - pidc->pre_error[(pidc->cur_error_offset + 2) % 3])
             + pidc->ko * other;

    pidc->cur_error_offset = (uint8_t) ((pidc->cur_error_offset + 1) % 3);

    if(output > pidc->max_output)
        output = pidc->max_output;
    else if(output < pidc->min_output)
        output = pidc->min_output;

    return output;
}