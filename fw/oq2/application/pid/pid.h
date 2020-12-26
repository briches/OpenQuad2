/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\pid\pid.h                           /
 * Project: OQ2                                                                                    /
 * Created Date: Thursday, December 24th 2020, 5:08:19 pm                                          /
 * Author: Brandon Riches                                                                          /
 * Email: richesbc@gmail.com                                                                       /
 * -----                                                                                           /
 *                                                                                                 /
 * Copyright (c) 2020 OpenQuad2.                                                                   /
 * All rights reserved.                                                                            /
 *                                                                                                 /
 * Redistribution and use in source or binary forms, with or without modification,                 /
 * are not permitted without express written approval of OpenQuad2                                 /
 * -----                                                                                           /
 * HISTORY:                                                                                        /
*/


#ifndef PID_H__
#define PID_H__
#include <stdint.h>
#include <limits.h>
#include <math.h>

#define PID_VALUE_INVALID   (-1*HUGE_VALF)

typedef struct
{
    float P;
    float I;
    float D;
} pid_params_t;

typedef struct
{
    float P_lim;    // Limit the P * err term
    float I_lim;    // Limit the size of the integral
    float D_lim;    // Limit the derivative
} pid_limits_t;


/**
 * @brief Main PID context struct.
 * 
 * Holds all state info, parameters
 * 
 */
typedef struct pid
{
  
    // Tuning parameters
    pid_params_t params;

    // Integral term will not be allowed to exceed this value
    // Derivative term will not be allowed to exceed this value
    // Proportional term will not be allowed to exceed this value
    pid_limits_t limits;

    // The previous calculated error
    float err_prev;

    // The integrating value
    float err_integral;

    // The previous control output
    float output_prev;

    // The previous measurement input
    float last_input;

    // The setpoint
    float setpoint;
} pid_ctx_t;


void pid_initialize_ctx(pid_ctx_t * pctx, 
                        pid_params_t *pparams, 
                        pid_limits_t *plimits);

float pid_calculate(pid_ctx_t * pctx, float new_state, float dt);


#endif