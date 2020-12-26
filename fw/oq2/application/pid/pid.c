/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\pid\pid.c                           /
 * Project: OQ2                                                                                    /
 * Created Date: Thursday, December 24th 2020, 5:08:22 pm                                          /
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


#include "pid.h"
#include "pid_config.h"
#include <string.h>
#include <stdint.h>


#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(PID_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(PID_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(PID_MODULE_ID, fmt, ##__VA_ARGS__)


/**
 * @brief Init PID context with user provided parameters, and default values
 * 
 * @param p_ctx pointer to pid_ctx_t struct
 * @param p_params P, I, D coefficients contained in struct pid_params_t
 * @param p_limits If limits are needed on any of the values, like integral or derivative
 */
void pid_initialize_ctx(pid_ctx_t * pctx, 
                        pid_params_t *pparams, 
                        pid_limits_t *plimits)
{
    memset(pctx, 0x00, sizeof(pid_ctx_t));

    // Apply user settings
    memcpy(&pctx->params, pparams, sizeof(pid_params_t));
    memcpy(&pctx->limits, plimits, sizeof(pid_limits_t));

    // Apply first time values 
    pctx->err_prev = PID_VALUE_INVALID;
    pctx->err_integral = 0;
    pctx->output_prev = PID_VALUE_INVALID;
    pctx->last_input = PID_VALUE_INVALID;

    debug_printf("Init PID@0x%08X", pctx);
}

float pid_calculate(pid_ctx_t * pctx, float new_state, float dt)
{
    float error = pctx->setpoint - new_state;

    float pterm, iterm, dterm;
    float output;

    if(dt == 0)
        return 0;

    // Update the integral
    pctx->err_integral += error * dt;

    pterm = pctx->params.P * error;
    iterm = pctx->params.I * pctx->err_integral;
    dterm = pctx->params.D * (error - pctx->err_prev) / dt;

    if(pctx->err_prev == PID_VALUE_INVALID)
    {
        dterm = 0;
    }

    // char sign = (dterm >= 0) ? '+' : '-';

    // debug_printf("%3.5f,%3.5f,%3.5f, %c", error, pctx->err_prev, dterm,  sign);

    pctx->err_prev = error;

    // Constrain the proportional error to the limit
    if(fabs(pterm) > pctx->limits.P_lim && pctx->limits.P_lim != 0)
    {
        pterm = (pterm > 0) ? pctx->limits.P_lim : -1 * pctx->limits.P_lim;
    }

    // Constrain the integral to the limit
    if(fabs(pctx->err_integral) > pctx->limits.I_lim && pctx->limits.I_lim != 0)
    {
        pctx->err_integral = (pctx->err_integral > 0) ? pctx->limits.I_lim : -1*pctx->limits.I_lim;
    }

    // Constrain the derivative to the limit
    if(fabs(dterm) > pctx->limits.D_lim && pctx->limits.D_lim != 0)
    {
        dterm = (dterm > 0) ? pctx->limits.D_lim : -1*pctx->limits.D_lim;
    }

    // Calculate result
    output = pterm + iterm + dterm;

    // debug_printf("%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,%3.4f,%3.4f", new_state, 
    //                                                         pctx->setpoint,
    //                                                         error,
    //                                                         pterm,
    //                                                         iterm,
    //                                                         dterm,
    //                                                         output);

    pctx->output_prev = output;

    return output;
}
