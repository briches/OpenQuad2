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

/**
 * @brief Init PID context with user provided parameters, and default values
 * 
 * @param p_ctx pointer to pid_ctx_t struct
 * @param p_params P, I, D coefficients contained in struct pid_params_t
 * @param p_limits If limits are needed on any of the values, like integral or derivative
 */
void pid_initialize_ctx(pid_ctx_t * p_ctx, 
                        pid_params_t *p_params, 
                        pid_limits_t *p_limits)
{
    memset(p_ctx, 0x00, sizeof(pid_ctx_t));

    // Apply user settings
    memcpy(&p_ctx->params, p_params, sizeof(pid_params_t));
    memcpy(&p_ctx->limits, p_limits, sizeof(pid_limits_t));

    // Apply first time values 
    p_ctx->err_prev = PID_VALUE_INVALID;
    p_ctx->err_integral = PID_VALUE_INVALID;
    p_ctx->output_prev = PID_VALUE_INVALID;
    p_ctx->last_input = PID_VALUE_INVALID;
}

