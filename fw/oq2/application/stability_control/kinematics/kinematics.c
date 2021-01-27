/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\kinematics\kinematics.c             /
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, December 13th 2020, 11:05:52 am                                           /
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

#include "kinematics.h"
#include "main.h"
#include "stability_config.h"
#include "timer.h"
#include "arm_math.h"

#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(KINEMATICS_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(KINEMATICS_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(KINEMATICS_MODULE_ID, fmt, ##__VA_ARGS__)

#define KINEMATICS_LOG_HEADER "Roll Rate, Raw Roll, Kalman Roll, Pitch Rate, Raw Pitch, Kalman Pitch"

#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2
/*********************************************************************************************/
/* FIR Filter Parameters and Memory ---------------------------------------------------------*/
// Coefficients for angle filters on pitch, roll, yaw
static float fir_coeffs[SC_FIR_NUM_TAPS] = 
{0.16596037, 0.23995298, 0.25911969, 0.20166299, 0.10638556, 0.02479441};

// Memory for arm_fir filter state, pitch, roll, and yaw
static float pitch_fir_state[SC_FIR_BLOCK_SIZE + SC_FIR_NUM_TAPS - 1];
static float roll_fir_state[SC_FIR_BLOCK_SIZE + SC_FIR_NUM_TAPS - 1];
static float yaw_fir_state[SC_FIR_BLOCK_SIZE + SC_FIR_NUM_TAPS - 1];

// Arm math FIR instances for pitch, roll, yaw
static arm_fir_instance_f32 pitch_fir_inst;
static arm_fir_instance_f32 roll_fir_inst;
static arm_fir_instance_f32 yaw_fir_inst;



static inline float _kinematics_get_seconds()
{
    return timer_get_elapsed();
}

uint32_t kinematics_initialize(kinematics_ctx_t * pctx, bool lib_mode_9)
{
    if(pctx == NULL)
        return -1;

    memset(pctx, 0x00, sizeof(kinematics_ctx_t));

    pctx->lib_mode_9 = lib_mode_9;

    pctx->timestamp = _kinematics_get_seconds();

    debug_printf("Kinematics init");

    // Init arm_math lib finite impulse response filters
    arm_fir_init_f32(&pitch_fir_inst, SC_FIR_NUM_TAPS, fir_coeffs, pitch_fir_state, SC_FIR_BLOCK_SIZE);
    arm_fir_init_f32(&roll_fir_inst, SC_FIR_NUM_TAPS, fir_coeffs, roll_fir_state, SC_FIR_BLOCK_SIZE);
    arm_fir_init_f32(&yaw_fir_inst, SC_FIR_NUM_TAPS, fir_coeffs, yaw_fir_state, SC_FIR_BLOCK_SIZE);


    return 1;
}

uint32_t kinematics_new_motionfx_data_callback(kinematics_ctx_t * pctx, MFX_output_t * p_data)
{
    float yaw, pitch, roll;

    // Get the current time, and calculate the delta since the last call. 
    float time_s = _kinematics_get_seconds();
    float dt_s = time_s - pctx->timestamp;
    pctx->timestamp = time_s;

    // Store MFX output data
    memcpy(&pctx->mfx_data, p_data, sizeof(MFX_output_t));

    // Store offset angles
    if(pctx->lib_mode_9)
    {
        arm_fir_f32(&yaw_fir_inst, &p_data->rotation_9X[0], &pctx->yaw, SC_FIR_BLOCK_SIZE);
        arm_fir_f32(&pitch_fir_inst, &p_data->rotation_9X[1], &pctx->pitch, SC_FIR_BLOCK_SIZE);
        arm_fir_f32(&roll_fir_inst, &p_data->rotation_9X[2], &pctx->roll, SC_FIR_BLOCK_SIZE);
        pctx->pitch -= SC_FACTORY_PITCH_OFFSET_DEG;
        pctx->roll -= SC_FACTORY_ROLL_OFFSET_DEG;
    }
    else
    {
        arm_fir_f32(&yaw_fir_inst, &p_data->rotation_6X[0], &pctx->yaw, SC_FIR_BLOCK_SIZE);
        arm_fir_f32(&pitch_fir_inst, &p_data->rotation_6X[1], &pctx->pitch, SC_FIR_BLOCK_SIZE);
        arm_fir_f32(&roll_fir_inst, &p_data->rotation_6X[2], &pctx->roll, SC_FIR_BLOCK_SIZE);
        pctx->pitch -= SC_FACTORY_PITCH_OFFSET_DEG;
        pctx->roll -= SC_FACTORY_ROLL_OFFSET_DEG;
    }

    // if(dt_s != 0)
    // {
    //     pctx->pitch_rate = (pitch - pctx->pitch) / dt_s;
    //     pctx->roll_rate = (roll - pctx->roll) / dt_s;
    //     pctx->yaw_rate = (yaw - pctx->yaw) / dt_s;
    // }

    pctx->vx_inertial += p_data->linear_acceleration_6X[0] * dt_s;
    pctx->vy_inertial += p_data->linear_acceleration_6X[1] * dt_s;
    pctx->vz_inertial += p_data->linear_acceleration_6X[2] * dt_s;

    pctx->x_inertial += pctx->vx_inertial * dt_s;
    pctx->y_inertial += pctx->vy_inertial * dt_s;
    pctx->z_inertial += pctx->vz_inertial * dt_s;


    // debug_printf("x, %3.1f, y, %3.1f, z, %3.1f", pctx->x_inertial, pctx->y_inertial, pctx->z_inertial);
    // debug_printf("x, %3.3f, y, %3.3f, z, %3.3f", p_data->linear_acceleration_6X[0], 
    //                                             p_data->linear_acceleration_6X[1], 
    //                                             p_data->linear_acceleration_6X[2]);
    // debug_printf("%4.4f, %4.4f", p_data->rotation_6X[1], pctx->pitch);

    // debug_printf(" heading: %3.3f, (%3.3f)", p_data->heading_6X, p_data->headingErr_6X);

    // debug_printf(", %3.5f", dt_s);

    return 1;
}

uint32_t kinematics_new_elevation_data_callback(kinematics_ctx_t * pctx, float elevation)
{
    pctx->z_baro = elevation;

    return 1;
}


