/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\kinematics\kinematics.h             /
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, December 13th 2020, 11:05:48 am                                           /
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


#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <stdint.h>
#include <stdbool.h>
#include "arm_math.h"
#include "motion_fx.h"

#define DEG_TO_RAD(x) (x * PI / 180.0)
#define RAD_TO_DEG(x) (x * 180.0 / PI)

typedef struct kinematics_data
{
    // Angles in degrees
    float pitch,
    roll,
    yaw,

    // Angular rates in dps
    pitch_rate,
    roll_rate,
    yaw_rate,

    // Positions
    x_inertial,
    y_inertial,
    z_inertial,
    z_baro,

    // Velocities
    vx_inertial,
    vy_inertial,
    vz_inertial,
    vz_baro;

    // Time in seconds
    float timestamp;

    bool lib_mode_9;

    MFX_output_t mfx_data;

} kinematics_ctx_t;

uint32_t kinematics_initialize(kinematics_ctx_t * pctx, bool lib_mode_9);
uint32_t kinematics_new_motionfx_data_callback(kinematics_ctx_t * pctx, MFX_output_t * p_data);
uint32_t kinematics_new_elevation_data_callback(kinematics_ctx_t * pctx, float elevation);

float kinematics_get_pitch();
float kinematics_get_roll();


#endif