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
#include "math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD(x) (x * M_PI / 180.0)
#define RAD_TO_DEG(x) (x * 180.0 / M_PI)

typedef struct kinematics_data
{
    double pitch,
    roll,
    yaw,
    phi,
    
    gpitch, 
    groll,
    gyaw,
    
    pitchRate,
    rollRate,
    yawRate,
    
    vx, vy,

    altitude,
    climbRate,
    prevClimbRate,

    io_ax,
    io_ay,
    io_az,
    io_wx,
    io_wy,
    io_wz,

    yaw_mag;

    uint32_t timestamp;
} kinematics_data_t;

uint32_t kinematics_initialize();
uint32_t kinematics_update_accel_gyro(float * accel_mg, float * rate_mdps);

float kinematics_get_pitch();
float kinematics_get_roll();


#endif