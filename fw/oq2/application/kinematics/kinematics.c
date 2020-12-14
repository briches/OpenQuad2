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


#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(KINEMATICS_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(KINEMATICS_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(KINEMATICS_MODULE_ID, fmt, ##__VA_ARGS__)

#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2

uint32_t kinematics_update_accel_gyro(float * accel_mg, float * rate_mdps)
{
    // Calculate the squares
    float x_sq = accel_mg[X_INDEX] * accel_mg[X_INDEX];
    float y_sq = accel_mg[Y_INDEX] * accel_mg[Y_INDEX];
    float z_sq = accel_mg[Z_INDEX] * accel_mg[Z_INDEX];

    // Calculate the total magnitude
    float magnitudeApprox = sqrt(x_sq + y_sq + z_sq);

    // Calculate angles roll and pitch
    float roll_accel = -atan2(accel_mg[Y_INDEX], sqrt(z_sq + x_sq));
    float pitch_accel = atan2(accel_mg[X_INDEX], sqrt(y_sq + z_sq));

    // Convert to degrees
    roll_accel = RAD_TO_DEG(roll_accel);
    pitch_accel = RAD_TO_DEG(pitch_accel);

    // debug_printf("roll, %3.1f, pitch, %3.1f", roll_accel, pitch_accel);

    return 0;
}