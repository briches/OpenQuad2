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
#include "kalman.h"

#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(KINEMATICS_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(KINEMATICS_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(KINEMATICS_MODULE_ID, fmt, ##__VA_ARGS__)

#define KINEMATICS_LOG_HEADER "Roll Rate, Raw Roll, Kalman Roll, Pitch Rate, Raw Pitch, Kalman Pitch"

#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2

static kalman_filter_ctx_t roll_ctx;
static kalman_filter_ctx_t pitch_ctx;

uint32_t kinematics_initialize()
{
    kalman_init(&roll_ctx);
    kalman_init(&pitch_ctx);

    debug_printf("%s", KINEMATICS_LOG_HEADER);

    return 1;
}

int32_t last_time = -1;

uint32_t kinematics_update_accel_gyro(float * accel_mg, float * rate_mdps)
{
    uint32_t timestamp_now = HAL_GetTick();
    float delta_time = (last_time < 0) ? 0 : timestamp_now - last_time;
    delta_time /= 1000.0f;

    // Calculate the squares
    float x_sq = accel_mg[X_INDEX] * accel_mg[X_INDEX];
    float y_sq = accel_mg[Y_INDEX] * accel_mg[Y_INDEX];
    float z_sq = accel_mg[Z_INDEX] * accel_mg[Z_INDEX];

    // Calculate the total magnitude
    float magnitudeApprox = sqrt(x_sq + y_sq + z_sq);

    // Calculate angles roll and pitch
    float roll_accel = atan2(accel_mg[Y_INDEX], sqrt(z_sq + x_sq));
    float pitch_accel = -atan2(accel_mg[X_INDEX], sqrt(y_sq + z_sq));

    // Convert to degrees
    roll_accel = RAD_TO_DEG(roll_accel);
    pitch_accel = RAD_TO_DEG(pitch_accel);

    // Get rotational rate
    float roll_rate_dps = -1.0 * rate_mdps[X_INDEX] / 1000.0f;
    float pitch_rate_dps = -1.0f * rate_mdps[Y_INDEX] / 1000.0f;

    kalman_calulate_new(&roll_ctx, roll_accel, roll_rate_dps, delta_time);
    kalman_calulate_new(&pitch_ctx, pitch_accel, pitch_rate_dps, delta_time);

    // #define KINEMATICS_LOG_HEADER "Roll Rate, Raw Roll, Kalman Roll, Pitch Rate, Raw Pitch, Kalman Pitch"
    // debug_printf(", %3.1f, %3.1f, %3.1f, %3.1f, %3.1f, %3.1f", 
    // roll_rate_dps, roll_accel, roll_ctx.angle, pitch_rate_dps, pitch_accel, pitch_ctx.angle);
    // debug_printf(", %3.1f, %3.1f", roll_accel, roll_ctx.angle);
    // debug_printf(", %3.1f, %3.1f", -5.0f, 5.0f);

    last_time = timestamp_now;
    return 0;
}