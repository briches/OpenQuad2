/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\kalman\kalman.h                     /
 * Project: OQ2                                                                                    /
 * Created Date: Monday, December 14th 2020, 5:30:43 pm                                            /
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
/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */
// https://github.com/TKJElectronics/KalmanFilter


#ifndef _KALMAN_H__
#define _KALMAN_H__

#include "main.h"

// Decrease Q to increase following of input
// Increase Q to increase filtering
#define KALMAN_DEFAULT_Q_ANGLE      0.01f
#define KALMAN_DEFAULT_Q_BIAS       0.01f
#define KALMAN_DEFAULT_R_MEASURE    0.03f

typedef struct kalman_filter_ctx
{
    /* Kalman filter variables */
    float q_angle; // Process noise variance for the accelerometer
    float q_bias; // Process noise variance for the gyro bias
    float r_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
} kalman_filter_ctx_t;

// Init context to default
void    kalman_init(    kalman_filter_ctx_t * ctx);

// Calculation routines
float   kalman_calulate_new(   kalman_filter_ctx_t * ctx, float new_angle, float new_rate, float dt);


#endif