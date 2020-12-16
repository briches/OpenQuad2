/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\kalman\kalman.c                     /
 * Project: OQ2                                                                                    /
 * Created Date: Monday, December 14th 2020, 5:36:18 pm                                            /
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

#include "kalman.h"
#include <string.h>

/**
 * @brief Initialize a kalman context to the default values
 * 
 * @param pctx 
 */
void kalman_init(kalman_filter_ctx_t * pctx)
{
    pctx->q_angle = KALMAN_DEFAULT_Q_ANGLE;
    pctx->q_bias = KALMAN_DEFAULT_Q_BIAS;
    pctx->r_measure = KALMAN_DEFAULT_R_MEASURE;
    pctx->angle = 0.0f; 
    pctx->bias = 0.0f;

    // Since we assume that the bias is 0 and we know the starting angle (use setAngle), 
    // the error covariance matrix is set like so - 
    // see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    memset(pctx->P, 0x00, sizeof(pctx->P));
}

/**
 * @brief Calculate a new angle estimate based off of new sensor input and the previous state
 * 
 * @param pctx pointer to kalman_filter_ctx_t 
 * @param new_angle degrees 
 * @param new_rate degrees per second 
 * @param dt delta time in seconds 
 * @return float new calculated angle estimate 
 */
float kalman_calulate_new(kalman_filter_ctx_t * pctx, float new_angle, float new_rate, float dt)
{
    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    pctx->rate = new_rate - pctx->bias;
    pctx->angle += dt * pctx->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    pctx->P[0][0] += dt * (dt * pctx->P[1][1] - pctx->P[0][1] - pctx->P[1][0] + pctx->q_angle);
    pctx->P[0][1] -= dt * pctx->P[1][1];
    pctx->P[1][0] -= dt * pctx->P[1][1];
    pctx->P[1][1] += pctx->q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = pctx->P[0][0] + pctx->r_measure; // Estimate error

    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = pctx->P[0][0] / S;
    K[1] = pctx->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = new_angle - pctx->angle; // Angle difference
    /* Step 6 */
    pctx->angle += K[0] * y;
    pctx->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = pctx->P[0][0];
    float P01_temp = pctx->P[0][1];

    pctx->P[0][0] -= K[0] * P00_temp;
    pctx->P[0][1] -= K[0] * P01_temp;
    pctx->P[1][0] -= K[1] * P00_temp;
    pctx->P[1][1] -= K[1] * P01_temp;

    return pctx->angle;
}