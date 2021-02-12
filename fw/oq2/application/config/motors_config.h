/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\config\motors_config.h              /
 * Project: OQ2                                                                                    /
 * Created Date: Saturday, December 26th 2020, 10:41:01 am                                         /
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

#ifndef MOTORS_CONFIG_H
#define MOTORS_CONFIG_H

#include "app_config.h"

#define MC_NUM_MOTORS               4
#define MC_PWM_PERIOD               3097.2217f

/**
 * The actual startup for the default stspin32f0a is 1060 us, 
 * but the drive becomes smooth at 1130.
 * 
 */
#define MC_STARTUP_TON_US_PERIOD    900
#define MC_MIN_TON_US_PERIOD        1061
#define MC_MAX_TON_US_PERIOD        2084
#define MC_TON_US_RANGE             (MC_MAX_TON_US_PERIOD - MC_MIN_TON_US_PERIOD)

#endif
