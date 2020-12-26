/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\config\stability_config.h           /
 * Project: OQ2                                                                                    /
 * Created Date: Thursday, December 24th 2020, 11:05:25 am                                         /
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


#ifndef _STABILITY_CONFIG_H_
#define _STABILITY_CONFIG_H_

#include "app_config.h"

/***************************************************************************************************
STM32 MotionFX lib config
*/
#define SC_ENABLE_6AXIS_MFX_LIB     APP_CONFIG_ENABLED
#define SC_ENABLE_9AXIS_MFX_LIB     APP_CONFIG_DISABLED


#define SC_OP_MODE_NORMAL           0
#define SC_OP_MODE_MAG_CAL          1

#define SC_OPERATING_MODE           SC_OP_MODE_NORMAL

/***************************************************************************************************
MEMS Config
*/
#define SC_MEMS_DETECT_RETRY_MAX    10

/***************************************************************************************************
Default Sensor Offsets, cal values, etc
*/
#define SC_MFX_ATIME                    (0.899)
// #define SC_MFX_ATIME                    (0.2)
#define SC_FACTORY_PITCH_OFFSET_DEG     (1.870343)
#define SC_FACTORY_ROLL_OFFSET_DEG      (-0.854811)

#define SC_FACTORY_1G_VALUE_MPS         (9.81f)

#define SC_SEA_LEVEL_PRESS_HPA          (1013.25)
#define SC_AIR_TEMPERATURE              (-4)

/***************************************************************************************************
Attitude PID values
*/
#define SC_PITCH_PID_P                  1
#define SC_PITCH_PID_I                  1
#define SC_PITCH_PID_D                  0
#define SC_PITCH_PID_P_LIM              100
#define SC_PITCH_PID_I_LIM              100
#define SC_PITCH_PID_D_LIM              0

#define SC_ROLL_PID_P                   1
#define SC_ROLL_PID_I                   1
#define SC_ROLL_PID_D                   0
#define SC_ROLL_PID_P_LIM               100
#define SC_ROLL_PID_I_LIM               100
#define SC_ROLL_PID_D_LIM               0

#define SC_YAW_PID_P                    0.1
#define SC_YAW_PID_I                    0.1
#define SC_YAW_PID_D                    0
#define SC_YAW_PID_P_LIM                10
#define SC_YAW_PID_I_LIM                10
#define SC_YAW_PID_D_LIM                0


#endif