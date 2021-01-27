/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\flight_app\flight_app.h             /
 * Project: OQ2                                                                                    /
 * Created Date: Friday, January 8th 2021, 1:54:47 pm                                              /
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


#ifndef FLIGHT_APP_H
#define FLIGHT_APP_H


typedef enum
{
    FLIGHT_MOTOR_DISARM,
    FLIGHT_RESET,
    FLIGHT_INIT,
    FLIGHT_MOTOR_ARM,
    FLIGHT_ESC_DFU_START,
    FLIGHT_ESC_DFU_ERASE,
    FLIGHT_ESC_DFU_WRITE,
    FLIGHT_ESC_DFU_VERIFY,
    FLIGHT_MOTOR_STARTUP,
    FLIGHT_MOTOR_POST_STARTUP,
    FLIGHT_MOTOR_READY,
} flight_state_t;


void flight_thread(void* argument);
void flight_app_new_stability_inputs(float * pitch_ctrl, float * roll_ctrl, float * yaw_ctrl);
void flight_app_critical_angle_callback();
flight_state_t flight_app_get_state();

#endif