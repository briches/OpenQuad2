/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\motors\motors.h                     /
 * Project: OQ2                                                                                    /
 * Created Date: Saturday, December 26th 2020, 10:40:38 am                                         /
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

#ifndef MOTORS_H_
#define MOTORS_H_

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim24;

typedef enum motor_armed
{
    MOTOR_DISARMED = 0,
    MOTOR_ARMED = 1,
} motor_arm_t;

typedef struct motors
{
    bool armed;

    uint8_t index;

    float period_us;

    // PWM Timer ref
    TIM_HandleTypeDef * htimer;
    uint32_t channel;
    __IO uint32_t ccr;

    // GPIO pin ref
    GPIO_pin_t arm_pin;

} motor_t;


void        motor_controllers_init              ( void );
void        motor_controllers_set_arm_state     (uint8_t index, motor_arm_t armed);
motor_arm_t motor_controllers_get_arm_state     (uint8_t index);
uint8_t     motors_get_thrust_setting           ();
void        motors_set_thrust_setting           (uint8_t new_thrust_setting);
void        motor_controllers_control_input     (float * pitch_ctrl, float * roll_ctrl, float * yaw_ctrl);


#endif
