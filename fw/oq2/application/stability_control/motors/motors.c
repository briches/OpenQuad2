/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\motors\motors.c                     /
 * Project: OQ2                                                                                    /
 * Created Date: Saturday, December 26th 2020, 10:40:35 am                                         /
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


#include "motors.h"
#include "main.h"
#include "debug_log.h"
#include "motors_config.h"

#define debug_error(fmt, ...)           debug_error(MOTORS_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(MOTORS_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(MOTORS_MODULE_ID, fmt, ##__VA_ARGS__)

static motor_t motor1;
static motor_t motor2;
static motor_t motor3;
static motor_t motor4;

// Percentage setting of thrust. 
static uint8_t m_thrust_setting = 0;

/**
 * @brief Constrain a value to between MC_MIN_TON_US_PERIOD and MC_MAX_TON_US_PERIOD
 * 
 * @param input_us microseconds period
 * @return float constrained period in microseconds
 */
__always_inline static void _motor_constrain_us(motor_t * motor)
{
    motor->period_us = (motor->period_us > MC_MAX_TON_US_PERIOD) ? 
            MC_MAX_TON_US_PERIOD : 
                (motor->period_us < MC_MIN_TON_US_PERIOD) ?
                    MC_MIN_TON_US_PERIOD : motor->period_us;
} 

static inline void _motor_set_on_period(motor_t * motor)
{
    uint16_t on = (0xFFFF) * (float)(motor->period_us / MC_PWM_PERIOD);
    
    __HAL_TIM_SET_COMPARE(motor->htimer, motor->channel, on);
}

static void _motor_set_arm(motor_t * motor, motor_arm_t arm)
{
    if(arm != MOTOR_ARMED && arm != MOTOR_DISARMED)
        return;

    HAL_GPIO_WritePin(motor->arm_pin.port, motor->arm_pin.pin, (GPIO_PinState) arm);

    if(arm == MOTOR_ARMED)
    {
        debug_printf("ARMING motor %u", motor->index);

        motor->period_us = MC_MIN_TON_US_PERIOD;
        motor->armed = true;

        _motor_set_on_period(motor);
        HAL_TIM_PWM_Start(motor->htimer, motor->channel);
    }
    else
    {
        debug_printf("DISARMING motor %u", motor->index);

        HAL_TIM_PWM_Stop(motor->htimer, motor->channel);
        motor->armed = false;

    }
}

/**
 * @brief Initialize the ARMx GPIO, the PWM peripheral, and set the DISARMED state
 * for all N motors.
 * 
 */
void motor_controllers_init()
{
    motor1.index = 1;
    motor1.arm_pin.pin = BLDC_ARM1_Pin;
    motor1.arm_pin.port = BLDC_ARM1_GPIO_Port;
    motor1.htimer = &htim1;
    motor1.channel = TIM_CHANNEL_3;
    motor1.ccr = motor1.htimer->Instance->CCR3;
    motor1.period_us = 0;

    motor2.index = 2;
    motor2.arm_pin.pin = BLDC_ARM2_Pin;
    motor2.arm_pin.port = BLDC_ARM2_GPIO_Port;
    motor2.htimer = &htim1;
    motor2.channel = TIM_CHANNEL_4;
    motor2.period_us = 0;

    motor3.index = 3;
    motor3.arm_pin.pin = BLDC_ARM3_Pin;
    motor3.arm_pin.port = BLDC_ARM3_GPIO_Port;
    motor3.htimer = &htim24;
    motor3.channel = TIM_CHANNEL_1;
    motor3.period_us = 0;

    motor4.index = 4;
    motor4.arm_pin.pin = BLDC_ARM4_Pin;
    motor4.arm_pin.port = BLDC_ARM4_GPIO_Port;
    motor4.htimer = &htim24;
    motor4.channel = TIM_CHANNEL_4;
    motor4.period_us = 0;

    _motor_set_arm(&motor1, MOTOR_DISARMED);
    _motor_set_arm(&motor2, MOTOR_DISARMED);
    _motor_set_arm(&motor3, MOTOR_DISARMED);
    _motor_set_arm(&motor4, MOTOR_DISARMED);

    _motor_set_on_period(&motor1);
    _motor_set_on_period(&motor2);
    _motor_set_on_period(&motor3);
    _motor_set_on_period(&motor4);
}

/**
 * @brief Arm or disarm a given motor
 * 
 * @param index The index of the motor
 * @param armed the state, ARMED or DISARMED
 */
void motor_controllers_set_arm_state(uint8_t index, motor_arm_t armed)
{
    switch(index)
    {
        case 1:
            _motor_set_arm(&motor1, armed);
        break;
        case 2:
            _motor_set_arm(&motor2, armed);
        break;
        case 3:
            _motor_set_arm(&motor3, armed);
        break;
        case 4:
            _motor_set_arm(&motor4, armed);
        break;
        default:
            break;
    }
}

/**
 * @brief Query the motor controller state (ARMED or DISARMED) for a given index.
 * 
 * @param index motor controller index
 * @return motor_arm_t ARMED or DISARMED
 */
motor_arm_t motor_controllers_get_arm_state(uint8_t index)
{
    switch(index)
    {
        case 1:
            return motor1.armed;
        break;
        case 2:
            return motor2.armed;
        break;
        case 3:
            return motor3.armed;
        break;
        case 4:
            return motor4.armed;
        break;
        default:
            break;
    }
}

/**
 * @brief Handle the new PID control output for pitch, roll, yaw.
 * 
 * @param pitch_ctrl PID control output for pitch
 * @param roll_ctrl PID control output for roll
 * @param yaw_ctrl PID control output for yaw
 */
void motor_controllers_control_input(float * pitch_ctrl, float * roll_ctrl, float * yaw_ctrl)
{
    // Pitch control. 
    // Motor 1 and 2 can INCREASE to INCREASE pitch
    // Motor 3 and 4 can DECREASE to INCREASE pitch
    motor1.period_us += * pitch_ctrl;
    motor2.period_us += * pitch_ctrl;
    motor3.period_us -= * pitch_ctrl;
    motor4.period_us -= * pitch_ctrl;

    // Roll control. 
    // Motor 2 and 3 can INCREASE to INCREASE roll
    // Motor 1 and 4 can DECREASE to INCREASE roll
    motor1.period_us -= * roll_ctrl;
    motor2.period_us += * roll_ctrl;
    motor3.period_us += * roll_ctrl;
    motor4.period_us -= * roll_ctrl;

    // Yaw control. 
    // Motor 1 and 3 can INCREASE to INCREASE yaw
    // Motor 2 and 4 can DECREASE to INCREASE yaw
    motor1.period_us += * yaw_ctrl;
    motor2.period_us -= * yaw_ctrl;
    motor3.period_us += * yaw_ctrl;
    motor4.period_us -= * yaw_ctrl;

    _motor_constrain_us(&motor1);
    _motor_constrain_us(&motor2);
    _motor_constrain_us(&motor3);
    _motor_constrain_us(&motor4);

    _motor_set_on_period(&motor1);
    _motor_set_on_period(&motor2);
    _motor_set_on_period(&motor3);
    _motor_set_on_period(&motor4);
}

/**
 * @brief Set a new thrust percentage
 * 
 * @param new_thrust_setting percent
 */
void motors_set_thrust_setting(uint8_t new_thrust_setting)
{
    if(new_thrust_setting > 100)
        return;
    
    m_thrust_setting = new_thrust_setting;
}

/**
 * @brief Get the current thrust percentage
 * 
 * @return uint8_t thrust percent
 */
uint8_t motors_get_thrust_setting()
{
    return m_thrust_setting;
}

