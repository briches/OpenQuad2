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
__always_inline static void _motor_constrain_us(motor_t* motor)
{
    if (motor->period_us > MC_MAX_TON_US_PERIOD)
    {
        motor->period_us = MC_MAX_TON_US_PERIOD;
    }
    else if (motor->period_us < MC_MIN_TON_US_PERIOD)
    {
        motor->period_us = MC_MIN_TON_US_PERIOD;
    }
}


static inline void _motor_set_on_period(motor_t* motor)
{
    uint16_t on = (0xFFFF) * (float)(motor->period_us / MC_PWM_PERIOD);

    // debug_printf("Motor1 us period: %5.0f, compare reg: %u", motor1.period_us, on);

    __HAL_TIM_SET_COMPARE(motor->htimer, motor->channel, on);
}


static void _motor_set_arm(motor_t* motor, motor_arm_t arm)
{
    if (arm != MOTOR_ARMED && arm != MOTOR_DISARMED)
        return;

    HAL_GPIO_WritePin(motor->arm_pin.port, motor->arm_pin.pin, (GPIO_PinState)arm);
    HAL_GPIO_WritePin(motor->nreset_pin.port, motor->nreset_pin.pin, (GPIO_PinState)arm);

    if (arm == MOTOR_ARMED)
    {
        debug_printf("ARMING motor %u", motor->index);

        motor->period_us = MC_STARTUP_TON_US_PERIOD;
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
    motor1.nreset_pin.pin = BLDC_nRESET1_Pin;
    motor1.nreset_pin.port = BLDC_nRESET1_GPIO_Port;
    motor1.boot_pin.pin = BLDC_BOOTSEL1_Pin;
    motor1.boot_pin.port = BLDC_BOOTSEL1_GPIO_Port;
    motor1.htimer = &htim1;
    motor1.channel = TIM_CHANNEL_3;
    motor1.period_us = 0;

    motor2.index = 2;
    motor2.arm_pin.pin = BLDC_ARM2_Pin;
    motor2.arm_pin.port = BLDC_ARM2_GPIO_Port;
    motor2.nreset_pin.pin = BLDC_nRESET2_Pin;
    motor2.nreset_pin.port = BLDC_nRESET2_GPIO_Port;
    motor2.boot_pin.pin = BLDC_BOOTSEL2_Pin;
    motor2.boot_pin.port = BLDC_BOOTSEL2_GPIO_Port;
    motor2.htimer = &htim1;
    motor2.channel = TIM_CHANNEL_4;
    motor2.period_us = 0;

    motor3.index = 3;
    motor3.arm_pin.pin = BLDC_ARM3_Pin;
    motor3.arm_pin.port = BLDC_ARM3_GPIO_Port;
    motor3.nreset_pin.pin = BLDC_nRESET3_Pin;
    motor3.nreset_pin.port = BLDC_nRESET3_GPIO_Port;
    motor3.boot_pin.pin = BLDC_BOOTSEL3_Pin;
    motor3.boot_pin.port = BLDC_BOOTSEL3_GPIO_Port;
    motor3.htimer = &htim24;
    motor3.channel = TIM_CHANNEL_1;
    motor3.period_us = 0;

    motor4.index = 4;
    motor4.arm_pin.pin = BLDC_ARM4_Pin;
    motor4.arm_pin.port = BLDC_ARM4_GPIO_Port;
    motor4.nreset_pin.pin = BLDC_nRESET4_Pin;
    motor4.nreset_pin.port = BLDC_nRESET4_GPIO_Port;
    motor4.boot_pin.pin = BLDC_BOOTSEL4_Pin;
    motor4.boot_pin.port = BLDC_BOOTSEL4_GPIO_Port;
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
    motor_controllers_set_boot_state(index, MOTOR_CONTROLLER_NORMAL_BOOT);
    motor_controllers_set_reset_state(index, MOTOR_CONTROLLER_NOT_RESET);
    
    switch (index)
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
    switch (index)
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
 * @brief Set the motor controller in reset or not
 * 
 * @param index Index of motor controller (1,2,3,4)
 * @param reset Logical RESET or not. 1 = Reset, 0 = not reset
 */
void motor_controllers_set_reset_state(uint8_t index, motor_controller_reset_t reset)
{
    switch (index)
    {
    case 1:
        HAL_GPIO_WritePin(motor1.nreset_pin.port, motor1.nreset_pin.pin, (GPIO_PinState)(reset));
        break;
    case 2:
        HAL_GPIO_WritePin(motor2.nreset_pin.port, motor2.nreset_pin.pin, (GPIO_PinState)(reset));
        break;
    case 3:
        HAL_GPIO_WritePin(motor3.nreset_pin.port, motor3.nreset_pin.pin, (GPIO_PinState)(reset));
        break;
    case 4:
        HAL_GPIO_WritePin(motor4.nreset_pin.port, motor4.nreset_pin.pin, (GPIO_PinState)(reset));
        break;
    default:
        break;
    }
}


/**
 * @brief Set the motor controller in normal or bootloader mode upon reset
 * 
 * @param index Index of motor controller (1,2,3,4)
 * @param normal_boot Logical normal boot or not.
 */
void motor_controllers_set_boot_state(uint8_t index, motor_controller_boot_t normal_boot)
{
    switch (index)
    {
    case 1:
        HAL_GPIO_WritePin(motor1.boot_pin.port, motor1.boot_pin.pin, (GPIO_PinState)(normal_boot));
        break;
    case 2:
        HAL_GPIO_WritePin(motor2.boot_pin.port, motor2.boot_pin.pin, (GPIO_PinState)(normal_boot));
        break;
    case 3:
        HAL_GPIO_WritePin(motor3.boot_pin.port, motor3.boot_pin.pin, (GPIO_PinState)(normal_boot));
        break;
    case 4:
        HAL_GPIO_WritePin(motor4.boot_pin.port, motor4.boot_pin.pin, (GPIO_PinState)(normal_boot));
        break;
    default:
        break;
    }
}


/**
 * @brief Set the PWM on period to the minimum, which will start the motors at minimum speed. 
 * 
 * @param index Motor index (1, 2, 3, 4)
 */
void motor_controllers_startup(uint8_t index)
{
    switch (index)
    {
    case 1:
        motor1.period_us = MC_MIN_TON_US_PERIOD;
        _motor_set_on_period(&motor1);
        break;
    case 2:
        motor2.period_us = MC_MIN_TON_US_PERIOD;
        _motor_set_on_period(&motor2);
        break;
    case 3:
        motor3.period_us = MC_MIN_TON_US_PERIOD;
        _motor_set_on_period(&motor3);
        break;
    case 4:
        motor4.period_us = MC_MIN_TON_US_PERIOD;
        _motor_set_on_period(&motor4);
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
void motor_controllers_control_input(float* pitch_ctrl, float* roll_ctrl, float* yaw_ctrl)
{
    // Thrust Control
    motor1.period_us = MC_MIN_TON_US_PERIOD + (((float)m_thrust_setting) / 100.0f) * MC_TON_US_RANGE;
    motor2.period_us = MC_MIN_TON_US_PERIOD + (((float)m_thrust_setting) / 100.0f) * MC_TON_US_RANGE;
    motor3.period_us = MC_MIN_TON_US_PERIOD + (((float)m_thrust_setting) / 100.0f) * MC_TON_US_RANGE;
    motor4.period_us = MC_MIN_TON_US_PERIOD + (((float)m_thrust_setting) / 100.0f) * MC_TON_US_RANGE;

    // Pitch control. 
    // Motor 1 and 2 can INCREASE to INCREASE pitch
    // Motor 3 and 4 can DECREASE to INCREASE pitch
    motor1.period_us += *pitch_ctrl;    // Balance offset
    motor2.period_us += *pitch_ctrl;    // Balance offset
    motor3.period_us -= *pitch_ctrl;
    motor4.period_us -= *pitch_ctrl;

    // Roll control. 
    // Motor 2 and 3 can INCREASE to INCREASE roll
    // Motor 1 and 4 can DECREASE to INCREASE roll
    motor1.period_us -= *roll_ctrl;
    motor2.period_us += *roll_ctrl;
    motor3.period_us += *roll_ctrl;
    motor4.period_us -= *roll_ctrl;

    // Yaw control. 
    // Motor 1 and 3 can INCREASE to INCREASE yaw
    // Motor 2 and 4 can DECREASE to INCREASE yaw
    motor1.period_us += *yaw_ctrl;
    motor2.period_us -= *yaw_ctrl;
    motor3.period_us += *yaw_ctrl;
    motor4.period_us -= *yaw_ctrl;

    // debug_printf("Motor1 us period: %4.0f", motor1.period_us);

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
    if (new_thrust_setting > 100)
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

