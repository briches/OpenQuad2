/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\flight_app\flight_app.c             /
 * Project: OQ2                                                                                    /
 * Created Date: Friday, January 8th 2021, 1:54:40 pm                                              /
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


#include "flight_app.h"
#include "flight_config.h"
#include "motors.h"
#include "esc_dfu.h"
#include "stability.h"
#include "led_blinky.h"

#include "debug_log.h"

#define debug_error(fmt, ...)           debug_error(FLIGHT_APP_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(FLIGHT_APP_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(FLIGHT_APP_MODULE_ID, fmt, ##__VA_ARGS__)

static flight_state_t m_flight_state = FLIGHT_RESET;

/**
 * @brief If angle exceeds SC_SAFETY_ROLL_LIMIT or SC_SAFETY_PITCH_LIMIT, disarm immediately
 * 
 */
void flight_app_critical_angle_callback()
{
    // return;
    m_flight_state = FLIGHT_MOTOR_DISARM;
    if (motor_controllers_get_arm_state(1) != MOTOR_DISARMED)
        motor_controllers_set_arm_state(1, MOTOR_DISARMED);

    if (motor_controllers_get_arm_state(2) != MOTOR_DISARMED)
        motor_controllers_set_arm_state(2, MOTOR_DISARMED);

    if (motor_controllers_get_arm_state(3) != MOTOR_DISARMED)
        motor_controllers_set_arm_state(3, MOTOR_DISARMED);

    if (motor_controllers_get_arm_state(4) != MOTOR_DISARMED)
        motor_controllers_set_arm_state(4, MOTOR_DISARMED);
}

/**
 * @brief Process basic stability control inputs from stability thread
 * 
 * @param pitch_ctrl pitch pid output
 * @param roll_ctrl roll pid output
 * @param yaw_ctrl yaw pid output
 */
void flight_app_new_stability_inputs(float* pitch_ctrl, float* roll_ctrl, float* yaw_ctrl)
{
    if (m_flight_state < FLIGHT_MOTOR_READY)
        return;

    // debug_printf("%4.1f", *pitch_ctrl);

    // Process the new inputs.
    // float ctrl = 25;
    float zero = 0;

    motor_controllers_control_input(pitch_ctrl, roll_ctrl, yaw_ctrl);
}

/**
 * @brief Receive arm command from controller
 * 
 */
void flight_app_on_arm_command(motor_arm_t arm)
{
    if(arm == MOTOR_ARMED)
    {
        if(m_flight_state == FLIGHT_INIT)
        {
            m_flight_state = FLIGHT_MOTOR_ARM;
        }
    }
    else
    {
        m_flight_state = FLIGHT_MOTOR_DISARM;
    }
}

/**
 * @brief  Function implementing the Flight application thread.
 * @param  argument: Not used
 * @retval None
 */
void flight_thread(void* argument)
{
    debug_printf("Flight app thread start");

    for (;;)
    {
        osDelay(FLIGHT_THREAD_PERIOD);

        // debug_printf("Flight State: %u", m_flight_state);

        switch (m_flight_state)
        {
        /*********************************************************************************************/
        /** Flight State: Motors Disarmed. Can end up here if exceeded critical safety limits        */
        case FLIGHT_MOTOR_DISARM:
            led_set_blink_mode(LED_BLINK_MODE_RED);
            
            // debug_error("Disarmed");
            if (motor_controllers_get_arm_state(1) != MOTOR_DISARMED)
                motor_controllers_set_arm_state(1, MOTOR_DISARMED);

            if (motor_controllers_get_arm_state(2) != MOTOR_DISARMED)
                motor_controllers_set_arm_state(2, MOTOR_DISARMED);

            if (motor_controllers_get_arm_state(3) != MOTOR_DISARMED)
                motor_controllers_set_arm_state(3, MOTOR_DISARMED);

            if (motor_controllers_get_arm_state(4) != MOTOR_DISARMED)
                motor_controllers_set_arm_state(4, MOTOR_DISARMED);
            break;

        /*********************************************************************************************/
        /** Flight Reset: First state upon reset. Sets the ESCs and motors to OFF (disarmed)         */
        case FLIGHT_RESET:
            motor_controllers_set_arm_state(1, MOTOR_DISARMED);
            motor_controllers_set_arm_state(2, MOTOR_DISARMED);
            motor_controllers_set_arm_state(3, MOTOR_DISARMED);
            motor_controllers_set_arm_state(4, MOTOR_DISARMED);
            // Let motor controllers power down.
            osDelay(1000);
            m_flight_state = FLIGHT_INIT;
            break;

        /*********************************************************************************************/
        /** Flight Init: After reset perform initialization                                          */
        case FLIGHT_INIT:
            // debug_printf("Flight Init (no action)");
            break;

        /*********************************************************************************************/
        /** Flight Motor Arm: Power on the escs and motors                                           */
        case FLIGHT_MOTOR_ARM:
            debug_printf("Armed state 1, 2, 3, 4");
            motor_controllers_set_arm_state(2, MOTOR_ARMED);
            motor_controllers_set_arm_state(3, MOTOR_ARMED);
            motor_controllers_set_arm_state(4, MOTOR_ARMED);
            motor_controllers_set_arm_state(1, MOTOR_ARMED);

            #ifndef ESC_DFU_ENABLED
                m_flight_state = FLIGHT_MOTOR_STARTUP;
            #else
                m_flight_state = FLIGHT_ESC_DFU_START;
            #endif
            // m_flight_state = FLIGHT_MOTOR_READY;
            break;
        
        /*********************************************************************************************/
        /** ESC DFU Start: Reboot the ESCs in bootloader mode                                        */
        case FLIGHT_ESC_DFU_START:

            debug_printf("ESC DFU Start.");

            // Set reset pin, then boot pin, wait, then release reset to enter bootloader
            for(int i = 1; i <= 4; i++)
            {
                motor_controllers_set_reset_state(i, MOTOR_CONTROLLER_RESET);
                motor_controllers_set_boot_state(i, MOTOR_CONTROLLER_BOOTLOADER);
                osDelay(5);
                motor_controllers_set_reset_state(i, MOTOR_CONTROLLER_NOT_RESET);
            }


            // ESC unit is now in bootloader. 
            osDelay(100);

            // Init DFU on the esc micro
            esc_dfu_init();

            m_flight_state = FLIGHT_ESC_DFU_ERASE;
            break;  

        /*********************************************************************************************/
        /** ESC DFU Erase: Erase firmware from the escs                                              */
        case FLIGHT_ESC_DFU_ERASE:

            if(esc_dfu_erase_chip() == 0)
                m_flight_state = FLIGHT_ESC_DFU_WRITE;
            else
                m_flight_state = FLIGHT_ESC_DFU_DONE;
            
            break;

        /*********************************************************************************************/
        /** ESC DFU Write: Write new firmware file to the escs                                       */
        case FLIGHT_ESC_DFU_WRITE:
        {

            debug_printf("Writing new app.");
            debug_printf("Flash storage location: 0x%08X (0x%08X B)", _FLASH_STORAGE_, _FLASH_STORAGE_SIZE_);
            uint8_t * p_esc_app = (uint8_t *) (_FLASH_STORAGE_);

            if(esc_dfu_write_app(p_esc_app, 22000) == 0)
                m_flight_state = FLIGHT_ESC_DFU_VERIFY;
            else
                m_flight_state = FLIGHT_ESC_DFU_DONE;

            
        } break;
        /*********************************************************************************************/
        /** ESC DFU Verify: Verify the flash on the escs against the file                            */
        case FLIGHT_ESC_DFU_VERIFY:
        {
            debug_printf("Doing flash verification");
            debug_printf("Flash storage location: 0x%08X (0x%08X B)", _FLASH_STORAGE_, _FLASH_STORAGE_SIZE_);

            uint8_t * p_esc_app = (uint8_t *) (_FLASH_STORAGE_);

            debug_printf("Verify pointer is 0x%08X", p_esc_app);

            if(esc_dfu_verify(p_esc_app, 22000) == 0)
            {
                esc_dfu_start_app();

                // Set reset pin, then boot pin, wait, then release reset to enter bootloader
                motor_controllers_set_reset_state(4, MOTOR_CONTROLLER_RESET);
                motor_controllers_set_boot_state(4, MOTOR_CONTROLLER_NORMAL_BOOT);
                osDelay(5);
                motor_controllers_set_reset_state(4, MOTOR_CONTROLLER_NOT_RESET);

                debug_printf("Verified!");

                m_flight_state = FLIGHT_ESC_DFU_DONE;
            }
            else
            {
                debug_error("DFU failed.");
                m_flight_state = FLIGHT_ESC_DFU_FAILED;
            }
            
        } break;
        /*********************************************************************************************/
        /** ESC DFU Done: The DFU should be... finished?                                             */
        case FLIGHT_ESC_DFU_DONE:

            break;

        /*********************************************************************************************/
        /** Motor Startup: Start the motors to the lowest speed possible                             */
        case FLIGHT_MOTOR_STARTUP:

            debug_printf("Motor startup");

            motor_controllers_startup(2);
            osDelay(500);

            motor_controllers_startup(3);
            osDelay(500);

            motor_controllers_startup(4);
            osDelay(500);

            motor_controllers_startup(1);
            osDelay(500);

            m_flight_state = FLIGHT_MOTOR_POST_STARTUP;
            break;

        /*********************************************************************************************/
        /** Motor Post Startup: Ramp the motors to operating speed                                   */
        case FLIGHT_MOTOR_POST_STARTUP:
        {
            static uint32_t count = 0;
            static uint8_t thrust_pct = 0;
            float zero = 0;

            debug_printf("Post startup.");

            motors_set_thrust_setting(thrust_pct);
            motor_controllers_control_input(&zero, &zero, &zero);

            count++;
            thrust_pct++;

            if (count >= 25)
                m_flight_state = FLIGHT_MOTOR_READY;
        } break;

        /*********************************************************************************************/
        /** Motors are ramped, calculate gyro bias                                                   */
        case FLIGHT_MOTOR_READY:
        {
            led_set_blink_mode(LED_BLINK_MODE_CYAN);

            debug_printf("Starting gyro bias.");

            stability_start_gyro_bias(true);

            osDelay(10000);

            stability_store_gyro_bias();

            m_flight_state = FLIGHT_READY;
        } break;

        /*********************************************************************************************/
        /** Flight Ready: Motors are at operating speed and craft is ready for further actions        */
        case FLIGHT_READY:
            led_set_blink_mode(LED_BLINK_MODE_GREEN);

            break;

        default:
            break;
        }
    }
}


flight_state_t flight_app_get_state()
{
    return m_flight_state;
}