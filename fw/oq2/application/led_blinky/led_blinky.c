/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\led_blinky\led_blinky.c             /
 * Project: OQ2                                                                                    /
 * Created Date: Tuesday, December 15th 2020, 7:57:05 pm                                           /
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

#include "led_blinky.h"
#include "main.h"
#include "app_config.h"

/**
 * @brief  Function implementing the led blinky task thread.
 * @param  argument: Not used
 * @retval None
 */
void led_thread_start(void* argument)
{
    static int m_state = 0;

    for (;;)
    {
        // HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        // HAL_Delay(10);
        // HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        // HAL_Delay(90);

        switch(m_state)
        {
            case 0:
            {
                m_state = 1;
                HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
                HAL_Delay(LED_THREAD_PERIOD);

            } break;

            case 1:
            {
                m_state = 2;
                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
                HAL_Delay(LED_THREAD_PERIOD);
            } break;

            case 2:
            {
                m_state = 0;
                HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
                osDelay(LED_THREAD_PERIOD);
            } break;
        }

        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        osDelay(LED_THREAD_PERIOD);
    }
}