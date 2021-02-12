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

static led_blink_mode_t m_mode = LED_BLINK_MODE_WHITE;

__always_inline
static void red_off()
{
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
}

__always_inline
static void red_on()
{
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
}

__always_inline
static void green_off()
{
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
}

__always_inline
static void green_on()
{
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
}

__always_inline
static void blue_off()
{
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
}

__always_inline
static void blue_on()
{
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
}

void led_set_blink_mode(led_blink_mode_t mode)
{
    m_mode = mode;
}

/**
 * @brief  Function implementing the led blinky task thread.
 * @param  argument: Not used
 * @retval None
 */
void led_thread(void* argument)
{
    static int m_state = 0;

    for (;;)
    {
        led_blink_mode_t this_round = m_mode;

        switch (this_round)
        {

        case LED_BLINK_MODE_NONE:
            red_off();
            green_off();
            blue_off();
            break;
        case LED_BLINK_MODE_RED:
            red_on();
            green_off();
            blue_off();
            break;
        case LED_BLINK_MODE_GREEN:
            red_off();
            green_on();
            blue_off();
            break;
        case LED_BLINK_MODE_BLUE:
            red_off();
            green_off();
            blue_on();
            break;
        case LED_BLINK_MODE_YELLOW:
            red_on();
            green_on();
            blue_off();
            break;
        case LED_BLINK_MODE_CYAN:
            red_off();
            green_on();
            blue_on();
            break;
        case LED_BLINK_MODE_PINK:
            red_on();
            green_off();
            blue_on();
            break;
        case LED_BLINK_MODE_WHITE:
            red_on();
            green_on();
            blue_on();
            break;
        }

        osDelay(LED_THREAD_PERIOD / 5);

        red_off();
        green_off();
        blue_off();

        osDelay(LED_THREAD_PERIOD * 4/5);
    }
}