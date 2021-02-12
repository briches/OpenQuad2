/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\led_blinky\led_blinky.h             /
 * Project: OQ2                                                                                    /
 * Created Date: Tuesday, December 15th 2020, 7:57:10 pm                                           /
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


#ifndef _LED_BLINKY_H_
#define _LED_BLINKY_H_

typedef enum 
{
    LED_BLINK_MODE_NONE,
    LED_BLINK_MODE_RED,
    LED_BLINK_MODE_GREEN,
    LED_BLINK_MODE_BLUE,
    LED_BLINK_MODE_YELLOW,
    LED_BLINK_MODE_CYAN,
    LED_BLINK_MODE_PINK,
    LED_BLINK_MODE_WHITE,
    LED_BLINK_NUMBER_OF_MODES, 
} led_blink_mode_t;

void led_set_blink_mode(led_blink_mode_t mode);

void led_thread(void* argument);

#endif