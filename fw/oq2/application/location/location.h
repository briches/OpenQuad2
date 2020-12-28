/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\location\location.h                 /
 * Project: OQ2                                                                                    /
 * Created Date: Tuesday, December 15th 2020, 8:04:58 pm                                           /
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

#ifndef _LOCATION_H_
#define _LOCATION_H_

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

extern UART_HandleTypeDef huart5;

typedef struct 
{
    bool valid;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t millis;
} utc_time_t;

void gnss_fix_status_callback();
void gnss_new_data_callback();
void gps_tx_complete_callback();
void gps_rx_complete_callback();

void location_thread_pre_init();
void location_thread_start(void* argument);



#endif