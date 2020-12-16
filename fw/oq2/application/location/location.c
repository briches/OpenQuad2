/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\location\location.c                 /
 * Project: OQ2                                                                                    /
 * Created Date: Tuesday, December 15th 2020, 8:05:02 pm                                           /
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


#include "location.h"
#include "app_config.h"
#include "debug_log.h"
#include "main.h"
#include <string.h>

#define debug_error(fmt, ...)           debug_error(LOCATION_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(LOCATION_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(LOCATION_MODULE_ID, fmt, ##__VA_ARGS__)


static char gps_uart_rx_buf[128];
static char gps_uart_tx_buf[128];

void gps_tx_complete_callback()
{
    debug_printf("gps_tx_complete_callback");
}

void gps_rx_complete_callback()
{
    // debug_printf("gps_rx_complete_callback");

    HAL_UART_Receive_IT(&huart5, gps_uart_rx_buf, 1);
}


/**
 * @brief Initialization prior to the kernel being initialized and any tasks being created
 * 
 */
void location_thread_pre_init()
{
}


/**
 * @brief  Function implementing the location task thread.
 * @param  argument: Not used
 * @retval None
 */
void location_thread_start(void* argument)
{
    size_t hello_size;
    
    uint8_t * p = gps_uart_tx_buf;

    *(p)++ = 0xb5;
    *(p)++ = 0x62;
    
    // class
    *(p)++ = 0x0A;

    // ID
    *(p)++ = 0x00;

    // LEnth

    // Payload

    // CK_A/CK_B

    // Start receiving
    HAL_UART_Receive_IT(&huart5, gps_uart_rx_buf, 1);

    HAL_GPIO_WritePin(GPS_RESETN_GPIO_Port, GPS_RESETN_Pin, GPIO_PIN_SET);

    for(;;)
    {
        osDelay(LOCATION_THREAD_PERIOD);

        if(osKernelGetState() == osKernelRunning)
        {
            // HAL_UART_Transmit_IT(&huart5, gps_uart_tx_buf, hello_size);
            // debug_printf("Location.");
        }

        debug_print_buffer(gps_uart_rx_buf, 30, 0, 16);
        
    }
}