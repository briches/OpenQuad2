/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\config\app_config.h                 /
 * Project: OQ2                                                                                    /
 * Created Date: Monday, December 14th 2020, 5:57:00 pm                                            /
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
#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

#include "cmsis_os.h"
#include "version.h"
#include <stdint.h>
#include <stdbool.h>

#define APP_CONFIG_ENABLED 1
#define APP_CONFIG_DISABLED 0

#define DEBUG_CYAN_HIGHLIGHT_SELECT     STM_UART_DFU_MODULE_ID
#define DEBUG_YELLOW_HIGHLIGHT_SELECT   ESC_DFU_MODULE_ID
#define WINC_MODULE_DRIVER_VERBOSE      APP_CONFIG_DISABLED
#define M2M_WIFI_EX_VERBOSE             APP_CONFIG_DISABLED
#define M2M_HIF_VERBOSE                 APP_CONFIG_DISABLED

/*********************************************************************************************/
/* Task Configuration------------------------------------------------------------------------*/
#define STABILITY_THREAD_PRIO           osPriorityRealtime7
#define STABILITY_THREAD_PERIOD         4
#define STABILITY_THREAD_STACK_SIZE     2048 * 4

#define FLIGHT_THREAD_PRIO              osPriorityHigh
#define FLIGHT_THREAD_PERIOD            150
#define FLIGHT_THREAD_STACK_SIZE        2048 * 4

#define NETWORK_THREAD_PRIO             osPriorityNormal1
#define NETWORK_THREAD_PERIOD           1000
#define NETWORK_THREAD_STACK_SIZE       25000 * 4

#define LOCATION_THREAD_PRIO            osPriorityNormal
#define LOCATION_THREAD_PERIOD          500
#define LOCATION_THREAD_STACK_SIZE      1024 * 4

#define LED_THREAD_PRIO                 osPriorityLow1
#define LED_THREAD_PERIOD               1000
#define LED_THREAD_STACK_SIZE           512 * 4

#define TASK_MANAGER_THREAD_PRIO        osPriorityLow1
#define TASK_MANAGER_THREAD_STACK_SIZE  512 * 4
#define TASK_MANAGER_THREAD_PERIOD      2500

/*********************************************************************************************/
/* Task Manager -----------------------------------------------------------------------------*/
#define APP_CONFIG_TASK_MANAGER_PRINT_OUTPUT    APP_CONFIG_DISABLED

/*********************************************************************************************/
/* Stability Thread -------------------------------------------------------------------------*/
// #define SENSOR_THREAD_IMU_USE_FIFO APP_CONFIG_DISABLED
#define SENSOR_THREAD_IMU_USE_INDIVIDUAL        APP_CONFIG_ENABLED

#endif
