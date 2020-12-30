/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\config\config_wifi.h                /
 * Project: OQ2                                                                                    /
 * Created Date: Monday, December 28th 2020, 7:10:28 am                                            /
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


#ifndef CONFIG_WIFI_H_
#define CONFIG_WIFI_H_

#include "app_config.h"

#define CONF_WINC_USE_SPI
#define CONF_WINC_SPI_LOW_DELAY
// #define CONF_WINC_SPI_DMA
// #define NM_LEVEL_INTERRUPT
#define NM_EDGE_INTERRUPT


#define CONF_WINC_SPI_INT_PIO_ID        0
#define CONF_WINC_SPI_INT_MASK          0
#define CONF_WINC_SPI_INT_PRIORITY      (0)
#define HOST_CONTROL_FLASH_OFFSET       0

/** Wi-Fi Settings */
#define MAIN_WLAN_SSID        "Click Here For Internet" /* < Destination SSID */
#define MAIN_WLAN_AUTH        M2M_WIFI_SEC_WPA_PSK /* < Security type */
#define MAIN_WLAN_PSK         "DrAspen409" /* < Password for Destination SSID */

#define MAIN_WIFI_M2M_BUFFER_SIZE 512
#define MAIN_WIFI_M2M_PRODUCT_NAME "heybb"

#define MAIN_WIFI_M2M_SERVER_PORT 1337
#define MAIN_WIFI_M2M_SERVER_IP   0xC0A80141


#endif