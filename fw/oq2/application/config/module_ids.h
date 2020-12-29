/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\DebugLog\module_ids.h                   /
 * Project: OQ2                                                                                    /
 * Created Date: Saturday, December 12th 2020, 7:36:14 am                                          /
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


#ifndef MODULE_IDS_H_
#define MODULE_IDS_H_

#define DEBUG_CYAN_HIGHLIGHT_SELECT    NETWORK_MODULE_ID
#define DEBUG_YELLOW_HIGHLIGHT_SELECT  MAIN_MODULE_ID

typedef enum
{
    TASK_MANAGER_MODULE_ID,
    MAIN_MODULE_ID,
    FREERTOS_MODULE_ID,
    STABILITY_MODULE_ID,
    KINEMATICS_MODULE_ID,
    LOCATION_MODULE_ID,
    PID_MODULE_ID,
    MOTORS_MODULE_ID,
    NETWORK_MODULE_ID,
    WINC3400_MODULE_ID,
    WINC3400_BSP_MODULE_ID,
    WINC3400_BUS_MODULE_ID,
    WINC3400_COMMON_MODULE_ID,
    WINC3400_CRYPTO_MODULE_ID,
    WINC3400_FLASH_MODULE_ID,
    WINC3400_HIF_MODULE_ID,
    WINC3400_OTA_MODULE_ID,
    WINC3400_PERIPH_MODULE_ID,
    WINC3400_SSL_MODULE_ID,
    WINC3400_WIFI_MODULE_ID,
    WINC3400_NMASIC_MODULE_ID,
    WINC3400_NMBUS_MODULE_ID,
    WINC3400_NMDRV_MODULE_ID,
    WINC3400_NMFLASH_MODULE_ID,
    WINC3400_NMI2C_MODULE_ID,
    WINC3400_NMSPI_MODULE_ID,
    WINC3400_NMUART_MODULE_ID,
    WINC3400_SOCKET_MODULE_ID,
    WINC3400_SPIFLASH_MODULE_ID,
    NUM_MODULES
} module_id_t;



#endif