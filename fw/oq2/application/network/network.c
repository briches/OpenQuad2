/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\network\wifi\network.c              /
 * Project: OQ2                                                                                    /
 * Created Date: Monday, December 28th 2020, 7:14:35 am                                            /
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

#include "network.h"
#include "wifi_config.h"
#include "m2m_wifi.h"
#include "nmasic.h"
#include <string.h> // to get memset

#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(NETWORK_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(NETWORK_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(NETWORK_MODULE_ID, fmt, ##__VA_ARGS__)

/** Mac address information. */
static uint8_t mac_addr[M2M_MAC_ADDRES_LEN];

/** User defined MAC Address. */
const char main_user_defined_mac_address[] = { 0xf8, 0xf0, 0x05, 0x20, 0x0b, 0x09 };


/**
 * @brief Callback to get the Wi-Fi status update.
 *
 * @param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_REQ_DHCP_CONF](@ref M2M_WIFI_REQ_DHCP_CONF)
 * @param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type.
 */
static void wifi_cb(uint8_t u8MsgType, void* pvMsg)
{
    switch (u8MsgType) {
    case M2M_WIFI_RESP_CON_STATE_CHANGED:
    {
        tstrM2mWifiStateChanged* pstrWifiState = (tstrM2mWifiStateChanged*)pvMsg;
        if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
            debug_printf("Connected, DHCP start");
        }
        else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
            debug_printf("Wi-Fi disconnected\r\n");

            /* Connect to defined AP. */
            m2m_wifi_connect((char*)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (void*)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
        }

        break;
    }

    case M2M_WIFI_REQ_DHCP_CONF:
    {
        uint8_t* pu8IPAddress = (uint8_t*)pvMsg;
        debug_printf("Wi-Fi connected\r\n");
        debug_printf("Wi-Fi IP is %u.%u.%u.%u\r\n",
            pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
        break;
    }

    default:
    {
        break;
    }
    }
}

/**
 * @brief Initialization prior to freertos being enabled
 *
 */
void network_thread_pre_init()
{
    debug_printf("Network thread pre-init");

    tstrWifiInitParam param;
    int8_t ret;
    uint8_t u8IsMacAddrValid;


    /* Initialize Wi-Fi parameters structure. */
    memset((uint8_t*)&param, 0, sizeof(tstrWifiInitParam));

    /* Initialize the BSP. */
    nm_bsp_init();

    /* Initialize Wi-Fi driver with data and status callbacks. */
    param.pfAppWifiCb = wifi_cb;
    ret = m2m_wifi_init(&param);
    if (M2M_SUCCESS != ret) {
        debug_printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
        while (1) {
        }
    }

    // /* Display WINC3400 chip information. */
    // debug_printf("Chip ID : 0x%x", (unsigned int)nmi_get_chipid());
    // debug_printf("RF Revision ID : %x", (unsigned int)nmi_get_rfrevid());
    // debug_printf("Done.");

    // /* Get MAC Address from OTP. */
    // m2m_wifi_get_otp_mac_address(mac_addr, &u8IsMacAddrValid);
    // if (!u8IsMacAddrValid) {
    //     debug_printf("USER MAC Address : ");

    //     /* Cannot found MAC Address from OTP. Set user defined MAC address. */
    //     m2m_wifi_set_mac_address((uint8_t*)main_user_defined_mac_address);
    // }
    // else {
    //     debug_printf("OTP MAC Address : ");
    // }

    // /* Get MAC Address. */
    // m2m_wifi_get_mac_address(mac_addr);

    // debug_printf("%02X:%02X:%02X:%02X:%02X:%02X\r\n",
    //     mac_addr[0], mac_addr[1], mac_addr[2],
    //     mac_addr[3], mac_addr[4], mac_addr[5]);

    debug_printf("Connecting to %s.\r\n", (char*)MAIN_WLAN_SSID);
    m2m_wifi_connect((char*)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (void*)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
    
}


/**
 * @brief  Function implementing the stability task thread.
 * @param  argument: Not used
 * @retval None
 */
void network_thread(void* argument)
{
    debug_printf("Network thread");

    for (;;)
    {

        taskENTER_CRITICAL();
        m2m_wifi_handle_events(NULL);
        taskEXIT_CRITICAL();

        osDelay(NETWORK_THREAD_PERIOD);
    }
}
