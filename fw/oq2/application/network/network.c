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
#include <string.h> // to get memset
#include "network.h"
#include "wifi_config.h"
#include "m2m_wifi.h"
#include "socket.h"
#include "nmasic.h"

#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(NETWORK_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(NETWORK_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(NETWORK_MODULE_ID, fmt, ##__VA_ARGS__)

/** Mac address information. */
static uint8_t mac_addr[M2M_MAC_ADDRES_LEN];

/** User defined MAC Address. */
const char main_user_defined_mac_address[] = { 0xf8, 0xf0, 0x05, 0x20, 0x0b, 0x09 };

/** Message format definitions. */
typedef struct s_msg_wifi_product {
    uint8_t name[9];
} t_msg_wifi_product;

/** Message format declarations. */
static t_msg_wifi_product msg_wifi_product = {
    .name = MAIN_WIFI_M2M_PRODUCT_NAME,
};

/** Receive buffer definition. */
static uint8_t gau8SocketTestBuffer[MAIN_WIFI_M2M_BUFFER_SIZE];

/** Socket for client */
static SOCKET tcp_client_socket = -1;

/** Wi-Fi connection state */
static uint8_t wifi_connected;


static struct sockaddr_in socket_addr;


/**
 * \brief Callback to get the Data from socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg socket event type. Possible values are:
 *  - SOCKET_MSG_BIND
 *  - SOCKET_MSG_LISTEN
 *  - SOCKET_MSG_ACCEPT
 *  - SOCKET_MSG_CONNECT
 *  - SOCKET_MSG_RECV
 *  - SOCKET_MSG_SEND
 *  - SOCKET_MSG_SENDTO
 *  - SOCKET_MSG_RECVFROM
 * \param[in] pvMsg is a pointer to message structure. Existing types are:
 *  - tstrSocketBindMsg
 *  - tstrSocketListenMsg
 *  - tstrSocketAcceptMsg
 *  - tstrSocketConnectMsg
 *  - tstrSocketRecvMsg
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void* pvMsg)
{
    switch (u8Msg) {
        /* Socket connected */
    case SOCKET_MSG_CONNECT:
    {
        tstrSocketConnectMsg* pstrConnect = (tstrSocketConnectMsg*)pvMsg;
        if (pstrConnect && pstrConnect->s8Error >= 0) {
            debug_printf("socket_cb: connect success!\r\n");
            send(tcp_client_socket, &msg_wifi_product, sizeof(t_msg_wifi_product), 0);
        }
        else {
            debug_printf("socket_cb: connect error!\r\n");
            close(tcp_client_socket);
            tcp_client_socket = -1;
        }
    }
    break;

    /* Message send */
    case SOCKET_MSG_SEND:
    {
        debug_printf("socket_cb: send success!\r\n");
        recv(tcp_client_socket, gau8SocketTestBuffer, sizeof(gau8SocketTestBuffer), 0);
    }
    break;

    /* Message receive */
    case SOCKET_MSG_RECV:
    {
        tstrSocketRecvMsg* pstrRecv = (tstrSocketRecvMsg*)pvMsg;
        if (pstrRecv && pstrRecv->s16BufferSize > 0) {
            debug_printf("socket_cb: recv success!\r\n");
            debug_printf("TCP Client Test Complete!\r\n");
        }
        else {
            debug_printf("socket_cb: recv error!\r\n");
            close(tcp_client_socket);
            tcp_client_socket = -1;
        }
    }

    break;

    default:
        break;
    }
}

/**
 * @brief Callback to get the Wi-Fi status update.
 *
 * @param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * @param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 */
static void wifi_cb(uint8_t u8MsgType, void* pvMsg)
{
    switch (u8MsgType) {
    case M2M_WIFI_RESP_CON_STATE_CHANGED:
    {
        tstrM2mWifiStateChanged* pstrWifiState = (tstrM2mWifiStateChanged*)pvMsg;
        if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
            debug_printf("Connected");
        }
        else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
            debug_printf("Wi-Fi disconnected");
            wifi_connected = M2M_WIFI_DISCONNECTED;

            /* Connect to defined AP. */
            m2m_wifi_connect((char*)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (void*)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
        }

        break;
    }

    case M2M_WIFI_REQ_DHCP_CONF:
    {
        uint8_t* pu8IPAddress = (uint8_t*)pvMsg;

        debug_printf("DHCP CONF, IP is %u.%u.%u.%u",
            pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);

        wifi_connected = M2M_WIFI_CONNECTED;
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
        debug_printf("main: m2m_wifi_init call error!(%d)", ret);
        while (1) {
        }
    }

    /* Display WINC3400 chip information. */
    debug_printf("Chip ID : 0x%x", (unsigned int)nmi_get_chipid());
    debug_printf("RF Revision ID : %x", (unsigned int)nmi_get_rfrevid());
    debug_printf("Done.");

    /* Get MAC Address from OTP. */
    m2m_wifi_get_otp_mac_address(mac_addr, &u8IsMacAddrValid);
    if (!u8IsMacAddrValid)
    {
        debug_printf("USER MAC Address : ");

        /* Cannot found MAC Address from OTP. Set user defined MAC address. */
        m2m_wifi_set_mac_address((uint8_t*)main_user_defined_mac_address);
    }
    else
    {
        debug_printf("OTP MAC Address : ");
    }

    /* Get MAC Address. */
    m2m_wifi_get_mac_address(mac_addr);

    debug_printf("%02X:%02X:%02X:%02X:%02X:%02X",
        mac_addr[0], mac_addr[1], mac_addr[2],
        mac_addr[3], mac_addr[4], mac_addr[5]);

    /* Initialize socket address structure. */
    socket_addr.sin_family = AF_INET;
    socket_addr.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT);
    socket_addr.sin_addr.s_addr = _htonl(MAIN_WIFI_M2M_SERVER_IP);

    /* Initialize socket module */
    socketInit();
    registerSocketCallback(socket_cb, NULL);

    /* Connect to Access Point */
    debug_printf("Connecting to %s.", (char*)MAIN_WLAN_SSID);
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

        if (wifi_connected == M2M_WIFI_CONNECTED) {
            /* Open client socket. */
            if (tcp_client_socket < 0) {
                if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
                    printf("main: failed to create TCP client socket error!\r\n");
                    continue;
                }

                /* Connect server */
                uint32_t ret = connect(tcp_client_socket, (struct sockaddr*)&socket_addr, sizeof(struct sockaddr_in));

                if (ret < 0) {
                    close(tcp_client_socket);
                    tcp_client_socket = -1;
                }
            }
        }

        osDelay(NETWORK_THREAD_PERIOD);
    }
}
