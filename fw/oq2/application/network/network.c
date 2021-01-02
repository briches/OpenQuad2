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
#include "lwip/ip.h"
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/dns.h"
#include "net_init.h"
#include "m2m_wifi_ex.h"
#include "FreeRTOS.h"
#include "timer.h"
#include "oq2_protocol.h"


#if defined(CONFIG_USE_ASYNC_API)
#include "lwip/sockets.h"
#include "lwip/sys.h"
#else
#include "socket.h"
static SOCKET tcp_client_socket = -1;
#endif


#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(NETWORK_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(NETWORK_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(NETWORK_MODULE_ID, fmt, ##__VA_ARGS__)

/** Mac address information. */
static uint8_t mac_addr[M2M_MAC_ADDRES_LEN];

/** User defined MAC Address. */
const char main_user_defined_mac_address[] = { 0xf8, 0xf0, 0x05, 0x20, 0x0b, 0x11 };

/** Wi-Fi status variable. */
static volatile bool gbConnectedWifi = false;

/** IP address of host. */
uint32_t gu32HostIp = 0;

/** Get host IP status variable. */
static volatile bool gbHostIpByName = false;

/** Server host name. */
static char server_host_name[] = "google.com";

#if LWIP_STATS
/** Used to compute LwIP bandwidth */
extern uint32_t lwip_tx_count;
extern uint32_t lwip_rx_count;
extern uint32_t lwip_tx_rate;
extern uint32_t lwip_rx_rate;
#endif


/**
 * @brief
 *
 * NETCONN_EVT_RCVPLUS,
  NETCONN_EVT_RCVMINUS,
  NETCONN_EVT_SENDPLUS,
  NETCONN_EVT_SENDMINUS,
  NETCONN_EVT_ERROR
 *
 * @param pnetconn
 * @param evt
 * @param len
 */
void net_callback(struct netconn* pnetconn, enum netconn_evt evt, u16_t len)
{
    debug_printf("Callback evt %u", evt);
}

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


#if !defined(CONFIG_USE_ASYNC_API)
static void socket_cb(SOCKET sock, uint8_t u8Msg, void* pvMsg)
{
    switch (u8Msg) {
        /* Socket connected */
    case SOCKET_MSG_CONNECT:
    {
        tstrSocketConnectMsg* pstrConnect = (tstrSocketConnectMsg*)pvMsg;

        if (pstrConnect && pstrConnect->s8Error >= 0) {
            debug_printf("socket_cb: connect success!");
            oq2p_connect_callback(sock);
        }
        else {
            debug_printf("socket_cb: connect error!");
            close(tcp_client_socket);
            tcp_client_socket = -1;
            oq2p_disconnect_callback(sock);
        }
    }
    break;

    /* Message send */
    case SOCKET_MSG_SEND:
    {
        // debug_printf("socket_cb: send success!");
    }
    break;

    /* Message receive */
    case SOCKET_MSG_RECV:
    {
        tstrSocketRecvMsg* pstrRecv = (tstrSocketRecvMsg*)pvMsg;

        oq2p_receive_callback(sock, pstrRecv);
    }

    break;

    default:
        break;
    }
}
#endif

/**
 * \brief Callback function of IP address.
 *
 * \param[in] hostName Domain name.
 * \param[in] hostIp Server IP.
 *
 * \return None.
 */
static void resolve_cb(const char* hostName, ip_addr_t* ipaddr, void* callback_arg)
{
    gu32HostIp = ipaddr->addr;
    gbHostIpByName = true;
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
static void wifi_cb(uint8_t msg_type, void* msg)
{
    switch (msg_type) {
    case M2M_WIFI_RESP_CON_STATE_CHANGED:
    {
        tstrM2mWifiStateChanged* pstrWifiState = (tstrM2mWifiStateChanged*)msg;
        if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
            debug_printf("Connected");
            net_interface_up(NET_IF_STA);
            m2m_wifi_request_dhcp_client_ex();
        }
        else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
            debug_printf("Wi-Fi disconnected");
            debug_printf("Reconnecting...");
            gbConnectedWifi = M2M_WIFI_DISCONNECTED;

            /* Connect to defined AP. */
            net_interface_down(NET_IF_STA);
            os_m2m_wifi_connect((char*)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (void*)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
        }

        break;
    }

    case M2M_WIFI_REQ_DHCP_CONF:
    {
        tstrM2MIPConfig2* strIpConfig = (tstrM2MIPConfig2*)msg;
        static u8_t resolve_addr[4];
        uint16_t a[8];// = trIpConfig->u8StaticIPv6;

        memcpy(&resolve_addr, strIpConfig, 4);

        ip4_addr_t addr;
        // blob together into uint32_t
        addr.addr = lwip_htonl(*(uint32_t*)&resolve_addr[0]);
        //flip byte order
        addr.addr = lwip_htonl(addr.addr);
        net_interface_dhcp_done(NET_IF_STA, &addr);

        debug_printf("%s", ip4addr_ntoa(&addr));

        debug_printf("wifi_cb: STA M2M_WIFI_REQ_DHCP_CONF");
        debug_printf("wifi_cb: STA IPv4 addr: %d.%d.%d.%d", resolve_addr[0], resolve_addr[1],
            resolve_addr[2], resolve_addr[3]);
        /*osprintf("\n\rwifi_cb: STA IPv6 addr: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
            htons(a[0]), htons(a[1]), htons(a[2]), htons(a[3]),
            htons(a[4]), htons(a[5]), htons(a[6]), htons(a[7]));*/
        gbConnectedWifi = true;

        /* Obtain the IP Address by network name. */
        // dns_gethostbyname(server_host_name, &resolve_addr, resolve_cb, 0);

    } break;

    case M2M_WIFI_RESP_CURRENT_RSSI:
    {
        int8_t rssi = * ((int8_t *)msg);

        // debug_printf("RSSI %d dBm", rssi, (uint8_t)rssi);
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
}


#if defined(CONFIG_USE_ASYNC_API)
void _socket_func()
{
    ip_addr_t local_ip;
    ip_addr_t remote_ip;
    struct netconn *tcp_socket;
    u16_t port;

    debug_printf("Socket async lwip API");

    bool connected = false;
    // const char ip[] = "192.168.1.65";

    // static struct sockaddr_in addr;
    // memset(&addr, 0, sizeof(addr));
    // addr.sin_len = sizeof(addr);
    // addr.sin_family = AF_INET;
    // addr.sin_port = PP_HTONS(1337);
    // addr.sin_addr.s_addr = inet_addr(ip);

    for (;;)
    {
        if (gbConnectedWifi == M2M_WIFI_CONNECTED && !connected)
        {
            connected = true;

            // debug_printf("Create server socket");
            // /* Create server socket. */
            // if ((tcp_socket = netconn_new(NETCONN_TCP)) == NULL) {
            //     debug_printf("iperf_tcp_task: could not create TCP socket!\n");
            //     while (1);
            // }

            // debug_printf("Bind server socket");
            // if (netconn_bind(tcp_socket, NULL, 1336) != ERR_OK) {
            //     debug_printf("iperf_tcp_task: could not bind TCP socket!\n");
            //     while (1);
            // }

            // debug_printf("Listen server socket");
            // if (netconn_listen(tcp_socket) != ERR_OK) {
            //     debug_printf("iperf_tcp_task: could not enter listen state for TCP socket!\n");
            //     while (1);
            // }

            // while (netconn_accept(tcp_socket, &conn) != ERR_OK) {
            //     vTaskDelay(10);
            // }

            /* Client socket */
            struct netconn* conn = netconn_new(NETCONN_TCP);

            ip_addr_t * plocalip = net_interface_get_ipaddr(NET_IF_STA);

            local_ip.addr = plocalip->addr;

            debug_printf("Create client socket");
            if (ERR_OK != netconn_bind(conn, &local_ip, 0)) 
            {
                debug_printf("iperf_tcp_send: bind failed\n");
                netconn_delete(conn);
            }

            remote_ip.addr = lwip_htonl(MAIN_WIFI_M2M_SERVER_IP);

            debug_printf("local %s", ip4addr_ntoa(&local_ip));
            debug_printf("remote %s", ip4addr_ntoa(&remote_ip));

            debug_printf("Connect client socket");

            // netconn_set_nonblocking(conn, O_NONBLOCK);

            if (ERR_OK != netconn_connect(conn, &remote_ip, MAIN_WIFI_M2M_SERVER_PORT)) 
            {
                debug_printf("iperf_tcp_send: connect failed\n");
                netconn_delete(conn);
            }

            debug_printf("Done");


            // /* create the socket */

            // int8_t socket = lwip_socket(AF_INET, SOCK_STREAM, 0);

            // if (socket < 0)
            // {
            //     debug_error("Socket not created: %d", socket);
            // }

            // // uint32_t opt = lwip_fcntl(socket, F_GETFL, 0);
            // // LWIP_ASSERT("ret != -1", ret != -1);

            // // opt |= O_NONBLOCK;
            // // ret = lwip_fcntl(socket, F_SETFL, opt);
            // // LWIP_ASSERT("ret != -1", ret != -1);

            // uint32_t opt = 1;
            // uint32_t ret = lwip_ioctl(socket, FIONBIO, &opt);
            // LWIP_ASSERT("ret == 0", ret == 0);

            // ret = lwip_connect(socket, (struct sockaddr*)&addr, sizeof(addr));
            // LWIP_ASSERT("ret == -1", ret == -1);

            // uint32_t err = errno;
            // LWIP_ASSERT("errno == EINPROGRESS", err == EINPROGRESS);

        }

        osDelay(NETWORK_THREAD_PERIOD);
    }
}
#else
void _socket_func()
{
    int32_t ret;

    debug_printf("Socket blocking API");

    socketInit();
    registerSocketCallback(socket_cb, NULL);

    uint32_t five_second_average[5] = {0};
    uint32_t byte_count = 0;
    uint32_t last_byte_count = 0;
    uint32_t rate_index = 0;

    for (;;)
    {
        if (gbConnectedWifi && tcp_client_socket == -1)
        {
            debug_printf("Create socket.");
            tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0);

            struct sockaddr_in addr;

            /* Initialize socket address structure. */
            addr.sin_family = AF_INET;
            addr.sin_port = _htons(MAIN_WIFI_M2M_SERVER_PORT);
            addr.sin_addr.s_addr = _htonl(MAIN_WIFI_M2M_SERVER_IP);

            debug_printf("Connect");
            ret = connect(tcp_client_socket, (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
            debug_printf("done.");

            if (ret < 0)
            {
                close(tcp_client_socket);
                tcp_client_socket = -1;
            }
        }

        byte_count = op2p_get_received_bytes() + op2p_get_sent_bytes();

        five_second_average[rate_index++] = byte_count - last_byte_count;

        last_byte_count = byte_count;

        if(rate_index >= 5)
            rate_index = 0;

        float bitrate = 0;

        for(int i = 0; i < 5; i++)
        {
            bitrate +=  8 * five_second_average[i];
        }

        bitrate /= 5;

        // debug_printf("BITRATE %3.2f kbps",  bitrate/1000); 

        os_m2m_wifi_req_curr_rssi();

        osDelay(NETWORK_THREAD_PERIOD);
    }
}

#endif


/**
 * @brief  Function implementing the stability task thread.
 * @param  argument: Not used
 * @retval None
 */
void network_thread(void* argument)
{
    debug_printf("Network thread");

    tstrWifiInitParam param;
    uint8_t u8IsMacAddrValid;
    uint32_t ret;

    oq2p_init();

    /* Initialize the network stack. */
    net_init();

    /* Initialize Wi-Fi parameters structure. */
    memset((uint8_t*)&param, 0, sizeof(tstrWifiInitParam));

    /* Initialize Wi-Fi driver with data and status callbacks. */
    param.pfAppWifiCb = wifi_cb;
    ret = os_m2m_wifi_init(&param);
    if (ret != M2M_SUCCESS) {
        debug_printf("main: m2m_wifi_init call error!(%d)", ret);
        while (1) {
        }
    }

    /* Get MAC Address from OTP. */
    os_m2m_wifi_get_otp_mac_address(mac_addr, &u8IsMacAddrValid);
    if (!u8IsMacAddrValid)
    {
        debug_printf("USER MAC Address : ");

        /* Cannot found MAC Address from OTP. Set user defined MAC address. */
        os_m2m_wifi_set_mac_address((uint8_t*)main_user_defined_mac_address);
    }
    else
    {
        debug_printf("OTP MAC Address : ");
    }

    /* Get MAC Address. */
    os_m2m_wifi_get_mac_address(mac_addr);

    debug_printf("%02X:%02X:%02X:%02X:%02X:%02X",
        mac_addr[0], mac_addr[1], mac_addr[2],
        mac_addr[3], mac_addr[4], mac_addr[5]);

    /* Connect to Access Point */
    debug_printf("Connecting to %s.", (char*)MAIN_WLAN_SSID);
    os_m2m_wifi_connect((char*)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (void*)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);


    // Either blocking with winc3400 driver, or non-blocking with netconn api
    _socket_func();
}