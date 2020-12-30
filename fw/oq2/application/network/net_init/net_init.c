/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\network\net_init\net_init.c         /
 * Project: OQ2                                                                                    /
 * Created Date: Wednesday, December 30th 2020, 7:06:10 am                                         /
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


#include <stdio.h>
#include <string.h>
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/autoip.h"
#include "netif/etharp.h"
#include <string.h>
#include "net_init.h"
#include "m2m_wifi.h"
#include "m2m_types.h"
#include "FreeRTOS.h"
#include "semphr.h"

#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(NET_INIT_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(MAIN_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(MAIN_MODULE_ID, fmt, ##__VA_ARGS__)

extern void app_network_up(uint8_t* ipv4_addr, uint8_t* ipv4_mask, uint8_t* ipv6_addr);
extern void app_network_down(void);
extern void socket_in_tcpip_task(void);
extern err_t winc_netif_init(struct netif* netif);
extern struct netif winc_netif_sta;
extern struct netif winc_netif_c_mode;

static TaskHandle_t net_tcpip_task;
static int net_started;
static SemaphoreHandle_t net_start_sem = NULL;
#define NET_S_NET_UP			0x1
#define NET_S_DHCP_RUNNING		0x2
#define NET_S_AUTOIP_RUNNING	0x4
#define NET_S_DHCP_SVR_RUNNING	0x8
static uint32_t net_state_sta;
static uint32_t net_mode_sta;
static uint32_t net_state_c;
static uint32_t net_mode_c;

int random_number(void)
{
    return rand();
}

static void status_callback(struct netif* netif)
{
    if (netif_is_up(netif)) {
        uint8_t* ip_addr = (uint8_t*)&netif->ip_addr;
        printf("Wi-Fi IP is %u.%u.%u.%u\r\n",
            ip_addr[0], ip_addr[1],
            ip_addr[2], ip_addr[3]);

            

    }
}

void net_add_winc_netif(void)
{
    ip_addr_t ip_addr;
    ip_addr.addr = 0;

    /* Add winc1000 STA interface. */
    netif_add(&winc_netif_sta, &ip_addr, &ip_addr, &ip_addr, 0, winc_netif_init, tcpip_input);
    netif_set_default(&winc_netif_sta);
#if LWIP_IPV6
    netif_create_ip6_linklocal_address(&winc_netif_sta, 1); /* Needed to sent data on the internet, not matching local netmask. */
#endif
#if LWIP_NETIF_STATUS_CALLBACK
    netif_set_status_callback(&winc_netif_sta, status_callback);
#endif

    /* Add winc1000 AP/WD interface. */
    // ip_addr_t mask = { .addr = SN_MASK_IP };
    // ip_addr_t gateway = { .addr = GW_ADDR_IP };
    // ip_addr.addr = GW_ADDR_IP;
    // netif_add(&winc_netif_c_mode, &ip_addr, &mask, &gateway, 0, winc_netif_init, tcpip_input);
    // net_set_mode(NET_IF_C, NET_MODE_AP | NET_MODE_USE_DHCP_SVR);
}

void net_remove_winc_netif(void)
{
    netif_remove(&winc_netif_sta);
    netif_remove(&winc_netif_c_mode);
}

int net_in_tcpip_task(void)
{
    return (net_tcpip_task == xTaskGetCurrentTaskHandle());
}

static void tcpip_init_done(void* arg)
{
    net_tcpip_task = xTaskGetCurrentTaskHandle();
    xSemaphoreGive(net_start_sem);
}

static void net_interface_up_imp(uint32_t net_if)
{
    if (net_if == NET_IF_STA)
    {
        /* Bring up interface in lwIP. */
        debug_printf("Link Up");
        netif_set_link_up(&winc_netif_sta);
        netif_set_up(&winc_netif_sta);
        
        net_state_sta |= NET_S_NET_UP;

        /* Interface 1 (STA). */
        if (net_mode_sta & NET_MODE_USE_DHCP)
        {
#if LWIP_DHCP
            dhcp_start(&winc_netif_sta);
            net_state_sta |= NET_S_DHCP_RUNNING;
#endif
        }
        else if (net_mode_sta & NET_MODE_USE_LINK_LOCAL)
        {
            #if LWIP_AUTOIP
            autoip_start(&winc_netif_sta);
            net_state_sta |= NET_S_AUTOIP_RUNNING;
            #endif
        }
    }
    else {
        /* Bring up interface in lwIP. */
        netif_set_link_up(&winc_netif_c_mode);
        net_state_c |= NET_S_NET_UP;

        /* Interface 2 (Concurrent mode). */
        if (net_mode_c & NET_MODE_USE_DHCP_SVR)
        {
            netif_set_up(&winc_netif_c_mode);
#if LWIP_UDP && LWIP_DHCP
            dhcp_start(&winc_netif_c_mode);
#endif
            net_state_c |= NET_S_DHCP_SVR_RUNNING;
        }
    }
}

static void net_interface_down_imp(uint32_t net_if)
{
    if (net_if == NET_IF_STA)
    {
        debug_printf("Link Down");
        netif_set_link_down(&winc_netif_sta);
        netif_set_down(&winc_netif_sta);
        net_state_sta &= ~NET_S_NET_UP;
        net_mode_sta &= ~(NET_MODE_USE_DHCP | NET_MODE_AP);
        if (net_state_sta & NET_S_DHCP_RUNNING)
        {
            #if LWIP_DHCP
            dhcp_stop(&winc_netif_sta);
            net_state_sta &= ~NET_S_DHCP_RUNNING;
            #endif
        }
        if (net_state_sta & NET_S_AUTOIP_RUNNING)
        {
#if LWIP_AUTOIP
            autoip_stop(&winc_netif_sta);
            net_state_sta &= ~NET_S_AUTOIP_RUNNING;
#endif
        }
        netif_set_down(&winc_netif_sta);
    }
    else
    {
        netif_set_link_down(&winc_netif_c_mode);
        net_state_c &= ~NET_S_NET_UP;
        net_mode_c &= ~NET_MODE_AP;
        if (net_state_c & NET_S_DHCP_SVR_RUNNING)
        {
#if LWIP_UDP && LWIP_DHCP
            dhcp_stop(&winc_netif_c_mode);
#endif
            net_state_c &= ~NET_S_DHCP_SVR_RUNNING;
        }
        netif_set_down(&winc_netif_c_mode);
    }
}

void net_interface_dhcp_done(uint32_t net_if, ip4_addr_t * paddr)
{
    if(net_if == NET_IF_STA)
    {
        netif_set_ipaddr(&winc_netif_sta, paddr);
    }
}

void net_interface_up(uint32_t net_if)
{
    net_interface_up_imp(net_if);
}

void net_interface_down(uint32_t net_if)
{
    net_interface_down_imp(net_if);
}

void net_set_mode(uint32_t net_if, uint32_t mode)
{
    if (net_if == NET_IF_STA)
    {
        net_mode_sta = mode;

        if ((mode & NET_MODE_USE_DHCP))
        {
            if ((net_state_sta & (NET_S_DHCP_RUNNING | NET_S_NET_UP)) == NET_S_NET_UP)
            {
#if LWIP_DHCP
                dhcp_start(&winc_netif_sta);
                net_state_sta |= NET_S_DHCP_RUNNING;
#endif
            }
        }
        else
        {
            if (net_state_sta & NET_S_DHCP_RUNNING)
            {
#if LWIP_DHCP
                dhcp_stop(&winc_netif_sta);
                net_state_sta &= ~NET_S_DHCP_RUNNING;
#endif
            }
        }
    }
    else
    {
        net_mode_c = mode;
    }
}

void net_init(void)
{
    if (!net_started)
    {
        net_started = 1;
        net_start_sem = xSemaphoreCreateBinary();
        xSemaphoreGive(net_start_sem);

        xSemaphoreTake(net_start_sem, portMAX_DELAY);
        tcpip_init(tcpip_init_done, 0);
        xSemaphoreTake(net_start_sem, portMAX_DELAY);
        vSemaphoreDelete(net_start_sem);
    }
}
