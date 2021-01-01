/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\network\oq2_protocol\oq2_protocol.h /
 * Project: OQ2                                                                                    /
 * Created Date: Wednesday, December 30th 2020, 9:25:54 am                                         /
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


#ifndef OQ2_PROTOCOL_H_
#define OQ2_PROTOCOL_H_

#include <stdint.h>
#include "wifi_config.h"

#if defined(CONFIG_USE_ASYNC_API)
#include "lwip/sockets.h"
#include "lwip/sys.h"
#else
#include "socket.h"
void oq2p_receive_callback(int8_t sock, tstrSocketRecvMsg * p_recv);
#endif

#define OQ2P_START          0xCA    // Packet must have this as first byte
#define OQ2P_END            0xFE    // Packet must have this as last byte
#define OQ2P_HEADER_LEN     6       // Length of header
#define OQ2P_OVERHEAD_LEN   7       // Length of all non-data bytes
#define OQ2P_LEN_MSB_INDEX  4
#define OQ2P_LEN_LSB_INDEX  5

typedef enum OQ2P_CLASS
{
    OQ2P_CLASS_DEBUG,
    OQ2P_CLASS_DATA,
    OQ2P_CLASS_CONTROL,
} oq2p_class_t;

typedef enum OQ2P_REQUEST_COMMAND
{
    OQ2P_COMMAND = 0,
    OQ2P_REQUEST = 1,
    OQ2P_RESPONSE = 2,
} oq2p_command_request_t;

typedef enum OQ2P_CONTROL_MID
{
    OQ2P_MID_ARM,
} oq2p_control_mid_t;


#define OQ2P_DATA_LEN_ARM_RESPONSE 4


void oq2p_connect_callback(int8_t sock);
void oq2p_disconnect_callback(int8_t sock);

uint32_t op2p_get_sent_bytes();
uint32_t op2p_get_received_bytes();

void oq2p_init();


#endif