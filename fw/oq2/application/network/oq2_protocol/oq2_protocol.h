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

/*********************************************************************************************/
/* Special bytes and offsets ----------------------------------------------------------------*/
#define OQ2P_START          0xCA    // Packet must have this as first byte
#define OQ2P_END            0xFE    // Packet must have this as last byte
#define OQ2P_HEADER_LEN     6       // Length of header
#define OQ2P_OVERHEAD_LEN   7       // Length of all non-data bytes
#define OQ2P_LEN_MSB_INDEX  4
#define OQ2P_LEN_LSB_INDEX  5

/*********************************************************************************************/
/* Message classes --------------------------------------------------------------------------*/
typedef enum
{
    OQ2P_CLASS_DEBUG,
    OQ2P_CLASS_DATA,
    OQ2P_CLASS_CONTROL,
} oq2p_class_t;

/*********************************************************************************************/
/* Message type: COMMAND, REQUEST, RESPONSE -------------------------------------------------*/
typedef enum
{
    OQ2P_COMMAND = 0,
    OQ2P_REQUEST = 1,
    OQ2P_RESPONSE = 2,
} oq2p_command_request_t;

/*********************************************************************************************/
/* CONTROL CLASS ----------------------------------------------------------------------------*/
typedef enum
{
    OQ2P_MID_ARM = 0,
    OQ2P_MID_PITCH_SETPOINT = 1,
    OQ2P_MID_ROLL_SETPOINT = 2,
    OQ2P_MID_YAW_SETPOINT = 3, 
    OQ2P_MID_ELEVATION_SETPOINT = 4,
    OQ2P_MID_SPEED_SETPOINT = 5,
    OQ2P_MID_THRUST = 6,
    OQ2P_MID_KEEP_ALIVE = 170, 
} oq2p_control_mid_t;

/*********************************************************************************************/
/* Data lengths for various messages --------------------------------------------------------*/
#define OQ2P_DATA_LEN_ARM_RESP 4
#define OQ2P_DATA_LEN_ANGLE_SP_RESP 2
#define OQ2P_DATA_LEN_THRUST_RESP 1

/*********************************************************************************************/
/* Callback hooks ---------------------------------------------------------------------------*/
void oq2p_connect_callback(int8_t sock);
void oq2p_disconnect_callback(int8_t sock);

/*********************************************************************************************/
/* Public protocol functions  ---------------------------------------------------------------*/
uint32_t op2p_get_sent_bytes();
uint32_t op2p_get_received_bytes();

// Application needs to implement this.
void application_oq2p_control_message_handler(oq2p_control_mid_t msg_id, oq2p_command_request_t cr, uint8_t* pdata, uint16_t data_length);

void oq2p_init();




#endif