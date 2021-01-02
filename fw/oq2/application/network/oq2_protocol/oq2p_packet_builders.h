/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\network\oq2_protocol\oq2p_messages.h/
 * Project: OQ2                                                                                    /
 * Created Date: Friday, January 1st 2021, 8:40:53 am                                              /
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

#ifndef OQ2P_PACKET_BUILDERS_H_
#define OQ2P_PACKET_BUILDERS_H_

#include "oq2_protocol.h"
#include "motors.h"
#include <stdint.h>
#include <string.h>

/*********************************************************************************************/
/* Macros -----------------------------------------------------------------------------------*/
// Cause a return -1 value if the available buffer isn't large enough
#define OQ2P_BUFFER_LENGTH_VERIFY(avail, required) \
    if(avail < (OQ2P_OVERHEAD_LEN + required)) \
        return -1;

// Write the given length into the correct location in a packet. Pkt should be the start.
#define OQ2P_ENCODE_DATA_LEN(pkt, length) \
    pkt[OQ2P_LEN_MSB_INDEX] = (uint8_t)(((uint16_t)length & 0xFF00) >> 8); \
    pkt[OQ2P_LEN_LSB_INDEX] = (uint8_t)(((uint16_t)length & 0x00FF) >> 0);

// Decode a big endian uint16 at pbuf
#define OQ2P_UINT16_BIG_DECODE(pbuf) \
    (((uint16_t)pbuf[0]) << 8) | \
    (((uint16_t)pbuf[1]) << 0);

// Decode a big endian uint32 at pbuf
#define OQ2P_UINT32_BIG_DECODE(pbuf) \
    (((uint32_t)pbuf[0]) << 24) | \
    (((uint32_t)pbuf[1]) << 16) | \
    (((uint32_t)pbuf[2]) << 8) | \
    (((uint32_t)pbuf[3]) << 0);

// Encode a 16bit unsigned int into pbuf as big endian 
#define OQ2P_UINT16_BIG_ENCODE(pbuf, value) \
    pbuf[0] = (uint8_t)(((uint16_t)value & 0xFF00) >> 8); \
    pbuf[1] = (uint8_t)(((uint16_t)value & 0x00FF) >> 0);

// Encode a 32bit unsigned int into pbuf as big endian 
#define OQ2P_UINT32_BIG_ENCODE(pbuf, value) \
    pbuf[0] = (uint8_t)(((uint32_t)value & 0xFF000000) >> 24); \
    pbuf[1] = (uint8_t)(((uint32_t)value & 0x00FF0000) >> 16); \
    pbuf[2] = (uint8_t)(((uint32_t)value & 0x0000FF00) >> 8); \
    pbuf[3] = (uint8_t)(((uint32_t)value & 0x000000FF) >> 0);

/*********************************************************************************************/
/* Message builder functions ----------------------------------------------------------------*/
int32_t oq2p_msg_build_arm_response(uint8_t * pbuf, uint16_t buf_len, motor_arm_t * armed_info_buf);
int32_t oq2p_msg_build_angle_setpoint_response(uint8_t* pbuf, uint16_t buf_len, uint8_t mid_index, int32_t angle);
int32_t oq2p_msg_build_thrust_response(uint8_t* pbuf, uint16_t buf_len, uint8_t thrust);

#endif