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

#define OQ2P_ENCODE_DATA_LEN(pkt, length) \
    pkt[OQ2P_LEN_MSB_INDEX] = (uint8_t)(((uint16_t)length & 0xFF00) >> 8); \
    pkt[OQ2P_LEN_LSB_INDEX] = (uint8_t)(((uint16_t)length & 0x00FF) >> 0);

int32_t oq2p_msg_build_arm_response(uint8_t * pbuf, uint16_t buf_len, motor_arm_t * armed_info_buf);

#endif