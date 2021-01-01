/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\network\oq2_protocol\oq2p_messages.c/
 * Project: OQ2                                                                                    /
 * Created Date: Friday, January 1st 2021, 8:40:46 am                                              /
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


#include "oq2p_packet_builders.h"
#include "motors.h"
/**
 * @brief Assemble a ARMED RESPONSE message into the provided buffer
 * 
 * @param pbuf Location for message to be created
 * @param buf_len Length of buffer given - if not long enough, will return -1
 * @param armed_info_buf a 4 byte long array of armed status for motors 1,2,3,4
 * @return int32_t -1 if error, otherwise length of message
 */
int32_t oq2p_msg_build_arm_response(uint8_t * pbuf, uint16_t buf_len, motor_arm_t * armed_info_buf)
{   
    uint8_t * ppacket = pbuf;

    if(buf_len < (OQ2P_OVERHEAD_LEN + OQ2P_DATA_LEN_ARM_RESPONSE))
    {
        return -1;
    }

    *(pbuf++) = OQ2P_START;
    *(pbuf++) = OQ2P_CLASS_CONTROL;
    *(pbuf++) = OQ2P_MID_ARM;
    *(pbuf++) = OQ2P_RESPONSE;
    *(pbuf++) = 0; // Length MSB - will come back to this.
    *(pbuf++) = 0; // Length LSB - will come back to this.

    *(pbuf++) = armed_info_buf[0];
    *(pbuf++) = armed_info_buf[1];
    *(pbuf++) = armed_info_buf[2];
    *(pbuf++) = armed_info_buf[3];

    OQ2P_ENCODE_DATA_LEN(ppacket, 4);

    *(pbuf++) = OQ2P_END;

    return (pbuf-ppacket);
}