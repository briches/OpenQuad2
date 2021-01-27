/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\utility\utility.c                   /
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, January 24th 2021, 8:31:17 am                                             /
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

#include <stdint.h>
#include "utility.h"

/**
 * @brief Routine to calculate the 8 bit XOR checksum of an array of bytes
 * 
 * @param data pointer to start of data
 * @param length length of data
 * @return uint8_t XOR checksum
 */
uint8_t _crc_xor_8(uint8_t * data, uint32_t length)
{
    uint8_t cs = 0x00;

    for(int i = 0; i < length; i++)
    {
        cs ^= data[i];
    }

    return cs;
}

/**
 * @brief Encode the uint32_t number "data" into the buffer, MSB at lowest (First) address
 * Big Endian
 * @param data data to encode
 * @param buf memory location to encode data into 
 * @return uint8_t 
 */
uint32_t uint32_big_encode(uint32_t data, uint8_t * buf)
{
    buf[0] = (data & 0xFF000000) >> 24;
    buf[1] = (data & 0xFF0000) >> 16;
    buf[2] = (data & 0xFF00) >> 8;
    buf[3] = (data & 0xFF) >> 0;
    return 4;
}

/**
 * @brief Encode the uint32_t number "data" into the buffer, LSB at lowest (First) address
 * Little Endian
 * @param data data to encode
 * @param buf memory location to encode data into 
 */
uint32_t uint32_encode(uint32_t data, uint8_t * buf)
{
    buf[3] = (data & 0xFF000000) >> 24;
    buf[2] = (data & 0xFF0000) >> 16;
    buf[1] = (data & 0xFF00) >> 8;
    buf[0] = (data & 0xFF) >> 0;
    return 4;
}