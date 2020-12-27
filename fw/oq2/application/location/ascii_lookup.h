/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\location\nmea_checksum_lookup.h     /
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, December 27th 2020, 7:18:56 am                                            /
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


#ifndef _ASCII_LOOKUP_H_
#define _ASCII_LOOKUP_H_

#include <stdint.h>

// Looking up a character - '0' will return it's value in decimal. 
const uint8_t ASCII_LOOKUP[] = 
{
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    0,  // :
    0,  // ;
    0,  // <
    0,  // =
    0,  // >
    0,  // ?
    0,  // @
    10, // A
    11, // B
    12, // C
    13, // D
    14, // E
    15, // F
};

#endif