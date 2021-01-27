/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\utility\utility.h                   /
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, January 24th 2021, 8:31:13 am                                             /
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


#ifndef UTILITY_H_
#define UTILITY_H_

uint8_t _crc_xor_8(uint8_t * data, uint32_t length);

uint32_t uint32_big_encode(uint32_t data, uint8_t * buf);
uint32_t uint32_encode(uint32_t data, uint8_t * buf);



#endif