/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\stability_thread\platform\mems_platform.h/
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, December 13th 2020, 9:34:28 am                                            /
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

#ifndef MEMS_PLATFORM_H_
#define MEMS_PLATFORM_H_

#include <string.h>
#include <stdio.h>

typedef struct {
    void* hbus;
    uint8_t i2c_address;
    uint8_t cs_port;
    uint8_t cs_pin;
} sensbus_t;

int32_t platform_write_imu(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len);
int32_t platform_read_imu(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len);
int32_t platform_write_mag(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len);
int32_t platform_read_mag(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len);
int32_t platform_write_baro(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len);
int32_t platform_read_baro(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len);
void platform_delay(uint32_t ms);


#endif

