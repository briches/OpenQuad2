/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\stability_control\stability_thread.h/
 * Project: OQ2                                                                                    /
 * Created Date: Saturday, December 12th 2020, 7:23:52 am                                          /
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


#ifndef STABILITY_H__
#define STABILITY_H__

#include "main.h"

extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi2;

void stability_thread_pre_init();
void stability_thread(void* argument);

void imu_int1_callback();
void imu_int2_callback();
void imu_mag_drdy_callback();
void baro_int_callback();


#endif