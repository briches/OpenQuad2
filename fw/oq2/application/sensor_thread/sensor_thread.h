/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Core\Src\sensor_thread\sensor_thread.h
 * Project: OQ2
 * Created Date: Saturday, December 12th 2020, 7:23:52 am
 * Author: Brandon Riches
 * Email: richesbc@gmail.com
 * -----
 * Last Modified: 
 * Modified By: 
 * -----
 * 
 * Copyright (c) 2020 OpenQuad2.
 * All rights reserved.
 * 
 * Redistribution and use in source or binary forms, with or without modification,
 * are not permitted without express written approval of OpenQuad2
 * -----
 * HISTORY:
 */





#ifndef SENSOR_THREAD_H__
#define SENSOR_THREAD_H__

#include "main.h"



extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi2;

void sensor_thread_start(void* argument);


#endif