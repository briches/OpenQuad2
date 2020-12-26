/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\timer\timer.h                       /
 * Project: OQ2                                                                                    /
 * Created Date: Friday, December 25th 2020, 8:14:17 am                                            /
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


#ifndef TIMER_H_INCL_
#define TIMER_H_INCL_

#include "main.h"

extern LPTIM_HandleTypeDef hlptim1;

void timer_time_start();

void timer_lptim_match_callback();

float timer_get_elapsed();

#endif