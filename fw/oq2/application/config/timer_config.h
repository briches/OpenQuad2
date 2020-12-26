/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\config\timer_config.h               /
 * Project: OQ2                                                                                    /
 * Created Date: Friday, December 25th 2020, 9:05:52 am                                            /
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


#ifndef TIMER_CONFIG_H_
#define TIMER_CONFIG_H_

/**
 * The clock is PCLK1, with divider 128 -> 1074218.75 Hz
 * 
 * So F_match = PCLK1 / 128 / period
 * 
 * So DC1F gives a match frequency of 20Hz
 * 
 */
#define TC_TIMER_MATCH_VALUE    0xD1CF
#define TC_TIMER_TICK_PERIOD_S  0.0000009309091f
#define TC_TIMER_MATCH_PERIOD_S 0.05000005818f

#endif