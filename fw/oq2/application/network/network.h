/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\network\wifi\network.h              /
 * Project: OQ2                                                                                    /
 * Created Date: Monday, December 28th 2020, 7:14:48 am                                            /
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


#ifndef NETWORK_H_
#define NETWORK_H_

void network_thread_pre_init();
void network_thread(void* argument);


#endif