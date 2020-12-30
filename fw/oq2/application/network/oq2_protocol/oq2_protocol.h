/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\network\oq2_protocol\oq2_protocol.h /
 * Project: OQ2                                                                                    /
 * Created Date: Wednesday, December 30th 2020, 9:25:54 am                                         /
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


#ifndef OQ2_PROTOCOL_H_
#define OQ2_PROTOCOL_H_

#include <stdint.h>

void oq2p_connect_callback(int8_t sock);
void oq2p_disconnect_callback(int8_t sock);
void oq2p_receive_callback(int8_t sock);

#endif