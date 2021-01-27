/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\flight_app\esc_dfu\esc_dfu.h        /
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, January 24th 2021, 7:18:39 am                                             /
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


#ifndef ESC_DFU_H_
#define ESC_DFU_H_

#include <stdint.h>
#include <stdbool.h>

uint32_t esc_dfu_init();
uint32_t esc_dfu_process();
void esc_dfu_rx_complete_callback(uint8_t index);
void esc_dfu_tx_complete_callback(uint8_t index);

typedef enum
{
    DFU_STATE_INIT,
    DFU_STATE_VERIFY,
    DFU_STATE_DONE,
} dfu_state_t;



#endif