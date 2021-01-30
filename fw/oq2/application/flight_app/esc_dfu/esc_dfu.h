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

// API
uint32_t esc_dfu_init();
uint32_t esc_dfu_erase_chip();
uint32_t esc_dfu_write_app(uint8_t * p_esc_app, uint32_t length);
uint32_t esc_dfu_verify(uint8_t * p_verify, uint32_t length);
uint32_t esc_dfu_start_app();

// Main application calls these callbacks when uart interrupts are received.
void esc_dfu_rx_complete_callback(uint8_t index);
void esc_dfu_tx_complete_callback(uint8_t index);


#endif