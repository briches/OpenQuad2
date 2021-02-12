/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\QSPI_FLASH\qspi_flash.h                 /
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, January 31st 2021, 7:07:10 am                                             /
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


#ifndef QSPI_FLASH_H_
#define QSPI_FLASH_H_

#include <stdbool.h>

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_ospi.h"

bool qspi_flash_init_S25FL256S(OSPI_HandleTypeDef* hospi);
bool qf_erase_sector(uint16_t sector_index);
bool qf_read_data(uint32_t address, uint8_t * buffer, uint32_t length);
bool qf_write_data(uint32_t address, uint8_t * p_data, uint32_t length);

#endif