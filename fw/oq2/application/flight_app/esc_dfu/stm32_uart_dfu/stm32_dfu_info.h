/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\flight_app\esc_dfu\stm32_uart_dfu\stm32_dfu_info.h/
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, January 24th 2021, 11:05:34 am                                            /
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


#ifndef STM32_DFU_INFO_H_
#define STM32_DFU_INFO_H_


#include <stdint.h>
#include <stdlib.h>

#define SZ_128 0x00000080
#define SZ_256 0x00000100
#define SZ_1K 0x00000400
#define SZ_2K 0x00000800
#define SZ_16K 0x00004000
#define SZ_32K 0x00008000
#define SZ_64K 0x00010000
#define SZ_128K 0x00020000
#define SZ_256K 0x00040000

typedef enum
{
    F_NO_ME,    /* Mass-Erase not supported */
    F_OBLL,     /* OBL_LAUNCH required */
    F_PEMPTY,   /* clear PEMPTY bit required */
}flags_t;

typedef struct
{
    uint32_t ID;
    char* name;
    uint32_t ram_start;
    uint32_t ram_end;
    uint32_t flash_start;
    uint32_t flash_end;
    uint32_t pages_per_sector;
    uint32_t page_size;
    uint32_t option_start;
    uint32_t option_end;
    uint32_t mem_start;
    uint32_t mem_end;
    uint32_t flags;
} stm32_dev_info_t;


#endif