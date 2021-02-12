/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\qspi_flash\S25FL256S.h              /
 * Project: OQ2                                                                                    /
 * Created Date: Monday, February 1st 2021, 5:17:16 pm                                             /
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


#ifndef S25FL256S_H_
#define S25FL256S_H_

// The populated flash is two different blocks, with different sectors:
// Erase Block Region 1:
// 32 Sectors (0x001Fh)
// 4-kB sectors = 256B x 0x0010h

// Erase Block Region 2:
// 510 Sectors = 510-1 = 0x01FD 
// 64kB sectors = 0x0100h x 256B

#define S25FL256S_SIZE                              0x02000000 // 256 Mb

// Sizes and numbers of sectors
#define S25FL256S_PARAMETER_SECTOR_SIZE   0x1000 // 4 kB sectors
#define S25FL256S_PARAMETER_SECTOR_COUNT  32
#define S25FL256S_UNIFORM_SECTOR_SIZE   0x10000 // 64 kB sectors
#define S25FL256S_UNIFORM_SECTOR_COUNT  510

// Sector indexes
#define S25FL256S_FIRST_PARAMETER_SECTOR_INDEX      0
#define S25FL256S_FIRST_UNIFORM_SECTOR_INDEX        S25FL256S_PARAMETER_SECTOR_COUNT

// Addresses of sectors
#define S25FL256S_PARAMETER_SECTION_START_ADDR      0x00000000 // Starts at beginning
#define S25FL256S_PARAMETER_SECTION_END_ADDR        0x0001FFFF // Ends at 128 kB - 1
#define S25FL256S_UNIFORM_SECTION_START_ADDR        0x00020000 // offset by 128 kB
#define S25FL256S_UNIFORM_SECTION_END_ADDR          0x01FFFFFF   // 256 Mb
#define S25FL256S_LAST_ADDR                         S25FL256S_UNIFORM_SECTION_END_ADDR
// Page Sizes
#define S25FL256S_PAGE_SIZE 0x100

/* S25FL256S Commands */
#define CMD_READ_JEDEC_ID       0x9F
#define CMD_READ_STATUS_REG1    0x05
#define CMD_WREN                0x06
// Erase Commands
#define CMD_ERASE_4KB_4PAE_32B  0x21
#define CMD_ERASE_SECTOR        0xDC
#define CMD_BULK_ERASE          0x60 // Or, 0xC7 ??
// Program Commands
#define CMD_PAGE_PGM_4PP        0x12
#define CMD_SDR_QUAD_IO_PGM_4QPP 0x34  // 4QPP
// Read Commands
#define CMD_4READ               0x13
#define CMD_QUAD_IO_READ_4QOR   0x6C
#define CMD_DDR_QUAD_IO_READ    0xEE   // 4DDRQIOR


/* S25FL256S Bitmasks */
#define BITMASK_WRITE_ENABLE_LATCH 0x02
#define MATCH_WRITE_ENABLED 0x02

#define BITMASK_OP_IN_PROGRESS 0x01
#define MATCH_OP_IN_PROGRESS 0x01
#define MATCH_NO_OP_IN_PROGRESS 0x00


/* Status Register 1 (SR1) */
typedef struct
{
    uint8_t wip : 1;
    uint8_t wel : 1;
    uint8_t bp0 : 1;
    uint8_t bp1 : 1;
    uint8_t bp2 : 1;
    uint8_t e_err : 1;
    uint8_t p_err : 1;
    uint8_t srwd : 1;
} s25fl_status1_reg_t;


#endif