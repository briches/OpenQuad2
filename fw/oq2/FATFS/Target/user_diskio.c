/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\FATFS\Target\user_diskio.c                      /
 * Project: OQ2                                                                                    /
 * Created Date: Saturday, February 6th 2021, 9:25:24 am                                           /
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


/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
  /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
 /* USER CODE BEGIN 0 */
 /* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "qspi_flash.h"
#include "S25FL256S.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(FATFS_IO_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(FATFS_IO_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(FATFS_IO_MODULE_ID, fmt, ##__VA_ARGS__)

#define M_SECTOR_SIZE 65536 // (2 * S25FL256S_PAGE_SIZE) 
#define M_ERASE_SIZE 65536 // (S25FL256S_UNIFORM_SECTOR_SIZE)
/* Private variables ---------------------------------------------------------*/

extern OSPI_HandleTypeDef hospi1;

/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

static uint8_t sector_status[512] = {0};

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize(BYTE pdrv);
DSTATUS USER_status(BYTE pdrv);
DRESULT USER_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void* buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize(
    BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
    /* USER CODE BEGIN INIT */
    debug_printf("Fatfs DiskIO Init.");

    if(qspi_flash_init_S25FL256S(&hospi1))
        Stat = 0;
    else
        Stat = STA_NODISK;

    debug_printf("Success: %u", Stat);

    return Stat;
    /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status(
    BYTE pdrv       /* Physical drive number to identify the drive */
)
{
    /* USER CODE BEGIN STATUS */
    // debug_printf("Fatfs DiskIO status: %u", Stat);
    return Stat;
    /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read(
    BYTE pdrv,      /* Physical drive nmuber to identify the drive */
    BYTE* buff,     /* Data buffer to store read data */
    DWORD sector,   /* Sector address in LBA */
    UINT count      /* Number of sectors to read */
)
{
    /* USER CODE BEGIN READ */
    uint32_t address = sector * M_SECTOR_SIZE;
    uint32_t length = count * M_SECTOR_SIZE;

    bool res = qf_read_data(address, buff, length);

    if (res)
        return RES_OK;
    else
        return RES_ERROR;

    /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write(
    BYTE pdrv,          /* Physical drive nmuber to identify the drive */
    const BYTE* buff,   /* Data to be written */
    DWORD sector,       /* Sector address in LBA */
    UINT count          /* Number of sectors to write */
)
{
    // Sector is in "number of pages" of S25FL256S_PAGE_SIZE
    uint32_t address = sector * M_SECTOR_SIZE;
    uint32_t length = count * M_SECTOR_SIZE;

    uint32_t erase_sector = address / M_ERASE_SIZE;

    osDelay(10);

    debug_printf("WRITE: %u fatfs sector(s) from index %u at flash addr 0x%08X,  %u bytes", count, sector,  address, length);

    if(!qf_erase_sector(erase_sector))
            debug_error("Couldn't erase sector... continue with caution.");

    // uint8_t check_buf[512] = {0};
    // qf_read_data(address, check_buf, sizeof(check_buf));

    // uint32_t sum = 0;
    // for(int i = 0; i < sizeof(check_buf); i++)
    // {
    //     sum += (0xFF - check_buf[i]);
    // }
    
    // if(sum > 0)
    // {
    //     debug_printf("Need to erase sector %u", erase_sector);

    //     if(!qf_erase_sector(erase_sector))
    //         debug_error("Couldn't erase sector... continue with caution.");
    // }

    if (qf_write_data(address, (uint8_t*) buff, length))
        return RES_OK;
    else
        return RES_ERROR;
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl(
    BYTE pdrv,      /* Physical drive nmuber (0..) */
    BYTE cmd,       /* Control code */
    void* buff      /* Buffer to send/receive control data */
)
{
    /* USER CODE BEGIN IOCTL */
    DRESULT res = RES_ERROR;
    BSP_SD_CardInfo CardInfo;

    if(Stat & STA_NOINIT)
        debug_printf("ioctl stat %u", Stat);

    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd)
    {
    case CTRL_SYNC:
        /**
         * @brief Makes sure that the device has finished pending write process. 
         * If the disk I/O layer or storage device has a write-back cache, the 
         * dirty cache data must be committed to media immediately. Nothing to 
         * do for this command if each write operation to the media is completed 
         * within the disk_write function.
         */
        res = RES_OK;
        break;

    case GET_SECTOR_COUNT:
        /**
         * @brief Retrieves number of available sectors, the largest allowable 
         * LBA + 1, on the drive into the LBA_t variable pointed by buff. This 
         * command is used by f_mkfs and f_fdisk function to determine the size 
         * of volume/partition to be created. It is required when FF_USE_MKFS == 1.
         */
        *(DWORD*)buff = (S25FL256S_SIZE /  M_SECTOR_SIZE);
        debug_printf("logical sector count: %u", *(DWORD*)buff);
        res = RES_OK;
        break;

    case GET_SECTOR_SIZE:
        /**
         * @brief Retrieves sector size, minimum data unit for generic read/write, 
         * into the WORD variable pointed by buff. Valid sector sizes are 512, 1024, 
         * 2048 and 4096. This command is required only if FF_MAX_SS > FF_MIN_SS. 
         * When FF_MAX_SS == FF_MIN_SS, this command will be never used and the 
         * read/write function must work in FF_MAX_SS bytes/sector.
         */
        *(WORD*)buff = (uint16_t) M_SECTOR_SIZE; // 512
        debug_printf("sector size: 0x%04X", *(WORD*)buff);
        res = RES_OK;
        break;

    case GET_BLOCK_SIZE:
        /**
         * @brief Retrieves erase block size of the flash memory media in unit of 
         * sector into the DWORD variable pointed by buff. The allowable value is 
         * 1 to 32768 in power of 2. Return 1 if the erase block size is unknown 
         * or non flash memory media. This command is used by only f_mkfs function 
         * and it attempts to align data area on the erase block boundary. It is 
         * required when FF_USE_MKFS == 1.
         */
        *(DWORD*)buff = (M_ERASE_SIZE / M_SECTOR_SIZE); // 128
        debug_printf("erase block size %u (in units of sectors)", *(DWORD*)buff);
        res = RES_OK;
        break;

    default:
        res = RES_PARERR;
    }

    return res;
    /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
