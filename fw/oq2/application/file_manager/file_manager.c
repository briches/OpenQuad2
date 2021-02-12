/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\file_manager\file_manager.c         /
 * Project: OQ2                                                                                    /
 * Created Date: Saturday, February 6th 2021, 9:53:41 am                                           /
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


#include "file_manager.h"
#include "fatfs.h"
#include <string.h>

#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(FILE_MANAGER_THREAD_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(FILE_MANAGER_THREAD_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(FILE_MANAGER_THREAD_MODULE_ID, fmt, ##__VA_ARGS__)


static uint8_t working_buffer[512];

void test_init()
{
    FATFS fs;
    FRESULT res;
    debug_printf("Ready!");

    // mount the default drive
    res = f_mount(&fs, "", 0);
    if(res != FR_OK) {
        debug_error("f_mount() failed, res = %d", res);
        return;
    }

    debug_printf("f_mount() done!");

    res = f_mkfs("", FM_ANY | FM_SFD, 0, working_buffer, sizeof(working_buffer));
    if(res != FR_OK) {
        debug_error("f_mkfs() failed, res = %d", res);
        return;
    }

    osDelay(10);

    bool created_fs = false;
    uint8_t count = 0;
    uint32_t freeClust;
    FATFS* fs_ptr = &fs;

    do
    {
        count ++;
        debug_printf("Getting free bytes.");
        res = f_getfree("", &freeClust, &fs_ptr); // Warning! This fills fs.n_fatent and fs.csize!
        if(res != FR_OK) 
        {
            debug_error("f_getfree() failed, res = %d", res);
            debug_printf("Creating FAT FS");
            
            res = f_mkfs("", FM_ANY | FM_SFD, 65536, working_buffer, sizeof(working_buffer));
            if(res != FR_OK) {
                debug_error("f_mkfs() failed, res = %d", res);
                return;
            }
        }
        else
        {
            created_fs = true;
        }
    } while(!created_fs && count < 5);

    if(count == 5)
    {
        debug_error("Process failed.");
        return;
    }

    debug_printf("f_getfree() done!");
    osDelay(10);

    uint32_t totalBlocks = (fs.n_fatent - 2) * fs.csize;
    uint32_t freeBlocks = freeClust * fs.csize;

    debug_printf("Total blocks: %u (%u Mb)", totalBlocks, totalBlocks / 256);
    debug_printf("Free blocks: %u (%u Mb)", freeBlocks, freeBlocks / 256);
    osDelay(10);

    DIR dir;
    res = f_opendir(&dir, "/");
    if(res != FR_OK) {
        debug_error("f_opendir() failed, res = %d", res);
        return;
    }
    osDelay(10);

    FILINFO fileInfo;
    uint32_t totalFiles = 0;
    uint32_t totalDirs = 0;
    debug_printf("--------");
    debug_printf("Root Directory:");
    for(;;) {
        res = f_readdir(&dir, &fileInfo);
        if((res != FR_OK) || (fileInfo.fname[0] == '\0')) {
            break;
        }
        
        if(fileInfo.fattrib & AM_DIR) {
            debug_printf("  DIR  %s", fileInfo.fname);
            totalDirs++;
        } else {
            debug_printf("  FILE %s", fileInfo.fname);
            totalFiles++;
        }
    }
    osDelay(10);

    debug_printf("(total: %lu dirs, %lu files)", totalDirs, totalFiles);
    debug_printf("--------");

    res = f_closedir(&dir);
    if(res != FR_OK) {
        debug_printf("f_closedir() failed, res = %d", res);
        return;
    }

    osDelay(10);

    debug_printf("Writing to log.txt...");

    char writeBuff[128];
    snprintf(writeBuff, sizeof(writeBuff), "Total blocks: %lu (%lu Mb); Free blocks: %lu (%lu Mb)",
        totalBlocks, totalBlocks / 2000,
        freeBlocks, freeBlocks / 2000);

    FIL logFile;
    res = f_open(&logFile, "log.txt", FA_OPEN_APPEND | FA_WRITE | FA_CREATE_NEW);
    if(res != FR_OK) {
        debug_error("f_open() failed, res = %d", res);
        return;
    }

    unsigned int bytesToWrite = strlen(writeBuff);
    unsigned int bytesWritten;
    res = f_write(&logFile, writeBuff, bytesToWrite, &bytesWritten);
    if(res != FR_OK) {
        debug_printf("f_write() failed, res = %d", res);
        return;
    }

    if(bytesWritten < bytesToWrite) {
        debug_printf("WARNING! Disk is full, bytesToWrite = %lu, bytesWritten = %lu", bytesToWrite, bytesWritten);
    }

    res = f_close(&logFile);
    if(res != FR_OK) {
        debug_error("f_close() failed, res = %d", res);
        return;
    }

    debug_printf("Reading file...");
    FIL msgFile;
    res = f_open(&msgFile, "log.txt", FA_READ);
    if(res != FR_OK) {
        debug_error("f_open() failed, res = %d", res);
        return;
    }

    char readBuff[128];
    unsigned int bytesRead;
    res = f_read(&msgFile, readBuff, sizeof(readBuff)-1, &bytesRead);
    if(res != FR_OK) {
        debug_error("f_read() failed, res = %d", res);
        return;
    }

    readBuff[bytesRead] = '\0';
    debug_printf("```%s```", readBuff);

    res = f_close(&msgFile);
    if(res != FR_OK) {
        debug_error("f_close() failed, res = %d", res);
        return;
    }

    // Unmount
    res = f_mount(NULL, "", 0);
    if(res != FR_OK) {
        debug_error("Unmount failed, res = %d", res);
        return;
    }

    debug_printf("Done!");
}

#include "qspi_flash.h"
extern OSPI_HandleTypeDef hospi1;

/**
 * @brief  Function implementing the file manager task thread.
 * @param  argument: Not used
 * @retval None
 */
void file_manager_thread(void* argument)
{
    static int m_state = 0;

    debug_printf("File Manager.");

    test_init();

    // qspi_flash_init_S25FL256S(&hospi1);

    for (;;)
    {
        osDelay(FILEMANAGER_THREAD_PERIOD);
    }
}