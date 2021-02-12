/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\flight_app\esc_dfu\esc_dfu.c        /
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, January 24th 2021, 7:18:34 am                                             /
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


#include "esc_dfu.h"
#ifdef FREERTOS
#include "cmsis_os.h"
#endif
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32_uart_dfu.h"
#include <stdbool.h>
#include "timer.h"
#include "main.h"

#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(ESC_DFU_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(ESC_DFU_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(ESC_DFU_MODULE_ID, fmt, ##__VA_ARGS__)

extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart9;
extern UART_HandleTypeDef huart1;

static stm_uart_dfu_ctx esc4_ctx;
static stm_uart_dfu_ctx esc3_ctx;
static stm_uart_dfu_ctx esc2_ctx;
static stm_uart_dfu_ctx esc1_ctx;
static uint8_t esc4_rx_char;
static uint8_t esc3_rx_char;
static uint8_t esc2_rx_char;
static uint8_t esc1_rx_char;
static uint8_t esc4_buffer[256] = {0};
static uint8_t esc3_buffer[256] = {0};
static uint8_t esc2_buffer[256] = {0};
static uint8_t esc1_buffer[256] = {0};


/**
 * @brief Handler for the UART RX complete interrupt.
 *
 */
void esc_dfu_rx_complete_callback(uint8_t index)
{
    switch (index)
    {
    case 1:
        stm_uart_dfu_rx_complete_callback(&esc1_ctx, esc1_rx_char);
        HAL_UART_Receive_IT(&huart7, &esc1_rx_char, 1);
        break;
    case 2:
        stm_uart_dfu_rx_complete_callback(&esc2_ctx, esc2_rx_char);
        HAL_UART_Receive_IT(&huart8, &esc2_rx_char, 1);
        break;
    case 3:
        stm_uart_dfu_rx_complete_callback(&esc3_ctx, esc3_rx_char);
        HAL_UART_Receive_IT(&huart9, &esc3_rx_char, 1);
        break;
    case 4:
        stm_uart_dfu_rx_complete_callback(&esc4_ctx, esc4_rx_char);
        HAL_UART_Receive_IT(&huart1, &esc4_rx_char, 1);
        break;
    }
}

/**
 * @brief HAndler for UART TX Complete interrupt
 *
 * @param index
 */
void esc_dfu_tx_complete_callback(uint8_t index)
{
    switch (index)
    {
    case 1:
        stm_uart_dfu_tx_complete_callback(&esc1_ctx);
        break;
    case 2:
        stm_uart_dfu_tx_complete_callback(&esc2_ctx);
        break;
    case 3:
        stm_uart_dfu_tx_complete_callback(&esc3_ctx);
        break;
    case 4:
        stm_uart_dfu_tx_complete_callback(&esc4_ctx);
        break;
    }
}

/**
 * @brief Generic interface function to handle sending data over uart for a particular uart handle
 *
 * @param p_handle pointer to uart handle
 * @param p_data pointer to data
 * @param data_len length of data to transmit
 * @return uint32_t status
 */
static uint32_t esc_dfu_send_data(void* p_handle, uint8_t* p_data, uint32_t data_len, bool blocking)
{
    UART_HandleTypeDef* puart = (UART_HandleTypeDef*)p_handle;

    // debug_print_buffer(p_data, data_len, 0, 8);

    if(blocking)
        return HAL_UART_Transmit(puart, p_data, data_len, 100);
    else
        return HAL_UART_Transmit_IT(puart, p_data, data_len);
}

/**
 * @brief Interface function to provide a delay inside the DFU state machine.
 * 
 * osDelay() used if you want lower priority tasks to be able to run
 * taskYIELD() used if you don't care about lower priority tasks, and just want this one to 
 * go as fast as possible.
 * 
 * @param ticks Used for osDelay(ticks). Not used for taskYIELD()
 */
static void esc_dfu_delay()
{
    // osDelay(5);
    taskYIELD();
}

/**
 * @brief Interface function to provide timestamps to use for timeouts in the STM UART DFU lib
 * 
 * @return float Seconds
 */
static float esc_dfu_timestamp()
{
    return timer_get_elapsed();
}

/**
 * @brief Initialize all the dfu contexts and try to connect to the targets
 * 
 * @return uint32_t error code, see STM_DFU_ERROR
 */
uint32_t esc_dfu_init()
{
    debug_printf("ESC DFU Init.");

    MX_UART7_Init();
    MX_UART8_Init();
    MX_UART9_Init();
    MX_USART1_UART_Init();

    esc1_ctx.handle = &huart7;
    esc1_ctx.send_data = esc_dfu_send_data;
    esc1_ctx.delay = esc_dfu_delay;
    esc1_ctx.get_time = esc_dfu_timestamp;
    esc1_ctx.timeout = 10;  // Seconds
    esc1_ctx.buffer = esc1_buffer;
    esc1_ctx.buffer_size = sizeof(esc1_buffer);

    esc2_ctx.handle = &huart8;
    esc2_ctx.send_data = esc_dfu_send_data;
    esc2_ctx.delay = esc_dfu_delay;
    esc2_ctx.get_time = esc_dfu_timestamp;
    esc2_ctx.timeout = 10;  // Seconds
    esc2_ctx.buffer = esc2_buffer;
    esc2_ctx.buffer_size = sizeof(esc2_buffer);

    esc3_ctx.handle = &huart9;
    esc3_ctx.send_data = esc_dfu_send_data;
    esc3_ctx.delay = esc_dfu_delay;
    esc3_ctx.get_time = esc_dfu_timestamp;
    esc3_ctx.timeout = 10;  // Seconds
    esc3_ctx.buffer = esc3_buffer;
    esc3_ctx.buffer_size = sizeof(esc3_buffer);

    esc4_ctx.handle = &huart1;
    esc4_ctx.send_data = esc_dfu_send_data;
    esc4_ctx.delay = esc_dfu_delay;
    esc4_ctx.get_time = esc_dfu_timestamp;
    esc4_ctx.timeout = 10;  // Seconds
    esc4_ctx.buffer = esc4_buffer;
    esc4_ctx.buffer_size = sizeof(esc4_buffer);

    HAL_StatusTypeDef res = HAL_UART_Receive_IT(&huart7, &esc1_rx_char, 1);
    if(res != HAL_OK)
    {
        debug_error("Error with receiving on huart7 %u", res);
        return res;
    }

    res = HAL_UART_Receive_IT(&huart8, &esc2_rx_char, 1);
    if(res != HAL_OK)
    {
        debug_error("Error with receiving on huart8 %u", res);
        return res;
    }

    res = HAL_UART_Receive_IT(&huart9, &esc3_rx_char, 1);
    if(res != HAL_OK)
    {
        debug_error("Error with receiving on huart9 %u", res);
        return res;
    }

    res = HAL_UART_Receive_IT(&huart1, &esc4_rx_char, 1);
    if(res != HAL_OK)
    {
        debug_error("Error with receiving on huart1 %u", res);
        return res;
    }

    osDelay(5);

    debug_printf("Init ESC 1 DFU");
    uint32_t stm_error = stm_uart_dfu_init(&esc1_ctx);
    
    if(stm_error != STM_DFU_NO_ERROR)
        debug_error("Problem initializing STM UART DFU: 0x%X", stm_error);
    else
    {
        debug_printf("Init success. Identified device: %s", esc1_ctx.pdev_info->name);
        debug_printf("Flash start: 0x%08X, %u", esc1_ctx.pdev_info->flash_start, esc1_ctx.pdev_info->flash_start);
        debug_printf("Flash length: 0x%08X, %u", esc1_ctx.pdev_info->flash_end-esc1_ctx.pdev_info->flash_start, esc1_ctx.pdev_info->flash_end-esc1_ctx.pdev_info->flash_start);
        debug_printf("Page Size: 0x%08X, %u", esc1_ctx.pdev_info->page_size, esc1_ctx.pdev_info->page_size);
    }

    osDelay(5);

    debug_printf("Init ESC 2 DFU");
    stm_error = stm_uart_dfu_init(&esc2_ctx);
    
    if(stm_error != STM_DFU_NO_ERROR)
        debug_error("Problem initializing STM UART DFU: 0x%X", stm_error);
    else
    {
        debug_printf("Init success. Identified device: %s", esc2_ctx.pdev_info->name);
        debug_printf("Flash start: 0x%08X, %u", esc2_ctx.pdev_info->flash_start, esc2_ctx.pdev_info->flash_start);
        debug_printf("Flash length: 0x%08X, %u", esc2_ctx.pdev_info->flash_end-esc2_ctx.pdev_info->flash_start, esc2_ctx.pdev_info->flash_end-esc2_ctx.pdev_info->flash_start);
        debug_printf("Page Size: 0x%08X, %u", esc2_ctx.pdev_info->page_size, esc2_ctx.pdev_info->page_size);
    }

    osDelay(5);

    debug_printf("Init ESC 3 DFU");
    stm_error = stm_uart_dfu_init(&esc3_ctx);
    
    if(stm_error != STM_DFU_NO_ERROR)
        debug_error("Problem initializing STM UART DFU: 0x%X", stm_error);
    else
    {
        debug_printf("Init success. Identified device: %s", esc3_ctx.pdev_info->name);
        debug_printf("Flash start: 0x%08X, %u", esc3_ctx.pdev_info->flash_start, esc3_ctx.pdev_info->flash_start);
        debug_printf("Flash length: 0x%08X, %u", esc3_ctx.pdev_info->flash_end-esc3_ctx.pdev_info->flash_start, esc3_ctx.pdev_info->flash_end-esc3_ctx.pdev_info->flash_start);
        debug_printf("Page Size: 0x%08X, %u", esc3_ctx.pdev_info->page_size, esc3_ctx.pdev_info->page_size);
    }

    osDelay(5);

    debug_printf("Init ESC 4 DFU");
    stm_error = stm_uart_dfu_init(&esc4_ctx);
    
    if(stm_error != STM_DFU_NO_ERROR)
        debug_error("Problem initializing STM UART DFU: 0x%X", stm_error);
    else
    {
        debug_printf("Init success. Identified device: %s", esc4_ctx.pdev_info->name);
        debug_printf("Flash start: 0x%08X, %u", esc4_ctx.pdev_info->flash_start, esc4_ctx.pdev_info->flash_start);
        debug_printf("Flash length: 0x%08X, %u", esc4_ctx.pdev_info->flash_end-esc4_ctx.pdev_info->flash_start, esc4_ctx.pdev_info->flash_end-esc4_ctx.pdev_info->flash_start);
        debug_printf("Page Size: 0x%08X, %u", esc4_ctx.pdev_info->page_size, esc4_ctx.pdev_info->page_size);
    }
    

    return stm_error;
}

/**
 * @brief Erase the whole chip flash.
 * 
 * @return uint32_t 
 */
uint32_t esc_dfu_erase_chip()
{
    uint32_t result;

    // Erase the whole flash
    uint32_t dev_size = esc4_ctx.pdev_info->flash_end - esc4_ctx.pdev_info->flash_start;
    uint32_t address = esc4_ctx.pdev_info->flash_start;

      
    debug_printf("Erase addr 0x%08X, size %u", address, dev_size);

    debug_printf("Erasing ESC 1 MCU flash.");  
    result = stm_uart_dfu_erase_flash(&esc1_ctx, address, dev_size);
    if(result != STM_DFU_NO_ERROR)
        debug_error("Erase flash returned error code 0x%X", result);

    debug_printf("Erasing ESC 2 MCU flash.");  
    result = stm_uart_dfu_erase_flash(&esc2_ctx, address, dev_size);
    if(result != STM_DFU_NO_ERROR)
        debug_error("Erase flash returned error code 0x%X", result);

    debug_printf("Erasing ESC 3 MCU flash.");  
    result = stm_uart_dfu_erase_flash(&esc3_ctx, address, dev_size);
    if(result != STM_DFU_NO_ERROR)
        debug_error("Erase flash returned error code 0x%X", result);

    debug_printf("Erasing ESC 4 MCU flash.");  
    result = stm_uart_dfu_erase_flash(&esc4_ctx, address, dev_size);
    if(result != STM_DFU_NO_ERROR)
        debug_error("Erase flash returned error code 0x%X", result);

    

    return result;
}

/**
 * @brief Write the application to the esc MCU
 * 
 * @param p_esc_app Pointer to the app data
 * @param length Length of application data
 * @return uint32_t error or result code
 */
uint32_t esc_dfu_write_app(uint8_t * p_esc_app, uint32_t length)
{
    uint32_t result = 0;

    debug_printf("Writing app to ESC 1");
    result = stm_uart_dfu_write_flash(&esc1_ctx, esc1_ctx.pdev_info->flash_start, length, p_esc_app);
    if(result != STM_DFU_NO_ERROR)
        debug_error("WRITE_FLASH returned error code 0x%X", result);

    debug_printf("Writing app to ESC 2");
    result = stm_uart_dfu_write_flash(&esc2_ctx, esc2_ctx.pdev_info->flash_start, length, p_esc_app);
    if(result != STM_DFU_NO_ERROR)
        debug_error("WRITE_FLASH returned error code 0x%X", result);

    debug_printf("Writing app to ESC 3");
    result = stm_uart_dfu_write_flash(&esc3_ctx, esc3_ctx.pdev_info->flash_start, length, p_esc_app);
    if(result != STM_DFU_NO_ERROR)
        debug_error("WRITE_FLASH returned error code 0x%X", result);

    debug_printf("Writing app to ESC 4");
    result = stm_uart_dfu_write_flash(&esc4_ctx, esc4_ctx.pdev_info->flash_start, length, p_esc_app);
    if(result != STM_DFU_NO_ERROR)
        debug_error("WRITE_FLASH returned error code 0x%X", result);

    return result;
}

/**
 * @brief Read and validate the flash on the target
 * 
 * @param p_verify Pointer to the reference data to use as verification
 * @param length Length of reference data
 * @return uint32_t error or result code
 */
uint32_t esc_dfu_verify(uint8_t * p_verify, uint32_t length)
{
    uint32_t result = 0;

    debug_printf("Verifying app on ESC 1");
    result = stm_uart_dfu_read_flash(&esc1_ctx, esc1_ctx.pdev_info->flash_start, length, p_verify);
    if(result != STM_DFU_NO_ERROR)
        debug_error("READ_FLASH returned error code 0x%X", result);

    debug_printf("Verifying app on ESC 2");
    result = stm_uart_dfu_read_flash(&esc2_ctx, esc2_ctx.pdev_info->flash_start, length, p_verify);
    if(result != STM_DFU_NO_ERROR)
        debug_error("READ_FLASH returned error code 0x%X", result);

    debug_printf("Verifying app on ESC 3");
    result = stm_uart_dfu_read_flash(&esc3_ctx, esc3_ctx.pdev_info->flash_start, length, p_verify);
    if(result != STM_DFU_NO_ERROR)
        debug_error("READ_FLASH returned error code 0x%X", result);

    debug_printf("Verifying app on ESC 4");
    result = stm_uart_dfu_read_flash(&esc4_ctx, esc4_ctx.pdev_info->flash_start, length, p_verify);
    if(result != STM_DFU_NO_ERROR)
        debug_error("READ_FLASH returned error code 0x%X", result);

    return result;
}


uint32_t esc_dfu_start_app()
{   
    stm_uart_dfu_start_app(&esc1_ctx);
    stm_uart_dfu_start_app(&esc2_ctx);
    stm_uart_dfu_start_app(&esc3_ctx);
    stm_uart_dfu_start_app(&esc4_ctx);
}

