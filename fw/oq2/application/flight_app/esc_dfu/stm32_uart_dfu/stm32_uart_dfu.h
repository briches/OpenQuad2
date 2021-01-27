/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\flight_app\esc_dfu\stm32_uart_dfu\stm32_uart_dfu.h/
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, January 24th 2021, 8:23:32 am                                             /
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


#ifndef STM32_UART_DFU_H_
#define STM32_UART_DFU_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32_dfu_info.h"

/*********************************************************************************************/
/* Info -----------------------------------------------------------------------------*/
// See ST AN3155 https://www.st.com/resource/en/application_note/cd00264342-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf
// for more info

/*********************************************************************************************/
/* DFU Commands -----------------------------------------------------------------------------*/
#define STM_DFU_CMD_GET             0x00
#define STM_DFU_CMD_GET_RP_STAT     0x01
#define STM_DFU_CMD_GET_ID          0x02
#define STM_DFU_CMD_READ            0x11
#define STM_DFU_CMD_GO              0x21
#define STM_DFU_CMD_WRITE           0x31
#define STM_DFU_CMD_ERASE           0x43
#define STM_DFU_CMD_EX_ERASE        0x44
#define STM_DFU_CMD_WRITE_PROTECT   0x63
#define STM_DFU_CMD_WRITE_UNPROTECT 0x73
#define STM_DFU_CMD_READ_PROTECT    0x82
#define STM_DFU_CMD_READ_UNPROTECT  0x92
#define STM_DFU_CMD_GET_CRC         0xA1

#define STM_DFU_SYNC                0x7F
#define STM_DFU_ACK                 0x79
#define STM_DFU_NACK                0x1F

/*********************************************************************************************/
/* DFU State machine ------------------------------------------------------------------------*/

typedef enum
{
    UART_DFU_STATE_INIT,
    UART_DFU_STATE_VERSION,
    UART_DFU_STATE_VERSION_RP, 
    UART_DFU_STATE_GET_ID,
    UART_DFU_STATE_GET_ID_RESPONSE,
    UART_DFU_STATE_READ_FLASH,
    UART_DFU_STATE_ERROR,
    UART_DFU_STATE_IDLE,
} stm_uart_dfu_state_t;

#define STM_DFU_NO_ERROR                0x00000000
#define STM_DFU_ERROR_TIMEOUT           0x00000001
#define STM_DFU_ERROR_NACK              0x00000002
#define STM_DFU_ERROR_RETRY             0x00000003
#define STM_DFU_ERROR_DEVICE_NOT_FOUND  0x00000004
#define STM_DFU_ERROR_INVALID_STATE     0x00000005
#define STM_DFU_ERROR_INVALID_ADDRESS   0x00000006
#define STM_DFU_ERROR_INVALID_LENGTH    0x00000007
#define STM_DFU_ERROR_UNKNOWN           0xFFFFFFFF

/*********************************************************************************************/
/* Generic interface functions---------------------------------------------------------------*/
// Handle, pointer to data, data length, blocking or not
#define BLOCKING_INTERFACE_TX true
#define NON_BLOCKING_INTERFACE_TX false
typedef uint32_t(*stmdfu_write_ptr)(void*, uint8_t*, uint32_t, bool);
typedef uint32_t(*stmdfu_read_ptr) (void*, uint8_t*, uint32_t, bool);
typedef void (*stmdfu_wait_function)();
typedef float (*stmdfu_timestamp_function)();

/*********************************************************************************************/
/* DFU context structure --------------------------------------------------------------------*/
typedef struct
{
    stm_uart_dfu_state_t state;

    // Interface function for writing data
    stmdfu_write_ptr  write_data;

    // Interface function for delay (see FreeRTOS vTaskYield(), or osDelay())
    stmdfu_wait_function delay;

    stmdfu_timestamp_function get_time;

    // Pointer to device specific HAL layer. ex. UART handle. 
    void* handle;

    // Discovered device info, flash size, page size, etc.
    stm32_dev_info_t * pdev_info;

    // DFU bootloader version
    uint8_t version_major;
    uint8_t version_minor;

    // Upon parsing the supported commands, if the device supports extended erase
    bool use_extended_erase;

    // Buffer for writing and reading packets to and from
    uint8_t* buffer;
    uint32_t buffer_size;

    // Flags
    bool tx_busy;
    bool waiting_for_rx;

    // Timeout in the same units as what ever the get_time function returns. Seconds, for example.
    float timeout;
    
    uint32_t pending_rx_bytes;      // In a read operation, how many bytes are expected to receive
    uint32_t received_byte_count;   // The count of received bytes
    uint32_t received_ack_count;    // The count of received acks

    // Functions
    uint32_t cmd_read_addr;
    uint32_t cmd_bytes_to_read;

    uint32_t cmd_write_addr;
    uint32_t cmd_bytes_to_write;

} stm_uart_dfu_ctx;


void stm_uart_dfu_tx_complete_callback(stm_uart_dfu_ctx * pctx);
void stm_uart_dfu_rx_complete_callback(stm_uart_dfu_ctx * pctx, uint8_t rx_byte);

uint32_t stm_uart_dfu_init(stm_uart_dfu_ctx* pctx);
uint32_t stm_uart_dfu_read_flash(stm_uart_dfu_ctx* pctx, uint32_t start_address, uint32_t num_bytes, uint8_t * p_verify);


#endif