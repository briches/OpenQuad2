/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\flight_app\esc_dfu\stm32_uart_dfu\stm32_uart_dfu.c/
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, January 24th 2021, 8:23:17 am                                             /
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


#include "stm32_uart_dfu.h"
#include "utility.h"
#include "string.h"
#include "stm32_dfu_info.h"

#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(STM_UART_DFU_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(STM_UART_DFU_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(STM_UART_DFU_MODULE_ID, fmt, ##__VA_ARGS__)


static bool m_print_rx_chars = false;

const stm32_dev_info_t mStm32DevInfoList[] =
{
    /* F0 */
    {0x440, "STM32F030x8/F05xxx",   0x20000800, 0x20002000, 0x08000000, 0x08010000, 4, SZ_1K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFEC00, 0x1FFFF800, 0},
    {0x442, "STM32F030xC/F09xxx",   0x20001800, 0x20008000, 0x08000000, 0x08040000, 2,SZ_2K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFD800, 0x1FFFF800, F_OBLL},
    {0x444, "STM32F03xx4/6",        0x20000800, 0x20001000, 0x08000000, 0x08008000, 4, SZ_1K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFEC00, 0x1FFFF800, 0},
    {0x445, "STM32F04xxx/F070x6",   0x20001800, 0x20001800, 0x08000000, 0x08008000, 4, SZ_1K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFC400, 0x1FFFF800, 0},
    {0x448, "STM32F070xB/F071xx/F72xx", 0x20001800, 0x20004000, 0x08000000, 0x08020000, 2, SZ_2K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFC800, 0x1FFFF800, 0},
    /* F1 */
    {0x412, "STM32F10xxx Low-density", 0x20000200, 0x20002800, 0x08000000, 0x08008000, 4, SZ_1K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFF000, 0x1FFFF800, 0},
    {0x410, "STM32F10xxx Medium-density", 0x20000200, 0x20005000, 0x08000000, 0x08020000, 4, SZ_1K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFF000, 0x1FFFF800, 0},
    {0x414, "STM32F10xxx High-density", 0x20000200, 0x20010000, 0x08000000, 0x08080000, 2, SZ_2K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFF000, 0x1FFFF800, 0},
    {0x420, "STM32F10xxx Medium-density VL", 0x20000200, 0x20002000, 0x08000000, 0x08020000, 4, SZ_1K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFF000, 0x1FFFF800, 0},
    {0x428, "STM32F10xxx High-density VL", 0x20000200, 0x20008000, 0x08000000, 0x08080000, 2, SZ_2K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFF000, 0x1FFFF800, 0},
    {0x418, "STM32F105xx/F107xx", 0x20001000, 0x20010000, 0x08000000, 0x08040000, 2, SZ_2K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFB000, 0x1FFFF800, 0},
    {0x430, "STM32F10xxx XL-density", 0x20000800, 0x20018000, 0x08000000, 0x08100000, 2, SZ_2K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFE000, 0x1FFFF800, 0},
    /* F2 */
    // {0x411, "STM32F2xxxx", 0x20002000, 0x20020000, 0x08000000, 0x08100000, 1, STM32DevInfo.f2f4, 0x1FFFC000, 0x1FFFC00F, 0x1FFF0000, 0x1FFF7800, 0},
    /* F3 */
    {0x432, "STM32F373xx/F378xx", 0x20001400, 0x20008000, 0x08000000, 0x08040000, 2, SZ_2K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFD800, 0x1FFFF800, 0},
    {0x422, "STM32F302xB(C)/F303xB(C)/F358xx", 0x20001400, 0x2000A000, 0x08000000, 0x08040000, 2, SZ_2K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFD800, 0x1FFFF800, 0},
    {0x439, "STM32F301xx/F302x4(6/8)/F318xx", 0x20001800, 0x20004000, 0x08000000, 0x08010000, 2, SZ_2K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFD800, 0x1FFFF800, 0},
    {0x438, "STM32F303x4(6/8)/F334xx/F328xx", 0x20001800, 0x20003000, 0x08000000, 0x08010000, 2, SZ_2K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFD800, 0x1FFFF800, 0},
    {0x446, "STM32F302xD(E)/F303xD(E)/F398xx", 0x20001800, 0x20010000, 0x08000000, 0x08080000, 2, SZ_2K, 0x1FFFF800, 0x1FFFF80F, 0x1FFFD800, 0x1FFFF800, 0},
    /* F4 */
    // {0x413, "STM32F40xxx/41xxx", 0x20003000, 0x20020000, 0x08000000, 0x08100000, 1, STM32DevInfo.f2f4, 0x1FFFC000, 0x1FFFC00F, 0x1FFF0000, 0x1FFF7800, 0},
    // {0x419, "STM32F42xxx/43xxx", 0x20003000, 0x20030000, 0x08000000, 0x08200000, 1, STM32DevInfo.f4db, 0x1FFEC000, 0x1FFFC00F, 0x1FFF0000, 0x1FFF7800, 0},
    // {0x423, "STM32F401xB(C)", 0x20003000, 0x20010000, 0x08000000, 0x08040000, 1, STM32DevInfo.f2f4, 0x1FFFC000, 0x1FFFC00F, 0x1FFF0000, 0x1FFF7800, 0},
    // {0x433, "STM32F401xD(E)", 0x20003000, 0x20018000, 0x08000000, 0x08080000, 1, STM32DevInfo.f2f4, 0x1FFFC000, 0x1FFFC00F, 0x1FFF0000, 0x1FFF7800, 0},
    // {0x458, "STM32F410xx", 0x20003000, 0x20008000, 0x08000000, 0x08020000, 1, STM32DevInfo.f2f4, 0x1FFFC000, 0x1FFFC00F, 0x1FFF0000, 0x1FFF7800, 0},
    // {0x431, "STM32F411xx", 0x20003000, 0x20020000, 0x08000000, 0x08080000, 1, STM32DevInfo.f2f4, 0x1FFFC000, 0x1FFFC00F, 0x1FFF0000, 0x1FFF7800, 0},
    // {0x441, "STM32F412xx", 0x20003000, 0x20040000, 0x08000000, 0x08100000, 1, STM32DevInfo.f2f4, 0x1FFFC000, 0x1FFFC00F, 0x1FFF0000, 0x1FFF7800, 0},
    // {0x421, "STM32F446xx", 0x20003000, 0x20020000, 0x08000000, 0x08080000, 1, STM32DevInfo.f2f4, 0x1FFFC000, 0x1FFFC00F, 0x1FFF0000, 0x1FFF7800, 0},
    // {0x434, "STM32F469xx/479xx", 0x20003000, 0x20060000, 0x08000000, 0x08200000, 1, STM32DevInfo.f4db, 0x1FFEC000, 0x1FFFC00F, 0x1FFF0000, 0x1FFF7800, 0},
    // {0x463, "STM32F413xx/423xx", 0x20003000, 0x20050000, 0x08000000, 0x08180000, 1, STM32DevInfo.f2f4, 0x1FFFC000, 0x1FFFC00F, 0x1FFF0000, 0x1FFF7800, 0},
    // /* F7 */
    // {0x452, "STM32F72xxx/73xxx", 0x20004000, 0x20040000, 0x08000000, 0x08080000, 1, STM32DevInfo.f2f4, 0x1FFF0000, 0x1FFF001F, 0x1FF00000, 0x1FF0EDC0, 0},
    // {0x449, "STM32F74xxx/75xxx", 0x20004000, 0x20050000, 0x08000000, 0x08100000, 1, STM32DevInfo.f7, 0x1FFF0000, 0x1FFF001F, 0x1FF00000, 0x1FF0EDC0, 0},
    // {0x451, "STM32F76xxx/77xxx", 0x20004000, 0x20080000, 0x08000000, 0x08200000, 1, STM32DevInfo.f7, 0x1FFF0000, 0x1FFF001F, 0x1FF00000, 0x1FF0EDC0, 0},
    /* L0 */
    {0x425, "STM32L031xx/041xx" , 0x20001000, 0x20002000, 0x08000000, 0x08008000, 32, SZ_128 , 0x1FF80000, 0x1FF8001F, 0x1FF00000, 0x1FF01000, 0},
    {0x417, "STM32L05xxx/06xxx" , 0x20001000, 0x20002000, 0x08000000, 0x08010000, 32, SZ_128 , 0x1FF80000, 0x1FF8001F, 0x1FF00000, 0x1FF01000, 0},
    // {0x447, "STM32L07xxx/08xxx" , 0x20002000, 0x20005000, 0x08000000, 0x08030000, 32, SZ_128 , 0x1FF80000, 0x1FF8001F, 0x1FF00000, 0x1FF02000,  Arrays.asList(F_NO_ME)},
    /* L1 */
    // {0x416, "STM32L1xxx6(8/B)" , 0x20000800, 0x20004000, 0x08000000, 0x08020000, 16, SZ_256 , 0x1FF80000, 0x1FF8001F, 0x1FF00000, 0x1FF01000, Arrays.asList(F_NO_ME)},
    {0x429, "STM32L1xxx6(8/B)A" , 0x20001000, 0x20008000, 0x08000000, 0x08020000, 16, SZ_256 , 0x1FF80000, 0x1FF8001F, 0x1FF00000, 0x1FF01000, 0},
    // {0x427, "STM32L1xxxC" , 0x20001000, 0x20008000, 0x08000000, 0x08040000, 16, SZ_256 , 0x1FF80000, 0x1FF8001F, 0x1FF00000, 0x1FF02000, Arrays.asList(F_NO_ME)},
    {0x436, "STM32L1xxxD" , 0x20001000, 0x2000C000, 0x08000000, 0x08060000, 16, SZ_256 , 0x1FF80000, 0x1FF8009F, 0x1FF00000, 0x1FF02000, 0},
    // {0x437, "STM32L1xxxE" , 0x20001000, 0x20014000, 0x08000000, 0x08080000, 16, SZ_256 , 0x1FF80000, 0x1FF8009F, 0x1FF00000, 0x1FF02000, Arrays.asList(F_NO_ME)},
    /* L4 */
    {0x415, "STM32L476xx/486xx" , 0x20003100, 0x20018000, 0x08000000, 0x08100000,  1, SZ_2K  , 0x1FFF7800, 0x1FFFF80F, 0x1FFF0000, 0x1FFF7000, 0},
    /* G0 */
    {0x466, "STM32G03xxx/04xxx" , 0x20001000, 0x20009000, 0x08000000, 0x08020000,  1, SZ_2K  , 0x1FFF7800, 0x1FFFF80F, 0x1FFF0000, 0x1FFF7000, 0},
    {0x460, "STM32G07xxx/08xxx"  , 0x20001000, 0x20009000, 0x08000000, 0x08020000,  1, SZ_2K  , 0x1FFF7800, 0x1FFFF80F, 0x1FFF0000, 0x1FFF7000, 0},
    /* G4 */
    {0x468, "STM32G431xx/441xx" , 0x20004000, 0x20009000, 0x08000000, 0x08020000,  1, SZ_2K  , 0x1FFF7800, 0x1FFFF80F, 0x1FFF0000, 0x1FFF7000, 0},
    {0x469, "STM32G47xxx/48xxx" , 0x20004000, 0x20009000, 0x08000000, 0x08080000,  1, SZ_2K  , 0x1FFF7800, 0x1FFFF80F, 0x1FFF0000, 0x1FFF7000, 0},
    /* These are not (yet) in AN2606: */
    {0x641, "Medium_Density PL" , 0x20000200, 0x20005000, 0x08000000, 0x08020000,  4, SZ_1K  , 0x1FFFF800, 0x1FFFF80F, 0x1FFFF000, 0x1FFFF800, 0},
    {0x9a8, "STM32W-128K" , 0x20000200, 0x20002000, 0x08000000, 0x08020000,  4, SZ_1K  , 0x08040800, 0x0804080F, 0x08040000, 0x08040800, 0},
    {0x9b0, "STM32W-256K" , 0x20000200, 0x20004000, 0x08000000, 0x08040000,  4, SZ_2K  , 0x08040800, 0x0804080F, 0x08040000, 0x08040800, 0},
};

/*********************************************************************************************/
/** Callbacks - Application Must Implement                                                   */
/*********************************************************************************************/
void stm_uart_dfu_tx_complete_callback(stm_uart_dfu_ctx* pctx)
{
    pctx->tx_busy = false;
}

void stm_uart_dfu_rx_complete_callback(stm_uart_dfu_ctx* pctx, uint8_t rx_byte)
{
    pctx->buffer[pctx->received_byte_count] = rx_byte;

    // if(m_print_rx_chars)
        // debug_printf("Rx %02X", rx_byte);

    pctx->received_byte_count++;

    if (rx_byte == STM_DFU_ACK)
        pctx->received_ack_count++;

    pctx->waiting_for_rx = false;
}

/*********************************************************************************************/
/** Static Functions                                                                         */
/*********************************************************************************************/

/**
 * @brief Check the list of supported devices for the given produdct ID
 *
 * @param ID
 * @return stm32_dev_info_t*
 */
static stm32_dev_info_t* _stm_dfu_find_matching_device(uint16_t ID)
{
    uint32_t list_size = sizeof(mStm32DevInfoList) / sizeof(mStm32DevInfoList[0]);

    for (int i = 0; i < list_size; i++)
    {
        if (mStm32DevInfoList[i].ID == ID)
        {
            return (stm32_dev_info_t*)&mStm32DevInfoList[i];
        }
    }

    return NULL;
}

static uint32_t stm_dfu_info_get_page_from_addr(stm32_dev_info_t * pctx, uint32_t address) 
{
    if(address < pctx->flash_start || address > pctx->flash_end)
        return -1;

    return ((address - pctx->flash_start) / pctx->page_size);
}


/**
 * @brief Send data to the device using the provided abstraction function
 * 
 * @param pctx 
 * @param pdata pointer to data
 * @param length length of data
 * @param blocking Write using blocking API or not.
 */
static void _stm_dfu_device_send_(stm_uart_dfu_ctx* pctx, uint8_t* pdata, uint32_t length, bool blocking)
{
    pctx->tx_busy = true;
    pctx->waiting_for_rx = true;
    pctx->received_byte_count = 0;
    pctx->received_ack_count = 0;

    if(pctx->send_data(pctx->handle, pdata, length, blocking) != 0)
        debug_error("Problem sending data with interface function.");
}

/**
 * @brief Use the context's delay function and get_time fun
 * 
 * @param pctx 
 * @param acks 
 * @return true 
 * @return false 
 */
static bool _stm_dfu_timeout_on_acks(stm_uart_dfu_ctx* pctx, uint8_t acks)
{
    float now = pctx->get_time();

    while(pctx->received_ack_count < acks)
    {
        pctx->delay();

        if((pctx->get_time() - now) > pctx->timeout)
            return false;
    }

    return true;
}

/**
 * @brief Send the STM UART DFU Sync Character 0x7F to establish connection 
 * 
 * @param pctx 
 * @return uint32_t error code
 */
static uint32_t _stm_dfu_connect(stm_uart_dfu_ctx * pctx)
{
    uint32_t retval = STM_DFU_NO_ERROR;

    // Send sync character
    pctx->buffer[0] = STM_DFU_SYNC;
    _stm_dfu_device_send_(pctx, pctx->buffer, 1, NON_BLOCKING_INTERFACE_TX);

    // Wait for response. A single ack confirms that we have established communication 
    if(_stm_dfu_timeout_on_acks(pctx, 1) == false)
        return STM_DFU_ERROR_TIMEOUT;

    debug_printf("Got first ACK.");
    retval = STM_DFU_NO_ERROR; // NO error, keep going.
    pctx->state = UART_DFU_STATE_VERSION;

    return retval;
}

static uint32_t _stm_dfu_get_version(stm_uart_dfu_ctx* pctx)
{
    uint32_t retval = STM_DFU_NO_ERROR;

    pctx->buffer[0] = STM_DFU_CMD_GET;
    pctx->buffer[1] = ~pctx->buffer[0];

    _stm_dfu_device_send_(pctx, pctx->buffer, 2, NON_BLOCKING_INTERFACE_TX);

    // Wait for response. A start and finish ACK
    if(_stm_dfu_timeout_on_acks(pctx, 2) == false)
        return STM_DFU_ERROR_TIMEOUT;

    // Handle response
    uint8_t num_bytes = pctx->buffer[1];
    uint8_t version = pctx->buffer[2];

    pctx->version_major = (version & 0xF0) >> 4;
    pctx->version_minor = (version & 0x0F) >> 0;

    debug_printf("DFU version %X.%X", pctx->version_major, pctx->version_minor);
    debug_printf("Parsing command list.");

    // The rest of the bytes here are the list of supported commands.
    // Check for support of the Extended Erase command
    for (int i = 0; i < num_bytes; i++)
    {
        if (pctx->buffer[3 + i] == STM_DFU_CMD_EX_ERASE)
        {
            pctx->use_extended_erase = true;
            debug_printf("Extended Erase supported.");
        }
        if (pctx->buffer[3 + i] == STM_DFU_CMD_ERASE)
        {
            pctx->use_extended_erase = true;
            debug_printf("Erase supported.");
        }
    }

    pctx->state = UART_DFU_STATE_VERSION_RP;

    retval = STM_DFU_NO_ERROR;

    return retval;
}

static uint32_t _stm_dfu_get_version_read_protect(stm_uart_dfu_ctx* pctx)
{
    uint32_t retval = STM_DFU_NO_ERROR;

    // Fill buffer with the command and CRC
    pctx->buffer[0] = STM_DFU_CMD_GET_RP_STAT;
    pctx->buffer[1] = ~pctx->buffer[0];

    _stm_dfu_device_send_(pctx, pctx->buffer, 2, NON_BLOCKING_INTERFACE_TX);

    // Wait for response. A start and finish ACK
    if(_stm_dfu_timeout_on_acks(pctx, 2) == false)
        return STM_DFU_ERROR_TIMEOUT;

    // Handle response
    uint8_t boot_version = pctx->buffer[1];
    uint8_t option1 = pctx->buffer[2];
    uint8_t option2 = pctx->buffer[3];

    debug_printf("RP Boot version %u.%u", (boot_version >> 4), (boot_version & 0x0f));
    debug_printf("option bytes: %u %u", option1, option2);

    pctx->state = UART_DFU_STATE_GET_ID;

    retval = STM_DFU_NO_ERROR;
    
    return retval;
}

static uint32_t _stm_dfu_get_device_id(stm_uart_dfu_ctx* pctx)
{
    uint32_t retval = STM_DFU_NO_ERROR;

    pctx->buffer[0] = STM_DFU_CMD_GET_ID;
    pctx->buffer[1] = ~pctx->buffer[0];

    _stm_dfu_device_send_(pctx, pctx->buffer, 2, NON_BLOCKING_INTERFACE_TX);

    // Wait for response. A start and finish ACK
    if(_stm_dfu_timeout_on_acks(pctx, 2) == false)
        return STM_DFU_ERROR_TIMEOUT;

    // Parse product ID
    uint16_t ID = (pctx->buffer[2]) << 8 | pctx->buffer[3];

    debug_printf("PID 0x%X", ID);

    // Check our known list of IDs for a match.
    pctx->pdev_info = _stm_dfu_find_matching_device(ID);

    // Match not found
    if (pctx->pdev_info == NULL)
    {
        debug_error("Device 0x%X not found in list.");
        retval = STM_DFU_ERROR_DEVICE_NOT_FOUND;
        pctx->state = UART_DFU_STATE_ERROR;
    }
    else
    {
        debug_printf("%s identified!", pctx->pdev_info->name);
        retval = STM_DFU_NO_ERROR;
        pctx->state = UART_DFU_STATE_IDLE;
    }

    return retval;
}


/*********************************************************************************************/
/** API                                                                                      */
/*********************************************************************************************/
/**
 * @brief Initialize the bootloader context using the provided buffer and
 * interface functions
 *
 * @param pctx pointer to context
 * @return uint32_t write data error code according to interface function definition
 */
uint32_t stm_uart_dfu_init(stm_uart_dfu_ctx* pctx)
{
    uint32_t retval = STM_DFU_NO_ERROR;

    pctx->received_ack_count = 0;

    // Set up the context
    memset(pctx->buffer, 0x00, pctx->buffer_size);
    pctx->state = UART_DFU_STATE_INIT;
    pctx->pdev_info = NULL;
    pctx->received_byte_count = 0;
    pctx->received_ack_count = 0;
    pctx->use_extended_erase = false;

    // Send the start character to see if we can get a response.
    retval = _stm_dfu_connect(pctx);
    if(retval != STM_DFU_NO_ERROR)
        return retval;

    // Check the bootloader version with the GET command
    retval = _stm_dfu_get_version(pctx);
    if(retval != STM_DFU_NO_ERROR)
        return retval;

    // Check the read protect version 
    retval = _stm_dfu_get_version_read_protect(pctx);
    if(retval != STM_DFU_NO_ERROR)
        return retval;

    // Get the device ID.
    retval = _stm_dfu_get_device_id(pctx);
    if(retval != STM_DFU_NO_ERROR)
        return retval;

    return retval;
}


/**
 * @brief Erase flash based on address and number of bytes. Can only erase in units of the chips PAGE_SIZE
 * 
 * @param pctx DFU context
 * @param start_address Starting address to erase from. Will round down to the nearest page, sector, region
 * @param num_bytes NUmber of bytes to erase
 * @return uint32_t error code / result code based on or state of the uart DFU context
 */
uint32_t stm_uart_dfu_erase_flash(stm_uart_dfu_ctx* pctx, uint32_t start_address, uint32_t num_bytes)
{
    int32_t start_page = stm_dfu_info_get_page_from_addr(pctx->pdev_info, start_address);
    int32_t end_page = stm_dfu_info_get_page_from_addr(pctx->pdev_info, start_address + num_bytes - 1);

    debug_printf("Start page %u, end page %u", start_page, end_page);

    if(start_page < 0 || end_page < 0)
        return STM_DFU_ERROR_INVALID_ADDRESS;

    // Check correct state
    if(pctx->state < UART_DFU_STATE_IDLE)
        return STM_DFU_ERROR_INVALID_STATE;

    // Check Length for validity (can't extend past flash end)
    if(start_address + num_bytes > pctx->pdev_info->flash_end)
        return STM_DFU_ERROR_INVALID_LENGTH;

    // Check for extended erase or not
    if(!pctx->use_extended_erase)
    {
        debug_error("Only extended erase is supported, lol.");
        return STM_DFU_ERROR_DEVICE_NOT_FOUND;
    }

    pctx->state = UART_DFU_STATE_ERASE_FLASH;

    uint16_t page_erase_code = 0x0000;

    // Check if we are doing a mass erase.
    int32_t chip_num_pages = (pctx->pdev_info->flash_end-pctx->pdev_info->flash_start) / pctx->pdev_info->page_size;
    int32_t pages_to_erase = (end_page - start_page + 1);

    if(chip_num_pages == pages_to_erase)
    {
        debug_printf("Global erase.");
        page_erase_code = STM_DFU_ERASE_CODE_GLOBAL_ERASE;
    }
    else
    {
        debug_printf("Erasing %u pages from 0x%08X.", pages_to_erase, start_address);
        page_erase_code = pages_to_erase;
    }

    // Send extended erase command
    /********* Command EX ERASE ********************************************/
    uint8_t* p = pctx->buffer;
    pctx->buffer[0] = STM_DFU_CMD_EX_ERASE;
    pctx->buffer[1] = ~pctx->buffer[0];

    _stm_dfu_device_send_(pctx, pctx->buffer, 2, NON_BLOCKING_INTERFACE_TX);

    // Wait for response. A start and finish ACK
    if(_stm_dfu_timeout_on_acks(pctx, 1) == false)
        return STM_DFU_ERROR_TIMEOUT;

    debug_printf("Got ACK on EX ERASE Command");

    // Send two bytes N, number of pages to erase.
    // N = 0xFFFY, where Y is from 0 to F means special erase
    // 0xFFFF global mass erase
    // 0xFFFE bank 1 mass erase
    // 0xFFFD bank 2 mass erase

    // Special erase?
    if(page_erase_code == STM_DFU_ERASE_CODE_GLOBAL_ERASE)
    {
        p = pctx->buffer;
        p += uint16_big_encode(page_erase_code, p); 
        *(p++) = _crc_xor_8(pctx->buffer, p-pctx->buffer);

        _stm_dfu_device_send_(pctx, pctx->buffer, p - pctx->buffer, NON_BLOCKING_INTERFACE_TX);

        // Wait for response.
        if(_stm_dfu_timeout_on_acks(pctx, 1) == false)
            return STM_DFU_ERROR_TIMEOUT;
    }
    else
    {      
        // Not a special erase
        // Encode the number of pages to erase minus 1.
        p = pctx->buffer;
        p += uint16_big_encode((pages_to_erase - 1), p);  

        // Encode page index to erase and the checksum of the length and number of pages.
        for(int i = 0; i < pages_to_erase; i++)
        {
            p += uint16_big_encode((start_page + i), p);
        }
        
        // CRC of block
        *(p++) = _crc_xor_8(pctx->buffer, p-pctx->buffer);

        _stm_dfu_device_send_(pctx, pctx->buffer, p - pctx->buffer, NON_BLOCKING_INTERFACE_TX);
        debug_print_buffer(pctx->buffer, p-pctx->buffer, 0, 16);

        // Wait for response.
        if(_stm_dfu_timeout_on_acks(pctx, 1) == false)
            return STM_DFU_ERROR_TIMEOUT;
    }

    debug_printf("Erase completed!");
    pctx->state = UART_DFU_STATE_IDLE;

    return STM_DFU_NO_ERROR;
}


/**
 * @brief Write data to the MCU's flash region.
 * 
 * @param pctx DFU context
 * @param start_address memory address to write data to. Must be in the valid flash region
 * @param num_bytes Number of bytes to write (number of bytes contained at p_data)
 * @param p_data Pointer to data that should be written
 * @return uint32_t error code / result code based on or state of the uart DFU context
 */
uint32_t stm_uart_dfu_write_flash(stm_uart_dfu_ctx* pctx, uint32_t start_address, uint32_t num_bytes, uint8_t * p_data)
{
    uint32_t retval = STM_DFU_NO_ERROR;

    // Check correct state
    if(pctx->state < UART_DFU_STATE_IDLE)
        return STM_DFU_ERROR_INVALID_STATE;

    // Okay, we are in correct state. 
    // Check start address for validity.
    if(start_address < pctx->pdev_info->flash_start)
        return STM_DFU_ERROR_INVALID_ADDRESS;

    // Check Length for validity (can't extend past flash end)
    if(start_address + num_bytes > pctx->pdev_info->flash_end)
        return STM_DFU_ERROR_INVALID_LENGTH;

    // Check data length is a multiple of 4
    if(num_bytes % 4 != 0)
        return STM_DFU_ERROR_INVALID_LENGTH;

    // Okay everything's ok.
    pctx->cmd_write_addr = start_address;
    pctx->cmd_bytes_to_write = num_bytes;
    pctx->state = UART_DFU_STATE_WRITE_FLASH;

    uint32_t bytes_written = 0;

    debug_printf("Writing %u bytes to flash at 0x%08X", pctx->cmd_bytes_to_write, pctx->cmd_write_addr);

    // Execute successive writes until all the data is written.
    while(pctx->cmd_bytes_to_write)
    {
        /********* Command Write ********************************************/
        uint8_t* p = pctx->buffer;
        pctx->buffer[0] = STM_DFU_CMD_WRITE;
        pctx->buffer[1] = ~pctx->buffer[0];

        // debug_print_buffer(pctx->buffer, 2, 0, 16);

        _stm_dfu_device_send_(pctx, pctx->buffer, 2, NON_BLOCKING_INTERFACE_TX);

        // Wait for ACK response.
        if(_stm_dfu_timeout_on_acks(pctx, 1) == false)
            return STM_DFU_ERROR_TIMEOUT;

        /********* Write Address ********************************************/
        p = pctx->buffer;
        p += uint32_big_encode(pctx->cmd_write_addr, p);
        *(p++) = _crc_xor_8(pctx->buffer, p-pctx->buffer);

        // debug_print_buffer(pctx->buffer, p - pctx->buffer, 0, 16);

        _stm_dfu_device_send_(pctx, pctx->buffer, p-pctx->buffer, NON_BLOCKING_INTERFACE_TX);

        // Wait for ACK response.
        if(_stm_dfu_timeout_on_acks(pctx, 1) == false)
            return STM_DFU_ERROR_TIMEOUT;

        /********* Length and Data*******************************************/
        uint16_t chunk_length = 0;
        // THe max length of a write is 256.
        if(pctx->cmd_bytes_to_write > 256)
            chunk_length = 256;
        else
            chunk_length = pctx->cmd_bytes_to_write;
        
        // Encode number of bytes - 1, and the data
        p = pctx->buffer;
        *(p++) = (uint8_t)(chunk_length - 1);
        memcpy(p, p_data + bytes_written, chunk_length);
        p += chunk_length;
        *(p++) = _crc_xor_8(pctx->buffer, p - pctx->buffer);
        
        // Send
        // debug_print_buffer(pctx->buffer, p - pctx->buffer, 0, 16);
        _stm_dfu_device_send_(pctx, pctx->buffer, p-pctx->buffer, NON_BLOCKING_INTERFACE_TX);

        // Wait for response.
        if(_stm_dfu_timeout_on_acks(pctx, 1) == false)
            return STM_DFU_ERROR_TIMEOUT;

        debug_printf("Wrote %u B at 0x%08X", chunk_length, pctx->cmd_write_addr);

        // Update all the state variables so next time through the loop we are good to go
        bytes_written += chunk_length;
        pctx->cmd_bytes_to_write -= chunk_length;
        pctx->cmd_write_addr += chunk_length;
    }

    debug_printf("Write complete.");

    return retval;
}


/**
 * @brief Read data from the STM DFU flash at the location "start_address", and compare to p_verify if provided 
 * 
 * @param pctx context object
 * @param start_address memory address to read memory from. Must be in the valid flash region
 * @param num_bytes Number of bytes to read/compare
 * @param p_verify Pointer to verification data. NULL if no verify
 * @return uint32_t error code / result code based on compare result or state of the uart DFU context
 */
uint32_t stm_uart_dfu_read_flash(stm_uart_dfu_ctx* pctx, uint32_t start_address, uint32_t num_bytes, uint8_t * p_verify)
{
    uint32_t retval = STM_DFU_NO_ERROR;

    // Check correct state
    if(pctx->state < UART_DFU_STATE_IDLE)
        return STM_DFU_ERROR_INVALID_STATE;

    // Okay, we are in correct state. 
    // Check start address for validity.
    if(start_address < pctx->pdev_info->flash_start)
        return STM_DFU_ERROR_INVALID_ADDRESS;

    // Check Length for validity (can't extend past flash end)
    if(start_address + num_bytes > pctx->pdev_info->flash_end)
        return STM_DFU_ERROR_INVALID_LENGTH;

    // Okay everything's ok.
    pctx->cmd_read_addr = start_address;
    pctx->cmd_bytes_to_read = num_bytes;
    pctx->state = UART_DFU_STATE_READ_FLASH;

    uint32_t verified_bytes_count = 0;

    debug_printf("Reading %u bytes from flash at 0x%08X", pctx->cmd_bytes_to_read, pctx->cmd_read_addr);

    while(pctx->cmd_bytes_to_read)
    {
        /********* Command READ ********************************************/
        uint8_t* p = pctx->buffer;
        pctx->buffer[0] = STM_DFU_CMD_READ;
        pctx->buffer[1] = ~pctx->buffer[0];

        _stm_dfu_device_send_(pctx, pctx->buffer, 2, NON_BLOCKING_INTERFACE_TX);

        // Wait for response. A start and finish ACK
        if(_stm_dfu_timeout_on_acks(pctx, 1) == false)
            return STM_DFU_ERROR_TIMEOUT;

        /********* Read Address ********************************************/
        uint32_t addr = pctx->cmd_read_addr;

        p = pctx->buffer;

        // Encode address to read from.
        p += uint32_big_encode(addr, p);
        *(p++) = _crc_xor_8((p - 4), 4);

        _stm_dfu_device_send_(pctx, pctx->buffer, p-pctx->buffer, NON_BLOCKING_INTERFACE_TX);

        // Wait for response.
        if(_stm_dfu_timeout_on_acks(pctx, 1) == false)
            return STM_DFU_ERROR_TIMEOUT;

        /********* Read Num Bytes ********************************************/
        // Encode number of bytes to read
        if(pctx->cmd_bytes_to_read > 256)
            pctx->pending_rx_bytes = 256;
        else
            pctx->pending_rx_bytes = pctx->cmd_bytes_to_read;
        
        pctx->buffer[0] = (uint8_t)(pctx->pending_rx_bytes - 1);
        pctx->buffer[1] = ~pctx->buffer[0];

        _stm_dfu_device_send_(pctx, pctx->buffer, 2, NON_BLOCKING_INTERFACE_TX);

        // Wait for response.
        if(_stm_dfu_timeout_on_acks(pctx, 1) == false)
            return STM_DFU_ERROR_TIMEOUT;

        // Instead of resetting the byte count, we'll include the ACK in this receive. THis avoids a 
        // race depending on the baud rate

        // Here we should now receive the bytes.
        while (pctx->received_byte_count < pctx->pending_rx_bytes + 1)
            pctx->delay();

        // Skip the original ACK
        pctx->received_byte_count -= 1;

        // debug_printf("Got %u bytes", pctx->pending_rx_bytes);
        // debug_print_buffer(pctx->buffer + 1, pctx->received_byte_count, 0, 16);

        // IF we are doing a verify, check now.
        if(p_verify != NULL)
        {
            if(memcmp(p_verify + verified_bytes_count, pctx->buffer + 1, pctx->received_byte_count) != 0)
            {
                debug_error("Verify failed at 0x%08X.", pctx->cmd_read_addr);
                return STM_DFU_ERROR_UNKNOWN;
            }
            else
                debug_printf("Verified %u B at 0x%08X", pctx->pending_rx_bytes, pctx->cmd_read_addr);
            
            verified_bytes_count += pctx->received_byte_count;
        }

        // Check if we are doing another read or not.
        pctx->cmd_bytes_to_read -= pctx->pending_rx_bytes;

        // debug_printf("Rx %u B, %u B remaining.", pctx->received_byte_count, pctx->cmd_bytes_to_read);

        if(!pctx->cmd_bytes_to_read)
        {
            // Break loop
            pctx->state = UART_DFU_STATE_IDLE;
            retval = STM_DFU_NO_ERROR;
            break;
        }
        else
            pctx->cmd_read_addr += pctx->pending_rx_bytes;
    }

    return retval;
}


uint32_t stm_uart_dfu_start_app(stm_uart_dfu_ctx* pctx)
{
    // Check correct state
    if(pctx->state < UART_DFU_STATE_IDLE)
        return STM_DFU_ERROR_INVALID_STATE;

    /********* Command Go ********************************************/
    uint8_t* p = pctx->buffer;
    pctx->buffer[0] = STM_DFU_CMD_GO;
    pctx->buffer[1] = ~pctx->buffer[0];

    _stm_dfu_device_send_(pctx, pctx->buffer, 2, NON_BLOCKING_INTERFACE_TX);

    // Wait for response.
    if(_stm_dfu_timeout_on_acks(pctx, 1) == false)
        return STM_DFU_ERROR_TIMEOUT;


    /********* Jump Address ********************************************/
    uint32_t addr = pctx->pdev_info->flash_start;

    p = pctx->buffer;

    // Encode address to read from.
    p += uint32_big_encode(addr, p);
    *(p++) = _crc_xor_8((p - 4), 4);

    _stm_dfu_device_send_(pctx, pctx->buffer, p-pctx->buffer, NON_BLOCKING_INTERFACE_TX);

    // Wait for response.
    if(_stm_dfu_timeout_on_acks(pctx, 1) == false)
        return STM_DFU_ERROR_TIMEOUT;

    debug_printf("Succesfully started app.");   
}