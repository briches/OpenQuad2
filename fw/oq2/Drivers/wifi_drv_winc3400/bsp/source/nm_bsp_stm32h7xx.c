/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\wifi_drv_winc3400\bsp\source\nm_bsp_stm32h7xx.c/
 * Project: OQ2                                                                                    /
 * Created Date: Monday, December 28th 2020, 6:01:59 pm                                            /
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
#include "nm_bsp.h"
#include "nm_common.h"
#include "wifi_config.h"
#include "main.h"
#include "FreeRTOS.h"

#include "debug_log.h"
#define M2M_ERR(fmt, ...)           debug_error(WINC3400_BSP_MODULE_ID, fmt, ##__VA_ARGS__)
#define M2M_INFO(fmt, ...)          debug_printf(WINC3400_BSP_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(WINC3400_BSP_MODULE_ID, fmt, ##__VA_ARGS__)

static tpfNmBspIsr gpfIsr;
static bool volatile pinInterruptEnabled = false;

void winc3400_pin_isr()
{
    if (gpfIsr && pinInterruptEnabled) {
        gpfIsr();
    }
}

/*
 *    @fn        init_chip_pins
 *    @brief    Initialize reset, chip enable and wake pin
 */
static void init_chip_pins(void)
{
    HAL_GPIO_WritePin(WIFI_RESETN_GPIO_Port, WIFI_RESETN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WIFI_CHIP_EN_GPIO_Port, WIFI_CHIP_EN_Pin, GPIO_PIN_RESET);
}

/*
*    @fn        nm_bsp_init
*    @brief    Initialize BSP
*    @return    0 in case of success and -1 in case of failure
*/
int8_t nm_bsp_init(void)
{
    gpfIsr = NULL;

    /* Initialize chip IOs. */
    init_chip_pins();

    /* Perform chip reset. */
    nm_bsp_reset();

    return 0;
}

/**
*   @fn      nm_bsp_deinit
*   @brief   De-iInitialize BSP
*   @return  0 in case of success and -1 in case of failure
*/
int8_t nm_bsp_deinit(void)
{
    HAL_GPIO_WritePin(WIFI_RESETN_GPIO_Port, WIFI_RESETN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WIFI_CHIP_EN_GPIO_Port, WIFI_CHIP_EN_Pin, GPIO_PIN_RESET);
    return M2M_SUCCESS;
}

/**
 *    @fn        nm_bsp_reset
 *    @brief    Reset the WINC SoC by setting CHIP_EN and RESET_N signals low,
 *           CHIP_EN high then RESET_N high
 */
void nm_bsp_reset(void)
{
    HAL_GPIO_WritePin(WIFI_RESETN_GPIO_Port, WIFI_RESETN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WIFI_CHIP_EN_GPIO_Port, WIFI_CHIP_EN_Pin, GPIO_PIN_RESET);
    nm_bsp_sleep(1);
    HAL_GPIO_WritePin(WIFI_CHIP_EN_GPIO_Port, WIFI_CHIP_EN_Pin, GPIO_PIN_SET);
    nm_bsp_sleep(10);
    HAL_GPIO_WritePin(WIFI_RESETN_GPIO_Port, WIFI_RESETN_Pin, GPIO_PIN_SET);
    nm_bsp_sleep(5);
}

/*
*    @fn        nm_bsp_sleep
*    @brief    Sleep in units of mSec
*    @param[IN]    u32TimeMsec
*                Time in milliseconds
*/
void nm_bsp_sleep(uint32_t u32TimeMsec)
{
    if(osKernelGetState() == osKernelRunning)
    {
        osDelay(u32TimeMsec);
    }
    else
    {
        HAL_Delay(u32TimeMsec);
    }
}

/*
*    @fn        nm_bsp_register_isr
*    @brief    Register interrupt service routine
*    @param[IN]    pfIsr
*                Pointer to ISR handler
*/
void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{
    gpfIsr = pfIsr;

    pinInterruptEnabled = true;
}

/*
*    @fn        nm_bsp_interrupt_ctrl
*    @brief    Enable/Disable interrupts
*    @param[IN]    u8Enable
*                '0' disable interrupts. '1' enable interrupts
*/
void nm_bsp_interrupt_ctrl(uint8_t u8Enable)
{
    pinInterruptEnabled = u8Enable;
}