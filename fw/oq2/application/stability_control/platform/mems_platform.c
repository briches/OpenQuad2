/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\stability_thread\platform\mems_platform.c/
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, December 13th 2020, 9:34:22 am                                            /
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

#include "mems_platform.h"

#include "main.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal_i2c.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_gpio.h"
#include "lsm9ds1_reg.h"
#include "lps22hh_reg.h"

/*
 * @brief  Write generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write_imu(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len)
{
    sensbus_t* sensbus = (sensbus_t*)handle;

    HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
    return 0;
}

/*
 * @brief  Write generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write_mag(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len)
{
    sensbus_t* sensbus = (sensbus_t*)handle;

    /* Write multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
    return 0;
}

/*
 * @brief  Read generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read_imu(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len)
{
    sensbus_t* sensbus = (sensbus_t*)handle;

    HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
    return 0;
}

/*
 * @brief  Read generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read_mag(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len)
{
    sensbus_t* sensbus = (sensbus_t*)handle;

    /* Read multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
    return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write_baro(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len)
{
    uint8_t tx_buffer[64] = {0};

    tx_buffer[0] = reg;
    memcpy(tx_buffer+1, bufp, len);

    HAL_SPI_Transmit(handle, tx_buffer, len+1, 100);
    return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read_baro(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len)
{
    uint8_t tx_buffer[64] = {0};
    uint8_t rx_buffer[64] = {0};

    reg |= 0x80;

    tx_buffer[0] = reg;
    HAL_SPI_TransmitReceive(handle, tx_buffer, rx_buffer, len+1, 100);

    memcpy(bufp, rx_buffer+1, len);
    return 0;
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
void platform_delay(uint32_t ms)
{
    if(osKernelGetState() == osKernelRunning)
        osDelay(ms);
    else
        HAL_Delay(ms);
}