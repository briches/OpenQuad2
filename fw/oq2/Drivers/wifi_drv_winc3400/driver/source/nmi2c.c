/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\wifi_drv_winc3400\driver\source\nmi2c.c /
 * Project: OQ2                                                                                    /
 * Created Date: Monday, December 28th 2020, 6:48:01 am                                            /
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


/**
 *
 * \file
 *
 * \brief This module contains WINC3400 I2C protocol bus APIs implementation.
 *
 * Copyright (c) 2017-2019 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include "nm_common.h"

#ifdef CONF_WINC_USE_I2C

#include "nmi2c.h"
#include "nm_bus_wrapper.h"


#include "debug_log.h"
#define M2M_ERR(fmt, ...)               debug_error(WINC3400_NMI2C_MODULE_ID, fmt, ##__VA_ARGS__)
#define M2M_INFO(fmt, ...)              debug_printf(WINC3400_NMI2C_MODULE_ID, fmt, ##__VA_ARGS__)


/**
 *  @fn         nm_i2c_read_reg_with_ret
 *  @brief      Read register with error code return
 *  @param[in]  u32Addr
 *                  Register address
 *  @param[out] pu32RetVal
 *                  Pointer to u32 variable used to return the read value
 *  @return     @ref M2M_SUCCESS in case of success and @ref M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_i2c_read_reg_with_ret(uint32_t u32Addr, uint32_t* pu32RetVal)
{
    uint8_t b[6];
    uint8_t rsz;
    tstrNmI2cDefault strI2c;
    int8_t s8Ret = M2M_SUCCESS;

    if(u32Addr < 0xff) { /* clockless i2c */
        b[0] = 0x09;
        b[1] = (uint8_t)(u32Addr);
        rsz = 1;
        strI2c.u16Sz = 2;
    } else {
        b[0] = 0x80;
        b[1] = (uint8_t)(u32Addr >> 24);
        b[2] = (uint8_t)(u32Addr >> 16);
        b[3] = (uint8_t)(u32Addr >> 8);
        b[4] = (uint8_t)(u32Addr);
        b[5] = 0x04;
        rsz = 4;
        strI2c.u16Sz = 6;
    }

    strI2c.pu8Buf = b;

    if(M2M_SUCCESS == nm_bus_ioctl(NM_BUS_IOCTL_W, &strI2c))
    {
        strI2c.u16Sz = rsz;
        if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strI2c))
        {
            M2M_ERR("read error\n");
            s8Ret = M2M_ERR_BUS_FAIL;
        }
    }
    else
    {
        M2M_ERR("failed to send cfg bytes\n");
        s8Ret = M2M_ERR_BUS_FAIL;
    }

    if(rsz == 1) {
        *pu32RetVal = b[0];
    } else {
        *pu32RetVal = b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    }
    return s8Ret;
}

/**
 *  @fn         nm_i2c_read_reg
 *  @brief      Read register
 *  @param[in]  u32Addr
 *                  Register address
 *  @return     Register value
 */
uint32_t nm_i2c_read_reg(uint32_t u32Addr)
{
    uint32_t val;
    nm_i2c_read_reg_with_ret(u32Addr, &val);
    return val;
}

/**
 *  @fn         nm_i2c_write_reg
 *  @brief      Write register
 *  @param[in]  u32Addr
 *                  Register address
 *  @param[in]  u32Val
 *                  Value to be written to the register
 *  @return     @ref M2M_SUCCESS in case of success and @ref M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_i2c_write_reg(uint32_t u32Addr, uint32_t u32Val)
{
    tstrNmI2cDefault strI2c;
    uint8_t b[16];
    int8_t s8Ret = M2M_SUCCESS;

    if(u32Addr < 0xff) { /* clockless i2c */
        b[0] = 0x19;
        b[1] = (uint8_t)(u32Addr);
        b[2] = (uint8_t)(u32Val);
        strI2c.u16Sz = 3;
    } else {
        b[0] = 0x90;
        b[1] = (uint8_t)(u32Addr >> 24);
        b[2] = (uint8_t)(u32Addr >> 16);
        b[3] = (uint8_t)(u32Addr >> 8);
        b[4] = (uint8_t)u32Addr;
        b[5] = 0x04;
        b[6] = (uint8_t)u32Val;
        b[7] = (uint8_t)(u32Val >> 8);
        b[8] = (uint8_t)(u32Val >> 16);
        b[9] = (uint8_t)(u32Val >> 24);
        strI2c.u16Sz = 10;
    }

    strI2c.pu8Buf = b;

    if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_W, &strI2c))
    {
        M2M_ERR("write error\n");
        s8Ret = M2M_ERR_BUS_FAIL;
    }

    return s8Ret;
}

/**
 *   @fn         nm_i2c_read_block
 *   @brief      Read block of data
 *   @param[in]  u32Addr
 *                   Start address
 *   @param[out] puBuf
 *                   Pointer to a buffer used to return the read data
 *   @param[in]  u16Sz
 *                   Number of bytes to read. The buffer size must be >= u16Sz
 *   @return     @ref M2M_SUCCESS in case of success and @ref M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_i2c_read_block(uint32_t u32Addr, uint8_t *pu8Buf, uint16_t u16Sz)
{
    tstrNmI2cDefault strI2c;
    uint8_t au8Buf[7];
    int8_t s8Ret = M2M_SUCCESS;

    au8Buf[0] = 0x02;
    au8Buf[1] = (uint8_t)(u32Addr >> 24);
    au8Buf[2] = (uint8_t)(u32Addr >> 16);
    au8Buf[3] = (uint8_t)(u32Addr >> 8);
    au8Buf[4] = (uint8_t)(u32Addr >> 0);
    au8Buf[5] = (uint8_t)(u16Sz >> 8);
    au8Buf[6] = (uint8_t)(u16Sz);

    strI2c.pu8Buf = au8Buf;
    strI2c.u16Sz = sizeof(au8Buf);

    if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_W, &strI2c))
    {
        M2M_ERR("write error\n");
        s8Ret = M2M_ERR_BUS_FAIL;
    }
    else
    {
        strI2c.pu8Buf = pu8Buf;
        strI2c.u16Sz = u16Sz;

        if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_R, &strI2c))
        {
            M2M_ERR("read error\n");
            s8Ret = M2M_ERR_BUS_FAIL;
        }
    }

    return s8Ret;
}

/**
 *  @fn         nm_i2c_write_block
 *  @brief      Write block of data
 *  @param[in]  u32Addr
 *                  Start address
 *  @param[in]  puBuf
 *                  Pointer to the buffer holding the data to be written
 *  @param[in]  u16Sz
 *                  Number of bytes to write. The buffer size must be >= u16Sz
 *  @return     @ref M2M_SUCCESS in case of success and @ref M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_i2c_write_block(uint32_t u32Addr, uint8_t *pu8Buf, uint16_t u16Sz)
{
    uint8_t au8Buf[7];
    tstrNmI2cSpecial strI2c;
    int8_t s8Ret = M2M_SUCCESS;

    au8Buf[0] = 0x12;
    au8Buf[1] = (uint8_t)(u32Addr >> 24);
    au8Buf[2] = (uint8_t)(u32Addr >> 16);
    au8Buf[3] = (uint8_t)(u32Addr >> 8);
    au8Buf[4] = (uint8_t)(u32Addr);
    au8Buf[5] = (uint8_t)(u16Sz >> 8);
    au8Buf[6] = (uint8_t)(u16Sz);

    strI2c.pu8Buf1 = au8Buf;
    strI2c.pu8Buf2 = pu8Buf;
    strI2c.u16Sz1 = sizeof(au8Buf);
    strI2c.u16Sz2 = u16Sz;

    if(M2M_SUCCESS != nm_bus_ioctl(NM_BUS_IOCTL_W_SPECIAL, &strI2c))
    {
        M2M_ERR("write error\n");
        s8Ret = M2M_ERR_BUS_FAIL;
    }

    return s8Ret;
}

#endif
/* EOF */
