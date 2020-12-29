/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\wifi_drv_winc3400\driver\source\nmbus.h /
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
 * \brief This module contains WINC3400 bus APIs implementation.
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

#ifndef _NMBUS_H_
#define _NMBUS_H_

#include "nm_common.h"
#include "nm_bus_wrapper.h"


#ifdef __cplusplus
extern "C" {
#endif
/**
 *  @fn         nm_bus_iface_init
 *  @brief      Initialize bus interface
 *  @return     @ref M2M_SUCCESS in case of success and @ref M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_bus_iface_init(uint8_t *pvInitVal, uint32_t req_serial_number);

/**
 *  @fn         nm_bus_iface_deinit
 *  @brief      Deinitialize bus interface
 *  @return     @ref M2M_SUCCESS in case of success and @ref M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_bus_iface_deinit(void);

/**
 *  @fn         nm_bus_reset
 *  @brief      Reset bus interface
 *  @return     @ref M2M_SUCCESS in case of success and @ref M2M_ERR_BUS_FAIL in case of failure
 *  @version    1.0
 */
int8_t nm_bus_reset(void);

/**
 *  @fn         nm_bus_iface_reconfigure
 *  @brief      Reconfigure bus interface
 *  @return     @ref M2M_SUCCESS in case of success and @ref M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_bus_iface_reconfigure(void *ptr);

/**
 *  @fn         nm_read_reg
 *  @brief      Read register
 *  @param[in]  u32Addr
 *                  Register address
 *  @return     Register value
 */
uint32_t nm_read_reg(uint32_t u32Addr);

/**
 *  @fn         nm_read_reg_with_ret
 *  @brief      Read register with error code return
 *  @param[in]  u32Addr
 *                  Register address
 *  @param[out] pu32RetVal
 *                  Pointer to u32 variable used to return the read value
 *  @return     @ref M2M_SUCCESS in case of success and @ref M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_read_reg_with_ret(uint32_t u32Addr, uint32_t* pu32RetVal);

/**
 *  @fn         nm_write_reg
 *  @brief      Write register
 *  @param[in]  u32Addr
 *                  Register address
 *  @param[in]  u32Val
 *                  Value to be written to the register
 *  @return     @ref M2M_SUCCESS in case of success and @ref M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_write_reg(uint32_t u32Addr, uint32_t u32Val);

/**
 *  @fn         nm_read_block
 *  @brief      Read block of data
 *  @param[in]  u32Addr
 *                  Start address
 *  @param[out] puBuf
 *                  Pointer to a buffer used to return the read data
 *  @param[in]  u32Sz
 *                  Number of bytes to read. The buffer size must be >= u32Sz
 *  @return     @ref M2M_SUCCESS in case of success and @ref M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_read_block(uint32_t u32Addr, uint8_t *puBuf, uint32_t u32Sz);

/**
 *  @fn         nm_write_block
 *  @brief      Write block of data
 *  @param[in]  u32Addr
 *                  Start address
 *  @param[in]  puBuf
 *                  Pointer to the buffer holding the data to be written
 *  @param[in]  u32Sz
 *                  Number of bytes to write. The buffer size must be >= u32Sz
 *  @return     @ref M2M_SUCCESS in case of success and @ref M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_write_block(uint32_t u32Addr, uint8_t *puBuf, uint32_t u32Sz);

#ifdef __cplusplus
}
#endif

#endif /* _NMBUS_H_ */
