/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\wifi_drv_winc3400\bus_wrapper\include\nm_bus_wrapper.h/
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
 * \brief This module contains WINC3400 bus wrapper APIs declarations.
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

#ifndef _NM_BUS_WRAPPER_H_
#define _NM_BUS_WRAPPER_H_

#include "nm_common.h"

/**
    BUS Type
**/
#define  NM_BUS_TYPE_I2C        ((uint8_t)0)
#define  NM_BUS_TYPE_SPI        ((uint8_t)1)
#define  NM_BUS_TYPE_UART       ((uint8_t)2)
/**
    IOCTL commands
**/
#define NM_BUS_IOCTL_R          ((uint8_t)0)  /*!< Read only ==> I2C/UART. Parameter:tstrNmI2cDefault/tstrNmUartDefault */
#define NM_BUS_IOCTL_W          ((uint8_t)1)  /*!< Write only ==> I2C/UART. Parameter type tstrNmI2cDefault/tstrNmUartDefault*/
#define NM_BUS_IOCTL_W_SPECIAL  ((uint8_t)2)  /*!< Write two buffers within the same transaction
                                                (same start/stop conditions) ==> I2C only. Parameter:tstrNmI2cSpecial */
#define NM_BUS_IOCTL_RW         ((uint8_t)3)  /*!< Read/Write at the same time ==> SPI only. Parameter:tstrNmSpiRw */

#define NM_BUS_IOCTL_WR_RESTART ((uint8_t)4)              /*!< Write buffer then made restart condition then read ==> I2C only. parameter:tstrNmI2cSpecial */

/**
 *  @struct tstrNmBusCapabilities
 *  @brief  Structure holding bus capabilities information
 *  @sa     NM_BUS_TYPE_I2C, NM_BUS_TYPE_SPI
 */
typedef struct
{
    uint16_t  u16MaxTrxSz;    /*!< Maximum transfer size. Must be >= 16 bytes*/
} tstrNmBusCapabilities;

/**
 *  @struct tstrNmI2cDefault
 *  @brief  Structure holding I2C default operation parameters
 *  @sa     NM_BUS_IOCTL_R, NM_BUS_IOCTL_W
 */
typedef struct
{
    uint8_t u8SlaveAdr;
    uint8_t   *pu8Buf;    /*!< Operation buffer */
    uint16_t  u16Sz;      /*!< Operation size */
} tstrNmI2cDefault;

/**
 *  @struct tstrNmI2cSpecial
 *  @brief  Structure holding I2C special operation parameters
 *  @sa     NM_BUS_IOCTL_W_SPECIAL
 */
typedef struct
{
    uint8_t u8SlaveAdr;
    uint8_t   *pu8Buf1;   /*!< pointer to the 1st buffer */
    uint8_t   *pu8Buf2;   /*!< pointer to the 2nd buffer */
    uint16_t  u16Sz1;     /*!< 1st buffer size */
    uint16_t  u16Sz2;     /*!< 2nd buffer size */
} tstrNmI2cSpecial;

/**
 *  @struct tstrNmSpiRw
 *  @brief  Structure holding SPI R/W parameters
 *  @sa     NM_BUS_IOCTL_RW
 */
typedef struct
{
    uint8_t   *pu8InBuf;      /*!< pointer to input buffer.
                            Can be set to null and in this case zeros should be sent at MOSI */
    uint8_t   *pu8OutBuf;     /*!< pointer to output buffer.
                            Can be set to null and in this case data from MISO can be ignored  */
    uint16_t  u16Sz;          /*!< Transfere size */
} tstrNmSpiRw;


/**
 *  @struct tstrNmUartDefault
 *  @brief  Structure holding UART default operation parameters
 *  @sa     NM_BUS_IOCTL_R, NM_BUS_IOCTL_W
 */
typedef struct
{
    uint8_t   *pu8Buf;    /*!< Operation buffer */
    uint16_t  u16Sz;      /*!< Operation size */
} tstrNmUartDefault;
/*!< Bus capabilities. This structure must be declared at platform specific bus wrapper */
extern tstrNmBusCapabilities egstrNmBusCapabilities;


#ifdef __cplusplus
extern "C" {
#endif

#if (defined XDMAC_SPI) && ((defined __SAME70Q21__) || (defined __SAME70Q21B__))
extern int8_t spi_xdmac_init();
#endif

/**
 *  @fn     nm_bus_init
 *  @brief  Initialize the bus wrapper
 *  @param [in] req_com_port
 *                  Used to choose com port assigned to device like edbg
 *  @param [in] req_serial_number
 *                  Used to choose peripheral, eg if 2 aardvark's are connected to pc.
 *                  Pass 0 to select first/only or if using UART.
 *  @return ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_bus_init(uint8_t *req_com_port, uint32_t req_serial_number);

/**
 *  @fn     nm_bus_ioctl
 *  @brief  send/receive from the bus
 *  @param [in] u8Cmd
 *                  IOCTL command for the operation
 *  @param [in] pvParameter
 *                  Arbitrary parameter depending on IOCTL
 *  @return ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
 *  @note   For SPI only, it's important to be able to send/receive at the same time
 */
int8_t nm_bus_ioctl(uint8_t u8Cmd, void* pvParameter);

/**
 *  @fn     nm_bus_deinit
 *  @brief  De-initialize the bus wrapper
 *  @return ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_bus_deinit(void);

/**
 *  @fn         nm_bus_reinit
 *  @brief      re-initialize the bus wrapper
 *  @param [in] void *config
 *                  re-init configuration data
 *  @return     ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
 */
int8_t nm_bus_reinit(void *);

/**
 *  @fn         nm_bus_get_chip_type
 *  @brief      get chip type
 *  @return     ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
 */

#ifdef CONF_WINC_USE_UART
uint8_t nm_bus_get_chip_type(void);
uint8_t nm_bus_get_sb_type(void);
#endif

/**
 *  @fn         spi_rw
 *  @brief      Process SPI Read/Write operation
 *  @param      pu8Mosi TX Data buffer
 *  @param      pu8Miso RX Data buffer
 *  @param      u16Sz Transfer length
 *  @return     ZERO in case of success and M2M_ERR_BUS_FAIL in case of failure
 */
#ifdef CONF_WINC_USE_SPI
int8_t nm_spi_rw(uint8_t* pu8Mosi, uint8_t* pu8Miso, uint16_t u16Sz);
#endif

#ifdef __cplusplus
}
#endif

#endif  /*_NM_BUS_WRAPPER_H_*/
