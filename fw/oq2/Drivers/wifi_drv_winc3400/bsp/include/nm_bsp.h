/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\wifi_drv_winc3400\bsp\include\nm_bsp.h  /
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
 * \brief This module contains WINC3400 BSP APIs declarations.
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

/** @defgroup nm_bsp BSP
    @brief
        Description of the BSP (<strong>B</strong>oard <strong>S</strong>upport <strong>P</strong>ackage) module.
    @{
        @defgroup   DataT       Data Types
        @defgroup   BSPDefine   Defines
        @defgroup   BSPAPI      Functions
        @brief
            Lists the available BSP (<strong>B</strong>oard <strong>S</strong>upport <strong>P</strong>ackage) APIs.
    @}
 */

/**@addtogroup BSPDefine
   @{
 */
#ifndef _NM_BSP_H_
#define _NM_BSP_H_

#include "nm_common.h"

#include <stdint.h>

#define NMI_API
/*!<
 *  Attribute used to define the memory section to map Functions in host memory.
 */

/*!<
 *  Used for code portability.
 */

#ifndef NULL
#define NULL ((void*)0)
#endif
/*!<
*   Void Pointer to '0' in case NULL is not defined.
*/

#define BSP_MIN(x,y) ((x)>(y)?(y):(x))
/*!<
 *    Computes the minimum value between \b x and \b y.
 */
/**@}*/     //BSPDefine

/**@addtogroup DataT
 * @{
 */
/*!
 * @typedef     void (*tpfNmBspIsr) (void);
 * @brief       Pointer to function.\n Used as a data type of ISR function registered by
 *              @ref nm_bsp_register_isr.
 * @return      None
 */
typedef void (*tpfNmBspIsr)(void);

#ifndef CORTUS_APP

#ifdef __cplusplus
extern "C" {
#endif

/**@addtogroup BSPAPI
 * @{
 */
/*!
 * @fn          int8_t nm_bsp_init(void);
 * @brief       This function is used to initialize the <strong>B</strong>oard <strong>S</strong>upport
 *              <strong>P</strong>ackage <strong>(BSP)</strong> in order to prepare the WINC before any
 *              WINC API calls.
 * @details     The nm_bsp_init function is the first function that should be called at the beginning of
 *              every application to initialize the BSP and the WINC board. Otherwise, the rest of the BSP
 *              function calls will return with failure. This function should also be called after the WINC
 *              has been switched off with a successful call to @ref nm_bsp_deinit in order to reinitialize
 *              the BSP before the Application can use any of the WINC APIs again. After the function
 *              initializes the WINC, a hard reset must be applied to start the WINC board by calling
 *              @ref nm_bsp_reset.
 *
 * @note        Implementation of this function is host dependent.
 * @warning     Omitting this function will lead to unavailability of host-chip communication.
 *
 * @see         nm_bsp_deinit, nm_bsp_reset
 * @return      The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
int8_t nm_bsp_init(void);

/*!
 * @fn          int8_t nm_bsp_deinit(void);
 * @brief       This function is used to de-initialize the BSP and turn off the WINC board.
 * @details     The nm_bsp_deinit is the last function that should be called after the application has
 *              finished and before the WINC is switched off. A call to this function will turn off the WINC
 *              board by setting CHIP_EN and RESET_N signals low. Every function call of @ref nm_bsp_init
 *              should be matched with a call to nm_bsp_deinit. Failure to do so may result in the WINC
 *              consuming higher power than expected, since it won't be properly de-initialized.
 * @pre         The BSP should be initialized through @ref nm_bsp_init first.
 * @note        Implementation of this function is host dependent.
 * @warning     Omitting this function may lead to unknown behavior in case of soft reset.
 * @see         nm_bsp_init
 * @return      The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.

 */
int8_t nm_bsp_deinit(void);

/*!
 * @fn          void nm_bsp_reset(void);
 * @brief       Performs a hardware reset to the WINC board.
 * @details     The nm_bsp_reset is used to perform a hard reset on the WINC board by setting CHIP_EN and
 *              RESET_N signals low, then after an appropriate delay this function puts CHIP_EN high then
 *              RESET_N high, for more details on the timing between signals please check the WINC data-sheet.
 *              After a successful call, the WINC board firmware will kick off to load and start the WINC
 *              firmware. This function should be called to reset the WINC firmware after the BSP is
 *              initialized and before the start of any communication with WINC board. Calling this
 *              function at any other time will result in losing the state and connections saved in the
 *              WINC board and starting again from the initial state. The host driver will need to be
 *              de-initialized before calling nm_bsp_reset and initialized again, which can be achieved by
 *              use of @ref m2m_wifi_init and @ref m2m_wifi_deinit.
 * @pre         Initialize the BSP first by calling @ref nm_bsp_init.
 * @note        Implementation of this function is host dependent and called by HIF layer.
 * @warning     Calling this function will drop any connection and lose the internal state saved on the WINC firmware.
 * @see         nm_bsp_init, m2m_wifi_init,  m2m_wifi_deinit
 * @return      None
 */
void nm_bsp_reset(void);

/*!
 * @fn          void nm_bsp_sleep(uint32_t u32TimeMsec);
 * @brief       Used to put the host to sleep for the specified duration (in milliseconds).
 *              Forcing the host to sleep for extended period may lead to host not being able to respond
 *              to WINC board events. It is important to be considerate while choosing the sleep period.
 * @param [in]  u32TimeMsec
 *                  Time unit in milliseconds.
 * @pre         Initialize @ref nm_bsp_init first.
 * @note        Implementation of this function is host dependent.
 * @warning     Maximum value must nor exceed 4294967295 milliseconds which is equal to 4294967.295 seconds.
 * @see         nm_bsp_init
 * @return      None
 */
void nm_bsp_sleep(uint32_t u32TimeMsec);

/*!
 * @fn          void nm_bsp_register_isr(tpfNmBspIsr pfIsr);
 * @brief       Register the host interface interrupt service routine.
 * @details     The WINC board uses the SPI interface to communicate with the host. This function registers
 *              the SPI interrupt to notify the host whenever there is an outstanding message from the WINC
 *              board. This function should be called during the initialization of the host interface. It is
 *              an internal driver function and shouldn't be called by the Application.
 * @param [in]  pfIsr
 *                  Pointer to ISR handler in the HIF layer.
 * @note        Implementation of this function is host dependent and called by HIF layer.
 * @warning     Make sure that ISR for IRQ pin for WINC is disabled by default in your implementation.
 * @see         tpfNmBspIsr
 * @return      None
 */
void nm_bsp_register_isr(tpfNmBspIsr pfIsr);

/*!
 * @fn          void nm_bsp_interrupt_ctrl(uint8_t u8Enable);
 * @brief       Enable/Disable interrupts from the WINC.
 * @details     This function can be used to enable/disable the WINC to host interrupts, depending on how
 *              the driver is implemented. It is an internal driver function and shouldn't be called by
 *              the application.
 * @param [in]  u8Enable
 *                  - '0' disable interrupts.
 *                  - '1' enable interrupts.
 * @pre         The interrupt must be registered using @ref nm_bsp_register_isr first.
 * @note        Implementation of this function is host dependent and called by HIF layer.
 * @see         tpfNmBspIsr, nm_bsp_register_isr
 * @return      None
 */
void nm_bsp_interrupt_ctrl(uint8_t u8Enable);


void winc3400_pin_isr();

/**@}*/     // BSPAPI

#ifdef __cplusplus
}
#endif

#endif

/**
 * @addtogroup BSPDefine
 * @{
 */
#ifdef _NM_BSP_BIG_END
/*! Switch endianness of 32bit word (In the case that Host is BE) */
#define NM_BSP_B_L_32(x) \
((((x) & 0x000000FF) << 24) + \
(((x) & 0x0000FF00) << 8)  + \
(((x) & 0x00FF0000) >> 8)   + \
(((x) & 0xFF000000) >> 24))

/*! Switch endianness of 16bit word (In the case that Host is BE) */
#define NM_BSP_B_L_16(x) \
((((x) & 0x00FF) << 8) + \
(((x)  & 0xFF00) >> 8))
#else
/*! Retain endianness of 32bit word (In the case that Host is LE) */
#define NM_BSP_B_L_32(x)  (x)
/*! Retain endianness of 16bit word (In the case that Host is LE) */
#define NM_BSP_B_L_16(x)  (x)
#endif
/**@}*/     //BSPDefine

#endif  /*_NM_BSP_H_*/