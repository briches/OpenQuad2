/**
 *
 * \file
 *
 * \brief This module contains SAM4S BSP APIs implementation.
 *
 * Copyright (c) 2018-2019 Microchip Technology Inc. and its subsidiaries.
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

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "asf.h"
#include "conf_winc.h"
#include "delay.h"


static tpfNmBspIsr gpfIsr;


static void chip_isr(uint32_t id, uint32_t mask)
{
	if ((id == CONF_WINC_SPI_INT_PIO_ID) && (mask == CONF_WINC_SPI_INT_MASK)) {
        if(gpfIsr) {
            gpfIsr();
        }
    }
}

static void init_chip_pins(void)
{
#ifdef __SAM4SD32C__
	pio_configure_pin(CONF_WINC_PIN_RESET, PIO_TYPE_PIO_OUTPUT_0|PIO_PULLUP);
	pio_configure_pin(CONF_WINC_PIN_CHIP_ENABLE, PIO_TYPE_PIO_OUTPUT_0|PIO_PULLUP);
	pio_configure_pin(CONF_WINC_PIN_WAKE, PIO_TYPE_PIO_OUTPUT_0|PIO_PULLUP);
	pio_configure_pin(CONF_WINC_SPI_CS_GPIO, PIO_DEFAULT|PIO_PULLUP);
	pio_set_pin_high(CONF_WINC_SPI_CS_GPIO);
#else
	pio_configure_pin(CONF_WINC_PIN_RESET, PIO_TYPE_PIO_OUTPUT_0);
	pio_configure_pin(CONF_WINC_PIN_CHIP_ENABLE, PIO_TYPE_PIO_OUTPUT_0);
	pio_configure_pin(CONF_WINC_PIN_WAKE, PIO_TYPE_PIO_OUTPUT_0);
#endif
}

/*
 *  @fn         nm_bsp_init
 *  @brief      Initialize BSP
 *  @return     0 in case of success and -1 in case of failure
 */
int8_t nm_bsp_init(void)
{
    gpfIsr = NULL;

    init_chip_pins();
#if (CONF_WINC_USE_SPI == 0)
    nm_bsp_reset();
#endif
    return 0;
}

int8_t nm_bsp_deinit(void)
{
	pio_set_pin_low(CONF_WINC_PIN_CHIP_ENABLE);
	pio_set_pin_low(CONF_WINC_PIN_RESET);
	return M2M_SUCCESS;
}

void nm_bsp_reset(void)
{
	pio_set_pin_high(CONF_WINC_PIN_CHIP_ENABLE);
	nm_bsp_sleep(5);
	pio_set_pin_high(CONF_WINC_PIN_RESET);
}

/*
 *  @fn         nm_bsp_sleep
 *  @brief      Sleep in units of mSec
 *  @param[IN]  u32TimeMsec
 *                  Time in milliseconds
 */
void nm_bsp_sleep(uint32_t u32TimeMsec)
{
    while(u32TimeMsec--) {
        delay_ms(1);
    }
}

/*
 *  @fn         nm_bsp_register_isr
 *  @brief      Register interrupt service routine
 *  @param[IN]  pfIsr
 *                  Pointer to ISR handler
 *  @sa         tpfNmBspIsr
 */
void nm_bsp_register_isr(tpfNmBspIsr pfIsr)
{
    gpfIsr = pfIsr;

	/* Configure PGIO pin for interrupt from SPI slave, used when slave has data to send. */
	sysclk_enable_peripheral_clock(CONF_WINC_SPI_INT_PIO_ID);
	pio_configure_pin(CONF_WINC_SPI_INT_PIN, PIO_TYPE_PIO_INPUT);
	pio_pull_up(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_MASK, PIO_PULLUP);
//	pio_set_debounce_filter(CONF_WIFI_NC_SPI_INT_PIO, CONF_WIFI_NC_SPI_INT_MASK, 10);
	pio_handler_set_pin(CONF_WINC_SPI_INT_PIN, PIO_IT_LOW_LEVEL, chip_isr);
	pio_enable_interrupt(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_MASK);
	pio_handler_set_priority(CONF_WINC_SPI_INT_PIO, (IRQn_Type)CONF_WINC_SPI_INT_PIO_ID,
			CONF_WINC_SPI_INT_PRIORITY);
}
/*
 *  @fn         nm_bsp_interrupt_ctrl
 *  @brief      Enable/Disable interrupts
 *  @param[IN]  u8Enable
 *                  '0' disable interrupts. '1' enable interrupts
 */
void nm_bsp_interrupt_ctrl(uint8_t u8Enable)
{
	if (u8Enable) {
		pio_get_interrupt_status(CONF_WINC_SPI_INT_PIO);
		pio_enable_interrupt(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_MASK);
	}
	else {
		pio_disable_interrupt(CONF_WINC_SPI_INT_PIO, CONF_WINC_SPI_INT_MASK);
	}
}

