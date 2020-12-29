/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\wifi_drv_winc3400\bsp\include\nm_bsp_samd21_app copy.h/
 * Project: OQ2                                                                                    /
 * Created Date: Monday, December 28th 2020, 9:20:29 am                                            /
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

#ifndef _NM_BSP_STM32H7XX_APP_H_
#define _NM_BSP_STM32H7XX_APP_H_

#include "nm_bsp.h"
#include "nm_common.h"


#define M2M_PRINTX(x)  nm_bsp_uart_send((const uint8_t *)x,sizeof(x))
/**
*Extern global variables
*
*/
extern uint32_t gu32Jiffies20ms;
extern uint32_t gu32Jiffies1ms;
#ifdef __cplusplus
extern "C"{
#endif

/*
*
*/
#define SW1		NBIT0
#define SW2		NBIT1
/**/

#define NM_BSP_PERM_FIRMWARE_SIZE	(1024UL*256)	/* Permanent storage size available for the firmware */

#define TICK_RES							20		/*!< Tick resolution in milliseconds*/
#define TICK_RES_SLEEP						20		/*it must be equal tick or higher*/

#define NM_BSP_TIME_MSEC			(gu32Jiffies20ms * TICK_RES)


/**
*
*Callback functions
*/
typedef void (*tpfNmBspBtnPress)(uint8_t u8Btn, uint8_t u8Type);
typedef void (*tpfNmBspTimerCb)(void);

/**
*	@fn		nm_bsp_init
*	@brief	Initialize BSP
*	@return	0 in case of success and -1 in case of failure
*	@version	1.0
*/
int8_t nm_bsp_app_init(void);

/*
*	@fn			nm_bsp_app_configurable_timer_init
*	@brief		Initialize the Configurable Timer
*	@version	1.0
*/
void nm_bsp_app_configurable_timer_init(uint32_t u32Period);

/**
*	@fn		nm_bsp_deinit
*	@brief	De-iInitialize BSP
*	@return	0 in case of success and -1 in case of failure
*	@version	1.0
*/
int8_t nm_bsp_app_deinit(void);

/**
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
*	@version	1.0
*/
void nm_bsp_uart_send(const uint8_t *pu8Buf, uint16_t u16Sz);
/**
*	@fn		nm_bsp_btn_init
*	@brief	Initialize buttons driver
*	@version	1.0
*/
void nm_bsp_btn_init(tpfNmBspBtnPress pfBtnCb);
#ifdef _STATIC_PS_
/**
*	@fn		nm_bsp_register_wake_isr
*	@brief	REGISTER wake up timer
*	@version	1.0
*/
void nm_bsp_register_wake_isr(tpfNmBspIsr pfIsr,uint32_t u32MsPeriod);
/**
*	@fn		nm_bsp_wake_ctrl
*	@brief	control wake up timer
*	@version	1.0
*/
void nm_bsp_wake_ctrl(uint8_t en);

#endif
#if (defined _STATIC_PS_)||(defined _DYNAMIC_PS_)
/**
*	@fn		nm_bsp_enable_mcu_ps
*	@brief	Start POWER SAVE FOR MCU
*	@version	1.0
*/
void nm_bsp_enable_mcu_ps(void);
#endif

/**
*	@fn		nm_bsp_start_timer
*	@brief	Start 20ms timer
*	@version	1.0
*/
void nm_bsp_start_timer(tpfNmBspTimerCb pfCb, uint32_t u32Period);

/*
*	@fn			nm_bsp_start_1ms_timer
*	@brief		Start 1ms timer
*	@version	1.0
*/
void nm_bsp_start_1ms_timer(tpfNmBspTimerCb pfCb);

/*
*	@fn			nm_bsp_start_configurable_timer
*	@brief		Start configurable timer
*	@version	1.0
*/
void nm_bsp_start_configurable_timer(tpfNmBspTimerCb pfCb);

/**
*	@fn		nm_bsp_stop_timer
*	@brief	Stop 20ms timer
*	@version	1.0
*/
void nm_bsp_stop_timer(void);

/*
*	@fn			nm_bsp_stop_1ms_timer
*	@brief		Stop 1ms timer
*	@version	1.0
*/
void nm_bsp_stop_1ms_timer(void);

/*
*	@fn			nm_bsp_stop_configurable_timer
*	@brief		Stop configurable timer
*	@version	1.0
*/
void nm_bsp_stop_configurable_timer(void);


#ifdef __cplusplus
}
#endif
#endif /* _NM_BSP_SAMD21_H_ */
