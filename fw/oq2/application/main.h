/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\main.h                              /
 * Project: OQ2                                                                                    /
 * Created Date: Wednesday, December 2nd 2020, 7:44:42 pm                                          /
 * Author: Brandon Riches                                                                          /
 * Email: richesbc@gmail.com                                                                       /
 * -----                                                                                           /
 * Last Modified: Wed Feb 03 2021                                                                  /
 * Modified By: Brandon Riches                                                                     /
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

void MX_CRC_Init(void);
void MX_CRYP_Init(void);
void MX_HASH_Init(void);
void MX_I2C1_Init(void);
void MX_I2C2_Init(void);
void MX_OCTOSPI1_Init(void);
void MX_RNG_Init(void);
void MX_SDMMC1_SD_Init(void);
void MX_SPI1_Init(void);
void MX_SPI2_Init(void);
void MX_LPTIM1_Init(void);
void MX_TIM1_Init(void);
void MX_TIM24_Init(void);
void MX_UART4_Init(void);
void MX_UART5_Init(void);
void MX_UART7_Init(void);
void MX_UART8_Init(void);
void MX_UART9_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_ADC3_Init(void);

typedef struct GPIO_PIN
{
    uint16_t pin;
    GPIO_TypeDef * port;
} GPIO_pin_t;


/* Private defines -----------------------------------------------------------*/
#define FLASH_NRESET_Pin GPIO_PIN_3
#define FLASH_NRESET_GPIO_Port GPIOE
#define IMU_AG_DEN_Pin GPIO_PIN_4
#define IMU_AG_DEN_GPIO_Port GPIOE
#define IMU_SDO_AG_Pin GPIO_PIN_5
#define IMU_SDO_AG_GPIO_Port GPIOE
#define IMU_SDO_MAG_Pin GPIO_PIN_6
#define IMU_SDO_MAG_GPIO_Port GPIOE
#define EVENT_IN_Pin GPIO_PIN_0
#define EVENT_IN_GPIO_Port GPIOC
#define VBAT_SENSE_Pin GPIO_PIN_3
#define VBAT_SENSE_GPIO_Port GPIOC
#define WIFI_SER_CFG_Pin GPIO_PIN_5
#define WIFI_SER_CFG_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_0
#define LED_B_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_1
#define LED_G_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_2
#define LED_R_GPIO_Port GPIOB
#define BLDC_BOOTSEL1_Pin GPIO_PIN_10
#define BLDC_BOOTSEL1_GPIO_Port GPIOD
#define BLDC_BOOTSEL2_Pin GPIO_PIN_11
#define BLDC_BOOTSEL2_GPIO_Port GPIOD
#define BLDC_BOOTSEL3_Pin GPIO_PIN_12
#define BLDC_BOOTSEL3_GPIO_Port GPIOD
#define BLDC_BOOTSEL4_Pin GPIO_PIN_13
#define BLDC_BOOTSEL4_GPIO_Port GPIOD
#define WIFI_CHIP_EN_Pin GPIO_PIN_6
#define WIFI_CHIP_EN_GPIO_Port GPIOC
#define WIFI_RESETN_Pin GPIO_PIN_7
#define WIFI_RESETN_GPIO_Port GPIOC
#define BLDC_ARM4_Pin GPIO_PIN_8
#define BLDC_ARM4_GPIO_Port GPIOA
#define BLDC_ARM3_Pin GPIO_PIN_10
#define BLDC_ARM3_GPIO_Port GPIOA
#define BLDC_ARM2_Pin GPIO_PIN_15
#define BLDC_ARM2_GPIO_Port GPIOA
#define BLDC_ARM1_Pin GPIO_PIN_0
#define BLDC_ARM1_GPIO_Port GPIOD
#define BLDC_nRESET1_Pin GPIO_PIN_1
#define BLDC_nRESET1_GPIO_Port GPIOD
#define BLDC_nRESET2_Pin GPIO_PIN_3
#define BLDC_nRESET2_GPIO_Port GPIOD
#define BLDC_nRESET3_Pin GPIO_PIN_4
#define BLDC_nRESET3_GPIO_Port GPIOD
#define BLDC_nRESET4_Pin GPIO_PIN_5
#define BLDC_nRESET4_GPIO_Port GPIOD
#define BARO_INT_Pin GPIO_PIN_6
#define BARO_INT_GPIO_Port GPIOD
#define BARO_INT_EXTI_IRQn EXTI9_5_IRQn
#define IMU_MAG_DRDY_Pin GPIO_PIN_7
#define IMU_MAG_DRDY_GPIO_Port GPIOD
#define IMU_MAG_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define IMU_AG_INT2_Pin GPIO_PIN_9
#define IMU_AG_INT2_GPIO_Port GPIOG
#define IMU_AG_INT2_EXTI_IRQn EXTI9_5_IRQn
#define IMU_AG_INT1_Pin GPIO_PIN_10
#define IMU_AG_INT1_GPIO_Port GPIOG
#define IMU_AG_INT1_EXTI_IRQn EXTI15_10_IRQn
#define IMU_MAG_INT_Pin GPIO_PIN_11
#define IMU_MAG_INT_GPIO_Port GPIOG
#define IMU_MAG_INT_EXTI_IRQn EXTI15_10_IRQn
#define GPS_TIME_Pin GPIO_PIN_12
#define GPS_TIME_GPIO_Port GPIOG
#define GPS_TIME_EXTI_IRQn EXTI15_10_IRQn
#define GPS_EXTINT_Pin GPIO_PIN_13
#define GPS_EXTINT_GPIO_Port GPIOG
#define GPS_EXTINT_EXTI_IRQn EXTI15_10_IRQn
#define WIFI_IRQ_Pin GPIO_PIN_14
#define WIFI_IRQ_GPIO_Port GPIOG
#define WIFI_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define GPS_RESETN_Pin GPIO_PIN_5
#define GPS_RESETN_GPIO_Port GPIOB
#define IMU_CS_AG_Pin GPIO_PIN_8
#define IMU_CS_AG_GPIO_Port GPIOB
#define IMU_CS_MAG_Pin GPIO_PIN_9
#define IMU_CS_MAG_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
