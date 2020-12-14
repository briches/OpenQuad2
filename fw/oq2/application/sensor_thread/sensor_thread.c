/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\sensor_thread\sensor_thread.c       /
 * Project: OQ2                                                                                    /
 * Created Date: Saturday, December 12th 2020, 7:23:57 am                                          /
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

#include "sensor_thread.h"
#include <string.h>
#include <stdio.h>
#include "lsm9ds1_reg.h"
#include "lps22hh_reg.h"
#include "debug_log.h"
#include "cmsis_os.h"
#include "mems_platform.h"
#include "main.h"
#include "kinematics.h"

#define debug_error(fmt, ...)           debug_error(SENSOR_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(SENSOR_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(SENSOR_MODULE_ID, fmt, ##__VA_ARGS__)


#define LSM9DS1_BUS hi2c2
#define LPS22HH_BUS hspi2

typedef union {
    int16_t i16bit[3];
    uint8_t u8bit[6];
} axis3bit16_t;

#define BOOT_TIME_LSM9DS1 20  // ms
#define BOOT_TIME_LPS22HH 5   // ms

#define IMU_FIFO_THRESHOLD 16

static volatile uint32_t m_int1_count = 0;
static volatile uint32_t m_int2_count = 0;
static bool m_sensors_initialized = false;

/* IMU variables ---------------------------------------------------------*/
static sensbus_t mag_bus =
{
    &LSM9DS1_BUS,
    LSM9DS1_MAG_I2C_ADD_H,
    0,
    0
};

static sensbus_t imu_bus =
{
    &LSM9DS1_BUS,
    LSM9DS1_IMU_I2C_ADD_H,
    0,
    0
};

static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t data_raw_magnetic_field;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI_lsm;
static lsm9ds1_status_t lsm9ds1;
static uint8_t rst_lsm;

/* Pressure Sensor variables ---------------------------------------------------------*/
static uint32_t data_raw_pressure;
static int16_t data_raw_temperature;
static float pressure_hPa;
static float temperature_degC;
static uint8_t whoamI_lps, rst_lps;
static lps22hh_reg_t lps22hh;

/* Sensor context variables ---------------------------------------------------------*/
stmdev_ctx_t dev_ctx_imu;
stmdev_ctx_t dev_ctx_mag;
stmdev_ctx_t dev_ctx_lps;


/**
 * @brief Initialize the LSM9DS1 9DoF inertial sensor
 * 
 */
static void lsm9ds1_init()
{
    debug_printf("LSM9DS1 Init");

    /* Check device ID */
    lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI_lsm);

    if (whoamI_lsm.imu != LSM9DS1_IMU_ID || whoamI_lsm.mag != LSM9DS1_MAG_ID)
    {
        debug_error("Sensor not identified!");
        return;
    }

    /* Restore default configuration */
    lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

    do
    {
        lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst_lsm);
    } while (rst_lsm);

    /* Enable Block Data Update */
    lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

    /* Set full scale */
    lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
    lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
    lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);

    /* Configure filtering chain - See datasheet for filtering chain details */
    /* Accelerometer filtering chain */
    lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
    lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_9);
    lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);

    /* Gyroscope filtering chain */
    lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
    lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
    lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);

    /* Set Output Data Rate / Power mode */
    lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_14Hz9);
    lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

    /* Set Accelerometer data ready interrupt on INT 1_A/G pin */
    lsm9ds1_pin_int1_route_t int1 = {0};
    int1.int1_drdy_xl = PROPERTY_ENABLE;
    lsm9ds1_pin_int1_route_set(&dev_ctx_imu, int1);

    lsm9ds1_pin_int2_route_t int2 = {0};
    int2.int2_fth = PROPERTY_ENABLE;
    lsm9ds1_pin_int2_route_set(&dev_ctx_imu, int2);

    // Configure FIFO as continuous stream mode, watermark threshold of 16 samples
    lsm9ds1_fifo_mode_set(&dev_ctx_imu, LSM9DS1_STREAM_TO_FIFO_MODE);
    lsm9ds1_fifo_watermark_set(&dev_ctx_imu, IMU_FIFO_THRESHOLD);
}

/**
 * @brief Initialize the LPS22HH pressure sensor
 * 
 */
static void lps22hh_init()
{
    debug_printf("LPS22HH Init");
    /* Check device ID */
    whoamI_lps = 0;
    lps22hh_device_id_get(&dev_ctx_lps, &whoamI_lps);

    if (whoamI_lps != LPS22HH_ID)
        debug_error("LPS22HH id not matched! Got 0x%02X", whoamI_lps);

    /* Restore default configuration */
    lps22hh_reset_set(&dev_ctx_lps, PROPERTY_ENABLE);

    do {
        lps22hh_reset_get(&dev_ctx_lps, &rst_lps);
    } while (rst_lps);

    /* Enable Block Data Update */
    lps22hh_block_data_update_set(&dev_ctx_lps, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    lps22hh_data_rate_set(&dev_ctx_lps, LPS22HH_10_Hz_LOW_NOISE);
}

/**
 * @brief Read IMU GYRO and ACCEL data. 
 * 
 * Can be called from the task or from the ISR
 */
static void read_imu_data()
{
    /* Read device status register */
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &lsm9ds1);

    if (lsm9ds1.status_imu.xlda && lsm9ds1.status_imu.gda)
    {
        /* Read imu data */
        memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
        memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));

        lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration.u8bit);
        lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate.u8bit);

        acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]);
        acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]);
        acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]);

        angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
        angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
        angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

        kinematics_update_accel_gyro(acceleration_mg, angular_rate_mdps);

        // debug_printf("IMU - [mg]:%3.1f\t%3.1f\t%3.1f\t [mdps]:%3.1f\t%3.1f\t%3.1f",
        // acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
        // angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
    }
}

static void read_fifo_info()
{
    uint8_t val;

    lsm9ds1_fifo_data_level_get(&dev_ctx_imu, &val);

    debug_printf("FIFO level %u", val);
}

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
void sensor_thread_start(void* argument)
{
    /* Initialize inertial sensors (IMU) driver interface */
    dev_ctx_imu.write_reg = platform_write_imu;
    dev_ctx_imu.read_reg = platform_read_imu;
    dev_ctx_imu.handle = (void*)&imu_bus;

    /* Initialize magnetic sensors driver interface */
    dev_ctx_mag.write_reg = platform_write_mag;
    dev_ctx_mag.read_reg = platform_read_mag;
    dev_ctx_mag.handle = (void*)&mag_bus;

    /* Initialize pressure sensor driver interface */
    dev_ctx_lps.write_reg = platform_write_baro;
    dev_ctx_lps.read_reg = platform_read_baro;
    dev_ctx_lps.handle = (void*)&LPS22HH_BUS;

    /* Wait sensor boot time */
    platform_delay(BOOT_TIME_LSM9DS1);

    lsm9ds1_init();

    lps22hh_init();

    m_sensors_initialized = true;

    for (;;)
    {
        TickType_t now = HAL_GetTick();
#if 0
        lps22hh_read_reg(&dev_ctx_lps, LPS22HH_STATUS, (uint8_t*)&lps22hh, 1);

        if (lps22hh.status.p_da)
        {
            memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
            lps22hh_pressure_raw_get(&dev_ctx_lps, &data_raw_pressure);
            pressure_hPa = lps22hh_from_lsb_to_hpa(data_raw_pressure);

            debug_printf("pressure [hPa]:%6.1f", pressure_hPa);
        }

        if (lps22hh.status.t_da)
        {
            memset(&data_raw_temperature, 0x00, sizeof(int16_t));
            lps22hh_temperature_raw_get(&dev_ctx_lps, &data_raw_temperature);
            temperature_degC = lps22hh_from_lsb_to_celsius(data_raw_temperature);
            debug_printf("temperature [degC]:%6.1f", temperature_degC);
        }
#endif
        if(m_int1_count)
        {
            // read_imu_data();
            debug_printf("Handled int1 from sensor task.");
        }

        if(m_int2_count)
        {
            debug_printf("Handled int2 from sensor task.");
            read_fifo_info();
        }


#if 0
        if (lsm9ds1.status_mag.zyxda)
        {
            /* Read magnetometer data */
            memset(data_raw_magnetic_field.u8bit, 0x00, 3 * sizeof(int16_t));
            lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field.u8bit);

            magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[0]);
            magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[1]);
            magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[2]);

            debug_printf("MAG - [mG]:%4.2f\t%4.2f\t%4.2f",
                magnetic_field_mgauss[0], magnetic_field_mgauss[1],
                magnetic_field_mgauss[2]);
        }
#endif 

        m_int1_count = 0;
        m_int2_count = 0;

        // debug_printf("T = %u", HAL_GetTick() - now);

        // HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        // HAL_Delay(10);
        // HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        // HAL_Delay(90);

        // HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        // HAL_Delay(10);
        // HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        // HAL_Delay(90);

        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
        osDelay(10);
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
        osDelay(10);

        osDelay(10);
    }
}


/**
 * @brief Callback function to be called when the HAL event HAL_GPIO_EXTI_Callback
 * is executed, with the pin IMU_AG_INT1_Pin
 * 
 * INT1 configured as the accel interrupt
 * 
 */
void sensors_imu_int1_callback()
{
    // If we haven't done initialization yet, we'll have to handle this in the task. 
    if(m_sensors_initialized == false)
    {
        m_int1_count += 1;
    }
    m_int1_count += 1;

    // Enter a critical region and get the latest data
    
    // UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    // read_imu_data();

    // taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

/**
 * @brief Callback function to be called when the HAL event HAL_GPIO_EXTI_Callback
 * is executed, with the pin IMU_AG_INT2_Pin
 * 
 * INT2 configured as the Fifo threshold interrupt
 * 
 */
void sensors_imu_int2_callback()
{
    // If we haven't done initialization yet, we'll have to handle this in the task. 
    if(m_sensors_initialized == false)
    {
        m_int2_count += 1;
        return;
    }

    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    read_fifo_info();

    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}