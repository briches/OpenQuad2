/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\stability_thread\stability_thread.c /
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

#include "stability.h"
#include <string.h>
#include <stdio.h>
#include "lsm9ds1_reg.h"
#include "lps22hh_reg.h"
#include "debug_log.h"
#include "cmsis_os.h"
#include "mems_platform.h"
#include "main.h"
#include "stability_config.h"
#include "motion_fx.h"
#include "arm_math.h"
#include "kinematics.h"
#include "timer.h"
#include "pid.h"
#include "flight_app.h"

#define debug_error(fmt, ...)           debug_error(STABILITY_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(STABILITY_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(STABILITY_MODULE_ID, fmt, ##__VA_ARGS__)

#define LSM9DS1_BUS hi2c2
#define LPS22HH_BUS hspi2

static MFX_knobs_t m_mfx_knobs;
static kinematics_ctx_t m_kinematics;
static pid_ctx_t m_pitch_pid;
static pid_limits_t m_pitch_pid_limits =
{
    .P_lim = SC_PITCH_PID_P_LIM,
    .I_lim = SC_PITCH_PID_I_LIM,
    .D_lim = SC_PITCH_PID_D_LIM,
};
static pid_params_t m_pitch_pid_params =
{
    .P = SC_PITCH_PID_P,
    .I = SC_PITCH_PID_I,
    .D = SC_PITCH_PID_D,
};

static pid_ctx_t m_roll_pid;
static pid_limits_t m_roll_pid_limits =
{
    .P_lim = SC_ROLL_PID_P_LIM,
    .I_lim = SC_ROLL_PID_I_LIM,
    .D_lim = SC_ROLL_PID_D_LIM,
};
static pid_params_t m_roll_pid_params =
{
    .P = SC_ROLL_PID_P,
    .I = SC_ROLL_PID_I,
    .D = SC_ROLL_PID_D,
};

static pid_ctx_t m_yaw_pid;
static pid_limits_t m_yaw_pid_limits =
{
    .P_lim = SC_YAW_PID_P_LIM,
    .I_lim = SC_YAW_PID_I_LIM,
    .D_lim = SC_YAW_PID_D_LIM,
};
static pid_params_t m_yaw_pid_params =
{
    .P = SC_YAW_PID_P,
    .I = SC_YAW_PID_I,
    .D = SC_YAW_PID_D,
};

typedef union {
    int16_t i16bit[3];
    uint8_t u8bit[6];
} axis3bit16_t;

#define BOOT_TIME_LSM9DS1 20  // ms
#define BOOT_TIME_LPS22HH 5   // ms

#define IMU_FIFO_THRESHOLD  9
#define IMU_FIFO_MAX_ITEMS  32

static volatile uint32_t m_int1_count = 0;
static volatile uint32_t m_int2_count = 0;
static volatile uint32_t m_drdy_count = 0;
static volatile uint32_t m_baro_int_count = 0;
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

char MotionFX_LoadMagCalFromNVM(unsigned short int dataSize, unsigned int* data)
{
    debug_printf("MFX lib wants to load cals from NVM.");
    debug_printf("size: %u", dataSize);
    debug_printf("data pointer: 0x%08X", data);

    return 1;
}

char MotionFX_SaveMagCalInNVM(unsigned short int dataSize, unsigned int* data)
{
    debug_printf("MFX lib wants to save cals to NVM.");
    debug_printf("size: %u", dataSize);
    debug_printf("data pointer: 0x%08X", data);

    return 1;
}

/**
 * @brief Initialize the LSM9DS1 9DoF inertial sensor
 *
 */
static void lsm9ds1_init()
{
    debug_printf("LSM9DS1 Init");

    /* Check device ID */
    int retry = SC_MEMS_DETECT_RETRY_MAX;

    do
    {
        lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI_lsm);

        if (whoamI_lsm.imu != LSM9DS1_IMU_ID || whoamI_lsm.mag != LSM9DS1_MAG_ID)
        {
            debug_error("Sensor not identified!");
        }
        else
        {
            break;
        }

    } while (--retry);

    if (whoamI_lsm.imu != LSM9DS1_IMU_ID || whoamI_lsm.mag != LSM9DS1_MAG_ID)
    {
        debug_error("Failed to init LSM9DS1");
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
    lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
    lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);

    /* Gyroscope filtering chain */
    // Ultralight LP is highest cutoff
    // Extreme HP is highest cutoff
    lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
    lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_ULTRA_LIGHT);
    lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_OUT);

    /* Set Output Data Rate / Power mode */
    lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_238Hz);
    lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

    lsm9ds1_ctrl_reg9_t ctrl9;
    // ctrl9.drdy_mask_bit = 1;
    lsm9ds1_write_reg(&dev_ctx_mag, LSM9DS1_CTRL_REG9, (uint8_t*)&ctrl9, sizeof(ctrl9));

#if defined(SENSOR_THREAD_IMU_USE_INDIVIDUAL)

    /* Set Accelerometer data ready interrupt on INT 1_A/G pin */
    lsm9ds1_pin_int1_route_t int1 = { 0 };
    // int1.int1_drdy_xl = PROPERTY_ENABLE;
    lsm9ds1_pin_int1_route_set(&dev_ctx_imu, int1);

    /* No interrupts pathed to INT2 pin */
    lsm9ds1_pin_int2_route_t int2 = { 0 };
    lsm9ds1_pin_int2_route_set(&dev_ctx_imu, int2);

    lsm9ds1_fifo_mode_set(&dev_ctx_imu, LSM9DS1_FIFO_OFF);

#elif defined(SENSOR_THREAD_IMU_USE_FIFO)

    /* Set Accelerometer data ready interrupt on INT 1_A/G pin */
    lsm9ds1_pin_int1_route_t int1 = { 0 };
    lsm9ds1_pin_int1_route_set(&dev_ctx_imu, int1);

    /* Set accelerometer fifo threshold interrupt on INT2_A/G pin */
    lsm9ds1_pin_int2_route_t int2 = { 0 };
    int2.int2_fth = PROPERTY_ENABLE;
    lsm9ds1_pin_int2_route_set(&dev_ctx_imu, int2);

    // Configure FIFO as continuous stream mode, watermark threshold of 16 samples
    lsm9ds1_fifo_mode_set(&dev_ctx_imu, LSM9DS1_STREAM_TO_FIFO_MODE);
    lsm9ds1_fifo_watermark_set(&dev_ctx_imu, IMU_FIFO_THRESHOLD);

#else
#error "check app config and define either SENSOR_THREAD_IMU_USE_INDIVIDUAL or SENSOR_THREAD_IMU_USE_FIFO"
#endif
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
    int retry = SC_MEMS_DETECT_RETRY_MAX;

    do
    {
        lps22hh_device_id_get(&dev_ctx_lps, &whoamI_lps);

        if (whoamI_lps != LPS22HH_ID)
            debug_error("LPS22HH id not matched! Got 0x%02X", whoamI_lps);
        else
            break;

    } while (--retry);

    /* Restore default configuration */
    lps22hh_reset_set(&dev_ctx_lps, PROPERTY_ENABLE);

    do {
        lps22hh_reset_get(&dev_ctx_lps, &rst_lps);
    } while (rst_lps);

    /* Enable Block Data Update */
    lps22hh_block_data_update_set(&dev_ctx_lps, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    lps22hh_data_rate_set(&dev_ctx_lps, LPS22HH_1_Hz_LOW_NOISE);

    // Enable interrupts on DRDY pin
    lps22hh_ctrl_reg3_t ctrl3;
    ctrl3.drdy = 1;
    lps22hh_pin_int_route_set(&dev_ctx_lps, &ctrl3);
}

/**
 * @brief Read IMU GYRO and ACCEL single data
 *
 * Can be called from the task or from the ISR
 */
static void read_imu_data()
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

    // debug_printf("IMU - [mg]:%3.1f\t%3.1f\t%3.1f\t [mdps]:%3.1f\t%3.1f\t%3.1f",
    // acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
    // angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
}

/**
 * @brief Read magnetometer 3 axis data
 *
 * Can be called from the task or from the ISR
 */
static void read_mag_data()
{
    /* Read magnetometer data */
    memset(data_raw_magnetic_field.u8bit, 0x00, 3 * sizeof(int16_t));
    lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field.u8bit);

    magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[0]);
    magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[1]);
    magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[2]);

    // debug_printf("MAG - [mG]:%4.2f\t%4.2f\t%4.2f",
    //     magnetic_field_mgauss[0], magnetic_field_mgauss[1],
    //     magnetic_field_mgauss[2]);
}

/**
 * @brief Read barometer temperature and pressure data.
 *
 * Can be called from the task or from the ISR
 */
static void read_baro_data()
{
    memset(&data_raw_pressure, 0x00, sizeof(uint32_t));
    memset(&data_raw_temperature, 0x00, sizeof(int16_t));

    lps22hh_pressure_raw_get(&dev_ctx_lps, &data_raw_pressure);
    pressure_hPa = lps22hh_from_lsb_to_hpa(data_raw_pressure);

    lps22hh_temperature_raw_get(&dev_ctx_lps, &data_raw_temperature);
    temperature_degC = lps22hh_from_lsb_to_celsius(data_raw_temperature);

    // debug_printf("pressure [hPa]:%6.1f, temperature [degC]:%6.1f", pressure_hPa, temperature_degC);

    // Convert pressure in hPA to meters elevation
    float elevation = (powf(SC_SEA_LEVEL_PRESS_HPA / pressure_hPa, 0.19022256) - 1) * (SC_AIR_TEMPERATURE + 273.15) * 153.84615;

    kinematics_new_elevation_data_callback(&m_kinematics, elevation);

    // debug_printf("Elevation [m]:%6.1f, temperature [degC]:%6.1f", elevation, temperature_degC);
}

/**
 * @brief Callback function to be called when the HAL event HAL_GPIO_EXTI_Callback
 * is executed, with the pin IMU_AG_INT1_Pin
 *
 * INT1 configured as the accel interrupt
 *
 */
void imu_int1_callback()
{
    // If we haven't done initialization yet, we'll have to handle this in the task. 
    if (m_sensors_initialized == false)
    {
        m_int1_count += 1;
        return;
    }

    // Enter a critical region and get the latest data
    // UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    // taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

/**
 * @brief Callback function to be called when the HAL event HAL_GPIO_EXTI_Callback
 * is executed, with the pin IMU_MAG_DRDY
 *
 */
void imu_mag_drdy_callback()
{
    if (m_sensors_initialized == false)
    {
        m_baro_int_count += 1;
        return;
    }

    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    read_mag_data();
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

/**
 * @brief Callback function to be called when the HAL event HAL_GPIO_EXTI_Callback
 * is executed, with the pin BARO_INT_Pin
 *
 */
void baro_int_callback()
{
    if (m_sensors_initialized == false)
    {
        m_drdy_count += 1;
        return;
    }

    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    read_baro_data();
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

/**
 * @brief Read IMU data, Update and Propagate MotionFX kalman library
 *
 * @retval time delta since last call in seconds
 */
float stability_thread_read_and_calculate()
{
    uint32_t TAG = osKernelGetTickCount();

    // Get the time delta since the last data ready
    static volatile float timestamp = 0;
    float dT = 0;
    float now = timer_get_elapsed();
    dT = (now - timestamp);
    timestamp = now;

    if (dT < 0)
    {
        debug_error("%u, WATCHOUT!", TAG);
        float test = timer_get_elapsed();
    }

    // Read new sensor data that is available.
    read_imu_data();

    MFX_input_t data_in;
    MFX_output_t data_out;

    // ACcelerations in G's
    data_in.acc[0] = acceleration_mg[0] / 1000.0;
    data_in.acc[1] = acceleration_mg[1] / 1000.0;
    data_in.acc[2] = acceleration_mg[2] / 1000.0;

    // Rate in dps
    data_in.gyro[0] = angular_rate_mdps[0] / 1000.0;
    data_in.gyro[1] = angular_rate_mdps[1] / 1000.0;
    data_in.gyro[2] = angular_rate_mdps[2] / 1000.0;

    // Mag in uT / 50
    data_in.mag[0] = magnetic_field_mgauss[0] / (50.0f * 10.0f);
    data_in.mag[1] = magnetic_field_mgauss[1] / (50.0f * 10.0f);
    data_in.mag[2] = magnetic_field_mgauss[2] / (50.0f * 10.0f);

#if (SC_OPERATING_MODE == SC_OP_MODE_MAG_CAL)
    static MFX_MagCal_quality_t cal_quality = MFX_MAGCALUNKNOWN;

    MFX_MagCal_input_t mc_input_data;
    MFX_MagCal_output_t mc_output_data;
    mc_input_data.mag[0] = data_in.mag[0];
    mc_input_data.mag[1] = data_in.mag[1];
    mc_input_data.mag[2] = data_in.mag[2];
    mc_input_data.time_stamp = now;

    MotionFX_MagCal_run(&mc_input_data);

    MotionFX_MagCal_getParams(&mc_output_data);

    cal_quality = mc_output_data.cal_quality;

    // debug_printf("inx, %3.2f, iny, %3.2f, inz, %3.2f, Quality: %u, x, %3.2f, y, %3.2f, z, %3.2f", 
    //                 data_in.mag[0],
    //                 data_in.mag[1],
    //                 data_in.mag[2],
    //                 mc_output_data.cal_quality,
    //                 mc_output_data.hi_bias[0],
    //                 mc_output_data.hi_bias[1],
    //                 mc_output_data.hi_bias[2]);

    if (cal_quality == MFX_MAGCALGOOD)
    {
        debug_printf("Cald");
        MotionFX_MagCal_init(0, false);

        data_in.mag[0] -= mc_output_data.hi_bias[0];
        data_in.mag[1] -= mc_output_data.hi_bias[1];
        data_in.mag[2] -= mc_output_data.hi_bias[2];
    }
#endif

    MotionFX_propagate(&data_out, &data_in, &dT);
    MotionFX_update(&data_out, &data_in, &dT, NULL);

#if SC_ENABLE_6AXIS_MFX_LIB == APP_CONFIG_ENABLED
    float* p_angles = &data_out.rotation_6X[0];
#else
    float* p_angles = &data_out.rotation_9X[0];
#endif

    kinematics_new_motionfx_data_callback(&m_kinematics, &data_out);

    return dT;
}

/**
 * @brief Initialization prior to freertos being enabled
 *
 */
void stability_thread_pre_init()
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

    // Init kinematics context
    kinematics_initialize(&m_kinematics, SC_ENABLE_9AXIS_MFX_LIB);

    // Initialize PID controllers
    pid_initialize_ctx(&m_pitch_pid, &m_pitch_pid_params, &m_pitch_pid_limits);
    pid_initialize_ctx(&m_roll_pid, &m_roll_pid_params, &m_roll_pid_limits);
    pid_initialize_ctx(&m_yaw_pid, &m_yaw_pid_params, &m_yaw_pid_limits);
    m_pitch_pid.setpoint = 0;
    m_roll_pid.setpoint = 0;
    m_yaw_pid.setpoint = 0;

    /* Wait sensor boot time */
    platform_delay(BOOT_TIME_LSM9DS1);

    lsm9ds1_init();

    lps22hh_init();

    char buf[50];

    memset(buf, 0x00, sizeof(buf));

    // See motionfx lib docs for example code
    MotionFX_initialize();
    MotionFX_GetLibVersion(buf);
    MotionFX_getKnobs(&m_mfx_knobs);

    debug_printf("MFX lib ver: %s", buf);

    // Mag is rotated opposite to the accel and gyro for whatever reason
    m_mfx_knobs.mag_orientation[0] = 'n';
    m_mfx_knobs.mag_orientation[1] = 'w';
    m_mfx_knobs.mag_orientation[2] = 'u';

    // Gyro x is normal
    m_mfx_knobs.gyro_orientation[0] = 's';
    m_mfx_knobs.gyro_orientation[1] = 'w';
    m_mfx_knobs.gyro_orientation[2] = 'u';

    // Accel x is normal
    m_mfx_knobs.acc_orientation[0] = 's';
    m_mfx_knobs.acc_orientation[1] = 'w';
    m_mfx_knobs.acc_orientation[2] = 'u';

    m_mfx_knobs.ATime = SC_MFX_ATIME;
    m_mfx_knobs.start_automatic_gbias_calculation = 0;
    m_mfx_knobs.LMode = 2;
    m_mfx_knobs.modx = 1;

    MotionFX_setKnobs(&m_mfx_knobs);

    debug_printf("Accel ATime: %3.3f", m_mfx_knobs.ATime);
    debug_printf("Mag MTime: %3.3f", m_mfx_knobs.MTime);
    debug_printf("Dynamica accel FrTime: %3.3f", m_mfx_knobs.FrTime);
    debug_printf("Gyro bias LMode: %u", m_mfx_knobs.LMode);
    debug_printf("gbias_mag_th_sc_6X: %3.3f", m_mfx_knobs.gbias_mag_th_sc_6X);
    debug_printf("gbias_acc_th_sc_6X: %3.3f", m_mfx_knobs.gbias_acc_th_sc_6X);
    debug_printf("gbias_gyro_th_sc_6X: %3.3f", m_mfx_knobs.gbias_gyro_th_sc_6X);
    debug_printf("gbias_mag_th_sc_9X: %3.3f", m_mfx_knobs.gbias_mag_th_sc_9X);
    debug_printf("gbias_acc_th_sc_9X: %3.3f", m_mfx_knobs.gbias_acc_th_sc_9X);
    debug_printf("gbias_gyro_th_sc_9X: %3.3f", m_mfx_knobs.gbias_gyro_th_sc_9X);
    debug_printf("decimation modx: %u", m_mfx_knobs.modx);
    debug_printf("acc orientation: [%c, %c, %c, %c]", m_mfx_knobs.acc_orientation[0],
        m_mfx_knobs.acc_orientation[1],
        m_mfx_knobs.acc_orientation[2],
        m_mfx_knobs.acc_orientation[3]);
    debug_printf("gyro orientation: [%c, %c, %c, %c]", m_mfx_knobs.gyro_orientation[0],
        m_mfx_knobs.gyro_orientation[1],
        m_mfx_knobs.gyro_orientation[2],
        m_mfx_knobs.gyro_orientation[3]);
    debug_printf("mag orientation: [%c, %c, %c, %c]", m_mfx_knobs.mag_orientation[0],
        m_mfx_knobs.mag_orientation[1],
        m_mfx_knobs.mag_orientation[2],
        m_mfx_knobs.mag_orientation[3]);
    debug_printf("MFX_engine_output_ref_sys: %u", m_mfx_knobs.output_type);
    debug_printf("start_automatic_gbias_calculation: %d", m_mfx_knobs.start_automatic_gbias_calculation);
    debug_printf("");

    m_sensors_initialized = true;
}

/**
 * @brief  Function implementing the stability task thread.
 * @param  argument: Not used
 * @retval None
 */
void stability_thread(void* argument)
{
#if(SC_OPERATING_MODE == SC_OP_MODE_MAG_CAL)
    MotionFX_MagCal_init(20, true);
#endif
    MotionFX_enable_6X(SC_ENABLE_6AXIS_MFX_LIB);
    MotionFX_enable_9X(SC_ENABLE_9AXIS_MFX_LIB);

    for (;;)
    {
        // Recover from IMU data ready interrupt before we were ready
        if (m_int1_count)
        {
            read_imu_data();
            // debug_printf("Recovered int1 from sensor task.");
        }
        m_int1_count = 0;

        // Recover Mag Data interupt before we were ready
        if (m_drdy_count || HAL_GPIO_ReadPin(IMU_MAG_DRDY_GPIO_Port, IMU_MAG_DRDY_Pin))
        {
            read_mag_data();
            // debug_printf("Recovered drdy from sensor task.");
        }
        m_drdy_count = 0;

        // Recover barometer data interrupt from before we were ready
        if (m_baro_int_count || HAL_GPIO_ReadPin(BARO_INT_GPIO_Port, BARO_INT_Pin))
        {
            // debug_printf("Recovered baro int from sensor task.");
            read_baro_data();
        }
        m_baro_int_count = 0;

        // The main stability control loop
        // Calculate kinematics data
        float dT = stability_thread_read_and_calculate();

        static float pitch_pid_out = 0;
        static float roll_pid_out = 0;
        static float yaw_pid_out = 0;

        // Check angle within safe range
        if (fabs(m_kinematics.pitch) > SC_SAFETY_PITCH_LIMIT || fabs(m_kinematics.roll) > SC_SAFETY_ROLL_LIMIT)
        {
            flight_app_critical_angle_callback();
        }
        else
        {
            if (flight_app_get_state() >= FLIGHT_MOTOR_READY)
            {
                // Calculate pid control values
                pitch_pid_out = pid_calculate(&m_pitch_pid, m_kinematics.pitch, dT);
                // roll_pid_out = pid_calculate(&m_roll_pid, m_kinematics.roll, dT);
                // yaw_pid_out = pid_calculate(&m_yaw_pid, m_kinematics.yaw, dT);

                debug_printf("\t%3.2f \t%3.2f", m_kinematics.pitch, pitch_pid_out);


                // Selectively disable PID control
                // pitch_pid_out = 0;
                roll_pid_out = 0;
                yaw_pid_out = 0;
                flight_app_new_stability_inputs(&pitch_pid_out, &roll_pid_out, &yaw_pid_out);
            }

        }

        osDelay(STABILITY_THREAD_PERIOD);
    }
}

/*********************************************************************************************/
/* Getters / Setters for PID targets --------------------------------------------------------*/

void stability_set_pitch_target(float pitch_target)
{
    m_pitch_pid.setpoint = pitch_target;
}

float stability_get_pitch_target()
{
    return m_pitch_pid.setpoint;
}

void stability_set_roll_target(float roll_target)
{
    m_roll_pid.setpoint = roll_target;
}

float stability_get_roll_target()
{
    return m_roll_pid.setpoint;
}

void stability_set_yaw_target(float yaw_target)
{
    m_yaw_pid.setpoint = yaw_target;
}

float stability_get_yaw_target()
{
    return m_yaw_pid.setpoint;
}
