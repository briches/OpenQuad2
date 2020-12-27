/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\location\location.c                 /
 * Project: OQ2                                                                                    /
 * Created Date: Tuesday, December 15th 2020, 8:05:02 pm                                           /
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


#include "location.h"
#include "app_config.h"
#include "debug_log.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>
#include "nmea_protocol.h"

#define debug_error(fmt, ...)           debug_error(LOCATION_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(LOCATION_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(LOCATION_MODULE_ID, fmt, ##__VA_ARGS__)

static uint8_t gps_rx_buf[128];
static uint8_t gps_tx_buf[128];
static uint8_t gps_rx_char;
static volatile uint32_t message_length = 0;
static volatile uint32_t rx_count = 0;

char const sep = NMEA_SEPARATOR;

/*********************************************************************************************/
/* GNSS State -------------------------------------------------------------------------------*/
static bool m_general_fix_state = false;
static utc_time_t m_utc_time = {0};
static float m_latitude = 0;
static float m_longitude = 0;
static float m_hdop = 0;
static float m_elevation = 0;
static uint32_t m_tracking_sats = 0;
static uint32_t m_quality = 10;


/*********************************************************************************************/
/* Internal Functions -----------------------------------------------------------------------*/

/**
 * @brief Set the RESET state of the GPS module. Since the pin is negative logic,
 * the logic is flipped from RESET to pin state.
 *
 * @param reset to reset or not
 */
__always_inline
static void _gps_set_reset_state(bool reset)
{
    HAL_GPIO_WritePin(GPS_RESETN_GPIO_Port, GPS_RESETN_Pin, !reset);
}

/**
 * @brief Calculate the XOR checksum of the message and verify it against the
 * received message checksum
 *
 * @return true match
 * @return false mismatch
 */
static bool _gps_verify_checksum(uint8_t * pbuf)
{
    uint8_t cs_calc = _crc_xor_8(pbuf + NMEA_OFFSET_TALKER_ID, message_length - 3);
    uint8_t cs_rx = _nmea_parse_cs(pbuf + message_length - 2);

    if (cs_calc == cs_rx)
    {
        // debug_printf("Checksum match 0x%02X == 0x%02X", cs_calc, cs_rx);
        return true;
    }
    else
    {
        debug_error("Checksum mismatch 0x%02X != 0x%02X", cs_calc, cs_rx);
        // debug_error("%s", pbuf);
        return false;
    }
}

/**
 * @brief Parse received message.
 */
static void _gps_received_message_callback()
{
    uint8_t * pbuf;

    if(*gps_rx_buf == NMEA_START_CHAR)
    {
        pbuf = gps_rx_buf + 1;
        message_length--;
    }
    else if (*gps_rx_buf == NMEA_TALKER_ID_START_CHAR)
    {
        pbuf = gps_rx_buf;
    }

    // debug_printf("%s", gps_rx_buf);

    // Validate the message checksum
    if (_gps_verify_checksum(pbuf) == false)
        return;

    // Check that the talked ID is of GPS RECEIVER format
    if (pbuf[NMEA_OFFSET_TALKER_ID] != 'G')
    {
        debug_error("Received unhandled talker ID.");
        debug_error("%s", pbuf);
        return;
    }

    __unused
    char talker = pbuf[NMEA_OFFSET_SHORT_TALKER_ID];

    // Check the sentence format code for the message type
    if (_nmea_sf_match(pbuf, NMEA_SF_TEXT))
    {
        debug_printf("%s", pbuf + 15);

    }
    // "RMC"
    // Can get fix status from here. 
    else if (_nmea_sf_match(pbuf, NMEA_SF_REC_MIN_DATA))
    {
        // debug_error("%s", pbuf);

        char* token = strtok((char *)(pbuf), &sep);
        uint8_t token_count = 1;

        while (token != NULL)
        {
            switch (token_count)
            {
                case 1:
                    break;

                case 2:
                {
                    // If the TIME was null, we would get the status here.
                    if (*token == 'V')
                    {
                        m_general_fix_state = false;
                    }
                    else
                    {
                        // debug_printf("%s", token);

                        // Time wasn't null. Get the time.
                        m_utc_time.hours = _nmea_parse_int_from_string(token, 2);
                        m_utc_time.minutes = _nmea_parse_int_from_string(token + 2, 2);
                        m_utc_time.seconds = _nmea_parse_int_from_string(token + 4, 2);
                        m_utc_time.millis = _nmea_parse_int_from_string(token + 7, 2);
                        m_utc_time.valid = true;

                        // debug_printf("%u:%u:%u.%u UTC", m_utc_time.hours, m_utc_time.minutes, m_utc_time.seconds, m_utc_time.millis);
                    }
                } break;

                case 3:
                {
                    if(*token == 'V')
                    {
                        m_general_fix_state = false;
                    }
                    else if(*token == 'A')
                    {
                        m_general_fix_state = true;
                        // debug_printf("Fix state: %c", *token);
                    }
                } break;

                default:
                    break;
            }

            token = strtok(NULL, &sep);
            token_count++;
        }

        gnss_fix_status_callback();
    }
    // "GGA"
    else if (_nmea_sf_match(pbuf, NMEA_SF_GPS_FIX_DATA) &&
            m_general_fix_state == true)
    {
        // debug_error("%s", pbuf);

        char* token = strtok((char *)(pbuf), &sep);
        uint8_t token_count = 1;

        // All the following tokens will be parsed assuming the fix is aquired
        while (token != NULL)
        {
            switch (token_count)
            {
                case 1:
                    // GxGGA
                break;

                case 2:
                {
                    // Time wasn't null. Get the time.
                    m_utc_time.hours = _nmea_parse_int_from_string(token, 2);
                    m_utc_time.minutes = _nmea_parse_int_from_string(token + 2, 2);
                    m_utc_time.seconds = _nmea_parse_int_from_string(token + 4, 2);
                    m_utc_time.millis = _nmea_parse_int_from_string(token + 7, 2);
                    m_utc_time.valid = true;

                } break;

                // Latitude field
                case 3:
                {
                    float deg = _nmea_parse_int_from_string(token, 2);
                    float minutes_int = _nmea_parse_int_from_string(token+2, 2);
                    float minutes_dec = _nmea_parse_int_from_string(token+5, 5);

                    m_latitude = deg + (minutes_int + minutes_dec / 100000) / 60;
                } break;

                // Latitude direction (N or S)
                case 4:
                {
                    if(*token == 'S')
                        m_latitude *= -1;
                } break;

                // Longitude field
                case 5:
                {
                    float deg = _nmea_parse_int_from_string(token, 3);
                    float minutes_int = _nmea_parse_int_from_string(token+3, 2);
                    float minutes_dec = _nmea_parse_int_from_string(token+6, 5);

                    m_longitude = deg + (minutes_int + minutes_dec / 100000) / 60;
                } break;

                // Longitude Direction (E (+ve) or W (-ve))
                case 6: 
                {
                    if(*token == 'W')
                        m_longitude *= -1;
                } break;

                // Quality
                case 7:
                {
                    m_quality = _nmea_parse_int_from_string(token, 1);
                } break;

                // Number of tracking satellites used 
                case 8:
                {
                    m_tracking_sats = _nmea_parse_int_from_string(token, 2);
                } break;

                // Horizontal dilution of precision
                case 9:
                {
                    m_hdop = _nmea_parse_int_from_string(token, 1);
                    float hdop_dec = _nmea_parse_int_from_string(token+2, 2);

                    m_hdop += hdop_dec / 100;
                } break;

                // Altitude
                case 10:
                {
                    uint8_t tok_len = strlen(token);
                    m_elevation = _nmea_parse_int_from_string(token, tok_len-2);
                    m_elevation += (0.1f) * (float)_nmea_parse_int_from_string(token + tok_len-1, 1); 

                    gnss_new_data_callback();
                } break;

                default:
                    break;
            }

            token = strtok(NULL, &sep);
            token_count++;
        }

    }
    // GNS
    else if (_nmea_sf_match(pbuf, NMEA_SF_GNSS_FIX_DATA) &&
            m_general_fix_state == true) 
    {
        debug_error("%s", pbuf);
    }
    else
    {
        // debug_printf("%s", pbuf);
    }
}

/*********************************************************************************************/
/* Exported Functions -----------------------------------------------------------------------*/

/**
 * @brief Call external listeners for receiving a GPS fix
 * 
 */
void gnss_fix_status_callback()
{
    static bool last_fix = false;

    if(m_general_fix_state != last_fix)
    {
        last_fix = m_general_fix_state;
        debug_printf("Fix status updated: %u", m_general_fix_state);


        /** @todo Inform listeners */
    }
}

/**
 * @brief Call external listeners for GNSS data
 * 
 */
void gnss_new_data_callback()
{
    debug_printf("%u:%u:%u.%u UTC", m_utc_time.hours, m_utc_time.minutes, m_utc_time.seconds, m_utc_time.millis);
    debug_printf("lat/long: %3.6f, %3.6f", m_latitude, m_longitude);
    debug_printf("Quality: %u", m_quality);
    debug_printf("Satellites: %u", m_tracking_sats);
    debug_printf("HDOP: %1.2f", m_hdop);
    debug_printf("Elevation: %.1f", m_elevation);

    /** @todo Inform listeners */
}

/**
 * @brief Callback function for handling HAL_UART_TxCpltCallback for the GPS uart
 *
 */
void gps_tx_complete_callback()
{
    debug_printf("gps_tx_complete_callback");
}

/**
 * @brief Callback function for handling HAL_UART_RxCpltCallback for the GPS uart
 *
 */
void gps_rx_complete_callback()
{
    if (gps_rx_char != '\r' && gps_rx_char != '\n')
    {
        // Copy the received character into the RX buffer
        gps_rx_buf[rx_count++] = gps_rx_char;

        if (gps_rx_char == '*')
        {
            // Two more bytes to receive for the CRC
            message_length = rx_count + 2;
        }

        // Received full message
        if (rx_count == message_length)
        {
            _gps_received_message_callback();

            memset(gps_rx_buf, 0x00, sizeof(gps_rx_buf));
            rx_count = 0;
            message_length = 0;
        }
    }

    HAL_UART_Receive_IT(&huart5, &gps_rx_char, 1);
}

/**
 * @brief Initialization prior to the kernel being initialized and any tasks being created
 *
 */
void location_thread_pre_init()
{
    message_length = 0;
    rx_count = 0;
}

/**
 * @brief  Function implementing the location task thread.
 * @param  argument: Not used
 * @retval None
 */
void location_thread_start(void* argument)
{
    // Start receiving
    HAL_UART_Receive_IT(&huart5, &gps_rx_char, 1);

    // Disable GPS Reset
    _gps_set_reset_state(false);

    for (;;)
    {
        osDelay(LOCATION_THREAD_PERIOD);

        if (osKernelGetState() == osKernelRunning)
        {
            // HAL_UART_Transmit_IT(&huart5, gps_tx_buf, hello_size);
            // debug_printf("Location.");
        }

        // debug_print_buffer(gps_rx_buf, 30, 0, 16);

    }
}

