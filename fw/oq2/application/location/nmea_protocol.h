/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\location\nmea_protocol.h            /
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, December 27th 2020, 6:06:51 am                                            /
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


#ifndef NMEA_PROTOCOL_H_
#define NMEA_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>
#include "ascii_lookup.h"


/*********************************************************************************************/
/* NMEA Protocol Frame ----------------------------------------------------------------------*/

#define NMEA_START_CHAR                 '$'
#define NMEA_END_CHAR                   '*'
#define NMEA_SEPARATOR                  ','

// Offsets ignoring start char
#define NMEA_OFFSET_TALKER_ID           0
#define NMEA_OFFSET_SHORT_TALKER_ID     1
#define NMEA_OFFSET_SENTENCE_FORMAT     2
#define NMEA_OFFSET_DATA                5

#define NMEA_LENGTH_TALKED_ID           2
#define NMEA_LENGTH_SENTENCE_FORMAT     3

#define NMEA_TALKER_ID_START_CHAR       'G'
#define NMEA_TALKER_ID_GPS_SBAS_QZSS    'P'    // "$GP"
#define NMEA_TALKER_ID_GLONASS          'L'    // "$GL"
#define NMEA_TALKER_ID_GALILEO          'A'    // "$GA"
#define NMEA_TALKER_ID_BEIDOU           'B'    // "$GB"
#define NMEA_TALKER_ID_ANY_GNSS         'N'    // "$GN"

// Extra fields in NMEA 4.10 and above
#define NMEA_SF_SATELLITE_FAULT         "GBS"   // 31.2.3 GNSS satellite fault detection
#define NMEA_SF_GNSS_FIX_DATA           "GNS"   // 31.2.8 GNSS fix data
#define NMEA_SF_GNSS_RANGE_RESIDUALS    "GRS"   // 31.2.10 GNSS range residuals
#define NMEA_SF_DOP_NUM_SAT             "GSA"   // 31.2.11 GNSS DOP and active satellites
#define NMEA_SF_SAT_IN_VIEW             "GSV"   // 31.2.13 GNSS satellites in view
#define NMEA_SF_REC_MIN_DATA            "RMC"   // 31.2.14 Recommended minimum data

// Standard NMEA messages
#define NMEA_SF_DATAM_REFERENCE         "DTM"   // 31.2.1 Datum reference
#define NMEA_SF_GPS_FIX_DATA            "GGA"   // 31.2.4 Global positioning system fix data
#define NMEA_SF_LAT_LONG                "GLL"   // 31.2.5 Latitude and longitude, with time of position fix and status
#define NMEA_SF_POLL_BEIDOU_MESSAGE     "GBQ"   // 31.2.2 Poll a standard message (Talker ID GB)
#define NMEA_SF_POLL_GALILEO_MESSAGE    "GLQ"   // 31.2.6 Poll a standard message (Talker ID GL)
#define NMEA_SF_POLL_GNSS_MESSAGE       "GNQ"   // 31.2.7 Poll a standard message (Talker ID GN)
#define NMEA_SF_POLL_GPS_MESSAGE        "GPQ"   // 31.2.9 Poll a standard message (Talker ID GP)
#define NMEA_SF_PSEUDO_ERR_STAT         "GST"   // 31.2.12 GNSS pseudorange error statistics
#define NMEA_SF_TRUE_HEADING_STATUS     "THS"   // 31.2.15 True heading and status
#define NMEA_SF_TEXT                    "TXT"   // 31.2.16 Text transmission
#define NMEA_SF_GROUND_WATER_DISTANCE   "VLW"   // 31.2.17 Dual ground/water distance
#define NMEA_SF_GROUND_COURSE_SPEED     "VTG"   // 31.2.18 Course over ground and ground speed
#define NMEA_SF_TIME_AND_DATE           "ZDA"   // 31.2.19 Time and date

/**
 * @brief Parse two ascii characters representing a hex byte into a uint8_t
 * 
 * @param pdata 
 * @return __always_inline 
 */
__always_inline
uint8_t _nmea_parse_cs(uint8_t * pdata)
{
    // char char1 = *(pdata+0);
    // char char2 = *(pdata+1);

    // uint8_t look1 = char1 - '0';
    // uint8_t look2 = char2 - '0';

    return (ASCII_LOOKUP[(*(pdata+0) - '0')] << 4) |
           (ASCII_LOOKUP[(*(pdata+1) - '0')] << 0);
}

/**
 * @brief For a given string representation of a decimal number, 
 * return the decimal number as an integer.
 * 
 * Ex. 
 * _nmea_parse_int_from_string("1234", 4) -> 1234
 * 
 * @param pdata pointer to start of string
 * @param num_chars number of characters to parse
 * @return uint32_t integer value
 */
uint32_t _nmea_parse_int_from_string(char * pdata, uint32_t num_chars)
{
    uint32_t magnitude = pow(10, num_chars-1);
    uint32_t result = 0;

    for(int i = 0; i < num_chars; i++)
    {
        int look_index = (*(pdata+i) - '0');
        
        // Make sure look_index is still inside the ascii table
        if(look_index < 0 || look_index > sizeof(ASCII_LOOKUP))
            return 0;

        result += ASCII_LOOKUP[look_index] * magnitude;
        magnitude /= 10;
    }

    return result;
}

/**
 * @brief Compare the constant sentence format provided to the received sentence format
 * 
 * @param compare example NMEA_SF_GPS_FIX_DATA, 3 character long string constant
 * @return true if matched
 */
__always_inline 
bool _nmea_sf_match(uint8_t * pdata, char const * compare)
{
    return (memcmp(pdata+NMEA_OFFSET_SENTENCE_FORMAT, compare, NMEA_LENGTH_SENTENCE_FORMAT) == 0);
}

/**
 * @brief Routine to calculate the 8 bit XOR checksum of an array of bytes
 * 
 * @param data pointer to start of data
 * @param length length of data
 * @return uint8_t XOR checksum
 */
uint8_t _crc_xor_8(uint8_t * data, uint32_t length)
{
    uint8_t cs = 0x00;

    for(int i = 0; i < length; i++)
    {
        cs ^= data[i];
    }

    return cs;
}



#endif