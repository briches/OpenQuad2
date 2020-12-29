/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\wifi_drv_winc3400\common\source\nm_common.c/
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
 * \brief This module contains common APIs declarations.
 *
 * Copyright (c) 2016-2019 Microchip Technology Inc. and its subsidiaries.
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
#include "nm_common.h"

#include "debug_log.h"
#define M2M_ERR(fmt, ...)               debug_error(WINC3400_COMMON_MODULE_ID, fmt, ##__VA_ARGS__)
#define M2M_INFO(fmt, ...)              debug_printf(WINC3400_COMMON_MODULE_ID, fmt, ##__VA_ARGS__)

void m2m_memcpy(uint8_t *pDst, uint8_t *pSrc, uint32_t sz)
{
    if(sz == 0) return;
    do
    {
        *pDst = *pSrc;
        pDst++;
        pSrc++;
    } while(--sz);
}
uint8_t m2m_checksum(uint8_t *buf, int sz)
{
    uint8_t cs = 0;
    while(--sz)
    {
        cs ^= *buf;
        buf++;
    }

    return cs;
}

void m2m_memset(uint8_t *pBuf, uint8_t val, uint32_t sz)
{
    if(sz == 0) return;
    do
    {
        *pBuf = val;
        pBuf++;
    } while(--sz);
}

uint16_t m2m_strlen(const uint8_t *pcStr)
{
    uint16_t  u16StrLen = 0;
    while(*pcStr)
    {
        u16StrLen ++;
        pcStr++;
    }
    return u16StrLen;
}

uint8_t m2m_strncmp(uint8_t *pcS1, uint8_t *pcS2, uint16_t u16Len)
{
    for(; u16Len > 0; pcS1++, pcS2++, --u16Len)
        if(*pcS1 != *pcS2)
            return ((*(uint8_t *)pcS1 < *(uint8_t *)pcS2) ? -1 : +1);
        else if(*pcS1 == '\0')
            return 0;
    return 0;
}

/* Finds the occurrence of pcStr in pcIn.
If pcStr is part of pcIn it returns a valid pointer to the start of pcStr within pcIn.
Otherwise a NULL Pointer is returned.
*/
uint8_t *m2m_strstr(uint8_t *pcIn, uint8_t *pcStr)
{
    uint8_t u8c;
    uint16_t u16StrLen;

    u8c = *pcStr++;
    if(!u8c)
        return (uint8_t *) pcIn;  // Trivial empty string case

    u16StrLen = m2m_strlen(pcStr);
    do {
        uint8_t u8Sc;

        do {
            u8Sc = *pcIn++;
            if(!u8Sc)
                return (uint8_t *) 0;
        } while(u8Sc != u8c);
    } while(m2m_strncmp(pcIn, pcStr, u16StrLen) != 0);

    return (uint8_t *)(pcIn - 1);
}

int8_t m2m_memcmp(uint8_t *pu8Buff1, uint8_t *pu8Buff2, uint32_t u32Size)
{
    uint32_t  i;
    int8_t       s8Result = 0;
    for(i    = 0 ; i < u32Size ; i++)
    {
        if(pu8Buff1[i] != pu8Buff2[i])
        {
            s8Result = 1;
            break;
        }
    }
    return s8Result;
}

/* Convert hexchar to value 0-15 */
static uint8_t hexchar_2_val(uint8_t ch)
{
    ch -= 0x30;
    if(ch <= 9)
        return ch;
    ch |= 0x20;
    ch -= 0x31;
    if(ch <= 5)
        return ch + 10;
    return 0xFF;
}

/* Convert hexstring to bytes */
int8_t hexstr_2_bytes(uint8_t *pu8Out, uint8_t *pu8In, uint8_t u8SizeOut)
{
    while(u8SizeOut--)
    {
        uint8_t   u8Out = hexchar_2_val(*pu8In++);
        if(u8Out > 0xF)
            return M2M_ERR_INVALID_ARG;
        *pu8Out = u8Out * 0x10;
        u8Out = hexchar_2_val(*pu8In++);
        if(u8Out > 0xF)
            return M2M_ERR_INVALID_ARG;
        *pu8Out += u8Out;
        pu8Out++;
    }
    return M2M_SUCCESS;
}
