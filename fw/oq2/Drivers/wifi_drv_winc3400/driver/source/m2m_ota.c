/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\wifi_drv_winc3400\driver\source\m2m_ota.c/
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
 * \brief WINC3400 IoT OTA Interface.
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

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "nm_common.h"
#include "m2m_types.h"
#include "m2m_ota.h"
#include "m2m_wifi.h"
#include "m2m_hif.h"
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "debug_log.h"
#define M2M_ERR(fmt, ...)               debug_error(WINC3400_OTA_MODULE_ID, fmt, ##__VA_ARGS__)
#define M2M_INFO(fmt, ...)              debug_printf(WINC3400_OTA_MODULE_ID, fmt, ##__VA_ARGS__)

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
static tpfOtaUpdateCb gpfOtaUpdateCb = NULL;
static tpfOtaNotifCb  gpfOtaNotifCb  = NULL;

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/**
@fn         m2m_ota_cb(uint8_t u8OpCode, uint16_t u16DataSize, uint32_t u32Addr)
@brief      Internal OTA call back function.
@param[in]  u8OpCode
                HIF Opcode type.
@param[in]  u16DataSize
                HIF data length.
@param[in]  u32Addr
                HIF address.
*/
static void m2m_ota_cb(uint8_t u8OpCode, uint16_t u16DataSize, uint32_t u32Addr)
{
    int8_t s8Ret = M2M_SUCCESS;
    if(u8OpCode == M2M_OTA_RESP_NOTIF_UPDATE_INFO)
    {
        tstrOtaUpdateInfo strOtaUpdateInfo;
        m2m_memset((uint8_t *)&strOtaUpdateInfo, 0, sizeof(tstrOtaUpdateInfo));
        s8Ret = hif_receive(u32Addr, (uint8_t *)&strOtaUpdateInfo, sizeof(tstrOtaUpdateInfo), 0);
        if(s8Ret == M2M_SUCCESS)
        {
            if(gpfOtaNotifCb)
                gpfOtaNotifCb(&strOtaUpdateInfo);
        }
    }
    else if(u8OpCode == M2M_OTA_RESP_UPDATE_STATUS)
    {
        tstrOtaUpdateStatusResp strOtaUpdateStatusResp;
        m2m_memset((uint8_t *)&strOtaUpdateStatusResp, 0, sizeof(tstrOtaUpdateStatusResp));
        s8Ret = hif_receive(u32Addr, (uint8_t *) &strOtaUpdateStatusResp, sizeof(tstrOtaUpdateStatusResp), 0);
        if(s8Ret == M2M_SUCCESS)
        {
            if(gpfOtaUpdateCb)
                gpfOtaUpdateCb(strOtaUpdateStatusResp.u8OtaUpdateStatusType, strOtaUpdateStatusResp.u8OtaUpdateStatus);
        }
    }
    else
    {
        M2M_ERR("Invalid OTA resp %d ?\n", u8OpCode);
    }
}

/*!
@fn         NMI_API int8_t  m2m_ota_init(tpfOtaUpdateCb pfOtaUpdateCb, tpfOtaNotifCb pfOtaNotifCb)
@brief      Initialize the OTA layer.
@param[in]  pfOtaUpdateCb
                OTA Update callback function.
@param[in]  pfOtaNotifCb
                OTA notify callback function.
@return     The function returns @ref M2M_SUCCESS for success and a negative value otherwise.
*/
NMI_API int8_t  m2m_ota_init(tpfOtaUpdateCb pfOtaUpdateCb, tpfOtaNotifCb pfOtaNotifCb)
{
    int8_t ret = M2M_SUCCESS;

    if(pfOtaUpdateCb) {
        gpfOtaUpdateCb = pfOtaUpdateCb;
    } else {
        M2M_ERR("Invalid Ota update cb\n");
    }
    if(pfOtaNotifCb) {
        gpfOtaNotifCb = pfOtaNotifCb;
    } else {
        M2M_ERR("Invalid Ota notify cb\n");
    }

    hif_register_cb(M2M_REQ_GROUP_OTA, m2m_ota_cb);

    return ret;
}

/*!
@fn         NMI_API int8_t  m2m_ota_notif_set_url(uint8_t * u8Url)
@brief      Set the OTA url.
@param[in]  u8Url
                The url server address.
@return     The function returns @ref M2M_SUCCESS for success and a negative value otherwise.
*/
NMI_API int8_t  m2m_ota_notif_set_url(uint8_t *u8Url)
{
    int8_t ret = M2M_SUCCESS;
    uint16_t u16UrlSize = m2m_strlen(u8Url) + 1;
    /*Todo: we may change it to data pkt but we need to give it higher priority
            but the priority is not implemented yet in data pkt
    */
    ret = hif_send(M2M_REQ_GROUP_OTA, M2M_OTA_REQ_NOTIF_SET_URL, u8Url, u16UrlSize, NULL, 0, 0);
    return ret;
}

/*!
@fn         NMI_API int8_t  m2m_ota_notif_check_for_update(void)
@brief      Check for OTA update.
@return     The function returns @ref M2M_SUCCESS for success and a negative value otherwise.
*/
NMI_API int8_t  m2m_ota_notif_check_for_update(void)
{
    int8_t ret = M2M_SUCCESS;
    ret = hif_send(M2M_REQ_GROUP_OTA, M2M_OTA_REQ_NOTIF_CHECK_FOR_UPDATE, NULL, 0, NULL, 0, 0);
    return ret;
}

/*!
@fn         NMI_API int8_t m2m_ota_notif_sched(uint32_t u32Period)
@brief      Schedule OTA update.
@param[in]  u32Period
                Period in days
@return     The function returns @ref M2M_SUCCESS for success and a negative value otherwise.
*/
NMI_API int8_t m2m_ota_notif_sched(uint32_t u32Period)
{
    int8_t ret = M2M_SUCCESS;
    ret = hif_send(M2M_REQ_GROUP_OTA, M2M_OTA_REQ_NOTIF_CHECK_FOR_UPDATE, NULL, 0, NULL, 0, 0);
    return ret;
}

/*!
@fn         NMI_API int8_t m2m_ota_start_update(unsigned char * pcDownloadUrl)
@brief      Request OTA start update using the downloaded URL.
@param[in]  pcDownloadUrl
                The download firmware URL, you get it from device info.
@return     The function returns @ref M2M_SUCCESS for success and a negative value otherwise.
*/
NMI_API int8_t m2m_ota_start_update(unsigned char *pcDownloadUrl)
{
    int8_t ret = M2M_SUCCESS;
    uint16_t u16DurlSize = m2m_strlen(pcDownloadUrl) + 1;
    /*Todo: we may change it to data pkt but we need to give it higher priority
            but the priority is not implemented yet in data pkt
    */
    ret = hif_send(M2M_REQ_GROUP_OTA, M2M_OTA_REQ_START_UPDATE, pcDownloadUrl, u16DurlSize, NULL, 0, 0);
    return ret;
}

/*!
@fn         NMI_API int8_t m2m_ota_rollback(void)
@brief      Request OTA Rollback image.
@return     The function returns @ref M2M_SUCCESS for success and a negative value otherwise.
*/
NMI_API int8_t m2m_ota_rollback(void)
{
    int8_t ret = M2M_SUCCESS;
    tstrM2mRev strRev;

    ret = m2m_ota_get_firmware_version(&strRev);
    if(ret == M2M_SUCCESS)
    {
        if(M2M_GET_HIF_BLOCK(strRev.u16FirmwareHifInfo) == M2M_HIF_BLOCK_VALUE)
        {
            ret = hif_send(M2M_REQ_GROUP_OTA, M2M_OTA_REQ_ROLLBACK, NULL, 0, NULL, 0, 0);
        }
        else
        {
            ret = M2M_ERR_FAIL;
        }
    }
    return ret;
}

/*!
@fn         NMI_API int8_t m2m_ota_abort(void)
@brief      Request OTA Abort.
@return     The function returns @ref M2M_SUCCESS for success and a negative value otherwise.
*/
NMI_API int8_t m2m_ota_abort(void)
{
    int8_t ret = M2M_SUCCESS;
    ret = hif_send(M2M_REQ_GROUP_OTA, M2M_OTA_REQ_ABORT, NULL, 0, NULL, 0, 0);
    return ret;
}

/*!
@fn         NMI_API int8_t m2m_ota_switch_firmware(void)
@brief      Switch to the upgraded Firmware.
@return     The function returns @ref M2M_SUCCESS for success and a negative value otherwise.
*/
NMI_API int8_t m2m_ota_switch_firmware(void)
{
    int8_t ret = M2M_SUCCESS;
    tstrM2mRev strRev;

    ret = m2m_ota_get_firmware_version(&strRev);
    if(ret == M2M_SUCCESS)
    {
        if(M2M_GET_HIF_BLOCK(strRev.u16FirmwareHifInfo) == M2M_HIF_BLOCK_VALUE)
        {
            ret = hif_send(M2M_REQ_GROUP_OTA, M2M_OTA_REQ_SWITCH_FIRMWARE, NULL, 0, NULL, 0, 0);
        }
        else
        {
            ret = M2M_ERR_FAIL;
        }
    }
    return ret;
}
#if 0
#define M2M_OTA_FILE    "../../../m2m_ota.dat"
NMI_API int8_t m2m_ota_test(void)
{
    uint32_t page  = 0;
    uint8_t buffer[1500];
    uint32_t u32Sz = 0;
    int8_t ret = M2M_SUCCESS;
    FILE *fp =NULL;
    fp = fopen(M2M_OTA_FILE, "rb");
    if(fp)
    {
        fseek(fp, 0L, SEEK_END);
        u32Sz = ftell(fp);
        fseek(fp, 0L, SEEK_SET);

        while(u32Sz > 0)
        {
            {
                page = (rand()%1400);

                if((page<100)||(page>1400)) page  = 1400;
            }

            if(u32Sz>page)
            {
                u32Sz-=page;
            }
            else
            {
                page = u32Sz;
                u32Sz = 0;
            }
            printf("page %d\n", (int)page);
            fread(buffer, page, 1, fp);
            ret = hif_send(M2M_REQ_GROUP_OTA, M2M_OTA_REQ_TEST|M2M_REQ_DATA_PKT, NULL, 0, (uint8_t *)&buffer, page, 0);
            if(ret != M2M_SUCCESS)
            {
                M2M_ERR("\n");
            }
            nm_bsp_sleep(1);
        }
    }
    else
    {
        M2M_ERR("nO err\n");
    }
    return ret;
}
#endif

