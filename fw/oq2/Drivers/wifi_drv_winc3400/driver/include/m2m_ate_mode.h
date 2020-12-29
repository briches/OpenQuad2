/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\wifi_drv_winc3400\driver\include\m2m_ate_mode.h/
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
 * \brief WINC3400 Peripherals Application Interface.
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

#ifdef _M2M_ATE_FW_

#ifndef _M2M_ATE_MODE_H_
#define _M2M_ATE_MODE_H_

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "nm_common.h"
#include "m2m_types.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#define M2M_ATE_MAX_NUM_OF_RATES        (20)
/*!< Maximum number of all rates (b,g and n)
 */
#define M2M_ATE_MAX_FRAME_LENGTH        (1024)
/*!< Maximum number of length for each frame
 */
#define M2M_ATE_MIN_FRAME_LENGTH        (1)
/*!< Minimum number of length for each frame
 */


#define M2M_ATE_SUCCESS                 (M2M_SUCCESS)
/*!< No Error and operation has been completed successfully.
*/
#define M2M_ATE_ERR_VALIDATE            (M2M_ERR_FAIL)
/*!< Error in parameters passed to functions.
 */
#define M2M_ATE_ERR_TX_ALREADY_RUNNING  (-1)
/*!< This means that TX case is already running and RX or even TX can't start without stopping it first.
 */
#define M2M_ATE_ERR_RX_ALREADY_RUNNING  (-2)
/*!< This means that RX case is already running and TX or even RX can't start without stopping it first.
 */
#define M2M_ATE_ERR_UNHANDLED_CASE      (-3)
/*!< Invalid case.
 */
#define M2M_ATE_RX_DISABLE_DA               0x0
#define M2M_ATE_RX_ENABLE_DA                0x1

#define M2M_ATE_RX_DISABLE_SA               0x0
#define M2M_ATE_RX_ENABLE_SA                0x1

#define M2M_ATE_DISABLE_SELF_MACADDR        0x0
#define M2M_ATE_SET_SELF_MACADDR            0x1
/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
DATA TYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/*!
 *@enum     tenuM2mAteFwState
 *@brief    Enumeration used for change ATE firmware state
 */
typedef enum {
    M2M_ATE_FW_STATE_STOP           = 0x00,
    /*!< State to stop ATE firmware
     */
    M2M_ATE_FW_STATE_RUN            = 0x01,
    /*!< State to run ATE firmware
     */
} tenuM2mAteFwState;

/*!
 *@enum     tenuM2mAteTxRates
 *@brief    Used to get value of rate referenced by this index
 */
typedef enum {
    M2M_ATE_TX_RATE_1_Mbps_INDEX    = 0x00,
    M2M_ATE_TX_RATE_2_Mbps_INDEX    = 0x01,
    M2M_ATE_TX_RATE_55_Mbps_INDEX   = 0x02,
    M2M_ATE_TX_RATE_11_Mbps_INDEX   = 0x03,
    /*!< B-Rates
     */
    M2M_ATE_TX_RATE_6_Mbps_INDEX    = 0x04,
    M2M_ATE_TX_RATE_9_Mbps_INDEX    = 0x05,
    M2M_ATE_TX_RATE_12_Mbps_INDEX   = 0x06,
    M2M_ATE_TX_RATE_18_Mbps_INDEX   = 0x07,
    M2M_ATE_TX_RATE_24_Mbps_INDEX   = 0x08,
    M2M_ATE_TX_RATE_36_Mbps_INDEX   = 0x09,
    M2M_ATE_TX_RATE_48_Mbps_INDEX   = 0x0A,
    M2M_ATE_TX_RATE_54_Mbps_INDEX   = 0x0B,
    /*!< G-Rates
     */
    M2M_ATE_TX_RATE_MCS_0_INDEX     = 0x0C,
    M2M_ATE_TX_RATE_MCS_1_INDEX     = 0x0D,
    M2M_ATE_TX_RATE_MCS_2_INDEX     = 0x0E,
    M2M_ATE_TX_RATE_MCS_3_INDEX     = 0x0F,
    M2M_ATE_TX_RATE_MCS_4_INDEX     = 0x10,
    M2M_ATE_TX_RATE_MCS_5_INDEX     = 0x11,
    M2M_ATE_TX_RATE_MCS_6_INDEX     = 0x12,
    M2M_ATE_TX_RATE_MCS_7_INDEX     = 0x13,
    /*!< N-Rates
     */
} tenuM2mAteTxIndexOfRates;

/*!
 *@enum     tenuM2mAteTxDutyCycle
 *@brief    Values of duty cycle
 */
typedef enum {
    M2M_ATE_TX_DUTY_1               = 0x01,
    M2M_ATE_TX_DUTY_2               = 0x02,
    M2M_ATE_TX_DUTY_3               = 0x03,
    M2M_ATE_TX_DUTY_4               = 0x04,
    M2M_ATE_TX_DUTY_5               = 0x05,
    M2M_ATE_TX_DUTY_6               = 0x06,
    M2M_ATE_TX_DUTY_7               = 0x07,
    M2M_ATE_TX_DUTY_8               = 0x08,
    M2M_ATE_TX_DUTY_9               = 0x09,
    M2M_ATE_TX_DUTY_10              = 0xA0,
} tenuM2mAteTxDutyCycle;


#define M2M_ATE_TX_DUTY_MAX_VALUE   M2M_ATE_TX_DUTY_1
/*!< The maximum value of duty cycle
*/
#define M2M_ATE_TX_DUTY_MIN_VALUE   M2M_ATE_TX_DUTY_10
/*!< The minimum value of duty cycle
*/

/*!
 *@enum     tenuM2mAteTxDpdControl
 *@brief    Allowed values for DPD control
 */
typedef enum {
    M2M_ATE_TX_DPD_DYNAMIC  = 0x00,
    M2M_ATE_TX_DPD_BYPASS   = 0x01,
    M2M_ATE_TX_DPD_ENABLED  = 0x02,
} tenuM2mAteTxDpdControl;

/*!
 *@enum     tenuM2mAteTxGainSetting
 *@brief    Options for TX gain selection mode
 */
typedef enum {
    M2M_ATE_TX_GAIN_DYNAMIC = 0x00,
    M2M_ATE_TX_GAIN_BYPASS  = 0x01,
    M2M_ATE_TX_GAIN_FCC     = 0x02,
    M2M_ATE_TX_GAIN_TELEC   = 0x03,
} tenuM2mAteTxGainSetting;

/*!
 *@enum     tenuM2mAtePMUSetting
 *@brief    Used to Enable PMU or disable it
 */
typedef enum {
    M2M_ATE_PMU_DISABLE = 0x00,
    M2M_ATE_PMU_ENABLE  = 0x01,
} tenuM2mAtePMUSetting;

/*!
 *@enum     tenuM2mAteTxSource
 *@brief    Used to define if enable PHY continues mode or MAC
 */
typedef enum {
    M2M_ATE_TX_SRC_MAC  = 0x00,
    M2M_ATE_TX_SRC_PHY  = 0x01,
} tenuM2mAteTxSource;

/*!
 *@enum     tenuM2mAteTxMode
 *@brief    Used to define type of TX mode either normal or CW(Continuous Wave) TX sequence
 */
typedef enum {
    M2M_ATE_TX_MODE_NORM    = 0x00,
    M2M_ATE_TX_MODE_CW      = 0x01,
} tenuM2mAteTxMode;

/*!
 *@enum     tenuM2mAteChannels
 *@brief    Available channels for TX and RX
 */
typedef enum {
    M2M_ATE_CHANNEL_1   = 0x01,
    M2M_ATE_CHANNEL_2   = 0x02,
    M2M_ATE_CHANNEL_3   = 0x03,
    M2M_ATE_CHANNEL_4   = 0x04,
    M2M_ATE_CHANNEL_5   = 0x05,
    M2M_ATE_CHANNEL_6   = 0x06,
    M2M_ATE_CHANNEL_7   = 0x07,
    M2M_ATE_CHANNEL_8   = 0x08,
    M2M_ATE_CHANNEL_9   = 0x09,
    M2M_ATE_CHANNEL_10  = 0x0A,
    M2M_ATE_CHANNEL_11  = 0x0B,
    M2M_ATE_CHANNEL_12  = 0x0C,
    M2M_ATE_CHANNEL_13  = 0x0D,
    M2M_ATE_CHANNEL_14  = 0x0E,
} tenuM2mAteChannels;

/*!
 *@struct   tstrM2mAteRxStatus
 *@brief    Used to save statistics of RX case
 */
typedef struct {
    uint32_t num_rx_pkts;
    /*!< Number of total RX packet
     */
    uint32_t num_err_pkts;
    /*!< Number of RX failed packets
     */
    uint32_t num_good_pkts;
    /*!< Number of RX packets actually received
     */
} tstrM2mAteRxStatus;

/*!
 *@struct   tstrM2mAteTx
 *@brief    Used as data source in case of enabling TX test case
 */
typedef struct {
    uint32_t  num_frames;
    /*!< Number of frames to be sent where maximum number allowed is 4294967295 ul, and ZERO means infinite number of frames
     */
    uint32_t  data_rate;
    /*!< Rate to sent packets over to select rate use value of \ref tenuM2mAteTxIndexOfRates and pass it to \ref m2m_ate_get_tx_rate
     */
    uint8_t       channel_num;
    /*!< Channel number \ref tenuM2mAteChannels
     */
    uint8_t    duty_cycle;
    /*!< Duty cycle value between from 1 to 10, where maximum = 1, minimum = 10 \ref tenuM2mAteTxDutyCycle
     */
    uint16_t    frame_len;
    /*!< Use \ref M2M_ATE_MAX_FRAME_LENGTH (1024) as the maximum value while \ref M2M_ATE_MIN_FRAME_LENGTH (1) is the minimum value
     */
    uint8_t     tx_gain_sel;
    /*!< TX gain mode selection value \ref tenuM2mAteTxGainSetting
     */
    uint8_t     dpd_ctrl;
    /*!< DPD mode value\ref tenuM2mAteTxDpdControl
     */
    uint8_t     use_pmu;
    /*!< This is 0 if PMU is not used otherwise it must be be 1 \ref tenuM2mAtePMUSetting
     */
    uint8_t     phy_burst_tx;
    /*!< Source of Burst TX either PHY or MAC\ref tenuM2mAteTxSource
     */
    uint8_t     cw_tx;
    /*!< Mode of Burst TX either normal TX sequence or CW(Continuous Wave) TX sequence \ref tenuM2mAteTxMode
     */
    uint32_t     xo_offset_x1000;
    /*!< Signed XO offset value in PPM (Part Per Million) multiplied by 1000.
     */
    uint8_t     use_efuse_xo_offset;
    /*!< Set to 0 to use the XO offset provided in xo_offset_x1000. Set to 1 to use XO offset programmed on WINC efuse.
    */
    uint8_t peer_mac_addr[6];
    /*!< Set peer address to send directed frames to a certain address.
    */
} tstrM2mAteTx;

/*!
 *@struct   tstrM2mAteRx
 *@brief    Used as data source in case of enabling RX test case
 */
typedef struct {
    uint8_t       channel_num;
    /*!< Channel number \ref tenuM2mAteChannels
     */
    uint8_t     use_pmu;
    /*!< This is 0 if PMU is not used otherwise it must be be 1 \ref tenuM2mAtePMUSetting
     */
    uint32_t     xo_offset_x1000;
    /*!< Signed XO offset value in PPM (Part Per Million) multiplied by 1000.
     */
    uint8_t     use_efuse_xo_offset;
    /*!< Set to 0 to use the XO offset provided in xo_offset_x1000. Set to 1 to use XO offset programmed on WINC efuse.
    */
    uint8_t    self_mac_addr[6];
    /*!< Set to the self mac address required to be overridden.
    */
    uint8_t    sa_mac_addr[6];
    /*!< Set to the source mac address expected to filter frames from.
    */
    uint8_t    mac_filter_en_da;
    /*!< Flag set to enable or disable reception with destination address as a filter.
    */
    uint8_t    mac_filter_en_sa;
    /*!< Flag set to enable or disable reception with source address as a filter.
    */
    uint8_t   override_self_mac_addr;
    /*!< Flag set to enable or disable self mac address feature.
    */
} tstrM2mAteRx;

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#ifdef __cplusplus
extern "C" {
#endif

/*!
@fn \
    int8_t m2m_ate_init(void);

@brief
    This function used to download ATE firmware from flash and start it

@return
    The function SHALL return 0 for success and a negative value otherwise.
*/
int8_t m2m_ate_init(uint32_t req_serial_number);

/*!
@fn \
    int8_t m2m_ate_deinit(void);

@brief
    De-Initialization of ATE firmware mode

@return
    The function SHALL return 0 for success and a negative value otherwise.
*/
int8_t m2m_ate_deinit(void);

/*!
@fn \
    int8_t m2m_ate_set_fw_state(uint8_t);

@brief
    This function used to change ATE firmware status from running to stopped or vice versa.

@param [in] u8State
        Required state of ATE firmware, one of \ref tenuM2mAteFwState enumeration values.
@return
    The function SHALL return 0 for success and a negative value otherwise.
\sa
    m2m_ate_init
*/
int8_t m2m_ate_set_fw_state(uint8_t);

/*!
@fn \
    int8_t m2m_ate_get_fw_state(uint8_t);

@brief
    This function used to return status of ATE firmware.

@return
    The function SHALL return status of ATE firmware, one of \ref tenuM2mAteFwState enumeration values.
\sa
    m2m_ate_init, m2m_ate_set_fw_state
*/
int8_t m2m_ate_get_fw_state(void);

/*!
@fn \
    uint32_t m2m_ate_get_tx_rate(uint8_t);

@brief
    This function used to return value of TX rate required by application developer.

@param [in] u8Index
        Index of required rate , one of \ref tenuM2mAteTxIndexOfRates enumeration values.
@return
    The function SHALL return 0 for in case of failure otherwise selected rate value.
\sa
    tenuM2mAteTxIndexOfRates
*/
uint32_t m2m_ate_get_tx_rate(uint8_t);

/*!
@fn \
    int8_t m2m_ate_get_tx_status(void);

@brief
    This function used to return status of TX test case either running or stopped.

@return
    The function SHALL return status of ATE firmware, 1 if TX is running otherwise 0.
\sa
    m2m_ate_start_tx, m2m_ate_stop_tx
*/
int8_t m2m_ate_get_tx_status(void);

/*!
@fn \
    int8_t m2m_ate_start_tx(tstrM2mAteTx *)

@brief
    This function used to start TX test case.

@param [in] strM2mAteTx
        Type of \ref tstrM2mAteTx, with the values required to enable TX test case. You must use \ref m2m_ate_init first.
@return
    The function SHALL return 0 for success and a negative value otherwise.
\sa
    m2m_ate_init, m2m_ate_stop_tx, m2m_ate_get_tx_status
*/
int8_t m2m_ate_start_tx(tstrM2mAteTx *);

/*!
@fn \
    int8_t m2m_ate_stop_tx(void)

@brief
    This function used to stop TX test case.

@return
    The function SHALL return 0 for success and a negative value otherwise.
\sa
    m2m_ate_init, m2m_ate_start_tx, m2m_ate_get_tx_status
*/
int8_t m2m_ate_stop_tx(void);

/*!
@fn \
    int8_t m2m_ate_get_rx_status(uint8_t);

@brief
    This function used to return status of RX test case either running or stopped.

@return
    The function SHALL return status of ATE firmware, 1 if RX is running otherwise 0.
\sa
    m2m_ate_start_rx, m2m_ate_stop_rx
*/
int8_t m2m_ate_get_rx_status(void);

/*!
@fn \
    int8_t m2m_ate_start_rx(tstrM2mAteRx *)

@brief
    This function used to start RX test case.

@param [in] strM2mAteRx
        Type of \ref tstrM2mAteRx, with the values required to enable RX test case. You must use \ref m2m_ate_init first.
@return
    The function SHALL return 0 for success and a negative value otherwise.
\sa
    m2m_ate_init, m2m_ate_stop_rx, m2m_ate_get_rx_status
*/
int8_t m2m_ate_start_rx(tstrM2mAteRx *);

/*!
@fn \
    int8_t m2m_ate_stop_rx(void)

@brief
    This function used to stop RX test case.

@return
    The function SHALL return 0 for success and a negative value otherwise.
\sa
    m2m_ate_init, m2m_ate_start_rx, m2m_ate_get_rx_status
*/
int8_t m2m_ate_stop_rx(void);

/*!
@fn \
    int8_t m2m_ate_read_rx_status(tstrM2mAteRxStatus *)

@brief
    This function used to read RX statistics from ATE firmware.

@param [out]    strM2mAteRxStatus
        Type of \ref tstrM2mAteRxStatus used to save statistics of RX test case. You must use \ref m2m_ate_start_rx first.
@return
    The function SHALL return 0 for success and a negative value otherwise.
\sa
    m2m_ate_init, m2m_ate_start_rx
*/
int8_t m2m_ate_read_rx_status(tstrM2mAteRxStatus *);

/*!
@fn \
    int8_t m2m_ate_set_dig_gain(double dGaindB)

@brief
    This function is used to set the digital gain

@param [in] double dGaindB
        The digital gain value required to be set.
@return
    The function SHALL return 0 for success and a negative value otherwise.
*/
int8_t m2m_ate_set_dig_gain(double dGaindB);

/*!
@fn \
    int8_t m2m_ate_get_dig_gain(double * pdGaindB)

@brief
    This function is used to get the digital gain

@param [out]    double * pdGaindB
        The retrieved digital gain value obtained from HW registers.
@return
    The function SHALL return 0 for success and a negative value otherwise.
*/
int8_t m2m_ate_get_dig_gain(double *pdGaindB);

/*!
@fn \
    int8_t m2m_ate_get_pa_gain(uint32_t *paGain)

@brief
    This function is used to get the pa gain

@param [out]    uint32_t *paGain
        The retrieved pa gain value obtained from HW registers.
@return
    The function SHALL return 0 for success and a negative value otherwise.
*/
int8_t m2m_ate_get_pa_gain(uint32_t *paGain);

/*!
@fn \
    int8_t m2m_ate_get_ppa_gain(uint32_t * ppaGain)

@brief
    This function is used to get the ppa gain

@param [out]    uint32_t * ppaGain
        The retrieved ppa gain value obtained from HW registers.
@return
    The function SHALL return 0 for success and a negative value otherwise.
*/
int8_t m2m_ate_get_ppa_gain(uint32_t *ppaGain);

/*!
@fn \
    int8_t m2m_ate_get_tot_gain(double * pTotGaindB)

@brief
    This function is used to get the total gain

@param [out]    double * pTotGaindB
        The retrieved total gain value obtained from calculations made based on the digital gain, pa and ppa gain values.
@return
    The function SHALL return 0 for success and a negative value otherwise.
*/
int8_t m2m_ate_get_tot_gain(double *pTotGaindB);


#ifdef __cplusplus
}
#endif

#endif /* _M2M_CONFIG_MODE_H_ */

#endif //_M2M_ATE_FW_