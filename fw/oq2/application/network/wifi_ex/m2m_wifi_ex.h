/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\network\wifi_ex\m2m_wifi_ex.h       /
 * Project: OQ2                                                                                    /
 * Created Date: Wednesday, December 30th 2020, 7:06:34 am                                         /
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
 * \brief Wireless Link Controller Driver Declarations.
 *
 * Copyright (c) 2019 Microchip Technology Inc. and its subsidiaries.
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
#ifndef M2M_WIFI_EX_H_INCLUDED
#define M2M_WIFI_EX_H_INCLUDED
#include <m2m_wifi.h>

typedef void (*m2m_wifi_callback_t)(void *arg);

int8_t os_m2m_wifi_init(tstrWifiInitParam *param);
int8_t os_m2m_wifi_connect(char *pcSsid, uint8_t u8SsidLen, uint8_t u8SecType, void *pvAuthInfo, uint16_t u16Ch);
int8_t os_m2m_wifi_enable_ap(tstrM2MAPConfig *ap);
int8_t m2m_wifi_request_callback_ex(m2m_wifi_callback_t callback, void *arg);
int8_t m2m_wifi_request_dhcp_client_ex(void);
int8_t m2m_wifi_enable_ap_ex(const tstrM2MAPConfig* pstrM2MAPConfig);
int8_t m2m_wifi_disable_ap_ex(void);

// Macros to alias all other APIs to the _ex versions.
#define m2m_wifi_download_mode_ex			m2m_wifi_download_mode
#define m2m_wifi_deinit_ex					m2m_wifi_deinit
#define m2m_wifi_handle_events_ex			m2m_wifi_handle_events
#define m2m_wifi_default_connect_ex			m2m_wifi_default_connect
#define m2m_wifi_disconnect_ex				m2m_wifi_disconnect
#define m2m_wifi_start_provision_mode_ex	m2m_wifi_start_provision_mode
#define m2m_wifi_stop_provision_mode_ex		m2m_wifi_stop_provision_mode
#define m2m_wifi_get_connection_info_ex		m2m_wifi_get_connection_info
#define m2m_wifi_wps_ex						m2m_wifi_wps
#define m2m_wifi_wps_disable_ex				m2m_wifi_wps_disable
#define m2m_wifi_p2p_ex						m2m_wifi_p2p
#define m2m_wifi_p2p_disconnect_ex			m2m_wifi_p2p_disconnect
#define m2m_wifi_set_static_ip_ex			m2m_wifi_set_static_ip
#define m2m_wifi_request_dhcp_server_ex		m2m_wifi_request_dhcp_server
#define m2m_wifi_set_scan_options_ex		m2m_wifi_set_scan_options
#define m2m_wifi_set_scan_region_ex			m2m_wifi_set_scan_region
#define m2m_wifi_request_scan_ex			m2m_wifi_request_scan
#define m2m_wifi_get_num_ap_found_ex		m2m_wifi_get_num_ap_found
#define m2m_wifi_req_scan_result_ex			m2m_wifi_req_scan_result
#define m2m_wifi_req_curr_rssi_ex			m2m_wifi_req_curr_rssi
#define m2m_wifi_get_otp_mac_address_ex		m2m_wifi_get_otp_mac_address
#define m2m_wifi_get_mac_address_ex			m2m_wifi_get_mac_address
#define m2m_wifi_set_sleep_mode_ex			m2m_wifi_set_sleep_mode
#define m2m_wifi_request_sleep_ex			m2m_wifi_request_sleep
#define m2m_wifi_get_sleep_mode_ex			m2m_wifi_get_sleep_mode
#define m2m_wifi_req_client_ctrl_ex			m2m_wifi_req_client_ctrl
#define m2m_wifi_req_server_init_ex			m2m_wifi_req_server_init
#define m2m_wifi_set_device_name_ex			m2m_wifi_set_device_name
#define m2m_wifi_set_lsn_int_ex				m2m_wifi_set_lsn_int
#define m2m_wifi_enable_monitoring_mode_ex	m2m_wifi_enable_monitoring_mode
#define m2m_wifi_disable_monitoring_mode_ex	m2m_wifi_disable_monitoring_mode
#define m2m_wifi_send_wlan_pkt_ex			m2m_wifi_send_wlan_pkt
#define m2m_wifi_send_ethernet_pkt_ex		m2m_wifi_send_ethernet_pkt
#define m2m_wifi_set_sytem_time_ex			m2m_wifi_set_sytem_time
#define m2m_wifi_set_cust_InfoElement_ex	m2m_wifi_set_cust_InfoElement
#define m2m_wifi_enable_mac_mcast_ex		m2m_wifi_enable_mac_mcast
#define m2m_wifi_set_receive_buffer_ex		m2m_wifi_set_receive_buffer

int8_t os_m2m_wifi_download_mode(void);
int8_t os_m2m_wifi_deinit(void* arg);
int8_t os_m2m_wifi_default_connect(void);
int8_t os_m2m_wifi_disconnect(void);
int8_t os_m2m_wifi_start_provision_mode(tstrM2MAPConfig* pstrAPConfig, char* pcHttpServerDomainName, uint8_t bEnableHttpRedirect);
int8_t os_m2m_wifi_stop_provision_mode(void);
int8_t os_m2m_wifi_get_connection_info(void);
int8_t os_m2m_wifi_set_mac_address(uint8_t* au8MacAddress);
int8_t os_m2m_wifi_wps(uint8_t u8TriggerType, const char* pcPinNumber);
int8_t os_m2m_wifi_wps_disable(void);
int8_t os_m2m_wifi_p2p(uint8_t u8Channel);
int8_t os_m2m_wifi_p2p_disconnect(void);
int8_t os_m2m_wifi_disable_ap(void);
int8_t os_m2m_wifi_ap_get_assoc_info(void);
int8_t os_m2m_wifi_set_static_ip(tstrM2MIPConfig* pstrStaticIPConf);
int8_t os_m2m_wifi_set_scan_options(uint8_t u8NumOfSlot, uint8_t u8SlotTime);
int8_t os_m2m_wifi_set_scan_region(uint8_t ScanRegion);
int8_t os_m2m_wifi_request_scan(uint8_t ch);
int8_t os_m2m_wifi_request_scan_ssid(uint8_t ch, char* pcssid);
uint8_t os_m2m_wifi_get_num_ap_found(void);
int8_t os_m2m_wifi_req_scan_result(uint8_t index);
int8_t os_m2m_wifi_req_curr_rssi(void);
int8_t os_m2m_wifi_get_otp_mac_address(uint8_t* pu8MacAddr, uint8_t* pu8IsValid);
int8_t os_m2m_wifi_get_mac_address(uint8_t* pu8MacAddr);
int8_t os_m2m_wifi_set_sleep_mode(uint8_t PsTyp, uint8_t BcastEn);
int8_t os_m2m_wifi_request_sleep(uint32_t u32SlpReqTime);
int8_t os_m2m_wifi_req_client_ctrl(uint8_t cmd);
int8_t os_m2m_wifi_req_server_init(uint8_t ch);
int8_t os_m2m_wifi_set_device_name(uint8_t* pu8DeviceName, uint8_t u8DeviceNameLength);
int8_t os_m2m_wifi_set_lsn_int(tstrM2mLsnInt* pstrM2mLsnInt);
int8_t os_m2m_wifi_enable_monitoring_mode(tstrM2MWifiMonitorModeCtrl* pstrMtrCtrl, uint8_t* pu8PayloadBuffer, uint16_t u16BufferSize, uint16_t u16DataOffset);
int8_t os_m2m_wifi_disable_monitoring_mode(void);
int8_t os_m2m_wifi_send_wlan_pkt(uint8_t* pu8WlanPacket, uint16_t u16WlanHeaderLength, uint16_t u16WlanPktSize);
int8_t os_m2m_wifi_send_ethernet_pkt(uint8_t* pu8Packet, uint16_t u16PacketSize);
int8_t os_m2m_wifi_set_sytem_time(uint32_t u32UTCSeconds);
int8_t os_m2m_wifi_set_cust_InfoElement(uint8_t* pau8M2mCustInfoElement);
int8_t os_m2m_wifi_enable_mac_mcast(uint8_t* pu8MulticastMacAddress, uint8_t u8AddRemove);
int8_t os_m2m_wifi_set_receive_buffer(void* pvBuffer, uint16_t u16BufferLen);
int8_t os_m2m_wifi_set_control_ifc(uint8_t u8IfcId);
int8_t os_m2m_wifi_send_ethernet_pkt_ifc1(uint8_t* pu8Packet, uint16_t u16PacketSize);
uint8_t os_m2m_wifi_get_sleep_mode(void);

uint8_t os_m2m_socket_init(void);


#endif /* M2M_WIFI_EX_H_INCLUDED */
