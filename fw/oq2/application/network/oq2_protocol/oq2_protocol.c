/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\network\oq2_protocol\oq2_protocol.c /
 * Project: OQ2                                                                                    /
 * Created Date: Wednesday, December 30th 2020, 9:26:00 am                                         /
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

#include "FreeRTOS.h"
#include <string.h>
#include "timer.h"
#include "motors.h"
#include "oq2_protocol.h"
#include "oq2p_packet_builders.h"

#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(OQ2_PROTOCOL_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(OQ2_PROTOCOL_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(OQ2_PROTOCOL_MODULE_ID, fmt, ##__VA_ARGS__)




/** Receive buffer definition. */
static uint8_t* oq2p_in_buffer;
static uint8_t* oq2p_out_buffer;
static volatile bool m_init = false;

static char m_hello[] = "Hey there fancy pants.";

static int8_t m_sock = -1;

static uint32_t bytes_sent = 0;
static uint32_t bytes_received = 0;

/**
 * @brief Internal function to call socket layer send function
 */
__always_inline
static void _op2p_send(void* buf, uint16_t num)
{
    if (send(m_sock, buf, num, 0) == SOCK_ERR_NO_ERROR)
    {
        bytes_sent += num;
    }
}

/**
 * @brief Internal function to call socket layer receive function
 *
 * @param buf
 * @param num
 */
__always_inline
static void _op2p_receive(void* buf, uint16_t num)
{
    recv(m_sock, buf, num, 0);
}


static void application_oq2p_control_message_handler(oq2p_control_mid_t msg_id, oq2p_command_request_t cr, uint8_t* pdata, uint16_t data_length)
{
    uint8_t line_length = (data_length < 8) ? data_length : 8;

    // if()
    // debug_print_buffer(pdata, data_length, 0, line_length);

    switch (msg_id)
    {
    case OQ2P_MID_ARM:
    {
        // Handle Command to change motor armed states
        if (cr == OQ2P_COMMAND)
        {
            uint8_t arm = *(pdata++);
            uint8_t index = *(pdata++);

            if ((index > 4 && index != 0xFF) || index == 0)
            {
                debug_error("Incorrect motor index - expected 1, 2, 3, 4 or 255, got %u", index);
                return;
            }
            if (arm > 1)
            {
                debug_error("ARM should either be 1 or 0, got %u", arm);
                return;
            }

            // Correct arm command received. Handle accordingly.
            debug_printf("Command to change arm state to %u on index %u", arm, index);

            if (index == 0xFF)
            {
                motor_controllers_set_arm_state(1, arm);
                motor_controllers_set_arm_state(2, arm);
                motor_controllers_set_arm_state(3, arm);
                motor_controllers_set_arm_state(4, arm);
            }
            else
                motor_controllers_set_arm_state(index, arm);

        }
        // Handle Request to get current motor armed states
        else if(cr == OQ2P_REQUEST)
        {
            motor_arm_t arm_info[4] = {motor_controllers_get_arm_state(1), 
                                    motor_controllers_get_arm_state(2), 
                                    motor_controllers_get_arm_state(3), 
                                    motor_controllers_get_arm_state(4)};

            int32_t len = oq2p_msg_build_arm_response(oq2p_out_buffer, OQ2P_TX_BUFSIZE, arm_info);

            debug_printf("Sending armed state info response:");
            debug_print_buffer(oq2p_out_buffer, len, 0, 8);

            if(len > 0)
            {
                _op2p_send(oq2p_out_buffer, len);
            }
        }

    } break; // case OQ2P_MID_ARM

    }

}

/**
 * @brief Parse the incoming message and dispatch it to the correct handler based on
 * the message class
 *
 * @param buf pointer to the packet data start
 * @param packet_len length of the whole packet ()
 */
static void oq2p_parse_message(uint8_t* buf, uint16_t packet_len)
{
    uint8_t start, class, msg_id, request, length_msb, length_lsb, end;
    uint8_t* bufstart = buf;

    end = buf[packet_len - 1];
    start = *(buf++);

    if (start != OQ2P_START && end != OQ2P_END)
    {
        debug_error("Invalid start byte (0x%02X) or end byte (0x%02X)", start, end);
        return;
    }
    debug_printf("received OQ2P message");

    class = *(buf++);
    msg_id = *(buf++);
    request = *(buf++);
    length_msb = *(buf++);
    length_lsb = *(buf++);

    uint16_t length = (length_msb << 8) | (length_lsb);

    if (length != packet_len - OQ2P_OVERHEAD_LEN)
    {
        debug_error("Invalid packet length");
        return;
    }

    switch (class)
    {
    case OQ2P_CLASS_CONTROL:
    {
        application_oq2p_control_message_handler(msg_id, request, buf, length);
    } break;
    }
}

/**
 * @brief Callback for socket connected event
 *
 * @param sock
 */
void oq2p_connect_callback(int8_t sock)
{
    if (!m_init)
        return;

    m_sock = sock;
    debug_printf("oq2p_connect_callback() with int8_t %d", sock);

    _op2p_receive(oq2p_in_buffer, OQ2P_RX_BUFSIZE);
    _op2p_send(m_hello, (uint16_t)sizeof(m_hello));
}

/**
 * @brief Callback for socket disconnected event
 *
 * @param sock
 */
void oq2p_disconnect_callback(int8_t sock)
{
    if (!m_init)
        return;

    debug_printf("oq2p_disconnect_callback() from int8_t %d", sock);

    if (sock != m_sock)
        return;

    m_sock = -1;
}

#if !defined(CONFIG_USE_ASYNC_API)
void oq2p_receive_callback(int8_t sock, tstrSocketRecvMsg* p_recv)
{
    if (sock != m_sock)
        return;

    if (!m_init)
        return;

    if (p_recv && p_recv->s16BufferSize > 0)
    {
        // debug_printf("int8_t_cb: recv success!");
        bytes_received += p_recv->s16BufferSize;
        debug_printf("received %u B", p_recv->s16BufferSize);

        oq2p_parse_message(p_recv->pu8Buffer, p_recv->s16BufferSize);

        // debug_printf("%s", p_recv->pu8Buffer);
        // debug_print_buffer(p_recv->pu8Buffer, p_recv->s16BufferSize, 0, 16);
    }
    else
    {
        debug_printf("int8_t_cb: recv error!");
        close(sock);
        sock = -1;
    }

    memset(oq2p_in_buffer, 0x00, OQ2P_RX_BUFSIZE);
    _op2p_receive(oq2p_in_buffer, OQ2P_RX_BUFSIZE);
}
#endif


void oq2p_init()
{
    oq2p_in_buffer = pvPortMalloc(OQ2P_RX_BUFSIZE);
    oq2p_out_buffer = pvPortMalloc(OQ2P_TX_BUFSIZE);

    if (oq2p_in_buffer == NULL)
    {
        debug_error("Could not get memory for oq2p_in_buffer");
    }
    else if (oq2p_out_buffer == NULL)
    {
        debug_error("Could not get memory for oq2p_out_buffer");
    }
    else
    {
        m_init = true;
    }

    debug_printf("OQ2P in buffer is at 0x%08X (%u bytes)", oq2p_in_buffer, OQ2P_RX_BUFSIZE);
    debug_printf("OQ2P out buffer is at 0x%08X (%u bytes)", oq2p_out_buffer, OQ2P_TX_BUFSIZE);
}


uint32_t op2p_get_sent_bytes()
{
    return bytes_sent;
}

uint32_t op2p_get_received_bytes()
{
    return bytes_received;
}

