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

#include "oq2_protocol.h"
#include "m2m_wifi_ex.h"

#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(OQ2_PROTOCOL_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(OQ2_PROTOCOL_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(OQ2_PROTOCOL_MODULE_ID, fmt, ##__VA_ARGS__)

#include "FreeRTOS.h"
#include "timer.h"


/** Receive buffer definition. */
static uint8_t oq2p_in_buffer[MAIN_WIFI_M2M_BUFFER_SIZE];

/** Message format definitions. */
typedef struct s_msg_wifi_product {
    uint8_t name[9];
} t_msg_wifi_product;

/** Message format declarations. */
static t_msg_wifi_product msg_wifi_product = {
    .name = MAIN_WIFI_M2M_PRODUCT_NAME,
};

static char m_hello[] = "Hey there fancy pants.";

static int8_t m_sock = -1;


// void oq2p_connect_callback(int8_t sock)
// {
//     m_sock = sock;
//     debug_printf("oq2p_connect_callback() with int8_t %d", sock);

//     recv(m_sock, oq2p_in_buffer, sizeof(oq2p_in_buffer), 0);
//     send(m_sock, m_hello, (uint16_t) sizeof(m_hello), 0);
// }

// void oq2p_disconnect_callback(int8_t sock)
// {
//     if(sock != m_sock)
//         return;

//     m_sock = -1;
    
//     debug_printf("oq2p_disconnect_callback() from int8_t %d", sock);
// }

// void oq2p_receive_callback(int8_t sock)
// {
//     if(sock != m_sock)
//         return;

//     // if (p_recv && p_recv->s16BufferSize > 0) 
//     // {
//     //     debug_printf("int8_t_cb: recv success!");
//     // }
//     // else 
//     // {
//     //     debug_printf("int8_t_cb: recv error!");
//     //     close(sock);
//     //     sock = -1;
//     // }

//     // memset(oq2p_in_buffer, 0x00, sizeof(oq2p_in_buffer))
//     // recv(m_sock, oq2p_in_buffer, sizeof(oq2p_in_buffer), 0);

//     // debug_printf("received %s", p_recv->pu8Buffer);
//     close(m_sock);
// }


