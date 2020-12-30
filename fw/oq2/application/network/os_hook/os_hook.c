/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\network\os_hook\os_hook.c           /
 * Project: OQ2                                                                                    /
 * Created Date: Wednesday, December 30th 2020, 7:06:46 am                                         /
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

#include <os_hook.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
#include <lwip/tcpip.h>

#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(OS_HOOK_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(OS_HOOK_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(OS_HOOK_MODULE_ID, fmt, ##__VA_ARGS__)

extern void winc_netif_tx_from_queue(hif_msg_t* msg);
extern uint8_t m2m_wifi_handle_events(void* p);

/* HIF thread. */
TaskHandle_t hif_thread;

/* HIF thread queue. */
QueueHandle_t hif_queue = NULL;

/* HIF thread binary semaphore. */
static SemaphoreHandle_t hif_notify_sem;

/* HIF initialized. */
static uint8_t has_init = 0;
extern uint8_t g_WifiConnSent;

/**
 * \brief HIF thread entry function.
 */
uint32_t os_hif_task_counter;
uint32_t os_hif_task_msg_rec_counter;
uint32_t os_hif_task_msg_post_counter;
uint32_t os_hif_task_msg_post_counter_rx;
uint32_t os_hif_task_set_rx_done_counter;
uint32_t os_hif_task_false_intrpt_counter;

static void os_hif_task(void* pv)
{
    while (1) {
        hif_msg_t msg = { 0 };
        os_hif_task_counter++;

        msg.id = 0;
        if (xQueueReceive(hif_queue, (void*)&msg, portMAX_DELAY) != pdFALSE)
        {
            os_hif_task_msg_rec_counter++;
            /* WiFi init. */
            if (msg.id == MSG_START) {
                if (msg.handler)
                    msg.handler(msg.priv);
            }

            /* Incoming data packet. */
            else if (msg.id == MSG_RX) {
                m2m_wifi_handle_events(NULL);
                // debug_printf("rx from q");
            }
            /* Outgoing data packet. */
            else if ((msg.id == MSG_TX_STA) || (msg.id == MSG_TX_AP)) {
                winc_netif_tx_from_queue(&msg);
                // debug_printf("tx from q");
            }

            /* WiFi command. */
            else if (msg.id == MSG_CMD) {

                if (msg.handler)
                    msg.handler(msg.priv);

                // debug_printf("cmd from q");

            }

            /* Error. */
            else {
                debug_error("Warning: Wrong id  msg id %ld \r\n", msg.id);
            }
        }
    }
}


/**
 * \brief Put interrupt request in netif queue for later processing.
 */
void os_hook_isr(void)
{
    signed portBASE_TYPE woken = pdFALSE;
    hif_msg_t msg;

    msg.id = MSG_RX;
    msg.pbuf = NULL;
    msg.payload_size = 0;
    msg.payload = NULL;
    if (xQueueSendFromISR(hif_queue, &msg, &woken) != pdTRUE) {
        debug_error("os_hook_isr: queue full error!\n");
    }
    else {
        os_hif_task_msg_post_counter_rx++;
        portEND_SWITCHING_ISR(woken);
    }
    	// debug_printf("os_hook_isr: posted isr - os_hif_task_msg_post_counter_rx = %d\n", os_hif_task_msg_post_counter_rx);
}

/**
 * \brief Initialize netif thread.
 */
void os_hook_init(void)
{
    if (!has_init) {
        vSemaphoreCreateBinary(hif_notify_sem);
        xSemaphoreTake(hif_notify_sem, portMAX_DELAY);
        hif_queue = xQueueCreate(100, sizeof(hif_msg_t));

        //		xTaskCreate(os_hif_task, (const void *)"WiFiHIF", 1024, NULL, (configMAX_PRIORITIES - 1), &hif_thread);
        xTaskCreate(os_hif_task, (const void*)"WiFiHIF", (1024 * 4), NULL, (configMAX_PRIORITIES - 1), &hif_thread);
        has_init = 1;
    }
}

void os_hook_set_handle(void* task)
{
    hif_thread = task;
}

/**
 * \brief Run handler function in the netif thread context and return immediately.
 */
uint8_t os_hook_dispatch_no_wait(wifi_task_cb handler, void* p)
{
    hif_msg_t msg;
    if (hif_thread == xTaskGetCurrentTaskHandle()) {
        handler(p);
    }
    else {
        msg.id = MSG_CMD;
        msg.handler = handler;
        msg.priv = p;
        os_hif_task_msg_post_counter++;
        return xQueueSend(hif_queue, (void*)&msg, portMAX_DELAY);
    }
    return 0;
}

/**
 * \brief Run handler function in the netif thread context and wait for function return.
 */
void os_hook_dispatch_wait(wifi_task_cb handler, struct params_dispatch* p, void* pv)
{
    hif_msg_t msg;
    if (hif_thread == xTaskGetCurrentTaskHandle()) {
        p->signal_semaphore = 0;
        handler(pv);
    }
    else {
        p->signal_semaphore = 1;
        msg.id = MSG_CMD;
        msg.handler = handler;
        msg.priv = pv;
        os_hif_task_msg_post_counter++;
        xQueueSend(hif_queue, (void*)&msg, portMAX_DELAY);

        xSemaphoreTake(hif_notify_sem, portMAX_DELAY);
    }
}

/**
 * \brief Start WiFi in the netif thread context and wait for function return.
 */
void os_hook_send_start(wifi_task_cb handler, struct params_dispatch* p, void* pv)
{
    hif_msg_t msg;

    p->signal_semaphore = 1;
    msg.id = MSG_START;
    msg.handler = handler;
    msg.priv = pv;
    os_hif_task_msg_post_counter++;
    xQueueSend(hif_queue, (void*)&msg, portMAX_DELAY);
    xSemaphoreTake(hif_notify_sem, portMAX_DELAY);
}

/**
 * \brief Release lock to notify calling thread that netif job is done.
 */
void os_hook_notify(void)
{
    xSemaphoreGive(hif_notify_sem);
}
