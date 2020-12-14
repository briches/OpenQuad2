/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\DebugLog\debug_log.h                    /
 * Project: OQ2                                                                                    /
 * Created Date: Saturday, December 12th 2020, 7:36:14 am                                          /
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


#ifndef DEBUG_LOG_H_
#define DEBUG_LOG_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>

#include "module_ids.h"

// VT-100 Terminal Codes -------------------------------------------------------------------------

#define VT100_RESET                     "\x1B[0m"
#define VT100_CLEAR                     "\x1B[2J"

#define VT100_TEXT_BLACK                "\x1B[2;30m"
#define VT100_TEXT_RED                  "\x1B[2;31m"
#define VT100_TEXT_GREEN                "\x1B[2;32m"
#define VT100_TEXT_YELLOW               "\x1B[2;33m"
#define VT100_TEXT_BLUE                 "\x1B[2;34m"
#define VT100_TEXT_MAGENTA              "\x1B[2;35m"
#define VT100_TEXT_CYAN                 "\x1B[2;36m"
#define VT100_TEXT_WHITE                "\x1B[2;37m"

#define VT100_TEXT_BRIGHT_BLACK         "\x1B[1;30m"
#define VT100_TEXT_BRIGHT_RED           "\x1B[1;31m"
#define VT100_TEXT_BRIGHT_GREEN         "\x1B[1;32m"
#define VT100_TEXT_BRIGHT_YELLOW        "\x1B[1;33m"
#define VT100_TEXT_BRIGHT_BLUE          "\x1B[1;34m"
#define VT100_TEXT_BRIGHT_MAGENTA       "\x1B[1;35m"
#define VT100_TEXT_BRIGHT_CYAN          "\x1B[1;36m"
#define VT100_TEXT_BRIGHT_WHITE         "\x1B[1;37m"

#define VT100_BG_BLACK                  "\x1B[24;40m"
#define VT100_BG_RED                    "\x1B[24;41m"
#define VT100_BG_GREEN                  "\x1B[24;42m"
#define VT100_BG_YELLOW                 "\x1B[24;43m"
#define VT100_BG_BLUE                   "\x1B[24;44m"
#define VT100_BG_MAGENTA                "\x1B[24;45m"
#define VT100_BG_CYAN                   "\x1B[24;46m"
#define VT100_BG_WHITE                  "\x1B[24;47m"

#define VT100_BG_BRIGHT_BLACK           "\x1B[4;40m"
#define VT100_BG_BRIGHT_RED             "\x1B[4;41m"
#define VT100_BG_BRIGHT_GREEN           "\x1B[4;42m"
#define VT100_BG_BRIGHT_YELLOW          "\x1B[4;43m"
#define VT100_BG_BRIGHT_BLUE            "\x1B[4;44m"
#define VT100_BG_BRIGHT_MAGENTA         "\x1B[4;45m"
#define VT100_BG_BRIGHT_CYAN            "\x1B[4;46m"
#define VT100_BG_BRIGHT_WHITE           "\x1B[4;47m"


void debug_log_init(void);
void debug_log_tx_completed_callback(void);

void debug_timestamp_enable(bool enable);

void debug_printf(module_id_t source, const char *p_format, ...);

void debug_error(module_id_t source, const char *p_format, ...);

void debug_print_buffer(module_id_t source, 
                        const uint8_t *p_buffer, 
                        unsigned int length,
                        uint32_t start_address, 
                        int columns);

#endif