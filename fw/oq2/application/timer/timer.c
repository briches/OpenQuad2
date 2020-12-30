/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\timer\timer.c                       /
 * Project: OQ2                                                                                    /
 * Created Date: Friday, December 25th 2020, 8:14:20 am                                            /
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


#include "timer.h"
#include "cmsis_os.h"
#include "timer_config.h"
#include "arm_math.h"

static volatile float match_count = 0;

#include "debug_log.h"

#define debug_error(fmt, ...)           debug_error(MAIN_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(MAIN_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(MAIN_MODULE_ID, fmt, ##__VA_ARGS__)


void timer_time_start()
{   
    HAL_LPTIM_Counter_Start_IT(&hlptim1, TC_TIMER_MATCH_VALUE);
}

/**
 * @brief Callback when the counter reaches the period value
 * 
 * The clock is PCLK1, with divider 128 -> 1074218.75 Hz
 * 
 * So F_match = PCLK1 / 128 / period
 * 
 */
void timer_lptim_match_callback()
{
    // UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    match_count++;
    // __HAL_LPTIM_RESET_COUNTER(&hlptim1);
    // taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

static volatile float last = 0;

/**
 * @brief Get the elapsed time in seconds since "start" was called
 * 
 * @return float time in seconds
 */
float timer_get_elapsed()
{
    if(HAL_LPTIM_GetState(&hlptim1) != HAL_LPTIM_STATE_READY)
        return 0;

    int32_t ticks = HAL_LPTIM_ReadCounter(&hlptim1);
    // int32_t ticks1 = HAL_LPTIM_ReadCounter(&hlptim1);

    // if(abs(ticks1 - ticks) > 1)
    // {
    //     int i = 0;
    //     i += 1;
    // }

    float res = (match_count * TC_TIMER_MATCH_PERIOD_S) + ((float)ticks * TC_TIMER_TICK_PERIOD_S);

    // if((res - last) > 0.01 && last != 0)
    // {
    //     int i = 0;
    //     i += 1;
    // }
    
    // last = res;

    return res;
}


uint32_t sys_now()
{
    return (uint32_t) (1000.0 * timer_get_elapsed());
}