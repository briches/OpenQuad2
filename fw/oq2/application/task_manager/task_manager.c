/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\application\task_manager\task_manager.c         /
 * Project: OQ2                                                                                    /
 * Created Date: Tuesday, December 15th 2020, 6:36:53 pm                                           /
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


#include "task_manager.h"
#include "app_config.h"
#include "debug_log.h"
#include <string.h>
#include "main.h"

#define debug_error(fmt, ...)           debug_error(TASK_MANAGER_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(TASK_MANAGER_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(TASK_MANAGER_MODULE_ID, fmt, ##__VA_ARGS__)

/**
 * @brief  Function implementing the stability task thread.
 * @param  argument: Not used
 * @retval None
 */
void task_manager_thread(void* argument)
{

    char * runtime_stat_buf = pvPortMalloc(300);
    char * task_info_buf = pvPortMalloc(300);

    for(;;)
    {
        memset(runtime_stat_buf, 0x00, 300);
        memset(task_info_buf, 0x00, 300);

        vTaskGetRunTimeStats(runtime_stat_buf);
        vTaskList(task_info_buf);
        uint32_t free_heap = xPortGetFreeHeapSize();
        uint32_t min_heap = xPortGetMinimumEverFreeHeapSize();

        float percent_free = 100.0f*((float)free_heap);
        percent_free /= configTOTAL_HEAP_SIZE;
        
        #if (APP_CONFIG_TASK_MANAGER_PRINT_OUTPUT == APP_CONFIG_ENABLED)
        debug_printf("Free heap bytes = %u / %u (%u.%u%%)", 
                                                        free_heap, 
                                                        configTOTAL_HEAP_SIZE, 
                                                        (int)percent_free, (int)(10 * percent_free) % 10);
        debug_printf("Min ever free heap %u", min_heap);
        debug_printf("\r\nTask Runtime Stats:\r\nName\tExecutions\tUpTime\r\n%s\r\n", runtime_stat_buf);
        debug_printf("\r\nName\tStatus\tPrio\tRemainingStack\tTaskNum\r\n%s\r\n", task_info_buf);
        #endif

        osDelay(TASK_MANAGER_THREAD_PERIOD);
    }
}