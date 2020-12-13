/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\DebugLog\module_ids.h                   /
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


#ifndef MODULE_IDS_H_
#define MODULE_IDS_H_

typedef enum
{
   MAIN_MODULE_ID,
   FREERTOS_MODULE_ID,
   SENSOR_MODULE_ID,
   KINEMATICS_MODULE_ID,
   NUM_MODULES
} module_id_t;



#define DEBUG_CYAN_HIGHLIGHT_SELECT    KINEMATICS_MODULE_ID
#define DEBUG_YELLOW_HIGHLIGHT_SELECT  SENSOR_MODULE_ID

#endif