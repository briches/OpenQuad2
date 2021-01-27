/*
 * File: c:\Users\Brandon\Downloads\en.stsw-esc002v1\STSW-ESC002V1\Middlewares\ST\MC_6Step_Lib\Inc\stm32_nucleo_boards.h/
 * Project: OQ2                                                                                    /
 * Created Date: Wednesday, January 6th 2021, 2:45:54 pm                                           /
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
 ******************************************************************************
 * @file    stm32_nucleo_boards.h
 * @author  IPC Rennes
 * @version V2.0.0
 * @date    13-December-2017
 * @brief   This file provides the interface between the MC-lib and STM Nucleo
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#if defined(STEVAL_ESC002V1)
    #if defined(STM32F031x6)

        #include "MC_SixStep_param_32F0.h"
        #include "STEVAL-ESC002V1.h"

    #endif
#endif

#if defined(ESC_STSPIN32F0A)
#if defined(STM32F031x6)
#include "MC_SixStep_param_32F0.h"
#include "ESC-STSPIN32F0A.h"
#endif
#endif

#if defined(STEVAL_SPIN3202)
#if defined(STM32F031x6)
#include "MC_SixStep_param_32F0.h"
#include "STEVAL-SPIN3202.h"
#endif
#endif

#if defined(IHM07M1)
#if defined(STM32F030x8)
#include "MC_SixStep_param_F030.h"
#include "stm32F030_nucleo_ihm07m1.h"
#endif
#if defined(STM32F103xB)
#include "MC_SixStep_param_F103.h"
#include "stm32F103_nucleo_ihm07m1.h"
#endif
#if defined(STM32F302x8)
#include "MC_SixStep_param_F302.h"
#include "stm32F302_nucleo_ihm07m1.h"
#endif
#if defined(STM32F401xE)
#include "MC_SixStep_param_F401.h"
#include "stm32F401_nucleo_ihm07m1.h"
#endif
#endif




