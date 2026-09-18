/********************************************************************************************************
 * @file    app_prj_config.h
 *
 * @brief   This is the header file for Telink RISC-V MCU
 *
 * @author  2.4G Group
 * @date    2026
 *
 * @par     Copyright (c) 2026, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *
 *******************************************************************************************************/
#pragma once

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#include "tl_common.h"
#include "../app_config.h"
#include "tl_string.h"
#include "../../common/app_spp.h"


#define clock_time stimer_get_tick

#define RF_MODE_APP_FUN_ENABLE          1


/**
 * @brief   Pairing procedure timeout (milliseconds)
 *
 * Defines the maximum duration allowed for the pairing procedure before
 * it is considered failed and aborted.
 */
#define    P24G_PAIR_TIMEOUT_MS              60000


/**
 * @brief   Reconnection procedure timeout (milliseconds)
 *
 * Defines the maximum time allowed for attempting reconnection with a
 * previously paired device.
 */
#define    P24G_RECONN_TIMEOUT_MS            60000


#define TPSLL_PM_OPTM_LEVEL1_START_TIME_S       10 //0 for disable
#define TPSLL_PM_OPTM_LEVEL2_START_TIME_S       (TPSLL_PM_OPTM_LEVEL1_START_TIME_S + 10) //0 for disable
#define TPSLL_PM_OPTM_LEVEL3_START_TIME_S       (TPSLL_PM_OPTM_LEVEL2_START_TIME_S + 40) //0 for disable

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
