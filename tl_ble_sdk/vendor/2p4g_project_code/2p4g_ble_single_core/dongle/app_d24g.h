/********************************************************************************************************
 * @file    app_d24g.h
 *
 * @brief   This is the header file for Telink RISC-V MCU
 *
 * @author  2.4G Group
 * @date    2025
 *
 * @par     Copyright (c) 2025, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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
#ifndef __APP_D24G_H__
#define __APP_D24G_H__

#include "app_prj_config.h"
#include "../app_common_config.h"

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

/**
 * @brief     Initialize user-side 2.4GHz application module
 *
 * This function performs user-level initialization for the 2.4GHz
 * application layer. It is typically responsible for initializing
 * application context, registering callbacks, and preparing resources
 * required before the 2.4G stack starts running.
 *
 * @param[in] none
 *
 * @return    none
 *
 * @note      This function is usually called during system startup.
 */
void pp_d24g_user_init(void);


/**
 * @brief     Normal keyboard report structure
 *
 * This structure describes a standard keyboard report containing
 * control keys, report length, and normal key values.
 *
 * @param ctrl   Control key bitmap (e.g. Ctrl, Shift, Alt)
 * @param len    Length of valid key data
 * @param nk     Normal keyboard key array (up to 6 keys)
 */
typedef struct
{
    unsigned char ctrl;
    unsigned char len;
    unsigned char nk[6];    /**< Normal keyboard keys */
} normal_key_t;


/**
 * @brief     Application context for 2.4GHz device
 *
 * This structure stores runtime context information related to
 * the 2.4GHz connection, pairing state, peer device information,
 * and current operating parameters.
 *
 * @param peer_tag         Tag or identifier of the peer device
 * @param paired_dev_type  Type of the paired peer device
 * @param mac              Local device MAC address
 * @param peer_mac         Peer device MAC address
 * @param report_rate      Current report rate setting
 * @param dev_status       Current device status
 */
typedef struct {
    uint32_t            usb_desc_tick;  
    uint16_t            peer_tag;
    uint8_t             paired_dev_type;
    uint8_t             mac[MAC_ADDR_LEN];
    uint8_t             peer_mac[MAC_ADDR_LEN];
    report_rate_t       report_rate;
    tpsll_dev_status_e  dev_status;
    uint8_t             is_usb_desc_done;
    uint8_t             pending_led_sync;
} app_ctx_t;

/**
 * @brief     Global application context instance
 *
 * This variable holds the global runtime context for the 2.4GHz
 * application layer and is shared across application modules.
 */
extern app_ctx_t app_ctx;

/**
 * @brief     SPP command identifiers
 *
 * This enumeration defines command numbers used in SPP
 * (Serial Port Profile–like) data communication.
 */
typedef enum
{
    LED_STATUS = 1,    /**< LED status control command */
} spp_cmd_num;


/**
 * @brief     Pairing timeout tick counter
 *
 * This variable records the timeout tick value used during
 * the pairing procedure. It is typically updated by timer
 * or interrupt logic.
 *
 * @note      Declared as volatile since it may be accessed
 *            in both interrupt and main contexts.
 */
extern volatile unsigned int tick_pair_timeout;

bool rr_is_valid(report_rate_t rr);

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif
