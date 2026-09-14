/********************************************************************************************************
 * @file    app_2p4g.h
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
#ifndef __APP_D24G_RF_H__
#define __APP_D24G_RF_H__

#include "../app_config.h"
#include "tl_fifo.h"
#include "../../app_common_config.h"

typedef void (*p24g_sm_cmd_handler_t)(uint8_t *data, uint16_t len);

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

/* Generic event header */
typedef struct p24g_evt
{
    uint8_t type;
    uint8_t opcode;
    uint8_t len;
    uint8_t data[0];
} __attribute__((packed)) p24g_evt_t;


extern pl_fifo_t LLTxFifo;
extern pl_fifo_t *n22SppRxFifo;
extern pl_fifo_t *n22KbRxFifo;


void pp_d24g_user_init(void);
void pp_d24g_main_loop(void);


void app_2p4g_mb_km_data_cb(u8* buf);
void app_2p4g_mb_n22_set_kb_rx_fifo(u8* buf);
void app_2p4g_mb_n22_set_spp_rx_fifo(u8* buf);
void app_2p4g_mb_n22_set_app_ctx(u8* buf);
void app_2p4g_mb_n22_set_pm_info_ctx(u8* buf);
void app_2p4g_mb_set_clk_info_cb(void* buf);


/**
 * @brief   Register a shared memory (SM) command handler for the 2.4GHz protocol
 *
 * This function registers a handler function for a specified SM command in the
 * 2.4GHz protocol stack. When the given command is received through the SM
 * (shared memory) communication channel, the registered handler will be invoked
 * with the associated data buffer.
 *
 * @param[in] cmd       Command identifier of type @ref p24g_sm_cmd_e
 * @param[in] handler   Callback function pointer of type @ref p24g_sm_cmd_handler_t
 *
 * @return   0: success
 *           Other: fail
 *
 * @note     If a handler for the same command is already registered, it will
 *           be overwritten by the new handler.
 */
uint8_t p24g_register_sm_cmd_handler(p24g_sm_cmd_e cmd, p24g_sm_cmd_handler_t handler);

/**
 * @brief     Shared memory (SM) receive callback for 2.4GHz N22 core
 *
 * This callback function is invoked when the shared memory (SM) interface
 * receives data from another MCU core in an dual-core system. It is
 * part of the inter-core communication mechanism used by the 2.4GHz protocol
 * stack running on the N22 core.
 *
 * @param[in] data  Pointer to the received data buffer
 * @param[in] len   Length of the received data in bytes
 *
 * @note      The buffer pointed by @p data is only valid during the callback
 *            execution. If the data needs to be stored for later processing,
 *            it must be copied to a safe memory area before the callback returns.
 *
 * @return    None
 */
void app_2p4g_n22_sm_rx_cb(uint8_t *data, uint16_t len);

/**
 * @brief     Send a shared memory (SM) message in the 2.4GHz protocol
 *
 * This function sends a message to the 2.4GHz protocol layer through the
 * shared memory (SM) interface. The SM interface is used for inter-core
 * communication in dual-core systems, enabling fast and low-latency data
 * exchange between the application processor d25f and the RF protocol core n22.
 *
 * @param[in] type  Message type identifier
 * @param[in] op    Message operation code
 * @param[in] data  Pointer to the message data buffer
 * @param[in] len   Length of the message data in bytes
 * 
 * @return    0 if sending is successful, non-zero error code if failed
 *
 */
uint8_t p24g_send_sm_msg(uint8_t type, uint8_t op,  uint8_t *data, uint8_t len);



/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif
