/********************************************************************************************************
 * @file    app_public.c
 *
 * @brief   This is the source file for Telink RISC-V MCU
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
#include "tl_common.h"

#include "app_2p4g.h"
#include "app_public.h"
#include "../app_config.h"
#include "app_prj_config.h"
#include "app_d24g_rf_link.h"

#include "stack/multicore_comm/service/service_n22.h"
#include "stack/multicore_comm/service/mcc.h"

app_dual_core_flag_ctx_t* app_dual_core_flag_ctx_ptr = NULL;
p24g_pm_info_ctx_t* p24g_pm_info_ctx_ptr = NULL;

_attribute_aligned_(4) app_ctx_t app_ctx;

ST_FLASH_DEV_INFO flash_dev_info ;
int dev_info_idx;

ST_FLASH_DEV_OTHER_INFO flash_dev_other_info  __attribute__ ((aligned (4)));
int dev_other_info_idx;


static _attribute_ram_code_ void tlk_mailbox_n22_set_mode(uint8_t* data)
{
    func_mode  = (u8)data[0];

    mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_MODE, NULL);
}

void app_n22_core_comm_init(void)
{

    mcc_n22_service_init();
    mcc_mb_register_cb(TLK_MB_D25F_TO_N22_MODE, (mb_recv_cb_t)tlk_mailbox_n22_set_mode);

    while(func_mode > KB_MODE_USB)
    {
        uint8_t msg[8] = {0};
        mb_polling((void *)msg);
        mb_recv_cb[msg[0]](&msg[1]);
    }
}

static void app_2p4g_dual_core_comm_init(void)
{
    mcc_n22_register_shm_recv_cb(TLK_SHM_MSG_2P4G, (shm_recv_cb_t)app_2p4g_n22_sm_rx_cb);

    mcc_mb_register_cb(TLK_MB_D25F_TO_N22_KM_DATA, (mb_recv_cb_t)app_2p4g_mb_km_data_cb);
    mcc_mb_register_cb(TLK_MB_D25F_TO_N22_2P4G_KB_TX_ADDRESS, (mb_recv_cb_t)app_2p4g_mb_n22_set_kb_rx_fifo);
    mcc_mb_register_cb(TLK_MB_D25F_TO_N22_2P4G_SPP_TX_ADDRESS, (mb_recv_cb_t)app_2p4g_mb_n22_set_spp_rx_fifo);
    mcc_mb_register_cb(TLK_MB_D25F_TO_N22_2P4G_APP_CTX_ADDRESS, (mb_recv_cb_t)app_2p4g_mb_n22_set_app_ctx);
    mcc_mb_register_cb(TLK_MB_D25F_TO_N22_SET_CLK_INFO, (mb_recv_cb_t)app_2p4g_mb_set_clk_info_cb);
    mcc_mb_register_cb(TLK_MB_D25F_TO_N22_2P4G_PM_INFO_ADDRESS, (mb_recv_cb_t)app_2p4g_mb_n22_set_pm_info_ctx);
}

void p24g_user_init_normal(void)
{
    trng_init();

    app_2p4g_dual_core_comm_init();

#if RF_MODE_APP_FUN_ENABLE
    pp_d24g_user_init();
#endif

    core_interrupt_enable();

    tlkapi_printf(APP_LOG_EN, "n22 kb init\n");

    #if DBG_2P4G_IO_EN
    // tpsll_stack_debug_io_init(GPIO_PG5, GPIO_PG4, GPIO_PG6, GPIO_PG7, GPIO_PH0, GPIO_PH1, GPIO_PH2, 0);
    tpsll_stack_debug_io_init(GPIO_PD4, GPIO_PD5, GPIO_PD6, 0, 0, 0, 0, 0);
    #endif
}


_attribute_ram_code_sec_ void app_2p4g_main_loop(void)
{
    mcc_n22_loop();

#if RF_MODE_APP_FUN_ENABLE
    pp_d24g_main_loop();
#endif
}
