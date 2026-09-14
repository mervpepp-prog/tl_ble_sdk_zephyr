/********************************************************************************************************
 * @file    main.c
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
#include "drivers.h"
#include "2p4g/app_public.h"
#include "2p4g/app_d24g_stack.h"
#include "../app_common_config.h"

// u8 func_mode = KB_MODE_2P4G;
u8 func_mode = 0xFF;


/**
 * @brief      application system initialization
 * @param[in]  none.
 * @return     none.
 */
__INLINE void blc_app_system_init(void)
{
    clic_init();
    clic_preempt_feature_en();
}

/**
 * @brief       This is main function
 * @param[in]   none
 * @return      none
 */
int main(void)
{
    //N22 not used
    // blc_pm_select_internal_32k_crystal();

    blc_app_system_init ();
    
    app_n22_core_comm_init();

#if (TLKAPI_DEBUG_ENABLE)
    tlkapi_debug_init();
#endif

    if (func_mode == KB_MODE_2P4G)
    {
        p24g_user_init_normal();
    }
    else if(func_mode == KB_MODE_BLE)
    {
        rf_n22_dig_init();
        rf_drv_ble_init ();
        extern void app_ble_init_normal(void);
        app_ble_init_normal ();
    }

    tlkapi_printf(APP_LOG_EN, "n22 kb modle %x\r\n", func_mode);
    irq_enable ();

    while (1)
    {
#if (TLKAPI_DEBUG_ENABLE)
        tlkapi_debug_handler();
#endif

        if (func_mode == KB_MODE_2P4G) {
            app_2p4g_main_loop();
        } else {
            extern void app_ble_main_loop(void);
            app_ble_main_loop ();
        }
    }

    return 0;
}

extern void blc_sdk_irq_handler(void);
_attribute_ram_code_sec_ void rf_irq_handler(void)
{

    if (func_mode == KB_MODE_2P4G) {
        tpsll_rf_irq_handler();
    } else {
         blc_sdk_irq_handler();
    }
}

_attribute_ram_code_sec_ void stimer_irq_handler(void)
{
    if (func_mode == KB_MODE_2P4G)
    {
        tpsll_stimer_irq_handler();
    }
    else
    {
         blc_sdk_irq_handler();
    }
}

_attribute_ram_code_sec_ void timer1_irq_handler(void)
{
    tpsll_timer1_irq_handler();
}

#if defined(MCU_CORE_TL322X_N22)
CLIC_ISR_REGISTER(rf_irq_handler, IRQ_ZB_RT)
CLIC_ISR_REGISTER(stimer_irq_handler, IRQ_SYSTIMER)
CLIC_ISR_REGISTER(timer1_irq_handler, IRQ_TIMER1)
#else
PLIC_ISR_REGISTER(rf_irq_handler, IRQ_ZB_RT)
PLIC_ISR_REGISTER(stimer_irq_handler, IRQ_SYSTIMER)
PLIC_ISR_REGISTER(timer1_irq_handler, IRQ_TIMER1)
#endif
