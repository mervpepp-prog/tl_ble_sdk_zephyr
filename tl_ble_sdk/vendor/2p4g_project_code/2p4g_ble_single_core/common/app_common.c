/********************************************************************************************************
 * @file    app_common.c
 *
 * @brief   This is the source file for Telink RISC-V MCU
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

#include "app_common.h"

void pp_timer0_init(void)
{
    timer_set_init_tick(TIMER0, 0);

    timer_set_mode(TIMER0, TIMER_MODE_SYSCLK);

    timer_set_irq_mask(FLD_TMR0_MODE_IRQ);

    plic_interrupt_enable(IRQ_TIMER0);
    plic_set_priority(IRQ_TIMER0, IRQ_PRI_LEV3);
}

_attribute_ram_code_sec_ void pp_timer0_start(uint32_t sys_tick)
{
    unsigned int tick = sys_tick * sys_clk.pclk / RF_SYSTEM_TIMER_TICK_1US;
    timer_set_init_tick(TIMER0, 0);
    timer_set_cap_tick(TIMER0, tick);
    timer_start(TIMER0);
}

_attribute_ram_code_sec_ void pp_timer0_stop(void)
{
    timer_stop(TIMER0);
}

void pp_timer1_init(void)
{
    timer_set_init_tick(TIMER1, 0);

    timer_set_mode(TIMER1, TIMER_MODE_SYSCLK);

    timer_set_irq_mask(FLD_TMR1_MODE_IRQ);
    plic_interrupt_enable(IRQ_TIMER1);
    plic_set_priority(IRQ_TIMER1, IRQ_PRI_LEV3);
}

_attribute_ram_code_sec_ void pp_timer1_start(uint32_t sys_tick)
{

    unsigned int tick = sys_tick * sys_clk.pclk / RF_SYSTEM_TIMER_TICK_1US;
    timer_set_init_tick(TIMER1, 0);

    timer_set_cap_tick(TIMER1, tick);

    timer_start(TIMER1);
}

_attribute_ram_code_sec_ void pp_timer1_stop(void)
{
    timer_stop(TIMER1);
}

#define BT_INQUIRY_ACCESS_CODE    0x9E8B33U
#define BLE_BROADCAST_ACCESS_CODE 0x8E89BED6U
bool is_valid_access_code(uint32_t ac)
{
    uint32_t t;


    if ((ac == BT_INQUIRY_ACCESS_CODE) ||
        (ac == BLE_BROADCAST_ACCESS_CODE))
    {
        return false;
    }

    /* Detect 4 consecutive 1 */
    t = ac & (ac >> 1);
    t &= (t >> 2);

    if (t)
    {
        return false;
    }

    /* Detect 4 consecutive 0 */
    ac = ~ac;

    t = ac & (ac >> 1);
    t &= (t >> 2);

    if (t)
    {
        return false;
    }

    return true;
}
