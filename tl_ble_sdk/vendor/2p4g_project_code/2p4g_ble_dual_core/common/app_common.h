/********************************************************************************************************
 * @file    app_common.h
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

 #include "tl_fifo.h"
 #include "tl_common.h"
#define rf_stimer_get_tick()     stimer_get_tick()
#define RF_SYSTEM_TIMER_TICK_1US SYSTEM_TIMER_TICK_1US
#define RF_SYSTEM_TIMER_TICK_1MS SYSTEM_TIMER_TICK_1MS

void pp_timer0_init(void);
void pp_timer0_start(uint32_t sys_tick);
void pp_timer0_stop(void);
void pp_timer1_init(void);
void pp_timer1_start(uint32_t sys_tick);
void pp_timer1_stop(void);
bool is_valid_access_code(uint32_t ac);
