/********************************************************************************************************
 * @file    app.h
 *
 * @brief   This is the header file for BLE SDK
 *
 * @author  BLE GROUP
 * @date    06,2022
 *
 * @par     Copyright (c) 2022, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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
#ifndef VENDOR_APP_H_
#define VENDOR_APP_H_

#include "../../stack/2p4g/tl_tpll/tl_tpll.h"


/**
 * @brief       user initialization when MCU power on or wake_up from deepSleep mode
 * @param[in]   none
 * @return      none
 */
void user_init_normal(void);


/**
 * @brief       user initialization when MCU wake_up from deepSleep_retention mode
 * @param[in]   none
 * @return      none
 */
void user_init_deepRetn(void);


/**
 * @brief     BLE main loop
 * @param[in]  none.
 * @return     none.
 */
void main_loop(void);


// /**
//  * @brief       D25F communication configuration functions.
//  *              Based on tl_tpll_start, provide individual parameter setting
//  *              for interoperating with D25F.
//  */
// uint32_t n22_set_mode(tl_tpll_mode_t mode);
// uint32_t n22_set_bitrate(TPLL_BitrateTypeDef bitrate);
// uint32_t n22_start_tx(void);
// uint32_t n22_set_retransmit_para(uint16_t count, uint16_t delay_us);
// uint32_t n22_write_payload(uint8_t pipe, uint8_t *data, uint8_t len);
// uint32_t n22_set_access_code_length(uint8_t length);
// uint32_t n22_set_access_code(uint8_t pipe, uint8_t *addr);
// uint32_t n22_set_rf_channel(uint8_t chn);

#endif /* VENDOR_APP_H_ */
