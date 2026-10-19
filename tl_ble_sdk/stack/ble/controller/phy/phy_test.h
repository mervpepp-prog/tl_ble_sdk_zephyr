/********************************************************************************************************
 * @file    phy_test.h
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
#ifndef PHY_TEST_H_
#define PHY_TEST_H_

#include "tl_common.h"

typedef int (*blc_phy_test_handler_t)(void); /* ble add in sdk */

#define BLC_PHYTEST_DISABLE 0
#define BLC_PHYTEST_ENABLE  1

/**
 * @brief      for user to initialize PHY test module
 * @param      none
 * @return     none
 */
void blc_phy_initPhyTest_module(void);


/**
 * @brief      for user to set PHY test enable or disable
 * @param[in]  en - 1: enable; 0:disable
 * @return     status: 0x00 command OK, no other rvalue
 */
ble_sts_t blc_phy_setPhyTestEnable(u8 en);


/**
 * @brief      for user to get PHY test status: enable or disable
 * @param      none
 * @return     1: PHY test is enable; 0: PHY test is disable
 */
bool blc_phy_isPhyTestEnable(void);

/**
 * @brief      This function is used to manipulate "phytest"
 * @param[in]  *p - the phy test Command packet
 * @param[in]  n  - Reserved for future use
 * @return     0
 */
int blc_phytest_cmd_handler(u8 *p, int n);

///////////////////// LL internal state accessors (implemented in the lib) /////////////////////////
//  phy_test.c is released as source code and must not touch the stack internal
//  globals directly; these accessors are implemented in the library (ll.c).

/**
 * @brief      mark the PHY test module as initialized (LL internal flag)
 * @param[in]  en - 1: initialized; 0: not initialized
 * @return     none
 */
void blc_ll_setPhyTestEn(u8 en);

/**
 * @brief      get the PHY test running flag (LL internal flag)
 * @param      none
 * @return     1: PHY test is running; 0: not running
 */
u8 blc_ll_getPhyTestStart(void);

/**
 * @brief      set the PHY test running flag (LL internal flag)
 * @param[in]  en - 1: running; 0: stopped
 * @return     none
 */
void blc_ll_setPhyTestStart(u8 en);

/**
 * @brief      reset the LL state machine state to NONE (LL internal state)
 * @param      none
 * @return     none
 */
void blc_ll_resetBlmsState(void);

#endif /* PHY_TEST_H_ */
