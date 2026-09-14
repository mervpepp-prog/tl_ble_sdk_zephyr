/********************************************************************************************************
 * @file    ext_fast_settle.h
 *
 * @brief   This is the header file for BLE SDK
 *
 * @author  BLE GROUP
 * @date    06,2024
 *
 * @par     Copyright (c) 2024, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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

#ifndef DRIVERS_EXT_DRIVER_EXT_FAST_SETTLE_H_
#define DRIVERS_EXT_DRIVER_EXT_FAST_SETTLE_H_

// #include "lib/include/rf/rf_common.h"

/**
 * @brief      Enable fast settle for 1Mbps PHY
 */
#ifndef  EXT_FASTSETTLE_1M_PHY_EN
#define  EXT_FASTSETTLE_1M_PHY_EN    1
#endif
/**
 * @brief      Enable fast settle for 2Mbps PHY
 */
#ifndef  EXT_FASTSETTLE_2M_PHY_EN
#define  EXT_FASTSETTLE_2M_PHY_EN    1
#endif
/**
 * @brief      Enable fast settle for S2 PHY
 */
#ifndef  EXT_FASTSETTLE_S2_PHY_EN
#define  EXT_FASTSETTLE_S2_PHY_EN    1
#endif
/**
 * @brief      Enable fast settle for S8 PHY
 */
#ifndef  EXT_FASTSETTLE_S8_PHY_EN
#define  EXT_FASTSETTLE_S8_PHY_EN    1
#endif

/**
 * @brief      Enable fast settle for 6M PHY
 */
#ifndef  EXT_FASTSETTLE_6M_PHY_EN
#define  EXT_FASTSETTLE_6M_PHY_EN    1
#endif

// PHY initialization options (bitmask)
// PHY initialization options (bitmask, aligned with EXT_FAST_SETTLE_* naming convention)
typedef enum {
    EXT_FAST_SETTLE_PHY_NONE    = 0x00,  // No PHY initialization (theoretical option, 1M is mandatory)
    EXT_FAST_SETTLE_PHY_1M      = 0x01,  // Initialize 1M PHY (always enabled by default)
    EXT_FAST_SETTLE_PHY_2M      = 0x02,  // Initialize 2M PHY (requires EXT_FASTSETTLE_2M_PHY_EN)
    EXT_FAST_SETTLE_PHY_6M      = 0x04,  // Initialize 6M PHY (requires EXT_FASTSETTLE_6M_PHY_EN)
    EXT_FAST_SETTLE_PHY_S2      = 0x08,  // Initialize S2 PHY (requires EXT_FASTSETTLE_S2_PHY_EN)
    EXT_FAST_SETTLE_PHY_S8      = 0x10,  // Initialize S8 PHY (requires EXT_FASTSETTLE_S8_PHY_EN)
    EXT_FAST_SETTLE_PHY_ALL     = 0XFF  // Initialize all supported PHYs
} ext_fast_settle_phy_type_t;


/**
 * @brief      Recover fast settle registers after suspend/ deepsleep retention
 * @param[in]  none
 * @return     none
 */
_attribute_ram_code_sec_noinline_ void  ext_fast_settle_recover(void);

/**
 * @brief      Initialize fast settle function
 * @param[in]  tx_fast_en - TX fast settle enable(0=disable,1=enable)
 * @param[in]  rx_fast_en - RX fast settle enable(0=disable,1=enable)
 *  @param[in]  phy_mask - Bitmask of EXT_FAST_SETTLE_PHY_* to initialize (1M is forced)
 *  @return     none
 */
void ext_fast_settle_init(unsigned char tx_fast_en, unsigned char rx_fast_en, ext_fast_settle_phy_type_t phy_mask);



/**
 *  @brief      Select fast settle parameters by PHY type (with initialization check)
 *  @param[in]  phy - PHY type identifier (EXT_FAST_SETTLE_PHY_* enumeration)
 *  @return     0 if successful, 1 for unknown PHY type, 2 for uninitialized PHY
 */
unsigned char  ext_fast_select_phy(ext_fast_settle_phy_type_t phy);


/**
 *  @brief      Set fast settle parameters by PHY type (with initialization check)
 *  @param[in]  phy - PHY type identifier (EXT_FAST_SETTLE_PHY_* enumeration)
 *  @return     0 if successful, 1 for unknown PHY type, 2 for uninitialized PHY
 */
 unsigned char ext_fast_settle_set_val(ext_fast_settle_phy_type_t phy);
/**
 * @brief      Alias for ext_FastSettle_recover
 */
#define  ble_rf_fast_settle_recover   ext_fast_settle_recover

/**
 * @brief      Alias for ext_FastSettle_init
 */
#define  blc_ll_initFastSettle(tx,rx)         do{   \
                                              ext_fast_settle_init(tx,rx,EXT_FAST_SETTLE_PHY_ALL); \
                                              rf_set_ble_1M_mode();\
                                              }while(0);
#endif /* DRIVERS_EXT_DRIVER_EXT_FAST_SETTLE_H_ */
