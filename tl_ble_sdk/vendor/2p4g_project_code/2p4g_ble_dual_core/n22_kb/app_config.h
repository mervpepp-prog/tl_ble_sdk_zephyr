/********************************************************************************************************
 * @file    app_config.h
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

#include "config.h"
// #include "../app_common_config.h"



///////////////////////// BOARD TYPE SELECT ////////////////////////////////
#define HW_EVK_KEYBOARD             0 //2*16
#define HW_ALG_KEYBOARD             1 //8*16

#define HW_BOARD_TYPE               HW_ALG_KEYBOARD

#define TLKAPI_DEBUG_ENABLE   0
#define APP_LOG_EN            (1&TLKAPI_DEBUG_ENABLE)
#define TLKAPI_DEBUG_CHANNEL  TLKAPI_DEBUG_CHANNEL_UART
#define TLKAPI_DEBUG_UART_PORT        DBG_UART_PORT1


#define TLKAPI_DEBUG_UART_TX_PIN            GPIO_FC_PG0
#define TLKAPI_DEBUG_UART_BAUDRATE          1000000

#if MCU_CORE_TYPE == MCU_CORE_TL322X
    #define PLIC_ENABLE           0
    #define CLIC_ENABLE           1
#endif

#define BLE_APP_PM_ENABLE     1


///////////////////////// UI Configuration ////////////////////////////////////////////////////
#define UI_LED_ENABLE 0


///////////////////////// Dual Message Configuration ///////////////////////////////////////
#define MAILBOX_D25F          0
#define TLK_MESSAGE_D25F      0
#define MAILBOX_N22           1
#undef TLK_MESSAGE_N22
#define TLK_MESSAGE_N22       1// Added space after comma (formatting fix)

///////////////////////// Board Select ////////////////////////////////////////////////////
#if (MCU_CORE_TYPE == MCU_CORE_TL322X)
       #define BOARD_SELECT BOARD_322X_EVK_C1T382A20
#endif

#define DRV_RSSI_SNIFFER_MODE_ENABLE 0

/* BLE Configurations */
#define APP_FLASH_INIT_LOG_EN 0
#define APP_CONTR_EVT_LOG_EN  0 //controller event
#define APP_HOST_EVT_LOG_EN   0
#define APP_SMP_LOG_EN        0
#define APP_SIMPLE_SDP_LOG_EN 0
#define APP_PAIR_LOG_EN       0
#define APP_KEY_LOG_EN        0
#define APP_HCI_LOG_EN        0
#define APP_MAILBOX_LOG_EN    1
#define APP_MESSAGE_LOG_EN    0
#define APP_CS_LOG_EN         0

#define ACL_CENTRAL_MAX_NUM 0 // ACL central maximum number
#define ACL_PERIPHR_MAX_NUM 1 // ACL peripheral maximum number

#define DEBUG_GPIO_ENABLE     0

#define DUAL_CORE_MODE_ENABLED 1

#define HCI_UART              0
#define HCI_USB               1
#define HCI_SHAREMEMORY       2
#define HCI_INTERFACE         HCI_SHAREMEMORY

#define ESL_RAM_OPTIMIZATION    0
#define HCI_TR_EN               0
#define HARDWARE_CHANNEL_SOUNDING_SUPPORT_EN 0

#if 1
#define CUSTOM_DARWIN_FMN_ENABLE 0
#define ULL_FOR_CIS_EN 0
#define DOCKKIT_MODIFY_ATT_RSP_LEN_EN 0
#define LL_FEATURE_SUPPORT_LE_PING 0
#define LL_FEATURE_SUPPORT_PRIVACY 1
#define LL_FEATURE_SUPPORT_LE_CODED_PHY 0
#define LL_FEATURE_SUPPORT_REMOTE_PUBLIC_KEY_VALIDATION 0
#define LL_FEATURE_SUPPORT_LE_EXTENDED_ADVERTISING 0
#define LL_FEATURE_SUPPORT_LE_LEGACY_SCANNING 0
#define LL_FEATURE_SUPPORT_LE_EXTENDED_SCANNING 0
#define LL_FEATURE_SUPPORT_LE_EXTENDED_INITIATE 0

#define LL_FEATURE_SUPPORT_LE_PERIODIC_ADVERTISING 0
#define LL_FEATURE_SUPPORT_LE_PERIODIC_ADVERTISING_SYNC 0
#define LL_FEATURE_SUPPORT_PERIODIC_ADVERTISING_WITH_RESPONSES_ADVERTISER 0
#define LL_FEATURE_SUPPORT_LE_PAST_SENDER 0
#define LL_FEATURE_SUPPORT_LE_PAST_RECIPIENT 0

#define LL_FEATURE_SUPPORT_CHANNEL_SELECTION_ALGORITHM2 1
#define LL_FEATURE_SUPPORT_PHY_TEST_MODE                            0
#define LL_FEATURE_SUPPORT_ISOCHRONOUS_CHANNELS 0
#define LL_FEATURE_SUPPORT_ISOCHRONOUS_TEST_MODE 0
#define LL_FEATURE_SUPPORT_CONNECTION_SUBRATING_HOST 0
#define LL_FEATURE_SUPPORT_CHANNEL_CLASSIFICATION 0
#define LL_FEATURE_SUPPORT_ADVERTISING_CODING_SELECTION 0
#define LL_FEATURE_SUPPORT_ADVERTISING_CODING_SELECT_HOST_SUPPORT 0

#define LL_FEATURE_SUPPORT_EXTENDED_FEATURE_SET 0
#define HARDWARE_CHANNEL_SOUNDING_SUPPORT_EN 0
#define LL_FEATURE_SUPPORT_MONITORING_ADVERTISERS 0
#define LL_FEATURE_SUPPORT_FRAME_SPACE_UPDATE 0
#define L2CAP_CREDIT_BASED_FLOW_CONTROL_MODE_EN 0
#define L2CAP_SERVER_FEATURE_SUPPORTED_EATT 0
#define CIS_ADD_CIE 0
#define CIS_CIE_CENTRAL_OPTIMIZE 0
#define CIS_TSK_JUMP_ADD_PDU 0
#define FANQH_OPTIMIZE_BIS_API 0
#define OS_SUP_EN 0
#define OS_SUP_LONG_SLEEP 0
#define UPDATE_SCHEDULER_FOR_PAWR_RSP 0

#define  EXT_FASTSETTLE_S2_PHY_EN    0
#define  EXT_FASTSETTLE_S8_PHY_EN    0
#define  EXT_FASTSETTLE_6M_PHY_EN    0

#define CONN_MAX_NUM_CONFIG CONN_MAX_NUM_C0_P1

#define BLUETOOTH_VER BLUETOOTH_VER_5_0
#endif

#include "../common/default_config.h"
//#include "bqb_config.h"
