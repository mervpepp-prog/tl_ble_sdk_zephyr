/********************************************************************************************************
 * @file    app_config.h
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
#pragma once

#include "config.h"
#include "bqb_config.h"


#define BLE_APP_PM_ENABLE             1
#define PM_DEEPSLEEP_RETENTION_ENABLE 0

#define HCI_TR_EN 0
#if HCI_TR_EN
    /*! HCI UART transport pin define */
    #if (MCU_CORE_TYPE == MCU_CORE_B91)
        #define  EXT_HCI_UART_CHANNEL      UART0
        #define  EXT_HCI_UART_IRQ          IRQ_UART0
        #define HCI_TR_RX_PIN              UART0_RX_PD3 //UART0_RX_PB3 //--->EBQ TX
        #define HCI_TR_TX_PIN              UART0_TX_PD2 //UART0_TX_PB2 //--->EBQ RX
        #define HCI_TR_BAUDRATE            (1000000)
        #define HCI_UART_SoftwareRxDone_EN 0
        /*** RTS/CTS Pin ***/
        #if (HCI_UART_SoftwareRxDone_EN)
            #define HCI_TR_RTS_PIN UART0_RTS_PD1
            #define HCI_TR_CTS_PIN UART0_CTS_PD0
        #endif
    #elif (MCU_CORE_TYPE == MCU_CORE_B92)
        #define HCI_TR_RX_PIN   GPIO_FC_PC6
        #define HCI_TR_TX_PIN   GPIO_FC_PC7
        #define HCI_TR_BAUDRATE (1000000)
    #elif (MCU_CORE_TYPE == MCU_CORE_TL721X)
        #define HCI_TR_RX_PIN   GPIO_FC_PB4
        #define HCI_TR_TX_PIN   GPIO_FC_PB5
        #define HCI_TR_BAUDRATE (1000000)
    #elif (MCU_CORE_TYPE == MCU_CORE_TL321X)
        #define HCI_TR_RX_PIN   GPIO_FC_PC4
        #define HCI_TR_TX_PIN   GPIO_FC_PC5
        #define HCI_TR_BAUDRATE (1000000)
    #elif (MCU_CORE_TYPE == MCU_CORE_TL322X)
        #define HCI_TR_RX_PIN   GPIO_FC_PE1
        #define HCI_TR_TX_PIN   GPIO_FC_PE2
        #define HCI_TR_BAUDRATE (1000000)

        #define DBG_HCI_TR      1
    #endif

    /*! HCI transport buffer size define. */
    #define HCI_TR_RX_BUF_SIZE (300)
    #define HCI_TR_TX_BUF_SIZE (300)

    #define HCI_DFU_EN         0
#else
    #define HCI_DFU_EN 0
#endif


#define ACL_CENTRAL_MAX_NUM 4 // ACL central maximum number
#define ACL_PERIPHR_MAX_NUM 4 // ACL peripheral maximum number

#if MCU_CORE_TYPE == MCU_CORE_TL322X
    #define PLIC_ENABLE           0
    #define CLIC_ENABLE           1
#endif

///////////////////////// Application Configuration ////////////////////////////////////////////////////


///////////////////////// UI Configuration ////////////////////////////////////////////////////
#define UI_LED_ENABLE 0

///////////////////////// DEBUG  Configuration ////////////////////////////////////////////////
#define DEBUG_GPIO_ENABLE     0
#define CS_DEBUG_MODE         0
#define BLT_ERR_PROCESS       ERR_TRIGGER_CODE_STUCK

#define TLKAPI_DEBUG_ENABLE   0
#define TLKAPI_DEBUG_CHANNEL  TLKAPI_DEBUG_CHANNEL_UART

#define APP_LOG_EN            1
#define APP_FLASH_INIT_LOG_EN 1
#define APP_CONTR_EVT_LOG_EN  1 //controller event
#define APP_HOST_EVT_LOG_EN   1
#define APP_SMP_LOG_EN        1
#define APP_SIMPLE_SDP_LOG_EN 1
#define APP_PAIR_LOG_EN       1
#define APP_KEY_LOG_EN        1
#define APP_HCI_LOG_EN        1
#define APP_MAILBOX_LOG_EN    1
#define APP_MESSAGE_LOG_EN    1
#define APP_CS_LOG_EN         1

#define MAILBOX_D25F          0
#define TLK_MESSAGE_D25F      0
#define MAILBOX_N22           1
#define TLK_MESSAGE_N22       1

#define APP_SYNCHRONIZED_RECEIVER_EN            0
#define APP_ISOCHRONOUS_BROADCASTER_SYNC_EN     0
#define APP_PAST_EN                             0
#define APP_POWER_CONTROL                       0
#define  APP_PAWR_EN                             0

#if APP_PAWR_EN
    #define LL_FEATURE_SUPPORT_PERIODIC_ADVERTISING_WITH_RESPONSES_ADVERTISER 1
    #define LL_FEATURE_SUPPORT_PERIODIC_ADVERTISING_WITH_RESPONSES_SCANNER 1
#endif
#if (MCU_CORE_TYPE == MCU_CORE_TL322X)
       #define BOARD_SELECT BOARD_322X_EVK_C1T371A20
#endif

#define HCI_UART              0
#define HCI_USB               1
#define HCI_SHAREMEMORY       2
#define HCI_INTERFACE         HCI_SHAREMEMORY
#define MAILBOX_FALSH_WR_NOTIFY 1

#define APP_CS_CONFIG_PER_ACL 1
#define APP_CS_CONFIG_NUM     (ACL_CENTRAL_MAX_NUM + ACL_PERIPHR_MAX_NUM) * APP_CS_CONFIG_PER_ACL

#define DUAL_CORE_MODE_ENABLED 1

#define CS_USE_TX_POWER_LEVEL RF_POWER_P6p50dBm

///////////////////////// LE Feature Switches /////////////////////////////////////

#define HARDWARE_CHANNEL_SOUNDING_SUPPORT_EN 0

#if 1
#define BLMS_PM_ENABLE 1

#undef ACL_CENTRAL_MAX_NUM
#define ACL_CENTRAL_MAX_NUM 0

#undef ACL_PERIPHR_MAX_NUM
#define ACL_PERIPHR_MAX_NUM 1

#define CONN_MAX_NUM_CONFIG CONN_MAX_NUM_C0_P1

#define BLUETOOTH_VER BLUETOOTH_VER_5_0

#define ULL_FOR_CIS_EN 0
#define OS_SUP_EN 0
#define OS_SUP_LONG_SLEEP 0

#define  EXT_FASTSETTLE_S2_PHY_EN    0
#define  EXT_FASTSETTLE_S8_PHY_EN    0

#define LL_FEATURE_SUPPORT_LE_PING 0
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
#define LL_FEATURE_SUPPORT_ISOCHRONOUS_CHANNELS 0
#define LL_FEATURE_SUPPORT_ISOCHRONOUS_TEST_MODE 0
#define LL_FEATURE_SUPPORT_CONNECTION_SUBRATING_HOST 0
#define LL_FEATURE_SUPPORT_CHANNEL_CLASSIFICATION 0
#define LL_FEATURE_SUPPORT_ADVERTISING_CODING_SELECTION 0
#define LL_FEATURE_SUPPORT_ADVERTISING_CODING_SELECT_HOST_SUPPORT 0
#define LL_FEATURE_SUPPORT_PERIPHERAL_INITIATED_FEATURES_EXCHANGE 0
#define LL_FEATURE_SUPPORT_EXTENDED_FEATURE_SET 0
#define LL_FEATURE_SUPPORT_CHANNEL_SOUNDING_REFLECTOR 0
#define LL_FEATURE_SUPPORT_CHANNEL_SOUNDING_INITIATOR 0
#define LL_FEATURE_SUPPORT_CS_TEST 0
#define LL_FEATURE_SUPPORT_MONITORING_ADVERTISERS 0
#define LL_FEATURE_SUPPORT_MONITORING_ADVERTISERS 0
#define LL_FEATURE_SUPPORT_FRAME_SPACE_UPDATE 0
#define LL_FEATURE_SUPPORT_PERIODIC_ADVERTISING_WITH_RESPONSES_SCANNER 0
#define LL_FEATURE_SUPPORT_HIGHER_DATA_THROUGHPUT 0
#define LL_FEATURE_SUPPORT_PHY_TEST_MODE 1
#define LL_FEATURE_SUPPORT_LE_2M_PHY 1
#define LL_FEATURE_SUPPORT_LE_CODED_PHY 0
#endif

///////////////////////// HCI / LE Feature Switches (APP_LE_*) /////////////////////////////////////
// App-level switches used by the app_hci.c two-level command tables (b85m style):
// pure app values, no stack internal macros here - keep them in sync with the LL
// feature configuration the stack of this project is built with (the defaults
// below mirror the effective stack config of this project).
// NOTE: APP_LE_EXTENDED_ADV_EN/APP_LE_PERIODIC_ADV_EN/APP_LE_EXTENDED_SCAN_EN/
// APP_LE_EXTENDED_INIT_EN/APP_LE_PHY_TEST_EN are defined in bqb_config.h,
// APP_PAWR_EN/APP_PAST_EN/APP_POWER_CONTROL/APP_LE_CHANNEL_SOUNDING are defined
// above; they keep controlling both the LL module init and the app_hci command
// table entries.
#ifndef APP_HCI_FLOW_CTRL_EN
#define APP_HCI_FLOW_CTRL_EN                     0
#endif
#ifndef APP_LE_PING_AUTH_PAYLOAD_TIMEOUT_EN
#define APP_LE_PING_AUTH_PAYLOAD_TIMEOUT_EN      0
#endif
#ifndef APP_LE_CHANNEL_CLASSIFICATION_EN
#define APP_LE_CHANNEL_CLASSIFICATION_EN         0
#endif
#ifndef APP_LE_LEGACY_SCANNING_EN
#define APP_LE_LEGACY_SCANNING_EN                0
#endif
#ifndef APP_LE_CREATE_CONN_EN
#define APP_LE_CREATE_CONN_EN                    0
#endif
#ifndef APP_LE_DATA_LENGTH_EXTENSION_EN
#define APP_LE_DATA_LENGTH_EXTENSION_EN          1
#endif
#ifndef APP_LE_P256KEY_EN
#define APP_LE_P256KEY_EN                        0
#endif
#ifndef APP_LE_PRIVACY_EN
#define APP_LE_PRIVACY_EN                        1
#endif
#ifndef APP_LE_RPA_DATA_RELATED_EN
#define APP_LE_RPA_DATA_RELATED_EN               0
#endif
#ifndef APP_LE_2M_PHY_EN
#define APP_LE_2M_PHY_EN                         1
#endif
#ifndef APP_LE_CODED_PHY_EN
#define APP_LE_CODED_PHY_EN                      0
#endif
#ifndef APP_LE_HDT_EN
#define APP_LE_HDT_EN                            0
#endif
#ifndef APP_PAWR_ADV_EN
#define APP_PAWR_ADV_EN                          (APP_PAWR_EN)
#endif
#ifndef APP_PAWR_SCAN_EN
#define APP_PAWR_SCAN_EN                         (APP_PAWR_EN)
#endif
#ifndef APP_LE_PERIODIC_ADV_SYNC_EN
#define APP_LE_PERIODIC_ADV_SYNC_EN              0
#endif
#ifndef APP_LE_AOA_AOD_EN
#define APP_LE_AOA_AOD_EN                        0
#endif
#ifndef APP_LE_PAST_EN
#define APP_LE_PAST_EN                           (APP_PAST_EN)
#endif
#ifndef APP_LE_ISO_EN
#define APP_LE_ISO_EN                            0
#endif
#ifndef APP_LE_CIS_CENTRAL_EN
#define APP_LE_CIS_CENTRAL_EN                    0
#endif
#ifndef APP_LE_CIS_PERIPHERAL_EN
#define APP_LE_CIS_PERIPHERAL_EN                 0
#endif
#ifndef APP_LE_BIG_BROADCASTER_EN
#define APP_LE_BIG_BROADCASTER_EN                0
#endif
#ifndef APP_LE_BIG_SYNC_EN
#define APP_LE_BIG_SYNC_EN                       0
#endif
#ifndef APP_LE_ISO_TEST_EN
#define APP_LE_ISO_TEST_EN                       0
#endif
#ifndef APP_LE_POWER_CONTROL_EN
#define APP_LE_POWER_CONTROL_EN                  (APP_POWER_CONTROL)
#endif
#ifndef APP_LE_SUBRATE_EN
#define APP_LE_SUBRATE_EN                        0
#endif
#ifndef APP_LE_ADV_CODING_SEL_EN
#define APP_LE_ADV_CODING_SEL_EN                 0
#endif
#ifndef APP_LE_DBAF_EN
#define APP_LE_DBAF_EN                           0
#endif
#ifndef APP_LE_EXTENDED_FEATURE_SET_EN
#define APP_LE_EXTENDED_FEATURE_SET_EN           0
#endif
#ifndef APP_LE_CHANNEL_SOUNDING_EN
#define APP_LE_CHANNEL_SOUNDING_EN               (APP_LE_CHANNEL_SOUNDING)
#endif
#ifndef APP_LE_MONITORING_ADVERTISERS_EN
#define APP_LE_MONITORING_ADVERTISERS_EN         0
#endif
#ifndef APP_LE_FRAME_SPACE_UPDATE_EN
#define APP_LE_FRAME_SPACE_UPDATE_EN             0
#endif
#ifndef APP_LE_SCI_EN
#define APP_LE_SCI_EN                            0
#endif

#include "../common/default_config.h"
#include "bqb_config.h"
