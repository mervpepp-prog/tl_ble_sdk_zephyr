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


#define BLE_APP_PM_ENABLE             0
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
#if BLC_ZEPHYR_BLE_INTEGRATION
#define BLMS_PM_ENABLE 0

#undef ACL_CENTRAL_MAX_NUM
#define ACL_CENTRAL_MAX_NUM 0

#undef ACL_PERIPHR_MAX_NUM
#define ACL_PERIPHR_MAX_NUM 1

#define BLUETOOTH_VER BLUETOOTH_VER_5_0

#define CUSTOM_DARWIN_FMN_ENABLE 0
#define ULL_FOR_CIS_EN 0
#define DOCKKIT_MODIFY_ATT_RSP_LEN_EN 0
#define LL_FEATURE_SUPPORT_LE_PING 0
#define LL_FEATURE_SUPPORT_PRIVACY 1
#define LL_FEATURE_SUPPORT_LE_CODED_PHY 1
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

#define LL_FEATURE_SUPPORT_CHANNEL_SELECTION_ALGORITHM2 0
#define LL_FEATURE_SUPPORT_PHY_TEST_MODE                            1
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
#define SMP_REAL_ENCRYPTION_BUSY_ENABLE 0
#define OS_SUP_EN 0
#define OS_SUP_LONG_SLEEP 0
#define UPDATE_SCHEDULER_FOR_PAWR_RSP 0

#define LL_FEATURE_SUPPORT_PERIODIC_ADVERTISING_WITH_RESPONSES_SCANNER 0

#define LL_ACL_CEN_EN 0
#endif


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
#include "../common/default_config.h"
#include "bqb_config.h"
