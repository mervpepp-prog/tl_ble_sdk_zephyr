/********************************************************************************************************
 * @file    app_config.h
 *
 * @brief   This is the header file for 2.4G SDK
 *
 * @author  2.4G Group
 * @date    2024
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
#pragma once
#include "config.h"

#define __2P4G_BLE_SC_KMD_EN__  1

#define TEST_SECU_SIGN_VRFY_EN  0

///////////////////////// BOARD TYPE SELECT ////////////////////////////////
#define HW_EVK_DONGLE               0 //2*16
#define HW_PRJ_DONGLE               1 //8*16

#define HW_BOARD_TYPE               HW_EVK_DONGLE

///////////////////////// DEBUG  Configuration ////////////////////////////////////////////////
#define TLKAPI_DEBUG_ENABLE         0

#define APP_LOG_EN            (1&TLKAPI_DEBUG_ENABLE)
#define APP_CONTR_EVT_LOG_EN  (0&TLKAPI_DEBUG_ENABLE) //controller event
#define APP_HOST_EVT_LOG_EN  (0&TLKAPI_DEBUG_ENABLE)
#define APP_KEY_LOG_EN       (0&TLKAPI_DEBUG_ENABLE)
#define APP_FLASH_INIT_LOG_EN (0&TLKAPI_DEBUG_ENABLE)

#define TLKAPI_DEBUG_CHANNEL        TLKAPI_DEBUG_CHANNEL_UART
#define TLKAPI_DEBUG_UART_PORT      DBG_UART_PORT0
#define TLKAPI_DEBUG_UART_TX_PIN    GPIO_FC_PG0
#define TLKAPI_DEBUG_UART_BAUDRATE  1000000

#define SPP_TEST_EN                 0


///////////////////////// APP FUNCTION ENABLE ////////////////////////////
#define ALG_KEYSCAN_APP_FUN_ENABLE    0
#define USB_APP_FUN_ENABLE           1
#define RF_MODE_APP_FUN_ENABLE      1

#define APP_REPORT_MOUSE_TO_USB_EN  1

#define DRIVER_BQB_TEST             0

///////////////////////// UI ////////////////////////////////

#define ACL_CENTRAL_MAX_NUM 0 // ACL central maximum number
#define ACL_PERIPHR_MAX_NUM 4 // ACL peripheral maximum number

///////////////////////// Feature Configuration////////////////////////////////////////////////
#define ACL_PERIPHR_SMP_ENABLE        1 //1 for smp,  0 no security
#define BLE_OTA_SERVER_ENABLE         1

#if MCU_CORE_TYPE == MCU_CORE_TL322X
    #define BLE_APP_PM_ENABLE         0
    #define PM_DEEPSLEEP_RETENTION_ENABLE 0
#else
    #define BLE_APP_PM_ENABLE         1
    #define PM_DEEPSLEEP_RETENTION_ENABLE 1
#endif

#define EMI_TEST_FUN_ENABLE           1
#define BATT_CHECK_ENABLE             0

#define     APP_WATCHDOG_ENABLE        0

#ifdef MCU_STARTUP_FLASH
    #define MCU_RUN_IN_FLASH          1
#else
    #define MCU_RUN_IN_FLASH          0
#endif

#ifdef MCU_STARTUP_NVM
    #define MCU_RUN_IN_NVM            1
#else
    #define MCU_RUN_IN_NVM            0
#endif

/* Flash Protection:
 * 1. Flash protection is enabled by default in SDK. User must enable this function on their final mass production application.
 * 2. User should use "Unlock" command in Telink BDT tool for Flash access during development and debugging phase.
 * 3. Flash protection demonstration in SDK is a reference design based on sample code. Considering that user's final application may
 *    different from sample code, for example, user's final firmware size is bigger, or user have a different OTA design, or user need
 *    store more data in some other area of Flash, all these differences imply that Flash protection reference design in SDK can not
 *    be directly used on user's mass production application without any change. User should refer to sample code, understand the
 *    principles and methods, then change and implement a more appropriate mechanism according to their application if needed.
 */
#if (MCU_CORE_TYPE == MCU_CORE_TL322X)
    #define APP_FLASH_PROTECTION_ENABLE 1
    #define APP_NVM_PROTECTION_ENABLE   1
#else
    #define APP_FLASH_PROTECTION_ENABLE 1
    #define APP_NVM_PROTECTION_ENABLE   1
#endif

#if MCU_CORE_TYPE == MCU_CORE_TL322X
    #define PLIC_ENABLE           1
    #define CLIC_ENABLE           0
#endif

///////////////////////// flash address Configuration////////////////////////////////////////////////

#define P24G_PAIR_INF_FLASH_ADDR_512K   0x77000      //512K
#define CFG_ADR_MAC_512K_RRAM           0x7F000
#define CFG_ADR_CALIBRATION_512K_RRAM   0x7E000
/* SMP pairing and key information area */
#define FLASH_ADR_SMP_PAIRING_512K_RRAM 0x74000 //74000 & 75000 & 76000 & 77000
/* for ACL Central simple SDP: bonding ACL Peripheral GATT service critical information area */
#define FLASH_SDP_ATT_ADDRESS_512K_RRAM 0x72000 //72000 & 73000

#define P24G_PAIR_INF_FLASH_ADDR_1M     0xF7000      //1M

#define P24G_PAIR_INF_FLASH_ADDR_2M     0x1F7000     //2M

#define P24G_PAIR_INF_FLASH_ADDR_4M     0x3F7000     //4M

///////////////////////// OS settings /////////////////////////////////////////////////////////
#define FREERTOS_ENABLE           0
#define OS_SEPARATE_STACK_SPACE   1 //Separate the task stack and interrupt stack space
#define configTOTAL_HEAP_SIZE     (16 * 1024)
#define configISR_PLIC_STACK_SIZE 640

/////////////////////// Board Select Configuration ///////////////////////////////
#define BOARD_SELECT BOARD_322X_EVK_C1T382A20

///////////////////////// UI Configuration ////////////////////////////////////////////////////
#define UI_LED_ENABLE      1
#define UI_KEYBOARD_ENABLE 0


/////////////////// DEEP SAVE FLG //////////////////////////////////
#define USED_DEEP_ANA_REG PM_ANA_REG_POWER_ON_CLR_BUF1 //u8,can save 8 bit info when deep
#define LOW_BATT_FLG      BIT(0)                       //if 1: low battery
#define CONN_DEEP_FLG     BIT(1)                       //if 1: conn deep, 0: adv deep


#if FREERTOS_ENABLE
    #define traceAPP_LED_Task_Toggle()   //gpio_toggle(GPIO_CH01);
    #define traceAPP_BLE_Task_BEGIN()    //gpio_write(GPIO_CH02,1);
    #define traceAPP_BLE_Task_END()      //gpio_write(GPIO_CH02,0);
    #define traceAPP_KEY_Task_BEGIN()    //gpio_write(GPIO_CH03,1);
    #define traceAPP_KEY_Task_END()      //gpio_write(GPIO_CH03,0);
    #define traceAPP_BAT_Task_BEGIN()    //gpio_write(GPIO_CH04,1);
    #define traceAPP_BAT_Task_END()      //gpio_write(GPIO_CH04,0);

    #define traceAPP_MUTEX_Task_BEGIN()  //gpio_write(GPIO_CH05,1);
    #define traceAPP_MUTEX_Task_END()    //gpio_write(GPIO_CH05,0);

    #define tracePort_IrqHandler_BEGIN() //gpio_write(GPIO_CH06,1);
    #define tracePort_IrqHandler_END()   //gpio_write(GPIO_CH06,0);

#endif

#define ESL_RAM_OPTIMIZATION 0
#define LL_CS_SNIFFER_MODE_ENABLE 0

#ifndef __CHANNEL_SOUNDING_EN__
#define __CHANNEL_SOUNDING_EN__ 0
#endif

#include "../common/default_config.h"
