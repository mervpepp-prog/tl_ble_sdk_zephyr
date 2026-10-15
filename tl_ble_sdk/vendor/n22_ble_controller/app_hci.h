/********************************************************************************************************
 * @file    app_hci.h
 *
 * @brief   App-side HCI service header for the n22_ble_controller project.
 *          The shared HCI interface is owned by the stack (stack/ble/hci/hci.h);
 *          this header is the app entry point: it pulls in the stack interface
 *          and the app owned two-level command tables (app_cmd_table.h).
 *          The app_hci module (vendor/n22_ble_controller/app_hci.c) owns the
 *          complete HCI command dispatch of this application (b85m app_hci
 *          style): the tables, blc_hci_dispatch() and the packet entries.
 *          Enable/disable the table entries via the APP_LE_* switches
 *          defined in app_config.h.
 *
 * @author  BLE GROUP
 * @date    2026.10
 *
 * @par     Copyright (c) 2026, Telink Semiconductor (Shanghai) Co., Ltd.
 *          All rights reserved.
 *
 *          The information contained herein is confidential property of Telink
 *          Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *          of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *          Co., Ltd. and the licensee or the terms described here-in. This heading
 *          MUST NOT be removed from this file.
 *
 *          Licensee shall not delete, modify or alter (or permit any third party to delete, modify, or
 *          alter) any information contained herein in whole or in part except as expressly authorized
 *          by Telink semiconductor (shanghai) Co., Ltd. Otherwise, licensee shall be solely responsible
 *          for any claim to the extent arising out of or relating to said deletion(s), modification(s) or
 *          alteration(s).
 *
 *          Licensees are granted free, non-transferable use of the information in this
 *          file under Mutual Non-Disclosure Agreement. NO WARRANTY OF ANY KIND is provided.
 *
 *******************************************************************************************************/
#pragma once

#include "app_config.h"
#include "stack/ble/hci/hci.h"   // shared HCI interface (stack-owned)
#include "app_cmd_table.h"       // app owned two-level command tables

// App-level HCI/LE feature switches (APP_LE_*) are defined in app_config.h.
// They enable/disable the command entries of the app_hci tables; the defaults
// map to the stack LL feature macros so the tables always match the handlers
// compiled in stack/ble/hci/hci_cmd.c. Override any macro in app_config.h
// (or via -D) to trim the corresponding HCI command set of this application.
#ifndef L2CAP_CONTINUING_PKT
#define L2CAP_CONTINUING_PKT 0x01
#endif

#ifndef smemcpy
#define smemcpy     tlk_mem_cpy
#endif