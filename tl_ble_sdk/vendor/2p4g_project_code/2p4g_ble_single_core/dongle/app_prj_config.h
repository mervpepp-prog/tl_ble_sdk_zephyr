/********************************************************************************************************
 * @file    app_prj_config.h
 *
 * @brief   This is the header file for Telink RISC-V MCU
 *
 * @author  Driver Group
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
/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#include "tl_common.h"
#include "app_config.h"
#include "app_d24g.h"
#include "app_public.h"
#include "../app_common_config.h"
#include "../common/app_spp.h"
#define clock_time stimer_get_tick

#define CFG_MY_ID_ADDR   0x3FF000
#define CFG_SIDE_INF_ADDR   0x3FC000


#define USB_MOUSE      1
#define USB_KEYBOARD   2
#define USB_MICROPHONE 3
#define USB_SPEAKER    4
#define USB_CDC        5
#define USB_DEMO_TYPE  USB_KEYBOARD


#define     LED_GREEN   GPIO_PC7
#define     LED_WHITE   GPIO_PC6
#define     LED_BLUE    GPIO_PC5
#define     LED_RED     GPIO_PC4

#define  PIN_LED_CON        GPIO_PF6
#define  PIN_LED_TOG        GPIO_PF0
#define  PIN_LED_DISCON     GPIO_PF1
#define  PIN_LED_PAIR       GPIO_PF3


#define LED1 GPIO_PC4
#define LED2 GPIO_PC7
#define LED3 GPIO_PC6
#define LED4 GPIO_PC5
#define KEY1 GPIO_PG3
#define KEY2 GPIO_PG2
#define KEY3 GPIO_PG1
#define KEY4 GPIO_PG0


#if TLKAPI_DEBUG_ENABLE
#define print_app_usb tlk_printf
#ifndef print_app_usb
    #define print_app_usb printf
#endif

#define debug_print_keyscan tlk_printf
#ifndef debug_print_keyscan
    #define debug_print_keyscan printf
#endif

#define print_app_public tlk_printf
#ifndef print_app_public
    #define print_app_public printf
#endif

#else
#define print_app_usb(...)
#define debug_print_keyscan(...)
#define print_app_public(...)
#endif


/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif
