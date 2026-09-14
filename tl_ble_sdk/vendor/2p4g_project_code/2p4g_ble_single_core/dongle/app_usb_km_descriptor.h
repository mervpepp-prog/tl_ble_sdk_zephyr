/********************************************************************************************************
 * @file    app_usb_km_descriptor.h
 *
 * @brief   This is the header file for Telink RISC-V MCU
 *
 * @author  Driver Group
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
#ifndef __APP_USB_KM_DESCRIPTOR_H__
#define __APP_USB_KM_DESCRIPTOR_H__

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#define USB_HIGH_SPEED_EN 0

#define ID_VENDOR  0x248a
#define ID_PRODUCT 0x8006
#define ID_VERSION 0x0100

#define STRING_VENDOR  "Telink Semi-conductor Ltd, Co"
#define STRING_PRODUCT "Telink Keyboard"
#define STRING_SERIAL  "Keyboard demo"

#define APP_USB_KM_DESC_MANU_FACTURER_STR       "Telink"
#define APP_USB_KM_DESC_PRODUCT_STR             "TL322X Keyboard"
#define APP_USB_KM_DESC_SERIAL_NUM_STR          "123456"

#define HID_KEYBOARD_IN_ENDPOINT_ADDRESS  0x81
#define HID_KEYBOARD_IN_ENDPOINT_NUM  0x01
#define HID_KEYBOARD_IN_ENDPOINT_SIZE     0x10
#define HID_KEYBOARD_IN_ENDPOINT_INTERVAL 0x01

#define HID_MOUSE_IN_ENDPOINT_ADDRESS  0x82
#define HID_MOUSE_IN_ENDPOINT_NUM  0x02
#define HID_MOUSE_IN_ENDPOINT_SIZE     0x08
#define HID_MOUSE_IN_ENDPOINT_INTERVAL 0x01

#define HID_ALL_KB_IN_ENDPOINT_ADDRESS  0x83
#define HID_ALL_KB_IN_ENDPOINT_NUM  0x03
#define HID_ALL_KB_IN_ENDPOINT_SIZE     0x20
#define HID_ALL_KB_IN_ENDPOINT_INTERVAL 0x01


#define APP_MOUSE_ENDPOINT_BUFF_SIZE                32 /* unit: word */
#define APP_KEYBOARD_ENDPOINT_BUFF_SIZE             32 /* unit: word */
#define APP_KEYBOARD_ALL_KEY_ENDPOINT_BUFF_SIZE     32 /* unit: word */
#define APP_MOUSE_ENDPOINT_BUFF_MIN_SIZE            2

#define  USB_VENDOR_REPORT_ID              05
#define CUSTOM_OUTPUT_REPORT_ID            09
#define USB_OTA_LENGTH                    64


#define CDC_DATA_IN_ENDPOINT_ADDRESS 0x84
#define CDC_DATA_IN_ENDPOINT_SIZE    USB_OTA_LENGTH

#define CDC_DATA_OUT_ENDPOINT_ADDRESS 0x05
#define CDC_DATA_OUT_ENDPOINT_SIZE    USB_OTA_LENGTH



/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif // __HID_KEYBOARD_DESCRIPTOR_H__
