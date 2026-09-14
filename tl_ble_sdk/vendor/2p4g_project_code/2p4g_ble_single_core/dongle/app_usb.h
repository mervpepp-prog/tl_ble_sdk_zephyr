/********************************************************************************************************
 * @file    app_usb.h
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
#ifndef __APP_USB_H__
#define __APP_USB_H__

void app_mouse_usb_ep_buff_fill(void);
unsigned char app_mouse_usb_ep_buf_has_data(void);
unsigned char app_mouse_8k_usb_ep_buf_has_data(void);

void app_usb_init(void);
void app_usb_main_loop(void);

unsigned char app_mouse_report_to_usb(unsigned char *buf);
unsigned char app_usb_ep_is_idle(unsigned char ep_num);
void app_hid_set_report(unsigned char bus, unsigned char hid_itf, unsigned char report_id, unsigned char report_type, unsigned char *report, unsigned short report_len);
void app_usb_mouse_report_test(void);
uint8_t app_get_usb_wake_status(void);
void app_set_usb_wake_status(uint8_t state);


extern unsigned char kb_usb_led_status;

_attribute_ram_code_sec_ unsigned char app_rf_normal_key_report_to_usb(unsigned char *buf, unsigned char len);
_attribute_ram_code_sec_ void app_usb_bus_reset_init(void);

#define USB_OTA_ENABLE                       1

#define USB_ACTIVE_STATE            0
#define USB_ENTER_SUSPEND           1
#define USB_EXIT_SUSPEND            2


#if USB_OTA_ENABLE 

typedef struct{
    unsigned char   report_id;
    unsigned char   opcode;
    unsigned short  length;
    unsigned char   dat[60];
}usb_data_t;

typedef struct{
    unsigned short adr_index;
    unsigned char   data[16];
    unsigned short crc_16;
} ota_pdu16_t;

void usb_ota_init(void);
unsigned char  usb_ota_loop(void);
void usb_ota_reboot_check(void);
void usb_ota_reboot_init(void);
uint32_t app_get_usb_enum_status(void);
#endif
/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif




/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif // __HID_KEYBOARD_DESCRIPTOR_H__
