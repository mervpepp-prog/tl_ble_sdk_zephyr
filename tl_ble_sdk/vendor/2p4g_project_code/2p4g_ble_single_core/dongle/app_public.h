/********************************************************************************************************
 * @file    app_public.h
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
#ifndef __APP_PUBIC_H__
#define __APP_PUBIC_H__

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

#include "app_prj_config.h"
#include "tl_fifo.h"
#include "app_d24g_stack.h"
#include "app_d24g_rf_link.h"
#include "app_d24g.h"
#include "stack/ble/ble.h"
#include "app_usb.h"
#include "lib/include/nvm.h"
#include "../app_common_config.h"
#include "../common/app_common.h"
#if DRIVER_BQB_TEST
#include "bqb.h"
#endif

#define   LED_IS_ON  1
#define   LED_IS_OFF 0

typedef struct
{
    unsigned int side_id; //4
    uint8_t peer_addr[MAC_ADDR_LEN];//6
    uint8_t peer_type;

} ST_FLASH_DEV_INFO;

extern ST_FLASH_DEV_INFO flash_dev_info;

extern unsigned int flash_sector_2p4_inf;

extern int dev_info_idx;

extern volatile unsigned int   usb_connected_ok;

extern pl_fifo_t tx_fifo;

extern volatile bool g_nvm_bulk_write;

// typedef fifo_cb_t spp_cb_t;
// typedef fifo_cb_t data_cb_t;

void read_storage_to_ram(unsigned int addr, unsigned int len, unsigned char *data);
void write_storage_from_ram(unsigned int addr, unsigned int len, unsigned int *data);
pm_dig_vol_mode_e app_dig_vol_get_val(void);
pm_dig_vol_mode_e app_dig_vol_raise_to_1v1(void);
void app_dig_vol_restore(pm_dig_vol_mode_e vol);

int flash_info_load(unsigned int s_addr, unsigned int *d_addr,  int len);


/**
 * @brief   save data to flash
 * @param[in]   addr    - the base address of flash.
 * @param[in]   len     - the length(in byte, must be above 0) of content needs to read out from the page.
 * @param[in]   buf     - data  buffer(ram address)
 * @param[in,out]  offset     - The offset address of the last data
 * @return  1: success  ,other: fail
 */

_attribute_ram_code_sec_ int save_data_to_flash(unsigned long addr, int len, unsigned int *buf,int *offset);


unsigned char  Draw_a_square_test(signed short *x,signed short *y);
unsigned char Draw_a_circle(signed short *x,signed short *y);
void app_get_mouse_data(void);
void app_get_kb_data(void);

void app_notfiy_led_to_kb(unsigned char status);
void app_deferred_led_sync_check(void);


void app_uart_init(void);

void rf_print_cnt_debug(void);

void app_debug_print(unsigned int d1, unsigned int d2, unsigned int d3, unsigned int d4);



void app_halt_system(uint16_t error_code, const char* format, ...);

void tprintf(unsigned char* format, ...);
extern unsigned int app_version;
uint32_t fnv1a_hash(const uint8_t *data, size_t len);
uint16_t fnv1a_16(uint8_t *data, size_t len);

#if (APP_FLASH_PROTECTION_ENABLE)
void flash_lock_op(void);
void flash_unlock_op(void);
#endif
#if(EMI_TEST_FUN_ENABLE)

extern u8 emi_flg;
void usb_host_cmd_proc(void);
#endif

void app_allow_pair_new_device_power_on_handle(void);
// void pp_timer0_init(void);
// void pp_timer0_stop(void);
// void pp_timer1_init(void);
// void pp_timer1_start(uint32_t sys_tick);
// void pp_timer1_stop(void);


/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif // __HID_KEYBOARD_DESCRIPTOR_H__

