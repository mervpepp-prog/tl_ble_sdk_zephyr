/********************************************************************************************************
 * @file    app_public.h
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
#ifndef __APP_PUBLIC_H__
#define __APP_PUBLIC_H__


#include "../../common/app_common.h"
#include "../../app_common_config.h"

extern u8 func_mode;


typedef enum
{
    CLOCK_CONFIG_1V1_192_96  = 0,
    CLOCK_CONFIG_1V_192_32,
} app_clock_config_e;


typedef struct
{
    uint16_t peer_tag;
    uint8_t paired_dev_type;
    uint8_t rr_idx;
    
    bool save_rr_flag;
    bool rr_chg_flag;
    uint8_t dev_status;             /* tpsll_dev_status_e */
    uint8_t flash_save_report_rate;            /* report_rate_t */

    uint8_t mac[MAC_ADDR_LEN];
    uint8_t peer_mac[MAC_ADDR_LEN];
} app_ctx_t;


extern app_ctx_t app_ctx;

typedef struct
{
    bool is_stk_busy;
} app_dual_core_flag_ctx_t;

extern app_dual_core_flag_ctx_t* app_dual_core_flag_ctx_ptr;

typedef struct {
    bool     suspend_allowed;   /* Whether D25F is allowed to enter suspend mode */
    uint32_t next_wakeup_tick;  /* Absolute stimer tick for the next mandatory wakeup of N22 */
} p24g_pm_info_ctx_t;

extern p24g_pm_info_ctx_t* p24g_pm_info_ctx_ptr;


typedef struct
{
    unsigned int side_id; //4
    uint8_t peer_addr[MAC_ADDR_LEN];//6


    // unsigned char key[12];//12

    // unsigned char slave_mac_addr[6];//6

    // unsigned char mode;
    // unsigned char mast_id;
    // unsigned short temp1; //24

    // unsigned char  temp2[8]; //32

} ST_FLASH_DEV_INFO;
extern ST_FLASH_DEV_INFO flash_dev_info;
extern int dev_info_idx;
// extern unsigned int  flash_sector_2p4_inf;

typedef struct
{
    // uint32_t side_id; //4
    uint8_t report_rate; //1

} ST_FLASH_DEV_OTHER_INFO;

extern ST_FLASH_DEV_OTHER_INFO flash_dev_other_info;
extern int dev_other_info_idx;
// extern uint32_t  flash_sector_2p4_other_inf;


typedef enum
{
    NO_SLEEP=0,
    LOW_POWER_OFF_SLEEP = 1,
    D24G_PAIR_TIMEOUT_SLEEP=2,
    D24G_RECONNECT_TIMEOUT_SLEEP=3 ,
    D24G_CONNECT_NO_ACTIVE_TIMEOUT_SLEEP=4,
    D24G_DEEP_SLEEP_REQ=5,
    BLE_PAIR_TIMEOUT_SLEEP=6,
    BLE_RECONNECT_TIMEOUT_SLEEP=7,
    BLE_CONNECT_NO_ACTIVE_TIMEOUT_SLEEP=8,
    BLE_HOST_POWER_OFF_SLEEP=9,
}SLEEP_REASON_ENUM;


void app_n22_core_comm_init(void);

/**
 * @brief       user initialization when MCU power on or wake_up from deepSleep mode
 * @param[in]   none
 * @return      none
 */
// void user_init_normal(void);
void p24g_user_init_normal(void);


/**
 * @brief     2.4G main loop
 * @param[in]  none.
 * @return     none.
 */
void app_2p4g_main_loop(void);


#endif /* __APP_PUBLIC_H__ */
