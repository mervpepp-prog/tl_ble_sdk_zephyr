/********************************************************************************************************
 * @file    app_2p4g.c
 *
 * @brief   This is the source file for Telink RISC-V MCU
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
#include "app_public.h"
#include "app_2p4g.h"
#include "app_d24g_stack.h"
#include "app_d24g_rf_link.h"
#include "app_prj_config.h"
#include "stack/multicore_comm/service/mcc.h"
#include "stack/multicore_comm/service/service_n22.h"


volatile unsigned int timeout_flag;
volatile tpsll_dev_status_e last_connect_status = STATE_NONE;
static uint8_t g_stack_inited = 0;

uint8_t encrypt_key[] = {0x3A, 0x7B, 0xC2, 0x9D};
uint8_t encrypt_key_len = 4;

/**
 * Note: This descriptor is NOT actually used for USB enumeration.
 * It is only reserved for pairing encryption verification logic.
 * The VID defined in device_desc must match the VID on dongle side;
 * Mismatched VID will lead to pairing failure between device and dongle.
 */
static _attribute_aligned_(4) unsigned char device_desc[] =
{
    0x12,
    0x01,
    0x00,0x02,
    0x00,
    0x00,
    0x00, 
    0x40,
    0x8A,0x24,  // VendorID 0x248a   8  9
    0x04,0xF5,
    0x00,0x01,
    0x01,
    0x02,
    0x03,
    0x01
};

/**
 * @brief Get USB device descriptor pointer
 * @param bus USB bus instance, unused here
 * @return Pointer to static aligned device descriptor buffer
 */
_attribute_ram_code_sec_ unsigned char *usbd_get_device_descriptor(unsigned char bus)
{
    (void)bus;
    return (unsigned char *)&device_desc;
}

void app_var_callback(tpsll_app_var_t *p_app_var, tpsll_access_type_e access_type,tpsll_app_var_id_e var_id);

static tpsll_stack_config_t config = {
    .app_var_cb = app_var_callback,
    .app_cb = {
        .usbd_get_dev_descp = usbd_get_device_descriptor,
        .app_p24g_send_sm_msg = p24g_send_sm_msg,
    }
};

/**
 * @brief share memory message handler table
 */
static p24g_sm_cmd_handler_t p24g_cmd_table[P24G_SM_CMD_MAX] = {0};


_attribute_ram_code_sec_ uint8_t p24g_send_sm_msg(uint8_t type, uint8_t op,  uint8_t *data, uint8_t len)
{
    static u8 TxBuf[90] = {0}; //[important!!]

    p24g_evt_t *p_evt = (p24g_evt_t *)TxBuf;

    p_evt->type = type;
    p_evt->opcode = op;
    uint8_t data_len = len > (sizeof(TxBuf) - 3) ? sizeof(TxBuf) - 3 : len;

    if(data && data_len)
    {
        tmemcpy(p_evt->data, data, data_len);
        p_evt->len = data_len;
    }
    
    if (!mcc_n22_shm_send_msg(TxBuf, sizeof(p24g_evt_t) + p_evt->len, TLK_SHM_MSG_2P4G))
    {
        // tlkapi_printf(APP_LOG_EN, "n22 send sm message success\n");
    }
    else
    {
        // tlkapi_printf(APP_LOG_EN, "n22 send sm message fail\n");
        return 1; // Error: message sending failed
    }

    return 0; // Success
}



_attribute_ram_code_sec_ void app_2p4g_n22_sm_rx_cb(uint8_t *data, uint16_t len)
{
    if (!data || len == 0)
        return;

    uint8_t cmd = data[0];
    if (cmd < P24G_SM_CMD_MAX && p24g_cmd_table[cmd])
    {
        p24g_cmd_table[cmd](data, len);
    }
    else
    {
        tlkapi_send_string_data(APP_LOG_EN, "error! unknow sm command", data, len);
    }
}

/**
 * @brief resister a command handler for a specific command type
 */
uint8_t p24g_register_sm_cmd_handler(p24g_sm_cmd_e cmd, p24g_sm_cmd_handler_t handler) 
{
    uint8_t ret = TLK_ERR_INVALID_PARAM;

    if (cmd < P24G_SM_CMD_MAX && handler) {
        p24g_cmd_table[cmd] = handler;
        ret = TLK_SUCCESS;
    }else {
        tlkapi_printf(APP_LOG_EN, "p24g_register_sm_cmd_handler error %x\n", cmd);
    }

    return ret;
}


_attribute_ram_code_ void app_2p4g_mb_km_data_cb(u8* buf)
{
    uint8_t* data = (uint8_t*)buf;

    tlkapi_send_string_data(APP_LOG_EN, "n22 kb app_2p4g_mb_km_data_cb",data,7);
}

_attribute_ram_code_ void app_2p4g_mb_n22_set_kb_rx_fifo(u8* buf)
{
    uint8_t* data = (uint8_t*)buf;
    config.km_fifo_ptr = (uint32_t *)((u32)data[3] + ((u32)data[4]<<8) + ((u32)(data)[5]<<16) + ((u32)(data)[6]<<24));

    tlkapi_printf(APP_LOG_EN, "n22KbFifoAddr:  %x\r\n", (uint32_t)config.km_fifo_ptr);
    // tlkapi_send_string_data(APP_LOG_EN, "n22KbFifo", config.km_fifo_ptr, 20);
}

_attribute_ram_code_ void app_2p4g_mb_n22_set_spp_rx_fifo(u8* buf)
{
    uint8_t* data = (uint8_t*)buf;
    config.spp_fifo_ptr = (uint32_t *)((u32)data[3] + ((u32)data[4]<<8) + ((u32)(data)[5]<<16) + ((u32)(data)[6]<<24));

    tlkapi_printf(APP_LOG_EN, "n22SppFifoAddr:  %x\r\n", (u32)(config.spp_fifo_ptr));
    // tlkapi_send_string_data(APP_LOG_EN, "n22SppFifo", config.spp_fifo_ptr, 20);
}

_attribute_ram_code_ void app_2p4g_mb_n22_set_app_ctx(u8* buf)
{
    uint8_t* data = (uint8_t*)buf;
    app_dual_core_flag_ctx_ptr = (app_dual_core_flag_ctx_t *)((u32)data[3] + ((u32)data[4]<<8) + ((u32)(data)[5]<<16) + ((u32)(data)[6]<<24));
    config.is_stk_busy_ptr = &app_dual_core_flag_ctx_ptr->is_stk_busy;

    tlkapi_printf(APP_LOG_EN, "app_dual_core_flag_ctx addr:  %x\r\n", app_dual_core_flag_ctx_ptr);
}

_attribute_ram_code_ void app_2p4g_mb_n22_set_pm_info_ctx(u8* buf)
{
    uint8_t* data = (uint8_t*)buf;
    p24g_pm_info_ctx_ptr = (p24g_pm_info_ctx_t *)((u32)data[3] + ((u32)data[4]<<8) + ((u32)(data)[5]<<16) + ((u32)(data)[6]<<24));
    config.next_wakeup_tick_ptr = &p24g_pm_info_ctx_ptr->next_wakeup_tick;
    config.suspend_allowed_ptr = &p24g_pm_info_ctx_ptr->suspend_allowed;

    tlkapi_printf(APP_LOG_EN, "p24g_pm_info_ctx addr:  %x\r\n", p24g_pm_info_ctx_ptr);
}

_attribute_ram_code_ void app_2p4g_mb_set_clk_info_cb(void *buf)
{
    uint8_t* data = (uint8_t*)buf;
    sys_clk_t *p_sys_clk = (sys_clk_t *)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));

    sys_clk.pll_clk = p_sys_clk->pll_clk;
    sys_clk.cclk = p_sys_clk->cclk;
    sys_clk.hclk_n22 = p_sys_clk->hclk_n22;
    sys_clk.pclk = p_sys_clk->pclk;
    sys_clk.mspi_clk = p_sys_clk->mspi_clk;

    #if (TLKAPI_DEBUG_ENABLE)
    uart_hw_fsm_reset(DEBUG_UART_CHANNEL);
    tlkapi_debug_init();
    #endif
    tlkapi_send_string_data(APP_LOG_EN, "n22 kb app_2p4g_mb_set_clk_info_cb", data, 7);
}


_attribute_ram_code_sec_ static void app_2p4g_handle_set_kb_mode(uint8_t *data, uint16_t len)
{
    (void)len;
    p24g_evt_t *p_evt = (p24g_evt_t *)data;
    if (p_evt->type == P24G_SM_CMD_SET_KB_MODE)
    {
        if (p_evt->opcode == KB_MODE_USB)
        {
            tlkapi_send_string_data(APP_LOG_EN, "usb mode, close RF", data, len);
        }
        else if (p_evt->opcode == KB_MODE_2P4G)
        {
            tlkapi_send_string_data(APP_LOG_EN, "2.4G mode, open RF", data, len);
        }
    }
}

_attribute_ram_code_sec_ static void app_2p4g_handle_change_report_rate(uint8_t *data, uint16_t len)
{
    (void)len;
    p24g_evt_t *p_evt = (p24g_evt_t *)data;

    if (p_evt->type == P24G_SM_CMD_REPORT_RATE_CHANGE)
    {
        uint8_t rr = p_evt->data[0];
        int ret = tpsll_change_report_rate(rr);
        if(ret == 0)
        {
            tlkapi_printf(APP_LOG_EN, "change report rate %x success", rr);
        }
        else
        {
            tlkapi_printf(APP_LOG_EN, "change report rate %x fail ret = %d", rr, ret);
        }
    }
}

_attribute_ram_code_sec_ static void app_2p4g_handle_set_pairing(uint8_t *data, uint16_t len)
{
    p24g_evt_t *p_evt = (p24g_evt_t *)data;
    if (p_evt->type == P24G_SM_CMD_PAIRING)
    {
        uint8_t enable = p_evt->opcode;
        tlkapi_send_string_data(APP_LOG_EN, "enter pairing mode", data, len);

        if (enable)
        {
            tpsll_enable_pairing(true, PAIRING_TX_POWER_DEFAULT);
        }
        tlkapi_send_string_data(APP_LOG_EN, "enter pairing state", &enable, 1);
    }
}

_attribute_ram_code_sec_ static void app_2p4g_handle_set_state(uint8_t *data, uint16_t len)
{
    (void)len;
    p24g_evt_t *p_evt = (p24g_evt_t *)data;
    if (p_evt->type == P24G_SM_CMD_SET_STATE)
    {
        if (p_evt->opcode == P24G_SM_OP_ENABLE_RECONN)
        {
            ; // reconnect
            uint8_t enable = p_evt->data[0];

            if(enable)
            {
                tpsll_enable_reconn(true);
                tlkapi_send_string_data(APP_LOG_EN, "enter reconnect state", 0, 0);
            }
        }
    }
}

_attribute_ram_code_sec_ static void app_2p4g_handle_misc(uint8_t *data, uint16_t len)
{
    p24g_evt_t *p_evt = (p24g_evt_t *)data;
    static tpsll_stack_var_t var = {0};
    switch (p_evt->opcode)
    {
        case P24G_SM_OP_MISC_TRANS_MAC:

            tmemcpy(app_ctx.mac, p_evt->data, 6);
            uint16_t tag = tpsll_fnv1a_16(app_ctx.mac, MAC_ADDR_LEN);
            tag = (tag & 0x3fff) | (DEVICE_TYPE_KB << 14);

            var.local_tag = tag,

            tmemcpy(var.mac, app_ctx.mac, MAC_ADDR_LEN);
            tpsll_stack_set_var(&var, TPSLL_STACK_VAR_SET_PAIRING_INFO);

            tlkapi_send_string_data(APP_LOG_EN, "get mac from d25f", app_ctx.mac, 6);
            tlkapi_printf(APP_LOG_EN, "app_inf.tag:%x\n", tag);
        break;

        case P24G_SM_OP_MISC_PEER_INFO:
            tmemcpy(app_ctx.peer_mac, p_evt->data, 6);
            app_ctx.peer_tag = tpsll_fnv1a_16(app_ctx.peer_mac, MAC_ADDR_LEN);

            var.peer_tag = app_ctx.peer_tag,

            tmemcpy(var.peer_mac, app_ctx.peer_mac, MAC_ADDR_LEN);
            tpsll_stack_set_var(&var, TPSLL_STACK_VAR_SET_PAIRING_INFO);


            tlkapi_send_string_data(APP_LOG_EN, "get pair info from d25f", app_ctx.peer_mac, 6);
            tlkapi_printf(APP_LOG_EN, "app_ctx.peer_tag:%x\n", app_ctx.peer_tag);
        break;

        case P24G_SM_OP_MISC_REPORT_RATE: // power on info from d25f
            if ((p_evt->data[0] == REPORT_RATE_8K) || (p_evt->data[0] == REPORT_RATE_4K) || (p_evt->data[0] == REPORT_RATE_2K) || (p_evt->data[0] == REPORT_RATE_1K)
                    || (p_evt->data[0] == REPORT_RATE_500) || (p_evt->data[0] == REPORT_RATE_250) || (p_evt->data[0] == REPORT_RATE_125)) {
                app_ctx.flash_save_report_rate = p_evt->data[0];
                tlkapi_printf(APP_LOG_EN, "app_ctx.flash_save_report_rate:%x\n",  app_ctx.flash_save_report_rate);
            } else {
                app_ctx.flash_save_report_rate = APP_DEFAULT_REPORT_RATE;
            }
            tlkapi_send_string_data(APP_LOG_EN, "get report rate from d25f", data, len);
        break;

        default:
            break;
    }
}


static void app_p24g_sm_cmd_hanlder_init(void)
{
    p24g_register_sm_cmd_handler(P24G_SM_CMD_PAIRING,               app_2p4g_handle_set_pairing);
    p24g_register_sm_cmd_handler(P24G_SM_CMD_REPORT_RATE_CHANGE,    app_2p4g_handle_change_report_rate);
    p24g_register_sm_cmd_handler(P24G_SM_CMD_SET_STATE,             app_2p4g_handle_set_state);
    p24g_register_sm_cmd_handler(P24G_SM_CMD_SET_KB_MODE,           app_2p4g_handle_set_kb_mode);
    p24g_register_sm_cmd_handler(P24G_SM_CMD_MISC,                  app_2p4g_handle_misc);
}


void app_var_callback(tpsll_app_var_t *p_app_var, tpsll_access_type_e access_type,tpsll_app_var_id_e var_id)
{
    if (p_app_var == NULL) return ;

    switch (var_id) {
        case TPSLL_APP_VAR_SQUARE_SIDE_LEN:
            if (access_type == TPSLL_ACCESS_GET) {
                // p_app_var->square_side_len = g_square_side_len;
            } else {
                // g_square_side_len = p_app_var->square_side_len;
            }
            break;
        case TPSLL_APP_VAR_AUTOTEST_MODE:
//            extern unsigned char auto_test_mode;
            if (access_type == TPSLL_ACCESS_GET) {
                // p_app_var->auto_test_mode_en = auto_test_mode;
            } else {
                // auto_test_mode = p_app_var->auto_test_mode_en;
            }
            break;
        case TPSLL_APP_VAR_AUTOTEST_MOUSE:
            if (access_type == TPSLL_ACCESS_GET) {
                // p_app_var->auto_test_mouse_en = auto_test_mouse;
            } else {
                // auto_test_mouse = p_app_var->auto_test_mouse_en;
            }
            break;
        default:
            break;
    }
}


void pp_d24g_user_init(void)
{
    app_ctx.flash_save_report_rate = REPORT_RATE_NONE;
    app_ctx.dev_status = STATE_POWERON;

    app_p24g_sm_cmd_hanlder_init();

    // app_ctx.rr_idx = app_rr_get_index(tpsll_get_report_rate());

    tpsll_set_timeout_ms(P24G_TIMEOUT_TYPE_PAIR, P24G_PAIR_TIMEOUT_MS);
    tpsll_set_timeout_ms(P24G_TIMEOUT_TYPE_RECONN, P24G_RECONN_TIMEOUT_MS);

    tpsll_pm_optm_delay_time_set(TPSLL_PM_OPTM_LEVEL1, TPSLL_PM_OPTM_LEVEL1_START_TIME_S);
    tpsll_pm_optm_delay_time_set(TPSLL_PM_OPTM_LEVEL2, TPSLL_PM_OPTM_LEVEL2_START_TIME_S);
    tpsll_pm_optm_delay_time_set(TPSLL_PM_OPTM_LEVEL3, TPSLL_PM_OPTM_LEVEL3_START_TIME_S);

    tpsll_rf_init();

    pp_timer1_init();
    tpsll_stimer_init();
}


/**
 * @brief       in 2.4g mode  device status check
 * @param[in]    none
 * @return      none
 */
_attribute_ram_code_sec_ void app_pp_check_connect_status(void)
{
    static unsigned int led_tick;

    if(last_connect_status!=tpsll_get_state() )
    {
        led_tick=clock_time()|1;
        (void)led_tick;

        last_connect_status = tpsll_get_state();
        tlkapi_printf(APP_LOG_EN, "d24g_status=%d\r\n", last_connect_status);

        tpsll_stack_var_t var;
        tpsll_stack_get_var(&var,TPSLL_STACK_VAR_SET_PAIRING_INFO);
        if (var.pair_success_flag)
        {
            tpsll_stack_var_t tmp = {
                .pair_success_flag = 0,
            };
            tpsll_stack_set_var(&tmp, TPSLL_STACK_VAR_SET_SUCCESS_FLG);
            // if (flash_dev_info.side_id != var.side_id)
            // {
            //     flash_dev_info.side_id = var.side_id;
            //     tmemcpy(flash_dev_info.peer_addr, var.peer_mac, MAC_ADDR_LEN);
            //     tlkapi_send_string_data(APP_LOG_EN, "save peer addr", (unsigned char *)&flash_dev_info.side_id, 10);
            //     // DBG_GPIO_TOGGLE(APP_IO_EN, GPIO_PG6);
            //     save_data_to_flash(flash_sector_2p4_inf, sizeof(ST_FLASH_DEV_INFO), (unsigned char *)&flash_dev_info.side_id, (int *)&dev_info_idx);
            //     // DBG_GPIO_TOGGLE(APP_IO_EN, GPIO_PG6);
            // }
        }
    }
}


_attribute_ram_code_sec_ void pp_d24g_main_loop(void)
{
    if (g_stack_inited) {
        tpsll_stack_main_loop();
        app_pp_check_connect_status();
    } else {
        if (config.km_fifo_ptr && config.spp_fifo_ptr && app_ctx.flash_save_report_rate && app_dual_core_flag_ctx_ptr) {
            tpsll_stack_init(true, app_ctx.flash_save_report_rate, &config);
            tpsll_set_pair_encrypt_key(encrypt_key, encrypt_key_len);
            g_stack_inited = 1;
            tlkapi_printf(APP_LOG_EN, "tpsll stack inited\r\n");
        }
    }
}
