/********************************************************************************************************
 * @file    app_d24g.c
 *
 * @brief   This is the source file for Telink RISC-V MCU
 *
 * @author  2.4G Group
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
#include "app_public.h"
#include "tl_string.h"
#include "app_usb.h"

extern unsigned int tick_allow_pair_new_deviec_afer_power_on;
extern unsigned int tick_timer_pair_reconnect;

unsigned int g_flash_sector_mac_address=CFG_MY_ID_ADDR;

volatile unsigned int tick_pair_timeout;

unsigned char current_indicator = 0;

_attribute_aligned_(4)  app_ctx_t app_ctx;

static const report_rate_t rr_table[] =
{
    REPORT_RATE_4K,
    REPORT_RATE_2K,
    REPORT_RATE_1K,
    REPORT_RATE_500,
    REPORT_RATE_250,
    REPORT_RATE_125,
    REPORT_RATE_8K
};

#define RR_NUM  (sizeof(rr_table) / sizeof(report_rate_t))

#define DPI_NUM (sizeof(dpi_table) / sizeof(dpi_t))

_attribute_ram_code_sec_ uint8_t app_rr_get_index(report_rate_t rr)
{
    for (unsigned int i = 0; i < sizeof(rr_table) / sizeof(rr_table[0]); i++) {
        if (rr_table[i] == rr) {
            return i;
        }
    }
    return 0;  // not found, use default
}

#if 0
void p24g_rr_chg_test(void)
{
    static uint16_t g_rr_chg_cnt = 1;

    if(g_rr_chg_cnt % 50 == 0)
    {
        #if 0
        if(tpsll_get_report_rate() == REPORT_RATE_4K)
        {
            tpsll_change_report_rate(REPORT_RATE_8K);
        }else if(tpsll_get_report_rate() == REPORT_RATE_8K)
        {
            // gpio_toggle(GPIO_PH0);
            tpsll_change_report_rate(REPORT_RATE_4K);
        }
        #else
        static uint8_t last_rr_idx = 0;
        uint8_t idx = (last_rr_idx + 1) % RR_NUM;

        last_rr_idx = idx;
        tpsll_change_report_rate(rr_table[idx]);
        tlkapi_printf(1, "2rr:%d\n", rr_table[idx]);
        #endif
    }
    g_rr_chg_cnt++;
}
#endif

_attribute_ram_code_sec_  int app_rf_rx_data_push(unsigned char cmd,unsigned char *buf, int len)
{
    (void)cmd;
    (void)buf;
    (void)len;

    if(cmd==NORMAL_KB_DATA_CMD)
    {

        unsigned char tmp[8]={0,0,0,0,0,0,0,0};
         normal_key_t  *kb=(normal_key_t  *)&buf[0];

         tmp[0]=kb->ctrl;
         
         if(kb->len<=6)
         {
            tmemcpy(&tmp[2],&kb->nk[0],kb->len);
         }
        pp_fifo_push(&tx_fifo, NORMAL_KB_DATA_CMD, tmp, 8);
        //print_app_rf("rx_data=%d,%d,%d,%d\r\n",len,buf[0],buf[1],buf[2]);
    }
    else
    {
        // if(cmd==MOUSE_DATA)
        // {
        //     cnt_to_usb++;
        // }
        pp_fifo_push(&tx_fifo, cmd, buf, len);
        
    }

    return -1;
}
#if 1
//#define BIT_IS_POW2(x)     (!((x) & ((x)-1)))

_attribute_ram_code_sec_ bool rr_is_valid(report_rate_t rr)
{
    for (unsigned int i = 0; i < sizeof(rr_table) / sizeof(rr_table[0]); i++) {
        if (rr_table[i] == rr) {
            return true;
        }
    }
    return false;
}

/**
* @brief          Check the legality of the connect access code
* @param[in]      code  -  the connect access code
* @param[in]      adv_acc  -  the adv access code
* @return    none
*/

  int validateConnAccessCode(unsigned int code,unsigned int adv_acc)
  {
     unsigned char *p = (unsigned char*)&code;
     if((p[0] == p[1]) && (p[0] == p[2]) && (p[0] == p[3])){
         return 3;
     }
    #if 0
     if(code==PAIR_ACCESS_CODE)
     {
         return 5;
     }
    #endif
     unsigned int c = code ^ adv_acc;
     if(0 == c || (BIT_IS_POW2(c))) return 4;
 
     unsigned int cnt = 31;
     unsigned int b = code & 0x01;
     unsigned int zeroes = 0, ones = 0, transitions = 0, high_transitions = 0;
     if(b){
         ones = 1;
     }else{
         zeroes = 1;
     }
     while(cnt --){
         code >>= 1;
         unsigned int b1 = code & 0x01;
         if(b)
         {
             
             ones += (b == b1);
             
             zeroes = 1;
         }
         else
         {
             zeroes += (b == b1);
             
             ones = 1;
         }
         if(zeroes > 6 || ones > 6)
         {
             return 1;
         }
         transitions += (b != b1);
         
         if(cnt < 5)
         {
             high_transitions += (b != b1);
             
         }
         b = b1;
     }
     if(transitions > 24 || high_transitions < 2){
         return 2;
     }
 
     return 0;
     
 }

#endif


void app_var_callback(tpsll_app_var_t *p_app_var, tpsll_access_type_e access_type,tpsll_app_var_id_e var_id)
{
    if (p_app_var == NULL) return;

    switch (var_id) {
        case TPSLL_APP_VAR_SQUARE_SIDE_LEN:
            extern unsigned int g_square_side_len;
            if (access_type == TPSLL_ACCESS_GET) {
                p_app_var->square_side_len = g_square_side_len;
            } else {
                g_square_side_len = p_app_var->square_side_len;
            }
            break;

        default:
            break;
    }
}



_attribute_ram_code_sec_ static void app_2p4g_event_handler(tpsll_evt_t evt, void *data, uint16_t len)
{
    TL_UNUSED(data);
    TL_UNUSED(len);

    switch(evt) {
    case TPSLL_EVT_STACK_START:
        tick_allow_pair_new_deviec_afer_power_on = clock_time() | 1;
        tick_timer_pair_reconnect = clock_time() | 1;
        tlkapi_printf(APP_LOG_EN, "2p4g stack start ...\n");
        break;

    case TPSLL_EVT_DEV_CONNECTED:
        app_ctx.dev_status = STATE_CONNECTED;

        if (app_ctx.paired_dev_type == DEVICE_TYPE_KB) {
            app_ctx.pending_led_sync = 1;
        }

        tlkapi_send_string_data(APP_LOG_EN, "connected", data, len);
        break;

    case TPSLL_EVT_DEV_DISCONNECTED:
        app_ctx.dev_status = STATE_DISCONNECTED;
        tlkapi_printf(APP_LOG_EN, "disconnected\n");
        break;

    case TPSLL_EVT_DEV_PAIRED:
        //process after pairing
        {
            tpsll_stack_var_t var;
            tpsll_stack_get_var(&var,TPSLL_STACK_VAR_SET_PAIRING_INFO);
            if (flash_dev_info.side_id != var.side_id)
            {
                flash_dev_info.side_id = var.side_id;
                flash_dev_info.peer_type = var.connecting_dev;
                // ll_inf.paired_dev_type = var.connecting_dev;
                app_ctx.paired_dev_type = var.connecting_dev;
                uint16_t peer_tag = fnv1a_16(flash_dev_info.peer_addr, MAC_ADDR_LEN);
                app_ctx.peer_tag = (peer_tag & 0x3fff) | (app_ctx.paired_dev_type << 14);
                tmemcpy(flash_dev_info.peer_addr, var.peer_mac, MAC_ADDR_LEN);
                tlkapi_send_string_data(APP_LOG_EN, "saving pairing info to flash...", 0, 0);
                save_data_to_flash(flash_sector_2p4_inf, sizeof(ST_FLASH_DEV_INFO), &flash_dev_info.side_id, &dev_info_idx);
            }
        }
        break;

    case TPSLL_EVT_REPORT_RATE_CHANGED:
        tlkapi_printf(APP_LOG_EN, "report rate changed to 0x%02X\n", ((uint8_t *)data)[0]);
        // app_ctx.save_rr_flag = true;
        // app_ctx.rr_idx =  app_rr_get_index(tpsll_get_report_rate());
        app_ctx.report_rate = tpsll_get_report_rate();
        
        break;

    case TPSLL_EVT_ENTER_SUSPEND:
        //process before enter suspend
        break;

    case TPSLL_EVT_EXIT_SUSPEND:
        //process after exit suspend
        break;
    case TPSLL_EVT_SPP_DATA_RECV:
        //process spp rx
        //tlkapi_send_string_data(APP_LOG_EN, "spp_rx", data, len);

        uint8_t *spp_data = (uint8_t *)data;

        if (spp_data[0] == TPSLL_SPP_TEST_DATA)
        {
            static uint16_t last_id = 0xffff;
            uint8_t cur_id = spp_data[1];

            if (last_id == 0xffff) {
                last_id = cur_id;
            } else {
                uint8_t expected_id = (uint8_t)(last_id + 1);

                if (cur_id != expected_id) {
                    // tlkapi_printf(APP_LOG_EN, "expected %u, got %u\n", 
                    //    expected_id, cur_id);
                }
                last_id = cur_id;
            }
            #if 0
            for (int i = 2; i < len; i++) {
                if (spp_data[i] != ((spp_data[1] + (i - 1)) & 0xff)) {
                    // tlkapi_send_string_data(APP_LOG_EN, "err spp_rx", spp_data, len);
                    break;
                }
            }
            #endif
        }

        break;
    case TPSLL_EVT_USR_SPEC_ACK_RECV:
        tlkapi_printf(APP_LOG_EN,"user data send success: %x\n", *((uint8_t*)data));
        break;
    default:
        break;
    }
}

void app_deferred_led_sync_check(void)
{
    if (!app_ctx.pending_led_sync)
    {
        return;
    }

    uint8_t led_buf[2] = {TPSLL_SPP_LED_STATUS, current_indicator};

    int ret = tpsll_send_spp_data(led_buf, sizeof(led_buf), 3, NULL, NULL);

    if (ret == 0)
    {
        app_ctx.pending_led_sync = 0;
    }
}

extern unsigned char app_consume_key_report_to_usb(unsigned char *buf);
extern unsigned char app_system_key_report_to_usb(unsigned char *buf);
extern unsigned char app_all_key_report_to_usb(unsigned char *buf);
extern unsigned char *usbd_get_device_descriptor(unsigned char bus);

static tpsll_stack_config_t config = {
    .app_var_cb = app_var_callback,
    .app_cb = {
        .app_mouse_report = app_mouse_report_to_usb,
        .app_consume_key_report = app_consume_key_report_to_usb,
        .app_system_key_report = app_system_key_report_to_usb,
        .app_all_key_report = app_all_key_report_to_usb,
        .usbd_get_dev_descp = usbd_get_device_descriptor,
        .app_get_usb_status = app_get_usb_wake_status,
        .app_set_usb_status = app_set_usb_wake_status,
        .draw_square_test = Draw_a_square_test,
        .app_rf_hw_init = pp_rf_hw_init,
    }
};

void pp_d24g_user_init(void)
{
    tpsll_rf_init();

    tpsll_stack_init(&config);

    app_ctx.dev_status = STATE_POWERON;

    tpsll_register_callback(TPSLL_EVT_STACK_START, app_2p4g_event_handler);
    tpsll_register_callback(TPSLL_EVT_REPORT_RATE_CHANGED, app_2p4g_event_handler);
    tpsll_register_callback(TPSLL_EVT_DEV_CONNECTED, app_2p4g_event_handler);
    tpsll_register_callback(TPSLL_EVT_DEV_DISCONNECTED, app_2p4g_event_handler);
    tpsll_register_callback(TPSLL_EVT_DEV_PAIRED, app_2p4g_event_handler);
    tpsll_register_callback(TPSLL_EVT_ENTER_SUSPEND, app_2p4g_event_handler);
    tpsll_register_callback(TPSLL_EVT_EXIT_SUSPEND, app_2p4g_event_handler);
    tpsll_register_callback(TPSLL_EVT_SPP_DATA_RECV, app_2p4g_event_handler);
    tpsll_register_callback(TPSLL_EVT_USR_SPEC_ACK_RECV, app_2p4g_event_handler);

    pp_timer0_init();
    pp_timer1_init();

    tpsll_set_proc_ir_btn_us(30);
}

