/********************************************************************************************************
 * @file    app_usb.c
 *
 * @brief   This is the source file for Telink RISC-V MCU
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
#include "app_usb_km_descriptor.h"
#include "app_public.h"
#include "tl_usb/class/hid/usbd_hid.h"
#include "tl_string.h"
#include "version_cfg.h"

#define USB_BUFF_UNIT              4 /* one word */

volatile   unsigned int  vbus_status=1;

static unsigned char          cdc_out_buffer[USB_OTA_LENGTH]={0,0};

static unsigned char          cdc_in_buffer[USB_OTA_LENGTH]={0,0};

static volatile unsigned short cdc_out_data_len;


volatile unsigned char ep_is_busy[8] = {0,0,0,0,0,0,0,0};

volatile   unsigned int  usb_status_flag;
volatile   unsigned int  last_usb_status_flag;
volatile   unsigned int  usb_status_tick;
volatile   unsigned char remote_wakeup = USB_ACTIVE_STATE;
// volatile   unsigned int  usb_status_flag_retain;
static volatile unsigned char usb_suspend_flag = 0;

static usbd_driver_t usbd_hid_driver;

uint8_t is_ota_ing = 0;

unsigned int   first_send_flag=0;

extern unsigned char g_usbd_remote_wakeup_en;

extern uint32_t esd_test_tick;
u8      host_cmd[8];


_attribute_ram_code_sec_ void app_usb_bus_reset_init(void)
{
    // app_ctx.is_usb_desc_done = 0;
    tpsll_set_usb_start_flag(0);

    esd_test_tick = 0;
    // usb_status_flag=0;
    last_usb_status_flag=0xffff;
    usb_status_tick=0;
    for(int i=0;i<USBD_SUPPORT_ENDPOINT_MAX;i++)
    {
        ep_is_busy[i]=0;
    }
    pp_fifo_reset(&tx_fifo);
    if(!app_get_usb_enum_status())
        usbd_hid_reset_interface_data();
    remote_wakeup = USB_EXIT_SUSPEND;
    usbd_ep_read(0, CDC_DATA_OUT_ENDPOINT_ADDRESS, cdc_out_buffer, sizeof(cdc_out_buffer));
}
_attribute_ram_code_sec_ void app_usb_status_clear(unsigned int bit)
{
    usb_status_flag&=0xffff-bit;
    usb_status_tick=stimer_get_tick()|1;
}
_attribute_ram_code_sec_ void app_usb_status_set(unsigned int bit)
{
    usb_status_flag|=bit;
    usb_status_tick=stimer_get_tick()|1;
}

_attribute_ram_code_sec_ void app_usb_status_check(void)
{
    if((usb_status_flag & USB_OK_FLAG_APP) == 0)
    {
        // First time trigger: record tick
        if(usb_status_tick == 0)
        {
            usb_status_tick = stimer_get_tick() | 1;
        }
        // Check 100ms timeout
        else if(clock_time_exceed(usb_status_tick, 100000))
        {
            if(usb_status_flag & CONFIG_SET_FLAG_APP)
            {
                // Check if USB enumerated successfully
                if(usb_status_flag & MAP_GET_FLAG_APP)
                {
                    app_usb_status_set(USB_OK_FLAG_APP);
                    first_send_flag = 1;
                    esd_test_tick = 0;
                    tpsll_usb_enum_done_handler();
                    usb_status_tick = 0;
                }
                // USB not ready: check 2s timeout for recovery
                else if(clock_time_exceed(usb_status_tick, 2000000))
                {
                    app_usb_status_set(USB_OK_FLAG_APP);
                    first_send_flag = 1;
                    esd_test_tick = 0;
                    tpsll_usb_enum_done_handler();
                    usb_status_tick = 0;
                }
            }
        }
    }
    #if 0
     unsigned int x=reg_usb_dctl;
    // if(reg_usb_dctl&FLD_USB_DCTL_SFTDISCON)
     {
        
        x=reg_usb_dsts;
        app_debug("reg_usb_dsts=%04x\r\n",x);

        
     }
    #endif
    
     if(last_usb_status_flag!=usb_status_flag)
     {
        last_usb_status_flag=usb_status_flag;
        print_app_usb("usb_status=%04x\r\n",usb_status_flag);
     }
    
}
_attribute_ram_code_sec_ unsigned char app_usb_epin_send(unsigned char ep_addr,unsigned char *buf,unsigned char len)
{
    unsigned char ep_num = ep_addr&0x7f;
    #if 0
    if((usb_status_flag&USB_OK_FLAG_APP)==0)
    {
        return 1;
    }
    if(ep_is_busy[ep_num]!=0)
    {
        app_debug("ep is_busy\r\n");
        return 2;
    }
    #endif
    if(len==0)
    {
        return 3;
    }

    if (g_usbd_remote_wakeup_en)
    {
        usb0hw_remote_wakeup();
    }

    ep_is_busy[ep_num]=1;  //must  before  usbd_ep_write
    usbd_ep_write(0, ep_addr, buf, len);

    return 0;
}
_attribute_ram_code_sec_ unsigned char app_usb_ep_is_idle(unsigned char ep_num)
 {
    if((usb_status_flag&USB_OK_FLAG_APP)==0)
    {
        return 1;
    }
    if(ep_is_busy[ep_num]!=0)
    {
        //app_debug("ep is_busy\r\n");
        return 2;
    }

    return 0;
 }
_attribute_ram_code_sec_ void app_usb_ep1_in_complete_handler(unsigned char bus, unsigned char ep_index, unsigned int len)
{
    (void)bus;
    (void)ep_index;
    (void)len;
    unsigned char ep_num = ep_index&0x7f;
    ep_is_busy[ep_num]=0;
    //usb_log("usbd keyboard send complete callback ep_addr = %d, len = %d\r\n", ep_addr, len);

}
_attribute_ram_code_sec_ void app_usb_ep2_in_complete_handler(unsigned char bus, unsigned char ep_index, unsigned int len)
{
    (void)bus;
    (void)ep_index;
    (void)len;
    unsigned char ep_num = ep_index&0x7f;
    ep_is_busy[ep_num]=0;
}
_attribute_ram_code_sec_ void app_usb_ep3_in_complete_handler(unsigned char bus, unsigned char ep_index, unsigned int len)
{
    (void)bus;
    (void)ep_index;
    (void)len;
    unsigned char ep_num = ep_index&0x7f;
    ep_is_busy[ep_num]=0;
}

_attribute_ram_code_sec_ void app_mouse_usb_ep_buff_fill(void)
{
    static unsigned char tmp[8]={1,0,0,0,0,0,0,0};// first is report id
    unsigned char len=HID_MOUSE_IN_ENDPOINT_SIZE;

    if(usbd_hid[0].protocol_mode == 0)// if((usb_status_flag&BIOS_MODE_FLAG_APP))
    {
        len=3;
    }

//    usb0hw_flush_tx_fifo(HID_MOUSE_IN_ENDPOINT_NUM);
    app_usb_epin_send(HID_MOUSE_IN_ENDPOINT_ADDRESS, tmp, len);
}

_attribute_ram_code_sec_ unsigned char app_mouse_usb_ep_buf_has_data(void)
{
    return (((reg_usb_dtxfsts(HID_MOUSE_IN_ENDPOINT_NUM) & FLD_USB_DTXFSTS_INEPTXFSPCAVAIL)) < APP_MOUSE_ENDPOINT_BUFF_MIN_SIZE);
}

_attribute_ram_code_sec_ unsigned char app_mouse_8k_usb_ep_buf_has_data(void)
{
    return (((reg_usb_dtxfsts(HID_MOUSE_IN_ENDPOINT_NUM) & FLD_USB_DTXFSTS_INEPTXFSPCAVAIL)) < APP_MOUSE_ENDPOINT_BUFF_SIZE);
}

_attribute_ram_code_sec_ void app_usb_ep4_in_complete_handler(unsigned char bus, unsigned char ep_index, unsigned int len)
{
    (void)bus;
    (void)ep_index;
    (void)len;
    unsigned char ep_num = ep_index&0x7f;
    ep_is_busy[ep_num]=0;
}
void usbd_set_configuration_callback(unsigned char bus, unsigned char config_num)
{
    (void)bus;
    (void)config_num;
    usb_log("usb set config callback\r\n");

    /* receive first cdc out data. */
    usbd_ep_read(bus, CDC_DATA_OUT_ENDPOINT_ADDRESS, cdc_out_buffer, sizeof(cdc_out_buffer));
}

#if RF_PKT_IDX_DEBUG_EN
static void debug_gpio_with_cnt(int16_t cnt)
{
    if(cnt < 0)
    {
        cnt = -cnt;
    }
    for(uint8_t i = 0; i < cnt; i++)
    {
        ZW_TG(GPIO_PH3);
    }
}
#endif

extern void print_ms_buf(void);
static volatile  int16_t acc_xp = 0;
static volatile  int16_t acc_xn = 0;

_attribute_ram_code_sec_ unsigned char app_mouse_report_to_usb(unsigned char *buf)
{
    static unsigned char tmp[8]={1,0,0,0,0,0,0,0};// first is report id
    static volatile unsigned char prev_btn = 0;
//    static volatile unsigned char btn_rp_cnt = 0;
    unsigned char len=HID_MOUSE_IN_ENDPOINT_SIZE;
    unsigned char  status=0;

    tmemset(tmp,0,sizeof(tmp));
    tmp[0] = 1;
    #if RF_PKT_IDX_DEBUG_EN
    // gpio_toggle(GPIO_PH0);
    static uint8_t pre_v = 0;
    static uint32_t zcont = 0;
    zcont++;
    // if(pre_v != (buf[1] - 1) && pre_v != 0xff && zcont > 10)
    if(pre_v != (buf[1] - 1) && pre_v != 0x8 && zcont > 10)
    {
        // ZW_TG(GPIO_PG7);
        // print_ms_buf();
        
        // pre_v = buf[1];
        // tlkapi_printf(APP_LOG_EN, "pre_v:%x v:%x", pre_v, buf[1]);
    }

    #if 1

    pre_v = buf[1];

    static int16_t acc_x = 0;
    static uint8_t x_s = 0;
    static uint8_t x_e = 0;
    static uint8_t y_c = 0;

    int16_t x_m = *((int16_t *)(buf + 1));
    int16_t y_m = *((int16_t *)(buf + 3));
    debug_gpio_with_cnt(x_m);
    // debug_gpio_with_cnt(y_m);

    #if 0
    if(x_m > 0)
    {
        acc_xp += x_m;
    }else if(x_m < 0)
    {
        acc_xn += x_m;
    }

    if(x_m != 0)
    {
        // ZW_TG(GPIO_PG5);
        // acc_x += buf[1];
        acc_x += x_m;
        x_s = 1;
    }
    if(x_s == 1 && y_m != 0)
    // if(x_s == 1)
    {
        x_e = 1;
        y_c++;
    }
    #endif

    #if 0

    if(x_e ==1 && y_c > 10)
    {
        // ZW_TG(GPIO_PH0);
        if(acc_x > 0 && acc_x != 200)
        // if(acc_x > 0 && acc_x != 50)
        {
            ZW_TG(GPIO_PH4);
            // ZW_TG(GPIO_PH0);
            tlkapi_printf(APP_LOG_EN, "acc_x=%d\r\n", acc_x);
        }else if(acc_x < 0 && acc_x != -200 ){
        // }else if(acc_x < 0 && acc_x != -50 ){
            ZW_TG(GPIO_PH4);
            tlkapi_printf(APP_LOG_EN, "nacc_x=%d\r\n", acc_x);
            ZW_TG(GPIO_PH4);
        }
        x_e = 0;
        x_s = 0;
        acc_x = 0;
        y_c = 0;
    }
        #endif
    #endif

    #endif
    
    #if 0
    static uint32_t ucnt = 0;
    // if (ucnt < 10)
    if (ucnt > 50 && ucnt < 60)
    {
        // tlkapi_send_string_data(APP_LOG_EN, "usb", buf, 6);
        // debug_gpio_with_cnt();
        // debug_gpio_with_cnt(buf[1]);
    }
    ucnt++;
    #endif

    #if !APP_REPORT_MOUSE_TO_USB_EN
    return 0;
    #endif

    if (!tpsll_get_usb_start_flag()) {
       
        return 3;
    }

//    if ((buf[0] == 0) && (buf[1] == 0) && (buf[2] == 0) &&
//            (buf[3] == 0) && (buf[4] == 0) && (buf[5] == 0)) {
//        return 4;
//    }

    status=app_usb_ep_is_idle(HID_MOUSE_IN_ENDPOINT_NUM);
    if(status!=0)
    {
        //  gpio_toggle(GPIO_PG7);
        return status;
    }

//    if (((reg_usb_dtxfsts(HID_MOUSE_IN_ENDPOINT_NUM) & FLD_USB_DTXFSTS_INEPTXFSPCAVAIL) * 4) < 2*4 /* size of EP2 buffer */)
//    {
//        usb0hw_flush_tx_fifo(HID_MOUSE_IN_ENDPOINT_NUM);
////        gpio_toggle(GPIO_PH5);
//    }
    if(usbd_hid[0].protocol_mode == 1)// if((usb_status_flag&BIOS_MODE_FLAG_APP)==0)
    {
        short x = (short)(buf[1] | (buf[2] << 8));
        short y = (short)(buf[3] | (buf[4] << 8));
        unsigned char btn = buf[0];
        unsigned char z_wheel = buf[5];
        unsigned char t_wheel = buf[6];

        if (x == 0 && y == 0 && z_wheel == 0 && t_wheel == 0 && btn == prev_btn) {
            return 3;
        }

        #if 0
        if (x == 0 && y == 0 && wheel == 0 && btn == prev_btn && btn_rp_cnt >= 3) {
            return 3;
        }
        if(prev_btn != btn)
        {
            btn_rp_cnt = 0;
            prev_btn = btn;
        }
        #endif


        prev_btn = btn;
        tmemcpy(&tmp[1], &buf[0], 7);

    }
    else
   {
        #if 0
        tmp[0]=buf[0];
       
        
        tmp[1]=buf[1];
        tmp[2]=buf[3];
        #else   
                tmp[0]=buf[0];
             s16 xy=(s16)(buf[1]+256*buf[2]);
             if(xy>127)
             {
                 tmp[1]=127;
             }
             else if(xy<-127)
             {
                tmp[1]=-127;
             }
            else
            {
                tmp[1]=buf[1];
            }

            xy=(s16)(buf[3]+256*buf[4]);
             if(xy>127)
             {
                 tmp[2]=127;
             }
             else if(xy<-127)
             {
                tmp[2]=-127;
             }
            else
            {
                tmp[2]=buf[3];
            }
        #endif
        len=3;
    }

    //if(tmp[1] > 0)
    //{
    //    gpio_toggle(GPIO_PG7);
    //}
    // ZW_TG(GPIO_PG7);
    
    return app_usb_epin_send(HID_MOUSE_IN_ENDPOINT_ADDRESS, tmp, len);
}

_attribute_ram_code_sec_ unsigned char app_normal_key_report_to_usb(unsigned char *buf)
{
    static unsigned char tmp[8]={0,0,1,0,0,0,0,0};
    unsigned char  status=0;
    status=app_usb_ep_is_idle(HID_KEYBOARD_IN_ENDPOINT_NUM);
    if(status!=0)
    {
        return status;
    }
    tmemcpy(&tmp[0],&buf[0],8);
    return app_usb_epin_send(HID_KEYBOARD_IN_ENDPOINT_ADDRESS, tmp, 8);
}

_attribute_ram_code_sec_ unsigned char app_rf_normal_key_report_to_usb(unsigned char *buf, unsigned char len)
{
    static unsigned char tmp[8]={0,0,0,0,0,0,0,0};
    unsigned char  status=0;
    status=app_usb_ep_is_idle(HID_KEYBOARD_IN_ENDPOINT_NUM);
    if(status!=0)
    {
        return status;
    }
    tmemset(&tmp[0],0,sizeof(tmp));
    // if(len)
    {
        tmp[0] = buf[0];
        tmp[1] = 0;
        tmemcpy(&tmp[2],&buf[1],len - 1);
    }
//    else{
        // tmemset(&tmp[2],0,sizeof(tmp));
//    }
    return app_usb_epin_send(HID_KEYBOARD_IN_ENDPOINT_ADDRESS, tmp, 8);
}

_attribute_ram_code_sec_ unsigned char app_consume_key_report_to_usb(unsigned char *buf)
{
    static unsigned char tmp[3]={2,0,0}; // first is report id
    unsigned char  status=0;
    status=app_usb_ep_is_idle(HID_MOUSE_IN_ENDPOINT_NUM);
    if(status!=0)
    {
        return status;
    }
    tmemcpy(&tmp[1],&buf[0],2);
    return app_usb_epin_send(HID_MOUSE_IN_ENDPOINT_ADDRESS, tmp, 3);
}
_attribute_ram_code_sec_ unsigned char app_system_key_report_to_usb(unsigned char *buf)
{
    static unsigned char tmp[2]={3,0}; // first is report id 
    unsigned char  status=0;
    status=app_usb_ep_is_idle(HID_MOUSE_IN_ENDPOINT_NUM);
    if(status!=0)
    {
        return status;
    }
    tmp[1]=buf[0];
    return app_usb_epin_send(HID_MOUSE_IN_ENDPOINT_ADDRESS, tmp, 2);
}
_attribute_ram_code_sec_ unsigned char app_all_key_report_to_usb(unsigned char *buf)
{
    static unsigned char tmp[17]={8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // first is report id 
    unsigned char  status=0;
    status=app_usb_ep_is_idle(HID_ALL_KB_IN_ENDPOINT_NUM);
    if(status!=0)
    {
        return status;
    }
    tmemcpy(&tmp[1],&buf[0],16);
    return app_usb_epin_send(HID_ALL_KB_IN_ENDPOINT_ADDRESS, tmp, 17);
}


_attribute_ram_code_sec_ unsigned char app_ep4_ota_report_to_usb(void)
{
    unsigned char  status=0;
    status=app_usb_ep_is_idle(CDC_DATA_IN_ENDPOINT_ADDRESS&0x7f);
    if(status!=0)
    {
        return status;
    }
   
    return app_usb_epin_send(CDC_DATA_IN_ENDPOINT_ADDRESS, cdc_in_buffer, CDC_DATA_IN_ENDPOINT_SIZE);
}
void usbd_cdc_epout_callback(unsigned char bus, unsigned char ep_addr, unsigned int len)
{
    (void)bus;
    (void)ep_addr;
    (void)len;
   // usb_log("usbd cdc out complete callback ep_addr = %d, len = %d\r\n", ep_addr, len);
   if((cdc_out_buffer[0] == USB_VENDOR_REPORT_ID) && (len == USB_OTA_LENGTH))
   {
         cdc_out_data_len = len;
         memcpy_(&cdc_in_buffer[0],&cdc_out_buffer[0],USB_OTA_LENGTH);
   }
  
    
    //cdc_in_buffer[2]=cdc_out_data_len;
    usbd_ep_read(0, CDC_DATA_OUT_ENDPOINT_ADDRESS, cdc_out_buffer, sizeof(cdc_out_buffer));

    
    
}

void app_usb_init(void)
{
    #if USB_OTA_ENABLE  
        usb_ota_init();
#endif  
    usbd_hid_driver.driver_num          = 0;
    usbd_hid_driver.usbd_driver_handler = usbd_hid_interface_request_handler;
    usbd_driver_register(0, &usbd_hid_driver, 0, HID_KEYBOARD_IN_ENDPOINT_ADDRESS);
    usbd_endpoint_register(0, HID_KEYBOARD_IN_ENDPOINT_ADDRESS, app_usb_ep1_in_complete_handler);

    usbd_driver_register(0, &usbd_hid_driver, 1, HID_MOUSE_IN_ENDPOINT_ADDRESS);
    usbd_endpoint_register(0, HID_MOUSE_IN_ENDPOINT_ADDRESS, app_usb_ep2_in_complete_handler);

    usbd_driver_register(0, &usbd_hid_driver, 2, HID_ALL_KB_IN_ENDPOINT_ADDRESS);
    usbd_endpoint_register(0, HID_ALL_KB_IN_ENDPOINT_ADDRESS, app_usb_ep3_in_complete_handler);

    // tl_usbd_enum_done_register(tpsll_usb_enum_done_handler);
    tl_usbd_sof_register(tpsll_usb_sof_handler);
    usbd_driver_register(0, &usbd_hid_driver, 3, CDC_DATA_IN_ENDPOINT_ADDRESS);//vendor  in  ep
    usbd_driver_register(0, &usbd_hid_driver, 3, CDC_DATA_OUT_ENDPOINT_ADDRESS);//vendor  out  ep
    usbd_endpoint_register(0, CDC_DATA_IN_ENDPOINT_ADDRESS, app_usb_ep4_in_complete_handler);
    
     usbd_endpoint_register(0, CDC_DATA_OUT_ENDPOINT_ADDRESS, usbd_cdc_epout_callback);


    
    usb0hw_init(USB0_SPEED_HIGH);//first must init 

    usb0hw_set_grxfsiz(0x100);
    usb0hw_set_epin_size(USB0_EP0, 0x100, 64);
    usb0hw_set_epin_size(USB0_EP1, 0x100 + 64, APP_KEYBOARD_ENDPOINT_BUFF_SIZE);
    usb0hw_set_epin_size(USB0_EP2, 0x100 + 64 + APP_KEYBOARD_ENDPOINT_BUFF_SIZE, APP_MOUSE_ENDPOINT_BUFF_SIZE);
    usb0hw_set_epin_size(USB0_EP3, 0x100 + 64 + APP_KEYBOARD_ENDPOINT_BUFF_SIZE + APP_MOUSE_ENDPOINT_BUFF_SIZE, APP_KEYBOARD_ALL_KEY_ENDPOINT_BUFF_SIZE);
    usb0hw_set_epin_size(USB0_EP4, 0x100 + 64 + APP_KEYBOARD_ENDPOINT_BUFF_SIZE + APP_MOUSE_ENDPOINT_BUFF_SIZE + APP_KEYBOARD_ALL_KEY_ENDPOINT_BUFF_SIZE, 64);


    plic_set_priority(IRQ_USB0, IRQ_PRI_LEV1);
    plic_interrupt_enable(IRQ_USB0);
}
_attribute_ram_code_sec_ void app_usb_kb_report_test(void)
{
    static unsigned char kb_data[8]={0,0,0,0,0,0,0,0};
     delay_ms(2);
     unsigned char  status=0;
    status=app_usb_ep_is_idle(HID_KEYBOARD_IN_ENDPOINT_NUM);
    if(status!=0)
    {
        return ;
    }

    if(kb_data[2]==0)
    {
         kb_data[2]     = 0x04;
    }
    else
    {
         kb_data[2]     = 0x0;
    }
    app_normal_key_report_to_usb(kb_data);
        
    

}

_attribute_ram_code_sec_  short data_test(void)
{
    static short cnt = 0;

    cnt++;

    if (cnt > 4)
        cnt = -4;

    if (cnt == 0)
        gpio_toggle(GPIO_PH4);

    return cnt;
}

//extern unsigned char  Draw_two_point_test(signed short *x);
_attribute_ram_code_sec_ void app_usb_mouse_report_test(void)
{
    static t_mouse_data ms_data;
//   unsigned char  status=0;
//   status=app_usb_ep_is_idle(HID_MOUSE_IN_ENDPOINT_NUM);
//  if(status!=0)
//  {
//      return ;
//  }
//
//  Draw_a_square_test(&ms_data.x,&ms_data.y);
//  Draw_two_point_test(&ms_data.x);
    ms_data.x = data_test();

    if (!app_mouse_report_to_usb(&ms_data.btn)) {
        gpio_toggle(GPIO_PH2);
        gpio_toggle(GPIO_PH2);
    } else {
        gpio_toggle(GPIO_PH2);
//        gpio_toggle(GPIO_PH1);
    }
    
}
unsigned char test_all_kb(unsigned char *buf)
{
    static unsigned char tmp[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  
    if(buf[2]==0)
    {
        tmp[0]=0;
    }
    else
    {
        tmp[0]=0x10;
    }
    return app_all_key_report_to_usb(tmp);
}
_attribute_ram_code_sec_ void app_hid_set_report(unsigned char bus, unsigned char hid_itf, unsigned char report_id, unsigned char report_type, unsigned char *report, unsigned short report_len)
{
    (void)bus;

    if((report_id==0)&&(report_len==1)&&(report_type==2)&&(hid_itf==1))
    {
        app_notfiy_led_to_kb(report[0]);
        extern unsigned char current_indicator;
        current_indicator = report[0];
    }
    if((report_id == 0x05)&&(report_len==8))
    {
        memcpy_(&host_cmd[0],&report[0],report_len);
        print_app_usb("receive hid cmd =%02x,%02x,%02x,%02x,%02x,%02x,%02x\r\n",host_cmd[0],host_cmd[1],host_cmd[2],host_cmd[3],host_cmd[4],host_cmd[5],host_cmd[6]);
    }
    #if 0
    print_app_usb("hid_itf=%d,report_id=%d,report_type=%d,report_len=%d ",hid_itf,report_id,report_type,report_len);

    for(int i=0;i<report_len;i++)
    {
        print_app_usb("hid_itf=%d",report[i]);
    }
    print_app_usb("\r\n");
    #endif
}

_attribute_ram_code_sec_ void app_usb_report_to_pc(void)
{
    unsigned char  *p= pp_fifo_get_ptr(&tx_fifo);
    if(p!=0)
    {
        unsigned char cmd=p[1];
        unsigned char ret=0; 
        //unsigned char len=p[0];
        if(cmd==MOUSE_DATA)
        {
            ret=app_mouse_report_to_usb(&p[3]);
        }
        else if(cmd==NORMAL_KB_DATA_CMD)
        {
            ret=app_normal_key_report_to_usb(&p[2]);
            //ret=test_all_kb(&p[2]);
        }
        else if(cmd==CONSUME_KB_DATA_CMD)
        {
            ret=app_consume_key_report_to_usb(&p[2]);
        }
        else if(cmd==SYSTEM_KB_DATA_CMD)
        {
            ret=app_system_key_report_to_usb(&p[2]);
        }
        else if(cmd==ALL_KB_DATA_CMD)
        {
            ret=app_all_key_report_to_usb(&p[2]);
        }
        if(ret==0)
        {
            pp_fifo_pop(&tx_fifo);
        }
    }
    
}

_attribute_ram_code_sec_ void app_usb_main_loop(void)
{
    app_usb_status_check();
  #if 1
//    static unsigned int tick=0;
//
//    if(clock_time_exceed(tick, 100)) {
//        tick=stimer_get_tick();
//        app_usb_mouse_report_test();
//    }
  #else  //user fifo 
     //app_get_mouse_data();
   //app_get_kb_data();
   if(vbus_status==1)
   {
 
     app_usb_report_to_pc();
   }
    #endif
}
#if USB_OTA_ENABLE

#if MCU_RUN_IN_NVM
#define NVM_PAGE_SIZE      256
#define OTA_PKT_SIZE       16
#endif

#define  IC_BIN_MARK  0x544c4e4b
#define  IC_BOOT_MARK_ADDR  0x20
#define  IC_BIN_FW_SIZE_ADDR  0x18


#if MCU_RUN_IN_NVM  //has bootloader
#define  IC_FIRST_FW_ADDR   IMAGE_OFFSET//if has bootloader This address must be consistent with the bootloader
#define  IC_SECOND_FW_ADDR    0x30000 // if has bootloader This address must be consistent with the bootloader
#else
#define  IC_FIRST_FW_ADDR   0x00000//if has bootloader This address must be consistent with the bootloader
#define  IC_SECOND_FW_ADDR    0x20000 // if has bootloader This address must be consistent with the bootloader

#endif

#if MCU_RUN_IN_NVM
// raise the Vcore to 1.1V once at OTA start and restore it at OTA end
volatile pm_dig_vol_mode_e g_nvm_ota_saved_dig_vol = DIG_VOL_1V1_MODE;

void usb_ota_dig_vol_raise(void)
{
    g_nvm_ota_saved_dig_vol = app_dig_vol_raise_to_1v1();
}

void usb_ota_dig_vol_restore(void)
{
    app_dig_vol_restore(g_nvm_ota_saved_dig_vol);
    g_nvm_ota_saved_dig_vol = DIG_VOL_1V1_MODE;
}
#endif


 unsigned char ota_start_flag = 0;
 u32 ota_start_tick;
unsigned char  ota_success_flag=0;
unsigned int bin_crc;

//extern  unsigned int ota_program_offset;
 
/**
 * @brief       This function crs 16
 * @param[in]   len - data len
 * @param[in]   pD  - data
 * @return     crc 
 * @note        
 */
_attribute_ram_code_sec_ unsigned short crc16_user (unsigned char *pD, int len)
{
    unsigned short crc = 0xffff;//crc16 init ffff
    unsigned short  crc16_poly1[2] = {0, 0xa001};
    int i,j;

    for(j=len; j>0; j--)//for every data
    {
        unsigned char ds = *pD++;//pointer pd
        for(i=0; i<8; i++)
        {
            crc = (crc >> 1) ^ crc16_poly1[(crc ^ ds ) & 1];//deal every bit
            ds = ds >> 1;//right move 1 bit
        }
    }

    return crc;//return crc
}
static const unsigned long fw_crc32_half_tbl[16] = {
    0x00000000,
    0x1db71064,
    0x3b6e20c8,
    0x26d930ac,
    0x76dc4190,
    0x6b6b51f4,
    0x4db26158,
    0x5005713c,
    0xedb88320,
    0xf00f9344,
    0xd6d6a3e8,
    0xcb61b38c,
    0x9b64c2b0,
    0x86d3d2d4,
    0xa00ae278,
    0xbdbdf21c};

    #define FW_READ_SIZE 256 ///16 ///256 require more stack space



/***********************************
 * this function must be called after the function sys_init.
 * sys_init will set the ota_program_offset value.
 */
/**
 * @brief       This function is used to check the firmware is ok or not
 * @param[in]   crc_init_value - the initial value of CRC
 * @return      0 - CRC is check success
 *              1 - CRC is check fail
 */
_attribute_ram_code_sec_ bool flash_fw_check_app(u32 code_addr)
{
    static u32 fw_crc_init = 0xFFFFFFFF;
    

    /////find the real FW flash address
    u32 fw_flashAddr = code_addr;
    u32 fw_size;
    u8  fw_tmpdata[FW_READ_SIZE];    ///

    read_storage_to_ram((fw_flashAddr ), 32, (u8 *)&fw_tmpdata[0]); ///0x18 store bin size value

    fw_size = *(u32 *)&fw_tmpdata[IC_BIN_FW_SIZE_ADDR];

    u16 fw_Block;
    u16 fw_remainSizeByte;
    fw_Block          = fw_size / FW_READ_SIZE;
    fw_remainSizeByte = fw_size % FW_READ_SIZE;

    int i = 0;

    u8  ota_dat[FW_READ_SIZE << 1];
    for (i = 0; i < fw_Block; i++) { ///Telink bin must align 16 bytes.

        read_storage_to_ram((fw_flashAddr + i * FW_READ_SIZE), FW_READ_SIZE, fw_tmpdata);

        //FW_READ_SIZE byte OTA data32  half byteCRC
        for (int j = 0; j < FW_READ_SIZE; j++) {
            ota_dat[j * 2]     = fw_tmpdata[j] & 0x0f;
            ota_dat[j * 2 + 1] = fw_tmpdata[j] >> 4;
        }
        fw_crc_init = crc32_half_cal(fw_crc_init, ota_dat, (unsigned long *)(unsigned int)fw_crc32_half_tbl, (FW_READ_SIZE << 1));
        wd_clear();
    }

    //////////////////////////////
    if (fw_remainSizeByte != 4) {
        read_storage_to_ram((fw_flashAddr + fw_size - fw_remainSizeByte), (fw_remainSizeByte - 4), fw_tmpdata);
        for (i = 0; i < (fw_remainSizeByte - 4); i++) {
            ota_dat[i * 2]     = fw_tmpdata[i] & 0x0f;
            ota_dat[i * 2 + 1] = fw_tmpdata[i] >> 4;
        }
        fw_crc_init = crc32_half_cal(fw_crc_init, ota_dat, (unsigned long *)(unsigned int)fw_crc32_half_tbl, ((fw_remainSizeByte - 4) << 1));
    }

    ////////read crc value and compare
    static u32 fw_check_value;
    read_storage_to_ram((fw_flashAddr + fw_size - 4), 4, (u8 *)&fw_check_value);
     tlkapi_printf(APP_LOG_EN, "fw_init=%08x,fw_check=%08x,\r\n" ,fw_crc_init,fw_check_value);
    if (fw_check_value != fw_crc_init) {
        return 1; ///CRC check fail
    }

    return 0;     ///CRC check ok
}
void clear_ota_mark(unsigned int addr)
{
#if MCU_RUN_IN_FLASH
        flash_erase_sector(addr);    //Invalid flag
#else
    unsigned char buf[48]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    write_storage_from_ram(addr,48,(unsigned int*)buf);  
#endif
}
void usb_ota_init(void)
{
    unsigned int  startup_flag=0;
#if MCU_RUN_IN_FLASH
    unsigned int flash_unlock_step=0;
    int cur_flash_setor;

    unsigned int tmp1            = 0;
    unsigned int tmp2                  = 0;
    unsigned int tmp3                  = 0;
    unsigned int tmp4                  = 0;
#endif
    read_storage_to_ram(IC_FIRST_FW_ADDR+IC_BOOT_MARK_ADDR,4,(u8 *)&startup_flag);

    if((startup_flag==IC_BIN_MARK))
    {
        ota_program_offset=IC_SECOND_FW_ADDR;
    }
    else
    {
        ota_program_offset=IC_FIRST_FW_ADDR;  
    }
   // unsigned int fw_size=0;
   // flash_read_page(BIN_MAX_SIZE-ota_program_offset+0x18,4,(u8 *)&fw_size);
   // fw_size=fw_size-4;
   // flash_read_page(BIN_MAX_SIZE-ota_program_offset+fw_size,4,(u8 *)&bin_crc);
       

#if MCU_RUN_IN_FLASH
    unsigned int bank_size=(IC_SECOND_FW_ADDR-IC_FIRST_FW_ADDR)/4096;
    for (unsigned int i = 0; i < bank_size; i++) // "size>>12" equal to "size/4K"
    {
         cur_flash_setor = ota_program_offset + i * 0x1000;
         flash_read_page(cur_flash_setor, 4, (u8 *)&tmp1);
        flash_read_page(cur_flash_setor + 1024, 4, (u8 *)&tmp2);
        flash_read_page(cur_flash_setor + 2048, 4, (u8 *)&tmp3);
        flash_read_page(cur_flash_setor + 4000, 4, (u8 *)&tmp4);
         if (tmp1 != ONES_32 || tmp2 != ONES_32 || tmp3 != ONES_32 || tmp4 != ONES_32)
         {
            if(flash_unlock_step==0)
            {
                flash_unlock_step=1;
 #if (APP_FLASH_PROTECTION_ENABLE)
    flash_unlock_op();
#endif
            }
            flash_erase_sector(cur_flash_setor);
         }
    }
    if(flash_unlock_step==1)
    {
#if APP_FLASH_PROTECTION_ENABLE
        flash_lock_op();
#endif
    }
#endif

#if MCU_RUN_IN_NVM
    read_storage_to_ram(ota_program_offset+IC_BOOT_MARK_ADDR,4,(u8 *)&startup_flag);
    if((startup_flag==IC_BIN_MARK))
    {
        clear_ota_mark(ota_program_offset);
    }
#endif
    tlkapi_printf(APP_LOG_EN, "ota_program_offset=%08x\r\n", ota_program_offset);
    
}
unsigned char need_reboot;
unsigned int tick_need_reboot;
void usb_ota_reboot_init(void)
{
    need_reboot=1;
    tick_need_reboot=clock_time();
    
}

int usb_ota_resut(u8 result)
{
    usb_data_t *p=(usb_data_t *)(&cdc_in_buffer[0]);
    int ret= 0;
    p->report_id = USB_VENDOR_REPORT_ID;
    p->opcode = 0x02;
    p->length = 3;
    p->dat[0] = 0x06;
    p->dat[1] = 0xff;
    p->dat[2] = result;
    
    ota_start_flag=0;

#if MCU_RUN_IN_NVM
    g_nvm_bulk_write = false;
    nvm_mtp_lock_set(NVM_MTP_LOCK_ALL_512K);
    usb_ota_dig_vol_restore();
#endif

    usb_ota_reboot_init();
 
    return ret;
}




u8 usb_ota_write(usb_data_t *p_usb)
{
//    static u16 ota_index=0;
    static u16 start_index=0;
    static u32 flash_write_addr=0;
    static u8 first_3_packet_data_buf[16 * 3];
    static u32 fw_size=0;
    static u8 ota_error_flag=0;
#if MCU_RUN_IN_NVM
    static u8 ota_page_buf[NVM_PAGE_SIZE] = {0};
    static u16 page_buf_offset = 0;
#endif
    ota_pdu16_t *pd = (ota_pdu16_t*)&p_usb->dat[0];

    if((pd->adr_index== CMD_OTA_START)&&(p_usb->length==2))
    {

    #if APP_WATCHDOG_ENABLE
        wd_stop();
    #endif
        if(ota_start_flag)
        {
            start_reboot();
            return 0;
        }
        ota_start_tick = clock_time();
        ota_start_flag = 1;
        is_ota_ing = 1;
#if MCU_RUN_IN_NVM
#if APP_NVM_PROTECTION_ENABLE
        g_nvm_bulk_write = true; // set to true during USB OTA
        usb_ota_dig_vol_raise();
        if (nvm_read_reg(NVM_READ_MP_CMD) == NVM_MTP_LOCK_ALL_512K) {
            nvm_mtp_lock_set(NVM_MTP_LOCK_NONE);
        }
#endif
#endif

#if MCU_RUN_IN_FLASH
#if (APP_FLASH_PROTECTION_ENABLE)
        flash_unlock_op();
#endif
#endif
//        ota_index=0;

        flash_write_addr=0;
        start_index=0;
        fw_size=0;
        ota_error_flag=OTA_SUCCESS;
        ota_success_flag=0;
        //flash_erase_sector(ota_program_offset);
#if MCU_RUN_IN_NVM
        memset(ota_page_buf, 0, NVM_PAGE_SIZE);
        page_buf_offset = 0;
#endif
        tlkapi_printf(APP_LOG_EN, "usb_ota_start\r\n" );
        return ota_error_flag;
    }
    else if((ota_error_flag==OTA_SUCCESS)&&ota_start_flag)
    {
        if((pd->adr_index == CMD_OTA_END)&&(p_usb->length==6))
        {
            
            //printf("---usb_ota_end\r\n");
            u32 *telink_mark=(u32*)&first_3_packet_data_buf[32];
            if(telink_mark[0]!=IC_BIN_MARK)
            {
                ota_error_flag = OTA_FIRMWARE_MARK_ERR;
                return ota_error_flag;
            }
            u32 real_bin_size = 0;
            real_bin_size = fw_size-4;
            //printf("---real_bin_size=%d_%d.fw_size=%d.\n",real_bin_size,(start_index*16),fw_size);
            if(real_bin_size!=(start_index*16))
            {
                ota_error_flag=OTA_FW_SIZE_ERR;
                return ota_error_flag;
            }

        #if MCU_RUN_IN_NVM
            // put the remaining data in page_buf to rram
            if (page_buf_offset > 0)
            {
                u32 page_write_addr = ota_program_offset + flash_write_addr - page_buf_offset;
                write_storage_from_ram(page_write_addr, NVM_PAGE_SIZE, (unsigned int*)ota_page_buf);
                page_buf_offset = 0;
                memset(ota_page_buf, 0, NVM_PAGE_SIZE);
            }
        #endif

            //printf("******** write first packet to addr:0x%03x ********\n",ota_program_offset);
            write_storage_from_ram(ota_program_offset, 16 * 3,(unsigned int*)first_3_packet_data_buf);

            u8 read_flash_buf[48];       
            read_storage_to_ram(ota_program_offset, 16 * 3, read_flash_buf);

            if(memcmp(read_flash_buf, first_3_packet_data_buf, 48))
            {  //do not equal
                clear_ota_mark(ota_program_offset);
                ota_error_flag = OTA_WRITE_FLASH_ERR;
                return ota_error_flag;
            }

//            u32 flag = 0;
            
            u8 ret = flash_fw_check_app(ota_program_offset); 


            if(ret==0)
            {
                ota_success_flag=1;
                //extern u32 fw_crc_init;
                //printf("---fw_crc_init=0x%04x\r\n",fw_crc_init);

                //printf("---usb ota success,erase Firmware\r\n");
                usb_ota_resut(OTA_SUCCESS);
                gpio_write(GPIO_PF0, 1);
            }
            else
            {
                //printf("---usb ota fail,erase OTA\r\n");
                clear_ota_mark(ota_program_offset);
                ota_error_flag=OTA_FW_CHECK_ERR;
                return ota_error_flag;
            }
        }
        else 
        {
            if((p_usb->length%20)!=0)
            {
                ota_error_flag=OTA_PDU_LEN_ERR;
                return ota_error_flag;
            }
            u8 cnt=p_usb->length/20;
            
            for(u8 i=0;i<cnt;i++)
            {
                pd=(ota_pdu16_t*)&p_usb->dat[20*i];
                //printf("pd_crc=%02x.user_crc=0x%02x.\n",pd->crc,crc16_user((u8*)&pd->cmd,18));
                if(crc16_user((u8*)&pd->adr_index,18)==pd->crc_16)
                {
                    if(pd->adr_index==0x0000)//first_data
                    {
                        memcpy(first_3_packet_data_buf,pd->data,16);
                        start_index = 0;
                    } else if (pd->adr_index==0x0001) {
                        memcpy(&first_3_packet_data_buf[16], pd->data,16);
                        fw_size = pd->data[8] | (pd->data[9] <<8) |(pd->data[10]<<16) | (pd->data[11]<<24);
                        start_index = pd->adr_index;
                        if((fw_size) > (IC_SECOND_FW_ADDR-IC_FIRST_FW_ADDR))//128k
                        {
                            ota_error_flag=OTA_FW_SIZE_ERR;
                            return ota_error_flag;
                        }

                        if((fw_size) < flash_write_addr)
                        {
                            ota_error_flag = OTA_FW_SIZE_ERR;
                            return ota_error_flag;
                        }
                    } else if (pd->adr_index==0x0002) {
                        start_index=pd->adr_index;
                        memcpy(&first_3_packet_data_buf[32], pd->data, 16);
                    }
                    else
                    {
                        if((start_index+1)!=pd->adr_index)
                        {
                            ota_error_flag=OTA_DATA_PACKET_SEQ_ERR;
                            return ota_error_flag;
                        }
                        start_index=pd->adr_index;

                        if((fw_size)<flash_write_addr)
                        {
                            ota_error_flag=OTA_FW_SIZE_ERR;
                            return ota_error_flag;
                        }

                        if(ota_error_flag==OTA_SUCCESS)
                        {
                            #if 0
                            if((flash_write_addr%4096)==0)
                            {
                                flash_erase_sector(flash_write_addr+ota_program_offset);
                            }
                            #endif                         
                        #if MCU_RUN_IN_FLASH
                            write_storage_from_ram(flash_write_addr+ota_program_offset,16,(unsigned int*)pd->data);
                        #elif MCU_RUN_IN_NVM      
                            memcpy(&ota_page_buf[page_buf_offset], pd->data, OTA_PKT_SIZE);
                            page_buf_offset += OTA_PKT_SIZE;

                            if(page_buf_offset >= NVM_PAGE_SIZE)
                            {
                                u32 page_write_addr = ota_program_offset + flash_write_addr - (NVM_PAGE_SIZE - OTA_PKT_SIZE);
                                write_storage_from_ram(page_write_addr, NVM_PAGE_SIZE, (unsigned int*)ota_page_buf);
                                page_buf_offset = 0;
                                memset(ota_page_buf, 0, NVM_PAGE_SIZE);
                            }
                        #endif
                        }
                    }
                    flash_write_addr+=16;
                }
                else
                {
                    ota_error_flag=OTA_DATA_CRC_ERR;
                    return ota_error_flag;
                }
            }
        }
    }

    return ota_error_flag;
}

 void usb_ota_timeout_check(void)
{
    if(ota_start_flag)
    {
        if(clock_time_exceed(ota_start_tick, 30000000))
        {
            usb_ota_resut(OTA_TIMEOUT);
            
        }
    }
    if(need_reboot)
    {
        if(clock_time_exceed(tick_need_reboot, 1000000))
        {
            if(ota_success_flag)
            {
                unsigned int  addr=IC_FIRST_FW_ADDR;
                if(ota_program_offset==IC_FIRST_FW_ADDR)
                {
                    addr=IC_SECOND_FW_ADDR;
                    
                
                }
                clear_ota_mark(addr);
                
            }
            sleep_ms(100);
            start_reboot();
            
        }
        
        
    }
}

void app_spp_suspend_cb(unsigned char *data, unsigned char len, bool success, void *user_arg)
{
    (void)user_arg;
    (void)len;
    (void)data;
    if (success) {
        tpsll_request_suspend();
    }
}

void usbd_suspend_callback(unsigned char bus)
{
    (void)bus;
    if(app_get_usb_wake_status() == USB_ENTER_SUSPEND)
    {
        printf("susp err\n");
        usb0hw_power_down();
        usb0hw_pcgc_clk_en();
        usb0hw_phy_pll_en();
        usb0hw_init(USB0_SPEED_HIGH);
        usb0hw_set_grxfsiz(0x100);
        usb0hw_set_epin_size(USB0_EP0, 0x100, 64);
        usb0hw_set_epin_size(USB0_EP1, 0x100 + 64, APP_KEYBOARD_ENDPOINT_BUFF_SIZE);
        usb0hw_set_epin_size(USB0_EP2, 0x100 + 64+APP_KEYBOARD_ENDPOINT_BUFF_SIZE, APP_MOUSE_ENDPOINT_BUFF_SIZE);
        usb0hw_set_epin_size(USB0_EP3, 0x100 + 64+32+APP_MOUSE_ENDPOINT_BUFF_SIZE, 32);
        usb0hw_set_epin_size(USB0_EP4, 0x100 + 64+32+APP_MOUSE_ENDPOINT_BUFF_SIZE+32, 64);
        usb0hw_set_epin_size(USB0_EP5, 0x100 + 64+32+APP_MOUSE_ENDPOINT_BUFF_SIZE+96, 64);
        return;
    }

    usbd_hid_reset_interface_data();

    usb0hw_flush_tx_fifo(0x10);  /* 0x10: flush all TX FIFOs */
    for (int i = 0; i < USBD_SUPPORT_ENDPOINT_MAX; i++)
    {
        ep_is_busy[i] = 0;
    }

    usb0hw_pcgc_clk_dis();
    usb0hw_phy_pll_dis();

    app_set_usb_wake_status(USB_ENTER_SUSPEND);
    usb_suspend_flag = 1;

    if(tpsll_get_state() != STATE_CONNECTED)
    {
        tpsll_request_suspend();
    }
    else if(tpsll_get_state() == STATE_CONNECTED)
    {
        uint8_t susp_buf[1] = {TPSLL_SPP_PC_SUSPEND};
        tpsll_send_spp_data(susp_buf, sizeof(susp_buf), 3, app_spp_suspend_cb, NULL);
    }
}

void usbd_resume_callback(unsigned char bus)
{
    (void)bus;
    remote_wakeup = USB_EXIT_SUSPEND;
    usb_suspend_flag = 0;
    usb0hw_pcgc_clk_en();
    usb0hw_phy_pll_en();
}

void usbd_resetdet_callback(unsigned char bus)
{
    (void)bus;
    remote_wakeup = USB_EXIT_SUSPEND;
    usb_suspend_flag = 0;
    usb0hw_pcgc_clk_en();
    usb0hw_phy_pll_en();
}

uint32_t app_get_usb_enum_status(void)
{
    return (usb_status_flag&USB_OK_FLAG_APP);
}

uint8_t app_get_usb_wake_status(void)
{
    return remote_wakeup;
}

void app_set_usb_wake_status(uint8_t state)
{
    remote_wakeup = state;
}

unsigned char  usb_ota_loop(void)
{
    usb_ota_timeout_check();
    if(cdc_out_data_len==0)
    {
        
        return 0;
    }
    // start_reboot();

    usb_data_t *p=(usb_data_t *)(&cdc_in_buffer[0]);
    //if ((p->report_id == USB_VENDOR_REPORT_ID))
    { //if received USB data is OTA data
        if ((p->opcode == 0x01) && (p->length == 0))
        { //PC get firmware version,
            p->length = 12;
            memcpy(&p->dat[0], (u8 *)&app_version, 4);
            memcpy(&p->dat[4], (u8 *)&bin_crc, 4);
            memcpy(&p->dat[8], (u8 *)&ota_program_offset, 4);
        }
        else if (p->opcode == 0x02)
        { //PC send OTA data
            
            u8 ota_error_flag=usb_ota_write(p);
            if(ota_error_flag)
            {
                usb_ota_resut(ota_error_flag);
            }
        }

        /* ACK received packet data to PC */
        if (app_ep4_ota_report_to_usb() == 0)
        {
            cdc_out_data_len=0; 

            return 0;
        }
    }


    return 1;
}

#endif