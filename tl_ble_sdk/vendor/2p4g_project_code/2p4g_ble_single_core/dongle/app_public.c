/********************************************************************************************************
 * @file    app_public.c
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
#include "tl_common.h"
#include "app_public.h"
#include "app_usb.h"
#include "app_d24g_rf_link.h"
//#include "stack/ble/ble.h"
#include "uart.h"
#include "tl_string.h"

#define WDT_INTV_MS     (1000)

#define RRAM_READ_WRITE_MIN_SIZE 16

#define CONNECTED_LED_DEBUG_SET(x) gpio_set_level(PIN_LED_CON, x)

uint8_t encrypt_key[] = {0x3A, 0x7B, 0xC2, 0x9D};
uint8_t encrypt_key_len = 4;

ST_FLASH_DEV_INFO flash_dev_info  __attribute__ ((aligned (4)));
int dev_info_idx;
uint32_t g_2p4g_error_code = 0;

unsigned int flash_sector_2p4_inf=CFG_SIDE_INF_ADDR;

unsigned int tick_allow_pair_new_deviec_afer_power_on = 0;
unsigned char flag_allow_pair_new_deviec_afer_power_on = 1;
unsigned int tick_timer_pair_reconnect = 0;

static unsigned char g_last_kb_led_status = 0xff;

volatile bool g_nvm_bulk_write = false; // set to true during USB OTA
//extern void nvm_reg_read(unsigned int addr, unsigned int len, unsigned int *data);
//extern void nvm_reg_write(unsigned int addr, unsigned int len, unsigned int *data);


#if (APP_FLASH_PROTECTION_ENABLE)
void flash_lock_op(void)
{
    flash_protection_init();

    u16 flash_lockBlock_cmd = 0;

    flash_lockBlock_cmd = flash_change_app_lock_block_to_flash_lock_block(FLASH_LOCK_FW_LOW_512K);

    if (blc_flashProt.init_err) {
        tlkapi_printf(APP_FLASH_PROT_LOG_EN, "[FLASH][PROT] flash protection initialization error!!!\n");
    }

    tlkapi_printf(APP_FLASH_PROT_LOG_EN, "[FLASH][PROT] initialization, lock flash\n");
    flash_lock(flash_lockBlock_cmd);
}

void flash_unlock_op(void)
{
    flash_unlock();
}

#endif

void app_readFlashSize_autoConfigCustomFlashSector(void)
{
    if (blc_flash_capacity == FLASH_SIZE_1M) 
    {
        flash_sector_2p4_inf=P24G_PAIR_INF_FLASH_ADDR_1M;
    }
    else if (blc_flash_capacity == FLASH_SIZE_2M) 
    {
        flash_sector_2p4_inf=P24G_PAIR_INF_FLASH_ADDR_2M;
    }
    else if (blc_flash_capacity == FLASH_SIZE_4M) 
    {
        flash_sector_2p4_inf=P24G_PAIR_INF_FLASH_ADDR_4M;
    }
    else //if (blc_flash_capacity == FLASH_SIZE_512K) 
    {
        flash_sector_2p4_inf=P24G_PAIR_INF_FLASH_ADDR_512K;
    }
}

void blc_readRramSize_autoConfigCustomFlashSector(void)
{
    blc_flash_capacity = FLASH_SIZE_512K;
    flash_sector_mac_address = CFG_ADR_MAC_512K_RRAM;
    flash_sector_calibration = CFG_ADR_CALIBRATION_512K_RRAM;
    flash_sector_smp_storage = FLASH_ADR_SMP_PAIRING_512K_RRAM;

#if (ACL_CENTRAL_SIMPLE_SDP_ENABLE)
    flash_sector_simple_sdp_att = FLASH_SDP_ATT_ADDRESS_512K_RRAM;
#endif

    tlkapi_printf(APP_FLASH_INIT_LOG_EN, "[RRAM][INI] 512K Rram, MAC on %x\r\n", flash_sector_mac_address);
}

void p24g_pairing_info_check(void)
{
    dev_info_idx = flash_info_load(flash_sector_2p4_inf, &flash_dev_info.side_id, sizeof(ST_FLASH_DEV_INFO));
    if (dev_info_idx >= 0)
    {
        // DBG_GPIO_TOGGLE(APP_IO_EN, GPIO_PH0);
        // tpsll_enable_pairing(true);
        uint16_t peer_tag = fnv1a_16(flash_dev_info.peer_addr, MAC_ADDR_LEN);
        tlkapi_send_string_data(APP_LOG_EN, "paired", &flash_dev_info.side_id, 12);
        if (flash_dev_info.peer_type & DEVICE_TYPE_KB)
        {
            app_ctx.peer_tag = (peer_tag & 0x3fff) | (DEVICE_TYPE_KB << 14);
            app_ctx.paired_dev_type = DEVICE_TYPE_KB;
        }
        else if (flash_dev_info.peer_type & DEVICE_TYPE_MS)
        {
            app_ctx.peer_tag = (peer_tag & 0x3fff) | (DEVICE_TYPE_MS << 14);
            app_ctx.paired_dev_type = DEVICE_TYPE_MS;
        }
        tmemcpy(app_ctx.peer_mac, flash_dev_info.peer_addr, MAC_ADDR_LEN);

        
        // tlkapi_send_string_data(APP_LOG_EN, "peer_tag", &app_ctx.peer_tag, 2);
        tlkapi_send_string_data(APP_LOG_EN, "enable reconnect...", 0, 0);

        tpsll_stack_var_t var = {
            .peer_tag = app_ctx.peer_tag,
            .paired_dev_type = app_ctx.paired_dev_type,
            .peer_type = flash_dev_info.peer_type,
        };
        tmemcpy(var.peer_mac, app_ctx.peer_mac, MAC_ADDR_LEN);
        tpsll_stack_set_var(&var, TPSLL_STACK_VAR_SET_PAIRING_INFO); 
        tpsll_enable_reconn(true);
    
        // p24g_send_sm_msg(P24G_SM_CMD_PAIRING, true, 0, 0);
        // p24g_enable_pairing(true);

    }
    else
    {
        tlkapi_send_string_data(APP_LOG_EN, "enable pairing...", &flash_dev_info.side_id, 12);
        flag_allow_pair_new_deviec_afer_power_on = 0;
        tpsll_enable_pairing(true);
    }

    // tlkapi_send_string_data(APP_LOG_EN, "mac...", app_ctx.mac, 6);
    // tlkapi_send_string_data(APP_LOG_EN, "dev_info_idx", &dev_info_idx, 4);
}

int tvsnprintf(char *str, unsigned int size, unsigned char *format, ...);

volatile  u32 debugRAddr=0,debugRLen=0,debugRData=0,debugRFlag=0;
void read_storage_to_ram(unsigned int addr, unsigned int len, unsigned char *data)
{
#if MCU_RUN_IN_NVM

    debugRAddr=addr;
    debugRLen=len;
    debugRData=(u32)data;
    debugRFlag = 1;

    u8 readDat[RRAM_READ_WRITE_MIN_SIZE] = {0};
    u32 length = len;
    u32 *ptr=(u32 *)data;
    if(len<RRAM_READ_WRITE_MIN_SIZE)
    {
        length = RRAM_READ_WRITE_MIN_SIZE;
        ptr = (u32 *)&readDat[0];
    }
    while (reg_nvm_state & BIT(1))
    {
    }
    nvm_reg_read(addr,length,ptr);
    if(len<RRAM_READ_WRITE_MIN_SIZE)
    {
        memcpy((u8 *)data,(u8 *)readDat,len);
    }
    debugRFlag = 0;
#else
    flash_read_page(addr,len,(u8 *)data);
#endif
}

pm_dig_vol_mode_e app_dig_vol_get_val(void)
{
    return (reg_boot_idcode(0) & FLD_DIG_LDO_1V1_EN) ? DIG_VOL_1V1_MODE : DIG_VOL_1V_MODE;
}

pm_dig_vol_mode_e app_dig_vol_raise_to_1v1(void)
{
    pm_dig_vol_mode_e cur = app_dig_vol_get_val();
    if (cur != DIG_VOL_1V1_MODE) {
        pm_set_dig_ldo(DIG_VOL_1V1_MODE, 1000);
    }
    return cur;
}

void app_dig_vol_restore(pm_dig_vol_mode_e vol)
{
    if (vol != DIG_VOL_1V1_MODE) {
        pm_set_dig_ldo(vol, 1000);
    }
}

volatile u32 debugWAddr=0,debugWLen=0,debugWData=0,debugWFlag=0;
void write_storage_from_ram(unsigned int addr, unsigned int len, unsigned int *data)
{
#if MCU_RUN_IN_NVM
    bool vol_raised = false;
    pm_dig_vol_mode_e prev_vol = DIG_VOL_1V1_MODE;
    if (!g_nvm_bulk_write) {
        prev_vol = app_dig_vol_raise_to_1v1();
        if (prev_vol != DIG_VOL_1V1_MODE) {
            vol_raised = true;
        }
    }
    
    debugWAddr=addr;
    debugWLen=len;
    debugWData=(u32)data;
    debugWFlag = 1;

    u8 readDat[RRAM_READ_WRITE_MIN_SIZE] = {0};
    u32 length = len;
    u32 *ptr=(u32 *)data;
    while (reg_nvm_state & BIT(1))
    {
    }
    if(len<RRAM_READ_WRITE_MIN_SIZE)
    {
        length = RRAM_READ_WRITE_MIN_SIZE;
        ptr = (u32 *)&readDat[0];

        nvm_reg_read(addr,length,ptr);
        memcpy((u8 *)ptr,(u8 *)data,len);
    }

#if APP_NVM_PROTECTION_ENABLE
    if (!g_nvm_bulk_write) {
        if (nvm_read_reg(NVM_READ_MP_CMD) == NVM_MTP_LOCK_ALL_512K) {
            nvm_mtp_lock_set(NVM_MTP_LOCK_NONE);
        }
    }
#endif
    nvm_reg_write(addr,length,ptr);
#if APP_NVM_PROTECTION_ENABLE
    if (!g_nvm_bulk_write) {
        nvm_mtp_lock_set(NVM_MTP_LOCK_ALL_512K);
    }
#endif
    debugWFlag = 0;

    if (vol_raised) {
        app_dig_vol_restore(prev_vol);
    }
#else
    flash_write_page(addr,len,(u8 *)data);
#endif
}

 /**
  * @brief   read flash information
  * @param[in]   s_addr    - the base address of flash.
  * @param[in]   len     - the length(in byte, must be above 0) of content needs to read out from the page.
  * @param[out]  d_addr     - the start address of the buffer(ram address). 
  * @return  The offset address of the last data
  */
 
int flash_info_load(unsigned int s_addr, unsigned int *d_addr,  int len)
{
     int idx;
     unsigned int buf;
     int length = len;
#if MCU_RUN_IN_NVM
     if (length < RRAM_READ_WRITE_MIN_SIZE) {
        length = RRAM_READ_WRITE_MIN_SIZE;
     }
#endif

     for (idx = 0; idx < (4096 - length); idx += length)
     {
#if MCU_RUN_IN_NVM
         read_storage_to_ram((unsigned int)(s_addr + idx), length, (unsigned char *)(&buf));
         if (buf == 0) {
             break;
         }
#else
         read_storage_to_ram((unsigned int)(s_addr + idx), 4, (unsigned char *)(&buf));
         if (buf == 0xffffffff) {
             break;
         }
#endif
     }

     idx -= len;
     if (idx < 0)        // no binding
     {
         return idx;
     }
#if MCU_RUN_IN_NVM
     if (idx < RRAM_READ_WRITE_MIN_SIZE) {
        idx = 0;
     }
#endif
     read_storage_to_ram((unsigned int)(s_addr + idx), len, (unsigned char *)d_addr);

     if (idx > 3000)             //3k, erase flash
     {
#if MCU_RUN_IN_FLASH
         flash_erase_sector((unsigned int)s_addr);
#endif

        // sleep_us(10);
         write_storage_from_ram((unsigned int)s_addr, len, d_addr);
         idx = 0;
     }
     return idx;
}
 
 /**
  * @brief   save data to flash
  * @param[in]   addr    - the base address of flash.
  * @param[in]   len     - the length(in byte, must be above 0) of content needs to read out from the page.
  * @param[in]   buf     - data  buffer(ram address)
  * @param[in,out]  offset     - The offset address of the last data
  * @return  1: success  ,other: fail
  */
 
_attribute_ram_code_sec_ int save_data_to_flash(unsigned long addr, int len, unsigned int *buf,int *offset)
{
     unsigned char tmp_buf[len];
     int tmp_offset=0;
     
 
     if(offset[0]>(4096-2*len))
     {
#if MCU_RUN_IN_FLASH
         flash_erase_sector(addr);
#endif
         offset[0]=-len;
     }

     tmp_offset=offset[0]+len;
     
     for(unsigned char i=0;i<3;i++)
     {
         write_storage_from_ram(addr+tmp_offset, len, buf);
         read_storage_to_ram(addr+tmp_offset, len, tmp_buf);
         if(tmemcmp(tmp_buf, buf, len)==0)
         {
             offset[0]=tmp_offset;
             return 1;//#define SUCCESS                   0x00
         }
         else
         {
#if MCU_RUN_IN_FLASH
             flash_erase_sector(addr);
#endif
             tmp_offset=0;
         }
     }
 
     return 0;//write fail  
}

_attribute_ram_code_sec_ uint32_t fnv1a_hash(const uint8_t *data, size_t len) 
{
    uint32_t hash = 0x811C9DC5;
    
    for (size_t i = 0; i < len; i++) {
        hash ^= data[i];
        hash = hash + (hash << 24) + (hash << 8) + (hash << 5);
        hash ^= (hash >> 16);//Avalanche Effect
    }

    return hash;
}

/**
 * @brief FNV-1a 16-bit hash function
 * @param data   
 * @param len    
 * @return 16-bit hash value
 */
_attribute_ram_code_sec_ uint16_t fnv1a_16(uint8_t *data, size_t len)
{
    uint16_t hash = 0x55AA;
    
    for (size_t i = 0; i < len; i++) {

        hash ^= data[i];
        hash = (hash << 5) | (hash >> 11);
        
        hash ^= (hash >> 8);//Avalanche Effect
    }

    return hash;
}


static inline void app_wdt_init(void)
{
    if (wd_get_status())
    {
        wd_clear_status();
    }
    wd_set_interval_ms(WDT_INTV_MS);
    /**
     * Wd_clear() must be executed before each call to wd_start() to avoid abnormal watchdog reset time because the initial count value is not 0.
     * For example, the watchdog is reset soon or a few minutes later.
     */
    wd_clear();
    wd_start();
}


void user_init(void)
{
#if (HW_BOARD_TYPE == HW_EVK_DONGLE)

#if (DBG_2P4G_IO_EN == 1)
    gpio_function_en(LED_GREEN | LED_WHITE | LED_BLUE | LED_RED);
    gpio_output_en(LED_GREEN | LED_WHITE | LED_BLUE | LED_RED);
    gpio_input_dis(LED_GREEN | LED_WHITE | LED_BLUE | LED_RED);
    gpio_set_level((LED_GREEN | LED_WHITE | LED_BLUE | LED_RED), LED_IS_OFF);

    gpio_function_en(GPIO_PG4 | GPIO_PG5 | GPIO_PG6 | GPIO_PG7);
    gpio_output_en(GPIO_PG4 | GPIO_PG5 | GPIO_PG6 | GPIO_PG7);
    gpio_input_dis(GPIO_PG4 | GPIO_PG5 | GPIO_PG6 | GPIO_PG7);

    gpio_function_en(GPIO_PH0 | GPIO_PH1 | GPIO_PH2 | GPIO_PH3 | GPIO_PH4 | GPIO_PH5);
    gpio_output_en(GPIO_PH0 | GPIO_PH1 | GPIO_PH2 | GPIO_PH3 | GPIO_PH4 | GPIO_PH5);
    gpio_input_dis(GPIO_PH0 | GPIO_PH1 | GPIO_PH2 | GPIO_PH3 | GPIO_PH4 | GPIO_PH5);

    gpio_function_en(GPIO_PB4);
    gpio_output_en(GPIO_PB4);
    gpio_input_dis(GPIO_PB4);
    gpio_set_level(GPIO_PB4, 1);
    gpio_set_level(GPIO_PB4, 0);
    gpio_set_level(GPIO_PB4, 1);
    gpio_set_level(GPIO_PB4, 0);

    gpio_function_en(PIN_LED_CON | PIN_LED_PAIR | PIN_LED_DISCON | PIN_LED_TOG);
    gpio_output_en(PIN_LED_CON | PIN_LED_PAIR | PIN_LED_DISCON | PIN_LED_TOG);
    gpio_input_dis(PIN_LED_CON | PIN_LED_PAIR | PIN_LED_DISCON | PIN_LED_TOG);
    gpio_set_level((PIN_LED_CON | PIN_LED_PAIR | PIN_LED_DISCON | PIN_LED_TOG), LED_IS_OFF);
#endif

#else
    gpio_function_en(LED_GREEN | LED_WHITE | LED_BLUE | LED_RED);
    gpio_output_en(LED_GREEN | LED_WHITE | LED_BLUE | LED_RED);
    gpio_input_dis(LED_GREEN | LED_WHITE | LED_BLUE | LED_RED);
    gpio_set_level((LED_GREEN | LED_WHITE | LED_BLUE | LED_RED), LED_IS_OFF);
    
    
    
#endif

    
    trng_init();


#if ALG_KEYSCAN_APP_FUN_ENABLE
     alg_keyscan_init();
#endif


#if (TLKAPI_DEBUG_ENABLE)
    tlkapi_debug_init();
    blc_debug_enableStackLog(STK_LOG_NONE);
#endif

    const unsigned char* version_num = tpsll_get_stack_version();
    // tlkapi_send_string_data(APP_LOG_EN, "version", version_num, 21);
    tlkapi_send_string_data(0, "version", version_num, 21);

    tlkapi_printf(APP_LOG_EN, "dongle init\r\n");

    #if DBG_2P4G_IO_EN
    // tpsll_stack_debug_io_init(GPIO_PG5, GPIO_PG4, GPIO_PG6, GPIO_PG7, GPIO_PH0, GPIO_PH1, GPIO_PH2, GPIO_PH4);
    tpsll_stack_debug_io_init(GPIO_PG5, GPIO_PG4, GPIO_PG6, GPIO_PG7, GPIO_PH0, GPIO_PH1, 0, 0);

    // gpio_function_en(GPIO_PH0);
    // gpio_output_en(GPIO_PH0);
    // gpio_input_dis(GPIO_PH0);

    #endif

#if MCU_RUN_IN_NVM
    blc_readRramSize_autoConfigCustomFlashSector();
#else
    blc_readFlashSize_autoConfigCustomFlashSector();
#endif

    app_readFlashSize_autoConfigCustomFlashSector();

    /* attention that this function must be called after "blc readFlashSize_autoConfigCustomFlashSector" !!!*/
    blc_app_loadCustomizedParameters_normal();

#if MCU_RUN_IN_FLASH
#if (APP_FLASH_PROTECTION_ENABLE)
    flash_lock_op();
#endif
#endif

    u8 mac_public[6];
    u8 mac_random_static[6];

    blc_initMacAddress(flash_sector_mac_address, mac_public, mac_random_static);
    tmemcpy(app_ctx.mac, mac_public, MAC_ADDR_LEN);
    tmemcpy(ll_inf.mac, app_ctx.mac, MAC_ADDR_LEN);
    ll_inf.tag = fnv1a_16(ll_inf.mac, MAC_ADDR_LEN);

    #if USB_APP_FUN_ENABLE
    app_usb_init();//must be after pp_d24g_user_init to wait rssi scan complete
    #endif
    if (!(analog_read_reg8(PM_ANA_REG_POWER_ON_CLR_BUF1) & 0x4)) {

        #if RF_MODE_APP_FUN_ENABLE
        pp_d24g_user_init();

        tpsll_set_pair_encrypt_key(encrypt_key, encrypt_key_len);
        tpsll_set_pairing_rssi_threshold(PAIRING_RSSI_THRESHOLD_DFT);

        p24g_pairing_info_check();

        #endif
    }
    app_wdt_init();

    flash_plic_preempt_config(1, 1);
    plic_preempt_feature_en(CORE_PREEMPT_PRI_MODE0);
    core_interrupt_enable();

    pm_set_usb0_wakeup();
    pm_set_suspend_power_cfg(FLD_PD_USB_EN, 1);

    tlkapi_printf(APP_LOG_EN, "dongle init done %x\r\n", read_reg8(0x17021e));
}

_attribute_ram_code_sec_ void led_test(void)
{
     static unsigned int tick=0;
     unsigned int interval=500000;

     if(tpsll_get_state() == STATE_PAIRING) {
        interval=200000;
     }

     if(clock_time_exceed(tick, interval)) {
         tick=stimer_get_tick();
         gpio_toggle(LED_BLUE);
     }
}

_attribute_ram_code_sec_ void app_notfiy_led_to_kb(unsigned char status)
{
    if (g_last_kb_led_status != status)
    {
        g_last_kb_led_status = status;
        uint8_t led_buf[2] = {TPSLL_SPP_LED_STATUS, g_last_kb_led_status};

        int ret = tpsll_send_spp_data(led_buf, sizeof(led_buf), 3, NULL, NULL);

        if (ret)
        {
            tlkapi_send_string_data(APP_LOG_EN, "app send kb led status failed", &ret, 1);
        }
    }
}

 /**
  * @brief       in 2.4g mode  device status check
  * @param[in]    none
  * @return      none
  */
 
_attribute_ram_code_sec_ void app_pp_check_connect_status(void)
{
    static  int last_connect_status=0xff;

    if (last_connect_status != (int)tpsll_get_state())
    {
#if 1
        if(tpsll_get_state()==STATE_CONNECTED)
        {
            gpio_set_level(PIN_LED_CON, 1);
        }
        else
        {
             gpio_set_level(PIN_LED_CON, 0);
        }
#endif
        last_connect_status = tpsll_get_state();

        if(ll_inf.dev_now_status==STATE_PAIRING)
        {
            tick_pair_timeout=clock_time();
//            CONNECTED_LED_DEBUG_SET(LED_IS_OFF);
        }
        else if(ll_inf.dev_now_status==STATE_RECONNECT)
        {
//            CONNECTED_LED_DEBUG_SET(LED_IS_OFF);
        }
        else if(ll_inf.dev_now_status==STATE_NORMAL)
        {
//            CONNECTED_LED_DEBUG_SET(LED_IS_ON);
        }
    }
    app_allow_pair_new_device_power_on_handle();
    // if(ll_inf.dev_now_status==STATE_PAIRING)
    // {
    //     if(clock_time_exceed(tick_pair_timeout, 15000000))
    //     {
    //         ll_inf.dev_now_status=STATE_RECONNECT;
    //         pp_set_pair_enable(0);
    //     }
    // }

}

#if(SPP_TEST_EN == 1)
void app_spp_cb(unsigned char *data, unsigned char len, bool success, void *user_arg)
{
    uint32_t *data_id = (uint32_t *)user_arg;

    if(success)
    {
        tlkapi_send_string_data(APP_LOG_EN, "spp send success", data, len);
        tlkapi_send_string_data(APP_LOG_EN, "data_id", data_id, 4);
    }else{
        tlkapi_send_string_data(APP_LOG_EN, "spp send fail", data, len);
    }
}

static void app_spp_test(void)
{
    #define RETRY_NUM   5
    static uint32_t spp_test_cnt = 0;
    static uint8_t spp_fill_flag = 0;
    static uint8_t spp_b[65] = {TPSLL_SPP_TEST_DATA, 0};

    if (!spp_fill_flag)
    {
        for (uint8_t i = 1; i < sizeof(spp_b); i++)
        {
            spp_b[i] = i;
        }

        spp_fill_flag++;
    }
    if(app_ctx.dev_status != STATE_CONNECTED)
    {
        return;
    }

    spp_test_cnt++;
    // if (spp_test_cnt % 50 == 0)
    {
        static uint32_t data_id = 0x1234;
        static uint8_t len = sizeof(spp_b);
        void *user_arg = &data_id;

        if (0 == tpsll_send_spp_data(spp_b, len, RETRY_NUM, app_spp_cb, user_arg)) {
            spp_b[1]++;

            #if 0
            if (len < sizeof(spp_b)) {
                len++;
            } else {
                len = 2;
            }

            for (uint32_t i = 2; i < len; i++)
            {
                spp_b[i] = spp_b[1] + (i - 1);
            }
            #endif
        }
    }
}
#endif

uint32_t esd_test_tick = 0;
_attribute_ram_code_sec_ void main_loop(void)
{
    while (1)
    {
#if TEST_SECU_SIGN_VRFY_EN
        extern unsigned char AAAtest_cnt;
        AAAtest_cnt ++;
#endif
        wd_clear();
    #if (TLKAPI_DEBUG_ENABLE)
        tlkapi_debug_handler();
    #endif

        // led_test();

    #if USB_APP_FUN_ENABLE
        app_usb_main_loop();
    #if USB_OTA_ENABLE
        usb_ota_loop();
    #endif
    #endif
    #if(EMI_TEST_FUN_ENABLE)
        usb_host_cmd_proc();
        if(emi_flg) 
        {
          return;
        }
    #endif
    extern uint8_t is_ota_ing;
    extern unsigned char ota_start_flag;
    if(esd_test_tick)
    {
        if(clock_time_exceed(esd_test_tick, 1000000))
        {
           if(!is_ota_ing && !ota_start_flag)
              start_reboot();
        }
    }

    if (!(analog_read_reg8(PM_ANA_REG_POWER_ON_CLR_BUF1) & 0x4)) {

    #if RF_MODE_APP_FUN_ENABLE
        app_pp_check_connect_status();
        tpsll_stack_main_loop();
        app_deferred_led_sync_check();
    #endif
    }
        #if (SPP_TEST_EN == 1)
        static unsigned int test_tick = 0;

        if (clock_time_exceed(test_tick, 50000))
        {
            test_tick = stimer_get_tick();

            app_spp_test();
        }
        #endif
    }
}

_attribute_aligned_(4)  unsigned char buf_txfifo[40*32];

pl_fifo_t tx_fifo={
     .size=40,
     .num=32,
     .wptr=0,
     .rptr=0,
     .p=buf_txfifo,
 };


t_mouse_data ms_data;

unsigned int g_square_side_len = 200;
/**
 * @brief   Simulate mouse data
 * @param[in,out]  x     - x data(ram address).

 * @param[in,out]  y     - y data(ram address)
 * @return  1: success 
 */
_attribute_ram_code_sec_  unsigned char  Draw_a_square_test(signed short *x,signed short *y)
{
     static unsigned int cnt = 0;
     static unsigned char flag = 0;
    signed short step = 1;
    // static uint32_t cnt2 = 0;
    // static unsigned char round = 0;

     if (cnt >= g_square_side_len)
    //if (cnt >= 50)
    {
        cnt = 0;
        flag++;
        if (flag > 3)
        {
            flag = 0;
            #if 0
            round++;
            
            if(round >= 1)
            {
                step = 0;
                // core_interrupt_disable();
                // while(1);
            }
                #endif
        }

    }
    cnt++;
    // if (flag == 4)
    // {
    
    // }
    // if ((flag == 0) && (round == 0))
    if ((flag == 0))
    {
    #if 0
        cnt2++;
        if(cnt2>10)
        {
            // cnt2=0;
            step = 11;
        }
        if (cnt2 > 11 && cnt2 <= 21)
        {
            step = 0;
        }
        if(cnt2 > 21)
        {
            step = 1;
        }
    #endif
        x[0] = step;
        y[0] = 0;

    }
    else if (flag == 1)
    {
        x[0] = 0;
        y[0] = step;
    }
    else if (flag == 2)
    {
        x[0] = -step;
        y[0] = 0;
    }
    else if (flag == 3)
    {
        x[0] = 0;
        y[0] = -step;
        // cnt2 = 0;
    }
    return 1;

}


/**
 * @brief   Simulate mouse data
 * @param[in,out]  x     - x data(ram address).

 * @return  1: success 
 */
 _attribute_ram_code_sec_  unsigned char  Draw_two_point_test(signed short *x)
{
  
     static unsigned char flag = 0;
    signed short step = 1;

    flag = 1-flag;
       
    if (flag == 0)
    {
        x[0] = step;
       
    }
    else
    {
        x[0] = -step;
    }
   
    return 1;

}

#if 1
typedef struct {
    s8 cir_x;
    s8 cir_y;
} circle_data_t;

const circle_data_t circle_radius_100pix[] =
{
    {100, 1}, {100, 2}, {100, 3}, {100, 4}, {100, 5}, {100, 6}, {100, 7}, {100, 8}, {100, 9}, {100, 10},
    {99, 11}, {99, 12}, {99, 13}, {99, 14}, {99, 15}, {99, 16}, {99, 17}, {98, 18}, {98, 19}, {98, 20},
    {98, 21}, {98, 22}, {97, 23}, {97, 24}, {97, 25}, {97, 26}, {96, 27}, {96, 28}, {96, 29}, {95, 30},
    {95, 31}, {95, 32}, {94, 33}, {94, 34}, {94, 35}, {93, 36}, {93, 37}, {93, 38}, {92, 39}, {92, 40},
    {91, 41}, {91, 42}, {90, 43}, {90, 44}, {89, 45}, {89, 46}, {88, 47}, {88, 48}, {87, 49}, {87, 50},
    {86, 51}, {85, 52}, {85, 53}, {84, 54}, {84, 55}, {83, 56}, {82, 57}, {81, 58}, {81, 59}, {80, 60},
    {79, 61}, {78, 62}, {78, 63}, {77, 64}, {76, 65}, {75, 66}, {74, 67}, {73, 68}, {72, 69}, {71, 70},
    {70, 71}, {69, 72}, {68, 73}, {67, 74}, {66, 75}, {65, 76}, {64, 77}, {63, 78}, {62, 78}, {61, 79},
    {60, 80}, {59, 81}, {58, 81}, {57, 82}, {56, 83}, {55, 84}, {54, 84}, {53, 85}, {52, 85}, {51, 86},
    {50, 87}, {49, 87}, {48, 88}, {47, 88}, {46, 89}, {45, 89}, {44, 90}, {43, 90}, {42, 91}, {41, 91},
    {40, 92}, {39, 92}, {38, 93}, {37, 93}, {36, 93}, {35, 94}, {34, 94}, {33, 94}, {32, 95}, {31, 95},
    {30, 95}, {29, 96}, {28, 96}, {27, 96}, {26, 97}, {25, 97}, {24, 97}, {23, 97}, {22, 98}, {21, 98},
    {20, 98}, {19, 98}, {18, 98}, {17, 99}, {16, 99}, {15, 99}, {14, 99}, {13, 99}, {12, 99}, {11, 99},
    {10, 100}, {9, 100}, {8, 100}, {7, 100}, {6, 100}, {5, 100}, {4, 100}, {3, 100}, {2, 100}, {1, 100},
    {-1, 100}, {-2, 100}, {-3, 100}, {-4, 100}, {-5, 100}, {-6, 100}, {-7, 100}, {-8, 100}, {-9, 100}, {-10, 100},
    {-11, 99}, {-12, 99}, {-13, 99}, {-14, 99}, {-15, 99}, {-16, 99}, {-17, 99}, {-18, 98}, {-19, 98}, {-20, 98},
    {-21, 98}, {-22, 98}, {-23, 97}, {-24, 97}, {-25, 97}, {-26, 97}, {-27, 96}, {-28, 96}, {-29, 96}, {-30, 95},
    {-31, 95}, {-32, 95}, {-33, 94}, {-34, 94}, {-35, 94}, {-36, 93}, {-37, 93}, {-38, 93}, {-39, 92}, {-40, 92},
    {-41, 91}, {-42, 91}, {-43, 90}, {-44, 90}, {-45, 89}, {-46, 89}, {-47, 88}, {-48, 88}, {-49, 87}, {-50, 87},
    {-51, 86}, {-52, 85}, {-53, 85}, {-54, 84}, {-55, 84}, {-56, 83}, {-57, 82}, {-58, 81}, {-59, 81}, {-60, 80},
    {-61, 79}, {-62, 78}, {-63, 78}, {-64, 77}, {-65, 76}, {-66, 75}, {-67, 74}, {-68, 73}, {-69, 72}, {-70, 71},
    {-71, 70}, {-72, 69}, {-73, 68}, {-74, 67}, {-75, 66}, {-76, 65}, {-77, 64}, {-78, 63}, {-78, 62}, {-79, 61},
    {-80, 60}, {-81, 59}, {-81, 58}, {-82, 57}, {-83, 56}, {-84, 55}, {-84, 54}, {-85, 53}, {-85, 52}, {-86, 51},
    {-87, 50}, {-87, 49}, {-88, 48}, {-88, 47}, {-89, 46}, {-89, 45}, {-90, 44}, {-90, 43}, {-91, 42}, {-91, 41},
    {-92, 40}, {-92, 39}, {-93, 38}, {-93, 37}, {-93, 36}, {-94, 35}, {-94, 34}, {-94, 33}, {-95, 32}, {-95, 31},
    {-95, 30}, {-96, 29}, {-96, 28}, {-96, 27}, {-97, 26}, {-97, 25}, {-97, 24}, {-97, 23}, {-98, 22}, {-98, 21},
    {-98, 20}, {-98, 19}, {-98, 18}, {-99, 17}, {-99, 16}, {-99, 15}, {-99, 14}, {-99, 13}, {-99, 12}, {-99, 11},
    {-100, 10}, {-100, 9}, {-100, 8}, {-100, 7}, {-100, 6}, {-100, 5}, {-100, 4}, {-100, 3}, {-100, 2}, {-100, 1},
    {-100, -1}, {-100, -2}, {-100, -3}, {-100, -4}, {-100, -5}, {-100, -6}, {-100, -7}, {-100, -8}, {-100, -9}, {-100, -10},
    {-99, -11}, {-99, -12}, {-99, -13}, {-99, -14}, {-99, -15}, {-99, -16}, {-99, -17}, {-98, -18}, {-98, -19}, {-98, -20},
    {-98, -21}, {-98, -22}, {-97, -23}, {-97, -24}, {-97, -25}, {-97, -26}, {-96, -27}, {-96, -28}, {-96, -29}, {-95, -30},
    {-95, -31}, {-95, -32}, {-94, -33}, {-94, -34}, {-94, -35}, {-93, -36}, {-93, -37}, {-93, -38}, {-92, -39}, {-92, -40},
    {-91, -41}, {-91, -42}, {-90, -43}, {-90, -44}, {-89, -45}, {-89, -46}, {-88, -47}, {-88, -48}, {-87, -49}, {-87, -50},
    {-86, -51}, {-85, -52}, {-85, -53}, {-84, -54}, {-84, -55}, {-83, -56}, {-82, -57}, {-81, -58}, {-81, -59}, {-80, -60},
    {-79, -61}, {-78, -62}, {-78, -63}, {-77, -64}, {-76, -65}, {-75, -66}, {-74, -67}, {-73, -68}, {-72, -69}, {-71, -70},
    {-70, -71}, {-69, -72}, {-68, -73}, {-67, -74}, {-66, -75}, {-65, -76}, {-64, -77}, {-63, -78}, {-62, -78}, {-61, -79},
    {-60, -80}, {-59, -81}, {-58, -81}, {-57, -82}, {-56, -83}, {-55, -84}, {-54, -84}, {-53, -85}, {-52, -85}, {-51, -86},
    {-50, -87}, {-49, -87}, {-48, -88}, {-47, -88}, {-46, -89}, {-45, -89}, {-44, -90}, {-43, -90}, {-42, -91}, {-41, -91},
    {-40, -92}, {-39, -92}, {-38, -93}, {-37, -93}, {-36, -93}, {-35, -94}, {-34, -94}, {-33, -94}, {-32, -95}, {-31, -95},
    {-30, -95}, {-29, -96}, {-28, -96}, {-27, -96}, {-26, -97}, {-25, -97}, {-24, -97}, {-23, -97}, {-22, -98}, {-21, -98},
    {-20, -98}, {-19, -98}, {-18, -98}, {-17, -99}, {-16, -99}, {-15, -99}, {-14, -99}, {-13, -99}, {-12, -99}, {-11, -99},
    {-10, -100}, {-9, -100}, {-8, -100}, {-7, -100}, {-6, -100}, {-5, -100}, {-4, -100}, {-3, -100}, {-2, -100}, {-1, -100},
    {1, -100}, {2, -100}, {3, -100}, {4, -100}, {5, -100}, {6, -100}, {7, -100}, {8, -100}, {9, -100}, {10, -100},
    {11, -99}, {12, -99}, {13, -99}, {14, -99}, {15, -99}, {16, -99}, {17, -99}, {18, -98}, {19, -98}, {20, -98},
    {21, -98}, {22, -98}, {23, -97}, {24, -97}, {25, -97}, {26, -97}, {27, -96}, {28, -96}, {29, -96}, {30, -95},
    {31, -95}, {32, -95}, {33, -94}, {34, -94}, {35, -94}, {36, -93}, {37, -93}, {38, -93}, {39, -92}, {40, -92},
    {41, -91}, {42, -91}, {43, -90}, {44, -90}, {45, -89}, {46, -89}, {47, -88}, {48, -88}, {49, -87}, {50, -87},
    {51, -86}, {52, -85}, {53, -85}, {54, -84}, {55, -84}, {56, -83}, {57, -82}, {58, -81}, {59, -81}, {60, -80},
    {61, -79}, {62, -78}, {63, -78}, {64, -77}, {65, -76}, {66, -75}, {67, -74}, {68, -73}, {69, -72}, {70, -71},
    {71, -70}, {72, -69}, {73, -68}, {74, -67}, {75, -66}, {76, -65}, {77, -64}, {78, -63}, {78, -62}, {79, -61},
    {80, -60}, {81, -59}, {81, -58}, {82, -57}, {83, -56}, {84, -55}, {84, -54}, {85, -53}, {85, -52}, {86, -51},
    {87, -50}, {87, -49}, {88, -48}, {88, -47}, {89, -46}, {89, -45}, {90, -44}, {90, -43}, {91, -42}, {91, -41},
    {92, -40}, {92, -39}, {93, -38}, {93, -37}, {93, -36}, {94, -35}, {94, -34}, {94, -33}, {95, -32}, {95, -31},
    {95, -30}, {96, -29}, {96, -28}, {96, -27}, {97, -26}, {97, -25}, {97, -24}, {97, -23}, {98, -22}, {98, -21},
    {98, -20}, {98, -19}, {98, -18}, {99, -17}, {99, -16}, {99, -15}, {99, -14}, {99, -13}, {99, -12}, {99, -11},
    {100, -10}, {100, -9}, {100, -8}, {100, -7}, {100, -6}, {100, -5}, {100, -4}, {100, -3}, {100, -2}, {100, -1},
};

_attribute_ram_code_sec_ unsigned char Draw_a_circle(signed short *x,signed short *y)
{
    static u16 data_idx = 0;
    circle_data_t* p_circle_data = (circle_data_t*)&circle_radius_100pix;
    u16 max_data_cnt = sizeof(circle_radius_100pix)/sizeof(circle_data_t);

    u16 data_idx_next = (data_idx < (max_data_cnt -1)) ? (data_idx+1) : 0;
    x[0] = p_circle_data[data_idx_next].cir_x - p_circle_data[data_idx].cir_x;
    y[0] = p_circle_data[data_idx_next].cir_y - p_circle_data[data_idx].cir_y;
    x[0] = x[0] * 2;    //Enlarge X-axis movement step for bigger circle
    y[0] = y[0] * 2;    //Enlarge Y-axis movement step for bigger circle

    if(data_idx < (max_data_cnt -1))
        data_idx++;
    else
        data_idx = 0;

    return 1;
}
#endif

// _attribute_ram_code_sec_ static unsigned char __Draw_a_square_test(signed short *x, signed short *y)
// {
//     #if 0
//     return Draw_a_square(x,y);
//     #else
//     return Draw_a_circle(x,y);
//     #endif
// }

void app_get_kb_data(void)
{
    static unsigned char kb_data[8]={0,0,0,0,0,0,0,0};
    static unsigned int tick;
    
    if(clock_time_exceed(tick, 1000))
    {
        tick+=1000 * SYSTEM_TIMER_TICK_1US;
        if(kb_data[2]==0)
        {
             kb_data[2]     = 0x04;
        }
        else
        {
             kb_data[2]     = 0x0;
        }
        pp_fifo_push(&tx_fifo, NORMAL_KB_DATA_CMD, kb_data, 8);
        
    }
    

}
#define MOUSE_INTERVAL_US   125

void app_get_mouse_data(void)
{
    
    static unsigned int tick;
    int has_new_data_flag=0;
    int ret=0;
    #if 0
    unsigned char  status=0;
    status=app_usb_ep_is_idle(HID_MOUSE_IN_ENDPOINT_NUM);
    if(status!=0)
    {
        return ;
    }
    #else
    if(clock_time_exceed(tick, MOUSE_INTERVAL_US))
    #endif
    {
        tick+=MOUSE_INTERVAL_US * SYSTEM_TIMER_TICK_1US;
        ret=Draw_a_square_test(&ms_data.x,&ms_data.y);
        has_new_data_flag|=ret;
        if(has_new_data_flag)
        {
            pp_fifo_push(&tx_fifo, MOUSE_DATA, &ms_data.report_id, sizeof(t_mouse_data));
        }
    }
    

}
unsigned char uart_tx_buff[2][128*2] __attribute__((aligned(4)));
volatile int uart_active_buf_idx = 0;

void app_uart_init(void)
{    
    unsigned short div;
    unsigned char  bwpc;
    uart_hw_fsm_reset(UART0);
    uart_set_pin(UART0, GPIO_FC_PG7, GPIO_FC_PC0);
    uart_cal_div_and_bwpc(2000000, sys_clk.pclk * 1000 * 1000, &div, &bwpc);
    uart_set_rx_timeout_with_exp(UART0, bwpc, 12, UART_BW_MUL2, 0);
    uart_init(UART0, div, bwpc, UART_PARITY_NONE, UART_STOP_BIT_ONE);
    uart_set_tx_dma_config(UART0, DMA3);
    uart_set_rx_dma_config(UART0, DMA2);
    uart_set_irq_mask(UART0, UART_TXDONE_MASK);
    plic_interrupt_enable(IRQ_UART0);


}
//_attribute_ram_code_sec_ void uart0_irq_handler(void)
//{
//    if (uart_get_irq_status(UART0, UART_TXDONE_IRQ_STATUS))
//    {
//        uart_clr_irq_status(UART0, UART_TXDONE_IRQ_STATUS);
//    }
//}
//PLIC_ISR_REGISTER(uart0_irq_handler, IRQ_UART0);
#if 0
unsigned char print_buf[120] = "crc_err=%d,rx_to=%d, d_ok:%d d_err:%d per=%d fno:%x chn_mask:%x\r\n";

extern uint8_t chn_mask;
extern volatile bool chn_mask_change;
void rf_print_cnt_debug(void)
{
    static unsigned int tick;
    // if(clock_time_exceed(tick, 1000000))
    if(chn_mask_change )
    {
        gpio_set_level(GPIO_PG6, 1);
        chn_mask_change = false;
        tick+=1000000*SYSTEM_TIMER_TICK_1US;

        unsigned int       rx_irq=cnt_rx_irq;
        unsigned int       crc_ok=cnt_rx_crc_ok;
        unsigned int       d_ok=cnt_rx_d_ok;
        unsigned int       crc_err=cnt_rx_crc_err;
        unsigned int       data_err=cnt_data_err;

        unsigned int      tx_cnt=cnt_tx;
        unsigned int      tx_irq=cnt_tx_irq;
        unsigned int      cnt_usb=cnt_to_usb;
        unsigned int      per=1000-cnt_usb*1000/8000;
//      unsigned int      loss=8000-cnt_usb;
        int buf_idx = uart_active_buf_idx ^ 1;
        // unsigned int len = snprintf((char*)uart_tx_buff[buf_idx], sizeof(uart_tx_buff[0]),
        //  "r_irq=%d,crc_ok=%d,d_ok=%d,crc_err=%d,t_cnt=%d,t_irq=%d,to_usb=%d,per=%d,loss=%d\r\n",
        //  rx_irq,crc_ok, d_ok,crc_err,tx_cnt,tx_irq,cnt_usb,per,loss);
gpio_set_level(GPIO_PG6, 0);
        // unsigned int len = snprintf((char*)uart_tx_buff[buf_idx], sizeof(uart_tx_buff[0]),
        //     "crc_err=%d,rx_to=%d, d_ok:%d d_err:%d per=%d\r\n", crc_err, rf_rx_fail_cnt,d_ok, data_err, per);
        // unsigned int len = tvsnprintf((char*)uart_tx_buff[buf_idx], sizeof(uart_tx_buff[0]),
        //     print_buf, crc_err, rf_rx_fail_cnt,d_ok, data_err, per, ll_ctx.fno_last, chn_mask);
            // rx_irq,crc_ok, d_ok,crc_err,tx_cnt,tx_irq,cnt_usb,per,loss);
gpio_set_level(GPIO_PG6, 1);
        cnt_rx_irq-=rx_irq;
        cnt_rx_crc_ok-=crc_ok;
        cnt_rx_d_ok-=d_ok;
        cnt_rx_crc_err-=crc_err;
        cnt_data_err -=data_err;
        cnt_tx-=tx_cnt;
        cnt_tx_irq-=tx_irq;
        cnt_to_usb-=cnt_usb;
        rf_rx_fail_cnt = 0;
         // DMA sends valid data (without sending the termination character \0)
//          uart_send_dma(UART0, uart_tx_buff[buf_idx], len);
        // Switch buffers
         uart_active_buf_idx = buf_idx;
         gpio_set_level(GPIO_PG6, 0);

    }
}
#endif

//extern volatile unsigned int sof_cnt;
//extern volatile unsigned int stimer_cnt[4];
//extern volatile unsigned int systick[4];

#include "tl_usb/core/usbd_core.h"
extern unsigned char ep0_out_data[64];
void app_debug_print(unsigned int d1, unsigned int d2, unsigned int d3, unsigned int d4)
{

    plic_interrupt_disable(IRQ_SYSTIMER);

    core_interrupt_disable();
    usb_control_request_t *setup = (usb_control_request_t *)ep0_out_data;
    int buf_idx = uart_active_buf_idx ^ 1;
    unsigned int len = snprintf((char *)uart_tx_buff[buf_idx],
                                sizeof(uart_tx_buff[0]),
                                "gt1:%d  gt2:%d gt3:%d gt4:%d\n ep0 data:0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n setup:re-%d type-%d dir-%d \n", d1, d2, d3, d4,
                                ep0_out_data[0], ep0_out_data[1], ep0_out_data[2], ep0_out_data[3], ep0_out_data[4], ep0_out_data[5],
                                ep0_out_data[6], ep0_out_data[7], setup->bmRequestType_bit.recipient, setup->bmRequestType_bit.type, setup->bmRequestType_bit.direction);
//                                "last_stick:%d  timestamp:%d delta:%d stick:%d\nsof:%d %d %d %d\nsys:%d %d %d %d", d1, d2,
//                              d3, d4, stimer_cnt[(sof_cnt+1)%4], stimer_cnt[(sof_cnt+2)%4], stimer_cnt[(sof_cnt+3)%4],
//                              stimer_cnt[sof_cnt%4], systick[(syscnt+1)%4], systick[(syscnt+2)%4], systick[(syscnt+3)%4], systick[syscnt%4]);

    if (len >= sizeof(uart_tx_buff[0]))
    {
        len = sizeof(uart_tx_buff[0]) - 1;
        uart_tx_buff[buf_idx][len] = '\n'; // Ensure the presence of line breaks
    }

    uart_send_dma(UART0, uart_tx_buff[buf_idx], len);

    uart_active_buf_idx = buf_idx;

    while (1)
    {
        delay_ms(100);
        gpio_toggle(LED1);
    }
}

void app_halt_system(uint16_t error_code, const char* format, ...)
{
//    plic_interrupt_disable(IRQ_SYSTIMER | IRQ_ZB_RT | IRQ_USB0 | IRQ_TIMER1 | IRQ_TIMER0);
    core_interrupt_disable();

    g_2p4g_error_code = error_code;

    int buf_idx = uart_active_buf_idx ^ 1;
    unsigned int len1 = snprintf((char *)uart_tx_buff[buf_idx],
                                sizeof(uart_tx_buff[0]),
                                "err_code:0x%lX ", g_2p4g_error_code);

    va_list args;
    va_start(args, format);

    unsigned int len2 = vsnprintf((char *)&uart_tx_buff[buf_idx][len1], sizeof(uart_tx_buff[0]) - len1, format, args);

    va_end(args);

    unsigned int len = len1 + len2;


    if (len >= sizeof(uart_tx_buff[0]))
    {
        len = sizeof(uart_tx_buff[0]) - 1;
    }
    uart_tx_buff[buf_idx][len] = 0;

    tlk_printf("%s", &uart_tx_buff[buf_idx][0]);

    uart_active_buf_idx = buf_idx;

    while (g_2p4g_error_code)
    {
#if (TLKAPI_DEBUG_ENABLE)
        tlkapi_debug_handler();
#endif
        delay_ms(100);
        gpio_toggle(LED1);
    }
}

_attribute_ram_code_sec_ static void append_char(char *str, char c, unsigned int *pos, unsigned int max_size)
{
    if (*pos < max_size - 1) {
        str[*pos] = c;
        (*pos)++;
    }
}

_attribute_ram_code_sec_ static void append_string(char *str, unsigned char *src, unsigned int *pos, unsigned int max_size)
{
    while (*src && *pos < max_size - 1) {
        str[*pos] = *src;
        src++;
        (*pos)++;
    }
}

_attribute_ram_code_sec_ static void append_int(char *str, int value, unsigned int *pos, unsigned int max_size)
{
    char buffer[20];
    int i = 0;
    int negative = 0;

    if (value < 0) {
        negative = 1;
        value = -value;
    }

    do {
        buffer[i++] = '0' + (value % 10);
        value /= 10;
    } while (value > 0);

    if (negative) {
        buffer[i++] = '-';
    }

    for (int j = i - 1; j >= 0; j--) {
        append_char(str, buffer[j], pos, max_size);
    }
}

_attribute_ram_code_sec_ static void append_double(char *str, double value, unsigned int *pos, unsigned int max_size)
{
    // Handle negative numbers
    if (value < 0) {
        append_char(str, '-', pos, max_size);
        value = -value;
    }

    int integer_part = (int)value;
    double fractional_part = value - integer_part;

    // Convert integer part
    append_int(str, integer_part, pos, max_size);
    append_char(str, '.', pos, max_size);

    // Convert fractional part
    for (int i = 0; i < 6; i++) { // Assuming 6 decimal places
        fractional_part *= 10;
        int digit = (int)fractional_part;
        append_char(str, '0' + digit, pos, max_size);
        fractional_part -= digit;
    }
}

_attribute_ram_code_sec_ static void append_hex(char *str, unsigned int value, unsigned int *pos, unsigned int max_size, int uppercase)
{
    char buffer[16];
    int i = 0;
    const char *hex_digits = uppercase ? "0123456789ABCDEF" : "0123456789abcdef";

    if (value == 0) {
        append_char(str, '0', pos, max_size);
        return;
    }

    while (value > 0 && i < 16) {
        buffer[i++] = hex_digits[value % 16];
        value /= 16;
    }

    for (int j = i - 1; j >= 0; j--) {
        append_char(str, buffer[j], pos, max_size);
    }
}

_attribute_ram_code_sec_ int tvsnprintf(char *str, unsigned int size, unsigned char *format, ...)
{
    va_list args;
    va_start(args, format);

    unsigned int pos = 0;
    unsigned char *p = format;

    while (*p && pos < size - 1) {
        if (*p == '%') {
            p++;
            switch (*p) {
                case 'd':
                    append_int(str, va_arg(args, int), &pos, size);
                    break;
                case 'x':
                    append_hex(str, va_arg(args, unsigned int), &pos, size, 0); // lowercase
                    break;
                case 'X':
                    append_hex(str, va_arg(args, unsigned int), &pos, size, 1); // uppercase
                    break;
                case 'f':
                    append_double(str, va_arg(args, double), &pos, size);
                    break;
                case 's':
                    append_string(str, va_arg(args, unsigned char*), &pos, size);
                    break;
                case '%':
                    append_char(str, '%', &pos, size);
                    break;
                default:
                    append_char(str, '%', &pos, size);
                    append_char(str, *p, &pos, size);
                    break;
            }
            p++;
        } else {
            append_char(str, *p, &pos, size);
            p++;
        }
    }

    str[pos] = '\0';
    va_end(args);

    return pos; // Return the number of characters that would have been written
}

_attribute_ram_code_sec_ void tprintf(unsigned char* format, ...)
{
    int buf_idx = uart_active_buf_idx ^ 1;

    va_list args;
    va_start(args, format);

    unsigned int len = tvsnprintf((char *)uart_tx_buff[buf_idx], sizeof(uart_tx_buff[0]), format, args);

    va_end(args);

    uart_send_dma(UART0, uart_tx_buff[buf_idx], len);

    uart_active_buf_idx = buf_idx;
}
void app_allow_pair_new_device_power_on_handle(void)
{
    static int flag=0;
    if(tpsll_get_state()==STATE_CONNECTED)
    {
        flag_allow_pair_new_deviec_afer_power_on=0;
    }
    else if(flag_allow_pair_new_deviec_afer_power_on)
    {
        if(tick_allow_pair_new_deviec_afer_power_on && clock_time_exceed(tick_allow_pair_new_deviec_afer_power_on, 5000000))
        {
            flag_allow_pair_new_deviec_afer_power_on=0;
            tpsll_enable_reconn(true);
        }
        else if(tick_timer_pair_reconnect && clock_time_exceed(tick_timer_pair_reconnect, 100000))
        {
            tick_timer_pair_reconnect = clock_time() | 1;

            if(tpsll_get_state()!=STATE_RECONNECT)
            {
                tpsll_enable_reconn(true);
            }
            else
            {
                tpsll_enable_pairing(1);
            }
            flag=1-flag;
        }
        
    }
    
    
}
