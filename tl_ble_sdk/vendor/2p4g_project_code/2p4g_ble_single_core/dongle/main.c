/********************************************************************************************************
 * @file    main.c
 *
 * @brief   This is the source file for Telink RISC-V MCU
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
#include "app_public.h"
#include "../../hsm/sign_verify.h"
extern void user_init(void);
extern void main_loop(void);
#define sc_clk_1 REG_ADDR8(0x140800 + 0x48)
#define sc_clk_2 REG_ADDR8(0x140800 + 0x69)

  unsigned int app_version=0x260411;

#if DRIVER_BQB_TEST
u8 bqb_test_mode(void)
{
// determine whether to enter the bqb test mode based on factors such as GPIO, wakeup source, etc
    return 1;
}
#endif


#if TEST_SECU_SIGN_VRFY_EN
#ifndef FLASH_BASS_ADDR
#define FLASH_BASS_ADDR  0x20000000
#endif
volatile uint32_t Secure_data_bin_addr_test = FLASH_BASS_ADDR + 0x50000;//bin
volatile uint32_t Secure_data_start_addr_test = FLASH_BASS_ADDR + 0x50000 + 0x333D4;//tag magic
//volatile uint32_t Secure_data_start_addr_test = FLASH_BASS_ADDR + 0x50000;//tag magic
volatile uint32_t Secure_data_end_addr_test = FLASH_BASS_ADDR + 0x50000 + 0x34226;//end
volatile uint32_t Root_cert_addr = FLASH_BASS_ADDR + 0x40000;//end
unsigned int bin_size = 0x333f3 - 31;
volatile unsigned char AAAtest_cnt = 0;
volatile int AAAtest_stat = 0;
volatile unsigned int AAAtest_time_us = 0;
#endif

/**
 * @brief      application system initialization
 * @param[in]  none.
 * @return     none.
 * @note       LDO mode is used by default to meet compatibility requirements.
 *             If the hardware supports DCDC, DCDC mode is recommended.
 */
void tlkapp_system_init(void)
{
#ifdef MCU_STARTUP_NVM // only defined in dongle_nvm version
    analog_write_reg8(areg_aon_0x05, analog_read_reg8(areg_aon_0x05) & ~(FLD_24M_RC_PD)); //power on 24M RC
    clock_set_all_clock_to_default();
#endif
    sys_init(LDO_1P25_LDO_1P8, VBAT_MAX_VALUE_GREATER_THAN_3V6, INTERNAL_CAP_XTAL24M);

    gpio_init(0);

    lpc_pem_flash_prot_config(LPC_INPUT_PB5, PEM0, DMA7);


    pm_update_status_info(1);
    gpio_set_up_down_res(GPIO_SWS, GPIO_PIN_PULLUP_1M);
    wd_32k_stop();
    wd_stop();

    pm_set_dig_ldo(DIG_VOL_1V1_MODE, 1000);

//    PLL_192M_D25F_192M_HCLK_N22_96M_PCLK_96M_MSPI_48M;//CLOCK_INIT;
    PLL_192M_D25F_96M_HCLK_N22_96M_PCLK_96M_MSPI_48M;

    //CLK_DIV2
    //     CCLK_DIV1_TO_HCLK_DIV1_TO_PCLK = 0, //cclk:hclk:pclk = 1:1:1; //14.6
    // CCLK_DIV1_TO_HCLK_DIV2_TO_PCLK = 1, //cclk:hclk:pclk = 1:1:2;//14.6
    // CCLK_DIV1_TO_HCLK_DIV4_TO_PCLK = 2, //cclk:hclk:pclk = 1:1:4;//14.6
    // CCLK_DIV2_TO_HCLK_DIV2_TO_PCLK = 4, //cclk:hclk:pclk = 1:2:2;//14.75
    // CCLK_DIV2_TO_HCLK_DIV4_TO_PCLK = 5, //cclk:hclk:pclk = 1:2:4;//14.9


    
    //CLK_DIV4
    //     CCLK_DIV1_TO_HCLK_DIV1_TO_PCLK = 0, //cclk:hclk:pclk = 1:1:1; //15
    // CCLK_DIV1_TO_HCLK_DIV2_TO_PCLK = 1, //cclk:hclk:pclk = 1:1:2;//15
    // CCLK_DIV1_TO_HCLK_DIV4_TO_PCLK = 2, //cclk:hclk:pclk = 1:1:4;//15
    // CCLK_DIV2_TO_HCLK_DIV2_TO_PCLK = 4, //cclk:hclk:pclk = 1:2:2;//15.25
    // CCLK_DIV2_TO_HCLK_DIV4_TO_PCLK = 5, //cclk:hclk:pclk = 1:2:4;//15.25

    //CLK_DIV8
    //     CCLK_DIV1_TO_HCLK_DIV1_TO_PCLK = 0, //cclk:hclk:pclk = 1:1:1; //15.35
    // CCLK_DIV1_TO_HCLK_DIV2_TO_PCLK = 1, //cclk:hclk:pclk = 1:1:2;//
    // CCLK_DIV1_TO_HCLK_DIV4_TO_PCLK = 2, //cclk:hclk:pclk = 1:1:4;//15.35
    // CCLK_DIV2_TO_HCLK_DIV2_TO_PCLK = 4, //cclk:hclk:pclk = 1:2:2;//15.9
    // CCLK_DIV2_TO_HCLK_DIV4_TO_PCLK = 5, //cclk:hclk:pclk = 1:2:4;//

    // clock_init(CLK_BASEBAND_PLL_192M, CLK_DIV8, CCLK_DIV2_TO_HCLK_DIV2_TO_PCLK, CLK_DIV4);
    // PLL_192M_D25F_24M_HCLK_N22_24M_PCLK_12M_MSPI_48M;

    sys_n22_init(0x20080000);
    // pm_set_dig_module_power_switch(FLD_PD_ZB_EN,PM_POWER_UP);
    rf_n22_dig_init();
    rf_clr_irq_mask(FLD_RF_IRQ_ALL);
}



/**
 * @brief       This is main function
 * @param[in]   none
 * @return      none
 */
int main(void)
{
    tlkapp_system_init();

#if MCU_RUN_IN_NVM
#if APP_NVM_PROTECTION_ENABLE
    if (nvm_read_reg(NVM_READ_MP_CMD) != NVM_MTP_LOCK_ALL_512K) {
        nvm_mtp_lock_set(NVM_MTP_LOCK_ALL_512K);
    }
#endif
#endif

#if DRIVER_BQB_TEST
    if(bqb_test_mode()){
        bqb_user_init();
        while(1){
            bqb_serviceloop();
        }
    }
#endif

    user_init();

    if(app_version==0)
    {
        delay_us(1);
    }
#if TEST_SECU_SIGN_VRFY_EN

    unsigned int start_tick = clock_time();
    int ret = tlk_secu_sign_verify(Secure_data_bin_addr_test, bin_size, Secure_data_start_addr_test, Root_cert_addr, 1);
    AAAtest_stat = ret;
    unsigned int end_tick = clock_time();
    AAAtest_time_us = (end_tick - start_tick)/24;

    if(ret)
    {
        AAAtest_cnt = 0x0a;
        while(1);
    }
#endif



    while (1) {
        main_loop();
    }

    return 0;
}

_attribute_ram_code_sec_ void rf_irq_handler(void)
{
    tpsll_rf_irq_handler();
    extern uint32_t esd_test_tick;
    if(app_get_usb_enum_status())
        esd_test_tick = stimer_get_tick();
}

_attribute_ram_code_sec_ void stimer_irq_handler(void)
{
    tpsll_stimer_irq_handler();
}


_attribute_ram_code_sec_ void timer1_irq_handler(void)
{
    tpsll_timer1_irq_handler();
}


        #if defined(MCU_CORE_TL322X_N22)
CLIC_ISR_REGISTER(rf_irq_handler, IRQ_ZB_RT)
        #else
PLIC_ISR_REGISTER(rf_irq_handler, IRQ_ZB_RT)
        #endif
PLIC_ISR_REGISTER(stimer_irq_handler, IRQ_SYSTIMER)
PLIC_ISR_REGISTER(timer1_irq_handler, IRQ_TIMER1)

