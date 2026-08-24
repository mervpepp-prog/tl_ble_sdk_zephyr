/********************************************************************************************************
 * @file    main.c
 *
 * @brief   This is the source file for BLE SDK
 *
 * @author  BLE GROUP
 * @date    06,2022
 *
 * @par     Copyright (c) 2022, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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
#include <vendor/n22_2p4g_tpll/app.h>
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "hci_transport/hci_dfu.h"

#include "stack/multicore_comm/service/service_n22.h"
extern  void n22_send_tx_status(uint8_t tx_status);

volatile unsigned int rx_irq_cnt_rx_dr       = 0;
volatile unsigned int rx_irq_cnt_invalid_pid = 0;
volatile unsigned int rx_irq_cnt_rx          = 0;
volatile unsigned int rx_irq_cnt_tx          = 0;
volatile unsigned int rx_irq_cnt_tx_ds       = 0;
volatile unsigned int math_cnt               = 0;
volatile unsigned int unmath_cnt             = 0;
volatile unsigned int rx_sync_cnt            = 0;
volatile unsigned int rx_timeout_flag        = 0;
volatile unsigned int irq_cnt_rx_timeout     = 0;
volatile unsigned char rx_flag               = 0;
volatile unsigned char rx_dr_flag            = 0;
volatile unsigned int irq_cnt_max_retry   = 0;


volatile unsigned char tx_ds_flag       = 0;
volatile unsigned char maxretry_flag = 0;
volatile unsigned char tx_done_flag = 0;
__attribute__((section(".ram_code"))) __attribute__((optimize("-Os"))) void tpll_irq_handler(void)
{
    unsigned char pipe = TPLL_GetTXPipe();

    if (rf_get_irq_status(FLD_RF_IRQ_TX)) {
        reg_rf_irq_status = FLD_RF_IRQ_TX;
        rx_irq_cnt_tx++;
        tx_done_flag = 1;
    }
    if (rf_get_irq_status(FLD_RF_IRQ_RX)) {
        reg_rf_irq_status = FLD_RF_IRQ_RX;
        rx_irq_cnt_rx++;
        rx_flag = 1;
    }
    if (rf_get_irq_status(FLD_RF_IRQ_TX_DS)) {
        //Data Sent TX FIFO interrupt. Asserted when packet transmitted on TX.
        //If AUTO_ACK is activated, this bit is set high only when ACK is received.
        reg_rf_irq_status = FLD_RF_IRQ_TX_DS;
        rx_irq_cnt_tx_ds++;
        tx_ds_flag = 1;
    }
    if (rf_get_irq_status(FLD_RF_IRQ_TX_RETRYCNT)) {
        //tx_retrycnt_irq:Maximum number of TX retransmits interrupt.
        //Asserted when retry counter reaches the max numbers.
        reg_rf_irq_status = FLD_RF_IRQ_TX_RETRYCNT;
        irq_cnt_max_retry++;
        maxretry_flag = 1;
        //adjust rptr
        TPLL_UpdateTXFifoRptr(pipe);
    }
    if (rf_get_irq_status(FLD_RF_IRQ_RX_TIMEOUT)) {
        rx_timeout_flag = 1;
        irq_cnt_rx_timeout++;
        rf_clr_irq_status(FLD_RF_IRQ_RX_TIMEOUT);
    }
        rf_clr_irq_status(FLD_RF_IRQ_ALL);
}

CLIC_ISR_REGISTER(tpll_irq_handler, IRQ_ZB_RT)


/**
 * @brief       BLE RF interrupt handler.
 * @param[in]   none
 * @return      none
 */
_attribute_ram_code_ void rf_irq_handler(void)
{
    DBG_CHN14_HIGH;
    //gpio_write(GPIO_PC1, 1);

    tpll_irq_handler();
//    blc_sdk_irq_handler();
#if 0
    if(blm_btxbrx_state){
        gpio_write(GPIO_PC2, 1);
    }
    else{
        gpio_write(GPIO_PC2, 0);
    }
#endif
    DBG_CHN14_LOW;
    //gpio_write(GPIO_PC1, 0);
}
//CLIC_ISR_REGISTER(rf_irq_handler, IRQ_ZB_RT)

/**
 * @brief       System timer interrupt handler.
 * @param[in]   none
 * @return      none
 */
_attribute_ram_code_ void stimer_irq_handler(void)
{
    DBG_CHN15_HIGH;
    //gpio_write(GPIO_PC0, 1);

    blc_sdk_irq_handler();
#if 0
    if(blm_btxbrx_state){
        gpio_write(GPIO_PC2, 1);
    }
    else{
        gpio_write(GPIO_PC2, 0);
    }
#endif
    DBG_CHN15_LOW;
    //gpio_write(GPIO_PC0, 0);
}
//CLIC_ISR_REGISTER(stimer_irq_handler, IRQ_SYSTIMER)

#if (APP_LE_CHANNEL_SOUNDING_TEST_MODE || APP_LE_CHANNEL_SOUNDING)
/**
 * @brief       timer0 interrupt handler.
 * @param[in]   none
 * @return      none
 */
_attribute_ram_code_ void timer0_irq_handler(void)
{
#if (MCU_CORE_TYPE == MCU_CORE_TL721X)
    if (timer_get_irq_status(FLD_TMR0_MODE_IRQ))
#else
    if (timer_get_irq_status(FLD_TMR0_MODE_IRQ))
#endif
    {
        u32 r = core_interrupt_disable();
        reg_tmr_ctrl0 &= ~FLD_TMR0_EN;
#if (MCU_CORE_TYPE == MCU_CORE_TL721X)
        timer_clr_irq_status(FLD_TMR0_MODE_IRQ); //clear irq status
#else
        timer_clr_irq_status(FLD_TMR0_MODE_IRQ); //clear irq status
#endif
        core_restore_interrupt(r);

        if (ll_cs_rawData_process_cb) {
            ll_cs_rawData_process_cb();
        }
        if (ll_cs_hci_subevent_report_cb) {
            ll_cs_hci_subevent_report_cb();
        }
    }
}
CLIC_ISR_REGISTER(timer0_irq_handler, IRQ_TIMER0)
#endif
/**
 * @brief      application system initialization
 * @param[in]  none.
 * @return     none.
 */
__INLINE void blc_app_system_init(void)
{
    clic_init();
    clic_preempt_feature_en();
    rf_n22_dig_init();
}

volatile u32 connection_cnt;
volatile u32 crc_ok;
volatile u32 crc_error;
volatile u32 first_timeout;





/**
 * @brief       This is main function
 * @param[in]   none
 * @return      none
 */
int main(void)
{
#if HCI_DFU_EN
    blc_ota_setFirmwareSizeAndBootAddress(DFU_NEW_FW_MAX_SIZE, DFU_NEW_FW_ADDR_BASE);
#endif

    /* this function must called before "sys_init()" when:
     * (1). For all IC: using 32K RC for power management,
       (2). For B91 only: even no power management */
#if  !DUAL_CORE_MODE_ENABLED
    blc_pm_select_internal_32k_crystal();
#endif
    blc_app_system_init();

    /* detect if MCU is wake_up from deep retention mode */
    int deepRetWakeUp = pm_is_MCU_deepRetentionWakeup(); //MCU deep retention wakeUp

    mcc_n22_service_init();

    user_init_normal();


    while (1)
    {
        main_loop();
    }

    return 0;
}
