/********************************************************************************************************
 * @file    app_d24g_rf_link.c
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
#include "tl_common.h"

#include "app_d24g_rf_link.h"
#include "app_d24g_stack.h"
#include "app_prj_config.h"
#include "app_public.h"
#include "../../common/stack/d24g_stack_common.h"

_attribute_aligned_(4) rf_packet_t  rf_tx_packet;
volatile unsigned int rf_status;
//volatile int rf_tx_length=2;
volatile unsigned int rf_rx_start_tick;
volatile signed char app_rf_channel=RF_FREQ;

unsigned char rx_packet[RX_FIFO_NUM * RX_FIFO_DEP] __attribute__((aligned(4)));

#define RF_HIGH_PER_MODE    1

static inline void rf_bb_dbg_setting(void)
{
    REG_ADDR8(0x140C00 + 0x100) |= (BIT(1) | BIT(2));
    gpio_function_dis(GPIO_PA0); // PA[0]: TX_EN
    gpio_function_dis(GPIO_PA1); // PA[1]: TX_ON
    gpio_function_dis(GPIO_PA2); // PA[2]: RX_EN
    // gpio_function_dis(GPIO_PB2); // PA[3]:RX_vld
    // gpio_function_dis(GPIO_PB3); // PA[4]: RX_DATA

    // gpio_function_dis(GPIO_PA4);                 // PA[4]: RX_HIT_SYNC
    // gpio_set_mux_function(GPIO_FC_PA4, DBG_BB0); // PA[4]: RX_HIT_SYNC

    gpio_set_mux_function(GPIO_FC_PA0, DBG_BB0); // PA[0]: TX_EN
    gpio_set_mux_function(GPIO_FC_PA1, DBG_BB0); // PA[1]: TX_ON
    gpio_set_mux_function(GPIO_FC_PA2, DBG_BB0); // PA[2]: RX_EN
    // gpio_set_mux_function(GPIO_FC_PB2, DBG_BB0); // PA[1]: TX_ON
    // gpio_set_mux_function(GPIO_FC_PB3, DBG_BB0); // PA[2]: RX_EN
}

#if  RF_FAST_SET_APP_ENABLE
extern _attribute_data_retention_sec_ rf_fast_settle_t *g_fast_settle_cal_val_ptr;

_attribute_data_retention_sec_ rf_fast_settle_t fs_cv_1m;

/**
 *  @brief      This function is used to get the calibration value of rf tx/rx fast settle
 *  @param[in]  tx_settle_us    After adjusting the timing sequence, the time required for tx to settle.
 *  @param[in]  rx_settle_us    After adjusting the timing sequence, the time required for rx to settle.
 *  @param[in]  fs_cv           Fast settle related calibration value storage variable address.
 *  @return     none
 */
void rf_fast_settle_get_val(rf_tx_fast_settle_time_e tx_settle_us, rf_rx_fast_settle_time_e rx_settle_us, rf_fast_settle_t *fs_cv)
{
    //tx
#if ((CHIP_TYPE == MCU_CORE_B91) || (CHIP_TYPE == MCU_CORE_B92))
    rf_set_tx_rx_off(); //STOP_RF_STATE_MACHINE;
    rf_clr_irq_status(FLD_RF_IRQ_ALL);
    rf_set_tx_settle_time(113);        //adjust TX settle time

    for (unsigned char f_chn = 0; f_chn <= 80; f_chn++) {
        rf_set_chn(f_chn);
        rf_set_txmode();
        delay_us(113); //Wait for calibration to stabilize
        rf_tx_fast_settle_get_cal_val(tx_settle_us, f_chn, fs_cv);

        rf_set_tx_rx_off(); //STOP_RF_STATE_MACHINE;
        rf_clr_irq_status(FLD_RF_IRQ_ALL);
    }
#elif ((CHIP_TYPE == MCU_CORE_TL721X) || (CHIP_TYPE == MCU_CORE_TL321X) || (CHIP_TYPE == MCU_CORE_TL322X))
    rf_set_tx_rx_off(); //STOP_RF_STATE_MACHINE;
    rf_clr_irq_status(FLD_RF_IRQ_ALL);
    rf_set_tx_settle_time(113);        //adjust TX settle time

    for (unsigned char f_chn = 4; f_chn <= 80; f_chn+=10) {
        rf_set_chn(f_chn);
        rf_set_txmode();
        delay_us(113); //Wait for calibration to stabilize
        rf_tx_fast_settle_get_cal_val(tx_settle_us, f_chn, fs_cv);

        rf_set_tx_rx_off(); //STOP_RF_STATE_MACHINE;
        rf_clr_irq_status(FLD_RF_IRQ_ALL);
    }
#endif

    //rx
#if ((CHIP_TYPE == MCU_CORE_B91) || (CHIP_TYPE == MCU_CORE_B92))
    rf_set_rx_settle_time(85); //adjust RX settle time
    rf_set_rxmode();
    delay_us(85); //Wait for the rx packetization action to complete
    rf_rx_fast_settle_get_cal_val(rx_settle_us, 0, fs_cv);
    rf_set_tx_rx_off(); //STOP_RF_STATE_MACHINE;
    rf_clr_irq_status(FLD_RF_IRQ_ALL);
#elif (CHIP_TYPE == MCU_CORE_TL721X)
    rf_set_rx_settle_time(85); //adjust RX settle time
    for (unsigned char f_chn = 4; f_chn <= 80; f_chn += 10) {
        rf_set_chn(f_chn);
        rf_set_rxmode();
        delay_us(85); //Wait for the rx packetization action to complete
        rf_rx_fast_settle_get_cal_val(rx_settle_us, f_chn, fs_cv);

        rf_set_tx_rx_off(); //STOP_RF_STATE_MACHINE;
        rf_clr_irq_status(FLD_RF_IRQ_ALL);
    }
#elif ((CHIP_TYPE == MCU_CORE_TL321X)||(CHIP_TYPE == MCU_CORE_TL322X))
    rf_set_rx_settle_time(93); //adjust RX settle time
    for (unsigned char f_chn = 4; f_chn <= 80; f_chn += 10) {
        rf_set_chn(f_chn);
        rf_set_rxmode();
        delay_us(93); //Wait for the rx packetization action to complete
        rf_rx_fast_settle_get_cal_val(rx_settle_us, f_chn, fs_cv);

        rf_set_tx_rx_off(); //STOP_RF_STATE_MACHINE;
        rf_clr_irq_status(FLD_RF_IRQ_ALL);
    }
#endif
}

/**
 *  @brief      This function is used to set the calibration value of rf tx/rx fast settle
 *  @param[in]  tx_settle_us    After adjusting the timing sequence, the time required for tx to settle.
 *  @param[in]  rx_settle_us    After adjusting the timing sequence, the time required for rx to settle.
 *  @param[in]  fs_cv           Fast settle related calibration value storage variable address.
 *  @return     none
 */
void rf_fast_settle_set_val(rf_tx_fast_settle_time_e tx_settle_us, rf_rx_fast_settle_time_e rx_settle_us, rf_fast_settle_t *fs_cv)
{
    g_fast_settle_cal_val_ptr = fs_cv;
#if ((CHIP_TYPE == MCU_CORE_B91) || (CHIP_TYPE == MCU_CORE_B92))
    rf_tx_fast_settle_set_cal_val(tx_settle_us, 0, fs_cv);
    rf_rx_fast_settle_set_cal_val(rx_settle_us, 0, fs_cv);
#elif (CHIP_TYPE == MCU_CORE_TL721X)
    for (unsigned char f_chn = 4; f_chn <= 80; f_chn += 10) {
        rf_tx_fast_settle_set_cal_val(tx_settle_us, f_chn, fs_cv);
        rf_rx_fast_settle_set_cal_val(rx_settle_us, f_chn, fs_cv);
    }
    rf_cali_linear_fit(fs_cv);
#elif (CHIP_TYPE == MCU_CORE_TL321X)
    for (unsigned char f_chn = 4; f_chn <= 80; f_chn += 10) {
        rf_tx_fast_settle_set_cal_val(tx_settle_us, f_chn, fs_cv);
        rf_rx_fast_settle_set_cal_val(rx_settle_us, f_chn, fs_cv);
    }
#elif (CHIP_TYPE == MCU_CORE_TL322X)
    for (unsigned char f_chn = 4; f_chn <= 80; f_chn += 10) {
        rf_tx_fast_settle_set_cal_val(tx_settle_us, f_chn, fs_cv);
        rf_rx_fast_settle_set_cal_val(rx_settle_us, f_chn, fs_cv);
    }
    rf_cali_linear_fit(fs_cv);
#endif
}

void pp_rf_fast_settle_init(void)
{
    rf_fast_settle_get_val(TX_SETTLE_TIME_15US, RX_SETTLE_TIME_15US, &fs_cv_1m);
    rf_fast_settle_set_val(TX_SETTLE_TIME_15US, RX_SETTLE_TIME_15US, &fs_cv_1m);

     rf_fast_settle_config(TX_SETTLE_TIME_15US, RX_SETTLE_TIME_15US);

    rf_tx_fast_settle_en();
    rf_rx_fast_settle_en();
    rf_set_tx_settle_time(12);
    rf_set_rx_settle_time(12);
}
#endif
volatile signed char last_chn=0x7f;
_attribute_ram_code_sec_ void pp_rf_set_chn(signed char chn)
{
    signed char my_chn;

//  if (chn == 30) {
//       DBG_GPIO_SET_LEVEL(APP_IO_EN,GPIO_PG4, 1);
//  } else if (chn == 12) {
//       DBG_GPIO_SET_LEVEL(APP_IO_EN,GPIO_PG4, 0);
//  } else {
//       DBG_GPIO_TOGGLE(APP_IO_EN,GPIO_PG4);
//       DBG_GPIO_TOGGLE(APP_IO_EN,GPIO_PG4);
//       DBG_GPIO_TOGGLE(APP_IO_EN,GPIO_PG4);
//       DBG_GPIO_TOGGLE(APP_IO_EN,GPIO_PG4);
//  }
    if(last_chn!=chn)
    {
        last_chn=chn;
    }
    else
    {
        return;
    }
    #if 1//(HOP_FREQ_FUN_ENABLE)
        my_chn=chn;
    #else
        (void) chn;
        my_chn=RF_FREQ;
    #endif
    
    #if RF_IS_HAS_PN
        rf_set_ble_chn(my_chn);
    #else
        rf_set_chn(my_chn);
    #endif
        
}

_attribute_ram_code_sec_ void pp_rf_auto_mode_reset(void)
{
    rf_dma_chn_dis(RF_RX_DMA);
    rf_set_tx_rx_off_auto_mode();
    rf_dma_reset();
    rf_clr_dig_logic_state();
    rf_set_tx_dma(2, 128);
    rf_set_rx_dma(rx_packet, RX_FIFO_NUM - 1, RX_FIFO_DEP);
    // rf_set_rx_maxlen(32);
    delay_us(2);
}


void pp_rf_init(void)
{
     #if RF_GPIO_DEBUG_ENABLE

      gpio_function_en(RF_MTX_PIN);
     gpio_output_en(RF_MTX_PIN);
     gpio_input_dis(RF_MTX_PIN);


     gpio_function_en(RF_MRX_PIN);
     gpio_output_en(RF_MRX_PIN);
     gpio_input_dis(RF_MRX_PIN);

      gpio_function_en(RF_MRX_CRC_ERROR_PIN);
     gpio_output_en(RF_MRX_CRC_ERROR_PIN);
     gpio_input_dis(RF_MRX_CRC_ERROR_PIN);


      gpio_function_en(RF_MRX_TIMEOUT_PIN);
     gpio_output_en(RF_MRX_TIMEOUT_PIN);
     gpio_input_dis(RF_MRX_TIMEOUT_PIN);

     
      gpio_function_en(RF_M_CHANNEL_CHANGE_PIN);
     gpio_output_en(RF_M_CHANNEL_CHANGE_PIN);
     gpio_input_dis(RF_M_CHANNEL_CHANGE_PIN);
 #else
    #if (((HW_BOARD_TYPE == HW_EVK_KEYBOARD) || (HW_BOARD_TYPE == HW_ALG_KEYBOARD)) && DBG_2P4G_IO_EN)
    rf_bb_dbg_setting();
    #endif
#endif

    rf_n22_dig_init();
    rf_mode_init();
#if RF_IS_HAS_PN
    rf_set_ble_6M_mode();
#else
    rf_set_ble_6M_NO_PN_mode();
#endif

     // Set modem rate to 48 MHz for 4/6 Mbps operation
    rf_modem_rate_mode(RF_48M_MODEM_RATE);//When using 4 Mbps or 6 Mbps data rates, the 48 MHz modem rate must be applied.
#if (RF_HIGH_PER_MODE)
    rf_modem_hp_path(1);
#else
    rf_modem_hp_path(0);
#endif
    rf_set_tx_dma(2, 128);
    rf_set_rx_dma(rx_packet, RX_FIFO_NUM - 1, RX_FIFO_DEP);


#if FAST_SETTLE
    if (!pm_is_MCU_deepRetentionWakeup())
    {
        ext_fast_settle_init(0x81, 0x81, EXT_FAST_SETTLE_PHY_6M);
    }
    ext_fast_settle_set_val(EXT_FAST_SETTLE_PHY_6M);
    rf_set_tx_settle_time(12);
    rf_set_rx_settle_time(12);
#endif

    pp_rf_set_chn(RF_FREQ);
    rf_set_power_level(RF_POWER);

#if 1
    write_reg8(0x170002, 0xC5); //preamble len, trailer must be 7B
    rf_set_access_code_len(5);
    write_reg8(0x17000c,0x55);//access code
    write_reg8(0x17051a,36);//sync thr driver

    write_reg16(0x170206, 14);// rx wait

    write_reg8(0x17022f,0x10); //reduce 0.5us rx path delay

    write_reg8(0x170625,read_reg8(0x170625)&0x3f);
    write_reg8(0x170624,(read_reg8(0x170624)&0xfc)|0x02);//125ns  tx pa ramp time
    write_reg8(0x170216,(read_reg8(0x170216)&0x0f)|0x20);//tx en delay
    
    //When the received packet length is greater than the maximum packet length, receive the minimum number of packets.
    reg_rf_rxtcrcpkt &= (~FLD_RF_LEN_SET_O);
#endif

    app_rf_set_rx_wait(1); // rx wait

#if defined(MCU_CORE_TL322X_N22)
    clic_init();
    clic_interrupt_enable(IRQ_ZB_RT);
#else
    plic_interrupt_enable(IRQ_ZB_RT);
#endif
    //rf_set_irq_mask(FLD_RF_IRQ_RX);
    rf_set_irq_mask(FLD_RF_IRQ_TX|FLD_RF_IRQ_RX|FLD_RF_IRQ_RX_TIMEOUT|FLD_RF_IRQ_FIRST_TIMEOUT);
#if defined(MCU_CORE_TL322X_N22)
    clic_set_priority(IRQ_ZB_RT, IRQ_PRI_LEV2);
#else
    plic_set_priority(IRQ_ZB_RT, IRQ_PRI_LEV2);
#endif
}

/**
*  @brief    rf_set_tx_len
*  @param[in]  len length  

*  @return  none
*/

_attribute_ram_code_sec_ void pp_rf_set_tx_len(unsigned char len)
{
    #if ID_INCLUDE_PACKET_ENABLE
        rf_tx_packet.rf_len=len+2;
    #else
        rf_tx_packet.rf_len=len;
    #endif
    rf_tx_packet.dma_len=rf_tx_packet_dma_len(rf_tx_packet.rf_len+2);
}

/**
*  @brief    rf start tx
*  @param[in]  none  

*  @return  none
*/

_attribute_ram_code_sec_ void pp_rf_start_stx(void )
{
    rf_status= TX_START_RF_STATUS;

    RF_TX_GPIO_DEBUG_SET(1);

    rf_start_stx ((unsigned char *)&rf_tx_packet.dma_len, rf_stimer_get_tick());
}

_attribute_ram_code_sec_ void pp_rf_start_stx2rx(void )
{
#if 0//(RF_IS_STX_FIRST==1)
    static signed char last_chn=-86;
    if(last_chn!=app_rf_channel)
    {
        last_chn=app_rf_channel;
        
        pp_rf_set_chn(last_chn);
        delay_us(2);
    }
#endif
    rf_status= TX_START_RF_STATUS;
    
    
    
    RF_TX_GPIO_DEBUG_SET(1);
    
    rf_start_stx2rx ((unsigned char *)&rf_tx_packet.dma_len, rf_stimer_get_tick());
    
}


_attribute_ram_code_sec_ void app_rf_start_stx2rx(unsigned int tick, unsigned int timeout_us)
{
    reg_rf_rx_timeout = timeout_us;
    rf_start_stx2rx ((unsigned char *)&rf_tx_packet.dma_len, tick);
}

_attribute_ram_code_sec_ void pp_rf_start_srx(unsigned int tick,unsigned int timeout_us)
{
#if (RF_IS_STX_FIRST==0)

    static signed char last_chn=-86;
    if(last_chn!=app_rf_channel)
    {
        last_chn=app_rf_channel;
        pp_rf_set_chn(last_chn);
        delay_us(2);
    }
#endif
    rf_status = RX_START_RF_STATUS;
    reg_rf_ll_rx_fst_timeout= timeout_us;                   // first timeout.
    reg_rf_ll_cmd_schedule = tick;
    reg_rf_ll_ctrl3 |= FLD_RF_R_CMD_SCHEDULE_EN;    // Enable cmd_schedule mode.
    reg_rf_ll_cmd = 0x86;
    RF_RX_GPIO_DEBUG_SET(1);
    rf_rx_start_tick=stimer_get_tick();
}

_attribute_ram_code_sec_ void p24g_set_rx_wait(unsigned char len)
{
    /* 
       6 bytes payload ---> rx_wait = 10 us
       Calculate using integer arithmetic to avoid floating-point operations.
       To avoid floating-point operations, integer arithmetic is used. The formula
       idle_time = 7 + (6 - len) * 1.333...` is scaled by 3.
       Original formula: (30 * 3) + (6 - len) * 4 = 39 + (6 - len) * 4
       Adding 1 before division by 3 to implement rounding to the nearest integer
    */
    uint8_t rx_wait = (33 + (17 - len) * 4 + 1) / 3;

    app_rf_set_rx_wait(rx_wait - 3);
}

_attribute_ram_code_sec_  void app_rf_stx2rx_set_tx_len(unsigned char len)
{
    app_rf_set_rx_wait((14 - len) * 4 / 3); //max length 12, rf 6M
}

