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
#include "app_public.h"
#include "../common/stack/d24g_stack_common.h"

unsigned char rx_packet[RX_FIFO_NUM * RX_FIFO_DEP] __attribute__((aligned(4)));

rf_packet_t rf_tx_packet       __attribute__((aligned(4)));
volatile unsigned int rf_status;
//volatile int rf_tx_length=2;
volatile unsigned int rf_rx_start_tick;

#define RF_HIGH_PER_MODE    1
volatile signed char app_rf_channel=RF_FREQ;


static inline void rf_bb_dbg_setting(void)
{
    REG_ADDR8(0x140C00 + 0x100) |= (BIT(1) | BIT(2));
    gpio_function_dis(GPIO_PA0);                 // PA[0]: TX_EN
    gpio_function_dis(GPIO_PA1);                 // PA[1]: TX_ON
    gpio_function_dis(GPIO_PA2);                 // PA[2]: RX_EN
    //gpio_function_dis(GPIO_PA3);                 // PA[3]: bb clk
    //gpio_function_dis(GPIO_PA4);                 // PA[4]: RX_HIT_SYNC

    gpio_function_dis(GPIO_PB0);                 // PB[0]: SCLK
    gpio_function_dis(GPIO_PB1);                 // PB[1]: TX_DATA
    gpio_function_dis(GPIO_PB2);                 // PB[2]: RX VLD
    gpio_function_dis(GPIO_PB3);                 // PB[3]: RX_DATA
    //gpio_function_dis(GPIO_PD0);                 // PD[0]: DMA CYC


    gpio_set_mux_function(GPIO_FC_PA0, DBG_BB0); // PA[0]: TX_EN
    gpio_set_mux_function(GPIO_FC_PA1, DBG_BB0); // PA[1]: TX_ON
    gpio_set_mux_function(GPIO_FC_PA2, DBG_BB0); // PA[2]: RX_EN
    //gpio_set_mux_function(GPIO_FC_PA3, DBG_BB0); // PA[3]: bb clk
    //gpio_set_mux_function(GPIO_FC_PA4, DBG_BB0); // PA[4]: RX_HIT_SYNC

    gpio_set_mux_function(GPIO_FC_PB0, DBG_BB0); // PB[0]: SCLK
    gpio_set_mux_function(GPIO_FC_PB1, DBG_BB0); // PB[1]: TX_DATA
    gpio_set_mux_function(GPIO_FC_PB2, DBG_BB0); // PB[2]: RX_vld
    gpio_set_mux_function(GPIO_FC_PB3, DBG_BB0); // PB[3]: RX_DATA
   // gpio_set_mux_function(GPIO_FC_PD0, DBG_BB0); // PD[0]: DMACYC
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
#elif ((CHIP_TYPE == MCU_CORE_TL321X) || (CHIP_TYPE == MCU_CORE_TL322X))
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
_attribute_ram_code_sec_ signed char pp_rf_get_last_chn(void)
{
    
    return  last_chn;
}

_attribute_ram_code_sec_ void pp_rf_set_chn(signed char chn)
{
    signed char my_chn;

    if (chn == 30) {
         DBG_GPIO_SET_LEVEL(APP_IO_EN,GPIO_PB4, 1);
    } else if (chn == 12) {
         DBG_GPIO_SET_LEVEL(APP_IO_EN,GPIO_PB4, 0);
    } else {
         DBG_GPIO_TOGGLE(APP_IO_EN, GPIO_PB4);
         DBG_GPIO_TOGGLE(APP_IO_EN, GPIO_PB4);
         DBG_GPIO_TOGGLE(APP_IO_EN, GPIO_PB4);
         DBG_GPIO_TOGGLE(APP_IO_EN, GPIO_PB4);
    }
    chn = 30;
    if(last_chn!=chn)
    {
        last_chn=chn;
    }
    else
    {
        return;
    }
    #if 1// (HOP_FREQ_FUN_ENABLE)
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
    rf_set_tx_rx_off_auto_mode();
    rf_dma_chn_dis(RF_RX_DMA);
    rf_dma_reset();
    rf_clr_dig_logic_state();
    rf_set_tx_dma(2, 128);
    rf_set_rx_dma(rx_packet, RX_FIFO_NUM - 1, RX_FIFO_DEP);
    rf_set_rx_maxlen(32);
    delay_us(2);
}

_attribute_ram_code_sec_ void pp_rf_hw_init(uint8_t chn)
{
    pp_rf_set_chn(chn);
    rf_set_power_level(RF_POWER);

#if 1
     write_reg8(0x170002, 0xC5); //preamble len, trailer must be 7B
    rf_set_access_code_len(5);
    write_reg8(0x17000c,0x55);//access code
    write_reg8(0x17051a,36);//sync thr

    write_reg16(0x170206,0);//rx wait
    write_reg16(0x17020e,0);//tx wait

    write_reg8(0x17022f,0x10);//reduce 0.5us rx path delay

    write_reg8(0x17021e,0x80);//

    write_reg8(0x170625,read_reg8(0x170625)&0x3f);
    write_reg8(0x170624,(read_reg8(0x170624)&0xfc)|0x02);//125ns  tx pa ramp time

    write_reg8(0x170216,(read_reg8(0x170216)&0x0f)|0x20);//tx en delay
//  write_reg8(0x170216,0x2d);//tx en delay

    //When the received packet length is greater than the maximum packet length, receive the minimum number of packets.
    reg_rf_rxtcrcpkt &= (~FLD_RF_LEN_SET_O);
#endif
}

void pp_rf_init(void)
{
#if (HW_BOARD_TYPE == HW_EVK_DONGLE)
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
    #endif
    #if DBG_2P4G_IO_EN
    rf_bb_dbg_setting();
    #endif
#endif

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
    
    pp_rf_auto_mode_reset();


#if FAST_SETTLE
    ext_fast_settle_init(0x81, 0x81, EXT_FAST_SETTLE_PHY_6M);
    ext_fast_settle_set_val(EXT_FAST_SETTLE_PHY_6M);
    rf_set_tx_settle_time(12);
    rf_set_rx_settle_time(12);
#endif

    pp_rf_hw_init(app_rf_channel);
#if defined(MCU_CORE_TL322X_N22)
       clic_init();
       clic_interrupt_enable(IRQ_ZB_RT);
 #else
       plic_interrupt_enable(IRQ_ZB_RT);
#endif
       //rf_set_irq_mask(FLD_RF_IRQ_RX);
      rf_set_irq_mask(FLD_RF_IRQ_TX|FLD_RF_IRQ_RX|FLD_RF_IRQ_RX_TIMEOUT|FLD_RF_IRQ_FIRST_TIMEOUT);
      // rf_start_srx(rf_stimer_get_tick());
    plic_set_priority(IRQ_ZB_RT, IRQ_PRI_LEV2);

     
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
    // cnt_tx++;
    rf_start_stx ((unsigned char *)&rf_tx_packet.dma_len, rf_stimer_get_tick());
    
}

_attribute_ram_code_sec_ void pp_rf_start_srx(unsigned int tick,unsigned int timeout_us)
{
    rf_status = RX_START_RF_STATUS;
    reg_rf_ll_rx_fst_timeout= timeout_us;                   // first timeout.
    reg_rf_ll_cmd_schedule = tick;
    reg_rf_ll_ctrl3 |= FLD_RF_R_CMD_SCHEDULE_EN;    // Enable cmd_schedule mode.
    reg_rf_ll_cmd = 0x86;
    RF_RX_GPIO_DEBUG_SET(1);
    // rf_rx_start_tick=stimer_get_tick();
}

_attribute_ram_code_sec_ void pp_rf_start_stx2rx(void *addr, unsigned int tick, unsigned int timeout_us)
{
    reg_rf_rx_timeout = timeout_us;
    rf_dma_set_src_address(RF_TX_DMA, (unsigned int)(addr));
    reg_rf_ll_cmd_schedule = tick;
    reg_rf_ll_ctrl3 |= FLD_RF_R_CMD_SCHEDULE_EN; // Enable cmd_schedule mode.
    reg_rf_ll_cmd = 0x87;                        // single tx2rx.
}

_attribute_ram_code_sec_ void pp_rf_start_srx2tx(void *addr, unsigned int tick, unsigned int timeout_us)
{
    reg_rf_ll_rx_fst_timeout = timeout_us;       // first timeout
    reg_rf_ll_cmd_schedule   = tick;
    reg_rf_ll_ctrl3 |= FLD_RF_R_CMD_SCHEDULE_EN; // Enable cmd_schedule mode.
    rf_dma_set_src_address(RF_TX_DMA, (unsigned int)(addr));
    reg_rf_ll_rest_pid = 0x3f;
    reg_rf_ll_cmd      = 0x88;                   // single rx2tx
}


