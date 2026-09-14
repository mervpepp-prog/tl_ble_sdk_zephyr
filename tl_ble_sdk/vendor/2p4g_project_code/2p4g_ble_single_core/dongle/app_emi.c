/********************************************************************************************************
 * @file    app_emi.c
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

#include "app_prj_config.h"
#include "lib/include/emi_internal.h"

#if (EMI_TEST_FUN_ENABLE)
u8 emi_mode = 0xff;
u8 last_emi_mode =0xff;
u8 emi_channel = 0;
u8 emi_power =RF_POWER_P10p00dBm;
u8 emi_data_type = 0;
u8 emi_pkt_num =0;
int emi_rf_mode =RF_MODE_BLE_1M_NO_PN;
u8 emi_hop =0;
u8 emi_change_flag =0;

typedef enum 
{
    CARRIER_ONLY_MODE=0,
    CARRIER_DATA_MODE=1, // TX_burst
    TX_MODE=2, // TX burst_hop
    RX_MODE=3,
}EMI_MODE;


u8 emi_flg = 0;

#define MAX_RF_CHANNEL         40
#define EMI_TX_PAYLOAD_LEN       37   /**< Length of transmitted payload. */
#define EMI_PKT_TONE_DLY         40   /**< Duration of tone in packet tone mode,Unit in microseconds. */
#define EMI_PKT_DUTY_CYCLE       50   /**< TX Packet duty cycle setting.Default: 50 (50% duty cycle) */

/**
 * @brief   Initialize the frequency hopping channels for RF communication.
 */
const unsigned char rf_chn_hopping[MAX_RF_CHANNEL] = {
    48,  4,  66, 62, 44, 12, 38, 16, 26, 20,
    22, 68, 18, 28, 42, 32, 34, 36, 14, 40,
    30, 54, 46,  2, 50, 52, 80, 56, 78, 74,
     8, 64,  6, 24, 70, 72, 60, 76, 58, 10,
};

void emi_carrier_only(rf_mode_e rf_mode, unsigned char pwr, signed char rf_chn)
{
    (void)rf_mode;

    rf_emi_tx_single_tone(pwr, rf_chn);
    
    while (1)
    {

    }
    
    rf_emi_stop();
    rf_set_tx_rx_off();
    rf_dma_reset();
    rf_clr_dig_logic_state();
}
void emi_carrier_data(rf_mode_e rf_mode, unsigned char pwr, signed char rf_chn, u8 data_type)
{
    rf_emi_tx_continue_update_data(rf_mode, pwr, rf_chn, data_type);
    while (1)
    {
        rf_continue_mode_run();
    }
    
    rf_emi_stop();
    rf_dma_reset();
    rf_clr_dig_logic_state();

}
void emi_tx_mode(rf_mode_e rf_mode, unsigned char pwr, signed char rf_chn,u8 data_type)
{
    rf_emi_tx_burst_setup(rf_mode, pwr, rf_chn, data_type);
    while (1)
    {
        rf_emi_tx_burst_loop(rf_mode, data_type);
    }
    
    rf_emi_stop();
}
void emirx(rf_mode_e rf_mode, unsigned char pwr, signed char rf_chn)
{
    (void)pwr;
    rf_emi_rx_setup(rf_mode, rf_chn);

    while (1)
    {
        rf_emi_rx_loop();
    }
    
    rf_emi_stop();
}


/**
 * @brief   emi_init
 * @param   none
 * @return  none
 */
void emi_init(void)
{
    rf_mode_init();
    rf_access_code_comm(EMI_ACCESS_CODE); 
}

void emi_rf_mode_handle(u8 data)
{
    if(data == 0)
    {
        emi_rf_mode = RF_MODE_BLE_1M_NO_PN;
    }
    else if(data ==1)
    {
        emi_rf_mode = RF_MODE_BLE_2M_NO_PN;
    }
    else if(data ==2)
    {
        emi_rf_mode = RF_MODE_PRIVATE_1M;
    }
    else if(data ==3)
    {
        emi_rf_mode = RF_MODE_PRIVATE_2M;
    }
    else if(data ==4)
    {
        emi_rf_mode = RF_MODE_BLE_6M_NO_PN;
    }
    else if(data ==5)
    {
        emi_rf_mode = RF_MODE_PRIVATE_6M;
    }
    else if(data ==6)
    {
        emi_rf_mode = RF_MODE_LR_S8_125K;
    }
    else if(data ==7)
    {
        emi_rf_mode = RF_MODE_ZIGBEE_250K;
    }
    else if(data ==8)
    {
        emi_rf_mode = RF_MODE_LR_S2_500K;
    }
}
void emi_power_handle(u8 data)
{
    if(data>128)//set minus db
    {
        if(data<150)
        {
         emi_power = RF_POWER_P0p00dBm -(150-data);
        }
    }
    else if(data ==0)
    {
        emi_power = RF_POWER_P0p00dBm;
    }
    else if(data ==1)
    {
        emi_power = RF_POWER_P1p00dBm;
    }
    else if(data ==2)
    {
        emi_power = RF_POWER_P2p00dBm;
    }
    else if(data ==3)
    {
        emi_power = RF_POWER_P3p00dBm;
    }
    else if(data ==4)
    {
        emi_power = RF_POWER_P4p00dBm;
    }
    else if(data ==5)
    {
        emi_power = RF_POWER_P5p00dBm;
    }
    else if(data ==6)
    {
        emi_power = RF_POWER_P6p00dBm;
    }
    else if(data ==7)
    {
        emi_power = RF_POWER_P7p00dBm;
    }
    else if(data ==8)
    {
        emi_power = RF_POWER_P8p00dBm;
    }
    else if(data ==9)
    {
        emi_power = RF_POWER_P9p00dBm;
    }
    else if(data ==10)
    {
        emi_power = RF_POWER_P10p00dBm;
    }
}
/**
 * @brief   EMI test loop
 * @param   none
 * @return  none
 */
void rf_bb_dbg_setting(void)
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

volatile unsigned char emi_init_flag = 0;
void usb_host_cmd_proc(void)
{
    extern u8 host_cmd[8];
    //u8 tbl_chn[4] = {05, 46, 80, 80};

    u8 dongle_host_cmd1;

    if ((host_cmd[0] == 0x5) && (host_cmd[1] == 0x3))
    {
        if (!(analog_read_reg8(PM_ANA_REG_POWER_ON_CLR_BUF1) & 0x4)) {
            analog_write_reg8(PM_ANA_REG_POWER_ON_CLR_BUF1, analog_read_reg8(PM_ANA_REG_POWER_ON_CLR_BUF1) | 0x4);
            start_reboot();
        }

        
        emi_flg = 1;
        host_cmd[0] = 0; //clear CMD


        dongle_host_cmd1 = host_cmd[1];

        if ((dongle_host_cmd1 > 0) && (dongle_host_cmd1 < 13)) //1-12:
        {

            pp_timer0_stop();
            pp_timer1_stop();
            wd_stop(); //stop normal watch dog
            wd_32k_stop(); //stop 32K watch dog
            if(last_emi_mode == CARRIER_ONLY_MODE)
            {
                rf_emi_stop();
                rf_dma_reset();
                rf_clr_dig_logic_state();
            }
            else if(last_emi_mode == TX_MODE || last_emi_mode == CARRIER_DATA_MODE)
            {
                rf_set_tx_rx_off_auto_mode();
                rf_emi_stop();
                rf_dma_reset();
                rf_clr_dig_logic_state();
            }
            else if(last_emi_mode == RX_MODE)
            {
                rf_emi_stop();
            }


            if (!emi_init_flag) {
                emi_init();
                emi_init_flag = 1;
            }


            emi_channel = host_cmd[4]; //get channel
            emi_mode = host_cmd[2] % 4; //get mode
            emi_data_type = host_cmd[6];
            emi_hop = host_cmd[7];
            if(host_cmd[3]<=8)
            {
              emi_rf_mode_handle(host_cmd[3]);
            }
            emi_power_handle(host_cmd[5]);
            tlkapi_printf(APP_LOG_EN,"---mode=%d,channel=%d, rf mode=%d,emi_power =%d,data_tye is %d\n",emi_mode,emi_channel,emi_rf_mode,emi_power,emi_data_type);
            if(last_emi_mode!=emi_mode)
            {
                last_emi_mode = emi_mode;
            }
            if (emi_mode == CARRIER_ONLY_MODE) //carri
            {
                rf_emi_tx_single_tone(emi_power, emi_channel);

            }
            else if (emi_mode == CARRIER_DATA_MODE) //tx burst
            {
                //data type: 2 : tx55  , 1: tx0f, 0:rand data
                emi_cfg_param.rf_mode = emi_rf_mode;
                emi_cfg_param.rf_chn = emi_channel;
                emi_cfg_param.power_level = emi_power;
                emi_cfg_param.pkt_type = emi_data_type;
                rf_emi_tx_burst_setup_v1();
            }
            else if (emi_mode == TX_MODE) //tx burst_hop
            {
                emi_cfg_param.rf_mode = emi_rf_mode;
                emi_cfg_param.rf_chn = rf_chn_hopping[0];
                emi_cfg_param.power_level = emi_power;
                emi_cfg_param.pkt_type = emi_data_type;
                rf_emi_tx_burst_setup_v1();
            }
            else if (emi_mode == RX_MODE) //rx
            {
                emi_cfg_param.rf_mode = emi_rf_mode;
                emi_cfg_param.rf_chn = emi_channel;
                rf_emi_rx_setup_v1();
            }
        }
    }

    if (emi_flg)
    {
        if (emi_mode == CARRIER_DATA_MODE) //cd
        {
            rf_emi_tx_burst_loop_v1();
        }
        else if (emi_mode == TX_MODE) //tx
        {
            unsigned int  chnidx = 1;
            unsigned int  pulse_time = 0;

            u32  cd_tick = clock_time();
            while(!host_cmd[0]) {
                pulse_time = (EMI_TX_PAYLOAD_LEN) | (EMI_PKT_TONE_DLY << 8); //Multiplexed as pulse time

                while (!clock_time_exceed(cd_tick, pulse_time)) {
                    rf_emi_tx_burst_loop_v1(); // run 10ms
                }
                rf_set_tx_rx_off_auto_mode();
                delay_us(((pulse_time / EMI_PKT_DUTY_CYCLE) * 100) - pulse_time); // delay 10ms

                cd_tick = clock_time();
                emi_cfg_param.rf_chn = rf_chn_hopping[chnidx - 1];
                rf_emi_tx_burst_setup_v1();
                (chnidx >= MAX_RF_CHANNEL) ? (chnidx = 1) : (chnidx++);
            }
        }
        else if (emi_mode == RX_MODE) //rx
        {
            rf_emi_rx_loop_v1();

        }

    }
}
#endif


