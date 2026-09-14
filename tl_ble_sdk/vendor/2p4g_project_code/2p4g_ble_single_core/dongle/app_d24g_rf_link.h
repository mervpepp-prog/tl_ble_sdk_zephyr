/********************************************************************************************************
 * @file    app_d24g_rf_link.h
 *
 * @brief   This is the header file for Telink RISC-V MCU
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
#ifndef __APP_D24G_RF_LINK_H__
#define __APP_D24G_RF_LINK_H__

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

    #define print_app_rf(...)
#ifndef print_app_rf
    #define print_app_rf printf
#endif


#define RF_FAST_SET_APP_ENABLE   0
#define FAST_SETTLE         1
#define RF_IS_STX_FIRST   0
#define RF_IS_HAS_PN    1


//7     10  17  25  30  35
//18    28  40  56  66  76
#define RF_FREQ       30

#define rf_stimer_get_tick()     stimer_get_tick()
#define RF_SYSTEM_TIMER_TICK_1US SYSTEM_TIMER_TICK_1US
#define RF_SYSTEM_TIMER_TICK_1MS SYSTEM_TIMER_TICK_1MS

#define RX_FIFO_NUM    1
#define RX_FIFO_DEP   128

typedef enum 
{
    IDLE_RF_STATUS,
    TX_START_RF_STATUS,
    TX_END_RF_STATUS,
    RX_START_RF_STATUS,
    RX_END_OK_RF_STATUS,
    RX_END_CRC_ERROR_RF_STATUS,
    RX_END_TIMEOUT_RF_STATUS,
    RX_END_FIST_TIMEOUT_RF_STATUS,
}APP_RF_STATUS_E;
extern volatile unsigned int rf_status;

#define ID_INCLUDE_PACKET_ENABLE  0

extern unsigned char rx_packet[];
extern volatile int rf_tx_length;
extern volatile unsigned int rf_rx_start_tick;

void pp_rf_hw_init(uint8_t chn);
void pp_rf_init(void);
void pp_rf_start_srx(unsigned int tick,unsigned int timeout_us);
void pp_rf_start_stx(void );
void pp_rf_start_stx2rx(void *addr, unsigned int tick, unsigned int timeout_us);
void pp_rf_start_srx2tx(void *addr, unsigned int tick, unsigned int timeout_us);

    /**
    *  @brief    rf_set_tx_len
    *  @param[in]  len length  
    
    *  @return  none
    */
    
 void pp_rf_set_tx_len(unsigned char len);

 void pp_rf_set_chn(signed char chn);
 signed char pp_rf_get_last_chn(void);
 void pp_rf_auto_mode_reset(void);

#define RF_GPIO_DEBUG_ENABLE  0

#if RF_GPIO_DEBUG_ENABLE

    
    #define RF_MTX_PIN                  GPIO_PD4
    #define RF_MRX_PIN                  GPIO_PD5
    #define RF_MRX_TIMEOUT_PIN          GPIO_PD6
    #define RF_MRX_CRC_ERROR_PIN        GPIO_PG6
    #define RF_M_CHANNEL_CHANGE_PIN     GPIO_PD7
    #define MOUSE_IN_TIME_DEBUG_PIN     GPIO_PG7
    


    #define RF_TX_GPIO_DEBUG_SET(x)         DBG_GPIO_SET_LEVEL(APP_IO_EN, RF_MTX_PIN,x)

    #define RF_RX_GPIO_DEBUG_SET(x)     DBG_GPIO_SET_LEVEL(APP_IO_EN, RF_MRX_PIN,x)
    #define RF_RX_GPIO_DEBUG_TOGGLE     DBG_GPIO_TOGGLE(APP_IO_EN, RF_MRX_PIN)


    //#define RF_RX_CRC_GPIO_DEBUG_SET(x)           DBG_GPIO_SET_LEVEL(APP_IO_EN, RF_MRX_CRC_ERROR_PIN,x)
    #define RF_RX_CRC_GPIO_DEBUG_TOGGLE         DBG_GPIO_TOGGLE(APP_IO_EN, RF_MRX_CRC_ERROR_PIN)


    //#define RF_RX_TIMEOUT_GPIO_DEBUG_SET(x)       DBG_GPIO_SET_LEVEL(APP_IO_EN, RF_MRX_TIMEOUT_PIN,x)
    #define RF_RX_TIMEOUT_GPIO_DEBUG_TOGGLE     DBG_GPIO_TOGGLE(APP_IO_EN, RF_MRX_TIMEOUT_PIN)

    #define RF_CHANNEL_CHANGE_GPIO_DEBUG_SET(x)     DBG_GPIO_SET_LEVEL(APP_IO_EN, RF_M_CHANNEL_CHANGE_PIN,x)
    #define RF_CHANNEL_CHANGE_GPIO_DEBUG_TOGGLE DBG_GPIO_TOGGLE(APP_IO_EN, RF_M_CHANNEL_CHANGE_PIN)



    #define MOUSE_DATA_IN_TIME_DEBUG_TOGGLE     DBG_GPIO_TOGGLE(APP_IO_EN, MOUSE_IN_TIME_DEBUG_PIN)
    #define MOUSE_DATA_IN_TIME_DEBUG_SET(x)     DBG_GPIO_TOGGLE(APP_IO_EN, MOUSE_IN_TIME_DEBUG_PIN,x)

#else

    #define RF_TX_GPIO_DEBUG_SET(x) 

    #define RF_RX_GPIO_DEBUG_SET(x)     
    #define RF_RX_GPIO_DEBUG_TOGGLE     


    #define RF_RX_CRC_GPIO_DEBUG_SET(x)         
    #define RF_RX_CRC_GPIO_DEBUG_TOGGLE         


    //#define RF_RX_TIMEOUT_GPIO_DEBUG_SET(x)       
    #define RF_RX_TIMEOUT_GPIO_DEBUG_TOGGLE 

    #define RF_CHANNEL_CHANGE_GPIO_DEBUG_TOGGLE 

    #define MOUSE_DATA_IN_TIME_DEBUG_TOGGLE     
    #define MOUSE_DATA_IN_TIME_DEBUG_SET(x) 
    
#endif

#define RF_POWER RF_VANT_POWER_P6p40
// #define RF_POWER RF_POWER_P10p00dBm

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif
