/********************************************************************************************************
 * @file    bqb.h
 *
 * @brief   This is the header file for Telink RISC-V MCU
 *
 * @author  Driver Group
 * @date    2019
 *
 * @par     Copyright (c) 2019, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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
#ifndef BQB_H_
#define BQB_H_
#include "tl_common.h"
#include "rf_certification.h"
/**
 *  @brief  command type for BQB Test
 */
typedef enum
{
    CMD_SETUP   = 0,
    CMD_RX_TEST = 1,
    CMD_TX_TEST = 2,
    CMD_END     = 3,
} Test_Command_e;

/**
 *  @brief  command status for BQB Test
 */
typedef enum
{
    SETUP_STATE = 0x10,
    RX_STATE,
    TX_STATE,
    END_STATE
} Test_Status_e;

typedef enum
{
    UART_NUM0 = UART0,
    UART_NUM1 = UART1,
    UART_NONE,
} uart_num_redef_e;

#define ACCESS_CODE              0x29417671
#define BQB_UART_BAUD            115200 //(9600)//(57600)//(1000000)
#define SWITCH_POWER_MODE        0x00   /**< Power mode of the chip.(LDO_LDO:0x00;DCDC_LDO:0x01;DCDC_DCDC:0x03)*/
#define SWITCH_INTERNAL_CAP      0      /**< Capacitor selection: 0 for internal cap, 1 for external cap. */
#define SWITCH_CALI_POSITION     0      /**< Calibration position: 0 for FLASH, 1 for OTP. */
#define SWITCH_GPIO_VOLTAGE      0      /**< The configuration of GPIO voltage. (0:GPIO voltage 3.3V 1:GPIO voltage 1.8V)*/
#define PA_FUNCTION_ENABLE       0      /**< PA enable flag (0: disabled,1: enabled). */
#define PA_BYPASS_ENABLE         0      /**< PA bypass enable flag (0: disabled,1: enabled).*/
#define SWIRE_THROUGH_USB_ENABLE 0      /**< Enable Swire through USB(0: disabled,1: enabled) */
#define POWER_SLICE_EN           0      /**< Power slice enable flag (0: disabled,1: enabled)*/
#define BQB_VOLTAGE_TYPE         1      /**< RF tx power voltage type (1:vbat,0:vant)*/

/* set TX return result */
#define RETURN_TX_LEN_EN 0 //1: return tx length, 0:do not return tx length

#define BQB_TX_POWER RF_POWER_P10p00dBm

/* set uart port */
#define BQB_UART_TX_PORT GPIO_FC_PA1
#define BQB_UART_RX_PORT GPIO_FC_PA2

/* global value for configuration */
extern bqb_setting_t g_bqb_setting;


/* global value for uart */
extern uart_num_redef_e uart_using;

void bqb_user_init(void);

/**
 * @brief       This function serves to initialize  BQB
 * @param[in]   flash_size - flash size: 0->512K, 1->2M, 2-0xff->1M.
 * @return      none.
 */
void bqbtest_init(void);

/**
 * @brief   This function serves to read the uart data and execute BQB program
 * @param   none.
 * @return  none.
 */
void bqb_serviceloop(void);

/**
 * @brief     uart send data function with not DMA method.
 *            variable uart_TxIndex,it must cycle the four registers 0x90 0x91 0x92 0x93 for the design of SOC.
 *            so we need variable to remember the index.
 * @param[in] uartData - the data to be send.
 * @return    none
 */
void bqb_uart_send_byte(unsigned char uartData);


/**
 * @brief       This function is used to initialize the calibration value of rf fast settle
 * @param[in]   none.
 * @return      none.
 * @note        This function is only used when testing fast settle with BQB
 */
void bqb_fast_settle_init(void);

#ifdef RF_CERTIFICATION_CGF_EN
    #define PA_SUPPORT_USR_CONFIG 1
#else
    #define PA_SUPPORT_USR_CONFIG 0
#endif

#if (PA_SUPPORT_USR_CONFIG)

/**
 * @brief        This function serves to initialize PA setting.
 * @param[in]   addr       - Address to save PA related operations.
 * @param[in]   bypass_en  - To set PA bypass mode, 0:disable 1:enable.
 * @return         none
 */
extern void pa_setting_init(unsigned int addr, unsigned char bypass_en);

#else

/**
 * @brief        This function serves to initialize PA setting.
 * @param[in]   pa_en  - To set PA enable, 0:disable 1:enable.
 * @param[in]   bypass_en  - To set PA bypass mode, 0:disable 1:enable.
 * @return         none
 */
extern void pa_setting_init(unsigned char pa_en, unsigned char bypass_en);

#endif

#endif /* BQB_H_ */
