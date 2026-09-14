/********************************************************************************************************
 * @file    app_d24g_stack.h
 *
 * @brief   This is the header file for Telink RISC-V MCU, specifically for 2.4G wireless protocol stack applications.
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
#ifndef __APP_D24G_STACK_H__
#define __APP_D24G_STACK_H__
#include "../app_common_config.h"
#include "tl_fifo.h"

typedef fifo_cb_t spp_cb_t;

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

// TPSLL stack version
#define TPSLL_STACK_VERSION         "TPSLL_STACK_V_001100"

#define PAIRING_RSSI_THRESHOLD_DFT  (-60)

typedef enum {
    TPSLL_ALG_DISABLE = 0,
    TPSLL_ALG_ENABLE  = 1
} TPSLL_ALG_STATUS_E;

typedef enum
{
    TPSLL_KB_ACTIVE_PM_OPTM_LEVEL1 = 0,
    TPSLL_KB_ACTIVE_PM_OPTM_LEVEL2,
    TPSLL_KB_ACTIVE_PM_OPTM_LEVEL3,
    TPSLL_KB_ACTIVE_PM_OPTM_LEVEL4,
    TPSLL_KB_ACTIVE_PM_OPTM_LEVEL5
} tpsll_kb_active_pm_level_e;


/**
 * Note:
 * When adding struct members, you must adhere to the struct alignment principles to achieve
 * maximum code execution efficiency and ensure compatibility.
 * **/
// Structure storing application asynchronous state and context
typedef struct
{
    /* 4-byte aligned group (members aligned to 4-byte boundary) */
    uint32_t side_id; // Side identifier (distinguishes between master/slave or left/right devices)

    /* 2-byte aligned group (members aligned to 2-byte boundary) */

    uint16_t tag; // General purpose tag (for state tracking)

    /* 1-byte aligned group (members aligned to 1-byte boundary) */
    uint8_t dev_now_status;    // Current device status (e.g., idle, connected, pairing)

    uint8_t connecting_dev; // Identifier of device being connected to
    uint8_t paired_dev_type; // Type of paired device (e.g., keyboard, mouse)

    /* arrays naturally 1-byte aligned */
    uint8_t mac[MAC_ADDR_LEN];      // Local device MAC address
    uint8_t peer_mac[MAC_ADDR_LEN]; // Peer device MAC address

    /* explicit padding to ensure 4-byte alignment of the entire struct */
    uint8_t _rsv_align[2];

} ll_inf_t;
// Global instance of application asynchronous state
extern ll_inf_t  ll_inf;


/**
 * @brief   RF interrupt handler
 *
 * This function handles RF-related interrupts, including packet reception,
 * transmission completion, and error conditions. It processes RF events
 * and triggers appropriate stack state transitions.
 *
 * @return    0: success
 */
uint8_t tpsll_rf_irq_handler(void);


// Current keyboard LED status (volatile: may be modified by interrupts)
// (bitmask for Num Lock, Caps Lock, Scroll Lock, etc.)
extern   volatile unsigned char  kb_led_status;

/**
 * @brief   initialize rf module
 *
 * This function configure and initialize rf module.
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
 */
uint8_t tpsll_rf_init(void);

/**
*  @brief    Enable or disable pairing mode
*  @param[in]  enable    true: enable pairing mode; false: disable pairing mode
*  @return    0: success
*            Other: failure (e.g., invalid parameter)
*/
uint8_t tpsll_enable_pairing(bool enable);


/**
 * @brief Initialize TPSLL stack with configuration and application callbacks.
 * @param p_config Pointer to the stack configuration structure.
 * @note  Must be called before using any stack APIs.
 * 
 * @return    0: success  
 *            Other: failure (e.g., invalid parameter)
 
 */
uint8_t tpsll_stack_init(tpsll_stack_config_t *p_config);

/**
*  @brief    Handler for USB enumeration completion event
*            (performs actions after USB is enumerated, e.g., start communication)
*  @param[in]  none
*  @return  none
*/
void tpsll_usb_enum_done_handler(void);

/**
*  @brief    Handler for USB SOF (Start of Frame) interrupt
*            (synchronizes operations with USB frame timing)
*  @param[in]  none
*  @return  none
*/
void tpsll_usb_sof_handler(void);

/**
*  @brief    Handler for Timer1 interrupt (used for protocol stack timing)
*  @param[in]  none
*  @return    0: success
*/
uint8_t tpsll_timer1_irq_handler(void);

/**
*  @brief    Handler for software timer interrupt (processes software timer events)
*  @param[in]  none
*  @return    0: success
*/
uint8_t tpsll_stimer_irq_handler(void);

/**
*  @brief    Initialize debug IO pins for protocol stack debugging
*            (configures GPIOs for monitoring stack behavior, e.g., TX/RX indicators)
*  @param[in]  io0~io7    GPIO pins to be used for debugging
*  @return    0: success
*            Other: failure (e.g., invalid parameter)
*/
uint8_t tpsll_stack_debug_io_init(gpio_pin_e io0, gpio_pin_e io1, gpio_pin_e io2, gpio_pin_e io3,
                                    gpio_pin_e io4, gpio_pin_e io5, gpio_pin_e io6, gpio_pin_e io7);

/**
*  @brief      Get the current device state
*  @param[in]  none
*  @return     Current device state (of type tpsll_dev_status_e)
*/
tpsll_dev_status_e tpsll_get_state(void);

/**
 * @brief     Enable or disable the 2.4GHz automatic reconnection feature
 *
 * This function enables or disables the automatic reconnection mechanism
 * in the 2.4GHz protocol stack. 
 *
 * @param[in] enable    true: enable reconnection  
 *                      false: disable reconnection
 *
 * @return    0: success  
 *            Other: failure (e.g., invalid parameter or operation not allowed)
 *
 * @note      This function should be called after the device has been paired.
 * 
 */
uint8_t tpsll_enable_reconn(bool enable);

/**
 * @brief Set (write) stack layer variable
 *
 * This function writes a value to the specified stack layer variable,
 * allowing runtime configuration of stack parameters.
 * 
 * @param p_stack_var: Pointer to stack variable data to be set
 * @param var_id: Stack variable ID to configure
 * @return    0: success  
 *            Other: failure (e.g., invalid parameter or operation not allowed)
 *
 */
uint8_t tpsll_stack_set_var(const tpsll_stack_var_t *p_stack_var, tpsll_stack_var_id_e var_id);

/**
 * @brief Get (read) stack layer variable
 *
 * This function reads the current value of a specified stack layer variable,
 * allowing the application to query runtime stack parameters.
 * 
 * @param p_stack_var: Pointer to store read stack variable data
 * @param var_id: Stack variable ID to read
 * @return    0: success  
 *            Other: failure (e.g., invalid parameter or operation not allowed)
 *
 */
uint8_t tpsll_stack_get_var(tpsll_stack_var_t *p_stack_var, tpsll_stack_var_id_e var_id);

/**
 * @brief     Send SPP (Serial Packet Protocol) data through the 2.4GHz stack
 *
 * This function sends an SPP data payload with optional retry mechanism
 * and an application callback. The stack handles packet formatting,
 * transmission scheduling, and retry operations based on the specified
 * retry count.
 *
 * @param[in] data        Pointer to SPP payload buffer
 * @param[in] len         Length of the payload, valid range: 1 ~ 65 bytes
 * @param[in] retry_num   Maximum number of retransmissions allowed
 * @param[in] cb          Callback invoked when transmission completes
 *                        (success or failure), type: @ref spp_cb_t
 * @param[in] user_arg    User-defined argument passed to the callback
 *
 * @return
 *      0  : Success  
 *     >0  : Error code (FIFO full, invalid parameters, or internal failure)
 *
 * @note      The callback is executed after the transmission result is known.
 *            The maximum payload length supported is 65 bytes; passing a len
 *            value greater than 65 will trigger an invalid parameter error.
 *            
 */
uint8_t tpsll_send_spp_data(uint8_t *data, uint8_t len, uint8_t retry_num,
                        spp_cb_t cb, void *user_arg);

/**
 * @brief   Reset spp fifo
 *
 * This function clears all pending SPP data in the FIFO by resetting
 * both read and write pointers and clearing the SPP transmission flag.
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
 */
uint8_t tpsll_reset_spp_fifo(void);

/**
 * @brief   Change data report rate
 *
 * This function modifies the data report rate for the connected device.
 *
 * @param[in] rr       New report rate to set (type: @ref report_rate_t)
 *
 * @return  Status code:
 *          - 0 : Success
 *          - Non-zero : Failure (specific codes depend on implementation)
 */
uint8_t tpsll_change_report_rate(report_rate_t rr);

/**
 * @brief   Get current report rate
 *
 * This function returns the current data report rate configuration
 * for the connected device (e.g., mouse/keyboard report frequency).
 *
 * @return  Current report rate (type: @ref report_rate_t)
 */
report_rate_t tpsll_get_report_rate(void);

/**
 * @brief   Register event callback function
 *
 * This function registers a callback to be invoked when specific 2.4G
 * stack events occur (e.g., connection established, data received).
 *
 * @param[in] evt    Event type to monitor (type: @ref tpsll_evt_t)
 * @param[in] cb     Callback function to register (type: @ref tpsll_evt_callback_t)
 *
 * @return  Status code:
 *          - 0 : Success
 *          - Non-zero : Failure (e.g., invalid parameters)
 */
uint8_t tpsll_register_callback(tpsll_evt_t evt, tpsll_evt_callback_t cb);

/**
*  @brief    Main loop function for protocol stack processing
*            (handles state machines, data transmission/reception, etc.)
*  @param[in]  none
*  @return    0: success
*/
uint8_t tpsll_stack_main_loop(void);

/**
 * @brief     Set processing time (in microseconds) for IR button by user
 *
 * This function configures the gap time used for IR button sampling
 * in the 2.4GHz protocol stack with 8K report rate mode. The setting only 
 * takes effect when the specified interval value is within a valid range.
 *
 * @param[in] us     Processing time in microseconds
 *                   Valid range: 20 ~ 60 (inclusive)
 *
 * @return
 *      0   : Success  
 *     >0   : Error code (invalid parameter or operation failed)
 *
 * @note     If the parameter value is less than 20 or greater than 60,
 *           the configuration will not be applied and an error code
 *           will be returned.
 */
uint8_t tpsll_set_proc_ir_btn_us(uint8_t us);

/**
 *  @brief    Request the stack to enable enter suspend mode
 *  This function request the stack enter suspend mode
 *  @param    NONE
 *  @return  int - 0: success; other: not allowed
 */
uint8_t tpsll_request_suspend(void);

/**
 *  @brief    Set the suspend timing interval
 *  This function request the stack enter suspend mode
 *  @param   suspend_interval_ms  Processing time in microseconds
 *           Valid range: 10 ~ 500 (inclusive)
 *  @return
 *      0   : Success
 *     >0   : Error code (invalid parameter or operation failed)
 */
uint8_t tpsll_set_suspend_interval(uint16_t suspend_interval_ms);

/**
 * @brief   Get the version number of the 2.4G stack
 *
 * This function retrieves the version string of the 2.4G stack, which is defined
 * as a read-only constant and used for version identification and debugging.
 *
 * @return  const char* - Pointer to the read-only stack version string.
 */
const uint8_t* tpsll_get_stack_version(void);

/**
 * @brief     Set pairing encryption key for Dongle side by user
 *
 * This function configures the custom encryption key and key length used for
 * secure pairing process in the 2.4GHz protocol stack. The stack will generate
 * a 4-byte encrypted pairing key based on this custom key and USB VID.
 *
 * @param[in] key      Pointer to user-defined encryption key buffer
 * @param[in] key_len  Length of the user-defined encryption key
 *                     Valid range: 1 ~ 32 (inclusive)
 *
 * @return
 *      0   : Success
 *     >0   : Error code (invalid parameter or operation failed)
 *
 * @note     If key is NULL or key length out of valid range,
 *           the configuration will not be applied and an error code
 *           will be returned.
 */
uint8_t tpsll_set_pair_encrypt_key(const uint8_t *key, uint8_t key_len);


/**
 * @brief Set pairing access code used during pairing procedure.
 *
 * The access code must be a valid 32-bit value and shall not contain
 * more than 3 consecutive identical bits (continuous 0s or 1s).
 *
 * Invalid examples:
 * - 0b0000 1100
 * - 0b0111 1001
 *
 * @param[in] ac
 *     4-byte pairing access code.
 *
 * @return
 *     - 0      : Set successfully.
 *     - Others : Set failed due to invalid access code or internal error.
 */
uint8_t tpsll_set_pairing_access_code(uint32_t ac);

/**
 * @brief Set RSSI threshold for pairing mode proximity check.
 *
 * During pairing, the received pairing packet RSSI must exceed this threshold
 * to continue the pairing process. This ensures the mouse / keyboard and dongle are
 * physically close enough before pairing succeeds.
 *
 * @param[in] threshold
 *     RSSI threshold value in dBm (int8_t, e.g. -60).
 *     Higher values (less negative) require closer proximity.
 *
 * @note
 *     This threshold and the mouse-side / keyboard-side pairing tx power_level are COUPLED.
 *     Changing one requires re-tuning the other against actual distance requirements.
 *
 *     Current tested combination (reference):
 *       Mouse / Keyboard TX power : -4.7 dBm
 *       Dongle threshold          : -60 dBm
 *     Expected behavior:
 *       < 0.5m      : instant pairing
 *       0.5m - 1m   : can pair, but slower
 *       > 2m        : pairing fails
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
 */
uint8_t tpsll_set_pairing_rssi_threshold(int8_t threshold);

/**
 * @brief     Enable or disable the algorithm function
 *
 * This function configures the working state of the algorithm module
 * in the 2.4GHz protocol stack.
 *
 * @param[in] enable     Enable or disable the algorithm function
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
 *
 * @note     This function should be called after stack initialization.
 */
uint8_t tpsll_alg_set_enable(TPSLL_ALG_STATUS_E enable);

/**
 * @brief     Get current status of the algorithm function
 *
 * This function retrieves the current working state (enabled or disabled) of the
 * algorithm module in the 2.4GHz protocol stack for status query
 * and debugging purposes.
 *
 * @return
 *      TPSLL_ALG_DISABLE   Algorithm function is disabled
 *      TPSLL_ALG_ENABLE    Algorithm function is enabled
 *
 * @note     The returned status is the real-time configuration of the module.
 */
TPSLL_ALG_STATUS_E tpsll_alg_get_status(void);

/**
 * @brief Set the USB SOF (Start of Frame) start flag.
 *
 * This function sets the internal flag indicating whether USB SOF
 * signaling has started.
 *
 * @param[in] flag    USB start indicator:
 *                    - Non-zero: USB has started
 *                    - 0: USB has not started
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
 */
uint8_t tpsll_set_usb_start_flag(uint8_t flag);

/**
 * @brief Get the USB SOF (Start of Frame) start flag.
 *
 * This function returns the current value of the internal USB start flag,
 * indicating whether USB SOF signaling has been detected and is active.
 *
 * @return  Current USB start flag value:
 *          - Non-zero: USB has started
 *          - 0: USB has not started
 */
uint8_t tpsll_get_usb_start_flag(void);

/**
 * @brief     set the start time of keyboard active power optimization stage
 *
 * This function set the start time of keyboard active power optimization stage in the 2.4GHz protocol stack,
 * must be consistent with the keyboard configuration.
 * The valid range of stage_start_time_ms is from 300ms to 30 minutes.
 *
 * @param[in] stage      Power optimization level in use
 * @param[in] stage_start_time_ms      Start time (from keyboard event active) of the power optimization level in milliseconds
 *
 * @return
 *      0   : Success
 *     >0   : Error code (invalid parameter or operation failed)
 */
uint8_t tpsll_kb_active_pm_optm_level_start_time_set(tpsll_kb_active_pm_level_e stage, uint32_t stage_start_time_ms);

/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
}
#endif

#endif  // __APP_D24G_STACK_H__