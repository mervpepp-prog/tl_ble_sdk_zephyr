/********************************************************************************************************
 * @file    app_d24g_stack.h
 *
 * @brief   This is the header file for Telink RISC-V MCU 2.4G stack application
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

#include "../../app_common_config.h"

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
#endif

typedef fifo_cb_t spp_cb_t;
typedef fifo_cb_t kb_cb_t;


// TPSLL stack version
#define TPSLL_STACK_VERSION         "TPSLL_STACK_V_001100"


#define APP_DEFAULT_REPORT_RATE     REPORT_RATE_8K //fixed

#define PAIRING_TX_POWER_DEFAULT    RF_POWER_N4p70dBm

typedef enum
{
    TPSLL_KB_ACTIVE_PM_OPTM_LEVEL1 = 0,
    TPSLL_KB_ACTIVE_PM_OPTM_LEVEL2,
    TPSLL_KB_ACTIVE_PM_OPTM_LEVEL3,
    TPSLL_KB_ACTIVE_PM_OPTM_LEVEL4,
    TPSLL_KB_ACTIVE_PM_OPTM_LEVEL5
} tpsll_kb_active_pm_level_e;

typedef enum
{
    TPSLL_PM_OPTM_LEVEL1 = BIT(0),  /**< 1st-level PM optimization: reduce keep-alive sync frequency, enter suspend during idle gaps */
    TPSLL_PM_OPTM_LEVEL2 = BIT(1),  /**< 2nd-level PM optimization: further reduce keep-alive sync, expand RX window for suspend support */
    TPSLL_PM_OPTM_LEVEL3 = BIT(2),  /**< 3rd-level PM optimization: disconnect link and enter IDLE state */
    TPSLL_PM_OPTM_LEVEL4 = BIT(3),  /**< 4th-level PM optimization: enter sleep mode */
} tpsll_pm_optm_level_e;

/**
 * Note:
 * When adding struct members, you must adhere to the struct alignment principles to achieve
 * maximum code execution efficiency and ensure compatibility.
 * **/
typedef struct
{
    /* ---- 4-byte aligned ---- */
    uint32_t side_id; /* Device side identifier */

    /* ---- 2-byte aligned ---- */

    uint16_t tag;      /* Device tag identifier */
    uint16_t peer_tag; /* Peer device tag identifier */

    /* ---- 1-byte aligned ---- */
    uint8_t dev_now_status;    /* Current device status */
    uint8_t pair_success_flag; /* Pairing success indicator flag */
    uint8_t save_rr_flag;      /* Save report rate flag */

    uint8_t ms_idle_cnt; /* Mouse idle counter */

    uint8_t mac[MAC_ADDR_LEN];      /* Local MAC address */
    uint8_t peer_mac[MAC_ADDR_LEN]; /* Peer MAC address */

    /* explicit padding if needed to align whole struct to 4 bytes */
    // uint8_t rsv[3]; /* Reserved for alignment */

} ll_inf_t;

extern ll_inf_t  ll_inf;

/**
 * @brief   Compute FNV-1a 16-bit hash for a data buffer
 *
 * This function calculates the FNV-1a (Fowler-Noll-Vo) 16-bit hash value
 * for the given data buffer. The FNV-1a algorithm provides a fast,
 * non-cryptographic hash suitable for data integrity checks, tag
 * generation, and lookup table indexing in the 2.4G protocol stack.
 *
 * @param[in] data    Pointer to the input data buffer
 * @param[in] len     Length of the input data in bytes
 *
 * @return  16-bit FNV-1a hash value
 */
uint16_t tpsll_fnv1a_16(uint8_t *data, size_t len);

/**
 * @brief   Initialize stack timer
 *
 * This function initializes the timer used by the 2.4G stack for
 * timing management, including connection intervals and timeout tracking.
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
 */
uint8_t tpsll_stimer_init(void);

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
 * @brief   RF interrupt handler
 *
 * This function handles RF-related interrupts, including packet reception,
 * transmission completion, and error conditions. It processes RF events
 * and triggers appropriate stack state transitions.
 *
 * @return    0: success
 */
uint8_t tpsll_rf_irq_handler(void);


/**
 * @brief   Main stack processing loop
 *
 * This is the core processing function for the 2.4G stack, responsible for
 * managing connection states, processing incoming/outgoing packets,
 * handling pairing procedures, and maintaining communication with peers.
 * It should be called periodically from the main application loop.
 *
 * @return    0: success
 */
uint8_t tpsll_stack_main_loop(void);


/**
 * @brief   Get current RF link status
 *
 * This function returns the current status of the RF link, indicating
 * whether a valid connection is established and its quality metrics.
 *
 * @return  Link status code (specific values depend on implementation)
 */
unsigned int tpsll_get_rf_link_status(void);


/**
 * @brief   Enable/disable pairing functionality
 *
 * This function controls whether the device is in pairing mode, allowing
 * it to discover and establish connections with new peer devices.
 *
 * @param[in] enable       Pairing enable flag:
 *                         - true  : Enable pairing mode
 *                         - false : Disable pairing mode
 * @param[in] power_level  TX power level applied during pairing.
 *                         See @ref rf_power_level_e for available levels.
 *
 * @return  0: success  
 *          Other: failure (e.g., invalid parameter)
 * 
 * @note   Pairing tx power_level and the dongle-side pairing rssi threshold
 *         are COUPLED parameters. Changing one requires re-tuning the other
 *         against the actual pairing distance requirements.
 *
 *         Current tested combination (reference):
 *           Keyboard TX power : -4.7 dBm
 *           Dongle threshold  : -60 dBm
 *         Expected behavior:
 *           < 0.5m    : instant pairing
 *           0.5m - 1m : can pair, but slower
 *           > 2m      : pairing fails
 */
uint8_t tpsll_enable_pairing(bool enable, rf_power_level_e power_level);


/**
 * @brief   Get current device state
 *
 * This function returns the current operational state of the 2.4G device,
 * such as idle, pairing, connected, or disconnected.
 *
 * @return  Current device state (type: @ref tpsll_dev_status_e)
 */
tpsll_dev_status_e tpsll_get_state(void);

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
 * @brief   Initialize 2.4G stack
 *
 * @param[in] is_alg_kb  true: analog keyboard, false: digital keyboard
 * @param[in] rr    report rate after connection 
 * @param[in] p_config  Pointer to stack configuration structure
 *
 * This function performs all necessary initialization for the 2.4G
 * communication stack, including RF configuration, timer setup,
 * state initialization, and resource allocation.
 *
 * @return    0: success  
 *            Other: failure (e.g., invalid parameter)
 */
uint8_t tpsll_stack_init(bool is_alg_kb, report_rate_t rr, tpsll_stack_config_t *p_config);


/**
 * @brief   Stack timer interrupt handler
 *
 * This function handles interrupts from the stack timer, managing
 * timing-critical operations such as connection interval maintenance,
 * timeout detection, and scheduled transmissions.
 *
 * @return    0: success
 */
uint8_t tpsll_stimer_irq_handler(void);


/**
 * @brief   Timer1 interrupt handler
 *
 * This function processes Timer1 interrupts used by the 2.4G stack
 * for specific timing functions (implementation-dependent).
 *
 * @return    0: success
 */
uint8_t tpsll_timer1_irq_handler(void);


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
 * @brief   Initialize stack debug IO pins
 *
 * This function configures specified GPIO pins for debug purposes,
 * allowing monitoring of stack internal states through physical pins.
 *
 * @param[in] io0 - io7    GPIO pins to use for debug signals
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
 */
uint8_t tpsll_stack_debug_io_init(gpio_pin_e io0, gpio_pin_e io1, gpio_pin_e io2, gpio_pin_e io3,
                                    gpio_pin_e io4, gpio_pin_e io5, gpio_pin_e io6, gpio_pin_e io7);


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
 * @brief   Enable/disable reconnection functionality
 *
 * This function controls whether the device will attempt to reconnect
 * to previously paired devices when a connection is lost.
 *
 * @param[in] enable    Reconnection enable flag:
 *                      - true  : Enable reconnection attempts
 *                      - false : Disable reconnection attempts
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
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
 * @brief     Send keyboard data through the 2.4GHz stack
 *
 * This function sends keyboard data payload with a specified command type,
 * and supports an application callback for notification of transmission status.
 * The stack handles keyboard packet formatting and transmission scheduling.
 *
 * @param[in] data        Pointer to keyboard data payload buffer
 * @param[in] len         Length of the keyboard data payload
 * @param[in] cmd         Keyboard command type
 * @param[in] cb          Callback invoked when keyboard data transmission completes
 *                        (success or failure), type: @ref kb_cb_t
 * @param[in] user_arg    User-defined argument passed to the keyboard data callback
 *
 * @return
 *      0  : Success
 *    !=0  : Error code (FIFO full, invalid parameters, or internal stack failure)
 *
 * @note      The callback is executed after the transmission result is determined.
 *            This function is executed in RAM for high-speed processing requirements.
 *
 */
uint8_t tpsll_send_keyboard_data(uint8_t *data, uint8_t len, uint8_t cmd,
                            kb_cb_t cb, void *user_arg);


/**
 * @brief   Enable/disable 2.4G WFI (Wait For Interrupt) mode
 *
 * This function controls the WFI power-saving mode for the 2.4G radio,
 * allowing the radio to enter low-power states while waiting for events.
 *
 * @param[in] en    WFI enable flag:
 *                  - true  : Enable WFI mode
 *                  - false : Disable WFI mode
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
 */
uint8_t tpsll_wfi_en(bool en);


/**
 * @brief   Control RF module power
 *
 * This function enables or disables the RF transceiver module,
 * turning off RF functionality completely when disabled.
 *
 * @param[in] enable    RF enable flag:
 *                      - true  : Enable RF module
 *                      - false : Disable RF module
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
 */
uint8_t tpsll_rf_control(bool enable);


/**
 * @brief   Control power management features
 *
 * This function enables or disables power management features in the
 * 2.4G stack, such as automatic sleep modes and power-saving states.
 *
 * @param[in] enable    PM enable flag:
 *                      - true  : Enable power management
 *                      - false : Disable power management
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
 */
uint8_t tpsll_pm_control(bool enable);


/**
 * @brief   Configure timeout parameters for 2.4G stack power management
 *
 * This function configures millisecond-based timeouts for pairing, reconnection 
 * and connection in the 2.4G wireless stack, effective for subsequent power management.
 *
 * @param[in] timeout_type  Enumerated type of timeout to configure
 * @param[in] timeout_ms    Timeout value in milliseconds; the minimum valid value is 120ms
 *
 * @return
 *      0   : Success  
 *     >0   : Error code (invalid parameter or operation failed)
 * @note    If the input timeout_ms is less than the minimum threshold (120ms), 
 *          it will be automatically clamped to the minimum valid value to avoid invalid configuration.
 */
uint8_t tpsll_set_timeout_ms(tpsll_timeout_type_e timeout_type, uint32_t timeout_ms);

/**
 * @brief   Refresh the time count for power management optimization
 *
 * This function resets the timer used to trigger power-saving optimizations.
 *
 * @return
 *      0   : Success
 *     >0   : Error code (invalid parameter or operation failed)
 */
uint8_t tpsll_pm_optm_time_refresh(void);

/**
 * @brief   Set the delay time for power management optimization at a given level.
 *
 * This function configures the delay duration (in seconds) before entering a 
 * specific power optimization level. The timer starts counting from the most 
 * recent action or event.
 *
 * @param[in] optm_level    Power management optimization level to configure.
 *                          Valid values:
 *                          - TPSLL_PM_OPTM_LEVEL1: First-level optimization
 *                          - TPSLL_PM_OPTM_LEVEL2: Second-level optimization
 *                          - TPSLL_PM_OPTM_LEVEL3: Third-level optimization
 * @param[in] delay_time_s  Delay time in seconds.
 *                          Valid range: 1 ~ 7200 (1 second to 3 hour).
 *                          Set to 0 to disable this level (never enter).
 *
 * @return  TLK_SUCCESS (0) on success.
 * @return  TLK_ERR_INVALID_PARAM if optm_level is invalid or start_time_s is out of range.
 */
uint8_t tpsll_pm_optm_delay_time_set(tpsll_pm_optm_level_e optm_level, uint32_t delay_time_s);

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
 * @brief     Set pairing encryption key for Mouse side by user
 *
 * This function configures the custom encryption key and key length used for
 * secure pairing process in the 2.4GHz protocol stack.
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
 * @brief Push keyboard/mouse application data to the KM FIFO queue.
 *
 * This function enqueues keyboard or mouse application data into the
 * KM (Keyboard/Mouse) FIFO for subsequent transmission over the 2.4GHz
 * protocol stack.
 *
 * @param[in] cmd      Indicate the data type for km fifo
 * @param[in] buf      Pointer to payload buffer
 * @param[in] len      Length of the payload
 *
 * @return
 *      0   : Success
 *     >0   : Error code (invalid parameter or operation failed)
 */
int tpsll_push_kmfifo(uint8_t cmd, uint8_t *buf, uint8_t len);

/**
 * @brief Pop the current element from the KM FIFO queue.
 *
 * This function removes the current (head) element from the KM FIFO, advancing 
 * the read pointer to the next queued element.
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
 */
uint8_t tpsll_pop_kmfifo(void);

/**
 * @brief Reset the KM FIFO queue.
 *
 * This function clears all pending data in the KM FIFO by resetting
 * both read and write pointers to their initial positions.
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
 */
uint8_t tpsll_reset_kmfifo(void);

/**
 * @brief Get the data pointer of the current KM FIFO element.
 *
 * This function returns a pointer to the data buffer of the current
 * (head) element in the KM FIFO without removing it.
 *
 * @return Pointer to the current KM FIFO element's data buffer,
 *         or NULL if the FIFO is empty.
 */
uint8_t* tpsll_get_ptr_kmfifo(void);

/**
 * @brief Get the number of elements in the KM FIFO queue.
 *
 * This function returns the count of pending elements currently
 * queued in the KM FIFO.
 *
 * @return Number of elements currently in the KM FIFO.
 */
uint16_t tpsll_get_num_kmfifo(void);

/**
 * @brief Request the device to enter sleep mode
 *
 * @details This function sends a request to trigger the device sleep procedure.
 *          It will initiate power-saving operations, disable unnecessary peripherals,
 *          and transition the system to low-power sleep state when conditions are met.
 *
 * @return    0: success
 *            Other: failure (e.g., invalid parameter)
 */
uint8_t tpsll_request_sleep(void);

/**
 * @brief     set the start time of keyboard active power optimization stage
 *
 * This function set the start time of keyboard active power optimization stage in the 2.4GHz protocol stack,
 * must be consistent with the dongle configuration.
 * The valid range of stage_start_time_ms is from 300ms to 30 minutes.
 * (Low power with low latency)
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

#endif