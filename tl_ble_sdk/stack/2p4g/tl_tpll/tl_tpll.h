/********************************************************************************************************
 * @file    tl_tpll.h
 *
 * @brief   This is the header file for 2.4G SDK
 *
 * @author  2.4G Group
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
#ifndef _TL_TPLL_H_
#define _TL_TPLL_H_

#include "driver.h"
#include "types.h"
#include "../tpll/tpll.h"
#ifdef __cplusplus
extern "C"
{
#endif
#define TL_TPLL_MAX_PAYLOAD_LENGTH      192 /** 252 is the largest possible payload size according to the requirement, including 3 MAC bytes. */
#define TL_TPLL_ERROR_BASE_NUM          (0x0)                        /** Global error base */
#define TL_TPLL_SUCCESS                 (TL_TPLL_ERROR_BASE_NUM + 0) /** Successful command */
#define TL_TPLL_ERROR_NULL              (TL_TPLL_ERROR_BASE_NUM + 1) /** Null Pointer */
#define TL_TPLL_ERROR_INVALID_PARAM     (TL_TPLL_ERROR_BASE_NUM + 2) /** Invalid Parameter */
#define TL_TPLL_ERROR_BUSY              (TL_TPLL_ERROR_BASE_NUM + 3) /** Busy */
#define TL_TPLL_INVALID_STATE           (TL_TPLL_ERROR_BASE_NUM + 4) /** Invalid state, operation disallowed in this state */
#define TL_TPLL_ERROR_BUFFER_EMPTY      (TL_TPLL_ERROR_BASE_NUM + 5) /** FIFO is empty */
#define TL_TPLL_ERROR_NO_MEM            (TL_TPLL_ERROR_BASE_NUM + 6) /** No Memory for operation */
#define TL_TPLL_ERROR_INVALID_LENGTH    (TL_TPLL_ERROR_BASE_NUM + 7) /** Invalid Length */
#define TL_TPLL_ERROR_NOT_IN_RX_MODE    (TL_TPLL_ERROR_BASE_NUM + 8) /** Not in RX mode */



/** @brief Macro for verifying that a function returned TL_TPLL_SUCCESS. It will cause the exterior
 *        function to return error code of statement if it is not @ref TL_TPLL_SUCCESS.
 *
 * @param[in] statement     Statement to check against TL_TPLL_SUCCESS.
 */
#define VERIFY_SUCCESS(statement)                                                                                      \
    do                                                                                                                 \
    {                                                                                                                  \
        uint32_t _err_code = (uint32_t)(statement);                                                                    \
        if (_err_code != TL_TPLL_SUCCESS)                                                                              \
        {                                                                                                              \
            return _err_code;                                                                                          \
        }                                                                                                              \
    } while (0)

/** @brief  Telink primary link layer transmission modes. */
typedef enum
{
    TL_TPLL_TXMODE_AUTO,
    TL_TPLL_TXMODE_MANUAL
} tl_tpll_tx_mode_t;

/** @brief TL_TPLL modes. */
typedef enum {
    TL_TPLL_MODE_PTX,          /**< Primary transmitter mode. */
    TL_TPLL_MODE_PRX           /**< Primary receiver mode.    */
} tl_tpll_mode_t;

/** @brief Telink primary link layer event IDs used to indicate the type of the event. */
typedef enum
{
    TL_TPLL_EVENT_TX_SUCCESS,  /** Event triggered on TX success   */
    TL_TPLL_EVENT_TX_FAILED,   /** Event triggered on TX failure   */
    TL_TPLL_EVENT_RX_RECEIVED, /** Event triggered on RX received  */
} tl_tpll_event_id_t;

/** An enum describing the Telink primary link layer mode state-machine status.
 *
 */
typedef enum
{
    TL_TPLL_STATE_IDLE = 0,  /**< idle */
    TL_TPLL_STATE_ACTIVE,    /**< active */
    TL_TPLL_STATE_TX_SETTLE, /**< tx settle*/
    TL_TPLL_STATE_TX,        /**< tx */
    TL_TPLL_STATE_RX_WAIT,   /**< rx wait */
    TL_TPLL_STATE_RX,        /**< rx */
    TL_TPLL_STATE_TX_WAIT,   /**< tx wait */
} tl_tpll_mainstate_t;

/** @brief Definition of the event handler for the module. */
typedef void (*tl_tpll_event_handler_t)(tl_tpll_event_id_t evt_id);

///** @brief Telink primary link layer bitrate modes. */
//typedef enum
//{
//    TL_TPLL_BITRATE_1MBPS = RF_MODE_PRI_GENERIC_1M, /** 1 Mb radio mode. */
//} tl_tpll_bitrate_t;

/** @brief Telink primary link layer pipe IDs. */
typedef enum
{
    TL_TPLL_PIPE0 = 0,      /** Select pipe0               */
    TL_TPLL_PIPE1,          /** Select pipe1               */
    TL_TPLL_PIPE2,          /** Select pipe2               */
    TL_TPLL_PIPE3,          /** Select pipe3               */
    TL_TPLL_PIPE4,          /** Select pipe4               */
    TL_TPLL_PIPE5,          /** Select pipe5               */
    TL_TPLL_PIPE6,          /** Select pipe6               */
    TL_TPLL_PIPE7,          /** Select pipe7               */
    TL_TPLL_PIPE_ALL = 0xFF /** Close or open all pipes    */
} tl_tpll_pipeid_t;

/** @brief Telink primary link layer address width. */
typedef enum
{
    TL_TPLL_ADDRESS_WIDTH_3BYTES = 3, /** Set address width to 3 bytes */
    TL_TPLL_ADDRESS_WIDTH_4BYTES,     /** Set address width to 4 bytes */
    TL_TPLL_ADDRESS_WIDTH_5BYTES      /** Set address width to 5 bytes */
} tl_tpll_address_width_t;


/** @brief Telink primary link layer payload.
 *
 * @details This payload is used both for transmissions and for acknowledging a received packet with a payload.
 */
typedef struct
{
    uint8_t length;                                      /** Length of the packet (maximum value is @ref TL_TPLL_MAX_PAYLOAD_LENGTH) */
    uint8_t pipe;                                        /** Pipe used for this payload */
    uint8_t noack;                                       /** Flag indicating that this packet will not be acknowledgement. Flag is ignored when selective
                                                            auto ack is enabled. 1 for no ack, 0 for ack */
    uint8_t pid;                                         /** PID assigned during communication */
    int8_t  rssi;                                        /** RSSI for the received packet */
    uint32_t rx_timestamp;                               /** Time stamp for the received packet */
    uint8_t data[TL_TPLL_MAX_PAYLOAD_LENGTH];            /** The payload data */
    bool crc;                                            /** 1 for crc error, 0 for no crc error */
} tl_tpll_payload_t;

/**@brief Main configuration structure for the module. */
typedef struct
{
    TPLL_ModeFormatTypeDef      protocol;                /** Telink primary link layer protocol */
    tl_tpll_mode_t               mode;                   /** Telink primary link layer mode */
    tl_tpll_event_handler_t      event_handler;          /** Telink primary link layer event handler */

    // General TL_TPLL parameters
    TPLL_BitrateTypeDef          bitrate;                /** Telink primary link layer bitrate mode */
//    tl_tpll_crc_t                crc;                  /** Telink primary link layer bitrate crc mode */

    rf_power_level_index_e       tx_output_power_idx;    /** Telink primary link layer radio transmission power idx */

    uint16_t                     retransmit_delay;       /** The delay between each retransmission of unacknowledged packets */
    uint16_t                     retransmit_count;       /** The number of retransmission attempts before transmission fail */

    // Control settings
    tl_tpll_tx_mode_t            tx_mode;

    uint8_t                      radio_irq_priority;
    uint8_t                      event_irq_priority;
    uint8_t                      payload_length;         /** Length of the payload (maximum length depends on the platforms that are used on each side */

//    bool                         selective_auto_ack;
//    tl_tpll_pipeid_t             pipe;                   /** Pipe ID */
    rf_irq_e                     irq_mask;               /** IRQ mask */
    bool                         deepRetWakeUp;
    bool                         fast_settle_en;
} tl_tpll_config_t;

/** @brief Default address configuration for Telink primary link layer. */
#define TL_TPLL_ADDR_DEFAULT                                                                                           \
    {                                                                                                                  \
        .base_address_0          = {0xA2, 0x67, 0x89},                                                                 \
        .base_address_1          = {0x71,0x67, 0x89},                                                                  \
        .pipe_prefixes           = {0x9B, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8},                                   \
        .addr_length             = 4,                                                                                  \
        .pipe_nums               = TL_TPLL_PIPE_COUNT                                                                  \
    }

/** @brief  Default radio parameters.*/
#define TL_TPLL_DEFAULT_CONFIG                                                                                         \
    {                                                                                                                  \
        .protocol                = TPLL_MODE_LEGACY_FORMAT,                                                            \
        .mode                    = TL_TPLL_MODE_PRX,                                                                   \
        .bitrate                 = TPLL_BITRATE_2MBPS,                                                                 \
        .tx_output_power_idx     = RF_POWER_INDEX_P0p00dBm,                                                            \
        .retransmit_delay        = 150,                                                                                \
        .retransmit_count        = 3,                                                                                  \
        .tx_mode                 = TL_TPLL_TXMODE_AUTO,                                                                \
        .radio_irq_priority      = 1,                                                                                  \
        .event_irq_priority      = 2,                                                                                  \
        .payload_length          = 32,                                                                                 \
        .irq_mask                = 0,                                                                                  \
        .fast_settle_en          = 0                                                                                   \
    }

/** @brief Implementation details for TRF_NUM_VA_ARGS */
#define TRF_LEN_VA_ARGS_CAL(                                                                                           \
    _0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, \
    _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, \
    _48, _49, _50, _51, _52, _53, _54, _55, _56, _57, _58, _59, _60, _61, _62, _63, _64, _65, _66, _67, _68, _69, _70, \
    _71, _72, _73, _74, _75, _76, _77, _78, _79, _80, _81, _82, _83, _84, _85, _86, _87, _88, _89, _90, _91, _92, _93, \
    _94, _95, _96, _97, _98, _99, _100, _101, _102, _103, _104, _105, _106, _107, _108, _109, _110, _111, _112, _113,  \
    _114, _115, _116, _117, _118, _119, _120, _121, _122, _123, _124, _125, _126, _127, N, ...)                        \
    N

/**@brief Macro to get the number of arguments in a call variadic macro call
 *
 * param[in]    ...     List of arguments
 *
 * @retval  Number of variadic arguments in the argument list
 */
#define TRF_NUM_VA_ARGS(...)                                                                                           \
    TRF_LEN_VA_ARGS_CAL(__VA_ARGS__, 128, 127, 126, 125, 124, 123, 122, 121, 120, 119, 118, 117, 116, 115, 114, 113,   \
                        112, 111, 110, 109, 108, 107, 106, 105, 104, 103, 102, 101, 100, 99, 98, 97, 96, 95, 94, 93,   \
                        92, 91, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 70,    \
                        69, 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47,    \
                        46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24,    \
                        23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)

/** @brief Macro to create an initializer for a TX data packet.
 *
 * @details This macro generates an initializer. Using the initializer is more efficient
 *          than setting the individual parameters dynamically.
 *
 * @param[in]   _pipeid The pipe to use for the data packet.
 * @param[in]   ...     Comma separated list of character data to put in the TX buffer.
 *
 * @return  Initializer that sets up the pipe, length, and byte array for content of the TX data.
 */
#define TL_TPLL_CREATE_PAYLOAD(_pipeid, ...)                                                                          \
    {.pipe = _pipeid, .length = TRF_NUM_VA_ARGS(__VA_ARGS__), .data = {__VA_ARGS__}};

/**@brief Function for initializing the Telink primary link layer module.
 *
 * @param  p_config     Parameters for initializing the module.
 *
 * @retval  TL_TPLL_SUCCESS             If initialization was successful.
 * @retval  TL_TPLL_ERROR_NULL           If the @p p_config argument was NULL.
 * @retval  TL_TPLL_ERROR_BUSY          If the function failed because the radio is busy.
 */
uint32_t tl_tpll_init(tl_tpll_config_t const *p_config);


/**@brief Function for suspending the Telink primary link layer module.
 *
 * Calling this function stops ongoing communications without changing the queues.
 *
 * @retval  TL_TPLL_SUCCESS             If Telink primary link layer was suspended.
 * @retval  TL_TPLL_ERROR_BUSY          If the function failed because the radio is busy.
 */
uint32_t tl_tpll_suspend(void);


/**@brief Function for disabling the Telink primary link layer module.
 *
 * Calling this function disables the TL_TPLL module immediately.
 * Doing so might stop ongoing communications.
 *
 * @retval  TL_TPLL_SUCCESS             If Telink primary link layer was disabled.
 */
uint32_t tl_tpll_disable(void);


/**@brief Function for checking if the Telink primary link layer module is idle.
 *
 * @retval true                     If the module is idle.
 * @retval false                    If the module is busy.
 */
bool tl_tpll_is_idle(void);


/**@brief Function for writing a payload for transmission or acknowledgement.
 *
 * @details This function writes a payload that is added to the tx fifo.
 *
 * @param[in]   p_payload     The pointer to the structure that contains information and state of the payload.
 *
 * @retval  TL_TPLL_SUCCESS                     If the payload was successfully queued for writing.
 * @retval  TL_TPLL_ERROR_NULL                  If the required parameter was NULL.
 * @retval  TL_TPLL_INVALID_STATE               If the module is not initialized.
 * @retval  TL_TPLL_ERROR_NO_MEM                If the TX FIFO is full.
 * @retval  TL_TPLL_ERROR_INVALID_LENGTH        If the payload length was invalid (zero or larger than the allowed maximum).
 */
uint32_t tl_tpll_write_payload(tl_tpll_payload_t const * p_payload);


/**@brief Function for reading an RX payload.
 *
 * @param[in,out]   p_payload   The pointer to the structure that contains information and state of the payload.
 *
 * @retval  TL_TPLL_SUCCESS                     If the data was read successfully.
 * @retval  TL_TPLL_ERROR_NULL                  If the required parameter was NULL.
 * @retval  TL_TPLL_INVALID_STATE               If the module is not initialized.
 */
uint32_t tl_tpll_read_rx_payload(tl_tpll_payload_t * p_payload);


/**@brief Function for starting transmission.
 *
 * @retval  TL_TPLL_SUCCESS                     If the TX started successfully.
 * @retval  TL_TPLL_ERROR_BUFFER_EMPTY          If the TX did not start because the FIFO buffer is empty.
 * @retval  TL_TPLL_ERROR_BUSY                  If the function failed because the radio is busy.
 */
uint32_t tl_tpll_start_tx(void);


/**@brief Function for starting to receive data.
 *
 * @retval  TL_TPLL_SUCCESS                     If the transmission was started successfully.
 * @retval  TL_TPLL_ERROR_BUSY                  If the function failed because the radio is busy.
 */
uint32_t tl_tpll_start_rx(void);

void tl_tpll_start_rx_2360Mhz(void);

/** @brief Function for stopping data reception.
 *
 * @retval  TL_TPLL_SUCCESS                     If data reception was stopped successfully.
 * @retval  tl_tpll_ERROR_NOT_IN_RX_MODE    If the function failed because the module is not in RX mode.
 */
uint32_t tl_tpll_stop_rx(void);


/**@brief Function for removing remaining items from the TX buffer.
 *
 * This function clears the TX FIFO buffer.
 *
 * @retval  TL_TPLL_SUCCESS                     If pending items in the TX buffer were successfully cleared.
 * @retval  TL_TPLL_INVALID_STATE               If the module is not initialized.
 */
uint32_t tl_tpll_flush_tx(void);

/**@brief Function for removing remaining items from the RX buffer.
 *
 * @retval  TL_TPLL_SUCCESS                     If the pending items in the RX buffer were successfully cleared.
 * @retval  TL_TPLL_INVALID_STATE               If the module is not initialized.
 */
uint32_t tl_tpll_flush_rx(void);


/**@brief Function for setting the length of the address.
 *
 * @param[in]       length              Length of the Telink primary link layer address (in bytes).
 *
 * @retval  TL_TPLL_SUCCESS                      If the address length was set successfully.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM          If the address length was invalid.
 * @retval  TL_TPLL_ERROR_BUSY                   If the function failed because the radio is busy.
 */
uint32_t tl_tpll_set_address_length(uint8_t length);


/**@brief Function for setting the base address for pipe 0.
 *
 * @param[in]       p_addr      Pointer to the address data.
 *
 * @retval  TL_TPLL_SUCCESS                     If the base address was set successfully.
 * @retval  TL_TPLL_ERROR_BUSY                  If the function failed because the radio is busy.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM         If the function failed because the address given was too close to a zero address.
 * @retval  TL_TPLL_ERROR_NULL                  If the required parameter was NULL.
 */
uint32_t tl_tpll_set_base_address_0(uint8_t const * p_addr);


/**@brief Function for setting the base address for pipe 1 to pipe 7.
 *
 * @param[in]       p_addr      Pointer to the address data.
 *
 * @retval  TL_TPLL_SUCCESS                     If the base address was set successfully.
 * @retval  TL_TPLL_ERROR_BUSY                  If the function failed because the radio is busy.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM         If the function failed because the address given was too close to a zero address.
 * @retval  TL_TPLL_ERROR_NULL                  If the required parameter was NULL.
 */
uint32_t tl_tpll_set_base_address_1(uint8_t const * p_addr);


/**@brief Function for setting the number of pipes and the pipe prefix addresses.
 *
 * This function configures the number of available pipes, enables the pipes,
 * and sets their prefix addresses.
 *
 * @param[in]   p_prefixes      Pointer to a char array that contains the prefix for each pipe.
 * @param[in]   num_pipes       Number of pipes. Must be less than or equal to @ref tl_tpll_PIPE_COUNT.
 *
 * @retval  TL_TPLL_SUCCESS                     If the prefix addresses were set successfully.
 * @retval  TL_TPLL_ERROR_BUSY                  If the function failed because the radio is busy.
 * @retval  TL_TPLL_ERROR_NULL                  If a required parameter was NULL.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM         If an invalid number of pipes was given or if the address given was too close to a zero address.
 */
uint32_t tl_tpll_set_prefixes(uint8_t const * p_prefixes, uint8_t num_pipes);


/**@brief Function for enabling pipes.
 *
 * The @p enable_mask parameter must contain the same number of pipes as has been configured
 * with @ref tl_tpll_set_prefixes. This number may not be greater than the number defined by
 * @ref tl_tpll_PIPE_COUNT
 *
 * @param   enable_mask         Bitfield mask to enable or disable pipes. Setting a bit to
 *                              0 disables the pipe. Setting a bit to 1 enables the pipe.
 *
 * @retval  TL_TPLL_SUCCESS                     If the pipes were enabled and disabled successfully.
 * @retval  TL_TPLL_ERROR_BUSY                  If the function failed because the radio is busy.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM         If the function failed because the enable_mask is neither for pipe 0-7, nor pipe all.
 */
uint32_t tl_tpll_enable_pipes(uint8_t enable_mask);


/**@brief Function for updating the prefix for a pipe.
 *
 * @param   pipe    Pipe for which to set the prefix.
 * @param   prefix  Prefix to set for the pipe.
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_ERROR_BUSY                      If the function failed because the radio is busy.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM             If the given pipe number was invalid or if the address given was too close to a zero address.
 */
uint32_t tl_tpll_update_prefix(uint8_t pipe, uint8_t prefix);

/** @brief Function for setting the channel to use for the radio.
 *
 * The module must be in an idle state to call this function. As a PTX, the
 * application must wait for an idle state and as a PRX, the application must stop RX
 * before changing the channel. After changing the channel, operation can be resumed.
 *
 * @param[in]   channel                         Channel to use for radio.
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_INVALID_STATE                   If the module is not initialized.
 * @retval  TL_TPLL_ERROR_BUSY                      If the module was not in idle state.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM             If the channel is invalid (larger than 100).
 */
uint32_t tl_tpll_set_rf_channel(int16_t channel);


/**@brief Function for getting the current radio channel.
 *
 * @param[in, out] p_channel    Pointer to the channel data.
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_ERROR_NULL                      If the required parameter was NULL.
 */
uint32_t tl_tpll_get_rf_channel(uint32_t * p_channel);


/**@brief Function for setting the radio output power.
 *
 * @param[in]   tx_output_power    Output power.
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_ERROR_BUSY                      If the function failed because the radio is busy.
 */
uint32_t tl_tpll_set_tx_power(rf_power_level_index_e tx_output_power_idx);


/**@brief Function for setting the packet retransmit delay.
 *
 * @param[in]   delay_us                           Delay between retransmissions. unit in us
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_ERROR_BUSY                      If the function failed because the radio is busy.
 */
uint32_t tl_tpll_set_retransmit_delay(uint16_t delay_us);


/**@brief Function for setting the number of retransmission attempts.
 *
 * @param[in]   count                           Number of retransmissions.
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_ERROR_BUSY                      If the function failed because the radio is busy.
 */
uint32_t tl_tpll_set_retransmit_count(uint16_t count);


/**@brief Function for setting the radio bitrate.
 *
 * @param[in]   bitrate                         Radio bitrate.
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_ERROR_BUSY                      If the function failed because the radio is busy.
 */
uint32_t tl_tpll_set_bitrate(TPLL_BitrateTypeDef bitrate);


/**@brief Function for reusing a packet ID for a specific pipe.
 *
 * The ESB protocol uses a 2-bit sequence number (packet ID) to identify
 * retransmitted packets. By default, the packet ID is incremented for every
 * uploaded packet. Use this function to prevent this and send two different
 * packets with the same packet ID.
 *
 * @param[in]   pipe                            Pipe.
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_ERROR_BUSY                      If the function failed because the radio is busy.
 */
uint32_t tl_tpll_reuse_pid(uint8_t pipe);

/**
 * @brief   Function for set one pipe as a TX pipe.
 *
 * @param   pipe_id                 Pipe to be set as a TX pipe.
 *
 * @retval  TL_TPLL_SUCCESS                If set tx pipe success.
 * @retval  TL_TPLL_ERROR_BUSY             If the function failed because the radio is busy.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM    bad pipe nums
 */
uint32_t tl_tpll_set_tx_pipe(tl_tpll_pipeid_t pipe_id);

/**
 * @brief   Function to check whether the PRX state machine is sending ack
 *
 * @param   none
 *
 * @retval  1                If the PRX state machine is sending ack
 * @retval  0                If the PRX state machine is not sending ack
 *  */
bool tl_tpll_is_SendAck(void);

/**
 * @brief   Function to check whether the PRX state machine is sending ack
 *
 * @param   none
 *
 * @retval  1                If the PRX state machine is tx
 * @retval  0                If the PRX state machine is not tx
 *  */
bool tl_tpll_is_Tx(void);

/**
 * @brief       Check if TX FIFO is full.
 * @param       pipe_id pipe id for which to check.
 * @return      TRUE TX FIFO full or not.
 */
_attribute_ram_code_sec_noinline_ bool tl_tpll_tx_fifo_full(uint8_t pipe);

/**
 * @brief       Function to get the pid value of specific pipe.
 * @param       pipe id for which pid.
 * @return      pid value.
 */
_attribute_ram_code_sec_noinline_ uint8_t tl_tpll_pid_status(uint8_t pipe);

/**@brief Function for removing the oldest entry from the TX buffer.
 *
 * This function will remove the next element scheduled to be sent from the TX FIFO queue.
 * This is useful if you want to skip a packet which was never acknowledged.
 *
 * @retval  TL_TPLL_SUCCESS                     If the operation completed successfully.
 * @retval  TL_TPLL_INVALID_STATE               If the module is not initialized.
 * @retval  TL_TPLL_ERROR_BUFFER_EMPTY          If there are no items in the queue to remove.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_skip_tx(uint8_t pipe);


/**@brief Function for removing the newest entry from the TX buffer.
 *
 * This function will remove the most recently added element from the FIFO queue.
 *
 * @retval  TL_TPLL_SUCCESS                     If the operation completed successfully.
 * @retval  TL_TPLL_INVALID_STATE               If the module is not initialized.
 * @retval  TL_TPLL_ERROR_BUFFER_EMPTY          If there are no items in the queue to remove.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_pop_tx(uint8_t pipe);

/**@brief Function for resetting rf module.
 *
 *
 * @retval  TL_TPLL_SUCCESS                     If the operation completed successfully.
 * @retval  TL_TPLL_INVALID_STATE               If the module is not initialized.
 * @retval  TL_TPLL_ERROR_BUFFER_EMPTY          If there are no items in the queue to remove.
 */
uint32_t tl_tpll_reset_rf(void);


#define smemset     tlk_mem_set
#define smemcpy     tlk_mem_cpy
#define smemcmp     tlk_mem_cmp


#endif /* _TL_TPLL_H_ */

