/********************************************************************************************************
 * @file    tl_tpll.c
 *
 * @brief   This is the source file for 2.4G SDK
 *
 * @author  2.4G Group
 * @date    2024
 *
 * @par     Copyright (c) 2024, Telink Semiconductor (Shanghai) Co., Ltd.
 *          All rights reserved.
 *
 *          The information contained herein is confidential property of Telink
 *          Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *          of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *          Co., Ltd. and the licensee or the terms described here-in. This heading
 *          MUST NOT be removed from this file.
 *
 *          Licensee shall not delete, modify or alter (or permit any third party to delete, modify, or
 *          alter) any information contained herein in whole or in part except as expressly authorized
 *          by Telink semiconductor (shanghai) Co., Ltd. Otherwise, licensee shall be solely responsible
 *          for any claim to the extent arising out of or relating to such deletion(s), modification(s)
 *          or alteration(s).
 *
 *          Licensees are granted free, non-transferable use of the information in this
 *          file under Mutual Non-Disclosure Agreement. NO WARRANTY of ANY KIND is provided.
 *
 *******************************************************************************************************/
#include "driver.h"
#include "types.h"
#include "tl_tpll.h"
#include "common/tl_string.h"
#include "stack/2p4g/tpll/tpll.h"
#define REG_TL_TPLL_LL_CRC_EN                   (0x170000) // crc enable/disable
#define REG_TL_TPLL_LL_CRC_BYTES                (0x170111) // crc bytes.
#define REG_TL_TPLL_LL_PREAMBLE_LENGTH          (0x170002) // preamble length
#if (TPLL_HP_MODE_EN == 1)
#define REG_BASEBAND_ACCESS_CODE_SYNC_THRESHOLD (0x17051a)
#else
#define REG_BASEBAND_ACCESS_CODE_SYNC_THRESHOLD (0x17044e)
#endif
#define REG_BASEBAND_PID_RESET_EN               (0x170201)
#define REG_LINKLAYER_STATE_MACHINE_STATUS      (read_reg8(0x170224) & 0x07)
#define REG_DMA_TX_FIFO_WPTR_PIPE0              (0x170900)
#define REG_DMA_TX_FIFO_RPTR_PIPE0              (0x170901)
#define REG_DMA_TX_FIFO_WPTR_PIPE6              (0x170918)
#define REG_DMA_TX_FIFO_RPTR_PIPE6              (0x170919)
#define REG_DMA_TX_FIFO_WPTR_PIPE7              (0x17091a)
#define REG_DMA_TX_FIFO_RPTR_PIPE7              (0x17091b)
#define REG_DMA_RX_FIFO_WPTR                    (0x1708f4)
#define REG_DMA_RX_FIFO_RPTR                    (0x1708f5)

#define TL_TPLL_ADDR_MIN_LEN                    3   /** Minimum address length */
#define TL_TPLL_ADDR_MAX_LEN                    5   /** Maximum address length */
#define TL_TPLL_PIPE_COUNT                      8   /** Maximum address length */
#define TL_TX_PAYLOAD_LEN  16

_attribute_data_retention_sec_ static tl_tpll_event_handler_t  m_event_handler;
_attribute_data_retention_sec_ bool                            m_tl_tpll_initialized = false;
//_attribute_data_retention_sec_ unsigned int                    m_channel;
//_attribute_data_retention_sec_ unsigned int                    pipe_pid[TL_TPLL_PIPE_COUNT]                                   = {0};
_attribute_data_retention_sec_ unsigned int                    pid_status = 0;
_attribute_data_retention_sec_ tl_tpll_config_t                m_config_local;

static unsigned char rxpara_tpll_flag = 1;
/* valid_chn_table: bitmask for valid channels (0-80), each bit represents one channel */
static u32 hs_valid_chn_table[3] = {
    0x000007FF,   /* bits 0-31:  channels 0-10 valid */
    0x1FFFFFFC,   /* bits 32-63: channels 34-60 valid (bits 2-28) */
    0x00000000,   /* bits 64-95: channels 64-80 none valid */
};

/* valid_chn_table: bitmask for valid channels (0-80), each bit represents one channel */
static u32 normal_valid_chn_table[3] = {
    0x03800003,   /* bits 0-31:  channels 0, 1, 23, 24, 25 valid */
    0x00038000,   /* bits 32-63: channels 47, 48, 49 valid (bits 15, 16, 17) */
    0x00000380,   /* bits 64-95: channels 71, 72, 73 valid (bits 7, 8, 9) */
};


extern unsigned char ptx_buffer[TL_TPLL_PIPE_COUNT * TPLL_TX_FIFO_SIZE * TPLL_TX_FIFO_NUM];
extern unsigned char rx_buf[TPLL_PIPE_RX_FIFO_SIZE * TPLL_PIPE_RX_FIFO_NUM];

inline u8 hs_chn_in_rxtxlf_table(u8 chn){
    return (chn <= 80) ? ((hs_valid_chn_table[chn >> 5] >> (chn & 0x1F)) & 1) : 0;
}

inline u8 normal_chn_in_rxtxlf_table(u8 chn){
    return (chn <= 80) ? ((normal_valid_chn_table[chn >> 5] >> (chn & 0x1F)) & 1) : 0;
}

/** @brief Macro for verifying statement to be true. It will cause the exterior function to return
 *        err_code if the statement is not true.
 *
 * @param[in]   statement   Statement to test.
 * @param[in]   err_code    Error value to return if test was invalid.
 *
 * @retval      nothing, but will cause the exterior function to return @p err_code if @p statement
 *              is false.
 */
#define VERIFY_TRUE(statement, err_code) \
    do                                   \
    {                                    \
        if (!(statement))                \
        {                                \
            return err_code;             \
        }                                \
    } while (0)

/** @brief Macro for verifying statement to be false. It will cause the exterior function to return
 *        err_code if the statement is not false.
 *
 * @param[in]   statement   Statement to test.
 * @param[in]   err_code    Error value to return if test was invalid.
 *
 * @retval      nothing, but will cause the exterior function to return @p err_code if @p statement
 *              is true.
 */
#define VERIFY_FALSE(statement, err_code) \
    do                                    \
    {                                     \
        if ((statement))                  \
        {                                 \
            return err_code;              \
        }                                 \
    } while (0)

/** @brief Macro for verifying that the module is initialized. It will cause the exterior function to
 *        return if not.
 *
 * @param[in] param  The variable to check if is NULL.
 */
#define VERIFY_PARAM_NOT_NULL(param) VERIFY_FALSE(((param) == NULL), TL_TPLL_ERROR_NULL)

#define VERIFY_PAYLOAD_LENGTH(p)                                       \
    do                                                                 \
    {                                                                  \
        if (p->length == 0 || p->length > TL_TPLL_MAX_PAYLOAD_LENGTH) \
        {                                                              \
            return TL_TPLL_ERROR_INVALID_LENGTH;                          \
        }                                                              \
    } while (0)

typedef struct
{
    uint8_t base_address_0[TL_TPLL_ADDR_MAX_LEN - 1];
    uint8_t base_address_1[TL_TPLL_ADDR_MAX_LEN - 1];
    uint8_t pipe_prefixes[TL_TPLL_PIPE_COUNT];
    uint8_t addr_length;
    uint8_t pipe_nums;
    uint8_t rx_pipes_enabled;       /**< Bitfield for enabled pipes. */
    uint8_t rf_channel;             /**< Channel to use (must be between 0 and 100). */
} tl_tpll__address_t;

_attribute_data_retention_sec_ tl_tpll__address_t m_tl_tpll_addr = TL_TPLL_ADDR_DEFAULT;
_attribute_data_retention_sec_ uint8_t m_addr_0[TL_TPLL_ADDR_MAX_LEN] = {0};
_attribute_data_retention_sec_ uint8_t m_addr_1[TL_TPLL_ADDR_MAX_LEN] = {0};

//crc_config
_attribute_data_retention_sec_ TPLL_CrcConfig_t TPLL_CrcConfig = {
    .init_value    = 0xffffffff,
    .poly          = 0x00001021,
    .xor_out       = 0,
    .byte_order    = 1,
    .start_cal_pos = 0,
    .len           = 2,
};

/**
 *  @note  The TPLL_GenericHeader_t function to set generic packet header,
 *         You need to configure the size in bit and the value of h0,h1 and payload len.
 *         You also can configure the start location of pid and no ack bit if you need.
 *         only generic mode need to configure this!!!
*/
TPLL_GenericHeader_t TPLL_GenericHeader = {
    .h0_size         = 5,
    .h1_size         = 4,
    .length_size     = 15,
    .h0_val          = 26, //0x22 to enable no ack
    .h1_val          = 14,
    .length_val      = TL_TX_PAYLOAD_LEN,
    .pid_start_bit   = 0,
    .noack_start_bit = 2,
};


/**@brief Function for initializing the Telink primary link layer module.
 *
 * @param  p_config     Parameters for initializing the module.
 *
 * @retval  TL_TPLL_SUCCESS             If initialization was successful.
 * @retval  TL_TPLL_ERROR_NULL           If the @p p_config argument was NULL.
 * @retval  TL_TPLL_ERROR_BUSY          If the function failed because the radio is busy.
 */

_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_init(tl_tpll_config_t const *p_config)
{
    if (p_config == NULL)
        return TL_TPLL_ERROR_NULL;

    smemcpy(&m_config_local, p_config, sizeof(tl_tpll_config_t));

    // Not sure if it's needed.
    if (p_config->deepRetWakeUp) {
    } else {
        TPLL_set_lowest_ret_ldo_voltage();
    }

//    m_event_handler = p_config->event_handler;

    TPLL_SetFormatMode(p_config->protocol);
    tl_tpll_set_bitrate(p_config->bitrate);
    tl_tpll_set_tx_power(p_config->tx_output_power_idx);

    if (p_config->fast_settle_en) {
        TPLL_fast_settle_init(TX_SETTLE_TIME_15US,RX_SETTLE_TIME_15US);

        // In order to interoperate with the N* SDK,
        // the PRX side needs to wait 80 µs before sending the ACK when fast settling is enabled.
        if (p_config->mode == TL_TPLL_MODE_PRX) {
            TPLL_TxWaitSet(120);
        }
    } else {
        TPLL_TxSettleSet(149);
        TPLL_RxSettleSet(85);
    }

    TPLL_DmaInit(rx_buf, ptx_buffer);

    if (p_config->mode == TL_TPLL_MODE_PTX)
    {
        TPLL_ModeSet(TPLL_MODE_PTX);
        TPLL_SetAutoRetry(p_config->retransmit_count, p_config->retransmit_delay);
    }
    else if (p_config->mode == TL_TPLL_MODE_PRX)
    {
        TPLL_ModeSet(TPLL_MODE_PRX);
    }

    TPLL_EnableNoAck(1);
    TPLL_Set_Local_Ack_Bit(0);
    TPLL_Preamble_Set(4);
    TPLL_CrcSet(TPLL_CrcConfig);

    if(p_config->bitrate != TPLL_BITRATE_4MBPS && p_config->bitrate != TPLL_BITRATE_6MBPS)
    {
        if (p_config->mode == TL_TPLL_MODE_PTX) {
            TPLL_tx_mi_0p32_aura_mode();
        } else if (p_config->mode == TL_TPLL_MODE_PRX) {
            TPLL_rx_mi_0p32_aura_mode();
        }
    }

    TPLL_set_RXT_MaxLen(TL_TPLL_MAX_PAYLOAD_LENGTH);

    //configure irq
#if defined(MCU_CORE_TL322X_N22)
    clic_interrupt_enable(IRQ_ZB_RT);
#elif !defined(MCU_CORE_TL322X_N22)
    plic_interrupt_enable(IRQ_ZB_RT);
#endif
    rf_clr_irq_mask(FLD_RF_IRQ_ALL);
    rf_set_irq_mask(p_config->irq_mask);
    core_interrupt_enable();

    m_tl_tpll_initialized = true;

    return TL_TPLL_SUCCESS;
}


/**@brief Function for suspending the Telink primary link layer module.
 *
 * Calling this function stops ongoing communications without changing the queues.
 *
 * @retval  TL_TPLL_SUCCESS             If Telink primary link layer was suspended.
 * @retval  TL_TPLL_ERROR_BUSY          If the function failed because the radio is busy.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_suspend(void)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);

    rf_set_tx_rx_off(); //STOP_RF_STATE_MACHINE;

    return TL_TPLL_SUCCESS;
}


/**@brief Function for disabling the Telink primary link layer module.
 *
 * Calling this function disables the TL_TPLL module immediately.
 * Doing so might stop ongoing communications.
 *
 * @retval  TL_TPLL_SUCCESS             If Telink primary link layer was disabled.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_disable(void)
{
    write_reg8(REG_BB_LL_BASE_ADDR, 0x80);
//    TPLL_ModeStop();

    return TL_TPLL_SUCCESS;
}


/**@brief Function for checking if the Telink primary link layer module is idle.
 *
 * @retval true                     If the module is idle.
 * @retval false                    If the module is busy.
 */
_attribute_ram_code_sec_noinline_ bool tl_tpll_is_idle(void)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);
    return TL_TPLL_SUCCESS;
}


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
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_write_payload(tl_tpll_payload_t const * p_payload)
{
//    uint8_t wptr, rptr, pipe_id;
    VERIFY_TRUE(m_tl_tpll_initialized, TL_TPLL_INVALID_STATE);
    VERIFY_PARAM_NOT_NULL(p_payload);
    VERIFY_PAYLOAD_LENGTH(p_payload);
    VERIFY_TRUE(p_payload->pipe < TL_TPLL_PIPE_COUNT, TL_TPLL_ERROR_INVALID_PARAM);

    if (m_config_local.mode == TL_TPLL_MODE_PTX) {
        TPLL_ClosePipe(TPLL_PIPE_ALL);
        TPLL_OpenPipe(p_payload->pipe);
        TPLL_SetTXPipe(p_payload->pipe);
    }

    TPLL_WriteTxPayload(p_payload->pipe, ptx_buffer, p_payload->data, p_payload->length);

    if (m_config_local.mode == TL_TPLL_MODE_PTX && m_config_local.tx_mode == TL_TPLL_TXMODE_AUTO &&
            TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS) {
        tl_tpll_start_tx(); // TX trigger
    }

    return TL_TPLL_SUCCESS;
}

_attribute_data_retention_sec_ volatile unsigned char *rx_packet      = NULL;
/**@brief Function for reading an RX payload.
 *
 * @param[in,out]   p_payload   The pointer to the structure that contains information and state of the payload.
 *
 * @retval  TL_TPLL_SUCCESS                     If the data was read successfully.
 * @retval  TL_TPLL_ERROR_NULL                  If the required parameter was NULL.
 * @retval  TL_TPLL_INVALID_STATE               If the module is not initialized.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_read_rx_payload(tl_tpll_payload_t * p_payload)
{
    VERIFY_PARAM_NOT_NULL(p_payload);

    VERIFY_TRUE(m_tl_tpll_initialized == true, TL_TPLL_INVALID_STATE);

    rx_packet = TPLL_GetRxPacket(rx_buf);
    TPLL_ReadRxPayload_t ReadRxPayload  = TPLL_ReadRxPayload((unsigned char *)&(p_payload->data[0]), rx_buf);


    p_payload->crc = TPLL_IsCrcValid(rx_packet);

    p_payload->noack   = ReadRxPayload.noack;
    p_payload->length = ReadRxPayload.rx_payload_len;

    VERIFY_FALSE(p_payload->length > TL_TPLL_MAX_PAYLOAD_LENGTH, TL_TPLL_ERROR_INVALID_PARAM);

    p_payload->pipe   = ReadRxPayload.rx_pipe;
    p_payload->rssi   = ReadRxPayload.rx_rssi;
    p_payload->pid   = ReadRxPayload.pid;
    p_payload->rx_timestamp = ReadRxPayload.rx_timestamp;
    return TL_TPLL_SUCCESS;
}

/**
 * @brief   Check for TX FIFO empty.
 *
 * @details Use this function to check if TX FIFO is empty.
 *
 * @param[in]   pipe_id             specify the pipe
 *
 * @return  TX FIFO empty bit
 * @retval  FALSE TX FIFO NOT empty
 * @retval  RUE TX FIFO empty
 *
 */
_attribute_ram_code_sec_noinline_ uint8_t tx_fifo_empty(uint8_t pipe_id)
{
    if (pipe_id < 6) {
        return (read_reg8((REG_DMA_TX_FIFO_WPTR_PIPE0 + (pipe_id << 1)))) ==
                (read_reg8((REG_DMA_TX_FIFO_RPTR_PIPE0 + (pipe_id << 1))));
    } else if (pipe_id == 6) {
        return (read_reg8(REG_DMA_TX_FIFO_WPTR_PIPE6) == read_reg8(REG_DMA_TX_FIFO_RPTR_PIPE6));
    } else {
        return (read_reg8(REG_DMA_TX_FIFO_WPTR_PIPE7) == read_reg8(REG_DMA_TX_FIFO_RPTR_PIPE7));
    }
}

/**@brief Function for starting transmission.
 *
 * @retval  TL_TPLL_SUCCESS                     If the TX started successfully.
 * @retval  TL_TPLL_ERROR_BUFFER_EMPTY          If the TX did not start because the FIFO buffer is empty.
 * @retval  TL_TPLL_ERROR_BUSY                  If the function failed because the radio is busy.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_start_tx(void)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);

    uint8_t ret = false;
    for (int pipe_id = 0; pipe_id < TL_TPLL_PIPE_COUNT; pipe_id++)
    {
        if (!tx_fifo_empty(pipe_id))
            ret = true;
    }
    if (!ret) {
        return TL_TPLL_ERROR_BUFFER_EMPTY;
    }

    write_reg8(REG_BB_LL_BASE_ADDR, 0x80);     // stop first

    if (TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS) {
//      debug_tpll_trigger[7]++;
        reg_rf_ll_cmd_schedule = stimer_get_tick;
//      reg_rf_ll_ctrl3 |= FLD_RF_R_CMD_SCHEDULE_EN;
        write_reg8(REG_BB_LL_BASE_ADDR, 0x83); //trig
//                      while(TPLL_STATE_MACHINE_STATUS_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS);

        return TL_TPLL_SUCCESS;
    }
//    debug_tpll_trigger[8]++;
    return TL_TPLL_ERROR_BUSY;
}


/**@brief Function for starting to transmit data from the FIFO buffer.
 *
 * @retval  TL_TPLL_SUCCESS                     If the transmission was started successfully.
 * @retval  TL_TPLL_ERROR_BUSY                  If the function failed because the radio is busy.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_start_rx(void)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);
    write_reg8(REG_BB_LL_BASE_ADDR, 0x80);     // stop first

    if( m_config_local.bitrate==TPLL_BITRATE_4MBPS || m_config_local.bitrate==TPLL_BITRATE_6MBPS)
    {
        if (rxpara_tpll_flag == 1)
        {
        #if (1)
           rf_set_rxpara();
        #endif
           rxpara_tpll_flag = 0;
        }
     //2400~2410 2434~2460
       if (hs_chn_in_rxtxlf_table(m_tl_tpll_addr.rf_channel)) {
        #if (1)
           rf_ldot_ldo_rxtxlf_bypass_en();
        #endif
       } else {
       #if (1)
        rf_ldot_ldo_rxtxlf_bypass_dis();
       #endif
       }
    }
    else if(m_config_local.bitrate==TPLL_BITRATE_2MBPS)
    {
        if (rxpara_tpll_flag == 1)
        {
        #if (1)
           rf_set_rxpara();
        #endif
           rxpara_tpll_flag = 0;
        }
     //2400, 2401, 2423, 2424, 2425, 2447, 2448, 2449, 2471, 2472, 2473
       if (normal_chn_in_rxtxlf_table(m_tl_tpll_addr.rf_channel)) {
        #if (1)
           rf_ldot_ldo_rxtxlf_bypass_en();
        #endif
       } else {
       #if (1)
        rf_ldot_ldo_rxtxlf_bypass_dis();
       #endif
       }
    }

    if (TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS) {
        write_reg8(REG_BB_LL_BASE_ADDR, 0x84); //trig
//        delay_us(1);
//        write_reg8(REG_BASEBAND_PID_RESET_EN,0);
        return TL_TPLL_SUCCESS;
    }

    return TL_TPLL_ERROR_BUSY;
}


_attribute_ram_code_sec_noinline_ void tl_tpll_start_rx_2360Mhz(void)
{

}


/** @brief Function for stopping data reception.
 *
 * @retval  TL_TPLL_SUCCESS                     If data reception was stopped successfully.
 * @retval  TL_TPLL_ERROR_NOT_IN_RX_MODE    If the function failed because the module is not in RX mode.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_stop_rx(void)
{
    VERIFY_FALSE(TL_TPLL_STATE_TX_SETTLE == REG_LINKLAYER_STATE_MACHINE_STATUS ||
                 TL_TPLL_STATE_TX == REG_LINKLAYER_STATE_MACHINE_STATUS ||
                 TL_TPLL_STATE_TX_WAIT == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_NOT_IN_RX_MODE);

//    TPLL_ModeSet(TPLL_MODE_PTX);
    write_reg8(REG_BB_LL_BASE_ADDR, 0x80);

    return TL_TPLL_SUCCESS;
}


/**@brief Function for removing remaining items from the TX buffer.
 *
 * This function clears the TX FIFO buffer.
 *
 * @retval  TL_TPLL_SUCCESS                     If pending items in the TX buffer were successfully cleared.
 * @retval  TL_TPLL_INVALID_STATE               If the module is not initialized.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_flush_tx(void)
{
    VERIFY_TRUE(m_tl_tpll_initialized == true, TL_TPLL_INVALID_STATE);

    for (int pipe_id = 0; pipe_id < TL_TPLL_PIPE_COUNT -2; pipe_id++) {
        write_reg8((REG_DMA_TX_FIFO_RPTR_PIPE0 + pipe_id), 0x80);
    }

    //pipe6,7 not near with pipe 5
    write_reg8((REG_DMA_TX_FIFO_RPTR_PIPE6), 0x80);
    write_reg8((REG_DMA_TX_FIFO_RPTR_PIPE7), 0x80);

    return TL_TPLL_SUCCESS;
}

/**@brief Function for removing remaining items from the RX buffer.
 *
 * @retval  TL_TPLL_SUCCESS                     If the pending items in the RX buffer were successfully cleared.
 * @retval  TL_TPLL_INVALID_STATE               If the module is not initialized.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_flush_rx(void)
{
    VERIFY_TRUE(m_tl_tpll_initialized == true, TL_TPLL_INVALID_STATE);

    write_reg8(REG_DMA_RX_FIFO_RPTR, 0x80);

    return TL_TPLL_SUCCESS;
}


/**@brief Function for setting the length of the address.
 *
 * @param[in]       length              Length of the Telink primary link layer address (in bytes).
 *
 * @retval  TL_TPLL_SUCCESS                      If the address length was set successfully.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM          If the address length was invalid.
 * @retval  TL_TPLL_ERROR_BUSY                   If the function failed because the radio is busy.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_set_address_length(uint8_t length)
{
    if (length < TL_TPLL_ADDR_MIN_LEN || length > TL_TPLL_ADDR_MAX_LEN)
        return TL_TPLL_ERROR_INVALID_PARAM;

    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);

    rf_set_access_code_len(length);
    uint8_t val = read_reg8(REG_BASEBAND_ACCESS_CODE_SYNC_THRESHOLD);
    val = (val & 0xc0) | ((length * 8 - 2) & 0x3f);
    write_reg8(REG_BASEBAND_ACCESS_CODE_SYNC_THRESHOLD, val); //set the length of address code sync window

    return TL_TPLL_SUCCESS;
}

/**
 * @brief       Get the width of the address.
 * @param       none.
 * @return      Width of the TPLL address (in bytes).
 */
uint8_t tl_tpll_get_address_length(void)
{
    return read_reg8(0x170005) & 0x07;
}

/**@brief Function for setting the base address for pipe 0.
 *
 * @param[in]       p_addr      Pointer to the address data.
 *
 * @retval  TL_TPLL_SUCCESS                     If the base address was set successfully.
 * @retval  TL_TPLL_ERROR_BUSY                  If the function failed because the radio is busy.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM         If the function failed because the address given was too close to a zero address.
 * @retval  TL_TPLL_ERROR_NULL                  If the required parameter was NULL.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_set_base_address_0(uint8_t const * p_addr)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);
    VERIFY_PARAM_NOT_NULL(p_addr);

    m_tl_tpll_addr.addr_length = tl_tpll_get_address_length();
    smemcpy(m_tl_tpll_addr.base_address_0, &p_addr[0], m_tl_tpll_addr.addr_length - 1);

    m_addr_0[0] = m_tl_tpll_addr.pipe_prefixes[0];
    smemcpy(&m_addr_0[1], &p_addr[0], m_tl_tpll_addr.addr_length - 1);

    VERIFY_FALSE(m_addr_0 == 0x0, TL_TPLL_ERROR_INVALID_PARAM);

    if (false == m_tl_tpll_initialized) {
        TPLL_ClosePipe(TPLL_PIPE_ALL);
    }
    TPLL_SetAddress(TPLL_PIPE0, m_addr_0);
//    TPLL_OpenPipe(TPLL_PIPE0);

    return TL_TPLL_SUCCESS;
}


/**@brief Function for setting the base address for pipe 1 to pipe 7.
 *
 * @param[in]       p_addr      Pointer to the address data.
 *
 * @retval  TL_TPLL_SUCCESS                     If the base address was set successfully.
 * @retval  TL_TPLL_ERROR_BUSY                  If the function failed because the radio is busy.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM         If the function failed because the address given was too close to a zero address.
 * @retval  TL_TPLL_ERROR_NULL                  If the required parameter was NULL.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_set_base_address_1(uint8_t const * p_addr)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);
    VERIFY_PARAM_NOT_NULL(p_addr);

    m_tl_tpll_addr.addr_length = tl_tpll_get_address_length();
    smemcpy(m_tl_tpll_addr.base_address_1, &p_addr[0], m_tl_tpll_addr.addr_length - 1);

    m_addr_1[0] = m_tl_tpll_addr.pipe_prefixes[1];
    smemcpy(&m_addr_1[1], &p_addr[0], m_tl_tpll_addr.addr_length - 1);

//    VERIFY_FALSE(m_addr_1 == 0x0, TL_TPLL_ERROR_INVALID_PARAM);

    if (false == m_tl_tpll_initialized) {
        TPLL_ClosePipe(TPLL_PIPE_ALL);
    }
    TPLL_SetAddress(TPLL_PIPE1, m_addr_1);
//    TPLL_OpenPipe(TPLL_PIPE1);

    return TL_TPLL_SUCCESS;
}


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
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_set_prefixes(uint8_t const * p_prefixes, uint8_t num_pipes)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);
    VERIFY_PARAM_NOT_NULL(p_prefixes);
    VERIFY_TRUE(num_pipes <= TL_TPLL_PIPE_COUNT, TL_TPLL_ERROR_INVALID_PARAM);

    smemcpy(m_tl_tpll_addr.pipe_prefixes, p_prefixes, num_pipes);
    m_tl_tpll_addr.pipe_nums = num_pipes;

    for (int i = 0; i < m_tl_tpll_addr.pipe_nums; i++) {
        switch (i)
            {
            case TPLL_PIPE0:
                m_addr_0[0] = m_tl_tpll_addr.pipe_prefixes[0];
                VERIFY_FALSE(m_addr_0 == 0x0, TL_TPLL_ERROR_INVALID_PARAM);
                TPLL_SetAddress(TPLL_PIPE0, m_addr_0);
                break;
            case TPLL_PIPE1:
                m_addr_1[0] = m_tl_tpll_addr.pipe_prefixes[1];
                VERIFY_FALSE(m_addr_1 == 0x0, TL_TPLL_ERROR_INVALID_PARAM);
                TPLL_SetAddress(TPLL_PIPE1, m_addr_1);
                break;
            case TPLL_PIPE2:
            case TPLL_PIPE3:
            case TPLL_PIPE4:
            case TPLL_PIPE5:
            case TPLL_PIPE6:
            case TPLL_PIPE7:
                m_addr_1[0] = m_tl_tpll_addr.pipe_prefixes[i];
                VERIFY_FALSE(m_addr_1 == 0x0, TL_TPLL_ERROR_INVALID_PARAM);
                TPLL_SetAddress(i, m_addr_1);
                break;
            default:
                break;
        }
//        TPLL_OpenPipe(i);
    }

    return TL_TPLL_SUCCESS;
}


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
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_enable_pipes(uint8_t enable_mask)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);
    TPLL_ClosePipe(TPLL_PIPE_ALL);

    if (enable_mask == 0x01) {
        TPLL_OpenPipe(TPLL_PIPE0);
    } else if (enable_mask == 0x02){
        TPLL_OpenPipe(TPLL_PIPE1);
    } else if (enable_mask == 0x04) {
        TPLL_OpenPipe(TPLL_PIPE2);
    } else  if (enable_mask == 0x08) {
        TPLL_OpenPipe(TPLL_PIPE3);
    } else if (enable_mask == 0x10) {
        TPLL_OpenPipe(TPLL_PIPE4);
    } else if (enable_mask == 0x20) {
        TPLL_OpenPipe(TPLL_PIPE5);
    } else if (enable_mask == 0x40) {
        TPLL_OpenPipe(TPLL_PIPE6);
    } else if (enable_mask == 0x80) {
        TPLL_OpenPipe(TPLL_PIPE7);
    } else if (enable_mask == 0xff) {
        TPLL_OpenPipe(TPLL_PIPE_ALL);
    } else {
        return TL_TPLL_ERROR_INVALID_PARAM;
    }
    m_tl_tpll_addr.rx_pipes_enabled = enable_mask;

    return TL_TPLL_SUCCESS;
}


/**@brief Function for updating the prefix for a pipe.
 *
 * @param   pipe    Pipe for which to set the prefix.
 * @param   prefix  Prefix to set for the pipe.
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_ERROR_BUSY                      If the function failed because the radio is busy.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM             If the given pipe number was invalid or if the address given was too close to a zero address.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_update_prefix(uint8_t pipe, uint8_t prefix)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);
    VERIFY_TRUE(pipe < TL_TPLL_PIPE_COUNT, TL_TPLL_ERROR_INVALID_PARAM);

    m_tl_tpll_addr.pipe_prefixes[pipe] = prefix;

    switch (pipe)
        {
        case TPLL_PIPE0:
            m_addr_0[0] = m_tl_tpll_addr.pipe_prefixes[0];
            VERIFY_FALSE(m_addr_0 == 0x0, TL_TPLL_ERROR_INVALID_PARAM);
            TPLL_SetAddress(TPLL_PIPE0, m_addr_0);
            break;
        case TPLL_PIPE1:
            m_addr_1[0] = m_tl_tpll_addr.pipe_prefixes[1];
            VERIFY_FALSE(m_addr_1 == 0x0, TL_TPLL_ERROR_INVALID_PARAM);
            TPLL_SetAddress(TPLL_PIPE1, m_addr_1);
            break;
        case TPLL_PIPE2:
        case TPLL_PIPE3:
        case TPLL_PIPE4:
        case TPLL_PIPE5:
        case TPLL_PIPE6:
        case TPLL_PIPE7:
            m_addr_1[0] = m_tl_tpll_addr.pipe_prefixes[pipe];
            VERIFY_FALSE(m_addr_1 == 0x0, TL_TPLL_ERROR_INVALID_PARAM);
            TPLL_SetAddress(pipe, m_addr_1);
            break;
        default:
            break;
        }

//    TPLL_OpenPipe(pipe);

    return TL_TPLL_SUCCESS;
}

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
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_set_rf_channel(int16_t channel)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);
    VERIFY_TRUE(channel <= 80, TL_TPLL_ERROR_INVALID_PARAM);

    m_tl_tpll_addr.rf_channel = channel;
    TPLL_SetRFChannel(channel);

    return TL_TPLL_SUCCESS;
}


/**@brief Function for getting the current radio channel.
 *
 * @param[in, out] p_channel    Pointer to the channel data.
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_ERROR_NULL                      If the required parameter was NULL.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_get_rf_channel(uint32_t * p_channel)
{
    VERIFY_PARAM_NOT_NULL(p_channel);

    *p_channel = m_tl_tpll_addr.rf_channel;

    return TL_TPLL_SUCCESS;
}


/**@brief Function for setting the radio output power.
 *
 * @param[in]   tx_output_power    Output power.
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_ERROR_BUSY                      If the function failed because the radio is busy.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_set_tx_power(rf_power_level_index_e tx_output_power_idx)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);

    rf_set_power_level_index(tx_output_power_idx);

    return TL_TPLL_SUCCESS;
}

#define REG_LINKLAYER_AUTO_RETRY_TIMES (0x170214)
#define REG_LINKLAYER_AUTO_RETRY_DELAY (0x170210)
/**@brief Function for setting the packet retransmit delay.
 *
 * @param[in]   delay                           Delay between retransmissions.
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_ERROR_BUSY                      If the function failed because the radio is busy.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_set_retransmit_delay(uint16_t delay_us)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);

    delay_us &= 0x0fff;
    unsigned short tmp = read_reg16(REG_LINKLAYER_AUTO_RETRY_DELAY);
    tmp &= 0xf000;
    tmp |= delay_us;
    write_reg16(REG_LINKLAYER_AUTO_RETRY_DELAY, tmp);

    return TL_TPLL_SUCCESS;
}


/**@brief Function for setting the number of retransmission attempts.
 *
 * @param[in]   count                           Number of retransmissions.
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_ERROR_BUSY                      If the function failed because the radio is busy.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_set_retransmit_count(uint16_t count)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);

    count &= 0x0f; //accommodate with another chip
    write_reg8(REG_LINKLAYER_AUTO_RETRY_TIMES, count);

    return TL_TPLL_SUCCESS;
}


/**@brief Function for setting the radio bitrate.
 *
 * @param[in]   bitrate                         Radio bitrate.
 *
 * @retval  TL_TPLL_SUCCESS                         If the operation completed successfully.
 * @retval  TL_TPLL_ERROR_BUSY                      If the function failed because the radio is busy.
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_set_bitrate(TPLL_BitrateTypeDef bitrate)
{
//    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);

//    if (bitrate == TL_TPLL_BITRATE_1MBPS)
//    {
//        TPLL_SetFormatMode(TPLL_MODE_GENERIC_FORMAT);
//        TPLL_SetBitrate(TPLL_BITRATE_1MBPS);
//    }

    return TPLL_SetBitrate(bitrate);
}


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
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_reuse_pid(uint8_t pipe)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);

    uint8_t bitoffset = pipe * 2;
    uint8_t curr_pid = (read_reg16(0x170222) >> bitoffset) & 0x03;
    curr_pid = (curr_pid + 3) % 4;
//    write_reg16(0x170222, read_reg16(0x170222) & (~(0x03 << bitoffset)));
//    write_reg16(0x170222, read_reg16(0x170222) | (curr_pid << bitoffset));
    write_reg8(REG_BASEBAND_PID_RESET_EN, 1 << pipe);
    rf_set_prx_init_pid(pid_status);
    return TL_TPLL_SUCCESS;
}

/**
 * @brief   Function for set one pipe as a TX pipe.
 *
 * @param   pipe_id                 Pipe to be set as a TX pipe.
 *
 * @retval  TL_TPLL_SUCCESS                If set tx pipe success.
 * @retval  TL_TPLL_ERROR_BUSY             If the function failed because the radio is busy.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM    bad pipe nums
 */
_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_set_tx_pipe(tl_tpll_pipeid_t pipe_id)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == read_reg8(REG_LINKLAYER_STATE_MACHINE_STATUS), TL_TPLL_ERROR_BUSY);

    VERIFY_TRUE(pipe_id <= TL_TPLL_PIPE7, TL_TPLL_ERROR_INVALID_PARAM);
    // enable PTX in the specified pipe
    unsigned char tmp = read_reg8(0x170215);
    tmp &= 0xf8;

    tmp |= pipe_id;

    tmp |= 0x10;
    write_reg8(0x170215, tmp);

    return TL_TPLL_SUCCESS;
}

_attribute_ram_code_sec_noinline_ bool tl_tpll_is_SendAck(void)
{
    return (TL_TPLL_STATE_TX_WAIT == REG_LINKLAYER_STATE_MACHINE_STATUS)? 1: 0;
}


_attribute_ram_code_sec_noinline_ bool tl_tpll_is_RxAck(void)
{
    return false;
}


_attribute_ram_code_sec_noinline_ bool tl_tpll_is_Tx(void)
{
    return TL_TPLL_STATE_TX == REG_LINKLAYER_STATE_MACHINE_STATUS;
}

_attribute_ram_code_sec_noinline_ uint8_t tl_tpll_pid_status(uint8_t pipe)
{
    VERIFY_TRUE(TL_TPLL_STATE_IDLE == REG_LINKLAYER_STATE_MACHINE_STATUS, TL_TPLL_ERROR_BUSY);

    uint8_t bitoffset = pipe * 2;
    uint8_t curr_pid = (read_reg16(0x170222) >> bitoffset) & 0x03;
    return curr_pid;
}

_attribute_ram_code_sec_noinline_ bool tl_tpll_tx_fifo_full(uint8_t pipe)
{
    VERIFY_TRUE(pipe <= TL_TPLL_PIPE7, TL_TPLL_ERROR_INVALID_PARAM);
    if (TPLL_TxFifoFull(pipe)) {
        return true;
    } else {
        return false;
    }

}

_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_skip_tx(uint8_t pipe)
{
//    VERIFY_TRUE(TL_TPLL_STATE_IDLE == read_reg8(REG_LINKLAYER_STATE_MACHINE_STATUS), TL_TPLL_ERROR_BUSY);
    VERIFY_TRUE(pipe <= TL_TPLL_PIPE7, TL_TPLL_ERROR_INVALID_PARAM);
    if (TPLL_TxFifoEmpty(pipe)) {
        return TL_TPLL_ERROR_BUFFER_EMPTY;
    }
    if (pipe >=0 && pipe <=5) {
        write_reg8((REG_DMA_TX_FIFO_RPTR_PIPE0 + pipe * 2), 0x40);
    } else if (pipe == TL_TPLL_PIPE6){
        write_reg(REG_DMA_TX_FIFO_RPTR_PIPE6, 0x40);
    } else if (pipe == TL_TPLL_PIPE7) {
        write_reg(REG_DMA_TX_FIFO_RPTR_PIPE7, 0x40);
    }
    return TL_TPLL_SUCCESS;
}

_attribute_ram_code_sec_noinline_ uint32_t tl_tpll_pop_tx(uint8_t pipe)
{
//    VERIFY_TRUE(TL_TPLL_STATE_IDLE == read_reg8(REG_LINKLAYER_STATE_MACHINE_STATUS), TL_TPLL_ERROR_BUSY);
    VERIFY_TRUE(pipe <= TL_TPLL_PIPE7, TL_TPLL_ERROR_INVALID_PARAM);

    uint8_t        wptr, rptr;
    if(pipe < 6){ //pipe0~pipe5
        wptr = read_reg8(REG_DMA_TX_FIFO_WPTR_PIPE0 + (pipe << 1));
        rptr = read_reg8(REG_DMA_TX_FIFO_RPTR_PIPE0 + (pipe << 1));
    }
    else if(pipe == 6){ //pipe6
        wptr = read_reg8(REG_DMA_TX_FIFO_WPTR_PIPE6);
        rptr = read_reg8(REG_DMA_TX_FIFO_RPTR_PIPE6);
    }
    else if(pipe == 7){ //pipe7
        wptr = read_reg8(REG_DMA_TX_FIFO_WPTR_PIPE7);
        rptr = read_reg8(REG_DMA_TX_FIFO_RPTR_PIPE7);
    }
    if ((wptr% TPLL_TX_FIFO_NUM) == (rptr % TPLL_TX_FIFO_NUM)) { // if the tx-fifo is empty, return  immediately
        return TL_TPLL_ERROR_BUFFER_EMPTY;
    }
    if (wptr == 0) {
        wptr = 31;
    } else {
        wptr--;
    }

    if(pipe < 6){
        write_reg8(REG_DMA_TX_FIFO_WPTR_PIPE0 + (pipe << 1), wptr);
    }
    else if(pipe == 6){ //pipe6
        write_reg8(REG_DMA_TX_FIFO_WPTR_PIPE6, wptr);
    }
    else if(pipe == 7){ //pipe7
        write_reg8(REG_DMA_TX_FIFO_WPTR_PIPE7, wptr);
    }

    return TPLL_SUCCESS;
}

uint32_t tl_tpll_reset_rf(void)
{
    // reset rf
    dma_reset();
    rf_reset_register_value();
    return TPLL_SUCCESS;
}
