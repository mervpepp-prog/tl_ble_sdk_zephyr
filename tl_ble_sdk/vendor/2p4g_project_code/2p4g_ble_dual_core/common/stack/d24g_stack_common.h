/********************************************************************************************************
 * @file    d24g_stack_common.h
 *
 * @brief   Common definitions for 2.4G stack across all modules (dongle, mouse, keyboard).
 *          This header centralizes shared macros, enums, and simple types to eliminate
 *          code duplication and ensure consistency.
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
#ifndef __D24G_STACK_COMMON_H__
#define __D24G_STACK_COMMON_H__

#include "types.h"
#include "tl_fifo.h"
#include "../../app_common_config.h"
#include "tl_common.h"


/* Note: Standard types (uint8_t, uint32_t, bool, etc.) are provided by tl_common.h */

/* ============================================================
 *  Common Macros
 * ============================================================ */

/* ---- Stack Event Queue ---- */
#define STACK_EVT_QUEUE_SIZE      32
#define STACK_EVT_DATA_MAX_LEN    78

/* ---- Access Code ---- */
#define ACCESS_CODE_BIT_LENGTH 32
#define MAX_CONSECUTIVE_BITS 3

/* ---- RF Register ---- */
#define REG_RXCHN_EN    (REG_TL_MODEM_BASE_ADDR + 0x4d)

/**
 * @brief Pairing/Connection Access Code
 * @note This value must be consistent across dongle, mouse, and keyboard.
 */
#define ACCESS_CODE   0x47d9b2c6

/* ---- RF Packet Constants ---- */
#define RF_PKT_PREAMBLE_LEN                         (5)
#define RF_PKT_AC_LEN                               (5)
#define RF_PKT_HEADER_LEN                           (2)
#define RF_PKT_DATA_LEN                             (12)
#define RF_PKT_CRC_LEN                              (3)
#define RF_PKT_ACK_LEN                              (3)
#define RF_SETTLE_TIME_US                           (15)
#define RF_RX_PATH_DELAY_US                         (8)
#define RF_TX_PATH_DELAY_US                         (2)
#define RF_RX_HIT_SYNC_US                           (16)
#define RF_GET_DMA_LEN(rf_len)                      (((rf_len + 13) / 4) * 4)

/* ---- SN/NESN Operations ---- */
#define SN_MASK        0x01
#define NESN_MASK      0x02
#define GET_SN(b)      ((b) & SN_MASK)
#define GET_NESN(b)    (((b) >> 1) & 0x01)
#define SET_SN(b, sn)  ((b) = ((b) & ~SN_MASK) | ((sn) & SN_MASK))
#define SET_NESN(b, n) ((b) = ((b) & ~NESN_MASK) | (((n) & 0x01) << 1))

/* ---- Flow Control ---- */
#define FLOW_PKT_NEW        0x01
#define FLOW_PKT_SENT       0x02

/* ---- Task Slots ---- */
#define MAX_TASK_SLOTS_1K       (5)
#define MAX_TASK_SLOTS_500      (8)
#define MAX_TASK_SLOTS_250      (16)
#define MAX_TASK_SLOTS_125      (16)
#define MAX_TASK_SLOTS_2K       (3)
#define MAX_TASK_SLOTS_4K       (2)

/* ---- SPP / FIFO ---- */
#define KM_SPP_MAX_LEN                          (1 + 65) /*opcode + spp data*/
#define DONGLE_SPP_MAX_LEN                      (49)
#define LL_FIFO_SIZE 16
#define LL_FIFO_NUM 8

/* ---- Timing ---- */
#define TIMER1_AHEAD_US (3)
#define D_TIMER1_TX_SLOT_AHEAD_US   (2)
#define RF_TRIG_AND_MARGIN  (1)
#define APP_RF_12B_RX_FSM_TIME_US                     (58)
#define APP_RF_24B_RX_FSM_TIME_US                     (74)
#define D_SUSPEND_EARLY_TIME_US                (120)

/* ---- Slot Timing ---- */
#define SLOT1_OFFSET_TIME_US_500_1K_2K          (170)
#define SLOT_TIME_US_500_1K_2K                  (150)
#define SLOT1_OFFSET_TIME_US_125                (200)
#define SLOT_TIME_US_125                        (190)
#define KB_ALL_KEY_MARGIN_TIME                  (13)
#define FOR_KB_ALL_KEY_SYNC_MARGIN_TIME         (10)
#define KM_SLOT0_MARGIN_US_500_1K_2K            (7)
#define KM_SLOT1_TIME_US_500_1K_2K              (SLOT1_OFFSET_TIME_US_500_1K_2K + KB_ALL_KEY_MARGIN_TIME)

/* ---- Report Rate ---- */
#define REPORT_RATE_MASK (REPORT_RATE_125 | REPORT_RATE_250 | REPORT_RATE_500 |\
                            REPORT_RATE_1K | REPORT_RATE_2K | REPORT_RATE_4K | REPORT_RATE_8K)
#define REPORT_RATE_NOT_8K_MSK (REPORT_RATE_125 | REPORT_RATE_250 | REPORT_RATE_500 |\
                                REPORT_RATE_1K | REPORT_RATE_2K | REPORT_RATE_4K)

/* ---- Pairing / Reconnection ---- */
#define TPSLL_NEW_RECON_EN                          (1)
#define TPSLL_NEW_PAIR_EN                           (1)
#define TPSLL_PAIR_CHN_MAX_NUM                      (8)
#define TPSLL_CONN_QUICK_LOST_TIME_US               (500 * 1000)
#define TPSLL_RR_CHG_WITH_RECON_EN                  (1)
#define TPSLL_RR_DELAY_SWITCH_FNO_N                 (20)
#define TPSLL_RR_SWITCH_CONFIG_TIME_US              (15000)
#define TPSLL_RR_CHG_RECON_TIMEOUT_US               (1000 * 1000)
#define TPSLL_CONNECTING_CHECK_EN                   (1)
#define PAIRING_RSP_MAX_RETRY_CNT                   (5)

/* ---- RF Timing ---- */
#define DONGLE_INTERVAL_US                          (250)
// #define PAIRING_INTERVAL_US                         (250)
#define TPSLL_TIMEOUT_MIN_MS                        (120)
#define TPSLL_TIMEOUT_MAX_MS                        (3600000 * 2)
// #define PARING_STIMER_TO_TX_US                      (4)
#define KM_PAIRING_SRX_TIMEOUT_US                   (42)
#define KB_PAIRING_RX_TIMEOUT                       (31000)
#define KB_PAIRING_RSP_RX_TIMEOUT                   (100)
#define KB_CONNECTED_RX_TIMEOUT                     (43)
#define KB_CONNECTED_KB_RX_TIMEOUT                  (47)
#define KM_CONNECTED_SPP_MODE_RX_TIMEOUT            (50)
#define KB_PAIRING_STIMER_TO_RX_MARGIN_US           (2)

/* ---- Keep Alive ---- */
#define TPSLL_KEEP_ALIVE_EN                             1
#define TPSLL_8K_KEEP_ALIVE_SYNC_IN_US         (4000)
#define TPSLL_KEEP_ALIVE_NOTIFY_EN                  (1)
#define TPSLL_KEEP_ALIVE2_NOTIFY_EN                 (1)

/* ---- FHSS / Channel ---- */
#define TPSLL_FHSS_EN                               (1)
#define TPSLL_N8K_FHSS_EN                           (1)
#define TPSLL_FHSS_CHN_MAX_NUM                      (24)
#define TPSLL_HOP_CHN_NUM                           (3)
#define TPSLL_FHSS_AFH_SLOT_SHIFT                   (9)
#define TPSLL_N8K_FHSS_AFH_SLOT_SHIFT               (10)
#define TPSLL_FHSS_PER_T_FNO_NUM                    ((1 << TPSLL_FHSS_AFH_SLOT_SHIFT) - 1)
#define TPSLL_N8K_CHN_EXPAND_EN                     (1)
#define TPSLL_N8K_CHN_EXPAND_ONLY_IN_NSLOT0_EN      (1)
#define TPSLL_RECON_RSSI_SCAN_EN                    (1)
#define TPSLL_RECON_RSSI_SCAN_MAX_EN                (1)
#define TPSLL_RECON_CHN_NUM                         TPSLL_HOP_CHN_NUM
#define RSSI_SCAN_WIN_US                            (100)
#define RSSI_SCAN_MARGIN_US                         (5)
#define RSSI_SCAN_CHANNEL_COUNT                     (40)
#define RSSI_SCAN_ROUNDS                            (3)
#define RSSI_SCAN_TOP_CHANNEL_COUNT                 TPSLL_FHSS_CHN_MAX_NUM
#define TPSLL_RECON_RSSI_SCAN_TIME_US               ((RSSI_SCAN_WIN_US + RSSI_SCAN_MARGIN_US) * RSSI_SCAN_CHANNEL_COUNT * RSSI_SCAN_ROUNDS + 1000)
#define ENFORCE_CHANNEL_SPACING                     (0)
#define MIN_CHANNEL_SPACING                         (3)
#define TPSLL_WIFI_CHANNEL_COUNT                    (13)
#define TPSLL_8K_STX_SRX_EN                         (1)
#define TPSLL_8K_CHN_HOP_TURN_EN                    (0)

/* ---- Pairing Packet ---- */
#define PAIR_REQ_ENCRYPT_KEY_OFFSET                 (3)
#define TPSLL_PAIR_ENCRYPT_KEY_LEN                  (4)
#define TPSLL_PAIR_REQ_MAX_LEN                      (PAIR_REQ_ENCRYPT_KEY_OFFSET + TPSLL_PAIR_ENCRYPT_KEY_LEN)
#define TPSLL_PAIR_RSP_PKT_LEN                      (11)
#define TPSLL_PAIR_CON_REQ_LEN                      (12)
#define TPSLL_PAIR_CON_RSP_LEN                      (10)
#define TPSLL_NEW_SYNC_EN                           (1)
#define TPSLL_CONN_TICK_UPDATA_SYNC_DIFF_US         (36)

/* ---- KB Active PM Optimization ---- */
#define TPSLL_KB_ACTIVE_PM_OPTM_EN                   (1)
#define TPSLL_KB_ALIVE_STAGED_TIME_MS(n)            (n * 4)
#define TPSLL_KB_ALIVE_STAGED_TIME_S(n)             (TPSLL_KB_ALIVE_STAGED_TIME_MS(n) * 1000)
#define TPSLL_KB_ALIVE_STAGED_TIME_M(n)             (TPSLL_KB_ALIVE_STAGED_TIME_S(n) * 60)
#define TPSLL_KB_ALIVE_MAX_STAGED_TIME_MS           (30 * 60 * 1000)
#define TPSLL_KB_ALIVE_MIN_STAGED_TIME_MS           (300)
#define TPSLL_KB_ALIVE_STAGE0_FNO_MASK              (0x00)
#define TPSLL_KB_8K_ALIVE_STAGE1_FNO_MASK           (0x03) //interval 1ms
#define TPSLL_KB_8K_ALIVE_STAGE2_FNO_MASK           (0x07) //interval 2ms
#define TPSLL_KB_8K_ALIVE_STAGE3_FNO_MASK           (0x0f) //interval 4ms
#define TPSLL_KB_8K_ALIVE_STAGE4_FNO_MASK           (0x1f) //interval 8ms
#define TPSLL_KB_8K_ALIVE_STAGE5_FNO_MASK           (0x3f) //interval 16ms
#define TPSLL_KB_ALIVE_TIME_STEP_MS                 (16)

#define TPSLL_KB_ACTIVE_PM_OPTM_LEVEL1_START_MS     (1000)
#define TPSLL_KB_ACTIVE_PM_OPTM_LEVEL3_START_MS     (2000)
/* ---- PM Optimization ---- */
#define TPSLL_PM_OPTM_EN                            (1)
/* for keyboard */
#define TPSLL_PM_OPTM_8K_ONLY_USE_2T_EN             (1)
#define TPSLL_PM_OPTM_START_TIME_MIN_S              (1)
#define TPSLL_PM_OPTM_START_TIME_MAX_S              (3 * 60 * 60)
#define TPSLL_PM_OPTM_LEVEL1_KEEP_ALIVE_SYNC_US     (4 * 1000)
#define TPSLL_PM_OPTM_LEVEL2_8K_KEEP_ALIVE_SYNC_US  (16 * 1000)
#define TPSLL_PM_OPTM_LEVEL2_125_KEEP_ALIVE_SYNC_US (16 * 1000) // sync interval: 16ms + 8ms = 24ms (must be multiple of 8ms)
#define TPSLL_PM_OPTM_LEVEL2_250_KEEP_ALIVE_SYNC_US (16 * 1000) // sync interval: 16ms + 4ms = 20ms (must be multiple of 4ms)
#define TPSLL_PM_OPTM_LEVEL2_N8K_KEEP_ALIVE_SYNC_US (20 * 1000)
#define TPSLL_PM_OPTM_8K_RX_WIN_EXPAND_US           (20)
#define TPSLL_PM_OPTM_N8K_RX_WIN_EXPAND_US          (20)

#define TPSLL_KB_KEEP_ALIVE_PER_EN                  (1)

/* ---- Misc ---- */
#define HOP_CHANNEL_METHOD                          (1)
#define COMB_METHOD                                 (0)
#define MAX_COMB_NUM                                (3)
#define RSSI_SCAN_EN                                (1)
#define AFH_ENABLE                                  (0)
#define SPP_MAX_CONT_CNT                        (10)
// #define TPSLL_SUSPEND_EN                       (1)
#define TPSLL_ACC_SWITCH_WITH_RR_EN            (1)
#define IS_ODD(x)  ((x) & 1)
#define IS_EVEN(x) (!((x) & 1))
#define TPSLL_PAIR_TICK_UPDATA_SYNC_DIFF_US         (30)
#define TPSLL_RECON_TICK_UPDATA_SYNC_DIFF_US        (31)
#define TPSLL_8K_WHEEL_SEND_IN_LL_EN                (1)
#define RECONN_MAX_CH       35
#define RECONN_CHN_NUM      8
#define PAIR_CHN_NO 11
#define CONN_CHN_NO 29
#define WFI_TRUE   (true)
#define WFI_FALSE  (false)
#define SPP_SW_INTV_CNT_THRESHOLD    (10)
#define RR_CHG_DLY_CNT  (64)

#ifndef ZW_TG
    #define ZW_TG(pin)  do{}while(0)
#endif
#ifndef ZW_WR
    #define ZW_WR(pin, val)  do{}while(0)
#endif

typedef struct{
    unsigned int dma_len;

    unsigned char snnesn : 2;   // snnesn (bit 0-1)
    unsigned char pkt_type : 2;
    unsigned char s_n : 1;
    unsigned char n_tab : 1; //next channel table flag
    unsigned char rsvd : 2;
    unsigned char   rf_len;             //LEN(6)_RFU(2)

    unsigned char  buf[68];
} rf_packet_t;
extern rf_packet_t  rf_tx_packet;


/* ============================================================
 *  Common Enums
 * ============================================================ */

/**
 * @brief Pairing encryption status
 */
typedef enum {
    PAIR_ENCRYPT_OK    = 0,
    PAIR_ENCRYPT_ERR   = 1
} pair_encrypt_status_t;

/**
 * @brief Pairing step enumeration
 */
typedef enum {
    PAIR_STEP_NONE = 0,
    PAIR_STEP1,
    PAIR_STEP2,
    PAIR_STEP3,
    PAIR_STEP4,
} tpsll_pair_e;

/**
 * @brief Reconnection step enumeration
 */
typedef enum {
    RECON_STEP_NONE = 0,
    RECON_STEP1,
    RECON_STEP2,
    RECON_STEP3,
    RECON_STEP4,
} tpsll_recon_e;

/**
 * @brief Report mode type
 */
typedef enum {
    REPORT_MODE_FAST = 0,
    REPORT_MODE_NORMAL,
} report_mode_t;

/**
 * @brief Slot mode for KM/SPP scheduling
 */
typedef enum {
    SLOT_MODE_KM,
    SLOT_MODE_SPP,
} p24g_slot_mode_e;

/* ============================================================
 *  Common Type Definitions
 * ============================================================ */

/**
 * @brief Stack event element
 */
typedef struct {
    tpsll_evt_t evt;
    uint16_t len;
    uint8_t data[STACK_EVT_DATA_MAX_LEN];
} stack_event_elem_t;

/**
 * @brief Stack event queue
 */
typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    stack_event_elem_t buf[STACK_EVT_QUEUE_SIZE];
} stack_event_queue_t;

static inline uint16_t next_idx(uint16_t idx) {
    return (idx + 1) % STACK_EVT_QUEUE_SIZE;
}

static inline bool stack_evt_queue_full(const stack_event_queue_t *q) {
    uint16_t next = (q->head + 1) % STACK_EVT_QUEUE_SIZE;
    return (next == q->tail);
}

/* Check if empty */
static inline bool stack_evt_queue_empty(const stack_event_queue_t *q) {
    return (q->head == q->tail);
}


/* ============================================================
 *  Common Function Declarations
 * ============================================================ */

uint32_t fnv1a_32_hash(const uint8_t *data, size_t len);

uint8_t wait_rf_fsm_idle_timeout(unsigned int timeout_us);

unsigned char * pp_rf_tx_fifo_get_ptr(pl_fifo_t *f);

void tpsll_gen_reconn_chn(uint8_t a, uint8_t out[RECONN_CHN_NUM]);


void tpsll_generate_deterministic_ble_channels(const uint8_t *existing_chs, uint8_t *new_chs);
void rf_rssi_scan_param_set(void);
void tpsll_bubble_sort_rssi_index(uint16_t *chn_rssi, uint8_t num, uint8_t *sorted_idx);


_attribute_ram_code_sec_ static inline uint32_t prng(uint32_t x) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return x;
}

_attribute_ram_code_sec_ static inline uint8_t bit_count(uint8_t x) {
    x = x - ((x >> 1) & 0x55);
    x = (x & 0x33) + ((x >> 2) & 0x33);
    return (((x + (x >> 4)) & 0x0F));
}

_attribute_ram_code_sec_ static inline uint16_t p24g_crc16(uint16_t crc, uint8_t *pD, int32_t len)
{
    #define poly 0x8408
    for (int32_t j = 0; j < len; j++) {
        uint32_t ds = pD[j];
        for (int32_t i = 0; i < 8; i++) {
            int32_t bit = (crc ^ ds) & 1;
            crc >>= 1;
            if (bit) crc ^= poly;
            ds >>= 1;
        }
    }
    return crc;
}


static inline uint32_t p24g_access_code_16to32(uint16_t code)
{
    uint32_t r = 0;
    for (int32_t i = 0; i < 16; i += 2) {
        r <<= 4;
        int32_t c = (code >> i) & 3;
        r |= (uint32_t)((c << 2) | ((~c) & 3));
    }
    return r;
}


static inline uint32_t mac_hash32(const uint8_t mac_addr[6])
{
    uint32_t h = 0;
    for (int i = 0; i < 6; i++) h = (h * 131u) ^ mac_addr[i];

    h ^= h << 13; h ^= h >> 17; h ^= h << 5;
    return h;
}

static inline uint8_t map_to_safe_ble_channel(uint32_t h)
{
    uint32_t m = h % 29u;
    if (m < 7u) {
        return (uint8_t)(2u + m);        // 2..8
    } else {
        return (uint8_t)(13u + (m - 7u)); // 13..34
    }
}


static inline uint32_t generate_valid_access_code(uint32_t initial_ac)
{
    uint32_t ac = initial_ac;
    uint32_t last_bit = ac & 1;
    uint32_t count = 1;

    for (int i = 1; i < ACCESS_CODE_BIT_LENGTH; i++) {
        uint32_t next_bit = (ac >> i) & 1;

        if (next_bit == last_bit) {
            count++;
            if (count > MAX_CONSECUTIVE_BITS) {

                next_bit = 1 - last_bit;
                count = 1;
            }
        } else {
            count = 1;
        }


        ac = (ac & ~(1u << i)) | (next_bit << i);

        last_bit = next_bit;
    }

    return ac;
}

static inline uint8_t prng8(uint8_t *s)
{
    uint8_t x = *s;
    x ^= x << 3;
    x ^= x >> 5;
    x ^= x << 1;
    *s = x;
    return x;
}

static inline bool tpsll_is_rf_busy(void)
{
    return (0 != (read_reg8(0x170224) & 0x7));
}

#endif /* __D24G_STACK_COMMON_H__ */
