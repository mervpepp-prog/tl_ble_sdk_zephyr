/********************************************************************************************************
 * @file    app_d24g_stack.c
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
#include "tl_common.h"
#include "app_d24g_rf_link.h"
#include "app_d24g_stack.h"
#include "tl_fifo.h"
// #include "app_public.h"
#include "tl_string.h"
#include "../../common/app_common.h"
#include "../../common/stack/d24g_stack_common.h"
#include "stack/multicore_comm/service/service_n22.h"
#include "stack/multicore_comm/service/mcc.h"


#define TPSLL_NEW_KEEP_ALIVE_EN                 1
#define TPSLL_NEW_8K_RF_TIMING_EN               1

#if TPSLL_NEW_KEEP_ALIVE_EN
/* TPSLL_8K_KEEP_ALIVE_SYNC_IN_US moved to d24g_stack_common.h */
#endif
#define TPSLL_SPP_NOT_SWITCH_EN                 1
#define TPSLL_NON_8K_SPP_NOT_SW_EN              1
#define TPSLL_SPP_FRAG_EN                       1

#if TPSLL_SPP_NOT_SWITCH_EN
#define TPSLL_SPP_KM_RATIO                      3 // spp:ms = 1:3

#endif

/* KM_SPP_MAX_LEN moved to d24g_stack_common.h */
/* DONGLE_SPP_MAX_LEN moved to d24g_stack_common.h */
#if TPSLL_NEW_8K_RF_TIMING_EN
#define KM_2T_SPP_MAX_LEN                       (13)
#else
#define KM_2T_SPP_MAX_LEN                       (24)
#endif

/* SLOT1_OFFSET_TIME_US_500_1K_2K moved to d24g_stack_common.h */
/* SLOT_TIME_US_500_1K_2K moved to d24g_stack_common.h */
/* SLOT1_OFFSET_TIME_US_125 moved to d24g_stack_common.h */
/* SLOT_TIME_US_125 moved to d24g_stack_common.h */
#define TX_MARGIN_US_125                        (40)
/* KB_ALL_KEY_MARGIN_TIME moved to d24g_stack_common.h */
/* FOR_KB_ALL_KEY_SYNC_MARGIN_TIME moved to d24g_stack_common.h */
/* KM_SLOT0_MARGIN_US_500_1K_2K moved to d24g_stack_common.h */
/* KM_SLOT1_TIME_US_500_1K_2K moved to d24g_stack_common.h */
#define KM_SLOT_TIME_US_500_1K_2K               (SLOT_TIME_US_500_1K_2K + KB_ALL_KEY_MARGIN_TIME)

/* MAX_TASK_SLOTS_1K moved to d24g_stack_common.h */
/* MAX_TASK_SLOTS_500 moved to d24g_stack_common.h */
/* MAX_TASK_SLOTS_250 moved to d24g_stack_common.h */
/* MAX_TASK_SLOTS_125 moved to d24g_stack_common.h */
/* MAX_TASK_SLOTS_2K moved to d24g_stack_common.h */
/* MAX_TASK_SLOTS_4K moved to d24g_stack_common.h */

/* RR_CHG_DLY_CNT moved to d24g_stack_common.h */

#define RR_SW_SKIP_CNT  (30)

#define TPSLL_NO_EVENT_PWR_DOWN_EN                      0

// Enable frequency hopping function (1: enabled, 0: disabled)
// #define HOP_FREQ_FUN_ENABLE   1

/* ACCESS_CODE is now defined in app_d24g_stack_common.h */


/* FLOW_PKT_NEW moved to d24g_stack_common.h */
/* FLOW_PKT_SENT moved to d24g_stack_common.h */

/* SN_MASK moved to d24g_stack_common.h */
/* NESN_MASK moved to d24g_stack_common.h */

/* GET_SN moved to d24g_stack_common.h */
/* GET_NESN moved to d24g_stack_common.h */
/* SET_SN moved to d24g_stack_common.h */
/* SET_NESN moved to d24g_stack_common.h */

#define TPSLL_8K_CHN_EXPAND_EN                      (0)

#define SWITCH_DELAY_INTERVAL  (20)

#define HIGH_REPORT_RATES (REPORT_RATE_8K | REPORT_RATE_4K)

#define KM_RECONN_RX_WIN    (13000)

#define SLOT0_TX_TIME_US    (60)

/* D_SUSPEND_EARLY_TIME_US moved to d24g_stack_common.h */

#define MS_IDLE_CNT     (10)

/* SPP_MAX_CONT_CNT moved to d24g_stack_common.h */

#define TPSLL_SUSPEND_EN                       (1)
/* TPSLL_ACC_SWITCH_WITH_RR_EN moved to d24g_stack_common.h */

#define TPSLL_RR_CHG_FROM_DONGLE                    (0)
#define   IS_3T_2R      1   
#define   IS_1T_1R      0  

#define KB_SPP_FRAG_EN          1

#if KB_SPP_FRAG_EN
#define SPP_FRAG_HDR_LEN            1
#define SPP_MAX_CHUNK_LEN           48
#define SPP_MAX_PACKET_LEN          256

typedef enum {
    SPP_FRAG_TYPE_NO_FRAG           = 0,
    SPP_FRAG_TYPE_MID               = 1,
    SPP_FRAG_TYPE_END               = 2,
    SPP_FRAG_TYPE_RESERVED          = 3,
} spp_frag_type_t;

#define SPP_OPCODE_DEFAULT          0
#define SPP_FRAG_TYPE_MASK          0x03
#define SPP_OPCODE_SHIFT            2

// SPP reassembly context
typedef struct {
    uint8_t  buf[SPP_MAX_PACKET_LEN];
    uint16_t cur_len;                    
    uint8_t  opcode;                     
    bool     is_receiving;               
} spp_reassemble_t;

static spp_reassemble_t g_spp_rx_assemble = {0};
#endif

typedef enum
{
    TPSLL_INTER_EVT_SPP_SEND_COMP = TPSLL_EVT_MAX,
    // TPSLL_INTER_EVT_KS_DISABLE,
    TPSLL_INTER_EVT_MAX,
} tpsll_inter_evt_t;

/* IS_ODD moved to d24g_stack_common.h */
/* IS_EVEN moved to d24g_stack_common.h */

/* REPORT_RATE_MASK moved to d24g_stack_common.h */

/* REPORT_RATE_NOT_8K_MSK moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////
/* TPSLL_NEW_RECON_EN moved to d24g_stack_common.h */
/* TPSLL_NEW_PAIR_EN moved to d24g_stack_common.h */

/* TPSLL_PAIR_TICK_UPDATA_SYNC_DIFF_US moved to d24g_stack_common.h */
/* TPSLL_RECON_TICK_UPDATA_SYNC_DIFF_US moved to d24g_stack_common.h */
/* TPSLL_PAIR_CHN_MAX_NUM moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////
/* DONGLE_INTERVAL_US moved to d24g_stack_common.h */
#if TPSLL_NEW_RECON_EN
#define PAIRING_INTERVAL_US                         (250)
#else
#define PAIRING_INTERVAL_US                         (300)
#endif
#define DONGLE_PARING_STIMER_TO_TX_MARGIN_US        (3)     /* Note: This macro needs to be adjusted according to the code */
/* KB_PAIRING_STIMER_TO_RX_MARGIN_US moved to d24g_stack_common.h */
/* TPSLL_TIMEOUT_MIN_MS moved to d24g_stack_common.h */
/* TPSLL_TIMEOUT_MAX_MS moved to d24g_stack_common.h */

#define PARING_STIMER_TO_TX_US                      (4)
/* KM_PAIRING_SRX_TIMEOUT_US moved to d24g_stack_common.h */

/* KB_PAIRING_RX_TIMEOUT moved to d24g_stack_common.h */
/* KB_PAIRING_RSP_RX_TIMEOUT moved to d24g_stack_common.h */
/* KB_CONNECTED_RX_TIMEOUT moved to d24g_stack_common.h */
/* KB_CONNECTED_KB_RX_TIMEOUT moved to d24g_stack_common.h */
/* KM_CONNECTED_SPP_MODE_RX_TIMEOUT moved to d24g_stack_common.h */

/* RF_PKT_PREAMBLE_LEN moved to d24g_stack_common.h */
/* RF_PKT_AC_LEN moved to d24g_stack_common.h */
/* RF_PKT_HEADER_LEN moved to d24g_stack_common.h */
/* RF_PKT_DATA_LEN moved to d24g_stack_common.h */
/* RF_PKT_CRC_LEN moved to d24g_stack_common.h */

/* RF_PKT_ACK_LEN moved to d24g_stack_common.h */

/* RF_SETTLE_TIME_US moved to d24g_stack_common.h */
/* RF_RX_PATH_DELAY_US moved to d24g_stack_common.h */
/* RF_TX_PATH_DELAY_US moved to d24g_stack_common.h */
/* RF_RX_HIT_SYNC_US moved to d24g_stack_common.h */
//#define CONNECTED_CODE_MARGIN_US                    (3)
//#define RF_TX_WAIT                                  (1)
//#define RF_RX_WAIT                                  (1)

/* RF_GET_DMA_LEN moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////////////////

#define APP_RF_TIME_FINE_ADJ                        (1)

// #define TIMER1_DELAY_US  (5)                     
// #define STIMER_AHEAD_US  (2) 
/* TIMER1_AHEAD_US moved to d24g_stack_common.h */
/* RF_TRIG_AND_MARGIN moved to d24g_stack_common.h */
/* D_TIMER1_TX_SLOT_AHEAD_US moved to d24g_stack_common.h */

// 15 + 1 + ((2+5+5+12+3) * 4 + 2) / 3 + 8
// #define RF_STX_WITH_DATA_LEN_TIME_US(data_len)  ((RF_SETTLE_TIME_US + ((RF_PKT_PREAMBLE_LEN + RF_PKT_AC_LEN + RF_PKT_HEADER_LEN + (data_len) + RF_PKT_CRC_LEN) * 4 + 2) / 3) + RF_RX_PATH_DELAY_US)
#define APP_SLOT_TX_MARGIN_US                       (2) /* SLOT to TX margin */
#define APP_TX_DONE_TO_SLOT_MARGIN_US               (1) /* TX done to NEXT SLOT margin */

#define APP_SLOT0_START_TIME_US                     (0) /* SLOT0 start time */
/* 3 slot0 tx tick*/
// #define APP_SLOT0_1T_START_TIME_US                  (APP_SLOT0_START_TIME_US + APP_SLOT_TX_MARGIN_US + 1) /* 1T start time */
#define APP_SLOT0_1T_START_TIME_US                  (3) /* 1T start time */
/* 61 */
// #define APP_SLOT1_START_TIME_US                     (APP_SLOT0_1T_START_TIME_US + RF_STX_WITH_DATA_LEN_TIME_US(RF_PKT_DATA_LEN) + APP_TX_DONE_TO_SLOT_MARGIN_US) /* SLOT1 start time */
// #define APP_SLOT1_START_TIME_US                     (62 - 5)

/* APP_RF_12B_RX_FSM_TIME_US moved to d24g_stack_common.h */
/* APP_RF_24B_RX_FSM_TIME_US moved to d24g_stack_common.h */


/* 58 slot1 timer1*/
#if TPSLL_NEW_8K_RF_TIMING_EN
#define APP_SLOT1_START_TIME_US                     (APP_RF_12B_RX_FSM_TIME_US + APP_SLOT0_1T_START_TIME_US - TIMER1_AHEAD_US + 9)
#else
#define APP_SLOT1_START_TIME_US                     (APP_RF_12B_RX_FSM_TIME_US + APP_SLOT0_1T_START_TIME_US - TIMER1_AHEAD_US)
#endif
/* 62 slot1 tx tick*/
#define APP_SLOT1_2T_START_TIME_US                  (APP_RF_12B_RX_FSM_TIME_US + APP_SLOT0_1T_START_TIME_US + RF_TRIG_AND_MARGIN) /* 2T start time */


/* 133 slot2 timer1*/
#define APP_SLOT2_START_TIME_US                     (APP_SLOT1_2T_START_TIME_US + APP_RF_24B_RX_FSM_TIME_US - TIMER1_AHEAD_US)  /* SLOT2 START*/
/* 137 slot2 tx tick*/
#define APP_SLOT2_3T_START_TIME_US                  (APP_SLOT1_2T_START_TIME_US + APP_RF_24B_RX_FSM_TIME_US + RF_TRIG_AND_MARGIN) /* 2T start time */

/* 193 slot3 timer1*/
#if TPSLL_NEW_8K_RF_TIMING_EN
#define APP_DONGLE_SLOT3_START_TIME_US              (APP_SLOT2_3T_START_TIME_US + APP_RF_12B_RX_FSM_TIME_US - D_TIMER1_TX_SLOT_AHEAD_US - 2)  /* SLOT3 START*/
#else
#define APP_DONGLE_SLOT3_START_TIME_US              (APP_SLOT2_3T_START_TIME_US + APP_RF_12B_RX_FSM_TIME_US - D_TIMER1_TX_SLOT_AHEAD_US)  /* SLOT3 START*/
#endif
/* 196 slot3 tx tick*/
#define APP_SLOT3_1T_START_TIME_US                  (APP_SLOT2_3T_START_TIME_US + APP_RF_12B_RX_FSM_TIME_US + RF_TRIG_AND_MARGIN)

/* 217 */
#define APP_PAIRING_TS_TO_DONGLE_NEXT_STIMER_TIME      (DONGLE_INTERVAL_US - (DONGLE_PARING_STIMER_TO_TX_MARGIN_US + RF_SETTLE_TIME_US + RF_RX_HIT_SYNC_US))
#define APP_PAIRING_CALI_FINE_ADJ_WITH_TICK         (22)


/* 199 */
// #define APP_DONGLE_SLOT3_START_TIME_US              (APP_SLOT2_3T_START_TIME_US + RF_STX_WITH_DATA_LEN_TIME_US(24) + APP_TX_DONE_TO_SLOT_MARGIN_US)  /* SLOT3 START*/
/* 231 */
// #define APP_DONGLE_1T_TIMESTAMP_TIME_US                       (APP_DONGLE_SLOT3_START_TIME_US + APP_SLOT_TX_MARGIN_US + RF_SETTLE_TIME_US + RF_RX_HIT_SYNC_US)

/* 227 KM hit sync us*/
#define APP_KM_RX_TS_US                             (APP_SLOT3_1T_START_TIME_US + RF_SETTLE_TIME_US + RF_RX_HIT_SYNC_US)
#define APP_KM_RX_TS_US_NOT_8K                             (200 - 1 + RF_SETTLE_TIME_US + RF_RX_HIT_SYNC_US)
/* 23*/
// #define APP_CONN_TS_TO_NEXT_STIMER_US    (DONGLE_INTERVAL_US - APP_KM_RX_TS_US)
#if TPSLL_NEW_8K_RF_TIMING_EN
#define APP_CONN_TS_TO_NEXT_STIMER_US               (DONGLE_INTERVAL_US - APP_KM_RX_TS_US - 1 + 3)
#define APP_CONN_CALI_FINE_ADJ_WITH_TICK            (5)
#else
#define APP_CONN_TS_TO_NEXT_STIMER_US               (DONGLE_INTERVAL_US - APP_KM_RX_TS_US - 1)
#define APP_CONN_CALI_FINE_ADJ_WITH_TICK            (3)
#endif

#define APP_CONN_TS_TO_NEXT_STIMER_US_NOT_8K    ((DONGLE_INTERVAL_US) - (APP_KM_RX_TS_US_NOT_8K))


//////////////////////////////////////////////////////////////////

#define T2_FIRST_3_PKT_AND_TX_SETTLE                   (55)
#define MOTION_3950_BURST_READ_TIME_US                 (13) 
#define TX_BUFF_READY_MARGIN                           (12 + 3)//3us sw margin
#define APP_RF_2T_SENSOR_OFFSET_US                    (T2_FIRST_3_PKT_AND_TX_SETTLE - MOTION_3950_BURST_READ_TIME_US - TX_BUFF_READY_MARGIN)

//////////////////////////////////////////////////////////////////
/* low power */
#define APP_LOW_POWER_PREPARE_LOOP_CNT                  (1)
#define APP_SUSPEND_LIMIT_MIN_TIME_US                   (2000) //0 for unlimited
#define APP_SUSPEND_LONG_TIME_MIN_US                    (0) //0 for only short suspend
#define APP_WAKEUP_EARLY_TIME_US                        (50)
#define APP_LOW_CLK_WAKEUP_EARLY_TIME_US                (86)
#define APP_WAKEUP_EARLY_FOR_LONG_SUSPEND_TIME_US       (650)
#define APP_WFI_WAKEUP_EARLY_TIME_US                    (20)

#define TPSLL_DEEP_RET_EN                               (1)
#define TPSLL_DEEP_RET_LIMIT_MIN_TIME_US                (20 * 1000) //0 for unlimited
#define TPSLL_WAKEUP_EARLY_FOR_DEEP_RET_TIME_US         (200)
//////////////////////////////////////////////////////////////////
#define CLOSE_RF_WITH_USB_CONNECTED                     0

/* TPSLL_KEEP_ALIVE_EN moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////
#define LL_LOST_8K_MAX_CNT                          (200)
#define LL_LOST_N8K_MAX_CNT                         (100)

#define MS_BUF_N8K_IDX                              (0)

#if TPSLL_KEEP_ALIVE_EN
#define KEEP_ALIVE_DELAY_US                             (100 * 1000) /* us */
#define KEEP_ALIVE_SWITCH_8K_DELAY_US                   (2000 * 1000) /* us */
#define KEEP_ALIVE_INTERVAL_8K_MAX_NUM                  (100) /* n*250us */
#define KEEP_ALIVE_INTERVAL_8K_MAX_CNT                  (KEEP_ALIVE_INTERVAL_8K_MAX_NUM - 1)
#define KEEP_ALIVE_INTERVAL_4K_MAX_NUM                  (400) /* n*250us */
#define KEEP_ALIVE_INTERVAL_4K_MAX_CNT                  (KEEP_ALIVE_INTERVAL_4K_MAX_NUM - 1)
#define KEEP_ALIVE_INTERVAL_2K_MAX_NUM                  (200) /* n*500us */
#define KEEP_ALIVE_INTERVAL_2K_MAX_CNT                  (KEEP_ALIVE_INTERVAL_2K_MAX_NUM - 1)
#define KEEP_ALIVE_INTERVAL_1K_MAX_NUM                  (100) /* n*1ms */
#define KEEP_ALIVE_INTERVAL_1K_MAX_CNT                  (KEEP_ALIVE_INTERVAL_1K_MAX_NUM - 1)
#define KEEP_ALIVE_INTERVAL_500_MAX_NUM                 (50) /* n*2ms */
#define KEEP_ALIVE_INTERVAL_500_MAX_CNT                 (KEEP_ALIVE_INTERVAL_500_MAX_NUM - 1)
/* 8k stimer sync countdown with keep alive */
#define KEEP_ALIVE_STIMER_SYNC_CNT                      (1)
/* TPSLL_KEEP_ALIVE_NOTIFY_EN moved to d24g_stack_common.h */
#endif

#define KEEP_ALIVE_STIMER_SYNC_TH_US                    (6)
//////////////////////////////////////////////////////////////////
/* TPSLL_CONN_QUICK_LOST_TIME_US moved to d24g_stack_common.h */
#define TPSLL_CONN_QUICK_LOST_TIMEOUT_US            (100 * 1000) //0 for disable
#define TPSLL_CONN_TIMEOUT_8K_US                    (50 * 1000)
//////////////////////////////////////////////////////////////////
/* TPSLL_RR_CHG_WITH_RECON_EN moved to d24g_stack_common.h */

/* TPSLL_RR_DELAY_SWITCH_FNO_N moved to d24g_stack_common.h */
/* TPSLL_RR_SWITCH_CONFIG_TIME_US moved to d24g_stack_common.h */
#define TPSLL_SAVE_RR_CONFIG_TIME_US                (4000)
/* TPSLL_RR_CHG_RECON_TIMEOUT_US moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////
/* TPSLL_CONNECTING_CHECK_EN moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////
#define DEBUG_RF_REPORT_CNT_ENABLE                  (0)
/* HOP_CHANNEL_METHOD moved to d24g_stack_common.h */
/* COMB_METHOD moved to d24g_stack_common.h */
/* MAX_COMB_NUM moved to d24g_stack_common.h */
#define SKIP_INTV_FOR_WRITE_FLASH                   (20)
/* RSSI_SCAN_EN moved to d24g_stack_common.h */
/* AFH_ENABLE moved to d24g_stack_common.h */
/* PAIRING_RSP_MAX_RETRY_CNT moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////////////////
/* TPSLL_N8K_CHN_EXPAND_EN moved to d24g_stack_common.h */
/* TPSLL_N8K_CHN_EXPAND_ONLY_IN_NSLOT0_EN moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////////////////
/* Frequency-Hopping Spread Spectrum */
/* TPSLL_FHSS_EN moved to d24g_stack_common.h */
/* TPSLL_N8K_FHSS_EN moved to d24g_stack_common.h */
/* TPSLL_FHSS_CHN_MAX_NUM moved to d24g_stack_common.h */
/* TPSLL_HOP_CHN_NUM moved to d24g_stack_common.h */


/* TPSLL_FHSS_AFH_SLOT_SHIFT moved to d24g_stack_common.h */

#if TPSLL_N8K_FHSS_EN
/* TPSLL_N8K_FHSS_AFH_SLOT_SHIFT moved to d24g_stack_common.h */
#define CHN_IDX_LEN                                 (2)
#define LL_PKT_OFFSET                               (4)
#else
#define CHN_IDX_LEN                                 (0)
#define LL_PKT_OFFSET                               (2)
#endif

#define PL_APP_OFFSET_1                         (2)
#define PL_APP_OFFSET_2                         (0)
#define PL_APP_FULL_OFFSET_1    (PL_APP_OFFSET_1 + CHN_IDX_LEN)
#define PL_APP_FULL_OFFSET_2    (PL_APP_OFFSET_2 + CHN_IDX_LEN)

/* TPSLL_FHSS_PER_T_FNO_NUM moved to d24g_stack_common.h */

//////////////////////////////////////////////////////////////////
/* TPSLL_RECON_RSSI_SCAN_EN moved to d24g_stack_common.h */
/* TPSLL_RECON_RSSI_SCAN_MAX_EN moved to d24g_stack_common.h */

/* TPSLL_RECON_CHN_NUM moved to d24g_stack_common.h */
/* RSSI_SCAN_WIN_US moved to d24g_stack_common.h */
/* RSSI_SCAN_MARGIN_US moved to d24g_stack_common.h */
/* RSSI_SCAN_CHANNEL_COUNT moved to d24g_stack_common.h */
/* RSSI_SCAN_ROUNDS moved to d24g_stack_common.h */
/* RSSI_SCAN_TOP_CHANNEL_COUNT moved to d24g_stack_common.h */
/* TPSLL_RECON_RSSI_SCAN_TIME_US moved to d24g_stack_common.h */

/* ENFORCE_CHANNEL_SPACING moved to d24g_stack_common.h */
/* MIN_CHANNEL_SPACING moved to d24g_stack_common.h */

/* TPSLL_WIFI_CHANNEL_COUNT moved to d24g_stack_common.h */

//////////////////////////////////////////////////////////////////////////////
/* TPSLL_8K_STX_SRX_EN moved to d24g_stack_common.h */

/* turn 8K report rate channels in order */
/* TPSLL_8K_CHN_HOP_TURN_EN moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////
#if TPSLL_NEW_PAIR_EN
/* PAIR_REQ_ENCRYPT_KEY_OFFSET moved to d24g_stack_common.h */
#else
#define PAIR_REQ_ENCRYPT_KEY_OFFSET                 (1)
#endif
//////////////////////////////////////////////////////////////////////////////
/* TPSLL_PAIR_ENCRYPT_KEY_LEN moved to d24g_stack_common.h */

/* TPSLL_PAIR_REQ_MAX_LEN moved to d24g_stack_common.h */
/* TPSLL_PAIR_RSP_PKT_LEN moved to d24g_stack_common.h */
/* TPSLL_PAIR_CON_REQ_LEN moved to d24g_stack_common.h */
/* TPSLL_PAIR_CON_RSP_LEN moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////////////////
/* km sync with dongle's stimer */
/* TPSLL_NEW_SYNC_EN moved to d24g_stack_common.h */

/* TPSLL_CONN_TICK_UPDATA_SYNC_DIFF_US moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////////////////
/* TPSLL_KB_ACTIVE_PM_OPTM_EN moved to d24g_stack_common.h */

/* TPSLL_KB_ALIVE_STAGED_TIME_MS moved to d24g_stack_common.h */
/* TPSLL_KB_ALIVE_STAGED_TIME_S moved to d24g_stack_common.h */
/* TPSLL_KB_ALIVE_STAGED_TIME_M moved to d24g_stack_common.h */

/* TPSLL_KB_ALIVE_MAX_STAGED_TIME_MS moved to d24g_stack_common.h */
/* TPSLL_KB_ALIVE_MIN_STAGED_TIME_MS moved to d24g_stack_common.h */

/* TPSLL_KB_ALIVE_STAGE0_FNO_MASK moved to d24g_stack_common.h */

/* TPSLL_KB_8K_ALIVE_STAGE1_FNO_MASK moved to d24g_stack_common.h */
/* TPSLL_KB_8K_ALIVE_STAGE2_FNO_MASK moved to d24g_stack_common.h */
/* TPSLL_KB_8K_ALIVE_STAGE3_FNO_MASK moved to d24g_stack_common.h */
/* TPSLL_KB_8K_ALIVE_STAGE4_FNO_MASK moved to d24g_stack_common.h */
/* TPSLL_KB_8K_ALIVE_STAGE5_FNO_MASK moved to d24g_stack_common.h */
// #define TPSLL_KB_8K_ALIVE_STAGE6_FNO_MASK           (0x7f) //T = 32ms

/* TPSLL_KB_ALIVE_TIME_STEP_MS moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////////////////
/* TPSLL_PM_OPTM_EN moved to d24g_stack_common.h */
/* TPSLL_PM_OPTM_8K_ONLY_USE_2T_EN moved to d24g_stack_common.h */

/* TPSLL_PM_OPTM_START_TIME_MIN_S moved to d24g_stack_common.h */
/* TPSLL_PM_OPTM_START_TIME_MAX_S moved to d24g_stack_common.h */

/* TPSLL_PM_OPTM_8K_RX_WIN_EXPAND_US moved to d24g_stack_common.h */
#define TPSLL_PM_OPTM_LEVEL4                                    BIT(3)

#if TPSLL_PM_OPTM_EN
#define TPSLL_PM_OPTM_N8K_SUSPEND_EN                            (1)
#else
#define TPSLL_PM_OPTM_N8K_SUSPEND_EN                            (0)
#endif

#define TPSLL_PM_OPTM_LEVEL_EXITING                             BIT(7)

#define TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN           (1)
#define TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_SLOT_TIME_US        (15)

#define TPSLL_PM_OPTM_250_NEED_ENABLE_KS_EARLY_TIME_US          (90)

#define TPSLL_PM_OPTM_IN_CHRG_H_CLK_NEED_ENABLE_KS_EARLY_TIME_US    (50)
#define TPSLL_PM_OPTM_IN_CHRG_L_CLK_NEED_ENABLE_KS_EARLY_TIME_US    (90)

#define TPSLL_PM_OPTM_IDLE_INTERVAL_MIN_MS                      (20)
#define TPSLL_PM_OPTM_IDLE_INTERVAL_MAX_MS                      (500)
#define TPSLL_PM_OPTM_REF_FNO_CNT_MAX                           (0xFFFFFFF)

#define TPSLL_PM_OPTM_LEVEL3_REF_FNO_CNT_START                  (4)

#define TPSLL_PM_OPTM_PAIR_RECON_KS_DISABLE_FNO_MASK            (0x7F)
#define TPSLL_PM_OPTM_PAIR_RECON_KS_DISABLE_DELAY_US            (50)
#define TPSLL_PM_OPTM_TIMER1_KS_DISABLE_DELAY_US                (45)
#define TPSLL_PM_OPTM_LEVEL3_TIMER1_KS_DISABLE_DELAY_US         (15)
#define TPSLL_PM_OPTM_DEEP_RET_STIMER_START_DELAY_US            (20)
#define TPSLL_PM_OPTM_DEEP_RET_TIMER1_KS_DISABLE_DELAY_US       (22)

#define TPSLL_PM_OPTM_LEVEL3_DEFAULT_INTERVAl_MS                (50)
#define TPSLL_PM_OPTM_LEVEL4_DEFAULT_INTERVAl_MS                (200)
//////////////////////////////////////////////////////////////////////////////
#define TPSLL_UPDATE_STIMER_HANDLE_TOO_CLOSE_TH_US              (2)
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/* Timer0 anchor offset (in microseconds) used by app_set_d25f_timer0_anchor,
 * one value per report rate. Update here to tune timing across all rates. */
#define D25F_KEYSCAN_INTERVAL_96M       (50)
#define D25F_KEYSCAN_INTERVAL_32M       (280)

#define D25F_TIMER0_ANCHOR_US_8K        (15)
#define D25F_TIMER0_ANCHOR_US_4K        (DONGLE_INTERVAL_US - D25F_KEYSCAN_INTERVAL_96M)
#define D25F_TIMER0_ANCHOR_US_2K        ((DONGLE_INTERVAL_US << 1) - D25F_KEYSCAN_INTERVAL_32M)
#define D25F_TIMER0_ANCHOR_US_1K        ((DONGLE_INTERVAL_US << 2) - D25F_KEYSCAN_INTERVAL_32M)
#define D25F_TIMER0_ANCHOR_US_500       ((DONGLE_INTERVAL_US << 3) - D25F_KEYSCAN_INTERVAL_32M)
#define D25F_TIMER0_ANCHOR_US_250       ((DONGLE_INTERVAL_US << 4) - D25F_KEYSCAN_INTERVAL_32M)
#define D25F_TIMER0_ANCHOR_US_125       ((DONGLE_INTERVAL_US << 5) - D25F_KEYSCAN_INTERVAL_32M)
#define D25F_TIMER0_ANCHOR_US_DEFAULT   (30)


#ifndef ZW_TG
    /* ZW_TG moved to d24g_stack_common.h */
#endif

#ifndef ZW_WR
    /* ZW_WR moved to d24g_stack_common.h */
#endif

/* RECONN_MAX_CH moved to d24g_stack_common.h */
/* RECONN_CHN_NUM moved to d24g_stack_common.h */

uint8_t reconn_chn_table[RECONN_CHN_NUM];

static pl_fifo_t *km_fifo_ptr = NULL;
static pl_fifo_t *spp_fifo_ptr = NULL;

// #define KM_FIFO_SIZE 40
// #define KM_FIFO_NUM 32
// _attribute_aligned_(4) uint8_t buf_fifo_b[KM_FIFO_SIZE * KM_FIFO_NUM];
// pl_fifo_t km_fifo = {
//     .size = KM_FIFO_SIZE,
//     .num = KM_FIFO_NUM,
//     .wptr = 0,
//     .rptr = 0,
//     .p = buf_fifo_b,
// };


void tpsll_set_next_keep_alive_stimer_anchor(uint32_t alive_us);

/* LL_FIFO_SIZE moved to d24g_stack_common.h */
/* LL_FIFO_NUM moved to d24g_stack_common.h */
_attribute_aligned_(4) uint8_t ll_fifo_b[LL_FIFO_SIZE * LL_FIFO_NUM];

pl_fifo_t ll_fifo = {
    .size = LL_FIFO_SIZE,
    .num = LL_FIFO_NUM,
    .wptr = 0,
    .rptr = 0,
    .p = ll_fifo_b,
};

volatile __attribute__((section(".sdk_version"))) unsigned char d24g_rf_version[] = "$$$"TPSLL_STACK_VERSION"$$$";
const uint8_t tpsll_stack_version[] = TPSLL_STACK_VERSION;

_attribute_data_retention_ tpsll_dev_status_e g_state = STATE_POWERON;

_attribute_data_retention_ static tpsll_app_cb_t *s_app_cb = NULL;

#define PAIR_ACCESS_CODE       0xec322997

_attribute_data_retention_ unsigned char last_valid_key_buf[10];
_attribute_data_retention_ unsigned char last_all_key_buf[16];

_attribute_data_retention_ static tpsll_evt_callback_t p24g_callbacks[TPSLL_INTER_EVT_MAX] = {0};

_attribute_data_retention_ volatile unsigned int g_change_rr_delay = 0;

_attribute_data_retention_ static bool enable_pairing_flag = false;
_attribute_data_retention_ static rf_power_level_e pairing_tx_power = PAIRING_TX_POWER_DEFAULT;


/* PAIR_CHN_NO moved to d24g_stack_common.h */
/* CONN_CHN_NO moved to d24g_stack_common.h */

/* WFI_TRUE moved to d24g_stack_common.h */
/* WFI_FALSE moved to d24g_stack_common.h */

/* SPP_SW_INTV_CNT_THRESHOLD moved to d24g_stack_common.h */

#if TPSLL_RR_CHG_WITH_RECON_EN
typedef enum
{
    RR_RECON_STEP_NONE   = 0,
    RR_RECON_STEP1,
    RR_RECON_STEP2,
    RR_RECON_STEP3,
    RR_RECON_STEP4,
    RR_RECON_STEP5,
    RR_RECON_STEP6,
    RR_RECON_STEP7
} tpsll_rr_recon_e;
#endif

/* tpsll_pair_e, tpsll_recon_e, pair_encrypt_status_t are now defined in app_d24g_stack_common.h */

typedef struct {
    /* ---- 4-byte aligned ---- */
    uint32_t lost_cnt;              /* Connection lost counter */
    uint32_t lost_cnt_max;          /* Connection lost counter */
    uint32_t stick;                 /* Current stick value */
    uint32_t last_stick;            /* Previous stick value */
    uint32_t interval_us;           /* Communication interval in microseconds */
    uint32_t fno_conn;              /* Connection frequency number */

    #if TPSLL_PM_OPTM_EN
    uint32_t pm_optm_ref_fno_cnt;
    uint32_t pm_optm_rr_chgd_ref_fno_cnt_th; /* report rate just changed refer fno count threshold before keep alive */
    uint32_t pm_optm_1st_ref_fno_cnt_th; /* keep alive 1st threshold */
    uint32_t pm_optm_2nd_ref_fno_cnt_th; /* keep alive 2nd threshold */
    uint32_t pm_optm_3rd_ref_fno_cnt_th; /* disconnect */
    uint32_t pm_optm_4th_ref_fno_cnt_delta;

    uint32_t pm_optm_1st_start_time_ms;
    uint32_t pm_optm_2nd_start_time_ms;
    uint32_t pm_optm_3rd_start_time_ms;
    uint32_t pm_optm_4th_start_time_ms;
    #else /* end of TPSLL_PM_OPTM_EN */
    uint32_t alive_delay_time;
    uint32_t alive_delay_ref_tick;
    uint32_t alive_secondary_ref_cnt;
    uint32_t event_active_timeout;
    uint32_t secondary_active_timeout;
    #endif /* end of TPSLL_PM_OPTM_EN #else */

    uint32_t recon_acc;
    uint32_t pair_acc;
    uint32_t paired_cnt;
    uint32_t pair_timeout_cnt;
    uint32_t reconnected_cnt;
    uint32_t reconnect_timeout_cnt;
    uint32_t connected_active_tick;
    uint32_t disconnect_timeout_us;
    uint32_t no_event_cnt;
    uint32_t no_event_cnt_max;
    uint32_t no_event_cnt_max_set;
    uint32_t read_btn_tick;
    uint32_t stick_diff;
    #if TPSLL_CONN_QUICK_LOST_TIME_US
    uint32_t quick_discon_active_tick;
    #endif
    #if TPSLL_RR_CHG_WITH_RECON_EN
    uint32_t rr_chg_recon_time_tick;
    #endif
    #if TPSLL_RECON_RSSI_SCAN_EN
    uint32_t recon_rx_start_tick;
    #endif

    uint32_t pair_encrypt_key;      /* 4 bytes pair encrypt key */
    uint32_t connected_tick;
    int32_t tick_diff_max;

    #if TPSLL_KB_ACTIVE_PM_OPTM_EN
    uint32_t kb_alive_intervals_cnt; /* interval count */
    uint32_t kb_active_stage1_start_time_steps; /* n TPSLL_KB_ALIVE_TIME_STEP_MS */
    uint32_t kb_active_stage2_start_time_steps; /* n TPSLL_KB_ALIVE_TIME_STEP_MS */
    uint32_t kb_active_stage3_start_time_steps; /* n TPSLL_KB_ALIVE_TIME_STEP_MS */
    uint32_t kb_active_stage4_start_time_steps; /* n TPSLL_KB_ALIVE_TIME_STEP_MS */
    uint32_t kb_active_stage5_start_time_steps; /* n TPSLL_KB_ALIVE_TIME_STEP_MS */
    #ifdef TPSLL_KB_8K_ALIVE_STAGE6_FNO_MASK
    uint32_t kb_active_stage6_start_time_steps; /* n TPSLL_KB_ALIVE_TIME_STEP_MS */
    #endif
    #endif
    uint32_t * next_wakeup_tick_ptr;
    bool * is_stk_busy_ptr;
    bool * suspend_allowed_ptr;

    /* ---- 2-byte aligned ---- */
    uint16_t fno;                   /* Current frequency number */
    uint16_t fno_last;              /* Previous frequency number */
    #if TPSLL_PM_OPTM_EN
    uint16_t fno_last_ref;          /* Last reference frequency number */
    uint16_t pm_optm_3rd_interval_ms; /* level3 interval */
    uint16_t pm_optm_4th_interval_ms; /* level4 interval */
    #endif /* end of TPSLL_PM_OPTM_EN */
    uint16_t slot1_us;              /* Slot 1 timing in microseconds */

    uint16_t pending_rr_fno;        /* Pending report rate frequency number */
    uint16_t pending_sm_fno;        /* Pending sleep mode frequency number */
    uint16_t t0_us;                 /* T0 timing reference in microseconds */
    uint16_t d25f_timer0_anchor_us; /* Timer0 anchor offset in microseconds,
                                        updated on report rate change */
    uint16_t pr_cnt;                /* Pairing retry counter */

    uint16_t usb_vid;               /* USB Vendor ID */

    uint16_t alive_interval_cnt;
    uint16_t alive_interval_max;

    uint16_t keep_alive_sync_us;
    uint16_t suspend_wakeup_early_time_us;

    /* ---- 1-byte aligned ---- */
    uint8_t customer_key[ENCRYPT_KEY_MAX_LEN];  /* Customer defined encrypt key */
    uint8_t customer_key_len;                   /* Customer defined encrypt key length */
    uint8_t task_slot;              /* Current task slot */
    uint8_t step_3t1r;              /* 3T1R step configuration */
    uint8_t step_2t2r;              /* 2T2R step configuration */
    int8_t last_valid_slot_id;      /* Last valid slot identifier */

    uint8_t report_rate;            /* Current report rate */
    uint8_t last_rr;                /* Previous report rate */
    uint8_t pending_rr;             /* Pending report rate */
    uint8_t pending_rr_spp;         /* Pending report rate */
    uint8_t pair_st;                /* Pairing state */
    uint8_t pair_cnt;               /* Pairing counter */
    uint8_t tx_sn;                  /* Transmit sequence number */
    uint8_t rx_nesn;                /* Receive next expected sequence number */
    uint8_t chg_rr;                 /* Report rate change indicator */
    uint8_t chg_sm;                 /* Sleep mode change indicator */
    uint8_t used_sm;                /* Currently used sleep mode */
    uint8_t max_slot;               /* Maximum available slot count */

    uint8_t chg_dly_cnt;            /* Change delay counter */
    uint8_t reconn_chn;             /* Reconnection channel */
    // uint8_t ch_idx;                 /* Channel index */
    bool enter_conn_flag;           /* Connection entry flag */
    bool acked;                     /* Acknowledgment received flag */
    bool acked_ak;

    bool spp_flag;                  /* SPP mode flag */
    bool kb_flag;                   /* Keyboard mode flag */
    bool ak_flag;                   /* All key 8K data flag */
    bool ms_flag;                   /* Mouse mode flag */
    bool ll_flag;                   /* Link layer activity flag */
    bool is_rr_h2l;                 /* Report rate high-to-low change flag */
    bool is_rr_changing;       // Flag: report rate is currently changing (true when in progress)
    bool is_rr_chg_acked;           /* Report rate change acknowledged flag */
    bool is_sm_changing;            /* Sleep mode changing in progress flag */
    bool is_sm_chg_acked;           /* Sleep mode change acknowledged flag */
    bool precise_tx;                /* Precise transmission timing flag */

    uint8_t real_rr;//used for spp switch
    uint8_t spp_sw_thres;
    uint8_t peer_spp_flag;
    uint8_t sn_8k;
    uint8_t nesn_8k;
    bool wfi_en;
    bool wfi_inter_en;
    bool wfi_flag;
    bool suspend_en;
    bool suspend_flag;
    bool sleep_req;
    bool stimer_running;
    uint8_t allow_enter_sleep;
    bool is_spp_pre_acked;
    bool rx_over;
    bool cur_rx_flag;
    uint8_t key_length;
    // uint8_t btn_scan;
    #if TPSLL_KEEP_ALIVE_EN
    uint8_t synced;
    uint8_t sync_cntdwn;
    bool is_rr_just_changed;
    #endif
    bool is_rr_l2h;
    uint8_t slp_tx_cnt;
    bool s_8k;
    uint8_t stk_state;
    uint8_t user_rr;
    #if TPSLL_RR_CHG_WITH_RECON_EN
    uint8_t rr_chg_with_recon;
    uint8_t exit_spp_mode;
    #endif
    uint8_t pair_step;
    uint8_t recon_step;
    #if TPSLL_CONNECTING_CHECK_EN
    uint8_t connecting_check;
    #endif
    #if TPSLL_RECON_RSSI_SCAN_EN
    uint8_t recon_rssi_scanning_cnt;
    #endif
    uint8_t reconn_chn_idx;
#if TPSLL_SPP_NOT_SWITCH_EN
    uint8_t spp_quota_active;
    uint8_t ms_send_cnt;
#endif
    uint8_t spp_cnt;
    uint8_t time_synced_cnt;
    bool allow_spp;
    #if TPSLL_NEW_KEEP_ALIVE_EN
    bool is_keep_alive;
    uint8_t keep_alive_cnt;
    bool key_scan_disable;
    uint8_t ka_mask;
    bool is_keep_alive_sync;//True means need synchronized in keep alive mode
    #endif
    #if TPSLL_KB_ACTIVE_PM_OPTM_EN
    uint8_t kb_alive_fno_msk;
    uint8_t kb_alive_16ms_intervals;
    uint8_t kb_alive_16ms_intervals_msk;
    uint8_t kb_alive_16ms_intervals_shift;
    #endif
    #if TPSLL_N8K_FHSS_EN
    uint8_t afh_shift;
    #endif
    uint8_t srx_task_slot;
    #if TPSLL_PM_OPTM_EN
    uint8_t pm_optm_level;
    #endif
    bool ks_need_disable;
    bool ks_need_enable;
    bool reconn_from_request_sleep;
    bool is_rf_inited;
    bool is_alg_kb;
} ll_ctx_t;

_attribute_data_retention_ static ll_ctx_t  ll_ctx = {
    .interval_us = DONGLE_INTERVAL_US,
    .wfi_en = 1,
    .wfi_inter_en = 1,
    .wfi_flag = 0,
    .suspend_en = 1,
    .suspend_flag = 0,
    .allow_enter_sleep = 1,
    .key_length = 0,
    // .btn_scan = 1
    #if TPSLL_KEEP_ALIVE_EN
    .synced = 0,
    .sync_cntdwn = 0,
    .alive_interval_cnt = 0,
    .alive_interval_max = 0,
    .is_rr_just_changed = false,
    .key_scan_disable = false,
    .keep_alive_sync_us = 4000,//default set 4ms
    #endif
    .is_rr_l2h = false,
    .sleep_req = false,
    .slp_tx_cnt = 0,
    .paired_cnt = 0,
    .connected_active_tick = 0,
    .pair_timeout_cnt = 0,
    .cur_rx_flag = false,
    .stk_state = STATE_POWERON,
    .user_rr = APP_DEFAULT_REPORT_RATE,
    #if TPSLL_RR_CHG_WITH_RECON_EN
    .rr_chg_recon_time_tick = 0,
    .rr_chg_with_recon = RR_RECON_STEP_NONE,
    .exit_spp_mode = 0,
    #endif
    .recon_step = RECON_STEP_NONE,
    .pending_rr = 0,
    .pending_rr_spp = 0,
    #if TPSLL_RECON_RSSI_SCAN_EN
    #if TPSLL_NEW_RECON_EN
    .recon_rssi_scanning_cnt = 0,
    #else
    .recon_rssi_scanning_cnt = 0xff,
    #endif
    .recon_rx_start_tick = 0,
    #endif
    .reconn_chn_idx = 0,
    .rx_over = true,
    .allow_spp = false,
    .pair_acc = ACCESS_CODE,
    #if TPSLL_PM_OPTM_EN
    .pm_optm_1st_start_time_ms = 3 * 1000,
    .pm_optm_2nd_start_time_ms = 5 * 1000,
    .pm_optm_3rd_start_time_ms = 7 * 1000,
    .pm_optm_4th_start_time_ms = 0,//30 * 60 * 1000,
    .pm_optm_3rd_interval_ms = TPSLL_PM_OPTM_LEVEL3_DEFAULT_INTERVAl_MS,
    .pm_optm_4th_interval_ms = TPSLL_PM_OPTM_LEVEL4_DEFAULT_INTERVAl_MS,
    #else
    .secondary_active_timeout = 5,//default set 5 seconds
    .alive_delay_ref_tick = 1,
    #endif /* end of TPSLL_PM_OPTM_EN */
    .suspend_wakeup_early_time_us = APP_WAKEUP_EARLY_TIME_US,
    .reconn_from_request_sleep = false,
};

typedef enum
{
    TASK_SLOT0 = 0,
    TASK_SLOT1 = 1,
    TASK_SLOT2 = 2,
    TASK_SLOT3 = 3,

    STEP_3T1R_NONE = 0xB0,
    STEP_3T1R_1T = 0,
    STEP_3T1R_2T = 1,
    STEP_3T1R_3T = 2,
    STEP_3T1R_1R = 3,

    STEP_2T2R_NONE = 0xC0,
    STEP_2T2R_1T = 0,
    STEP_2T2R_2T = 1,

    P24G_PKT_TYPE_SYNC          = 0,
    P24G_PKT_TYPE_KB            = 0,
    P24G_PKT_TYPE_MS            = 1,
    P24G_PKT_TYPE_SPP           = 2,
    P24G_PKT_TYPE_LL            = 3,
    P24G_PKT_TYPE_PENDING_SPP   = 2,

}tpsll_stack_e;


#if DEBUG_RF_REPORT_CNT_ENABLE
volatile unsigned int  rf_tx_cnt=0;
volatile unsigned int  rf_tx_irq_cnt=0;

volatile unsigned int  rf_rx_irq_cnt=0;
volatile unsigned int  rf_rx_crc_ok_cnt=0;
volatile unsigned int  rf_rx_data_ok_cnt=0;
volatile unsigned int  rf_rx_crc_error_cnt=0;
volatile unsigned int  rf_rx_timeout_cnt=0;


volatile unsigned int  rf_time_cnt=0;
volatile unsigned int   rf_tx_debug_print=0;
#endif


enum
{
    PAIR_STATE_NONE = 0,
    PAIR_STATE_INIT = 1,
    PAIR_STATE_RSP = 2,
    PAIR_STATE_CFM = 3,
    PAIR_STATE_REJOIN = 4,
    KMD_CMD_PAIRING_REQ = 0X10,
    KMD_CMD_PAIRING_RSP = 0X11,
    KMD_CMD_CONN_REQ = 0X12,
    KMD_CMD_CONN_RSP = 0X13,
    KMD_CMD_CONN_DAT = 0X14,
    KMD_CMD_DETACH = 0X15,
    KMD_CMD_REJOIN = 0X16,
    KMD_CMD_POLL = 0X17,
};

// #if TPSLL_FHSS_EN
// #define CHN_TABL_LEN    TPSLL_HOP_CHN_NUM
// #else
#define CHN_TABL_LEN    4
// #endif

#if TPSLL_NEW_PAIR_EN
typedef struct pair_req_packet
{
    uint8_t cmd;
    uint16_t km_tag;
    uint32_t encrypt_key;
} __attribute__((packed)) __attribute__((aligned(4))) pair_req_packet_t;

typedef struct pair_rsp_packet
{
    uint8_t cmd;
    uint8_t mac[MAC_ADDR_LEN];
    uint16_t km_tag;
    uint16_t diff_us;
} __attribute__((packed)) __attribute__((aligned(4))) pair_rsp_packet_t;

typedef struct conn_req_packet
{
    uint8_t cmd;
    uint8_t rr;
    uint8_t mac[MAC_ADDR_LEN];
    uint16_t tag;
    uint16_t km_tag;
} __attribute__((packed)) __attribute__((aligned(4))) conn_req_packet_t;

typedef struct conn_rsp_packet
{
    uint8_t cmd;
    uint16_t fno_last;
    uint16_t d_tag;
    uint16_t km_tag;
    #if TPSLL_RECON_RSSI_SCAN_EN
    uint8_t chn_tab[TPSLL_HOP_CHN_NUM];
    #endif
} __attribute__((packed)) __attribute__((aligned(4))) conn_rsp_packet_t;
#else /* end of TPSLL_NEW_PAIR_EN */

typedef struct conn_req_packet
{
    uint32_t dma_len;
    uint8_t type;
    uint8_t len;
    uint8_t cmd;
    uint8_t mac[MAC_ADDR_LEN];
    uint8_t chn_table[CHN_TABL_LEN];
    uint16_t fno_last;
    // uint16_t dtag;
    uint16_t tag;
} __attribute__((packed)) __attribute__((aligned(4))) conn_req_packet_t;
#endif /* end of TPSLL_NEW_PAIR_EN #else */

typedef struct reconn_req_packet
{
    uint32_t dma_len;
    uint8_t type;
    uint8_t len;
    uint8_t cmd;
    #if TPSLL_NEW_RECON_EN
    uint8_t rr;
    #else
    uint8_t chn_table[CHN_TABL_LEN];
    uint16_t fno_last;
    #endif
    uint16_t tag;
    #if TPSLL_NEW_RECON_EN
    uint16_t km_tag;
    #endif
} __attribute__((packed)) __attribute__((aligned(4))) reconn_req_packet_t;

typedef struct reconn_rsp_packet
{
    uint32_t dma_len;
    uint8_t type;
    uint8_t len;
    uint8_t cmd;
    uint16_t d_tag;
    #if TPSLL_NEW_RECON_EN
    uint16_t fno_last;
    uint16_t diff_us;
    #else
    uint16_t km_tag;
    uint8_t rr;
    #endif
    #if TPSLL_RECON_RSSI_SCAN_EN
    uint8_t chn_tab[TPSLL_RECON_CHN_NUM];
    #endif
    uint16_t crc16;
} __attribute__((packed)) __attribute__((aligned(4))) reconn_rsp_packet_t;


_attribute_data_retention_
_attribute_aligned_(4) ll_inf_t ll_inf;


#if TPSLL_N8K_CHN_EXPAND_EN
static uint8_t chn_table[8] = {0};
#else
uint8_t chn_table[CHN_TABL_LEN] = {8, 17, 22, 32};
#endif
uint8_t chn_list[3] = {0};

typedef enum
{
    DBG_IO_IDX0 = 0,
    DBG_IO_IDX1,
    DBG_IO_IDX2,
    DBG_IO_IDX3,
    DBG_IO_IDX4,
    DBG_IO_IDX5,
    DBG_IO_IDX6,
    DBG_IO_IDX7,
    DBG_IO_IDX_MAX,
} debug_io_index_t;

static uint16_t dbg_io[DBG_IO_IDX_MAX] = {0, 0, 0, 0, 0, 0, 0, 0};

#if TPSLL_FHSS_EN
static volatile uint8_t g_hop_chn_idx_tab_next[TPSLL_HOP_CHN_NUM] = {0, 0, 0};

#if 0
static uint8_t g_fhss_valid_chn_tab[TPSLL_FHSS_CHN_MAX_NUM] = { /*(n)wifi*/
    0,17, 1,19, 2,21, 3,22, 4,24, 5,27,
    8,28,10,29,12,32,14,34,15,35,16,36
};
#else
static int8_t g_fhss_valid_chn_tab[TPSLL_FHSS_CHN_MAX_NUM] = { /*(n)wifi*/
    0, 19,  1, 21,  2, 22,  4, 24,  5, 27,  7, 28,
    8, 29, 12, 32, 14, 33, 15, 34, 16, 35, 17, 36
};

#if !TPSLL_N8K_FHSS_EN
#define  NON_8K_CHN_NUM  32
static int8_t g_n8k_chn_tab[NON_8K_CHN_NUM] = { /*(n)wifi*/
    0,  19, 33, 21, 2,  22,  32, 16,  5,  27,  7,  35,
    8,  29, 4,  12, 26, 1,   34, 15,  24, 17,  28, 36, 
    3,  14, 37, 6,  30, 13,  23, 9
};
#endif


// static uint8_t invalid_chnl_tab[] = {
//     3,  6,  9, 10, 11,
//     13, 18, 20, 23, 25,
//     26, 30, 31, 37, 38, 39
// };
#endif

#if TPSLL_NEW_PAIR_EN
static int8_t g_pair_valid_chn_tab[TPSLL_PAIR_CHN_MAX_NUM] = {
    0, 21, 8, 22, 34, 35, 17, 36
};
#endif

static inline bool tpsll_all_fifo_is_empty(void)
{
    return pp_fifo_is_empty(km_fifo_ptr) && pp_fifo_is_empty(&ll_fifo) && pp_fifo_is_empty(spp_fifo_ptr);
}

/**
 * The channel list for current interval has already been generated in advance before this point
 */
static inline bool tpsll_is_afh_slot(void)
{
    static uint32_t last;

    #if TPSLL_N8K_FHSS_EN
    uint32_t cur = ll_ctx.fno >> ll_ctx.afh_shift;
    #else
    uint32_t cur = ll_ctx.fno >> TPSLL_FHSS_AFH_SLOT_SHIFT;
    #endif
    uint32_t diff = cur ^ last;

    last = cur;

    return diff != 0;
}

static volatile uint16_t g_fhss_idx_buf_data_next = 0;
static volatile uint16_t g_fhss_idx_buf_data_cur = 0;
#endif

#if TPSLL_RECON_RSSI_SCAN_EN
typedef enum
{
    STATE_SCAN_IDLE = 0,
    STATE_SCAN_RUNNING,
    STATE_SCAN_DONE,
} RSSI_SCAN_STATE;

uint32_t rssi_result[RSSI_SCAN_CHANNEL_COUNT] = {0};
uint16_t scan_counts[RSSI_SCAN_CHANNEL_COUNT] = {0};
uint16_t g_chn_rssi_max[RSSI_SCAN_CHANNEL_COUNT] = {0};

#define REG_RSSI_MAX_BB REG_ADDR8(REG_TL_MODEM_BASE_ADDR + 0x62) //rssi_max_bb
#define CLR_REG_RSSI_MAX_BB()       REG_ADDR8(REG_TL_MODEM_BASE_ADDR + 0x24) = 0x04
/* REG_RXCHN_EN moved to d24g_stack_common.h */

volatile uint8_t current_channel = 0;
volatile uint16_t current_round = 0;
volatile RSSI_SCAN_STATE scan_state = STATE_SCAN_IDLE;

static inline int is_channel_valid(int ch)
{
    TL_UNUSED(ch);
    // return !((ch == 0) || (ch >= 10 && ch <= 11) || (ch >= 36 && ch <= 39));
    // for (int idx = 0; idx < sizeof(invalid_chnl_tab); idx++) {
    //     if (invalid_chnl_tab[idx] == ch) {
    //         return false;
    //     }
    // }

    return true;
}

void rf_rssi_scan_start(void)
{
    tmemset(rssi_result, 0, sizeof(rssi_result));
    tmemset(scan_counts, 0, sizeof(scan_counts));
    tmemset(g_chn_rssi_max, 0, sizeof(g_chn_rssi_max));

    current_channel = 0;
    current_round = 0;
    scan_state = STATE_SCAN_RUNNING;

    current_channel = 0;
    while (!is_channel_valid(current_channel))
    {
        current_channel++;
    }

    rf_set_ble_chn(current_channel);
    pp_rf_start_srx(stimer_get_tick(), RSSI_SCAN_WIN_US);
}

void rf_select_best_channels(uint8_t *out_channels, uint8_t num)
{
    uint8_t rssi_avg[RSSI_SCAN_CHANNEL_COUNT];

    for (uint8_t i = 0; i < RSSI_SCAN_CHANNEL_COUNT; ++i)
    {
        if (scan_counts[i])
        {
            rssi_avg[i] = rssi_result[i] / scan_counts[i];
        }
        else
        {
            rssi_avg[i] = 127;
        }
    }

    uint8_t channel_idx[RSSI_SCAN_CHANNEL_COUNT];
    for (uint8_t i = 0; i < RSSI_SCAN_CHANNEL_COUNT; ++i)
        channel_idx[i] = i;

    for (uint8_t i = 0; i < RSSI_SCAN_CHANNEL_COUNT - 1; ++i)
    {
        for (uint8_t j = i + 1; j < RSSI_SCAN_CHANNEL_COUNT; ++j)
        {
            if (rssi_avg[channel_idx[j]] < rssi_avg[channel_idx[i]])
            {
                uint8_t tmp = channel_idx[i];
                channel_idx[i] = channel_idx[j];
                channel_idx[j] = tmp;
            }
        }
    }

    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
    // tlkapi_send_string_data(APP_LOG_EN, "chn idx rssi sort(s->b)", channel_idx, 40);
    // tlkapi_send_string_data(APP_LOG_EN, "rssi_avg", rssi_avg, 40);
    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);

    uint8_t selected = 0;
    for (uint8_t i = 0; i < RSSI_SCAN_CHANNEL_COUNT && selected < num; ++i)
    {
        uint8_t ch = channel_idx[i];
        if (!is_channel_valid(ch)) continue;

        #if ENFORCE_CHANNEL_SPACING
        uint8_t too_close = 0;
        for (uint8_t j = 0; j < selected; ++j)
        {
            if (tabs(ch - out_channels[j]) < MIN_CHANNEL_SPACING)
            {
                too_close = 1;
                break;
            }
        }
        if (too_close)
            continue;
        #endif

        out_channels[selected++] = ch;
    }

    #if 0
    DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
    printf("after spacing:\n");
    for (int i = 0; i < num; ++i)
    {
        printf("[%d]: %d  ", i, out_channels[i]);
    }
    printf("\n");
    DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
    #endif
}

#if TPSLL_RECON_RSSI_SCAN_EN && !TPSLL_NEW_RECON_EN
static void rf_rssi_scan_param_restore(void)
{
    rf_access_code_comm(ll_ctx.pair_acc);
    write_reg8(REG_RXCHN_EN, 1);
    write_reg8(0x17051a,36);//sync thr
    rf_set_irq_mask(FLD_RF_IRQ_TX|FLD_RF_IRQ_RX|FLD_RF_IRQ_RX_TIMEOUT|FLD_RF_IRQ_FIRST_TIMEOUT);
}


static void rf_rssi_scan(void)
{
    core_interrupt_enable();
    rf_rssi_scan_param_set();
    rf_rssi_scan_start();

    while (1)
    {
        if (scan_state == STATE_SCAN_DONE)
        {
            uint8_t best_channels[RSSI_SCAN_CHANNEL_COUNT];
            // uint8_t best_channels[RSSI_SCAN_TOP_CHANNEL_COUNT];
            // uint16_t wifi_chn_rssi_sum[TPSLL_WIFI_CHANNEL_COUNT];
            // uint8_t wifi_chn_rssi_sorted_idx[TPSLL_WIFI_CHANNEL_COUNT];

            #if TPSLL_RECON_RSSI_SCAN_MAX_EN
            // uint8_t fhss_valid_chn_rssi_sorted_idx[TPSLL_FHSS_CHN_MAX_NUM];
            // uint8_t k = 0;
            tpsll_bubble_sort_rssi_index(g_chn_rssi_max, RSSI_SCAN_CHANNEL_COUNT, best_channels);

            // for (int i = 0; i < RSSI_SCAN_CHANNEL_COUNT; i++) {
            //     for (int j = 0; j < TPSLL_FHSS_CHN_MAX_NUM; j++) {
            //         if (best_channels[i] == g_fhss_valid_chn_tab[j]) {
            //             fhss_valid_chn_rssi_sorted_idx[k] = best_channels[i];
            //             k++;
            //             break;
            //         }
            //     }
            // }
            #else
            // rf_select_best_channels(best_channels, RSSI_SCAN_TOP_CHANNEL_COUNT);
            rf_select_best_channels(best_channels, RSSI_SCAN_CHANNEL_COUNT);
            #endif
            #if 0

            wifi_chn_rssi_sum[0] = g_chn_rssi_max[37] / 2 + g_chn_rssi_max[0] + g_chn_rssi_max[1] + g_chn_rssi_max[2] +
                                g_chn_rssi_max[3] + g_chn_rssi_max[4] + g_chn_rssi_max[5] + g_chn_rssi_max[6] +
                                g_chn_rssi_max[7] + g_chn_rssi_max[8] + g_chn_rssi_max[9] / 2;
            wifi_chn_rssi_sum[1] = g_chn_rssi_max[2] + g_chn_rssi_max[3] + g_chn_rssi_max[4] + g_chn_rssi_max[5] +
                                g_chn_rssi_max[6] + g_chn_rssi_max[7] + g_chn_rssi_max[8] + g_chn_rssi_max[9] +
                                g_chn_rssi_max[10] + g_chn_rssi_max[38];
            wifi_chn_rssi_sum[2] = g_chn_rssi_max[4] / 2 + g_chn_rssi_max[5] + g_chn_rssi_max[6] + g_chn_rssi_max[7] +
                                g_chn_rssi_max[8] + g_chn_rssi_max[9] + g_chn_rssi_max[10] + g_chn_rssi_max[38] +
                                g_chn_rssi_max[11] + g_chn_rssi_max[12] + g_chn_rssi_max[13] / 2;
            wifi_chn_rssi_sum[3] = g_chn_rssi_max[7] + g_chn_rssi_max[8] + g_chn_rssi_max[9] + g_chn_rssi_max[10] +
                                g_chn_rssi_max[38] + g_chn_rssi_max[11] + g_chn_rssi_max[12] + g_chn_rssi_max[13] +
                                g_chn_rssi_max[14] + g_chn_rssi_max[15];
            wifi_chn_rssi_sum[4] = g_chn_rssi_max[9] / 2 + g_chn_rssi_max[10] + g_chn_rssi_max[38] + g_chn_rssi_max[11] +
                                g_chn_rssi_max[12] + g_chn_rssi_max[13] + g_chn_rssi_max[14] + g_chn_rssi_max[15] +
                                g_chn_rssi_max[16] + g_chn_rssi_max[17] + g_chn_rssi_max[18] / 2;
            wifi_chn_rssi_sum[5] = g_chn_rssi_max[11] + g_chn_rssi_max[12] + g_chn_rssi_max[13] + g_chn_rssi_max[14] +
                                g_chn_rssi_max[15] + g_chn_rssi_max[16] + g_chn_rssi_max[17] + g_chn_rssi_max[18] +
                                g_chn_rssi_max[19] + g_chn_rssi_max[20];
            wifi_chn_rssi_sum[6] = g_chn_rssi_max[13] / 2 + g_chn_rssi_max[14] + g_chn_rssi_max[15] + g_chn_rssi_max[16] +
                                g_chn_rssi_max[17] + g_chn_rssi_max[18] + g_chn_rssi_max[19] + g_chn_rssi_max[20] +
                                g_chn_rssi_max[21] + g_chn_rssi_max[22] + g_chn_rssi_max[23] / 2;
            wifi_chn_rssi_sum[7] = g_chn_rssi_max[16] + g_chn_rssi_max[17] + g_chn_rssi_max[18] + g_chn_rssi_max[19] +
                                g_chn_rssi_max[20] + g_chn_rssi_max[21] + g_chn_rssi_max[22] + g_chn_rssi_max[23] +
                                g_chn_rssi_max[24] + g_chn_rssi_max[25];
            wifi_chn_rssi_sum[8] = g_chn_rssi_max[18] /2 + g_chn_rssi_max[19] + g_chn_rssi_max[20] + g_chn_rssi_max[21] +
                                g_chn_rssi_max[22] + g_chn_rssi_max[23] + g_chn_rssi_max[24] + g_chn_rssi_max[25] +
                                g_chn_rssi_max[26] + g_chn_rssi_max[27] + g_chn_rssi_max[28] / 2;
            wifi_chn_rssi_sum[9] = g_chn_rssi_max[21] + g_chn_rssi_max[22] + g_chn_rssi_max[23] + g_chn_rssi_max[24] +
                                g_chn_rssi_max[25] + g_chn_rssi_max[26] + g_chn_rssi_max[27] + g_chn_rssi_max[28] +
                                g_chn_rssi_max[29] + g_chn_rssi_max[30];
            wifi_chn_rssi_sum[10] = g_chn_rssi_max[23] / 2 + g_chn_rssi_max[24] + g_chn_rssi_max[25] + g_chn_rssi_max[26] +
                                g_chn_rssi_max[27] + g_chn_rssi_max[28] + g_chn_rssi_max[29] + g_chn_rssi_max[30] +
                                g_chn_rssi_max[31] + g_chn_rssi_max[32] + g_chn_rssi_max[33] / 2;
            wifi_chn_rssi_sum[11] = g_chn_rssi_max[26] + g_chn_rssi_max[27] + g_chn_rssi_max[28] + g_chn_rssi_max[29] +
                                g_chn_rssi_max[30] + g_chn_rssi_max[31] + g_chn_rssi_max[32] + g_chn_rssi_max[33] +
                                g_chn_rssi_max[34] + g_chn_rssi_max[35];
            wifi_chn_rssi_sum[12] = g_chn_rssi_max[28] + g_chn_rssi_max[29] + g_chn_rssi_max[30] + g_chn_rssi_max[31] +
                                g_chn_rssi_max[32] + g_chn_rssi_max[33] + g_chn_rssi_max[34] + g_chn_rssi_max[35] +
                                g_chn_rssi_max[36] + g_chn_rssi_max[39];

            tpsll_bubble_sort_rssi_index(wifi_chn_rssi_sum, TPSLL_WIFI_CHANNEL_COUNT, wifi_chn_rssi_sorted_idx);

            printf("wifi:");
            for (int i = 0; i < TPSLL_WIFI_CHANNEL_COUNT; i++) {
                printf("[%d]%d", wifi_chn_rssi_sorted_idx[i] + 1, wifi_chn_rssi_sum[wifi_chn_rssi_sorted_idx[i]]);
                // printf("<%d,%d>\n", i + 1, wifi_chn_rssi_sum[i]);
            }
            printf("\n");
            #endif

            #if 0
            tlkapi_send_string_data(APP_LOG_EN, "best chns(H)", best_channels, RSSI_SCAN_TOP_CHANNEL_COUNT);
            #endif

            // tmemcpy(reconn_chn_table, best_channels, TPSLL_RECON_CHN_NUM);
            int k = 0;

            for (int i = 0; i < RSSI_SCAN_CHANNEL_COUNT && k < TPSLL_RECON_CHN_NUM; i++) {
                for (int j = 0; j < TPSLL_FHSS_CHN_MAX_NUM; j++)
                if (best_channels[i] == g_fhss_valid_chn_tab[j]) {
                    reconn_chn_table[k] = g_fhss_valid_chn_tab[j];
                    k++;
                }
            }
            // tlkapi_printf(APP_LOG_EN, "rcn(%d)(%d)(%d)\n", reconn_chn_table[0], reconn_chn_table[1], reconn_chn_table[2]);

            // tlkapi_printf(1, "fhss chns:<\n");
            // for(int i = 0; i < TPSLL_FHSS_CHN_MAX_NUM; i++) {
            //     tlkapi_printf(1, "[%d]%d\n", i, g_fhss_valid_chn_tab[i]);
            // }
            // tlkapi_printf(1, ">\n");
            break;
        }
    }

    rf_rssi_scan_param_restore();
}
#endif

_attribute_ram_code_sec_ static void proc_rssi_scan(void)
{
    uint8_t rssi = REG_RSSI_MAX_BB;
    rssi_result[current_channel] += rssi;
    scan_counts[current_channel]++;
    if (g_chn_rssi_max[current_channel] < rssi)
        g_chn_rssi_max[current_channel] = rssi; //max rssi

    CLR_REG_RSSI_MAX_BB();
    // printf("current_channel: %d  rssi:%d \n", current_channel, rssi);

    do {
        current_channel++;
    } while (current_channel < RSSI_SCAN_CHANNEL_COUNT && !is_channel_valid(current_channel));

    if (current_channel >= RSSI_SCAN_CHANNEL_COUNT)
    {
        // delay_ms(1);
        current_channel = 0;
        while (!is_channel_valid(current_channel))
        {
            current_channel++;
        }
        current_round++;

        if (current_round >= RSSI_SCAN_ROUNDS)
        {
            scan_state = STATE_SCAN_DONE;
            return;
        }
    }
    rf_set_ble_chn(current_channel);
    pp_rf_start_srx(stimer_get_tick(), RSSI_SCAN_WIN_US);
}
#endif // TPSLL_RECON_RSSI_SCAN_EN end

void app_d24g_set_report_rate(uint8_t rr);

_attribute_ram_code_sec_ void tpsll_reset_spp_fifo(void)
{//handle in d25f
    // unsigned int r = core_interrupt_disable();
    // spp_fifo_ptr->wptr = 0;
    // spp_fifo_ptr->rptr = 0;
    // ll_ctx.spp_flag = false;
    // core_restore_interrupt(r);
}

#if KB_SPP_FRAG_EN
_attribute_ram_code_sec_ static void tpsll_spp_rx_assemble_reset(void)
{
    g_spp_rx_assemble.cur_len = 0;
    g_spp_rx_assemble.is_receiving = false;
    g_spp_rx_assemble.opcode = SPP_OPCODE_DEFAULT;
}
#endif

// // Utilize a large-step jumping approach to avoid repetition, for example: base, base+3, base+7, base+11
// const uint8_t chn_steps[3] = { 0, 7, 11 };
//
//static inline int tick1_exceed_tick2(unsigned int tick1, unsigned int tick2)
//{
//    return (unsigned int)(tick1 - tick2) < 0xc0000000; // 3/4 of max, about 130 second
//}

static void tpsll_stack_set_pairing_info(const tpsll_stack_var_t *p_stack_var)
{
    tmemcpy(ll_inf.peer_mac, p_stack_var->peer_mac, MAC_ADDR_LEN);
    tmemcpy(ll_inf.mac, p_stack_var->mac, MAC_ADDR_LEN);

    ll_inf.peer_tag = p_stack_var->peer_tag;
    ll_inf.tag = p_stack_var->local_tag;
    tlkapi_printf(APP_LOG_EN, "tag:%x\n", ll_inf.tag);
}

static void tpsll_stack_get_pairing_info(tpsll_stack_var_t *p_stack_var)
{
    tmemcpy(p_stack_var->peer_mac, ll_inf.peer_mac, MAC_ADDR_LEN);
    p_stack_var->side_id = ll_inf.side_id;
    p_stack_var->pair_success_flag = ll_inf.pair_success_flag;
}

uint8_t tpsll_stack_get_var( tpsll_stack_var_t *p_stack_var, tpsll_stack_var_id_e var_id)
{
    if (p_stack_var == NULL)
    {
        return TLK_ERR_NULL;
    }
    if (var_id >= TPSLL_STACK_VAR_MAX)
    {
        return TLK_ERR_INVALID_PARAM;
    }

    switch (var_id) {
        case TPSLL_STACK_VAR_SET_PAIRING_INFO:
            tpsll_stack_get_pairing_info(p_stack_var);
            break;

        default:
            break;
    }

    return TLK_SUCCESS;
}

uint8_t tpsll_stack_set_var(const tpsll_stack_var_t *p_stack_var, tpsll_stack_var_id_e var_id)
{
    if (p_stack_var == NULL)
    {
        return TLK_ERR_NULL;
    }
    if(var_id >= TPSLL_STACK_VAR_MAX)
    {
        return TLK_ERR_INVALID_PARAM;
    }

    switch (var_id) {
        case TPSLL_STACK_VAR_SET_PAIRING_INFO:
            tpsll_stack_set_pairing_info(p_stack_var);
            break;
        case TPSLL_STACK_VAR_SET_SUCCESS_FLG:
            ll_inf.pair_success_flag = p_stack_var->pair_success_flag;
            break;
        default:
            break;
    }

    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ static void p24g_evt_cb(tpsll_evt_t evt, void *data, uint16_t len)
{
    if (evt < TPSLL_EVT_MAX && p24g_callbacks[evt]) {
        p24g_callbacks[evt](evt, data, len);
    }
}

uint8_t tpsll_register_callback(tpsll_evt_t evt, tpsll_evt_callback_t cb)
{
    if (evt >= TPSLL_EVT_MAX) {
        return -1;
    }
    p24g_callbacks[evt] = cb;
    return 0;
}

uint8_t tpsll_inter_register_callback(tpsll_inter_evt_t evt, tpsll_evt_callback_t cb)
{
    if (evt >= TPSLL_INTER_EVT_MAX || evt < TPSLL_INTER_EVT_SPP_SEND_COMP) {
        return -1;
    }
    p24g_callbacks[evt] = cb;
    return 0;
}

#ifndef ENTER_CRITICAL
unsigned int r_int;

#define ENTER_CRITICAL()    do { (r_int) = core_interrupt_disable(); } while(0)
#define EXIT_CRITICAL()     do { core_restore_interrupt(r_int); } while(0)
#endif

static stack_event_queue_t g_evt_q = { .head = 0, .tail = 0 };


_attribute_ram_code_sec_ uint8_t tpsll_wfi_en(bool en)
{
    ll_ctx.wfi_en = en;
    return TLK_SUCCESS;
}

static inline uint8_t tpsll_wfi_inter_en(bool en)
{
    ll_ctx.wfi_inter_en = en;
    return TLK_SUCCESS;
}

static inline void tpsll_suspend_inter_en(bool en)
{
    ll_ctx.suspend_en = en;
}

static inline bool tpsll_is_keep_alive_active(void)
{
    #if TPSLL_PM_OPTM_EN
    return (ll_ctx.pm_optm_level & (TPSLL_PM_OPTM_LEVEL1 | TPSLL_PM_OPTM_LEVEL2));
    #else
    return !ll_ctx.alive_delay_ref_tick;
    #endif
}

static inline bool tpsll_is_active_state(void)
{
    #if TPSLL_PM_OPTM_EN
    return !ll_ctx.pm_optm_level;
    #else
    return ll_ctx.alive_delay_ref_tick;
    #endif
}

static inline bool tpsll_pm_optm_is_long_suspend_support_level_active(void)
{
    #if TPSLL_PM_OPTM_EN
    return (ll_ctx.pm_optm_level & TPSLL_PM_OPTM_LEVEL2);
    #else
    return !ll_ctx.alive_secondary_ref_cnt;
    #endif
}

static inline bool tpsll_pm_optm_is_level2_active(void)
{
    #if TPSLL_PM_OPTM_EN
    return (ll_ctx.is_keep_alive && (ll_ctx.pm_optm_level & TPSLL_PM_OPTM_LEVEL2));
    #else
    return false;
    #endif
}
// _attribute_ram_code_sec_ void tpsll_set_event_active_timeout_ms(uint32_t ms) 
// {
//     #if TPSLL_KEEP_ALIVE_EN
//     #if TPSLL_PM_OPTM_EN
//     if (tpsll_pm_optm_is_long_suspend_support_level_active()) {
//         ll_ctx.pm_optm_ref_fno_cnt = 1;
//     } else {
//         ll_ctx.pm_optm_ref_fno_cnt = 2;
//     }
//     ll_ctx.fno_last_ref = ll_ctx.fno_last;
//     #else /* end of TPSLL_PM_OPTM_EN */
//     ll_ctx.event_active_timeout = ms * 1000;
//     ll_ctx.alive_delay_ref_tick = clock_time() | 1;
//     #endif /* end of TPSLL_PM_OPTM_EN #else */
//     #endif
//     #if TPSLL_NO_EVENT_PWR_DOWN_EN
//     ll_ctx.no_event_cnt = 0;
//     #endif
// }

static inline void tpsll_event_active_time_fresh_value(uint32_t stick) 
{
    #if TPSLL_KEEP_ALIVE_EN
    #if TPSLL_PM_OPTM_EN
    (void)stick;
    if (tpsll_pm_optm_is_long_suspend_support_level_active() &&
            (ll_ctx.report_rate & (REPORT_RATE_8K | REPORT_RATE_4K | REPORT_RATE_2K | REPORT_RATE_1K | REPORT_RATE_500))) { //Q|TODO: 125, 250
        ll_ctx.pm_optm_ref_fno_cnt = 1;
    } else {
        ll_ctx.pm_optm_ref_fno_cnt = 2;
    }

    ll_ctx.fno_last_ref = ll_ctx.fno_last;
    #else /* end of TPSLL_PM_OPTM_EN */
    ll_ctx.alive_delay_ref_tick = stick | 1;
    #endif /* end of TPSLL_PM_OPTM_EN #else */
    #endif
    // #if TPSLL_NO_EVENT_PWR_DOWN_EN
    // ll_ctx.no_event_cnt = 0;
    // #endif
}

uint8_t tpsll_pm_optm_time_refresh(void)
{
    #if TPSLL_KB_ACTIVE_PM_OPTM_EN
    ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
    ll_ctx.kb_alive_intervals_cnt = 0;
    #endif
    #if TPSLL_PM_OPTM_EN
    tpsll_event_active_time_fresh_value(0);
    #else
    tpsll_event_active_time_fresh_value(clock_time());
    #endif
    return TLK_SUCCESS;
}

static inline void tpsll_event_active_timeout_check(void)
{
#if TPSLL_KEEP_ALIVE_EN && !TPSLL_PM_OPTM_EN
    if ((!(ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250))) && ll_ctx.alive_delay_ref_tick) {
        if (ll_ctx.is_rr_just_changed) {
            if (clock_time_exceed(ll_ctx.alive_delay_ref_tick, ll_ctx.alive_delay_time)) {
                ll_ctx.is_rr_just_changed = false;
                ll_ctx.alive_delay_ref_tick = 0;
            }
        } else if (ll_ctx.event_active_timeout && clock_time_exceed(ll_ctx.alive_delay_ref_tick, ll_ctx.event_active_timeout)) {
            ll_ctx.alive_delay_ref_tick = 0;
        }
    }
#endif
}

static inline bool stack_evt_post_from_isr(int evt, const void *data, uint16_t len)
{
    if (evt >= TPSLL_INTER_EVT_MAX) return false;


    if (len > STACK_EVT_DATA_MAX_LEN) {

        len = STACK_EVT_DATA_MAX_LEN;
    }

    ENTER_CRITICAL();
    stack_event_queue_t *q = &g_evt_q;
    if (stack_evt_queue_full(q)) {
        /* queue full -> drop event */
        EXIT_CRITICAL();
        return false;
    }
    uint16_t idx = q->head;
    q->buf[idx].evt = evt;
    q->buf[idx].len = len;
    if (data != NULL && len > 0) {
        /* copy minimal bytes */
        tmemcpy(q->buf[idx].data, data, len);
    }
    /* advance head */
    q->head = next_idx(idx);
    EXIT_CRITICAL();
    return true;
}


static void stack_poll_events(uint16_t max_process)
{
    uint16_t processed = 0;
    stack_event_queue_t *q = &g_evt_q;

    while (!stack_evt_queue_empty(q)) {
        if (max_process != 0 && processed >= max_process) break;

        /* Guard access while popping */
        ENTER_CRITICAL();
        if (stack_evt_queue_empty(q)) {
            EXIT_CRITICAL();
            break;
        }

        uint16_t idx = q->tail;
        stack_event_elem_t evt_elem;
 
        evt_elem = q->buf[idx]; /* struct copy */
        q->tail = next_idx(idx);
        EXIT_CRITICAL();

        tpsll_evt_callback_t cb = NULL;
        if ( (uint8_t)evt_elem.evt < (uint8_t)TPSLL_INTER_EVT_MAX) {
            cb = p24g_callbacks[evt_elem.evt];
        }
        if (cb) {
            cb(evt_elem.evt, (evt_elem.len ? evt_elem.data : NULL), evt_elem.len);
        }
        processed++;
    }
}

static void tpsll_stack_event_handler(tpsll_inter_evt_t evt, void *data, uint16_t len)
{
    TL_UNUSED(data);
    TL_UNUSED(len);

    switch(evt) {
    case TPSLL_INTER_EVT_SPP_SEND_COMP:
    {
        uint8_t *payload = (uint8_t*)data;
        uint16 pdu_len = len-9;
        void *user_arg = *(void **)(&payload[pdu_len + 4]);
        fifo_cb_t spp_cb = *(fifo_cb_t *)(&payload[pdu_len]);
        if (spp_cb)
        {
            spp_cb(payload, pdu_len, true, user_arg);
        }
        break;
    }

    // case TPSLL_INTER_EVT_KS_DISABLE:
    //     if (ll_ctx.is_alg_kb && !ll_ctx.key_scan_disable) {
    //         s_app_cb->app_ks_pwm_mode_disable();
    //         ll_ctx.key_scan_disable = true;
    //     }
    //     break;

    default:
        break;
    }
}

_attribute_ram_code_sec_ bool app_mouse_has_data( t_mouse_data* data) 
{
    static uint8_t last_btn = 0;
    // return (data->btn != 0) ||
    //        (data->x != 0)   ||
    //        (data->y != 0)   ||
    //        (data->z_wheel != 0);

    if ((last_btn != data->btn) ||
            (data->x != 0)   ||
            (data->y != 0)   ||
            (data->z_wheel != 0))
    {
        last_btn = data->btn;
        return true;
    }
    return false;
}


void pp_fill_paring_pkt(void)
{
    pp_rf_set_tx_len(1);
    rf_tx_packet.buf[0] = PAIRING_RSP;
}

_attribute_ram_code_sec_ static inline void p24g_set_snnesn(void)
{
    SET_SN(rf_tx_packet.snnesn, ll_ctx.tx_sn);
    SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);
}


#if RF_IS_HAS_PN
    volatile    signed char rf_adv_chn[3]={35,17,15};
    volatile    signed char rf_con_chn[4]={7,25,11,30};
#else
    volatile    signed char rf_adv_chn[3]={76,40,36};
    volatile    signed char rf_con_chn[4]={18,56,28,66};
#endif

//400ns @192M
_attribute_ram_code_sec_ void calc_hop_channels(uint16_t fno, uint8_t ch_mask) 
{

    #if 1
    (void)ch_mask;
    #if TPSLL_8K_CHN_HOP_TURN_EN
    uint32_t seed = fno % CHN_TABL_LEN;
    chn_list[0] = chn_table[seed];
    chn_list[1] = chn_table[(seed + 1) % (CHN_TABL_LEN)];
    chn_list[2] = chn_table[(seed + 2) % (CHN_TABL_LEN)];
    #else
    uint32_t seed = prng( (((uint32_t)fno << 9) ^ ((uint32_t)fno >> 3)) + 0x73 );
    // #if (CHN_TABL_LEN == 4)
    // chn_list[0] = chn_table[seed & (CHN_TABL_LEN - 1)];
    // chn_list[1] = chn_table[(seed + 1) & (CHN_TABL_LEN - 1)];
    // chn_list[2] = chn_table[(seed + 2) & (CHN_TABL_LEN - 1)];
    // #else
    chn_list[0] = chn_table[seed % TPSLL_HOP_CHN_NUM];
    chn_list[1] = chn_table[(seed + 1) % TPSLL_HOP_CHN_NUM];
    chn_list[2] = chn_table[(seed + 2) % TPSLL_HOP_CHN_NUM];
    // #endif
    #endif
    // (void)fno;
    // chn_list[0] = 35;//chn_table[0];
    // chn_list[1] = 35;//chn_table[0];
    // chn_list[2] = 35;//chn_table[0];


    #else
    uint32_t seed = prng(fno * 0xA529 + 0x73);

    uint8_t m = ch_mask & 0x0F;
    uint8_t c = bit_count(m);
    if (c == 0) return; 

    uint8_t chn_idx[4];
    uint8_t n = 0;

    if (m & 0x01) chn_idx[n++] = 0;
    if (m & 0x02) chn_idx[n++] = 1;
    if (m & 0x04) chn_idx[n++] = 2;
    if (m & 0x08) chn_idx[n++] = 3;

    uint8_t r0 = seed & 0x03;
    uint8_t r1 = (seed >> 4) & 0x03;
    uint8_t r2 = (seed >> 8) & 0x03;

    if (n == 1) {
        uint8_t idx = chn_idx[0];
        chn_list[0] = chn_table[idx];
        chn_list[1] = chn_table[idx];
        chn_list[2] = chn_table[idx];
    } else if (n == 2) {
        uint8_t i0 = chn_idx[r0 % 2];
        uint8_t i1 = chn_idx[r1 % 2];
        uint8_t i2 = chn_idx[r2 % 2];
        chn_list[0] = chn_table[i0];
        chn_list[1] = chn_table[i1];
        chn_list[2] = chn_table[i2];
    } else {
        uint8_t i0 = chn_idx[r0 % n];

        uint8_t i1;
        do {
            i1 = chn_idx[r1 % n];
            r1++;
        } while (i1 == i0);

        uint8_t i2;
        do {
            i2 = chn_idx[r2 % n];
            r2++;
        } while ((i2 == i0) || (i2 == i1));

        chn_list[0] = chn_table[i0];
        chn_list[1] = chn_table[i1];
        chn_list[2] = chn_table[i2];
    }
#endif
}


#define SYNCWORD_MASK          0xA5A5A5A5u
#define BT_INQUIRY_ACCESS_CODE    0x9E8B33U
#define BLE_BROADCAST_ACCESS_CODE 0x8E89BED6U


_attribute_ram_code_sec_ void p24g_gen_ac_and_channel(uint8_t mac_addr[6], uint32_t *ac, uint8_t *channel)
{
    uint16_t c16 = p24g_crc16(0xFFFFu, mac_addr, 6);
    uint32_t c32 = p24g_access_code_16to32(c16) ^ SYNCWORD_MASK;

    uint32_t h = mac_hash32(mac_addr);
    uint32_t rot = (h & 7u) + 5u;
    c32 = (c32 << rot) | (c32 >> (32 - rot));

    #if TPSLL_ACC_SWITCH_WITH_RR_EN
    /* generate different access code for different reprot rate */
    uint32_t gen_ac = generate_valid_access_code(c32) ^ ll_ctx.report_rate ^ (((uint32_t)ll_ctx.report_rate) << 8) ^ (((uint32_t)ll_ctx.report_rate) << 15);
    
    if (gen_ac == BT_INQUIRY_ACCESS_CODE || gen_ac == BLE_BROADCAST_ACCESS_CODE)
    {
        gen_ac = ~gen_ac;
    }
    *ac = gen_ac;
    #else
    *ac = generate_valid_access_code(c32);
    #endif

    if (channel) {
        *channel = map_to_safe_ble_channel(h);
    }
}

 /**
 * @brief FNV-1a 32-bit hash function
 * @param data   
 * @param len    
 * @return 16-bit hash value
 */
 _attribute_ram_code_sec_ uint32_t tpsll_fnv1a_hash(uint8_t *data, size_t len) 
{
    uint32_t hash = 0x811C9DC5;
    
    for (size_t i = 0; i < len; i++) {
        hash ^= data[i];
        hash = hash + (hash << 24) + (hash << 8) + (hash << 5);
        hash ^= (hash >> 16);
    }

    return hash;
}

/**
 * @brief FNV-1a 16-bit hash function
 * @param data   
 * @param len    
 * @return 16-bit hash value
 */
_attribute_ram_code_sec_ uint16_t tpsll_fnv1a_16(uint8_t *data, size_t len)
{
    uint16_t hash = 0x55AA;
    
    for (size_t i = 0; i < len; i++) {

        hash ^= data[i];
        hash = (hash << 5) | (hash >> 11);
        
        hash ^= (hash >> 8);
    }

    return hash;
}

static inline void tpsll_set_state(tpsll_dev_status_e state)
{
    ll_ctx.stk_state = state;

    #if TPSLL_RR_CHG_WITH_RECON_EN
    if (!ll_ctx.rr_chg_with_recon)
    #endif
    {
        g_state = state;
    }
}

_attribute_ram_code_sec_ void tpsll_enable_pairing_internal(void)
{
    ll_ctx.report_rate = APP_DEFAULT_REPORT_RATE;
    mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_CLK_REINIT_TRIGGER, &ll_ctx.report_rate);
    app_d24g_set_report_rate(APP_DEFAULT_REPORT_RATE);
    rf_set_power_level(pairing_tx_power);
    rf_access_code_comm(ll_ctx.pair_acc);
    // write_reg8(0x17000c,0x55);//related to acc
    ll_ctx.step_3t1r = STEP_3T1R_NONE;
    ll_ctx.pair_st = STATE_PAIR_INIT;
    #if TPSLL_RR_CHG_WITH_RECON_EN
    ll_ctx.rr_chg_with_recon = RR_RECON_STEP_NONE;
    #endif
    tpsll_set_state(STATE_PAIRING);
    rf_set_irq_mask(FLD_RF_IRQ_TX);
    ll_ctx.pr_cnt = 0;

    #if TPSLL_RECON_RSSI_SCAN_EN
    if (ll_ctx.recon_rssi_scanning_cnt == 0xff) {
        ll_ctx.recon_rssi_scanning_cnt = 0;
    }
    #endif
    ll_ctx.connected_active_tick = 0;
    ll_ctx.lost_cnt = 0;
    ll_ctx.d25f_timer0_anchor_us = D25F_TIMER0_ANCHOR_US_DEFAULT;
}

_attribute_ram_code_sec_ uint8_t tpsll_enable_pairing(bool enable, rf_power_level_e power_level)
{
    if(enable)
    {
        int count = sizeof(rf_power_Level_list) / sizeof(rf_power_Level_list[0]);
        int i;
        for (i = 0; i < count; i++) {
            if (rf_power_Level_list[i] == power_level) {
                break;
            }
        }
        if (i == count) {
            return TLK_ERR_INVALID_PARAM;
        }

        enable_pairing_flag = 1;
        pairing_tx_power = power_level;
    }
    else
    {
        // rf_access_code_comm(~ll_ctx.pair_acc);
        // write_reg8(0x17000c,0xAA);//related to acc
        ll_ctx.pair_st = STATE_PAIR_NONE;
        rf_clr_irq_mask(FLD_RF_IRQ_TX);
    }
    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ uint8_t tpsll_enable_reconn(bool enable)
{
    if (enable) {
        ll_ctx.report_rate = APP_DEFAULT_REPORT_RATE;
        mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_CLK_REINIT_TRIGGER, &ll_ctx.report_rate);
        app_d24g_set_report_rate(APP_DEFAULT_REPORT_RATE);

        ll_ctx.pr_cnt = 0;
#if 1
        //uint32_t acc;

        uint8_t xor_mac[MAC_ADDR_LEN] = {0};
        for (int i = 0; i < MAC_ADDR_LEN; i++)
        {
            xor_mac[i] = ll_inf.mac[i] ^ ll_inf.peer_mac[i];
        }

        #if TPSLL_NEW_RECON_EN
        p24g_gen_ac_and_channel(xor_mac, &ll_ctx.recon_acc, NULL);
        rf_access_code_comm(ll_ctx.recon_acc);
        #else
        p24g_gen_ac_and_channel(xor_mac, &acc, &ll_ctx.reconn_chn);
        rf_access_code_comm(acc);
        #endif

        #if TPSLL_RECON_RSSI_SCAN_EN && !TPSLL_NEW_RECON_EN
        if (ll_ctx.recon_rssi_scanning_cnt == 0xff) {
            ll_ctx.recon_rssi_scanning_cnt = 1;
        }
        #endif

        // #if TPSLL_RR_CHG_WITH_RECON_EN && TPSLL_RECON_RSSI_SCAN_EN
        // if (ll_ctx.rr_chg_with_recon)
        // #endif
        // {
        //     tpsll_gen_reconn_chn(ll_ctx.reconn_chn, reconn_chn_table);
        // }


#if 0
        uint8_t tmo[11] = {0};
        #if TPSLL_NEW_RECON_EN
        tmemcpy(tmo, &ll_ctx.recon_acc, 4);
        #else
        tmemcpy(tmo, &acc, 4);
        #endif
        tmemcpy(&tmo[4], ll_inf.mac, 6);
        tmo[10] = ll_ctx.reconn_chn;
        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (!ll_ctx.rr_chg_with_recon)
        {
            // tlkapi_send_string_data(APP_LOG_EN, "reconn ac chn", tmo, 11);
            // tlkapi_send_string_data(APP_LOG_EN, "reconn chn", reconn_chn_table, 11);
        }
        #endif
        // tlkapi_send_string_data(APP_LOG_EN, "reconn ac chn", tmo, 11);
#endif

#else
        rf_access_code_comm(ll_ctx.pair_acc);
        write_reg8(0x17000c, 0x55); //related to acc
        rf_set_ble_chn(PAIR_CHN);
#endif


        tpsll_set_state(STATE_RECONNECT);
        rf_set_irq_mask(FLD_RF_IRQ_TX);
        // app_p24g_reset_report_rate();
    } else {
        // rf_access_code_comm(~ll_ctx.pair_acc);
        // write_reg8(0x17000c,0xAA);//related to acc
    }
    return TLK_SUCCESS;
}

static inline void app_d24g_keep_only_last_key_in_fifo(void)
{
    if (pp_fifo_get_num(km_fifo_ptr) > 1) {
        km_fifo_ptr->rptr = km_fifo_ptr->wptr - 1;
    }
}

_attribute_ram_code_sec_ static void app_d24g_enter_conn(void)
{
    calc_hop_channels(ll_ctx.fno_last + 1, 0);

    tpsll_set_state(STATE_CONNECTED);
    tpsll_enable_pairing(false, pairing_tx_power);
    rf_set_power_level(RF_POWER);
    
    uint32_t acc;
    p24g_gen_ac_and_channel(ll_inf.peer_mac, &acc, 0);
    rf_access_code_comm(acc);

    #if TPSLL_RR_CHG_WITH_RECON_EN
    if (ll_ctx.rr_chg_with_recon) {
        unsigned char *p = pp_fifo_get_ptr(&ll_fifo);

        if (p)
        {
            if(p[2] == TPSLL_LL_CMD_RR_CHG_RCNN_REQ)
            {
                pp_fifo_pop(&ll_fifo);
            }
        }
    } else
    #endif
    {
        if (!ll_ctx.reconn_from_request_sleep) {
            pp_fifo_reset(km_fifo_ptr);    //flush key before connected
        } else {
            app_d24g_keep_only_last_key_in_fifo();    // keep only the latest key for wakeup PC
            ll_ctx.reconn_from_request_sleep = false;
        }
        pp_fifo_reset(&ll_fifo);
        pp_fifo_reset(spp_fifo_ptr);

        tlkapi_send_string_data(APP_LOG_EN, "conn ac", &acc, 4);
        tlkapi_send_string_data(APP_LOG_EN, "dongle ac", ll_inf.peer_mac, 6);
    }

    #if TPSLL_FHSS_EN
    g_fhss_idx_buf_data_cur = 0;
    g_fhss_idx_buf_data_next = 0;
    #endif

    ll_ctx.fno_conn = ll_ctx.fno_last | 1;

    ll_ctx.spp_flag = false;
    ll_ctx.ll_flag = false;
    ll_ctx.chg_dly_cnt = 0;

    if(ll_ctx.report_rate & REPORT_RATE_8K)
    {
        ll_ctx.tx_sn = 0;
        ll_ctx.rx_nesn = 1;
        ll_ctx.sn_8k = 1;
        ll_ctx.nesn_8k = 0;
        ll_ctx.s_8k = 0;
        #if TPSLL_SPP_NOT_SWITCH_EN
        SET_SN(rf_tx_packet.snnesn, ll_ctx.tx_sn);
        SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);
        ll_ctx.spp_quota_active = 0;
        ll_ctx.ms_send_cnt = 0;
        #endif
    }
    else
    {
        ll_ctx.tx_sn          = 0;
        ll_ctx.rx_nesn        = 1;
        SET_SN(rf_tx_packet.snnesn, ll_ctx.tx_sn);
        SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);
    }

    ll_ctx.suspend_flag = false;
    ll_ctx.used_sm = SLOT_MODE_KM;
    ll_inf.ms_idle_cnt = 20;
    ll_ctx.peer_spp_flag = P24G_PKT_TYPE_SYNC;
    ll_ctx.is_spp_pre_acked = true;
    ll_ctx.rx_over = true;
    ll_ctx.allow_spp = false;

    #if KB_SPP_FRAG_EN
    tpsll_spp_rx_assemble_reset();
    #endif

    tmemset(rf_tx_packet.buf, 0, STACK_EVT_QUEUE_SIZE);
    #if TPSLL_KEEP_ALIVE_NOTIFY_EN
    rf_tx_packet.n_tab = 0;
    #endif
    // p24g_evt_cb(TPSLL_EVT_DEV_CONNECTED, (void *)ll_inf.peer_mac, MAC_ADDR_LEN);

    ll_ctx.connected_active_tick = stimer_get_tick() | 1;

    #if TPSLL_PM_OPTM_EN
    ll_ctx.pm_optm_level = 0;
    ll_ctx.ks_need_disable = false;
    #endif

    #if TPSLL_KEEP_ALIVE_EN
    tpsll_event_active_time_fresh_value(ll_ctx.connected_active_tick);
    ll_ctx.alive_interval_max = KEEP_ALIVE_INTERVAL_8K_MAX_CNT;
    ll_ctx.alive_interval_cnt = ll_ctx.alive_interval_max;
    #endif

    #if TPSLL_KB_ACTIVE_PM_OPTM_EN
    ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
    ll_ctx.kb_alive_intervals_cnt = 0;
    #endif

    #if TPSLL_CONN_QUICK_LOST_TIME_US
    ll_ctx.quick_discon_active_tick = ll_ctx.connected_active_tick;
    #endif

    #if TPSLL_CONNECTING_CHECK_EN
    ll_ctx.connecting_check = 1;
    #endif

    #if TPSLL_RR_CHG_WITH_RECON_EN
    if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP5) {
        ll_ctx.rr_chg_with_recon = RR_RECON_STEP6;
    } else if (!ll_ctx.rr_chg_with_recon)
    #endif
    // stack_evt_post_from_isr(TPSLL_EVT_DEV_CONNECTED, (void *)ll_inf.peer_mac, MAC_ADDR_LEN);
    s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_DEV_CONNECTED, ll_inf.peer_mac, MAC_ADDR_LEN);
}

_attribute_ram_code_sec_ static void tpsll_connect_lost(void)
{
    ll_ctx.connected_active_tick = 0;
    ll_ctx.reconnected_cnt = 0;

    ll_ctx.lost_cnt = 0;
    ll_ctx.step_3t1r = STEP_3T1R_NONE;
    #if TPSLL_FHSS_EN
    ll_ctx.pending_rr = 0;
    ll_ctx.pending_rr_spp = 0;
    #endif

    #if TPSLL_RECON_RSSI_SCAN_EN && !TPSLL_NEW_RECON_EN
    ll_ctx.recon_rssi_scanning_cnt = 1;
    #endif

    tpsll_set_state(STATE_IDLE);
    pp_timer1_stop();
    timer_clr_irq_status(FLD_TMR1_MODE_IRQ);
}

uint8_t tpsll_enter_idle(tpsll_pm_optm_level_e level)
{
    if (level < TPSLL_PM_OPTM_LEVEL3) {
        return TLK_ERR_INVALID_PARAM;
    }

    tpsll_connect_lost();
    ll_ctx.report_rate = APP_DEFAULT_REPORT_RATE;
    mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_CLK_REINIT_TRIGGER, &ll_ctx.report_rate);
    if (ll_ctx.pm_optm_ref_fno_cnt <= ll_ctx.pm_optm_3rd_ref_fno_cnt_th) {
        ll_ctx.pm_optm_ref_fno_cnt = ll_ctx.pm_optm_3rd_ref_fno_cnt_th;
    }

    #if TPSLL_PM_OPTM_EN
    ll_ctx.pm_optm_level = level;

    if (level & TPSLL_PM_OPTM_LEVEL3) {
        ll_ctx.pm_optm_ref_fno_cnt = TPSLL_PM_OPTM_LEVEL3_REF_FNO_CNT_START;
        // stack_evt_post_from_isr(TPSLL_EVT_PM_OPTM_ENTER_LEVEL3, 0, 0);
        s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_PM_OPTM_ENTER_LEVEL3, 0, 0);
    } else {
        // stack_evt_post_from_isr(TPSLL_EVT_PM_OPTM_ENTER_LEVEL4, 0, 0);
        s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_PM_OPTM_ENTER_LEVEL4, 0, 0);
    }
    #endif
    ll_ctx.suspend_wakeup_early_time_us = APP_WAKEUP_EARLY_TIME_US;
    timer_start(TIMER1);
    pp_fifo_reset(km_fifo_ptr);

    return TLK_SUCCESS;
}

#if TPSLL_N8K_CHN_EXPAND_EN
_attribute_ram_code_sec_ static void tpsll_n8k_chn_expand(void)
{
    chn_table[3] = chn_table[1];

    tpsll_generate_deterministic_ble_channels(chn_table, chn_table + 4);

    chn_table[7] = chn_table[4];
}
#endif

/**
 *  @brief    Notify the underlying device of the protocol to prepare for entering sleep mode, and rf link must idle 
 *  @param[in]  en  enbale  
 *  @return  0: rf is idle status,  other: is not idle
 */
_attribute_ram_code_sec_ unsigned int tpsll_get_rf_link_status(void)
{
    return rf_status;
}

uint8_t tpsll_stimer_init(void)
{
    DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
    if (pm_is_MCU_deepRetentionWakeup()) {
        if (ll_ctx.stk_state == STATE_IDLE)
        {
            ll_ctx.stick = stimer_get_tick() +  TPSLL_PM_OPTM_DEEP_RET_STIMER_START_DELAY_US * SYSTEM_TIMER_TICK_1US;
            if (ll_ctx.is_alg_kb && (ll_ctx.pm_optm_level & TPSLL_PM_OPTM_LEVEL4)) {
                ll_ctx.key_scan_disable = false;
            }
        } else {
            ll_ctx.stick = stimer_get_tick() + 10 * SYSTEM_TIMER_TICK_1MS;
            // tlkapi_printf(APP_LOG_EN, "<%d-%x,%d-%d>\n", ll_ctx.stk_state, ll_ctx.pm_optm_level, ll_ctx.pm_optm_3rd_interval_ms, ll_ctx.pm_optm_4th_interval_ms);
        }
    } else {
        ll_ctx.stick = stimer_get_tick() + 10 * SYSTEM_TIMER_TICK_1MS;
    }

    systimer_set_irq_capture(ll_ctx.stick); //set capture tick
    systimer_set_irq_mask();
    #if defined(MCU_CORE_TL322X_N22)
    clic_set_priority(IRQ_SYSTIMER, IRQ_PRI_LEV3);
    clic_interrupt_vector_en(IRQ_SYSTIMER);
    clic_interrupt_enable(IRQ_SYSTIMER);
    #else
    plic_interrupt_enable(IRQ_SYSTIMER);
    plic_set_priority(IRQ_SYSTIMER, IRQ_PRI_LEV3);
    #endif
    ll_ctx.stimer_running = true;
    return TLK_SUCCESS;
}

_attribute_ram_code_sec_  short data_test(void)
{
    static short cnt = 0;

    cnt++;

    if (cnt > 4)
        cnt = -4;

    if (cnt == 0)
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, GPIO_PH4);

    return cnt;
}

_attribute_ram_code_sec_ uint8_t app_fill_ll_data(void)
{
    uint8_t len = 0;
    unsigned char *p = pp_fifo_get_ptr(&ll_fifo);
    if (p != 0)
    {
        uint8_t fifo_data_len = p[0];
        uint8_t cmd = p[1];
        if (cmd == TPSLL_LL_CMD)
        {
            len = fifo_data_len;
            tmemcpy((void *)&rf_tx_packet.buf[0], (void *)&p[2], fifo_data_len);
            rf_tx_packet.pkt_type = P24G_PKT_TYPE_LL;
            ll_ctx.ll_flag = true;
        }
    }
    return len;
}

_attribute_ram_code_sec_ uint8_t tpsll_fill_spp_data(void)
{
    uint8_t len = 0;
    unsigned char *p = pp_fifo_get_ptr(spp_fifo_ptr);
    if (p != 0)
    {
        // uint8_t fifo_data_len = p[0];
        uint8_t cmd = p[1];
        if (cmd == TPSLL_SPP_DATA)
        {
            len = p[0];
            pp_rf_set_tx_len(len);
            tmemcpy((void *)&rf_tx_packet.buf[0], (void *)&p[2], len);
            rf_tx_packet.pkt_type = P24G_PKT_TYPE_SPP;
            ll_ctx.spp_flag = true;
            #if TPSLL_SPP_NOT_SWITCH_EN
            // ll_ctx.spp_quota_active = 1;
            ll_ctx.ms_send_cnt = 0;
            #endif

        }
    }
    #if TPSLL_NO_EVENT_PWR_DOWN_EN
    if (len)
    {
        ll_ctx.no_event_cnt = 0;
    }
    #endif
    return len;
}

_attribute_ram_code_sec_ void app_fill_kb_data(bool allow_ll_data)
{
    if(!ll_ctx.is_spp_pre_acked)
    {
        return;
    }

    unsigned char  *p= pp_fifo_get_ptr(km_fifo_ptr);
    if(p!=0)
    {
        unsigned char cmd = p[1];
        unsigned char kb_len = p[0];

        if(cmd == NORMAL_KB_DATA_CMD || cmd == CONSUME_KB_DATA_CMD || cmd == SYSTEM_KB_DATA_CMD)
        {
            //old data and ack
            if((!tmemcmp(last_valid_key_buf, p, kb_len + 2)) && ll_ctx.acked)
            {
                if (allow_ll_data && !pp_fifo_is_empty(&ll_fifo))
                {
                    uint8_t data_length = app_fill_ll_data();
                    pp_rf_set_tx_len(data_length);
                }
                else
                {
                    pp_rf_set_tx_len(0);
                    rf_tx_packet.pkt_type = P24G_PKT_TYPE_KB;
                    ll_ctx.key_length = 0;
                }
                pp_fifo_pop(km_fifo_ptr);
                ll_ctx.kb_flag = false;
            }
            else
            {
                rf_tx_packet.pkt_type = P24G_PKT_TYPE_KB;
                rf_tx_packet.buf[0] = cmd;

                if(cmd == NORMAL_KB_DATA_CMD)
                {
                    rf_tx_packet.buf[1] = p[2];
                    tmemcpy(&rf_tx_packet.buf[2], &p[4], kb_len - 2);
                    pp_rf_set_tx_len(kb_len);
                    ll_ctx.key_length = kb_len;
                }
                else
                {
                    tmemcpy(&rf_tx_packet.buf[1], &p[2], kb_len);
                    pp_rf_set_tx_len(kb_len + 1);
                    ll_ctx.key_length = kb_len + 1;
                }

                ll_ctx.last_valid_slot_id = ll_ctx.step_3t1r;
                tmemcpy(last_valid_key_buf, p, kb_len + 2);
                ll_ctx.kb_flag = true;
            }
        }
        else if(cmd == ALL_KB_DATA_CMD)
        {
            if (ll_ctx.report_rate & REPORT_RATE_8K) 
            {
                if (ll_ctx.step_3t1r == STEP_3T1R_1T || ll_ctx.step_3t1r == STEP_3T1R_3T)
                {
                    pp_rf_set_tx_len(0);
                    ll_ctx.key_length = 0;
                    rf_tx_packet.pkt_type = P24G_PKT_TYPE_KB;
                    return;
                }
                else
                {
                    rf_tx_packet.pkt_type = P24G_PKT_TYPE_KB;
                    rf_tx_packet.buf[0] = cmd;

                    tmemcpy(&rf_tx_packet.buf[1], &p[2], kb_len);
                    pp_rf_set_tx_len(kb_len + 1);
                    ll_ctx.key_length = kb_len + 1;

                    ll_ctx.last_valid_slot_id = ll_ctx.step_3t1r; // update last valid slot id
                    tmemcpy(last_all_key_buf, &p[2], kb_len);
                    ll_ctx.ak_flag = true;
                    // tlkapi_send_string_data(APP_LOG_EN, "all", rf_tx_packet.buf, kb_len + 1);
                }
            }
            else
            {
                //old data and ack
                if ((!tmemcmp(last_all_key_buf, &p[2], kb_len)) && ll_ctx.acked)
                {
                    if (allow_ll_data && !pp_fifo_is_empty(&ll_fifo))
                    {
                        uint8_t data_length = app_fill_ll_data();
                        pp_rf_set_tx_len(data_length);
                    }
                    else
                    {
                        pp_rf_set_tx_len(0);
                        ll_ctx.key_length = 0;
                        rf_tx_packet.pkt_type = P24G_PKT_TYPE_KB;
                    }
                    pp_fifo_pop(km_fifo_ptr);
                    ll_ctx.kb_flag = false;
                }
                else
                {
                    rf_tx_packet.pkt_type = P24G_PKT_TYPE_KB;
                    rf_tx_packet.buf[0] = cmd;

                    tmemcpy(&rf_tx_packet.buf[1], &p[2], kb_len);
                    pp_rf_set_tx_len(kb_len + 1);
                    ll_ctx.key_length = kb_len + 1;

                    ll_ctx.last_valid_slot_id = ll_ctx.step_3t1r; // update last valid slot id
                    tmemcpy(last_all_key_buf, &p[2], kb_len);
                    ll_ctx.kb_flag = true;

                    // tlkapi_send_string_data(APP_LOG_EN, "all", rf_tx_packet.buf, kb_len + 1);
                }
            }
        }

    }else{
        if (allow_ll_data && !pp_fifo_is_empty(&ll_fifo))
        {
            uint8_t data_length = app_fill_ll_data();

            if (rf_tx_packet.buf[0] == TPSLL_LL_CMD_RR_CHG_REQ)
            {
                ll_ctx.pending_rr_fno += g_change_rr_delay;
                rf_tx_packet.buf[2] = ll_ctx.pending_rr_fno & 0xff;
                rf_tx_packet.buf[3] = (ll_ctx.pending_rr_fno >> 8) & 0xff;
                g_change_rr_delay = 0;
            }

            pp_rf_set_tx_len(data_length);
        }
        #if TPSLL_NON_8K_SPP_NOT_SW_EN
        else if (!pp_fifo_is_empty(spp_fifo_ptr) && (ll_ctx.task_slot == TASK_SLOT0) && (ll_ctx.report_rate != REPORT_RATE_8K))
        {
            uint8_t data_length = tpsll_fill_spp_data();
            pp_rf_set_tx_len(data_length);
            // tlkapi_printf(APP_LOG_EN, "data_len:%d \n", data_length);
        }
        #endif
        else
        {
            pp_rf_set_tx_len(0);
            ll_ctx.key_length = 0;
            rf_tx_packet.pkt_type = P24G_PKT_TYPE_KB;
        }
    }
}


_attribute_ram_code_sec_ void tpsll_fill_null_pkt(void)
{
    pp_rf_set_tx_len(0);
    rf_tx_packet.pkt_type = P24G_PKT_TYPE_MS;
}

// static void tpsll_set_96Mcclk_rx_wait(uint8_t len)
// {
//     /*
//    17 bytes payload ---> rx_wait = 8 us
//    Calculate using integer arithmetic to avoid floating-point operations.
//    To avoid floating-point operations, integer arithmetic is used. The formula
//    idle_time = 7 + (6 - len) * 1.333...` is scaled by 3.
//    Original formula: (8 * 3) + (17 - len) * 4 = 24 + (17 - len) * 4
//    Adding 1 before division by 3 to implement rounding to the nearest integer
// */
//     if (len > 17)
//     {
//         len = 17;
//     }
//     uint8_t rx_wait = (24 + (17 - len) * 4 + 1) / 3;

//     app_rf_set_rx_wait(rx_wait);
// }

_attribute_ram_code_sec_ void app_non_8k_rf_pkt_send(void)
{
    bool fifo_empty_flag = pp_fifo_is_empty(km_fifo_ptr) && pp_fifo_is_empty(spp_fifo_ptr) && pp_fifo_is_empty(&ll_fifo);

        #if TPSLL_KEEP_ALIVE_EN
        #if TPSLL_NEW_KEEP_ALIVE_EN
        if (tpsll_is_keep_alive_active() && fifo_empty_flag)
        {
            if (!ll_ctx.is_keep_alive)
            {
                ll_ctx.is_keep_alive = true;
                ll_ctx.synced = 0;
                #if TPSLL_KEEP_ALIVE_NOTIFY_EN
                rf_tx_packet.n_tab = 1;
                #endif
                ll_ctx.keep_alive_cnt = 0;
            }

            #if !TPSLL_PM_OPTM_N8K_SUSPEND_EN
            ll_ctx.keep_alive_cnt++;

            if (((ll_ctx.keep_alive_cnt & ll_ctx.ka_mask) != 0) && (ll_ctx.synced))
            {
                ll_ctx.is_keep_alive_sync = false;
                tpsll_set_next_keep_alive_stimer_anchor(ll_ctx.keep_alive_sync_us);
                #if !TPSLL_PM_OPTM_EN
                if(ll_ctx.alive_secondary_ref_cnt)
                {
                    if(ll_ctx.alive_secondary_ref_cnt >= ((uint16_t)(ll_ctx.fno - ll_ctx.fno_last + 1)))
                    {
                        ll_ctx.alive_secondary_ref_cnt -= (uint16_t)(ll_ctx.fno - ll_ctx.fno_last + 1);
                        // gpio_toggle(dbg_io[DBG_IO_IDX2]);
                    }
                    else
                    {
                        ll_ctx.alive_secondary_ref_cnt = 0;
                    }
                }
                #endif /* end of !TPSLL_PM_OPTM_EN */
                return;
            }
            else
            {
                ll_ctx.is_keep_alive_sync = true;
            }

            #endif /* end of !TPSLL_PM_OPTM_N8K_SUSPEND_EN */

            if (ll_ctx.task_slot == TASK_SLOT0) {
                ll_ctx.synced = 0;
            }
        }
        else
        {
            if (ll_ctx.is_keep_alive)
            {
                ll_ctx.is_keep_alive = false;
                #if TPSLL_KEEP_ALIVE_NOTIFY_EN
                rf_tx_packet.n_tab = 0;
                #endif
                #if TPSLL_KB_ACTIVE_PM_OPTM_EN
                ll_ctx.synced = 0;
                ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
                #endif
                ll_ctx.is_keep_alive_sync = true;
            }
            #if TPSLL_KB_ACTIVE_PM_OPTM_EN
            if (!(ll_ctx.fno_last & ll_ctx.kb_alive_fno_msk)) {
                ll_ctx.synced = 0;
            }
            #endif
        }

        if(ll_ctx.report_rate & (REPORT_RATE_4K | REPORT_RATE_2K))
        {
            #if TPSLL_KB_ACTIVE_PM_OPTM_EN
            if ((!tpsll_is_keep_alive_active() && fifo_empty_flag && (ll_ctx.fno_last & ll_ctx.kb_alive_fno_msk) && ll_ctx.synced)
                #if TPSLL_PM_OPTM_N8K_SUSPEND_EN
                || (tpsll_is_keep_alive_active() && (ll_ctx.task_slot > TASK_SLOT0))
                #endif
            ) {
                return;
            }
            #endif
        }
        else
        #endif
        {
            if (((tpsll_is_keep_alive_active()
                #if TPSLL_PM_OPTM_N8K_SUSPEND_EN
                //  && (ll_ctx.task_slot > TASK_SLOT0)
                && ll_ctx.synced
                #endif
            ) || (!tpsll_is_keep_alive_active() && (ll_ctx.fno_last & ll_ctx.kb_alive_fno_msk))) && ll_ctx.synced && fifo_empty_flag)
            {
                return;
            }
        }
        #endif

        // if (ll_ctx.step_3t1r == STEP_3T1R_1T || ll_ctx.step_2t2r == STEP_2T2R_1T) { //SLOT0 1T
        // if(1)// (ll_ctx.task_slot == TASK_SLOT0)
        { // SLOT0 1T
            // rf_set_ble_chn(chn_list[STEP_2T2R_1T]);
            // rf_set_ble_chn(CONN_CHN_NO);
            if((ll_ctx.report_rate & REPORT_RATE_125) || (ll_ctx.report_rate & REPORT_RATE_250))
            {
                if(!(ll_ctx.task_slot & 1)) {
                    #if TPSLL_N8K_FHSS_EN
                    #if TPSLL_N8K_CHN_EXPAND_EN
                    uint8_t ch;
                    if(ll_ctx.task_slot) {
                        ch = chn_table[(ll_ctx.fno_last + (ll_ctx.task_slot >> 1)) & 7];
                    } else {
                        ch = chn_table[(ll_ctx.fno_last + (ll_ctx.task_slot >> 1)) & 3];
                    }
                    #else
                    uint8_t ch = chn_table[(ll_ctx.fno_last + (ll_ctx.task_slot >> 1)) & 3];
                    #endif
                    #else
                    uint8_t ch = g_n8k_chn_tab[(ll_ctx.fno_last + (ll_ctx.task_slot >> 1)) & (NON_8K_CHN_NUM - 1)];
                    #endif
                    rf_set_ble_chn(ch);
                    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
                }
            }else{
                #if TPSLL_N8K_FHSS_EN
                #if TPSLL_N8K_CHN_EXPAND_EN
                uint8_t ch;
                if(ll_ctx.task_slot) {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 7];
                } else {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                }
                #else
                uint8_t ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                #endif
                #else
                uint8_t ch = g_n8k_chn_tab[(ll_ctx.fno_last + ll_ctx.task_slot) & (NON_8K_CHN_NUM - 1)];
                #endif
                rf_set_ble_chn(ch);
            }

            // rf_set_ble_chn(chn_table[ll_ctx.ch_idx + ((ll_ctx.task_slot + ll_ctx.fno_last) & 3)]);

            // rf_set_ble_chn(chn_table[(ll_ctx.task_slot + ll_ctx.fno_last) & 3]);
            
            // rf_set_ble_chn(chn_table[(ll_ctx.task_slot / 2) + ll_ctx.ch_idx]);

            uint32_t tx_tick = stimer_get_tick();
            if (ll_ctx.precise_tx)
            {
                if (ll_ctx.task_slot == TASK_SLOT0) {
                    tx_tick = ll_ctx.last_stick + (SLOT0_TX_TIME_US) * RF_SYSTEM_TIMER_TICK_1US;
                } else {
                    tx_tick = ll_ctx.last_stick + (SLOT1_OFFSET_TIME_US_125 + (ll_ctx.task_slot - 1) * SLOT_TIME_US_125 + TX_MARGIN_US_125) * RF_SYSTEM_TIMER_TICK_1US;
                }
            }else{
                if(ll_ctx.report_rate != REPORT_RATE_4K)
                {
                    tx_tick = stimer_get_tick() + 7 * RF_SYSTEM_TIMER_TICK_1US /* make sure dma_len ready before rf send */;
                }
            }
            if (ll_ctx.task_slot == TASK_SLOT0 && ll_ctx.report_rate != REPORT_RATE_4K)
            {
            #if TPSLL_NON_8K_SPP_NOT_SW_EN
                #if TPSLL_NEW_SYNC_EN
                if (!pp_fifo_is_empty(spp_fifo_ptr) && ((ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500)) || (ll_ctx.time_synced_cnt >= 2)))
                #else
                // if (!pp_fifo_is_empty(spp_fifo_ptr))
                if (!pp_fifo_is_empty(spp_fifo_ptr) && (ll_ctx.time_synced_cnt >= 2))
                #endif
                {
                    rf_set_tx_settle_time(18 + 14);
                }else{
  
                    rf_set_tx_settle_time(18);
                }
            #else
                rf_set_tx_settle_time(18);
            #endif
                
            }else{
                rf_set_tx_settle_time(12);
            }
            if (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250)) {
                app_rf_start_stx2rx(tx_tick, KB_CONNECTED_KB_RX_TIMEOUT);
            } else {
                #if TPSLL_NON_8K_SPP_NOT_SW_EN
                if (ll_ctx.report_rate & (REPORT_RATE_4K | REPORT_RATE_2K)) {
                    #if TPSLL_PM_OPTM_N8K_SUSPEND_EN
                    if (tpsll_pm_optm_is_long_suspend_support_level_active()) {
                        app_rf_start_stx2rx(tx_tick, KB_CONNECTED_KB_RX_TIMEOUT + 12 + TPSLL_PM_OPTM_N8K_RX_WIN_EXPAND_US * 2);
                    } else
                    #endif
                    app_rf_start_stx2rx(tx_tick, KB_CONNECTED_KB_RX_TIMEOUT + 12);
                } else {
                    app_rf_start_stx2rx(tx_tick, KB_CONNECTED_KB_RX_TIMEOUT + 22);
                }
                #else
                app_rf_start_stx2rx(tx_tick, KB_CONNECTED_KB_RX_TIMEOUT + 22);
                #endif
            }
            ll_ctx.rx_over = false;
            ll_ctx.srx_task_slot = ll_ctx.task_slot;
        }
        //  p24g_set_snnesn();

            {
                app_fill_kb_data(TRUE);

                if(ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500))
                {
                    if (ll_ctx.task_slot == TASK_SLOT0) {
                        if (rf_tx_packet.pkt_type >= P24G_PKT_TYPE_SPP) {
                            app_rf_set_rx_wait(12);
                        } else {
                            app_rf_set_rx_wait(10);
                        }
                    }
                    else{
                        app_rf_set_rx_wait(4);
                    }
                }
                else if(ll_ctx.report_rate & (REPORT_RATE_2K))
                {
                    if (ll_ctx.task_slot == TASK_SLOT0) {
                        if ((rf_tx_packet.pkt_type >= P24G_PKT_TYPE_SPP)
                            #if TPSLL_PM_OPTM_N8K_SUSPEND_EN
                            || tpsll_pm_optm_is_long_suspend_support_level_active()
                            #endif
                        ) {
                            app_rf_set_rx_wait(12);
                        } else {
                            app_rf_set_rx_wait(32);
                        }
                    }
                    else{
                        app_rf_set_rx_wait(4);
                    }
                }
                else if (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250))
                {
                    if (ll_ctx.task_slot == TASK_SLOT0) {
                        if (rf_tx_packet.pkt_type >= P24G_PKT_TYPE_SPP) {
                            app_rf_set_rx_wait(6);
                        } else {
                            app_rf_set_rx_wait(6);
                        }
                    }
                    else{
                        app_rf_set_rx_wait(5);
                    }
                }
                else if(ll_ctx.report_rate & REPORT_RATE_4K)
                {
                    if (ll_ctx.task_slot == TASK_SLOT0) {
                        if ((rf_tx_packet.pkt_type >= P24G_PKT_TYPE_SPP)
                            #if TPSLL_PM_OPTM_N8K_SUSPEND_EN
                            || tpsll_pm_optm_is_long_suspend_support_level_active()
                            #endif
                        ) {
                            app_rf_set_rx_wait(12);
                        } else {
                            p24g_set_rx_wait(ll_ctx.key_length);
                        }
                    }
                    else{
                        app_rf_set_rx_wait(6);
                    }
                }
            }

            #if TPSLL_NON_8K_SPP_NOT_SW_EN
            rf_set_rx_maxlen(68);
            #else

            rf_set_rx_maxlen(RF_PKT_ACK_LEN + 3);
            #endif

            ll_ctx.is_spp_pre_acked = false;

            // if(ll_ctx.task_slot < 2)
            // {
            //     tlkapi_printf(APP_LOG_EN, "fno:%x slot:%x ch:%x\n", ll_ctx.fno_last, ll_ctx.task_slot, ch);
            // }

    if (!fifo_empty_flag) {
        tpsll_pm_optm_time_refresh();
    }
}

_attribute_ram_code_sec_ void tpsll_8k_km_send(void)
{
    unsigned char data_len = 0;

    #if TPSLL_KEEP_ALIVE_EN
    bool fifo_empty_flag = FALSE; 

    #if TPSLL_SPP_NOT_SWITCH_EN
    if (pp_fifo_is_empty(km_fifo_ptr) && pp_fifo_is_empty(spp_fifo_ptr) && pp_fifo_is_empty(&ll_fifo))
    #else
    if (pp_fifo_is_empty(km_fifo_ptr) && pp_fifo_is_empty(&ll_fifo)) 
    #endif
    {
        fifo_empty_flag = TRUE;
    }

    #if TPSLL_NEW_KEEP_ALIVE_EN

    if (fifo_empty_flag && tpsll_is_keep_alive_active())
    {
        if (!ll_ctx.is_keep_alive)
        {
            ll_ctx.is_keep_alive = true;
            ll_ctx.synced = 0;
            #if TPSLL_KEEP_ALIVE_NOTIFY_EN
            rf_tx_packet.n_tab = 1;
            #endif
            #if TPSLL_KEEP_ALIVE2_NOTIFY_EN
            rf_tx_packet.buf[0] = 0;
            #endif
        }
    }
    else
    {
        if (ll_ctx.is_keep_alive)
        {
            ll_ctx.is_keep_alive = false;
            #if TPSLL_KEEP_ALIVE_NOTIFY_EN
            rf_tx_packet.n_tab = 0;
            #endif
            #if TPSLL_KB_ACTIVE_PM_OPTM_EN
            ll_ctx.synced = 0;
            ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
            #endif
        }

        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        if (!(ll_ctx.fno_last & ll_ctx.kb_alive_fno_msk)) {
            ll_ctx.synced = 0;
        }
        #endif
    }

    #if TPSLL_KB_ACTIVE_PM_OPTM_EN
    if (fifo_empty_flag && (ll_ctx.fno_last & ll_ctx.kb_alive_fno_msk) && ll_ctx.synced) {
        return;
    }
    #endif
    #else
    if (ll_ctx.synced && fifo_empty_flag && tpsll_is_keep_alive_active()) {
        return;
    }
    #endif
    #endif
    #if !TPSLL_SPP_NOT_SWITCH_EN
    SET_NESN(rf_tx_packet.snnesn, ll_ctx.sn_8k);
    SET_SN(rf_tx_packet.snnesn, ll_ctx.s_8k);
    #endif

    if (ll_ctx.step_3t1r == STEP_3T1R_1T)
    { // SLOT0 1T

        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        if(tpsll_is_active_state() && !ll_ctx.kb_alive_fno_msk)
        #else
        if(!ll_ctx.is_keep_alive)
        #endif
        {
            rf_set_ble_chn(chn_list[ll_ctx.step_3t1r]);
            //            rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, ll_ctx.last_stick + (APP_SLOT0_1T_START_TIME_US)*RF_SYSTEM_TIMER_TICK_1US);
            #if TPSLL_NEW_8K_RF_TIMING_EN
            rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, ll_ctx.last_stick + 72 - 1 * RF_SYSTEM_TIMER_TICK_1US); // for saving time
            #else
            rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, ll_ctx.last_stick + 72); // for saving time
            #endif

            if (!pp_fifo_is_empty(spp_fifo_ptr) && !ll_ctx.spp_quota_active && !ll_ctx.ak_flag)
            {
                unsigned char *p = pp_fifo_get_ptr(spp_fifo_ptr);

                if (p && (p[0] > KM_2T_SPP_MAX_LEN)) {
                    data_len = tpsll_fill_spp_data();
                } else {
                    rf_tx_packet.pkt_type = P24G_PKT_TYPE_SYNC;
                }

                pp_rf_set_tx_len(data_len);
            } else
            // #if TPSLL_SPP_NOT_SWITCH_EN
            // if (pp_fifo_is_empty(spp_fifo_ptr) || ll_ctx.spp_quota_active)
            // // if (pp_fifo_is_empty(spp_fifo_ptr) )
            // #endif
            {
                app_fill_kb_data(FALSE);
                #if TPSLL_SPP_NOT_SWITCH_EN
                ll_ctx.ms_send_cnt++;
                if (ll_ctx.ms_send_cnt >= TPSLL_SPP_KM_RATIO)
                {
                    ll_ctx.spp_quota_active = 0;
                    ll_ctx.ms_send_cnt = 0;
                }
                #endif
                //  if(rf_tx_packet.buf[6])
                //  {
                //      l_press = 1;
                //  }
                //  rf_tx_packet.buf[0] = MOUSE_DATA;

                //                if (6 == data_len)
                //                    data_len = 12;
                //                pp_rf_set_tx_len(data_len);
            }
            // #if TPSLL_SPP_NOT_SWITCH_EN
            // else
            // {
            //     unsigned char *p = pp_fifo_get_ptr(spp_fifo_ptr);

            //     if (p && (p[0] > KM_2T_SPP_MAX_LEN)) {
            //         data_len = tpsll_fill_spp_data();
            //     }

            //     pp_rf_set_tx_len(data_len);
            //     #if 0
            //     if (ll_ctx.tx_sn)
            //     {
            //         ZW_TG(GPIO_PG6);
            //     }
            //     else
            //     {
            //         ZW_TG(GPIO_PG6);
            //         ZW_TG(GPIO_PG6);
            //     }

            //     if (ll_ctx.rx_nesn)
            //     {
            //         ZW_TG(GPIO_PH0);
            //     }
            //     else
            //     {
            //         ZW_TG(GPIO_PH0);
            //         ZW_TG(GPIO_PH0);
            //     }
            //     #endif

            // }
            // #endif
        }
        // if(l_press && dcnt < 20)
        // {
        //     tlkapi_send_string_data(APP_LOG_EN, "tx", rf_tx_packet.buf, 12);
        // }
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_2T)
    { // SLOT1 2T
        // rf_set_ble_chn(chn_list[ll_ctx.step_3t1r]);
        // if (ll_ctx.report_rate & REPORT_RATE_8K)
        #if TPSLL_SPP_NOT_SWITCH_EN
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        if((rf_tx_packet.pkt_type != P24G_PKT_TYPE_SPP) &&
        #if !TPSLL_PM_OPTM_8K_ONLY_USE_2T_EN
            tpsll_is_active_state() &&
        #endif
            !ll_ctx.kb_alive_fno_msk)
        #else
        if((rf_tx_packet.pkt_type != P24G_PKT_TYPE_SPP))
        #endif
        #endif
        {
            rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, ll_ctx.last_stick + APP_SLOT1_2T_START_TIME_US * RF_SYSTEM_TIMER_TICK_1US); // set 2T tick time

            {
                #if TPSLL_KEEP_ALIVE_EN
                if (tpsll_all_fifo_is_empty()) { //sync with empty packet
                    rf_tx_packet.pkt_type = P24G_PKT_TYPE_SYNC;
                    #if TPSLL_KEEP_ALIVE2_NOTIFY_EN
                    if (tpsll_pm_optm_is_level2_active()) {
                        rf_tx_packet.buf[0] = 0xA5;
                        pp_rf_set_tx_len(1);
                    } else
                    #endif
                    {
                        pp_rf_set_tx_len(0);
                    }
                    return;
                }
                #endif

                #if TPSLL_SPP_NOT_SWITCH_EN
                if (!pp_fifo_is_empty(spp_fifo_ptr) && !ll_ctx.spp_quota_active && !ll_ctx.ak_flag)
                {
                    unsigned char *p = pp_fifo_get_ptr(spp_fifo_ptr);

                    if (p && (p[0] <= KM_2T_SPP_MAX_LEN)) {

                        data_len = tpsll_fill_spp_data();
                    }

                    pp_rf_set_tx_len(data_len);
                }
                else
                #endif
                app_fill_kb_data(TRUE);
            }
        }
        // if(l_press && dcnt < 20)
        // {
        //     tlkapi_send_string_data(APP_LOG_EN, "tx", rf_tx_packet.buf, 24);
        // }
    }
    #if TPSLL_PM_OPTM_8K_ONLY_USE_2T_EN
    else if ((ll_ctx.step_3t1r == STEP_3T1R_3T) && tpsll_is_active_state())
    #else
    else if ((ll_ctx.step_3t1r == STEP_3T1R_3T))
    #endif
    { // SLOT2 3T ( and SLOT3 1R)
        // rf_set_ble_chn(chn_list[ll_ctx.step_3t1r]);
        #if TPSLL_8K_STX_SRX_EN
        rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, ll_ctx.last_stick + APP_SLOT2_3T_START_TIME_US * RF_SYSTEM_TIMER_TICK_1US);
        #else
        app_rf_start_stx2rx(ll_ctx.last_stick + APP_SLOT2_3T_START_TIME_US * RF_SYSTEM_TIMER_TICK_1US, KB_CONNECTED_RX_TIMEOUT); // set 3T tick time
        ll_ctx.rx_over = false;
        ll_ctx.cur_rx_flag = true;
        #endif
        // rf_tx_packet.buf[0] = 0xA3;

        {
            #if TPSLL_KEEP_ALIVE_EN
            if (fifo_empty_flag) { //sync with empty packet
                rf_tx_packet.pkt_type = P24G_PKT_TYPE_SYNC;
                #if TPSLL_8K_STX_SRX_EN
                pp_rf_set_tx_len(0);
                #else
                pp_rf_set_tx_len(0);
                #endif
                #if TPSLL_SPP_NOT_SWITCH_EN
                // rf_set_rx_maxlen(RF_PKT_ACK_LEN + DONGLE_SPP_MAX_LEN);
                rf_set_rx_maxlen(RF_PKT_ACK_LEN + KM_SPP_MAX_LEN);
                #else
                rf_set_rx_maxlen(RF_PKT_ACK_LEN);
                #endif
                // calc_hop_channels(ll_ctx.fno_last + 1, chn_mask);
                // ll_ctx.cur_rx_flag = true;
                return;
            }
            #endif
            app_fill_kb_data(FALSE);
            app_rf_stx2rx_set_tx_len(rf_tx_packet.rf_len);

            #if TPSLL_SPP_NOT_SWITCH_EN
            // rf_set_rx_maxlen(RF_PKT_ACK_LEN + DONGLE_SPP_MAX_LEN);
            rf_set_rx_maxlen(RF_PKT_ACK_LEN + DONGLE_SPP_MAX_LEN);
            #else
            rf_set_rx_maxlen(RF_PKT_ACK_LEN);
            #endif
        }

        // calc next slot hop channels
        // calc_hop_channels(ll_ctx.fno_last + 1, chn_mask);

        // if(l_press && dcnt < 20)
        // {
        //     tlkapi_send_string_data(APP_LOG_EN, "tx", rf_tx_packet.buf, 12);
        // }
        // ll_ctx.cur_rx_flag = true;
    }
    if (!fifo_empty_flag) {
        tpsll_pm_optm_time_refresh();
    }
}

_attribute_ram_code_sec_ void tpsll_8k_spp_slot_send(void)
{
    rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, stimer_get_tick()); // set 2T tick time

}

_attribute_ram_code_sec_ void app_8k_rf_pkt_send(void)
{
    if (ll_ctx.stk_state == STATE_CONNECTED)
    {
        tpsll_8k_km_send();
        // if(ll_ctx.used_sm == SLOT_MODE_KM)
        // {
        //     tpsll_8k_km_slot_send();
        // }else{
        //     tpsll_8k_spp_slot_send();
        // }
    }
    else if (ll_ctx.stk_state == STATE_PAIRING)
    { // pairing RSP

        if (ll_ctx.pair_st == STATE_PAIR_INIT)
        {
            rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, rf_stimer_get_tick());
            rf_tx_packet.buf[0] = CMD_PAIR_RSP;
            rf_tx_packet.buf[1] = ll_inf.tag & 0xff;
            rf_tx_packet.buf[2] = (ll_inf.tag >> 8) & 0xff;

            // rf_tx_packet.buf[1] = 10;
            pp_rf_set_tx_len(3);
        }else if (ll_ctx.pair_st == STATE_PAIR_RSP)
        {
            rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, rf_stimer_get_tick() + 5 * SYSTEM_TIMER_TICK_1US);
            rf_tx_packet.buf[0] = CMD_CONN_RSP;
            rf_tx_packet.buf[1] = SKIP_INTV_FOR_WRITE_FLASH;
            // rf_tx_packet.buf[1] = 1;
            rf_tx_packet.buf[2] = ll_inf.tag & 0xff;
            rf_tx_packet.buf[3] = (ll_inf.tag >> 8) & 0xff;

            rf_tx_packet.buf[4] = ll_inf.peer_tag & 0xff;
            rf_tx_packet.buf[5] = (ll_inf.peer_tag >> 8) & 0xff;
            tmemcpy(&rf_tx_packet.buf[6], ll_inf.mac, MAC_ADDR_LEN);

            // rf_tx_packet.buf[1] = 10;
            // pp_rf_set_tx_len(12);
            pp_rf_set_tx_len(13);
            rf_tx_packet.buf[12] = ll_ctx.user_rr;
            ll_ctx.pending_rr_fno = ll_ctx.fno_last + TPSLL_RR_DELAY_SWITCH_FNO_N;
            ll_ctx.chg_rr = ll_ctx.user_rr;
            ll_ctx.is_rr_changing = true;
        }
    }
    #if !TPSLL_NEW_RECON_EN
    else if (ll_ctx.stk_state == STATE_RECONNECT) {
        rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, rf_stimer_get_tick() + 3 * SYSTEM_TIMER_TICK_1US);

        reconn_rsp_packet_t *pt = (reconn_rsp_packet_t *)&rf_tx_packet;
        pt->cmd                 = CMD_RECONN_RSP;
//        pt->rr_customer         = ll_inf.report_rate_customer;
        pt->d_tag               = ll_inf.peer_tag;
        pt->km_tag              = ll_inf.tag;

        // pp_rf_set_tx_len(5);
        #if TPSLL_RECON_RSSI_SCAN_EN
        pp_rf_set_tx_len(9);
        pt->rr = ll_ctx.user_rr;
        pt->chn_tab[0] = reconn_chn_table[0];
        pt->chn_tab[1] = reconn_chn_table[1];
        pt->chn_tab[2] = reconn_chn_table[2];
        #else
        pp_rf_set_tx_len(6);
        pt->rr = ll_ctx.user_rr;
        #endif
        ll_ctx.pending_rr_fno = ll_ctx.fno_last + TPSLL_RR_DELAY_SWITCH_FNO_N;
        ll_ctx.chg_rr = ll_ctx.user_rr;
        ll_ctx.is_rr_changing = true;
    }
    #endif /* end of !TPSLL_NEW_RECON_EN */
}

_attribute_ram_code_sec_ void app_rf_pkt_send(void)
{
    switch (ll_ctx.report_rate)
    {
        case REPORT_RATE_8K:
            app_8k_rf_pkt_send();

            break;
        default:
            app_non_8k_rf_pkt_send();
        break;
    }
}

_attribute_ram_code_sec_ void app_set_d25f_timer0_anchor(void)
{
    if ((ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250)) || tpsll_is_keep_alive_active()) {
        return;
    }
    pp_timer0_start(ll_ctx.d25f_timer0_anchor_us * SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));
}

_attribute_ram_code_sec_ void app_set_next_stimer_anchor(void)
{
    if (ll_ctx.stk_state == STATE_IDLE) {
        ll_ctx.stick += (ll_ctx.pm_optm_level & TPSLL_PM_OPTM_LEVEL3) ?
            ((uint32_t)ll_ctx.pm_optm_3rd_interval_ms) * 1000 * SYSTEM_TIMER_TICK_1US :
            ((uint32_t)ll_ctx.pm_optm_4th_interval_ms) * 1000 * SYSTEM_TIMER_TICK_1US;
        systimer_set_irq_capture(ll_ctx.stick);
        return;
    }

    #if TPSLL_RECON_RSSI_SCAN_EN && !TPSLL_NEW_RECON_EN
    if (ll_ctx.recon_rssi_scanning_cnt) {
        if (ll_ctx.recon_rssi_scanning_cnt == 1) {
            ll_ctx.stick += 150 * SYSTEM_TIMER_TICK_1US;//ll_ctx.stick += 100 * SYSTEM_TIMER_TICK_1US;
        } else {
            ll_ctx.stick += TPSLL_RECON_RSSI_SCAN_TIME_US * SYSTEM_TIMER_TICK_1US;
        }
        ll_ctx.recon_rssi_scanning_cnt++;
        systimer_set_irq_capture(ll_ctx.stick);
        return;
    }
    #endif

    if (ll_ctx.stk_state == STATE_CONNECTED || ll_ctx.pair_st == STATE_PAIR_RSP) {
        // ll_ctx.stick += DONGLE_INTERVAL_US * SYSTEM_TIMER_TICK_1US;
        ll_ctx.stick += ll_ctx.interval_us * SYSTEM_TIMER_TICK_1US;
    } else {
        ll_ctx.stick += PAIRING_INTERVAL_US * SYSTEM_TIMER_TICK_1US;
    }

    #if 0// TPSLL_NEW_KEEP_ALIVE_EN
    if(ll_ctx.report_rate & REPORT_RATE_8K)
    {
        ll_ctx.fno += (ll_ctx.interval_us / DONGLE_INTERVAL_US);
    }else
    #endif
    {
        ll_ctx.fno++;
    }
    

    if(ll_ctx.is_rr_h2l){
        ll_ctx.stick -= 4 * SYSTEM_TIMER_TICK_1US;
        ll_ctx.is_rr_h2l = false;
    }

    #if TPSLL_RR_CHG_WITH_RECON_EN
    #if TPSLL_NEW_RECON_EN
    if (ll_ctx.rr_chg_with_recon) {
        if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP2) {
            ll_ctx.rr_chg_with_recon = RR_RECON_STEP3;
        } else if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP3) {
            ll_ctx.rr_chg_with_recon = RR_RECON_STEP4;
            if (TPSLL_SAVE_RR_CONFIG_TIME_US > ll_ctx.interval_us) {
                ll_ctx.stick += (TPSLL_SAVE_RR_CONFIG_TIME_US - ll_ctx.interval_us) * SYSTEM_TIMER_TICK_1US;
            } else {
                ll_ctx.stick -= (ll_ctx.interval_us - TPSLL_SAVE_RR_CONFIG_TIME_US) * SYSTEM_TIMER_TICK_1US;
            }
            ll_ctx.rr_chg_recon_time_tick = clock_time() | 1;
            tpsll_wfi_inter_en(false);
            tpsll_suspend_inter_en(false);
            tpsll_connect_lost();
            tpsll_enable_reconn(true);
            // stack_evt_post_from_isr(TPSLL_EVT_SAVE_REPORT_RATE, (void *)&ll_ctx.user_rr, 1);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_SAVE_REPORT_RATE, (void *)&ll_ctx.user_rr, 1);
            // if (ll_ctx.user_rr == APP_DEFAULT_REPORT_RATE) {
            //     stack_evt_post_from_isr(TPSLL_EVT_REPORT_RATE_CHANGED, (void *)&ll_ctx.report_rate, 1);
            // }
        } else if ((ll_ctx.rr_chg_with_recon == RR_RECON_STEP6) && (ll_ctx.report_rate & REPORT_RATE_8K)) {
            if (!ll_ctx.is_rr_changing)
            {
                app_set_d25f_timer0_anchor();
            }
        }
    } else if (ll_ctx.chg_dly_cnt) {
        ll_ctx.stick += TPSLL_RR_SWITCH_CONFIG_TIME_US * SYSTEM_TIMER_TICK_1US;
        ll_ctx.chg_dly_cnt = 0;
    } else {
        app_set_d25f_timer0_anchor();
    }
    #else /* end of TPSLL_NEW_RECON_EN */
    if (ll_ctx.rr_chg_with_recon) {
        if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP2) {
            ll_ctx.rr_chg_with_recon = RR_RECON_STEP3;
        } else if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP3) {
            ll_ctx.rr_chg_with_recon = RR_RECON_STEP4;
            if (TPSLL_SAVE_RR_CONFIG_TIME_US > ll_ctx.interval_us) {
                ll_ctx.stick += (TPSLL_SAVE_RR_CONFIG_TIME_US - ll_ctx.interval_us) * SYSTEM_TIMER_TICK_1US;
            } else {
                ll_ctx.stick -= (ll_ctx.interval_us - TPSLL_SAVE_RR_CONFIG_TIME_US) * SYSTEM_TIMER_TICK_1US;
            }
            ll_ctx.rr_chg_recon_time_tick = clock_time() | 1;
            tpsll_wfi_inter_en(false);
            tpsll_suspend_inter_en(false);
            tpsll_connect_lost();
            tpsll_enable_reconn(true);
            // stack_evt_post_from_isr(TPSLL_EVT_SAVE_REPORT_RATE, (void *)&ll_ctx.user_rr, 1);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_SAVE_REPORT_RATE, (void *)&ll_ctx.user_rr, 1);
            if (ll_ctx.user_rr == APP_DEFAULT_REPORT_RATE) {
                // stack_evt_post_from_isr(TPSLL_EVT_REPORT_RATE_CHANGED, (void *)&ll_ctx.report_rate, 1);
                s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_REPORT_RATE_CHANGED,  (void *)&ll_ctx.report_rate, 1);
            }
        }
    } else if (ll_ctx.chg_dly_cnt) {
        ll_ctx.stick += TPSLL_RR_SWITCH_CONFIG_TIME_US * SYSTEM_TIMER_TICK_1US;
        ll_ctx.chg_dly_cnt = 0;
    }
    #endif /* end of TPSLL_NEW_RECON_EN #else */
    #else
    if(ll_ctx.chg_dly_cnt)
    {
        ll_ctx.stick += ll_ctx.chg_dly_cnt * ll_ctx.interval_us * SYSTEM_TIMER_TICK_1US;
        ll_ctx.chg_dly_cnt = 0;
    }
    #endif
    if (ll_ctx.is_rr_l2h) {
        ll_ctx.stick += 3 * SYSTEM_TIMER_TICK_1US;
        ll_ctx.is_rr_l2h = false;
    }
    systimer_set_irq_capture(ll_ctx.stick);
}

_attribute_ram_code_sec_ void app_update_pairing_stimer_anchor(uint16_t diff_us)
{
    #if TPSLL_NEW_RECON_EN
/////////////////////////////////////////
    systimer_clr_irq_mask();

    if (ll_ctx.stk_state == STATE_PAIRING) {
        if(ll_ctx.report_rate & REPORT_RATE_8K)
        {
            ll_ctx.stick = reg_rf_timestamp + diff_us * SYSTEM_TIMER_TICK_1US - TPSLL_PAIR_TICK_UPDATA_SYNC_DIFF_US * SYSTEM_TIMER_TICK_1US;
        }else {
            // ll_ctx.stick = reg_rf_timestamp + (APP_PAIRING_TS_TO_DONGLE_NEXT_STIMER_TIME - 3) * SYSTEM_TIMER_TICK_1US - APP_PAIRING_CALI_FINE_ADJ_WITH_TICK - 3;
        }
        systimer_set_irq_capture(ll_ctx.stick);
    } else if (ll_ctx.stk_state == STATE_RECONNECT) {
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
        ll_ctx.stick = reg_rf_timestamp + diff_us * SYSTEM_TIMER_TICK_1US - (TPSLL_RECON_TICK_UPDATA_SYNC_DIFF_US * SYSTEM_TIMER_TICK_1US);
        systimer_set_irq_capture(ll_ctx.stick);
    }
/////////////////////////////////////////
    #else /* end of TPSLL_NEW_RECON_EN */
    (void)diff_us;
    systimer_clr_irq_mask();
    DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);

    if (ll_ctx.stk_state == STATE_PAIRING || ll_ctx.stk_state == STATE_RECONNECT) {
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
        if(ll_ctx.report_rate & REPORT_RATE_8K)
        {
            ll_ctx.stick = reg_rf_timestamp + APP_PAIRING_TS_TO_DONGLE_NEXT_STIMER_TIME * SYSTEM_TIMER_TICK_1US - APP_PAIRING_CALI_FINE_ADJ_WITH_TICK;
        }else {
            // ll_ctx.stick = reg_rf_timestamp + (APP_PAIRING_TS_TO_DONGLE_NEXT_STIMER_TIME - 3) * SYSTEM_TIMER_TICK_1US - APP_PAIRING_CALI_FINE_ADJ_WITH_TICK - 3;
        }
        systimer_set_irq_capture(ll_ctx.stick);
    }
    #endif /* end of TPSLL_NEW_RECON_EN #else */
    systimer_clr_irq_status();
    systimer_set_irq_mask();
}

 #if TPSLL_NEW_KEEP_ALIVE_EN
_attribute_ram_code_sec_ void tpsll_set_next_keep_alive_stimer_anchor(uint32_t alive_us)
{
    ll_ctx.fno += (alive_us / ll_ctx.interval_us);
    ll_ctx.stick += alive_us * SYSTEM_TIMER_TICK_1US;
    systimer_set_irq_capture(ll_ctx.stick);
    
}
#endif

// volatile unsigned int zcnt = 0;
// volatile unsigned int zcnt1 = 0;
_attribute_ram_code_sec_ void app_update_conn_stimer_anchor(uint8_t ack)
{
    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
    unsigned int new_stick = 0;
    unsigned int now_stick = 0;
    int tick_diff = 0;
    ll_ctx.tick_diff_max = 6 * SYSTEM_TIMER_TICK_1US;


    if(ll_ctx.report_rate & REPORT_RATE_8K) {
        new_stick = reg_rf_timestamp + APP_CONN_TS_TO_NEXT_STIMER_US * SYSTEM_TIMER_TICK_1US - APP_CONN_CALI_FINE_ADJ_WITH_TICK;
        #if TPSLL_PM_OPTM_EN
        if (tpsll_pm_optm_is_long_suspend_support_level_active()) {
            ll_ctx.tick_diff_max = TPSLL_PM_OPTM_8K_RX_WIN_EXPAND_US * SYSTEM_TIMER_TICK_1US;
            if (ack && (ll_ctx.pm_optm_ref_fno_cnt == 1)) {
                ll_ctx.pm_optm_ref_fno_cnt = 2;
                ll_ctx.pm_optm_level = TPSLL_PM_OPTM_LEVEL_EXITING;
            }
        }
        #endif
    } else if (ll_ctx.report_rate & REPORT_RATE_4K) {
        new_stick = reg_rf_timestamp + (ll_ctx.interval_us - 107) * SYSTEM_TIMER_TICK_1US - 4;
        ll_ctx.tick_diff_max = 10 * SYSTEM_TIMER_TICK_1US;
        #if TPSLL_PM_OPTM_EN
        if (tpsll_pm_optm_is_long_suspend_support_level_active()) {
            ll_ctx.tick_diff_max = TPSLL_PM_OPTM_N8K_RX_WIN_EXPAND_US * SYSTEM_TIMER_TICK_1US;
            if (ack && (ll_ctx.pm_optm_ref_fno_cnt == 1)) {
                ll_ctx.pm_optm_ref_fno_cnt = 2;
                ll_ctx.pm_optm_level = TPSLL_PM_OPTM_LEVEL_EXITING;
            }
        }
        #endif
    } else if (ll_ctx.report_rate & REPORT_RATE_2K) {
        // new_stick = reg_rf_timestamp + APP_CONN_TS_TO_NEXT_STIMER_US_NOT_8K * SYSTEM_TIMER_TICK_1US - APP_CONN_CALI_FINE_ADJ_WITH_TICK - 12 - 12;
        new_stick = reg_rf_timestamp +  (ll_ctx.interval_us - 106 - 7 - 21 - 7 - FOR_KB_ALL_KEY_SYNC_MARGIN_TIME) * SYSTEM_TIMER_TICK_1US + 11 - 10;
        ll_ctx.tick_diff_max = 10 * SYSTEM_TIMER_TICK_1US;
        #if TPSLL_PM_OPTM_EN
        if (tpsll_pm_optm_is_long_suspend_support_level_active()) {
            ll_ctx.tick_diff_max = TPSLL_PM_OPTM_N8K_RX_WIN_EXPAND_US * SYSTEM_TIMER_TICK_1US;
            if (ack && (ll_ctx.pm_optm_ref_fno_cnt == 1)) {
                ll_ctx.pm_optm_ref_fno_cnt = 2;
                ll_ctx.pm_optm_level = TPSLL_PM_OPTM_LEVEL_EXITING;
            }
        }
        #endif
    }
    // else if (ll_ctx.report_rate & REPORT_RATE_1K) {
    //     // new_stick = reg_rf_timestamp + APP_CONN_TS_TO_NEXT_STIMER_US_NOT_8K * SYSTEM_TIMER_TICK_1US - APP_CONN_CALI_FINE_ADJ_WITH_TICK - 12 - 12;
    //     new_stick = reg_rf_timestamp +  (ll_ctx.interval_us - 106 - 7 - 21 - 7 - FOR_KB_ALL_KEY_SYNC_MARGIN_TIME) * SYSTEM_TIMER_TICK_1US;
    //     ll_ctx.tick_diff_max = 10 * SYSTEM_TIMER_TICK_1US;
    // } else if (ll_ctx.report_rate & REPORT_RATE_500) {
    //     // new_stick = reg_rf_timestamp + APP_CONN_TS_TO_NEXT_STIMER_US_NOT_8K * SYSTEM_TIMER_TICK_1US - APP_CONN_CALI_FINE_ADJ_WITH_TICK - 12 - 12;
    //     new_stick = reg_rf_timestamp +  (ll_ctx.interval_us - 106 - 7 - 21 - 7 - FOR_KB_ALL_KEY_SYNC_MARGIN_TIME) * SYSTEM_TIMER_TICK_1US - SYSTEM_TIMER_TICK_1US / 5;
    //     ll_ctx.tick_diff_max = 10 * SYSTEM_TIMER_TICK_1US;
    // }

    #if TPSLL_NEW_KEEP_ALIVE_EN
    if(ll_ctx.report_rate & REPORT_RATE_8K) {
        ENTER_CRITICAL();
        now_stick = clock_time();

        if (ll_ctx.cur_rx_flag) {
            if (tick1_exceed_tick2(now_stick + TPSLL_UPDATE_STIMER_HANDLE_TOO_CLOSE_TH_US * SYSTEM_TIMER_TICK_1US, ll_ctx.stick)) {
                //STIMER IRQ is too close, set the stimer to trigger after 2us
                systimer_set_irq_capture(now_stick + TPSLL_UPDATE_STIMER_HANDLE_TOO_CLOSE_TH_US * SYSTEM_TIMER_TICK_1US);
                systimer_clr_irq_status();
            }
            tick_diff = (int)(new_stick - (ll_ctx.last_stick + DONGLE_INTERVAL_US * SYSTEM_TIMER_TICK_1US));
        } else {
            //STIMER IRQ has been handled
            tick_diff = (int)(new_stick - ll_ctx.last_stick);
            if (tick_diff) {
                //set next stick with one more interval time
                ll_ctx.stick = new_stick + DONGLE_INTERVAL_US * SYSTEM_TIMER_TICK_1US;
                systimer_set_irq_capture(ll_ctx.stick);
            }
            EXIT_CRITICAL();
            return;
        }
    } else if(ll_ctx.report_rate & REPORT_RATE_4K) {
        tick_diff = (int)(new_stick - (ll_ctx.last_stick + DONGLE_INTERVAL_US * SYSTEM_TIMER_TICK_1US));
    } else {
        tick_diff = (int)(new_stick - ll_ctx.stick);
    }
    #else
    int tick_diff = (int)(new_stick - ll_ctx.stick);
    #endif

    if (tick_diff) {
        // if (tick_diff < tick_diff_max && tick_diff > - tick_diff_max) {
        // if ((tick_diff < ll_ctx.tick_diff_max && tick_diff > - ll_ctx.tick_diff_max) &&
        //     (((ll_ctx.report_rate & REPORT_RATE_8K) && ll_ctx.cur_rx_flag) || (ll_ctx.report_rate & REPORT_RATE_NOT_8K_MSK)))
        {
            if (tick_diff < ll_ctx.tick_diff_max && tick_diff > - ll_ctx.tick_diff_max) {
            // uint32_t th_tick = clock_time() + 1 * RF_SYSTEM_TIMER_TICK_1US;

            // if (tick1_exceed_tick2(ll_ctx.stick, th_tick) && tick1_exceed_tick2(new_stick, th_tick))
            {
            #if TPSLL_NEW_KEEP_ALIVE_EN
            if (ll_ctx.report_rate & (REPORT_RATE_8K|REPORT_RATE_4K))
            {
                // ll_ctx.stick -= tick_diff;
                ll_ctx.stick += tick_diff;
            }
            else
            {
                ll_ctx.stick = new_stick;
            }
            #else
            ll_ctx.stick = new_stick;
            #endif
            systimer_set_irq_capture(ll_ctx.stick);
            }
            // zcnt = 0;

            #if TPSLL_KEEP_ALIVE_EN
            if (ack && tpsll_is_keep_alive_active() && !ll_ctx.is_rr_changing) {
                #if KEEP_ALIVE_STIMER_SYNC_CNT
                if ((ll_ctx.report_rate & (REPORT_RATE_8K | REPORT_RATE_4K)) && (abs(tick_diff) >= KEEP_ALIVE_STIMER_SYNC_TH_US * SYSTEM_TIMER_TICK_1US)) {
                    ll_ctx.sync_cntdwn = KEEP_ALIVE_STIMER_SYNC_CNT;
                    if (ll_ctx.sync_cntdwn) {
                        ll_ctx.sync_cntdwn--;
                    }
                } else {
                    if (ll_ctx.sync_cntdwn) {
                        ll_ctx.sync_cntdwn--;
                    } else {
                        ll_ctx.synced = 1;
                        ll_ctx.alive_interval_cnt = ll_ctx.alive_interval_max;
                        #if TPSLL_NEW_KEEP_ALIVE_EN
                        if (ll_ctx.report_rate & (REPORT_RATE_8K) || ((ll_ctx.report_rate & (REPORT_RATE_4K | REPORT_RATE_2K)) && ll_ctx.is_keep_alive)) {
                            tpsll_set_next_keep_alive_stimer_anchor(ll_ctx.keep_alive_sync_us);
                            ll_ctx.keep_alive_cnt = 0;
                        }
                        #endif
                    }
                }
                #else
                ll_ctx.synced = 1;
                ll_ctx.alive_interval_cnt = ll_ctx.alive_interval_max;
                #endif
            }
            #if TPSLL_KB_ACTIVE_PM_OPTM_EN
            else if (ack && ll_ctx.kb_alive_fno_msk && !ll_ctx.is_rr_changing) {
                #if KEEP_ALIVE_STIMER_SYNC_CNT
                if ((ll_ctx.report_rate & (REPORT_RATE_8K | REPORT_RATE_4K | REPORT_RATE_2K)) && (abs(tick_diff) >= 2 * SYSTEM_TIMER_TICK_1US)) {
                    ll_ctx.sync_cntdwn = 1;
                    if (ll_ctx.sync_cntdwn) {
                        ll_ctx.sync_cntdwn--;
                    }
                } else {
                    if (ll_ctx.sync_cntdwn) {
                        ll_ctx.sync_cntdwn--;
                    } else {
                        ll_ctx.synced = 1;
                    }
                }
                #else
                ll_ctx.synced = 1;
                #endif
            }
            #endif /* end of TPSLL_KB_ACTIVE_PM_OPTM_EN */
            #else
            (void)ack;
            #endif
            #if TPSLL_NEW_SYNC_EN
            if (!(ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500)))
            {
                ll_ctx.time_synced_cnt++;
            }
            #endif
        }
        // else if ((ll_ctx.report_rate & REPORT_RATE_8K) && !ll_ctx.cur_rx_flag) {
        //     ll_ctx.stick = new_stick + DONGLE_INTERVAL_US * SYSTEM_TIMER_TICK_1US;
        //     systimer_set_irq_capture(ll_ctx.stick);
        }
        if(ll_ctx.report_rate & REPORT_RATE_8K) {
            EXIT_CRITICAL();
        }
    }
    else
    {
        if(ll_ctx.report_rate & REPORT_RATE_8K) {
            EXIT_CRITICAL();
        }
        #if TPSLL_KEEP_ALIVE_EN
        if (ack && tpsll_is_keep_alive_active() && !ll_ctx.is_rr_changing) {
            #if KEEP_ALIVE_STIMER_SYNC_CNT
            if (ll_ctx.sync_cntdwn) {
                ll_ctx.sync_cntdwn--;
            } else {
                ll_ctx.synced = 1;
                ll_ctx.alive_interval_cnt = ll_ctx.alive_interval_max;
                #if TPSLL_NEW_KEEP_ALIVE_EN
                if (ll_ctx.report_rate & (REPORT_RATE_8K) || ((ll_ctx.report_rate & (REPORT_RATE_4K | REPORT_RATE_2K)) && ll_ctx.is_keep_alive)) {
                    ENTER_CRITICAL();
                    tpsll_set_next_keep_alive_stimer_anchor(ll_ctx.keep_alive_sync_us);
                    EXIT_CRITICAL();
                    ll_ctx.keep_alive_cnt = 0;
                }
                #endif
            }
            #else
            ll_ctx.synced = 1;
            ll_ctx.alive_interval_cnt = ll_ctx.alive_interval_max;
            #endif
        }
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        else if (ack && ll_ctx.kb_alive_fno_msk && !ll_ctx.is_rr_changing) {
            #if KEEP_ALIVE_STIMER_SYNC_CNT
            if ((ll_ctx.report_rate & (REPORT_RATE_8K | REPORT_RATE_4K | REPORT_RATE_2K)) && (abs(tick_diff) >= 1 * SYSTEM_TIMER_TICK_1US)) {
                ll_ctx.sync_cntdwn = 1;
            } else {
                if (ll_ctx.sync_cntdwn) {
                    ll_ctx.sync_cntdwn--;
                } else {
                    ll_ctx.synced = 1;
                }
            }
            #else
            ll_ctx.synced = 1;
            #endif
        }
        #endif /* end of TPSLL_KB_ACTIVE_PM_OPTM_EN */
        #endif /* end of TPSLL_KEEP_ALIVE_EN */
    }

    #if 0
    if(!tick_diff)
    {
        zcnt = 0;
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
    }

   
    if (!(ll_ctx.report_rate & REPORT_RATE_8K))
    {
        uint32_t sync_tick = reg_rf_timestamp;
        uint32_t sync_dis = sync_tick - ll_ctx.last_stick;
        zcnt1++;
        if (zcnt1 < 5)
            tlkapi_printf(0x10F1, " ll_ctx.last_stick:%x ll_ctx.stick:%x new_stick:%x tick_diff:%x sync_tick:%x sync_dis:%x rr:%d\n",
                            ll_ctx.last_stick, ll_ctx.stick, new_stick, tick_diff, sync_tick, sync_dis, ll_ctx.report_rate);
    }
    #endif
}

_attribute_ram_code_sec_ void app_update_conn_stimer_anchor_125(int8_t d_us)
{
    // static uint32_t zdcnt = 0;
    
    // if(zdcnt < 5)
    {
        // zdcnt++;
        // tlkapi_printf(APP_LOG_EN, "d_us:%d\n", d_us);
    }

    if (d_us && d_us > -D_SUSPEND_EARLY_TIME_US && d_us < D_SUSPEND_EARLY_TIME_US) {
    // if (d_us && d_us > -20 && d_us < 20) {
        ll_ctx.stick -= d_us * RF_SYSTEM_TIMER_TICK_1US;
        ll_ctx.last_stick -= d_us * RF_SYSTEM_TIMER_TICK_1US;
        systimer_set_irq_capture(ll_ctx.stick);
        // if ((d_us > 4) || (d_us < -4)) {
        //     tlkapi_printf(APP_LOG_EN, "(%d)", d_us);
        //     // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
        //     // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
        // }
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX1]);
        ll_ctx.time_synced_cnt = 2;
    } else if (d_us) {
        tlkapi_printf(APP_LOG_EN, "offset too large!!! %d us \n", d_us);
    } else {
        //d_us is 0
    }

    #if TPSLL_KEEP_ALIVE_EN && TPSLL_PM_OPTM_N8K_SUSPEND_EN
    if (tpsll_is_keep_alive_active() && !ll_ctx.is_rr_changing) {
        if (ll_ctx.pm_optm_ref_fno_cnt == 1) {
            ll_ctx.pm_optm_ref_fno_cnt = 2;
            ll_ctx.pm_optm_level = TPSLL_PM_OPTM_LEVEL_EXITING;
        } else {
            tpsll_set_next_keep_alive_stimer_anchor(ll_ctx.keep_alive_sync_us);
        }
    }
    #endif
}

#if TPSLL_NEW_SYNC_EN
static inline void tpsll_update_conn_stimer_anchor_new(uint16_t diff_us)
{
    if (diff_us) {
        ll_ctx.stick = reg_rf_timestamp + diff_us * SYSTEM_TIMER_TICK_1US - TPSLL_CONN_TICK_UPDATA_SYNC_DIFF_US * SYSTEM_TIMER_TICK_1US;
        systimer_set_irq_capture(ll_ctx.stick);
        // tlkapi_printf(APP_LOG_EN, "(%d\n", diff_us);
    }
    #if TPSLL_KEEP_ALIVE_EN
    if (tpsll_is_keep_alive_active() && !ll_ctx.is_rr_changing) {
        #if KEEP_ALIVE_STIMER_SYNC_CNT
        // ll_ctx.synced = 1;
        ll_ctx.alive_interval_cnt = ll_ctx.alive_interval_max;
        #if TPSLL_PM_OPTM_N8K_SUSPEND_EN
        if (ll_ctx.pm_optm_ref_fno_cnt == 1) {
            ll_ctx.pm_optm_ref_fno_cnt = 2;
            ll_ctx.pm_optm_level = TPSLL_PM_OPTM_LEVEL_EXITING;
        } else {
            tpsll_set_next_keep_alive_stimer_anchor(ll_ctx.keep_alive_sync_us);
        }
        #else
        #if TPSLL_NEW_KEEP_ALIVE_EN
        if (ll_ctx.is_keep_alive) {
            tpsll_set_next_keep_alive_stimer_anchor(ll_ctx.keep_alive_sync_us);
            ll_ctx.keep_alive_cnt = 0;
        }
        #endif /* end of TPSLL_NEW_KEEP_ALIVE_EN */
        #endif
        #else
        ll_ctx.synced = 1;
        ll_ctx.alive_interval_cnt = ll_ctx.alive_interval_max;
        #endif
    }
    #endif
    ll_ctx.synced = 1;
}
#endif


static inline bool tpsll_is_report_rate_valid(uint8_t rr)
{
    return (rr && ((rr & (rr - 1)) == 0) && (rr & REPORT_RATE_MASK));
}

_attribute_ram_code_sec_ void app_d24g_set_report_rate(uint8_t rr)
{
    ll_ctx.report_rate = rr;
    if(ll_ctx.last_rr != rr)
    {
        ll_ctx.last_rr = rr;
        if (ll_ctx.suspend_allowed_ptr) {
            *ll_ctx.suspend_allowed_ptr = false;
        }
    }else{
        return;
    }
    // if (rr & REPORT_RATE_8K)
    // {
    //     ll_ctx.slot1_us = APP_SLOT1_START_TIME_US;
    //     // ll_inf.slot1_2t_us = APP_SLOT1_2T_START_TIME_US;
    // }
    // else
    // {
    //     ll_ctx.slot1_us = APP_SLOT1_START_TIME_US + 16 + 50 - 6 + 3; // 121
    //     // ll_inf.slot1_2t_us = APP_SLOT1_2T_START_TIME_US + 16 + 50;
    //     app_rf_set_rx_wait(0); // rx wait
    // }

    if (rr & REPORT_RATE_8K) {
        ll_ctx.lost_cnt_max = LL_LOST_8K_MAX_CNT;
    } else {
        ll_ctx.lost_cnt_max = LL_LOST_N8K_MAX_CNT;
    }

    switch (rr)
    {
    case REPORT_RATE_8K:
    // case REPORT_RATE_4K:
        ll_ctx.d25f_timer0_anchor_us = D25F_TIMER0_ANCHOR_US_8K;
        ll_ctx.interval_us = DONGLE_INTERVAL_US;
        ll_ctx.slot1_us = APP_SLOT1_START_TIME_US;
        rf_set_tx_settle_time(12);
        app_rf_set_rx_wait(1); // rx wait
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT;
        ll_ctx.precise_tx = false;
        ll_ctx.sn_8k = 1;
        ll_ctx.nesn_8k = 0;
        ll_ctx.s_8k = 0;
        rf_tx_packet.snnesn = 0;
        #if TPSLL_KEEP_ALIVE_EN
        ll_ctx.alive_interval_max = KEEP_ALIVE_INTERVAL_8K_MAX_CNT;
        #endif
        #if TPSLL_SPP_NOT_SWITCH_EN
        ll_ctx.spp_quota_active = 0;
        ll_ctx.ms_send_cnt = 0;
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        ll_ctx.kb_alive_16ms_intervals_shift = 6;
        ll_ctx.kb_alive_16ms_intervals = 1 << 6;
        ll_ctx.kb_alive_16ms_intervals_msk = (1 << 6) - 1;
        #endif
        #if TPSLL_N8K_FHSS_EN
        ll_ctx.afh_shift = TPSLL_FHSS_AFH_SLOT_SHIFT;
        #endif
        ll_ctx.suspend_wakeup_early_time_us = APP_WAKEUP_EARLY_TIME_US;

        break;

    case REPORT_RATE_4K:
        ll_ctx.d25f_timer0_anchor_us = D25F_TIMER0_ANCHOR_US_4K;
        ll_ctx.interval_us = DONGLE_INTERVAL_US;
        // ll_ctx.slot1_us = APP_SLOT1_START_TIME_US;
        ll_ctx.slot1_us = 130;
        rf_set_tx_settle_time(12);
        app_rf_set_rx_wait(1); // rx wait
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT;
        ll_ctx.precise_tx = false;
        ll_ctx.max_slot = MAX_TASK_SLOTS_4K;
        #if TPSLL_KEEP_ALIVE_EN
        ll_ctx.alive_interval_max = KEEP_ALIVE_INTERVAL_4K_MAX_CNT;
        ll_ctx.ka_mask = 0x7;
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        ll_ctx.kb_alive_16ms_intervals_shift = 6;
        ll_ctx.kb_alive_16ms_intervals = 1 << 6;
        ll_ctx.kb_alive_16ms_intervals_msk = (1 << 6) - 1;
        #endif
        #if TPSLL_N8K_FHSS_EN
        ll_ctx.afh_shift = TPSLL_N8K_FHSS_AFH_SLOT_SHIFT;
        #endif
        ll_ctx.suspend_wakeup_early_time_us = APP_WAKEUP_EARLY_TIME_US;

        break;
    case REPORT_RATE_2K:
        ll_ctx.d25f_timer0_anchor_us = D25F_TIMER0_ANCHOR_US_2K;
        ll_ctx.interval_us = DONGLE_INTERVAL_US << 1;
        ll_ctx.max_slot = MAX_TASK_SLOTS_2K;
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT >> 1;
        ll_ctx.precise_tx = false;
        rf_set_tx_settle_time(18);//for more data prepare time
        #if TPSLL_KEEP_ALIVE_EN
        ll_ctx.alive_interval_max = KEEP_ALIVE_INTERVAL_2K_MAX_CNT;
        ll_ctx.ka_mask = 0xf;
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        ll_ctx.kb_alive_16ms_intervals_shift = 5;
        ll_ctx.kb_alive_16ms_intervals = 1 << 5;
        ll_ctx.kb_alive_16ms_intervals_msk = (1 << 5) - 1;
        #endif
        #if TPSLL_N8K_FHSS_EN
        ll_ctx.afh_shift = TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 1;
        #endif
        ll_ctx.suspend_wakeup_early_time_us = APP_LOW_CLK_WAKEUP_EARLY_TIME_US;
        break;
    case REPORT_RATE_1K:
        ll_ctx.d25f_timer0_anchor_us = D25F_TIMER0_ANCHOR_US_1K;
        ll_ctx.interval_us = DONGLE_INTERVAL_US << 2;
        rf_set_tx_settle_time(18);//for more data prepare time
        // app_rf_set_rx_wait(8); // rx wait
        ll_ctx.max_slot = MAX_TASK_SLOTS_1K;
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT >> 2;
        ll_ctx.precise_tx = false;
        #if TPSLL_KEEP_ALIVE_EN
        ll_ctx.alive_interval_max = KEEP_ALIVE_INTERVAL_1K_MAX_CNT;
        ll_ctx.ka_mask = 0x1;
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        ll_ctx.kb_alive_16ms_intervals_shift = 4;
        ll_ctx.kb_alive_16ms_intervals = 1 << 4;
        ll_ctx.kb_alive_16ms_intervals_msk = (1 << 4) - 1;
        #endif
        ll_ctx.time_synced_cnt = 0;
        #if TPSLL_N8K_FHSS_EN
        ll_ctx.afh_shift = TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 2;
        #endif
        ll_ctx.suspend_wakeup_early_time_us = APP_LOW_CLK_WAKEUP_EARLY_TIME_US;
        break;
    case REPORT_RATE_500:
        ll_ctx.d25f_timer0_anchor_us = D25F_TIMER0_ANCHOR_US_500;
        ll_ctx.interval_us = DONGLE_INTERVAL_US << 3;
        ll_ctx.max_slot = MAX_TASK_SLOTS_500;
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT >> 3;
        ll_ctx.precise_tx = false;
        rf_set_tx_settle_time(18);//for more data prepare time
        #if TPSLL_KEEP_ALIVE_EN
        ll_ctx.alive_interval_max = KEEP_ALIVE_INTERVAL_500_MAX_CNT;
        ll_ctx.ka_mask = 0x3;
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        ll_ctx.kb_alive_16ms_intervals_shift = 3;
        ll_ctx.kb_alive_16ms_intervals = 1 << 3;
        ll_ctx.kb_alive_16ms_intervals_msk = (1 << 3) - 1;
        #endif
        #if TPSLL_N8K_FHSS_EN
        ll_ctx.afh_shift = TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 3;
        #endif
        ll_ctx.suspend_wakeup_early_time_us = APP_LOW_CLK_WAKEUP_EARLY_TIME_US;
        break;
    case REPORT_RATE_250:
        ll_ctx.d25f_timer0_anchor_us = D25F_TIMER0_ANCHOR_US_250;
        ll_ctx.interval_us = DONGLE_INTERVAL_US << 4;
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT >> 4;
        ll_ctx.max_slot = MAX_TASK_SLOTS_250;
        ll_ctx.precise_tx = true;
        rf_set_tx_settle_time(12);
        #if TPSLL_KEEP_ALIVE_EN
        ll_ctx.alive_interval_max = 0;
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        ll_ctx.kb_alive_16ms_intervals_shift = 0;
        ll_ctx.kb_alive_16ms_intervals = 0;
        ll_ctx.kb_alive_16ms_intervals_msk = 0;
        #endif
        #if TPSLL_N8K_FHSS_EN
        ll_ctx.afh_shift = TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 4;
        #endif
        ll_ctx.suspend_wakeup_early_time_us = APP_LOW_CLK_WAKEUP_EARLY_TIME_US;
        break;
    case REPORT_RATE_125:
        ll_ctx.d25f_timer0_anchor_us = D25F_TIMER0_ANCHOR_US_125;
        ll_ctx.interval_us = DONGLE_INTERVAL_US << 5;
        ll_ctx.max_slot = MAX_TASK_SLOTS_125;
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT >> 5;
        ll_ctx.precise_tx = true;
        rf_set_tx_settle_time(12);
        #if TPSLL_KEEP_ALIVE_EN
        ll_ctx.alive_interval_max = 0;
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        ll_ctx.kb_alive_16ms_intervals_shift = 0;
        ll_ctx.kb_alive_16ms_intervals = 0;
        ll_ctx.kb_alive_16ms_intervals_msk = 0;
        #endif
        #if TPSLL_N8K_FHSS_EN
        ll_ctx.afh_shift = TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 5;
        #endif
        ll_ctx.suspend_wakeup_early_time_us = APP_LOW_CLK_WAKEUP_EARLY_TIME_US;
        break;
    default:
        ll_ctx.d25f_timer0_anchor_us = D25F_TIMER0_ANCHOR_US_DEFAULT;
        ll_ctx.interval_us = DONGLE_INTERVAL_US;
        break;
    }

    #if TPSLL_NEW_KEEP_ALIVE_EN
    ll_ctx.is_keep_alive_sync = true;
    #endif

    #if TPSLL_KEEP_ALIVE_EN
    ll_ctx.synced = 0;
    ll_ctx.alive_interval_cnt = ll_ctx.alive_interval_max;

    #if TPSLL_PM_OPTM_EN
    ll_ctx.pm_optm_1st_ref_fno_cnt_th = ll_ctx.pm_optm_1st_start_time_ms * 1000 / ll_ctx.interval_us;
    ll_ctx.pm_optm_2nd_ref_fno_cnt_th = ll_ctx.pm_optm_2nd_start_time_ms * 1000 / ll_ctx.interval_us;
    ll_ctx.pm_optm_3rd_ref_fno_cnt_th = ll_ctx.pm_optm_3rd_start_time_ms * 1000 / ll_ctx.interval_us;
    ll_ctx.pm_optm_level = 0;

    if (rr & REPORT_RATE_8K) {
        ll_ctx.pm_optm_rr_chgd_ref_fno_cnt_th = ((ll_ctx.pm_optm_1st_start_time_ms * 1000 >= KEEP_ALIVE_SWITCH_8K_DELAY_US) ?
                                                ll_ctx.pm_optm_1st_ref_fno_cnt_th : (KEEP_ALIVE_SWITCH_8K_DELAY_US / ll_ctx.interval_us));
        ll_ctx.tick_diff_max = 6 * SYSTEM_TIMER_TICK_1US;
    } else {
        ll_ctx.pm_optm_rr_chgd_ref_fno_cnt_th = ((ll_ctx.pm_optm_1st_start_time_ms * 1000 >= KEEP_ALIVE_DELAY_US) ?
                                                ll_ctx.pm_optm_1st_ref_fno_cnt_th : (KEEP_ALIVE_DELAY_US / ll_ctx.interval_us));
        ll_ctx.tick_diff_max = 10 * SYSTEM_TIMER_TICK_1US;
    }
    #else /* end of TPSLL_PM_OPTM_EN */

    ll_ctx.alive_secondary_ref_cnt = ll_ctx.secondary_active_timeout*1000000/ll_ctx.interval_us;

    if (rr & REPORT_RATE_8K) {
        ll_ctx.alive_delay_time = (ll_ctx.event_active_timeout > KEEP_ALIVE_SWITCH_8K_DELAY_US) ? ll_ctx.event_active_timeout : KEEP_ALIVE_SWITCH_8K_DELAY_US;
        ll_ctx.tick_diff_max = 6 * SYSTEM_TIMER_TICK_1US;
    } else {
        ll_ctx.alive_delay_time = (ll_ctx.event_active_timeout > KEEP_ALIVE_DELAY_US) ? ll_ctx.event_active_timeout : KEEP_ALIVE_DELAY_US;
        ll_ctx.tick_diff_max = 10 * SYSTEM_TIMER_TICK_1US;
    }
    #endif /* end of TPSLL_PM_OPTM_EN #else */

    ll_ctx.is_rr_just_changed = true;

    #endif /* end of TPSLL_KEEP_ALIVE_EN */
}

_attribute_ram_code_sec_ void tpsll_generate_pair_encrypt_key(void)
{
    static uint8_t hash_buf[ENCRYPT_KEY_MAX_LEN + 2];
    uint16_t vid = ll_ctx.usb_vid;

    uint8_t len = ll_ctx.customer_key_len;
    memcpy(hash_buf, ll_ctx.customer_key, len);
    hash_buf[len]   = (vid >> 8) & 0xFF;
    hash_buf[len+1] = vid & 0xFF;

    ll_ctx.pair_encrypt_key = fnv1a_32_hash(hash_buf, len + 2);
}

uint8_t tpsll_set_pair_encrypt_key(const uint8_t *key, uint8_t key_len)
{
    if (key == NULL || key_len == 0 || key_len > ENCRYPT_KEY_MAX_LEN) {
        return TLK_ERR_INVALID_PARAM;
    }

    memcpy(ll_ctx.customer_key, key, key_len);
    ll_ctx.customer_key_len = key_len;

    if (s_app_cb && s_app_cb->usbd_get_dev_descp)
    {
        uint8_t *desc = s_app_cb->usbd_get_dev_descp(0);
        ll_ctx.usb_vid = desc[9] << 8 | desc[8];
        tpsll_generate_pair_encrypt_key();
        return TLK_SUCCESS;
    }
    else
    {
        return TLK_ERR_NULL;
    }
}

/**
 * @brief  validate the encrypt key of the PAIR_REQ packet
 */
pair_encrypt_status_t tpsll_verify_pair_encrypt_key(const rf_packet_t *pr)
{
    uint32_t recv_key;
    uint32_t local_key;

    recv_key  = ((uint32_t)pr->buf[1] << 24);
    recv_key |= ((uint32_t)pr->buf[2] << 16);
    recv_key |= ((uint32_t)pr->buf[3] <<  8);
    recv_key |= ((uint32_t)pr->buf[4] <<  0);

    local_key = ll_ctx.pair_encrypt_key;

    if (recv_key == local_key) {
        return PAIR_ENCRYPT_OK;
    } else {
        return PAIR_ENCRYPT_ERR;
    }
}

_attribute_ram_code_sec_ void app_rf_abnormal_handle(void)
{
}

// _attribute_ram_code_sec_ static void handle_ack(uint8_t ack_mask) {
//     if (ll_ctx.last_valid_slot_id < 0) return;  //last interval is all NULL packet

//     ll_ctx.acked = (ack_mask >> ll_ctx.last_valid_slot_id) & 0x01;
//     // printf("last_valid_slot_id: %d, acked: %d\n", last_valid_slot_id, acked);
// }
_attribute_ram_code_sec_ static void handle_ack(uint8_t ack_mask)
{
    if (ll_ctx.spp_flag && (ack_mask & 0x02))
    {
        // pp_fifo_pop(n22SppRxFifo);
        // ll_ctx.spp_flag = false;
        // ack_mask &= ~(0x1 << 1);
        // local_8k_spp_tx_sn ^= 1;
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
    }
    if (ll_ctx.ll_flag && (ack_mask & 0x02))
    {
        pp_fifo_pop(&ll_fifo);
        ll_ctx.ll_flag = false;
        ack_mask &= ~(0x1 << 1);
        
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
    }
    if (ll_ctx.last_valid_slot_id < 0)
        return; // last interval is all NULL packet

    ll_ctx.acked = (ack_mask >> ll_ctx.last_valid_slot_id) & 0x01;
    // tlkapi_printf(APP_LOG_EN, "last_valid_slot_id: %d, acked: %d\n", last_valid_slot_id, acked);
}


_attribute_ram_code_sec_ static void pp_rf_proc_tx(void)
{

    if (ll_ctx.step_3t1r == STEP_3T1R_1T)
    {
        // KB 1T done
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_2T)
    {
        // if (!(ll_ctx.report_rate & REPORT_RATE_8K))
        // {
        //     calc_hop_channels(ll_ctx.fno_last + 1, chn_mask);
        // }
        // KB 2T done
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_3T)
    {
        // KB 3T done
        //  DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
        // calc_hop_channels(ll_ctx.fno_last + 1, chn_mask);

        // enter KB 1T
        // ll_ctx.step_3t1r = STEP_3T1R_1R; //
    }

    RF_TX_GPIO_DEBUG_SET(0);
#if DEBUG_RF_REPORT_CNT_ENABLE
    rf_tx_irq_cnt++;
#endif
    {
        RF_RX_GPIO_DEBUG_SET(1);
    }

    if ((ll_ctx.stk_state == STATE_PAIRING || ll_ctx.stk_state == STATE_RECONNECT) && (ll_ctx.enter_conn_flag))
    {
        ll_ctx.enter_conn_flag = false;
        app_d24g_enter_conn();
    }

    if(ll_ctx.sleep_req)
    {
        ll_ctx.slp_tx_cnt++;
        if(ll_ctx.slp_tx_cnt > 16)
        {
            ll_ctx.slp_tx_cnt = 0;
            ll_ctx.sleep_req = false;
            // stack_evt_post_from_isr(TPSLL_EVT_SLEEP_ENTER_REQ, 0, 0);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_SLEEP_ENTER_REQ, 0, 0);
            tpsll_enter_idle(TPSLL_PM_OPTM_LEVEL4);
        }
    }
}

_attribute_ram_code_sec_ static void pp_proc_pairing_rx(rf_packet_t *pr)
{
    #if TPSLL_NEW_PAIR_EN
/////////////////////////////////////////

    if (ll_ctx.pair_st == STATE_PAIR_INIT)
    {
        pair_rsp_packet_t * pkt = (pair_rsp_packet_t *)(pr->buf);

        if ((pkt->cmd == CMD_PAIR_RSP) && (pkt->km_tag == ll_inf.tag)) {
            app_update_pairing_stimer_anchor(pkt->diff_us);
            pp_timer1_stop();
            timer_clr_irq_status(FLD_TMR1_MODE_IRQ); // clear irq status

            tmemcpy(ll_inf.peer_mac, pkt->mac, MAC_ADDR_LEN);
            ll_inf.peer_tag = tpsll_fnv1a_16(ll_inf.peer_mac, MAC_ADDR_LEN);
            ll_inf.side_id = tpsll_fnv1a_hash(ll_inf.peer_mac, MAC_ADDR_LEN);

            // tlkapi_printf(APP_LOG_EN, "diff us:%d\n", pkt->diff_us);

            ll_ctx.pair_cnt = 0;
            ll_ctx.pair_st = STATE_PAIR_RSP;

            // tlkapi_printf(APP_LOG_EN, "pairing rsp: pr mac(%02X%02X%02X%02X%02X%02X)tag(0x%X)\n", ll_inf.peer_mac[0], ll_inf.peer_mac[1], ll_inf.peer_mac[2],
            //     ll_inf.peer_mac[3], ll_inf.peer_mac[4], ll_inf.peer_mac[5], ll_inf.peer_tag);
            tlkapi_send_string_data(APP_LOG_EN, "pairing rsp", (unsigned char *)pkt, pr->rf_len);
        } else {
            tlkapi_send_string_data(APP_LOG_EN, "pairing rsp err", (unsigned char *)pkt, pr->rf_len);
        }
    } else if (ll_ctx.pair_st == STATE_PAIR_RSP) {
        conn_rsp_packet_t * pkt = (conn_rsp_packet_t *)(pr->buf);

        if ((pkt->cmd == CMD_CONN_RSP) && (pkt->km_tag == ll_inf.tag) && (pkt->d_tag == ll_inf.peer_tag)) {
            pp_timer1_stop();
            timer_clr_irq_status(FLD_TMR1_MODE_IRQ); // clear irq status

            ll_ctx.fno_last = pkt->fno_last;
            ll_ctx.fno      = ll_ctx.fno_last;

            if (ll_ctx.user_rr == APP_DEFAULT_REPORT_RATE) {
                // stack_evt_post_from_isr(TPSLL_EVT_REPORT_RATE_CHANGED, (void *)&ll_ctx.report_rate, 1);
                s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_REPORT_RATE_CHANGED,  (void *)&ll_ctx.report_rate, 1);
            } else {
                ll_ctx.pending_rr_fno = ll_ctx.fno_last + TPSLL_RR_DELAY_SWITCH_FNO_N;
                ll_ctx.chg_rr = ll_ctx.user_rr;
                ll_ctx.is_rr_changing = true;
            }

            app_d24g_enter_conn();

            #if TPSLL_RECON_RSSI_SCAN_EN
            chn_table[0] = pkt->chn_tab[0];
            chn_table[1] = pkt->chn_tab[1];
            chn_table[2] = pkt->chn_tab[2];
            #if TPSLL_N8K_CHN_EXPAND_EN
            tpsll_n8k_chn_expand();
            #else
            #if (CHN_TABL_LEN == 4)
            chn_table[3] = chn_table[1];
            #endif
            #endif
            #endif

            ll_inf.pair_success_flag = 1;
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SAVE_PAIR_INFO, 0, (void *)ll_inf.peer_mac, MAC_ADDR_LEN);

            tlkapi_send_string_data(APP_LOG_EN, "conn rsp", (unsigned char *)pkt, pr->rf_len);
        } else {
            tlkapi_send_string_data(APP_LOG_EN, "conn rsp err", (unsigned char *)pkt, pr->rf_len);
        }
    }
/////////////////////////////////////////
    #else /* end of TPSLL_NEW_PAIR_EN */
    if (ll_ctx.pair_st == STATE_PAIR_INIT && pr->buf[0] == CMD_PAIR_REQ)
    {
        // app_halt_system(0x1222, "ll_ctx.pair_st:%d cmd:%x\n", ll_ctx.pair_st, pr->buf[0]);

        if (tpsll_verify_pair_encrypt_key(pr) != PAIR_ENCRYPT_OK) {
            tlkapi_send_string_data(APP_LOG_EN, "pairing_req verify encrypt key fail!", (unsigned char *)pr, pr->dma_len);
            return;
        }

        app_update_pairing_stimer_anchor(0);
        app_rf_pkt_send();

        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);

        ll_ctx.pair_cnt = 0;
        ll_ctx.pair_st = STATE_PAIR_RSP;
        tlkapi_send_string_data(APP_LOG_EN, "pairing_req", (unsigned char *)pr, pr->dma_len);
    }
    else if (ll_ctx.pair_st == STATE_PAIR_RSP && pr->buf[0] == CMD_CONN_REQ)
    {
        conn_req_packet_t *prc = (conn_req_packet_t *)pr;

        if (prc->tag == ll_inf.tag)
        {
            app_update_pairing_stimer_anchor(0);

            tmemcpy(ll_inf.peer_mac, prc->mac, MAC_ADDR_LEN);
            DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
            ll_inf.peer_tag = tpsll_fnv1a_16(ll_inf.peer_mac, MAC_ADDR_LEN);
            app_rf_pkt_send();

            ll_inf.side_id = tpsll_fnv1a_hash(ll_inf.peer_mac, MAC_ADDR_LEN);
            // chn_table =;
            ll_ctx.fno_last = prc->fno_last;
            ll_ctx.fno = ll_ctx.fno_last + 1;

            #if RSSI_SCAN_EN
            tmemcpy(chn_table, prc->chn_table, CHN_TABL_LEN);
            #endif

            #if 1
            ll_ctx.stick += SKIP_INTV_FOR_WRITE_FLASH * SYSTEM_TIMER_TICK_1US * DONGLE_INTERVAL_US;
            systimer_set_irq_capture(ll_ctx.stick);
            #endif
            ll_inf.pair_success_flag = 1;
            ll_ctx.enter_conn_flag = true;
            tlkapi_send_string_data(APP_LOG_EN, "conn_req", (unsigned char *)pr, pr->dma_len);
        }else{
            tlkapi_send_string_data(APP_LOG_EN, "conn_req err", (unsigned char *)pr, pr->dma_len);
            // app_halt_system(0x1227, "ll_ctx.pair_st:%d cmd:%x tag:%x peer_tag:%x\n", ll_ctx.pair_st, pr->buf[0], ll_inf.tag, prc->tag);

        }
    }else{
        // app_halt_system(0x2222, "ll_ctx.pair_st:%d cmd:%x %x\n", ll_ctx.pair_st, pr->buf[0], pr->buf[1]);
    }
    #endif /* end of TPSLL_NEW_PAIR_EN #else */
}

_attribute_ram_code_sec_ static void pp_proc_reconn_rx(rf_packet_t *p)
{
    #if TPSLL_NEW_RECON_EN
/////////////////////////////////////////
    reconn_rsp_packet_t *pr = (reconn_rsp_packet_t *)p;

    // if (pr->cmd == CMD_RECONN_REQ && pr->d_tag == ll_inf.tag) { 
    if (pr->cmd == CMD_RECONN_RSP && pr->d_tag == ll_inf.peer_tag) {
        uint16_t calc_crc = p24g_crc16(0xFFFF, (uint8_t *)&pr->cmd, offsetof(reconn_rsp_packet_t, crc16) - offsetof(reconn_rsp_packet_t, cmd));
        if (calc_crc != pr->crc16) {
            tlkapi_printf(APP_LOG_EN, "reconn_rsp crc err, calc_crc:0x%04x, rcvd_crc:0x%04x\n", calc_crc, pr->crc16);
            return;
        }
    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
        app_update_pairing_stimer_anchor(pr->diff_us);
        ll_ctx.recon_step = RECON_STEP_NONE;
        pp_timer1_stop();
        timer_clr_irq_status(FLD_TMR1_MODE_IRQ); // clear irq status
        ll_ctx.fno_last = pr->fno_last;
        ll_ctx.fno      = ll_ctx.fno_last;

        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP4) {
            ll_ctx.rr_chg_with_recon = RR_RECON_STEP5;
        }
        #endif
        if (ll_ctx.user_rr == APP_DEFAULT_REPORT_RATE) {
            // stack_evt_post_from_isr(TPSLL_EVT_REPORT_RATE_CHANGED, (void *)&ll_ctx.report_rate, 1);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_REPORT_RATE_CHANGED,  (void *)&ll_ctx.report_rate, 1);
        } else {
            ll_ctx.pending_rr_fno = ll_ctx.fno_last + TPSLL_RR_DELAY_SWITCH_FNO_N;
            ll_ctx.chg_rr = ll_ctx.user_rr;
            ll_ctx.is_rr_changing = true;
        }

        app_d24g_enter_conn();

        #if TPSLL_RECON_RSSI_SCAN_EN
        {
            chn_table[0] = pr->chn_tab[0];
            chn_table[1] = pr->chn_tab[1];
            chn_table[2] = pr->chn_tab[2];
            #if TPSLL_N8K_CHN_EXPAND_EN
            tpsll_n8k_chn_expand();
            #else
            #if (CHN_TABL_LEN == 4)
            chn_table[3] = chn_table[1];
            #endif
            #endif
        }
        #endif

        // tlkapi_printf(APP_LOG_EN, "fno(0x%X)chn(%d-%d-%d)\n", pr->fno_last, chn_table[0], chn_table[1], chn_table[2]);

    } else {
        tlkapi_send_string_data(APP_LOG_EN, "reconn_req err", (unsigned char *)pr, pr->dma_len);
        tlkapi_printf(APP_LOG_EN, "tag:%x peer_tag:%x\n", ll_inf.tag, pr->d_tag);
    }
/////////////////////////////////////////
    #else /* end of TPSLL_NEW_RECON_EN */
    reconn_req_packet_t *pr = (reconn_req_packet_t *)p;

    if (pr->cmd == CMD_RECONN_REQ && pr->tag == ll_inf.tag) { 
        ll_ctx.fno_last = pr->fno_last;
        ll_ctx.fno      = ll_ctx.fno_last + 1;

        app_update_pairing_stimer_anchor(0);
        app_rf_pkt_send();

        #if TPSLL_RECON_RSSI_SCAN_EN
        // #if TPSLL_RR_CHG_WITH_RECON_EN
        // if (ll_ctx.rr_chg_with_recon) {
        //     #if RSSI_SCAN_EN
        //     tmemcpy(chn_table, pr->chn_table, CHN_TABL_LEN);
        //     // tmemcpy(hop_ctrl.chn_table, pr->chn_table, CHN_TABL_LEN);
        //     #endif
        // } else
        // #endif
        {
            chn_table[0] = reconn_chn_table[0];
            chn_table[1] = reconn_chn_table[1];
            chn_table[2] = reconn_chn_table[2];
        }
        #else
        #if RSSI_SCAN_EN
        tmemcpy(chn_table, pr->chn_table, CHN_TABL_LEN);
        // tmemcpy(hop_ctrl.chn_table, pr->chn_table, CHN_TABL_LEN);
        #endif
        #endif
        ll_ctx.enter_conn_flag = true;
        ll_ctx.recon_step = RECON_STEP2;

        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (!ll_ctx.rr_chg_with_recon)
        #endif
        tlkapi_send_string_data(APP_LOG_EN, "reconn_req", (unsigned char *)pr, pr->dma_len);

        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP4) {
            ll_ctx.rr_chg_with_recon = RR_RECON_STEP5;
        }
        #endif
    } else {
        tlkapi_send_string_data(APP_LOG_EN, "reconn_req err", (unsigned char *)pr, pr->dma_len);
        tlkapi_printf(APP_LOG_EN, "tag:%x peer_tag:%x\n", ll_inf.tag, pr->tag);
    }
    #endif /* end of TPSLL_NEW_RECON_EN #else */
}

_attribute_ram_code_sec_ static void tpsll_proc_spp_rx(uint8_t *rbuf, uint8_t rlen)
{
#if TPSLL_SPP_FRAG_EN
    if (rlen < 1) {
        tpsll_spp_rx_assemble_reset();
        return;
    }

    uint8_t hdr = rbuf[0];
    uint8_t frag_type = hdr & SPP_FRAG_TYPE_MASK;
    uint8_t opcode = (hdr >> SPP_OPCODE_SHIFT) & 0x3F;
    uint8_t payload_len = rlen - 1;

    // Case 1: No fragmentation
    if (frag_type == SPP_FRAG_TYPE_NO_FRAG)
    {
        tpsll_spp_rx_assemble_reset();
        // tlkapi_send_string_data(APP_LOG_EN, "spp rcv(no frag)", (void *)&rbuf[1], payload_len);
        // stack_evt_post_from_isr(TPSLL_EVT_SPP_DATA_RECV, (void *)&rbuf[1], payload_len);
        s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_DATA_TYPE_SPP, rbuf[1], &rbuf[2], payload_len - 1);
        return;
    }

    // Case 2: Middle fragment (not last packet)
    if (frag_type == SPP_FRAG_TYPE_MID)
    {
        tpsll_spp_rx_assemble_reset();

        g_spp_rx_assemble.is_receiving = true;
        g_spp_rx_assemble.opcode = opcode;

        if ((g_spp_rx_assemble.cur_len + payload_len) >= SPP_MAX_PACKET_LEN) {
            return;
        }
        // Copy payload (skip header)
        
        tmemcpy(&g_spp_rx_assemble.buf[g_spp_rx_assemble.cur_len], &rbuf[1], payload_len);
        g_spp_rx_assemble.cur_len += payload_len;
    }
    else if (frag_type == SPP_FRAG_TYPE_END) // Case 3: End fragment (last packet)
    {
        // Invalid state: no start fragment received
        if (!g_spp_rx_assemble.is_receiving) {
            tpsll_spp_rx_assemble_reset();
            return;
        }

        if ((g_spp_rx_assemble.cur_len + payload_len) >= SPP_MAX_PACKET_LEN) {
            tpsll_spp_rx_assemble_reset();
            return;
        }

        // Copy tail payload
        tmemcpy(&g_spp_rx_assemble.buf[g_spp_rx_assemble.cur_len], &rbuf[1], payload_len);
        g_spp_rx_assemble.cur_len += payload_len;

        // tlkapi_send_string_data(APP_LOG_EN, "spp recv success", g_spp_rx_assemble.buf, g_spp_rx_assemble.cur_len);
        // stack_evt_post_from_isr(TPSLL_EVT_SPP_DATA_RECV, (void *)g_spp_rx_assemble.buf, g_spp_rx_assemble.cur_len);
        s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_DATA_TYPE_SPP, g_spp_rx_assemble.buf[0], &g_spp_rx_assemble.buf[1], g_spp_rx_assemble.cur_len - 1);

        tpsll_spp_rx_assemble_reset();
    }
    else {
        tpsll_spp_rx_assemble_reset();
    }

#else

    // stack_evt_post_from_isr(TPSLL_EVT_SPP_DATA_RECV, rbuf, rlen);
    s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_DATA_TYPE_SPP, rbuf[0], &rbuf[1], rlen - 1);

#endif
}



_attribute_ram_code_sec_ static void p24g_proc_conn_rx_8k(rf_packet_t *pr)
{
    // if (ll_ctx.step_3t1r == STEP_3T1R_1R)
    {
            // app_update_stimer_anchor();
            app_update_conn_stimer_anchor(pr->buf[0] & 0x7);

            if ((pr->buf[0] & 0x7) && tpsll_is_keep_alive_active() && ll_ctx.synced && tpsll_all_fifo_is_empty()) {
                #if TPSLL_PM_OPTM_EN
                ll_ctx.suspend_flag = true;
                #endif /* end of TPSLL_PM_OPTM_EN */
            }

            if(pr->rf_len > 2) {
                #if TPSLL_FHSS_EN
                if (pr->n_tab) {
                    g_hop_chn_idx_tab_next[0] = pr->buf[1] & 0x1f;
                    g_hop_chn_idx_tab_next[1] = (pr->buf[1] >> 5) | ((pr->buf[2] & 0x3) << 3);
                    g_hop_chn_idx_tab_next[2] = pr->buf[2] >> 2;
                    g_fhss_idx_buf_data_next =  pr->buf[1] | (pr->buf[2] << 8);
                } 
                #endif
#if !TPSLL_SPP_NOT_SWITCH_EN
                if ((pr->snnesn & SN_MASK) != ll_ctx.sn_8k) {
                    //new packet
                    ll_ctx.sn_8k = pr->snnesn & SN_MASK;
                    SET_NESN(rf_tx_packet.snnesn, ll_ctx.sn_8k);

                    #if TPSLL_RR_CHG_WITH_RECON_EN
                    if (pr->pkt_type == P24G_PKT_TYPE_LL) {
                        if (pr->buf[2] == TPSLL_LL_CMD_RR_CHG_RCNN_RSP) {
                            if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP1) {
                                ll_ctx.rr_chg_with_recon = RR_RECON_STEP2;
                            }
                        }
                    } else
                    #endif
                    {
                        // stack_evt_post_from_isr(TPSLL_EVT_USR_SPEC_DATA_RECV, (void *)&pr->buf[2], pr->rf_len - 2);
                        s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_USR_SPEC_DATA_RECV, &pr->buf[2], pr->rf_len - 2);
                    }
                }
#endif
            }
#if !TPSLL_SPP_NOT_SWITCH_EN
            if (GET_NESN(pr->snnesn) != ll_ctx.s_8k) { //acked
                ll_ctx.s_8k ^= 1;
                SET_SN(rf_tx_packet.snnesn, ll_ctx.s_8k);
                ll_ctx.acked_ak = true;
            }
            else {
                ll_ctx.acked_ak = false;
            }
#endif


            handle_ack(pr->buf[0] & 0x7);
            ll_ctx.peer_spp_flag = pr->pkt_type;
    }
//////////////////////////////////////////////////////////////////////////////
    #if TPSLL_SPP_NOT_SWITCH_EN
    uint8_t sn = GET_SN(pr->snnesn);
    uint8_t nesn = GET_NESN(pr->snnesn);

// DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX4], ll_ctx.rx_nesn);
// DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
// DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX5], sn);
    // if (sn != ll_ctx.rx_nesn)
    // { // old packet
    //     return;
    // }

    // ll_ctx.rx_nesn ^= 1;
    // SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);
// DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
// DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX4], ll_ctx.rx_nesn);

    if (nesn != ll_ctx.tx_sn) // acked
    {
        ll_ctx.tx_sn ^= 1;
        SET_SN(rf_tx_packet.snnesn, ll_ctx.tx_sn);

        if (ll_ctx.ak_flag)
        {
            uint8_t *ptr = pp_fifo_get_ptr(km_fifo_ptr);
            ll_ctx.ak_flag = false;
            if (ptr) {
                pp_fifo_pop(km_fifo_ptr);
            }
        }
        if (ll_ctx.spp_flag) // spp mode only one slot
        {
            uint8_t *ptr = pp_fifo_get_ptr(spp_fifo_ptr);
            // uint8_t len = ptr[0];

            ll_ctx.spp_flag = false;
            if (ptr) {
                pp_fifo_pop(spp_fifo_ptr);
                // stack_evt_post_from_isr(TPSLL_INTER_EVT_SPP_SEND_COMP, (void *)&ptr[2], ptr[0] + 9);
                s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SPP_SEND_COMP, 0, &ptr[2], ptr[0] + 9);
            }

            ll_ctx.spp_quota_active = 1;
        }
        if (ll_ctx.ll_flag)
        {
            uint8_t *ptr = pp_fifo_get_ptr(&ll_fifo);

            if (ptr) {
                pp_fifo_pop(&ll_fifo);
            }

            ll_ctx.ll_flag = false;

            #if TPSLL_RR_CHG_WITH_RECON_EN
            if (pr->pkt_type == P24G_PKT_TYPE_LL)
            {
                if (pr->buf[2] == TPSLL_LL_CMD_RR_CHG_RCNN_RSP)
                {
                    if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP1)
                    {
                        ll_ctx.rr_chg_with_recon = RR_RECON_STEP2;
                    }
                }
            }
            else
            #endif
            {
                // stack_evt_post_from_isr(TPSLL_EVT_USR_SPEC_DATA_RECV, (void *)&p->buf[3], p->rf_len - 3);
            }
        }
    }

    if ((pr->pkt_type < P24G_PKT_TYPE_SPP) || (sn != ll_ctx.rx_nesn))
    { // invalid packet
        ll_ctx.rx_over = true;
        return;
    }
    ll_ctx.rx_nesn ^= 1;
    SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);

    ll_ctx.rx_over = true;

    if (pr->pkt_type == P24G_PKT_TYPE_SPP) {
        // stack_evt_post_from_isr(TPSLL_EVT_SPP_DATA_RECV, (void *)&pr->buf[3], pr->rf_len - 3);
        tpsll_proc_spp_rx(&pr->buf[3], pr->rf_len - 3);
    }
    else if (pr->pkt_type == P24G_PKT_TYPE_LL) {
        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (pr->buf[3] == TPSLL_LL_CMD_RR_CHG_RCNN_RSP) {
            if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP1) {
                ll_ctx.rr_chg_with_recon = RR_RECON_STEP2;
            }
        }
        #endif
        #if TPSLL_RR_CHG_FROM_DONGLE
        else if(pr->buf[3] == TPSLL_LL_CMD_RR_CHG_REQ)
        {
            // stack_evt_post_from_isr(TPSLL_EVT_REPORT_RATE_CHANGE_REQ, (void *)&pr->buf[4], 1);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_REPORT_RATE_CHANGE_REQ, (void *)&pr->buf[4], 1);
        }
        #endif
        if(pr->buf[3] == TPSLL_LL_CMD_UL_DATA_SEND_REQ)
        {
            // stack_evt_post_from_isr(TPSLL_EVT_USR_SPEC_DATA_RECV, (void *)&pr->buf[4], pr->rf_len - 4);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_USR_SPEC_DATA_RECV, (void *)&pr->buf[4], pr->rf_len - 4);
        }
    }
    #endif
//////////////////////////////////////////////////////////////////////////////
}

_attribute_ram_code_sec_ static void p24g_proc_conn_rx_non_8k(rf_packet_t *p)
{
    uint8_t sn = GET_SN(p->snnesn);
    uint8_t nesn = GET_NESN(p->snnesn);

    {
        if (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250))
        {
            //  tlkapi_send_string_data(APP_LOG_EN, "125_rx", &p->dma_len, p->rf_len + 6);
            app_update_conn_stimer_anchor_125(p->buf[1]);
        }
        #if TPSLL_NEW_SYNC_EN
        else if (ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500))
        {
            tpsll_update_conn_stimer_anchor_new(p->buf[0] | ((uint16_t)(p->buf[1]) << 8));
            // if ((ll_ctx.srx_task_slot == TASK_SLOT0) && (rf_tx_packet.pkt_type < P24G_PKT_TYPE_SPP))
            {
                if (tpsll_is_keep_alive_active() && ll_ctx.synced && tpsll_all_fifo_is_empty()) {
                    #if TPSLL_PM_OPTM_EN
                    ll_ctx.suspend_flag = true;
                    #endif /* end of TPSLL_PM_OPTM_EN */
                }
            }
        }
        #endif
        else
        {
            if ((ll_ctx.srx_task_slot == TASK_SLOT0) && (rf_tx_packet.pkt_type < P24G_PKT_TYPE_SPP))
            {
                app_update_conn_stimer_anchor(nesn != ll_ctx.tx_sn);
                if((nesn != ll_ctx.tx_sn) && ll_ctx.report_rate & (REPORT_RATE_4K | REPORT_RATE_2K))
                {
                    if (tpsll_is_keep_alive_active() && ll_ctx.synced && tpsll_all_fifo_is_empty()) {
                        #if TPSLL_PM_OPTM_EN
                        ll_ctx.suspend_flag = true;
                        #endif /* end of TPSLL_PM_OPTM_EN */
                    }
                }
            }
        }
    }

    ll_ctx.peer_spp_flag = p->pkt_type;
    
    #if TPSLL_N8K_FHSS_EN
    if(p->n_tab)
    {
        if (ll_ctx.report_rate & (REPORT_RATE_4K | REPORT_RATE_2K))
        {
            g_hop_chn_idx_tab_next[0] = p->buf[0] & 0x1f;
            g_hop_chn_idx_tab_next[1] = (p->buf[0] >> 5) | ((p->buf[1] & 0x3) << 3);
            g_hop_chn_idx_tab_next[2] = p->buf[1] >> 2;
            g_fhss_idx_buf_data_next = p->buf[0] | (p->buf[1] << 8);
        }
        else
        {
            g_hop_chn_idx_tab_next[0] = p->buf[2] & 0x1f;
            g_hop_chn_idx_tab_next[1] = (p->buf[2] >> 5) | ((p->buf[3] & 0x3) << 3);
            g_hop_chn_idx_tab_next[2] = p->buf[3] >> 2;
            g_fhss_idx_buf_data_next = p->buf[2] | (p->buf[3] << 8);
        }
    }
    #endif

#if 0
    if (sn)
    {
        ZW_TG(GPIO_PD6);
    }
    else
    {
        ZW_TG(GPIO_PD6);
        ZW_TG(GPIO_PD6);
    }

    if (nesn)
    {
        ZW_TG(GPIO_PD7);
    }
    else
    {
        ZW_TG(GPIO_PD7);
        ZW_TG(GPIO_PD7);
    }
    #endif

#if 0
    static uint32_t sncnt = 0;
    if (sncnt < 2)
    {
        sncnt++;
        tlkapi_send_string_data(APP_LOG_EN, "1k_rx", &p->dma_len, p->rf_len + 6);
        //                    DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
    }

#endif

    if (sn != ll_ctx.rx_nesn)
    { // old packet
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
        ll_ctx.rx_over = true;
        return;
    }

    ll_ctx.rx_nesn ^= 1;
    SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);

    // ZW_TG(GPIO_PG7);

    if (nesn != ll_ctx.tx_sn) // acked
    {
        ll_ctx.tx_sn ^= 1;
        SET_SN(rf_tx_packet.snnesn, ll_ctx.tx_sn);
        // ZW_TG(GPIO_PG7);
        if (pp_fifo_get_ptr(km_fifo_ptr) && (ll_ctx.ms_flag || ll_ctx.kb_flag))
        {
            pp_fifo_pop(km_fifo_ptr);
            ll_ctx.ms_flag = false;
            ll_ctx.kb_flag = false;
        }

        if (ll_ctx.ll_flag)
        {
            uint8_t *p_ll = pp_fifo_get_ptr(&ll_fifo);
            if (p_ll != 0)
            {
                pp_fifo_pop(&ll_fifo);
                ll_ctx.ll_flag = false;
                if(p_ll[2] == TPSLL_LL_CMD_RR_CHG_REQ)
                {
                    ll_ctx.is_rr_chg_acked = true;
                }
            }
        }
        // if (!pp_fifo_get_num(km_fifo_ptr) && !pp_fifo_get_num(&ll_fifo) && !pp_fifo_get_num(spp_fifo_ptr))
        if ((!pp_fifo_get_num(km_fifo_ptr) && !pp_fifo_get_num(&ll_fifo)) ||
            ((ll_ctx.report_rate & REPORT_RATE_125) && (ll_ctx.srx_task_slot >= (ll_ctx.max_slot - 1))))
        {
            if (ll_ctx.srx_task_slot < (ll_ctx.max_slot - 1)) {
                pp_timer1_stop();
            }

            if (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250))
            {
                ll_ctx.suspend_flag = true;
            }
        }

        if (pp_fifo_get_ptr(spp_fifo_ptr) && ll_ctx.spp_flag) // spp mode only one slot
        {

            uint8_t *ptr = pp_fifo_get_ptr(spp_fifo_ptr);
            // uint8_t len = ptr[0];
            // unsigned char *payload = &ptr[2];
            // void *user_arg = *(void **)(&ptr[len + 6]);
            // fifo_cb_t spp_cb = *(fifo_cb_t *)(&ptr[len + 2]);

            // if (spp_cb)
            // {
            //     spp_cb(payload, len, true, user_arg);
            // }
            // stack_evt_post_from_isr(TPSLL_INTER_EVT_SPP_SEND_COMP, (void *)&ptr[2], len+9);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SPP_SEND_COMP, 0, &ptr[2], ptr[0] + 9);

            pp_fifo_pop(spp_fifo_ptr);
            ll_ctx.spp_flag = false;
        }

        if(p->pkt_type == P24G_PKT_TYPE_SYNC)
        {
            #if TPSLL_N8K_FHSS_EN
            uint8_t offset;
            if (ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500 | REPORT_RATE_250 | REPORT_RATE_125))
            {
                offset = PL_APP_FULL_OFFSET_1;
            }else{
                offset = PL_APP_FULL_OFFSET_2;
            }

            if(p->rf_len > offset)
            {
                // stack_evt_post_from_isr(TPSLL_EVT_USR_SPEC_DATA_RECV, (void *)&p->buf[offset], p->rf_len - offset);
                s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_USR_SPEC_DATA_RECV, (void *)&p->buf[offset], p->rf_len - offset);
            }
            #else
            if(p->rf_len > 2)
            {
                // stack_evt_post_from_isr(TPSLL_EVT_USR_SPEC_DATA_RECV, (void *)&p->buf[2], p->rf_len-2);
                s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_USR_SPEC_DATA_RECV, (void *)&p->buf[2], p->rf_len-2);
            }
            #endif
        }

        ll_ctx.is_spp_pre_acked = true;
        ll_ctx.spp_sw_thres++;
    }

    ll_ctx.rx_over = true;

    if(p->pkt_type == P24G_PKT_TYPE_SPP)
    {
        #if TPSLL_NEW_SYNC_EN
        if (ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500 | REPORT_RATE_250 | REPORT_RATE_125))
        #else
        if (ll_ctx.report_rate & (REPORT_RATE_250 | REPORT_RATE_125))
        #endif
        {
            #if TPSLL_N8K_FHSS_EN
            tpsll_proc_spp_rx(&p->buf[PL_APP_FULL_OFFSET_1], p->rf_len - PL_APP_FULL_OFFSET_1);
            #else
            tpsll_proc_spp_rx(&p->buf[2], p->rf_len - 2);
            #endif
        }
        else {
            #if TPSLL_N8K_FHSS_EN
            tpsll_proc_spp_rx(&p->buf[PL_APP_FULL_OFFSET_2], p->rf_len - PL_APP_FULL_OFFSET_2);
            #else
            tpsll_proc_spp_rx(p->buf, p->rf_len);
            #endif
        }
    }

    if (g_spp_rx_assemble.is_receiving && (ll_ctx.task_slot < (ll_ctx.max_slot - 1)))
    {
        timer_start(TIMER1);
        ll_ctx.suspend_flag = false;
    }

    #if TPSLL_RR_CHG_WITH_RECON_EN
    if (p->pkt_type == P24G_PKT_TYPE_LL) {
        if (p->buf[2] == TPSLL_LL_CMD_RR_CHG_RCNN_RSP) {
            if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP1) {
                ll_ctx.rr_chg_with_recon = RR_RECON_STEP2;
            }
        }
        else if(p->buf[2] == TPSLL_LL_CMD_UL_DATA_SEND_REQ)
        {
            // stack_evt_post_from_isr(TPSLL_EVT_USR_SPEC_DATA_RECV, (void *)&p->buf[3], p->rf_len - 3);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_USR_SPEC_DATA_RECV, (void *)&p->buf[3], p->rf_len - 3);
        }
    }
    #endif
}

_attribute_ram_code_sec_ static void p24g_proc_conn_rx(rf_packet_t *pr)
{

    if(ll_ctx.report_rate & REPORT_RATE_8K)
    {
        p24g_proc_conn_rx_8k(pr);
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        if (pr->rf_len > 3) {
            ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
            ll_ctx.kb_alive_intervals_cnt = 0;
            tpsll_event_active_time_fresh_value(clock_time());
        }
        #endif
    }else{
        p24g_proc_conn_rx_non_8k(pr);
        if ((ll_ctx.report_rate & REPORT_RATE_4K) && tick1_exceed_tick2(ll_ctx.last_stick + 70 * RF_SYSTEM_TIMER_TICK_1US, clock_time())) {
            if (ll_ctx.rx_over) {
                ll_ctx.rx_over = false;
                ll_ctx.srx_task_slot = ll_ctx.task_slot;
                #if TPSLL_N8K_FHSS_EN
                #if TPSLL_N8K_CHN_EXPAND_EN
                uint8_t ch;
                if(ll_ctx.task_slot) {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 7];
                } else {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                }
                #else
                uint8_t ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                #endif
                #else
                uint8_t ch = g_n8k_chn_tab[(ll_ctx.fno_last + ll_ctx.task_slot) & (NON_8K_CHN_NUM - 1)];
                #endif
                rf_set_ble_chn(ch);
                pp_rf_start_srx(ll_ctx.last_stick + 73 * RF_SYSTEM_TIMER_TICK_1US, KB_CONNECTED_KB_RX_TIMEOUT + 10);
            }
        }
        else if ((ll_ctx.report_rate & REPORT_RATE_2K) && tick1_exceed_tick2(ll_ctx.last_stick + 100 * RF_SYSTEM_TIMER_TICK_1US, clock_time())) {
            if (ll_ctx.rx_over) {
                ll_ctx.rx_over = false;
                ll_ctx.srx_task_slot = ll_ctx.task_slot;
                #if TPSLL_N8K_FHSS_EN
                #if TPSLL_N8K_CHN_EXPAND_EN
                uint8_t ch;
                if(ll_ctx.task_slot) {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 7];
                } else {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                }
                #else
                uint8_t ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                #endif
                #else
                uint8_t ch = g_n8k_chn_tab[(ll_ctx.fno_last + ll_ctx.task_slot) & (NON_8K_CHN_NUM - 1)];
                #endif
                rf_set_ble_chn(ch);
                pp_rf_start_srx(ll_ctx.last_stick + 110 * RF_SYSTEM_TIMER_TICK_1US, KB_CONNECTED_KB_RX_TIMEOUT + 17);
            }
        }
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        if (pr->rf_len > LL_PKT_OFFSET) {
            ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
            ll_ctx.kb_alive_intervals_cnt = 0;
            tpsll_event_active_time_fresh_value(clock_time());
        }
        #endif
        else if ((ll_ctx.report_rate & REPORT_RATE_250) && tpsll_is_active_state() && (ll_ctx.srx_task_slot >= 9) && ll_ctx.suspend_flag && ll_ctx.is_alg_kb) {
            pp_timer1_start(ll_ctx.stick - stimer_get_tick() - TPSLL_PM_OPTM_250_NEED_ENABLE_KS_EARLY_TIME_US * RF_SYSTEM_TIMER_TICK_1US);
            ll_ctx.ks_need_enable = true;
            ll_ctx.suspend_flag = false;
        }
    }
}

_attribute_ram_code_sec_ static void pp_rf_proc_rx(void)
{
    RF_RX_GPIO_DEBUG_SET(0);
    unsigned char *raw_pkt = rf_get_rx_packet_addr(RX_FIFO_NUM, RX_FIFO_DEP, rx_packet);

//    tlk_printf ("kb init\r\n");
    rf_packet_t *p = (rf_packet_t *)raw_pkt;

    DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX1]);

    if (!(reg_rf_dec_err & 0x10) && ((uint32_t)(p->dma_len) == (uint32_t)RF_GET_DMA_LEN(p->rf_len)))
    {
        if(ll_ctx.stk_state == STATE_CONNECTED)
        {
            p24g_proc_conn_rx(p);
            ll_ctx.connected_active_tick = stimer_get_tick() | 1;
        }else if(ll_ctx.stk_state == STATE_RECONNECT)
        {
            pp_proc_reconn_rx(p);
        }
        else if(ll_ctx.stk_state == STATE_PAIRING)
        {
            pp_proc_pairing_rx(p);
        }

        if ((ll_ctx.stk_state == STATE_CONNECTED) && ll_ctx.lost_cnt)
        {
            ll_ctx.lost_cnt = 0;
        }

        #if TPSLL_CONNECTING_CHECK_EN
        ll_ctx.connecting_check = 0;
        #endif

        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX1]);
    }
    else
    {
        // ll_ctx.rx_over = true;
        if (ll_ctx.report_rate & ~(REPORT_RATE_250 | REPORT_RATE_125)) {
            ll_ctx.rx_over = true;
        }

        if ((ll_ctx.stk_state == STATE_CONNECTED) && !ll_ctx.lost_cnt)
        {
            ll_ctx.lost_cnt = 1;

            if ((ll_ctx.report_rate & REPORT_RATE_4K) && tick1_exceed_tick2(ll_ctx.last_stick + 70 * RF_SYSTEM_TIMER_TICK_1US, clock_time())) {
                if (ll_ctx.rx_over) {
                    ll_ctx.rx_over = false;
                    ll_ctx.srx_task_slot = ll_ctx.task_slot;
                    #if TPSLL_N8K_FHSS_EN
                    #if TPSLL_N8K_CHN_EXPAND_EN
                    uint8_t ch;
                    if(ll_ctx.task_slot) {
                        ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 7];
                    } else {
                        ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                    }
                    #else
                    uint8_t ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                    #endif
                    #else
                    uint8_t ch = g_n8k_chn_tab[(ll_ctx.fno_last + ll_ctx.task_slot) & (NON_8K_CHN_NUM - 1)];
                    #endif
                    rf_set_ble_chn(ch);
                    pp_rf_start_srx(ll_ctx.last_stick + 73 * RF_SYSTEM_TIMER_TICK_1US, KB_CONNECTED_KB_RX_TIMEOUT);
                }
            }
            else if ((ll_ctx.report_rate & REPORT_RATE_2K) && tick1_exceed_tick2(ll_ctx.last_stick + 100 * RF_SYSTEM_TIMER_TICK_1US, clock_time())) {
                if (ll_ctx.rx_over) {
                    ll_ctx.rx_over = false;
                    ll_ctx.srx_task_slot = ll_ctx.task_slot;
                    #if TPSLL_N8K_FHSS_EN
                    #if TPSLL_N8K_CHN_EXPAND_EN
                    uint8_t ch;
                    if(ll_ctx.task_slot) {
                        ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 7];
                    } else {
                        ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                    }
                    #else
                    uint8_t ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                    #endif
                    #else
                    uint8_t ch = g_n8k_chn_tab[(ll_ctx.fno_last + ll_ctx.task_slot) & (NON_8K_CHN_NUM - 1)];
                    #endif
                    rf_set_ble_chn(ch);
                    pp_rf_start_srx(ll_ctx.last_stick + 110 * RF_SYSTEM_TIMER_TICK_1US, KB_CONNECTED_KB_RX_TIMEOUT + 17);
                }
            }
        }

        if (ll_ctx.report_rate & (REPORT_RATE_250 | REPORT_RATE_125)) {
            if ((ll_ctx.report_rate & REPORT_RATE_125) && !ll_ctx.suspend_flag)
            {
                if (ll_ctx.srx_task_slot >= (ll_ctx.max_slot - 1))
                {
                    pp_timer1_stop();
                    ll_ctx.suspend_flag = true;
                }
            }
            ll_ctx.rx_over = true;
        }
        app_rf_abnormal_handle();

        if((ll_ctx.stk_state == STATE_PAIRING) || (ll_ctx.stk_state == STATE_RECONNECT))
        {
            ll_ctx.pr_cnt = 0;
        }
    }

    // calc_hop_channels(ll_ctx.fno_last + 1);
}

_attribute_ram_code_sec_ static void pp_rf_proc_fst_to(void)
{
    rf_status = IDLE_RF_STATUS;
    RF_RX_TIMEOUT_GPIO_DEBUG_TOGGLE;

    if (ll_ctx.step_3t1r == STEP_3T1R_1R)
    {
        // KB 1R timeout
    }

    if ((ll_ctx.stk_state == STATE_CONNECTED) && !ll_ctx.lost_cnt)
    {
        ll_ctx.lost_cnt = 1;
    }
}


_attribute_ram_code_sec_ static void pp_rf_proc_rx_to(void)
{
    rf_status=IDLE_RF_STATUS;

    RF_RX_TIMEOUT_GPIO_DEBUG_TOGGLE;

    app_rf_abnormal_handle();


    if ((ll_ctx.stk_state == STATE_CONNECTED) && !ll_ctx.lost_cnt) {
        ll_ctx.lost_cnt = 1;
    }

}

_attribute_ram_code_sec_ uint8_t tpsll_rf_irq_handler(void)
{
    //DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN,dbg_io[DBG_IO_IDX1], 1);

    if(rf_get_irq_status(FLD_RF_IRQ_RX))
    {
        rf_clr_irq_status(FLD_RF_IRQ_RX);
        pp_rf_proc_rx();
    }else if(rf_get_irq_status(FLD_RF_IRQ_TX))
    {
        rf_clr_irq_status(FLD_RF_IRQ_TX);
        pp_rf_proc_tx();
    }
    else if(rf_get_irq_status(FLD_RF_IRQ_FIRST_TIMEOUT))
    {
        pp_rf_proc_fst_to();
        // ll_ctx.rx_over = true;
        if (ll_ctx.report_rate & ~(REPORT_RATE_250 | REPORT_RATE_125)) {
            ll_ctx.rx_over = true;
        }
        if ((ll_ctx.report_rate & REPORT_RATE_4K) && tick1_exceed_tick2(ll_ctx.last_stick + 70 * RF_SYSTEM_TIMER_TICK_1US, clock_time())) {
            if (ll_ctx.rx_over) {
                ll_ctx.rx_over = false;
                ll_ctx.srx_task_slot = ll_ctx.task_slot;
                #if TPSLL_N8K_FHSS_EN
                #if TPSLL_N8K_CHN_EXPAND_EN
                uint8_t ch;
                if(ll_ctx.task_slot) {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 7];
                } else {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                }
                #else
                uint8_t ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                #endif
                #else
                uint8_t ch = g_n8k_chn_tab[(ll_ctx.fno_last + ll_ctx.task_slot) & (NON_8K_CHN_NUM - 1)];
                #endif
                rf_set_ble_chn(ch);
                pp_rf_start_srx(ll_ctx.last_stick + 73 * RF_SYSTEM_TIMER_TICK_1US, KB_CONNECTED_KB_RX_TIMEOUT + 10);
            }
        }
        else if ((ll_ctx.report_rate & REPORT_RATE_2K) && tick1_exceed_tick2(ll_ctx.last_stick + 100 * RF_SYSTEM_TIMER_TICK_1US, clock_time())) {
            if (ll_ctx.rx_over) {
                ll_ctx.rx_over = false;
                ll_ctx.srx_task_slot = ll_ctx.task_slot;
                #if TPSLL_N8K_FHSS_EN
                #if TPSLL_N8K_CHN_EXPAND_EN
                uint8_t ch;
                if(ll_ctx.task_slot) {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 7];
                } else {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                }
                #else
                uint8_t ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                #endif
                #else
                uint8_t ch = g_n8k_chn_tab[(ll_ctx.fno_last + ll_ctx.task_slot) & (NON_8K_CHN_NUM - 1)];
                #endif
                rf_set_ble_chn(ch);
                pp_rf_start_srx(ll_ctx.last_stick + 110 * RF_SYSTEM_TIMER_TICK_1US, KB_CONNECTED_KB_RX_TIMEOUT + 17);
            }
        }
        #if TPSLL_RECON_RSSI_SCAN_EN
        if(scan_state == STATE_SCAN_RUNNING) {
            proc_rssi_scan();
        }else {
            if(!ll_ctx.recon_rssi_scanning_cnt && ((ll_ctx.stk_state == STATE_PAIRING) || (ll_ctx.stk_state == STATE_RECONNECT)))
            {
                ll_ctx.pr_cnt = 0;
            }
        }
        #endif
        if (ll_ctx.report_rate & (REPORT_RATE_250 | REPORT_RATE_125)) {
            if ((ll_ctx.report_rate & REPORT_RATE_125) && !ll_ctx.suspend_flag)
            {
                if (ll_ctx.srx_task_slot >= (ll_ctx.max_slot - 1))
                {
                    pp_timer1_stop();
                    ll_ctx.suspend_flag = true;
                }
            }
            ll_ctx.rx_over = true;
        }
        rf_clr_irq_status(FLD_RF_IRQ_FIRST_TIMEOUT);
    }
    else if(rf_get_irq_status(FLD_RF_IRQ_RX_TIMEOUT))
    {
        pp_rf_proc_rx_to();
        // ll_ctx.rx_over = true;
        if (ll_ctx.report_rate & ~(REPORT_RATE_250 | REPORT_RATE_125)) {
            ll_ctx.rx_over = true;
        }
        if ((ll_ctx.report_rate & REPORT_RATE_4K) && tick1_exceed_tick2(ll_ctx.last_stick + 70 * RF_SYSTEM_TIMER_TICK_1US, clock_time())) {
            if (ll_ctx.rx_over) {
                ll_ctx.rx_over = false;
                ll_ctx.srx_task_slot = ll_ctx.task_slot;
                #if TPSLL_N8K_FHSS_EN
                #if TPSLL_N8K_CHN_EXPAND_EN
                uint8_t ch;
                if(ll_ctx.task_slot) {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 7];
                } else {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                }
                #else
                uint8_t ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                #endif
                #else
                uint8_t ch = g_n8k_chn_tab[(ll_ctx.fno_last + ll_ctx.task_slot) & (NON_8K_CHN_NUM - 1)];
                #endif
                rf_set_ble_chn(ch);
                pp_rf_start_srx(ll_ctx.last_stick + 73 * RF_SYSTEM_TIMER_TICK_1US, KB_CONNECTED_KB_RX_TIMEOUT + 10);
            }
        }
        else if ((ll_ctx.report_rate & REPORT_RATE_2K) && tick1_exceed_tick2(ll_ctx.last_stick + 100 * RF_SYSTEM_TIMER_TICK_1US, clock_time())) {
            if (ll_ctx.rx_over) {
                ll_ctx.rx_over = false;
                ll_ctx.srx_task_slot = ll_ctx.task_slot;
                #if TPSLL_N8K_FHSS_EN
                #if TPSLL_N8K_CHN_EXPAND_EN
                uint8_t ch;
                if(ll_ctx.task_slot) {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 7];
                } else {
                    ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                }
                #else
                uint8_t ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
                #endif
                #else
                uint8_t ch = g_n8k_chn_tab[(ll_ctx.fno_last + ll_ctx.task_slot) & (NON_8K_CHN_NUM - 1)];
                #endif
                rf_set_ble_chn(ch);
                pp_rf_start_srx(ll_ctx.last_stick + 110 * RF_SYSTEM_TIMER_TICK_1US, KB_CONNECTED_KB_RX_TIMEOUT + 17);
            }
        }
        if (ll_ctx.report_rate & (REPORT_RATE_250 | REPORT_RATE_125)) {
            if ((ll_ctx.report_rate & REPORT_RATE_125) && !ll_ctx.suspend_flag)
            {
                if (ll_ctx.srx_task_slot >= (ll_ctx.max_slot - 1))
                {
                    pp_timer1_stop();
                    ll_ctx.suspend_flag = true;
                }
            }
            ll_ctx.rx_over = true;
        }
        rf_clr_irq_status(FLD_RF_IRQ_RX_TIMEOUT);
    }

    // if ((ll_ctx.report_rate & REPORT_RATE_125) && !ll_ctx.suspend_flag)
    // {
    //     if (ll_ctx.srx_task_slot >= (ll_ctx.max_slot - 1))
    //     {
    //         pp_timer1_stop();
    //         ll_ctx.suspend_flag = true;
    //     }
    // }

    if (!ll_ctx.suspend_flag)
    {
        ll_ctx.wfi_flag = WFI_TRUE;
    }
    // rf_clr_irq_status(FLD_RF_IRQ_ALL);
    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ void app_stack_state_update(void)
{
    static tpsll_dev_status_e last_state = STATE_NONE;
    static unsigned int state_change_tick = 0;

    if (last_state != ll_ctx.stk_state) {
        last_state = ll_ctx.stk_state;
        state_change_tick = stimer_get_tick() | 1;
    }

    if (ll_ctx.stk_state == STATE_PAIRING) {
        unsigned int cur_tick = stimer_get_tick() | 1;
//        if(state_change_tick && clock_time_exceed(state_change_tick, PAIR_TIMEOUT_US))
        if (state_change_tick && ((unsigned int)(cur_tick - state_change_tick) > PAIR_TIMEOUT_US * SYSTEM_TIMER_TICK_1US))
        {
            #if 1//(HW_BOARD_TYPE==HW_DIG_KEYBOARD)
            // stack_evt_post_from_isr(TPSLL_EVT_PAIR_TIMEOUT, 0, 0);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_PAIR_TIMEOUT, 0, 0);
            tpsll_enter_idle(TPSLL_PM_OPTM_LEVEL4);
            #endif
//            app_halt_system(0xe0, "state_change_tick:%d, cur_tick:%d, dt:%d(%d), r:%d\n", state_change_tick, cur_tick, (unsigned int)(cur_tick - state_change_tick),
//                            PAIR_TIMEOUT_US * SYSTEM_TIMER_TICK_1US, ((unsigned int)(cur_tick - state_change_tick) > PAIR_TIMEOUT_US * SYSTEM_TIMER_TICK_1US));
            state_change_tick = 0;
        }
    }
}


_attribute_ram_code_sec_ static bool p24g_rr_switch_check(void)
{

#if 1
    bool rr_changed = false;

    if (ll_ctx.is_rr_changing /*&& ll_ctx.is_rr_chg_acked*/&& (ll_ctx.pending_rr_fno == ll_ctx.fno_last))
    {
        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP1) {
            ll_ctx.rr_chg_with_recon = RR_RECON_STEP3;
            return true;
        }
        #endif

        if(ll_ctx.last_rr == ll_ctx.chg_rr)//change slot mode
        {
            ll_ctx.is_rr_changing = false;
            tpsll_wfi_inter_en(true);
            tpsll_suspend_inter_en(true);
            return rr_changed;
        }
        if ((ll_ctx.last_rr & REPORT_RATE_2K) && (ll_ctx.chg_rr & REPORT_RATE_8K)) {
            ll_ctx.is_rr_l2h = true;
        } else if ((ll_ctx.last_rr & HIGH_REPORT_RATES) && !(ll_ctx.chg_rr & HIGH_REPORT_RATES)) {
            ll_ctx.is_rr_h2l = true;
        }

        app_d24g_set_report_rate(ll_ctx.chg_rr);
        mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_CLK_REINIT_TRIGGER, &ll_ctx.chg_rr);

        #if TPSLL_ACC_SWITCH_WITH_RR_EN
        {
            uint32_t acc;
            p24g_gen_ac_and_channel(ll_inf.peer_mac, &acc, 0);
            rf_access_code_comm(acc);
            // tlkapi_printf(APP_LOG_EN, "acc(%X)", acc);
        }
        #endif

        rr_changed = true;
        ll_ctx.is_rr_changing = false;
        ll_ctx.spp_flag = false;
        ll_ctx.ll_flag = false;
        ll_ctx.suspend_flag = false;
        ll_ctx.rx_over = true;


        if (ll_ctx.report_rate & REPORT_RATE_8K)
        {
            ll_ctx.tx_sn = 0;
            ll_ctx.rx_nesn = 1;
            ll_ctx.s_8k = 0;
            rf_tx_packet.snnesn = 0;
            #if TPSLL_SPP_NOT_SWITCH_EN
            SET_SN(rf_tx_packet.snnesn, ll_ctx.tx_sn);
            SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);
            #endif
        }
        else
        {
            ll_ctx.tx_sn = 0;
            ll_ctx.rx_nesn = 1;
            SET_SN(rf_tx_packet.snnesn, ll_ctx.tx_sn);
            SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);
        }

        #if KB_SPP_FRAG_EN
        tpsll_spp_rx_assemble_reset();
        #endif

        if(ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250 | REPORT_RATE_8K))
        {
            tpsll_suspend_inter_en(true);
        }

        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP6) {
            ll_ctx.rr_chg_with_recon = RR_RECON_STEP_NONE;
            ll_ctx.rr_chg_recon_time_tick = 0;
            tpsll_wfi_inter_en(true);
            tpsll_suspend_inter_en(true);
        }
        #endif
        // stack_evt_post_from_isr(TPSLL_EVT_REPORT_RATE_CHANGED, (void *)&ll_ctx.report_rate, 1);
        s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_REPORT_RATE_CHANGED,  (void *)&ll_ctx.report_rate, 1);

        ll_ctx.is_spp_pre_acked = true;

    }
    #if TPSLL_RR_CHG_WITH_RECON_EN
    else if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP3) {
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
        return true;
    }
    #endif

    return rr_changed;

#endif

    
}

static inline void tpsll_connect_lost_check(void)
{
    if (ll_ctx.lost_cnt)
    {
        ll_ctx.lost_cnt++;
        if (ll_ctx.lost_cnt > ll_ctx.lost_cnt_max)
        { /* 50ms */
            #if TPSLL_CONN_QUICK_LOST_TIME_US
            if (ll_ctx.quick_discon_active_tick && clock_time_exceed(ll_ctx.quick_discon_active_tick, TPSLL_CONN_QUICK_LOST_TIME_US)) {
                ll_ctx.quick_discon_active_tick = 0;
            }
            #endif
            // app_halt_system(ERROR_LOST_CONN, "app_stimer_irq_handler lost_cnt > 400 \n");
            #if TPSLL_CONN_QUICK_LOST_TIME_US
            if (ll_ctx.connected_active_tick && clock_time_exceed(ll_ctx.connected_active_tick, ll_ctx.quick_discon_active_tick ? TPSLL_CONN_QUICK_LOST_TIMEOUT_US : (ll_ctx.report_rate & REPORT_RATE_8K) ? TPSLL_CONN_TIMEOUT_8K_US : ll_ctx.disconnect_timeout_us))
            #else
            if (ll_ctx.connected_active_tick && clock_time_exceed(ll_ctx.connected_active_tick, ll_ctx.disconnect_timeout_us))
            #endif
            {
                ll_ctx.connected_active_tick = 0;
                tpsll_connect_lost();
                tpsll_enable_reconn(true);
                // stack_evt_post_from_isr(TPSLL_EVT_DEV_DISCONNECTED, (void *)ll_inf.peer_mac, MAC_ADDR_LEN);
                s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_DEV_DISCONNECTED, (void *)ll_inf.peer_mac, MAC_ADDR_LEN);
            }
        }
        #if TPSLL_CONNECTING_CHECK_EN
        if (ll_ctx.connecting_check) {
            if (ll_ctx.connecting_check < (TPSLL_RR_DELAY_SWITCH_FNO_N - 3)) {
                ll_ctx.connecting_check++;
            } else {
                tpsll_connect_lost();
                tpsll_enable_reconn(true);
                #if TPSLL_RR_CHG_WITH_RECON_EN
                if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP6) {
                    ll_ctx.rr_chg_with_recon = RR_RECON_STEP4;
                    tlkapi_printf(APP_LOG_EN, "connecting retry >>\n");
                } else
                #endif
                {
                    tlkapi_printf(APP_LOG_EN, "connecting fail !!!\n");
                }
            }
        }
        #endif
    }
}

#if TPSLL_KB_ACTIVE_PM_OPTM_EN
static inline void tpsll_active_pm_optm_update(void)
{
    uint32_t kb_alive_time_steps = ll_ctx.kb_alive_intervals_cnt >> ll_ctx.kb_alive_16ms_intervals_shift;

    if (kb_alive_time_steps < (TPSLL_KB_ALIVE_MAX_STAGED_TIME_MS / TPSLL_KB_ALIVE_TIME_STEP_MS)) {
        if ((ll_ctx.kb_alive_intervals_cnt & ll_ctx.kb_alive_16ms_intervals_msk) == 0) {
            if (ll_ctx.kb_active_stage1_start_time_steps && (kb_alive_time_steps == ll_ctx.kb_active_stage1_start_time_steps)) {
                ll_ctx.kb_alive_fno_msk = TPSLL_KB_8K_ALIVE_STAGE1_FNO_MASK;
            }
            else if (ll_ctx.kb_active_stage2_start_time_steps && (kb_alive_time_steps == ll_ctx.kb_active_stage2_start_time_steps)) {
                ll_ctx.kb_alive_fno_msk = TPSLL_KB_8K_ALIVE_STAGE2_FNO_MASK;
            }
            else if (ll_ctx.kb_active_stage3_start_time_steps && (kb_alive_time_steps == ll_ctx.kb_active_stage3_start_time_steps)) {
                ll_ctx.kb_alive_fno_msk = TPSLL_KB_8K_ALIVE_STAGE3_FNO_MASK;
            }
            else if (ll_ctx.kb_active_stage4_start_time_steps && (kb_alive_time_steps == ll_ctx.kb_active_stage4_start_time_steps)) {
                ll_ctx.kb_alive_fno_msk = TPSLL_KB_8K_ALIVE_STAGE4_FNO_MASK;
            }
            else if (ll_ctx.kb_active_stage5_start_time_steps && (kb_alive_time_steps == ll_ctx.kb_active_stage5_start_time_steps)) {
                ll_ctx.kb_alive_fno_msk = TPSLL_KB_8K_ALIVE_STAGE5_FNO_MASK;
            }
            #ifdef TPSLL_KB_8K_ALIVE_STAGE6_FNO_MASK
            else if (ll_ctx.kb_active_stage6_start_time_steps && (kb_alive_time_steps == ll_ctx.kb_active_stage6_start_time_steps)) {
                ll_ctx.kb_alive_fno_msk = TPSLL_KB_8K_ALIVE_STAGE6_FNO_MASK;
            }
            #endif
        }
        ll_ctx.kb_alive_intervals_cnt++;
    }
}
#endif /* end if TPSLL_KB_ACTIVE_PM_OPTM_EN */

#if TPSLL_PM_OPTM_EN
static inline void tpsll_pm_optm_level_update(void)
{
    if (ll_ctx.is_rr_just_changed) {
        if (ll_ctx.pm_optm_ref_fno_cnt >= ll_ctx.pm_optm_rr_chgd_ref_fno_cnt_th) {
            ll_ctx.is_rr_just_changed = false;
        }
    } else if (ll_ctx.pm_optm_3rd_ref_fno_cnt_th && (ll_ctx.pm_optm_ref_fno_cnt >= ll_ctx.pm_optm_3rd_ref_fno_cnt_th)) {
        if (!(ll_ctx.pm_optm_level & TPSLL_PM_OPTM_LEVEL3)) {
            ll_ctx.pm_optm_level = TPSLL_PM_OPTM_LEVEL3;
            tpsll_enter_idle(TPSLL_PM_OPTM_LEVEL3);
            // ll_ctx.ks_need_disable = false;
        }
    } else if (ll_ctx.pm_optm_2nd_ref_fno_cnt_th &&
            (ll_ctx.pm_optm_ref_fno_cnt >= ll_ctx.pm_optm_2nd_ref_fno_cnt_th)) {
        if (!(ll_ctx.pm_optm_level & TPSLL_PM_OPTM_LEVEL2)) {
            ll_ctx.pm_optm_level = TPSLL_PM_OPTM_LEVEL2;
            if (ll_ctx.report_rate & REPORT_RATE_8K) {
                ll_ctx.keep_alive_sync_us = TPSLL_PM_OPTM_LEVEL2_8K_KEEP_ALIVE_SYNC_US;
            } else if (ll_ctx.report_rate & REPORT_RATE_250) {
                ll_ctx.keep_alive_sync_us = TPSLL_PM_OPTM_LEVEL2_250_KEEP_ALIVE_SYNC_US;
            } else if (ll_ctx.report_rate & REPORT_RATE_125) {
                ll_ctx.keep_alive_sync_us = TPSLL_PM_OPTM_LEVEL2_125_KEEP_ALIVE_SYNC_US;
            } else {
                ll_ctx.keep_alive_sync_us = TPSLL_PM_OPTM_LEVEL2_N8K_KEEP_ALIVE_SYNC_US;
            }
            ll_ctx.keep_alive_sync_us -= ll_ctx.interval_us;

            // stack_evt_post_from_isr(TPSLL_EVT_PM_OPTM_ENTER_LEVEL2, 0, 0);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_PM_OPTM_ENTER_LEVEL2, 0, 0);
            // ll_ctx.ks_need_disable = false;
        }
    } else if (ll_ctx.pm_optm_1st_ref_fno_cnt_th &&
            (ll_ctx.report_rate & ~REPORT_RATE_125) &&
            (ll_ctx.pm_optm_ref_fno_cnt >= ll_ctx.pm_optm_1st_ref_fno_cnt_th)) {
        if (!(ll_ctx.pm_optm_level & TPSLL_PM_OPTM_LEVEL1)) {
            ll_ctx.pm_optm_level = TPSLL_PM_OPTM_LEVEL1;
            // ll_ctx.keep_alive_sync_us = TPSLL_PM_OPTM_LEVEL1_KEEP_ALIVE_SYNC_US;
            ll_ctx.keep_alive_sync_us = TPSLL_PM_OPTM_LEVEL1_KEEP_ALIVE_SYNC_US - ll_ctx.interval_us;
            // stack_evt_post_from_isr(TPSLL_EVT_PM_OPTM_ENTER_LEVEL1, 0, 0);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_PM_OPTM_ENTER_LEVEL1, 0, 0);
            // ll_ctx.ks_need_disable = false;
        }
    } else {
        if (ll_ctx.pm_optm_level) {
            ll_ctx.pm_optm_level = 0;
            // stack_evt_post_from_isr(TPSLL_EVT_PM_OPTM_EXIT, 0, 0);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_PM_OPTM_EXIT, 0, 0);
            // ll_ctx.ks_need_disable = false;
        }
    }
}
#endif /* end of TPSLL_PM_OPTM_EN */

_attribute_ram_code_sec_ static void tpsll_connected_stimer_irq_handler(void)
{
    uint8_t task_slot_last = ll_ctx.task_slot;
    // SLOT0 start
    // KB 1T
    ll_ctx.step_3t1r = STEP_3T1R_1T;
    ll_ctx.task_slot = TASK_SLOT0;

    if (!p24g_rr_switch_check())
    {
        bool tx_valid_flag = true;

        {
            // app_rf_set_rx_wait(1); // rx wait
            #if TPSLL_SPP_NOT_SWITCH_EN
            if (ll_ctx.report_rate & REPORT_RATE_8K) {
                rf_packet_t  *p = (rf_packet_t *)rx_packet;

                if (tpsll_is_rf_busy()) {
                    #if TPSLL_PM_OPTM_EN
                    if((p->pkt_type == P24G_PKT_TYPE_SPP) || tpsll_pm_optm_is_long_suspend_support_level_active())
                    #else
                    if(p->pkt_type == P24G_PKT_TYPE_SPP)
                    #endif
                    {
                        tx_valid_flag = false;
                    } else {
                        rf_set_tx_rx_off_auto_mode();
                        ll_ctx.rx_over = true;
                    }
                }
            }
            else if (ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500)) {
                if (tpsll_is_rf_busy()) {
                    rf_set_tx_rx_off_auto_mode();
                    ll_ctx.rx_over = true;
                }
            }
            else if(ll_ctx.report_rate & REPORT_RATE_4K) {
                if (tpsll_is_rf_busy() && (ll_ctx.srx_task_slot == TASK_SLOT0)) {
                    rf_set_tx_rx_off_auto_mode();
                    ll_ctx.rx_over = true;
                }
            }
            #endif

            if (!ll_ctx.rx_over) {
                tx_valid_flag = false;
            }

            if (tx_valid_flag)
            {
                if(ll_ctx.report_rate & REPORT_RATE_8K)
                {
                    app_8k_rf_pkt_send();
                }else{
                    app_non_8k_rf_pkt_send();
                }
            }

            #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
            if ((ll_ctx.report_rate & (REPORT_RATE_2K | REPORT_RATE_1K | REPORT_RATE_500)) && tpsll_is_active_state() && ll_ctx.is_alg_kb) {
                mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_ENABLE_TRIGGER, NULL);
                ll_ctx.key_scan_disable = false;
            }
            #endif

            ll_ctx.last_valid_slot_id = -1;

            // unsigned int r = core_interrupt_disable();
            // pp_timer1_start(APP_SLOT1_START_TIME_US * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); //set time for SLOT1
            if (ll_ctx.report_rate & REPORT_RATE_8K)
            {
                ll_ctx.acked = false;
                calc_hop_channels(ll_ctx.fno_last + 1, 0);
                #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
                if(tpsll_is_keep_alive_active() && !ll_ctx.key_scan_disable && ll_ctx.is_alg_kb) {
                    ll_ctx.ks_need_disable = true;
                    pp_timer1_start(TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_SLOT_TIME_US * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT1
                } else
                #endif /* end of TPSLL_PM_OPTM_EN */
                {
                    pp_timer1_start(ll_ctx.slot1_us * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT1
                }
            }
            else if (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250))
            {
                #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN && TPSLL_PM_OPTM_N8K_SUSPEND_EN
                if (ll_ctx.is_alg_kb) {
                    ll_ctx.ks_need_disable = true;
                }
                #endif /* end of TPSLL_PM_OPTM_EN */
                pp_timer1_start((SLOT1_OFFSET_TIME_US_125)*RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT1
            }
            else
            {
                // ll_ctx.btn_scan = 1;
                // pp_timer1_start((SLOT_TIME_US - 10) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT1
                #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN && TPSLL_PM_OPTM_N8K_SUSPEND_EN
                if(ll_ctx.report_rate & REPORT_RATE_4K)
                #else
                if((ll_ctx.report_rate & REPORT_RATE_4K) && ll_ctx.is_keep_alive_sync)
                #endif
                {
                    #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN && TPSLL_PM_OPTM_N8K_SUSPEND_EN
                    if(tpsll_is_keep_alive_active() && !ll_ctx.key_scan_disable && ll_ctx.is_alg_kb) {
                        ll_ctx.ks_need_disable = true;
                        pp_timer1_start(TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_SLOT_TIME_US * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT1
                    } else
                    #endif /* end of TPSLL_PM_OPTM_EN */
                    if (tx_valid_flag) {
                        pp_timer1_start((127)*RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT1
                    }
                }else{
                    #if TPSLL_NEW_KEEP_ALIVE_EN && !TPSLL_PM_OPTM_N8K_SUSPEND_EN
                    if(ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500) && ll_ctx.is_keep_alive_sync)
                    #else
                    if(ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500))
                    #endif
                    {
                        #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN && TPSLL_PM_OPTM_N8K_SUSPEND_EN
                        if(tpsll_is_keep_alive_active() && !ll_ctx.key_scan_disable && ll_ctx.is_alg_kb) {
                            ll_ctx.ks_need_disable = true;
                            pp_timer1_start(KM_SLOT1_TIME_US_500_1K_2K * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT1
                        } else if (tpsll_is_active_state() && ll_ctx.is_alg_kb) {
                            ll_ctx.ks_need_disable = true;
                            pp_timer1_start(TPSLL_PM_OPTM_TIMER1_KS_DISABLE_DELAY_US * RF_SYSTEM_TIMER_TICK_1US);
                        } else
                        #endif /* end of TPSLL_PM_OPTM_EN */
                        pp_timer1_start(KM_SLOT1_TIME_US_500_1K_2K * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT1
                    }
                    #if TPSLL_NEW_KEEP_ALIVE_EN && !TPSLL_PM_OPTM_N8K_SUSPEND_EN
                    else if(ll_ctx.is_keep_alive_sync)
                    #else
                    else
                    #endif
                    {//2K
                        #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN && TPSLL_PM_OPTM_N8K_SUSPEND_EN
                        if(tpsll_is_keep_alive_active() && !ll_ctx.key_scan_disable && ll_ctx.is_alg_kb) {
                            ll_ctx.ks_need_disable = true;
                            pp_timer1_start(KM_SLOT1_TIME_US_500_1K_2K * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT1
                        } else if (tpsll_is_active_state() && ll_ctx.is_alg_kb) {
                            ll_ctx.ks_need_disable = true;
                            pp_timer1_start(TPSLL_PM_OPTM_TIMER1_KS_DISABLE_DELAY_US * RF_SYSTEM_TIMER_TICK_1US);
                        } else
                        #endif /* end of TPSLL_PM_OPTM_EN */
                        pp_timer1_start(KM_SLOT1_TIME_US_500_1K_2K * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT1
                    }
                }
//                ll_ctx.all_pkt_sent = false;
            }

            // core_restore_interrupt(r);
            #if TPSLL_KEEP_ALIVE_EN
            if (tpsll_is_keep_alive_active() && (ll_ctx.report_rate & (/*REPORT_RATE_4K | */REPORT_RATE_2K | REPORT_RATE_1K | REPORT_RATE_500)) &&
                    ll_ctx.synced && (task_slot_last >= (ll_ctx.max_slot - 1)))
            {
                if (ll_ctx.alive_interval_cnt) {
                    ll_ctx.alive_interval_cnt--;
                } else {
                    ll_ctx.alive_interval_cnt = ll_ctx.alive_interval_max;
                    ll_ctx.synced = 0;
                }
            }

            if (ll_ctx.key_scan_disable
                #if TPSLL_PM_OPTM_EN
                // ((ll_ctx.report_rate & REPORT_RATE_8K) && !tpsll_is_keep_alive_active())
                #if TPSLL_PM_OPTM_N8K_SUSPEND_EN
                && (ll_ctx.report_rate & ~(REPORT_RATE_250 | REPORT_RATE_125))
                #endif
                && tpsll_is_active_state()
                #endif
                && ll_ctx.is_alg_kb
                // ((ll_ctx.report_rate & REPORT_RATE_NOT_8K_MSK) && !ll_ctx.synced))
            ) {
                mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_ENABLE_TRIGGER, NULL);
                ll_ctx.key_scan_disable = false;
            }
            #endif
        }
    }

    if(ll_ctx.spp_cnt > SPP_MAX_CONT_CNT)
    {
        ll_ctx.time_synced_cnt = 0;
    }

    ll_ctx.paired_cnt = 0;
    ll_ctx.reconnected_cnt = 0;

    ll_ctx.cur_rx_flag = false;
    ll_ctx.wfi_flag = WFI_TRUE;

    #if TPSLL_KB_ACTIVE_PM_OPTM_EN
    if(ll_ctx.report_rate & (REPORT_RATE_4K | REPORT_RATE_2K | REPORT_RATE_1K))
    {
        if (tpsll_is_keep_alive_active() || !tpsll_all_fifo_is_empty()) {
            ll_ctx.kb_alive_intervals_cnt = 0;
            ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
        } else {
            tpsll_active_pm_optm_update();
        }
    }
    #endif /* end of TPSLL_KB_ACTIVE_PM_OPTM_EN */

    #if TPSLL_PM_OPTM_EN
    if ((ll_ctx.pm_optm_ref_fno_cnt > 1) && (ll_ctx.pm_optm_ref_fno_cnt < TPSLL_PM_OPTM_REF_FNO_CNT_MAX)) {
        ll_ctx.pm_optm_ref_fno_cnt += (uint16_t)(ll_ctx.fno_last - ll_ctx.fno_last_ref);
        ll_ctx.fno_last_ref = ll_ctx.fno_last;
        tpsll_pm_optm_level_update();
    }

    if ((((ll_ctx.report_rate & (REPORT_RATE_2K | REPORT_RATE_1K | REPORT_RATE_500)) && tpsll_is_keep_alive_active()) ||
        (ll_ctx.report_rate & (REPORT_RATE_250 | REPORT_RATE_125))) &&
        ll_ctx.ks_need_disable) {
        if (!ll_ctx.key_scan_disable) {
            mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_DISABLE_TRIGGER, NULL);
            ll_ctx.key_scan_disable = true;
        }
        ll_ctx.ks_need_disable = false;
    }
    #endif /* end of TPSLL_PM_OPTM_EN */
}

_attribute_ram_code_sec_ static void tpsll_pairing_stimer_irq_handler(void)
{
    #if TPSLL_NEW_PAIR_EN
////////////////////////////////////////////////////////////////////
    if (0 != (read_reg8(0x170224) & 0x7))
    {
        rf_set_tx_rx_off_auto_mode();
    }

    ll_ctx.paired_cnt++;
    if (ll_ctx.paired_cnt >= ll_ctx.pair_timeout_cnt)
    {
        ll_ctx.paired_cnt = 0;
        // stack_evt_post_from_isr(TPSLL_EVT_PAIR_TIMEOUT, 0, 0);
        s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_PAIR_TIMEOUT, 0, 0);
        tpsll_enter_idle(TPSLL_PM_OPTM_LEVEL4);
        return;
    }

    if (ll_ctx.pair_st == STATE_PAIR_INIT) {
// gpio_toggle(dbg_io[DBG_IO_IDX2]);
// gpio_toggle(dbg_io[DBG_IO_IDX2]);
        #if TPSLL_RECON_RSSI_SCAN_EN
        {
            ll_ctx.reconn_chn = g_pair_valid_chn_tab[ll_ctx.reconn_chn_idx-- % TPSLL_PAIR_CHN_MAX_NUM];
            rf_set_ble_chn(ll_ctx.reconn_chn);
        }
        #else
        static unsigned char reconn_chn_idx = 0;
        rf_set_ble_chn(reconn_chn_table[reconn_chn_idx & (RECONN_CHN_NUM - 1)]);
        reconn_chn_idx++;
        #endif

        rf_access_code_comm(ll_ctx.pair_acc);
        app_rf_start_stx2rx(ll_ctx.last_stick + PARING_STIMER_TO_TX_US * RF_SYSTEM_TIMER_TICK_1US, KM_PAIRING_SRX_TIMEOUT_US + 5);

        pp_rf_set_tx_len(TPSLL_PAIR_REQ_MAX_LEN);

        pair_req_packet_t * pt = (pair_req_packet_t *)(rf_tx_packet.buf);

        pt->cmd = CMD_PAIR_REQ;
        pt->km_tag = ll_inf.tag;
        pt->encrypt_key = ll_ctx.pair_encrypt_key;

        rf_set_rx_maxlen(TPSLL_PAIR_RSP_PKT_LEN);
        ll_ctx.pair_cnt = 0;

        ll_ctx.pair_step = PAIR_STEP1;

        #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
        if (!(ll_ctx.fno_last & TPSLL_PM_OPTM_PAIR_RECON_KS_DISABLE_FNO_MASK) && ll_ctx.is_alg_kb) {
            mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_ENABLE_TRIGGER, NULL);
            ll_ctx.key_scan_disable = false;
            ll_ctx.ks_need_disable = true;
            pp_timer1_start(TPSLL_PM_OPTM_PAIR_RECON_KS_DISABLE_DELAY_US * RF_SYSTEM_TIMER_TICK_1US);
        }
        #endif
    }
    else if (ll_ctx.pair_st == STATE_PAIR_RSP) {
        if (ll_ctx.pair_cnt > PAIRING_RSP_MAX_RETRY_CNT)
        {
            tpsll_enable_pairing(true, pairing_tx_power);
            return;
        }

        app_rf_start_stx2rx(ll_ctx.last_stick + PARING_STIMER_TO_TX_US * RF_SYSTEM_TIMER_TICK_1US, KM_PAIRING_SRX_TIMEOUT_US + 5);

        rf_access_code_comm(~ll_ctx.pair_acc);

        pp_rf_set_tx_len(TPSLL_PAIR_CON_REQ_LEN);

        conn_req_packet_t *pt = (conn_req_packet_t *)(rf_tx_packet.buf);

        pt->cmd = CMD_CONN_REQ;
        pt->rr = ll_ctx.user_rr;
        pt->tag = ll_inf.peer_tag;
        pt->km_tag = ll_inf.tag;
        tmemcpy(pt->mac, ll_inf.mac, MAC_ADDR_LEN);

        #if TPSLL_RECON_RSSI_SCAN_EN
        rf_set_rx_maxlen(TPSLL_PAIR_CON_RSP_LEN);
        #else
        rf_set_rx_maxlen(TPSLL_PAIR_CON_RSP_LEN - 3);
        #endif

        ll_ctx.pair_cnt++;
        ll_ctx.pair_step = PAIR_STEP2;
        pp_timer1_start((135) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));
    }
////////////////////////////////////////////////////////////////////
    #else /* end of TPSLL_NEW_PAIR_EN */
    ll_ctx.paired_cnt++;
    if (ll_ctx.paired_cnt >= ll_ctx.pair_timeout_cnt)
    {
        ll_ctx.paired_cnt = 0;
        // stack_evt_post_from_isr(TPSLL_EVT_PAIR_TIMEOUT, 0, 0);
        s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_PAIR_TIMEOUT, 0, 0);
        tpsll_enter_idle(TPSLL_PM_OPTM_LEVEL4);
        return;
    }

    if (ll_ctx.pair_st == STATE_PAIR_INIT)
    {
        ll_ctx.pr_cnt++;
        if ((ll_ctx.pr_cnt > 1) && (ll_ctx.pr_cnt % 400))
        {
            return;
        }
        rf_set_ble_chn(PAIR_CHN_NO);
        rf_access_code_comm(ll_ctx.pair_acc);
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
        // pairing SRX
       pp_rf_start_srx(stimer_get_tick(), KB_PAIRING_RX_TIMEOUT);
        // pp_rf_start_srx(stimer_get_tick(), 56);
        rf_set_rx_maxlen(5);
        // pp_rf_start_srx(ll_ctx.last_stick + KB_PAIRING_STIMER_TO_RX_MARGIN_US*RF_SYSTEM_TIMER_TICK_1US, 200);
        ll_ctx.pair_step = PAIR_STEP1;
    }
    else if (ll_ctx.pair_st == STATE_PAIR_RSP)
    {

        if (ll_ctx.pair_cnt > PAIRING_RSP_MAX_RETRY_CNT)
        {
            tpsll_enable_pairing(true, pairing_tx_power);

            return;
        }
        rf_set_ble_chn(PAIR_CHN_NO);
        rf_access_code_comm(~ll_ctx.pair_acc);
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
        // pairing SRX
        pp_rf_start_srx(stimer_get_tick(), KB_PAIRING_RSP_RX_TIMEOUT);
        rf_set_rx_maxlen(18);
        ll_ctx.pair_cnt++;
        ll_ctx.pair_step = PAIR_STEP2;
    }
    #endif /* end of TPSLL_NEW_PAIR_EN #else */
}

_attribute_ram_code_sec_ static void tpsll_reconn_stimer_irq_handler(void)
{
    #if TPSLL_NEW_RECON_EN
////////////////////////////////////////////////////////////////////
    if (0 != (read_reg8(0x170224) & 0x7))
    {
        rf_set_tx_rx_off_auto_mode();
    }
// gpio_toggle(dbg_io[DBG_IO_IDX2]);

    ll_ctx.reconnected_cnt++;
    if (ll_ctx.reconnected_cnt >= ll_ctx.reconnect_timeout_cnt)
    {
        ll_ctx.reconnected_cnt = 0;
        // stack_evt_post_from_isr(TPSLL_EVT_RECONNECT_TIMEOUT, 0, 0);
        s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_RECONNECT_TIMEOUT, 0, 0);
        tpsll_enter_idle(TPSLL_PM_OPTM_LEVEL4);
    }

    #if TPSLL_RECON_RSSI_SCAN_EN
    {
        ll_ctx.reconn_chn = g_fhss_valid_chn_tab[ll_ctx.reconn_chn_idx-- % TPSLL_FHSS_CHN_MAX_NUM];
        rf_set_ble_chn(ll_ctx.reconn_chn);
    }
    #else
    static unsigned char reconn_chn_idx = 0;
    // rf_set_ble_chn(PAIR_CHN);
    // rf_set_ble_chn(ll_ctx.reconn_chn);
    rf_set_ble_chn(reconn_chn_table[reconn_chn_idx & (RECONN_CHN_NUM - 1)]);
    reconn_chn_idx++;
    #endif
    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);

    app_rf_start_stx2rx(ll_ctx.last_stick + PARING_STIMER_TO_TX_US * RF_SYSTEM_TIMER_TICK_1US, KM_PAIRING_SRX_TIMEOUT_US + 5);

    reconn_req_packet_t *pt = (reconn_req_packet_t *)&rf_tx_packet;

    pp_rf_set_tx_len(6);

    pt->cmd = CMD_RECONN_REQ;
    pt->rr = ll_ctx.user_rr;
    pt->tag = ll_inf.peer_tag;
    pt->km_tag = ll_inf.tag;

    #if TPSLL_RECON_RSSI_SCAN_EN
    #if TPSLL_8K_CHN_EXPAND_EN
    rf_set_rx_maxlen(12 + 3);
    #else
    rf_set_rx_maxlen(12);
    #endif
    #else
    rf_set_rx_maxlen(9);
    #endif

    ll_ctx.recon_step = RECON_STEP1;
    #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
    if (!(ll_ctx.fno_last & TPSLL_PM_OPTM_PAIR_RECON_KS_DISABLE_FNO_MASK) && ll_ctx.is_alg_kb) {
        mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_ENABLE_TRIGGER, NULL);
        ll_ctx.key_scan_disable = false;
        ll_ctx.ks_need_disable = true;
        pp_timer1_start(TPSLL_PM_OPTM_PAIR_RECON_KS_DISABLE_DELAY_US * RF_SYSTEM_TIMER_TICK_1US);
    } else
    #endif
    pp_timer1_start((105) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));
////////////////////////////////////////////////////////////////////
    #else /* end of TPSLL_NEW_RECON_EN */

    ll_ctx.reconnected_cnt++;
    if (ll_ctx.reconnected_cnt >= ll_ctx.reconnect_timeout_cnt)
    {
        ll_ctx.reconnected_cnt = 0;
        // stack_evt_post_from_isr(TPSLL_EVT_RECONNECT_TIMEOUT, 0, 0);
        s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_RECONNECT_TIMEOUT, 0, 0);
        tpsll_enter_idle(TPSLL_PM_OPTM_LEVEL4);
        return;
    }

    #if TPSLL_RECON_RSSI_SCAN_EN
    if (ll_ctx.recon_rssi_scanning_cnt) {
        ll_ctx.reconn_chn_idx = 0;
        ll_ctx.recon_rx_start_tick = 0;
        return;
    }
    #endif

    ll_ctx.pr_cnt++;
    if ((ll_ctx.pr_cnt > 1) && (ll_ctx.pr_cnt % 45))
    {
        return;
    }

    #if TPSLL_RECON_RSSI_SCAN_EN
    uint32_t recon_rx_win_us = KM_RECONN_RX_WIN;

    if (!ll_ctx.recon_rx_start_tick) {
        ll_ctx.recon_rx_start_tick = clock_time() | 1;
    }
    if (ll_ctx.reconn_chn_idx > 2) {
        if (clock_time_exceed(ll_ctx.recon_rx_start_tick, KM_RECONN_RX_WIN * 3 + 200 * 2)) {
            ll_ctx.recon_rx_start_tick = 0;
            ll_ctx.recon_rssi_scanning_cnt = 1;
            return;
        } else { //enough time left
            recon_rx_win_us = (((KM_RECONN_RX_WIN * 3 + 200 * 2) * SYSTEM_TIMER_TICK_1US) - clock_time()) / SYSTEM_TIMER_TICK_1US;
        }
    }
    #endif

    // rf_set_ble_chn(PAIR_CHN);
    // rf_set_ble_chn(ll_ctx.reconn_chn);
    #if TPSLL_RECON_RSSI_SCAN_EN
    // #if TPSLL_RR_CHG_WITH_RECON_EN
    // if (ll_ctx.rr_chg_with_recon) {
    //     rf_set_ble_chn(reconn_chn_table[ll_ctx.reconn_chn_idx & (RECONN_CHN_NUM - 1)]);
    // } else
    // #endif
    {
        ll_ctx.reconn_chn = reconn_chn_table[ll_ctx.reconn_chn_idx % TPSLL_RECON_CHN_NUM];
        rf_set_ble_chn(ll_ctx.reconn_chn);
    }
    #else
    rf_set_ble_chn(reconn_chn_table[ll_ctx.reconn_chn_idx & (RECONN_CHN_NUM - 1)]);
    #endif
    ll_ctx.reconn_chn_idx++;
    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
    // pairing SRX


    // pp_rf_start_srx(stimer_get_tick(), KM_RECONN_RX_WIN);
    {
    #if TPSLL_RECON_RSSI_SCAN_EN
    pp_rf_start_srx(stimer_get_tick(), recon_rx_win_us);
    #else
    pp_rf_start_srx(stimer_get_tick(), KM_RECONN_LONG_RX_WIN);
    #endif
    write_reg8(0x17051a,38);
    }
    // pp_rf_start_srx(stimer_get_tick(), 300000);
    rf_set_rx_maxlen(9);

    ll_ctx.recon_step = RECON_STEP1;
    #endif /* end of TPSLL_NEW_RECON_EN #else */
}

_attribute_ram_code_sec_ static void tpsll_idle_stimer_irq_handler(void)
{
    #if TPSLL_PM_OPTM_EN
    #if TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
    pp_timer1_start((ll_ctx.pm_optm_level & TPSLL_PM_OPTM_LEVEL3) ? TPSLL_PM_OPTM_LEVEL3_TIMER1_KS_DISABLE_DELAY_US * RF_SYSTEM_TIMER_TICK_1US :
                        TPSLL_PM_OPTM_DEEP_RET_TIMER1_KS_DISABLE_DELAY_US * RF_SYSTEM_TIMER_TICK_1US);
    #else
    ll_ctx.suspend_flag = true;
    #endif

    if ((ll_ctx.pm_optm_level & TPSLL_PM_OPTM_LEVEL3) && (ll_ctx.pm_optm_ref_fno_cnt < TPSLL_PM_OPTM_REF_FNO_CNT_MAX)) {
        ll_ctx.pm_optm_ref_fno_cnt++;
        if (ll_ctx.pm_optm_4th_ref_fno_cnt_delta && (ll_ctx.pm_optm_ref_fno_cnt > (ll_ctx.pm_optm_4th_ref_fno_cnt_delta + TPSLL_PM_OPTM_LEVEL3_REF_FNO_CNT_START))) {
            ll_ctx.pm_optm_level = TPSLL_PM_OPTM_LEVEL4;
            // stack_evt_post_from_isr(TPSLL_EVT_PM_OPTM_ENTER_LEVEL4, 0, 0);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_PM_OPTM_ENTER_LEVEL4, 0, 0);
        }
    }
    #endif /* end of TPSLL_PM_OPTM_EN */

    ll_ctx.wfi_flag = true;
    ll_ctx.lost_cnt = 0;
}

_attribute_ram_code_sec_ uint8_t tpsll_stimer_irq_handler(void)
{
    DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);

    if (systimer_get_irq_status())
    {
        if (enable_pairing_flag)
        {
            tpsll_enable_pairing_internal();
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_PAIRING_ENTER,0, 0);
            enable_pairing_flag = 0;
        }
        ll_ctx.fno_last = ll_ctx.fno;
        ll_ctx.last_stick = ll_ctx.stick;

        if (ll_ctx.stk_state == STATE_CONNECTED)
        {
            tpsll_connected_stimer_irq_handler();
        } else if (ll_ctx.stk_state == STATE_RECONNECT)
        {
            tpsll_reconn_stimer_irq_handler();
        } else if (ll_ctx.stk_state == STATE_PAIRING)
        {
            tpsll_pairing_stimer_irq_handler();
        } else if (ll_ctx.stk_state == STATE_IDLE)
        {
            tpsll_idle_stimer_irq_handler();
        }

        app_set_next_stimer_anchor();
        systimer_clr_irq_status();
        tpsll_connect_lost_check();
        app_stack_state_update();
        wd_clear();//Temporarily placed here.
    }

    #if TPSLL_RR_CHG_WITH_RECON_EN
    if (ll_ctx.rr_chg_recon_time_tick &&
        clock_time_exceed(ll_ctx.rr_chg_recon_time_tick, TPSLL_RR_CHG_RECON_TIMEOUT_US)) {
        ll_ctx.rr_chg_with_recon = RR_RECON_STEP_NONE;
        ll_ctx.rr_chg_recon_time_tick = 0;

        if (((ll_ctx.user_rr & REPORT_RATE_8K) && (ll_ctx.report_rate & REPORT_RATE_8K) && ll_ctx.connecting_check) ||
            (ll_ctx.user_rr & ~REPORT_RATE_8K)) {
            // stack_evt_post_from_isr(TPSLL_EVT_DEV_DISCONNECTED, (void *)ll_inf.peer_mac, MAC_ADDR_LEN);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_DEV_DISCONNECTED, (void *)ll_inf.peer_mac, MAC_ADDR_LEN);
        }
    }
    #endif

    #if TPSLL_FHSS_EN
    if (ll_ctx.stk_state == STATE_CONNECTED) {
        if (tpsll_is_afh_slot()
            // && ((chn_table[0] != g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[0]]) ||
            // (chn_table[1] != g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[1]]) ||
            // (chn_table[2] != g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[2]]))
            ){
            if (g_fhss_idx_buf_data_cur != g_fhss_idx_buf_data_next) {
                g_fhss_idx_buf_data_cur = g_fhss_idx_buf_data_next;
                chn_table[0] = g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[0]];
                chn_table[1] = g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[1]];
                chn_table[2] = g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[2]];
                #if TPSLL_N8K_CHN_EXPAND_EN
                if (ll_ctx.report_rate & REPORT_RATE_NOT_8K_MSK) {
                    tpsll_n8k_chn_expand();
                }
                #else
                #if (CHN_TABL_LEN == 4)
                chn_table[3] = chn_table[1];
                #endif
                #endif

                    // tlkapi_printf(APP_LOG_EN, "(%d)(%d)(%d)\n", chn_table[0],
                    //         chn_table[1], chn_table[2]);
                #if 0
                for (int i = 0; i < g_hop_chn_idx_tab_next[0]; i++) {
                    DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
                }
                DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
                for (int i = 0; i < g_hop_chn_idx_tab_next[1]; i++) {
                    DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
                }
                DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
                for (int i = 0; i < g_hop_chn_idx_tab_next[2]; i++) {
                    DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
                }
                DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
                #endif
            }
            // tlkapi_printf(1, "hop_chns:%d-%d-%d\n", g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[0]],
            //         g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[1]], g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[2]]);
        }
    }
    #endif
// if ((ll_ctx.fno_last & 0x03) == 0) {
// DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
// }

    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ void tpsll_timer1_irq_8k_handler(void)
{
    #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
    if (ll_ctx.ks_need_enable) {
        if (ll_ctx.key_scan_disable) {
            mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_ENABLE_TRIGGER, NULL);
            ll_ctx.key_scan_disable = false;
        }
        ll_ctx.ks_need_enable = false;
        return;
    }
    #endif
    if (ll_ctx.task_slot == TASK_SLOT0)
    {
        #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
        if(ll_ctx.ks_need_disable) {
            ll_ctx.ks_need_disable = false;
            pp_timer1_start(APP_SLOT1_START_TIME_US * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT1
            if (!ll_ctx.key_scan_disable) {
                mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_DISABLE_TRIGGER, NULL);
                ll_ctx.key_scan_disable = true;
            }
            if (!tpsll_all_fifo_is_empty()) {
                tpsll_pm_optm_time_refresh();
            }
            return;
        }
        #endif
        // SLOT1 start
        // KB 2T
        ll_ctx.step_3t1r = STEP_3T1R_2T;

        
            #if TPSLL_SPP_NOT_SWITCH_EN
            if(rf_tx_packet.pkt_type != P24G_PKT_TYPE_SPP)
            #endif
            rf_set_ble_chn(chn_list[ll_ctx.step_3t1r]);
        

        // while(0 != (read_reg8(0x170224) & 0x7));
        #if !TPSLL_SPP_NOT_SWITCH_EN
        if (wait_rf_fsm_idle_timeout(3)) // 8k: after TX_EN 4K: 2us
        {
            rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
        }
        #endif

        ll_ctx.task_slot = TASK_SLOT1;
        app_rf_pkt_send();

        unsigned int r = core_interrupt_disable();
        pp_timer1_start(APP_SLOT2_START_TIME_US * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT2
        core_restore_interrupt(r);

        // #if TPSLL_PM_OPTM_EN && !TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
        // if (tpsll_is_keep_alive_active() && !ll_ctx.key_scan_disable && ll_ctx.is_alg_kb) {
        //     stack_evt_post_from_isr(TPSLL_INTER_EVT_KS_DISABLE, NULL, 0);
        // }
        // #endif
    }
    else if (ll_ctx.task_slot == TASK_SLOT1)
    {
        // SLOT2 start

        // KB 3T
        ll_ctx.step_3t1r = STEP_3T1R_3T;
        rf_set_ble_chn(chn_list[ll_ctx.step_3t1r]);
        // while(0 != (read_reg8(0x170224) & 0x7));
        if (wait_rf_fsm_idle_timeout(1)) // 8k: after TX_EN 4K: none
        {
            rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
        }
        app_rf_pkt_send();

        ll_ctx.task_slot = TASK_SLOT2;

        #if TPSLL_8K_STX_SRX_EN

        #if TPSLL_PM_OPTM_EN
        // if (tpsll_pm_optm_is_long_suspend_support_level_active() && !ll_ctx.synced) {
        if (tpsll_pm_optm_is_long_suspend_support_level_active()) {
            unsigned int r = core_interrupt_disable();
            pp_timer1_start((APP_DONGLE_SLOT3_START_TIME_US - TPSLL_PM_OPTM_8K_RX_WIN_EXPAND_US) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT2
            core_restore_interrupt(r);
        } else
        #endif
        {
            unsigned int r = core_interrupt_disable();
            pp_timer1_start(APP_DONGLE_SLOT3_START_TIME_US * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT2
            core_restore_interrupt(r);
        }

        #endif
#if 0
            unsigned int r = core_interrupt_disable();
            pp_timer1_start(APP_DONGLE_SLOT3_START_TIME_US * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));  //set time for SLOT3
            core_restore_interrupt(r);
#endif
    }
    else if (ll_ctx.task_slot == TASK_SLOT2)
    {
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        if (!pp_fifo_is_empty(km_fifo_ptr) || !pp_fifo_is_empty(spp_fifo_ptr) || !pp_fifo_is_empty(&ll_fifo)
            || (!(ll_ctx.fno_last & ll_ctx.kb_alive_fno_msk)  || !ll_ctx.synced))
        #endif
        {
            #if TPSLL_8K_STX_SRX_EN
            #if TPSLL_PM_OPTM_EN
            if (tpsll_pm_optm_is_long_suspend_support_level_active()) {
                pp_rf_start_srx(stimer_get_tick(), KB_CONNECTED_RX_TIMEOUT + TPSLL_PM_OPTM_8K_RX_WIN_EXPAND_US * 2);
            } else
            #endif
            {
                pp_rf_start_srx(stimer_get_tick(), KB_CONNECTED_RX_TIMEOUT);
            }
            ll_ctx.rx_over = false;
            ll_ctx.cur_rx_flag = true;
            #endif
        }
        // SLOT3 start
        ll_ctx.task_slot = TASK_SLOT3;
        // KB RX

        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        if (!(pp_fifo_is_empty(km_fifo_ptr) && pp_fifo_is_empty(spp_fifo_ptr) && pp_fifo_is_empty(&ll_fifo))) {
            ll_ctx.kb_alive_intervals_cnt = 0;
            ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
        }
        if (tpsll_is_keep_alive_active()) { //keep alive state
            ll_ctx.kb_alive_intervals_cnt = 0;
            ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
        } else {
            uint32_t kb_alive_time_steps = ll_ctx.kb_alive_intervals_cnt >> ll_ctx.kb_alive_16ms_intervals_shift;

            if (kb_alive_time_steps < (TPSLL_KB_ALIVE_MAX_STAGED_TIME_MS / TPSLL_KB_ALIVE_TIME_STEP_MS)) {
                if ((ll_ctx.kb_alive_intervals_cnt & ll_ctx.kb_alive_16ms_intervals_msk) == 0) {
                    if (ll_ctx.kb_active_stage1_start_time_steps && (kb_alive_time_steps == ll_ctx.kb_active_stage1_start_time_steps)) {
                        ll_ctx.kb_alive_fno_msk = TPSLL_KB_8K_ALIVE_STAGE1_FNO_MASK;
                    }
                    else if (ll_ctx.kb_active_stage2_start_time_steps && (kb_alive_time_steps == ll_ctx.kb_active_stage2_start_time_steps)) {
                        ll_ctx.kb_alive_fno_msk = TPSLL_KB_8K_ALIVE_STAGE2_FNO_MASK;
                    }
                    else if (ll_ctx.kb_active_stage3_start_time_steps && (kb_alive_time_steps == ll_ctx.kb_active_stage3_start_time_steps)) {
                        ll_ctx.kb_alive_fno_msk = TPSLL_KB_8K_ALIVE_STAGE3_FNO_MASK;
                    }
                    else if (ll_ctx.kb_active_stage4_start_time_steps && (kb_alive_time_steps == ll_ctx.kb_active_stage4_start_time_steps)) {
                        ll_ctx.kb_alive_fno_msk = TPSLL_KB_8K_ALIVE_STAGE4_FNO_MASK;
                    }
                    else if (ll_ctx.kb_active_stage5_start_time_steps && (kb_alive_time_steps == ll_ctx.kb_active_stage5_start_time_steps)) {
                        ll_ctx.kb_alive_fno_msk = TPSLL_KB_8K_ALIVE_STAGE5_FNO_MASK;
                    }
                    #ifdef TPSLL_KB_8K_ALIVE_STAGE6_FNO_MASK
                    else if (ll_ctx.kb_active_stage6_start_time_steps && (kb_alive_time_steps == ll_ctx.kb_active_stage6_start_time_steps)) {
                        ll_ctx.kb_alive_fno_msk = TPSLL_KB_8K_ALIVE_STAGE6_FNO_MASK;
                    }
                    #endif
                }
                ll_ctx.kb_alive_intervals_cnt++;
            }
        }
        #endif /* end of TPSLL_KB_ACTIVE_PM_OPTM_EN */
    }
}

_attribute_ram_code_sec_ void tpsll_timer1_irq_non_8k_handler(void)
{
    {
        #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
        if (ll_ctx.ks_need_disable) {
            if (!ll_ctx.key_scan_disable) {
                mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_DISABLE_TRIGGER, NULL);
                ll_ctx.key_scan_disable = true;
            }
            ll_ctx.ks_need_disable = false;

            if (ll_ctx.report_rate & (REPORT_RATE_2K | REPORT_RATE_1K | REPORT_RATE_500)) {
                pp_timer1_start(KM_SLOT1_TIME_US_500_1K_2K * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));
            }
            else if (ll_ctx.report_rate & REPORT_RATE_4K) {
                pp_timer1_start(127 * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT1

                if (!tpsll_all_fifo_is_empty()) {
                    tpsll_pm_optm_time_refresh();
                }
            }
            return;
        } else if (ll_ctx.ks_need_enable) {
            if (ll_ctx.key_scan_disable) {
                mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_ENABLE_TRIGGER, NULL);
                ll_ctx.key_scan_disable = false;
            }
            ll_ctx.ks_need_enable = false;
            return;
        }
        #endif
        ll_ctx.task_slot++;//stimer is the 1st slot

        if(ll_ctx.report_rate & REPORT_RATE_4K) {
            if (tpsll_is_rf_busy() && (ll_ctx.srx_task_slot == TASK_SLOT1)) {
                rf_set_tx_rx_off_auto_mode();
                ll_ctx.rx_over = true;
            }
        }

        if (ll_ctx.rx_over)
        {
            if (wait_rf_fsm_idle_timeout(3)) // 8k: after TX_EN 4K: 2us
            {
                rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
            }

            app_non_8k_rf_pkt_send();
        }
        else if ((ll_ctx.report_rate & REPORT_RATE_125) && (ll_ctx.task_slot >= (ll_ctx.max_slot - 1)))
        {
            ll_ctx.suspend_flag = true;
        }

        #if TPSLL_NEW_KEEP_ALIVE_EN
        #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN && TPSLL_PM_OPTM_N8K_SUSPEND_EN
        if (ll_ctx.task_slot < (ll_ctx.max_slot - 1))
        #else
        if ((ll_ctx.task_slot < (ll_ctx.max_slot - 1)) && ll_ctx.is_keep_alive_sync)
        #endif
        #else
        if (ll_ctx.task_slot < (ll_ctx.max_slot - 1))
        #endif
        {
            unsigned int r = core_interrupt_disable();
            if (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250))
            {
                pp_timer1_start((SLOT1_OFFSET_TIME_US_125 + (ll_ctx.task_slot) * SLOT_TIME_US_125) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));
            }
            else if (ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500))
            {
                pp_timer1_start(((ll_ctx.task_slot + 1) * KM_SLOT_TIME_US_500_1K_2K + 10) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));
            }
            else
            {
                pp_timer1_start((ll_ctx.task_slot + 1) * KM_SLOT_TIME_US_500_1K_2K * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT2
            }
            core_restore_interrupt(r);
        }
        else if ((ll_ctx.report_rate & REPORT_RATE_250) && ll_ctx.is_alg_kb) {
            pp_timer1_start(ll_ctx.stick - stimer_get_tick() - TPSLL_PM_OPTM_250_NEED_ENABLE_KS_EARLY_TIME_US * RF_SYSTEM_TIMER_TICK_1US);
            ll_ctx.ks_need_enable = true;
        }
    }
}

#if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
static inline void tpsll_timer1_irq_idle_mode_handler(void)
{
    #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
    if (ll_ctx.ks_need_enable) {
        if (ll_ctx.key_scan_disable) {
            mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_ENABLE_TRIGGER, NULL);
            ll_ctx.key_scan_disable = false;
        }
        ll_ctx.ks_need_enable = false;
        return;
    }
    else if (ll_ctx.is_alg_kb && !ll_ctx.key_scan_disable) {
        mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_DISABLE_TRIGGER, NULL);
        ll_ctx.key_scan_disable = true;
    }
    #endif
    ll_ctx.suspend_flag = true;
}
#endif

static inline void tpsll_timer1_irq_reconn_handler(void)
{
    #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
    if (ll_ctx.ks_need_disable) {
        mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_DISABLE_TRIGGER, NULL);
        ll_ctx.key_scan_disable = true;
        ll_ctx.ks_need_disable = false;

        pp_timer1_start(105 * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));
        return;
    }
    #endif
    if (ll_ctx.recon_step == RECON_STEP1) {
        if (!wait_rf_fsm_idle_timeout(1)) {
            #if !TPSLL_RECON_RSSI_SCAN_EN
            rf_set_ble_chn((ll_ctx.reconn_chn + 13) & 0x3F);
            #endif
            pp_rf_start_srx(stimer_get_tick(), KM_PAIRING_SRX_TIMEOUT_US + 5);
        }
        pp_timer1_start((160) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));

        ll_ctx.recon_step = RECON_STEP2;
    } else if (ll_ctx.recon_step == RECON_STEP2) {
        if (!wait_rf_fsm_idle_timeout(1)) {
            #if !TPSLL_RECON_RSSI_SCAN_EN
            rf_set_ble_chn((ll_ctx.reconn_chn + 2 * 13) & 0x3F);
            #endif
            pp_rf_start_srx(stimer_get_tick(), KM_PAIRING_SRX_TIMEOUT_US + 5);
        }
        pp_timer1_stop();

        ll_ctx.recon_step = RECON_STEP_NONE;
    }
}

static inline void tpsll_timer1_irq_pairing_handler(void)
{
    #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
    if (ll_ctx.ks_need_disable) {
        mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_DISABLE_TRIGGER, NULL);
        ll_ctx.key_scan_disable = true;
        ll_ctx.ks_need_disable = false;
        return;
    }
    #endif

    if (ll_ctx.pair_step < RECON_STEP3) {
        if (!wait_rf_fsm_idle_timeout(1))
        {
            pp_rf_start_srx(stimer_get_tick(), KM_PAIRING_SRX_TIMEOUT_US + 5);
        }

        ll_ctx.pair_step++;
    } else {
        ll_ctx.pair_step = PAIR_STEP_NONE;
    }

    if (ll_ctx.is_alg_kb && !ll_ctx.key_scan_disable) {
        mcc_n22_mb_send_data(TLK_MB_D25F_TO_N22_KS_DISABLE_TRIGGER, NULL);
        ll_ctx.key_scan_disable = true;
    }
}

_attribute_ram_code_sec_ uint8_t tpsll_timer1_irq_handler(void)
{
    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
    // ZW_TG(GPIO_PG7);

    if (timer_get_irq_status(FLD_TMR1_MODE_IRQ))
    {
        pp_timer1_stop();

        if(ll_ctx.stk_state == STATE_CONNECTED)
        {
            if (ll_ctx.report_rate & REPORT_RATE_8K)
            {
                tpsll_timer1_irq_8k_handler();
            }
            else
            {
                tpsll_timer1_irq_non_8k_handler();
            }
        }
        #if TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN
        else if(ll_ctx.stk_state == STATE_IDLE)
        {
            tpsll_timer1_irq_idle_mode_handler();
        }
        #endif /* end of TPSLL_PM_OPTM_EN && TPSLL_PM_OPTM_KEEP_ALIVE_DISABLE_KS_IN_IRQ_EN */
        #if TPSLL_NEW_RECON_EN
        else if(ll_ctx.stk_state == STATE_RECONNECT)
        {
            tpsll_timer1_irq_reconn_handler();
        }
        else if(ll_ctx.stk_state == STATE_PAIRING)
        {
            tpsll_timer1_irq_pairing_handler();
        }
        #endif /* end of TPSLL_NEW_RECON_EN */
        timer_clr_irq_status(FLD_TMR1_MODE_IRQ); // clear irq status
    }
    ll_ctx.wfi_flag = WFI_TRUE;
    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ uint8_t  tpsll_change_report_rate(report_rate_t rr)
{
    #define CMD_RR_SIZE 5
    uint8_t cmd[CMD_RR_SIZE] = {0};

    if((ll_ctx.report_rate == rr) || (!tpsll_is_report_rate_valid(rr)))
    {
        return TLK_ERR_NULL;
    }

    if (ll_ctx.stk_state != STATE_CONNECTED) {
        return TLK_ERR_INVALID_STATE;
    }

    #if TPSLL_FHSS_EN
    if (ll_ctx.pending_rr && (ll_ctx.pending_rr != rr)) {
        return TLK_ERR_BUSY;
    }
    if (g_fhss_idx_buf_data_cur != g_fhss_idx_buf_data_next) {
        ll_ctx.pending_rr = rr;
        return TLK_SUCCESS;
    }
    #endif

    if (!tpsll_is_active_state()) {
        ll_ctx.pending_rr = rr;
        tpsll_pm_optm_time_refresh();
        return TLK_SUCCESS;
    }

    tpsll_suspend_inter_en(false);

    unsigned int r = core_interrupt_disable();
    uint16_t target_fno = ll_ctx.fno_last + SWITCH_DELAY_INTERVAL;

    ll_ctx.user_rr = rr;

    #if TPSLL_RR_CHG_WITH_RECON_EN
    if (!ll_ctx.exit_spp_mode) {
        ll_ctx.rr_chg_with_recon = RR_RECON_STEP1;
        cmd[0] = TPSLL_LL_CMD_RR_CHG_RCNN_REQ;
    } else {
        ll_ctx.exit_spp_mode = 0;
        cmd[0] = TPSLL_LL_CMD_RR_CHG_REQ;
    }
    #else
    cmd[0] = TPSLL_LL_CMD_RR_CHG_REQ;
    #endif

    cmd[1] = rr;
    cmd[2] = target_fno & 0xff;
    cmd[3] = (target_fno >> 8) & 0xff;
    cmd[4] = 0;
    ll_ctx.pending_rr_fno = target_fno;
    ll_ctx.chg_rr = rr;
    ll_ctx.is_rr_changing = true;

    pp_fifo_push(&ll_fifo, TPSLL_LL_CMD, cmd, CMD_RR_SIZE); // push to fifo
    ll_ctx.is_rr_chg_acked = false;
    core_restore_interrupt(r);

    #if TPSLL_KEEP_ALIVE_EN
    ll_ctx.synced = 0;
    #if TPSLL_PM_OPTM_EN
    ll_ctx.pm_optm_rr_chgd_ref_fno_cnt_th = KEEP_ALIVE_DELAY_US / ll_ctx.interval_us;
    #else
    ll_ctx.alive_delay_time = KEEP_ALIVE_DELAY_US;
    #endif
    tpsll_pm_optm_time_refresh();

    #endif

    #if TPSLL_FHSS_EN
    ll_ctx.pending_rr_spp = 0;
    ll_ctx.pending_rr = 0;
    #endif

    return 0;
}

static inline void tpsll_change_report_rate_check(void)
{
    #if TPSLL_FHSS_EN
    if (ll_ctx.stk_state == STATE_CONNECTED) {
        if ((g_fhss_idx_buf_data_cur == g_fhss_idx_buf_data_next) && ll_ctx.pending_rr) {
            tpsll_change_report_rate(ll_ctx.pending_rr);
        }
    }
    #endif
}

_attribute_ram_code_sec_ uint8_t  p24g_change_slot(p24g_slot_mode_e sm)
{
    #define CMD_SM_SIZE 4
    uint8_t cmd[CMD_SM_SIZE] = {0};

    if(ll_ctx.used_sm == sm)
    {
        return 1;
    }

    unsigned int r = core_interrupt_disable();
    uint16_t target_fno = ll_ctx.fno_last + SWITCH_DELAY_INTERVAL;
    

    cmd[0] = TPSLL_LL_CMD_SLOT_CHG_REQ;
    cmd[1] = sm;
    cmd[2] = target_fno & 0xff;
    cmd[3] = (target_fno >> 8) & 0xff;
    ll_ctx.pending_sm_fno = target_fno;
    ll_ctx.chg_rr = sm;
    ll_ctx.is_sm_changing = true;

    pp_fifo_push(&ll_fifo, TPSLL_LL_CMD, cmd, CMD_SM_SIZE); // push to fifo
    ll_ctx.is_sm_chg_acked = false;
    core_restore_interrupt(r);

    return 0;
}

_attribute_ram_code_sec_ uint8_t tpsll_rf_control(bool enable)
{
    if(enable)
    {
        if(!ll_ctx.stimer_running)
        {
             tpsll_stimer_init();
        }
    }
    else
    {
        if(ll_ctx.stimer_running)
        {
            #if defined(MCU_CORE_TL322X_N22)
            clic_interrupt_disable(IRQ_SYSTIMER);
            #else
            plic_interrupt_disable(IRQ_SYSTIMER);
            #endif
            pp_timer1_stop();
            rf_status = IDLE_RF_STATUS;
            ll_ctx.stimer_running = 0;
        }
    }
    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ uint8_t tpsll_pm_control(bool enable)
{
    if(enable)
    {
        ll_ctx.allow_enter_sleep = 1;
    }
    else
    {
        ll_ctx.allow_enter_sleep = 0;
    }
    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ uint8_t tpsll_set_timeout_ms(tpsll_timeout_type_e timeout_type, uint32_t timeout_ms)
{
    if (timeout_ms < TPSLL_TIMEOUT_MIN_MS)
    {
        timeout_ms = TPSLL_TIMEOUT_MIN_MS;
        tlkapi_printf(APP_LOG_EN, "Interval too short, the minimum timeout period is %dms \n", TPSLL_TIMEOUT_MIN_MS);
        return TLK_ERR_INVALID_PARAM;
    }
    else if (timeout_ms > TPSLL_TIMEOUT_MAX_MS)
    {
        timeout_ms = TPSLL_TIMEOUT_MAX_MS;
        tlkapi_printf(APP_LOG_EN, "Interval too long, the maximum timeout period is %dms \n", TPSLL_TIMEOUT_MAX_MS);
        return TLK_ERR_INVALID_PARAM;
    }

    if (timeout_type == P24G_TIMEOUT_TYPE_PAIR)
    {
        u32 timeout_cnt = (u32)(timeout_ms * 4);
        ll_ctx.pair_timeout_cnt = timeout_cnt;
    }
    else if (timeout_type == P24G_TIMEOUT_TYPE_DISCONN)
    {
        u32 timeout_cnt = (u32)(timeout_ms * 1000);
        ll_ctx.disconnect_timeout_us = timeout_cnt;
    }
    else if (timeout_type == P24G_TIMEOUT_TYPE_RECONN)
    {
        u32 timeout_cnt = (u32)(timeout_ms * 4);
        ll_ctx.reconnect_timeout_cnt = timeout_cnt;
    }
    else if (timeout_type == P24G_TIMEOUT_TYPE_NOACTIVE)
    {
        u32 timeout_cnt = (u32)(timeout_ms * 4);
        ll_ctx.no_event_cnt_max_set = timeout_cnt;
    }

    return TLK_SUCCESS;
}

#if !TPSLL_PM_OPTM_EN
uint8_t tpsll_set_secondary_active_s(uint32_t timeout_s)
{
    if(timeout_s<1 || timeout_s>3600)
    {
        return TLK_ERR_INVALID_PARAM;
    }
    ll_ctx.secondary_active_timeout = timeout_s;
    return TLK_SUCCESS;
}
#endif

uint8_t tpsll_pm_optm_delay_time_set(tpsll_pm_optm_level_e optm_level, uint32_t delay_time_s)
{
    #if TPSLL_PM_OPTM_EN
    if (delay_time_s > TPSLL_PM_OPTM_START_TIME_MAX_S)
    {
        return TLK_ERR_INVALID_PARAM;
    }

    switch(optm_level)
    {
        case TPSLL_PM_OPTM_LEVEL1:
            ll_ctx.pm_optm_1st_start_time_ms = delay_time_s * 1000;
            ll_ctx.pm_optm_1st_ref_fno_cnt_th = ll_ctx.pm_optm_1st_start_time_ms * 1000 / ll_ctx.interval_us;

            if (ll_ctx.report_rate & REPORT_RATE_8K) {
                ll_ctx.pm_optm_rr_chgd_ref_fno_cnt_th = ((ll_ctx.pm_optm_1st_start_time_ms * 1000 >= KEEP_ALIVE_SWITCH_8K_DELAY_US) ?
                                                        ll_ctx.pm_optm_1st_ref_fno_cnt_th : (KEEP_ALIVE_SWITCH_8K_DELAY_US / ll_ctx.interval_us));
            } else {
                ll_ctx.pm_optm_rr_chgd_ref_fno_cnt_th = ((ll_ctx.pm_optm_1st_start_time_ms * 1000 >= KEEP_ALIVE_DELAY_US) ?
                                                        ll_ctx.pm_optm_1st_ref_fno_cnt_th : (KEEP_ALIVE_DELAY_US / ll_ctx.interval_us));
            }
            break;
        case TPSLL_PM_OPTM_LEVEL2:
            ll_ctx.pm_optm_2nd_start_time_ms = delay_time_s * 1000;
            ll_ctx.pm_optm_2nd_ref_fno_cnt_th = ll_ctx.pm_optm_2nd_start_time_ms * 1000 / ll_ctx.interval_us;
            break;
        case TPSLL_PM_OPTM_LEVEL3:
            ll_ctx.pm_optm_3rd_start_time_ms = delay_time_s * 1000;
            ll_ctx.pm_optm_3rd_ref_fno_cnt_th = ll_ctx.pm_optm_3rd_start_time_ms * 1000 / ll_ctx.interval_us;
            if (ll_ctx.pm_optm_4th_start_time_ms  > ll_ctx.pm_optm_3rd_start_time_ms) {
                ll_ctx.pm_optm_4th_ref_fno_cnt_delta = (ll_ctx.pm_optm_4th_start_time_ms - ll_ctx.pm_optm_3rd_start_time_ms) / ll_ctx.pm_optm_3rd_interval_ms;
            } else {
                ll_ctx.pm_optm_4th_ref_fno_cnt_delta = 0; // no level4
            }
            break;
        case TPSLL_PM_OPTM_LEVEL4:
            // ll_ctx.pm_optm_4th_start_time_ms = delay_time_s * 1000;
            // if (ll_ctx.pm_optm_4th_start_time_ms  > ll_ctx.pm_optm_3rd_start_time_ms) {
            //     ll_ctx.pm_optm_4th_ref_fno_cnt_delta = (ll_ctx.pm_optm_4th_start_time_ms - ll_ctx.pm_optm_3rd_start_time_ms) / ll_ctx.pm_optm_3rd_interval_ms;
            // } else {
            //     ll_ctx.pm_optm_4th_ref_fno_cnt_delta = 0;
            // }
            // tlkapi_printf(APP_LOG_EN, "4thD %d\n", ll_ctx.pm_optm_4th_ref_fno_cnt_delta);
            break;
        default:
            tlkapi_printf(APP_LOG_EN, "optm level invalid (0x%x)\n", optm_level);
            return TLK_ERR_INVALID_PARAM;
    }

    tlkapi_printf(APP_LOG_EN, "optm 0x%x delay time %d s\n", optm_level, delay_time_s);

    return TLK_SUCCESS;
    #else
    return TLK_ERR_CMD_NOT_SUPPORT;
    #endif /* end of TPSLL_PM_OPTM_EN */
}

uint8_t tpsll_pm_optm_level3_interval_time_set(uint32_t interval_ms)
{
    #if TPSLL_PM_OPTM_EN
    if (interval_ms < TPSLL_PM_OPTM_IDLE_INTERVAL_MIN_MS || interval_ms > TPSLL_PM_OPTM_IDLE_INTERVAL_MAX_MS)
    {
        return TLK_ERR_INVALID_PARAM;
    }

    ll_ctx.pm_optm_3rd_interval_ms = interval_ms;

    if (ll_ctx.pm_optm_4th_ref_fno_cnt_delta && (ll_ctx.pm_optm_4th_start_time_ms  > ll_ctx.pm_optm_3rd_start_time_ms)) {
        ll_ctx.pm_optm_4th_ref_fno_cnt_delta = (ll_ctx.pm_optm_4th_start_time_ms - ll_ctx.pm_optm_3rd_start_time_ms) / ll_ctx.pm_optm_3rd_interval_ms;
    } else {
        ll_ctx.pm_optm_4th_ref_fno_cnt_delta = 0;
    }

    if (ll_ctx.pm_optm_level & TPSLL_PM_OPTM_LEVEL3) {
        ll_ctx.interval_us = interval_ms * 1000;
    }

    return TLK_SUCCESS;
    #else
    return TLK_ERR_CMD_NOT_SUPPORT;
    #endif /* end of TPSLL_PM_OPTM_EN */
}

uint8_t tpsll_pm_optm_level4_interval_time_set(uint32_t interval_ms)
{
    #if TPSLL_PM_OPTM_EN
    if (interval_ms < TPSLL_PM_OPTM_IDLE_INTERVAL_MIN_MS || interval_ms > TPSLL_PM_OPTM_IDLE_INTERVAL_MAX_MS)
    {
        return TLK_ERR_INVALID_PARAM;
    }

    ll_ctx.pm_optm_4th_interval_ms = interval_ms;

    if (ll_ctx.pm_optm_level & TPSLL_PM_OPTM_LEVEL4) {
        ll_ctx.interval_us = interval_ms * 1000;
    }

    return TLK_SUCCESS;
    #else
    return TLK_ERR_CMD_NOT_SUPPORT;
    #endif /* end of TPSLL_PM_OPTM_EN */
}

uint8_t tpsll_request_sleep(void)
{
    tlkapi_printf(APP_LOG_EN, "slp req\n");
    ll_ctx.sleep_req = true;
    ll_ctx.reconn_from_request_sleep = true;
    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ report_rate_t tpsll_get_report_rate(void)
{
    return ll_ctx.report_rate;
}

_attribute_ram_code_sec_ uint8_t tpsll_send_spp_data(uint8_t *data, uint8_t len, uint8_t retry_num, spp_cb_t cb, void *user_arg)
{
    uint8_t ret = 0;

    if(ll_ctx.is_rr_changing || (g_state != STATE_CONNECTED) || (ll_ctx.stk_state != STATE_CONNECTED))
    {
        return TLK_ERR_INVALID_STATE;
    }
    if(!data)
    {
        return TLK_ERR_NULL;
    }
    if((len > (KM_SPP_MAX_LEN - 1)) || (len == 0))
    {
        return TLK_ERR_INVALID_LENGTH;
    }

    if (!ll_ctx.allow_spp)
    {
        if (clock_time_exceed(ll_ctx.connected_tick, 50 * 1000))
        {
            ll_ctx.allow_spp = true;
        }
        else
        {
            return TLK_ERR_BUSY;
        }
    }

    unsigned int r = core_interrupt_disable();
    ret = pp_fifo_push_extra(spp_fifo_ptr, TPSLL_SPP_DATA, data, len, retry_num, cb, user_arg);
    core_restore_interrupt(r);

    tpsll_event_active_time_fresh_value(clock_time());

    return ret;
}

_attribute_ram_code_sec_ uint8_t tpsll_send_keyboard_data(uint8_t *data, uint8_t len, uint8_t cmd, kb_cb_t cb, void *user_arg)
{
    (void)cb;
    (void)user_arg;
    if(!data)
    {
        return TLK_ERR_NULL;
    }

    if (cmd == NORMAL_KB_DATA_CMD)
    {
        tpsll_event_active_time_fresh_value(clock_time());
        return  pp_fifo_push(km_fifo_ptr, NORMAL_KB_DATA_CMD, data, len);
    }
    else if (cmd == CONSUME_KB_DATA_CMD)
    {
        return  pp_fifo_push(km_fifo_ptr, CONSUME_KB_DATA_CMD, data, len);
    }
    else if (cmd == SYSTEM_KB_DATA_CMD)
    {
        return  pp_fifo_push(km_fifo_ptr, SYSTEM_KB_DATA_CMD, data, len);
    }
    else if (cmd == ALL_KB_DATA_CMD)
    {
        return  pp_fifo_push(km_fifo_ptr, ALL_KB_DATA_CMD, data, len);
    }
    return TLK_SUCCESS;
}

static inline void tpsll_idle_wakeup_check(void)
{
    if (ll_ctx.stk_state == STATE_IDLE) {
        #if TPSLL_PM_OPTM_EN
        if (!pp_fifo_is_empty(km_fifo_ptr) || !ll_ctx.pm_optm_3rd_ref_fno_cnt_th || (ll_ctx.pm_optm_ref_fno_cnt < TPSLL_PM_OPTM_LEVEL3_REF_FNO_CNT_START))
        #else
        if (!pp_fifo_is_empty(km_fifo_ptr))
        #endif
        {
            tpsll_rf_init();
            if (ll_inf.peer_tag) {
                tpsll_enable_reconn(true);
            }
            else {
                tpsll_enable_pairing(true, pairing_tx_power);
            }
            // stack_evt_post_from_isr(TPSLL_EVT_PM_OPTM_EXIT, 0, 0);
            s_app_cb->app_p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, TPSLL_EVT_PM_OPTM_EXIT, 0, 0);
            ll_ctx.suspend_flag = false;
            ll_ctx.stick = clock_time() + 100 * SYSTEM_TIMER_TICK_1US;
            systimer_set_irq_capture(ll_ctx.stick);
        }
    }
}

static inline void app_2p4g_sleep_check_loop(void)
{
    static uint8_t low_power_prepare_loop_cnt = APP_LOW_POWER_PREPARE_LOOP_CNT;

    if (ll_ctx.suspend_flag && !ll_ctx.allow_enter_sleep && ll_ctx.is_alg_kb) {
        ll_ctx.suspend_flag = false;
        if ((ll_ctx.stk_state == STATE_CONNECTED) && (ll_ctx.report_rate & (REPORT_RATE_2K | REPORT_RATE_1K | REPORT_RATE_500 | REPORT_RATE_250 | REPORT_RATE_125))) {
            pp_timer1_start((ll_ctx.stick - stimer_get_tick()) - TPSLL_PM_OPTM_IN_CHRG_L_CLK_NEED_ENABLE_KS_EARLY_TIME_US * RF_SYSTEM_TIMER_TICK_1US);
        } else {
            pp_timer1_start((ll_ctx.stick - stimer_get_tick()) - TPSLL_PM_OPTM_IN_CHRG_H_CLK_NEED_ENABLE_KS_EARLY_TIME_US * RF_SYSTEM_TIMER_TICK_1US);
        }
        ll_ctx.ks_need_enable = true;
    } else
    if (ll_ctx.suspend_en && ll_ctx.suspend_flag && ll_ctx.allow_enter_sleep)
    {
        uint8_t short_suspend_flag = 1;
        uint8_t data[2] = {0, 0};

        if (low_power_prepare_loop_cnt) {
            low_power_prepare_loop_cnt--;
            return;
        }


        ll_ctx.suspend_flag = false;
        low_power_prepare_loop_cnt = APP_LOW_POWER_PREPARE_LOOP_CNT;

        #if TPSLL_SUSPEND_EN
        if (ll_ctx.pm_optm_level & TPSLL_PM_OPTM_LEVEL4) {
            if (tick1_exceed_tick2(ll_ctx.stick, stimer_get_tick() + TPSLL_DEEP_RET_LIMIT_MIN_TIME_US * SYSTEM_TIMER_TICK_1US)) {
                ll_ctx.is_rf_inited = false;
#ifdef N22_ZEPHYR_SUPPORT
                *ll_ctx.next_wakeup_tick_ptr =  ll_ctx.stick - TPSLL_WAKEUP_EARLY_FOR_DEEP_RET_TIME_US * SYSTEM_TIMER_TICK_1US;
                *ll_ctx.suspend_allowed_ptr = true;
#else

#endif
            }
        }

        if (!ll_ctx.key_scan_disable && ll_ctx.is_alg_kb) {
            data[0] = 1;
            ll_ctx.key_scan_disable = true;
        }
        p24g_evt_cb(TPSLL_EVT_ENTER_SUSPEND, data, 1);

        if (tick1_exceed_tick2(ll_ctx.stick, stimer_get_tick() + APP_SUSPEND_LIMIT_MIN_TIME_US * SYSTEM_TIMER_TICK_1US)) {
            #if APP_SUSPEND_LONG_TIME_MIN_US
            if (tick1_exceed_tick2(ll_ctx.stick, stimer_get_tick() + APP_SUSPEND_LONG_TIME_MIN_US * SYSTEM_TIMER_TICK_1US)) {
                short_suspend_flag = 0;
            }
            #endif
#ifdef N22_ZEPHYR_SUPPORT
            *ll_ctx.next_wakeup_tick_ptr = ll_ctx.stick
                                    - ((short_suspend_flag ? ll_ctx.suspend_wakeup_early_time_us : APP_WAKEUP_EARLY_FOR_LONG_SUSPEND_TIME_US)) * SYSTEM_TIMER_TICK_1US;
            *ll_ctx.suspend_allowed_ptr = true;
            core_entry_wfi_mode();
#else
            
#endif

        } else if (ll_ctx.wfi_en && ll_ctx.wfi_inter_en) {
            *ll_ctx.suspend_allowed_ptr = false;
            core_entry_wfi_mode();
        }

        if (ll_ctx.key_scan_disable) {
            data[0] = 1;
            ll_ctx.key_scan_disable = false;
        } else {
            data[0] = 0;
        }
        p24g_evt_cb(TPSLL_EVT_EXIT_SUSPEND, data, 1);
        #else
        *ll_ctx.suspend_allowed_ptr = false;
        core_entry_wfi_mode();
        #endif
    } else if (ll_ctx.wfi_en && ll_ctx.wfi_inter_en && ll_ctx.wfi_flag)
    {
        if (low_power_prepare_loop_cnt) {
            low_power_prepare_loop_cnt--;
            return;
        }

        ll_ctx.wfi_flag = WFI_FALSE;
        low_power_prepare_loop_cnt = APP_LOW_POWER_PREPARE_LOOP_CNT;
// DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX2], 0);
        *ll_ctx.suspend_allowed_ptr = false;
        core_entry_wfi_mode();
// DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX2], 1);
    }
}

_attribute_ram_code_sec_ uint8_t tpsll_stack_main_loop(void)
{
    app_2p4g_sleep_check_loop();
    tpsll_change_report_rate_check();
    tpsll_idle_wakeup_check();

    /* process up to N events per loop to avoid starving other tasks (0 = process all) */
    stack_poll_events(0);

    #if TPSLL_KEEP_ALIVE_EN
    tpsll_event_active_timeout_check();
    #endif

    #if TPSLL_RECON_RSSI_SCAN_EN && !TPSLL_NEW_RECON_EN
    if (ll_ctx.recon_rssi_scanning_cnt > 2) {
        rf_rssi_scan();
        ll_ctx.recon_rssi_scanning_cnt = 0;

        tpsll_enable_reconn(true);
    }
    #endif
    return TLK_SUCCESS;
}

uint8_t tpsll_stack_debug_io_init(gpio_pin_e io0, gpio_pin_e io1, gpio_pin_e io2, gpio_pin_e io3, gpio_pin_e io4, gpio_pin_e io5, gpio_pin_e io6, gpio_pin_e io7)
{
    dbg_io[DBG_IO_IDX0] = io0;
    dbg_io[DBG_IO_IDX1] = io1;
    dbg_io[DBG_IO_IDX2] = io2;
    dbg_io[DBG_IO_IDX3] = io3;
    dbg_io[DBG_IO_IDX4] = io4;
    dbg_io[DBG_IO_IDX5] = io5;
    dbg_io[DBG_IO_IDX6] = io6;
    dbg_io[DBG_IO_IDX7] = io7;

    for (uint8_t idx = 0; idx < DBG_IO_IDX_MAX; idx++) {
        if (dbg_io[idx]) {
            gpio_function_en(dbg_io[idx]);
            gpio_output_en(dbg_io[idx]);
            gpio_input_dis(dbg_io[idx]);
            DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[idx]);
            DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[idx]);
        }
    }
    return TLK_SUCCESS;
}

tpsll_dev_status_e tpsll_get_state(void)
{
    return g_state;
}

int tpsll_push_kmfifo(uint8_t cmd, uint8_t *buf, uint8_t len)
{
    return pp_fifo_push(km_fifo_ptr, cmd, buf, len);
}

uint8_t tpsll_pop_kmfifo(void)
{
    pp_fifo_pop(km_fifo_ptr);
    return TLK_SUCCESS;
}

uint8_t tpsll_reset_kmfifo(void)
{
    pp_fifo_reset(km_fifo_ptr);
    return TLK_SUCCESS;
}

uint8_t * tpsll_get_ptr_kmfifo(void)
{
    return pp_fifo_get_ptr(km_fifo_ptr);
}

uint16_t tpsll_get_num_kmfifo(void)
{
    return pp_fifo_get_num(km_fifo_ptr);
}

const uint8_t* tpsll_get_stack_version(void)
{
    return tpsll_stack_version;
}

uint8_t tpsll_set_pairing_access_code(uint32_t ac)
{
    if (!is_valid_access_code(ac)) {
        return TLK_ERR_INVALID_PARAM;
    }
    ll_ctx.pair_acc = ac;
    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ uint8_t tpsll_kb_active_pm_optm_level_start_time_set(tpsll_kb_active_pm_level_e stage, uint32_t stage_start_time_ms)
{
    uint32_t start_time_steps = stage_start_time_ms / TPSLL_KB_ALIVE_TIME_STEP_MS;

    if (start_time_steps < (TPSLL_KB_ALIVE_MIN_STAGED_TIME_MS / TPSLL_KB_ALIVE_TIME_STEP_MS) ||
        (start_time_steps >= (TPSLL_KB_ALIVE_MAX_STAGED_TIME_MS / TPSLL_KB_ALIVE_TIME_STEP_MS))) {
        return TLK_ERR_INVALID_PARAM;
    }

    #if TPSLL_KB_ACTIVE_PM_OPTM_EN
    switch (stage)
    {
    case TPSLL_KB_ACTIVE_PM_OPTM_LEVEL1:
        ll_ctx.kb_active_stage1_start_time_steps = start_time_steps;
        break;
    case TPSLL_KB_ACTIVE_PM_OPTM_LEVEL2:
        ll_ctx.kb_active_stage2_start_time_steps = start_time_steps;
        break;
    case TPSLL_KB_ACTIVE_PM_OPTM_LEVEL3:
        ll_ctx.kb_active_stage3_start_time_steps = start_time_steps;
        break;
    case TPSLL_KB_ACTIVE_PM_OPTM_LEVEL4:
        ll_ctx.kb_active_stage4_start_time_steps = start_time_steps;
        break;
    case TPSLL_KB_ACTIVE_PM_OPTM_LEVEL5:
        ll_ctx.kb_active_stage5_start_time_steps = start_time_steps;
        break;
    #ifdef TPSLL_KB_8K_ALIVE_STAGE6_FNO_MASK
    case KB_ALIVE_STAGE6:
        ll_ctx.kb_active_stage6_start_time_steps = start_time_steps;
        break;
    #endif
    default:
        return TLK_ERR_INVALID_PARAM;
    }
    #endif /* end of TPSLL_KB_ACTIVE_PM_OPTM_EN */

    return TLK_SUCCESS;
}

uint8_t tpsll_rf_init(void)
{
    if (ll_ctx.is_rf_inited) {
        return TLK_ERR_INVALID_STATE;
    } else {
        pp_rf_init();
        ll_ctx.is_rf_inited = true;
        return TLK_SUCCESS;
    }
}

uint8_t tpsll_stack_init(bool is_alg_kb, report_rate_t rr, tpsll_stack_config_t *p_config)
{
    tlkapi_printf(APP_LOG_EN, "tpsll_stack_init ..\n");

    if (!p_config)
    {
        tlkapi_printf(APP_LOG_EN, "[TPSLL] have null cb, init failed!\n");
        return TLK_ERR_NULL;
    }
    s_app_cb = &p_config->app_cb;
    km_fifo_ptr = (pl_fifo_t *)p_config->km_fifo_ptr;
    spp_fifo_ptr = (pl_fifo_t *)p_config->spp_fifo_ptr;
    ll_ctx.is_stk_busy_ptr = p_config->is_stk_busy_ptr;
    ll_ctx.next_wakeup_tick_ptr = p_config->next_wakeup_tick_ptr;
    ll_ctx.suspend_allowed_ptr = p_config->suspend_allowed_ptr;


    tlkapi_printf(APP_LOG_EN, "tpsll_stack_init..\nkm_fifo addr: %x\nspp_fifo addr: %x\n",
            (uint32_t)km_fifo_ptr, (uint32_t)spp_fifo_ptr);

    if (!s_app_cb) {
        tlkapi_printf(APP_LOG_EN, "[TPSLL] app_cb NULL, init failed!\n");
        return TLK_ERR_NULL;
    }
    if (!s_app_cb->app_p24g_send_sm_msg) {
        tlkapi_printf(APP_LOG_EN, "[TPSLL] app_p24g_send_sm_msg NULL, init failed!\n");
        return TLK_ERR_NULL;
    }

    ll_ctx.is_alg_kb = is_alg_kb;

    pp_fill_paring_pkt();

    ll_ctx.step_3t1r = STEP_3T1R_NONE;
    ll_ctx.lost_cnt = 0;
    ll_ctx.lost_cnt_max = LL_LOST_8K_MAX_CNT;

    if (tpsll_is_report_rate_valid(rr))
    {
        ll_ctx.user_rr = rr;
        tlkapi_printf(APP_LOG_EN, "RR<0x%02X>\n", ll_ctx.user_rr);
    }
    else
    {
        ll_ctx.user_rr = APP_DEFAULT_REPORT_RATE;
        tlkapi_printf(APP_LOG_EN, "default RR<0x%02X>\n", ll_ctx.user_rr);
    }

    tmemset(rf_tx_packet.buf, 0, 32);

    tpsll_inter_register_callback(TPSLL_INTER_EVT_SPP_SEND_COMP, (tpsll_evt_callback_t)tpsll_stack_event_handler);
    // tpsll_inter_register_callback(TPSLL_INTER_EVT_KS_DISABLE, (tpsll_evt_callback_t)tpsll_stack_event_handler);

    #if TPSLL_KB_ACTIVE_PM_OPTM_LEVEL1_START_MS
    if (TLK_SUCCESS != tpsll_kb_active_pm_optm_level_start_time_set(TPSLL_KB_ACTIVE_PM_OPTM_LEVEL1, TPSLL_KB_ACTIVE_PM_OPTM_LEVEL1_START_MS)) {
        tlkapi_printf(APP_LOG_EN,"set kb alive pm optm level1 start time failed\n");
    }
    #endif
    #if TPSLL_KB_ACTIVE_PM_OPTM_LEVEL3_START_MS
    if (TLK_SUCCESS != tpsll_kb_active_pm_optm_level_start_time_set(TPSLL_KB_ACTIVE_PM_OPTM_LEVEL3, TPSLL_KB_ACTIVE_PM_OPTM_LEVEL3_START_MS)) {
        tlkapi_printf(APP_LOG_EN,"set kb alive pm optm level3 start time failed\n");
    }
    #endif

    tlkapi_printf(APP_LOG_EN, "tpsll_stack_init done\n");
    return TLK_SUCCESS;
}