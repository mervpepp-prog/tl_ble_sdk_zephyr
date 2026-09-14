/********************************************************************************************************
 * @file    app_d24g_stack.c
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
#include "tl_fifo.h"
#include "app_public.h"
#include "tl_string.h"
#include "../common/stack/d24g_stack_common.h"


#define MOUSE_8K_USE_64M_CCLK_EN                 1

#if MOUSE_8K_USE_64M_CCLK_EN
//18+18+12
#define TPSLL_NEW_8K_RF_TIMING_EN                1
/**
 * Note: This macro definition can only be enabled when TPSLL_NEW_8K_RF_TIMING_EN = 1
 * Meanwhile, MOUSE_8K_USE_64M_CCLK_EN needs to be set to 1
 * 13+13+9
*/
#define TPSLL_NEW_8K_RF_PACKET_FORMAT_EN         1
#define TPSLL_NEW_KEEP_ALIVE_EN                  1
#else
#define TPSLL_NEW_8K_RF_TIMING_EN                0
#define TPSLL_NEW_8K_RF_PACKET_FORMAT_EN         0
#define TPSLL_NEW_KEEP_ALIVE_EN                  0
#endif

#define TPSLL_8K_PKT_LOSS_SNNESN_OP_EN           1

#if TPSLL_NEW_KEEP_ALIVE_EN
#define TPSLL_8K_KEEP_ALIVE_SYNC_IN_US         (4000)
#endif

// Enable 8k packet alg function (1: enabled, 0: disabled)
#define TPSLL_PACKET_LOSS_COMPENSATE_EN          1

#define XY_CAP_COMPENSATE_EN                     1

#define TPSLL_8K_COMPRESS_XY_DATA_EN             0
#define MAX_XY                                   (120)


#define TPSLL_SPP_NOT_SWITCH_EN                  1
#define TPSLL_NON_8K_SPP_NOT_SW_EN               1

/* KM_SPP_MAX_LEN moved to d24g_stack_common.h */
/* DONGLE_SPP_MAX_LEN moved to d24g_stack_common.h */

#if TPSLL_PACKET_LOSS_COMPENSATE_EN
#define TPSLL_KALMAN_PRED_MAX_NUM               (12)
#define TPSLL_KALMAN_PRED_REAL_NUM              (2)
#define TPSLL_KALMAN_PRED_THRESHOLD             (1)
#define TPSLL_KALMAN_PRED_MOVE_LEVEL_TH         (3)

#define TPSLL_KALMAN_PARAM_GROUP                (2)

#if TPSLL_KALMAN_PARAM_GROUP == 0
    #define TPSLL_KALMAN_Q                    (0.5f)
    #define TPSLL_KALMAN_R                    (1.0f)
#elif TPSLL_KALMAN_PARAM_GROUP == 1
    #define TPSLL_KALMAN_Q                    (1.0e-6f)
    #define TPSLL_KALMAN_R                    (32.0f)
#elif TPSLL_KALMAN_PARAM_GROUP == 2
    #define TPSLL_KALMAN_Q                    (1.0e-6f)
    #define TPSLL_KALMAN_R                    (96.0f)
#endif

#endif


#if TPSLL_NEW_8K_RF_PACKET_FORMAT_EN

#define TPSLL_RECONN_ALIGN_US                   (6)
#define TPSLL_RX1_PKT_LEN                       (13)
#define TPSLL_RX2_PKT_LEN                       (13)
#define NEW_8K_PKT_SOF_ADJ                      (14 + 5) // when testing 8k latency, stm32 device need to add 5us offset
#define KM_2T_OFFSET_US                         (9)
#define KM_3T_OFFSET_US                         (9)
#else
#define TPSLL_RECONN_ALIGN_US                   (5)
#define TPSLL_RX1_PKT_LEN                       (18)
#define TPSLL_RX2_PKT_LEN                       (18)
#define NEW_8K_PKT_SOF_ADJ                      (0)
#define KM_2T_OFFSET_US                         (0)
#define KM_3T_OFFSET_US                         (0)
#endif

#define SQUARE_DEBUG_TEST   (0)


/* SPP_MAX_CONT_CNT moved to d24g_stack_common.h */
#define NSPP_CNT    (2)

// Enable frequency hopping function (1: enabled, 0: disabled)
// #define HOP_FREQ_FUN_ENABLE   1

/* ACCESS_CODE is now defined in app_d24g_stack_common.h */


/* SLOT1_OFFSET_TIME_US_500_1K_2K moved to d24g_stack_common.h */
/* SLOT_TIME_US_500_1K_2K moved to d24g_stack_common.h */
/* SLOT1_OFFSET_TIME_US_125 moved to d24g_stack_common.h */
/* SLOT_TIME_US_125 moved to d24g_stack_common.h */
#define KM_SLOT0_TX_TIME_US                     (60)
#define KM_TX_MARGIN_US_125                     (40)
/* KB_ALL_KEY_MARGIN_TIME moved to d24g_stack_common.h */
/* FOR_KB_ALL_KEY_SYNC_MARGIN_TIME moved to d24g_stack_common.h */
#define SLOT_SYNC_OFFSET_TIME_US_500_1K_2K      (76 + 21 + FOR_KB_ALL_KEY_SYNC_MARGIN_TIME)
/* KM_SLOT0_MARGIN_US_500_1K_2K moved to d24g_stack_common.h */
/* KM_SLOT1_TIME_US_500_1K_2K moved to d24g_stack_common.h */
#define KM_SLOT_TIME_US_500_1K_2K               (SLOT_TIME_US_500_1K_2K + KB_ALL_KEY_MARGIN_TIME)

/* MAX_TASK_SLOTS_1K moved to d24g_stack_common.h */
/* MAX_TASK_SLOTS_500 moved to d24g_stack_common.h */
/* MAX_TASK_SLOTS_250 moved to d24g_stack_common.h */
/* MAX_TASK_SLOTS_125 moved to d24g_stack_common.h */
/* MAX_TASK_SLOTS_2K moved to d24g_stack_common.h */
/* MAX_TASK_SLOTS_4K moved to d24g_stack_common.h */

#define TPSLL_CONN_TIMEOUT_8K_MS                (100)
#define TPSLL_MS_8K_INTERVAL                    (4)
#define TPSLL_LOST_CNT_MAX_8K                   (TPSLL_CONN_TIMEOUT_8K_MS * TPSLL_MS_8K_INTERVAL); //n interval

/* RR_CHG_DLY_CNT moved to d24g_stack_common.h */

/* FLOW_PKT_NEW moved to d24g_stack_common.h */
/* FLOW_PKT_SENT moved to d24g_stack_common.h */

/* SN_MASK moved to d24g_stack_common.h */
/* NESN_MASK moved to d24g_stack_common.h */

/* GET_SN moved to d24g_stack_common.h */
/* GET_NESN moved to d24g_stack_common.h */
/* SET_SN moved to d24g_stack_common.h */
/* SET_NESN moved to d24g_stack_common.h */


#define APP_STK_BASIC_ACK_LEN                       (2) /* CHN (1) + SYNC (1) */

#define RX_SYNC_TIME_TICK      (1152) //(RX_SYNC_TIME_US * RF_SYSTEM_TIMER_TICK_1US)

#if TPSLL_NON_8K_SPP_NOT_SW_EN
#define RF_MAX_LEN      KM_SPP_MAX_LEN
#else
#define RF_MAX_LEN      (17)
#endif

#define DONGLE_UL_DATA_IDLE                 0x00
#define DONGLE_UL_DATA_PENDING              0x01
#define DONGLE_UL_DATA_SENDING              0x02
//////////////////////////////////////////////////////////////////////////////
/* D_SUSPEND_EARLY_TIME_US moved to d24g_stack_common.h */
#define TIMER1_125_250_1ST_TIMER_US                 ((SLOT1_OFFSET_TIME_US_125) + D_SUSPEND_EARLY_TIME_US)
//////////////////////////////////////////////////////////////////////////////
#define TPSLL_RF_TX_POWER_TRIM_EN                           (1)
//////////////////////////////////////////////////////////////////////////////
#define TPSLL_RECON_ENTER_PM_OPTM_MAX_START_TIME_S          (120)
#define TPSLL_RECON_ENTER_PM_OPTM_DEFAULT_START_TIME_S      (10)
#define TPSLL_RECON_PM_OPTM_MAX_INTERVAL_TIME_MS            (200)
#define TPSLL_RECON_PM_OPTM_INTERVAL_TIME_MS                (60)
#define TPSLL_RECON_OPTM_CNT_REMAINDER                      (10)

typedef enum
{
    TPSLL_INTER_EVT_SPP_SEND_COMP = TPSLL_EVT_MAX,

    TPSLL_INTER_EVT_MAX,
} tpsll_inter_evt_t;

typedef struct {
    int8_t x;
    int8_t y;
} ms_xy_s_t;

typedef struct {
    int16_t x;
    int16_t y;
} ms_xy_t;



static tpsll_app_var_cb_t s_app_var_cb = NULL;
static tpsll_app_cb_t *s_app_cb = NULL;

typedef void (*rf_irq_handler_t)(void);

static volatile rf_irq_handler_t s_rf_irq_handler;
void tpsll_channel_statistic(uint8_t chn_idx, bool err);


#define REPORT_RATE_MASK (REPORT_RATE_125 | REPORT_RATE_250 | REPORT_RATE_500 |\
                            REPORT_RATE_1K | REPORT_RATE_2K | REPORT_RATE_4K | REPORT_RATE_8K)

#define REPORT_RATE_NOT_8K_MSK (REPORT_RATE_125 | REPORT_RATE_250 | REPORT_RATE_500 |\
                                REPORT_RATE_1K | REPORT_RATE_2K | REPORT_RATE_4K)
//////////////////////////////////////////////////////////////////////////////
/* TPSLL_NEW_RECON_EN moved to d24g_stack_common.h */
/* TPSLL_NEW_PAIR_EN moved to d24g_stack_common.h */

/* TPSLL_PAIR_CHN_MAX_NUM moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////
/* DONGLE_INTERVAL_US moved to d24g_stack_common.h */
#define PAIRING_INTERVAL_US                         (300)
#define DONGLE_SUSPEND_INTERVAL_US                  (100000)//(50000)
#define DONGLE_SUSPEND_TIMER1_US                    (155)
#define DONGLE_PARING_STIMER_TO_TX_MARGIN_US        (2)     /* Note: This macro needs to be adjusted according to the code */
/* KB_PAIRING_STIMER_TO_RX_MARGIN_US moved to d24g_stack_common.h */

#define PARING_STIMER_TO_TX_US                      (4)
#define DONGLE_PAIRING_SRX_TIMEOUT_US               (42)     /* Note: This macro needs to be adjusted according to the code */
// #define DONGLE_PAIRING_SRX_TIMEOUT_US               (56)     /* Note: This macro needs to be adjusted according to the code */
#define DONGLE_RECONN_LONG_RX_WIN                   (13000)
#if TPSLL_NEW_8K_RF_PACKET_FORMAT_EN
#define DONGLE_CONNECTED_SRX_TIMEOUT_US             (42 + 3 + 3)     /* Note: This macro needs to be adjusted according to the code */
#else
#define DONGLE_CONNECTED_SRX_TIMEOUT_US             (42)     /* Note: This macro needs to be adjusted according to the code */
#endif
#define DONGLE_CONNECTED_L_SRX_TIMEOUT_US           (65 + 10)     /* Note: This macro needs to be adjusted according to the code */
#define DONGLE_CONNECTED_1ST_SUSPEND_SRX_TIMEOUT_US           (D_SUSPEND_EARLY_TIME_US + (SLOT_TIME_US_125 * 2))     /* Note: This macro needs to be adjusted according to the code */
#define DONGLE_CONNECTED_SUSPEND_SRX_TIMEOUT_US           (SLOT_TIME_US_125 * 2)     /* Note: This macro needs to be adjusted according to the code */
#define DONGLE_CONNECTED_SPP_MODE_RX_TIMEOUT            (87)    /* Note: This macro needs to be adjusted according to the code */


/* RF_PKT_PREAMBLE_LEN moved to d24g_stack_common.h */
/* RF_PKT_AC_LEN moved to d24g_stack_common.h */
/* RF_PKT_HEADER_LEN moved to d24g_stack_common.h */
/* RF_PKT_DATA_LEN moved to d24g_stack_common.h */
/* RF_PKT_CRC_LEN moved to d24g_stack_common.h */

/* RF_PKT_ACK_LEN moved to d24g_stack_common.h */
#define RF_PKT_NULL_ACK_LEN                         (2)
#define RF_PKT_4K_2K_NULL_ACK_LEN                   (0)
#define RF_PKT_NULL_125_ACK_LEN                     (2)
#define RF_PKT_NULL_500_ACK_LEN                     (2)

/* RF_SETTLE_TIME_US moved to d24g_stack_common.h */
/* RF_RX_PATH_DELAY_US moved to d24g_stack_common.h */
/* RF_TX_PATH_DELAY_US moved to d24g_stack_common.h */
/* RF_RX_HIT_SYNC_US moved to d24g_stack_common.h */
//#define CONNECTED_CODE_MARGIN_US                    (3)
//#define RF_TX_WAIT                                  (1)
//#define RF_RX_WAIT                                  (1)

/* RF_GET_DMA_LEN moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////////////////

#define TIMER1_FINE_ADJ                     (5)

// #define RF_SET_TIME_US                           (2)


/* TIMER1_AHEAD_US moved to d24g_stack_common.h */
/* D_TIMER1_TX_SLOT_AHEAD_US moved to d24g_stack_common.h */
/* RF_TRIG_AND_MARGIN moved to d24g_stack_common.h */

// 15 + 1 + ((2+5+5+12+3) * 4 + 2) / 3 + 8
// #define RF_STX_WITH_DATA_LEN_TIME_US(data_len)      ((RF_SETTLE_TIME_US + ((RF_PKT_PREAMBLE_LEN + RF_PKT_AC_LEN + RF_PKT_HEADER_LEN + (data_len) + RF_PKT_CRC_LEN) * 4 + 2) / 3) + RF_RX_PATH_DELAY_US)

/* APP_RF_12B_RX_FSM_TIME_US moved to d24g_stack_common.h */
/* APP_RF_24B_RX_FSM_TIME_US moved to d24g_stack_common.h */

/* 3 slot0 tx tick*/
#define APP_SLOT0_1T_START_TIME_US                  (3) /* 1T start time */


/* 58 slot1 timer1*/
#if TPSLL_NEW_8K_RF_TIMING_EN
    #if TPSLL_NEW_8K_RF_PACKET_FORMAT_EN
    #define APP_SLOT1_START_TIME_US                     (APP_RF_12B_RX_FSM_TIME_US + APP_SLOT0_1T_START_TIME_US - TIMER1_AHEAD_US + 4 + KM_2T_OFFSET_US)/* SLOT1 start time */
    #else
    #define APP_SLOT1_START_TIME_US                     (APP_RF_12B_RX_FSM_TIME_US + APP_SLOT0_1T_START_TIME_US - TIMER1_AHEAD_US + 8)/* SLOT1 start time */
    #endif
#else
#define APP_SLOT1_START_TIME_US                     (APP_RF_12B_RX_FSM_TIME_US + APP_SLOT0_1T_START_TIME_US - TIMER1_AHEAD_US)/* SLOT1 start time */
#endif
/* 62 slot1 tx tick*/
#define APP_SLOT1_2T_START_TIME_US                  (APP_RF_12B_RX_FSM_TIME_US + APP_SLOT0_1T_START_TIME_US + RF_TRIG_AND_MARGIN) /* 2T start time */


/* 133 slot2 timer1*/
#if TPSLL_NEW_8K_RF_PACKET_FORMAT_EN
#define APP_SLOT2_START_TIME_US                     (APP_SLOT1_2T_START_TIME_US + APP_RF_24B_RX_FSM_TIME_US - TIMER1_AHEAD_US - 9)  /* SLOT2 START*/
#else
#define APP_SLOT2_START_TIME_US                     (APP_SLOT1_2T_START_TIME_US + APP_RF_24B_RX_FSM_TIME_US - TIMER1_AHEAD_US)  /* SLOT2 START*/
#endif
/* 137 slot2 tx tick*/
#define APP_SLOT2_3T_START_TIME_US                  (APP_SLOT1_2T_START_TIME_US + APP_RF_24B_RX_FSM_TIME_US + RF_TRIG_AND_MARGIN) /* 2T start time */


/* 193 slot3 timer1*/
#if TPSLL_NEW_8K_RF_PACKET_FORMAT_EN
#define APP_DONGLE_SLOT3_START_TIME_US              (APP_SLOT2_3T_START_TIME_US + APP_RF_12B_RX_FSM_TIME_US - D_TIMER1_TX_SLOT_AHEAD_US - 5 - 8)  /* SLOT3 START*/
/* 196 slot3 tx tick*/
#define APP_SLOT3_1T_START_TIME_US                  (APP_SLOT2_3T_START_TIME_US + APP_RF_12B_RX_FSM_TIME_US + RF_TRIG_AND_MARGIN + 1 - 5 - 7)

#else
#define APP_DONGLE_SLOT3_START_TIME_US              (APP_SLOT2_3T_START_TIME_US + APP_RF_12B_RX_FSM_TIME_US - D_TIMER1_TX_SLOT_AHEAD_US)  /* SLOT3 START*/
/* 196 slot3 tx tick*/
#define APP_SLOT3_1T_START_TIME_US                  (APP_SLOT2_3T_START_TIME_US + APP_RF_12B_RX_FSM_TIME_US + RF_TRIG_AND_MARGIN + 1)

#endif

#define MOUSE_SOF_ADJ                               (43)
#define MOUSE_SOF_NEW_8K_PKT_SN_NESN_ADJ            (2)//Increasing this value means deferring the SOF.
#define DONGLE_SOF_INTERVAL_US                      (125)
//#define DONGLE_STIMER_TO_SOF_DELAY_US               (140) //for mouse latency
#define DONGLE_STIMER_TO_SOF_DELAY_US               (APP_DONGLE_SLOT3_START_TIME_US - MOUSE_SOF_ADJ + MOUSE_SOF_NEW_8K_PKT_SN_NESN_ADJ + NEW_8K_PKT_SOF_ADJ) //for keyboard latency
#define DONGLE_4K_STIMER_TO_SOF_DELAY_US            (78) //for keyboard latency



// Frequency hopping channel selection method (1: specific method selected)
/* HOP_CHANNEL_METHOD moved to d24g_stack_common.h */
// Channel combination method (0: specific combination method selected)
/* COMB_METHOD moved to d24g_stack_common.h */
// Maximum number of channel combinations supported
/* MAX_COMB_NUM moved to d24g_stack_common.h */
// Enable RSSI (Received Signal Strength Indicator) scanning (1: enabled, 0: disabled)
/* RSSI_SCAN_EN moved to d24g_stack_common.h */
// Enable Adaptive Frequency Hopping (AFH) (1: enabled, 0: disabled)
/* AFH_ENABLE moved to d24g_stack_common.h */

#if AFH_ENABLE  // If AFH is enabled, define related parameters

// Maximum number of channels supported by AFH
#define MAX_CHANNELS 4
// Minimum value for channel quality evaluation (lower means poorer quality)
#define MIN_QUALITY -5
// Maximum value for channel quality evaluation (higher means better quality)
#define MAX_QUALITY 5
// Quality threshold for determining valid channels (channels above this are considered usable)
#define QUALITY_THRESH -2
// Minimum number of valid channels required for normal communication
#define MIN_VALID_CHANNELS 3

// Maximum value for 4-bit countdown counter (0-15)
#define COUNTDOWN_BIT4_MAX 15

// Exponent for AFH interval calculation (2^9 = 512)
#define AFH_INTERVAL_EXP 9  
// AFH interval in microseconds (512us, calculated as 2^AFH_INTERVAL_EXP)
#define AFH_INTERVAL     (1 << AFH_INTERVAL_EXP)  

#endif
//////////////////////////////////////////////////////////////////////////////

// Default report rate for the application (8KHz)
#define APP_DEFAULT_REPORT_RATE  REPORT_RATE_8K
// Alternative default report rate (1KHz)
// #define APP_DEFAULT_REPORT_RATE  REPORT_RATE_1K
//////////////////////////////////////////////////////////////////////////////
/* TPSLL_CONN_QUICK_LOST_TIME_US moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////////////////
/* TPSLL_RR_CHG_WITH_RECON_EN moved to d24g_stack_common.h */
#define TPSLL_RR_CHG_WITH_RECON_NO_RSP_EN           (1)

/* TPSLL_RR_DELAY_SWITCH_FNO_N moved to d24g_stack_common.h */
// #define TPSLL_RR_SWITCH_CONFIG_TIME_US              (3000)
/* TPSLL_RR_SWITCH_CONFIG_TIME_US moved to d24g_stack_common.h */
/* TPSLL_RR_CHG_RECON_TIMEOUT_US moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////////////////
#define TPSLL_RX_MS_DATA_VALID_LEN                  (5)

#define TPSLL_WHEEL_TEST_EN                         (0)
#define TPSLL_LOCAL_MS_DATA_TEST_EN                 (0)

#define TPSLL_X_Y_FLUSH_HANDLE_EN                   (1)
//////////////////////////////////////////////////////////////////////////////
/* usb sof sync */
#define STIMER_SOF_SYNC_WITH_KMD_SYNCED_EN          (1)
#if STIMER_SOF_SYNC_WITH_KMD_SYNCED_EN
#define STIMER_SOF_DELTA_ADJ_DELAY_CNT              (0)
#else
#define STIMER_SOF_DELTA_ADJ_DELAY_CNT              (1)
#endif

#define STIMER_SOF_DELTA_ADJ_MAX_US                 (1)
//////////////////////////////////////////////////////////////////////////////
#define TPSLL_8K_CHN_EXPAND_EN                      (0)

/* TPSLL_N8K_CHN_EXPAND_EN moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////////////////
/* Frequency-Hopping Spread Spectrum */
/* TPSLL_FHSS_EN moved to d24g_stack_common.h */
/* TPSLL_N8K_FHSS_EN moved to d24g_stack_common.h */

#if TPSLL_N8K_FHSS_EN
#define TPSLL_N8K_FHSS_AFH_SLOT_SHIFT               (10)                 // 512 interval
#define TPSLL_N8K_FHSS_PER_AFH_WAIT_FNO_NUM         (0x40)
// #define TPSLL_N8K_FHSS_PER_AFH_WAIT_FNO_NUM_2K      (0x100)

#define LL_PKT_OFFSET                               (4)

typedef struct
{
    uint16_t afh_mask;
    uint16_t update_pos;
    // uint16_t wait_fno_num;
    uint8_t afh_shift;
} afh_cfg_t;
static afh_cfg_t g_afh_cfg;


#else
#define LL_PKT_OFFSET                               (2)
#endif

#define TPSLL_FHSS_REPORT_PER_EN                    (1)
#define TPSLL_REPORT_PER_TH                         (6) 
#define TPSLL_REPORT_PER_KICK_OUT_CNT               (2)

/* TPSLL_FHSS_CHN_MAX_NUM moved to d24g_stack_common.h */

#if TPSLL_8K_CHN_EXPAND_EN
#define TPSLL_HOP_CHN_NUM                           (6)
#else
#define TPSLL_HOP_CHN_NUM                           (3)
#endif

#define TPSLL_FHSS_CHN_SPACE_CHECK_EN               (1)
#define TPSLL_HOP_CHN_TAB_KEEP_GOOD_NUM             (1)
#define TPSLL_FHSS_CHN_HISTORY_SCORE_TH             (30)
#define TPSLL_FHSS_CHN_PER_GOOD_TH                  (20)//(30)

#define TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_ADJ_EN    (1)
#define TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_DEFAULT   (15) /* n/n+1 */
#define TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_MAX       (80) /* n/n+1 */
#define TPSLL_FHSS_GOOD_CHN_PER_WEIGHT_STEP         (10)
#define TPSLL_FHSS_BAD_CHN_PER_WEIGHT_STEP          (4)

/* TPSLL_FHSS_AFH_SLOT_SHIFT moved to d24g_stack_common.h */
/* TPSLL_FHSS_PER_T_FNO_NUM moved to d24g_stack_common.h */
#define TPSLL_FHSS_PER_AFH_WAIT_FNO_NUM             (0x60)

#define TPSLL_FHSS_EXPECTED_REPORT_CNT              ((TPSLL_FHSS_PER_T_FNO_NUM + 1) * 2)

#define TPSLL_FHSS_CHN_FLG_VALID                    BIT(0)
#define TPSLL_FHSS_CHN_FLG_CURRENT                  BIT(1)

#define TPSLL_FHSS_FIX_OLD_CHN_IDX_EN               (1)

/* force selection of separated channels */
#define TPSLL_FHSS_FORCE_SELECT_SEPARATED_CHN_EN    (0)
//////////////////////////////////////////////////////////////////////////////
/* TPSLL_RECON_RSSI_SCAN_EN moved to d24g_stack_common.h */
/* TPSLL_RECON_RSSI_SCAN_MAX_EN moved to d24g_stack_common.h */

/* RSSI_SCAN_CHANNEL_COUNT moved to d24g_stack_common.h */
/* TPSLL_WIFI_CHANNEL_COUNT moved to d24g_stack_common.h */

#if TPSLL_RECON_RSSI_SCAN_EN && TPSLL_NEW_RECON_EN
#define TPSLL_RECON_CHN_NUM                         TPSLL_HOP_CHN_NUM
#define RSSI_SCAN_WIN_US                            (100)
#define RSSI_SCAN_MARGIN_US                         (5)
#define RSSI_SCAN_ROUNDS                            (3)
#define RSSI_SCAN_TOP_CHANNEL_COUNT                 TPSLL_FHSS_CHN_MAX_NUM
#define TPSLL_RECON_RSSI_SCAN_TIME_US               ((RSSI_SCAN_WIN_US + RSSI_SCAN_MARGIN_US) * RSSI_SCAN_CHANNEL_COUNT * RSSI_SCAN_ROUNDS + 1000)
#endif

#define TPSLL_FHSS_BAD_CHN_HISTORY_ADD_SCORE        (20)
/* turn 8K report rate channels in order */
/* TPSLL_8K_CHN_HOP_TURN_EN moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////////////////
#define TPSLL_RF_SCAN_EN                            (1)

#define TPSLL_RF_SCAN_CHN_TOTAL                     (40)
#define TPSLL_RF_SCAN_WIN_US                        (100)
#define TPSLL_RF_SCAN_ROUNDS                        (3)

#define TPSLL_RF_SCAN_START_TIME_US                 (1000)
#define TPSLL_RF_SCAN_IDLE_MIN_TIME_US              (1000 * 1000)

#define TPSLL_RF_SCAN_SELECT_CHN_NUM                (TPSLL_HOP_CHN_NUM)
#define TPSLL_RF_SCAN_BEST_CHN_PER_DECREASE_VALUE   (6)
#define TPSLL_RF_SCAN_BEST_CHN_PER_WEIGHT_DECREASE_VALUE      (6)
//////////////////////////////////////////////////////////////////////////////

#if TPSLL_RF_SCAN_EN
#define TPSLL_AVOID_WIFI_CHN_USE_RX3_EN          1
#else
#define TPSLL_AVOID_WIFI_CHN_USE_RX3_EN          0
#endif

#define TPSLL_AVOID_WIFI_CHN_SCAN_CNT            8
//////////////////////////////////////////////////////////////////////////////
#define TPSLL_AVOID_WIFI_CHN_EN                     (1)

#define TPSLL_WIFI_BEST_CHN_NUM                     (3)
#define TPSLL_WIFI_RSSI_SUM_HISTORY_N               (4)
#define TPSLL_WIFI_RSSI_SUM_NEW_N                   (1)
#define TPSLL_WIFI_RSSI_SUM_TOTAL_N                 (TPSLL_WIFI_RSSI_SUM_HISTORY_N + TPSLL_WIFI_RSSI_SUM_NEW_N)

#define TPSLL_WIFI_RSSI_SCORE_EN                    (1)

#define TPSLL_WIFI_RSSI_SCORE_MAX_EN                (0) //0 for weight average

#define TPSLL_VALID_CHN_SCORE_HISTORY_N             (1)
#define TPSLL_VALID_CHN_SCORE_NEW_N                 (2)
#define TPSLL_WIFI_RSSI_SCORE_MUILT                 (3)
//////////////////////////////////////////////////////////////////////////////
#define ABNORMAL_RETRY_FIX                          (1)
/* TPSLL_ACC_SWITCH_WITH_RR_EN moved to d24g_stack_common.h */

#define TPSLL_MOUSE_BTN_FILTER_EN                   (0)
#define TPSLL_BTN_MIN_TIME_US                       (10 * 1000)
//////////////////////////////////////////////////////////////////////////////
/* TPSLL_8K_WHEEL_SEND_IN_LL_EN moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////////////////
#define TPSLL_FAST_SETTLE_CAL_FUN_EN                (0)
#define TPSLL_KEEP_ACTIVE_WAIT_TIME_MS              (10 * 1000)

#if TPSLL_FAST_SETTLE_CAL_FUN_EN
#define TPSLL_FAST_SETTLE_CAL_WAIT_MS               (1)
#define TPSLL_FAST_SETTLE_CAL_TIMEOUT_MS            (60 * 1000)
#endif
//////////////////////////////////////////////////////////////////////////////

#define TPSLL_LL_MAX_PAYLOAD_LEN                    (16)
//////////////////////////////////////////////////////////////////////////////
#if TPSLL_NEW_PAIR_EN
#define PAIR_REQ_ENCRYPT_KEY_OFFSET                 (3)
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

//////////////////////////////////////////////////////////////////////////////
#define TPSLL_TX_VALID_LEFT_TIME_US                 (75)
//////////////////////////////////////////////////////////////////////////////
/* TPSLL_KEEP_ALIVE_NOTIFY_EN moved to d24g_stack_common.h */

#define TPSLL_KEEP_ALIVE_PER_EN                     (1)
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

/* ZW_TG moved to d24g_stack_common.h */

/* ZW_WR moved to d24g_stack_common.h */

/* PAIRING_RSP_MAX_RETRY_CNT moved to d24g_stack_common.h */
#define INTV_FOR_SAVE_INFO_TO_FLASH   10

#define DONGLE_SUSPEND_SUPPORT          1

#define TPSLL_USB_DATA_CACHE_EN         1

#define DONGLE_SPP_FRAG_EN              1

#if DONGLE_SPP_FRAG_EN
#define SPP_FRAG_HDR_LEN            1
#define SPP_MAX_CHUNK_LEN           33
#define SPP_MAX_PACKET_LEN          256
//////////////////////////////////////////////////////////////////////////////
/* TPSLL_PM_OPTM_EN moved to d24g_stack_common.h */
//////////////////////////////////////////////////////////////////////////////

typedef enum {
    SPP_FRAG_TYPE_NO_FRAG          = 0,
    SPP_FRAG_TYPE_MID              = 1,
    SPP_FRAG_TYPE_END              = 2,
    SPP_FRAG_TYPE_RESERVED         = 3,
} spp_frag_type_t;

#define SPP_OPCODE_DEFAULT          0
#define SPP_OPCODE_SHIFT            2
#endif

static uint8_t spp_frag_total = 0;
static uint8_t spp_frag_acked = 0;
static uint8_t *spp_original_data  = NULL;
static uint8_t  spp_original_len   = 0;
static bool     spp_sending_busy   = false;
static uint8_t spp_send_buf[48 + 1];

static void spp_send_reset(void);

volatile bool chn_mask_change = false;

tpsll_dev_status_e g_state = STATE_PAIRING;

#define APP_RF_MOUSE_REPORT_EN      1

volatile unsigned int  rx_step=0;
static tpsll_evt_callback_t p24g_callbacks[TPSLL_INTER_EVT_MAX] = {0};

volatile __attribute__((section(".sdk_version"))) unsigned char d24g_rf_version[] = "$$$"TPSLL_STACK_VERSION"$$$";
const uint8_t tpsll_stack_version[] = TPSLL_STACK_VERSION;

static TPSLL_ALG_STATUS_E g_kalman_enable = TPSLL_ALG_ENABLE;

#define   IS_3T_2R      1   
#define   IS_1T_1R      0  

#define  profile_type   IS_3T_2R

volatile  unsigned char rx_pair_id_same_cnt = 0;
volatile  unsigned char last_data_type = 0;

volatile  unsigned char last_usb_report_len = 0;
_attribute_aligned_(4)  unsigned char last_usb_report_buf[RF_PKT_DATA_LEN];
_attribute_aligned_(4)  unsigned char last_all_report_buf[17];

//Note: The per function only works when AFH is enabled.
#define PER_HIGH_TH    300   // 30% * 1000/100 = 100
#define PER_LOW_TH     200   // 20%
#define PER_ALPHA_NUM    1   // α = 1/1000, Equivalent to smoothing approximately 1000 packages.
#define PER_ALPHA_DEN 1000

#if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
typedef enum
{
    TPSLL_PKT_ALL_LOSS = 0,
    TPSLL_PKT_2_LOSS,
    TPSLL_PKT_ALL_RCVD,

} tpsll_pkt_status_e;

typedef struct {
    int16_t x;
    int16_t y;
} ms_acc_data_t;

#endif

#if TPSLL_PACKET_LOSS_COMPENSATE_EN
typedef struct {
    int16_t x;
    int16_t y;
} pred_acc_data_t;

typedef enum
{
    KF_PRED_MOVE_LEVEL_NORMAL,
    KF_PRED_MOVE_LEVEL_FAST,
    KF_PRED_MOVE_LEVEL_NONE,
} tpsll_kf_level_e;

volatile pred_acc_data_t pred_max_p;
volatile pred_acc_data_t pred_max_n;
#endif

#if TPSLL_RR_CHG_WITH_RECON_EN
typedef enum
{
    RR_RECON_STEP_NONE = 0,
    RR_RECON_STEP1,
    RR_RECON_STEP2,
    RR_RECON_STEP3,
    RR_RECON_STEP4,
    RR_RECON_STEP5,
    RR_RECON_STEP6,
} tpsll_rr_recon_e;
#endif

/* tpsll_pair_e, tpsll_recon_e, report_mode_t are now defined in app_d24g_stack_common.h */

typedef enum
{
    RECON_RSSI_SCAN_STEP_NONE = 0,
    RECON_RSSI_SCAN_STEP_PREPARE,
    RECON_RSSI_SCAN_STEP_READY,
    RECON_RSSI_SCAN_STEP_START,
} tpsll_recon_rssi_e;

#if TPSLL_8K_WHEEL_SEND_IN_LL_EN
typedef enum
{
    TPSLL_LL_STK_CMD_NONE               =   TPSLL_LL_CMD_MAX,
    TPSLL_LL_STK_CMD_WHEEL_DATA,
} tpsll_ll_stk_cmd_e;
#endif

static uint16_t ms3_cnt = 0;

signed short auto_draw_flag = 0;
// per_est unit: 0 ~ 1000 -> 0% ~ 100%
// static uint16_t per_est = 0;
// static report_mode_t report_mode = REPORT_MODE_NORMAL;

#if TPSLL_MOUSE_BTN_FILTER_EN
typedef struct btn_rec
{
    uint32_t prs_tick;
    uint32_t rls_tick;
    uint32_t hold_tick;
    uint8_t last;
} btn_rec_t;
#endif

/* pair_encrypt_status_t is now defined in app_d24g_stack_common.h */

typedef struct
{
    /* 4-byte aligned group (members aligned to 4-byte boundary) */
    uint32_t stick;            // Current stick value (timing/synchronization parameter)
    uint32_t last_stick;       // Previous stick value
    uint32_t interval_us;      // Communication interval in microseconds
    uint32_t fno_conn;         // Connection frequency number (channel used for connection)
    uint32_t early_stick;      // Early stick value (preliminary synchronization)
    uint32_t last_e_stick;     // Previous early stick value
    #if TPSLL_CONN_QUICK_LOST_TIME_US
    uint32_t quick_discon_active_tick;
    #endif
    #if TPSLL_RR_CHG_WITH_RECON_EN
    uint32_t rr_chg_recon_time_tick;
    #endif
    #if TPSLL_RECON_RSSI_SCAN_EN
    uint32_t recon_rx_start_tick;
    #endif

    uint32_t conn_acc;
    uint32_t recon_acc;
    uint32_t pair_acc;
    uint32_t pair_encrypt_key; // 4 bytes pair encrypt key
    uint32_t connected_tick;

    #if TPSLL_FAST_SETTLE_CAL_FUN_EN
    uint32_t fast_settle_cal_tick;
    uint32_t fast_settle_cal_timeout_tick;
    #endif
    uint32_t idle_tick;
    uint32_t rx_win_us;
    #if TPSLL_KEEP_ALIVE_NOTIFY_EN
    uint32_t keep_alive_refer_tick;
    #endif

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
    uint32_t recon_start_ref_tick;
    uint32_t recon_enter_pm_optm_start_time_ms;

    /* 2-byte aligned group (members aligned to 2-byte boundary) */
    uint16_t fno;              // Current frequency number (current channel index)
    uint16_t fno_last;         // Previous frequency number
    uint16_t slot1_us;         // Duration of time slot 1 in microseconds

    uint16_t slot2_us;         // Duration of time slot 2 in microseconds
    uint16_t slot3_us;         // Duration of time slot 3 in microseconds
    uint16_t pending_rr_fno;   // Pending report rate frequency number (for rate changes)
    uint16_t pr_cnt;           // Packet retry counter (number of retries for a packet)

    uint16_t ktag;             // Keyboard-specific tag
    uint16_t mtag;             // Mouse-specific tag

    uint16_t usb_vid;          // USB Vendor ID

    uint16_t per_est;
    uint16_t lost_cnt;          /* Connection lost counter */
    uint16_t lost_cnt_max;      /* Connection max lost counter */
    #if TPSLL_FHSS_REPORT_PER_EN
    uint16_t report_per;
    uint16_t report_pkt_cnt;
    #endif
    #if TPSLL_PACKET_LOSS_COMPENSATE_EN
    uint16_t pred_fno;
    pred_acc_data_t pred_acc;
    uint16_t pkt2_rp_num;
    uint16_t pkt2_loss_num;
    #endif
    #if TPSLL_KEEP_ALIVE_NOTIFY_EN
    uint16_t keep_alive_expect_fno;
    uint16_t keep_alive_expect_fno_last;
    #endif
    uint16_t suspend_interval_ms;

    #if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
    ms_acc_data_t acc;
    ms_acc_data_t last_acc;
    #endif


    /* 1-byte aligned group (members aligned to 1-byte boundary) */
    uint8_t customer_key[ENCRYPT_KEY_MAX_LEN];  // Customer defined encrypt key
    uint8_t customer_key_len;                   // Customer defined encrypt key length
    uint8_t task_slot;         // Current task slot (active time slot)
    uint8_t step_3t1r;         // Current 3T1R step (see tpsll_stack_e for possible values)
    uint8_t step_2t2r;         // Current 2T2R step (see tpsll_stack_e for possible values)
    uint8_t report_rate;            /* Current report rate */
    uint8_t ack_mask;          // Acknowledgment mask (bitmask for tracking received ACKs)
    uint8_t last_ack_mask;          // Acknowledgment mask (bitmask for tracking received ACKs)
    uint8_t last_rr;           // Previous report rate
    uint8_t pair_st;           // Pairing state (stage in pairing process)
    uint8_t pair_cnt;          // Pairing attempt counter (number of pairing tries)

    uint8_t tx_sn;                  /* Transmit sequence number */
    uint8_t rx_nesn;                /* Receive next expected sequence number */
    uint8_t chg_rr;            // Report rate change flag (1: rate change pending)
    uint8_t max_slot;          // Maximum number of time slots available
    uint8_t chg_dly_cnt;       // Change delay counter (delay before applying settings)
    uint8_t reconn_chn;        // Reconnection channel (channel to use for reconnection)

    bool    spp_flag;          // SPP mode flag (true: SPP mode active)
    bool    ll_flag;           // Link layer flag (true: link layer activity)
    bool is_rr_changing;       // Flag: report rate is currently changing (true when in progress)
    bool wfi_en;
    bool wfi_flag;
#if DONGLE_SUSPEND_SUPPORT
    uint8_t suspend_enable;
    uint8_t suspend_allow;
#endif
    #if TPSLL_MOUSE_BTN_FILTER_EN
    btn_rec_t ms_btn_rec;
    #endif
    
    report_mode_t report_mode;
    uint8_t switch_special_cnt;
    uint8_t sn_8k;
    uint8_t nesn_8k;
    bool is_last_report;
    bool spp_mode;
    bool pending_spp_mode;
    bool last_pkt_acked;
    bool tx_pending;
    int8_t z_wheel_data;
    int8_t t_wheel_data;
    int8_t conn_ignore_data_cnt;
    
    uint8_t ack_report;
    uint8_t stk_state;
    uint8_t user_rr;
    uint8_t user_rr_last;
    #if TPSLL_RR_CHG_WITH_RECON_EN
    uint8_t rr_chg_with_recon;
    #endif
    #if STIMER_SOF_SYNC_WITH_KMD_SYNCED_EN
    bool synced;
    #endif
    bool local_ms_data_test;
    uint8_t pair_step;
    uint8_t recon_step;
    uint8_t ir_btn_proc_us;
    #if TPSLL_RECON_RSSI_SCAN_EN
    uint8_t recon_chn_idx;
    uint8_t recon_rssi_scan_step;
    #endif

    #if TPSLL_FHSS_EN
    bool tbl_update;
    #endif
    #if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
    tpsll_pkt_status_e last_intv_pkt_st;
    tpsll_pkt_status_e intv_pkt_st;
    uint8_t last_sn;//for new 8k packet loss operation
    uint8_t last_nesn;//for new 8k packet loss operation
    bool is_need_restore_acc;
    #endif
    #if TPSLL_PACKET_LOSS_COMPENSATE_EN
    uint8_t pred_num;
    uint8_t pred_level;
    uint8_t last_btn;
    bool is_pkt1_reported;
    bool is_pkt2_reported;
    bool is_rp_pkt_loss;
    bool is_rp_pkt1_loss;
    bool is_pred_2;
    #endif
    uint8_t spp_cnt;
    uint8_t nspp_cnt;
    bool allow_spp;
    bool enter_conn_flag;    // Connection entry flag
    bool is_stack_running;
    bool rx_over;
    bool tx_over;

    #if TPSLL_FAST_SETTLE_CAL_FUN_EN
    bool fast_settle_cal_en;
    bool fast_settle_calib_done;
    bool fast_settle_calib_timeout;
    #endif
    bool idle_flag;
    #if TPSLL_AVOID_WIFI_CHN_USE_RX3_EN
    bool is_2r_to;
    bool do_rssi_scan;
    uint8_t is_2r_success_cnt;
    #endif
    #if TPSLL_KEEP_ALIVE_NOTIFY_EN
    uint8_t is_keep_alive;
    #endif
    #if TPSLL_KEEP_ALIVE2_NOTIFY_EN
    bool is_keep_alive2_active;
    #endif
    #if TPSLL_KB_ACTIVE_PM_OPTM_EN
    uint8_t kb_alive_fno_msk;
    uint8_t kb_alive_16ms_intervals;
    uint8_t kb_alive_16ms_intervals_msk;
    uint8_t kb_alive_16ms_intervals_shift;
    #endif
    #if TPSLL_N8K_FHSS_EN
    uint8_t slot0_chn;
    bool is_slot0_rx_ok;
    #endif
    uint8_t recon_pm_optm_interval_ms;
    uint8_t recon_enter_pm_optm_cnt;
    bool recon_rx_for_pkt;
} ll_ctx_t;

ll_ctx_t ll_ctx ={
    #if TPSLL_MOUSE_BTN_FILTER_EN
    .ms_btn_rec.prs_tick = 0,
    .ms_btn_rec.rls_tick = 0,
    .ms_btn_rec.hold_tick = 0,
    .ms_btn_rec.last = 0,
    #endif
    .last_pkt_acked = true,
    .wfi_en = 1,
    .wfi_flag = 0,
#if DONGLE_SUSPEND_SUPPORT
    .suspend_enable = 0,
    .suspend_allow = 0,
#endif
    .z_wheel_data = 0,
    .t_wheel_data = 0,
    .stk_state = STATE_PAIRING,
    .user_rr = APP_DEFAULT_REPORT_RATE,
    .user_rr_last = APP_DEFAULT_REPORT_RATE,
    #if TPSLL_RR_CHG_WITH_RECON_EN
    .rr_chg_recon_time_tick = 0,
    .rr_chg_with_recon = RR_RECON_STEP_NONE,
    #endif
    #if STIMER_SOF_SYNC_WITH_KMD_SYNCED_EN
    .synced = false,
    #endif
    .local_ms_data_test = 0,
    .recon_step = RECON_STEP_NONE,
    #if TPSLL_RECON_RSSI_SCAN_EN
    .recon_chn_idx = 0,
    .recon_rssi_scan_step = 0xff,
    .recon_rx_start_tick = 0,
    #endif
    .is_stack_running = false,
    .rx_over = true,
    .tx_over = true,
    #if TPSLL_FAST_SETTLE_CAL_FUN_EN
    .fast_settle_cal_en = 0,
    .fast_settle_calib_done = 0,
    .fast_settle_calib_timeout = 0,
    .fast_settle_cal_tick = 0,
    .fast_settle_cal_timeout_tick = 0,
    #endif
    .idle_flag = 0, 
    .idle_tick = 0,
    .pair_acc = ACCESS_CODE,
    .suspend_interval_ms = 170,        //For LDO mode, 170ms is recommended;For DCDC mode, 80ms is recommended.
    .recon_pm_optm_interval_ms = TPSLL_RECON_PM_OPTM_INTERVAL_TIME_MS,
    .recon_enter_pm_optm_start_time_ms = TPSLL_RECON_ENTER_PM_OPTM_DEFAULT_START_TIME_S * 1000,
};

typedef enum
{
    TASK_SLOT0 = 0,            // Task time slot 0 (scheduling slot 0)
    TASK_SLOT1 = 1,            // Task time slot 1 (scheduling slot 1)
    TASK_SLOT2 = 2,            // Task time slot 2 (scheduling slot 2)
    TASK_SLOT3 = 3,            // Task time slot 3 (scheduling slot 3)

    STEP_3T1R_NONE = 0xB0,     // No 3T1R (3 Transmit 1 Receive) step active
    STEP_3T1R_1T_D_1R = 0,     // 3T1R step: 1 transmit (KB) - 1 receive (dongle)
    STEP_3T1R_2T_D_2R = 1,     // 3T1R step: 2 transmits (KB) - 2 receives (dongle)
    STEP_3T1R_3T_D_3R = 2,     // 3T1R step: 3 transmits (KB) - 3 receives (dongle)
    STEP_3T1R_1R_D_1T,         // 3T1R step: 1 receive (KB) - 1 transmit (dongle)

    P24G_PKT_TYPE_SYNC          = 0,  // 2.4G synchronization packet type
    P24G_PKT_TYPE_KB            = 0,  // 2.4G keyboard packet type
    P24G_PKT_TYPE_MS            = 1,  // 2.4G mouse packet type
    P24G_PKT_TYPE_SPP           = 2,  // 2.4G SPP packet type
    P24G_PKT_TYPE_LL            = 3,  // 2.4G link layer packet type
    P24G_PKT_TYPE_PENDING_SPP   = 2,  // 2.4G pending SPP packet type

    STEP_2T2R_NONE = 0xC0,     // No 2T2R (2 Transmit 2 Receive) step active
    STEP_2T2R_1T = 0,          // 2T2R step: 1 transmit
    STEP_2T2R_2T = 1,          // 2T2R step: 2 transmits

}tpsll_stack_e;

typedef struct
{
    unsigned char  btn;
    signed short  x;
    signed short  y; 
    int8_t z_wheel;
    int8_t t_wheel;
} tpsll_mouse_dat_t;

static tpsll_mouse_dat_t last_ms_data = {0, 0, 0, 0, 0};

#if TPSLL_USB_DATA_CACHE_EN
static tpsll_mouse_dat_t lost_ms_data = {0, 0, 0, 0, 0};
#endif

/* LL_FIFO_SIZE, LL_FIFO_NUM moved to d24g_stack_common.h */
_attribute_aligned_(4) uint8_t ll_fifo_b[LL_FIFO_SIZE * LL_FIFO_NUM];

pl_fifo_t ll_fifo = {
    .size = LL_FIFO_SIZE,
    .num = LL_FIFO_NUM,
    .wptr = 0,
    .rptr = 0,
    .p = ll_fifo_b,
};

#define WHEEL_FIFO_SIZE 2
#define WHEEL_FIFO_NUM 8
uint8_t wheel_fifo_b[WHEEL_FIFO_SIZE * WHEEL_FIFO_NUM];
// _attribute_aligned_(4) uint8_t spp_fifo_b[SPP_FIFO_SIZE * SPP_FIFO_NUM];

pl_fifo_t wheel_fifo = {
    .size = WHEEL_FIFO_SIZE,
    .num = WHEEL_FIFO_NUM,
    .wptr = 0,
    .rptr = 0,
    .p = wheel_fifo_b,
};

#if TPSLL_WHEEL_TEST_EN
static int8_t g_wheel_test = 20;
static uint32_t g_test_tick = 0;
#endif

#define PAIRING_CHN_NUM 2

#define PAIRING_CHN_TAB_DFT     {2, 34}
//static unsigned char pairing_chn_tab[PAIRING_CHN_NUM] = PAIRING_CHN_TAB_DFT;
//static unsigned char pairing_chn_idx = 0;

/* PAIR_CHN_NO moved to d24g_stack_common.h */
/* CONN_CHN_NO moved to d24g_stack_common.h */

static unsigned int curr_cnt_to_usb = 0;

/* RECONN_MAX_CH moved to d24g_stack_common.h */
/* RECONN_CHN_NUM moved to d24g_stack_common.h */

uint8_t reconn_chn_table[RECONN_CHN_NUM];

#if RSSI_SCAN_EN && !TPSLL_NEW_RECON_EN
#define RSSI_SCAN_WIN_US                1000
#define RSSI_SCAN_ROUNDS                10
#endif

#if TPSLL_FHSS_EN
#define RSSI_SCAN_TOP_CHANNEL_COUNT     TPSLL_FHSS_CHN_MAX_NUM
#else
#define RSSI_SCAN_TOP_CHANNEL_COUNT               4
#endif


/* ENFORCE_CHANNEL_SPACING moved to d24g_stack_common.h */
/* MIN_CHANNEL_SPACING moved to d24g_stack_common.h */

typedef enum
{
    STATE_SCAN_IDLE = 0,
    STATE_SCAN_RUNNING,
    STATE_SCAN_DONE,
} RSSI_SCAN_STATE;

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

uint8_t tpsll_fill_spp_data(void);

static uint16_t dbg_io[DBG_IO_IDX_MAX] = {0, 0, 0, 0, 0, 0, 0, 0};

static bool print_chan_table = false;

uint32_t rssi_result[RSSI_SCAN_CHANNEL_COUNT] = {0};
uint16_t scan_counts[RSSI_SCAN_CHANNEL_COUNT] = {0};
uint16_t g_chn_rssi_max[RSSI_SCAN_CHANNEL_COUNT] = {0};

#define REG_RSSI_MAX_BB REG_ADDR8(REG_TL_MODEM_BASE_ADDR + 0x62) //rssi_max_bb
#define CLR_REG_RSSI_MAX_BB()       REG_ADDR8(REG_TL_MODEM_BASE_ADDR + 0x24) = 0x04
/* REG_RXCHN_EN moved to d24g_stack_common.h */

static signed char g_pairing_rssi_threshold = PAIRING_RSSI_THRESHOLD_DFT;

volatile uint8_t current_channel = 0;
volatile uint16_t current_round = 0;
volatile RSSI_SCAN_STATE scan_state = STATE_SCAN_IDLE;

void app_d24g_set_report_rate(uint8_t rr);
void rf_select_best_channels(uint8_t *out_channels, uint8_t num);

#if TPSLL_FHSS_EN
static volatile uint32_t g_hop_map_cur = 0; //bit mask in fhss channel table
static volatile uint32_t g_hop_map_next = 0; //bit mask in fhss channel table
static uint16_t g_fhss_pkt_expect[40];
static uint16_t g_fhss_pkt_err[40];
static uint8_t g_fhss_chn_per[TPSLL_FHSS_CHN_MAX_NUM];
#if TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_ADJ_EN
static uint8_t g_fhss_chn_per_cntdwn_slow_weight[TPSLL_FHSS_CHN_MAX_NUM];
#endif
static uint8_t g_hop_chn_idx_tab_next[TPSLL_HOP_CHN_NUM];

// static uint8_t g_fhss_valid_chn_tab[TPSLL_FHSS_CHN_MAX_NUM] = { /*(n)wifi*/
//     /*37*/0, 1, 2, 3,/*4(@1)*/4,5,/*6,7(2)*/8,/*9(3)*/10,
//     /*38,11(4)*/12,/*13(5)*/14,/*15,16(@6)*/15,16,17,/*18(7)*/
//     19,/*20,21(8)*/21,22,/*23(9)*/24,/*25,26(10)*/27,
//     /*28(@11)*/28,29,/*30,31(12)*/32,/*33(13)*/34,35,36/*39*/
// };

// static uint8_t g_fhss_valid_chn_tab[TPSLL_FHSS_CHN_MAX_NUM] = { /*(n)wifi*/
//     /*4(@1)*/4,5,/*6,7(2)*/8,/*9(3)*/10,
//     /*38,11(4)*/12,/*13(5)*/14,/*15,16(@6)*/15,16,17,/*18(7)*/
//     19,/*20,21(8)*/21,22,/*23(9)*/24,/*25,26(10)*/27,    /*37*/0, 1, 2, 3,
//     /*28(@11)*/28,29,/*30,31(12)*/32,/*33(13)*/34,35,36/*39*/
// };
// static uint8_t g_fhss_valid_chn_tab[TPSLL_FHSS_CHN_MAX_NUM] = { /*(n)wifi*/
    
//     /*38,11(4)*/12,/*13(5)*/14,/*15,16(@6)*/15,16,17,/*18(7)*/
//     19,/*20,21(8)*/21,22,/*23(9)*/24,/*25,26(10)*/27,    /*37*/0, 1, 2, 3,
//     /*28(@11)*/28,29,/*30,31(12)*/32,/*33(13)*/34,35,36,/*39*//*4(@1)*/4,5,/*6,7(2)*/8,/*9(3)*/10
// };
// static uint8_t g_fhss_valid_chn_tab[TPSLL_FHSS_CHN_MAX_NUM] = { /*(n)wifi*/
//     0, 1, 0, 1,
//     0, 1, 0, 1,
//     0, 1, 0, 1,
//     0, 1, 0, 1,
//     0, 1, 0, 1,
//     0, 1, 35,36
// };
// static uint8_t g_fhss_valid_chn_tab[TPSLL_FHSS_CHN_MAX_NUM] = { /*(n)wifi*/
//     36, 36, 36, 36,
//     36, 36, 36, 36,
//     36, 36, 36, 36,
//     36, 36, 36, 36,
//     36, 36, 36, 36,
//     36, 36, 36, 36
// };

#if 0
static int8_t g_fhss_valid_chn_tab[TPSLL_FHSS_CHN_MAX_NUM] = { /*(n)wifi*/
    0,17, 1,19, 2,21, 3,22, 4,24, 5,27,
    8,28,10,29,12,32,14,34,15,35,16,36
};

static uint8_t invalid_chnl_tab[] = {
    6,  7,  9,  11, 13,
    18, 20, 23, 25, 26,
    30, 31, 33, 37, 38, 39
};
#else
static int8_t g_fhss_valid_chn_tab[TPSLL_FHSS_CHN_MAX_NUM] = { /*(n)wifi*/
    0, 19,  1, 21,  2, 22,  4, 24,  5, 27,  7, 28,
    8, 29, 12, 32, 14, 33, 15, 34, 16, 35, 17, 36
};
#define  NON_8K_CHN_NUM  32

#if !TPSLL_N8K_FHSS_EN
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

#if TPSLL_AVOID_WIFI_CHN_EN
typedef struct
{
    uint8_t num;
    uint8_t table[10]; /* p24g valid channel */
} p24g_chn_t;

typedef struct
{
    uint16_t rssi_sum[TPSLL_WIFI_CHANNEL_COUNT];
    p24g_chn_t valid_chn[TPSLL_WIFI_CHANNEL_COUNT];
    uint8_t rssi_sorted_idx[TPSLL_WIFI_CHANNEL_COUNT];
    uint8_t valid_chn_score[TPSLL_FHSS_CHN_MAX_NUM];
    bool chn_score_valid;
} tpsll_wifi_chn_t;

static tpsll_wifi_chn_t wifi_chn_inf = {
    .valid_chn = {
    /*1*/ {.num = 7, .table = {0, 1, 2, 4, 5, 7, 8}},
    /*2*/ {.num = 5, .table = {2, 4, 5, 7, 8}},
    /*3*/ {.num = 5, .table = {4, 5, 7, 8, 12}},
    /*4*/ {.num = 5, .table = {7, 8, 12, 14, 15}},
    /*5*/ {.num = 5, .table = {12, 14, 15, 16, 17}},
    /*6*/ {.num = 6, .table = {12, 14, 15, 16, 17, 19}},
    /*7*/ {.num = 7, .table = {14, 15, 16, 17, 19, 21, 22}},
    /*8*/ {.num = 6, .table = {16, 17, 19, 21, 22, 24}},
    /*9*/ {.num = 6, .table = {19, 21, 22, 24, 27, 28}},
    /*10*/{.num = 6, .table = {21, 22, 24, 27, 28, 29}},
    /*11*/{.num = 6, .table = {24, 27, 28, 29, 32, 33}},
    /*12*/{.num = 7, .table = {27, 28, 29, 32, 33, 34, 35}},
    /*13*/{.num = 7, .table = {28, 29, 32, 33, 34, 35, 36}},
    },
};
#endif /* end of TPSLL_AVOID_WIFI_CHN_EN */

#if TPSLL_RF_SCAN_EN
typedef enum
{
    RF_SCAN_STATE_IDLE       = BIT(0),
    RF_SCAN_STATE_RUNNING    = BIT(1),
    RF_SCAN_STATE_PAUSE      = BIT(2),
    RF_SCAN_STATE_DONE       = BIT(3),
} tpsll_rf_scan_step_e;

typedef struct
{
    uint32_t start_tick;
    uint32_t done_tick;
    // uint32_t rssi_result[TPSLL_RF_SCAN_CHN_TOTAL];
    // uint16_t scan_counts[TPSLL_RF_SCAN_CHN_TOTAL];
    uint16_t chn_rssi_max[TPSLL_RF_SCAN_CHN_TOTAL];
    volatile uint16_t cur_round;
    volatile uint8_t cur_chn;
    volatile uint8_t state;
    uint8_t select_chn_table[TPSLL_RF_SCAN_SELECT_CHN_NUM];
    uint8_t select_chn_ready;
    uint8_t temp_switch_chn;
} tpsll_rf_scan_ctx_t;

static tpsll_rf_scan_ctx_t rf_scan_ctx = {
    .state = RF_SCAN_STATE_IDLE,
    .select_chn_table = {0, 0, 0},
    .temp_switch_chn = 0xff,
};
#endif /* end of TPSLL_RF_SCAN_EN */

#if TPSLL_NEW_PAIR_EN
static int8_t g_pair_valid_chn_tab[TPSLL_PAIR_CHN_MAX_NUM] = {
    0, 21, 8, 22, 34, 35, 17, 36
};
#endif

static inline int is_channel_valid(int ch)
{
    TL_UNUSED(ch);
    #if !TPSLL_NEW_RECON_EN
    // return !((ch == 0) || (ch >= 10 && ch <= 11) || (ch >= 36 && ch <= 39));
    for (uint8_t idx = 0; idx < sizeof(invalid_chnl_tab); idx++) {
        if (invalid_chnl_tab[idx] == ch) {
            return false;
        }
    }
    #endif

    return true;
}
#else
static inline int is_channel_valid(int ch)
{
    return !((ch >= 0 && ch <= 1) || (ch >= 9 && ch <= 12) || (ch >= 35 && ch <= 39));
}
#endif


#if TPSLL_PACKET_LOSS_COMPENSATE_EN

#if 1
// Implement fabsf using RISC-V inline assembly
static float fabsf_asm(float x) {

    float result;
    // Perform single-precision floating-point absolute value operation using the fabs.s instruction
    // The "f" constraint indicates the use of floating-point registers
    __asm__ volatile (
        "fabs.s %0, %1"          // Instruction: fabs.s target register, source register
        : "=f"(result)           // Output operand: %0 must be a floating-point register
        : "f"(x)                 // Input operand: %1 must be a floating-point register
        :                         // No clobber list to declare
    );

    return result;
}
#else
//  Reference implementation using bit manipulation
float fabsf_bitwise(float x) {
    // Interpret floating-point number x as a 32-bit unsigned integer by binary bits
    unsigned int bits = *(unsigned int*)&x;
    
    // Clear the highest bit (sign bit), 0x7FFFFFFF is 0b01111111111111111111111111111111
    bits = bits & 0x7FFFFFFF;
    
    // Reinterpret the modified binary bits as a floating-point number and return
    return *(float*)&bits;
}
#endif

/**
 * Velocity Kalman filter structure
 *  State vector: [vx, vy]^T (2D velocity)
 */
typedef struct {
    float vx;          // X-axis velocity estimation
    float vy;          // Y-axis velocity estimation
    float P[2][2];     // State covariance matrix (2x2)
    float process_noise; // Process noise intensity (variance of velocity change rate, unit: position unit^2/time^3)
    float meas_noise;    // Displacement measurement noise variance (unit: position unit^2)
} KalmanFilterVel;

KalmanFilterVel g_kf;


static void kalman_vel_init(KalmanFilterVel *kf, float init_vx, float init_vy,
                     float process_noise, float meas_noise);

static void kalman_vel_predict(KalmanFilterVel *kf, float dt, float *pred_dx, float *pred_dy);

static void kalman_vel_update(KalmanFilterVel *kf, float dx_meas, float dy_meas, float dt_meas);





static void kalman_vel_init(KalmanFilterVel *kf, float init_vx, float init_vy,
                     float process_noise, float meas_noise) {

    kf->vx = init_vx;
    kf->vy = init_vy;

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            kf->P[i][j] = (i == j) ? 1000.0f : 0.0f;
        }
    }

    kf->process_noise = process_noise; // q
    kf->meas_noise = meas_noise;       // r_d

}

//60ns @96M
static void kalman_vel_predict(KalmanFilterVel *kf, float dt, float *pred_dx, float *pred_dy) {
    // vx_pred = vx, vy_pred = vy

    float q_dt = kf->process_noise * dt;
    kf->P[0][0] += q_dt;
    kf->P[1][1] += q_dt;

    if (pred_dx != NULL && pred_dy != NULL) {
        *pred_dx = kf->vx * dt;
        *pred_dy = kf->vy * dt;
    }
}

static void kalman_vel_update(KalmanFilterVel *kf, float dx_meas, float dy_meas, float dt_meas) {


    float zx = dx_meas / dt_meas;
    float zy = dy_meas / dt_meas;

    float r = kf->meas_noise / (dt_meas * dt_meas);
    float R[2][2] = {{r, 0.0f}, {0.0f, r}};

    float y[2];
    y[0] = zx - kf->vx;
    y[1] = zy - kf->vy;

    float S[2][2];
    S[0][0] = kf->P[0][0] + R[0][0];
    S[0][1] = kf->P[0][1] + R[0][1];
    S[1][0] = kf->P[1][0] + R[1][0];
    S[1][1] = kf->P[1][1] + R[1][1];

    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (fabsf_asm(det) < 1e-9f) {
        return;
    }
    float invS[2][2];
    invS[0][0] =  S[1][1] / det;
    invS[0][1] = -S[0][1] / det;
    invS[1][0] = -S[1][0] / det;
    invS[1][1] =  S[0][0] / det;

    float K[2][2];
    K[0][0] = kf->P[0][0] * invS[0][0] + kf->P[0][1] * invS[1][0];
    K[0][1] = kf->P[0][0] * invS[0][1] + kf->P[0][1] * invS[1][1];
    K[1][0] = kf->P[1][0] * invS[0][0] + kf->P[1][1] * invS[1][0];
    K[1][1] = kf->P[1][0] * invS[0][1] + kf->P[1][1] * invS[1][1];

    kf->vx += K[0][0] * y[0] + K[0][1] * y[1];
    kf->vy += K[1][0] * y[0] + K[1][1] * y[1];

    float IminusK[2][2];
    IminusK[0][0] = 1.0f - K[0][0];
    IminusK[0][1] = -K[0][1];
    IminusK[1][0] = -K[1][0];
    IminusK[1][1] = 1.0f - K[1][1];

    float newP[2][2] = {0};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                newP[i][j] += IminusK[i][k] * kf->P[k][j];
            }
        }
    }

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            kf->P[i][j] = newP[i][j];
        }
    }

}


/**********************************alg test end**************************/
#endif

#if RF_PKT_IDX_DEBUG_EN
static void debug_gpio_with_cnt_pg5(int16_t cnt)
{
    // return;
    if(cnt < 0)
    {
        cnt = -cnt;
    }
    for(uint8_t i = 0; i < cnt; i++)
    {
        ZW_TG(GPIO_PG5);
    }
}

static void debug_gpio_with_cnt4(int16_t cnt)
{
    return;
    if(cnt < 0)
    {
        cnt = -cnt;
    }
    for(uint8_t i = 0; i < cnt; i++)
    {
        ZW_TG(GPIO_PG5);
    }
}

static void debug_gpio_with_cnt_pg6(int16_t cnt)
{

    // return;
    if(cnt < 0)
    {
        cnt = -cnt;
        // cnt = 2;
    }
    for(uint8_t i = 0; i < cnt; i++)
    {
        ZW_TG(GPIO_PG6);
    }
}

static void debug_gpio_with_cnt5(int16_t cnt)
{

    if(cnt < 0)
    {
        cnt = -cnt;
        // cnt = 2;
    }
    for(uint8_t i = 0; i < cnt; i++)
    {
        ZW_TG(GPIO_PG6);
    }
}


static void debug_gpio_with_cnt6(int16_t cnt)
{

    // return;
    if(cnt < 0)
    {
        cnt = -cnt;
        // cnt = 2;
    }
    for(uint8_t i = 0; i < cnt; i++)
    {
        ZW_TG(GPIO_PH1);
    }
}


static void debug_gpio_with_cnt(int16_t cnt)
{
    // return;
    // if(cnt == 1 || cnt  == -1)
    // {
    //     return;
    // }

    if(cnt < 0)
    {
        cnt = -cnt;
    }
    for(uint8_t i = 0; i < cnt; i++)
    {
        ZW_TG(GPIO_PH2);
    }
}



static void debug_gpio_with_cnt1(int16_t cnt)
{
    return;
    for(uint8_t i = 0; i < cnt; i++)
    {
        ZW_TG(GPIO_PG6);
    }
}

static void debug_gpio_with_cnt11(int16_t cnt)
{
    // return;
    for(uint8_t i = 0; i < cnt; i++)
    {
        ZW_TG(GPIO_PG7);
    }
}

static void debug_gpio_with_cnt_ph0(int16_t cnt)
{
    return;
    for(uint8_t i = 0; i < cnt; i++)
    {
        ZW_TG(GPIO_PH0);
    }
}


static void debug_gpio_with_cnt_ph3(int16_t cnt)
{
    // return;
    if(cnt < 0)
    {
        cnt = -cnt;
    }

    for(uint8_t i = 0; i < cnt; i++)
    {
        ZW_TG(GPIO_PH3);
    }
}

static void debug_gpio_with_cnt_ph4(int16_t cnt)
{
    // return;
    if(cnt < 0)
    {
        cnt = -cnt;
    }

    for(uint8_t i = 0; i < cnt; i++)
    {
        ZW_TG(GPIO_PH4);
    }
}
#endif


volatile  int  recon_step=0;
volatile  unsigned int  recon_tick=0;
volatile   unsigned char  kb_led_status=0;


volatile int  rf_rx_fail_cnt=0;
volatile  unsigned char rf_pair_fail_cnt=0;
volatile int rf_rx_length=0;


_attribute_aligned_(4) ll_inf_t  ll_inf;
volatile  unsigned int tick_connect;

volatile int cnt_rx_irq=0;
volatile int cnt_rx_crc_ok=0;
volatile int cnt_rx_d_ok=0;
volatile int cnt_rx_crc_err=0;

volatile int cnt_tx=0;
volatile int cnt_tx_irq=0;

volatile int cnt_to_usb;
volatile int cnt_data_err;

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
    uint8_t chn_tab[TPSLL_HOP_CHN_NUM];
    #endif
    uint16_t crc16;
} __attribute__((packed)) __attribute__((aligned(4))) reconn_rsp_packet_t;

#if TPSLL_8K_CHN_EXPAND_EN || TPSLL_N8K_CHN_EXPAND_EN
static uint8_t chn_table[8] = {0};
#else
uint8_t chn_table[CHN_TABL_LEN] = {8, 17, 22, 32};
#endif
uint8_t chn_list[3] = {0};
uint8_t chn_mask = 0x0F;

#if AFH_ENABLE

int8_t chn_quality[MAX_CHANNELS] = {0};

uint8_t switch_countdown = 0; 
uint8_t next_mask = 0x0F;
static uint8_t chn_reverse_table[40];
static uint8_t pending_disable_mask = 0;

#define INVALID_GEN           0xFF
#define MAX_VALID_GEN         0xFE

static uint8_t kick_generation = 0;
// static uint8_t kick_gen[MAX_CHANNELS] = {INVALID_GEN, INVALID_GEN, INVALID_GEN, INVALID_GEN};
uint8_t kick_gen[MAX_CHANNELS] = {INVALID_GEN, INVALID_GEN, INVALID_GEN, INVALID_GEN};

#endif

unsigned char g_usb_sof_start = 0;
static unsigned char g_local_data_en = 0;

#define DEBUG_SYNC_DIFF_TIME    0

#if DEBUG_SYNC_DIFF_TIME
static int32_t g_diff_us = 0; //debug
#endif

// extern unsigned int g_square_side_len;
extern unsigned char app_consume_key_report_to_usb(unsigned char *buf);
extern unsigned char app_system_key_report_to_usb(unsigned char *buf);
extern unsigned char app_all_key_report_to_usb(unsigned char *buf);
void app_fill_paring_pkt(void);
void app_set_next_stimer_anchor(void);


void tpsll_stack_get_app_var(tpsll_app_var_t *p_app_var, tpsll_app_var_id_e var_id)
{
    if (s_app_var_cb != NULL && p_app_var != NULL) {
        s_app_var_cb(p_app_var, TPSLL_ACCESS_GET, var_id);
    }
}

void tpsll_stack_set_app_var(tpsll_app_var_t *p_app_var, tpsll_app_var_id_e var_id)
{
    if (s_app_var_cb != NULL && p_app_var != NULL) {
        s_app_var_cb((tpsll_app_var_t *)p_app_var, TPSLL_ACCESS_SET, var_id);
    }
}

static inline bool tpsll_is_report_rate_valid(uint8_t rr)
{
    return (rr && ((rr & (rr - 1)) == 0) && (rr & REPORT_RATE_MASK));
}

void app_square_use_local_data(unsigned int side_len)
{
    g_local_data_en = 1;
    // g_square_side_len = side_len;

    tpsll_app_var_t var = {
        .square_side_len = side_len,
    };  

    tpsll_stack_set_app_var(&var, TPSLL_APP_VAR_SQUARE_SIDE_LEN);
    
}

static inline int tabs(int x)
{
    return x < 0 ? -x : x;
}

_attribute_ram_code_sec_ uint8_t tpsll_reset_spp_fifo(void)
{
    unsigned int r = core_interrupt_disable();
    spp_fifo.wptr = 0;
    spp_fifo.rptr = 0;
    ll_ctx.spp_flag = false;
    core_restore_interrupt(r);
    return TLK_SUCCESS;
}

//static inline int tick1_exceed_tick2(unsigned int tick1, unsigned int tick2)
//{
//    return (unsigned int)(tick1 - tick2) < 0xc0000000; // 3/4 of max, about 130 second
//}


// _attribute_ram_code_sec_ static void p24g_evt_cb(tpsll_evt_t evt, void *data, uint16_t len)
// {
//     if (evt < TPSLL_EVT_MAX && p24g_callbacks[evt]) {
//         p24g_callbacks[evt](evt, data, len);
//     }
// }

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
        if ((uint8_t)evt_elem.evt < (uint8_t)TPSLL_INTER_EVT_MAX) {
            cb = p24g_callbacks[evt_elem.evt];
        }
        if (cb) {
            cb(evt_elem.evt, (evt_elem.len ? evt_elem.data : NULL), evt_elem.len);
        }
        processed++;
    }
}

static void tpsll_stack_event_handler(tpsll_evt_t evt, void *data, uint16_t len)
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
        if (spp_frag_total > 0)
        {
            spp_frag_acked++;

            if (spp_frag_acked < spp_frag_total)
            {
                return;
            }

            if (spp_cb)
            {
                spp_cb(spp_original_data, spp_original_len, true, user_arg);
            }

            spp_frag_total     = 0;
            spp_frag_acked      = 0;
            spp_original_data   = NULL;
            spp_original_len    = 0;
            spp_sending_busy   = false;
        }
        else
        {
            if (spp_cb)
            {
            #if DONGLE_SPP_FRAG_EN
                spp_cb(payload + 1, pdu_len - 1, true, user_arg);
            #else
                spp_cb(payload, pdu_len, true, user_arg);
            #endif
            }
        }
        break;
    }

    default:
        break;
    }
}

static inline void app_rf_set_rx_wait(unsigned char us)
{
    write_reg16(0x170206, us);
}

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
    #if TPSLL_8K_CHN_EXPAND_EN
    chn_list[3] = chn_table[(seed + 3) % TPSLL_HOP_CHN_NUM];
    chn_list[4] = chn_table[(seed + 4) % TPSLL_HOP_CHN_NUM];
    chn_list[5] = chn_table[(seed + 5) % TPSLL_HOP_CHN_NUM];
    #endif
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

_attribute_ram_code_sec_ void check_report_mode_by_per(void)
{
    return;
    if(auto_draw_flag)
    {
        return;
    }
    if (ll_ctx.per_est >= PER_HIGH_TH)
    {
        if (ll_ctx.report_mode != REPORT_MODE_NORMAL)
        {
            ll_ctx.report_mode = REPORT_MODE_NORMAL;
            ll_ctx.switch_special_cnt = 2;
            //DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
            tlkapi_send_string_data(APP_LOG_EN, "report mode change to normal", &ll_ctx.per_est, 2);
        }
    }
    else if (ll_ctx.per_est <= PER_LOW_TH)
    {
        if (ll_ctx.report_mode != REPORT_MODE_FAST)
        {
            ll_ctx.report_mode = REPORT_MODE_FAST;
            ll_ctx.switch_special_cnt = 1;
            //DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
            tlkapi_send_string_data(APP_LOG_EN, "report mode change to fast", &ll_ctx.per_est, 2);
            //DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
        }
    }
}

//Note: The per function only works when AFH is enabled.
static short per_err = 0;
//250ns d25f @96MHz
_attribute_ram_code_sec_ void per_update(bool success)
{
    uint16_t err = success ? 0 : 1000; // 0% or 100%

    //DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);

    // IIR update: per_est = per_est*(1-α) + err*α

    short diff = err - ll_ctx.per_est;

    per_err += PER_ALPHA_NUM * diff;

    uint16_t delta = per_err / PER_ALPHA_DEN;
    per_err -= delta * PER_ALPHA_DEN;

    ll_ctx.per_est += delta;

    //DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
}

#if AFH_ENABLE

_attribute_ram_code_sec_ static void app_afh_init(void)
{
    tmemset(chn_quality, 0, sizeof(chn_quality));
    chn_mask = 0x0F;
    next_mask = 0x0F;
    kick_generation = 0;
    switch_countdown = 0;
    pending_disable_mask = 0;
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        kick_gen[i] = INVALID_GEN;
    }
}

_attribute_ram_code_sec_ void init_reverse_table(void) {
    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        chn_reverse_table[chn_table[i]] = i;
    }
}


_attribute_ram_code_sec_ void update_channel_quality(uint8_t chn_no, bool success) {

    uint8_t chn_idx = chn_reverse_table[chn_no];

    // if(success)
    // {
    //     per_update(true);
    // }
    // else
    // {
    //     per_update(false);
    // }

    per_update(success);

    if (pending_disable_mask & (1 << chn_idx)) {
        return;
    }

    // if(chn_idx >= MAX_CHANNELS)
    // {
    //  return;
    // }

    if (success) {
        if (chn_quality[chn_idx] < MAX_QUALITY) chn_quality[chn_idx]++;
    } else {
        if (chn_quality[chn_idx] > MIN_QUALITY) chn_quality[chn_idx]--;
    }
}


static inline bool is_int8_earlier(uint8_t a, uint8_t b) {
    return (uint8_t)(b - a) < 128;
}

uint8_t b_per[16] = "per";
_attribute_ram_code_sec_ void check_and_update_mask(void)
{
    uint8_t new_mask = 0;
    uint8_t good_count = 0;

    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        if (chn_quality[i] >= QUALITY_THRESH) {
            new_mask |= (1 << i);
            good_count++;
            kick_gen[i] = INVALID_GEN;
        } else if (chn_mask & (1 << i)) {
            if (kick_generation == MAX_VALID_GEN) {
                for (uint8_t k = 0; k < MAX_CHANNELS; k++) {
                    kick_gen[k] = INVALID_GEN;
                }
                kick_generation = 0;
            }
            kick_gen[i] = kick_generation++;
        }
    }

    while (good_count < MIN_VALID_CHANNELS) {
        uint8_t oldest_idx = 0xFF;
        uint8_t oldest_gen = INVALID_GEN;

        for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
            if (!(new_mask & (1 << i)) && kick_gen[i] != INVALID_GEN) {
                if (oldest_gen == INVALID_GEN || is_int8_earlier(kick_gen[i], oldest_gen)) {
                    oldest_gen = kick_gen[i];
                    oldest_idx = i;
                }
            }
        }

        if (oldest_idx != 0xFF)
        {
            new_mask |= (1 << oldest_idx);
            chn_quality[oldest_idx] = 0;
            kick_gen[oldest_idx] = INVALID_GEN;
            good_count++;
        }
        else
        {
            // fallback: Forcefully supplement the first disabled channel
            for (uint8_t j = 0; j < MAX_CHANNELS && good_count < MIN_VALID_CHANNELS; j++)
            {
                if ((new_mask & (1 << j)) == 0)
                {
                    new_mask |= (1 << j);
                    chn_quality[j] = 0;
                    good_count++;
                }
            }
            break;
        }
    }

    uint8_t removed = chn_mask & (~new_mask);
    if (removed) {
        pending_disable_mask |= removed;
    }

    if (new_mask != chn_mask) {
        next_mask = new_mask;
        switch_countdown = COUNTDOWN_BIT4_MAX;
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
    }

static uint16_t max_per = 0x0;
if(max_per < ll_ctx.per_est)
{
    max_per = ll_ctx.per_est;
    // tlkapi_send_string_data(APP_LOG_EN, (char*)b_per, &ll_ctx.per_est, 2);
}
}


uint8_t b_chn_mask_tag[16] = "d chn_m chg";
volatile uint8_t b_chn_mask_all[4];
_attribute_ram_code_sec_ static void fill_chn_mask(void)
{
    if (switch_countdown > 0)
    {
        rf_tx_packet.buf[0] = (rf_tx_packet.buf[0] & 0x0F) | ((next_mask & 0x0F) << 4);
        rf_tx_packet.cnt = switch_countdown & 0x0F;

        switch_countdown--;
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
        if (switch_countdown == 0)
        {
            b_chn_mask_all[0] = chn_mask;
            b_chn_mask_all[1] = next_mask;
            b_chn_mask_all[2] = ll_ctx.fno & 0xff;
            b_chn_mask_all[3] = (ll_ctx.fno >> 8) & 0xff;

            chn_mask = next_mask;
            pending_disable_mask = 0;
            chn_mask_change = true;
            // tlkapi_send_string_data(APP_LOG_EN, (char*)b_chn_mask_tag, b_chn_mask_all, 4);
            // tlkapi_send_string_data(APP_LOG_EN, "chn idx rssi sort(s->b)", channel_idx, 40);
        }
        // rf_tx_packet.buf[1] = 0x99;
    }
    else
    {
        rf_tx_packet.buf[0] = (rf_tx_packet.buf[0] & 0x0F) | ((chn_mask & 0x0F) << 4);
        rf_tx_packet.cnt = 0;
        // rf_tx_packet.buf[1] = 0x0;
    }
}
#endif

#if TPSLL_N8K_FHSS_EN

_attribute_ram_code_sec_ void tpsll_n8k_channel_update_per(void)
{
    //calculate the per of current hopping-channels
    int h = 0;
    // uint8_t per = 0;
    uint8_t hop_chn_per[TPSLL_HOP_CHN_NUM] = {TPSLL_FHSS_CHN_PER_GOOD_TH, TPSLL_FHSS_CHN_PER_GOOD_TH, TPSLL_FHSS_CHN_PER_GOOD_TH};

    // bool b_flag = false;
    tmemset((void *)hop_chn_per, 0, sizeof(hop_chn_per));

    for (int32_t i = 0; (i < TPSLL_FHSS_CHN_MAX_NUM) && (h < TPSLL_HOP_CHN_NUM); i++) {
        if (g_hop_map_cur & BIT(i)) {
            uint8_t chn = g_fhss_valid_chn_tab[i];

            if (g_fhss_pkt_expect[chn]) {
                hop_chn_per[h] = g_fhss_pkt_err[chn] * 100 / g_fhss_pkt_expect[chn];

                if (hop_chn_per[h] > 100) {
                    hop_chn_per[h] = 100;
                }
            } else {
                hop_chn_per[h] = 0;
            }
            #if 0
            // ZW_TG(GPIO_PH0);
            if(hop_chn_per[h] > TPSLL_FHSS_CHN_PER_GOOD_TH)
            {
                b_flag = true;
                ZW_TG(GPIO_PG5);
                tlkapi_printf(APP_LOG_EN, "per--%d %d :0x%08X-0x%08X\n", g_fhss_pkt_err[chn], g_fhss_pkt_expect[chn], i, hop_chn_per[h]);
            }
            #endif

            h++;
        }
    }

    if (ll_ctx.tbl_update) {
        // ZW_TG(GPIO_PG6);
        // ZW_TG(GPIO_PG6);
        return;
    }
    // ZW_TG(GPIO_PG6);

    ll_ctx.tbl_update = true;
    // tlkapi_send_string_data(APP_LOG_EN, "total_per", &g_fhss_chn_per, sizeof(g_fhss_chn_per));


    //update all channel per
    int k = 0;
    for (int i = 0; i < TPSLL_FHSS_CHN_MAX_NUM; i++)
    {
        
        if ((g_hop_map_cur & BIT(i)) && k < TPSLL_HOP_CHN_NUM)
        {
            // uint8_t pper = g_fhss_chn_per[i];
            if (g_fhss_chn_per[i] > TPSLL_FHSS_CHN_HISTORY_SCORE_TH)
            {
                g_fhss_chn_per[i] = (g_fhss_chn_per[i] * 7 + hop_chn_per[k]) >> 3;
                // if(hop_chn_per[k] > 20)
                // {
                //     tlkapi_printf(APP_LOG_EN, "bnew_per %d %d %d %d\n", i, pper, hop_chn_per[k], g_fhss_chn_per[i]);
                // }
            }
            else
            {
                g_fhss_chn_per[i] = (g_fhss_chn_per[i] * 1 + hop_chn_per[k]) >> 1;
                // if(hop_chn_per[k] > 20)
                // {
                //     tlkapi_printf(APP_LOG_EN, "snew_per %d %d %d %d\n", i, pper, hop_chn_per[k], g_fhss_chn_per[i]);
                // }

            }
            k++;
        }
        else
        {
            #if TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_ADJ_EN
            if (g_fhss_chn_per_cntdwn_slow_weight[i] < TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_DEFAULT) {
                g_fhss_chn_per_cntdwn_slow_weight[i] = TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_DEFAULT;
            } else if (g_fhss_chn_per_cntdwn_slow_weight[i] > TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_MAX) {
                g_fhss_chn_per_cntdwn_slow_weight[i] = TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_MAX;
            }
            g_fhss_chn_per[i] = (g_fhss_chn_per[i] * g_fhss_chn_per_cntdwn_slow_weight[i]) / (g_fhss_chn_per_cntdwn_slow_weight[i] + 1);
            #else
            g_fhss_chn_per[i] = (g_fhss_chn_per[i] * 15) >> 4;
            #endif
            if (g_fhss_chn_per[i] < TPSLL_FHSS_CHN_PER_GOOD_TH - 1)
            {
                g_fhss_chn_per[i] = TPSLL_FHSS_CHN_PER_GOOD_TH - 1;
            }
        }
    }

    #if 0
    if(b_flag)
    {
        ZW_TG(GPIO_PG6);
        tlkapi_send_string_data(APP_LOG_EN, "aper", &g_fhss_chn_per, sizeof(g_fhss_chn_per));
    }
    #endif

    tpsll_channel_statistic(0xff, 0);
}
#endif


#if TPSLL_FHSS_EN
/**
 * The channel list for current interval has already been generated in advance before this point
 */
static inline bool tpsll_is_afh_slot(void)
{
    // return ((ll_ctx.fno & TPSLL_FHSS_PER_T_FNO_NUM) == 0);
    static uint32_t last;


    #if TPSLL_N8K_FHSS_EN
    uint32_t cur = ll_ctx.fno >> g_afh_cfg.afh_shift;
    #else
    uint32_t cur = ll_ctx.fno >> TPSLL_FHSS_AFH_SLOT_SHIFT;
    #endif
    uint32_t diff = cur ^ last;

    last = cur;

    return diff != 0;
}

static inline bool tpsll_is_channel_update_slot(void)
{
    #if TPSLL_N8K_FHSS_EN
    return ((ll_ctx.fno & g_afh_cfg.afh_mask) == g_afh_cfg.update_pos);
    #else
    return ((ll_ctx.fno & TPSLL_FHSS_PER_T_FNO_NUM) == (TPSLL_FHSS_PER_T_FNO_NUM - TPSLL_FHSS_PER_AFH_WAIT_FNO_NUM));
    #endif
}

_attribute_ram_code_sec_ void tpsll_afh_channel_gen_idx_tab(uint32_t map, uint8_t *tab)
{
    // u32 chm[2] = {0, 0};
    int32_t h = 0;

    for (int idx = 0; (idx < TPSLL_FHSS_CHN_MAX_NUM) && (h < TPSLL_HOP_CHN_NUM); idx++)
    {
        if (map & (BIT(idx)))
        {
            tab[h] = idx;
            h++;
        }
    }
    // tlkapi_printf(1, "(m-0x%08x)hop_chns_idx:%d-%d-%d\n", map, tab[0], tab[1], tab[2]);
}

_attribute_ram_code_sec_ void tpsll_channel_statistic(uint8_t chn_idx, bool err)
{//index in fhss channel table
    if (chn_idx > 39) {
        tmemset(g_fhss_pkt_expect, 0, sizeof(g_fhss_pkt_expect));
        tmemset(g_fhss_pkt_err, 0, sizeof(g_fhss_pkt_err));
        return;
    }

    #if !TPSLL_KEEP_ALIVE_PER_EN 
    if (ll_ctx.is_keep_alive) {
        return;
    }
    #else
//     if (ll_ctx.is_keep_alive && ((ll_ctx.step_3t1r != STEP_3T1R_3T_D_3R)
//             || (ll_ctx.keep_alive_expect_fno != ll_ctx.keep_alive_expect_fno_last))) {
//         return;
//     }

    #if TPSLL_PM_OPTM_EN && !TPSLL_KB_KEEP_ALIVE_PER_EN
    if ((ll_inf.paired_dev_type & DEVICE_TYPE_KB) && ll_ctx.is_keep_alive) {
        return;
    }
    #endif

    if (ll_ctx.is_keep_alive) {
        if (
            #if TPSLL_KB_KEEP_ALIVE_PER_EN
            ((ll_inf.paired_dev_type & DEVICE_TYPE_KB) ? (
                #if TPSLL_KEEP_ALIVE2_NOTIFY_EN
                ll_ctx.is_keep_alive2_active ||
                #endif
                (ll_ctx.step_3t1r != STEP_3T1R_2T_D_2R)) : (ll_ctx.step_3t1r != STEP_3T1R_3T_D_3R)) ||
            #else
            (ll_ctx.step_3t1r != STEP_3T1R_3T_D_3R) ||
            #endif
            (ll_ctx.keep_alive_expect_fno != ll_ctx.keep_alive_expect_fno_last))
        {
            return;
        }
    }
    #if TPSLL_KB_ACTIVE_PM_OPTM_EN
    else if (ll_inf.paired_dev_type & DEVICE_TYPE_KB) {
        if (ll_ctx.fno_last & ll_ctx.kb_alive_fno_msk) {
            return;
        } else if (ll_ctx.kb_alive_fno_msk && (ll_ctx.report_rate & REPORT_RATE_8K) && (ll_ctx.step_3t1r != STEP_3T1R_3T_D_3R)) {
            return;
        }
    }
    #endif /* end of TPSLL_KB_ACTIVE_PM_OPTM_EN */
    #endif

    g_fhss_pkt_expect[chn_idx]++;

    if (err) {
        g_fhss_pkt_err[chn_idx]++;
    }
}

_attribute_ram_code_sec_ void tpsll_channel_update_per(void)
{
    // #if APP_LOG_EN //for debug, keep it
    // uint8_t print_buf[600] = {0};
    // uint16_t print_idx = 0;
    // #endif

    #if TPSLL_N8K_FHSS_EN
    if(!(ll_ctx.report_rate & REPORT_RATE_8K))
    {
        tpsll_n8k_channel_update_per();
        return;
    }
    #endif

#if TPSLL_8K_CHN_EXPAND_EN
    uint8_t hop_chn_per[TPSLL_HOP_CHN_NUM] = {TPSLL_FHSS_CHN_PER_GOOD_TH, TPSLL_FHSS_CHN_PER_GOOD_TH, TPSLL_FHSS_CHN_PER_GOOD_TH,
                                                TPSLL_FHSS_CHN_PER_GOOD_TH, TPSLL_FHSS_CHN_PER_GOOD_TH, TPSLL_FHSS_CHN_PER_GOOD_TH};
#else
    uint8_t hop_chn_per[TPSLL_HOP_CHN_NUM] = {TPSLL_FHSS_CHN_PER_GOOD_TH, TPSLL_FHSS_CHN_PER_GOOD_TH, TPSLL_FHSS_CHN_PER_GOOD_TH};
#endif

    //calculate the per of current hopping-channels
    int h = 0;
    // uint8_t per = 0;

    // tmemset((void *)hop_chn_per, TPSLL_FHSS_CHN_PER_GOOD_TH, sizeof(hop_chn_per));

    #if TPSLL_PM_OPTM_EN && !TPSLL_KB_KEEP_ALIVE_PER_EN
    if ((ll_inf.paired_dev_type & DEVICE_TYPE_KB) && ll_ctx.is_keep_alive) {
        return;
    }
    #endif

    #if TPSLL_RF_SCAN_EN && !TPSLL_KEEP_ALIVE_PER_EN
    if (!ll_ctx.is_keep_alive)
    #endif
    {
        for (int32_t i = 0; (i < TPSLL_FHSS_CHN_MAX_NUM) && (h < TPSLL_HOP_CHN_NUM); i++) {
            if (g_hop_map_cur & BIT(i)) {
                uint8_t chn = g_fhss_valid_chn_tab[i];

                if (g_fhss_pkt_expect[chn]) {
                    hop_chn_per[h] = g_fhss_pkt_err[chn] * 100 / g_fhss_pkt_expect[chn];

                    if (hop_chn_per[h] > 100) {
                        hop_chn_per[h] = 100;
                    }
                } else {
                    hop_chn_per[h] = 0xff;
                }

                h++;
            }
        }
    }

    if (ll_ctx.tbl_update) {
        return;
    }

    ll_ctx.tbl_update = true;


    //update all channel per
    int k = 0;
    for (int i = 0; i < TPSLL_FHSS_CHN_MAX_NUM; i++)
    {
        if (hop_chn_per[k] > 100) {
            k++;
            continue;
        }

        if ((g_hop_map_cur & BIT(i)) && (k < TPSLL_HOP_CHN_NUM))
        {
            #if !TPSLL_KEEP_ALIVE_PER_EN
            if (!ll_ctx.is_keep_alive)
            #endif
            {
                if (g_fhss_chn_per[i] > TPSLL_FHSS_CHN_HISTORY_SCORE_TH)
                {
                    g_fhss_chn_per[i] = (g_fhss_chn_per[i] * 7 + hop_chn_per[k]) >> 3;
                }
                else
                {
                    g_fhss_chn_per[i] = (g_fhss_chn_per[i] * 1 + hop_chn_per[k]) >> 1;
                }
                k++;
            }
        }
        else
        {
            #if TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_ADJ_EN
            {
                if (g_fhss_chn_per_cntdwn_slow_weight[i] < TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_DEFAULT) {
                    g_fhss_chn_per_cntdwn_slow_weight[i] = TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_DEFAULT;
                } else if (g_fhss_chn_per_cntdwn_slow_weight[i] > TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_MAX) {
                    g_fhss_chn_per_cntdwn_slow_weight[i] = TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_MAX;
                }
            }

            #if 0//TPSLL_KEEP_ALIVE_PER_EN
            int8_t diff = g_fhss_chn_per[i] - TPSLL_FHSS_CHN_PER_GOOD_TH;

            if (diff > 0) {
                g_fhss_chn_per[i] = ((uint16_t)g_fhss_chn_per[i] * g_fhss_chn_per_cntdwn_slow_weight[i]) / (g_fhss_chn_per_cntdwn_slow_weight[i] + 1);
                if (g_fhss_chn_per[i] < TPSLL_FHSS_CHN_PER_GOOD_TH) {
                    g_fhss_chn_per[i] = TPSLL_FHSS_CHN_PER_GOOD_TH;
                }
            } else if (diff < 0) {
                g_fhss_chn_per[i] += ((uint16_t)g_fhss_chn_per[i] * g_fhss_chn_per_cntdwn_slow_weight[i]) / 
                    (TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_MAX + TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_DEFAULT  - 1 - g_fhss_chn_per_cntdwn_slow_weight[i]);

                if (g_fhss_chn_per[i] > TPSLL_FHSS_CHN_PER_GOOD_TH) {
                    g_fhss_chn_per[i] = TPSLL_FHSS_CHN_PER_GOOD_TH;
                }
            }
            #else
            g_fhss_chn_per[i] = (g_fhss_chn_per[i] * g_fhss_chn_per_cntdwn_slow_weight[i]) / (g_fhss_chn_per_cntdwn_slow_weight[i] + 1);
            #endif
            #else
            g_fhss_chn_per[i] = (g_fhss_chn_per[i] * 15) >> 4;
            #endif
            #if 1//!TPSLL_KEEP_ALIVE_PER_EN
            if (g_fhss_chn_per[i] < TPSLL_FHSS_CHN_PER_GOOD_TH - 1)
            {
                g_fhss_chn_per[i] = TPSLL_FHSS_CHN_PER_GOOD_TH - 1;
            }
            #endif

            #if TPSLL_WIFI_RSSI_SCORE_EN
            if (wifi_chn_inf.chn_score_valid) {
                // if ((rf_scan_ctx.state & RF_SCAN_STATE_IDLE) && rf_scan_ctx.select_chn_ready) {
                g_fhss_chn_per[i] = ((uint16_t)g_fhss_chn_per[i] * 6 / 8 + TPSLL_FHSS_CHN_PER_GOOD_TH) * (wifi_chn_inf.valid_chn_score[i] + 10) / ((TPSLL_WIFI_CHANNEL_COUNT + 10) * TPSLL_WIFI_RSSI_SCORE_MUILT);
                // }
            }
            #endif
        }
    }

    #if TPSLL_RF_SCAN_EN
    // if (ll_ctx.is_keep_alive && (rf_scan_ctx.state & RF_SCAN_STATE_IDLE) && rf_scan_ctx.select_chn_ready) {
    if ((rf_scan_ctx.state & RF_SCAN_STATE_IDLE) && rf_scan_ctx.select_chn_ready) {
        #if TPSLL_WIFI_RSSI_SCORE_EN
        // if (wifi_chn_inf.chn_score_valid) {
        //     for (int v = 0; v < TPSLL_FHSS_CHN_MAX_NUM; v++) {
        //         g_fhss_chn_per[v] = ((uint16_t)g_fhss_chn_per[v] * 6 / 8 + TPSLL_FHSS_CHN_PER_GOOD_TH) * (wifi_chn_inf.valid_chn_score[v] + 10) / ((TPSLL_WIFI_CHANNEL_COUNT + 10) * TPSLL_WIFI_RSSI_SCORE_MUILT);
        //     }
        // }
        #else /* end of TPSLL_WIFI_RSSI_SCORE_EN */
        for(int i = 0; i < TPSLL_RF_SCAN_SELECT_CHN_NUM; i++)
        {
            for (int v = 0; v < TPSLL_FHSS_CHN_MAX_NUM; v++)
            {
                if (g_fhss_valid_chn_tab[v] == rf_scan_ctx.select_chn_table[i]) {
                    uint8_t n = TPSLL_RF_SCAN_SELECT_CHN_NUM - i;
                    uint8_t dec_v = n * TPSLL_RF_SCAN_BEST_CHN_PER_DECREASE_VALUE;

                    if (g_fhss_chn_per[v] > dec_v) {
                        g_fhss_chn_per[v] -= dec_v;
                    } else {
                        g_fhss_chn_per[v] = 0;
                    }

                    dec_v = n * TPSLL_RF_SCAN_BEST_CHN_PER_WEIGHT_DECREASE_VALUE;
                    if (g_fhss_chn_per_cntdwn_slow_weight[i] > dec_v) {
                        g_fhss_chn_per_cntdwn_slow_weight[i] -= dec_v;
                    } else {
                        g_fhss_chn_per_cntdwn_slow_weight[i] = 1;
                    }
                    break;
                }
            }
        }

        // tlkapi_printf(APP_LOG_EN, "<%d-%d-%d>\n", rf_scan_ctx.select_chn_table[0], rf_scan_ctx.select_chn_table[1], rf_scan_ctx.select_chn_table[2]);

        #endif /* end of TPSLL_WIFI_RSSI_SCORE_EN #else */

        // print_idx += sprintf(&print_buf[print_idx], "per:");
        // for (int i = 0; i < TPSLL_FHSS_CHN_MAX_NUM; i++) {
        //     // print_idx += sprintf(&print_buf[print_idx], "<%02d>%03d", best_channels[i], rf_scan_ctx.chn_rssi_max[best_channels[i]]);
        //     print_idx += sprintf(&print_buf[print_idx], "[%d]%d", g_fhss_valid_chn_tab[i], g_fhss_chn_per[i]);
        //     // print_idx += sprintf(&print_buf[print_idx], "[%d]%d", i + 1, (wifi_chn_inf.rssi_sum[i] - 300));
        // }
        // print_idx += sprintf(&print_buf[print_idx], "\n");
        // printf("%s", print_buf);

        rf_scan_ctx.select_chn_ready = 0;
    }
    #endif /* end of TPSLL_RF_SCAN_EN */

    tpsll_channel_statistic(0xff, 0);
}

_attribute_ram_code_sec_ uint32_t tpsll_channel_select(uint8_t flag, uint8_t *map)
{
    uint8_t m = 0xff;
    uint32_t k = TPSLL_FHSS_CHN_MAX_NUM;
    // uint8_t shuffled_24[24] = {
    //     5, 18, 2, 14, 9, 0, 22, 7,
    //     13, 3, 19, 11, 6, 23, 1, 16,
    //     8, 21, 12, 4, 17, 10, 15, 20
    // };

    for (int j = 0; j < TPSLL_FHSS_CHN_MAX_NUM; j++)
    {
        // int id = shuffled_24[j];
        int id = j;

        if (((map[id] & flag) == flag) && (g_fhss_chn_per[id] < m) && (id < TPSLL_FHSS_CHN_MAX_NUM) )
        {
            m = g_fhss_chn_per[id];
            k = id;
        }
    }

    return k;
}

_attribute_ram_code_sec_ void tpsll_channel_update_map(void)
{
    uint8_t map[TPSLL_FHSS_CHN_MAX_NUM] = {0};
    int32_t ncur = 0;

    for (int i=0; i < TPSLL_FHSS_CHN_MAX_NUM; i++) {
        map[i] = TPSLL_FHSS_CHN_FLG_VALID;
        if (g_hop_map_cur & BIT(i) && ncur < TPSLL_HOP_CHN_NUM) {
            map[i] |= TPSLL_FHSS_CHN_FLG_CURRENT;
            ncur++;
        }
    }

    g_hop_map_next = 0;

    //////////// find good channel in current channel map, at least TPSLL_HOP_CHN_TBL_KEEP_GOOD_NUM ///////////////
    uint32_t id;
    int i;
    #if TPSLL_FHSS_CHN_SPACE_CHECK_EN
    int h = 0;
    uint8_t id_tab[TPSLL_HOP_CHN_NUM];
    #endif

    #if TPSLL_FHSS_REPORT_PER_EN
    uint8_t r_cnt = 0;
    #endif

    for (i = 0; i < ncur; i++) 
    {
        id = tpsll_channel_select(TPSLL_FHSS_CHN_FLG_VALID | TPSLL_FHSS_CHN_FLG_CURRENT, map);

        // if(id == TPSLL_FHSS_CHN_MAX_NUM && i< (TPSLL_HOP_CHN_TAB_KEEP_GOOD_NUM + 1/*new one*/)){
        if(id == TPSLL_FHSS_CHN_MAX_NUM && i< TPSLL_HOP_CHN_TAB_KEEP_GOOD_NUM){
            id = tpsll_channel_select(TPSLL_FHSS_CHN_FLG_VALID, map);
        }
        // if ((id < TPSLL_FHSS_CHN_MAX_NUM) && (g_fhss_chn_per[id] < TPSLL_FHSS_CHN_PER_GOOD_TH || i < (TPSLL_HOP_CHN_TAB_KEEP_GOOD_NUM + 1/*new one*/))) {
        if ((id < TPSLL_FHSS_CHN_MAX_NUM) && (g_fhss_chn_per[id] < TPSLL_FHSS_CHN_PER_GOOD_TH || i < TPSLL_HOP_CHN_TAB_KEEP_GOOD_NUM)) {
            map[id] &= ~TPSLL_FHSS_CHN_FLG_VALID;
            g_hop_map_next |= BIT(id);

            #if TPSLL_FHSS_CHN_SPACE_CHECK_EN
            id_tab[h] = id;
            h++;
            #endif

            #if TPSLL_FHSS_REPORT_PER_EN

            #if TPSLL_N8K_FHSS_EN
            if(ll_ctx.report_rate & REPORT_RATE_8K)
            {
            #endif
                r_cnt++;
                #if TPSLL_RF_SCAN_EN
                if (!ll_ctx.is_keep_alive)
                #endif
                if((r_cnt >= (TPSLL_HOP_CHN_NUM - TPSLL_REPORT_PER_KICK_OUT_CNT))  && (ll_ctx.report_per >= TPSLL_REPORT_PER_TH))
                {
                    break;
                }
            #if TPSLL_N8K_FHSS_EN
            }
            #endif

            #endif
        }
        else
        {
            break;
        }
    }

    #if TPSLL_FHSS_CHN_SPACE_CHECK_EN
    for (;h < TPSLL_HOP_CHN_NUM;)
    {
        id = tpsll_channel_select(TPSLL_FHSS_CHN_FLG_VALID, map);
        #if TPSLL_FHSS_CHN_SPACE_CHECK_EN
        id_tab[h] = id;
        map[id] &= ~TPSLL_FHSS_CHN_FLG_VALID;
        #if TPSLL_FHSS_FORCE_SELECT_SEPARATED_CHN_EN
        if (h < (TPSLL_HOP_CHN_NUM - 1))
        #else
        //if (h < TPSLL_HOP_CHN_NUM)
        if(h < 1)
        #endif
        {
            g_hop_map_next |= BIT(id);
            h++;
        } else {
            uint8_t reject = 0;
            for (i = 0; i < h; i++) {
                if (abs(g_fhss_valid_chn_tab[id_tab[i]] - g_fhss_valid_chn_tab[id_tab[h]]) < 2) {
                    reject = 1;
                    break;
                }
            }
            if (!reject) {
                g_hop_map_next |= BIT(id);
                h++;
            }
        }
        #endif
    }
    #else
    if (i < TPSLL_HOP_CHN_NUM)
    {
        id = tpsll_channel_select(TPSLL_FHSS_CHN_FLG_VALID, map);
        g_hop_map_next |= BIT(id);
    }
    #endif

    #if TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_ADJ_EN
    for (i=0; i < TPSLL_FHSS_CHN_MAX_NUM; i++) {
        if (g_hop_map_cur & BIT(i)) {
            if (g_hop_map_next & BIT(i)) { // good channel
                if (g_fhss_chn_per_cntdwn_slow_weight[i] > TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_DEFAULT)
                    g_fhss_chn_per_cntdwn_slow_weight[i] -= TPSLL_FHSS_GOOD_CHN_PER_WEIGHT_STEP;
            } else {
                g_fhss_chn_per_cntdwn_slow_weight[i] += TPSLL_FHSS_BAD_CHN_PER_WEIGHT_STEP;
                // tlkapi_printf(APP_LOG_EN, "%d@%d\n", g_fhss_valid_chn_tab[i], g_fhss_chn_per[i]);
            }
        }
    }
    #endif

    for (i = 0; i < TPSLL_FHSS_CHN_MAX_NUM; i++)
    {
        // if (g_hop_map_cur & BIT(i)) {
        //     if (g_fhss_chn_per[i] > 50) {
        //         tlkapi_printf(1, "<i%d-c%d-p%d>\n", i, g_fhss_valid_chn_tab[i], g_fhss_chn_per[i]);
        //     }
        // }

        if (!(g_hop_map_next & BIT(i)) && (g_hop_map_cur & BIT(i)))
        {
            int per = 80 + g_fhss_chn_per[i];
            g_fhss_chn_per[i] = per > 100 ? 100 : per;
        }
    }

    if (g_hop_map_cur != g_hop_map_next)
    {
        // tlkapi_send_string_data(1, "fhssPER", &g_fhss_chn_per, sizeof(g_fhss_chn_per));
        // tlkapi_printf(1, "mcn:0x%08X-0x%08X\n", g_hop_map_cur, g_hop_map_next);
    } else {
        // tlkapi_send_string_data(1, "fhssPer", &g_fhss_chn_per, sizeof(g_fhss_chn_per));
    }
}

_attribute_ram_code_sec_ void tpsll_fhss_recon_update_chn_per(void)
{
    g_hop_map_cur = 0;

    for (int i = 0; i < TPSLL_HOP_CHN_NUM; i++) {
        for (int j = 0; j < TPSLL_FHSS_CHN_MAX_NUM; j++) {
            if (chn_table[i] == g_fhss_valid_chn_tab[j]) {
                g_hop_map_cur |= BIT(j);
                g_fhss_chn_per_cntdwn_slow_weight[j] -= TPSLL_FHSS_GOOD_CHN_PER_WEIGHT_STEP;
                break;
            }
        }
    }
    g_hop_map_next = g_hop_map_cur;

    int j = 0;

    for (int i = 0; i < TPSLL_FHSS_CHN_MAX_NUM; i++) {
        j = 0;

        for (; j < TPSLL_HOP_CHN_NUM; j++) {
            if (g_fhss_valid_chn_tab[i] == chn_table[j]) {
                break;
            }
        }

        if (j >= TPSLL_HOP_CHN_NUM) {
            // g_fhss_chn_per[i] = (g_fhss_chn_per[i] * 1 + TPSLL_FHSS_BAD_CHN_HISTORY_ADD_SCORE * 5) / 6;
            g_fhss_chn_per[i] += TPSLL_FHSS_BAD_CHN_HISTORY_ADD_SCORE;
            g_fhss_chn_per_cntdwn_slow_weight[i] += TPSLL_FHSS_BAD_CHN_PER_WEIGHT_STEP;

            if (g_fhss_chn_per[i] > 100) {
                g_fhss_chn_per[i] = 100;
            }
        }
    }
}

_attribute_ram_code_sec_ void tpsll_fhss_fix_old_chn_index(void)
{
    #define TPSLL_IDX_P_VALID           0x80
    uint8_t hop_chn_idx_tab_next[TPSLL_HOP_CHN_NUM];
    // uint8_t tmp_idx_tab[TPSLL_HOP_CHN_NUM];
    // uint8_t used_mask[TPSLL_HOP_CHN_NUM] = {0}; 
    // uint8_t k = 0;

    for (int i = 0; i < TPSLL_HOP_CHN_NUM; i++) {
        hop_chn_idx_tab_next[i] = g_hop_chn_idx_tab_next[i];
        // printf("<%d>", g_hop_chn_idx_tab_next[i]);
        g_hop_chn_idx_tab_next[i] = TPSLL_IDX_P_VALID;
    }

    for (int i = 0; i < TPSLL_HOP_CHN_NUM; i++) {
        for (int j = 0; j < TPSLL_HOP_CHN_NUM; j++) {
            if ((hop_chn_idx_tab_next[j] != 0xff) && (chn_table[i] == g_fhss_valid_chn_tab[hop_chn_idx_tab_next[j]])) {
                g_hop_chn_idx_tab_next[i] = hop_chn_idx_tab_next[j];
                hop_chn_idx_tab_next[j] = 0xff;
                break;
            }
        }
    }
    for (int i = 0; i < TPSLL_HOP_CHN_NUM; i++) {
        if (g_hop_chn_idx_tab_next[i] & TPSLL_IDX_P_VALID) {
            for (int j = 0; j < TPSLL_HOP_CHN_NUM; j++) {
                if (hop_chn_idx_tab_next[j] != 0xff) {
                    g_hop_chn_idx_tab_next[i] = hop_chn_idx_tab_next[j];
                    hop_chn_idx_tab_next[j] = 0xff;
                    break;
                }
            }
        }
    }
}

#if 0
_attribute_ram_code_sec_ void tpsll_fill_tx_pkt_chn_table(void)
{
    rf_tx_packet.n_tab = 1;
    #if TPSLL_8K_CHN_EXPAND_EN
    if (ll_ctx.fno_last & 0x01) {
        rf_tx_packet.buf[1] = (g_hop_chn_idx_tab_next[0] & 0x1f) | ((g_hop_chn_idx_tab_next[1] & 0x7) << 5);
        rf_tx_packet.buf[2] = ((g_hop_chn_idx_tab_next[1] & 0x1f) >> 3) | ((g_hop_chn_idx_tab_next[2] & 0x1f) << 2);
    } else {
        rf_tx_packet.buf[1] = (g_hop_chn_idx_tab_next[3] & 0x1f) | ((g_hop_chn_idx_tab_next[4] & 0x7) << 5);
        rf_tx_packet.buf[2] = ((g_hop_chn_idx_tab_next[4] & 0x1f) >> 3) | ((g_hop_chn_idx_tab_next[5] & 0x1f) << 2);
    }
    #else
    rf_tx_packet.buf[1] = (g_hop_chn_idx_tab_next[0] & 0x1f) | ((g_hop_chn_idx_tab_next[1] & 0x7) << 5);
    rf_tx_packet.buf[2] = ((g_hop_chn_idx_tab_next[1] & 0x1f) >> 3) | ((g_hop_chn_idx_tab_next[2] & 0x1f) << 2);
    #endif
}
#endif

_attribute_ram_code_sec_ void tpsll_fill_tx_pkt_chn_table(void)
{
    rf_tx_packet.n_tab = 1;

    #if TPSLL_8K_CHN_EXPAND_EN
    if (ll_ctx.fno_last & 0x01) {
        rf_tx_packet.buf[1] = (g_hop_chn_idx_tab_next[0] & 0x1f) | ((g_hop_chn_idx_tab_next[1] & 0x7) << 5);
        rf_tx_packet.buf[2] = ((g_hop_chn_idx_tab_next[1] & 0x1f) >> 3) | ((g_hop_chn_idx_tab_next[2] & 0x1f) << 2);
    } else {
        rf_tx_packet.buf[1] = (g_hop_chn_idx_tab_next[3] & 0x1f) | ((g_hop_chn_idx_tab_next[4] & 0x7) << 5);
        rf_tx_packet.buf[2] = ((g_hop_chn_idx_tab_next[4] & 0x1f) >> 3) | ((g_hop_chn_idx_tab_next[5] & 0x1f) << 2);
    }
    #else
    
    const uint8_t ch0 = g_hop_chn_idx_tab_next[0] & 0x1F;
    const uint8_t ch1 = g_hop_chn_idx_tab_next[1] & 0x1F;
    const uint8_t ch2 = g_hop_chn_idx_tab_next[2] & 0x1F;


    const uint8_t val_low  = ch0 | ((ch1 & 0x07) << 5);
    const uint8_t val_high = (ch1 >> 3) | (ch2 << 2);


#if TPSLL_N8K_FHSS_EN
    const uint32_t r = ll_ctx.report_rate;

    if (r & REPORT_RATE_8K)
    {
        rf_tx_packet.buf[1] = val_low;
        rf_tx_packet.buf[2] = val_high;
    }
    else if (r & (REPORT_RATE_4K | REPORT_RATE_2K))
    {
        rf_tx_packet.buf[0] = val_low;
        rf_tx_packet.buf[1] = val_high;
    }
    else
    {
        rf_tx_packet.buf[2] = val_low;
        rf_tx_packet.buf[3] = val_high;
    }
#else
    rf_tx_packet.buf[1] = val_low;
    rf_tx_packet.buf[2] = val_high;
#endif
#endif
}

#if TPSLL_FHSS_REPORT_PER_EN
static inline void tpsll_update_report_per(void)
{
    if(ll_ctx.report_pkt_cnt > TPSLL_FHSS_EXPECTED_REPORT_CNT)
    {
        ll_ctx.report_pkt_cnt = 0;
    }

    uint8_t cur_per = (TPSLL_FHSS_EXPECTED_REPORT_CNT -ll_ctx.report_pkt_cnt) * 100 / TPSLL_FHSS_EXPECTED_REPORT_CNT;

    // uint16_t recv_cnt = TPSLL_FHSS_EXPECTED_REPORT_CNT;

    ll_ctx.report_per  = (ll_ctx.report_per * 3 + cur_per) >> 2;


    // tlkapi_printf(APP_LOG_EN, "%d  pkt:%d cur_per:%d per:%d\n", recv_cnt, ll_ctx.report_pkt_cnt,cur_per, ll_ctx.report_per);
    ll_ctx.report_pkt_cnt = 0;
}
#endif
#endif /* TPSLL_FHSS_EN */

void pp_rf_tx_fifo_pop (pl_fifo_t *f)
{
    f->rptr++;
}

static inline bool tpsll_all_fifo_is_empty(void)
{
    return pp_fifo_is_empty(&ll_fifo) && pp_fifo_is_empty(&spp_fifo);
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

static void tpsll_stack_set_pairing_info(const tpsll_stack_var_t *p_stack_var)
{
    tmemcpy(ll_inf.peer_mac, p_stack_var->peer_mac, MAC_ADDR_LEN);
    ll_inf.paired_dev_type = p_stack_var->paired_dev_type;

    if (ll_inf.paired_dev_type & DEVICE_TYPE_KB)
    {
        ll_ctx.ktag = p_stack_var->peer_tag;
    }
    else if (p_stack_var->peer_type & DEVICE_TYPE_MS)
    {
        ll_ctx.mtag = p_stack_var->peer_tag;
    }
    ll_inf.tag = fnv1a_16(ll_inf.mac, MAC_ADDR_LEN);
    tlkapi_printf(APP_LOG_EN, "tag:%x\n", ll_inf.tag);
}

static void tpsll_stack_get_pairing_info(tpsll_stack_var_t *p_stack_var)
{
    tmemcpy(p_stack_var->peer_mac, ll_inf.peer_mac, MAC_ADDR_LEN);
    p_stack_var->connecting_dev = ll_inf.connecting_dev;
    p_stack_var->side_id = ll_inf.side_id;
    ll_inf.paired_dev_type = ll_inf.connecting_dev;
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

        default:
            break;
    }

    return TLK_SUCCESS;
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



_attribute_ram_code_sec_ uint8_t tpsll_enable_pairing(bool enable)
{
    if(enable)
    {
        rf_access_code_comm(ll_ctx.pair_acc);
        // write_reg8(0x17000c,0x55);//related to acc
        #if TPSLL_RR_CHG_WITH_RECON_EN
        ll_ctx.rr_chg_with_recon = RR_RECON_STEP_NONE;
        #endif
        tpsll_set_state(STATE_PAIRING);
        ll_ctx.pair_st = STATE_PAIR_INIT;
        ll_ctx.pair_cnt = 0;
        #if AFH_ENABLE
        app_afh_init();
        #endif
        #if TPSLL_RECON_RSSI_SCAN_EN && TPSLL_NEW_RECON_EN
        if (ll_ctx.recon_rssi_scan_step == 0xff) {
            #if TPSLL_NEW_PAIR_EN
            ll_ctx.recon_rssi_scan_step = RECON_RSSI_SCAN_STEP_PREPARE;
            #else
            ll_ctx.recon_rssi_scan_step = RECON_RSSI_SCAN_STEP_NONE;
            #endif
        }
        #endif
    }
    else
    {
        // rf_access_code_comm(~ll_ctx.pair_acc);
        // write_reg8(0x17000c,0xAA);//related to acc
        ll_ctx.pair_st = STATE_PAIR_NONE;
    }

    #if TPSLL_RF_TX_POWER_TRIM_EN
    rf_set_vant_power_trim_level(RF_VANT_NORMAL_POWER);
    #endif

    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ uint8_t tpsll_enable_reconn(bool enable)
{
    if (enable)
    {
        #if TPSLL_ACC_SWITCH_WITH_RR_EN
        ll_ctx.report_rate = APP_DEFAULT_REPORT_RATE;
        #endif

        #if TPSLL_RECON_RSSI_SCAN_EN && TPSLL_NEW_RECON_EN
        if (ll_ctx.recon_rssi_scan_step == 0xff) {
            ll_ctx.recon_rssi_scan_step = RECON_RSSI_SCAN_STEP_PREPARE;
        }
        #endif

        #if 1
        // uint32_t acc;

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

        // #if TPSLL_RR_CHG_WITH_RECON_EN && TPSLL_RECON_RSSI_SCAN_EN
        // if (ll_ctx.rr_chg_with_recon)
        // #endif
        // {
        //     tpsll_gen_reconn_chn(ll_ctx.reconn_chn, reconn_chn_table);
        // }

        rf_access_code_comm(acc);
        // write_reg8(0x17000c, 0x55);
        #endif

        #else
        rf_access_code_comm(ll_ctx.pair_acc);
        write_reg8(0x17000c,0x55);//related to acc
        rf_set_ble_chn(PAIR_CHN);
        #endif

#if 1
        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (!ll_ctx.rr_chg_with_recon)
        #endif
        {
        // uint8_t tmo[6] = {0};
        // #if TPSLL_NEW_RECON_EN
        // tmemcpy(tmo, &ll_ctx.recon_acc, 4);
        // #else
        // tmemcpy(tmo, &acc, 4);
        // #endif
        // tmo[4] = ll_ctx.reconn_chn;
        // tlkapi_send_string_data(APP_LOG_EN, "reconn ac chn", tmo, 6);

        // uint8_t tmo[11] = {0};
        // tmemcpy(tmo, &acc, 4);
        // tmemcpy(&tmo[4], ll_inf.peer_mac, 6);
        // tmo[10] = ll_ctx.reconn_chn;
        // tlkapi_send_string_data(APP_LOG_EN, "reconn ac chn", tmo, 11);
        // tlkapi_send_string_data(APP_LOG_EN, "reconn chn", reconn_chn_table, 8);
        }
#endif

        tpsll_set_state(STATE_RECONNECT);
#if AFH_ENABLE
        // p24g_afh_init();
#endif
    }
    else
    {
        // rf_access_code_comm(~ll_ctx.pair_acc);
        // write_reg8(0x17000c, 0xAA); // related to acc
    }

    #if TPSLL_RF_TX_POWER_TRIM_EN
    rf_set_vant_power_trim_level(RF_VANT_NORMAL_POWER);
    #endif

    ll_ctx.recon_enter_pm_optm_cnt = 0;
    ll_ctx.recon_start_ref_tick = clock_time() | 1;

    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ static void app_d24g_enter_conn(void)
{
    tpsll_set_state(STATE_CONNECTED);
    calc_hop_channels(ll_ctx.fno_last + 1, chn_mask);
    tpsll_enable_pairing(false);
    
    // uint32_t acc;
    p24g_gen_ac_and_channel(ll_inf.mac, &ll_ctx.conn_acc, 0);
    // rf_access_code_comm(ll_ctx.pair_acc);
    rf_access_code_comm(ll_ctx.conn_acc);

    #if TPSLL_RR_CHG_WITH_RECON_EN
    if (ll_ctx.rr_chg_with_recon) {
        unsigned char *p = pp_fifo_get_ptr(&ll_fifo);

        if (p)
        {
            if(p[2] == TPSLL_LL_CMD_RR_CHG_RCNN_RSP)
            {
                pp_fifo_pop(&ll_fifo);
            }
        }
    } else
    #endif
    {
    pp_fifo_reset(&spp_fifo);
    pp_fifo_reset(&ll_fifo);

    tlkapi_send_string_data(APP_LOG_EN, "conn ac", &ll_ctx.conn_acc, 4);
    //tlkapi_send_string_data(APP_LOG_EN, "dongle ac", ll_inf.mac, 6);
    }


    // ll_ctx.report_mode = REPORT_MODE_FAST;
    ll_ctx.report_mode = REPORT_MODE_NORMAL;
    ll_ctx.switch_special_cnt = 0;
    ll_ctx.is_last_report = false;
    ll_ctx.fno_conn = ll_ctx.fno_last | 1;

    ll_ctx.spp_flag = false;
    ll_ctx.ll_flag = false;

    ll_ctx.last_pkt_acked = true;
    ll_ctx.rx_over = true;
    ll_ctx.tx_over = true;

    ll_ctx.conn_ignore_data_cnt = 10;

    if (ll_ctx.report_rate & REPORT_RATE_8K)
    {
        ll_ctx.tx_sn = 1;
        ll_ctx.rx_nesn = 0;
        ll_ctx.sn_8k = 0;
        ll_ctx.nesn_8k = 0;
        #if TPSLL_SPP_NOT_SWITCH_EN
        SET_SN(rf_tx_packet.snnesn, ll_ctx.tx_sn);
        SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);
        #else
        SET_SN(rf_tx_packet.snnesn, ll_ctx.sn_8k);
        SET_NESN(rf_tx_packet.snnesn, ll_ctx.nesn_8k);
        #endif
#if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        ll_ctx.last_intv_pkt_st = TPSLL_PKT_ALL_RCVD;
        ll_ctx.intv_pkt_st = TPSLL_PKT_ALL_RCVD;
        ll_ctx.last_sn = 1;
        ll_ctx.last_nesn = 1;
        ll_ctx.acc.x = 0;
        ll_ctx.acc.y = 0;
        ll_ctx.last_acc.x = 0;
        ll_ctx.last_acc.y = 0;
#endif

#if TPSLL_PACKET_LOSS_COMPENSATE_EN
    if(g_kalman_enable == TPSLL_ALG_ENABLE) 
    {
        kalman_vel_init(&g_kf, 0.0f, 0.0f, TPSLL_KALMAN_Q, TPSLL_KALMAN_R);
        ll_ctx.last_btn = 0;
    }
#endif
    }
    else
    {
        ll_ctx.tx_sn = 1;
        ll_ctx.rx_nesn = 0;
        SET_SN(rf_tx_packet.snnesn, ll_ctx.tx_sn);
        SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);
    }

    #if TPSLL_CONN_QUICK_LOST_TIME_US
    ll_ctx.quick_discon_active_tick = clock_time();
    #endif

    #if TPSLL_RR_CHG_WITH_RECON_EN
    if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP5) {
        ll_ctx.rr_chg_with_recon = RR_RECON_STEP6;
        ll_ctx.rr_chg_recon_time_tick = 0;
    } else if (!ll_ctx.rr_chg_with_recon)
    #endif
    {
        stack_evt_post_from_isr(TPSLL_EVT_DEV_CONNECTED, (void *)ll_inf.peer_mac, MAC_ADDR_LEN);
    }

    #if TPSLL_WHEEL_TEST_EN
    g_test_tick = clock_time() | 1;
    #endif
    #if TPSLL_FHSS_REPORT_PER_EN
    ll_ctx.report_pkt_cnt = 0;
    ll_ctx.report_per = 0;
    #endif
    ll_ctx.connected_tick = stimer_get_tick();
    ll_ctx.allow_spp = false;

    #if TPSLL_KEEP_ALIVE_NOTIFY_EN
    ll_ctx.is_keep_alive = 0;
    #endif
    #if TPSLL_RF_SCAN_EN
    rf_scan_ctx.state = RF_SCAN_STATE_IDLE;
    rf_scan_ctx.temp_switch_chn = 0xff;
    #endif

    #if TPSLL_KB_ACTIVE_PM_OPTM_EN
    if (ll_inf.paired_dev_type & DEVICE_TYPE_KB) {
        ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
        ll_ctx.kb_alive_intervals_cnt = 0;
    }
    #endif

    current_channel = 0;

    #if TPSLL_RF_TX_POWER_TRIM_EN
    rf_set_vant_power_trim_level(RF_VANT_HIGH_POWER);
    #endif
}

#if TPSLL_N8K_CHN_EXPAND_EN
_attribute_ram_code_sec_ static void tpsll_n8k_chn_expand(void)
{
    chn_table[3] = chn_table[1];

    tpsll_generate_deterministic_ble_channels(chn_table, chn_table + 4);

    chn_table[7] = chn_table[4];
    print_chan_table = true;
}
#endif

_attribute_ram_code_sec_ static void tpsll_connect_lost(void)
{
    ll_ctx.lost_cnt = 0;
    #if TPSLL_RECON_RSSI_SCAN_EN &&TPSLL_NEW_RECON_EN
    ll_ctx.recon_rssi_scan_step = RECON_RSSI_SCAN_STEP_PREPARE;
    #endif
    // tpsll_set_state(STATE_PAIRING);
    ll_ctx.step_3t1r = STEP_3T1R_NONE;
    app_d24g_set_report_rate(REPORT_RATE_8K);
    tpsll_enable_reconn(true);
    spp_send_reset();
    auto_draw_flag = 0;
    g_local_data_en = 0;
    ll_ctx.spp_mode = false;
    tmemset((void*)&lost_ms_data, 0, sizeof(lost_ms_data));
    #if TPSLL_FHSS_EN
    tmemset(g_fhss_pkt_expect, 0, 40);
    tmemset(g_fhss_pkt_err, 0, 40);
    tmemset(g_fhss_chn_per, TPSLL_FHSS_CHN_PER_GOOD_TH, TPSLL_FHSS_CHN_MAX_NUM);
    #if TPSLL_FHSS_CHN_HISTORY_PER_WEIGHT_ADJ_EN
    tmemset(g_fhss_chn_per_cntdwn_slow_weight, 0, TPSLL_FHSS_CHN_MAX_NUM);
    #endif
    #endif
    #if TPSLL_KEEP_ALIVE_NOTIFY_EN
    ll_ctx.is_keep_alive = 0;
    #endif
    #if TPSLL_RF_SCAN_EN
    rf_scan_ctx.state = RF_SCAN_STATE_IDLE;
    #endif
}

#if TPSLL_AVOID_WIFI_CHN_USE_RX3_EN

static void rf_rx3_rssi_scan_param_set(void)
{
    rf_access_code_comm(0x1a2b3c4d);
    write_reg8(REG_RXCHN_EN, 0);
    write_reg8(0x17051a,40);//sync thr
}

static void rf_rx3_rssi_scan_param_restore(void)
{
    rf_access_code_comm(ll_ctx.conn_acc);

    write_reg8(REG_RXCHN_EN, 1);
    write_reg8(0x17051a,36);//sync thr
    // rf_set_irq_mask(FLD_RF_IRQ_TX|FLD_RF_IRQ_RX|FLD_RF_IRQ_RX_TIMEOUT|FLD_RF_IRQ_FIRST_TIMEOUT);
}

// _attribute_ram_code_sec_ static void proc_rx3_rssi_scan(void)
// {
//     uint8_t rssi = REG_RSSI_MAX_BB;
//     rssi_result[current_channel] += rssi;
//     scan_counts[current_channel]++;

//     if (g_chn_rssi_max[current_channel] < rssi)
//         g_chn_rssi_max[current_channel] = rssi; //max rssi

//     CLR_REG_RSSI_MAX_BB();


//     if (current_channel >= RSSI_SCAN_CHANNEL_COUNT)
//     {
//         // delay_ms(1);
//         current_channel = 0;
//         while (!is_channel_valid(current_channel))
//         {
//             current_channel++;
//         }
//         current_round++;

//         if (current_round >= RSSI_SCAN_ROUNDS)
//         {
//             scan_state = STATE_SCAN_DONE;
//             current_round = 0;
//         }
//     }

// }

_attribute_ram_code_sec_ static void tpsll_rx3_rssi_scan(void)
{
    uint8_t rssi = REG_RSSI_MAX_BB;

    #if TPSLL_NEW_RECON_EN
    if (rf_scan_ctx.chn_rssi_max[rf_scan_ctx.cur_chn] < rssi)
        rf_scan_ctx.chn_rssi_max[rf_scan_ctx.cur_chn] = rssi; //max rssi
    #endif

    CLR_REG_RSSI_MAX_BB();

    if ((rf_scan_ctx.cur_round >= TPSLL_RF_SCAN_ROUNDS) && (rf_scan_ctx.temp_switch_chn == 0xff))
    {
        rf_scan_ctx.state = RF_SCAN_STATE_DONE;
        rf_scan_ctx.cur_round = 0;
    }
}

// _attribute_ram_code_sec_ static void set_rx3_rssi_scan_chn(void)
// {
//     // if ((current_channel == 1) || (current_channel == 5) || (current_channel == 12) ||
//     //     (current_channel == 17) || (current_channel == 28) || (current_channel == 34))
//     // printf("[%d:%d]\n", current_channel, rssi);

//     do
//     {
//         current_channel++;
//     } while (current_channel < RSSI_SCAN_CHANNEL_COUNT && !is_channel_valid(current_channel));


//     // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
//     rf_set_ble_chn(current_channel);
// }

_attribute_ram_code_sec_ static void tpsll_set_rx3_rssi_scan_chn(void)
{
    if (rf_scan_ctx.temp_switch_chn == 0xff) { //normal
        rf_scan_ctx.cur_chn++;

        if (rf_scan_ctx.cur_chn >= TPSLL_RF_SCAN_CHN_TOTAL)
        {
            rf_scan_ctx.cur_chn = 0;
            rf_scan_ctx.cur_round++;
        }
    } else { // use last avoid channel
        rf_scan_ctx.cur_chn = rf_scan_ctx.temp_switch_chn;
        rf_scan_ctx.temp_switch_chn = 0xff;
    }

    if (chn_list[STEP_3T1R_3T_D_3R] == rf_scan_ctx.cur_chn) {
        // avoid channel
        rf_scan_ctx.temp_switch_chn = rf_scan_ctx.cur_chn;

        if (rf_scan_ctx.cur_chn < (TPSLL_RF_SCAN_CHN_TOTAL - 1))
        {
            rf_scan_ctx.cur_chn++;
        } else {
            rf_scan_ctx.cur_chn = 0;
            rf_scan_ctx.cur_round++;
        }
    }

    CLR_REG_RSSI_MAX_BB();

    rf_set_ble_chn(rf_scan_ctx.cur_chn);
}

static void tpsll_avoid_wifi_chn_use_rx3(void)
{
    rf_rx3_rssi_scan_param_set();
    tpsll_set_rx3_rssi_scan_chn();
    // rf_set_ble_chn(ll_ctx.rx3_rssi_scan_chn);
    // ll_ctx.rx3_rssi_scan_chn++;
    
}


#endif

// #if 1
// unsigned char uart_tx_buff1[2][128*2] __attribute__((aligned(4)));
// void app_rf_buf_print(unsigned char *buf, unsigned int data_len)
// {

//     static volatile int uart_active_buf1 = 0;
//     plic_interrupt_disable(IRQ_SYSTIMER);

//     core_interrupt_disable();
//     int buf_idx = uart_active_buf1 ^ 1;
//     unsigned int len = snprintf((char *)uart_tx_buff1[buf_idx],
//                                 sizeof(uart_tx_buff1[0]),
//                                 "mouse_rx(%d):\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n"
//                                 "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", data_len,
//                                 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
//                                 buf[12], buf[13], buf[14], buf[15]);

//     if (len >= sizeof(uart_tx_buff1[0]))
//     {
//         len = sizeof(uart_tx_buff1[0]) - 1;
//         uart_tx_buff1[buf_idx][len] = '\n'; // Ensure the presence of line breaks
//     }

//     uart_send_dma(UART0, uart_tx_buff1[buf_idx], len);

//     uart_active_buf1 = buf_idx;

//     while (1)
//     {
//         delay_ms(100);
//         DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, LED1);
//     }
// }
// #endif


/** 
 *  This function is used to send LL data with a length of 16 bytes or less.
 */

_attribute_ram_code_sec_ uint8_t  p24g_fill_ll_pkt(void)
{//for none 8k report rate
    uint8_t len = RF_PKT_NULL_ACK_LEN;
    unsigned char *p = pp_fifo_get_ptr(&ll_fifo);
    if (p)
    {
        // len = p[0];
        //uint8_t cmd = p[1];
        #if 0
        if (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250)) {
            tmemcpy(&rf_tx_packet.buf[2], &p[2], len);
            len += RF_PKT_NULL_125_ACK_LEN;
        } else {
            tmemcpy(&rf_tx_packet.buf[1], &p[2], len);
            len += 1;
        }
            #endif
        // tmemcpy(&rf_tx_packet.buf[2], &p[2], len);
        rf_tx_packet.pkt_type = P24G_PKT_TYPE_LL;
        tmemcpy(&rf_tx_packet.buf[LL_PKT_OFFSET], &p[2], p[0]);
        len = p[0] + LL_PKT_OFFSET;
        ll_ctx.ll_flag = true;
        #if TPSLL_RR_CHG_WITH_RECON_EN && !TPSLL_RR_CHG_WITH_RECON_NO_RSP_EN
        if (p[2] == TPSLL_LL_CMD_RR_CHG_RCNN_RSP) {
            if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP1) {
                ll_ctx.rr_chg_with_recon = RR_RECON_STEP2;
            }
        }
        #endif
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
        return len;
    }
    // return len + 2;
    return len;
}

_attribute_ram_code_sec_ uint8_t p24g_fill_null_pkt(void)
{
    
    rf_tx_packet.pkt_type = P24G_PKT_TYPE_SYNC;

#if TPSLL_N8K_FHSS_EN
    uint8_t len = RF_PKT_NULL_ACK_LEN + 2;
    if (ll_ctx.report_rate & (REPORT_RATE_4K | REPORT_RATE_2K))
    {
        len = RF_PKT_NULL_ACK_LEN;
    }
    
    // if (g_hop_map_cur != g_hop_map_next)
    // {

    //     len += 2;
    // }

    
    return len;
#else
    return RF_PKT_NULL_ACK_LEN;
#endif
}

_attribute_ram_code_sec_ uint8_t  p24g_fill_null_pkt_125(void)
{
    // pp_rf_set_tx_len(2);
    rf_tx_packet.pkt_type = P24G_PKT_TYPE_SYNC;

    // rf_tx_packet.buf[2] = 0;
    return RF_PKT_NULL_125_ACK_LEN;
}

_attribute_ram_code_sec_ static void app_d24g_change_step(void)
{
    #if TPSLL_FHSS_EN
    if (ll_ctx.stk_state == STATE_CONNECTED)
    {
        tpsll_channel_statistic(chn_list[ll_ctx.step_3t1r], false);
    }
    #endif

    if (ll_ctx.step_3t1r == STEP_3T1R_1T_D_1R)
    {
        // dongle 1R err
        ll_ctx.step_3t1r = STEP_3T1R_2T_D_2R;
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_2T_D_2R)
    {
        // dongle 2R err
        ll_ctx.step_3t1r = STEP_3T1R_3T_D_3R;
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
    {
        // dongle 3R err
        ll_ctx.step_3t1r = STEP_3T1R_1R_D_1T;
    }
}

_attribute_aligned_(4) static t_mouse_data mouse_rx[4] = {
        {1, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0}
};
static volatile unsigned char ms_idx = 3; /* index of data to write to USB first */
static volatile unsigned char ms_rp_idx = 0; /* index of data to write to USB first */

volatile unsigned char g_ms_need_report[4] = {0, 0, 0, 0};


#if SQUARE_DEBUG_TEST

#define SIDE_TARGET        200
#define RETRANS_WINDOW     (SIDE_TARGET >> 1)


typedef enum {
    EDGE_RIGHT  = 0,    // right x+1
    EDGE_DOWN   = 1,    // down y+1
    EDGE_LEFT   = 2,    // left x-1
    EDGE_UP     = 3     // up y-1
} EdgeTypeDef;


typedef struct
{
    int32_t total[4];
    uint32_t count[4];

    EdgeTypeDef curr_edge;
    EdgeTypeDef last_edge;
    uint16_t retrans_cnt;
} SquareStat_T;

static SquareStat_T sq = {0};

void SquareStat_Init(void)
{
    for (int i = 0; i < 4; i++)
    {
        sq.total[i] = 0;
        sq.count[i] = 0;
    }
    sq.curr_edge = EDGE_RIGHT;
    sq.retrans_cnt = 0;
}


static void SwitchEdge(EdgeTypeDef new_edge)
{
    sq.last_edge = sq.curr_edge;

    sq.curr_edge = new_edge;
    sq.retrans_cnt = RETRANS_WINDOW;
}

static void CheckLastEdgeTotal(void)
{
    int target = (sq.last_edge == EDGE_RIGHT || sq.last_edge == EDGE_DOWN) ? SIDE_TARGET : -SIDE_TARGET;
    int actual = sq.total[sq.last_edge];

    if (actual == target)
    {
        // ZW_TG(GPIO_PH2);
    }
    else
    {
        static int sq_cnt = 0;
        // if(sq_cnt < 3)
        {

            tlkapi_printf(APP_LOG_EN, "%d: a:%d t:%d le:%d\n", sq_cnt, actual, target, sq.last_edge);
            sq_cnt++;
        }
        // ZW_TG(GPIO_PH0);
    }

    sq.total[sq.last_edge] = 0;
}

void SquareStat_Process(int16_t x, int16_t y)
{

    if (sq.curr_edge == EDGE_RIGHT && y >= 1)
    {
        SwitchEdge(EDGE_DOWN);
    }
    else if (sq.curr_edge == EDGE_DOWN && x <= -1)
    {
        SwitchEdge(EDGE_LEFT);
    }
    else if (sq.curr_edge == EDGE_LEFT && y <= -1)
    {
        SwitchEdge(EDGE_UP);
    }
    else if (sq.curr_edge == EDGE_UP && x >= 1)
    {
        SwitchEdge(EDGE_RIGHT);
    }

    if (sq.retrans_cnt > 0)
    {
        sq.retrans_cnt--;

 
        if (x >= 1)
        {
            sq.total[EDGE_RIGHT] += x;
            sq.count[EDGE_RIGHT]++;
        }
        if (y >= 1)
        {
            sq.total[EDGE_DOWN] += y;
            sq.count[EDGE_DOWN]++;
        }
        if (x <= -1)
        {
            sq.total[EDGE_LEFT] += x;
            sq.count[EDGE_LEFT]++;
        }
        if (y <= -1)
        {
            sq.total[EDGE_UP] += y;
            sq.count[EDGE_UP]++;
        }


        if (sq.retrans_cnt == 0)
        {
            CheckLastEdgeTotal();
        }
    }

    else
    {
        switch (sq.curr_edge)
        {
            case EDGE_RIGHT:
                if (x >= 1 && y == 0)
                {
                    sq.total[EDGE_RIGHT] += x;
                    sq.count[EDGE_RIGHT]++;
                }
                break;

            case EDGE_DOWN:
                if (y >= 1 && x == 0)
                {
                    sq.total[EDGE_DOWN] += y;
                    sq.count[EDGE_DOWN]++;
                }
                break;

            case EDGE_LEFT:
                if (x <= -1 && y == 0)
                {
                    sq.total[EDGE_LEFT] += x;
                    sq.count[EDGE_LEFT]++;
                }
                break;

            case EDGE_UP:
                if (y <= -1 && x == 0)
                {
                    sq.total[EDGE_UP] += y;
                    sq.count[EDGE_UP]++;
                }
                break;
        }
    }
}


#endif

_attribute_ram_code_sec_ unsigned char app_mouse_report_to_usb_internal(unsigned char *buf)
{
    #if TPSLL_FHSS_REPORT_PER_EN
    ll_ctx.report_pkt_cnt++;
    #endif

    #if 1

    short x = (short)(buf[1] | (buf[2] << 8));
    short y = (short)(buf[3] | (buf[4] << 8));
    uint8_t btn = buf[0];

    if(btn == 0xff)
    {
        return 4;
    }


    

    #if SQUARE_DEBUG_TEST
    static bool is_start = false;
    if(!is_start && x > 0)
    {
        is_start = true;
    }

    if(is_start)
    {
        SquareStat_Process(x,y);
    }
    #endif

    

    #if 0//workaround for now
    #define LOW_THRESHOLD -200
    #define HIGH_THRESHOLD 200
    if(ll_ctx.report_rate & REPORT_RATE_8K )
    {
        if (x > HIGH_THRESHOLD || x < LOW_THRESHOLD || y > HIGH_THRESHOLD || y < LOW_THRESHOLD)
        {
          //  tlkapi_send_string_data(APP_LOG_EN, "rp", tmp, 8);
          return;
        }
        if(btn > 0x1f)
        {
            return;
        }
        // return;
    }
    #endif
    
    #if TPSLL_PACKET_LOSS_COMPENSATE_EN
    if(g_kalman_enable == TPSLL_ALG_ENABLE)
    {
        ll_ctx.last_btn = buf[0];
    }
    #endif

    // if(ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
    // {

    // if(x > 1 || x < -1)
    // {
    //     ZW_TG(GPIO_PH4);
    // }
    // }



    // if (x == 0xff || x == -256 || y == 0xff || y == -256)
    // {
    //     return 4;
    // }
    #endif

    // static uint32_t icnc = 0;
    // if(icnc < 20)
    // {
    //      tlkapi_send_string_data(APP_LOG_EN, "rp", buf, 8);
    // }
    // icnc++;

    #if XY_CAP_COMPENSATE_EN
    #define XY_REPORT_CLIP_TH        (50)
    #define XY_ABNORMAL_REJECT_TH    (1000)

    static int x_remain = 0;
    static int y_remain = 0;

    if (ll_ctx.report_rate & REPORT_RATE_8K)
    {
        int abs_x = (x >= 0) ? x : -x;
        int abs_y = (y >= 0) ? y : -y;
        
        if(abs_x >= XY_ABNORMAL_REJECT_TH || abs_y >= XY_ABNORMAL_REJECT_TH ||
           x == 255 || x == -256 || y == 255 || y == -256)
        {
            return 3;
        }
    }
    
    if (ll_ctx.report_rate & REPORT_RATE_8K)
    {
        if (x_remain != 0 || x >= XY_REPORT_CLIP_TH || x <= -XY_REPORT_CLIP_TH)
        {
            x_remain += x;
            if(x_remain >= XY_REPORT_CLIP_TH)
            {
                x = XY_REPORT_CLIP_TH;
            }
            else if (x_remain <= -XY_REPORT_CLIP_TH)
            {
                x = -XY_REPORT_CLIP_TH;
            }
            else
            {
                x = (short)x_remain;
            }
            x_remain -= x;
        }

        if (y_remain != 0 || y >= XY_REPORT_CLIP_TH || y <= -XY_REPORT_CLIP_TH)
        {
            y_remain += y;
            if (y_remain >= XY_REPORT_CLIP_TH)
            {
                y = XY_REPORT_CLIP_TH;
            }
            else if (y_remain <= -XY_REPORT_CLIP_TH)
            {
                y = -XY_REPORT_CLIP_TH;
            }
            else
            {
                y = (short)y_remain;
            }
            y_remain -= y;
        }

        buf[1] = (uint8_t)(x & 0xff);
        buf[2] = (uint8_t)(((uint16_t)x >> 8) & 0xff);
        buf[3] = (uint8_t)(y & 0xff);
        buf[4] = (uint8_t)(((uint16_t)y >> 8) & 0xff);
    }
    #endif

    if(s_app_cb && s_app_cb->app_mouse_report)
    {
        return s_app_cb->app_mouse_report(buf);
    }else{

        return 1;
    }
}

void print_ms_buf(void)
{
    tlkapi_send_string_data(APP_LOG_EN, "m_rx", mouse_rx, sizeof(mouse_rx));
}

#if TPSLL_LOCAL_MS_DATA_TEST_EN
_attribute_ram_code_sec_ static unsigned char Draw_a_square(signed short *x,signed short *y)
{
    static unsigned int cnt = 0;
    static unsigned char flag = 0;
    signed short step = 1;

    cnt++;
    if (cnt >= 200)
    {
        cnt = 0;
        flag++;
        if (flag > 3)
        {
            flag = 0;
        }

    }
    if (flag == 0)
    {
        x[0] = step;
        y[0] = 0;
    }
    else if (flag == 1)
    {
        x[0] = 0;
        y[0] = step;
    }
    else if (flag == 2)
    {
        x[0] = -step;
        y[0] = 0;
    }
    else if (flag == 3)
    {
        x[0] = 0;
        y[0] = -step;
    }
    return 1;
}

_attribute_ram_code_sec_ unsigned char tpsll_mouse_report_data_test(unsigned char *buf)
{
    uint8_t ret = 10;

    if (ll_ctx.conn_ignore_data_cnt) {
        return;
    }

    Draw_a_square(((signed short*)(buf + 1)), ((signed short*)(buf + 3)));

    if (ll_ctx.report_rate & REPORT_RATE_8K) {
        if (app_mouse_8k_usb_ep_buf_has_data())
        {
            usb0hw_flush_tx_fifo(HID_MOUSE_IN_ENDPOINT_NUM);
            ll_ctx.z_wheel_data += last_ms_data.z_wheel;
            #if TPSLL_X_Y_FLUSH_HANDLE_EN
            *((signed short*)(buf + 1)) += last_ms_data.x;
            *((signed short*)(buf + 3)) += last_ms_data.y;
            #endif
            // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
        }
    } else { //n8k
        ll_ctx.z_wheel_data = buf[5];

        if (app_mouse_usb_ep_buf_has_data())
        {
            usb0hw_flush_tx_fifo(HID_MOUSE_IN_ENDPOINT_NUM);
            ll_ctx.z_wheel_data += last_ms_data.z_wheel;
        }
    }

    buf[5] = ll_ctx.z_wheel_data;


    #if TPSLL_MOUSE_BTN_FILTER_EN
    if(ll_ctx.ms_btn_rec.last != buf[0]) {
        // if (buf[0] && ((ll_ctx.ms_btn_rec.last | buf[0]) > ll_ctx.ms_btn_rec.last)) { //press
        if ((ll_ctx.ms_btn_rec.last | buf[0]) > ll_ctx.ms_btn_rec.last) { //press
            if(ll_ctx.ms_btn_rec.last & buf[0])//another btn press at the same time
            {
                ret = app_mouse_report_to_usb_internal(buf);
            }
            else if (ll_ctx.ms_btn_rec.rls_tick) {
                if (clock_time_exceed(ll_ctx.ms_btn_rec.rls_tick, TPSLL_BTN_MIN_TIME_US) && clock_time_exceed(ll_ctx.ms_btn_rec.hold_tick, TPSLL_BTN_MIN_TIME_US)) { //enough time
                    ret = app_mouse_report_to_usb_internal(buf);
                }
            }
            ll_ctx.ms_btn_rec.prs_tick = clock_time() | 1;
        } else { //release
                if ((ll_ctx.ms_btn_rec.last & buf[0]) && (ll_ctx.ms_btn_rec.last & buf[0]) < ll_ctx.ms_btn_rec.last)//another btn release at the same time
                {
                ret = app_mouse_report_to_usb_internal(buf);
                }
            else if (ll_ctx.ms_btn_rec.prs_tick) {
                if (clock_time_exceed(ll_ctx.ms_btn_rec.prs_tick, TPSLL_BTN_MIN_TIME_US)) { //enough time
                    ret = app_mouse_report_to_usb_internal(buf);
                }
            }
            ll_ctx.ms_btn_rec.rls_tick = clock_time() | 1;
        }

        ll_ctx.ms_btn_rec.last = buf[0];
    } else {
        ret = app_mouse_report_to_usb_internal(buf);
        if (buf[0]) { //press
            ll_ctx.ms_btn_rec.hold_tick = clock_time() | 1;
        }
        // ll_ctx.ms_btn_rec.last = buf[0];
    }
    #else
    ret = app_mouse_report_to_usb_internal(buf);
    #endif
    last_ms_data.z_wheel = buf[5];
    #if TPSLL_X_Y_FLUSH_HANDLE_EN
    last_ms_data.x = *((signed short*)(buf + 1));
    last_ms_data.y = *((signed short*)(buf + 3));
    #endif

    ll_ctx.z_wheel_data = 0;

    return ret;
}
#endif /* TPSLL_LOCAL_MS_DATA_TEST_EN */



_attribute_ram_code_sec_ unsigned char app_mouse_report_data(unsigned char *buf)
{
    uint8_t ret = 10;

#if TPSLL_LOCAL_MS_DATA_TEST_EN
    return 0;
#endif

    if (ll_ctx.conn_ignore_data_cnt) {
        return 0;
    }

    if (g_local_data_en)
    {
        if (app_mouse_usb_ep_buf_has_data())
        {
            return ret;
        }

        unsigned char tmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        t_mouse_data *ms = (t_mouse_data *)tmp;
        if(s_app_cb && s_app_cb->draw_square_test)
            s_app_cb->draw_square_test(&ms->x, &ms->y);
        ret = app_mouse_report_to_usb_internal(&tmp[1]);
        if(ret)
        { // fail
            // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
            // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
            // app_rf_buf_print((unsigned char *)mouse_rx, 16);
            // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);

        }
        else
        {
            // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
        }
    }else {
        if (ll_ctx.report_rate & REPORT_RATE_8K)
        {
#if 0 //use local data report for debug, not remove 

        unsigned char tmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        t_mouse_data *ms = (t_mouse_data *)tmp;
        // Draw_a_square_test(&ms->x, &ms->y);
        Draw_a_circle(&ms->x, &ms->y);

        if (app_mouse_8k_usb_ep_buf_has_data())
        {
            last_ms_data.x += *((int16_t *)(tmp + 2));
            last_ms_data.y += *((int16_t *)(tmp + 4));
            ZW_TG(GPIO_PH0);
            return;
        }

        *((int16_t *)(tmp + 2)) += last_ms_data.x;
        *((int16_t *)(tmp + 4)) += last_ms_data.y;

        
         ret = app_mouse_report_to_usb_internal(&tmp[1]);
         last_ms_data.x = 0;
         last_ms_data.y = 0;
         ZW_TG(GPIO_PG6);
         return;
#endif

    #if RF_PKT_IDX_DEBUG_EN
            // int16_t x = *((int16_t *)(buf + 1));
            // debug_gpio_with_cnt(x);
    #endif

            if (app_mouse_8k_usb_ep_buf_has_data())
            {
#if TPSLL_USB_DATA_CACHE_EN
                lost_ms_data.x += *((int16_t *)(buf + 1));
                lost_ms_data.y += *((int16_t *)(buf + 3));
                lost_ms_data.z_wheel += ll_ctx.z_wheel_data;
                lost_ms_data.t_wheel += ll_ctx.t_wheel_data;
                lost_ms_data.btn |= buf[0];
                return ret;
#else

                usb0hw_flush_tx_fifo(HID_MOUSE_IN_ENDPOINT_NUM);
                ll_ctx.z_wheel_data += last_ms_data.z_wheel;
                ll_ctx.t_wheel_data += last_ms_data.t_wheel;
#if TPSLL_X_Y_FLUSH_HANDLE_EN
                *((int16_t *)(buf + 1)) += last_ms_data.x;
                *((int16_t *)(buf + 3)) += last_ms_data.y;
#endif

#endif
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
            }
        }
        else
        { // n8k
            ll_ctx.z_wheel_data = buf[5];
            ll_ctx.t_wheel_data = buf[6];

            if (app_mouse_usb_ep_buf_has_data())
            {
#if TPSLL_USB_DATA_CACHE_EN
                lost_ms_data.x += *((int16_t *)(buf + 1));
                lost_ms_data.y += *((int16_t *)(buf + 3));
                lost_ms_data.z_wheel += buf[5];
                lost_ms_data.t_wheel += buf[6];
                lost_ms_data.btn |= buf[0];
                return ret;
#else

                usb0hw_flush_tx_fifo(HID_MOUSE_IN_ENDPOINT_NUM);
                ll_ctx.z_wheel_data += last_ms_data.z_wheel;
                ll_ctx.t_wheel_data += last_ms_data.t_wheel;

#endif
            }
        }

#if TPSLL_USB_DATA_CACHE_EN
        *((int16_t *)(buf + 1)) += lost_ms_data.x;
        *((int16_t *)(buf + 3)) += lost_ms_data.y;
        ll_ctx.z_wheel_data += lost_ms_data.z_wheel;
        ll_ctx.z_wheel_data += lost_ms_data.t_wheel;
        buf[0] |= lost_ms_data.btn;
#endif

        buf[5] = ll_ctx.z_wheel_data;
        buf[6] = ll_ctx.t_wheel_data;

#if 0
        signed short x_val = buf[1] | (buf[2] << 8);
        if(x_val == 0x6098)
        {
            return 1;
        }
#endif

        #if TPSLL_MOUSE_BTN_FILTER_EN
        if(ll_ctx.ms_btn_rec.last != buf[0]) {
            // if (buf[0] && ((ll_ctx.ms_btn_rec.last | buf[0]) > ll_ctx.ms_btn_rec.last)) { //press
            if ((ll_ctx.ms_btn_rec.last | buf[0]) > ll_ctx.ms_btn_rec.last) { //press
                if(ll_ctx.ms_btn_rec.last & buf[0])//another btn press at the same time
                {
                    ret = app_mouse_report_to_usb_internal(buf);
                }
                else if (ll_ctx.ms_btn_rec.rls_tick) {
                    if (clock_time_exceed(ll_ctx.ms_btn_rec.rls_tick, TPSLL_BTN_MIN_TIME_US) && clock_time_exceed(ll_ctx.ms_btn_rec.hold_tick, TPSLL_BTN_MIN_TIME_US)) { //enough time
                        ret = app_mouse_report_to_usb_internal(buf);
                    }
                }
                ll_ctx.ms_btn_rec.prs_tick = clock_time() | 1;
            } else { //release
                 if ((ll_ctx.ms_btn_rec.last & buf[0]) && (ll_ctx.ms_btn_rec.last & buf[0]) < ll_ctx.ms_btn_rec.last)//another btn release at the same time
                 {
                    ret = app_mouse_report_to_usb_internal(buf);
                 }
                else if (ll_ctx.ms_btn_rec.prs_tick) {
                    if (clock_time_exceed(ll_ctx.ms_btn_rec.prs_tick, TPSLL_BTN_MIN_TIME_US)) { //enough time
                        ret = app_mouse_report_to_usb_internal(buf);
                    }
                }
                ll_ctx.ms_btn_rec.rls_tick = clock_time() | 1;
            }

            ll_ctx.ms_btn_rec.last = buf[0];
        } else {
            ret = app_mouse_report_to_usb_internal(buf);
            if (buf[0]) { //press
                ll_ctx.ms_btn_rec.hold_tick = clock_time() | 1;
            }
            // ll_ctx.ms_btn_rec.last = buf[0];
        }
        #else
        ret = app_mouse_report_to_usb_internal(buf);
        #endif
        
        #if TPSLL_USB_DATA_CACHE_EN
        tmemset((void*)&lost_ms_data, 0, sizeof(lost_ms_data));
        #else



        #if TPSLL_X_Y_FLUSH_HANDLE_EN
        last_ms_data.x = *((int16_t *)(buf + 1));
        last_ms_data.y = *((int16_t *)(buf + 3));
        last_ms_data.z_wheel = buf[5];
        last_ms_data.t_wheel = buf[6];
        #endif
        #endif
        
    }

    ll_ctx.z_wheel_data = 0;
    ll_ctx.t_wheel_data = 0;

    return ret;
}

#if TPSLL_PACKET_LOSS_COMPENSATE_EN
//2.5us @96M
_attribute_ram_code_sec_ void tpsll_kalman_filtering_update(uint8_t *buf)
{
    int16_t x = *((int16_t *)(buf + 1));
    int16_t y = *((int16_t *)(buf + 3));

#if RF_PKT_IDX_DEBUG_EN
    debug_gpio_with_cnt(x);
#endif

    uint32_t dt_meas = 125;
    bool have_h_d = false;

    if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
    {
        kalman_vel_update(&g_kf, x, y, dt_meas);
        ll_ctx.is_pkt2_reported = true;
    }
    else
    {

        if (!ll_ctx.is_rp_pkt_loss)
        {
            dt_meas = 125;
        }
        else
        {
            uint16_t dfno = ll_ctx.fno_last - ll_ctx.pred_fno;
            uint16_t mt = 1;
            if (ll_ctx.is_pred_2)
            {
                mt = dfno * 2;
                
                ZW_TG(GPIO_PH1);
            }
            else
            {
                ZW_TG(GPIO_PH1);
                mt = dfno * 2 + 1;
                ZW_TG(GPIO_PH1);
            }

            if(ll_ctx.is_pkt1_reported)
            {
                mt -= 1;
            }
            
            #if 1
            if(dfno == 1 && (ll_ctx.last_intv_pkt_st != TPSLL_PKT_ALL_LOSS))
            {
                if(ll_ctx.last_intv_pkt_st != TPSLL_PKT_ALL_LOSS)
                {
                    dt_meas = (mt - ll_ctx.pkt2_rp_num) * 125;
                    have_h_d = true;
                }else{
                    // ZW_TG(GPIO_PH0);
                    dt_meas = 125;
                    // ZW_TG(GPIO_PH0);
                    // ZW_TG(GPIO_PH0);

                }
                // ZW_TG(GPIO_PH0);

            }else
            #endif
            
            {

                if(ll_ctx.is_rp_pkt_loss && (((ll_ctx.last_ack_mask & 7)) || (ll_ctx.is_pkt1_reported) ))
                {
                    dt_meas = (mt - ll_ctx.pkt2_rp_num) * 125;
                    // ZW_TG(GPIO_PH0);
                    have_h_d = true;
                    // ZW_TG(GPIO_PH0);
                }
                #if 0
                else if(ll_ctx.is_rp_pkt_loss && ll_ctx.is_pkt1_reported )
                {
                    ZW_TG(GPIO_PH0);
                    dt_meas = (mt - ll_ctx.pkt2_rp_num) * 125;
                    have_h_d = true;
                    ZW_TG(GPIO_PH0);
                    ZW_TG(GPIO_PH0);
                }
                #endif
                else if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_LOSS){
                    // ZW_TG(GPIO_PH0);
                    // ZW_TG(GPIO_PH0);
                    dt_meas = 125;
                    // ZW_TG(GPIO_PH0);
                    // ZW_TG(GPIO_PH0);
                }
            }
            // dt_meas = (dfno * 2 + 1 - ll_ctx.pkt2_rp_num) * 125;
            // dt_meas = (dfno * 2 + ll_ctx.pkt2_loss_num) * 125;
            // if (ll_ctx.pkt2_rp_num)
            // {
            //     dt_meas = (dfno * 2 - ll_ctx.pkt2_rp_num) * 125;
            // }
            // else
            // {
            //     dt_meas = (dfno + 2) * 125;
            // }

        }
        kalman_vel_update(&g_kf, x, y, dt_meas);

        if (have_h_d)
        {

            // if (ll_ctx.is_rp_pkt_loss)
            {
                *((int16_t *)(buf + 1)) -= ll_ctx.pred_acc.x;
                *((int16_t *)(buf + 3)) -= ll_ctx.pred_acc.y;

                ll_ctx.pred_num = 0;
                ll_ctx.pred_acc.x = 0;
                ll_ctx.pred_acc.y = 0;
            }
            ll_ctx.is_rp_pkt_loss = false;
            ll_ctx.pred_fno = 0;
            ll_ctx.pkt2_rp_num = 0;
            ll_ctx.pkt2_loss_num = 0;
            ll_ctx.is_pred_2 = false;
        }
        

        ll_ctx.is_pkt1_reported = true;

        #if 0
        uint32_t nm = dt_meas / 125;
        // static uint8_t dtcnt = 0;
        // if(nm != 1 && dtcnt < 10)
        // {
        //     dtcnt++;
        //     tlkapi_printf(1, "%d dt_meas:%d nm:%d\n",dtcnt, dt_meas, nm);
        //     debug_gpio_with_cnt11(nm);
        // }

        debug_gpio_with_cnt11(nm);
        #endif
    }
    ll_ctx.pred_level = KF_PRED_MOVE_LEVEL_NONE;


    
   

}

int abs_sum(signed short a, signed short b)
{

    int32_t x = (a >= 0) ? a : -a;
    int32_t y = (b >= 0) ? b : -b;
    return x + y;
}

#endif

uint8_t tpsll_alg_set_enable(TPSLL_ALG_STATUS_E enable)
{
    if(enable == TPSLL_ALG_ENABLE || enable == TPSLL_ALG_DISABLE)
    {
        g_kalman_enable = enable;
    }
    return TLK_SUCCESS;
}

TPSLL_ALG_STATUS_E tpsll_alg_get_status(void)
{
    return g_kalman_enable;
}

_attribute_ram_code_sec_ static void report_mouse_data_try(void)
{
    if (ll_ctx.report_mode == REPORT_MODE_FAST)
    {
        if (ll_ctx.step_3t1r == STEP_3T1R_2T_D_2R)
        {

            if (!ll_ctx.is_last_report && (g_ms_need_report[(ms_idx) & 3] == 1))
            {
                mouse_rx[(ms_idx + 1) & 3].x = mouse_rx[(ms_idx + 1) & 3].x + mouse_rx[(ms_idx) & 3].x;
                mouse_rx[(ms_idx + 1) & 3].y = mouse_rx[(ms_idx + 1) & 3].y + mouse_rx[(ms_idx) & 3].y;
                mouse_rx[(ms_idx + 1) & 3].btn |= mouse_rx[(ms_idx) & 3].btn;
                #if (ABNORMAL_RETRY_FIX)
                g_ms_need_report[(ms_idx + 1) & 3] = 1;
                #else
                g_ms_need_report[(ms_idx) & 3] = 0;
                #endif
            }

            if (g_ms_need_report[(ms_idx + 1) & 3] == 1)
            {
                if (app_mouse_report_data(&mouse_rx[(ms_idx + 1) & 3].btn) != 0)
                // if (app_mouse_report_data(&mouse_rx[(ms_idx + 1) & 3].btn) != 0)
                {
                    //                        tlkapi_send_string_data(APP_LOG_EN, "chn idx rssi sort(s->b)", channel_idx, 40);
                }

                #if (ABNORMAL_RETRY_FIX)
                tmemset(&mouse_rx[(ms_idx + 1) & 3].btn, 0, 6);
                #endif
                g_ms_need_report[(ms_idx + 1) & 3] = 0;
            }
            else
            {
                ll_ctx.is_last_report = false;
            }
#if 0
            else
            { // report the previous one
                if (app_mouse_report_data(&mouse_rx[(ms_idx) & 3].btn) != 0)
                // if (app_mouse_report_data(&mouse_rx[(ms_idx + 1) & 3].btn) != 0)
                {
                    //                        tlkapi_send_string_data(APP_LOG_EN, "chn idx rssi sort(s->b)", channel_idx, 40);
                }
            }
#endif
            #if (ABNORMAL_RETRY_FIX)
            if (!ll_ctx.is_last_report && (g_ms_need_report[(ms_idx) & 3] == 1))
            {
                tmemset(&mouse_rx[(ms_idx) & 3].btn, 0, 6);
                g_ms_need_report[(ms_idx) & 3] = 0;
            }
            #endif
        }
        else if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
        {
            if (!ll_ctx.is_last_report && (g_ms_need_report[(ms_idx + 1) & 3] == 1))
            {
                mouse_rx[(ms_idx + 2) & 3].x = mouse_rx[(ms_idx + 2) & 3].x + mouse_rx[(ms_idx + 1) & 3].x;
                mouse_rx[(ms_idx + 2) & 3].y = mouse_rx[(ms_idx + 2) & 3].y + mouse_rx[(ms_idx + 1) & 3].y;
                mouse_rx[(ms_idx + 2) & 3].btn |= mouse_rx[(ms_idx + 1) & 3].btn;
                #if (ABNORMAL_RETRY_FIX)
                g_ms_need_report[(ms_idx + 2) & 3] = 1;
                #else
                g_ms_need_report[(ms_idx + 1) & 3] = 0;
                #endif
            }

            #if (ABNORMAL_RETRY_FIX)
            if (g_ms_need_report[(ms_idx + 2) & 3] == 1)
            {
            #endif
                if (app_mouse_report_data(&mouse_rx[(ms_idx + 2) & 3].btn) != 0)
                // if (app_mouse_report_data(&mouse_rx[(ms_idx + 1) & 3].btn) != 0)
                {
                    //                        tlkapi_send_string_data(APP_LOG_EN, "chn idx rssi sort(s->b)", channel_idx, 40);
                }

                #if (ABNORMAL_RETRY_FIX)
                tmemset(&mouse_rx[(ms_idx + 2) & 3].btn, 0, 6);
                #endif
                g_ms_need_report[(ms_idx + 2) & 3] = 0;
            #if (ABNORMAL_RETRY_FIX)
            }
            #endif
        }
        else
        {
            ll_ctx.is_last_report = false;
        }
        #if (ABNORMAL_RETRY_FIX)
        if (!ll_ctx.is_last_report && (g_ms_need_report[(ms_idx + 1) & 3] == 1))
        {
            tmemset(&mouse_rx[(ms_idx + 1) & 3].btn, 0, 6);
            g_ms_need_report[(ms_idx + 1) & 3] = 0;
        }
        #endif
    }
    else if (ll_ctx.report_mode == REPORT_MODE_NORMAL)
    {

        #if 0 //use local data report for debug, not remove 

        unsigned char tmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        t_mouse_data *ms = (t_mouse_data *)tmp;
        Draw_a_square_test(&ms->x, &ms->y);
        Draw_a_circle(&ms->x, &ms->y);

        if (app_mouse_8k_usb_ep_buf_has_data())
        {
            last_ms_data.x += *((int16_t *)(tmp + 2));
            last_ms_data.y += *((int16_t *)(tmp + 4));
            ZW_TG(GPIO_PH0);
            return;
        }

        *((int16_t *)(tmp + 2)) += last_ms_data.x;
        *((int16_t *)(tmp + 4)) += last_ms_data.y;

        
         ret = app_mouse_report_to_usb_internal(&tmp[1]);
         last_ms_data.x = 0;
         last_ms_data.y = 0;
         ZW_TG(GPIO_PG6);
         return;
    #endif

#if TPSLL_PACKET_LOSS_COMPENSATE_EN
        float pred_dx, pred_dy;
        if(g_kalman_enable == TPSLL_ALG_ENABLE)
        {
            kalman_vel_predict(&g_kf, 125, &pred_dx, &pred_dy);
        }
#endif
        // todo
        if (g_ms_need_report[ms_rp_idx])
        {

#if RF_PKT_IDX_DEBUG_EN
            // debug_gpio_with_cnt11(ms_rp_idx + 1);
#endif
            #if TPSLL_PACKET_LOSS_COMPENSATE_EN
            if(g_kalman_enable == TPSLL_ALG_ENABLE)
            {
                tpsll_kalman_filtering_update((uint8_t *)&mouse_rx[ms_rp_idx].btn);
            }
            
            #endif

            if (app_mouse_report_data(&mouse_rx[ms_rp_idx].btn) != 0)
            {
            }
            else
            {
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
            }
#if (ABNORMAL_RETRY_FIX)
            tmemset(&mouse_rx[ms_rp_idx].btn, 0, 6);
#endif
            g_ms_need_report[ms_rp_idx] = 0;
            #if TPSLL_PACKET_LOSS_COMPENSATE_EN
            if(g_kalman_enable == TPSLL_ALG_ENABLE)
            {
                if (ll_ctx.is_rp_pkt_loss && ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
                {
                    ll_ctx.pkt2_rp_num++;
                }
            }
            #endif
        }

#if TPSLL_PACKET_LOSS_COMPENSATE_EN
        else
        {
            if(g_kalman_enable == TPSLL_ALG_ENABLE)
            {
                //700ns @96m
                if (ll_ctx.pred_num < TPSLL_KALMAN_PRED_MAX_NUM)
                {
                        // if (pred_dx > TPSLL_KALMAN_PRED_THRESHOLD)
                        // {
                        //     pred_dx = TPSLL_KALMAN_PRED_THRESHOLD;
                        // }else if(pred_dx < -TPSLL_KALMAN_PRED_THRESHOLD)
                        // {
                        //     pred_dx = -TPSLL_KALMAN_PRED_THRESHOLD;  
                        // }
                        // if (pred_dy > TPSLL_KALMAN_PRED_THRESHOLD)
                        // {
                        //     pred_dy = TPSLL_KALMAN_PRED_THRESHOLD;
                        // }else if(pred_dy < -TPSLL_KALMAN_PRED_THRESHOLD)
                        // {
                        //     pred_dy = -TPSLL_KALMAN_PRED_THRESHOLD;  
                        // }

                        if (ll_ctx.pred_level == KF_PRED_MOVE_LEVEL_NONE)
                        {
                            int16_t p_x = (int16_t)pred_dx;
                            int16_t p_y = (int16_t)pred_dy;

                            int16_t m_sum = abs_sum(p_x, p_y);

                            if (m_sum > TPSLL_KALMAN_PRED_MOVE_LEVEL_TH)
                            {
                                // ZW_TG(GPIO_PH0);
                                ll_ctx.pred_level = KF_PRED_MOVE_LEVEL_FAST;
                            }
                            else
                            {
                                // ZW_TG(GPIO_PG7);
                                ll_ctx.pred_level = KF_PRED_MOVE_LEVEL_NORMAL;
                            }
                        }

                        if (ll_ctx.pred_level == KF_PRED_MOVE_LEVEL_FAST)
                        {
                            // ZW_TG(GPIO_PG6);
                            int16_t f_x = (int16_t)pred_dx;
                            int16_t f_y = (int16_t)pred_dy;
                            int16_t s_x = (f_x > 0) ? 1 : ((f_x < 0) ? -1 : 0);
                            int16_t s_y = (f_y > 0) ? 1 : ((f_y < 0) ? -1 : 0);
                            if (ll_ctx.pred_num & 1)
                            {
                                mouse_rx[ms_rp_idx].x = -s_x;
                                mouse_rx[ms_rp_idx].y = -s_y;
                            }
                            else
                            {
                                mouse_rx[ms_rp_idx].x = s_x;
                                mouse_rx[ms_rp_idx].y = s_y;
                            }
                        }
                        else
                        {
                            // ZW_TG(GPIO_PG5);
                            if(ll_ctx.pred_num < TPSLL_KALMAN_PRED_REAL_NUM)
                            {
                                
                                mouse_rx[ms_rp_idx].x = pred_dx;
                                mouse_rx[ms_rp_idx].y = pred_dy;

                            }
                            else
                            {
                                int16_t t_x = (int16_t)pred_dx;
                                int16_t t_y = (int16_t)pred_dy;

                                if (t_x || t_y)
                                {
                                    int16_t s_x = (t_x > 0) ? 1 : ((t_x < 0) ? -1 : 0);
                                    int16_t s_y = (t_y > 0) ? 1 : ((t_y < 0) ? -1 : 0);
                                    if (ll_ctx.pred_num & 1)
                                    {
                                        mouse_rx[ms_rp_idx].x = -s_x;
                                        mouse_rx[ms_rp_idx].y = -s_y;
                                    }
                                    else
                                    {
                                        mouse_rx[ms_rp_idx].x = s_x;
                                        mouse_rx[ms_rp_idx].y = s_y;
                                    }
                                }else{
                                    mouse_rx[ms_rp_idx].x = 0;
                                    mouse_rx[ms_rp_idx].y = 0;
                                }
                            }
                        }

                    mouse_rx[ms_rp_idx].btn = ll_ctx.last_btn;
                    mouse_rx[ms_rp_idx].z_wheel = 0;
                    mouse_rx[ms_rp_idx].t_wheel = 0;
                    ZW_TG(GPIO_PH4);

                    
                    ll_ctx.pred_acc.x += mouse_rx[ms_rp_idx].x;
                    ll_ctx.pred_acc.y += mouse_rx[ms_rp_idx].y;

                    app_mouse_report_data(&mouse_rx[ms_rp_idx].btn);
                    ll_ctx.pred_num++;
                    ll_ctx.is_rp_pkt_loss = true;
                    // ZW_TG(GPIO_PH4);

                    if (!ll_ctx.pred_fno)
                    {
                        ll_ctx.pred_fno = ll_ctx.fno_last;
                        if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
                        {
                            ll_ctx.is_pred_2 = true;
                        }
                    }


                    ll_ctx.is_pkt1_reported = false;

                    #if 0
                    int8_t te11 = -2;

                    if(ll_ctx.pred_acc.x > 0)
                    {
                        if(pred_max_p.x < ll_ctx.pred_acc.x)
                        {
                        pred_max_p.x =  ll_ctx.pred_acc.x;
                        tlkapi_printf(1, "x:%d %d y:%d %d test:%d\n", pred_max_p.x, pred_max_n.x, pred_max_p.y, pred_max_n.y, te11)

                        }
                    }else if(ll_ctx.pred_acc.x < 0)
                    {
                        if(pred_max_n.x > ll_ctx.pred_acc.x)
                        {
                        pred_max_n.x =  ll_ctx.pred_acc.x;
                        tlkapi_printf(1, "x:%d %d y:%d %d test:%d\n", pred_max_p.x, pred_max_n.x, pred_max_p.y, pred_max_n.y, te11)

                        }
                    }

                    if(ll_ctx.pred_acc.y > 0)
                    {
                        if(pred_max_p.y < ll_ctx.pred_acc.y)
                        {
                        pred_max_p.y =  ll_ctx.pred_acc.y;
                        tlkapi_printf(1, "x:%d %d y:%d %d test:%d\n", pred_max_p.x, pred_max_n.x, pred_max_p.y, pred_max_n.y, te11)

                        }
                    }else if(ll_ctx.pred_acc.y < 0)
                    {
                        if(pred_max_n.y > ll_ctx.pred_acc.y)
                        {
                        pred_max_n.y =  ll_ctx.pred_acc.y;
                        tlkapi_printf(1, "x:%d %d y:%d %d test:%d\n", pred_max_p.x, pred_max_n.x, pred_max_p.y, pred_max_n.y, te11)
                        }
                    }
                    #endif
                }
                

                if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
                {
                    ll_ctx.is_pkt2_reported = false;
                    ll_ctx.pkt2_loss_num++;
                    
                }
                else{
                    ll_ctx.is_rp_pkt1_loss = true;
                }
            }
        }
#endif
    }

    // ms_rp_idx = (ms_rp_idx + 1) & 3;
}

_attribute_ram_code_sec_ static void report_mouse_data(void)
{

    // if (!app_mouse_usb_ep_buf_has_data())

    // pkt1 to usb
    #if 0
    if (g_local_data_en)
    {
        unsigned char tmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        t_mouse_data *ms = (t_mouse_data *)tmp;
        Draw_a_square_test(&ms->x, &ms->y);
        if (app_mouse_report_data(&tmp[1]) != 0)
        { // fail
            DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
            DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
            // app_rf_buf_print((unsigned char *)mouse_rx, 16);
            DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
        }
        else
        {
            DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
        }
    }
    else
    #endif
    {
        // if (ll_ctx.step_3t1r == STEP_3T1R_1T_D_1R)
        if (ll_ctx.report_mode == REPORT_MODE_FAST)
        {

            if (ll_ctx.step_3t1r == STEP_3T1R_2T_D_2R)
            {
                // The first data report after switching the report mode is always in 1R.
                // if (!ll_ctx.switch_special_cnt)
                {
                    if (g_ms_need_report[(ms_idx + 1) & 3] == 1)
                    {

                        if (!ll_ctx.is_last_report && (g_ms_need_report[(ms_idx) & 3] == 1))
                        {
                            mouse_rx[(ms_idx + 1) & 3].x = mouse_rx[(ms_idx + 1) & 3].x + mouse_rx[(ms_idx) & 3].x;
                            mouse_rx[(ms_idx + 1) & 3].y = mouse_rx[(ms_idx + 1) & 3].y + mouse_rx[(ms_idx) & 3].y;
                            #if (!ABNORMAL_RETRY_FIX)
                            g_ms_need_report[(ms_idx) & 3] = 0;
                            #endif
                        }

                        if (g_ms_need_report[(ms_idx + 2) & 3] == 1)
                        {
                            if(mouse_rx[(ms_idx + 2) & 3].btn != 0)
                            {
                                mouse_rx[(ms_idx + 1) & 3].btn = mouse_rx[(ms_idx + 1) & 3].btn | mouse_rx[(ms_idx + 2) & 3].btn;
                            }
                        }

                        // ms_data_check((ms_idx + 1) & 3);
                        
                        if (app_mouse_report_data(&mouse_rx[(ms_idx + 1) & 3].btn) != 0)
                        // if (app_mouse_report_data(&mouse_rx[(ms_idx + 1) & 3].btn) != 0)
                        {
                            //                        tlkapi_send_string_data(APP_LOG_EN, "chn idx rssi sort(s->b)", channel_idx, 40);
                        }

                        #if (ABNORMAL_RETRY_FIX)
                        tmemset(&mouse_rx[(ms_idx + 1) & 3].btn, 0, 6);
                        #endif
                        g_ms_need_report[(ms_idx + 1) & 3] = 0;
                        ll_ctx.is_last_report = true;

                        #if (ABNORMAL_RETRY_FIX)
                        if (!ll_ctx.is_last_report && (g_ms_need_report[(ms_idx) & 3] == 1))
                        {
                            tmemset(&mouse_rx[(ms_idx) & 3].btn, 0, 6);
                            g_ms_need_report[(ms_idx) & 3] = 0;
                        }
                        #endif
                    }
                    // else
                    // {
                    //     // ms_data_check((ms_idx) & 3);
                    //     if (app_mouse_report_to_usb_internal(&mouse_rx[(ms_idx) & 3].btn) != 0)
                    //     {
                    //     }
                    // }
                }
#if 0
                    else
                    {
                        /**
                         * When switching from normal to fast mode, the package being reported this time should be reported
                         * together with the two subsequent packages that would otherwise need to be reported.
                         *
                         */
                        mouse_rx[ms_rp_idx].x = mouse_rx[ms_rp_idx].x + mouse_rx[(ms_rp_idx + 1) & 3].x + mouse_rx[(ms_rp_idx + 2) & 3].x;
                        mouse_rx[ms_rp_idx].y = mouse_rx[ms_rp_idx].y + mouse_rx[(ms_rp_idx + 1) & 3].y + mouse_rx[(ms_rp_idx + 2) & 3].y;
                        if (g_ms_need_report[ms_rp_idx] == 1)
                        {

                            ms_data_check((ms_rp_idx) & 3);

                            if (app_mouse_report_data(&mouse_rx[ms_rp_idx].btn) != 0)
                            // if (app_mouse_report_data(&mouse_rx[ms_rp_idx].btn) != 0)
                            {
                            }

                            g_ms_need_report[ms_rp_idx] = 0;
                        }
                    }
#endif
            }
            else if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
            {
                if (g_ms_need_report[(ms_idx + 2) & 3] == 1)
                {
                    if (!ll_ctx.is_last_report && (g_ms_need_report[(ms_idx + 1) & 3] == 1))
                    {
                        mouse_rx[(ms_idx + 2) & 3].x = mouse_rx[(ms_idx + 2) & 3].x + mouse_rx[(ms_idx + 1) & 3].x;
                        mouse_rx[(ms_idx + 2) & 3].y = mouse_rx[(ms_idx + 2) & 3].y + mouse_rx[(ms_idx + 1) & 3].y;
                        #if (!ABNORMAL_RETRY_FIX)
                        g_ms_need_report[(ms_idx + 1) & 3] = 0;
                        #endif
                    }

                    // ms_data_check((ms_idx + 2) & 3);

                    // if (g_ms_need_report[(ms_idx + 2) & 3] == 1)
                    // {
                    //     if (mouse_rx[(ms_idx + 2) & 3].btn != 0)
                    //     {
                    //         mouse_rx[(ms_idx + 1) & 3].btn = mouse_rx[(ms_idx + 1) & 3].btn | mouse_rx[(ms_idx + 2) & 3].btn;
                    //     }
                    // }

                                if(ms3_cnt >= 1 && ms3_cnt < 3)
                                {

                                    // tlkapi_send_string_data(APP_LOG_EN, "rpb", &mouse_rx[(ms_idx + 2) & 3].btn, 6);
                                }

                    if (app_mouse_report_data(&mouse_rx[(ms_idx + 2) & 3].btn) != 0)
                    // if (app_mouse_report_data(&mouse_rx[(ms_idx + 2) & 3].btn) != 0)
                    {
                    }
                    #if (ABNORMAL_RETRY_FIX)
                    tmemset(&mouse_rx[(ms_idx + 2) & 3].btn, 0, 6);
                    #endif
                    g_ms_need_report[(ms_idx + 2) & 3] = 0;
                    ll_ctx.is_last_report = true;

                    #if (ABNORMAL_RETRY_FIX)
                    if (!ll_ctx.is_last_report && (g_ms_need_report[(ms_idx + 1) & 3] == 1))
                    {
                        tmemset(&mouse_rx[(ms_idx + 1) & 3].btn, 0, 6);
                        g_ms_need_report[(ms_idx + 1) & 3] = 0;
                    }
                    #endif
                }
                // else
                // {
                //     ll_ctx.is_last_report = false;
                // }
            }
        }
        else if (ll_ctx.report_mode == REPORT_MODE_NORMAL)
        {
            #if TPSLL_PACKET_LOSS_COMPENSATE_EN
                float pred_dx, pred_dy;
                if(g_kalman_enable == TPSLL_ALG_ENABLE)
                {    
                    kalman_vel_predict(&g_kf, 125, &pred_dx, &pred_dy);
                }
            #endif



            if (g_ms_need_report[ms_rp_idx])
            {

// ms_data_check((ms_rp_idx) & 3);
#if RF_PKT_IDX_DEBUG_EN
                // debug_gpio_with_cnt11(ms_rp_idx + 1);
#endif


                #if TPSLL_PACKET_LOSS_COMPENSATE_EN
                if(g_kalman_enable == TPSLL_ALG_ENABLE)
                {
                    tpsll_kalman_filtering_update((uint8_t *)&mouse_rx[ms_rp_idx].btn);
                }
                #endif

                if (app_mouse_report_data(&mouse_rx[ms_rp_idx].btn) != 0)
                { // fail

                }
                else
                {
                    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                }
#if (ABNORMAL_RETRY_FIX)
                tmemset(&mouse_rx[(ms_rp_idx) & 3].btn, 0, 6);
                g_ms_need_report[ms_rp_idx] = 0;

                #if TPSLL_PACKET_LOSS_COMPENSATE_EN
                if(g_kalman_enable == TPSLL_ALG_ENABLE)
                {
                    if (ll_ctx.is_rp_pkt_loss && ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
                    {
                        ll_ctx.pkt2_rp_num++;
                    }
                }
                #endif
#endif
            }
            #if TPSLL_PACKET_LOSS_COMPENSATE_EN
            else
            {
                if(g_kalman_enable == TPSLL_ALG_ENABLE)
                {
                        //700ns @96m
                    if(ll_ctx.pred_num < TPSLL_KALMAN_PRED_MAX_NUM)
                    {
                        // int8_t te11 = -2;

                        // if (pred_dx > TPSLL_KALMAN_PRED_THRESHOLD)
                        // {
                        //     pred_dx = TPSLL_KALMAN_PRED_THRESHOLD;
                        // }else if(pred_dx < -TPSLL_KALMAN_PRED_THRESHOLD)
                        // {
                        //     pred_dx = -TPSLL_KALMAN_PRED_THRESHOLD;  
                        // }
                        // if (pred_dy > TPSLL_KALMAN_PRED_THRESHOLD)
                        // {
                        //     pred_dy = TPSLL_KALMAN_PRED_THRESHOLD;
                        // }else if(pred_dy < -TPSLL_KALMAN_PRED_THRESHOLD)
                        // {
                        //      pred_dy = -TPSLL_KALMAN_PRED_THRESHOLD;  
                        // }
                        if (ll_ctx.pred_level == KF_PRED_MOVE_LEVEL_NONE)
                        {
                            int16_t p_x = (int16_t)pred_dx;
                            int16_t p_y = (int16_t)pred_dy;

                            int16_t m_sum = abs_sum(p_x, p_y);

                            if (m_sum > TPSLL_KALMAN_PRED_MOVE_LEVEL_TH)
                            {
                                // ZW_TG(GPIO_PH0);
                                ll_ctx.pred_level = KF_PRED_MOVE_LEVEL_FAST;
                            }
                            else
                            {
                                // ZW_TG(GPIO_PG7);
                                ll_ctx.pred_level = KF_PRED_MOVE_LEVEL_NORMAL;
                            }
                        }

                        if (ll_ctx.pred_level == KF_PRED_MOVE_LEVEL_FAST)
                        {
                            // ZW_TG(GPIO_PG6);
                            int16_t f_x = (int16_t)pred_dx;
                            int16_t f_y = (int16_t)pred_dy;
                            int16_t s_x = (f_x > 0) ? 1 : ((f_x < 0) ? -1 : 0);
                            int16_t s_y = (f_y > 0) ? 1 : ((f_y < 0) ? -1 : 0);
                            if (ll_ctx.pred_num & 1)
                            {
                                mouse_rx[ms_rp_idx].x = -s_x;
                                mouse_rx[ms_rp_idx].y = -s_y;
                            }
                            else
                            {
                                mouse_rx[ms_rp_idx].x = s_x;
                                mouse_rx[ms_rp_idx].y = s_y;
                            }
                        }
                        else
                        {
                            // ZW_TG(GPIO_PG5);
                            if (ll_ctx.pred_num < TPSLL_KALMAN_PRED_REAL_NUM)
                            {
                                
                                mouse_rx[ms_rp_idx].x = pred_dx;
                                mouse_rx[ms_rp_idx].y = pred_dy;

                            }
                            else
                            {

                                int16_t t_x = (int16_t)pred_dx;
                                int16_t t_y = (int16_t)pred_dy;

                                if (t_x || t_y)
                                {
                                    int16_t s_x = (t_x > 0) ? 1 : ((t_x < 0) ? -1 : 0);
                                    int16_t s_y = (t_y > 0) ? 1 : ((t_y < 0) ? -1 : 0);
                                    if (ll_ctx.pred_num & 1)
                                    {
                                        mouse_rx[ms_rp_idx].x = -s_x;
                                        mouse_rx[ms_rp_idx].y = -s_y;
                                    }
                                    else
                                    {
                                        mouse_rx[ms_rp_idx].x = s_x;
                                        mouse_rx[ms_rp_idx].y = s_y;
                                    }
                                }
                                else
                                {
                                    mouse_rx[ms_rp_idx].x = 0;
                                    mouse_rx[ms_rp_idx].y = 0;
                                }
                            }
                        }
                    mouse_rx[ms_rp_idx].btn = ll_ctx.last_btn;
                    mouse_rx[ms_rp_idx].z_wheel = 0;
                    mouse_rx[ms_rp_idx].t_wheel = 0;

                    ll_ctx.pred_acc.x += mouse_rx[ms_rp_idx].x;
                    ll_ctx.pred_acc.y += mouse_rx[ms_rp_idx].y;

                    app_mouse_report_data(&mouse_rx[ms_rp_idx].btn);
                    ll_ctx.pred_num++;
                    ll_ctx.is_rp_pkt_loss = true;

                    ZW_TG(GPIO_PH4);

                    if (!ll_ctx.pred_fno)
                    {
                        ll_ctx.pred_fno = ll_ctx.fno_last;
                        if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
                        {
                            ll_ctx.is_pred_2 = true;
                        }
                    }


                    ll_ctx.is_pkt1_reported = false;

                    #if 0

                    if(ll_ctx.pred_acc.x > 0)
                    {
                        if(pred_max_p.x < ll_ctx.pred_acc.x)
                        {
                        pred_max_p.x =  ll_ctx.pred_acc.x;
                        tlkapi_printf(1, "x:%d %d y:%d %d test:%d\n", pred_max_p.x, pred_max_n.x, pred_max_p.y, pred_max_n.y, te11)

                        }
                    }else if(ll_ctx.pred_acc.x < 0)
                    {
                        if(pred_max_n.x > ll_ctx.pred_acc.x)
                        {
                        pred_max_n.x =  ll_ctx.pred_acc.x;
                        tlkapi_printf(1, "x:%d %d y:%d %d test:%d\n", pred_max_p.x, pred_max_n.x, pred_max_p.y, pred_max_n.y, te11)

                        }
                    }

                    if(ll_ctx.pred_acc.y > 0)
                    {
                        if(pred_max_p.y < ll_ctx.pred_acc.y)
                        {
                        pred_max_p.y =  ll_ctx.pred_acc.y;
                        tlkapi_printf(1, "x:%d %d y:%d %d test:%d\n", pred_max_p.x, pred_max_n.x, pred_max_p.y, pred_max_n.y, te11)

                        }
                    }else if(ll_ctx.pred_acc.y < 0)
                    {
                        if(pred_max_n.y > ll_ctx.pred_acc.y)
                        {
                        pred_max_n.y =  ll_ctx.pred_acc.y;
                        tlkapi_printf(1, "x:%d %d y:%d %d test:%d\n", pred_max_p.x, pred_max_n.x, pred_max_p.y, pred_max_n.y, te11)
                        }
                    }
                    #endif

                        
                    }
                    if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
                    {
                        ll_ctx.is_pkt2_reported = false;
                        ll_ctx.pkt2_loss_num++;
                    }
                }
                
            }
            #endif

#if (!ABNORMAL_RETRY_FIX)
            g_ms_need_report[ms_rp_idx] = 0;
#endif

#if 0
            else
            {
                /**
                 * When switching from fast report mode  to normal report mode, the two packages
                 * starting from the current packet do not need to be reported.
                 * **/
                ll_ctx.switch_special_cnt--;
            }
#endif

#if TPSLL_PACKET_LOSS_COMPENSATE_EN
if(g_kalman_enable == TPSLL_ALG_ENABLE)
{
    // #define PRED_THRESHOLD 40
    // int16_t r_x = (int16_t)pred_dx;
    // int16_t r_y = (int16_t)pred_dy;
    // if(r_x > TPSLL_KALMAN_PRED_THRESHOLD || r_x < -TPSLL_KALMAN_PRED_THRESHOLD )
    // {
    //     ZW_TG(GPIO_PH2);
    // }
    // if(r_y > TPSLL_KALMAN_PRED_THRESHOLD || r_y < -TPSLL_KALMAN_PRED_THRESHOLD )
    // {
    //     ZW_TG(GPIO_PH3);
    // }
}

#endif
        }
    }

    // ms_rp_idx = (ms_rp_idx + 1) & 3;
}

#if TPSLL_NEW_8K_RF_TIMING_EN


#if TPSLL_NEW_8K_RF_PACKET_FORMAT_EN

#define XY_D_TH (20)



#if TPSLL_8K_COMPRESS_XY_DATA_EN

static uint8_t nr = 0;
static uint8_t nr_s = 0;

_attribute_ram_code_sec_ static void store_ms_data2_buffer(rf_packet_t *p)
{
    #if 0 //zw add for debug, not remove
    uint8_t  is_valid = 0;
    #endif

    if (ll_ctx.step_3t1r == STEP_3T1R_1T_D_1R)
    {

        // pkt1
        if (g_ms_need_report[(ms_idx - 1) & 3] == 0)
        {
            // tmemcpy(&mouse_rx[(ms_idx - 1) & 3].x, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN - 1);
            // int16_t a;
            // int16_t x_v = (int16_t)p->buf[0];
            // mouse_rx[(ms_idx + 2) & 3].x = a - MAX_XY;

            // a = (int16_t)p->buf[0];
            // mouse_rx[(ms_idx - 1) & 3].x = a - MAX_XY;
            mouse_rx[(ms_idx - 1) & 3].x = (int16_t)p->buf[0] - MAX_XY;
            mouse_rx[(ms_idx - 1) & 3].y = (int16_t)p->buf[1];
            g_ms_need_report[(ms_idx - 1) & 3] = 1;
        }
  
       

        if (g_ms_need_report[(ms_idx) & 3] == 0)
        {
            // tmemcpy(&mouse_rx[(ms_idx) & 3].x, &p->buf[4], TPSLL_RX_MS_DATA_VALID_LEN - 1);
            mouse_rx[(ms_idx) & 3].x = (int16_t)p->buf[2];
            mouse_rx[(ms_idx) & 3].y = (int16_t)p->buf[3];
            g_ms_need_report[(ms_idx) & 3] = 1;
        }

         #if  TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        ms_xy_s_t *p0 = (ms_xy_t *)&(p->buf[0]);
        ms_xy_s_t *p1 = (ms_xy_s_t *)&(p->buf[2]);
        ms_xy_t *p2 = (ms_xy_t *)&(p->buf[5]);


        #if 0

        // ms_xy_t *p0 = (ms_xy_t *)&(p->buf[0]);
        // ms_xy_t *p1 = (ms_xy_t *)&(p->buf[4]);
        // ms_xy_t *p2 = (ms_xy_t *)&(p->buf[9]);

        
        if(p0->x > XY_D_TH || p0->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
        }
        if(p1->x > XY_D_TH || p1->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
            if(nr < 8)
            {
                tlkapi_send_string_data(APP_LOG_EN, "1rx", &p->dma_len, p->rf_len + 5);
                ZW_TG(GPIO_PG7);
                nr_s = 1;
                nr++;
            }
        }
        if(p2->x > XY_D_TH || p2->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
        }


        if(p0->y > XY_D_TH || p0->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
        }
        if(p1->y > XY_D_TH || p1->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
        }
        if(p2->y > XY_D_TH || p2->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
        }
        #endif


        #if  RF_PKT_IDX_DEBUG_EN
        debug_gpio_with_cnt_ph0(p0->x)
        debug_gpio_with_cnt_pg5(p1->x);
        debug_gpio_with_cnt_pg6(p2->x);
        
        #endif
        //  debug_gpio_with_cnt_pg5(p1->x);

        if (ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_RCVD)
        {
            p2->x -= ll_ctx.last_acc.x;
            p2->y -= ll_ctx.last_acc.y;
            ll_ctx.acc.x += p2->x;
            ll_ctx.acc.y += p2->y;
        }
        else if (ll_ctx.last_intv_pkt_st == TPSLL_PKT_2_LOSS)
        {
            p2->x = p2->x - ll_ctx.last_acc.x - p1->x;
            p2->y = p2->y - ll_ctx.last_acc.y - p1->y;
            ll_ctx.acc.x += p2->x + p1->x;
            ll_ctx.acc.y += p2->y + p1->y;
        }
        else if (ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_LOSS)
        {
            p2->x = p2->x - ll_ctx.last_acc.x - p0->x - p1->x;
            p2->y = p2->y - ll_ctx.last_acc.y - p0->y - p1->y;
            ll_ctx.acc.x += p2->x + p1->x + p0->x;
            ll_ctx.acc.y += p2->y + p1->y + p0->y;
        }

        #endif

        // pkt3  1R: The 3rd packet is always a new packet.
        tmemcpy(&mouse_rx[(ms_idx + 1) & 3].btn, &p->buf[8], TPSLL_RX_MS_DATA_VALID_LEN);
        g_ms_need_report[(ms_idx + 1) & 3] = 1;

        if (ll_ctx.report_mode == REPORT_MODE_NORMAL)
        {
            mouse_rx[0].btn = mouse_rx[1].btn = mouse_rx[2].btn = mouse_rx[3].btn = p->buf[4];
        }
        #if 0 //zw add for debug, not remove
        if ((p->buf[1] == 1 && p->buf[7] == 2) ||
            (p->buf[1] == 3 && p->buf[7] == 4) ||
            (p->buf[1] == 5 && p->buf[7] == 6) ||
            (p->buf[1] == 7 && p->buf[7] == 8))
        {
            is_valid = 1;
        }
        if (!is_valid)
        {
            // ZW_TG(GPIO_PH0);
            // tlkapi_send_string_data(APP_LOG_EN, "1R", &p->buf[0], 12);
        }
        #endif

        // debug_gpio_with_cnt_ph0(ms_idx + 1);

        #if  RF_PKT_IDX_DEBUG_EN && TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        // debug_gpio_with_cnt_ph0(ms_idx + 1);
        debug_gpio_with_cnt_pg6(ll_ctx.last_acc.x);
        ZW_TG(GPIO_PG4);
        
        debug_gpio_with_cnt_pg6(ll_ctx.acc.x);
        debug_gpio_with_cnt_pg5(p2->x);
        #endif
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_2T_D_2R)
    {

        // pkt1
        // if (g_ms_need_report[(ms_idx - 1) & 3] == 0)
        // {
        //     tmemcpy(&mouse_rx[(ms_idx - 1) & 3].btn, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN);
        //     g_ms_need_report[(ms_idx - 1) & 3] = 1;
        // }
        // pkt2
        if (g_ms_need_report[(ms_idx) & 3] == 0)
        {
            // tmemcpy(&mouse_rx[(ms_idx) & 3].x, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN - 1);
            mouse_rx[(ms_idx) & 3].x = (int16_t)p->buf[0];
            mouse_rx[(ms_idx) & 3].y = (int16_t)p->buf[1];
            g_ms_need_report[(ms_idx) & 3] = 1;
        }

        #if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
         ms_xy_s_t *p0 = (ms_xy_s_t *)&(p->buf[0]);
         ms_xy_t *p1 = (ms_xy_t *)&(p->buf[2]);
         ms_xy_s_t *p2 = (ms_xy_s_t *)&(p->buf[7]);
        //  ms_xy_t *p3 = (ms_xy_t *)&(p->buf[19]);


        #if 0
        // ms_xy_t *p0 = (ms_xy_t *)&(p->buf[0]);
        // ms_xy_t *p1 = (ms_xy_t *)&(p->buf[4]);
        // ms_xy_t *p2 = (ms_xy_t *)&(p->buf[9]);

        
        if(p0->x > XY_D_TH || p0->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
        }
        if(p1->x > XY_D_TH || p1->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
        }
        if(p2->x > XY_D_TH || p2->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
        }

            if(nr < 8 && nr_s)
            {
                tlkapi_send_string_data(APP_LOG_EN, "2rx", &p->dma_len, p->rf_len + 5);
                ZW_TG(GPIO_PG7);
                nr++;
            }


        if(p0->y > XY_D_TH || p0->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
        }
        if(p1->y > XY_D_TH || p1->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
        }
        if(p2->y > XY_D_TH || p2->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
        }
            #endif


        #if  RF_PKT_IDX_DEBUG_EN
        debug_gpio_with_cnt_pg5(p0->x);
        debug_gpio_with_cnt_pg6(p1->x);
        

        debug_gpio_with_cnt_ph3(p2->x);
        // debug_gpio_with_cnt_ph4(p2->x);
        #endif
         if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_RCVD)
         {
            p1->x -= ll_ctx.last_acc.x;
            p1->y -= ll_ctx.last_acc.y;

            if (ll_ctx.ack_mask & 1)
            {
                ll_ctx.acc.x += p2->x;
                ll_ctx.acc.y += p2->y;
            }
            else
            {
                ll_ctx.acc.x += p1->x + p2->x;
                ll_ctx.acc.y += p1->y + p2->y;
            }

         }else if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_2_LOSS)
         {
            p1->x = p1->x - ll_ctx.last_acc.x - p0->x;
            p1->y = p1->y - ll_ctx.last_acc.y - p0->y;

            if (ll_ctx.ack_mask & 1)
            {
                ll_ctx.acc.x += p2->x;
                ll_ctx.acc.y += p2->y;
            }
            else
            {
                ll_ctx.acc.x += p2->x + p1->x + p0->x;
                ll_ctx.acc.y += p2->y + p1->y + p0->y;
            }
         }else if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_LOSS)
         {
            p1->x = p1->x - ll_ctx.last_acc.x - p0->x;
            p1->y = p1->y - ll_ctx.last_acc.y - p0->y;
                        
            if (ll_ctx.ack_mask & 1)
            {
                ll_ctx.acc.x += p2->x;
                ll_ctx.acc.y += p2->y;
            }
            else
            {
                ll_ctx.acc.x += p2->x + p1->x + p0->x;
                ll_ctx.acc.y += p2->y + p1->y + p0->y;
            }
         }

        #endif

        // pkt2

        if (g_ms_need_report[(ms_idx + 1) & 3] == 0)
        {
            tmemcpy(&mouse_rx[(ms_idx + 1) & 3].x, &p->buf[2], TPSLL_RX_MS_DATA_VALID_LEN - 1);
            // mouse_rx[(ms_idx + 1) & 3].x = (int16_t)p->buf[2];
            // mouse_rx[(ms_idx + 1) & 3].y = (int16_t)p->buf[3];
            g_ms_need_report[(ms_idx + 1) & 3] = 1;
        }
        // pkt3 2R: The third packet is always a new packet.
        // tmemcpy(&mouse_rx[(ms_idx + 2) & 3].btn, &p->buf[8], TPSLL_RX_MS_DATA_VALID_LEN);
        mouse_rx[(ms_idx + 2) & 3].btn = p->buf[6];
        mouse_rx[(ms_idx + 2) & 3].x = (int16_t)p->buf[7];
        mouse_rx[(ms_idx + 2) & 3].y = (int16_t)p->buf[8];
        g_ms_need_report[(ms_idx + 2) & 3] = 1;

        // int16_t a = (int16_t)p->buf[7];
        // mouse_rx[(ms_idx + 2) & 3].x = (int16_t)p->buf[7] - MAX_XY;

        // if(mouse_rx[(ms_idx + 2) & 3].x  == -2)
        // {
        //     ZW_TG(GPIO_PH0);
        // }else{
        //    ZW_TG(GPIO_PG6); 
        // }
        if (ll_ctx.report_mode == REPORT_MODE_NORMAL)
        {
            mouse_rx[0].btn = mouse_rx[1].btn = mouse_rx[2].btn = mouse_rx[3].btn = p->buf[6];
        }

        #if 0 //zw add for debug, not remove
        if ((p->buf[1] == 2 && p->buf[7] == 3 && p->buf[13] == 4 && p->buf[19] == 5) ||
            (p->buf[1] == 4 && p->buf[7] == 5 && p->buf[13] == 6 && p->buf[19] == 7) ||
            (p->buf[1] == 6 && p->buf[7] == 7 && p->buf[13] == 8 && p->buf[19] == 1) ||
            (p->buf[1] == 8 && p->buf[7] == 1 && p->buf[13] == 2 && p->buf[19] == 3))
        {
            is_valid = 1;
        }
        if (!is_valid)
        {
            // ZW_TG(GPIO_PH0);
            // tlkapi_send_string_data(APP_LOG_EN, "2R", &p->buf[1], 24);
        }
        #endif

        // if(ll_ctx.zw_cnt > 0 && ll_ctx.zw_cnt < 20)
        // {
        //     tlkapi_send_string_data(APP_LOG_EN, "2R", &p->buf[1], 24);
        // }
        // debug_gpio_with_cnt_ph0(ms_idx + 1);
        #if  RF_PKT_IDX_DEBUG_EN && TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        debug_gpio_with_cnt_ph0(ms_idx + 1);
        debug_gpio_with_cnt_pg6(ll_ctx.last_acc.x);

        ZW_TG(GPIO_PG4);

        debug_gpio_with_cnt_pg6(ll_ctx.acc.x);
        debug_gpio_with_cnt_pg5(p1->x);
        #endif
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
    {


        // return;
#if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        ms_xy_t *p0 = (ms_xy_t *)&(p->buf[0]);
        ms_xy_s_t *p1 = (ms_xy_s_t *)&(p->buf[5]);

        #if 0
        // ms_xy_t *p0 = (ms_xy_t *)&(p->buf[0]);
        // ms_xy_t *p1 = (ms_xy_t *)&(p->buf[5]);
        
        if(p0->x > XY_D_TH || p0->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
        }
        if(p1->x > XY_D_TH || p1->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
        }

        if(p0->y > XY_D_TH || p0->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
        }
        if(p1->y > XY_D_TH || p1->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
        }
        #endif

        #if  RF_PKT_IDX_DEBUG_EN
        debug_gpio_with_cnt_pg5(p0->x);
        // debug_gpio_with_cnt6(ll_ctx.ack_mask);
        #endif

        p0->x -= ll_ctx.last_acc.x;
        p0->y -= ll_ctx.last_acc.y;
        if ((ll_ctx.ack_mask & 3) == 0)
        {
            ll_ctx.acc.x += p1->x + p0->x;
            ll_ctx.acc.y += p1->y + p0->y;
        }
        else if ((ll_ctx.ack_mask & 3) == 1)
        {
            ll_ctx.acc.x += p1->x;
            ll_ctx.acc.y += p1->y;
        }

#endif
        // pkt1
        if (g_ms_need_report[(ms_idx + 1) & 3] == 0 || mouse_rx[(ms_idx + 1) & 3].btn)
        {
        // if(ms3_cnt >= 1 && ms3_cnt < 3)
        // {

        //     tlkapi_send_string_data(APP_LOG_EN, "cpb1", &mouse_rx, sizeof(mouse_rx));
        // }
            tmemcpy(&mouse_rx[(ms_idx + 1) & 3].x, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN - 1);
            // mouse_rx[(ms_idx + 1) & 3].x = (int16_t)p->buf[0];
            // mouse_rx[(ms_idx + 1) & 3].y = (int16_t)p->buf[1];
            g_ms_need_report[(ms_idx + 1) & 3] = 1;
        }
        // pk2
        // tmemcpy(&mouse_rx[(ms_idx + 2) & 3].btn, &p->buf[6], 6);
        if (g_ms_need_report[(ms_idx + 2) & 3] == 0 || p->buf[4])
        {
        // if(ms3_cnt >= 1 && ms3_cnt < 3)
        // {

        //     tlkapi_send_string_data(APP_LOG_EN, "cpb2", &mouse_rx, sizeof(mouse_rx));
        // }
            // tmemcpy(&mouse_rx[(ms_idx + 2) & 3].btn, &p->buf[4], TPSLL_RX_MS_DATA_VALID_LEN);
             mouse_rx[(ms_idx + 2) & 3].btn = p->buf[4];
             mouse_rx[(ms_idx + 2) & 3].x = (int16_t)p->buf[5];
            mouse_rx[(ms_idx + 2) & 3].y = (int16_t)p->buf[6];
            g_ms_need_report[(ms_idx + 2) & 3] = 1;
        }
        if (ll_ctx.report_mode == REPORT_MODE_NORMAL)
        {
            mouse_rx[0].btn = mouse_rx[1].btn = mouse_rx[2].btn = mouse_rx[3].btn = p->buf[4];
        }
        if(ms3_cnt >= 1 && ms3_cnt < 3)
        {

            // tlkapi_send_string_data(APP_LOG_EN, "cpb", &mouse_rx, sizeof(mouse_rx));
        }

        #if 0 //zw add for debug, not remove
        if ((p->buf[1] == 2 && p->buf[7] == 3) ||
            (p->buf[1] == 4 && p->buf[7] == 5) ||
            (p->buf[1] == 6 && p->buf[7] == 7) ||
            (p->buf[1] == 8 && p->buf[7] == 1))
        {
            is_valid = 1;
        }
        if (!is_valid)
        {
            // ZW_TG(GPIO_PH0);
            // tlkapi_send_string_data(APP_LOG_EN, "3R", &p->buf[0], 12);
        }
        #endif

//        if(ll_ctx.zw_cnt > 0 && ll_ctx.zw_cnt < 20)
//        {
//            // tlkapi_send_string_data(APP_LOG_EN, "3R", &p->buf[0], 12);
//        }
        // debug_gpio_with_cnt_ph0(ms_idx + 1);
        #if  RF_PKT_IDX_DEBUG_EN && TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        debug_gpio_with_cnt_ph0(ms_idx + 1);
        debug_gpio_with_cnt_pg6(ll_ctx.last_acc.x);
        debug_gpio_with_cnt_pg5(p0->x);
        ZW_TG(GPIO_PG4);

        debug_gpio_with_cnt_pg6(ll_ctx.acc.x);
        debug_gpio_with_cnt_pg5(p1->x);
        
        #endif
    }
}


#else

_attribute_ram_code_sec_ static void store_ms_data2_buffer(rf_packet_t *p)
{
    #if 0 //zw add for debug, not remove
    uint8_t  is_valid = 0;
    #endif

    if (ll_ctx.step_3t1r == STEP_3T1R_1T_D_1R)
    {

        // pkt1
        if (g_ms_need_report[(ms_idx - 1) & 3] == 0)
        {
            tmemcpy(&mouse_rx[(ms_idx - 1) & 3].x, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN - 1);
            g_ms_need_report[(ms_idx - 1) & 3] = 1;
        }
  
       

        if (g_ms_need_report[(ms_idx) & 3] == 0)
        {
            tmemcpy(&mouse_rx[(ms_idx) & 3].x, &p->buf[4], TPSLL_RX_MS_DATA_VALID_LEN - 1);
            g_ms_need_report[(ms_idx) & 3] = 1;
        }

         #if  TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        ms_xy_t *p0 = (ms_xy_t *)&(p->buf[0]);
        ms_xy_t *p1 = (ms_xy_t *)&(p->buf[4]);
        ms_xy_t *p2 = (ms_xy_t *)&(p->buf[9]);


        #if 0

        // ms_xy_t *p0 = (ms_xy_t *)&(p->buf[0]);
        // ms_xy_t *p1 = (ms_xy_t *)&(p->buf[4]);
        // ms_xy_t *p2 = (ms_xy_t *)&(p->buf[9]);

        
        if(p0->x > XY_D_TH || p0->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
        }
        if(p1->x > XY_D_TH || p1->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
            if(nr < 8)
            {
                tlkapi_send_string_data(APP_LOG_EN, "1rx", &p->dma_len, p->rf_len + 5);
                ZW_TG(GPIO_PG7);
                nr_s = 1;
                nr++;
            }
        }
        if(p2->x > XY_D_TH || p2->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
        }


        if(p0->y > XY_D_TH || p0->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
        }
        if(p1->y > XY_D_TH || p1->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
        }
        if(p2->y > XY_D_TH || p2->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
        }
        #endif


        #if  RF_PKT_IDX_DEBUG_EN
        debug_gpio_with_cnt_ph0(p0->x)
        debug_gpio_with_cnt_pg5(p1->x);
        debug_gpio_with_cnt_pg6(p2->x);
        
        #endif
        //  debug_gpio_with_cnt_pg5(p1->x);

        if (ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_RCVD)
        {
            p2->x -= ll_ctx.last_acc.x;
            p2->y -= ll_ctx.last_acc.y;
            ll_ctx.acc.x += p2->x;
            ll_ctx.acc.y += p2->y;
        }
        else if (ll_ctx.last_intv_pkt_st == TPSLL_PKT_2_LOSS)
        {
            p2->x = p2->x - ll_ctx.last_acc.x - p1->x;
            p2->y = p2->y - ll_ctx.last_acc.y - p1->y;
            ll_ctx.acc.x += p2->x + p1->x;
            ll_ctx.acc.y += p2->y + p1->y;
        }
        else if (ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_LOSS)
        {
            p2->x = p2->x - ll_ctx.last_acc.x - p0->x - p1->x;
            p2->y = p2->y - ll_ctx.last_acc.y - p0->y - p1->y;
            ll_ctx.acc.x += p2->x + p1->x + p0->x;
            ll_ctx.acc.y += p2->y + p1->y + p0->y;
        }

        #endif

        // pkt3  1R: The 3rd packet is always a new packet.
        tmemcpy(&mouse_rx[(ms_idx + 1) & 3].btn, &p->buf[8], TPSLL_RX_MS_DATA_VALID_LEN);
        g_ms_need_report[(ms_idx + 1) & 3] = 1;

        if (ll_ctx.report_mode == REPORT_MODE_NORMAL)
        {
            mouse_rx[0].btn = mouse_rx[1].btn = mouse_rx[2].btn = mouse_rx[3].btn = p->buf[8];
        }
        #if 0 //zw add for debug, not remove
        if ((p->buf[1] == 1 && p->buf[7] == 2) ||
            (p->buf[1] == 3 && p->buf[7] == 4) ||
            (p->buf[1] == 5 && p->buf[7] == 6) ||
            (p->buf[1] == 7 && p->buf[7] == 8))
        {
            is_valid = 1;
        }
        if (!is_valid)
        {
            // ZW_TG(GPIO_PH0);
            // tlkapi_send_string_data(APP_LOG_EN, "1R", &p->buf[0], 12);
        }
        #endif

        // debug_gpio_with_cnt_ph0(ms_idx + 1);

        #if  RF_PKT_IDX_DEBUG_EN && TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        // debug_gpio_with_cnt_ph0(ms_idx + 1);
        debug_gpio_with_cnt_pg6(ll_ctx.last_acc.x);
        ZW_TG(GPIO_PG4);
        
        debug_gpio_with_cnt_pg6(ll_ctx.acc.x);
        debug_gpio_with_cnt_pg5(p2->x);
        #endif
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_2T_D_2R)
    {


        // pkt1
        // if (g_ms_need_report[(ms_idx - 1) & 3] == 0)
        // {
        //     tmemcpy(&mouse_rx[(ms_idx - 1) & 3].btn, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN);
        //     g_ms_need_report[(ms_idx - 1) & 3] = 1;
        // }
        // pkt2
        if (g_ms_need_report[(ms_idx) & 3] == 0)
        {
            tmemcpy(&mouse_rx[(ms_idx) & 3].x, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN - 1);
            g_ms_need_report[(ms_idx) & 3] = 1;
        }

        #if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
         ms_xy_t *p0 = (ms_xy_t *)&(p->buf[0]);
         ms_xy_t *p1 = (ms_xy_t *)&(p->buf[4]);
         ms_xy_t *p2 = (ms_xy_t *)&(p->buf[9]);
        //  ms_xy_t *p3 = (ms_xy_t *)&(p->buf[19]);


        #if 0
        // ms_xy_t *p0 = (ms_xy_t *)&(p->buf[0]);
        // ms_xy_t *p1 = (ms_xy_t *)&(p->buf[4]);
        // ms_xy_t *p2 = (ms_xy_t *)&(p->buf[9]);

        
        if(p0->x > XY_D_TH || p0->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
        }
        if(p1->x > XY_D_TH || p1->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
        }
        if(p2->x > XY_D_TH || p2->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
        }

            if(nr < 8 && nr_s)
            {
                tlkapi_send_string_data(APP_LOG_EN, "2rx", &p->dma_len, p->rf_len + 5);
                ZW_TG(GPIO_PG7);
                nr++;
            }


        if(p0->y > XY_D_TH || p0->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
        }
        if(p1->y > XY_D_TH || p1->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
        }
        if(p2->y > XY_D_TH || p2->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
        }
            #endif


        #if  RF_PKT_IDX_DEBUG_EN
        debug_gpio_with_cnt_pg5(p0->x);
        debug_gpio_with_cnt_pg6(p1->x);
        

        debug_gpio_with_cnt_ph3(p2->x);
        // debug_gpio_with_cnt_ph4(p2->x);
        #endif
         if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_RCVD)
         {
            p1->x -= ll_ctx.last_acc.x;
            p1->y -= ll_ctx.last_acc.y;

            if (ll_ctx.ack_mask & 1)
            {
                ll_ctx.acc.x += p2->x;
                ll_ctx.acc.y += p2->y;
            }
            else
            {
                ll_ctx.acc.x += p1->x + p2->x;
                ll_ctx.acc.y += p1->y + p2->y;
            }

         }else if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_2_LOSS)
         {
            p1->x = p1->x - ll_ctx.last_acc.x - p0->x;
            p1->y = p1->y - ll_ctx.last_acc.y - p0->y;

            if (ll_ctx.ack_mask & 1)
            {
                ll_ctx.acc.x += p2->x;
                ll_ctx.acc.y += p2->y;
            }
            else
            {
                ll_ctx.acc.x += p2->x + p1->x + p0->x;
                ll_ctx.acc.y += p2->y + p1->y + p0->y;
            }
         }else if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_LOSS)
         {
            p1->x = p1->x - ll_ctx.last_acc.x - p0->x;
            p1->y = p1->y - ll_ctx.last_acc.y - p0->y;
                        
            if (ll_ctx.ack_mask & 1)
            {
                ll_ctx.acc.x += p2->x;
                ll_ctx.acc.y += p2->y;
            }
            else
            {
                ll_ctx.acc.x += p2->x + p1->x + p0->x;
                ll_ctx.acc.y += p2->y + p1->y + p0->y;
            }
         }

        #endif

        // pkt2

        if (g_ms_need_report[(ms_idx + 1) & 3] == 0)
        {
            tmemcpy(&mouse_rx[(ms_idx + 1) & 3].x, &p->buf[4], TPSLL_RX_MS_DATA_VALID_LEN - 1);
            g_ms_need_report[(ms_idx + 1) & 3] = 1;
        }
        // pkt3 2R: The third packet is always a new packet.
        tmemcpy(&mouse_rx[(ms_idx + 2) & 3].btn, &p->buf[8], TPSLL_RX_MS_DATA_VALID_LEN);
        g_ms_need_report[(ms_idx + 2) & 3] = 1;
        if (ll_ctx.report_mode == REPORT_MODE_NORMAL)
        {
            mouse_rx[0].btn = mouse_rx[1].btn = mouse_rx[2].btn = mouse_rx[3].btn = p->buf[8];
        }

        #if 0 //zw add for debug, not remove
        if ((p->buf[1] == 2 && p->buf[7] == 3 && p->buf[13] == 4 && p->buf[19] == 5) ||
            (p->buf[1] == 4 && p->buf[7] == 5 && p->buf[13] == 6 && p->buf[19] == 7) ||
            (p->buf[1] == 6 && p->buf[7] == 7 && p->buf[13] == 8 && p->buf[19] == 1) ||
            (p->buf[1] == 8 && p->buf[7] == 1 && p->buf[13] == 2 && p->buf[19] == 3))
        {
            is_valid = 1;
        }
        if (!is_valid)
        {
            // ZW_TG(GPIO_PH0);
            // tlkapi_send_string_data(APP_LOG_EN, "2R", &p->buf[1], 24);
        }
        #endif

        // if(ll_ctx.zw_cnt > 0 && ll_ctx.zw_cnt < 20)
        // {
        //     tlkapi_send_string_data(APP_LOG_EN, "2R", &p->buf[1], 24);
        // }
        // debug_gpio_with_cnt_ph0(ms_idx + 1);
        #if  RF_PKT_IDX_DEBUG_EN && TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        debug_gpio_with_cnt_ph0(ms_idx + 1);
        debug_gpio_with_cnt_pg6(ll_ctx.last_acc.x);

        ZW_TG(GPIO_PG4);

        debug_gpio_with_cnt_pg6(ll_ctx.acc.x);
        debug_gpio_with_cnt_pg5(p1->x);
        #endif
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
    {


        // return;
#if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        ms_xy_t *p0 = (ms_xy_t *)&(p->buf[0]);
        ms_xy_t *p1 = (ms_xy_t *)&(p->buf[5]);

        #if 0
        // ms_xy_t *p0 = (ms_xy_t *)&(p->buf[0]);
        // ms_xy_t *p1 = (ms_xy_t *)&(p->buf[5]);
        
        if(p0->x > XY_D_TH || p0->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
        }
        if(p1->x > XY_D_TH || p1->x < -XY_D_TH)
        {
            ZW_TG(GPIO_PH0);
            ZW_TG(GPIO_PH0);
        }

        if(p0->y > XY_D_TH || p0->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
        }
        if(p1->y > XY_D_TH || p1->y < -XY_D_TH)
        {
            ZW_TG(GPIO_PG6);
            ZW_TG(GPIO_PG6);
        }
        #endif

        #if  RF_PKT_IDX_DEBUG_EN
        debug_gpio_with_cnt_pg5(p0->x);
        // debug_gpio_with_cnt6(ll_ctx.ack_mask);
        #endif

        p0->x -= ll_ctx.last_acc.x;
        p0->y -= ll_ctx.last_acc.y;
        if ((ll_ctx.ack_mask & 3) == 0)
        {
            ll_ctx.acc.x += p1->x + p0->x;
            ll_ctx.acc.y += p1->y + p0->y;
        }
        else if ((ll_ctx.ack_mask & 3) == 1)
        {
            ll_ctx.acc.x += p1->x;
            ll_ctx.acc.y += p1->y;
        }

#endif
        // pkt1
        if (g_ms_need_report[(ms_idx + 1) & 3] == 0 || mouse_rx[(ms_idx + 1) & 3].btn)
        {
        // if(ms3_cnt >= 1 && ms3_cnt < 3)
        // {

        //     tlkapi_send_string_data(APP_LOG_EN, "cpb1", &mouse_rx, sizeof(mouse_rx));
        // }
            tmemcpy(&mouse_rx[(ms_idx + 1) & 3].x, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN - 1);
            g_ms_need_report[(ms_idx + 1) & 3] = 1;
        }
        // pk2
        // tmemcpy(&mouse_rx[(ms_idx + 2) & 3].btn, &p->buf[6], 6);
        if (g_ms_need_report[(ms_idx + 2) & 3] == 0 || p->buf[4])
        {
        // if(ms3_cnt >= 1 && ms3_cnt < 3)
        // {

        //     tlkapi_send_string_data(APP_LOG_EN, "cpb2", &mouse_rx, sizeof(mouse_rx));
        // }
            tmemcpy(&mouse_rx[(ms_idx + 2) & 3].btn, &p->buf[4], TPSLL_RX_MS_DATA_VALID_LEN);
            g_ms_need_report[(ms_idx + 2) & 3] = 1;
        }
        if (ll_ctx.report_mode == REPORT_MODE_NORMAL)
        {
            mouse_rx[0].btn = mouse_rx[1].btn = mouse_rx[2].btn = mouse_rx[3].btn = p->buf[4];
        }
        if(ms3_cnt >= 1 && ms3_cnt < 3)
        {

            // tlkapi_send_string_data(APP_LOG_EN, "cpb", &mouse_rx, sizeof(mouse_rx));
        }

        #if 0 //zw add for debug, not remove
        if ((p->buf[1] == 2 && p->buf[7] == 3) ||
            (p->buf[1] == 4 && p->buf[7] == 5) ||
            (p->buf[1] == 6 && p->buf[7] == 7) ||
            (p->buf[1] == 8 && p->buf[7] == 1))
        {
            is_valid = 1;
        }
        if (!is_valid)
        {
            // ZW_TG(GPIO_PH0);
            // tlkapi_send_string_data(APP_LOG_EN, "3R", &p->buf[0], 12);
        }
        #endif

//        if(ll_ctx.zw_cnt > 0 && ll_ctx.zw_cnt < 20)
//        {
//            // tlkapi_send_string_data(APP_LOG_EN, "3R", &p->buf[0], 12);
//        }
        // debug_gpio_with_cnt_ph0(ms_idx + 1);
        #if  RF_PKT_IDX_DEBUG_EN && TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        debug_gpio_with_cnt_ph0(ms_idx + 1);
        debug_gpio_with_cnt_pg6(ll_ctx.last_acc.x);
        debug_gpio_with_cnt_pg5(p0->x);
        ZW_TG(GPIO_PG4);

        debug_gpio_with_cnt_pg6(ll_ctx.acc.x);
        debug_gpio_with_cnt_pg5(p1->x);
        
        #endif
    }
}
#endif//TPSLL_8K_COMPRESS_XY_DATA_EN


#else //TPSLL_NEW_8K_RF_PACKET_FORMAT_EN

_attribute_ram_code_sec_ static void store_ms_data2_buffer(rf_packet_t *p)
{
    #if 0 //zw add for debug, not remove
    uint8_t  is_valid = 0;
    #endif

    if (ll_ctx.step_3t1r == STEP_3T1R_1T_D_1R)
    {
        // pkt1
        if (g_ms_need_report[(ms_idx - 1) & 3] == 0)
        {
            tmemcpy(&mouse_rx[(ms_idx - 1) & 3].btn, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN);
            g_ms_need_report[(ms_idx - 1) & 3] = 1;
        }
  
       

        if (g_ms_need_report[(ms_idx) & 3] == 0)
        {
            tmemcpy(&mouse_rx[(ms_idx) & 3].btn, &p->buf[6], TPSLL_RX_MS_DATA_VALID_LEN);
            g_ms_need_report[(ms_idx) & 3] = 1;
        }

         #if  TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        ms_xy_t *p0 = (ms_xy_t *)&(p->buf[1]);
        ms_xy_t *p1 = (ms_xy_t *)&(p->buf[7]);
        ms_xy_t *p2 = (ms_xy_t *)&(p->buf[13]);

        #if  RF_PKT_IDX_DEBUG_EN
        debug_gpio_with_cnt_pg5(p2->x);
        #endif
        //  debug_gpio_with_cnt_pg5(p1->x);

        if (ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_RCVD)
        {
            p2->x -= ll_ctx.last_acc.x;
            p2->y -= ll_ctx.last_acc.y;
            ll_ctx.acc.x += p2->x;
            ll_ctx.acc.y += p2->y;
        }
        else if (ll_ctx.last_intv_pkt_st == TPSLL_PKT_2_LOSS)
        {
            p2->x = p2->x - ll_ctx.last_acc.x - p1->x;
            p2->y = p2->y - ll_ctx.last_acc.y - p1->y;
            ll_ctx.acc.x += p2->x + p1->x;
            ll_ctx.acc.y += p2->y + p1->y;
        }
        else if (ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_LOSS)
        {
            p2->x = p2->x - ll_ctx.last_acc.x - p0->x - p1->x;
            p2->y = p2->y - ll_ctx.last_acc.y - p0->y - p1->y;
            ll_ctx.acc.x += p2->x + p1->x + p0->x;
            ll_ctx.acc.y += p2->y + p1->y + p0->y;
        }

        #endif

        // pkt3  1R: The 3rd packet is always a new packet.
        tmemcpy(&mouse_rx[(ms_idx + 1) & 3].btn, &p->buf[12], TPSLL_RX_MS_DATA_VALID_LEN);
        g_ms_need_report[(ms_idx + 1) & 3] = 1;

        if (p->buf[12] && ll_ctx.report_mode == REPORT_MODE_NORMAL)
        {
            mouse_rx[0].btn = mouse_rx[1].btn = mouse_rx[2].btn = mouse_rx[3].btn = p->buf[12];
        }
        #if 0 //zw add for debug, not remove
        if ((p->buf[1] == 1 && p->buf[7] == 2) ||
            (p->buf[1] == 3 && p->buf[7] == 4) ||
            (p->buf[1] == 5 && p->buf[7] == 6) ||
            (p->buf[1] == 7 && p->buf[7] == 8))
        {
            is_valid = 1;
        }
        if (!is_valid)
        {
            // ZW_TG(GPIO_PH0);
            // tlkapi_send_string_data(APP_LOG_EN, "1R", &p->buf[0], 12);
        }
        #endif

        // debug_gpio_with_cnt_ph0(ms_idx + 1);

        #if  RF_PKT_IDX_DEBUG_EN && TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        // debug_gpio_with_cnt_ph0(ms_idx + 1);
        debug_gpio_with_cnt_pg6(ll_ctx.last_acc.x);
        ZW_TG(GPIO_PG4);
        
        debug_gpio_with_cnt_pg6(ll_ctx.acc.x);
        debug_gpio_with_cnt_pg5(p2->x);
        #endif
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_2T_D_2R)
    {
        // pkt1
        // if (g_ms_need_report[(ms_idx - 1) & 3] == 0)
        // {
        //     tmemcpy(&mouse_rx[(ms_idx - 1) & 3].btn, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN);
        //     g_ms_need_report[(ms_idx - 1) & 3] = 1;
        // }
        // pkt2
        if (g_ms_need_report[(ms_idx) & 3] == 0)
        {
            tmemcpy(&mouse_rx[(ms_idx) & 3].btn, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN);
            g_ms_need_report[(ms_idx) & 3] = 1;
        }

        #if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
         ms_xy_t *p0 = (ms_xy_t *)&(p->buf[1]);
         ms_xy_t *p1 = (ms_xy_t *)&(p->buf[7]);
         ms_xy_t *p2 = (ms_xy_t *)&(p->buf[13]);
        //  ms_xy_t *p3 = (ms_xy_t *)&(p->buf[19]);
        #if  RF_PKT_IDX_DEBUG_EN
        debug_gpio_with_cnt_pg5(p0->x);
        debug_gpio_with_cnt_pg6(p1->x);
        

        debug_gpio_with_cnt_ph3(p2->x);
        // debug_gpio_with_cnt_ph4(p2->x);
        #endif
         if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_RCVD)
         {
            p1->x -= ll_ctx.last_acc.x;
            p1->y -= ll_ctx.last_acc.y;

            if (ll_ctx.ack_mask & 1)
            {
                ll_ctx.acc.x += p2->x;
                ll_ctx.acc.y += p2->y;
            }
            else
            {
                ll_ctx.acc.x += p1->x + p2->x;
                ll_ctx.acc.y += p1->y + p2->y;
            }

         }else if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_2_LOSS)
         {
            p1->x = p1->x - ll_ctx.last_acc.x - p0->x;
            p1->y = p1->y - ll_ctx.last_acc.y - p0->y;

            if (ll_ctx.ack_mask & 1)
            {
                ll_ctx.acc.x += p2->x;
                ll_ctx.acc.y += p2->y;
            }
            else
            {
                ll_ctx.acc.x += p2->x + p1->x + p0->x;
                ll_ctx.acc.y += p2->y + p1->y + p0->y;
            }
         }else if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_LOSS)
         {
            p1->x = p1->x - ll_ctx.last_acc.x - p0->x;
            p1->y = p1->y - ll_ctx.last_acc.y - p0->y;
                        
            if (ll_ctx.ack_mask & 1)
            {
                ll_ctx.acc.x += p2->x;
                ll_ctx.acc.y += p2->y;
            }
            else
            {
                ll_ctx.acc.x += p2->x + p1->x + p0->x;
                ll_ctx.acc.y += p2->y + p1->y + p0->y;
            }
         }

        #endif

        // pkt2

        if (g_ms_need_report[(ms_idx + 1) & 3] == 0)
        {
            tmemcpy(&mouse_rx[(ms_idx + 1) & 3].btn, &p->buf[6], TPSLL_RX_MS_DATA_VALID_LEN);
            g_ms_need_report[(ms_idx + 1) & 3] = 1;
        }
        // pkt3 2R: The third packet is always a new packet.
        tmemcpy(&mouse_rx[(ms_idx + 2) & 3].btn, &p->buf[12], TPSLL_RX_MS_DATA_VALID_LEN);
        g_ms_need_report[(ms_idx + 2) & 3] = 1;
        if (p->buf[12] && ll_ctx.report_mode == REPORT_MODE_NORMAL)
        {
            mouse_rx[0].btn = mouse_rx[1].btn = mouse_rx[2].btn = mouse_rx[3].btn = p->buf[12];
        }

        #if 0 //zw add for debug, not remove
        if ((p->buf[1] == 2 && p->buf[7] == 3 && p->buf[13] == 4 && p->buf[19] == 5) ||
            (p->buf[1] == 4 && p->buf[7] == 5 && p->buf[13] == 6 && p->buf[19] == 7) ||
            (p->buf[1] == 6 && p->buf[7] == 7 && p->buf[13] == 8 && p->buf[19] == 1) ||
            (p->buf[1] == 8 && p->buf[7] == 1 && p->buf[13] == 2 && p->buf[19] == 3))
        {
            is_valid = 1;
        }
        if (!is_valid)
        {
            // ZW_TG(GPIO_PH0);
            // tlkapi_send_string_data(APP_LOG_EN, "2R", &p->buf[1], 24);
        }
        #endif

        // if(ll_ctx.zw_cnt > 0 && ll_ctx.zw_cnt < 20)
        // {
        //     tlkapi_send_string_data(APP_LOG_EN, "2R", &p->buf[1], 24);
        // }
        // debug_gpio_with_cnt_ph0(ms_idx + 1);
        #if  RF_PKT_IDX_DEBUG_EN && TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        debug_gpio_with_cnt_ph0(ms_idx + 1);
        debug_gpio_with_cnt_pg6(ll_ctx.last_acc.x);

        ZW_TG(GPIO_PG4);

        debug_gpio_with_cnt_pg6(ll_ctx.acc.x);
        debug_gpio_with_cnt_pg5(p1->x);
        #endif
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
    {

        // return;
#if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        ms_xy_t *p0 = (ms_xy_t *)&(p->buf[1]);
        ms_xy_t *p1 = (ms_xy_t *)&(p->buf[7]);

        #if  RF_PKT_IDX_DEBUG_EN
        debug_gpio_with_cnt_pg5(p0->x);
        // debug_gpio_with_cnt6(ll_ctx.ack_mask);
        #endif

        p0->x -= ll_ctx.last_acc.x;
        p0->y -= ll_ctx.last_acc.y;
        if ((ll_ctx.ack_mask & 3) == 0)
        {
            ll_ctx.acc.x += p1->x + p0->x;
            ll_ctx.acc.y += p1->y + p0->y;
        }
        else if ((ll_ctx.ack_mask & 3) == 1)
        {
            ll_ctx.acc.x += p1->x;
            ll_ctx.acc.y += p1->y;
        }

#endif
        // pkt1
        if (g_ms_need_report[(ms_idx + 1) & 3] == 0 || mouse_rx[(ms_idx + 1) & 3].btn)
        {
        // if(ms3_cnt >= 1 && ms3_cnt < 3)
        // {

        //     tlkapi_send_string_data(APP_LOG_EN, "cpb1", &mouse_rx, sizeof(mouse_rx));
        // }
            tmemcpy(&mouse_rx[(ms_idx + 1) & 3].btn, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN);
            g_ms_need_report[(ms_idx + 1) & 3] = 1;
        }
        // pk2
        // tmemcpy(&mouse_rx[(ms_idx + 2) & 3].btn, &p->buf[6], 6);
        if (g_ms_need_report[(ms_idx + 2) & 3] == 0 || p->buf[6])
        {
        // if(ms3_cnt >= 1 && ms3_cnt < 3)
        // {

        //     tlkapi_send_string_data(APP_LOG_EN, "cpb2", &mouse_rx, sizeof(mouse_rx));
        // }
            tmemcpy(&mouse_rx[(ms_idx + 2) & 3].btn, &p->buf[6], TPSLL_RX_MS_DATA_VALID_LEN);
            g_ms_need_report[(ms_idx + 2) & 3] = 1;
        }
        if (p->buf[6] && ll_ctx.report_mode == REPORT_MODE_NORMAL)
        {
            mouse_rx[0].btn = mouse_rx[1].btn = mouse_rx[2].btn = mouse_rx[3].btn = p->buf[6];
        }
        if(ms3_cnt >= 1 && ms3_cnt < 3)
        {

            // tlkapi_send_string_data(APP_LOG_EN, "cpb", &mouse_rx, sizeof(mouse_rx));
        }

        #if 0 //zw add for debug, not remove
        if ((p->buf[1] == 2 && p->buf[7] == 3) ||
            (p->buf[1] == 4 && p->buf[7] == 5) ||
            (p->buf[1] == 6 && p->buf[7] == 7) ||
            (p->buf[1] == 8 && p->buf[7] == 1))
        {
            is_valid = 1;
        }
        if (!is_valid)
        {
            // ZW_TG(GPIO_PH0);
            // tlkapi_send_string_data(APP_LOG_EN, "3R", &p->buf[0], 12);
        }
        #endif

//        if(ll_ctx.zw_cnt > 0 && ll_ctx.zw_cnt < 20)
//        {
//            // tlkapi_send_string_data(APP_LOG_EN, "3R", &p->buf[0], 12);
//        }
        // debug_gpio_with_cnt_ph0(ms_idx + 1);
        #if  RF_PKT_IDX_DEBUG_EN && TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        debug_gpio_with_cnt_ph0(ms_idx + 1);
        debug_gpio_with_cnt_pg6(ll_ctx.last_acc.x);
        debug_gpio_with_cnt_pg5(p0->x);
        ZW_TG(GPIO_PG4);

        debug_gpio_with_cnt_pg6(ll_ctx.acc.x);
        debug_gpio_with_cnt_pg5(p1->x);
        
        #endif
    }
}

#endif //TPSLL_NEW_8K_RF_PACKET_FORMAT_EN

#else
_attribute_ram_code_sec_ static void store_ms_data2_buffer(rf_packet_t *p)
{
   #if 0  
    if (auto_draw_flag < 100)
    {
        signed short draw_flag = p->buf[1] | (p->buf[2] << 8);
        if (draw_flag == 0x6098)
        {
            signed short auto_draw_len = p->buf[3] | (p->buf[4] << 8);
            {
                app_square_use_local_data(auto_draw_len);
            }
            auto_draw_flag = draw_flag;
        }else{
            auto_draw_flag++;
        }
        return;
    }
    #endif

    #if 0 //zw add for debug, not remove
    uint8_t  is_valid = 0;
    #endif

    if (ll_ctx.step_3t1r == STEP_3T1R_1T_D_1R)
    {
        // pkt1
        if (g_ms_need_report[(ms_idx) & 3] == 0)
        {
            tmemcpy(&mouse_rx[(ms_idx) & 3].btn, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN);
            g_ms_need_report[(ms_idx) & 3] = 1;
        }
  
        #if  TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        ms_xy_t *p0 = (ms_xy_t *)&(p->buf[1]);
        ms_xy_t *p1 = (ms_xy_t *)&(p->buf[7]);


#if 0
        #define XY_TH2 (200)
        if (p0->x > XY_TH2 || p0->x < -XY_TH2 || p0->y > XY_TH2 || p0->y < -XY_TH2)
        {
            ZW_TG(GPIO_PG6);
        }
        if (p1->x > XY_TH2 || p1->x < -XY_TH2 || p1->y > XY_TH2 || p1->y < -XY_TH2)
        {
            ZW_TG(GPIO_PG6);
        }
        #endif

#if RF_PKT_IDX_DEBUG_EN
        debug_gpio_with_cnt_pg5(p1->x);
#endif
        //  debug_gpio_with_cnt_pg5(p1->x);
  
         if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_RCVD)
         {
            p1->x -= ll_ctx.last_acc.x;
            p1->y -= ll_ctx.last_acc.y;
            ll_ctx.acc.x += p1->x;
            ll_ctx.acc.y += p1->y;

         }else if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_2_LOSS)
         {
            p1->x = p1->x - ll_ctx.last_acc.x - p0->x;
            p1->y = p1->y - ll_ctx.last_acc.y - p0->y;
            ll_ctx.acc.x += p1->x + p0->x;
            ll_ctx.acc.y += p1->y + p0->y;

         }else if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_LOSS)
         {
            p1->x = p1->x - ll_ctx.last_acc.x - p0->x;
            p1->y = p1->y - ll_ctx.last_acc.y - p0->y;
            // p1->x -= p0->x;
            // p1->y -= p0->y;
            ll_ctx.acc.x += p1->x + p0->x;
            ll_ctx.acc.y += p1->y + p0->y;

         }
         #endif

        // pkt2  1R: The second packet is always a new packet.
        tmemcpy(&mouse_rx[(ms_idx + 1) & 3].btn, &p->buf[6], TPSLL_RX_MS_DATA_VALID_LEN);
        g_ms_need_report[(ms_idx + 1) & 3] = 1;

        if (ll_ctx.report_mode == REPORT_MODE_NORMAL)
        {
            mouse_rx[0].btn = mouse_rx[1].btn = mouse_rx[2].btn = mouse_rx[3].btn = p->buf[6];
        }
        #if 0 //zw add for debug, not remove
        if ((p->buf[1] == 1 && p->buf[7] == 2) ||
            (p->buf[1] == 3 && p->buf[7] == 4) ||
            (p->buf[1] == 5 && p->buf[7] == 6) ||
            (p->buf[1] == 7 && p->buf[7] == 8))
        {
            is_valid = 1;
        }
        if (!is_valid)
        {
            // ZW_TG(GPIO_PH0);
            // tlkapi_send_string_data(APP_LOG_EN, "1R", &p->buf[0], 12);
        }
        #endif

#if RF_PKT_IDX_DEBUG_EN && TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        // debug_gpio_with_cnt_ph0(ms_idx + 1);
        debug_gpio_with_cnt_pg6(ll_ctx.last_acc.x);
        ZW_TG(GPIO_PG4);
        
        debug_gpio_with_cnt_pg6(ll_ctx.acc.x);
        debug_gpio_with_cnt_pg5(p1->x);
#endif
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_2T_D_2R)
    {
        // pkt1
        if (g_ms_need_report[(ms_idx - 1) & 3] == 0)
        {
            tmemcpy(&mouse_rx[(ms_idx - 1) & 3].btn, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN);
            g_ms_need_report[(ms_idx - 1) & 3] = 1;
        }
        // pkt2
        if (g_ms_need_report[(ms_idx) & 3] == 0)
        {
            tmemcpy(&mouse_rx[(ms_idx) & 3].btn, &p->buf[6], TPSLL_RX_MS_DATA_VALID_LEN);
            g_ms_need_report[(ms_idx) & 3] = 1;
        }

        #if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
         ms_xy_t *p0 = (ms_xy_t *)&(p->buf[1]);
         ms_xy_t *p1 = (ms_xy_t *)&(p->buf[7]);
         ms_xy_t *p2 = (ms_xy_t *)&(p->buf[13]);
         ms_xy_t *p3 = (ms_xy_t *)&(p->buf[19]);

#if 0
        #define XY_TH2 (200)
        if (p0->x > XY_TH2 || p0->x < -XY_TH2 || p0->y > XY_TH2 || p0->y < -XY_TH2)
        {
            ZW_TG(GPIO_PG6);
        }
        if (p1->x > XY_TH2 || p1->x < -XY_TH2 || p1->y > XY_TH2 || p1->y < -XY_TH2)
        {
            ZW_TG(GPIO_PG6);
        }

        if (p2->x > XY_TH2 || p2->x < -XY_TH2 || p2->y > XY_TH2 || p2->y < -XY_TH2)
        {
            ZW_TG(GPIO_PG6);
        }
        if (p3->x > XY_TH2 || p3->x < -XY_TH2 || p3->y > XY_TH2 || p3->y < -XY_TH2)
        {
            ZW_TG(GPIO_PG6);
        }
        #endif
#if RF_PKT_IDX_DEBUG_EN
        debug_gpio_with_cnt_pg5(p2->x);
        

        debug_gpio_with_cnt_ph3(p0->x);
        // debug_gpio_with_cnt_ph4(p1->x);
#endif
         if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_RCVD)
         {
            p2->x -= ll_ctx.last_acc.x;
            p2->y -= ll_ctx.last_acc.y;

            if (ll_ctx.ack_mask & 1)
            {
                ll_ctx.acc.x += p3->x;
                ll_ctx.acc.y += p3->y;
            }
            else
            {
                ll_ctx.acc.x += p3->x + p2->x;
                ll_ctx.acc.y += p3->y + p2->y;
            }

         }else if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_2_LOSS)
         {
            p2->x = p2->x - ll_ctx.last_acc.x - p1->x;
            p2->y = p2->y - ll_ctx.last_acc.y - p1->y;

            if (ll_ctx.ack_mask & 1)
            {
                ll_ctx.acc.x += p3->x;
                ll_ctx.acc.y += p3->y;
            }
            else
            {
                ll_ctx.acc.x += p3->x + p2->x + p1->x;
                ll_ctx.acc.y += p3->y + p2->y + p1->y;
            }
         }else if(ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_LOSS)
         {
            p2->x = p2->x - ll_ctx.last_acc.x - p1->x - p0->x;
            p2->y = p2->y - ll_ctx.last_acc.y - p1->y - p0->y;
                        
            if (ll_ctx.ack_mask & 1)
            {
                ll_ctx.acc.x += p3->x;
                ll_ctx.acc.y += p3->y;
            }
            else
            {
                ll_ctx.acc.x += p3->x + p2->x + p1->x + p0->x;
                ll_ctx.acc.y += p3->y + p2->y + p1->y + p0->y;
            }
         }

        #endif

        // pkt3

         /** 
            Force update pkt3:
                * Because this packet is definitely real data. For example, if the entire previous 
                * interval all packets was lost, the correct data cannot be accurately restored in RX0.
         **/
        #if !TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        if (g_ms_need_report[(ms_idx + 1) & 3] == 0)
        #endif
        {
            tmemcpy(&mouse_rx[(ms_idx + 1) & 3].btn, &p->buf[12], TPSLL_RX_MS_DATA_VALID_LEN);
            g_ms_need_report[(ms_idx + 1) & 3] = 1;
        }
        // pkt4 2R: The fourth packet is always a new packet.
        tmemcpy(&mouse_rx[(ms_idx + 2) & 3].btn, &p->buf[18], TPSLL_RX_MS_DATA_VALID_LEN);
        g_ms_need_report[(ms_idx + 2) & 3] = 1;
        if (ll_ctx.report_mode == REPORT_MODE_NORMAL)
        {
            mouse_rx[0].btn = mouse_rx[1].btn = mouse_rx[2].btn = mouse_rx[3].btn = p->buf[18];
        }

        #if 0 //zw add for debug, not remove
        if ((p->buf[1] == 2 && p->buf[7] == 3 && p->buf[13] == 4 && p->buf[19] == 5) ||
            (p->buf[1] == 4 && p->buf[7] == 5 && p->buf[13] == 6 && p->buf[19] == 7) ||
            (p->buf[1] == 6 && p->buf[7] == 7 && p->buf[13] == 8 && p->buf[19] == 1) ||
            (p->buf[1] == 8 && p->buf[7] == 1 && p->buf[13] == 2 && p->buf[19] == 3))
        {
            is_valid = 1;
        }
        if (!is_valid)
        {
            // ZW_TG(GPIO_PH0);
            // tlkapi_send_string_data(APP_LOG_EN, "2R", &p->buf[1], 24);
        }
        #endif

        // if(ll_ctx.zw_cnt > 0 && ll_ctx.zw_cnt < 20)
        // {
        //     tlkapi_send_string_data(APP_LOG_EN, "2R", &p->buf[1], 24);
        // }
#if RF_PKT_IDX_DEBUG_EN && TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        debug_gpio_with_cnt_ph0(ms_idx + 1);
        debug_gpio_with_cnt_pg6(ll_ctx.last_acc.x);

        ZW_TG(GPIO_PG4);

        debug_gpio_with_cnt_pg6(ll_ctx.acc.x);
        debug_gpio_with_cnt_pg5(p2->x);
#endif
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
    {
        // return;
#if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        ms_xy_t *p0 = (ms_xy_t *)&(p->buf[1]);
        ms_xy_t *p1 = (ms_xy_t *)&(p->buf[7]);

#if 0
        #define XY_TH2 (200)
        if (p0->x > XY_TH2 || p0->x < -XY_TH2 || p0->y > XY_TH2 || p0->y < -XY_TH2)
        {
            ZW_TG(GPIO_PG6);
        }
        if (p1->x > XY_TH2 || p1->x < -XY_TH2 || p1->y > XY_TH2 || p1->y < -XY_TH2)
        {
            ZW_TG(GPIO_PG6);
        }
        #endif

        #if  RF_PKT_IDX_DEBUG_EN
        debug_gpio_with_cnt_pg5(p0->x);
        // debug_gpio_with_cnt6(ll_ctx.ack_mask);
        #endif

        p0->x -= ll_ctx.last_acc.x;
        p0->y -= ll_ctx.last_acc.y;
        if ((ll_ctx.ack_mask & 3) == 0)
        {
            ll_ctx.acc.x += p1->x + p0->x;
            ll_ctx.acc.y += p1->y + p0->y;
        }
        else if ((ll_ctx.ack_mask & 3) == 1)
        {
            ll_ctx.acc.x += p1->x;
            ll_ctx.acc.y += p1->y;
        }

#endif
        // pkt1
        if (g_ms_need_report[(ms_idx + 1) & 3] == 0 || mouse_rx[(ms_idx + 1) & 3].btn)
        {
        // if(ms3_cnt >= 1 && ms3_cnt < 3)
        // {

        //     tlkapi_send_string_data(APP_LOG_EN, "cpb1", &mouse_rx, sizeof(mouse_rx));
        // }
            tmemcpy(&mouse_rx[(ms_idx + 1) & 3].btn, &p->buf[0], TPSLL_RX_MS_DATA_VALID_LEN);
            g_ms_need_report[(ms_idx + 1) & 3] = 1;
        }
        // pk2
        // tmemcpy(&mouse_rx[(ms_idx + 2) & 3].btn, &p->buf[6], 6);
        if (g_ms_need_report[(ms_idx + 2) & 3] == 0 || p->buf[6])
        {
        // if(ms3_cnt >= 1 && ms3_cnt < 3)
        // {

        //     tlkapi_send_string_data(APP_LOG_EN, "cpb2", &mouse_rx, sizeof(mouse_rx));
        // }
            tmemcpy(&mouse_rx[(ms_idx + 2) & 3].btn, &p->buf[6], TPSLL_RX_MS_DATA_VALID_LEN);
            g_ms_need_report[(ms_idx + 2) & 3] = 1;
        }
        if (ll_ctx.report_mode == REPORT_MODE_NORMAL)
        {
            mouse_rx[0].btn = mouse_rx[1].btn = mouse_rx[2].btn = mouse_rx[3].btn = p->buf[6];
        }
        if(ms3_cnt >= 1 && ms3_cnt < 3)
        {

            // tlkapi_send_string_data(APP_LOG_EN, "cpb", &mouse_rx, sizeof(mouse_rx));
        }

        #if 0 //zw add for debug, not remove
        if ((p->buf[1] == 2 && p->buf[7] == 3) ||
            (p->buf[1] == 4 && p->buf[7] == 5) ||
            (p->buf[1] == 6 && p->buf[7] == 7) ||
            (p->buf[1] == 8 && p->buf[7] == 1))
        {
            is_valid = 1;
        }
        if (!is_valid)
        {
            // ZW_TG(GPIO_PH0);
            // tlkapi_send_string_data(APP_LOG_EN, "3R", &p->buf[0], 12);
        }
        #endif

//        if(ll_ctx.zw_cnt > 0 && ll_ctx.zw_cnt < 20)
//        {
//            // tlkapi_send_string_data(APP_LOG_EN, "3R", &p->buf[0], 12);
//        }
        #if  RF_PKT_IDX_DEBUG_EN && TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        debug_gpio_with_cnt_ph0(ms_idx + 1);
        debug_gpio_with_cnt_pg6(ll_ctx.last_acc.x);
        debug_gpio_with_cnt_pg5(p0->x);
        ZW_TG(GPIO_PG4);

        debug_gpio_with_cnt_pg6(ll_ctx.acc.x);
        debug_gpio_with_cnt_pg5(p1->x);
        
        #endif
    }
}

#endif


_attribute_ram_code_sec_ void app_rf_8k_abnormal_handle(void)
{
    #if TPSLL_FHSS_EN
    // if (ll_ctx.stk_state == STATE_CONNECTED)
    {
        tpsll_channel_statistic(chn_list[ll_ctx.step_3t1r], true);
    }
    #endif
#if AFH_ENABLE
    // if (ll_ctx.stk_state == STATE_CONNECTED)
    {
        update_channel_quality(chn_list[ll_ctx.step_3t1r], false);
    }
#endif

    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX7]);
    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);

    if (ll_ctx.step_3t1r == STEP_3T1R_1T_D_1R)
    {
        // dongle 1R timeout
        
        ll_ctx.step_3t1r = STEP_3T1R_2T_D_2R;
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_2T_D_2R)
    {
        #if TPSLL_AVOID_WIFI_CHN_USE_RX3_EN
        ll_ctx.is_2r_to = true;//Even if a CRC error occurs, it will have no impact.
        #endif
        ms_rp_idx = (ms_idx - 1) & 3;

    #if APP_RF_MOUSE_REPORT_EN
        // if(g_ms_need_report[(ms_idx + 1) & 3] == 1)
        // if (ll_ctx.report_mode == REPORT_MODE_NORMAL)
        // {
        //     report_mouse_data();
        // }

        // else
        #if (ABNORMAL_RETRY_FIX)
        if (last_data_type == P24G_PKT_TYPE_MS) {
        #endif
        report_mouse_data_try();
        #if (ABNORMAL_RETRY_FIX)
        }
        #endif
        // if (ll_ctx.report_mode == REPORT_MODE_FAST)
        // {
        //     ll_ctx.is_last_report = false;
        // }
    #endif

        ll_ctx.step_3t1r = STEP_3T1R_3T_D_3R;
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
    {
        ms_rp_idx = (ms_idx) & 3;
        #if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        if((ll_inf.paired_dev_type & DEVICE_TYPE_MS) && ((ll_ctx.ack_mask & 2) == 0))
        {
            rf_access_code_comm(~ll_ctx.conn_acc);
            ll_ctx.is_need_restore_acc = true;
        }
        #endif
        // dongle 3R timeout
 #if APP_RF_MOUSE_REPORT_EN
        // if(g_ms_need_report[(ms_idx + 1) & 3] == 1)
        // if (ll_ctx.report_mode == REPORT_MODE_NORMAL)
        // {
        //     report_mouse_data();
        // }

        // else

        #if (ABNORMAL_RETRY_FIX)
        if (last_data_type == P24G_PKT_TYPE_MS) {
        #endif
        report_mouse_data_try();
        #if (ABNORMAL_RETRY_FIX)
        }
        #endif
        // if (ll_ctx.report_mode == REPORT_MODE_FAST)
        // {
        //     ll_ctx.is_last_report = false;
        // }
    #endif
        ll_ctx.step_3t1r = STEP_3T1R_1R_D_1T;
    }

    // if ((ll_ctx.stk_state == STATE_CONNECTED) && !ll_ctx.lost_cnt)
    if (!ll_ctx.lost_cnt)
    {
        ll_ctx.lost_cnt = 1;
    }
}

_attribute_ram_code_sec_ void app_rf_n8k_abnormal_handle(void)
{
#if AFH_ENABLE
    if (ll_ctx.stk_state == STATE_CONNECTED)
    {
        update_channel_quality(chn_list[ll_ctx.step_3t1r], false);
    }
#endif

    DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX7]);

    #if TPSLL_N8K_FHSS_EN
    if((ll_ctx.task_slot == TASK_SLOT0) || (ll_ctx.task_slot == TASK_SLOT1 && \
                                (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250)) && !ll_ctx.is_slot0_rx_ok))
    {
        // ZW_TG(GPIO_PH0);
        // if(ll_ctx.slot0_chn == chn_table[(ll_ctx.task_slot + ll_ctx.fno_last) & 3])
        // {
        //     ZW_TG(GPIO_PG6);
        // }
        tpsll_channel_statistic(ll_ctx.slot0_chn, true);
    }
    #endif

    if ((ll_ctx.stk_state == STATE_CONNECTED) && !ll_ctx.lost_cnt)
    {
        ll_ctx.lost_cnt = 1;
    }
}

#if RSSI_SCAN_EN

void rf_rssi_scan_start(void)
{
    tmemset(rssi_result, 0, sizeof(rssi_result));
    tmemset(scan_counts, 0, sizeof(scan_counts));
    #if TPSLL_NEW_RECON_EN
    tmemset(g_chn_rssi_max, 0, sizeof(g_chn_rssi_max));
    #endif

    current_channel = 0;
    current_round = 0;
    scan_state = STATE_SCAN_RUNNING;

    #if !TPSLL_NEW_RECON_EN
    while (!is_channel_valid(current_channel))
    {
        current_channel++;
    }
    #endif

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
    tlkapi_send_string_data(APP_LOG_EN, "chn idx rssi sort(s->b)", channel_idx, 40);
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

#if !TPSLL_RF_SCAN_EN
static void rf_rssi_scan_param_restore(void)
{
    #if TPSLL_NEW_RECON_EN
    rf_access_code_comm(ll_ctx.recon_acc);
    #else
    rf_access_code_comm(ll_ctx.pair_acc);
    #endif
    write_reg8(REG_RXCHN_EN, 1);
    write_reg8(0x17051a,36);//sync thr
    rf_set_irq_mask(FLD_RF_IRQ_TX|FLD_RF_IRQ_RX|FLD_RF_IRQ_RX_TIMEOUT|FLD_RF_IRQ_FIRST_TIMEOUT);
}
#endif

#if TPSLL_NEW_RECON_EN
// static void tpsll_bubble_sort_index(uint8_t *value, uint8_t num, uint8_t *sorted_idx) {
//     for (uint8_t i = 0; i < num; i++) {
//         sorted_idx[i] = i;
//     }

//     for (uint8_t i = 0; i < num - 1; i++) {
//         for (uint8_t j = 0; j < num - 1 - i; j++) {
//             if (value[sorted_idx[j]] > value[sorted_idx[j + 1]]) {
//                 uint8_t temp = sorted_idx[j];
//                 sorted_idx[j] = sorted_idx[j + 1];
//                 sorted_idx[j + 1] = temp;
//             }
//         }
//     }
// }
#endif

#if TPSLL_RF_SCAN_EN
static void tpsll_rf_scan_reset(void)
{
    // #if TPSLL_NEW_RECON_EN
    // tmemset(rf_scan_ctx.chn_rssi_max, 0, sizeof(rf_scan_ctx.chn_rssi_max));
    // #endif

    rf_scan_ctx.cur_chn = 0;
    rf_scan_ctx.cur_round = 0;
    rf_scan_ctx.temp_switch_chn = 0xff;
}

static void tpsll_rf_scan_start(void)
{
    rf_rssi_scan_param_set();

    tpsll_rf_scan_reset();
    CLR_REG_RSSI_MAX_BB();
    rf_set_ble_chn(rf_scan_ctx.cur_chn);
    pp_rf_start_srx(stimer_get_tick(), TPSLL_RF_SCAN_WIN_US);
}

static void tpsll_rf_scan_cotinue(void)
{
    rf_rssi_scan_param_set();
    CLR_REG_RSSI_MAX_BB();
    rf_set_ble_chn(rf_scan_ctx.cur_chn);
    pp_rf_start_srx(stimer_get_tick(), TPSLL_RF_SCAN_WIN_US);
}

static void tpsll_rf_state_restore(void)
{
    if (ll_ctx.stk_state == STATE_CONNECTED) {
    rf_access_code_comm(ll_ctx.conn_acc);
    } else {
        #if TPSLL_NEW_RECON_EN
        rf_access_code_comm(ll_ctx.recon_acc);
        #else
        rf_access_code_comm(ll_ctx.pair_acc);
        #endif
    }
    write_reg8(REG_RXCHN_EN, 1);
    write_reg8(0x17051a,36);//sync thr
    rf_set_irq_mask(FLD_RF_IRQ_TX|FLD_RF_IRQ_RX|FLD_RF_IRQ_RX_TIMEOUT|FLD_RF_IRQ_FIRST_TIMEOUT);
}

_attribute_ram_code_sec_ static void tpsll_rssi_scan(void)
{
    uint8_t rssi = REG_RSSI_MAX_BB;

    #if TPSLL_NEW_RECON_EN
    if (rf_scan_ctx.chn_rssi_max[rf_scan_ctx.cur_chn] < rssi)
        rf_scan_ctx.chn_rssi_max[rf_scan_ctx.cur_chn] = rssi; //max rssi
    #endif

    CLR_REG_RSSI_MAX_BB();

    if (ll_ctx.is_keep_alive && clock_time_exceed(rf_scan_ctx.start_tick, (TPSLL_8K_KEEP_ALIVE_SYNC_IN_US - TPSLL_RF_SCAN_START_TIME_US) - (TPSLL_RF_SCAN_WIN_US + 5))) {
        tpsll_rf_state_restore();
        rf_scan_ctx.state = RF_SCAN_STATE_PAUSE;
        return;
    }

    rf_scan_ctx.cur_chn++;

    if (rf_scan_ctx.cur_chn >= TPSLL_RF_SCAN_CHN_TOTAL)
    {
        rf_scan_ctx.cur_chn = 0;
        rf_scan_ctx.cur_round++;

        if (rf_scan_ctx.cur_round >= TPSLL_RF_SCAN_ROUNDS)
        {
            tpsll_rf_state_restore();
            rf_scan_ctx.state = RF_SCAN_STATE_DONE;
// DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
            return;
        }
    }

    rf_set_ble_chn(rf_scan_ctx.cur_chn);
    pp_rf_start_srx(stimer_get_tick(), TPSLL_RF_SCAN_WIN_US);
}

#if TPSLL_WIFI_RSSI_SCORE_EN

#if TPSLL_WIFI_RSSI_SCORE_MAX_EN
_attribute_ram_code_sec_ static void tpsll_wifi_calculate_valid_chn_score_max(void)
{
    uint8_t intern_valid_chn_score[TPSLL_FHSS_CHN_MAX_NUM] = {0};
    // Initialize output array to zero
    // tmemset(wifi_chn_inf.valid_chn_score, 0, sizeof(wifi_chn_inf.valid_chn_score));

    // Iterate through the WiFi channel busyness ranking results
    // (Most busy WiFi channel is at rssi_sorted_idx[TPSLL_WIFI_CHANNEL_COUNT-1])
    for (int rank = 0; rank < TPSLL_WIFI_CHANNEL_COUNT; ++rank) {
        uint8_t wifi_chn_idx = wifi_chn_inf.rssi_sorted_idx[rank]; // Get the array index of the WiFi channel ranked (rank+1)th (0-based)

        if (wifi_chn_idx >= TPSLL_WIFI_CHANNEL_COUNT) {
            tlkapi_printf(APP_LOG_EN, "Warning: Invalid WiFi channel index %d found at rank %d\n", wifi_chn_idx, rank);
            continue;
        }

        uint8_t weight = (rank + 1) * TPSLL_WIFI_RSSI_SCORE_MUILT; 

        const p24g_chn_t* current_valid_table = &wifi_chn_inf.valid_chn[wifi_chn_idx];
        for (int i = 0; i < current_valid_table->num; ++i) {
            uint8_t candidate_chn = current_valid_table->table[i];

            if (candidate_chn > 39) {
                 tlkapi_printf(APP_LOG_EN, "Warning: Invalid BLE channel candidate index %d found in WiFi channel %d's table\n", candidate_chn, wifi_chn_idx + 1);
                 continue;
            }

            // Find this candidate channel in g_fhss_valid_chn_tab
            for(int j = 0; j < TPSLL_FHSS_CHN_MAX_NUM; j++){
                if(g_fhss_valid_chn_tab[j] == candidate_chn){
                    if(weight > intern_valid_chn_score[j]){
                        intern_valid_chn_score[j] = weight; // Take the maximum value
                    }
                    break;
                }
            }
        }
    }

    wifi_chn_inf.chn_score_valid = false;
    for (int v = 0; v < TPSLL_FHSS_CHN_MAX_NUM; v++) {
        wifi_chn_inf.valid_chn_score[v] = (wifi_chn_inf.valid_chn_score[v] * TPSLL_VALID_CHN_SCORE_HISTORY_N +
            intern_valid_chn_score[v] * TPSLL_VALID_CHN_SCORE_NEW_N) / (TPSLL_VALID_CHN_SCORE_HISTORY_N + TPSLL_VALID_CHN_SCORE_NEW_N);
    }
    wifi_chn_inf.chn_score_valid = true;
}
#endif

_attribute_ram_code_sec_ static void tpsll_wifi_calculate_valid_chn_score_weight_avg(void)
{
    uint16_t total_weighted_score[TPSLL_FHSS_CHN_MAX_NUM] = {0};
    uint8_t count_per_ble_chn[TPSLL_FHSS_CHN_MAX_NUM] = {0};
    uint8_t intern_valid_chn_score[TPSLL_FHSS_CHN_MAX_NUM] = {0};

    // tmemset(wifi_chn_inf.valid_chn_score, 0, sizeof(wifi_chn_inf.valid_chn_score));

    for (int rank = 0; rank < TPSLL_WIFI_CHANNEL_COUNT; ++rank) {
        uint8_t wifi_chn_idx = wifi_chn_inf.rssi_sorted_idx[rank];

        if (wifi_chn_idx >= TPSLL_WIFI_CHANNEL_COUNT) {
            tlkapi_printf(APP_LOG_EN, "Warning: Invalid WiFi channel index %d found at rank %d\n", wifi_chn_idx, rank);
            continue;
        }

        uint8_t weight = (rank + 1) * TPSLL_WIFI_RSSI_SCORE_MUILT;

        const p24g_chn_t* current_valid_table = &wifi_chn_inf.valid_chn[wifi_chn_idx];

        for (int i = 0; i < current_valid_table->num; ++i) {
            uint8_t candidate_chn = current_valid_table->table[i];

            if (candidate_chn > 39) {
                 tlkapi_printf(APP_LOG_EN, "Warning: Invalid BLE channel candidate index %d found in WiFi channel %d's table\n", candidate_chn, wifi_chn_idx + 1);
                 continue;
            }

            // Find this candidate channel in g_fhss_valid_chn_tab
            for(int j = 0; j < TPSLL_FHSS_CHN_MAX_NUM; j++){
                if(g_fhss_valid_chn_tab[j] == candidate_chn){
                    total_weighted_score[j] += weight;
                    count_per_ble_chn[j]++;
                    break; // Break inner loop after finding, to avoid duplicate checks
                }
            }
        }
    }

    // Calculate the final score as the weighted average
    for(int j = 0; j < TPSLL_FHSS_CHN_MAX_NUM; j++){
        if(count_per_ble_chn[j] > 0){ // Avoid division by zero
            // Use integer division to get an average score
            // wifi_chn_inf.valid_chn_score[j] = (uint16_t)(total_weighted_score[j] / count_per_ble_chn[j]);
            intern_valid_chn_score[j] = (uint16_t)(total_weighted_score[j] / count_per_ble_chn[j]);
        } else {
            // This should ideally not happen if all valid channels are covered by at least one WiFi channel's table
            // wifi_chn_inf.valid_chn_score[j] = 0;
            intern_valid_chn_score[j] = 0;
        }
    }

    wifi_chn_inf.chn_score_valid = false;
    for (int v = 0; v < TPSLL_FHSS_CHN_MAX_NUM; v++) {
        wifi_chn_inf.valid_chn_score[v] = (wifi_chn_inf.valid_chn_score[v] * TPSLL_VALID_CHN_SCORE_HISTORY_N +
            intern_valid_chn_score[v] * TPSLL_VALID_CHN_SCORE_NEW_N) / (TPSLL_VALID_CHN_SCORE_HISTORY_N + TPSLL_VALID_CHN_SCORE_NEW_N);
    }
    wifi_chn_inf.chn_score_valid = true;
}
#endif

static void tpsll_rssi_scan_check(void)
{
    if (rf_scan_ctx.state & RF_SCAN_STATE_DONE)
    {
        // #if APP_LOG_EN //for debug, keep it
        // uint8_t print_buf[600] = {0};
        // uint16_t print_idx = 0;
        // #endif

        uint8_t best_channels[TPSLL_RF_SCAN_CHN_TOTAL];

        // print_idx += sprintf(&print_buf[print_idx], "ble:");

        // for (int i = 0; i < TPSLL_RF_SCAN_CHN_TOTAL; i++) {
        //     // print_idx += sprintf(&print_buf[print_idx], "<%02d>%03d", best_channels[i], rf_scan_ctx.chn_rssi_max[best_channels[i]]);
        //     print_idx += sprintf(&print_buf[print_idx], "<%02d>%02d", i, rf_scan_ctx.chn_rssi_max[i]);
        // }
        // print_idx += sprintf(&print_buf[print_idx], "\n");

        // printf("%s", print_buf);

        #if TPSLL_AVOID_WIFI_CHN_EN
        wifi_chn_inf.rssi_sum[0] = (wifi_chn_inf.rssi_sum[0] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (rf_scan_ctx.chn_rssi_max[37] / 2 + rf_scan_ctx.chn_rssi_max[0] + rf_scan_ctx.chn_rssi_max[1] + rf_scan_ctx.chn_rssi_max[2] +
                            rf_scan_ctx.chn_rssi_max[3] + rf_scan_ctx.chn_rssi_max[4] + rf_scan_ctx.chn_rssi_max[5] + rf_scan_ctx.chn_rssi_max[6] +
                            rf_scan_ctx.chn_rssi_max[7] + rf_scan_ctx.chn_rssi_max[8] + rf_scan_ctx.chn_rssi_max[9] / 2) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

        wifi_chn_inf.rssi_sum[1] = (wifi_chn_inf.rssi_sum[1] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (rf_scan_ctx.chn_rssi_max[2] + rf_scan_ctx.chn_rssi_max[3] + rf_scan_ctx.chn_rssi_max[4] + rf_scan_ctx.chn_rssi_max[5] +
                            rf_scan_ctx.chn_rssi_max[6] + rf_scan_ctx.chn_rssi_max[7] + rf_scan_ctx.chn_rssi_max[8] + rf_scan_ctx.chn_rssi_max[9] +
                            rf_scan_ctx.chn_rssi_max[10] + rf_scan_ctx.chn_rssi_max[38]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

        wifi_chn_inf.rssi_sum[2] = (wifi_chn_inf.rssi_sum[2] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (rf_scan_ctx.chn_rssi_max[4] / 2 + rf_scan_ctx.chn_rssi_max[5] + rf_scan_ctx.chn_rssi_max[6] + rf_scan_ctx.chn_rssi_max[7] +
                            rf_scan_ctx.chn_rssi_max[8] + rf_scan_ctx.chn_rssi_max[9] + rf_scan_ctx.chn_rssi_max[10] + rf_scan_ctx.chn_rssi_max[38] +
                            rf_scan_ctx.chn_rssi_max[11] + rf_scan_ctx.chn_rssi_max[12] + rf_scan_ctx.chn_rssi_max[13] / 2) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

        wifi_chn_inf.rssi_sum[3] = (wifi_chn_inf.rssi_sum[3] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (rf_scan_ctx.chn_rssi_max[7] + rf_scan_ctx.chn_rssi_max[8] + rf_scan_ctx.chn_rssi_max[9] + rf_scan_ctx.chn_rssi_max[10] +
                            rf_scan_ctx.chn_rssi_max[38] + rf_scan_ctx.chn_rssi_max[11] + rf_scan_ctx.chn_rssi_max[12] + rf_scan_ctx.chn_rssi_max[13] +
                            rf_scan_ctx.chn_rssi_max[14] + rf_scan_ctx.chn_rssi_max[15]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

        wifi_chn_inf.rssi_sum[4] = (wifi_chn_inf.rssi_sum[4] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (rf_scan_ctx.chn_rssi_max[9] / 2 + rf_scan_ctx.chn_rssi_max[10] + rf_scan_ctx.chn_rssi_max[38] + rf_scan_ctx.chn_rssi_max[11] +
                            rf_scan_ctx.chn_rssi_max[12] + rf_scan_ctx.chn_rssi_max[13] + rf_scan_ctx.chn_rssi_max[14] + rf_scan_ctx.chn_rssi_max[15] +
                            rf_scan_ctx.chn_rssi_max[16] + rf_scan_ctx.chn_rssi_max[17] + rf_scan_ctx.chn_rssi_max[18] / 2) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

        wifi_chn_inf.rssi_sum[5] = (wifi_chn_inf.rssi_sum[5] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (rf_scan_ctx.chn_rssi_max[11] + rf_scan_ctx.chn_rssi_max[12] + rf_scan_ctx.chn_rssi_max[13] + rf_scan_ctx.chn_rssi_max[14] +
                            rf_scan_ctx.chn_rssi_max[15] + rf_scan_ctx.chn_rssi_max[16] + rf_scan_ctx.chn_rssi_max[17] + rf_scan_ctx.chn_rssi_max[18] +
                            rf_scan_ctx.chn_rssi_max[19] + rf_scan_ctx.chn_rssi_max[20]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

        wifi_chn_inf.rssi_sum[6] = (wifi_chn_inf.rssi_sum[6] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (rf_scan_ctx.chn_rssi_max[13] / 2 + rf_scan_ctx.chn_rssi_max[14] + rf_scan_ctx.chn_rssi_max[15] + rf_scan_ctx.chn_rssi_max[16] +
                            rf_scan_ctx.chn_rssi_max[17] + rf_scan_ctx.chn_rssi_max[18] + rf_scan_ctx.chn_rssi_max[19] + rf_scan_ctx.chn_rssi_max[20] +
                            rf_scan_ctx.chn_rssi_max[21] + rf_scan_ctx.chn_rssi_max[22] + rf_scan_ctx.chn_rssi_max[23] / 2) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

        wifi_chn_inf.rssi_sum[7] = (wifi_chn_inf.rssi_sum[7] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (rf_scan_ctx.chn_rssi_max[16] + rf_scan_ctx.chn_rssi_max[17] + rf_scan_ctx.chn_rssi_max[18] + rf_scan_ctx.chn_rssi_max[19] +
                            rf_scan_ctx.chn_rssi_max[20] + rf_scan_ctx.chn_rssi_max[21] + rf_scan_ctx.chn_rssi_max[22] + rf_scan_ctx.chn_rssi_max[23] +
                            rf_scan_ctx.chn_rssi_max[24] + rf_scan_ctx.chn_rssi_max[25]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

        wifi_chn_inf.rssi_sum[8] = (wifi_chn_inf.rssi_sum[8] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (rf_scan_ctx.chn_rssi_max[18] /2 + rf_scan_ctx.chn_rssi_max[19] + rf_scan_ctx.chn_rssi_max[20] + rf_scan_ctx.chn_rssi_max[21] +
                            rf_scan_ctx.chn_rssi_max[22] + rf_scan_ctx.chn_rssi_max[23] + rf_scan_ctx.chn_rssi_max[24] + rf_scan_ctx.chn_rssi_max[25] +
                            rf_scan_ctx.chn_rssi_max[26] + rf_scan_ctx.chn_rssi_max[27] + rf_scan_ctx.chn_rssi_max[28] / 2) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

        wifi_chn_inf.rssi_sum[9] = (wifi_chn_inf.rssi_sum[9] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (rf_scan_ctx.chn_rssi_max[21] + rf_scan_ctx.chn_rssi_max[22] + rf_scan_ctx.chn_rssi_max[23] + rf_scan_ctx.chn_rssi_max[24] +
                            rf_scan_ctx.chn_rssi_max[25] + rf_scan_ctx.chn_rssi_max[26] + rf_scan_ctx.chn_rssi_max[27] + rf_scan_ctx.chn_rssi_max[28] +
                            rf_scan_ctx.chn_rssi_max[29] + rf_scan_ctx.chn_rssi_max[30]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

        wifi_chn_inf.rssi_sum[10] = (wifi_chn_inf.rssi_sum[10] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (rf_scan_ctx.chn_rssi_max[23] / 2 + rf_scan_ctx.chn_rssi_max[24] + rf_scan_ctx.chn_rssi_max[25] + rf_scan_ctx.chn_rssi_max[26] +
                            rf_scan_ctx.chn_rssi_max[27] + rf_scan_ctx.chn_rssi_max[28] + rf_scan_ctx.chn_rssi_max[29] + rf_scan_ctx.chn_rssi_max[30] +
                            rf_scan_ctx.chn_rssi_max[31] + rf_scan_ctx.chn_rssi_max[32] + rf_scan_ctx.chn_rssi_max[33] / 2) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

        wifi_chn_inf.rssi_sum[11] = (wifi_chn_inf.rssi_sum[11] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (rf_scan_ctx.chn_rssi_max[26] + rf_scan_ctx.chn_rssi_max[27] + rf_scan_ctx.chn_rssi_max[28] + rf_scan_ctx.chn_rssi_max[29] +
                            rf_scan_ctx.chn_rssi_max[30] + rf_scan_ctx.chn_rssi_max[31] + rf_scan_ctx.chn_rssi_max[32] + rf_scan_ctx.chn_rssi_max[33] +
                            rf_scan_ctx.chn_rssi_max[34] + rf_scan_ctx.chn_rssi_max[35]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

        wifi_chn_inf.rssi_sum[12] = (wifi_chn_inf.rssi_sum[12] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (rf_scan_ctx.chn_rssi_max[28] + rf_scan_ctx.chn_rssi_max[29] + rf_scan_ctx.chn_rssi_max[30] + rf_scan_ctx.chn_rssi_max[31] +
                            rf_scan_ctx.chn_rssi_max[32] + rf_scan_ctx.chn_rssi_max[33] + rf_scan_ctx.chn_rssi_max[34] + rf_scan_ctx.chn_rssi_max[35] +
                            rf_scan_ctx.chn_rssi_max[36] + rf_scan_ctx.chn_rssi_max[39]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

        tpsll_bubble_sort_rssi_index(wifi_chn_inf.rssi_sum, TPSLL_WIFI_CHANNEL_COUNT, wifi_chn_inf.rssi_sorted_idx);

        #if TPSLL_WIFI_RSSI_SCORE_EN
        #if TPSLL_WIFI_RSSI_SCORE_MAX_EN
        tpsll_wifi_calculate_valid_chn_score_max();
        #else
        tpsll_wifi_calculate_valid_chn_score_weight_avg();
        #endif
        // tpsll_bubble_sort_index(wifi_chn_inf.valid_chn_score, TPSLL_FHSS_CHN_MAX_NUM, best_channels);
        #endif /* end of TPSLL_WIFI_RSSI_SCORE_EN */
        #endif /* end of TPSLL_AVOID_WIFI_CHN_EN */

        #if 0
        tlkapi_send_string_data(APP_LOG_EN, "best chns(H)", best_channels, RSSI_SCAN_TOP_CHANNEL_COUNT);
        #endif

        // tmemcpy(reconn_chn_table, best_channels, TPSLL_RECON_CHN_NUM);
        int k = 0;

        #if 1//!TPSLL_WIFI_RSSI_SCORE_EN
        if ((ll_ctx.stk_state == STATE_PAIRING) || ll_ctx.stk_state == STATE_RECONNECT) {
            tpsll_bubble_sort_rssi_index(rf_scan_ctx.chn_rssi_max, TPSLL_RF_SCAN_CHN_TOTAL, best_channels);
        }
        #endif

        #if 1//!TPSLL_WIFI_RSSI_SCORE_EN
        for (int i = 0; i < TPSLL_RF_SCAN_CHN_TOTAL && k < TPSLL_RF_SCAN_SELECT_CHN_NUM; i++) {
            #if TPSLL_NEW_PAIR_EN
            if (ll_ctx.stk_state == STATE_PAIRING) {
                for (int j = 0; j < TPSLL_PAIR_CHN_MAX_NUM; j++) {
                    if (best_channels[i] == g_pair_valid_chn_tab[j]) {
                        reconn_chn_table[k] = g_pair_valid_chn_tab[j];
                        k++;
                    }
                }
            } else
            #endif
            if (ll_ctx.stk_state == STATE_RECONNECT)
            {
                #if TPSLL_AVOID_WIFI_CHN_EN
//////////////////////////////////////////////////////////////////////////////
                for (int w = 0; w < TPSLL_WIFI_BEST_CHN_NUM; w++) {
                    for (int j = 0; j < wifi_chn_inf.valid_chn[wifi_chn_inf.rssi_sorted_idx[w]].num; j++) {
                        if (best_channels[i] == wifi_chn_inf.valid_chn[wifi_chn_inf.rssi_sorted_idx[w]].table[j]) {
                            #if !TPSLL_WIFI_RSSI_SCORE_EN
                            rf_scan_ctx.select_chn_table[k] = best_channels[i];
                            #endif
                            reconn_chn_table[k] = best_channels[i];
                            k++;
                            w = TPSLL_WIFI_BEST_CHN_NUM;
                            break;
                        }
                    }
                }
//////////////////////////////////////////////////////////////////////////////
                #else
                for (int j = 0; j < TPSLL_FHSS_CHN_MAX_NUM; j++) {
                    if (best_channels[i] == g_fhss_valid_chn_tab[j]) {
                        reconn_chn_table[k] = g_fhss_valid_chn_tab[j];
                        k++;
                        break;
                    }
                }
                #endif
            }
        }
        #else /* end of !TPSLL_WIFI_RSSI_SCORE_EN */
        if (ll_ctx.stk_state == STATE_PAIRING) {
            for (int i = 0; i < TPSLL_FHSS_CHN_MAX_NUM && k < TPSLL_RF_SCAN_SELECT_CHN_NUM; i++)
            {
                for (int j = 0; j < TPSLL_PAIR_CHN_MAX_NUM; j++) {
                    if (g_fhss_valid_chn_tab[best_channels[i]] == g_pair_valid_chn_tab[j]) {
                        reconn_chn_table[k] = g_pair_valid_chn_tab[j];
                        k++;
                    }
                }
            }
        } else if (ll_ctx.stk_state == STATE_RECONNECT) {
            for (int i = 0; i < TPSLL_RF_SCAN_SELECT_CHN_NUM; i++)
            {
                reconn_chn_table[i] = g_fhss_valid_chn_tab[best_channels[i]];
            }
        }
        #endif /* end of !TPSLL_WIFI_RSSI_SCORE_EN #else */

        #if TPSLL_NEW_RECON_EN
        tmemset(rf_scan_ctx.chn_rssi_max, 0, sizeof(rf_scan_ctx.chn_rssi_max));
        #endif
        rf_scan_ctx.temp_switch_chn = 0xff;

        rf_scan_ctx.done_tick = clock_time() | 1;
        rf_scan_ctx.select_chn_ready = 1;
        rf_scan_ctx.state = RF_SCAN_STATE_IDLE;

        #if TPSLL_AVOID_WIFI_CHN_EN
        // printf("wifi:");
        // for (int i = 0; i < TPSLL_WIFI_CHANNEL_COUNT; i++) {
        //     printf("[%d]%d", wifi_chn_inf.rssi_sorted_idx[i] + 1, wifi_chn_inf.rssi_sum[wifi_chn_inf.rssi_sorted_idx[i]]);
        //     // printf("[%d]", wifi_chn_inf.rssi_sorted_idx[i] + 1);
        //     // printf("<%d,%d>\n", i + 1, wifi_chn_inf.rssi_sum[i]);
        // }
        // printf("\n");

        // print_idx += sprintf(&print_buf[print_idx], "wifi:");

        // for (int i = 0; i < TPSLL_WIFI_CHANNEL_COUNT; i++) {
        //     // print_idx += sprintf(&print_buf[print_idx], "<%02d>%03d", best_channels[i], rf_scan_ctx.chn_rssi_max[best_channels[i]]);
        //     print_idx += sprintf(&print_buf[print_idx], "[%d]%d", wifi_chn_inf.rssi_sorted_idx[i] + 1, wifi_chn_inf.rssi_sum[wifi_chn_inf.rssi_sorted_idx[i]]);
        //     // print_idx += sprintf(&print_buf[print_idx], "[%d]%d", i + 1, (wifi_chn_inf.rssi_sum[i] - 300));
        // }
        // print_idx += sprintf(&print_buf[print_idx], "\n");

        // #if TPSLL_WIFI_RSSI_SCORE_EN
        // print_idx += sprintf(&print_buf[print_idx], "valid chn score:");
        // for (int i = 0; i < RSSI_SCAN_CHANNEL_COUNT; i++) {
        //     for (int v = 0; v < TPSLL_FHSS_CHN_MAX_NUM; v++) {
        //         if (g_fhss_valid_chn_tab[v] == i)
        //             print_idx += sprintf(&print_buf[print_idx], "[%d]%d", i, wifi_chn_inf.valid_chn_score[v]);
        //     }
        // }
        // print_idx += sprintf(&print_buf[print_idx], "\n");
        // #endif

        // printf("%s", print_buf);
        #endif
    
        // if (ll_ctx.stk_state == STATE_PAIRING) {
        //     tlkapi_printf(APP_LOG_EN, "(%d-%d-%d)\n", reconn_chn_table[0], reconn_chn_table[1], reconn_chn_table[2]);
        // } else {
        //     tlkapi_printf(APP_LOG_EN, "(%d)(%d)(%d)\n", reconn_chn_table[0], reconn_chn_table[1], reconn_chn_table[2]);
        // }
        // #if (TPSLL_RF_SCAN_SELECT_CHN_NUM == 6)
        // tlkapi_printf(APP_LOG_EN, "<%d-%d-%d-%d-%d-%d>\n", rf_scan_ctx.select_chn_table[0], rf_scan_ctx.select_chn_table[1],
        //                     rf_scan_ctx.select_chn_table[2], rf_scan_ctx.select_chn_table[3],
        //                     rf_scan_ctx.select_chn_table[4], rf_scan_ctx.select_chn_table[5]);
        // #else
        // tlkapi_printf(APP_LOG_EN, "<%d-%d-%d>\n", rf_scan_ctx.select_chn_table[0], rf_scan_ctx.select_chn_table[1],
        //                     rf_scan_ctx.select_chn_table[2]);
        // #endif

        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]); 
    }
}
#else /* end of TPSLL_RF_SCAN_EN */

static void rf_rssi_scan(void)
{
    // delay_ms(100);

    core_interrupt_enable();
    rf_rssi_scan_param_set();
    rf_rssi_scan_start();

    while (1)
    {
        if (scan_state == STATE_SCAN_DONE)
        {
            #if TPSLL_NEW_RECON_EN
            uint8_t best_channels[RSSI_SCAN_CHANNEL_COUNT];
            // uint8_t best_channels[RSSI_SCAN_TOP_CHANNEL_COUNT];

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

            #if TPSLL_AVOID_WIFI_CHN_EN
            // wifi_chn_inf.rssi_sum[0] = g_chn_rssi_max[37] / 2 + g_chn_rssi_max[0] + g_chn_rssi_max[1] + g_chn_rssi_max[2] +
            //                     g_chn_rssi_max[3] + g_chn_rssi_max[4] + g_chn_rssi_max[5] + g_chn_rssi_max[6] +
            //                     g_chn_rssi_max[7] + g_chn_rssi_max[8] + g_chn_rssi_max[9] / 2;
            wifi_chn_inf.rssi_sum[0] = (wifi_chn_inf.rssi_sum[0] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (g_chn_rssi_max[37] / 2 + g_chn_rssi_max[0] + g_chn_rssi_max[1] + g_chn_rssi_max[2] +
                                g_chn_rssi_max[3] + g_chn_rssi_max[4] + g_chn_rssi_max[5] + g_chn_rssi_max[6] +
                                g_chn_rssi_max[7] + g_chn_rssi_max[8] + g_chn_rssi_max[9] / 2) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;
            // wifi_chn_inf.rssi_sum[1] = g_chn_rssi_max[2] + g_chn_rssi_max[3] + g_chn_rssi_max[4] + g_chn_rssi_max[5] +
            //                     g_chn_rssi_max[6] + g_chn_rssi_max[7] + g_chn_rssi_max[8] + g_chn_rssi_max[9] +
            //                     g_chn_rssi_max[10] + g_chn_rssi_max[38];
            wifi_chn_inf.rssi_sum[1] = (wifi_chn_inf.rssi_sum[1] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (g_chn_rssi_max[2] + g_chn_rssi_max[3] + g_chn_rssi_max[4] + g_chn_rssi_max[5] +
                                g_chn_rssi_max[6] + g_chn_rssi_max[7] + g_chn_rssi_max[8] + g_chn_rssi_max[9] +
                                g_chn_rssi_max[10] + g_chn_rssi_max[38]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;
            // wifi_chn_inf.rssi_sum[2] = g_chn_rssi_max[4] / 2 + g_chn_rssi_max[5] + g_chn_rssi_max[6] + g_chn_rssi_max[7] +
            //                     g_chn_rssi_max[8] + g_chn_rssi_max[9] + g_chn_rssi_max[10] + g_chn_rssi_max[38] +
            //                     g_chn_rssi_max[11] + g_chn_rssi_max[12] + g_chn_rssi_max[13] / 2;
            wifi_chn_inf.rssi_sum[2] = (wifi_chn_inf.rssi_sum[2] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (g_chn_rssi_max[4] / 2 + g_chn_rssi_max[5] + g_chn_rssi_max[6] + g_chn_rssi_max[7] +
                                g_chn_rssi_max[8] + g_chn_rssi_max[9] + g_chn_rssi_max[10] + g_chn_rssi_max[38] +
                                g_chn_rssi_max[11] + g_chn_rssi_max[12] + g_chn_rssi_max[13] / 2) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

            // wifi_chn_inf.rssi_sum[3] = g_chn_rssi_max[7] + g_chn_rssi_max[8] + g_chn_rssi_max[9] + g_chn_rssi_max[10] +
            //                     g_chn_rssi_max[38] + g_chn_rssi_max[11] + g_chn_rssi_max[12] + g_chn_rssi_max[13] +
            //                     g_chn_rssi_max[14] + g_chn_rssi_max[15];
            wifi_chn_inf.rssi_sum[3] = (wifi_chn_inf.rssi_sum[3] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (g_chn_rssi_max[7] + g_chn_rssi_max[8] + g_chn_rssi_max[9] + g_chn_rssi_max[10] +
                                g_chn_rssi_max[38] + g_chn_rssi_max[11] + g_chn_rssi_max[12] + g_chn_rssi_max[13] +
                                g_chn_rssi_max[14] + g_chn_rssi_max[15]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

            // wifi_chn_inf.rssi_sum[4] = g_chn_rssi_max[9] / 2 + g_chn_rssi_max[10] + g_chn_rssi_max[38] + g_chn_rssi_max[11] +
            //                     g_chn_rssi_max[12] + g_chn_rssi_max[13] + g_chn_rssi_max[14] + g_chn_rssi_max[15] +
            //                     g_chn_rssi_max[16] + g_chn_rssi_max[17] + g_chn_rssi_max[18] / 2;
            wifi_chn_inf.rssi_sum[4] = (wifi_chn_inf.rssi_sum[4] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (g_chn_rssi_max[9] / 2 + g_chn_rssi_max[10] + g_chn_rssi_max[38] + g_chn_rssi_max[11] +
                                g_chn_rssi_max[12] + g_chn_rssi_max[13] + g_chn_rssi_max[14] + g_chn_rssi_max[15] +
                                g_chn_rssi_max[16] + g_chn_rssi_max[17] + g_chn_rssi_max[18] / 2) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

            // wifi_chn_inf.rssi_sum[5] = g_chn_rssi_max[11] + g_chn_rssi_max[12] + g_chn_rssi_max[13] + g_chn_rssi_max[14] +
            //                     g_chn_rssi_max[15] + g_chn_rssi_max[16] + g_chn_rssi_max[17] + g_chn_rssi_max[18] +
            //                     g_chn_rssi_max[19] + g_chn_rssi_max[20];
            wifi_chn_inf.rssi_sum[5] = (wifi_chn_inf.rssi_sum[5] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (g_chn_rssi_max[11] + g_chn_rssi_max[12] + g_chn_rssi_max[13] + g_chn_rssi_max[14] +
                                g_chn_rssi_max[15] + g_chn_rssi_max[16] + g_chn_rssi_max[17] + g_chn_rssi_max[18] +
                                g_chn_rssi_max[19] + g_chn_rssi_max[20]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

            // wifi_chn_inf.rssi_sum[6] = g_chn_rssi_max[13] / 2 + g_chn_rssi_max[14] + g_chn_rssi_max[15] + g_chn_rssi_max[16] +
            //                     g_chn_rssi_max[17] + g_chn_rssi_max[18] + g_chn_rssi_max[19] + g_chn_rssi_max[20] +
            //                     g_chn_rssi_max[21] + g_chn_rssi_max[22] + g_chn_rssi_max[23] / 2;
            wifi_chn_inf.rssi_sum[6] = (wifi_chn_inf.rssi_sum[6] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (g_chn_rssi_max[13] / 2 + g_chn_rssi_max[14] + g_chn_rssi_max[15] + g_chn_rssi_max[16] +
                                g_chn_rssi_max[17] + g_chn_rssi_max[18] + g_chn_rssi_max[19] + g_chn_rssi_max[20] +
                                g_chn_rssi_max[21] + g_chn_rssi_max[22] + g_chn_rssi_max[23] / 2) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

            // wifi_chn_inf.rssi_sum[7] = g_chn_rssi_max[16] + g_chn_rssi_max[17] + g_chn_rssi_max[18] + g_chn_rssi_max[19] +
            //                     g_chn_rssi_max[20] + g_chn_rssi_max[21] + g_chn_rssi_max[22] + g_chn_rssi_max[23] +
            //                     g_chn_rssi_max[24] + g_chn_rssi_max[25];
            wifi_chn_inf.rssi_sum[7] = (wifi_chn_inf.rssi_sum[7] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (g_chn_rssi_max[16] + g_chn_rssi_max[17] + g_chn_rssi_max[18] + g_chn_rssi_max[19] +
                                g_chn_rssi_max[20] + g_chn_rssi_max[21] + g_chn_rssi_max[22] + g_chn_rssi_max[23] +
                                g_chn_rssi_max[24] + g_chn_rssi_max[25]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

            // wifi_chn_inf.rssi_sum[8] = g_chn_rssi_max[18] /2 + g_chn_rssi_max[19] + g_chn_rssi_max[20] + g_chn_rssi_max[21] +
            //                     g_chn_rssi_max[22] + g_chn_rssi_max[23] + g_chn_rssi_max[24] + g_chn_rssi_max[25] +
            //                     g_chn_rssi_max[26] + g_chn_rssi_max[27] + g_chn_rssi_max[28] / 2;
            wifi_chn_inf.rssi_sum[8] = (wifi_chn_inf.rssi_sum[8] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (g_chn_rssi_max[18] /2 + g_chn_rssi_max[19] + g_chn_rssi_max[20] + g_chn_rssi_max[21] +
                                g_chn_rssi_max[22] + g_chn_rssi_max[23] + g_chn_rssi_max[24] + g_chn_rssi_max[25] +
                                g_chn_rssi_max[26] + g_chn_rssi_max[27] + g_chn_rssi_max[28] / 2) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

            // wifi_chn_inf.rssi_sum[9] = (g_chn_rssi_max[21] + g_chn_rssi_max[22] + g_chn_rssi_max[23] + g_chn_rssi_max[24] +
            //                     g_chn_rssi_max[25] + g_chn_rssi_max[26] + g_chn_rssi_max[27] + g_chn_rssi_max[28] +
            //                     g_chn_rssi_max[29] + g_chn_rssi_max[30]);
            wifi_chn_inf.rssi_sum[9] = (wifi_chn_inf.rssi_sum[9] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (g_chn_rssi_max[21] + g_chn_rssi_max[22] + g_chn_rssi_max[23] + g_chn_rssi_max[24] +
                                g_chn_rssi_max[25] + g_chn_rssi_max[26] + g_chn_rssi_max[27] + g_chn_rssi_max[28] +
                                g_chn_rssi_max[29] + g_chn_rssi_max[30]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

            // wifi_chn_inf.rssi_sum[10] = g_chn_rssi_max[23] / 2 + g_chn_rssi_max[24] + g_chn_rssi_max[25] + g_chn_rssi_max[26] +
            //                     g_chn_rssi_max[27] + g_chn_rssi_max[28] + g_chn_rssi_max[29] + g_chn_rssi_max[30] +
            //                     g_chn_rssi_max[31] + g_chn_rssi_max[32] + g_chn_rssi_max[33] / 2;
            wifi_chn_inf.rssi_sum[10] = (wifi_chn_inf.rssi_sum[10] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (g_chn_rssi_max[23] / 2 + g_chn_rssi_max[24] + g_chn_rssi_max[25] + g_chn_rssi_max[26] +
                                g_chn_rssi_max[27] + g_chn_rssi_max[28] + g_chn_rssi_max[29] + g_chn_rssi_max[30] +
                                g_chn_rssi_max[31] + g_chn_rssi_max[32] + g_chn_rssi_max[33] / 2) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

            // wifi_chn_inf.rssi_sum[11] = g_chn_rssi_max[26] + g_chn_rssi_max[27] + g_chn_rssi_max[28] + g_chn_rssi_max[29] +
            //                     g_chn_rssi_max[30] + g_chn_rssi_max[31] + g_chn_rssi_max[32] + g_chn_rssi_max[33] +
            //                     g_chn_rssi_max[34] + g_chn_rssi_max[35];
            wifi_chn_inf.rssi_sum[11] = (wifi_chn_inf.rssi_sum[11] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (g_chn_rssi_max[26] + g_chn_rssi_max[27] + g_chn_rssi_max[28] + g_chn_rssi_max[29] +
                                g_chn_rssi_max[30] + g_chn_rssi_max[31] + g_chn_rssi_max[32] + g_chn_rssi_max[33] +
                                g_chn_rssi_max[34] + g_chn_rssi_max[35]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

            // wifi_chn_inf.rssi_sum[12] = g_chn_rssi_max[28] + g_chn_rssi_max[29] + g_chn_rssi_max[30] + g_chn_rssi_max[31] +
            //                     g_chn_rssi_max[32] + g_chn_rssi_max[33] + g_chn_rssi_max[34] + g_chn_rssi_max[35] +
            //                     g_chn_rssi_max[36] + g_chn_rssi_max[39];
            wifi_chn_inf.rssi_sum[12] = (wifi_chn_inf.rssi_sum[12] * TPSLL_WIFI_RSSI_SUM_HISTORY_N + (g_chn_rssi_max[28] + g_chn_rssi_max[29] + g_chn_rssi_max[30] + g_chn_rssi_max[31] +
                                g_chn_rssi_max[32] + g_chn_rssi_max[33] + g_chn_rssi_max[34] + g_chn_rssi_max[35] +
                                g_chn_rssi_max[36] + g_chn_rssi_max[39]) * TPSLL_WIFI_RSSI_SUM_NEW_N) / TPSLL_WIFI_RSSI_SUM_TOTAL_N;

            tpsll_bubble_sort_rssi_index(wifi_chn_inf.rssi_sum, TPSLL_WIFI_CHANNEL_COUNT, wifi_chn_inf.rssi_sorted_idx);

            // printf("wifi:");
            // for (int i = 0; i < TPSLL_WIFI_CHANNEL_COUNT; i++) {
            //     printf("[%d]%d", wifi_chn_inf.rssi_sorted_idx[i] + 1, wifi_chn_inf.rssi_sum[wifi_chn_inf.rssi_sorted_idx[i]]);
            //     // printf("<%d,%d>\n", i + 1, wifi_chn_inf.rssi_sum[i]);
            // }
            // printf("\n");

            #endif /* end of TPSLL_AVOID_WIFI_CHN_EN */

            #if 0
            tlkapi_send_string_data(APP_LOG_EN, "best chns(H)", best_channels, RSSI_SCAN_TOP_CHANNEL_COUNT);
            #endif

            // tmemcpy(reconn_chn_table, best_channels, TPSLL_RECON_CHN_NUM);
            int k = 0;

            for (int i = 0; i < RSSI_SCAN_CHANNEL_COUNT && k < TPSLL_RECON_CHN_NUM; i++) {
                #if TPSLL_NEW_PAIR_EN
                if (ll_ctx.stk_state == STATE_PAIRING) {
                    for (int j = 0; j < TPSLL_PAIR_CHN_MAX_NUM; j++) {
                        if (best_channels[i] == g_pair_valid_chn_tab[j]) {
                            reconn_chn_table[k] = g_pair_valid_chn_tab[j];
                            k++;
                        }
                    }
                } else
                #endif
                {
                    #if TPSLL_AVOID_WIFI_CHN_EN
//////////////////////////////////////////////////////////////////////////////
                    for (int w = 0; w < TPSLL_WIFI_BEST_CHN_NUM; w++) {
                        for (int j = 0; j < wifi_chn_inf.valid_chn[wifi_chn_inf.rssi_sorted_idx[w]].num; j++) {
                            if (best_channels[i] == wifi_chn_inf.valid_chn[wifi_chn_inf.rssi_sorted_idx[w]].table[j]) {
                                reconn_chn_table[k] = best_channels[i];
                                k++;
                                w = TPSLL_WIFI_BEST_CHN_NUM;
                                break;
                            }
                        }
                    }
//////////////////////////////////////////////////////////////////////////////
                    #else
                    for (int j = 0; j < TPSLL_FHSS_CHN_MAX_NUM; j++) {
                        if (best_channels[i] == g_fhss_valid_chn_tab[j]) {
                            reconn_chn_table[k] = g_fhss_valid_chn_tab[j];
                            k++;
                            break;
                        }
                    }
                    #endif
                }
            }

            // #if TPSLL_8K_CHN_EXPAND_EN
            // tlkapi_printf(APP_LOG_EN, "rcn(%d)(%d)(%d)(%d)(%d)(%d)\n", reconn_chn_table[0], reconn_chn_table[1], reconn_chn_table[2],
            //                 reconn_chn_table[3], reconn_chn_table[4], reconn_chn_table[5]);
            // #else
            // tlkapi_printf(APP_LOG_EN, "rcn(%d)(%d)(%d)\n", reconn_chn_table[0], reconn_chn_table[1], reconn_chn_table[2]);
            // #endif

            // tlkapi_printf(1, "fhss chns:<\n");
            // for(int i = 0; i < TPSLL_FHSS_CHN_MAX_NUM; i++) {
            //     tlkapi_printf(1, "[%d]%d\n", i, g_fhss_valid_chn_tab[i]);
            // }
            // tlkapi_printf(1, ">\n");
            #else /* end of TPSLL_NEW_RECON_EN */
            uint8_t best_channels[RSSI_SCAN_TOP_CHANNEL_COUNT];
            rf_select_best_channels(best_channels, RSSI_SCAN_TOP_CHANNEL_COUNT);

            #if 1
            tlkapi_send_string_data(APP_LOG_EN, "best chns(H)", best_channels, RSSI_SCAN_TOP_CHANNEL_COUNT);
            #endif

            #if TPSLL_FHSS_EN
            tpsll_channel_statistic(0xff, 0);

            tlkapi_send_string_data(APP_LOG_EN, "fhss chns(H)", g_fhss_valid_chn_tab, RSSI_SCAN_TOP_CHANNEL_COUNT);

            // tlkapi_printf(1, "fhss chns:<\n");
            // for(int i = 0; i < TPSLL_FHSS_CHN_MAX_NUM; i++) {
            //     tlkapi_printf(1, "[%d]%d\n", i, g_fhss_valid_chn_tab[i]);
            // }
            // tlkapi_printf(1, ">\n");

            g_hop_map_cur = 0;
            tmemset(chn_table, 0, 3);
            for (int i = 0; i < TPSLL_HOP_CHN_NUM; i++) {
                for (int j = 0; j < TPSLL_FHSS_CHN_MAX_NUM; j++) {
                    if (best_channels[i] == g_fhss_valid_chn_tab[j]) {
                        chn_table[i] = g_fhss_valid_chn_tab[j];
                        g_hop_map_cur |= BIT(j);
                    }
                }
            }
            g_hop_map_next = g_hop_map_cur;
        // tlkapi_printf(1, "m(0x%08X)\nchn_tab:%d,%d,%d\n", g_hop_map_cur,
        //             chn_table[0], chn_table[1], chn_table[2]);

            #else
            // scan_state = STATE_SCAN_DONE;
            tmemcpy(chn_table, best_channels, CHN_TABL_LEN);
            // chn_table[0] = 33;
            // chn_table[1] = 33;
            // chn_table[2] = 33;
            // chn_table[3] = 33;
            #endif

            #if AFH_ENABLE
            init_reverse_table();
            #endif
            #endif /* end of TPSLL_NEW_RECON_EN #else */

            break;

        }
    }

    rf_rssi_scan_param_restore();
}

_attribute_ram_code_sec_ static void proc_rssi_scan(void)
{
    uint8_t rssi = REG_RSSI_MAX_BB;
    rssi_result[current_channel] += rssi;
    scan_counts[current_channel]++;

    #if TPSLL_NEW_RECON_EN
    if (g_chn_rssi_max[current_channel] < rssi)
        g_chn_rssi_max[current_channel] = rssi; //max rssi
    #endif

    CLR_REG_RSSI_MAX_BB();
    // if ((current_channel == 1) || (current_channel == 5) || (current_channel == 12) ||
    //     (current_channel == 17) || (current_channel == 28) || (current_channel == 34))
    // printf("[%d:%d]\n", current_channel, rssi);

    do
    {
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
    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
    rf_set_ble_chn(current_channel);
    pp_rf_start_srx(stimer_get_tick(), RSSI_SCAN_WIN_US);
}
#endif /* end of TPSLL_RF_SCAN_EN #else */
#endif // RSSI_SCAN_EN end


// _attribute_ram_code_sec_ static inline void p24g_set_snnesn(void)
// {
//     SET_SN(rf_tx_packet.snnesn, ll_ctx.tx_sn);
//     SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);
// }

// _attribute_ram_code_sec_ void tpsll_fill_spp_flag(void)
// {
//     if (!pp_fifo_is_empty(&spp_fifo))
//     {
//         rf_tx_packet.pkt_type = P24G_PKT_TYPE_PENDING_SPP;
//     }else{
//         rf_tx_packet.pkt_type = P24G_PKT_TYPE_SYNC;
//     }
// }

_attribute_ram_code_sec_ static uint8_t p24g_fill_tx_packet(void)
{
    uint8_t len = 0;

    if(ll_ctx.report_rate != REPORT_RATE_8K && !ll_ctx.last_pkt_acked)
    {
        // if (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250)) {
        //     len = RF_PKT_NULL_125_ACK_LEN;
        // }
        // #if TPSLL_NEW_SYNC_EN
        // else if (ll_ctx.report_rate & REPORT_RATE_500) {
        //     len = RF_PKT_NULL_500_ACK_LEN;
        // }
        // #endif

        return len;
    }


    ll_ctx.last_pkt_acked = false;

    if (!pp_fifo_is_empty(&ll_fifo) && !ll_ctx.spp_flag)
    {
        #if TPSLL_SPP_NOT_SWITCH_EN
        if (ll_ctx.report_rate & REPORT_RATE_8K) {
            unsigned char *p = pp_fifo_get_ptr(&ll_fifo);
            if (p)
            {
                rf_tx_packet.pkt_type = P24G_PKT_TYPE_LL;
                tmemcpy(&rf_tx_packet.buf[3], &p[2], p[0]);
                ll_ctx.ll_flag = true;
                #if TPSLL_RR_CHG_WITH_RECON_EN && !TPSLL_RR_CHG_WITH_RECON_NO_RSP_EN
                if (p[2] == TPSLL_LL_CMD_RR_CHG_RCNN_RSP) {
                    if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP1) {
                        ll_ctx.rr_chg_with_recon = RR_RECON_STEP2;
                    }
                }
                #endif
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
                len = 3 + p[0];
            } else {
                len = 3;
            }
        } else {
            len = p24g_fill_ll_pkt();
        }
        #else
        len = p24g_fill_ll_pkt();
        #endif
    }
    else if (!pp_fifo_is_empty(&spp_fifo))
    {
        #if TPSLL_SPP_NOT_SWITCH_EN || TPSLL_NON_8K_SPP_NOT_SW_EN
        if (ll_ctx.report_rate & REPORT_RATE_8K) {
            rf_tx_packet.pkt_type = P24G_PKT_TYPE_SPP;
            unsigned char *p = pp_fifo_get_ptr(&spp_fifo);

// DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
            tmemcpy((void *)&rf_tx_packet.buf[3], (void *)&p[2], p[0]);
            len = 3 + p[0];
            ll_ctx.spp_flag = true;

        }
        #if TPSLL_NEW_SYNC_EN
        else if ((ll_ctx.nspp_cnt >= NSPP_CNT) || (ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500)))
        #else
        else if(ll_ctx.nspp_cnt >= NSPP_CNT)
        #endif
        {
            // ll_ctx.spp_cnt = 0;
            len = tpsll_fill_spp_data();
            // tlkapi_printf(APP_LOG_EN, "data_len:%d \n", data_length);
        }else{
            len = p24g_fill_null_pkt();
        }
        #else
        rf_tx_packet.pkt_type = P24G_PKT_TYPE_PENDING_SPP;

        if (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250)) {
            len = RF_PKT_NULL_125_ACK_LEN;
        } else {
            len = RF_PKT_NULL_ACK_LEN;
        }
        #endif
    }
    else
    {
        #if TPSLL_SPP_NOT_SWITCH_EN || TPSLL_NON_8K_SPP_NOT_SW_EN
        if (ll_ctx.report_rate & REPORT_RATE_8K) {
            len = 3;
            rf_tx_packet.pkt_type = P24G_PKT_TYPE_SYNC;
        } else
        #endif
        {
            len = p24g_fill_null_pkt();
        }
        // if (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250)) {
        //     len = p24g_fill_null_pkt_125();
        // } else {
        //     len = p24g_fill_null_pkt();
        // }
    }

    if(!ll_ctx.spp_flag && (ll_ctx.task_slot == TASK_SLOT0))
    {
        ll_ctx.nspp_cnt++;
        ll_ctx.spp_cnt = 0;
    }
    // tpsll_fill_spp_flag();
    return len;
}


_attribute_ram_code_sec_ uint8_t tpsll_fill_spp_data(void)
{//for none 8k report rate (125 ~ 4k)
    // tlkapi_printf(APP_LOG_EN, "wptr:%d rptr:%d\n",spp_fifo.wptr,spp_fifo.rptr);
    uint8_t len = 0;
    unsigned char *p = pp_fifo_get_ptr(&spp_fifo);
    if (p != 0)
    {
        uint8_t fifo_data_len = p[0];
        uint8_t cmd = p[1];
        if (cmd == TPSLL_SPP_DATA)
        {
            len = fifo_data_len;
            #if TPSLL_NEW_SYNC_EN
            if (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250 | (REPORT_RATE_1K | REPORT_RATE_500)))
            #else
            if (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250))
            #endif
            {
                #if TPSLL_N8K_FHSS_EN
                // if(g_hop_map_cur != g_hop_map_next)
                {
                    tmemcpy((void *)&rf_tx_packet.buf[4], (void *)&p[2], fifo_data_len);
                    len += 4;
                }
                #if 0
                else{
                    tmemcpy((void *)&rf_tx_packet.buf[2], (void *)&p[2], fifo_data_len);
                    len += 2;
                }
                #endif
                #else
                tmemcpy((void *)&rf_tx_packet.buf[2], (void *)&p[2], fifo_data_len);
                len += 2;
                #endif
                
            }
            else
            {
                #if TPSLL_N8K_FHSS_EN
                // if(g_hop_map_cur != g_hop_map_next)
                {
                    tmemcpy((void *)&rf_tx_packet.buf[2], (void *)&p[2], fifo_data_len);
                    len += 2;
                }
                #if 0
                else{
                    tmemcpy((void *)&rf_tx_packet.buf[0], (void *)&p[2], fifo_data_len);
                }
                #endif
                #else
                tmemcpy((void *)&rf_tx_packet.buf[0], (void *)&p[2], fifo_data_len);
                #endif
            }

            rf_tx_packet.pkt_type = P24G_PKT_TYPE_SPP;
            ll_ctx.spp_flag = true;
        }
    }
    return len;
}


#if !TPSLL_SPP_NOT_SWITCH_EN
_attribute_ram_code_sec_ static uint8_t tpsll_fill_spp_mode_tx_packet(void)
{

    uint8_t data_length = 0;
    if (!pp_fifo_is_empty(&ll_fifo))
    {
       data_length = p24g_fill_ll_pkt();
    }
    else if (!pp_fifo_is_empty(&spp_fifo))
    {
        data_length = tpsll_fill_spp_data();
        // tlkapi_printf(APP_LOG_EN, "data_len:%d \n", data_length);
    }
    else
    {
        if (ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250))
        {
           data_length = p24g_fill_null_pkt_125();
        }else{
            data_length = p24g_fill_null_pkt();
        }
    }

    // tpsll_fill_spp_flag();

    return data_length;

}
#endif

#if TPSLL_NEW_PAIR_EN
_attribute_ram_code_sec_ void tpsll_pair_pkt_send(void)
{
    if (ll_ctx.pair_st == STATE_PAIR_INIT) {
        uint32_t stick = rf_stimer_get_tick();
        uint16_t diff_us = (ll_ctx.stick + DONGLE_INTERVAL_US * SYSTEM_TIMER_TICK_1US - (stick + 5 * SYSTEM_TIMER_TICK_1US)) / SYSTEM_TIMER_TICK_1US;

        rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, stick + 5 * SYSTEM_TIMER_TICK_1US);

        pair_rsp_packet_t * pt = (pair_rsp_packet_t *)(rf_tx_packet.buf);

        pt->cmd = CMD_PAIR_RSP;

        tmemcpy(pt->mac, ll_inf.mac, MAC_ADDR_LEN);

        if (ll_inf.connecting_dev & DEVICE_TYPE_KB)
        {
            pt->km_tag = ll_ctx.ktag;
        }
        else if (ll_inf.connecting_dev & DEVICE_TYPE_MS)
        {
            pt->km_tag = ll_ctx.mtag;
        }

        pt->diff_us = diff_us;

        pp_rf_set_tx_len(TPSLL_PAIR_RSP_PKT_LEN);
        // tlkapi_printf(APP_LOG_EN, "diff us:%d\n", pt->diff_us);
    } else if (ll_ctx.pair_st == STATE_PAIR_CFM) {
        conn_rsp_packet_t *pt = (conn_rsp_packet_t *)(rf_tx_packet.buf);

        rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, rf_stimer_get_tick() + 5 * SYSTEM_TIMER_TICK_1US);

        pt->cmd         = CMD_CONN_RSP;
        pt->fno_last    = ll_ctx.fno_last;
        pt->d_tag       = ll_inf.tag;

        if (ll_inf.connecting_dev & DEVICE_TYPE_KB)
        {
            pt->km_tag = ll_ctx.ktag;
        }
        else if (ll_inf.connecting_dev & DEVICE_TYPE_MS)
        {
            pt->km_tag = ll_ctx.mtag;
        }

        #if TPSLL_RECON_RSSI_SCAN_EN
        #if TPSLL_8K_CHN_EXPAND_EN
        pp_rf_set_tx_len(TPSLL_PAIR_CON_RSP_LEN + 3);
        pt->chn_tab[0] = reconn_chn_table[0];
        pt->chn_tab[1] = reconn_chn_table[1];
        pt->chn_tab[2] = reconn_chn_table[2];
        pt->chn_tab[3] = reconn_chn_table[3];
        pt->chn_tab[4] = reconn_chn_table[4];
        pt->chn_tab[5] = reconn_chn_table[5];
        #else
        pp_rf_set_tx_len(TPSLL_PAIR_CON_RSP_LEN);
        pt->chn_tab[0] = reconn_chn_table[0];
        pt->chn_tab[1] = reconn_chn_table[1];
        pt->chn_tab[2] = reconn_chn_table[2];
        #endif
        #else
        pp_rf_set_tx_len(TPSLL_PAIR_CON_RSP_LEN - 3);
        #endif
    }
}
#endif

#if TPSLL_NEW_RECON_EN
_attribute_ram_code_sec_ void tpsll_recon_pkt_send(void)
{
    reconn_rsp_packet_t *pt = (reconn_rsp_packet_t *)&rf_tx_packet;
    uint32_t stick = rf_stimer_get_tick();

    rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, stick + 5 * SYSTEM_TIMER_TICK_1US);

    pt->cmd                 = CMD_RECONN_RSP;
    pt->d_tag              = ll_inf.tag;

    pt->fno_last = ll_ctx.fno_last;
    pt->diff_us = (ll_ctx.last_stick + DONGLE_INTERVAL_US * SYSTEM_TIMER_TICK_1US - (stick + TPSLL_RECONN_ALIGN_US * SYSTEM_TIMER_TICK_1US)) / SYSTEM_TIMER_TICK_1US;

    #if TPSLL_RECON_RSSI_SCAN_EN
    #if TPSLL_8K_CHN_EXPAND_EN
    pp_rf_set_tx_len(12 + 3);
    pt->chn_tab[0] = reconn_chn_table[0];
    pt->chn_tab[1] = reconn_chn_table[1];
    pt->chn_tab[2] = reconn_chn_table[2];
    pt->chn_tab[3] = reconn_chn_table[3];
    pt->chn_tab[4] = reconn_chn_table[4];
    pt->chn_tab[5] = reconn_chn_table[5];
    #else
    pp_rf_set_tx_len(12);
    pt->chn_tab[0] = reconn_chn_table[0];
    pt->chn_tab[1] = reconn_chn_table[1];
    pt->chn_tab[2] = reconn_chn_table[2];
    #endif
    #else
    pp_rf_set_tx_len(9);
    #endif
    pt->crc16 = p24g_crc16(0xFFFF, (uint8_t *)&pt->cmd, offsetof(reconn_rsp_packet_t, crc16) - offsetof(reconn_rsp_packet_t, cmd));
}
#endif

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
        return 1;
    }

    memcpy(ll_ctx.customer_key, key, key_len);
    ll_ctx.customer_key_len = key_len;

    if (s_app_cb && s_app_cb->usbd_get_dev_descp)
    {
        uint8_t *desc = s_app_cb->usbd_get_dev_descp(0);
        ll_ctx.usb_vid = desc[9] << 8 | desc[8];
        
        tpsll_generate_pair_encrypt_key();
    }

    return 0;
}

/**
 * @brief  validate the encrypt key of the PAIR_REQ packet
 */
pair_encrypt_status_t tpsll_verify_pair_encrypt_key(uint32_t key)
{
    if (key == ll_ctx.pair_encrypt_key) {
        return PAIR_ENCRYPT_OK;
    } else {
        return PAIR_ENCRYPT_ERR;
    }
}

uint8_t tpsll_set_pairing_rssi_threshold(int8_t threshold)
{
    g_pairing_rssi_threshold = threshold;
    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ static void p24g_proc_8k_paring_rx(rf_packet_t *p)
{
    #if TPSLL_NEW_PAIR_EN
/////////////////////////////////////////

    if (ll_ctx.pair_st == STATE_PAIR_INIT) {
        pair_req_packet_t * pr = (pair_req_packet_t *)(p->buf);

        if (pr->cmd == CMD_PAIR_REQ) {
            signed char rssi = (signed char)rf_get_rssi();          
            if (rssi < g_pairing_rssi_threshold) {
                tlkapi_send_string_data(APP_LOG_EN, "pairing_req rssi too low", (unsigned char *)&rssi, 1);
                return;
            }
            
            if (tpsll_verify_pair_encrypt_key(pr->encrypt_key) != PAIR_ENCRYPT_OK) {
                tlkapi_send_string_data(APP_LOG_EN, "pairing_req verify encrypt key fail!", (unsigned char *)p, p->dma_len);
                return;
            }

            if ((pr->km_tag >> 14) & DEVICE_TYPE_KB)
            {
                ll_ctx.ktag = pr->km_tag;
                ll_inf.connecting_dev = DEVICE_TYPE_KB;
            }
            else if ((pr->km_tag >> 14) & DEVICE_TYPE_MS)
            {
                ll_ctx.mtag = pr->km_tag;
                ll_inf.connecting_dev = DEVICE_TYPE_MS;
            }


            unsigned int r = core_interrupt_disable();

            tpsll_pair_pkt_send();

            ll_ctx.stick += DONGLE_INTERVAL_US * SYSTEM_TIMER_TICK_1US;
            stimer_clr_irq_mask_d25f(FLD_SYSTEM_IRQ_MASK_D25F);
            stimer_set_irq_capture_d25f(ll_ctx.stick);
            stimer_clr_irq_status_d25f(FLD_SYSTEM_IRQ_D25F);
            stimer_set_irq_mask_d25f(FLD_SYSTEM_IRQ_MASK_D25F);

            core_restore_interrupt(r);

            // tlkapi_printf(APP_LOG_EN, "pairing_req: pr-kmtag(0x%X)\n", pr->km_tag);
            // tlkapi_printf(APP_LOG_EN, "pairing req: dmac(%02X%02X%02X%02X%02X%02X)kmtag(0x%X)\n", ll_inf.mac[0], ll_inf.mac[1], ll_inf.mac[2],
            //     ll_inf.mac[3], ll_inf.mac[4], ll_inf.mac[5], pr->km_tag);
            ll_ctx.pair_cnt = 0;
            ll_ctx.pair_st = STATE_PAIR_RSP;
            ll_ctx.pair_step = PAIR_STEP2;
            tlkapi_send_string_data(APP_LOG_EN, "pairing_req", (unsigned char *)p, p->dma_len);
        }
    }
    else if (ll_ctx.pair_st == STATE_PAIR_RSP)
    {
        conn_req_packet_t * pr = (conn_req_packet_t *)(p->buf);

        if ((pr->cmd == CMD_CONN_REQ) && (pr->tag == ll_inf.tag) && ((pr->km_tag == ll_ctx.mtag) || (pr->km_tag == ll_ctx.ktag))) {
            ll_ctx.pair_st = STATE_PAIR_CFM;
            tpsll_pair_pkt_send();

            tmemcpy(ll_inf.peer_mac, pr->mac, MAC_ADDR_LEN);
            ll_inf.side_id = fnv1a_hash(ll_inf.peer_mac, MAC_ADDR_LEN);

            if (tpsll_is_report_rate_valid(pr->rr)) {
                ll_ctx.user_rr = pr->rr;
                ll_ctx.chg_rr = ll_ctx.user_rr;
                ll_ctx.pending_rr_fno = ll_ctx.fno_last + TPSLL_RR_DELAY_SWITCH_FNO_N;
                ll_ctx.is_rr_changing = true;
            } else {
                ll_ctx.user_rr = APP_DEFAULT_REPORT_RATE;
            }

            #if TPSLL_RECON_RSSI_SCAN_EN
            {
                chn_table[0] = reconn_chn_table[0];
                chn_table[1] = reconn_chn_table[1];
                chn_table[2] = reconn_chn_table[2];
                #if TPSLL_8K_CHN_EXPAND_EN
                chn_table[3] = reconn_chn_table[3];
                chn_table[4] = reconn_chn_table[4];
                chn_table[5] = reconn_chn_table[5];
                #else
                #if TPSLL_N8K_CHN_EXPAND_EN
                tpsll_n8k_chn_expand();
                #else
                #if (CHN_TABL_LEN == 4)
                chn_table[3] = chn_table[1];
                #endif
                #endif
                #endif

                tpsll_fhss_recon_update_chn_per();
            }
            #else
            #if RSSI_SCAN_EN
            tmemcpy(chn_table, pr->chn_table, CHN_TABL_LEN);
            #endif
            #endif

            ll_ctx.enter_conn_flag = true;
            ll_ctx.pair_step = PAIR_STEP3;

            stack_evt_post_from_isr(TPSLL_EVT_DEV_PAIRED, NULL, 0);

            // tlkapi_printf(APP_LOG_EN, "conn_req: tag(0x%X)pr-tag(0x%X)pr-kmtag(0x%X)\n", ll_inf.tag, pr->tag, ll_ctx.mtag, ll_ctx.ktag, pr->km_tag);
            tlkapi_send_string_data(APP_LOG_EN, "conn_req", (unsigned char *)p, p->dma_len);
        } else {
            // tlkapi_printf(APP_LOG_EN, "conn_req err: cmd(%d)tag(0x%X)pr-tag(0x%X)pr-kmtag(0x%X)\n", pr->cmd, ll_inf.tag, pr->tag, pr->km_tag);
            tlkapi_send_string_data(APP_LOG_EN, "conn_req err", (unsigned char *)p, p->dma_len);
        }
    }else{
    //     tlkapi_send_string_data(APP_LOG_EN, "conn_req err", (unsigned char *)p, p->dma_len);
    }
/////////////////////////////////////////
    #else /* end of TPSLL_NEW_PAIR_EN */
    uint16_t tag = 0;
    uint16_t peer_tag = 0;

    if (ll_ctx.pair_st == STATE_PAIR_INIT && p->buf[0] == CMD_PAIR_RSP)
    {
        ll_ctx.pair_step = PAIR_STEP2;
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
        ll_ctx.pair_st = STATE_PAIR_RSP;
        tag = p->buf[1] | (p->buf[2] << 8);
        if ((tag >> 14) & DEVICE_TYPE_KB)
        {
            ll_ctx.ktag = tag;
            ll_inf.connecting_dev = DEVICE_TYPE_KB;
        }
        else if ((tag >> 14) & DEVICE_TYPE_MS)
        {
            ll_ctx.mtag = tag;
            ll_inf.connecting_dev = DEVICE_TYPE_MS;
        }
        tlkapi_send_string_data(APP_LOG_EN, "pairing_rs", (unsigned char *)p, p->dma_len);
        // app_halt_system(0x9988, "state:%x pair_st:%x %x tag:%x ktag:%x mtag:%x\n", ll_ctx.stk_state, ll_ctx.pair_st, p->buf[0], tag, ll_ctx.ktag, ll_ctx.mtag);
    }
    else if (ll_ctx.pair_st == STATE_PAIR_RSP && p->buf[0] == CMD_CONN_RSP)
    {
        ll_ctx.pair_step = PAIR_STEP_NONE;
        pp_timer1_stop();
        timer_clr_irq_status(FLD_TMR1_MODE_IRQ); // clear irq status

        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
        tag = p->buf[4] | (p->buf[5] << 8);
        peer_tag = p->buf[2] | (p->buf[3] << 8);
        tmemcpy(ll_inf.peer_mac, &p->buf[6], MAC_ADDR_LEN);
        ll_inf.side_id = fnv1a_hash(ll_inf.peer_mac, MAC_ADDR_LEN);
        if ((ll_inf.tag == tag) && (peer_tag == ll_ctx.ktag || peer_tag == ll_ctx.mtag))
        {
            if (p->buf[1] > 1)
            {
                ll_ctx.stick += p->buf[1] * SYSTEM_TIMER_TICK_1US * DONGLE_INTERVAL_US;
                stimer_set_irq_capture_d25f(ll_ctx.stick);
            }
            stack_evt_post_from_isr(TPSLL_EVT_DEV_PAIRED, NULL, 0);
            app_d24g_enter_conn();

            if (tpsll_is_report_rate_valid(p->buf[12])) {
                ll_ctx.user_rr = p->buf[12];
                ll_ctx.chg_rr = ll_ctx.user_rr;
                ll_ctx.pending_rr_fno = ll_ctx.fno_last + TPSLL_RR_DELAY_SWITCH_FNO_N;
                ll_ctx.is_rr_changing = true;
            } else {
                ll_ctx.user_rr = APP_DEFAULT_REPORT_RATE;
            }

            tlkapi_send_string_data(APP_LOG_EN, "conn_rsp", &p->rf_len, p->rf_len + 1);
        }
        else
        {
            DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
            tlkapi_send_string_data(APP_LOG_EN, "conn_rsp err", &p->rf_len, p->rf_len + 1);
            // app_halt_system(0x1987, "state:%x pair_st:%x  tag:%x %x peer_tag:%x %x\n", ll_ctx.stk_state, ll_ctx.pair_st, ll_inf.tag, tag, ll_ctx.ktag, peer_tag);
            // tlk_printf("state:%x pair_st:%x  tag:%x %x peer_tag:%x %x\n", ll_ctx.stk_state, ll_ctx.pair_st, ll_inf.tag, tag, ll_ctx.ktag, peer_tag);
        }
    }
    else
    {
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);

        // app_halt_system(0x1988, "state:%x pair_st:%x  tag:%x %x peer_tag:%x %x\n", ll_ctx.stk_state, ll_ctx.pair_st, ll_inf.tag, tag, ll_ctx.ktag, peer_tag);
    }
    #endif /* end of TPSLL_NEW_PAIR_EN #else */
}

_attribute_ram_code_sec_ static void p24g_proc_reconn_rx(rf_packet_t *p)
{
    #if TPSLL_NEW_RECON_EN
/////////////////////////////////////////
    reconn_req_packet_t *pr = (reconn_req_packet_t *)p;

    if (pr->cmd == CMD_RECONN_REQ && pr->tag == ll_inf.tag && (pr->km_tag == ll_ctx.ktag || pr->km_tag == ll_ctx.mtag)) {
        unsigned int r = core_interrupt_disable();

        ll_ctx.last_stick = ll_ctx.stick;
        tpsll_recon_pkt_send();

        ll_ctx.stick += DONGLE_INTERVAL_US * SYSTEM_TIMER_TICK_1US;
        stimer_clr_irq_mask_d25f(FLD_SYSTEM_IRQ_MASK_D25F);
        stimer_set_irq_capture_d25f(ll_ctx.stick);

        if (tpsll_is_report_rate_valid(pr->rr)) {
            ll_ctx.user_rr = pr->rr;
            ll_ctx.chg_rr = ll_ctx.user_rr;
            ll_ctx.pending_rr_fno = ll_ctx.fno_last + TPSLL_RR_DELAY_SWITCH_FNO_N;
            ll_ctx.is_rr_changing = true;
        } else {
            ll_ctx.user_rr = APP_DEFAULT_REPORT_RATE;
        }
        stimer_clr_irq_status_d25f(FLD_SYSTEM_IRQ_D25F);
        stimer_set_irq_mask_d25f(FLD_SYSTEM_IRQ_MASK_D25F);

        core_restore_interrupt(r);

        #if TPSLL_RECON_RSSI_SCAN_EN
        {
            chn_table[0] = reconn_chn_table[0];
            chn_table[1] = reconn_chn_table[1];
            chn_table[2] = reconn_chn_table[2];
            #if TPSLL_8K_CHN_EXPAND_EN
            chn_table[3] = reconn_chn_table[3];
            chn_table[4] = reconn_chn_table[4];
            chn_table[5] = reconn_chn_table[5];
            #else
            #if TPSLL_N8K_CHN_EXPAND_EN
            tpsll_n8k_chn_expand();
            #else
            #if (CHN_TABL_LEN == 4)
            chn_table[3] = chn_table[1];
            #endif
            #endif
            #endif

            tpsll_fhss_recon_update_chn_per();
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
        {
        tlkapi_send_string_data(APP_LOG_EN, "reconn_req", (unsigned char *)pr, pr->dma_len);
        // tlkapi_printf(APP_LOG_EN, "fno(0x%x)\n", ll_ctx.fno_last);
        // tlkapi_printf(APP_LOG_EN, "rr(0x%x-0x%x)\n", pr->rr, ll_ctx.user_rr);
        }

        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP4) {
            ll_ctx.rr_chg_with_recon = RR_RECON_STEP5;
        }
        #endif
    } else {
        tlkapi_send_string_data(APP_LOG_EN, "reconn_req err", (unsigned char *)pr, pr->dma_len);
        tlkapi_printf(APP_LOG_EN, "tag:%x peer_tag:%x\n", ll_inf.tag, pr->tag);
    }
/////////////////////////////////////////
    #else /* end of TPSLL_NEW_RECON_EN */
    reconn_rsp_packet_t *pr = (reconn_rsp_packet_t *)p;
    if (pr->cmd == CMD_RECONN_RSP && pr->d_tag == ll_inf.tag && (pr->km_tag == ll_ctx.ktag || pr->km_tag == ll_ctx.mtag))
    {
        ll_ctx.recon_step = RECON_STEP_NONE;
        pp_timer1_stop();
        timer_clr_irq_status(FLD_TMR1_MODE_IRQ); // clear irq status

        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP4) {
            ll_ctx.rr_chg_with_recon = RR_RECON_STEP5;
        }
        #endif

#if DONGLE_SUSPEND_SUPPORT
        if (s_app_cb && s_app_cb->app_get_usb_status)
        {
            if(s_app_cb->app_get_usb_status() == USB_ENTER_SUSPEND && ll_ctx.suspend_allow)
            {
                app_set_usb_wake_status(USB_ACTIVE_STATE);
                pp_timer1_stop();
                ll_ctx.suspend_allow = 0;
                ll_ctx.stick = ll_ctx.last_stick + (DONGLE_SUSPEND_TIMER1_US * (ll_ctx.task_slot + 1) + 5 + (DONGLE_INTERVAL_US - DONGLE_SUSPEND_TIMER1_US))*SYSTEM_TIMER_TICK_1US;
                ll_ctx.interval_us = DONGLE_INTERVAL_US;

                stimer_set_irq_capture_d25f(ll_ctx.stick);
            }
        }
#endif

        app_d24g_enter_conn();
        // ll_inf.report_rate_customer = pr->rr_customer;
        // ll_inf.report_rate_customer_set_flag = 1;
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);

        if (tpsll_is_report_rate_valid(pr->rr)) {
            ll_ctx.user_rr = pr->rr;
            ll_ctx.chg_rr = ll_ctx.user_rr;
            ll_ctx.pending_rr_fno = ll_ctx.fno_last + TPSLL_RR_DELAY_SWITCH_FNO_N;
            ll_ctx.is_rr_changing = true;
        } else {
            ll_ctx.user_rr = APP_DEFAULT_REPORT_RATE;
        }

        #if TPSLL_RECON_RSSI_SCAN_EN
        // #if TPSLL_RR_CHG_WITH_RECON_EN
        // if (!ll_ctx.rr_chg_with_recon)
        // #endif
        {
            chn_table[0] = pr->chn_tab[0];
            chn_table[1] = pr->chn_tab[1];
            chn_table[2] = pr->chn_tab[2];

            tpsll_fhss_recon_update_chn_per();
        }
        #endif

        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (ll_ctx.user_rr == APP_DEFAULT_REPORT_RATE) {
            ll_ctx.user_rr_last = ll_ctx.user_rr;
            stack_evt_post_from_isr(TPSLL_EVT_REPORT_RATE_CHANGED, (void *)&ll_ctx.report_rate, 1);
        }
        #endif
    }else{
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
        tlkapi_send_string_data(APP_LOG_EN, "reconn_rsp err", (unsigned char *)pr, pr->dma_len);
        tlkapi_printf(APP_LOG_EN, "tag:%x ktag:%x mtag:%x\n", ll_inf.tag, ll_ctx.ktag,ll_ctx.mtag);
    }
    #endif /* end of TPSLL_NEW_RECON_EN #else */
}

_attribute_ram_code_sec_ static void p24g_proc_kb_rx(rf_packet_t *p)
{
    if (p->buf[0] == NORMAL_KB_DATA_CMD)
    {
        if (p->buf[1] == 0xE1 && p->buf[2] == 0xE2)
        {
            return;
        }
        if (p->rf_len != last_usb_report_len || (tmemcmp(last_usb_report_buf, &p->buf[0], p->rf_len) != 0))
        {
            tmemcpy(last_usb_report_buf, &p->buf[0], p->rf_len);

            if (app_rf_normal_key_report_to_usb(&p->buf[1], p->rf_len - 1))
            {
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);

                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
                // tlkapi_send_string_data(APP_LOG_EN, "toUsb", (unsigned char *)&p->buf[1], p->rf_len -1);
            }
            else
            {
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                // tlkapi_send_string_data(APP_LOG_EN, "nk", (unsigned char *)&p->buf[0], p->rf_len);
            }
            tmemset(&last_usb_report_buf[p->rf_len], 0xff, sizeof(last_usb_report_buf) - p->rf_len);
            last_usb_report_len = p->rf_len;
        }

        // return;
    }

    else if (p->buf[0] == CONSUME_KB_DATA_CMD)
    {
        if (p->buf[1] == 0xE1 && p->buf[2] == 0xE2)
        {
            return;
        }
        if (p->rf_len != last_usb_report_len || (tmemcmp(last_usb_report_buf, &p->buf[0], p->rf_len) != 0))
        {
            tmemcpy(last_usb_report_buf, &p->buf[0], p->rf_len);

            // if (app_consume_key_report_to_usb(&p->buf[1]))
            if (s_app_cb && s_app_cb->app_consume_key_report && s_app_cb->app_consume_key_report(&p->buf[1]))
            {
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);

                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
                // tlkapi_send_string_data(APP_LOG_EN, "toUsb", (unsigned char *)&p->buf[1], p->rf_len -1);
            }
            else
            {
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                // tlkapi_send_string_data(APP_LOG_EN, "ck", (unsigned char *)&p->buf[0], p->rf_len);
            }
            tmemset(&last_usb_report_buf[p->rf_len], 0xff, sizeof(last_usb_report_buf) - p->rf_len);
            last_usb_report_len = p->rf_len;
        }
    }

    else if (p->buf[0] == SYSTEM_KB_DATA_CMD)
    {
        if (p->rf_len != last_usb_report_len || (tmemcmp(last_usb_report_buf, &p->buf[0], p->rf_len) != 0))
        {
            if (p->buf[1] == 0xE1 && p->buf[2] == 0xE2)
            {
                return;
            }
            else
            {
                tmemcpy(last_usb_report_buf, &p->buf[0], p->rf_len);
            }


            // if (app_system_key_report_to_usb(&p->buf[1]))
            if (s_app_cb && s_app_cb->app_system_key_report && s_app_cb->app_system_key_report(&p->buf[1]))
            {
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);

                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
                // tlkapi_send_string_data(APP_LOG_EN, "toUsb", (unsigned char *)&p->buf[1], p->rf_len -1);
            }
            else
            {
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                // tlkapi_send_string_data(APP_LOG_EN, "sk", (unsigned char *)&p->buf[0], p->rf_len);
            }
            tmemset(&last_usb_report_buf[p->rf_len], 0xff, sizeof(last_usb_report_buf) - p->rf_len);
            last_usb_report_len = p->rf_len;
        }
    }

    else if (p->buf[0] == ALL_KB_DATA_CMD)
    {
        #if !TPSLL_SPP_NOT_SWITCH_EN
        if((ll_ctx.report_rate & REPORT_RATE_8K) && (ll_ctx.step_3t1r == STEP_3T1R_2T_D_2R))
        {
            if (GET_SN(p->snnesn) == ll_ctx.nesn_8k)//ack
            { //new sn packet, handle only in 2R
                ll_ctx.nesn_8k ^= 1;
                SET_NESN(rf_tx_packet.snnesn, ll_ctx.nesn_8k);
            }
        }
        #endif


        if (tmemcmp(last_all_report_buf, &p->buf[0], p->rf_len) != 0)
        {
            if (p->buf[1] == 0xE1 && p->buf[2] == 0xE2)
            {
                return;
            }
            else
            {
                tmemcpy(last_all_report_buf, &p->buf[0], p->rf_len);
            }


            // if (app_all_key_report_to_usb(&p->buf[1]))
            if (s_app_cb && s_app_cb->app_all_key_report && s_app_cb->app_all_key_report(&p->buf[1]))
            {
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);

                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
                // tlkapi_send_string_data(APP_LOG_EN, "toUsb", (unsigned char *)&p->buf[1], p->rf_len -1);
            }
            else
            {
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                // tlkapi_send_string_data(APP_LOG_EN, "ak", (unsigned char *)&p->buf[0], p->rf_len);
            }
        }
    }
}

_attribute_ram_code_sec_ static void p24g_proc_8k_ms_rx(rf_packet_t *p)
{
    if (ll_ctx.step_3t1r == STEP_3T1R_1T_D_1R)
    {
        // SLOT0 1T D_1R
        {
#if APP_RF_MOUSE_REPORT_EN
            if (p->rf_len != 0)
            {
                store_ms_data2_buffer(p);
// report_mouse_data();
#if 0
                         //pkt1
                         if (g_ms_need_report[ms_idx] == 0) {
                             tmemcpy(&mouse_rx[(ms_idx)].btn, &p->buf[0], 6);
                             g_ms_need_report[ms_idx] = 1;
                         }
                         //pkt2
                         if (g_ms_need_report[(ms_idx + 1) & 3] == 0) {
                             tmemcpy(&mouse_rx[(ms_idx + 1) & 3].btn, &p->buf[6], 6);
                             g_ms_need_report[(ms_idx + 1) & 3] = 1;
                         }
#endif
            }

#else
//                         app_usb_mouse_report_test();
#endif
            curr_cnt_to_usb++;
            cnt_rx_d_ok++;
        }
        // ll_ctx.step_3t1r = STEP_3T1R_2T_D_2R;
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_2T_D_2R)
    {
        ms_rp_idx = (ms_idx - 1) & 3;
// SLOT1 2T D_2R
#if APP_RF_MOUSE_REPORT_EN
        if (p->rf_len != 0)
        {
            store_ms_data2_buffer(p);
            report_mouse_data();
        }
#endif

        curr_cnt_to_usb = 2;
        cnt_rx_d_ok++;
        // ll_ctx.step_3t1r = STEP_3T1R_3T_D_3R;
    }
    else if (ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)
    {
        ms_rp_idx = (ms_idx) & 3;
        // SLOT2 3T D_3R
        {
            // if (curr_cnt_to_usb != 2)
            // {
            //     curr_cnt_to_usb++;
            // }
#if APP_RF_MOUSE_REPORT_EN
            if (p->rf_len != 0)
            {
                
                store_ms_data2_buffer(p);
                
                

                report_mouse_data();
            }

#else
            app_usb_mouse_report_test();
#endif
            cnt_rx_d_ok++;
        }

        // ll_ctx.step_3t1r = STEP_3T1R_1R_D_1T;
    }
}

_attribute_ram_code_sec_ static void p24g_proc_ll_rx(rf_packet_t *p)
{
    // tlkapi_send_string_data(APP_LOG_EN, "ll_rx", &p->dma_len, p->rf_len + 6);
    if (p->buf[0] == TPSLL_LL_CMD_TERMINATE_CONN_REQ)
    {
//         pp_fifo_push(&LLTxFifo, TPSLL_LL_CMD_TERMINATE_CONN_RSP, 0, 0);
//         uint8_t reason = TPSLL_LL_CONN_TERMINATION_BY_PEER;
//         p24g_send_sm_msg(P24G_SM_CMD_SET_STATE, STATE_DISCONNECTED, &reason, 1);

// #if defined(MCU_CORE_TL322X_N22)
//         clic_interrupt_disable(IRQ_SYSTIMER);
// #else
//         plic_interrupt_disable(IRQ_SYSTIMER);
// #endif
    } else if (p->buf[0] == TPSLL_LL_CMD_RR_CHG_REQ){
        ll_ctx.chg_rr = p->buf[1];
        ll_ctx.pending_rr_fno = p->buf[2] | (p->buf[3] << 8);
        ll_ctx.is_rr_changing = true;
        ll_ctx.pending_spp_mode = p->buf[4];
        #if TPSLL_FHSS_EN
        g_hop_map_next = g_hop_map_cur;
        #endif
        // pp_fifo_push(&ll_fifo, TPSLL_LL_CMD_RR_CHG_RSP, 0, 0);
    } else if (p->buf[0] == TPSLL_LL_CMD_RR_CHG_RCNN_REQ) {
        #if TPSLL_RR_CHG_WITH_RECON_EN
        #if TPSLL_RR_CHG_WITH_RECON_NO_RSP_EN
        ll_ctx.rr_chg_with_recon = RR_RECON_STEP2;
        #else
        uint8_t cmd[3] = {TPSLL_LL_CMD_RR_CHG_RCNN_RSP, 0, 0};
        pp_fifo_push(&ll_fifo, TPSLL_LL_CMD, cmd, 1);

        ll_ctx.rr_chg_with_recon = RR_RECON_STEP1;
        #endif

        ll_ctx.pending_rr_fno = p->buf[2] | (p->buf[3] << 8);
        ll_ctx.is_rr_changing = true;
        ll_ctx.pending_spp_mode = p->buf[4];
        #if TPSLL_FHSS_EN
        g_hop_map_next = g_hop_map_cur;
        #endif
        #endif
    }
    #if TPSLL_8K_WHEEL_SEND_IN_LL_EN
    else if (p->buf[0] == TPSLL_LL_STK_CMD_WHEEL_DATA) {
        ll_ctx.z_wheel_data = p->buf[1];
        ll_ctx.t_wheel_data = p->buf[2];
    }
    #endif
}

_attribute_ram_code_sec_ static void tpsll_proc_spp_rx(rf_packet_t *p)
{
    // tlkapi_send_string_data(APP_LOG_EN, "spp_rx", &p->dma_len, p->rf_len + 6);
    // p24g_evt_cb(TPSLL_EVT_SPP_DATA_RECV, (void *)p->buf, p->rf_len);

    stack_evt_post_from_isr(TPSLL_EVT_SPP_DATA_RECV, (void *)p->buf, p->rf_len);

    // ZW_TG(GPIO_PH0);
}

// #if TPSLL_SPP_NOT_SWITCH_EN
// _attribute_ram_code_sec_ static void tpsll_proc_ll_spp_rx_ack(rf_packet_t *p)
// {
//     uint8_t sn = GET_SN(p->snnesn);
//     uint8_t nesn = GET_NESN(p->snnesn);
//     // tlkapi_send_string_data(APP_LOG_EN, "rx", &p->dma_len, 8);
//     // ZW_TG(GPIO_PG5);

//     #if 1
//     if(sn)
//     {
//         ZW_TG(GPIO_PH0);
//     }else{
//         ZW_TG(GPIO_PH0);
//         ZW_TG(GPIO_PH0);
//     }

//     if(nesn)
//     {
//         ZW_TG(GPIO_PG6);
//     }else{
//         ZW_TG(GPIO_PG6);
//         ZW_TG(GPIO_PG6);
//     }
//     #endif

//     if (sn != ll_ctx.rx_nesn)
//     { // old packet
//         // ZW_TG(GPIO_PH0);
//         return;
//     }

//     ll_ctx.rx_nesn ^= 1;
//     SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);
//     ZW_TG(GPIO_PG7);

// DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX4], nesn);
// DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
// DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX5], ll_ctx.tx_sn);
//     if (nesn != ll_ctx.tx_sn) //tx packet acked
//     {
//         ll_ctx.tx_sn ^= 1;
//         SET_SN(rf_tx_packet.snnesn, ll_ctx.tx_sn);
// DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
// DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX5], ll_ctx.tx_sn);

//         ZW_TG(GPIO_PG7);

//         ll_ctx.last_pkt_acked = true;

//         //Q|TODO: special data
//         // if (ll_ctx.ul_data_sending == DONGLE_UL_DATA_SENDING)
//         // {
//         //     uint8_t *event = pp_fifo_get_ptr(&ul_fifo);

//         //     // TODO notify to app layer tx success
//         //     stack_evt_post_from_isr(TPSLL_EVT_USR_SPEC_ACK_RECV, (void *)&event[2], event[0]);

//         //     pp_fifo_pop(&ul_fifo);
//         //     ll_ctx.ul_data_sending = DONGLE_UL_DATA_IDLE;
//         // }
//         if (ll_ctx.ll_flag)
//         {
//             uint8_t *ptr = pp_fifo_get_ptr(&ll_fifo);

//             if (ptr) {
//                 pp_fifo_pop(&ll_fifo);
//             }
//             ll_ctx.ll_flag = false;

//             #if TPSLL_RR_CHG_WITH_RECON_EN
//             if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP2)
//             {
//                 ll_ctx.rr_chg_with_recon = RR_RECON_STEP3;
//                 ll_ctx.rr_chg_recon_time_tick = clock_time() | 1;

//                 tpsll_connect_lost();
//             }
//             #endif
//         }

//         if (ll_ctx.spp_flag) // spp mode only one slot
//         {
//             uint8_t *ptr = pp_fifo_get_ptr(&spp_fifo);
//             // uint8_t len = ptr[0];

//             if (ptr) {
//                 pp_fifo_pop(&spp_fifo);
//                 stack_evt_post_from_isr(TPSLL_INTER_EVT_SPP_SEND_COMP, (void *)&ptr[2], ptr[0] + 9);
//                 DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2] | dbg_io[DBG_IO_IDX3]);
//             }

//             ll_ctx.spp_flag = false;
//             ZW_TG(GPIO_PG7);
//         }

//         // if(ll_ctx.ul_data_sending==DONGLE_UL_DATA_SENDING && !ll_ctx.spp_flag)
//         // {
//         //     uint8_t *event = pp_fifo_get_ptr(&ul_fifo);

//         //     //TODO notify to app layer tx success
//         //     stack_evt_post_from_isr(TPSLL_EVT_USR_SPEC_ACK_RECV, (void *)&event[2], event[0]);

//         //     pp_fifo_pop(&ul_fifo);

//         //     ll_ctx.ul_data_sending = DONGLE_UL_DATA_IDLE;
//         // }

//         if (pp_fifo_get_ptr(&spp_fifo) && ll_ctx.spp_flag) // spp mode only one slot
//         {
//             uint8_t *ptr = pp_fifo_get_ptr(&spp_fifo);
//             // uint8_t len = ptr[0];

//             stack_evt_post_from_isr(TPSLL_INTER_EVT_SPP_SEND_COMP, (void *)&ptr[2], ptr[0] + 9);

//             pp_fifo_pop(&spp_fifo);
//             ll_ctx.spp_flag = false;
//         }
//     }

//     if (p->pkt_type == P24G_PKT_TYPE_SPP) {
//         stack_evt_post_from_isr(TPSLL_EVT_SPP_DATA_RECV, (void *)p->buf, p->rf_len);
//     }
//     else if(p->pkt_type == P24G_PKT_TYPE_LL) {
//         p24g_proc_ll_rx(p);
//     }
// }
// #endif

_attribute_ram_code_sec_ void tpsll_wfi_en(bool en)
{
    ll_ctx.wfi_en = en;
}

/**
 * @brief       Clear idle state and reset fast settle calibration flag
 * @param[in]   p   rf_packet_t pointer
 * @return      None
 */
_attribute_ram_code_sec_ static void tpsll_reset_idle_and_calib_check(rf_packet_t *p)
{
    if (p->rf_len != 0)
    {
        // Reset idle status
        ll_ctx.idle_tick = stimer_get_tick();
        ll_ctx.idle_flag = 0;

        // Disable fast settle calibration when receive keep-alive packet
#if TPSLL_FAST_SETTLE_CAL_FUN_EN
        ll_ctx.fast_settle_cal_en = 0;
    }

    if (ll_ctx.idle_flag && (p->rf_len == 0))
    {
        ll_ctx.fast_settle_cal_tick = stimer_get_tick();
        ll_ctx.fast_settle_cal_en = 0;
        // gpio_toggle(GPIO_PG7);
#endif
    }
}

_attribute_ram_code_sec_ static void p24g_proc_8k_conn_rx(rf_packet_t *p)
{
#if 0
    static uint32_t pcntt = 0;
    // if (pcntt < 20)//pcntt never lower than 0
    if (pcntt < 20 && (p->buf[7] != 0 || (p->rf_len == 0x18 &&p->buf[19] != 0)))//pcntt never lower than 0
    {
        pcntt++;
        // tlkapi_send_string_data(APP_LOG_EN, "rx", &p->rf_len, p->rf_len + 1);
        // tlkapi_send_string_data(APP_LOG_EN, "rx", &p->dma_len, p->rf_len + 5);
        // ZW_TG(GPIO_PG7);
        //ll_ctx.zw_cnt++;
       
    }
    
#endif
    // #if TPSLL_SPP_NOT_SWITCH_EN
    // if((ll_ctx.step_3t1r == STEP_3T1R_2T_D_2R) && (ll_ctx.spp_flag || (p->pkt_type == P24G_PKT_TYPE_SPP) || ll_ctx.ll_flag || (p->pkt_type == P24G_PKT_TYPE_LL)))
    // {
    //     DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2] | dbg_io[DBG_IO_IDX3]);
    //     tpsll_proc_ll_spp_rx_ack(p);
    //     app_d24g_change_step();
    //     ll_ctx.lost_cnt = 0;
    //     return;
    // }
    // #endif

    ll_ctx.ack_mask |= (1 << ll_ctx.step_3t1r);
    // if ((ll_ctx.step_3t1r != STEP_3T1R_2T_D_2R) || ((p->pkt_type != P24G_PKT_TYPE_SPP) && (p->pkt_type != P24G_PKT_TYPE_LL)))
    if ((p->pkt_type < P24G_PKT_TYPE_SPP) || (ll_ctx.step_3t1r != STEP_3T1R_2T_D_2R))
    {
        if(ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R)//Note: 3R irq after Tx, 11us away from TX_ON
        {

    #if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
            // if (ll_ctx.ack_mask & 6)
            {
                rf_tx_packet.s_n = !p->s_n;
            }
            ll_ctx.last_nesn = !p->s_n;

            
            ll_ctx.intv_pkt_st = TPSLL_PKT_ALL_RCVD;

            if (p->s_n != ll_ctx.last_sn)//Separate execution is for the timing of packet sending
            {
                ll_ctx.acc.x = 0;
                ll_ctx.acc.y = 0;
                ll_ctx.last_acc.x = 0;
                ll_ctx.last_acc.y = 0;
            }

    #else
            // rf_tx_packet.buf[0] = (rf_tx_packet.buf[0] & 0xF0) | (ll_ctx.ack_mask & 0x7);
    #endif

            rf_tx_packet.buf[0] = (rf_tx_packet.buf[0] & 0xF8) | (ll_ctx.ack_mask & 0x7);
        }

        #if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        else
        {
            // if (p->s_n != ll_ctx.last_sn && ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_RCVD)//Separate execution is for the timing of packet sending
            if (p->s_n != ll_ctx.last_sn)//Separate execution is for the timing of packet sending
            {
                ll_ctx.acc.x = 0;
                ll_ctx.acc.y = 0;
                ll_ctx.last_acc.x = 0;
                ll_ctx.last_acc.y = 0;
            }

            if (ll_ctx.step_3t1r == STEP_3T1R_1T_D_1R)
            {
                ll_ctx.intv_pkt_st = TPSLL_PKT_2_LOSS;
                ll_ctx.last_nesn = p->s_n;
            }
            else if (ll_ctx.step_3t1r == STEP_3T1R_2T_D_2R)
            {
                rf_tx_packet.s_n = !p->s_n;
                ll_ctx.last_nesn = !p->s_n;
                ll_ctx.intv_pkt_st = TPSLL_PKT_ALL_RCVD;
            }
        }

        ll_ctx.last_sn = p->s_n;
        #endif

        #if AFH_ENABLE
        update_channel_quality(chn_list[ll_ctx.step_3t1r], true);
        #endif


        #if !TPSLL_SPP_NOT_SWITCH_EN
        // ll_ctx.last_pkt_acked = false;
        if (GET_NESN(p->snnesn) == ll_ctx.sn_8k)
        {
            ll_ctx.sn_8k ^= 1;
            SET_SN(rf_tx_packet.snnesn, ll_ctx.sn_8k);

            if(ll_ctx.ll_flag)
            {
                pp_fifo_pop(&ll_fifo);
                ll_ctx.ll_flag = false;

                #if TPSLL_RR_CHG_WITH_RECON_EN && !TPSLL_RR_CHG_WITH_RECON_NO_RSP_EN
                if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP2) {
                    ll_ctx.rr_chg_with_recon = RR_RECON_STEP3;
                    ll_ctx.rr_chg_recon_time_tick = clock_time() | 1;

                    tpsll_connect_lost();
                }
                #endif
            }
        }
        #endif

        // if(p->rf_len == 3)
        // {
        //  app_halt_system(0x100d, "rx len:%x %x %x %x %x\n", p->rf_len, p->buf[0], p->buf[1], p->buf[2], p->buf[3]);
        // }
        if(p->pkt_type == P24G_PKT_TYPE_KB)
        {
            p24g_proc_kb_rx(p);
            // app_d24g_change_step();
        } else if((p->pkt_type == P24G_PKT_TYPE_MS) && (p->rf_len >= 5) && !ll_ctx.is_rr_changing)
        {
            #if !TPSLL_SPP_NOT_SWITCH_EN //Q|TODO: 8k wheel data !!!
            if (ll_ctx.step_3t1r == STEP_3T1R_1T_D_1R)
            {
            // DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX4], ll_ctx.nesn_8k);
            // DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX3], GET_SN(p->snnesn));
                if (GET_SN(p->snnesn) == ll_ctx.nesn_8k) { //new sn packet, handle only in 1R
                    ll_ctx.nesn_8k ^= 1;
                    SET_NESN(rf_tx_packet.snnesn, ll_ctx.nesn_8k);

                    #if TPSLL_WHEEL_TEST_EN
                    ll_ctx.z_wheel_data = g_wheel_test;
                    g_wheel_test = 0;
                    // g_wheel_test = -g_wheel_test;
                    #else
                    ll_ctx.z_wheel_data = (p->buf[5] >> 4) & 0x0F;  // Get z_wheel (high 4-bit of buf[5])
                    ll_ctx.t_wheel_data = p->buf[5] & 0x0F;         // Get t_wheel (low 4-bit of buf[5])
                    (ll_ctx.z_wheel_data & 0x08) ? (ll_ctx.z_wheel_data |= 0xF0) : (ll_ctx.z_wheel_data &= 0x0F); //Sign-extend z_wheel
                    (ll_ctx.t_wheel_data & 0x08) ? (ll_ctx.t_wheel_data |= 0xF0) : (ll_ctx.t_wheel_data &= 0x0F); //Sign-extend t_wheel
                    #endif
                    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
                    if (ll_ctx.z_wheel_data || ll_ctx.t_wheel_data) {
                    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
                    }
                }
            // DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX4], ll_ctx.nesn_8k);
            }
            #if TPSLL_FHSS_EN
            if (ll_ctx.stk_state == STATE_CONNECTED)
            {
                tpsll_channel_statistic(chn_list[ll_ctx.step_3t1r], false);
            }
            #endif
            #endif

            p24g_proc_8k_ms_rx(p);

            #if (ABNORMAL_RETRY_FIX)
            last_data_type = P24G_PKT_TYPE_MS;
            #endif
        }
        #if !TPSLL_SPP_NOT_SWITCH_EN
        else if(p->pkt_type == P24G_PKT_TYPE_LL)
        {
            p24g_proc_ll_rx(p);
            //app_d24g_change_step();
        }
        #endif
        else{
            //app_d24g_change_step();
        }
        // app_d24g_change_step();

        #if (ABNORMAL_RETRY_FIX)
        if (p->pkt_type != P24G_PKT_TYPE_MS)
        {
            if (last_data_type == P24G_PKT_TYPE_MS) {
                last_data_type = 0;
                g_ms_need_report[0] = 0;
                g_ms_need_report[1] = 0;
                g_ms_need_report[2] = 0;
                g_ms_need_report[3] = 0;
            }
        }
        #endif

        ll_ctx.lost_cnt = 0;

        #if 0// TPSLL_8K_PKT_LOSS_SNNESN_OP_EN

        if (ll_ctx.last_intv_pkt_st == TPSLL_PKT_2_LOSS)
        {

            ZW_TG(GPIO_PH4);
        }
        else if (ll_ctx.last_intv_pkt_st == TPSLL_PKT_ALL_LOSS)
        {

            ZW_TG(GPIO_PH4);
            ZW_TG(GPIO_PH4);
        }
        #endif
    }

//////////////////////////////////////////////////////////////////////////////
    #if TPSLL_SPP_NOT_SWITCH_EN
    if(ll_ctx.step_3t1r == STEP_3T1R_2T_D_2R)
    {
        do {
            uint8_t sn = GET_SN(p->snnesn);
            uint8_t nesn = GET_NESN(p->snnesn);
            // tlkapi_send_string_data(APP_LOG_EN, "rx", &p->dma_len, 8);
            // ZW_TG(GPIO_PG5);

            #if 0
            if(sn)
            {
                ZW_TG(GPIO_PH0);
            }else{
                ZW_TG(GPIO_PH0);
                ZW_TG(GPIO_PH0);
            }

            if(nesn)
            {
                ZW_TG(GPIO_PG6);
            }else{
                ZW_TG(GPIO_PG6);
                ZW_TG(GPIO_PG6);
            }
            #endif

        // DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX4], nesn);
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
        // DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX5], ll_ctx.tx_sn);
            if (nesn != ll_ctx.tx_sn) //tx packet acked
            {
                ll_ctx.tx_sn ^= 1;
                SET_SN(rf_tx_packet.snnesn, ll_ctx.tx_sn);
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
        // DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX5], ll_ctx.tx_sn);

                ll_ctx.last_pkt_acked = true;

                if (ll_ctx.ll_flag)
                {
                    uint8_t *ptr = pp_fifo_get_ptr(&ll_fifo);

                    ll_ctx.ll_flag = false;
                    if (ptr) {
                        if (ptr[2] == TPSLL_LL_CMD_UL_DATA_SEND_REQ)
                        {
                            stack_evt_post_from_isr(TPSLL_EVT_USR_SPEC_ACK_RECV, (void *)&ptr[3], ptr[0] - 1);
                        }
                        pp_fifo_pop(&ll_fifo);
                    }

                    #if TPSLL_RR_CHG_WITH_RECON_EN && !TPSLL_RR_CHG_WITH_RECON_NO_RSP_EN
                    if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP2)
                    {
                        ll_ctx.rr_chg_with_recon = RR_RECON_STEP3;
                        ll_ctx.rr_chg_recon_time_tick = clock_time() | 1;

                        tpsll_connect_lost();
                    }
                    #endif
                }

                if (ll_ctx.spp_flag) // spp mode only one slot
                {
                    uint8_t *ptr = pp_fifo_get_ptr(&spp_fifo);
                    // uint8_t len = ptr[0];

                    ll_ctx.spp_flag = false;
                    if (ptr) {
                        pp_fifo_pop(&spp_fifo);
                        stack_evt_post_from_isr(TPSLL_INTER_EVT_SPP_SEND_COMP, (void *)&ptr[2], ptr[0] + 9);
                        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2] | dbg_io[DBG_IO_IDX3]);
                    }

                    //ZW_TG(GPIO_PG7);
                }
            }

        // DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX4], sn);
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
        // DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX5], ll_ctx.rx_nesn);
            if (ll_inf.connecting_dev == DEVICE_TYPE_MS)
            {
                if ((p->pkt_type < P24G_PKT_TYPE_SPP) || (sn != ll_ctx.rx_nesn))
                { // mouse invalid packet
                    break;
                }
            }
            else
            {
                if ((p->rf_len == 0) || (sn != ll_ctx.rx_nesn) || (p->pkt_type == P24G_PKT_TYPE_MS) || ((p->pkt_type == P24G_PKT_TYPE_KB) && (p->buf[0] != ALL_KB_DATA_CMD)))
                { // keyboard invalid packet
                    break;
                }
            }
            ll_ctx.rx_nesn ^= 1;
            SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);

        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
        // DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX5], ll_ctx.rx_nesn);

            if (p->pkt_type == P24G_PKT_TYPE_SPP) {
                stack_evt_post_from_isr(TPSLL_EVT_SPP_DATA_RECV, (void *)p->buf, p->rf_len);
            }
            else if(p->pkt_type == P24G_PKT_TYPE_LL) {
                p24g_proc_ll_rx(p);
            }
        } while(0);
    }

    #endif
//////////////////////////////////////////////////////////////////////////////
    tpsll_reset_idle_and_calib_check(p);
    app_d24g_change_step();

    #if TPSLL_KB_ACTIVE_PM_OPTM_EN
    if ((ll_inf.paired_dev_type & DEVICE_TYPE_KB) && p->rf_len) {
        ll_ctx.kb_alive_intervals_cnt = 0;
        ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
    }
    #endif
}

_attribute_ram_code_sec_ static void rf_irq_proc_8k_rx(void)
{
        RF_RX_GPIO_DEBUG_SET(0);
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX1]);

        unsigned char *rx_buf = rf_get_rx_packet_addr(RX_FIFO_NUM, RX_FIFO_DEP, rx_packet);
        rf_packet_t  *p=(rf_packet_t *)rx_buf;

        if (!(reg_rf_dec_err & 0x10) && ((uint32_t)(p->dma_len) == (uint32_t)RF_GET_DMA_LEN(p->rf_len)))
        {
//           rf_tx_packet.snnesn=p->snnesn;
             if (ll_ctx.stk_state == STATE_CONNECTED)
             {
                p24g_proc_8k_conn_rx(p);
                #if STIMER_SOF_SYNC_WITH_KMD_SYNCED_EN
                ll_ctx.synced = true;
                #endif

                #if TPSLL_KEEP_ALIVE_NOTIFY_EN
                #if TPSLL_KEEP_ALIVE_PER_EN && !TPSLL_KB_KEEP_ALIVE_PER_EN
                if (!ll_ctx.is_keep_alive && p->n_tab) {
                    tmemset(g_fhss_chn_per, TPSLL_FHSS_CHN_PER_GOOD_TH - 1, sizeof(g_fhss_chn_per));
                }
                #endif
                ll_ctx.is_keep_alive = p->n_tab;
                if (ll_ctx.is_keep_alive) {
                    ll_ctx.keep_alive_refer_tick = clock_time() | 1;
                    #if TPSLL_RF_SCAN_EN
                    if (!rf_scan_ctx.done_tick) {
                        rf_scan_ctx.done_tick = ll_ctx.keep_alive_refer_tick;
                        rf_scan_ctx.state = RF_SCAN_STATE_IDLE;
                    }
                    #endif
                    #if TPSLL_FHSS_REPORT_PER_EN
                    ll_ctx.report_pkt_cnt = 0;
                    ll_ctx.report_per = 0;
                    #endif
                    #if TPSLL_NEW_KEEP_ALIVE_EN
                    ll_ctx.keep_alive_expect_fno = (ll_inf.paired_dev_type & DEVICE_TYPE_MS) ? ll_ctx.fno : ll_ctx.fno_last + (TPSLL_8K_KEEP_ALIVE_SYNC_IN_US / DONGLE_INTERVAL_US);
                    #endif
                    #if TPSLL_KEEP_ALIVE2_NOTIFY_EN
                    if (!ll_ctx.is_keep_alive2_active && (p->buf[0] == 0xA5)) {
                        ll_ctx.is_keep_alive2_active = true;
                    }
                    #endif
                } else {
                    ll_ctx.keep_alive_refer_tick = 0;
                    #if TPSLL_RF_SCAN_EN
                    rf_scan_ctx.done_tick = 0;
                    #endif
                    #if TPSLL_KEEP_ALIVE2_NOTIFY_EN
                    ll_ctx.is_keep_alive2_active = false;
                    #endif
                }
                #endif /* end of TPSLL_KEEP_ALIVE_NOTIFY_EN */
             } else if (ll_ctx.stk_state == STATE_RECONNECT)
             {
                p24g_proc_reconn_rx(p);
             }
             else if (ll_ctx.stk_state == STATE_PAIRING) {
                p24g_proc_8k_paring_rx(p);
                //  tlkapi_send_string_data(APP_LOG_EN, "p rx", &p->rf_len, p->rf_len + 1);
             } 
             DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX1]);
             cnt_rx_crc_ok++;
        }
        else
        {
            cnt_rx_crc_err++;
            RF_RX_CRC_GPIO_DEBUG_TOGGLE;
            if (ll_ctx.stk_state == STATE_CONNECTED) {
                app_rf_8k_abnormal_handle();
            }
        }

        rx_packet[5] = 0;   //Clear length to filter invalid packets in the next slot.

        cnt_rx_irq++;
}

_attribute_ram_code_sec_ void app_rf_irq_8k_handler(void)
{
    
    if(rf_get_irq_status(FLD_RF_IRQ_RX))
    {
        rf_status = IDLE_RF_STATUS;
        rf_clr_irq_status(FLD_RF_IRQ_RX);
        rf_irq_proc_8k_rx();
        ll_ctx.rx_over = true;
    }

    if(rf_get_irq_status(FLD_RF_IRQ_FIRST_TIMEOUT))
    {
        rf_clr_irq_status(FLD_RF_IRQ_FIRST_TIMEOUT);
        RF_RX_TIMEOUT_GPIO_DEBUG_TOGGLE;
        RF_RX_GPIO_DEBUG_SET(0);

        if (ll_ctx.stk_state == STATE_CONNECTED) {
            #if TPSLL_RF_SCAN_EN
            if (rf_scan_ctx.state & RF_SCAN_STATE_RUNNING) {
                tpsll_rssi_scan();
                // tpsll_rssi_scan_check();
            } else
            #endif
            {
                #if TPSLL_AVOID_WIFI_CHN_USE_RX3_EN
                uint8_t check_rssi = (ll_ctx.do_rssi_scan && ll_ctx.step_3t1r == STEP_3T1R_3T_D_3R);
                #endif

                app_rf_8k_abnormal_handle();

                #if TPSLL_AVOID_WIFI_CHN_USE_RX3_EN
                if (check_rssi) {
                    tpsll_rx3_rssi_scan();
                    // tpsll_rssi_scan_check();
                }
                #endif
            }
        } else
        #if TPSLL_RECON_RSSI_SCAN_EN && TPSLL_NEW_RECON_EN
        #if TPSLL_RF_SCAN_EN
        if (rf_scan_ctx.state & RF_SCAN_STATE_RUNNING) {
            tpsll_rssi_scan();
        }
        #else
        if(scan_state == STATE_SCAN_RUNNING) {
            proc_rssi_scan();
        }
        #endif
        else if(!ll_ctx.recon_rssi_scan_step && ((ll_ctx.stk_state == STATE_PAIRING) || (ll_ctx.stk_state == STATE_RECONNECT))) {
            ll_ctx.pr_cnt = 0;
        }//else
        #else
    #if RSSI_SCAN_EN
        if(scan_state == STATE_SCAN_RUNNING)
        {
            proc_rssi_scan();
        }//else
    #endif
        #endif
        // {
        //     app_rf_8k_abnormal_handle();
        // }
        ll_ctx.rx_over = true;
    }

    if(rf_get_irq_status(FLD_RF_IRQ_RX_TIMEOUT))
    {
        // rf_rx_fail_cnt++;
        rf_clr_irq_status(FLD_RF_IRQ_RX_TIMEOUT);
        //print_app_rf("rx timeout\r\n");
        rf_status=IDLE_RF_STATUS;

        RF_RX_TIMEOUT_GPIO_DEBUG_TOGGLE;

        if (ll_ctx.stk_state == STATE_CONNECTED) {
            app_rf_8k_abnormal_handle();
        }

        ll_ctx.rx_over = true;
    }

    if(rf_get_irq_status(FLD_RF_IRQ_TX))
    {
        cnt_tx_irq++;
        rf_clr_irq_status(FLD_RF_IRQ_TX);
        RF_TX_GPIO_DEBUG_SET(0);

        #if TPSLL_KEEP_ALIVE_NOTIFY_EN
        if (ll_ctx.is_keep_alive && (ll_inf.paired_dev_type & DEVICE_TYPE_MS) && ll_ctx.keep_alive_refer_tick && clock_time_exceed(ll_ctx.keep_alive_refer_tick, TPSLL_RF_SCAN_START_TIME_US)) {
            ll_ctx.keep_alive_refer_tick = 0;
            #if TPSLL_RF_SCAN_EN
            if (pp_fifo_is_empty(&ll_fifo) && pp_fifo_is_empty(&spp_fifo) && (rf_scan_ctx.state & RF_SCAN_STATE_IDLE)
                && rf_scan_ctx.done_tick && clock_time_exceed(rf_scan_ctx.done_tick, TPSLL_RF_SCAN_IDLE_MIN_TIME_US)) {
                rf_scan_ctx.state = RF_SCAN_STATE_RUNNING;
                rf_scan_ctx.start_tick = clock_time() | 1;
                tpsll_rf_scan_start();
            }
            else if (pp_fifo_is_empty(&ll_fifo) && pp_fifo_is_empty(&spp_fifo) && (rf_scan_ctx.state & RF_SCAN_STATE_PAUSE)) {
                rf_scan_ctx.state = RF_SCAN_STATE_RUNNING;
                rf_scan_ctx.start_tick = clock_time() | 1;
                tpsll_rf_scan_cotinue();
            }
            #endif
        } else
        #endif /* end of TPSLL_KEEP_ALIVE_NOTIFY_EN */
        #if TPSLL_NEW_RECON_EN
        if ((ll_ctx.stk_state == STATE_RECONNECT) && ll_ctx.enter_conn_flag) {
            if (ll_ctx.recon_step < RECON_STEP4) {
                #if !TPSLL_RECON_RSSI_SCAN_EN
                rf_set_ble_chn((ll_ctx.reconn_chn + (ll_ctx.recon_step - RECON_STEP1) * 13) & 0x3F);
                #endif
                tpsll_recon_pkt_send();
                ll_ctx.recon_step++;
                return;
            } else {
                ll_ctx.enter_conn_flag = false;

                #if DONGLE_SUSPEND_SUPPORT
                if (s_app_cb && s_app_cb->app_get_usb_status && s_app_cb->app_set_usb_status)
                {
                    if(s_app_cb->app_get_usb_status() == USB_ENTER_SUSPEND && ll_ctx.suspend_allow)
                    {
                        s_app_cb->app_set_usb_status(USB_ACTIVE_STATE);
                        pp_timer1_stop();
                        ll_ctx.suspend_allow = 0;
                        ll_ctx.suspend_enable = 0;
                    }
                }
                #endif
                app_d24g_enter_conn();
                #if TPSLL_RR_CHG_WITH_RECON_EN
                if (ll_ctx.user_rr == APP_DEFAULT_REPORT_RATE) {
                    ll_ctx.user_rr_last = ll_ctx.user_rr;
                    if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP6) {
                        ll_ctx.rr_chg_with_recon = RR_RECON_STEP_NONE;
                        ll_ctx.rr_chg_recon_time_tick = 0;
                    }
                    stack_evt_post_from_isr(TPSLL_EVT_REPORT_RATE_CHANGED, (void *)&ll_ctx.report_rate, 1);
                }
                #endif
                return;
            }
        }
        #endif
        #if TPSLL_NEW_PAIR_EN
        else if ((ll_ctx.stk_state == STATE_PAIRING) && (ll_ctx.pair_st == STATE_PAIR_CFM)) {
            if (ll_ctx.pair_step < PAIR_STEP4) {
                rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, rf_stimer_get_tick() + 30 * SYSTEM_TIMER_TICK_1US);
                ll_ctx.pair_step++;
            } else {
                #if DONGLE_SUSPEND_SUPPORT
                if(app_get_usb_wake_status() == USB_ENTER_SUSPEND && ll_ctx.suspend_allow)
                {
                    app_set_usb_wake_status(USB_ACTIVE_STATE);
                    pp_timer1_stop();
                    ll_ctx.suspend_allow = 0;
                    ll_ctx.suspend_enable = 0;
                }
                #endif
                ll_ctx.pair_st = STATE_PAIR_NONE;
                ll_ctx.pair_step = PAIR_STEP_NONE;
                ll_ctx.enter_conn_flag = false;
                app_d24g_enter_conn();
            }
            return;
        }
        #endif

    #if 0
        // pkt2 to usb
        if (g_ms_need_report[(ms_idx + 1) & 3])
        {
            if (!app_mouse_usb_ep_buf_has_data()) {
                if (g_local_data_en) {
                    unsigned char tmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
                    t_mouse_data *ms = (t_mouse_data *)tmp;
                    Draw_a_square_test(&ms->x, &ms->y);
                    if (app_mouse_report_data(&tmp[1]) != 0)
                    { // fail
                        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                        // app_rf_buf_print((unsigned char *)mouse_rx, 16);
                        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
                    }
                    else
                    {
                        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                    }
                } else {
                    if (app_mouse_report_data(&mouse_rx[(ms_idx + 1) & 3].btn) != 0)
                    { // fail
                        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                        // app_rf_buf_print((unsigned char *)mouse_rx, 16);
                        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
                    }
                    else
                    {
                        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                    }
                }
            }
            g_ms_need_report[(ms_idx + 1) & 3] = 0;
        }
        #endif

        if (ll_ctx.stk_state == STATE_CONNECTED)
        {
            check_report_mode_by_per();
        }

        cnt_to_usb += curr_cnt_to_usb;
        curr_cnt_to_usb = 0;

    }
    ll_ctx.wfi_flag = true;
}


_attribute_ram_code_sec_ static void p24g_proc_1k_ms_rx(rf_packet_t *p)
{
    // if (!app_mouse_usb_ep_buf_has_data())
    {
        // pkt1 to usb
        if (g_local_data_en)
        {
            unsigned char tmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            //t_mouse_data *ms = (t_mouse_data *)tmp;
            // Draw_a_square_test(&ms->x, &ms->y);
            if (app_mouse_report_data(&tmp[1]) != 0)
            { // fail
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                // // app_rf_buf_print((unsigned char *)mouse_rx, 16);
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
            }
            else
            {
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
            }
        }
        else
        {
            #if 1
            if(p->rf_len < 5)
            {
                return;
            }

            //will broken timing
            // tlkapi_send_string_data(APP_LOG_EN, "rx_1k", &p->dma_len, p->rf_len + 6);

            if (app_mouse_report_data(&p->buf[0]) != 0)
            { // fail
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
                // // app_rf_buf_print((unsigned char *)mouse_rx, 16);
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
            }
            else
            {
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX6]);
            }
            #endif
        }
    }
}

_attribute_ram_code_sec_ static void tpsll_calculate_stick_diff(uint32_t tx_tick)
{
    uint16_t diff_us = 0;
    diff_us = (ll_ctx.stick - tx_tick) / RF_SYSTEM_TIMER_TICK_1US;
    rf_tx_packet.buf[0] = diff_us & 0xff;
    rf_tx_packet.buf[1] = (diff_us >> 8) & 0xff;
}

_attribute_ram_code_sec_ static void p24g_proc_1k_conn_rx(rf_packet_t *p)
{
    bool tx_valid_flag = false;
    uint8_t sn         = GET_SN(p->snnesn);
    uint8_t nesn       = GET_NESN(p->snnesn);

    // if (ll_ctx.task_slot != TASK_SLOT0)
    #if TPSLL_NON_8K_SPP_NOT_SW_EN
    #if TPSLL_NEW_SYNC_EN
    if ((ll_ctx.task_slot > TASK_SLOT1) || (p->pkt_type >= P24G_PKT_TYPE_SPP) || (ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500)))
    #else
    // if ((ll_ctx.task_slot > TASK_SLOT1 && ll_ctx.report_rate != REPORT_RATE_4K) || ((ll_ctx.report_rate <= REPORT_RATE_2K) && (p->pkt_type >= P24G_PKT_TYPE_SPP) ))
    // if ((ll_ctx.task_slot > TASK_SLOT1 && ll_ctx.report_rate != REPORT_RATE_4K) || ((p->pkt_type >= P24G_PKT_TYPE_SPP) ))
    if ((ll_ctx.task_slot > TASK_SLOT1) || (p->pkt_type >= P24G_PKT_TYPE_SPP))
    #endif
    #else
    if (ll_ctx.task_slot > TASK_SLOT1 && ll_ctx.report_rate != REPORT_RATE_4K)
    #endif
    {
        uint32_t tx_tick = stimer_get_tick();
        // if((p->pkt_type >= P24G_PKT_TYPE_SPP))
        if ((p->pkt_type >= P24G_PKT_TYPE_SPP) || (ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500)))
        {
            tx_tick += 6 * SYSTEM_TIMER_TICK_1US;
        }

        // if((ll_ctx.task_slot < ll_ctx.max_slot) || (ll_ctx.task_slot == ll_ctx.max_slot && pp_fifo_is_empty(&spp_fifo)))
        // if (tick1_exceed_tick2(ll_ctx.stick, tx_tick + TPSLL_TX_VALID_LEFT_TIME_US * RF_SYSTEM_TIMER_TICK_1US))
        {
            // rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, stimer_get_tick());
            ll_ctx.tx_over = false;
            rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, tx_tick);
            tx_valid_flag = true;
        }

        #if TPSLL_NEW_SYNC_EN
        if (ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500)) {
            tpsll_calculate_stick_diff(tx_tick);
        }
        #endif
        // ZW_TG(GPIO_PG7);
    }

#if TPSLL_N8K_FHSS_EN
    if (g_hop_map_cur != g_hop_map_next)
    {
        tpsll_fill_tx_pkt_chn_table();
        // ZW_TG(GPIO_PH0);
    }else{
        rf_tx_packet.n_tab = 0;
    }
#endif

    if (ll_ctx.lost_cnt)
    {
        ll_ctx.lost_cnt = 0;
    }

    #if 0
    if(sn)
    {
        ZW_TG(GPIO_PH0);
    }else{
        ZW_TG(GPIO_PH0);
        ZW_TG(GPIO_PH0);
    }

    if(nesn)
    {
        ZW_TG(GPIO_PG6);
    }else{
        ZW_TG(GPIO_PG6);
        ZW_TG(GPIO_PG6);
    }
    #endif

#if 0
    static uint32_t sncnt = 0;
    if (p->rf_len > 5 && sncnt < 5)
    {
        sncnt++;
        tlkapi_send_string_data(APP_LOG_EN, "1k_rx", &p->dma_len, p->rf_len + 6);
        //                    DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
    }

#endif

    if (sn != ll_ctx.rx_nesn)
    {   // old packet
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
        // if (ll_ctx.task_slot != TASK_SLOT0)
        // ll_ctx.last_pkt_acked = false;
        // {
        //     p24g_set_snnesn();
        // }

        #if TPSLL_NEW_SYNC_EN
        // if ((ll_ctx.task_slot > TASK_SLOT1) || (p->pkt_type >= P24G_PKT_TYPE_SPP) || (ll_ctx.report_rate & REPORT_RATE_500))
        if (tx_valid_flag)
        #else
        // if (((ll_ctx.task_slot >= TASK_SLOT1) && (ll_ctx.report_rate != REPORT_RATE_4K)) || ((p->pkt_type >= P24G_PKT_TYPE_SPP) ))
        if ((ll_ctx.task_slot > TASK_SLOT1) || (p->pkt_type >= P24G_PKT_TYPE_SPP))
        #endif
        {
            uint8_t len = p24g_fill_tx_packet();
            if(len)
            {
                pp_rf_set_tx_len(len);
            }
        }

#if AFH_ENABLE
        update_channel_quality(chn_list[ll_ctx.step_3t1r], true);
#endif
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
        // acked_flag = true;
        ll_ctx.last_pkt_acked = true;
        if(ll_ctx.ll_flag)
        {
            uint8_t *ptr = pp_fifo_get_ptr(&ll_fifo);

            ll_ctx.ll_flag = false;
            if (ptr) {
                if (ptr[2] == TPSLL_LL_CMD_UL_DATA_SEND_REQ)
                {
                    stack_evt_post_from_isr(TPSLL_EVT_USR_SPEC_ACK_RECV, (void *)&ptr[3], ptr[0] - 1);
                }
                pp_fifo_pop(&ll_fifo);
            }

            #if TPSLL_RR_CHG_WITH_RECON_EN && !TPSLL_RR_CHG_WITH_RECON_NO_RSP_EN
            if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP2) {
                ll_ctx.rr_chg_with_recon = RR_RECON_STEP3;
                ll_ctx.rr_chg_recon_time_tick = clock_time() | 1;

                tpsll_connect_lost();
            }
            #endif
        }

        if (pp_fifo_get_ptr(&spp_fifo) && ll_ctx.spp_flag) // spp mode only one slot
        {
            ll_ctx.spp_flag = false;
            // ZW_TG(GPIO_PG7);

            uint8_t *ptr = pp_fifo_get_ptr(&spp_fifo);
            uint8_t len = ptr[0];
            pp_fifo_pop(&spp_fifo);
            // unsigned char *payload = &ptr[2];
            // void *user_arg = *(void **)(&ptr[len + 6]);
            // fifo_cb_t spp_cb = *(fifo_cb_t *)(&ptr[len + 2]);

            // if (spp_cb)
            // {
            //     spp_cb(payload, len, true, user_arg);
            // }
            stack_evt_post_from_isr(TPSLL_INTER_EVT_SPP_SEND_COMP, (void *)&ptr[2], len+9);
        }
    }
    #if TPSLL_NEW_SYNC_EN
    // if ((ll_ctx.task_slot > TASK_SLOT1) || (p->pkt_type >= P24G_PKT_TYPE_SPP) || (ll_ctx.report_rate & REPORT_RATE_500))
    if (tx_valid_flag)
    #else
    // if (((ll_ctx.task_slot >= TASK_SLOT1) && (ll_ctx.report_rate != REPORT_RATE_4K)) || ((p->pkt_type >= P24G_PKT_TYPE_SPP) ))
    if ((ll_ctx.task_slot > TASK_SLOT1) || (p->pkt_type >= P24G_PKT_TYPE_SPP))
    #endif
    {
        uint8_t len = p24g_fill_tx_packet();

        if(len)
        {
            pp_rf_set_tx_len(len);
        }

    }



    // if (ll_ctx.task_slot != TASK_SLOT0)
    // {
    //     p24g_set_snnesn();
    // }

    tpsll_reset_idle_and_calib_check(p);

    if(p->pkt_type == P24G_PKT_TYPE_KB)
    {
        p24g_proc_kb_rx(p);
    }else if(p->pkt_type == P24G_PKT_TYPE_MS)
    {
        p24g_proc_1k_ms_rx(p);
    }else if(p->pkt_type == P24G_PKT_TYPE_LL)
    {
        p24g_proc_ll_rx(p);
    }
    else if(p->pkt_type == P24G_PKT_TYPE_SPP)
    {
        tpsll_proc_spp_rx(p);
    }

    #if TPSLL_KB_ACTIVE_PM_OPTM_EN
    if ((ll_inf.paired_dev_type & DEVICE_TYPE_KB) && p->rf_len) {
        ll_ctx.kb_alive_intervals_cnt = 0;
        ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
    }
    #endif
}


_attribute_ram_code_sec_ static void p24g_calculate_diff_tick(void)
{
    static uint32_t zdcnt = 0;
    static uint32_t slot_sync_tick_tab[] = {
    /*0*/(KM_SLOT0_TX_TIME_US + 34) * RF_SYSTEM_TIMER_TICK_1US,
    /*1*/(KM_SLOT0_TX_TIME_US + 34 + 180 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    /*2*/(KM_SLOT0_TX_TIME_US + 34 + 370 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    /*3*/(KM_SLOT0_TX_TIME_US + 34 + 560 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    /*4*/(KM_SLOT0_TX_TIME_US + 34 + 750 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    /*5*/(KM_SLOT0_TX_TIME_US + 34 + 940 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    /*6*/(KM_SLOT0_TX_TIME_US + 34 + 1130 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    /*7*/(KM_SLOT0_TX_TIME_US + 34 + 1320 - 7) * RF_SYSTEM_TIMER_TICK_1US,

    /*8*/(KM_SLOT0_TX_TIME_US + 34 + 1510 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    /*9*/(KM_SLOT0_TX_TIME_US + 34 + 1700 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    /*10*/(KM_SLOT0_TX_TIME_US + 34 + 1890 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    /*11*/(KM_SLOT0_TX_TIME_US + 34 + 2080 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    /*12*/(KM_SLOT0_TX_TIME_US + 34 + 2270 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    /*13*/(KM_SLOT0_TX_TIME_US + 34 + 2460 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    /*14*/(KM_SLOT0_TX_TIME_US + 34 + 2650 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    /*15*/(KM_SLOT0_TX_TIME_US + 34 + 2840 - 7) * RF_SYSTEM_TIMER_TICK_1US,
    };

    // if(zdcnt < 5)
    {
        uint32_t rtick         = reg_rf_timestamp;
        // uint32_t expected_tick = ll_ctx.last_stick  -  DONGLE_INTERVAL_US * RF_SYSTEM_TIMER_TICK_1US + RX_SYNC_TIME_TICK;
        // uint32_t expected_tick = ll_ctx.last_stick  +   93 * RF_SYSTEM_TIMER_TICK_1US - 2;
        // uint32_t expected_tick = ll_ctx.last_stick +  ((ll_ctx.task_slot == 0) ? ((KM_SLOT0_TX_TIME_US + 34) * RF_SYSTEM_TIMER_TICK_1US - 2) :
        //                             ((SLOT1_OFFSET_TIME_US_125 + (ll_ctx.task_slot - 1) * SLOT_TIME_US_125 + KM_TX_MARGIN_US_125 + 29 + 14 - (ll_ctx.task_slot & 1) * 7) * RF_SYSTEM_TIMER_TICK_1US));
        uint32_t expected_tick = ll_ctx.last_stick + slot_sync_tick_tab[ll_ctx.task_slot] + ll_ctx.ir_btn_proc_us;

        int32_t diff_tick     = rtick - expected_tick;
        int32_t diff_us = diff_tick / RF_SYSTEM_TIMER_TICK_1US;

#if DEBUG_SYNC_DIFF_TIME
        g_diff_us = diff_us;
#endif
        zdcnt++;
        // if (zdcnt < 5)
        // {

        //     uint32_t dt = rtick - ll_ctx.last_stick;
        //     tlkapi_printf(APP_LOG_EN, "last_tick:%d rtick:%d dt:%d %d:us  diff_tick:%d diff_us:%d\n", ll_ctx.last_stick, rtick, dt, dt / 24, diff_tick, diff_us);
        // }

        #if 0
        static int32_t  max_du = -1000;
        static int32_t  min_du = 1000;
        if(max_du < diff_us)
        {
     
            max_du = diff_us;
            uint32_t dt = rtick - ll_ctx.last_stick;
            tlkapi_printf(APP_LOG_EN, "last_tick:%d rtick:%d dt:%d %d:us  diff_tick:%d diff_us:%d max_du:%d\n", ll_ctx.last_stick, rtick, dt, dt / 24, diff_tick, diff_us, max_du);

        }
        if(min_du > diff_us)
        {

            min_du = diff_us;
            uint32_t dt = rtick - ll_ctx.last_stick;
            tlkapi_printf(APP_LOG_EN, "---last_tick:%d rtick:%d dt:%d %d:us  diff_tick:%d diff_us:%d min_du:%d\n", ll_ctx.last_stick, rtick, dt, dt / 24, diff_tick, diff_us, min_du);

        } 

        #endif
        // if (diff_us && (diff_us < -20 || diff_us > 20)) {
        //     uint32_t dt = rtick - ll_ctx.last_stick;
        //     tlkapi_printf(APP_LOG_EN, "last_tick:%d rtick:%d dt:%d %d:us  diff_tick:%d diff_us:%d max_du:%d\n", ll_ctx.last_stick, rtick, dt, dt / 24, diff_tick, diff_us, max_du);

        // }


        rf_tx_packet.buf[1] = (uint8_t)(diff_us & 0xff);


        // for (int i = 0; i < (ll_ctx.fno_last - ll_inf.fno_task); i++) {
        //     DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
        //     // tlkapi_printf(APP_LOG_EN, "expected_tick:%d rtick:%d diff_tick:%d diff_us:%d  zdcnt:%d\n", expected_tick, rtick, diff_tick, diff_us, zdcnt);
        // }
#if 0
        if(diff_tick < -23 || zdcnt < 5)
        {
            tlkapi_printf(APP_LOG_EN, "expected_tick:%d rtick:%d diff_tick:%d diff_us:%d  zdcnt:%d\n", expected_tick, rtick, diff_tick, diff_us, zdcnt);
        }else if(diff_tick > 23 || zdcnt < 5){
            tlkapi_printf(APP_LOG_EN, "expected_tick:%d rtick:%d diff_tick:%d diff_us:%d zdcnt:%d\n", expected_tick, rtick, diff_tick, diff_us, zdcnt);
        }
            #endif

        
        // zdcnt++;
    }

#if AFH_ENABLE
        // fill_chn_mask();
        // rf_tx_packet.mask = type >> 4;
        // rf_tx_packet.cnt = (type >> 2) & 0x3;
#endif

}

_attribute_ram_code_sec_ static void p24g_proc_125_conn_rx(rf_packet_t *p)
{
    uint8_t sn         = GET_SN(p->snnesn);
    uint8_t nesn       = GET_NESN(p->snnesn);

    // unsigned int tx_moment = ll_ctx.last_stick + (APP_RF_RECV_TX_TIME_US) * RF_SYSTEM_TIMER_TICK_1US;
    ll_ctx.tx_over = false;
    rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, stimer_get_tick());
    // ll_inf.rcv_bytes = p->rf_len;

    // DBG_STACK_GPIO_SET_LEVEL(STACK_IO_EN, dbg_io[DBG_IO_IDX1], sn);
    // pp_rf_set_tx_len(APP_STK_BASIC_ACK_LEN);
    // if(ll_ctx.task_slot == TASK_SLOT0)
    {
        p24g_calculate_diff_tick();
    }

    #if TPSLL_N8K_FHSS_EN
    if (g_hop_map_cur != g_hop_map_next)
    {
        tpsll_fill_tx_pkt_chn_table();
        // ZW_TG(GPIO_PH0);
    }else{
        rf_tx_packet.n_tab = 0;
    }
#endif

    // p24g_fill_null_pkt_125();
    // rf_tx_packet.buf[0] = 0x8c;
    // rf_tx_packet.buf[1] = 0x8c;
    // rf_tx_packet.buf[2] = 0x8c;

    // rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, stimer_get_tick() + 3 * RF_SYSTEM_TIMER_TICK_1US);
    // if (!pp_fifo_is_empty(&ll_fifo))
    // {
    //     p24g_fill_ll_pkt();
    // }
    // else
    // {
    //     p24g_fill_null_pkt();
    // }

    if (ll_ctx.lost_cnt)
    {
        ll_ctx.lost_cnt = 0;
    }

    #if 0

    if(sn)
    {
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
    }else{
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
    }

    if(nesn)
    {
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
    }else{
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
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
    {   // old packet
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
        // if (ll_ctx.task_slot != TASK_SLOT0)
        // ll_ctx.last_pkt_acked = false;
        // {
        //     p24g_set_snnesn();
        // }
#if AFH_ENABLE
        // update_channel_quality(chn_list[ll_ctx.step_3t1r], true);
#endif
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
        // acked_flag = true;
        ll_ctx.last_pkt_acked = true;
        if(ll_ctx.ll_flag)
        {
            uint8_t *ptr = pp_fifo_get_ptr(&ll_fifo);

            ll_ctx.ll_flag = false;
            if (ptr) {
                if (ptr[2] == TPSLL_LL_CMD_UL_DATA_SEND_REQ)
                {
                    stack_evt_post_from_isr(TPSLL_EVT_USR_SPEC_ACK_RECV, (void *)&ptr[3], ptr[0] - 1);
                }
                pp_fifo_pop(&ll_fifo);
            }

            #if TPSLL_RR_CHG_WITH_RECON_EN && !TPSLL_RR_CHG_WITH_RECON_NO_RSP_EN
            if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP2) {
                ll_ctx.rr_chg_with_recon = RR_RECON_STEP3;
                ll_ctx.rr_chg_recon_time_tick = clock_time() | 1;

                tpsll_connect_lost();
            }
            #endif
        }

        #if TPSLL_NON_8K_SPP_NOT_SW_EN
        if (pp_fifo_get_ptr(&spp_fifo) && ll_ctx.spp_flag) // spp mode only one slot
        {

            uint8_t *ptr = pp_fifo_get_ptr(&spp_fifo);
            uint8_t len = ptr[0];
            // unsigned char *payload = &ptr[2];
            // void *user_arg = *(void **)(&ptr[len + 6]);
            // fifo_cb_t spp_cb = *(fifo_cb_t *)(&ptr[len + 2]);

            // if (spp_cb)
            // {
            //     spp_cb(payload, len, true, user_arg);
            // }
            stack_evt_post_from_isr(TPSLL_INTER_EVT_SPP_SEND_COMP, (void *)&ptr[2], len+9);

            pp_fifo_pop(&spp_fifo);
            ll_ctx.spp_flag = false;
        }
        #endif
    }
    // if (ll_ctx.task_slot != TASK_SLOT0)
    // {
    //     p24g_set_snnesn();
    // }

    tpsll_reset_idle_and_calib_check(p);

    uint8_t len = p24g_fill_tx_packet();
    if(len)
        pp_rf_set_tx_len(len);

    if(p->pkt_type == P24G_PKT_TYPE_KB)
    {
        p24g_proc_kb_rx(p);
    }else if(p->pkt_type == P24G_PKT_TYPE_MS)
    {
        
        p24g_proc_1k_ms_rx(p);
    }else if(p->pkt_type == P24G_PKT_TYPE_LL)
    {
        p24g_proc_ll_rx(p);
    }
    #if TPSLL_NON_8K_SPP_NOT_SW_EN
    else if(p->pkt_type == P24G_PKT_TYPE_SPP)
    {
        tpsll_proc_spp_rx(p);
    }
    #endif

    #if TPSLL_N8K_FHSS_EN
    if(ll_ctx.task_slot == TASK_SLOT0)
    {
        ll_ctx.is_slot0_rx_ok = true;
    }
    #endif

#if DEBUG_SYNC_DIFF_TIME
    if (g_diff_us && (g_diff_us < -1 || g_diff_us > 1) && (g_diff_us > -120 && g_diff_us < 120)) {
    // if (g_diff_us && (g_diff_us > -120 && g_diff_us < 120)) {
    // if (g_diff_us) {
        tlkapi_printf(APP_LOG_EN, "(%d#%d)\n", ll_ctx.task_slot, g_diff_us);
        g_diff_us = 0;
        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
    }
    // if (g_diff_us && (g_diff_us > -120 && g_diff_us < 120)) {
    //     g_diff_us = 0;
    //     DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
    // }
#endif
}


_attribute_ram_code_sec_ void app_rf_irq_1k_handler(void)
{

    if(rf_get_irq_status(FLD_RF_IRQ_RX))
    {
        rf_status = IDLE_RF_STATUS;
        rf_clr_irq_status(FLD_RF_IRQ_RX);
        unsigned char *rx_buf = rf_get_rx_packet_addr(RX_FIFO_NUM, RX_FIFO_DEP, rx_packet);
        rf_packet_t  *p=(rf_packet_t *)rx_buf;

        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX1]);

        if (!(reg_rf_dec_err & 0x10) && ((uint32_t)(p->dma_len) == (uint32_t)RF_GET_DMA_LEN(p->rf_len)))
        {
//           rf_tx_packet.snnesn=p->snnesn;

             if (ll_ctx.stk_state == STATE_CONNECTED)
             {
                 p24g_proc_1k_conn_rx(p);
                #if TPSLL_KEEP_ALIVE_NOTIFY_EN
                ll_ctx.is_keep_alive = p->n_tab;
                if (ll_ctx.is_keep_alive) {
                    ll_ctx.keep_alive_refer_tick = clock_time() | 1;
                }
                #endif

                #if TPSLL_N8K_FHSS_EN
                
                if (ll_ctx.task_slot == TASK_SLOT0)
                {
                    tpsll_channel_statistic(ll_ctx.slot0_chn, false);

                }
                #endif
             }
             else if (ll_ctx.stk_state == STATE_PAIRING)
             {

                 if (p->buf[0] == PAIRING_RSP)
                 {
                     if (p->buf[1] > 1)
                     {
                         ll_ctx.stick += p->buf[1] * SYSTEM_TIMER_TICK_1US * DONGLE_INTERVAL_US;
                         stimer_set_irq_capture_d25f(ll_ctx.stick);
                     }
                     app_d24g_enter_conn();
                 }
             }
             DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX1]);
             cnt_rx_crc_ok++;
        }
        else
        {
            cnt_rx_crc_err++;
            RF_RX_CRC_GPIO_DEBUG_TOGGLE;
            app_rf_n8k_abnormal_handle();
        }
        cnt_rx_irq++;
        ll_ctx.rx_over = true;
    }

     if(rf_get_irq_status(FLD_RF_IRQ_FIRST_TIMEOUT))
    {
        rf_rx_fail_cnt++;
        rf_clr_irq_status(FLD_RF_IRQ_FIRST_TIMEOUT);
        RF_RX_TIMEOUT_GPIO_DEBUG_TOGGLE;
        RF_RX_GPIO_DEBUG_SET(0);

        app_rf_n8k_abnormal_handle();
        ll_ctx.rx_over = true;
    }

    if(rf_get_irq_status(FLD_RF_IRQ_RX_TIMEOUT))
    {
        // rf_rx_fail_cnt++;
        rf_clr_irq_status(FLD_RF_IRQ_RX_TIMEOUT);
        //print_app_rf("rx timeout\r\n");
        rf_status=IDLE_RF_STATUS;

        RF_RX_TIMEOUT_GPIO_DEBUG_TOGGLE;

        app_rf_n8k_abnormal_handle();
        #if RSSI_SCAN_EN && !TPSLL_NEW_RECON_EN
        if(scan_state == STATE_SCAN_RUNNING)
        {
            proc_rssi_scan();
        }
        #endif
        ll_ctx.rx_over = true;
    }

    if(rf_get_irq_status(FLD_RF_IRQ_TX))
    {
        ll_ctx.tx_over = true;
        cnt_tx_irq++;
        rf_clr_irq_status(FLD_RF_IRQ_TX);
        RF_TX_GPIO_DEBUG_SET(0);

        cnt_to_usb += curr_cnt_to_usb;
        curr_cnt_to_usb = 0;

        //print_app_rf("tx_done\r\n");
    }
    ll_ctx.wfi_flag = true;
}

_attribute_ram_code_sec_ void p24g_125_conti_rx(void)
{

    if ((ll_ctx.report_rate & REPORT_RATE_125 || ll_ctx.report_rate & REPORT_RATE_250) && \
                                ll_ctx.task_slot < (ll_ctx.max_slot / 2))
    {
        // SLOT_TIME_US_125 * 2
        uint32_t a_tick = stimer_get_tick() - ll_ctx.last_stick;
        uint16_t a_us = a_tick / RF_SYSTEM_TIMER_TICK_1US;
        uint16_t r_us = (ll_ctx.task_slot / 2 + 1) * 2 * SLOT_TIME_US_125 - a_us;
        if (r_us > 120)
        {
            pp_rf_start_srx(stimer_get_tick(), r_us - 15 - 96 + 20);
            ll_ctx.rx_over = false;
        }
    }
}

_attribute_ram_code_sec_ void app_rf_irq_125_handler(void)
{

    if(rf_get_irq_status(FLD_RF_IRQ_RX))
    {
        rf_status = IDLE_RF_STATUS;
        rf_clr_irq_status(FLD_RF_IRQ_RX);

        unsigned char *rx_buf = rf_get_rx_packet_addr(RX_FIFO_NUM, RX_FIFO_DEP, rx_packet);
        rf_packet_t  *p=(rf_packet_t *)rx_buf;

        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX1]);
        if (!(reg_rf_dec_err & 0x10) && ((uint32_t)(p->dma_len) == (uint32_t)RF_GET_DMA_LEN(p->rf_len)))
        {
//           rf_tx_packet.snnesn=p->snnesn;

             if (ll_ctx.stk_state == STATE_CONNECTED)
             {
                p24g_proc_125_conn_rx(p);
                #if TPSLL_N8K_FHSS_EN
                
                if (ll_ctx.task_slot == TASK_SLOT0)
                {
                    tpsll_channel_statistic(ll_ctx.slot0_chn, false);

                }
                #endif
             }
             else if (ll_ctx.stk_state == STATE_PAIRING)
             {

                 if (p->buf[0] == PAIRING_RSP)
                 {
                     if (p->buf[1] > 1)
                     {
                         ll_ctx.stick += p->buf[1] * SYSTEM_TIMER_TICK_1US * DONGLE_INTERVAL_US;
                         stimer_set_irq_capture_d25f(ll_ctx.stick);
                     }
                     app_d24g_enter_conn();
                 }
             }
             DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX1]);
             cnt_rx_crc_ok++;
        }
        else
        {
            cnt_rx_crc_err++;
            RF_RX_CRC_GPIO_DEBUG_TOGGLE;
            app_rf_n8k_abnormal_handle();
            p24g_125_conti_rx();
        }

        cnt_rx_irq++;
    }

     if(rf_get_irq_status(FLD_RF_IRQ_FIRST_TIMEOUT))
    {
        rf_rx_fail_cnt++;
        rf_clr_irq_status(FLD_RF_IRQ_FIRST_TIMEOUT);
        RF_RX_TIMEOUT_GPIO_DEBUG_TOGGLE;
        RF_RX_GPIO_DEBUG_SET(0);

        app_rf_n8k_abnormal_handle();

    }

    if(rf_get_irq_status(FLD_RF_IRQ_RX_TIMEOUT))
    {
        // rf_rx_fail_cnt++;
        rf_clr_irq_status(FLD_RF_IRQ_RX_TIMEOUT);
        //print_app_rf("rx timeout\r\n");
        rf_status=IDLE_RF_STATUS;

        RF_RX_TIMEOUT_GPIO_DEBUG_TOGGLE;

        app_rf_n8k_abnormal_handle();
        #if RSSI_SCAN_EN && !TPSLL_NEW_RECON_EN
        if(scan_state == STATE_SCAN_RUNNING)
        {
            proc_rssi_scan();
        }
        #endif
    }

    if(rf_get_irq_status(FLD_RF_IRQ_TX))
    {
        cnt_tx_irq++;
        rf_clr_irq_status(FLD_RF_IRQ_TX);
        RF_TX_GPIO_DEBUG_SET(0);

        cnt_to_usb += curr_cnt_to_usb;
        curr_cnt_to_usb = 0;

        p24g_125_conti_rx();

        //print_app_rf("tx_done\r\n");
    }
    ll_ctx.wfi_flag = true;
}



/**
*  @brief    rf irq handler
*  @param[in]  none   

*  @return  none
*/
_attribute_ram_code_sec_ uint8_t tpsll_rf_irq_handler(void)
{

    #if 0
    switch (ll_ctx.report_rate)
    {
    case REPORT_RATE_8K:
        app_rf_irq_8k_handler();
        break;
    case REPORT_RATE_4K:
    case REPORT_RATE_1K:
    case REPORT_RATE_2K:
    case REPORT_RATE_500:
        app_rf_irq_1k_handler();
        break;
    case REPORT_RATE_250:
    case REPORT_RATE_125:
        app_rf_irq_125_handler();
        break;
    default:
        break;
    }
    #endif
    /**
     * s_rf_irq_handler is assigned at the stack layer, so it is guaranteed to be non-null. 
     * Therefore, no null check is added for execution efficiency
     */

    (*s_rf_irq_handler)();

    // rf_clr_irq_status(FLD_RF_IRQ_ALL);
    return TLK_SUCCESS;
}

/**
 * @brief       Check idle state and trigger fast settle calibration after delay
 * @param       None
 * @return      None
 * @note        Called periodically in tpsll_stack_main_loop
 */
static void tpsll_idle_and_fast_settle_cal_check(void)
{
    // Static tick to record the time when entering idle state
    // static unsigned int idle_start_tick = 0;

    // Enter idle mode when no data received for timeout
    if (!ll_ctx.idle_flag && clock_time_exceed(ll_ctx.idle_tick, TPSLL_KEEP_ACTIVE_WAIT_TIME_MS * 1000))
    {
        ll_ctx.idle_flag = 1;
#if TPSLL_FAST_SETTLE_CAL_FUN_EN
        ll_ctx.fast_settle_cal_tick = stimer_get_tick();
    }

    // Trigger fast settle calibration after idle wait time
    if (!ll_ctx.fast_settle_cal_en && ll_ctx.idle_flag &&
        clock_time_exceed(ll_ctx.fast_settle_cal_tick, TPSLL_FAST_SETTLE_CAL_WAIT_MS * 1000))
    {
        ll_ctx.fast_settle_cal_en = 1;
        // gpio_toggle(GPIO_PH0);
    }

    if (!ll_ctx.fast_settle_calib_timeout && ll_ctx.idle_flag && clock_time_exceed(ll_ctx.fast_settle_cal_timeout_tick, TPSLL_FAST_SETTLE_CAL_TIMEOUT_MS * 1000))
    {
        ll_ctx.fast_settle_calib_timeout = 1;
        ll_ctx.fast_settle_calib_done = 0;
#endif
    }
}

void pp_stimer_init(void)
{
    ll_ctx.stick = stimer_get_tick() + 2 * SYSTEM_TIMER_TICK_1MS;
    stimer_set_irq_capture_d25f(ll_ctx.stick); //set capture tick

    stimer_set_irq_mask_d25f(FLD_SYSTEM_IRQ_MASK_D25F);
    plic_set_priority(IRQ_SYSTIMER, IRQ_PRI_LEV3);
    plic_interrupt_enable(IRQ_SYSTIMER);
}

void pp_stimer_start(uint32_t tick)
{
    stimer_set_irq_capture_d25f(tick); //set capture tick
}

_attribute_ram_code_sec_ uint8_t tpsll_change_report_rate(report_rate_t rr)
{
    uint8_t cmd_len = 2;
    uint8_t cmd[2] = {0};

    if(ll_ctx.report_rate == rr || !rr_is_valid(rr))
    {
        return TLK_ERR_NULL;
    }
    if (ll_ctx.stk_state != STATE_CONNECTED) {
        return TLK_ERR_INVALID_STATE;
    }

    cmd[0] = TPSLL_LL_CMD_RR_CHG_REQ;
    cmd[1] = rr;

    pp_fifo_push(&ll_fifo, TPSLL_LL_CMD, cmd, cmd_len); // push to fifo

    return 0;
}

_attribute_ram_code_sec_ void app_d24g_set_report_rate(uint8_t rr)
{
    ll_ctx.report_rate = rr;
    if (ll_ctx.last_rr != rr)
    {
        ll_ctx.last_rr = rr;
    }
    else
    {
        return;
    }

    #if 0
    if(rr & REPORT_RATE_8K)
    {
        ll_ctx.slot1_us = APP_SLOT1_START_TIME_US;
        ll_ctx.slot2_us = APP_SLOT2_START_TIME_US;
    }else{
        ll_ctx.slot1_us = 76;
        ll_ctx.slot2_us = APP_SLOT1_START_TIME_US + 16 + 50 - 6  + 3 - 2;//119
        // ll_ctx.slot2_us = 198;
        ll_ctx.slot3_us = 197;
        // ll_inf.slot1_2t_us = 197;

        write_reg16(0x17020e, 0); // tx wait
    }
    #endif

    switch (rr)
    {
    case REPORT_RATE_8K:
    // case REPORT_RATE_4K:
        ll_ctx.interval_us = DONGLE_INTERVAL_US;
        ll_ctx.slot1_us = APP_SLOT1_START_TIME_US;
        ll_ctx.slot2_us = APP_SLOT2_START_TIME_US + KM_3T_OFFSET_US;
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT;
        ll_ctx.sn_8k = 0;
        ll_ctx.nesn_8k = 0;
        ll_ctx.lost_cnt_max = TPSLL_LOST_CNT_MAX_8K; //n * 250us
        ll_ctx.ack_report = 0;
        ll_ctx.report_mode = REPORT_MODE_NORMAL;
        #if !TPSLL_SPP_NOT_SWITCH_EN
        SET_SN(rf_tx_packet.snnesn, ll_ctx.sn_8k);
        SET_NESN(rf_tx_packet.snnesn, ll_ctx.nesn_8k);
        #endif
        #if TPSLL_FHSS_REPORT_PER_EN
        ll_ctx.report_pkt_cnt = 0;
        ll_ctx.report_per = 0;
        #endif
        #if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
        ll_ctx.last_intv_pkt_st = TPSLL_PKT_ALL_RCVD;
        ll_ctx.intv_pkt_st = TPSLL_PKT_ALL_RCVD;
        ll_ctx.last_sn = 1;
        ll_ctx.last_nesn = 1;
        ll_ctx.acc.x = 0;
        ll_ctx.acc.y = 0;
        ll_ctx.last_acc.x = 0;
        ll_ctx.last_acc.y = 0;


        #if TPSLL_PACKET_LOSS_COMPENSATE_EN
        if(g_kalman_enable == TPSLL_ALG_ENABLE)
        {
            kalman_vel_init(&g_kf, 0.0f, 0.0f, TPSLL_KALMAN_Q, TPSLL_KALMAN_R);
            ll_ctx.last_btn = 0;
        }
        #endif
        #endif
        #if TPSLL_FAST_SETTLE_CAL_FUN_EN
        ll_ctx.fast_settle_cal_en = 0;
        ll_ctx.fast_settle_calib_timeout = 0;
        #endif
        ll_ctx.idle_flag = 0; 
        ll_ctx.idle_tick = stimer_get_tick();
        s_rf_irq_handler = app_rf_irq_8k_handler;
        #if TPSLL_N8K_FHSS_EN
        g_afh_cfg.afh_shift = TPSLL_FHSS_AFH_SLOT_SHIFT;
        g_afh_cfg.afh_mask = TPSLL_FHSS_PER_T_FNO_NUM;
        g_afh_cfg.update_pos = g_afh_cfg.afh_mask - TPSLL_FHSS_PER_AFH_WAIT_FNO_NUM;
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        ll_ctx.kb_alive_16ms_intervals_shift = 6;
        ll_ctx.kb_alive_16ms_intervals = 1 << 6;
        ll_ctx.kb_alive_16ms_intervals_msk = (1 << 6) - 1;
        #endif
        break;

    case REPORT_RATE_4K:
        ll_ctx.interval_us = DONGLE_INTERVAL_US;
        ll_ctx.slot1_us = APP_SLOT1_START_TIME_US;
        ll_ctx.slot2_us = APP_SLOT2_START_TIME_US;
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT;
        ll_ctx.max_slot = MAX_TASK_SLOTS_4K;
        rf_set_tx_wait_time(1);//Note: 0 mean 0xfff
        ll_ctx.lost_cnt_max = 800; //n * 250us
        #if TPSLL_FAST_SETTLE_CAL_FUN_EN
        ll_ctx.fast_settle_cal_en = 0;
        ll_ctx.fast_settle_calib_timeout = 0;
        #endif
        ll_ctx.idle_flag = 0; 
        ll_ctx.idle_tick = stimer_get_tick();
        ll_ctx.rx_win_us = DONGLE_CONNECTED_L_SRX_TIMEOUT_US - 10;
        s_rf_irq_handler = app_rf_irq_1k_handler;
        #if TPSLL_N8K_FHSS_EN
        g_afh_cfg.afh_shift = TPSLL_N8K_FHSS_AFH_SLOT_SHIFT;
        g_afh_cfg.afh_mask = ((1 << TPSLL_N8K_FHSS_AFH_SLOT_SHIFT) - 1);
        g_afh_cfg.update_pos = g_afh_cfg.afh_mask - TPSLL_N8K_FHSS_PER_AFH_WAIT_FNO_NUM;
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        ll_ctx.kb_alive_16ms_intervals_shift = 6;
        ll_ctx.kb_alive_16ms_intervals = 1 << 6;
        ll_ctx.kb_alive_16ms_intervals_msk = (1 << 6) - 1;
        #endif
        break;
    case REPORT_RATE_2K:
        ll_ctx.interval_us = DONGLE_INTERVAL_US << 1;
        ll_ctx.max_slot = MAX_TASK_SLOTS_2K;
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT >> 1;
        rf_set_tx_wait_time(1);
        ll_ctx.lost_cnt_max = 400; //n * 500us
        #if TPSLL_FAST_SETTLE_CAL_FUN_EN
        ll_ctx.fast_settle_cal_en = 0;
        ll_ctx.fast_settle_calib_timeout = 0;
        #endif
        ll_ctx.idle_flag = 0; 
        ll_ctx.idle_tick = stimer_get_tick();
        ll_ctx.rx_win_us = DONGLE_CONNECTED_L_SRX_TIMEOUT_US;
        s_rf_irq_handler = app_rf_irq_1k_handler;
        #if TPSLL_N8K_FHSS_EN
        g_afh_cfg.afh_shift = TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 1;
        g_afh_cfg.afh_mask = ((1 << (TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 1)) - 1);
        g_afh_cfg.update_pos = g_afh_cfg.afh_mask - TPSLL_N8K_FHSS_PER_AFH_WAIT_FNO_NUM;
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        if (ll_inf.paired_dev_type & DEVICE_TYPE_KB) {
            ll_ctx.kb_alive_16ms_intervals_shift = 5;
            ll_ctx.kb_alive_16ms_intervals = 1 << 5;
            ll_ctx.kb_alive_16ms_intervals_msk = (1 << 5) - 1;
        }
        #endif
        break;
    case REPORT_RATE_1K:
        ll_ctx.interval_us = DONGLE_INTERVAL_US << 2;
        ll_ctx.max_slot = MAX_TASK_SLOTS_1K;
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT >> 2;
        rf_set_tx_wait_time(1);
        ll_ctx.lost_cnt_max = 200; //n * 1ms
        #if TPSLL_FAST_SETTLE_CAL_FUN_EN
        ll_ctx.fast_settle_cal_en = 0;
        ll_ctx.fast_settle_calib_timeout = 0;
        #endif
        ll_ctx.idle_flag = 0; 
        ll_ctx.idle_tick = stimer_get_tick();
        ll_ctx.rx_win_us = DONGLE_CONNECTED_L_SRX_TIMEOUT_US + 23;
        s_rf_irq_handler = app_rf_irq_1k_handler;
        #if TPSLL_N8K_FHSS_EN
        g_afh_cfg.afh_shift = TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 2;
        g_afh_cfg.afh_mask = ((1 << (TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 2)) - 1);
        g_afh_cfg.update_pos = g_afh_cfg.afh_mask - (TPSLL_N8K_FHSS_PER_AFH_WAIT_FNO_NUM >> 1);
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        if (ll_inf.paired_dev_type & DEVICE_TYPE_KB) {
            ll_ctx.kb_alive_16ms_intervals_shift = 4;
            ll_ctx.kb_alive_16ms_intervals = 1 << 4;
            ll_ctx.kb_alive_16ms_intervals_msk = (1 << 4) - 1;
        }
        #endif
        break;
    case REPORT_RATE_500:
        ll_ctx.interval_us = DONGLE_INTERVAL_US << 3;
        ll_ctx.max_slot = MAX_TASK_SLOTS_500;
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT >> 3;
        rf_set_tx_wait_time(1);
        ll_ctx.lost_cnt_max = 200; //n * 2ms
        #if TPSLL_FAST_SETTLE_CAL_FUN_EN
        ll_ctx.fast_settle_cal_en = 0;
        ll_ctx.fast_settle_calib_timeout = 0;
        #endif
        ll_ctx.idle_flag = 0; 
        ll_ctx.idle_tick = stimer_get_tick();
        ll_ctx.rx_win_us = DONGLE_CONNECTED_L_SRX_TIMEOUT_US + 23;
        s_rf_irq_handler = app_rf_irq_1k_handler;
        #if TPSLL_N8K_FHSS_EN
        g_afh_cfg.afh_shift = TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 3;
        g_afh_cfg.afh_mask = ((1 << (TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 3)) - 1);
        g_afh_cfg.update_pos = g_afh_cfg.afh_mask - (TPSLL_N8K_FHSS_PER_AFH_WAIT_FNO_NUM >> 1);
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        if (ll_inf.paired_dev_type & DEVICE_TYPE_KB) {
            ll_ctx.kb_alive_16ms_intervals_shift = 3;
            ll_ctx.kb_alive_16ms_intervals = 1 << 3;
            ll_ctx.kb_alive_16ms_intervals_msk = (1 << 3) - 1;
        }
        #endif
        break;
    case REPORT_RATE_250:
        ll_ctx.interval_us = DONGLE_INTERVAL_US << 4;
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT >> 4;
        ll_ctx.max_slot = MAX_TASK_SLOTS_250;
        rf_set_tx_wait_time(6);
        ll_ctx.lost_cnt_max = 50; //n * 4ms
        #if TPSLL_FAST_SETTLE_CAL_FUN_EN
        ll_ctx.fast_settle_cal_en = 0;
        ll_ctx.fast_settle_calib_timeout = 0;
        #endif
        ll_ctx.idle_flag = 0; 
        ll_ctx.idle_tick = stimer_get_tick();
        s_rf_irq_handler = app_rf_irq_125_handler;
        #if TPSLL_N8K_FHSS_EN
        g_afh_cfg.afh_shift = TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 4;
        g_afh_cfg.afh_mask = ((1 << (TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 4)) - 1);
        g_afh_cfg.update_pos = g_afh_cfg.afh_mask - (TPSLL_N8K_FHSS_PER_AFH_WAIT_FNO_NUM >> 2);
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        if (ll_inf.paired_dev_type & DEVICE_TYPE_KB) {
            ll_ctx.kb_alive_16ms_intervals_shift = 0;
            ll_ctx.kb_alive_16ms_intervals = 0;
            ll_ctx.kb_alive_16ms_intervals_msk = 0;
        }
        #endif
        break;
    case REPORT_RATE_125:
        ll_ctx.interval_us = DONGLE_INTERVAL_US << 5;
        ll_ctx.chg_dly_cnt = RR_CHG_DLY_CNT >> 5;
        ll_ctx.max_slot = MAX_TASK_SLOTS_125;
        rf_set_tx_wait_time(6);
        ll_ctx.lost_cnt_max = 25; //n * 8ms
        #if TPSLL_FAST_SETTLE_CAL_FUN_EN
        ll_ctx.fast_settle_cal_en = 0;
        ll_ctx.fast_settle_calib_timeout = 0;
        #endif
        ll_ctx.idle_flag = 0; 
        ll_ctx.idle_tick = stimer_get_tick();
        s_rf_irq_handler = app_rf_irq_125_handler;
        #if TPSLL_N8K_FHSS_EN
        g_afh_cfg.afh_shift = TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 5;
        g_afh_cfg.afh_mask = ((1 << (TPSLL_N8K_FHSS_AFH_SLOT_SHIFT - 5)) - 1);
        g_afh_cfg.update_pos = g_afh_cfg.afh_mask - (TPSLL_N8K_FHSS_PER_AFH_WAIT_FNO_NUM >> 2);
        #endif
        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        if (ll_inf.paired_dev_type & DEVICE_TYPE_KB) {
            ll_ctx.kb_alive_16ms_intervals_shift = 0;
            ll_ctx.kb_alive_16ms_intervals = 0;
            ll_ctx.kb_alive_16ms_intervals_msk = 0;
        }
        #endif
        break;
    default:
        ll_ctx.interval_us = DONGLE_INTERVAL_US;
        break;
    }

    rf_set_rx_maxlen(RF_MAX_LEN);

    #if TPSLL_KEEP_ALIVE_NOTIFY_EN
    ll_ctx.is_keep_alive = 0;
    #endif

    tmemset(&last_ms_data, 0, 7);
}

_attribute_ram_code_sec_ void app_fill_paring_pkt(void)
{
    rf_packet_t *pt = &rf_tx_packet;
    if (ll_ctx.pair_st == STATE_PAIR_INIT)
    {
        pp_rf_set_tx_len(1);
        pt->buf[0] = CMD_PAIR_REQ;
    }
    
    
    // rf_tx_packet.buf[0] = PAIRING_CMD;
    // rf_tx_packet.buf[1] = ll_ctx.fno_last & 0xff;
    // rf_tx_packet.buf[2] = (ll_ctx.fno_last >> 8) & 0xff;
    // rf_tx_packet.buf[3] = ll_inf.side_id & 0xff;
}


_attribute_ram_code_sec_ void tpsll_usb_enum_done_handler(void)
{
    g_usb_sof_start = 0;
    app_usb_bus_reset_init();

    if (!ll_ctx.is_stack_running) {
        pp_stimer_init();
        ll_ctx.is_stack_running = true;
    }
    stack_evt_post_from_isr(TPSLL_EVT_STACK_START, 0, 0);
}

_attribute_ram_code_sec_ void tpsll_usb_sof_handler(void)
{
//    static unsigned char usb_ep_ready = 0;
    if (!g_usb_sof_start) {
        switch (ll_ctx.report_rate)
        {
            case REPORT_RATE_8K:
                g_usb_sof_start = 1;
                ll_ctx.stick = usb0hw_get_timer_stamp() + (DONGLE_INTERVAL_US - DONGLE_STIMER_TO_SOF_DELAY_US) * SYSTEM_TIMER_TICK_1US;
                break;

            default:
//                if (usb_ep_ready == 0) {
//                    if (app_usb_ep_is_idle(HID_MOUSE_IN_ENDPOINT_NUM)) {
//                        return;
//                    }
//                    app_mouse_usb_ep_buff_fill();
//                    usb_ep_ready = 1;
//                    return;
//                } else if (usb_ep_ready == 1) {
//                    if (app_mouse_usb_ep_buf_has_data())
//                        return;
//                }
//                usb_ep_ready = 0;
                g_usb_sof_start = 1;
                ll_ctx.stick = usb0hw_get_timer_stamp() + (DONGLE_SOF_INTERVAL_US - DONGLE_4K_STIMER_TO_SOF_DELAY_US) * SYSTEM_TIMER_TICK_1US;
                break;
        }
        // stimer_set_irq_capture_d25f(ll_ctx.stick); //set capture tick
        app_set_next_stimer_anchor();
    }

}

_attribute_ram_code_sec_ void app_set_next_stimer_anchor(void)
{
    #if TPSLL_RECON_RSSI_SCAN_EN && TPSLL_NEW_RECON_EN
    if ((ll_ctx.stk_state == STATE_RECONNECT)
        #if TPSLL_NEW_PAIR_EN
        || (ll_ctx.stk_state == STATE_PAIRING)
        #endif
    ) {
        if (ll_ctx.recon_rssi_scan_step && !ll_ctx.suspend_enable) {
            if (ll_ctx.recon_rssi_scan_step == RECON_RSSI_SCAN_STEP_PREPARE) {
                ll_ctx.stick += 150 * SYSTEM_TIMER_TICK_1US;
                ll_ctx.chg_dly_cnt = 0;
            } else {
                ll_ctx.stick += TPSLL_RECON_RSSI_SCAN_TIME_US * SYSTEM_TIMER_TICK_1US;

                #if TPSLL_RR_CHG_WITH_RECON_EN
                if (ll_ctx.rr_chg_with_recon >= RR_RECON_STEP4) {
                    ll_ctx.rr_chg_with_recon = RR_RECON_STEP3;
                }
                #endif
            }
            ll_ctx.recon_rssi_scan_step++;
        } else if ((ll_ctx.stk_state == STATE_RECONNECT) && !ll_ctx.recon_start_ref_tick && !ll_ctx.recon_rx_for_pkt) {
            ll_ctx.recon_rx_for_pkt = true;
            ll_ctx.stick += ll_ctx.recon_pm_optm_interval_ms * SYSTEM_TIMER_TICK_1MS;
        }
        #if DONGLE_SUSPEND_SUPPORT
        else if (ll_ctx.suspend_allow && ll_ctx.suspend_enable) {
            ll_ctx.stick += ll_ctx.suspend_interval_ms * SYSTEM_TIMER_TICK_1US * 1000;
        }
        #endif
        else {
            ll_ctx.stick += DONGLE_INTERVAL_US * SYSTEM_TIMER_TICK_1US;

            #if TPSLL_RR_CHG_WITH_RECON_EN
            if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP3) {
                ll_ctx.rr_chg_with_recon = RR_RECON_STEP4;
                ll_ctx.stick = usb0hw_get_timer_stamp() + (DONGLE_INTERVAL_US * 2 - DONGLE_STIMER_TO_SOF_DELAY_US) * SYSTEM_TIMER_TICK_1US;
            }
            #endif
        }
        stimer_set_irq_capture_d25f(ll_ctx.stick);
        return;
    }
    #endif

    #if STIMER_SOF_DELTA_ADJ_DELAY_CNT
    static uint8_t delay_cnt = STIMER_SOF_DELTA_ADJ_DELAY_CNT;
    #endif
    int delta = 0;
#if DEBUG_SYNC_DIFF_TIME
    int32_t dbg_delta = 0;
#endif
    unsigned int usb_sof_timestamp = usb0hw_get_timer_stamp();

    if (g_usb_sof_start) {
        //DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
        switch (ll_ctx.report_rate)
        {
            case REPORT_RATE_8K:
                delta = ((usb_sof_timestamp + (DONGLE_INTERVAL_US - DONGLE_STIMER_TO_SOF_DELAY_US) * SYSTEM_TIMER_TICK_1US) - ll_ctx.last_stick);
                break;

            // case REPORT_RATE_4K:
            //     delta = ((usb_sof_timestamp + (DONGLE_INTERVAL_US - DONGLE_STIMER_TO_SOF_DELAY_US) * SYSTEM_TIMER_TICK_1US) - ll_ctx.last_stick);
            // break;

            default:
                // if((ll_ctx.report_rate & REPORT_RATE_125) || (ll_ctx.report_rate & REPORT_RATE_250))
                // {
                // } else {
                //     delta = ((usb_sof_timestamp + (DONGLE_INTERVAL_US - DONGLE_STIMER_TO_SOF_DELAY_US - D_SUSPEND_EARLY_TIME_US) * SYSTEM_TIMER_TICK_1US) - ll_ctx.last_stick);
                // }
                // delta = ((usb_sof_timestamp + (DONGLE_SOF_INTERVAL_US - DONGLE_4K_STIMER_TO_SOF_DELAY_US) * SYSTEM_TIMER_TICK_1US) - ll_ctx.last_stick);
                break;
        }

        #if DEBUG_SYNC_DIFF_TIME
        dbg_delta = delta;
        #endif

        #if STIMER_SOF_DELTA_ADJ_MAX_US
        if ((delta > -DONGLE_SOF_INTERVAL_US * SYSTEM_TIMER_TICK_1US) && (delta < DONGLE_SOF_INTERVAL_US * SYSTEM_TIMER_TICK_1US)) {
            if (delta > STIMER_SOF_DELTA_ADJ_MAX_US * SYSTEM_TIMER_TICK_1US) {
                #if STIMER_SOF_SYNC_WITH_KMD_SYNCED_EN
                if (ll_ctx.synced) {
                    delta = STIMER_SOF_DELTA_ADJ_MAX_US * SYSTEM_TIMER_TICK_1US;
                } else {
                    delta = 0;
                }
                #elif STIMER_SOF_DELTA_ADJ_DELAY_CNT
                if (delay_cnt) {
                    delay_cnt--;
                    delta = 0;
                } else {
                    delay_cnt = STIMER_SOF_DELTA_ADJ_DELAY_CNT;
                    delta = STIMER_SOF_DELTA_ADJ_MAX_US * SYSTEM_TIMER_TICK_1US;
                }
                #else
                delta = STIMER_SOF_DELTA_ADJ_MAX_US * SYSTEM_TIMER_TICK_1US;
                #endif
            } else if (delta < -STIMER_SOF_DELTA_ADJ_MAX_US * SYSTEM_TIMER_TICK_1US) {
                #if STIMER_SOF_SYNC_WITH_KMD_SYNCED_EN
                if (ll_ctx.synced) {
                    delta = -STIMER_SOF_DELTA_ADJ_MAX_US * SYSTEM_TIMER_TICK_1US;
                } else {
                    delta = 0;
                }
                #elif STIMER_SOF_DELTA_ADJ_DELAY_CNT
                if (delay_cnt) {
                    delay_cnt--;
                    delta = 0;
                } else {
                    delay_cnt = STIMER_SOF_DELTA_ADJ_DELAY_CNT;
                    delta = -STIMER_SOF_DELTA_ADJ_MAX_US * SYSTEM_TIMER_TICK_1US;
                }
                #else
                delta = -STIMER_SOF_DELTA_ADJ_MAX_US * SYSTEM_TIMER_TICK_1US;
                #endif
            }
        } else {
            delta = 0;
        }
        ll_ctx.stick += ll_ctx.interval_us * SYSTEM_TIMER_TICK_1US + delta;
        #else
        ll_ctx.stick += ll_ctx.interval_us * SYSTEM_TIMER_TICK_1US + (((delta > -DONGLE_SOF_INTERVAL_US * SYSTEM_TIMER_TICK_1US) && (delta < DONGLE_SOF_INTERVAL_US * SYSTEM_TIMER_TICK_1US)) ? delta : 0);
        #endif
    } else {
        ll_ctx.stick += ll_ctx.interval_us * SYSTEM_TIMER_TICK_1US;
    }

    ll_ctx.fno++;

    #if TPSLL_RR_CHG_WITH_RECON_EN && !TPSLL_NEW_RECON_EN
    if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP3) {
        ll_ctx.rr_chg_with_recon = RR_RECON_STEP4;
        if (ll_ctx.user_rr_last & (~REPORT_RATE_8K)) {
            ll_ctx.stick = usb0hw_get_timer_stamp() + (DONGLE_INTERVAL_US - DONGLE_STIMER_TO_SOF_DELAY_US) * SYSTEM_TIMER_TICK_1US;
        }
    }
    #endif
    if (ll_ctx.chg_dly_cnt)
    {
        #if TPSLL_RR_CHG_WITH_RECON_EN
        ll_ctx.stick += TPSLL_RR_SWITCH_CONFIG_TIME_US * SYSTEM_TIMER_TICK_1US;
        #else
        ll_ctx.stick += ll_ctx.chg_dly_cnt * ll_ctx.interval_us * SYSTEM_TIMER_TICK_1US;
        #endif
        ll_ctx.chg_dly_cnt = 0;
    }
    
    if((ll_ctx.report_rate & REPORT_RATE_125) || (ll_ctx.report_rate & REPORT_RATE_250))
    {
        ll_ctx.early_stick = ll_ctx.stick - D_SUSPEND_EARLY_TIME_US * SYSTEM_TIMER_TICK_1US;
        stimer_set_irq_capture_d25f(ll_ctx.early_stick);
    }else{
        stimer_set_irq_capture_d25f(ll_ctx.stick);
    }

    #if STIMER_SOF_SYNC_WITH_KMD_SYNCED_EN
    ll_ctx.synced = false;
    #endif
#if DEBUG_SYNC_DIFF_TIME
    // if((ll_ctx.report_rate & REPORT_RATE_125) || (ll_ctx.report_rate & REPORT_RATE_250))
    // {
    //     static int16_t last_delta_us = 0;
    //     if (last_delta_us != (dbg_delta / 24)) {
    //         last_delta_us = dbg_delta / 24;
    //         tlkapi_printf(APP_LOG_EN, "<%d>\n", last_delta_us);
    //     }
    // }
#endif
}

_attribute_ram_code_sec_ void dongle_fill_tx_data(void)
{
    pp_rf_set_tx_len(RF_PKT_ACK_LEN);
#if AFH_ENABLE
    fill_chn_mask();
    // rf_tx_packet.mask = type >> 4;
    // rf_tx_packet.cnt = (type >> 2) & 0x3;
#endif
    rf_tx_packet.buf[0] = (rf_tx_packet.buf[0] & 0xF8) | (ll_ctx.ack_mask & 0x7);
    // rf_tx_packet.buf[2]++;
}

#if !TPSLL_SPP_NOT_SWITCH_EN
_attribute_ram_code_sec_ static void tpsll_stimer_spp_mode_handler(void)
{

    // tpsll_spp_mode_pkt_send();

    // ll_ctx.ch_idx = (ll_ctx.task_slot + ll_ctx.fno_last) & 3;
    uint8_t ch = chn_table[ll_ctx.fno_last & 3];

    // rf_set_ble_chn(chn_table[(ll_ctx.task_slot / 2) + ll_ctx.ch_idx]);
    // rf_set_ble_chn(chn_table[(ll_ctx.task_slot + ll_ctx.fno_last) & 3]);
    rf_set_ble_chn(ch);

    pp_rf_start_srx(stimer_get_tick(), DONGLE_CONNECTED_SPP_MODE_RX_TIMEOUT);

    pp_timer1_start((250) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick)); // set time for SLOT1
}
#endif

_attribute_ram_code_sec_ static bool p24g_rr_switch_check(void)
{

#if 1
    bool rr_changed = false;

    if (ll_ctx.is_rr_changing && (ll_ctx.pending_rr_fno == ll_ctx.fno_last))
    {
        ll_ctx.lost_cnt = 0;

        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP2) {
            ll_ctx.rr_chg_with_recon = RR_RECON_STEP3;
            ll_ctx.spp_mode = ll_ctx.pending_spp_mode;
            ll_ctx.is_rr_changing = false;
            ll_ctx.rr_chg_recon_time_tick = clock_time() | 1;

            tpsll_connect_lost();

            if(ll_ctx.ll_flag)
            {
                pp_fifo_pop(&ll_fifo);
                ll_ctx.ll_flag = false;
            }
            return true;
        }
        #endif

        if(ll_ctx.last_rr == ll_ctx.chg_rr)//change slot mode
        {
            ll_ctx.spp_mode = ll_ctx.pending_spp_mode;
            ll_ctx.is_rr_changing = false;
            rf_set_rx_maxlen(KM_SPP_MAX_LEN);
            return rr_changed;
        }

        app_d24g_set_report_rate(ll_ctx.chg_rr);

        #if TPSLL_ACC_SWITCH_WITH_RR_EN
        {
            uint32_t acc;
            p24g_gen_ac_and_channel(ll_inf.mac, &acc, 0);
            rf_access_code_comm(acc);
            // tlkapi_printf(APP_LOG_EN, "acc(%X)", acc);
        }
        #endif

        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (ll_ctx.rr_chg_with_recon == RR_RECON_STEP6) {
            ll_ctx.rr_chg_with_recon = RR_RECON_STEP_NONE;
            ll_ctx.rr_chg_recon_time_tick = 0;
            ll_ctx.user_rr_last = ll_ctx.user_rr;
            stack_evt_post_from_isr(TPSLL_EVT_REPORT_RATE_CHANGED, (void *)&ll_ctx.report_rate, 1);
        } else
        #endif
        if(ll_ctx.spp_mode == ll_ctx.pending_spp_mode)
        {
            // p24g_evt_cb(TPSLL_EVT_REPORT_RATE_CHANGED, NULL, 0);
            stack_evt_post_from_isr(TPSLL_EVT_REPORT_RATE_CHANGED, (void *)&ll_ctx.report_rate, 1);
        }


        ll_ctx.spp_mode = ll_ctx.pending_spp_mode;
        
        rr_changed = true;
        ll_ctx.is_rr_changing = false;
        ll_ctx.spp_flag = false;
        ll_ctx.ll_flag = false;
        ll_ctx.last_pkt_acked = true;
        ll_ctx.rx_over = true;
        ll_ctx.tx_over = true;

        ll_ctx.tx_sn = 1;
        ll_ctx.rx_nesn = 0;
        SET_SN(rf_tx_packet.snnesn, ll_ctx.tx_sn);
        SET_NESN(rf_tx_packet.snnesn, ll_ctx.rx_nesn);

        if(ll_ctx.spp_mode)
        {
            rf_set_rx_maxlen(KM_SPP_MAX_LEN);
        }
    }

    return rr_changed;

#endif
}

#if TPSLL_FAST_SETTLE_CAL_FUN_EN
/**
 * @brief       Dynamic calibration for fast settle
 * @param[in]   batch_num: Number of continuous calibration get operations per function call
 *                   1 = Normal single step mode
 *                  >1 = Batch mode (multi get + one set)
 * @note        TX calibration for all valid channels + RX calibration for 0,10,20...80
 * @return      none
 */
void tpsll_fast_settle_dynamic_calib(uint8_t batch_num)
{
    uint8_t exec_cnt = 0;
    static uint8_t ch_index = 0;
    extern rf_fast_settle_t ext_fast_settle_6M;

    /* Exit if calibration already done */
    if (ll_ctx.fast_settle_calib_done)
    {
        ch_index = 0;
        return;
    }

    /* Limit batch number to avoid overflow */
    if (batch_num == 0)
    {
        batch_num = 1;
    }

    /* -------------------------- */
    /* Step 1: Execute multiple calibration get in sequence */
    /* -------------------------- */
    while (exec_cnt < batch_num && !ll_ctx.fast_settle_calib_done)
    {
        // gpio_write(GPIO_PG6, 1);
        /* TX calibration get (valid FHSS channels) */
        if (ch_index < TPSLL_FHSS_CHN_MAX_NUM)
        {
            ext_fast_settle_get_val_step(0, g_fhss_valid_chn_tab[ch_index],
                                        TX_FAST_SETTLE_LEVEL, RX_FAST_SETTLE_LEVEL,
                                        (rf_fast_settle_t *)&ext_fast_settle_6M);
        }
        /* RX calibration get (0,10,20...80) */
        else
        {
            uint8_t rx_chn = (ch_index - TPSLL_FHSS_CHN_MAX_NUM) * 10 + FCAL_BASE_CHANNEL;
            ext_fast_settle_get_val_step(1, rx_chn,
                                        TX_FAST_SETTLE_LEVEL, RX_FAST_SETTLE_LEVEL,
                                        (rf_fast_settle_t *)&ext_fast_settle_6M);
        }
        // gpio_write(GPIO_PG6, 0);
        ch_index++;
        exec_cnt++;

        /* Check if one full round calibration is completed */
        if (ch_index >= (TPSLL_FHSS_CHN_MAX_NUM + 8))
        {
            ch_index = 0;
            ll_ctx.fast_settle_calib_done = 1;
            ll_ctx.fast_settle_calib_timeout = 0;
            ll_ctx.fast_settle_cal_timeout_tick = stimer_get_tick();
        }
    }

    /* -------------------------- */
    /* Step 2: One-time set after batch get finished */
    /* -------------------------- */
    ext_fast_settle_set_val(EXT_FAST_SETTLE_PHY_6M);
    rf_set_tx_settle_time(12);
    rf_set_rx_settle_time(12);

    /* RF channel table re-config according to report rate */
    if (ll_ctx.report_rate & REPORT_RATE_8K)
    {
        if(s_app_cb && s_app_cb->app_rf_hw_init)
        s_app_cb->app_rf_hw_init(chn_list[STEP_3T1R_1T_D_1R]);
    }
    else if ((ll_ctx.report_rate & REPORT_RATE_125) || (ll_ctx.report_rate & REPORT_RATE_250))
    {
        
        if(s_app_cb && s_app_cb->app_rf_hw_init)
        s_app_cb->app_rf_hw_init(chn_table[(ll_ctx.fno_last) & 3]);
    }
    else
    {
        if(s_app_cb && s_app_cb->app_rf_hw_init)
        s_app_cb->app_rf_hw_init(chn_table[(ll_ctx.task_slot + ll_ctx.fno_last) & 3]);
    }
}
#endif

_attribute_ram_code_sec_ static void connected_stimer_irq_handler(void)
{
//     #if TPSLL_SPP_NOT_SWITCH_EN
//     if ((ll_ctx.report_rate & REPORT_RATE_NOT_8K_MSK) && wait_rf_fsm_idle_timeout(5))
//     #else
//     if (wait_rf_fsm_idle_timeout(5))
//     #endif
//     {
//         rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
//     }
    ll_ctx.task_slot = TASK_SLOT0;

    if (!p24g_rr_switch_check()
        #if TPSLL_RF_SCAN_EN
        && (rf_scan_ctx.state & (RF_SCAN_STATE_IDLE | RF_SCAN_STATE_PAUSE | RF_SCAN_STATE_DONE))
        #endif
        )
    {
        bool rx_valid_flag = true;
        // SLOT0 start
        ll_ctx.step_3t1r = STEP_3T1R_1T_D_1R; // dongle 1R start

        // rf_set_ble_chn(chn_list[STEP_3T1R_1T_D_1R]);

        #if !TPSLL_SPP_NOT_SWITCH_EN
        if (!ll_ctx.spp_mode)
        #endif
        {
                #if TPSLL_SPP_NOT_SWITCH_EN || TPSLL_NON_8K_SPP_NOT_SW_EN
                rf_packet_t  *p=(rf_packet_t *)rx_packet;
                if (ll_ctx.rx_over) {
                    p->pkt_type = P24G_PKT_TYPE_MS;
                    p->dma_len = 0;
                    p->rf_len = 0;
                }
                // rf_set_rx_maxlen(RF_PKT_DATA_LEN);
                rf_set_rx_maxlen(KM_SPP_MAX_LEN);
                #else
                rf_set_rx_maxlen(RF_PKT_DATA_LEN);
                // rf_set_rx_maxlen(48);
                #endif
        
                #if TPSLL_FAST_SETTLE_CAL_FUN_EN
                static bool fast_switch_flag = 1;
                if ((ll_ctx.report_rate & (REPORT_RATE_8K | REPORT_RATE_4K | REPORT_RATE_2K)) && ll_ctx.fast_settle_calib_timeout)
                {
                    fast_switch_flag = ll_ctx.fast_settle_cal_en ? !fast_switch_flag : 1;
                }
                else
                {
                    fast_switch_flag = 1;
                }
                if (fast_switch_flag)
                {
                #endif
            if (ll_ctx.report_rate & REPORT_RATE_8K)
            {
                #if TPSLL_SPP_NOT_SWITCH_EN
                if (tpsll_is_rf_busy()) {
                    if(rf_tx_packet.pkt_type == P24G_PKT_TYPE_SPP) {
                        rx_valid_flag = false;
                        #if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
                        if(ll_ctx.is_need_restore_acc)
                        {
                            rf_access_code_comm(ll_ctx.conn_acc);
                            ll_ctx.is_need_restore_acc = false;
                        }
                        #endif
                    } else {
                        rf_set_tx_rx_off_auto_mode();
                    }
                }
                if (rx_valid_flag)
                #endif
                {
                    rf_set_ble_chn(chn_list[STEP_3T1R_1T_D_1R]);
                    pp_rf_start_srx(stimer_get_tick(), DONGLE_CONNECTED_SRX_TIMEOUT_US);
                    ll_ctx.rx_over = false;
                    #if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
                    if(ll_ctx.is_need_restore_acc)
                    {
                        rf_access_code_comm(ll_ctx.conn_acc);
                        ll_ctx.is_need_restore_acc = false;
                    }
                    #endif
                } else {
                    ll_ctx.step_3t1r = STEP_3T1R_2T_D_2R;
                }
                calc_hop_channels(ll_ctx.fno_last + 1, chn_mask);
                #if TPSLL_NEW_8K_RF_TIMING_EN
                pp_timer1_start((APP_SLOT1_START_TIME_US - 5) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick) + TIMER1_FINE_ADJ);
                #else
                pp_timer1_start(APP_SLOT1_START_TIME_US * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick) + TIMER1_FINE_ADJ);
                #endif

                ll_ctx.last_ack_mask = ll_ctx.ack_mask;
                ll_ctx.ack_mask = 0;

                #if TPSLL_LOCAL_MS_DATA_TEST_EN
                pp_timer0_start(140 * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));
                ll_ctx.local_ms_data_test = 1;
                #endif
                #if TPSLL_8K_PKT_LOSS_SNNESN_OP_EN
                rf_tx_packet.s_n = ll_ctx.last_nesn;

                ll_ctx.last_intv_pkt_st = ll_ctx.intv_pkt_st;
                ll_ctx.intv_pkt_st = TPSLL_PKT_ALL_LOSS;
                     
                ll_ctx.last_acc.x = ll_ctx.acc.x;
                ll_ctx.last_acc.y = ll_ctx.acc.y;
                #endif
                #if TPSLL_SPP_NOT_SWITCH_EN
                rf_tx_packet.pkt_type = P24G_PKT_TYPE_SYNC;
                #endif
            }
            else if ((ll_ctx.report_rate & (REPORT_RATE_125 | REPORT_RATE_250)))
            {
                // #if TPSLL_SPP_NOT_SWITCH_EN
                // if (tpsll_is_rf_busy()) {
                //     if(rf_tx_packet.pkt_type == P24G_PKT_TYPE_SPP) {
                //         rx_valid_flag = false;
                //     } else {
                //         rf_set_tx_rx_off_auto_mode();
                //     }
                // }
                // if (rx_valid_flag)
                // #endif
                {
                // rf_set_ble_chn(CONN_CHN_NO);
                // rf_set_ble_chn(chn_table[ll_ctx.task_slot & 3]);
                #if TPSLL_N8K_FHSS_EN
                ll_ctx.slot0_chn = chn_table[(ll_ctx.fno_last) & 3];
                // uint8_t ch = chn_table[(ll_ctx.fno_last) & 3];
                rf_set_ble_chn(ll_ctx.slot0_chn);
                #else
                uint8_t ch = g_n8k_chn_tab[(ll_ctx.fno_last) & (NON_8K_CHN_NUM - 1)];
                rf_set_ble_chn(ch);
                #endif

                // rf_set_ble_chn(chn_table[(ll_ctx.task_slot / 2) + ll_ctx.ch_idx]);
                // rf_set_ble_chn(chn_table[(ll_ctx.task_slot + ll_ctx.fno_last) & 3]);

                
                // pp_rf_start_srx2tx((unsigned char *)&rf_tx_packet.dma_len, stimer_get_tick(), DONGLE_CONNECTED_SRX_TIMEOUT_US);
                // dongle_fill_tx_data();
                // pp_rf_start_srx2tx(stimer_get_tick(), DONGLE_CONNECTED_1ST_SUSPEND_SRX_TIMEOUT_US);
                // pp_rf_start_srx2tx((unsigned char *)&rf_tx_packet.dma_len, stimer_get_tick(), DONGLE_CONNECTED_1ST_SUSPEND_SRX_TIMEOUT_US - 15);

                pp_rf_start_srx(stimer_get_tick(), DONGLE_CONNECTED_1ST_SUSPEND_SRX_TIMEOUT_US - 55);
                // pp_rf_start_srx2tx((unsigned char *)&rf_tx_packet.dma_len, stimer_get_tick(), DONGLE_CONNECTED_L_SRX_TIMEOUT_US + 6);
                ll_ctx.rx_over = false;

                #if TPSLL_N8K_FHSS_EN
                ll_ctx.is_slot0_rx_ok = false;
                #endif

                // #if  !TPSLL_NON_8K_SPP_NOT_SW_EN
                // rf_set_rx_maxlen(RF_PKT_DATA_LEN);
                // #endif
                }
                // if (ll_ctx.task_slot < 2)
                // {
                //     tlkapi_printf(APP_LOG_EN, "fno:%x slot:%x ch:%x\n", ll_ctx.fno_last, ll_ctx.task_slot, ch);
                // }
                // rf_set_rx_maxlen(RF_PKT_DATA_LEN * 2);
                pp_timer1_start((TIMER1_125_250_1ST_TIMER_US + ll_ctx.ir_btn_proc_us) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.early_stick));
            }
            else
            {
                #if TPSLL_SPP_NOT_SWITCH_EN
                if ((ll_ctx.report_rate & REPORT_RATE_4K) && tpsll_is_rf_busy()) {
                    if(rf_tx_packet.pkt_type == P24G_PKT_TYPE_SPP) {
                        rx_valid_flag = false;
                    } else {
                        rf_set_tx_rx_off_auto_mode();
                    }
                } else if (ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500)) {
                    if (tpsll_is_rf_busy()) {
                        rf_set_tx_rx_off_auto_mode();
                    }
                    ll_ctx.rx_over = true;
                    ll_ctx.tx_over = true;
                }
                if (rx_valid_flag && ll_ctx.rx_over && ll_ctx.tx_over)
                #endif
                {
                    // rf_set_ble_chn(CONN_CHN_NO);
                    // rf_set_ble_chn(chn_table[ll_ctx.task_slot & 3]);
                    #if TPSLL_N8K_FHSS_EN
                    ll_ctx.slot0_chn = chn_table[(ll_ctx.task_slot + ll_ctx.fno_last) & 3];
                    rf_set_ble_chn(ll_ctx.slot0_chn);
                    #else
                    rf_set_ble_chn(g_n8k_chn_tab[(ll_ctx.task_slot + ll_ctx.fno_last) & (NON_8K_CHN_NUM - 1)]);
                    #endif
                    // pp_rf_start_srx2tx((unsigned char *)&rf_tx_packet.dma_len, stimer_get_tick(), DONGLE_CONNECTED_SRX_TIMEOUT_US);
                    // dongle_fill_tx_data();
                    
                    // if(ll_ctx.report_rate & REPORT_RATE_1K)
                    // {
                    //     pp_rf_start_srx(stimer_get_tick(), DONGLE_CONNECTED_L_SRX_TIMEOUT_US + 30);
                    //     ll_ctx.rx_over = false;
                    // }
                    // else 

                    #if 0
                    if (ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500))
                    {
                        pp_rf_start_srx(stimer_get_tick(), DONGLE_CONNECTED_L_SRX_TIMEOUT_US + 23);
                        ll_ctx.rx_over = false;
                    }
                    else if (ll_ctx.report_rate & (REPORT_RATE_4K))
                    {
                        pp_rf_start_srx(stimer_get_tick(), ll_ctx.rx_win_us);
                        ll_ctx.rx_over = false;
                    }
                    else
                    {
                        pp_rf_start_srx(stimer_get_tick(), DONGLE_CONNECTED_L_SRX_TIMEOUT_US);
                        ll_ctx.rx_over = false;
                    }
                    #endif

                    if (ll_inf.paired_dev_type & DEVICE_TYPE_MS) {
                        if (ll_ctx.report_rate & REPORT_RATE_2K) {
                            pp_rf_start_srx(ll_ctx.last_stick + 25 * RF_SYSTEM_TIMER_TICK_1US, ll_ctx.rx_win_us);
                        } else {
                            pp_rf_start_srx(stimer_get_tick(), ll_ctx.rx_win_us);
                        }
                    } else {
                        pp_rf_start_srx(stimer_get_tick(), ll_ctx.rx_win_us);
                    }
                    ll_ctx.rx_over = false;

                    // #if  !TPSLL_NON_8K_SPP_NOT_SW_EN
                    // rf_set_rx_maxlen(RF_MAX_LEN);
                    // #endif
                }
                if(ll_ctx.report_rate & REPORT_RATE_4K)
                {
                    pp_timer1_start((73) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick) - 5);
                }else{
                    pp_timer1_start(SLOT_SYNC_OFFSET_TIME_US_500_1K_2K * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick) - 5);
                }
            }
            
                #if TPSLL_FAST_SETTLE_CAL_FUN_EN
                }
                else
                {
                    if ((ll_ctx.report_rate & (REPORT_RATE_2K)))
                    {
                        tpsll_fast_settle_dynamic_calib(3);
                    }
                    else
                    {
                        tpsll_fast_settle_dynamic_calib(1);
                    }
                }
                #endif
        }
        #if !TPSLL_SPP_NOT_SWITCH_EN
        else{
            tpsll_stimer_spp_mode_handler();
        }
        #endif

        if (ll_ctx.report_rate == REPORT_RATE_8K)
        {
            ms_idx = (ms_idx + 2) & 3;
        }
        else
        {
            if (ll_ctx.spp_cnt >= SPP_MAX_CONT_CNT)
            {
                ll_ctx.nspp_cnt = 0;
            }
        }
        ll_ctx.ack_report >>= 2;
    }

    // ll_ctx.task_slot = TASK_SLOT0;

    if (ll_ctx.conn_ignore_data_cnt) {
        ll_ctx.conn_ignore_data_cnt--;
    }

    #if TPSLL_WHEEL_TEST_EN
    if(g_test_tick && clock_time_exceed(g_test_tick,  5 * 1000))
    {
        static int8_t cnt = 0;

        g_test_tick = clock_time() | 1;

        // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
        if(cnt == 0) {
            cnt = 1;
            g_wheel_test = 20;
        } else if(cnt == 1) {
            cnt = 0;
            g_wheel_test = -20;
        }
    }
    #endif

    #if TPSLL_KEEP_ALIVE_NOTIFY_EN
    if (ll_ctx.is_keep_alive && (ll_ctx.keep_alive_expect_fno == ll_ctx.fno_last)) {
        ll_ctx.keep_alive_expect_fno_last = ll_ctx.keep_alive_expect_fno;
    }
    #endif

    #if TPSLL_KB_ACTIVE_PM_OPTM_EN
    if(ll_inf.paired_dev_type & DEVICE_TYPE_KB)
    {
        if (ll_ctx.report_rate & (REPORT_RATE_4K | REPORT_RATE_2K)) {
            if (!tpsll_all_fifo_is_empty()) {
                ll_ctx.kb_alive_intervals_cnt = 0;
                ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
            }
            uint32_t kb_alive_time_steps = ll_ctx.kb_alive_intervals_cnt >> ll_ctx.kb_alive_16ms_intervals_shift;

            if (kb_alive_time_steps < (TPSLL_KB_ALIVE_MAX_STAGED_TIME_MS / TPSLL_KB_ALIVE_TIME_STEP_MS)) {
                if (ll_ctx.kb_alive_intervals_cnt && ((ll_ctx.kb_alive_intervals_cnt & ll_ctx.kb_alive_16ms_intervals_msk) == 0)) {
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
    }
    #endif /* end of TPSLL_KB_ACTIVE_PM_OPTM_EN */
}

_attribute_ram_code_sec_ static void pairing_stimer_irq_handler(void)
{
    #if TPSLL_NEW_PAIR_EN
////////////////////////////////////////////////////////////////////
    #if TPSLL_RECON_RSSI_SCAN_EN
    if (ll_ctx.recon_rssi_scan_step) {
        ll_ctx.recon_chn_idx = 0;
        ll_ctx.recon_rx_start_tick = 0;
        return;
    }
    #endif

    if (tpsll_is_rf_busy() || !ll_ctx.rx_over) {
        return;
    }

    #if TPSLL_RECON_RSSI_SCAN_EN
    uint32_t recon_rx_win_us = DONGLE_RECONN_LONG_RX_WIN;

    if (!ll_ctx.recon_rx_start_tick) {
        ll_ctx.recon_rx_start_tick = clock_time() | 1;
    }
    if (ll_ctx.recon_chn_idx > 2) {
        if (clock_time_exceed(ll_ctx.recon_rx_start_tick, DONGLE_RECONN_LONG_RX_WIN * 3 + 200 * 2)) {
            ll_ctx.recon_rx_start_tick = 0;
            #if DONGLE_SUSPEND_SUPPORT
            if(app_get_usb_wake_status() == USB_ENTER_SUSPEND && ll_ctx.suspend_allow)
            {
                if(app_get_usb_wake_status() == USB_ENTER_SUSPEND && ll_ctx.suspend_allow )
                {
                    ll_ctx.suspend_enable = 1;
                }
            }
            #endif /* end of DONGLE_SUSPEND_SUPPORT */
            ll_ctx.recon_rssi_scan_step = 1;
            return;
        } else { //enough time left
            recon_rx_win_us = ((ll_ctx.recon_rx_start_tick + (DONGLE_RECONN_LONG_RX_WIN * 3 + 200 * 2) * SYSTEM_TIMER_TICK_1US) - clock_time()) / SYSTEM_TIMER_TICK_1US;

            if (recon_rx_win_us > (DONGLE_RECONN_LONG_RX_WIN * 3 / 2)) {
                recon_rx_win_us = DONGLE_RECONN_LONG_RX_WIN;
            }
        }
    }
    #endif

    if (ll_ctx.pair_st == STATE_PAIR_INIT) {
        #if TPSLL_RECON_RSSI_SCAN_EN
        {
            #if TPSLL_8K_CHN_EXPAND_EN
            ll_ctx.reconn_chn = reconn_chn_table[ll_ctx.recon_chn_idx % 3];
            #else
            ll_ctx.reconn_chn = reconn_chn_table[ll_ctx.recon_chn_idx % TPSLL_RECON_CHN_NUM];
            #endif
            rf_set_ble_chn(ll_ctx.reconn_chn);
        }
        #else
        rf_set_ble_chn(reconn_chn_table[ll_ctx.recon_chn_idx & (RECONN_CHN_NUM - 1)]);
        #endif
        ll_ctx.recon_chn_idx++;

        rf_access_code_comm(ll_ctx.pair_acc);
        rf_set_rx_maxlen(TPSLL_PAIR_REQ_MAX_LEN);
        write_reg8(0x17051a,38);
    }
    else if (ll_ctx.pair_st == STATE_PAIR_RSP) {
        rf_access_code_comm(~ll_ctx.pair_acc);
        rf_set_rx_maxlen(TPSLL_PAIR_CON_REQ_LEN);
        recon_rx_win_us = DONGLE_PAIRING_SRX_TIMEOUT_US;
    }

    #if TPSLL_RECON_RSSI_SCAN_EN
    pp_rf_start_srx(stimer_get_tick(), recon_rx_win_us);
    #else
    pp_rf_start_srx(stimer_get_tick(), DONGLE_RECONN_LONG_RX_WIN);
    #endif

    ll_ctx.rx_over = false;
    ll_ctx.pair_step = PAIR_STEP1;
////////////////////////////////////////////////////////////////////
    #else /* end of TPSLL_NEW_PAIR_EN */
    if (ll_ctx.pair_st == STATE_PAIR_INIT)
    {
        if(++ll_ctx.pr_cnt % 40)
        {
            return;
        }
        if (0 != (read_reg8(0x170224) & 0x7))
        {
            rf_set_tx_rx_off_auto_mode();
        }
        rf_set_ble_chn(PAIR_CHN_NO);
        rf_access_code_comm(ll_ctx.pair_acc);
        // write_reg8(0x17021e,0x80);//
        pp_rf_start_stx2rx((unsigned char *)&rf_tx_packet.dma_len, ll_ctx.last_stick + PARING_STIMER_TO_TX_US * RF_SYSTEM_TIMER_TICK_1US, DONGLE_PAIRING_SRX_TIMEOUT_US);

        //        rf_packet_t *pt = &rf_tx_packet;
        if (ll_ctx.pair_st == STATE_PAIR_INIT)
        {
            pp_rf_set_tx_len(5);
            rf_tx_packet.buf[0] = CMD_PAIR_REQ;
            rf_tx_packet.buf[PAIR_REQ_ENCRYPT_KEY_OFFSET]     = (ll_ctx.pair_encrypt_key >> 24) & 0xFF;
            rf_tx_packet.buf[PAIR_REQ_ENCRYPT_KEY_OFFSET + 1] = (ll_ctx.pair_encrypt_key >> 16) & 0xFF;
            rf_tx_packet.buf[PAIR_REQ_ENCRYPT_KEY_OFFSET + 2] = (ll_ctx.pair_encrypt_key >>  8) & 0xFF;
            rf_tx_packet.buf[PAIR_REQ_ENCRYPT_KEY_OFFSET + 3] = (ll_ctx.pair_encrypt_key >>  0) & 0xFF;
        }

        rf_set_rx_maxlen(5);
        ll_ctx.pair_cnt = 0;

        ll_ctx.pair_step = PAIR_STEP1;
    }
    else if (ll_ctx.pair_st == STATE_PAIR_RSP)
    {
        if (ll_ctx.pair_cnt > PAIRING_RSP_MAX_RETRY_CNT)
        {
            tpsll_enable_pairing(true);
            return;
        }

        if (0 != (read_reg8(0x170224) & 0x7))
        {
            rf_set_tx_rx_off_auto_mode();
        }

        rf_set_ble_chn(PAIR_CHN_NO);
        rf_access_code_comm(~ll_ctx.pair_acc);
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);


        unsigned int tx_moment = ll_ctx.last_stick + PARING_STIMER_TO_TX_US * RF_SYSTEM_TIMER_TICK_1US;
        if (tick1_exceed_tick2(tx_moment, stimer_get_tick()))
        {
            pp_rf_start_stx2rx((unsigned char *)&rf_tx_packet.dma_len, ll_ctx.last_stick + PARING_STIMER_TO_TX_US * RF_SYSTEM_TIMER_TICK_1US, DONGLE_PAIRING_SRX_TIMEOUT_US + 10);
            conn_req_packet_t *pt = (conn_req_packet_t *)&rf_tx_packet;

            pp_rf_set_tx_len(15);
            pt->cmd = CMD_CONN_REQ;
            tmemcpy(pt->mac, ll_inf.mac, MAC_ADDR_LEN);
            // app_halt_system(0x9988, "pt mac:%x %x %x  %x %x %x\n", pt->mac[0], pt->mac[1],pt->mac[2],pt->mac[3],pt->mac[4],pt->mac[5]);
    #if RSSI_SCAN_EN
        tmemcpy(pt->chn_table, chn_table, CHN_TABL_LEN);
    #endif
            // pt->chn_table[0] = 0x1;
            // pt->chn_table[1] = 0x2;
            // pt->chn_table[2] = 0x3;
            // pt->chn_table[3] = 0x4;

            pt->fno_last = ll_ctx.fno_last;

            if (ll_inf.connecting_dev & DEVICE_TYPE_KB)
            {
                pt->tag = ll_ctx.ktag;
            }
            else if (ll_inf.connecting_dev & DEVICE_TYPE_MS)
            {
                pt->tag = ll_ctx.mtag;
            }
            // rf_tx_packet.buf[3] = ll_inf.side_id & 0xff;
            // rf_set_rx_maxlen(12);
            rf_set_rx_maxlen(13);
        }

        pp_timer1_start((135) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));

        ll_ctx.pair_cnt++;
    }
    // app_fill_paring_pkt();
    #endif /* end of TPSLL_NEW_PAIR_EN #else */
}

_attribute_ram_code_sec_ static void reconnect_stimer_irq_handler(void)
{
    #if TPSLL_NEW_RECON_EN
////////////////////////////////////////////////////////////////////
    #if TPSLL_RECON_RSSI_SCAN_EN
    if (ll_ctx.recon_rssi_scan_step) {
        ll_ctx.recon_chn_idx = 0;
        ll_ctx.recon_rx_start_tick = 0;

        if (ll_ctx.recon_start_ref_tick && clock_time_exceed(ll_ctx.recon_start_ref_tick, ll_ctx.recon_enter_pm_optm_start_time_ms * 1000)) {
            ll_ctx.recon_enter_pm_optm_cnt = 1;
            ll_ctx.recon_start_ref_tick = 0;
        }
        return;
    }
    #endif

    if (tpsll_is_rf_busy() || !ll_ctx.rx_over) {
        return;
    }

    #if TPSLL_RECON_RSSI_SCAN_EN
    uint32_t recon_rx_win_us = DONGLE_RECONN_LONG_RX_WIN;

    if (!ll_ctx.recon_rx_start_tick) {
        ll_ctx.recon_rx_start_tick = clock_time() | 1;
    }


#if DONGLE_SUSPEND_SUPPORT
    if (s_app_cb && s_app_cb->app_get_usb_status)
    {
        if(s_app_cb->app_get_usb_status() == USB_ENTER_SUSPEND)
            recon_rx_win_us = DONGLE_RECONN_LONG_RX_WIN - 6000;
    }
    if(ll_ctx.recon_chn_idx > 0)
    {
        if (clock_time_exceed(ll_ctx.recon_rx_start_tick, DONGLE_RECONN_LONG_RX_WIN - 6000)) {
            ll_ctx.recon_rx_start_tick = 0;

            if (s_app_cb && s_app_cb->app_get_usb_status)
            {
                if(s_app_cb->app_get_usb_status() == USB_ENTER_SUSPEND && ll_ctx.suspend_allow)
                {
                    ll_ctx.suspend_enable = 1;
                    ll_ctx.recon_rssi_scan_step = 0;
                    ll_ctx.recon_rx_start_tick = 0;

                    return;
                }
            }
        }
    }
#endif /* end of DONGLE_SUSPEND_SUPPORT */
    if (s_app_cb && s_app_cb->app_get_usb_status)
    {
        if(s_app_cb->app_get_usb_status() != USB_ENTER_SUSPEND)
        {
            if (ll_ctx.recon_chn_idx > 2) {
                if (clock_time_exceed(ll_ctx.recon_rx_start_tick, DONGLE_RECONN_LONG_RX_WIN * 3 + 200 * 2)) {
                    ll_ctx.recon_rx_start_tick = 0;
                    // #if DONGLE_SUSPEND_SUPPORT
                    // if (s_app_cb && s_app_cb->app_get_usb_status)
                    // {
                    //     if(s_app_cb->app_get_usb_status() == USB_ENTER_SUSPEND && ll_ctx.suspend_allow)
                    //     {
                    //         if(s_app_cb->app_get_usb_status() == USB_ENTER_SUSPEND && ll_ctx.suspend_allow )
                    //         {
                    //             ll_ctx.suspend_enable = 1;
                    //         }
                    //     }
                    // }
                    // #endif /* end of DONGLE_SUSPEND_SUPPORT */
                    if (!ll_ctx.recon_start_ref_tick) {
                        if (ll_ctx.recon_enter_pm_optm_cnt < 200) {
                            ll_ctx.recon_enter_pm_optm_cnt++;
                        } else {
                            ll_ctx.recon_enter_pm_optm_cnt = 1;
                        }
                        if ((ll_ctx.recon_enter_pm_optm_cnt % TPSLL_RECON_OPTM_CNT_REMAINDER) == 0) {
                            ll_ctx.recon_rssi_scan_step = 1;
                        } else {
                            ll_ctx.recon_rx_for_pkt = false;
                        }
                        ll_ctx.recon_chn_idx = 0;
                        ll_ctx.recon_rx_start_tick = 0;
                    } else {
                        ll_ctx.recon_rssi_scan_step = 1;
                    }
                    return;
                } else { //enough time left
                    recon_rx_win_us = ((ll_ctx.recon_rx_start_tick + (DONGLE_RECONN_LONG_RX_WIN * 3 + 200 * 2) * SYSTEM_TIMER_TICK_1US) - clock_time()) / SYSTEM_TIMER_TICK_1US;
                    if (recon_rx_win_us > (DONGLE_RECONN_LONG_RX_WIN * 3 / 2)) {
                        recon_rx_win_us = DONGLE_RECONN_LONG_RX_WIN;
                    }
                }
            }
        }
    }
    #endif

    #if TPSLL_RECON_RSSI_SCAN_EN
    {
        #if TPSLL_8K_CHN_EXPAND_EN
        ll_ctx.reconn_chn = reconn_chn_table[ll_ctx.recon_chn_idx % 3];
        #else
        ll_ctx.reconn_chn = reconn_chn_table[ll_ctx.recon_chn_idx % TPSLL_RECON_CHN_NUM];
        #endif
        rf_set_ble_chn(ll_ctx.reconn_chn);
    }
    #else
    rf_set_ble_chn(reconn_chn_table[ll_ctx.recon_chn_idx & (RECONN_CHN_NUM - 1)]);
    #endif
    ll_ctx.recon_chn_idx++;
    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
    // pairing SRX


    // pp_rf_start_srx(stimer_get_tick(), KM_RECONN_RX_WIN);
    {
    #if TPSLL_RECON_RSSI_SCAN_EN
    pp_rf_start_srx(stimer_get_tick(), recon_rx_win_us);
    #else
    pp_rf_start_srx(stimer_get_tick(), DONGLE_RECONN_LONG_RX_WIN);
    #endif
    write_reg8(0x17051a,38);
    }
    // pp_rf_start_srx(stimer_get_tick(), 300000);
    rf_set_rx_maxlen(6);

    ll_ctx.rx_over = false;
    ll_ctx.recon_step = RECON_STEP1;
////////////////////////////////////////////////////////////////////
    #else /* end of TPSLL_NEW_RECON_EN */

#if DONGLE_SUSPEND_SUPPORT
    if (s_app_cb && s_app_cb->app_get_usb_status)
    {
        if(s_app_cb->app_get_usb_status() == USB_ENTER_SUSPEND && ll_ctx.suspend_allow)
        {
            ll_ctx.task_slot = TASK_SLOT0;

            if (0 != (read_reg8(0x170224) & 0x7))
            {
                rf_set_tx_rx_off_auto_mode();
            }

            // rf_set_ble_chn(PAIR_CHN);
            rf_set_ble_chn(g_fhss_valid_chn_tab[TPSLL_FHSS_CHN_MAX_NUM - 1]);
            // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);

            unsigned int tx_moment = ll_ctx.last_stick + PARING_STIMER_TO_TX_US * RF_SYSTEM_TIMER_TICK_1US;
            if (tick1_exceed_tick2(tx_moment, stimer_get_tick()))
            {
                // gpio_toggle(dbg_io[DBG_IO_IDX1]);
                // if (!(ll_ctx.report_rate & REPORT_RATE_125))
                {
                    pp_rf_start_stx2rx((unsigned char *)&rf_tx_packet.dma_len, ll_ctx.last_stick + PARING_STIMER_TO_TX_US * RF_SYSTEM_TIMER_TICK_1US, DONGLE_PAIRING_SRX_TIMEOUT_US + 5);
                    // pp_rf_start_stx();
                    // rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, ll_ctx.last_stick + PARING_STIMER_TO_TX_US * RF_SYSTEM_TIMER_TICK_1US);
                }
                pp_timer1_start(DONGLE_SUSPEND_TIMER1_US * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick) - 5);

                reconn_req_packet_t *pt = (reconn_req_packet_t *)&rf_tx_packet;

                pp_rf_set_tx_len(9);
                pt->cmd = CMD_RECONN_REQ;

        #if RSSI_SCAN_EN
                tmemcpy(pt->chn_table, chn_table, CHN_TABL_LEN);
                // tmemcpy(pt->chn_table, hop_ctrl.chn_table, CHN_TABL_LEN);
        #endif


                pt->fno_last = ll_ctx.fno_last;

                if (ll_inf.paired_dev_type & DEVICE_TYPE_KB)
                {
                    pt->tag = ll_ctx.ktag;
                }
                else if (ll_inf.paired_dev_type & DEVICE_TYPE_MS)
                {
                    pt->tag = ll_ctx.mtag;
                }
                // rf_tx_packet.buf[3] = ll_inf.side_id & 0xff;
                rf_set_rx_maxlen(6);
            }
            ll_ctx.pair_cnt++;
            return;
        }
    }
#endif /* end of DONGLE_SUSPEND_SUPPORT */

    ll_ctx.pr_cnt++;

    if (0 != (read_reg8(0x170224) & 0x7))
    {
        rf_set_tx_rx_off_auto_mode();
    }

    static unsigned char reconn_chn_idx = 0;

    #if TPSLL_RECON_RSSI_SCAN_EN
    // #if TPSLL_RR_CHG_WITH_RECON_EN
    // if (ll_ctx.rr_chg_with_recon) {
    //     rf_set_ble_chn(reconn_chn_table[reconn_chn_idx & (RECONN_CHN_NUM - 1)]);
    //     reconn_chn_idx++;
    // } else
    // #endif
    {
        ll_ctx.reconn_chn = g_fhss_valid_chn_tab[ll_ctx.recon_chn_idx-- % TPSLL_FHSS_CHN_MAX_NUM];
        rf_set_ble_chn(ll_ctx.reconn_chn);
    }
    #else
    // rf_set_ble_chn(PAIR_CHN);
    // rf_set_ble_chn(ll_ctx.reconn_chn);
    rf_set_ble_chn(reconn_chn_table[reconn_chn_idx & (RECONN_CHN_NUM - 1)]);
    reconn_chn_idx++;
    #endif
    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);

    unsigned int tx_moment = ll_ctx.last_stick + PARING_STIMER_TO_TX_US * RF_SYSTEM_TIMER_TICK_1US;
    if (tick1_exceed_tick2(tx_moment, stimer_get_tick()))
    {
        // if (!(ll_ctx.report_rate & REPORT_RATE_125))
        {
            pp_rf_start_stx2rx((unsigned char *)&rf_tx_packet.dma_len, ll_ctx.last_stick + PARING_STIMER_TO_TX_US * RF_SYSTEM_TIMER_TICK_1US, DONGLE_PAIRING_SRX_TIMEOUT_US + 5);
        }
        // else
        // { // 125Hz
        //     pp_rf_start_stx2rx((unsigned char *)&rf_tx_packet.dma_len, ll_ctx.last_stick + PARING_STIMER_TO_TX_US * RF_SYSTEM_TIMER_TICK_1US, DONGLE_2M_PAIRING_SRX_TIMEOUT_US);
        // }
        reconn_req_packet_t *pt = (reconn_req_packet_t *)&rf_tx_packet;

        pp_rf_set_tx_len(9);
        pt->cmd = CMD_RECONN_REQ;
        // pt->tag = ll_ctx.ktag; //use ktag as reconnect tag
        // tmemcpy(pt->mac, ll_inf.mac, MAC_ADDR_LEN);
        // app_halt_system(0x9988, "pt mac:%x %x %x  %x %x %x\n", pt->mac[0], pt->mac[1],pt->mac[2],pt->mac[3],pt->mac[4],pt->mac[5]);
#if RSSI_SCAN_EN
        tmemcpy(pt->chn_table, chn_table, CHN_TABL_LEN);
        // tmemcpy(pt->chn_table, hop_ctrl.chn_table, CHN_TABL_LEN);
#endif
        // tlkapi_send_string_data(APP_LOG_EN, "chn tab", pt->chn_table, 4);
        // pt->chn_table[0] = 0x1;
        // pt->chn_table[1] = 0x2;
        // pt->chn_table[2] = 0x3;
        // pt->chn_table[3] = 0x4;

        pt->fno_last = ll_ctx.fno_last;

        if (ll_inf.paired_dev_type & DEVICE_TYPE_KB)
        {
            pt->tag = ll_ctx.ktag;
        }
        else if (ll_inf.paired_dev_type & DEVICE_TYPE_MS)
        {
            pt->tag = ll_ctx.mtag;
        }
        // rf_tx_packet.buf[3] = ll_inf.side_id & 0xff;
        #if TPSLL_RECON_RSSI_SCAN_EN
        // #if TPSLL_RR_CHG_WITH_RECON_EN
        // if (ll_ctx.rr_chg_with_recon) {
        //     rf_set_rx_maxlen(6);
        // } else
        // #endif
        rf_set_rx_maxlen(9);
        #else
        rf_set_rx_maxlen(6);
        #endif
    }
    else
    {
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
        // clic_interrupt_disable(IRQ_SYSTIMER);
        // tlkapi_printf(APP_LOG_EN, "tx_mom: %x tick:%x\n", tx_moment, stimer_get_tick());
    }

    ll_ctx.recon_step = RECON_STEP1;
    pp_timer1_start((105) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));

    ll_ctx.pair_cnt++;

    // if (!((ll_ctx.recon_chn_idx + 1) % TPSLL_FHSS_CHN_MAX_NUM)) {
    //     DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
    // }
    // app_fill_paring_pkt();

    #endif /* end of TPSLL_NEW_RECON_EN #else*/
}

_attribute_ram_code_sec_ uint8_t tpsll_stimer_irq_handler(void)
{
    DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
    if (stimer_get_irq_status_d25f(FLD_SYSTEM_IRQ_D25F))
    {
        ll_ctx.fno_last = ll_ctx.fno;
        ll_ctx.last_stick = ll_ctx.stick;
        ll_ctx.last_e_stick = ll_ctx.early_stick;

        if (ll_ctx.stk_state == STATE_CONNECTED)
        {
            //connected
            connected_stimer_irq_handler();
        } else if (ll_ctx.stk_state == STATE_RECONNECT)
        {
            //reconnect
            reconnect_stimer_irq_handler();
        }
        else
        {
            // pairing
            pairing_stimer_irq_handler();
        }

        if (ll_ctx.lost_cnt)
        {
            ll_ctx.lost_cnt++;

            #if TPSLL_CONN_QUICK_LOST_TIME_US
            if (ll_ctx.quick_discon_active_tick && clock_time_exceed(ll_ctx.quick_discon_active_tick, TPSLL_CONN_QUICK_LOST_TIME_US)) {
                ll_ctx.quick_discon_active_tick = 0;
            }
            #endif

            if (ll_ctx.lost_cnt > ((ll_ctx.quick_discon_active_tick && !(ll_ctx.report_rate & REPORT_RATE_8K)) ? (ll_ctx.lost_cnt_max / 4) : ll_ctx.lost_cnt_max))
            {
                tpsll_connect_lost();
                stack_evt_post_from_isr(TPSLL_EVT_DEV_DISCONNECTED, (void *)ll_inf.peer_mac, MAC_ADDR_LEN);
            }
        }

        app_set_next_stimer_anchor();
        stimer_clr_irq_status_d25f(FLD_SYSTEM_IRQ_D25F);
        wd_clear();//Temporarily placed here.

        #if TPSLL_RR_CHG_WITH_RECON_EN
        if (ll_ctx.rr_chg_recon_time_tick &&
            clock_time_exceed(ll_ctx.rr_chg_recon_time_tick, TPSLL_RR_CHG_RECON_TIMEOUT_US)) {
            ll_ctx.rr_chg_recon_time_tick = 0;
            ll_ctx.rr_chg_with_recon = RR_RECON_STEP_NONE;
            spp_send_reset();
            stack_evt_post_from_isr(TPSLL_EVT_DEV_DISCONNECTED, (void *)ll_inf.peer_mac, MAC_ADDR_LEN);
        }
        #endif

        #if TPSLL_FHSS_EN
        if (ll_ctx.stk_state == STATE_CONNECTED) {
            #if TPSLL_N8K_FHSS_EN
            if (tpsll_is_channel_update_slot() && !ll_ctx.conn_ignore_data_cnt
                && pp_fifo_is_empty(&spp_fifo) && !ll_ctx.is_rr_changing
                #if !TPSLL_SPP_NOT_SWITCH_EN
                && !ll_ctx.spp_mode
                #endif
            )
            #else
            if (tpsll_is_channel_update_slot() && (ll_ctx.report_rate & REPORT_RATE_8K) && !ll_ctx.conn_ignore_data_cnt
                && pp_fifo_is_empty(&spp_fifo) && !ll_ctx.is_rr_changing && !ll_ctx.pending_spp_mode
                #if !TPSLL_SPP_NOT_SWITCH_EN
                && !ll_ctx.spp_mode
                #endif
            )
            #endif
            {
                #if TPSLL_FHSS_REPORT_PER_EN
                if(ll_ctx.report_rate & REPORT_RATE_8K)
                {
                    tpsll_update_report_per();
                }
                #endif
                tpsll_channel_update_per();
                tpsll_channel_update_map();
                if (g_hop_map_next && (g_hop_map_cur != g_hop_map_next)) {
                    tpsll_afh_channel_gen_idx_tab(g_hop_map_next, (uint8_t *)(void *)g_hop_chn_idx_tab_next);

                    #if TPSLL_FHSS_FIX_OLD_CHN_IDX_EN
                    tpsll_fhss_fix_old_chn_index();
                    #endif

                }
            } else if (tpsll_is_afh_slot()){
                // return;
                if (g_hop_map_next && (g_hop_map_cur != g_hop_map_next)) {
                    g_hop_map_cur = g_hop_map_next;
                    chn_table[0] = g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[0]];
                    chn_table[1] = g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[1]];
                    chn_table[2] = g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[2]];
                    #if TPSLL_8K_CHN_EXPAND_EN
                    chn_table[3] = g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[3]];
                    chn_table[4] = g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[4]];
                    chn_table[5] = g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[5]];
                    // tlkapi_printf(APP_LOG_EN, "%d(%d)%d(%d)%d(%d)%d-(%d)%d(%d)%d(%d)%d\n", ll_ctx.is_keep_alive, chn_table[0], g_fhss_chn_per[g_hop_chn_idx_tab_next[0]],
                    //         chn_table[1], g_fhss_chn_per[g_hop_chn_idx_tab_next[1]], chn_table[2],  g_fhss_chn_per[g_hop_chn_idx_tab_next[2]],
                    //         chn_table[3], g_fhss_chn_per[g_hop_chn_idx_tab_next[3]], chn_table[4],  g_fhss_chn_per[g_hop_chn_idx_tab_next[4]],
                    //         chn_table[5], g_fhss_chn_per[g_hop_chn_idx_tab_next[5]], chn_table[6],  g_fhss_chn_per[g_hop_chn_idx_tab_next[6]]);
                    #else
                    #if TPSLL_N8K_CHN_EXPAND_EN
                    if (ll_ctx.report_rate & REPORT_RATE_NOT_8K_MSK) {
                        tpsll_n8k_chn_expand();
                    }
                    #else
                    #if (CHN_TABL_LEN == 4)
                    chn_table[3] = chn_table[1];
                    #endif
                    #endif
                    // tlkapi_printf(APP_LOG_EN, "%d(%d)%d(%d)%d(%d)%d\n", ll_ctx.is_keep_alive, chn_table[0], g_fhss_chn_per[g_hop_chn_idx_tab_next[0]],
                    //         chn_table[1], g_fhss_chn_per[g_hop_chn_idx_tab_next[1]], chn_table[2],  g_fhss_chn_per[g_hop_chn_idx_tab_next[2]]);
                    // tlkapi_printf(APP_LOG_EN, "%d(%d)%d(%d)%d(%d)%d-(%d)(%d)(%d)(%d)(%d)\n", ll_ctx.is_keep_alive, chn_table[0], g_fhss_chn_per[g_hop_chn_idx_tab_next[0]],
                    //         chn_table[1], g_fhss_chn_per[g_hop_chn_idx_tab_next[1]], chn_table[2],  g_fhss_chn_per[g_hop_chn_idx_tab_next[2]],
                    //     chn_table[3], chn_table[4], chn_table[5], chn_table[6], chn_table[7]);
                    #endif
            // tlkapi_printf(1, "hop_chns:%d-%d-%d\n", g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[0]],
            //         g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[1]], g_fhss_valid_chn_tab[g_hop_chn_idx_tab_next[2]]);
                    // for (int i = 0; i < g_hop_chn_idx_tab_next[0]; i++) {
                    //     DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
                    // }
                    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
                    // for (int i = 0; i < g_hop_chn_idx_tab_next[1]; i++) {
                    //     DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
                    // }
                    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
                    // for (int i = 0; i < g_hop_chn_idx_tab_next[2]; i++) {
                    //     DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
                    // }
                    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX0]);
                }
                tpsll_channel_statistic(0xff, 0);
                ll_ctx.tbl_update = false;
                // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
            }
        }
        #endif
    }

    ll_ctx.wfi_flag = true;

// if ((ll_ctx.fno_last & 0x03) == 0) {
// DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
// }
    return TLK_SUCCESS;
}



#if TPSLL_LOCAL_MS_DATA_TEST_EN
_attribute_ram_code_sec_ void timer0_irq_handler(void)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    if (timer_get_irq_status(FLD_TMR0_MODE_IRQ)) {
        timer_clr_irq_status(FLD_TMR0_MODE_IRQ); //clear irq status
        pp_timer0_stop();
        tpsll_mouse_report_data_test(buf);
        if (ll_ctx.local_ms_data_test) {
            pp_timer0_start(200 * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));
        }
        DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX2]);
    }
}
PLIC_ISR_REGISTER(timer0_irq_handler, IRQ_TIMER0)
#endif

_attribute_ram_code_sec_ void timer1_irq_handler_8k(void)
{

    // while(0 != (read_reg8(0x170224) & 0x7));
    #if TPSLL_SPP_NOT_SWITCH_EN
    rf_packet_t  *pr=(rf_packet_t *)rx_packet;
    #endif

    if (ll_ctx.task_slot == TASK_SLOT0)
    {
        // SLOT1
        #if TPSLL_SPP_NOT_SWITCH_EN
        // uint8_t crc_ok = !(reg_rf_dec_err & 0x10);
        // if(p->pkt_type == P24G_PKT_TYPE_SPP && crc_ok)
        if(pr->pkt_type == P24G_PKT_TYPE_SPP)
        {

            #if 0
            uint8_t rf_status = (read_reg8(0x170224) & 0x7);
            static uint8_t rcnt = 0;
            if(rcnt < 5)
            {
                rcnt++;
                // tlkapi_printf(APP_LOG_EN, "rf_status:%x %d %d\n", rf_status, p->dma_len, p->rf_len);
            }
            #endif
            // if(ll_ctx.step_3t1r == STEP_3T1R_1T_D_1R)
            {
                app_rf_8k_abnormal_handle();
            }
        }else{
            rf_set_ble_chn(chn_list[STEP_3T1R_2T_D_2R]);
            // ZW_TG(GPIO_PH0);

            uint8_t rf_fsm = read_reg8(0x170224) & 0x7;
            bool rf_busy = (rf_fsm != 0);

            #if TPSLL_NEW_8K_RF_TIMING_EN

            if ((rf_busy && pr->rf_len != TPSLL_RX1_PKT_LEN) || (rf_busy && wait_rf_fsm_idle_timeout(3) > 0)) 
            #else
            if ((rf_busy && pr->rf_len != RF_PKT_DATA_LEN) || (rf_busy && wait_rf_fsm_idle_timeout(3) > 0)) 

            #endif
            {
                rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
                rf_set_ble_chn(chn_list[STEP_3T1R_2T_D_2R]);
            }
            // rf_set_ble_chn(chn_list[STEP_3T1R_2T_D_2R]);
            
            pp_rf_start_srx(stimer_get_tick(), DONGLE_CONNECTED_SRX_TIMEOUT_US);
            ll_ctx.rx_over = false;
            #if TPSLL_AVOID_WIFI_CHN_USE_RX3_EN
            ll_ctx.is_2r_to = false;
            #endif

            #if TPSLL_NEW_8K_RF_TIMING_EN
            rf_set_rx_maxlen(18);
            #else
            rf_set_rx_maxlen(RF_PKT_DATA_LEN * 2);
            #endif
        }
        #else
        rf_set_ble_chn(chn_list[STEP_3T1R_2T_D_2R]);
        
        if (wait_rf_fsm_idle_timeout(3)) // 8k: 2.25us 4k:300ns
        {
            rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
        }

        pp_rf_start_srx(stimer_get_tick(), DONGLE_CONNECTED_SRX_TIMEOUT_US);
        rf_set_rx_maxlen(RF_PKT_DATA_LEN * 2);
        #endif

        // D_2R
        ll_ctx.task_slot = TASK_SLOT1;

        // unsigned int r = core_interrupt_disable();
        // unsigned int t0 = APP_SLOT2_START_TIME_US * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); //change time for SLOT2
        unsigned int t0 = ll_ctx.slot2_us * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); // change time for SLOT2
        pp_timer1_start(t0);
        // core_restore_interrupt(r);
    }
    else if ((ll_ctx.task_slot == TASK_SLOT1))
    {
        // SLOT2
        // rf_set_ble_chn(chn_list[STEP_3T1R_3T_D_3R]);
    #if TPSLL_AVOID_WIFI_CHN_USE_RX3_EN

        if(!ll_ctx.is_2r_to)
        {
            ll_ctx.is_2r_success_cnt++;
        }
        if (ll_ctx.is_2r_success_cnt >= TPSLL_AVOID_WIFI_CHN_SCAN_CNT)
        {
            ll_ctx.is_2r_success_cnt = 0;
            ll_ctx.do_rssi_scan = true;

        }
        else
        {
            ll_ctx.do_rssi_scan = false;
        }

        if (!ll_ctx.do_rssi_scan)
        {
            rf_set_ble_chn(chn_list[STEP_3T1R_3T_D_3R]);
        }
        else
        {
            tpsll_avoid_wifi_chn_use_rx3();
        }
        
    #else
        rf_set_ble_chn(chn_list[STEP_3T1R_3T_D_3R]);
    #endif
        // while(0 != (read_reg8(0x170224) & 0x7));
        // if (wait_rf_fsm_idle_timeout(3)) // 8k: 2.25us
        // {
        //     rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
        // }

        uint8_t rf_fsm = read_reg8(0x170224) & 0x7;
        bool rf_busy = (rf_fsm != 0);

    #if TPSLL_NEW_8K_RF_TIMING_EN
        if ((rf_busy && pr->rf_len != TPSLL_RX2_PKT_LEN) || (rf_busy && wait_rf_fsm_idle_timeout(3) > 0))
    #else
        if ((rf_busy && (pr->rf_len != (RF_PKT_DATA_LEN * 2))) || (rf_busy && wait_rf_fsm_idle_timeout(3) > 0))
    #endif
        {
            rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
            #if TPSLL_AVOID_WIFI_CHN_USE_RX3_EN
            if (!ll_ctx.do_rssi_scan)
            {
                rf_set_ble_chn(chn_list[STEP_3T1R_3T_D_3R]);
            }else{
                tpsll_avoid_wifi_chn_use_rx3();
            }
            #else
            rf_set_ble_chn(chn_list[STEP_3T1R_3T_D_3R]);
            #endif
            // rf_set_ble_chn(chn_list[STEP_3T1R_3T_D_3R]);         
        }

        pp_rf_start_srx(stimer_get_tick(), DONGLE_CONNECTED_SRX_TIMEOUT_US);
        
        ll_ctx.rx_over = false;
        rf_set_rx_maxlen(RF_PKT_DATA_LEN);
        // rf_set_ble_chn(15);
        // D_3R
        ll_ctx.task_slot = TASK_SLOT2;

        unsigned int r = core_interrupt_disable();
        unsigned int t0 = (APP_DONGLE_SLOT3_START_TIME_US + KM_3T_OFFSET_US) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); // change time for SLOT3
        pp_timer1_start(t0);
        core_restore_interrupt(r);

#if AFH_ENABLE
        if ((ll_ctx.fno_last & (AFH_INTERVAL - 1)) == 0)
        {
            // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
            check_and_update_mask();
            //DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
        }
#endif
        // calc_hop_channels(ll_ctx.fno_last + 1, chn_mask);
    }
    else if ((ll_ctx.task_slot == TASK_SLOT2))
    {
        // while(0 != (read_reg8(0x170224) & 0x7));

        #if TPSLL_AVOID_WIFI_CHN_USE_RX3_EN
        if(ll_ctx.do_rssi_scan)
        {
            rf_rx3_rssi_scan_param_restore();;
        }
        #endif
        if (wait_rf_fsm_idle_timeout(2)) // 8k: 1us
        {
            rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
        }
        #if TPSLL_AVOID_WIFI_CHN_USE_RX3_EN
        if (ll_ctx.do_rssi_scan)
        {
            rf_set_ble_chn(chn_list[STEP_3T1R_3T_D_3R]);
        }
        #endif
        // SLOT3
        // D_1T
        unsigned int tx_moment = ll_ctx.last_stick + (APP_SLOT3_1T_START_TIME_US + KM_3T_OFFSET_US) * RF_SYSTEM_TIMER_TICK_1US;
        if (tick1_exceed_tick2(tx_moment + 3 * RF_SYSTEM_TIMER_TICK_1US, stimer_get_tick()))
        {
//            rf_set_vant_power_trim_level(RF_VANT_HIGH_POWER);
            rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, tx_moment);
            // pp_rf_set_tx_len(RF_PKT_ACK_LEN);
        }
        else
        {
            //DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
            // app_halt_system(0x1f03, "stimer tx_moment passed %x %x", tx_moment, stimer_get_tick());
        }
// rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, ll_ctx.last_stick + APP_SLOT3_1T_START_TIME_US * RF_SYSTEM_TIMER_TICK_1US);
// pp_rf_set_tx_len(RF_PKT_ACK_LEN);
#if AFH_ENABLE
        fill_chn_mask();
        // rf_tx_packet.mask = type >> 4;
        // rf_tx_packet.cnt = (type >> 2) & 0x3;
#endif

        #if TPSLL_SPP_NOT_SWITCH_EN
        #if !TPSLL_FHSS_EN
        uint8_t len = p24g_fill_tx_packet();
        if (len)
            pp_rf_set_tx_len(len);

        #else
        if (g_hop_map_cur != g_hop_map_next) {
            tpsll_fill_tx_pkt_chn_table();
            if (!pp_fifo_is_empty(&ll_fifo) && !ll_ctx.spp_flag) {
                unsigned char *p = pp_fifo_get_ptr(&ll_fifo);

                pp_rf_set_tx_len(3 + p[0]);
                tmemcpy((void *)&rf_tx_packet.buf[3], (void *)&p[2], p[0]); //fill spp data
                rf_tx_packet.pkt_type = P24G_PKT_TYPE_LL;
                ll_ctx.ll_flag = true;
            } else if (!pp_fifo_is_empty(&spp_fifo) && !ll_ctx.is_rr_changing) {
                unsigned char *p = pp_fifo_get_ptr(&spp_fifo);

                pp_rf_set_tx_len(3 + p[0]);
                tmemcpy((void *)&rf_tx_packet.buf[3], (void *)&p[2], (p[0] > DONGLE_SPP_MAX_LEN) ? DONGLE_SPP_MAX_LEN : p[0]); //fill spp data
                rf_tx_packet.pkt_type = P24G_PKT_TYPE_SPP;
                ll_ctx.spp_flag = true;
            } else{
                pp_rf_set_tx_len(3);
            }
        } else {
            uint8_t len = p24g_fill_tx_packet();
            if (len)
                pp_rf_set_tx_len(len);
            rf_tx_packet.n_tab = 0;
        }
        #endif /* end of TPSLL_FHSS_EN */

        #else
        #if !TPSLL_FHSS_EN
        uint8_t len = p24g_fill_tx_packet();
        if (len)
            pp_rf_set_tx_len(len);

        #else
        if (g_hop_map_cur != g_hop_map_next) {
            pp_rf_set_tx_len(3); //only in 8K !!!!!!!!
            tpsll_fill_tx_pkt_chn_table();
        } else {
            uint8_t len = p24g_fill_tx_packet();
            if (len)
                pp_rf_set_tx_len(len);
            rf_tx_packet.n_tab = 0;
        }
        #endif /* end of TPSLL_FHSS_EN */
        #endif /* end of !TPSLL_SPP_NOT_SWITCH_EN */
        // rf_tx_packet.buf[1] = ll_ctx.ack_report;

        // if (rf_tx_packet.pkt_type != P24G_PKT_TYPE_SPP)
        if ((rf_tx_packet.pkt_type != P24G_PKT_TYPE_SPP) && (rf_tx_packet.pkt_type != P24G_PKT_TYPE_LL))
        {
            rf_tx_packet.pkt_type = ll_ctx.report_mode;
        }

        // tpsll_fill_spp_flag();
        // rf_tx_packet.buf[0] = ll_ctx.ack_mask & 0x7;
        rf_tx_packet.buf[0] = (rf_tx_packet.buf[0] & 0xF0) | (ll_ctx.ack_mask & 0x7);
        rf_tx_packet.buf[0] = (rf_tx_packet.buf[0] & 0x07) | ((ll_ctx.fno & 0x1f) << 3); // put fno low 5 bits
        ll_ctx.task_slot = TASK_SLOT3;
        if (ll_ctx.is_rr_changing)
        {
            
            // tlkapi_send_string_data(APP_LOG_EN, "---send_ack", &rf_tx_packet.buf[0] , 1);
        }

        #if TPSLL_KB_ACTIVE_PM_OPTM_EN
        if (ll_inf.paired_dev_type & DEVICE_TYPE_KB) {
            if (!tpsll_all_fifo_is_empty()) {
                ll_ctx.kb_alive_intervals_cnt = 0;
                ll_ctx.kb_alive_fno_msk = TPSLL_KB_ALIVE_STAGE0_FNO_MASK;
            }

            uint32_t kb_alive_time_steps = ll_ctx.kb_alive_intervals_cnt >> ll_ctx.kb_alive_16ms_intervals_shift;

            if (kb_alive_time_steps < (TPSLL_KB_ALIVE_MAX_STAGED_TIME_MS / TPSLL_KB_ALIVE_TIME_STEP_MS)) {
                if (ll_ctx.kb_alive_intervals_cnt && ((ll_ctx.kb_alive_intervals_cnt & ll_ctx.kb_alive_16ms_intervals_msk) == 0)) {
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

_attribute_ram_code_sec_ void timer1_irq_handler_non_8k(void)
{
    bool rf_valid_flag = true;

    #if TPSLL_NEW_SYNC_EN
    if ((ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500))) {
        rf_valid_flag = !tpsll_is_rf_busy();
    }
    #endif

    if (ll_ctx.task_slot == TASK_SLOT0)
    {
        #if TPSLL_NON_8K_SPP_NOT_SW_EN
        rf_packet_t  *p=(rf_packet_t *)rx_packet;
        if((p->pkt_type != P24G_PKT_TYPE_SPP) && rf_valid_flag)
        {
        #endif


        #if TPSLL_NEW_SYNC_EN
        if ((ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500))) {
            if(ll_ctx.rx_over && ll_ctx.tx_over) {
                #if 0
                uint32_t tx_tick = stimer_get_tick() + 5 * RF_SYSTEM_TIMER_TICK_1US;
                rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, tx_tick);
                ll_ctx.tx_over = false;
                tpsll_calculate_stick_diff(tx_tick);

                uint8_t len = p24g_fill_tx_packet();
                if(len)
                    pp_rf_set_tx_len(len);
                #endif
            }
        } else
        #endif
        {
            // rf_set_ble_chn(chn_list[STEP_3T1R_2T_D_2R]);
            // while(0 != (read_reg8(0x170224) & 0x7));
            if (ll_ctx.tx_over && wait_rf_fsm_idle_timeout(1)) // 8k: 2.25us 4k:300ns
            {
                rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
                ll_ctx.rx_over = true;
            }

            if(ll_ctx.rx_over && ll_ctx.tx_over)
            {
                unsigned int tx_moment;
                if(ll_ctx.report_rate != REPORT_RATE_4K)
                {
                    tx_moment = ll_ctx.last_stick + (79 + 21 + 6 + FOR_KB_ALL_KEY_SYNC_MARGIN_TIME) * RF_SYSTEM_TIMER_TICK_1US;
                }
                else
                {
                    tx_moment = ll_ctx.last_stick + (76) * RF_SYSTEM_TIMER_TICK_1US;
                }
                if (tick1_exceed_tick2(tx_moment + RF_SYSTEM_TIMER_TICK_1US, stimer_get_tick()))
                {
                    rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, tx_moment);
                    ll_ctx.tx_over = false;
                    // pp_rf_set_tx_len(RF_PKT_ACK_LEN);
                    // rf_tx_packet.buf[2]++;
                }
                else
                {
                    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
                    // app_halt_system(0x1f03, "stimer tx_moment passed %x %x", tx_moment, stimer_get_tick());
                }

                uint8_t len = p24g_fill_tx_packet();
                if(len)
                {
                    pp_rf_set_tx_len(len);
                }

            #if TPSLL_N8K_FHSS_EN
                if (g_hop_map_cur != g_hop_map_next)
                {
                    tpsll_fill_tx_pkt_chn_table();
                }
                else
                {
                    rf_tx_packet.n_tab = 0;
                }
            #endif
            }

    }
        // p24g_set_snnesn();//First trigger the TX, then update SN and NESN in the Normal RX interrupt(for 6B payload).

            // uint8_t len = p24g_fill_tx_packet();
            // if(len)
            //     pp_rf_set_tx_len(len);

        #if TPSLL_NON_8K_SPP_NOT_SW_EN
        }
        #endif
        if (ll_ctx.spp_flag )
        {
            ll_ctx.spp_cnt++;
        }

       // pp_rf_set_tx_len(3);

        ll_ctx.task_slot++;

        // unsigned int r = core_interrupt_disable();
        // unsigned int t0 = APP_SLOT2_START_TIME_US * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); //change time for SLOT2
        unsigned int t0;
        if(ll_ctx.report_rate != REPORT_RATE_4K)
        {
            t0 = (KM_SLOT0_MARGIN_US_500_1K_2K + KM_SLOT1_TIME_US_500_1K_2K) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); // change time for SLOT2
        }else{
            t0 = (127) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); // change time for SLOT2
        }
        pp_timer1_start(t0);
       
        // core_restore_interrupt(r);
    }
    // else if ((ll_ctx.task_slot == TASK_SLOT1))
    else if (ll_ctx.task_slot <= ll_ctx.max_slot)
    {
        #if TPSLL_NON_8K_SPP_NOT_SW_EN
        // rf_packet_t  *p=(rf_packet_t *)rx_packet;
        // if(rf_valid_flag && !((ll_ctx.task_slot == TASK_SLOT1  || ll_ctx.task_slot == TASK_SLOT3) &&  ((p->pkt_type == P24G_PKT_TYPE_SPP) || (rf_tx_packet.pkt_type == P24G_PKT_TYPE_SPP)) ))
        // if(rf_valid_flag && ll_ctx.rx_over && ll_ctx.tx_over && !((ll_ctx.task_slot == TASK_SLOT1  || ll_ctx.task_slot == TASK_SLOT3) &&
        //     ((p->pkt_type == P24G_PKT_TYPE_SPP) || (rf_tx_packet.pkt_type == P24G_PKT_TYPE_SPP))))
        // if(rf_valid_flag && ll_ctx.rx_over && ll_ctx.tx_over && !((ll_ctx.task_slot == TASK_SLOT1  || ll_ctx.task_slot == TASK_SLOT3) &&
        //     ((p->pkt_type == P24G_PKT_TYPE_SPP) || (rf_tx_packet.pkt_type == P24G_PKT_TYPE_SPP))))
        if(rf_valid_flag && ll_ctx.rx_over && ll_ctx.tx_over)
        {

        // #if TPSLL_NON_8K_SPP_NOT_SW_EN
        // if(!((ll_ctx.task_slot == TASK_SLOT1 || ll_ctx.task_slot == TASK_SLOT3) && rf_tx_packet.pkt_type == P24G_PKT_TYPE_SPP))
        // {
        // #endif
        #endif
        // SLOT2
        // rf_set_ble_chn(chn_list[STEP_3T1R_2T_D_2R]);
        // rf_set_ble_chn(CONN_CHN_NO);
        // rf_set_ble_chn(chn_table[ll_ctx.task_slot & 3]);
        #if TPSLL_N8K_FHSS_EN
        #if TPSLL_N8K_CHN_EXPAND_EN
        rf_set_ble_chn(chn_table[(ll_ctx.task_slot + ll_ctx.fno_last) & 7]);
        #else
        rf_set_ble_chn(chn_table[(ll_ctx.task_slot + ll_ctx.fno_last) & 3]);
        #endif
        #else
        rf_set_ble_chn(g_n8k_chn_tab[(ll_ctx.task_slot + ll_ctx.fno_last) & (NON_8K_CHN_NUM - 1)]);
        #endif
        // while(0 != (read_reg8(0x170224) & 0x7));
        // if (wait_rf_fsm_idle_timeout(3)) // 8k: 2.25us
        // {
        //     rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
        // }

        // debug_gpio_with_cnt_ph0(ll_ctx.task_slot);

        if (!(read_reg8(0x170224) & 0x7) && ll_ctx.rx_over && ll_ctx.tx_over)
        {
            // if (ll_ctx.report_rate & REPORT_RATE_4K)
            // {
            //     pp_rf_start_srx2tx((unsigned char *)&rf_tx_packet.dma_len, stimer_get_tick(), DONGLE_CONNECTED_L_SRX_TIMEOUT_US);
            //     uint8_t len = p24g_fill_tx_packet();
            //     if (len)
            //         pp_rf_set_tx_len(len);
            // }
            // else
            {
                pp_rf_start_srx(stimer_get_tick(), DONGLE_CONNECTED_L_SRX_TIMEOUT_US);
                ll_ctx.rx_over = false;
            }
        }

        // pp_rf_start_srx2tx((unsigned char *)&rf_tx_packet.dma_len, stimer_get_tick(), DONGLE_CONNECTED_L_SRX_TIMEOUT_US + 6);
        // p24g_set_snnesn();
        // p24g_fill_tx_packet();
        // rf_set_rx_maxlen(RF_MAX_LEN);

        #if TPSLL_NON_8K_SPP_NOT_SW_EN
        }
        #endif
        
        // rf_set_ble_chn(15);
        // D_3R
        ll_ctx.task_slot++;

        if (ll_ctx.task_slot < ll_ctx.max_slot)
        {
            unsigned int r = core_interrupt_disable();
            // unsigned int t0 = 197 * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); // change time for SLOT3
            unsigned int t0 = (KM_SLOT0_MARGIN_US_500_1K_2K + ll_ctx.task_slot * KM_SLOT_TIME_US_500_1K_2K) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); // change time for SLOT2

            pp_timer1_start(t0);
            core_restore_interrupt(r);
        }
        #if TPSLL_FAST_SETTLE_CAL_FUN_EN
        else if ((ll_ctx.task_slot == ll_ctx.max_slot) && (ll_ctx.idle_flag))
        {
            if (ll_ctx.fast_settle_cal_en && (ll_ctx.report_rate & (REPORT_RATE_1K | REPORT_RATE_500)))
            {
            // gpio_toggle(GPIO_PG7);
                unsigned int r = core_interrupt_disable();
                unsigned int t0 = (KM_SLOT0_MARGIN_US_500_1K_2K + ll_ctx.task_slot * KM_SLOT_TIME_US_500_1K_2K - 50) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); // change time for SLOT2

                ll_ctx.task_slot++;
                pp_timer1_start(t0);
                core_restore_interrupt(r);
            }
        }
        #endif
#if AFH_ENABLE
        if ((ll_ctx.fno_last & (AFH_INTERVAL - 1)) == 0)
        {
            // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
            // check_and_update_mask();
            // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX5]);
        }
#endif
        // calc_hop_channels(ll_ctx.fno_last + 1, chn_mask);
    }
    #if TPSLL_FAST_SETTLE_CAL_FUN_EN
    else
    {
        if (ll_ctx.idle_flag && ll_ctx.fast_settle_calib_timeout)
        {
        // gpio_toggle(GPIO_PG7);
            if (ll_ctx.report_rate & REPORT_RATE_1K)
            {
                tpsll_fast_settle_dynamic_calib(1);
            }
            else if (ll_ctx.report_rate & REPORT_RATE_500)
            {
                tpsll_fast_settle_dynamic_calib(5);
            }
        }
    }
    #endif
    #if 0
    else if ((ll_ctx.task_slot == TASK_SLOT2))
    {
        if (wait_rf_fsm_idle_timeout(2)) // 4k: 1us
        {
            rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
        }

        unsigned int tx_moment = ll_ctx.last_stick + 200 * RF_SYSTEM_TIMER_TICK_1US;
        if (tick1_exceed_tick2(tx_moment + RF_SYSTEM_TIMER_TICK_1US, stimer_get_tick()))
        {
            rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, tx_moment);
            pp_rf_set_tx_len(RF_PKT_ACK_LEN);
            rf_tx_packet.buf[2]++;
        }
        else
        {
            // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
            // app_halt_system(0x1f03, "stimer tx_moment passed %x %x", tx_moment, stimer_get_tick());
        }
#if AFH_ENABLE
        fill_chn_mask();
        // rf_tx_packet.mask = type >> 4;
        // rf_tx_packet.cnt = (type >> 2) & 0x3;
#endif
        rf_tx_packet.buf[0] = ll_ctx.ack_mask & 0x7;
        rf_tx_packet.buf[0] = (rf_tx_packet.buf[0] & 0xF0) | (ll_ctx.ack_mask & 0x7);
        rf_tx_packet.buf[2]++;
        ll_ctx.task_slot = TASK_SLOT3;
    }
    #endif
}

_attribute_ram_code_sec_ void timer1_irq_handler_125(void)
{
    #if 0
    // if (ll_ctx.task_slot == TASK_SLOT0)
    {

        // rf_set_ble_chn(chn_list[STEP_3T1R_2T_D_2R]);
        // while(0 != (read_reg8(0x170224) & 0x7));
        if (wait_rf_fsm_idle_timeout(1)) // 8k: 2.25us 4k:300ns
        {

            rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
        }

        #if 0
        unsigned int tx_moment = ll_ctx.last_stick + 78 * RF_SYSTEM_TIMER_TICK_1US;
        if (tick1_exceed_tick2(tx_moment + RF_SYSTEM_TIMER_TICK_1US, stimer_get_tick()))
        {
            rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, tx_moment);
            pp_rf_set_tx_len(RF_PKT_ACK_LEN);
            rf_tx_packet.buf[2]++;
        }
        else
        {
            // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
            // app_halt_system(0x1f03, "stimer tx_moment passed %x %x", tx_moment, stimer_get_tick());
        }
            #endif

          pp_rf_start_srx2tx((unsigned char *)&rf_tx_packet.dma_len, stimer_get_tick(), DONGLE_CONNECTED_SUSPEND_SRX_TIMEOUT_US);
        // p24g_set_snnesn();
        rf_set_rx_maxlen(RF_PKT_DATA_LEN * 2);
        
        p24g_set_snnesn();
        if(!pp_fifo_is_empty(&ll_fifo))
        {
            p24g_fill_ll_pkt();
        }else{
            p24g_fill_null_pkt();
        }

        ll_ctx.task_slot++;

        // unsigned int r = core_interrupt_disable();
        // unsigned int t0 = APP_SLOT2_START_TIME_US * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); //change time for SLOT2
       
        unsigned int t0 = (ll_ctx.task_slot * SLOT_TIME_US) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); // change time for SLOT2
        pp_timer1_start(t0);
       
        // core_restore_interrupt(r);
    }
        #endif

    #if 1
    // else if ((ll_ctx.task_slot == TASK_SLOT1))
    // else
    if (ll_ctx.task_slot <= (ll_ctx.max_slot - 1))
    {
        // SLOT2
        // rf_set_ble_chn(chn_list[STEP_3T1R_2T_D_2R]);
        // rf_set_ble_chn(CONN_CHN_NO);
        // rf_set_ble_chn(chn_table[ll_ctx.task_slot & 3]);
        // rf_set_ble_chn(chn_table[(ll_ctx.task_slot + ll_ctx.fno_last) & 3]);
        ll_ctx.task_slot++;

        if(!(ll_ctx.task_slot & 1))
        {
            #if TPSLL_N8K_FHSS_EN
            #if TPSLL_N8K_CHN_EXPAND_EN
            uint8_t ch = chn_table[(ll_ctx.fno_last + (ll_ctx.task_slot >> 1)) & 7];
            #else
            uint8_t ch = chn_table[(ll_ctx.fno_last + (ll_ctx.task_slot >> 1)) & 3];
            #endif
            #else
            uint8_t ch = g_n8k_chn_tab[(ll_ctx.fno_last + (ll_ctx.task_slot >> 1)) & (NON_8K_CHN_NUM - 1)];
            #endif
            // uint8_t ch = chn_table[(ll_ctx.fno_last + ll_ctx.task_slot) & 3];
            rf_set_ble_chn(ch);

            // rf_set_ble_chn(4);
            // while(0 != (read_reg8(0x170224) & 0x7));
            if (wait_rf_fsm_idle_timeout(3)) // 8k: 2.25us
            {
                rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
            }
            // pp_rf_start_srx(stimer_get_tick(), DONGLE_CONNECTED_SRX_TIMEOUT_US);
            // pp_rf_start_srx2tx((unsigned char *)&rf_tx_packet.dma_len, stimer_get_tick(), DONGLE_CONNECTED_SUSPEND_SRX_TIMEOUT_US - 15);
            pp_rf_start_srx(stimer_get_tick(), DONGLE_CONNECTED_SUSPEND_SRX_TIMEOUT_US - 50);
            ll_ctx.rx_over = false;
            // p24g_set_snnesn();
            // rf_set_rx_maxlen(RF_PKT_DATA_LEN * 2);
            // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX4]);
        } else if (0 == (read_reg8(0x170224) & 0x7)) {
            pp_rf_start_srx(stimer_get_tick(), (DONGLE_CONNECTED_SUSPEND_SRX_TIMEOUT_US / 2 - 50));
            ll_ctx.rx_over = false;
            // p24g_set_snnesn();
            // rf_set_rx_maxlen(RF_PKT_DATA_LEN * 2);
        }

        // rf_set_ble_chn(15);
        // D_3R

        // if(ll_ctx.task_slot < 2)
        // {
        //     // tlkapi_printf(APP_LOG_EN, "fno:%x slot:%x ch:%x\n", ll_ctx.fno_last, ll_ctx.task_slot, ch);
        // }

        

        // if (ll_ctx.task_slot < ((ll_ctx.max_slot / 2) - 1))
        if (ll_ctx.task_slot < (ll_ctx.max_slot - 1))
        {
            unsigned int r = core_interrupt_disable();
            // unsigned int t0 = 197 * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); // change time for SLOT3
            // unsigned int t0 = ((ll_ctx.task_slot + 1) * 2 * SLOT_TIME_US + D_SUSPEND_EARLY_TIME_US) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); // change time for SLOT2
            // unsigned int t0 = (SLOT1_OFFSET_TIME_US_125 + ll_ctx.ir_btn_proc_us) * RF_SYSTEM_TIMER_TICK_1US + ll_ctx.task_slot * SLOT_TIME_US_125 * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); // change time for SLOT2
            // pp_timer1_start(t0);

            pp_timer1_start((SLOT1_OFFSET_TIME_US_125 + ((ll_inf.connecting_dev & DEVICE_TYPE_MS) ? ll_ctx.ir_btn_proc_us : 0)) * RF_SYSTEM_TIMER_TICK_1US +
                            ll_ctx.task_slot * SLOT_TIME_US_125 * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));

            core_restore_interrupt(r);
        }
#if TPSLL_FAST_SETTLE_CAL_FUN_EN
        else if ((ll_ctx.task_slot == (ll_ctx.max_slot - 1)) && (ll_ctx.idle_flag))
        {
        // gpio_toggle(GPIO_PG7);
            unsigned int r = core_interrupt_disable();
            unsigned int t0 = (SLOT1_OFFSET_TIME_US_125 + ll_ctx.ir_btn_proc_us) * RF_SYSTEM_TIMER_TICK_1US + ll_ctx.task_slot * SLOT_TIME_US_125 * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick); // change time for SLOT2

            ll_ctx.task_slot++;
            pp_timer1_start(t0);
            core_restore_interrupt(r);
        }
    }
    else
    {
        if (ll_ctx.idle_flag && ll_ctx.fast_settle_calib_timeout)
        {
        // gpio_toggle(GPIO_PG7);
            if (ll_ctx.report_rate & REPORT_RATE_250)
            {
                tpsll_fast_settle_dynamic_calib(16);
            }
            else if (ll_ctx.report_rate & REPORT_RATE_125)
            {
                tpsll_fast_settle_dynamic_calib(TPSLL_FHSS_CHN_MAX_NUM + 8);
            }
        }
#endif
    }
    #endif
}

#if !TPSLL_SPP_NOT_SWITCH_EN
_attribute_ram_code_sec_ void tpsll_timer1_irq_handler_spp_mode(void)
{
    ll_ctx.task_slot++;

    // ll_ctx.ch_idx = (ll_ctx.task_slot + ll_ctx.fno_last) & 3;
    uint8_t ch = chn_table[(ll_ctx.task_slot + ll_ctx.fno_last) & 3];
    rf_set_ble_chn(ch);

    unsigned int tx_moment = ll_ctx.last_stick + 272 * RF_SYSTEM_TIMER_TICK_1US;
    if (tick1_exceed_tick2(tx_moment, stimer_get_tick()))
    {
        //            rf_set_vant_power_trim_level(RF_VANT_HIGH_POWER);
        rf_start_stx((unsigned char *)&rf_tx_packet.dma_len, tx_moment);
        
        uint8_t len = tpsll_fill_spp_mode_tx_packet();
        pp_rf_set_tx_len(len);
        // p24g_set_snnesn();
    }
}
#endif

#if DONGLE_SUSPEND_SUPPORT && !TPSLL_NEW_RECON_EN
_attribute_ram_code_sec_ void tpsll_timer1_irq_handler_suspend(void)
{
    if (ll_ctx.task_slot <= (TPSLL_FHSS_CHN_MAX_NUM - 1))
    {
        if (wait_rf_fsm_idle_timeout(3))
        {
            rf_set_tx_rx_off_auto_mode(); // turn off tx,rx
        }

        static unsigned char reconn_chn_idx = 0;
        rf_set_ble_chn(g_fhss_valid_chn_tab[reconn_chn_idx & (TPSLL_FHSS_CHN_MAX_NUM - 1)]);
        reconn_chn_idx++;

        pp_rf_start_stx2rx((unsigned char *)&rf_tx_packet.dma_len, stimer_get_tick() + PARING_STIMER_TO_TX_US * RF_SYSTEM_TIMER_TICK_1US, DONGLE_PAIRING_SRX_TIMEOUT_US + 5);
                reconn_req_packet_t *pt = (reconn_req_packet_t *)&rf_tx_packet;

        pp_rf_set_tx_len(9);
        pt->cmd = CMD_RECONN_REQ;
        // pt->tag = ll_ctx.ktag; //use ktag as reconnect tag
        // tmemcpy(pt->mac, ll_inf.mac, MAC_ADDR_LEN);
        // app_halt_system(0x9988, "pt mac:%x %x %x  %x %x %x\n", pt->mac[0], pt->mac[1],pt->mac[2],pt->mac[3],pt->mac[4],pt->mac[5]);
#if RSSI_SCAN_EN
        tmemcpy(pt->chn_table, chn_table, CHN_TABL_LEN);
        // tmemcpy(pt->chn_table, hop_ctrl.chn_table, CHN_TABL_LEN);
#endif


        pt->fno_last = ll_ctx.fno_last;

        if (ll_inf.paired_dev_type & DEVICE_TYPE_KB)
        {
            pt->tag = ll_ctx.ktag;
        }
        else if (ll_inf.paired_dev_type & DEVICE_TYPE_MS)
        {
            pt->tag = ll_ctx.mtag;
        }
        // rf_tx_packet.buf[3] = ll_inf.side_id & 0xff;
        rf_set_rx_maxlen(10);
        ll_ctx.task_slot++;

        unsigned int t0 = (DONGLE_SUSPEND_TIMER1_US * (ll_ctx.task_slot + 1)) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick);
        pp_timer1_start(t0);
    }
    else if(ll_ctx.task_slot > (TPSLL_FHSS_CHN_MAX_NUM - 1))
    {
        if (s_app_cb && s_app_cb->app_get_usb_status)
        {
            if(s_app_cb->app_get_usb_status() == USB_ENTER_SUSPEND && ll_ctx.suspend_allow)
            {
                ll_ctx.suspend_enable = 1;
            }
        }

        ll_ctx.task_slot++;
    }
}
#endif

_attribute_ram_code_sec_ uint8_t tpsll_timer1_irq_handler(void)
{
    // DBG_STACK_GPIO_TOGGLE(STACK_IO_EN, dbg_io[DBG_IO_IDX3]);
    // ZW_TG(GPIO_PG7);
    if (timer_get_irq_status(FLD_TMR1_MODE_IRQ))
    {
        pp_timer1_stop();
        if (ll_ctx.stk_state == STATE_CONNECTED)
        {
            #if !TPSLL_SPP_NOT_SWITCH_EN
            if (!ll_ctx.spp_mode)
            #endif
            {
                switch (ll_ctx.report_rate)
                {
                case REPORT_RATE_8K:
                    timer1_irq_handler_8k();

                    break;
                case REPORT_RATE_250:
                case REPORT_RATE_125:
                    timer1_irq_handler_125();

                    break;
                default:
                    timer1_irq_handler_non_8k();
                    break;
                }
            }
            #if !TPSLL_SPP_NOT_SWITCH_EN
            else
            {
                tpsll_timer1_irq_handler_spp_mode();
            }
            #endif
        }
        #if !TPSLL_NEW_RECON_EN
        else if(ll_ctx.stk_state == STATE_RECONNECT)
        {
            if (ll_ctx.recon_step == RECON_STEP1) {
                if (!wait_rf_fsm_idle_timeout(1))
                {
                    #if !TPSLL_RECON_RSSI_SCAN_EN
                    rf_set_ble_chn((ll_ctx.reconn_chn + 13) & 0x3F);
                    #endif
                    pp_rf_start_srx(stimer_get_tick(), DONGLE_PAIRING_SRX_TIMEOUT_US + 5);
                }
                pp_timer1_start((155) * RF_SYSTEM_TIMER_TICK_1US - (stimer_get_tick() - ll_ctx.last_stick));

                ll_ctx.recon_step = RECON_STEP2;
            } else if (ll_ctx.recon_step == RECON_STEP2) {
                if (!wait_rf_fsm_idle_timeout(1)) {
                    #if !TPSLL_RECON_RSSI_SCAN_EN
                    rf_set_ble_chn((ll_ctx.reconn_chn + 2 * 13) & 0x3F);
                    #endif
                    pp_rf_start_srx(stimer_get_tick(), DONGLE_PAIRING_SRX_TIMEOUT_US + 5);
                }
                pp_timer1_stop();

                ll_ctx.recon_step = RECON_STEP_NONE;
            } else
            #if DONGLE_SUSPEND_SUPPORT && !TPSLL_NEW_RECON_EN
            if (s_app_cb && s_app_cb->app_get_usb_status)
            {
                if(s_app_cb->app_get_usb_status() == USB_ENTER_SUSPEND && ll_ctx.suspend_allow)
                {
                    tpsll_timer1_irq_handler_suspend();
                }
            }
            #endif
            ;
        }
        #endif /* endif of !TPSLL_NEW_RECON_EN */
        else if (ll_ctx.stk_state == STATE_PAIRING)
        {
            if (ll_ctx.pair_step == PAIR_STEP2) {
                if (!wait_rf_fsm_idle_timeout(1)) {
                    rf_set_ble_chn(PAIR_CHN_NO + 13);
                    pp_rf_start_srx(stimer_get_tick(), DONGLE_PAIRING_SRX_TIMEOUT_US);
                }
                pp_timer1_stop();
                ll_ctx.pair_step = PAIR_STEP_NONE;
            }
        }
        ll_ctx.wfi_flag = true;
        timer_clr_irq_status(FLD_TMR1_MODE_IRQ); // clear irq status
    }
    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ report_rate_t tpsll_get_report_rate(void)
{
    return ll_ctx.report_rate;
}

static void spp_send_reset(void)
{
    spp_frag_total = 0;
    spp_frag_acked = 0;
    spp_original_data = NULL;
    spp_original_len = 0;
    spp_sending_busy = false;
}

uint8_t tpsll_send_spp_data(uint8_t *data, uint8_t len, uint8_t retry_num, spp_cb_t cb, void *user_arg)
{
    uint8_t ret = 0;

    if(ll_ctx.is_rr_changing || (g_state != STATE_CONNECTED) || (ll_ctx.stk_state != STATE_CONNECTED))
    {
        spp_send_reset();
        return TLK_ERR_INVALID_STATE;
    }

    if(!data)
    {
        spp_send_reset();
        return TLK_ERR_NULL;
    }

    if (spp_sending_busy) {
        return TLK_ERR_BUSY;
    }

    if((len > (KM_SPP_MAX_LEN - 1)) || (len == 0))
    {
        spp_send_reset();
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

#if DONGLE_SPP_FRAG_EN
    // Case 1: Small packet, no fragmentation needed
    if (len <= SPP_MAX_CHUNK_LEN)
    {
        // header: opcode(6bit) + fragment type(2bit)
        uint8_t header = (SPP_OPCODE_DEFAULT << SPP_OPCODE_SHIFT) | SPP_FRAG_TYPE_NO_FRAG;
        spp_send_buf[0] = header;
        tmemcpy(&spp_send_buf[1], data, len);

        unsigned int r = core_interrupt_disable();
        ret = pp_fifo_push_extra(&spp_fifo, TPSLL_SPP_DATA, spp_send_buf, 1 + len, retry_num, cb, user_arg);
        core_restore_interrupt(r);
    }
    else // Case 2: Large packet, auto split into multiple fragments
    {
        uint16_t offset = 0;
        uint8_t total_frags = ((uint16_t)len + SPP_MAX_CHUNK_LEN - 1) / SPP_MAX_CHUNK_LEN;
        // uint8_t frag_idx = 0;

        if (pp_fifo_get_remaining_slots(&spp_fifo) < 2) {
            return TLK_ERR_BUFFER_FULL;
        }

        spp_sending_busy = true;

        spp_frag_total = total_frags;
        spp_frag_acked = 0;
        spp_original_data = data;
        spp_original_len  = len;

        for (uint8_t idx = 0; idx < total_frags; idx++)
        {
            uint8_t chunk_payload_len = len - offset;
            if (chunk_payload_len > SPP_MAX_CHUNK_LEN)
            {
                chunk_payload_len = SPP_MAX_CHUNK_LEN;
            }

            uint8_t frag_type;
            if (idx == total_frags - 1)
            {
                frag_type = SPP_FRAG_TYPE_END;
            }
            else
            {
                frag_type = SPP_FRAG_TYPE_MID;
            }

            // header: opcode(6bit) + fragment type(2bit)
            uint8_t header = (SPP_OPCODE_DEFAULT << SPP_OPCODE_SHIFT) | frag_type;
            spp_send_buf[0] = header;
            tmemcpy(&spp_send_buf[1], &data[offset], chunk_payload_len);

            unsigned int r = core_interrupt_disable();
            ret = pp_fifo_push_extra(&spp_fifo, TPSLL_SPP_DATA, spp_send_buf, 1 + chunk_payload_len, retry_num, cb, user_arg);
            core_restore_interrupt(r);

            if (ret != 0)
            {
                spp_send_reset();
                break;
            }

            offset += chunk_payload_len;
        }
    }

#else

    unsigned int r = core_interrupt_disable();
    ret = pp_fifo_push_extra(&spp_fifo, TPSLL_SPP_DATA, data, len, retry_num, cb, user_arg);
    core_restore_interrupt(r);

#endif

    return ret;
}

uint8_t tpsll_set_proc_ir_btn_us(uint8_t us)
{
    if(us < 20 || us > 60)
    {
        return TLK_ERR_INVALID_PARAM;
    }
    ll_ctx.ir_btn_proc_us = us;
    return TLK_SUCCESS;
}

uint8_t tpsll_request_suspend(void)
{
    if(s_app_cb && s_app_cb->app_get_usb_status && (s_app_cb->app_get_usb_status() == USB_ENTER_SUSPEND))
    {
        // tlkapi_printf(APP_LOG_EN,"susp allow\n");
        #if TPSLL_NEW_RECON_EN
        ll_ctx.suspend_allow = 1;
        tpsll_connect_lost();
        pp_timer1_stop();
        timer_clr_irq_status(FLD_TMR1_MODE_IRQ);
        #else
        app_d24g_set_report_rate(REPORT_RATE_8K);
        ll_ctx.suspend_allow = 1;
        tpsll_set_state(STATE_RECONNECT);
        ll_ctx.interval_us = DONGLE_SUSPEND_INTERVAL_US;
        pp_timer1_stop();
        #endif
        return TLK_SUCCESS;
    }
    else
    {
        return TLK_ERR_INVALID_STATE;
    }
}

uint8_t tpsll_set_suspend_interval(uint16_t suspend_interval_ms)
{
    if(suspend_interval_ms < 10 || suspend_interval_ms > 500)
    {
        return TLK_ERR_INVALID_PARAM;
    }
    ll_ctx.suspend_interval_ms = suspend_interval_ms;
    return TLK_SUCCESS;
}

/**
*  @brief      set usb sof start flag
*  @param[in]  flag  indicate whether usb sof has started
*  @return  none
*/
uint8_t tpsll_set_usb_start_flag(uint8_t flag)
{
    g_usb_sof_start = flag;
    return TLK_SUCCESS;
}

/**
*  @brief      get usb sof start flag
*  @param[in]  none
*  @return  flag of usb sof start
*/
uint8_t tpsll_get_usb_start_flag(void)
{
    return g_usb_sof_start;
}

/**
*  @brief    2.4g stack main loop 
*  @param[in]  none   

*  @return  none
*/
_attribute_ram_code_sec_ uint8_t tpsll_stack_main_loop(void)
{
#if DONGLE_SUSPEND_SUPPORT
    if(s_app_cb && s_app_cb->app_get_usb_status && s_app_cb->app_set_usb_status)
    {
        if(s_app_cb->app_get_usb_status() == USB_ENTER_SUSPEND && ll_ctx.suspend_allow)
        {
            if (ll_ctx.suspend_enable && tick1_exceed_tick2(ll_ctx.stick, stimer_get_tick() + 8000 * SYSTEM_TIMER_TICK_1US))
            {
                ll_ctx.suspend_enable = 0;
                pm_set_suspend_power_cfg(FLD_PD_ZB_EN, 1);
                // gpio_set_high_level(dbg_io[DBG_IO_IDX2]);
                pm_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_CORE|PM_WAKEUP_TIMER, PM_TICK_STIMER, ll_ctx.stick-500*SYSTEM_TIMER_TICK_1US);
                // pm_set_dig_module_power_switch(FLD_PD_ZB_EN, PM_POWER_UP);
                // rf_n22_dig_init();
                // app_set_usb_wake_status(USB_ACTIVE_STATE);
                #if !TPSLL_NEW_RECON_EN
                tpsll_enable_reconn(true);
                #endif
                // gpio_set_low_level(dbg_io[DBG_IO_IDX2]);
                // pp_rf_init();
            }
        }
        if(s_app_cb->app_get_usb_status() == USB_EXIT_SUSPEND)
        {
            //gpio_toggle(dbg_io[DBG_IO_IDX3]);
            s_app_cb->app_set_usb_status(USB_ACTIVE_STATE);
            ll_ctx.suspend_allow = 0;
            #if !TPSLL_NEW_RECON_EN
            ll_ctx.interval_us = 250;
            tpsll_enable_reconn(true);
            #endif
        }
    }
#endif
    if (ll_ctx.wfi_en && ll_ctx.wfi_flag && !ll_ctx.suspend_enable)
    {
        ll_ctx.wfi_flag = false;
        core_entry_wfi_mode();
    }

    #if TPSLL_RECON_RSSI_SCAN_EN && TPSLL_NEW_RECON_EN
    if (ll_ctx.is_stack_running && (ll_ctx.recon_rssi_scan_step >= RECON_RSSI_SCAN_STEP_START)) {
        #if TPSLL_RF_SCAN_EN
        if (rf_scan_ctx.state & RF_SCAN_STATE_IDLE) {
            rf_scan_ctx.state = RF_SCAN_STATE_RUNNING;
            tpsll_rf_scan_start();
            ll_ctx.wfi_flag = false;
        } else if (rf_scan_ctx.state & RF_SCAN_STATE_DONE) {
            tpsll_rssi_scan_check();
            ll_ctx.recon_rssi_scan_step = RECON_RSSI_SCAN_STEP_NONE;
            ll_ctx.wfi_flag = true;
        }
        #else
        rf_rssi_scan();
        ll_ctx.recon_rssi_scan_step = RECON_RSSI_SCAN_STEP_NONE;
        #endif
    }
    #if TPSLL_RF_SCAN_EN
    else if (ll_ctx.is_keep_alive) {
        tpsll_rssi_scan_check();
    }
    #endif
    #endif /* end of TPSLL_RECON_RSSI_SCAN_EN && TPSLL_NEW_RECON_EN */

    tpsll_idle_and_fast_settle_cal_check();
    /* process up to N events per loop to avoid starving other tasks (0 = process all) */
    stack_poll_events(0);

    // if (print_chan_table) {
    //     print_chan_table = false;
    //     tlkapi_printf(APP_LOG_EN, "(%d)(%d)(%d)(%d)(%d)(%d)(%d)(%d)\n", chn_table[0],
    //             chn_table[1], chn_table[2], chn_table[3], chn_table[4], chn_table[5], chn_table[6], chn_table[7]);
    // }
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

_attribute_ram_code_sec_ uint8_t tpsll_reconn_pm_optm_interval_idle_time(uint16_t time_ms)
{
    if (!time_ms || (time_ms > TPSLL_RECON_PM_OPTM_MAX_INTERVAL_TIME_MS)) {
        return TLK_ERR_INVALID_PARAM;
    }

    ll_ctx.recon_pm_optm_interval_ms = time_ms;

    return TLK_SUCCESS;
}

_attribute_ram_code_sec_ uint8_t tpsll_reconn_enter_pm_optm_start_time(uint16_t optm_start_time_s)
{
    if (!optm_start_time_s || (optm_start_time_s > TPSLL_RECON_ENTER_PM_OPTM_MAX_START_TIME_S)) {
        return TLK_ERR_INVALID_PARAM;
    }

    ll_ctx.recon_enter_pm_optm_start_time_ms = optm_start_time_s * 1000;

    return TLK_SUCCESS;
}

uint8_t tpsll_stack_init(tpsll_stack_config_t *p_config)
{
    if (!p_config || !p_config->app_var_cb)
    {
        tlkapi_printf(APP_LOG_EN, "[TPSLL] have null cb, init failed!\n");
        return TLK_ERR_NULL;
    }
    s_app_var_cb = p_config->app_var_cb;
    s_app_cb = &p_config->app_cb;

#if AFH_ENABLE
    init_reverse_table();
#endif

    rf_tx_packet.buf[0] = 0XA0;
    rf_tx_packet.buf[1] = 0xA1;
    rf_tx_packet.buf[2] = 0;

    tmemset(last_usb_report_buf, 0xff, sizeof(last_usb_report_buf));

    app_d24g_set_report_rate(APP_DEFAULT_REPORT_RATE);

// #if RSSI_SCAN_EN
//     rf_rssi_scan();
// #endif

    app_fill_paring_pkt();
    ll_ctx.lost_cnt = 0;
    ll_ctx.step_3t1r = STEP_3T1R_NONE;

    tpsll_alg_set_enable(TPSLL_ALG_ENABLE);

    #if TPSLL_MOUSE_BTN_FILTER_EN
    ll_ctx.ms_btn_rec.prs_tick = clock_time() | 1;
    ll_ctx.ms_btn_rec.rls_tick = clock_time() | 1;
    ll_ctx.ms_btn_rec.hold_tick = clock_time() | 1;
    #endif
    tpsll_inter_register_callback(TPSLL_INTER_EVT_SPP_SEND_COMP, tpsll_stack_event_handler);

    #if SQUARE_DEBUG_TEST
    SquareStat_Init();
    #endif

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

    return TLK_SUCCESS;
}

uint8_t tpsll_set_pairing_access_code(uint32_t ac)
{
    if (!is_valid_access_code(ac)) {
        return TLK_ERR_INVALID_PARAM;
    }
    ll_ctx.pair_acc = ac;
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

uint8_t tpsll_rf_init(void)
{
    pp_rf_init();
    return TLK_SUCCESS;
}

tpsll_dev_status_e tpsll_get_state(void)
{
    return g_state;
}

const uint8_t* tpsll_get_stack_version(void)
{
    return tpsll_stack_version;
}
