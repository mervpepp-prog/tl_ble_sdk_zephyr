/********************************************************************************************************
 * @file    app_common_config.h
 *
 * @brief   This is the header file for BLE SDK
 *
 * @author  BLE GROUP
 * @date    06,2022
 *
 * @par     Copyright (c) 2022, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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
#pragma once

#include "config.h"
#include "bit.h"
#include "types.h"


#define MAC_ADDR_LEN                6 //MAC address length

#define DBG_2P4G_IO_EN      0 //2.4G global gpio debug control

/**** !!!!!! Note: The code here must be removed when releasing the SDK ****/
#define ZW_GPIO_DEBUG    0
#if ZW_GPIO_DEBUG
    #define ZW_TG(pin)  gpio_toggle(pin)
    #define ZW_WR(pin, val)  gpio_set_level(pin, val)
#else
    #define ZW_TG(pin)  do{}while(0)
    #define ZW_WR(pin, val)  do{}while(0)
#endif

//Can only be enabled when drawing square in simulation; otherwise it will cause a crash.
#define RF_PKT_IDX_DEBUG_EN     0
/**** !!!!!! Note: The code here must be removed when releasing the SDK ****/

#define DBG_GPIO_NOOP() \
    do {                \
    } while (0)

#if DBG_2P4G_IO_EN
#define STACK_IO_EN         1 //stack
#define APP_IO_EN           1 //app

#define DBG_GPIO_SET_LEVEL(en, pin, level) \
    do {                                   \
        if (en) {                          \
            gpio_set_level(pin, level);    \
        }                                  \
    } while (0)


#define DBG_GPIO_TOGGLE(en, pin) \
    do {                         \
        if (en) {                \
            gpio_toggle(pin);    \
        }                        \
    } while (0)

#define DBG_STACK_GPIO_SET_LEVEL(en, pin, level) \
    do {                                         \
        if (en && (pin)) {                       \
            gpio_set_level(pin, level);          \
        }                                        \
    } while (0)

#define DBG_STACK_GPIO_TOGGLE(en, pin) \
    do {                               \
        if (en && (pin)) {             \
            gpio_toggle(pin);          \
        }                              \
    } while (0)
#else
#define STACK_IO_EN         0 //stack
#define APP_IO_EN           0 //app

#define DBG_GPIO_SET_LEVEL(en, pin, level)       DBG_GPIO_NOOP()
#define DBG_GPIO_TOGGLE(en, pin)                 DBG_GPIO_NOOP()
#define DBG_STACK_GPIO_SET_LEVEL(en, pin, level) DBG_GPIO_NOOP()
#define DBG_STACK_GPIO_TOGGLE(en, pin)           DBG_GPIO_NOOP()
#endif

typedef enum {
    KB_MODE_2P4G                        =   0,
    KB_MODE_BLE                         =   1,
    KB_MODE_USB                         =   2,
} kb_mode_t;

#define APP_PC_STATUS_TYPE                  0x00U
#define APP_LED_STATUS_TYPE                 0x01U

typedef enum
{
    CMD_PAIR_REQ                        = 0X10,
    CMD_PAIR_RSP                        = 0X11,
    CMD_CONN_REQ                        = 0X12,
    CMD_CONN_RSP                        = 0X13,
    CMD_CONN_DAT                        = 0X14,
    CMD_DETACH                          = 0X15,
    CMD_REJOIN                          = 0X16,
    CMD_POLL                            = 0X17,

    CMD_RECONN_REQ                      = 0X18,
    CMD_RECONN_RSP                      = 0X19,

    TPSLL_MS_DATA                       = 0x50,
    TPSLL_LL_CMD                        = 0x51,
    TPSLL_SPP_DATA                      = 0x52,
    TPSLL_LL_NULL                       = 0x53,

} tpsll_cmd_e;

typedef enum
{
    STATE_POWERON                       = 0,
    STATE_PAIRING,
    STATE_RECONNECT,
    STATE_NORMAL,
    STATE_CONNECTED,
    STATE_DISCONNECTED,
    STATE_IDLE,
    STATE_NONE,

} tpsll_dev_status_e;


typedef enum
{
    STATE_PAIR_NONE                     = 0,
    STATE_PAIR_INIT,
    STATE_PAIR_RSP,
    STATE_PAIR_CFM,
    STATE_REJOIN,
} tpsll_pair_status_e;

typedef enum {

    P24G_SM_CMD_PAIRING                 =   0x00,
    P24G_SM_CMD_LL_CONTROL,
    P24G_SM_CMD_MOUSE_DRAW,
    P24G_SM_CMD_SET_STATE,
    P24G_SM_CMD_SAVE_PAIR_INFO,
    P24G_SM_CMD_SET_KB_MODE,
    P24G_SM_CMD_DATA_TYPE_SPP,
    P24G_SM_CMD_SPP_SEND_COMP,
    P24G_SM_CMD_MISC,
    P24G_SM_CMD_REPORT_RATE_CHANGE,
    P24G_SM_CMD_MAX,
    P24G_SM_CMD_NONE                    =   0xFF,

} p24g_sm_cmd_e;
 
typedef enum {
    P24G_SM_OP_NONE                     =   0x00,
    P24G_SM_OP_TERMINATE_CONN           =   0x51,
    P24G_SM_OP_ENTER_RF_IDLE            =   0x52,
    P24G_SM_OP_MISC_TRANS_MAC           =   0x53,
    P24G_SM_OP_MISC_PEER_INFO           =   0x54,
    P24G_SM_OP_ENABLE_RECONN            =   0x55,
    P24G_SM_OP_MISC_STOP_STIMER         =   0x56,
    P24G_SM_OP_MISC_REPORT_RATE         =   0x57,
    P24G_SM_OP_MISC_SAVE_REPORT_RATE    =   0x58,
    P24G_SM_OP_MISC_RF_MODE             =   0x59,
    P24G_SM_OP_MISC_SUSP_RET            =   0x5A,
    P24G_SM_OP_MISC_LONG_SUSP_RET       =   0x5B,
} p24g_sm_op_e;

typedef enum
{
    TPSLL_SPP_NONE                       =   0x60,
    TPSLL_SPP_LED_STATUS                 =   0x61,
    TPSLL_SPP_REPORT_RATE                =   0x62,
    TPSLL_SPP_BATT_CAP                   =   0x63,
    TPSLL_SPP_CHG_STATUS                 =   0x64,
    TPSLL_SPP_TEST_DATA                  =   0x65,
    TPSLL_SPP_ALL_KEY_DATA               =   0x66,
    TPSLL_SPP_CONSUME_KEY_DATA           =   0x67,
    TPSLL_SPP_PC_SUSPEND                 =   0x68,
} tpsll_spp_cmd_e;

typedef enum
{
    TPSLL_LL_CMD_NONE                    =   0x70,
    TPSLL_LL_CMD_TERMINATE_CONN_REQ      =   0x71,
    TPSLL_LL_CMD_TERMINATE_CONN_RSP      =   0x72,
    TPSLL_LL_CMD_8K_RR_ADAPT_REQ         =   0x73,
    TPSLL_LL_CMD_8K_RR_ADAPT_RSP         =   0x74,
    TPSLL_LL_CMD_RR_CHG_REQ              =   0x75,
    TPSLL_LL_CMD_RR_CHG_RSP              =   0x76,
    TPSLL_LL_CMD_SLOT_CHG_REQ            =   0x77,
    TPSLL_LL_CMD_SLOT_CHG_RSP            =   0x78,
    TPSLL_LL_CMD_RR_CHG_RCNN_REQ         =   0x79,
    TPSLL_LL_CMD_RR_CHG_RCNN_RSP         =   0x7A,
    TPSLL_LL_CMD_UL_DATA_SEND_REQ        =   0x7B,
    TPSLL_LL_CMD_MAX,
} tpsll_ll_cmd_e;

typedef enum
{
    TPSLL_EVT_STACK_START = 0,
    TPSLL_EVT_DEV_CONNECTED,
    TPSLL_EVT_DEV_DISCONNECTED,
    TPSLL_EVT_PAIRING_ENTER,
    TPSLL_EVT_DEV_PAIRED,
    TPSLL_EVT_REPORT_RATE_CHANGED,
    TPSLL_EVT_ENTER_SUSPEND,
    TPSLL_EVT_EXIT_SUSPEND,
    TPSLL_EVT_SPP_DATA_RECV,
    TPSLL_EVT_PAIR_TIMEOUT,
    TPSLL_EVT_RECONNECT_TIMEOUT,
    TPSLL_EVT_PM_OPTM_ENTER_LEVEL1,
    TPSLL_EVT_PM_OPTM_ENTER_LEVEL2,
    TPSLL_EVT_PM_OPTM_ENTER_LEVEL3,
    TPSLL_EVT_PM_OPTM_ENTER_LEVEL4,
    TPSLL_EVT_PM_OPTM_EXIT,
    TPSLL_EVT_NO_ACTIVE_TIMEOUT,
    TPSLL_EVT_USR_SPEC_DATA_RECV,
    TPSLL_EVT_USR_SPEC_ACK_RECV,
    TPSLL_EVT_SLEEP_ENTER_REQ,
    TPSLL_EVT_SAVE_REPORT_RATE,
    TPSLL_EVT_MAX
} tpsll_evt_t;

typedef enum
{
    REPORT_RATE_8K                      = BIT(7),
    REPORT_RATE_4K                      = BIT(0),
    REPORT_RATE_2K                      = BIT(1),
    REPORT_RATE_1K                      = BIT(2),
    REPORT_RATE_500                     = BIT(3),
    REPORT_RATE_250                     = BIT(4),
    REPORT_RATE_125                     = BIT(5),
    REPORT_RATE_NONE                     = 0,

    RR_8K_HIGH                          = 0,
    RR_8K_LOW                           = 1,
    REPORT_RATE_8K_MODE_LOW             = REPORT_RATE_500,
    REPORT_RATE_8K_MODE_HIGH            = REPORT_RATE_8K,
} report_rate_t;

typedef enum
{
    DPI_12800 = 0xFF,
    DPI_7000 = 0x8B,
    DPI_3600 = 0x48,
    DPI_1600 = 0x20,
    DPI_400 = 0x08,
} dpi_t;

typedef enum
{
    P24G_TIMEOUT_TYPE_PAIR      = 0,
    P24G_TIMEOUT_TYPE_RECONN    = 1,
    P24G_TIMEOUT_TYPE_DISCONN   = 2,
    P24G_TIMEOUT_TYPE_NOACTIVE  = 3,

}tpsll_timeout_type_e;

typedef enum
{
    MOUSE_DATA=3,
    SPP_DATA = 4,
    SPP_DATA_ACK = 5,
    NORMAL_KB_DATA_CMD = 6,
    CONSUME_KB_DATA_CMD = 7,
    SYSTEM_KB_DATA_CMD = 8,
    ALL_KB_DATA_CMD = 9,

    PAIRING_CMD = 0XA1,
    PAIRING_RSP = 0XA2,

    DEVICE_TYPE_KB = BIT(0),
    DEVICE_TYPE_MS = BIT(1),

    ERROR_DATA = 0X55,

} tpsll_stack_cmd_e;


// Structure representing mouse data packet
typedef struct
{
    unsigned char  report_id;  // Report identifier (distinguishes different report types)
    unsigned char  btn;        // Button states (bitmask for left/right/middle buttons)
    signed short  x;           // X-axis movement (signed value for direction)
    signed short  y;           // Y-axis movement (signed value for direction)
    unsigned char  z_wheel;    // Z-axis (scroll wheel) movement
    unsigned char  t_wheel;    // Optional: Tilt wheel movement (commented out)
} t_mouse_data;

/**
 * @brief Enumeration for variable access operation type
 */
typedef enum {
    TPSLL_ACCESS_GET,              /**< Read (get) operation */
    TPSLL_ACCESS_SET,              /**< Write (set) operation */
} tpsll_access_type_e;

/**
 * @brief Enumeration for stack layer variable ID (identifiers)
 */
typedef enum {
    TPSLL_STACK_VAR_SET_PAIRING_INFO,  /**< ID for setting pairing information */
    TPSLL_STACK_VAR_SET_SUCCESS_FLG,
    TPSLL_STACK_VAR_MAX,              /**< Maximum number of stack variables (boundary) */
} tpsll_stack_var_id_e;

/**
 * @brief Enumeration for application layer variable ID (identifiers)
 */
typedef enum {
    TPSLL_APP_VAR_SQUARE_SIDE_LEN,    /**< ID for square side length variable */
    TPSLL_APP_VAR_AUTOTEST_MODE,
    TPSLL_APP_VAR_AUTOTEST_MOUSE,

    TPSLL_APP_VAR_MAX,                /**< Maximum number of app variables (boundary) */
} tpsll_app_var_id_e;


/**
 * Configure the button scanning frequency using the division factor corresponding to the report rate. 
 * Note that the factor must be a power of two (2^N).
 */
typedef enum {
    TPSLL_BTN_SCAN_DIV_1    = 1,
    TPSLL_BTN_SCAN_DIV_2    = 2,
    TPSLL_BTN_SCAN_DIV_4    = 4,
    TPSLL_BTN_SCAN_DIV_8    = 8,
    TPSLL_BTN_SCAN_DIV_16   = 16,
    TPSLL_BTN_SCAN_DIV_32   = 32,
} tpsll_btn_scan_div_e;

/**
 * @brief Structure for app access stack layer parameters
 */
typedef struct {
    uint16_t            peer_tag;     /**< Peer device tag (identifier) */
    uint16_t            local_tag;
    uint32_t            side_id;
    uint8_t             paired_dev_type; /**< Paired device type */
    uint8_t             peer_type;     /**< Peer device type */
    uint8_t             connecting_dev;
    uint8_t             pair_success_flag;
    uint8_t             mac[MAC_ADDR_LEN]; /**< Local MAC address */
    uint8_t             peer_mac[MAC_ADDR_LEN]; /**< Peer device MAC address */
} tpsll_stack_var_t;

/**
 * @brief Structure for stack access application layer parameters
 */
typedef struct {
    uint32_t            square_side_len; /**< Square side length value */
    uint8_t             auto_test_mode_en;
    uint8_t             auto_test_mouse_en;
} tpsll_app_var_t;

typedef uint8_t (*app_mouse_report_t)(uint8_t *buf);
typedef uint8_t (*app_consume_key_report_t)(uint8_t *buf);
typedef uint8_t (*app_system_key_report_t)(uint8_t *buf);
typedef uint8_t (*app_all_key_report_t)(uint8_t *buf);
typedef uint8_t *(*usbd_get_dev_descp_t)(uint8_t bus);
typedef uint8_t (*app_get_usb_wake_status_t)(void);
typedef void (*app_set_usb_wake_status_t)(uint8_t status);
typedef void (*app_set_pt_level_t)(uint8_t value);
typedef void (*app_2p4g_clock_reinit_t)(report_rate_t report_rate);
typedef uint8_t (*app_2p4g_get_mouse_data_t)(signed short *x,signed short *y, signed char *z_wheel, signed char *t_wheel);
typedef void (*app_p24g_read_motion_data_t)(void);
typedef uint8_t (*read_optical_btn_data_t)(void);
typedef uint8_t (*read_ms_btn_data_t)(void);
typedef uint8_t (*get_ms_btn_value_t)(void);
typedef uint8_t (*read_mechanical_btn_data_t)(void);
typedef uint8_t (*Draw_a_square_test_t)(signed short *x,signed short *y);
typedef void (*app_rf_hw_init_t)(uint8_t chn);
typedef bool (*PAW3395_Is_Sensor_Idle_t)(void);
typedef void (*app_btn_set_scan_div_t)(tpsll_btn_scan_div_e div);
typedef uint8_t (*app_read_wheel_t)(int8_t *z_wheel, int8_t *t_wheel);
typedef bool (*app_rr_is_valid_t)(report_rate_t rr);
typedef void (*app_ks_pwm_mode_disable_t)(void);
typedef void (*app_ks_pwm_mode_enable_t)(void);
typedef uint8_t (*app_p24g_send_sm_msg_t)(uint8_t type, uint8_t op,  uint8_t *data, uint8_t len);

typedef struct
{
    app_mouse_report_t              app_mouse_report;
    app_consume_key_report_t        app_consume_key_report;
    app_system_key_report_t         app_system_key_report;
    app_all_key_report_t            app_all_key_report;
    usbd_get_dev_descp_t            usbd_get_dev_descp;
    app_get_usb_wake_status_t       app_get_usb_status;
    app_set_usb_wake_status_t       app_set_usb_status;
    app_set_pt_level_t              app_set_pt;
    app_2p4g_clock_reinit_t         app_clock_reinit;
    app_2p4g_get_mouse_data_t       app_get_ms_data;
    app_p24g_read_motion_data_t     app_read_motion_data;
    read_optical_btn_data_t         app_read_optical_btn;
    read_mechanical_btn_data_t      app_read_mech_btn;
    read_ms_btn_data_t              app_read_btn;
    get_ms_btn_value_t              app_get_btn_value;
    Draw_a_square_test_t            draw_square_test;
    app_rf_hw_init_t                app_rf_hw_init;
    PAW3395_Is_Sensor_Idle_t        app_is_sensor_idle;
    app_btn_set_scan_div_t          app_btn_set_scan_div;
    app_read_wheel_t                app_read_wheel;
    app_rr_is_valid_t               app_rr_is_valid;
    app_ks_pwm_mode_enable_t        app_ks_pwm_mode_enable;
    app_ks_pwm_mode_disable_t       app_ks_pwm_mode_disable;
    app_p24g_send_sm_msg_t          app_p24g_send_sm_msg;
} tpsll_app_cb_t;

/**
 * @brief Callback function type for stack event notification
 *
 * @param evt: Stack event type
 * @param data: Pointer to event-related data buffer
 * @param len: Length of the event data (in bytes)
 */
typedef void (*tpsll_evt_callback_t)(tpsll_evt_t evt, void *data, uint16_t len);

/**
 * @brief Callback function type for application layer variable access
 *
 * @param p_app_var: Pointer to app variable structure
 * @param access_type: Access operation type (GET/SET)
 * @param var_id: ID of the accessed app variable
 */
typedef void (*tpsll_app_var_cb_t)(tpsll_app_var_t *p_app_var, tpsll_access_type_e access_type, tpsll_app_var_id_e var_id);


typedef struct {
    tpsll_app_var_cb_t app_var_cb;
    tpsll_app_cb_t  app_cb;
    uint32_t * km_fifo_ptr;
    uint32_t * spp_fifo_ptr;
    uint32_t * next_wakeup_tick_ptr;
    bool * suspend_allowed_ptr;
    bool * is_stk_busy_ptr;
    uint8_t  report_rate;
} tpsll_stack_config_t;



#define ENCRYPT_KEY_MAX_LEN     32
#define ENCRYPT_HASH_KEY_LEN    4
#define VENDOR_ID_LEN           2


#define TLK_ERR_BASE_NUM                                    (0x0)    /**< Global error base */

#define TLK_SUCCESS                                         (TLK_ERR_BASE_NUM + 0)    /**< Successful command */
#define TLK_ERR_NULL                                        (TLK_ERR_BASE_NUM + 1)    /**< Null pointer */
#define TLK_ERR_INVALID_PARAM                               (TLK_ERR_BASE_NUM + 2)    /**< Invalid parameter */
#define TLK_ERR_BUSY                                        (TLK_ERR_BASE_NUM + 3)    /**< Device or resourse Busy */
#define TLK_ERR_INVALID_STATE                               (TLK_ERR_BASE_NUM + 4)    /**< Invalid state, operation disallowed in this state */
#define TLK_ERR_BUFFER_EMPTY                                (TLK_ERR_BASE_NUM + 5)    /**< Buffer/FIFO is empty */
#define TLK_ERR_NO_MEM                                      (TLK_ERR_BASE_NUM + 6)    /**< No memory available for operation */
#define TLK_ERR_INVALID_LENGTH                              (TLK_ERR_BASE_NUM + 7)    /**< Invalid length */
#define TLK_ERR_TIMEOUT                                     (TLK_ERR_BASE_NUM + 8)    /**< Operation timed out */
#define TLK_ERR_INTERNAL                                    (TLK_ERR_BASE_NUM + 9)    /**< Internal error */
#define TLK_ERR_DEV_NOT_FOUND                               (TLK_ERR_BASE_NUM + 10)   /**< Destination device not found */
#define TLK_ERR_CMD_NOT_SUPPORT                             (TLK_ERR_BASE_NUM + 11)   /**< cmd not supported */
#define TLK_ERR_BUFFER_FULL                                 (TLK_ERR_BASE_NUM + 12)   /**< Buffer/FIFO is full */




