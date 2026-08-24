/********************************************************************************************************
 * @file    app.c
 *
 * @brief   This is the source file for BLE SDK
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
#include <vendor/n22_2p4g_tpll/app_buffer.h>
#include <vendor/n22_2p4g_tpll/app_config.h>
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"

#include "hci_transport/hci_tr.h"
#include "hci_transport/hci_dfu.h"
#include "../../stack/2p4g/tpll/tpll.h"
#include "../../stack/2p4g/tl_tpll/tl_tpll.h"


#define MY_APP_ADV_CHANNEL  BLT_ENABLE_ADV_ALL
#define MY_ADV_INTERVAL_MIN ADV_INTERVAL_30MS
#define MY_ADV_INTERVAL_MAX ADV_INTERVAL_40MS

#include "stack/ble/hci/hci_vendor.h"

#include "stack/multicore_comm/service/mcc.h"
#include "stack/multicore_comm/service/service_n22.h"

#define N22_MAILBOX_TEST            0

//////////////////////////////////////////////////////////////////////////////
//  Adv Packet, Response Packet
//////////////////////////////////////////////////////////////////////////////
const u8 tbl_advData[] = {
    0x05,
    0x09,
    't',
    'H',
    'C',
    'I',
    0x02,
    0x01,
    0x05, // BLE limited discoverable mode and BR/EDR not supported
};

const u8 tbl_scanRsp[] = {
    0x07,
    0x09,
    't',
    'H',
    'C',
    'I',
    '0',
    '1', //scan name " tHCI01"
};

/////////////////////////////////////blc_register_hci_handler for spp//////////////////////////////
int rx_from_uart_cb(void) //UART data send to Master,we will handler the data as CMD or DATA
{
    return 0;
}

int tx_to_uart_cb(void)
{
    return 0;
}

u8 led_status;

unsigned char app_vendor_cb(u8 pCmdparaLen, u8 opCode_ocf, hci_vendor_CmdParams_t *pCmd, hci_vendor_EndStatusParam_t *pRetParam)
{
    u8 re_length;
    u8 result;
    re_length = 0;
    switch (opCode_ocf) //1byte
    {
    case 0X00:
        //01 00 FD 01 01 open  led
        //01 00 FD 01 00 close led
        if (pCmdparaLen == 1) {
            #if UI_LED_ENABLE
            gpio_write(GPIO_LED_GREEN, pCmd[0]);
            #endif 
            led_status = pCmd[0];
             result     = BLE_SUCCESS;
            blt_hci_vendor_setEventCode(HCI_EVT_CMD_COMPLETE);
            re_length = hci_cmdComplete_evt(1, opCode_ocf, HCI_CMD_VENDOR_OPCODE_OGF | HCI_VENDOR_CMD_FU_OPCODE_OGF, 1, &result, pRetParam);
        }
        break;
    case 0X01:
        //01 01 FD 01 00 get led status
        result = led_status;
        blt_hci_vendor_setEventCode(HCI_EVT_CMD_STATUS);
        re_length = hci_cmdStatus_evt(1, opCode_ocf, HCI_CMD_VENDOR_OPCODE_OGF | HCI_VENDOR_CMD_FU_OPCODE_OGF, result, pRetParam);
        break;
    case 0x02:
        //TODO
        break;
    }
    return re_length;
}

#if (APP_PAWR_EN)
#define APP_PAWR_SYNC_RSP_DATA_LENGTH 100
#define APP_PAWR_SYNC_SETS_NUMBER     1
_attribute_ble_data_retention_ u8 app_pawr_sync_rsp_buf[APP_PAWR_SYNC_RSP_DATA_LENGTH * APP_PAWR_SYNC_SETS_NUMBER] = {0};
#endif

#if APP_LE_PERIODIC_ADV_EN
    /*
     * @brief   Periodic Advertising Set number and data buffer length
     *
     * APP_PERID_ADV_SETS_NUMBER:
     * Number of Supported Periodic Advertising Sets, no exceed "PERIODIC_ADV_NUMBER_MAX"
     *
     * APP_PERID_ADV_DATA_LENGTH:
     * Maximum Periodic Advertising Data Length. can not exceed 1650.
     */
    #define APP_PERID_ADV_SETS_NUMBER 2   //1//EBQ test need to change it to the supported value
    #define APP_PERID_ADV_DATA_LENGTH 100 //1024
    _attribute_ble_data_retention_ u8 app_peridAdvSet_buffer[PERD_ADV_PARAM_LENGTH * APP_PERID_ADV_SETS_NUMBER];
    _attribute_iram_noinit_data_ u8   app_peridAdvData_buffer[APP_PERID_ADV_DATA_LENGTH * APP_PERID_ADV_SETS_NUMBER];

#endif

#if APP_PAWR_EN
    #define APP_PAWR_SUBEVT_DATA_COUNT   4
    #define APP_PAWR_SUBEVT_DATA_LENGTH  100
   _attribute_iram_bss_  u8 app_peridAdvSet_buffer[PERD_ADV_PARAM_LENGTH * APP_PERID_ADV_SETS_NUMBER];
   _attribute_iram_bss_ u8 app_peridAdvWrData_buffer[APP_PERID_ADV_SETS_NUMBER * APP_PAWR_SUBEVT_DATA_LENGTH * APP_PAWR_SUBEVT_DATA_COUNT];
#endif
///////////////////////////////////////////

int app_hci_send_data(u32 h, u8 *para, int n)
{
    u8 send_buf[HCI_TX_FIFO_SIZE];
    u8 *p = &send_buf[0];
    u16 len = 0;
    u8 is_adv_rpt = 0;   //adv report
    if (!p || n >= HCI_TX_FIFO_SIZE) {
        tlkapi_printf(APP_HCI_LOG_EN, "[APP][HCI][ERROR]  TX FIFO overflow - Failed to send data");
        return -1;
    }

    int nl = n + 4;
    if (h & HCI_FLAG_EVENT_BT_STD) {
        if ((h & 0xff) == HCI_EVT_LE_META &&
            (para[0] == HCI_SUB_EVT_LE_ADVERTISING_REPORT || para[0] == HCI_SUB_EVT_LE_EXTENDED_ADVERTISING_REPORT)) {
            is_adv_rpt = 1;
            if(mcc_n22_hci_is_tx_over_threshold(80))
            {
//                  static u32 counter = 0;
//                  tlk_printf(" s %d  %d  %d \r\n",counter++,free_len ,revert_space_for_acl);
                    return -1;
            }
        }

        len = n + 3;                  //3: 1(HCI Type) + 1(Event Code) + 1(Parameter Total Length)
        *p++ = HCI_TYPE_EVENT;          //HCI Type
        *p++ = h;                       //Event Code
        *p++ = n;                       //Parameter Total Length
        smemcpy(p, para, n);
        p += n;
    } else if (h & HCI_FLAG_ACL_BT_STD) //ACL data
    {
        /*
        +------+-------+---------+-----------+-----------------+
        | 2    | 1     | 2       | 2         | n               |
        +------+-------+---------+-----------+-----------------+
        | len  | type  | handle  | data_len  | data_total_len  |
        +------+-------+---------+-----------+-----------------+
         */
        n    = para[1]; // para[1]: rf_len

        len = n + 5; // handle| PB | BC :2byte    data total length: 2byte  : ACL date Type(0x02) 1byte

        *p++ = HCI_TYPE_ACL_DATA;
        *p++ = h;                                                                         // para[1]: llid       //Handle:12|PB flag:2|BC flag:2 (2B)
        *p++ = ((h >> 8) & 0x0F) | ((para[0] & 3) == 0x01 /* L2CAP_CONTINUING_PKT */ ? 0x10 : 0x20); //start llid 2 ->0x20 ;  continue llid 1 ->0x10
        *p++ = n;                                                                         //HCI ACL Data Total Length (2B)
        *p++ = n >> 8;
        smemcpy(p, para + 2, n);
        p += n;

    }
    else
    {
        tlkapi_printf(APP_HCI_LOG_EN, "[APP][HCI][ERROR]  Unsupported operation - blc_hci_send_data");
    }

    if(len)
    {
        tlkapi_send_string_data(APP_HCI_LOG_EN, "[APP][HCI] Event packet",send_buf,len);
        mcc_n22_hci_send_msg(send_buf, len);
        delay_us(10);
    }
    else
    {
        tlkapi_printf(APP_HCI_LOG_EN, "[APP][HCI][ERROR] TX FIFO is empty (length 0) - Cannot send data");
    }

    return 0;
}


/* Forward declarations of n22_* functions (defined later in this file) */
uint32_t n22_set_mode(tl_tpll_mode_t mode);
uint32_t n22_set_bitrate(TPLL_BitrateTypeDef bitrate);
uint32_t n22_set_retransmit_para(uint16_t count, uint16_t delay_us);
uint32_t n22_write_payload(uint8_t pipe,uint8_t ack, uint8_t *data, uint8_t len);
uint32_t n22_set_access_code_length(uint8_t length);
uint32_t n22_set_access_code(uint8_t pipe, uint8_t *addr);
uint32_t n22_set_rf_channel(uint8_t chn);
uint32_t n22_set_set_prefixes(uint8_t * p_prefixes, uint8_t num_pipes);
uint32_t n22_set_prefixes(uint8_t *p_prefixes, uint8_t num_pipes);
uint32_t n22_enable_pipes(uint8_t enable_mask);
uint32_t n22_apply_config(void);
uint32_t n22_start_tx(void);
uint32_t n22_start_rx(void);
/**
 * @brief       Dual-core 2.4G HCI command protocol definitions.
 *
 * Command packet format (Host core -> N22 core, via shared memory):
 *   byte 0: HCI_TYPE_CMD (0x01)      - type tag
 *   byte 1: cmd                         - sub-command opcode (@ref n22_2p4g_cmd_e)
 *   byte 2: param_len                   - parameter length (excluding header)
 *   byte 3..: param                     - parameter data
 *
 * Response packet format (N22 core -> Host core):
 *   byte 0: HCI_TYPE_CMD (0x01)
 *   byte 1: cmd                         - echoed sub-command opcode
 *   byte 2: status                      - execution status (0 = success, others = error code)
 */
#define N22_2P4G_CMD_HEADER_LEN     3   /* HCI_TYPE_CMD + cmd + param_len */

typedef enum {
    N22_2P4G_CMD_SET_MODE           = 0x01, /* param: mode(1B) */
    N22_2P4G_CMD_SET_BITRATE        = 0x02, /* param: bitrate(1B) */
    N22_2P4G_CMD_SET_RETRANSMIT     = 0x03, /* param: count(2B LE) + delay_us(2B LE) */
    N22_2P4G_CMD_WRITE_PAYLOAD      = 0x04, /* param: pipe(1B) + data(NB) */
    N22_2P4G_CMD_SET_ACCESS_CODE_LEN= 0x05, /* param: length(1B) */
    N22_2P4G_CMD_SET_ACCESS_CODE    = 0x06, /* param: pipe(1B) + addr(NB) */
    N22_2P4G_CMD_SET_RF_CHANNEL     = 0x07, /* param: channel(1B) */
    N22_2P4G_CMD_SET_PREFIXES       = 0x08, /* param: num_pipes(1B) + prefixes(NB) */
    N22_2P4G_CMD_ENABLE_PIPES       = 0x09, /* param: enable_mask(1B) */
    N22_2P4G_CMD_APPLY_CONFIG       = 0x0A, /* param: none */
    N22_2P4G_CMD_START_RX           = 0x0B, /* param: none */
    N22_2P4G_CMD_START_TX           = 0x0C, /* param: none */
    N22_2P4G_CMD_START_RX_STA       = 0x0D, /* param: none */
    N22_2P4G_CMD_START_TX_STA       = 0x0E, /* param: none */
} n22_2p4g_cmd_e;

/**
 * @brief       Dispatch a 2.4G HCI command and send back a response.
 * @param[in]   p   Pointer to command packet (byte 0 is HCI_TYPE_CMD).
 * @param[in]   n   Total packet length in bytes.
 * @return      0 on success, -1 on invalid packet.
 */
static int n22_2p4g_cmd_dispatch(u8 *p, int n)
{
    u8 rsp[4];

    if (n < N22_2P4G_CMD_HEADER_LEN) {
        return -1;
    }

    u8 cmd       = p[1];
    u8 param_len = p[2];
    u8 *param    = &p[3];

    if ((n - N22_2P4G_CMD_HEADER_LEN) < param_len) {
        return -1;
    }

    uint32_t status = TL_TPLL_SUCCESS;

    switch (cmd)
    {
    case N22_2P4G_CMD_SET_MODE:
        if (param_len < 1) { status = TL_TPLL_ERROR_INVALID_PARAM; break; }
        status = n22_set_mode((tl_tpll_mode_t)param[0]);
        break;

    case N22_2P4G_CMD_SET_BITRATE:
        if (param_len < 1) { status = TL_TPLL_ERROR_INVALID_PARAM; break; }
        status = n22_set_bitrate((TPLL_BitrateTypeDef)param[0]);
        break;

    case N22_2P4G_CMD_SET_RETRANSMIT:
        if (param_len < 4) { status = TL_TPLL_ERROR_INVALID_PARAM; break; }
        {
            uint16_t count    = (uint16_t)param[0] | ((uint16_t)param[1] << 8);
            uint16_t delay_us = (uint16_t)param[2] | ((uint16_t)param[3] << 8);
            status = n22_set_retransmit_para(count, delay_us);
        }
        break;

    case N22_2P4G_CMD_WRITE_PAYLOAD:
        if (param_len < 1) { status = TL_TPLL_ERROR_INVALID_PARAM; break; }
        {
            u8 pipe = param[0];
            u8 ack = param[1];
            u8 *data = &param[2];
            u8 data_len = param_len - 2;
            status = n22_write_payload(pipe,ack, data, data_len);
        }
        break;

    case N22_2P4G_CMD_SET_ACCESS_CODE_LEN:
        if (param_len < 1) { status = TL_TPLL_ERROR_INVALID_PARAM; break; }
        status = n22_set_access_code_length(param[0]);
        break;

    case N22_2P4G_CMD_SET_ACCESS_CODE:
        if (param_len < 1) { status = TL_TPLL_ERROR_INVALID_PARAM; break; }
        {
            u8 pipe = param[0];
            u8 *addr = &param[1];
            status = n22_set_access_code(pipe, addr);
        }
        break;

    case N22_2P4G_CMD_SET_RF_CHANNEL:
        if (param_len < 1) { status = TL_TPLL_ERROR_INVALID_PARAM; break; }
        status = n22_set_rf_channel(param[0]);
        break;

    case N22_2P4G_CMD_SET_PREFIXES:
        if (param_len < 1) { status = TL_TPLL_ERROR_INVALID_PARAM; break; }
        {
            u8 num_pipes = param[0];
            u8 *prefixes = &param[1];
            status = n22_set_prefixes(prefixes, num_pipes);
        }
        break;
    case N22_2P4G_CMD_ENABLE_PIPES:
        if (param_len < 1) { status = TL_TPLL_ERROR_INVALID_PARAM; break; }
        status = n22_enable_pipes(param[0]);
        break;
    case N22_2P4G_CMD_APPLY_CONFIG:
        status = n22_apply_config();
        break;

    case N22_2P4G_CMD_START_RX:
        status = n22_start_rx();
        break;

    case N22_2P4G_CMD_START_TX:
        status = n22_start_tx();
        break;


    default:
        status = TL_TPLL_ERROR_INVALID_PARAM;
        break;
    }

    /* Build and send response: HCI_TYPE_CMD + cmd + status */
    rsp[0] = HCI_TYPE_CMD;
    rsp[1] = cmd;
    rsp[2] = 1;
    rsp[3] = (u8)status;
    mcc_n22_pri_send_msg(rsp, 4);

    return 0;
}

#if (TIFS_VARIATION_WORKAROUND_MLP_CODE_IN_RAM)
_attribute_ble_ram_code_noinline_
#else
_attribute_no_inline_
#endif
int pri_hci_handler(u8 *p, int n)
{
    //tlkapi_send_string_data((stkLog_mask & STK_LOG_HCI), "hci rx",p, n);
//return 0;

    if (p[0] == HCI_TYPE_ACL_DATA) {
        // must make sure p is align 2 byte or the type cast will error
//        blc_hci_receiveHostACLData((hci_acl_data_pkt_t *)(p + 1));


        return 1;
    }
    else if (p[0] == HCI_TYPE_CMD) {
        n22_2p4g_cmd_dispatch(p, n);
    }


    return 0;
}
extern unsigned int n22_tpll_init(int deepRetWakeUp);
volatile unsigned int debug_tpll[48] = {0};
/**
 * @brief      use initialization
 * @param[in]  none.
 * @return     none.
 */
void user_init_normal(void)
{
/* clock sync && other configuration address sync */
#if (HCI_INTERFACE == HCI_SHAREMEMORY)
    /* register ble h2c send callback */
//    mcc_n22_register_shm_recv_cb(TLK_SHM_MSG_HCI, (shm_recv_cb_t)blc_hci_handler);

    mcc_n22_register_shm_recv_cb(TLK_SHM_MSG_2P4G, (shm_recv_cb_t)pri_hci_handler);
    /* register ble c2h send callback */
    extern int n22_blc_hci_send_data(u32 h, u8 *para, int n);
//    blc_hci_registerControllerEventHandler(n22_blc_hci_send_data);
#endif
    delay_ms(200);
    n22_tpll_init(0);
    //////////////////////////// basic hardware Initialization  Begin //////////////////////////////////
    /* random number generator must be initiated here( in the beginning of user_init_normal).
     * When deepSleep retention wakeUp, no need initialize again */
    random_generator_init();

//    blc_ll_initBasicMCU();


#if 0
#if (TLKAPI_DEBUG_ENABLE)
    tlkapi_debug_init();
    blc_debug_enableStackLog(STK_LOG_HCI);
#endif



    //blc_readFlashSize_autoConfigCustomFlashSector();

    /* attention that this function must be called after "blc readFlashSize_autoConfigCustomFlashSector" !!!*/
    //blc_app_loadCustomizedParameters_normal();
    //////////////////////////// basic hardware Initialization  End /////////////////////////////////


    //////////////////////////// BLE stack Initialization  Begin //////////////////////////////////

    u8 mac_public[6];
    u8 mac_random_static[6];
    /* tl322x: Generate a default address */    //////////// Controller Initialization  Begin /////////////////////////
    blc_ll_initBasicMCU();

    blc_ll_initStandby_module(mac_public); //mandatory

    blc_ll_initLegacyAdvertising_module();
#if ACL_CENTRAL_MAX_NUM
    blc_ll_initLegacyScanning_module();

    blc_ll_initLegacyInitiating_module();
#endif
    blc_ll_initAclConnection_module();
    #if ACL_CENTRAL_MAX_NUM
        blc_ll_initAclCentralRole_module();
    #endif
    #if ACL_PERIPHR_MAX_NUM
        blc_ll_initAclPeriphrRole_module();
    #endif
    blc_ll_configLegacyAdvEnableStrategy(LEG_ADV_EN_STRATEGY_3);

    blc_ll_setAclConnMaxOctetsNumber(ACL_CONN_MAX_RX_OCTETS, ACL_CENTRAL_MAX_TX_OCTETS, ACL_PERIPHR_MAX_TX_OCTETS);

    /* all ACL connection share same RX FIFO */
    blc_ll_initAclConnRxFifo(app_acl_rx_fifo, ACL_RX_FIFO_SIZE, ACL_RX_FIFO_NUM);

    /* ACL Central TX FIFO */
#if ACL_CENTRAL_MAX_NUM
    blc_ll_initAclCentralTxFifo(app_acl_cen_tx_fifo, ACL_CENTRAL_TX_FIFO_SIZE, ACL_CENTRAL_TX_FIFO_NUM, ACL_CENTRAL_MAX_NUM);
#endif
#if ACL_PERIPHR_MAX_NUM
    /* ACL Peripheral TX FIFO */
    blc_ll_initAclPeriphrTxFifo(app_acl_per_tx_fifo, ACL_PERIPHR_TX_FIFO_SIZE, ACL_PERIPHR_TX_FIFO_NUM, ACL_PERIPHR_MAX_NUM);
#endif

    #if (APP_WORKAROUND_TX_FIFO_4K_LIMITATION_EN && (ACL_CENTRAL_MAX_TX_OCTETS > 230 || ACL_PERIPHR_MAX_TX_OCTETS > 230))
        /* extend TX FIFO size for MAX_TX_OCTETS > 230 if user want use 16 or 32 FIFO */
        blc_ll_initAclConnCacheTxFifo(app_acl_cache_Txfifo, 260, 32);
    #endif

    blc_ll_setMaxConnectionNumber(ACL_CENTRAL_MAX_NUM, ACL_PERIPHR_MAX_NUM);
#if ACL_CENTRAL_MAX_NUM
    blc_ll_setAclCentralBaseConnectionInterval(CONN_INTERVAL_10MS);
    blc_ll_setCreateConnectionTimeout(50000); //[!!!important]
#endif
    rf_set_power_level_index(RF_POWER_P3dBm);

    //////////// HCI Initialization  Begin /////////////////////////
    #if HCI_TR_EN
    /* HCI RX FIFO */
    if (blc_ll_initHciRxFifo(app_hci_rxfifo, HCI_RX_FIFO_SIZE, HCI_RX_FIFO_NUM) != BLE_SUCCESS) {
        while (1)
            ;
    }
    /* HCI TX FIFO */
    if (blc_ll_initHciTxFifo(app_hci_txfifo, HCI_TX_FIFO_SIZE, HCI_TX_FIFO_NUM) != BLE_SUCCESS) {
        while (1)
            ;
    }
    #endif
    /* HCI RX ACL FIFO (host to controller)*/
    if (blc_ll_initHciAclDataFifo(app_hci_rxAclfifo, HCI_RX_ACL_FIFO_SIZE, HCI_RX_ACL_FIFO_NUM) != BLE_SUCCESS) {
        while (1)
            ;
    }


    /* HCI Data && Event */
    blc_hci_registerControllerDataHandler(blc_hci_sendACLData2Host);
 //   blc_hci_registerControllerEventHandler(app_hci_send_data); //controller hci event to host all processed in this func


    //bluetooth event
    blc_hci_setEventMask_cmd(HCI_EVT_MASK_DISCONNECTION_COMPLETE);
    //bluetooth low energy(LE) event, all enable
    #if (APP_LE_CHANNEL_SOUNDING_TEST_MODE)
        blc_hci_le_setEventMask_cmd(0xFFFFFFFF);
        blc_hci_le_setEventMask_2_cmd(0x7FFFFFFF);
    #elif (APP_PAWR_EN)
        blc_hci_le_setEventMask_cmd(HCI_LE_EVT_MASK_CONNECTION_COMPLETE | HCI_LE_EVT_MASK_ADVERTISING_REPORT | HCI_LE_EVT_MASK_CONNECTION_UPDATE_COMPLETE |HCI_LE_EVT_MASK_PERIODIC_ADVERTISING_SYNC_TRANSFER_RECEIVED);
    #else
        blc_hci_le_setEventMask_cmd(HCI_LE_EVT_MASK_CONNECTION_COMPLETE | HCI_LE_EVT_MASK_ADVERTISING_REPORT | HCI_LE_EVT_MASK_CONNECTION_UPDATE_COMPLETE );
    #endif

    #if APP_LE_EXTENDED_ADV_EN
        /* Extended ADV module and ADV Set Parameters buffer initialization */
        blc_ll_initExtendedAdvModule_initExtendedAdvSetParamBuffer(app_extAdvSetParam_buf, APP_EXT_ADV_SETS_NUMBER);
        blc_ll_initExtendedAdvDataBuffer(app_extAdvData_buf, APP_EXT_ADV_DATA_LENGTH);
        blc_ll_initExtendedScanRspDataBuffer(app_extScanRspData_buf, APP_EXT_SCANRSP_DATA_LENGTH);
    #endif

    #if APP_LE_PERIODIC_ADV_EN
        blc_ll_initPeriodicAdvModule_initPeriodicdAdvSetParamBuffer(app_peridAdvSet_buffer, APP_PERID_ADV_SETS_NUMBER);
        blc_ll_initPeriodicAdvDataBuffer(app_peridAdvData_buffer, APP_PERID_ADV_DATA_LENGTH);
    #endif


    #if APP_LE_EXTENDED_SCAN_EN
        blc_ll_initExtendedScanning_module();
        blc_ll_configScanEnableStrategy(SCAN_STRATEGY_1);
    #endif

    #if (APP_LE_EXTENDED_INIT_EN)
        blc_ll_initExtendedInitiating_module();
    #endif

    #if (APP_SYNCHRONIZED_RECEIVER_EN || APP_ISOCHRONOUS_BROADCASTER_SYNC_EN)
        blc_ll_initPeriodicAdvertisingSynchronization_module();
    #endif

    #if (APP_PAST_EN)
        blc_ll_initPAST_module();
    #endif

    #if (APP_POWER_CONTROL)
        blc_ll_initPCL_module();
    #endif

    #if (APP_CHN_CLASS_EN)
        blc_ll_initChnClass_feature();
    #endif

    #if (APP_PAWR_EN)
        blc_ll_initPAwRsync_module(1); //PAwR sync
        blc_ll_initPAwRsync_rspDataBuffer(app_pawr_sync_rsp_buf, APP_PAWR_SYNC_RSP_DATA_LENGTH);

        blc_ll_initPeriodicAdvWrModule_initPeriodicdAdvWrSetParamBuffer(app_peridAdvSet_buffer, APP_PERID_ADV_SETS_NUMBER);
        blc_ll_initPeriodicAdvWrDataBuffer(app_peridAdvWrData_buffer, APP_PAWR_SUBEVT_DATA_LENGTH, APP_PAWR_SUBEVT_DATA_COUNT);
    #endif

    #if (APP_LE_CHANNEL_SOUNDING)
        blc_ll_initCsConfigParam(app_CsConfigParam, APP_CS_CONFIG_NUM);
        blc_ll_initCsRxFifo(app_cs_rx_buf, CS_RX_FIFO_SIZE, CS_RX_FIFO_NUM);

        //Set cs use tx power level
        blc_cs_set_tx_power_level(CS_USE_TX_POWER_LEVEL);

        blc_ll_initCsInitiatorModule();

        blc_ll_initCsReflectorModule();
    #endif

    #if APP_LE_2M_CODED_PHY_EN
        blc_ll_init2MPhyCodedPhy_feature(); //need 2M/Coded PHY feature
    #endif

    #if APP_LE_CHANNEL_SELECTION_ALGORITHM_2_EN
        blc_ll_initChannelSelectionAlgorithm_2_feature(); //need CSA #2
    #endif

    #if (APP_LE_CHANNEL_SOUNDING_TEST_MODE)
        blc_ll_initCsConfigParam(app_CsConfigParam, APP_CS_CONFIG_NUM);
        blc_ll_initCsRxFifo(app_cs_rx_buf, CS_RX_FIFO_SIZE, CS_RX_FIFO_NUM);
        //Set cs use tx power level
        blc_cs_set_tx_power_level(CS_USE_TX_POWER_LEVEL);
    #endif

    #if (APP_LE_PHY_TEST_EN)
        blc_phy_initPhyTest_module();
    #endif

    blc_ll_setAutoExchangeDataLengthEnable(0);

    irq_enable();

    u8 error_code = blc_contr_checkControllerInitialization();
    if (error_code != INIT_SUCCESS) {
        /* It's recommended that user set some UI alarm to know the exact error, e.g. LED shine, print log */
           
        #if (TLKAPI_DEBUG_ENABLE)
            tlkapi_send_string_data(APP_LOG_EN, "[APP][INI] Controller INIT ERROR", &error_code, 1);
            while (1) {
                tlkapi_debug_handler();
            }
        #else
            while (1)
                ;
        #endif
    }

    #if HCI_TR_EN
    {
        /* HCI Transport initialization */
        HCI_TransportInit();
    }
    #endif

    #if HCI_DFU_EN
    {
        DFU_Init();
    }
    #endif





    blc_hci_registerControllerVendorCmdProcess_Callback(hci_vendor_Process);


#if (BLE_APP_PM_ENABLE)
    blc_ll_initPowerManagement_module();
    blc_pm_setSleepMask(PM_SLEEP_LEG_ADV | PM_SLEEP_ACL_PERIPHR);

    #if (BLE_APP_PM_ENABLE && PM_DEEPSLEEP_RETENTION_ENABLE)
//    blc_app_setDeepsleepRetentionSramSize();
    blc_pm_setDeepsleepRetentionEnable(PM_DeepRetn_Enable);
    blc_pm_setDeepsleepRetentionThreshold(95);
        /*!< early wakeup time with a threshold of approxiamtely 30us. */
        #if (MCU_CORE_TYPE == MCU_CORE_B91)
    blc_pm_setDeepsleepRetentionEarlyWakeupTiming(620);
        #elif (MCU_CORE_TYPE == MCU_CORE_B92)
    blc_pm_setDeepsleepRetentionEarlyWakeupTiming(655);
        #elif (MCU_CORE_TYPE == MCU_CORE_TL321X)
    blc_pm_setDeepsleepRetentionEarlyWakeupTiming(540);
        #elif (MCU_CORE_TYPE == MCU_CORE_TL721X)
    blc_pm_setDeepsleepRetentionEarlyWakeupTiming(580);
       #elif (MCU_CORE_TYPE == MCU_CORE_TL322X)
    blc_pm_setDeepsleepRetentionEarlyWakeupTiming(920);
        #endif
    #else
    blc_pm_setDeepsleepRetentionEnable(PM_DeepRetn_Disable);
    #endif
#endif
#endif
    tlkapi_send_string_data(APP_LOG_EN, "[APP][INI] n22 ble controller init", 0, 0);
    debug_tpll[46] = 0xaa5555aa;
    //tlkapi_printf(APP_LOG_EN,"[APP][INI] n22 ble controller init build time: %s %s\n", __DATE__, __TIME__);
}

void user_init_deepRetn(void)
{
#if (PM_DEEPSLEEP_RETENTION_ENABLE)
    blc_app_loadCustomizedParameters_deepRetn();

    blc_ll_initBasicMCU(); //mandatory

    blc_ll_recoverDeepRetention();

    DBG_CHN0_HIGH;
    irq_enable();
#endif
}

tl_tpll_payload_t tx_payload = TL_TPLL_CREATE_PAYLOAD(
    TL_TPLL_PIPE0, 0xab, 0xcd, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f);

// ptx_buffer and rx_buf will be used by tpll.c and tl_tpll.c.
unsigned char ptx_buffer[TPLL_PIPE_NUM * TPLL_TX_FIFO_SIZE * TPLL_TX_FIFO_NUM] __attribute__((aligned(4))) = {};
unsigned char rx_buf[TPLL_PIPE_RX_FIFO_SIZE * TPLL_PIPE_RX_FIFO_NUM] __attribute__((aligned(4)))           = {};




volatile tl_tpll_config_t tl_tpll_config = TL_TPLL_DEFAULT_CONFIG;
volatile tl_tpll_payload_t ReadRxPayload;
volatile unsigned int rx_crc_cnt = 0;
extern volatile unsigned char  rx_timeout_flag;
extern volatile unsigned char  tx_ds_flag;
extern volatile unsigned char maxretry_flag;
extern volatile unsigned char tx_done_flag;
extern volatile unsigned int math_cnt;
extern volatile unsigned char rx_flag;


#if (TIFS_VARIATION_WORKAROUND_MLP_CODE_IN_RAM)
_attribute_ram_code_
#endif
__attribute__((optimize("-O0"))) void main_loop(void)
{
    #if HCI_TR_EN
        HCI_TransportPoll();
    #endif

    #if HCI_DFU_EN
        DFU_TaskStart();
    #endif

    #if (HCI_INTERFACE == HCI_SHAREMEMORY)
        mcc_n22_loop();
    #endif

        debug_tpll[47]++;
        unsigned char flag=0;
        if(tl_tpll_config.mode == TL_TPLL_MODE_PTX && tx_payload.noack == 1)
        {
            if (rx_timeout_flag | tx_ds_flag | maxretry_flag ) {//| tx_done_flag                tx_done_flag = 0;


                n22_send_tx_status(0);
                debug_tpll[43]++;

                math_cnt++;


                rx_timeout_flag = 0;
                tx_ds_flag = 0;
                maxretry_flag = 0;

            }
        }
        else if(tl_tpll_config.mode == TL_TPLL_MODE_PTX && tx_payload.noack == 0)
        {
            flag = tx_done_flag;
            if (flag) {//| tx_done_flag
                n22_send_tx_status(0);
                debug_tpll[42]++;
                tx_done_flag = 0;
            }

        }
        else if(tl_tpll_config.mode == TL_TPLL_MODE_PRX)
        {
            flag = rx_flag;
            if (1 == flag) {
                debug_tpll[42]++;
                rx_flag = 0;
                tl_tpll_read_rx_payload(&ReadRxPayload);

                if (ReadRxPayload.crc == 1) {
                    rx_crc_cnt++;
                    n22_send_rx_sta(&ReadRxPayload);
                }


            }
        }

//    #if (TLKAPI_DEBUG_ENABLE)
//        tlkapi_debug_handler();
//    #endif
    #if    UI_LED_ENABLE
    static volatile unsigned int tickNow = 0;
    if (clock_time_exceed(tickNow, 100*1000)) {
        tickNow = clock_time();
         gpio_toggle(GPIO_LED_WHITE);
    }
    #endif

}









_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os"))) unsigned int n22_tpll_init(int deepRetWakeUp)
{
    uint32_t err_code = 0;

    uint8_t base_address_0[4] = {0x41, 0x76, 0x71, 0xe7};
    uint8_t base_address_1[4] = {0xe7, 0xe7, 0xe7, 0xe7};
    uint8_t addr_prefix[8] = {0x29, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};



    tl_tpll_config.mode = TL_TPLL_MODE_PTX;
    tl_tpll_config.bitrate = TPLL_BITRATE_4MBPS;
    tl_tpll_config.tx_output_power_idx = RF_POWER_INDEX_P0p00dBm;
    tl_tpll_config.retransmit_delay = 150;
    tl_tpll_config.retransmit_count = 0;
    tl_tpll_config.payload_length = 32;
    tl_tpll_config.irq_mask = FLD_RF_IRQ_RX_DR | FLD_RF_IRQ_RX | FLD_RF_IRQ_TX | FLD_RF_IRQ_TX_DS | FLD_RF_IRQ_RX_TIMEOUT | FLD_RF_IRQ_PKT_MATCH | FLD_RF_IRQ_PKT_UNMATCH;
    tl_tpll_config.deepRetWakeUp = deepRetWakeUp;
    tl_tpll_config.fast_settle_en = 1;
    tx_payload.noack = 1;

    clic_interrupt_enable(IRQ_ZB_RT);
    rf_clr_irq_mask(FLD_RF_IRQ_ALL);
    core_interrupt_enable();

    return err_code;
}




/**
 * @brief       D25F communication configuration functions.
 *              Based on tl_tpll_start, provide individual parameter setting
 *              for interoperating with D25F.
 */

/**
 * @brief   Mode setting function - Set PTX/PRX mode for D25F communication.
 * @param   mode    TL_TPLL_MODE_PTX or TL_TPLL_MODE_PRX.
 * @retval  TL_TPLL_SUCCESS              If mode was set successfully.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM  If mode was invalid.
 */
uint32_t n22_set_mode(tl_tpll_mode_t mode)
{
    if (mode != TL_TPLL_MODE_PTX && mode != TL_TPLL_MODE_PRX) {
        return TL_TPLL_ERROR_INVALID_PARAM;
    }

    if (mode == TL_TPLL_MODE_PTX) {
        tl_tpll_config.mode = TL_TPLL_MODE_PTX;
    } else {
        tl_tpll_config.mode = TL_TPLL_MODE_PRX;
    }

    return TL_TPLL_SUCCESS;
}

/**
 * @brief   Bitrate function - Set radio bitrate for D25F communication.
 * @param   bitrate    Radio bitrate (TPLL_BITRATE_1MBPS / 2MBPS / 4MBPS / 6MBPS / 500kBPS / 250KBPS).
 * @retval  TL_TPLL_SUCCESS   If bitrate was set successfully.
 */
uint32_t n22_set_bitrate(TPLL_BitrateTypeDef bitrate)
{
//    return tl_tpll_set_bitrate(bitrate);
    tl_tpll_config.bitrate = bitrate;
}

/**
 * @brief   Retransmit parameters - Set retransmit count and delay for D25F communication.
 * @param   count      Number of retransmission attempts (0 disables retransmission).
 * @param   delay_us   Delay between retransmissions in microseconds.
 * @retval  TL_TPLL_SUCCESS   If retransmit parameters were set successfully.
 */
uint32_t n22_set_retransmit_para(uint16_t count, uint16_t delay_us)
{
    tl_tpll_config.retransmit_delay = delay_us;
    tl_tpll_config.retransmit_count = count;

    return TL_TPLL_SUCCESS;
}

/**
 * @brief   Payload function - Write TX payload for D25F communication.
 * @param   pipe    Pipe to transmit on (0 ~ TPLL_PIPE_NUM-1).
 * @param   data    Pointer to payload data buffer.
 * @param   len     Payload length in bytes (1 ~ TL_TPLL_MAX_PAYLOAD_LENGTH).
 * @retval  TL_TPLL_SUCCESS              If payload was written successfully.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM  If parameters were invalid.
 */
uint32_t n22_write_payload(uint8_t pipe,uint8_t ack, uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0 || len > TL_TPLL_MAX_PAYLOAD_LENGTH || pipe >= TPLL_PIPE_NUM) {
        return TL_TPLL_ERROR_INVALID_PARAM;
    }

    tx_payload.pipe   = pipe;
    tx_payload.length = len;
    tx_payload.noack  = ack;
    TPLL_EnableNoAck(ack);
//    for (uint8_t i = 0; i < len; i++) {
//      tx_payload.data[i] = data[i];
//    }
    smemcpy(&tx_payload.data[0],data,len);

    return TL_TPLL_SUCCESS;
}

/**
 * @brief   Access code length setting function - Set access code (address) length for D25F communication.
 * @param   length    Access code length in bytes (3 ~ 5).
 * @retval  TL_TPLL_SUCCESS              If length was set successfully.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM  If length was invalid.
 * @retval  TL_TPLL_ERROR_BUSY           If radio is busy.
 * @note    Must be called before n22_set_access_code().
 */
uint32_t n22_set_access_code_length(uint8_t length)
{
    uint32_t error = 0;
     if(TL_TPLL_ADDRESS_WIDTH_3BYTES == length ||
        TL_TPLL_ADDRESS_WIDTH_4BYTES == length ||
        TL_TPLL_ADDRESS_WIDTH_5BYTES == length)
    {
            error = tl_tpll_set_address_length(length);
            return error;
    }

    return TL_TPLL_ERROR_INVALID_PARAM;
}



/**
 * @brief   Access code length setting function - Set access code (address) length for D25F communication.
 * @param   length    Access code length in bytes (3 ~ 5).
 * @retval  TL_TPLL_SUCCESS              If length was set successfully.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM  If length was invalid.
 * @retval  TL_TPLL_ERROR_BUSY           If radio is busy.
 * @note    Must be called before n22_set_access_code().
 */
uint32_t n22_set_rf_channel(uint8_t chn)
{
    uint32_t err_code = tl_tpll_set_rf_channel(chn);
    VERIFY_SUCCESS(err_code);
    return err_code;
}



/**
 * @brief   Set access code function - Set access code for a specific pipe for D25F communication.
 * @param   pipe    Pipe to set access code for (0 ~ TPLL_PIPE_NUM-1).
 * @param   addr    Pointer to access code byte array (length must match n22_set_access_code_length()).
 * @retval  TL_TPLL_SUCCESS              If access code was set successfully.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM  If parameters were invalid.
 * @note    Access code length must be set first via n22_set_access_code_length().
 */
uint32_t n22_set_access_code(uint8_t pipe, uint8_t *addr)
{
    if (addr == NULL || pipe >= TPLL_PIPE_NUM) {
        return TL_TPLL_ERROR_INVALID_PARAM;
    }

    if(pipe==0)
        tl_tpll_set_base_address_0(addr);
    else if(pipe==1)
        tl_tpll_set_base_address_1(addr);

    return TL_TPLL_SUCCESS;
}


/**
 * @brief   Set access code function - Set access code for a specific pipe for D25F communication.
 * @param   pipe    Pipe to set access code for (0 ~ TPLL_PIPE_NUM-1).
 * @param   addr    Pointer to access code byte array (length must match n22_set_access_code_length()).
 * @retval  TL_TPLL_SUCCESS              If access code was set successfully.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM  If parameters were invalid.
 * @note    Access code length must be set first via n22_set_access_code_length().
 */
uint32_t n22_set_prefixes(uint8_t *p_prefixes, uint8_t num_pipes)
{
    uint32_t err_code;
    err_code = tl_tpll_set_prefixes(p_prefixes, num_pipes);
    VERIFY_SUCCESS(err_code);
    return err_code;
}



/**
 * @brief   Enable pipes function - Enable one or all pipes for D25F communication.
 * @param   enable_mask   Bitfield mask (0x01~0x80 for single pipe, 0xFF for all).
 * @retval  TL_TPLL_SUCCESS              If pipes were enabled successfully.
 * @retval  TL_TPLL_ERROR_INVALID_PARAM  If enable_mask was invalid.
 * @retval  TL_TPLL_ERROR_BUSY           If radio is busy.
 */
uint32_t n22_enable_pipes(uint8_t enable_mask)
{
    return tl_tpll_enable_pipes(enable_mask);
}


/**
 * @brief   Apply config function - Call tl_tpll_init with current tl_tpll_config.
 *          Should be called after all n22_set_* functions to apply settings to hardware.
 * @retval  TL_TPLL_SUCCESS              If config was applied successfully.
 * @retval  TL_TPLL_ERROR_NULL           If tl_tpll_config was invalid (should not happen).
 */
uint32_t n22_apply_config(void)
{
    return tl_tpll_init(&tl_tpll_config);
}


/**
 * @brief   Transmit function - Start transmission for D25F communication.
 * @retval  TL_TPLL_SUCCESS              If TX started successfully.
 * @retval  TL_TPLL_ERROR_BUFFER_EMPTY   If TX FIFO is empty.
 * @retval  TL_TPLL_ERROR_BUSY           If radio is busy.
 */
uint32_t n22_start_tx(void)
{
      return tl_tpll_write_payload(&tx_payload);
}

/**
 * @brief   Transmit function - Start transmission for D25F communication.
 * @retval  TL_TPLL_SUCCESS              If TX started successfully.
 * @retval  TL_TPLL_ERROR_BUFFER_EMPTY   If TX FIFO is empty.
 * @retval  TL_TPLL_ERROR_BUSY           If radio is busy.
 */
uint32_t n22_start_rx(void)
{
      return tl_tpll_start_rx();
}



/**
 * @brief   TX status report function - Send TX status to D25F via shared memory.
 *          Packet format: HCI_TYPE_CMD + N22_2P4G_CMD_START_TX_STA + tx_status
 * @param   tx_status   TX status value to report (0=success, others=error).
 */volatile unsigned int debug_shm[16] = {0};
void n22_send_tx_status(uint8_t tx_status)
{
    u8 rsp[4];
    rsp[0] = HCI_TYPE_CMD;
    rsp[1] = N22_2P4G_CMD_START_TX_STA;
    rsp[2] = 0x01;
    rsp[3] = tx_status;
    debug_shm[0]++;
    debug_shm[1] = mcc_n22_pri_send_msg(rsp, 4);
}

/**
 * @brief   RX status report function - Send ReadRxPayload to D25F via shared memory.
 *          Packet format: HCI_TYPE_CMD + N22_2P4G_CMD_START_RX_STA + sizeof(ReadRxPayload) + ReadRxPayload
 * @param   p_rx_payload    Pointer to ReadRxPayload structure.
 */
void n22_send_rx_sta(tl_tpll_payload_t *p_rx_payload)
{
    u8 *p_payload = (u8 *)p_rx_payload;
    u8 param_len = sizeof(tl_tpll_payload_t);
    u8 send_len = N22_2P4G_CMD_HEADER_LEN + param_len;
    u8 buf[N22_2P4G_CMD_HEADER_LEN + sizeof(tl_tpll_payload_t)];

    buf[0] = HCI_TYPE_CMD;
    buf[1] = N22_2P4G_CMD_START_RX_STA;
    buf[2] = param_len;
    smemcpy(&buf[3], p_payload, param_len);

    mcc_n22_pri_send_msg(buf, send_len);
}
