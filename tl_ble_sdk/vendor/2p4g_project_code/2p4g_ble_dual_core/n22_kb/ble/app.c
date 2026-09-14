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
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"

#include "app_buffer.h"
#include "hci_transport/hci_tr.h"
#include "hci_transport/hci_dfu.h"
#include "bqb_config.h"

#define MY_APP_ADV_CHANNEL  BLT_ENABLE_ADV_ALL
#define MY_ADV_INTERVAL_MIN ADV_INTERVAL_30MS
#define MY_ADV_INTERVAL_MAX ADV_INTERVAL_40MS

#include "stack/ble/hci/hci_vendor.h"

#include "stack/multicore_comm/service/mcc.h"
#include "stack/multicore_comm/service/service_n22.h"

#define N22_MAILBOX_TEST            0


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
//            gpio_write(GPIO_LED_GREEN, pCmd[0]);
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
    TL_UNUSED(h);

    mcc_n22_hci_send_msg(para, n);
    return 0;
}

unsigned char sKey[16] __attribute__((aligned(4)))      = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
unsigned char plaintext[16] __attribute__((aligned(4))) = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
unsigned char ciphertext[16] __attribute__((aligned(4)))   = {0};
unsigned char cipher[16] __attribute__((aligned(4))) = {0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a};


/**
 * @brief      use initialization
 * @param[in]  none.
 * @return     none.
 */
void app_ble_init_normal(void)
{

  /* clock sync && other configuration address sync */
  #if (HCI_INTERFACE == HCI_SHAREMEMORY)
      /* register ble h2c send callback */
      extern void blc_hci_handler_n22(u8 *p, unsigned short int n);
      mcc_n22_register_shm_recv_cb(TLK_SHM_MSG_HCI, (shm_recv_cb_t)blc_hci_handler_n22);
      /* register ble c2h send callback */
      extern int n22_blc_hci_send_data(u32 h, u8 *para, int n);
      blc_hci_registerControllerEventHandler(n22_blc_hci_send_data);
  #endif


    //////////////////////////// basic hardware Initialization  Begin //////////////////////////////////
    /* random number generator must be initiated here( in the beginning of user_init_normal).
     * When deepSleep retention wakeUp, no need initialize again */
    random_generator_init();

#if (TLKAPI_DEBUG_ENABLE)
    tlkapi_debug_init();
    blc_debug_enableStackLog(STK_LOG_HCI|STK_LOG_BSTNOOP);
#endif

    //////////////////////////// basic hardware Initialization  End /////////////////////////////////


    //////////////////////////// BLE stack Initialization  Begin //////////////////////////////////

    u8 mac_public[6] = {0};
    /* tl322x: Generate a default address */    //////////// Controller Initialization  Begin /////////////////////////
    blc_ll_initBasicMCU();

    blc_ll_initStandby_module(mac_public); //mandatory

    blc_ll_initLegacyAdvertising_module();

    #if APP_LE_LEGACY_SCAN_EN
    blc_ll_initLegacyScanning_module();
    #endif

    #if APP_LE_LEGACY_INIT_EN
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
    /* ACL Peripheral TX FIFO */
    blc_ll_initAclPeriphrTxFifo(app_acl_per_tx_fifo, ACL_PERIPHR_TX_FIFO_SIZE, ACL_PERIPHR_TX_FIFO_NUM, ACL_PERIPHR_MAX_NUM);

    #if (APP_WORKAROUND_TX_FIFO_4K_LIMITATION_EN && (ACL_CENTRAL_MAX_TX_OCTETS > 230 || ACL_PERIPHR_MAX_TX_OCTETS > 230))
        /* extend TX FIFO size for MAX_TX_OCTETS > 230 if user want use 16 or 32 FIFO */
        blc_ll_initAclConnCacheTxFifo(app_acl_cache_Txfifo, 260, 32);
    #endif

    blc_ll_setMaxConnectionNumber(ACL_CENTRAL_MAX_NUM, ACL_PERIPHR_MAX_NUM);
    blc_ll_setAclCentralBaseConnectionInterval(CONN_INTERVAL_10MS);
    blc_ll_setCreateConnectionTimeout(50000); //[!!!important]

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
    blc_ll_setAdvIntervalCheckEnable(0);

    /* HCI Data && Event */
    blc_hci_registerControllerDataHandler(blc_hci_sendACLData2Host);
    //blc_hci_registerControllerEventHandler(app_hci_send_data); //controller hci event to host all processed in this func
//    void blc_hci_registerControllerEventHandler_V1(hci_event_handler_t handler);
//    blc_hci_registerControllerEventHandler_V1(app_hci_send_data);
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
        chn_sound_capabilities_t appCsLocalSupportCap = {
            .Num_Config_Supported                   = 1,//range 1-4
            .max_consecutive_procedures_supported   = 0,
            .Num_Antennas_Supported                 = NUM_ANT_SUPPORT,
            .Max_Antenna_Paths_Supported            = MAX_ANT_PATHS_SUPPORT,

            .Roles_Supported                        = CS_INIT_REFL_ROLE,
            .Mode_Types                             = 0,  //mandatory mode1 and mode 2
            .RTT_Capability                         = 0,//150ns
            .RTT_AA_Only_N                          = 240,//todo
            .RTT_Sounding_N                         = 240,
            .RTT_Random_Payload_N                   = 240,
            .Optional_NADM_Sounding_Capability = 0,
            .Optional_NADM_Random_Capability =0,
            .Optional_CS_SYNC_PHYs_Supported = 0,//just mandatory 1M PHY
            .Optional_Subfeatures_Supported = 0,
            .Optional_T_IP1_Times_Supported = 0,//only support 145us
            .Optional_T_IP2_Times_Supported = 0,//only support 145us
            .Optional_T_FCS_Times_Supported = 0,//only support 150us
            .Optional_T_PM_Times_Supported  = CS_T_PM_20US,
            .T_SW_Time_Supported = 10,//10us
            .Optional_TX_SNR_Capability = 0xff,
        };
        blc_ll_initCsConfigParam(app_CsConfigParam, APP_CS_CONFIG_NUM);
        blc_ll_initCsRxFifo(app_cs_rx_buf, CS_RX_FIFO_SIZE, CS_RX_FIFO_NUM);

        //Set cs use tx power level
        blc_cs_set_tx_power_level(CS_USE_TX_POWER_LEVEL);

        blc_ll_initCsInitiatorModule(&appCsLocalSupportCap);

        blc_ll_initCsReflectorModule(&appCsLocalSupportCap);
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
        // write_log32(0x88880000 | error_code);
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





 //  blt_hci_vendor_setFuVendorCallback(app_vendor_cb);

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



    tlkapi_printf(APP_LOG_EN,"[APP][INI] n22 ble controller init build time: %s %s\n", __DATE__, __TIME__);

    //check aes code
    //TL322X Since this mode is being used for the first time, there might be some abnormal situations.
    //Therefore, before powering on, we should conduct a check here.
//    aes_encrypt(sKey, plaintext, ciphertext); //encrypt
//    for (unsigned char i = 0; i < 16; i++) {
//        if (ciphertext[i] != cipher[i])       //check cipher result
//        {
//            tlkapi_printf(APP_LOG_EN,"[APP][INI] aes fail 1\n");
//            while (1);
//        }
//    }
//    tlkapi_printf(APP_LOG_EN,"[APP][INI] aes sucess\n");
}

void app_ble_init_deepRetn(void)
{
#if (PM_DEEPSLEEP_RETENTION_ENABLE)
    blc_app_loadCustomizedParameters_deepRetn();

    blc_ll_initBasicMCU(); //mandatory

    blc_ll_recoverDeepRetention();

    DBG_CHN0_HIGH;
    irq_enable();
#endif
}


#if (TIFS_VARIATION_WORKAROUND_MLP_CODE_IN_RAM)
_attribute_ram_code_
#endif
void app_ble_main_loop(void)
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

    blc_sdk_main_loop();

    #if (TLKAPI_DEBUG_ENABLE)
        tlkapi_debug_handler();
    #endif
    #if    UI_LED_ENABLE
    static volatile unsigned int tickNow = 0;
    if (clock_time_exceed(tickNow, 100*1000)) {
        tickNow = clock_time();
//         gpio_toggle(GPIO_LED_WHITE);
    }
    #endif

}
