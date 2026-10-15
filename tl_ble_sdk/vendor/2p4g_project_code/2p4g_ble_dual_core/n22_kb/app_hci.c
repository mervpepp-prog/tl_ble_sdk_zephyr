/********************************************************************************************************
 * @file    app_hci.c
 *
 * @brief   App-side HCI implementation (n22_kb dual-core projects, shared by
 *          2p4g_ble_n22_kb/zephyr/nvm).
 *          App-side HCI implementation (b85m app_hci style):
 *          - the command handlers are implemented in stack/ble/hci/hci_cmd.c
 *          - the two-level command tables below are owned by this module and
 *            can be trimmed/extended via the feature switches in app_config.h
 *          - the table-driven dispatch (blc_hci_dispatch) and the host packet
 *            entries (blc_hci_handler/blc_hci_handler_n22) are part of this
 *            module, so the application owns its HCI command set
 *          - the OGF(0x3F) vendor specific commands are delegated to the
 *            callback registered by
 *            blc_hci_registerControllerVendorCmdProcess_Callback()
 *
 * @author  BLE GROUP
 * @date    2026.10
 *
 * @par     Copyright (c) 2022, Telink Semiconductor (Shanghai) Co., Ltd.
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
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/controller/ble_controller.h"
#include "static_assert.h"
#include "app_cmd_table.h"
#include "app_hci.h"

_attribute_ble_data_retention_ hci_vendor_process_callback_t   blc_vendor_hci_cmd_handler     = NULL;

_attribute_ble_data_retention_ blc_hci_user_handler_t blc_hci_user_handler = 0;

void blc_hci_register_user_handler(void *usrHandler)
{
    blc_hci_user_handler = usrHandler;
}

void blc_hci_registerControllerVendorCmdProcess_Callback(hci_vendor_process_callback_t handler)
{
  blc_vendor_hci_cmd_handler = handler;
}

//////////////////////////////////////////////////////////////////////////////////
//  Two-level table driven HCI command dispatch
//      level 1: OGF group table (app_ogf_table) maps the OGF field of the
//               command opcode to one OCF sub-table
//      level 2: OCF sub-table maps the full 2-byte opcode to its handler
//      The tables below are owned by this application (b85m app_hci style):
//      trim/extend them via the feature switches of app_config.h.
//      The command handlers are implemented in stack/ble/hci/hci_cmd.c.
//////////////////////////////////////////////////////////////////////////////////

STATIC_ASSERT(HCI_OPCODE_VALUE(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_EVENT_MASK) == ((HCI_CMD_LE_OPCODE_OGF << 8) | HCI_CMD_LE_SET_EVENT_MASK));
STATIC_ASSERT(HCI_OPCODE_VALUE(HCI_OGF_VENDOR, 0) == (HCI_CMD_VENDOR_OPCODE_OGF << 8));
#if (APP_LE_HDT_EN)
STATIC_ASSERT(HCI_OPCODE_VALUE(HCI_OGF_HIGHER_DATA_THROUGHPUT, HCI_CMD_LE_SET_CIG_PARAMETERS_V3) == ((0xFB << 8) | HCI_CMD_LE_SET_CIG_PARAMETERS_V3));
#endif

#if (HCI_CMD_LOG_EN)
#define HCI_CMD_ENTRY(ogf, ocf, handler, name)  { HCI_OPCODE_VALUE(ogf, ocf), handler, name },
#else
#define HCI_CMD_ENTRY(ogf, ocf, handler, name)  { HCI_OPCODE_VALUE(ogf, ocf), handler },
#endif

// OGF(0x02): Link Policy (no command supported, all reply unknown command)
static const hci_ocf_cmd_t app_link_policy_ocf_table[1] = {0};

// OGF(0x06): Testing (no command supported, all reply unknown command)
static const hci_ocf_cmd_t app_testing_ocf_table[1] = {0};

//////////////////////////////////////////////////////////////////////////////////
//  OCF sub-tables (one entry per command)
//  NOTE: every entry is gated by its APP_LE_* switch only (b85m style): the
//  switches are defined in app_config.h of this project and trim/extend the
//  HCI command set of this application.
//////////////////////////////////////////////////////////////////////////////////

// OGF(0x01): Link Control
const hci_ocf_cmd_t app_link_ctrl_ocf_table[] = {
    HCI_CMD_ENTRY(HCI_OGF_LINK_CONTROL, HCI_CMD_INQUIRY, hci_cmd_handle_inquiry, "Inquiry")
    HCI_CMD_ENTRY(HCI_OGF_LINK_CONTROL, HCI_CMD_DISCONNECT, hci_cmd_handle_disconnect, "Disconnect")
    HCI_CMD_ENTRY(HCI_OGF_LINK_CONTROL, HCI_CMD_READ_REMOTE_NAME_REQ, hci_cmd_handle_read_remote_name_req, "Read Remote Name Request")
    HCI_CMD_ENTRY(HCI_OGF_LINK_CONTROL, HCI_CMD_READ_REMOTE_VER_INFO, hci_cmd_handle_read_remote_ver_info, "Read Remote Version Information")
};

// OGF(0x03): Controller & Baseband
const hci_ocf_cmd_t app_cbc_ocf_table[] = {
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_SET_EVENT_MASK, hci_cmd_handle_set_event_mask, "Set Event Mask")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_RESET, hci_cmd_handle_reset, "Reset")
#if !BQB_TEST_EN
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_LOCAL_NAME, hci_cmd_handle_write_local_name, "Write Local Name")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_READ_LOCAL_NAME, hci_cmd_handle_read_local_name, "Read Local Name")
#endif
#if (APP_HCI_FLOW_CTRL_EN)
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_SET_CONTROLLER_TO_HOST_FLOW_CTRL, hci_cmd_handle_set_ctrl_to_host_flow_ctrl, "Set Controller To Host Flow Control")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_HOST_BUF_SIZE, hci_cmd_handle_host_buf_size, "Host Buffer Size")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_HOST_NUM_OF_COMPLETE_PACKETS, hci_cmd_handle_host_num_of_complete_packets, "Host Number Of Completed Packets")
#endif
#if (APP_LE_PING_AUTH_PAYLOAD_TIMEOUT_EN)
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_READ_AUTH_PAYLOAD_TIMEOUT, hci_cmd_handle_read_auth_payload_timeout, "Read Authenticated Payload Timeout")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_AUTH_PAYLOAD_TIMEOUT, hci_cmd_handle_write_auth_payload_timeout, "Write Authenticated Payload Timeout")
#endif
#if (APP_LE_CHANNEL_CLASSIFICATION_EN)
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_READ_AFH_CHN_ASSESSMENT_MODE, hci_cmd_handle_read_afh_chn_assessment_mode, "Read AFH Channel Assessment Mode")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_AFH_CHN_ASSESSMENT_MODE, hci_cmd_handle_write_afh_chn_assessment_mode, "Write AFH Channel Assessment Mode")
#endif
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_SET_EVT_MASK_PAGE_2, hci_cmd_handle_set_evt_mask_page_2, "Set Event Mask Page 2")

    //  BR/EDR only commands without BLE implementation, shared stub handler
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_SET_EVENT_FILTER, hci_cmd_handle_br_edr_stub, "Set Event Filter")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_PIN_TYPE, hci_cmd_handle_br_edr_stub, "Write Pin Type")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_CREATE_NEW_UINT_KEY, hci_cmd_handle_br_edr_stub, "Create New Unit Key")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_DELETE_STORED_LINK_KEY, hci_cmd_handle_br_edr_stub, "Delete Stored Link Key")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_CONNECTION_ACCEPT_TIMEOUT, hci_cmd_handle_br_edr_stub, "Write Connection Accept Timeout")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_PAGE_TIMEOUT, hci_cmd_handle_br_edr_stub, "Write Page Timeout")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_SCAN_ENABLE, hci_cmd_handle_br_edr_stub, "Write Scan Enable")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_PAGE_SCAN_ACTIVITY, hci_cmd_handle_br_edr_stub, "Write Page Scan Activity")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_INQUIRY_SCAN_ACTIVITY, hci_cmd_handle_br_edr_stub, "Write Inquiry Scan Activity")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_AUTHENTICATION_ENABLE, hci_cmd_handle_br_edr_stub, "Write Authentication Enable")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_CLASS_OF_DEVICE, hci_cmd_handle_br_edr_stub, "Write Class Of Device")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_VOICE_SETTING, hci_cmd_handle_br_edr_stub, "Write Voice Setting")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_NUM_BROADCAST_RETRANSMISSIONS, hci_cmd_handle_br_edr_stub, "Write Number Of Broadcast Retransmissions")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_HOLD_MODE_ACTIVITY, hci_cmd_handle_br_edr_stub, "Write Hold Mode Activity")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_SYNCHRONOUS_FLOW_CONTROL_ENABLE, hci_cmd_handle_br_edr_stub, "Write Synchronous Flow Control Enable")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_CURRENT_IAC_LAP, hci_cmd_handle_br_edr_stub, "Write Current IAC LAP")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_INQUIRY_SCAN_TYPE, hci_cmd_handle_br_edr_stub, "Write Inquiry Scan Type")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_INQUIRY_MODE, hci_cmd_handle_br_edr_stub, "Write Inquiry Mode")
    HCI_CMD_ENTRY(HCI_OGF_CONTROLLER_BASEBAND, HCI_CMD_WRITE_PAGE_SCAN_TYPE, hci_cmd_handle_br_edr_stub, "Write Page Scan Type")
};

// OGF(0x04): Informational Parameters
const hci_ocf_cmd_t app_ip_ocf_table[] = {
    HCI_CMD_ENTRY(HCI_OGF_INFORMATIONAL, HCI_CMD_READ_LOCAL_VER_INFO, hci_cmd_handle_read_local_ver_info, "Read Local Version Information")
    HCI_CMD_ENTRY(HCI_OGF_INFORMATIONAL, HCI_CMD_READ_LOCAL_SUPPORTED_CMDS, hci_cmd_handle_read_local_supported_cmds, "Read Local Supported Commands")
    HCI_CMD_ENTRY(HCI_OGF_INFORMATIONAL, HCI_CMD_READ_LOCAL_SUPPORTED_FEATURES, hci_cmd_handle_read_local_supported_features, "Read Local Supported Features")
#if !BQB_TEST_EN
    HCI_CMD_ENTRY(HCI_OGF_INFORMATIONAL, HCI_CMD_READ_EXTENDED_LOCAL_SUPPORTED_FEATURES, hci_cmd_handle_read_extended_local_supported_features, "Read Extended Local Supported Features")
    HCI_CMD_ENTRY(HCI_OGF_INFORMATIONAL, HCI_CMD_READ_BUFFER_SIZE_COMMAND, hci_cmd_handle_read_buffer_size_command, "Read Buffer Size")
#endif
    HCI_CMD_ENTRY(HCI_OGF_INFORMATIONAL, HCI_CMD_READ_BD_ADDR, hci_cmd_handle_read_bd_addr, "Read BD Addr")
};

// OGF(0x05): Status Parameters
#if (APP_LE_POWER_CONTROL_EN)
const hci_ocf_cmd_t app_sp_ocf_table[] = {
    HCI_CMD_ENTRY(HCI_OGF_STATUS_PARAMETERS, HCI_CMD_READ_RSSI, hci_cmd_handle_read_rssi, "Read RSSI")
};
#else
const hci_ocf_cmd_t app_sp_ocf_table[1] = {0};
#endif

// OGF(0x08): LE Controller
const hci_ocf_cmd_t app_le_ocf_table[] = {
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_EVENT_MASK, hci_cmd_handle_le_set_event_mask, "LE Set Event Mask")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_BUF_SIZE, hci_cmd_handle_le_read_buf_size, "LE Read Buffer Size")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_LOCAL_SUPPORTED_FEATURES, hci_cmd_handle_le_read_local_supported_features, "LE Read Local Supported Features")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_RANDOM_ADDR, hci_cmd_handle_le_set_random_addr, "LE Set Random Address")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_ADVERTISE_PARAMETERS, hci_cmd_handle_le_set_advertise_parameters, "LE Set Advertising Parameters")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_ADVERTISING_CHANNEL_TX_POWER, hci_cmd_handle_le_read_advertising_channel_tx_power, "LE Read Advertising Channel Tx Power")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_ADVERTISE_DATA, hci_cmd_handle_le_set_advertise_data, "LE Set Advertising Data")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_SCAN_RSP_DATA, hci_cmd_handle_le_set_scan_rsp_data, "LE Set Scan Response Data")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_ADVERTISE_ENABLE, hci_cmd_handle_le_set_advertise_enable, "LE Set Advertising Enable")
#if (APP_LE_LEGACY_SCANNING_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_SCAN_PARAMETERS, hci_cmd_handle_le_set_scan_parameters, "LE Set Scan Parameters")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_SCAN_ENABLE, hci_cmd_handle_le_set_scan_enable, "LE Set Scan Enable")
#if (APP_LE_CREATE_CONN_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CREATE_CONNECTION, hci_cmd_handle_le_create_connection, "LE Create Connection")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CREATE_CONNECTION_CANCEL, hci_cmd_handle_le_create_connection_cancel, "LE Create Connection Cancel")
#endif
#endif
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_WHITE_LIST_SIZE, hci_cmd_handle_le_read_white_list_size, "LE Read White List Size")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CLEAR_WHITE_LIST, hci_cmd_handle_le_clear_white_list, "LE Clear White List")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ADD_DEVICE_TO_WHITE_LIST, hci_cmd_handle_le_add_device_to_white_list, "LE Add Device To White List")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_REMOVE_DEVICE_FROM_WL, hci_cmd_handle_le_remove_device_from_wl, "LE Remove Device From White List")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CONNECTION_UPDATE, hci_cmd_handle_le_connection_update, "LE Connection Update")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_HOST_CHANNEL_CLASSIFICATION, hci_cmd_handle_le_set_host_channel_classification, "LE Set Host Channel Classification")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_CHANNEL_MAP, hci_cmd_handle_le_read_channel_map, "LE Read Channel Map")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_REMOTE_USED_FEATURES, hci_cmd_handle_le_read_remote_used_features, "LE Read Remote Used Features")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ENCRYPT, hci_cmd_handle_le_encrypt, "LE Encrypt")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_RANDOM, hci_cmd_handle_le_random, "LE Random")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_START_ENCRYPTION, hci_cmd_handle_le_start_encryption, "LE Start Encryption")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_LONG_TERM_KEY_REQUESTED_REPLY, hci_cmd_handle_le_long_term_key_requested_reply, "LE Long Term Key Requested Reply")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_LONG_TERM_KEY_REQUESTED_NEGATIVE_REPLY, hci_cmd_handle_le_long_term_key_requested_negative_reply, "LE Long Term Key Requested Negative Reply")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_SUPPORTED_STATES, hci_cmd_handle_le_read_supported_states, "LE Read Supported States")
#if (APP_LE_PHY_TEST_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_RECEIVER_TEST, hci_cmd_handle_le_receiver_test, "LE Receiver Test")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_TRANSMITTER_TEST, hci_cmd_handle_le_transmitter_test, "LE Transmitter Test")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_TEST_END, hci_cmd_handle_le_test_end, "LE Test End")
#endif
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_REMOTE_CONNECTION_PARAM_REQ_REPLY, hci_cmd_handle_empty, "LE Remote Connection Parameter Request Reply")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_REMOTE_CONNECTION_PARAM_REQ_NEGATIVE_REPLY, hci_cmd_handle_empty, "LE Remote Connection Parameter Request Negative Reply")
#if (APP_LE_DATA_LENGTH_EXTENSION_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_DATA_LENGTH, hci_cmd_handle_le_set_data_length, "LE Set Data Length")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_SUGGESTED_DEFAULT_DATA_LENGTH, hci_cmd_handle_le_read_suggested_default_data_length, "LE Read Suggested Default Data Length")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_WRITE_SUGGESTED_DEFAULT_DATA_LENGTH, hci_cmd_handle_le_write_suggested_default_data_length, "LE Write Suggested Default Data Length")
#endif
#if (APP_LE_P256KEY_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_LOCAL_P256_PUBLIC_KEY, hci_cmd_handle_le_read_local_p256_public_key, "LE Read Local P-256 Public Key")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_GENERATE_DHKEY, hci_cmd_handle_le_generate_dhkey, "LE Generate DHKey")
#endif
#if (APP_LE_PRIVACY_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ADD_DEVICE_TO_RESOLVING_LIST, hci_cmd_handle_le_add_device_to_resolving_list, "LE Add Device To Resolving List")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST, hci_cmd_handle_le_remove_device_from_resolving_list, "LE Remove Device From Resolving List")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CLEAR_RESOLVING_LIST, hci_cmd_handle_le_clear_resolving_list, "LE Clear Resolving List")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_RESOLVING_LIST_SIZE, hci_cmd_handle_le_read_resolving_list_size, "LE Read Resolving List Size")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_PEER_RESOLVABLE_ADDRESS, hci_cmd_handle_le_read_peer_resolvable_address, "LE Read Peer Resolvable Address")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_LOCAL_RESOLVABLE_ADDRESS, hci_cmd_handle_le_read_local_resolvable_address, "LE Read Local Resolvable Address")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_ADDRESS_RESOLUTION_ENABLE, hci_cmd_handle_le_set_address_resolution_enable, "LE Set Address Resolution Enable")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TIMEOUT, hci_cmd_handle_le_set_resolvable_private_address_timeout, "LE Set Resolvable Private Address Timeout")
#endif
#if (APP_LE_DATA_LENGTH_EXTENSION_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_MAX_DATA_LENGTH, hci_cmd_handle_le_read_max_data_length, "LE Read Maximum Data Length")
#endif
#if (APP_LE_2M_PHY_EN || APP_LE_CODED_PHY_EN || APP_LE_HDT_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_PHY, hci_cmd_handle_le_read_phy, "LE Read PHY")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_DEFAULT_PHY, hci_cmd_handle_le_set_default_phy, "LE Set Default PHY")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_PHY, hci_cmd_handle_le_set_phy, "LE Set PHY")
#endif
#if (APP_LE_PHY_TEST_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ENHANCED_RECEIVER_TEST, hci_cmd_handle_le_enhanced_receiver_test, "LE Enhanced Receiver Test")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ENHANCED_TRANSMITTER_TEST, hci_cmd_handle_le_enhanced_transmitter_test, "LE Enhanced Transmitter Test")
#endif
#if (APP_LE_EXTENDED_ADV_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_ADVERTISING_SET_RANDOM_ADDRESS, hci_cmd_handle_le_set_advertising_set_random_address, "LE Set Advertising Set Random Address")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_EXTENDED_ADVERTISING_PARAMETERS, hci_cmd_handle_le_set_extended_advertising_parameters, "LE Set Extended Advertising Parameters")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_EXTENDED_ADVERTISING_DATA, hci_cmd_handle_le_set_extended_advertising_data, "LE Set Extended Advertising Data")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_EXTENDED_SCAN_RESPONSE_DATA, hci_cmd_handle_le_set_extended_scan_response_data, "LE Set Extended Scan Response Data")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_EXTENDED_ADVERTISING_ENABLE, hci_cmd_handle_le_set_extended_advertising_enable, "LE Set Extended Advertising Enable")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_MAXIMUM_ADVERTISING_DATA_LENGTH, hci_cmd_handle_le_read_maximum_advertising_data_length, "LE Read Maximum Advertising Data Length")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_NUMBER_OF_SUPPORTED_ADVERTISING_SETS, hci_cmd_handle_le_read_number_of_supported_advertising_sets, "LE Read Number Of Supported Advertising Sets")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_REMOVE_ADVERTISING_SET, hci_cmd_handle_le_remove_advertising_set, "LE Remove Advertising Set")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CLEAR_ADVERTISING_SETS, hci_cmd_handle_le_clear_advertising_sets, "LE Clear Advertising Sets")
#endif
#if (APP_LE_PERIODIC_ADV_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_PERIODIC_ADVERTISING_PARAMETERS, hci_cmd_handle_le_set_periodic_advertising_parameters, "LE Set Periodic Advertising Parameters")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_PERIODIC_ADVERTISING_DATA, hci_cmd_handle_le_set_periodic_advertising_data, "LE Set Periodic Advertising Data")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_PERIODIC_ADVERTISING_ENABLE, hci_cmd_handle_le_set_periodic_advertising_enable, "LE Set Periodic Advertising Enable")
#endif
#if (APP_PAWR_ADV_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_PERIODIC_ADVERTISING_SUBEVENT_DATA, hci_cmd_handle_le_set_periodic_advertising_subevent_data, "LE Set Periodic Advertising Subevent Data")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_PERIODIC_ADVERTISING_PARAMETERS_V2, hci_cmd_handle_le_set_periodic_advertising_parameters_v2, "LE Set Periodic Advertising Parameters V2")
#endif
#if (APP_PAWR_SCAN_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_PERIODIC_ADVERTISING_RESPONSE_DATA, hci_cmd_handle_le_set_periodic_advertising_response_data, "LE Set Periodic Advertising Response Data")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_PERIODIC_SYNC_SUBEVENT, hci_cmd_handle_le_set_periodic_sync_subevent, "LE Set Periodic Sync Subevent")
#endif
#if (APP_LE_EXTENDED_SCAN_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_EXTENDED_SCAN_PARAMETERS, hci_cmd_handle_le_set_extended_scan_parameters, "LE Set Extended Scan Parameters")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_EXTENDED_SCAN_ENABLE, hci_cmd_handle_le_set_extended_scan_enable, "LE Set Extended Scan Enable")
#endif
#if (APP_LE_EXTENDED_INIT_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_EXTENDED_CREATE_CONNECTION, hci_cmd_handle_le_extended_create_connection, "LE Extended Create Connection")
#if (APP_PAWR_ADV_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_EXTENDED_CREATE_CONNECTION_V2, hci_cmd_handle_le_extended_create_connection_v2, "LE Extended Create Connection V2")
#endif
#endif
#if (APP_LE_PERIODIC_ADV_SYNC_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_PERIODIC_ADVERTISING_CREATE_SYNC, hci_cmd_handle_le_periodic_advertising_create_sync, "LE Periodic Advertising Create Sync")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_PERIODIC_ADVERTISING_CREATE_SYNC_CANCEL, hci_cmd_handle_le_periodic_advertising_create_sync_cancel, "LE Periodic Advertising Create Sync Cancel")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_PERIODIC_ADVERTISING_TERMINATE_SYNC, hci_cmd_handle_le_periodic_advertising_terminate_sync, "LE Periodic Advertising Terminate Sync")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ADD_DEVICE_TO_PERIODIC_ADVERTISER_LIST, hci_cmd_handle_le_add_device_to_periodic_advertiser_list, "LE Add Device To Periodic Advertiser List")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_REMOVE_DEVICE_FROM_PERIODIC_ADVERTISER_LIST, hci_cmd_handle_le_remove_device_from_periodic_advertiser_list, "LE Remove Device From Periodic Advertiser List")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CLEAR_PERIODIC_ADVERTISER_LIST, hci_cmd_handle_le_clear_periodic_advertiser_list, "LE Clear Periodic Advertiser List")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_PERIODIC_ADVERTISER_LIST_SIZE, hci_cmd_handle_le_read_periodic_advertiser_list_size, "LE Read Periodic Advertiser List Size")
#endif
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_TRANSMIT_POWER, hci_cmd_handle_le_read_transmit_power, "LE Read Transmit Power")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_RF_PATH_COMPENSATION, hci_cmd_handle_le_read_rf_path_compensation, "LE Read RF Path Compensation")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_WRITE_RF_PATH_COMPENSATION, hci_cmd_handle_le_write_rf_path_compensation, "LE Write RF Path Compensation")
#if (APP_LE_PRIVACY_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_PRIVACY_MODE, hci_cmd_handle_le_set_privacy_mode, "LE Set Privacy Mode")
#endif
#if (APP_LE_PHY_TEST_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_RECEIVER_TEST_V3, hci_cmd_handle_le_receiver_test_v3, "LE Receiver Test V3")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_TRANSMITTER_TEST_V3, hci_cmd_handle_empty, "LE Transmitter Test V3")
#endif
#if (APP_LE_AOA_AOD_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_CONNECTIONLESS_CTE_TRANSMIT_PARAMETERS, hci_cmd_handle_le_set_connectionless_cte_transmit_parameters, "LE Set Connectionless CTE Transmit Parameters")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_CONNECTIONLESS_CTE_TRANSMIT_ENABLE, hci_cmd_handle_le_set_connectionless_cte_transmit_enable, "LE Set Connectionless CTE Transmit Enable")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_CONNECTIONLESS_IQ_SAMPLING_ENABLE, hci_cmd_handle_le_set_connectionless_iq_sampling_enable, "LE Set Connectionless IQ Sampling Enable")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_CONNECTION_CTE_RECEIVE_PARAMETERS, hci_cmd_handle_le_set_connection_cte_receive_parameters, "LE Set Connection CTE Receive Parameters")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_CONNECTION_CTE_TRANSMIT_PARAMETERS, hci_cmd_handle_le_set_connection_cte_transmit_parameters, "LE Set Connection CTE Transmit Parameters")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CONNECTION_REQUEST_ENABLE, hci_cmd_handle_le_connection_request_enable, "LE Connection CTE Request Enable")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CONNECTION_RESPONSE_ENABLE, hci_cmd_handle_le_connection_response_enable, "LE Connection CTE Response Enable")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_ANTENNA_INFORMATION, hci_cmd_handle_le_read_antenna_information, "LE Read Antenna Information")
#endif
#if (APP_LE_PERIODIC_ADV_SYNC_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_PERIODIC_ADVERTISING_RECEIVE_ENABLE, hci_cmd_handle_le_set_periodic_advertising_receive_enable, "LE Set Periodic Advertising Receive Enable")
#endif
#if (APP_LE_PAST_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_PERIODIC_ADVERTISING_SYNC_TRANSFER, hci_cmd_handle_le_periodic_advertising_sync_transfer, "LE Periodic Advertising Sync Transfer")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_PERIODIC_ADVERTISING_SET_INFO_TRANSFER, hci_cmd_handle_le_periodic_advertising_set_info_transfer, "LE Periodic Advertising Set Info Transfer")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_PERIODIC_ADV_SYNC_TRANSFER_PARAMETERS, hci_cmd_handle_le_set_periodic_adv_sync_transfer_parameters, "LE Set Periodic Advertising Sync Transfer Parameters")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_DEFAULT_PERIODIC_ADV_SYNC_TRANSFER_PARAMS, hci_cmd_handle_le_set_default_periodic_adv_sync_transfer_params, "LE Set Default Periodic Advertising Sync Transfer Parameters")
#endif
#if (APP_LE_P256KEY_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_GENERATE_DHKEY_V2, hci_cmd_handle_le_generate_dhkey_v2, "LE Generate DHKey V2")
#endif
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_MODIFY_SLEEP_CLOCK_ACCURACY, hci_cmd_handle_empty, "LE Modify Sleep Clock Accuracy")
#if (APP_LE_ISO_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_BUFFER_SIZE_V2, hci_cmd_handle_le_read_buffer_size_v2, "LE Read Buffer Size V2")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_ISO_TX_SYNC, hci_cmd_handle_le_read_iso_tx_sync, "LE Read ISO TX Sync")
#endif
#if (APP_LE_CIS_CENTRAL_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_CIG_PARAMETERS, hci_cmd_handle_le_set_cig_parameters, "LE Set CIG Parameters")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_CIG_PARAMETERS_TEST, hci_cmd_handle_le_set_cig_parameters_test, "LE Set CIG Parameters Test")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CREATE_CIS, hci_cmd_handle_le_create_cis, "LE Create CIS")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_REMOVE_CIG, hci_cmd_handle_le_remove_cig, "LE Remove CIG")
#endif
#if (APP_LE_CIS_PERIPHERAL_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ACCEPT_CIS_REQUEST, hci_cmd_handle_le_accept_cis_request, "LE Accept CIS Request")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_REJECT_CIS_REQUEST, hci_cmd_handle_le_reject_cis_request, "LE Reject CIS Request")
#endif
#if (APP_LE_BIG_BROADCASTER_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CREATE_BIG, hci_cmd_handle_le_create_big, "LE Create BIG")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CREATE_BIG_TEST, hci_cmd_handle_le_create_big_test, "LE Create BIG Test")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_TERMINATE_BIG, hci_cmd_handle_le_terminate_big, "LE Terminate BIG")
#endif
#if (APP_LE_BIG_SYNC_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_BIG_CREATE_SYNC, hci_cmd_handle_le_big_create_sync, "LE BIG Create Sync")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_BIG_TERMINATE_SYNC, hci_cmd_handle_le_big_terminate_sync, "LE BIG Terminate Sync")
#endif
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_REQUEST_PEER_SCA, hci_cmd_handle_empty, "LE Request Peer SCA")
#if (APP_LE_ISO_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SETUP_ISO_DATA_PATH, hci_cmd_handle_le_setup_iso_data_path, "LE Setup ISO Data Path")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_REMOVE_ISO_DATA_PATH, hci_cmd_handle_le_remove_iso_data_path, "LE Remove ISO Data Path")
#endif
#if (APP_LE_ISO_TEST_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ISO_TRANSMIT_TEST, hci_cmd_handle_le_iso_transmit_test, "LE ISO Transmit Test")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ISO_RECEIVE_TEST, hci_cmd_handle_le_iso_receive_test, "LE ISO Receive Test")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ISO_READ_TEST_COUNTERS, hci_cmd_handle_le_iso_read_test_counters, "LE ISO Read Test Counters")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ISO_TEST_END, hci_cmd_handle_le_iso_test_end, "LE ISO Test End")
#endif
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_HOST_FEATURE, hci_cmd_handle_le_set_host_feature, "LE Set Host Feature")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_ISO_LINK_QUALITY, hci_cmd_handle_empty, "LE Read ISO Link Quality")
#if (APP_LE_POWER_CONTROL_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ENHANCED_READ_TRANSMIT_POWER_LEVEL, hci_cmd_handle_le_enhanced_read_transmit_power_level, "LE Enhanced Read Transmit Power Level")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_REMOTE_TRANSMIT_POWER_LEVEL, hci_cmd_handle_le_read_remote_transmit_power_level, "LE Read Remote Transmit Power Level")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_PATH_LOSS_REPORTING_PARAMETERS, hci_cmd_handle_le_set_path_loss_reporting_parameters, "LE Set Path Loss Reporting Parameters")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_PATH_LOSS_REPORTING_ENABLE, hci_cmd_handle_le_set_path_loss_reporting_enable, "LE Set Path Loss Reporting Enable")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_TRANSMIT_POWER_REPORTING_ENABLE, hci_cmd_handle_le_set_transmit_power_reporting_enable, "LE Set Transmit Power Reporting Enable")
#endif
#if (APP_LE_PRIVACY_EN) && (APP_LE_RPA_DATA_RELATED_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_DATA_RELATE_ADDRESS_CHANGES, hci_cmd_handle_le_set_data_relate_address_changes, "LE Set Data Related Address Changes")
#endif
#if (APP_LE_SUBRATE_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_DEFAULT_SUBRATE, hci_cmd_handle_le_set_default_subrate, "LE Set Default Subrate")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SUBRATE_REQUEST, hci_cmd_handle_le_subrate_request, "LE Subrate Request")
#endif
#if (APP_LE_ADV_CODING_SEL_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_EXTENDED_ADVERTISING_PARAMETERS_V2, hci_cmd_handle_le_set_extended_advertising_parameters_v2, "LE Set Extended Advertising Parameters V2")
#endif
#if (APP_LE_DBAF_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_DECISION_DATA, hci_cmd_handle_le_set_decision_data, "LE Set Decision Data")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SET_DECISION_INSTRUCTIONS, hci_cmd_handle_le_set_decision_instructions, "LE Set Decision Instructions")
#endif
#if (APP_LE_EXTENDED_FEATURE_SET_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_ALL_SUPPORTED_FEATURES, hci_cmd_handle_le_read_all_supported_features, "LE Read All Supported Features")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_ALL_REMOTE_FEATURES, hci_cmd_handle_le_read_all_remote_features, "LE Read All Remote Features")
#endif
#if (APP_LE_CHANNEL_SOUNDING_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_READ_LOCAL_SUPPORTED_CAPABILITIES, hci_cmd_handle_le_cs_read_local_supported_capabilities, "LE CS Read Local Supported Capabilities")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_READ_REMOTE_SUPPORTED_CAPABILITIES, hci_cmd_handle_le_cs_read_remote_supported_capabilities, "LE CS Read Remote Supported Capabilities")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_WRITE_REMOTE_SUPPORTED_CAPABILITIES, hci_cmd_handle_le_cs_write_remote_supported_capabilities, "LE CS Write Remote Supported Capabilities")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_SECURITY_ENABLE, hci_cmd_handle_le_cs_security_enable, "LE CS Security Enable")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_SET_DEFAULT_SETTINGS, hci_cmd_handle_le_cs_set_default_settings, "LE CS Set Default Settings")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_READ_REMOTE_FAE_TABLE, hci_cmd_handle_le_cs_read_remote_fae_table, "LE CS Read Remote FAE Table")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_WRITE_CACHE_REMOTE_FAE_TABLE, hci_cmd_handle_le_cs_write_cache_remote_fae_table, "LE CS Write Cache Remote FAE Table")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_CREATE_CONFIG, hci_cmd_handle_le_cs_create_config, "LE CS Create Config")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_REMOVE_CONFIG, hci_cmd_handle_le_cs_remove_config, "LE CS Remove Config")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_SET_Channel_CLASSIFICATION, hci_cmd_handle_le_cs_set_channel_classification, "LE CS Set Channel Classification")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_SET_PROCEDURE_PARAMETERS, hci_cmd_handle_le_cs_set_procedure_parameters, "LE CS Set Procedure Parameters")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_PROCEDURE_ENABLE, hci_cmd_handle_le_cs_procedure_enable, "LE CS Procedure Enable")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_TEST, hci_cmd_handle_le_cs_test, "LE CS Test")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CS_TEST_END, hci_cmd_handle_le_cs_test_end, "LE CS Test End")
#endif
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SCI_SET_HOST_FEATURE, hci_cmd_handle_le_sci_set_host_feature, "LE SCI Set Host Feature")
#if (APP_LE_MONITORING_ADVERTISERS_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ADD_DEVICE_TO_MONITORED_ADVERTISERS_LIST, hci_cmd_handle_le_add_device_to_monitored_advertisers_list, "LE Add Device To Monitored Advertisers List")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_REMOVE_DEVICE_FROM_MONITORED_ADVERTISERS_LIST, hci_cmd_handle_le_remove_device_from_monitored_advertisers_list, "LE Remove Device From Monitored Advertisers List")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_CLEAR_MONITORED_ADVERTISERS_LIST, hci_cmd_handle_le_clear_monitored_advertisers_list, "LE Clear Monitored Advertisers List")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_MONITORED_ADVERTISERS_LIST_SIZE, hci_cmd_handle_le_read_monitored_advertisers_list_size, "LE Read Monitored Advertisers List Size")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_ENABLE_MONITORING_ADVERTISERS, hci_cmd_handle_le_enable_monitoring_advertisers, "LE Enable Monitoring Advertisers")
#endif
#if (APP_LE_FRAME_SPACE_UPDATE_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_FRAME_SPACE_UPDATE, hci_cmd_handle_le_frame_space_update, "LE Frame Space Update")
#endif
#if (APP_LE_SCI_EN)
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SCI_CONNECTION_RATE_REQ, hci_cmd_handle_le_sci_connection_rate_req, "LE SCI Connection Rate Request")
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_SCI_SET_DEFAULT_RATE_PARAMETERS, hci_cmd_handle_le_sci_set_default_rate_parameters, "LE SCI Set Default Rate Parameters")
#endif
    HCI_CMD_ENTRY(HCI_OGF_LE_CONTROLLER, HCI_CMD_LE_READ_MIN_SUPPORT_CONN_INTERVAL, hci_cmd_handle_le_read_min_support_conn_interval, "LE Read Minimum Supported Connection Interval")
};

#if (APP_LE_HDT_EN)
// OGF(0x3E): Higher Data Throughput test
const hci_ocf_cmd_t app_hdt_ocf_table[] = {
    HCI_CMD_ENTRY(HCI_OGF_HIGHER_DATA_THROUGHPUT, HCI_CMD_LE_READ_MAXIMUM_DATA_LENGTH_V2, hci_cmd_handle_le_read_maximum_data_length_v2, "LE Read Maximum Data Length V2")
    HCI_CMD_ENTRY(HCI_OGF_HIGHER_DATA_THROUGHPUT, HCI_CMD_REFRESH_ENCRYPTION_KEY_V2, hci_cmd_handle_refresh_encryption_key_v2, "Refresh Encryption Key V2")
    HCI_CMD_ENTRY(HCI_OGF_HIGHER_DATA_THROUGHPUT, HCI_CMD_LE_START_ENCRYPTION_V2, hci_cmd_handle_le_start_encryption_v2, "LE Start Encryption V2")
    HCI_CMD_ENTRY(HCI_OGF_HIGHER_DATA_THROUGHPUT, HCI_CMD_HDT_TEST_END_V2, hci_cmd_handle_empty, "HDT Test End V2")
    HCI_CMD_ENTRY(HCI_OGF_HIGHER_DATA_THROUGHPUT, HCI_CMD_LE_TRANSMITTER_TEST_V5, hci_cmd_handle_le_transmitter_test_v5, "LE Transmitter Test V5")
    HCI_CMD_ENTRY(HCI_OGF_HIGHER_DATA_THROUGHPUT, HCI_CMD_LE_READ_HDT_LOCAL_SUPPORTED_CAPABILITIES, hci_cmd_handle_empty, "LE Read HDT Local Supported Capabilities")
    HCI_CMD_ENTRY(HCI_OGF_HIGHER_DATA_THROUGHPUT, HCI_CMD_LE_SET_HDT_PARAMETERS_TEST, hci_cmd_handle_le_set_hdt_parameters_test, "LE Set HDT Parameters Test")
    HCI_CMD_ENTRY(HCI_OGF_HIGHER_DATA_THROUGHPUT, HCI_CMD_LE_SET_HDT_DEFAULT_PARAMETERS, hci_cmd_handle_le_set_hdt_default_parameters, "LE Set HDT Default Parameters")
#if (APP_LE_BIG_BROADCASTER_EN)
    HCI_CMD_ENTRY(HCI_OGF_HIGHER_DATA_THROUGHPUT, HCI_CMD_LE_CREATE_BIG_TEST_V2, hci_cmd_handle_le_create_big_test_v2, "LE Create BIG Test V2")
    HCI_CMD_ENTRY(HCI_OGF_HIGHER_DATA_THROUGHPUT, HCI_CMD_LE_CREATE_BIG_V2, hci_cmd_handle_le_create_big_v2, "LE Create BIG V2")
#endif
#if (APP_LE_CIS_CENTRAL_EN)
    HCI_CMD_ENTRY(HCI_OGF_HIGHER_DATA_THROUGHPUT, HCI_CMD_LE_SET_CIG_PARAMETERS_TEST_V3, hci_cmd_handle_le_set_cig_parameters_test_v3, "LE Set CIG Parameters Test V3")
    HCI_CMD_ENTRY(HCI_OGF_HIGHER_DATA_THROUGHPUT, HCI_CMD_LE_SET_CIG_PARAMETERS_V3, hci_cmd_handle_le_set_cig_parameters_v3, "LE Set CIG Parameters V3")
#endif
};
#endif

//////////////////////////////////////////////////////////////////////////////////
//  OGF top-level table (level 1), owned by this application
//////////////////////////////////////////////////////////////////////////////////
const hci_ogf_group_t app_ogf_table[] = {
    { HCI_OGF_LINK_CONTROL,         app_link_ctrl_ocf_table, sizeof(app_link_ctrl_ocf_table) / sizeof(app_link_ctrl_ocf_table[0]) },
    { HCI_OGF_LINK_POLICY,          app_link_policy_ocf_table, 0 }, //no link policy command supported
    { HCI_OGF_CONTROLLER_BASEBAND,  app_cbc_ocf_table,        sizeof(app_cbc_ocf_table) / sizeof(app_cbc_ocf_table[0]) },
    { HCI_OGF_INFORMATIONAL,         app_ip_ocf_table,         sizeof(app_ip_ocf_table) / sizeof(app_ip_ocf_table[0]) },
    { HCI_OGF_STATUS_PARAMETERS,    app_sp_ocf_table,         sizeof(app_sp_ocf_table) / sizeof(app_sp_ocf_table[0]) },
    { HCI_OGF_TESTING,              app_testing_ocf_table,     0 },  //no testing command supported
    { HCI_OGF_LE_CONTROLLER,        app_le_ocf_table,          sizeof(app_le_ocf_table) / sizeof(app_le_ocf_table[0]) },
#if (APP_LE_HDT_EN)
    { HCI_OGF_HIGHER_DATA_THROUGHPUT, app_hdt_ocf_table,      sizeof(app_hdt_ocf_table) / sizeof(app_hdt_ocf_table[0]) },
#endif
};

#undef HCI_CMD_ENTRY

u8 hci_cmd_log_en = 0;

void blc_hci_setCmdLogEnable(u8 en)
{
    hci_cmd_log_en = en ? 1 : 0;
}

static void hci_cmd_reply_unknown_cmd(u8 ocf, u8 ogf)
{
    u8 para[8] = {0};
    u8 status  = (u8)HCI_ERR_UNKNOWN_HCI_CMD;

    if (hci_enableHciCmdCmplEvt_is_En()) {
        u8 resultLen = hci_cmdComplete_evt(1, ocf, ogf, 1, &status, para);
        blc_hci_send_event(HCI_FLAG_EVENT_BT_STD | HCI_EVT_CMD_COMPLETE, para, resultLen);
    } else {
        hci_cmdStatus_evt(1, ocf, ogf, status, para);
        blc_hci_send_event(HCI_FLAG_EVENT_BT_STD | HCI_EVT_CMD_STATUS, para, 4);
    }
}

#if (TIFS_VARIATION_WORKAROUND_MLP_CODE_IN_RAM)
_attribute_ble_ram_code_noinline_
#else
_attribute_no_inline_
#endif
static void blc_hci_cmd_table_dispatch(u8 *p)
{
    u16 opcode        = p[1] | (p[2] << 8);
    u8  ocf           = p[1];
    u8  ogf           = p[2];
    u8  ogf_field     = (u8)((opcode >> 10) & 0x3F);
    const hci_ogf_group_t *p_group = NULL;
    const hci_ocf_cmd_t   *p_entry = NULL;
    u8 i;

    //////////////////////////////////////////////////////////////////////////////////
    //  OGF(0x3F) vendor specific commands are always delegated to the callback
    //  registered by blc_hci_registerControllerVendorCmdProcess_Callback() first;
    //  without the callback reply like an unknown command (legacy behavior)
    //////////////////////////////////////////////////////////////////////////////////
    if (ogf_field == HCI_OGF_VENDOR) {
        if (blc_vendor_hci_cmd_handler) {
            u8 para[0xFF] = {0};
            u8 resultLen  = blc_vendor_hci_cmd_handler(p[3], p[2], p[1], p + 4, para);
            u32 header    = HCI_FLAG_EVENT_BT_STD | para[0];
            blc_hci_send_event(header, &para[1], resultLen);
            return;
        }

        hci_cmd_reply_unknown_cmd(ocf, ogf);
        return;
    }

    // 1. search the OGF top-level table
    for (i = 0; i < (u8)(sizeof(app_ogf_table) / sizeof(app_ogf_table[0])); i++) {
        if (app_ogf_table[i].ogf == ogf_field) {
            p_group = &app_ogf_table[i];
            break;
        }
    }

    // 2. search the OCF sub-table of the matched OGF group
    if (p_group) {
        for (i = 0; i < p_group->entry_cnt; i++) {
            if (p_group->ocf_sub_table[i].opcode == opcode) {
                p_entry = &p_group->ocf_sub_table[i];
                break;
            }
        }
    }

    if (p_entry) {
    #if HCI_CMD_LOG_EN
        // unified command log: opcode, command name and parameters
        if (hci_cmd_log_en) {
            tlkapi_printf((stkLog_mask & STK_LOG_HCI), "[HCI][CMD] opcode 0x%04X %s, para len %d\n", opcode, p_entry->cmd_name, p[3]);
            tlkapi_send_string_data((stkLog_mask & STK_LOG_HCI), "[HCI][CMD] params", p + 4, p[3]);
        }
    #endif

        // 3. invoke the handler; the handler replies its own event
        hci_cmd_evt_pending = 1;
        p_entry->handler(opcode, p + 4);

        // 4. if the handler did not reply (empty handler / feature disabled at
        //    run time), reply the unified error event
        if (hci_cmd_evt_pending) {
            hci_cmd_evt_pending = 0;
            my_dump_str_data(IUT_HCI_LOG_EN, "[HCI][CMD] Unprocessed Command", &ocf, 1);
            hci_cmd_reply_unknown_cmd(ocf, ogf);
        }
    } else if (p_group) {
        // known OGF group, unknown OCF
        my_dump_str_data(IUT_HCI_LOG_EN, "[HCI][CMD] Unsupported Command", &ocf, 1);
        hci_cmd_reply_unknown_cmd(ocf, ogf);
    } else {
        //  01 06 04 03 01 00 13: disconnect (handle 01 00; reason: 0x13)
        //  01 0d 04 02 01 00   : read remote version info
        //  01 01 0c   ... set event mask
        //  01 03 0c   ... reset hci
        //  01 01 10   ... read local version info
        //  01 03 10   ... read local supported feature
        //  01 05 10   ... read buffer size
        //  01 09 10   ... read BD address
        //  unknown OGF (legacy behavior): reply Command Complete with success
        u8 status = 0; //OK
        u32 ret   = HCI_EVT_CMD_COMPLETE_STATUS(1, ocf, ogf, status);
        blc_hci_send_event(HCI_FLAG_EVENT_BT_STD | HCI_EVT_CMD_COMPLETE, (u8 *)&ret, 4);
    }
}

int blc_hci_dispatch(u8 *p, int n)
{
    //tlkapi_send_string_data((stkLog_mask & STK_LOG_HCI), "hci rx",p, n);

    /* user handler call-back */
    if (blc_hci_user_handler) {
        if (blc_hci_user_handler(p, n)) {
            return 0;
        }
    }

    if (p[0] == HCI_TYPE_ACL_DATA) {
        // must make sure p is align 2 byte or the type cast will error
        blc_hci_receiveHostACLData((void *)(p + 1));

        return 1;
    }
    else if (p[0] == HCI_TYPE_ISO_DATA) {
    #if (APP_LE_ISO_EN)
        // must make sure p is align 2 byte or the type cast will error
        blc_hci_packIsoData((iso_data_packet_t *)(p + 1));
    #endif

        return 1;
    }
    else if (p[0] == HCI_TYPE_CMD) {
        blc_hci_cmd_table_dispatch(p);
    }

    return 0;
}

#if (TIFS_VARIATION_WORKAROUND_MLP_CODE_IN_RAM)
_attribute_ble_ram_code_noinline_
#else
_attribute_no_inline_
#endif
int blc_hci_handler(u8 *p, int n)
{
    return blc_hci_dispatch(p, n);
}

#if (TIFS_VARIATION_WORKAROUND_MLP_CODE_IN_RAM)
_attribute_ble_ram_code_noinline_
#else
_attribute_no_inline_
#endif
void blc_hci_handler_n22(u8 *p, unsigned short int n)
{
    blc_hci_dispatch(p, n);
}

#if HCI_SEND_NUM_OF_CMP_AFT_ACK
_attribute_ble_ram_code_noinline_
#endif
int blc_hci_send_data(u32 h, u8 *para, int n)
{
    u8 *p = NULL;
    if (((bltHci_txfifo.wptr - bltHci_txfifo.rptr) & 255) < bltHci_txfifo.num) {
        p = bltHci_txfifo.p + (bltHci_txfifo.wptr & bltHci_txfifo.mask) * bltHci_txfifo.size;
    }

    if (!p || n >= (int)bltHci_txfifo.size) {
#if (UPPER_TESTER_HCI_LOG_EN)
        if (h & HCI_FLAG_EVENT_BT_STD) {
            my_dump_str_data(UPPER_TESTER_HCI_LOG_EN, "controller TX FIFO overflow", 0, 0);
        }
#endif

        return -1;
    }

    int nl = n + 4;
    if (h & HCI_FLAG_EVENT_TLK_MODULE) {
        *p++ = nl;
        *p++ = nl >> 8;
        *p++ = 0xff;
        *p++ = n + 2;
        *p++ = h;
        *p++ = h >> 8;
        smemcpy(p, para, n);
        p += n;
    } else if (h & HCI_FLAG_EVENT_BT_STD) {
        //skip le adv active evt
        if ((h & 0xff) == HCI_EVT_LE_META &&
            (para[0] == HCI_SUB_EVT_LE_ADVERTISING_REPORT || para[0] == HCI_SUB_EVT_LE_EXTENDED_ADVERTISING_REPORT)) {
            p = NULL;

            if (((bltHci_txfifo.wptr - bltHci_txfifo.rptr) & 255) < bltHci_txfifo.num - HCI_ADV_REPORT_EVT_RSVD_FIFO) { //keep 3 fifo left for others evt
                p = bltHci_txfifo.p + (bltHci_txfifo.wptr & bltHci_txfifo.mask) * bltHci_txfifo.size;
            }
            if (!p) {
                return -1;
            }
        }
#if (0)
        // debug-yuexin
        if (h & 0xff == HCI_EVT_LE_META) {
            u8  subEvt_code = p[0];
            u16 handle      = p[1] | (p[2] << 8);

            if (subEvt_code == HCI_SUB_EVT_LE_CS_SUBEVENT_RESULT) {
                hci_le_csSubeventResultEvt_t *pCsSubevent = (hci_le_csSubeventResultEvt_t *)p;
                /* print subevent data to txt file to calculate distance with other company */
                u8 tempBuff[258];
                tempBuff[0] = 0x04; //type
                tempBuff[1] = 0x3E; //event_code
                tempBuff[2] = n;    // total_len
                smemcpy(tempBuff + 3, &pCsSubevent->Subevent_Code, n);
                tlkapi_printf(1, "subevent len***:%d\r\n", n);
                tlkapi_send_string_data(1, "cs subevent data", tempBuff, n + 3);
                debugwait();
            }
            //------HCI LE event: LE CS Subevent Result Continue event-------------------------------
            else if (subEvt_code == HCI_SUB_EVT_LE_CS_SUBEVENT_RESULT_CONTINUE) {
                hci_le_csSubeventResultContinueEvt_t *pCsSubevent = (hci_le_csSubeventResultContinueEvt_t *)p;
                /* print subevent continue data to txt file to calculate distance with other company */
                u8 tempBuff[258];
                tempBuff[0] = 0x04;
                tempBuff[1] = 0x3E; //event_code
                tempBuff[2] = n;    // total_len
                smemcpy(tempBuff + 3, &pCsSubevent->Subevent_Code, n);
                tlkapi_printf(1, "continue subevent len***:%d\r\n", n);
                tlkapi_send_string_data((stkLog_mask & STK_LOG_HCI), "cs continue subevent data", tempBuff, n + 3);
                debugwait();
            }
        }
#endif
#if HCI_TX_FIFO_OPTIMIZE_EN
        if (!(para[0] == HCI_SUB_EVT_LE_ADVERTISING_REPORT || para[0] == HCI_SUB_EVT_LE_EXTENDED_ADVERTISING_REPORT)) {
            u8 w = bltHci_txfifo.wptr - 1;

            if ((u8)(bltHci_txfifo.wptr - bltHci_txfifo.rptr) >= 3) {
                while (w != bltHci_txfifo.rptr) //H5 slide win = 1
                {
                    u8 *pBuf = bltHci_txfifo.p + (w & bltHci_txfifo.mask) * bltHci_txfifo.size;
                    if (pBuf[2] == HCI_TYPE_ACL_DATA ||
                        (pBuf[2] == HCI_TYPE_EVENT && (pBuf[3] != 0x3E || (pBuf[3] == 0x3E && pBuf[5] != 0x02 && pBuf[5] != 0x0D)))) {
                        break;
                    }
                    w--;
                }

                if (w != (u8)(bltHci_txfifo.wptr - 1)) {
                    w++;
                    p = bltHci_txfifo.p + (w & bltHci_txfifo.mask) * bltHci_txfifo.size;

                    *p++ = n + 3;          //3: 1(HCI Type) + 1(Event Code) + 1(Parameter Total Length)
                    *p++ = 0x0;
                    *p++ = HCI_TYPE_EVENT; //HCI Type
                    *p++ = h;              //Event Code
                    *p++ = n;              //Parameter Total Length
                    smemcpy(p, para, n);
                    p += n;

                    return 0; //[!!!important]
                }
            }
        }
#endif


        *p++ = n + 3;                   //3: 1(HCI Type) + 1(Event Code) + 1(Parameter Total Length)
        *p++ = (n + 3) >> 8;

        *p++ = HCI_TYPE_EVENT;          //HCI Type
        *p++ = h;                       //Event Code
        *p++ = n;                       //Parameter Total Length
        smemcpy(p, para, n);
        p += n;
    } else if (h & HCI_FLAG_ACL_BT_STD) //ACL data
    {
#if HCI_TX_FIFO_OPTIMIZE_EN
        p = NULL;
        if (((bltHci_txfifo.wptr - bltHci_txfifo.rptr) & 255) < bltHci_txfifo.num - 2) { //keep 2 fifo left for others evt
            p = bltHci_txfifo.p + (bltHci_txfifo.wptr & bltHci_txfifo.mask) * bltHci_txfifo.size;
        }
        if (!p) {
            return -1;
        }

        u8 w = bltHci_txfifo.wptr - 1;

        if ((u8)(bltHci_txfifo.wptr - bltHci_txfifo.rptr) >= bltHci_txfifo.num - 8) {
            while (w != bltHci_txfifo.rptr) //H5 slide win = 1
            {
                u8 *pBuf = bltHci_txfifo.p + (w & bltHci_txfifo.mask) * bltHci_txfifo.size;
                if (pBuf[2] == HCI_TYPE_ACL_DATA ||
                    (pBuf[2] == HCI_TYPE_EVENT && (pBuf[3] != 0x3E || (pBuf[3] == 0x3E && pBuf[5] != 0x02 && pBuf[5] != 0x0D)))) {
                    break;
                }
                w--;
            }

            if (w != (u8)(bltHci_txfifo.wptr - 1)) {
                w++;
                p = bltHci_txfifo.p + (w & bltHci_txfifo.mask) * bltHci_txfifo.size;

                n    = para[1];                                                                   // para[1]: rf_len
                *p++ = n + 5;                                                                     // handle| PB | BC :2byte    data total length: 2byte  : ACL date Type(0x02) 1byte
                *p++ = (n + 5) >> 8;
                *p++ = 0x02;
                *p++ = h;                                                                         // para[1]: llid       //Handle:12|PB flag:2|BC flag:2 (2B)
                *p++ = ((h >> 8) & 0x0F) | ((para[0] & 3) == L2CAP_CONTINUING_PKT ? 0x10 : 0x20); //start llid 2 ->0x20 ;  continue llid 1 ->0x10
                *p++ = n;                                                                         //HCI ACL Data Total Length (2B)
                *p++ = n >> 8;
                smemcpy(p, para + 2, n);
                p += n;
    #if HCI_CONTROLLER_TO_HOST_FLOW_CTRL_EN
                hci_reduceOneHostAvailBuf();
    #endif
                return 0; //[!!!important]
            }
        }
#endif

        /*
        +------+-------+---------+-----------+-----------------+
        | 2    | 1     | 2       | 2         | n               |
        +------+-------+---------+-----------+-----------------+
        | len  | type  | handle  | data_len  | data_total_len  |
        +------+-------+---------+-----------+-----------------+
         */
        n    = para[1]; // para[1]: rf_len
        *p++ = n + 5;   // handle| PB | BC :2byte    data total length: 2byte  : ACL date Type(0x02) 1byte
        *p++ = (n + 5) >> 8;

        *p++ = HCI_TYPE_ACL_DATA;
        *p++ = h;                                                                         // para[1]: llid       //Handle:12|PB flag:2|BC flag:2 (2B)
        *p++ = ((h >> 8) & 0x0F) | ((para[0] & 3) == L2CAP_CONTINUING_PKT ? 0x10 : 0x20); //start llid 2 ->0x20 ;  continue llid 1 ->0x10
        *p++ = n;                                                                         //HCI ACL Data Total Length (2B)
        *p++ = n >> 8;
        smemcpy(p, para + 2, n);
        p += n;

#if HCI_CONTROLLER_TO_HOST_FLOW_CTRL_EN
        hci_reduceOneHostAvailBuf();
#endif
    } else if (h & HCI_FLAG_EVENT_PHYTEST_2_WIRE_UART) {
        *p++ = n; //length
        *p++ = n >> 8;
        smemcpy(p, para, n);
    }
#if (APP_LE_ISO_EN)
    else if (h & HCI_FLAG_ISO_DATA_STD) {
        int len = 1 + n; //type(1)
        *p++    = U16_LO(len);
        *p++    = U16_HI(len);
        *p++    = HCI_TYPE_ISO_DATA;
        smemcpy(p, para, n);
    }
#endif
#if (0)
    else if (h & HCI_FLAG_ISO_DATA_STD) {

        /* HCI ISO out DATA format in telink
        +------+-------+---------+-----------------------+------------+----------------------+-----------------+----------+
        | 2    | 1     | 2       | 2                     | 4          | 2                    | 2               | n        |
        +------+-------+---------+-----------------------+------------+----------------------+-----------------+----------+
        | len  | type  | handle  | ISO_data_load_length  | timestamp  | packet_sequence_num  | iso_sdu_length  | sd_data  |
        +------+-------+---------+-----------------------+------------+----------------------+-----------------+----------+
         */

        u16           iso_data_load_len, len;
        sdu_packet_t *sdu = (sdu_packet_t *)para;

        if (gIsoTsEn) {
            iso_data_load_len = sdu->iso_sdu_len + 8;
        } else {
            iso_data_load_len = sdu->iso_sdu_len + 4;
        }
        len = iso_data_load_len + 5; // type(1), handle(2), iso_data_load_length(2)


        *p++ = U16_LO(len);
        *p++ = U16_HI(len);
        *p++ = HCI_TYPE_ISO_DATA;

        u16 hBit = ((h & 0xFFF) | (HCI_ISO_SDU_COMPLETE << 12) | (((gIsoTsEn) ? 1 : 0) << 14));
        //handle 2byte
        *p++ = U16_LO(hBit);
        *p++ = U16_HI(hBit);

        //iso_data_load_len
        iso_data_load_len &= 0x3FFF;
        *p++ = U16_LO(iso_data_load_len);
        *p++ = U16_HI(iso_data_load_len);

        //timestamp 4byte if ts is set
        if (gIsoTsEn) {
            *p++ = U32_BYTE0(sdu->timestamp);
            *p++ = U32_BYTE1(sdu->timestamp);
            *p++ = U32_BYTE2(sdu->timestamp);
            *p++ = U32_BYTE3(sdu->timestamp);
        }

        //packet sequence number
        *p++ = U16_LO(sdu->pkt_seq_num);
        *p++ = U16_HI(sdu->pkt_seq_num);

        //iso_sdu len
        u16 iso_sdu_len = (sdu->iso_sdu_len & 0x0fff) | (sdu->pkt_st << 14);
        *p++            = U16_LO(iso_sdu_len);
        *p++            = U16_HI(iso_sdu_len);

        smemcpy(p, sdu->data, sdu->iso_sdu_len);
    }
#endif


    bltHci_txfifo.wptr++;

    return 0;
}

#if (MCU_CORE_TYPE == MCU_CORE_TL322X && TLK_MESSAGE_N22)
   #include "stack/multicore_comm/service/service_n22.h"

#if HCI_SEND_NUM_OF_CMP_AFT_ACK
_attribute_ble_ram_code_noinline_
#endif
int n22_blc_hci_send_data(u32 h, u8 *para, int n)
{
    u8  buf[256];
    u8 *p = buf;

    if (h & HCI_FLAG_EVENT_TLK_MODULE) {
    } else if (h & HCI_FLAG_EVENT_BT_STD) {
        /* for avoiding fifo overflow, a tx_fifo over threshold should be added. */
        if((h & 0xff) == HCI_EVT_LE_META &&
            (para[0] == HCI_SUB_EVT_LE_ADVERTISING_REPORT || para[0] == HCI_SUB_EVT_LE_EXTENDED_ADVERTISING_REPORT)){
            if(mcc_n22_hci_is_tx_over_threshold(80))
                return -1;
        }

        *p++ = HCI_TYPE_EVENT; //HCI Type
        *p++ = h;              //Event Code
        *p++ = n;              //Parameter Total Length
        smemcpy(p, para, n);
        while(mcc_n22_hci_send_msg(buf, n + 3)!=0);
    } else if (h & HCI_FLAG_ACL_BT_STD) {
        *p++ = HCI_TYPE_ACL_DATA;
        *p++ = h;                                                                         // para[1]: llid      //Handle:12|PB flag:2|BC flag:2 (2B)
        *p++ = ((h >> 8) & 0x0F) | ((para[0] & 3) == 0x01 ? 0x10 : 0x20); //start llid 2 ->0x20 ;  continue llid 1 ->0x10
        *p++ = n;                                                                         //HCI ACL Data Total Length (2B)
        *p++ = n >> 8;
        smemcpy(p, para + 2, n);
        while(mcc_n22_hci_send_msg(buf, n + 5)!=0);

    #if HCI_CONTROLLER_TO_HOST_FLOW_CTRL_EN
        hci_reduceOneHostAvailBuf();
    #endif
    } else if (h & HCI_FLAG_EVENT_PHYTEST_2_WIRE_UART) {
    }
    else if (h & HCI_FLAG_ISO_DATA_STD) {
        *p++ = HCI_TYPE_ISO_DATA;
        smemcpy(p, para, n);
        while(mcc_n22_hci_send_msg(buf, n + 1)!=0);
    }

    return 0;
}

#endif  /* TLK_MESSAGE_N22 */
