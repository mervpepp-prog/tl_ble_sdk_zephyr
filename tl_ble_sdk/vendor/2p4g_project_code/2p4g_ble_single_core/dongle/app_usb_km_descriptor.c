/********************************************************************************************************
 * @file    app_usb_km_descriptor.c
 *
 * @brief   This is the source file for Telink RISC-V MCU
 *
 * @author  Driver Group
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
#include "app_usb_km_descriptor.h"
#include "app_prj_config.h"
#if (USB_DEMO_TYPE == USB_KEYBOARD)
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "tl_usb/core/usbd_core.h"
#include "tl_usb/class/hid/usbd_hid.h"
#include "tl_string.h"

static _attribute_aligned_(4) unsigned char device_desc[] =
{
    0x12,       // sizeof(USB_Descriptor_Device_t)
    0x01,       // Type = DTYPE_Device
    0x00,0x02,  // 0x0110 : USBSpecification, USB 1.1 ,0x0200 USB2.0
    0x00,       // Class : USB_CSCP_NoDeviceClass
    0x00,       // SubClass : USB_CSCP_NoDeviceSubclass
    0x00,       // Protocol : USB_CSCP_NoDeviceProtocol
    0x40,       // Endpoint0Size, Maximum Packet Size for Zero Endpoint. Valid Sizes are 8, 16, 32, 64
    0x8A,0x24,  // VendorID 0x248a   8  9
    USBD_HID_INTERFACE_NUM,0xF4,    // ProductID default value   mouse only:2781  mouse_keyboard_kit:2782
    0x00,0x01,  // 0x0100 : ReleaseNumber
    0x01,       // ManufacturerStrIndex = USB_STRING_VENDOR   @@@ pay attention
    0x02,       // ProductStrIndex = USB_STRING_PRODUCT       @@@ pay attention
    0x03,       // SerialNumStrIndex=USB_STRING_SERIAL
    0x01         // NumberOfConfigurations
};

_attribute_ram_code_sec_ unsigned char *usbd_get_device_descriptor(unsigned char bus)
{
    (void)bus;
    return (unsigned char *)&device_desc;
}

_attribute_aligned_(4) unsigned char  mouse_report_desc[] =
{
/*******************mouse report=66************************/
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x02,  // Usage (Mouse)
    0xA1, 0x01,  // Collection (Application)
    0x85, 0X01,  // Report Id
    0x09, 0x01,  //   Usage (Pointer)
    0xA1, 0x00,  //   Collection (Physical)
    0x05, 0x09,  //  Usage Page (Buttons)
    0x19, 0x01,  //  Usage Minimum (01) - Button 1
    0x29, 0x05,  //  Usage Maximum (03) - Button 3
    0x15, 0x00,  //  Logical Minimum (0)
    0x25, 0x01,  //  Logical Maximum (1)
    0x75, 0x01,  //  Report Size (1)
    0x95, 0x05,  //  Report Count (3)
    0x81, 0x02,  //  Input (Data, Variable, Absolute) - Button states
    0x75, 0x03,  //  Report Size (5)
    0x95, 0x01,  //  Report Count (1)
    0x81, 0x01,  //  Input (Constant) - Padding or Reserved bits

    0x05, 0x01,      //  Usage Page (Generic Desktop Control)
    0x09, 0x30,  // Usage (X)
    0x09, 0x31,  // Usage (Y)


    0x16, 0x01, 0x80, //  LOGICAL_MINIMUM(0)
    0x26, 0xff, 0x7f,
    0x75, 0x10, //  Report Size (16)
    0x95, 0x02, //  Report Count (2)


    0x81, 0x06, //  Input (Data, Variable, Relative)

    //0x05,0x01,             //  Usage Page (Generic Desktop Control)
    0x09, 0x38,          //  Usage (Wheel)
    0x15, 0x81,          //  Logical Minimum (-4)
    0x25, 0x7F,          //  Logical Maximum (3)
    0x75, 0x08,          //  Report Size (3)
    0x95, 0x01,          //  Report Count (1)
    0x81, 0x06,          //  Input (Data, Variable, Relative)

    0x05,0x0c,          // Usage Page(Consumer)
    0x0a,0x38,0x02,     // Usage (AC Pan) tilt
    0x15,0x81,          //  Logical Minimum (-128)
    0x25,0x7F,          //  Logical Maximum (127)
    0x75,0x08,          //  Report Size (8)
    0x95,0x01,          //  Report Count (3)
    0x81,0x06,          // Input (Data, Variable, Relative)


    0xC0,         //   End Collection
    0xC0,         // End Collection

/*******************consume report=25************************/
    0x05, 0x0C,   // Usage Page (Consumer)
    0x09, 0x01,   // Usage (Consumer Control)
    0xA1, 0x01,   // Collection (Application)
    0x85, 0x02,   //     Report Id
    0x75, 0x10,     //global, report size 16 bits
    0x95, 0x01,     //global, report count 1
    0x15, 0x01,     //global, min  0x01
    0x26, 0x8c, 0x02, //global, max  0x28c
    0x19, 0x01,     //local, min   0x01
    0x2a, 0x8c, 0x02, //local, max    0x28c
    0x81, 0x00,     //main,  input data varible, absolute
    0xc0,         //main, end collection

/*******************system power report=33************************/
    0x05, 0x01,     //gobal,  USAGE_PAGE 1 (Generic Desktop)
    0x09, 0x80,     //local, usage ID 0x80 system control
    0xa1, 0x01,     //main conllection
    0x85, 0x03,     //global report ID 0x4
    
    0x09, 0x82,     //local usage ID 0x82 system sleep
    0x09, 0x81,     //local usage ID 0x81 system power down
    0x09, 0x83,     //local usage ID 0x83 system wakeup

    0x15, 0x00,     //global min 01
    0x25, 0x01,     //gobal, max 3
    0x19, 0x01,  //  Usage Minimum (01) - Button 1
    0x29, 0x03,  //  Usage Maximum (03) - Button 3
    
    0x75,0x01,     //global, report size 2
    0x95,0x03,     //report count  1    
    0x81,0x02,     //main, input data, var, abs, No Prefer, NULL state
    //0x75,0x03,  //     Report Size (5)
    0x95,0x05,  //   Report Count (1)
    0x81,0x01,  //   Input (Constant) - Padding or Reserved bits

    0xc0,        //end of collection

    0x06, 0x00, 0xff,   // Usage Page (Vendor-defined),
    0x09, 0x00, // USAGE ()
    0xa1, 0x01, // Collection (Application),
    0x85, 0x05, // Report ID (5)
    0x09, 0x00, // Usage,
    0x15, 0x00, // Logical Minimum (0),
    0x26, 0xff, 0x00,       // Logical Maximum (1),
    0x75, 0x08, // Report Size (8),
    0x95, 0x07, // Report Count (7),
    0x81, 0x02, // Input
    0x09, 0x01,
    0x91, 0x02, // Output (Data, Variable, Absolute)
    0x09, 0x02,
    0xB1, 0x02, // feature (Data, Variable, Absolute)
    0xc0,   // END_COLLECTION




};

_attribute_aligned_(4) unsigned char  kb_report_desc[] =
{
 //keyboard report in
     0x05, 0x01,     // Usage Pg (Generic Desktop)
     0x09, 0x06,     // Usage (Keyboard)
     0xA1, 0x01,     // Collection: (Application)
     //0x85, REPORT_ID_KEYBOARD_INPUT_AAA,   // Report Id (keyboard)
                   //
     0x05, 0x07,     // Usage Pg (Key Codes)
     0x19, 0xE0,     // Usage Min (224)  VK_CTRL:0xe0
     0x29, 0xE7,     // Usage Max (231)  VK_RWIN:0xe7
     0x15, 0x00,     // Log Min (0)
     0x25, 0x01,     // Log Max (1)
                   //
                   // Modifier byte
     0x75, 0x01,     // Report Size (1)   1 bit * 8
     0x95, 0x08,     // Report Count (8)
     0x81, 0x02,     // Input: (Data, Variable, Absolute)
                   //
                   // Reserved byte
     0x95, 0x01,     // Report Count (1)
     0x75, 0x08,     // Report Size (8)
     0x81, 0x01,     // Input: (Constant)
 
     //keyboard output
     //5 bit led ctrl: NumLock CapsLock ScrollLock Compose kana
     0x95, 0x05,    //Report Count (5)
     0x75, 0x01,    //Report Size (1)
     0x05, 0x08,    //Usage Pg (LEDs )
     0x19, 0x01,    //Usage Min
     0x29, 0x05,    //Usage Max
     0x91, 0x02,    //Output (Data, Variable, Absolute)
     //3 bit reserved
     0x95, 0x01,    //Report Count (1)
     0x75, 0x03,    //Report Size (3)
     0x91, 0x01,    //Output (Constant)
 
     // Key arrays (6 bytes)
     0x95, 0x06,     // Report Count (6)
     0x75, 0x08,     // Report Size (8)
     0x15, 0x00,     // Log Min (0)
     0x26, 0xF1,0x00,    // Log Max (241)
     0x05, 0x07,     // Usage Pg (Key Codes)
     0x19, 0x00,     // Usage Min (0)
     0x2a, 0xf1,0x00,    // Usage Max (241)
     0x81, 0x00,     // Input: (Data, Array)
 
     0xC0,            // End Collection
 };

_attribute_aligned_(4) unsigned char  all_kb_report_desc[] =
{

#if 1
         //keyboard report in
          0x05, 0x01,  // Usage Pg (Generic Desktop)
          0x09, 0x06,  // Usage (Keyboard)
          0xA1, 0x01,  // Collection: (Application)
          0x85, 0x08,  // Report Id (keyboard)
         
          0x05, 0x07,  // Usage Pg (Key Codes)
          0x19, 0x00,  // Usage Min (224)  VK_CTRL:0xe0
          0x29, 0x80,  // Usage Max (231)  VK_RWIN:0xe7
          0x15, 0x00,  // Log Min (0)
          0x25, 0x01,  // Log Max (1)
        
          0x75, 0x01,  // Report Size (1)   1 bit * 8
          0x95, 0x80,  // Report Count (8)
          0x81, 0x02,  // Input: (Data, Variable, Absolute
         
          0xC0,             // End Collection
#endif
};
_attribute_aligned_(4) unsigned char vendor_report_desc[] = {

    /******************* usb ota ************************/
    0x06, 0xEF, 0xff,  //global usage page
    0x09, 0x00,  //usage undefined
    0xa1, 0x01,  //main collection
    0x85, USB_VENDOR_REPORT_ID, //global report ID 0x6
    0x15, 0x00,  //LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00, //LOGICAL_MAXIMUM (255)
    0x95, (USB_OTA_LENGTH-1),   //Report Count (32) //MTU_SIZE=23
    0x75, 0x08,  //Report Size (8)
    0x09, 0x01,
    0x81, 0x02,  //INPUT (Data,Var,Abs)
    0x09, 0x02,
    0x91, 0x02,  //Output (Data, Variable, Absolute)
    0x09, 0x03,
    0xB1, 0x02,  //feature (Data, Variable, Absolute)
    0xc0,    //main, end collection
};

#define CFG_LENGTH 116//9+9+9+7+9+9+7+9+9+7+9+9+7+7
_attribute_aligned_(4) unsigned char config_desc[CFG_LENGTH] = {
    //USB_Descriptor_Configuration_Header_t
    0x09,                //sizeof(USB_Descriptor_Configuration_Header_t)
    0x02,                //type= DTYPE_Configuration
    CFG_LENGTH,0x00,     //TotalConfigurationSize,  @@@@ will change @@@@   //0x5b  //0x42
    USBD_HID_INTERFACE_NUM,      //TotalInterfaces          @@@@ will change @@@@
    0x01,           //Configuration index
    0x00,           //ConfigurationStrIndex = NO_DESCRIPTOR
    0xa0,           //ConfigAttributes = USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_REMOTEWAKEUP
    0x32,           //MaxPowerConsumption = USB_CONFIG_POWER_MA(50)  MaxPower = 100mA
    
//--------------0 Interface_t Descriptor--------------------------------------
    0x09,            //size = sizeof(USB_Descriptor_Interface_t)
    0x04,            //type = DTYPE_Interface
    0x00,   //InterfaceNumber          @@@@ will change @@@@
    0x00,  //AlternateSetting = 0
    0x01,  //TotalEndpoints = 1@@@
    0x03,       //Class = HID_CSCP_HIDClass
    0x01,       //SubClass = HID_CSCP_BootSubclass,
    0x02,            //Protocol = HID_CSCP_MouseBootProtocol  @@@@ change  1 keyboard 2 mouse
    0x00,            //InterfaceStrIndex = NO_DESCRIPTOR
//----------------HID_Descriptor---------------------------------------------
    0x09,            //size = sizeof(USB_HID_Descriptor_HID_t)
    0x21,            //type = HID_DTYPE_HID
    0x11,0x01,       //HIDSpec = 0x0111
    0x21,            //CountryCode = USB_HID_COUNTRY_US
    0x01,                //TotalReportDescriptors
    0x22,            //HIDReportType = HID_DTYPE_Report
    sizeof(mouse_report_desc), 0x00,      //HIDReportLength[2] @@@@ change 

//------------------Endpoint Description-----------------
    0x07,           //size = sizeof(USB_Descriptor_Endpoint_t)
    0x05,           //type = DTYPE_Endpoint
    HID_MOUSE_IN_ENDPOINT_ADDRESS, //EndpointAddress  @@@@ change 
    0x03,           //Attributes = EP_TYPE_INTERRUPT
    HID_MOUSE_IN_ENDPOINT_SIZE,0,//EndpointSize = 0x0008  @@@@ change 
    HID_MOUSE_IN_ENDPOINT_INTERVAL,             //PollingIntervalMS  @@@@ change 
//--------------1 Interface_t Descriptor--------------------------------------
    0x09,            //size = sizeof(USB_Descriptor_Interface_t)
    0x04,            //type = DTYPE_Interface
    0x01,   //InterfaceNumber          @@@@ will change @@@@
    0x00,  //AlternateSetting = 0
    0x01,  //TotalEndpoints = 1@@@
    0x03,       //Class = HID_CSCP_HIDClass
    0x01,       //SubClass = HID_CSCP_BootSubclass,
    0x01,            //Protocol = HID_CSCP_MouseBootProtocol  @@@@ change  1 keyboard 2 mouse
    0x00,            //InterfaceStrIndex = NO_DESCRIPTOR
//----------------HID_Descriptor---------------------------------------------
    0x09,            //size = sizeof(USB_HID_Descriptor_HID_t)
    0x21,            //type = HID_DTYPE_HID
    0x11,0x01,       //HIDSpec = 0x0111
    0x21,            //CountryCode = USB_HID_COUNTRY_US
    0x01,                //TotalReportDescriptors
    0x22,            //HIDReportType = HID_DTYPE_Report
    sizeof(kb_report_desc), 0x00,     //HIDReportLength[2] 

//------------------Endpoint Description-----------------
    0x07,           //size = sizeof(USB_Descriptor_Endpoint_t)
    0x05,           //type = DTYPE_Endpoint
    HID_KEYBOARD_IN_ENDPOINT_ADDRESS, //EndpointAddress 
    0x03,           //Attributes = EP_TYPE_INTERRUPT
    HID_KEYBOARD_IN_ENDPOINT_SIZE,0,//EndpointSize = 0x0008
    HID_KEYBOARD_IN_ENDPOINT_INTERVAL,              //PollingIntervalMS

//--------------2 Interface_t Descriptor--------------------------------------
    0x09,            //size = sizeof(USB_Descriptor_Interface_t)
    0x04,            //type = DTYPE_Interface
    0x02,   //InterfaceNumber          @@@@ will change @@@@
    0x00,  //AlternateSetting = 0
    0x01,  //TotalEndpoints = 1@@@
    0x03,       //Class = HID_CSCP_HIDClass
    0x01,       //SubClass = HID_CSCP_BootSubclass,
    0x01,            //Protocol = HID_CSCP_MouseBootProtocol  @@@@ change  1 keyboard 2 mouse
    0x00,            //InterfaceStrIndex = NO_DESCRIPTOR
//----------------HID_Descriptor---------------------------------------------
    0x09,            //size = sizeof(USB_HID_Descriptor_HID_t)
    0x21,            //type = HID_DTYPE_HID
    0x11,0x01,       //HIDSpec = 0x0111
    0x21,            //CountryCode = USB_HID_COUNTRY_US
    0x01,                //TotalReportDescriptors
    0x22,            //HIDReportType = HID_DTYPE_Report
    sizeof(all_kb_report_desc), 0x00,     //HIDReportLength[2] 

//------------------Endpoint Description-----------------
    0x07,           //size = sizeof(USB_Descriptor_Endpoint_t)
    0x05,           //type = DTYPE_Endpoint
    HID_ALL_KB_IN_ENDPOINT_ADDRESS, //EndpointAddress 
    0x03,           //Attributes = EP_TYPE_INTERRUPT
    HID_ALL_KB_IN_ENDPOINT_SIZE,0,//EndpointSize = 0x0008
    HID_ALL_KB_IN_ENDPOINT_INTERVAL,                //PollingIntervalMS

//--------------3 Interface_t Descriptor--------------------------------------
    0x09,            //size = sizeof(USB_Descriptor_Interface_t)
    0x04,            //type = DTYPE_Interface
    0x03,   //InterfaceNumber          @@@@ will change @@@@
    0x00,  //AlternateSetting = 0
    0x02,  //TotalEndpoints = 1@@@
    0x03,       //Class = HID_CSCP_HIDClass
    0x00,       //SubClass = HID_CSCP_BootSubclass,
    0x00,            //Protocol = HID_CSCP_MouseBootProtocol  @@@@ change  1 keyboard 2 mouse
    0x00,            //InterfaceStrIndex = NO_DESCRIPTOR
//----------------HID_Descriptor---------------------------------------------
    0x09,            //size = sizeof(USB_HID_Descriptor_HID_t)
    0x21,            //type = HID_DTYPE_HID
    0x11,0x01,       //HIDSpec = 0x0111
    0x21,            //CountryCode = USB_HID_COUNTRY_US
    0x01,                //TotalReportDescriptors
    0x22,            //HIDReportType = HID_DTYPE_Report
    sizeof(vendor_report_desc), 0x00,     //HIDReportLength[2] 

//------------------Endpoint Description-----------------
    0x07,           //size = sizeof(USB_Descriptor_Endpoint_t)
    0x05,           //type = DTYPE_Endpoint
    CDC_DATA_IN_ENDPOINT_ADDRESS, //EndpointAddress 
    0x03,           //Attributes = EP_TYPE_INTERRUPT
    CDC_DATA_IN_ENDPOINT_SIZE,0,//EndpointSize = 0x0008
    1,              //PollingIntervalMS
//------------------Endpoint Description-----------------
    0x07,           //size = sizeof(USB_Descriptor_Endpoint_t)
    0x05,           //type = DTYPE_Endpoint
    CDC_DATA_OUT_ENDPOINT_ADDRESS, //EndpointAddress 
    0x03,           //Attributes = EP_TYPE_INTERRUPT
    CDC_DATA_OUT_ENDPOINT_SIZE,0,//EndpointSize = 0x0008
    1,              //PollingIntervalMS

};

_attribute_ram_code_sec_ unsigned char *usbd_get_config_descriptor(unsigned char bus)
{
    (void)bus;
    app_usb_status_set(CONFIG_SET_FLAG_APP);
    return (unsigned char *)&config_desc;
}

_attribute_aligned_(4) unsigned char device_qualifier[] = {0x0a, 0x06, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00};

_attribute_ram_code_sec_ unsigned char *usbd_get_device_qualifier_descriptor(unsigned char bus)
{
    (void)bus;

    return device_qualifier;
}

// this example use the same configuration for both high and full speed mode
_attribute_aligned_(4) unsigned char other_speed_config[CFG_LENGTH];

_attribute_ram_code_sec_ unsigned char *usbd_get_other_speed_configuration_descriptor(unsigned char bus, unsigned char index)
{
    (void)bus;
    (void)index;
    tmemcpy(&other_speed_config[0], &config_desc[0], CFG_LENGTH);
    other_speed_config[1] = USB_DESC_OTHER_SPEED_CONFIG;

    return other_speed_config;
}

_attribute_aligned_(4) char dev_desc_language[4] = {0x09, 0x04, 0};
_attribute_aligned_(4) char dev_desc_manufacturer_name[sizeof(APP_USB_KM_DESC_MANU_FACTURER_STR) + 1] = APP_USB_KM_DESC_MANU_FACTURER_STR;
_attribute_aligned_(4) char dev_desc_product_name[sizeof(APP_USB_KM_DESC_PRODUCT_STR) + 1] = APP_USB_KM_DESC_PRODUCT_STR;
_attribute_aligned_(4) char dev_desc_serial_num[sizeof(APP_USB_KM_DESC_SERIAL_NUM_STR) + 1] = APP_USB_KM_DESC_SERIAL_NUM_STR;

_attribute_aligned_(4) char *string_desc_arr[] = {
    dev_desc_language, // 0: is supported language is English (0x0409)
    dev_desc_manufacturer_name, // 1: Manufacturer
    dev_desc_product_name, // 2: Product
    dev_desc_serial_num, // 3: Serials will use unique ID if possible
};

_attribute_aligned_(4) static unsigned short _desc_str[64 + 1];

_attribute_ram_code_sec_ unsigned char *usbd_get_string_descriptor(unsigned char bus, unsigned char index)
{
    (void)bus;
    (void)index;
    unsigned short chr_count;

    switch (index) {
    case 0:
        tmemcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
        break;

    default:
        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
            return NULL;
        }

        char *str = string_desc_arr[index];

        // Cap at max char
        chr_count              = tstrlen(str);
        size_t max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1; // -1 for string type
        if (chr_count > max_count) {
            chr_count = max_count;
        }

        // Convert ASCII string into UTF-16
        for (size_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
        break;
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (uint16_t)((USB_DESC_STRING << 8) | (2 * chr_count + 2));

    return (unsigned char *)_desc_str;
}

_attribute_ram_code_sec_ unsigned char *usbd_hid_get_report_descriptor(unsigned char bus, unsigned char intf, unsigned short *len)
{
    (void)bus;
    (void)intf;
    (void)len;
    app_usb_status_set(MAP_GET_FLAG_APP);
    app_ctx.is_usb_desc_done = 1;
    app_ctx.usb_desc_tick = stimer_get_tick() | 1;
    if(intf==1)
    {
       *len=sizeof(kb_report_desc);
       return (unsigned char*)&kb_report_desc[0];
    }
    else if(intf==0)
    {
       *len=sizeof(mouse_report_desc);
       return (unsigned char*)&mouse_report_desc[0];
       
    }
    else if(intf==2)
    {
       *len=sizeof(all_kb_report_desc);
       return (unsigned char*)&all_kb_report_desc[0];
       
    }
    else if(intf==3)
    {
       *len=sizeof(vendor_report_desc);
       return (unsigned char*)&vendor_report_desc[0];
       
    }
    return NULL;
}

#endif
