/********************************************************************************************************
 * @file    version_detection.c
 *
 * @brief   This is the header file for 2.4G SDK
 *
 * @author  2.4G GROUP
 * @date    06,2022
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

#include "version_detection.h"
#include "tl_common.h"

unsigned char get_sdk_version(unsigned char *pbuf, unsigned char number)
{
    //The parameter "number" is not used now, it is left here because there was it and has been released.
    //In future, this "number" can be used for a specific purpose, such as if (number == 12) {do something, or return a vendor version}
    (void)number; //clean warning

    //  struct version_format{
    //      char    CERTIFICATION_MARK;         //BLE 5.<certification_mark>
    //      char    SOFT_STRUCTURE;
    //      char    MAJOR_VERSION;
    //      char    MINOR_VERSION;
    //      char    PATCH;
    //      char    month[3];                   //In Eng, like Dec
    //
    //      char    date;                       //Showed in DEC
    //      char    year;
    //      char    hour;
    //      char    minute;
    //  };
    u8 version[] = {
        CERTIFICATION_MARK,
        SOFT_STRUCTURE,
        MAJOR_VERSION,
        MINOR_VERSION,
        PATCH_NUM,
        CUSTOM_MAJOR_VERSION,
        CUSTOM_MINOR_VERSION,
        __DATE__[0],
        __DATE__[1],
        __DATE__[2],                                         //Month
        ((__DATE__[4] - 0x30) << 4) | (__DATE__[5] - 0x30),  //Date
        ((__DATE__[9] - 0x30) << 4) | (__DATE__[10] - 0x30), //Year
        ((__TIME__[0] - 0x30) << 4) | (__TIME__[1] - 0x30),  //Hour
        ((__TIME__[3] - 0x30) << 4) | (__TIME__[4] - 0x30),  //Minute
    };

    memcpy(pbuf, version, sizeof(version));

    return sizeof(version);
}
