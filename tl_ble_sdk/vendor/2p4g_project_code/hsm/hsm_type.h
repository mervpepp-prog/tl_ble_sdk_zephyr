/********************************************************************************************************
 * @file    hsm_type.h
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

#ifndef HSM_TYPES_H_
#define HSM_TYPES_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


typedef unsigned short tlk_hsm_key_type_t;
typedef unsigned char tlk_hsm_ecc_family_t;


typedef unsigned char tlk_hsm_dh_family_t;
typedef unsigned int tlk_hsm_algorithm_t;
typedef unsigned int tlk_hsm_key_lifetime_t;
typedef unsigned char tlk_hsm_key_persistence_t;
typedef unsigned int tlk_hsm_key_location_t;
typedef unsigned int tlk_hsm_key_id_t;
typedef signed int  tlk_hsm_status_t;

#define TLK_HSM_MAX_KEY_BYTES       64U     // Maximum key length: 32 bytes



/** */
#define HSM_SUCCESS                      ((tlk_hsm_status_t)0)        //
#define HSM_ERROR_GENERIC_ERROR          ((tlk_hsm_status_t)-1)       //
#define HSM_ERROR_NOT_SUPPORTED          ((tlk_hsm_status_t)-2)       //
#define HSM_ERROR_NOT_IMPLEMENTED        ((tlk_hsm_status_t)-3)       //
#define HSM_ERROR_INVALID_ARGUMENT       ((tlk_hsm_status_t)-4)       //
#define HSM_ERROR_INVALID_HANDLE         ((tlk_hsm_status_t)-5)       //
#define HSM_ERROR_BAD_STATE              ((tlk_hsm_status_t)-6)       //
#define HSM_ERROR_BUFFER_TOO_SMALL       ((tlk_hsm_status_t)-7)       //
#define HSM_ERROR_ALREADY_EXISTS         ((tlk_hsm_status_t)-8)       //
#define HSM_ERROR_DOES_NOT_EXIST         ((tlk_hsm_status_t)-9)       //
#define HSM_ERROR_INSUFFICIENT_MEMORY    ((tlk_hsm_status_t)-10)      //
#define HSM_ERROR_INSUFFICIENT_STORAGE   ((tlk_hsm_status_t)-11)      //
#define HSM_ERROR_INSUFFICIENT_ENTROPY   ((tlk_hsm_status_t)-12)      //
#define HSM_ERROR_COMMUNICATION_FAILURE  ((tlk_hsm_status_t)-13)      //
#define HSM_ERROR_TIMEOUT                ((tlk_hsm_status_t)-14)      //
#define HSM_ERROR_HARDWARE_FAILURE       ((tlk_hsm_status_t)-15)      //
#define HSM_ERROR_CORRUPTION             ((tlk_hsm_status_t)-16)      //
#define HSM_ERROR_AUTHENTICATION_FAILURE ((tlk_hsm_status_t)-17)      //
#define HSM_ERROR_PERMISSION_DENIED      ((tlk_hsm_status_t)-18)      //
#define HSM_ERROR_NOT_PERMITTED          ((tlk_hsm_status_t)-19)      //
#define HSM_ERROR_INVALID_SIGNATURE      ((tlk_hsm_status_t)-20)      //
#define HSM_ERROR_INVALID_PADDING        ((tlk_hsm_status_t)-21)      //
#define HSM_ERROR_INVALID_HASH           ((tlk_hsm_status_t)-22)      //
#define HSM_ERROR_INVALID_KEY_LENGTH     ((tlk_hsm_status_t)-23)      //
#define HSM_ERROR_INVALID_KEY_TYPE       ((tlk_hsm_status_t)-24)      //
#define HSM_ERROR_INVALID_ALGORITHM      ((tlk_hsm_status_t)-25)      //
#define HSM_ERROR_INVALID_MODE           ((tlk_hsm_status_t)-26)      //
#define HSM_ERROR_INVALID_USAGE          ((tlk_hsm_status_t)-27)      //
#define HSM_ERROR_INVALID_FLAGS          ((tlk_hsm_status_t)-28)      //
#define HSM_ERROR_INVALID_DATA           ((tlk_hsm_status_t)-29)      //
#define HSM_ERROR_INVALID_LENGTH         ((tlk_hsm_status_t)-30)      //
#endif
