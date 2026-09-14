/********************************************************************************************************
 * @file    app.c
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
#ifndef TLK_VERIFY_H_
#define TLK_VERIFY_H_
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


/**
 * @brief Security signature verification function
 *
 * Verifies the digital signature of a given binary data block using a root public key/key material.
 * Supports enabling/disabling the signature verification process via a control flag.
 *
 * @param bin_addr      Pointer to the start address of the raw binary data to be verified
 * @param bin_size      Size of the raw binary data in bytes  (FW size + TAGE + CERT + SIGN)
 * @param ota_size      FW size no cert etc.
 * @param tage_addr     Pointer to the start address of the signature tag to be checked
 * @param root_key_addr Pointer to the root public key/key data used for signature verification
 * @param secu_sign_vrfy_en Enable control for signature verification
 *                          - 0: Disable signature verification
 *                          - 1: Enable signature verification
 *
 * @return int          Status code
 *                      - 0: Success (verification passed or verification disabled)
 *                      - Non-zero: Error (invalid parameters, verification failed, internal error)
 */
int tlk_secu_sign_verify(unsigned int bin_addr, unsigned int bin_size,unsigned int ota_size, unsigned int tage_addr, unsigned int root_cert_addr,uint8_t secu_sign_vrfy_en);

#endif
