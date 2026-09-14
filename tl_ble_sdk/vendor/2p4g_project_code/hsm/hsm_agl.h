/********************************************************************************************************
 * @file    hsm_alg.h
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
#ifndef HSM_AGL_H_
#define HSM_AGL_H_
#include "hsm_type.h"

#define RSA2048_LEN_IN_WORDS  64   // 2048 bit = 8 * 32bit unsigned int
#define RSA2048_LEN_BYTES     256 // 2048 bit = 256 bytes


tlk_hsm_status_t tlk_hsm_ecc256_verify_data(unsigned char *pubkey,
                                    uint8_t *hash,
                                    size_t hash_len,
                                    uint8_t *signature,
                                    size_t signature_len);
tlk_hsm_status_t tlk_hsm_ecc256_sign_data(unsigned char *data,
                                    unsigned char *prikey,
                                    uint8_t *hash,
                                    size_t hash_len,
                                    uint8_t *signature,
                                    size_t signature_len);

tlk_hsm_status_t tlk_sha256(unsigned char *msg, unsigned int msg_bytes, unsigned char *digest);
tlk_hsm_status_t tlk_hsm_crypto_engine_init(void);
int tlk_rsa2048_verify(uint8_t *message, uint32_t msg_len, uint8_t *pubKey_n, uint8_t *signature);
#endif
