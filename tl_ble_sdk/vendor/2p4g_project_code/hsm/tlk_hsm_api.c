/********************************************************************************************************
 * @file    hsm_api.c
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

#include "tlk_hsm_api.h"
#include "sign_verify.h"
void HSM_INIT(void)
{
    tlk_hsm_crypto_engine_init();
}


tlk_hsm_status_t HSM_ECC256_Verify_Data(uint8_t *pubKey, uint8_t *hash, size_t hash_len, uint8_t *signature, size_t signature_len)
{
    tlk_hsm_status_t status = 0;
    status = tlk_hsm_ecc256_verify_data( pubKey , hash,  hash_len, signature,  signature_len);
    return status;
}

tlk_hsm_status_t HSM_ECC256_Sign_Data(uint8_t *data, uint8_t *prikey, uint8_t *hash, size_t hash_len, uint8_t *signature, size_t signature_len)
{
    tlk_hsm_status_t status = 0;
    status = tlk_hsm_ecc256_sign_data( data, prikey, hash,  hash_len, signature,  signature_len);
    return status;
}


tlk_hsm_status_t HSM_Sha256(unsigned char *msg, unsigned int msg_bytes, unsigned char *digest)
{
    tlk_hsm_status_t status = 0;
    status = tlk_sha256( msg, msg_bytes, digest);
    return status;
}



tlk_hsm_status_t HSM_Secu_Sign_Verify(unsigned int bin_addr, unsigned int bin_size, unsigned int ota_size, unsigned int tage_addr, unsigned int root_key_addr,uint8_t secu_sign_vrfy_en)
{
    tlk_hsm_status_t status = 0;
    status = tlk_secu_sign_verify(bin_addr, bin_size, ota_size, tage_addr, root_key_addr, secu_sign_vrfy_en);
    return status;
}
