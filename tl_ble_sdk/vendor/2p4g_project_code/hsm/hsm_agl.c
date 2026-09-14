/********************************************************************************************************
 * @file    hsm_alg.c
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

#include "driver.h"
#include "hsm_agl.h"


void array_reverse_copy(const uint8_t *src, uint8_t *dest, uint32_t len)
{
    if (src == NULL || dest == NULL || len == 0) {
        return;
    }
    memcpy(dest, src, len);
    uint32_t start = 0;
    uint32_t end = len - 1;
    uint8_t temp;

    while (start < end) {
        temp = dest[start];
        dest[start] = dest[end];
        dest[end] = temp;

        start++;
        end--;
    }
}

tlk_hsm_status_t tlk_hsm_crypto_engine_init(void)
{
    trng_dig_en();
    ske_dig_en();
    hash_dig_en();
    pke_dig_en();
    return HSM_SUCCESS;
}

tlk_hsm_status_t tlk_sha256(unsigned char *msg, unsigned int msg_bytes, unsigned char *digest)
{
    tlk_hsm_status_t status = HSM_SUCCESS;
    status = sha256(msg, msg_bytes, digest);
    return status;
}

tlk_hsm_status_t tlk_hsm_ecdsa_std_sign(eccp_curve_t *curve, unsigned char *hash, unsigned int hash_len, unsigned char *data, unsigned char *prikey, unsigned char *signature)
{
    tlk_hsm_status_t status = HSM_SUCCESS;
    status = ecdsa_sign(curve, hash, hash_len, data, prikey, signature);
    if (status != HSM_SUCCESS )
    {
        return status;
    }
//  printf("\r\n --------------tlk_hsm_ecc256_sign -------------- ");
    return status;
}


//esdsa256r1
tlk_hsm_status_t tlk_hsm_ecc256_sign_data(unsigned char *data,
                                    unsigned char *prikey,
                                    uint8_t *hash,
                                    size_t hash_len,
                                    uint8_t *signature,
                                    size_t signature_len)
{
    (void)signature_len;
    eccp_curve_t *curve   = (eccp_curve_t *)secp256r1;
    unsigned int nByteLen         = GET_BYTE_LEN(curve->eccp_n_bitLen);
    signature_len = 2*nByteLen;
    tlk_hsm_status_t status = HSM_SUCCESS;
    status = tlk_hsm_ecdsa_std_sign(curve, hash, hash_len, data, prikey, signature);
    // if (status != HSM_SUCCESS )
    // {
    //     return status;
    // }
    return status;
}


tlk_hsm_status_t tlk_hsm_ecdsa_std_sign_verify(eccp_curve_t *curve, unsigned char *hash, unsigned int hash_len, unsigned char *pubkey, unsigned char *signature)
{
    tlk_hsm_status_t status = HSM_SUCCESS;
    status = ecdsa_verify(curve, hash, hash_len, pubkey, signature);
    if (status != HSM_SUCCESS )
    {
        return status;
    }
//  printf("\r\n --------------tlk_hsm_ecc256_verify -------------- ");
    return status;
}


//esdsa256r1
tlk_hsm_status_t tlk_hsm_ecc256_verify_data(unsigned char *pubkey,
                                    uint8_t *hash,
                                    size_t hash_len,
                                    uint8_t *signature,
                                    size_t signature_len)
{
    (void)signature_len;
    eccp_curve_t *curve   = (eccp_curve_t *)secp256r1;
    tlk_hsm_status_t status = HSM_SUCCESS;
    status = tlk_hsm_ecdsa_std_sign_verify(curve, hash, hash_len, pubkey, signature);
    // if (status != HSM_SUCCESS)
    // {
    //     return status;
    // }
    return status;
}

// Public exponent e = 65537 (0x10001)
static const unsigned int RSA_PUB_E = 0x10001;
static const unsigned int E_BIT_LEN = 17;
unsigned int length = 0x333f3;

// RSA2048 public key modulus n (Big-Endian format)



int tlk_rsa2048_verify(uint8_t *message, uint32_t msg_len, uint8_t *pubKey_n, uint8_t *signature)
{

    print_buf_U8((unsigned char *)&msg_len, 4, "tlk_rsa2048_verify:");
    if((message == NULL) || (pubKey_n == NULL) ||(signature == NULL)|| (msg_len == 0))
    {
        return -1;// input invalid
    }

    uint8_t out_words[RSA2048_LEN_BYTES] = {0};
    uint8_t decrypted[RSA2048_LEN_BYTES] = {0};
    uint8_t sha256_digest[32] = {0};  // SHA256 digest output (32 bytes)

    tlk_sha256(message, msg_len, sha256_digest);

    print_buf_U8(sha256_digest, 32, "AAAsha256_digest_1:");

    print_buf_U8(signature, 32, "signature :");
    print_buf_U8(pubKey_n, 32, "pubKey :");
    print_buf_U8(message, msg_len, "message :");
    print_buf_U8((unsigned char *)&msg_len, 4, "msg_len :");
    ///////////////////////Sign////////////////////
    //256 reverse
    unsigned int sig_words_revs[RSA2048_LEN_IN_WORDS] = {0};

    //sign last
    array_reverse_copy((uint8_t *)signature, (uint8_t *)sig_words_revs, RSA2048_LEN_IN_WORDS*4);
    print_buf_U8((unsigned char *)sig_words_revs, 32*8, "sig_words_revs :");

    ///////////////////////RSA_N////////////////////
    unsigned int n_data_revs[RSA2048_LEN_IN_WORDS] = {0};
    unsigned int rsa_pub_e = RSA_PUB_E;
    array_reverse_copy((uint8_t *)pubKey_n, (uint8_t *)n_data_revs, RSA2048_LEN_IN_WORDS*4);

    
    RSA_ModExp((unsigned int*)sig_words_revs, &rsa_pub_e, (unsigned int*)n_data_revs,
               (unsigned int*)out_words, E_BIT_LEN, 2048);

    // Convert result from unsigned int array back to byte array
    array_reverse_copy((uint8_t *)out_words, (uint8_t *)decrypted, RSA2048_LEN_IN_WORDS*4);
    print_buf_U8(out_words, 256, "out_words:");
    print_buf_U8(&decrypted[RSA2048_LEN_BYTES - 32], 32, "decrypted:");

    // Compare the two hashes to determine verification result
    if (memcmp(&decrypted[RSA2048_LEN_BYTES - 32], sha256_digest, 32) == 0) {
        print_buf_U8(sha256_digest, 32, "Success :");
        print_buf_U8(&decrypted[RSA2048_LEN_BYTES - 32], 32, "Success :");
        return 0;
    } else {
        print_buf_U8(sha256_digest, 32, "fail digest:");
        print_buf_U8(decrypted, 32, "fail out_words:");
        return -2;
    }
}



