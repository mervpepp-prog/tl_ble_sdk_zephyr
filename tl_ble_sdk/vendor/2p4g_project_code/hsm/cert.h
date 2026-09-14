/********************************************************************************************************
 * @file    x509
 *
 * @brief   This is the source file for 2.4G SDK
 *
 * @author  2.4G Group
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

#ifndef VENDOR_CERT_H_
#define VENDOR_CERT_H_
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define RSA_2048_SIGN_LEN_BYTES 256


#define FLASH_READ_MODE 1

#define FLASH_READ_BUFF_SIZE 4096
/**
 * @brief Extract RSA public key (modulus + public exponent) from X.509 DER certificate
 *
 * @param der        Input: Pointer to DER formatted certificate data
 * @param der_len    Input: Length of DER data
 * @param n          Output: RSA modulus (Modulus)
 * @param n_len      Input: Buffer size / Output: Actual modulus length
 * @param e          Output: RSA public exponent (Exponent)
 * @param e_len      Input: Buffer size / Output: Actual exponent length
 *
 * @return 0 on success, negative value on error
 */
int x509_get_rsa_pubkey(const uint8_t *der, uint32_t der_len,
                        uint8_t *n, uint32_t *n_len,
                        uint8_t *e, uint32_t *e_len);

/**
 * @brief Extract signature value from X.509 DER certificate
 *
 * @param der        Input: Pointer to DER formatted certificate data
 * @param der_len    Input: Length of DER data
 * @param sig        Output: Certificate signature data
 * @param sig_len    Input: Buffer size / Output: Actual signature length
 *
 * @return 0 on success, negative value on error
 */
int x509_get_signature(const uint8_t *der, uint32_t der_len,
                       uint8_t *sig, uint32_t *sig_len);

// ===================== Configuration =====================
#define MAX_CERTS      2       // Max number of certificates to extract
#define MAX_CERT_SIZE  2048    // Max size of a single certificate in bytes
/**
 * @struct Certificate
 * @brief  Structure to store extracted certificate data and length
 */
typedef struct {
    unsigned char der_data[MAX_CERT_SIZE];//der
    unsigned char local_pub_key_n[256];// cert bubkey n
    unsigned int pub_key_n_addr;//cert bubkey n
    unsigned int sign_addr;        //cert sign_addr
    unsigned int tbs_addr;        //cert tbs_addr
    unsigned int tbs_length;    //cert tbs_length
    unsigned int end_addr;        //cert end_addr
    unsigned int len;             //cert len
} Certificate;


typedef struct {
    Certificate cert_info;
    unsigned char pubkey_n[256];//use to compute pubkey
} Cert_Info;


int get_tage_info(unsigned char *tage_addr, unsigned int *cert_size , unsigned int *tage_size);
int read_cert_info(unsigned char *info_addr, unsigned int info_len, Cert_Info *cert_info, unsigned int *cert_len);

#endif
