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

#include "sign_verify.h"
#include "cert.h"
#include "hsm_agl.h"
#include "tl_string.h"
#include "driver.h"
typedef enum
{
    SIGNVRFY_VERIFYSIGNATURE_IDLE  = 0,  //
    SIGNVRFY_ROOTKEYISVALID        = 1,  //
    SIGNVRFY_VERIFY_SIGNATURE_ROOT_CERT  = 2,  //
    SIGNVRFY_VERIFY_SIGNATURE_INTER_CERT = 3,  //
    SIGNVRFY_VERIFY_SIGNATURE_CLIENT_CERT = 4, //
    SIGNVRFY_VERIFY_SIGNATURE_LAST = 5, //
    SIGNVRFY_VERIFYSIGNATURE_SUCCESS  = 6,  //
    SIGNVRFY_VERIFY_END               = 7,  //
} u_Secu_SignVrfyState_t;

typedef enum
{

    SIGNVRFY_VERIFY_IDLE                           =  1,  //
    SIGNVRFY_VERIFY_SUCCESS                        =  0,  //
    ERROR_INPUT_IS_INVALID                        = -1,  //
    ERROR_SIGNVRFY_VERIFY_SIGNATURE_ROOT_CERT   = -2,  //
    ERROR_SIGNVRFY_VERIFY_SIGNATURE_INTER_CERT     = -3,  //
    ERROR_SIGNVRFY_VERIFY_SIGNATURE_CLIENT_CERT = -4, //
    ERROR_SIGNVRFY_VERIFY_SIGNATURE_LAST        = -5, //
    ERROR_GET_TAGE_INFO                            = -6,  //
    ERROR_SIGNVRFY_VERIFY_END                       = -7,  //
    ERROR_READ_CERT_INFO                           = -8,  //
    ERROR_READ_ROOT_CERT_INFO                           = -9,  //
    ERROR_READ_INTER_CERT_INFO                           = -10,  //
    ERROR_READ_CLIENT_CERT_INFO                           = -11,  //

} u_Secu_SignVrfyErrorState_t;




u_Secu_SignVrfyState_t secu_sign_vrfy_status = SIGNVRFY_VERIFYSIGNATURE_IDLE;
u_Secu_SignVrfyState_t secu_sing_vrfy_error_status = SIGNVRFY_VERIFYSIGNATURE_IDLE;


Cert_Info cert_info;

int tlk_secu_sign_verify_init(void)
{
    int status = 0;
    status = tlk_hsm_crypto_engine_init();
    return status;
}


unsigned int tlk_sha256_flash_read(unsigned int flash_addr, unsigned int data_len, unsigned char *digest)
{

    print_buf_U8((unsigned char *)&flash_addr, 4, "AAAflash_addr :");
    print_buf_U8((unsigned char *)&data_len, 4, "AAAdata_len :");
    unsigned char read_flash_data[FLASH_READ_BUFF_SIZE];
    HASH_CTX     ctx[1];
    unsigned int i, ret;

    ret = hash_init(ctx, HASH_SHA256);
    if (HASH_SUCCESS != ret) {
        return ret;
    } else {
        ;
    }

    unsigned int flash_read_offset = 0;

    unsigned int node_num;
    node_num = data_len/FLASH_READ_BUFF_SIZE;

    for (i = 0U; i < node_num; i++) {

        flash_read_page(flash_addr + flash_read_offset , FLASH_READ_BUFF_SIZE,  (unsigned char *)read_flash_data);
        flash_addr += FLASH_READ_BUFF_SIZE;
        ret = hash_update(ctx, read_flash_data, FLASH_READ_BUFF_SIZE);
        if (HASH_SUCCESS != ret) {
            return ret;
        } else {
            ;
        }
    }

    if(data_len%FLASH_READ_BUFF_SIZE)
    {
        flash_read_page(flash_addr + flash_read_offset , data_len%FLASH_READ_BUFF_SIZE,  (unsigned char *)read_flash_data);
        ret = hash_update(ctx, read_flash_data, data_len%FLASH_READ_BUFF_SIZE);
        if (HASH_SUCCESS != ret) {
            return ret;
        } else {
            ;
        }
    }

    return hash_final(ctx, digest);

    return ret;
}

int tlk_secu_sign_verify(unsigned int bin_addr, unsigned int bin_size, unsigned int ota_size, unsigned int tage_addr, unsigned int root_cert_addr ,uint8_t secu_sign_vrfy_en)
{
    print_buf_U8((unsigned char *)&bin_addr, 4, "bin_addr :");
    print_buf_U8((unsigned char *)&tage_addr, 4, "tage_addr :");
    print_buf_U8((unsigned char *)&bin_size, 4, "bin_size :");
    print_buf_U8((unsigned char *)&root_cert_addr, 4, "root_cert_addr :");
#if FLASH_READ_MODE
    unsigned char read_flash_data[FLASH_READ_BUFF_SIZE];
    flash_read_page(tage_addr, 256, (unsigned char *)read_flash_data);
    print_buf_U8(read_flash_data, 256, "read_flash_data_tage_addr :");
#else
    unsigned char *bin_p = bin_addr;
    print_buf_U8(bin_p, 512, "bin_p :");
#endif

//  unsigned char *bin_p = bin_addr;
//  print_buf_U8(bin_p, 512, "bin_p :");


    int ret = SIGNVRFY_VERIFY_IDLE;
    unsigned int cert_data_size = 0;
    unsigned int cert_size_size = 0;
    unsigned int tage_data_size = 0;
    unsigned int cert_begin_addr = 0;

    if((bin_size == 0) || (tage_addr == 0))
    {
        return ERROR_INPUT_IS_INVALID;
    }

    if(!secu_sign_vrfy_en){
        return 0;
    }else{

#if FLASH_READ_MODE
        ret = get_tage_info((unsigned char *)read_flash_data, &cert_data_size, &tage_data_size);//return
#else
        ret = get_tage_info((unsigned char *)tage_addr, &cert_data_size, &tage_data_size);//return
#endif
        if(ret)
        {

            ret = ERROR_READ_ROOT_CERT_INFO;
            return ret;
        }
        cert_size_size = cert_data_size;
        ret = tlk_secu_sign_verify_init();
        if(ret)
        {
            return ret;
        }
        //compute next cert addr
        cert_begin_addr = tage_addr + tage_data_size; //tage size is 31


        secu_sign_vrfy_en = 0;
        secu_sign_vrfy_status = SIGNVRFY_VERIFY_SIGNATURE_ROOT_CERT;
        print_buf_U8((unsigned char *)&secu_sign_vrfy_status, 1, "SIGNVRFY_VERIFY_REQ :");
    }

    if(secu_sign_vrfy_status == SIGNVRFY_VERIFY_SIGNATURE_ROOT_CERT)
    {
        unsigned int root_cert_data_size = 2048;
        unsigned int root_cert_pem_size = 0;
#if FLASH_READ_MODE
        flash_read_page(root_cert_addr, FLASH_READ_BUFF_SIZE, (unsigned char *)read_flash_data);
        print_buf_U8(read_flash_data, 256, "read_flash_data_tage_addr :");
        ret = read_cert_info(read_flash_data, root_cert_data_size, &cert_info, &root_cert_pem_size);
#else
        ret = read_cert_info(root_cert_addr, root_cert_data_size, &cert_info, &root_cert_pem_size);
#endif

        if(ret)
        {
            ret = ERROR_READ_INTER_CERT_INFO;
            return ret;
        }


        if(tlk_rsa2048_verify((uint8_t *)&cert_info.cert_info.der_data[cert_info.cert_info.tbs_addr], cert_info.cert_info.tbs_length, (uint8_t *)&cert_info.cert_info.der_data[cert_info.cert_info.pub_key_n_addr], &cert_info.cert_info.der_data[cert_info.cert_info.sign_addr]))
        {
            ret = ERROR_SIGNVRFY_VERIFY_SIGNATURE_ROOT_CERT;
            return ret;
        }

        memcpy((void*)cert_info.pubkey_n, (const void *)&cert_info.cert_info.der_data[cert_info.cert_info.pub_key_n_addr], RSA2048_LEN_BYTES);
        // memcpy(cert_info.pubkey_n, (unsigned char*)&cert_info.cert_info.der_data[cert_info.cert_info.pub_key_n_addr], RSA2048_LEN_BYTES);
        print_buf_U8(cert_info.pubkey_n, RSA2048_LEN_BYTES, "SIGNVRFY_VERIFYCERTSIGNATURE_1 :");


        secu_sign_vrfy_status = SIGNVRFY_VERIFY_SIGNATURE_INTER_CERT;
        print_buf_U8((unsigned char *)&secu_sign_vrfy_status, 1, "SIGNVRFY_VERIFYCERTSIGNATURE_1 :");
    }

    if(secu_sign_vrfy_status == SIGNVRFY_VERIFY_SIGNATURE_INTER_CERT)//
    {
        unsigned int cert_a_pem_size = 0;

#if FLASH_READ_MODE
        flash_read_page(cert_begin_addr, FLASH_READ_BUFF_SIZE, (unsigned char *)read_flash_data);
        print_buf_U8(read_flash_data, 256, "read_flash_data_tage_addr :");
        ret = read_cert_info(read_flash_data, cert_data_size, &cert_info, &cert_a_pem_size);
        cert_info.cert_info.end_addr = cert_begin_addr + cert_a_pem_size;
#else
        ret = read_cert_info(cert_begin_addr, cert_data_size,  &cert_info , &cert_a_pem_size);
#endif
        if(ret)
        {
            ret = ERROR_READ_INTER_CERT_INFO;
            return ret;
        }
        //First  pubkey:  sign :   input:
        if(tlk_rsa2048_verify((uint8_t *)&cert_info.cert_info.der_data[cert_info.cert_info.tbs_addr], cert_info.cert_info.tbs_length, (uint8_t *)cert_info.pubkey_n, &cert_info.cert_info.der_data[cert_info.cert_info.sign_addr]))
        {
            ret = ERROR_SIGNVRFY_VERIFY_SIGNATURE_INTER_CERT;
            return ret;
        }

        memcpy((void*)cert_info.pubkey_n, (const void*)&cert_info.cert_info.der_data[cert_info.cert_info.pub_key_n_addr], RSA2048_LEN_BYTES);

        cert_data_size = cert_data_size - cert_a_pem_size;
        cert_begin_addr = cert_info.cert_info.end_addr;
        secu_sign_vrfy_status = SIGNVRFY_VERIFY_SIGNATURE_CLIENT_CERT;

        //log
        print_buf_U8((unsigned char *)&cert_data_size, 4, "cert_data_size :");
        print_buf_U8(cert_info.pubkey_n, RSA2048_LEN_BYTES, "cert_info.pubkey_n :");
        print_buf_U8((unsigned char *)&cert_data_size, 4, "cert_data_size :");
        print_buf_U8((unsigned char *)&cert_begin_addr, 4, "cert_begin_addr :");
        print_buf_U8((unsigned char *)&secu_sign_vrfy_status, 1, "SIGNVRFY_VERIFYCERTSIGNATURE_2 :");
    }

    if(secu_sign_vrfy_status == SIGNVRFY_VERIFY_SIGNATURE_CLIENT_CERT)
    {
        unsigned int cert_b_pem_size = 0;

#if FLASH_READ_MODE
        flash_read_page(cert_begin_addr, FLASH_READ_BUFF_SIZE, (unsigned char *)read_flash_data);
        print_buf_U8(read_flash_data, 256, "read_flash_data_tage_addr :");
        ret = read_cert_info(read_flash_data, cert_data_size, &cert_info, &cert_b_pem_size);
        cert_info.cert_info.end_addr = cert_begin_addr + cert_b_pem_size;
#else
        ret = read_cert_info(cert_begin_addr, cert_data_size,  &cert_info , &cert_b_pem_size);
#endif
        if(ret)
        {
            ret = ERROR_READ_CLIENT_CERT_INFO;
            return ret;
        }

        if(tlk_rsa2048_verify((uint8_t *)&cert_info.cert_info.der_data[cert_info.cert_info.tbs_addr], cert_info.cert_info.tbs_length, (uint8_t *)cert_info.pubkey_n, &cert_info.cert_info.der_data[cert_info.cert_info.sign_addr]))
        {
            ret = ERROR_SIGNVRFY_VERIFY_SIGNATURE_CLIENT_CERT;
            return ret;
        }

        memcpy((void*)cert_info.pubkey_n, (const void*)&cert_info.cert_info.der_data[cert_info.cert_info.pub_key_n_addr], RSA2048_LEN_BYTES);
        print_buf_U8(cert_info.pubkey_n, RSA2048_LEN_BYTES, "cert_info.pubkey_n :");
        cert_data_size = cert_data_size - (cert_info.cert_info.end_addr - cert_begin_addr);
        cert_begin_addr = cert_info.cert_info.end_addr;

        secu_sign_vrfy_status = SIGNVRFY_VERIFY_SIGNATURE_LAST;
        print_buf_U8((unsigned char *)&secu_sign_vrfy_status, 1, "SIGNVRFY_VERIFYCERTSIGNATURE_3 :");
    }

    if(secu_sign_vrfy_status == SIGNVRFY_VERIFY_SIGNATURE_LAST)
    {
        print_buf_U8((unsigned char *)&cert_begin_addr, 4, "cert_begin_addr :");
        uint8_t sign_data[RSA2048_LEN_BYTES] = {0};
        uint8_t sha256_digest_0[32] = {0};
        unsigned int input_size = bin_size + tage_data_size;
        print_buf_U8((unsigned char *)&input_size, 4, "input_size :");

        unsigned int sign_addr =  bin_addr + ota_size + tage_data_size + cert_size_size;
#if FLASH_READ_MODE

        flash_read_page(sign_addr, RSA2048_LEN_BYTES, (unsigned char *)read_flash_data);
        memcpy((void*)sign_data, (const void*)read_flash_data, RSA2048_LEN_BYTES);

        tlk_sha256_flash_read(bin_addr, ota_size + tage_data_size, sha256_digest_0);
#else

        unsigned int sign_offset = bin_size - (ota_size + tage_data_size + cert_size_size);

        print_buf_U8(&sign_offset, 4, "TTTsign_offsetTTT :");
        print_buf_U8(&sign_addr, 4, "TTTsign_addrTTT :");
//        memcpy((void*)sign_data, (const void*)(cert_begin_addr+2), RSA2048_LEN_BYTES);
        memcpy((void*)sign_data, (const void*)sign_addr, RSA2048_LEN_BYTES);

//        memcpy((void*)sign_data, (const void*)(cert_begin_addr+1), RSA2048_LEN_BYTES);

        tlk_sha256(bin_addr, ota_size + tage_data_size, sha256_digest_0);
#endif

        print_buf_U8(sign_data, RSA2048_LEN_BYTES, "sign_data :");

        print_buf_U8(sha256_digest_0, 32, "sha256_digest_0 :");
        if(tlk_rsa2048_verify(sha256_digest_0, 32, (uint8_t *)cert_info.pubkey_n, sign_data))
        {
            ret = ERROR_SIGNVRFY_VERIFY_SIGNATURE_LAST;
            return ret;
        }
        secu_sign_vrfy_status = SIGNVRFY_VERIFYSIGNATURE_SUCCESS;
        print_buf_U8((unsigned char *)&secu_sign_vrfy_status, 1, "SIGNVRFY_VERIFYSIGNATURE_SUCCESS :");
    }

    if(secu_sign_vrfy_status == SIGNVRFY_VERIFYSIGNATURE_SUCCESS)
    {
        secu_sign_vrfy_status = SIGNVRFY_VERIFY_END;
        print_buf_U8((unsigned char *)&secu_sign_vrfy_status, 1, "SIGNVRFY_VERIFYSIGNATURE_SUCCESS :");
    }

    if(secu_sign_vrfy_status == SIGNVRFY_VERIFY_END)
    {
        ret = SIGNVRFY_VERIFY_SUCCESS;
        print_buf_U8((unsigned char *)&secu_sign_vrfy_status, 1, "SIGNVRFY_VERIFY_END :");
    }
    return ret;
}
