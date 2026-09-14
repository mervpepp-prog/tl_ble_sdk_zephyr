/********************************************************************************************************
 * @file    app_rx.c
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

#include "cert.h"
#include "driver.h"
#include "tl_string.h"

static const uint8_t base64_decoding_table[256] = {
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x3E,0xFF,0xFF,0xFF,0x3F,
    0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,
    0x0F,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,
    0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,0x30,0x31,0x32,0x33,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};
#if 1
//
uint8_t *base64_decode(const uint8_t *base64_data, size_t base64_len, uint8_t *out, size_t *out_len) {
    if (!base64_data || !out_len) return NULL;

    print_buf_U8(base64_data, base64_len, "base64_decode :");
    size_t valid_len = 0;
    for (size_t i = 0; i < base64_len; i++) {
        if (base64_data[i] != '=' && base64_data[i] != '\n' && base64_data[i] != '\r') {
            valid_len++;
        }
    }

    if (!out) return NULL;

    size_t out_pos = 0;
    uint32_t temp = 0;
    int bits = 0;

    for (size_t i = 0; i < base64_len; i++) {
        uint8_t c = base64_data[i];
        uint8_t val = base64_decoding_table[c];

        if (val == 0xFF) continue;

        temp = (temp << 6) | val;
        bits += 6;

        if (bits >= 8) {
            bits -= 8;
            out[out_pos++] = (temp >> bits) & 0xFF;
        }
    }

    *out_len = out_pos;
    return out;
}

#endif


int asn1_read_length(const unsigned char *p, int max, int *out_len) {
    if (max < 1) return -1;
    int len = p[0];
    if (len & 0x80) {
        int octets = len & 0x7F;
        if (octets > 4 || max < 1 + octets) return -1;
        len = 0;
        for (int i = 0; i < octets; i++) len = (len << 8) | p[1 + i];
        *out_len = len;
        return 1 + octets;
    } else {
        *out_len = len;
        return 1;
    }
}

const unsigned char* asn1_skip_tlv(const unsigned char *p, int remaining) {
    if (remaining < 1) return NULL;
    int len_bytes, value_len;
    len_bytes = asn1_read_length(p + 1, remaining - 1, &value_len);
    if (len_bytes < 0) return NULL;
    if (1 + len_bytes + value_len > remaining) return NULL;
    return p + 1 + len_bytes + value_len;
}

const unsigned char* asn1_enter_sequence(const unsigned char *p, int remaining, int *out_len) {
    if (remaining < 1 || p[0] != 0x30) return NULL;
    int len_bytes, value_len;
    len_bytes = asn1_read_length(p + 1, remaining - 1, &value_len);
    if (len_bytes < 0) return NULL;
    if (1 + len_bytes + value_len > remaining) return NULL;
    *out_len = value_len;
    return p + 1 + len_bytes;
}

const unsigned char* asn1_read_integer(const unsigned char *p, int remaining, int *out_len) {
    if (remaining < 1 || p[0] != 0x02) return NULL;
    int len_bytes, value_len;
    len_bytes = asn1_read_length(p + 1, remaining - 1, &value_len);
    if (len_bytes < 0) return NULL;
    if (1 + len_bytes + value_len > remaining) return NULL;
    *out_len = value_len;
    return p + 1 + len_bytes;
}

int find_rsa_pubkey_in_tbs(const unsigned char *tbs, int tbs_len,
                           const unsigned char **n, int *n_len,
                           const unsigned char **e, int *e_len) {
    const unsigned char rsa_oid[] = {0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x01};
    int oid_len = sizeof(rsa_oid);

    for (int i = 0; i <= tbs_len - oid_len; i++) {
        if (memcmp(tbs + i, rsa_oid, oid_len) == 0) {
            const unsigned char *p = tbs + i + oid_len;
            if (p[0] == 0x05 && p[1] == 0x00) p += 2;
            if (p[0] != 0x03) continue;
            int bit_len, len_bytes;
            len_bytes = asn1_read_length(p + 1, tbs_len - (p - tbs) - 1, &bit_len);
            if (len_bytes < 0) continue;
            if (bit_len < 1) continue;
            const unsigned char *bit_data = p + 1 + len_bytes;
            int unused_bits = bit_data[0];
            if (unused_bits != 0) continue;
            const unsigned char *rsa_pub = bit_data + 1;
            int rsa_pub_len = bit_len - 1;
            if (rsa_pub_len < 2 || rsa_pub[0] != 0x30) continue;
            int seq_len;
            len_bytes = asn1_read_length(rsa_pub + 1, rsa_pub_len - 1, &seq_len);
            if (len_bytes < 0) continue;
            const unsigned char *seq_data = rsa_pub + 1 + len_bytes;
            int seq_remaining = seq_len;
            if (seq_remaining < 1 || seq_data[0] != 0x02) continue;
            int mod_len_val;
            const unsigned char *mod = asn1_read_integer(seq_data, seq_remaining, &mod_len_val);
            if (!mod) continue;
            *n = mod;
            *n_len = mod_len_val;
            int mod_total = (seq_data + 1 + len_bytes + mod_len_val) - seq_data;
            seq_data += mod_total;
            seq_remaining -= mod_total;
            if (seq_remaining < 1 || seq_data[0] != 0x02) continue;
            int exp_len_val;
            const unsigned char *exp = asn1_read_integer(seq_data, seq_remaining, &exp_len_val);
            if (!exp) continue;
            *e = exp;
            *e_len = exp_len_val;
            return 0;
        }
    }
    return -1;
}


int parse_x509_certificate(const unsigned char *der, int der_len,
                           const unsigned char **tbs, int *tbs_len,
                           const unsigned char **sig, int *sig_len,
                           const unsigned char **n, int *n_len,
                           const unsigned char **e, int *e_len)
{
    int cert_len;
    const unsigned char *cert_val = asn1_enter_sequence(der, der_len, &cert_len);
    if (!cert_val) {
        return -1;
    }
    const unsigned char *p = cert_val;
    int remaining = cert_len;

    // tbsCertificate
    if (remaining < 1 || p[0] != 0x30) {
        return -1;
    }
    int tbs_value_len, len_bytes;
    len_bytes = asn1_read_length(p + 1, remaining - 1, &tbs_value_len);
    if (len_bytes < 0) return -1;
    *tbs = p;
    *tbs_len = 1 + len_bytes + tbs_value_len;
    p += *tbs_len;
    remaining -= *tbs_len;

    // signatureAlgorithm
    if (remaining < 1 || p[0] != 0x30) {
        return -1;
    }
    const unsigned char *next = asn1_skip_tlv(p, remaining);
    if (!next) return -1;
    int consumed = next - p;
    p += consumed;
    remaining -= consumed;

    // signatureValue
    if (remaining < 1 || p[0] != 0x03) {
        return -1;
    }
    int bit_len;
    len_bytes = asn1_read_length(p + 1, remaining - 1, &bit_len);
    if (len_bytes < 0) return -1;
    if (bit_len < 1) return -1;
    const unsigned char *bit_data = p + 1 + len_bytes;
    *sig = bit_data + 1;
    *sig_len = bit_len - 1;

    if (find_rsa_pubkey_in_tbs(*tbs, *tbs_len, n, n_len, e, e_len) != 0) {
        return -1;
    }
    return 0;
}




typedef struct {
    const uint8_t *der_ptr;
    uint32_t       der_len;
} der_buf_ta;

int base64_decode_char(unsigned char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    if (c == '=') return 0;  // padding
    return -1;
}

int pem_to_der(const unsigned char *pem, size_t pem_len,
                     unsigned char *der, size_t *der_len) {

   print_buf_U8(pem, pem_len, "pemdata :");

    const char begin_marker[] = "-----BEGIN CERTIFICATE-----";
    const char end_marker[] = "-----END CERTIFICATE-----";
    const unsigned char *begin = NULL;
    for (size_t i = 0; i < pem_len - sizeof(begin_marker); i++) {
        if (memcmp(&pem[i], begin_marker, sizeof(begin_marker)-1) == 0) {
            begin = &pem[i + sizeof(begin_marker)-1];
            break;
        }
    }
    if (!begin) return -1;

    const unsigned char *end = NULL;

    end = &pem[pem_len - sizeof(end_marker) + 1];

    if (!end) return -2;
    print_buf_U8(begin, sizeof(begin_marker), "base64_idx :");
    print_buf_U8(end, sizeof(end_marker), "base64_idx :");
    base64_decode(begin+1, pem_len - sizeof(begin_marker) - sizeof(end_marker), der, der_len);

    return 0;
}

/*****************************CERT INFO*********************************************/
#define CERT_BEGIN_LEN 27

unsigned char cert_begin[CERT_BEGIN_LEN] = {
    0x2D, 0x2D, 0x2D, 0x2D, 0x2D, 0x42, 0x45, 0x47,
    0x49, 0x4E, 0x20, 0x43, 0x45, 0x52, 0x54, 0x49,
    0x46, 0x49, 0x43, 0x41, 0x54, 0x45, 0x2D, 0x2D,
    0x2D, 0x2D, 0x2D
};


#define CERT_END_LEN 25
unsigned char cert_end[CERT_END_LEN] = {
    0x2D, 0x2D, 0x2D, 0x2D, 0x2D,
    0x45, 0x4E, 0x44, 0x20, 0x43,
    0x45, 0x52, 0x54, 0x49, 0x46,
    0x49, 0x43, 0x41, 0x54, 0x45,
    0x2D, 0x2D, 0x2D, 0x2D, 0x2D
};


#define TAGE_MAGIC_LEN 12
unsigned char tag_magic[TAGE_MAGIC_LEN] = {
    0x00, 0x00, 0x00, 0x00, 0x4D, 0x49, 0xEF, 0x54,
    0x46, 0x4F, 0x54, 0x41
};



unsigned char signature_last[256] = {0};

int extract_certs(const unsigned char *input, int input_len, unsigned char *certs, int *cert_size)
{
    int cert_count = 0;
    int pos = 0;

    // Search until end of buffer or max certificate count reached
    while (pos <= input_len - CERT_BEGIN_LEN && cert_count < 1)
    {
        // Match certificate starting pattern
        if (memcmp(input + pos, cert_begin, CERT_BEGIN_LEN) == 0)
        {
            int start_pos = pos;
            pos += CERT_BEGIN_LEN;

            // Search for certificate ending pattern
            while (pos <= input_len - CERT_END_LEN)
            {
                if (memcmp(input + pos, cert_end, CERT_END_LEN) == 0)
                {
                    int end_pos = pos + CERT_END_LEN;
                    int cert_len = end_pos - start_pos;

                    // Copy certificate if length is valid
                    if (cert_len > 0 && cert_len < MAX_CERT_SIZE)
                    {
                        tmemcpy((void*)certs, (const void*)(input + start_pos), cert_len);
                        *cert_size = cert_len;
                        print_buf_U8((unsigned char*)&cert_len, 4, "cert_len :");
                        cert_count++;
                    }

                    pos = end_pos;

                    int end_addr = (int)(input + end_pos);
                    int start_addr = (int)(input + start_pos);

                    print_buf_U8((unsigned char*)&start_addr, 4, "start_addr :");
                    print_buf_U8((unsigned char*)&end_addr, 4, "end_addr :");
                    break;
                }
                pos++;
            }
        }
        else
        {
            pos++;
        }
    }

    return cert_count;
}


int get_tage_info(unsigned char *tage_addr, unsigned int *cert_size, unsigned int *tage_size)
{
    if(memcmp(tag_magic, tage_addr, TAGE_MAGIC_LEN) != 0)
    {
            print_buf_U8(tage_addr, 12, "tage_addr :");
            print_buf_U8(tag_magic, 12, "tag_magic :");
            delay_ms(2);
            return -6;
    }

    if(memcmp(tag_magic, tage_addr, TAGE_MAGIC_LEN) != 0)
    {
       return -6;
    }
   uint32_t cert_data_len = (tage_addr[24]<<8)|(tage_addr[25] + 256);//Cert Size + 256
   uint32_t tage_data_len = (tage_addr[16]<<8)|tage_addr[17];
   print_buf_U8((unsigned char*)&cert_data_len, 4, "Secure_data_len :");
   *cert_size = cert_data_len;
   *tage_size = tage_data_len;
   return 0;

}



int read_cert_info(unsigned char *info_addr, unsigned int info_len, Cert_Info *cert_info, unsigned int *cert_len)
{

    if((info_addr == NULL) || (cert_info == NULL) || (cert_len == NULL) || (info_len == 0))
    {
       return -8;// input invalid
    }

   print_buf_U8((unsigned char*)&info_len, 4 , "info_len :");

   printf("*********************************File Info Output!!!***********************************\n");
   // cert_a
   uint8_t cert_pem_buff[2048] = {0};
   uint32_t cert_pem_len = 0;
   //get cert pem data
   uint8_t cert_valid = extract_certs(info_addr, info_len, cert_pem_buff, (int*)&cert_pem_len);// next cert addr = tage_addr + tage_size + cert0_size;

   if((cert_pem_len ==0) || (cert_valid == 0))
   {
      return -32;
   }

   *cert_len = cert_pem_len;

   //get cert der data
   int result = pem_to_der(cert_pem_buff, cert_pem_len, cert_info->cert_info.der_data, &cert_info->cert_info.len);

   if(result)
   {
      return -33;
   }

   print_buf_U8((unsigned char*)&cert_info->cert_info.len, 4 , "&cert_info->cert_info.len :");
   print_buf_U8((unsigned char*)cert_info->cert_info.der_data, cert_info->cert_info.len , "cert_info->cert_info.der_data:");

   cert_info->cert_info.end_addr = (unsigned int)(info_addr + cert_pem_len);//
    const unsigned char *tbs = NULL;
    const unsigned char *sig = NULL;
    const unsigned char *n = NULL;
    const unsigned char *e = NULL;

    int sig_len, n_len, e_len;
    int ret = parse_x509_certificate(cert_info->cert_info.der_data, cert_info->cert_info.len,
                                     &tbs, (int*)&cert_info->cert_info.tbs_length,
                                     &sig, &sig_len,
                                     &n, &n_len,
                                     &e, &e_len);
   if(ret)
   {
      return -34;
   }
//    unsigned int tbs_addr, sig_addr, n_addr;

    cert_info->cert_info.tbs_addr = tbs - cert_info->cert_info.der_data;
    cert_info->cert_info.sign_addr = sig - cert_info->cert_info.der_data;
    cert_info->cert_info.pub_key_n_addr = n - cert_info->cert_info.der_data + 1; //n len 257

//    memcpy(cert_info->cert_info.local_pub_key_n, &n[1], 256);
//    print_buf_U8(&cert_info->cert_info.der_data[cert_info->cert_info.tbs_addr], cert_info->cert_info.tbs_length, "tbs_info");
//    print_buf_U8(tbs, tbs_len, "tbs_der");
//    print_buf_U8(&cert_info->cert_info.der_data[cert_info->cert_info.sign_addr], 256, "sign_info");
//     print_buf_U8(sig, sig_len, "signature");
//    print_buf_U8(cert_info->cert_info.local_pub_key_n, 256, "cert_info->cert_info.local_pub_key_n");
//    print_buf_U8(&cert_info->cert_info.der_data[cert_info->cert_info.pub_key_n_addr], 256, "rsa_modulus");


   return 0;
}


