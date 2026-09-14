/********************************************************************************************************
 * @file    tlk_hsm_api.h *
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

#ifndef TLK_HSM_API_H_
#define TLK_HSM_API_H_
#include "hsm_type.h"
#include "hsm_agl.h"



/**
 * @brief Verifies the ECC-256 digital signature of a hash value using the corresponding public key.
 *
 * This function is used to perform ECC (Elliptic Curve Cryptography) 256-bit signature verification
 * through the HSM (Hardware Security Module). It validates whether the given digital signature
 * is legitimate by comparing it with the hash value of the original data and the provided public key.
 * The verification result indicates whether the signature is valid, which helps ensure data integrity
 * and non-repudiation.
 *
 * @param pubKey Pointer to the ECC-256 public key (uncompressed format, typically 64 bytes: 32 bytes for x-coordinate + 32 bytes for y-coordinate).
 * @param hash Pointer to the hash value of the data to be verified (e.g., SHA-256 hash, consistent with the hash algorithm used during signature generation).
 * @param hash_len Length of the hash value in bytes (e.g., 32 bytes for SHA-256).
 * @param signature Pointer to the ECC-256 digital signature (ASN.1 DER format or raw format, depending on HSM configuration).
 * @param signature_len Length of the digital signature in bytes.
 *
 * @return tlk_hsm_status_t PSA status code indicating the result of the verification:
 *         - PSA_SUCCESS: Signature verification succeeded, the signature is valid.
 *         - PSA_ERROR_INVALID_ARGUMENT: One or more input parameters are invalid (e.g., NULL pointer, invalid length).
 *         - PSA_ERROR_INVALID_SIGNATURE: Signature verification failed, the signature is invalid or does not match the public key/hash.
 *         - PSA_ERROR_HARDWARE_FAILURE: HSM hardware error occurred during verification.
 *         - Other PSA error codes: Corresponding error occurred during the verification process.
 */
tlk_hsm_status_t HSM_ECC256_Verify_Data(uint8_t *pubKey, uint8_t *hash, size_t hash_len, uint8_t *signature, size_t signature_len);

/**
 * @brief Generates the ECC-256 digital signature for the given data hash using the corresponding private key via HSM.
 *
 * This function leverages the Hardware Security Module (HSM) to perform ECC (Elliptic Curve Cryptography) 256-bit digital signature generation.
 * It uses the provided ECC-256 private key to sign the pre-computed hash value of the target data, generating a digital signature that can be used
 * for subsequent verification. The generated signature ensures the integrity of the data and the non-repudiation of the signer.
 * Note: The input hash value should be consistent with the hash algorithm agreed upon for signature verification (e.g., SHA-256).
 *
 * @param data Pointer to the original data to be signed (used for contextual reference; the actual signature is generated based on the hash value).
 * @param prikey Pointer to the ECC-256 private key (typically 32 bytes in length, used to generate the digital signature; must be kept confidential).
 * @param hash Pointer to the pre-computed hash value of the original data (the hash result of the data parameter, e.g., SHA-256 hash).
 * @param hash_len Length of the hash value in bytes (e.g., 32 bytes for SHA-256; must match the hash algorithm used).
 * @param signature Pointer to the buffer used to store the generated ECC-256 digital signature (output parameter; format depends on HSM configuration, e.g., ASN.1 DER or raw format).
 * @param signature_len Length of the signature buffer in bytes (specifies the maximum storage capacity; the actual signature length may be less than this value).
 *
 * @return tlk_hsm_status_t PSA status code indicating the result of the signature generation:
 *         - PSA_SUCCESS: ECC-256 digital signature generated successfully and stored in the signature buffer.
 *         - PSA_ERROR_INVALID_ARGUMENT: One or more input parameters are invalid (e.g., NULL pointer, invalid hash/signature length, invalid private key format).
 *         - PSA_ERROR_INVALID_PRIVATE_KEY: The provided private key is invalid (e.g., incorrect length, invalid value for ECC-256 curve).
 *         - PSA_ERROR_HARDWARE_FAILURE: HSM hardware error occurred during signature generation.
 *         - PSA_ERROR_BUFFER_TOO_SMALL: The signature buffer (signature) is too small to store the generated signature.
 *         - Other PSA error codes: Corresponding error occurred during the signature generation process.
 */
tlk_hsm_status_t HSM_ECC256_Sign_Data(uint8_t *data, uint8_t *prikey, uint8_t *hash, size_t hash_len, uint8_t *signature, size_t signature_len);

/**
 * @brief Computes the SHA-256 hash value of the input message using the Hardware Security Module (HSM).
 *
 * This function utilizes the HSM to perform SHA-256 (Secure Hash Algorithm 256-bit) hash computation,
 * which generates a fixed 32-byte hash value from the input message. SHA-256 is a cryptographic hash function
 * that produces a unique, fixed-size digest for any input data, ensuring data integrity (any change in the input
 * message will result in a significantly different hash value). Using HSM enhances security by keeping the
 * hash computation process and related operations within a secure hardware environment.
 *
 * @param msg Pointer to the input message (data) to be hashed. This can be any arbitrary binary data.
 * @param msg_bytes Length of the input message in bytes, specified as an unsigned integer.
 * @param digest Pointer to the output buffer used to store the computed SHA-256 hash value.
 *               This buffer must be at least 32 bytes in size to accommodate the fixed-length SHA-256 digest.
 *
 * @return tlk_hsm_status_t PSA status code indicating the result of the hash computation:
 *         - PSA_SUCCESS: SHA-256 hash computed successfully, and the result is stored in the digest buffer.
 *         - PSA_ERROR_INVALID_ARGUMENT: One or more input parameters are invalid (e.g., NULL pointer for msg or digest, msg_bytes equals 0).
 *         - PSA_ERROR_HARDWARE_FAILURE: HSM hardware error occurred during the hash computation process.
 *         - PSA_ERROR_BUFFER_TOO_SMALL: The digest buffer is too small to store the 32-byte SHA-256 hash value.
 *         - Other PSA error codes: Corresponding error occurred during the hash computation.
 */
tlk_hsm_status_t HSM_Sha256(unsigned char *msg, unsigned int msg_bytes, unsigned char *digest);

/**
 * @brief Initializes the Hardware Security Module (HSM) to a ready state for subsequent cryptographic operations.
 *
 * This function performs the necessary initialization procedures for the HSM, including but not limited to
 * power-on self-test (POST), hardware reset, initialization of internal security registers, configuration of
 * cryptographic algorithms (e.g., SHA-256, ECC-256), and validation of HSM hardware integrity.
 * It must be called once before any other HSM-related functions (such as HSM_Sha256, HSM_ECC256_Sign_Data, etc.)
 * to ensure the HSM operates correctly and securely.
 *
 * @note This function is idempotent in most cases, meaning multiple calls will not cause adverse effects,
 *       but it is recommended to call it only once at the system startup phase to avoid unnecessary overhead.
 *
 * @return None. This function has no return value as it primarily performs initialization operations.
 *         If an initialization failure occurs (e.g., HSM hardware error), it may trigger a system-level alert
 *         or fault handling mechanism (depending on the overall system design).
 */
void HSM_INIT(void);
tlk_hsm_status_t HSM_Secu_Sign_Verify(unsigned int bin_addr, unsigned int bin_size, unsigned int ota_size, unsigned int tage_addr, unsigned int root_key_addr,uint8_t secu_sign_vrfy_en);
#endif
