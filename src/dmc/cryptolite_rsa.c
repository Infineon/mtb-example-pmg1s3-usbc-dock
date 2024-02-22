/******************************************************************************
* File Name: cryptolite_rsa.c
*
* Description:  This file provides wrapper function to verify RSA signature.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_cryptolite.h"
#include "cryptolite_rsa.h"


/** This macro converts byte count to bit count*/
#define CRYPTO_MAKE_BYTES_TO_BITS(bytes)            ((bytes) * 8UL)

/** RSA2048 signature length. */
#define CRYPTO_RSA_SIGN_LEN_BITS                    (2048UL)

/* RSA public key exponent length in bits. */
#define CRYPTO_RSA_PUB_KEY_EXP_LENGTH                (17U)


CY_ALIGN(4) uint8_t gl_rsaSignature[VU_BITS_TO_BYTES(CRYPTO_RSA_SIGN_LEN_BITS)];

CY_ALIGN(4) uint8_t gl_barrettCoeff[VU_BITS_TO_BYTES_WORD_ALIGN(CRYPTO_RSA_SIGN_LEN_BITS+1)];

CY_ALIGN(4) uint8_t gl_montRInvModulus[VU_BITS_TO_BYTES_WORD_ALIGN(CRYPTO_RSA_SIGN_LEN_BITS+1)];

CY_ALIGN(4) uint8_t gl_montModDer[VU_BITS_TO_BYTES_WORD_ALIGN(CRYPTO_RSA_SIGN_LEN_BITS)];

CY_ALIGN(4) uint8_t gl_processedShaDigest[VU_BITS_TO_BYTES_WORD_ALIGN(CRYPTO_RSA_SIGN_LEN_BITS)];

CY_ALIGN(4) uint8_t gl_publicKeyExp[3] = {0x01, 0x00, 0x01};

CY_ALIGN(4) uint8_t gl_internalBuffer[  4*VU_BITS_TO_WORDS(2*CRYPTO_RSA_SIGN_LEN_BITS+1)
                                      + 4*VU_BITS_TO_WORDS(2*CRYPTO_RSA_SIGN_LEN_BITS+1)
                                      + 4*VU_BITS_TO_WORDS(CRYPTO_RSA_SIGN_LEN_BITS)];


/*
 * This function verifies if the given signature matches with the SHA256 hash.
 */
bool rsa_verify_signature (uint8_t* hash, uint16_t hash_len,
                           uint8_t* signature, uint16_t sign_len,
                           uint8_t* public_key, uint16_t key_len)
{
    cy_en_cryptolite_status_t cryptoStatus;
    cy_stc_cryptolite_rsa_pub_key_t keyInfo;
    uint32_t i;

    /* Convert RSA signature from big endian to little endian. */
    for(i = 0; i < sign_len; i++)
    {
        gl_rsaSignature[i] = signature[sign_len - 1 - i];
    }

    /* Public key info */
    keyInfo.moduloPtr        = (uint8_t *)public_key;
    keyInfo.moduloLength     = CRYPTO_MAKE_BYTES_TO_BITS(key_len);
    keyInfo.barretCoefPtr    = (uint8_t *)gl_barrettCoeff;
    keyInfo.inverseModuloPtr = (uint8_t *)gl_montRInvModulus;
    keyInfo.rBarPtr          = (uint8_t *)gl_montModDer;
    keyInfo.privateBuffer    = (uint8_t *)gl_internalBuffer;
    keyInfo.pubExpPtr        = (uint8_t *)gl_publicKeyExp;
    keyInfo.pubExpLength     = CRYPTO_RSA_PUB_KEY_EXP_LENGTH;
    keyInfo.calculateCoeff   = true;

    /* Verify signature */
    cryptoStatus = Cy_Cryptolite_Rsa_Verify(CRYPTOLITE,
                                hash_len,
                                hash,
                                gl_rsaSignature,
                                sign_len,
                                &keyInfo,
                                gl_processedShaDigest);

    return (CY_CRYPTOLITE_SUCCESS == cryptoStatus ? true : false);
}

/* [] END OF FILE */
