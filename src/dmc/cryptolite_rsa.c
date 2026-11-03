/******************************************************************************
* File Name: cryptolite_rsa.c
*
* Description:  This file provides wrapper function to verify RSA signature.
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2024-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
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
