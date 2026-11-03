/******************************************************************************
* File Name: cryptolite_rsa.h
*
* Description: This file defines functions prototype for RSA signature verification.
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

#ifndef _CRYPTOLITE_RSA_H_
#define _CRYPTOLITE_RSA_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Function verifies the RSA signature using provide public key.
 *
 * @param hash - Pointer to the hash buffer
 * @param hash_len - Hash length in bytes.
 * @param signature - Pointer to the RSA signature.
 * @param sign_len -  RSA signature length in bytes.
 * @param public_key - Pointer to the public key buffer.
 * @param key_len - Public key length in bytes.
 *
 * @return true if success otherwise false.
 */
bool rsa_verify_signature (uint8_t* hash, uint16_t hash_len, uint8_t* signature,
                           uint16_t sign_len, uint8_t* public_key, uint16_t key_len);

#endif /* _CRYPTOLITE_RSA_H_ */

/* [] END OF FILE */
